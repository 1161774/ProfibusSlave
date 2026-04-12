/**
 * WiFi.c
 * WiFi station — connects to the configured AP and reconnects automatically
 * whenever the link drops.
 *
 * Architecture
 * ============
 * The ESP-IDF WiFi driver manages its own internal tasks.  All application
 * interaction happens through the default system event loop:
 *
 *   WIFI_EVENT_STA_START        → call esp_wifi_connect()
 *   WIFI_EVENT_STA_CONNECTED    → link established; SNTP can reach the server
 *   WIFI_EVENT_STA_DISCONNECTED → schedule a delayed reconnect via FreeRTOS timer
 *   IP_EVENT_STA_GOT_IP         → DHCP complete; log the assigned address
 *
 * WiFi_Init() returns as soon as esp_wifi_start() is called.  The actual
 * association and DHCP lease complete asynchronously, so the PROFIBUS stack
 * can start processing frames while the network link is being established.
 *
 * Reconnect delay
 * ===============
 * Reconnecting immediately on every disconnect causes log spam when the AP
 * is unreachable (e.g. reason 201 = NO_AP_FOUND).  A one-shot FreeRTOS
 * software timer fires WIFI_RECONNECT_DELAY_MS after each disconnect.
 * The timer is safe to start from the event handler (ISR-safe variant is not
 * needed because the system event loop runs in a task context).
 *
 * NVS
 * ===
 * The WiFi driver stores radio calibration data in NVS (Non-Volatile
 * Storage), so nvs_flash_init() must be called before esp_wifi_init().
 * If the NVS partition is full or its version has changed (common after a
 * reflash), we erase and reinitialise — calibration data is discarded but
 * will be regenerated on the next run.
 */

#include "WiFi.h"
#include "Logging/pb_log.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <string.h>

/* Delay between a disconnect and the next esp_wifi_connect() attempt */
#define WIFI_RECONNECT_DELAY_MS     5000

static TimerHandle_t s_reconnect_timer;
static wifi_ip_cb_t  s_on_ip_acquired;

/* ------------------------------------------------------------------ */
/* Reconnect timer callback                                           */
/* ------------------------------------------------------------------ */

static const char *authmode_str(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN:            return "Open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3-SAE";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "unknown";
    }
}

static const char *cipher_str(wifi_cipher_type_t c)
{
    switch (c) {
    case WIFI_CIPHER_TYPE_NONE:     return "none";
    case WIFI_CIPHER_TYPE_WEP40:    return "WEP40";
    case WIFI_CIPHER_TYPE_WEP104:   return "WEP104";
    case WIFI_CIPHER_TYPE_TKIP:     return "TKIP";
    case WIFI_CIPHER_TYPE_CCMP:     return "CCMP";
    case WIFI_CIPHER_TYPE_TKIP_CCMP:return "TKIP+CCMP";
    case WIFI_CIPHER_TYPE_GCMP:     return "GCMP";
    case WIFI_CIPHER_TYPE_GCMP256:  return "GCMP-256";
    default:                        return "unknown";
    }
}

static void log_ap_info(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return;

    const char *bw = (ap.second == WIFI_SECOND_CHAN_NONE) ? "20 MHz" : "40 MHz";
    const char *phys =
        ap.phy_11ax ? "ax" :
        ap.phy_11n  ? "n"  :
        ap.phy_11g  ? "g"  : "b";

    ESP_LOGD(TAG_WIFI,
             "AP:   BSSID=%02x:%02x:%02x:%02x:%02x:%02x  Ch=%d (%s)  "
             "RSSI=%d dBm  Auth=%s  Cipher=%s/%s  PHY=802.11%s",
             ap.bssid[0], ap.bssid[1], ap.bssid[2],
             ap.bssid[3], ap.bssid[4], ap.bssid[5],
             ap.primary, bw, ap.rssi,
             authmode_str(ap.authmode),
             cipher_str(ap.pairwise_cipher), cipher_str(ap.group_cipher),
             phys);
}

static const char *wifi_reason_str(uint8_t reason)
{
    switch (reason) {
    case 1:   return "UNSPECIFIED";
    case 2:   return "AUTH_EXPIRE (auth handshake timed out)";
    case 3:   return "AUTH_LEAVE (AP deauthenticated us)";
    case 4:   return "ASSOC_EXPIRE";
    case 8:   return "ASSOC_LEAVE (clean disconnect)";
    case 14:  return "MIC_FAILURE (TKIP integrity check failed)";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT (wrong password?)";
    case 200: return "BEACON_TIMEOUT (AP stopped responding)";
    case 201: return "NO_AP_FOUND (SSID not visible / wrong band)";
    case 202: return "AUTH_FAIL (wrong password / auth rejected)";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    case 206: return "AP_TSF_RESET";
    case 207: return "ROAMING";
    case 210: return "NO_AP_FOUND_W_COMPATIBLE_SECURITY (authmode mismatch)";
    case 211: return "NO_AP_FOUND_IN_AUTHMODE_THRESHOLD";
    case 212: return "NO_AP_FOUND_IN_RSSI_THRESHOLD (signal too weak)";
    default:  return "unknown";
    }
}

static void reconnect_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    ESP_LOGI(TAG_WIFI, "Retrying connection to \"%s\"", CONFIG_WIFI_SSID);
    esp_wifi_connect();
}

/* ------------------------------------------------------------------ */
/* Event handler                                                      */
/* ------------------------------------------------------------------ */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {

        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG_WIFI, "Station started — connecting to \"%s\"",
                     CONFIG_WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_WIFI, "Associated with AP");
            log_ap_info();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d =
                (wifi_event_sta_disconnected_t *)event_data;
            /*
             * Common reason codes:
             *   201 WIFI_REASON_NO_AP_FOUND   — SSID not visible
             *   202 WIFI_REASON_AUTH_FAIL      — wrong password
             *   8   WIFI_REASON_ASSOC_LEAVE    — we asked to disconnect
             */
            ESP_LOGW(TAG_WIFI, "Disconnected: %s — retry in %d ms",
                     wifi_reason_str(d->reason), WIFI_RECONNECT_DELAY_MS);
            xTimerStart(s_reconnect_timer, 0);
            break;
        }

        default:
            break;
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /*
         * Cancel any pending reconnect timer.  A DISCONNECTED event can fire
         * during the auth handshake before the connection ultimately succeeds;
         * if that timer were allowed to fire it would call esp_wifi_connect()
         * while already connected, triggering an immediate disconnect.
         */
        xTimerStop(s_reconnect_timer, 0);

        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGD(TAG_WIFI, "IP:   addr=" IPSTR "  mask=" IPSTR "  gw=" IPSTR,
                 IP2STR(&ev->ip_info.ip),
                 IP2STR(&ev->ip_info.netmask),
                 IP2STR(&ev->ip_info.gw));
        log_ap_info();
        if (s_on_ip_acquired) {
            s_on_ip_acquired();
        }
    }
}

/* ------------------------------------------------------------------ */
/* WiFi_Init                                                          */
/* ------------------------------------------------------------------ */

void WiFi_Init(wifi_ip_cb_t on_ip_acquired)
{
    s_on_ip_acquired = on_ip_acquired;
    /* NVS is required by the WiFi driver for calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG_WIFI, "NVS partition invalid — erasing and reinitialising");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    s_reconnect_timer = xTimerCreate("wifi_reconnect",
                                     pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS),
                                     pdFALSE,       /* one-shot */
                                     NULL,
                                     reconnect_timer_cb);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register for all WiFi events and for the IP-obtained event */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Connection completes asynchronously — see wifi_event_handler above */
}
