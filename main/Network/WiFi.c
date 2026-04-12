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
            ESP_LOGW(TAG_WIFI, "Disconnected (reason %d) — retry in %d ms",
                     d->reason, WIFI_RECONNECT_DELAY_MS);
            xTimerStart(s_reconnect_timer, 0);
            break;
        }

        default:
            break;
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
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
