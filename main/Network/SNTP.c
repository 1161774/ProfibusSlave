/**
 * SNTP.c
 * SNTP time synchronisation.
 *
 * Polling interval
 * ================
 * The ESP32 crystal oscillator drifts ~25–50 ppm.  The table below shows
 * how that drift accumulates between syncs and how long smooth mode takes
 * to correct it (correction rate: ±1 ms/s):
 *
 *   Interval     Worst-case drift    Smooth correction time
 *   ──────────   ────────────────    ─────────────────────
 *    5 min            15 ms          ~15 s      (imperceptible)
 *    1 hour  (◀)     180 ms          ~3 min     (imperceptible)
 *    6 hours        1.08 s          ~18 min     (visible in logs)
 *   24 hours         4.3 s          ~72 min     (degraded accuracy)
 *
 * One hour (CONFIG_SNTP_SYNC_INTERVAL_S default = 3600) is the LWIP default
 * and the recommended setting:
 *   • Drift stays well below 200 ms — imperceptible for logging purposes.
 *   • Smooth correction converges in ~3 minutes — no visible log jumps.
 *   • Each sync is a single UDP round-trip; one per hour is negligible load.
 *   • Shorter intervals waste network traffic with diminishing accuracy gain.
 *   • Longer intervals risk multi-second drift and slow smooth corrections.
 *
 * ⚠ Minimum bound: CONFIG_LWIP_SNTP_UPDATE_DELAY (default 3 600 000 ms).
 *   esp_sntp_set_sync_interval() silently clamps to this value if the
 *   configured interval is shorter.  To use a shorter interval, reduce
 *   CONFIG_LWIP_SNTP_UPDATE_DELAY via menuconfig:
 *     Component config → LWIP → SNTP → Request interval to update time
 *
 * Smooth sync mode
 * ================
 * SNTP_SYNC_MODE_SMOOTH slews the system clock at ±1 ms/s rather than
 * jumping it to the corrected value.  This prevents discontinuous timestamps
 * in log output.  At a 1-hour sync interval the correction is always ≤180 ms
 * and converges within ~3 minutes.
 *
 * First sync
 * ==========
 * SNTP_WaitSync() blocks for up to CONFIG_SNTP_STARTUP_TIMEOUT_MS
 * (default 10 000 ms) after startup.  The LWIP stack adds a random jitter
 * of up to CONFIG_LWIP_SNTP_MAXIMUM_STARTUP_DELAY (default 5 000 ms) before
 * sending the first request, so the timeout must comfortably exceed this.
 * If the timeout expires, the function returns false and startup continues.
 */

#include "SNTP.h"
#include "Logging/pb_log.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <time.h>

/* Set by time_sync_cb on the first successful NTP response */
#define SNTP_SYNCED_BIT     BIT0

static EventGroupHandle_t s_sntp_events;

/* ------------------------------------------------------------------ */
/* Sync notification callback                                         */
/* ------------------------------------------------------------------ */

static void time_sync_cb(struct timeval *tv)
{
    (void)tv;   /* timeval is available but we re-read via time() for clarity */

    char tbuf[26];
    time_t now = time(NULL);
    struct tm tm_info;
    gmtime_r(&now, &tm_info);
    strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_info);
    ESP_LOGI(TAG_SNTP, "Synchronised: %s UTC", tbuf);

    /* Signal SNTP_WaitSync() on the first sync; stays set for subsequent ones */
    xEventGroupSetBits(s_sntp_events, SNTP_SYNCED_BIT);
}

/* ------------------------------------------------------------------ */
/* SNTP_Init                                                          */
/* ------------------------------------------------------------------ */

void SNTP_Init(void)
{
    s_sntp_events = xEventGroupCreate();

    /*
     * Smooth sync: adjusts the clock at ±1 ms/s instead of jumping.
     * Must be configured before esp_sntp_init().
     */
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_SNTP_SERVER);

    /*
     * Set the polling interval from Kconfig.  See "Polling interval"
     * notes at the top of this file for the rationale and the minimum
     * bound imposed by CONFIG_LWIP_SNTP_UPDATE_DELAY.
     */
    esp_sntp_set_sync_interval((uint32_t)CONFIG_SNTP_SYNC_INTERVAL_S * 1000U);

    sntp_set_time_sync_notification_cb(time_sync_cb);

    esp_sntp_init();

    ESP_LOGI(TAG_SNTP, "Started — server=%s  interval=%d s",
             CONFIG_SNTP_SERVER, CONFIG_SNTP_SYNC_INTERVAL_S);
}

/* ------------------------------------------------------------------ */
/* SNTP_WaitSync                                                      */
/* ------------------------------------------------------------------ */

bool SNTP_WaitSync(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_sntp_events,
        SNTP_SYNCED_BIT,
        pdFALSE,            /* do not clear on exit — stays set for callers after startup */
        pdTRUE,             /* wait for all specified bits                                */
        pdMS_TO_TICKS(timeout_ms));

    if (!(bits & SNTP_SYNCED_BIT)) {
        ESP_LOGW(TAG_SNTP,
                 "Initial sync not complete after %lu ms — continuing without time",
                 (unsigned long)timeout_ms);
        return false;
    }
    return true;
}
