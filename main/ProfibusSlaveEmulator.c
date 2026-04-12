/**
 * ProfibusSlaveEmulator.c
 * Application entry point for the PROFIBUS slave emulator.
 *
 * Startup sequence:
 *   1. Configure logging levels (before anything prints).
 *   2. Initialise the PROFIBUS controller (routing table).
 *   3. Create application task(s) — each registers its slave with AddSlave().
 *   4. Wait briefly for tasks to register before starting Serial.
 *   5. Initialise Serial (UART + RX/TX tasks).
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "Profibus/Controller.h"
#include "Serial/Serial.h"
#include "Logging/pb_log.h"
#include "Application/ET200S.h"
#include "Application/VSD.h"

/* ================================================================== */
/* Logging configuration                                              */
/* ================================================================== */

/**
 * Configure_Logging()
 * ───────────────────
 * Central place to set per-module log verbosity.
 * Call this first in app_main, before any other initialisation.
 *
 * Available levels (ascending verbosity):
 *   ESP_LOG_NONE    — silent
 *   ESP_LOG_ERROR   — errors only
 *   ESP_LOG_WARN    — warnings + errors
 *   ESP_LOG_INFO    — normal operational messages       ← recommended default
 *   ESP_LOG_DEBUG   — detailed protocol trace
 *   ESP_LOG_VERBOSE — everything
 *
 * ⚠ TIMING WARNING ⚠
 * At 115200 baud each log call costs ~2–9 ms (synchronous UART write).
 * DEBUG calls exist in the RX→TX hot path (SAP dispatch, diag detail).
 * Setting TAG_PROTOCOL to DEBUG will reintroduce timing violations.
 *
 * Recommended fix if you need protocol debug detail:
 *   • Increase console baud to 921600 in menuconfig
 *     (Component config → ESP System Settings → UART console baud rate)
 *   • Your terminal must match — then each log call costs ~0.7 ms.
 *
 * Log tags (defined in Logging/pb_log.h):
 *   TAG_SERIAL      "Serial"        — RX / TX hex frame dumps
 *   TAG_CONTROLLER  "PBController"  — frame routing, byte counts
 *   TAG_PROTOCOL    "PBProtocol"    — SAP dispatch, state machine
 *   TAG_ET200S      "ET200S"        — ET200S application layer
 *   TAG_VSD         "VSD"           — KFC750 VSD application layer
 *   TAG_EMULATOR    "PBEmulator"    — startup / heartbeat
 */
static void Configure_Logging(void)
{
    /* Default: suppress everything not explicitly enabled below */
    esp_log_level_set("*",            ESP_LOG_WARN);

    /* Frame hex dumps — the most useful debug output */
    esp_log_level_set(TAG_RX,         ESP_LOG_INFO);   /* inbound frames  */
    esp_log_level_set(TAG_TX,         ESP_LOG_INFO);   /* outbound frames */

    /* Physical layer init messages and UART errors */
    esp_log_level_set(TAG_SERIAL,     ESP_LOG_INFO);

    /* Routing — "RX N bytes → TX M bytes" summary per frame */
    esp_log_level_set(TAG_CONTROLLER, ESP_LOG_WARN);   /* quiet by default; INFO for byte counts */

    /* Protocol state machine and SAP dispatch.
     * Switch to ESP_LOG_DEBUG for detailed trace (see timing warning above). */
    esp_log_level_set(TAG_PROTOCOL,   ESP_LOG_WARN);

    /* Application tasks */
    esp_log_level_set(TAG_ET200S,     ESP_LOG_INFO);
    esp_log_level_set(TAG_VSD,        ESP_LOG_INFO);

    /* Entry point / heartbeat */
    esp_log_level_set(TAG_EMULATOR,   ESP_LOG_INFO);
}

/* ================================================================== */
/* Slave instances                                                    */
/* ================================================================== */

static profibusSlave    et200s_slave;
static ET200S_Simulator et200s_sim = {
    .name            = "ET200S",
    .profibusAddress = 8,           /* Must match Siemens HW Config / Network view */
    .slave           = &et200s_slave,
    .inputs          = { .di_card1 = 0x00, .di_card2 = 0x00 },
    .mutex           = NULL,
};

/*
 * KFC750 VSD — disabled by default.
 * Uncomment the xTaskCreate line in app_main to enable at address 10.
 */
static profibusSlave slave_vsd;
static VSDSimulator  vsd_sim = {
    .vsdName         = "VSD",
    .vsdPriority     = 8,
    .profibusAddress = 10,
    .profibusSlave   = &slave_vsd,
    .speedSetpoint   = 0.0f,
    .currentSpeed    = 0.0f,
    .state           = STOPPED,
    .vsdMutex        = NULL,
};

/* ================================================================== */
/* app_main                                                           */
/* ================================================================== */

void app_main(void)
{
    Configure_Logging();

    ESP_LOGI(TAG_EMULATOR, "PROFIBUS Slave Emulator starting");

    InitialiseController();

    /* ---- ET200S at address 8 ---- */
    et200s_sim.mutex = xSemaphoreCreateMutex();
    xTaskCreate(ET200S_TaskEntry, "et200s", 4096, &et200s_sim,
                configMAX_PRIORITIES - 2, NULL);

    /* ---- VSD at address 10 ---- */
    vsd_sim.vsdMutex = xSemaphoreCreateMutex();
    xTaskCreate(taskEntry, vsd_sim.vsdName, 4096, &vsd_sim, vsd_sim.vsdPriority, NULL);

    /*
     * Give application tasks time to register their slaves before
     * the serial layer starts processing frames.
     */
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial_Init();

    ESP_LOGI(TAG_EMULATOR, "All tasks running — ET200S addr=8 ident=0x80E0");

    /* Heartbeat loop — slave is entirely event-driven from here */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG_EMULATOR,
                 "ET200S state=%d diag=%lu prm=%lu cfg=%lu dxchg=%lu "
                 "DI=[0x%02X 0x%02X]",
                 et200s_slave.State.ReadyState,
                 (unsigned long)et200s_slave.cnt_diag,
                 (unsigned long)et200s_slave.cnt_prm,
                 (unsigned long)et200s_slave.cnt_cfg,
                 (unsigned long)et200s_slave.cnt_data_exch,
                 et200s_slave.input.data[0],
                 et200s_slave.input.data[1]);
        ESP_LOGI(TAG_EMULATOR,
                 "VSD    state=%d diag=%lu prm=%lu cfg=%lu dxchg=%lu "
                 "speed=%.1f%%",
                 slave_vsd.State.ReadyState,
                 (unsigned long)slave_vsd.cnt_diag,
                 (unsigned long)slave_vsd.cnt_prm,
                 (unsigned long)slave_vsd.cnt_cfg,
                 (unsigned long)slave_vsd.cnt_data_exch,
                 vsd_sim.currentSpeed);
    }
}
