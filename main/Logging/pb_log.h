#ifndef PB_LOG_H
#define PB_LOG_H

/**
 * pb_log.h
 * Fast single-call hex frame logger for PROFIBUS debugging.
 *
 * WHY NOT ESP_LOG_BUFFER_HEXDUMP?
 * ================================
 * ESP_LOG_BUFFER_HEXDUMP calls esp_log_write() once per 16 bytes and
 * includes an address column plus an ASCII column — output like:
 *
 *   I (394) RX:
 *   0x000: 68 05 05 68  88 81 6d 3c  3e f0 16              h..h..m<>..
 *
 * At 115200 baud each esp_log_write() costs ~9 ms for 16 bytes.
 * For a 17-byte frame that is two calls = ~18 ms — dominating the
 * PROFIBUS response budget entirely.
 *
 * pb_log_hex() builds the complete hex string in a stack buffer first,
 * then makes a SINGLE esp_log_write() call — output like:
 *
 *   I (394) RX: 68 05 05 68 88 81 6d 3c 3e f0 16
 *
 * At 115200 baud this costs ~6 ms for the same frame.  At 921600 baud
 * (change CONFIG_ESP_CONSOLE_UART_BAUDRATE in sdkconfig) it drops to
 * ~0.7 ms — well within the PROFIBUS Tsdx window.
 *
 * For zero-latency logging, consider Segger RTT (requires JTAG probe).
 *
 * CONSOLE BAUD RATE
 * =================
 * The console UART baud rate is set in sdkconfig:
 *   CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600
 * Run "idf.py menuconfig" → Component config → ESP System Settings →
 * UART console baud rate.  Your terminal must match.
 */

#include <stdint.h>
#include "esp_log.h"

/* ------------------------------------------------------------------ */
/* Module log tags — single source of truth for esp_log_level_set()  */
/* ------------------------------------------------------------------ */
#define TAG_SERIAL      "Serial"        /* Physical layer init/errors  */
#define TAG_RX          "RX"            /* Inbound frame hex dumps     */
#define TAG_TX          "TX"            /* Outbound frame hex dumps    */
#define TAG_CONTROLLER  "PBController"  /* Frame routing               */
#define TAG_PROTOCOL    "PBProtocol"    /* SAP dispatch, state machine */
#define TAG_ET200S      "ET200S"        /* ET200S application          */
#define TAG_VSD         "VSD"           /* KFC750 VSD application      */
#define TAG_EMULATOR    "PBEmulator"    /* app_main / startup          */

/* ------------------------------------------------------------------ */
/* pb_log_hex                                                         */
/* ------------------------------------------------------------------ */

/**
 * Log a byte array as a compact hex string with timestamp.
 *
 * Output format:
 *   I (timestamp) <tag>: XX XX XX XX XX ...
 *
 * @param level  ESP log level (e.g. ESP_LOG_INFO)
 * @param tag    Log tag string (use TAG_* constants above)
 * @param data   Pointer to bytes to log
 * @param len    Number of bytes (capped at PB_LOG_HEX_MAX internally)
 */
void pb_log_hex(esp_log_level_t level, const char *tag,
                const uint8_t *data, uint16_t len);

/* Maximum bytes logged per call (stack buffer = 3 × PB_LOG_HEX_MAX + 1) */
#define PB_LOG_HEX_MAX  128

#endif /* PB_LOG_H */
