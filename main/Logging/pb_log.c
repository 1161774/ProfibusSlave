/**
 * pb_log.c
 * Fast single-call hex frame logger.
 *
 * Implementation notes
 * ====================
 * - The hex string is built in a stack-allocated buffer using a direct
 *   nibble lookup — avoids snprintf() overhead entirely.
 * - One ESP_LOG_LEVEL() call is made at the end — one mutex acquisition,
 *   one esp_log_write() → one UART write transaction.
 * - No heap allocation, no loops calling esp_log_write multiple times.
 */

#include "pb_log.h"

static const char HEX_CHARS[] = "0123456789abcdef";

void pb_log_hex(esp_log_level_t level, const char *tag,
                const uint8_t *data, uint16_t len)
{
    /* Stack buffer: "XX " per byte, last byte "XX", plus null terminator */
    char buf[PB_LOG_HEX_MAX * 3 + 1];

    uint16_t n   = (len <= PB_LOG_HEX_MAX) ? len : PB_LOG_HEX_MAX;
    uint16_t pos = 0;

    for (uint16_t i = 0; i < n; i++) {
        buf[pos++] = HEX_CHARS[data[i] >> 4];
        buf[pos++] = HEX_CHARS[data[i] & 0x0F];
        buf[pos++] = ' ';
    }
    /* Overwrite the trailing space with null terminator */
    if (pos > 0) pos--;
    buf[pos] = '\0';

    /* Single log call — single UART write */
    ESP_LOG_LEVEL(level, tag, "%s", buf);
}
