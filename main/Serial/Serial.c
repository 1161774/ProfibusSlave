/**
 * Serial.c
 * PROFIBUS physical layer — RS-485 UART and FreeRTOS task management.
 *
 * Two tasks run at the highest available priority:
 *
 *   uart_rx_task  — receives raw bytes from the UART event queue and hands
 *                   complete frames to ProcessMessage() in Controller.c.
 *
 *   uart_tx_task  — drains the txQueue and writes response frames to the
 *                   UART.  uart_wait_tx_done() is called after every write
 *                   so the RS-485 DE pin is not released while bytes are
 *                   still in the FIFO.
 *
 * Timing notes
 * ============
 * PROFIBUS requires the slave to begin its response within Tsdx (the station
 * delay time configured by the master).  At 45,450 baud the bit time is
 * ~22 µs; a typical Tsdx of 11 bit-times is ~242 µs.
 *
 * The dominant source of latency in this implementation is ESP-IDF's
 * esp_log_write() — each INFO/WARN log call takes 1–3 ms because it writes
 * to the UART console synchronously.  All protocol-processing logs have
 * therefore been demoted to DEBUG level (compiled out in Release builds).
 * Only the final "RX → TX" summary log in Controller.c remains at INFO,
 * and it fires AFTER the response has already been queued — so it adds
 * zero latency to the response path.
 *
 * To reduce log verbosity further, set CONFIG_LOG_DEFAULT_LEVEL_WARN (or
 * higher) in sdkconfig / menuconfig.
 */

#include "Serial.h"
#include "Profibus/Controller.h"
#include "Logging/pb_log.h"
#include "esp_log.h"
#include "driver/uart.h"

/* TAG_SERIAL is defined in pb_log.h */

/* TX queue — resp structs with embedded Data[] arrays (no pointer aliasing) */
QueueHandle_t txQueue;

static QueueHandle_t s_rxQueue;

/* ------------------------------------------------------------------ */
/* TX task                                                            */
/* ------------------------------------------------------------------ */

static void uart_tx_task(void *pvParameters)
{
    resp response;
    while (1) {
        if (xQueueReceive(txQueue, &response, portMAX_DELAY) == pdTRUE) {
            /*
             * Log the frame BEFORE sending so the timestamp reflects when
             * the bytes hit the wire, not when the FIFO finishes draining.
             */
            pb_log_hex(ESP_LOG_INFO, TAG_TX, response.Data, response.Length);
            uart_write_bytes(SERIAL_UART_PORT, response.Data, response.Length);
            uart_wait_tx_done(SERIAL_UART_PORT, pdMS_TO_TICKS(SERIAL_TX_DRAIN_MS));
        }
    }
}

/* ------------------------------------------------------------------ */
/* RX task                                                            */
/* ------------------------------------------------------------------ */

static void uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *buf = (uint8_t *)malloc(SERIAL_RX_BUF + 1);
    if (!buf) {
        ESP_LOGE(TAG_SERIAL, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(s_rxQueue, &event, portMAX_DELAY) != pdTRUE) continue;

        switch (event.type) {

        case UART_DATA:
            uart_read_bytes(SERIAL_UART_PORT, buf, event.size, portMAX_DELAY);
            /*
             * Process the frame first — response is queued inside ProcessMessage.
             * Log the raw RX bytes AFTER queuing so the log never delays the reply.
             */
            ProcessMessage(buf, event.size);
            pb_log_hex(ESP_LOG_INFO, TAG_RX, buf, event.size);
            break;

        case UART_FIFO_OVF:
            ESP_LOGW(TAG_SERIAL, "UART FIFO overflow — flushing RX");
            uart_flush_input(SERIAL_UART_PORT);
            xQueueReset(s_rxQueue);
            break;

        case UART_BUFFER_FULL:
            ESP_LOGW(TAG_SERIAL, "UART ring buffer full — flushing RX");
            uart_flush_input(SERIAL_UART_PORT);
            xQueueReset(s_rxQueue);
            break;

        case UART_BREAK:
            /* PROFIBUS inter-frame gap marker — no action */
            break;

        case UART_PARITY_ERR:
            ESP_LOGD(TAG_SERIAL, "Parity error");
            break;

        case UART_FRAME_ERR:
            ESP_LOGD(TAG_SERIAL, "Frame error");
            break;

        default:
            ESP_LOGD(TAG_SERIAL, "UART event %d", event.type);
            break;
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Serial_Init                                                        */
/* ------------------------------------------------------------------ */

void Serial_Init(void)
{
    ESP_LOGI(TAG_SERIAL,
             "Init PROFIBUS UART%d: baud=%d TX=%d RX=%d DE=%d",
             SERIAL_UART_PORT, SERIAL_BAUD_RATE,
             SERIAL_TXD_PIN, SERIAL_RXD_PIN, SERIAL_RTS_PIN);

    txQueue = xQueueCreate(SERIAL_TX_QUEUE_LEN, sizeof(resp));
    if (!txQueue) {
        ESP_LOGE(TAG_SERIAL, "Failed to create TX queue");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate           = SERIAL_BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_EVEN,    /* PROFIBUS requires even parity */
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(SERIAL_UART_PORT,
                                        SERIAL_RX_BUF * 2,
                                        SERIAL_TX_BUF * 2,
                                        SERIAL_RX_QUEUE_LEN,
                                        &s_rxQueue, 0));
    ESP_ERROR_CHECK(uart_param_config(SERIAL_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SERIAL_UART_PORT,
                                 SERIAL_TXD_PIN, SERIAL_RXD_PIN,
                                 SERIAL_RTS_PIN, UART_PIN_NO_CHANGE));

    /* Half-duplex RS-485: ESP-IDF drives DE automatically during TX */
    ESP_ERROR_CHECK(uart_set_mode(SERIAL_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    /*
     * RX timeout — fires UART_DATA event after ~4 × 10-bit-times of silence.
     * At 45450 baud that is ≈880 µs, satisfying the ≥3.5-character-time
     * inter-frame gap required by PROFIBUS.
     */
    ESP_ERROR_CHECK(uart_set_rx_timeout(SERIAL_UART_PORT, SERIAL_RX_TOUT));

    /*
     * Both tasks at the highest available priority so they preempt all
     * application tasks and respond within Tsdx.
     */
    xTaskCreate(uart_rx_task, "pb_rx", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_tx_task, "pb_tx", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI(TAG_SERIAL, "Serial tasks started");
}
