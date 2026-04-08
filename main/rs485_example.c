/**
 * rs485_example.c
 * App entry point — UART init, PROFIBUS tasks.
 *
 * Hardware:
 *   UART2  RXD → GPIO22
 *          TXD → GPIO23
 *          RTS → GPIO18  (drives DE/~RE on RS-485 transceiver)
 *   Baud: 45,450 bps, 8E1 (even parity — required by PROFIBUS)
 *
 * The ESP-IDF UART RS-485 half-duplex mode automatically drives the
 * RTS/DE pin during transmission.  uart_wait_tx_done() is called after
 * every write to ensure the bus is released before the master expects
 * a response.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "Application/VSD.h"
#include "Profibus/Controller.h"
#include "UART/uartMessage.h"

#define TAG "Profibus"

/* GPIO pins (match sdkconfig / hardware) */
#define ECHO_TEST_TXD   (23)
#define ECHO_TEST_RXD   (22)
#define ECHO_TEST_RTS   (18)   /* DE/~RE */
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT  (2)
#define BAUD_RATE       (45450)

#define RX_BUF_SIZE     (512)
#define TX_BUF_SIZE     (512)
#define RX_QUEUE_LEN    (20)
#define TX_QUEUE_LEN    (10)

/* UART timeout: ~3.5 character times at 45450 baud ≈ 10 bit-times per char
 * TOUT register counts in units of ~10 bit-times, so 3 gives ~30 bit-times. */
#define UART_RX_TOUT    (3)

/* Max time to wait for TX FIFO to drain (ms) */
#define TX_DRAIN_WAIT_MS (20)

static QueueHandle_t rxQueue;
QueueHandle_t txQueue;  /* extern in Controller.h */

const int uart_num = ECHO_UART_PORT;

/* ------------------------------------------------------------------ */
/* TX task — dequeues response frames and transmits                   */
/* ------------------------------------------------------------------ */

static void uart_transmit_task(void *pvParameters)
{
    resp response;
    while (1) {
        if (xQueueReceive(txQueue, &response, portMAX_DELAY)) {
            ESP_LOG_BUFFER_HEXDUMP("TX", response.Data, response.Length, ESP_LOG_DEBUG);
            uart_write_bytes(uart_num, response.Data, response.Length);
            /* CRITICAL: wait for FIFO to drain before releasing the RS-485 bus.
             * Without this, DE/~RE toggles while bytes are still in the FIFO. */
            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(TX_DRAIN_WAIT_MS));
        }
    }
}

/* ------------------------------------------------------------------ */
/* RX task — receives raw bytes and dispatches to protocol layer      */
/* ------------------------------------------------------------------ */

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(rxQueue, &event, portMAX_DELAY)) {
            switch (event.type) {

            case UART_DATA:
                uart_read_bytes(uart_num, data, event.size, portMAX_DELAY);
                ESP_LOG_BUFFER_HEXDUMP("RX", data, event.size, ESP_LOG_DEBUG);
                ProcessMessage(data, event.size);
                break;

            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART FIFO overflow — flushing");
                uart_flush_input(uart_num);
                xQueueReset(rxQueue);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART ring buffer full — flushing");
                uart_flush_input(uart_num);
                xQueueReset(rxQueue);
                break;

            case UART_BREAK:
                /* PROFIBUS inter-frame gap — no action needed */
                break;

            case UART_PARITY_ERR:
                ESP_LOGD(TAG, "UART parity error");
                break;

            case UART_FRAME_ERR:
                ESP_LOGD(TAG, "UART frame error");
                break;

            default:
                ESP_LOGD(TAG, "UART event %d", event.type);
                break;
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* UART initialisation                                                */
/* ------------------------------------------------------------------ */

static void uart_init(void)
{
    ESP_LOGI(TAG, "Init PROFIBUS UART: port=%d baud=%d TX=%d RX=%d DE=%d",
             ECHO_UART_PORT, BAUD_RATE, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS);

    txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(resp));
    if (!txQueue) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate           = BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_EVEN,   /* PROFIBUS requires even parity */
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num,
                                        RX_BUF_SIZE * 2, TX_BUF_SIZE * 2,
                                        RX_QUEUE_LEN, &rxQueue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num,
                                  ECHO_TEST_TXD, ECHO_TEST_RXD,
                                  ECHO_TEST_RTS, ECHO_TEST_CTS));
    /* RS-485 half-duplex: DE pin driven automatically via RTS during TX */
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    /* RX timeout: triggers UART_DATA event after ~3.5 char times of silence */
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, UART_RX_TOUT));
}

/* ------------------------------------------------------------------ */
/* app_main                                                           */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ProfibusSlave starting");

    InitialiseController();

    /*
     * Create VSD instances.
     * Each VSD needs its own profibusSlave struct and mutex.
     * The profibusSlave is registered with the controller in taskEntry().
     *
     * profibusAddress must match what's configured in the Siemens master
     * (HW Config / Network view).
     */
    static profibusSlave slave2;
    VSDSimulator vsd2 = {
        .vsdName        = "VSD2",
        .vsdPriority    = 8,
        .profibusAddress= 2,
        .profibusSlave  = &slave2,
        .speedSetpoint  = 0.0f,
        .currentSpeed   = 0.0f,
        .state          = STOPPED,
        .vsdMutex       = NULL,
    };
    vsd2.vsdMutex = xSemaphoreCreateMutex();

    /* VSD task runs at lower priority than UART tasks */
    xTaskCreate(taskEntry, vsd2.vsdName, 4096, &vsd2, vsd2.vsdPriority, NULL);

    /* Give the VSD task a moment to register the slave before UART starts */
    vTaskDelay(pdMS_TO_TICKS(10));

    uart_init();

    /* UART event task at highest priority — must respond within Tsdx */
    xTaskCreate(uart_event_task,    "uart_rx",  4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_transmit_task, "uart_tx",  4096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI(TAG, "All tasks started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
//        ESP_LOGI(TAG, "Slave2 state=%d speed=%.1f%%",
//                 slave2.State.ReadyState, vsd2.currentSpeed);
    }
}
