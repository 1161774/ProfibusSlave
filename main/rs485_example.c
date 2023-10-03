/* Uart Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "Application/VSD.h"
#include "Profibus/Controller.h"
#include "uart/uartMessage.h"

/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
*/
#define TAG "Profibus"


// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define ECHO_TEST_TXD   (23)
#define ECHO_TEST_RXD   (22)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (18)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

#define RX_BUF_SIZE        (127)
#define TX_BUF_SIZE        (127)
#define RX_QUEUE_LENGTH    (20)
#define TX_QUEUE_LENGTH    (10)

#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)
static QueueHandle_t rxQueue;
QueueHandle_t txQueue;

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (2)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (33) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks




const int uart_num = ECHO_UART_PORT;

void uart_transmint_task(void *pvParameters) 
{
    resp Response;
    while (1)
    {
        if (xQueueReceive(txQueue, &Response, (TickType_t)portMAX_DELAY))
        {
            ESP_LOG_BUFFER_HEXDUMP("Transmit", Response.Data, Response.Length, ESP_LOG_INFO);
            uart_write_bytes(uart_num, Response.Data, Response.Length);
        }
    }
    
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1)
    {
        //Waiting for UART event.
        if(xQueueReceive(rxQueue, (void * )&event, (TickType_t)portMAX_DELAY)) 
        {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/ 
                case UART_DATA:
                    uart_read_bytes(uart_num, data, event.size, portMAX_DELAY);
                    ProcessMessage(data, event.size);

                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_num);
                    xQueueReset(rxQueue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_num);
                    xQueueReset(rxQueue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
//                    ESP_LOGI(TAG, "UART BREAK EVENT:");
//                    ESP_LOG_BUFFER_HEXDUMP("   Break", data, event.size, ESP_LOG_INFO);
//                    ProcessMessage(data, event.size);
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }


        }
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
}

static void uart_init(void)
{

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Init 485 driver");


    txQueue = xQueueCreate(TX_QUEUE_LENGTH, TX_BUF_SIZE);
    if (txQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART queue");
        return;
    }

    uart_config_t uart_config = {
        .baud_rate = 45450,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, RX_QUEUE_LENGTH, &rxQueue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, 10));//3

}







void app_main(void)
{
    InitialiseController();

    // Create two VSD simulator instances
    profibusSlave profibus1, profibus2;
    VSDSimulator vsd1 = { "VSD1", 1, 0x08, &profibus1, 0.0, 0.0, STOPPED, NULL };
    VSDSimulator vsd2 = { "VSD2", 2, 0x0A, &profibus2, 0.0, 0.0, STOPPED, NULL };

    // Create mutexes for each VSD
    vsd1.vsdMutex = xSemaphoreCreateMutex();
    vsd2.vsdMutex = xSemaphoreCreateMutex();

    // Create tasks for each VSD
    xTaskCreate(taskEntry, vsd1.vsdName, 4096, &vsd1, vsd1.vsdPriority, NULL);
//    xTaskCreate(taskEntry, vsd2.vsdName, 4096, &vsd2, vsd2.vsdPriority, NULL);

    // Simulate receiving Profibus commands for the first VSD (vsd1)
 //   processProfibusCommand(&vsd1, "SET_SPEED 50.0");
 //   processProfibusCommand(&vsd1, "START");

 
    uart_init();

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(uart_transmint_task, "uart_transmit_task", 4096, NULL, configMAX_PRIORITIES, NULL);

while (1)
{
    vTaskDelay(1);
}

}


