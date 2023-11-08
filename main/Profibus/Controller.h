#ifndef PROFIBUS_CONTROLLER_H
#define PROFIBUS_CONTROLLER_H

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "Protocol.h"
#include "Slave.h"
#include "Utility/list.h"
#include "uart/uartMessage.h"

#define TAG_PROFIBUSCONTROLLER "Profibus Controller"

extern Node* ProfibusSlaves;
extern QueueHandle_t txQueue;



uint8_t InitialiseController(void);

void ProcessMessage(uint8_t *Data, uint32_t Length);

void AddSlave(uint8_t slaveAddress, profibusSlave* slave);

#endif // PROFIBUS_CONTROLLER_H