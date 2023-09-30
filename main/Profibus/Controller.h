#ifndef PROFIBUS_CONTROLLER_H
#define PROFIBUS_CONTROLLER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/list.h"
#include "esp_log.h"
#include "Protocol.h"
#include "Slave.h"
#include "Utility/list.h"

#define TAG_PROFIBUSCONTROLLER "Profibus Controller"

extern Node* ProfibusSlaves;

uint8_t InitialiseController(void);

void ProcessMessage(uint8_t *Data, uint32_t Length);

void AddSlave(uint8_t slaveAddress, profibusSlave slave);

#endif // PROFIBUS_CONTROLLER_H