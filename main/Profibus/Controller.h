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
#include "Serial/Serial.h"   /* provides txQueue */

#include "Logging/pb_log.h"   /* TAG_CONTROLLER and all other shared log tags */

extern Node *ProfibusSlaves;

uint8_t InitialiseController(void);
void    ProcessMessage(uint8_t *Data, uint32_t Length);
void    AddSlave(uint8_t slaveAddress, profibusSlave *slave);

#endif // PROFIBUS_CONTROLLER_H