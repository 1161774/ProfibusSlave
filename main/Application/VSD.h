#ifndef VSD_SIMULATOR_H
#define VSD_SIMULATOR_H

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Profibus/Controller.h"

typedef enum {
    RAMPING_UP,
    AT_SPEED,
    RAMPING_DOWN,
    STOPPED
} VSDState;

typedef struct {
    const char      *vsdName;
    UBaseType_t      vsdPriority;
    uint8_t          profibusAddress;
    profibusSlave   *profibusSlave;
    float            speedSetpoint;   /* % of rated speed (0-100) */
    float            currentSpeed;    /* % of rated speed (simulated) */
    VSDState         state;
    SemaphoreHandle_t vsdMutex;
} VSDSimulator;

void taskEntry(void *pvParameters);
void processProfibusCommand(VSDSimulator *vsd, const char *command);

#endif // VSD_SIMULATOR_H
