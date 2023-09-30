
#ifndef VSD_SIMULATOR_H
#define VSD_SIMULATOR_H

#include "Profibus/Controller.h"

// Simulated Profibus library (replace with actual Profibus library)
typedef struct {
    // Method to simulate receiving a Profibus command
    void (*receiveCommand)(const char* command);
} Profibus;

typedef enum {
    RAMPING_UP,
    AT_SPEED,
    RAMPING_DOWN,
    STOPPED
} VSDState;

typedef struct {
    const char* vsdName;
    UBaseType_t vsdPriority;
//    Profibus* profibus;
    uint8_t profibusAddress;
    profibusSlave* profibusSlave;
    float speedSetpoint;
    float currentSpeed;
    VSDState state;
    SemaphoreHandle_t vsdMutex;

} VSDSimulator;



void taskEntry(void* pvParameters);
void processProfibusCommand(VSDSimulator* vsd, const char* command);

#endif // VSD_SIMULATOR_H
