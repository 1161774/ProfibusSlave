#ifndef PROFIBUS_SLAVE_H
#define PROFIBUS_SLAVE_H

#include <stdint.h>
#include "Protocol.h"

typedef enum {
    SS_POWERON,    // Power On
    SS_WPRM,       // Wait for Parameter
    SS_WCFG,       // Wait for Configuration
    SS_DXCHG       // Ready for Data Exchange
} profibusSlaveReadyState;

typedef struct {
    uint8_t Address;  // address defaults to 126 as defined in standard
    uint8_t ID_LOW;         // As defined in GSD
    uint8_t ID_HIGH;        // As defined in GSD
} profibusSlaveConfig;

typedef struct {
    profibusSlaveReadyState ReadyState;
    uint8_t Frozen;
    uint8_t Sync;
    uint8_t Group;
} profibusSlaveState;

typedef struct {
    uint8_t FreezeReq;
    uint8_t SyncReq;
    uint8_t LockReq;
    uint8_t WatchdogStatus;
    uint32_t WatchdogVal;   // milliseconds
    DPV1_Status_1 dpv1Status1;
    DPV1_Status_2 dpv1Status2;
    DPV1_Status_3 dpv1Status3;
} profibusSlaveControl;


typedef struct profibusSlave{
    profibusSlaveConfig Config;
    profibusSlaveState State;
    profibusSlaveControl Control;
} profibusSlave;




#endif // PROFIBUS_SLAVE_H
