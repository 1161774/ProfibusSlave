#ifndef PROFIBUS_SLAVE_H
#define PROFIBUS_SLAVE_H

#include <stdint.h>
#include "Protocol.h"

typedef enum {
    SS_POWERON,    // Power On
    SS_WPRM,       // Wait for Parameter (Set_Prm)
    SS_WCFG,       // Wait for Configuration (Chk_Cfg)
    SS_DXCHG       // Data Exchange (cyclic I/O)
} profibusSlaveReadyState;

typedef struct {
    uint8_t Address;    // Station address (1-125)
    uint8_t ID_HIGH;    // Ident number high byte (from GSD Ident_Number)
    uint8_t ID_LOW;     // Ident number low byte
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
    uint32_t WatchdogVal;   // milliseconds (WD_Fact_1 * WD_Fact_2 * 10)
    DPV1_Status_1 dpv1Status1;
    DPV1_Status_2 dpv1Status2;
    DPV1_Status_3 dpv1Status3;
} profibusSlaveControl;

/*
 * I/O and parametrisation data.
 *
 * output: bytes written by the master each Data_Exchange cycle (master → slave)
 * input:  bytes returned to the master each Data_Exchange cycle (slave → master)
 *
 * For KFC750 PPO5 (cfg byte 0xFF = 16-byte I/O, word-consistent):
 *   output.len = 16  (8-byte PKW + 8-byte PZD: ctrl word, speed setpoint...)
 *   input.len  = 16  (8-byte PKW + 8-byte PZD: status word, actual speed...)
 */
#define PB_MAX_IO_LEN   244
#define PB_MAX_CFG_LEN  244
#define PB_MAX_PRM_LEN  244

typedef struct {
    uint8_t data[PB_MAX_IO_LEN];
    uint8_t len;
} profibusIOBuffer;

typedef struct profibusSlave {
    profibusSlaveConfig     Config;
    profibusSlaveState      State;
    profibusSlaveControl    Control;

    /* Parametrisation (from Set_Prm) */
    uint8_t prm_data[PB_MAX_PRM_LEN];
    uint8_t prm_len;

    /* Configuration (from Chk_Cfg).
     * If cfg_strict=1, Chk_Cfg bytes are validated against cfg_data/cfg_len.
     * If cfg_strict=0 (default), any config is accepted and stored. */
    uint8_t cfg_data[PB_MAX_CFG_LEN];
    uint8_t cfg_len;
    uint8_t cfg_strict;

    /* Cyclic I/O - update input.data from your application task.
     * Protocol.c writes to output.data on every Data_Exchange.      */
    profibusIOBuffer output;    /* master → slave */
    profibusIOBuffer input;     /* slave → master */

    /* Last good Data_Exchange timestamp (esp_timer_get_time() ms) */
    int64_t last_rx_time_ms;

} profibusSlave;

#endif // PROFIBUS_SLAVE_H
