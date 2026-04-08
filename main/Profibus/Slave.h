#ifndef PROFIBUS_SLAVE_H
#define PROFIBUS_SLAVE_H

#include <stdint.h>
#include "Protocol.h"

typedef enum {
    SS_POWERON,    /* Initial power-on                */
    SS_WPRM,       /* Waiting for Set_Prm             */
    SS_WCFG,       /* Waiting for Chk_Cfg             */
    SS_DXCHG       /* Cyclic Data_Exchange active     */
} profibusSlaveReadyState;

typedef struct {
    uint8_t Address;    /* Station address 1-125                */
    uint8_t ID_HIGH;    /* Ident_Number high byte (from GSD)    */
    uint8_t ID_LOW;     /* Ident_Number low byte                */
} profibusSlaveConfig;

typedef struct {
    profibusSlaveReadyState ReadyState;
    uint8_t Frozen;     /* Set by FREEZE global control        */
    uint8_t Sync;       /* Set by SYNC global control          */
    uint8_t Group;      /* Group allocation from Set_Prm       */
} profibusSlaveState;

typedef struct {
    uint8_t  FreezeReq;
    uint8_t  SyncReq;
    uint8_t  LockReq;
    uint8_t  WatchdogStatus;
    uint32_t WatchdogVal;    /* Timeout in ms (WD_Fact1 × WD_Fact2 × 10) */
    DPV1_Status_1 dpv1Status1;
    DPV1_Status_2 dpv1Status2;
    DPV1_Status_3 dpv1Status3;
} profibusSlaveControl;

/*
 * I/O buffer — shared between Protocol.c (writes) and the application task (reads).
 * For KFC750 PPO5 (cfg byte 0xFF):
 *   16 bytes each direction: 8 PKW (parameter channel) + 8 PZD (process data)
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

    /* Address of master currently controlling us (0xFF = none) */
    uint8_t master_address;

    /* Parametrisation data received from Set_Prm */
    uint8_t prm_data[PB_MAX_PRM_LEN];
    uint8_t prm_len;

    /* Configuration received from Chk_Cfg.
     * cfg_strict=1 → Chk_Cfg is validated against cfg_data/cfg_len.
     * cfg_strict=0 → any config is accepted and stored (default). */
    uint8_t cfg_data[PB_MAX_CFG_LEN];
    uint8_t cfg_len;
    uint8_t cfg_strict;

    /* Diagnostic fault flags (cleared after Slave_Diag response is sent) */
    uint8_t diag_prm_fault;   /* Set when Set_Prm is rejected */
    uint8_t diag_cfg_fault;   /* Set when Chk_Cfg is rejected */

    /* Cyclic I/O buffers.
     *   output: filled by Protocol.c on each Data_Exchange
     *   input:  filled by your application task, returned to master
     * Both protected by caller's mutex (see VSD.c). */
    profibusIOBuffer output;   /* master → slave (control word, setpoint…) */
    profibusIOBuffer input;    /* slave → master (status word, actual speed…) */

    /* Watchdog: timestamp of last good Data_Exchange (esp_timer_get_time ms) */
    int64_t last_rx_time_ms;

    /* Counters (informational) */
    uint32_t cnt_diag;
    uint32_t cnt_prm;
    uint32_t cnt_cfg;
    uint32_t cnt_data_exch;

} profibusSlave;

#endif /* PROFIBUS_SLAVE_H */
