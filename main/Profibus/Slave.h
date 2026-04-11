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

    /* Diagnostic flags (reflected in Status byte 2 of Slave_Diag response) */
    uint8_t diag_prm_req;     /* Prm_Req  bit0: 1 = slave needs Set_Prm.
                                * Must be 1 at power-on. Master will not send
                                * Set_Prm until this is 1, and checks it is 0
                                * before advancing past WAIT_PRM state.       */
    uint8_t diag_prm_fault;   /* Prm_Fault bit6 of Status1: Set_Prm rejected */
    uint8_t diag_cfg_fault;   /* Cfg_Fault bit2 of Status1: Chk_Cfg rejected */
    uint8_t diag_ext_fault;   /* Ext_Diag  bit3 of Status1: fault in ext diag.
                                * NOTE: Setting this bit is separate from sending
                                * extended diag bytes (ext_diag_len > 0).
                                * The real ET200S sends extended module-status bytes
                                * every cycle (Status1 bit3 = 0, all-OK).
                                * Only set diag_ext_fault=1 when a genuine module
                                * fault needs to be signalled to the master.    */

    /*
     * Stat_Diag (Status2 bit1): slave requests master to keep polling Slave_Diag.
     *
     * The real ET200S sets this after a successful Chk_Cfg to force the master
     * to keep reading diagnostics until the station is fully settled.  After
     * diag_stat_diag_threshold consecutive Slave_Diag responses the slave clears
     * the bit, allowing the master to begin cyclic Data_Exchange.
     *
     * This matches the captured conversation: Status2=0x0E (Stat_Diag=1) in
     * frames 9-15, then Status2=0x0C (Stat_Diag=0) in frame 17.
     *
     * diag_stat_diag           : 1 = Stat_Diag bit is set in responses
     * diag_stat_diag_count     : number of diag responses sent with bit set
     * diag_stat_diag_threshold : clear bit after this many responses (default 4)
     *
     * Set diag_stat_diag_threshold = 0 to disable the feature entirely.
     */
    uint8_t  diag_stat_diag;
    uint8_t  diag_stat_diag_count;
    uint8_t  diag_stat_diag_threshold;

    /*
     * Extended diagnostic data appended after the 6 mandatory Slave_Diag bytes.
     *
     * If ext_diag_len > 0, Protocol.c includes these bytes in every Slave_Diag
     * response (once the slave is in SS_DXCHG or later).
     * Status1 bit3 (Ext_Diag) is set automatically when ext_diag_len > 0.
     *
     * For the ET200S this contains:
     *   Block 1 (9 bytes):  49 00 00 00 00 00 00 00 00
     *   Block 2 (20 bytes): 14 82 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
     */
#define PB_MAX_EXT_DIAG_LEN  57
    uint8_t ext_diag_data[PB_MAX_EXT_DIAG_LEN];
    uint8_t ext_diag_len;

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
