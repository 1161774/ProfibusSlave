/**
 * ET200S.c
 * Simulator task for a Siemens ET 200S distributed I/O station.
 *
 * Hardware emulated:
 *   Slot 1: PM-E 24VDC power module  (cfg 0x00, no I/O)
 *   Slot 2: 4DI DC24V               (cfg 0x10, 1 byte input)
 *   Slot 3: 4DI DC24V               (cfg 0x10, 1 byte input)
 *
 * Ident: 0x80E0  (IM151-1 HIGH FEATURE)
 *
 * Protocol behaviour verified against captured hex dump:
 *
 *   1. Initial Slave_Diag response (6 bytes):
 *        Status1=0x02 (Not_Ready), Status2=0x05 (Prm_Req, Always1)
 *
 *   2. After Chk_Cfg succeeds, Slave_Diag response grows to 35 bytes
 *      (6 mandatory + 29 extended) and Stat_Diag=1 forces master to keep
 *      polling.  After ET200S_STAT_DIAG_POLLS consecutive polls the slave
 *      clears Stat_Diag (matching frame 17 in the dump).
 *
 *   3. Extended diag structure (29 bytes, verbatim from the capture):
 *        Block 1 (device-related, 9 bytes):
 *          49 00 00 00 00 00 00 00 00
 *        Block 2 (static module status, 20 bytes):
 *          14 82 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *
 *   4. Input data (2 bytes, slave → master on each Data_Exchange):
 *        byte 0 = di_card1 (slot 2 digital inputs, bits 3:0)
 *        byte 1 = di_card2 (slot 3 digital inputs, bits 3:0)
 */

#include "ET200S.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG_ET200S "ET200S"

/* Number of Stat_Diag polls before clearing the bit.
 * The captured dump shows 4 polls with Stat_Diag=1 then it clears. */
#define ET200S_STAT_DIAG_POLLS  4

/* Config bytes returned in Get_Cfg and validated against Chk_Cfg */
const uint8_t ET200S_CFG_BYTES[ET200S_CFG_LEN] = {
    0x00,   /* Slot 1: PM-E 24VDC — special identifier, no data */
    0x10,   /* Slot 2: 4DI DC24V  — 1 byte input               */
    0x10,   /* Slot 3: 4DI DC24V  — 1 byte input               */
};

/* ---------------------------------------------------------------------------
 * Input data update — copies current simulated DI state into the slave's
 * input buffer so Protocol.c picks it up on the next Data_Exchange.
 * Must be called with sim->mutex held.
 * --------------------------------------------------------------------------- */
static void UpdateInputData(ET200S_Simulator *sim)
{
    profibusSlave *s = sim->slave;
    s->input.data[0] = sim->inputs.di_card1 & 0x0F;
    s->input.data[1] = sim->inputs.di_card2 & 0x0F;
    s->input.len     = ET200S_INPUT_LEN;
}

/* ---------------------------------------------------------------------------
 * Main task loop
 * --------------------------------------------------------------------------- */
static void RunET200S(ET200S_Simulator *sim)
{
    while (1) {
        if (xSemaphoreTake(sim->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            /* Keep input buffer fresh so master always reads current values */
            if (sim->slave->State.ReadyState == SS_DXCHG) {
                UpdateInputData(sim);
            }

            xSemaphoreGive(sim->mutex);
        }

        /* 10 ms loop — fast enough for any reasonable DP cycle time */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ---------------------------------------------------------------------------
 * Task entry point
 * --------------------------------------------------------------------------- */
void ET200S_TaskEntry(void *pvParameters)
{
    ET200S_Simulator *sim = (ET200S_Simulator *)pvParameters;
    profibusSlave    *s   = sim->slave;

    /* ---- Initialise slave state ---- */
    memset(s, 0, sizeof(profibusSlave));

    s->State.ReadyState  = SS_POWERON;
    s->master_address    = 0xFF;
    s->diag_prm_req      = 1;   /* Tell master: I need Set_Prm */
    s->diag_prm_fault    = 0;
    s->diag_cfg_fault    = 0;
    s->diag_stat_diag    = 0;
    s->diag_stat_diag_count = 0;
    s->diag_ext_fault    = 0;

    /* Ident number from GSD (SI0180E0.GSD) */
    s->Config.Address  = sim->profibusAddress;
    s->Config.ID_HIGH  = ET200S_IDENT_HIGH;
    s->Config.ID_LOW   = ET200S_IDENT_LOW;

    /* Pre-load the expected config so Get_Cfg can answer immediately
     * and Chk_Cfg can be validated.
     * cfg_strict=1: reject any Chk_Cfg that doesn't match exactly. */
    memcpy(s->cfg_data, ET200S_CFG_BYTES, ET200S_CFG_LEN);
    s->cfg_len    = ET200S_CFG_LEN;
    s->cfg_strict = 1;

    /* I/O buffer sizes */
    s->input.len  = ET200S_INPUT_LEN;
    s->output.len = ET200S_OUTPUT_LEN;
    memset(s->input.data,  0, ET200S_INPUT_LEN);
    memset(s->output.data, 0, sizeof(s->output.data));

    /* Stat_Diag poll counter threshold (matches ET200S behaviour: 4 polls) */
    s->diag_stat_diag_threshold = ET200S_STAT_DIAG_POLLS;

    /*
     * Extended diagnostic data — loaded verbatim from the captured conversation.
     *
     * After Chk_Cfg succeeds the ET200S appends 29 bytes of extended diag to
     * every Slave_Diag response.  The data is split into two PROFIBUS diag blocks:
     *
     *   Block 1 — device-related (type=0x01, length=9):
     *     49 00 00 00 00 00 00 00 00
     *     Byte 0: 0x49 = header (bits[7:6]=01 device-related, bits[5:0]=9 bytes)
     *     Bytes 1-8: reserved/zero
     *
     *   Block 2 — module status (type=0x00 static, length=20):
     *     14 82 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
     *     Byte 0: 0x14 = header (bits[7:6]=00 static, bits[5:0]=20 bytes)
     *     Byte 1: 0x82 = slot 1 module status (0x82 = good/present)
     *     Bytes 2-19: remaining slot status bytes (all 0x00 = OK)
     *
     * Status1 bit3 (Ext_Diag) is automatically set by DiagStatus1() when
     * ext_diag_len > 0.
     */
    static const uint8_t et200s_ext_diag[] = {
        /* Block 1: device-related diag, 9 bytes */
        0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        /* Block 2: static module status, 20 bytes */
        0x14, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    memcpy(s->ext_diag_data, et200s_ext_diag, sizeof(et200s_ext_diag));
    s->ext_diag_len = sizeof(et200s_ext_diag);  /* 29 bytes */

    /* Register with the PROFIBUS controller */
    AddSlave(sim->profibusAddress, s);

    ESP_LOGI(TAG_ET200S,
             "ET200S '%s' registered at addr=%u ident=0x%02X%02X",
             sim->name, sim->profibusAddress,
             ET200S_IDENT_HIGH, ET200S_IDENT_LOW);
    ESP_LOGI(TAG_ET200S,
             "Config: PM-E(0x00) + 4DI(0x10) + 4DI(0x10) => %u byte(s) in, %u byte(s) out",
             ET200S_INPUT_LEN, ET200S_OUTPUT_LEN);

    RunET200S(sim);
}

/* ---------------------------------------------------------------------------
 * Public helper — set individual DI bit
 * --------------------------------------------------------------------------- */
void ET200S_SetDI(ET200S_Simulator *sim, uint8_t di_num, bool value)
{
    if (xSemaphoreTake(sim->mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    if (di_num <= 3) {
        if (value) sim->inputs.di_card1 |=  (1u << di_num);
        else       sim->inputs.di_card1 &= ~(1u << di_num);
    } else if (di_num <= 7) {
        uint8_t bit = di_num - 4;
        if (value) sim->inputs.di_card2 |=  (1u << bit);
        else       sim->inputs.di_card2 &= ~(1u << bit);
    }

    xSemaphoreGive(sim->mutex);
}

void ET200S_SetCard1(ET200S_Simulator *sim, uint8_t byte_val)
{
    if (xSemaphoreTake(sim->mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    sim->inputs.di_card1 = byte_val & 0x0F;
    xSemaphoreGive(sim->mutex);
}

void ET200S_SetCard2(ET200S_Simulator *sim, uint8_t byte_val)
{
    if (xSemaphoreTake(sim->mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    sim->inputs.di_card2 = byte_val & 0x0F;
    xSemaphoreGive(sim->mutex);
}
