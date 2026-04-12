/**
 * VSD.c
 * KFC750 Variable Speed Drive emulator task.
 *
 * Implements the KFC750 PROFIBUS PPO5 protocol (32 bytes I/O):
 *
 *   Bytes  0-7   PKW area — parameter read/write channel
 *   Bytes  8-9   PZD1 — Control Word (master→slave) / Status Word (slave→master)
 *   Bytes 10-11  PZD2 — Speed Reference (master→slave, 0x4000=100%) /
 *                        Actual frequency (slave→master, same scale)
 *   Bytes 12-31  PZD3-12 — motor telemetry (slave→master only)
 *
 * Control Word command encoding (bits 2:0):
 *   0 = No action    1 = Decelerate to min freq
 *   2 = Hold         3 = Accelerate to REF setpoint
 *   4 = Stop (ramp to 0)
 * Bit  7 = Fault reset (acts on 0→1 rising edge)
 * Bit 10 = PROFIBUS control enable (must be set for PLC to control drive)
 *
 * Motor simulation runs every 100 ms.  Ramp rates are derived from
 * parameters ID111 (accel time) and ID112 (decel time).
 */

#include "VSD.h"
#include "Application/KFC750/kfc750_protocol.h"
#include "Application/KFC750/kfc750_params.h"
#include "Logging/pb_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define VSD_TICK_MS         100     /* simulation loop period (ms) */

/* Simulated electrical ratings */
#define RATED_VOLTAGE_V     220.0f  /* output voltage at max frequency */
#define DC_BUS_VOLTAGE_V    310.0f  /* DC bus ≈ 230 VAC × √2 */
#define RATED_CURRENT_A     5.0f    /* rated motor current at full load */
#define POWER_FACTOR        0.85f   /* typical induction motor power factor */

/* ------------------------------------------------------------------ */
/* Big-endian helpers                                                  */
/* ------------------------------------------------------------------ */

static inline uint16_t rd16(const uint8_t *b, int off)
{
    return (uint16_t)(((uint16_t)b[off] << 8) | b[off + 1]);
}

static inline int16_t rds16(const uint8_t *b, int off)
{
    return (int16_t)rd16(b, off);
}

static inline uint32_t rd32(const uint8_t *b, int off)
{
    return ((uint32_t)b[off    ] << 24) | ((uint32_t)b[off + 1] << 16) |
           ((uint32_t)b[off + 2] <<  8) |  (uint32_t)b[off + 3];
}

static inline void wr16(uint8_t *b, int off, uint16_t v)
{
    b[off    ] = (uint8_t)(v >> 8);
    b[off + 1] = (uint8_t)(v);
}

static inline void wr32(uint8_t *b, int off, uint32_t v)
{
    b[off    ] = (uint8_t)(v >> 24);
    b[off + 1] = (uint8_t)(v >> 16);
    b[off + 2] = (uint8_t)(v >>  8);
    b[off + 3] = (uint8_t)(v);
}

/* ------------------------------------------------------------------ */
/* PKW channel — parameter read/write                                 */
/* ------------------------------------------------------------------ */

static void ProcessPKW(VSDSimulator *vsd, const uint8_t *out)
{
    uint16_t pke     = rd16(out, KFC750_PKE_OFFSET);
    uint16_t ind     = rd16(out, KFC750_IND_OFFSET);
    uint32_t pwe     = rd32(out, KFC750_PWE_OFFSET);
    uint8_t  reqType = (uint8_t)((pke >> 12) & 0x0Fu);
    uint16_t paramId = pke & 0x0FFFu;

    if (reqType == PKE_REQ_NO_REQUEST) {
        /* No request pending — clear any previous response */
        vsd->pkwRespPke = 0x0000;
        vsd->pkwRespInd = 0x0000;
        vsd->pkwRespPwe = 0x00000000;
        return;
    }

    if (paramId == 0 || paramId >= KFC750_PARAM_COUNT) {
        ESP_LOGW(TAG_VSD, "%s: PKW invalid ID %u", vsd->vsdName, paramId);
        vsd->pkwRespPke = (uint16_t)((PKE_RESP_REJECTED << 12) | (paramId & 0x0FFFu));
        vsd->pkwRespInd = ind;
        vsd->pkwRespPwe = 0x00000003; /* error: parameter not supported */
        return;
    }

    if (reqType == PKE_REQ_READ_WORD || reqType == PKE_REQ_READ_DWORD) {
        uint32_t val = KFC750_Params_Read(&vsd->params, paramId);
        vsd->pkwRespPke = (uint16_t)((PKE_RESP_WORD_VALUE << 12) | paramId);
        vsd->pkwRespInd = ind;
        vsd->pkwRespPwe = val;
        ESP_LOGD(TAG_VSD, "%s: PKW read  P%u = %lu",
                 vsd->vsdName, paramId, (unsigned long)val);

    } else if (reqType == PKE_REQ_WRITE_WORD || reqType == PKE_REQ_WRITE_DWORD) {
        KFC750_Params_Write(&vsd->params, paramId, pwe);
        vsd->pkwRespPke = (uint16_t)((PKE_RESP_WORD_VALUE << 12) | paramId);
        vsd->pkwRespInd = ind;
        vsd->pkwRespPwe = pwe;
        ESP_LOGI(TAG_VSD, "%s: PKW write P%u = %lu",
                 vsd->vsdName, paramId, (unsigned long)pwe);

    } else {
        ESP_LOGW(TAG_VSD, "%s: PKW unsupported request type 0x%X",
                 vsd->vsdName, reqType);
        vsd->pkwRespPke = (uint16_t)((PKE_RESP_REJECTED << 12) | paramId);
        vsd->pkwRespInd = ind;
        vsd->pkwRespPwe = 0x00000001; /* error: function not supported */
    }
}

/* ------------------------------------------------------------------ */
/* PZD channel — control word + speed reference                       */
/* ------------------------------------------------------------------ */

static void ProcessPZD(VSDSimulator *vsd, const uint8_t *out)
{
    uint16_t cw      = rd16 (out, KFC750_PZD1_OFFSET);
    int16_t  ref     = rds16(out, KFC750_PZD2_OFFSET);
    uint16_t prevCw  = vsd->ctrlWord;
    vsd->ctrlWord    = cw;

    /* Fault reset: rising edge on bit 7 */
    if ((cw & KFC750_CW_FAULT_RESET) && !(prevCw & KFC750_CW_FAULT_RESET)) {
        if (vsd->faultCode) {
            ESP_LOGI(TAG_VSD, "%s: fault %u reset via CW", vsd->vsdName, vsd->faultCode);
            vsd->faultCode = 0;
        }
    }

    if (!(cw & KFC750_CW_PROFIBUS_ENABLE)) {
        /* PROFIBUS control not enabled — drive ignores PLC commands */
        vsd->targetFreq = 0.0f;
        return;
    }

    float maxFreq = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_MAX_FREQ) / 100.0f;
    float minFreq = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_MIN_FREQ) / 100.0f;
    if (maxFreq < 1.0f) maxFreq = 50.0f; /* safety guard */

    /* Convert REF word (−0x4000…+0x4000) to Hz */
    float refFreq = (float)ref * maxFreq / (float)KFC750_SPEED_SCALE;
    if (refFreq < 0.0f)    refFreq = 0.0f;
    if (refFreq > maxFreq) refFreq = maxFreq;

    uint8_t cmd = (uint8_t)(cw & KFC750_CW_CMD_MASK);
    switch (cmd) {

    case KFC750_CW_CMD_ACCEL:
        vsd->targetFreq = refFreq;
        /* Enforce minimum: if non-zero setpoint is below min, clamp to min */
        if (vsd->targetFreq > 0.0f && vsd->targetFreq < minFreq)
            vsd->targetFreq = minFreq;
        break;

    case KFC750_CW_CMD_DECEL:
        vsd->targetFreq = minFreq;
        break;

    case KFC750_CW_CMD_HOLD:
        vsd->targetFreq = vsd->currentFreq; /* freeze at current value */
        break;

    case KFC750_CW_CMD_STOP:
    case KFC750_CW_CMD_NO_ACTION:
    default:
        vsd->targetFreq = 0.0f;
        break;
    }

    ESP_LOGD(TAG_VSD, "%s: CW=0x%04X cmd=%u ref=%.1f Hz target=%.1f Hz",
             vsd->vsdName, cw, cmd, refFreq, vsd->targetFreq);
}

/* ------------------------------------------------------------------ */
/* Motor simulation — one tick                                        */
/* ------------------------------------------------------------------ */

static void SimulateMotor(VSDSimulator *vsd)
{
    /* Faulted drive cannot run */
    if (vsd->faultCode) vsd->targetFreq = 0.0f;

    float maxFreq   = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_MAX_FREQ)   / 100.0f;
    float accelTime = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_ACCEL_TIME) /  10.0f;
    float decelTime = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_DECEL_TIME) /  10.0f;

    /* Guard against zero/corrupt parameter values */
    if (maxFreq   < 1.0f) maxFreq   = 50.0f;
    if (accelTime < 0.1f) accelTime = 10.0f;
    if (decelTime < 0.1f) decelTime = 10.0f;

    /* Hz change allowed in one tick */
    float accelStep = maxFreq / accelTime * ((float)VSD_TICK_MS / 1000.0f);
    float decelStep = maxFreq / decelTime * ((float)VSD_TICK_MS / 1000.0f);

    float diff = vsd->targetFreq - vsd->currentFreq;

    if (diff > accelStep) {
        vsd->currentFreq += accelStep;
        vsd->state = RAMPING_UP;
    } else if (diff < -decelStep) {
        vsd->currentFreq -= decelStep;
        vsd->state = RAMPING_DOWN;
    } else {
        vsd->currentFreq = vsd->targetFreq;
        vsd->state       = (vsd->targetFreq < 0.01f) ? STOPPED : AT_SPEED;
    }

    /* Keep legacy % fields in sync for heartbeat logging */
    vsd->currentSpeed  = (maxFreq > 0.0f) ? (vsd->currentFreq / maxFreq * 100.0f) : 0.0f;
    vsd->speedSetpoint = (maxFreq > 0.0f) ? (vsd->targetFreq  / maxFreq * 100.0f) : 0.0f;
}

/* ------------------------------------------------------------------ */
/* Status Word                                                         */
/* ------------------------------------------------------------------ */

static uint16_t BuildStatusWord(const VSDSimulator *vsd)
{
    uint16_t sw = 0;

    if (!vsd->faultCode) sw |= KFC750_SW_READY;
    if ( vsd->faultCode) sw |= KFC750_SW_FAULT;

    if (vsd->currentFreq > 0.01f) {
        sw |= KFC750_SW_RUNNING;
    } else {
        sw |= KFC750_SW_ZERO_SPEED;
    }

    if (vsd->state == AT_SPEED   && vsd->currentFreq > 0.01f) sw |= KFC750_SW_AT_SPEED;
    if (vsd->state == RAMPING_UP)   sw |= KFC750_SW_ACCEL;
    if (vsd->state == RAMPING_DOWN) sw |= KFC750_SW_DECEL;

    /* PROFIBUS link always active once in Data_Exchange */
    sw |= KFC750_SW_REMOTE;
    sw |= KFC750_SW_PROFIBUS_OK;

    return sw;
}

/* ------------------------------------------------------------------ */
/* Build input buffer (slave → master)                                */
/* ------------------------------------------------------------------ */

static void BuildInputData(VSDSimulator *vsd)
{
    profibusSlave *slave = vsd->profibusSlave;
    uint8_t       *in   = slave->input.data;

    memset(in, 0, KFC750_FRAME_LEN);
    slave->input.len = KFC750_FRAME_LEN;

    /* PKW response */
    wr16(in, KFC750_PKE_OFFSET, vsd->pkwRespPke);
    wr16(in, KFC750_IND_OFFSET, vsd->pkwRespInd);
    wr32(in, KFC750_PWE_OFFSET, vsd->pkwRespPwe);

    /* PZD1: Status Word */
    vsd->statusWord = BuildStatusWord(vsd);
    wr16(in, KFC750_PZD1_OFFSET, vsd->statusWord);

    /* PZD2: Actual frequency (same scale as REF) */
    float maxFreq = (float)KFC750_Params_Read(&vsd->params, KFC750_PARAM_MAX_FREQ) / 100.0f;
    if (maxFreq < 1.0f) maxFreq = 50.0f;
    int16_t act = (int16_t)(vsd->currentFreq * (float)KFC750_SPEED_SCALE / maxFreq);
    wr16(in, KFC750_PZD2_OFFSET, (uint16_t)act);

    /* PZD3: Output frequency Hz×100 */
    wr16(in, KFC750_PZD3_OFFSET, (uint16_t)(vsd->currentFreq * 100.0f));

    /* PZD4: Output voltage (V/Hz V/f curve — linear to max) */
    float volts = (maxFreq > 0.0f) ? (RATED_VOLTAGE_V * vsd->currentFreq / maxFreq) : 0.0f;
    wr16(in, KFC750_PZD4_OFFSET, (uint16_t)volts);

    /* PZD5: Output current A×10 (proportional to load) */
    float amps = (maxFreq > 0.0f) ? (RATED_CURRENT_A * vsd->currentFreq / maxFreq) : 0.0f;
    wr16(in, KFC750_PZD5_OFFSET, (uint16_t)(amps * 10.0f));

    /* PZD6: DC bus voltage (V) — constant once energised */
    wr16(in, KFC750_PZD6_OFFSET, (uint16_t)DC_BUS_VOLTAGE_V);

    /* PZD7: Active fault code */
    wr16(in, KFC750_PZD7_OFFSET, vsd->faultCode);

    /* PZD8: Output power kW×10  (3-phase: P = √3 × V × I × cosφ) */
    float power_kw = (1.732f * volts * amps * POWER_FACTOR) / 1000.0f;
    wr16(in, KFC750_PZD8_OFFSET, (uint16_t)(power_kw * 10.0f));

    /* PZD9: Target frequency Hz×100 */
    wr16(in, KFC750_PZD9_OFFSET, (uint16_t)(vsd->targetFreq * 100.0f));

    /* PZD10-12: AI1, AI2, reserved — left zero */
}

/* ------------------------------------------------------------------ */
/* Main simulation loop                                               */
/* ------------------------------------------------------------------ */

static void RunVSD(VSDSimulator *vsd)
{
    while (1) {
        if (xSemaphoreTake(vsd->vsdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            if (vsd->profibusSlave->State.ReadyState == SS_DXCHG &&
                vsd->profibusSlave->output.len >= KFC750_FRAME_LEN) {

                const uint8_t *out = vsd->profibusSlave->output.data;
                ProcessPKW(vsd, out);
                ProcessPZD(vsd, out);
            }

            SimulateMotor(vsd);
            BuildInputData(vsd);

            xSemaphoreGive(vsd->vsdMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(VSD_TICK_MS));
    }
}

/* ------------------------------------------------------------------ */
/* Task entry                                                         */
/* ------------------------------------------------------------------ */

void taskEntry(void *pvParameters)
{
    VSDSimulator *vsd = (VSDSimulator *)pvParameters;

    /* Initialise parameter table with factory defaults */
    KFC750_Params_Init(&vsd->params);

    /* Initialise PROFIBUS slave state */
    vsd->profibusSlave->State.ReadyState = SS_POWERON;
    vsd->profibusSlave->State.Frozen     = 0;
    vsd->profibusSlave->State.Sync       = 0;
    vsd->profibusSlave->State.Group      = 0;
    vsd->profibusSlave->master_address   = 0xFF;
    vsd->profibusSlave->diag_prm_req     = 1;
    vsd->profibusSlave->diag_prm_fault   = 0;
    vsd->profibusSlave->diag_cfg_fault   = 0;
    vsd->profibusSlave->cnt_diag         = 0;
    vsd->profibusSlave->cnt_prm          = 0;
    vsd->profibusSlave->cnt_cfg          = 0;
    vsd->profibusSlave->cnt_data_exch    = 0;

    /* KFC750 GSD identity: Ident_Number = 0x7050 */
    vsd->profibusSlave->Config.Address = vsd->profibusAddress;
    vsd->profibusSlave->Config.ID_HIGH = 0x70;
    vsd->profibusSlave->Config.ID_LOW  = 0x50;

    /* PPO5 I/O size: 32 bytes each direction */
    vsd->profibusSlave->input.len  = KFC750_FRAME_LEN;
    vsd->profibusSlave->output.len = KFC750_FRAME_LEN;
    memset(vsd->profibusSlave->input.data,  0, KFC750_FRAME_LEN);
    memset(vsd->profibusSlave->output.data, 0, KFC750_FRAME_LEN);

    /* Accept any Chk_Cfg from the master without strict validation */
    vsd->profibusSlave->cfg_strict = 0;

    /* Initialise drive runtime state */
    vsd->currentFreq = 0.0f;
    vsd->targetFreq  = 0.0f;
    vsd->ctrlWord    = 0;
    vsd->faultCode   = 0;
    vsd->pkwRespPke  = 0;
    vsd->pkwRespInd  = 0;
    vsd->pkwRespPwe  = 0;

    /* Pre-fill input buffer so first Data_Exchange returns valid data */
    BuildInputData(vsd);

    /* Register with PROFIBUS controller */
    AddSlave(vsd->profibusAddress, vsd->profibusSlave);

    ESP_LOGI(TAG_VSD, "Starting '%s' at PROFIBUS addr=%u ident=0x7050 "
             "(KFC750 PPO5, 32-byte I/O)",
             vsd->vsdName, vsd->profibusAddress);

    RunVSD(vsd);
}
