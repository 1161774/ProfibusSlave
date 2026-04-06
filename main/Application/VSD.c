/**
 * VSD.c
 * Variable Speed Drive simulator task.
 *
 * This registers a profibusSlave with the controller and runs a loop
 * that:
 *   - reads output data (master → slave) written by Protocol.c on each
 *     Data_Exchange cycle (control word + speed setpoint)
 *   - updates input data (slave → master) with simulated drive status
 *     (status word + actual speed)
 *
 * KFC750 PPO5 data format (16 bytes each direction):
 *   Bytes 0-7:  PKW area (parameter request/response) — zeroed here
 *   Bytes 8-9:  PZD1 — Control Word / Status Word
 *   Bytes 10-11: PZD2 — Speed Setpoint / Actual Speed
 *   Bytes 12-15: PZD3-4 — additional (zeroed here)
 *
 * Control Word (CW) bits relevant for basic operation:
 *   bit0  ON                   (1 = run command)
 *   bit1  No coast stop        (1 = normal)
 *   bit2  No quick stop        (1 = normal)
 *   bit3  Enable operation     (1 = normal)
 *   bit4  Enable ramp           
 *   bit5  Unfreeze ramp        
 *   bit6  Enable setpoint      
 *   bit7  Reset fault          (0→1 transition)
 *   ...
 *   0x047E = Ready to run (bits 1,2,3,4,5,6 set)
 *   0x047F = Run (bit0 also set)
 *
 * Status Word (SW) bits:
 *   bit0  Ready to switch on
 *   bit1  Ready to operate
 *   bit2  Operation enabled
 *   bit3  Fault
 *   bit4  No coast stop
 *   bit5  No quick stop
 *   bit6  Switch on disabled
 *   bit7  Warning
 *   bit8  Speed == setpoint (at speed)
 *   bit9  Remote (under PLC control)
 *   bit10 Target reached
 *   bit11 Internal limit active
 *   ...
 *   0x0637 = Running, at speed, remote
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "VSD.h"

#define VSD_TAG "VSD"

/* KFC750 PPO5: 16 bytes each direction (8 PKW + 8 PZD) */
#define KFC750_IO_LEN   16
#define PKW_LEN         8   /* bytes 0-7: parameter channel  */
#define PZD1_OFFSET     8   /* bytes 8-9: control/status word */
#define PZD2_OFFSET     10  /* bytes 10-11: setpoint/actual   */

/* Speed scaling: 0x4000 (16384) = 100% of rated speed */
#define SPEED_SCALE     16384

static void UpdateInputData(VSDSimulator *vsd)
{
    profibusSlave *slave = vsd->profibusSlave;

    /* Determine status word based on VSD state */
    uint16_t statusWord;
    switch (vsd->state) {
    case RAMPING_UP:
    case RAMPING_DOWN:
        statusWord = 0x0237;  /* Running, not yet at speed */
        break;
    case AT_SPEED:
        statusWord = 0x0637;  /* Running, at speed, remote */
        break;
    case STOPPED:
    default:
        statusWord = 0x0231;  /* Ready, stopped */
        break;
    }

    /* Actual speed as fraction of rated (scaled) */
    int16_t actualSpeed = (int16_t)(vsd->currentSpeed * SPEED_SCALE / 100.0f);

    uint8_t *in = slave->input.data;
    memset(in, 0, KFC750_IO_LEN);
    slave->input.len = KFC750_IO_LEN;

    /* PKW area (bytes 0-7): leave as zeros (no parameter request active) */

    /* PZD1: Status Word (big-endian) */
    in[PZD1_OFFSET    ] = (uint8_t)(statusWord >> 8);
    in[PZD1_OFFSET + 1] = (uint8_t)(statusWord & 0xFF);

    /* PZD2: Actual Speed (big-endian) */
    in[PZD2_OFFSET    ] = (uint8_t)(actualSpeed >> 8);
    in[PZD2_OFFSET + 1] = (uint8_t)(actualSpeed & 0xFF);
}

static void ProcessOutputData(VSDSimulator *vsd)
{
    profibusSlave *slave = vsd->profibusSlave;

    if (slave->output.len < PZD1_OFFSET + 2) return;  /* Not enough data yet */

    uint8_t *out = slave->output.data;

    /* PZD1: Control Word */
    uint16_t ctrlWord = ((uint16_t)out[PZD1_OFFSET] << 8) | out[PZD1_OFFSET + 1];

    /* PZD2: Speed Setpoint (scaled, big-endian) */
    int16_t spScaled = (int16_t)(((uint16_t)out[PZD2_OFFSET] << 8) | out[PZD2_OFFSET + 1]);
    float   setpoint = (float)spScaled * 100.0f / (float)SPEED_SCALE;

    bool runCmd = (ctrlWord & 0x0001) != 0;

    ESP_LOGD(VSD_TAG, "%s: CW=0x%04X SP=%.1f%% run=%d",
             vsd->vsdName, ctrlWord, setpoint, runCmd);

    if (runCmd) {
        vsd->speedSetpoint = setpoint;
    } else {
        vsd->speedSetpoint = 0.0f;
    }
}

static void RunVSD(VSDSimulator *vsd)
{
    const float RAMP_RATE = 2.0f;   /* % per 100 ms */

    while (1) {
        if (xSemaphoreTake(vsd->vsdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            /* Read outputs from master and update setpoint */
            if (vsd->profibusSlave->State.ReadyState == SS_DXCHG) {
                ProcessOutputData(vsd);
            }

            /* Simulate drive ramp */
            if (vsd->currentSpeed < vsd->speedSetpoint) {
                vsd->currentSpeed += RAMP_RATE;
                if (vsd->currentSpeed >= vsd->speedSetpoint) {
                    vsd->currentSpeed = vsd->speedSetpoint;
                    vsd->state = AT_SPEED;
                } else {
                    vsd->state = RAMPING_UP;
                }
            } else if (vsd->currentSpeed > vsd->speedSetpoint) {
                vsd->currentSpeed -= RAMP_RATE;
                if (vsd->currentSpeed <= vsd->speedSetpoint) {
                    vsd->currentSpeed = vsd->speedSetpoint;
                    vsd->state = (vsd->speedSetpoint == 0.0f) ? STOPPED : AT_SPEED;
                } else {
                    vsd->state = RAMPING_DOWN;
                }
            }

            /* Update input data for next Data_Exchange response */
            UpdateInputData(vsd);

            xSemaphoreGive(vsd->vsdMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void processProfibusCommand(VSDSimulator *vsd, const char *command)
{
    if (strncmp(command, "START", 5) == 0) {
        vsd->speedSetpoint = 50.0f;
    } else if (strncmp(command, "STOP", 4) == 0) {
        vsd->speedSetpoint = 0.0f;
    } else if (strncmp(command, "SET_SPEED", 9) == 0) {
        float sp;
        if (sscanf(command, "SET_SPEED %f", &sp) == 1) {
            vsd->speedSetpoint = sp;
        }
    }
}

void taskEntry(void *pvParameters)
{
    VSDSimulator *vsd = (VSDSimulator *)pvParameters;

    /* Initialise slave state */
    vsd->profibusSlave->State.ReadyState = SS_POWERON;
    vsd->profibusSlave->State.Frozen     = 0;
    vsd->profibusSlave->State.Sync       = 0;
    vsd->profibusSlave->State.Group      = 0;

    /* Identity from kfc750.gsd: Ident_Number = 0x7050 */
    vsd->profibusSlave->Config.Address  = vsd->profibusAddress;
    vsd->profibusSlave->Config.ID_HIGH  = 0x70;
    vsd->profibusSlave->Config.ID_LOW   = 0x50;

    /* Pre-set I/O lengths for KFC750 PPO5 (cfg byte 0xFF = 16 bytes I/O).
     * Protocol.c will also derive these from Chk_Cfg, but setting them
     * here means input data is ready from the first cycle. */
    vsd->profibusSlave->input.len  = KFC750_IO_LEN;
    vsd->profibusSlave->output.len = KFC750_IO_LEN;
    memset(vsd->profibusSlave->input.data,  0, KFC750_IO_LEN);
    memset(vsd->profibusSlave->output.data, 0, KFC750_IO_LEN);

    /* Optionally enforce strict Chk_Cfg validation against KFC750 PPO5 cfg.
     * KFC750 PPO5 module has a single cfg byte: 0xFF. */
    vsd->profibusSlave->cfg_data[0] = 0xFF;
    vsd->profibusSlave->cfg_len     = 1;
    vsd->profibusSlave->cfg_strict  = 0;  /* Set to 1 to enforce strict match */

    /* Populate initial status word so master gets sensible data immediately */
    UpdateInputData(vsd);

    /* Register with the controller */
    AddSlave(vsd->profibusAddress, vsd->profibusSlave);

    ESP_LOGI(VSD_TAG, "Starting VSD '%s' at PROFIBUS addr=%u ident=0x7050",
             vsd->vsdName, vsd->profibusAddress);

    RunVSD(vsd);
}
