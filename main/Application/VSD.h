/**
 * VSD.h
 * KFC750 Variable Speed Drive simulator — public interface.
 *
 * VSDSimulator holds all runtime state for one KFC750 slave instance.
 * Declare it statically in app_main and pass a pointer to taskEntry().
 *
 * Fields read from outside this module (e.g. heartbeat log):
 *   currentSpeed    — current output as % of rated speed (0–100)
 *   state           — high-level drive state (STOPPED / RAMPING_UP / etc.)
 *   profibusSlave   — pointer to the underlying PROFIBUS slave struct
 */

#ifndef VSD_SIMULATOR_H
#define VSD_SIMULATOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Profibus/Controller.h"
#include "Application/KFC750/kfc750_params.h"

typedef enum {
    RAMPING_UP,
    AT_SPEED,
    RAMPING_DOWN,
    STOPPED
} VSDState;

typedef struct {
    /* ---- configuration (set before taskEntry) ---- */
    const char       *vsdName;
    UBaseType_t       vsdPriority;
    uint8_t           profibusAddress;
    profibusSlave    *profibusSlave;
    SemaphoreHandle_t vsdMutex;

    /* ---- observable state (read-only from outside) ---- */
    float    speedSetpoint; /* % of rated speed  (0–100) */
    float    currentSpeed;  /* % of rated speed  (0–100) */
    VSDState state;         /* high-level drive state */

    /* ---- KFC750 runtime state (private to VSD.c) ---- */
    float    currentFreq;   /* current output frequency (Hz) */
    float    targetFreq;    /* target frequency (Hz) */
    uint16_t ctrlWord;      /* last received Control Word */
    uint16_t statusWord;    /* last built Status Word */
    uint16_t faultCode;     /* active fault code (0 = none) */

    /* PKW response held until the next Data_Exchange */
    uint16_t pkwRespPke;
    uint16_t pkwRespInd;
    uint32_t pkwRespPwe;

    /* Parameter table */
    KFC750_Params params;
} VSDSimulator;

/**
 * FreeRTOS task entry point.
 * @param pvParameters  pointer to a VSDSimulator instance.
 */
void taskEntry(void *pvParameters);

#endif /* VSD_SIMULATOR_H */
