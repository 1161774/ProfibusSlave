#ifndef ET200S_H
#define ET200S_H

/**
 * ET200S.h
 * Simulator for a Siemens ET 200S distributed I/O station.
 *
 * Hardware configuration being emulated (as seen in the captured hex dump):
 *   Slot 1: PM-E 24VDC power module        — cfg byte 0x00 (no I/O data)
 *   Slot 2: 4DI DC24V digital input card   — cfg byte 0x10 (1 byte input)
 *   Slot 3: 4DI DC24V digital input card   — cfg byte 0x10 (1 byte input)
 *
 * PROFIBUS parameters (from captured conversation):
 *   Ident Number : 0x80E0  (IM151-1 HIGH FEATURE, GSD: SI0180E0.GSD)
 *   Config bytes : 0x00 0x10 0x10
 *   I/O size     : 2 bytes input, 0 bytes output
 *
 * Input byte layout (slave → master):
 *   input[0] : slot 2 — 4DI card 1 (bits 3:0 = DI 0–3, bits 7:4 unused)
 *   input[1] : slot 3 — 4DI card 2 (bits 3:0 = DI 4–7, bits 7:4 unused)
 *
 * The application task periodically updates the input data to reflect
 * the current simulated digital input states so the master always gets
 * fresh data on each Data_Exchange cycle.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Profibus/Controller.h"

/* ---------------------------------------------------------------------------
 * ET200S configuration constants
 * These come directly from the captured hex dump (frame 5 / frame 7).
 * --------------------------------------------------------------------------- */
#define ET200S_IDENT_HIGH   0x80
#define ET200S_IDENT_LOW    0xE0

/* Config bytes that the master will send in Chk_Cfg (and that we return in Get_Cfg).
 * Three bytes: PM-E special (0x00) + 4DI (0x10) + 4DI (0x10). */
#define ET200S_CFG_LEN      3
extern const uint8_t ET200S_CFG_BYTES[ET200S_CFG_LEN];

/* I/O sizes derived from the config bytes above */
#define ET200S_INPUT_LEN    2   /* 2 bytes in  (one per 4DI card) */
#define ET200S_OUTPUT_LEN   0   /* 0 bytes out (no output modules) */

/* ---------------------------------------------------------------------------
 * Simulated digital input state
 * The application can update these flags at any time; the task copies them
 * into slave->input.data on each cycle.
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t di_card1;   /* Bits 3:0 → DI 0-3 of slot 2 card */
    uint8_t di_card2;   /* Bits 3:0 → DI 4-7 of slot 3 card */
} ET200S_Inputs;

/* ---------------------------------------------------------------------------
 * ET200S simulator context
 * --------------------------------------------------------------------------- */
typedef struct {
    const char       *name;
    uint8_t           profibusAddress;
    profibusSlave    *slave;
    ET200S_Inputs     inputs;         /* Current simulated input state     */
    SemaphoreHandle_t mutex;          /* Protects inputs + slave I/O buffs */
} ET200S_Simulator;

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

/**
 * FreeRTOS task entry point.
 * Pass a pointer to a fully-initialised ET200S_Simulator as pvParameters.
 * This function never returns.
 */
void ET200S_TaskEntry(void *pvParameters);

/**
 * Set a single digital input bit.
 *   sim     : the simulator instance
 *   di_num  : 0-3 = card1 bits, 4-7 = card2 bits
 *   value   : 0 = off, non-zero = on
 */
void ET200S_SetDI(ET200S_Simulator *sim, uint8_t di_num, bool value);

/**
 * Set all inputs on card 1 at once (bits 3:0 used).
 */
void ET200S_SetCard1(ET200S_Simulator *sim, uint8_t byte_val);

/**
 * Set all inputs on card 2 at once (bits 3:0 used).
 */
void ET200S_SetCard2(ET200S_Simulator *sim, uint8_t byte_val);

#endif /* ET200S_H */
