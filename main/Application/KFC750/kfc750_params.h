/**
 * kfc750_params.h
 * KFC750 parameter table — in-memory store for PROFIBUS PKW read/write.
 *
 * Parameters are indexed by their ID number (1–255).
 * Default values match KFC750 factory defaults for 50 Hz operation.
 *
 * Value encoding (follows KFC750 convention):
 *   Frequencies  : Hz × 100  (e.g. 5000 = 50.00 Hz)
 *   Times        : tenths of seconds  (e.g. 100 = 10.0 s)
 *   Control modes: integer enum  (e.g. 3 = PROFIBUS)
 */

#ifndef KFC750_PARAMS_H
#define KFC750_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

#define KFC750_PARAM_COUNT  256  /* IDs 0–255; ID 0 is unused */

typedef struct {
    uint32_t values[KFC750_PARAM_COUNT];
} KFC750_Params;

/**
 * Initialise the parameter table with factory defaults.
 * Must be called once before any read/write.
 */
void KFC750_Params_Init(KFC750_Params *p);

/**
 * Read parameter @id.
 * Returns 0 for invalid or uninitialised IDs.
 */
uint32_t KFC750_Params_Read(const KFC750_Params *p, uint16_t id);

/**
 * Write @value to parameter @id.
 * Returns true on success, false if @id is out of range.
 */
bool KFC750_Params_Write(KFC750_Params *p, uint16_t id, uint32_t value);

#endif /* KFC750_PARAMS_H */
