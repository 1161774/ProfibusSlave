/**
 * kfc750_params.c
 * KFC750 parameter table implementation.
 */

#include "Application/KFC750/kfc750_params.h"
#include "Application/KFC750/kfc750_protocol.h"
#include <string.h>

void KFC750_Params_Init(KFC750_Params *p)
{
    memset(p->values, 0, sizeof(p->values));

    /* Frequency parameters (Hz × 100) */
    p->values[KFC750_PARAM_DESIRED_FREQ] = 5000; /* 50.00 Hz */
    p->values[KFC750_PARAM_MIN_FREQ]     =    0; /*  0.00 Hz */
    p->values[KFC750_PARAM_BOOST_FREQ]   =  500; /*  5.00 Hz */
    p->values[KFC750_PARAM_MAX_FREQ]     = 5000; /* 50.00 Hz */

    /* Ramp times (tenths of seconds) */
    p->values[KFC750_PARAM_ACCEL_TIME]   = 100;  /* 10.0 s */
    p->values[KFC750_PARAM_DECEL_TIME]   = 100;  /* 10.0 s */

    /* Control */
    p->values[KFC750_PARAM_CTRL_CENTER]  =   3;  /* PROFIBUS */
    p->values[KFC750_PARAM_LOCK]         =   0;  /* Unlocked */
}

uint32_t KFC750_Params_Read(const KFC750_Params *p, uint16_t id)
{
    if (id == 0 || id >= KFC750_PARAM_COUNT) return 0;
    return p->values[id];
}

bool KFC750_Params_Write(KFC750_Params *p, uint16_t id, uint32_t value)
{
    if (id == 0 || id >= KFC750_PARAM_COUNT) return false;
    p->values[id] = value;
    return true;
}
