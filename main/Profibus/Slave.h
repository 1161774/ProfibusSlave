#ifndef PROFIBUS_SLAVE_H
#define PROFIBUS_SLAVE_H

#include <stdint.h>

typedef enum {
    SS_POWERON,    // Power On
    SS_WPRM,       // Wait for Parameter
    SS_WCFG,       // Wait for Configuration
    SS_DXCHG       // Ready for Data Exchange
} profibusSlaveState;

typedef struct {
    profibusSlaveState state;
} profibusSlave;


// address defaults to 126 as defined in standard
//uint8_t slave_addr = 126;

// default state is poweron
//profibusSlaveState state = POWERON;



#endif // PROFIBUS_SLAVE_H
