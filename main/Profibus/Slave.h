#ifndef PROFIBUS_SLAVE_H
#define PROFIBUS_SLAVE_H

#include <stdint.h>

typedef enum {
    POWERON,    // Power On
    WPRM,       // Wait for Parameter
    WCFG,       // Wait for Configuration
    DXCHG       // Ready for Data Exchange
} profibusSlaveState;

typedef struct {
    profibusSlaveState state;
} profibusSlave;


// address defaults to 126 as defined in standard
//uint8_t slave_addr = 126;

// default state is poweron
//profibusSlaveState state = POWERON;



#endif // PROFIBUS_SLAVE_H
