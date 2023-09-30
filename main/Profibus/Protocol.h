#ifndef PROFIBUS_PROTOCOL_H
#define PROFIBUS_PROTOCOL_H

#include <stdint.h>
#include "Slave.h"

#define TELEGRAM_TYPE_SD1 (0x10)  // Start Delimiter 1 - 
#define TELEGRAM_TYPE_SD2 (0x68)  // Start Delimiter 2 - 
#define TELEGRAM_TYPE_SD3 (0xA2)  // Start Delimiter 3 - 
#define TELEGRAM_TYPE_SD4 (0xDC)  // Start Delimiter 4 - 

#define TELEGRAM_ED (0x16)  // End Delimiter
#define TELEGRAM_SC (0xE5)  // Short Confirmation


uint8_t checksum(uint8_t *data, uint32_t length);

uint8_t GetMessageType(uint8_t *Data, uint32_t Length, uint8_t *MessageType, uint8_t *SourceAddress, uint8_t *DestinationAddress);

uint8_t ProcessTelegramSD1(uint8_t code, profibusSlaveState* state, uint8_t* response);

#endif // PROFIBUS_PROTOCOL_H
