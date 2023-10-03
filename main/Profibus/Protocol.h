#ifndef PROFIBUS_PROTOCOL_H
#define PROFIBUS_PROTOCOL_H

#include <stdint.h>
#include "Slave.h"
#include "esp_log.h"
#include "uart/uartMessage.h"

#define TAG_PROTOCOL "Profibus Protocol"

//TODO - actually figure out how big to make this buffer
#define MAX_RESPONSE (512) //255 bytes Should be the longest that a profibus message can be


#define TELEGRAM_TYPE_SD1 (0x10)  // Start Delimiter 1 - 
#define TELEGRAM_TYPE_SD2 (0x68)  // Start Delimiter 2 - 
#define TELEGRAM_TYPE_SD3 (0xA2)  // Start Delimiter 3 - 
#define TELEGRAM_TYPE_SD4 (0xDC)  // Start Delimiter 4 - 

#define TELEGRAM_ED (0x16)  // End Delimiter
#define TELEGRAM_SC (0xE5)  // Short Confirmation


//uint8_t GetChecksum(uint8_t *data, uint32_t length);

uint8_t GetMessageType(uint8_t* pData, uint32_t Length, uint8_t* pMessageType, uint8_t* pSourceAddress, uint8_t* pDestinationAddress);

uint8_t ProcessTelegramSD1(uint8_t* pMessage, profibusSlaveState* pState, resp* pResponse);

#endif // PROFIBUS_PROTOCOL_H
