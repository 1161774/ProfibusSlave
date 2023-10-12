#ifndef PROFIBUS_PROTOCOL_H
#define PROFIBUS_PROTOCOL_H

#include <string.h>
#include <stdint.h>
#include "Slave.h"
#include "esp_log.h"
#include "uart/uartMessage.h"

#define TAG_PROTOCOL "Profibus Protocol"

//TODO - actually figure out how big to make this buffer
#define MAX_RESPONSE (512) //255 bytes Should be the longest that a profibus message can be


//#define TELEGRAM_TYPE_SD1 (0x10)  // Start Delimiter 1 - 
//#define TELEGRAM_TYPE_SD2 (0x68)  // Start Delimiter 2 - 
//#define TELEGRAM_TYPE_SD3 (0xA2)  // Start Delimiter 3 - 
//#define TELEGRAM_TYPE_SD4 (0xDC)  // Start Delimiter 4 - 

typedef enum {
    TELEGRAM_SD1 = 0x10,    // Telegram with no payload
    TELEGRAM_SD2 = 0x68,    // Telegram with variable length payload
    TELEGRAM_SD3 = 0xA2,    // Telegram with fixed length payload
    TELEGRAM_SD4 = 0xDC,    // Token Pass
} TelegramTypes;

#define TELEGRAM_ED (0x16)  // End Delimiter
#define TELEGRAM_SC (0xE5)  // Short Confirmation

typedef enum {
    FC_TE                   = 0x00, // Time Event
    FC_SEND_DATA_ACK_LOW    = 0x03, // Send Data, Request Acknowledge, Low Priority
    FC_SEND_DATA_NACK_LOW   = 0x04, // Send Data, No Acknowledge, Low Priority
    FC_SEND_DATA_ACK_HIG    = 0x05, // Send Data, Request Acknowledge, High Priority
    FC_SEND_DATA_NACK_HIG   = 0x06, // Send Data, No Acknowledge, High Priority
    FC_MSRD                 = 0x07, // Send Request Data with Multicast Reply
    FC_CV                   = 0x08, // Clock Value
    FC_FDL_STATUS           = 0x09, // Fieldbus Data Later Status
    FC_SRD_LOW              = 0x0C, // Send and Request Data, Low Priority
    FC_SRD_HIGH             = 0x0D, // Send and Request Data, High Priority
    FC_REQ_ID_REPL          = 0x0E, // Request Identity With Reply
    FC_REQ_LSAP_REPL        = 0x0F, // Request LSAP Status With Reply
} FunctionCodes;

typedef enum {
    FRH_SLAVE               = 0x00, // Slave
    FRH_MASTER_NOT_READY    = 0x01, // Master, not ready
    FRH_MASTER_REQ_TOKEN    = 0x02, // Master, ready, without token
    FRH_MASTER_READY        = 0x03, // Master, ready, with token
} FunctionCodeResponsesHigh;

typedef enum {
    FRL_OK = 0x00,  // OK
    FRL_UE = 0x01,  // User Error
    FRL_NR = 0x02,  // No Resources
    FRL_RS = 0x03,  // SAP Not Enabled
    FRL_DL = 0x08,  // Data low 
    FRL_NRD = 0x09,  // No response data ready
    FRL_DH  = 0x0A,  // Data high
    FRL_RDL = 0x0C, // Data not received and data low
    FRL_RDH = 0x0D, // Data not received and data high
} FunctionCodeResponsesLow;


typedef struct {
    uint8_t SrcAddress;
    uint8_t DstAddress;
    TelegramTypes MessageType;
    uint8_t FunctionCode;
    uint8_t PDU[250];
    uint8_t PDULength;
} ProfibusMessage;


uint8_t GetMessage(uint8_t* pData, uint32_t Length,  ProfibusMessage* Message);

uint8_t ProcessFunction(ProfibusMessage Message, profibusSlaveState* pState, resp* pResponse);

#endif // PROFIBUS_PROTOCOL_H
