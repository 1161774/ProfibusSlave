#ifndef PROFIBUS_PROTOCOL_H
#define PROFIBUS_PROTOCOL_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "Serial/Serial.h"     /* resp, MAX_RESPONSE, txQueue */

#include "Logging/pb_log.h"   /* TAG_PROTOCOL and all other shared log tags */

typedef struct profibusSlave profibusSlave; // forward declaration

/* ------------------------------------------------------------------ */
/* Frame types (start delimiters)                                     */
/* ------------------------------------------------------------------ */
typedef enum {
    TELEGRAM_SD1 = 0x10,    // Fixed-length, no data
    TELEGRAM_SD2 = 0x68,    // Variable-length
    TELEGRAM_SD3 = 0xA2,    // Fixed 8-byte data
    TELEGRAM_SD4 = 0xDC,    // Token frame
    TELEGRAM_SC  = 0xE5,    // Short acknowledge
    TELEGRAM_ED  = 0x16     // End delimiter
} TelegramTypes;

/* ------------------------------------------------------------------ */
/* FC byte — lower nibble = function, upper bits = REQ/FCV/FCB        */
/* ------------------------------------------------------------------ */
#define FC_REQ_BIT   0x40   // Set in requests from master
#define FC_FCV_BIT   0x10   // Frame Count Valid
#define FC_FCB_BIT   0x20   // Frame Count Bit (alternates each new request)

typedef enum {
    FC_TE                = 0x00, // Time Event
    FC_SEND_DATA_ACK_LOW = 0x03, // Send Data with Acknowledge, low priority
    FC_SEND_DATA_NACK_LOW= 0x04, // Send Data, No Acknowledge, low priority
    FC_SEND_DATA_ACK_HIG = 0x05, // Send Data with Acknowledge, high priority
    FC_SEND_DATA_NACK_HIG= 0x06, // Send Data, No Acknowledge, high priority (broadcast)
    FC_MSRD              = 0x07, // Send Request with Multicast Reply
    FC_CV                = 0x08, // Clock Value
    FC_FDL_STATUS        = 0x09, // FDL Status
    FC_SRD_LOW           = 0x0C, // Send and Request Data, low priority
    FC_SRD_HIGH          = 0x0D, // Send and Request Data, high priority
    FC_REQ_ID_REPL       = 0x0E, // Request Identity
    FC_REQ_LSAP_REPL     = 0x0F, // Request LSAP Status
} FunctionCodes;

/* Response FC byte: high nibble encodes slave type + data status */
#define FC_RESP_SLAVE_DL   0x08  // Slave, Data Low (no high-priority data pending)
#define FC_RESP_SLAVE_DH   0x0A  // Slave, Data High

/* ------------------------------------------------------------------ */
/* PROFIBUS-DP Service Access Points (SAP numbers)                    */
/* ------------------------------------------------------------------ */
#define SAP_BIT         0x80    // Bit 7 of DA/SA signals SAP extension follows

/*
 * PROFIBUS-DP Service Access Points (slave DSAPs)
 * Source: Felser PROFIBUS Manual - felser.ch/profibus-manual/service_access_point.html
 *
 * SAP (dec) | Hex  | Name
 * ----------+------+------------------
 *   55      | 0x37 | Set_Slave_Adr
 *   56      | 0x38 | Rd_Inp
 *   57      | 0x39 | Rd_Outp
 *   58      | 0x3A | Global_Control   (broadcast SDN, no response)
 *   59      | 0x3B | Get_Cfg          (slave returns its accepted cfg bytes)
 *   60      | 0x3C | Slave_Diag
 *   61      | 0x3D | Set_Prm
 *   62      | 0x3E | Chk_Cfg
 *   NIL     | ---  | Data_Exchange    (no SAP extension in frame)
 */
#define SAP_SET_ADDR    0x37    // 55: Set_Slave_Adr
#define SAP_RD_INP      0x38    // 56: Read Inputs
#define SAP_RD_OUTP     0x39    // 57: Read Outputs
#define SAP_GLB_CTRL    0x3A    // 58: Global_Control (broadcast, no response)
#define SAP_GET_CFG     0x3B    // 59: Get_Cfg
#define SAP_SLAVE_DIAG  0x3C    // 60: Slave_Diag
#define SAP_SET_PRM     0x3D    // 61: Set_Prm
#define SAP_CHK_CFG     0x3E    // 62: Chk_Cfg

#define SAP_DATA_EXCH   0x3E    // 62: Default cyclic Data_Exchange

/*
 * Note: Data_Exchange uses the same SAP number (0x3E / 62) as Chk_Cfg, but
 * Data_Exchange frames have NO SAP extension bytes in the frame — the SAP bit
 * on DA/SA is not set. The dispatch in ProcessFunction distinguishes them by
 * checking has_sap first: no SAP extension → Data_Exchange.
 */

/* ------------------------------------------------------------------ */
/* Helper macro — append one byte to a response buffer               */
/* ------------------------------------------------------------------ */
#define BUILD_RESPONSE(struc, val)  ((struc)->Data[(struc)->Length++] = (val))

/* ------------------------------------------------------------------ */
/* Diagnostic status bytes (Protocol.c builds these from slave state) */
/* ------------------------------------------------------------------ */
/*
 * Diag byte 1 (Status 1):
 *  bit0 Station_Not_Existent  (slave sets 0)
 *  bit1 Station_Not_Ready     (1 = not in Data_Exchange)
 *  bit2 Cfg_Fault             (1 = Chk_Cfg mismatch)
 *  bit3 Ext_Diag              (1 = extended diag follows)
 *  bit4 Not_Supported
 *  bit5 Invalid_Slave_Resp    (master sets; slave always 0)
 *  bit6 Prm_Fault             (1 = Set_Prm rejected)
 *  bit7 Master_Lock           (slave always 0)
 *
 * Diag byte 2 (Status 2):
 *  bit0 Prm_Req               (slave requests re-parametrisation)
 *  bit1 Stat_Diag             (master must keep requesting diag)
 *  bit2 Always 1
 *  bit3 WD_On                 (1 = watchdog active)
 *  bit4 Freeze_Mode
 *  bit5 Sync_Mode
 *  bit6 Reserved (0)
 *  bit7 Deactivated           (slave always 0)
 *
 * Diag byte 3 (Status 3):
 *  bits 0-6 Reserved (0)
 *  bit7 Ext_Diag_Overflow
 */

/* ------------------------------------------------------------------ */
/* DPV1 parameter bytes (in Set_Prm PDU)                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t reserved0           : 1;
    uint8_t reserved1           : 1;
    uint8_t Timebase            : 1;
    uint8_t reserved3           : 1;
    uint8_t reserved4           : 1;
    uint8_t SlaveAsPublisher    : 1;
    uint8_t SlaveInFailsafe     : 1;
    uint8_t SlaveinDPV1Mode     : 1;
} DPV1_Status_1;

typedef struct {
    uint8_t CheckCfgMode            : 1;
    uint8_t reserved1               : 1;
    uint8_t SwitchOnAlarmUpdate     : 1;
    uint8_t SwitchOnStatusAlarm     : 1;
    uint8_t SwitchOnVendorAlarm     : 1;
    uint8_t SwitchOnDiagnosticAlarm : 1;
    uint8_t SwitchOnProcessAlarm    : 1;
    uint8_t SwitchOnPlugAlarm       : 1;
} DPV1_Status_2;

typedef enum {
    MAX_ALARM_1, MAX_ALARM_2, MAX_ALARM_4,  MAX_ALARM_8,
    MAX_ALARM_12,MAX_ALARM_16,MAX_ALARM_24, MAX_ALARM_32
} _MaxAlarms;

typedef struct {
    _MaxAlarms MaxAlarms      : 3;
    uint8_t PRMStructure      : 1;
    uint8_t IsochronousMode   : 1;
    uint8_t reserved5         : 1;
    uint8_t reserved6         : 1;
    uint8_t RedundancyEnabled : 1;
} DPV1_Status_3;

/* ------------------------------------------------------------------ */
/* Parsed message (output of GetMessage)                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t       MasterAddress;  // SA from master (stripped of SAP bit)
    uint8_t       SlaveAddress;   // DA as received (bit7 set = SAP extension)
    TelegramTypes MessageType;
    uint8_t       FunctionCode;
    uint8_t       PDU[250];       // For SAP messages: PDU[0]=DSAP, PDU[1]=SSAP, PDU[2..]=data
    uint8_t       PDULength;
} ProfibusMessage;

/* ------------------------------------------------------------------ */
/* API                                                                */
/* ------------------------------------------------------------------ */

/**
 * Parse raw bytes into a ProfibusMessage.
 * Returns 0 on success, non-zero if frame is invalid/unrecognised.
 */
uint8_t GetMessage(uint8_t *pData, uint32_t Length, ProfibusMessage *Message);

/**
 * Process a parsed message and build a response in pResponse.
 * Returns 0 if a response should be transmitted, non-zero if no response.
 */
uint8_t ProcessFunction(ProfibusMessage Message, profibusSlave *pSlave, resp *pResponse);

#endif // PROFIBUS_PROTOCOL_H
