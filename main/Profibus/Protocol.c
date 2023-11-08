#include "Protocol.h"
#include "Slave.h"


uint8_t GetChecksum(uint8_t* pData, uint32_t length)
{
  uint8_t checksum = 0;

  while(length--)
  {
    checksum += pData[length];
  }

  return checksum;
}

/*
page 250 

Status.0 - Station Not Existent - Slave always sets 0
Status.1 - Station Not Ready - Slave not in data exchange mode
Status.2 - Config Fault - Configuration from master does not match slave config
Status.3 - Extended Diagnostics - Indicates additional diagnostics will follow starting from byte 7
Status.4 - Not Supported - Slave doesn't support this request
Status.5 - Invalid slave response - Set from the master. Slave always sets zero
Status.6 - Param Fault - Fault in parameter telegram
Status.7 - Master Lock - Slave always sets zero.

*/

uint8_t GetDiagnosticStatus1(profibusSlaveReadyState state)
{
  uint8_t status = 0;

  status = (0)                  << 0 | // always 0
           !STATE_DXCG(state)   << 1 | 
           (0)                  << 2 | // TODO Config fault
           (0)                  << 3 | // TODO Extended diagnostics
           (0)                  << 4 | // TODO Request not supported
           (0)                  << 5 | // always 0
           (0)                  << 6 | // TODO Parameter fault
           (0)                  << 7;   // always 0

  return status;
}

/*
page 251 

Status.0 - Prm_Req - Slave request parameters to be resent
Status.1 - Stat_Diag - Slave request master to continue requesting status
Status.2 - ALWAYS 1 - Error check
Status.3 - Watchdog is on
Status.4 - Freeze mode
Status.5 - Sync mode
Status.6 - reserved
Status.7 - Deactivated - slave always sets zero

*/
uint8_t GetDiagnosticStatus2(profibusSlaveState State)
{
  uint8_t status = 0;

  status = (0)                  << 0 | // the example sets this bit for some reason
           (0)                  << 1 | // 
           (1)                  << 2 | // Always 1
           (0)                  << 3 | // TODO Watchdog
           (State.Frozen)       << 4 | // TODO Freeze mode
           (State.Sync)         << 5 | // TODO Sync mode
           (0)                  << 6 | // Always 0 (reserved)
           (0)                  << 7;  // Always 0

  return status;
}

/*
page 253ish

Status.0 - reserved
Status.1 - reserved
Status.2 - reserved
Status.3 - reserved
Status.4 - reserved
Status.5 - reserved
Status.6 - reserved
Status.7 - Extended diagnostics overflow - slave needs master to continue to requesting diagnostics

*/
uint8_t GetDiagnosticStatus3()
{
  uint8_t status = 0;

  status = (0)                  << 0 || // Always 0 (reserved)
           (0)                  << 1 || // Always 0 (reserved)
           (0)                  << 2 || // Always 0 (reserved)
           (0)                  << 3 || // Always 0 (reserved)
           (0)                  << 4 || // Always 0 (reserved)
           (0)                  << 5 || // Always 0 (reserved)
           (0)                  << 6 || // Always 0 (reserved)
           (0)                  << 7;   // TODO Ext_Diag_Overflow

  return status;
}


uint8_t GetMessage(uint8_t* pData, uint32_t Length,  ProfibusMessage* Message)
{

  // Ensure there's enough data to read
  if (Length < 3) return 1;
   
  uint8_t checksum, csCalc;
  Message->MasterAddress = 0;
  Message->SlaveAddress = 0;

  Message->MessageType = (TelegramTypes)pData[0];
  switch (Message->MessageType)
  {
  case TELEGRAM_SD1:
    checksum = pData[4];
    csCalc = GetChecksum(&pData[1], 3);

    if (checksum != csCalc)
    {
      ESP_LOGW(TAG_PROTOCOL, "Checksum Mismatch - expected %d | got %d", checksum, csCalc);
      return 1;
    }

    Message->SlaveAddress = pData[1];
    Message->MasterAddress = pData[2];
    Message->FunctionCode = pData[3];
    Message->PDULength = 0;

    return 0;

  case TELEGRAM_SD2:

    uint8_t payloadLength = pData[1];
    checksum = pData[4 + payloadLength];
    csCalc = GetChecksum(&pData[4], payloadLength);

    if (checksum != csCalc)
    {
      ESP_LOGW(TAG_PROTOCOL, "Checksum Mismatch - expected %d | got %d", checksum, csCalc);
      return 1;
    }

    Message->SlaveAddress = pData[4];
    Message->MasterAddress = pData[5];
    Message->FunctionCode = pData[6];
    Message->PDULength = payloadLength - 3;
    memcpy(&Message->PDU, &pData[7], Message->PDULength);
    return 0;

  case TELEGRAM_SD3:
    
    checksum = pData[5];
    csCalc = GetChecksum(&pData[1], 4);

    if (checksum != csCalc)
    {
      ESP_LOGW(TAG_PROTOCOL, "Checksum Mismatch - expected %d | got %d", checksum, csCalc);
      return 1;
    }

    Message->SlaveAddress = pData[1];
    Message->MasterAddress = pData[2];
    Message->FunctionCode = pData[3];
    Message->PDU[0] = pData[4];
    Message->PDULength = 1;
    return 0;

  case TELEGRAM_SD4:

    Message->SlaveAddress = pData[1];
    Message->MasterAddress = pData[2];
    Message->PDULength = 0;
    return 0;

  default:
    return 1;
  }
}

uint8_t ProcessFunction(ProfibusMessage Message, profibusSlave* pSlave, resp* pResponse){

  uint8_t DSAP, SSAP, payloadLength;
  pResponse->Length = 0;

  switch (Message.FunctionCode & 0x0F)
  {
       
    // 0x00, Time Event
    case FC_TE:                  
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x03, Send Data, Request Acknowledge, Low Priority
    case FC_SEND_DATA_ACK_LOW:   
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x04, Send Data, No Acknowledge, Low Priority
    case FC_SEND_DATA_NACK_LOW:  
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x05, Send Data, Request Acknowledge, High Priority
    case FC_SEND_DATA_ACK_HIG:   
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x06, Send Data, No Acknowledge, High Priority
    case FC_SEND_DATA_NACK_HIG:  

      DSAP = Message.PDU[0];  // Destination Service Access Point
      SSAP = Message.PDU[1];  // Source Service Access Point

      switch (DSAP)
      {
      case 0x3A:
        
        uint8_t control = Message.PDU[2];
        uint8_t group   = Message.PDU[3];

        // TODO: control & 0x02 => Clear Data, Page 261

        if(control & 0x08) pSlave->State.Frozen = 1;
        if(control & 0x04) pSlave->State.Frozen = 0;
        
        if(control & 0x20) pSlave->State.Sync = 1;
        if(control & 0x10) pSlave->State.Sync = 0;

        pSlave->State.Group = group;
        
        break;

      default:
        ESP_LOGW(TAG_PROTOCOL, "DSAP %x (%d) from message %x not yet implemented", DSAP, DSAP, (uint8_t)Message.FunctionCode);
        break;
      }

      return 1; // not an error, no response required

    break;

    // 0x07, Send Request Data with Multicast Reply
    case FC_MSRD:                
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x08, Clock Value
    case FC_CV:                  
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x09, Fieldbus Data Later Status
    case FC_FDL_STATUS:

      pResponse->Data[pResponse->Length++] = Message.MessageType;
      pResponse->Data[pResponse->Length++] = Message.MasterAddress;
      pResponse->Data[pResponse->Length++] = Message.SlaveAddress;
      pResponse->Data[pResponse->Length++] = 0x00;
      pResponse->Data[pResponse->Length++] = GetChecksum(&pResponse->Data[1], 3);
      pResponse->Data[pResponse->Length++] = TELEGRAM_ED;

      return 0;

    break;

    // 0x0C, Send and Request Data, Low Priority
    case FC_SRD_LOW:             
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;

    // 0x0D, Send and Request Data, High Priority
    case FC_SRD_HIGH:            

      DSAP = Message.PDU[0];  // Destination Service Access Point
      SSAP = Message.PDU[1];  // Source Service Access Point

      switch (DSAP)
      {
      case 0x3B & 0x00:
  
/*        pResponse->Data[pResponse->Length++] = TELEGRAM_SD2;  
        pResponse->Data[pResponse->Length++] = 0x00;          // length is empty for now, populate later
        pResponse->Data[pResponse->Length++] = 0x00;          // length is empty for now, populate later
        pResponse->Data[pResponse->Length++] = TELEGRAM_SD2;
        pResponse->Data[pResponse->Length++] = Message.MasterAddress;
        pResponse->Data[pResponse->Length++] = pSlave->Config.Address | 0x80;
        pResponse->Data[pResponse->Length++] = 0x08; // Function code response (typical response)                        
        pResponse->Data[pResponse->Length++] = SSAP;  
        pResponse->Data[pResponse->Length++] = DSAP;  

        // TODO - fill in these bits

        payloadLength = pResponse->Length - 4;
        pResponse->Data[1] = payloadLength;
        pResponse->Data[2] = payloadLength;
        pResponse->Data[pResponse->Length++] = GetChecksum(&pResponse->Data[4], payloadLength);
        pResponse->Data[pResponse->Length++] = TELEGRAM_ED;
*/
        BUILD_RESPONSE(pResponse, TELEGRAM_SC);

        return 0; // return 0 when happy to respond


      case 0x3C:
        // Update state
        if(pSlave->State.ReadyState == SS_POWERON) pSlave->State.ReadyState = SS_WPRM;

        pResponse->Data[pResponse->Length++] = TELEGRAM_SD2;  
        pResponse->Data[pResponse->Length++] = 0x00;          // length is empty for now, populate later
        pResponse->Data[pResponse->Length++] = 0x00;          // length is empty for now, populate later
        pResponse->Data[pResponse->Length++] = TELEGRAM_SD2;
        pResponse->Data[pResponse->Length++] = Message.MasterAddress;
        pResponse->Data[pResponse->Length++] = pSlave->Config.Address | 0x80;
        pResponse->Data[pResponse->Length++] = 0x08; // Function code response (typical response)                        
        pResponse->Data[pResponse->Length++] = SSAP;  
        pResponse->Data[pResponse->Length++] = DSAP;  
        pResponse->Data[pResponse->Length++] = GetDiagnosticStatus1(pSlave->State.ReadyState);  
        pResponse->Data[pResponse->Length++] = GetDiagnosticStatus2(pSlave->State);  
        pResponse->Data[pResponse->Length++] = GetDiagnosticStatus3();  
        pResponse->Data[pResponse->Length++] = 0xff;    // TODO Address of master exercising cyclic control over slave (default 0xFF)
        pResponse->Data[pResponse->Length++] = pSlave->Config.ID_HIGH;
        pResponse->Data[pResponse->Length++] = pSlave->Config.ID_LOW;
        if(pResponse->Data[9] & 0x08)
        {
          /* TODO Extended diagnostics */
        }

        payloadLength = pResponse->Length - 4;
        pResponse->Data[1] = payloadLength;
        pResponse->Data[2] = payloadLength;
        pResponse->Data[pResponse->Length++] = GetChecksum(&pResponse->Data[4], payloadLength);
        pResponse->Data[pResponse->Length++] = TELEGRAM_ED;

        return 0; 
      
      case 0x3D & 0x01:

        // page 225

        uint8_t stationStatus, minTSDR, group;
        uint8_t watchdog1, watchdog2;
        uint8_t identHigh, identLow;
        uint8_t DPV1Status_1, DPV1Status_2, DPV1Status_3;


        stationStatus = Message.PDU[2];
        watchdog1     = Message.PDU[3];
        watchdog2     = Message.PDU[4];
        minTSDR       = Message.PDU[5]; // don't care about this
        identHigh     = Message.PDU[6];
        identLow      = Message.PDU[7];
        group         = Message.PDU[8]; // todo - something with this

        if(identHigh != pSlave->Config.ID_HIGH || identLow != pSlave->Config.ID_LOW)
        {
          ESP_LOGW(TAG_PROTOCOL, "Ident Mismatch. Expected %x%x, received %x%x", pSlave->Config.ID_HIGH, pSlave->Config.ID_LOW, identHigh, identLow);
          return 1;
        }

        pSlave->Control.WatchdogStatus = (stationStatus & 0x08) > 0;
        pSlave->Control.FreezeReq = (stationStatus & 0x10) > 0;
        pSlave->Control.SyncReq   = (stationStatus & 0x20) > 0;

        if(stationStatus & 0x80) pSlave->Control.LockReq = 1;
        if(stationStatus & 0x40) pSlave->Control.LockReq = 0; // unlock takes priority over lock

        pSlave->Control.WatchdogVal = watchdog1 * watchdog2 * 10;

        pSlave->Control.dpv1Status1 = *(DPV1_Status_1 *)&Message.PDU[9];
        pSlave->Control.dpv1Status2 = *(DPV1_Status_2 *)&Message.PDU[10];
        pSlave->Control.dpv1Status3 = *(DPV1_Status_3 *)&Message.PDU[11];

        ESP_LOGI(TAG_PROTOCOL, "TODO: Confirm these are equal!: %d => %d", Message.PDU[11], *(uint8_t *)&pSlave->Control.dpv1Status3);
        //TODO - continue here!

        return 1; //return 0 when ready to send


      default:
        ESP_LOGW(TAG_PROTOCOL, "DSAP %x (%d) from message %x not yet implemented. PDU:", DSAP, DSAP, (uint8_t)Message.FunctionCode);
        ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, Message.PDU, Message.PDULength, ESP_LOG_WARN);
        return 1;
      }



    break;

    // 0x0E, Request Identity With Reply
    case FC_REQ_ID_REPL:         
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;
    
    // 0x0F, Request LSAP Status With Reply
    case FC_REQ_LSAP_REPL:       
      ESP_LOGW(TAG_PROTOCOL, "Message %x (%d) not yet implemented", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;
  
  default:
        ESP_LOGE(TAG_PROTOCOL, "Unknown message - %x (%d)", (uint8_t)Message.FunctionCode, (uint8_t)Message.FunctionCode);

    break;
  }

  return 1;
}



