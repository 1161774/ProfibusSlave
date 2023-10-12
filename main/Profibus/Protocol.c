#include "Protocol.h"


uint8_t GetChecksum(uint8_t* pData, uint32_t length)
{
  uint8_t checksum = 0;

  while(length--)
  {
    checksum += pData[length];
  }

  return checksum;
}

uint8_t GetMessage(uint8_t* pData, uint32_t Length,  ProfibusMessage* Message)
{

  // Ensure there's enough data to read
  if (Length < 3) return 1;
   
  uint8_t checksum, csCalc;
  Message->SrcAddress = 0;
  Message->DstAddress = 0;

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

    Message->DstAddress = pData[1];
    Message->SrcAddress = pData[2];
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

    Message->DstAddress = pData[4];
    Message->SrcAddress = pData[5];
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

    Message->DstAddress = pData[1];
    Message->SrcAddress = pData[2];
    Message->FunctionCode = pData[3];
    Message->PDU[0] = pData[4];
    Message->PDULength = 1;
    return 0;

  case TELEGRAM_SD4:

    Message->DstAddress = pData[1];
    Message->SrcAddress = pData[2];
    Message->PDULength = 0;
    return 0;

  default:
    return 1;
  }
}

uint8_t ProcessFunction(ProfibusMessage Message, profibusSlaveState* pState, resp* pResponse){
  //68 05 05 68 8a 81 6d 3c  3e f2 16

  //dest = 8a
  //src = 81
  //fc = 6c
  //pdu = 3c 3e

  pResponse->Length = 0;

  switch (Message.FunctionCode & 0x0F)
  {
       
    // 0x00, Time Event
    case FC_TE:                  
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);
    break;

    // 0x03, Send Data, Request Acknowledge, Low Priority
    case FC_SEND_DATA_ACK_LOW:   
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x04, Send Data, No Acknowledge, Low Priority
    case FC_SEND_DATA_NACK_LOW:  
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x05, Send Data, Request Acknowledge, High Priority
    case FC_SEND_DATA_ACK_HIG:   
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x06, Send Data, No Acknowledge, High Priority
    case FC_SEND_DATA_NACK_HIG:  
//      ESP_LOGI(TAG_PROTOCOL, "fc:%x, len=%d, pdu=%s", Message.FunctionCode, Message.PDULength, Message.PDU);
      ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, Message.PDU, Message.PDULength, ESP_LOG_INFO);
//      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x07, Send Request Data with Multicast Reply
    case FC_MSRD:                
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x08, Clock Value
    case FC_CV:                  
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x09, Fieldbus Data Later Status
    case FC_FDL_STATUS:

      pResponse->Data[pResponse->Length++] = Message.MessageType;
      pResponse->Data[pResponse->Length++] = Message.SrcAddress;
      pResponse->Data[pResponse->Length++] = Message.DstAddress;
      pResponse->Data[pResponse->Length++] = 0x00;
      pResponse->Data[pResponse->Length++] = GetChecksum(&pResponse->Data[1], 3);
      pResponse->Data[pResponse->Length++] = TELEGRAM_ED;

      return 0;

    break;

    // 0x0C, Send and Request Data, Low Priority
    case FC_SRD_LOW:             
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x0D, Send and Request Data, High Priority
    case FC_SRD_HIGH:            
//      ESP_LOGI(TAG_PROTOCOL, "fc:%x, len=%d, pdu=%s", Message.FunctionCode, Message.PDULength, Message.PDU);
      ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, Message.PDU, Message.PDULength, ESP_LOG_INFO);
//      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;

    // 0x0E, Request Identity With Reply
    case FC_REQ_ID_REPL:         
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;
    
    // 0x0F, Request LSAP Status With Reply
    case FC_REQ_LSAP_REPL:       
      ESP_LOGW(TAG_PROTOCOL, "Message %x not yet implemented", (uint8_t)Message.FunctionCode);

    break;
  
  default:
        ESP_LOGE(TAG_PROTOCOL, "Unknown message - %x", (uint8_t)Message.FunctionCode);

    break;
  }

  return 1;
}



