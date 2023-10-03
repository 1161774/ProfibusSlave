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

uint8_t GetMessageType(uint8_t* pData, uint32_t Length, uint8_t* pMessageType, uint8_t* pSourceAddress, uint8_t* pDestinationAddress)
{

  // Ensure there's enough data to read
  if (Length < 3) return -1;
   
   uint8_t checksum;
  *pDestinationAddress = 0;
  *pSourceAddress = 0;

   *pMessageType = pData[0];
  switch (*pMessageType)
  {
  case TELEGRAM_TYPE_SD1:
    checksum = pData[4];
    if (checksum != GetChecksum(&pData[1], 3))
    {
      ESP_LOGW(TAG_PROTOCOL, "Checksum Mismatch");
      return 1;
    }

    *pDestinationAddress = pData[1];
    *pSourceAddress = pData[2];
    return 0;

  case TELEGRAM_TYPE_SD2:

    uint8_t payloadLength = pData[1];
    *pDestinationAddress = pData[4];
    *pSourceAddress = pData[5];
    
    checksum = pData[4 + payloadLength];

    if (checksum != GetChecksum(&pData[4], payloadLength))
    {
      ESP_LOGW(TAG_PROTOCOL, "Checksum Mismatch");
      return 1;
    }

    return 0;

  case TELEGRAM_TYPE_SD3:
    ESP_LOGE(TAG_PROTOCOL, "SD3 NOT YET IMPLEMENTED");
    ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, pData, Length, ESP_LOG_WARN);
    return 1;

  case TELEGRAM_TYPE_SD4:

    *pDestinationAddress = pData[1];
    *pSourceAddress = pData[2];
    return 0;

  default:
    return 1;
  }
}


uint8_t ProcessTelegramSD1(uint8_t* pMessage, profibusSlaveState* pState, resp* pResponse)
{
  uint8_t messageType = pMessage[0];
  uint8_t slaveAddr = pMessage[1];
  uint8_t masterAddr = pMessage[2];
  uint8_t functionCode = pMessage[3];
  pResponse->Length = 0;

  if(functionCode == 0x49) 
  {
    pResponse->Data[pResponse->Length++] = messageType;
    pResponse->Data[pResponse->Length++] = masterAddr;
    pResponse->Data[pResponse->Length++] = slaveAddr;
    pResponse->Data[pResponse->Length++] = 0x00;
    pResponse->Data[pResponse->Length++] = GetChecksum(&pResponse->Data[1], 3);
    pResponse->Data[pResponse->Length++] = TELEGRAM_ED;

    *pState = SS_WPRM;
    return 0;

  }

  return 1;
}




