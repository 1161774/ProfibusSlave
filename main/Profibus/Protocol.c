#include "Protocol.h"


uint8_t checksum(uint8_t *data, uint32_t length)
{
  uint8_t checksum = 0;

  while(length--)
  {
    checksum += data[length];
  }

  return checksum;
}

uint8_t GetMessageType(uint8_t *Data, uint32_t Length, uint8_t *MessageType, uint8_t *SourceAddress, uint8_t *DestinationAddress)
{
  // Ensure there's enough data to read
  if (Length < 3) return -1;
   
   *MessageType = Data[0];
  switch (*MessageType)
  {
  case TELEGRAM_TYPE_SD1:
  case TELEGRAM_TYPE_SD3:
  case TELEGRAM_TYPE_SD4:
    *DestinationAddress = Data[1];
    *SourceAddress = Data[2];
    return 0;

  case TELEGRAM_TYPE_SD2:
    *DestinationAddress = Data[4]; 
    *SourceAddress = Data[5]; 
    return 0;

  default:
    return 1;
  }
}


uint8_t ProcessTelegramSD1(uint8_t code, profibusSlaveState* state, uint8_t* response)
{
  *response = 0;
  if(code == 0x49) 
  {
    *response = 0x00;
    return 0;
  }

  return 1;
}




/*uint8_t addressmatch (uint8_t destination)
{
  if ((destination != slave_addr) &&                // Slave
      (destination != slave_addr + SAP_OFFSET) &&   // SAP Slave
      (destination != BROADCAST_ADD) &&             // Broadcast
      (destination != BROADCAST_ADD + SAP_OFFSET)){  // SAP Broadcast
        return false;
      }
  return true;
}*/