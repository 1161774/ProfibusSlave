#include "Controller.h"


Node* ProfibusSlaves;


uint8_t InitialiseController()
{
    initializeList(&ProfibusSlaves);
    return 0;
}

void ProcessMessage(uint8_t *MessageData, uint32_t Length)
{
    void* slaveData = NULL;
    profibusSlave* slave = NULL;

    uint8_t messageType, destinationAddress, sourceAddress;

    GetMessageType(MessageData, Length, &messageType, &sourceAddress, &destinationAddress);

    // We don't care about SD4 telegrams - we're a slave and have no need to track the token
    if (messageType == TELEGRAM_TYPE_SD4) return;

    // Master is asking about a slave we don't know about
    // TODO SAP Offset, Broadcast address
    if(!isInList(ProfibusSlaves, destinationAddress, &slaveData)) return;

    // Get details about the slave 
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Found address %d", destinationAddress);

    // Get the details of the slave being requested
    slave = (profibusSlave*)slaveData;

    switch (messageType)
    {
    case TELEGRAM_TYPE_SD1:

        uint8_t response = 0;
        ProcessTelegramSD1(MessageData, &slave->state, &response);

        
        break;
    
    default:
        return;
    }






}

void AddSlave(uint8_t slaveAddress, profibusSlave slave)
{
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Adding slave %d", slaveAddress);

    addToList(&ProfibusSlaves, slaveAddress, &slave);
}