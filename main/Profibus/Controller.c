#include "Controller.h"


Node* ProfibusSlaves;

resp Response;

uint8_t InitialiseController()
{
    initializeList(&ProfibusSlaves);
    Response.Data = (uint8_t*)malloc(MAX_RESPONSE);

    return 0;
}

void ProcessMessage(uint8_t *pMessageData, uint32_t Length)
{
    void* slaveData = NULL;
    profibusSlave* slave = NULL;

    uint8_t messageType, destinationAddress, sourceAddress;

    GetMessageType(pMessageData, Length, &messageType, &sourceAddress, &destinationAddress);

    // We don't care about SD4 telegrams - we're a slave and have no need to track the token
    if (messageType == TELEGRAM_TYPE_SD4) return;

    // Check to see if we're a target recipient - broadcast address or registered slave
    // No need to continue if not addressed to us
    if( (0x7f & destinationAddress) != 0x7f &&
        !isInList(ProfibusSlaves, (0x7F & destinationAddress), &slaveData)
        ) return;

    ESP_LOG_BUFFER_HEXDUMP("Received", pMessageData, Length, ESP_LOG_INFO);

    // Get the details of the slave being requested
    if(slaveData == NULL)
    {
        slave = (profibusSlave *)calloc(1, sizeof(profibusSlave));
        if(slave == NULL){ESP_LOGW(TAG_PROFIBUSCONTROLLER, "Error allocating memory"); return;}
    }
    else
    {
        slave = (profibusSlave*)slaveData;
    }

    // prepare a response...
    uint8_t permitResponse = 0;
    memset(Response.Data, 0, MAX_RESPONSE);

    switch (messageType)
    {
    case TELEGRAM_TYPE_SD1:
        permitResponse = ProcessTelegramSD1(pMessageData, &slave->state, &Response) == 0 ? 1 : 0;
        break;
    
    case TELEGRAM_TYPE_SD2:
        // 68 09 09 68 ff 81 44 34  36 5d 00 84 00 0f 16
        // 68 09 09 68 ff 81 44 34  36 5d 00 68 00 f3 16
        break;

    case TELEGRAM_TYPE_SD3:
        break;

    default:
        break;
    }

    //respond
    if(permitResponse)
    {
        xQueueSend(txQueue, &Response, portMAX_DELAY);
    }

    // free the pointer we created if needed
    if(slaveData == NULL) free(slave);
}

void AddSlave(uint8_t slaveAddress, profibusSlave slave)
{
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Adding slave %d", slaveAddress);

    addToList(&ProfibusSlaves, slaveAddress, &slave);
}