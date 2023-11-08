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

    ProfibusMessage message;
    memset(message.PDU, 0, sizeof(message.PDU));

    GetMessage(pMessageData, Length, &message);


    // We don't care about SD4 telegrams - we're a slave and have no need to track the token
    if (message.MessageType == TELEGRAM_SD4) return;

    // Check to see if we're a target recipient - broadcast address or registered slave
    // No need to continue if not addressed to us
    bool isBroadcastAddress = (0x80 & message.SlaveAddress) > 0;
    bool isRegisteredAddress = isInList(ProfibusSlaves, (0x7F & message.SlaveAddress), &slaveData);

    if( !isBroadcastAddress && !isRegisteredAddress ) return;

 //   ESP_LOG_BUFFER_HEXDUMP("Received", pMessageData, Length, ESP_LOG_INFO);


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

    permitResponse = ProcessFunction(message, slave, &Response) == 0;

    //respond
    if(permitResponse)
    {
        xQueueSend(txQueue, &Response, portMAX_DELAY);
    }

    // free the pointer we created if needed
    if(slaveData == NULL) free(slave);
}

void AddSlave(uint8_t slaveAddress, profibusSlave* slave)
{
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Adding slave %d", slaveAddress);

    addToList(&ProfibusSlaves, slaveAddress, slave);
}
