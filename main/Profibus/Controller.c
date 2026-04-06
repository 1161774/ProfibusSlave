#include "Controller.h"

Node* ProfibusSlaves;
resp Response;

uint8_t InitialiseController()
{
    initializeList(&ProfibusSlaves);
    Response.Data = (uint8_t *)malloc(MAX_RESPONSE);
    if (!Response.Data) {
        ESP_LOGE(TAG_PROFIBUSCONTROLLER, "Failed to allocate response buffer");
        return 1;
    }
    return 0;
}

void ProcessMessage(uint8_t *pMessageData, uint32_t Length)
{
    ProfibusMessage message;
    memset(message.PDU, 0, sizeof(message.PDU));

    if (GetMessage(pMessageData, Length, &message) != 0) return;

    /* Ignore token frames */
    if (message.MessageType == TELEGRAM_SD4) return;

    /* Determine target address (strip SAP bit) */
    uint8_t targetAddr = message.SlaveAddress & ~SAP_BIT;
    bool isBroadcast   = (targetAddr == 0x7F || targetAddr == 0xFF);

    void *slaveData = NULL;
    bool isRegistered = isInList(ProfibusSlaves, targetAddr, &slaveData);

    if (!isBroadcast && !isRegistered) return;

    if (isBroadcast) {
        /*
         * Broadcast (Global_Control etc.) — deliver to all registered slaves.
         * No response expected for broadcast services.
         */
        Node *cur = ProfibusSlaves;
        while (cur) {
            profibusSlave *slave = (profibusSlave *)cur->data;
            memset(Response.Data, 0, MAX_RESPONSE);
            Response.Length = 0;
            ProcessFunction(message, slave, &Response);
            /* No response transmitted for broadcasts */
            cur = cur->next;
        }
        return;
    }

    /* Unicast — message addressed to a specific registered slave */
    profibusSlave *slave = (profibusSlave *)slaveData;

    memset(Response.Data, 0, MAX_RESPONSE);
    Response.Length = 0;

    uint8_t send_response = (ProcessFunction(message, slave, &Response) == 0);

    if (send_response && Response.Length > 0) {
        ESP_LOG_BUFFER_HEXDUMP("TX", Response.Data, Response.Length, ESP_LOG_DEBUG);
        xQueueSend(txQueue, &Response, pdMS_TO_TICKS(5));
    }
}

void AddSlave(uint8_t slaveAddress, profibusSlave *slave)
{
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Registering slave addr=%u ident=%02X%02X",
             slaveAddress, slave->Config.ID_HIGH, slave->Config.ID_LOW);
    addToList(&ProfibusSlaves, slaveAddress, slave);
}
