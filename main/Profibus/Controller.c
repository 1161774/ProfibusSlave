#include "Controller.h"

Node *ProfibusSlaves;

/*
 * Single response buffer allocated on the stack of ProcessMessage.
 * Because resp now embeds its Data[] array, xQueueSend copies the entire
 * payload into the queue — no shared-pointer race possible.
 */

uint8_t InitialiseController(void)
{
    initializeList(&ProfibusSlaves);
    return 0;
}

/*
 * Classify whether a broadcast-addressed SAP service expects individual
 * slave responses.
 */
static bool broadcast_expects_response(const ProfibusMessage *msg)
{
    uint8_t fc_low = msg->FunctionCode & 0x0F;

    if (fc_low == 0x09) return true;    /* FDL Status */

    if (msg->SlaveAddress & SAP_BIT) {
        if (msg->PDULength < 1) return false;
        uint8_t dsap = msg->PDU[0] & ~SAP_BIT;
        switch (dsap) {
            case 0x3A:  /* Get_Cfg    */
            case 0x3C:  /* Slave_Diag */
                return true;
            default:
                return false;
        }
    }

    return false;
}

void ProcessMessage(uint8_t *pMessageData, uint32_t Length)
{
    ProfibusMessage message;
    memset(message.PDU, 0, sizeof(message.PDU));

    if (GetMessage(pMessageData, Length, &message) != 0) {
        ESP_LOGE(TAG_CONTROLLER, "Corrupt frame (%lu bytes)", (unsigned long)Length);
        ESP_LOG_BUFFER_HEXDUMP(TAG_CONTROLLER, pMessageData, Length, ESP_LOG_ERROR);
        return;
    }

    /* Ignore token frames silently */
    if (message.MessageType == TELEGRAM_SD4) return;
    uint8_t targetAddr = message.SlaveAddress & ~SAP_BIT;
    bool isBroadcast   = (targetAddr == 0x7F || targetAddr == 0xFF);

    void *slaveData   = NULL;
    bool isRegistered = isInList(ProfibusSlaves, targetAddr, &slaveData);

    if (!isBroadcast && !isRegistered) return;

    resp Response;   /* stack-allocated; embedded Data[] is copied into queue */

    if (isBroadcast) {
        bool respond = broadcast_expects_response(&message);
        Node *cur = ProfibusSlaves;
        while (cur) {
            profibusSlave *slave = (profibusSlave *)cur->data;
            memset(Response.Data, 0, sizeof(Response.Data));
            Response.Length = 0;

            uint8_t send = (ProcessFunction(message, slave, &Response) == 0);

            if (respond && send && Response.Length > 0) {
                xQueueSend(txQueue, &Response, pdMS_TO_TICKS(5));
            }
            else {
//                ESP_LOGW(TAG_CONTROLLER, "Didn't send %u bytes", Response.Length);
            }

            cur = cur->next;
        }
        ESP_LOGD(TAG_CONTROLLER, "BC addr=0x%02X (%lu bytes)", targetAddr, (unsigned long)Length);
        return;
    }

    /* Unicast */
    profibusSlave *slave = (profibusSlave *)slaveData;
    memset(Response.Data, 0, sizeof(Response.Data));
    Response.Length = 0;

    uint8_t send_response = (ProcessFunction(message, slave, &Response) == 0);

    if (send_response && Response.Length > 0) {
        /*
         * Queue the response BEFORE any logging.
         * Each ESP_LOGI call spends 1-3 ms writing to the UART console —
         * logging before queuing was adding 15-20 ms of dead time between
         * receiving a request and transmitting the reply.
         */
        xQueueSend(txQueue, &Response, pdMS_TO_TICKS(5));
        ESP_LOGI(TAG_CONTROLLER, "RX %lu B → TX %u B addr=%u",
                 (unsigned long)Length, Response.Length, targetAddr);
    } else {
        ESP_LOGD(TAG_CONTROLLER, "RX %lu B addr=%u (no response)",
                 (unsigned long)Length, targetAddr);
    }
}

void AddSlave(uint8_t slaveAddress, profibusSlave *slave)
{
    ESP_LOGI(TAG_CONTROLLER, "Registered slave addr=%u ident=%02X%02X",
             slaveAddress, slave->Config.ID_HIGH, slave->Config.ID_LOW);
    addToList(&ProfibusSlaves, slaveAddress, slave);
}
