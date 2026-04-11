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

/*
 * Classify whether a broadcast-addressed SAP service expects individual
 * slave responses.
 *
 * Most broadcasts (Global_Control, Set_Slave_Add) are fire-and-forget —
 * no response. But some are discovery probes sent to DA=0xFF where each
 * slave that recognises itself should respond individually:
 *
 *   Get_Cfg   (0x3A) — master probing what config each slave expects
 *   Slave_Diag (0x3C) — master collecting diagnostics from all slaves
 *   FDL_Status (FC=0x09, no SAP) — bus presence check
 *
 * For these, the broadcast path must still transmit the response built
 * by ProcessFunction.
 */
static bool broadcast_expects_response(const ProfibusMessage *msg)
{
    uint8_t fc_low = msg->FunctionCode & 0x0F;

    /* FDL Status (no SAP extension) always responds */
    if (fc_low == 0x09) return true;

    /* For SAP-extended frames, check DSAP */
    if (msg->SlaveAddress & SAP_BIT) {
        if (msg->PDULength < 1) return false;
        uint8_t dsap = msg->PDU[0] & ~SAP_BIT;
        switch (dsap) {
            case 0x3A:  /* Get_Cfg   */
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
        ESP_LOGE(TAG_PROFIBUSCONTROLLER, "Corrupt Frame");
        ESP_LOG_BUFFER_HEXDUMP(TAG_PROFIBUSCONTROLLER, pMessageData, Length, ESP_LOG_ERROR);
        return;
    }

    /* Ignore token frames */
    if (message.MessageType == TELEGRAM_SD4) return;


    if (message.MessageType == TELEGRAM_SD1) return;

    ESP_LOG_BUFFER_HEXDUMP("RX", pMessageData, Length, ESP_LOG_INFO);



    /* Determine target address (strip SAP bit) */
    uint8_t targetAddr = message.SlaveAddress & ~SAP_BIT;
    bool isBroadcast   = (targetAddr == 0x7F || targetAddr == 0xFF);

    void *slaveData = NULL;
    bool isRegistered = isInList(ProfibusSlaves, targetAddr, &slaveData);

    
    //Message isn't for us.
    if (!isBroadcast && !isRegistered) {
//        ESP_LOGD(TAG_PROFIBUSCONTROLLER, "Not addressed to us (addr=%u (0x%02X))", targetAddr, targetAddr);
        return;
    }

    if (isBroadcast) {
        /*
         * Broadcast — deliver to every registered slave.
         *
         * Most broadcast services (Global_Control, Set_Slave_Add) require
         * no response. A few discovery services (Get_Cfg, Slave_Diag) are
         * addressed to 0xFF but still expect each individual slave to reply
         * with its own address in the SA field.
         */
        bool respond = broadcast_expects_response(&message);

        Node *cur = ProfibusSlaves;
        while (cur) {
            profibusSlave *slave = (profibusSlave *)cur->data;
            memset(Response.Data, 0, MAX_RESPONSE);
            Response.Length = 0;

            uint8_t send = (ProcessFunction(message, slave, &Response) == 0);

            if (respond && send && Response.Length > 0) {
		        ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Sent %u bytes", Response.Length);
                xQueueSend(txQueue, &Response, pdMS_TO_TICKS(5));
            }
            else {
                ESP_LOGW(TAG_PROFIBUSCONTROLLER, "Didn't send %u bytes", Response.Length);
            }

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
        ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Sent %u bytes", Response.Length);
        xQueueSend(txQueue, &Response, pdMS_TO_TICKS(5));
    }
    else {
        ESP_LOGW(TAG_PROFIBUSCONTROLLER, "Didn't send %u bytes", Response.Length);
    }
}

void AddSlave(uint8_t slaveAddress, profibusSlave *slave)
{
    ESP_LOGI(TAG_PROFIBUSCONTROLLER, "Registering slave addr=%u ident=%02X%02X",
             slaveAddress, slave->Config.ID_HIGH, slave->Config.ID_LOW);
    addToList(&ProfibusSlaves, slaveAddress, slave);
}
