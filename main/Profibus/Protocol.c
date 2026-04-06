/**
 * Protocol.c
 * PROFIBUS-DP slave protocol — frame parsing and SAP dispatch.
 *
 * State machine:
 *   SS_POWERON → SS_WPRM   (automatic on first Slave_Diag)
 *   SS_WPRM   → SS_WCFG    (valid Set_Prm received)
 *   SS_WCFG   → SS_DXCHG   (valid Chk_Cfg received)
 *   Any state → SS_WPRM    (ident mismatch, cfg fault, or explicit re-prm)
 *
 * SAPs handled:
 *   0x32  Set_Prm         FC: SDA (0x03) — slave ACKs with SC
 *   0x33  Chk_Cfg         FC: SDA (0x03) — slave ACKs with SC
 *   0x3C  Slave_Diag      FC: SRD (0x0C/0x0D) — slave responds with diag bytes
 *   0x3D  Global_Control  FC: SDN (0x06) broadcast — no response
 *   0x3E  Data_Exchange   FC: SRD (0x0C/0x0D) — slave responds with input data
 *   No SAP extension → Data_Exchange (default)
 */

#include "Protocol.h"
#include "Slave.h"
#include "esp_timer.h"

/* ------------------------------------------------------------------ */
/* FCS (arithmetic sum, mod 256)                                      */
/* ------------------------------------------------------------------ */

static uint8_t CalcFCS(const uint8_t *pData, uint32_t length)
{
    uint8_t fcs = 0;
    for (uint32_t i = 0; i < length; i++) fcs += pData[i];
    return fcs;
}

/* Keep the old name available for any callers */
uint8_t GetChecksum(uint8_t *pData, uint32_t length)
{
    return CalcFCS(pData, length);
}

/* ------------------------------------------------------------------ */
/* Diagnostic byte builders                                           */
/* ------------------------------------------------------------------ */

static uint8_t DiagStatus1(profibusSlaveReadyState state)
{
    /*
     * bit1: Station_Not_Ready — set when NOT in Data_Exchange
     * All other bits 0 (no config fault, no ext diag, etc.)
     */
    return (state != SS_DXCHG) ? 0x02 : 0x00;
}

static uint8_t DiagStatus2(const profibusSlaveState *s)
{
    /*
     * bit2: Always 1 (mandatory per spec)
     * bit3: Watchdog on
     * bit4: Freeze mode
     * bit5: Sync mode
     */
    return (uint8_t)(
        (1           << 2) |
        (s->Frozen   << 4) |
        (s->Sync     << 5)
    );
}

static uint8_t DiagStatus3(void)
{
    return 0x00;  // No extended diag overflow
}

/* ------------------------------------------------------------------ */
/* Response frame builders                                            */
/* ------------------------------------------------------------------ */

/**
 * Build an SD2 response frame into pResponse.
 *
 * @param masterAddr   Master station address (without SAP bit)
 * @param slaveAddr    Our slave address (without SAP bit)
 * @param fc           Response function code (e.g. FC_RESP_SLAVE_DL)
 * @param dsap         DSAP for response (master's SSAP from request), 0 = no SAP
 * @param ssap         SSAP for response (our SAP), 0 = no SAP
 * @param pdu          Payload bytes (may be NULL if pdu_len=0)
 * @param pdu_len      Number of payload bytes
 */
static void BuildSD2Response(resp *pResponse,
                             uint8_t masterAddr, uint8_t slaveAddr,
                             uint8_t fc,
                             uint8_t dsap, uint8_t ssap,
                             const uint8_t *pdu, uint8_t pdu_len)
{
    pResponse->Length = 0;

    bool has_sap = (dsap != 0 || ssap != 0);

    BUILD_RESPONSE(pResponse, TELEGRAM_SD2);
    BUILD_RESPONSE(pResponse, 0x00);        // LE placeholder
    BUILD_RESPONSE(pResponse, 0x00);        // LEr placeholder
    BUILD_RESPONSE(pResponse, TELEGRAM_SD2);

    /* DA: master address, bit7 set if SAP extension follows */
    BUILD_RESPONSE(pResponse, masterAddr | (has_sap ? SAP_BIT : 0));
    /* SA: our address, bit7 set if SAP extension follows */
    BUILD_RESPONSE(pResponse, slaveAddr  | (has_sap ? SAP_BIT : 0));
    BUILD_RESPONSE(pResponse, fc);

    if (has_sap) {
        BUILD_RESPONSE(pResponse, dsap | SAP_BIT);
        BUILD_RESPONSE(pResponse, ssap | SAP_BIT);
    }

    if (pdu && pdu_len) {
        for (uint8_t i = 0; i < pdu_len; i++) {
            BUILD_RESPONSE(pResponse, pdu[i]);
        }
    }

    /* Fill in lengths (bytes from DA to end of data, index 4 onwards) */
    uint8_t payload_len = (uint8_t)(pResponse->Length - 4);
    pResponse->Data[1] = payload_len;
    pResponse->Data[2] = payload_len;

    /* FCS over bytes from DA (index 4) through payload */
    BUILD_RESPONSE(pResponse, CalcFCS(&pResponse->Data[4], payload_len));
    BUILD_RESPONSE(pResponse, TELEGRAM_ED);
}

/* ------------------------------------------------------------------ */
/* GetMessage — parse raw bytes into ProfibusMessage                  */
/* ------------------------------------------------------------------ */

uint8_t GetMessage(uint8_t *pData, uint32_t Length, ProfibusMessage *Message)
{
    if (Length < 3) return 1;

    Message->MasterAddress = 0;
    Message->SlaveAddress  = 0;
    Message->PDULength     = 0;
    Message->MessageType   = (TelegramTypes)pData[0];

    switch (Message->MessageType)
    {
    case TELEGRAM_SD1:
        /*  SD1 | DA | SA | FC | FCS | ED   (6 bytes) */
        if (Length < 6) return 1;
        if (pData[5] != TELEGRAM_ED) return 1;
        if (CalcFCS(&pData[1], 3) != pData[4]) {
            ESP_LOGW(TAG_PROTOCOL, "SD1 FCS mismatch");
            return 1;
        }
        Message->SlaveAddress  = pData[1];
        Message->MasterAddress = pData[2];
        Message->FunctionCode  = pData[3];
        Message->PDULength     = 0;
        return 0;

    case TELEGRAM_SD2: {
        /*  SD2 | LE | LEr | SD2 | DA | SA | FC | PDU... | FCS | ED */
        if (Length < 6) return 1;
        uint8_t le  = pData[1];
        uint8_t ler = pData[2];
        if (le != ler || pData[3] != TELEGRAM_SD2) return 1;
        if ((uint32_t)(le + 6) > Length) return 1;  /* not enough data yet */
        if (pData[4 + le + 1] != TELEGRAM_ED) return 1;

        uint8_t fcs_rx   = pData[4 + le];
        uint8_t fcs_calc = CalcFCS(&pData[4], le);
        if (fcs_rx != fcs_calc) {
            ESP_LOGW(TAG_PROTOCOL, "SD2 FCS mismatch");
            return 1;
        }

        Message->SlaveAddress  = pData[4];          /* DA — bit7 set = SAP extension */
        Message->MasterAddress = pData[5];          /* SA */
        Message->FunctionCode  = pData[6];          /* FC */
        Message->PDULength     = le - 3;            /* bytes after DA SA FC */
        if (Message->PDULength > sizeof(Message->PDU)) {
            Message->PDULength = sizeof(Message->PDU);
        }
        memcpy(Message->PDU, &pData[7], Message->PDULength);
        return 0;
    }

    case TELEGRAM_SD3:
        /*  SD3 | DA | SA | FC | D0..D7 | FCS | ED  (14 bytes) */
        if (Length < 14) return 1;
        if (pData[13] != TELEGRAM_ED) return 1;
        if (CalcFCS(&pData[1], 11) != pData[12]) {
            ESP_LOGW(TAG_PROTOCOL, "SD3 FCS mismatch");
            return 1;
        }
        Message->SlaveAddress  = pData[1];
        Message->MasterAddress = pData[2];
        Message->FunctionCode  = pData[3];
        Message->PDULength     = 8;
        memcpy(Message->PDU, &pData[4], 8);
        return 0;

    case TELEGRAM_SD4:
        /*  SD4 | DA | SA  (3 bytes, no FCS, no ED) */
        if (Length < 3) return 1;
        Message->SlaveAddress  = pData[1];
        Message->MasterAddress = pData[2];
        Message->PDULength     = 0;
        return 0;

    case TELEGRAM_SC:
        /* Short acknowledge — 1 byte only */
        return 0;

    default:
        return 1;
    }
}

/* ------------------------------------------------------------------ */
/* SAP handlers                                                       */
/* ------------------------------------------------------------------ */

/* Helper: reset slave to WAIT_PRM state */
static void ResetToWaitPrm(profibusSlave *pSlave)
{
    ESP_LOGW(TAG_PROTOCOL, "Slave addr=%u → SS_WPRM", pSlave->Config.Address);
    pSlave->State.ReadyState = SS_WPRM;
    pSlave->prm_len = 0;
    /* Do NOT reset cfg_data if cfg_strict=1 so we re-validate next time */
}

/* ---- Slave_Diag (SAP 0x3C) --------------------------------------- */

static uint8_t HandleSlaveDiag(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    /*
     * Respond with 6 mandatory diagnostic bytes:
     *   [0] Status 1  [1] Status 2  [2] Status 3
     *   [3] Master address (who parameterised us, 0xFF if none)
     *   [4] Ident_Number high
     *   [5] Ident_Number low
     * Extended diag may follow (not implemented here).
     *
     * DSAP in response = SSAP from request (master's SAP)
     * SSAP in response = 0x3C (our Slave_Diag SAP)
     */

    /* Advance from POWERON → WPRM on first diag request */
    if (pSlave->State.ReadyState == SS_POWERON) {
        pSlave->State.ReadyState = SS_WPRM;
    }

    uint8_t masterAddr = msg.MasterAddress & ~SAP_BIT;
    uint8_t ssap_req   = (msg.PDULength >= 2) ? (msg.PDU[1] & ~SAP_BIT) : 0x3E;
    uint8_t dsap_resp  = ssap_req;      /* Echo master's SSAP as our DSAP */
    uint8_t ssap_resp  = SAP_SLAVE_DIAG;

    uint8_t masterExercising = (pSlave->State.ReadyState == SS_DXCHG)
                             ? masterAddr : 0xFF;

    uint8_t diag[6];
    diag[0] = DiagStatus1(pSlave->State.ReadyState);
    diag[1] = DiagStatus2(&pSlave->State);
    diag[2] = DiagStatus3();
    diag[3] = masterExercising;
    diag[4] = pSlave->Config.ID_HIGH;
    diag[5] = pSlave->Config.ID_LOW;

    BuildSD2Response(pResponse,
                     masterAddr, pSlave->Config.Address,
                     FC_RESP_SLAVE_DL,
                     dsap_resp, ssap_resp,
                     diag, sizeof(diag));

    ESP_LOGD(TAG_PROTOCOL, "Slave_Diag → state=%d diag=[%02X %02X %02X]",
             pSlave->State.ReadyState, diag[0], diag[1], diag[2]);
    return 0;  /* Send response */
}

/* ---- Set_Prm (SAP 0x32) ------------------------------------------ */

static uint8_t HandleSetPrm(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    /*
     * PDU layout (after DSAP/SSAP bytes, so PDU[2..]):
     *  [2]   Station_Status  (lock/unlock/WD/freeze/sync bits)
     *  [3]   WD_Fact_1
     *  [4]   WD_Fact_2
     *  [5]   Min_Tsdr (slave ignores)
     *  [6]   Ident_Number high
     *  [7]   Ident_Number low
     *  [8]   Group_Ident
     *  [9..] User_Prm_Data (optional, up to Max_User_Prm_Data_Len bytes)
     *
     * Minimum PDU: 2 (SAP) + 7 (fixed) = 9 bytes.
     * Response: SC (0xE5) on success; Slave_Diag with not-ready on failure.
     */

    if (msg.PDULength < 9) {
        ESP_LOGW(TAG_PROTOCOL, "Set_Prm PDU too short (%u)", msg.PDULength);
        return 1;  /* No response — master will retry */
    }

    uint8_t stationStatus = msg.PDU[2];
    uint8_t wd1           = msg.PDU[3];
    uint8_t wd2           = msg.PDU[4];
    /* msg.PDU[5] = Min_Tsdr — ignored */
    uint8_t identHigh     = msg.PDU[6];
    uint8_t identLow      = msg.PDU[7];
    uint8_t group         = msg.PDU[8];

    /* Validate ident number */
    if (identHigh != pSlave->Config.ID_HIGH || identLow != pSlave->Config.ID_LOW) {
        ESP_LOGW(TAG_PROTOCOL, "Set_Prm ident mismatch: expected %02X%02X, got %02X%02X",
                 pSlave->Config.ID_HIGH, pSlave->Config.ID_LOW, identHigh, identLow);
        ResetToWaitPrm(pSlave);
        return 1;
    }

    /* Station status flags */
    pSlave->Control.WatchdogStatus = (stationStatus & 0x08) != 0;
    pSlave->Control.FreezeReq      = (stationStatus & 0x10) != 0;
    pSlave->Control.SyncReq        = (stationStatus & 0x20) != 0;

    if (stationStatus & 0x80) pSlave->Control.LockReq = 1;
    if (stationStatus & 0x40) pSlave->Control.LockReq = 0; /* Unlock takes priority */

    /* Watchdog timeout = WD_Fact_1 × WD_Fact_2 × 10 ms */
    pSlave->Control.WatchdogVal = (uint32_t)wd1 * (uint32_t)wd2 * 10;

    pSlave->State.Group = group;

    /* DPV1 status bytes (optional — present if PDU long enough) */
    if (msg.PDULength >= 12) {
        pSlave->Control.dpv1Status1 = *(DPV1_Status_1 *)&msg.PDU[9];
        pSlave->Control.dpv1Status2 = *(DPV1_Status_2 *)&msg.PDU[10];
        pSlave->Control.dpv1Status3 = *(DPV1_Status_3 *)&msg.PDU[11];
    }

    /* Store user parameter data (bytes after the 7 fixed + 2 SAP = PDU[9..]) */
    uint8_t user_offset = 9;  /* PDU[0..1]=SAP, [2..8]=fixed prm, [9+]=user */
    if (msg.PDULength > user_offset) {
        pSlave->prm_len = msg.PDULength - user_offset;
        if (pSlave->prm_len > PB_MAX_PRM_LEN) pSlave->prm_len = PB_MAX_PRM_LEN;
        memcpy(pSlave->prm_data, &msg.PDU[user_offset], pSlave->prm_len);
    } else {
        pSlave->prm_len = 0;
    }

    /* Advance state */
    pSlave->State.ReadyState = SS_WCFG;

    ESP_LOGI(TAG_PROTOCOL, "Set_Prm OK: addr=%u WD=%lums → SS_WCFG",
             pSlave->Config.Address, (unsigned long)pSlave->Control.WatchdogVal);

    /* Respond with Short Acknowledge */
    BUILD_RESPONSE(pResponse, TELEGRAM_SC);
    return 0;
}

/* ---- Chk_Cfg (SAP 0x33) ------------------------------------------ */

static uint8_t HandleChkCfg(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    /*
     * PDU layout: DSAP | SSAP | cfg_byte_0 | cfg_byte_1 | ...
     * Cfg bytes describe the I/O modules.  For KFC750 PPO5: single byte 0xFF.
     *
     * 0xFF = 1111 1111
     *   bits[7:6] = 11 → I/O (input + output)
     *   bit5      =  1 → word consistency
     *   bits[3:0] = 0xF → length-1 = 15, so 16 bytes each direction
     *
     * If cfg_strict=0 (default): accept any cfg and store it.
     * If cfg_strict=1: validate against pre-loaded cfg_data.
     *
     * Response: SC on success, no response on failure (master re-prms us).
     */

    if (pSlave->State.ReadyState != SS_WCFG) {
        ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg received in wrong state (%d)", pSlave->State.ReadyState);
        ResetToWaitPrm(pSlave);
        return 1;
    }

    /* PDU starts with DSAP, SSAP, then cfg bytes */
    if (msg.PDULength < 3) {
        ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg PDU too short (%u)", msg.PDULength);
        ResetToWaitPrm(pSlave);
        return 1;
    }

    uint8_t  cfg_offset = 2;   /* skip DSAP + SSAP */
    uint8_t  cfg_len    = msg.PDULength - cfg_offset;
    uint8_t *cfg_bytes  = &msg.PDU[cfg_offset];

    if (pSlave->cfg_strict && pSlave->cfg_len > 0) {
        /* Strict validation */
        if (cfg_len != pSlave->cfg_len ||
            memcmp(cfg_bytes, pSlave->cfg_data, cfg_len) != 0) {
            ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg mismatch (got %u bytes)", cfg_len);
            ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, cfg_bytes, cfg_len, ESP_LOG_WARN);
            ResetToWaitPrm(pSlave);
            return 1;
        }
    } else {
        /* Permissive: accept and store */
        if (cfg_len > PB_MAX_CFG_LEN) cfg_len = PB_MAX_CFG_LEN;
        memcpy(pSlave->cfg_data, cfg_bytes, cfg_len);
        pSlave->cfg_len = cfg_len;

        /* Derive I/O lengths from cfg bytes if not already set by application */
        if (pSlave->output.len == 0 && pSlave->input.len == 0) {
            uint8_t in_len = 0, out_len = 0;
            for (uint8_t i = 0; i < cfg_len; i++) {
                uint8_t b    = cfg_bytes[i];
                uint8_t type = (b >> 6) & 0x03;
                uint8_t len  = (b & 0x0F) + 1;
                if (type == 0x01) in_len  += len;         /* input only  */
                else if (type == 0x02) out_len += len;    /* output only */
                else if (type == 0x03) { in_len += len; out_len += len; } /* I/O */
            }
            pSlave->input.len  = (in_len  > PB_MAX_IO_LEN) ? PB_MAX_IO_LEN : in_len;
            pSlave->output.len = (out_len > PB_MAX_IO_LEN) ? PB_MAX_IO_LEN : out_len;
            ESP_LOGI(TAG_PROTOCOL, "Chk_Cfg: derived I=%u O=%u bytes from cfg",
                     pSlave->input.len, pSlave->output.len);
        }
    }

    pSlave->State.ReadyState = SS_DXCHG;
    pSlave->last_rx_time_ms  = esp_timer_get_time() / 1000LL;

    ESP_LOGI(TAG_PROTOCOL, "Chk_Cfg OK (%u cfg bytes) → SS_DXCHG  I=%u O=%u",
             cfg_len, pSlave->input.len, pSlave->output.len);

    BUILD_RESPONSE(pResponse, TELEGRAM_SC);
    return 0;
}

/* ---- Data_Exchange (SAP 0x3E, or no SAP) ------------------------- */

static uint8_t HandleDataExchange(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    /*
     * Master sends output data (control word, setpoints).
     * Slave responds with input data (status word, actual values).
     *
     * If slave is not in SS_DXCHG, respond with Slave_Diag instead
     * (master will then re-parametrise us).
     *
     * For SAP-addressed exchange: PDU[0]=DSAP, PDU[1]=SSAP, PDU[2..]=outputs
     * For default exchange (no SAP): PDU[0..]=outputs
     */

    if (pSlave->State.ReadyState != SS_DXCHG) {
        /* Not ready — master needs to re-parametrise. Send diag. */
        ESP_LOGD(TAG_PROTOCOL, "Data_Exchange while not in DXCHG (state=%d)", pSlave->State.ReadyState);
        return HandleSlaveDiag(msg, pSlave, pResponse);
    }

    /* Update watchdog timestamp */
    pSlave->last_rx_time_ms = esp_timer_get_time() / 1000LL;

    /* Extract output data */
    bool    has_sap    = (msg.SlaveAddress & SAP_BIT) != 0;
    uint8_t data_offset = has_sap ? 2 : 0;

    if (msg.PDULength > data_offset) {
        uint8_t out_len = msg.PDULength - data_offset;
        if (out_len > PB_MAX_IO_LEN) out_len = PB_MAX_IO_LEN;
        memcpy(pSlave->output.data, &msg.PDU[data_offset], out_len);
        pSlave->output.len = out_len;
    }

    /* Build response with input data */
    if (has_sap) {
        uint8_t ssap_req  = msg.PDU[1] & ~SAP_BIT;
        uint8_t dsap_resp = ssap_req;
        uint8_t ssap_resp = SAP_DATA_EXCH;
        BuildSD2Response(pResponse,
                         msg.MasterAddress & ~SAP_BIT, pSlave->Config.Address,
                         FC_RESP_SLAVE_DL,
                         dsap_resp, ssap_resp,
                         pSlave->input.data, pSlave->input.len);
    } else {
        /* Default SAP — no SAP bytes in response */
        BuildSD2Response(pResponse,
                         msg.MasterAddress & ~SAP_BIT, pSlave->Config.Address,
                         FC_RESP_SLAVE_DL,
                         0, 0,
                         pSlave->input.data, pSlave->input.len);
    }

    return 0;
}

/* ---- Global_Control (SAP 0x3D, broadcast SDN) -------------------- */

static uint8_t HandleGlobalControl(ProfibusMessage msg, profibusSlave *pSlave)
{
    /*
     * PDU: DSAP | SSAP | Control_Command | Group_Select
     * No response — broadcast service.
     */
    if (msg.PDULength < 4) return 1;

    uint8_t ctrl  = msg.PDU[2];
    uint8_t group = msg.PDU[3];

    /* Group select: 0 = all groups, non-zero = specific group bit mask */
    if (group != 0 && !(group & pSlave->State.Group)) return 1;

    if (ctrl & 0x02) { /* CLEAR DATA */
        memset(pSlave->output.data, 0, pSlave->output.len);
    }
    if (ctrl & 0x08) pSlave->State.Frozen = 1;  /* FREEZE  */
    if (ctrl & 0x04) pSlave->State.Frozen = 0;  /* UNFREEZE */
    if (ctrl & 0x20) pSlave->State.Sync   = 1;  /* SYNC    */
    if (ctrl & 0x10) pSlave->State.Sync   = 0;  /* UNSYNC  */

    return 1;  /* No response for broadcast */
}

/* ---- FDL_Status (FC=0x09) ---------------------------------------- */

static uint8_t HandleFDLStatus(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    /*
     * SD1 response: SD1 | masterAddr | slaveAddr | 0x00 | FCS | ED
     */
    uint8_t masterAddr = msg.MasterAddress & ~SAP_BIT;

    BUILD_RESPONSE(pResponse, TELEGRAM_SD1);
    BUILD_RESPONSE(pResponse, masterAddr);
    BUILD_RESPONSE(pResponse, pSlave->Config.Address);
    BUILD_RESPONSE(pResponse, 0x00);
    BUILD_RESPONSE(pResponse, CalcFCS(&pResponse->Data[1], 3));
    BUILD_RESPONSE(pResponse, TELEGRAM_ED);

    return 0;
}

/* ------------------------------------------------------------------ */
/* ProcessFunction — top-level dispatch                               */
/* ------------------------------------------------------------------ */

uint8_t ProcessFunction(ProfibusMessage msg, profibusSlave *pSlave, resp *pResponse)
{
    pResponse->Length = 0;

    uint8_t fc_low = msg.FunctionCode & 0x0F;

    /* ---- FDL Status (FC_FDL_STATUS = 0x09) — no SAP, always answer ---- */
    if (fc_low == FC_FDL_STATUS) {
        return HandleFDLStatus(msg, pSlave, pResponse);
    }

    /* ---- Token frames — ignore ---------------------------------------- */
    if (msg.MessageType == TELEGRAM_SD4) return 1;

    /* ---- Determine if SAP extension is present ----------------------- */
    bool has_sap = (msg.SlaveAddress & SAP_BIT) != 0;

    if (!has_sap) {
        /* No SAP extension → default Data_Exchange */
        return HandleDataExchange(msg, pSlave, pResponse);
    }

    /* ---- SAP dispatch ------------------------------------------------- */
    uint8_t dsap = msg.PDU[0] & ~SAP_BIT;   /* Strip bit7 (sometimes set by master) */
    uint8_t ssap = (msg.PDULength >= 2) ? (msg.PDU[1] & ~SAP_BIT) : 0;

    ESP_LOGD(TAG_PROTOCOL, "SAP: DSAP=0x%02X SSAP=0x%02X FC=0x%02X state=%d",
             dsap, ssap, msg.FunctionCode, pSlave->State.ReadyState);

    switch (dsap)
    {
    case SAP_SLAVE_DIAG:    /* 0x3C */
        return HandleSlaveDiag(msg, pSlave, pResponse);

    case SAP_SET_PRM:       /* 0x32 */
        return HandleSetPrm(msg, pSlave, pResponse);

    case SAP_CHK_CFG:       /* 0x33 */
        return HandleChkCfg(msg, pSlave, pResponse);

    case SAP_DATA_EXCH:     /* 0x3E */
        return HandleDataExchange(msg, pSlave, pResponse);

    case SAP_GLB_CTRL:      /* 0x3D */
        return HandleGlobalControl(msg, pSlave);

    default:
        ESP_LOGW(TAG_PROTOCOL, "Unhandled DSAP 0x%02X (FC=0x%02X)", dsap, msg.FunctionCode);
        return 1;
    }
}
