/**
 * Protocol.c
 * PROFIBUS-DP slave protocol — frame parsing and SAP dispatch.
 *
 * Slave state machine:
 *   SS_POWERON  → SS_WPRM   (on first Slave_Diag)
 *   SS_WPRM     → SS_WCFG   (valid Set_Prm: ident match)
 *   SS_WCFG     → SS_DXCHG  (valid Chk_Cfg)
 *   any state   → SS_WPRM   (ident mismatch, cfg fault, WD timeout)
 *
 * SAPs handled (DSAP on the slave):
 *   0x32  Set_Prm       FC=SDA  → respond SC
 *   0x33  Chk_Cfg       FC=SDA  → respond SC
 *   0x3A  Get_Cfg       FC=SRD  → respond SD2 with our cfg bytes
 *   0x3C  Slave_Diag    FC=SRD  → respond SD2 with 6 diag bytes
 *   0x3D  Global_Ctrl   FC=SDN  → no response (broadcast)
 *   0x3E  Data_Exchange FC=SRD  → respond SD2 with input data
 *   no SAP extension    FC=SRD  → treated as Data_Exchange
 *
 * Additional FDL services handled directly from the FC byte (no SAP):
 *   FC=0x09  FDL_Status  → respond SD1, FC=0x00
 *   FC=0x0F  Req_LSAP    → respond SC
 */

#include "Protocol.h"
#include "Slave.h"
#include "esp_timer.h"

/* ------------------------------------------------------------------ */
/* FCS: arithmetic sum mod-256                                        */
/* ------------------------------------------------------------------ */

static uint8_t CalcFCS(const uint8_t *data, uint32_t len)
{
    uint8_t s = 0;
    for (uint32_t i = 0; i < len; i++) s += data[i];
    return s;
}

uint8_t GetChecksum(uint8_t *data, uint32_t len)
{
    return CalcFCS(data, len);
}

/* ------------------------------------------------------------------ */
/* Diagnostic byte builders                                           */
/* ------------------------------------------------------------------ */

/*
 * Status byte 1:
 *   bit1 Station_Not_Ready  (1 when NOT in Data_Exchange)
 *   bit6 Prm_Fault          (set via pSlave->diag_prm_fault)
 *   bit2 Cfg_Fault          (set via pSlave->diag_cfg_fault)
 * all others 0.
 */
static uint8_t DiagStatus1(const profibusSlave *s)
{
    uint8_t b = 0;
    if (s->State.ReadyState != SS_DXCHG) b |= 0x02; /* Station_Not_Ready */
    if (s->diag_cfg_fault)               b |= 0x04; /* Cfg_Fault         */
    if (s->diag_ext_fault)               b |= 0x08; /* Ext_Diag (fault)  */
    if (s->diag_prm_fault)               b |= 0x40; /* Prm_Fault         */
    return b;
}

/*
 * Status byte 2:
 *   bit2 Always 1
 *   bit3 WD_On (watchdog enabled)
 *   bit4 Freeze_Mode
 *   bit5 Sync_Mode
 */
static uint8_t DiagStatus2(const profibusSlave *s)
{
    /*
     * bit0 Prm_Req    — slave is requesting (re-)parametrisation.
     * bit1 Stat_Diag  — slave requests master to keep polling Slave_Diag.
     *                   Used by ET200S after Chk_Cfg until settled.
     * bit2 Always 1   — mandatory per spec.
     * bit3 WD_On      — watchdog is active.
     * bit4 Freeze_Mode
     * bit5 Sync_Mode
     */
    return (uint8_t)(
        (s->diag_prm_req            << 0) |
        (s->diag_stat_diag          << 1) |
        (1                          << 2) |
        (s->Control.WatchdogStatus  << 3) |
        (s->State.Frozen            << 4) |
        (s->State.Sync              << 5)
    );
}

/* Status byte 3: all reserved — always 0 */
static uint8_t DiagStatus3(void) { return 0x00; }

/* ------------------------------------------------------------------ */
/* SD2 response builder                                               */
/* ------------------------------------------------------------------ */

/*
 * Builds a complete SD2 response frame into pResponse.
 *
 * Per EN 50170 / Felser PROFIBUS Manual:
 *   - DA and SA address bytes have bit7 set when a SAP extension is present
 *     (this signals to the receiver that DSAP+SSAP follow in the PDU).
 *   - The DSAP and SSAP bytes inside the PDU do NOT have bit7 set.
 *
 * Wire layout:
 *   SD2 | LE | LEr | SD2 | DA[|SAP_BIT] | SA[|SAP_BIT] | FC | [DSAP | SSAP] | pdu... | FCS | ED
 *
 * Pass dsap=0 and ssap=0 for non-SAP frames (plain Data_Exchange responses).
 */
static void BuildSD2Response(resp *pResponse,
                             uint8_t masterAddr, uint8_t slaveAddr,
                             uint8_t fc,
                             uint8_t dsap, uint8_t ssap,
                             const uint8_t *pdu, uint8_t pdu_len)
{
    pResponse->Length = 0;
    bool sap = (dsap | ssap) != 0;

    BUILD_RESPONSE(pResponse, TELEGRAM_SD2);
    BUILD_RESPONSE(pResponse, 0x00);            /* LE placeholder  */
    BUILD_RESPONSE(pResponse, 0x00);            /* LEr placeholder */
    BUILD_RESPONSE(pResponse, TELEGRAM_SD2);

    /* DA and SA: set SAP_BIT to signal that DSAP/SSAP follow */
    BUILD_RESPONSE(pResponse, masterAddr | (sap ? SAP_BIT : 0));
    BUILD_RESPONSE(pResponse, slaveAddr  | (sap ? SAP_BIT : 0));
    BUILD_RESPONSE(pResponse, fc);

    /* DSAP and SSAP in the PDU: plain values, no SAP_BIT */
    if (sap) {
        BUILD_RESPONSE(pResponse, dsap);
        BUILD_RESPONSE(pResponse, ssap);
    }

    for (uint8_t i = 0; i < pdu_len; i++) {
        BUILD_RESPONSE(pResponse, pdu[i]);
    }

    uint8_t payload_len = (uint8_t)(pResponse->Length - 4);
    pResponse->Data[1] = payload_len;
    pResponse->Data[2] = payload_len;

    BUILD_RESPONSE(pResponse, CalcFCS(&pResponse->Data[4], payload_len));
    BUILD_RESPONSE(pResponse, TELEGRAM_ED);
}

/* ------------------------------------------------------------------ */
/* GetMessage: parse raw UART bytes into ProfibusMessage              */
/* ------------------------------------------------------------------ */

uint8_t GetMessage(uint8_t *pData, uint32_t Length, ProfibusMessage *Message)
{
    if (Length < 3) 
    {
        ESP_LOGW(TAG_PROTOCOL, "Message too short");
        return 1;
    }

    Message->MasterAddress = 0;
    Message->SlaveAddress  = 0;
    Message->PDULength     = 0;
    Message->MessageType   = (TelegramTypes)pData[0];

    switch (Message->MessageType)
    {
    case TELEGRAM_SD1:
        /* SD1 | DA | SA | FC | FCS | ED  (6 bytes) */
        if (Length < 6) return 1;
        if (pData[5] != TELEGRAM_ED) return 1;
        if (CalcFCS(&pData[1], 3) != pData[4]) {
            ESP_LOGW(TAG_PROTOCOL, "SD1 FCS mismatch");
            return 1;
        }
        Message->SlaveAddress  = pData[1];
        Message->MasterAddress = pData[2];
        Message->FunctionCode  = pData[3];
        return 0;

    case TELEGRAM_SD2: {
        /* SD2 | LE | LEr | SD2 | DA | SA | FC | PDU... | FCS | ED */
        if (Length < 6) return 1;
        uint8_t le = pData[1], ler = pData[2];
        if (le != ler || pData[3] != TELEGRAM_SD2) return 1;
        if ((uint32_t)(le + 6) > Length) return 1;
        if (pData[4 + le + 1] != TELEGRAM_ED) return 1;

        if (CalcFCS(&pData[4], le) != pData[4 + le]) {
            ESP_LOGW(TAG_PROTOCOL, "SD2 FCS mismatch");
            return 1;
        }
        Message->SlaveAddress  = pData[4];       /* DA — bit7 = SAP extension */
        Message->MasterAddress = pData[5];       /* SA */
        Message->FunctionCode  = pData[6];
        Message->PDULength     = le - 3;
        if (Message->PDULength > sizeof(Message->PDU))
            Message->PDULength = sizeof(Message->PDU);
        memcpy(Message->PDU, &pData[7], Message->PDULength);
        return 0;
    }

    case TELEGRAM_SD3:
        /* SD3 | DA | SA | FC | 8×data | FCS | ED  (14 bytes) */
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
        /* SD4 | DA | SA  (3 bytes, no FCS) */
        if (Length < 3) return 1;
        Message->SlaveAddress  = pData[1];
        Message->MasterAddress = pData[2];
        return 0;

    case TELEGRAM_SC:
        return 0;

    default:
        ESP_LOGE(TAG_PROTOCOL, "Unhandled Message");
        ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, pData, Length, ESP_LOG_ERROR);

        return 1;
    }
}

/* ------------------------------------------------------------------ */
/* Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static void ResetToWaitPrm(profibusSlave *s)
{
    ESP_LOGW(TAG_PROTOCOL, "addr=%u → SS_WPRM", s->Config.Address);
    s->State.ReadyState     = SS_WPRM;
    s->prm_len              = 0;
    s->master_address       = 0xFF;
    s->diag_prm_req         = 1;   /* Signal to master: please re-parametrise */
    s->diag_stat_diag       = 0;   /* Clear Stat_Diag — station not settled   */
    s->diag_stat_diag_count = 0;
    s->diag_ext_fault       = 0;   /* Clear fault ext-diag on reset           */
    /* Leave cfg_data and ext_diag_data intact for re-validation */
}

/*
 * Extract the DSAP and SSAP from the PDU.
 * Returns false if the PDU is too short to contain a SAP extension
 * (this would be a malformed frame — caller should ignore it).
 */
static bool ParseSAP(const ProfibusMessage *msg, uint8_t *dsap_out, uint8_t *ssap_out)
{
    if (msg->PDULength < 2) return false;
    *dsap_out = msg->PDU[0] & ~SAP_BIT;
    *ssap_out = msg->PDU[1] & ~SAP_BIT;
    return true;
}

/* ------------------------------------------------------------------ */
/* SAP 0x3C — Slave_Diag                                             */
/* ------------------------------------------------------------------ */

static uint8_t HandleSlaveDiag(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * Slave_Diag response structure
     * ==============================
     * Always 6 mandatory bytes:
     *   [0] Status1  [1] Status2  [2] Status3
     *   [3] master_address (0xFF = unparametrised)
     *   [4] Ident_Number high  [5] Ident_Number low
     *
     * If s->ext_diag_len > 0 these are appended verbatim after byte [5].
     * Status1 bit3 (Ext_Diag) is set by DiagStatus1() when ext_diag_len > 0.
     *
     * Stat_Diag countdown
     * ====================
     * When diag_stat_diag=1 the master is required to keep polling.
     * After diag_stat_diag_threshold consecutive responses we clear the bit,
     * signalling the master that the station is fully settled.
     *
     * This matches the captured ET200S conversation:
     *   Frames 9,11,13,15: Status2 bit1 = 1  (keep polling)
     *   Frame 17:          Status2 bit1 = 0  (ready for Data_Exchange)
     *
     * Transition POWERON → WPRM on first diag request.
     */
    if (s->State.ReadyState == SS_POWERON)
        s->State.ReadyState = SS_WPRM;

    /* Advance Stat_Diag countdown if active */
    if (s->diag_stat_diag) {
        s->diag_stat_diag_count++;
        if (s->diag_stat_diag_threshold > 0 &&
            s->diag_stat_diag_count >= s->diag_stat_diag_threshold) {
            s->diag_stat_diag       = 0;
            s->diag_stat_diag_count = 0;
            ESP_LOGI(TAG_PROTOCOL,
                     "Slave_Diag: Stat_Diag cleared after %u polls",
                     s->diag_stat_diag_threshold);
        }
    }

    uint8_t dsap_req, ssap_req;
    if (!ParseSAP(&msg, &dsap_req, &ssap_req))
        ssap_req = SAP_DATA_EXCH;

    /* Build the response PDU: 6 mandatory bytes + optional extended diag */
    uint8_t diag[6 + PB_MAX_EXT_DIAG_LEN];
    diag[0] = DiagStatus1(s);
    diag[1] = DiagStatus2(s);
    diag[2] = DiagStatus3();
    diag[3] = s->master_address;
    diag[4] = s->Config.ID_HIGH;
    diag[5] = s->Config.ID_LOW;

    uint8_t diag_len = 6;
    // if (s->ext_diag_len > 0) {
    //     memcpy(&diag[6], s->ext_diag_data, s->ext_diag_len);
    //     diag_len += s->ext_diag_len;
    // }

    BuildSD2Response(pResponse,
                     msg.MasterAddress & ~SAP_BIT,
                     s->Config.Address,
                     FC_RESP_SLAVE_DL,
                     ssap_req,
                     SAP_SLAVE_DIAG,
                     diag, diag_len);

    s->diag_prm_fault = 0;
    s->diag_cfg_fault = 0;
    s->cnt_diag++;
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x32 — Set_Prm                                                */
/* ------------------------------------------------------------------ */

static uint8_t HandleSetPrm(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * PDU layout (PDU[0]=DSAP, PDU[1]=SSAP, then fixed bytes):
     *   [2]  Station_Status  lock/unlock/WD/freeze/sync bits
     *   [3]  WD_Fact_1
     *   [4]  WD_Fact_2
     *   [5]  Min_Tsdr         (slave ignores)
     *   [6]  Ident_Number high
     *   [7]  Ident_Number low
     *   [8]  Group_Ident
     *   [9+] User_Prm_Data   (optional)
     *
     * Minimum PDU length: 2 (SAPs) + 7 (fixed) = 9 bytes.
     * SC on success, no response on failure (master will re-try after diag).
     */
    if (msg.PDULength < 9) {
        ESP_LOGW(TAG_PROTOCOL, "Set_Prm too short (%u bytes)", msg.PDULength);
        s->diag_prm_fault = 1;
        return 1;
    }

    uint8_t station_status = msg.PDU[2];
    uint8_t wd1            = msg.PDU[3];
    uint8_t wd2            = msg.PDU[4];
    /* PDU[5] = Min_Tsdr — slave ignores */
    uint8_t ident_high     = msg.PDU[6];
    uint8_t ident_low      = msg.PDU[7];
    uint8_t group          = msg.PDU[8];

    /* Validate ident number */
    if (ident_high != s->Config.ID_HIGH || ident_low != s->Config.ID_LOW) {
        ESP_LOGW(TAG_PROTOCOL,
                 "Set_Prm ident mismatch: expected %02X%02X got %02X%02X",
                 s->Config.ID_HIGH, s->Config.ID_LOW, ident_high, ident_low);
        s->diag_prm_fault = 1;
        ResetToWaitPrm(s);
        return 1;
    }

    /* Station status flags */
    s->Control.WatchdogStatus = (station_status & 0x08) != 0;
    s->Control.FreezeReq      = (station_status & 0x10) != 0;
    s->Control.SyncReq        = (station_status & 0x20) != 0;
    if (station_status & 0x80) s->Control.LockReq = 1;
    if (station_status & 0x40) s->Control.LockReq = 0; /* Unlock overrides */

    /* Watchdog: timeout_ms = WD_Fact_1 × WD_Fact_2 × 10 */
    s->Control.WatchdogVal = (uint32_t)wd1 * (uint32_t)wd2 * 10;
    s->State.Group = group;

    /* Record which master parametrised us */
    if (s->Control.LockReq)
        s->master_address = msg.MasterAddress & ~SAP_BIT;
    else if (!s->Control.LockReq && s->Control.WatchdogStatus == 0)
        s->master_address = 0xFF; /* Unlock */

    /* DPV1 status bytes (optional — present if PDU long enough) */
    if (msg.PDULength >= 12) {
        s->Control.dpv1Status1 = *(DPV1_Status_1 *)&msg.PDU[9];
        s->Control.dpv1Status2 = *(DPV1_Status_2 *)&msg.PDU[10];
        s->Control.dpv1Status3 = *(DPV1_Status_3 *)&msg.PDU[11];
    }

    /* Store user parameter data (PDU[9..] or after DPV1 bytes) */
    const uint8_t prm_start = (msg.PDULength >= 12) ? 12 : 9;
    if (msg.PDULength > prm_start) {
        s->prm_len = msg.PDULength - prm_start;
        if (s->prm_len > PB_MAX_PRM_LEN) s->prm_len = PB_MAX_PRM_LEN;
        memcpy(s->prm_data, &msg.PDU[prm_start], s->prm_len);
    } else {
        s->prm_len = 0;
    }

    s->diag_prm_fault    = 0;
    s->diag_prm_req      = 0;   /* Master can now advance past WAIT_PRM */
    s->State.ReadyState  = SS_WCFG;
    s->cnt_prm++;

    ESP_LOGI(TAG_PROTOCOL, "Set_Prm OK: addr=%u WD=%lums → SS_WCFG",
             s->Config.Address, (unsigned long)s->Control.WatchdogVal);

    BUILD_RESPONSE(pResponse, TELEGRAM_SC);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x33 — Chk_Cfg                                                */
/* ------------------------------------------------------------------ */

static uint8_t HandleChkCfg(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * PDU: DSAP | SSAP | cfg_byte_0 | cfg_byte_1 | ...
     *
     * cfg byte encoding per EN 50170 (compact format):
     *   bits[7:6] = 00 special, 01 input, 10 output, 11 I/O
     *   bit5      = word/byte consistency
     *   bits[3:0] = length-1  (0x0F → 16 bytes)
     *
     * KFC750 PPO5: single byte 0xFF = 11 1 1 1111
     *   type=11 (I/O), consistency=word, len=16 bytes each direction.
     *
     * If cfg_strict=0: accept and store whatever master sends.
     * If cfg_strict=1: validate against pre-loaded cfg_data/cfg_len.
     */
    if (s->State.ReadyState != SS_WCFG) {
        ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg in wrong state (%d)", s->State.ReadyState);
        ResetToWaitPrm(s);
        return 1;
    }

    if (msg.PDULength < 2) {
        ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg PDU too short (%u)", msg.PDULength);
        s->diag_cfg_fault = 1;
        ResetToWaitPrm(s);
        return 1;
    }

    const uint8_t  cfg_off  = 2;   /* skip DSAP + SSAP */
    const uint8_t  cfg_len  = msg.PDULength - cfg_off;
    const uint8_t *cfg_bytes = &msg.PDU[cfg_off];

    if (s->cfg_strict && s->cfg_len > 0) {
        if (cfg_len != s->cfg_len ||
            memcmp(cfg_bytes, s->cfg_data, cfg_len) != 0) {
            ESP_LOGW(TAG_PROTOCOL, "Chk_Cfg mismatch (got %u bytes)", cfg_len);
            ESP_LOG_BUFFER_HEXDUMP(TAG_PROTOCOL, cfg_bytes, cfg_len, ESP_LOG_WARN);
            s->diag_cfg_fault = 1;
            ResetToWaitPrm(s);
            return 1;
        }
    } else {
        /* Permissive mode: accept and store */
        uint8_t store_len = cfg_len < PB_MAX_CFG_LEN ? cfg_len : PB_MAX_CFG_LEN;
        memcpy(s->cfg_data, cfg_bytes, store_len);
        s->cfg_len = store_len;

        /* Derive I/O sizes from cfg bytes if application hasn't pre-set them.
         *
         * Compact format byte (Felser Manual - kompaktes_format.html):
         *   bit7    = consistency (1=whole module, 0=per byte/word)
         *   bit6    = unit       (1=words/16-bit,  0=bytes/8-bit)
         *   bits5:4 = type       (01=input, 10=output, 11=I/O)
         *   bits3:0 = count-1    (0=1 unit, 15=16 units)
         *
         * Byte count = (bits3:0 + 1) * (bit6 ? 2 : 1)
         */
        if (s->output.len == 0 && s->input.len == 0) {
            uint8_t in_total = 0, out_total = 0;
            for (uint8_t i = 0; i < store_len; i++) {
                uint8_t b     = cfg_bytes[i];
                uint8_t type  = (b >> 4) & 0x03;  /* bits[5:4] */
                uint8_t count = (b & 0x0F) + 1;   /* bits[3:0] + 1 */
                uint8_t words = (b >> 6) & 0x01;  /* bit6: 1=words, 0=bytes */
                uint8_t blen  = count * (words ? 2u : 1u);
                if      (type == 0x01) in_total  += blen;
                else if (type == 0x02) out_total += blen;
                else if (type == 0x03) { in_total += blen; out_total += blen; }
                /* type==0x00: special format identifier byte — not a data byte */
            }
            s->input.len  = in_total  < PB_MAX_IO_LEN ? in_total  : PB_MAX_IO_LEN;
            s->output.len = out_total < PB_MAX_IO_LEN ? out_total : PB_MAX_IO_LEN;
            ESP_LOGI(TAG_PROTOCOL, "Chk_Cfg: derived I=%u O=%u from cfg",
                     s->input.len, s->output.len);
        }
    }

    s->diag_cfg_fault   = 0;
    s->State.ReadyState = SS_DXCHG;
    s->last_rx_time_ms  = esp_timer_get_time() / 1000LL;
    s->cnt_cfg++;

    /*
     * Arm Stat_Diag: forces the master to keep polling Slave_Diag until the
     * slave clears the bit (after diag_stat_diag_threshold polls).
     * Only arm if the slave has configured a threshold (ET200S sets 4).
     */
    if (s->diag_stat_diag_threshold > 0) {
        s->diag_stat_diag       = 1;
        s->diag_stat_diag_count = 0;
        ESP_LOGI(TAG_PROTOCOL,
                 "Chk_Cfg OK: Stat_Diag armed, will clear after %u polls",
                 s->diag_stat_diag_threshold);
    }

    ESP_LOGI(TAG_PROTOCOL, "Chk_Cfg OK (%u cfg bytes) → SS_DXCHG  I=%u O=%u",
             cfg_len, s->input.len, s->output.len);

    BUILD_RESPONSE(pResponse, TELEGRAM_SC);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x3A — Get_Cfg                                                */
/* ------------------------------------------------------------------ */

static uint8_t HandleGetCfg(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * Master asks: "what configuration did you actually accept?"
     * Slave responds with the cfg bytes stored during Chk_Cfg.
     *
     * Get_Cfg can arrive in any state — Siemens masters send it at startup
     * (before Set_Prm) to discover what the slave expects.  If we have no
     * cfg yet, respond with our default/expected cfg so the master can
     * decide whether its project config matches.
     *
     * Frame structure built by BuildSD2Response:
     *   SD2 | LE | LEr | SD2 | DA|SAP | SA|SAP | FC | DSAP|SAP | SSAP|SAP | <cfg bytes> | FCS | ED
     *
     * The SAP bytes go into the frame header via BuildSD2Response — do NOT
     * put them in the pdu[] buffer or they appear twice and corrupt the frame.
     * The payload (pdu[]) contains ONLY the raw cfg bytes.
     */
    uint8_t dsap_req, ssap_req;
    if (!ParseSAP(&msg, &dsap_req, &ssap_req)) ssap_req = SAP_DATA_EXCH;

    /* Use stored cfg if available, otherwise fall back to the default */
    const uint8_t *cfg_src = s->cfg_data;
    uint8_t        cfg_len = s->cfg_len;

    /* If not yet configured, send our expected cfg so master can compare */
    if (cfg_len == 0) {
        ESP_LOGD(TAG_PROTOCOL, "Get_Cfg: no cfg stored yet, sending empty");
    }

    /* Payload is just the raw cfg bytes — BuildSD2Response adds SAP header */
    BuildSD2Response(pResponse,
                     msg.MasterAddress & ~SAP_BIT,
                     s->Config.Address,
                     FC_RESP_SLAVE_DL,
                     ssap_req,     /* DSAP of response = master's SSAP */
                     SAP_GET_CFG,  /* SSAP of response = our SAP        */
                     cfg_src, cfg_len);

    ESP_LOGI(TAG_PROTOCOL, "GetConfig, return %u bytes", pResponse->Length);
    ESP_LOG_BUFFER_HEXDUMP("Payload:", pResponse->Data, pResponse->Length, ESP_LOG_DEBUG);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x3E / no SAP — Data_Exchange                                 */
/* ------------------------------------------------------------------ */

static uint8_t HandleDataExchange(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * If not in SS_DXCHG, respond with Slave_Diag so the master knows
     * to re-parametrise.
     *
     * Output data from master:
     *   No-SAP frame: PDU[0..] = output bytes
     *   SAP frame:    PDU[0]=DSAP, PDU[1]=SSAP, PDU[2..]=output bytes
     */

     ESP_LOGI(TAG_PROTOCOL, "DExchg");

    if (s->State.ReadyState != SS_DXCHG) {
        ESP_LOGE(TAG_PROTOCOL, "Data_Exchange not in DXCHG (state=%d)",
                 s->State.ReadyState);
        return HandleSlaveDiag(msg, s, pResponse);
    }

    s->last_rx_time_ms = esp_timer_get_time() / 1000LL;

    bool    has_sap    = (msg.SlaveAddress & SAP_BIT) != 0;
    uint8_t data_off   = has_sap ? 2 : 0;

    /* Write output data from master into the slave's output buffer */
    if (msg.PDULength > data_off) {
        uint8_t out_len = msg.PDULength - data_off;
        if (out_len > PB_MAX_IO_LEN) out_len = PB_MAX_IO_LEN;
        memcpy(s->output.data, &msg.PDU[data_off], out_len);
        s->output.len = out_len;
    }

    /* Build response with current input data */
    if (has_sap) {
        uint8_t dsap_req, ssap_req;
        ParseSAP(&msg, &dsap_req, &ssap_req);
        BuildSD2Response(pResponse,
                         msg.MasterAddress & ~SAP_BIT,
                         s->Config.Address,
                         FC_RESP_SLAVE_DL,
                         ssap_req, SAP_DATA_EXCH,
                         s->input.data, s->input.len);
    } else {
        BuildSD2Response(pResponse,
                         msg.MasterAddress & ~SAP_BIT,
                         s->Config.Address,
                         FC_RESP_SLAVE_DL,
                         0, 0,
                         s->input.data, s->input.len);
    }

    s->cnt_data_exch++;
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x3D — Global_Control  (broadcast, no response)               */
/* ------------------------------------------------------------------ */

static uint8_t HandleGlobalControl(ProfibusMessage msg, profibusSlave *s)
{
    /*
     * PDU: DSAP | SSAP | Control_Command | Group_Select
     *
     * Control_Command bits:
     *   bit1  Clear_Data  (zero outputs)
     *   bit2  UNFREEZE
     *   bit3  FREEZE
     *   bit4  UNSYNC
     *   bit5  SYNC
     */
    if (msg.PDULength < 4) return 1;

    uint8_t ctrl  = msg.PDU[2];
    uint8_t group = msg.PDU[3];

    /* Group_Select: 0 = all groups, non-zero = specific group bits */
    if (group != 0 && !(group & (1u << s->State.Group))) return 1;

    if (ctrl & 0x02) memset(s->output.data, 0, s->output.len); /* Clear_Data */
    if (ctrl & 0x08) s->State.Frozen = 1;  /* FREEZE   */
    if (ctrl & 0x04) s->State.Frozen = 0;  /* UNFREEZE */
    if (ctrl & 0x20) s->State.Sync   = 1;  /* SYNC     */
    if (ctrl & 0x10) s->State.Sync   = 0;  /* UNSYNC   */

    return 1; /* Broadcast — no response */
}

/* ------------------------------------------------------------------ */
/* FC=0x09 — FDL Status (bus presence check)                         */
/* ------------------------------------------------------------------ */

static uint8_t HandleFDLStatus(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * Master probes for active slaves before parametrisation.
     * Response: SD1, FC=0x00 (PROFIBUS_SLAVE | Data_Low).
     *
     * SD1: SD1 | DA | SA | FC | FCS | ED
     */
    uint8_t master_addr = msg.MasterAddress & ~SAP_BIT;

    BUILD_RESPONSE(pResponse, TELEGRAM_SD1);
    BUILD_RESPONSE(pResponse, master_addr);
    BUILD_RESPONSE(pResponse, s->Config.Address);
    BUILD_RESPONSE(pResponse, 0x00);   /* FC: slave, data low */
    BUILD_RESPONSE(pResponse, CalcFCS(&pResponse->Data[1], 3));
    BUILD_RESPONSE(pResponse, TELEGRAM_ED);

    ESP_LOGI(TAG_PROTOCOL, "FDL Status, return %u bytes", pResponse->Length);
    ESP_LOG_BUFFER_HEXDUMP("Payload:", pResponse->Data, pResponse->Length, ESP_LOG_DEBUG);

    return 0;
}

/* ------------------------------------------------------------------ */
/* FC=0x0F — Request LSAP Status                                     */
/* ------------------------------------------------------------------ */

static uint8_t HandleReqLSAP(resp *pResponse)
{
    /* Slave acknowledges with SC — we don't support full LSAP reporting */
    BUILD_RESPONSE(pResponse, TELEGRAM_SC);

    ESP_LOGI(TAG_PROTOCOL, "ReqLSAP, return %u bytes", pResponse->Length);
    ESP_LOG_BUFFER_HEXDUMP("Payload:", pResponse->Data, pResponse->Length, ESP_LOG_DEBUG);

    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x34 — Set_Slave_Add                                           */
/* ------------------------------------------------------------------ */

static uint8_t HandleSetSlaveAddr(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    (void)pResponse;  /* SDN service — no response transmitted */
    /*
     * PDU layout: DSAP | SSAP | New_Address | ID_HIGH | ID_LOW | No_Add_Chg
     *
     * IMPORTANT: Set_Slave_Add is sent with FC=SDN (Send Data No Acknowledge,
     * FC low nibble = 0x04 or 0x06).  The master does NOT expect any response.
     * Transmitting SC here puts a spurious byte on the bus that collides with
     * the master's next request (which immediately follows) and corrupts it.
     *
     * Correct behaviour: parse and log only, return 1 (no response).
     *
     * KFC750 GSD has Set_Slave_Add_supp = 0, so address changes are ignored.
     * To support address changes: validate ident, store to NVS, update
     * s->Config.Address, re-register with controller (or reboot).
     */
    if (msg.PDULength < 6) {
        ESP_LOGW(TAG_PROTOCOL, "Set_Slave_Add PDU too short (%u)", msg.PDULength);
        return 1;  /* SDN — no response */
    }

    uint8_t new_addr   = msg.PDU[2];
    uint8_t id_high    = msg.PDU[3];
    uint8_t id_low     = msg.PDU[4];
    uint8_t no_add_chg = msg.PDU[5];

    if (id_high != s->Config.ID_HIGH || id_low != s->Config.ID_LOW) {
        ESP_LOGD(TAG_PROTOCOL,
                 "Set_Slave_Add: ident mismatch (%02X%02X vs %02X%02X) — not for us",
                 id_high, id_low, s->Config.ID_HIGH, s->Config.ID_LOW);
        return 1;
    }

    ESP_LOGI(TAG_PROTOCOL,
             "Set_Slave_Add: new_addr=%u no_add_chg=%u (not supported, ignored)",
             new_addr, no_add_chg);

    (void)no_add_chg;
    return 1;  /* SDN — no response */
}


/* ------------------------------------------------------------------ */
/* SAP 0x38 — Rd_Inp  (read current input data)                      */
/* ------------------------------------------------------------------ */

static uint8_t HandleRdInp(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * Master requests a snapshot of the slave's current input data.
     * Used by Class 2 masters (engineering tools) for diagnostics.
     * Available in any state — no state restriction.
     */
    uint8_t dsap_req, ssap_req;
    if (!ParseSAP(&msg, &dsap_req, &ssap_req)) ssap_req = 0x3E;

    BuildSD2Response(pResponse,
                     msg.MasterAddress & ~SAP_BIT,
                     s->Config.Address,
                     FC_RESP_SLAVE_DL,
                     ssap_req, SAP_RD_INP,
                     s->input.data, s->input.len);

    ESP_LOGD(TAG_PROTOCOL, "Rd_Inp: returned %u bytes", s->input.len);
    return 0;
}

/* ------------------------------------------------------------------ */
/* SAP 0x39 — Rd_Outp  (read current output data)                    */
/* ------------------------------------------------------------------ */

static uint8_t HandleRdOutp(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    /*
     * Master requests a snapshot of the slave's current output data
     * (what the master last wrote to us). Used for diagnostics.
     */
    uint8_t dsap_req, ssap_req;
    if (!ParseSAP(&msg, &dsap_req, &ssap_req)) ssap_req = 0x3E;

    BuildSD2Response(pResponse,
                     msg.MasterAddress & ~SAP_BIT,
                     s->Config.Address,
                     FC_RESP_SLAVE_DL,
                     ssap_req, SAP_RD_OUTP,
                     s->output.data, s->output.len);

    ESP_LOGD(TAG_PROTOCOL, "Rd_Outp: returned %u bytes", s->output.len);
    return 0;
}


/* ------------------------------------------------------------------ */
/* ProcessFunction — top-level dispatch                               */
/* ------------------------------------------------------------------ */

uint8_t ProcessFunction(ProfibusMessage msg, profibusSlave *s, resp *pResponse)
{
    pResponse->Length = 0;

    uint8_t fc_low = msg.FunctionCode & 0x0F;

    ESP_LOGW(TAG_PROTOCOL, "*******************");
    ESP_LOGI(TAG_PROTOCOL, "Received: slave=%u, fc=0x%02X, Nessage Type=0x%02X", msg.SlaveAddress & ~SAP_BIT, msg.FunctionCode, msg.MessageType);
    ESP_LOG_BUFFER_HEXDUMP("     PDU:", msg.PDU, msg.PDULength, ESP_LOG_DEBUG);
    
    /* ---- FDL-level services (no SAP extension needed) ---- */
    if (fc_low == FC_FDL_STATUS)  return HandleFDLStatus(msg, s, pResponse);
    if (fc_low == FC_REQ_LSAP_REPL) return HandleReqLSAP(pResponse);

    /* ---- No SAP extension: default Data_Exchange ---- */
    bool has_sap = (msg.SlaveAddress & SAP_BIT) != 0;
    if (!has_sap) {
        return HandleDataExchange(msg, s, pResponse);
    }

    /* ---- SAP dispatch ---- */
    if (msg.PDULength < 2) {
        /* SAP bit set but no room for DSAP+SSAP — malformed frame */
        ESP_LOGW(TAG_PROTOCOL, "SAP frame too short (PDULength=%u)", msg.PDULength);
        return 1;
    }

    uint8_t dsap = msg.PDU[0] & ~SAP_BIT;
    uint8_t ssap = msg.PDU[1] & ~SAP_BIT;

    ESP_LOGI(TAG_PROTOCOL, "SAP DSAP=0x%02X SSAP=0x%02X FC=0x%02X state=%d DA=0x%02X",
             dsap, ssap, msg.FunctionCode, s->State.ReadyState, msg.SlaveAddress);

    switch (dsap)
    {
    case SAP_SLAVE_DIAG:    /* 0x3C - 60 */
        return HandleSlaveDiag(msg, s, pResponse);

    case SAP_SET_PRM:       /* 0x3D - 61 */
        return HandleSetPrm(msg, s, pResponse);

    case SAP_CHK_CFG:       /* 0x3E - 62 */
        return HandleChkCfg(msg, s, pResponse);

    case SAP_GET_CFG:       /* 0x3B - 59 */
        return HandleGetCfg(msg, s, pResponse);

    case SAP_SET_ADDR:      /* 0x37 - 55 */
        return HandleSetSlaveAddr(msg, s, pResponse);

    case SAP_GLB_CTRL:      /* 0x3A - 58 - broadcast, no response */
        return HandleGlobalControl(msg, s);

    case SAP_RD_INP:        /* 0x38 - 56 */
        return HandleRdInp(msg, s, pResponse);

    case SAP_RD_OUTP:       /* 0x39 - 57 */
        return HandleRdOutp(msg, s, pResponse);

    default:
        ESP_LOGW(TAG_PROTOCOL,
                 "Unhandled DSAP=0x%02X SSAP=0x%02X FC=0x%02X state=%d",
                 dsap, ssap, msg.FunctionCode, s->State.ReadyState);
        return 1;
    }
}
