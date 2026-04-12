/**
 * kfc750_protocol.h
 * KFC750 Variable Speed Drive — PROFIBUS PPO5 frame layout, Control/Status
 * Word bit definitions, PKW request/response types, and parameter IDs.
 *
 * PPO5 frame layout (32 bytes each direction):
 * ┌────────┬───────────────────────────────────────────────────────────┐
 * │ Byte   │ Field                                                     │
 * ├────────┼───────────────────────────────────────────────────────────┤
 * │  0-1   │ PKE — Parameter ID (bits 11:0) + request/response type   │
 * │        │        (bits 15:12, upper nibble)                         │
 * │  2-3   │ IND — Parameter sub-index                                │
 * │  4-7   │ PWE — Parameter value (32-bit, big-endian)               │
 * ├────────┼───────────────────────────────────────────────────────────┤
 * │  8-9   │ PZD1 — Control Word (master→slave) / Status Word         │
 * │        │         (slave→master)                                    │
 * │ 10-11  │ PZD2 — Speed Reference (master→slave) / Actual frequency │
 * │        │         (slave→master)  Scaled: 0x4000 = 100% of MaxFreq │
 * │ 12-13  │ PZD3 — Output frequency Hz×100 (slave→master)           │
 * │ 14-15  │ PZD4 — Output voltage V (slave→master)                  │
 * │ 16-17  │ PZD5 — Output current A×10 (slave→master)               │
 * │ 18-19  │ PZD6 — DC bus voltage V (slave→master)                  │
 * │ 20-21  │ PZD7 — Active fault code (slave→master)                 │
 * │ 22-23  │ PZD8 — Output power kW×10 (slave→master)               │
 * │ 24-25  │ PZD9 — Target frequency Hz×100 (slave→master)           │
 * │ 26-27  │ PZD10 — AI1 analogue input (slave→master)               │
 * │ 28-29  │ PZD11 — AI2 analogue input (slave→master)               │
 * │ 30-31  │ PZD12 — Reserved (slave→master)                         │
 * └────────┴───────────────────────────────────────────────────────────┘
 */

#ifndef KFC750_PROTOCOL_H
#define KFC750_PROTOCOL_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Frame dimensions                                                   */
/* ------------------------------------------------------------------ */

#define KFC750_FRAME_LEN    32  /* bytes each direction */

/* PKW area offsets */
#define KFC750_PKE_OFFSET    0
#define KFC750_IND_OFFSET    2
#define KFC750_PWE_OFFSET    4

/* PZD area offsets */
#define KFC750_PZD1_OFFSET   8   /* CW / SW */
#define KFC750_PZD2_OFFSET  10   /* REF / ACT */
#define KFC750_PZD3_OFFSET  12   /* output frequency Hz×100 */
#define KFC750_PZD4_OFFSET  14   /* output voltage V */
#define KFC750_PZD5_OFFSET  16   /* output current A×10 */
#define KFC750_PZD6_OFFSET  18   /* DC bus voltage V */
#define KFC750_PZD7_OFFSET  20   /* fault code */
#define KFC750_PZD8_OFFSET  22   /* output power kW×10 */
#define KFC750_PZD9_OFFSET  24   /* target frequency Hz×100 */
#define KFC750_PZD10_OFFSET 26   /* AI1 */
#define KFC750_PZD11_OFFSET 28   /* AI2 */
#define KFC750_PZD12_OFFSET 30   /* reserved */

/* Speed scale: 0x4000 = 100% of max frequency */
#define KFC750_SPEED_SCALE  16384

/* ------------------------------------------------------------------ */
/* PKE — request type (bits 15:12 of PKE word, master→slave)         */
/* ------------------------------------------------------------------ */

typedef enum {
    PKE_REQ_NO_REQUEST   = 0x0, /* No request pending */
    PKE_REQ_READ_WORD    = 0x1, /* Read parameter (16-bit value) */
    PKE_REQ_WRITE_WORD   = 0x2, /* Write parameter (16-bit value) */
    PKE_REQ_READ_DWORD   = 0x3, /* Read parameter (32-bit value) */
    PKE_REQ_WRITE_DWORD  = 0x4, /* Write parameter (32-bit value) */
} KFC750_PkeRequest;

/* PKE — response type (bits 15:12 of PKE word, slave→master) */
typedef enum {
    PKE_RESP_NO_RESPONSE = 0x0, /* No response */
    PKE_RESP_WORD_VALUE  = 0x1, /* Parameter value ready (word or dword) */
    PKE_RESP_REJECTED    = 0x7, /* Request rejected / parameter not supported */
} KFC750_PkeResponse;

/* ------------------------------------------------------------------ */
/* Control Word (CW) — master → slave, PZD1                          */
/* ------------------------------------------------------------------ */

/* Bits 2:0 — motor command */
#define KFC750_CW_CMD_MASK          0x0007u
#define KFC750_CW_CMD_NO_ACTION     0x0000u /* No action — continue */
#define KFC750_CW_CMD_DECEL         0x0001u /* Decelerate to minimum frequency */
#define KFC750_CW_CMD_HOLD          0x0002u /* Hold current frequency */
#define KFC750_CW_CMD_ACCEL         0x0003u /* Accelerate to REF setpoint */
#define KFC750_CW_CMD_STOP          0x0004u /* Ramp to stop (freq → 0) */

/* Bit 7 — fault reset (acts on 0→1 rising edge) */
#define KFC750_CW_FAULT_RESET       0x0080u

/* Bit 10 — PROFIBUS control enable */
#define KFC750_CW_PROFIBUS_ENABLE   0x0400u

/* ------------------------------------------------------------------ */
/* Status Word (SW) — slave → master, PZD1                           */
/* ------------------------------------------------------------------ */

#define KFC750_SW_READY             0x0001u /* Drive ready, no fault */
#define KFC750_SW_RUNNING           0x0002u /* Motor running (freq > 0) */
#define KFC750_SW_FAULT             0x0008u /* Fault active */
#define KFC750_SW_AT_SPEED          0x0010u /* Frequency at setpoint */
#define KFC750_SW_REMOTE            0x0020u /* Under PROFIBUS control */
#define KFC750_SW_ZERO_SPEED        0x0040u /* At zero frequency */
#define KFC750_SW_ACCEL             0x0080u /* Ramping up */
#define KFC750_SW_DECEL             0x0100u /* Ramping down */
#define KFC750_SW_PROFIBUS_OK       0x0200u /* PROFIBUS comms healthy */

/* ------------------------------------------------------------------ */
/* Parameter IDs (selected)                                           */
/* ------------------------------------------------------------------ */

#define KFC750_PARAM_DESIRED_FREQ   101 /* Desired frequency Hz×100 */
#define KFC750_PARAM_MIN_FREQ       102 /* Minimum frequency Hz×100 */
#define KFC750_PARAM_BOOST_FREQ     103 /* Boost (start) frequency Hz×100 */
#define KFC750_PARAM_MAX_FREQ       104 /* Maximum frequency Hz×100 */
#define KFC750_PARAM_ACCEL_TIME     111 /* Acceleration time (tenths of seconds) */
#define KFC750_PARAM_DECEL_TIME     112 /* Deceleration time (tenths of seconds) */
#define KFC750_PARAM_CTRL_CENTER    122 /* Control source (0=keypad, 3=PROFIBUS) */
#define KFC750_PARAM_LOCK           166 /* Parameter lock (0=unlocked) */

#endif /* KFC750_PROTOCOL_H */
