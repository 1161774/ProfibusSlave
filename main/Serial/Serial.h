#ifndef SERIAL_H
#define SERIAL_H

/**
 * Serial.h
 * PROFIBUS physical layer — RS-485 UART configuration, response buffer
 * type, and public API.
 *
 * Hardware (match your board):
 *   UART2  RXD → GPIO22
 *          TXD → GPIO23
 *          RTS → GPIO18  (DE/~RE on RS-485 transceiver)
 *
 * Baud: 45,450 bps, 8E1 (even parity — PROFIBUS mandatory)
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ---- GPIO / UART port ------------------------------------------------- */
#define SERIAL_UART_PORT    (2)
#define SERIAL_TXD_PIN      (23)
#define SERIAL_RXD_PIN      (22)
#define SERIAL_RTS_PIN      (18)    /* DE/~RE */

/* ---- Baud rate --------------------------------------------------------- */
#define SERIAL_BAUD_RATE    (45450)

/* ---- Buffer / queue sizes ---------------------------------------------- */
#define SERIAL_RX_BUF       (512)
#define SERIAL_TX_BUF       (512)
#define SERIAL_RX_QUEUE_LEN (20)
#define SERIAL_TX_QUEUE_LEN (10)

/*
 * RX timeout: triggers UART_DATA event after silence >= this many
 * 10-bit-time units (~220 µs each at 45450 baud).
 * 4 units ≈ 880 µs — just over 3.5 character times (PROFIBUS inter-frame gap).
 */
#define SERIAL_RX_TOUT      (4)

/*
 * Maximum time to wait for the TX FIFO to drain after uart_write_bytes().
 * At 45450 baud a 50-byte frame takes ~12 ms; 25 ms is safe headroom.
 */
#define SERIAL_TX_DRAIN_MS  (25)

/* ---- Response frame buffer --------------------------------------------- */
/*
 * Maximum bytes in a single PROFIBUS response frame.
 * 6 mandatory diag + 57 max ext diag + 7 SD2 framing = 70 bytes worst case.
 * 256 gives comfortable headroom for any DP-V1 response.
 */
#define MAX_RESPONSE        (256)

/*
 * resp — response buffer passed through the TX queue by value.
 *
 * Data is embedded (not a pointer) so each queue slot owns its own copy
 * of the payload, eliminating the race where a second incoming frame
 * could overwrite the buffer before the TX task has sent the first one.
 */
typedef struct {
    uint8_t  Data[MAX_RESPONSE];
    uint16_t Length;
} resp;

/* ---- TX queue ---------------------------------------------------------- */
/*
 * txQueue is owned by Serial.c and written by Controller.c via xQueueSend.
 * Declared extern here so both modules share the handle without a circular
 * dependency.
 */
extern QueueHandle_t txQueue;

/* ---- API --------------------------------------------------------------- */

/**
 * Initialise the UART and start the RX and TX tasks.
 * Call after InitialiseController() and after all slaves have registered
 * (give slave tasks ~50 ms to call AddSlave before calling this).
 */
void Serial_Init(void);

#endif /* SERIAL_H */
