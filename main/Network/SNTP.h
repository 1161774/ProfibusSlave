#ifndef SNTP_H
#define SNTP_H

/**
 * SNTP.h
 * SNTP time synchronisation — initialisation and startup-sync helper.
 *
 * After SNTP_Init() the ESP-IDF SNTP client polls the configured NTP server
 * periodically (see CONFIG_SNTP_SYNC_INTERVAL_S).  Corrections are applied
 * in smooth mode so timestamps in logs never jump.
 *
 * Call SNTP_WaitSync() once during startup to allow the first sync to
 * complete before the rest of the system logs timestamped events.  If the
 * network is not yet available the call returns false after the timeout —
 * the system continues running and the SNTP client will sync automatically
 * once the WiFi link is established.
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialise the SNTP client and start periodic polling.
 * Must be called after WiFi_Init() (the default event loop must exist).
 * Returns immediately — the first synchronisation is asynchronous.
 */
void SNTP_Init(void);

/**
 * Block until the first NTP synchronisation completes or the timeout expires.
 *
 * @param timeout_ms  Maximum time to wait, in milliseconds.
 * @return  true  — synchronisation succeeded within the timeout.
 *          false — timeout elapsed with no successful sync.
 *
 * A false return is not fatal.  The SNTP client continues polling and will
 * apply the first correction as soon as the server is reachable.
 */
bool SNTP_WaitSync(uint32_t timeout_ms);

#endif /* SNTP_H */
