#ifndef WIFI_H
#define WIFI_H

/**
 * WiFi.h
 * WiFi station — asynchronous connection and auto-reconnect.
 *
 * Credentials are configured in menuconfig under "Network Configuration":
 *   CONFIG_WIFI_SSID      SSID of the access point
 *   CONFIG_WIFI_PASSWORD  WPA/WPA2 passphrase (empty string = open network)
 *
 * WiFi_Init() returns immediately after starting the connection attempt.
 * The association and DHCP lease complete asynchronously.  The supplied
 * callback is invoked each time an IP address is acquired (initial connect
 * and every reconnect after a drop).
 */

/**
 * Callback invoked from the system event loop each time an IP address is
 * obtained.  Keep it short — it runs inside the event-loop task.
 */
typedef void (*wifi_ip_cb_t)(void);

/**
 * Initialise NVS flash, the TCP/IP stack, and the WiFi station driver.
 * Starts the connection attempt asynchronously.
 *
 * @param on_ip_acquired  Called each time a DHCP lease is obtained.
 *                        Pass NULL if no notification is needed.
 */
void WiFi_Init(wifi_ip_cb_t on_ip_acquired);

#endif /* WIFI_H */
