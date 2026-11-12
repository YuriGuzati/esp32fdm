#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

typedef enum {
    WIFI_RESULT_STA_CONNECTED,  /* connected to the saved network */
    WIFI_RESULT_STA_PENDING,    /* saved network unreachable: retrying in background,
                                   emergency AP open for a while (Ender Smart F15-03) */
    WIFI_RESULT_AP_MODE,        /* no credentials: setup AP + captive portal */
} wifi_result_t;

/**
 * Initialize WiFi: try STA with NVS/Kconfig creds. With credentials it never
 * gives up (reconnects forever with exponential backoff); without credentials
 * it starts the setup AP.
 */
wifi_result_t wifi_init(void);

/** True while the STA has an IP address. */
bool wifi_is_connected(void);

/** Block until the STA has an IP address or the timeout expires. */
bool wifi_wait_connected(TickType_t ticks);

/** RSSI of the current AP. Returns false when not connected. */
bool wifi_get_rssi(int8_t *rssi);

/** True while the temporary emergency AP is open. */
bool wifi_emergency_ap_active(void);

/** Number of successful reconnections since boot (after the first connection). */
uint32_t wifi_get_reconnect_count(void);

/**
 * Get the current IP address as a string.
 * Returns pointer to a static buffer — valid until next call.
 */
const char *wifi_get_ip_str(void);

/**
 * Get the stored WiFi SSID from NVS.
 * Copies into buf (up to buf_size-1 chars + NUL). Returns true if found.
 */
bool wifi_get_ssid(char *buf, size_t buf_size);

/**
 * Erase stored WiFi credentials from NVS and reboot.
 */
void wifi_reset_credentials(void);

/**
 * Get the device hostname (used for DHCP and mDNS).
 * Returns pointer to a static buffer — always valid, never NULL.
 */
const char *wifi_get_hostname(void);

/**
 * Set the device hostname. Saves to NVS and takes effect on next reboot.
 */
void wifi_set_hostname(const char *name);
