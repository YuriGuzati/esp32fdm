#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback invoked from USB RX ISR context — keep it fast. */
typedef void (*usb_serial_rx_cb_t)(const uint8_t *data, size_t len, void *user_ctx);

/**
 * Initialize USB Host stack and CDC-ACM/VCP drivers.
 * Starts background tasks on Core 1 for USB enumeration and device polling.
 * rx_cb is called from the CDC driver context when data arrives from the printer.
 */
esp_err_t usb_serial_init(usb_serial_rx_cb_t rx_cb, void *rx_cb_ctx);

/**
 * Send data to the connected USB serial device (printer).
 * Blocks until data is transmitted or returns error if no device connected.
 */
esp_err_t usb_serial_send(const uint8_t *data, size_t len);

/**
 * Set line coding (baud rate, data bits, parity, stop bits).
 * data_bits: 5-8, parity: 0=none 1=odd 2=even, stop_bits: 0=1bit 2=2bits
 */
esp_err_t usb_serial_set_line_coding(uint32_t baud_rate, uint8_t data_bits,
                                      uint8_t parity, uint8_t stop_bits);

/**
 * Set DTR and RTS control lines.
 */
esp_err_t usb_serial_set_control_lines(bool dtr, bool rts);

/**
 * Returns true if a USB serial device is currently connected and opened.
 */
bool usb_serial_is_connected(void);

/**
 * Ender Smart (lote 9): true when the open device is a USB-UART bridge (CH34x, CP210x,
 * FTDI), false for a native CDC-ACM device or when nothing is open.
 */
bool usb_serial_is_uart_bridge(void);

#ifdef __cplusplus
}
#endif
