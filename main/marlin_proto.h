#pragma once

/*
 * Internal interface between printer_comm.c (Marlin protocol layer)
 * and host_print.c (host print state machine).
 *
 * Not part of the public API — only included by these two files.
 */

#include "printer_comm.h"
#include "usb_serial.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Query types (shared between protocol layer and host print) ---- */

typedef enum {
    QUERY_NONE,
    QUERY_M105,
    QUERY_M27,
    QUERY_M27C,
    QUERY_M114,
    QUERY_M119,
    QUERY_CMD,
} query_type_t;

/* ---- Shared constants ---- */

/* Internal command types for host print (sent via s_cmd_queue) */
#define PCMD_HOST_START  100
#define PCMD_HOST_PAUSE  101
#define PCMD_HOST_RESUME 102
#define PCMD_HOST_CANCEL 103

/* Host print command timeout: must be longer than the M113 keepalive
 * interval (2s) so Marlin has a chance to send "busy: processing"
 * before we give up and retry. */
#define HOST_CMD_TIMEOUT_MS  3000

/* Unsupported command tracking */
#define UNSUPPORTED_CMD_MAX 16

/* Ender Smart (F15-07): longest line Marlin accepts (MAX_CMD_SIZE 96 incl. NUL).
 * Longer lines are refused instead of being truncated or overflowing a buffer. */
#define MARLIN_MAX_LINE 95

/* ---- Shared state (owned by printer_comm.c) ---- */

/** Printer state and its mutex */
extern SemaphoreHandle_t g_state_mutex;
extern printer_state_t   g_state;

/** Cooldown timestamp — set by busy: handler, checked by host_print to defer */
extern int64_t s_cmd_cooldown_until_us;

/** Commands Marlin reported as unknown (numeric codes, e.g. 73 for M73) */
extern uint16_t s_unsupported_cmds[UNSUPPORTED_CMD_MAX];
extern int s_unsupported_count;

/* ---- Functions exported by printer_comm.c for host_print.c ---- */

/** Send a GCode command and block until "ok" or timeout.
 *  Returns true if "ok" was received.  Standard timeout, no stall pings. */
bool marlin_send_cmd(const char *gcode, query_type_t qtype);

/** Host print variant: shorter timeout (3s), with CDC stall ping retries. */
bool marlin_send_cmd_hp(const char *gcode, query_type_t qtype);

/** Process any buffered RX data (auto-reports, etc.) */
void marlin_drain_rx(void);

/** Check if USB serial is currently connected */
static inline bool marlin_is_connected(void)
{
    return usb_serial_is_connected();
}

/* ---- Functions exported by host_print.c for printer_comm.c ---- */

/** Initialize host print module (called once from printer_comm_init) */
void hp_init(void);

/** Returns true if a host print is active (printing or paused) */
bool hp_is_active(void);

/** Returns true if host print has a deferred pending line */
bool hp_has_pending(void);

/** Returns true if host print is paused */
bool hp_is_paused(void);

/** Command handlers (called from printer_comm_task queue dispatch) */
void hp_cmd_start(const char *filename);
void hp_cmd_pause(void);
void hp_cmd_resume(void);
void hp_cmd_cancel(void);

/** Run one iteration of the host print streaming loop.
 *  Sends up to 4 GCode lines per call. */
void hp_tick(void);

/** Notifications from process_line() RX parsing */
void hp_notify_resend(int32_t line_num);

/** Ender Smart (F15-06): Marlin boot banner seen — the printer restarted. */
void hp_notify_printer_reset(void);

/** Ender Smart (F15-06): USB link lost while a host print is active. */
void hp_notify_usb_lost(void);

/** Fill printer_state_t fields related to host print progress.
 *  Called while g_state_mutex is held. */
void hp_fill_state(printer_state_t *out);

/** Public API pass-throughs */
esp_err_t hp_start(const char *filename);
esp_err_t hp_pause(void);
esp_err_t hp_resume(void);
esp_err_t hp_cancel(void);

#ifdef __cplusplus
}
#endif
