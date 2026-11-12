#pragma once

#include "esp_err.h"
#include "layout.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PRINTER_DISCONNECTED,
    PRINTER_OPERATIONAL,
    PRINTER_PRINTING,
    PRINTER_PAUSED,
    PRINTER_ERROR,
} printer_opstate_t;

typedef struct {
    printer_opstate_t opstate;
    float hotend_actual, hotend_target;
    float bed_actual, bed_target;
    float progress_pct;          /* 0-100, or -1 if unknown */
    int32_t print_time_s;        /* elapsed print time, or -1 */
    int32_t print_time_left_s;   /* estimated remaining, or -1 */
    float x, y, z;
    float e;                     /* logical E from M114 (Ender Smart F15-09) */
    char filename[64];
    float layer_height;          /* from slicer metadata, 0 if unknown */
    float first_layer_height;    /* from slicer metadata, 0 if unknown */
    float object_height;         /* max Z from slicer metadata, 0 if unknown */
    int32_t total_layers;        /* computed from metadata, -1 if unknown */
    bool host_printing;          /* true if ESP32 is streaming GCode from SD */
    int32_t current_layer;       /* current layer from ;LAYER:N comments */
    float probe_z_offset;        /* Z probe offset from M290 query (Marlin) */
    int64_t last_update_us;      /* esp_timer_get_time() of last successful parse */
    char error_msg[96];          /* why opstate is PRINTER_ERROR (Ender Smart F15-06/08) */
} printer_state_t;

typedef enum {
    PCMD_PAUSE,      /* M25 (SD) or M0 */
    PCMD_RESUME,     /* M24 */
    PCMD_CANCEL,     /* M524 or abort sequence */
    PCMD_RAW,        /* raw gcode string */
} printer_cmd_type_t;

typedef struct {
    printer_cmd_type_t type;
    char gcode[96];  /* only used for PCMD_RAW */
    uint8_t sync_id; /* Ender Smart (F15-18): non-zero = a printer_comm_send_gcode_sync() waits for THIS command */
} printer_cmd_t;

/* Temperature history sample */
typedef struct {
    int64_t timestamp_us;       /* esp_timer_get_time() when recorded */
    float hotend_actual, hotend_target;
    float bed_actual, bed_target;
} temp_sample_t;

#define TEMP_HISTORY_MAX 900    /* 1 hour at 4s intervals */

/**
 * Record a temperature history sample with explicit values.
 * Called by both Marlin and Klipper backends after updating temps.
 */
void printer_comm_record_temp_sample(float hotend_actual, float hotend_target,
                                     float bed_actual, float bed_target);

/**
 * Copy temperature history samples oldest-first into buf.
 * Returns actual number of samples copied (up to TEMP_HISTORY_MAX).
 */
int printer_comm_get_temp_history(temp_sample_t *buf, int max_count);

/**
 * Initialize the printer communication module.
 * Starts a polling task that queries the printer over USB serial.
 * Must be called after usb_serial_init().
 */
esp_err_t printer_comm_init(void);

/**
 * USB serial RX callback — pass to usb_serial_init().
 * Feeds received data into the printer_comm parser.
 */
void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx);

/**
 * Thread-safe copy of current printer state.
 */
void printer_comm_get_state(printer_state_t *out);

/**
 * Queue a command to send to the printer.
 * Non-blocking; returns ESP_ERR_TIMEOUT if queue is full.
 */
esp_err_t printer_comm_send_cmd(const printer_cmd_t *cmd);

/**
 * Enable/disable simulation mode.
 * When enabled, get_state returns fake printer data (printing, temps, progress).
 */
void printer_comm_set_simulate(bool enable);

/**
 * Returns true if simulation mode is active.
 */
bool printer_comm_is_simulating(void);

/**
 * Get a human-readable name for the current backend ("Marlin" or "Klipper").
 */
const char *printer_comm_backend_name(void);

/**
 * Get the configured Moonraker host (empty string if not set).
 */
const char *printer_comm_get_mr_host(void);

/**
 * Get the configured Moonraker port.
 */
uint16_t printer_comm_get_mr_port(void);

/**
 * Save printer backend configuration to NVS.
 * Takes effect after reboot.
 */
esp_err_t printer_comm_save_config(printer_backend_t backend,
                                   const char *mr_host, uint16_t mr_port,
                                   bool pause_abort);

/**
 * Register printer config HTTP endpoints on the given server.
 * Called from httpd.c during server setup.
 */
esp_err_t printer_config_register_httpd(void *server_handle);

/**
 * Render backend selection section (Marlin/Klipper, Moonraker fields, Save & Reboot).
 */
void printer_config_render_backend(html_buf_t *p);

/**
 * Render Marlin-specific settings section (pause command, Save).
 */
void printer_config_render_marlin(html_buf_t *p);

/* ---- G-code file info extraction (see gcode_scan.h) ---- */
#include "gcode_scan.h"

/**
 * Start host-based GCode printing from a file on /sdcard.
 * The file is streamed line-by-line to the printer via USB serial.
 */
esp_err_t printer_comm_host_print_start(const char *filename);

/**
 * Pause host print (stops sending lines, heaters stay on).
 */
esp_err_t printer_comm_host_print_pause(void);

/**
 * Resume host print after pause.
 */
esp_err_t printer_comm_host_print_resume(void);

/**
 * Cancel host print (closes file, turns off heaters, disables motors).
 */
esp_err_t printer_comm_host_print_cancel(void);

/**
 * Returns true if a host print is currently active (printing or paused).
 */
bool printer_comm_is_host_printing(void);

/**
 * Ender Smart (F15-10): server-side policy for raw G-code coming from the UI/API.
 * While a print is running or paused only status queries and safe fine-tuning
 * (fan, speed/flow, babystep, adjusting an already-on heater, cooling) are allowed.
 * Returns false and fills reason when the command is refused.
 */
bool printer_comm_cmd_allowed(const char *gcode, char *reason, size_t reason_len);

/**
 * Ender Smart (F15-13): true while a print is running or paused (host print or
 * printer SD). OTA, reboot, Wi-Fi reset, bed probing and RFC 2217 are refused then.
 */
bool printer_comm_job_active(void);

/**
 * Suppress automatic polling (M105/M27/M114/M119).
 * Used by terminal manual mode to avoid response clutter.
 */
void printer_comm_set_polling_suppressed(bool suppress);
bool printer_comm_is_polling_suppressed(void);

/**
 * Set a callback invoked for every line received from the printer.
 * Called from the comm task context.  Pass NULL to clear.
 */
void printer_comm_set_line_callback(void (*cb)(const char *line));

/**
 * Send a raw GCode command and block until the printer responds with "ok".
 * Only one caller at a time.  Returns ESP_ERR_TIMEOUT if no response within timeout_ms.
 */
esp_err_t printer_comm_send_gcode_sync(const char *gcode, int timeout_ms);

#ifdef __cplusplus
}
#endif
