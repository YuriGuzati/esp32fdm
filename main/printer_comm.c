#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "httpd.h"

#include "usb_serial.h"
#include "terminal.h"
#include "url_util.h"

#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "layout.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cache.h"
#include "gcode_scan.h"
#include "marlin_proto.h"

static const char *TAG = "printer_comm";

/* ---- Configuration ---- */

#define POLL_M105_INTERVAL_MS           4000
#define POLL_M105_HOSTPRINT_INTERVAL_MS 30000   /* Reduced frequency during host print */
#define POLL_M27_INTERVAL_MS            10000
#define POLL_M114_INTERVAL_MS           30000
#define POLL_M119_INTERVAL_MS           30000   /* Filament sensor check during host print */
#define CMD_RESPONSE_TIMEOUT_MS         6000
#define WAIT_SILENT_TIMEOUT_MS          240000  /* G28/G29: can be silent for minutes, needs long timeout */
#define WAIT_TEMP_TIMEOUT_MS            30000   /* M109/M190/M116: first temp report can be delayed */
#define HOST_STALL_PING_RETRIES         5       /* Echo-ping retries before giving up on a stalled cmd */
#define RX_LINE_BUF_SIZE        256
#define CMD_QUEUE_SIZE          8

/* ---- Line callback (used by bedlevel mesh parsing) ---- */
static void (*s_line_callback)(const char *line);

/* ---- Synchronous command send ---- */
static SemaphoreHandle_t s_sync_cmd_sem;
/* Ender Smart (F15-18 / A2): the old single "pending" flag was released by ANY raw
 * command finishing and ignored the result, so a failed G28 still started G29. */
static volatile uint8_t   s_sync_wait_id;   /* id of the command a caller waits for */
static volatile bool      s_sync_ok;        /* "ok" received for that command */
static uint8_t            s_sync_seq;
/* F15-23: "Resend: N" seen while sending a user command */
static volatile int32_t   s_last_resend = -1;
static uint32_t           s_spam_lines;     /* Creality debug lines dropped */

#define NVS_NAMESPACE           "pcomm"
#define NVS_KEY_BACKEND         "backend"
#define NVS_KEY_MR_HOST         "mr_host"
#define NVS_KEY_MR_PORT         "mr_port"
#define NVS_KEY_PAUSE_CMD       "pause_cmd"
#define NVS_KEY_FILAMENT_CHK    "fil_chk"
#define NVS_KEY_ECHO_PING       "echo_ping"

/* ---- Backend config ---- */

typedef enum {
    PAUSE_CMD_M25  = 0,  /* Pause SD print (can resume) */
    PAUSE_CMD_M524 = 1,  /* Abort SD print (workaround for buggy firmware) */
} pause_cmd_t;

static printer_backend_t s_backend = PRINTER_BACKEND_MARLIN;
static char s_mr_host[64] = "";
static uint16_t s_mr_port = 7125;
static pause_cmd_t s_pause_cmd = PAUSE_CMD_M25;
static bool s_filament_check = false;  /* Poll M119 for filament runout during host print */
static char s_echo_ping_cmds[128] = "";  /* Comma-separated GCode prefixes that need CDC flush ping */

/* ---- Temperature history circular buffer ---- */

static temp_sample_t *s_temp_history;  /* allocated in PSRAM */
static int s_temp_head;   /* next write index */
static int s_temp_count;  /* number of valid samples */

/* ---- State (Marlin backend) ---- */

printer_state_t g_state;
SemaphoreHandle_t g_state_mutex;

static StreamBufferHandle_t s_rx_stream;
static QueueHandle_t s_cmd_queue;

/* Line buffer for assembling responses */
static char s_line_buf[RX_LINE_BUF_SIZE];
static size_t s_line_len;

/* Simulation mode */
#if CONFIG_ESP32FDM_SIMULATION
static bool s_simulate = false;
#else
/* Ender Smart (F15-05): simulation compiled out; the branches below are dead code. */
static const bool s_simulate = false;
#endif
static int64_t s_sim_start_us;
static printer_opstate_t s_sim_opstate;
static int64_t s_sim_pause_elapsed_us;  /* accumulated elapsed time before pause */

/* Tracking which query we're waiting for (query_type_t defined in marlin_proto.h) */
static query_type_t s_pending_query;

/* Polling suppression (terminal manual mode) */
static bool s_polling_suppressed;

/* Timestamps for polling intervals */
static int64_t s_last_m105_us;
static int64_t s_last_m27_us;
static int64_t s_last_m114_us;
static int64_t s_last_m119_us;
static bool s_filament_sensor_present;  /* true once M119 reports a filament: line */

/* One-shot M290 query on connect */
static bool s_m290_queried;

/* Print start time tracking */
static int64_t s_print_start_us;
static bool s_need_filename;
static bool s_have_m73;        /* true once we've seen M73 — prefer slicer progress over M27 bytes */

/* Cooldown after pause/resume/cancel — let firmware settle */
int64_t s_cmd_cooldown_until_us;

/* Unsupported command tracking — commands that Marlin reports as unknown.
 * Store just the numeric code (e.g. 73 for M73, 900 for M900). */
uint16_t s_unsupported_cmds[UNSUPPORTED_CMD_MAX];
int s_unsupported_count;

/** Check if a GCode command matches one of the echo-ping prefixes. */
static bool needs_echo_ping(const char *gcode)
{
    if (s_echo_ping_cmds[0] == '\0') return false;
    const char *cmd = gcode;
    if ((cmd[0] == 'N' || cmd[0] == 'n') && cmd[1] >= '0' && cmd[1] <= '9') {
        const char *sp = strchr(cmd, ' ');
        if (sp) cmd = sp + 1;
    }
    char buf[128];
    strncpy(buf, s_echo_ping_cmds, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';
    char *tok = strtok(buf, ", ");
    while (tok) {
        while (*tok == ' ') tok++;
        size_t tlen = strlen(tok);
        if (tlen > 0 && strncasecmp(cmd, tok, tlen) == 0) {
            char next = cmd[tlen];
            if (next == '\0' || next == ' ' || next == '*') return true;
        }
        tok = strtok(NULL, ", ");
    }
    return false;
}

/** Find value after a key character, e.g. 'T' in "T:205.3 /210.0" */
static bool parse_temp_pair(const char *line, char key,
                            float *actual, float *target)
{
    /* Look for "X:" pattern */
    char pattern[4];
    snprintf(pattern, sizeof(pattern), "%c:", key);
    const char *p = strstr(line, pattern);
    if (!p) return false;

    p += 2; /* skip "X:" */
    *actual = strtof(p, (char **)&p);

    /* Look for " /NNN" target temperature */
    while (*p == ' ') p++;
    if (*p == '/') {
        p++;
        *target = strtof(p, NULL);
    }
    return true;
}

void printer_comm_record_temp_sample(float hotend_actual, float hotend_target,
                                     float bed_actual, float bed_target)
{
    temp_sample_t *sample = &s_temp_history[s_temp_head];
    sample->timestamp_us = esp_timer_get_time();
    sample->hotend_actual = hotend_actual;
    sample->hotend_target = hotend_target;
    sample->bed_actual = bed_actual;
    sample->bed_target = bed_target;
    s_temp_head = (s_temp_head + 1) % TEMP_HISTORY_MAX;
    if (s_temp_count < TEMP_HISTORY_MAX) s_temp_count++;
}

/** Parse M105 response: "ok T:205.3 /210.0 B:60.1 /60.0" */
static void parse_m105(const char *line)
{
    float ha, ht, ba, bt;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (parse_temp_pair(line, 'T', &ha, &ht)) {
        g_state.hotend_actual = ha;
        g_state.hotend_target = ht;
    }
    if (parse_temp_pair(line, 'B', &ba, &bt)) {
        g_state.bed_actual = ba;
        g_state.bed_target = bt;
    }

    g_state.last_update_us = esp_timer_get_time();

    printer_comm_record_temp_sample(g_state.hotend_actual, g_state.hotend_target,
                                    g_state.bed_actual, g_state.bed_target);

    /* Update opstate from temps if not already printing */
    if (g_state.opstate == PRINTER_DISCONNECTED) {
        g_state.opstate = PRINTER_OPERATIONAL;
    }

    xSemaphoreGive(g_state_mutex);
}

/** Parse M27 response: "SD printing byte 12345/67890" or "Not SD printing" */
static void parse_m27(const char *line)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    const char *sd = strstr(line, "SD printing byte");
    if (sd) {
        long pos = 0, total = 0;
        if (sscanf(sd, "SD printing byte %ld/%ld", &pos, &total) == 2 && total > 0) {

            if (g_state.opstate != PRINTER_PAUSED) {
                if (g_state.opstate != PRINTER_PRINTING) {
                    s_print_start_us = esp_timer_get_time();
                    s_need_filename = true;
                    s_have_m73 = false;
                }
                g_state.opstate = PRINTER_PRINTING;
            }

            /* M27 byte-position progress — use as fallback if slicer doesn't embed M73 */
            if (!s_have_m73) {
                g_state.progress_pct = (float)pos / (float)total * 100.0f;

                int64_t elapsed_us = esp_timer_get_time() - s_print_start_us;
                g_state.print_time_s = (int32_t)(elapsed_us / 1000000);
                if (g_state.progress_pct > 0.1f) {
                    float total_est = (float)g_state.print_time_s / (g_state.progress_pct / 100.0f);
                    g_state.print_time_left_s = (int32_t)(total_est - g_state.print_time_s);
                    if (g_state.print_time_left_s < 0) g_state.print_time_left_s = 0;
                }
            } else {
                /* Still track elapsed time from wall clock */
                int64_t elapsed_us = esp_timer_get_time() - s_print_start_us;
                g_state.print_time_s = (int32_t)(elapsed_us / 1000000);
            }

            /* Estimate object height and layers from Z + progress */
            if (g_state.z > 0.1f && g_state.progress_pct > 1.0f) {
                g_state.object_height = g_state.z / (g_state.progress_pct / 100.0f);
                float lh = g_state.layer_height > 0 ? g_state.layer_height : 0.2f;
                g_state.total_layers = (int32_t)(g_state.object_height / lh + 0.5f);
                /* Estimate current layer from Z */
                float flh = g_state.first_layer_height > 0 ? g_state.first_layer_height : lh;
                if (g_state.z <= flh)
                    g_state.current_layer = 1;
                else
                    g_state.current_layer = 1 + (int32_t)((g_state.z - flh) / lh + 0.5f);
            }
        }
    } else if (strstr(line, "Not SD printing")) {
        if (g_state.opstate == PRINTER_PRINTING || g_state.opstate == PRINTER_PAUSED) {
            g_state.opstate = PRINTER_OPERATIONAL;
            g_state.progress_pct = -1;
            g_state.print_time_s = -1;
            g_state.print_time_left_s = -1;
            g_state.filename[0] = '\0';
            g_state.object_height = 0;
            g_state.total_layers = -1;
        }
    }

    xSemaphoreGive(g_state_mutex);
}

/** Parse M27 C response: "Current file: filename.gcode" */
static void parse_m27c(const char *line)
{
    const char *prefix = "Current file: ";
    const char *p = strstr(line, prefix);
    if (!p) return;

    p += strlen(prefix);
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    strncpy(g_state.filename, p, sizeof(g_state.filename) - 1);
    g_state.filename[sizeof(g_state.filename) - 1] = '\0';
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG, "Current file: %s", g_state.filename);
}

/** Parse M114 response: "X:100.00 Y:50.00 Z:0.30 E:123.45" */
static void parse_m114(const char *line)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    const char *p;
    p = strstr(line, "X:");
    if (p) g_state.x = strtof(p + 2, NULL);
    p = strstr(line, "Y:");
    if (p) g_state.y = strtof(p + 2, NULL);
    p = strstr(line, "Z:");
    if (p) g_state.z = strtof(p + 2, NULL);
    p = strstr(line, "E:");
    if (p) g_state.e = strtof(p + 2, NULL);

    xSemaphoreGive(g_state_mutex);
}

/** Parse M119 response line for filament sensor.
 *  Marlin reports: "filament: TRIGGERED" (filament present) or "filament: open" (runout).
 *  Only acts during host printing — triggers auto-pause on runout. */
static void parse_m119(const char *line)
{
    const char *p = strstr(line, "filament:");
    if (!p) return;

    s_filament_sensor_present = true;
    const char *val = p + 9;
    while (*val == ' ') val++;

    if (strncasecmp(val, "open", 4) == 0) {
        /* Filament runout detected */
        if (hp_is_active() && !hp_is_paused()) {
            ESP_LOGW(TAG, "Filament runout detected via M119 — pausing host print");
            printer_cmd_t cmd = { .type = PCMD_PAUSE };
            xQueueSend(s_cmd_queue, &cmd, 0);
        }
    }
}

/** Process one complete line from the printer */
static void process_line(const char *line)
{
    /* Skip empty lines */
    if (line[0] == '\0') return;

    /* Ender Smart (F15-23): the Creality firmware prints debug garbage on the serial
     * port while the Z offset is adjusted on its screen ("d:babystep",
     * "b:%d2,dis:%d", hundreds of lines per adjustment). Drop it. */
    if (strstr(line, "d:babystep") || strstr(line, "dis:%d")) {
        if ((s_spam_lines++ % 200) == 0)
            ESP_LOGW(TAG, "Dropping Creality babystep debug lines (%lu so far)",
                     (unsigned long)s_spam_lines);
        return;
    }

    /* Forward line to callback (used by bedlevel mesh parser) */
    if (s_line_callback) {
        s_line_callback(line);
    }

    ESP_LOGI(TAG, "RX: %s", line);

    /* Ender Smart (F15-06): Marlin boot banner means the printer restarted (power
     * glitch, reset). Seen on the Ender-3 Max Neo: "start" (often garbled),
     * "DWIN handshake error.", "BL24CXX Check succeeded!". Any host print must stop. */
    if (strcmp(line, "start") == 0 || strstr(line, "DWIN handshake") ||
        strstr(line, "BL24CXX Check")) {
        ESP_LOGW(TAG, "Printer restart detected (\"%s\")", line);
        hp_notify_printer_reset();
    }

    /* Content-based matching — handles unsolicited auto-reports too */
    if (strstr(line, "T:")) {
        parse_m105(line);
    } else if (strstr(line, "SD printing") || strstr(line, "Not SD")) {
        parse_m27(line);
    } else if (strstr(line, "Current file:")) {
        parse_m27c(line);
    } else if (strstr(line, "X:") && strstr(line, "Y:") && strstr(line, "Z:")) {
        parse_m114(line);
    } else if (strstr(line, "filament:")) {
        parse_m119(line);
    }

    /* Parse M290 response: "echo:Probe Offset Z-1.69" */
    const char *poff = strstr(line, "Probe Offset Z");
    if (poff) {
        float z = strtof(poff + 14, NULL);
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_state.probe_z_offset = z;
        xSemaphoreGive(g_state_mutex);
        ESP_LOGI(TAG, "Probe Z offset: %.2f", z);
    }

    /* Detect unsupported commands: echo:Unknown command: "M73 P15 R21" */
    const char *unk = strstr(line, "Unknown command:");
    if (unk) {
        /* Find the M/G code in the quoted string */
        const char *q = strchr(unk, '"');
        if (q && (q[1] == 'M' || q[1] == 'G')) {
            uint16_t code = (uint16_t)atoi(q + 2);
            /* Add to unsupported list if not already there */
            bool found = false;
            for (int i = 0; i < s_unsupported_count; i++) {
                if (s_unsupported_cmds[i] == code) { found = true; break; }
            }
            if (!found && s_unsupported_count < UNSUPPORTED_CMD_MAX) {
                s_unsupported_cmds[s_unsupported_count++] = code;
                ESP_LOGW(TAG, "Marking %c%u as unsupported",
                         q[1], code);
            }
        }
    }

    /* M73 from slicer — Marlin echoes "echo:Unknown command: "M73 P15 R21"" */
    const char *m73 = strstr(line, "M73");
    if (m73) {
        int pct = -1;
        int mins = -1;
        const char *pp = strstr(m73, "P");
        if (pp) pct = atoi(pp + 1);
        const char *rp = strstr(m73, "R");
        if (rp) mins = atoi(rp + 1);
        if (pct >= 0 && pct <= 100) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            s_have_m73 = true;
            g_state.progress_pct = (float)pct;
            if (mins >= 0) {
                g_state.print_time_left_s = mins * 60;
            }
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "M73: progress=%d%% remaining=%dmin", pct, mins);
        }
    }

    /* Marlin safety errors — cancel print immediately.
     *
     * Fatal (printer halts, ignores further commands):
     *   "Error:Thermal Runaway, system stopped! Heater_ID: ..."
     *   "Error:Heating failed, system stopped! Heater_ID: ..."
     *   "Error:MAXTEMP triggered, system stopped! Heater_ID: ..."
     *   "Error:MINTEMP triggered, system stopped! Heater_ID: ..."
     *   "Error:Thermal Malfunction, system stopped! Heater_ID: ..."
     *   "Error:Printer halted. kill() called!"
     *   "Error:Printer stopped due to errors. ..."
     *
     * Non-fatal (printer keeps running but output is garbage):
     *   "echo: cold extrusion prevented"
     *   "echo: too long extrusion prevented"
     */
    {
        bool fatal = false;
        bool safety_err = false;

        if (strncmp(line, "Error:", 6) == 0) {
            const char *msg = line + 6;
            if (strstr(msg, "Thermal Runaway") ||
                strstr(msg, "Heating failed") ||
                strstr(msg, "MAXTEMP triggered") ||
                strstr(msg, "MINTEMP triggered") ||
                strstr(msg, "Thermal Malfunction") ||
                strstr(msg, "Printer halted") ||
                strstr(msg, "Printer stopped")) {
                fatal = true;
                safety_err = true;
            }
        } else if (strstr(line, "cold extrusion prevented") ||
                   strstr(line, "too long extrusion prevented")) {
            safety_err = true;
        }

        if (safety_err) {
            if (hp_is_active()) {
                ESP_LOGE(TAG, "Safety error — aborting host print: %s", line);
                printer_cmd_t cmd = { .type = PCMD_HOST_CANCEL };
                xQueueSend(s_cmd_queue, &cmd, 0);
            } else if (g_state.opstate == PRINTER_PRINTING) {
                ESP_LOGE(TAG, "Safety error — cancelling SD print: %s", line);
                printer_cmd_t cmd = { .type = PCMD_CANCEL };
                xQueueSend(s_cmd_queue, &cmd, 0);
            }
            if (fatal) {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.opstate = PRINTER_ERROR;
                xSemaphoreGive(g_state_mutex);
            }
            return;
        }
    }

    /* Marlin action commands: //action:pause, //action:out_of_filament, //action:resume */
    if (strncmp(line, "//action:", 9) == 0) {
        const char *action = line + 9;
        if (strcmp(action, "pause") == 0 || strcmp(action, "out_of_filament") == 0) {
            if (hp_is_active() && !hp_is_paused()) {
                ESP_LOGW(TAG, "Printer requested pause: %s", action);
                printer_cmd_t cmd = { .type = PCMD_PAUSE };
                xQueueSend(s_cmd_queue, &cmd, 0);
            }
        } else if (strcmp(action, "resume") == 0) {
            if (hp_is_active() && hp_is_paused()) {
                ESP_LOGI(TAG, "Printer requested resume");
                printer_cmd_t cmd = { .type = PCMD_RESUME };
                xQueueSend(s_cmd_queue, &cmd, 0);
            }
        }
        return;
    }

    /* Marlin says "busy: processing" when it can't accept commands yet */
    if (strstr(line, "busy:")) {
        s_cmd_cooldown_until_us = esp_timer_get_time() + 4000000LL; /* back off 4s */
        s_pending_query = QUERY_NONE;
        return;
    }

    /* G29 bed leveling in progress — Marlin sends "measured_z:" for each probe
     * point instead of "ok".  Back off longer than normal busy to span the
     * gap between probes (~6-8 s).  The final "ok" after the mesh grid
     * clears the cooldown and resumes polling. */
    if (strstr(line, "measured_z:")) {
        s_cmd_cooldown_until_us = esp_timer_get_time() + 10000000LL; /* 10 s */
        s_pending_query = QUERY_NONE;
        return;
    }

    /* Marlin resend request: "Resend: N123" or "rs N123" */
    if (strncasecmp(line, "resend", 6) == 0 || strncmp(line, "rs ", 3) == 0) {
        const char *p = line;
        while (*p && (*p < '0' || *p > '9')) p++; /* skip to first digit */
        if (*p) {
            int32_t rline = (int32_t)atol(p);
            hp_notify_resend(rline);
            s_last_resend = rline;
            ESP_LOGW(TAG, "Resend requested: N%ld", (long)rline);
        }
        /* Don't return — Marlin sends "ok" after "Resend:" which clears s_pending_query */
    }

    /* "ok" signals end of response for current query.
     * Marlin often sends "ok T:205.3 /210.0 B:60.1 /60.0" — temp data on the ok line. */
    if (strncmp(line, "ok", 2) == 0) {
        if (strstr(line, "T:")) {
            parse_m105(line);
        }
        s_pending_query = QUERY_NONE;
        /* Clear cooldown so the wait-loop doesn't mistake this real "ok"
         * for a "busy: processing" clear and re-arm the query. */
        s_cmd_cooldown_until_us = 0;
    }
}

/** Drain buffered RX data, processing any complete lines (e.g. auto-reports) */
void marlin_drain_rx(void)
{
    uint8_t rx_byte;
    while (xStreamBufferReceive(s_rx_stream, &rx_byte, 1, 0) > 0) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                process_line(s_line_buf);
                s_line_len = 0;
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }
}

/** Check if a GCode line is a long-running command that blocks until done.
 *  M109=wait hotend, M190=wait bed, M116=wait all temps — send temp reports while waiting.
 *  G29=bed leveling — sends probe measurements, temp auto-reports while running.
 *  G4=dwell, G28=homing — produce NO output while running (truly silent).
 *  These need extended timeouts.
 *  If is_silent is non-NULL, sets it to true for commands that produce no interim feedback.
 *  If timeout_override_ms is non-NULL, sets it to a command-specific timeout (0 = use default). */
static bool is_wait_cmd(const char *gcode, bool *is_silent, int *timeout_override_ms)
{
    if (is_silent) *is_silent = false;
    if (timeout_override_ms) *timeout_override_ms = 0;
    /* Skip line number prefix "N123 " */
    const char *p = gcode;
    if (*p == 'N' || *p == 'n') {
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
    }
    if (*p == 'M') {
        int code = atoi(p + 1);
        return (code == 109 || code == 190 || code == 116);
    }
    if (*p == 'G') {
        int code = atoi(p + 1);
        if (code == 29) {
            /* G29 bed leveling: can be silent for minutes between temp
             * stabilization and first probe point (homing in between).
             * Treat as silent for long initial + extension timeout. */
            if (is_silent) *is_silent = true;
            return true;
        }
        if (code == 28) {
            /* G28 homing — truly silent, needs long initial timeout. */
            if (is_silent) *is_silent = true;
            return true;
        }
        if (code == 4) {
            /* G4 Snnn (seconds) or Pnnn (milliseconds) — parse dwell time */
            if (is_silent) *is_silent = true;
            if (timeout_override_ms) {
                int dwell_ms = 0;
                const char *s = strchr(p, 'S');
                if (!s) s = strchr(p, 's');
                const char *pm = strchr(p, 'P');
                if (!pm) pm = strchr(p, 'p');
                if (s) dwell_ms = atoi(s + 1) * 1000;
                else if (pm) dwell_ms = atoi(pm + 1);
                /* 20% margin (min 3s) for command processing overhead */
                int margin = dwell_ms / 5;
                if (margin < 3000) margin = 3000;
                *timeout_override_ms = dwell_ms + margin;
            }
            return true;
        }
    }
    return false;
}

/** Send a gcode command to the printer, wait for response lines.
 *  timeout_override_ms: >0 forces that timeout (0 = auto-detect from command type).
 *  max_stall_pings: echo pings to send on timeout before giving up (0 = no pings). */
static bool marlin_send_cmd_ex(const char *gcode, query_type_t qtype,
                                int timeout_override_ms, int max_stall_pings)
{
    if (!usb_serial_is_connected()) return false;

    /* Process any buffered unsolicited data before sending */
    s_pending_query = QUERY_NONE;
    marlin_drain_rx();

    /* Abort if drain_rx saw busy: and set cooldown */
    if (esp_timer_get_time() < s_cmd_cooldown_until_us) return false;

    /* Ender Smart (F15-07 / C9): the old code used a 100-byte buffer but sent the
     * *intended* length, leaking stack bytes for long commands. Refuse instead. */
    size_t glen = strlen(gcode);
    if (glen > MARLIN_MAX_LINE) {
        ESP_LOGE(TAG, "Refusing command longer than %d chars (%u): %.40s...",
                 MARLIN_MAX_LINE, (unsigned)glen, gcode);
        return false;
    }

    s_pending_query = qtype;
    s_line_len = 0;

    /* Send command with newline */
    char buf[MARLIN_MAX_LINE + 2];
    int len = snprintf(buf, sizeof(buf), "%s\n", gcode);
    ESP_LOGI(TAG, "TX: %s", gcode);
    terminal_feed_tx(gcode);
    esp_err_t err = usb_serial_send((const uint8_t *)buf, len);
    if (err != ESP_OK) {
        s_pending_query = QUERY_NONE;
        return false;
    }

    /* Brief pause after TX — gives STM32 CDC time to process the USB
     * OUT packet before we start polling for the response. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Echo ping: some STM32 CDC implementations hold TX data until the
     * next OUT transaction.  Send a harmless M118 to flush the response.
     * The "echo:ping" reply is ignored by process_line. */
    if (needs_echo_ping(gcode)) {
        const char *ping = "M118 E1 ping\n";
        usb_serial_send((const uint8_t *)ping, strlen(ping));
    }

    /* Wait for "ok" or timeout, processing lines as they arrive. */
    bool wait_silent = false;
    int wait_override_ms = 0;
    bool wait = is_wait_cmd(gcode, &wait_silent, &wait_override_ms);
    /* Timeout selection:
     * - Caller override: exact timeout for host print or special commands.
     * - Silent waits (G28, G29): 120s — no interim feedback at all.
     * - G4 dwell: parsed from S/P parameter — can be arbitrarily long.
     * - Temp waits (M109, M190, M116): 30s — first temp report may be delayed.
     * - Default: 6s. */
    int timeout_ms;
    if (timeout_override_ms > 0)
        timeout_ms = timeout_override_ms;
    else if (wait_override_ms > 0)
        timeout_ms = wait_override_ms;
    else if (wait_silent)
        timeout_ms = WAIT_SILENT_TIMEOUT_MS;
    else if (wait)
        timeout_ms = WAIT_TEMP_TIMEOUT_MS;
    else
        timeout_ms = CMD_RESPONSE_TIMEOUT_MS;
    int64_t start_us = esp_timer_get_time();
    int64_t deadline = start_us + timeout_ms * 1000LL;
    int stall_pings = 0;
    uint8_t rx_byte;

    while (s_pending_query != QUERY_NONE) {
        /* Bail immediately if USB was unplugged while waiting */
        if (!usb_serial_is_connected()) {
            ESP_LOGW(TAG, "USB disconnected while waiting for %s", gcode);
            s_pending_query = QUERY_NONE;
            return false;
        }

        int64_t remaining_us = deadline - esp_timer_get_time();
        if (remaining_us <= 0) {
            /* Send an echo ping to flush a stuck CDC response before giving up.
             * Keeps us in sync with Marlin instead of advancing past a sent command. */
            if (stall_pings < max_stall_pings) {
                stall_pings++;
                ESP_LOGW(TAG, "CDC stall on %s — ping flush attempt %d/%d",
                         gcode, stall_pings, max_stall_pings);
                const char *ping = "M118 E1 ping\n";
                usb_serial_send((const uint8_t *)ping, strlen(ping));
                deadline = esp_timer_get_time() + timeout_ms * 1000LL;
                continue;
            }
            ESP_LOGW(TAG, "CDC TX stall on %s — no response after %d pings, giving up",
                     gcode, stall_pings);
            s_pending_query = QUERY_NONE;
            return false;
        }

        TickType_t ticks = pdMS_TO_TICKS(remaining_us / 1000);
        if (ticks == 0) ticks = 1;

        size_t n = xStreamBufferReceive(s_rx_stream, &rx_byte, 1, ticks);
        if (n == 0) continue;

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                process_line(s_line_buf);
                s_line_len = 0;

                /* "busy: processing" clears s_pending_query in process_line.
                 * For wait/long commands, restore it — we still need the real "ok".
                 * For all commands, extend deadline since printer is alive. */
                if (s_pending_query == QUERY_NONE
                    && esp_timer_get_time() < s_cmd_cooldown_until_us) {
                    if (wait) {
                        s_pending_query = qtype;
                    }
                    int ext_ms = wait_silent ? timeout_ms : CMD_RESPONSE_TIMEOUT_MS;
                    deadline = esp_timer_get_time() + (int64_t)ext_ms * 1000LL;
                }
                /* Wait commands: printer sends temp reports ("T:... W:N") while
                 * waiting — extend deadline on any received line, not just "busy". */
                else if (wait && s_pending_query != QUERY_NONE) {
                    int ext_ms = wait_silent ? timeout_ms : CMD_RESPONSE_TIMEOUT_MS;
                    deadline = esp_timer_get_time() + (int64_t)ext_ms * 1000LL;
                }
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }

    return true;
}

/** Standard send: auto-detect timeout, no stall pings. */
bool marlin_send_cmd(const char *gcode, query_type_t qtype)
{
    return marlin_send_cmd_ex(gcode, qtype, 0, 0);
}

/** Host print send: shorter timeout, with stall ping retries. */
bool marlin_send_cmd_hp(const char *gcode, query_type_t qtype)
{
    return marlin_send_cmd_ex(gcode, qtype, HOST_CMD_TIMEOUT_MS, HOST_STALL_PING_RETRIES);
}

/** Ender Smart (F15-23): send a user/API command with line number and checksum.
 *  Marlin only accepts a checksum together with a line number, so the counter is
 *  reset with an unnumbered "M110 N0" and the command goes as "N1 <cmd>*cs".
 *  A corrupted line (seen on 2026-11-12 as "M11M105" during a babystep burst) is
 *  answered with "Resend: 1" and sent again instead of being executed garbled.
 *  Only used with no host print running (the job owns the line counter then). */
static bool marlin_send_checked(const char *gcode)
{
    char line[MARLIN_MAX_LINE + 1];
    int n = snprintf(line, sizeof(line), "N1 %s", gcode);
    if (n < 0 || n > MARLIN_MAX_LINE - 4) {
        ESP_LOGE(TAG, "Command too long to send with checksum: %.40s", gcode);
        return false;
    }
    uint8_t cs = 0;
    for (const char *p = line; *p; p++) cs ^= (uint8_t)*p;
    snprintf(line + n, sizeof(line) - n, "*%u", cs);

    if (!marlin_send_cmd("M110 N0", QUERY_CMD))
        return false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        s_last_resend = -1;
        bool ok = marlin_send_cmd(line, QUERY_CMD);
        if (s_last_resend < 0)
            return ok;
        ESP_LOGW(TAG, "Printer asked to resend the command (attempt %d/3): %s", attempt, gcode);
        if (s_last_resend != 1 && !marlin_send_cmd("M110 N0", QUERY_CMD))
            return false;
    }
    ESP_LOGE(TAG, "Command not accepted after 3 resends: %s", gcode);
    return false;
}

/* ---- Polling task ---- */

static void printer_comm_task(void *arg)
{
    ESP_LOGI(TAG, "Printer comm task started");

    /* Initialize timing */
    s_last_m105_us = 0;
    s_last_m27_us = 0;
    s_last_m114_us = 0;

    while (true) {
        /* Wait for USB device to connect */
        if (!usb_serial_is_connected()) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            if (!hp_is_active() && g_state.opstate != PRINTER_ERROR) {
                g_state.opstate = PRINTER_DISCONNECTED;
            }
            xSemaphoreGive(g_state_mutex);
            /* Ender Smart (F15-06 / C4): a host print never resumes by itself after
             * the link drops — the Ender may have reset (cold, unhomed). */
            if (hp_is_active()) {
                hp_notify_usb_lost();
            }
            s_m290_queried = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int64_t now = esp_timer_get_time();

        /* Drain command queue first */
        printer_cmd_t cmd;
        while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
            const char *gcode = NULL;

            /* Host print commands */
            if (cmd.type == PCMD_HOST_START) {
                hp_cmd_start(cmd.gcode); continue;
            } else if (cmd.type == PCMD_HOST_PAUSE) {
                hp_cmd_pause(); continue;
            } else if (cmd.type == PCMD_HOST_RESUME) {
                hp_cmd_resume(); continue;
            } else if (cmd.type == PCMD_HOST_CANCEL) {
                hp_cmd_cancel(); continue;
            }

            /* Route pause/resume/cancel to host print if active */
            if (hp_is_active()) {
                if (cmd.type == PCMD_PAUSE) { hp_cmd_pause(); continue; }
                if (cmd.type == PCMD_RESUME) { hp_cmd_resume(); continue; }
                if (cmd.type == PCMD_CANCEL) { hp_cmd_cancel(); continue; }
            }

            switch (cmd.type) {
            case PCMD_PAUSE:
                ESP_LOGI(TAG, "Draining planner (M400) before pause...");
                marlin_send_cmd("M400", QUERY_CMD);
                gcode = (s_pause_cmd == PAUSE_CMD_M524) ? "M524" : "M25";
                break;
            case PCMD_RESUME: gcode = "M24"; break;
            case PCMD_CANCEL: gcode = "M524"; break;
            case PCMD_RAW:    gcode = cmd.gcode; break;
            default: break;
            }
            if (gcode) {
                ESP_LOGI(TAG, "Sending command: %s", gcode);
                bool ok;
                if (cmd.type == PCMD_RAW && !hp_is_active()) {
                    /* F15-23: user/API commands get a line number and checksum, so a
                     * byte lost while the Ender is busy is detected and resent */
                    ok = marlin_send_checked(gcode);
                } else {
                    ok = marlin_send_cmd(gcode, QUERY_CMD);
                }

                /* Notify the synchronous caller waiting for THIS command */
                if (cmd.type == PCMD_RAW && cmd.sync_id != 0 && cmd.sync_id == s_sync_wait_id) {
                    s_sync_ok = ok;
                    s_sync_wait_id = 0;
                    if (s_sync_cmd_sem) xSemaphoreGive(s_sync_cmd_sem);
                }

                if (cmd.type == PCMD_PAUSE) {
                    s_cmd_cooldown_until_us = esp_timer_get_time() + 5000000LL; /* 5s */
                }
            }
        }

        /* ---- Host print: stream GCode lines ---- */
        hp_tick();

        /* Macro: skip remaining polls if cooldown active (set by busy: handler) */
#define CHECK_COOLDOWN() \
        do { if (esp_timer_get_time() < s_cmd_cooldown_until_us) goto poll_done; } while(0)

        CHECK_COOLDOWN();

        /* Skip all polling when suppressed (terminal manual mode) */
        if (s_polling_suppressed) goto poll_done;

        /* During active host printing, only poll M105 at reduced frequency
         * (for Obico temp display). Also poll M119 for filament sensor.
         * Skip M27/M114 entirely — position is tracked from the GCode stream.
         *
         * CRITICAL: skip polling entirely when a retry is pending.
         * marlin_drain_rx() inside send_query would consume the late "ok" for the
         * timed-out command, desynchronising Marlin's line counter and
         * causing an unrecoverable retry loop. */
        if (hp_is_active() && !hp_is_paused()) {
            if (hp_has_pending()) goto poll_done;
            now = esp_timer_get_time();
            if (now - s_last_m105_us >= POLL_M105_HOSTPRINT_INTERVAL_MS * 1000LL) {
                marlin_send_cmd("M105", QUERY_M105);
                s_last_m105_us = esp_timer_get_time();
                CHECK_COOLDOWN();
            }
            /* Poll M119 for filament runout detection (if enabled) */
            if (s_filament_check) {
                now = esp_timer_get_time();
                if (now - s_last_m119_us >= POLL_M119_INTERVAL_MS * 1000LL) {
                    marlin_send_cmd("M119", QUERY_M119);
                    s_last_m119_us = esp_timer_get_time();
                }
            }
            goto poll_done;
        }

        /* Query M290 once on connect to get current probe Z offset */
        if (!s_m290_queried) {
            s_m290_queried = true;
            marlin_send_cmd("M290", QUERY_CMD);
            CHECK_COOLDOWN();
        }

        /* Poll M105 (temperatures) every 4s */
        now = esp_timer_get_time();
        if (now - s_last_m105_us >= POLL_M105_INTERVAL_MS * 1000LL) {
            marlin_send_cmd("M105", QUERY_M105);
            s_last_m105_us = esp_timer_get_time();
            CHECK_COOLDOWN();
        }

        /* Poll M27 (SD progress) every 10s — skip during host print */
        if (!hp_is_active()) {
            now = esp_timer_get_time();
            if (now - s_last_m27_us >= POLL_M27_INTERVAL_MS * 1000LL) {
                marlin_send_cmd("M27", QUERY_M27);
                s_last_m27_us = esp_timer_get_time();
                CHECK_COOLDOWN();
            }

            /* Fetch filename once when printing starts */
            if (s_need_filename) {
                s_need_filename = false;
                marlin_send_cmd("M27 C", QUERY_M27C);
                CHECK_COOLDOWN();
            }
        }

        /* Poll M114 (position) every 30s */
        now = esp_timer_get_time();
        if (now - s_last_m114_us >= POLL_M114_INTERVAL_MS * 1000LL) {
            marlin_send_cmd("M114", QUERY_M114);
            s_last_m114_us = esp_timer_get_time();
        }

poll_done:
#undef CHECK_COOLDOWN

        /* Shorter delay during host print for throughput, normal otherwise */
        vTaskDelay(pdMS_TO_TICKS(hp_is_active() && !hp_is_paused() ? 20 : 100));
    }
}

/* ---- NVS config ---- */

static void load_config_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "No pcomm NVS namespace, using defaults (Marlin)");
        return;
    }

    uint8_t backend_val = 0;
    if (nvs_get_u8(nvs, NVS_KEY_BACKEND, &backend_val) == ESP_OK) {
        s_backend = (backend_val == 1) ? PRINTER_BACKEND_KLIPPER : PRINTER_BACKEND_MARLIN;
    }

    size_t len = sizeof(s_mr_host);
    nvs_get_str(nvs, NVS_KEY_MR_HOST, s_mr_host, &len);

    nvs_get_u16(nvs, NVS_KEY_MR_PORT, &s_mr_port);
    if (s_mr_port == 0) s_mr_port = 7125;

    uint8_t pause_val = 0;
    if (nvs_get_u8(nvs, NVS_KEY_PAUSE_CMD, &pause_val) == ESP_OK) {
        s_pause_cmd = (pause_val == 1) ? PAUSE_CMD_M524 : PAUSE_CMD_M25;
    }

    uint8_t fil_chk = 0;
    if (nvs_get_u8(nvs, NVS_KEY_FILAMENT_CHK, &fil_chk) == ESP_OK) {
        s_filament_check = (fil_chk == 1);
    }

    size_t ping_len = sizeof(s_echo_ping_cmds);
    if (nvs_get_str(nvs, NVS_KEY_ECHO_PING, s_echo_ping_cmds, &ping_len) != ESP_OK) {
        s_echo_ping_cmds[0] = '\0';
    }

    nvs_close(nvs);

    /* Fall back to Marlin if Klipper selected but no host configured */
    if (s_backend == PRINTER_BACKEND_KLIPPER && s_mr_host[0] == '\0') {
        ESP_LOGW(TAG, "Klipper selected but no Moonraker host — falling back to Marlin");
        s_backend = PRINTER_BACKEND_MARLIN;
    }

    ESP_LOGI(TAG, "Config loaded: backend=%s, moonraker=%s:%u, pause=%s, filament_check=%s, echo_ping=[%s]",
             s_backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin",
             s_mr_host, s_mr_port,
             s_pause_cmd == PAUSE_CMD_M524 ? "M524 (abort)" : "M25 (pause)",
             s_filament_check ? "on" : "off",
             s_echo_ping_cmds);
}

/* ---- Public API ---- */

void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return;
    if (len > 0) {
        if (s_rx_stream) {
            xStreamBufferSend(s_rx_stream, data, len, 0);
        }
        terminal_feed_rx(data, len);
    }
}

void printer_comm_get_state(printer_state_t *out)
{
    if (s_simulate) {
        /* Compute elapsed print time (freezes when paused/cancelled) */
        int64_t elapsed_us;
        if (s_sim_opstate == PRINTER_PRINTING) {
            elapsed_us = s_sim_pause_elapsed_us + (esp_timer_get_time() - s_sim_start_us);
        } else {
            elapsed_us = s_sim_pause_elapsed_us;
        }
        int32_t elapsed_s = (int32_t)(elapsed_us / 1000000);
        float progress = (float)(elapsed_s % 600) / 6.0f; /* 0-100% over 10 min */

        memset(out, 0, sizeof(*out));
        out->opstate = s_sim_opstate;
        out->hotend_actual = 205.0f + (elapsed_s % 10) * 0.3f;
        out->hotend_target = (s_sim_opstate == PRINTER_OPERATIONAL) ? 0.0f : 210.0f;
        out->bed_actual = 59.5f + (elapsed_s % 5) * 0.2f;
        out->bed_target = (s_sim_opstate == PRINTER_OPERATIONAL) ? 0.0f : 60.0f;
        out->last_update_us = esp_timer_get_time();

        if (s_sim_opstate == PRINTER_PRINTING || s_sim_opstate == PRINTER_PAUSED) {
            out->progress_pct = progress;
            out->print_time_s = elapsed_s;
            out->print_time_left_s = (int32_t)((100.0f - progress) / 100.0f * 3600);
            out->x = 100.0f + (elapsed_s % 20) * 2.5f;
            out->y = 50.0f + (elapsed_s % 15) * 3.0f;
            out->z = 0.2f * (elapsed_s / 30);
            snprintf(out->filename, sizeof(out->filename), "simulated_benchy.gcode");
        } else {
            out->progress_pct = -1;
            out->print_time_s = -1;
            out->print_time_left_s = -1;
        }
        return;
    }

    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        klipper_backend_get_state(out);
        return;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    memcpy(out, &g_state, sizeof(printer_state_t));
    xSemaphoreGive(g_state_mutex);
}

int printer_comm_get_temp_history(temp_sample_t *buf, int max_count)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    int count = s_temp_count < max_count ? s_temp_count : max_count;
    /* Copy oldest-first: start from (head - count) wrapped */
    int start = (s_temp_head - s_temp_count + TEMP_HISTORY_MAX) % TEMP_HISTORY_MAX;
    /* Skip oldest samples if count < s_temp_count */
    start = (start + (s_temp_count - count)) % TEMP_HISTORY_MAX;
    for (int i = 0; i < count; i++) {
        buf[i] = s_temp_history[(start + i) % TEMP_HISTORY_MAX];
    }
    xSemaphoreGive(g_state_mutex);
    return count;
}

void printer_comm_set_simulate(bool enable)
{
#if CONFIG_ESP32FDM_SIMULATION
    s_simulate = enable;
    if (enable) {
        s_sim_start_us = esp_timer_get_time();
        s_sim_pause_elapsed_us = 0;
        s_sim_opstate = PRINTER_PRINTING;
        ESP_LOGW(TAG, "Simulation mode ENABLED (printing)");
    } else {
        ESP_LOGI(TAG, "Simulation mode disabled");
    }
#else
    (void)enable;
    ESP_LOGE(TAG, "Simulation mode is not available in this build");
#endif
}

bool printer_comm_is_simulating(void)
{
    return s_simulate;
}

bool printer_comm_job_active(void)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    return st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED ||
           (s_backend == PRINTER_BACKEND_MARLIN && hp_is_active());
}

bool printer_comm_cmd_allowed(const char *gcode, char *reason, size_t reason_len)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    bool active = st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED ||
                  (s_backend == PRINTER_BACKEND_MARLIN && hp_is_active());
    if (!active)
        return true;

    while (*gcode == ' ')
        gcode++;
    char letter = (char)toupper((unsigned char)gcode[0]);
    int code = atoi(gcode + 1);

    if (letter == 'M') {
        switch (code) {
        /* status queries and messages */
        case 27: case 31: case 73: case 105: case 114: case 115: case 117: case 118: case 119:
        /* safe fine-tuning while printing */
        case 106: case 107: case 220: case 221: case 290:
            return true;
        case 104:
        case 140: {
            const char *s = strchr(gcode, 'S');
            if (!s)
                s = strchr(gcode, 's');
            float v = s ? strtof(s + 1, NULL) : 0.0f;
            float current_target = (code == 104) ? st.hotend_target : st.bed_target;
            if (v <= 0.0f || current_target > 0.0f)
                return true;   /* cooling, or fine-tuning a heater that is already on */
            snprintf(reason, reason_len,
                     "Ligar um aquecedor desligado não é permitido durante a impressão");
            return false;
        }
        default:
            break;
        }
    }

    snprintf(reason, reason_len, "Comando bloqueado durante a impressão: %.20s", gcode);
    return false;
}

esp_err_t printer_comm_send_cmd(const printer_cmd_t *cmd)
{
    if (s_simulate) {
        switch (cmd->type) {
        case PCMD_PAUSE:
            if (s_sim_opstate == PRINTER_PRINTING) {
                /* Freeze elapsed time */
                s_sim_pause_elapsed_us += esp_timer_get_time() - s_sim_start_us;
                s_sim_opstate = PRINTER_PAUSED;
                ESP_LOGI(TAG, "[SIM] Paused");
            }
            break;
        case PCMD_RESUME:
            if (s_sim_opstate == PRINTER_PAUSED) {
                /* Resume timer from where we left off */
                s_sim_start_us = esp_timer_get_time();
                s_sim_opstate = PRINTER_PRINTING;
                ESP_LOGI(TAG, "[SIM] Resumed");
            }
            break;
        case PCMD_CANCEL:
            if (s_sim_opstate == PRINTER_PRINTING || s_sim_opstate == PRINTER_PAUSED) {
                s_sim_opstate = PRINTER_OPERATIONAL;
                s_sim_pause_elapsed_us = 0;
                ESP_LOGI(TAG, "[SIM] Cancelled");
            }
            break;
        case PCMD_RAW:
            ESP_LOGI(TAG, "[SIM] Raw gcode: %s", cmd->gcode);
            break;
        }
        return ESP_OK;
    }
    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        return klipper_backend_send_cmd(cmd);
    }

    if (xQueueSend(s_cmd_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_init(void)
{
    load_config_from_nvs();

    /* Temp history in PSRAM to save internal SRAM */
    s_temp_history = heap_caps_calloc(TEMP_HISTORY_MAX, sizeof(temp_sample_t),
                                      MALLOC_CAP_SPIRAM);
    if (!s_temp_history) return ESP_ERR_NO_MEM;

    g_state_mutex = xSemaphoreCreateMutex();
    if (!g_state_mutex) return ESP_ERR_NO_MEM;

    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "Starting Klipper/Moonraker backend → %s:%u", s_mr_host, s_mr_port);
        return klipper_backend_start(s_mr_host, s_mr_port);
    }

    /* Marlin backend */

    s_rx_stream = xStreamBufferCreate(2048, 1);
    if (!s_rx_stream) return ESP_ERR_NO_MEM;

    s_cmd_queue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(printer_cmd_t));
    if (!s_cmd_queue) return ESP_ERR_NO_MEM;

    /* Initialize state */
    memset(&g_state, 0, sizeof(g_state));
    g_state.opstate = PRINTER_DISCONNECTED;
    g_state.progress_pct = -1;
    g_state.print_time_s = -1;
    g_state.print_time_left_s = -1;
    g_state.total_layers = -1;

    xTaskCreatePinnedToCore(printer_comm_task, "printer_comm", 6144,
                            NULL, 8, NULL, 1);  /* Core 1: with USB host, isolated from WiFi/HTTP */

    ESP_LOGI(TAG, "Printer comm initialized (Marlin backend)");
    return ESP_OK;
}

/* ---- Host print public API ---- */

esp_err_t printer_comm_host_print_start(const char *filename) { return hp_start(filename); }
esp_err_t printer_comm_host_print_pause(void)  { return hp_pause(); }
esp_err_t printer_comm_host_print_resume(void) { return hp_resume(); }
esp_err_t printer_comm_host_print_cancel(void) { return hp_cancel(); }
bool printer_comm_is_host_printing(void) { return hp_is_active(); }

void printer_comm_set_polling_suppressed(bool suppress)
{
    s_polling_suppressed = suppress;
}

bool printer_comm_is_polling_suppressed(void)
{
    return s_polling_suppressed;
}

/* ---- Line callback ---- */

void printer_comm_set_line_callback(void (*cb)(const char *line))
{
    s_line_callback = cb;
}

/* ---- Synchronous GCode send ---- */

esp_err_t printer_comm_send_gcode_sync(const char *gcode, int timeout_ms)
{
    if (!s_sync_cmd_sem) {
        s_sync_cmd_sem = xSemaphoreCreateBinary();
        if (!s_sync_cmd_sem) return ESP_ERR_NO_MEM;
    }

    printer_cmd_t cmd = { .type = PCMD_RAW };
    strlcpy(cmd.gcode, gcode, sizeof(cmd.gcode));

    if (++s_sync_seq == 0) s_sync_seq = 1;   /* 0 means "nobody waits" */
    cmd.sync_id = s_sync_seq;
    xSemaphoreTake(s_sync_cmd_sem, 0);       /* drop a stale give */
    s_sync_ok = false;
    s_sync_wait_id = cmd.sync_id;

    esp_err_t err = printer_comm_send_cmd(&cmd);
    if (err != ESP_OK) {
        s_sync_wait_id = 0;
        return err;
    }

    if (xSemaphoreTake(s_sync_cmd_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_sync_wait_id = 0;
        return ESP_ERR_TIMEOUT;
    }
    return s_sync_ok ? ESP_OK : ESP_FAIL;
}

/* ---- Backend getters ---- */

printer_backend_t printer_comm_get_backend(void)
{
    return s_backend;
}

const char *printer_comm_backend_name(void)
{
    return s_backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin";
}

const char *printer_comm_get_mr_host(void)
{
    return s_mr_host;
}

uint16_t printer_comm_get_mr_port(void)
{
    return s_mr_port;
}

/* ---- NVS save ---- */

esp_err_t printer_comm_save_config(printer_backend_t backend,
                                   const char *mr_host, uint16_t mr_port,
                                   bool pause_abort)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_set_u8(nvs, NVS_KEY_BACKEND, (uint8_t)backend);
    nvs_set_str(nvs, NVS_KEY_MR_HOST, mr_host ? mr_host : "");
    nvs_set_u16(nvs, NVS_KEY_MR_PORT, mr_port);
    nvs_set_u8(nvs, NVS_KEY_PAUSE_CMD, pause_abort ? 1 : 0);

    err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Config saved: backend=%s, moonraker=%s:%u, pause=%s",
             backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin",
             mr_host ? mr_host : "", mr_port,
             pause_abort ? "M524 (abort)" : "M25 (pause)");
    return err;
}

/* ---- Web UI HTTP handlers ---- */

void printer_config_render_backend(html_buf_t *p)
{
    const char *checked_marlin  = (s_backend == PRINTER_BACKEND_MARLIN)  ? "checked" : "";
    const char *checked_klipper = (s_backend == PRINTER_BACKEND_KLIPPER) ? "checked" : "";

    html_buf_printf(p,
        "<form id='f-printer' method='POST' action='/printer/config'>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='backend' value='marlin' %s> Marlin (USB serial)</label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='backend' value='klipper' %s> Klipper (Moonraker HTTP)</label>"
        "</div>"
        "<div style='margin-left:20px'>"
        "<label style='display:block;margin:8px 0 4px'>Moonraker Host/IP:<input type='text' name='mr_host' value='%s' maxlength='63' style='width:100%%;box-sizing:border-box;padding:6px'></label>"
        "<label style='display:block;margin:8px 0 4px'>Moonraker Port:<input type='number' name='mr_port' value='%u' min='1' max='65535' style='width:100%%;box-sizing:border-box;padding:6px'></label>"
        "</div>"
        "<p class='hint'><a href='/printer/config/test'>Test Moonraker connection</a></p>"
        "</form>",
        checked_marlin, checked_klipper, s_mr_host, s_mr_port);
}

void printer_config_render_marlin(html_buf_t *p)
{
    const char *checked_m25  = (s_pause_cmd == PAUSE_CMD_M25)  ? "checked" : "";
    const char *checked_m524 = (s_pause_cmd == PAUSE_CMD_M524) ? "checked" : "";
    const char *checked_fil  = s_filament_check ? "checked" : "";

    html_buf_printf(p,
        "<form id='f-marlin' method='POST' action='/printer/config/marlin'>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>Pause Command</b></label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='pause_cmd' value='m25' %s> M25 &mdash; Pause SD print (can resume)</label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='pause_cmd' value='m524' %s> M524 &mdash; Abort print (if M25 crashes firmware)</label>"
        "</div>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>Filament Sensor</b></label>"
        "<label style='display:block;margin:8px 0 4px'><input type='checkbox' name='filament_chk' value='1' %s> Poll filament sensor (M119) during host print &mdash; auto-pause on runout</label>"
        "</div>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>CDC Echo Ping</b></label>"
        "<p style='margin:4px 0;font-size:13px;color:#666'>Some printers hold USB responses until the next command is sent. "
        "Adding commands here sends a harmless M118 ping after them to flush the response. "
        "Typical candidates: <b>M119, G92</b></p>"
        "<input type='text' name='echo_ping' value='%s' placeholder='e.g. M119, G92'"
        " style='width:100%%;padding:6px;font-family:monospace'>"
        "</div>"
        "</form>",
        checked_m25, checked_m524, checked_fil, s_echo_ping_cmds);
}

static esp_err_t printer_config_get_handler(httpd_req_t *req)
{
    /* Redirect to unified settings page */
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t printer_config_post_handler(httpd_req_t *req)
{
    if (printer_comm_job_active()) {
        /* Ender Smart (F15-13): saving this config reboots the ESP32 */
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Blocked while a print is running or paused (this would reboot)");
    }

    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    /* Parse form fields */
    printer_backend_t backend = PRINTER_BACKEND_MARLIN;
    char mr_host[64] = "";
    uint16_t mr_port = 7125;

    if (strstr(body, "backend=klipper")) {
        backend = PRINTER_BACKEND_KLIPPER;
    }

    /* Extract mr_host */
    char *hp = strstr(body, "mr_host=");
    if (hp) {
        hp += 8;
        char *end = strchr(hp, '&');
        size_t hlen = end ? (size_t)(end - hp) : strlen(hp);
        if (hlen >= sizeof(mr_host)) hlen = sizeof(mr_host) - 1;
        memcpy(mr_host, hp, hlen);
        mr_host[hlen] = '\0';
    }

    /* Extract mr_port */
    char *pp = strstr(body, "mr_port=");
    if (pp) {
        mr_port = (uint16_t)atoi(pp + 8);
        if (mr_port == 0) mr_port = 7125;
    }

    bool pause_abort = (s_pause_cmd == PAUSE_CMD_M524);  /* preserve current */

    esp_err_t err = printer_comm_save_config(backend, mr_host, mr_port, pause_abort);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS write failed");
        return ESP_FAIL;
    }

    html_buf_t pg;
    html_buf_init(&pg);
    layout_html_begin(&pg, "Saved", "/settings");
    html_buf_printf(&pg,
        "<h2>Configuration saved!</h2>"
        "<p>Rebooting in 3 seconds...</p>"
        "<script>setTimeout(function(){location.href='/settings'},5000)</script>");
    layout_html_end(&pg);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, pg.data, pg.len);
    html_buf_free(&pg);

    /* Reboot after a short delay so the response can be sent */
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();

    return ESP_OK;
}

static esp_err_t printer_config_marlin_post_handler(httpd_req_t *req)
{
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    pause_cmd_t new_cmd = (strstr(body, "pause_cmd=m524") != NULL)
                           ? PAUSE_CMD_M524 : PAUSE_CMD_M25;
    s_pause_cmd = new_cmd;

    bool new_fil_chk = (strstr(body, "filament_chk=1") != NULL);
    s_filament_check = new_fil_chk;

    /* Parse echo_ping field */
    char new_ping[128] = "";
    char *ep = strstr(body, "echo_ping=");
    if (ep) {
        url_decode_field(ep + 10, new_ping, sizeof(new_ping));
        /* Trim whitespace */
        char *start = new_ping;
        while (*start == ' ') start++;
        if (start != new_ping) memmove(new_ping, start, strlen(start) + 1);
        size_t plen = strlen(new_ping);
        while (plen > 0 && new_ping[plen-1] == ' ') new_ping[--plen] = '\0';
    }
    strncpy(s_echo_ping_cmds, new_ping, sizeof(s_echo_ping_cmds));
    s_echo_ping_cmds[sizeof(s_echo_ping_cmds) - 1] = '\0';

    /* Persist to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_PAUSE_CMD, (uint8_t)new_cmd);
        nvs_set_u8(nvs, NVS_KEY_FILAMENT_CHK, new_fil_chk ? 1 : 0);
        nvs_set_str(nvs, NVS_KEY_ECHO_PING, s_echo_ping_cmds);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t printer_config_test_handler(httpd_req_t *req)
{
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%u/printer/info", s_mr_host, s_mr_port);

    /* Quick connectivity test */
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        httpd_resp_set_type(req, "application/json");
        const char *fail = "{\"ok\":false,\"error\":\"client init failed\"}";
        return httpd_resp_send(req, fail, strlen(fail));
    }

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    char resp[128];
    if (err == ESP_OK && status == 200) {
        snprintf(resp, sizeof(resp),
                 "{\"ok\":true,\"host\":\"%s\",\"port\":%u}", s_mr_host, s_mr_port);
    } else {
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"error\":\"HTTP %d, err=%s\"}",
                 status, esp_err_to_name(err));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, strlen(resp));
}

esp_err_t printer_config_register_httpd(void *server_handle)
{
    httpd_handle_t server = (httpd_handle_t)server_handle;

    httpd_uri_t get_cfg = {
        .uri     = "/printer/config",
        .method  = HTTP_GET,
        .handler = printer_config_get_handler,
    };
    HTTPD_REGISTER(server, &get_cfg);

    httpd_uri_t post_cfg = {
        .uri     = "/printer/config",
        .method  = HTTP_POST,
        .handler = printer_config_post_handler,
    };
    HTTPD_REGISTER(server, &post_cfg);

    httpd_uri_t marlin_cfg = {
        .uri     = "/printer/config/marlin",
        .method  = HTTP_POST,
        .handler = printer_config_marlin_post_handler,
    };
    HTTPD_REGISTER(server, &marlin_cfg);

    httpd_uri_t test_cfg = {
        .uri     = "/printer/config/test",
        .method  = HTTP_GET,
        .handler = printer_config_test_handler,
    };
    HTTPD_REGISTER(server, &test_cfg);

    return ESP_OK;
}
