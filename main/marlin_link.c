/*
 * Ender Smart (F2-01, lote 15): Marlin link: RX lines, answer parsing, send and wait for "ok",
 * echo pings and the lote 13 sync marker.
 * Moved out of printer_comm.c without changing the function bodies (only printer_comm_task changed:
 * 4 calls to the queue loops now in command_arbiter.c, and marlin_sync_wait(0)). The log tag stays
 * "printer_comm", so the log lines are the same as before.
 */
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
#include "printer_comm_priv.h"

static const char *TAG = "printer_comm";

/* ---- Link settings ---- */

#define CMD_RESPONSE_TIMEOUT_MS         6000
#define WAIT_SILENT_TIMEOUT_MS          240000  /* G28/G29: can be silent for minutes, needs long timeout */
#define WAIT_TEMP_TIMEOUT_MS            30000   /* M109/M190/M116: first temp report can be delayed */
#define HOST_STALL_PING_RETRIES         5       /* Echo-ping retries before giving up on a stalled cmd */
#define RX_LINE_BUF_SIZE        256

/* ---- Link state ---- */

/* ---- Line callback (used by bedlevel mesh parsing) ---- */
static void (*s_line_callback)(const char *line);

/* ---- Ender Smart (F15-15): quieter log + lost RX bytes ---- */
volatile uint32_t s_rx_dropped;   /* bytes the RX stream buffer had no room for */
static bool    s_tx_quiet;               /* set while sending host-print stream lines */
bool    s_pos_logged;             /* first M114 answer after a (re)connect is logged */

/* ---- Ender Smart (F2-02): M115 identification and capabilities ---- */
#define PRINTER_CAPS_MAX 48
typedef struct { char name[28]; uint8_t on; } printer_cap_t;
static char          s_fw_line[200];
static printer_cap_t s_caps[PRINTER_CAPS_MAX];
static int           s_caps_count;
static int64_t       s_info_time_us;      /* when FIRMWARE_NAME was last seen, 0 = never */

/* F15-23: "Resend: N" seen while sending a user command */
static volatile int32_t   s_last_resend = -1;
static uint32_t           s_spam_lines;     /* Creality debug lines dropped */

/* Ender Smart (lote 6): bumped by every parsed M114 answer, so a pause can tell a fresh
 * position from a stale one (the 1st pause on 2026-11-12 saved X0.4 Y20 from long before). */
volatile uint32_t s_m114_seq;
/* Ender Smart (lote 6): how long a command waits for a busy printer before it is sent anyway */
#define BUSY_WAIT_MAX_US (15 * 1000000LL)
/* Ender Smart (lote 6): longest wait for the "ok" of a normal (non-wait) command, even while
 * "busy: processing" keeps arriving */
#define NONWAIT_MAX_US   (120 * 1000000LL)
/* Ender Smart (lote 6): set by host_print around pause/resume/cancel commands. Only then a
 * command sent while the printer is busy waits for it; every other caller keeps the old
 * behaviour (refused at once, the caller sees false), so a click during an operation started
 * on the Ender's own screen is not executed minutes later. */
volatile bool s_send_patient;
/* Ender Smart (lote 6): bumped by every "Unknown command:" line. An "ok" follows it, so a
 * sequence checks this to know its command was really understood (a byte lost on the USB
 * turned "M110 M105" into "M11M105" on 2026-11-12). */
volatile uint32_t s_unknown_seq;
/* Ender Smart (lote 6): the last "busy:" was "paused for user/input" (M0/M1/M600 waiting for
 * someone at the printer): then the wait for an "ok" is not stretched, as before. */
static bool s_busy_for_user;
/* ... and how many such lines came: a pause/resume/cancel command that sees one was only
 * queued by the Ender, not done, so the sequence treats it as a failure */
volatile uint32_t s_busy_user_seq;
/* Ender Smart (lote 9): every ping (M118 E1 ping) gets its own "echo:ping" and "ok". These
 * count the echoes, the "ok" lines, and the "ok" count when the last echo came, so a command
 * that sent pings can read on until the last ping's "ok" (see settle_pings). */
static volatile uint32_t s_ping_echo_seq;
static volatile uint32_t s_ok_seq;
static volatile uint32_t s_ok_seq_at_echo;
#define PING_SETTLE_US        (300 * 1000LL)  /* M118 is instant: its reply follows at once */
#define PARTIAL_LINE_WAIT_US  (20 * 1000LL)   /* a 96-char line takes ~8.3 ms at 115200 */
char s_echo_ping_cmds[128] = "";  /* Comma-separated GCode prefixes that need CDC flush ping */
/* Ender Smart (lote 13): replies the Ender still owes. A command whose "ok" did not come in
 * time stays in the Ender's queue (a poll sent while it was busy with something started on its
 * own screen, e.g. the leveling), and its late answer would be taken as the answer to a later
 * command: on 2026-11-14 eight late "Not SD printing" came right after a job had started and
 * wiped the job from the screen. Every reply that comes before the echo of a marker
 * (M118 E1 essync <n>) sent after the last command belongs to older commands; the "ok" after
 * the echo is the marker's own. Until then the polls are not sent and a job does not start.
 * Printer task only. */
bool s_need_sync;          /* a command was given up: replies may still come */
bool s_mk_sent;          /* a marker is out, sent after the last command */
bool s_mk_echo;          /* ... and its echo came */
static uint32_t s_mk_id;
static uint32_t s_mk_ok_at_echo;
static int64_t s_mk_sent_us;
static int64_t s_mk_echo_us;      /* when the marker's echo came */
static int64_t s_last_ok_us;      /* when the last "ok" came */
static uint32_t s_restart_seq;    /* bumped when the Ender is seen restarting */
/* A marker is sent again when the Ender answered something after it and then stayed quiet for
 * MK_QUIET_US without the marker's echo and "ok" (lost, e.g. in an RX overflow), and anyway
 * after MK_RESEND_US. Not on a timer while the Ender answers nothing (busy with its screen,
 * waiting on M600): each marker takes room in its small serial buffer (review). */
#define MK_QUIET_US  (2 * 1000000LL)
#define MK_RESEND_US (300 * 1000000LL)

/* Ender Smart (F2-03) */
int64_t s_last_auto_temp_us;     /* last unsolicited temperature line */

StreamBufferHandle_t s_rx_stream;

/* Line buffer for assembling responses */
static char s_line_buf[RX_LINE_BUF_SIZE];
static size_t s_line_len;

/* Tracking which query we're waiting for (query_type_t defined in marlin_proto.h) */
static query_type_t s_pending_query;

static bool s_filament_sensor_present;  /* true once M119 reports a filament: line */

/* Print start time tracking */
static int64_t s_print_start_us;
bool s_need_filename;
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
    /* Ender Smart (lote 13): during a job sent by the ESP32 the Ender is not printing from
     * its own card, so an M27 answer is a late one. On 2026-11-14 eight of them ("Not SD
     * printing") came right after a job had started and wiped it from the screen. */
    if (hp_is_active()) return;

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
    if (hp_is_active()) return;   /* lote 13: a late answer; the job's name stays */

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
    s_m114_seq++;

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

    /* lote 9: a ping's echo; strstr because a lost byte gave "4echo:ping" (T-13) */
    if (strstr(line, "echo:ping")) {
        s_ping_echo_seq++;
        /* an "ok" glued in front ("okecho:ping", a lost newline) is counted below and is
         * not the ping's: the ping's own "ok" is still to come */
        s_ok_seq_at_echo = s_ok_seq + (strncmp(line, "ok", 2) == 0 ? 1u : 0u);
    }

    /* lote 13: the marker's echo ("echo:essync 7"). An Ender without M118 would answer
     * 'echo:Unknown command: "M118 E1 essync 7"', which also comes after every older reply.
     * A glued "ok" in front is the previous command's, as for the ping above. */
    if (s_mk_sent && !s_mk_echo) {
        const char *mk = strstr(line, "essync ");
        if (mk && strtoul(mk + 7, NULL, 10) == s_mk_id) {
            s_mk_echo = true;
            s_mk_echo_us = esp_timer_get_time();
            s_mk_ok_at_echo = s_ok_seq + (strncmp(line, "ok", 2) == 0 ? 1u : 0u);
        }
    }

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

    /* F15-15: answers to the automatic polls, temperature reports, "ok" of the host-print
     * stream and "busy:" go to DEBUG; the first position after a reconnect stays visible
     * (it shows whether the Ender kept its position, i.e. did not restart). */
    bool is_ok = strncmp(line, "ok", 2) == 0 && (line[2] == '\0' || line[2] == ' ');
    bool is_pos = strstr(line, "X:") && strstr(line, "Y:") && strstr(line, "Z:") && strstr(line, "Count");
    bool quiet = strncmp(line, "busy:", 5) == 0 ||
                 (strstr(line, "T:") && strstr(line, "B:")) ||
                 strstr(line, "Not SD printing") || strstr(line, "SD printing byte") ||
                 (is_ok && (s_tx_quiet || hp_is_active() || s_pending_query == QUERY_M105 ||
                            s_pending_query == QUERY_M27 || s_pending_query == QUERY_M114 ||
                            s_pending_query == QUERY_M119)) ||
                 (is_pos && s_pos_logged);
    if (is_pos) s_pos_logged = true;
    if (quiet)
        ESP_LOGD(TAG, "RX: %s", line);
    else
        ESP_LOGI(TAG, "RX: %s", line);

    /* F2-02: M115 answer. A new FIRMWARE_NAME line starts a fresh capability list. */
    if (strncmp(line, "FIRMWARE_NAME:", 14) == 0) {
        strlcpy(s_fw_line, line, sizeof(s_fw_line));
        s_caps_count = 0;
        s_info_time_us = esp_timer_get_time();
    } else if (strncmp(line, "Cap:", 4) == 0 && s_caps_count < PRINTER_CAPS_MAX) {
        const char *nm = line + 4, *colon = strrchr(nm, ':');
        if (colon && colon > nm) {
            size_t n = (size_t)(colon - nm);
            if (n >= sizeof(s_caps[0].name)) n = sizeof(s_caps[0].name) - 1;
            memcpy(s_caps[s_caps_count].name, nm, n);
            s_caps[s_caps_count].name[n] = '\0';
            for (char *c = s_caps[s_caps_count].name; *c; c++)   /* keep it JSON-safe */
                if (!isalnum((unsigned char)*c) && *c != '_') *c = '_';
            s_caps[s_caps_count].on = (uint8_t)(atoi(colon + 1) != 0);
            s_caps_count++;
        }
    }

    /* Ender Smart (F15-06): Marlin boot banner means the printer restarted (power
     * glitch, reset). Seen on the Ender-3 Max Neo: "start" (often garbled),
     * "DWIN handshake error.", "BL24CXX Check succeeded!". Any host print must stop. */
    if (strcmp(line, "start") == 0 || strstr(line, "DWIN handshake") ||
        strstr(line, "BL24CXX Check")) {
        ESP_LOGW(TAG, "Printer restart detected (\"%s\")", line);
        hp_notify_printer_reset();
        s_autotemp_sent = false;   /* F2-03: M155 is not kept across a restart */
        /* lote 13: a restarted Ender owes nothing (its queue is gone) */
        s_restart_seq++;
        s_need_sync = false;
        s_mk_sent = false;
        s_mk_echo = false;
    }

    /* Content-based matching — handles unsolicited auto-reports too */
    if (strstr(line, "T:")) {
        if (s_pending_query != QUERY_M105) s_last_auto_temp_us = esp_timer_get_time();  /* F2-03 */
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
        s_unknown_seq++;   /* lote 6 */
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
    /* lote 12: the Ender's answer to M112 (only kill() prints this) */
    if (s_halted && !s_kill_seen && (strstr(line, "Printer halted") || strstr(line, "kill() called"))) {
        s_kill_seen = true;
        ESP_LOGE(TAG, "Emergency stop confirmed by the Ender: %s", line);
        hp_estop_confirmed();
    }

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
                printer_cmd_t cmd = { .type = PCMD_HOST_SAFETY_CANCEL };   /* lote 9 */
                strncpy(cmd.gcode, line, sizeof(cmd.gcode) - 1);   /* the reason, for the screen */
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
        s_busy_for_user = strstr(line, "paused for") != NULL;   /* lote 6 */
        if (s_busy_for_user) s_busy_user_seq++;
        s_cmd_cooldown_until_us = esp_timer_get_time() + 4000000LL; /* back off 4s */
        s_pending_query = QUERY_NONE;
        return;
    }

    /* G29 bed leveling in progress — Marlin sends "measured_z:" for each probe
     * point instead of "ok".  Back off longer than normal busy to span the
     * gap between probes (~6-8 s).  The final "ok" after the mesh grid
     * clears the cooldown and resumes polling. */
    if (strstr(line, "measured_z:")) {
        s_busy_for_user = false;   /* lote 6: a probe point, not a person at the printer */
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
        s_ok_seq++;                /* lote 9: see settle_pings */
        s_last_ok_us = esp_timer_get_time();   /* lote 13: see mk_send_if_needed */
        /* lote 13: the marker's own "ok": every older command is answered */
        if (s_mk_echo && s_ok_seq != s_mk_ok_at_echo) {
            s_need_sync = false;
            s_mk_sent = false;
            s_mk_echo = false;
            ESP_LOGI(TAG, "The Ender answered every older command (marker %lu)",
                     (unsigned long)s_mk_id);
        }
        if (strstr(line, "T:")) {
            parse_m105(line);
        }
        s_pending_query = QUERY_NONE;
        /* Clear cooldown so the wait-loop doesn't mistake this real "ok"
         * for a "busy: processing" clear and re-arm the query. */
        s_cmd_cooldown_until_us = 0;
        s_busy_for_user = false;   /* lote 6 */
    }
}

/** Add one received byte to the line being assembled; process it at the end of line. */
static void rx_feed_byte(uint8_t rx_byte)
{
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

/** Drain buffered RX data, processing any complete lines (e.g. auto-reports) */
void marlin_drain_rx(void)
{
    uint8_t rx_byte;
    while (xStreamBufferReceive(s_rx_stream, &rx_byte, 1, 0) > 0)
        rx_feed_byte(rx_byte);
}

/** Ender Smart (lote 9): read on until the echo of ping number target_echo and an "ok"
 *  after it, for PING_SETTLE_US at most.
 *  When a command's "ok" was only late (not lost), the ping sent meanwhile adds a second
 *  "ok". Before, the 10 ms after the next line's TX usually let that line's own "ok" arrive
 *  and the drain before the line after dropped it. Without the delay the extra "ok" would
 *  answer the next line, and every later line, one line early (found in review). */
static void settle_pings(uint32_t target_echo)
{
    int64_t give_up = esp_timer_get_time() + PING_SETTLE_US;
    uint8_t b;
    while (esp_timer_get_time() < give_up) {
        if ((int32_t)(s_ping_echo_seq - target_echo) >= 0 && s_ok_seq != s_ok_seq_at_echo)
            return;
        if (!usb_serial_is_connected() || s_halted)
            return;
        if (xStreamBufferReceive(s_rx_stream, &b, 1, pdMS_TO_TICKS(10)) > 0)
            rx_feed_byte(b);
    }
    ESP_LOGW(TAG, "Ping replies incomplete after %lld ms (echo %lu of %lu)",
             PING_SETTLE_US / 1000, (unsigned long)s_ping_echo_seq, (unsigned long)target_echo);
    s_need_sync = true;            /* lote 13: the missing replies may still come */
}

/** Ender Smart (lote 13): sends the marker when none is out after the last command, when the
 *  one out was lost (see MK_QUIET_US), or after MK_RESEND_US. False if the USB write failed. */
static bool mk_send_if_needed(void)
{
    if (!s_need_sync) return true;    /* nothing owed: no marker (2nd review) */
    int64_t now = esp_timer_get_time();
    /* lost: the Ender answered something after the marker and then stayed quiet, or the
     * echo came and its "ok" did not ("echo:essync 7ok" with the newline lost) */
    bool lost = s_mk_sent &&
                ((s_last_ok_us > s_mk_sent_us && now - s_last_ok_us > MK_QUIET_US) ||
                 (s_mk_echo && now - s_mk_echo_us > MK_QUIET_US));
    if (s_mk_sent && !lost && now - s_mk_sent_us <= MK_RESEND_US)
        return true;
    /* the leading newline ends a line the Ender may have got only in part (a lost byte), so
     * the marker is not glued to it, nor the next command to the marker (review) */
    char mk[40];
    uint32_t id = s_mk_id + 1;
    int n = snprintf(mk, sizeof(mk), "\nM118 E1 essync %lu\n", (unsigned long)id);
    if (usb_serial_send((const uint8_t *)mk, (size_t)n) != ESP_OK)
        return false;
    s_mk_id = id;
    s_mk_sent = true;
    s_mk_echo = false;
    s_mk_sent_us = now;
    mk[n - 1] = '\0';
    terminal_feed_tx(mk + 1);
    ESP_LOGI(TAG, "The Ender may still owe replies: marker sent (%s)%s", mk + 1,
             lost ? "; the previous one was lost" : "");
    return true;
}

/** Ender Smart (lote 13): while replies are owed, sends the marker if needed and reads until
 *  it is answered or wait_us passes (0: only what is already here). Returns true when nothing
 *  is owed. Printer task only, with no command waiting. */
static bool sync_wait(int64_t wait_us)
{
    if (!s_need_sync) return true;
    int64_t until = esp_timer_get_time() + wait_us;
    s_pending_query = QUERY_NONE;
    uint8_t b;
    for (;;) {
        /* first: the byte read at the end of the last round may have ended the sync, and then
         * no new marker may go out (2nd review) */
        if (!s_need_sync) return true;
        if (!usb_serial_is_connected() || s_halted || s_estop) return false;
        if (!mk_send_if_needed()) return false;
        marlin_drain_rx();
        if (!s_need_sync) return true;
        if (esp_timer_get_time() >= until) return false;
        if (xStreamBufferReceive(s_rx_stream, &b, 1, pdMS_TO_TICKS(10)) > 0)
            rx_feed_byte(b);
    }
}

uint32_t marlin_restart_seq(void)
{
    return s_restart_seq;
}

bool marlin_replies_owed(void)
{
    return s_need_sync;
}

bool marlin_sync_wait(int64_t wait_us)
{
    return sync_wait(wait_us);
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
        if (code == 400) {
            /* Ender Smart (lote 6): M400 answers only when every queued move is done.
             * As a normal command it timed out or ended on "busy:", and its late "ok"
             * was taken for the next command's answer (the pause read a stale position). */
            if (timeout_override_ms) *timeout_override_ms = 60000;
            return true;
        }
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
    if (s_halted) return false;        /* lote 12: after M112 nothing more goes out */

    /* Process any buffered unsolicited data before sending */
    s_pending_query = QUERY_NONE;
    marlin_drain_rx();
    /* Ender Smart (lote 9): a line the Ender started right after the last "ok" (the M155
     * temperature report comes from its idle loop, i.e. just after an "ok") may still be
     * arriving. The 10 ms after each TX used to give it time; without them it would be cut
     * by the reset below and the report lost (found in review). Let it finish first. */
    if (s_line_len > 0) {
        int64_t until = esp_timer_get_time() + PARTIAL_LINE_WAIT_US;
        uint8_t pb;
        while (s_line_len > 0 && esp_timer_get_time() < until) {
            if (!usb_serial_is_connected()) break;
            if (xStreamBufferReceive(s_rx_stream, &pb, 1, pdMS_TO_TICKS(2)) > 0)
                rx_feed_byte(pb);
        }
    }

    /* Abort if drain_rx saw busy: and set cooldown.
     * Ender Smart (lote 6): this silently lost commands for callers that ignore the result.
     * On 2026-11-12 a pause lost the retract, lift and park after one "busy:", leaving the
     * nozzle on the part. Pause/resume/cancel (s_send_patient) now wait for the printer
     * instead: its "ok" clears the cooldown; after BUSY_WAIT_MAX_US the command is sent
     * anyway and Marlin queues it. Other callers keep the immediate refusal. */
    if (esp_timer_get_time() < s_cmd_cooldown_until_us) {
        if (!s_send_patient) return false;
        int64_t give_up = esp_timer_get_time() + BUSY_WAIT_MAX_US;
        uint8_t b;
        while (esp_timer_get_time() < s_cmd_cooldown_until_us && esp_timer_get_time() < give_up) {
            if (!usb_serial_is_connected()) return false;
            if (xStreamBufferReceive(s_rx_stream, &b, 1, pdMS_TO_TICKS(50)) > 0)
                rx_feed_byte(b);
        }
        s_pending_query = QUERY_NONE;
    }

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
    const uint32_t echo_base = s_ping_echo_seq;   /* lote 9 */
    int pings = 0;

    /* Send command with newline */
    char buf[MARLIN_MAX_LINE + 2];
    int len = snprintf(buf, sizeof(buf), "%s\n", gcode);
    /* F15-15: automatic polls and the host-print stream go to DEBUG; commands stay visible */
    if (s_tx_quiet || qtype == QUERY_M105 || qtype == QUERY_M27 || qtype == QUERY_M114 || qtype == QUERY_M119)
        ESP_LOGD(TAG, "TX: %s", gcode);
    else
        ESP_LOGI(TAG, "TX: %s", gcode);
    terminal_feed_tx(gcode);
    esp_err_t err = usb_serial_send((const uint8_t *)buf, len);
    if (err != ESP_OK) {
        s_pending_query = QUERY_NONE;
        return false;
    }
    /* lote 13: a marker out before this command would be answered before it: it no longer
     * tells when everything is answered, so the next sync_wait() sends a new one (always,
     * not only while replies are owed: 2nd review) */
    s_mk_sent = false;
    s_mk_echo = false;

    /* Brief pause after TX — gives STM32 CDC time to process the USB
     * OUT packet before we start polling for the response.
     * Ender Smart (lote 9 / F2-05): only for a native CDC-ACM device. A USB-UART bridge
     * (the Ender's CH340) buffers the reply itself; on 2026-11-13 these 10 ms were 60% of
     * each line's time (F2-04: 58-60 lines/s in all). The wait loop below blocks until
     * the reply arrives, so nothing is lost without it. */
    if (!usb_serial_is_uart_bridge())
        vTaskDelay(pdMS_TO_TICKS(10));

    /* Echo ping: some STM32 CDC implementations hold TX data until the
     * next OUT transaction.  Send a harmless M118 to flush the response.
     * The "echo:ping" reply is ignored by process_line. */
    /* lote 9: not for a USB-UART bridge, which never holds the reply back */
    if (needs_echo_ping(gcode) && !usb_serial_is_uart_bridge()) {
        const char *ping = "M118 E1 ping\n";
        usb_serial_send((const uint8_t *)ping, strlen(ping));
        pings++;
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
        /* lote 12: an emergency stop does not wait for this "ok" (G28/G29/M109 can take
         * minutes), and nothing of the sequence this command belongs to goes out after it */
        if (s_halted) {
            ESP_LOGW(TAG, "Emergency stop while waiting for %s: not waiting", gcode);
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
                pings++;
                deadline = esp_timer_get_time() + timeout_ms * 1000LL;
                continue;
            }
            ESP_LOGW(TAG, "CDC TX stall on %s — no response after %d pings, giving up",
                     gcode, stall_pings);
            s_pending_query = QUERY_NONE;
            /* lote 13: the command was sent; its answer (and the pings') may still come */
            s_need_sync = true;
            s_mk_sent = false;
            s_mk_echo = false;
            return false;
        }

        TickType_t ticks = pdMS_TO_TICKS(remaining_us / 1000);
        if (ticks == 0) ticks = 1;
        if (ticks > pdMS_TO_TICKS(100)) ticks = pdMS_TO_TICKS(100);   /* lote 12: see s_halted soon */

        size_t n = xStreamBufferReceive(s_rx_stream, &rx_byte, 1, ticks);
        if (n == 0) continue;

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                process_line(s_line_buf);
                s_line_len = 0;

                /* "busy: processing" clears s_pending_query in process_line.
                 * Ender Smart (lote 6): restore it for normal commands too, not only the
                 * wait commands. "busy: processing" means this command is still running
                 * and its "ok" is still to come; the old code stopped waiting, so that late
                 * "ok" was taken as the answer to the next command (the pause read a stale
                 * position). Not for "paused for user" (M0/M600 waits for a person, maybe
                 * for ever: the old behaviour, and pause/resume/cancel treat it as a
                 * failure), and a normal command waits NONWAIT_MAX_US at most. The deadline
                 * is extended, never shortened, since the printer is alive. */
                if (s_pending_query == QUERY_NONE
                    && esp_timer_get_time() < s_cmd_cooldown_until_us
                    && !s_busy_for_user) {
                    s_pending_query = qtype;
                    int ext_ms = wait_silent ? timeout_ms : CMD_RESPONSE_TIMEOUT_MS;
                    int64_t nd = esp_timer_get_time() + (int64_t)ext_ms * 1000LL;
                    if (!wait && nd > start_us + NONWAIT_MAX_US) nd = start_us + NONWAIT_MAX_US;
                    if (nd > deadline) deadline = nd;
                }
                /* Wait commands: printer sends temp reports ("T:... W:N") while
                 * waiting — extend deadline on any received line, not just "busy". */
                else if (wait && s_pending_query != QUERY_NONE) {
                    int ext_ms = wait_silent ? timeout_ms : CMD_RESPONSE_TIMEOUT_MS;
                    int64_t nd = esp_timer_get_time() + (int64_t)ext_ms * 1000LL;
                    if (nd > deadline) deadline = nd;
                }
                /* lote 13: "paused for user" ends this wait (the old behaviour, above), but
                 * this command's "ok" is still to come, after the person at the printer */
                if (s_pending_query == QUERY_NONE && s_busy_for_user) {
                    s_need_sync = true;
                    s_mk_sent = false;
                    s_mk_echo = false;
                }
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }

    /* lote 9: only without the 10 ms delay; a native CDC device that holds replies would
     * just cost 300 ms here, and keeps the old behaviour */
    if (pings > 0 && usb_serial_is_uart_bridge())
        settle_pings(echo_base + (uint32_t)pings);
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
    s_tx_quiet = true;
    bool ok = marlin_send_cmd_ex(gcode, qtype, HOST_CMD_TIMEOUT_MS, HOST_STALL_PING_RETRIES);
    s_tx_quiet = false;
    return ok;
}

/** Ender Smart (lote 9): standard timeout, but logged at DEBUG like the other stream
 *  lines (a job's G4 lines filled the log at INFO in the F2-04 test). */
bool marlin_send_cmd_quiet(const char *gcode, query_type_t qtype)
{
    s_tx_quiet = true;
    bool ok = marlin_send_cmd_ex(gcode, qtype, 0, 0);
    s_tx_quiet = false;
    return ok;
}

/** Ender Smart (F15-23): send a user/API command with line number and checksum.
 *  Marlin only accepts a checksum together with a line number, so the counter is
 *  reset with an unnumbered "M110 N0" and the command goes as "N1 <cmd>*cs".
 *  A corrupted line (seen on 2026-11-12 as "M11M105" during a babystep burst) is
 *  answered with "Resend: 1" and sent again instead of being executed garbled.
 *  Only used with no host print running (the job owns the line counter then). */
bool marlin_send_checked(const char *gcode)
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

void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return;
    if (len > 0) {
        if (s_rx_stream) {
            size_t n = xStreamBufferSend(s_rx_stream, data, len, 0);
            if (n < len) s_rx_dropped += (uint32_t)(len - n);   /* F15-15 */
        }
        terminal_feed_rx(data, len);
    }
}

uint32_t printer_comm_rx_dropped(void)
{
    return s_rx_dropped;
}

void printer_comm_info_json(char *out, size_t out_size)
{
    size_t o = 0;
    o += snprintf(out + o, out_size - o, "{\"firmware_line\":\"");
    for (const unsigned char *p = (const unsigned char *)s_fw_line; *p && o + 8 < out_size; p++) {
        if (*p == '"' || *p == '\\') { out[o++] = '\\'; out[o++] = (char)*p; }
        else if (*p >= 0x20 && *p < 0x7f) out[o++] = (char)*p;
    }
    o += snprintf(out + o, out_size - o, "\",\"age_s\":%lld,\"caps\":{",
                  s_info_time_us ? (long long)((esp_timer_get_time() - s_info_time_us) / 1000000) : -1LL);
    for (int i = 0; i < s_caps_count && o + 40 < out_size; i++)
        o += snprintf(out + o, out_size - o, "%s\"%s\":%d", i ? "," : "", s_caps[i].name, s_caps[i].on);
    snprintf(out + o, out_size - o, "}}");
}

void printer_comm_set_line_callback(void (*cb)(const char *line))
{
    s_line_callback = cb;
}
