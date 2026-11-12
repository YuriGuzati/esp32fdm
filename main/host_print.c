#include "marlin_proto.h"
#include "gcode_scan.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "host_print";

/* ---- Constants ---- */

#define HOST_RESEND_MAX_RETRIES 5
#define HOST_LINE_HISTORY_SIZE  5

/* Park position and retract settings */
#define PARK_X       0.0f
#define PARK_Y       0.0f
#define PARK_Z_LIFT  5.0f    /* mm to lift Z on pause */
#define PARK_RETRACT 2.0f    /* mm filament retract on pause */

/* ---- Host print state ---- */

typedef struct {
    /* State */
    bool printing, paused;

    /* File I/O */
    FILE *file;
    char filename[64];
    int32_t lines_sent, total_lines;

    /* Pending line (deferred on busy) */
    char pending_line[256];
    bool has_pending;
    bool line_truncated;

    /* Layer tracking */
    int32_t layer;
    bool has_layer_comments;

    /* Timing */
    int64_t start_us, pause_elapsed_us;

    /* Marlin line numbering & checksum */
    int32_t marlin_line;        /* Next Marlin line number to assign */
    int32_t resend_line;        /* Line requested by Resend:, or -1 */
    char numbered_line[160];    /* Formatted "Nxxx gcode*cs" for resend */
    int resend_retries;

    /* Rolling history of recently sent numbered lines */
    char line_history[HOST_LINE_HISTORY_SIZE][160];
    int32_t line_history_num[HOST_LINE_HISTORY_SIZE];
    int line_history_idx;

    /* Park position */
    float park_x, park_y, park_z;

    /* Ender Smart (F15-09): modes as set by the file, restored after a pause */
    bool abs_xyz;              /* G90 (true) / G91 */
    bool abs_e;                /* M82 (true) / M83 */
    bool homed;                /* a G28 was sent in this job: XY park is safe */
    float last_f;              /* last feedrate from G0/G1 F..., 0 = unknown */
    float park_e;              /* logical E at pause (for G92 E in absolute mode) */
    float saved_hotend, saved_bed;  /* targets at pause, reheated on resume */

    /* Ender Smart (F15-06/07): reasons to stop streaming */
    volatile bool printer_reset;   /* Marlin boot banner seen */
    volatile bool usb_lost;        /* USB dropped during the job */
    bool desync;                   /* printer asked for a line we no longer have */
    bool line_too_long;            /* a line does not fit Marlin's buffer */
} hp_ctx_t;

static hp_ctx_t s_hp;

/* ---- Helpers ---- */

/** Show why the job stopped (opstate ERROR + message), until the user clears it. */
static void hp_set_error(const char *msg)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_ERROR;
    strncpy(g_state.error_msg, msg, sizeof(g_state.error_msg) - 1);
    g_state.error_msg[sizeof(g_state.error_msg) - 1] = '\0';
    xSemaphoreGive(g_state_mutex);
    ESP_LOGE(TAG, "%s", msg);
}

static void hp_clear_error(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (g_state.opstate == PRINTER_ERROR) {
        g_state.opstate = usb_serial_is_connected() ? PRINTER_OPERATIONAL : PRINTER_DISCONNECTED;
        ESP_LOGI(TAG, "Error acknowledged: %s", g_state.error_msg);
    }
    g_state.error_msg[0] = '\0';
    xSemaphoreGive(g_state_mutex);
}

/** Track the modes and feedrate the file sets, so a pause can restore them (F15-09). */
static void hp_track_modes(const char *line)
{
    char c0 = (char)toupper((unsigned char)line[0]);
    int code = atoi(line + 1);
    if (c0 == 'G' && (code == 90 || code == 91) && !isdigit((unsigned char)line[3])) {
        s_hp.abs_xyz = (code == 90);
        /* In this Marlin, G90/G91 also reset the extruder mode to follow XYZ */
        s_hp.abs_e = (code == 90);
    } else if (c0 == 'M' && (code == 82 || code == 83) && !isdigit((unsigned char)line[3])) {
        s_hp.abs_e = (code == 82);
    } else if (c0 == 'G' && code == 28) {
        s_hp.homed = true;
    } else if (c0 == 'G' && (code == 0 || code == 1)) {
        const char *f = strchr(line, 'F');
        if (!f) f = strchr(line, 'f');
        if (f && (f[-1] == ' ')) {
            float v = strtof(f + 1, NULL);
            if (v > 0.0f) s_hp.last_f = v;
        }
    }
}

/** Compute Marlin XOR checksum: XOR all bytes in the string */
static uint8_t marlin_checksum(const char *line)
{
    uint8_t cs = 0;
    while (*line) cs ^= (uint8_t)*line++;
    return cs;
}

/** Format a numbered line: "Nxxx gcode*cs" into dst */
static int format_numbered_line(char *dst, size_t size, int32_t line_num, const char *gcode)
{
    int n = snprintf(dst, size, "N%ld %s", (long)line_num, gcode);
    if (n < 0 || (size_t)n >= size - 5) return -1; /* no room for *cs */
    uint8_t cs = marlin_checksum(dst);
    n += snprintf(dst + n, size - n, "*%u", cs);
    return n;
}

/** Look up a previously sent numbered line from the rolling history. */
static const char *history_lookup(int32_t line_num)
{
    for (int i = 0; i < HOST_LINE_HISTORY_SIZE; i++) {
        if (s_hp.line_history_num[i] == line_num && s_hp.line_history[i][0])
            return s_hp.line_history[i];
    }
    return NULL;
}

/** Store a numbered line in the rolling history. */
static void history_store(int32_t line_num, const char *formatted)
{
    int idx = s_hp.line_history_idx;
    strncpy(s_hp.line_history[idx], formatted, sizeof(s_hp.line_history[idx]));
    s_hp.line_history[idx][sizeof(s_hp.line_history[idx]) - 1] = '\0';
    s_hp.line_history_num[idx] = line_num;
    s_hp.line_history_idx = (idx + 1) % HOST_LINE_HISTORY_SIZE;
}

/** Check if a raw GCode command is long-running (G28, G29, M109, M190, M116, G4).
 *  These need standard (long) timeouts, not the short host-print timeout. */
static bool is_long_running_cmd(const char *gcode)
{
    const char *p = gcode;
    if (*p == 'G' || *p == 'g') {
        int code = atoi(p + 1);
        return (code == 4 || code == 28 || code == 29);
    }
    if (*p == 'M' || *p == 'm') {
        int code = atoi(p + 1);
        return (code == 109 || code == 190 || code == 116);
    }
    return false;
}

/** Send a numbered GCode command with checksum, handle Resend: retries.
 *  Long-running commands (G28, G29, M109, etc.) use standard timeout;
 *  normal GCode lines use the short host-print timeout with stall pings. */
static bool send_query_numbered(const char *gcode)
{
    /* Pick sender: long-running commands need standard timeout (auto-detected),
     * normal lines use short HP timeout with CDC stall ping retries. */
    bool (*sender)(const char *, query_type_t) =
        is_long_running_cmd(gcode) ? marlin_send_cmd : marlin_send_cmd_hp;

    s_hp.resend_line = -1;
    s_hp.resend_retries = 0;

    int nlen = format_numbered_line(s_hp.numbered_line, sizeof(s_hp.numbered_line),
                                    s_hp.marlin_line, gcode);
    if (nlen < 0 || nlen > MARLIN_MAX_LINE) {
        /* Ender Smart (F15-07): never truncate or send it unnumbered — stop the job */
        ESP_LOGE(TAG, "Line too long for Marlin (%d > %d): %.40s...",
                 nlen < 0 ? (int)strlen(gcode) : nlen, MARLIN_MAX_LINE, gcode);
        s_hp.line_too_long = true;
        return false;
    }

    history_store(s_hp.marlin_line, s_hp.numbered_line);

    if (!sender(s_hp.numbered_line, QUERY_CMD)) {
        return false;
    }

    /* Handle resend requests */
    while (s_hp.resend_line >= 0) {
        if (s_hp.resend_line == s_hp.marlin_line + 1) {
            /* Printer already executed our line and wants the next one.
             * This means our command succeeded -- just advance the counter. */
            ESP_LOGW(TAG, "Printer already has N%ld, wants N%ld -- command accepted",
                     (long)s_hp.marlin_line, (long)s_hp.resend_line);
            s_hp.resend_line = -1;
            s_hp.marlin_line++;
            return true;
        }
        if (s_hp.resend_line != s_hp.marlin_line) {
            /* Marlin wants a different line than the one we just sent.
             * Look it up in history and replay from there. */
            const char *old = history_lookup(s_hp.resend_line);
            if (old) {
                int32_t replay_from = s_hp.resend_line;
                ESP_LOGW(TAG, "Replaying from N%ld to N%ld (history hit)",
                         (long)replay_from, (long)s_hp.marlin_line);
                s_hp.resend_line = -1;
                /* Resend all lines from the requested one up to current */
                for (int32_t n = replay_from; ; n++) {
                    const char *line_data;
                    if (n == s_hp.marlin_line) {
                        line_data = s_hp.numbered_line;
                    } else {
                        line_data = history_lookup(n);
                        if (!line_data) break; /* shouldn't happen */
                    }
                    if (!sender(line_data, QUERY_CMD)) {
                        return false;
                    }
                    if (n == s_hp.marlin_line) break;
                }
                continue; /* check for further resend requests */
            }
            /* Ender Smart (F15-06 / C4): a line outside the history usually means the
             * printer restarted (it asks for N1 again). The old code resynced with
             * M110 and kept streaming the middle of the file into a cold, unhomed
             * machine. Stop instead. */
            ESP_LOGE(TAG, "Resend N%ld not in history (current N%ld) -- stopping job",
                     (long)s_hp.resend_line, (long)s_hp.marlin_line);
            s_hp.resend_line = -1;
            s_hp.desync = true;
            return false;
        }

        s_hp.resend_retries++;
        if (s_hp.resend_retries > HOST_RESEND_MAX_RETRIES) {
            ESP_LOGE(TAG, "Resend retries exhausted for N%ld", (long)s_hp.marlin_line);
            s_hp.resend_line = -1;
            return false;
        }

        ESP_LOGW(TAG, "Resending N%ld (attempt %d)", (long)s_hp.marlin_line, s_hp.resend_retries);
        s_hp.resend_line = -1;
        if (!sender(s_hp.numbered_line, QUERY_CMD)) {
            return false;
        }
    }

    s_hp.marlin_line++;
    return true;
}

/* ---- Host print lifecycle ---- */

/** Save position, E and temperatures, then park the head (F15-09). */
static void host_print_park(void)
{
    char buf[64];
    marlin_send_cmd("M400", QUERY_CMD);           /* Wait for moves to finish first */
    marlin_send_cmd("M114", QUERY_M114);          /* ... so the position is the real one */

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    s_hp.park_x = g_state.x;
    s_hp.park_y = g_state.y;
    s_hp.park_z = g_state.z;
    s_hp.park_e = g_state.e;
    s_hp.saved_hotend = g_state.hotend_target;
    s_hp.saved_bed = g_state.bed_target;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG, "Parking from X%.2f Y%.2f Z%.2f E%.3f (%s, E %s, F%.0f, %.0f/%.0f C)",
             s_hp.park_x, s_hp.park_y, s_hp.park_z, s_hp.park_e,
             s_hp.abs_xyz ? "G90" : "G91", s_hp.abs_e ? "M82" : "M83", s_hp.last_f,
             s_hp.saved_hotend, s_hp.saved_bed);

    marlin_send_cmd("M83", QUERY_CMD);            /* Relative E just for the retract */
    snprintf(buf, sizeof(buf), "G1 E-%.1f F2400", PARK_RETRACT);
    marlin_send_cmd(buf, QUERY_CMD);
    marlin_send_cmd("G91", QUERY_CMD);
    snprintf(buf, sizeof(buf), "G1 Z%.1f F600", PARK_Z_LIFT);
    marlin_send_cmd(buf, QUERY_CMD);              /* Lift Z */
    marlin_send_cmd("G90", QUERY_CMD);
    if (s_hp.homed) {
        /* Absolute park target only after homing (position known) */
        snprintf(buf, sizeof(buf), "G1 X%.1f Y%.1f F6000", PARK_X, PARK_Y);
        marlin_send_cmd(buf, QUERY_CMD);          /* Park at corner */
    }
}

/** Reheat, restore position, prime, then restore the file's modes, E and F (F15-09).
 *  The old code forced M83 at the end, which broke files with absolute E (Cura default). */
static void host_print_unpark(void)
{
    char buf[64];

    /* Reheat and WAIT before touching the part or priming */
    if (s_hp.saved_bed > 0.0f) {
        snprintf(buf, sizeof(buf), "M140 S%.0f", s_hp.saved_bed);
        marlin_send_cmd(buf, QUERY_CMD);
    }
    if (s_hp.saved_hotend > 0.0f) {
        snprintf(buf, sizeof(buf), "M109 S%.0f", s_hp.saved_hotend);
        ESP_LOGI(TAG, "Reheating nozzle to %.0f C before resuming", s_hp.saved_hotend);
        marlin_send_cmd(buf, QUERY_CMD);
    }

    ESP_LOGI(TAG, "Unparking to X%.2f Y%.2f Z%.2f", s_hp.park_x, s_hp.park_y, s_hp.park_z);
    marlin_send_cmd("G90", QUERY_CMD);
    snprintf(buf, sizeof(buf), "G1 X%.2f Y%.2f F6000", s_hp.park_x, s_hp.park_y);
    marlin_send_cmd(buf, QUERY_CMD);              /* XY first, still lifted */
    snprintf(buf, sizeof(buf), "G1 Z%.2f F600", s_hp.park_z);
    marlin_send_cmd(buf, QUERY_CMD);              /* then lower Z */

    marlin_send_cmd("M83", QUERY_CMD);
    snprintf(buf, sizeof(buf), "G1 E%.1f F2400", PARK_RETRACT);
    marlin_send_cmd(buf, QUERY_CMD);              /* undo the retract */

    /* Restore the file's modes. G90/G91 first: in this Marlin it also resets E mode. */
    marlin_send_cmd(s_hp.abs_xyz ? "G90" : "G91", QUERY_CMD);
    if (s_hp.abs_e) {
        marlin_send_cmd("M82", QUERY_CMD);
        snprintf(buf, sizeof(buf), "G92 E%.5f", s_hp.park_e);
        marlin_send_cmd(buf, QUERY_CMD);          /* logical E back to what the file expects */
    } else {
        marlin_send_cmd("M83", QUERY_CMD);
    }
    if (s_hp.last_f > 0.0f) {
        snprintf(buf, sizeof(buf), "G1 F%.0f", s_hp.last_f);
        marlin_send_cmd(buf, QUERY_CMD);          /* feedrate for the next move */
    }
}

/** Ender Smart (F15-06/08): stop a job whose printer state is unknown or unsafe.
 *  Unlike a cancel it never moves the head — after a printer reset the position is
 *  unknown — it only turns the heaters off (a safe action) and reports why. */
static void host_print_interrupt(const char *why)
{
    if (s_hp.file) {
        fclose(s_hp.file);
        s_hp.file = NULL;
    }
    bool connected = usb_serial_is_connected();
    if (connected) {
        marlin_send_cmd("M104 S0", QUERY_CMD);
        marlin_send_cmd("M140 S0", QUERY_CMD);
        marlin_send_cmd("M77", QUERY_CMD);
    }
    s_hp.printing = false;
    s_hp.paused = false;
    s_hp.resend_line = -1;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.progress_pct = -1;
    g_state.print_time_left_s = -1;
    g_state.host_printing = false;
    xSemaphoreGive(g_state_mutex);

    char msg[96];
    snprintf(msg, sizeof(msg), "Impressão interrompida: %s.%s", why,
             connected ? " Aquecedores desligados." : "");
    hp_set_error(msg);
}

/** Finish host print (complete or cancel) */
static void host_print_finish(bool cancelled)
{
    if (s_hp.file) {
        fclose(s_hp.file);
        s_hp.file = NULL;
    }

    bool connected = usb_serial_is_connected();

    if (connected) {
        /* Reset Marlin's line counter so it stops expecting numbered lines.
         * Must be sent as a numbered command with the expected line number. */
        char reset[40];
        format_numbered_line(reset, sizeof(reset), s_hp.marlin_line, "M110 N0");
        marlin_send_cmd(reset, QUERY_CMD);

        /* Stop Marlin print timer */
        marlin_send_cmd("M77", QUERY_CMD);
        marlin_send_cmd("M117", QUERY_CMD);  /* Clear LCD message */

        /* Disable host keepalive -- restore default (disabled) to reduce chatter */
        marlin_send_cmd("M113 S0", QUERY_CMD);
    }
    s_hp.resend_line = -1;

    if (cancelled) {
        if (connected) {
            ESP_LOGI(TAG, "Host print cancelled -- parking and turning off heaters");
            marlin_send_cmd("M400", QUERY_CMD);       /* Wait for planner to drain */
            marlin_send_cmd("G91", QUERY_CMD);         /* Relative positioning */
            marlin_send_cmd("G1 Z10 F600", QUERY_CMD); /* Raise nozzle 10mm */
            marlin_send_cmd("G90", QUERY_CMD);         /* Absolute positioning */
            /* Ender Smart: an absolute XY move is only safe after homing. Cancelling
             * during the initial heat-up (before G28) used to drive the head to
             * X0 Y200 from an unknown position. */
            if (s_hp.homed)
                marlin_send_cmd("G1 X0 Y200 F3000", QUERY_CMD); /* Park head at rear-left */
            marlin_send_cmd("M104 S0", QUERY_CMD);     /* Hotend off */
            marlin_send_cmd("M140 S0", QUERY_CMD);     /* Bed off */
            marlin_send_cmd("M107", QUERY_CMD);         /* Fan off */
            marlin_send_cmd("M84", QUERY_CMD);          /* Disable steppers */
        } else {
            ESP_LOGW(TAG, "Host print cancelled -- USB disconnected, skipping park/cooldown");
        }
    } else {
        int64_t elapsed_us = s_hp.pause_elapsed_us + (esp_timer_get_time() - s_hp.start_us);
        ESP_LOGI(TAG, "Host print complete: %s (%ld lines, %lds)",
                 s_hp.filename, (long)s_hp.lines_sent,
                 (long)(elapsed_us / 1000000));
        if (connected) {
            marlin_send_cmd("M117 Print complete", QUERY_CMD);
        }
    }

    s_hp.printing = false;
    s_hp.paused = false;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = connected ? PRINTER_OPERATIONAL : PRINTER_DISCONNECTED;
    g_state.progress_pct = -1;
    g_state.print_time_s = -1;
    g_state.print_time_left_s = -1;
    g_state.host_printing = false;
    g_state.current_layer = 0;
    g_state.filename[0] = '\0';
    xSemaphoreGive(g_state_mutex);
}

/** Start host print from within the task context */
void hp_cmd_start(const char *filename)
{
    if (s_hp.printing) {
        ESP_LOGW(TAG, "Host print already active");
        return;
    }

    char path[128];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return;
    }

    gcode_scan_t scan;
    gcode_scan_file(f, &scan);

    ESP_LOGI(TAG, "Host print starting: %s (%ld lines)", filename, (long)scan.total_lines);

    s_hp.file = f;
    s_hp.printing = true;
    s_hp.paused = false;
    s_hp.lines_sent = 0;
    s_hp.total_lines = scan.total_lines;
    s_hp.layer = 0;
    s_hp.has_layer_comments = false;
    s_hp.start_us = esp_timer_get_time();
    s_hp.pause_elapsed_us = 0;
    s_hp.has_pending = false;
    s_hp.line_truncated = false;
    /* Ender Smart: Marlin defaults until the file says otherwise */
    s_hp.abs_xyz = true;
    s_hp.abs_e = true;
    s_hp.homed = false;
    s_hp.last_f = 0.0f;
    s_hp.printer_reset = false;
    s_hp.usb_lost = false;
    s_hp.desync = false;
    s_hp.line_too_long = false;
    hp_clear_error();
    strncpy(s_hp.filename, filename, sizeof(s_hp.filename) - 1);
    s_hp.filename[sizeof(s_hp.filename) - 1] = '\0';

    /* Reset Marlin line numbering for reliable transmission */
    s_hp.marlin_line = 0;
    s_hp.resend_line = -1;
    memset(s_hp.line_history, 0, sizeof(s_hp.line_history));
    s_hp.line_history_idx = 0;
    char sync[40];
    format_numbered_line(sync, sizeof(sync), 0, "M110 N0");
    marlin_send_cmd(sync, QUERY_CMD);
    s_hp.marlin_line = 1;

    /* Enable host keepalive -- Marlin sends "busy: processing" every 2s
     * during long-running commands (G28, G29, etc.) so we know it's alive. */
    marlin_send_cmd("M113 S2", QUERY_CMD);

    /* Tell Marlin we're printing -- starts LCD print timer & enables host mode */
    marlin_send_cmd("M75", QUERY_CMD);

    /* Show filename on LCD */
    char lcd_msg[128];
    snprintf(lcd_msg, sizeof(lcd_msg), "M117 %s", filename);
    marlin_send_cmd(lcd_msg, QUERY_CMD);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PRINTING;
    g_state.progress_pct = 0;
    g_state.print_time_s = 0;
    g_state.print_time_left_s = -1;
    g_state.host_printing = true;
    g_state.current_layer = 0;
    g_state.object_height = scan.max_z;
    g_state.total_layers = scan.total_layers;
    g_state.layer_height = scan.layer_height;
    g_state.first_layer_height = scan.first_layer_height;
    strncpy(g_state.filename, filename, sizeof(g_state.filename) - 1);
    g_state.filename[sizeof(g_state.filename) - 1] = '\0';
    xSemaphoreGive(g_state_mutex);
}

/** Pause host print: park head, stop timer */
void hp_cmd_pause(void)
{
    if (!s_hp.printing || s_hp.paused) return;
    if (s_hp.usb_lost || s_hp.printer_reset) return;   /* hp_tick interrupts the job */
    s_hp.paused = true;
    s_hp.pause_elapsed_us += esp_timer_get_time() - s_hp.start_us;
    host_print_park();
    marlin_send_cmd("M76", QUERY_CMD);  /* Pause Marlin print timer */
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PAUSED;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Host print paused (head parked)");
}

/** Resume host print: restore position, restart timer */
void hp_cmd_resume(void)
{
    if (!s_hp.printing || !s_hp.paused) return;
    if (s_hp.usb_lost || s_hp.printer_reset) return;   /* never resume after a drop */
    marlin_send_cmd("M75", QUERY_CMD);  /* Resume Marlin print timer */
    host_print_unpark();
    s_hp.paused = false;
    s_hp.start_us = esp_timer_get_time();
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PRINTING;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Host print resumed (head restored)");
}

/** Cancel host print. With no job running it acknowledges a shown error. */
void hp_cmd_cancel(void)
{
    if (!s_hp.printing) {
        hp_clear_error();
        return;
    }
    if (s_hp.usb_lost || s_hp.printer_reset) {
        /* A cancel queued while the link was down must not park: position unknown */
        host_print_interrupt(s_hp.printer_reset ? "a Ender reiniciou" : "a conexão USB caiu");
        return;
    }
    host_print_finish(true);
}

/* ---- Host print streaming loop ---- */

void hp_tick(void)
{
    if (!s_hp.printing) return;

    /* Ender Smart (F15-06 / C4): never resume by itself after the printer restarted
     * or the USB link dropped. Wait for the link, turn the heaters off, report. */
    if (s_hp.printer_reset || s_hp.usb_lost) {
        if (!marlin_is_connected()) return;
        host_print_interrupt(s_hp.printer_reset ? "a Ender reiniciou"
                                                : "a conexão USB caiu");
        return;
    }

    if (s_hp.paused) return;
    if (!marlin_is_connected()) return;

    int lines_this_batch = 0;
    char line[256];

    while (lines_this_batch < 4 && s_hp.file) {
        if (esp_timer_get_time() < s_cmd_cooldown_until_us) break;

        /* Use pending line from previous failed send, or read new */
        if (s_hp.has_pending) {
            strncpy(line, s_hp.pending_line, sizeof(line));
            s_hp.has_pending = false;
        } else {
            if (!fgets(line, sizeof(line), s_hp.file)) {
                if (ferror(s_hp.file)) {
                    /* Ender Smart (F15-08 / C8): a read error is NOT the end of the
                     * file — the end G-code never ran. Park, heaters off, report. */
                    host_print_finish(true);
                    hp_set_error("Impressão cancelada: erro de leitura do cartão SD. "
                                 "Bico afastado e aquecedores desligados.");
                    break;
                }
                /* EOF -- print complete */
                host_print_finish(false);
                break;
            }

            /* Strip newline and detect truncated (split) long lines.
             * fgets fills the buffer without a newline when the line
             * is longer than sizeof(line)-1.  The continuations of
             * such lines must be skipped until we see a newline. */
            size_t ln = strlen(line);
            bool has_newline = (ln > 0 && (line[ln-1] == '\n' || line[ln-1] == '\r'));
            while (ln > 0 && (line[ln-1] == '\n' || line[ln-1] == '\r'))
                line[--ln] = '\0';

            if (s_hp.line_truncated) {
                /* Still consuming a split long line -- skip until newline */
                s_hp.line_truncated = !has_newline;
                s_hp.lines_sent++;
                continue;
            }
            if (!has_newline && ln > 0) {
                /* First chunk of a long line -- skip it and mark truncated */
                s_hp.line_truncated = true;
                /* Still count and check if it was a comment */
                if (line[0] != ';') {
                    ESP_LOGW(TAG, "Skipping long line (%d chars): %.40s...", (int)ln, line);
                }
                s_hp.lines_sent++;
                continue;
            }

            /* Parse layer comments for tracking */
            if (strncmp(line, ";LAYER:", 7) == 0) {
                s_hp.layer = atoi(line + 7);
                s_hp.has_layer_comments = true;
            } else if (strncmp(line, ";LAYER_CHANGE", 13) == 0) {
                /* First LAYER_CHANGE = layer 0 (0-based, like ;LAYER:) */
                if (s_hp.has_layer_comments)
                    s_hp.layer++;
                s_hp.has_layer_comments = true;
            }

            /* Skip empty lines and comments */
            if (ln == 0 || line[0] == ';') {
                s_hp.lines_sent++;
                continue;
            }

            /* Strip inline comments */
            char *comment = strchr(line, ';');
            if (comment) {
                *comment = '\0';
                /* Trim trailing spaces */
                ln = strlen(line);
                while (ln > 0 && line[ln-1] == ' ') line[--ln] = '\0';
            }
            if (ln == 0) {
                s_hp.lines_sent++;
                continue;
            }

            /* Skip lines that aren't valid GCode commands.
             * Valid GCode starts with G, M, T, N, O, or F.
             * Slicer annotations like "27 @ 4.6mm" would otherwise
             * be sent raw, causing response desync. */
            char first = line[0];
            if (first != 'G' && first != 'g' &&
                first != 'M' && first != 'm' &&
                first != 'T' && first != 't' &&
                first != 'N' && first != 'n' &&
                first != 'O' && first != 'o' &&
                first != 'F' && first != 'f') {
                ESP_LOGW(TAG, "Skipping non-GCode line: %s", line);
                s_hp.lines_sent++;
                continue;
            }
        }

        /* Check if this command was previously reported as unsupported */
        if ((line[0] == 'M' || line[0] == 'G') && s_unsupported_count > 0) {
            uint16_t code = (uint16_t)atoi(line + 1);
            bool skip = false;
            for (int i = 0; i < s_unsupported_count; i++) {
                if (s_unsupported_cmds[i] == code) { skip = true; break; }
            }
            if (skip) {
                /* Still parse M73 locally for our own progress tracking */
                if (line[0] == 'M' && code == 73) {
                    const char *pp = strstr(line, "P");
                    const char *rp = strstr(line, "R");
                    int pct = pp ? atoi(pp + 1) : -1;
                    int mins = rp ? atoi(rp + 1) : -1;
                    if (pct >= 0 && pct <= 100) {
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_state.progress_pct = (float)pct;
                        if (mins >= 0)
                            g_state.print_time_left_s = mins * 60;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
                s_hp.lines_sent++;
                continue;
            }
        }

        /* Track Z from G0/G1 moves for real-time position */
        if ((line[0] == 'G') && (line[1] == '0' || line[1] == '1') && line[2] == ' ') {
            const char *zp = strstr(line, "Z");
            if (zp && (zp == line + 3 || *(zp - 1) == ' ')) {
                float z = strtof(zp + 1, NULL);
                if (z > 0.01f) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.z = z;
                    xSemaphoreGive(g_state_mutex);
                }
            }
        }

        /* Send to printer with line numbering & checksum. */
        if (!send_query_numbered(line)) {
            if (s_hp.line_too_long) {
                /* F15-07: the printer is fine, so a normal cancel (park + cool) is safe */
                host_print_finish(true);
                hp_set_error("Impressão cancelada: linha de G-code longa demais para a "
                             "Ender (mais de 95 caracteres).");
                break;
            }
            if (s_hp.desync || s_hp.printer_reset) {
                host_print_interrupt(s_hp.printer_reset ? "a Ender reiniciou"
                                                        : "a impressora pediu uma linha antiga");
                break;
            }
            if (esp_timer_get_time() < s_cmd_cooldown_until_us) {
                /* Printer said "busy: processing" -- defer */
                strncpy(s_hp.pending_line, line, sizeof(s_hp.pending_line));
                s_hp.has_pending = true;
                break;
            }
            /* send_query already retried with echo pings. No answer means the
             * printer state is unknown (it may have reset without a banner), so do
             * not move the head: heaters off and report (F15-06). */
            ESP_LOGE(TAG, "Command failed after retries: %s", line);
            host_print_interrupt("a impressora parou de responder");
            break;
        }
        hp_track_modes(line);
        s_hp.lines_sent++;
        lines_this_batch++;

        /* Update progress */
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (s_hp.total_lines > 0) {
            g_state.progress_pct = (float)s_hp.lines_sent / (float)s_hp.total_lines * 100.0f;
        }
        int64_t elapsed_us = s_hp.pause_elapsed_us + (esp_timer_get_time() - s_hp.start_us);
        g_state.print_time_s = (int32_t)(elapsed_us / 1000000);
        g_state.current_layer = s_hp.layer + 1;  /* ;LAYER: is 0-based, display 1-based */
        /* If no ;LAYER: comments in file, estimate from Z */
        if (!s_hp.has_layer_comments && g_state.z > 0 && g_state.layer_height > 0) {
            float flh = g_state.first_layer_height > 0 ? g_state.first_layer_height : g_state.layer_height;
            if (g_state.z <= flh)
                g_state.current_layer = 1;
            else
                g_state.current_layer = 1 + (int32_t)((g_state.z - flh) / g_state.layer_height + 0.5f);
        }
        if (g_state.progress_pct > 0.1f) {
            float total_est = (float)g_state.print_time_s / (g_state.progress_pct / 100.0f);
            g_state.print_time_left_s = (int32_t)(total_est - g_state.print_time_s);
            if (g_state.print_time_left_s < 0) g_state.print_time_left_s = 0;
        }
        xSemaphoreGive(g_state_mutex);
    }
}

/* ---- Query functions ---- */

void hp_init(void)
{
    /* Struct is zero-initialized -- nothing to do */
}

bool hp_is_active(void)
{
    return s_hp.printing;
}

bool hp_has_pending(void)
{
    return s_hp.has_pending;
}

bool hp_is_paused(void)
{
    return s_hp.paused;
}

/* ---- Notifications from RX parsing ---- */

void hp_notify_resend(int32_t line_num)
{
    s_hp.resend_line = line_num;
}

void hp_notify_printer_reset(void)
{
    if (s_hp.printing && !s_hp.printer_reset) {
        s_hp.printer_reset = true;
        ESP_LOGE(TAG, "Printer restarted during host print -- job will be interrupted");
    }
}

void hp_notify_usb_lost(void)
{
    if (s_hp.printing && !s_hp.usb_lost) {
        s_hp.usb_lost = true;
        hp_set_error("Impressão interrompida: a conexão USB caiu. "
                     "Reconecte para desligar os aquecedores.");
    }
}

/* ---- Fill state (no-op for now -- progress updated inline in hp_tick) ---- */

void hp_fill_state(printer_state_t *out)
{
    (void)out;
}

/* ---- Public API pass-throughs ---- */

esp_err_t hp_start(const char *filename)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_START };
    strncpy(cmd.gcode, filename, sizeof(cmd.gcode) - 1);
    cmd.gcode[sizeof(cmd.gcode) - 1] = '\0';
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_pause(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_PAUSE };
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_resume(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_RESUME };
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_cancel(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_CANCEL };
    return printer_comm_send_cmd(&cmd);
}
