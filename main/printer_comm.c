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

/* Ender Smart (F2-01, lote 15): printer_comm.c was split without changing the function bodies (only
 * printer_comm_task changed: 4 calls to the queue loops, and marlin_sync_wait(0)).
 * - marlin_link.c: the Marlin link (RX lines, answer parsing, send and wait for "ok", echo
 *   pings, the lote 13 sync marker);
 * - command_arbiter.c: the command queue, what may run when, the emergency stop;
 * - here: the printer task (queue, host print, polling), configuration, state and web handlers;
 * - printer_comm_priv.h: the state the three share. */

/* ---- Configuration ---- */

#define POLL_M105_INTERVAL_MS           4000
#define POLL_M105_HOSTPRINT_INTERVAL_MS 30000   /* Reduced frequency during host print */
#define POLL_M27_INTERVAL_MS            10000
#define POLL_M114_INTERVAL_MS           30000
#define POLL_M119_INTERVAL_MS           30000   /* Filament sensor check during host print */
#define CMD_QUEUE_SIZE          8

static int64_t s_last_heartbeat_us;      /* F15-15: one "Status:" line per minute */

#define NVS_NAMESPACE           "pcomm"
#define NVS_KEY_BACKEND         "backend"
#define NVS_KEY_MR_HOST         "mr_host"
#define NVS_KEY_MR_PORT         "mr_port"
#define NVS_KEY_PAUSE_CMD       "pause_cmd"
#define NVS_KEY_FILAMENT_CHK    "fil_chk"
#define NVS_KEY_ECHO_PING       "echo_ping"
#define NVS_KEY_AUTOTEMP        "autotemp"
#define NVS_KEY_PAUSE_RETR      "pause_retr"   /* lote 6: tenths of mm */
#define NVS_KEY_PAUSE_TEMP      "pause_temp"   /* lote 6: nozzle C while paused, 0 = keep */
#define NVS_KEY_PAUSE_WIPE      "pause_wipe"   /* lote 7: purge line on resume, 1 = on */

/* ---- Backend config ---- */

printer_backend_t s_backend = PRINTER_BACKEND_MARLIN;
static char s_mr_host[64] = "";
static uint16_t s_mr_port = 7125;
pause_cmd_t s_pause_cmd = PAUSE_CMD_M25;
static bool s_filament_check = false;  /* Poll M119 for filament runout during host print */
/* Ender Smart (F2-03): temperature autoreport (M155 S2) instead of polling M105.
 * Off by default; when the user turns it on, M155 S2 is sent on every connection. */
static bool s_autotemp = false;
bool s_autotemp_sent;            /* M155 S2 sent on this connection */
static volatile bool s_autotemp_dirty;  /* setting changed: send M155 S2 / S0 */
static int64_t s_last_m155_us;
#define AUTOTEMP_FRESH_US (6 * 1000000LL)  /* reports newer than this replace M105 */
/* Ender Smart (lote 6): pause settings. The Ender-3 Max Neo is a Bowden machine and its
 * Cura profile retracts 6.5 mm; the old fixed 2 mm let the nozzle ooze for the whole pause
 * (seen on 2026-11-12). A lower nozzle temperature while paused stops the ooze; resume
 * waits (M109) for the print temperature again. */
static uint16_t s_pause_retract_x10 = 65;  /* 0..100 = 0.0..10.0 mm */
static uint16_t s_pause_temp_c = 170;      /* 0 = keep the print temperature */
/* Ender Smart (lote 7): purge line + wipe at the left edge before resuming (host_print.c) */
static bool s_pause_wipe = true;

/* ---- Temperature history circular buffer ---- */

static temp_sample_t *s_temp_history;  /* allocated in PSRAM */
static int s_temp_head;   /* next write index */
static int s_temp_count;  /* number of valid samples */

/* ---- State (Marlin backend) ---- */

printer_state_t g_state;
SemaphoreHandle_t g_state_mutex;

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

/* Polling suppression (terminal manual mode) */
static bool s_polling_suppressed;

/* Timestamps for polling intervals */
static int64_t s_last_m105_us;
static int64_t s_last_m27_us;
static int64_t s_last_m114_us;
static int64_t s_last_m119_us;

/* One-shot M290 query on connect */
static bool s_m290_queried;

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

/* ---- Polling task ---- */

static void printer_comm_task(void *arg)
{
    ESP_LOGI(TAG, "Printer comm task started");

    /* Initialize timing */
    s_last_m105_us = 0;
    s_last_m27_us = 0;
    s_last_m114_us = 0;

    while (true) {
        /* lote 12 (F3-07): emergency stop requested by the HTTP task. Nothing queued runs; the
         * job ends without commands (after M112 the Ender is halted and would not answer). */
        if (s_estop) {
            int mode = s_estop_mode;
            arbiter_drop_queue_estop();
            s_estop = false;                     /* before: the test mode sends heaters off */
            hp_cmd_estop(mode);
            continue;
        }
        if (s_halted) {
            if (usb_serial_connect_gen() != s_halt_gen) {
                /* the link was opened again: the Ender was power-cycled or replugged */
                s_halted = false;
                s_m290_queried = false;
                s_pos_logged = false;
                s_autotemp_sent = false;
                s_need_sync = false;    /* lote 13 */
                s_mk_sent = false;
                s_mk_echo = false;
                ESP_LOGW(TAG, "Emergency stop released: the USB link was opened again");
            } else {
                arbiter_drop_queue_halted();
                if (usb_serial_is_connected())
                    marlin_drain_rx();           /* reads the Ender's "kill() called" */
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

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
            /* Ender Smart (lote 11): nothing queued while the Ender is away may run when it comes
             * back (it may have reset, and nobody is asking any more). A preparation is aborted,
             * and its start is dropped with an error on screen. */
            hp_prepare_abort("a Ender desconectou durante a preparação");
            arbiter_drop_queue_disconnected();
            s_m290_queried = false;
            s_pos_logged = false;
            s_autotemp_sent = false;
            s_need_sync = false;    /* lote 13: a new connection starts with nothing owed */
            s_mk_sent = false;
            s_mk_echo = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int64_t now = esp_timer_get_time();

        /* Drain command queue first */
        arbiter_dispatch_queue();

        /* ---- Host print: stream GCode lines ---- */
        bool hp_more = hp_tick();   /* lote 9: true = stopped at the round limit, more ready */

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
            bool auto_fresh = s_autotemp && now - s_last_auto_temp_us < AUTOTEMP_FRESH_US;
            if (!auto_fresh && now - s_last_m105_us >= POLL_M105_HOSTPRINT_INTERVAL_MS * 1000LL) {
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

        /* Ender Smart (lote 13): while the Ender still owes replies (a poll given up while it
         * was busy with its own screen), only the marker goes out. Before, a new M27/M114 went
         * every 10/30 s and piled up in its queue; their late answers came after a job start. */
        if (s_need_sync) {
            marlin_sync_wait(0);
            if (s_need_sync) goto poll_done;
        }

        /* Query M290 once on connect to get current probe Z offset */
        if (!s_m290_queried) {
            s_m290_queried = true;
            marlin_send_cmd("M290", QUERY_CMD);
            CHECK_COOLDOWN();
        }

        /* Ender Smart (F2-03): apply the autoreport setting (on connect, on change, and again
         * if the reports stopped, e.g. after an Ender restart that was not seen) */
        now = esp_timer_get_time();
        if (s_autotemp_dirty ||
            (s_autotemp && !s_autotemp_sent) ||
            (s_autotemp && now - s_last_auto_temp_us > 15000000LL && now - s_last_m155_us > 60000000LL)) {
            bool on = s_autotemp;
            s_autotemp_dirty = false;
            s_last_m155_us = now;
            if (on || s_autotemp_sent) {
                ESP_LOGI(TAG, "Temperature autoreport %s", on ? "on (M155 S2)" : "off (M155 S0)");
                marlin_send_cmd(on ? "M155 S2" : "M155 S0", QUERY_CMD);
            }
            s_autotemp_sent = on;
            CHECK_COOLDOWN();
        }

        /* Poll M105 (temperatures) every 4s — not needed while autoreport lines arrive.
         * Ender Smart (lote 6): read what is buffered first. When idle nothing else drains
         * the RX stream for up to 10 s, so the reports looked stale and an M105 went out
         * every ~10 s even with M155 on (seen on 2026-11-12). */
        marlin_drain_rx();
        now = esp_timer_get_time();
        bool auto_fresh = s_autotemp && now - s_last_auto_temp_us < AUTOTEMP_FRESH_US;
        if (!auto_fresh && now - s_last_m105_us >= POLL_M105_INTERVAL_MS * 1000LL) {
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

        /* F15-15: one status line per minute replaces the per-poll log lines */
        if (esp_timer_get_time() - s_last_heartbeat_us >= 60000000LL) {
            s_last_heartbeat_us = esp_timer_get_time();
            printer_state_t hb;
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            memcpy(&hb, &g_state, sizeof(hb));
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "Status: HE %.1f/%.0f BED %.1f/%.0f Z %.2f%s%s, rx_dropped %lu, stack_free %u",
                     hb.hotend_actual, hb.hotend_target, hb.bed_actual, hb.bed_target, hb.z,
                     hp_is_active() ? ", printing " : "", hp_is_active() ? hb.filename : "",
                     (unsigned long)s_rx_dropped,
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));  /* lote 6: bytes never used */
        }

        /* Shorter delay during host print for throughput, normal otherwise.
         * Ender Smart (lote 9 / F2-05): while the job has lines ready, only yield one tick:
         * the 20 ms after every round of 4 lines were 30% of each line's time (F2-04). */
        if (hp_more)
            vTaskDelay(1);
        else
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

    uint8_t autotemp = 0;
    if (nvs_get_u8(nvs, NVS_KEY_AUTOTEMP, &autotemp) == ESP_OK) {
        s_autotemp = (autotemp == 1);
    }

    uint16_t u16 = 0;
    if (nvs_get_u16(nvs, NVS_KEY_PAUSE_RETR, &u16) == ESP_OK && u16 <= 100)
        s_pause_retract_x10 = u16;
    if (nvs_get_u16(nvs, NVS_KEY_PAUSE_TEMP, &u16) == ESP_OK && (u16 == 0 || (u16 >= 100 && u16 <= 260)))
        s_pause_temp_c = u16;
    uint8_t wipe = 1;
    if (nvs_get_u8(nvs, NVS_KEY_PAUSE_WIPE, &wipe) == ESP_OK)
        s_pause_wipe = (wipe == 1);

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

/* Ender Smart (Fase 4): printer part of GET /api/settings */
void printer_comm_settings_json(char *out, size_t out_size)
{
    char echo[160];
    size_t o = 0;
    for (const char *p = s_echo_ping_cmds; *p && o + 2 < sizeof(echo); p++) {
        if (*p == '"' || *p == '\\') echo[o++] = '\\';
        echo[o++] = *p;
    }
    echo[o] = '\0';
    char host[80];
    o = 0;
    for (const char *p = s_mr_host; *p && o + 2 < sizeof(host); p++) {
        if (*p == '"' || *p == '\\') host[o++] = '\\';
        host[o++] = *p;
    }
    host[o] = '\0';
    snprintf(out, out_size,
             "\"backend\":\"%s\",\"mr_host\":\"%s\",\"mr_port\":%u,"
             "\"pause_cmd\":\"%s\",\"filament_chk\":%s,\"echo_ping\":\"%s\",\"autotemp\":%s,"
             "\"pause_retract\":%.1f,\"pause_temp\":%u,\"pause_wipe\":%s",
             s_backend == PRINTER_BACKEND_KLIPPER ? "klipper" : "marlin",
             host, (unsigned)s_mr_port,
             s_pause_cmd == PAUSE_CMD_M524 ? "m524" : "m25",
             s_filament_check ? "true" : "false", echo, s_autotemp ? "true" : "false",
             s_pause_retract_x10 / 10.0f, (unsigned)s_pause_temp_c,
             s_pause_wipe ? "true" : "false");
}

bool printer_comm_pause_wipe(void)
{
    return s_pause_wipe;
}

/* ---- Ender Smart (lote 6): pause settings for host_print.c ---- */

float printer_comm_pause_retract_mm(void)
{
    return s_pause_retract_x10 / 10.0f;
}

int printer_comm_pause_temp_c(void)
{
    return s_pause_temp_c;
}

/* ---- Ender Smart (F2-03) ---- */

bool printer_comm_autotemp_active(void)
{
    return s_autotemp && esp_timer_get_time() - s_last_auto_temp_us < AUTOTEMP_FRESH_US;
}

esp_err_t printer_comm_request_info(void)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return ESP_ERR_NOT_SUPPORTED;
    if (!usb_serial_is_connected()) return ESP_ERR_INVALID_STATE;
    if (printer_comm_job_active()) return ESP_ERR_INVALID_STATE;
    printer_cmd_t cmd = { .type = PCMD_RAW };
    strlcpy(cmd.gcode, "M115", sizeof(cmd.gcode));
    return printer_comm_send_cmd(&cmd);
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

    /* lote 13: 8 KB (was 2 KB). The mesh the Ender prints after a leveling (5x5 plus the
     * 21x21 subdivided grid, ~4.5 KB in a burst) overflowed it on 2026-11-14 (rx_dropped 279) */
    s_rx_stream = xStreamBufferCreate(8192, 1);
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

    /* lote 8: host print module; reads the job journal (a job that was running when the
     * ESP32 went down gets its heaters turned off once the Ender answers) */
    hp_init();

    /* lote 6: 6144 -> 8192; printer_state_t grew (error_msg/notice) and is copied on the
     * stack here and in the mesh parser, which runs inside process_line.
     * lote 12: 8192 -> 10240; the lowest seen was 3624 B free (lote 11, fast send) */
    xTaskCreatePinnedToCore(printer_comm_task, "printer_comm", 10240,
                            NULL, 8, NULL, 1);  /* Core 1: with USB host, isolated from WiFi/HTTP */

    ESP_LOGI(TAG, "Printer comm initialized (Marlin backend)");
    return ESP_OK;
}

/* ---- Host print public API ---- */

esp_err_t printer_comm_prepare_begin(const char *filename, const char **why)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) {
        if (why) *why = "Só no modo Marlin";
        return ESP_ERR_NOT_SUPPORTED;
    }
    return hp_prepare_begin(filename, why);
}
void printer_comm_prepare_run(void) { hp_prepare_run(); }
bool printer_comm_preparing(char *name, size_t name_len, int *pct)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) {
        if (name && name_len) name[0] = '\0';
        if (pct) *pct = 0;
        return false;
    }
    return hp_preparing(name, name_len, pct);
}
bool printer_comm_prepare_cancel(void) { return s_backend == PRINTER_BACKEND_MARLIN && hp_prepare_cancel(); }

void printer_comm_ack_error(void)
{
    if (s_backend == PRINTER_BACKEND_MARLIN) {
        hp_ack_error();
        return;
    }
    printer_cmd_t cmd = { .type = PCMD_CANCEL };   /* Klipper: as before */
    printer_comm_send_cmd(&cmd);
}
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

/* F2-01 (lote 15): the line callback is in marlin_link.c, the synchronous GCode send in
 * command_arbiter.c */

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
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "config");
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
    char body[512];
    /* lote 6: a body longer than the buffer used to be cut silently (the last field,
     * filament_chk, could be lost and the sensor turned off): refuse it */
    if (req->content_len >= sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too long");
        return ESP_FAIL;
    }
    /* lote 6: the body may arrive in more than one piece: read all of it */
    int len = 0, timeouts = 0;
    while (len < (int)req->content_len) {
        int r = httpd_req_recv(req, body + len, req->content_len - len);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 3) continue;
        if (r <= 0) break;
        len += r;
    }
    if (len <= 0 || len < (int)req->content_len) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, len <= 0 ? "Empty body" : "Incomplete body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    /* Ender Smart (lote 6): pause retract (mm) and nozzle temperature while paused.
     * Validated before anything is changed; a field that is not sent keeps its value. */
    const char *pr = strstr(body, "pause_retract=");
    const char *pt = strstr(body, "pause_temp=");
    int new_retr = -1, new_ptemp = -1;
    if (pr) {
        char *endp = NULL;
        float v = strtof(pr + 14, &endp);
        if (endp == pr + 14 || !(v >= 0.0f && v <= 10.0f)) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "pause_retract: 0 a 10 mm");
            return ESP_FAIL;
        }
        new_retr = (int)(v * 10.0f + 0.5f);
    }
    if (pt) {
        char *endp = NULL;
        int v = (int)strtol(pt + 11, &endp, 10);
        if (endp == pt + 11 || !(v == 0 || (v >= 100 && v <= 260))) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "pause_temp: 0 (manter) ou 100 a 260 C");
            return ESP_FAIL;
        }
        new_ptemp = v;
    }
    if (new_retr >= 0) s_pause_retract_x10 = (uint16_t)new_retr;
    if (new_ptemp >= 0) s_pause_temp_c = (uint16_t)new_ptemp;
    /* lote 7: purge line on resume; a field that is not sent keeps its value */
    const char *pw = strstr(body, "pause_wipe=");
    if (pw) s_pause_wipe = (pw[11] == '1');

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

    /* Ender Smart (F2-03) */
    bool autotemp_given = strstr(body, "autotemp=") != NULL;
    if (autotemp_given) {
        bool on = strstr(body, "autotemp=1") != NULL;
        if (on != s_autotemp) {
            s_autotemp = on;
            s_autotemp_dirty = true;
        }
    }

    /* Persist to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_PAUSE_CMD, (uint8_t)new_cmd);
        nvs_set_u8(nvs, NVS_KEY_FILAMENT_CHK, new_fil_chk ? 1 : 0);
        nvs_set_str(nvs, NVS_KEY_ECHO_PING, s_echo_ping_cmds);
        if (autotemp_given) nvs_set_u8(nvs, NVS_KEY_AUTOTEMP, s_autotemp ? 1 : 0);
        if (new_retr >= 0) nvs_set_u16(nvs, NVS_KEY_PAUSE_RETR, s_pause_retract_x10);
        if (new_ptemp >= 0) nvs_set_u16(nvs, NVS_KEY_PAUSE_TEMP, s_pause_temp_c);
        if (pw) nvs_set_u8(nvs, NVS_KEY_PAUSE_WIPE, s_pause_wipe ? 1 : 0);
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
