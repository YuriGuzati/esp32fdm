/*
 * Ender Smart (F2-01, lote 15): command arbiter: the command queue and what may run when (policy
 * during a job, preparation and emergency stop).
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

/* ---- Arbiter state ---- */

/* ---- Synchronous command send ---- */
static SemaphoreHandle_t s_sync_cmd_sem;
/* Ender Smart (F15-18 / A2): the old single "pending" flag was released by ANY raw
 * command finishing and ignored the result, so a failed G28 still started G29. */
static volatile uint8_t   s_sync_wait_id;   /* id of the command a caller waits for */
static volatile bool      s_sync_ok;        /* "ok" received for that command */
static uint8_t            s_sync_seq;

QueueHandle_t s_cmd_queue;

/* lote 12 (F3-07): emergency stop, set from the HTTP task, handled by the printer task.
 * s_halted latches after a real M112: nothing is written to the USB until the link is opened
 * again (usb_serial_connect_gen() changes: the Ender was power-cycled or replugged). */
volatile bool s_estop;
static volatile bool s_estop_m112;
volatile bool s_halted;
volatile uint32_t s_halt_gen;
volatile bool s_kill_seen;
volatile int  s_estop_mode;     /* for hp_cmd_estop(): 0 test, 1 M112 written, 2 failed */

/** Ender Smart (lote 9): a command (pause, cancel, API) is waiting in the queue */
bool printer_comm_cmd_waiting(void)
{
    return s_cmd_queue && uxQueueMessagesWaiting(s_cmd_queue) > 0;
}

bool printer_comm_job_active(void)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    return st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED ||
           (s_backend == PRINTER_BACKEND_MARLIN && (hp_is_active() || hp_prepare_busy()));
}

bool printer_comm_cmd_allowed(const char *gcode, char *reason, size_t reason_len)
{
    if (s_halted) {
        snprintf(reason, reason_len, "%s", "A Ender está travada pela parada de emergência: tire o USB da ESP32, desligue e religue a Ender");
        return false;
    }
    printer_state_t st;
    printer_comm_get_state(&st);
    bool active = st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED ||
                  (s_backend == PRINTER_BACKEND_MARLIN && (hp_is_active() || hp_prepare_busy()));
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

/* ---- lote 12 (F3-07): emergency stop ---- */
bool printer_comm_estop_requested(void) { return s_estop || s_halted; }

bool printer_comm_halted(void) { return s_halted; }

bool printer_comm_kill_seen(void) { return s_kill_seen; }

esp_err_t printer_comm_emergency_stop(bool test_no_m112)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return ESP_ERR_NOT_SUPPORTED;
    if (!usb_serial_is_connected()) return ESP_ERR_INVALID_STATE;
    if (test_no_m112) {
        if (s_halted) return ESP_ERR_INVALID_STATE;   /* the real stop's message stays */
        s_estop_m112 = false;
        s_estop_mode = 0;
        s_estop = true;                /* no more job lines */
        hp_prepare_cancel();
        ESP_LOGW(TAG, "EMERGENCY STOP (TEST): M112 not sent; the job is interrupted, heaters off, no motion");
        return ESP_OK;
    }
    /* Real stop. Latch first: from here marlin_send_cmd_ex() writes nothing and stops waiting,
     * and hp_tick() sends no job line (printer_comm_estop_requested() includes s_halted). The
     * printer task is told (s_estop) only after the write, with its result known (review).
     * A 2nd M112 keeps s_kill_seen: a halted Ender does not print kill() again. */
    if (!s_halted) s_kill_seen = false;
    s_halt_gen = usb_serial_connect_gen();
    s_halted = true;
    hp_prepare_cancel();               /* a preparation stops; the stop's message says why */
    /* the leading newline ends a partial line in the Ender's buffer, if a byte was lost */
    static const char m112[] = "\nM112\n";
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++)
        err = usb_serial_send((const uint8_t *)m112, sizeof(m112) - 1);
    terminal_feed_tx("M112");
    if (err != ESP_OK) {
        /* the Ender did not get it and is still running: no latch, so the job can at least
         * be interrupted with the heaters off; the screen says to use the power button */
        ESP_LOGE(TAG, "EMERGENCY STOP: M112 could NOT be written to the USB (%s)", esp_err_to_name(err));
        s_estop_m112 = false;
        s_estop_mode = 2;
        s_estop = true;                /* before releasing the latch: no gap for hp_tick() */
        s_halted = false;
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "EMERGENCY STOP: M112 written to the USB");
    s_estop_m112 = true;
    s_estop_mode = 1;
    s_estop = true;
    return ESP_OK;
}

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

/** F2-01: moved from printer_comm_task(). After an emergency stop: nothing queued runs. */
void arbiter_drop_queue_estop(void)
{
    printer_cmd_t stale;
    while (xQueueReceive(s_cmd_queue, &stale, 0) == pdTRUE) {
        if (stale.type == PCMD_HOST_START)
            hp_cmd_start(stale.gcode);   /* the preparation was cancelled: only cleans up */
        else
            ESP_LOGW(TAG, "Emergency stop: dropped queued command (type %d) %.40s",
                     stale.type, stale.type == PCMD_RAW ? stale.gcode : "");
    }
}

/** F2-01: moved from printer_comm_task(). While halted after M112: nothing queued runs. */
void arbiter_drop_queue_halted(void)
{
    printer_cmd_t stale;
    while (xQueueReceive(s_cmd_queue, &stale, 0) == pdTRUE) {
        if (stale.type == PCMD_HOST_START)
            hp_cmd_start(stale.gcode);
        else
            ESP_LOGW(TAG, "Ender halted (emergency stop): dropped queued command (type %d) %.40s",
                     stale.type, stale.type == PCMD_RAW ? stale.gcode : "");
    }
}

/** F2-01: moved from printer_comm_task(). The Ender went away. Called right after
 *  hp_notify_usb_lost() and hp_prepare_abort() in printer_comm_task(): the "cancelled above" in the
 *  comments inside is that hp_prepare_abort(). */
void arbiter_drop_queue_disconnected(void)
{
    printer_cmd_t stale;
    while (xQueueReceive(s_cmd_queue, &stale, 0) == pdTRUE) {
        if (stale.type == PCMD_HOST_START) {
            hp_cmd_start(stale.gcode);          /* cancelled above: only cleans up */
        } else if ((stale.type == PCMD_HOST_CANCEL || stale.type == PCMD_HOST_SAFETY_CANCEL) &&
                   hp_is_active()) {
            /* A job is on: hp_notify_usb_lost() marked it, and hp_tick() interrupts it
             * with the heaters off when the Ender is back. Ending it here would leave
             * nobody to turn them off. */
            ESP_LOGW(TAG, "Ender disconnected: cancel dropped, the job is interrupted "
                     "(heaters off) when the Ender is back%s%.60s",
                     stale.type == PCMD_HOST_SAFETY_CANCEL ? "; the Ender had said: " : "",
                     stale.type == PCMD_HOST_SAFETY_CANCEL ? stale.gcode : "");
        } else if (stale.type == PCMD_HOST_CANCEL) {
            hp_cmd_cancel();                    /* no job: only clears the error */
        } else if (stale.type == PCMD_HOST_SAFETY_CANCEL) {
            hp_cmd_safety_cancel(stale.gcode);  /* no job: only adds to the error */
        } else {
            ESP_LOGW(TAG, "Ender disconnected: dropped queued command (type %d) %.40s",
                     stale.type, stale.type == PCMD_RAW ? stale.gcode : "");
        }
    }
}

/** F2-01: moved from printer_comm_task(). Runs the queued commands (host print, API, terminal). */
void arbiter_dispatch_queue(void)
{
    printer_cmd_t cmd;
    while (!s_estop && !s_halted && xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
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
        } else if (cmd.type == PCMD_HOST_SAFETY_CANCEL) {
            hp_cmd_safety_cancel(cmd.gcode); continue;
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
}
