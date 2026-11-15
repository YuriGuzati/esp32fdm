/* Ender Smart (lote 14): what happened in the run before this boot. See bootdiag.h. */
#include "bootdiag.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "printer_comm.h"
#include "wifi.h"

static const char *TAG = "bootdiag";

#define BD_NS        "bootdiag"
#define BD_KEY_BOOTS "boots"
#define BD_KEY_REC   "rec"
#define BD_RTC_MAGIC 0x45534254u            /* "ESBT" */
#define BD_FAST_S    600                    /* first 10 min: save every minute */
#define BD_FAST_EVERY_S 60
#define BD_SLOW_EVERY_S 600
#define BD_WIFI_MIN_GAP_S 30                /* a Wi-Fi change is saved at most this often */
#define BD_FLAG_PROVISIONAL 0x01            /* only the record written at boot (uptime 0) */

typedef struct {
    uint32_t boot;          /* boot number this record belongs to */
    uint32_t uptime_s;      /* uptime at the save */
    uint32_t epoch;         /* wall clock at the save (0: not synced yet) */
    uint32_t wifi_first_s;  /* uptime of the first Wi-Fi connection (0: never) */
    uint16_t disc_reason;   /* last Wi-Fi disconnect reason (0: none) */
    uint8_t  wifi_now;      /* connected at the save */
    uint8_t  eap;           /* emergency AP open at the save */
    uint8_t  job;           /* a job was active at the save */
    uint8_t  started;       /* esp_reset_reason_t that started that run */
    uint16_t flags;         /* BD_FLAG_* */
} bd_rec_t;

typedef struct {
    uint32_t magic;
    bd_rec_t rec;
    uint32_t chk;
} bd_rtc_t;

/* Survives panic, watchdog and software resets; garbage after a power loss (the check says so).
 * Holds the whole record (review): with a soft reset everything in it is at most 5 s old. */
static RTC_NOINIT_ATTR bd_rtc_t s_rtc;

static uint32_t s_boot;
static bd_rec_t s_prev;
static bool s_have_prev;
static const char *s_prev_src = "none";     /* "rtc": up to 5 s old; "nvs": up to 1-10 min old */
static volatile bool s_started;             /* printer_comm is up: its state may be asked */
static volatile uint8_t s_job;              /* last known "job active", for the RTC timer */
static esp_timer_handle_t s_rtc_timer;
static SemaphoreHandle_t s_save_lock;       /* the task and bootdiag_save_now() (printer task) */
static bool s_saved_in_job;                 /* the last NVS save was during a job */
static int64_t s_job_save_us;               /* ... and when */
#define BD_JOB_EVERY_US (15 * 60 * 1000000LL)

static uint32_t fnv1a(const void *p, size_t n, uint32_t h)
{
    const uint8_t *b = (const uint8_t *)p;
    for (size_t i = 0; i < n; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t rtc_chk(const bd_rtc_t *r)
{
    return fnv1a(&r->rec, sizeof(r->rec), fnv1a(&r->magic, sizeof(r->magic), 2166136261u));
}

/* Only cheap getters (static fields): safe before wifi_init()/printer_comm_init() and from the
 * esp_timer task. The job flag comes from s_job, set by whoever can ask printer_comm. */
static void fill_rec(bd_rec_t *r)
{
    memset(r, 0, sizeof(*r));
    r->boot = s_boot;
    r->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    time_t now = time(NULL);
    r->epoch = now > 1600000000 ? (uint32_t)now : 0;
    int64_t first = wifi_first_connect_us();
    r->wifi_first_s = first > 0 ? (uint32_t)(first / 1000000) : 0;
    if (first > 0 && r->wifi_first_s == 0) r->wifi_first_s = 1;   /* 0 means "never" */
    r->disc_reason = wifi_last_disconnect_reason();
    r->wifi_now = wifi_is_connected() ? 1 : 0;
    r->eap = wifi_emergency_ap_active() ? 1 : 0;
    r->job = s_job;
    r->started = (uint8_t)esp_reset_reason();
}

static void rtc_timer_cb(void *arg)
{
    (void)arg;
    bd_rtc_t r = { .magic = BD_RTC_MAGIC };
    fill_rec(&r.rec);
    r.chk = rtc_chk(&r);
    s_rtc = r;
}

static bool save_rec(const bd_rec_t *r)
{
    static bool warned;
    nvs_handle_t h;
    esp_err_t err = nvs_open(BD_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, BD_KEY_REC, r, sizeof(*r));
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err != ESP_OK && !warned) {          /* once: a full NVS must not flood the log (review) */
        warned = true;
        ESP_LOGW(TAG, "Could not save the run record: %s (not repeated)", esp_err_to_name(err));
    }
    return err == ESP_OK;
}

void bootdiag_save_now(void)
{
    if (!s_started || !s_save_lock) return;
    xSemaphoreTake(s_save_lock, portMAX_DELAY);
    bool job = printer_comm_job_active();
    s_job = job ? 1 : 0;
    int64_t now = esp_timer_get_time();
    /* during a job at most every 15 min (the journal calls every 5): fewer flash writes while
     * printing; the start, a pause and a resume always go (2nd review) */
    if (!job || !s_saved_in_job || now - s_job_save_us >= BD_JOB_EVERY_US) {
        bd_rec_t r;
        fill_rec(&r);
        save_rec(&r);
        s_job_save_us = now;
        s_saved_in_job = job;
    }
    xSemaphoreGive(s_save_lock);
}

static void bootdiag_task(void *arg)
{
    (void)arg;
    uint32_t last_save_s = 0;
    uint8_t last_wifi = 2;                   /* unknown: the first round saves */
    uint32_t last_wifi_save_s = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000);
        s_job = printer_comm_job_active() ? 1 : 0;
        /* not during a job: a flash write stalls both cores. The job journal already writes,
         * and bootdiag_save_now() goes with it (review) */
        if (s_job) continue;
        uint8_t wifi_now = wifi_is_connected() ? 1 : 0;
        uint32_t every = up < BD_FAST_S ? BD_FAST_EVERY_S : BD_SLOW_EVERY_S;
        bool first = last_wifi == 2;
        bool due = up - last_save_s >= every;
        bool wifi_changed = wifi_now != last_wifi && up - last_wifi_save_s >= BD_WIFI_MIN_GAP_S;
        if (!first && !due && !wifi_changed) continue;
        /* under the lock, and the job checked again: a job that starts meanwhile saves its own
         * record (job=1) through the journal, and this one must not land after it (2nd review) */
        xSemaphoreTake(s_save_lock, portMAX_DELAY);
        bool job_now = printer_comm_job_active();
        if (!job_now) {
            bd_rec_t r;
            fill_rec(&r);
            save_rec(&r);
            s_saved_in_job = false;
        }
        xSemaphoreGive(s_save_lock);
        if (job_now) continue;
        /* moved on even when the save failed: no retry every 5 s (review) */
        last_save_s = up;
        if (first || wifi_changed) last_wifi_save_s = up;
        last_wifi = wifi_now;
    }
}

void bootdiag_init(void)
{
    s_save_lock = xSemaphoreCreateMutex();
    uint32_t boots = 0;
    bd_rec_t nv;
    bool have_nv = false;
    nvs_handle_t h;
    if (nvs_open(BD_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_get_u32(h, BD_KEY_BOOTS, &boots);
        size_t len = sizeof(nv);
        have_nv = nvs_get_blob(h, BD_KEY_REC, &nv, &len) == ESP_OK && len == sizeof(nv);
        s_boot = boots + 1;
        if (nvs_set_u32(h, BD_KEY_BOOTS, s_boot) == ESP_OK) nvs_commit(h);
        nvs_close(h);
    } else {
        s_boot = 1;
        ESP_LOGW(TAG, "NVS namespace unavailable: no boot record");
    }

    /* the RTC copy is at most 5 s old when no power was lost in between: prefer it. Compared with
     * the counter, not with the NVS record: that one may be older if a save failed (2nd review) */
    bool rtc_ok = s_rtc.magic == BD_RTC_MAGIC && s_rtc.chk == rtc_chk(&s_rtc) &&
                  boots != 0 && s_rtc.rec.boot == boots;
    if (rtc_ok) {
        s_prev = s_rtc.rec;
        s_have_prev = true;
        s_prev_src = "rtc";
    } else if (have_nv) {
        s_prev = nv;
        s_have_prev = true;
        s_prev_src = "nvs";
    }

    /* this run's first record: a run that dies before the next save still shows up */
    bd_rec_t r;
    fill_rec(&r);
    r.flags = BD_FLAG_PROVISIONAL;
    save_rec(&r);
    rtc_timer_cb(NULL);
    const esp_timer_create_args_t ta = { .callback = rtc_timer_cb, .name = "bootdiag_rtc" };
    if (esp_timer_create(&ta, &s_rtc_timer) == ESP_OK)
        esp_timer_start_periodic(s_rtc_timer, 5 * 1000000ULL);

    if (s_have_prev)
        ESP_LOGI(TAG, "Boot %lu. Previous run (boot %lu): up %lu s (%s%s), Wi-Fi %s%s, last disconnect "
                      "reason %u, emergency AP %s, job %s",
                 (unsigned long)s_boot, (unsigned long)s_prev.boot, (unsigned long)s_prev.uptime_s,
                 s_prev_src, (s_prev.flags & BD_FLAG_PROVISIONAL) ? ", boot record only" : "",
                 s_prev.wifi_now ? "connected" : "not connected",
                 s_prev.wifi_first_s ? "" : " (never connected)", (unsigned)s_prev.disc_reason,
                 s_prev.eap ? "open" : "closed", s_prev.job ? "active" : "none");
    else
        ESP_LOGI(TAG, "Boot %lu. No record of a previous run", (unsigned long)s_boot);
}

void bootdiag_start(void)
{
    /* after printer_comm_init(): printer_comm_job_active() needs its mutex */
    if (!s_save_lock) {
        ESP_LOGW(TAG, "No memory for the save lock: the run record is not kept up to date");
        return;
    }
    s_started = true;
    xTaskCreatePinnedToCore(bootdiag_task, "bootdiag", 3072, NULL, 1, NULL, 0);
}

uint32_t bootdiag_boot_count(void)
{
    return s_boot;
}

static const char *reason_name(uint8_t r)
{
    switch ((esp_reset_reason_t)r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "other";
    }
}

int bootdiag_json(char *buf, size_t buf_size)
{
    int n;
    if (!s_have_prev)
        n = snprintf(buf, buf_size, ",\"boot_count\":%lu,\"prev\":null", (unsigned long)s_boot);
    else
        n = snprintf(buf, buf_size,
                     ",\"boot_count\":%lu,\"prev\":{\"boot\":%lu,\"uptime_s\":%lu,\"src\":\"%s\","
                     "\"provisional\":%s,\"epoch\":%lu,\"wifi_first_s\":%lu,\"wifi_now\":%s,"
                     "\"disc_reason\":%u,\"emergency_ap\":%s,\"job\":%s,\"started\":\"%s\"}",
                     (unsigned long)s_boot, (unsigned long)s_prev.boot,
                     (unsigned long)s_prev.uptime_s, s_prev_src,
                     (s_prev.flags & BD_FLAG_PROVISIONAL) ? "true" : "false",
                     (unsigned long)s_prev.epoch, (unsigned long)s_prev.wifi_first_s,
                     s_prev.wifi_now ? "true" : "false", (unsigned)s_prev.disc_reason,
                     s_prev.eap ? "true" : "false", s_prev.job ? "true" : "false",
                     reason_name(s_prev.started));
    if (n < 0) n = 0;
    if ((size_t)n >= buf_size) n = buf_size ? (int)buf_size - 1 : 0;
    return n;
}
