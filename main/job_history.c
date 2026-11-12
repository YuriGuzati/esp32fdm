/* Ender Smart (F3-06): history of host prints.
 *
 * One JSON object per line in /cache/jobs.jsonl (LittleFS). Appending is cheap; when the
 * file holds more than JOB_HISTORY_MAX + JOB_HISTORY_SLACK lines it is rewritten with the
 * newest JOB_HISTORY_MAX. Only prints streamed by this ESP32 are recorded (a print started
 * from the Ender's own screen and SD card is not visible here). */

#include "job_history.h"
#include "httpd.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "job_history";

#define JOB_HISTORY_FILE  "/cache/jobs.jsonl"
#define JOB_HISTORY_TMP   "/cache/jobs.tmp"
#define JOB_HISTORY_MAX   50
#define JOB_HISTORY_SLACK 10
#define JOB_LINE_MAX      640   /* file (160) + reason (200) + fields, escaped */

static SemaphoreHandle_t s_lock;

static void lock(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

static void json_esc(const char *in, char *out, size_t out_size)
{
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)(in ? in : ""); *p && o + 7 < out_size; p++) {
        if (*p == '"' || *p == '\\') { out[o++] = '\\'; out[o++] = (char)*p; }
        else if (*p < 0x20) o += snprintf(out + o, out_size - o, "\\u%04x", *p);
        else out[o++] = (char)*p;
    }
    out[o] = '\0';
}

/* Keep only the newest JOB_HISTORY_MAX lines. Caller holds the lock. */
static void trim_if_needed(void)
{
    FILE *f = fopen(JOB_HISTORY_FILE, "r");
    if (!f) return;
    char *line = malloc(JOB_LINE_MAX + 2);
    if (!line) { fclose(f); return; }
    int n = 0;
    while (fgets(line, JOB_LINE_MAX + 2, f)) n++;
    if (n <= JOB_HISTORY_MAX + JOB_HISTORY_SLACK) { free(line); fclose(f); return; }

    rewind(f);
    FILE *t = fopen(JOB_HISTORY_TMP, "w");
    if (t) {
        int i = 0;
        while (fgets(line, JOB_LINE_MAX + 2, f)) {
            if (i++ >= n - JOB_HISTORY_MAX) fputs(line, t);
        }
        fclose(t);
    }
    fclose(f);
    free(line);
    if (t) {
        remove(JOB_HISTORY_FILE);
        rename(JOB_HISTORY_TMP, JOB_HISTORY_FILE);
        ESP_LOGI(TAG, "History trimmed to %d jobs", JOB_HISTORY_MAX);
    }
}

void job_history_add(const job_record_t *r)
{
    char file[160], reason[200], line[JOB_LINE_MAX];
    json_esc(r->file, file, sizeof(file));
    json_esc(r->reason, reason, sizeof(reason));
    snprintf(line, sizeof(line),
             "{\"file\":\"%s\",\"start\":%lld,\"duration\":%ld,\"result\":\"%s\",\"reason\":\"%s\","
             "\"lines\":%ld,\"total_lines\":%ld,\"layer\":%ld,\"total_layers\":%ld}\n",
             file, (long long)r->start_epoch, (long)r->duration_s, r->result, reason,
             (long)r->lines_sent, (long)r->total_lines, (long)r->layer, (long)r->total_layers);

    lock();
    FILE *f = fopen(JOB_HISTORY_FILE, "a");
    if (f) {
        fputs(line, f);
        fclose(f);
        trim_if_needed();
    }
    unlock();
    if (f) ESP_LOGI(TAG, "Recorded job: %s, %s, %ld s", r->file, r->result, (long)r->duration_s);
    else ESP_LOGW(TAG, "Could not open %s", JOB_HISTORY_FILE);
}

/* GET /api/history — JSON array, newest first */
static esp_err_t history_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    lock();
    FILE *f = fopen(JOB_HISTORY_FILE, "r");
    if (!f) {
        unlock();
        return httpd_resp_sendstr(req, "[]");
    }
    /* At most JOB_HISTORY_MAX + JOB_HISTORY_SLACK lines of JOB_LINE_MAX: read them all */
    const int cap = JOB_HISTORY_MAX + JOB_HISTORY_SLACK;
    char **lines = calloc(cap, sizeof(char *));
    int n = 0;
    char buf[JOB_LINE_MAX + 2];
    while (lines && fgets(buf, sizeof(buf), f)) {
        size_t len = strlen(buf);
        while (len && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) buf[--len] = '\0';
        if (!len || buf[0] != '{') continue;
        if (n == cap) { free(lines[0]); memmove(lines, lines + 1, (cap - 1) * sizeof(char *)); n--; }
        lines[n] = strdup(buf);
        if (lines[n]) n++;
    }
    fclose(f);
    unlock();

    httpd_resp_sendstr_chunk(req, "[");
    for (int i = n - 1; i >= 0; i--) {
        if (i != n - 1) httpd_resp_sendstr_chunk(req, ",");
        httpd_resp_sendstr_chunk(req, lines[i]);
        free(lines[i]);
    }
    free(lines);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* POST /api/history/clear */
static esp_err_t history_clear_handler(httpd_req_t *req)
{
    lock();
    remove(JOB_HISTORY_FILE);
    unlock();
    ESP_LOGI(TAG, "History cleared");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t job_history_register_httpd(httpd_handle_t server)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    httpd_uri_t get = { .uri = "/api/history", .method = HTTP_GET, .handler = history_get_handler };
    HTTPD_REGISTER(server, &get);
    httpd_uri_t clr = { .uri = "/api/history/clear", .method = HTTP_POST, .handler = history_clear_handler };
    HTTPD_REGISTER(server, &clr);
    return ESP_OK;
}
