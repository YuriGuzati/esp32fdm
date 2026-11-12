#include "logbuf.h"
#include "httpd.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "layout.h"

static const char *TAG = "logbuf";

/* ---- Ring buffer ---- */

#define LOGBUF_SIZE (2 * 1024 * 1024)   /* 2 MB in PSRAM */
#define FMT_BUF_SIZE 512                /* stack buffer for vsnprintf */

static char *s_buf;
static int   s_head;       /* next write position */
static int   s_count;      /* valid bytes in buffer (≤ LOGBUF_SIZE) */
static SemaphoreHandle_t s_mutex;
static vprintf_like_t s_orig_vprintf;

/* ---- Ring helpers ---- */

static void ring_write(const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        s_buf[s_head] = data[i];
        s_head = (s_head + 1) % LOGBUF_SIZE;
    }
    if (s_count + (int)len > LOGBUF_SIZE)
        s_count = LOGBUF_SIZE;
    else
        s_count += (int)len;
}

/* ---- esp_log hook ---- */

static int log_vprintf(const char *fmt, va_list args)
{
    /* Format into stack buffer */
    char tmp[FMT_BUF_SIZE];
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    if (len < 0) len = 0;
    if (len >= (int)sizeof(tmp)) len = (int)sizeof(tmp) - 1;

    /* Write to ring buffer */
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ring_write(tmp, len);
        xSemaphoreGive(s_mutex);
    }

    /* Forward to original vprintf (UART) */
    /* We already have the formatted string, just write it */
    for (int i = 0; i < len; i++)
        putchar(tmp[i]);

    return len;
}

/* ---- Init ---- */

void logbuf_init(void)
{
    s_buf = heap_caps_calloc(1, LOGBUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_buf) {
        printf("logbuf: PSRAM alloc failed!\n");
        return;
    }
    s_head = 0;
    s_count = 0;
    s_mutex = xSemaphoreCreateMutex();

    s_orig_vprintf = esp_log_set_vprintf(log_vprintf);
    ESP_LOGI(TAG, "Log buffer initialized (%d KB, PSRAM)", LOGBUF_SIZE / 1024);
}

/* ---- TAG classification ---- */

static bool is_printer_tag(const char *line, int len)
{
    /*
     * ESP_LOG format: "X (12345) TAG: message\n"
     * TAG starts after ") " — find it and check.
     */
    const char *paren_close = memchr(line, ')', len);
    if (!paren_close || paren_close - line + 2 >= len) return false;

    const char *tag_start = paren_close + 2;  /* skip ") " */
    int remaining = len - (int)(tag_start - line);

    if (remaining >= 12 && memcmp(tag_start, "printer_comm", 12) == 0) return true;
    if (remaining >= 7  && memcmp(tag_start, "klipper", 7) == 0)      return true;
    return false;
}

static bool is_heap_line(const char *line, int len)
{
    /* Match "HEAP [" anywhere in line — covers log_heap() and heap_monitor_task */
    return memmem(line, len, "HEAP [", 6) != NULL;
}

/* ---- HTTP handler (async) ---- */

typedef enum { FILTER_ALL, FILTER_PRINTER, FILTER_SYSTEM, FILTER_HEAP } log_filter_t;

typedef struct {
    httpd_req_t *req;
    log_filter_t filter;
} logs_ctx_t;

static bool s_logs_busy;   /* Ender Smart (F15-22): one /logs response at a time */

static void logs_task(void *arg)
{
    logs_ctx_t *ctx = (logs_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    log_filter_t filter = ctx->filter;
    bool send_ok = true;  /* track client disconnect */

    /* Snapshot the ring buffer to avoid holding mutex during HTTP send */
    char *snap = heap_caps_malloc(LOGBUF_SIZE, MALLOC_CAP_SPIRAM);
    int snap_len = 0;
    int snap_start = 0;

    if (snap && s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        snap_len = s_count;
        snap_start = (s_head - s_count + LOGBUF_SIZE) % LOGBUF_SIZE;
        /* Linearize ring into snapshot */
        if (snap_start + snap_len <= LOGBUF_SIZE) {
            memcpy(snap, s_buf + snap_start, snap_len);
        } else {
            int first = LOGBUF_SIZE - snap_start;
            memcpy(snap, s_buf + snap_start, first);
            memcpy(snap + first, s_buf, snap_len - first);
        }
        xSemaphoreGive(s_mutex);
    }

    httpd_resp_set_type(req, "text/html");

    /* HTML header via layout */
    html_buf_t page;
    html_buf_init(&page);
    layout_html_begin(&page, "Logs", "/logs");

    /* Filter buttons */
    html_buf_printf(&page,
        "<div style='margin:12px 0'>"
        "<b>Filter:</b> ");
    if (filter == FILTER_ALL)
        html_buf_printf(&page, "<b>[All]</b> ");
    else
        html_buf_printf(&page, "<a href='/logs'>All</a> ");
    if (filter == FILTER_PRINTER)
        html_buf_printf(&page, "<b>[Printer]</b> ");
    else
        html_buf_printf(&page, "<a href='/logs?filter=printer'>Printer</a> ");
    if (filter == FILTER_SYSTEM)
        html_buf_printf(&page, "<b>[System]</b> ");
    else
        html_buf_printf(&page, "<a href='/logs?filter=system'>System</a> ");
    if (filter == FILTER_HEAP)
        html_buf_printf(&page, "<b>[Heap]</b> ");
    else
        html_buf_printf(&page, "<a href='/logs?filter=heap'>Heap</a> ");
    html_buf_printf(&page,
        " | <form method='POST' action='/logs/clear' style='display:inline'>"
        "<button type='submit' onclick=\"return confirm('Clear all logs?')\">"
        "Clear</button></form>");
    html_buf_printf(&page, "</div>"
        "<pre style='background:#1e1e1e;color:#d4d4d4;padding:12px;"
        "font-size:13px;border-radius:4px;white-space:pre-wrap;word-wrap:break-word;"
        "max-height:70vh;overflow-y:auto' id='log'>");

    /* Send header */
    if (httpd_resp_send_chunk(req, page.data, page.len) != ESP_OK)
        send_ok = false;
    html_buf_free(&page);

    /* Stream log lines from snapshot.
     * Ender Smart (F15-22): the chunk buffer now spans lines and is flushed only
     * when full — the old code sent one HTTP chunk per log line (~37 000 tiny
     * chunks for a full 2 MB buffer), holding a socket for minutes. */
    char chunk[1024];
    int ci = 0;
    if (send_ok && snap && snap_len > 0) {
        int pos = 0;
        while (pos < snap_len && send_ok) {
            /* Find end of line */
            int line_start = pos;
            while (pos < snap_len && snap[pos] != '\n') pos++;
            if (pos < snap_len) pos++;  /* include the \n */

            int line_len = pos - line_start;

            /* Apply filter */
            if (filter != FILTER_ALL) {
                bool printer = is_printer_tag(snap + line_start, line_len);
                bool heap = is_heap_line(snap + line_start, line_len);
                if (filter == FILTER_PRINTER && !printer) continue;
                if (filter == FILTER_SYSTEM && printer) continue;
                if (filter == FILTER_HEAP && !heap) continue;
            }

            /* HTML-escape into the shared chunk buffer: we need to escape <, >, & */
            for (int i = line_start; i < line_start + line_len && send_ok; i++) {
                char c = snap[i];
                if (c == '<') {
                    if (ci + 4 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&lt;", 4); ci += 4;
                } else if (c == '>') {
                    if (ci + 4 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&gt;", 4); ci += 4;
                } else if (c == '&') {
                    if (ci + 5 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&amp;", 5); ci += 5;
                } else {
                    if (ci + 1 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    chunk[ci++] = c;
                }
            }
        }
    }
    if (ci > 0 && send_ok) {
        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
            send_ok = false;
    }

    if (send_ok) {
        /* HTML footer */
        html_buf_t foot;
        html_buf_init(&foot);
        html_buf_printf(&foot, "</pre>"
            "<script>var e=document.getElementById('log');e.scrollTop=e.scrollHeight;</script>");
        layout_html_end(&foot);
        httpd_resp_send_chunk(req, foot.data, foot.len);
        html_buf_free(&foot);

        /* End chunked response */
        httpd_resp_send_chunk(req, NULL, 0);
    }

    if (snap) heap_caps_free(snap);

    httpd_req_async_handler_complete(req);
    free(ctx);
    __atomic_clear(&s_logs_busy, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

static esp_err_t logs_handler(httpd_req_t *req)
{
    /* Ender Smart (F15-22): each request snapshots the whole 2 MB buffer in PSRAM;
     * two or three at once took heap_min from 3.7 MB to 1.7 MB on 2026-11-12.
     * Serve one at a time. */
    if (__atomic_test_and_set(&s_logs_busy, __ATOMIC_ACQ_REL)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "5");
        return httpd_resp_sendstr(req, "Logs are already being sent to another client, try again in a few seconds");
    }

    /* Parse filter from query string (before async clone) */
    log_filter_t filter = FILTER_ALL;
    char qbuf[32] = {0};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qbuf, "filter", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "printer") == 0) filter = FILTER_PRINTER;
            else if (strcmp(val, "system") == 0) filter = FILTER_SYSTEM;
            else if (strcmp(val, "heap") == 0) filter = FILTER_HEAP;
        }
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        __atomic_clear(&s_logs_busy, __ATOMIC_RELEASE);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    logs_ctx_t *ctx = malloc(sizeof(logs_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        __atomic_clear(&s_logs_busy, __ATOMIC_RELEASE);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    ctx->filter = filter;

    /* 6KB stack: 1KB chunk buf + html_buf + PSRAM snapshot pointer */
    if (xTaskCreatePinnedToCore(logs_task, "logs", 6144, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        __atomic_clear(&s_logs_busy, __ATOMIC_RELEASE);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- Clear handler ---- */

static esp_err_t logs_clear_handler(httpd_req_t *req)
{
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_head = 0;
        s_count = 0;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "Log buffer cleared");
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/logs");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ---- Registration ---- */

esp_err_t logbuf_register_httpd(httpd_handle_t server)
{
    httpd_uri_t uri_get = {
        .uri = "/logs", .method = HTTP_GET, .handler = logs_handler,
    };
    HTTPD_REGISTER(server, &uri_get);

    httpd_uri_t uri_clear = {
        .uri = "/logs/clear", .method = HTTP_POST, .handler = logs_clear_handler,
    };
    HTTPD_REGISTER(server, &uri_clear);

    ESP_LOGI(TAG, "Log viewer registered at /logs");
    return ESP_OK;
}
