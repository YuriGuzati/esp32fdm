/* Ender Smart (lote 16, F3-04): WebSocket of events at /api/v1/events.
 *
 * The page used to ask for /api/status every 2 s and /api/stats every 5 s. Now the ESP32 pushes
 * them to the open pages; the page falls back to the old polling whenever the socket is down.
 *
 * Threading: the handshake, the frames from the browser, the pushes and the closes all run in the
 * HTTP server task (the pushes through httpd_queue_work). The small "ws_events" task only paces the
 * ticks. So the JSON builders shared with the REST handlers (httpd_status_json/httpd_stats_json)
 * and the buffers below are only ever touched by that one task.
 *
 * The socket takes no commands: frames from the browser are read and dropped (they only show that
 * the client is alive, like the PONG to our PING). */

#include "events_ws.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "bootdiag.h"
#include "httpd.h"
#include "printer_comm.h"
#include "version.h"

static const char *TAG = "events_ws";

#define WS_MAGIC     0x45575331u   /* "EWS1": the session context is ours */
#define WS_FD_LIST   16            /* >= max_open_sockets of the main server (13) */
#define WS_MSG_SIZE  2048          /* status (<= ~1.6 KB with every text field full) + envelope */
#define WS_RX_MAX    128           /* the browser sends nothing we use; anything bigger is closed */

typedef struct {
    uint32_t magic;
    int64_t  last_rx_us;   /* last frame received from the browser (the PONG to our PING counts) */
    int64_t  close_req_us; /* when we asked the server to close it */
    bool     closing;
} ws_client_t;

#define WS_CLOSE_RETRY_US  (3 * 1000 * 1000)

static httpd_handle_t s_hd;
static int s_clients;                 /* accepted clients; changed only in the HTTP server task */
static bool s_tick_pending;           /* a tick is queued in the HTTP server task */
static uint32_t s_tick_queued_ms;
static TaskHandle_t s_pacer;

/* Only the HTTP server task touches these */
static bool s_force;                  /* a client was just accepted: status and stats on the next tick */
static char *s_msg;                   /* the message being built */
static char *s_last_status;           /* the last status object sent */
static int s_last_status_len = -1;
static int64_t s_last_status_us, s_last_stats_us, s_last_ping_us, s_last_temp_ts_us, s_last_tick_us;

int events_ws_client_count(void)
{
    return __atomic_load_n(&s_clients, __ATOMIC_ACQUIRE);
}

int events_ws_pacer_stack_free(void)
{
    return s_pacer ? (int)uxTaskGetStackHighWaterMark(s_pacer) : -1;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* NaN/inf would print "nan"/"inf", which is not JSON */
static float fin(float v)
{
    return isfinite(v) ? v : 0.0f;
}

/* Send for the WebSocket sessions: the whole buffer, or an error within EVENTS_WS_SEND_LIMIT_MS.
 * The server calls it twice per frame (header, then payload), so a frame waits at most twice
 * that. The default send blocks up to 5 s (send_wait_timeout) and may then write only part of a
 * frame; a phone that went to sleep with the page open would hold the whole web server that long
 * at every push. On error the frame send fails, the session is closed, and the page reconnects. */
static int ws_send_bounded(httpd_handle_t hd, int fd, const char *buf, size_t len, int flags)
{
    (void)hd;
    (void)flags;
    if (!buf) return HTTPD_SOCK_ERR_INVALID;
    int64_t deadline = esp_timer_get_time() + (int64_t)EVENTS_WS_SEND_LIMIT_MS * 1000;
    size_t off = 0;
    while (off < len) {
        int r = send(fd, buf + off, len - off, MSG_DONTWAIT);
        if (r > 0) {
            off += (size_t)r;
            continue;
        }
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            int64_t left = deadline - esp_timer_get_time();
            if (left <= 0) return HTTPD_SOCK_ERR_TIMEOUT;
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            struct timeval tv = { .tv_sec = 0, .tv_usec = left > 50000 ? 50000 : (long)left };
            select(fd + 1, NULL, &wfds, NULL, &tv);
            continue;
        }
        return HTTPD_SOCK_ERR_FAIL;
    }
    return (int)len;
}

/* Session context destructor: the HTTP server calls it when the session goes away (the page was
 * closed, the network dropped, we closed it, or the LRU purge took it). */
static void ws_ctx_free(void *ctx)
{
    ws_client_t *c = (ws_client_t *)ctx;
    if (c && c->magic == WS_MAGIC) {
        c->magic = 0;
        int left = __atomic_sub_fetch(&s_clients, 1, __ATOMIC_ACQ_REL);
        ESP_LOGI(TAG, "WebSocket client closed (%d open)", left);
    }
    free(ctx);
}

static void ws_drop(httpd_handle_t hd, int fd, ws_client_t *c, const char *why)
{
    if (c->closing) return;
    ESP_LOGW(TAG, "Closing WebSocket client fd %d: %s", fd, why);
    /* Nothing more is sent to it either way. The close goes through the server's control socket:
     * if that request is refused, ws_collect asks again on the next tick (a lost one, after 3 s),
     * or the client would keep its slot with nothing ever sent to it. */
    c->closing = true;
    c->close_req_us = esp_timer_get_time();
    if (httpd_sess_trigger_close(hd, fd) != ESP_OK) c->close_req_us -= WS_CLOSE_RETRY_US;
}

static int ws_collect(httpd_handle_t hd, int *fds_out, ws_client_t **cs, int max, int64_t now)
{
    size_t n = WS_FD_LIST;
    int fds[WS_FD_LIST];
    if (httpd_get_client_list(hd, &n, fds) != ESP_OK) return 0;
    int k = 0;
    for (size_t i = 0; i < n && k < max; i++) {
        if (httpd_ws_get_fd_info(hd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        ws_client_t *c = (ws_client_t *)httpd_sess_get_ctx(hd, fds[i]);
        if (!c || c->magic != WS_MAGIC) continue;
        if (c->closing) {
            /* still here 3 s after we asked: that close request was lost, ask again */
            if (now - c->close_req_us > WS_CLOSE_RETRY_US) {
                c->close_req_us = now;
                httpd_sess_trigger_close(hd, fds[i]);
            }
            continue;
        }
        fds_out[k] = fds[i];
        cs[k] = c;
        k++;
    }
    return k;
}

static void ws_broadcast(httpd_handle_t hd, const int *fds, ws_client_t **cs, int n,
                         httpd_ws_type_t type, const char *data, size_t len)
{
    httpd_ws_frame_t f = {
        .final = true, .fragmented = false, .type = type,
        .payload = (uint8_t *)data, .len = len,
    };
    for (int i = 0; i < n; i++) {
        if (cs[i]->closing) continue;
        if (httpd_ws_send_frame_async(hd, fds[i], &f) != ESP_OK)
            ws_drop(hd, fds[i], cs[i], "send failed or took too long");
    }
}

/* One tick, in the HTTP server task */
static void ws_tick_work(void *arg)
{
    httpd_handle_t hd = (httpd_handle_t)arg;
    int fds[EVENTS_WS_MAX_CLIENTS + 4];
    ws_client_t *cs[EVENTS_WS_MAX_CLIENTS + 4];
    int64_t now = esp_timer_get_time();
    int n = ws_collect(hd, fds, cs, (int)(sizeof(fds) / sizeof(fds[0])), now);
    bool force = s_force;
    s_force = false;
    /* The server task was held (a long request) since the last tick: the PONGs of that time may be
     * waiting unread in the sockets (the server runs this work before it reads them), so that time
     * does not count as silence. */
    int64_t held = s_last_tick_us != 0 ? now - s_last_tick_us - (int64_t)EVENTS_WS_TICK_MS * 1000 : 0;
    s_last_tick_us = now;
    if (n == 0 || !s_msg || !s_last_status) goto done;

    /* 1. A client that answers nothing, not even the PONG to our PING */
    for (int i = 0; i < n; i++) {
        if (held > (int64_t)EVENTS_WS_TICK_MS * 1000) {
            cs[i]->last_rx_us += held;
            if (cs[i]->last_rx_us > now) cs[i]->last_rx_us = now;
        }
        if (now - cs[i]->last_rx_us > (int64_t)EVENTS_WS_IDLE_CLOSE_MS * 1000)
            ws_drop(hd, fds[i], cs[i], "no answer from the browser for 35 s");
    }

    /* 2. Status: when it changes, and at least every 5 s (so the page can tell a dead link) */
    static const char ST_PRE[] = "{\"t\":\"status\",\"d\":";
    const size_t st_pre = sizeof(ST_PRE) - 1;
    memcpy(s_msg, ST_PRE, st_pre);
    int sl = httpd_status_json(s_msg + st_pre, WS_MSG_SIZE - st_pre - 2);
    if (sl > 0) {
        bool changed = sl != s_last_status_len || memcmp(s_last_status, s_msg + st_pre, (size_t)sl) != 0;
        if (changed || force || now - s_last_status_us >= (int64_t)EVENTS_WS_HEARTBEAT_MS * 1000) {
            memcpy(s_last_status, s_msg + st_pre, (size_t)sl);
            s_last_status_len = sl;
            s_msg[st_pre + sl] = '}';
            s_msg[st_pre + sl + 1] = '\0';
            ws_broadcast(hd, fds, cs, n, HTTPD_WS_TYPE_TEXT, s_msg, st_pre + (size_t)sl + 1);
            s_last_status_us = now;
        }
    }

    /* 3. Stats (CPU, memory, Wi-Fi...), every 5 s */
    if (force || now - s_last_stats_us >= (int64_t)EVENTS_WS_STATS_MS * 1000) {
        static const char SS_PRE[] = "{\"t\":\"stats\",\"d\":";
        const size_t ss_pre = sizeof(SS_PRE) - 1;
        memcpy(s_msg, SS_PRE, ss_pre);
        int tl = httpd_stats_json(s_msg + ss_pre, WS_MSG_SIZE - ss_pre - 2, HTTPD_CPU_WS);
        if (tl > 0) {
            s_msg[ss_pre + tl] = '}';
            s_msg[ss_pre + tl + 1] = '\0';
            ws_broadcast(hd, fds, cs, n, HTTPD_WS_TYPE_TEXT, s_msg, ss_pre + (size_t)tl + 1);
        }
        s_last_stats_us = now;
    }

    /* 4. Each new temperature sample, in the same row format as GET /api/temp_history */
    temp_sample_t smp;
    if (printer_comm_get_temp_history(&smp, 1) == 1 && smp.timestamp_us != s_last_temp_ts_us) {
        s_last_temp_ts_us = smp.timestamp_us;
        int tl = snprintf(s_msg, WS_MSG_SIZE, "{\"t\":\"temp\",\"d\":[%lld,%.1f,%.0f,%.1f,%.0f]}",
                          (long long)(smp.timestamp_us / 1000000), fin(smp.hotend_actual), fin(smp.hotend_target),
                          fin(smp.bed_actual), fin(smp.bed_target));
        if (tl > 0 && tl < WS_MSG_SIZE)
            ws_broadcast(hd, fds, cs, n, HTTPD_WS_TYPE_TEXT, s_msg, (size_t)tl);
    }

    /* 5. PING: the browser answers by itself (also in a background tab) */
    if (now - s_last_ping_us >= (int64_t)EVENTS_WS_PING_MS * 1000) {
        ws_broadcast(hd, fds, cs, n, HTTPD_WS_TYPE_PING, NULL, 0);
        s_last_ping_us = now;
    }

done:
    __atomic_store_n(&s_tick_pending, false, __ATOMIC_RELEASE);
}

static void ws_kick(void)
{
    uint32_t t = now_ms();
    if (__atomic_load_n(&s_tick_pending, __ATOMIC_ACQUIRE)) {
        /* The server task is busy with a long request. After 10 s assume the queued item was lost,
         * or the pushes would stop for good; a duplicate tick is harmless. */
        if (t - __atomic_load_n(&s_tick_queued_ms, __ATOMIC_ACQUIRE) < 10000) return;
    }
    __atomic_store_n(&s_tick_queued_ms, t, __ATOMIC_RELEASE);
    __atomic_store_n(&s_tick_pending, true, __ATOMIC_RELEASE);
    if (httpd_queue_work(s_hd, ws_tick_work, s_hd) != ESP_OK)
        __atomic_store_n(&s_tick_pending, false, __ATOMIC_RELEASE);
}

static void ws_pacer_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EVENTS_WS_TICK_MS));
        if (events_ws_client_count() > 0) ws_kick();
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        if (httpd_ws_get_fd_info(req->handle, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            /* A plain GET (no "Upgrade: websocket"), e.g. through a proxy: the server calls the
             * handler without a handshake. Answer it instead of dropping the connection. */
            httpd_resp_set_status(req, "426 Upgrade Required");
            httpd_resp_set_hdr(req, "Upgrade", "websocket");
            httpd_resp_set_type(req, "text/plain");
            return httpd_resp_sendstr(req, "WebSocket only (ws://<esp32>/api/v1/events)");
        }
        /* The HTTP server has just answered the handshake (101) */
        if (events_ws_client_count() >= EVENTS_WS_MAX_CLIENTS) {
            char full[40];
            int fl = snprintf(full, sizeof(full), "{\"t\":\"full\",\"max\":%d}", EVENTS_WS_MAX_CLIENTS);
            httpd_ws_frame_t f = { .final = true, .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)full, .len = (size_t)fl };
            httpd_ws_send_frame(req, &f);
            ESP_LOGW(TAG, "WebSocket refused (fd %d): already %d clients", fd, EVENTS_WS_MAX_CLIENTS);
            return ESP_FAIL;   /* the server closes this socket */
        }
        httpd_sess_set_send_override(req->handle, fd, ws_send_bounded);
        /* small frames, sent at once: without this each payload waits for the ACK of its header */
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        const esp_app_desc_t *app = esp_app_get_description();
        char hello[320];
        int hl = snprintf(hello, sizeof(hello),
            "{\"t\":\"hello\",\"api\":1,\"boot\":%lu,\"fw\":\"%s\",\"fw_label\":\"%s\","
            "\"tick_ms\":%d,\"heartbeat_ms\":%d,\"stats_ms\":%d,\"ping_ms\":%d,\"max_clients\":%d}",
            (unsigned long)bootdiag_boot_count(), app ? app->version : "?", FW_VERSION,
            EVENTS_WS_TICK_MS, EVENTS_WS_HEARTBEAT_MS, EVENTS_WS_STATS_MS, EVENTS_WS_PING_MS,
            EVENTS_WS_MAX_CLIENTS);
        if (hl <= 0 || hl >= (int)sizeof(hello)) return ESP_FAIL;
        httpd_ws_frame_t hf = { .final = true, .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)hello, .len = (size_t)hl };
        if (httpd_ws_send_frame(req, &hf) != ESP_OK) return ESP_FAIL;

        ws_client_t *c = calloc(1, sizeof(*c));
        if (!c) return ESP_FAIL;
        c->magic = WS_MAGIC;
        c->last_rx_us = esp_timer_get_time();
        /* Only now: every failure above leaves no context and no count behind */
        req->sess_ctx = c;
        req->free_ctx = ws_ctx_free;
        int open = __atomic_add_fetch(&s_clients, 1, __ATOMIC_ACQ_REL);
        ESP_LOGI(TAG, "WebSocket client fd %d connected (%d open)", fd, open);
        s_force = true;
        ws_kick();
        return ESP_OK;
    }

    /* A frame from the browser (text, binary or the PONG to our PING): read it whole and drop it */
    httpd_ws_frame_t f;
    memset(&f, 0, sizeof(f));
    if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;
    if (f.len > WS_RX_MAX) return ESP_FAIL;
    uint8_t buf[WS_RX_MAX + 1];
    if (f.len > 0) {
        f.payload = buf;
        if (httpd_ws_recv_frame(req, &f, f.len) != ESP_OK) return ESP_FAIL;
    }
    ws_client_t *c = (ws_client_t *)req->sess_ctx;
    if (c && c->magic == WS_MAGIC) c->last_rx_us = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t events_ws_register(httpd_handle_t server)
{
    s_hd = server;
    s_msg = heap_caps_malloc_prefer(WS_MSG_SIZE, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    s_last_status = heap_caps_malloc_prefer(WS_MSG_SIZE, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    if (!s_msg || !s_last_status) {
        ESP_LOGE(TAG, "No memory for the event buffers; /api/v1/events not registered");
        return ESP_ERR_NO_MEM;
    }
    httpd_uri_t ws = {
        .uri = "/api/v1/events", .method = HTTP_GET, .handler = ws_handler,
        .is_websocket = true, .handle_ws_control_frames = false,
    };
    HTTPD_REGISTER(server, &ws);
    if (xTaskCreatePinnedToCore(ws_pacer_task, "ws_events", 3072, NULL, 3, &s_pacer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Could not start the ws_events task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Events at /api/v1/events (up to %d clients)", EVENTS_WS_MAX_CLIENTS);
    return ESP_OK;
}
