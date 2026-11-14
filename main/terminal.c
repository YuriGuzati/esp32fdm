#include "terminal.h"
#include "httpd.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "printer_comm.h"
#include "layout.h"
#include "url_util.h"

static const char *TAG = "terminal";

/* ---- Ring buffer ----
 * Ender Smart (lote 10): 64 KB instead of 4 KB. During a fast host print the echo of the job
 * lines ("> N.. G1 ..*cs" and "ok") passes 7 KB/s, and 4 KB held less than 0.6 s: a page that
 * polls once a second lost lines, and the first line of each answer came cut.
 *
 * The writers (USB RX callback, printer task, HTTP send) take a spinlock only for the copy, so
 * they never drop bytes; before, they dropped them whenever a poll held the mutex. A poll copies
 * without the lock and checks afterwards that no writer reached what it copied (seqlock style). */

#define TERM_RING_SIZE  (64 * 1024)                       /* power of two */
#define TERM_POLL_MAX   (16 * 1024)                       /* most bytes in one poll answer */
#define TERM_SAFE_SPAN  (TERM_RING_SIZE - 16 * 1024)      /* a poll never starts older than this */

static char *s_ring;                /* allocated in PSRAM */
static uint64_t s_seq = 0;          /* monotonic total bytes written; ring index = seq % size */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_ready;
static uint32_t s_boot_id;           /* random per boot, made at the first poll (Wi-Fi is up) */

void terminal_init(void)
{
    s_ring = heap_caps_calloc(1, TERM_RING_SIZE, MALLOC_CAP_SPIRAM);
    s_ready = (s_ring != NULL);
    if (s_ready)
        ESP_LOGI(TAG, "Terminal initialized (ring %d bytes, PSRAM)", TERM_RING_SIZE);
    else
        ESP_LOGE(TAG, "Terminal ring allocation failed: the terminal stays empty");
}

/* call with s_lock held */
static void ring_put_locked(const char *d, size_t len)
{
    if (len > TERM_RING_SIZE) {             /* only the last ring's worth can stay */
        s_seq += len - TERM_RING_SIZE;
        d += len - TERM_RING_SIZE;
        len = TERM_RING_SIZE;
    }
    size_t pos = (size_t)(s_seq & (TERM_RING_SIZE - 1));
    size_t first = TERM_RING_SIZE - pos;
    if (first > len) first = len;
    memcpy(s_ring + pos, d, first);
    if (len > first) memcpy(s_ring, d + first, len - first);
    s_seq += len;
}

/* Up to three pieces, kept together (a "> cmd\n" line is never split by RX bytes) */
static void ring_write3(const char *a, size_t al, const char *b, size_t bl, const char *c, size_t cl)
{
    if (!s_ready) return;
    portENTER_CRITICAL_SAFE(&s_lock);
    if (al) ring_put_locked(a, al);
    if (bl) ring_put_locked(b, bl);
    if (cl) ring_put_locked(c, cl);
    portEXIT_CRITICAL_SAFE(&s_lock);
}

static uint64_t ring_seq(void)
{
    portENTER_CRITICAL_SAFE(&s_lock);
    uint64_t v = s_seq;
    portEXIT_CRITICAL_SAFE(&s_lock);
    return v;
}

/* Without the lock: the caller checks ring_seq() afterwards (see terminal_poll_handler) */
static void ring_copy(uint64_t from, char *out, size_t n)
{
    size_t pos = (size_t)(from & (TERM_RING_SIZE - 1));
    size_t first = TERM_RING_SIZE - pos;
    if (first > n) first = n;
    memcpy(out, s_ring + pos, first);
    if (n > first) memcpy(out + first, s_ring, n - first);
}

void terminal_feed_rx(const uint8_t *data, size_t len)
{
    /* Called from the USB Host callback: the spinlock is held only for the memcpy */
    if (!data || len == 0) return;
    ring_write3((const char *)data, len, NULL, 0, NULL, 0);
}

void terminal_feed_tx(const char *cmd)
{
    if (!cmd) return;
    ring_write3("> ", 2, cmd, strlen(cmd), "\n", 1);
}

/* ---- Send command ---- */

static esp_err_t terminal_send_cmd(const char *cmd)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED)
        return ESP_ERR_INVALID_STATE;

    /* Echo to ring buffer */
    terminal_feed_tx(cmd);

    /* Queue via printer_comm */
    printer_cmd_t pcmd = { .type = PCMD_RAW };
    strncpy(pcmd.gcode, cmd, sizeof(pcmd.gcode) - 1);
    pcmd.gcode[sizeof(pcmd.gcode) - 1] = '\0';
    return printer_comm_send_cmd(&pcmd);
}

/* ---- HTTP handlers ---- */

static esp_err_t terminal_page_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "terminal");
}

static esp_err_t terminal_send_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    /* Parse cmd= field */
    const char *cmd_start = strstr(buf, "cmd=");
    if (!cmd_start) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing cmd");
        return ESP_FAIL;
    }
    cmd_start += 4;

    char cmd[96];
    url_decode_field(cmd_start, cmd, sizeof(cmd));

    if (cmd[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty command");
        return ESP_FAIL;
    }

    esp_err_t err = terminal_send_cmd(cmd);
    if (err == ESP_ERR_INVALID_STATE) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "Printing in progress");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Send failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/** GET /terminal/poll?seq=N → {"seq":M,"lost":L,"more":B,"boot":K,"data":"..."}
 *  Ender Smart (lote 10):
 *  - seq 0 (a page that just opened), or a seq larger than ours (the page still has the count
 *    from before a reboot of the ESP32): the last TERM_POLL_MAX bytes, from the start of a line;
 *  - a seq older than the ring keeps: "lost" says how many bytes the page missed (the cut line
 *    included) and the answer starts at a line;
 *  - at most TERM_POLL_MAX bytes per answer, cut after a newline: "more" asks the page to
 *    poll again right away (a cut in the middle of a UTF-8 character broke the JSON);
 *  - "boot" changes at every boot: a page that sees a new value asks again from seq 0;
 *  - the next poll sends back "seq" as it came. */
static esp_err_t terminal_poll_handler(httpd_req_t *req)
{
    char qbuf[96] = {0};
    uint64_t client_seq = 0;
    esp_err_t qerr = httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
    if (qerr == ESP_OK || qerr == ESP_ERR_HTTPD_RESULT_TRUNC) {   /* a long query keeps seq= */
        char val[24];
        if (httpd_query_key_value(qbuf, "seq", val, sizeof(val)) == ESP_OK)
            client_seq = strtoull(val, NULL, 10);
    }

    char *raw = NULL;
    size_t n = 0, skip = 0;
    uint64_t start = 0, head = 0, lost = 0;
    bool align = false;
    for (int attempt = 0; s_ready && attempt < 3; attempt++) {
        head = ring_seq();
        uint64_t oldest = head > TERM_SAFE_SPAN ? head - TERM_SAFE_SPAN : 0;
        lost = 0;
        align = false;
        if (client_seq == 0 || client_seq > head) {
            start = head > TERM_POLL_MAX ? head - TERM_POLL_MAX : 0;
            align = start > 0;
        } else if (client_seq < oldest) {
            lost = oldest - client_seq;
            start = oldest;
            align = true;
        } else {
            start = client_seq;
        }
        n = (size_t)(head - start);
        if (n > TERM_POLL_MAX) n = TERM_POLL_MAX;
        if (n == 0) break;
        if (!raw) {
            raw = malloc(TERM_POLL_MAX);
            if (!raw) {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
                return ESP_FAIL;
            }
        }
        ring_copy(start, raw, n);
        /* a writer overwrites seq x - TERM_RING_SIZE when it writes seq x */
        if (ring_seq() - start <= TERM_RING_SIZE) break;
        n = 0;   /* lapped during the copy (16 KB written in a few ms): copy again */
    }
    if (!s_ready) head = 0;
    if (n == 0) lost = 0;   /* nothing was sent: the next poll counts the loss once */
    if (n == TERM_POLL_MAX && start + n < head) {
        /* cut by the size limit: end after the last newline, so no line and no UTF-8
         * character is split between two answers (kept as is if 16 KB have no newline) */
        size_t k = n;
        while (k > 0 && raw[k - 1] != '\n') k--;
        if (k > 0) n = k;
    }
    if (s_boot_id == 0) {
        s_boot_id = esp_random();
        if (s_boot_id == 0) s_boot_id = 1;
    }

    if (align && n > 0) {
        const char *nl = memchr(raw, '\n', n);
        if (nl) skip = (size_t)(nl - raw) + 1;
        if (lost) lost += skip;   /* the cut line was missed too */
    }
    uint64_t resp_seq = n > 0 ? start + n : (client_seq > head || client_seq == 0 ? head : client_seq);
    bool more = resp_seq < head;

    /* {"seq":..,"lost":..,"more":false,"boot":..,"data":"..."}: every byte escapes to 2 at most */
    size_t json_cap = 112 + (n - skip) * 2;
    char *json = malloc(json_cap);
    if (!json) {
        free(raw);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    int pos = snprintf(json, json_cap, "{\"seq\":%llu,\"lost\":%llu,\"more\":%s,\"boot\":%lu,\"data\":\"",
                       (unsigned long long)resp_seq, (unsigned long long)lost, more ? "true" : "false",
                       (unsigned long)s_boot_id);
    for (size_t i = skip; i < n && pos < (int)json_cap - 8; i++) {
        char c = raw[i];
        if (c == '"') { json[pos++] = '\\'; json[pos++] = '"'; }
        else if (c == '\\') { json[pos++] = '\\'; json[pos++] = '\\'; }
        else if (c == '\n') { json[pos++] = '\\'; json[pos++] = 'n'; }
        else if (c == '\r') { json[pos++] = '\\'; json[pos++] = 'r'; }
        else if (c == '\t') { json[pos++] = '\\'; json[pos++] = 't'; }
        else if ((unsigned char)c < 0x20) { /* skip other control chars */ }
        else { json[pos++] = c; }
    }
    json[pos++] = '"';
    json[pos++] = '}';
    json[pos] = '\0';
    free(raw);

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json, pos);
    free(json);
    return ret;
}

static esp_err_t terminal_polling_handler(httpd_req_t *req)
{
    char buf[32];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char val[4] = {0};
    if (httpd_query_key_value(buf, "suppress", val, sizeof(val)) == ESP_OK) {
        bool suppress = (val[0] == '1');
        printer_comm_set_polling_suppressed(suppress);
        ESP_LOGI(TAG, "Polling %s", suppress ? "suppressed" : "resumed");
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ---- Registration ---- */

esp_err_t terminal_register_httpd(httpd_handle_t server)
{
    httpd_uri_t page = {
        .uri = "/terminal", .method = HTTP_GET, .handler = terminal_page_handler,
    };
    HTTPD_REGISTER(server, &page);

    httpd_uri_t send_uri = {
        .uri = "/terminal/send", .method = HTTP_POST, .handler = terminal_send_handler,
    };
    HTTPD_REGISTER(server, &send_uri);

    httpd_uri_t poll_uri = {
        .uri = "/terminal/poll", .method = HTTP_GET, .handler = terminal_poll_handler,
    };
    HTTPD_REGISTER(server, &poll_uri);

    httpd_uri_t polling_uri = {
        .uri = "/terminal/polling", .method = HTTP_POST, .handler = terminal_polling_handler,
    };
    HTTPD_REGISTER(server, &polling_uri);

    ESP_LOGI(TAG, "Terminal endpoints registered");
    return ESP_OK;
}
