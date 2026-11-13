#include "terminal.h"
#include "httpd.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "printer_comm.h"
#include "layout.h"
#include "url_util.h"

static const char *TAG = "terminal";

/* ---- Ring buffer ---- */

#define TERM_RING_SIZE 4096

static char *s_ring;               /* allocated in PSRAM */
static int  s_ring_head = 0;       /* write position in ring */
static uint32_t s_seq = 0;         /* monotonic total bytes written */
static SemaphoreHandle_t s_mutex;

void terminal_init(void)
{
    s_ring = heap_caps_calloc(1, TERM_RING_SIZE, MALLOC_CAP_SPIRAM);
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Terminal initialized (ring %d bytes, PSRAM)", TERM_RING_SIZE);
}

static void ring_write(const char *data, size_t len)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = data[i];
        s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    }
    s_seq += len;
    xSemaphoreGive(s_mutex);
}

void terminal_feed_rx(const uint8_t *data, size_t len)
{
    if (len == 0 || !s_mutex) return;
    /* Non-blocking: called from USB Host callback context —
     * blocking here stalls ALL USB transfers until the mutex
     * is released.  If the mutex is held (HTTP reading terminal),
     * just drop these bytes rather than stalling USB. */
    if (xSemaphoreTake(s_mutex, 0) != pdTRUE) return;
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = (char)data[i];
        s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    }
    s_seq += len;
    xSemaphoreGive(s_mutex);
}

void terminal_feed_tx(const char *cmd)
{
    if (!cmd || !s_mutex) return;
    /* Non-blocking — same reason as terminal_feed_rx: the caller
     * (printer_comm task) must not stall on terminal I/O. */
    if (xSemaphoreTake(s_mutex, 0) != pdTRUE) return;
    const char *prefix = "> ";
    for (int i = 0; prefix[i]; i++) {
        s_ring[s_ring_head] = prefix[i];
        s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    }
    size_t len = strlen(cmd);
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = cmd[i];
        s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    }
    s_ring[s_ring_head] = '\n';
    s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    s_seq += 2 + len + 1;
    xSemaphoreGive(s_mutex);
}

/* ---- Send command ---- */

static esp_err_t terminal_send_cmd(const char *cmd)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED)
        return ESP_ERR_INVALID_STATE;

    /* Echo to ring buffer */
    ring_write("> ", 2);
    ring_write(cmd, strlen(cmd));
    ring_write("\n", 1);

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

static esp_err_t terminal_poll_handler(httpd_req_t *req)
{
    /* Parse seq from query string */
    char qbuf[32] = {0};
    uint32_t client_seq = 0;
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qbuf, "seq", val, sizeof(val)) == ESP_OK) {
            client_seq = (uint32_t)strtoul(val, NULL, 10);
        }
    }

    char *json = NULL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uint32_t avail;
    if (client_seq >= s_seq) {
        /* No new data */
        avail = 0;
    } else if (s_seq - client_seq > TERM_RING_SIZE) {
        /* Client fell behind — send everything in buffer */
        avail = TERM_RING_SIZE;
    } else {
        avail = s_seq - client_seq;
    }

    if (avail == 0) {
        xSemaphoreGive(s_mutex);
        /* Empty response */
        char resp[32];
        snprintf(resp, sizeof(resp), "{\"seq\":%lu,\"data\":\"\"}", (unsigned long)s_seq);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, resp, strlen(resp));
    }

    /* Allocate buffer for JSON: {"seq":N,"data":"..."} with escaping headroom */
    size_t json_cap = 32 + avail * 2;  /* worst case: every byte needs escaping */
    json = malloc(json_cap);
    if (!json) {
        xSemaphoreGive(s_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int pos = snprintf(json, json_cap, "{\"seq\":%lu,\"data\":\"", (unsigned long)s_seq);

    /* Read from ring buffer, JSON-escape as we go */
    int start = (s_ring_head - (int)avail + TERM_RING_SIZE) % TERM_RING_SIZE;
    for (uint32_t i = 0; i < avail && pos < (int)json_cap - 8; i++) {
        char c = s_ring[(start + i) % TERM_RING_SIZE];
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

    xSemaphoreGive(s_mutex);

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
