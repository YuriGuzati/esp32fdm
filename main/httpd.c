#include "httpd.h"

#include <string.h>
#include <stdlib.h>

#include "camera.h"
#include "wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "sdcard.h"
#include "sdcard_httpd.h"
#include "printer_comm.h"
#include "usb_serial.h"
#include "layout.h"

#include "obico_client.h"
#include "terminal.h"
#include "bedlevel.h"
#include "job_history.h"
#include "url_util.h"
#include "logbuf.h"
#include "ota.h"
#include "version.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"

static const char *TAG = "httpd";

/* Forward declaration — used by api_command_handler and others */
static bool parse_form_field(const char *body, const char *name,
                             char *out, size_t out_size);

/* Baby-step Z offset — seeded from printer (M290 query), updated locally on adjustments.
 * Re-synced from printer_state_t.probe_z_offset when the printer is queried. */
static float s_baby_z = 0.0f;
static bool  s_baby_z_seeded = false;  /* true once we got a real value from the printer */

#define STREAM_TARGET_FPS 5
#define STREAM_FRAME_INTERVAL_US (1000000 / STREAM_TARGET_FPS)

static const char *STREAM_BOUNDARY = "esp32fdm_frame";

/* Ender Smart (F15-22): each MJPEG client holds a socket and a task for as long as
 * the page stays open; cap them so the dashboard and the API always get through. */
#define MAX_STREAM_CLIENTS 3
static int s_stream_clients;
static httpd_handle_t s_main_server;

/* ---- /capture handler (async) ---- */

static void capture_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;

    camera_frame_t *frame = camera_get_frame();
    if (!frame) {
        httpd_resp_send_500(req);
    } else {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition",
                           "inline; filename=capture.jpg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, (const char *)frame->buf, frame->len);
        camera_release_frame(frame);
    }

    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

/* Inline SVG favicon: 3D printer hotend silhouette */
static const char FAVICON_SVG[] =
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'>"
    "<rect x='4' y='2' width='24' height='6' rx='1' fill='#555'/>"
    "<rect x='6' y='8' width='20' height='4' fill='#777'/>"
    "<rect x='13' y='12' width='6' height='8' fill='#555'/>"
    "<polygon points='12,20 20,20 18,26 14,26' fill='#d4a017'/>"
    "<circle cx='16' cy='28' r='2' fill='#e74c3c'/>"
    "</svg>";

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, FAVICON_SVG, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t camera_unavailable(httpd_req_t *req)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "Camera not available (init failed at boot, see /logs)");
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    if (!camera_is_available()) {
        return camera_unavailable(req);
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(capture_task, "capture", 4096, async_req,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- /stream handler (MJPEG multipart) ---- */

/* Stream task runs in background, freeing the httpd thread */
static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    int fd = httpd_req_to_sockfd(req);

    ESP_LOGI(TAG, "Stream client connected");

    char part_header[128];
    int64_t last_frame_time = 0;

    while (true) {
        /* Frame rate limiter */
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_frame_time;
        if (elapsed < STREAM_FRAME_INTERVAL_US) {
            vTaskDelay(pdMS_TO_TICKS((STREAM_FRAME_INTERVAL_US - elapsed) / 1000));
        }

        camera_frame_t *frame = camera_get_frame();
        last_frame_time = esp_timer_get_time();
        if (!frame) {
            ESP_LOGW(TAG, "Stream: no frame available, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int hdr_len = snprintf(part_header, sizeof(part_header),
                               "--%s\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n"
                               "\r\n",
                               STREAM_BOUNDARY, (unsigned)frame->len);

        /* Send part header + JPEG + trailing CRLF via raw socket */
        if (send(fd, part_header, hdr_len, 0) <= 0 ||
            send(fd, (const char *)frame->buf, frame->len, 0) <= 0 ||
            send(fd, "\r\n", 2, 0) <= 0) {
            camera_release_frame(frame);
            break;
        }
        camera_release_frame(frame);
    }

    ESP_LOGI(TAG, "Stream client disconnected");
    httpd_req_async_handler_complete(req);
    __atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    /* Ender Smart (F15-04): without a camera the stream task would loop forever on
     * "no frame available", holding a socket and a task per client. */
    if (!camera_is_available()) {
        return camera_unavailable(req);
    }

    if (__atomic_add_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL) > MAX_STREAM_CLIENTS) {
        __atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Too many video clients (max 3). Close another camera page.");
    }

    int fd = httpd_req_to_sockfd(req);

    /* TCP_NODELAY for low-latency frame delivery */
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    /* Send HTTP response header before handing off to background task */
    const char *resp_hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=esp32fdm_frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "X-Framerate: 5\r\n"
        "\r\n";
    if (httpd_send(req, resp_hdr, strlen(resp_hdr)) <= 0) {
        __atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL);
        return ESP_FAIL;
    }

    /* Clone the request for async use and spawn a background task */
    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start async stream handler");
        __atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL);
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(stream_task, "stream", 4096, async_req,
                                 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create stream task");
        httpd_req_async_handler_complete(async_req);
        __atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_ACQ_REL);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- GET /api/status — JSON for status bar + dashboard ---- */

static esp_err_t api_status_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);

    const char *state_str;
    switch (st.opstate) {
        case PRINTER_PRINTING: state_str = "printing"; break;
        case PRINTER_PAUSED:   state_str = "paused"; break;
        case PRINTER_ERROR:    state_str = "error"; break;
        case PRINTER_OPERATIONAL: state_str = "operational"; break;
        default: state_str = "disconnected"; break;
    }

    bool usb = usb_serial_is_connected();
    int32_t layer = st.current_layer;
    int32_t total_layers = st.total_layers;

    /* Sync baby Z from printer when available (probe_z_offset is 0 until M290 response) */
    if (!s_baby_z_seeded && st.probe_z_offset != 0.0f) {
        s_baby_z = st.probe_z_offset;
        s_baby_z_seeded = true;
    }

    char json[896];
    snprintf(json, sizeof(json),
        "{\"state\":\"%s\",\"backend\":\"%s\",\"hotend\":[%.1f,%.1f],\"bed\":[%.1f,%.1f],"
        "\"progress\":%.1f,\"filename\":\"%s\",\"elapsed\":%ld,\"remaining\":%ld,"
        "\"layer\":\"%ld/%ld\",\"z\":%.1f,\"baby_z\":%.2f,\"usb\":%s,\"error\":\"%s\","
        "\"notice\":\"%s\"}",
        state_str, printer_comm_backend_name(),
        st.hotend_actual, st.hotend_target,
        st.bed_actual, st.bed_target,
        st.progress_pct >= 0 ? st.progress_pct : 0.0f,
        st.filename,
        (long)(st.print_time_s >= 0 ? st.print_time_s : 0),
        (long)(st.print_time_left_s >= 0 ? st.print_time_left_s : 0),
        (long)(layer >= 0 ? layer : 0),
        (long)(total_layers >= 0 ? total_layers : 0),
        st.z, s_baby_z,
        usb ? "true" : "false",
        st.opstate == PRINTER_ERROR ? st.error_msg : "",
        st.notice);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

/* ---- GET /api/stats — per-core CPU usage ---- */

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
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

/* Accumulated idle ticks from previous call, per core */
static uint32_t s_prev_idle[2];
static uint32_t s_prev_total;
static bool     s_stats_initialized;

static esp_err_t api_stats_handler(httpd_req_t *req)
{
#if defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = malloc(num_tasks * sizeof(TaskStatus_t));
    if (!task_array) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    uint32_t total_runtime;
    UBaseType_t count = uxTaskGetSystemState(task_array, num_tasks, &total_runtime);

    /* Sum idle task runtime per core (IDLE0, IDLE1) */
    uint32_t idle_now[2] = {0, 0};
    for (UBaseType_t i = 0; i < count; i++) {
        const char *name = task_array[i].pcTaskName;
        if (strcmp(name, "IDLE0") == 0 || strcmp(name, "IDLE") == 0)
            idle_now[0] = task_array[i].ulRunTimeCounter;
        else if (strcmp(name, "IDLE1") == 0)
            idle_now[1] = task_array[i].ulRunTimeCounter;
    }

    int cpu0_pct = 0, cpu1_pct = 0;
    if (s_stats_initialized && total_runtime > s_prev_total) {
        uint32_t dt = total_runtime - s_prev_total;
        /* total_runtime is wall-clock timer ticks (same for both cores) */
        if (dt > 0) {
            uint32_t idle0_delta = idle_now[0] - s_prev_idle[0];
            uint32_t idle1_delta = idle_now[1] - s_prev_idle[1];
            cpu0_pct = 100 - (int)(idle0_delta * 100 / dt);
            cpu1_pct = 100 - (int)(idle1_delta * 100 / dt);
            if (cpu0_pct < 0) cpu0_pct = 0;
            if (cpu1_pct < 0) cpu1_pct = 0;
            if (cpu0_pct > 100) cpu0_pct = 100;
            if (cpu1_pct > 100) cpu1_pct = 100;
        }
    }
    s_prev_idle[0] = idle_now[0];
    s_prev_idle[1] = idle_now[1];
    s_prev_total = total_runtime;
    s_stats_initialized = true;

    free(task_array);

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

    /* Ender Smart (F15-17): diagnose restarts and Wi-Fi from the network, without the UART. */
    int8_t rssi = 0;
    bool sta = wifi_get_rssi(&rssi);
    /* F15-22: open HTTP sessions and video clients, to see socket pressure */
    size_t n_clients = 16;
    int client_fds[16];
    if (!s_main_server || httpd_get_client_list(s_main_server, &n_clients, client_fds) != ESP_OK)
        n_clients = 0;
    char json[448];
    snprintf(json, sizeof(json),
        "{\"cpu0\":%d,\"cpu1\":%d,\"heap\":%lu,\"heap_min\":%lu,"
        "\"uptime_s\":%lld,\"reset_reason\":\"%s\",\"rssi\":%d,"
        "\"wifi\":\"%s\",\"emergency_ap\":%s,\"wifi_reconnects\":%lu,\"camera\":%s,"
        "\"http_clients\":%u,\"stream_clients\":%d,\"rx_dropped\":%lu,\"temp_mode\":\"%s\"}",
        cpu0_pct, cpu1_pct, (unsigned long)free_heap, (unsigned long)min_heap,
        (long long)(esp_timer_get_time() / 1000000), reset_reason_str(esp_reset_reason()),
        sta ? rssi : 0, sta ? "connected" : "disconnected",
        wifi_emergency_ap_active() ? "true" : "false",
        (unsigned long)wifi_get_reconnect_count(),
        camera_is_available() ? "true" : "false",
        (unsigned)n_clients, __atomic_load_n(&s_stream_clients, __ATOMIC_ACQUIRE),
        (unsigned long)printer_comm_rx_dropped(), printer_comm_autotemp_active() ? "auto" : "poll");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
#else
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"stats not enabled\"}", HTTPD_RESP_USE_STRLEN);
#endif
}

/* ---- Ender Smart (Fase 4): new interface at /app and its settings JSON ---- */

extern const uint8_t app_html_start[] asm("_binary_app_html_start");
extern const uint8_t app_html_end[]   asm("_binary_app_html_end");

static esp_err_t app_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)app_html_start, app_html_end - app_html_start);
}

/* Ender Smart (F4-07): the old pages were removed (decision of the user, 2026-11-12, after
 * the new UI was confirmed in real prints on 2026-11-13). Their addresses open the matching
 * screen of the new UI; the data and action routes are unchanged. */
esp_err_t httpd_redirect_app(httpd_req_t *req, const char *screen)
{
    char loc[48];
    snprintf(loc, sizeof(loc), "/app#%s", screen);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", loc);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, NULL, 0);
}

/* Ender Smart (Fase 4): icon and manifest for "Add to home screen" */
extern const uint8_t icon192_png_start[] asm("_binary_icon_192_png_start");
extern const uint8_t icon192_png_end[]   asm("_binary_icon_192_png_end");
extern const uint8_t touch_png_start[]   asm("_binary_apple_touch_icon_png_start");
extern const uint8_t touch_png_end[]     asm("_binary_apple_touch_icon_png_end");

static esp_err_t send_png(httpd_req_t *req, const uint8_t *start, const uint8_t *end)
{
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, (const char *)start, end - start);
}
static esp_err_t icon192_handler(httpd_req_t *req) { return send_png(req, icon192_png_start, icon192_png_end); }
static esp_err_t touch_icon_handler(httpd_req_t *req) { return send_png(req, touch_png_start, touch_png_end); }

static esp_err_t manifest_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/manifest+json");
    return httpd_resp_sendstr(req,
        "{\"name\":\"Ender Smart\",\"short_name\":\"Ender Smart\",\"start_url\":\"/app\",\"scope\":\"/\","
        "\"display\":\"standalone\",\"background_color\":\"#0F1115\",\"theme_color\":\"#0F1115\","
        "\"icons\":[{\"src\":\"/icon-192.png\",\"sizes\":\"192x192\",\"type\":\"image/png\",\"purpose\":\"any maskable\"},"
        "{\"src\":\"/apple-touch-icon.png\",\"sizes\":\"180x180\",\"type\":\"image/png\"}]}");
}

static void json_str(char *out, size_t out_size, const char *in)
{
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)(in ? in : ""); *p && o + 7 < out_size; p++) {
        if (*p == '"' || *p == '\\') { out[o++] = '\\'; out[o++] = (char)*p; }
        else if (*p < 0x20) o += snprintf(out + o, out_size - o, "\\u%04x", *p);
        else out[o++] = (char)*p;
    }
    out[o] = '\0';
}

/* GET /api/settings — current values for the new interface's forms (read-only) */
static esp_err_t api_settings_handler(httpd_req_t *req)
{
    char ssid_raw[33] = {0};
    wifi_get_ssid(ssid_raw, sizeof(ssid_raw));
    char ssid[80], host[80], jhost[140], obico_url[200];
    json_str(ssid, sizeof(ssid), ssid_raw);
    json_str(host, sizeof(host), wifi_get_hostname());
    json_str(jhost, sizeof(jhost), obico_get_janus_host());
    json_str(obico_url, sizeof(obico_url), obico_get_server_url());

    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    int bright = 0, contrast = 0, sat = 0, ae = 0;
    camera_get_image_params(&bright, &contrast, &sat, &ae);

    char printer[512];   /* lote 6: + pause_retract/pause_temp */
    printer_comm_settings_json(printer, sizeof(printer));

    char *json = malloc(1400);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    snprintf(json, 1400,
        "{\"hostname\":\"%s\",\"ssid\":\"%s\",\"ip\":\"%s\","
        "\"fw\":\"%s\",\"fw_label\":\"%s\",\"partition\":\"%s\",\"idf\":\"%s\","
        "\"camera\":{\"available\":%s,\"rotate180\":%s,\"xclk\":%d,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,\"ae_level\":%d},"
        "\"janus_host\":\"%s\",\"janus_port\":%u,"
        "\"obico\":{\"linked\":%s,\"server_url\":\"%s\"},"
        "\"rfc2217\":%s,%s}",
        host, ssid, wifi_get_ip_str(),
        app ? app->version : "?", FW_VERSION, running ? running->label : "?",
        app ? app->idf_ver : "?",
        camera_is_available() ? "true" : "false", camera_get_rotate180() ? "true" : "false",
        camera_get_xclk_mhz(), bright, contrast, sat, ae,
        jhost, (unsigned)obico_get_janus_port(),
        obico_is_linked() ? "true" : "false", obico_url,
#if CONFIG_RFC2217_ENABLE
        "true",
#else
        "false",
#endif
        printer);

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    free(json);
    return ret;
}

/* ---- Ender Smart (F2-02): GET /api/printer_info and POST /api/printer_info/refresh ---- */

static esp_err_t api_printer_info_handler(httpd_req_t *req)
{
    char *json = malloc(2560);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    printer_comm_info_json(json, 2560);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    free(json);
    return ret;
}

static esp_err_t api_printer_info_refresh_handler(httpd_req_t *req)
{
    /* M115 only reads the firmware identification: no motion, no heating */
    esp_err_t err = printer_comm_request_info();
    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK)
        return httpd_resp_sendstr(req, "{\"ok\":true}");
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, err == ESP_ERR_NOT_SUPPORTED ? "{\"ok\":false,\"err\":\"Only for Marlin\"}"
                                   : "{\"ok\":false,\"err\":\"Printer not connected or a print is running\"}");
}

/* ---- GET /api/temp_history — temperature history for chart ---- */

static esp_err_t api_temp_history_handler(httpd_req_t *req)
{
    /* Allocate on heap — samples are 24 bytes each, 900 * 24 = ~21KB */
    temp_sample_t *samples = malloc(TEMP_HISTORY_MAX * sizeof(temp_sample_t));
    if (!samples) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int count = printer_comm_get_temp_history(samples, TEMP_HISTORY_MAX);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    /* Use chunked response to avoid a huge JSON buffer */
    char chunk[128];
    httpd_resp_sendstr_chunk(req, "{\"samples\":[");

    for (int i = 0; i < count; i++) {
        temp_sample_t *s = &samples[i];
        int64_t ts_sec = s->timestamp_us / 1000000;
        int len = snprintf(chunk, sizeof(chunk),
            "%s[%lld,%.1f,%.0f,%.1f,%.0f]",
            i > 0 ? "," : "",
            (long long)ts_sec,
            s->hotend_actual, s->hotend_target,
            s->bed_actual, s->bed_target);
        httpd_resp_send_chunk(req, chunk, len);
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_send_chunk(req, NULL, 0);  /* finish chunked response */

    free(samples);
    return ESP_OK;
}

/* ---- POST /api/command — send raw GCode ---- */

static esp_err_t api_command_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char cmd[96] = {0};
    if (!parse_form_field(buf, "cmd", cmd, sizeof(cmd)) || cmd[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "cmd required");
        return ESP_FAIL;
    }

    /* Ender Smart (F15-10 / C6): the server decides, not the page's JavaScript */
    char reason[96];
    if (!printer_comm_cmd_allowed(cmd, reason, sizeof(reason))) {
        ESP_LOGW(TAG, "/api/command refused: %s (%s)", cmd, reason);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, reason);
    }

    /* Track baby-step Z offset accumulator */
    const char *m290 = strcasestr(cmd, "M290");
    if (m290) {
        const char *zp = strchr(m290, 'Z');
        if (!zp) zp = strchr(m290, 'z');
        if (zp) s_baby_z += strtof(zp + 1, NULL);
    }

    printer_cmd_t pcmd = { .type = PCMD_RAW };
    strncpy(pcmd.gcode, cmd, sizeof(pcmd.gcode) - 1);
    printer_comm_send_cmd(&pcmd);

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", 2);
}

/* ---- / root handler — dashboard ---- */

static esp_err_t root_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old dashboard was removed; "/" opens the new UI */
    return app_handler(req);
}

/* ---- /settings handler ---- */

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "config");
}

/* ---- /camera handler ---- */

static esp_err_t utils_page_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "sistema");
}

static esp_err_t camera_page_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "camera");
}

static esp_err_t camera_settings_post_handler(httpd_req_t *req)
{
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    int bright = 0, contrast = 0, sat = 0, ae = 0;
    char val[8];
    if (parse_form_field(buf, "brightness", val, sizeof(val))) bright = atoi(val);
    if (parse_form_field(buf, "contrast", val, sizeof(val)))   contrast = atoi(val);
    if (parse_form_field(buf, "saturation", val, sizeof(val))) sat = atoi(val);
    if (parse_form_field(buf, "ae_level", val, sizeof(val)))   ae = atoi(val);

    camera_set_image_params(bright, contrast, sat, ae);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t video_config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len < 0) recv_len = 0;
    buf[recv_len] = '\0';

    /* Camera rotation */
    bool rotate = (strstr(buf, "rotate180=1") != NULL);
    camera_set_rotate180(rotate);

    /* Camera XCLK */
    char xclk_str[4] = {0};
    if (parse_form_field(buf, "xclk", xclk_str, sizeof(xclk_str))) {
        camera_set_xclk_mhz(atoi(xclk_str));
    }

    /* Janus proxy host/port */
    char janus_host[64] = {0};
    char port_str[8] = {0};
    parse_form_field(buf, "janus_host", janus_host, sizeof(janus_host));
    parse_form_field(buf, "janus_port", port_str, sizeof(port_str));

    uint16_t janus_port = 17800;
    if (port_str[0]) {
        int p = atoi(port_str);
        if (p > 0 && p <= 65535) janus_port = (uint16_t)p;
    }
    obico_set_janus_proxy(janus_host, janus_port);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- /device/config handler ---- */

static esp_err_t device_config_post_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len < 0) recv_len = 0;
    buf[recv_len] = '\0';

    char hostname[32] = {0};
    parse_form_field(buf, "hostname", hostname, sizeof(hostname));

    if (hostname[0] && printer_comm_job_active()) {
        /* Ender Smart (F15-13): saving the name reboots the ESP32 */
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Blocked while a print is running or paused (this would reboot)");
    }

    if (hostname[0]) {
        wifi_set_hostname(hostname);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_sendstr(req, "Device name saved. Rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- /wifi/reset handler ---- */

static esp_err_t wifi_reset_handler(httpd_req_t *req)
{
    if (printer_comm_job_active()) {
        /* Ender Smart (F15-13): erasing Wi-Fi reboots into setup mode */
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Blocked while a print is running or paused");
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "WiFi credentials erased. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    wifi_reset_credentials();
    return ESP_OK;  /* unreachable */
}

/* ---- Captive portal helpers ---- */

static bool parse_form_field(const char *body, const char *name,
                             char *out, size_t out_size)
{
    char key[36];
    snprintf(key, sizeof(key), "%s=", name);
    const char *start = strstr(body, key);
    if (!start) return false;
    start += strlen(key);

    url_decode_field(start, out, out_size);
    return true;
}

static const char CAPTIVE_HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>ESP32 FDM WiFi Setup</title>"
    "<style>"
    "body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px}"
    "input{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:10px;font-size:18px;background:#2196F3;color:white;"
    "border:none;border-radius:4px;cursor:pointer}"
    "label{font-weight:bold}"
    "</style></head><body>"
    "<h2>WiFi Setup</h2>"
    "<form method=\"POST\" action=\"/\">"
    "<label>SSID</label>"
    "<input type=\"text\" name=\"ssid\" required maxlength=\"32\" autocomplete=\"off\">"
    "<label>Password</label>"
    "<input type=\"password\" name=\"password\" maxlength=\"64\">"
    "<button type=\"submit\">Connect</button>"
    "</form></body></html>";

static esp_err_t captive_root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, CAPTIVE_HTML, sizeof(CAPTIVE_HTML) - 1);
}

static esp_err_t captive_root_post_handler(httpd_req_t *req)
{
    char buf[200];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char ssid[33] = {0};
    char pass[65] = {0};

    if (!parse_form_field(buf, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }
    parse_form_field(buf, "password", pass, sizeof(pass));

    /* Save to NVS */
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("wifi", NVS_READWRITE, &nvs));
    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "password", pass);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "WiFi credentials saved for \"%s\" — rebooting", ssid);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!DOCTYPE html><html><body style=\"font-family:sans-serif;text-align:center;margin-top:60px\">"
        "<h2>Saved!</h2><p>Rebooting and connecting to WiFi...</p>"
        "</body></html>");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- Public API ---- */

esp_err_t httpd_start_captive_portal(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.max_uri_handlers = 4;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start captive portal: 0x%x", err);
        return err;
    }

    /* Order matters: exact matches first, wildcard last */
    httpd_uri_t root_get = {
        .uri = "/", .method = HTTP_GET, .handler = captive_root_get_handler,
    };
    HTTPD_REGISTER(server, &root_get);

    httpd_uri_t root_post = {
        .uri = "/", .method = HTTP_POST, .handler = captive_root_post_handler,
    };
    HTTPD_REGISTER(server, &root_post);

    httpd_uri_t wildcard = {
        .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler,
    };
    HTTPD_REGISTER(server, &wildcard);

    ESP_LOGI(TAG, "Captive portal started on port 80");
    return ESP_OK;
}

esp_err_t httpd_start_server(void)
{
    /* HTTP server on port 80 (Core 0) */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 10240;  /* TLS (mbedTLS) in /obico/link handler needs ~8KB */
    config.max_uri_handlers = 72;   /* Ender Smart: 60 in use after lote 5 */
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: 0x%x", err);
        return err;
    }
    s_main_server = server;

    httpd_uri_t app_uri = {
        .uri      = "/app",
        .method   = HTTP_GET,
        .handler  = app_handler,
    };
    HTTPD_REGISTER(server, &app_uri);

    httpd_uri_t settings_api_uri = {
        .uri      = "/api/settings",
        .method   = HTTP_GET,
        .handler  = api_settings_handler,
    };
    HTTPD_REGISTER(server, &settings_api_uri);

    httpd_uri_t manifest_uri = { .uri = "/manifest.webmanifest", .method = HTTP_GET, .handler = manifest_handler };
    HTTPD_REGISTER(server, &manifest_uri);
    httpd_uri_t icon192_uri = { .uri = "/icon-192.png", .method = HTTP_GET, .handler = icon192_handler };
    HTTPD_REGISTER(server, &icon192_uri);
    httpd_uri_t touch_uri = { .uri = "/apple-touch-icon.png", .method = HTTP_GET, .handler = touch_icon_handler };
    HTTPD_REGISTER(server, &touch_uri);
    job_history_register_httpd(server);

    httpd_uri_t printer_info_uri = { .uri = "/api/printer_info", .method = HTTP_GET, .handler = api_printer_info_handler };
    HTTPD_REGISTER(server, &printer_info_uri);
    httpd_uri_t printer_info_refresh_uri = { .uri = "/api/printer_info/refresh", .method = HTTP_POST, .handler = api_printer_info_refresh_handler };
    HTTPD_REGISTER(server, &printer_info_refresh_uri);

    httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
    };
    HTTPD_REGISTER(server, &root_uri);

    httpd_uri_t favicon_uri = {
        .uri      = "/favicon.svg",
        .method   = HTTP_GET,
        .handler  = favicon_handler,
    };
    HTTPD_REGISTER(server, &favicon_uri);

    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
    };
    HTTPD_REGISTER(server, &capture_uri);

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
    };
    HTTPD_REGISTER(server, &stream_uri);

    httpd_uri_t api_status = {
        .uri      = "/api/status",
        .method   = HTTP_GET,
        .handler  = api_status_handler,
    };
    HTTPD_REGISTER(server, &api_status);

    httpd_uri_t api_stats = {
        .uri      = "/api/stats",
        .method   = HTTP_GET,
        .handler  = api_stats_handler,
    };
    HTTPD_REGISTER(server, &api_stats);

    httpd_uri_t api_temp_history = {
        .uri      = "/api/temp_history",
        .method   = HTTP_GET,
        .handler  = api_temp_history_handler,
    };
    HTTPD_REGISTER(server, &api_temp_history);

    httpd_uri_t api_command = {
        .uri      = "/api/command",
        .method   = HTTP_POST,
        .handler  = api_command_handler,
    };
    HTTPD_REGISTER(server, &api_command);

    httpd_uri_t camera_page = {
        .uri      = "/camera",
        .method   = HTTP_GET,
        .handler  = camera_page_handler,
    };
    HTTPD_REGISTER(server, &camera_page);

    httpd_uri_t settings_uri = {
        .uri      = "/settings",
        .method   = HTTP_GET,
        .handler  = settings_get_handler,
    };
    HTTPD_REGISTER(server, &settings_uri);

    httpd_uri_t video_cfg_post = {
        .uri      = "/video/config",
        .method   = HTTP_POST,
        .handler  = video_config_post_handler,
    };
    HTTPD_REGISTER(server, &video_cfg_post);

    httpd_uri_t cam_settings_post = {
        .uri      = "/camera/settings",
        .method   = HTTP_POST,
        .handler  = camera_settings_post_handler,
    };
    HTTPD_REGISTER(server, &cam_settings_post);

    httpd_uri_t device_cfg_post = {
        .uri      = "/device/config",
        .method   = HTTP_POST,
        .handler  = device_config_post_handler,
    };
    HTTPD_REGISTER(server, &device_cfg_post);

    httpd_uri_t wifi_reset = {
        .uri      = "/wifi/reset",
        .method   = HTTP_POST,
        .handler  = wifi_reset_handler,
    };
    HTTPD_REGISTER(server, &wifi_reset);

    httpd_uri_t utils_uri = {
        .uri      = "/utils",
        .method   = HTTP_GET,
        .handler  = utils_page_handler,
    };
    HTTPD_REGISTER(server, &utils_uri);

    sdcard_httpd_register(server);

    terminal_register_httpd(server);
    if (printer_comm_get_backend() != PRINTER_BACKEND_KLIPPER) {
        bedlevel_register_httpd(server);
    }
    logbuf_register_httpd(server);
    obico_register_httpd(server);
    printer_config_register_httpd(server);
    ota_register_httpd(server);

    ESP_LOGI(TAG, "HTTP server started on port 80 (stream at /stream)");

    return ESP_OK;
}
