#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"

/**
 * Register a URI handler with error logging.
 * Logs an error if registration fails (e.g. max_uri_handlers exceeded).
 */
#define HTTPD_REGISTER(server, uri_ptr) do { \
    esp_err_t _err = httpd_register_uri_handler(server, uri_ptr); \
    if (_err != ESP_OK) { \
        ESP_LOGE("httpd", "Failed to register %s %s: %s (increase max_uri_handlers?)", \
                 (uri_ptr)->method == HTTP_GET ? "GET" : "POST", \
                 (uri_ptr)->uri, esp_err_to_name(_err)); \
    } \
} while(0)

/* Ender Smart (lote 16, F3-03): the same handler also under its /api/v1 name.
 * The old address keeps working; both run the very same function. */
#define HTTPD_REGISTER_V1(server, uri_ptr, v1_uri) do { \
    httpd_uri_t _v1 = *(uri_ptr); \
    _v1.uri = (v1_uri); \
    HTTPD_REGISTER(server, &_v1); \
} while(0)

/* Ender Smart (lote 16): the JSON objects of GET /api/status and GET /api/stats, also pushed by
 * the WebSocket /api/v1/events. They keep state (baby Z seed, CPU counters), so call them only
 * from the HTTP server task. Return the length written, or -1 if it did not fit.
 * The CPU % is measured since the previous call with the same window. */
#define HTTPD_CPU_REST 0
#define HTTPD_CPU_WS   1
int httpd_status_json(char *json, size_t json_size);
int httpd_stats_json(char *json, size_t json_size, int window);

/**
 * Start the HTTP server on port 80.
 * Registers /stream (MJPEG) and /capture (single JPEG) endpoints.
 */
esp_err_t httpd_start_server(void);

/** Ender Smart (F4-07): 302 to "/app#<screen>" — the old pages now open the new UI. */
esp_err_t httpd_redirect_app(httpd_req_t *req, const char *screen);

/**
 * Start a captive portal HTTP server for WiFi configuration.
 * Only GET / (config form), POST / (save + reboot), and wildcard redirect.
 */
esp_err_t httpd_start_captive_portal(void);
