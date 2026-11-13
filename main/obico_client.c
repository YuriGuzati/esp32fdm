#include "obico_client.h"
#include "httpd.h"

#include "camera.h"
#include "printer_comm.h"
#include "layout.h"
#include "url_util.h"

#include "cJSON.h"
#include "esp_camera.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "obico";

/* ---- NVS Storage ---- */

#define NVS_NAMESPACE "obico"
#define NVS_KEY_TOKEN "auth_token"
#define NVS_KEY_JANUS_HOST "janus_host"
#define NVS_KEY_JANUS_PORT "janus_port"
#define NVS_KEY_SERVER_URL "server_url"
#define TOKEN_MAX_LEN 128
#define SERVER_URL_MAX_LEN 128

static char s_auth_token[TOKEN_MAX_LEN];
static bool s_linked = false;

/* ---- Server URL runtime config ---- */
static char s_server_url[SERVER_URL_MAX_LEN];

/* ---- Janus proxy runtime config ---- */
static char s_janus_host[64];
static uint16_t s_janus_port = 17800;

/* ---- Obico state ---- */

static bool s_should_watch = false;  /* Obico AI wants to watch */
static bool s_viewing = false;       /* User viewing webcam in Obico UI */
static TaskHandle_t s_ws_task_handle = NULL;
static TaskHandle_t s_snap_task_handle = NULL;
static esp_websocket_client_handle_t s_ws_client = NULL;

/* ---- Janus proxy state ---- */

static int s_janus_sock = -1;
static TaskHandle_t s_janus_rx_task_handle = NULL;
static SemaphoreHandle_t s_janus_sock_mutex = NULL;

static void janus_proxy_task(void *arg);
static void janus_send_to_sidecar(const char *json_str, int len);
static void janus_connect(void);
static void janus_disconnect(void);

static bool janus_proxy_configured(void)
{
    return s_janus_host[0] != '\0';
}

/* ---- Forward declarations ---- */

static void obico_ws_task(void *arg);
static void obico_snap_task(void *arg);

/* ---- NVS helpers ---- */

static esp_err_t load_token_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return err;

    size_t len = TOKEN_MAX_LEN;
    err = nvs_get_str(nvs, NVS_KEY_TOKEN, s_auth_token, &len);
    nvs_close(nvs);

    if (err == ESP_OK && len > 1) {
        s_linked = true;
        ESP_LOGI(TAG, "Auth token loaded from NVS");
    }
    return err;
}

static esp_err_t save_token_to_nvs(const char *token)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_str(nvs, NVS_KEY_TOKEN, token);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t clear_token_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_erase_key(nvs, NVS_KEY_TOKEN);
    nvs_commit(nvs);
    nvs_close(nvs);
    return ESP_OK;
}

static void load_janus_config_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        /* NVS not initialized yet — use Kconfig defaults */
#ifdef CONFIG_OBICO_JANUS_PROXY_HOST
        strncpy(s_janus_host, CONFIG_OBICO_JANUS_PROXY_HOST, sizeof(s_janus_host) - 1);
#endif
#ifdef CONFIG_OBICO_JANUS_PROXY_PORT
        s_janus_port = CONFIG_OBICO_JANUS_PROXY_PORT;
#endif
        return;
    }

    size_t len = sizeof(s_janus_host);
    if (nvs_get_str(nvs, NVS_KEY_JANUS_HOST, s_janus_host, &len) != ESP_OK) {
        /* Not in NVS — fall back to Kconfig */
#ifdef CONFIG_OBICO_JANUS_PROXY_HOST
        strncpy(s_janus_host, CONFIG_OBICO_JANUS_PROXY_HOST, sizeof(s_janus_host) - 1);
#endif
    }

    uint16_t port = 0;
    if (nvs_get_u16(nvs, NVS_KEY_JANUS_PORT, &port) == ESP_OK && port > 0) {
        s_janus_port = port;
    } else {
#ifdef CONFIG_OBICO_JANUS_PROXY_PORT
        s_janus_port = CONFIG_OBICO_JANUS_PROXY_PORT;
#endif
    }

    nvs_close(nvs);
}

static void load_server_url_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        strncpy(s_server_url, CONFIG_OBICO_SERVER_URL, sizeof(s_server_url) - 1);
        return;
    }

    size_t len = sizeof(s_server_url);
    if (nvs_get_str(nvs, NVS_KEY_SERVER_URL, s_server_url, &len) != ESP_OK) {
        strncpy(s_server_url, CONFIG_OBICO_SERVER_URL, sizeof(s_server_url) - 1);
    }
    nvs_close(nvs);
}


static double get_unix_timestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

/* ---- Device Linking (REST API) ---- */

esp_err_t obico_link_with_code(const char *code)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/api/v1/octo/verify/?code=%s",
             s_server_url, code);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
        .buffer_size = 2048,       /* Obico cloud sends large response headers */
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");

    /* POST body with agent info */
    const char *body = "{\"agent_name\":\"esp32fdm\",\"agent_version\":\"1.0.0\"}";
    int body_len = strlen(body);

    /* Manual flow: open → write → fetch_headers → read (perform() eats the body) */
    esp_err_t err = esp_http_client_open(client, body_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Link HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int written = esp_http_client_write(client, body, body_len);
    if (written < 0) {
        ESP_LOGE(TAG, "Link HTTP write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "Link response: status=%d, content_len=%d", status, content_len);

    if (status != 200) {
        ESP_LOGE(TAG, "Link failed with HTTP %d", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    /* Read response body */
    char resp_buf[1024];
    int total_read = 0;
    while (total_read < (int)sizeof(resp_buf) - 1) {
        int r = esp_http_client_read(client, resp_buf + total_read,
                                     sizeof(resp_buf) - 1 - total_read);
        if (r <= 0) break;
        total_read += r;
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (total_read <= 0) {
        ESP_LOGE(TAG, "No response body");
        return ESP_FAIL;
    }
    resp_buf[total_read] = '\0';
    ESP_LOGI(TAG, "Link response body (%d bytes): %.128s", total_read, resp_buf);

    /* Parse JSON response for auth token */
    cJSON *root = cJSON_Parse(resp_buf);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse link response");
        return ESP_FAIL;
    }

    const cJSON *token_item = cJSON_GetObjectItem(root, "printer");
    if (token_item) {
        /* Obico returns { "printer": { "auth_token": "xxx" } } */
        const cJSON *auth = cJSON_GetObjectItem(token_item, "auth_token");
        if (auth && cJSON_IsString(auth)) {
            strncpy(s_auth_token, auth->valuestring, TOKEN_MAX_LEN - 1);
            s_auth_token[TOKEN_MAX_LEN - 1] = '\0';
            save_token_to_nvs(s_auth_token);
            s_linked = true;
            ESP_LOGI(TAG, "Device linked successfully");
            cJSON_Delete(root);

            /* Start the background tasks now that we're linked */
            if (!s_ws_task_handle) {
                xTaskCreatePinnedToCore(obico_ws_task, "obico_ws", 6144,
                                        NULL, 5, &s_ws_task_handle, 0);
            }
            if (!s_snap_task_handle) {
                xTaskCreatePinnedToCore(obico_snap_task, "obico_snap", 6144,
                                        NULL, 4, &s_snap_task_handle, 0);
            }
            if (!s_janus_rx_task_handle && janus_proxy_configured()) {
                s_janus_sock_mutex = xSemaphoreCreateMutex();
                xTaskCreatePinnedToCore(janus_proxy_task, "janus_proxy", 4096,
                                        NULL, 5, &s_janus_rx_task_handle, 0);
            }
            return ESP_OK;
        }
    }

    /* Fallback: maybe token is at top level */
    const cJSON *auth_direct = cJSON_GetObjectItem(root, "auth_token");
    if (auth_direct && cJSON_IsString(auth_direct)) {
        strncpy(s_auth_token, auth_direct->valuestring, TOKEN_MAX_LEN - 1);
        s_auth_token[TOKEN_MAX_LEN - 1] = '\0';
        save_token_to_nvs(s_auth_token);
        s_linked = true;
        ESP_LOGI(TAG, "Device linked successfully");
        cJSON_Delete(root);

        if (!s_ws_task_handle) {
            xTaskCreatePinnedToCore(obico_ws_task, "obico_ws", 6144,
                                    NULL, 5, &s_ws_task_handle, 0);
        }
        if (!s_snap_task_handle) {
            xTaskCreatePinnedToCore(obico_snap_task, "obico_snap", 6144,
                                    NULL, 4, &s_snap_task_handle, 0);
        }
#ifdef CONFIG_OBICO_JANUS_PROXY_HOST
        if (!s_janus_rx_task_handle && janus_proxy_configured()) {
            s_janus_sock_mutex = xSemaphoreCreateMutex();
            xTaskCreatePinnedToCore(janus_proxy_task, "janus_proxy", 4096,
                                    NULL, 5, &s_janus_rx_task_handle, 0);
        }
#endif
        return ESP_OK;
    }

    ESP_LOGE(TAG, "No auth_token in response: %s", resp_buf);
    cJSON_Delete(root);
    return ESP_FAIL;
}

esp_err_t obico_unlink(void)
{
    s_linked = false;
    s_auth_token[0] = '\0';
    clear_token_from_nvs();

    /* Stop WebSocket */
    if (s_ws_client) {
        esp_websocket_client_close(s_ws_client, portMAX_DELAY);
        esp_websocket_client_destroy(s_ws_client);
        s_ws_client = NULL;
    }

    /* Tasks will notice s_linked == false and idle */
    ESP_LOGI(TAG, "Device unlinked");
    return ESP_OK;
}

bool obico_is_linked(void)
{
    return s_linked;
}

/* ---- Status JSON builder ---- */

static const char *opstate_to_text(printer_opstate_t st)
{
    switch (st) {
    case PRINTER_OPERATIONAL: return "Operational";
    case PRINTER_PRINTING:    return "Printing";
    case PRINTER_PAUSED:      return "Paused";
    case PRINTER_ERROR:       return "Error";
    default:                  return "Offline";
    }
}

/* Track simulated print start for current_print_ts */
static int64_t s_current_print_ts = -1;  /* unix timestamp when print started, -1 = not printing */

static char *build_status_json(const printer_state_t *ps)
{
    cJSON *root = cJSON_CreateObject();

    double now_ts = get_unix_timestamp();
    int64_t now_ts_int = (int64_t)now_ts;

    /*
     * current_print_ts: required by server for print tracking.
     * Must be consistent integer timestamp for entire print, -1 when not printing.
     */
    bool printing = ps->opstate == PRINTER_PRINTING;
    bool paused = ps->opstate == PRINTER_PAUSED;
    if (printing || paused) {
        if (s_current_print_ts < 0) {
            s_current_print_ts = now_ts_int;
        }
        cJSON_AddNumberToObject(root, "current_print_ts", (double)s_current_print_ts);
    } else {
        s_current_print_ts = -1;
        cJSON_AddNumberToObject(root, "current_print_ts", -1);
    }

    cJSON *status = cJSON_AddObjectToObject(root, "status");

    /* Timestamp — integer */
    cJSON_AddNumberToObject(status, "_ts", (double)now_ts_int);

    /* State */
    cJSON *state = cJSON_AddObjectToObject(status, "state");
    cJSON_AddStringToObject(state, "text", opstate_to_text(ps->opstate));

    cJSON *flags = cJSON_AddObjectToObject(state, "flags");
    bool operational = ps->opstate != PRINTER_DISCONNECTED && ps->opstate != PRINTER_ERROR;
    bool error = ps->opstate == PRINTER_ERROR;

    cJSON_AddBoolToObject(flags, "operational", operational);
    cJSON_AddBoolToObject(flags, "printing", printing);
    cJSON_AddBoolToObject(flags, "paused", paused);
    cJSON_AddBoolToObject(flags, "cancelling", false);
    cJSON_AddBoolToObject(flags, "pausing", false);
    cJSON_AddBoolToObject(flags, "resuming", false);
    cJSON_AddBoolToObject(flags, "finishing", false);
    cJSON_AddBoolToObject(flags, "error", error);
    cJSON_AddBoolToObject(flags, "ready", operational && !printing && !paused);
    cJSON_AddBoolToObject(flags, "closedOrError",
                          ps->opstate == PRINTER_DISCONNECTED || error);

    /* Temperatures — server expects "tool0" and "bed" */
    cJSON *temps = cJSON_AddObjectToObject(status, "temperatures");
    cJSON *tool0 = cJSON_AddObjectToObject(temps, "tool0");
    cJSON_AddNumberToObject(tool0, "actual", ps->hotend_actual);
    cJSON_AddNumberToObject(tool0, "target", ps->hotend_target);

    cJSON *bed = cJSON_AddObjectToObject(temps, "bed");
    cJSON_AddNumberToObject(bed, "actual", ps->bed_actual);
    cJSON_AddNumberToObject(bed, "target", ps->bed_target);

    /* Current Z height and layer number */
    cJSON_AddNumberToObject(status, "currentZ", ps->z);
    if (printing || paused) {
        float lh = ps->layer_height > 0 ? ps->layer_height : 0.2f;
        float flh = ps->first_layer_height > 0 ? ps->first_layer_height : lh;
        int cur_layer = 0;
        if (ps->z > 0) {
            if (ps->z <= flh) {
                cur_layer = 1;
            } else {
                cur_layer = 1 + (int)((ps->z - flh) / lh + 0.5f);
            }
        }
        cJSON_AddNumberToObject(status, "currentLayerHeight", cur_layer);
    }

    /* File metadata — needed by Obico UI for Z/layer totals */
    if (ps->object_height > 0 || ps->total_layers > 0) {
        cJSON *fmeta = cJSON_AddObjectToObject(status, "file_metadata");
        if (ps->object_height > 0) {
            cJSON *analysis = cJSON_AddObjectToObject(fmeta, "analysis");
            cJSON *pa = cJSON_AddObjectToObject(analysis, "printingArea");
            cJSON_AddNumberToObject(pa, "maxZ", ps->object_height);
        }
        if (ps->total_layers > 0) {
            cJSON *obico_meta = cJSON_AddObjectToObject(fmeta, "obico");
            cJSON_AddNumberToObject(obico_meta, "totalLayerCount", ps->total_layers);
        }
    }

    /* Progress */
    cJSON *progress = cJSON_AddObjectToObject(status, "progress");
    if (ps->progress_pct >= 0) {
        cJSON_AddNumberToObject(progress, "completion", ps->progress_pct);
    } else {
        cJSON_AddNullToObject(progress, "completion");
    }
    if (ps->print_time_s >= 0) {
        cJSON_AddNumberToObject(progress, "printTime", ps->print_time_s);
    } else {
        cJSON_AddNullToObject(progress, "printTime");
    }
    if (ps->print_time_left_s >= 0) {
        cJSON_AddNumberToObject(progress, "printTimeLeft", ps->print_time_left_s);
    } else {
        cJSON_AddNullToObject(progress, "printTimeLeft");
    }
    cJSON_AddNumberToObject(progress, "filepos", 0);

    /* Job */
    cJSON *job = cJSON_AddObjectToObject(status, "job");
    cJSON *file = cJSON_AddObjectToObject(job, "file");
    if (ps->filename[0]) {
        cJSON_AddStringToObject(file, "name", ps->filename);
    } else {
        cJSON_AddNullToObject(file, "name");
    }

    /* Settings */
    cJSON *settings = cJSON_AddObjectToObject(root, "settings");
    cJSON *agent = cJSON_AddObjectToObject(settings, "agent");
    cJSON_AddStringToObject(agent, "name", "esp32fdm");
    cJSON_AddStringToObject(agent, "version", "1.0.0");

    /* Webcam — declare camera exists (snapshots only, no WebRTC stream) */
    cJSON *webcams = cJSON_AddArrayToObject(settings, "webcams");
    cJSON *cam = cJSON_CreateObject();
    cJSON_AddStringToObject(cam, "name", "ESP32-CAM");
    cJSON_AddBoolToObject(cam, "is_primary_camera", true);
    cJSON_AddBoolToObject(cam, "flipV", false);
    cJSON_AddBoolToObject(cam, "flipH", false);
    cJSON_AddNumberToObject(cam, "rotation", 0);
    cJSON_AddStringToObject(cam, "streamRatio", "4:3");
    cJSON_AddItemToArray(webcams, cam);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

/* ---- WebSocket event handler ---- */

static void ws_event_handler(void *handler_args, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket connected to Obico");
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WebSocket disconnected");
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x01 && data->data_len > 0) {
            /* Text frame — parse JSON */
            char *buf = malloc(data->data_len + 1);
            if (!buf) break;
            memcpy(buf, data->data_ptr, data->data_len);
            buf[data->data_len] = '\0';

            ESP_LOGI(TAG, "WS RX: %.*s",
                     data->data_len > 256 ? 256 : (int)data->data_len, buf);

            cJSON *root = cJSON_Parse(buf);
            if (root) {
                /* Handle remote_status */
                const cJSON *remote = cJSON_GetObjectItem(root, "remote_status");
                if (remote) {
                    const cJSON *watch = cJSON_GetObjectItem(remote, "should_watch");
                    if (watch && cJSON_IsBool(watch)) {
                        s_should_watch = cJSON_IsTrue(watch);
                        ESP_LOGI(TAG, "should_watch=%d", s_should_watch);
                    }
                    const cJSON *viewing = cJSON_GetObjectItem(remote, "viewing");
                    if (viewing && cJSON_IsBool(viewing)) {
                        s_viewing = cJSON_IsTrue(viewing);
                        ESP_LOGI(TAG, "viewing=%d", s_viewing);
                    }
                }

                /* Handle Janus signaling relay */
                const cJSON *janus_msg = cJSON_GetObjectItem(root, "janus");
                if (janus_msg && cJSON_IsString(janus_msg) && janus_proxy_configured()) {
                    const char *janus_str = janus_msg->valuestring;
                    int janus_len = strlen(janus_str);
                    ESP_LOGI(TAG, "WS->sidecar janus msg (%d bytes)", janus_len);
                    janus_send_to_sidecar(janus_str, janus_len);
                }

                /* Handle commands: [{"cmd": "pause", "args": {...}, "initiator": "api"}] */
                const cJSON *cmds = cJSON_GetObjectItem(root, "commands");
                if (cmds && cJSON_IsArray(cmds)) {
                    int count = cJSON_GetArraySize(cmds);
                    for (int i = 0; i < count; i++) {
                        const cJSON *item = cJSON_GetArrayItem(cmds, i);
                        if (!cJSON_IsObject(item)) continue;

                        const cJSON *cmd_field = cJSON_GetObjectItem(item, "cmd");
                        if (!cmd_field || !cJSON_IsString(cmd_field)) continue;

                        const char *cmd_str = cmd_field->valuestring;
                        printer_cmd_t pcmd;
                        memset(&pcmd, 0, sizeof(pcmd));

                        if (strcmp(cmd_str, "pause") == 0) {
                            pcmd.type = PCMD_PAUSE;
                            printer_comm_send_cmd(&pcmd);
                            ESP_LOGI(TAG, "Obico command: pause");
                        } else if (strcmp(cmd_str, "resume") == 0) {
                            pcmd.type = PCMD_RESUME;
                            printer_comm_send_cmd(&pcmd);
                            ESP_LOGI(TAG, "Obico command: resume");
                        } else if (strcmp(cmd_str, "cancel") == 0) {
                            pcmd.type = PCMD_CANCEL;
                            printer_comm_send_cmd(&pcmd);
                            ESP_LOGI(TAG, "Obico command: cancel");
                        } else {
                            ESP_LOGW(TAG, "Unknown Obico command: %s", cmd_str);
                        }
                    }
                }

                cJSON_Delete(root);
            }
            free(buf);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
            if (data->error_handle.esp_tls_last_esp_err) {
                ESP_LOGW(TAG, "WS error: TLS: %s",
                         esp_err_to_name(data->error_handle.esp_tls_last_esp_err));
            }
            if (data->error_handle.esp_tls_stack_err) {
                ESP_LOGW(TAG, "WS error: TLS stack: 0x%x",
                         data->error_handle.esp_tls_stack_err);
            }
            if (data->error_handle.esp_transport_sock_errno) {
                ESP_LOGW(TAG, "WS error: socket errno %d (%s)",
                         data->error_handle.esp_transport_sock_errno,
                         strerror(data->error_handle.esp_transport_sock_errno));
            }
        } else {
            ESP_LOGW(TAG, "WS error type: %d", data->error_handle.error_type);
        }
        break;

    default:
        break;
    }
}

/* ---- WebSocket status task ---- */

static void obico_ws_task(void *arg)
{
    ESP_LOGI(TAG, "Obico WebSocket task started");

    while (true) {
        if (!s_linked) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* Build WebSocket URL */
        char ws_url[256];
        /* Convert https:// to wss:// or http:// to ws:// */
        const char *server = s_server_url;
        if (strncmp(server, "https://", 8) == 0) {
            snprintf(ws_url, sizeof(ws_url), "wss://%s/ws/dev/", server + 8);
        } else if (strncmp(server, "http://", 7) == 0) {
            snprintf(ws_url, sizeof(ws_url), "ws://%s/ws/dev/", server + 7);
        } else {
            snprintf(ws_url, sizeof(ws_url), "wss://%s/ws/dev/", server);
        }

        /* Build auth header */
        char auth_header[TOKEN_MAX_LEN + 20];
        snprintf(auth_header, sizeof(auth_header), "bearer %s", s_auth_token);

        esp_websocket_client_config_t ws_config = {
            .uri = ws_url,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .reconnect_timeout_ms = 10000,
            .network_timeout_ms = 20000,
            .buffer_size = 2048,
        };

        s_ws_client = esp_websocket_client_init(&ws_config);
        if (!s_ws_client) {
            ESP_LOGE(TAG, "Failed to create WebSocket client");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        /* Set auth header */
        esp_websocket_client_append_header(s_ws_client,
                                           "Authorization", auth_header);

        esp_websocket_register_events(s_ws_client, WEBSOCKET_EVENT_ANY,
                                      ws_event_handler, NULL);

        ESP_LOGI(TAG, "WS connecting to: %s", ws_url);

        int64_t t0 = esp_timer_get_time();
        esp_err_t err = esp_websocket_client_start(s_ws_client);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WebSocket start failed: %s", esp_err_to_name(err));
            esp_websocket_client_destroy(s_ws_client);
            s_ws_client = NULL;
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        /* Wait up to 20s for connection to establish (TLS handshake is slow) */
        for (int i = 0; i < 40; i++) {
            if (esp_websocket_client_is_connected(s_ws_client)) break;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        int elapsed_ms = (int)((esp_timer_get_time() - t0) / 1000);
        if (!esp_websocket_client_is_connected(s_ws_client)) {
            ESP_LOGE(TAG, "WebSocket failed to connect after %d ms", elapsed_ms);
            esp_websocket_client_stop(s_ws_client);
            esp_websocket_client_destroy(s_ws_client);
            s_ws_client = NULL;
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        ESP_LOGI(TAG, "WebSocket connected in %d ms", elapsed_ms);

        /* Send status periodically while connected */
        while (s_linked && esp_websocket_client_is_connected(s_ws_client)) {
            printer_state_t ps;
            printer_comm_get_state(&ps);

            char *json = build_status_json(&ps);
            if (json) {
                esp_websocket_client_send_text(s_ws_client, json,
                                               strlen(json), pdMS_TO_TICKS(5000));
                ESP_LOGD(TAG, "Status sent (%d bytes)", (int)strlen(json));
                free(json);
            }

            vTaskDelay(pdMS_TO_TICKS(CONFIG_OBICO_STATUS_INTERVAL * 1000));
        }

        /* Cleanup and reconnect */
        ESP_LOGW(TAG, "WebSocket disconnected, will reconnect...");
        esp_websocket_client_stop(s_ws_client);
        esp_websocket_client_destroy(s_ws_client);
        s_ws_client = NULL;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ---- Snapshot upload task ---- */

static esp_err_t upload_snapshot(const uint8_t *jpeg_buf, size_t jpeg_len)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/api/v1/octo/pic/",
             s_server_url);

    char auth_header[TOKEN_MAX_LEN + 20];
    snprintf(auth_header, sizeof(auth_header), "Token %s", s_auth_token);

    /*
     * Build multipart/form-data body manually.
     * Format:
     * --boundary\r\n
     * Content-Disposition: form-data; name="pic"; filename="snapshot.jpg"\r\n
     * Content-Type: image/jpeg\r\n
     * \r\n
     * <JPEG data>
     * \r\n
     * --boundary--\r\n
     */
    static const char *boundary = "esp32fdm_boundary";

    /* Part header */
    char part_header[256];
    int hdr_len = snprintf(part_header, sizeof(part_header),
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"pic\"; filename=\"snapshot.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n"
        "\r\n",
        boundary);

    /* Part footer */
    char part_footer[64];
    int ftr_len = snprintf(part_footer, sizeof(part_footer),
        "\r\n--%s--\r\n", boundary);

    int total_len = hdr_len + jpeg_len + ftr_len;

    /* Content-Type header with boundary */
    char content_type[80];
    snprintf(content_type, sizeof(content_type),
             "multipart/form-data; boundary=%s", boundary);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .buffer_size = 2048,       /* Obico cloud sends large response headers */
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", content_type);

    /* Open connection with known content length */
    esp_err_t err = esp_http_client_open(client, total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Snapshot HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    /* Write multipart body */
    esp_http_client_write(client, part_header, hdr_len);
    esp_http_client_write(client, (const char *)jpeg_buf, jpeg_len);
    esp_http_client_write(client, part_footer, ftr_len);

    /* Read response */
    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    (void)content_length;

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status >= 200 && status < 300) {
        ESP_LOGI(TAG, "Snapshot uploaded (%u bytes, HTTP %d)",
                 (unsigned)jpeg_len, status);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Snapshot upload failed: HTTP %d", status);
        return ESP_FAIL;
    }
}

static void obico_snap_task(void *arg)
{
    ESP_LOGI(TAG, "Obico snapshot task started");

    while (true) {
        if (!s_linked) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        printer_state_t ps;
        printer_comm_get_state(&ps);
        bool printing = (ps.opstate == PRINTER_PRINTING || ps.opstate == PRINTER_PAUSED);

        /*
         * Match official Obico agent behavior:
         * - Not printing AND nobody viewing → skip upload entirely
         * - Printing or viewing, with should_watch or viewing → fast (10s)
         * - Printing, but no should_watch and no viewing → slow (120s)
         */
        if (!printing && !s_viewing) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* Capture and upload */
        camera_frame_t *frame = camera_get_frame();
        if (frame) {
            upload_snapshot(frame->buf, frame->len);
            camera_release_frame(frame);
        } else {
            ESP_LOGW(TAG, "Snapshot capture failed");
        }

        int interval_s;
        if (s_viewing || s_should_watch) {
            interval_s = CONFIG_OBICO_SNAP_INTERVAL_PRINTING;
        } else {
            interval_s = CONFIG_OBICO_SNAP_INTERVAL_IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(interval_s * 1000));
    }
}

/* ---- HTTP endpoints for device linking ---- */

void obico_render_settings(html_buf_t *p)
{
    if (s_linked) {
        html_buf_printf(p,
            "<form id='f-obico' method='POST' action='/obico/unlink'></form>"
            "<p>Status: <span style='color:#4CAF50;font-weight:bold'>Linked</span>"
            " to %s</p>", s_server_url);
    } else {
        html_buf_printf(p,
            "<p>Status: <span style='color:#999'>Not linked</span></p>"
            "<form id='f-obico-server' method='POST' action='/obico/server'>"
            "<label>Server URL<br><input type='url' name='server_url' value='%s' "
            "placeholder='https://app.obico.io' "
            "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'></label>"
            "<button type='submit' style='margin:8px 0'>Save</button>"
            "</form>"
            "<form id='f-obico' method='POST' action='/obico/link'>"
            "<p>Enter the 6-digit code from <a href='%s' target='_blank'>Obico</a>:</p>"
            "<input type='text' name='code' placeholder='123456' maxlength='6' pattern='[0-9]{6}' required "
            "style='padding:8px;width:100%%;box-sizing:border-box;margin:8px 0'>"
            "</form>",
            s_server_url, s_server_url);
    }
#if CONFIG_ESP32FDM_SIMULATION
    html_buf_printf(p,
        "<p class='hint'><a href='/obico/status'>Status</a> | "
        "<a href='/obico/simulate'>Simulate</a></p>");
#else
    html_buf_printf(p,
        "<p class='hint'><a href='/obico/status'>Status</a></p>");
#endif
}

static esp_err_t obico_link_get_handler(httpd_req_t *req)
{
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "config");
}

static esp_err_t obico_link_post_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    /* Parse "code=123456" from form data */
    char code[16] = {0};
    const char *p = strstr(buf, "code=");
    if (p) {
        p += 5;
        int i = 0;
        while (*p && *p != '&' && i < 15) {
            code[i++] = *p++;
        }
        code[i] = '\0';
    }

    if (strlen(code) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing code");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Linking with code: %s", code);
    esp_err_t err = obico_link_with_code(code);

    html_buf_t hb;
    html_buf_init(&hb);
    layout_html_begin(&hb, err == ESP_OK ? "Linked" : "Link Failed", "/settings");
    if (err == ESP_OK) {
        html_buf_printf(&hb,
            "<h2 style='color:#3c763d'>Linked successfully!</h2>"
            "<p>This printer is now connected to Obico.</p>");
    } else {
        html_buf_printf(&hb,
            "<h2 style='color:#a94442'>Linking failed</h2>"
            "<p>Check the code and try again. <a href='/settings'>Back</a></p>");
    }
    layout_html_end(&hb);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, hb.data, hb.len);
    html_buf_free(&hb);
    return ret;
}

static esp_err_t obico_server_handler(httpd_req_t *req)
{
    char buf[256];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char url[SERVER_URL_MAX_LEN] = {0};
    const char *p = strstr(buf, "server_url=");
    if (p) {
        url_decode_field(p + 11, url, sizeof(url));
    }

    if (strlen(url) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing server_url");
        return ESP_FAIL;
    }

    obico_set_server_url(url);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t obico_unlink_handler(httpd_req_t *req)
{
    obico_unlink();

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t obico_status_handler(httpd_req_t *req)
{
    printer_state_t ps;
    printer_comm_get_state(&ps);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "linked", s_linked);
    cJSON_AddStringToObject(root, "server", s_server_url);
    cJSON_AddStringToObject(root, "printer_state", opstate_to_text(ps.opstate));
    cJSON_AddNumberToObject(root, "hotend", ps.hotend_actual);
    cJSON_AddNumberToObject(root, "bed", ps.bed_actual);
    if (ps.progress_pct >= 0) {
        cJSON_AddNumberToObject(root, "progress", ps.progress_pct);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t ret = httpd_resp_send(req, json, strlen(json));
    free(json);
    return ret;
}

#if CONFIG_ESP32FDM_SIMULATION
static esp_err_t obico_simulate_handler(httpd_req_t *req)
{
    bool currently_on = printer_comm_is_simulating();
    printer_comm_set_simulate(!currently_on);

    const char *state_str = !currently_on ? "ON &mdash; printing with fake data"
                                          : "OFF &mdash; real printer state";
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, "Simulate", "/settings");
    html_buf_printf(&p,
        "<h2>Simulation: %s</h2>"
        "<p><a href='/obico/simulate'>Toggle</a> | "
        "<a href='/obico/status'>Status</a></p>",
        state_str);
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}
#endif /* CONFIG_ESP32FDM_SIMULATION */

esp_err_t obico_register_httpd(void *server_handle)
{
    httpd_handle_t server = (httpd_handle_t)server_handle;

    httpd_uri_t link_get = {
        .uri      = "/obico/link",
        .method   = HTTP_GET,
        .handler  = obico_link_get_handler,
    };
    HTTPD_REGISTER(server, &link_get);

    httpd_uri_t link_post = {
        .uri      = "/obico/link",
        .method   = HTTP_POST,
        .handler  = obico_link_post_handler,
    };
    HTTPD_REGISTER(server, &link_post);

    httpd_uri_t server_post = {
        .uri      = "/obico/server",
        .method   = HTTP_POST,
        .handler  = obico_server_handler,
    };
    HTTPD_REGISTER(server, &server_post);

    httpd_uri_t unlink_post = {
        .uri      = "/obico/unlink",
        .method   = HTTP_POST,
        .handler  = obico_unlink_handler,
    };
    HTTPD_REGISTER(server, &unlink_post);

    httpd_uri_t status_uri = {
        .uri      = "/obico/status",
        .method   = HTTP_GET,
        .handler  = obico_status_handler,
    };
    HTTPD_REGISTER(server, &status_uri);

#if CONFIG_ESP32FDM_SIMULATION
    /* Ender Smart (F15-05): only in development builds. GET would let a click or a
     * browser prefetch switch to fake data and silently drop every printer command. */
    httpd_uri_t sim_uri = {
        .uri      = "/obico/simulate",
        .method   = HTTP_GET,
        .handler  = obico_simulate_handler,
    };
    HTTPD_REGISTER(server, &sim_uri);
#endif

    ESP_LOGI(TAG, "Obico HTTP endpoints registered");
    return ESP_OK;
}

/* ---- Janus proxy relay ---- */

static void janus_connect(void)
{
    if (s_janus_sock >= 0) return;

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(s_janus_port),
    };
    inet_pton(AF_INET, s_janus_host, &dest.sin_addr);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Janus proxy socket create failed");
        return;
    }

    /* 5s connect timeout */
    struct timeval tv = { .tv_sec = 5 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGW(TAG, "Janus proxy connect to %s:%d failed",
                 s_janus_host, s_janus_port);
        close(sock);
        return;
    }

    xSemaphoreTake(s_janus_sock_mutex, portMAX_DELAY);
    s_janus_sock = sock;
    xSemaphoreGive(s_janus_sock_mutex);

    ESP_LOGI(TAG, "Connected to Janus proxy at %s:%d",
             s_janus_host, s_janus_port);
}

static void janus_disconnect(void)
{
    xSemaphoreTake(s_janus_sock_mutex, portMAX_DELAY);
    if (s_janus_sock >= 0) {
        close(s_janus_sock);
        s_janus_sock = -1;
    }
    xSemaphoreGive(s_janus_sock_mutex);
}

/**
 * Send a length-prefixed message to the sidecar.
 * Wire format: 4-byte big-endian length + JSON payload.
 */
static void janus_send_to_sidecar(const char *json_str, int len)
{
    xSemaphoreTake(s_janus_sock_mutex, portMAX_DELAY);
    if (s_janus_sock < 0) {
        xSemaphoreGive(s_janus_sock_mutex);
        return;
    }

    uint8_t hdr[4];
    hdr[0] = (len >> 24) & 0xFF;
    hdr[1] = (len >> 16) & 0xFF;
    hdr[2] = (len >>  8) & 0xFF;
    hdr[3] = (len >>  0) & 0xFF;

    int sent = send(s_janus_sock, hdr, 4, 0);
    if (sent == 4) {
        sent = send(s_janus_sock, json_str, len, 0);
    }
    if (sent < 0) {
        ESP_LOGW(TAG, "Janus proxy send failed, disconnecting");
        close(s_janus_sock);
        s_janus_sock = -1;
    }
    xSemaphoreGive(s_janus_sock_mutex);
}

/**
 * Read a length-prefixed message from the sidecar.
 * Returns malloc'd buffer (caller frees) or NULL on error.
 */
static char *janus_recv_from_sidecar(int *out_len)
{
    int sock = s_janus_sock;
    if (sock < 0) return NULL;

    uint8_t hdr[4];
    int r = recv(sock, hdr, 4, MSG_WAITALL);
    if (r != 4) return NULL;

    int msg_len = (hdr[0] << 24) | (hdr[1] << 16) | (hdr[2] << 8) | hdr[3];
    if (msg_len <= 0 || msg_len > 8192) {
        ESP_LOGW(TAG, "Janus proxy bad msg len: %d", msg_len);
        return NULL;
    }

    char *buf = malloc(msg_len + 1);
    if (!buf) return NULL;

    r = recv(sock, buf, msg_len, MSG_WAITALL);
    if (r != msg_len) {
        free(buf);
        return NULL;
    }
    buf[msg_len] = '\0';
    *out_len = msg_len;
    return buf;
}

/**
 * Task: reads from sidecar TCP, wraps as {"janus":"..."} and sends to Obico WS.
 */
static void janus_proxy_task(void *arg)
{
    ESP_LOGI(TAG, "Janus proxy relay task started");

    while (true) {
        if (!s_linked) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* (Re)connect to sidecar */
        if (s_janus_sock < 0) {
            janus_connect();
            if (s_janus_sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }
        }

        /* Read a message from sidecar */
        int msg_len = 0;
        char *msg = janus_recv_from_sidecar(&msg_len);
        if (!msg) {
            /* Connection lost or timeout */
            ESP_LOGW(TAG, "Janus proxy recv failed, reconnecting...");
            janus_disconnect();
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        ESP_LOGI(TAG, "Sidecar→WS janus msg (%d bytes)", msg_len);

        /* Wrap as {"janus":"<escaped_json>"} and send to Obico WS */
        if (s_ws_client && esp_websocket_client_is_connected(s_ws_client)) {
            cJSON *wrapper = cJSON_CreateObject();
            cJSON_AddStringToObject(wrapper, "janus", msg);
            char *json_out = cJSON_PrintUnformatted(wrapper);
            cJSON_Delete(wrapper);

            if (json_out) {
                esp_websocket_client_send_text(s_ws_client, json_out,
                                               strlen(json_out), pdMS_TO_TICKS(5000));
                free(json_out);
            }
        }

        free(msg);
    }
}

/* ---- Public getter/setter for server URL ---- */

const char *obico_get_server_url(void)
{
    return s_server_url;
}

esp_err_t obico_set_server_url(const char *url)
{
    if (!url || url[0] == '\0') return ESP_ERR_INVALID_ARG;

    strncpy(s_server_url, url, sizeof(s_server_url) - 1);
    s_server_url[sizeof(s_server_url) - 1] = '\0';

    /* Strip trailing slash */
    size_t len = strlen(s_server_url);
    while (len > 0 && s_server_url[len - 1] == '/') {
        s_server_url[--len] = '\0';
    }

    /* Save to NVS */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_set_str(nvs, NVS_KEY_SERVER_URL, s_server_url);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Obico server URL updated: %s", s_server_url);
    return ESP_OK;
}

/* ---- Public getters/setters for Janus config ---- */

const char *obico_get_janus_host(void)
{
    return s_janus_host;
}

uint16_t obico_get_janus_port(void)
{
    return s_janus_port;
}

esp_err_t obico_set_janus_proxy(const char *host, uint16_t port)
{
    /* Update runtime vars */
    if (host) {
        strncpy(s_janus_host, host, sizeof(s_janus_host) - 1);
        s_janus_host[sizeof(s_janus_host) - 1] = '\0';
    } else {
        s_janus_host[0] = '\0';
    }
    if (port > 0) {
        s_janus_port = port;
    }

    /* Save to NVS */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_set_str(nvs, NVS_KEY_JANUS_HOST, s_janus_host);
    nvs_set_u16(nvs, NVS_KEY_JANUS_PORT, s_janus_port);
    nvs_commit(nvs);
    nvs_close(nvs);

    /* Disconnect existing Janus connection so it reconnects with new settings */
    if (s_janus_sock_mutex) {
        janus_disconnect();
    }

    /* Start Janus task if newly configured and linked */
    if (s_linked && janus_proxy_configured() && !s_janus_rx_task_handle) {
        if (!s_janus_sock_mutex) {
            s_janus_sock_mutex = xSemaphoreCreateMutex();
        }
        xTaskCreatePinnedToCore(janus_proxy_task, "janus_proxy", 4096,
                                NULL, 5, &s_janus_rx_task_handle, 0);
    }

    ESP_LOGI(TAG, "Janus proxy config updated: host=%s port=%d",
             s_janus_host, s_janus_port);
    return ESP_OK;
}

/* ---- Public init ---- */

esp_err_t obico_client_init(void)
{
    /* Load server URL from NVS (or Kconfig default) */
    load_server_url_from_nvs();

    /* Load Janus proxy config from NVS (or Kconfig defaults) */
    load_janus_config_from_nvs();

    /* Try to load existing token */
    load_token_from_nvs();

    if (s_linked) {
        ESP_LOGI(TAG, "Device is linked to Obico — starting tasks");

        xTaskCreatePinnedToCore(obico_ws_task, "obico_ws", 6144,
                                NULL, 5, &s_ws_task_handle, 0);
        xTaskCreatePinnedToCore(obico_snap_task, "obico_snap", 6144,
                                NULL, 4, &s_snap_task_handle, 0);
        if (janus_proxy_configured()) {
            s_janus_sock_mutex = xSemaphoreCreateMutex();
            xTaskCreatePinnedToCore(janus_proxy_task, "janus_proxy", 4096,
                                    NULL, 5, &s_janus_rx_task_handle, 0);
            ESP_LOGI(TAG, "Janus proxy relay enabled -> %s:%d",
                     s_janus_host, s_janus_port);
        }
    } else {
        ESP_LOGI(TAG, "Device not linked — visit /obico/link to connect");
    }

    return ESP_OK;
}
