#include "ota.h"
#include "httpd.h"

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_image_format.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "version.h"

static const char *TAG = "ota";

/* ---- Boot confirmation ---- */

void ota_confirm_boot(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "First boot after OTA — marking firmware valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

static const char *ota_state_str(esp_ota_img_states_t s)
{
    switch (s) {
    case ESP_OTA_IMG_NEW:            return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
    case ESP_OTA_IMG_VALID:          return "valid";
    case ESP_OTA_IMG_INVALID:        return "invalid";
    case ESP_OTA_IMG_ABORTED:        return "aborted";
    case ESP_OTA_IMG_UNDEFINED:      return "undefined";
    default:                         return "?";
    }
}

static void boot_confirm_task(void *arg)
{
    uint32_t delay_s = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_s * 1000));
    ota_confirm_boot();
    vTaskDelete(NULL);
}

void ota_schedule_boot_confirm(uint32_t delay_s)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    bool have_state = running && esp_ota_get_state_partition(running, &state) == ESP_OK;
    ESP_LOGI(TAG, "Running from %s, OTA state: %s",
             running ? running->label : "?", have_state ? ota_state_str(state) : "n/a");

#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (have_state && state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "New firmware: will be confirmed after %lu s of stable running "
                      "(a crash or reset before that rolls back)", (unsigned long)delay_s);
        xTaskCreatePinnedToCore(boot_confirm_task, "ota_ok", 3072,
                                (void *)(uintptr_t)delay_s, 1, NULL, 0);
    }
#else
    (void)delay_s;
    (void)boot_confirm_task;
#endif
}

/* ---- OTA upload (async) ---- */

#define OTA_BUF_SIZE 4096

typedef struct {
    httpd_req_t *req;
} ota_upload_ctx_t;

static void ota_upload_task(void *arg)
{
    ota_upload_ctx_t *ctx = (ota_upload_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    free(ctx);

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        goto done;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        goto done;
    }

    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        goto done;
    }

    int remaining = req->content_len;
    int total = remaining;
    bool error = false;
    bool header_checked = false;
    int received_total = 0;

    ESP_LOGI(TAG, "OTA update starting: %d bytes → %s", total, update->label);

    while (remaining > 0) {
        int to_read = remaining < OTA_BUF_SIZE ? remaining : OTA_BUF_SIZE;
        int received = httpd_req_recv(req, buf, to_read);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "OTA recv error at %d/%d", total - remaining, total);
            error = true;
            break;
        }

        /* Validate project name from the image header (first chunk) */
        if (!header_checked && received_total == 0 && received >= 256) {
            header_checked = true;
            /* esp_app_desc_t starts at offset 32 (image header + segment header) */
            const esp_app_desc_t *new_desc = (const esp_app_desc_t *)(buf + sizeof(esp_image_header_t)
                                              + sizeof(esp_image_segment_header_t));
            if (new_desc->magic_word != ESP_APP_DESC_MAGIC_WORD) {
                ESP_LOGE(TAG, "Bad app descriptor magic: 0x%08x", (unsigned)new_desc->magic_word);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                    "Not a valid ESP32 application image");
                goto done;
            }

            const esp_app_desc_t *cur_desc = esp_app_get_description();
            if (strcmp(new_desc->project_name, cur_desc->project_name) != 0) {
                ESP_LOGE(TAG, "Project mismatch: running \"%s\", uploaded \"%s\"",
                         cur_desc->project_name, new_desc->project_name);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                    "Wrong firmware — project name does not match");
                goto done;
            }

            ESP_LOGI(TAG, "Firmware verified: project \"%s\", version \"%s\"",
                     new_desc->project_name, new_desc->version);
        }

        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            error = true;
            break;
        }

        received_total += received;
        remaining -= received;
    }

    free(buf);

    if (error) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
        goto done;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s (image validation error?)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Firmware validation failed — bad image?");
        goto done;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        goto done;
    }

    ESP_LOGI(TAG, "OTA update successful! Rebooting to %s", update->label);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Firmware updated. Rebooting...\"}");

    httpd_req_async_handler_complete(req);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return;  /* unreachable */

done:
    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware data");
        return ESP_FAIL;
    }

    /* Sanity check: firmware should be at least 256 bytes and under 2MB */
    if (req->content_len < 256 || req->content_len > 0x200000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware size");
        return ESP_FAIL;
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async begin failed");
        return ESP_FAIL;
    }

    ota_upload_ctx_t *ctx = malloc(sizeof(*ctx));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;

    if (xTaskCreatePinnedToCore(ota_upload_task, "ota_up", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        free(ctx);
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- HTTP registration ---- */

void ota_register_httpd(httpd_handle_t server)
{
    httpd_uri_t upload_uri = {
        .uri     = "/ota/upload",
        .method  = HTTP_POST,
        .handler = ota_upload_handler,
    };
    HTTPD_REGISTER(server, &upload_uri);
}

/* ---- Settings page section ---- */

void ota_render_settings(html_buf_t *p)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_app_desc_t *app = esp_app_get_description();

    html_buf_printf(p,
        "<hr><div class='sh'><h2>Firmware Update</h2></div>"
        "<p class='hint'>Running: <b>v%s</b> on <b>%s</b>"
        " &middot; Built %s %s</p>"
        "<form id='ota-form'>"
        "<input type='file' id='ota-file' accept='.bin' "
        "style='margin:8px 0;display:block'>"
        "<button type='submit' id='ota-btn'>Upload &amp; Flash</button>"
        "</form>"
        "<progress id='ota-prog' max='100' value='0' "
        "style='width:100%%;display:none;margin:8px 0'></progress>"
        "<div id='ota-status' style='font-size:13px;color:#666;margin:4px 0'></div>"
        "<script>"
        "document.getElementById('ota-form').onsubmit=function(e){"
        "e.preventDefault();"
        "var f=document.getElementById('ota-file').files[0];"
        "if(!f){alert('Select a firmware .bin file');return;}"
        "if(!confirm('Flash firmware '+f.name+' ('+Math.round(f.size/1024)+' KB)?'))return;"
        "var xhr=new XMLHttpRequest();"
        "var prog=document.getElementById('ota-prog');"
        "var stat=document.getElementById('ota-status');"
        "var btn=document.getElementById('ota-btn');"
        "btn.disabled=true;prog.style.display='block';"
        "xhr.upload.onprogress=function(ev){"
        "if(ev.lengthComputable){prog.value=Math.round(ev.loaded/ev.total*100);"
        "stat.textContent='Uploading: '+Math.round(ev.loaded/1024)+'/'+Math.round(ev.total/1024)+' KB';}};"
        "xhr.onload=function(){"
        "if(xhr.status==200){stat.textContent='Firmware flashed! Rebooting...';"
        "stat.style.color='#2e7d32';"
        "setTimeout(function(){location.reload()},8000);}"
        "else{stat.textContent='Error: '+xhr.responseText;stat.style.color='#c62828';btn.disabled=false;}};"
        "xhr.onerror=function(){stat.textContent='Upload failed';stat.style.color='#c62828';btn.disabled=false;};"
        "xhr.open('POST','/ota/upload');"
        "xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "xhr.send(f);};"
        "</script>",
        FW_VERSION, running ? running->label : "?",
        app ? app->date : "?", app ? app->time : "?");
}
