#include "sdcard_httpd.h"
#include "httpd.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "layout.h"
#include "url_util.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "mbedtls/sha256.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <strings.h>

static const char *TAG = "sdcard_httpd";

#define MOUNT_POINT "/sdcard"
#define UPLOAD_BUF_SIZE 8192
/* Ender Smart (F3-02): uploads go to a hidden temporary name and are renamed only
 * when complete, so an interrupted upload never leaves a truncated file behind. */
#define UPLOAD_TMP_PREFIX ".up_"

/* Ender Smart (F15-14 / A5): one rule for every file name coming from the network.
 * Rejects path traversal ("..", "/", "\"), hidden/leading-dot names, control
 * characters and characters that break JSON/HTML/filesystems (" < > | : * ?). */
static bool sd_name_ok(const char *n)
{
    size_t len = strlen(n);
    /* No separators are allowed, so ".." inside a name cannot traverse; a leading
     * dot (".", "..", hidden files) is refused outright. */
    if (len == 0 || len >= 64 || n[0] == '.')
        return false;
    for (const unsigned char *p = (const unsigned char *)n; *p; p++) {
        if (*p < 0x20 || *p == 0x7f)
            return false;
        if (strchr("/\\\"<>|:*?", *p))
            return false;
    }
    return true;
}

/* JSON string escaping for names already on the card (copied from a PC). */
static void json_escape(const char *in, char *out, size_t out_size)
{
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)in; *p && o + 7 < out_size; p++) {
        if (*p == '"' || *p == '\\') {
            out[o++] = '\\';
            out[o++] = (char)*p;
        } else if (*p < 0x20) {
            o += snprintf(out + o, out_size - o, "\\u%04x", *p);
        } else {
            out[o++] = (char)*p;
        }
    }
    out[o] = '\0';
}


/* ---- GET /sd — HTML file manager ---- */

static esp_err_t sd_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, "SD Card", "/sd");
    html_buf_printf(&p,
        "<style>"
        "#modal-bg{display:none;position:fixed;top:0;left:0;width:100%%;height:100%%;background:rgba(0,0,0,0.5);z-index:100;align-items:center;justify-content:center}"
        "#modal-bg.show{display:flex}"
        "#modal{background:#fff;border-radius:8px;padding:20px;max-width:420px;width:90%%;max-height:80vh;overflow-y:auto;position:relative}"
        "#modal h3{margin:0 0 12px}"
        "#modal .close{position:absolute;top:8px;right:12px;font-size:1.4em;cursor:pointer;background:none;border:none;color:#666}"
        "#modal .thumb{text-align:center;margin:8px 0}"
        "#modal .thumb img{max-width:100%%;max-height:200px;border-radius:4px}"
        "#modal table{width:100%%}"
        "#modal td{padding:4px 8px}"
        "#modal td:first-child{color:#666;white-space:nowrap}"
        "</style>"
        "<div id='modal-bg' onclick='if(event.target===this)closeInfo()'>"
        "<div id='modal'>"
        "<button class='close' onclick='closeInfo()'>&times;</button>"
        "<h3 id='info-name'></h3>"
        "<div class='thumb' id='info-thumb'></div>"
        "<table>"
        "<tr><td>Size</td><td id='info-size'></td></tr>"
        "<tr><td>Layers</td><td id='info-layers'></td></tr>"
        "<tr><td>Layer Height</td><td id='info-lh'></td></tr>"
        "<tr><td>Object Height</td><td id='info-oh'></td></tr>"
        "<tr><td>Est. Time</td><td id='info-time'></td></tr>"
        "<tr><td>Filament</td><td id='info-fil'></td></tr>"
        "</table>"
        "</div></div>"
        "<h3>Upload</h3>"
        "<input type='file' id='file' accept='.gcode,.g,.gc'>"
        "<button onclick='upload()'>Upload</button>"
        "<progress id='upload-prog' value='0' max='100' style='width:100%%;display:none;margin:4px 0'></progress>"
        "<span id='upload-status'></span>"
        "<h3>Files</h3>"
        "<table><thead><tr><th>File</th><th></th></tr></thead>"
        "<tbody id='files'></tbody></table>"
        "<script>"
        "function fmt(b){if(b<1024)return b+'B';if(b<1048576)return(b/1024).toFixed(1)+'KB';return(b/1048576).toFixed(1)+'MB'}"
        "function fmtDate(t){if(!t)return'-';var d=new Date(t*1000),m=d.getMonth()+1,dy=d.getDate(),y=d.getFullYear(),h=d.getHours(),mi=d.getMinutes();return y+'-'+(m<10?'0':'')+m+'-'+(dy<10?'0':'')+dy+' '+(h<10?'0':'')+h+':'+(mi<10?'0':'')+mi}"
        "function fmtTime(s){if(s<0)return'Unknown';var h=Math.floor(s/3600),m=Math.floor((s%%3600)/60);return h>0?h+'h '+m+'m':m+'m'}"
        "function info(name){"
        "document.getElementById('info-name').textContent=name;"
        "document.getElementById('info-thumb').innerHTML='Loading...';"
        "document.getElementById('info-size').textContent='...';"
        "document.getElementById('info-layers').textContent='...';"
        "document.getElementById('info-lh').textContent='...';"
        "document.getElementById('info-oh').textContent='...';"
        "document.getElementById('info-time').textContent='...';"
        "document.getElementById('info-fil').textContent='...';"
        "document.getElementById('modal-bg').classList.add('show');"
        "fetch('/sd/info?filename='+encodeURIComponent(name)).then(function(r){return r.json()}).then(function(d){"
        "document.getElementById('info-size').textContent=fmt(d.size);"
        "document.getElementById('info-layers').textContent=d.layers>0?d.layers:'Unknown';"
        "document.getElementById('info-lh').textContent=d.layerHeight>0?d.layerHeight.toFixed(2)+' mm':'Unknown';"
        "document.getElementById('info-oh').textContent=d.objectHeight>0?d.objectHeight.toFixed(1)+' mm':'Unknown';"
        "document.getElementById('info-time').textContent=fmtTime(d.estTime);"
        "var fil='';"
        "if(d.filamentMm>0){fil=(d.filamentMm/1000).toFixed(2)+'m';if(d.filamentG>0)fil+=' / '+d.filamentG.toFixed(1)+'g'}"
        "else fil='Unknown';"
        "document.getElementById('info-fil').textContent=fil;"
        "if(d.thumbnail)document.getElementById('info-thumb').innerHTML='<img src=\"'+d.thumbnail+'\">';"
        "else document.getElementById('info-thumb').innerHTML='<em>No preview</em>';"
        "}).catch(function(){document.getElementById('info-thumb').innerHTML='<em>Error loading info</em>'})}"
        "function closeInfo(){document.getElementById('modal-bg').classList.remove('show')}"
        /* Ender Smart (F15-14): names are escaped for HTML; the Info button refers to
         * the file by index instead of pasting the name into JavaScript source */
        "var F=[];"
        "function esc(s){return String(s).replace(/[&<>\"']/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]})}"
        "function load(){"
        "fetch('/sd/files').then(function(r){return r.json()}).then(function(f){"
        "f.sort(function(a,b){return(b.mtime||0)-(a.mtime||0)});F=f;"
        "var h='';f.forEach(function(x,i){var n=esc(x.name);"
        "h+='<tr><td>'+n+'<br><span style=\"color:#888;font-size:.75em\">'+fmt(x.size)+' &mdash; '+fmtDate(x.mtime)+'</span></td><td>'"
        "+'<button onclick=\"info(F['+i+'].name)\" style=\"background:#48f;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">Info</button> '"
        "+'<form method=POST action=/sd/print style=display:inline onsubmit=\"return confirm(\\'Start printing this file?\\')\"><input type=hidden name=filename value=\"'+n+'\"><button type=submit style=\"background:#4a4;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">Print</button></form> '"
        "+'<form method=POST action=/sd/delete style=display:inline onsubmit=\"return confirm(\\'Delete this file?\\')\"><input type=hidden name=filename value=\"'+n+'\"><button type=submit style=\"background:#f44;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">Del</button></form>'"
        "+'</td></tr>'});"
        "document.getElementById('files').innerHTML=h||'<tr><td colspan=2>No files</td></tr>';});}"
        "function upload(){"
        "var f=document.getElementById('file').files[0];if(!f)return;"
        "var x=new XMLHttpRequest();"
        "x.upload.onprogress=function(e){if(e.lengthComputable){"
        "var p=document.getElementById('upload-prog');p.style.display='block';p.value=Math.round(e.loaded/e.total*100);"
        "document.getElementById('upload-status').textContent=fmt(e.loaded)+'/'+fmt(e.total);}};"
        "x.onload=function(){document.getElementById('upload-prog').style.display='none';"
        "document.getElementById('upload-status').textContent=x.status==200?'Done':'Error: '+x.responseText;load();};"
        "x.open('POST','/sd/upload');x.setRequestHeader('X-Filename',f.name);x.send(f);}"
        "load();"
        "</script>");
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/* ---- GET /sd/files — JSON file list ---- */

static esp_err_t sd_files_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    if (!sdcard_is_mounted()) {
        return httpd_resp_send(req, "[]", 2);
    }

    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        return httpd_resp_send(req, "[]", 2);
    }

    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;
    struct dirent *ent;
    char path[300];
    struct stat st;

    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.') continue;

        snprintf(path, sizeof(path), MOUNT_POINT "/%s", ent->d_name);
        if (stat(path, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) continue;

        char esc[300];
        json_escape(ent->d_name, esc, sizeof(esc));
        char entry[400];
        snprintf(entry, sizeof(entry), "%s{\"name\":\"%s\",\"size\":%ld,\"mtime\":%ld}",
                 first ? "" : ",", esc, (long)st.st_size, (long)st.st_mtime);
        httpd_resp_sendstr_chunk(req, entry);
        first = false;
    }
    closedir(dir);

    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ---- POST /sd/upload — stream file to SD (async, dual-core) ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
    char want_sha[65];   /* Ender Smart (F15-21): optional X-SHA256 header, "" if absent */
} sd_upload_ctx_t;

static void hex32(const uint8_t *in, char *out)
{
    for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", in[i]);
    out[64] = '\0';
}

/* SD writer context — shared between recv task (Core 0) and writer task (Core 1) */
typedef struct {
    StreamBufferHandle_t stream;
    volatile bool        recv_done;   /* recv task finished (success or error) */
    volatile bool        recv_error;  /* recv had an error */
    volatile bool        write_error; /* writer had an error */
    FILE                *file;
    int                  total;
    char                 path[300];
} sd_writer_ctx_t;

#define SD_STREAM_SIZE  1048576
#define SD_WRITE_BUF    32768

static void sd_writer_task(void *arg)
{
    sd_writer_ctx_t *wctx = (sd_writer_ctx_t *)arg;
    /* SDMMC DMA on ESP32-S3 cannot access PSRAM — use internal DMA-capable RAM */
    int wbuf_size = SD_WRITE_BUF;
    char *wbuf = NULL;
    while (wbuf_size >= 512) {
        wbuf = heap_caps_malloc(wbuf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (wbuf) break;
        wbuf_size /= 2;
    }
    if (!wbuf) {
        ESP_LOGE(TAG, "SD writer: no internal DMA memory");
        wctx->write_error = true;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SD writer buffer: %d bytes (internal DMA)", wbuf_size);

    int fd = fileno(wctx->file);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        wctx->write_error = true;
        free(wbuf);
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        size_t got = xStreamBufferReceive(wctx->stream, wbuf, wbuf_size,
                                           pdMS_TO_TICKS(100));
        if (got > 0) {
            if (write(fd, wbuf, got) != (ssize_t)got) {
                ESP_LOGE(TAG, "SD write error");
                wctx->write_error = true;
                break;
            }
        }

        /* Exit when recv is done and stream is drained */
        if (wctx->recv_done && xStreamBufferIsEmpty(wctx->stream)) break;
        if (wctx->recv_error) break;
    }

    fclose(wctx->file);
    wctx->file = NULL;  /* signal main task: we closed it */
    free(wbuf);
    vTaskDelete(NULL);
}

static void sd_upload_task(void *arg)
{
    sd_upload_ctx_t *ctx = (sd_upload_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    char path[300], final_path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/" UPLOAD_TMP_PREFIX "%s", ctx->filename);
    snprintf(final_path, sizeof(final_path), MOUNT_POINT "/%s", ctx->filename);
    unlink(path);  /* leftover of an earlier interrupted upload */

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    FILE *f = fopen(path, "w");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        goto done;
    }

    /* Allocate stream buffer storage in PSRAM to save internal RAM */
    uint8_t *stream_store = heap_caps_malloc(SD_STREAM_SIZE + 1, MALLOC_CAP_SPIRAM);
    StaticStreamBuffer_t *stream_static = heap_caps_malloc(sizeof(StaticStreamBuffer_t), MALLOC_CAP_SPIRAM);
    StreamBufferHandle_t stream = NULL;
    if (stream_store && stream_static) {
        stream = xStreamBufferCreateStatic(SD_STREAM_SIZE, 1, stream_store, stream_static);
    }
    if (!stream) {
        free(stream_store);
        free(stream_static);
        fclose(f);
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        goto done;
    }

    sd_writer_ctx_t wctx = {
        .stream      = stream,
        .recv_done   = false,
        .recv_error  = false,
        .write_error = false,
        .file        = f,
        .total       = req->content_len,
    };
    strlcpy(wctx.path, path, sizeof(wctx.path));

    /* Writer task on Core 1 — handles SD writes in parallel with recv */
    TaskHandle_t writer = NULL;
    if (xTaskCreatePinnedToCore(sd_writer_task, "sd_wr", 4096, &wctx,
                                 5, &writer, 1) != pdPASS) {
        vStreamBufferDelete(stream);
        free(stream_store);
        free(stream_static);
        fclose(f);
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Task create failed");
        goto done;
    }

    /* Recv loop on Core 0 — feeds data into StreamBuffer */
    char *rbuf = malloc(UPLOAD_BUF_SIZE);
    int remaining = req->content_len;
    int total = remaining;
    esp_err_t ret = ESP_OK;

    if (!rbuf) {
        wctx.recv_error = true;
        wctx.recv_done = true;
        ret = ESP_FAIL;
    }

    while (remaining > 0 && ret == ESP_OK) {
        if (wctx.write_error) {
            ret = ESP_FAIL;
            break;
        }

        int to_read = remaining < UPLOAD_BUF_SIZE ? remaining : UPLOAD_BUF_SIZE;
        int received = httpd_req_recv(req, rbuf, to_read);
        if (received <= 0) {
            ESP_LOGE(TAG, "Upload recv error: %d", received);
            ret = ESP_FAIL;
            break;
        }

        mbedtls_sha256_update(&sha, (const unsigned char *)rbuf, received);

        /* Push into stream buffer — may block if writer is slow */
        size_t sent = 0;
        while (sent < (size_t)received) {
            if (wctx.write_error) { ret = ESP_FAIL; break; }
            size_t n = xStreamBufferSend(stream, rbuf + sent, received - sent,
                                          pdMS_TO_TICKS(100));
            sent += n;
        }

        remaining -= received;
    }

    if (ret != ESP_OK) wctx.recv_error = true;
    wctx.recv_done = true;

    /* Wait for writer to finish draining */
    while (eTaskGetState(writer) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(rbuf);
    vStreamBufferDelete(stream);
    free(stream_store);
    free(stream_static);
    if (wctx.file) fclose(wctx.file);  /* writer task may have closed it already */

    uint8_t dig[32];
    char hex[65];
    mbedtls_sha256_finish(&sha, dig);
    hex32(dig, hex);

    if (ret != ESP_OK || wctx.write_error) {
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload failed");
    } else if (ctx->want_sha[0] && strcasecmp(ctx->want_sha, hex) != 0) {
        unlink(path);
        ESP_LOGW(TAG, "Upload %s: SHA-256 mismatch (got %s, expected %s), discarded", ctx->filename, hex, ctx->want_sha);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SHA-256 mismatch: file discarded");
    } else {
        /* FAT rename() does not replace an existing file */
        unlink(final_path);
        if (rename(path, final_path) != 0) {
            unlink(path);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Rename failed");
        } else {
            ESP_LOGI(TAG, "Uploaded %s (%d bytes, sha256 %s%s)", ctx->filename, total, hex,
                     ctx->want_sha[0] ? ", verified" : "");
            char esc[160], json[320];
            json_escape(ctx->filename, esc, sizeof(esc));
            snprintf(json, sizeof(json), "{\"ok\":true,\"name\":\"%s\",\"size\":%d,\"sha256\":\"%s\",\"verified\":%s}",
                     esc, total, hex, ctx->want_sha[0] ? "true" : "false");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json);
        }
    }

done:
    mbedtls_sha256_free(&sha);
    httpd_req_async_handler_complete(req);
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t sd_upload_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }

    /* Get filename from header (must read before async clone) */
    char filename[64];
    if (httpd_req_get_hdr_value_str(req, "X-Filename", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing X-Filename header");
        return ESP_FAIL;
    }

    /* Sanitize: strip path separators */
    char *slash = strrchr(filename, '/');
    if (slash) memmove(filename, slash + 1, strlen(slash + 1) + 1);
    slash = strrchr(filename, '\\');
    if (slash) memmove(filename, slash + 1, strlen(slash + 1) + 1);

    if (!sd_name_ok(filename)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    /* Ender Smart (F3-02): check before receiving anything */
    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty file");
        return ESP_FAIL;
    }
    int32_t free_kb = sdcard_get_free_kb();
    if (free_kb >= 0 && (int64_t)req->content_len + 1048576 > (int64_t)free_kb * 1024) {
        httpd_resp_set_status(req, "507 Insufficient Storage");
        return httpd_resp_sendstr(req, "Not enough free space on the SD card");
    }
    if (printer_comm_job_active()) {
        printer_state_t ps;
        printer_comm_get_state(&ps);
        if (strcmp(ps.filename, filename) == 0) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "This file is being printed");
        }
    }
    char want_sha[80] = "";
    if (httpd_req_get_hdr_value_str(req, "X-SHA256", want_sha, sizeof(want_sha)) == ESP_OK) {
        bool hex_ok = strlen(want_sha) == 64;
        for (int i = 0; hex_ok && i < 64; i++) {
            char c = (char)(want_sha[i] | 0x20);
            hex_ok = (want_sha[i] >= '0' && want_sha[i] <= '9') || (c >= 'a' && c <= 'f');
        }
        if (!hex_ok) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "X-SHA256 must be 64 hex characters");
            return ESP_FAIL;
        }
    } else {
        want_sha[0] = '\0';
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_upload_ctx_t *ctx = malloc(sizeof(sd_upload_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));
    strlcpy(ctx->want_sha, want_sha, sizeof(ctx->want_sha));

    if (xTaskCreatePinnedToCore(sd_upload_task, "sd_upload", 4096, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- POST /sd/delete ---- */

static esp_err_t sd_delete_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char *fn = strstr(body, "filename=");
    if (!fn) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    fn += 9;
    char *amp = strchr(fn, '&');
    if (amp) *amp = '\0';
    url_decode_inplace(fn);

    if (!sd_name_ok(fn)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", fn);

    if (unlink(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleted %s", fn);

    /* Redirect back to /sd */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/sd");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- POST /sd/print (async) ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
} sd_print_ctx_t;

static void sd_print_task(void *arg)
{
    sd_print_ctx_t *ctx = (sd_print_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    esp_err_t err;
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        err = klipper_backend_print_file(ctx->filename);
    } else {
        err = printer_comm_host_print_start(ctx->filename);
    }

    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to start print");
    } else {
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/sd");
        httpd_resp_send(req, NULL, 0);
    }

    httpd_req_async_handler_complete(req);
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t sd_print_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char *fn = strstr(body, "filename=");
    if (!fn) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    fn += 9;
    char *amp = strchr(fn, '&');
    if (amp) *amp = '\0';
    url_decode_inplace(fn);

    if (!sd_name_ok(fn)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_print_ctx_t *ctx = malloc(sizeof(sd_print_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, fn, sizeof(ctx->filename));

    if (xTaskCreatePinnedToCore(sd_print_task, "sd_print", 4096, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- POST /sd/pause, /sd/resume, /sd/cancel ---- */

static esp_err_t sd_pause_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_PAUSE };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_pause();
    }
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_resume_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_RESUME };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_resume();
    }
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_cancel_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_CANCEL };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_cancel();
    }
    return httpd_resp_send(req, "OK", 2);
}

/* ---- GET /sd/status — JSON print status ---- */

static esp_err_t sd_status_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    printer_state_t st;
    printer_comm_get_state(&st);

    bool is_printing;
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        /* Klipper: print status comes from Moonraker polling */
        is_printing = (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED);
    } else {
        /* Marlin: check host print state */
        is_printing = printer_comm_is_host_printing();
    }

    bool paused = (st.opstate == PRINTER_PAUSED);

    char json[256];
    if (is_printing) {
        snprintf(json, sizeof(json),
                 "{\"printing\":true,\"paused\":%s,\"filename\":\"%s\","
                 "\"progress\":%.1f,\"layer\":%ld,\"elapsed\":%ld}",
                 paused ? "true" : "false",
                 st.filename,
                 st.progress_pct >= 0 ? st.progress_pct : 0.0f,
                 (long)st.current_layer,
                 (long)(st.print_time_s >= 0 ? st.print_time_s : 0));
    } else {
        snprintf(json, sizeof(json), "{\"printing\":false}");
    }

    return httpd_resp_send(req, json, strlen(json));
}

/* ---- GET /sd/info — async G-code file info ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
} sd_info_ctx_t;

static void sd_info_task(void *arg)
{
    sd_info_ctx_t *ctx = (sd_info_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", ctx->filename);

    struct stat st;
    if (stat(path, &st) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        goto done;
    }

    gcode_file_info_t info;
    gcode_scan_file_info(path, &info);

    httpd_resp_set_type(req, "application/json");

    /* Send JSON in chunks to avoid huge stack buffer */
    char esc[160];
    json_escape(ctx->filename, esc, sizeof(esc));
    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"name\":\"%s\",\"size\":%ld,\"layers\":%ld,"
             "\"layerHeight\":%.2f,\"objectHeight\":%.2f,"
             "\"estTime\":%ld,\"filamentMm\":%.1f,\"filamentG\":%.1f",
             esc, (long)st.st_size, (long)info.total_layers,
             info.layer_height, info.max_z,
             (long)info.est_time_s, info.filament_used_mm, info.filament_used_g);
    httpd_resp_sendstr_chunk(req, buf);

    if (info.thumbnail_base64) {
        httpd_resp_sendstr_chunk(req, ",\"thumbnail\":\"data:image/png;base64,");
        httpd_resp_sendstr_chunk(req, info.thumbnail_base64);
        httpd_resp_sendstr_chunk(req, "\"");
        free(info.thumbnail_base64);
    } else {
        httpd_resp_sendstr_chunk(req, ",\"thumbnail\":null");
    }

    httpd_resp_sendstr_chunk(req, "}");
    httpd_resp_send_chunk(req, NULL, 0);

done:
    httpd_req_async_handler_complete(req);
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t sd_info_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }

    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char filename[64];
    if (httpd_query_key_value(query, "filename", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    url_decode_inplace(filename);

    /* Sanitize (F15-14) */
    if (!sd_name_ok(filename)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_info_ctx_t *ctx = malloc(sizeof(sd_info_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));

    if (xTaskCreatePinnedToCore(sd_info_task, "sd_info", 6144, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- Ender Smart (F15-21): GET /sd/sha256 and GET /sd/download ---- */

static bool s_sd_read_busy;   /* one SHA-256 or download at a time (SD bandwidth, sockets) */

typedef struct {
    httpd_req_t *req;
    char filename[64];
    bool download;
} sd_read_ctx_t;

/* attachment; filename="ascii fallback"; filename*=UTF-8''percent-encoded (RFC 6266) */
static void content_disposition(const char *name, char *out, size_t out_size)
{
    size_t o = (size_t)snprintf(out, out_size, "attachment; filename=\"");
    for (const unsigned char *p = (const unsigned char *)name; *p && o + 2 < out_size; p++)
        out[o++] = (*p >= 0x20 && *p < 0x7f) ? (char)*p : '_';
    o += snprintf(out + o, out_size - o, "\"; filename*=UTF-8''");
    for (const unsigned char *p = (const unsigned char *)name; *p && o + 4 < out_size; p++) {
        if ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || strchr("-._~", *p))
            out[o++] = (char)*p;
        else
            o += snprintf(out + o, out_size - o, "%%%02X", *p);
    }
    out[o < out_size ? o : out_size - 1] = '\0';
}

static void sd_read_task(void *arg)
{
    sd_read_ctx_t *ctx = (sd_read_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", ctx->filename);
    int64_t t0 = esp_timer_get_time();

    /* SDMMC DMA cannot reach PSRAM: read into internal DMA-capable RAM */
    int bsz = 16384;
    uint8_t *buf = NULL;
    while (bsz >= 512 && !(buf = heap_caps_malloc(bsz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)))
        bsz /= 2;
    int fd = open(path, O_RDONLY);

    if (fd < 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    } else if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    } else {
        mbedtls_sha256_context sha;
        mbedtls_sha256_init(&sha);
        mbedtls_sha256_starts(&sha, 0);
        if (ctx->download) {
            char cd[400];
            content_disposition(ctx->filename, cd, sizeof(cd));
            httpd_resp_set_type(req, "application/octet-stream");
            httpd_resp_set_hdr(req, "Content-Disposition", cd);
        }
        long total = 0;
        bool ok = true;
        while (true) {
            int n = read(fd, buf, bsz);
            if (n < 0) { ok = false; break; }
            if (n == 0) break;
            mbedtls_sha256_update(&sha, buf, n);
            total += n;
            if (ctx->download && httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) { ok = false; break; }
        }
        uint8_t dig[32];
        char hex[65];
        mbedtls_sha256_finish(&sha, dig);
        mbedtls_sha256_free(&sha);
        hex32(dig, hex);
        int ms = (int)((esp_timer_get_time() - t0) / 1000);

        if (ctx->download) {
            if (ok) httpd_resp_send_chunk(req, NULL, 0);
            ESP_LOGI(TAG, "Download %s: %ld bytes in %d ms%s, sha256 %s", ctx->filename, total, ms,
                     ok ? "" : " (interrupted)", hex);
        } else if (!ok) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD read error");
        } else {
            char esc[160], json[320];
            json_escape(ctx->filename, esc, sizeof(esc));
            snprintf(json, sizeof(json), "{\"name\":\"%s\",\"size\":%ld,\"sha256\":\"%s\",\"ms\":%d}", esc, total, hex, ms);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json);
            ESP_LOGI(TAG, "SHA-256 %s: %s (%ld bytes, %d ms)", ctx->filename, hex, total, ms);
        }
    }

    if (fd >= 0) close(fd);
    free(buf);
    httpd_req_async_handler_complete(req);
    free(ctx);
    __atomic_clear(&s_sd_read_busy, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

static esp_err_t sd_read_start(httpd_req_t *req, bool download)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }
    char query[256], filename[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "filename", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    url_decode_inplace(filename);
    if (!sd_name_ok(filename)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }
    /* A full-file read competes with the print's own SD reads */
    if (printer_comm_job_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Blocked while a print is running or paused");
    }
    if (__atomic_test_and_set(&s_sd_read_busy, __ATOMIC_ACQ_REL)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "10");
        return httpd_resp_sendstr(req, "Another SHA-256 or download is running, try again shortly");
    }

    httpd_req_t *async_req = NULL;
    sd_read_ctx_t *ctx = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        __atomic_clear(&s_sd_read_busy, __ATOMIC_RELEASE);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }
    ctx = malloc(sizeof(*ctx));
    if (!ctx) goto fail;
    ctx->req = async_req;
    ctx->download = download;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));
    if (xTaskCreatePinnedToCore(sd_read_task, download ? "sd_dl" : "sd_sha", 6144, ctx, 5, NULL, 0) != pdPASS)
        goto fail;
    return ESP_OK;

fail:
    free(ctx);
    httpd_req_async_handler_complete(async_req);
    __atomic_clear(&s_sd_read_busy, __ATOMIC_RELEASE);
    return ESP_FAIL;
}

static esp_err_t sd_sha256_handler(httpd_req_t *req) { return sd_read_start(req, false); }
static esp_err_t sd_download_handler(httpd_req_t *req) { return sd_read_start(req, true); }

/* Remove temporary files left by uploads interrupted before a reboot */
static void sd_cleanup_tmp_uploads(void)
{
    if (!sdcard_is_mounted()) return;
    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) return;
    struct dirent *ent;
    char path[300];
    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, UPLOAD_TMP_PREFIX, strlen(UPLOAD_TMP_PREFIX)) != 0) continue;
        snprintf(path, sizeof(path), MOUNT_POINT "/%s", ent->d_name);
        if (unlink(path) == 0) ESP_LOGW(TAG, "Removed unfinished upload %s", ent->d_name);
    }
    closedir(dir);
}

/* ---- Registration ---- */

esp_err_t sdcard_httpd_register(httpd_handle_t server)
{
    sd_cleanup_tmp_uploads();

    const httpd_uri_t uris[] = {
        { .uri = "/sd",        .method = HTTP_GET,  .handler = sd_page_handler },
        { .uri = "/sd/files",  .method = HTTP_GET,  .handler = sd_files_handler },
        { .uri = "/sd/upload", .method = HTTP_POST, .handler = sd_upload_handler },
        { .uri = "/sd/delete", .method = HTTP_POST, .handler = sd_delete_handler },
        { .uri = "/sd/print",  .method = HTTP_POST, .handler = sd_print_handler },
        { .uri = "/sd/pause",  .method = HTTP_POST, .handler = sd_pause_handler },
        { .uri = "/sd/resume", .method = HTTP_POST, .handler = sd_resume_handler },
        { .uri = "/sd/cancel", .method = HTTP_POST, .handler = sd_cancel_handler },
        { .uri = "/sd/info",   .method = HTTP_GET,  .handler = sd_info_handler },
        { .uri = "/sd/status", .method = HTTP_GET,  .handler = sd_status_handler },
        { .uri = "/sd/sha256", .method = HTTP_GET,  .handler = sd_sha256_handler },
        { .uri = "/sd/download", .method = HTTP_GET, .handler = sd_download_handler },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }

    ESP_LOGI(TAG, "SD card HTTP endpoints registered");
    return ESP_OK;
}
