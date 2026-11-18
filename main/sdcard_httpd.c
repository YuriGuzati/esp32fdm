#include "sdcard_httpd.h"
#include "httpd.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "layout.h"
#include "url_util.h"

#include "gcode_preview.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include <sys/socket.h>
#include <netinet/tcp.h>

#include <dirent.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
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

/* Ender Smart (lote 18): an upload whose data stops (a phone that loses Wi-Fi for a few
 * seconds) is waited for up to UPLOAD_STALL_MAX_US instead of failing at the first recv
 * timeout (recv_wait_timeout, 5 s). A closed connection still fails at once.
 *
 * One upload at a time: while a stalled upload waits, a second upload of the same file would
 * unlink and recreate the temporary file the first one still has open, and with
 * CONFIG_FATFS_FS_LOCK=0 FatFs allows that (the first f_close would then write back a directory
 * entry pointing at freed clusters). The flag is taken in the handler and released by the
 * upload task before it answers, so the client can start the next upload right away. */
#define UPLOAD_STALL_MAX_US (30 * 1000000LL)
static atomic_bool s_upload_busy;

static void upload_release(void)
{
    atomic_store(&s_upload_busy, false);
}

/* lote 18: refuse an upload before reading its body, then close the connection (ESP_FAIL).
 * Returning ESP_OK would make httpd_req_delete() read and discard the whole body in the server
 * task, 32 bytes at a time, freezing every other page (Pause, Cancel, emergency stop) for the
 * whole transfer. TCP_NODELAY sends the answer before the close; since the body is not read,
 * the client may still see only a connection reset. */
static esp_err_t upload_refuse(httpd_req_t *req, const char *status, const char *msg)
{
    int fd = httpd_req_to_sockfd(req);
    int one = 1;
    if (fd >= 0) setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, msg);
    return ESP_FAIL;
}

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
    /* Ender Smart (F4-07): the old page was removed; its screen in the new UI */
    return httpd_redirect_app(req, "arquivos");
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
    TaskHandle_t         owner;       /* lote 18: notified once the writer is done with this struct */
    unsigned             writer_stack_free;   /* lote 18b: high-water mark of sd_wr, for the log */
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
        xTaskNotifyGive(wctx->owner);   /* lote 18: last access to wctx */
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SD writer buffer: %d bytes (internal DMA)", wbuf_size);

    int fd = fileno(wctx->file);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        wctx->write_error = true;
        free(wbuf);
        xTaskNotifyGive(wctx->owner);
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
    wctx->writer_stack_free = (unsigned)uxTaskGetStackHighWaterMark(NULL);
    /* lote 18: the upload task waits for this instead of polling eTaskGetState() on a task
     * whose TCB the idle task may already have freed */
    xTaskNotifyGive(wctx->owner);
    vTaskDelete(NULL);
}

static bool sd_name_in_use(const char *filename);   /* lote 11, below */

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
        upload_release();
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
        upload_release();
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
        .owner       = xTaskGetCurrentTaskHandle(),
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
        upload_release();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Task create failed");
        goto done;
    }

    /* Recv loop on Core 0 — feeds data into StreamBuffer */
    char *rbuf = malloc(UPLOAD_BUF_SIZE);
    int remaining = req->content_len;
    int total = remaining;
    esp_err_t ret = ESP_OK;
    int64_t last_data = esp_timer_get_time();   /* lote 18 */
    bool stalled = false;

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
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            /* lote 18: no data for recv_wait_timeout, but the connection is still open */
            int64_t idle = esp_timer_get_time() - last_data;
            if (!stalled) {
                stalled = true;
                ESP_LOGW(TAG, "Upload %s: no data for %lld s (%d of %d bytes received), waiting up to %lld s",
                         ctx->filename, idle / 1000000, total - remaining, total, UPLOAD_STALL_MAX_US / 1000000);
            }
            if (idle < UPLOAD_STALL_MAX_US) {
                vTaskDelay(pdMS_TO_TICKS(20));   /* never a busy loop, even if recv returned at once */
                continue;
            }
            ESP_LOGE(TAG, "Upload %s: no data for %lld s, giving up", ctx->filename, idle / 1000000);
            ret = ESP_FAIL;
            break;
        }
        if (received <= 0) {
            ESP_LOGE(TAG, "Upload recv error: %d", received);
            ret = ESP_FAIL;
            break;
        }
        if (stalled) {
            ESP_LOGI(TAG, "Upload %s: data again after %lld ms without data", ctx->filename,
                     (esp_timer_get_time() - last_data) / 1000);
            stalled = false;
        }
        last_data = esp_timer_get_time();

        mbedtls_sha256_update(&sha, (const unsigned char *)rbuf, received);

        /* Push into stream buffer — may block if writer is slow */
        size_t sent = 0;
        while (sent < (size_t)received) {
            if (wctx.write_error) { ret = ESP_FAIL; break; }
            size_t n = xStreamBufferSend(stream, rbuf + sent, received - sent,
                                          pdMS_TO_TICKS(100));
            sent += n;
        }
        last_data = esp_timer_get_time();   /* lote 18: a slow SD card is not a silent client */

        remaining -= received;
    }

    if (ret != ESP_OK) wctx.recv_error = true;
    wctx.recv_done = true;

    /* Wait for writer to finish draining (lote 18: its notification, see sd_writer_task) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    free(rbuf);
    vStreamBufferDelete(stream);
    free(stream_store);
    free(stream_static);
    if (wctx.file) fclose(wctx.file);  /* writer task may have closed it already */

    uint8_t dig[32];
    char hex[65];
    mbedtls_sha256_finish(&sha, dig);
    hex32(dig, hex);

    /* lote 18: each branch finishes with the files first, then releases the upload slot, then answers */
    if (ret != ESP_OK || wctx.write_error) {
        unlink(path);
        upload_release();
        ESP_LOGW(TAG, "Upload %s failed after %d of %d bytes, discarded (stack_free %u, writer %u)", ctx->filename,
                 total - remaining, total, (unsigned)uxTaskGetStackHighWaterMark(NULL), wctx.writer_stack_free);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload failed");
    } else if (ctx->want_sha[0] && strcasecmp(ctx->want_sha, hex) != 0) {
        unlink(path);
        upload_release();
        ESP_LOGW(TAG, "Upload %s: SHA-256 mismatch (got %s, expected %s), discarded", ctx->filename, hex, ctx->want_sha);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SHA-256 mismatch: file discarded");
    } else {
        /* lote 11: a print or a preparation of this name may have started during the upload;
         * unlink() of an open file frees clusters the print is still reading */
        if (sd_name_in_use(ctx->filename)) {
            unlink(path);
            upload_release();
            ESP_LOGW(TAG, "Upload %s discarded: the file is being printed or prepared", ctx->filename);
            httpd_resp_set_status(req, "409 Conflict");
            httpd_resp_sendstr(req, "This file is being printed");
            goto done;
        }
        /* FAT rename() does not replace an existing file */
        unlink(final_path);
        if (rename(path, final_path) != 0) {
            unlink(path);
            upload_release();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Rename failed");
        } else {
            upload_release();
            gcode_preview_delete(ctx->filename);   /* lote 19: new content, the old preview no longer fits */
            ESP_LOGI(TAG, "Uploaded %s (%d bytes, sha256 %s%s, stack_free %u, writer %u)", ctx->filename, total, hex,
                     ctx->want_sha[0] ? ", verified" : "", (unsigned)uxTaskGetStackHighWaterMark(NULL),
                     wctx.writer_stack_free);
            char esc[160], json[320];
            json_escape(ctx->filename, esc, sizeof(esc));
            snprintf(json, sizeof(json), "{\"ok\":true,\"name\":\"%s\",\"size\":%d,\"sha256\":\"%s\",\"verified\":%s}",
                     esc, total, hex, ctx->want_sha[0] ? "true" : "false");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json);
        }
    }

done:
    /* lote 18: no upload_release() here: every path above already did it before answering,
     * and by now a new upload may hold the slot */
    mbedtls_sha256_free(&sha);
    httpd_req_async_handler_complete(req);
    free(ctx);
    vTaskDelete(NULL);
}

/** Ender Smart (lote 11): the file of the print running/paused, or being prepared for it.
 *  Overwriting or deleting it would make the print read changed or freed clusters. */
static bool sd_name_in_use(const char *filename)
{
    if (!printer_comm_job_active()) return false;
    printer_state_t ps;
    printer_comm_get_state(&ps);
    if (ps.filename[0] && strcmp(ps.filename, filename) == 0) return true;
    char prep[64];
    return printer_comm_preparing(prep, sizeof(prep), NULL) && strcmp(prep, filename) == 0;
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
    if (free_kb >= 0 && (int64_t)req->content_len + 1048576 > (int64_t)free_kb * 1024)
        return upload_refuse(req, "507 Insufficient Storage", "Not enough free space on the SD card");
    if (sd_name_in_use(filename))
        return upload_refuse(req, "409 Conflict", "This file is being printed");
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

    /* lote 18: one upload at a time (see UPLOAD_STALL_MAX_US) */
    bool idle = false;
    if (!atomic_compare_exchange_strong(&s_upload_busy, &idle, true)) {
        ESP_LOGW(TAG, "Upload %s refused: another upload is in progress", filename);
        return upload_refuse(req, "409 Conflict",
                             "Outro envio em andamento. Se ele parou, a ESP32 desiste dele depois de 30 s sem dados; "
                             "tente de novo em seguida.");
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        upload_release();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_upload_ctx_t *ctx = malloc(sizeof(sd_upload_ctx_t));
    if (!ctx) {
        upload_release();
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));
    strlcpy(ctx->want_sha, want_sha, sizeof(ctx->want_sha));

    /* lote 18b: 8 KB. With 4096 B the lote 18 panicked three times (stack overflow): a stalled or
     * reset upload went through more ESP_LOG calls (each one formats in the caller's stack, and
     * the log hook adds 512 B), and a normal upload already left only ~600 B free. */
    if (xTaskCreatePinnedToCore(sd_upload_task, "sd_upload", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        upload_release();
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

    /* lote 11: the old UI disabled the button, but the route deleted it anyway */
    if (sd_name_in_use(fn)) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Este arquivo está sendo impresso ou preparado para imprimir");
    }

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", fn);

    if (unlink(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleted %s", fn);
    gcode_preview_delete(fn);   /* lote 19 */

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
        return;
    }

    /* Ender Smart (lote 11): answer at once, then read the file here (not in the printer task)
     * while the screen shows "Preparando"; the start is queued at the end. */
    const char *why = "";
    err = printer_comm_prepare_begin(ctx->filename, &why);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, err == ESP_ERR_NOT_FOUND ? "404 Not Found"
                                 : err == ESP_ERR_INVALID_ARG ? "400 Bad Request" : "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, why);
        httpd_req_async_handler_complete(req);
        free(ctx);
        vTaskDelete(NULL);
        return;
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/sd");
    httpd_resp_send(req, NULL, 0);
    httpd_req_async_handler_complete(req);
    free(ctx);

    /* lowest priority above idle: a big file keeps this task busy for seconds */
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 1);
    printer_comm_prepare_run();
    ESP_LOGI(TAG, "sd_print stack free: %u of 6144 B", (unsigned)uxTaskGetStackHighWaterMark(NULL));
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

    /* lote 11: 4096 -> 6144, the file scan (strtof, logs with floats) now runs in this task */
    if (xTaskCreatePinnedToCore(sd_print_task, "sd_print", 6144, ctx,
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
        /* lote 11: with no job yet, "Cancelar" stops the preparation (nothing has moved).
         * The "OK" of an error also posts here: then it only acknowledges the error. */
        printer_state_t st;
        printer_comm_get_state(&st);
        if (st.opstate != PRINTER_ERROR && !printer_comm_is_host_printing() &&
            printer_comm_prepare_cancel())
            return httpd_resp_send(req, "OK", 2);
        printer_comm_host_print_cancel();
    }
    return httpd_resp_send(req, "OK", 2);
}

/* ---- POST /sd/prepare/cancel (lote 11): only a preparation, never a running print ---- */

static esp_err_t sd_prepare_cancel_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    if (printer_comm_prepare_cancel())
        return httpd_resp_sendstr(req, "OK");
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "Nenhum arquivo está sendo preparado (a impressão pode já ter começado: use Cancelar)");
}

/* ---- POST /sd/ack-error (lote 11): the "OK" of the error banner, never cancels a print ---- */

static esp_err_t sd_ack_error_handler(httpd_req_t *req)
{
    printer_comm_ack_error();
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
        httpd_resp_sendstr_chunk(req, "\",\"thumbnailSource\":\"file\"");
        free(info.thumbnail_base64);
    } else {
        /* lote 19: a preview drawn by the page from the G-code, if there is one for this size */
        size_t n = 0;
        uint8_t *img = gcode_preview_load(ctx->filename, (long)st.st_size, &n);
        size_t b64_len = 0;
        unsigned char *b64 = NULL;
        if (img) {
            mbedtls_base64_encode(NULL, 0, &b64_len, img, n);   /* size needed, with the final NUL */
            b64 = heap_caps_malloc(b64_len, MALLOC_CAP_SPIRAM);
            if (b64 && mbedtls_base64_encode(b64, b64_len, &b64_len, img, n) != 0) {
                free(b64);
                b64 = NULL;
            }
        }
        if (b64) {
            snprintf(buf, sizeof(buf), ",\"thumbnail\":\"data:%s;base64,", gcode_preview_mime(img, n));
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, (const char *)b64);
            httpd_resp_sendstr_chunk(req, "\",\"thumbnailSource\":\"preview\"");
        } else {
            httpd_resp_sendstr_chunk(req, ",\"thumbnail\":null,\"thumbnailSource\":null");
        }
        free(b64);
        free(img);
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

    /* lote 19: room for the still-encoded name ("%20" takes 3 characters); sd_name_ok() checks the
     * decoded length (< 64). With 128/64, a long name with spaces gave 400. */
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char filename[200];
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
        bool ok = true, by_job = false;
        while (true) {
            /* lote 19: the 409 above is only checked at the start; a print (or its preparation) that starts
             * during a long read stops it here, so the two never compete for the SD card */
            if (printer_comm_job_active() || printer_comm_preparing(NULL, 0, NULL)) { ok = false; by_job = true; break; }
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
                     ok ? "" : by_job ? " (stopped: a print started)" : " (interrupted)", hex);
        } else if (by_job) {
            httpd_resp_set_status(req, "409 Conflict");
            httpd_resp_set_type(req, "text/plain");
            httpd_resp_sendstr(req, "Stopped: a print started");
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

/* ---- POST /sd/preview?filename=&size= — Ender Smart (lote 19) ----
 * Body: a PNG (or JPEG) the page drew from the G-code. Kept on LittleFS (gcode_preview.c), never
 * on the SD card, and returned by /sd/info while the file has this size. */

typedef struct {
    httpd_req_t *req;
    char filename[64];
    long size;
} sd_preview_ctx_t;

/* One preview POST at a time: each holds 6 KB of internal RAM and up to 160 KB of PSRAM. */
static atomic_bool s_preview_busy;

static bool preview_job_busy(void)
{
    return printer_comm_job_active() || printer_comm_preparing(NULL, 0, NULL);
}

static void sd_preview_task(void *arg)
{
    sd_preview_ctx_t *ctx = (sd_preview_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    int total = (int)req->content_len;
    int got = 0;
    const char *status = "200 OK", *msg = NULL;
    char json[64] = "";
    uint8_t *img = heap_caps_malloc(total, MALLOC_CAP_SPIRAM);

    if (!img) {
        status = "500 Internal Server Error"; msg = "Out of memory";
    } else {
        int64_t last_data = esp_timer_get_time();
        while (got < total) {
            int r = httpd_req_recv(req, (char *)img + got, total - got);
            if (r == HTTPD_SOCK_ERR_TIMEOUT && esp_timer_get_time() - last_data < 15 * 1000000LL) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            if (r <= 0) break;
            got += r;
            last_data = esp_timer_get_time();
        }
        char path[300];
        snprintf(path, sizeof(path), MOUNT_POINT "/%s", ctx->filename);
        struct stat st;
        if (got < total) {
            status = "400 Bad Request"; msg = "Incomplete image";
        } else if (!gcode_preview_mime(img, total)) {
            status = "400 Bad Request"; msg = "Not a PNG or JPEG, or larger than 1024 px";
        } else if (preview_job_busy()) {
            /* checked again: a print may have started while the image arrived; no flash write then */
            status = "409 Conflict"; msg = "A impressora começou a imprimir ou a preparar um arquivo";
        } else if (stat(path, &st) != 0) {
            /* the file may have been deleted or replaced while the page was drawing */
            status = "404 Not Found"; msg = "File not found";
        } else if ((long)st.st_size != ctx->size) {
            status = "409 Conflict"; msg = "The file changed while the preview was drawn";
        } else if (gcode_preview_save(ctx->filename, ctx->size, img, (size_t)total) != ESP_OK) {
            status = "500 Internal Server Error"; msg = "Could not save the preview";
        } else {
            snprintf(json, sizeof(json), "{\"ok\":true,\"bytes\":%d}", total);
        }
    }
    ESP_LOGI(TAG, "Preview POST %s: %s (%d of %d bytes, stack_free %u)", ctx->filename, status, got, total,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    free(img);

    atomic_store(&s_preview_busy, false);   /* before answering: the page sends the next one right away */
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, msg ? "text/plain" : "application/json");
    httpd_resp_sendstr(req, msg ? msg : json);

    httpd_req_async_handler_complete(req);
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t sd_preview_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted())
        return upload_refuse(req, "500 Internal Server Error", "SD card not mounted");

    /* room for the still-encoded name; sd_name_ok() checks the decoded length (< 64) */
    char query[320], filename[200], size_s[24];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "filename", filename, sizeof(filename)) != ESP_OK ||
        httpd_query_key_value(query, "size", size_s, sizeof(size_s)) != ESP_OK)
        return upload_refuse(req, "400 Bad Request", "Missing filename or size");
    url_decode_inplace(filename);
    if (!sd_name_ok(filename))
        return upload_refuse(req, "400 Bad Request", "Invalid filename");
    char *end = NULL;
    long size = strtol(size_s, &end, 10);
    if (!end || *end || size <= 0)
        return upload_refuse(req, "400 Bad Request", "Invalid size");
    if (req->content_len == 0 || req->content_len > GCODE_PREVIEW_MAX_BYTES)
        return upload_refuse(req, "413 Payload Too Large", "The preview must be 1 byte to 160 KB");
    /* the same rule as the other heavy file operations: nothing while a print runs or is prepared */
    if (preview_job_busy())
        return upload_refuse(req, "409 Conflict", "A impressora está imprimindo ou preparando um arquivo");
    bool idle = false;
    if (!atomic_compare_exchange_strong(&s_preview_busy, &idle, true))
        return upload_refuse(req, "409 Conflict", "Another preview is being saved");

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        atomic_store(&s_preview_busy, false);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }
    sd_preview_ctx_t *ctx = malloc(sizeof(sd_preview_ctx_t));
    if (!ctx) {
        atomic_store(&s_preview_busy, false);
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));
    ctx->size = size;
    /* 6 KB: LittleFS writes and several ESP_LOG calls (see the lote 18 panics with 4 KB) */
    if (xTaskCreatePinnedToCore(sd_preview_task, "sd_prev", 6144, ctx, 5, NULL, 0) != pdPASS) {
        atomic_store(&s_preview_busy, false);
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
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
        { .uri = "/sd/prepare/cancel", .method = HTTP_POST, .handler = sd_prepare_cancel_handler },
        { .uri = "/sd/ack-error", .method = HTTP_POST, .handler = sd_ack_error_handler },
        { .uri = "/sd/info",   .method = HTTP_GET,  .handler = sd_info_handler },
        { .uri = "/sd/status", .method = HTTP_GET,  .handler = sd_status_handler },
        { .uri = "/sd/sha256", .method = HTTP_GET,  .handler = sd_sha256_handler },
        { .uri = "/sd/download", .method = HTTP_GET, .handler = sd_download_handler },
        { .uri = "/sd/preview", .method = HTTP_POST, .handler = sd_preview_handler },   /* lote 19 */
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }

    /* lote 16 (F3-03): the same handlers under /api/v1; the addresses above stay */
    const httpd_uri_t v1[] = {
        { .uri = "/api/v1/files",          .method = HTTP_GET,  .handler = sd_files_handler },
        { .uri = "/api/v1/files/info",     .method = HTTP_GET,  .handler = sd_info_handler },
        { .uri = "/api/v1/files/sha256",   .method = HTTP_GET,  .handler = sd_sha256_handler },
        { .uri = "/api/v1/files/download", .method = HTTP_GET,  .handler = sd_download_handler },
        { .uri = "/api/v1/files/preview",  .method = HTTP_POST, .handler = sd_preview_handler },
        { .uri = "/api/v1/files/upload",   .method = HTTP_POST, .handler = sd_upload_handler },
        { .uri = "/api/v1/files/delete",   .method = HTTP_POST, .handler = sd_delete_handler },
        { .uri = "/api/v1/job",            .method = HTTP_GET,  .handler = sd_status_handler },
        { .uri = "/api/v1/job/start",      .method = HTTP_POST, .handler = sd_print_handler },
        { .uri = "/api/v1/job/pause",      .method = HTTP_POST, .handler = sd_pause_handler },
        { .uri = "/api/v1/job/resume",     .method = HTTP_POST, .handler = sd_resume_handler },
        { .uri = "/api/v1/job/cancel",     .method = HTTP_POST, .handler = sd_cancel_handler },
        { .uri = "/api/v1/job/prepare_cancel", .method = HTTP_POST, .handler = sd_prepare_cancel_handler },
        { .uri = "/api/v1/job/ack_error",  .method = HTTP_POST, .handler = sd_ack_error_handler },
    };
    for (int i = 0; i < sizeof(v1) / sizeof(v1[0]); i++) {
        HTTPD_REGISTER(server, &v1[i]);
    }

    ESP_LOGI(TAG, "SD card HTTP endpoints registered");
    return ESP_OK;
}
