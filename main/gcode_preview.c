/*
 * Ender Smart (lote 19): previews of G-code files that carry no thumbnail. See gcode_preview.h.
 *
 * One file per preview in /cache/prev, named by a hash of the SD file name (LittleFS names are
 * limited to 64 characters, and SD names can have 63). The header keeps the full name and the SD
 * file size: a hash collision or a replaced file just reads as "no preview".
 *
 * The directory is kept under PREVIEW_MAX_BYTES and PREVIEW_MAX_ENTRIES (oldest first, by mtime,
 * which a load refreshes), so it never takes the space the job history and the mesh history need
 * on the same partition.
 */
#include "gcode_preview.h"
#include "cache.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "gcode_preview";

#define PREVIEW_DIR         "/cache/prev"
#define PREVIEW_MAGIC       0x31575650u   /* "PVW1" */
#define PREVIEW_MAX_ENTRIES 60
#define PREVIEW_MAX_BYTES   (3 * 1024 * 1024)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    int64_t  file_size;
    char     name[64];
    uint32_t len;
} preview_hdr_t;

/* One lock for every access: a rename over a file another task has open fails (EBUSY) on LittleFS.
 * NULL only if there was no RAM for it; then every call does nothing (no preview), never asserts. */
static SemaphoreHandle_t lock(void)
{
    static SemaphoreHandle_t s_lock;
    static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
    if (!s_lock) {
        SemaphoreHandle_t m = xSemaphoreCreateMutex();
        if (!m) return NULL;
        taskENTER_CRITICAL(&s_mux);
        if (!s_lock) {
            s_lock = m;
            m = NULL;
        }
        taskEXIT_CRITICAL(&s_mux);
        if (m) vSemaphoreDelete(m);
    }
    return s_lock;
}

static bool take(TickType_t wait)
{
    SemaphoreHandle_t m = lock();
    return m && xSemaphoreTake(m, wait) == pdTRUE;
}

static void give(void)
{
    xSemaphoreGive(lock());
}

static void preview_path(const char *name, char *out, size_t out_sz)
{
    uint32_t h = 2166136261u;   /* FNV-1a */
    for (const unsigned char *p = (const unsigned char *)name; *p; p++) {
        h ^= *p;
        h *= 16777619u;
    }
    snprintf(out, out_sz, PREVIEW_DIR "/%08lx.prv", (unsigned long)h);
}

/* Under the lock: removes the oldest previews until both limits hold. */
static void evict(void)
{
    for (int round = 0; round < PREVIEW_MAX_ENTRIES + 8; round++) {
        DIR *d = opendir(PREVIEW_DIR);
        if (!d) return;
        int count = 0;
        long bytes = 0;
        time_t oldest_t = 0;
        char oldest[64] = "";   /* our own names are "xxxxxxxx.prv" */
        char p[300];
        struct dirent *e;
        while ((e = readdir(d)) != NULL) {
            if (e->d_type != DT_REG) continue;
            snprintf(p, sizeof(p), PREVIEW_DIR "/%s", e->d_name);
            struct stat st;
            if (stat(p, &st) != 0) continue;
            count++;
            bytes += (long)st.st_size;
            if (!oldest[0] || st.st_mtime < oldest_t) {
                oldest_t = st.st_mtime;
                strlcpy(oldest, e->d_name, sizeof(oldest));
            }
        }
        closedir(d);
        if ((count <= PREVIEW_MAX_ENTRIES && bytes <= PREVIEW_MAX_BYTES) || !oldest[0]) return;
        snprintf(p, sizeof(p), PREVIEW_DIR "/%s", oldest);
        ESP_LOGI(TAG, "Evicting %s (%d previews, %ld bytes)", p, count, bytes);
        unlink(p);
    }
}

const char *gcode_preview_mime(const uint8_t *img, size_t len)
{
    if (!img || len < 24 || len > GCODE_PREVIEW_MAX_BYTES) return NULL;
    static const uint8_t png_sig[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
    if (memcmp(img, png_sig, 8) == 0 && memcmp(img + 12, "IHDR", 4) == 0) {
        uint32_t w = ((uint32_t)img[16] << 24) | ((uint32_t)img[17] << 16) | ((uint32_t)img[18] << 8) | img[19];
        uint32_t h = ((uint32_t)img[20] << 24) | ((uint32_t)img[21] << 16) | ((uint32_t)img[22] << 8) | img[23];
        if (w == 0 || h == 0 || w > GCODE_PREVIEW_MAX_SIDE || h > GCODE_PREVIEW_MAX_SIDE) return NULL;
        return "image/png";
    }
    if (img[0] == 0xff && img[1] == 0xd8 && img[2] == 0xff) return "image/jpeg";
    return NULL;
}

esp_err_t gcode_preview_save(const char *name, long file_size, const uint8_t *img, size_t len)
{
    if (!name || !name[0] || strlen(name) >= sizeof(((preview_hdr_t *)0)->name) || !gcode_preview_mime(img, len))
        return ESP_ERR_INVALID_ARG;

    char path[40], tmp[48];
    preview_path(name, path, sizeof(path));
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    preview_hdr_t hdr = { .magic = PREVIEW_MAGIC, .file_size = file_size, .len = (uint32_t)len };
    strlcpy(hdr.name, name, sizeof(hdr.name));

    if (!take(pdMS_TO_TICKS(5000))) return ESP_ERR_TIMEOUT;
    esp_err_t ret = ESP_FAIL;
    mkdir(PREVIEW_DIR, 0755);
    FILE *f = fopen(tmp, "wb");
    if (f) {
        bool ok = fwrite(&hdr, sizeof(hdr), 1, f) == 1 && fwrite(img, 1, len, f) == len;
        if (fclose(f) != 0) ok = false;
        if (ok && rename(tmp, path) == 0) {   /* LittleFS rename replaces the old preview */
            ret = ESP_OK;
            evict();
        } else {
            unlink(tmp);
        }
    }
    give();

    if (ret == ESP_OK) ESP_LOGI(TAG, "Preview of %s saved (%u bytes)", name, (unsigned)len);
    else ESP_LOGW(TAG, "Preview of %s not saved", name);
    return ret;
}

uint8_t *gcode_preview_load(const char *name, long file_size, size_t *len)
{
    *len = 0;
    if (!name || !name[0]) return NULL;
    char path[40];
    preview_path(name, path, sizeof(path));

    if (!take(pdMS_TO_TICKS(3000))) return NULL;   /* no preview this time, never a stuck /sd/info */
    uint8_t *img = NULL;
    FILE *f = fopen(path, "rb");
    if (f) {
        preview_hdr_t hdr;
        if (fread(&hdr, sizeof(hdr), 1, f) == 1 && hdr.magic == PREVIEW_MAGIC && hdr.file_size == (int64_t)file_size &&
            strncmp(hdr.name, name, sizeof(hdr.name)) == 0 && hdr.len > 0 && hdr.len <= GCODE_PREVIEW_MAX_BYTES) {
            img = heap_caps_malloc(hdr.len, MALLOC_CAP_SPIRAM);
            if (img && fread(img, 1, hdr.len, f) == hdr.len && gcode_preview_mime(img, hdr.len)) {
                *len = hdr.len;
            } else {
                free(img);
                img = NULL;
            }
        }
        fclose(f);
        if (img) cache_touch(path);   /* most recently used: eviction keeps it */
    }
    give();
    return img;
}

void gcode_preview_delete(const char *name)
{
    if (!name || !name[0]) return;
    char path[40];
    preview_path(name, path, sizeof(path));

    /* Called from the HTTP server task (/sd/delete): a short wait, never the whole server held behind
     * a preview being written. If it times out, the stale preview stays bound to the old file size. */
    if (!take(pdMS_TO_TICKS(200))) {
        ESP_LOGW(TAG, "Preview of %s not removed (busy)", name);
        return;
    }
    FILE *f = fopen(path, "rb");
    bool mine = false;
    if (f) {
        preview_hdr_t hdr;
        mine = fread(&hdr, sizeof(hdr), 1, f) == 1 && hdr.magic == PREVIEW_MAGIC &&
               strncmp(hdr.name, name, sizeof(hdr.name)) == 0;
        fclose(f);
    }
    if (mine && unlink(path) == 0) ESP_LOGI(TAG, "Preview of %s removed", name);
    give();
}
