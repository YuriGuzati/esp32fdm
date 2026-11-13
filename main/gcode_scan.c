#include "gcode_scan.h"
#include "cache.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "gcode_scan";

/* ---- Quick scan (used by host print start) ---- */

/** Ender Smart (lote 7): value of one axis word ("X12.5") of a G-code line, comments ignored. */
static bool gc_axis(const char *line, char axis, float *out)
{
    /* the letter starts a word when it follows a space, a tab or the previous word's number
     * ("G1X10Y20E1" is valid G-code too) */
    for (const char *p = line; *p && *p != ';'; p++) {
        if ((*p == axis || *p == axis + 32) && (p == line || !isalpha((unsigned char)p[-1]))) {
            char *end;
            float v = strtof(p + 1, &end);
            if (end == p + 1) return false;
            *out = v;
            return true;
        }
    }
    return false;
}

typedef struct { float x0, x1, y0, y1; bool any; } gc_box_t;

static void gc_box_add(gc_box_t *b, float x, float y)
{
    if (!b->any) { b->x0 = b->x1 = x; b->y0 = b->y1 = y; b->any = true; return; }
    if (x < b->x0) b->x0 = x;
    if (x > b->x1) b->x1 = x;
    if (y < b->y0) b->y0 = y;
    if (y > b->y1) b->y1 = y;
}

void gcode_scan_file(FILE *f, gcode_scan_t *out)
{
    memset(out, 0, sizeof(*out));
    out->total_layers = -1;

    char buf[256];
    int32_t max_layer = -1;
    int32_t layer_change_count = 0;
    float first_z = 0, prev_z = 0;
    bool seen_z = false;
    /* lote 7: part extent */
    gc_box_t all = {0}, part = {0};
    float cur_x = 0, cur_y = 0;
    bool x_known = false, y_known = false, rel_xy = false, after_marker = false, area_bad = false;

    rewind(f);
    while (fgets(buf, sizeof(buf), f)) {
        out->total_lines++;

        /* Slicer metadata comments */
        if (buf[0] == ';') {
            /* PrusaSlicer/SuperSlicer/OrcaSlicer: ; layer_height = 0.2 */
            const char *lh = strstr(buf, "layer_height");
            if (lh) {
                const char *eq = strchr(lh, '=');
                if (eq) {
                    float v = strtof(eq + 1, NULL);
                    if (v > 0.01f && v < 1.0f) {
                        /* "first_layer_height" vs "layer_height" */
                        if (strstr(buf, "first_layer_height"))
                            out->first_layer_height = v;
                        else
                            out->layer_height = v;
                    }
                }
            }
            /* ;LAYER:N — track highest layer number */
            if (strncmp(buf, ";LAYER:", 7) == 0) {
                int32_t n = atoi(buf + 7);
                if (n > max_layer) max_layer = n;
                after_marker = true;
            }
            /* OrcaSlicer: ;LAYER_CHANGE */
            if (strncmp(buf, ";LAYER_CHANGE", 13) == 0) {
                layer_change_count++;
                after_marker = true;
            }
            /* Cura: ;Layer height: 0.2 */
            const char *clh = strstr(buf, ";Layer height:");
            if (clh) {
                float v = strtof(clh + 14, NULL);
                if (v > 0.01f && v < 1.0f) out->layer_height = v;
            }
            continue;
        }

        /* lote 7: XY extent of the extrusion moves (absolute XY only) */
        if (buf[0] == 'G' || buf[0] == 'g') {
            int gc = atoi(buf + 1);
            float v;
            if (gc == 91) {
                rel_xy = true;
            } else if (gc == 90) {
                rel_xy = false;
            } else if (gc == 28) {
                x_known = y_known = false;          /* at the endstops: coordinates unknown */
            } else if (gc == 92) {
                /* G92 X/Y shifts the coordinates: the area would no longer be in the bed's
                 * frame, and "X1.5" would not be the physical left edge */
                if (gc_axis(buf, 'X', &v) || gc_axis(buf, 'Y', &v)) area_bad = true;
            } else if (gc == 2 || gc == 3) {
                /* arcs (arc fitting, ArcWelder) bulge beyond their end points: not tracked */
                if (gc_axis(buf, 'E', &v) || gc_axis(buf, 'X', &v) || gc_axis(buf, 'Y', &v))
                    area_bad = true;
            } else if (gc == 0 || gc == 1) {
                float vx = 0, vy = 0, e = 0;
                bool hx = gc_axis(buf, 'X', &vx), hy = gc_axis(buf, 'Y', &vy);
                bool he = gc_axis(buf, 'E', &e);
                if (hx || hy) {
                    /* Relative XY (G91) adds to the current position: Cura's end G-code
                     * does "G91 / G1 X5 Y5". An unknown position stays unknown. */
                    float nx = hx ? (rel_xy ? cur_x + vx : vx) : cur_x;
                    float ny = hy ? (rel_xy ? cur_y + vy : vy) : cur_y;
                    bool nx_known = rel_xy ? x_known : (x_known || hx);
                    bool ny_known = rel_xy ? y_known : (y_known || hy);
                    if (he && e > 0.0f && nx_known && ny_known) {
                        gc_box_add(&all, nx, ny);
                        if (after_marker) gc_box_add(&part, nx, ny);
                        if (x_known && y_known) {
                            gc_box_add(&all, cur_x, cur_y);
                            if (after_marker) gc_box_add(&part, cur_x, cur_y);
                        }
                    } else if (he && e > 0.0f) {
                        area_bad = true;            /* extrusion from an unknown position */
                    }
                    cur_x = nx; cur_y = ny;
                    x_known = nx_known; y_known = ny_known;
                }
            }
        }

        /* Track Z from G0/G1 moves */
        if (buf[0] == 'G' && (buf[1] == '0' || buf[1] == '1') && buf[2] == ' ') {
            const char *zp = strstr(buf, "Z");
            if (zp && (zp == buf + 3 || *(zp - 1) == ' ')) {
                float z = strtof(zp + 1, NULL);
                if (z > 0.01f) {
                    if (!seen_z) {
                        first_z = z;
                        seen_z = true;
                    }
                    if (z > out->max_z) out->max_z = z;
                    prev_z = z;
                }
            }
        }
    }
    rewind(f);

    /* Derive values if not found in comments */
    out->has_layer_markers = (max_layer >= 0 || layer_change_count > 0);
    if (max_layer >= 0)
        out->total_layers = max_layer + 1;  /* ;LAYER: is 0-based */
    else if (layer_change_count > 0)
        out->total_layers = layer_change_count;

    if (out->first_layer_height == 0 && first_z > 0)
        out->first_layer_height = first_z;

    if (out->layer_height == 0 && out->total_layers > 1 && out->max_z > 0) {
        /* Estimate: (maxZ - first_layer) / (layers - 1) */
        float flh = out->first_layer_height > 0 ? out->first_layer_height : first_z;
        out->layer_height = (out->max_z - flh) / (float)(out->total_layers - 1);
    }

    /* If still no total_layers, estimate from Z and layer height */
    if (out->total_layers <= 0 && out->max_z > 0 && out->layer_height > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : out->layer_height;
        out->total_layers = 1 + (int32_t)((out->max_z - flh) / out->layer_height + 0.5f);
    }

    (void)prev_z;
    ESP_LOGI(TAG, "GCode scan: %ld lines, maxZ=%.2f, layers=%ld, lh=%.2f, flh=%.2f",
             (long)out->total_lines, out->max_z, (long)out->total_layers,
             out->layer_height, out->first_layer_height);

    /* lote 7: part extent. With layer markers, only the moves after the first one (the start
     * purge lines come before it); without markers, every extrusion (purge lines included,
     * which only makes the area bigger). Never with relative XY. */
    const gc_box_t *b = out->has_layer_markers ? &part : &all;
    if (!area_bad && b->any) {
        out->has_part_area = true;
        out->part_x0 = b->x0; out->part_x1 = b->x1;
        out->part_y0 = b->y0; out->part_y1 = b->y1;
        ESP_LOGI(TAG, "Part area: X%.1f..%.1f Y%.1f..%.1f", b->x0, b->x1, b->y0, b->y1);
    } else {
        ESP_LOGI(TAG, "Part area unknown (%s)", area_bad ? "extrusion from an unknown position" : "no extrusion moves");
    }
}

/* ---- GCode scan cache (/cache/gcode/) ---- */

#define GCODE_CACHE_DIR  "/cache/gcode"
#define GCODE_CACHE_MAGIC 0x47434301  /* "GCC\x01" */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    long     file_size;       /* cache key */
    int32_t  total_layers;
    float    layer_height;
    float    first_layer_height;
    float    max_z;
    int32_t  est_time_s;
    float    filament_used_mm;
    float    filament_used_g;
    int32_t  thumb_w, thumb_h;
    uint32_t thumb_len;       /* 0 = no thumbnail, else base64 bytes follow */
} gcode_cache_hdr_t;

/* Build cache path from source filename (basename only). */
static void gcode_cache_path(const char *src_path, char *out, size_t out_sz)
{
    const char *base = strrchr(src_path, '/');
    base = base ? base + 1 : src_path;
    snprintf(out, out_sz, GCODE_CACHE_DIR "/%s.cache", base);
}

/* Try to load cached scan result. Returns true on hit. */
static bool gcode_cache_load(const char *src_path, long file_size,
                             gcode_file_info_t *out)
{
    char cpath[160];
    gcode_cache_path(src_path, cpath, sizeof(cpath));

    FILE *f = fopen(cpath, "rb");
    if (!f) return false;

    gcode_cache_hdr_t hdr;
    bool ok = false;
    if (fread(&hdr, sizeof(hdr), 1, f) == 1 &&
        hdr.magic == GCODE_CACHE_MAGIC &&
        hdr.file_size == file_size) {
        out->total_layers      = hdr.total_layers;
        out->layer_height      = hdr.layer_height;
        out->first_layer_height = hdr.first_layer_height;
        out->max_z             = hdr.max_z;
        out->est_time_s        = hdr.est_time_s;
        out->filament_used_mm  = hdr.filament_used_mm;
        out->filament_used_g   = hdr.filament_used_g;
        out->thumb_w           = hdr.thumb_w;
        out->thumb_h           = hdr.thumb_h;
        out->thumbnail_base64  = NULL;
        if (hdr.thumb_len > 0 && hdr.thumb_len < 200000) {
            char *tb = malloc(hdr.thumb_len + 1);
            if (tb && fread(tb, 1, hdr.thumb_len, f) == hdr.thumb_len) {
                tb[hdr.thumb_len] = '\0';
                out->thumbnail_base64 = tb;
            } else {
                free(tb);
            }
        }
        ok = true;
        ESP_LOGI(TAG, "GCode cache hit: %s", cpath);
    }
    fclose(f);
    if (ok) cache_touch(cpath);
    return ok;
}

#define GCODE_CACHE_MAX_ENTRIES 50

/* Save scan result to cache. */
static void gcode_cache_save(const char *src_path, long file_size,
                             const gcode_file_info_t *info)
{
    /* Ensure cache directory exists */
    mkdir(GCODE_CACHE_DIR, 0755);

    char cpath[160];
    gcode_cache_path(src_path, cpath, sizeof(cpath));

    FILE *f = fopen(cpath, "wb");
    if (!f) {
        ESP_LOGW(TAG, "GCode cache write failed: %s", cpath);
        return;
    }

    gcode_cache_hdr_t hdr = {
        .magic              = GCODE_CACHE_MAGIC,
        .file_size          = file_size,
        .total_layers       = info->total_layers,
        .layer_height       = info->layer_height,
        .first_layer_height = info->first_layer_height,
        .max_z              = info->max_z,
        .est_time_s         = info->est_time_s,
        .filament_used_mm   = info->filament_used_mm,
        .filament_used_g    = info->filament_used_g,
        .thumb_w            = info->thumb_w,
        .thumb_h            = info->thumb_h,
        .thumb_len          = info->thumbnail_base64 ? (uint32_t)strlen(info->thumbnail_base64) : 0,
    };
    fwrite(&hdr, sizeof(hdr), 1, f);
    if (hdr.thumb_len > 0)
        fwrite(info->thumbnail_base64, 1, hdr.thumb_len, f);
    fclose(f);
    ESP_LOGI(TAG, "GCode cache saved: %s (%u bytes)", cpath,
             (unsigned)(sizeof(hdr) + hdr.thumb_len));

    cache_evict_lru(GCODE_CACHE_DIR, GCODE_CACHE_MAX_ENTRIES);
}

/* ---- Public file info scan (head + tail for speed) ---- */

void gcode_scan_file_info(const char *path, gcode_file_info_t *out)
{
    memset(out, 0, sizeof(*out));
    out->total_layers = -1;
    out->est_time_s = -1;
    out->filament_used_mm = -1;
    out->filament_used_g = -1;

    struct stat st;
    if (stat(path, &st) != 0) return;
    long file_size = (long)st.st_size;

    /* Check cache first */
    if (gcode_cache_load(path, file_size, out))
        return;

    FILE *f = fopen(path, "r");
    if (!f) return;

    char buf[256];
    int32_t max_layer = -1;
    int32_t layer_change_count = 0;
    float first_z = 0, prev_z = 0;
    bool seen_z = false;

    /* Thumbnail state */
    bool in_thumbnail = false;
    int best_thumb_pixels = 0;
    int cur_thumb_w = 0, cur_thumb_h = 0;
    size_t thumb_alloc = 0, thumb_len = 0;
    char *thumb_buf = NULL;       /* accumulates current thumbnail */
    char *best_thumb = NULL;      /* best (largest) thumbnail found */
    int best_thumb_w = 0, best_thumb_h = 0;
    #define MAX_THUMB_DECODED 40960  /* 40KB cap */

    /* Parse a block of lines for metadata */
    #define PARSE_LINE() do { \
        if (buf[0] == ';') { \
            /* Layer height */ \
            const char *lh = strstr(buf, "layer_height"); \
            if (lh) { \
                const char *eq = strchr(lh, '='); \
                if (eq) { \
                    float v = strtof(eq + 1, NULL); \
                    if (v > 0.01f && v < 1.0f) { \
                        if (strstr(buf, "first_layer_height")) \
                            out->first_layer_height = v; \
                        else \
                            out->layer_height = v; \
                    } \
                } \
            } \
            /* Cura: ;Layer height: 0.2 */ \
            const char *clh = strstr(buf, ";Layer height:"); \
            if (clh) { \
                float v = strtof(clh + 14, NULL); \
                if (v > 0.01f && v < 1.0f) out->layer_height = v; \
            } \
            /* ;LAYER:N */ \
            if (strncmp(buf, ";LAYER:", 7) == 0) { \
                int32_t n = atoi(buf + 7); \
                if (n > max_layer) max_layer = n; \
            } \
            if (strncmp(buf, ";LAYER_CHANGE", 13) == 0) \
                layer_change_count++; \
            /* Print time: PrusaSlicer/OrcaSlicer */ \
            if (out->est_time_s < 0) { \
                const char *etp = strstr(buf, "estimated printing time"); \
                if (etp) { \
                    const char *eq = strchr(etp, '='); \
                    if (eq) { \
                        eq++; \
                        int h = 0, m = 0, s = 0; \
                        const char *p = eq; \
                        while (*p) { \
                            if (*p >= '0' && *p <= '9') { \
                                int val = atoi(p); \
                                while (*p >= '0' && *p <= '9') p++; \
                                if (*p == 'h') h = val; \
                                else if (*p == 'm') m = val; \
                                else if (*p == 's') s = val; \
                            } \
                            if (*p) p++; \
                        } \
                        out->est_time_s = h * 3600 + m * 60 + s; \
                    } \
                } \
            } \
            /* Print time: Cura ;TIME:5025 */ \
            if (out->est_time_s < 0 && strncmp(buf, ";TIME:", 6) == 0) { \
                int t = atoi(buf + 6); \
                if (t > 0) out->est_time_s = t; \
            } \
            /* Filament: PrusaSlicer */ \
            if (out->filament_used_mm < 0) { \
                const char *fm = strstr(buf, "filament used [mm]"); \
                if (fm) { \
                    const char *eq = strchr(fm, '='); \
                    if (eq) out->filament_used_mm = strtof(eq + 1, NULL); \
                } \
            } \
            if (out->filament_used_g < 0) { \
                const char *fg = strstr(buf, "filament used [g]"); \
                if (fg) { \
                    const char *eq = strchr(fg, '='); \
                    if (eq) out->filament_used_g = strtof(eq + 1, NULL); \
                } \
            } \
            /* Filament: Cura ;Filament used: 1.234m */ \
            if (out->filament_used_mm < 0) { \
                const char *fc = strstr(buf, ";Filament used:"); \
                if (fc) { \
                    float v = strtof(fc + 15, NULL); \
                    if (v > 0) out->filament_used_mm = v * 1000.0f; \
                } \
            } \
            /* Thumbnail parsing */ \
            if (strstr(buf, "; thumbnail begin")) { \
                int tw = 0, th = 0, tsz = 0; \
                if (sscanf(buf, "; thumbnail begin %dx%d %d", &tw, &th, &tsz) >= 2) { \
                    in_thumbnail = true; \
                    cur_thumb_w = tw; \
                    cur_thumb_h = th; \
                    /* Estimate base64 size (4/3 * decoded + padding) */ \
                    size_t est = (tsz > 0 ? (size_t)(tsz * 4 / 3 + 100) : 8192); \
                    if (est > MAX_THUMB_DECODED * 4 / 3 + 100) est = MAX_THUMB_DECODED * 4 / 3 + 100; \
                    free(thumb_buf); \
                    thumb_buf = malloc(est + 1); \
                    thumb_alloc = thumb_buf ? est : 0; \
                    thumb_len = 0; \
                } \
            } else if (in_thumbnail && strstr(buf, "; thumbnail end")) { \
                in_thumbnail = false; \
                if (thumb_buf && thumb_len > 0) { \
                    int pixels = cur_thumb_w * cur_thumb_h; \
                    if (pixels > best_thumb_pixels) { \
                        best_thumb_pixels = pixels; \
                        free(best_thumb); \
                        thumb_buf[thumb_len] = '\0'; \
                        best_thumb = thumb_buf; \
                        thumb_buf = NULL; \
                        best_thumb_w = cur_thumb_w; \
                        best_thumb_h = cur_thumb_h; \
                    } else { \
                        free(thumb_buf); \
                        thumb_buf = NULL; \
                    } \
                } \
            } else if (in_thumbnail && thumb_buf) { \
                /* Strip "; " prefix and append base64 data */ \
                const char *data = buf; \
                if (data[0] == ';' && data[1] == ' ') data += 2; \
                else if (data[0] == ';') data += 1; \
                size_t dlen = strlen(data); \
                /* Trim trailing whitespace */ \
                while (dlen > 0 && (data[dlen-1] == '\n' || data[dlen-1] == '\r' || data[dlen-1] == ' ')) \
                    dlen--; \
                if (thumb_len + dlen < thumb_alloc) { \
                    memcpy(thumb_buf + thumb_len, data, dlen); \
                    thumb_len += dlen; \
                } \
            } \
        } else if (buf[0] == 'G' && (buf[1] == '0' || buf[1] == '1') && buf[2] == ' ') { \
            const char *zp = strstr(buf, "Z"); \
            if (zp && (zp == buf + 3 || *(zp - 1) == ' ')) { \
                float z = strtof(zp + 1, NULL); \
                if (z > 0.01f) { \
                    if (!seen_z) { first_z = z; seen_z = true; } \
                    if (z > out->max_z) out->max_z = z; \
                    prev_z = z; \
                } \
            } \
        } \
    } while(0)

    /* Scan head: first 500 lines for metadata, thumbnails, time, filament */
    int head_lines = 0;
    while (head_lines < 500 && fgets(buf, sizeof(buf), f)) {
        head_lines++;
        PARSE_LINE();
    }

    long middle_start = ftell(f);

    /* Scan tail: last ~32KB for slicer summary comments (time, filament, etc.) */
    if (file_size > 32768) {
        long tail_offset = file_size - 32768;
        if (tail_offset > middle_start) {
            fseek(f, tail_offset, SEEK_SET);
            fgets(buf, sizeof(buf), f);  /* skip partial line */
            while (fgets(buf, sizeof(buf), f)) {
                PARSE_LINE();
            }
        } else {
            /* File small enough that head+tail overlap, scan remainder */
            while (fgets(buf, sizeof(buf), f)) {
                PARSE_LINE();
            }
        }
    } else {
        while (fgets(buf, sizeof(buf), f)) {
            PARSE_LINE();
        }
    }

    #undef PARSE_LINE
    #undef MAX_THUMB_DECODED

    free(thumb_buf);

    /* Fast bulk scan of the middle section for layers and Z moves only.
     * Uses large fread() chunks instead of per-line fgets() for SD card speed. */
    long tail_start = (file_size > 32768) ? (file_size - 32768) : file_size;
    if (middle_start < tail_start) {
        fseek(f, middle_start, SEEK_SET);
        /* Use internal DMA-capable RAM — SDMMC DMA on ESP32-S3 can't access PSRAM,
         * falls back to 512-byte bounce buffer per sector if buffer is in PSRAM */
        #define SCAN_BUF_SIZE 4096
        char *sbuf = heap_caps_malloc(SCAN_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!sbuf) sbuf = malloc(SCAN_BUF_SIZE);  /* PSRAM fallback */
        if (sbuf) {
            long bytes_left = tail_start - middle_start;
            /* Carry-over: partial line from end of previous chunk */
            char carry[128];
            int carry_len = 0;

            while (bytes_left > 0) {
                int to_read = bytes_left < SCAN_BUF_SIZE ? (int)bytes_left : SCAN_BUF_SIZE;
                int got = fread(sbuf, 1, to_read, f);
                if (got <= 0) break;
                bytes_left -= got;

                char *p = sbuf;
                char *end = sbuf + got;

                /* If we have carry-over, find end of that line first */
                if (carry_len > 0) {
                    char *nl = memchr(p, '\n', end - p);
                    int copy = nl ? (int)(nl - p) : (int)(end - p);
                    if (carry_len + copy < (int)sizeof(carry) - 1) {
                        memcpy(carry + carry_len, p, copy);
                        carry_len += copy;
                    }
                    if (nl) {
                        carry[carry_len] = '\0';
                        /* Process carried-over line */
                        if (carry[0] == ';') {
                            if (strncmp(carry, ";LAYER:", 7) == 0) {
                                int32_t n = atoi(carry + 7);
                                if (n > max_layer) max_layer = n;
                            } else if (strncmp(carry, ";LAYER_CHANGE", 13) == 0) {
                                layer_change_count++;
                            }
                        } else if (carry[0] == 'G' && (carry[1] == '0' || carry[1] == '1') && carry[2] == ' ') {
                            const char *zp = strstr(carry, "Z");
                            if (zp && (zp == carry + 3 || *(zp - 1) == ' ')) {
                                float z = strtof(zp + 1, NULL);
                                if (z > 0.01f) {
                                    if (!seen_z) { first_z = z; seen_z = true; }
                                    if (z > out->max_z) out->max_z = z;
                                    prev_z = z;
                                }
                            }
                        }
                        p = nl + 1;
                        carry_len = 0;
                    } else {
                        /* Entire chunk was part of one long line, skip it */
                        carry_len = 0;
                        continue;
                    }
                }

                /* Process complete lines within this chunk */
                while (p < end) {
                    char *nl = memchr(p, '\n', end - p);
                    if (!nl) {
                        /* Save partial line as carry-over */
                        int rem = (int)(end - p);
                        if (rem < (int)sizeof(carry)) {
                            memcpy(carry, p, rem);
                            carry_len = rem;
                        }
                        break;
                    }

                    /* Quick first-char filter — skip lines that can't match */
                    if (*p == ';') {
                        if (nl - p >= 7 && strncmp(p, ";LAYER:", 7) == 0) {
                            int32_t n = atoi(p + 7);
                            if (n > max_layer) max_layer = n;
                        } else if (nl - p >= 13 && strncmp(p, ";LAYER_CHANGE", 13) == 0) {
                            layer_change_count++;
                        }
                    } else if (*p == 'G' && nl - p > 3 && (p[1] == '0' || p[1] == '1') && p[2] == ' ') {
                        /* Look for Z parameter — scan for ' Z' or 'G0 Z'/'G1 Z' */
                        const char *zp = p + 3;
                        while (zp < nl - 1) {
                            if (*zp == 'Z' && (zp == p + 3 || *(zp - 1) == ' ')) {
                                float z = strtof(zp + 1, NULL);
                                if (z > 0.01f) {
                                    if (!seen_z) { first_z = z; seen_z = true; }
                                    if (z > out->max_z) out->max_z = z;
                                    prev_z = z;
                                }
                                break;
                            }
                            zp++;
                        }
                    }

                    p = nl + 1;
                }

                taskYIELD();  /* Let other tasks run between chunks */
            }
            free(sbuf);
        }
        #undef SCAN_BUF_SIZE
    }

    fclose(f);

    /* Derive layers */
    if (max_layer >= 0)
        out->total_layers = max_layer + 1;
    else if (layer_change_count > 0)
        out->total_layers = layer_change_count;

    if (out->first_layer_height == 0 && first_z > 0)
        out->first_layer_height = first_z;

    if (out->layer_height == 0 && out->total_layers > 1 && out->max_z > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : first_z;
        out->layer_height = (out->max_z - flh) / (float)(out->total_layers - 1);
    }

    if (out->total_layers <= 0 && out->max_z > 0 && out->layer_height > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : out->layer_height;
        out->total_layers = 1 + (int32_t)((out->max_z - flh) / out->layer_height + 0.5f);
    }

    /* Assign thumbnail */
    if (best_thumb) {
        out->thumbnail_base64 = best_thumb;
        out->thumb_w = best_thumb_w;
        out->thumb_h = best_thumb_h;
    }

    (void)prev_z;
    ESP_LOGI(TAG, "File info: layers=%ld, maxZ=%.2f, time=%lds, fil=%.0fmm/%.1fg, thumb=%dx%d",
             (long)out->total_layers, out->max_z, (long)out->est_time_s,
             out->filament_used_mm, out->filament_used_g,
             out->thumb_w, out->thumb_h);

    /* Save to cache for next time */
    gcode_cache_save(path, file_size, out);
}
