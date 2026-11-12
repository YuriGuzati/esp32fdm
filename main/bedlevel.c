#include "bedlevel.h"
#include "httpd.h"
#include "printer_comm.h"
#include "cache.h"
#include "layout.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

static const char *TAG = "bedlevel";

/* ---- Constants ---- */
#define BL_MESH_MAX    10
#define BL_CACHE_DIR   "/cache/beds"
#define BL_CONFIG_FILE BL_CACHE_DIR "/config.json"
#define BL_CACHE_MAX   100
#define PROBE_TIMEOUT_MS 1200000  /* 20 min — covers up to 10x10 mesh */

/* ---- Data types ---- */

typedef struct {
    int64_t timestamp;
    int grid_x, grid_y;
    float data[BL_MESH_MAX][BL_MESH_MAX];
    float min_z, max_z;
    float bed_c;     /* Ender Smart (Fase 4): bed temperature when the grid was read, -1 = unknown */
    int src;         /* Ender Smart (F5-01): 1 = G29 from this page, 2 = M420 V read from this page, 0 = other */
} bl_mesh_t;

typedef struct {
    float bed_w, bed_h;
    float screw_spacing;
    float screw_pitch;
    int origin;  /* 0=FL, 1=FR, 2=BL, 3=BR */
    /* Ender Smart (Fase 4): screw assistant settings shared by every browser */
    int cw_raises;    /* turning a knob clockwise (seen from below) raises the bed: 1, lowers: -1, unknown: 0 */
    float screw_fac;  /* how far the screws sit beyond the probed area (1.2 = 20 %) */
    int screw_ref;    /* 0 = keep the average height, 1..4 = FL, FR, BL, BR stays fixed */
} bl_config_t;

typedef enum {
    BL_IDLE,
    BL_HOMING,
    BL_PROBING,
    BL_READING,   /* Ender Smart (F5-01): M420 V, no motion */
    BL_DONE,
    BL_ERROR,
} bl_state_t;

/* ---- Static state ---- */
static bl_state_t s_state = BL_IDLE;
static bl_mesh_t  s_current_mesh;
static bl_config_t s_config = { .bed_w = 220, .bed_h = 220, .screw_spacing = 180, .screw_pitch = 1.0f, .screw_fac = 1.2f };
static char s_error_msg[64];
/* Ender Smart (F5-01) */
static volatile int      s_capture_src;     /* what the next captured grid comes from (see bl_mesh_t.src) */
static volatile uint32_t s_capture_count;   /* grids parsed so far (saved or not) */
static volatile bool     s_read_same;       /* last M420 V returned the same grid as the latest saved one */
static int               s_last_op;         /* 1 = probe, 2 = read */

/* ---- Config persistence ---- */

static void load_config(void)
{
    FILE *f = fopen(BL_CONFIG_FILE, "r");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 1024) { fclose(f); return; }
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return; }
    fread(buf, 1, sz, f);
    buf[sz] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;

    cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "bed_w")) && cJSON_IsNumber(v)) s_config.bed_w = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "bed_h")) && cJSON_IsNumber(v)) s_config.bed_h = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_spacing")) && cJSON_IsNumber(v)) s_config.screw_spacing = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_pitch")) && cJSON_IsNumber(v)) s_config.screw_pitch = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "origin")) && cJSON_IsNumber(v)) s_config.origin = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "cw_raises")) && cJSON_IsNumber(v)) s_config.cw_raises = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "screw_fac")) && cJSON_IsNumber(v)) s_config.screw_fac = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_ref")) && cJSON_IsNumber(v)) s_config.screw_ref = v->valueint;

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Config loaded: bed %.0fx%.0f screw_sp=%.0f pitch=%.2f origin=%d",
             s_config.bed_w, s_config.bed_h, s_config.screw_spacing, s_config.screw_pitch, s_config.origin);
}

static void save_config(void)
{
    mkdir(BL_CACHE_DIR, 0755);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "bed_w", s_config.bed_w);
    cJSON_AddNumberToObject(root, "bed_h", s_config.bed_h);
    cJSON_AddNumberToObject(root, "screw_spacing", s_config.screw_spacing);
    cJSON_AddNumberToObject(root, "screw_pitch", s_config.screw_pitch);
    cJSON_AddNumberToObject(root, "origin", s_config.origin);
    cJSON_AddNumberToObject(root, "cw_raises", s_config.cw_raises);
    cJSON_AddNumberToObject(root, "screw_fac", s_config.screw_fac);
    cJSON_AddNumberToObject(root, "screw_ref", s_config.screw_ref);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    FILE *f = fopen(BL_CONFIG_FILE, "w");
    if (f) { fputs(json, f); fclose(f); }
    free(json);
}

/* ---- Mesh cache I/O ---- */

static void save_mesh(const bl_mesh_t *mesh)
{
    mkdir(BL_CACHE_DIR, 0755);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ts", (double)mesh->timestamp);
    cJSON_AddNumberToObject(root, "gx", mesh->grid_x);
    cJSON_AddNumberToObject(root, "gy", mesh->grid_y);
    cJSON_AddNumberToObject(root, "min", mesh->min_z);
    cJSON_AddNumberToObject(root, "max", mesh->max_z);
    if (mesh->bed_c >= 0) cJSON_AddNumberToObject(root, "bed", mesh->bed_c);
    if (mesh->src > 0) cJSON_AddNumberToObject(root, "src", mesh->src);

    cJSON *rows = cJSON_AddArrayToObject(root, "d");
    for (int y = 0; y < mesh->grid_y; y++) {
        cJSON *row = cJSON_CreateArray();
        for (int x = 0; x < mesh->grid_x; x++) {
            cJSON_AddItemToArray(row, cJSON_CreateNumber(mesh->data[y][x]));
        }
        cJSON_AddItemToArray(rows, row);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    char path[80];
    snprintf(path, sizeof(path), BL_CACHE_DIR "/mesh_%lld.json", (long long)mesh->timestamp);

    FILE *f = fopen(path, "w");
    if (f) { fputs(json, f); fclose(f); }
    free(json);

    cache_touch(path);
    cache_evict_lru(BL_CACHE_DIR, BL_CACHE_MAX + 1);  /* +1 for config.json */
    ESP_LOGI(TAG, "Mesh saved: %s (%dx%d)", path, mesh->grid_x, mesh->grid_y);
}

static bool load_mesh_file(const char *path, bl_mesh_t *mesh)
{
    FILE *f = fopen(path, "r");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 8192) { fclose(f); return false; }
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return false; }
    fread(buf, 1, sz, f);
    buf[sz] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return false;

    memset(mesh, 0, sizeof(*mesh));
    mesh->timestamp = (int64_t)cJSON_GetObjectItem(root, "ts")->valuedouble;
    mesh->grid_x = cJSON_GetObjectItem(root, "gx")->valueint;
    mesh->grid_y = cJSON_GetObjectItem(root, "gy")->valueint;
    mesh->min_z = cJSON_GetObjectItem(root, "min")->valuedouble;
    mesh->max_z = cJSON_GetObjectItem(root, "max")->valuedouble;
    cJSON *bed = cJSON_GetObjectItem(root, "bed");
    mesh->bed_c = (bed && cJSON_IsNumber(bed)) ? (float)bed->valuedouble : -1.0f;
    cJSON *src = cJSON_GetObjectItem(root, "src");
    mesh->src = (src && cJSON_IsNumber(src)) ? src->valueint : 0;

    cJSON *rows = cJSON_GetObjectItem(root, "d");
    if (rows) {
        int y = 0;
        cJSON *row;
        cJSON_ArrayForEach(row, rows) {
            if (y >= BL_MESH_MAX) break;
            int x = 0;
            cJSON *val;
            cJSON_ArrayForEach(val, row) {
                if (x >= BL_MESH_MAX) break;
                mesh->data[y][x] = val->valuedouble;
                x++;
            }
            y++;
        }
    }

    cJSON_Delete(root);
    return true;
}

/* ---- Mesh list (sorted by timestamp descending) ---- */

typedef struct { int64_t ts; char path[48]; } mesh_entry_t;

static int mesh_list_cmp(const void *a, const void *b)
{
    int64_t ta = ((const mesh_entry_t *)a)->ts;
    int64_t tb = ((const mesh_entry_t *)b)->ts;
    return (tb > ta) - (tb < ta);  /* descending */
}

static int get_mesh_list(mesh_entry_t *out, int max)
{
    DIR *d = opendir(BL_CACHE_DIR);
    if (!d) return 0;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) && n < max) {
        if (strncmp(e->d_name, "mesh_", 5) != 0) continue;
        if (strlen(e->d_name) > 30) continue;  /* sanity */
        int64_t ts = 0;
        if (sscanf(e->d_name, "mesh_%lld.json", (long long *)&ts) != 1) continue;
        out[n].ts = ts;
        snprintf(out[n].path, sizeof(out[n].path), BL_CACHE_DIR "/%s", e->d_name);
        n++;
    }
    closedir(d);
    qsort(out, n, sizeof(mesh_entry_t), mesh_list_cmp);
    return n;
}

/* ---- Line-by-line mesh parsing state machine ---- */

typedef enum {
    MESH_PARSE_IDLE,
    MESH_PARSE_SKIP_HEADER,   /* saw marker, next line is column headers — skip it */
    MESH_PARSE_DATA,          /* reading data rows */
} mesh_parse_state_t;

static mesh_parse_state_t s_parse_state = MESH_PARSE_IDLE;
static bl_mesh_t s_parse_mesh;

/**
 * Called from process_line() for every RX line — always active.
 * Detects mesh output from G29 / M420 V regardless of how it was triggered
 * (probe button, terminal, printer UI, OctoPrint, etc.).
 */
static void mesh_parse_line(const char *line)
{
    switch (s_parse_state) {
    case MESH_PARSE_IDLE:
        if (strstr(line, "Leveling Grid:") || strstr(line, "Bed Topography")) {
            memset(&s_parse_mesh, 0, sizeof(s_parse_mesh));
            s_parse_mesh.min_z = 999.0f;
            s_parse_mesh.max_z = -999.0f;
            s_parse_state = MESH_PARSE_SKIP_HEADER;
            ESP_LOGI(TAG, "Mesh marker detected");
        }
        break;

    case MESH_PARSE_SKIP_HEADER:
        /* Ender Smart (F2-03): with M155 autoreport a temperature line (or "busy:")
         * can arrive in the middle of the grid — ignore it instead of breaking the grid */
        if ((strstr(line, "T:") && strstr(line, "B:")) || strncmp(line, "busy:", 5) == 0) break;
        /* Column header line (e.g. "      0      1      2 ...") — skip */
        s_parse_state = MESH_PARSE_DATA;
        break;

    case MESH_PARSE_DATA: {
        if ((strstr(line, "T:") && strstr(line, "B:")) || strncmp(line, "busy:", 5) == 0) break;  /* F2-03 */
        const char *p = line;
        while (*p == ' ') p++;
        if (*p < '0' || *p > '9') {
            /* Not a data row — mesh is complete */
            s_parse_state = MESH_PARSE_IDLE;
            if (s_parse_mesh.grid_y > 0 && s_parse_mesh.grid_x > 0 && s_capture_src == 2 &&
                s_parse_mesh.grid_x == s_current_mesh.grid_x && s_parse_mesh.grid_y == s_current_mesh.grid_y) {
                /* F5-01: a read that returns the grid already saved adds no history entry */
                bool same = true;
                for (int y = 0; y < s_parse_mesh.grid_y && same; y++)
                    for (int x = 0; x < s_parse_mesh.grid_x && same; x++)
                        same = fabsf(s_parse_mesh.data[y][x] - s_current_mesh.data[y][x]) < 0.0005f;
                if (same) {
                    s_read_same = true;
                    s_capture_count++;
                    ESP_LOGI(TAG, "Mesh read: same as the latest saved grid, not stored again");
                    break;
                }
            }
            if (s_parse_mesh.grid_y > 0 && s_parse_mesh.grid_x > 0) {
                s_parse_mesh.src = s_capture_src;
                s_parse_mesh.timestamp = (int64_t)time(NULL);
                /* Ender Smart (Fase 4): the bed changes shape with temperature — keep it with the mesh.
                 * Safe here: process_line() runs without the state mutex held. */
                printer_state_t ps;
                printer_comm_get_state(&ps);
                s_parse_mesh.bed_c = ps.last_update_us > 0 ? ps.bed_actual : -1.0f;
                save_mesh(&s_parse_mesh);
                s_current_mesh = s_parse_mesh;
                ESP_LOGI(TAG, "Mesh captured: %dx%d, range %.3f..%.3f",
                         s_parse_mesh.grid_x, s_parse_mesh.grid_y,
                         s_parse_mesh.min_z, s_parse_mesh.max_z);
                s_capture_count++;
            }
            break;
        }
        if (s_parse_mesh.grid_y >= BL_MESH_MAX) {
            s_parse_state = MESH_PARSE_IDLE;
            break;
        }
        /* Skip row index */
        while (*p >= '0' && *p <= '9') p++;

        int col = 0;
        while (*p && col < BL_MESH_MAX) {
            char *end;
            float val = strtof(p, &end);
            if (end == p) break;
            s_parse_mesh.data[s_parse_mesh.grid_y][col] = val;
            if (val < s_parse_mesh.min_z) s_parse_mesh.min_z = val;
            if (val > s_parse_mesh.max_z) s_parse_mesh.max_z = val;
            col++;
            p = end;
        }
        if (col > 0) {
            if (s_parse_mesh.grid_y == 0) s_parse_mesh.grid_x = col;
            s_parse_mesh.grid_y++;
        }
        break;
    }
    }
}

/* ---- Probe task ---- */

/* Ender Smart (F15-18): set atomically by the HTTP handler, cleared by the task */
static bool s_probe_busy;

static void probe_task(void *arg)
{
    s_state = BL_HOMING;
    s_last_op = 1;
    s_capture_src = 1;
    printer_comm_set_polling_suppressed(true);

    ESP_LOGI(TAG, "Probe: homing (G28)...");
    esp_err_t err = printer_comm_send_gcode_sync("G28", PROBE_TIMEOUT_MS);
    if (err != ESP_OK) {
        snprintf(s_error_msg, sizeof(s_error_msg), "G28 failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "%s", s_error_msg);
        s_state = BL_ERROR;
        goto cleanup;
    }

    s_state = BL_PROBING;
    ESP_LOGI(TAG, "Probe: leveling (G29)...");
    int64_t mesh_ts_before = s_current_mesh.timestamp;
    err = printer_comm_send_gcode_sync("G29", PROBE_TIMEOUT_MS);

    if (err != ESP_OK) {
        snprintf(s_error_msg, sizeof(s_error_msg), "G29 failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "%s", s_error_msg);
        s_state = BL_ERROR;
        goto cleanup;
    }

    printer_comm_set_polling_suppressed(false);

    /* Mesh is auto-captured by the always-active line callback.
     * The parser may still be processing trailing lines — poll briefly. */
    bool found = false;
    for (int i = 0; i < 10; i++) {
        if (s_current_mesh.timestamp != mesh_ts_before &&
            s_current_mesh.grid_y > 0 && s_current_mesh.grid_x > 0) {
            found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!found) {
        snprintf(s_error_msg, sizeof(s_error_msg), "No mesh found in G29 output");
        ESP_LOGW(TAG, "%s", s_error_msg);
        s_state = BL_ERROR;
    } else {
        s_state = BL_DONE;
    }
    __atomic_clear(&s_probe_busy, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
    return;

cleanup:
    printer_comm_set_polling_suppressed(false);
    s_capture_src = 0;
    __atomic_clear(&s_probe_busy, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

/* ---- Ender Smart (F5-01): read the grid stored in the Ender (M420 V), no motion ---- */

static void read_task(void *arg)
{
    s_state = BL_READING;
    s_last_op = 2;
    s_read_same = false;
    uint32_t before = s_capture_count;
    s_capture_src = 2;

    ESP_LOGI(TAG, "Mesh read (M420 V)...");
    esp_err_t err = printer_comm_send_gcode_sync("M420 V", 15000);
    bool got = false;
    for (int i = 0; i < 10 && err == ESP_OK; i++) {
        if (s_capture_count != before) { got = true; break; }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    s_capture_src = 0;

    if (err != ESP_OK) {
        snprintf(s_error_msg, sizeof(s_error_msg), "M420 V failed: %s", esp_err_to_name(err));
        s_state = BL_ERROR;
    } else if (!got) {
        snprintf(s_error_msg, sizeof(s_error_msg), "No grid in the M420 V answer");
        s_state = BL_ERROR;
    } else {
        s_state = BL_DONE;
    }
    if (s_state == BL_ERROR) ESP_LOGW(TAG, "%s", s_error_msg);
    __atomic_clear(&s_probe_busy, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

static esp_err_t bedlevel_read_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (printer_comm_job_active())
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Cannot read while printing\"}");
    if (__atomic_test_and_set(&s_probe_busy, __ATOMIC_ACQ_REL))
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Probing or reading already in progress\"}");
    s_error_msg[0] = '\0';
    if (xTaskCreatePinnedToCore(read_task, "bl_read", 4096, NULL, 5, NULL, 0) != pdPASS) {
        __atomic_clear(&s_probe_busy, __ATOMIC_RELEASE);
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Failed to start read task\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* ---- Mesh JSON builder ---- */

static void mesh_to_json(const bl_mesh_t *mesh, html_buf_t *p)
{
    html_buf_printf(p, "{\"ts\":%lld,\"gx\":%d,\"gy\":%d,\"min\":%.3f,\"max\":%.3f,",
                    (long long)mesh->timestamp, mesh->grid_x, mesh->grid_y, mesh->min_z, mesh->max_z);
    if (mesh->bed_c >= 0) html_buf_printf(p, "\"bed\":%.1f,", mesh->bed_c);
    if (mesh->src > 0) html_buf_printf(p, "\"src\":%d,", mesh->src);
    html_buf_printf(p, "\"d\":[");
    for (int y = 0; y < mesh->grid_y; y++) {
        html_buf_printf(p, "%s[", y ? "," : "");
        for (int x = 0; x < mesh->grid_x; x++) {
            html_buf_printf(p, "%s%.3f", x ? "," : "", mesh->data[y][x]);
        }
        html_buf_printf(p, "]");
    }
    html_buf_printf(p, "]}");
}

/* ---- HTTP handlers ---- */

static esp_err_t bedlevel_probe_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    /* Ender Smart (F15-18): G28+G29 moves the machine — require an explicit
     * confirmation in the request (a bare POST, e.g. from another page, is refused) */
    char body[64] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len < 0) len = 0;
    body[len] = '\0';
    if (!strstr(body, "confirm=1")) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Confirmation required\"}");
    }

    if (printer_comm_job_active()) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Cannot probe while printing\"}");
    }

    /* One probe at a time: atomic test-and-set closes the double-click race
     * (two requests before the task had changed s_state) */
    if (__atomic_test_and_set(&s_probe_busy, __ATOMIC_ACQ_REL)) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Probing already in progress\"}");
    }

    s_state = BL_IDLE;
    s_error_msg[0] = '\0';

    if (xTaskCreatePinnedToCore(probe_task, "bl_probe", 4096, NULL, 5, NULL, 0) != pdPASS) {
        __atomic_clear(&s_probe_busy, __ATOMIC_RELEASE);
        return httpd_resp_sendstr(req, "{\"ok\":false,\"err\":\"Failed to start probe task\"}");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t bedlevel_status_handler(httpd_req_t *req)
{
    const char *state_str;
    switch (s_state) {
    case BL_HOMING:  state_str = "homing"; break;
    case BL_PROBING: state_str = "probing"; break;
    case BL_READING: state_str = "reading"; break;
    case BL_DONE:    state_str = "done"; break;
    case BL_ERROR:   state_str = "error"; break;
    default:         state_str = "idle"; break;
    }

    html_buf_t p;
    html_buf_init(&p);
    html_buf_printf(&p, "{\"state\":\"%s\",\"op\":\"%s\"", state_str,
                    s_last_op == 2 ? "read" : s_last_op == 1 ? "probe" : "");
    if (s_state == BL_DONE && s_last_op == 2 && s_read_same) html_buf_printf(&p, ",\"same\":true");
    if (s_state == BL_ERROR && s_error_msg[0]) {
        html_buf_printf(&p, ",\"err\":\"%s\"", s_error_msg);
    }
    if (s_state == BL_DONE && s_current_mesh.grid_x > 0) {
        html_buf_printf(&p, ",\"mesh\":");
        mesh_to_json(&s_current_mesh, &p);
    }
    html_buf_printf(&p, "}");

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_meshlist_handler(httpd_req_t *req)
{
    mesh_entry_t *list = malloc(BL_CACHE_MAX * sizeof(mesh_entry_t));
    if (!list) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int n = get_mesh_list(list, BL_CACHE_MAX);

    html_buf_t p;
    html_buf_init(&p);
    html_buf_printf(&p, "[");
    for (int i = 0; i < n; i++) {
        html_buf_printf(&p, "%s%lld", i ? "," : "", (long long)list[i].ts);
    }
    html_buf_printf(&p, "]");

    free(list);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_mesh_handler(httpd_req_t *req)
{
    char ts_str[24] = {0};
    httpd_req_get_url_query_str(req, ts_str, sizeof(ts_str));
    int64_t ts = 0;
    char *eq = strchr(ts_str, '=');
    if (eq) ts = atoll(eq + 1);

    char path[48];
    snprintf(path, sizeof(path), BL_CACHE_DIR "/mesh_%lld.json", (long long)ts);

    bl_mesh_t mesh;
    if (!load_mesh_file(path, &mesh)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Mesh not found");
        return ESP_FAIL;
    }

    html_buf_t p;
    html_buf_init(&p);
    mesh_to_json(&mesh, &p);

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_config_get_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"bed_w\":%.1f,\"bed_h\":%.1f,\"screw_spacing\":%.1f,\"screw_pitch\":%.2f,\"origin\":%d,"
             "\"cw_raises\":%d,\"screw_fac\":%.2f,\"screw_ref\":%d}",
             s_config.bed_w, s_config.bed_h, s_config.screw_spacing, s_config.screw_pitch, s_config.origin,
             s_config.cw_raises, s_config.screw_fac, s_config.screw_ref);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t bedlevel_config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "bed_w")) && cJSON_IsNumber(v)) s_config.bed_w = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "bed_h")) && cJSON_IsNumber(v)) s_config.bed_h = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_spacing")) && cJSON_IsNumber(v)) s_config.screw_spacing = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_pitch")) && cJSON_IsNumber(v)) s_config.screw_pitch = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "origin")) && cJSON_IsNumber(v)) s_config.origin = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "cw_raises")) && cJSON_IsNumber(v) && v->valueint >= -1 && v->valueint <= 1)
        s_config.cw_raises = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "screw_fac")) && cJSON_IsNumber(v) && v->valuedouble >= 1.0 && v->valuedouble <= 2.0)
        s_config.screw_fac = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_ref")) && cJSON_IsNumber(v) && v->valueint >= 0 && v->valueint <= 4)
        s_config.screw_ref = v->valueint;
    cJSON_Delete(root);

    save_config();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* ---- Page handler ---- */

static esp_err_t bedlevel_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, "Bed Leveling", "/bedlevel");

    html_buf_printf(&p,
        "<style>"
        ".mesh-grid{display:inline-grid;gap:2px;margin:12px 0}"
        ".mesh-cell{width:48px;height:48px;display:flex;align-items:center;justify-content:center;"
        "font-size:11px;color:#fff;border-radius:3px;text-shadow:0 0 2px rgba(0,0,0,0.7)}"
        ".stats{display:flex;gap:16px;flex-wrap:wrap;margin:8px 0;font-size:14px}"
        ".stats span{padding:2px 8px;border-radius:4px}"
        ".badge-good{background:#4caf50;color:#fff}"
        ".badge-fair{background:#ff9800;color:#fff}"
        ".badge-poor{background:#f44336;color:#fff}"
        ".screw-tbl{margin:12px 0}"
        ".screw-tbl td{padding:4px 12px}"
        ".hist-nav{display:flex;align-items:center;gap:12px;margin:8px 0}"
        "#probe-btn:disabled{opacity:0.5}"
        "details{margin:12px 0}"
        "summary{cursor:pointer;font-weight:bold;font-size:15px}"
        "input[type=number]{width:80px;padding:4px}"
        ".cfg-row{margin:6px 0}"
        ".cfg-row label{display:inline-block;width:140px}"
        "</style>"

        "<h2>Bed Leveling</h2>"

        "<button id='probe-btn' onclick='probe()'>Probe Bed (G28 + G29)</button>"
        " <span id='probe-status'></span>"

        "<div id='mesh-stats'></div>"
        "<div id='mesh-box'></div>"

        "<div class='hist-nav'>"
        "<button onclick='histPrev()'>&#9664; Prev</button>"
        "<span id='hist-label'>-</span>"
        "<button onclick='histNext()'>Next &#9654;</button>"
        "</div>"

        "<details>"
        "<summary>Screw Adjustment</summary>"
        "<div style='margin:8px 0'>"
        "<button onclick='calcScrews(\"tighten\")'>Tighten (CW)</button>"
        " <button onclick='calcScrews(\"loosen\")'>Loosen (CCW)</button>"
        "</div>"
        "<div id='screw-result'></div>"
        "</details>"

        "<details>"
        "<summary>Bed Configuration</summary>"
        "<div class='cfg-row'><label>Bed width (mm):</label>"
        "<input type='number' id='cfg-bw' step='1'></div>"
        "<div class='cfg-row'><label>Bed height (mm):</label>"
        "<input type='number' id='cfg-bh' step='1'></div>"
        "<div class='cfg-row'><label>Screw spacing (mm):</label>"
        "<input type='number' id='cfg-ss' step='1'></div>"
        "<div class='cfg-row'><label>Screw pitch (mm/rot):</label>"
        "<input type='number' id='cfg-sp' step='0.01' value='1.0'></div>"
        "<div class='cfg-row'><label>Origin (0,0) corner:</label>"
        "<select id='cfg-or'>"
        "<option value='0'>Front-Left</option>"
        "<option value='1'>Front-Right</option>"
        "<option value='2'>Back-Left</option>"
        "<option value='3'>Back-Right</option>"
        "</select></div>"
        "<button onclick='saveCfg()' style='margin-top:8px'>Save Config</button>"
        " <span id='cfg-status'></span>"
        "</details>"
    );

    html_buf_printf(&p,
        "<script>"
        "var meshList=[],meshIdx=-1,curMesh=null,cfg={};"

        /* Color mapping: blue(low) -> green(0) -> red(high) */
        "function zColor(z,mn,mx){"
        "var am=Math.max(Math.abs(mn),Math.abs(mx),0.01);"
        "var n=z/am;"
        "var h=n<=0?120+(-n)*120:120-n*120;"
        "var s=Math.min(100,Math.abs(n)*150+30);"
        "return 'hsl('+h+','+s+'%%,42%%)'}"

        /* Render mesh grid + stats */
        "function renderMesh(m){"
        "if(!m||!m.gx){document.getElementById('mesh-box').innerHTML='<p class=\"hint\">No mesh data</p>';return}"
        "var g=m.gx,r=m.gy,d=m.d;"

        /* Stats */
        "var range=m.max-m.min;"
        "var maxDev=Math.max(Math.abs(m.min),Math.abs(m.max));"
        "var sum=0,sum2=0,cnt=0;"
        "for(var y=0;y<r;y++)for(var x=0;x<g;x++){sum+=d[y][x];cnt++}"
        "var avg=sum/cnt;sum2=0;"
        "for(var y=0;y<r;y++)for(var x=0;x<g;x++){var v=d[y][x]-avg;sum2+=v*v}"
        "var sd=Math.sqrt(sum2/cnt);"
        "var badge=range<0.3?'good':range<0.6?'fair':'poor';"
        "document.getElementById('mesh-stats').innerHTML="
        "'<div class=\"stats\">"
        "<span>Range: '+range.toFixed(3)+' mm</span>"
        "<span>Max dev: '+maxDev.toFixed(3)+' mm</span>"
        "<span>Std dev: '+sd.toFixed(3)+' mm</span>"
        "<span class=\"badge-'+badge+'\">'+badge.charAt(0).toUpperCase()+badge.slice(1)+'</span>"
        "</div>';"

        /* Grid */
        /* Render grid oriented so back=top, front=bottom, left=left, right=right. "
        "   Origin config tells us where mesh[0][0] is physically. */
        "var o=cfg.origin||0;"
        "var flipY=(o<2);var flipX=(o===1||o===3);"
        "var h='<div class=\"mesh-grid\" style=\"grid-template-columns:repeat('+g+',48px)\">';"
        "for(var yi=0;yi<r;yi++){var y=flipY?r-1-yi:yi;"
        "for(var xi=0;xi<g;xi++){var x=flipX?g-1-xi:xi;"
        "h+='<div class=\"mesh-cell\" style=\"background:'+zColor(d[y][x],m.min,m.max)+'\">'+d[y][x].toFixed(3)+'</div>'}}"
        "h+='</div>';"
        "document.getElementById('mesh-box').innerHTML=h}"

        /* Probe */
        "var pollId=0;"
        "function probe(){"
        "if(!confirm('Home all axes (G28) and probe the bed (G29)? The nozzle will touch the bed several times (~2-3 min). Stay near the printer.'))return;"
        "document.getElementById('probe-btn').disabled=true;"
        "document.getElementById('probe-status').textContent='Starting...';"
        "fetch('/bedlevel/probe',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'confirm=1'}).then(r=>r.json()).then(function(j){"
        "if(!j.ok){document.getElementById('probe-status').textContent=j.err;document.getElementById('probe-btn').disabled=false;return}"
        "pollId=setInterval(pollStatus,2000)})}"

        "function pollStatus(){"
        "fetch('/bedlevel/status').then(r=>r.json()).then(function(j){"
        "var st=j.state;"
        "document.getElementById('probe-status').textContent=st==='homing'?'Homing...':st==='probing'?'Probing...':st;"
        "if(st==='done'||st==='error'){"
        "clearInterval(pollId);"
        "document.getElementById('probe-btn').disabled=false;"
        "if(st==='done'&&j.mesh){curMesh=j.mesh;renderMesh(curMesh);loadMeshList()}"
        "if(st==='error')document.getElementById('probe-status').textContent='Error: '+(j.err||'unknown')}})}"

        /* History */
        "function loadMeshList(){"
        "fetch('/bedlevel/meshlist').then(r=>r.json()).then(function(l){"
        "meshList=l;meshIdx=0;updateHistLabel()})}"

        "function updateHistLabel(){"
        "var lbl=document.getElementById('hist-label');"
        "if(meshList.length===0){lbl.textContent='-';return}"
        "var ts=meshList[meshIdx];"
        "var d=new Date(ts*1000);"
        "lbl.textContent=(meshIdx+1)+'/'+meshList.length+' — '+d.toLocaleString()}"

        "function histPrev(){if(meshIdx<meshList.length-1){meshIdx++;loadMeshAt(meshIdx)}}"
        "function histNext(){if(meshIdx>0){meshIdx--;loadMeshAt(meshIdx)}}"

        "function loadMeshAt(i){"
        "fetch('/bedlevel/mesh?ts='+meshList[i]).then(r=>r.json()).then(function(m){"
        "curMesh=m;renderMesh(m);updateHistLabel()})}"

        /* Screw adjustment */
        "function calcScrews(mode){"
        "if(!curMesh||!curMesh.gx){document.getElementById('screw-result').innerHTML='<p>No mesh data</p>';return}"
        "var m=curMesh,gx=m.gx,gy=m.gy,d=m.d;"
        "var mx=Math.floor(gx/2),my=Math.floor(gy/2);"
        "var o=cfg.origin||0;var sFlipY=(o>=2);var sFlipX=(o===1||o===3);"
        "var avg=[0,0,0,0],cnt=[0,0,0,0];"
        "for(var y=0;y<gy;y++)for(var x=0;x<gx;x++){"
        "var py=sFlipY?gy-1-y:y;var px=sFlipX?gx-1-x:x;"
        "var qi=(py>=my?2:0)+(px>=mx?1:0);"
        "avg[qi]+=d[y][x];cnt[qi]++}"
        "for(var i=0;i<4;i++)if(cnt[i])avg[i]/=cnt[i];"
        "var labels=['Front-Left','Front-Right','Back-Left','Back-Right'];"
        "var ref;"
        "if(mode==='tighten'){"
        "ref=Math.min.apply(null,avg);"
        "}else{"
        "ref=Math.max.apply(null,avg)}"
        "var h='<table class=\"screw-tbl\"><tr><th>Screw</th><th>Avg Z</th><th>Correction</th></tr>';"
        "var pitch=cfg.screw_pitch||1;"
        "for(var i=0;i<4;i++){"
        "var delta=mode==='tighten'?(avg[i]-ref):(ref-avg[i]);"
        "var turns=Math.round(delta/pitch*8)/8;"
        "var dir=mode==='tighten'?'CW':'CCW';"
        "h+='<tr><td>'+labels[i]+'</td><td>'+avg[i].toFixed(3)+'</td><td>';"
        "if(Math.abs(turns)<0.06)h+='OK (reference)';"
        "else{var q=Math.floor(turns*4)/4;var e=turns-q;"
        "var s='';if(q>=0.25)s+=Math.round(q*4)+'/4';"
        "if(e>=0.1){if(s)s+=' + ';s+='1/8'}"
        "if(!s)s='1/8';"
        "h+=s+' turn'+((turns>0.25)?'s':'')+' '+dir}"
        "h+='</td></tr>'}"
        "h+='</table>';"
        "document.getElementById('screw-result').innerHTML=h}"

        /* Config */
        "function loadCfg(){"
        "fetch('/bedlevel/config').then(r=>r.json()).then(function(c){"
        "cfg=c;"
        "document.getElementById('cfg-bw').value=c.bed_w;"
        "document.getElementById('cfg-bh').value=c.bed_h;"
        "document.getElementById('cfg-ss').value=c.screw_spacing;"
        "document.getElementById('cfg-sp').value=c.screw_pitch;"
        "document.getElementById('cfg-or').value=c.origin||0})}"

        "function saveCfg(){"
        "var c={bed_w:+document.getElementById('cfg-bw').value,"
        "bed_h:+document.getElementById('cfg-bh').value,"
        "screw_spacing:+document.getElementById('cfg-ss').value,"
        "screw_pitch:+document.getElementById('cfg-sp').value,"
        "origin:+document.getElementById('cfg-or').value};"
        "fetch('/bedlevel/config',{method:'POST',body:JSON.stringify(c)}).then(r=>r.json()).then(function(j){"
        "if(j.ok){cfg=c;document.getElementById('cfg-status').textContent='Saved!';"
        "setTimeout(function(){document.getElementById('cfg-status').textContent=''},2000)}})}"

        /* Init: load config, check if probe in progress, load mesh list */
        "loadCfg();"
        "fetch('/bedlevel/status').then(r=>r.json()).then(function(j){"
        "if(j.state==='homing'||j.state==='probing'){"
        "document.getElementById('probe-btn').disabled=true;"
        "document.getElementById('probe-status').textContent=j.state==='homing'?'Homing...':'Probing...';"
        "pollId=setInterval(pollStatus,2000)}"
        "else if(j.state==='done'&&j.mesh){curMesh=j.mesh;renderMesh(curMesh)}});"
        "fetch('/bedlevel/meshlist').then(r=>r.json()).then(function(l){"
        "meshList=l;meshIdx=0;updateHistLabel();"
        "if(l.length>0&&!curMesh)loadMeshAt(0);"
        "else if(!curMesh)renderMesh(null)});"
        "</script>"
    );

    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/* ---- Registration ---- */

esp_err_t bedlevel_register_httpd(httpd_handle_t server)
{
    load_config();

    /* Always listen for mesh output from the printer */
    printer_comm_set_line_callback(mesh_parse_line);

    /* Load latest mesh into s_current_mesh */
    mesh_entry_t *list = malloc(BL_CACHE_MAX * sizeof(mesh_entry_t));
    if (list) {
        int n = get_mesh_list(list, BL_CACHE_MAX);
        if (n > 0) {
            load_mesh_file(list[0].path, &s_current_mesh);
            ESP_LOGI(TAG, "Loaded latest mesh: %dx%d", s_current_mesh.grid_x, s_current_mesh.grid_y);
        }
        free(list);
    }

    httpd_uri_t uris[] = {
        { .uri = "/bedlevel",        .method = HTTP_GET,  .handler = bedlevel_page_handler },
        { .uri = "/bedlevel/probe",  .method = HTTP_POST, .handler = bedlevel_probe_handler },
        { .uri = "/bedlevel/read",   .method = HTTP_POST, .handler = bedlevel_read_handler },
        { .uri = "/bedlevel/status", .method = HTTP_GET,  .handler = bedlevel_status_handler },
        { .uri = "/bedlevel/meshlist", .method = HTTP_GET, .handler = bedlevel_meshlist_handler },
        { .uri = "/bedlevel/mesh",   .method = HTTP_GET,  .handler = bedlevel_mesh_handler },
        { .uri = "/bedlevel/config", .method = HTTP_GET,  .handler = bedlevel_config_get_handler },
        { .uri = "/bedlevel/config", .method = HTTP_POST, .handler = bedlevel_config_post_handler },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }

    return ESP_OK;
}
