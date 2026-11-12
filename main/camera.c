#include "camera.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "camera";

#define NVS_NS_CAMERA  "camera"
#define NVS_KEY_ROT180 "rotate180"
#define NVS_KEY_BRIGHTNESS "bright"
#define NVS_KEY_CONTRAST   "contrast"
#define NVS_KEY_SATURATION "satur"
#define NVS_KEY_AE_LEVEL   "ae_level"
#define NVS_KEY_XCLK       "xclk_mhz"

static bool s_rotate180 = false;
static uint8_t s_xclk_mhz = 10;  /* default safe value */
static int8_t s_brightness = 0;   /* -2..2 */
static int8_t s_contrast = 0;     /* -2..2 */
static int8_t s_saturation = 0;   /* -2..2 */
static int8_t s_ae_level = 0;     /* -2..2 */
static bool s_available = false;  /* Ender Smart (F15-04): true only after a successful init */

/* ---- Triple-buffered PSRAM frames ---- */

#define FRAME_BUF_INIT_SIZE (50 * 1024)  /* 50 KB — covers most SVGA Q12 frames */
#define CAPTURE_FPS         10
#define CAPTURE_INTERVAL_MS (1000 / CAPTURE_FPS)
#define STATS_INTERVAL_S    60           /* how often to log capture stats */

#define FRAME_COUNT 3

static camera_frame_t s_frames[FRAME_COUNT];
static camera_frame_t *s_latest;       /* consumer-facing (front buffer) */
static portMUX_TYPE s_frame_mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Freenove ESP32-S3-WROOM pin mapping (DVP parallel interface).
 * Reference: Freenove schematic + board silkscreen.
 */
#define CAM_PIN_PWDN   -1  /* No power-down pin */
#define CAM_PIN_RESET   -1  /* No hardware reset pin */
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4   /* SCCB SDA */
#define CAM_PIN_SIOC    5   /* SCCB SCL */

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

esp_err_t camera_init(void)
{
    /* Load XCLK from NVS before camera init (needs reboot to change) */
    nvs_handle_t nvs_pre;
    if (nvs_open(NVS_NS_CAMERA, NVS_READONLY, &nvs_pre) == ESP_OK) {
        uint8_t val = 0;
        if (nvs_get_u8(nvs_pre, NVS_KEY_XCLK, &val) == ESP_OK && val >= 8 && val <= 20)
            s_xclk_mhz = val;
        nvs_close(nvs_pre);
    }

    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7       = CAM_PIN_D7,
        .pin_d6       = CAM_PIN_D6,
        .pin_d5       = CAM_PIN_D5,
        .pin_d4       = CAM_PIN_D4,
        .pin_d3       = CAM_PIN_D3,
        .pin_d2       = CAM_PIN_D2,
        .pin_d1       = CAM_PIN_D1,
        .pin_d0       = CAM_PIN_D0,
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,

        .xclk_freq_hz = s_xclk_mhz * 1000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_SVGA, /* 800x600 */
        .jpeg_quality = 12,             /* 1-63, lower = better quality */
        .fb_count     = 2,              /* 2 DMA buffers in PSRAM, GRAB_LATEST drops stale */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x (%s)", err, esp_err_to_name(err));
        return err;
    }
    s_available = true;

    /* Load rotation setting from NVS and apply */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t val = 0;
        if (nvs_get_u8(nvs, NVS_KEY_ROT180, &val) == ESP_OK)
            s_rotate180 = (val != 0);
        int8_t sval;
        if (nvs_get_i8(nvs, NVS_KEY_BRIGHTNESS, &sval) == ESP_OK) s_brightness = sval;
        if (nvs_get_i8(nvs, NVS_KEY_CONTRAST, &sval) == ESP_OK)   s_contrast = sval;
        if (nvs_get_i8(nvs, NVS_KEY_SATURATION, &sval) == ESP_OK) s_saturation = sval;
        if (nvs_get_i8(nvs, NVS_KEY_AE_LEVEL, &sval) == ESP_OK)   s_ae_level = sval;
        nvs_close(nvs);
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        bool is_ov3660 = (s->id.PID == OV3660_PID);
        /* OV3660 native orientation is 180° rotated vs OV2640.
         * Invert vflip for OV3660 so rotate180 toggle works the same way. */
        s->set_hmirror(s, s_rotate180 ? 1 : 0);
        s->set_vflip(s, s_rotate180 != is_ov3660 ? 1 : 0);
        s->set_brightness(s, s_brightness);
        s->set_contrast(s, s_contrast);
        s->set_saturation(s, s_saturation);
        s->set_ae_level(s, s_ae_level);
        ESP_LOGI(TAG, "Sensor: %s", is_ov3660 ? "OV3660" : "OV2640");
    }

    ESP_LOGI(TAG, "Camera initialized (SVGA JPEG Q%d, %d DMA buffers, XCLK %d MHz, rotate180=%d)",
             config.jpeg_quality, config.fb_count,
             (int)(config.xclk_freq_hz / 1000000), s_rotate180);
    return ESP_OK;
}

/* ---- Background capture task ---- */

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started (%d FPS target)", CAPTURE_FPS);

    /* Accept frames up to 20% early to avoid skipping a frame that's "close enough" */
    const int64_t min_interval_us = CAPTURE_INTERVAL_MS * 1000 * 80 / 100;
    int64_t last_kept_us = 0;

    /* Stats */
    const int64_t stats_interval_us = STATS_INTERVAL_S * 1000000LL;
    int64_t stats_start_us = esp_timer_get_time();
    uint32_t stats_captured = 0;
    uint64_t stats_total_bytes = 0;
    uint64_t stats_hold_us = 0;
    bool stats_pending = false;
    float stats_log_fps;
    uint32_t stats_log_frames, stats_log_avg_size, stats_log_avg_hold;
    int stats_log_secs;

    while (true) {
        /* Deferred stats log — outside DMA buffer hold */
        if (stats_pending) {
            stats_pending = false;
            ESP_LOGI(TAG, "capture: %.1f FPS, avg %"PRIu32" B, hold %"PRIu32" us (%"PRIu32" frames / %ds)",
                     stats_log_fps, stats_log_avg_size, stats_log_avg_hold,
                     stats_log_frames, stats_log_secs);
        }

        /* Capture from DMA — blocks until camera has a frame */
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int64_t fb_get_us = esp_timer_get_time();
        int64_t now_us = fb_get_us;
        stats_captured++;
        stats_total_bytes += fb->len;

        /* Check if stats are due — snapshot values, log deferred to next iteration */
        int64_t stats_elapsed = now_us - stats_start_us;
        if (stats_elapsed >= stats_interval_us) {
            stats_log_fps = stats_captured * 1000000.0f / stats_elapsed;
            stats_log_frames = stats_captured;
            stats_log_avg_size = (uint32_t)(stats_total_bytes / stats_captured);
            stats_log_avg_hold = (uint32_t)(stats_hold_us / stats_captured);
            stats_log_secs = (int)(stats_elapsed / 1000000);
            stats_captured = 0;
            stats_total_bytes = 0;
            stats_hold_us = 0;
            stats_start_us = now_us;
            stats_pending = true;
        }

        /* Drop frame if too soon since last kept frame */
        if (last_kept_us && (now_us - last_kept_us) < min_interval_us) {
            stats_hold_us += esp_timer_get_time() - fb_get_us;
            esp_camera_fb_return(fb);
            continue;
        }

        /* Find a free buffer (refcount == 0, not s_latest).
         * No lock needed: only capture task selects buffers, and consumers
         * can only increment refcount on s_latest — a non-latest buffer's
         * refcount can only stay the same or decrease. */
        camera_frame_t *latest = s_latest;
        camera_frame_t *back = NULL;
        for (int i = 0; i < FRAME_COUNT; i++) {
            if (&s_frames[i] != latest &&
                __atomic_load_n(&s_frames[i].refcount, __ATOMIC_ACQUIRE) == 0) {
                back = &s_frames[i];
                break;
            }
        }
        if (!back) {
            stats_hold_us += esp_timer_get_time() - fb_get_us;
            esp_camera_fb_return(fb);
            continue;  /* all buffers held by consumers */
        }

        /* Grow PSRAM buffer if needed */
        if (fb->len > back->buf_size) {
            size_t new_size = fb->len + 4096;  /* some headroom */
            uint8_t *new_buf = heap_caps_realloc(back->buf, new_size,
                                                  MALLOC_CAP_SPIRAM);
            if (!new_buf) {
                ESP_LOGW(TAG, "PSRAM realloc failed (%u bytes)", (unsigned)new_size);
                stats_hold_us += esp_timer_get_time() - fb_get_us;
                esp_camera_fb_return(fb);
                continue;
            }
            back->buf = new_buf;
            back->buf_size = new_size;
        }

        /* Copy JPEG to PSRAM back buffer */
        memcpy(back->buf, fb->buf, fb->len);
        back->len = fb->len;

        /* Return DMA buffer immediately */
        stats_hold_us += esp_timer_get_time() - fb_get_us;
        esp_camera_fb_return(fb);

        last_kept_us = now_us;

        /* Swap: back buffer becomes the new latest.
         * Spinlock pairs with camera_get_frame() to prevent a consumer
         * from reading s_latest and incrementing refcount on a stale frame
         * that the capture task is about to reuse. */
        taskENTER_CRITICAL(&s_frame_mux);
        s_latest = back;
        taskEXIT_CRITICAL(&s_frame_mux);

    }
}

void camera_start_capture_task(void)
{
    /* Allocate PSRAM frame buffers */
    for (int i = 0; i < FRAME_COUNT; i++) {
        s_frames[i].buf = heap_caps_malloc(FRAME_BUF_INIT_SIZE, MALLOC_CAP_SPIRAM);
        s_frames[i].buf_size = FRAME_BUF_INIT_SIZE;
        s_frames[i].len = 0;
        s_frames[i].refcount = 0;
    }

    s_latest = NULL;  /* no frame yet */

    ESP_LOGI(TAG, "Allocated %dx %u KB PSRAM frame buffers",
             FRAME_COUNT, (unsigned)(FRAME_BUF_INIT_SIZE / 1024));

    xTaskCreatePinnedToCore(camera_capture_task, "cam_cap", 3072,
                            NULL, 5, NULL, 0);
}

bool camera_is_available(void)
{
    return s_available;
}

camera_frame_t *camera_get_frame(void)
{
    taskENTER_CRITICAL(&s_frame_mux);
    camera_frame_t *f = s_latest;
    if (f) {
        f->refcount++;
    }
    taskEXIT_CRITICAL(&s_frame_mux);
    return f;
}

void camera_release_frame(camera_frame_t *frame)
{
    if (!frame) return;
    __atomic_sub_fetch(&frame->refcount, 1, __ATOMIC_RELEASE);
}

void camera_set_rotate180(bool enable)
{
    s_rotate180 = enable;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        bool is_ov3660 = (s->id.PID == OV3660_PID);
        s->set_hmirror(s, enable ? 1 : 0);
        s->set_vflip(s, enable != is_ov3660 ? 1 : 0);
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_ROT180, enable ? 1 : 0);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Rotate 180°: %s", enable ? "ON" : "OFF");
}

bool camera_get_rotate180(void)
{
    return s_rotate180;
}

void camera_set_image_params(int brightness, int contrast, int saturation, int ae_level)
{
    /* Clamp to -2..2 */
    #define CLAMP(v) ((v) < -2 ? -2 : (v) > 2 ? 2 : (v))
    s_brightness = CLAMP(brightness);
    s_contrast   = CLAMP(contrast);
    s_saturation = CLAMP(saturation);
    s_ae_level   = CLAMP(ae_level);
    #undef CLAMP

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, s_brightness);
        s->set_contrast(s, s_contrast);
        s->set_saturation(s, s_saturation);
        s->set_ae_level(s, s_ae_level);
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_i8(nvs, NVS_KEY_BRIGHTNESS, s_brightness);
        nvs_set_i8(nvs, NVS_KEY_CONTRAST,   s_contrast);
        nvs_set_i8(nvs, NVS_KEY_SATURATION, s_saturation);
        nvs_set_i8(nvs, NVS_KEY_AE_LEVEL,   s_ae_level);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Image: bright=%d contrast=%d sat=%d ae=%d",
             s_brightness, s_contrast, s_saturation, s_ae_level);
}

void camera_get_image_params(int *brightness, int *contrast, int *saturation, int *ae_level)
{
    if (brightness) *brightness = s_brightness;
    if (contrast)   *contrast   = s_contrast;
    if (saturation) *saturation = s_saturation;
    if (ae_level)   *ae_level   = s_ae_level;
}

void camera_set_xclk_mhz(int mhz)
{
    if (mhz != 10 && mhz != 12 && mhz != 16 && mhz != 20) return;
    s_xclk_mhz = mhz;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_XCLK, s_xclk_mhz);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "XCLK set to %d MHz (reboot to apply)", mhz);
}

int camera_get_xclk_mhz(void)
{
    return s_xclk_mhz;
}
