#include "wifi.h"
#include "camera.h"
#include "httpd.h"
#include "dns_server.h"
#include "usb_serial.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "obico_client.h"
#include "rfc2217.h"
#include "terminal.h"
#include "logbuf.h"
#include "mdns_service.h"
#include "ota.h"
#include "esp_littlefs.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

static void log_heap(const char *label)
{
    ESP_LOGI(TAG, "HEAP [%s] DMA: %lu free, %lu min, largest: %lu | Internal: %lu free | PSRAM: %lu free",
             label,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static void heap_monitor_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        log_heap("periodic");
    }
}

/* Ender Smart (F15-03): booted without the saved network — start mDNS once it is back */
static void mdns_when_connected_task(void *arg)
{
    wifi_wait_connected(portMAX_DELAY);
    mdns_service_init();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 FDM Printer Bridge starting...");

    /* NVS — required by WiFi and Obico token storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Log buffer — capture boot logs from here on */
    logbuf_init();
    ESP_LOGI(TAG, "Reset reason: %d", (int)esp_reset_reason());

    /* WiFi — with saved credentials it never gives up (F15-02/03) */
    wifi_result_t wifi_result = wifi_init();

    if (wifi_result == WIFI_RESULT_AP_MODE) {
        /* No credentials at all: setup portal only (original behaviour) */
        dns_server_start();
        httpd_start_captive_portal();
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "  AP MODE — configure WiFi at");
        ESP_LOGW(TAG, "  http://192.168.4.1/");
        ESP_LOGW(TAG, "========================================");
        ota_schedule_boot_confirm(CONFIG_BOOT_CONFIRM_DELAY_S);
        return;
    }

    if (wifi_result == WIFI_RESULT_STA_CONNECTED) {
        mdns_service_init();
    } else {
        /* WIFI_RESULT_STA_PENDING: keep booting so the printer, USB and camera work
         * without the router; Wi-Fi keeps retrying in the background */
        ESP_LOGW(TAG, "Saved Wi-Fi not reachable — continuing boot without it");
        xTaskCreatePinnedToCore(mdns_when_connected_task, "mdns_wait", 3072,
                                NULL, 2, NULL, 0);
    }

    log_heap("pre-camera");

    /* Camera — optional (F15-04): a camera fault must not take the printer down */
    if (camera_init() == ESP_OK) {
        camera_start_capture_task();
    } else {
        ESP_LOGE(TAG, "Camera unavailable — continuing without video "
                      "(/stream and /capture answer 503)");
    }
    log_heap("post-camera");

    /* SD card — optional, non-fatal if no card inserted */
    sdcard_init();

    /* LittleFS cache partition (~12MB) */
    esp_vfs_littlefs_conf_t lfs_conf = {
        .base_path = "/cache",
        .partition_label = "cache",
        .format_if_mount_failed = true,
    };
    ret = esp_vfs_littlefs_register(&lfs_conf);
    if (ret == ESP_OK) {
        size_t total = 0, used = 0;
        esp_littlefs_info("cache", &total, &used);
        ESP_LOGI(TAG, "LittleFS /cache: %u KB used / %u KB total",
                 (unsigned)(used / 1024), (unsigned)(total / 1024));
    } else {
        ESP_LOGW(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
    }

    /* Terminal ring buffer (must init before HTTP server & USB RX) */
    terminal_init();

    /* HTTP server (MJPEG stream + snapshot + SD card + all endpoints) */
    ESP_ERROR_CHECK(httpd_start_server());
    log_heap("post-httpd");

    /* Printer comm — always init with USB serial */
    ESP_ERROR_CHECK(usb_serial_init(printer_comm_rx_cb, NULL));
    ESP_ERROR_CHECK(printer_comm_init());

    /* Obico client */
    ESP_ERROR_CHECK(obico_client_init());
    log_heap("post-obico");

    xTaskCreatePinnedToCore(heap_monitor_task, "heap_mon", 3072,
                            NULL, 1, NULL, 0);

    /* RFC 2217 serial bridge — off by default (F15-11): it bypasses the command policy */
#if CONFIG_RFC2217_ENABLE
    ESP_ERROR_CHECK(rfc2217_start());
#else
    ESP_LOGI(TAG, "RFC 2217 bridge disabled (CONFIG_RFC2217_ENABLE=n)");
#endif

    /* HTTP server and USB host are up: confirm a fresh OTA image after a stable
     * period, otherwise the bootloader rolls back (F15-13) */
    ota_schedule_boot_confirm(CONFIG_BOOT_CONFIRM_DELAY_S);

    const char *ip = wifi_get_ip_str();
    printer_backend_t backend = printer_comm_get_backend();
    ESP_LOGI(TAG, "========================================");
    if (backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Klipper → %s:%u)",
                 printer_comm_get_mr_host(), printer_comm_get_mr_port());
    } else {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Marlin)");
    }
    ESP_LOGI(TAG, "  http://%s/", ip);
    ESP_LOGI(TAG, "========================================");
}
