#include "usb_serial.h"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

using namespace esp_usb;

static const char *TAG = "usb_serial";

static usb_serial_rx_cb_t s_rx_cb = nullptr;
static void *s_rx_cb_ctx = nullptr;

static SemaphoreHandle_t s_device_mutex = nullptr;
static cdc_acm_dev_hdl_t s_cdc_dev = nullptr;
static std::unique_ptr<CdcAcmDevice> s_vcp_dev;
static volatile bool s_device_connected = false;
static volatile uint32_t s_connect_gen;   /* Ender Smart (lote 11): +1 each time a device is opened */
static volatile bool s_is_uart_bridge = false;   /* lote 9: VCP (CH34x/CP210x/FTDI) open */

/* Semaphore given when USB Host lib detects a new device */
static SemaphoreHandle_t s_new_device_sem = nullptr;

/* ---- RX callback from CDC-ACM driver ---- */

static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    if (s_rx_cb && data_len > 0) {
        s_rx_cb(data, data_len, s_rx_cb_ctx);
    }
    return true;
}

/* ---- Device event callback ---- */

static void handle_device_event(const cdc_acm_host_dev_event_data_t *event,
                                 void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGW(TAG, "CDC-ACM error");
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "USB device disconnected");
        xSemaphoreTake(s_device_mutex, portMAX_DELAY);
        if (s_vcp_dev) {
            s_vcp_dev.reset();
        }
        s_cdc_dev = nullptr;
        s_device_connected = false;
        s_is_uart_bridge = false;
        xSemaphoreGive(s_device_mutex);
        break;
    default:
        break;
    }
}

/* ---- Try to open a connected USB device ---- */

static bool try_open_device(void)
{
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = handle_device_event,
        .data_cb = handle_rx,
        .user_arg = nullptr,
    };

    /* Try VCP drivers first (CH34x, CP210x, FTDI) */
    s_vcp_dev.reset(VCP::open(&dev_config));
    if (s_vcp_dev) {
        ESP_LOGI(TAG, "Opened VCP device");
        /* Set default line coding */
        cdc_acm_line_coding_t coding = {
            .dwDTERate = (uint32_t)CONFIG_PRINTER_BAUD_RATE,
            .bCharFormat = 0,  /* 1 stop bit */
            .bParityType = 0,  /* no parity */
            .bDataBits = 8,
        };
        s_vcp_dev->line_coding_set(&coding);
        s_vcp_dev->set_control_line_state(true, true);  /* DTR + RTS */
        s_is_uart_bridge = true;
        s_connect_gen = s_connect_gen + 1;
        s_device_connected = true;
        return true;
    }

    /* Try standard CDC-ACM */
    esp_err_t err = cdc_acm_host_open(0, 0, 0, &dev_config, &s_cdc_dev);
    if (err == ESP_OK && s_cdc_dev) {
        ESP_LOGI(TAG, "Opened CDC-ACM device");
        cdc_acm_line_coding_t coding = {
            .dwDTERate = (uint32_t)CONFIG_PRINTER_BAUD_RATE,
            .bCharFormat = 0,
            .bParityType = 0,
            .bDataBits = 8,
        };
        cdc_acm_host_line_coding_set(s_cdc_dev, &coding);
        cdc_acm_host_set_control_line_state(s_cdc_dev, true, true);
        s_is_uart_bridge = false;
        s_connect_gen = s_connect_gen + 1;
        s_device_connected = true;
        return true;
    }

    return false;
}

/* ---- USB Host library task ---- */

static void usb_host_lib_task(void *arg)
{
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGD(TAG, "No USB clients");
        }
    }
}

/* ---- CDC-ACM new device callback — wakes up the poll task ---- */

static void new_device_cb(usb_device_handle_t usb_dev)
{
    ESP_LOGI(TAG, "New USB device detected");
    xSemaphoreGive(s_new_device_sem);
}

/* ---- Device poll task — sleeps until a device is plugged in ---- */

static void usb_device_poll_task(void *arg)
{
    ESP_LOGI(TAG, "Waiting for USB device...");

    while (true) {
        if (s_device_connected) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Block until new_device_cb fires (no CPU usage while waiting) */
        if (xSemaphoreTake(s_new_device_sem, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(500));  /* Let enumeration complete */

            xSemaphoreTake(s_device_mutex, portMAX_DELAY);
            bool opened = try_open_device();
            xSemaphoreGive(s_device_mutex);

            if (opened) {
                ESP_LOGI(TAG, "Printer USB serial ready");
            } else {
                ESP_LOGW(TAG, "USB device detected but could not open");
            }
        }
    }
}

/* ---- Public API (extern "C") ---- */

extern "C" esp_err_t usb_serial_init(usb_serial_rx_cb_t rx_cb, void *rx_cb_ctx)
{
    s_rx_cb = rx_cb;
    s_rx_cb_ctx = rx_cb_ctx;
    s_device_mutex = xSemaphoreCreateMutex();
    s_new_device_sem = xSemaphoreCreateBinary();

    /* Install USB Host library */
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    /* Install CDC-ACM driver with new-device notification */
    const cdc_acm_host_driver_config_t acm_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 20,
        .xCoreID = 1,
        .new_dev_cb = new_device_cb,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&acm_config));

    /* Register VCP drivers */
    VCP::register_driver<CH34x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<FT23x>();

    ESP_LOGI(TAG, "USB Host initialized (CDC-ACM + CH34x + CP210x + FT23x)");

    /* Start USB host library event task */
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host_lib", 4096,
                            nullptr, 10, nullptr, 1);

    /* Start device poll task */
    xTaskCreatePinnedToCore(usb_device_poll_task, "usb_dev_poll", 4096,
                            nullptr, 15, nullptr, 1);

    return ESP_OK;
}

extern "C" bool usb_serial_is_uart_bridge(void)
{
    return s_device_connected && s_is_uart_bridge;
}

extern "C" esp_err_t usb_serial_send(const uint8_t *data, size_t len)
{
    xSemaphoreTake(s_device_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (s_vcp_dev) {
        ret = s_vcp_dev->tx_blocking(const_cast<uint8_t *>(data), len, 1000);
    } else if (s_cdc_dev) {
        ret = cdc_acm_host_data_tx_blocking(s_cdc_dev, data, len, 1000);
    }

    xSemaphoreGive(s_device_mutex);
    return ret;
}

extern "C" esp_err_t usb_serial_set_line_coding(uint32_t baud_rate,
                                                  uint8_t data_bits,
                                                  uint8_t parity,
                                                  uint8_t stop_bits)
{
    xSemaphoreTake(s_device_mutex, portMAX_DELAY);

    cdc_acm_line_coding_t coding = {
        .dwDTERate = baud_rate,
        .bCharFormat = stop_bits,
        .bParityType = parity,
        .bDataBits = data_bits,
    };

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (s_vcp_dev) {
        ret = s_vcp_dev->line_coding_set(&coding);
    } else if (s_cdc_dev) {
        ret = cdc_acm_host_line_coding_set(s_cdc_dev, &coding);
    }

    xSemaphoreGive(s_device_mutex);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Line coding: %lu baud, %d-%d-%d",
                 (unsigned long)baud_rate, data_bits, parity, stop_bits);
    }
    return ret;
}

extern "C" esp_err_t usb_serial_set_control_lines(bool dtr, bool rts)
{
    xSemaphoreTake(s_device_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (s_vcp_dev) {
        ret = s_vcp_dev->set_control_line_state(dtr, rts);
    } else if (s_cdc_dev) {
        ret = cdc_acm_host_set_control_line_state(s_cdc_dev, dtr, rts);
    }

    xSemaphoreGive(s_device_mutex);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Control lines: DTR=%d RTS=%d", dtr, rts);
    }
    return ret;
}

extern "C" bool usb_serial_is_connected(void)
{
    return s_device_connected;
}

extern "C" uint32_t usb_serial_connect_gen(void)
{
    return s_connect_gen;
}
