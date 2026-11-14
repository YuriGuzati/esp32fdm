#pragma once

#include "esp_http_server.h"
#include "layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mark current firmware as valid. Cancels automatic rollback if this is a new
 * OTA firmware.
 */
void ota_confirm_boot(void);

/**
 * Ender Smart (F15-13): log the running partition and its OTA state, then
 * confirm the image only after delay_s seconds of stable running. With
 * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, a new image that crashes or is reset
 * before that is rolled back by the bootloader. Call after the HTTP server and
 * the USB host are up.
 */
void ota_schedule_boot_confirm(uint32_t delay_s);

/**
 * Register OTA HTTP endpoints on the given server:
 *   POST /ota/upload  — receive firmware .bin and flash to next OTA slot
 */
void ota_register_httpd(httpd_handle_t server);

/** Ender Smart (lote 11): true while a firmware upload is being written (no print may start). */
bool ota_in_progress(void);

/**
 * Render the firmware update section for the settings page.
 */
void ota_render_settings(html_buf_t *p);

#ifdef __cplusplus
}
#endif
