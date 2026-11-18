#pragma once

/*
 * Ender Smart (lote 19): previews of G-code files that carry no thumbnail.
 *
 * The page (/app) downloads the G-code, draws the extrusion moves into a PNG (or JPEG) and posts
 * it to POST /sd/preview. The image is kept on the LittleFS cache partition, not on the SD card,
 * keyed by the file name and size, so a new file with the same name but another size never shows
 * an old preview. /sd/info returns it when the file has no embedded thumbnail.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GCODE_PREVIEW_MAX_BYTES (160 * 1024)
#define GCODE_PREVIEW_MAX_SIDE  1024

/** "image/png" or "image/jpeg" if img starts like one and its size fits; NULL otherwise. */
const char *gcode_preview_mime(const uint8_t *img, size_t len);

/** Stores the preview of SD file `name` (size `file_size`). Replaces an older one atomically. */
esp_err_t gcode_preview_save(const char *name, long file_size, const uint8_t *img, size_t len);

/** The preview of `name` with this `file_size`, malloc'd (PSRAM), or NULL. Caller frees. */
uint8_t *gcode_preview_load(const char *name, long file_size, size_t *len);

/** Removes the preview of `name`, if any (the file was deleted or replaced). */
void gcode_preview_delete(const char *name);

#ifdef __cplusplus
}
#endif
