#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Internal scan (used by host print start) ---- */

typedef struct {
    int32_t total_lines;
    float max_z;              /* highest Z from G0/G1 moves */
    float layer_height;       /* from slicer comment or computed */
    float first_layer_height; /* from slicer comment or first Z */
    int32_t total_layers;     /* from ;LAYER: comments */
    bool has_layer_markers;   /* Ender Smart (lote 6): ;LAYER: or ;LAYER_CHANGE seen */
} gcode_scan_t;

/**
 * Quick scan of an already-open GCode file: count lines, find max Z,
 * layer heights, total layers.  Rewinds file before and after.
 */
void gcode_scan_file(FILE *f, gcode_scan_t *out);

/* ---- Public file info scan (head + tail, cached) ---- */

typedef struct {
    int32_t total_layers;
    float layer_height;
    float first_layer_height;
    float max_z;
    int32_t est_time_s;        /* -1 if unknown */
    float filament_used_mm;    /* total mm, -1 if unknown */
    float filament_used_g;     /* total g, -1 if unknown */
    char *thumbnail_base64;    /* malloc'd, caller frees (NULL if none) */
    int thumb_w, thumb_h;      /* thumbnail dimensions */
} gcode_file_info_t;

/**
 * Scan a G-code file for metadata: layers, time, filament, thumbnail.
 * Scans head and tail of file for speed.  Results cached on LittleFS.
 * Caller must free thumbnail_base64.
 */
void gcode_scan_file_info(const char *path, gcode_file_info_t *out);

#ifdef __cplusplus
}
#endif
