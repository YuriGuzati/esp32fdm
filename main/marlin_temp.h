#pragma once

/*
 * Ender Smart (F15-25, lote 17): reads a Marlin temperature report.
 *
 * Plain C with no ESP-IDF headers, so it can be tested on a PC
 * (tools/host_test/test_marlin_temp.c).
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float hotend, hotend_target;
    float bed, bed_target;
    bool  has_hotend_target;   /* the line had " /<target>" after the hotend */
    bool  has_bed;             /* the line had " B:<temp>" */
    bool  has_bed_target;
} marlin_temps_t;

/**
 * True when the line is a temperature report, and fills *out.
 *
 * A report is a line whose first field is "T:<number>", after optional leading spaces and an
 * optional "ok " (the M105 answer):
 *   "ok T:33.05 /0.00 B:33.39 /0.00 @:0 B@:0"        M105
 *   " T:33.10 /0.00 B:33.36 /0.00 @:0 B@:0"          M155 autoreport
 *   " T:195.27 /200.00 B:60.03 /60.00 @:91 B@:23 W:3" while M109/M190 wait
 *
 * Lines that only contain "T:" somewhere are not reports: in the M115 answer,
 * "EXTRUDER_COUNT:1", "Cap:RUNOUT:0", "Cap:REPEAT:0" and the like.
 *
 * A target is only reported when the line has one; the caller keeps the old one otherwise.
 * The whole line is refused when a field it has cannot be read: a number that is not finite
 * or is outside -300..999 (a lost '.'), a '/' with no number, or " B:" with no number.
 */
bool marlin_parse_temp_report(const char *line, marlin_temps_t *out);

#ifdef __cplusplus
}
#endif
