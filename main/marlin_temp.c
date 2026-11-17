/*
 * Ender Smart (F15-25, lote 17): reads a Marlin temperature report. See marlin_temp.h.
 *
 * Before this, any line with "T:" was read as one (strstr), and a report with no "/target"
 * left the target in an uninitialized local. On 2026-11-16 the M115 answer put the hotend at
 * 1 and 0 degC in the history ("EXTRUDER_COUNT:1", "Cap:REPEAT:0", ...) and its target at 2.
 */
#include "marlin_temp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Outside this a byte was lost: "195.27" without its '.' reads 19527. A broken thermistor
 * still fits (Marlin prints about -15, some sensors -273). */
#define TEMP_MIN (-300.0f)
#define TEMP_MAX 999.0f

/* One number right at p ("-14.5", "205.30", ".5"), within TEMP_MIN..TEMP_MAX: no spaces,
 * no "inf"/"nan". strtof would also take "0x1A" or "2e2"; Marlin never prints those. */
static bool read_number(const char *p, float *value, const char **end)
{
    if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '.')) return false;
    char *e;
    float v = strtof(p, &e);
    if (e == p || !isfinite(v) || v < TEMP_MIN || v > TEMP_MAX) return false;
    *value = v;
    *end = e;
    return true;
}

/* "<temp>" and an optional " /<target>" right after it; p points just past "X:".
 * False when the number, or a target after a '/', cannot be read. */
static bool read_pair(const char *p, float *actual, float *target, bool *has_target,
                      const char **end)
{
    if (!read_number(p, actual, &p)) return false;
    *has_target = false;
    const char *q = p;
    while (*q == ' ') q++;
    if (*q == '/') {
        if (!read_number(q + 1, target, &q)) return false;
        *has_target = true;
        p = q;
    }
    *end = p;
    return true;
}

bool marlin_parse_temp_report(const char *line, marlin_temps_t *out)
{
    if (!line || !out) return false;

    const char *p = line;
    while (*p == ' ') p++;
    if (p[0] == 'o' && p[1] == 'k' && p[2] == ' ') {
        p += 3;
        while (*p == ' ') p++;
    }
    if (p[0] != 'T' || p[1] != ':') return false;

    marlin_temps_t t;
    memset(&t, 0, sizeof(t));
    if (!read_pair(p + 2, &t.hotend, &t.hotend_target, &t.has_hotend_target, &p)) return false;

    /* The bed is its own field, after a space: " B:33.39 /0.00" (not "B@:0") */
    const char *b = strstr(p, " B:");
    if (b) {
        const char *e;
        if (!read_pair(b + 3, &t.bed, &t.bed_target, &t.has_bed_target, &e)) return false;
        t.has_bed = true;
    }

    /* A field that is there but cannot be read (a lost byte, most likely) refuses the whole
     * line: the next report comes in 2 s. */
    *out = t;
    return true;
}
