/*
 * Ender Smart (F15-25, lote 17): PC test of main/marlin_temp.c.
 *
 *   Windows (Developer Command Prompt):  cl /nologo /W4 /I..\..\main test_marlin_temp.c ..\..\main\marlin_temp.c
 *   gcc/clang:                           cc -Wall -Wextra -I../../main test_marlin_temp.c ../../main/marlin_temp.c -lm
 *
 *   test_marlin_temp                 runs the cases below
 *   test_marlin_temp lines.txt       also reads every line of the file and prints the ones
 *                                    that are accepted with no "B:" or refused with "T:" and "B:"
 */
#include "marlin_temp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_fail;

static bool near(float a, float b) { return fabsf(a - b) < 0.001f; }

/* expect: ok=accepted; then the values (NAN = "not in the line") */
static void check(const char *line, bool ok, float h, float ht, float b, float bt)
{
    marlin_temps_t t;
    memset(&t, 0x5a, sizeof(t));
    bool got = marlin_parse_temp_report(line, &t);
    bool pass = got == ok;
    if (pass && ok) {
        pass = near(t.hotend, h) &&
               (isnan(ht) ? !t.has_hotend_target : (t.has_hotend_target && near(t.hotend_target, ht))) &&
               (isnan(b) ? !t.has_bed : (t.has_bed && near(t.bed, b))) &&
               (isnan(bt) ? !t.has_bed_target : (t.has_bed_target && near(t.bed_target, bt)));
    }
    if (!pass) {
        s_fail++;
        printf("FAIL  \"%s\": got %d", line, got);
        if (got)
            printf(" T %.2f /%s%.2f B%s %.2f /%s%.2f", t.hotend, t.has_hotend_target ? "" : "(none)",
                   t.hotend_target, t.has_bed ? "" : "(none)", t.bed,
                   t.has_bed_target ? "" : "(none)", t.bed_target);
        printf("\n");
    } else {
        printf("ok    \"%s\"\n", line);
    }
}

int main(int argc, char **argv)
{
    const float N = NAN;

    /* Real lines from the Ender (registros/) */
    check("ok T:33.05 /0.00 B:33.39 /0.00 @:0 B@:0 FAN0@:0 FAN1@:0", true, 33.05f, 0, 33.39f, 0);
    check(" T:33.10 /0.00 B:33.36 /0.00 @:0 B@:0 FAN0@:0 FAN1@:0", true, 33.10f, 0, 33.36f, 0);
    check(" T:195.27 /200.00 B:60.03 /60.00 @:91 B@:23 FAN0@:0 FAN1@:0 W:3", true, 195.27f, 200, 60.03f, 60);
    check(" T:205.10 /210.00 B:60.00 /60.00 @:40 B@:0 FAN0@:0 FAN1@:0 W:?", true, 205.10f, 210, 60, 60);

    /* The M115 answer of 2026-11-16 (F15-25): none of these is a temperature */
    check("FIRMWARE_NAME:Marlin V1.4.7_G (Nov  8 2022 10:12:34) SOURCE_CODE_URL:github.com/MarlinFirmware/Marlin "
          "PROTOCOL_VERSION:1.0 MACHINE_TYPE:Ender-3 Max Neo EXTRUDER_COUNT:1 UUID:cede2a2f-41a2-4748-9b12-c55c62f367ff",
          false, 0, 0, 0, 0);
    check("Cap:RUNOUT:0", false, 0, 0, 0, 0);
    check("Cap:BUILD_PERCENT:0", false, 0, 0, 0, 0);
    check("Cap:PROMPT_SUPPORT:0", false, 0, 0, 0, 0);
    check("Cap:REPEAT:0", false, 0, 0, 0, 0);
    check("Cap:AUTOREPORT_TEMP:1", false, 0, 0, 0, 0);

    /* Other lines with "T:" that are not reports */
    check("echo:Unknown command: \"M999 T:5 B:6\"", false, 0, 0, 0, 0);
    check("echo:essync T:1", false, 0, 0, 0, 0);
    check("ok", false, 0, 0, 0, 0);
    check("ok N12 P15 B3", false, 0, 0, 0, 0);
    check("", false, 0, 0, 0, 0);
    check("T:", false, 0, 0, 0, 0);
    check("T: 20", false, 0, 0, 0, 0);      /* a space after the colon is not Marlin's format */
    check("T:-", false, 0, 0, 0, 0);
    check("T:.", false, 0, 0, 0, 0);
    check("T:nan /0 B:1 /0", false, 0, 0, 0, 0);
    check("T:inf", false, 0, 0, 0, 0);
    check("T:-nan", false, 0, 0, 0, 0);
    check("okT:20", false, 0, 0, 0, 0);
    check("ok  T:20.5 /0.00 B:21 /0.00", true, 20.5f, 0, 21, 0);

    /* No target: the caller keeps the old one (old Marlin "T:198.6 E:0 W:?") */
    check("T:198.6 E:0 W:?", true, 198.6f, N, N, N);
    check("ok T:20.0 B:21.0", true, 20, N, 21, N);
    check("T:200.0 E:0 B:60.0", true, 200, N, 60, N);
    check("ok T:0", true, 0, N, N, N);
    check(" ok T:20 /0 B:20 /0", true, 20, 0, 20, 0);
    check(" T:-14.00 /0.00 B:22.10 /0.00", true, -14, 0, 22.1f, 0);   /* broken thermistor */
    check(" T:-273.15 /0.00 B:22.10 /0.00", true, -273.15f, 0, 22.1f, 0);

    /* Other Marlin options: the extra fields are left alone */
    check(" T:20.00 /0.00 (4031.00) B:20.00 /0.00 (4012.00) @:0 B@:0", true, 20, 0, 20, 0);   /* SHOW_TEMP_ADC_VALUES */
    check(" T:20 /0 B:21 /0 T0:20 /0 T1:22 /0 @:0 B@:0 @0:0 @1:0", true, 20, 0, 21, 0);        /* 2 hotends */
    check("ok T:20.1 /0.0 B:20.0 /0.0 T0:20.1 /0.0 @:0 B@:0 P:0.0 A:25.9", true, 20.1f, 0, 20, 0);

    /* A field that is there but cannot be read refuses the whole line (a lost byte) */
    check(" T:19527 /200.00 B:60.03 /60.00 @:91 B@:23", false, 0, 0, 0, 0);   /* lost '.' */
    check(" T:195.27 /20000 B:60.03 /60.00 @:91 B@:23", false, 0, 0, 0, 0);
    check(" T:195.27 /200.00 B:6003 /60.00 @:91 B@:23", false, 0, 0, 0, 0);
    check(" T:195.27 /200.00 B:60.03 /6000 @:91 B@:23", false, 0, 0, 0, 0);
    check("ok T:20.0 /x B:21.0 /", false, 0, 0, 0, 0);
    check(" T:20.0 /0.0 B:", false, 0, 0, 0, 0);
    check(" T:20.0 /0.0 B:nan /0", false, 0, 0, 0, 0);

    /* The bed field needs a space before it ("B@:0" is not it) */
    check(" T:20.0 /0.0 B@:0", true, 20, 0, N, N);
    check(" T:20.0 /0.0B:30 /0", true, 20, 0, N, N);

    /* An M114 answer glued to a report (lost newline) is not a report: it goes to parse_m114 */
    check("X:1.00 Y:2.00 Z:0.30 E:0.00 Count X:80 Y:160 Z:120 T:33.10 /0.00 B:33.36 /0.00", false, 0, 0, 0, 0);

    if (argc > 1) {
        FILE *f = fopen(argv[1], "rb");
        if (!f) { printf("cannot open %s\n", argv[1]); return 2; }
        char buf[512];
        long n = 0, acc = 0, odd = 0;
        while (fgets(buf, sizeof(buf), f)) {
            buf[strcspn(buf, "\r\n")] = '\0';
            n++;
            marlin_temps_t t;
            bool got = marlin_parse_temp_report(buf, &t);
            bool looks = strstr(buf, "T:") && strstr(buf, " B:") && strstr(buf, "@:");
            if (got) acc++;
            if (got != looks || (got && (!t.has_hotend_target || !t.has_bed || !t.has_bed_target))) {
                odd++;
                printf("CORPUS %s \"%s\"\n", got ? "accepted" : "refused", buf);
            }
        }
        fclose(f);
        printf("corpus: %ld lines, %ld accepted, %ld to look at\n", n, acc, odd);
    }

    printf(s_fail ? "%d FAILED\n" : "all passed\n", s_fail);
    return s_fail ? 1 : 0;
}
