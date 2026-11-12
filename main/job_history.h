#pragma once

/* Ender Smart (F3-06): history of host prints, kept on the LittleFS cache partition. */

#include "esp_err.h"
#include "esp_http_server.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *file;
    int64_t     start_epoch;    /* time(NULL) when the job started (0 = clock not set) */
    int32_t     duration_s;     /* printing time, pauses excluded */
    const char *result;         /* "completed", "cancelled" or "interrupted" */
    const char *reason;         /* why it stopped, "" if not applicable */
    int32_t     lines_sent, total_lines;
    int32_t     layer, total_layers;
} job_record_t;

/** Append one finished job (keeps the newest JOB_HISTORY_MAX). Called from the printer task. */
void job_history_add(const job_record_t *rec);

/** GET /api/history (newest first) and POST /api/history/clear */
esp_err_t job_history_register_httpd(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
