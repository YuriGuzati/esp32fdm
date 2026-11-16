#pragma once

/* Ender Smart (lote 16, F3-04): WebSocket of events at /api/v1/events.
 *
 * The server pushes JSON text frames; it does not take commands over the socket:
 *   {"t":"hello","api":1,"boot":N,"fw":"...","tick_ms":500,...}  once, right after the handshake
 *   {"t":"status","d":{...}}   same object as GET /api/status; on change (checked every 500 ms),
 *                              and at least every 5 s
 *   {"t":"stats","d":{...}}    same object as GET /api/stats; every 5 s
 *   {"t":"temp","d":[ts,he,ht,be,bt]}  each new temperature sample, same row as GET /api/temp_history
 *   {"t":"full","max":4}       the client limit was reached; the socket is closed after it
 * A WebSocket PING goes out every 10 s; a client that answers nothing for 35 s is closed.
 * Everything that touches the sockets or builds the JSON runs in the HTTP server task. */

#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EVENTS_WS_MAX_CLIENTS    4
#define EVENTS_WS_TICK_MS        500
#define EVENTS_WS_HEARTBEAT_MS   5000
#define EVENTS_WS_STATS_MS       5000
#define EVENTS_WS_PING_MS        10000
#define EVENTS_WS_IDLE_CLOSE_MS  35000
#define EVENTS_WS_SEND_LIMIT_MS  500   /* per send call; a frame is 2 calls (header, payload) */

/** Registers GET /api/v1/events and starts the small task that paces the pushes. */
esp_err_t events_ws_register(httpd_handle_t server);

/** Number of accepted WebSocket clients right now. */
int events_ws_client_count(void);

/** Free stack (bytes, high-water mark) of the ws_events task, or -1 if it is not running. */
int events_ws_pacer_stack_free(void);

#ifdef __cplusplus
}
#endif
