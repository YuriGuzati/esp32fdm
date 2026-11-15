#pragma once

/*
 * Ender Smart (F2-01, lote 15): state and functions shared by printer_comm.c, marlin_link.c
 * and command_arbiter.c after printer_comm.c was split. Internal: only these three files include
 * it. The variables kept their names, types and comments; only "static" went away.
 */

#include "marlin_proto.h"
#include "printer_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- printer_comm.c: configuration and polling ---- */

typedef enum {
    PAUSE_CMD_M25  = 0,  /* Pause SD print (can resume) */
    PAUSE_CMD_M524 = 1,  /* Abort SD print (workaround for buggy firmware) */
} pause_cmd_t;

extern printer_backend_t s_backend;
extern pause_cmd_t s_pause_cmd;
extern bool s_autotemp_sent;              /* M155 S2 sent on this connection */

/* ---- marlin_link.c ---- */

extern volatile uint32_t s_rx_dropped;    /* bytes the RX stream buffer had no room for */
extern bool s_pos_logged;                 /* first M114 answer after a (re)connect is logged */
extern char s_echo_ping_cmds[128];        /* GCode prefixes that need a CDC flush ping */
extern bool s_need_sync;                  /* lote 13: a command was given up */
extern bool s_mk_sent;                    /* lote 13: a marker is out */
extern bool s_mk_echo;                    /* lote 13: ... and its echo came */
extern int64_t s_last_auto_temp_us;       /* last unsolicited temperature line */
extern bool s_need_filename;              /* ask M27 C once an SD print starts */
extern StreamBufferHandle_t s_rx_stream;

/** F15-23: user/API command with line number and checksum (see marlin_link.c) */
bool marlin_send_checked(const char *gcode);

/* ---- command_arbiter.c ---- */

extern QueueHandle_t s_cmd_queue;
extern volatile bool s_estop;
extern volatile bool s_halted;
extern volatile uint32_t s_halt_gen;
extern volatile bool s_kill_seen;
extern volatile int  s_estop_mode;

/** The queue loops that were inside printer_comm_task() (see command_arbiter.c) */
void arbiter_drop_queue_estop(void);
void arbiter_drop_queue_halted(void);
void arbiter_drop_queue_disconnected(void);
void arbiter_dispatch_queue(void);

#ifdef __cplusplus
}
#endif
