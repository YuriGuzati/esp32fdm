#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Ender Smart (lote 14): what happened in the run before this boot.
 *
 * On 2026-11-14 the board was off the network for ~6 min after the user restarted it, and the
 * cause could not be found: the log lives in RAM and is lost on a restart. Now each boot gets a
 * number (NVS), and a small record of the run (uptime, Wi-Fi, last disconnect reason, job) is
 * saved in NVS every minute for the first 10 min and every 10 min after; during a job only
 * together with the job journal (every 5 min), since a flash write stalls both cores. For resets
 * without a power loss (panic, watchdog, software) the whole record also lives in RTC memory,
 * updated every 5 s by an esp_timer.
 */

/** Call once, after nvs_flash_init() and before wifi_init(): numbers the boot, loads the
 *  previous run's record and saves this run's first one (so a run that dies at once shows up). */
void bootdiag_init(void);

/** Call after printer_comm_init(): starts the task that keeps the record up to date. */
void bootdiag_start(void);

/** Saves the record now (the job journal calls it: during a job the task does not write, and
 *  the journal already writes the NVS every 5 min). Must not be called with g_state_mutex held. */
void bootdiag_save_now(void);

/** This boot's number (1 = first boot with lote 14). */
uint32_t bootdiag_boot_count(void);

/** Appends `,"boot_count":N,"prev":{...}` (or `"prev":null`) to a JSON object being built.
 *  Returns the number of characters written (snprintf semantics, clamped to buf_size - 1). */
int bootdiag_json(char *buf, size_t buf_size);
