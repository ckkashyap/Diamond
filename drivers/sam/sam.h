/*
 * sam.h — Surface Aggregator Module (SSAM/SSH) keyboard driver
 *
 * Public API: call sam_init() once, then poll sam_getchar().
 */
#pragma once
#include <stdint.h>

/*
 * Initialise the LPSS UART, configure SAM, and subscribe to keyboard events.
 * hhdm_offset: Limine higher-half direct mapping offset (from hhdm_req).
 */
void sam_init(uint64_t hhdm_offset);

/*
 * Poll for a decoded ASCII character from the SAM keyboard.
 * Returns the ASCII byte, or -1 if nothing is available.
 * Non-blocking; safe to call in a tight loop alongside other drivers.
 */
int sam_getchar(void);

/* Toggle verbose debug logging (TX/RX bytes, HID report hex dumps).  Off by
 * default; turn on via the shell when diagnosing keyboard issues. */
void sam_set_dbg(int on);
