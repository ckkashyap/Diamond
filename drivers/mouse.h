/*
 * mouse.h — PS/2 mouse driver via i8042 auxiliary channel.
 *
 * The driver accumulates relative deltas into absolute screen coordinates,
 * clamped to [0, screen_w-1] × [0, screen_h-1].
 *
 * Call mouse_init() once after video is up.  Then call mouse_poll() in a
 * tight loop; it is non-blocking and returns 1 only when a full 3-byte
 * PS/2 packet has been assembled.
 */
#pragma once
#include <stdint.h>

/* Initialise the PS/2 mouse and set the clamp bounds. */
void mouse_init(uint32_t screen_w, uint32_t screen_h);

/*
 * Poll for a mouse event.
 * Returns 1 when a complete packet has been received, 0 otherwise.
 *   *out_x, *out_y  — absolute pixel position, already clamped to screen
 *   *out_btns       — button bitmask: bit 0=left, bit 1=right, bit 2=middle
 */
int mouse_poll(int32_t *out_x, int32_t *out_y, uint8_t *out_btns);
