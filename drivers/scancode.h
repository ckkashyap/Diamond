/*
 * scancode.h — Shared PS/2 scancode-set-1 → ASCII decoding.
 *
 * Three input backends observe the same AT / PS-2 "set 1" make-code space and
 * previously each carried a verbatim copy of the translation table + case rule:
 *   - keyboard.c     (i8042 PS/2 controller)
 *   - hyperv.c       (Hyper-V synthetic keyboard over VMBus)
 *   - virtio_input.c (virtio-keyboard; Linux evdev KEY_* codes coincide with
 *                     set-1 for the main block, so the table is identical)
 *
 * The tables and the Shift/Caps case rule now live here once.  Each driver
 * still tracks its own Shift/Caps state, because they observe make/break
 * events differently, but they share this decode.
 *
 * (sam.c is intentionally NOT a user: it decodes USB-HID usage codes, which
 * are a different keycode space and need their own table.)
 */
#pragma once
#include <stdint.h>

/* Unshifted / shifted set-1 make-code → ASCII (0 = no printable mapping). */
extern const char scancode_set1_lower[128];
extern const char scancode_set1_upper[128];

/*
 * Translate a set-1 make code to ASCII, applying Shift and Caps Lock.
 * For letter keys Caps Lock inverts Shift; for every other key Caps is
 * ignored.  Returns the ASCII byte, or 0 if the code has no printable mapping
 * (out-of-range codes included).
 */
char scancode1_to_ascii(uint8_t code, int shift, int caps);
