/*
 * keyboard.c - PS/2 keyboard driver, scancode set 1, US QWERTY layout
 *
 * Polls the i8042 at I/O ports 0x60 / 0x64.
 * Handles Shift and Caps Lock; ignores key releases and extended codes.
 */

#include <stdint.h>
#include "../arch/x86/io.h"
#include "keyboard.h"
#include "input.h"
#include "scancode.h"
#include "virtio_input.h"
#include "sam/sam.h"
#include "hyperv/hyperv.h"

#define KB_DATA   0x60
#define KB_STATUS 0x64

static int s_shift = 0;
static int s_caps  = 0;
static int s_space_ps2 = 0;   /* 1 while space bar is physically held (PS/2) */

/* PS/2 i8042 keyboard: one non-blocking poll → ASCII byte or -1. */
static int ps2_kbd_poll(void) {
    uint8_t st = inb(KB_STATUS);
    if (!(st & 0x01u)) return -1;              /* output buffer empty */
    if (st & 0x20u) { (void)inb(KB_DATA); return -1; }  /* mouse byte — discard */

    uint8_t sc = inb(KB_DATA);

    /* Shift press / release */
    if (sc == 0x2A || sc == 0x36) { s_shift = 1; return -1; }
    if (sc == 0xAA || sc == 0xB6) { s_shift = 0; return -1; }

    /* Caps Lock toggle on press (0x3A); ignore release (0xBA) */
    if (sc == 0x3A) { s_caps ^= 1; return -1; }

    /* Space release (must be caught before the generic release handler) */
    if (sc == 0xB9) { s_space_ps2 = 0; return -1; }

    /* Skip other key releases (bit 7 set) and unmapped scancodes */
    if (sc & 0x80) return -1;
    if (sc >= 128)  return -1;

    /* Track space press */
    if (sc == 0x39) s_space_ps2 = 1;

    char c = scancode1_to_ascii(sc, s_shift, s_caps);
    return c ? (int)(unsigned char)c : -1;
}

/*
 * Keyboard input sources, polled in priority order by kb_getchar().  Unlike
 * the audio/video/net driver tables (which keep one active driver), every
 * source here coexists and is polled each call — see input.h.  Order: SAM
 * (Surface built-in) → VirtIO (USB-class) → Hyper-V synthetic (Gen 2 VMs:
 * no PS/2, no USB) → i8042 PS/2 port.  Each device is initialised separately
 * in kmain(); adding a keyboard source is now a one-line table entry.
 */
static const struct input_source s_kbd_sources[] = {
    { "sam",    sam_getchar           },
    { "virtio", virtio_input_key_poll },
    { "hyperv", hv_kbd_getchar        },
    { "ps2",    ps2_kbd_poll          },
};

int kb_getchar(void) {
    for (unsigned i = 0; i < sizeof(s_kbd_sources) / sizeof(s_kbd_sources[0]); i++) {
        int c = s_kbd_sources[i].poll_char();
        if (c >= 0) return c;
    }
    return -1;
}

int kb_space_held(void) {
    /* Drain PS/2 port, recording space press/release without losing other
     * characters (those are simply discarded — fine during audio playback). */
    for (;;) {
        uint8_t st = inb(KB_STATUS);
        if (!(st & 0x01u)) break;
        if (st & 0x20u) { (void)inb(KB_DATA); continue; }  /* mouse byte */
        uint8_t sc = inb(KB_DATA);
        if (sc == 0x39) s_space_ps2 = 1;   /* space press */
        if (sc == 0xB9) s_space_ps2 = 0;   /* space release */
    }
    /* Drain VirtIO keyboard events (updates s_vi_space; doesn't touch char buf) */
    virtio_input_update_kb();
    return s_space_ps2 || virtio_input_space_down();
}
