/*
 * keyboard.c - PS/2 keyboard driver, scancode set 1, US QWERTY layout
 *
 * Polls the i8042 at I/O ports 0x60 / 0x64.
 * Handles Shift and Caps Lock; ignores key releases and extended codes.
 */

#include <stdint.h>
#include "../arch/x86/io.h"
#include "keyboard.h"
#include "virtio_input.h"
#include "sam/sam.h"
#include "hyperv/hyperv.h"

#define KB_DATA   0x60
#define KB_STATUS 0x64

/* Unshifted scancode → ASCII (0 = no printable mapping) */
static const char sc_lower[128] = {
/*00*/ 0,  27, '1','2','3','4','5','6','7','8','9','0','-','=',  8, '\t',
/*10*/'q','w','e','r','t','y','u','i','o','p','[',']','\n',  0, 'a', 's',
/*20*/'d','f','g','h','j','k','l',';','\'','`',  0,'\\','z','x', 'c', 'v',
/*30*/'b','n','m',',','.','/',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

/* Shifted scancode → ASCII */
static const char sc_upper[128] = {
/*00*/ 0,  27, '!','@','#','$','%','^','&','*','(',')','_','+',  8, '\t',
/*10*/'Q','W','E','R','T','Y','U','I','O','P','{','}','\n',  0, 'A', 'S',
/*20*/'D','F','G','H','J','K','L',':','"', '~',  0, '|','Z','X', 'C', 'V',
/*30*/'B','N','M','<','>','?',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

static int s_shift = 0;
static int s_caps  = 0;
static int s_space_ps2 = 0;   /* 1 while space bar is physically held (PS/2) */

int kb_getchar(void) {
    /* Try SAM keyboard first (Surface Laptop 3 built-in keyboard). */
    int sam = sam_getchar();
    if (sam >= 0) return sam;

    /* Try VirtIO keyboard (USB-class input via virtio-keyboard-pci). */
    int vc = virtio_input_key_poll();
    if (vc >= 0) return vc;

    /* Try Hyper-V synthetic keyboard (Gen 2 VMs: no PS/2, no USB). */
    int hv = hv_kbd_getchar();
    if (hv >= 0) return hv;

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

    /* For letter keys, Caps Lock inverts the shift state */
    char base = sc_lower[sc];
    int use_shift = s_shift;
    if (base >= 'a' && base <= 'z') use_shift = s_shift ^ s_caps;

    char c = use_shift ? sc_upper[sc] : sc_lower[sc];
    return c ? (int)(unsigned char)c : -1;
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
