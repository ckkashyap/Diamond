/*
 * mouse.c — PS/2 mouse via i8042 auxiliary port (polling, no IRQ).
 *
 * Initialisation sequence:
 *   1. Enable aux device              (cmd 0xA8 → port 0x64)
 *   2. Read compaq status byte, enable IRQ12 + mouse clock, write back
 *   3. Reset mouse (0xFF) and wait for self-test 0xAA + device-ID 0x00
 *      This forces standard 3-byte mode; without it QEMU may leave the
 *      mouse in 4-byte IntelliMouse mode and every X/Y value is wrong.
 *   4. Set defaults (0xF6) then enable data reporting (0xF4)
 *
 * Packet format — standard PS/2, always 3 bytes after a reset:
 *   Byte 0: [YOVF][XOVF][YSGN][XSGN][1][MID][RGT][LFT]
 *   Byte 1: X displacement (unsigned; sign from byte-0 bit 4)
 *   Byte 2: Y displacement (unsigned; sign from byte-0 bit 5)
 *   PS/2 Y positive = mouse moved up ⟹ negate before adding to screen Y.
 *
 * Port 0x64 status bit 5 (MOBF) distinguishes mouse from keyboard bytes.
 */

#include <stdint.h>
#include "../arch/x86/io.h"
#include "mouse.h"
#include "virtio_input.h"

#define PS2_DATA   0x60u
#define PS2_STATUS 0x64u
#define PS2_CMD    0x64u

/* ── i8042 helpers ───────────────────────────────────────────────────────── */

static void ps2_wait_write(void) {
    int t = 100000;
    while ((inb(PS2_STATUS) & 0x02u) && --t);
}

/* Drain whatever is sitting in the output buffer (keyboard or mouse). */
static void ps2_drain(void) {
    int t = 32;
    while ((inb(PS2_STATUS) & 0x01u) && --t) (void)inb(PS2_DATA);
}

/*
 * Send one byte to the mouse (aux port) and consume the ACK (0xFA).
 * The i8042 requires writing 0xD4 to the command port first.
 */
static void ps2_mouse_cmd(uint8_t cmd) {
    ps2_wait_write(); outb(PS2_CMD,  0xD4u);
    ps2_wait_write(); outb(PS2_DATA, cmd);
    /* Wait for ACK. */
    int t = 100000;
    while (!(inb(PS2_STATUS) & 0x01u) && --t);
    if (inb(PS2_STATUS) & 0x01u) (void)inb(PS2_DATA);
}

/* Wait for one byte from the output buffer (either source) and discard it. */
static void ps2_eat(void) {
    int t = 200000;
    while (!(inb(PS2_STATUS) & 0x01u) && --t);
    if (inb(PS2_STATUS) & 0x01u) (void)inb(PS2_DATA);
}

/* ── Driver state ────────────────────────────────────────────────────────── */

static uint32_t s_sw  = 1280;
static uint32_t s_sh  = 800;
static int32_t  s_x   = 640;
static int32_t  s_y   = 400;

static uint8_t  s_pkt[3];
static int      s_idx = 0;
static int      s_live = 0;

/* ── Public API ──────────────────────────────────────────────────────────── */

void mouse_init(uint32_t screen_w, uint32_t screen_h) {
    s_sw  = screen_w;
    s_sh  = screen_h;
    s_x   = (int32_t)(screen_w / 2u);
    s_y   = (int32_t)(screen_h / 2u);
    s_idx = 0;

    ps2_drain();

    /* 1. Enable aux device. */
    ps2_wait_write(); outb(PS2_CMD, 0xA8u);
    ps2_drain();

    /* 2. Read compaq status byte, enable IRQ12, enable mouse clock. */
    ps2_wait_write(); outb(PS2_CMD, 0x20u);
    int t = 100000;
    while (!(inb(PS2_STATUS) & 0x01u) && --t);
    uint8_t cfg = (inb(PS2_STATUS) & 0x01u) ? inb(PS2_DATA) : 0x47u;
    cfg |=  0x02u;   /* enable IRQ12  */
    cfg &= ~0x20u;   /* enable mouse clock */
    ps2_wait_write(); outb(PS2_CMD,  0x60u);
    ps2_wait_write(); outb(PS2_DATA, cfg);

    /*
     * 3. Reset the mouse.
     * 0xFF → ACK (consumed by ps2_mouse_cmd) → 0xAA (self-test OK) → 0x00 (ID)
     * The reset forces standard 3-byte packets regardless of any previous state
     * (e.g. QEMU leaving the mouse in 4-byte IntelliMouse scroll-wheel mode).
     */
    ps2_mouse_cmd(0xFFu);
    ps2_eat();   /* 0xAA — self-test passed */
    ps2_eat();   /* 0x00 — device ID        */

    /* 4. Restore data reporting (reset disables it). */
    ps2_mouse_cmd(0xF6u);   /* set defaults       */
    ps2_mouse_cmd(0xF4u);   /* enable reporting   */

    s_live = 1;
}

int mouse_poll(int32_t *out_x, int32_t *out_y, uint8_t *out_btns) {
    /* Prefer VirtIO tablet (absolute, accurate) over PS/2 (relative). */
    if (virtio_input_has_tablet())
        return virtio_input_mouse_poll(out_x, out_y, out_btns);

    if (!s_live) return 0;

    /*
     * Read bytes that have MOBF (bit 5) set — these are from the mouse.
     * Bytes without bit 5 are keyboard data; the keyboard driver skips
     * bytes that DO have bit 5, so there is no race between the two.
     */
    while ((inb(PS2_STATUS) & 0x21u) == 0x21u) {
        uint8_t b = inb(PS2_DATA);

        /* Byte 0 of a PS/2 packet always has bit 3 set.  Re-sync if not. */
        if (s_idx == 0 && !(b & 0x08u)) continue;

        s_pkt[s_idx++] = b;
        if (s_idx < 3) continue;
        s_idx = 0;

        /* 9-bit signed X: magnitude in byte 1, sign in byte-0 bit 4. */
        int32_t dx =  (int32_t)s_pkt[1] - ((s_pkt[0] & 0x10u) ? 256 : 0);
        /* 9-bit signed Y: magnitude in byte 2, sign in byte-0 bit 5.
         * PS/2 Y+ = up on screen = decreasing screen-Y, so negate. */
        int32_t dy = -((int32_t)s_pkt[2] - ((s_pkt[0] & 0x20u) ? 256 : 0));

        s_x += dx;
        s_y += dy;
        if (s_x < 0)               s_x = 0;
        if (s_y < 0)               s_y = 0;
        if (s_x >= (int32_t)s_sw)  s_x = (int32_t)s_sw - 1;
        if (s_y >= (int32_t)s_sh)  s_y = (int32_t)s_sh - 1;

        *out_x    = s_x;
        *out_y    = s_y;
        *out_btns = s_pkt[0] & 0x07u;
        return 1;
    }
    return 0;
}
