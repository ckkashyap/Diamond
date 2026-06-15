/*
 * terminal.c - Framebuffer + serial console (SMP-safe)
 *
 * All output is serialised with a spinlock so multiple CPU cores can call
 * term_putchar / term_puts without interleaving characters on screen or
 * corrupting the cursor state.
 *
 * Shadow framebuffer: once kalloc is up, term_enable_shadow() allocates a
 * WB-cached mirror of the framebuffer.  All rendering targets the shadow,
 * then a one-shot blit pushes the modified region out to the real (WC/UC)
 * framebuffer.  This turns scrolling from "MMIO→MMIO copy with slow reads"
 * into "RAM→RAM memmove + write-only blit".
 */

#include <stdint.h>
#include "font.h"
#include "serial.h"
#include "keyboard.h"
#include "../kernel/alloc.h"
#include "../arch/x86/spinlock.h"
#include "terminal.h"

/* ── Colours (0xRRGGBB) ──────────────────────────────────────────────────── */
#define TERM_BG  0x1e1e2eu
#define TERM_FG  0xcdd6f4u

/* ── Terminal font size ───────────────────────────────────────────────────── */
#define TERM_FONT_SIZE  64.0f

/* ── State ───────────────────────────────────────────────────────────────── */
static void    *t_fb;
static uint8_t *t_shadow;          /* NULL until term_enable_shadow() */
static uint64_t t_fb_w;
static uint64_t t_fb_h;
static uint64_t t_fb_pitch;
static uint8_t  t_rs, t_gs, t_bs;

static int t_cell_w;
static int t_cell_h;
static int t_ascent;
static int t_ncols;
static int t_nrows;
static int t_col;
static int t_row;

static spinlock_t t_lock = SPINLOCK_INIT;

/* ── Internal helpers ────────────────────────────────────────────────────── */

/* Rendering target: shadow if allocated, framebuffer otherwise. */
static void *rt(void) { return t_shadow ? (void *)t_shadow : t_fb; }

/* Copy `bytes` from src to dst (forward, 64-bit at a time + byte tail). */
static void fast_copy(void *dst, const void *src, uint64_t bytes) {
    uint64_t       *d = (uint64_t *)dst;
    const uint64_t *s = (const uint64_t *)src;
    uint64_t words = bytes >> 3;
    for (uint64_t i = 0; i < words; i++) d[i] = s[i];
    uint8_t       *d8 = (uint8_t *)dst + (words << 3);
    const uint8_t *s8 = (const uint8_t *)src + (words << 3);
    for (uint64_t i = 0; i < (bytes & 7); i++) d8[i] = s8[i];
}

/* Blit a rectangle from shadow to the real framebuffer.  No-op if no shadow
 * (the rendering already went straight to t_fb in that mode).               */
static void blit_rect(int x, int y, int w, int h) {
    if (!t_shadow) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if ((uint64_t)(x + w) > t_fb_w) w = (int)t_fb_w - x;
    if ((uint64_t)(y + h) > t_fb_h) h = (int)t_fb_h - y;
    if (w <= 0 || h <= 0) return;

    /* Full-width rows are contiguous → a single fast_copy covers them all. */
    if (x == 0 && (uint64_t)w == t_fb_w) {
        uint64_t off = (uint64_t)y * t_fb_pitch;
        fast_copy((uint8_t *)t_fb + off, t_shadow + off,
                  (uint64_t)h * t_fb_pitch);
        return;
    }

    uint64_t bytes = (uint64_t)w * 4u;
    for (int dy = 0; dy < h; dy++) {
        uint64_t off = (uint64_t)(y + dy) * t_fb_pitch + (uint64_t)x * 4u;
        fast_copy((uint8_t *)t_fb + off, t_shadow + off, bytes);
    }
}

static void scroll_up(void) {
    uint64_t row_b = (uint64_t)t_cell_h * t_fb_pitch;
    uint8_t *base  = (uint8_t *)rt();

    /* Scroll by shifting the render target up one row.  On shadow this is
     * WB→WB and tears along at L1 bandwidth; on the fb direct path this is
     * the old slow MMIO→MMIO copy.                                         */
    fast_copy(base, base + row_b, (uint64_t)(t_nrows - 1) * row_b);
    fb_fill_rect(base, t_fb_pitch,
                 0, (t_nrows - 1) * t_cell_h, (int)t_fb_w, t_cell_h,
                 t_rs, t_gs, t_bs, TERM_BG);

    /* Push the whole screen out to the real framebuffer in one shot. */
    blit_rect(0, 0, (int)t_fb_w, (int)t_fb_h);
    t_row = t_nrows - 1;
}

/* Core output routine — called with t_lock already held. */
static void putch(char c) {
    serial_putchar(c);   /* mirror to COM1 (handles its own \n→\r\n) */

    switch (c) {
    case '\n':
        t_col = 0;
        t_row++;
        if (t_row >= t_nrows) scroll_up();
        return;
    case '\r':
        t_col = 0;
        return;
    case '\b':
        if (t_col > 0) {
            t_col--;
            fb_fill_rect(rt(), t_fb_pitch,
                         t_col * t_cell_w, t_row * t_cell_h,
                         t_cell_w, t_cell_h,
                         t_rs, t_gs, t_bs, TERM_BG);
            blit_rect(t_col * t_cell_w, t_row * t_cell_h, t_cell_w, t_cell_h);
        }
        return;
    case '\t':
        do { putch(' '); } while (t_col & 7);   /* recursive, lock held */
        return;
    default:
        break;
    }

    if ((unsigned char)c < 32 || (unsigned char)c > 126) return;

    /* Erase cell before drawing (previous glyph may have had different width) */
    fb_fill_rect(rt(), t_fb_pitch,
                 t_col * t_cell_w, t_row * t_cell_h,
                 t_cell_w, t_cell_h,
                 t_rs, t_gs, t_bs, TERM_BG);

    render_char_at(rt(), t_fb_w, t_fb_h, t_fb_pitch,
                   t_rs, t_gs, t_bs,
                   c, TERM_FG,
                   t_col * t_cell_w,
                   t_row * t_cell_h + t_ascent);

    blit_rect(t_col * t_cell_w, t_row * t_cell_h, t_cell_w, t_cell_h);

    if (++t_col >= t_ncols) {
        t_col = 0;
        if (++t_row >= t_nrows) scroll_up();
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void term_init(const uint8_t *font_data,
               void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
               uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    serial_init();

    t_fb       = fb;
    t_shadow   = (uint8_t *)0;
    t_fb_w     = fb_w;
    t_fb_h     = fb_h;
    t_fb_pitch = fb_pitch;
    t_rs       = r_shift;
    t_gs       = g_shift;
    t_bs       = b_shift;

    font_bake(font_data, TERM_FONT_SIZE);

    t_cell_w = font_cell_w();
    t_cell_h = font_cell_h();
    t_ascent = font_ascent();
    t_ncols  = (int)(fb_w  / (uint64_t)t_cell_w);
    t_nrows  = (int)(fb_h  / (uint64_t)t_cell_h);
    t_col    = 0;
    t_row    = 0;

    term_clear();
}

void term_enable_shadow(void) {
    spin_lock(&t_lock);
    if (t_shadow) { spin_unlock(&t_lock); return; }

    uint64_t size = t_fb_h * t_fb_pitch;
    uint8_t *raw = (uint8_t *)kmalloc(size + 64);
    if (!raw) { spin_unlock(&t_lock); return; }

    t_shadow = (uint8_t *)(((uint64_t)raw + 63) & ~(uint64_t)63);

    /* Initialise shadow to a clean background, then clear the framebuffer
     * to match.  The cursor stays where it was — new output will land on
     * the shadow and be blitted normally.                                 */
    fb_fill_rect(t_shadow, t_fb_pitch, 0, 0, (int)t_fb_w, (int)t_fb_h,
                 t_rs, t_gs, t_bs, TERM_BG);
    blit_rect(0, 0, (int)t_fb_w, (int)t_fb_h);
    t_col = 0;
    t_row = 0;
    spin_unlock(&t_lock);
}

int term_cols(void) { return t_ncols; }
int term_rows(void) { return t_nrows; }

void term_resize(void *new_fb, uint64_t new_w, uint64_t new_h, uint64_t new_pitch) {
    spin_lock(&t_lock);
    t_fb       = new_fb;
    t_fb_w     = new_w;
    t_fb_h     = new_h;
    t_fb_pitch = new_pitch;
    t_ncols    = (int)(new_w / (uint64_t)t_cell_w);
    t_nrows    = (int)(new_h / (uint64_t)t_cell_h);
    t_col      = 0;
    t_row      = 0;

    /* Shadow tied to old dimensions is no longer valid.  Drop it; caller
     * can term_enable_shadow() again if they want the fast path back.    */
    t_shadow = (uint8_t *)0;
    spin_unlock(&t_lock);
    term_clear();
}

void term_clear(void) {
    spin_lock(&t_lock);
    fb_fill_rect(rt(), t_fb_pitch,
                 0, 0, (int)t_fb_w, (int)t_fb_h,
                 t_rs, t_gs, t_bs, TERM_BG);
    blit_rect(0, 0, (int)t_fb_w, (int)t_fb_h);
    t_col = 0;
    t_row = 0;
    spin_unlock(&t_lock);
}

void term_putchar(char c) {
    spin_lock(&t_lock);
    putch(c);
    spin_unlock(&t_lock);
}

void term_puts(const char *s) {
    spin_lock(&t_lock);
    for (; *s; s++) putch(*s);
    spin_unlock(&t_lock);
}

int term_getchar(void) {
    for (;;) {
        int c;
        c = kb_getchar();
        if (c != -1) return c;
        c = serial_getchar();
        if (c != -1) return c;
        __asm__ volatile ("pause");
    }
}
