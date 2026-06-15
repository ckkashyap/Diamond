/*
 * font.h - TrueType font renderer public API
 */
#pragma once
#include <stdint.h>

/*
 * Bake the font atlas at pixel_height.
 * Must be called once before any render_char_at or metric queries.
 * Resets the internal bump allocator.
 * Returns 1 on success, 0 on failure.
 */
int font_bake(const uint8_t *font_data, float pixel_height);

/* Metrics — valid after a successful font_bake() call. */
int font_cell_w(void);    /* monospace advance width, pixels */
int font_cell_h(void);    /* line height (baseline-to-baseline), pixels */
int font_ascent(void);    /* baseline offset from the top of a cell, pixels */

/*
 * Render one printable ASCII character onto a 32bpp framebuffer.
 *
 * px  : left edge of the character in framebuffer pixels
 * py  : baseline of the character in framebuffer pixels
 *        (= row * font_cell_h() + font_ascent())
 *
 * Returns the advance (always == font_cell_w()).
 */
int render_char_at(void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
                   uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                   char ch, uint32_t fg_color, int px, int py);

/*
 * Fill a screen rectangle with a solid 0xRRGGBB colour.
 */
void fb_fill_rect(void *fb, uint64_t fb_pitch,
                  int x, int y, int w, int h,
                  uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                  uint32_t color);

/*
 * Original string renderer (kept for reference; not used by the terminal).
 */
void render_text(const uint8_t *font_data,
                 void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
                 uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                 const char *text, float font_size,
                 uint32_t color, int cx, int cy);
