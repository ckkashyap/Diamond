/*
 * terminal.h - Framebuffer + serial console public API
 */
#pragma once
#include <stdint.h>

/*
 * Initialize the terminal.
 * Bakes the font at TERM_FONT_SIZE, clears the screen to the background
 * colour, and positions the cursor at (0, 0).
 * Also initialises the serial port and the PS/2 keyboard driver.
 */
void term_init(const uint8_t *font_data,
               void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
               uint8_t r_shift, uint8_t g_shift, uint8_t b_shift);

/* Output a character to both the framebuffer and COM1.
 * Handles: '\n' (newline + scroll), '\r' (CR), '\b' (backspace+erase),
 *          '\t' (tab, 8-column), all printable ASCII. */
void term_putchar(char c);

/* Output a NUL-terminated string. */
void term_puts(const char *s);

/* Block until a character is available from the keyboard or serial port.
 * Returns the ASCII byte. */
int  term_getchar(void);

/* Clear the screen and home the cursor. */
void term_clear(void);

/* Update framebuffer dimensions after a mode change (e.g. BGA set_mode).
 * Recomputes the character grid, resets the cursor, and clears the screen. */
void term_resize(void *new_fb, uint64_t new_w, uint64_t new_h, uint64_t new_pitch);

/* Number of character columns and rows available. */
int  term_cols(void);
int  term_rows(void);

/* Allocate a WB-cached mirror of the framebuffer and make it the rendering
 * target; subsequent glyph draws, fills, and scrolls run in RAM and the
 * dirty region is blitted out.  Call once after kalloc_init() to turn
 * scrolling from MMIO-read-limited into RAM-bandwidth-limited.           */
void term_enable_shadow(void);
