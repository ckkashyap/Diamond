/*
 * video.h - Generic 2D framebuffer / video subsystem public API
 *
 * video_init() probes for hardware video drivers (BGA, VirtIO GPU).
 * All drawing calls operate on the active framebuffer, which is initially
 * the one provided by Limine.  If a hardware driver sets a new mode the
 * framebuffer pointer is updated automatically.
 */
#pragma once
#include <stdint.h>

/*
 * Initialise the video subsystem.
 * Must be called before any drawing or query functions.
 * hhdm_offset, kphys, kvirt — from Limine responses (for MMIO/DMA mapping).
 * fb / w / h / pitch / r_shift / g_shift / b_shift — Limine framebuffer info.
 * Returns 1 if a hardware driver was found, 0 if only the fallback is used.
 */
int video_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
               void *fb, uint32_t w, uint32_t h, uint32_t pitch,
               uint8_t r_shift, uint8_t g_shift, uint8_t b_shift);

/* Name of the active driver ("bga", "virtio-gpu", or "limine-fb"). */
const char *video_driver_name(void);

/* Current framebuffer dimensions and base pointer. */
uint32_t video_width(void);
uint32_t video_height(void);
void    *video_framebuffer(void);

/* Optional: change the display mode.  Returns 0 on success, -1 if not
 * supported by the active driver. */
int video_set_mode(uint32_t w, uint32_t h, uint32_t bpp);

/* ── 2D drawing primitives (all colours are 0x00RRGGBB) ─────────────────── */

/* Fill the entire framebuffer with one colour. */
void video_clear(uint32_t rgb);

/* Set a single pixel. No-op if (x,y) is out of range. */
void video_pixel(int x, int y, uint32_t rgb);

/* Horizontal line: y row, from x to x+w-1. */
void video_hline(int x, int y, int w, uint32_t rgb);

/* Vertical line: x column, from y to y+h-1. */
void video_vline(int x, int y, int h, uint32_t rgb);

/* Draw a filled rectangle. */
void video_rect(int x, int y, int w, int h, uint32_t rgb);

/* Draw a hollow rectangle (1-pixel border). */
void video_rect_border(int x, int y, int w, int h, uint32_t rgb);

/*
 * Blit a packed 0x00RRGGBB pixel array onto the framebuffer.
 * pixels[] has w×h entries in row-major order.
 */
void video_blit(int x, int y, int w, int h, const uint32_t *pixels);
