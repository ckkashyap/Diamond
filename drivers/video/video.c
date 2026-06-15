/*
 * video.c - Video subsystem: driver probe loop + 2D drawing primitives
 *
 * Stores one framebuffer description and dispatches drawing to it.
 * All pixel values are stored in the native framebuffer format using
 * the r/g/b shift values provided by Limine.
 */

#include <stdint.h>
#include "vid_drv.h"
#include "video.h"

/* ── Driver declarations ─────────────────────────────────────────────────── */

extern struct video_driver bga_driver;
extern struct video_driver virtio_gpu_driver;

/* Probe order: hardware-specific first; limine_fb_driver is the fallback. */
extern struct video_driver limine_fb_driver;

static struct video_driver * const s_drivers[] = {
    &bga_driver,
    &virtio_gpu_driver,
    &limine_fb_driver,   /* always succeeds */
};
#define N_DRIVERS ((int)(sizeof(s_drivers) / sizeof(s_drivers[0])))

/* ── Active framebuffer state ────────────────────────────────────────────── */

static struct video_driver *s_active = (void *)0;
static uint8_t  *s_fb    = (void *)0;
static uint32_t  s_w     = 0;
static uint32_t  s_h     = 0;
static uint32_t  s_pitch = 0;
static uint8_t   s_rsh   = 16;
static uint8_t   s_gsh   = 8;
static uint8_t   s_bsh   = 0;

/* ── Pixel composition ───────────────────────────────────────────────────── */

static inline uint32_t compose(uint32_t rgb) {
    uint8_t r = (uint8_t)((rgb >> 16) & 0xff);
    uint8_t g = (uint8_t)((rgb >>  8) & 0xff);
    uint8_t b = (uint8_t)((rgb >>  0) & 0xff);
    return ((uint32_t)r << s_rsh) |
           ((uint32_t)g << s_gsh) |
           ((uint32_t)b << s_bsh);
}

static inline uint32_t *pixel_ptr(int x, int y) {
    return (uint32_t *)(s_fb + (uint32_t)y * s_pitch + (uint32_t)x * 4u);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int video_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
               void *fb, uint32_t w, uint32_t h, uint32_t pitch,
               uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    s_fb    = (uint8_t *)fb;
    s_w     = w;
    s_h     = h;
    s_pitch = pitch;
    s_rsh   = r_shift;
    s_gsh   = g_shift;
    s_bsh   = b_shift;

    for (int i = 0; i < N_DRIVERS; i++) {
        void    *pfb   = fb;
        uint32_t pw    = w, ph = h, ppitch = pitch;
        if (s_drivers[i]->probe(hhdm_offset, kphys, kvirt,
                                &pfb, &pw, &ph, &ppitch,
                                r_shift, g_shift, b_shift)) {
            s_active = s_drivers[i];
            /* Update stored fb in case the driver changed the mode */
            s_fb    = (uint8_t *)pfb;
            s_w     = pw;
            s_h     = ph;
            s_pitch = ppitch;
            return (i < N_DRIVERS - 1) ? 1 : 0; /* 1=hw, 0=fallback */
        }
    }
    return 0;
}

const char *video_driver_name(void) {
    return s_active ? s_active->name : "none";
}
uint32_t video_width(void)  { return s_w; }
uint32_t video_height(void) { return s_h; }
void    *video_framebuffer(void) { return s_fb; }

int video_set_mode(uint32_t w, uint32_t h, uint32_t bpp) {
    if (!s_active || !s_active->set_mode) return -1;
    return s_active->set_mode(w, h, bpp);
}

void video_clear(uint32_t rgb) {
    uint32_t px = compose(rgb);
    for (uint32_t y = 0; y < s_h; y++) {
        uint32_t *row = (uint32_t *)(s_fb + y * s_pitch);
        for (uint32_t x = 0; x < s_w; x++) row[x] = px;
    }
}

void video_pixel(int x, int y, uint32_t rgb) {
    if ((uint32_t)x >= s_w || (uint32_t)y >= s_h) return;
    *pixel_ptr(x, y) = compose(rgb);
}

void video_hline(int x, int y, int w, uint32_t rgb) {
    if ((uint32_t)y >= s_h) return;
    uint32_t px = compose(rgb);
    int x1 = x < 0 ? 0 : x;
    int x2 = x + w;
    if ((uint32_t)x2 > s_w) x2 = (int)s_w;
    uint32_t *row = (uint32_t *)(s_fb + (uint32_t)y * s_pitch);
    for (int i = x1; i < x2; i++) row[i] = px;
}

void video_vline(int x, int y, int h, uint32_t rgb) {
    if ((uint32_t)x >= s_w) return;
    uint32_t px = compose(rgb);
    int y1 = y < 0 ? 0 : y;
    int y2 = y + h;
    if ((uint32_t)y2 > s_h) y2 = (int)s_h;
    for (int i = y1; i < y2; i++)
        *pixel_ptr(x, i) = px;
}

void video_rect(int x, int y, int w, int h, uint32_t rgb) {
    for (int row = y; row < y + h; row++)
        video_hline(x, row, w, rgb);
}

void video_rect_border(int x, int y, int w, int h, uint32_t rgb) {
    video_hline(x, y,         w, rgb);
    video_hline(x, y + h - 1, w, rgb);
    video_vline(x,         y, h, rgb);
    video_vline(x + w - 1, y, h, rgb);
}

void video_blit(int x, int y, int w, int h, const uint32_t *pixels) {
    for (int row = 0; row < h; row++) {
        int dy = y + row;
        if ((uint32_t)dy >= s_h) break;
        for (int col = 0; col < w; col++) {
            int dx = x + col;
            if ((uint32_t)dx < s_w)
                *pixel_ptr(dx, dy) = compose(pixels[row * w + col]);
        }
    }
}
