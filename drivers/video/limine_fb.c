/*
 * limine_fb.c - Limine framebuffer fallback video driver
 *
 * Always succeeds; uses whatever framebuffer Limine provided without
 * talking to any hardware directly.  Placed last in the probe table so
 * hardware-specific drivers get priority.
 */

#include <stdint.h>
#include "vid_drv.h"

static int limine_fb_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                           void **fb, uint32_t *w, uint32_t *h, uint32_t *pitch,
                           uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    /* Nothing to do; all parameters are already set by the caller. */
    (void)hhdm_offset; (void)kphys; (void)kvirt;
    (void)fb; (void)w; (void)h; (void)pitch;
    (void)r_shift; (void)g_shift; (void)b_shift;
    return 1;
}

struct video_driver limine_fb_driver = {
    .name     = "limine-fb",
    .probe    = limine_fb_probe,
    .set_mode = (void *)0,
};
