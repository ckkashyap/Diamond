/*
 * vid_drv.h - Internal video driver descriptor
 *
 * Each driver defines one static `struct video_driver X_driver`.
 * video.c iterates the driver table calling probe() until one succeeds.
 * Not part of the public API — include video.h instead.
 */
#pragma once
#include <stdint.h>

struct video_driver {
    const char *name;
    /*
     * probe: detect hardware; may adjust fb/w/h/pitch if it sets a new mode.
     * Returns 1 on success (hardware found), 0 if not present.
     * *fb, *w, *h, *pitch are in/out: on entry they hold Limine's values;
     * the driver may overwrite them if it switches to a different mode.
     */
    int (*probe)(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                 void **fb, uint32_t *w, uint32_t *h, uint32_t *pitch,
                 uint8_t r_shift, uint8_t g_shift, uint8_t b_shift);
    /* Optional: change display resolution (may be NULL). */
    int (*set_mode)(uint32_t w, uint32_t h, uint32_t bpp);
};
