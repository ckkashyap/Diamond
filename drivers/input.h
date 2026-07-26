/*
 * input.h — Keyboard input-source descriptor.
 *
 * This mirrors the driver-table idiom used by net (nic.h), audio (aud_drv.h)
 * and video (vid_drv.h), but with one deliberate difference:
 *
 *   audio / video / net  →  probe all, keep the ONE that wins, dispatch to it.
 *   keyboard input       →  poll EVERY source, in priority order, on each
 *                           kb_getchar() — a machine may legitimately have
 *                           several at once (e.g. a Surface's SAM keyboard
 *                           plus a USB/virtio keyboard, or PS/2 alongside the
 *                           Hyper-V synthetic keyboard).
 *
 * So a source exposes only a non-blocking poll; there is no single "active"
 * source.  Each poll returns an ASCII byte or -1 when nothing is pending.
 *
 * Device bring-up stays per-driver (sam_init / virtio_input_init / hv_init /
 * the i8042 needs none), because each keyboard sits on very different
 * hardware; only the getchar path is unified through this descriptor.
 */
#pragma once

struct input_source {
    const char *name;
    /* Non-blocking: return an ASCII byte (>= 0), or -1 if nothing is pending. */
    int (*poll_char)(void);
};
