/*
 * virtio_input.h — VirtIO input driver (tablet + keyboard)
 *
 * Provides absolute mouse coordinates and keyboard input via virtio-tablet-pci
 * and virtio-keyboard-pci devices.
 */
#pragma once
#include <stdint.h>

/*
 * Probe both virtio-tablet and virtio-keyboard PCI devices.
 * Must be called before mouse_poll() or kb_getchar() can use VirtIO input.
 * kphys / kvirt are the kernel physical base and virtual (HHDM) base.
 */
void virtio_input_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                       uint32_t screen_w, uint32_t screen_h);

/*
 * Poll for a new absolute mouse position/button state.
 * Returns 1 and fills *x, *y, *btns when new data is available; 0 otherwise.
 */
int  virtio_input_mouse_poll(int32_t *x, int32_t *y, uint8_t *btns);

/*
 * Poll for a key press from the VirtIO keyboard.
 * Returns ASCII character or -1 if nothing available.
 */
int  virtio_input_key_poll(void);

/* Returns 1 if a VirtIO tablet was found during init. */
int  virtio_input_has_tablet(void);

/* Returns 1 if the space bar is currently held on the VirtIO keyboard. */
int  virtio_input_space_down(void);

/* Drain the VirtIO keyboard event queue to update internal state (e.g.
 * space-key tracking) without consuming the character ring buffer. */
void virtio_input_update_kb(void);
