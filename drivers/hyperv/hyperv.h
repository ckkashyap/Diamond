/*
 * hyperv.h — Hyper-V VMBus synthetic keyboard (public API)
 *
 * Diamond runs as a Generation 2 (UEFI) Hyper-V guest, which exposes no PS/2
 * or USB keyboard: the only input device is the Hyper-V *Synthetic Keyboard*,
 * delivered over VMBus.  This driver is a minimal, fully-polled VMBus client
 * (Diamond has no IDT and runs with interrupts disabled) that brings up the
 * synthetic-keyboard channel and decodes keystroke packets to ASCII.
 *
 * All three entry points are safe to call unconditionally: if the kernel is
 * not running under Hyper-V (e.g. QEMU/KVM), hv_init() detects that via CPUID
 * and becomes a no-op, and hv_kbd_getchar() always returns -1.
 */
#pragma once
#include <stdint.h>

/*
 * Detect Hyper-V and, if present, bring up the VMBus synthetic keyboard.
 * hhdm/kphys/kvirt are the Limine HHDM offset and the kernel physical/virtual
 * bases (same values passed to virtio_input_init()); they are used to compute
 * guest-physical addresses of the DMA/hypercall pages.
 */
void hv_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);

/* Returns 1 if running under Hyper-V (valid after hv_init(), else 0). */
int hv_present(void);

/*
 * Poll the synthetic keyboard for one character.
 * Returns an ASCII byte (0..255) or -1 if nothing is available.
 */
int hv_kbd_getchar(void);

/*
 * Busy-delay for `ms` milliseconds using the Hyper-V partition reference
 * counter (exact, 100 ns resolution).  Returns 0 if the delay was performed,
 * or -1 if no Hyper-V reference time source is available (the caller should
 * then fall back to its own timing).  Safe to call unconditionally: off
 * Hyper-V it simply returns -1.  Needed because a Gen 2 guest has no working
 * 8254 PIT, so PIT-based delay loops would spin forever.
 */
int hv_delay_ms(uint32_t ms);
