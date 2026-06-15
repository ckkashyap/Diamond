/*
 * vm.h - Bare-metal virtual-memory helpers for driver MMIO mapping
 *
 * Limine maps all physical memory WB (write-back cached) into the HHDM.  That
 * is fine for RAM but wrong for device MMIO: 64-byte cache-line fills on a
 * read can confuse peripherals (LPSS UART stalls indefinitely; LPSS HDA
 * returns stale data; etc).  ioremap_uc() walks the kernel page tables,
 * splitting any 1 GiB or 2 MiB huge page that covers the target address, and
 * forces the 4 KiB page to UC (PCD+PWT) so the CPU never caches it.
 *
 * Must be called after kalloc_init() — it uses kmalloc() for new page tables.
 */
#pragma once
#include <stdint.h>

/* Force the 4 KiB page at virtual `va` to UC, mapping to physical `phys`.
 * `hhdm` is the Limine HHDM offset (so we can read/write page tables). */
void ioremap_uc(uint64_t va, uint64_t phys, uint64_t hhdm);
