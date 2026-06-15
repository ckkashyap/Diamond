/*
 * alloc.h - Dynamic memory allocator public API
 *
 * Backed by the largest USABLE physical region from the Limine memory map,
 * accessed via the HHDM.  kalloc_init() must be called once from kmain()
 * before any driver or application uses kmalloc/kfree.
 */
#pragma once
#include <stdint.h>

/* Initialise the heap at virtual address base, covering size bytes. */
void  kalloc_init(void *base, uint64_t size);

/* Allocate nbytes (rounded up to 16-byte alignment).  Returns NULL on OOM. */
void *kmalloc(uint64_t nbytes);

/* Release a pointer returned by kmalloc.  No-op on NULL. */
void  kfree(void *ptr);

/* Query heap usage.  Any output pointer may be NULL. */
void  kalloc_stats(uint64_t *used_out, uint64_t *free_out, uint64_t *total_out);
