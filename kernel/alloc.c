/*
 * alloc.c - First-fit free-list dynamic allocator
 *
 * Each allocation is preceded by a 32-byte header (struct block_hdr).
 * Because the header is 32 bytes and allocations are rounded to 16-byte
 * multiples, every returned pointer is 16-byte aligned (the Limine memory
 * map entries are always page-aligned, so heap_base is at least 4 KiB
 * aligned).
 *
 * On free, the block is marked free and immediately coalesced with any
 * adjacent free neighbours so the heap does not fragment from repeated
 * alloc/free cycles.
 */

#include <stdint.h>
#include "alloc.h"

/* ── Block header (32 bytes, naturally aligned) ──────────────────────────── */

struct block_hdr {
    uint64_t          size;  /* usable bytes after this header */
    uint64_t          free;  /* 1 = free, 0 = in use */
    struct block_hdr *next;  /* next block in the address-ordered list */
    struct block_hdr *prev;  /* previous block */
};

#define HDR_SIZE   ((uint64_t)sizeof(struct block_hdr))   /* 32 */
#define ALIGN16(n) (((n) + 15u) & ~(uint64_t)15u)
#define MIN_SPLIT  64u   /* don't split unless remainder data >= this */

/* ── Heap state ──────────────────────────────────────────────────────────── */

static struct block_hdr *s_first = (void *)0;
static uint64_t          s_total = 0;

/* ── Public API ──────────────────────────────────────────────────────────── */

void kalloc_init(void *base, uint64_t size) {
    if (!base || size <= HDR_SIZE) return;
    s_first = (struct block_hdr *)base;
    s_first->size = size - HDR_SIZE;
    s_first->free = 1;
    s_first->next = (void *)0;
    s_first->prev = (void *)0;
    s_total = size;
}

void *kmalloc(uint64_t nbytes) {
    if (!s_first || nbytes == 0) return (void *)0;
    nbytes = ALIGN16(nbytes);

    for (struct block_hdr *b = s_first; b; b = b->next) {
        if (!b->free || b->size < nbytes)
            continue;

        /* Split if the leftover would fit a header plus MIN_SPLIT data. */
        if (b->size >= nbytes + HDR_SIZE + MIN_SPLIT) {
            struct block_hdr *n =
                (struct block_hdr *)((uint8_t *)(b + 1) + nbytes);
            n->size = b->size - nbytes - HDR_SIZE;
            n->free = 1;
            n->next = b->next;
            n->prev = b;
            if (n->next) n->next->prev = n;
            b->next = n;
            b->size = nbytes;
        }

        b->free = 0;
        return (void *)(b + 1);
    }
    return (void *)0;  /* out of memory */
}

void kfree(void *ptr) {
    if (!ptr) return;
    struct block_hdr *b = (struct block_hdr *)ptr - 1;
    b->free = 1;

    /* Coalesce with next free neighbour. */
    if (b->next && b->next->free) {
        b->size += HDR_SIZE + b->next->size;
        b->next  = b->next->next;
        if (b->next) b->next->prev = b;
    }
    /* Coalesce with previous free neighbour. */
    if (b->prev && b->prev->free) {
        b->prev->size += HDR_SIZE + b->size;
        b->prev->next  = b->next;
        if (b->next) b->next->prev = b->prev;
    }
}

void kalloc_stats(uint64_t *used_out, uint64_t *free_out, uint64_t *total_out) {
    uint64_t used = 0, free = 0;
    for (struct block_hdr *b = s_first; b; b = b->next) {
        if (b->free) free += b->size;
        else         used += b->size;
    }
    if (used_out)  *used_out  = used;
    if (free_out)  *free_out  = free;
    if (total_out) *total_out = s_total;
}
