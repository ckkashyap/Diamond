/*
 * vm.c - Bare-metal VM helpers (see vm.h).
 */

#include <stdint.h>
#include "../../kernel/alloc.h"
#include "vm.h"

/* Allocate a 4 KiB-aligned zeroed page-table page from the heap. */
static uint64_t *alloc_pt_page(void) {
    uint8_t *raw = kmalloc(4096u + 4096u);
    uint64_t aligned = ((uint64_t)raw + 4095ULL) & ~4095ULL;
    uint64_t *pt = (uint64_t *)aligned;
    for (int i = 0; i < 512; i++) pt[i] = 0;
    return pt;
}

/* Split a 1 GiB huge page (bit 7 set in PDP entry) into 512 × 2 MiB pages. */
static void split_1g(uint64_t *pdp, int pdpi, uint64_t hhdm) {
    uint64_t e    = pdp[pdpi];
    uint64_t base = e & ~((1ULL << 30) - 1);
    uint64_t fl   = (e & 0xFFFu) | (1ULL << 7);
    uint64_t *pd  = alloc_pt_page();
    for (int i = 0; i < 512; i++)
        pd[i] = (base + (uint64_t)i * 0x200000u) | fl;
    pdp[pdpi] = ((uint64_t)pd - hhdm) | 3u;
}

/* Split a 2 MiB huge page (bit 7 set in PD entry) into 512 × 4 KiB pages. */
static void split_2m(uint64_t *pd, int pdi, uint64_t hhdm) {
    uint64_t e    = pd[pdi];
    uint64_t base = e & ~((1ULL << 21) - 1);
    uint64_t *pt  = alloc_pt_page();
    for (int i = 0; i < 512; i++)
        pt[i] = (base + (uint64_t)i * 0x1000u) | 3u;
    pd[pdi] = ((uint64_t)pt - hhdm) | 3u;
}

void ioremap_uc(uint64_t va, uint64_t phys, uint64_t hhdm) {
    int pml4i = (int)((va >> 39) & 0x1FF);
    int pdpi  = (int)((va >> 30) & 0x1FF);
    int pdi   = (int)((va >> 21) & 0x1FF);
    int pti   = (int)((va >> 12) & 0x1FF);

    __asm__ volatile ("wbinvd" ::: "memory");

    uint64_t cr3;
    __asm__ volatile ("mov %%cr3, %0" : "=r"(cr3));
    uint64_t *pml4 = (uint64_t *)((cr3 & ~0xFFFULL) + hhdm);

    if (!(pml4[pml4i] & 1u)) {
        uint64_t *p = alloc_pt_page();
        pml4[pml4i] = ((uint64_t)p - hhdm) | 3u;
    }
    uint64_t *pdp = (uint64_t *)((pml4[pml4i] & ~0xFFFULL) + hhdm);

    if (!(pdp[pdpi] & 1u)) {
        uint64_t *p = alloc_pt_page();
        pdp[pdpi] = ((uint64_t)p - hhdm) | 3u;
    } else if (pdp[pdpi] & (1ULL << 7)) {
        split_1g(pdp, pdpi, hhdm);
    }
    uint64_t *pd = (uint64_t *)((pdp[pdpi] & ~0xFFFULL) + hhdm);

    if (!(pd[pdi] & 1u)) {
        uint64_t *p = alloc_pt_page();
        pd[pdi] = ((uint64_t)p - hhdm) | 3u;
    } else if (pd[pdi] & (1ULL << 7)) {
        split_2m(pd, pdi, hhdm);
    }
    uint64_t *pt = (uint64_t *)((pd[pdi] & ~0xFFFULL) + hhdm);

    /* UC: PCD (bit 4) + PWT (bit 3) + present + writable */
    pt[pti] = (phys & ~0xFFFULL) | (1ULL << 4) | (1ULL << 3) | 3u;
    __asm__ volatile ("invlpg (%0)" :: "r"(va) : "memory");
}
