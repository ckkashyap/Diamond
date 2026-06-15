/*
 * cpu.h - per-CPU helpers: SSE enable, LAPIC ID read
 * Static-inline so any translation unit can use them without a link dep.
 */
#pragma once
#include <stdint.h>

/* Enable SSE/SSE2 on the current CPU core.
 * Must be called on EVERY core (BSP and each AP) before any floating-point
 * or XMM instruction is executed. */
static inline void cpu_enable_sse(void) {
    uint64_t cr0, cr4;
    __asm__ volatile ("mov %%cr0, %0" : "=r"(cr0));
    cr0 &= ~(1ULL << 2);   /* clear EM (emulation) */
    cr0 |=  (1ULL << 1);   /* set MP */
    __asm__ volatile ("mov %0, %%cr0" :: "r"(cr0));
    __asm__ volatile ("mov %%cr4, %0" : "=r"(cr4));
    cr4 |= (1ULL << 9);    /* OSFXSR     */
    cr4 |= (1ULL << 10);   /* OSXMMEXCPT */
    __asm__ volatile ("mov %0, %%cr4" :: "r"(cr4));
}

/* Read the LAPIC ID of the current CPU via CPUID leaf 1. */
static inline uint32_t cpu_lapic_id(void) {
    uint32_t eax, ebx, ecx, edx;
    __asm__ volatile ("cpuid"
        : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
        : "a"(1u), "c"(0u));
    return (ebx >> 24) & 0xff;
}
