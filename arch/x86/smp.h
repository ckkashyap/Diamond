/*
 * smp.h - Symmetric Multi-Processing public API
 */
#pragma once
#include <stdint.h>
#include "../../third_party/limine/limine.h"

#define SMP_MAX_CPUS  64

/*
 * Start all Application Processors found in the Limine SMP response.
 * Each AP enables SSE and enters an idle work loop.
 * Blocks until every AP reports online.
 */
void smp_init(struct limine_smp_response *resp);

/* Total number of logical CPUs (BSP + APs). */
int smp_cpu_count(void);

/* Index (0-based) of the calling CPU, identified by LAPIC ID. */
int smp_this_cpu(void);

/* LAPIC ID of CPU at index i. */
uint32_t smp_cpu_lapic(int i);

/*
 * Dispatch fn(arg) on CPU cpu_idx asynchronously.
 * Only one job can be queued per CPU at a time; call smp_wait() first
 * if the CPU might already be busy.
 */
void smp_dispatch(int cpu_idx, void (*fn)(void *), void *arg);

/* Spin until CPU cpu_idx has no pending job. */
void smp_wait(int cpu_idx);
