/*
 * smp.c - SMP initialisation via Limine goto_address protocol
 *
 * BSP writes goto_address on each AP entry in the Limine SMP response.
 * Each AP enables SSE, records itself online, then spins on a work slot.
 * The BSP waits until all APs are online before returning from smp_init().
 */

#include <stdint.h>
#include "../../third_party/limine/limine.h"
#include "cpu.h"
#include "smp.h"

/* ── Per-CPU state ───────────────────────────────────────────────────────── */

typedef void (*work_func_t)(void *);

struct cpu_slot {
    volatile int       online;
    uint32_t           lapic_id;
    /* Work queue: a single function + argument + done flag. */
    work_func_t volatile work_fn;
    void              *work_arg;
    volatile int       work_done;
};

static struct cpu_slot s_cpus[SMP_MAX_CPUS];
static int             s_ncpus = 1;     /* at least BSP */

/* ── AP entry point ──────────────────────────────────────────────────────── */

/* Called by Limine on each AP with the CPU's limine_smp_info pointer.
 * extra_argument carries the s_cpus[] index assigned by the BSP. */
static void ap_entry(struct limine_smp_info *info) {
    cpu_enable_sse();

    uint32_t idx = (uint32_t)(uintptr_t)info->extra_argument;
    s_cpus[idx].lapic_id = info->lapic_id;
    __sync_synchronize();
    s_cpus[idx].online = 1;           /* signal BSP we are alive */

    /* Work loop: idle until BSP dispatches a job. */
    for (;;) {
        work_func_t fn = s_cpus[idx].work_fn;
        if (fn) {
            void *arg = s_cpus[idx].work_arg;
            s_cpus[idx].work_fn = (work_func_t)0;
            __sync_synchronize();
            fn(arg);
            __sync_synchronize();
            s_cpus[idx].work_done = 1;
        }
        __asm__ volatile ("pause");
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void smp_init(struct limine_smp_response *resp) {
    if (!resp) return;

    int n = (int)resp->cpu_count;
    if (n > SMP_MAX_CPUS) n = SMP_MAX_CPUS;
    s_ncpus = n;

    for (int i = 0; i < n; i++) {
        struct limine_smp_info *cpu = resp->cpus[i];

        if (cpu->lapic_id == resp->bsp_lapic_id) {
            /* BSP is already running; just record its info. */
            s_cpus[i].lapic_id = cpu->lapic_id;
            s_cpus[i].online   = 1;
            continue;
        }

        /* Pass the slot index to the AP via extra_argument. */
        cpu->extra_argument = (uint64_t)(uintptr_t)i;
        __sync_synchronize();
        cpu->goto_address = ap_entry;   /* triggers AP startup */
    }

    /* Wait until every AP signals online. */
    for (int i = 0; i < n; i++) {
        while (!s_cpus[i].online)
            __asm__ volatile ("pause");
    }
}

int smp_cpu_count(void) { return s_ncpus; }

int smp_this_cpu(void) {
    uint32_t id = cpu_lapic_id();
    for (int i = 0; i < s_ncpus; i++)
        if (s_cpus[i].lapic_id == id) return i;
    return 0;
}

uint32_t smp_cpu_lapic(int i) {
    if (i < 0 || i >= s_ncpus) return 0;
    return s_cpus[i].lapic_id;
}

void smp_dispatch(int cpu_idx, void (*fn)(void *), void *arg) {
    if (cpu_idx <= 0 || cpu_idx >= s_ncpus) return;   /* 0 = BSP, skip */
    s_cpus[cpu_idx].work_done = 0;
    s_cpus[cpu_idx].work_arg  = arg;
    __sync_synchronize();
    s_cpus[cpu_idx].work_fn = (work_func_t)fn;
}

void smp_wait(int cpu_idx) {
    if (cpu_idx <= 0 || cpu_idx >= s_ncpus) return;
    while (s_cpus[cpu_idx].work_fn || !s_cpus[cpu_idx].work_done)
        __asm__ volatile ("pause");
}
