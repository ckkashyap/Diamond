/*
 * main.c - Diamond exokernel entry point
 *
 * Architecture (exokernel / OS-as-library):
 *   Everything runs at CPL 0.  Drivers are plain C libraries; the shell
 *   (the "application") calls them directly with no syscall boundary.
 *
 * Source layout:
 *   arch/x86/   — x86-specific: CPU, I/O ports, spinlock, PCI, SMP
 *   drivers/    — hardware drivers: terminal, serial, keyboard, font, net/
 *   apps/shell/ — the interactive shell demo application
 */

#include <stdint.h>
#include "../third_party/limine/limine.h"
#include "../arch/x86/cpu.h"
#include "../drivers/terminal.h"
#include "../arch/x86/smp.h"
#include "../drivers/net/net.h"
#include "../drivers/video/video.h"
#include "../drivers/audio/audio.h"
#include "../drivers/virtio_input.h"
#include "../drivers/sam/sam.h"
#include "../drivers/hyperv/hyperv.h"
#include "../apps/shell/shell.h"
#include "alloc.h"

/* ── Embedded font TTF (via objcopy) ─────────────────────────────────────── */
extern const uint8_t _binary_third_party_fonts_JetBrainsMono_Regular_ttf_start[];

/* ── Limine protocol requests ────────────────────────────────────────────── */

__attribute__((used, section(".limine_requests_start_marker")))
static volatile LIMINE_REQUESTS_START_MARKER;

__attribute__((used, section(".limine_requests")))
static volatile LIMINE_BASE_REVISION(2);

__attribute__((used, section(".limine_requests")))
static volatile struct limine_stack_size_request stack_req = {
    .id         = LIMINE_STACK_SIZE_REQUEST,
    .revision   = 0,
    .stack_size = 2 * 1024 * 1024,   /* 2 MiB — stb_truetype needs this */
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_framebuffer_request fb_req = {
    .id       = LIMINE_FRAMEBUFFER_REQUEST,
    .revision = 1,
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_hhdm_request hhdm_req = {
    .id       = LIMINE_HHDM_REQUEST,
    .revision = 0,
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_kernel_address_request kaddr_req = {
    .id       = LIMINE_KERNEL_ADDRESS_REQUEST,
    .revision = 0,
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_smp_request smp_req = {
    .id       = LIMINE_SMP_REQUEST,
    .revision = 0,
    .flags    = 0,
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_memmap_request memmap_req = {
    .id       = LIMINE_MEMMAP_REQUEST,
    .revision = 0,
};

__attribute__((used, section(".limine_requests")))
static volatile struct limine_module_request module_req = {
    .id       = LIMINE_MODULE_REQUEST,
    .revision = 0,
};

__attribute__((used, section(".limine_requests_end_marker")))
static volatile LIMINE_REQUESTS_END_MARKER;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void halt(void) {
    __asm__ volatile ("cli");
    for (;;) __asm__ volatile ("hlt");
}

/* ── Kernel entry ────────────────────────────────────────────────────────── */

/* Boot-time pointers that late-init drivers (e.g. on-demand iwlwifi from
 * a shell command) want access to.  Populated in kmain() before shell_run(). */
uint64_t g_hhdm_offset;
uint64_t g_kphys;
uint64_t g_kvirt;

/* Find a Limine module loaded with a given cmdline tag ("iwlwifi", ...).
 * Returns pointer + size through the out args, or returns 0 if not found. */
int limine_find_module(const char *cmdline_match,
                       const void **data_out, uint64_t *size_out) {
    *data_out = (void *)0;
    *size_out = 0;
    if (!module_req.response) return 0;
    for (uint64_t i = 0; i < module_req.response->module_count; i++) {
        struct limine_file *m = module_req.response->modules[i];
        if (!m || !m->cmdline) continue;
        /* match first run of chars */
        const char *a = cmdline_match, *b = m->cmdline;
        int ok = 1;
        while (*a && *b) { if (*a != *b) { ok = 0; break; } a++; b++; }
        if (ok && *a == 0) {
            *data_out = m->address;
            *size_out = m->size;
            return 1;
        }
    }
    return 0;
}

void kmain(void) {
    cpu_enable_sse();

    if (LIMINE_BASE_REVISION_SUPPORTED == 0) halt();
    if (!fb_req.response || fb_req.response->framebuffer_count == 0) halt();

    struct limine_framebuffer *fb = fb_req.response->framebuffers[0];
    term_init(
        _binary_third_party_fonts_JetBrainsMono_Regular_ttf_start,
        fb->address, fb->width, fb->height, fb->pitch,
        fb->red_mask_shift, fb->green_mask_shift, fb->blue_mask_shift
    );

    if (smp_req.response)
        smp_init(smp_req.response);

    /* ── Dynamic allocator ───────────────────────────────────────────────── */
    /* Find the largest USABLE physical region from the Limine memory map and
     * back the heap with it via the HHDM.  All physical memory is already
     * mapped at hhdm_offset, so no page-table work is needed. */
    if (memmap_req.response && hhdm_req.response) {
        struct limine_memmap_response *mm = memmap_req.response;
        uint64_t best_base = 0, best_len = 0;
        for (uint64_t i = 0; i < mm->entry_count; i++) {
            struct limine_memmap_entry *e = mm->entries[i];
            if (e->type == LIMINE_MEMMAP_USABLE && e->length > best_len) {
                best_base = e->base;
                best_len  = e->length;
            }
        }
        if (best_len) {
            kalloc_init((void *)(best_base + hhdm_req.response->offset),
                        best_len);
            /* With kalloc up, back the terminal with a WB shadow so glyph
             * rendering and scrolling don't pay MMIO read latency.       */
            term_enable_shadow();
        }
    }

    if (hhdm_req.response && kaddr_req.response) {
        g_hhdm_offset = hhdm_req.response->offset;
        g_kphys       = kaddr_req.response->physical_base;
        g_kvirt       = kaddr_req.response->virtual_base;

        net_init(
            hhdm_req.response->offset,
            kaddr_req.response->physical_base,
            kaddr_req.response->virtual_base
        );
        video_init(
            hhdm_req.response->offset,
            kaddr_req.response->physical_base,
            kaddr_req.response->virtual_base,
            fb->address,
            fb->width, fb->height, fb->pitch,
            fb->red_mask_shift, fb->green_mask_shift, fb->blue_mask_shift
        );
        /* If the video driver changed resolution (e.g. BGA set 1920×1080),
         * re-anchor the terminal to the new framebuffer and dimensions. */
        {
            uint32_t vw = video_width(), vh = video_height();
            if (vw != fb->width || vh != fb->height)
                term_resize(video_framebuffer(), vw, vh, (uint64_t)vw * 4u);
        }
        audio_init(
            hhdm_req.response->offset,
            kaddr_req.response->physical_base,
            kaddr_req.response->virtual_base
        );
        virtio_input_init(
            hhdm_req.response->offset,
            kaddr_req.response->physical_base,
            kaddr_req.response->virtual_base,
            video_width(), video_height()
        );
        sam_init(hhdm_req.response->offset);
        hv_init(
            hhdm_req.response->offset,
            kaddr_req.response->physical_base,
            kaddr_req.response->virtual_base
        );
    }

    shell_run();
    halt();
}
