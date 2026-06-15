/*
 * virtio_input.c — VirtIO input driver (legacy + modern PCI, tablet + keyboard)
 *
 * Supports both:
 *   Legacy (transitional): BAR0 = I/O port, PCI device ID 0x1012
 *   Modern (non-transitional): BAR0 = MMIO, PCI device ID 0x1052
 *
 * QEMU 8.x exposes virtio-tablet-pci / virtio-keyboard-pci as modern-only
 * (0x1052).  OVMF does not assign BAR addresses to these devices; we probe
 * the BAR size and assign MMIO addresses ourselves before mapping via the
 * Limine HHDM.
 */

#include <stdint.h>
#include "../arch/x86/pci.h"
#include "../arch/x86/io.h"
#include "virtio_input.h"

/* ── VirtIO PCI legacy register offsets (I/O port based) ────────────────── */
#define VIRTIO_DEV_FEATURES   0x00u
#define VIRTIO_DRV_FEATURES   0x04u
#define VIRTIO_QUEUE_ADDR     0x08u
#define VIRTIO_QUEUE_SIZE     0x0Cu
#define VIRTIO_QUEUE_SELECT   0x0Eu
#define VIRTIO_QUEUE_NOTIFY   0x10u
#define VIRTIO_DEV_STATUS     0x12u

/* VirtIO device status bits (same for legacy and modern) */
#define VIRTIO_S_ACK          1u
#define VIRTIO_S_DRIVER       2u
#define VIRTIO_S_DRIVER_OK    4u
#define VIRTIO_S_FEAT_OK      8u

/* VirtIO input legacy config space (at iobase+0x14) */
#define VCFG_SELECT   0x14u
#define VCFG_SUBSEL   0x15u
#define VCFG_SIZE     0x16u
#define VCFG_DATA     0x1Cu   /* 128-byte union */

/* Config select values */
#define VIRTIO_INPUT_CFG_EV_BITS   0x11u
#define VIRTIO_INPUT_CFG_ABS_INFO  3u

/* ABS_INFO offsets within the 128-byte union */
#define ABSINFO_MIN    0u
#define ABSINFO_MAX    4u

/* Linux input event types */
#define EV_SYN  0u
#define EV_KEY  1u
#define EV_ABS  3u

/* ABS axis codes */
#define ABS_X   0u
#define ABS_Y   1u

/* Mouse button codes */
#define BTN_LEFT   0x110u
#define BTN_RIGHT  0x111u
#define BTN_MIDDLE 0x112u

/* Virtqueue layout */
#define VRING_PAGE_BITS  12u
#define VRING_PAGE_SIZE  (1u << VRING_PAGE_BITS)
#define QUEUE_SIZE       16u   /* must be power of 2 */

struct __attribute__((packed)) vring_desc {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
};
#define VRING_DESC_F_WRITE  2u

struct __attribute__((packed)) vring_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[];
};

struct __attribute__((packed)) vring_used_elem {
    uint32_t id;
    uint32_t len;
};
struct __attribute__((packed)) vring_used {
    uint16_t flags;
    uint16_t idx;
    struct vring_used_elem ring[];
};

struct __attribute__((packed)) vi_event {
    uint16_t type;
    uint16_t code;
    uint32_t value;
};

/* ── Per-device state ────────────────────────────────────────────────────── */

#define QUEUE_BUF_BYTES  (VRING_PAGE_SIZE * 2u)

#define DEV_TABLET   0
#define DEV_KEYBOARD 1

static struct {
    int      ready;
    int      is_modern;   /* 0 = legacy I/O port, 1 = modern MMIO */

    /* Legacy only */
    uint16_t iobase;

    /* Modern only — MMIO pointers into HHDM */
    volatile uint8_t *common;         /* common config registers */
    volatile uint8_t *devcfg;         /* device-specific config */
    volatile uint8_t *notify_base;    /* notification region */
    uint16_t          notify_qoff;    /* queue_notify_off for eventq */
    uint32_t          notify_mult;    /* notify_off_multiplier */

    /* Virtqueue buffers (shared between legacy and modern) */
    uint8_t  mem[QUEUE_BUF_BYTES] __attribute__((aligned(4096)));
    struct vi_event bufs[QUEUE_SIZE];

    struct vring_desc  *desc;
    struct vring_avail *avail;
    struct vring_used  *used;

    uint16_t avail_idx;
    uint16_t last_used;
} s_dev[2];

/* ── Address translation helpers ─────────────────────────────────────────── */
static uint64_t s_hhdm;    /* Limine HHDM offset: phys → virt = phys + s_hhdm */
static uint64_t s_kphys;
static uint64_t s_kvirt;

/* Virtual kernel address → physical address (for DMA descriptors) */
static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

/* Physical address → HHDM virtual address (for MMIO register access) */
static volatile uint8_t *phys_to_mmio(uint64_t phys) {
    return (volatile uint8_t *)(uintptr_t)(phys + s_hhdm);
}

/* ── Global input state ──────────────────────────────────────────────────── */
static int      s_has_tablet   = 0;
static int      s_has_keyboard = 0;
static uint32_t s_screen_w     = 1280;
static uint32_t s_screen_h     = 800;
static uint32_t s_abs_max_x    = 32767;
static uint32_t s_abs_max_y    = 32767;

static int32_t  s_abs_x        = 0;
static int32_t  s_abs_y        = 0;
static uint8_t  s_btns         = 0;
static int      s_mouse_ready  = 0;

/* ── ASCII lookup table ──────────────────────────────────────────────────── */
static const char kc_lower[128] = {
/*00*/  0,  27,'1','2','3','4','5','6','7','8','9','0','-','=',  8,'\t',
/*10*/ 'q','w','e','r','t','y','u','i','o','p','[',']','\n',  0,'a','s',
/*20*/ 'd','f','g','h','j','k','l',';','\'','`',  0,'\\','z','x','c','v',
/*30*/ 'b','n','m',',','.','/',  0,'*',  0,' ',  0,  0,  0,  0,  0,  0,
/*40*/  0,  0,  0,  0,  0,  0,  0,'7','8','9','-','4','5','6','+','1',
/*50*/ '2','3','0','.',  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
static const char kc_upper[128] = {
/*00*/  0,  27,'!','@','#','$','%','^','&','*','(',')','_','+',  8,'\t',
/*10*/ 'Q','W','E','R','T','Y','U','I','O','P','{','}','\n',  0,'A','S',
/*20*/ 'D','F','G','H','J','K','L',':','"','~',  0,'|','Z','X','C','V',
/*30*/ 'B','N','M','<','>','?',  0,'*',  0,' ',  0,  0,  0,  0,  0,  0,
/*40*/  0,  0,  0,  0,  0,  0,  0,'7','8','9','-','4','5','6','+','1',
/*50*/ '2','3','0','.',  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
static int s_kb_shift = 0;
static int s_kb_caps  = 0;

/* ── MMIO BAR allocation (bump allocator above OVMF-assigned BARs) ───────── */
/* Use an address in the known PCI MMIO window (0x82200000 is above the last
 * OVMF-assigned BAR, ~0x820Axxxx, with a safe gap).                         */
static uint64_t s_mmio_alloc = 0x82200000u;

static uint64_t mmio_alloc_range(uint32_t size) {
    /* Align to size (always power-of-2 from BAR size calculation) */
    s_mmio_alloc = (s_mmio_alloc + (uint64_t)size - 1u) & ~((uint64_t)size - 1u);
    uint64_t p = s_mmio_alloc;
    s_mmio_alloc += size;
    return p;
}

/* Return the physical address for BAR `bar` (0-based), assigning one if 0. */
static uint64_t get_or_assign_bar(uint8_t pbus, uint8_t pdev, uint8_t pfn,
                                   uint8_t bar) {
    uint8_t reg = (uint8_t)(0x10u + (uint32_t)bar * 4u);
    uint32_t lo = pci_read32(pbus, pdev, pfn, reg);
    int is64 = ((lo & 0x6u) == 0x4u);
    uint64_t addr;

    if (is64) {
        uint32_t hi = pci_read32(pbus, pdev, pfn, reg + 4);
        addr = ((uint64_t)hi << 32) | (lo & ~0xFu);
    } else {
        addr = lo & ~0xFu;
    }

    /* Only keep the current address if it is non-zero AND fits in 32 bits
     * (i.e. within the low PCI MMIO window that Limine HHDM maps).
     * Addresses above 4 GiB live in the 64-bit PCIe window which Limine does
     * not include in the HHDM page tables — reassign to a 32-bit address. */
    if (addr && addr < 0x100000000ULL) return addr;

    /* Probe BAR size: write all-ones, read back size mask */
    pci_write32(pbus, pdev, pfn, reg, 0xFFFFFFFFu);
    uint32_t sz_mask = pci_read32(pbus, pdev, pfn, reg);
    pci_write32(pbus, pdev, pfn, reg, lo);   /* restore original */

    uint32_t size = ~(sz_mask & ~0xFu) + 1u;
    if (size < 4096u) size = 4096u;

    addr = mmio_alloc_range(size);

    /* Write the new 32-bit address; clear the upper BAR for 64-bit devices */
    pci_write32(pbus, pdev, pfn, reg, (uint32_t)addr | (lo & 0xFu));
    if (is64)
        pci_write32(pbus, pdev, pfn, (uint8_t)(reg + 4u), 0);

    return addr;
}

/* ── PCI capability list walker ──────────────────────────────────────────── */

/*
 * Find a VirtIO vendor-specific (cap_id=0x09) capability of cfg_type (1-5).
 * Fills *bar_out and *off_out with the BAR index and byte offset.
 * Returns the PCI config offset of the cap, or 0 if not found.
 */
static uint8_t find_vcap(uint8_t pbus, uint8_t pdev, uint8_t pfn,
                          uint8_t cfg_type,
                          uint8_t *bar_out, uint32_t *off_out) {
    /* Capabilities present bit in Status register */
    if (!(pci_read16(pbus, pdev, pfn, 0x06) & 0x10u)) return 0;

    /* First capability pointer is at config byte 0x34, bits [7:2] */
    uint8_t cap = (uint8_t)(pci_read32(pbus, pdev, pfn, 0x34) & 0xFCu);

    while (cap >= 0x40u) {
        uint32_t dw0 = pci_read32(pbus, pdev, pfn, cap);
        uint8_t cap_id   = (uint8_t)(dw0         & 0xFFu);
        uint8_t next_ptr = (uint8_t)((dw0 >>  8) & 0xFCu);
        uint8_t ctype    = (uint8_t)((dw0 >> 24) & 0xFFu);

        if (cap_id == 0x09u && ctype == cfg_type) {
            /* dw1: bar(8) | id(8) | padding(16) */
            uint32_t dw1 = pci_read32(pbus, pdev, pfn, cap + 4);
            *bar_out = (uint8_t)(dw1 & 0xFFu);
            /* dw2: offset(32) */
            *off_out = pci_read32(pbus, pdev, pfn, cap + 8);
            return cap;
        }
        cap = next_ptr;
    }
    return 0;
}

/* ── Virtqueue helpers ───────────────────────────────────────────────────── */

static void setup_queue(int d) {
    uint8_t *mem = s_dev[d].mem;
    s_dev[d].desc  = (struct vring_desc *)mem;
    s_dev[d].avail = (struct vring_avail *)(mem + QUEUE_SIZE * sizeof(struct vring_desc));
    uint64_t avail_end = (uint64_t)(uintptr_t)s_dev[d].avail + 6u + 2u * QUEUE_SIZE;
    uint64_t used_off  = (avail_end + VRING_PAGE_SIZE - 1u) & ~(uint64_t)(VRING_PAGE_SIZE - 1u);
    s_dev[d].used  = (struct vring_used *)used_off;
}

/* Fill eventq descriptors with device-writable vi_event buffers (legacy). */
static void fill_eventq_legacy(int d, uint16_t iobase) {
    for (uint16_t i = 0; i < QUEUE_SIZE; i++) {
        s_dev[d].desc[i].addr  = virt_to_phys(&s_dev[d].bufs[i]);
        s_dev[d].desc[i].len   = (uint32_t)sizeof(struct vi_event);
        s_dev[d].desc[i].flags = VRING_DESC_F_WRITE;
        s_dev[d].desc[i].next  = 0;
        s_dev[d].avail->ring[i] = i;
    }
    s_dev[d].avail->idx = QUEUE_SIZE;
    s_dev[d].avail_idx  = QUEUE_SIZE;
    s_dev[d].last_used  = 0;

    outw(iobase + VIRTIO_QUEUE_SELECT, 0);
    outl(iobase + VIRTIO_QUEUE_ADDR,
         (uint32_t)(virt_to_phys(s_dev[d].mem) >> VRING_PAGE_BITS));
}

/* Fill eventq and configure virtqueue via modern MMIO common config. */
static void fill_eventq_modern(int d) {
    for (uint16_t i = 0; i < QUEUE_SIZE; i++) {
        s_dev[d].desc[i].addr  = virt_to_phys(&s_dev[d].bufs[i]);
        s_dev[d].desc[i].len   = (uint32_t)sizeof(struct vi_event);
        s_dev[d].desc[i].flags = VRING_DESC_F_WRITE;
        s_dev[d].desc[i].next  = 0;
        s_dev[d].avail->ring[i] = i;
    }
    s_dev[d].avail->idx = QUEUE_SIZE;
    s_dev[d].avail_idx  = QUEUE_SIZE;
    s_dev[d].last_used  = 0;

    volatile uint8_t *cc = s_dev[d].common;

    /* Select queue 0 (eventq) */
    *(volatile uint16_t *)(cc + 0x16) = 0;
    __asm__ volatile ("" ::: "memory");

    /* No MSI-X */
    *(volatile uint16_t *)(cc + 0x1Au) = 0xFFFFu;

    /* Write 64-bit descriptor, driver (avail), and device (used) addresses */
    uint64_t desc_p  = virt_to_phys(s_dev[d].desc);
    uint64_t avail_p = virt_to_phys(s_dev[d].avail);
    uint64_t used_p  = virt_to_phys(s_dev[d].used);

    *(volatile uint32_t *)(cc + 0x20) = (uint32_t)desc_p;
    *(volatile uint32_t *)(cc + 0x24) = (uint32_t)(desc_p  >> 32);
    *(volatile uint32_t *)(cc + 0x28) = (uint32_t)avail_p;
    *(volatile uint32_t *)(cc + 0x2C) = (uint32_t)(avail_p >> 32);
    *(volatile uint32_t *)(cc + 0x30) = (uint32_t)used_p;
    *(volatile uint32_t *)(cc + 0x34) = (uint32_t)(used_p  >> 32);
    __asm__ volatile ("" ::: "memory");

    /* Save queue_notify_off for this queue (needed for notifications) */
    s_dev[d].notify_qoff = *(volatile uint16_t *)(cc + 0x1Eu);

    /* Enable queue */
    *(volatile uint16_t *)(cc + 0x1Cu) = 1;
    __asm__ volatile ("" ::: "memory");
}

/* ── Legacy device probe ─────────────────────────────────────────────────── */

static int probe_device_legacy(uint8_t pbus, uint8_t pdev, uint8_t pfn,
                                uint16_t iobase) {
    outb(iobase + VIRTIO_DEV_STATUS, 0);
    outb(iobase + VIRTIO_DEV_STATUS, VIRTIO_S_ACK | VIRTIO_S_DRIVER);
    outl(iobase + VIRTIO_DRV_FEATURES, 0);
    outb(iobase + VIRTIO_DEV_STATUS,
         VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK);

    /* Detect tablet (EV_ABS present) */
    outb(iobase + VCFG_SELECT, VIRTIO_INPUT_CFG_EV_BITS);
    outb(iobase + VCFG_SUBSEL, (uint8_t)EV_ABS);
    int is_tablet = (inb(iobase + VCFG_SIZE) > 0);

    uint32_t xmax = 0x7fffu, ymax = 0x7fffu;
    if (is_tablet) {
        outb(iobase + VCFG_SELECT, VIRTIO_INPUT_CFG_ABS_INFO);
        outb(iobase + VCFG_SUBSEL, ABS_X);
        if (inb(iobase + VCFG_SIZE)) {
            xmax = inl(iobase + VCFG_DATA + ABSINFO_MAX);
            if (!xmax) xmax = 0x7fffu;
        }
        outb(iobase + VCFG_SELECT, VIRTIO_INPUT_CFG_ABS_INFO);
        outb(iobase + VCFG_SUBSEL, ABS_Y);
        if (inb(iobase + VCFG_SIZE)) {
            ymax = inl(iobase + VCFG_DATA + ABSINFO_MAX);
            if (!ymax) ymax = 0x7fffu;
        }
    }

    int target = is_tablet ? DEV_TABLET : DEV_KEYBOARD;
    if (s_dev[target].ready) { outb(iobase + VIRTIO_DEV_STATUS, 0); return 0; }

    if (is_tablet) { s_abs_max_x = xmax; s_abs_max_y = ymax; }

    s_dev[target].iobase    = iobase;
    s_dev[target].is_modern = 0;

    outw(iobase + VIRTIO_QUEUE_SELECT, 0);
    if (!inw(iobase + VIRTIO_QUEUE_SIZE)) {
        outb(iobase + VIRTIO_DEV_STATUS, 0); return 0;
    }

    setup_queue(target);
    fill_eventq_legacy(target, iobase);
    outb(iobase + VIRTIO_DEV_STATUS,
         VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK | VIRTIO_S_DRIVER_OK);

    s_dev[target].ready = 1;
    (void)pbus; (void)pdev; (void)pfn;
    return 1;
}

/* ── Modern device probe (MMIO, VirtIO 1.0) ──────────────────────────────── */

static int probe_device_modern(uint8_t pbus, uint8_t pdev, uint8_t pfn) {
    uint8_t cc_bar, dc_bar, nc_bar;
    uint32_t cc_off, dc_off, nc_off;

    /* Walk PCI capability list to find the three required capabilities */
    uint8_t cc_cap = find_vcap(pbus, pdev, pfn, 1, &cc_bar, &cc_off); /* common */
    uint8_t dc_cap = find_vcap(pbus, pdev, pfn, 4, &dc_bar, &dc_off); /* device cfg */
    uint8_t nc_cap = find_vcap(pbus, pdev, pfn, 2, &nc_bar, &nc_off); /* notify */
    if (!cc_cap || !dc_cap || !nc_cap) return 0;

    /* notify_off_multiplier lives at cap+16 (an extra dword after the base struct) */
    uint32_t notify_mult = pci_read32(pbus, pdev, pfn, nc_cap + 16);

    /* Ensure each referenced BAR has a physical address assigned */
    uint64_t cc_phys = get_or_assign_bar(pbus, pdev, pfn, cc_bar);
    uint64_t dc_phys = (dc_bar == cc_bar) ? cc_phys
                     : get_or_assign_bar(pbus, pdev, pfn, dc_bar);
    uint64_t nc_phys = (nc_bar == cc_bar) ? cc_phys
                     : get_or_assign_bar(pbus, pdev, pfn, nc_bar);
    if (!cc_phys || !dc_phys || !nc_phys) return 0;

    /* Enable memory space decode now that BARs are populated */
    uint16_t cmd = pci_read16(pbus, pdev, pfn, 0x04);
    pci_write16(pbus, pdev, pfn, 0x04, (uint16_t)(cmd | 0x0006u));

    /* Map regions via HHDM */
    volatile uint8_t *common = phys_to_mmio(cc_phys + cc_off);
    volatile uint8_t *devcfg = phys_to_mmio(dc_phys + dc_off);
    volatile uint8_t *notify = phys_to_mmio(nc_phys + nc_off);

    /* ── VirtIO modern init sequence ── */

    /* 1. Reset */
    *(volatile uint8_t *)(common + 0x14) = 0;
    __asm__ volatile ("" ::: "memory");

    /* 2. ACK + DRIVER */
    *(volatile uint8_t *)(common + 0x14) = VIRTIO_S_ACK | VIRTIO_S_DRIVER;
    __asm__ volatile ("" ::: "memory");

    /* 3. Feature negotiation: must accept VIRTIO_F_VERSION_1 (bit 32) */
    *(volatile uint32_t *)(common + 0x00) = 1;   /* device_feature_select = page 1 */
    __asm__ volatile ("" ::: "memory");
    uint32_t dev_feat_hi = *(volatile uint32_t *)(common + 0x04);
    uint32_t drv_feat_hi = dev_feat_hi & 0x1u;   /* accept only VERSION_1 */

    *(volatile uint32_t *)(common + 0x08) = 0;   /* driver_feature_select = page 0 */
    *(volatile uint32_t *)(common + 0x0C) = 0;   /* accept no page-0 features */
    *(volatile uint32_t *)(common + 0x08) = 1;   /* driver_feature_select = page 1 */
    *(volatile uint32_t *)(common + 0x0C) = drv_feat_hi;
    __asm__ volatile ("" ::: "memory");

    /* 4. FEATURES_OK */
    *(volatile uint8_t *)(common + 0x14) =
        VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK;
    __asm__ volatile ("" ::: "memory");
    if (!(*(volatile uint8_t *)(common + 0x14) & VIRTIO_S_FEAT_OK)) return 0;

    /* 5. No MSI-X on config */
    *(volatile uint16_t *)(common + 0x10) = 0xFFFFu;

    /* 6. Detect tablet vs keyboard via device config EV_BITS */
    *(volatile uint8_t *)(devcfg + 0) = VIRTIO_INPUT_CFG_EV_BITS;
    *(volatile uint8_t *)(devcfg + 1) = (uint8_t)EV_ABS;
    __asm__ volatile ("" ::: "memory");
    int is_tablet = (*(volatile uint8_t *)(devcfg + 2) > 0);

    /* Use the standard VirtIO tablet range (0..32767) which matches the QMP
     * input-send-event range.  Attempting to read abs_max from the device
     * config MMIO returns garbage after BAR reassignment in some QEMU builds,
     * so we rely on the well-known default instead.                           */
    uint32_t xmax = 0x7fffu, ymax = 0x7fffu;

    int target = is_tablet ? DEV_TABLET : DEV_KEYBOARD;
    if (s_dev[target].ready) return 0;

    if (is_tablet) { s_abs_max_x = xmax; s_abs_max_y = ymax; }

    s_dev[target].is_modern    = 1;
    s_dev[target].common       = common;
    s_dev[target].devcfg       = devcfg;
    s_dev[target].notify_base  = notify;
    s_dev[target].notify_mult  = notify_mult;

    /* 7. Check and configure queue 0 */
    *(volatile uint16_t *)(common + 0x16) = 0;   /* queue_select = 0 */
    __asm__ volatile ("" ::: "memory");
    uint16_t qsz = *(volatile uint16_t *)(common + 0x18);
    if (!qsz) return 0;
    if (qsz > QUEUE_SIZE)
        *(volatile uint16_t *)(common + 0x18) = QUEUE_SIZE;

    setup_queue(target);
    fill_eventq_modern(target);

    /* 8. DRIVER_OK */
    *(volatile uint8_t *)(common + 0x14) =
        VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK | VIRTIO_S_DRIVER_OK;
    __asm__ volatile ("" ::: "memory");

    s_dev[target].ready = 1;
    return 1;
}

/* ── Top-level probe dispatcher ──────────────────────────────────────────── */

static int probe_device(uint8_t pbus, uint8_t pdev, uint8_t pfn) {
    /* Enable I/O + Bus mastering (memory space enabled per-path below) */
    uint16_t cmd = pci_read16(pbus, pdev, pfn, 0x04);
    pci_write16(pbus, pdev, pfn, 0x04, (uint16_t)(cmd | 0x0005u));

    uint32_t bar0 = pci_read32(pbus, pdev, pfn, 0x10);

    if (bar0 & 0x1u) {
        /* Legacy I/O BAR */
        return probe_device_legacy(pbus, pdev, pfn, (uint16_t)(bar0 & ~0x3u));
    }

    /* No I/O BAR → try modern MMIO (assigns BAR if needed) */
    return probe_device_modern(pbus, pdev, pfn);
}

/* ── Drain helpers (inline notification) ─────────────────────────────────── */

static void notify_queue(int d) {
    if (s_dev[d].is_modern) {
        uint32_t off = (uint32_t)s_dev[d].notify_qoff * s_dev[d].notify_mult;
        *(volatile uint16_t *)(s_dev[d].notify_base + off) = 0;  /* queue 0 */
    } else {
        outw(s_dev[d].iobase + VIRTIO_QUEUE_NOTIFY, 0);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void virtio_input_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                       uint32_t screen_w, uint32_t screen_h) {
    s_hhdm     = hhdm_offset;
    s_kphys    = kphys;
    s_kvirt    = kvirt;
    s_screen_w = screen_w;
    s_screen_h = screen_h;

    uint8_t bus, pdev, fn;

    /* Try transitional (legacy) device ID 0x1012 first */
    uint8_t after = 0xFF;
    for (int i = 0; i < 2; i++) {
        if (!pci_find_next(0x1AF4u, 0x1012u, after, &bus, &pdev, &fn)) break;
        probe_device(bus, pdev, fn);
        after = pdev;
    }

    /* Then try non-transitional (modern) device ID 0x1052 */
    after = 0xFF;
    for (int i = 0; i < 2; i++) {
        if (!pci_find_next(0x1AF4u, 0x1052u, after, &bus, &pdev, &fn)) break;
        probe_device(bus, pdev, fn);
        after = pdev;
    }

    s_has_tablet   = s_dev[DEV_TABLET].ready;
    s_has_keyboard = s_dev[DEV_KEYBOARD].ready;

    s_abs_x = (int32_t)(screen_w / 2u);
    s_abs_y = (int32_t)(screen_h / 2u);
}

int virtio_input_has_tablet(void) {
    return s_has_tablet;
}

/* ── Event draining ──────────────────────────────────────────────────────── */

static void drain_tablet(void) {
    int d = DEV_TABLET;
    if (!s_dev[d].ready) return;

    while (s_dev[d].last_used != s_dev[d].used->idx) {
        __asm__ volatile ("" ::: "memory");

        uint16_t slot = (uint16_t)(
            s_dev[d].used->ring[s_dev[d].last_used % QUEUE_SIZE].id % QUEUE_SIZE);
        s_dev[d].last_used++;

        struct vi_event *ev = &s_dev[d].bufs[slot];
        __asm__ volatile ("" ::: "memory");

        uint16_t ev_type  = ev->type;
        uint16_t ev_code  = ev->code;
        uint32_t ev_value = ev->value;

        if (ev_type == EV_ABS) {
            if (ev_code == ABS_X) {
                s_abs_x = (int32_t)(
                    ((uint64_t)ev_value * s_screen_w) / (s_abs_max_x + 1u));
                if (s_abs_x < 0) s_abs_x = 0;
                if (s_abs_x >= (int32_t)s_screen_w)
                    s_abs_x = (int32_t)s_screen_w - 1;
                s_mouse_ready = 1;
            } else if (ev_code == ABS_Y) {
                s_abs_y = (int32_t)(
                    ((uint64_t)ev_value * s_screen_h) / (s_abs_max_y + 1u));
                if (s_abs_y < 0) s_abs_y = 0;
                if (s_abs_y >= (int32_t)s_screen_h)
                    s_abs_y = (int32_t)s_screen_h - 1;
                s_mouse_ready = 1;
            }
        } else if (ev_type == EV_KEY) {
            if (ev_code == BTN_LEFT)
                s_btns = ev_value ? (s_btns | 0x01u) : (s_btns & ~0x01u);
            else if (ev_code == BTN_RIGHT)
                s_btns = ev_value ? (s_btns | 0x02u) : (s_btns & ~0x02u);
            else if (ev_code == BTN_MIDDLE)
                s_btns = ev_value ? (s_btns | 0x04u) : (s_btns & ~0x04u);
        } else if (ev_type == EV_SYN) {
            s_mouse_ready = 1;
        }

        /* Return descriptor to available ring */
        s_dev[d].desc[slot].addr  = virt_to_phys(&s_dev[d].bufs[slot]);
        s_dev[d].desc[slot].len   = (uint32_t)sizeof(struct vi_event);
        s_dev[d].desc[slot].flags = VRING_DESC_F_WRITE;
        s_dev[d].avail->ring[s_dev[d].avail_idx % QUEUE_SIZE] = slot;
        s_dev[d].avail_idx++;
        s_dev[d].avail->idx = s_dev[d].avail_idx;
        notify_queue(d);
    }
}

int virtio_input_mouse_poll(int32_t *x, int32_t *y, uint8_t *btns) {
    if (!s_has_tablet) return 0;
    drain_tablet();
    if (!s_mouse_ready) return 0;
    s_mouse_ready = 0;
    *x    = s_abs_x;
    *y    = s_abs_y;
    *btns = s_btns;
    return 1;
}

/* ── Keyboard ──────────────────────────────────────────────────────────────── */

#define KEY_BUF 16
static char s_key_buf[KEY_BUF];
static int  s_key_r = 0, s_key_w = 0;
static int  s_vi_space = 0;

static void drain_keyboard(void) {
    int d = DEV_KEYBOARD;
    if (!s_dev[d].ready) return;

    while (s_dev[d].last_used != s_dev[d].used->idx) {
        __asm__ volatile ("" ::: "memory");

        uint16_t slot = (uint16_t)(
            s_dev[d].used->ring[s_dev[d].last_used % QUEUE_SIZE].id % QUEUE_SIZE);
        s_dev[d].last_used++;

        struct vi_event *ev = &s_dev[d].bufs[slot];
        __asm__ volatile ("" ::: "memory");

        if (ev->type == EV_KEY && ev->value == 1) {
            uint16_t kc = ev->code;
            if (kc == 42 || kc == 54) { s_kb_shift = 1; goto recycle; }
            if (kc == 58) { s_kb_caps ^= 1; goto recycle; }
            if (kc == 57) s_vi_space = 1;

            if (kc < 128) {
                char base = kc_lower[kc];
                int use_shift = s_kb_shift;
                if (base >= 'a' && base <= 'z') use_shift = s_kb_shift ^ s_kb_caps;
                char c = use_shift ? kc_upper[kc] : kc_lower[kc];
                if (c) {
                    int nw = (s_key_w + 1) % KEY_BUF;
                    if (nw != s_key_r) {
                        s_key_buf[s_key_w] = c;
                        s_key_w = nw;
                    }
                }
            }
        } else if (ev->type == EV_KEY && ev->value == 0) {
            uint16_t kc = ev->code;
            if (kc == 42 || kc == 54) s_kb_shift = 0;
            if (kc == 57) s_vi_space = 0;
        }

recycle:
        s_dev[d].desc[slot].addr  = virt_to_phys(&s_dev[d].bufs[slot]);
        s_dev[d].desc[slot].len   = (uint32_t)sizeof(struct vi_event);
        s_dev[d].desc[slot].flags = VRING_DESC_F_WRITE;
        s_dev[d].avail->ring[s_dev[d].avail_idx % QUEUE_SIZE] = slot;
        s_dev[d].avail_idx++;
        s_dev[d].avail->idx = s_dev[d].avail_idx;
        notify_queue(d);
    }
}

int virtio_input_key_poll(void) {
    if (!s_has_keyboard) return -1;
    drain_keyboard();
    if (s_key_r == s_key_w) return -1;
    char c = s_key_buf[s_key_r];
    s_key_r = (s_key_r + 1) % KEY_BUF;
    return (int)(unsigned char)c;
}

void virtio_input_update_kb(void) {
    if (s_has_keyboard) drain_keyboard();
}

int virtio_input_space_down(void) {
    return s_vi_space;
}
