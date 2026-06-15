/*
 * bga.c - Bochs/QEMU BGA (VBE) display adapter driver
 *
 * PCI vendor/device: 0x1234 / 0x1111  (used by QEMU bochs-display and VGA)
 * The BGA exposes a Bochs VBE register interface at I/O ports 0x1CE (index)
 * and 0x1CF (data).  BAR0 holds the linear framebuffer physical address.
 *
 * QEMU devices that expose this interface:
 *   -device VGA          (also has BGA registers)
 *   -device bochs-display
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "vid_drv.h"

/* ── VBE register interface ──────────────────────────────────────────────── */
#define VBE_INDEX  0x01CEu
#define VBE_DATA   0x01CFu

/* VBE register indices */
#define VBE_REG_ID          0u
#define VBE_REG_XRES        1u
#define VBE_REG_YRES        2u
#define VBE_REG_BPP         3u
#define VBE_REG_ENABLE      4u
#define VBE_REG_BANK        5u
#define VBE_REG_VIRT_WIDTH  6u
#define VBE_REG_VIRT_HEIGHT 7u
#define VBE_REG_X_OFFSET    8u
#define VBE_REG_Y_OFFSET    9u

/* ENABLE register bits */
#define VBE_ENABLED         0x01u
#define VBE_LFB_ENABLED     0x40u
#define VBE_NOCLEARMEM      0x80u

/* Minimum expected VBE ID */
#define VBE_ID_MIN          0xB0C0u

static uint16_t bga_read(uint16_t reg) {
    outw(VBE_INDEX, reg);
    return inw(VBE_DATA);
}

static void bga_write(uint16_t reg, uint16_t val) {
    outw(VBE_INDEX, reg);
    outw(VBE_DATA, val);
}

/* ── Driver state ────────────────────────────────────────────────────────── */
static int s_present = 0;
static int bga_set_mode(uint32_t w, uint32_t h, uint32_t bpp);

/* ── Probe ───────────────────────────────────────────────────────────────── */

static int bga_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                     void **fb, uint32_t *w, uint32_t *h, uint32_t *pitch,
                     uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    (void)kphys; (void)kvirt;
    (void)r_shift; (void)g_shift; (void)b_shift;

    /* The BGA VBE registers are present when bochs-display or VGA is used.
     * Verify by reading the VBE ID register. */
    uint16_t id = bga_read(VBE_REG_ID);
    if (id < VBE_ID_MIN) return 0;

    /* Optionally: check for the PCI device too (vendor 0x1234) */
    uint8_t bus, dev, fn;
    if (!pci_find(0x1234, 0x1111, &bus, &dev, &fn)) return 0;

    /* BAR0 holds the LFB physical address; map it via HHDM */
    uint32_t bar0    = pci_read32(bus, dev, fn, 0x10);
    uint64_t fb_phys = bar0 & ~0xfU;
    if (fb_phys)
        *fb = (void *)(fb_phys + hhdm_offset);

    s_present = 1;  /* must be set before bga_set_mode checks it */

    /* Actively set the desired mode — don't trust whatever Limine left. */
    bga_set_mode(2560, 1440, 32);

    /* Read back what the hardware actually accepted. */
    uint16_t act_w = bga_read(VBE_REG_XRES);
    uint16_t act_h = bga_read(VBE_REG_YRES);
    *w     = act_w;
    *h     = act_h;
    *pitch = (uint32_t)act_w * 4u;

    return 1;
}

/* ── Mode set ────────────────────────────────────────────────────────────── */

static int bga_set_mode(uint32_t w, uint32_t h, uint32_t bpp) {
    if (!s_present) return -1;
    bga_write(VBE_REG_ENABLE,      0);                     /* disable */
    bga_write(VBE_REG_XRES,        (uint16_t)w);
    bga_write(VBE_REG_YRES,        (uint16_t)h);
    bga_write(VBE_REG_BPP,         (uint16_t)bpp);
    bga_write(VBE_REG_VIRT_WIDTH,  (uint16_t)w);
    bga_write(VBE_REG_VIRT_HEIGHT, (uint16_t)h);
    bga_write(VBE_REG_X_OFFSET,    0);
    bga_write(VBE_REG_Y_OFFSET,    0);
    bga_write(VBE_REG_ENABLE,      VBE_ENABLED | VBE_LFB_ENABLED);
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct video_driver bga_driver = {
    .name     = "bga",
    .probe    = bga_probe,
    .set_mode = bga_set_mode,
};
