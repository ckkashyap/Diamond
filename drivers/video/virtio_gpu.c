/*
 * virtio_gpu.c - VirtIO GPU driver (legacy PCI interface, 2D only)
 *
 * PCI: vendor 0x1AF4, device 0x1050 (modern) or 0x1010 (legacy/transitional)
 * QEMU devices: -device virtio-vga, -device virtio-gpu-pci
 *
 * This driver implements the VirtIO GPU 2D protocol:
 *   1. Negotiate features (VIRGL disabled, 2D only)
 *   2. GET_DISPLAY_INFO to find the scanout dimensions
 *   3. RESOURCE_CREATE_2D to allocate a host-side resource
 *   4. RESOURCE_ATTACH_BACKING to map guest memory to the resource
 *   5. SET_SCANOUT to connect the resource to the display
 *   6. After any draw, TRANSFER_TO_HOST_2D + RESOURCE_FLUSH to update screen
 *
 * The Limine framebuffer is used as the backing store so drawing via
 * video.c primitives naturally feeds the VirtIO GPU resource.
 *
 * Control virtqueue uses the legacy virtio-over-PCI I/O BAR protocol.
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "../../arch/x86/spinlock.h"
#include "vid_drv.h"

/* ── VirtIO PCI legacy I/O offsets ──────────────────────────────────────── */
#define VIO_DEV_FEAT    0x00u   /* Device feature bits */
#define VIO_DRV_FEAT    0x04u   /* Driver feature ack */
#define VIO_QUEUE_ADDR  0x08u   /* Queue PFN */
#define VIO_QUEUE_SIZE  0x0Cu   /* Queue size (read) */
#define VIO_QUEUE_SEL   0x0Eu   /* Queue selector */
#define VIO_QUEUE_NTFY  0x10u   /* Queue notify */
#define VIO_DEV_STATUS  0x12u   /* Device status */
#define VIO_ISR         0x13u   /* ISR */

#define VIO_S_ACK       1u
#define VIO_S_DRIVER    2u
#define VIO_S_FEAT_OK   8u
#define VIO_S_DRIVER_OK 4u

/* ── VirtIO GPU command types ────────────────────────────────────────────── */
#define VIRTIO_GPU_CMD_GET_DISPLAY_INFO       0x0100u
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_2D     0x0101u
#define VIRTIO_GPU_CMD_RESOURCE_UNREF         0x0102u
#define VIRTIO_GPU_CMD_SET_SCANOUT            0x0103u
#define VIRTIO_GPU_CMD_RESOURCE_FLUSH         0x0104u
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D    0x0105u
#define VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING 0x0106u
#define VIRTIO_GPU_RESP_OK_NODATA             0x1100u
#define VIRTIO_GPU_RESP_OK_DISPLAY_INFO       0x1101u

#define VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM 2u
#define VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM 1u

/* ── GPU protocol structs ────────────────────────────────────────────────── */

struct __attribute__((packed)) vgpu_ctrl_hdr {
    uint32_t type;
    uint32_t flags;
    uint64_t fence_id;
    uint32_t ctx_id;
    uint32_t _pad;
};

struct __attribute__((packed)) vgpu_rect {
    uint32_t x, y, width, height;
};

struct __attribute__((packed)) vgpu_display_info {
    struct vgpu_ctrl_hdr hdr;
    struct {
        struct vgpu_rect r;
        uint32_t enabled;
        uint32_t flags;
    } pmodes[16];
};

struct __attribute__((packed)) vgpu_create_2d {
    struct vgpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t format;
    uint32_t width;
    uint32_t height;
};

struct __attribute__((packed)) vgpu_mem_entry {
    uint64_t addr;
    uint32_t length;
    uint32_t _pad;
};

struct __attribute__((packed)) vgpu_attach_backing {
    struct vgpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t nr_entries;
    struct vgpu_mem_entry entries[1];
};

struct __attribute__((packed)) vgpu_set_scanout {
    struct vgpu_ctrl_hdr hdr;
    struct vgpu_rect r;
    uint32_t scanout_id;
    uint32_t resource_id;
};

struct __attribute__((packed)) vgpu_transfer_to_host {
    struct vgpu_ctrl_hdr hdr;
    struct vgpu_rect r;
    uint64_t offset;
    uint32_t resource_id;
    uint32_t _pad;
};

struct __attribute__((packed)) vgpu_flush {
    struct vgpu_ctrl_hdr hdr;
    struct vgpu_rect r;
    uint32_t resource_id;
    uint32_t _pad;
};

/* ── VirtIO descriptor/avail/used (same as virtio_net.c) ────────────────── */
#define VRING_PAGE_BITS  12
#define VRING_PAGE_SIZE  (1u << VRING_PAGE_BITS)
#define QUEUE_SIZE       4u

struct __attribute__((packed)) vring_desc {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
};
#define VRING_DESC_F_NEXT  1u
#define VRING_DESC_F_WRITE 2u

struct __attribute__((packed)) vring_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[QUEUE_SIZE];
};
struct __attribute__((packed)) vring_used_elem { uint32_t id; uint32_t len; };
struct __attribute__((packed)) vring_used {
    uint16_t flags;
    uint16_t idx;
    struct vring_used_elem ring[QUEUE_SIZE];
};

/* ── Static queue buffers ────────────────────────────────────────────────── */
static uint8_t s_ctrlq_mem[VRING_PAGE_SIZE * 2] __attribute__((aligned(VRING_PAGE_SIZE)));

/* Command + response buffer.
 * s_rsp must hold a vgpu_display_info: 24-byte hdr + 16*24 = 408 bytes → 512. */
static uint8_t s_cmd[256] __attribute__((aligned(64)));
static uint8_t s_rsp[512] __attribute__((aligned(64)));

/* ── Driver state ────────────────────────────────────────────────────────── */
static uint16_t s_iobase  = 0;
static int      s_ready   = 0;
static uint32_t s_res_w   = 0;
static uint32_t s_res_h   = 0;
static uint64_t s_kphys   = 0;
static uint64_t s_kvirt   = 0;
static void    *s_fb_virt = (void *)0;

static struct vring_desc  *s_desc;
static struct vring_avail *s_avail;
static struct vring_used  *s_used;
static uint16_t s_avail_idx = 0;
static uint16_t s_used_idx  = 0;

static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

/* ── Virtqueue helpers ───────────────────────────────────────────────────── */

static void vq_setup(void) {
    s_desc  = (struct vring_desc *)s_ctrlq_mem;
    /* Avail ring immediately after descriptors */
    s_avail = (struct vring_avail *)(s_ctrlq_mem + QUEUE_SIZE * 16u);
    /* Used ring at next page boundary */
    uint64_t used_off = (uint64_t)(uintptr_t)s_avail + 6u + 2u * QUEUE_SIZE;
    used_off = (used_off + VRING_PAGE_SIZE - 1u) & ~(uint64_t)(VRING_PAGE_SIZE - 1u);
    s_used = (struct vring_used *)used_off;
}

/* Send a two-descriptor command (request + response), wait for completion. */
static void vq_send(void *req, uint32_t req_len, void *rsp, uint32_t rsp_len) {
    /* Descriptor 0: request (device-readable) */
    s_desc[0].addr  = virt_to_phys(req);
    s_desc[0].len   = req_len;
    s_desc[0].flags = VRING_DESC_F_NEXT;
    s_desc[0].next  = 1;
    /* Descriptor 1: response (device-writable) */
    s_desc[1].addr  = virt_to_phys(rsp);
    s_desc[1].len   = rsp_len;
    s_desc[1].flags = VRING_DESC_F_WRITE;
    s_desc[1].next  = 0;

    s_avail->ring[s_avail_idx % QUEUE_SIZE] = 0;
    s_avail_idx++;
    s_avail->idx = s_avail_idx;

    /* Notify the device (queue 0 = controlq) */
    outw(s_iobase + VIO_QUEUE_NTFY, 0);

    /* Poll for completion */
    while (s_used->idx == s_used_idx)
        __asm__ volatile ("pause");
    s_used_idx = s_used->idx;
}

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int virtio_gpu_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt,
                            void **fb, uint32_t *w, uint32_t *h, uint32_t *pitch,
                            uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    (void)hhdm_offset;
    (void)r_shift; (void)g_shift; (void)b_shift;

    s_kphys   = kphys;
    s_kvirt   = kvirt;
    s_fb_virt = *fb;

    uint8_t bus, dev, fn;
    /* Try modern (0x1050) then transitional (0x1010) */
    if (!pci_find(0x1AF4, 0x1050, &bus, &dev, &fn) &&
        !pci_find(0x1AF4, 0x1010, &bus, &dev, &fn)) return 0;

    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0005);

    uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
    s_iobase = (uint16_t)(bar0 & ~0x3u);

    /* VirtIO reset + negotiate */
    outb(s_iobase + VIO_DEV_STATUS, 0);
    outb(s_iobase + VIO_DEV_STATUS, VIO_S_ACK | VIO_S_DRIVER);
    /* Accept no extra features for 2D-only operation */
    uint32_t dev_feat = inl(s_iobase + VIO_DEV_FEAT);
    outl(s_iobase + VIO_DRV_FEAT, dev_feat & 0u);   /* no features */
    outb(s_iobase + VIO_DEV_STATUS,
         VIO_S_ACK | VIO_S_DRIVER | VIO_S_FEAT_OK);

    /* Set up controlq (queue 0) */
    outw(s_iobase + VIO_QUEUE_SEL, 0);
    vq_setup();
    uint64_t qphys = virt_to_phys(s_ctrlq_mem);
    outl(s_iobase + VIO_QUEUE_ADDR, (uint32_t)(qphys >> VRING_PAGE_BITS));

    outb(s_iobase + VIO_DEV_STATUS,
         VIO_S_ACK | VIO_S_DRIVER | VIO_S_FEAT_OK | VIO_S_DRIVER_OK);

    /* ── GET_DISPLAY_INFO ────────────────────────────────────────────────── */
    struct vgpu_ctrl_hdr *hdr = (struct vgpu_ctrl_hdr *)s_cmd;
    hdr->type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;
    hdr->flags = 0; hdr->fence_id = 0; hdr->ctx_id = 0; hdr->_pad = 0;
    vq_send(s_cmd, (uint32_t)sizeof(*hdr), s_rsp, (uint32_t)sizeof(s_rsp));

    struct vgpu_display_info *di = (struct vgpu_display_info *)s_rsp;
    if (di->hdr.type == VIRTIO_GPU_RESP_OK_DISPLAY_INFO &&
        di->pmodes[0].enabled && di->pmodes[0].r.width > 0) {
        *w = di->pmodes[0].r.width;
        *h = di->pmodes[0].r.height;
        *pitch = (*w) * 4u;
        s_res_w = *w;
        s_res_h = *h;
    } else {
        s_res_w = *w;
        s_res_h = *h;
    }

    /* ── RESOURCE_CREATE_2D ──────────────────────────────────────────────── */
    struct vgpu_create_2d *c2d = (struct vgpu_create_2d *)s_cmd;
    c2d->hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    c2d->hdr.flags = 0; c2d->hdr.fence_id = 0;
    c2d->hdr.ctx_id = 0; c2d->hdr._pad = 0;
    c2d->resource_id = 1;
    c2d->format = VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM;
    c2d->width  = s_res_w;
    c2d->height = s_res_h;
    vq_send(s_cmd, (uint32_t)sizeof(*c2d), s_rsp, (uint32_t)sizeof(s_rsp));

    /* ── RESOURCE_ATTACH_BACKING ─────────────────────────────────────────── */
    struct vgpu_attach_backing *ab = (struct vgpu_attach_backing *)s_cmd;
    ab->hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
    ab->hdr.flags = 0; ab->hdr.fence_id = 0;
    ab->hdr.ctx_id = 0; ab->hdr._pad = 0;
    ab->resource_id = 1;
    ab->nr_entries  = 1;
    ab->entries[0].addr   = virt_to_phys(*fb);
    ab->entries[0].length = s_res_w * s_res_h * 4u;
    ab->entries[0]._pad   = 0;
    vq_send(s_cmd, (uint32_t)sizeof(*ab), s_rsp, (uint32_t)sizeof(s_rsp));

    /* ── SET_SCANOUT ─────────────────────────────────────────────────────── */
    struct vgpu_set_scanout *ss = (struct vgpu_set_scanout *)s_cmd;
    ss->hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    ss->hdr.flags = 0; ss->hdr.fence_id = 0;
    ss->hdr.ctx_id = 0; ss->hdr._pad = 0;
    ss->r.x = 0; ss->r.y = 0;
    ss->r.width = s_res_w; ss->r.height = s_res_h;
    ss->scanout_id  = 0;
    ss->resource_id = 1;
    vq_send(s_cmd, (uint32_t)sizeof(*ss), s_rsp, (uint32_t)sizeof(s_rsp));

    s_ready = 1;
    return 1;
}

static int virtio_gpu_set_mode(uint32_t w, uint32_t h, uint32_t bpp) {
    (void)w; (void)h; (void)bpp;
    return -1;   /* mode changing not implemented */
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct video_driver virtio_gpu_driver = {
    .name     = "virtio-gpu",
    .probe    = virtio_gpu_probe,
    .set_mode = virtio_gpu_set_mode,
};
