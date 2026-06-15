/*
 * virtio_net.c - VirtIO network driver (legacy PCI interface)
 *
 * PCI vendor/device: 0x1AF4 / 0x1000  (legacy transitional device)
 *
 * Uses the legacy VirtIO PCI interface (VIRTIO_PCI_CAP not required).
 * BAR0 is an I/O BAR giving access to the VirtIO header registers.
 *
 * Virtqueues:
 *   Queue 0: RX (device → driver)
 *   Queue 1: TX (driver → device)
 *
 * Each virtqueue has three regions packed into one contiguous BSS block:
 *   - Descriptor table: 16 bytes × queue_size
 *   - Available ring:   6 + 2 × queue_size bytes
 *   - Used ring:        6 + 8 × queue_size bytes
 * All three are placed in one page-aligned buffer per queue.
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "nic.h"

/* ── VirtIO PCI legacy I/O register offsets ──────────────────────────────── */
#define VIRTIO_DEV_FEATURES   0x00u   /* 32-bit device feature bits */
#define VIRTIO_DRV_FEATURES   0x04u   /* 32-bit driver feature ack */
#define VIRTIO_QUEUE_ADDR     0x08u   /* queue pfn (page frame number) */
#define VIRTIO_QUEUE_SIZE     0x0Cu   /* queue size (read-only) */
#define VIRTIO_QUEUE_SELECT   0x0Eu   /* queue selector */
#define VIRTIO_QUEUE_NOTIFY   0x10u   /* queue kick */
#define VIRTIO_DEV_STATUS     0x12u   /* device status byte */
#define VIRTIO_ISR            0x13u   /* ISR status */
/* virtio-net config starts at offset 0x14 (after common header) */
#define VIRTIO_NET_MAC        0x14u   /* 6 bytes */
#define VIRTIO_NET_STATUS     0x1Au   /* 16-bit link status */

/* Device status bits */
#define VIRTIO_S_ACK          1u
#define VIRTIO_S_DRIVER       2u
#define VIRTIO_S_DRIVER_OK    4u
#define VIRTIO_S_FEAT_OK      8u

/* Feature bits relevant to virtio-net */
#define VIRTIO_NET_F_MAC      (1u << 5)

/* Virtqueue page size (legacy: queue aligned to 4 KiB) */
#define VRING_PAGE_BITS  12
#define VRING_PAGE_SIZE  (1u << VRING_PAGE_BITS)

/* ── Virtqueue descriptor (16 bytes) ─────────────────────────────────────── */
struct __attribute__((packed)) vring_desc {
    uint64_t addr;    /* buffer physical address */
    uint32_t len;     /* buffer length */
    uint16_t flags;   /* VRING_DESC_F_* */
    uint16_t next;    /* next descriptor index (if NEXT flag set) */
};
#define VRING_DESC_F_NEXT   1u
#define VRING_DESC_F_WRITE  2u   /* device-writable (RX buffers) */

/* ── Available ring ──────────────────────────────────────────────────────── */
struct __attribute__((packed)) vring_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[];   /* indices into descriptor table */
};

/* ── Used ring ───────────────────────────────────────────────────────────── */
struct __attribute__((packed)) vring_used_elem {
    uint32_t id;
    uint32_t len;
};
struct __attribute__((packed)) vring_used {
    uint16_t flags;
    uint16_t idx;
    struct vring_used_elem ring[];
};

/* ── Queue parameters ────────────────────────────────────────────────────── */
#define QUEUE_SIZE   16u   /* must be power of 2 */

/* virtio-net prepends a 10-byte header to every packet */
struct __attribute__((packed)) virtio_net_hdr {
    uint8_t  flags;
    uint8_t  gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
};

/* ── Per-queue layout ────────────────────────────────────────────────────── */
/*
 * We place descriptor table + available ring + used ring in one contiguous
 * buffer.  The used ring must be page-aligned per the legacy spec, so we
 * over-allocate and align internally.
 *
 * Sizes for QUEUE_SIZE = 16:
 *   Descriptor table: 16 × 16 = 256 bytes
 *   Available ring:   6 + 2×16 = 38 bytes  → pad to 4K boundary
 *   Used ring:        6 + 8×16 = 134 bytes
 *
 * Total per queue < 8 KiB; we allocate 8 KiB aligned to 4 KiB.
 */
#define QUEUE_BUF_BYTES  (VRING_PAGE_SIZE * 2u)

/* RX queue (queue 0): device writes received packets here */
static uint8_t s_rxq_mem[QUEUE_BUF_BYTES] __attribute__((aligned(VRING_PAGE_SIZE)));
/* TX queue (queue 1): driver writes packets to send here */
static uint8_t s_txq_mem[QUEUE_BUF_BYTES] __attribute__((aligned(VRING_PAGE_SIZE)));

/* Packet payload buffers (each prefixed with virtio_net_hdr) */
#define PKT_BUF_SIZE  2048u
static uint8_t s_rx_bufs[QUEUE_SIZE][sizeof(struct virtio_net_hdr) + PKT_BUF_SIZE]
                __attribute__((aligned(4)));
static uint8_t s_tx_bufs[QUEUE_SIZE][sizeof(struct virtio_net_hdr) + PKT_BUF_SIZE]
                __attribute__((aligned(4)));

/* ── Queue accessors ─────────────────────────────────────────────────────── */
static struct vring_desc  *rxq_desc;
static struct vring_avail *rxq_avail;
static struct vring_used  *rxq_used;
static struct vring_desc  *txq_desc;
static struct vring_avail *txq_avail;
static struct vring_used  *txq_used;

/* Last seen used-ring index for each queue */
static uint16_t s_rxq_last_used = 0;
static uint16_t s_txq_last_used = 0;
/* Next available slot in the descriptor/avail ring */
static uint16_t s_rxq_avail_idx = 0;
static uint16_t s_txq_avail_idx = 0;

/* ── Driver state ────────────────────────────────────────────────────────── */
static uint16_t s_iobase = 0;
static uint8_t  s_mac[6];
static int      s_ready  = 0;
static uint64_t s_kphys  = 0;
static uint64_t s_kvirt  = 0;

static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

/* ── Queue setup helper ──────────────────────────────────────────────────── */

/*
 * Point the desc/avail/used pointers into the queue buffer.
 * The legacy spec requires: used ring starts at the first 4 KiB page
 * boundary after the descriptor table and available ring.
 */
static void setup_queue_pointers(uint8_t *mem,
                                 struct vring_desc  **desc_out,
                                 struct vring_avail **avail_out,
                                 struct vring_used  **used_out) {
    /* Descriptor table at start of buffer */
    *desc_out  = (struct vring_desc *)mem;
    /* Available ring immediately after */
    *avail_out = (struct vring_avail *)(mem + QUEUE_SIZE * sizeof(struct vring_desc));
    /* Used ring: aligned to next 4 KiB boundary */
    uint64_t avail_end = (uint64_t)(uintptr_t)(*avail_out)
                         + 6u + 2u * QUEUE_SIZE;
    uint64_t used_start = (avail_end + VRING_PAGE_SIZE - 1u)
                          & ~(uint64_t)(VRING_PAGE_SIZE - 1u);
    *used_out  = (struct vring_used *)used_start;
}

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int virtio_net_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset;
    s_kphys = kphys;
    s_kvirt = kvirt;

    uint8_t bus, dev, fn;
    if (!pci_find(0x1AF4, 0x1000, &bus, &dev, &fn)) return 0;

    /* Enable bus-mastering + I/O space */
    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0005);

    /* BAR0 is I/O */
    uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
    s_iobase = (uint16_t)(bar0 & ~0x3u);

    /* Reset device */
    outb(s_iobase + VIRTIO_DEV_STATUS, 0);

    /* Set ACKNOWLEDGE + DRIVER */
    outb(s_iobase + VIRTIO_DEV_STATUS, VIRTIO_S_ACK | VIRTIO_S_DRIVER);

    /* Negotiate features: accept only VIRTIO_NET_F_MAC */
    uint32_t dev_feat = inl(s_iobase + VIRTIO_DEV_FEATURES);
    uint32_t drv_feat = dev_feat & VIRTIO_NET_F_MAC;
    outl(s_iobase + VIRTIO_DRV_FEATURES, drv_feat);

    /* FEATURES_OK */
    outb(s_iobase + VIRTIO_DEV_STATUS,
         VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK);

    /* ── Set up RX queue (index 0) ───────────────────────────────────────── */
    outw(s_iobase + VIRTIO_QUEUE_SELECT, 0);
    uint16_t qsz = inw(s_iobase + VIRTIO_QUEUE_SIZE);
    if (qsz == 0 || qsz > QUEUE_SIZE) qsz = QUEUE_SIZE;

    setup_queue_pointers(s_rxq_mem, &rxq_desc, &rxq_avail, &rxq_used);

    /* Pre-fill all RX descriptors with device-writable buffers */
    for (uint16_t i = 0; i < qsz; i++) {
        rxq_desc[i].addr  = virt_to_phys(s_rx_bufs[i]);
        rxq_desc[i].len   = (uint32_t)sizeof(s_rx_bufs[i]);
        rxq_desc[i].flags = VRING_DESC_F_WRITE;
        rxq_desc[i].next  = 0;
        rxq_avail->ring[i] = i;
    }
    rxq_avail->idx = qsz;
    s_rxq_avail_idx = qsz;
    s_rxq_last_used = 0;

    /* Write queue PFN (physical page frame number of the descriptor table) */
    uint64_t rxq_phys = virt_to_phys(s_rxq_mem);
    outl(s_iobase + VIRTIO_QUEUE_ADDR,
         (uint32_t)(rxq_phys >> VRING_PAGE_BITS));

    /* ── Set up TX queue (index 1) ───────────────────────────────────────── */
    outw(s_iobase + VIRTIO_QUEUE_SELECT, 1);

    setup_queue_pointers(s_txq_mem, &txq_desc, &txq_avail, &txq_used);

    /* TX descriptors start empty (software owns all) */
    txq_avail->idx = 0;
    s_txq_avail_idx = 0;
    s_txq_last_used = 0;

    uint64_t txq_phys = virt_to_phys(s_txq_mem);
    outl(s_iobase + VIRTIO_QUEUE_ADDR,
         (uint32_t)(txq_phys >> VRING_PAGE_BITS));

    /* DRIVER_OK */
    outb(s_iobase + VIRTIO_DEV_STATUS,
         VIRTIO_S_ACK | VIRTIO_S_DRIVER | VIRTIO_S_FEAT_OK | VIRTIO_S_DRIVER_OK);

    /* Read MAC from device config space */
    for (int i = 0; i < 6; i++)
        s_mac[i] = inb(s_iobase + VIRTIO_NET_MAC + (uint8_t)i);

    s_ready = 1;
    return 1;
}

static void virtio_net_mac(uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) mac[i] = s_mac[i];
}

static int virtio_net_link_up(void) {
    if (!s_ready) return 0;
    uint16_t st = inw(s_iobase + VIRTIO_NET_STATUS);
    return (st & 1u) ? 1 : 0;   /* bit 0 = LINK_UP */
}

static int virtio_net_send(const void *data, uint16_t len) {
    if (!s_ready || len > PKT_BUF_SIZE) return -1;

    /* Find a slot the device has returned to us via the used ring */
    if (s_txq_last_used == txq_used->idx) return -1;   /* no free slot */
    uint16_t slot = (uint16_t)(txq_used->ring[s_txq_last_used % QUEUE_SIZE].id
                                % QUEUE_SIZE);
    s_txq_last_used++;

    /* Build virtio-net header + packet in the TX buffer */
    uint8_t *buf = s_tx_bufs[slot];
    struct virtio_net_hdr *hdr = (struct virtio_net_hdr *)buf;
    hdr->flags      = 0;
    hdr->gso_type   = 0;
    hdr->hdr_len    = 0;
    hdr->gso_size   = 0;
    hdr->csum_start = 0;
    hdr->csum_offset = 0;

    const uint8_t *src = (const uint8_t *)data;
    uint8_t       *dst = buf + sizeof(struct virtio_net_hdr);
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    /* Fill descriptor */
    uint16_t di = slot % QUEUE_SIZE;
    txq_desc[di].addr  = virt_to_phys(buf);
    txq_desc[di].len   = (uint32_t)(sizeof(struct virtio_net_hdr) + len);
    txq_desc[di].flags = 0;
    txq_desc[di].next  = 0;

    /* Put descriptor in the available ring */
    txq_avail->ring[s_txq_avail_idx % QUEUE_SIZE] = di;
    s_txq_avail_idx++;
    txq_avail->idx = s_txq_avail_idx;

    /* Notify the device (queue 1 = TX) */
    outw(s_iobase + VIRTIO_QUEUE_NOTIFY, 1);
    return 0;
}

static int virtio_net_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_ready) return -1;

    /* Check if device has placed a packet in the used ring */
    if (s_rxq_last_used == rxq_used->idx) return -1;

    uint32_t elem_len = rxq_used->ring[s_rxq_last_used % QUEUE_SIZE].len;
    uint16_t slot     = (uint16_t)(rxq_used->ring[s_rxq_last_used % QUEUE_SIZE].id
                                   % QUEUE_SIZE);
    s_rxq_last_used++;

    /* Strip virtio-net header */
    uint32_t hdr_size = sizeof(struct virtio_net_hdr);
    uint16_t pkt_len  = (elem_len > hdr_size)
                        ? (uint16_t)(elem_len - hdr_size) : 0;
    if (pkt_len > maxlen) pkt_len = maxlen;
    *len_out = pkt_len;

    const uint8_t *src = s_rx_bufs[slot] + hdr_size;
    uint8_t       *dst = (uint8_t *)buf;
    for (uint16_t i = 0; i < pkt_len; i++) dst[i] = src[i];

    /* Return descriptor to the available ring */
    rxq_desc[slot].addr  = virt_to_phys(s_rx_bufs[slot]);
    rxq_desc[slot].len   = (uint32_t)sizeof(s_rx_bufs[slot]);
    rxq_desc[slot].flags = VRING_DESC_F_WRITE;
    rxq_avail->ring[s_rxq_avail_idx % QUEUE_SIZE] = slot;
    s_rxq_avail_idx++;
    rxq_avail->idx = s_rxq_avail_idx;

    /* Notify device (queue 0 = RX) */
    outw(s_iobase + VIRTIO_QUEUE_NOTIFY, 0);
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct nic_driver virtio_net_driver = {
    .name    = "virtio-net",
    .probe   = virtio_net_probe,
    .mac     = virtio_net_mac,
    .link_up = virtio_net_link_up,
    .send    = virtio_net_send,
    .recv    = virtio_net_recv,
};
