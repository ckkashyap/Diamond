/*
 * pcnet.c - AMD PCnet-PCI II (Am79C970A) NIC driver
 *
 * PCI vendor/device: 0x1022 / 0x2000
 * Uses I/O BAR0.  Switched to 32-bit mode (SSIZE32) at init.
 * Configured via an Initialization Block in BSS.
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "nic.h"

/* ── I/O port offsets (32-bit DWIO mode) ────────────────────────────────── */
#define PCNET_RDP   0x10u   /* Register Data Port (32-bit) */
#define PCNET_RAP   0x14u   /* Register Address Port (32-bit) */
#define PCNET_RST   0x18u   /* Software Reset (read triggers reset) */
#define PCNET_BDP   0x1Cu   /* Bus Configuration Register Data Port */

/* CSR indices */
#define CSR0        0u    /* Controller Status */
#define CSR1        1u    /* IADR[15:0]  (init block low) */
#define CSR2        2u    /* IADR[31:16] (init block high) */
#define CSR4        4u    /* Test and Features */
#define CSR15       15u   /* Mode register */
#define CSR88       88u   /* Chip ID lower */
#define CSR89       89u   /* Chip ID upper */

/* BCR indices */
#define BCR20       20u   /* Software Style (SSIZE32) */

/* CSR0 bits */
#define CSR0_INIT   (1u << 0)
#define CSR0_STRT   (1u << 1)
#define CSR0_STOP   (1u << 2)
#define CSR0_IDON   (1u << 8)

/* ── Initialization Block (32-bit software style) ────────────────────────── */
struct __attribute__((packed)) pcnet_init_block {
    uint16_t mode;      /* 0x0000 = normal */
    uint8_t  rlen;      /* log2(RX ring size) in bits [7:4] */
    uint8_t  tlen;      /* log2(TX ring size) in bits [7:4] */
    uint8_t  mac[6];
    uint16_t _reserved;
    uint64_t ladr;      /* Logical Address Filter (multicast) */
    uint32_t rdra;      /* RX descriptor ring physical address */
    uint32_t tdra;      /* TX descriptor ring physical address */
};

/* ── Descriptor struct (32-bit mode, 16 bytes each) ─────────────────────── */
struct __attribute__((packed)) pcnet_desc {
    uint32_t addr;      /* buffer physical address */
    int16_t  bcnt;      /* buffer byte count (2's complement, negative) */
    uint16_t mcnt;      /* message byte count (RX: filled by hardware) */
    uint8_t  flags;     /* OWN = bit 7 */
    uint8_t  flags2;
    uint16_t flags3;
};

/* OWN bit: 1 = hardware owns, 0 = software owns */
#define OWN     (1u << 7)
/* STP (start of packet) + ENP (end of packet) for single-buffer TX */
#define STP     (1u << 9)
#define ENP     (1u << 8)

/* ── Ring sizes ──────────────────────────────────────────────────────────── */
#define TX_COUNT  4    /* must be power of 2, ≤ 512 */
#define RX_COUNT  4
#define BUF_SIZE  1536

/* RLEN/TLEN field: log2 of ring size in upper nibble */
#define RLEN_VAL  ((2u) << 4)   /* 2 = log2(4) */
#define TLEN_VAL  ((2u) << 4)

static struct pcnet_desc    s_rx_ring[RX_COUNT] __attribute__((aligned(16)));
static struct pcnet_desc    s_tx_ring[TX_COUNT] __attribute__((aligned(16)));
static uint8_t s_rx_bufs[RX_COUNT][BUF_SIZE]   __attribute__((aligned(16)));
static uint8_t s_tx_bufs[TX_COUNT][BUF_SIZE]   __attribute__((aligned(16)));
static struct pcnet_init_block s_init __attribute__((aligned(4)));

/* ── Driver state ────────────────────────────────────────────────────────── */
static uint16_t s_iobase  = 0;
static uint8_t  s_mac[6];
static int      s_ready   = 0;
static int      s_rx_idx  = 0;
static int      s_tx_idx  = 0;
static uint64_t s_kphys   = 0;
static uint64_t s_kvirt   = 0;

static uint32_t virt_to_phys32(const void *v) {
    return (uint32_t)((uint64_t)(uintptr_t)v - s_kvirt + s_kphys);
}

/* CSR access (RAP selects register, then RDP reads/writes it) */
static uint32_t csr_r(uint32_t idx) {
    outl(s_iobase + PCNET_RAP, idx);
    return inl(s_iobase + PCNET_RDP);
}
static void csr_w(uint32_t idx, uint32_t v) {
    outl(s_iobase + PCNET_RAP, idx);
    outl(s_iobase + PCNET_RDP, v);
}
static void bcr_w(uint32_t idx, uint32_t v) {
    outl(s_iobase + PCNET_RAP, idx);
    outl(s_iobase + PCNET_BDP, v);
}

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int pcnet_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset;
    s_kphys = kphys;
    s_kvirt = kvirt;

    uint8_t bus, dev, fn;
    if (!pci_find(0x1022, 0x2000, &bus, &dev, &fn)) return 0;

    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0005);

    uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
    s_iobase = (uint16_t)(bar0 & ~0x3u);

    /* Software reset: 32-bit read of RST port triggers reset */
    (void)inl(s_iobase + PCNET_RST);

    /* Short delay after reset */
    for (volatile int i = 0; i < 10000; i++) __asm__ volatile ("nop");

    /* Switch to 32-bit (DWIO) mode: write anything to RDP in 32-bit width */
    outl(s_iobase + PCNET_RDP, 0);

    /* BCR20 bit 8 (SSIZE32) selects 32-bit software style */
    bcr_w(BCR20, 0x0102u);   /* style 2 = PCnet-PCI II 32-bit */

    /* Read MAC from the first 6 bytes of the I/O PROM (byte-accessible) */
    for (int i = 0; i < 6; i++)
        s_mac[i] = inb(s_iobase + (uint8_t)i);
    for (int i = 0; i < 6; i++) s_init.mac[i] = s_mac[i];

    /* Build RX descriptors */
    for (int i = 0; i < RX_COUNT; i++) {
        s_rx_ring[i].addr  = virt_to_phys32(s_rx_bufs[i]);
        s_rx_ring[i].bcnt  = (int16_t)(-(int16_t)BUF_SIZE) | (int16_t)0xF000;
        s_rx_ring[i].mcnt  = 0;
        s_rx_ring[i].flags = OWN;   /* hardware owns all initially */
        s_rx_ring[i].flags2 = 0;
        s_rx_ring[i].flags3 = 0;
    }

    /* Build TX descriptors (software owns all) */
    for (int i = 0; i < TX_COUNT; i++) {
        s_tx_ring[i].addr  = 0;
        s_tx_ring[i].bcnt  = (int16_t)0xF000;
        s_tx_ring[i].mcnt  = 0;
        s_tx_ring[i].flags = 0;
        s_tx_ring[i].flags2 = 0;
        s_tx_ring[i].flags3 = 0;
    }

    /* Fill initialization block */
    s_init.mode    = 0x0000;
    s_init.rlen    = RLEN_VAL;
    s_init.tlen    = TLEN_VAL;
    s_init._reserved = 0;
    s_init.ladr    = 0xffffffffffffffffULL;   /* accept all multicast */
    s_init.rdra    = virt_to_phys32(s_rx_ring);
    s_init.tdra    = virt_to_phys32(s_tx_ring);

    /* Point CSR1/2 at the init block */
    uint32_t init_phys = virt_to_phys32(&s_init);
    csr_w(CSR1, init_phys & 0xffffu);
    csr_w(CSR2, init_phys >> 16);

    /* Stop → Init → wait for IDON → Start */
    csr_w(CSR0, CSR0_STOP);
    csr_w(CSR0, CSR0_INIT);
    for (int i = 0; i < 200000; i++) {
        if (csr_r(CSR0) & CSR0_IDON) break;
        __asm__ volatile ("pause");
    }
    if (!(csr_r(CSR0) & CSR0_IDON)) return 0;   /* init timeout */

    csr_w(CSR0, CSR0_STRT | CSR0_IDON);   /* clear IDON, start */

    s_rx_idx = 0;
    s_tx_idx = 0;
    s_ready  = 1;
    return 1;
}

static void pcnet_mac(uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) mac[i] = s_mac[i];
}

static int pcnet_link_up(void) {
    if (!s_ready) return 0;
    /* BCR4 LEDOUT bit indicates link — not always reliable; return 1 */
    return 1;
}

static int pcnet_send(const void *data, uint16_t len) {
    if (!s_ready || len > BUF_SIZE) return -1;

    struct pcnet_desc *d = &s_tx_ring[s_tx_idx];

    /* Wait until software owns this descriptor */
    while (d->flags & OWN) __asm__ volatile ("pause");

    const uint8_t *src = (const uint8_t *)data;
    uint8_t       *dst = s_tx_bufs[s_tx_idx];
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    d->addr  = virt_to_phys32(dst);
    d->bcnt  = (int16_t)(-(int16_t)len) | (int16_t)0xF000;
    d->mcnt  = 0;
    d->flags2 = 0;
    d->flags3 = 0;
    /* STP + ENP = single-buffer packet; set OWN last */
    d->flags = (uint8_t)(OWN | STP | ENP);

    /* Kick the transmitter */
    csr_w(CSR0, csr_r(CSR0) | (1u << 3));   /* TDMD: transmit demand */

    s_tx_idx = (s_tx_idx + 1) % TX_COUNT;
    return 0;
}

static int pcnet_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_ready) return -1;

    struct pcnet_desc *d = &s_rx_ring[s_rx_idx];
    if (d->flags & OWN) return -1;   /* still owned by hardware */

    uint16_t len = (uint16_t)(d->mcnt & 0x0fffu);
    /* Strip 4-byte FCS */
    if (len >= 4) len -= 4;
    if (len > maxlen) len = maxlen;
    *len_out = len;

    const uint8_t *src = s_rx_bufs[s_rx_idx];
    uint8_t       *dst = (uint8_t *)buf;
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    /* Return descriptor to hardware */
    d->bcnt  = (int16_t)(-(int16_t)BUF_SIZE) | (int16_t)0xF000;
    d->mcnt  = 0;
    d->flags2 = 0;
    d->flags3 = 0;
    d->flags = OWN;

    s_rx_idx = (s_rx_idx + 1) % RX_COUNT;
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct nic_driver pcnet_driver = {
    .name    = "pcnet",
    .probe   = pcnet_probe,
    .mac     = pcnet_mac,
    .link_up = pcnet_link_up,
    .send    = pcnet_send,
    .recv    = pcnet_recv,
};
