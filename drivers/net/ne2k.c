/*
 * ne2k.c - NE2000 PCI NIC driver (DP8390-compatible)
 *
 * PCI vendor/device: 0x10EC / 0x8029  (Realtek NE2000 clone)
 *                    0x1050 / 0x0940  (Winbond W89C940)
 * Uses I/O BAR0.  Page-register architecture (CR selects bank 0/1/2).
 * Internal 16 KiB RAM: pages 0x00–0x3F (TX) and 0x40–0xFF (RX ring).
 * Remote DMA copies packets between host memory and the internal RAM.
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "nic.h"

/* ── Register offsets (from I/O base) ───────────────────────────────────── */
/* Page 0 (read) */
#define NE_CR    0x00u   /* Command Register */
#define NE_CLDA0 0x01u   /* Current Local DMA Address 0 */
#define NE_CLDA1 0x02u   /* Current Local DMA Address 1 */
#define NE_BNRY  0x03u   /* Boundary Pointer */
#define NE_TSR   0x04u   /* Transmit Status Register */
#define NE_NCR   0x05u   /* Number of Collisions */
#define NE_ISR   0x07u   /* Interrupt Status Register */
#define NE_CRDA0 0x08u   /* Current Remote DMA Address 0 */
#define NE_RSR   0x0Cu   /* Receive Status Register */
/* Page 0 (write) */
#define NE_PSTART 0x01u  /* Page Start (RX ring start) */
#define NE_PSTOP  0x02u  /* Page Stop  (RX ring end) */
#define NE_TPSR   0x04u  /* Transmit Page Start */
#define NE_TBCR0  0x05u  /* Transmit Byte Count 0 */
#define NE_TBCR1  0x06u  /* Transmit Byte Count 1 */
#define NE_RSAR0  0x08u  /* Remote Start Address 0 */
#define NE_RSAR1  0x09u  /* Remote Start Address 1 */
#define NE_RBCR0  0x0Au  /* Remote Byte Count 0 */
#define NE_RBCR1  0x0Bu  /* Remote Byte Count 1 */
#define NE_RCR    0x0Cu  /* Receive Configuration Register */
#define NE_TCR    0x0Du  /* Transmit Configuration Register */
#define NE_DCR    0x0Eu  /* Data Configuration Register */
#define NE_IMR    0x0Fu  /* Interrupt Mask Register */
/* Page 1 */
#define NE_PAR0  0x01u   /* Physical Address 0–5 (MAC) */
#define NE_CURR  0x07u   /* Current Page */
#define NE_MAR0  0x08u   /* Multicast Address 0–7 */
/* Data port (ASIC) */
#define NE_DATA  0x10u   /* Remote DMA port (16-bit) */
#define NE_RESET 0x1Fu   /* Write triggers reset */

/* CR bits */
#define CR_STP   0x01u   /* Stop */
#define CR_STA   0x02u   /* Start */
#define CR_TXP   0x04u   /* Transmit Packet */
#define CR_RD0   0x08u   /* Remote DMA command bit 0 */
#define CR_RD1   0x10u   /* Remote DMA command bit 1 */
#define CR_RD2   0x20u   /* Remote DMA command bit 2 (abort/complete) */
#define CR_PS0   0x40u   /* Page select bit 0 */
#define CR_PS1   0x80u   /* Page select bit 1 */

#define CR_PAGE0 (CR_STP | CR_RD2)
#define CR_PAGE1 (CR_PS0 | CR_STP | CR_RD2)

/* ISR bits */
#define ISR_PRX  0x01u   /* Packet Received */
#define ISR_PTX  0x02u   /* Packet Transmitted */
#define ISR_RDC  0x40u   /* Remote DMA Complete */

/* DCR: 16-bit word DMA, FIFO threshold 8 bytes, normal addressing */
#define DCR_VAL  0x49u

/* ── NE2000 internal RAM layout (256-byte pages) ─────────────────────────── */
#define NE_TX_PAGE    0x20u   /* TX buffer area start (4 pages = 1 KiB) */
#define NE_RX_START   0x26u   /* RX ring start */
#define NE_RX_STOP    0x80u   /* RX ring end (exclusive) */

/* ── Receive packet header (prepended by NIC in the ring) ────────────────── */
struct __attribute__((packed)) ne2k_rx_hdr {
    uint8_t  status;
    uint8_t  next_page;   /* page number of next packet */
    uint16_t length;      /* total bytes including this header */
};

/* ── Driver state ────────────────────────────────────────────────────────── */
static uint16_t s_iobase   = 0;
static uint8_t  s_mac[6];
static int      s_ready    = 0;
static uint8_t  s_curr     = 0;   /* current page (RX write pointer) */
static uint8_t  s_bnry     = 0;   /* boundary (RX read pointer) */

/* Scratch buffer for remote DMA reads */
static uint8_t  s_rx_tmp[1536] __attribute__((aligned(2)));

/* ── Remote DMA helpers ──────────────────────────────────────────────────── */

/* Set up a remote DMA read of `count` bytes from NIC page-RAM at `addr` */
static void rdma_start_read(uint16_t addr, uint16_t count) {
    /* Abort any running DMA */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_RD2);
    /* Clear RDC */
    outb(s_iobase + NE_ISR, ISR_RDC);
    /* Set remote address and count */
    outb(s_iobase + NE_RSAR0, (uint8_t)(addr & 0xffu));
    outb(s_iobase + NE_RSAR1, (uint8_t)(addr >> 8));
    outb(s_iobase + NE_RBCR0, (uint8_t)(count & 0xffu));
    outb(s_iobase + NE_RBCR1, (uint8_t)(count >> 8));
    /* Start remote read DMA */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STA | CR_RD0);
}

/* Set up a remote DMA write of `count` bytes to NIC page-RAM at `addr` */
static void rdma_start_write(uint16_t addr, uint16_t count) {
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_RD2);
    outb(s_iobase + NE_ISR, ISR_RDC);
    outb(s_iobase + NE_RSAR0, (uint8_t)(addr & 0xffu));
    outb(s_iobase + NE_RSAR1, (uint8_t)(addr >> 8));
    outb(s_iobase + NE_RBCR0, (uint8_t)(count & 0xffu));
    outb(s_iobase + NE_RBCR1, (uint8_t)(count >> 8));
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STA | CR_RD1);
}

static void rdma_wait(void) {
    int i = 0;
    while (!(inb(s_iobase + NE_ISR) & ISR_RDC) && i++ < 100000)
        __asm__ volatile ("pause");
    outb(s_iobase + NE_ISR, ISR_RDC);
}

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int ne2k_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset;
    (void)kphys;
    (void)kvirt;

    uint8_t bus, dev, fn;
    /* Try both common NE2000 PCI IDs */
    if (!pci_find(0x10EC, 0x8029, &bus, &dev, &fn) &&
        !pci_find(0x1050, 0x0940, &bus, &dev, &fn)) return 0;

    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0005);

    uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
    s_iobase = (uint16_t)(bar0 & ~0x3u);

    /* Hard reset via the RESET port */
    outb(s_iobase + NE_RESET, inb(s_iobase + NE_RESET));
    for (volatile int i = 0; i < 10000; i++) __asm__ volatile ("nop");
    outb(s_iobase + NE_ISR, 0xff);   /* clear all interrupts */

    /* Page 0, stop */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STP | CR_RD2);
    /* 16-bit word DMA mode */
    outb(s_iobase + NE_DCR, DCR_VAL);
    /* Clear remote byte count */
    outb(s_iobase + NE_RBCR0, 0);
    outb(s_iobase + NE_RBCR1, 0);
    /* Monitor mode during init: ignore RX */
    outb(s_iobase + NE_RCR, 0x20);
    /* Loopback for TX */
    outb(s_iobase + NE_TCR, 0x02);
    /* RX ring boundaries */
    outb(s_iobase + NE_PSTART, NE_RX_START);
    outb(s_iobase + NE_PSTOP,  NE_RX_STOP);
    outb(s_iobase + NE_BNRY,   NE_RX_START);
    /* Mask all interrupts */
    outb(s_iobase + NE_IMR, 0x00);

    /* Read MAC from PROM via remote DMA (bytes 0–11, each byte appears twice) */
    rdma_start_read(0x0000, 12);
    uint8_t prom[12];
    for (int i = 0; i < 12; i++) prom[i] = inb(s_iobase + NE_DATA);
    rdma_wait();
    /* Every other byte is the MAC; take even-indexed bytes */
    for (int i = 0; i < 6; i++) s_mac[i] = prom[i * 2];

    /* Switch to page 1 to set CURR and multicast filters */
    outb(s_iobase + NE_CR, CR_PAGE1 | CR_STP | CR_RD2);
    for (int i = 0; i < 6; i++)
        outb(s_iobase + NE_PAR0 + (uint8_t)i, s_mac[i]);
    outb(s_iobase + NE_CURR, NE_RX_START + 1);
    for (int i = 0; i < 8; i++)
        outb(s_iobase + NE_MAR0 + (uint8_t)i, 0xff);
    s_curr = NE_RX_START + 1;
    s_bnry = NE_RX_START;

    /* Back to page 0, normal RX (broadcast + promiscuous), normal TX */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STP | CR_RD2);
    outb(s_iobase + NE_BNRY, s_bnry);
    outb(s_iobase + NE_RCR, 0x04);   /* accept broadcast */
    outb(s_iobase + NE_TCR, 0x00);   /* normal TX */

    /* Start */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STA | CR_RD2);

    s_ready = 1;
    return 1;
}

static void ne2k_mac(uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) mac[i] = s_mac[i];
}

static int ne2k_link_up(void) {
    /* NE2000 has no link-status register; assume up if initialized */
    return s_ready ? 1 : 0;
}

static int ne2k_send(const void *data, uint16_t len) {
    if (!s_ready || len > 1500) return -1;
    /* Ensure minimum Ethernet frame size */
    if (len < 64) {
        /* Would need to pad; copy to a zeroed buffer */
        static uint8_t pad[64];
        const uint8_t *src = (const uint8_t *)data;
        for (uint16_t i = 0; i < len; i++) pad[i] = src[i];
        for (uint16_t i = len; i < 64; i++) pad[i] = 0;
        data = pad;
        len  = 64;
    }
    uint16_t count = (uint16_t)((len + 1u) & ~1u);   /* round up to word */
    uint16_t dest  = (uint16_t)NE_TX_PAGE << 8;

    /* Remote DMA write: copy packet to NIC internal RAM */
    rdma_start_write(dest, count);
    const uint16_t *src16 = (const uint16_t *)data;
    for (uint16_t i = 0; i < count / 2; i++)
        outw(s_iobase + NE_DATA, src16[i]);
    rdma_wait();

    /* Set up TX: page start and byte count */
    outb(s_iobase + NE_TPSR,  NE_TX_PAGE);
    outb(s_iobase + NE_TBCR0, (uint8_t)(len & 0xffu));
    outb(s_iobase + NE_TBCR1, (uint8_t)(len >> 8));

    /* Transmit */
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STA | CR_RD2 | CR_TXP);

    /* Poll until TX complete */
    for (int i = 0; i < 100000; i++) {
        uint8_t isr = inb(s_iobase + NE_ISR);
        if (isr & ISR_PTX) { outb(s_iobase + NE_ISR, ISR_PTX); break; }
        __asm__ volatile ("pause");
    }
    return 0;
}

static int ne2k_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_ready) return -1;

    /* Read CURR from page 1 */
    outb(s_iobase + NE_CR, CR_PAGE1 | CR_STA | CR_RD2);
    uint8_t curr = inb(s_iobase + NE_CURR);
    outb(s_iobase + NE_CR, CR_PAGE0 | CR_STA | CR_RD2);

    uint8_t bnry = (uint8_t)(s_bnry + 1u);
    if (bnry >= NE_RX_STOP) bnry = NE_RX_START;

    if (bnry == curr) return -1;   /* ring empty */

    /* Read the 4-byte packet header from the ring */
    uint16_t hdr_addr = (uint16_t)bnry << 8;
    rdma_start_read(hdr_addr, 4);
    uint8_t h[4];
    for (int i = 0; i < 4; i++) h[i] = inb(s_iobase + NE_DATA);
    rdma_wait();

    struct ne2k_rx_hdr *hdr = (struct ne2k_rx_hdr *)h;
    uint16_t pkt_len = hdr->length;    /* includes 4-byte header */
    uint8_t  next    = hdr->next_page;

    if (pkt_len < 4 || pkt_len > 1518 + 4) {
        /* Bogus packet — advance past it */
        s_bnry = (next == NE_RX_START) ? (uint8_t)(NE_RX_STOP - 1) : (uint8_t)(next - 1);
        outb(s_iobase + NE_BNRY, s_bnry);
        return -1;
    }

    uint16_t data_len = (uint16_t)(pkt_len - 4);   /* strip header */
    uint16_t copy_len = data_len < maxlen ? data_len : maxlen;
    *len_out = copy_len;

    /* Remote DMA read of the payload (skip 4-byte header) */
    uint16_t payload_addr = (uint16_t)(hdr_addr + 4u);
    uint16_t read_count   = (uint16_t)((copy_len + 1u) & ~1u);
    rdma_start_read(payload_addr, read_count);
    uint16_t *dst16 = (uint16_t *)s_rx_tmp;
    for (uint16_t i = 0; i < read_count / 2; i++)
        dst16[i] = inw(s_iobase + NE_DATA);
    rdma_wait();

    uint8_t *dst = (uint8_t *)buf;
    for (uint16_t i = 0; i < copy_len; i++) dst[i] = s_rx_tmp[i];

    /* Advance BNRY */
    s_bnry = (next == NE_RX_START) ? (uint8_t)(NE_RX_STOP - 1) : (uint8_t)(next - 1);
    outb(s_iobase + NE_BNRY, s_bnry);
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct nic_driver ne2k_driver = {
    .name    = "ne2k",
    .probe   = ne2k_probe,
    .mac     = ne2k_mac,
    .link_up = ne2k_link_up,
    .send    = ne2k_send,
    .recv    = ne2k_recv,
};
