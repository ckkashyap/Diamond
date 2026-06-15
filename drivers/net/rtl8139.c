/*
 * rtl8139.c - Realtek RTL8139 NIC driver
 *
 * PCI vendor/device: 0x10EC / 0x8139
 * Uses I/O BAR0 (port I/O, not MMIO).
 * TX: 4 fixed descriptor slots (round-robin).
 * RX: 8 KiB + 16-byte wraparound ring in BSS.
 * No DMA address translation needed for RX (ring is copied via port I/O
 * is not used — the NIC DMA's directly into the ring buffer, so we need
 * virt_to_phys for the ring buffer physical address).
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/io.h"
#include "nic.h"

/* ── Register offsets (from I/O base) ───────────────────────────────────── */
#define RTL_MAC0       0x00u   /* MAC address bytes 0-5 */
#define RTL_MAR0       0x08u   /* Multicast address registers */
#define RTL_TXSTATUS0  0x10u   /* TX status for slot 0 (32-bit × 4) */
#define RTL_TXADDR0    0x20u   /* TX buffer address for slot 0 (32-bit × 4) */
#define RTL_RXBUF      0x30u   /* RX ring buffer start address */
#define RTL_CMD        0x37u   /* Command register (8-bit) */
#define RTL_CAPR       0x38u   /* Current address of packet read (16-bit) */
#define RTL_CBR        0x3Au   /* Current buffer address (16-bit, read-only) */
#define RTL_IMR        0x3Cu   /* Interrupt mask register (16-bit) */
#define RTL_ISR        0x3Eu   /* Interrupt status register (16-bit) */
#define RTL_TXCFG      0x40u   /* TX configuration (32-bit) */
#define RTL_RXCFG      0x44u   /* RX configuration (32-bit) */
#define RTL_MPC        0x4Cu   /* Missed packet counter (32-bit) */
#define RTL_CFG1       0x52u   /* Configuration register 1 (8-bit) */

/* CMD bits */
#define CMD_RST        0x10u
#define CMD_RE         0x08u   /* Receiver Enable */
#define CMD_TE         0x04u   /* Transmitter Enable */

/* RX config: accept all, no-wrap (ring + 16-byte overflow), 8K buffer */
#define RXCFG_VAL   0x0000008Fu   /* AAP|APM|AM|AB, no WRAP, 8K ring */

/* TX config: standard interframe gap, DMA burst 1024 */
#define TXCFG_VAL   0x00000600u

/* TX status: OWN bit (clear = hardware owns, set = software owns) */
#define TXS_OWN     (1u << 13)

/* ── Static buffers ──────────────────────────────────────────────────────── */
#define RX_BUF_SIZE   (8192 + 16 + 1500)   /* ring + overflow space */
#define TX_BUF_SIZE   1536
#define TX_SLOTS      4

static uint8_t s_rx_buf[RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx_buf[TX_SLOTS][TX_BUF_SIZE] __attribute__((aligned(4)));

/* ── Driver state ────────────────────────────────────────────────────────── */
static uint16_t s_iobase = 0;
static uint8_t  s_mac[6];
static int      s_ready   = 0;
static int      s_tx_slot = 0;
static uint16_t s_rx_ptr  = 0;   /* CAPR (our read pointer into the ring) */
static uint64_t s_kphys   = 0;
static uint64_t s_kvirt   = 0;

static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int rtl8139_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset;
    s_kphys = kphys;
    s_kvirt = kvirt;

    uint8_t bus, dev, fn;
    if (!pci_find(0x10EC, 0x8139, &bus, &dev, &fn)) return 0;

    /* Enable bus-mastering + I/O space */
    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0005);

    /* BAR0 is an I/O BAR (bit 0 = 1); strip type bits to get base port */
    uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
    s_iobase = (uint16_t)(bar0 & ~0x3u);

    /* Power on */
    outb(s_iobase + RTL_CFG1, 0x00);

    /* Software reset */
    outb(s_iobase + RTL_CMD, CMD_RST);
    while (inb(s_iobase + RTL_CMD) & CMD_RST) __asm__ volatile ("pause");

    /* Mask all interrupts */
    outw(s_iobase + RTL_IMR, 0x0000);

    /* Set RX buffer address (physical) */
    uint64_t rx_phys = virt_to_phys(s_rx_buf);
    outl(s_iobase + RTL_RXBUF, (uint32_t)(rx_phys & 0xffffffffu));

    /* Accept all packets, no-wrap 8K ring */
    outl(s_iobase + RTL_RXCFG, RXCFG_VAL);

    /* Set multicast registers to all-accept */
    outl(s_iobase + RTL_MAR0,     0xffffffffu);
    outl(s_iobase + RTL_MAR0 + 4, 0xffffffffu);

    /* TX configuration */
    outl(s_iobase + RTL_TXCFG, TXCFG_VAL);

    /* Enable RX + TX */
    outb(s_iobase + RTL_CMD, CMD_RE | CMD_TE);

    /* Read MAC from the I/O space PROM registers */
    for (int i = 0; i < 6; i++)
        s_mac[i] = inb(s_iobase + RTL_MAC0 + (uint8_t)i);

    s_rx_ptr = 0;
    s_tx_slot = 0;
    s_ready = 1;
    return 1;
}

static void rtl8139_mac(uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) mac[i] = s_mac[i];
}

static int rtl8139_link_up(void) {
    if (!s_ready) return 0;
    /* Media Status Register is at 0x58; bit 2 = link status (1 = down) */
    uint8_t msr = inb(s_iobase + 0x58);
    return (msr & 0x04) ? 0 : 1;
}

static int rtl8139_send(const void *data, uint16_t len) {
    if (!s_ready || len > TX_BUF_SIZE) return -1;

    /* Wait until the current slot is free (OWN bit set = available) */
    uint32_t offset = (uint32_t)s_tx_slot * 4u;
    while (!(inl(s_iobase + RTL_TXSTATUS0 + offset) & TXS_OWN))
        __asm__ volatile ("pause");

    /* Copy into the static TX buffer */
    const uint8_t *src = (const uint8_t *)data;
    uint8_t       *dst = s_tx_buf[s_tx_slot];
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    /* Set physical address of TX buffer */
    uint64_t phys = virt_to_phys(dst);
    outl(s_iobase + RTL_TXADDR0 + offset, (uint32_t)(phys & 0xffffffffu));

    /* Writing the length to TxStatus clears OWN and starts DMA */
    outl(s_iobase + RTL_TXSTATUS0 + offset, (uint32_t)(len & 0x1fffu));

    s_tx_slot = (s_tx_slot + 1) % TX_SLOTS;
    return 0;
}

static int rtl8139_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_ready) return -1;

    /* Check if a packet is available: CBR != CAPR (adjusted) */
    uint16_t cbr  = inw(s_iobase + RTL_CBR);
    uint16_t capr = (uint16_t)(s_rx_ptr % 8192u);
    if (capr == (cbr % 8192u)) return -1;   /* ring empty */

    /* Packet header: 2-byte status + 2-byte length at s_rx_ptr */
    uint32_t ptr = s_rx_ptr;
    /* uint16_t rx_status = *(uint16_t *)(s_rx_buf + ptr); */  /* unused */
    uint16_t rx_len = *(uint16_t *)(s_rx_buf + ptr + 2);
    /* rx_len includes the 4-byte CRC appended by the NIC */
    if (rx_len < 4) { s_rx_ptr = (s_rx_ptr + 4 + 3) & ~3u; return -1; }
    uint16_t data_len = rx_len - 4;

    uint16_t copy_len = data_len < maxlen ? data_len : maxlen;
    *len_out = copy_len;

    /* Copy packet data (skip 4-byte header) */
    const uint8_t *src = s_rx_buf + ptr + 4;
    uint8_t       *dst = (uint8_t *)buf;
    for (uint16_t i = 0; i < copy_len; i++) dst[i] = src[i];

    /* Advance read pointer (4-byte header + data + CRC, DWORD aligned) */
    s_rx_ptr = (uint16_t)((s_rx_ptr + 4 + rx_len + 3) & ~3u);
    s_rx_ptr %= 8192u;

    /* Update CAPR: hardware adds 0x10 to the value we write */
    outw(s_iobase + RTL_CAPR, (uint16_t)(s_rx_ptr - 0x10u));
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct nic_driver rtl8139_driver = {
    .name    = "rtl8139",
    .probe   = rtl8139_probe,
    .mac     = rtl8139_mac,
    .link_up = rtl8139_link_up,
    .send    = rtl8139_send,
    .recv    = rtl8139_recv,
};
