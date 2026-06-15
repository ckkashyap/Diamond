/*
 * e1000.c - Intel 82540EM (e1000) NIC driver
 *
 * PCI vendor/device: 0x8086 / 0x100E
 * BAR0: 32-bit MMIO (QEMU presents it as a 32-bit memory BAR).
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/spinlock.h"
#include "nic.h"

/* ── Register offsets ────────────────────────────────────────────────────── */
#define E1000_CTRL    0x0000u
#define E1000_STATUS  0x0008u
#define E1000_ICR     0x00C0u
#define E1000_IMC     0x00D8u
#define E1000_RCTL    0x0100u
#define E1000_TCTL    0x0400u
#define E1000_TIPG    0x0410u
#define E1000_RDBAL   0x2800u
#define E1000_RDBAH   0x2804u
#define E1000_RDLEN   0x2808u
#define E1000_RDH     0x2810u
#define E1000_RDT     0x2818u
#define E1000_TDBAL   0x3800u
#define E1000_TDBAH   0x3804u
#define E1000_TDLEN   0x3808u
#define E1000_TDH     0x3810u
#define E1000_TDT     0x3818u
#define E1000_RAL0    0x5400u
#define E1000_RAH0    0x5404u
#define E1000_MTA     0x5200u

#define CTRL_SLU    (1u << 6)
#define CTRL_RST    (1u << 26)
#define RCTL_EN     (1u << 1)
#define RCTL_UPE    (1u << 3)
#define RCTL_MPE    (1u << 4)
#define RCTL_BAM    (1u << 15)
#define RCTL_SECRC  (1u << 26)
#define TCTL_EN     (1u << 1)
#define TCTL_PSP    (1u << 3)
#define TCTL_CT     (0x10u << 4)
#define TCTL_COLD   (0x40u << 12)
#define TDCMD_EOP   0x01u
#define TDCMD_IFCS  0x02u
#define TDCMD_RS    0x08u
#define STA_DD      0x01u

/* ── Descriptor structs ──────────────────────────────────────────────────── */

struct __attribute__((packed)) tx_desc {
    uint64_t addr;
    uint16_t length;
    uint8_t  cso, cmd, sta, css;
    uint16_t special;
};

struct __attribute__((packed)) rx_desc {
    uint64_t addr;
    uint16_t length, checksum;
    uint8_t  sta, errors;
    uint16_t special;
};

/* ── Ring buffers ────────────────────────────────────────────────────────── */
#define TX_COUNT  16
#define RX_COUNT  16
#define BUF_SIZE  2048

static struct tx_desc s_tx_ring[TX_COUNT] __attribute__((aligned(16)));
static struct rx_desc s_rx_ring[RX_COUNT] __attribute__((aligned(16)));
static uint8_t s_rx_bufs[RX_COUNT][BUF_SIZE] __attribute__((aligned(16)));
static uint8_t s_tx_bufs[TX_COUNT][BUF_SIZE] __attribute__((aligned(16)));

/* ── Driver state ────────────────────────────────────────────────────────── */
static volatile uint32_t *s_regs = (void *)0;
static uint8_t  s_mac[6];
static int      s_ready  = 0;
static int      s_rx_idx = 0;
static int      s_tx_idx = 0;
static spinlock_t s_tx_lock = SPINLOCK_INIT;
static spinlock_t s_rx_lock = SPINLOCK_INIT;
static uint64_t s_kphys = 0;
static uint64_t s_kvirt = 0;

static uint64_t virt_to_phys(const void *v) {
    return (uint64_t)(uintptr_t)v - s_kvirt + s_kphys;
}

static uint32_t reg_r(uint32_t off) { return s_regs[off >> 2]; }
static void     reg_w(uint32_t off, uint32_t v) { s_regs[off >> 2] = v; }

/* ── Probe / init ────────────────────────────────────────────────────────── */

static int e1000_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    s_kphys = kphys;
    s_kvirt = kvirt;

    uint8_t bus, dev, fn;
    if (!pci_find(0x8086, 0x100E, &bus, &dev, &fn)) return 0;

    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, cmd | 0x0006);

    /* BAR0: check bits [2:1] for 32-bit vs 64-bit */
    uint32_t bar0_lo = pci_read32(bus, dev, fn, 0x10);
    uint64_t bar_phys;
    if ((bar0_lo & 0x6u) == 0x4u) {
        uint32_t bar0_hi = pci_read32(bus, dev, fn, 0x14);
        bar_phys = ((uint64_t)bar0_hi << 32) | (bar0_lo & ~0xfU);
    } else {
        bar_phys = bar0_lo & ~0xfU;
    }
    s_regs = (volatile uint32_t *)(bar_phys + hhdm_offset);

    reg_w(E1000_IMC, 0xffffffffu);
    (void)reg_r(E1000_ICR);

    reg_w(E1000_CTRL, reg_r(E1000_CTRL) | CTRL_RST);
    for (volatile int i = 0; i < 20000; i++) __asm__ volatile ("nop");
    while (reg_r(E1000_CTRL) & CTRL_RST) __asm__ volatile ("pause");

    reg_w(E1000_IMC, 0xffffffffu);
    (void)reg_r(E1000_ICR);

    reg_w(E1000_CTRL, reg_r(E1000_CTRL) | CTRL_SLU);

    for (int i = 0; i < 128; i++) reg_w(E1000_MTA + (uint32_t)i * 4, 0);

    for (int i = 0; i < RX_COUNT; i++) {
        s_rx_ring[i].addr   = virt_to_phys(s_rx_bufs[i]);
        s_rx_ring[i].sta    = 0;
        s_rx_ring[i].errors = 0;
    }
    uint64_t rx_phys = virt_to_phys(s_rx_ring);
    reg_w(E1000_RDBAL, (uint32_t)(rx_phys & 0xffffffffu));
    reg_w(E1000_RDBAH, (uint32_t)(rx_phys >> 32));
    reg_w(E1000_RDLEN, (uint32_t)(RX_COUNT * 16));
    reg_w(E1000_RDH,   0);
    reg_w(E1000_RDT,   RX_COUNT - 1);
    s_rx_idx = 0;

    reg_w(E1000_RCTL, RCTL_EN | RCTL_UPE | RCTL_MPE | RCTL_BAM | RCTL_SECRC);

    for (int i = 0; i < TX_COUNT; i++) {
        s_tx_ring[i].sta = STA_DD;
        s_tx_ring[i].cmd = 0;
    }
    uint64_t tx_phys = virt_to_phys(s_tx_ring);
    reg_w(E1000_TDBAL, (uint32_t)(tx_phys & 0xffffffffu));
    reg_w(E1000_TDBAH, (uint32_t)(tx_phys >> 32));
    reg_w(E1000_TDLEN, (uint32_t)(TX_COUNT * 16));
    reg_w(E1000_TDH,   0);
    reg_w(E1000_TDT,   0);
    s_tx_idx = 0;

    reg_w(E1000_TCTL, TCTL_EN | TCTL_PSP | TCTL_CT | TCTL_COLD);
    reg_w(E1000_TIPG, 0x0060200Au);

    uint32_t ral = reg_r(E1000_RAL0);
    uint32_t rah = reg_r(E1000_RAH0);
    s_mac[0] = (ral >>  0) & 0xff;
    s_mac[1] = (ral >>  8) & 0xff;
    s_mac[2] = (ral >> 16) & 0xff;
    s_mac[3] = (ral >> 24) & 0xff;
    s_mac[4] = (rah >>  0) & 0xff;
    s_mac[5] = (rah >>  8) & 0xff;

    s_ready = 1;
    return 1;
}

static void e1000_mac(uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) mac[i] = s_mac[i];
}

static int e1000_link_up(void) {
    if (!s_ready) return 0;
    return (reg_r(E1000_STATUS) & 0x02) ? 1 : 0;
}

static int e1000_send(const void *data, uint16_t len) {
    if (!s_ready || len > BUF_SIZE) return -1;

    spin_lock(&s_tx_lock);

    struct tx_desc *d = &s_tx_ring[s_tx_idx];
    while (!(d->sta & STA_DD)) __asm__ volatile ("pause");

    const uint8_t *src = (const uint8_t *)data;
    uint8_t       *dst = s_tx_bufs[s_tx_idx];
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    d->addr    = virt_to_phys(dst);
    d->length  = len;
    d->cmd     = TDCMD_EOP | TDCMD_IFCS | TDCMD_RS;
    d->sta     = 0;
    d->cso     = 0;
    d->css     = 0;
    d->special = 0;

    s_tx_idx = (s_tx_idx + 1) % TX_COUNT;
    reg_w(E1000_TDT, (uint32_t)s_tx_idx);

    spin_unlock(&s_tx_lock);
    return 0;
}

static int e1000_recv(void *buf, uint16_t maxlen, uint16_t *len_out) {
    if (!s_ready) return -1;

    spin_lock(&s_rx_lock);

    struct rx_desc *d = &s_rx_ring[s_rx_idx];
    if (!(d->sta & STA_DD)) {
        spin_unlock(&s_rx_lock);
        return -1;
    }

    uint16_t len = d->length;
    if (len > maxlen) len = maxlen;
    *len_out = len;

    const uint8_t *src = s_rx_bufs[s_rx_idx];
    uint8_t       *dst = (uint8_t *)buf;
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];

    d->sta    = 0;
    d->errors = 0;
    d->length = 0;
    reg_w(E1000_RDT, (uint32_t)s_rx_idx);
    s_rx_idx = (s_rx_idx + 1) % RX_COUNT;

    spin_unlock(&s_rx_lock);
    return 0;
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct nic_driver e1000_driver = {
    .name    = "e1000",
    .probe   = e1000_probe,
    .mac     = e1000_mac,
    .link_up = e1000_link_up,
    .send    = e1000_send,
    .recv    = e1000_recv,
};
