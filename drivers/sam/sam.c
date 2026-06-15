/*
 * sam.c — Surface Aggregator Module (SSAM/SSH) keyboard driver
 *
 * The Surface Laptop 3 built-in keyboard is a HID device hanging off the
 * Surface Aggregator Module (SAM, a STM32 hub).  Host talks to SAM over
 * Intel LPSS UART0 [8086:34a8] at PCI 0:1E.0 at 4 Mbaud, 8N1, HW RTS/CTS.
 *
 * What this driver does:
 *   1. PCI D3→D0 on the LPSS UART0 device
 *   2. Re-route GPP_C8-C11 pads back to UART0 native function (UEFI's _PS3
 *      leaves them muxed as GPIO with TXDIS set — without this, TX floats)
 *   3. Reprogram the 64-bit BAR (UEFI clears it at ExitBootServices)
 *   4. Map the UART MMIO as UC (WB kills LPSS via cache-line fills)
 *   5. Re-enable 8N1 / FIFO / AFCE on top of UEFI's clock+baud
 *   6. SSAM handshake: D0-entry → display-on → event subscribe via
 *      SSAM_EVENT_REGISTRY_REG(tid=KIP) — SL3 keyboard event registry
 *   7. sam_getchar() decodes DATA_NSQ HID reports to ASCII
 *
 * Key gotchas buried in past bring-up:
 *   - SAM is on UART0 (PCI 0:1E.0), NOT UART2.  Pads are GPP_C8..C11 in
 *     ICL-LP Community 4 (PID 0x6A), stride 0x10 (4 DWs per pad because
 *     ICL-LP has PINCTRL_FEATURE_DEBOUNCE), base 0x600.
 *   - PAD_CFG_DW0: bits[13:10]=PMODE (1=native UART0), bit8=GPIOTXDIS,
 *     bit9=GPIORXDIS.  Both must be cleared.
 *   - SL3 keyboard events are enabled via SSAM_EVENT_REGISTRY_REG
 *     (TC=0x21, TID=0x02, CID_en=0x01), NOT SSAM_EVENT_REGISTRY_SAM
 *     (TC=0x01, CID_en=0x0B) which is only for SL1/SL2.
 *   - Keyboard events arrive as DATA_NSQ (type=0x00) frames, no ACK needed.
 */

#include <stdint.h>
#include "../../arch/x86/pci.h"
#include "../../arch/x86/vm.h"
#include "../../drivers/terminal.h"
#include "../../kernel/alloc.h"
#include "sam.h"

/* ── PCI / BAR ────────────────────────────────────────────────────────────── */
#define SAM_VID       0x8086u
#define SAM_DID       0x34a8u   /* ICL-LP LPSS UART #0 */
#define SAM_BAR_PHYS  UINT64_C(0x4010003000)

/* ── DW APB UART (32-bit stride) ──────────────────────────────────────────── */
#define U_RBR  0x00u
#define U_THR  0x00u
#define U_IER  0x04u
#define U_FCR  0x08u
#define U_LCR  0x0Cu
#define U_MCR  0x10u
#define U_LSR  0x14u
#define U_MSR  0x18u

#define LSR_DR    0x01u
#define LSR_THRE  0x20u
#define LSR_TEMT  0x40u

#define RD32(base, off)        (*(volatile uint32_t *)((uint8_t *)(base) + (off)))
#define WR32(base, off, val)   (*(volatile uint32_t *)((uint8_t *)(base) + (off)) = (uint32_t)(val))

/* ── SSAM/SSH protocol ────────────────────────────────────────────────────── */
#define SSH_SYNC0   0xAAu
#define SSH_SYNC1   0x55u
#define SSH_DATA    0x80u   /* DATA_SEQ — sequenced, needs ACK  */
#define SSH_DATA_NS 0x00u   /* DATA_NSQ — unsequenced, no ACK (keyboard events) */
#define SSH_ACK     0x40u
#define SSH_NAK     0x04u

#define SSH_TC_SAM  0x01u   /* SAM controller target category */
#define SSH_TC_HID  0x15u   /* HID target category (for events) */
#define SSH_TC_REG  0x21u   /* SSAM_EVENT_REGISTRY_REG target category (SL3+) */
#define SSH_TID_SAM 0x01u
#define SSH_TID_KIP 0x02u   /* Keyboard Input Processor */

#define SSH_CID_D0_ENTRY    0x34u
#define SSH_CID_DISPLAY_ON  0x16u
#define SSH_CID_REG_EN      0x01u   /* REG registry: enable event */

#define RQID_BASE   0x27u   /* events use 0x01..0x26, host rqids start at 0x27 */

/* ── Driver state ─────────────────────────────────────────────────────────── */
static void    *s_base   = (void *)0;
static uint8_t  s_seq_tx = 0;
static uint8_t  s_rqid   = RQID_BASE;
static int      s_ready  = 0;

/* ── Diagnostic hex printers ──────────────────────────────────────────────── */
static void ph8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[v >> 4]);
    term_putchar(h[v & 0xf]);
}
static void ph32(uint32_t v) {
    ph8((uint8_t)(v >> 24)); ph8((uint8_t)(v >> 16));
    ph8((uint8_t)(v >>  8)); ph8((uint8_t)(v));
}

/* ── CRC-16/CCITT-FALSE (matches Linux crc_itu_t(0xffff, ...)) ────────────── */
static uint16_t crc16(const uint8_t *buf, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ── UART TX/RX ───────────────────────────────────────────────────────────── */
static void uart_tx(uint8_t b) {
    uint32_t n = 200000;
    while (!(RD32(s_base, U_LSR) & LSR_THRE) && --n) ;
    WR32(s_base, U_THR, b);
}

static int uart_rx_ready(void) {
    return (RD32(s_base, U_LSR) & LSR_DR) ? 1 : 0;
}

static uint8_t uart_rx_byte(void) {
    return (uint8_t)(RD32(s_base, U_RBR) & 0xFFu);
}

static void flush_rx(void) {
    while (uart_rx_ready()) (void)uart_rx_byte();
}

static void udelay(uint32_t n) {
    for (volatile uint32_t i = 0; i < n * 10u; i++) ;
}

/* Set to 1 to dump every TX/RX byte and MSR/LSR state for debugging.  */
static int s_dbg = 0;
void sam_set_dbg(int on) { s_dbg = on ? 1 : 0; }

/* ── SSAM frame TX ────────────────────────────────────────────────────────── */
static void ssam_tx_frame(uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint16_t plen) {
    uint8_t hdr[4] = { type,
                       (uint8_t)(plen & 0xFF),
                       (uint8_t)(plen >> 8),
                       seq };
    uint16_t fcrc = crc16(hdr, 4);
    uint16_t pcrc = plen ? crc16(payload, plen) : 0xFFFFu;

    if (s_dbg) {
        term_puts("  TX=[aa 55 ");
        ph8(hdr[0]); term_putchar(' ');
        ph8(hdr[1]); term_putchar(' ');
        ph8(hdr[2]); term_putchar(' ');
        ph8(hdr[3]); term_putchar(' ');
        ph8((uint8_t)(fcrc & 0xFF)); term_putchar(' ');
        ph8((uint8_t)(fcrc >> 8)); term_putchar(' ');
        for (uint16_t i = 0; i < plen; i++) { ph8(payload[i]); term_putchar(' '); }
        ph8((uint8_t)(pcrc & 0xFF)); term_putchar(' ');
        ph8((uint8_t)(pcrc >> 8));
        term_puts("]\n");
    }

    uart_tx(SSH_SYNC0); uart_tx(SSH_SYNC1);
    uart_tx(hdr[0]); uart_tx(hdr[1]); uart_tx(hdr[2]); uart_tx(hdr[3]);
    uart_tx((uint8_t)(fcrc & 0xFF)); uart_tx((uint8_t)(fcrc >> 8));
    for (uint16_t i = 0; i < plen; i++) uart_tx(payload[i]);
    uart_tx((uint8_t)(pcrc & 0xFF)); uart_tx((uint8_t)(pcrc >> 8));

    /* Wait for TX shift register to drain so the remote sees a full frame
     * before we start polling for its ACK.                                 */
    uint32_t n = 500000u;
    while (!(RD32(s_base, U_LSR) & LSR_TEMT) && --n) ;
    if (s_dbg && !n) {
        term_puts("  TX_STUCK LSR=0x"); ph32(RD32(s_base, U_LSR));
        term_puts(" MSR=0x"); ph32(RD32(s_base, U_MSR)); term_putchar('\n');
    }
}

static void ssam_send_cmd(uint8_t tc, uint8_t tid, uint8_t iid, uint8_t cid,
                          const uint8_t *data, uint8_t dlen) {
    uint8_t payload[64];
    payload[0] = 0x80;              /* ssh_command.type = CMD */
    payload[1] = tc;
    payload[2] = tid;
    payload[3] = 0x00;              /* sid = SSAM_SSH_TID_HOST */
    payload[4] = iid;
    payload[5] = s_rqid++;
    payload[6] = 0x00;              /* rqid high byte */
    payload[7] = cid;
    uint8_t plen = 8;
    for (uint8_t i = 0; i < dlen; i++) payload[plen++] = data[i];
    ssam_tx_frame(SSH_DATA, s_seq_tx++, payload, plen);
}

static void ssam_send_ack(uint8_t seq) {
    ssam_tx_frame(SSH_ACK, seq, (void *)0, 0);
}

/* ── SSAM frame RX state machine ──────────────────────────────────────────── */
#define RX_PAYLOAD_MAX  128

static uint8_t  s_rxstate = 0;
static uint8_t  s_rxhdr[4];
static uint8_t  s_rxpayload[RX_PAYLOAD_MAX];
static uint16_t s_rxpos;
static uint16_t s_rxplen;
static uint8_t  s_frame_type;
static uint8_t  s_frame_seq;
static uint16_t s_frame_plen;

static int ssam_rx_poll(void) {
    /* If a long stall (e.g. raytrace using all cores) caused the 64-byte
     * UART RX FIFO to overflow, bytes are lost mid-frame and our state
     * machine is now pointing into garbage.  LSR.OE (bit 1) reads-to-clear:
     * drain the FIFO and resync.                                         */
    uint32_t lsr = RD32(s_base, U_LSR);
    if (lsr & 0x02u) {
        while (uart_rx_ready()) (void)uart_rx_byte();
        s_rxstate = 0;
        return 0;
    }
    while (uart_rx_ready()) {
        uint8_t b = uart_rx_byte();
        switch (s_rxstate) {
        case 0:
            if (b == SSH_SYNC0) s_rxstate = 1;
            break;
        case 1:
            s_rxstate = (b == SSH_SYNC1) ? 2
                      : (b == SSH_SYNC0) ? 1
                      : 0;
            break;
        case 2: case 3: case 4: case 5:
            s_rxhdr[s_rxstate - 2] = b;
            s_rxstate++;
            if (s_rxstate == 6) {
                s_frame_type = s_rxhdr[0];
                s_rxplen     = (uint16_t)(s_rxhdr[1] | ((uint16_t)s_rxhdr[2] << 8));
                s_frame_seq  = s_rxhdr[3];
                s_rxpos      = 0;
                /* Sanity: keyboard HID reports are 20 bytes, ACK/NAK are 0.
                 * If plen is bigger than our buffer, the "AA 55" we saw
                 * was almost certainly inside an unrelated payload — resync. */
                if (s_rxplen > RX_PAYLOAD_MAX) s_rxstate = 0;
            }
            break;
        case 6: s_rxstate = 7; break;           /* fcrc_lo */
        case 7:
            s_rxpos   = 0;
            s_rxstate = (s_rxplen == 0) ? 10 : 8;
            break;                              /* fcrc_hi */
        case 8:
            if (s_rxpos < RX_PAYLOAD_MAX)
                s_rxpayload[s_rxpos] = b;
            s_rxpos++;
            if (s_rxpos >= s_rxplen) s_rxstate = 10;
            break;
        case 10: s_rxstate = 11; break;         /* pcrc_lo */
        case 11:
            s_frame_plen = s_rxpos;
            s_rxstate    = 0;
            return 1;
        }
    }
    return 0;
}

/* Send an SSH command and wait for ACK/DATA/NAK.  Returns 1 on ACK or DATA,
 * 0 on NAK or timeout.  When s_dbg is set, logs parsed frames and raw bytes
 * that don't complete a frame.                                             */
static int ssam_cmd_wait(uint8_t tc, uint8_t tid, uint8_t iid, uint8_t cid,
                         const uint8_t *data, uint8_t dlen) {
    ssam_send_cmd(tc, tid, iid, cid, data, dlen);
    int got = 0, ok = 0;
    uint32_t raw = 0;
    if (s_dbg) term_puts("  RX=[");
    for (uint32_t t = 4000000u; t; --t) {
        if (!ssam_rx_poll()) {
            if (got && t > 200000u) t = 200000u;
            continue;
        }
        if (s_dbg) {
            const char *ft =
                (s_frame_type == SSH_ACK)     ? "ACK" :
                (s_frame_type == SSH_NAK)     ? "NAK" :
                (s_frame_type == SSH_DATA)    ? "DAT" : "NSQ";
            term_puts(ft); term_putchar(':'); ph8(s_frame_seq);
            term_putchar('('); ph8((uint8_t)s_frame_plen); term_putchar(')');
            term_putchar(' ');
        }
        if (s_frame_type == SSH_DATA) ssam_send_ack(s_frame_seq);
        if (!got) {
            if (s_frame_type == SSH_ACK || s_frame_type == SSH_DATA ||
                s_frame_type == SSH_DATA_NS) { got = 1; ok = 1; }
            else if (s_frame_type == SSH_NAK) { got = 1; ok = 0; }
        }
    }
    /* Dump anything still sitting in RX FIFO after timeout */
    while (uart_rx_ready()) {
        uint8_t b = uart_rx_byte();
        if (s_dbg) { ph8(b); term_putchar(' '); }
        raw++;
    }
    if (s_dbg) {
        term_puts("] raw_tail=0x"); ph32(raw);
        term_puts(" LSR=0x"); ph32(RD32(s_base, U_LSR));
        term_puts(" MSR=0x"); ph32(RD32(s_base, U_MSR)); term_putchar('\n');
    }
    s_rxstate = 0;       /* reset RX state machine between commands */
    flush_rx();
    return got ? ok : 0;
}

/* ── HID keycode → ASCII ──────────────────────────────────────────────────── */
static const char s_hid_lower[64] = {
/*00*/  0,    0,    0,    0,
/*04*/  'a',  'b',  'c',  'd',  'e',  'f',  'g',  'h',
/*0C*/  'i',  'j',  'k',  'l',  'm',  'n',  'o',  'p',
/*14*/  'q',  'r',  's',  't',  'u',  'v',  'w',  'x',
/*1C*/  'y',  'z',  '1',  '2',  '3',  '4',  '5',  '6',
/*24*/  '7',  '8',  '9',  '0',  '\n', 27,   8,    '\t',
/*2C*/  ' ',  '-',  '=',  '[',  ']',  '\\', 0,    ';',
/*34*/  '\'', '`',  ',',  '.',  '/',  0,    0,    0,
};
static const char s_hid_upper[64] = {
/*00*/  0,    0,    0,    0,
/*04*/  'A',  'B',  'C',  'D',  'E',  'F',  'G',  'H',
/*0C*/  'I',  'J',  'K',  'L',  'M',  'N',  'O',  'P',
/*14*/  'Q',  'R',  'S',  'T',  'U',  'V',  'W',  'X',
/*1C*/  'Y',  'Z',  '!',  '@',  '#',  '$',  '%',  '^',
/*24*/  '&',  '*',  '(',  ')',  '\n', 27,   8,    '\t',
/*2C*/  ' ',  '_',  '+',  '{',  '}',  '|',  0,    ':',
/*34*/  '"',  '~',  '<',  '>',  '?',  0,    0,    0,
};

static uint8_t s_prev_keys[6];
static uint8_t s_caps = 0;
static uint8_t s_mods = 0;

static int hid_printable_key(uint8_t kc) {
    return kc < 64u && s_hid_lower[kc] != 0;
}

static uint8_t hid_report_data_offset(const uint8_t *report, uint8_t rlen) {
    if (rlen >= 9u && report[0] == 0x01u)
        return 1u;
    return 0u;
}

static int hid_decode(const uint8_t *report, uint8_t rlen) {
    if (rlen < 3) return -1;

    /* DEBUG: dump the first 8 bytes of every HID input report on screen so
     * we can see the real layout (modifier + reserved + keycodes positions).
     * The standard boot keyboard report is 8 bytes; SAM may prefix it with a
     * Report ID byte (then mods is at offset 1, not 0) — observing real
     * captures from "shift+a" vs "a" disambiguates which layout this is. */
    if (s_dbg) {
        term_puts("HID rlen="); ph8(rlen); term_puts(":");
        uint8_t n = rlen > 12 ? 12 : rlen;
        for (uint8_t k = 0; k < n; k++) { term_putchar(' '); ph8(report[k]); }
        term_putchar('\n');
    }

    uint8_t off = hid_report_data_offset(report, rlen);
    if ((uint16_t)off + 3u > rlen) return -1;
    if (s_dbg) {
        term_puts("HID off=");
        ph8(off);
        term_putchar('\n');
    }

    uint8_t cur[6] = {0,0,0,0,0,0};
    uint8_t cnt = 0;
    for (uint8_t i = (uint8_t)(off + 1u);
         i < rlen && cnt < 6;
         i++)
        if (hid_printable_key(report[i]) || report[i] == 0x39u)
            cur[cnt++] = report[i];

    uint8_t mods = report[off];
    uint8_t active_mods = mods ? mods : s_mods;
    if (mods || cnt == 0)
        s_mods = mods;
    int shift = (active_mods & 0x22u) ? 1 : 0;

    int result = -1;
    for (int i = 0; i < 6 && result == -1; i++) {
        uint8_t kc = cur[i];
        if (!kc) continue;
        int found = 0;
        for (int j = 0; j < 6; j++)
            if (s_prev_keys[j] == kc) { found = 1; break; }
        if (found) continue;

        if (kc == 0x39) { s_caps ^= 1; continue; }  /* Caps Lock */

        if (kc < 64) {
            char base = s_hid_lower[kc];
            int  use_shift = shift;
            if (base >= 'a' && base <= 'z') use_shift ^= s_caps;
            char c = use_shift ? s_hid_upper[kc] : s_hid_lower[kc];
            if (c) result = (int)(unsigned char)c;
        }
    }
    for (int i = 0; i < 6; i++) s_prev_keys[i] = cur[i];
    return result;
}

/* ── GPIO pinmux: set GPP_C8-11 to LPSS UART0 native function ─────────────── */
/*
 * UEFI's _PS3 on the SAM UART leaves C8-C11 muxed as GPIO with GPIOTXDIS set
 * (TX pad forced to hi-Z).  ACPI _PS0 would normally restore UART0 native
 * mode, but we skip ACPI entirely.  So do it manually via the ICL-LP sideband:
 *
 *   SBREG = PCI 0:1F.1 BAR0 (unhide via config+0xE0 bit 8).
 *   Community 4 (GPP_C) lives at PID 0x6A → SBREG + 0x6A0000.
 *   PAD_CFG_DW0 for pin N = COM4 + 0x600 + N*0x10  (stride is 0x10 because
 *   ICL-LP has PINCTRL_FEATURE_DEBOUNCE → 4 DWs per pad).
 *
 * PAD_CFG_DW0 bits of interest:
 *   [13:10] PMODE (1 = UART0 native function 1)
 *   [ 9]    GPIORXDIS (must be 0 to let the pad receive)
 *   [ 8]    GPIOTXDIS (must be 0 to let the pad transmit)
 */
static void sam_gpio_init(uint64_t hhdm) {
    /* Unhide P2SB (bit 8 of PCI 0:1F.1 config offset 0xE0) just long enough
     * to read BAR0, then rehide.                                           */
    uint32_t e0 = pci_read32(0, 0x1Fu, 1u, 0xE0u);
    pci_write32(0, 0x1Fu, 1u, 0xE0u, e0 & ~0x100u);
    uint32_t sbreg = pci_read32(0, 0x1Fu, 1u, 0x10u) & ~0xFu;
    pci_write32(0, 0x1Fu, 1u, 0xE0u, e0);

    if (!sbreg || sbreg == 0xFFFFFFF0u)
        sbreg = 0xFD000000u;   /* ICL-LP fallback */

    uint64_t com4 = (uint64_t)sbreg + 0x6A0000u;
    ioremap_uc(com4 + hhdm, com4, hhdm);

    for (int n = 8; n <= 11; n++) {
        volatile uint32_t *cfg = (volatile uint32_t *)
            (com4 + hhdm + 0x600u + (uint32_t)n * 0x10u);
        uint32_t v = *cfg;
        uint32_t nv = v;
        if (((nv >> 10u) & 0xFu) != 1u)
            nv = (nv & ~(0xFu << 10u)) | (1u << 10u);
        nv &= ~((1u << 8) | (1u << 9));   /* clear GPIOTXDIS + GPIORXDIS */
        if (nv != v) *cfg = nv;
    }
}

/* ── Public: sam_getchar ──────────────────────────────────────────────────── */
/*
 * Keyboard events arrive as DATA_NSQ (type=0x00) frames with:
 *   payload = [0x80][TC=0x15][TID=0x02][sid=0x00][iid=0x01][rqid_lo][rqid_hi][cid=0x00]
 *             + 12 bytes of HID keyboard report (modifiers, reserved, 6× keycodes)
 * DATA_NSQ doesn't need ACK; DATA_SEQ (battery/thermal, not us) does.
 */
int sam_getchar(void) {
    if (!s_ready) return -1;
    if (!ssam_rx_poll()) return -1;

    if (s_frame_type == SSH_ACK || s_frame_type == SSH_NAK) return -1;
    if (s_frame_type == SSH_DATA) ssam_send_ack(s_frame_seq);
    else if (s_frame_type != SSH_DATA_NS) return -1;

    if (s_frame_plen < 9u) return -1;
    if (s_rxpayload[1] != SSH_TC_HID || s_rxpayload[4] != 0x01u) return -1;

    uint8_t rlen = (uint8_t)(s_frame_plen - 8u);
    return hid_decode(&s_rxpayload[8], rlen);
}

/* ── Public: sam_init ─────────────────────────────────────────────────────── */
void sam_init(uint64_t hhdm_offset) {
    uint8_t bus, dev, fn;
    if (!pci_find(SAM_VID, SAM_DID, &bus, &dev, &fn)) {
        term_puts("SAM: UART not found\n");
        return;
    }

    /* PCI D3→D0 via the PM capability */
    uint8_t cap = (uint8_t)(pci_read32(bus, dev, fn, 0x34) & 0xFCu);
    while (cap) {
        uint32_t c = pci_read32(bus, dev, fn, cap);
        if ((c & 0xFFu) == 0x01u) {
            uint32_t pmcs = pci_read32(bus, dev, fn, cap + 4u);
            if (pmcs & 0x03u) {
                pci_write32(bus, dev, fn, cap + 4u, pmcs & ~0x03u);
                udelay(50000);
            }
            break;
        }
        cap = (uint8_t)((c >> 8) & 0xFCu);
    }

    sam_gpio_init(hhdm_offset);

    /* Reprogram the 64-bit BAR (UEFI clears it at ExitBootServices).  Clear
     * memory-decode first so the device doesn't claim the old address mid-
     * write, then program BAR and re-enable MSE.                           */
    uint32_t cmd0 = pci_read32(bus, dev, fn, 0x04);
    pci_write32(bus, dev, fn, 0x04, cmd0 & ~0x02u);
    pci_write32(bus, dev, fn, 0x10, (uint32_t)(SAM_BAR_PHYS & 0xFFFFFFFFu));
    pci_write32(bus, dev, fn, 0x14, (uint32_t)(SAM_BAR_PHYS >> 32));
    pci_write32(bus, dev, fn, 0x04, (cmd0 & ~0x02u) | 0x02u);

    s_base = (void *)(SAM_BAR_PHYS + hhdm_offset);
    ioremap_uc(SAM_BAR_PHYS + hhdm_offset, SAM_BAR_PHYS, hhdm_offset);

    /* Full LPSS init: deassert reset, program 120 MHz × 8/15 = 64 MHz feed,
     * set REMAP_ADDR, clear GEN bit 3 (empirically needed on SL3).         */
    WR32(s_base, 0x204u, 0x00u);
    WR32(s_base, 0x204u, 0x07u);
    udelay(500);
    uint32_t clk = (1u << 0) | (8u << 1) | (15u << 16);
    WR32(s_base, 0x200u, clk);
    WR32(s_base, 0x200u, clk | (1u << 31));
    WR32(s_base, 0x240u, (uint32_t)(SAM_BAR_PHYS & 0xFFFFFFFFu));
    WR32(s_base, 0x244u, (uint32_t)(SAM_BAR_PHYS >> 32));
    WR32(s_base, 0x208u, RD32(s_base, 0x208u) & ~(1u << 3u));

    /* DW APB UART: DLL=1, DLF=0 → 64M/16 = 4 Mbaud; 8N1, FIFO, DTR+RTS+AFCE */
    WR32(s_base, U_IER, 0x00u);
    WR32(s_base, U_LCR, 0x80u);
    WR32(s_base, 0x00u, 0x01u);
    WR32(s_base, 0x04u, 0x00u);
    WR32(s_base, 0xC0u, 0x00u);
    WR32(s_base, U_LCR, 0x03u);
    WR32(s_base, U_FCR, 0xC7u);
    WR32(s_base, U_MCR, 0x23u);
    flush_rx();

    /* Brief settle (~100 ms) so SAM's protocol layer notices DTR/RTS edges
     * and times out any in-flight UEFI frame before we start.              */
    for (uint32_t t = 500000u; t; --t) {
        if (uart_rx_ready()) (void)uart_rx_byte();
    }

    /* SSAM handshake.  Linux's ssam_controller_start() sends these in order:
     * firmware-version query first (SAM seems to require this handshake
     * before it will accept later commands), then D0-entry, display-on,
     * and finally the event subscribe.                                    */
    static const uint8_t sub[] = {
        0x15,   /* target_category = HID */
        0x00,   /* flags (surface_hid uses 0, NOT sequenced) */
        0x15,   /* request_id lo = ssh_tc_to_rqid(HID) = 0x0015 */
        0x00,   /* request_id hi */
        0x01,   /* instance_id = keyboard */
    };

    if (!ssam_cmd_wait(SSH_TC_SAM, SSH_TID_SAM, 0x00u, 0x13u, 0, 0))
        { term_puts("SAM: fw-ver failed\n"); return; }
    if (!ssam_cmd_wait(SSH_TC_SAM, SSH_TID_SAM, 0x00u, SSH_CID_D0_ENTRY, 0, 0))
        { term_puts("SAM: D0-entry failed\n"); return; }
    if (!ssam_cmd_wait(SSH_TC_SAM, SSH_TID_SAM, 0x00u, SSH_CID_DISPLAY_ON, 0, 0))
        { term_puts("SAM: display-on failed\n"); return; }
    if (!ssam_cmd_wait(SSH_TC_REG, SSH_TID_KIP, 0x00u, SSH_CID_REG_EN, sub, 5))
        { term_puts("SAM: subscribe failed\n"); return; }

    s_ready = 1;
    term_puts("SAM: keyboard ready\n");
}
