/*
 * hda.c - Intel High Definition Audio driver
 *
 * Supported PCI IDs:
 *   8086:293E  ICH9 (QEMU q35 machine)
 *   8086:2668  ICH6
 *
 * Uses the Immediate Command Interface (ICI) to discover the codec topology
 * (AFG → DAC → output pin), then sets up a single output stream with a
 * 32-entry BDL for PCM playback.  Beep is a square wave played via PCM.
 *
 * MMIO via BAR0 (64-bit or 32-bit, auto-detected from BAR type bits).
 *
 * HDA verb encoding:
 *   12-bit verb: [31:28]=codec [27:20]=node [19:8]=verb(12) [7:0]=data(8)
 *    4-bit verb: [31:28]=codec [27:20]=node [19:16]=verb(4) [15:0]=data(16)
 */

#include <stdint.h>
#include "../../arch/x86/io.h"
#include "../../arch/x86/pci.h"
#include "../../arch/x86/vm.h"
#include "../../drivers/terminal.h"
#include "../../kernel/alloc.h"
#include "aud_drv.h"

/* ── Diagnostic hex printers ──────────────────────────────────────────────── */
static void dph8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    term_putchar(h[v >> 4]); term_putchar(h[v & 0xf]);
}
static void dph32(uint32_t v) __attribute__((unused));
static void dph32(uint32_t v) {
    dph8((uint8_t)(v >> 24)); dph8((uint8_t)(v >> 16));
    dph8((uint8_t)(v >>  8)); dph8((uint8_t)(v));
}

/* ── PCI IDs ─────────────────────────────────────────────────────────────── */
#define HDA_VID       0x8086u
#define HDA_DID_ICH9  0x293Eu   /* QEMU intel-hda (also real ICH9) */
#define HDA_DID_ICH6  0x2668u   /* QEMU intel-hda (classic) */
#define HDA_DID_ICL   0x34C8u   /* Surface Laptop 3 (Ice Lake-LP cAVS) */

/* ── MMIO register offsets ───────────────────────────────────────────────── */
#define HDA_GCAP      0x00u  /* global capabilities (16-bit) */
#define HDA_GCTL      0x08u  /* global control (32-bit) */
#define HDA_STATESTS  0x0Eu  /* state change status (16-bit) */
#define HDA_INTCTL    0x20u  /* interrupt control (32-bit) */
#define HDA_ICOI      0x60u  /* immediate command output interface (32-bit) */
#define HDA_ICII      0x64u  /* immediate command input  interface (32-bit) */
#define HDA_ICIS      0x68u  /* immediate command status (16-bit) */

/* Stream descriptor register offsets (relative to stream base) */
#define SD_CTL    0x00u  /* control/status (32-bit) */
#define SD_LPIB   0x04u  /* link position in buffer (32-bit) */
#define SD_CBL    0x08u  /* cyclic buffer length (32-bit) */
#define SD_LVI    0x0Cu  /* last valid BDL index (16-bit) */
#define SD_FMT    0x12u  /* stream format (16-bit) */
#define SD_BDPL   0x18u  /* BDL lower base address (32-bit) */
#define SD_BDPU   0x1Cu  /* BDL upper base address (32-bit) */

/* GCTL bits */
#define GCTL_CRST   (1u << 0)

/* ICIS bits */
#define ICIS_ICB    (1u << 0)   /* immediate command busy */
#define ICIS_IRV    (1u << 1)   /* immediate result valid */

/* SD_CTL bits */
#define SDCTL_SRST  (1u << 0)   /* stream reset */
#define SDCTL_RUN   (1u << 1)   /* stream run */
#define SDCTL_IOC   (1u << 2)   /* interrupt on completion enable */

/* ── HDA verb encoding ───────────────────────────────────────────────────── */
/* 12-bit verb (most GET/SET commands): verb occupies bits [19:8], data [7:0] */
#define HDA_VERB(codec, node, verb12, data8) \
    ((uint32_t)((uint32_t)(codec) << 28) | \
     ((uint32_t)(node)  << 20) | \
     ((uint32_t)(verb12) << 8) | \
     ((uint32_t)(data8)))

/* 4-bit verb (SET_FORMAT, SET_AMP_GAIN): verb at [19:16], data at [15:0] */
#define HDA_VERB4(codec, node, verb4, data16) \
    ((uint32_t)((uint32_t)(codec) << 28) | \
     ((uint32_t)(node)  << 20) | \
     ((uint32_t)(verb4) << 16) | \
     ((uint32_t)(data16)))

/* ── Common verb codes ───────────────────────────────────────────────────── */
#define VERB_GET_PARAM      0xF00u  /* 12-bit */
#define VERB_SET_CONN_SEL   0x701u  /* 12-bit */
#define VERB_SET_POWER      0x705u  /* 12-bit */
#define VERB_SET_STREAM_CH  0x706u  /* 12-bit; data = (tag<<4)|chan */
#define VERB_SET_PIN_CTRL   0x707u  /* 12-bit */
#define VERB_SET_EAPDBTL    0x70Cu  /* 12-bit */
#define VERB_SET_FORMAT     0x02u   /* 4-bit  */
#define VERB_SET_AMP_GAIN   0x03u   /* 4-bit  */

/* ── GET_PARAM parameter IDs ─────────────────────────────────────────────── */
#define PARAM_NODE_COUNT    0x04u
#define PARAM_FUNC_TYPE     0x05u
#define PARAM_WIDGET_CAP    0x09u
#define PARAM_CONN_LIST_LEN 0x0Eu   /* bits[6:0] = #entries, bit7 = long form */

/* 12-bit verbs */
#define VERB_GET_CONN_LIST  0xF02u  /* data=starting index, returns 4 NIDs */
#define VERB_GET_PIN_DFLT   0xF1Cu  /* get default-config for pin */
#define VERB_GET_CONN_SEL   0xF01u  /* get currently-selected conn-list idx */

/* Widget capability: bits [23:20] = type */
#define WTYPE_DAC     0x0u
#define WTYPE_ADC     0x1u
#define WTYPE_MIXER   0x2u
#define WTYPE_SELECT  0x3u
#define WTYPE_PIN     0x4u
#define WTYPE_POWER   0x5u
#define WTYPE_VOL     0x6u
#define WTYPE_BEEP    0x7u

/* Pin default config bits (from VERB_GET_PIN_DFLT response) */
#define DFLT_PORTCONN_SHIFT  30        /* 0=jack, 1=none, 2=fixed, 3=both   */
#define DFLT_DEVICE_SHIFT    20        /* 0=LineOut, 1=Speaker, 2=Headphone */

/* Pin capability bits (from PARAM_PIN_CAP = 0x0C response) */
#define PINCAP_OUTPUT   (1u << 4)

/* ── HDA stream format word ──────────────────────────────────────────────── */
/* [14]=BASE(0=48k,1=44.1k) [13:11]=MULT [10:8]=DIV [6:4]=BITS [3:0]=CHAN-1 */
#define HDA_FMT_44100_16_STEREO  0x4011u  /* 44.1kHz, 16-bit, 2ch */
#define HDA_FMT_48000_16_STEREO  0x0011u  /* 48kHz,   16-bit, 2ch */

/* Amp gain: set=1, out=1, L=1, R=1, idx=0, mute=0, gain=0x7F (max) */
#define AMP_OUT_UNMUTE_MAX  0xB07Fu

/* Pin control: enable output, enable headphone */
#define PIN_CTRL_OUT  0xC0u

/* ── BDL ─────────────────────────────────────────────────────────────────── */
#define BDL_ENTRIES 32u

typedef struct {
    uint64_t addr;
    uint32_t len;
    uint32_t ioc;
} __attribute__((packed, aligned(128))) hda_bdle_t;

static hda_bdle_t s_bdl[BDL_ENTRIES] __attribute__((aligned(128)));

/* Dynamic DMA buffer sized for up to MAX_BEEP_SECS of uninterrupted playback.
 * One big buffer means one hda_play_pcm call per beep → no chunk boundaries
 * where the stream gets stopped/reset (which both clicks and lengthens the
 * beep by the reset latency).  Allocated at probe time from the heap.      */
#define MAX_BEEP_SECS 10u
static int16_t  *s_dma;
static uint32_t  s_dma_frames;   /* capacity in stereo frames */

/* ── Driver state ────────────────────────────────────────────────────────── */
static volatile uint8_t *s_base;   /* MMIO base (virtual) */
static uint32_t          s_sd_off; /* output stream descriptor byte offset */
static uint8_t           s_codec;  /* first codec address found */
static uint8_t           s_afg;    /* audio function group NID */
static uint8_t           s_dac;    /* output DAC NID */
static uint8_t           s_pin;    /* output pin NID */
static uint64_t          s_kphys;
static uint64_t          s_kvirt;
static uint64_t          s_hhdm;   /* HHDM offset — needed to virt→phys the heap DMA buffer */

/* ── MMIO accessors ──────────────────────────────────────────────────────── */

static inline uint32_t reg_r32(uint32_t off) {
    return *(volatile uint32_t *)(s_base + off);
}
static inline uint16_t reg_r16(uint32_t off) {
    return *(volatile uint16_t *)(s_base + off);
}
static inline void reg_w32(uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(s_base + off) = val;
}
static inline void reg_w16(uint32_t off, uint16_t val) {
    *(volatile uint16_t *)(s_base + off) = val;
}

static inline uint32_t sd_r32(uint32_t reg) {
    return reg_r32(s_sd_off + reg);
}
static inline void sd_w32(uint32_t reg, uint32_t val) {
    reg_w32(s_sd_off + reg, val);
}
static inline void sd_w16(uint32_t reg, uint16_t val) {
    reg_w16(s_sd_off + reg, val);
}

/* ── Immediate Command Interface ─────────────────────────────────────────── */

static uint32_t hda_cmd(uint32_t verb) {
    /* Clear IRV, wait for ICB=0 */
    for (int i = 0; i < 500000; i++) {
        if (!(reg_r16(HDA_ICIS) & ICIS_ICB)) break;
        __asm__ volatile ("pause");
    }
    reg_w16(HDA_ICIS, ICIS_IRV);  /* clear IRV */
    reg_w32(HDA_ICOI, verb);
    /* Set ICB to trigger send */
    reg_w16(HDA_ICIS, (uint16_t)(reg_r16(HDA_ICIS) | ICIS_ICB));

    /* Wait for IRV=1 (response ready) */
    for (int i = 0; i < 500000; i++) {
        if (reg_r16(HDA_ICIS) & ICIS_IRV) break;
        __asm__ volatile ("pause");
    }
    return reg_r32(HDA_ICII);
}

/* ── Topology walk ───────────────────────────────────────────────────────── */

/* s_path: NIDs of mixers/selectors between the pin and the DAC (inclusive of
 * none if pin connects directly to DAC).  s_path_len = how many to unmute.  */
static uint8_t s_path[8];
static uint8_t s_path_len;

/* Return connection-list entry `idx` of widget `nid` (short form only). */
static uint8_t conn_entry(uint8_t nid, uint8_t idx) {
    uint32_t r = hda_cmd(HDA_VERB(s_codec, nid, VERB_GET_CONN_LIST,
                                   (uint8_t)(idx & 0xFCu)));
    return (uint8_t)(r >> ((idx & 3) * 8));
}

/* Walk back from a pin through mixers / selectors until a DAC is reached.
 * Records each intermediate node in s_path and returns the DAC NID, or 0. */
static uint8_t walk_to_dac(uint8_t pin_nid) {
    uint8_t cur = pin_nid;
    s_path_len = 0;

    for (int hop = 0; hop < 6; hop++) {
        uint32_t clen = hda_cmd(HDA_VERB(s_codec, cur, VERB_GET_PARAM,
                                          PARAM_CONN_LIST_LEN));
        uint8_t n = (uint8_t)(clen & 0x7Fu);
        if (n == 0) return 0;

        /* First connection-list entry is the default input to this widget */
        uint8_t next = conn_entry(cur, 0);
        if (next == 0) return 0;

        uint32_t cap  = hda_cmd(HDA_VERB(s_codec, next, VERB_GET_PARAM,
                                          PARAM_WIDGET_CAP));
        uint8_t  type = (uint8_t)((cap >> 20) & 0xFu);

        if (type == WTYPE_DAC) return next;

        if (type == WTYPE_MIXER || type == WTYPE_SELECT) {
            if (s_path_len < sizeof(s_path))
                s_path[s_path_len++] = next;
            cur = next;
            continue;
        }
        return 0;   /* hit something we can't walk through */
    }
    return 0;       /* too many hops */
}

/* Returns 1 if AFG, DAC, and output pin were all found. */
static int hda_find_topology(void) {
    /* Root node (node 0): query function group count */
    uint32_t nc = hda_cmd(HDA_VERB(s_codec, 0, VERB_GET_PARAM, PARAM_NODE_COUNT));
    uint8_t  fg_total = (uint8_t)(nc & 0xFFu);
    uint8_t  fg_start = (uint8_t)((nc >> 16) & 0xFFu);
    if (fg_total == 0) return 0;

    /* Find AFG (function type byte 0x01) */
    s_afg = 0;
    for (uint8_t n = fg_start; n < fg_start + fg_total; n++) {
        uint32_t ft = hda_cmd(HDA_VERB(s_codec, n, VERB_GET_PARAM, PARAM_FUNC_TYPE));
        if ((ft & 0xFFu) == 0x01u) { s_afg = n; break; }
    }
    if (s_afg == 0) return 0;

    /* Power up AFG early so widgets respond */
    hda_cmd(HDA_VERB(s_codec, s_afg, VERB_SET_POWER, 0x00));
    for (volatile int i = 0; i < 100000; i++) __asm__ volatile ("pause");

    /* Query widgets within AFG */
    uint32_t wc = hda_cmd(HDA_VERB(s_codec, s_afg, VERB_GET_PARAM, PARAM_NODE_COUNT));
    uint8_t  w_total = (uint8_t)(wc & 0xFFu);
    uint8_t  w_start = (uint8_t)((wc >> 16) & 0xFFu);
    if (w_total == 0) return 0;

    /* Pick the best output pin:
     *   rank 3 = fixed-function speaker (internal laptop speaker)
     *   rank 2 = line out
     *   rank 1 = headphone jack
     *   rank 0 = anything else output-capable
     * In parallel pick the first DAC as a fallback.                      */
    uint8_t best_pin = 0, best_rank = 0, any_dac = 0;
    term_puts("HDA: widgets: ");
    for (uint8_t n = w_start; n < w_start + w_total; n++) {
        uint32_t cap  = hda_cmd(HDA_VERB(s_codec, n, VERB_GET_PARAM, PARAM_WIDGET_CAP));
        uint8_t  type = (uint8_t)((cap >> 20) & 0xFu);

        if (type == WTYPE_DAC && any_dac == 0) any_dac = n;

        if (type == WTYPE_PIN) {
            uint32_t df = hda_cmd(HDA_VERB(s_codec, n, VERB_GET_PIN_DFLT, 0));
            uint8_t  pc = (uint8_t)((df >> DFLT_PORTCONN_SHIFT) & 3u);
            uint8_t  dd = (uint8_t)((df >> DFLT_DEVICE_SHIFT) & 0xFu);
            if (pc == 1) continue;   /* no physical connection */

            uint8_t rank =
                (dd == 1) ? 3 :      /* speaker   */
                (dd == 0) ? 2 :      /* line out  */
                (dd == 2) ? 1 : 0;   /* headphone : other */

            term_putchar('p'); dph8(n); term_putchar('/'); dph8(dd); term_putchar(' ');

            if (rank > best_rank) { best_rank = rank; best_pin = n; }
        } else if (type == WTYPE_DAC) {
            term_putchar('D'); dph8(n); term_putchar(' ');
        }
    }
    term_putchar('\n');

    if (best_pin == 0) { s_dac = any_dac; s_pin = (uint8_t)(w_start + 1u); s_path_len = 0; return any_dac != 0; }
    s_pin = best_pin;

    /* Trace back from pin to a DAC */
    s_dac = walk_to_dac(best_pin);
    if (s_dac == 0) s_dac = any_dac;

    term_puts("HDA: picked pin=0x"); dph8(s_pin);
    term_puts(" DAC=0x"); dph8(s_dac);
    term_puts(" path=[");
    for (int i = 0; i < s_path_len; i++) { dph8(s_path[i]); term_putchar(' '); }
    term_puts("]\n");

    return s_dac != 0;
}

/* ── Probe ───────────────────────────────────────────────────────────────── */

static int hda_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    uint8_t bus, dev, fn;
    if (!pci_find(HDA_VID, HDA_DID_ICL,  &bus, &dev, &fn))
        if (!pci_find(HDA_VID, HDA_DID_ICH9, &bus, &dev, &fn))
            if (!pci_find(HDA_VID, HDA_DID_ICH6, &bus, &dev, &fn))
                return 0;

    s_kphys = kphys;
    s_kvirt = kvirt;
    s_hhdm  = hhdm_offset;

    /* D3→D0 via the PCI PM capability (UEFI may have left us in D3hot). */
    uint8_t cap = (uint8_t)(pci_read32(bus, dev, fn, 0x34) & 0xFCu);
    while (cap) {
        uint32_t c = pci_read32(bus, dev, fn, cap);
        if ((c & 0xFFu) == 0x01u) {
            uint32_t pmcs = pci_read32(bus, dev, fn, cap + 4u);
            if (pmcs & 0x03u) {
                pci_write32(bus, dev, fn, cap + 4u, pmcs & ~0x03u);
                /* Spec requires ≥ 10 ms for D3→D0 to settle.  At ~1 cycle
                 * per pause on a ~2 GHz core, 20 M pauses ≈ 10 ms.        */
                for (volatile int i = 0; i < 20000000; i++) __asm__ volatile ("pause");
            }
            break;
        }
        cap = (uint8_t)((c >> 8) & 0xFCu);
    }

    /* Enable bus master + MMIO space */
    uint16_t pcmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, (uint16_t)(pcmd | 0x0006u));

    /* BAR0: detect 32-bit vs 64-bit MMIO from type bits [2:1] */
    uint32_t bar0_lo = pci_read32(bus, dev, fn, 0x10);
    uint64_t mmio_phys;
    if ((bar0_lo & 0x6u) == 0x4u) {
        /* 64-bit BAR */
        uint32_t bar0_hi = pci_read32(bus, dev, fn, 0x14);
        mmio_phys = ((uint64_t)bar0_hi << 32) | (bar0_lo & ~0xFu);
    } else {
        mmio_phys = bar0_lo & ~0xFu;
    }
    if (mmio_phys == 0) return 0;

    /* Map MMIO via HHDM, forced to UC.  HDA register reads through the WB
     * HHDM return stale values on ICL-LP; UC mapping is required on real
     * hardware.  On QEMU this is harmless — a UC page is UC whether the
     * underlying BAR is emulated or real.                                 */
    uint64_t mmio_va = mmio_phys + hhdm_offset;
    ioremap_uc(mmio_va,            mmio_phys,            hhdm_offset);
    /* HDA regs span up to 0x2080 on cAVS (stream descriptors + PP); map
     * enough to cover everything we touch.                                */
    ioremap_uc(mmio_va + 0x1000u,  mmio_phys + 0x1000u,  hhdm_offset);
    ioremap_uc(mmio_va + 0x2000u,  mmio_phys + 0x2000u,  hhdm_offset);
    ioremap_uc(mmio_va + 0x3000u,  mmio_phys + 0x3000u,  hhdm_offset);
    s_base = (volatile uint8_t *)(uintptr_t)mmio_va;

    /* ── Controller reset ────────────────────────────────────────────────── */
    reg_w32(HDA_GCTL, 0);
    /* Wait for reset to take (CRST=0 means reset in progress) */
    for (int i = 0; i < 1000000; i++) {
        if (!(reg_r32(HDA_GCTL) & GCTL_CRST)) break;
        __asm__ volatile ("pause");
    }
    /* Bring out of reset */
    reg_w32(HDA_GCTL, GCTL_CRST);
    /* Wait for CRST=1 (controller ready) */
    for (int i = 0; i < 1000000; i++) {
        if (reg_r32(HDA_GCTL) & GCTL_CRST) break;
        __asm__ volatile ("pause");
    }
    /* Codec enumeration delay (spec requires ≥ 521 µs after CRST) */
    for (volatile int i = 0; i < 500000; i++) __asm__ volatile ("pause");

    /* Disable all interrupts */
    reg_w32(HDA_INTCTL, 0);

    /* Find first codec that responded after reset */
    uint16_t statests = reg_r16(HDA_STATESTS);
    s_codec = 0xFF;
    for (int i = 0; i < 15; i++) {
        if (statests & (1u << i)) { s_codec = (uint8_t)i; break; }
    }
    if (s_codec == 0xFF) return 0;  /* no codecs */

    /* Clear STATESTS */
    reg_w16(HDA_STATESTS, statests);

    /* Determine first output stream offset: 0x80 + ISS*0x20 */
    /* GCAP[15:12]=OSS, [11:8]=ISS */
    uint16_t gcap = reg_r16(HDA_GCAP);
    uint8_t  iss  = (uint8_t)((gcap >> 8) & 0xFu);
    s_sd_off = 0x80u + (uint32_t)iss * 0x20u;

    /* Walk codec topology */
    if (!hda_find_topology()) return 0;

    /* ── Configure codec for output ──────────────────────────────────────── */

    /* Power up every node on the path: DAC, intermediate mixers/selectors, pin. */
    hda_cmd(HDA_VERB(s_codec, s_dac, VERB_SET_POWER, 0x00));
    for (int i = 0; i < s_path_len; i++)
        hda_cmd(HDA_VERB(s_codec, s_path[i], VERB_SET_POWER, 0x00));
    hda_cmd(HDA_VERB(s_codec, s_pin, VERB_SET_POWER, 0x00));
    for (volatile int i = 0; i < 100000; i++) __asm__ volatile ("pause");

    /* Bind DAC to output stream tag 1, channel 0 */
    hda_cmd(HDA_VERB(s_codec, s_dac, VERB_SET_STREAM_CH, 0x10u));

    /* Set DAC output format: 44100 Hz, 16-bit, stereo */
    hda_cmd(HDA_VERB4(s_codec, s_dac, VERB_SET_FORMAT, HDA_FMT_44100_16_STEREO));

    /* Unmute / max out the output amp on every node from DAC to pin.  The
     * 0xF07F pattern hits both the output amp and input index 0 in one
     * shot, which covers mixers whose output is downstream and whose
     * relevant input is the previous node in our path.                   */
    hda_cmd(HDA_VERB4(s_codec, s_dac, VERB_SET_AMP_GAIN, AMP_OUT_UNMUTE_MAX));
    for (int i = 0; i < s_path_len; i++) {
        /* If it's a selector, ensure connection 0 is selected */
        hda_cmd(HDA_VERB(s_codec, s_path[i], VERB_SET_CONN_SEL, 0));
        hda_cmd(HDA_VERB4(s_codec, s_path[i], VERB_SET_AMP_GAIN, 0xF07Fu));
    }

    /* Pin: enable output (bit 6) + headphone-amp (bit 7) */
    hda_cmd(HDA_VERB(s_codec, s_pin, VERB_SET_PIN_CTRL, PIN_CTRL_OUT));
    /* Enable EAPD — laptops need this to power up the internal amp */
    hda_cmd(HDA_VERB(s_codec, s_pin, VERB_SET_EAPDBTL, 0x02u));
    /* Unmute pin amp (both output and input 0) */
    hda_cmd(HDA_VERB4(s_codec, s_pin, VERB_SET_AMP_GAIN, 0xF07Fu));

    /* Allocate one big DMA buffer (enough for MAX_BEEP_SECS of 44.1 kHz
     * stereo 16-bit) and align to 128 bytes for the HDA BDL requirement. */
    s_dma_frames = MAX_BEEP_SECS * 44100u;
    uint8_t *raw = (uint8_t *)kmalloc((uint64_t)s_dma_frames * 4u + 128u);
    if (!raw) return 0;
    s_dma = (int16_t *)(((uintptr_t)raw + 127u) & ~(uintptr_t)127);

    return 1;
}

/* ── PCM playback ────────────────────────────────────────────────────────── */

static int hda_play_pcm(const int16_t *buf, uint32_t n_frames,
                         uint32_t sample_rate) {
    (void)sample_rate;  /* fixed at 44100 Hz */

    /* Reset output stream */
    sd_w32(SD_CTL, sd_r32(SD_CTL) | SDCTL_SRST);
    for (volatile int i = 0; i < 50000; i++) __asm__ volatile ("pause");
    /* Wait for SRST to clear */
    for (int i = 0; i < 500000; i++) {
        sd_w32(SD_CTL, sd_r32(SD_CTL) & ~SDCTL_SRST);
        if (!(sd_r32(SD_CTL) & SDCTL_SRST)) break;
        __asm__ volatile ("pause");
    }

    uint32_t byte_len = n_frames * 4u;  /* 2 channels × 2 bytes */
    /* s_dma is heap-allocated (kmalloc, mapped via HHDM), so virt→phys is
     * simply v - hhdm_offset.  Earlier s_dma was a static array and we used
     * (v - kvirt) + kphys, but the heap isn't in the kernel image mapping. */
    uint64_t buf_phys = (uint64_t)(uintptr_t)buf - s_hhdm;

    /* Build BDL: split into ≤ 1 MiB chunks */
    uint32_t n_ent    = 0;
    uint32_t rem      = byte_len;
    uint64_t off      = 0;
    while (rem > 0 && n_ent < BDL_ENTRIES) {
        uint32_t chunk = rem < 0x100000u ? rem : 0x100000u;
        s_bdl[n_ent].addr = buf_phys + off;
        s_bdl[n_ent].len  = chunk;
        s_bdl[n_ent].ioc  = 1u;
        off  += chunk;
        rem  -= chunk;
        n_ent++;
    }
    if (n_ent == 0) return 0;

    uint64_t bdl_phys = ((uint64_t)(uintptr_t)s_bdl - s_kvirt) + s_kphys;

    /* Program stream descriptor */
    sd_w32(SD_BDPL, (uint32_t)(bdl_phys));
    sd_w32(SD_BDPU, (uint32_t)(bdl_phys >> 32));
    sd_w32(SD_CBL,  byte_len);
    sd_w16(SD_LVI,  (uint16_t)(n_ent - 1u));
    sd_w16(SD_FMT,  HDA_FMT_44100_16_STEREO);
    /* Set stream tag 1 in bits [23:20] */
    uint32_t ctl = sd_r32(SD_CTL);
    ctl = (ctl & ~(0xFu << 20)) | (1u << 20);
    sd_w32(SD_CTL, ctl);

    /* Re-bind DAC to stream tag 1 */
    hda_cmd(HDA_VERB(s_codec, s_dac, VERB_SET_STREAM_CH, 0x10u));

    /* Start DMA */
    sd_w32(SD_CTL, sd_r32(SD_CTL) | SDCTL_RUN);

    /* Poll LPIB until playback reaches end of buffer.  We can't use a fixed
     * iteration cap because the buffer duration varies from tens of ms to
     * seconds.  Instead, watch for LPIB stalling: if it hasn't advanced in
     * a while, DMA is stuck — bail.                                       */
    uint32_t prev_lpib = 0;
    uint32_t stall     = 0;
    for (;;) {
        uint32_t cur = sd_r32(SD_LPIB);
        if (cur >= byte_len - 512u) break;
        if (cur == prev_lpib) {
            if (++stall > 2000000u) break;   /* DMA wedged */
        } else {
            prev_lpib = cur;
            stall     = 0;
        }
        __asm__ volatile ("pause");
    }
    /* Brief tail silence */
    for (volatile int i = 0; i < 200000; i++) __asm__ volatile ("pause");

    /* Stop stream */
    sd_w32(SD_CTL, sd_r32(SD_CTL) & ~SDCTL_RUN);
    return 0;
}

/* ── Beep ────────────────────────────────────────────────────────────────── */

/* sin(x) in [-π, π] via quadrant reduction + 4-term Taylor on [0, π/2].
 * Max error ≈ 3e-6 — inaudible.  Avoids a libm dependency. */
static float fsin(float x) {
    const float PI  = 3.14159265358979f;
    const float PI2 = 6.28318530717959f;
    const float HPI = 1.57079632679490f;

    /* Reduce to [0, 2π) */
    x -= PI2 * (float)(int32_t)(x * (1.0f / PI2));
    if (x < 0.0f) x += PI2;

    /* Use symmetry to reduce to [0, π/2] */
    int neg = 0;
    if (x > PI)  { x -= PI; neg = 1; }
    if (x > HPI) { x = PI - x; }

    /* Taylor: sin(x) ≈ x − x³/6 + x⁵/120 − x⁷/5040 */
    float x2 = x * x;
    float r = x * (1.0f - x2 * (1.0f/6.0f - x2 * (1.0f/120.0f - x2 * (1.0f/5040.0f))));
    return neg ? -r : r;
}

static void hda_beep(uint32_t freq_hz, uint32_t ms) {
    if (freq_hz == 0 || !s_dma) return;

    const uint32_t rate      = 44100u;
    const float    TWO_PI    = 6.28318530717959f;
    const float    phase_inc = TWO_PI * (float)freq_hz / (float)rate;
    const float    amp       = 16000.0f;

    uint32_t total_frames = (rate / 1000u) * ms;
    if (total_frames > s_dma_frames) total_frames = s_dma_frames;  /* clamp */

    /* Fill the whole buffer with the sine wave in one pass */
    float phase = 0.0f;
    for (uint32_t f = 0; f < total_frames; f++) {
        int16_t v = (int16_t)(fsin(phase) * amp);
        s_dma[f * 2u + 0u] = v;
        s_dma[f * 2u + 1u] = v;
        phase += phase_inc;
        if (phase >= TWO_PI) phase -= TWO_PI;
    }

    /* Single DMA run — no chunk stitching, no reset gaps */
    hda_play_pcm(s_dma, total_frames, rate);
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct audio_driver hda_driver = {
    .name     = "hda",
    .probe    = hda_probe,
    .beep     = hda_beep,
    .play_pcm = hda_play_pcm,
};
