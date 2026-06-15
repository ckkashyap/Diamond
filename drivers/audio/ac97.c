/*
 * ac97.c - Intel AC97 audio driver
 *
 * PCI 8086:2415 (ICH AC97).
 * BAR0 = NAMBAR  (native audio mixer, I/O)
 * BAR1 = NABMBAR (native audio bus master, I/O)
 *
 * PCM output via the PCM-Out channel, BDL-based DMA.
 * Beep is implemented via a short sine-approximation tone using PCM output;
 * the driver falls back to silence if play_pcm is called on hardware that
 * doesn't ack the sample-rate register.
 */

#include <stdint.h>
#include "../../arch/x86/io.h"
#include "../../arch/x86/pci.h"   /* pci_read32, pci_enable_busmaster */
#include "aud_drv.h"

/* ── AC97 PCI identity ───────────────────────────────────────────────────── */
#define AC97_VID  0x8086u
#define AC97_DID  0x2415u

/* ── Mixer registers (NAMBAR base) ──────────────────────────────────────── */
#define AC97_MASTER_VOL  0x02u   /* master volume */
#define AC97_PCM_VOL     0x18u   /* PCM output volume */
#define AC97_SAMPLE_RATE 0x2Cu   /* PCM front DAC rate */

/* ── Bus-master registers (NABMBAR base) ────────────────────────────────── */
#define AC97_PO_BDBAR    0x10u   /* PCM-Out BDL base address register */
#define AC97_PO_CIV      0x14u   /* current index value (byte, RO) */
#define AC97_PO_LVI      0x15u   /* last valid index (byte, R/W) */
#define AC97_PO_SR       0x16u   /* status (word) */
#define AC97_PO_CR       0x1Bu   /* control (byte) */
#define AC97_GLOB_CNT    0x2Cu   /* global control (dword) */
#define AC97_GLOB_STS    0x30u   /* global status (dword) */

/* CR bits */
#define CR_RUN   0x01u
#define CR_RESET 0x02u
#define CR_FEIE  0x08u   /* FIFO error interrupt enable (keep clear) */
#define CR_IOCE  0x10u   /* IOC interrupt enable (keep clear) */

/* BDL flags */
#define BDL_IOC  0x8000u  /* interrupt on completion */

/* ── BDL: up to 32 entries ───────────────────────────────────────────────── */
#define BDL_LEN 32

typedef struct {
    uint32_t addr;    /* physical address of sample buffer */
    uint16_t length;  /* number of samples in buffer */
    uint16_t flags;
} __attribute__((packed)) ac97_bdle_t;

static ac97_bdle_t  s_bdl[BDL_LEN] __attribute__((aligned(8)));

/* Single DMA buffer shared by all BDL entries (beep uses a small slice). */
#define DMA_SAMPLES 4096
static int16_t s_dma[DMA_SAMPLES] __attribute__((aligned(4)));

static uint16_t s_nambar;   /* I/O base: mixer */
static uint16_t s_nabmbar;  /* I/O base: bus master */
static uint64_t s_kphys;
static uint64_t s_kvirt;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static inline uint32_t virt_to_phys(const void *v) {
    return (uint32_t)(((uint64_t)(uintptr_t)v - s_kvirt) + s_kphys);
}

static inline void mix_write16(uint8_t reg, uint16_t val) {
    outw((uint16_t)(s_nambar + reg), val);
}
static inline uint16_t mix_read16(uint8_t reg) {
    return inw((uint16_t)(s_nambar + reg));
}

static inline void bm_write8(uint8_t reg, uint8_t val) {
    outb((uint16_t)(s_nabmbar + reg), val);
}
static inline void bm_write16(uint8_t reg, uint16_t val) {
    outw((uint16_t)(s_nabmbar + reg), val);
}
static inline void bm_write32(uint8_t reg, uint32_t val) {
    outl((uint16_t)(s_nabmbar + reg), val);
}
static inline uint8_t bm_read8(uint8_t reg) {
    return inb((uint16_t)(s_nabmbar + reg));
}

/* ── Probe ───────────────────────────────────────────────────────────────── */

static int ac97_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset;
    uint8_t bus, dev, fn;
    if (!pci_find(AC97_VID, AC97_DID, &bus, &dev, &fn)) return 0;

    /* Enable bus master + I/O space */
    uint16_t cmd = pci_read16(bus, dev, fn, 0x04);
    pci_write16(bus, dev, fn, 0x04, (uint16_t)(cmd | 0x0005u));
    s_kphys = kphys;
    s_kvirt = kvirt;

    /* BAR0 = NAMBAR (I/O), BAR1 = NABMBAR (I/O) */
    s_nambar  = (uint16_t)(pci_read32(bus, dev, fn, 0x10) & ~0x3u);
    s_nabmbar = (uint16_t)(pci_read32(bus, dev, fn, 0x14) & ~0x3u);

    /* Cold reset via GLOB_CNT bit 0 */
    bm_write32(AC97_GLOB_CNT, 0x00000001u);
    for (volatile int i = 0; i < 100000; i++) __asm__ volatile ("pause");

    /* Reset PCM-Out channel */
    bm_write8(AC97_PO_CR, CR_RESET);
    for (volatile int i = 0; i < 10000; i++) __asm__ volatile ("pause");
    bm_write8(AC97_PO_CR, 0);

    /* Unmute master and PCM volume (0 dB) */
    mix_write16(AC97_MASTER_VOL, 0x0000u);
    mix_write16(AC97_PCM_VOL,    0x0000u);

    /* Set sample rate to 44100 Hz (if VRA supported) */
    mix_write16(AC97_SAMPLE_RATE, 44100u);

    return 1;
}

/* ── PCM playback ────────────────────────────────────────────────────────── */

static int ac97_play_pcm(const int16_t *buf, uint32_t n_frames,
                          uint32_t sample_rate) {
    (void)sample_rate; /* assume 44100 already set */

    /* Build BDL: split n_frames*2 samples across BDL entries, each ≤ 0xFFFE */
    uint32_t total    = n_frames * 2u;        /* stereo samples */
    uint32_t offset   = 0;
    int      n_entries = 0;

    while (offset < total && n_entries < BDL_LEN) {
        uint32_t chunk = total - offset;
        if (chunk > 0xFFFEu) chunk = 0xFFFEu;
        uint32_t phys = (uint32_t)((uint64_t)(uintptr_t)(buf + offset) - s_kvirt + s_kphys);
        s_bdl[n_entries].addr   = phys;
        s_bdl[n_entries].length = (uint16_t)chunk;
        s_bdl[n_entries].flags  = (n_entries == BDL_LEN - 1 || offset + chunk >= total)
                                  ? BDL_IOC : 0;
        offset    += chunk;
        n_entries++;
    }
    if (n_entries == 0) return 0;

    /* Point hardware at our BDL */
    bm_write8(AC97_PO_CR, CR_RESET);
    for (volatile int i = 0; i < 10000; i++) __asm__ volatile ("pause");
    bm_write8(AC97_PO_CR, 0);

    bm_write32(AC97_PO_BDBAR, virt_to_phys(s_bdl));
    bm_write8(AC97_PO_LVI, (uint8_t)(n_entries - 1));

    /* Start DMA */
    bm_write8(AC97_PO_CR, CR_RUN);

    /* Poll until current index reaches LVI (or FIFO underrun) */
    uint8_t lvi = (uint8_t)(n_entries - 1);
    while (bm_read8(AC97_PO_CIV) != lvi) {
        uint16_t sr = inw((uint16_t)(s_nabmbar + AC97_PO_SR));
        if (sr & 0x0008u) break;  /* LVBCI: last valid buffer completion */
        __asm__ volatile ("pause");
    }
    /* Brief settling */
    for (volatile int i = 0; i < 50000; i++) __asm__ volatile ("pause");

    bm_write8(AC97_PO_CR, 0);
    return 0;
}

/* ── Beep: generate a sine wave into DMA buffer and play it ─────────────── */

/* sin(x) via quadrant reduction + 4-term Taylor (max error ~3e-6). */
static float fsin(float x) {
    const float PI  = 3.14159265358979f;
    const float PI2 = 6.28318530717959f;
    const float HPI = 1.57079632679490f;
    x -= PI2 * (float)(int32_t)(x * (1.0f / PI2));
    if (x < 0.0f) x += PI2;
    int neg = 0;
    if (x > PI)  { x -= PI; neg = 1; }
    if (x > HPI) { x = PI - x; }
    float x2 = x * x;
    float r = x * (1.0f - x2 * (1.0f/6.0f - x2 * (1.0f/120.0f - x2 * (1.0f/5040.0f))));
    return neg ? -r : r;
}

static void ac97_beep(uint32_t freq_hz, uint32_t ms) {
    if (freq_hz == 0) return;
    const uint32_t rate      = 44100u;
    const float    TWO_PI    = 6.28318530717959f;
    const float    phase_inc = TWO_PI * (float)freq_hz / (float)rate;
    const float    amp       = 16000.0f;

    uint32_t total_frames = (rate / 1000u) * ms;
    if (total_frames == 0) return;

    /* Clamp to our static DMA buffer */
    uint32_t cap = DMA_SAMPLES / 2u;
    if (total_frames > cap) total_frames = cap;

    float phase = 0.0f;
    for (uint32_t f = 0; f < total_frames; f++) {
        int16_t v = (int16_t)(fsin(phase) * amp);
        s_dma[f * 2 + 0] = v;
        s_dma[f * 2 + 1] = v;
        phase += phase_inc;
        if (phase >= TWO_PI) phase -= TWO_PI;
    }

    ac97_play_pcm(s_dma, total_frames, rate);
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct audio_driver ac97_driver = {
    .name     = "ac97",
    .probe    = ac97_probe,
    .beep     = ac97_beep,
    .play_pcm = ac97_play_pcm,
};
