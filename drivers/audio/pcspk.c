/*
 * pcspk.c - PC Speaker audio driver
 *
 * Uses PIT channel 2 (I/O port 0x42) to generate square waves and the
 * speaker gate at I/O port 0x61 to enable/disable the speaker.
 * Always available — no PCI device required.
 *
 * Frequency = 1193182 Hz / divisor
 * PCM playback is not supported (returns -1).
 */

#include <stdint.h>
#include "../../arch/x86/io.h"
#include "aud_drv.h"

/* PIT ports */
#define PIT_CH2   0x42u   /* Channel 2 data port */
#define PIT_CMD   0x43u   /* Mode/command register */
#define PIT_SPK   0x61u   /* PC speaker control (bit 0 = gate, bit 1 = output) */

/* PIT_CMD: channel 2, lobyte/hibyte, mode 3 (square wave) */
#define PIT_CH2_SQUARE 0xB6u

#define PIT_FREQ  1193182u

static void pcspk_on(uint32_t freq_hz) {
    if (freq_hz == 0) return;
    uint32_t div = PIT_FREQ / freq_hz;
    outb(PIT_CMD, PIT_CH2_SQUARE);
    outb(PIT_CH2, (uint8_t)(div & 0xffu));
    outb(PIT_CH2, (uint8_t)((div >> 8) & 0xffu));
    /* Gate speaker to PIT ch2, and enable speaker output */
    outb(PIT_SPK, inb(PIT_SPK) | 0x03u);
}

static void pcspk_off(void) {
    outb(PIT_SPK, inb(PIT_SPK) & ~0x03u);
}

/* PIT channel 0 ports */
#define PIT_CH0   0x40u
#define PIT_CH0_FREE 0x34u  /* ch0, lobyte/hibyte, mode 2 (rate generator) */

/* Latch and read PIT channel 0's current 16-bit countdown value. */
static uint16_t pit0_read(void) {
    outb(PIT_CMD, 0x00u);          /* latch channel 0 */
    uint8_t lo = inb(PIT_CH0);
    uint8_t hi = inb(PIT_CH0);
    return (uint16_t)((uint16_t)hi << 8) | lo;
}

/* Delay for ms milliseconds using the PIT channel 0 counter.
 * PIT runs at 1,193,182 Hz → 1193 ticks per millisecond.
 * Unsigned subtraction handles the 16-bit wraparound correctly. */
static void delay_ms(uint32_t ms) {
    uint32_t ticks_needed = ms * 1193u;
    uint16_t prev = pit0_read();
    uint32_t accumulated = 0;
    while (accumulated < ticks_needed) {
        uint16_t cur = pit0_read();
        accumulated += (uint16_t)(prev - cur);
        prev = cur;
        __asm__ volatile ("pause");
    }
}

/* ── Driver callbacks ────────────────────────────────────────────────────── */

static int pcspk_probe(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    (void)hhdm_offset; (void)kphys; (void)kvirt;
    /* Start PIT channel 0 in free-running mode so delay_ms can use it.
     * Divisor 0 = 65536 → rolls over at ~18.2 Hz; the per-tick rate is
     * still exactly 1,193,182 Hz which is what delay_ms relies on. */
    outb(PIT_CMD, PIT_CH0_FREE);
    outb(PIT_CH0, 0x00u);
    outb(PIT_CH0, 0x00u);
    return 1;
}

static void pcspk_beep(uint32_t freq_hz, uint32_t ms) {
    pcspk_on(freq_hz);
    delay_ms(ms);
    pcspk_off();
}

static int pcspk_play_pcm(const int16_t *buf, uint32_t n_frames,
                           uint32_t sample_rate) {
    (void)buf; (void)n_frames; (void)sample_rate;
    return -1;   /* PC speaker cannot play PCM */
}

/* ── Driver descriptor ───────────────────────────────────────────────────── */

struct audio_driver pcspk_driver = {
    .name     = "pcspk",
    .probe    = pcspk_probe,
    .beep     = pcspk_beep,
    .play_pcm = pcspk_play_pcm,
};
