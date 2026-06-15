/*
 * audio.h - Generic audio subsystem public API
 *
 * audio_init() probes all compiled-in drivers; the first one that finds
 * hardware wins.  All subsequent calls dispatch to that driver.
 */
#pragma once
#include <stdint.h>

/*
 * Probe for a supported audio device.
 * kphys / kvirt — kernel address map (from Limine kernel-address response),
 * used by DMA-capable drivers (AC97, HDA) to translate BSS buffer addresses.
 * Returns 1 if a device was found, 0 otherwise.
 */
int audio_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);

/* Name of the active driver, or "none". */
const char *audio_driver_name(void);

/*
 * Produce a square-wave tone at freq_hz for ms milliseconds.
 * Blocking: returns after the tone has played.
 */
void audio_beep(uint32_t freq_hz, uint32_t ms);

/*
 * Play n_frames stereo 16-bit signed PCM samples at sample_rate Hz.
 * Blocking: returns after playback completes.
 * Returns 0 on success, -1 if the active driver does not support PCM.
 */
int audio_play_pcm(const int16_t *buf, uint32_t n_frames, uint32_t sample_rate);
