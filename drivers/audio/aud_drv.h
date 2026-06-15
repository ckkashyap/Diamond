/*
 * aud_drv.h - Internal audio driver descriptor
 */
#pragma once
#include <stdint.h>

struct audio_driver {
    const char *name;
    /* probe: return 1 if hardware found and initialised, 0 otherwise */
    int  (*probe)(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt);
    /* beep: blocking; produce a tone at freq_hz for ms milliseconds */
    void (*beep)(uint32_t freq_hz, uint32_t ms);
    /*
     * play_pcm: play n_frames stereo 16-bit signed samples at sample_rate Hz.
     * Blocking: returns after playback completes (or -1 if unsupported).
     */
    int  (*play_pcm)(const int16_t *buf, uint32_t n_frames, uint32_t sample_rate);
};
