/*
 * audio.c - Audio driver probe loop
 */

#include <stdint.h>
#include "aud_drv.h"
#include "audio.h"

extern struct audio_driver hda_driver;
extern struct audio_driver ac97_driver;
extern struct audio_driver pcspk_driver;

static struct audio_driver * const s_drivers[] = {
    &hda_driver,
    &ac97_driver,
    &pcspk_driver,   /* always available as fallback */
};
#define N_DRIVERS ((int)(sizeof(s_drivers) / sizeof(s_drivers[0])))

static struct audio_driver *s_active = (void *)0;

int audio_init(uint64_t hhdm_offset, uint64_t kphys, uint64_t kvirt) {
    for (int i = 0; i < N_DRIVERS; i++) {
        if (s_drivers[i]->probe(hhdm_offset, kphys, kvirt)) {
            s_active = s_drivers[i];
            return 1;
        }
    }
    return 0;
}

const char *audio_driver_name(void) {
    return s_active ? s_active->name : "none";
}

void audio_beep(uint32_t freq_hz, uint32_t ms) {
    if (s_active) s_active->beep(freq_hz, ms);
}

int audio_play_pcm(const int16_t *buf, uint32_t n_frames, uint32_t sample_rate) {
    if (!s_active || !s_active->play_pcm) return -1;
    return s_active->play_pcm(buf, n_frames, sample_rate);
}
