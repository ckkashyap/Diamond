/*
 * gen_samples.c — Generate piano-like PCM samples for Diamond's piano app.
 *
 * Produces six raw files (samples/piano_A1.raw … samples/piano_A6.raw):
 *   44100 Hz, 16-bit signed little-endian, mono, 3 seconds each.
 *
 * Timbre model (additive synthesis + exponential decay):
 *   fundamental   f   × amplitude 1.00, time-constant 1.50 s
 *   2nd harmonic  2f  × amplitude 0.40, time-constant 1.20 s
 *   3rd harmonic  3f  × amplitude 0.25, time-constant 1.00 s
 *   4th harmonic  4f  × amplitude 0.10, time-constant 0.80 s
 *
 * Uses only <stdio.h>, <math.h>, <stdint.h> — compile with:
 *   gcc -O2 -o tools/gen_samples tools/gen_samples.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define RATE     44100
#define DURATION 3          /* seconds */
#define FRAMES   (RATE * DURATION)
#define PEAK     28000      /* max amplitude (< 32767 to avoid clipping) */

/* anchor notes: name, frequency (Hz) */
static const struct { const char *name; double freq; } notes[] = {
    { "A1",   55.0 },
    { "A2",  110.0 },
    { "A3",  220.0 },
    { "A4",  440.0 },
    { "A5",  880.0 },
    { "A6", 1760.0 },
};
#define N_NOTES (int)(sizeof(notes)/sizeof(notes[0]))

int main(void) {
    static int16_t buf[FRAMES];

    for (int ni = 0; ni < N_NOTES; ni++) {
        double f  = notes[ni].freq;
        double dt = 1.0 / RATE;

        for (int i = 0; i < FRAMES; i++) {
            double t = i * dt;
            double v =
                1.00 * sin(2.0 * M_PI * 1.0 * f * t) * exp(-t / 1.50) +
                0.40 * sin(2.0 * M_PI * 2.0 * f * t) * exp(-t / 1.20) +
                0.25 * sin(2.0 * M_PI * 3.0 * f * t) * exp(-t / 1.00) +
                0.10 * sin(2.0 * M_PI * 4.0 * f * t) * exp(-t / 0.80);
            /* v is in [−1.75, +1.75]; normalise to [−PEAK, +PEAK] */
            v = v / 1.75 * PEAK;
            if (v >  32767.0) v =  32767.0;
            if (v < -32768.0) v = -32768.0;
            buf[i] = (int16_t)v;
        }

        char path[64];
        snprintf(path, sizeof(path), "samples/piano_%s.raw", notes[ni].name);
        FILE *f_out = fopen(path, "wb");
        if (!f_out) { perror(path); return 1; }
        fwrite(buf, sizeof(int16_t), FRAMES, f_out);
        fclose(f_out);
        printf("  %s (%.0f Hz) → %s\n", notes[ni].name, notes[ni].freq, path);
    }
    return 0;
}
