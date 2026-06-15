/*
 * raytrace.c - SMP Whitted ray-tracer demo
 *
 * Scene: 5 coloured spheres (3 primaries + mirror + gold) on a grey floor,
 *        one point light, Blinn-Phong shading, hard shadows, 2-bounce
 *        reflections.
 *
 * SMP strategy — dynamic scanline stealing:
 *   A shared atomic counter (g_next_y) hands out one scanline at a time.
 *   The BSP dispatches render_worker() to every AP via smp_dispatch(), then
 *   calls render_worker() itself.  After each worker drains the counter the
 *   BSP calls smp_wait() on each AP.  Every CPU ends up with roughly W/N
 *   scanlines, automatically balancing unequal runtimes.
 *
 * The demo command renders the scene twice: once single-core, then with all
 * CPUs, and prints the cycle counts and per-CPU line tallies so the N-core
 * speedup is visible.
 */

#include <stdint.h>
#include "../../drivers/video/video.h"
#include "../../drivers/terminal.h"
#include "../../arch/x86/smp.h"
#include "raytrace.h"

/* ── RDTSC timer ─────────────────────────────────────────────────────────── */

static inline uint64_t rdtsc(void) {
    uint32_t lo, hi;
    __asm__ volatile ("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}

/* ── 3-component float vector ────────────────────────────────────────────── */

typedef struct { float x, y, z; } v3;

static inline v3   v3mk(float x, float y, float z) { return (v3){x,y,z}; }
static inline v3   v3add(v3 a, v3 b)  { return v3mk(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline v3   v3sub(v3 a, v3 b)  { return v3mk(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline v3   v3sc(v3 v, float t){ return v3mk(v.x*t, v.y*t, v.z*t); }
static inline v3   v3mul(v3 a, v3 b)  { return v3mk(a.x*b.x, a.y*b.y, a.z*b.z); }
static inline float v3dot(v3 a, v3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(v3 v)       { return __builtin_sqrtf(v3dot(v,v)); }
static inline v3   v3unit(v3 v)       { return v3sc(v, 1.f/v3len(v)); }
static inline v3   v3neg(v3 v)        { return v3mk(-v.x, -v.y, -v.z); }
static inline v3   v3refl(v3 d, v3 n) { return v3sub(d, v3sc(n, 2.f*v3dot(d,n))); }

static inline v3 v3cross(v3 a, v3 b) {
    return v3mk(a.y*b.z - a.z*b.y,
                a.z*b.x - a.x*b.z,
                a.x*b.y - a.y*b.x);
}

static inline v3 v3clamp1(v3 c) {
    return v3mk(c.x < 0.f ? 0.f : c.x > 1.f ? 1.f : c.x,
                c.y < 0.f ? 0.f : c.y > 1.f ? 1.f : c.y,
                c.z < 0.f ? 0.f : c.z > 1.f ? 1.f : c.z);
}

/* Integer-exponent power via repeated squaring. */
static float fpow(float base, int exp) {
    if (base <= 0.f) return 0.f;
    float r = 1.f;
    while (exp > 0) {
        if (exp & 1) r *= base;
        base *= base;
        exp >>= 1;
    }
    return r;
}

/* ── Scene definition ────────────────────────────────────────────────────── */

typedef struct {
    v3    center;
    float radius;
    v3    diffuse;  /* base colour */
    int   spec_exp; /* Blinn-Phong exponent (0 = no specular highlight) */
    float refl;     /* mirror reflectivity [0,1] */
} Sphere;

#define N_SPHERES  6
#define MAX_DEPTH  2   /* reflection bounces */

static const Sphere S[N_SPHERES] = {
    /* floor — large sphere so the horizon is gently curved */
    { {0.f, -1001.f, 0.f},  1000.f, {0.55f,0.55f,0.55f},  12, 0.08f },
    /* trio: red / green / blue */
    { {-2.3f,  0.f, -4.f},    1.f,  {0.85f,0.18f,0.18f},  48, 0.05f },
    { { 0.0f,  0.f, -4.f},    1.f,  {0.18f,0.78f,0.22f},  48, 0.05f },
    { { 2.3f,  0.f, -4.f},    1.f,  {0.18f,0.28f,0.90f},  48, 0.05f },
    /* mirror sphere — floats above the trio */
    { { 0.0f,  1.5f,-2.5f},  0.90f, {0.88f,0.88f,0.90f}, 200, 0.82f },
    /* gold accent sphere */
    { {-1.3f, -0.4f,-2.2f},  0.60f, {0.92f,0.76f,0.12f},  90, 0.18f },
};

/* Single point light */
static const v3 LIGHT   = {4.f, 8.f,  2.f};
static const v3 L_COLOR = {1.45f, 1.35f, 1.15f};
static const v3 AMB     = {0.07f, 0.07f, 0.10f};

/* Camera */
static const v3 CAM_POS    = {0.f, 1.3f, 4.0f};
static const v3 CAM_TARGET = {0.f, 0.2f, -2.0f};

/* Image plane vectors (set by setup_camera) */
static v3 g_ll;   /* lower-left corner of the image plane */
static v3 g_hrz;  /* horizontal span (right edge - left edge) */
static v3 g_vrt;  /* vertical   span (top  edge - bot  edge) */

static void setup_camera(int w, int h) {
    v3 fw  = v3unit(v3sub(CAM_TARGET, CAM_POS));
    v3 rt  = v3unit(v3cross(fw, v3mk(0.f, 1.f, 0.f)));
    v3 up  = v3cross(rt, fw);              /* already unit */

    /* tan(30°) ≈ 0.57735 gives 60° vertical FOV */
    float half_h = 0.57735f;
    float half_w = half_h * ((float)w / (float)h);

    v3 center = v3add(CAM_POS, fw);        /* image plane centre */
    g_ll  = v3sub(v3sub(center, v3sc(rt, half_w)), v3sc(up, half_h));
    g_hrz = v3sc(rt, 2.f * half_w);
    g_vrt = v3sc(up, 2.f * half_h);
}

/* ── Sky background ──────────────────────────────────────────────────────── */

static v3 sky(v3 rd) {
    /* Horizon-to-zenith gradient: pale sky blue → deep sapphire */
    float t = 0.5f * (rd.y + 1.f);
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    v3 horizon = {0.72f, 0.86f, 1.00f};
    v3 zenith  = {0.10f, 0.28f, 0.72f};
    return v3add(v3sc(horizon, 1.f-t), v3sc(zenith, t));
}

/* ── Ray–sphere intersection ─────────────────────────────────────────────── */

/* Returns t > 0, or -1 on miss.  Ignores hits closer than 1e-4 (self-intersect guard). */
static float hit_sphere(v3 ro, v3 rd, const Sphere *s) {
    v3    oc = v3sub(ro, s->center);
    float b  = v3dot(oc, rd);
    float c  = v3dot(oc, oc) - s->radius * s->radius;
    float d  = b*b - c;
    if (d < 0.f) return -1.f;
    float sq = __builtin_sqrtf(d);
    float t  = -b - sq;
    if (t > 1e-4f) return t;
    t = -b + sq;
    if (t > 1e-4f) return t;
    return -1.f;
}

/* ── Recursive ray tracer ────────────────────────────────────────────────── */

static v3 trace(v3 ro, v3 rd, int depth) {
    /* Find closest hit */
    float t_min = 1e30f;
    int   idx   = -1;
    for (int i = 0; i < N_SPHERES; i++) {
        float t = hit_sphere(ro, rd, &S[i]);
        if (t > 0.f && t < t_min) { t_min = t; idx = i; }
    }
    if (idx < 0) return sky(rd);

    const Sphere *s = &S[idx];
    v3 p = v3add(ro, v3sc(rd, t_min));       /* hit point */
    v3 n = v3unit(v3sub(p, s->center));       /* outward normal */

    /* Light vector and distance */
    v3    lv   = v3sub(LIGHT, p);
    float ld   = v3len(lv);
    v3    ldir = v3sc(lv, 1.f / ld);

    /* Shadow: cast a ray toward the light */
    int shadowed = 0;
    for (int i = 0; i < N_SPHERES; i++) {
        float ts = hit_sphere(p, ldir, &S[i]);
        if (ts > 0.f && ts < ld) { shadowed = 1; break; }
    }

    /* Diffuse (Lambertian) */
    float ndotl = v3dot(n, ldir);
    if (ndotl < 0.f) ndotl = 0.f;

    /* Specular (Blinn-Phong half-vector) */
    v3    half = v3unit(v3add(ldir, v3neg(rd)));
    float nh   = v3dot(n, half);
    if (nh < 0.f) nh = 0.f;
    float sp = (s->spec_exp > 0) ? fpow(nh, s->spec_exp) * 0.65f : 0.f;

    /* Combine */
    v3 col;
    if (shadowed) {
        /* In shadow: ambient only */
        col = v3mul(s->diffuse, AMB);
    } else {
        v3 amb_c  = v3mul(s->diffuse, AMB);
        v3 diff_c = v3mul(s->diffuse, v3sc(L_COLOR, ndotl));
        v3 spec_c = v3sc(L_COLOR, sp);
        col = v3add(amb_c, v3add(diff_c, spec_c));
    }

    /* Reflection */
    if (depth > 0 && s->refl > 0.01f) {
        v3 rdir = v3refl(rd, n);
        v3 rcol = trace(v3add(p, v3sc(n, 2e-4f)), rdir, depth - 1);
        col = v3add(v3sc(col, 1.f - s->refl), v3sc(rcol, s->refl));
    }

    return v3clamp1(col);
}

/* ── Shared render state ─────────────────────────────────────────────────── */

static volatile int g_next_y;   /* atomic scanline counter */
static int          g_width;
static int          g_height;

struct rt_job {
    int cpu_id;
    int lines_done;
};
static struct rt_job g_jobs[SMP_MAX_CPUS];

/* Worker: grabs scanlines atomically until none remain. */
static void render_worker(void *arg) {
    struct rt_job *job = (struct rt_job *)arg;
    const int w = g_width, h = g_height;
    int lines = 0;

    for (;;) {
        int y = __sync_fetch_and_add(&g_next_y, 1);
        if (y >= h) break;

        float vy = 1.f - ((float)y + 0.5f) / (float)h;

        for (int x = 0; x < w; x++) {
            float vx = ((float)x + 0.5f) / (float)w;
            v3 target = v3add(v3add(g_ll, v3sc(g_hrz, vx)), v3sc(g_vrt, vy));
            v3 rd     = v3unit(v3sub(target, CAM_POS));
            v3 col    = trace(CAM_POS, rd, MAX_DEPTH);

            uint32_t r = (uint32_t)(col.x * 255.f);
            uint32_t g = (uint32_t)(col.y * 255.f);
            uint32_t b = (uint32_t)(col.z * 255.f);
            video_pixel(x, y, (r << 16) | (g << 8) | b);
        }
        lines++;
    }
    job->lines_done = lines;
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void print_u32(uint32_t v) {
    char buf[12];
    int i = 11;
    buf[i] = '\0';
    if (v == 0) { buf[--i] = '0'; }
    else { while (v) { buf[--i] = '0' + (char)(v % 10); v /= 10; } }
    term_puts(buf + i);
}

static void print_u64(uint64_t v) {
    char buf[21];
    int i = 20;
    buf[i] = '\0';
    if (v == 0) { buf[--i] = '0'; }
    else { while (v) { buf[--i] = '0' + (char)(v % 10); v /= 10; } }
    term_puts(buf + i);
}

/* Render the scene with the given CPU configuration.
 * ncpus=1 uses only the BSP; ncpus>1 uses BSP + APs. */
static uint64_t do_render(int ncpus) {
    for (int i = 0; i < ncpus; i++) {
        g_jobs[i].cpu_id     = i;
        g_jobs[i].lines_done = 0;
    }

    g_next_y = 0;
    __sync_synchronize();

    uint64_t t0 = rdtsc();

    /* Dispatch to APs first (they start immediately) */
    for (int i = 1; i < ncpus; i++)
        smp_dispatch(i, render_worker, &g_jobs[i]);

    /* BSP works alongside the APs */
    render_worker(&g_jobs[0]);

    /* Wait for all APs to finish */
    for (int i = 1; i < ncpus; i++)
        smp_wait(i);

    return rdtsc() - t0;
}

/* ── Public entry point ──────────────────────────────────────────────────── */

void raytrace_run(int slow) {
    int w = (int)video_width();
    int h = (int)video_height();
    if (w == 0 || h == 0) {
        term_puts("No video output available.\n");
        return;
    }
    g_width  = w;
    g_height = h;
    setup_camera(w, h);

    int ncpus = smp_cpu_count();
    uint64_t cyc1 = 0;

    /* ── Optional single-core pass ───────────────────────────────────────── */
    if (slow) {
        term_puts("Rendering ");
        print_u32((uint32_t)w); term_putchar('x'); print_u32((uint32_t)h);
        term_puts(" — single core... ");
        cyc1 = do_render(1);
        term_puts("done ("); print_u64(cyc1); term_puts(" cycles)\n");
    }

    /* ── All-core pass ───────────────────────────────────────────────────── */
    term_puts("Rendering ");
    print_u32((uint32_t)w); term_putchar('x'); print_u32((uint32_t)h);
    term_puts(" — "); print_u32((uint32_t)ncpus); term_puts(" cores... ");

    uint64_t cycN = do_render(ncpus);
    term_puts("done ("); print_u64(cycN); term_puts(" cycles)\n");

    /* ── Report ──────────────────────────────────────────────────────────── */
    if (slow && cyc1) {
        term_puts("Speedup: ");
        uint32_t speedup10 = (uint32_t)(cyc1 * 10 / (cycN ? cycN : 1));
        print_u32(speedup10 / 10);
        term_putchar('.');
        print_u32(speedup10 % 10);
        term_puts("x  (ideal: ");
        print_u32((uint32_t)ncpus);
        term_puts(".0x)\n");
    }

    term_puts("Work distribution:\n");
    for (int i = 0; i < ncpus; i++) {
        term_puts("  CPU "); print_u32((uint32_t)i);
        term_puts(":  "); print_u32((uint32_t)g_jobs[i].lines_done);
        term_puts(" lines\n");
    }

    term_puts("Press any key to continue...\n");
    (void)term_getchar();
}
