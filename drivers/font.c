/*
 * font.c - TrueType font rendering via stb_truetype
 *
 * Provides a freestanding environment for stb_truetype:
 * bump allocator, inline math, and manual memset/memcpy.
 */

#include <stdint.h>
#include <stddef.h>

/* ── Bump allocator ──────────────────────────────────────────────────────── */

static uint8_t heap[4 * 1024 * 1024];
static size_t  heap_top = 0;

static void *bump_alloc(size_t n, void *u) {
    (void)u;
    n = (n + 15u) & ~15u;
    if (heap_top + n > sizeof(heap)) return NULL;
    void *p = heap + heap_top;
    heap_top += n;
    return p;
}

static void bump_free(void *p, void *u) { (void)p; (void)u; }

/* ── Minimal string / math helpers ──────────────────────────────────────── */

static void *k_memset(void *dst, int c, size_t n) {
    uint8_t *p = dst;
    while (n--) *p++ = (uint8_t)c;
    return dst;
}

static void *k_memcpy(void *dst, const void *src, size_t n) {
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}

/* ── Math implementations using GCC builtins ─────────────────────────────── */

/* These are exported as real symbols because stb_truetype calls some of them
   directly (not through STBTT_* macros) in its cubic-bezier tessellator. */
double sqrt(double x)  { return __builtin_sqrt(x); }
double fabs(double x)  { return __builtin_fabs(x); }
double floor(double x) { return __builtin_floor(x); }
double ceil(double x)  { return __builtin_ceil(x); }

/* fmod: x - trunc(x/y)*y via integer truncation */
double fmod(double x, double y) {
    if (y == 0.0) return 0.0;
    double q = x / y;
    double t = (q < 0.0) ? __builtin_ceil(q) : __builtin_floor(q);
    return x - t * y;
}

/* pow: only used by SDF code which we never call at runtime;
   provide a stub so it links. */
double pow(double base, double exp) {
    (void)base; (void)exp;
    return 0.0;
}

/* acos / cos: ditto – only in SDF path */
double acos(double x) { (void)x; return 0.0; }
double cos(double x)  { (void)x; return 1.0; }

/* strlen: used by stbtt_FindMatchingFont for platform-string search */
size_t strlen(const char *s) {
    const char *p = s;
    while (*p) p++;
    return (size_t)(p - s);
}

/* Wrappers for the macro overrides */
static double k_sqrt(double x)  { return sqrt(x); }
static double k_fabs(double x)  { return fabs(x); }
static int    k_ifloor(double x) { return (int)floor(x); }
static int    k_iceil(double x)  { return (int)ceil(x); }

/* ── stb_truetype overrides (must be defined before including the header) ── */

#define STBTT_malloc(sz, u)      bump_alloc((sz), (u))
#define STBTT_free(p, u)         bump_free((p), (u))
#define STBTT_memset(a, b, c)    k_memset((a), (b), (c))
#define STBTT_memcpy(a, b, c)    k_memcpy((a), (b), (c))
#define STBTT_sqrt(x)            k_sqrt(x)
#define STBTT_fabs(x)            k_fabs(x)
#define STBTT_ifloor(x)          k_ifloor(x)
#define STBTT_iceil(x)           k_iceil(x)
#define STBTT_fmod(x, y)         fmod((x), (y))
#define STBTT_pow(x, y)          pow((x), (y))
#define STBTT_cos(x)             cos(x)
#define STBTT_acos(x)            acos(x)
#define STBTT_assert(x)          ((void)0)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wtype-limits"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"

#define STB_TRUETYPE_IMPLEMENTATION
#include "../third_party/stb/stb_truetype.h"

#pragma GCC diagnostic pop

/* ── Font atlas ──────────────────────────────────────────────────────────── */

#define ATLAS_W  1024
#define ATLAS_H  1024
#define FIRST_CH 32
#define NUM_CH   96

static uint8_t          font_atlas[ATLAS_W * ATLAS_H];
static stbtt_bakedchar  baked[NUM_CH];
static int              atlas_ready = 0;

/* Metrics populated by font_bake() */
static int s_cell_w = 0;
static int s_cell_h = 0;
static int s_ascent = 0;

static int font_init(const uint8_t *ttf_data, float pixel_height) {
    int r = stbtt_BakeFontBitmap(ttf_data, 0, pixel_height,
                                  font_atlas, ATLAS_W, ATLAS_H,
                                  FIRST_CH, NUM_CH, baked);
    atlas_ready = (r != 0);
    return atlas_ready;
}

/* ── Pixel composition ───────────────────────────────────────────────────── */

static inline uint32_t compose_pixel(uint32_t color, uint8_t alpha,
                                      uint32_t bg,
                                      uint8_t r_shift, uint8_t g_shift, uint8_t b_shift) {
    uint8_t fg_r = (color >> 16) & 0xff;
    uint8_t fg_g = (color >>  8) & 0xff;
    uint8_t fg_b =  color        & 0xff;

    uint8_t bg_r = (bg >> r_shift) & 0xff;
    uint8_t bg_g = (bg >> g_shift) & 0xff;
    uint8_t bg_b = (bg >> b_shift) & 0xff;

    /* Alpha blend: approximate /255 with >>8 (0.4% max error, much faster) */
    uint8_t r = (uint8_t)((fg_r * alpha + bg_r * (255 - alpha) + 128) >> 8);
    uint8_t g = (uint8_t)((fg_g * alpha + bg_g * (255 - alpha) + 128) >> 8);
    uint8_t b = (uint8_t)((fg_b * alpha + bg_b * (255 - alpha) + 128) >> 8);

    return ((uint32_t)r << r_shift) | ((uint32_t)g << g_shift) | ((uint32_t)b << b_shift);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

/*
 * render_text - draw a UTF-8 ASCII string onto a 32bpp framebuffer.
 *
 * font_data   : raw bytes of the .ttf file
 * fb          : pointer to framebuffer memory
 * fb_w/fb_h   : framebuffer dimensions in pixels
 * fb_pitch    : bytes per row
 * r/g/b_shift : bit positions for each colour channel (from limine_framebuffer)
 * text        : null-terminated string (ASCII 32-127)
 * font_size   : desired pixel height
 * color       : 0xRRGGBB foreground colour
 * cx / cy     : centre point on screen (text will be centred here)
 */
void render_text(const uint8_t *font_data,
                 void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
                 uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                 const char *text, float font_size,
                 uint32_t color, int cx, int cy) {

    if (!atlas_ready) {
        if (!font_init(font_data, font_size)) return;
    }

    /* First pass: measure total advance width */
    float adv = 0.0f, dummy_y = 0.0f;
    for (const char *p = text; *p; p++) {
        int ch = (unsigned char)*p;
        if (ch < FIRST_CH || ch >= FIRST_CH + NUM_CH) continue;
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(baked, ATLAS_W, ATLAS_H, ch - FIRST_CH, &adv, &dummy_y, &q, 1);
    }

    /* Baseline: centre the text block on (cx, cy) */
    float x = (float)cx - adv * 0.5f;
    float y = (float)cy;

    /* Second pass: blit each glyph */
    for (const char *p = text; *p; p++) {
        int ch = (unsigned char)*p;
        if (ch < FIRST_CH || ch >= FIRST_CH + NUM_CH) {
            /* Advance a space width for unknown chars */
            x += font_size * 0.5f;
            continue;
        }

        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(baked, ATLAS_W, ATLAS_H, ch - FIRST_CH, &x, &y, &q, 1);

        /* Integer screen and atlas bounds */
        int sx0 = k_ifloor(q.x0), sy0 = k_ifloor(q.y0);
        int sx1 = k_iceil(q.x1),  sy1 = k_iceil(q.y1);
        int aw  = sx1 - sx0;
        int ah  = sy1 - sy0;
        if (aw <= 0 || ah <= 0) continue;

        /* Convert UV → integer atlas pixel origin */
        int ax0 = k_ifloor(q.s0 * ATLAS_W);
        int ay0 = k_ifloor(q.t0 * ATLAS_H);

        /* Blit glyph using integer atlas indexing (1:1 pixel mapping) */
        for (int dy = 0; dy < ah; dy++) {
            int py = sy0 + dy;
            if ((uint64_t)py >= fb_h) continue;
            const uint8_t *arow = &font_atlas[(ay0 + dy) * ATLAS_W + ax0];
            uint32_t *frow = (uint32_t *)((uint8_t *)fb + (uint64_t)py * fb_pitch);
            for (int dx = 0; dx < aw; dx++) {
                int px = sx0 + dx;
                if ((uint64_t)px >= fb_w) continue;
                uint8_t alpha = arow[dx];
                if (alpha == 0) continue;
                frow[px] = compose_pixel(color, alpha, frow[px], r_shift, g_shift, b_shift);
            }
        }
    }
}

/* ── Terminal-oriented API ───────────────────────────────────────────────── */

int font_bake(const uint8_t *font_data, float pixel_height) {
    heap_top = 0;   /* reset bump allocator */
    int r = stbtt_BakeFontBitmap(font_data, 0, pixel_height,
                                  font_atlas, ATLAS_W, ATLAS_H,
                                  FIRST_CH, NUM_CH, baked);
    if (r == 0) { atlas_ready = 0; return 0; }
    atlas_ready = 1;

    /* Query vertical metrics for line height and baseline offset */
    stbtt_fontinfo info;
    stbtt_InitFont(&info, font_data,
                   stbtt_GetFontOffsetForIndex(font_data, 0));
    int ascent, descent, linegap;
    stbtt_GetFontVMetrics(&info, &ascent, &descent, &linegap);
    float scale = stbtt_ScaleForPixelHeight(&info, pixel_height);

    s_ascent = (int)(ascent * scale + 0.5f);
    s_cell_h = (int)((ascent - descent + linegap) * scale + 0.5f);
    if (s_cell_h < (int)pixel_height + 2)
        s_cell_h = (int)pixel_height + 2;

    /* Cell width from the advance of 'M' (all glyphs same advance, monospace) */
    float fx = 0.0f, fy = 0.0f;
    stbtt_aligned_quad q;
    stbtt_GetBakedQuad(baked, ATLAS_W, ATLAS_H, 'M' - FIRST_CH,
                       &fx, &fy, &q, 1);
    s_cell_w = (int)(fx + 0.5f);

    return 1;
}

int font_cell_w(void)  { return s_cell_w; }
int font_cell_h(void)  { return s_cell_h; }
int font_ascent(void)  { return s_ascent; }

int render_char_at(void *fb, uint64_t fb_w, uint64_t fb_h, uint64_t fb_pitch,
                   uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                   char ch, uint32_t fg_color, int px, int py) {
    if (!atlas_ready) return s_cell_w;

    int idx = (unsigned char)ch - FIRST_CH;
    if (idx < 0 || idx >= NUM_CH) return s_cell_w;

    float fx = (float)px;
    float fy = (float)py;
    stbtt_aligned_quad q;
    stbtt_GetBakedQuad(baked, ATLAS_W, ATLAS_H, idx, &fx, &fy, &q, 1);

    int sx0 = k_ifloor(q.x0), sy0 = k_ifloor(q.y0);
    int sx1 = k_iceil(q.x1),  sy1 = k_iceil(q.y1);
    int aw  = sx1 - sx0;
    int ah  = sy1 - sy0;
    if (aw <= 0 || ah <= 0) return s_cell_w;

    int ax0 = k_ifloor(q.s0 * ATLAS_W);
    int ay0 = k_ifloor(q.t0 * ATLAS_H);

    for (int dy = 0; dy < ah; dy++) {
        int ry = sy0 + dy;
        if (ry < 0 || (uint64_t)ry >= fb_h) continue;
        const uint8_t *arow = &font_atlas[(ay0 + dy) * ATLAS_W + ax0];
        uint32_t *frow = (uint32_t *)((uint8_t *)fb + (uint64_t)ry * fb_pitch);
        for (int dx = 0; dx < aw; dx++) {
            int rx = sx0 + dx;
            if (rx < 0 || (uint64_t)rx >= fb_w) continue;
            uint8_t alpha = arow[dx];
            if (alpha == 0) continue;
            frow[rx] = compose_pixel(fg_color, alpha, frow[rx],
                                     r_shift, g_shift, b_shift);
        }
    }
    return s_cell_w;
}

void fb_fill_rect(void *fb, uint64_t fb_pitch,
                  int x, int y, int w, int h,
                  uint8_t r_shift, uint8_t g_shift, uint8_t b_shift,
                  uint32_t color) {
    uint8_t cr = (color >> 16) & 0xff;
    uint8_t cg = (color >>  8) & 0xff;
    uint8_t cb =  color        & 0xff;
    uint32_t pixel = ((uint32_t)cr << r_shift) |
                     ((uint32_t)cg << g_shift) |
                     ((uint32_t)cb << b_shift);
    for (int dy = 0; dy < h; dy++) {
        uint32_t *row = (uint32_t *)((uint8_t *)fb +
                                     (uint64_t)(y + dy) * fb_pitch);
        for (int dx = 0; dx < w; dx++)
            row[x + dx] = pixel;
    }
}
