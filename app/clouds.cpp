/*
 * clouds.cpp - the cloud-deck background.
 *
 * The arithmetic is tools/clouds.c's, unchanged: the same value-noise fBm tile, the
 * same colour ladder, the same floor mapper. What differs is what surrounds it.
 *
 *   - NB0 is 7 rather than 8, so the noise tile is 128 square and the mip chain
 *     is 21 KB rather than 85. Two framebuffers and Jet's heap leave no room for
 *     the larger one, and the author of the original names this as the reduction
 *     to make.
 *
 *   - There is no PNG writer and no benchmark, and the flight path is not
 *     scripted: the heading comes from the caller, which is holding a stick.
 *
 *   - A frame is split into begin_frame() and render_band(), so the two cores can
 *     fill disjoint halves. Everything begin_frame() computes is read-only while
 *     the bands run, and a row depends on nothing but that state and its own y.
 *
 * It writes every pixel it is given, so it takes the place of clearing the
 * framebuffer rather than being drawn on top of a cleared one.
 */
#include <cmath>
#include <cstring>

#include "clouds.hpp"

/* ----------------------------- tunables ----------------------------- */

#define W          320
#define H          200

#define NB0        7            /* base noise tile is (1<<NB0) square */
#define NMIPS      4            /* mip chain kills the far-field shimmer */

#define FOGSTEPS   128          /* fog / elevation bands in the ladder */
#define DENSTEPS   64           /* density levels in the ladder */

#define FOCAL      220.0f       /* pinhole focal length in pixels (~72 deg) */
#define SPEED      200.0f       /* world units per second */

#define DECK_LO    120.0f       /* underside of the deck we fly under */
#define DECK_HI    215.0f       /* second sheet: gives the deck thickness */
#define DECK_DN   -700.0f       /* broken deck far below */

#define TEX_LO     0.40f        /* texels per world unit, per sheet */
#define TEX_HI     0.270f
#define TEX_DN     0.180f

#define FOG_UP     1500.0f      /* distance at which haze is 50% */
#define FOG_DN     3500.0f
#define TMAX       40000.0f     /* beyond this a row is pure haze */

#define VEIL_H     70.0f        /* clearance below which the deck swallows us */

#define SUN_AZ     0.55f        /* sun direction, radians */
#define SUN_EL     0.30f
#define GLOW_AMP   85.0f

/*
 * How magnified a row has to be before it is worth interpolating.
 *
 * Bilinear sampling is four fetches and two lerps per plane against one fetch,
 * and with two planes stacked it is the single most expensive thing in the
 * frame: filtering every row that qualifies at 0.75 costs 17 ms of a 33 ms
 * frame here, and halves the frame rate on its own.
 *
 * The rows that actually need it are the few nearest ones, where a texel covers
 * enough pixels for the grid to show; beyond that the mip chain is already doing
 * the work. Measured on this target, filtering costs:
 *
 *     0.75 (the original)   30 ms   29 fps
 *     0.42                  21 ms   38 fps
 *     0.25                  14 ms   57 fps
 *     off                   13 ms   60 fps
 *
 * so 0.25 keeps interpolation where a texel is genuinely large and costs almost
 * nothing against turning it off. Raise it if the nearest cloud looks blocky.
 */
#define FILT_DUT   0.25f

#define NS0        (1 << NB0)

/* ----------------------------- tables ------------------------------- */

static uint8_t  mipbuf[NS0 * NS0 * 4 / 3 + 4];
static uint8_t *mip[NMIPS];
static int      mipbits[NMIPS];

static uint16_t lut_up[FOGSTEPS][DENSTEPS];
static uint16_t lut_dn[FOGSTEPS][DENSTEPS];

/* Per-frame state, written by clouds_begin_frame() and read by the bands. */
static float camx, camy, camz, sinY, cosY, hor;
static float s_h_lo, s_h_hi, s_h_dn;
static float s_time;
static int   s_veilk;

/* ------------------------- noise generation ------------------------- */

static uint32_t rs = 20250918u;
static unsigned rnd(void) { rs = rs * 1664525u + 1013904223u; return rs >> 8; }

static uint32_t smooth16(uint32_t f)            /* f*f*(3-2f), 16.16 in/out */
{
    uint32_t s = (uint32_t)(((uint64_t)f * f) >> 16);
    return (uint32_t)(((uint64_t)s * (196608u - 2u * f)) >> 16);
}

static void make_noise(void)
{
    /* value-noise fBm, tileable, 8 bit, built in place with no scratch buffer
       bigger than one lattice (32x32 max). */
    static const int AMP[5] = { 104, 62, 36, 22, 13 };
    uint8_t lat[64 * 64];
    int oct, x, y, i, lo, hi;

    memset(mip[0], 0, NS0 * NS0);

    for (oct = 0; oct < 5; oct++) {
        int cb = oct + 2;                  /* lattice is (1<<cb) cells wide */
        int G = 1 << cb, sh = NB0 - cb, fm = (1 << sh) - 1;
        int ox = rnd() & (NS0 - 1), oy = rnd() & (NS0 - 1);

        for (i = 0; i < G * G; i++) lat[i] = rnd() & 0xff;

        for (y = 0; y < NS0; y++) {
            int yy = (y + oy) & (NS0 - 1);
            int y0 = yy >> sh, y1 = (y0 + 1) & (G - 1);
            uint32_t wy = smooth16((uint32_t)(yy & fm) << (16 - sh));
            for (x = 0; x < NS0; x++) {
                int xx = (x + ox) & (NS0 - 1);
                int x0 = xx >> sh, x1 = (x0 + 1) & (G - 1);
                uint32_t wx = smooth16((uint32_t)(xx & fm) << (16 - sh));
                int a = lat[y0 * G + x0], b = lat[y0 * G + x1];
                int c = lat[y1 * G + x0], d = lat[y1 * G + x1];
                int t = a + (int)(((int64_t)(b - a) * wx) >> 16);
                int u = c + (int)(((int64_t)(d - c) * wx) >> 16);
                int v = t + (int)(((int64_t)(u - t) * wy) >> 16);
                mip[0][(y << NB0) + x] += (uint8_t)((v * AMP[oct]) >> 8);
            }
        }
    }

    /* stretch to the full 0..255 range so the coverage curve is predictable */
    lo = 255; hi = 0;
    for (i = 0; i < NS0 * NS0; i++) {
        if (mip[0][i] < lo) lo = mip[0][i];
        if (mip[0][i] > hi) hi = mip[0][i];
    }
    if (hi > lo)
        for (i = 0; i < NS0 * NS0; i++)
            mip[0][i] = (uint8_t)(((mip[0][i] - lo) * 255) / (hi - lo));

    /* box-filtered mip chain */
    for (i = 1; i < NMIPS; i++) {
        int n = NS0 >> i, p = n << 1;
        for (y = 0; y < n; y++)
            for (x = 0; x < n; x++)
                mip[i][y * n + x] = (uint8_t)((mip[i - 1][(2 * y) * p + 2 * x] +
                                              mip[i - 1][(2 * y) * p + 2 * x + 1] +
                                              mip[i - 1][(2 * y + 1) * p + 2 * x] +
                                              mip[i - 1][(2 * y + 1) * p + 2 * x + 1] + 2) >> 2);
    }
}

/* --------------------------- colour ladder -------------------------- */

static uint16_t pack565(float r, float g, float b)
{
    int R = (int)(r + 0.5f), G = (int)(g + 0.5f), B = (int)(b + 0.5f);
    if (R < 0) R = 0;
    if (G < 0) G = 0;
    if (B < 0) B = 0;
    if (R > 255) R = 255;
    if (G > 255) G = 255;
    if (B > 255) B = 255;
    return (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
}

static float sstep(float a, float b, float x)
{
    float t = (x - a) / (b - a);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t * t * (3.0f - 2.0f * t);
}

#define MIX(a, b, t) ((a) + ((b) - (a)) * (t))

static void make_ladder(void)
{
    /* one colour per (fog band, density).  fog band doubles as "how far from
       straight overhead are we looking", which is what drives the gradient. */
    const float zen[3]  = {  56, 108, 200 };   /* zenith blue      */
    const float hor[3]  = { 152, 186, 226 };   /* sky at horizon   */
    const float haze[3] = { 208, 217, 232 };   /* what fog fades to*/
    const float lit[3]  = { 255, 255, 251 };   /* sunlit cloud     */
    const float shad[3] = { 116, 129, 154 };   /* dense underside  */
    const float gap[3]  = {  84, 104, 134 };   /* looking down through gaps */
    const float topl[3] = { 246, 248, 250 };   /* sunlit cloud tops */
    const float tops[3] = { 168, 180, 202 };
    int k, j, c;

    for (k = 0; k < FOGSTEPS; k++) {
        float F = (k + 0.5f) / FOGSTEPS;                 /* haze amount */
        float e = powf(1.0f - F, 0.70f);                 /* 1 overhead, 0 at horizon */
        float sky[3], dn[3];
        for (c = 0; c < 3; c++) {
            sky[c] = MIX(hor[c], zen[c], e);
            dn[c]  = MIX(haze[c], gap[c], e);
        }
        for (j = 0; j < DENSTEPS; j++) {
            float d = j / (float)(DENSTEPS - 1);
            float a, s, cl[3], o[3];

            /* -- underside of the deck above us -- */
            a = sstep(0.36f, 0.68f, d);                  /* coverage        */
            s = sstep(0.46f, 0.96f, d);                  /* self-shadowing  */
            for (c = 0; c < 3; c++) {
                cl[c] = MIX(lit[c], shad[c], s);
                o[c]  = MIX(MIX(sky[c], cl[c], a), haze[c], F);
            }
            lut_up[k][j] = pack565(o[0], o[1], o[2]);

            /* -- tops of the broken deck below us -- */
            a = sstep(0.40f, 0.80f, d);
            s = sstep(0.45f, 1.00f, d);
            for (c = 0; c < 3; c++) {
                cl[c] = MIX(tops[c], topl[c], s);
                o[c]  = MIX(MIX(dn[c], cl[c], a), haze[c], F);
            }
            lut_dn[k][j] = pack565(o[0], o[1], o[2]);
        }
    }
}

/* ---------------------------- plane sampler ------------------------- */

typedef struct {
    const uint8_t *tex;
    uint32_t bits, mask;
    uint32_t u, v;
    uint32_t du, dv;
    int      filt;              /* magnifying: interpolate instead of point */
} Samp;

/* set up one scanline of one horizontal plane at distance t */
static void setup(Samp *s, float t, float texscale, float uoff, float voff)
{
    const float ipp = 1.0f / FOCAL;
    const float sx0 = (0.5f - W * 0.5f) * ipp;      /* ray slope at x = 0 */
    float ts = texscale, dut = t * ipp * texscale, n;
    float u0, v0, du, dv;
    int m = 0;

    while (m + 1 < NMIPS && dut > 1.1f) { dut *= 0.5f; ts *= 0.5f; m++; }

    u0 = (camx + t * sinY + t * cosY * sx0 + uoff) * ts;
    v0 = (camz + t * cosY - t * sinY * sx0 + voff) * ts;
    du =  t * cosY * ipp * ts;
    dv = -t * sinY * ipp * ts;

    n = (float)(1 << mipbits[m]);
    u0 = fmodf(u0, n);
    v0 = fmodf(v0, n);
    if (u0 < 0) u0 += n;
    if (v0 < 0) v0 += n;

    s->tex  = mip[m];
    s->bits = (uint32_t)mipbits[m];
    s->mask = (1u << mipbits[m]) - 1u;
    s->u    = (uint32_t)(int32_t)(u0 * 65536.0f);
    s->v    = (uint32_t)(int32_t)(v0 * 65536.0f);
    s->du   = (uint32_t)(int32_t)(du * 65536.0f);
    s->dv   = (uint32_t)(int32_t)(dv * 65536.0f);
    s->filt = (dut < FILT_DUT);
}

#define FETCH(s) ((s).tex[((((s).v >> 16) & (s).mask) << (s).bits) | \
                          (((s).u >> 16) & (s).mask)])
#define STEP(s)  ((s).u += (s).du, (s).v += (s).dv)

/* Only used on the near rows, where one texel covers many pixels and point
   sampling would show the texel grid. */
static unsigned bilin(const Samp *s)
{
    uint32_t m = s->mask, b = s->bits;
    uint32_t x0 = (s->u >> 16) & m, x1 = (x0 + 1) & m;
    uint32_t y0 = (s->v >> 16) & m, y1 = (y0 + 1) & m;
    int fx = (int)((s->u >> 8) & 255), fy = (int)((s->v >> 8) & 255);
    int a = s->tex[(y0 << b) | x0], c = s->tex[(y0 << b) | x1];
    int d = s->tex[(y1 << b) | x0], e = s->tex[(y1 << b) | x1];
    int t = a + (((c - a) * fx) >> 8);
    int u = d + (((e - d) * fx) >> 8);
    return (unsigned)(t + (((u - t) * fy) >> 8));
}

static int fogband(float t, float fogd)
{
    int k = (int)(FOGSTEPS * (t / (t + fogd)));
    return k < 0 ? 0 : (k >= FOGSTEPS ? FOGSTEPS - 1 : k);
}

/* ------------------------------- glow ------------------------------- */

static uint8_t gx[W], gy[H];

static void setup_glow(float yaw)
{
    float da = SUN_AZ - yaw, sx, sy;
    int x, y;

    if (fabsf(da) > 1.25f) { memset(gy, 0, sizeof gy); return; }
    sx = W * 0.5f + tanf(da) * FOCAL;
    sy = hor - tanf(SUN_EL) * FOCAL;
    for (x = 0; x < W; x++) {
        float d = (x - sx) / 70.0f;
        gx[x] = (uint8_t)(GLOW_AMP * expf(-d * d));
    }
    for (y = 0; y < H; y++) {
        float d = (y - sy) / 52.0f;
        gy[y] = (uint8_t)(255.0f * expf(-d * d));
    }
}

static void apply_glow(uint16_t *line, int y)
{
    unsigned gyv = gy[y], x;
    if (!gyv) return;
    for (x = 0; x < W; x++) {
        unsigned g = (gx[x] * gyv) >> 8;
        if (g > 2) {
            uint16_t c = line[x];
            unsigned r = ((c >> 11) & 31) + (g >> 3);
            unsigned v = ((c >>  5) & 63) + (g >> 2);
            unsigned b =  (c        & 31) + ((g * 3) >> 5);
            if (r > 31) r = 31;
            if (v > 63) v = 63;
            if (b > 31) b = 31;
            line[x] = (uint16_t)((r << 11) | (v << 5) | b);
        }
    }
}


/* ------------------------------ the API ----------------------------- */

void clouds_init(void)
{
    int i, off = 0;

    for (i = 0; i < NMIPS; i++) {
        mipbits[i] = NB0 - i;
        mip[i] = mipbuf + off;
        off += (NS0 >> i) * (NS0 >> i);
    }

    make_noise();
    make_ladder();
}

float clouds_horizon(void) { return H * 0.5f; }

void clouds_begin_frame(float yaw, float dt)
{
    float veil, prox;
    int dcam;

    sinY = sinf(yaw);
    cosY = cosf(yaw);

    /* The camera bobs gently, which is much of what sells the flight: a fixed
     * height under a fixed deck reads as a still image sliding past. Two slow
     * sines with no common period, so it never settles into a rhythm. */
    s_time += dt;
    camy = 62.0f * sinf(0.29f * s_time) + 42.0f * sinf(0.11f * s_time + 2.0f);
    hor  = clouds_horizon();

    s_h_lo = DECK_LO - camy;
    if (s_h_lo < 6.0f) s_h_lo = 6.0f;
    s_h_hi = DECK_HI - camy;
    if (s_h_hi < s_h_lo + 6.0f) s_h_hi = s_h_lo + 6.0f;
    s_h_dn = DECK_DN - camy;

    /* About to be swallowed? The coarsest mip is large-scale density. */
    {
        int b = mipbits[NMIPS - 1], m = (1 << b) - 1;
        int iu = ((int)(camx * TEX_LO) >> (NB0 - b)) & m;
        int iv = ((int)(camz * TEX_LO) >> (NB0 - b)) & m;
        dcam = mip[NMIPS - 1][(iv << b) | iu];
    }
    prox = (VEIL_H - s_h_lo) / VEIL_H;
    if (prox < 0) prox = 0;
    if (prox > 1) prox = 1;
    veil = powf(prox, 0.7f) * sstep(0.20f, 0.80f, dcam / 255.0f);
    s_veilk = (int)(veil * (FOGSTEPS - 1));

    setup_glow(yaw);

    camx += SPEED * dt * sinY;
    camz += SPEED * dt * cosY;
}

void clouds_render_band(uint16_t *fb, int pitch, int y0, int y1)
{
    int y;

    for (y = y0; y < y1; y++) {
        uint16_t *line = fb + (size_t)y * pitch;
        float sy = (hor - (y + 0.5f)) * (1.0f / FOCAL);
        int x, k;

        if (sy > 1.0f / 8192) {                  /* looking up: cloud deck */
            float t = s_h_lo / sy;
            if (t > TMAX) {
                uint16_t c = lut_up[FOGSTEPS - 1][0];
                for (x = 0; x < W; x++) line[x] = c;
            } else {
                Samp a, b;
                const uint16_t *lut;
                setup(&a, t,          TEX_LO, 0.0f,   0.0f);
                setup(&b, s_h_hi / sy, TEX_HI, 613.0f, 271.0f);
                k = fogband(t, FOG_UP);
                if (k < s_veilk) k = s_veilk;
                lut = lut_up[k];
                if (a.filt | b.filt) {
                    for (x = 0; x < W; x++) {
                        line[x] = lut[(bilin(&a) + bilin(&b)) >> 3];
                        STEP(a); STEP(b);
                    }
                } else {
                    for (x = 0; x < W; x++) {
                        line[x] = lut[(FETCH(a) + FETCH(b)) >> 3];
                        STEP(a); STEP(b);
                    }
                }
            }
        } else if (sy < -1.0f / 8192) {          /* looking down: lower deck */
            float t = s_h_dn / sy;
            if (t > TMAX) {
                uint16_t c = lut_dn[FOGSTEPS - 1][0];
                for (x = 0; x < W; x++) line[x] = c;
            } else {
                Samp a;
                const uint16_t *lut;
                setup(&a, t, TEX_DN, 1907.0f, 3313.0f);
                k = fogband(t, FOG_DN);
                if (k < s_veilk) k = s_veilk;
                lut = lut_dn[k];
                if (a.filt) {
                    for (x = 0; x < W; x++) { line[x] = lut[bilin(&a) >> 2]; STEP(a); }
                } else {
                    for (x = 0; x < W; x++) { line[x] = lut[FETCH(a) >> 2]; STEP(a); }
                }
            }
        } else {                                 /* the horizon itself */
            uint16_t c = lut_up[FOGSTEPS - 1][0];
            for (x = 0; x < W; x++) line[x] = c;
        }

        apply_glow(line, y);
    }
}
