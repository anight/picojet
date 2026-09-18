/*
 * clouds.c -- "flying through the clouds" frame generator.
 *
 *   cc -O2 -o clouds clouds.c -lm
 *   ./clouds 900 out            # writes out/frame00000.png ...
 *   ./clouds -b                 # benchmark: render forever, fps once a second
 *   ffmpeg -framerate 30 -i out/frame%05d.png -pix_fmt yuv420p clouds.mp4
 *
 * Everything is 320x200 RGB565 internally (the PNGs are that framebuffer
 * expanded to 8 bit/channel, since PNG has no 565 format).
 *
 * How it works
 * ------------
 * The clouds are two horizontal *planes* above the camera, textured with one
 * precomputed, tileable fBm noise tile.  For a pinhole camera a horizontal
 * plane maps to the screen so that, on a given scanline, the distance t to the
 * plane is constant and the texture coordinate is *linear in x*.  So a row
 * costs one divide of setup and then one add per axis per pixel -- the old
 * "floor mapper".  Two planes at slightly different heights and texture
 * scales parallax against each other and read as volume.
 *
 * Colour never gets computed per pixel.  A ladder lut[fog][density] is built
 * once at startup: it already contains sky gradient, cloud shading, cloud
 * coverage curve and distance haze.  Per row we pick the fog band, per pixel
 * we do two texture fetches and one ladder fetch.  No per-pixel multiply,
 * no floats, no framebuffer -- one 320-pixel line buffer.
 *
 * Flying *into* a cloud is free: when the camera's clearance under the deck
 * gets small and the noise above it is dense, we simply clamp the fog band to
 * a minimum, and the whole frame washes out to haze.
 *
 * Memory (the point of the exercise):
 *     noise mip chain 256^2+128^2+64^2+32^2   85 KB
 *     colour ladders  2 * 128 * 64 * 2 B      32 KB
 *     line buffer     320 * 2 B              640 B
 * Drop NB0 to 7 and it is 21 KB + 32 KB, which fits anywhere.
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

/* ----------------------------- tunables ----------------------------- */

#define W          320
#define H          200
#define FPS        30

#define NB0        8            /* base noise tile is (1<<NB0) square */
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

#define TURN_T     20.0f        /* seconds for one right-then-left cycle */
#define TURN_A     1.00f        /* peak heading excursion, radians (~57 deg) */

#define SUN_AZ     0.55f        /* sun direction, radians */
#define SUN_EL     0.30f
#define GLOW_AMP   85.0f

#define NS0        (1 << NB0)

/* ----------------------------- tables ------------------------------- */

static uint8_t  mipbuf[NS0 * NS0 * 4 / 3 + 4];
static uint8_t *mip[NMIPS];
static int      mipbits[NMIPS];

static uint16_t lut_up[FOGSTEPS][DENSTEPS];
static uint16_t lut_dn[FOGSTEPS][DENSTEPS];

static uint16_t line[W];

/* Benchmark only.  fb stays NULL for normal rendering, where a finished row
   goes straight out to the PNG and no whole frame is ever held.  Delete both
   of these when porting -- the renderer itself never needs them. */
static uint16_t  fbuf[W * H];
static uint16_t *fb;

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

static float camx, camy, camz, sinY, cosY, hor;

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
    s->filt = (dut < 0.75f);
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

static void apply_glow(int y)
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

/* ------------------------------- PNG -------------------------------- */
/* Minimal writer: stored (uncompressed) deflate, one block per scanline.
   No zlib, no whole-frame buffer -- rows are streamed as they are rendered. */

static uint32_t crctab[256], crc;
static uint32_t s1 = 1, s2 = 0;
static FILE *fp;

static void crc_init(void)
{
    uint32_t i, j, c;
    for (i = 0; i < 256; i++) {
        for (c = i, j = 0; j < 8; j++) c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
        crctab[i] = c;
    }
}

static void raw32(uint32_t v)
{
    fputc(v >> 24, fp); fputc((v >> 16) & 255, fp);
    fputc((v >> 8) & 255, fp); fputc(v & 255, fp);
}

static void wr(const void *p, size_t n)
{
    const uint8_t *b = p;
    size_t i;
    fwrite(b, 1, n, fp);
    for (i = 0; i < n; i++) crc = crctab[(crc ^ b[i]) & 255] ^ (crc >> 8);
}

static void chunk(const char *tag, uint32_t len)
{
    raw32(len);
    crc = 0xffffffffu;
    wr(tag, 4);
}

static void endchunk(void) { raw32(crc ^ 0xffffffffu); }

static void adler(const uint8_t *b, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) { s1 = (s1 + b[i]) % 65521; s2 = (s2 + s1) % 65521; }
}

static void png_open(const char *path)
{
    static const uint8_t sig[8] = { 137, 'P', 'N', 'G', 13, 10, 26, 10 };
    uint8_t ihdr[13];
    uint8_t zh[2] = { 0x78, 0x01 };

    fp = fopen(path, "wb");
    if (!fp) { perror(path); exit(1); }
    fwrite(sig, 1, 8, fp);

    ihdr[0] = W >> 24; ihdr[1] = (W >> 16) & 255; ihdr[2] = (W >> 8) & 255; ihdr[3] = W & 255;
    ihdr[4] = H >> 24; ihdr[5] = (H >> 16) & 255; ihdr[6] = (H >> 8) & 255; ihdr[7] = H & 255;
    ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
    chunk("IHDR", 13); wr(ihdr, 13); endchunk();

    /* zlib header + H stored blocks of (filter byte + 3*W) + adler */
    chunk("IDAT", (uint32_t)(2 + H * (5 + 1 + 3 * W) + 4));
    wr(zh, 2);
    s1 = 1; s2 = 0;
}

static void png_row(const uint16_t *src, int last)
{
    uint8_t blk[5], row[1 + 3 * W];
    unsigned len = 1 + 3 * W, x;

    blk[0] = last ? 1 : 0;                       /* BFINAL, BTYPE = stored */
    blk[1] = len & 255; blk[2] = len >> 8;
    blk[3] = (~len) & 255; blk[4] = (~len >> 8) & 255;
    wr(blk, 5);

    row[0] = 0;                                  /* filter: none */
    for (x = 0; x < W; x++) {
        unsigned c = src[x], r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
        row[1 + 3 * x]     = (uint8_t)((r << 3) | (r >> 2));
        row[1 + 3 * x + 1] = (uint8_t)((g << 2) | (g >> 4));
        row[1 + 3 * x + 2] = (uint8_t)((b << 3) | (b >> 2));
    }
    wr(row, len);
    adler(row, len);
}

static void png_close(void)
{
    uint8_t ad[4];                               /* adler32 lives inside IDAT,
                                                    so it must go through wr() */
    ad[0] = (uint8_t)(s2 >> 8); ad[1] = (uint8_t)s2;
    ad[2] = (uint8_t)(s1 >> 8); ad[3] = (uint8_t)s1;
    wr(ad, 4);
    endchunk();
    chunk("IEND", 0); endchunk();
    fclose(fp);
}

/* ------------------------------ frame ------------------------------- */

static void render(int frame)
{
    float ts = frame / (float)FPS, yaw, veil, h_lo, h_hi, h_dn, prox;
    int y, veilk, dcam;

    /* Heading is one clean sine.  Phased as -cos so the rate of turn -- which
       is what the eye actually reads as "turning" -- starts at zero: fly
       straight, bank into a long right turn, straight again, then left. */
    yaw  = -TURN_A * cosf((6.2831853f / TURN_T) * ts);
    sinY = sinf(yaw); cosY = cosf(yaw);
    camy = 62.0f * sinf(0.29f * ts) + 42.0f * sinf(0.11f * ts + 2.0f);
    hor  = H * 0.5f;                 /* horizon pinned to the centre row */

    h_lo = DECK_LO - camy;
    if (h_lo < 6.0f) h_lo = 6.0f;
    h_hi = DECK_HI - camy;
    if (h_hi < h_lo + 6.0f) h_hi = h_lo + 6.0f;
    h_dn = DECK_DN - camy;

    /* are we about to be swallowed?  coarsest mip = large scale density */
    {
        int b = mipbits[NMIPS - 1], m = (1 << b) - 1;
        int iu = ((int)(camx * TEX_LO) >> (NB0 - b)) & m;
        int iv = ((int)(camz * TEX_LO) >> (NB0 - b)) & m;
        dcam = mip[NMIPS - 1][(iv << b) | iu];
    }
    prox = (VEIL_H - h_lo) / VEIL_H;
    if (prox < 0) prox = 0;
    if (prox > 1) prox = 1;
    veil = powf(prox, 0.7f) * sstep(0.20f, 0.80f, dcam / 255.0f);
    veilk = (int)(veil * (FOGSTEPS - 1));

    setup_glow(yaw);

    for (y = 0; y < H; y++) {
        float sy = (hor - (y + 0.5f)) * (1.0f / FOCAL);
        int x, k;

        if (sy > 1.0f / 8192) {                  /* looking up: cloud deck */
            float t = h_lo / sy;
            if (t > TMAX) {
                uint16_t c = lut_up[FOGSTEPS - 1][0];
                for (x = 0; x < W; x++) line[x] = c;
            } else {
                Samp a, b;
                const uint16_t *lut;
                setup(&a, t,        TEX_LO, 0.0f,   0.0f);
                setup(&b, h_hi / sy, TEX_HI, 613.0f, 271.0f);
                k = fogband(t, FOG_UP);
                if (k < veilk) k = veilk;
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
            float t = h_dn / sy;
            if (t > TMAX) {
                uint16_t c = lut_dn[FOGSTEPS - 1][0];
                for (x = 0; x < W; x++) line[x] = c;
            } else {
                Samp a;
                const uint16_t *lut;
                setup(&a, t, TEX_DN, 1907.0f, 3313.0f);
                k = fogband(t, FOG_DN);
                if (k < veilk) k = veilk;
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

        apply_glow(y);
        if (fb) memcpy(fb + (size_t)y * W, line, sizeof line);
        else    png_row(line, y == H - 1);
    }

    camx += SPEED / FPS * sinY;
    camz += SPEED / FPS * cosY;
}

/* ---------------------------- benchmark ----------------------------- */

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static volatile uint16_t sink;          /* keeps the frame buffer live */

static void benchmark(void)
{
    double tprev = now_s();
    unsigned long total = 0, mark = 0;
    int frame = 0;

    fb = fbuf;
    printf("rendering %dx%d into a static buffer, no PNG.  ^C to stop.\n", W, H);
    fflush(stdout);

    for (;;) {
        double t;

        render(frame);
        sink = fbuf[(total * 7919u) % (W * H)];
        if (++frame == 1000000) frame = 0;      /* keep the camera flying */
        total++;

        t = now_s();
        if (t - tprev >= 1.0) {
            double dt = t - tprev;
            unsigned long n = total - mark;
            printf("%8.1f fps   %6.3f ms/frame   (%lu frames)\n",
                   n / dt, 1000.0 * dt / n, total);
            fflush(stdout);
            tprev = t;
            mark = total;
        }
    }
}

int main(int argc, char **argv)
{
    int bench = (argc > 1 && (!strcmp(argv[1], "-b") || !strcmp(argv[1], "--bench")));
    int nframes = (!bench && argc > 1) ? atoi(argv[1]) : 900;   /* 30 s at 30 fps */
    const char *dir = argc > 2 ? argv[2] : ".";
    char path[512];
    int f, i, off = 0;

    for (i = 0; i < NMIPS; i++) {
        mipbits[i] = NB0 - i;
        mip[i] = mipbuf + off;
        off += (NS0 >> i) * (NS0 >> i);
    }

    crc_init();
    make_noise();
    make_ladder();

    if (bench) { benchmark(); return 0; }       /* never returns */

    for (f = 0; f < nframes; f++) {
        snprintf(path, sizeof path, "%s/frame%05d.png", dir, f);
        png_open(path);
        render(f);
        png_close();
        if ((f % 30) == 0) { printf("\r%d/%d", f, nframes); fflush(stdout); }
    }
    printf("\r%d frames written to %s\n", nframes, dir);
    return 0;
}
