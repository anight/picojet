/*
 * picojet flight: an F22 held in the middle of the screen, flying into a cloud
 * deck, with the stick steering left and right.
 *
 * Two renderers share one framebuffer, and the division of labour is the point.
 *
 *   clouds.cpp fills every pixel with the sky - a floor mapper over a noise tile,
 *   costing two texture fetches and a table lookup per pixel and no multiply. It
 *   replaces clearing the buffer rather than being drawn over a cleared one,
 *   which is why Jet runs with setClearBuffer(false).
 *
 *   Jet then draws the aircraft on top of it, textured and lit, into the same
 *   buffer.
 *
 * Both halves are split across the two cores the same way and in the same pass:
 * core 0 takes the top of the frame and core 1 the bottom, for the clouds and
 * then for the triangles, with one handshake covering both.
 *
 * Steering is yaw only, and it is indirect: the stick rolls the aircraft and the
 * roll turns it, each with the lag that gives. The horizon stays level because
 * the camera never rolls - the aeroplane banks into the turn instead, which is
 * the cue the eye reads. Holding the stick over turns the sky underneath
 * continuously; letting go unwinds the bank and leaves the heading it reached.
 */
#include <cmath>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "SDL2/SDL.h"
#include "psdl_pico.h"

#include "Jet.hpp"
#include "clouds.hpp"

#include "assets/mesh_f22.h"
#include "assets/tex_f22.h"
#include "assets/mesh_efa.h"
#include "assets/tex_efa.h"
#include "assets/mesh_f117.h"
#include "assets/tex_f117.h"

using namespace Renderer;

#ifndef PICOJET_SYS_CLOCK_KHZ
#define PICOJET_SYS_CLOCK_KHZ PSDL_PICO_SYS_CLOCK_KHZ
#endif

/*
 * Which way the mesh's nose points, as a yaw in degrees.
 *
 * Every mesh here has its long axis on X with the nose toward +X - checked, not
 * assumed: at each end of the bounding box the nose is the few vertices that
 * share one z, and the tail is the wide spread. This turns it to put the nose
 * down +Z, away from the camera, so the aircraft flies into the distance rather
 * than at the viewer.
 */
static constexpr int32_t PLANE_YAW_DEG = 270;

/* ------------------------------------------------------------- aircraft */

/*
 * The aircraft, and where each one's fire comes from.
 *
 * The nozzle is read off the mesh rather than guessed - it is the vertex ring on
 * the aft face of the engine, and every one of these was found by looking at
 * what the vertices actually do at the back of the model.
 *
 * How many plumes an aircraft gets is a question about the aircraft, not about
 * the mesh:
 *
 *   F-22       two engines, afterburning. Two plumes.
 *   EF-2000    two engines, afterburning. Two plumes, closer together and
 *              lower than the F-22's - its nozzles sit under the tail boom.
 *   F-117      two engines and no afterburner at all. Its exhausts are wide
 *              slots that mix the efflux with cold air precisely so there is
 *              nothing to see, which is most of the point of the aircraft.
 *              Drawing fire on it would be drawing the one thing it was built
 *              not to have. None.
 *
 * The biplane from the viewer is not here. At 597 triangles it needs a render
 * queue two and a half times the largest of these, and this demo has a cloud
 * deck and two framebuffers in the RAM the viewer spends on its heap.
 *
 * `nozzles` is a count rather than a flag because a single-engine jet wants one
 * plume on the centreline, and the build below handles that - though nothing in
 * this set is one.
 */
struct PlaneDef {
	const char     *name;
	const int16_t (*verts)[8];
	int             vertCount;
	const uint16_t(*tris)[3];
	int             triCount;
	const uint16_t *tex;
	int             texW, texH;

	int      nozzles;                     /* 0, 1 or 2 */
	int16_t  nozzleX, nozzleY, nozzleZ;   /* mouth, in mesh units; z is mirrored */
	int16_t  nozzleR;                     /* its radius, which scales the plume */
};

static const PlaneDef g_planes[] = {
	{ "F-22", mesh_f22_verts, MESH_F22_VERTS, mesh_f22_tris, MESH_F22_TRIS,
	  tex_f22_data, TEX_F22_W, TEX_F22_H,
	  2, -200, -25, 26, 14 },

	{ "EF-2000", mesh_efa_verts, MESH_EFA_VERTS, mesh_efa_tris, MESH_EFA_TRIS,
	  tex_efa_data, TEX_EFA_W, TEX_EFA_H,
	  2, -224, -44, 15, 13 },

	{ "F-117", mesh_f117_verts, MESH_F117_VERTS, mesh_f117_tris, MESH_F117_TRIS,
	  tex_f117_data, TEX_F117_W, TEX_F117_H,
	  0, 0, 0, 0, 0 },
};

static constexpr int PLANE_COUNT = (int)(sizeof g_planes / sizeof *g_planes);

/* Whose nozzles the plume geometry is currently built around. */
static const PlaneDef *g_plane = &g_planes[0];

/*
 * Where the aircraft sits, and how far the camera is above it.
 *
 * The horizon is the middle row, so anything level with the camera lands there.
 * Raising the camera by h drops the aircraft h * FOCAL / z rows below it: 545
 * over 2800 at a focal length of 220 puts it about 43 rows down, in the lower
 * half with the sky and the horizon above it.
 *
 * The two are tied together. Closing the distance to make the aircraft larger
 * also steepens the angle down to it, so the height has to come down in the same
 * proportion or it sinks out of the frame.
 */
static constexpr int32_t PLANE_Z    = 700 * JET32_WORLD_SCALE;
static constexpr int32_t CAM_HEIGHT = 545;

/*
 * The flight model, such as it is: the stick rolls the aircraft, and the roll
 * is what turns it.
 *
 * ROLL_RESPONSE is the only inertia in here, and everything else inherits it.
 * It is the rate constant of the roll chasing the stick, so the bank reaches
 * about 95% of what the stick asks for in 3/ROLL_RESPONSE seconds - a little
 * under two at this value. Lower is heavier.
 */
static constexpr float TURN_RATE     = 0.9f;    /* rad/s of yaw at full bank */
static constexpr float BANK_MAX      = 55.0f;   /* degrees of bank at full stick */
static constexpr float ROLL_RESPONSE = 1.8f;    /* 1/s */

static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 200;

/*
 * Two framebuffers, so the panel reads one while both cores fill the other.
 * 125 KB each, and here they are carrying a full-screen background as well as
 * the model, so the overlap is worth more than it was without one.
 */
static uint16_t g_framebuffer[2][SCREEN_W * SCREEN_H];

/* ------------------------------------------------------------ the scene */

static Scene  *g_scene;
static Camera  g_camera;

/*
 * The band the two cores split at. Both the clouds and the triangles use it, so
 * the halves of the frame stay on the same core and whatever each wrote is the
 * other's to leave alone.
 */
static constexpr int BAND_SPLIT = SCREEN_H / 2;

static constexpr int MAX_QUEUED_TRIS = 512;
static uint8_t g_flags_core0[MAX_QUEUED_TRIS];
static uint8_t g_flags_core1[MAX_QUEUED_TRIS];

/* What core 1 is being asked to do this frame. */
static uint16_t *volatile g_back;

/*
 * Core 1: the bottom half of the sky, then the bottom half of the geometry.
 *
 * One FIFO token covers both because they are strictly ordered - the clouds have
 * to be under the aircraft - and because a second handshake per frame would cost
 * more than it separates. Core 0 does its own top half between the two, so the
 * cores meet only at the token.
 */
static void core1_worker(void)
{
	for (;;) {
		/* Waiting for a frame is all this core does between frames, and it is
		 * the only thing picosdl can be told about it - everything outside the
		 * two brackets is the half-frame this core actually renders. */
		PSDL_CpuIdle();
		uint32_t n_tris = multicore_fifo_pop_blocking();
		PSDL_CpuBusy();

		clouds_render_band(g_back, SCREEN_W, BAND_SPLIT, SCREEN_H);

		memset(g_flags_core1, 0, n_tris);
		g_scene->rasterizeBand(BAND_SPLIT, SCREEN_H, g_flags_core1);

		multicore_fifo_push_blocking(1);
	}
}

/*
 * The raster half of the frame, as Jet's executor hook.
 *
 * By the time Jet calls this, prepareFrame() has already transformed and sorted;
 * the clouds are laid down before render() is entered, because Jet would
 * otherwise have nothing to draw over.
 */
static void raster_executor(Scene &scene)
{
	const int queued = scene.lastFrameDrawnTriangles;

	if (queued > MAX_QUEUED_TRIS) {
		printf("picojet: %d queued triangles exceeds MAX_QUEUED_TRIS (%d)\n",
		       queued, MAX_QUEUED_TRIS);
		fflush(stdout);
		for (;;) { }
	}

	/* Release core 1 here rather than in the frame loop: prepareFrame() has run
	 * by the time Jet calls the executor, so the queue core 1 is about to read
	 * is complete, and the token carries that ordering. */
	multicore_fifo_push_blocking((uint32_t)queued);

	clouds_render_band(g_back, SCREEN_W, 0, BAND_SPLIT);

	memset(g_flags_core0, 0, queued);
	scene.rasterizeBand(0, BAND_SPLIT, g_flags_core0);

	PSDL_CpuIdle();
	(void)multicore_fifo_pop_blocking();      /* core 1 has finished its half */
	PSDL_CpuBusy();

	int rasterized = 0;
	for (int i = 0; i < queued; ++i)
		if (g_flags_core0[i] | g_flags_core1[i])
			++rasterized;
	scene.lastFrameRasterizedTriangles = rasterized;
}

/* ------------------------------------------------------------ afterburner */

/*
 * Two exhaust plumes, as geometry in the aircraft's own local frame.
 *
 * Taken off the mesh rather than guessed: the f22 nozzles are the vertex rings
 * at x = -200, y = -25, z = +-26, which is the rear face of each nacelle. The
 * plume grows from there along -X, straight out of the back.
 *
 * Each plume is a pair of ribbons crossed at right angles rather than a cone.
 * A cone of any decent roundness costs ten times the triangles for a shape the
 * eye cannot tell apart at this size, and a single flat ribbon vanishes when the
 * aircraft banks it edge-on. Two crossed ribbons never present nothing, and are
 * eight quads for both engines together.
 *
 * All eight share one generated texture and one UNLIT material, so the colour is
 * continuous along the plume instead of stepping between per-segment materials.
 * UNLIT rather than the ADDITIVE mode Jet also offers, which is the obvious
 * choice for a flame: an add against this sky saturates. The cloud deck is a
 * bright lavender, so adding any hot colour to it pins every channel and the
 * plume comes out white. Opaque is both what the reference photograph shows and
 * the cheaper of the two - a plain write, no read-modify-write per pixel.
 */
static constexpr int FLAME_STATIONS = 5;
static constexpr int FLAME_SEGMENTS = FLAME_STATIONS - 1;

/*
 * Distance aft of the nozzle, and the half-width there, at full burn. The widths
 * are for a nozzle of radius FLAME_REF_R and are scaled to whatever the current
 * aircraft's is, so a smaller engine gets a proportionally thinner plume without
 * a second profile to maintain. The last is not zero because a zero-width quad is
 * a degenerate triangle, and 3 units is a point.
 *
 * The length is set by where the plume ends up on screen rather than by what an
 * afterburner looks like from the side. It grows toward the camera, so
 * perspective works against it hard: the nozzle is 2000 units away and every
 * unit aft is a unit nearer, and a plume of the length the reference photograph
 * suggests reaches within a few hundred units of the lens, sweeping off the
 * bottom of the frame as a pair of enormous wedges. 190 puts the tip about 25
 * rows below the nozzle, which reads as a trail and stays in the picture.
 */
static const int16_t flame_aft[FLAME_STATIONS]  = { 0, 30, 75, 130, 190 };
static const int16_t flame_half[FLAME_STATIONS] = { 14, 12,  9,   6,   3 };
static constexpr int  FLAME_REF_R = 14;   /* the radius those widths are drawn for */


/*
 * The flame texture, and what its two axes are for.
 *
 * U runs along the plume and carries the colour ramp, white-hot at the nozzle
 * down to a dark red at the tip. V is not a second spatial axis at all - it is
 * time. Every column is the same point of the plume under a different moment of
 * turbulence, and the plume is animated by scrolling V.
 *
 * Splitting them this way is what lets one texture do both jobs. Baking the ramp
 * and the turbulence into the same axis would mean that scrolling to animate the
 * fire also scrolls the ramp, and the white-hot part would march down the plume
 * and off the end. With the ramp pinned to U it stays welded to the geometry
 * while V boils underneath it.
 *
 * 64 by 32 is 4 KB, generated once at startup.
 */
static constexpr int FLAME_TEX_W = 64;     /* along the plume */
static constexpr int FLAME_TEX_H = 32;     /* time */
static uint16_t s_flame_tex[FLAME_TEX_W * FLAME_TEX_H];

/* Colour along the plume. */
struct FlameStop { float t; uint8_t r, g, b; };
static const FlameStop flame_ramp[] = {
	{ 0.00f, 255, 255, 236 },   /* the nozzle, blown out */
	{ 0.10f, 255, 236, 150 },
	{ 0.28f, 255, 168,  52 },
	{ 0.52f, 246,  92,  28 },
	{ 0.76f, 190,  42,  22 },
	{ 1.00f, 104,  20,  14 },
};

/*
 * Tileable value noise. The lattice wraps at (px, py), which is what makes V
 * loop seamlessly - the scroll runs forever and never shows a join.
 */
static float flame_lattice(int x, int y, int px, int py)
{
	x = ((x % px) + px) % px;
	y = ((y % py) + py) % py;
	uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + 0x9E3779B9u;
	h = (h ^ (h >> 13)) * 1274126177u;
	return (float)((h ^ (h >> 16)) & 0xFFFFu) * (1.0f / 65535.0f);
}

static float flame_noise(float x, float y, int px, int py)
{
	const int   xi = (int)floorf(x), yi = (int)floorf(y);
	float fx = x - (float)xi, fy = y - (float)yi;
	fx = fx * fx * (3.0f - 2.0f * fx);
	fy = fy * fy * (3.0f - 2.0f * fy);

	const float a = flame_lattice(xi,     yi,     px, py);
	const float b = flame_lattice(xi + 1, yi,     px, py);
	const float c = flame_lattice(xi,     yi + 1, px, py);
	const float d = flame_lattice(xi + 1, yi + 1, px, py);

	const float top = a + (b - a) * fx;
	return top + ((c + (d - c) * fx) - top) * fy;
}

static void flame_texture_init(void)
{
	for (int x = 0; x < FLAME_TEX_W; ++x) {
		const float t = (float)x / (float)FLAME_TEX_W;

		/* Walk the ramp to the pair of stops this column falls between. */
		int k = 0;
		while (k + 2 < (int)(sizeof flame_ramp / sizeof *flame_ramp) &&
		       t > flame_ramp[k + 1].t)
			++k;
		const FlameStop &s0 = flame_ramp[k], &s1 = flame_ramp[k + 1];
		const float      m  = (t - s0.t) / (s1.t - s0.t);
		const float base[3] = {
			s0.r + (s1.r - s0.r) * m,
			s0.g + (s1.g - s0.g) * m,
			s0.b + (s1.b - s0.b) * m,
		};

		/* The nozzle burns clean and the tail is all turbulence, so the noise
		 * is weighted by how far down the plume the column is. */
		const float amp = 0.08f + 0.80f * t;

		for (int y = 0; y < FLAME_TEX_H; ++y) {
			const float v = (float)y / (float)FLAME_TEX_H;

			const float n = 0.55f * flame_noise(t *  6.0f, v *  4.0f,  6,  4)
			              + 0.30f * flame_noise(t * 12.0f, v *  8.0f, 12,  8)
			              + 0.15f * flame_noise(t * 24.0f, v * 16.0f, 24, 16);

			const float g = (1.0f - amp) + amp * (0.25f + 1.35f * n);

			int r = (int)(base[0] * g), gg = (int)(base[1] * g), b = (int)(base[2] * g);
			if (r  > 255) r  = 255;
			if (gg > 255) gg = 255;
			if (b  > 255) b  = 255;

			s_flame_tex[y * FLAME_TEX_W + x] =
				(uint16_t)(((r & 0xF8) << 8) | ((gg & 0xFC) << 3) | (b >> 3));
		}
	}
}

/*
 * Where a vertex lives in the buffer. The build walks engine, then ribbon, then
 * station, then the two edges, and the per-frame update indexes straight back in
 * on the same formula rather than searching or rebuilding.
 */
static inline int flame_index(int side, int ribbon, int station, int edge)
{
	return (((side * 2 + ribbon) * FLAME_STATIONS) + station) * 2 + edge;
}

/*
 * Shape one engine's plume for this frame.
 *
 * `burn` is how hard it is running, around 1.0. It drives length and width
 * together, because a real plume that surges gets longer and fatter at once -
 * scaling only one reads as a wobble rather than a flame. `phase` is where this
 * engine is sampling the texture's time axis.
 */
static void flame_shape(Object &flame, int side, float burn, int32_t phase, int32_t S)
{
	/* One engine sits on the centreline; two straddle it. */
	const int32_t zc = (g_plane->nozzles == 1)
	                 ? 0
	                 : (side ? g_plane->nozzleZ : -g_plane->nozzleZ);
	const int32_t z  = zc * S;

	const float rk = (float)g_plane->nozzleR / (float)FLAME_REF_R;

	for (int ribbon = 0; ribbon < 2; ++ribbon) {
		for (int i = 0; i < FLAME_STATIONS; ++i) {
			/* The nozzle mouth is welded to the aircraft and never moves; the
			 * surge grows from it, so the scale is weighted by how far aft the
			 * station is. */
			const int32_t x  = (int32_t)(g_plane->nozzleX - (int)(flame_aft[i] * burn)) * S;
			const float   wk = rk * (1.0f + (burn - 1.0f) * ((float)i / FLAME_STATIONS));
			const int32_t r  = (int32_t)(flame_half[i] * wk) * S;

			/* Along the plume, so the ramp lands where it belongs. 1000 rather
			 * than a full 1024 because U wraps, and the tip must not land back
			 * on the white-hot first column. */
			const int32_t u = (int32_t)flame_aft[i] * 1000 / flame_aft[FLAME_STATIONS - 1];

			for (int edge = 0; edge < 2; ++edge) {
				const int32_t d = edge ? r : -r;
				Object::Vertex &vx = flame.vertices[flame_index(side, ribbon, i, edge)];

				const int32_t y = (int32_t)g_plane->nozzleY * S;
				vx.position = ribbon == 0 ? Vector3{ x, y + d, z }
				                          : Vector3{ x, y,     z + d };

				/* A little spread across the ribbon so the two edges are at
				 * different moments and the turbulence is not a flat band. */
				vx.uv = { u, phase + (edge ? 150 : 0) };
			}
		}
	}
}

/*
 * Rebuild the plumes for the aircraft now loaded.
 *
 * Called on every change rather than once, because the engine count and the
 * nozzle position both move. The vectors keep whatever capacity they reached, so
 * this reshuffles indices and touches no allocator - which matters, because the
 * mesh next to it is being refilled at the same moment and a heap in pieces is
 * how the model viewer used to fail.
 *
 * An aircraft with no exhaust to show gets the object disabled rather than
 * removed: Jet skips a disabled object in one test, and taking it out of the
 * scene list and putting it back is churn for the same result.
 */
static void build_flames(Object &flame, Material *mat, int32_t S)
{
	flame.vertices.clear();
	flame.triangles.clear();

	flame.enabled = (g_plane->nozzles > 0);
	if (!flame.enabled)
		return;

	flame.vertices.resize((size_t)g_plane->nozzles * 2 * FLAME_STATIONS * 2);

	/* Nothing here is lit, so the normal is never read; it still has to be a
	 * legal value rather than zero, which some paths normalise. */
	for (auto &v : flame.vertices)
		v.normal = { 0, 1024, 0 };

	for (int side = 0; side < g_plane->nozzles; ++side) {
		flame_shape(flame, side, 1.0f, 0, S);

		for (int ribbon = 0; ribbon < 2; ++ribbon) {
			const uint16_t base = (uint16_t)flame_index(side, ribbon, 0, 0);
			for (int i = 0; i < FLAME_SEGMENTS; ++i) {
				const uint16_t a = (uint16_t)(base + i * 2);
				flame.addTriangle(a, (uint16_t)(a + 1), (uint16_t)(a + 3), mat);
				flame.addTriangle(a, (uint16_t)(a + 3), (uint16_t)(a + 2), mat);
			}
		}
	}

	/* Both ribbons are flat, and which way they face depends on the bank. */
	flame.cullingMode = CullingMode::NO_CULLING;
	flame.calculateBoundingBox();
}

/*
 * Make it burn, at time `t`.
 *
 * Two things move and they move at different rates. The texture scrolls, which
 * is the boil - combustion turbulence has no beat to it and reads as noise. The
 * surge underneath changes the plume's size.
 *
 * How fast either may go has a hard ceiling that is nothing to do with taste.
 * The flame is sampled once a frame, so at the forty-odd frames a second this
 * runs at, anything above about 20 Hz aliases and reads as random jitter rather
 * than as fire. The fastest term below is near 12 Hz - four samples a cycle,
 * about as quick as it can be driven and still look like it is burning.
 *
 * Everything is offset between the engines. In step it looks like the screen
 * brightness changing; out of step, one nozzle running long while the other sags
 * is what the eye reads as fire.
 */
static constexpr float FLAME_BOIL = 11000.0f;   /* texture time-axis units per second */

static void flames_update(Object &flame, float t, int32_t S)
{
	if (!flame.enabled)
		return;

	for (int side = 0; side < g_plane->nozzles; ++side) {
		const float p    = side ? 2.39f : 0.0f;
		const float surge = 0.58f * sinf(t * 44.0f + p)
		                  + 0.30f * sinf(t * 73.0f + p * 1.7f)
		                  + 0.12f * sinf(t * 19.0f + p * 0.6f);

		const int32_t phase = (int32_t)(t * FLAME_BOIL) + (side ? 512 : 0);

		flame_shape(flame, side, 1.0f + 0.34f * surge, phase, S);
	}

	flame.calculateBoundingBox();
}

/* -------------------------------------------------------------- the demo */

int main(void)
{
	set_sys_clock_khz(PICOJET_SYS_CLOCK_KHZ, true);
	stdio_init_all();

	printf("\npicojet flight: F22 over a cloud deck\n");
	printf("sys clock %u Hz\n", (unsigned)clock_get_hz(clk_sys));

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) != 0) {
		printf("SDL_Init failed: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Color band_fg = { 200, 220, 255, SDL_ALPHA_OPAQUE };
	SDL_Color band_bg = {   0,   0,   0, SDL_ALPHA_OPAQUE };
	PSDL_StatusBands(SDL_TRUE, band_fg, band_bg);
	PSDL_SetFooterText("github.com/anight/picojet");

	clouds_init();
	printf("picojet: clouds ready\n");

	/* ------------------------------------------------------ scene setup */

	Scene scene(g_framebuffer[0], nullptr, SCREEN_W, SCREEN_H);
	g_scene = &scene;

	/* The sky is the clear. Jet must not paint over it. */
	scene.setClearBuffer(false);

	/*
	 * The camera matches the cloud renderer's pinhole exactly, or the aircraft
	 * would not sit in the sky it is flying through: clouds.cpp projects with a
	 * focal length of 220 pixels, and Jet's setFOV() derives its own factor as
	 * (screenWidth/2) / tan(fov/2). Solving that for 220 over a 320-wide screen
	 * gives 2 * atan(160/220) = 72 degrees.
	 */
	g_camera.setFOV(72.0f, (int32_t)SCREEN_W);
	g_camera.nearPlane = 64;
	g_camera.farPlane  = 20000;
	scene.setCamera(&g_camera);

	/* Lit from the same side as the cloud renderer's sun, so the aircraft looks
	 * like it is in that sky rather than in front of a picture of one. */
	DirectionalLight sun(Vector3{40, 30, 0}, Color{255, 250, 240}, 255);
	AmbientLight     amb(Color{170, 178, 195});
	scene.setDirectionalLight(&sun);
	scene.setAmbientLight(&amb);

	/*
	 * One Texture and one Material for the aircraft, retargeted on each change
	 * rather than rebuilt. Every texture is in flash and Jet only reads texel
	 * data, so switching aircraft is three stores.
	 */
	Texture  texPlane(TEX_F22_W, TEX_F22_H,
	                  const_cast<uint16_t *>(tex_f22_data), false, 0, false, CLAMP);
	Material matPlane(0xFFFF, &texPlane);
	matPlane.shadingMode = ShadingMode::GOURAUD;
	matPlane.specular    = 48;

	const int32_t S = JET32_WORLD_SCALE;

	flame_texture_init();
	Texture  texFlame(FLAME_TEX_W, FLAME_TEX_H, s_flame_tex);
	Material matFlame(0xFFFF, &texFlame);
	matFlame.shadingMode = ShadingMode::UNLIT;

	Object plane;
	Object flame;

	/*
	 * Size both meshes for the largest aircraft in the table before loading any
	 * of them, so a change is a refill rather than an allocation. std::vector
	 * keeps its capacity across clear(), so after this nothing here asks the
	 * allocator for anything again, and cycling cannot leave the heap in pieces.
	 */
	{
		int maxVerts = 0, maxTris = 0;
		for (int i = 0; i < PLANE_COUNT; ++i) {
			if (g_planes[i].vertCount > maxVerts) maxVerts = g_planes[i].vertCount;
			if (g_planes[i].triCount  > maxTris)  maxTris  = g_planes[i].triCount;
		}
		plane.vertices.reserve((size_t)maxVerts);
		plane.triangles.reserve((size_t)maxTris);
		flame.vertices.reserve(2 * 2 * FLAME_STATIONS * 2);
		flame.triangles.reserve(2 * 2 * FLAME_SEGMENTS * 2);
	}

	scene.addObject(&plane);
	scene.addObject(&flame);

	auto load_plane = [&](int index) {
		g_plane = &g_planes[index];

		texPlane.width  = g_plane->texW;
		texPlane.height = g_plane->texH;
		texPlane.data   = const_cast<uint16_t *>(g_plane->tex);

		plane.vertices.clear();
		plane.triangles.clear();
		for (int i = 0; i < g_plane->vertCount; ++i) {
			const int16_t *v = g_plane->verts[i];
			Object::Vertex vert;
			vert.position = { (int32_t)v[0] * S, (int32_t)v[1] * S, (int32_t)v[2] * S };
			vert.uv       = { (int32_t)v[3], (int32_t)v[4] };
			vert.normal   = { (int32_t)v[5], (int32_t)v[6], (int32_t)v[7] };
			plane.addVertex(vert);
		}
		for (int i = 0; i < g_plane->triCount; ++i)
			plane.addTriangle(g_plane->tris[i][0], g_plane->tris[i][1],
			                  g_plane->tris[i][2], &matPlane);
		plane.calculateBoundingBox();

		build_flames(flame, &matFlame, S);

		printf("picojet: %s - %d verts, %d tris, %d plume%s | heap %u KB\n",
		       g_plane->name, g_plane->vertCount, g_plane->triCount,
		       g_plane->nozzles, g_plane->nozzles == 1 ? "" : "s",
		       (unsigned)(mallinfo().uordblks / 1024));
	};

	/*
	 * Grow Jet's render queue on the biggest aircraft before anything else has
	 * been asked of the heap.
	 *
	 * The queue holds every submitted triangle and needs one contiguous block.
	 * Grown first it reaches the size the largest mesh needs and keeps it;
	 * grown after a few changes it makes a single large request into a heap
	 * that has been handing out and taking back meshes of assorted sizes, and
	 * fails with plenty free and none of it adjacent.
	 */
	int planeIndex = 0;
	{
		int biggest = 0;
		for (int i = 1; i < PLANE_COUNT; ++i)
			if (g_planes[i].triCount > g_planes[biggest].triCount)
				biggest = i;

		printf("picojet: warming the render queue on the %s, %d triangles\n",
		       g_planes[biggest].name, g_planes[biggest].triCount);

		load_plane(biggest);
		plane.setRotation(0, PLANE_YAW_DEG, 0);
		plane.setPosition(0, 0, PLANE_Z);
		flame.setRotation(0, PLANE_YAW_DEG, 0);
		flame.setPosition(0, 0, PLANE_Z);
		g_camera.setPosition(0, CAM_HEIGHT, 0);
		g_camera.setRotation(0, 0, 0);
		scene.setFramebuffer(g_framebuffer[0]);
		scene.prepareFrame();

		printf("picojet: queue warmed, %d triangles | heap %u KB\n",
		       scene.lastFrameDrawnTriangles, (unsigned)(mallinfo().uordblks / 1024));

		if (scene.lastFrameDrawnTriangles > MAX_QUEUED_TRIS) {
			printf("picojet: %d queued triangles exceeds MAX_QUEUED_TRIS (%d)\n",
			       scene.lastFrameDrawnTriangles, MAX_QUEUED_TRIS);
			fflush(stdout);
			for (;;) { }
		}

		load_plane(planeIndex);
	}

	multicore_launch_core1(core1_worker);
	printf("picojet: core 1 on rows %d..%d\n", BAND_SPLIT, SCREEN_H);

	/* ---------------------------------------------------------- the input */

	SDL_Joystick       *joystick   = NULL;
	SDL_GameController *controller = NULL;

	if (SDL_IsGameController(0))
		controller = SDL_GameControllerOpen(0);
	if (controller == NULL && SDL_NumJoysticks() > 0)
		joystick = SDL_JoystickOpen(0);

	printf("picojet: input - %s\n",
	       controller ? "gamepad" : (joystick ? "joystick" : "none"));

	/* ------------------------------------------------------- frame loop */

	float  burn_t      = 0.0f;   /* the afterburner's own clock */
	int    button_was_down = 0;
	float  yaw         = 0.0f;   /* heading, radians */
	float  bank        = 0.0f;   /* what the aeroplane shows for it, degrees */
	int    back_index  = 0;
	Uint32 last_ms     = SDL_GetTicks();
	int    frames      = 0;
	Uint32 fps_mark_ms = last_ms;

	for (bool running = true; running; ) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			if (ev.type == SDL_QUIT)
				running = false;
			else if (ev.type == SDL_KEYDOWN &&
			         ev.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
				running = false;
		}

		Uint32 now_ms = SDL_GetTicks();
		float  dt     = (now_ms - last_ms) / 1000.0f;
		last_ms       = now_ms;
		if (dt > 0.1f)
			dt = 0.1f;

		/*
		 * Left and right only. The deadzone is wide because neither the ADC
		 * stick nor the pad rests at exactly zero, and a heading that drifts on
		 * its own is worse than one that needs a firm push.
		 */
		int ax = 0;
		if (controller)
			ax = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
		else if (joystick)
			ax = SDL_JoystickGetAxis(joystick, 0);

		const int DEADZONE = 6000;
		float stick = 0.0f;
		if (ax > DEADZONE || ax < -DEADZONE)
			stick = (float)ax / 32768.0f;

		/*
		 * Next aircraft. A on a pad, or the stick's own click when that is all
		 * there is - picosdl reports the analog stick as a plain joystick, and
		 * pressing it down is button 0.
		 *
		 * On the edge, not the level, or holding it would run through the whole
		 * table in a fraction of a second.
		 */
		int button = 0;
		if (controller)
			button = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A);
		else if (joystick)
			button = SDL_JoystickGetButton(joystick, 0);

		if (button && !button_was_down) {
			planeIndex = (planeIndex + 1) % PLANE_COUNT;
			load_plane(planeIndex);
		}
		button_was_down = button;

		/*
		 * The stick commands a bank, and the aircraft rolls toward it rather
		 * than snapping to it. This is the whole of the inertia: a first-order
		 * lag, which is what a roll rate against a roll command actually is.
		 */
		const float wanted = stick * -BANK_MAX;
		bank += (wanted - bank) * (1.0f - expf(-dt * ROLL_RESPONSE));

		/*
		 * And the bank is what turns it. Yaw rate comes from the bank the
		 * aircraft is actually holding, not from the stick, so the lag above is
		 * the only thing that needs modelling - the heading inherits it.
		 *
		 * That is also why it feels right rather than merely slow. Push the
		 * stick and the aircraft rolls before it goes anywhere; centre it and
		 * the roll unwinds and the turn washes out with it, instead of the sky
		 * stopping dead under a wing that is still down.
		 */
		yaw += (bank / -BANK_MAX) * TURN_RATE * dt;

		/*
		 * The aircraft is held in the middle of the screen, nose into the
		 * distance, and only its bank changes. The world turns around it.
		 *
		 * The mesh's long axis is X, so PLANE_YAW_DEG turns the nose down +Z, away
		 * from the camera. The bank is then a rotation about that same Z.
		 */
		plane.setRotation(0, PLANE_YAW_DEG, (int32_t)bank);
		plane.setPosition(0, 0, PLANE_Z);

		/*
		 * The plumes are built in the aircraft's local frame, so they follow it
		 * by being given the same transform - no parenting, which Jet's Object
		 * does not have, and no per-frame rebuild of the geometry.
		 */
		flame.setRotation(0, PLANE_YAW_DEG, (int32_t)bank);
		flame.setPosition(0, 0, PLANE_Z);

		/* A flame that holds still looks painted on. */
		burn_t += dt;
		flames_update(flame, burn_t, S);

		/*
		 * The camera sits above and behind, and stays LEVEL - it is raised, never
		 * pitched. Pitching it would be the obvious way to look down at the
		 * aircraft, and it would break the sky: the cloud renderer pins its
		 * horizon to the middle row and has no notion of pitch, so tilting Jet's
		 * camera would slide the two apart and the aircraft would fly through a
		 * sky that disagreed with it.
		 *
		 * Level and raised, both still look down +Z, so their vanishing point is
		 * the same pixel - the centre of the screen, where the horizon is - and
		 * the aircraft simply hangs below that line.
		 */
		g_camera.setPosition(0, CAM_HEIGHT, 0);
		g_camera.setRotation(0, 0, 0);

		/* ----------------------------------------------------- the frame */

		uint16_t *back = g_framebuffer[back_index];
		while (PSDL_BufferBusy(back)) { }

		g_back = back;
		scene.setFramebuffer(back);

		clouds_begin_frame(yaw, dt);

		/* render() runs prepareFrame(), then the executor above - which lays the
		 * sky down on both cores before either draws a triangle over it - then
		 * the post-process and sprite passes. */
		scene.render(raster_executor);

		PSDL_PresentBuffer(back, SCREEN_W, SCREEN_H,
		                   SCREEN_W * (int)sizeof(uint16_t));
		back_index ^= 1;

		++frames;
		if (now_ms - fps_mark_ms >= 1000) {
			printf("picojet: %u fps, %s, heading %4d deg, bank %3d, %d tris\n",
			       (unsigned)(frames * 1000 / (now_ms - fps_mark_ms)),
			       g_plane->name,
			       ((int)(yaw * 57.2958f) % 360 + 360) % 360,
			       (int)bank, scene.lastFrameDrawnTriangles);
			frames      = 0;
			fps_mark_ms = now_ms;
		}
	}

	if (controller) SDL_GameControllerClose(controller);
	if (joystick)   SDL_JoystickClose(joystick);

	printf("picojet: stopped\n");
	SDL_Quit();
	return 0;
}
