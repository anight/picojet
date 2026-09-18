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

using namespace Renderer;

#ifndef PICOJET_SYS_CLOCK_KHZ
#define PICOJET_SYS_CLOCK_KHZ PSDL_PICO_SYS_CLOCK_KHZ
#endif

/*
 * Which way the mesh's nose points, as a yaw in degrees.
 *
 * f22.obj's long axis is X with the nose toward +X, and this turns it to put the
 * nose down +Z - away from the camera, so the aircraft flies into the distance
 * rather than at the viewer.
 */
static constexpr int32_t PLANE_YAW_DEG = 270;

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
/* ---- PROFILE ---- */
volatile uint32_t pf_c1_wait, pf_c1_clouds, pf_c1_tris;
volatile uint32_t pf_exec, pf_c0_clouds, pf_c0_tris, pf_join;

static void core1_worker(void)
{
	for (;;) {
		/* Waiting for a frame is all this core does between frames, and it is
		 * the only thing picosdl can be told about it - everything outside the
		 * two brackets is the half-frame this core actually renders. */
		absolute_time_t w0 = get_absolute_time();
		PSDL_CpuIdle();
		uint32_t n_tris = multicore_fifo_pop_blocking();
		PSDL_CpuBusy();
		absolute_time_t w1 = get_absolute_time();

		clouds_render_band(g_back, SCREEN_W, BAND_SPLIT, SCREEN_H);
		absolute_time_t w2 = get_absolute_time();

		memset(g_flags_core1, 0, n_tris);
		g_scene->rasterizeBand(BAND_SPLIT, SCREEN_H, g_flags_core1);
		absolute_time_t w3 = get_absolute_time();

		pf_c1_wait   += (uint32_t)absolute_time_diff_us(w0, w1);
		pf_c1_clouds += (uint32_t)absolute_time_diff_us(w1, w2);
		pf_c1_tris   += (uint32_t)absolute_time_diff_us(w2, w3);

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

	absolute_time_t e0 = get_absolute_time();
	clouds_render_band(g_back, SCREEN_W, 0, BAND_SPLIT);
	absolute_time_t e1 = get_absolute_time();

	memset(g_flags_core0, 0, queued);
	scene.rasterizeBand(0, BAND_SPLIT, g_flags_core0);
	absolute_time_t e2 = get_absolute_time();

	PSDL_CpuIdle();
	(void)multicore_fifo_pop_blocking();      /* core 1 has finished its half */
	PSDL_CpuBusy();
	absolute_time_t e3 = get_absolute_time();

	pf_c0_clouds += (uint32_t)absolute_time_diff_us(e0, e1);
	pf_c0_tris   += (uint32_t)absolute_time_diff_us(e1, e2);
	pf_join      += (uint32_t)absolute_time_diff_us(e2, e3);
	pf_exec      += (uint32_t)absolute_time_diff_us(e0, e3);

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
 * Distance aft of the nozzle, and the half-width there, at full burn. The last
 * is not zero because a zero-width quad is a degenerate triangle, and 3 units is
 * a point.
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

static constexpr int16_t FLAME_X = -200;   /* nozzle face, local units */
static constexpr int16_t FLAME_Y =  -25;
static constexpr int16_t FLAME_Z =   26;   /* and its mirror */

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
	const int32_t z = (side ? FLAME_Z : -FLAME_Z) * S;

	for (int ribbon = 0; ribbon < 2; ++ribbon) {
		for (int i = 0; i < FLAME_STATIONS; ++i) {
			/* The nozzle mouth is welded to the aircraft and never moves; the
			 * surge grows from it, so the scale is weighted by how far aft the
			 * station is. */
			const int32_t x  = (int32_t)(FLAME_X - (int)(flame_aft[i] * burn)) * S;
			const float   wk = 1.0f + (burn - 1.0f) * ((float)i / FLAME_STATIONS);
			const int32_t r  = (int32_t)(flame_half[i] * wk) * S;

			/* Along the plume, so the ramp lands where it belongs. 1000 rather
			 * than a full 1024 because U wraps, and the tip must not land back
			 * on the white-hot first column. */
			const int32_t u = (int32_t)flame_aft[i] * 1000 / flame_aft[FLAME_STATIONS - 1];

			for (int edge = 0; edge < 2; ++edge) {
				const int32_t d = edge ? r : -r;
				Object::Vertex &vx = flame.vertices[flame_index(side, ribbon, i, edge)];

				vx.position = ribbon == 0
				            ? Vector3{ x, (int32_t)FLAME_Y * S + d, z }
				            : Vector3{ x, (int32_t)FLAME_Y * S,     z + d };

				/* A little spread across the ribbon so the two edges are at
				 * different moments and the turbulence is not a flat band. */
				vx.uv = { u, phase + (edge ? 150 : 0) };
			}
		}
	}
}

/*
 * Build both plumes. The material is the caller's because it has to outlive this.
 */
static void build_flames(Object &flame, Material *mat, int32_t S)
{
	flame.vertices.resize(2 * 2 * FLAME_STATIONS * 2);
	flame.triangles.reserve(2 * 2 * FLAME_SEGMENTS * 2);

	/* Nothing here is lit, so the normal is never read; it still has to be a
	 * legal value rather than zero, which some paths normalise. */
	for (auto &v : flame.vertices)
		v.normal = { 0, 1024, 0 };

	for (int side = 0; side < 2; ++side) {
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
	for (int side = 0; side < 2; ++side) {
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

	Texture  texF22(TEX_F22_W, TEX_F22_H,
	                const_cast<uint16_t *>(tex_f22_data), false, 0, false, CLAMP);
	Material matF22(0xFFFF, &texF22);
	matF22.shadingMode = ShadingMode::GOURAUD;
	matF22.specular    = 48;

	const int32_t S = JET32_WORLD_SCALE;

	Object plane;
	plane.vertices.reserve(MESH_F22_VERTS);
	plane.triangles.reserve(MESH_F22_TRIS);
	for (int i = 0; i < MESH_F22_VERTS; ++i) {
		const int16_t *v = mesh_f22_verts[i];
		Object::Vertex vert;
		vert.position = { (int32_t)v[0] * S, (int32_t)v[1] * S, (int32_t)v[2] * S };
		vert.uv       = { (int32_t)v[3], (int32_t)v[4] };
		vert.normal   = { (int32_t)v[5], (int32_t)v[6], (int32_t)v[7] };
		plane.addVertex(vert);
	}
	for (int i = 0; i < MESH_F22_TRIS; ++i)
		plane.addTriangle(mesh_f22_tris[i][0], mesh_f22_tris[i][1],
		                  mesh_f22_tris[i][2], &matF22);
	plane.calculateBoundingBox();
	scene.addObject(&plane);

	flame_texture_init();
	Texture  texFlame(FLAME_TEX_W, FLAME_TEX_H, s_flame_tex);
	Material matFlame(0xFFFF, &texFlame);
	matFlame.shadingMode = ShadingMode::UNLIT;

	Object flame;
	build_flames(flame, &matFlame, S);
	scene.addObject(&flame);

	{
		struct mallinfo mi = mallinfo();
		printf("picojet: F22 %d verts, %d tris | heap %u KB used\n",
		       MESH_F22_VERTS, MESH_F22_TRIS, (unsigned)(mi.uordblks / 1024));
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
	float  yaw         = 0.0f;   /* heading, radians */
	float  bank        = 0.0f;   /* what the aeroplane shows for it, degrees */
	int    back_index  = 0;
	Uint32 last_ms     = SDL_GetTicks();
	int    frames      = 0;
	Uint32 fps_mark_ms = last_ms;

	uint32_t pf_input=0, pf_bufwait=0, pf_cb=0, pf_render=0, pf_present=0;
	absolute_time_t pf_mark = get_absolute_time();
	for (bool running = true; running; ) {
		absolute_time_t p0 = get_absolute_time();
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

		absolute_time_t p1 = get_absolute_time();
		uint16_t *back = g_framebuffer[back_index];
		while (PSDL_BufferBusy(back)) { }
		absolute_time_t p2 = get_absolute_time();

		g_back = back;
		scene.setFramebuffer(back);

		clouds_begin_frame(yaw, dt);
		absolute_time_t p3 = get_absolute_time();

		/* render() runs prepareFrame(), then the executor above - which lays the
		 * sky down on both cores before either draws a triangle over it - then
		 * the post-process and sprite passes. */
		scene.render(raster_executor);
		absolute_time_t p4 = get_absolute_time();

		PSDL_PresentBuffer(back, SCREEN_W, SCREEN_H,
		                   SCREEN_W * (int)sizeof(uint16_t));
		back_index ^= 1;
		absolute_time_t p5 = get_absolute_time();

		pf_input   += (uint32_t)absolute_time_diff_us(p0, p1);
		pf_bufwait += (uint32_t)absolute_time_diff_us(p1, p2);
		pf_cb      += (uint32_t)absolute_time_diff_us(p2, p3);
		pf_render  += (uint32_t)absolute_time_diff_us(p3, p4);
		pf_present += (uint32_t)absolute_time_diff_us(p4, p5);

		++frames;
		if (now_ms - fps_mark_ms >= 1000) {
			printf("picojet: %u fps, heading %4d deg, bank %3d, %d tris\n",
			       (unsigned)(frames * 1000 / (now_ms - fps_mark_ms)),
			       ((int)(yaw * 57.2958f) % 360 + 360) % 360,
			       (int)bank, scene.lastFrameDrawnTriangles);
			uint32_t wall = (uint32_t)absolute_time_diff_us(pf_mark, get_absolute_time());
			pf_mark = get_absolute_time();
			uint32_t f = frames ? frames : 1;
			uint32_t prep = pf_render - pf_exec;
			uint32_t c0busy = pf_input + pf_cb + prep + pf_c0_clouds + pf_c0_tris + pf_present;
			uint32_t c1busy = pf_c1_clouds + pf_c1_tris;
			printf("  CORE0 %2u%%: input %4u cloudsbegin %3u prepare %4u clouds %5u tris %4u present %3u | IDLE bufwait %3u join %5u\n",
			       (unsigned)(100u * c0busy / wall), (unsigned)(pf_input / f),
			       (unsigned)(pf_cb / f), (unsigned)(prep / f),
			       (unsigned)(pf_c0_clouds / f), (unsigned)(pf_c0_tris / f),
			       (unsigned)(pf_present / f), (unsigned)(pf_bufwait / f),
			       (unsigned)(pf_join / f));
			printf("  CORE1 %2u%%: clouds %5u tris %5u | IDLE token %5u    (frame %u us over %u)\n",
			       (unsigned)(100u * c1busy / wall), (unsigned)(pf_c1_clouds / f),
			       (unsigned)(pf_c1_tris / f), (unsigned)(pf_c1_wait / f),
			       (unsigned)(wall / f), (unsigned)wall);
			pf_input=pf_bufwait=pf_cb=pf_render=pf_present=0;
			pf_exec=pf_c0_clouds=pf_c0_tris=pf_join=0;
			pf_c1_wait=pf_c1_clouds=pf_c1_tris=0;
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
