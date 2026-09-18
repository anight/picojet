/*
 * picojet: the Jet software rasteriser on an RP2350, through picosdl.
 *
 * Jet is a fixed-function 3D rasteriser that owns no display and no main loop -
 * it fills a framebuffer and stops. picosdl owns the panel, the input and the
 * clock. This file is the whole of what sits between them, and it is short for
 * one reason worth stating up front:
 *
 *     Jet renders RGB565. The ST7789 wants RGB565. Nothing converts anything.
 *
 * picosdl is built here at PSDL_COLOR_DEPTH=16, where a pixel is an RGB565 value
 * and the display driver's DMA reads the client's framebuffer and hands it to the
 * panel without anything in between. Jet writes into that framebuffer directly,
 * so no pass converts, repacks or copies a pixel between the rasteriser and the
 * glass.
 *
 * picosdl allocates no framebuffer for anyone, so the ones below are ours, and
 * there are two of them: Jet rasterises into the back one while the DMA pushes
 * the front, which is worth most of the frame rate here. The panel push is
 * 17.4 ms of a frame that takes about the same again to render, so overlapping
 * them takes this from 30 fps to 56 - a hair under the 57 the panel link allows
 * if nothing else took any time at all.
 *
 * No window is created because nothing here wants an SDL_Surface: there is no
 * blitting to do over a frame Jet has already finished.
 *
 * The other thing this file does is use both cores. picosdl's core 1 is its
 * audio mixer, but this build has PICOSDL_AUDIO=OFF and so core 1 is free; Jet
 * splits a frame into y-bands that can be rasterised independently when
 * Z_BUFFERING is off, which it is. Core 0 takes the top half, core 1 the bottom.
 */
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "SDL2/SDL.h"
#include "psdl_pico.h"

#include "Jet.hpp"
#include "models.hpp"

using namespace Renderer;

/*
 * System clock, set by the build (see CMakeLists.txt) and falling back to the
 * board's own value.
 *
 * The ST7789 driver clocks SCK from PIO sideset at clkdiv 1, so SCK is sysclk/2
 * always and the panel's rated maximum of 62.5 MHz means 125 MHz is the highest
 * clock with nothing out of spec. Above that the panel is overclocked in
 * proportion, which a given panel may or may not tolerate; the symptom is torn,
 * speckled or shifted pixels rather than a blank screen.
 *
 * What it buys is the frame, because the frame is bound by that link rather than
 * by the processor: the push is 64000 pixels x 34 cycles / sysclk.
 */
#ifndef PICOJET_SYS_CLOCK_KHZ
#define PICOJET_SYS_CLOCK_KHZ PSDL_PICO_SYS_CLOCK_KHZ
#endif

/* The canvas. 320x200 letterboxed into the 320x240 panel by picosdl, leaving the
 * two 20-pixel bands it uses for the frame rate and the footer. */
static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 200;

/*
 * The framebuffers, and the largest thing in the program: 125 KB each.
 *
 * picosdl allocates no pixels, so these are ours. How many there are is the one
 * decision that changes the shape of the frame loop, and it is worth stating what
 * each costs.
 *
 * ONE. The buffer Jet rasterises into is the buffer the DMA is reading, so the
 * two cannot overlap: the frame is a push followed by a render, serially, and the
 * loop opens by waiting for the panel. Costs 125 KB and roughly half the frame
 * rate.
 *
 * TWO. Jet rasterises into the back buffer while the panel reads the front, so
 * the push costs nothing that the render was not already spending. Costs 250 KB
 * and is the default, because on this board there is room.
 *
 * Whether there is room is the whole question, and the answer is measured rather
 * than assumed: with two buffers the image is 368 KB of .bss against a 512 KB
 * SRAM region, leaving 88 KB clear of the 64 KB heap. Both core stacks are in
 * SCRATCH_X and SCRATCH_Y and come out of neither.
 *
 * No z-buffer either way: JetConfig.hpp has Z_BUFFERING off and SORT_TRIANGLES
 * on, so depth is resolved by sorting rather than per-pixel. That saves another
 * 125 KB - which is most of what pays for the second framebuffer - and is also
 * what makes the two-core split below legal.
 */
#ifndef PICOJET_FRAMEBUFFERS
#define PICOJET_FRAMEBUFFERS 2
#endif
#if PICOJET_FRAMEBUFFERS != 1 && PICOJET_FRAMEBUFFERS != 2
#error "PICOJET_FRAMEBUFFERS must be 1 or 2"
#endif

static uint16_t g_framebuffer[PICOJET_FRAMEBUFFERS][SCREEN_W * SCREEN_H];

/* ---------------------------------------------------------- model loading */

/*
 * How much heap one model costs once loaded.
 *
 * Jet's Object owns std::vectors, so building a mesh copies it out of flash into
 * RAM. Both halves are not equal in nature, though the arithmetic does not care:
 * the vertices are only ever read while rendering, but the triangles are sorted
 * in place every frame when SORT_TRIANGLES is on, so they have to be writable.
 * That is what stops the mesh being used straight out of flash.
 */
/*
 * On top of the mesh, Scene builds a render queue every frame whose entries carry
 * three expanded vertices each. Measured on this build it costs far more than the
 * mesh does - about 160 bytes per triangle of the model, against 78 for the mesh
 * itself - so a model's real demand is dominated by the part that is not the
 * model.
 *
 * Measured rather than derived: f117's 134 triangles take the heap from 14 KB to
 * 49 KB with a 14 KB mesh, and f22's 200 take it to 65 KB with a 23 KB mesh.
 */
static constexpr size_t QUEUE_BYTES_PER_TRI = 160;

/*
 * Triangles in the scene that are not the model. Zero as the scene stands.
 *
 * The render queue is per FRAME rather than per object: it holds everything
 * submitted, so anything else in the scene spends queue before the model gets
 * any, and a budget that counts only the model is not a budget. A backdrop is
 * easily several hundred triangles on its own.
 */
static int g_sceneTris;

static size_t model_ram(const ModelDef &m)
{
	return (size_t)m.vertCount * sizeof(Object::Vertex)
	     + (size_t)m.triCount  * sizeof(Object::Triangle)
	     + (size_t)(m.triCount + g_sceneTris) * QUEUE_BYTES_PER_TRI;
}

/*
 * What a model is allowed to cost: the 140 KB heap, less what the rest of the
 * scene holds permanently, less a margin.
 *
 * A hard gate rather than an attempt-and-recover, because there is no
 * recovering: with exceptions compiled out, a failed allocation inside
 * std::vector does not return, it panics. Checking first is the only way to
 * decline politely.
 */
static constexpr size_t MODEL_RAM_BUDGET = 145 * 1024;

/*
 * Build a Jet Object from the flash arrays.
 *
 * The vertex data is int16 to keep the flash image small; Jet wants its fixed
 * point, so positions are scaled by JET32_WORLD_SCALE here and UVs and normals
 * are already in it. reserve() first because addVertex() would otherwise grow the
 * vector by doubling, and a transient copy of a large mesh is exactly what this
 * heap cannot afford.
 */
static Object *build_model(const ModelDef &m, Material *material, int32_t worldScale)
{
	Object *o = new Object();
	o->vertices.reserve(m.vertCount);
	o->triangles.reserve(m.triCount);

	for (int i = 0; i < m.vertCount; ++i) {
		const int16_t *v = m.verts[i];
		Object::Vertex vert;
		vert.position = { (int32_t)v[0] * worldScale,
		                  (int32_t)v[1] * worldScale,
		                  (int32_t)v[2] * worldScale };
		vert.uv       = { (int32_t)v[3], (int32_t)v[4] };
		vert.normal   = { (int32_t)v[5], (int32_t)v[6], (int32_t)v[7] };
		o->addVertex(vert);
	}

	for (int i = 0; i < m.triCount; ++i)
		o->addTriangle(m.tris[i][0], m.tris[i][1], m.tris[i][2], material);

	o->calculateBoundingBox();
	return o;
}

/* ------------------------------------------------------------- the scene */

static Scene  *g_scene;
static Camera  g_camera;

/*
 * Per-band triangle flags for the parallel raster.
 *
 * Scene::rasterizeBand() writes lastFrameRasterizedTriangles when it is not
 * given a flags array, so two cores calling it with nullptr would both write one
 * int. Handing each core its own array is the documented way to run it in
 * parallel and removes the shared write entirely; ORing them afterwards counts
 * each triangle once even though both bands may have drawn part of it.
 *
 * Sized for the scene below with room to spare. Jet queues one entry per
 * surviving triangle, and prepareFrame() reports how many - checked each frame
 * rather than assumed, because overflowing this would be a silent out-of-bounds
 * write into whatever follows.
 */
static constexpr int MAX_QUEUED_TRIS = 2048;
static uint8_t g_flags_core0[MAX_QUEUED_TRIS];
static uint8_t g_flags_core1[MAX_QUEUED_TRIS];

/*
 * Core 1: rasterise the bottom half of whatever frame core 0 has prepared.
 *
 * The handshake is the inter-core FIFO rather than a flag in memory, because it
 * is a hardware mailbox with the ordering already guaranteed - core 0's writes
 * to the render queue are visible to core 1 before the token it pops, and core
 * 1's writes to the framebuffer are visible to core 0 after the token it pushes
 * back. A volatile flag would need explicit barriers to say the same thing.
 *
 * Safe because Z_BUFFERING is 0: the two bands write disjoint rows of the
 * framebuffer and share nothing else. Scene::rasterizeBand() takes its own copy
 * of the rasteriser so the y-band clip is per-core, and everything it reads -
 * the render queue, the materials, the lights - is const for the duration.
 */
static void core1_raster_loop(void)
{
	for (;;) {
		/* Waiting for a frame is all this core does between frames, and it is
		 * the only thing picosdl can be told about it - everything outside the
		 * two brackets is the half-frame this core actually renders. */
		PSDL_CpuIdle();
		uint32_t n_tris = multicore_fifo_pop_blocking();
		PSDL_CpuBusy();

		memset(g_flags_core1, 0, n_tris);
		g_scene->rasterizeBand(SCREEN_H / 2, SCREEN_H, g_flags_core1);

		multicore_fifo_push_blocking(1);
	}
}

/* How many triangles the last executor pass counted, for the console line. */
static int g_last_rasterized;

/*
 * The raster pass, through Jet's own executor hook.
 *
 * Scene::render() calls prepareFrame(), then this, then PostFX, then the 2D
 * sprite pass. Driving prepareFrame()/rasterizeBand() directly instead skips the
 * last two entirely, so a registered Sprite2D silently never draws and no post
 * effect runs. The hook exists precisely so a frontend can parallelise the middle
 * step without opting out of the rest, and its contract is the one thing it has
 * to honour: join the workers and publish the statistics before returning.
 */
static void raster_executor(Scene &scene)
{
	const int queued = scene.lastFrameDrawnTriangles;

	if (queued > MAX_QUEUED_TRIS) {
		/* rasterizeBand() indexes the flags array by queue position, so a queue
		 * longer than the array is an out-of-bounds write into whatever follows
		 * it - silent, and a long way from its cause. Stop instead, loudly, the
		 * way picosdl treats one of its fixed pools running out: the sizing is a
		 * decision this program made and got wrong, not a runtime condition to
		 * recover from. */
		printf("picojet: %d queued triangles exceeds MAX_QUEUED_TRIS (%d) - "
		       "raise it and rebuild\n", queued, MAX_QUEUED_TRIS);
		fflush(stdout);
		for (;;) { }
	}

	multicore_fifo_push_blocking((uint32_t)queued);

	memset(g_flags_core0, 0, queued);
	scene.rasterizeBand(0, SCREEN_H / 2, g_flags_core0);

	PSDL_CpuIdle();
	(void)multicore_fifo_pop_blocking();      /* core 1's band has landed */
	PSDL_CpuBusy();

	/* Published before returning, per the executor contract. Neither core wrote
	 * lastFrameRasterizedTriangles - both were given a flags array - so the
	 * union is counted here, where each triangle is counted once even though a
	 * triangle crossing the seam was drawn by both. */
	int rasterized = 0;
	for (int i = 0; i < queued; ++i)
		if (g_flags_core0[i] | g_flags_core1[i])
			++rasterized;

	scene.lastFrameRasterizedTriangles = rasterized;
	g_last_rasterized                  = rasterized;
}

/* ------------------------------------------------------------- the demo */

int main(void)
{
	/* Before stdio_init_all(), always: set_sys_clock_khz() re-parents clk_peri,
	 * and stdio derives the UART divisor from it when it starts. The other order
	 * leaves the console at the wrong baud rate. */
	set_sys_clock_khz(PICOJET_SYS_CLOCK_KHZ, true);
	stdio_init_all();

	printf("\npicojet: Jet on picosdl\n");
	printf("sys clock %u Hz\n", (unsigned)clock_get_hz(clk_sys));


	/*
	 * No SDL_INIT_AUDIO: this build has none, and core 1 is the raster worker
	 * instead of the mixer. Video still gets initialised - that is what brings
	 * the panel up and claims the display's DMA channels, which has to happen
	 * before anything else asks the SDK for a channel.
	 */
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) != 0) {
		printf("SDL_Init failed: %s\n", SDL_GetError());
		return 1;
	}

	/* No PSDL_CreateWindow() here: that exists to wrap a caller's buffer in an
	 * SDL_Surface so the blitters can reach it, and nothing in this program blits.
	 * The back buffer goes straight to PSDL_PresentBuffer() at the end of each
	 * frame. */

	SDL_Color band_fg = { 200, 220, 255, SDL_ALPHA_OPAQUE };
	SDL_Color band_bg = {   0,   0,   0, SDL_ALPHA_OPAQUE };
	PSDL_StatusBands(SDL_TRUE, band_fg, band_bg);
	PSDL_SetFooterText("picojet - Jet 3D on RP2350");

	/* ------------------------------------------------------ scene setup */

	Scene scene(g_framebuffer[0], nullptr, SCREEN_W, SCREEN_H);
	g_scene = &scene;

	/* 40% grey. RGB565 packs 5/6/5 bits, so 0x632C is 12/31, 25/63, 12/31 -
	 * 38.7%, 39.7%, 38.7% - which is as close to neutral 40% as the format gets. */
	scene.setBackcolor(0x632C);
	scene.setClearBuffer(true);

	/*
	 * 70 degrees HORIZONTALLY, whatever Camera.hpp's comment says.
	 *
	 * setFOV() computes fovFactor = (screenWidth/2) / tan(fov/2) and projects
	 * both axes through it, so the angle given is across the width and the
	 * vertical field follows from the aspect: at 320x200 that is 47 degrees, not
	 * 70. Worth knowing before sizing anything to fit the screen - the vertical
	 * half-angle is 23.6 degrees, so the height is the tight axis by a wide
	 * margin, and models are scaled in convert_assets.py to fit it.
	 */
	g_camera.setFOV((int32_t)70, (int32_t)SCREEN_W);
	g_camera.nearPlane = 128;
	g_camera.farPlane  = 4096 * JET32_WORLD_SCALE;
	scene.setCamera(&g_camera);

	/*
	 * Lit for textured models rather than for flat-coloured primitives.
	 *
	 * Jet modulates the sampled texel by the lighting, so a texture that is
	 * already mid-toned goes muddy under an ambient term meant to let bright
	 * material colours show shape. The ambient is most of what makes a face
	 * pointing away from the sun readable at all - with it low, half of every
	 * model was near black - so it carries the base exposure here and the
	 * directional supplies the shaping on top.
	 */
	DirectionalLight sun(Vector3{45, 35, 0}, Color{255, 250, 240}, 255);
	AmbientLight     amb(Color{205, 208, 215});
	scene.setDirectionalLight(&sun);
	scene.setAmbientLight(&amb);




	const int32_t S = JET32_WORLD_SCALE;


	/*
	 * Nothing in the scene but the model.
	 *
	 * A ground plane is tempting and expensive: a 10x10 textured grid is 540
	 * triangles, which is more render queue than most of these models need, and
	 * the queue is per frame and shared. Spending it on a backdrop roughly thirds
	 * what a model is allowed to be. When the model is the point, the model
	 * should have the memory.
	 *
	 * With nothing to stand on, models are centred on the origin and the camera
	 * looks there - no per-model height to correct for.
	 */

	/* ------------------------------------------------- the imported models */

	/*
	 * One model at a time, in the slot after the ground.
	 *
	 * Loading them all at once is not on the table - together they are 804 KB of
	 * mesh against a heap of 140 - so the viewer builds one, shows it, frees it
	 * and builds the next. Peak demand is then the largest model rather than the
	 * sum, which is what makes any of them affordable.
	 */
	/*
	 * Exposure, in the terms Jet actually works in.
	 *
	 * The per-channel modulation is
	 *
	 *     scale = ambient + brightness        clamped to 255 + specular
	 *     out   = texel * scale / 255
	 *
	 * so 255 is where a texel passes through at full strength, and everything
	 * below it darkens. With the ambient alone at 205, a face turned away from
	 * the sun holds 80% of its texture instead of falling to near black, which is
	 * what made these models look muddy.
	 *
	 * specular is not a highlight here, it is headroom: it raises the clamp to
	 * 255 + specular, and the band above 255 lerps each channel toward its
	 * maximum. Without it a lit face can only ever reach the texture's own
	 * brightness and never looks lit at all - 48 lets the sun actually brighten
	 * what it falls on, while leaving the shading contrast the ambient would
	 * otherwise flatten.
	 */
	Material matModel(0xFFFF);
	matModel.shadingMode = ShadingMode::GOURAUD;
	matModel.specular    = 48;

	Texture texModel(1, 1, nullptr, false, 0, false, CLAMP);
	matModel.diffuseMap = &texModel;

	/*
	 * An empty, disabled Object holds the slot whenever no model is in it.
	 * Scene walks its object list reading obj->enabled without a null check, so
	 * the slot has to contain something at all times - a skipped model leaves
	 * this here rather than a hole.
	 */
	static Object placeholder;
	placeholder.enabled = false;

	scene.addObject(&placeholder);
	const int modelSlot  = (int)scene.getObjects().size() - 1;
	int       modelIndex = -1;
	Object   *loaded     = nullptr;


	auto load_model = [&](int index) {
		/* Point the slot away before freeing, so it never names dead memory. */
		scene.getObjects()[modelSlot] = &placeholder;
		if (loaded) {
			delete loaded;
			loaded = nullptr;
		}

		const ModelDef &m = g_models[index];
		const size_t    need = model_ram(m);
		modelIndex = index;

		if (need > MODEL_RAM_BUDGET) {
			printf("picojet: %-14s SKIPPED - %d tris would need %u KB "
			       "(%u mesh + %u queue), budget is %u KB\n",
			       m.name, m.triCount, (unsigned)(need / 1024),
			       (unsigned)((m.vertCount * sizeof(Object::Vertex)
			                 + m.triCount * sizeof(Object::Triangle)) / 1024),
			       (unsigned)((m.triCount * QUEUE_BYTES_PER_TRI) / 1024),
			       (unsigned)(MODEL_RAM_BUDGET / 1024));
			return false;
		}

		/* The texels stay in flash. Jet only reads them, so the const_cast is the
		 * whole of what "flash-resident texture" costs - no copy, no SRAM. */
		texModel.width   = m.texW;
		texModel.height  = m.texH;
		texModel.data    = const_cast<uint16_t *>(m.texels);

		loaded = build_model(m, &matModel, S);

		/*
		 * Sit it on the ground rather than through it. Every model is centred on
		 * its own origin by the converter, so lifting it by its half-height puts
		 * its lowest point on the surface - and the half-height is per-model
		 * because an aircraft is far flatter than a crab.
		 */
		loaded->setPosition(0, 0, 0);
		scene.getObjects()[modelSlot] = loaded;

		struct mallinfo mi = mallinfo();
		printf("picojet: %-14s loaded - %d verts, %d tris, %dx%d texture in flash"
		       " | budgeted %u KB, heap now %u KB used / %u KB free\n",
		       m.name, m.vertCount, m.triCount, m.texW, m.texH,
		       (unsigned)(need / 1024),
		       (unsigned)(mi.uordblks / 1024), (unsigned)(mi.fordblks / 1024));
		return true;
	};

	/*
	 * Whatever the scene holds that is not the model - zero now, with no ground.
	 * Captured before the first model is loaded, because that is the only moment
	 * it means anything: afterwards getStatistics() counts the model too.
	 */
	{
		int o, t, v;
		scene.getStatistics(o, t, v);
		g_sceneTris = t;
		printf("picojet: scene holds %d triangles before any model\n", t);
	}

	/*
	 * Load the LARGEST model first, whatever the display order.
	 *
	 * Scene's render queue is a std::vector, so it needs one contiguous block,
	 * and it keeps whatever capacity it has grown to - clear() does not hand it
	 * back. Grown while the heap is still pristine, it reaches the size the
	 * biggest model needs and never has to grow again; every later model fits
	 * inside it.
	 *
	 * Grown late, it cannot. Cycling meshes of different sizes leaves the free
	 * space in pieces, and the queue's single large request then fails with tens
	 * of kilobytes free but none of it adjacent - the failure depends on the
	 * order models happen to be shown in, which is a miserable thing to debug.
	 *
	 * Picking the maximum here rather than ordering the table means a bigger
	 * model added later is covered without anyone having to remember.
	 */
	int firstModel = 0;
	for (int i = 1; i < MODEL_COUNT; ++i)
		if (g_models[i].triCount > g_models[firstModel].triCount)
			firstModel = i;

	printf("picojet: warming the render queue on %s, the largest at %d triangles\n",
	       g_models[firstModel].name, g_models[firstModel].triCount);
	load_model(firstModel);


	int objects, triangles, vertices;
	scene.getStatistics(objects, triangles, vertices);
	printf("picojet: %d objects, %d triangles, %d vertices\n",
	       objects, triangles, vertices);
	printf("picojet: %d framebuffer%s, %d bytes total\n",
	       PICOJET_FRAMEBUFFERS, PICOJET_FRAMEBUFFERS == 1 ? "" : "s",
	       (int)sizeof(g_framebuffer));

	/* Core 1 becomes the second rasteriser. Launched after the scene exists, so
	 * the first token it pops already refers to a prepared frame. */
	multicore_launch_core1(core1_raster_loop);
	printf("picojet: core 1 rasterising rows %d..%d\n", SCREEN_H / 2, SCREEN_H);

	/* ---------------------------------------------------------- the input */

	/*
	 * picosdl reports the analog stick as a joystick and the I2C pad as a real
	 * SDL_GameController. Open whichever exist - the board may have either, both
	 * or neither, and a build with PICOSDL_INPUT_* off has none, which is not an
	 * error. Either one steers; the controller's left stick and the board's
	 * stick both land on the same two axes.
	 */
	SDL_Joystick       *joystick   = NULL;
	SDL_GameController *controller = NULL;

	if (SDL_IsGameController(0))
		controller = SDL_GameControllerOpen(0);
	if (controller == NULL && SDL_NumJoysticks() > 0)
		joystick = SDL_JoystickOpen(0);

	printf("picojet: input - %s%s%s\n",
	       controller ? "gamepad " : "",
	       joystick   ? "joystick " : "",
	       (!controller && !joystick) ? "none (auto-orbit only)" : "");

	/* ------------------------------------------------------- frame loop */

	int    back_index  = 0;
	Uint32 model_mark_ms = SDL_GetTicks();
	int    a_was_down  = 0;
	float  spinX       = 0.0f;
	float  spinY       = 0.0f;
	float  angle       = 0.0f;
	float  pitch       = 0.0f;
	float  radius      = 760.0f * S;
	Uint32 last_ms     = SDL_GetTicks();
	int    frames      = 0;
	Uint32 fps_mark_ms = last_ms;

	for (bool running = true; running; ) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			if (ev.type == SDL_QUIT)
				running = false;
			else if (ev.type == SDL_KEYDOWN && ev.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
				running = false;
		}

		Uint32 now_ms = SDL_GetTicks();
		float  dt     = (now_ms - last_ms) / 1000.0f;
		last_ms       = now_ms;
		if (dt > 0.1f)          /* a console pause should not teleport the scene */
			dt = 0.1f;

		/*
		 * Tumble whatever model is loaded, on two axes at rates that do not
		 * divide into each other.
		 *
		 * Accumulated in degrees per SECOND and applied with setRotation()
		 * rather than stepped with rotate() per frame: a fixed step per frame
		 * ties the speed to the frame rate, and these models run anywhere from
		 * 25 to 56 fps, so the same call would spin a light model twice as fast
		 * as a heavy one.
		 *
		 * 61 about Y is a full turn in just under the six seconds a model is
		 * shown, so every side comes past the camera once while it is up. 43
		 * about X rolls it through that turn at a comparable rate, so the top and
		 * underside are seen as much as the flanks - a much slower X reads as a
		 * yaw with a slight wobble rather than as a tumble.
		 *
		 * Both prime, so the pair does not settle into a repeating figure: it
		 * takes 360 seconds to return to where it started.
		 */
		if (loaded) {
			spinX = fmodf(spinX + dt * 43.0f, 360.0f);
			spinY = fmodf(spinY + dt * 61.0f, 360.0f);
			loaded->setRotation((int32_t)spinX, (int32_t)spinY, 0);
		}

		/*
		 * Advance through the models on a timer, or on the pad's A button.
		 * Every one of them gets shown whether or not it fits: the ones that do
		 * not say so on the console and leave the slot empty for their turn,
		 * which is more informative than quietly omitting them from the list.
		 */
		bool advance = (now_ms - model_mark_ms >= 6000);
		if (controller) {
			int a = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A);
			if (a && !a_was_down)
				advance = true;
			a_was_down = a;
		}
		if (advance) {
			load_model((modelIndex + 1) % MODEL_COUNT);
			model_mark_ms = now_ms;
		}

		/*
		 * Steering. Axes are SDL's -32768..32767; the deadzone is wide because
		 * neither the ADC stick nor the seesaw pad rests at exactly zero, and
		 * without it the camera drifts on its own and the auto-orbit never
		 * resumes.
		 */
		int ax = 0, ay = 0;
		if (controller) {
			ax = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
			ay = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
		} else if (joystick) {
			ax = SDL_JoystickGetAxis(joystick, 0);
			ay = SDL_JoystickGetAxis(joystick, 1);
		}

		const int DEADZONE = 6000;
		const bool steering = (ax < -DEADZONE || ax > DEADZONE ||
		                       ay < -DEADZONE || ay > DEADZONE);

		if (steering) {
			angle += (ax / 32768.0f) * dt * 1.6f;
			pitch += (ay / 32768.0f) * dt * 320.0f * S;
		} else {
			/* Nothing on the stick: drift, so the board demonstrates itself
			 * with no one holding it. */
			angle += dt * 0.35f;
		}

		if (pitch < -120.0f * S) pitch = -120.0f * S;
		if (pitch >  900.0f * S) pitch =  900.0f * S;

		g_camera.setPosition((int32_t)(cosf(angle) * radius),
		                     (int32_t)(210 * S + pitch),
		                     (int32_t)(sinf(angle) * radius));
		g_camera.lookAt(Vector3{0, 0, 0});

		/*
		 * The frame.
		 *
		 * render() clears, transforms, culls and sorts, hands the raster pass to
		 * raster_executor() so both cores draw it, then runs PostFX and the 2D
		 * sprite pass and advances the frame counter.
		 *
		 * Everything before it is about making sure the buffer it is about to
		 * clear is not the one the panel is reading.
		 */
		uint16_t *back = g_framebuffer[back_index];

#if PICOJET_FRAMEBUFFERS == 1
		/*
		 * The only buffer is the one in flight, so wait for it. PSDL_PresentSync()
		 * rather than a poll on PSDL_BufferBusy(): there is nothing else this
		 * program could be doing, and the blocking call is the one picosdl counts
		 * as core 0 idle, which keeps the load figure on the status band honest.
		 */
		PSDL_PresentSync();
#else
		/*
		 * With two, the back buffer is by construction not the one in flight, so
		 * this never actually waits - and asking is still the right thing to
		 * write. It is correct for any number of buffers, it costs one register
		 * read, and it does not depend on this loop's idea of which buffer is
		 * where agreeing with picosdl's.
		 *
		 * What overlaps is the whole point: the panel reads the front buffer
		 * through the DMA while both cores rasterise into the back one, so the
		 * push stops costing the frame anything the render was not already
		 * spending.
		 */
		while (PSDL_BufferBusy(back)) { }
#endif

		scene.setFramebuffer(back);
		scene.render(raster_executor);

		/*
		 * Hands the back buffer to the DMA and returns. Internally this waits for
		 * the previous push first, which with two buffers is the only place the
		 * frame can stall - and only for however much of the push the render did
		 * not already cover.
		 *
		 * Pitch is in bytes, as everywhere in SDL.
		 */
		PSDL_PresentBuffer(back, SCREEN_W, SCREEN_H,
		                   SCREEN_W * (int)sizeof(uint16_t));

		back_index = (back_index + 1) % PICOJET_FRAMEBUFFERS;

		/* Console figures once a second. The panel's own header carries the
		 * frame rate and the core loads; this adds what only the scene knows. */
		++frames;
		if (now_ms - fps_mark_ms >= 1000) {
			printf("picojet: %-14s %u fps, %d objs, %d tris queued, %d rasterised%s\n",
			       g_models[modelIndex].name,
			       (unsigned)(frames * 1000 / (now_ms - fps_mark_ms)),
			       scene.lastFrameDrawnObjects,
			       scene.lastFrameDrawnTriangles,
			       g_last_rasterized,
			       steering ? " [steering]" : "");

			char footer[48];
			snprintf(footer, sizeof(footer), "%s - %d tris",
			         g_models[modelIndex].name, g_models[modelIndex].triCount);
			PSDL_SetFooterText(footer);
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
