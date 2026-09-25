// Runtime.cpp - the JetExamples runtime contract, on picosdl.
//
// Upstream, Esp32Jet::start() owns an ESP32's display, scanout and frame pacing,
// and each example supplies only its scene. This is the same contract on an
// RP2350 with an ST7789, so the example scenes compile here unmodified: an
// example's main.cpp calls Esp32Jet::start() from app_main() exactly as it does
// on the ESP32.
//
// The frame follows upstream's order - update, render, effects, afterRender,
// publish - with two framebuffers and both cores rasterising, as the rest of
// picojet does:
//
//   * Two framebuffers by default. The panel reads one while the next frame is
//     drawn into the other. PICOJET_EXAMPLE_FRAMEBUFFERS=1 gives the 128 KB of
//     the second back to the heap, for scenes that need it more than they need
//     the overlap.
//
//   * Both cores rasterise. Scene::render() takes an executor that replaces its
//     raster pass; this one hands rows RENDER_HEIGHT/2 and below to core 1 and
//     draws the top half on core 0. Upstream runs the same split with a depth
//     buffer as well as without, the bands being row-disjoint in both.
//
//   * Reflections read the previous frame. WATER_REFLECT materials sample
//     reflectBuffer, which is pointed at the buffer the panel is showing: a
//     completed image that neither core is writing. Upstream reads its previous
//     field for the same reason - current-frame reflections could read rows the
//     other core has not drawn yet - and it is what makes the split safe.
//
//     With a single framebuffer there is no previous frame to read, so
//     reflections read the frame being drawn and, as upstream does in the same
//     situation, the raster pass stays on one core.
#include "Runtime.hpp"
#include "Display.hpp"

#include <cstdio>
#include <cstring>
#include <malloc.h>
#include <vector>

#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "SDL2/SDL.h"
#include "psdl_pico.h"

#ifndef PICOJET_SYS_CLOCK_KHZ
#define PICOJET_SYS_CLOCK_KHZ PSDL_PICO_SYS_CLOCK_KHZ
#endif

#ifndef PICOJET_EXAMPLE_NAME
#define PICOJET_EXAMPLE_NAME "Jet example"
#endif

/* Supplied by the example, as on the ESP32. */
extern "C" void app_main(void);

int main(void)
{
	set_sys_clock_khz(PICOJET_SYS_CLOCK_KHZ, true);
	stdio_init_all();

	std::printf("\npicojet: %s\n", PICOJET_EXAMPLE_NAME);
	std::printf("sys clock %u Hz\n", (unsigned)clock_get_hz(clk_sys));

	app_main();

	/* start() does not return; an example that never calls it ends here. */
	for (;;)
		tight_loop_contents();
}

namespace Esp32Jet {
namespace {

#ifndef PICOJET_EXAMPLE_FRAMEBUFFERS
#define PICOJET_EXAMPLE_FRAMEBUFFERS 2
#endif
static_assert(PICOJET_EXAMPLE_FRAMEBUFFERS == 1 || PICOJET_EXAMPLE_FRAMEBUFFERS == 2,
              "PICOJET_EXAMPLE_FRAMEBUFFERS must be 1 or 2");

constexpr int W = Display::RENDER_WIDTH;
constexpr int H = Display::RENDER_HEIGHT;
constexpr int BAND_SPLIT = H / 2;
constexpr int BUFFERS = PICOJET_EXAMPLE_FRAMEBUFFERS;

uint16_t g_framebuffer[BUFFERS][W * H];

#if Z_BUFFERING
uint16_t g_zbuffer[W * H];
#endif

Renderer::Scene *g_scene;

/* Per-core record of which queued triangles each band rasterised, so the two
 * can be OR'd into one count without either core writing shared statistics.
 * Resized each frame; capacity only ever grows. */
std::vector<uint8_t> g_flags_core0, g_flags_core1;

/* Core 1: the bottom band, one token at a time. The token carries the queue
 * length, and its ordering is the whole handshake. */
void core1_worker(void)
{
	for (;;) {
		PSDL_CpuIdle();
		(void)multicore_fifo_pop_blocking();
		PSDL_CpuBusy();

		g_scene->rasterizeBand(BAND_SPLIT, H, g_flags_core1.data());

		multicore_fifo_push_blocking(1);
	}
}

/*
 * The raster pass, as Jet's executor hook.
 *
 * prepareFrame() has run by the time Jet calls this, so the render queue is
 * complete and core 1 may read it.
 */
void raster_executor(Renderer::Scene &scene)
{
	/* Reflections of the frame being drawn could read rows the other core has
	 * not written yet. */
	if (!scene.getRenderer()->reflectBuffer) {
		scene.rasterizeBand(0, H);
		return;
	}

	const size_t queued = (size_t)scene.lastFrameDrawnTriangles;

	g_flags_core0.assign(queued, 0);
	g_flags_core1.assign(queued, 0);

	multicore_fifo_push_blocking(1);
	scene.rasterizeBand(0, BAND_SPLIT, g_flags_core0.data());

	PSDL_CpuIdle();
	(void)multicore_fifo_pop_blocking();
	PSDL_CpuBusy();

	int rasterized = 0;
	for (size_t i = 0; i < queued; ++i)
		if (g_flags_core0[i] | g_flags_core1[i])
			++rasterized;
	scene.lastFrameRasterizedTriangles = rasterized;
}

} // namespace

void start(Init init, Update update, Update afterRender, RenderEffects renderEffects)
{
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		std::printf("SDL_Init failed: %s\n", SDL_GetError());
		for (;;) { }
	}

	SDL_Color band_fg = { 200, 220, 255, SDL_ALPHA_OPAQUE };
	SDL_Color band_bg = {   0,   0,   0, SDL_ALPHA_OPAQUE };
	PSDL_StatusBands(SDL_TRUE, band_fg, band_bg);
	PSDL_SetFooterText("github.com/anight/picojet");

#if Z_BUFFERING
	uint16_t *zbuffer = g_zbuffer;
#else
	uint16_t *zbuffer = nullptr;
#endif

	/* Allocated once and never freed, as upstream does; the scene outlives
	 * everything the example builds into it. */
	g_scene = new Renderer::Scene(g_framebuffer[0], zbuffer, W, H);
	init(*g_scene);

	multicore_launch_core1(core1_worker);

	std::printf("picojet: %dx%d, %d framebuffer%s, scene built, heap %u KB\n",
	            W, H, BUFFERS, BUFFERS == 1 ? "" : "s",
	            (unsigned)(mallinfo().uordblks / 1024));

	int    back_index  = 0;
	Uint32 last_ms     = SDL_GetTicks();
	Uint32 fps_mark_ms = last_ms;
	int    frames      = 0;

	for (;;) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) { }

		const Uint32 now_ms = SDL_GetTicks();
		float elapsed = (float)(now_ms - last_ms) / 1000.0f;
		last_ms = now_ms;
		if (elapsed <= 0.0f)
			elapsed = 1.0f / 1000.0f;

		if (update)
			update(elapsed);

		uint16_t *back = g_framebuffer[back_index];

		if (BUFFERS == 1) {
			/* The only buffer is the one on the panel; wait for it to land.
			 * PresentSync rather than a poll, so the wait counts as idle. */
			PSDL_PresentSync();
			g_scene->getRenderer()->reflectBuffer = nullptr;
		} else {
			while (PSDL_BufferBusy(back)) { }
			g_scene->getRenderer()->reflectBuffer = g_framebuffer[back_index ^ 1];
		}

		g_scene->setFramebuffer(back);

		g_scene->render(raster_executor);

		if (renderEffects)
			g_scene->lastFrameRasterizedTriangles += (int)renderEffects(*g_scene);
		if (afterRender)
			afterRender(elapsed);

		PSDL_PresentBuffer(back, W, H, W * (int)sizeof(uint16_t));
		back_index = (back_index + 1) % BUFFERS;

		++frames;
		if (now_ms - fps_mark_ms >= 1000) {
			std::printf("picojet: %u fps, %d tris, heap %u KB\n",
			            (unsigned)(frames * 1000 / (now_ms - fps_mark_ms)),
			            g_scene->lastFrameRasterizedTriangles,
			            (unsigned)(mallinfo().uordblks / 1024));
			frames      = 0;
			fps_mark_ms = now_ms;
		}
	}
}

} // namespace Esp32Jet
