// JetConfig.hpp - Jet renderer configuration for the picosdl / RP2350 frontend.
//
// Jet resolves `#include "JetConfig.hpp"` to whichever copy is on the include
// path, so this file is where this target's renderer is tuned. See
// Jet/src/JetConfig.example.hpp for the full list and the reasoning behind each
// switch; what follows records only why THIS target chose what it did.
//
// The shape of the target, which decides most of it:
//
//   * RP2350 at 125 MHz, two cores, 512 KB of SRAM, no FPU-hostile surprises.
//   * A 320x240 ST7789 that wants RGB565 and takes it over PIO and DMA with no
//     CPU involvement, into which a 320x200 canvas is letterboxed.
//   * picosdl built at PSDL_COLOR_DEPTH=16, so Jet's framebuffer IS the thing
//     handed to the panel. Nothing converts, quantises or copies it.
//
// That last point is what drives the buffer choices below, and it is worth
// stating plainly: the renderer's output format and the panel's input format
// are the same format. Every option here that would break that identity - a
// half-width buffer, a field buffer - is off, even where it would be faster,
// because re-expanding to full width on the CPU costs more than it saves.

// ---------------------------------------------------------------------------
// World-space scale
// ---------------------------------------------------------------------------

// 4 rather than 8: this is a 320x200 target, and the example file's own advice
// is that below roughly 480x320 the sub-pixel precision costs more in the
// integer transform chain than the edge stability is worth.
#define JET32_WORLD_SCALE 4

// ---------------------------------------------------------------------------
// Core rasterizer options
// ---------------------------------------------------------------------------

// The panel keeps whatever it was last sent and picosdl can push a sub-rectangle,
// so a dirty-tile map is available in principle. Off for now: the scene is a
// rotating one and nearly every tile is dirty every frame, so the bookkeeping
// would be pure overhead. Worth revisiting for a mostly-static scene.
#define RENDER_TILE_BUFFER 0
#define TILE_WIDTH  32
#define TILE_HEIGHT 32

#define FAST_Z 1
#define LAZY_Z 0

#define SCREEN_DOOR_ALPHA 1
#define SKIP_ZERO_AREA_TRIANGLES 1
#define NOISE_ALPHA 0

// No depth buffer. At 320x200 it would be 128 KB, and with SORT_TRIANGLES the
// painter's algorithm is correct for a scene of convex primitives that do not
// interpenetrate - which is what this demo draws. This is also what lets the two
// cores rasterise disjoint bands of the same frame without synchronising: with
// Z_BUFFERING off there is no shared per-pixel state between them, which
// Scene::rasterizeBand() documents as the condition for doing that safely.
#define Z_BUFFERING 0
#define SORT_TRIANGLES 1
#define SORT_SCENE_OBJECTS 0
#define SORT_SCENE_REVERSE 0

#define DEPTH_ALPHA_BLEND 1

// On. The output path does not care - it is already the panel's own format - so
// texturing costs the texel fetch and nothing else. Jet's texels are RGB565,
// which is the same thing the framebuffer and the panel are, so a sampled pixel
// is written out with no conversion anywhere in the chain.
#define TEXTURE_MAPPING 1

// Perspective-correct, at the cost of a divide per pixel.
//
// Affine interpolation walks UVs linearly in screen space, which is only right
// when the surface is parallel to the screen. Turned away from it, texels slide
// across the face as it rotates rather than staying in its plane - the PS1 warp.
// It is worst on few, large triangles seen at an angle, which is exactly a
// textured cube: two triangles a face, each filling a good part of the screen.
//
// Dividing UV by W per pixel is what makes the texture stay on the surface. It
// costs real time, but this target is bound by the panel link rather than by the
// processor - 17.4 ms of push against a frame that has spare - so on everything
// but the heaviest model it comes out of idle rather than out of the frame rate.
//
// It does forfeit JET_FAST_SIMPLE_SPANS, which requires the affine path. That
// costs nothing here: that path also requires HALF_WIDTH_BUFFERS and !LIGHTING,
// and this build has neither.
#define PERSPECTIVE_CORRECT_TEXTURES 1
#define BILINEAR_FILTER 0

// On. Gouraud interpolates brightness across a face, so a lit surface is a
// gradient rather than a set of flat steps - which is what the full RGB565 range
// is here to carry, and what a lit sphere shows most plainly.
#define LIGHTING 1

#define Z_BRIGHTNESS 0

#define FLOAT_CAMERA_ANGLES 1
#define FLOAT_SIN_CACHE_SCALE 10
#define FLOAT_TAN_CACHE_SCALE 1

// ---------------------------------------------------------------------------
// Buffer layout
// ---------------------------------------------------------------------------

// Both off, and for the same reason: the framebuffer is handed to the DMA as-is.
//
// HALF_WIDTH_BUFFERS would halve the fill cost and the 128 KB, but the panel
// wants 320 full-width RGB565 pixels per row, so something would have to double
// each column back up - 64000 pixels of CPU work per frame against a push that
// currently costs zero. FIELD_BUFFERS has the same problem plus a second
// half-height buffer to interleave.
//
// This is the trade the 16bpp path exists to make: spend the RAM, keep the CPU.
#define HALF_WIDTH_BUFFERS 0
#define FIELD_BUFFERS 0
#define SSR_FIELD_REFLECT 0

// ---------------------------------------------------------------------------
// Post-processing effects
// ---------------------------------------------------------------------------

// The "free" effects - free of an extra buffer, not of time. CRT was measured on
// this target: it costs about 8 ms a frame, taking 30 fps to 24. That is what a
// full-screen read-modify-write over 64000 RGB565 pixels costs at 125 MHz, and
// unlike the raster pass it runs on core 0 alone - Scene::render() applies PostFX
// after the executor has already joined both bands. Correct and available; off
// because a quarter of the frame rate is a lot to pay for scanlines.
#define POSTFX_CRT         0
#define POSTFX_CELLSHADING 0

// The buffered effects, and none of them fit. Bloom allocates two full-screen
// RGB565 buffers and motion blur one - 256 KB and 128 KB against a 512 KB part
// that has already spent 128 KB on the framebuffer. They are not disabled
// because they are slow; they are disabled because the memory is not there.
#define POSTFX_ANTIALIASING 0
#define POSTFX_BLOOM        0
#define POSTFX_MOTION_BLUR  0
#define POSTFX_CHROMATIC    0
#define POSTFX_PIXELATE     0

#define CRT_SCANLINE_INTENSITY 48
#define MOTION_BLUR_STRENGTH   50
#define CHROMATIC_OFFSET        2
#define PIXELATE_SIZE           4
#define CELLSHADING_CELL_BITS   4

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

#define DEBUG_OVERDRAW 0

// ---------------------------------------------------------------------------
// Depth / fog tuning  (world-space units, scaled by JET32_WORLD_SCALE)
// ---------------------------------------------------------------------------

#define zBrightFar   (1600 * JET32_WORLD_SCALE)
#define zBrightNear  ( 200 * JET32_WORLD_SCALE)
#define zBrightScale 48

#define depthFogFar  (4096 * JET32_WORLD_SCALE)
#define depthFogNear (3072 * JET32_WORLD_SCALE)

// ---------------------------------------------------------------------------
// Checkerboard rendering
// ---------------------------------------------------------------------------

#define CHECKERBOARD_MODE 0
#define CHECKERBOARD_RECONSTRUCTION 0

// ---------------------------------------------------------------------------
// Screen-space picking
// ---------------------------------------------------------------------------

// Nothing here picks, and 0 compiles the machinery out entirely rather than
// leaving a per-scanline test in the triangle hot path.
#define MAX_PICK_QUERIES 0

// ---------------------------------------------------------------------------
// Platform detection
// ---------------------------------------------------------------------------

// Jet's own guard. ESP_PLATFORM is never defined here, so the ESP32 paths
// (IRAM_ATTR, esp_attr.h, the S3 DSP intrinsics) stay out of the build and
// PERF_CRITICAL expands to nothing - which is correct on a part that runs code
// from XIP cache rather than from a separate instruction RAM.
#if defined(ESP_PLATFORM)
#define ESP32
#endif
