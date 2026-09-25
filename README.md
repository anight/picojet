# picojet

<p align="center">
  <a href="https://youtube.com/shorts/BPHVAzR6nX4">
    <img src="https://img.youtube.com/vi/BPHVAzR6nX4/oardefault.jpg" width="300"
         alt="picojet running on a Pico 2 W — click to play">
  </a>
</p>

Real-time 3D software rasterisation on a Raspberry Pi Pico 2 W, rendering
320×200 in 16-bit direct colour to an ST7789 panel across both CPU cores.

## Contents

- [Components](#components)
- [Hardware](#hardware)
- [Programs](#programs)
- [Building](#building)
- [Architecture](#architecture)
- [Memory](#memory)
- [Assets](#assets)
- [Cloud background](#cloud-background)
- [Configuration](#configuration)
- [Measured performance](#measured-performance)
- [Licence](#licence)

## Components

| Component | Role | Licence |
|---|---|---|
| [Jet](https://github.com/CubeCoders/Jet) | Fixed-point software rasteriser. Submodule. | MIT |
| [picosdl](https://github.com/anight/picosdl) | SDL2 subset for RP2040/RP2350: display, input, timing. Submodule. | BSD-2-Clause |
| [pio-st7789](https://github.com/anight/pio-st7789) | PIO/DMA ST7789 driver. Submodule of picosdl. | BSD-2-Clause |
| [JetExamples](https://github.com/CubeCoders/JetExamples) | Example scenes for Jet. Nine are vendored under `examples/`. | MIT |
| picojet | This repository: application layer, asset pipeline, cloud renderer. | BSD-2-Clause |

## Hardware

| Item | Detail |
|---|---|
| Board | Raspberry Pi Pico 2 W (RP2350: dual Cortex-M33 at 150 MHz, 520 KB SRAM, 4 MB flash) |
| Display | ST7789, 240×320, driven over PIO SPI with DMA |
| Input | Analogue joystick on the ADC, or an Adafruit Seesaw I2C gamepad |
| Programmer | CMSIS-DAP probe |

Pin assignments are defined in `picosdl/backend/pico/board.h`.

The canvas is 320×200, letterboxed into the 240-pixel panel height at y=20.
picosdl draws a status band in each remaining strip: frame rate and per-core load
above, a caption below.

## Programs

| Program | Description |
|---|---|
| `picojet` | Model viewer. Nine textured meshes, displayed one at a time. |
| `picojet-flight` | Flight demonstration. A jet aircraft over a procedurally generated cloud deck. |
| `example-*` | Nine scenes from JetExamples, each a separate program. |

### picojet

The model advances every six seconds, or immediately on the gamepad's A button.
It rotates continuously at 43°/s about X and 61°/s about Y, so that every face
passes the camera within its display period.

The camera orbits the origin. Horizontal stick deflection controls azimuth,
vertical deflection controls height. With the stick centred the camera drifts at
0.35 rad/s so that the board demonstrates itself unattended.

| Model | Triangles | Source |
|---|---|---|
| cube | 12 | [pikuma](https://pikuma.com/courses/learn-3d-computer-graphics-programming) course renderer |
| radio | 95 | OpenGameArt, CC0 |
| f117 | 134 | pikuma |
| f22 | 200 | pikuma |
| column | 212 | OpenGameArt, CC0 |
| efa | 224 | pikuma |
| crab | 476 | pikuma |
| sphere | 498 | pikuma |
| biplane | 597 | OpenGameArt, CC0 |

### picojet-flight

The aircraft is fixed at the centre of the frame and the world turns around it.
Horizontal stick deflection is the only control. The A button, or the joystick's
own click, cycles the aircraft.

| Aircraft | Triangles | Exhaust plumes |
|---|---|---|
| F-22 | 200 | 2 |
| EF-2000 | 224 | 2 |
| F-117 | 134 | 0 |

Plume counts follow the aircraft. The F-22 and EF-2000 have two afterburning
engines each. The F-117 has two engines and no afterburner: its exhausts are
slots that mix efflux with cold air to suppress the infrared signature, so no
flame is visible. Each nozzle position is taken from the vertex ring on the aft
face of the engine in the mesh itself.

The biplane is absent from this program. Its render queue requires roughly two
and a half times the memory of the largest aircraft here, which the cloud tables
and the second framebuffer have already consumed.

**Flight model.** The stick commands a bank angle. The aircraft rolls toward it
through a first-order lag, and the resulting bank produces the yaw rate. A single
constant, `ROLL_RESPONSE`, is the only inertia term; heading inherits it. Full
deflection corresponds to 55° of bank and 0.9 rad/s of yaw.

### JetExamples

Nine of the sixteen JetExamples scenes, each built as its own program. They run
unattended and take no input. Each keeps the renderer configuration it was
written with, so together they exercise features the two programs above leave
disabled: bilinear filtering, texture addressing modes, palette cycling,
particles, level of detail, sprites, blending, water reflections and CRT
post-processing.

| Program | Scene | Demonstrates | Framebuffers |
|---|---|---|---:|
| `example-template-cube` | Rotating cube | Minimal scene: camera and coloured geometry | 2 |
| `example-particles` | Particle Lab | Additive sparks, gravity-driven spray, alpha fades, fixed pools, distance culling | 2 |
| `example-textured-boxes` | Textured crate | Affine and perspective-correct mapping, nearest and bilinear sampling | 2 |
| `example-texture-features` | Texture Lab | Wrap, clamp and zero addressing, colour keys, palette cycling, texture LOD | 2 |
| `example-postfx-crt` | CRT / Arcade | Scanline post-processing | 2 |
| `example-lod-billboards` | Woodland | Distance-driven mesh simplification, billboard stand-ins, transition fading | 2 |
| `example-sprite-controls` | Air Mail | Sprite flips, per-material and per-sprite alpha, motion echoes, screen fades | 2 |
| `example-tropical-island` | Tropical island | Sky gradient, rippled water, reflections, additive sun sprites, lens flares | 1 |
| `example-sprites-blending` | After Hours | Additive glow meshes, sprite halos, translucent layers, mirrored-floor reflections | 1 |

The last two run with one framebuffer, whose 128 KB their render queues need.

The scenes were authored for 480×320. Their projection is derived from the
output size and adapts to 320×200; their on-screen captions are pre-rendered
bitmaps at fixed coordinates, and those placed below row 200 are not visible.

The remaining seven do not fit in the RP2350's SRAM. Each was built and run on
the board, and each exhausts the heap:

| Scene | Where it runs out |
|---|---|
| Depth comparison, Utah teapot lighting, Cel / Teapot | First frame. The 128 KB depth buffer and a 64 KB scene leave too little for the render queue, even with one framebuffer. |
| Neon Motorworks | First frame, after 145 KB of scene parsed from its embedded OBJ model. |
| REPEAT, MATTER | While building the scene. Upstream places their bulk data in PSRAM. |
| ESP 88 | Mid-film, at 5–6 fps, when a heavier scene is loaded. |

## Building

Requirements: the Raspberry Pi Pico SDK (`PICO_SDK_PATH`, or `~/.pico-sdk`),
CMake 3.13 or later, Ninja, and the `arm-none-eabi` toolchain.

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja
cmake --build build
```

Flashing requires a CMSIS-DAP probe:

```bash
picosdl/picodev.sh flash-and-logs build/picojet.elf
picosdl/picodev.sh flash-and-logs build/picojet-flight.elf
picosdl/picodev.sh flash-and-logs build/example-tropical-island.elf
```

## Architecture

### Direct-colour path

Jet renders RGB565 and the ST7789 accepts RGB565. No stage between the
rasteriser and the panel converts, repacks or copies a pixel: the DMA engine
reads Jet's framebuffer directly.

This requires `PSDL_COLOR_DEPTH=16` in picosdl, where a pixel is an RGB565 value
and the client owns the buffer, and the 16bpp mode of `pio-st7789`, where the
DMA hands framebuffer contents to the panel unaltered.

### Dual-core rasterisation

`Scene::render()` accepts a `RasterExecutor`, a hook that replaces the raster
pass and is required to join its workers and publish statistics before
returning. With audio compiled out of picosdl, core 1 is available as a second
rasteriser: core 0 prepares the frame and rasterises rows 0–99, core 1
rasterises rows 100–199.

This is safe because `Z_BUFFERING` is disabled. The two bands write disjoint
rows and share no per-pixel state, which is the condition `rasterizeBand()`
documents. Jet's own `scene_texture_queue` test, built against this
`JetConfig.hpp`, confirms that banded output is identical to a whole-frame
render across 144 frames.

The handshake uses the inter-core FIFO rather than a flag in memory. The FIFO is
a hardware mailbox with ordering already guaranteed; an equivalent flag would
require explicit memory barriers.

Both programs report per-core load through picosdl's `PSDL_CpuIdle()` and
`PSDL_CpuBusy()`, which bracket each core's blocking waits. Load is the
complement of the bracketed intervals.

### Double buffering

Two framebuffers of 128,000 bytes each. Core 0 selects the buffer not currently
being transmitted, polling `PSDL_BufferBusy()`, and rasterises into it while the
DMA engine reads the other. Measured with a single buffer, core 0 spends
approximately half of each frame blocked on the panel transfer.

`PICOJET_FRAMEBUFFERS` selects between one and two. At 1 the program waits with
`PSDL_PresentSync()`, which picosdl accounts as idle time on the calling core.

### JetExamples runtime

Upstream, each example supplies only its scene: `init(Scene&)` and
`update(seconds)`, passed to `Esp32Jet::start()`, whose ESP32 runtime owns the
display, scanout and frame pacing. `examples/runtime/` implements the same
contract on picosdl, so the scene sources compile unmodified; the vendored
directories are identical to upstream.

The frame follows upstream's order (update, render, effects, after-render,
publish) and the design above: whole RGB565 frames, two framebuffers by default,
and both cores rasterising a fixed half each. The ESP-IDF calls the scenes make
are PSRAM placement hints, memory queries and `esp_restart()`; small headers in
`examples/runtime/` provide them.

**One Jet build per example.** `JetConfig.hpp` is a set of compile-time
switches and the examples disagree about them, so Jet is compiled separately
against each example's own configuration. `examples/runtime/JetConfigPico.hpp`
then overrides only the frame layout: upstream renders half-width interlaced
fields for its scanout, this panel receives whole frames.

**Reflections read the previous frame.** `WATER_REFLECT` materials sample
`reflectBuffer`, which the runtime points at the buffer on the panel: a finished
image that neither core writes. Reflections of the frame being drawn could read
rows the other core has not reached, so with one framebuffer, where no previous
frame exists, the raster pass stays on one core, as upstream does in the same
case.

### System clock

`PICOJET_SYS_CLOCK_KHZ` defaults to 150000, the RP2350 SDK default.

The ST7789 driver derives SCK from a PIO side-set at a clock divider of 1, so
SCK is always half the system clock: 75 MHz against a rated maximum of 62.5 MHz,
an overclock of 20%.

The frame is bounded by this link rather than by the processor. Raising the
clock from 125 MHz to 150 MHz shortens the panel transfer from 17.4 ms to
14.5 ms and raises the observed ceiling from 57 fps to 68 fps.

Tolerance of the overclock varies between panels. A panel that does not tolerate
it displays torn, speckled or shifted pixels rather than failing outright. Build
with `-DPICOJET_SYS_CLOCK_KHZ=125000` for an in-specification clock at both
ends.

## Memory

The RP2350 provides 520 KB of SRAM, of which the two framebuffers occupy 250 KB.

| Program | `.text` | `.bss` | Configured heap |
|---|---:|---:|---:|
| `picojet` | 659,444 | 343,312 | 163,840 |
| `picojet-flight` | 359,772 | 397,924 | 49,152 |

Jet allocates from the heap in two places. `Object` holds the mesh in
`std::vector` members, and `Scene` builds a per-frame render queue containing
every submitted triangle, at approximately 128 bytes per triangle across its
three vectors. Exceptions are disabled, so a failed allocation terminates the
program rather than returning.

Two consequences govern how both programs load meshes.

1. **A mesh must be budgeted before it is constructed.** `picojet` checks the
   projected cost in `model_ram()` and reports models that do not fit rather
   than attempting them. The check accounts for the whole scene's triangles,
   because the render queue is per frame and not per object.

2. **The largest mesh is loaded first.** The render queue requires one
   contiguous allocation and retains whatever capacity it reaches. Grown while
   the heap is unfragmented it reaches its maximum immediately; grown after
   several meshes of differing sizes have been allocated and freed, the same
   request fails with sufficient total free memory but no adjacent block.

`picojet-flight` additionally reserves both mesh vectors for the largest
aircraft in its table before loading any of them. Because `std::vector` retains
capacity across `clear()`, changing aircraft performs no allocation at all.

Textures are read-only and remain in flash, at no cost in SRAM.

## Assets

`tools/convert_assets.py` converts `.obj` and `.png` files into C arrays. There
is no filesystem on the target, and Jet's own `ObjLoader` reads from files.

The converter performs three transformations:

- **De-indexing.** An `.obj` file indexes position, texture coordinate and
  normal separately, whereas Jet's `Object` holds a single vertex array carrying
  all three.
- **Normalisation by bounding sphere.** Scaling to a common bounding sphere
  radius, rather than to a common longest axis, makes models appear comparably
  sized. A cube fills its bounding box; an aircraft is largely empty space
  around a wingspan. Matching widths leaves the cube substantially larger on
  screen, and larger still once rotated onto its diagonal.
- **V-axis inversion.** `.obj` texture coordinates run bottom-up; Jet samples
  top-down.

Textures are emitted as 128×128 RGB565 arrays.

Meshes exceeding a triangle budget can be reduced by vertex clustering
(`--max-tris`). The technique degrades organic shapes acceptably and
hard-surface shapes badly, because clustering merges across precisely the
creases that define a panelled model. Two options exist for colour-atlas
textures, which consist of flat patches with every vertex addressing the centre
of one: each cluster takes its texture coordinate from the member vertex nearest
the cluster centre rather than from an average, since an averaged coordinate
falls between patches; and `--tex-nearest` resamples without filtering, since
smoothing an atlas bleeds adjacent patches across their boundaries.

To regenerate the assets:

```bash
python3 tools/convert_assets.py --assets <dir> --out app/assets \
    --tex-size 128 --mesh-scale 270 --max-tris 600
```

## Cloud background

`app/clouds.cpp` renders a full-screen cloud deck. It writes every pixel it is
given, so Jet's buffer clear is disabled and the sky replaces it.
`tools/clouds.c` is the reference implementation the port derives from.

The scene comprises two horizontal planes above the camera and one below,
textured with a precomputed tileable fBm noise tile of 128×128 texels and a
four-level mip chain. For a pinhole camera the distance to a horizontal plane is
constant along a scanline and the texture coordinate is linear in x, so a row
costs one division of setup followed by one addition per axis per pixel. The
inner loop is entirely 16.16 fixed-point integer arithmetic; floating point
appears only in per-row setup.

Colour is never computed per pixel. A lookup table indexed by fog band and
density is built once and already incorporates the sky gradient, cloud shading,
coverage curve and distance haze, so a pixel costs two texture fetches and one
table lookup.

Both renderers must project identically, or the aircraft will not appear to
occupy the sky. The cloud renderer uses a focal length of 220 pixels; Jet's
`setFOV()` derives its own factor as `(screenWidth/2) / tan(fov/2)`. Solving for
220 across a 320-pixel width gives a field of view of 72°.

The camera is raised above the aircraft and held level. It is never pitched: the
cloud renderer fixes its horizon at the middle row and has no concept of pitch,
so tilting Jet's camera would separate the two. Camera height and distance scale
together, since the aircraft's displacement below the horizon is
`height × focal / distance`.

**Bilinear filtering is the dominant cost.** `FILT_DUT` is the threshold in
texels per pixel above which a row is filtered. Rows near the viewer require it;
beyond that distance the mip chain already resolves the detail and interpolation
averages nearly equal texels.

| `FILT_DUT` | Cloud pass | Frame rate |
|---|---:|---:|
| 0.75 | 30 ms | 29 fps |
| 0.42 | 21 ms | 38 fps |
| **0.25** (default) | 14 ms | 57 fps |
| disabled | 13 ms | 60 fps |

## Configuration

### Build options

| Option | Default | Effect |
|---|---|---|
| `PICOJET_FRAMEBUFFERS` | 2 | Framebuffers cycled by the renderer, 1 or 2 |
| `PICOJET_SYS_CLOCK_KHZ` | 150000 | System clock; 125000 keeps the panel within specification |

The build forces the following picosdl options: `PICOSDL_COLOR_DEPTH=16`,
`PICOSDL_AUDIO=OFF` (core 1 is the second rasteriser),
`PICOSDL_INPUT_BT_KEYBOARD=OFF`, `PICOSDL_INPUT_JOYSTICK=ON`,
`PICOSDL_INPUT_GAMEPAD=ON`.

### Renderer configuration

`app/JetConfig.hpp` carries the full set with per-option reasoning. The
settings that determine the rest:

| Setting | Value | Rationale |
|---|---|---|
| `HALF_WIDTH_BUFFERS` | 0 | The panel requires 320 full-width pixels per row. A half-width buffer would oblige the CPU to duplicate every pixel on output, which is the copy this design exists to avoid. |
| `Z_BUFFERING` | 0 | Saves 128 KB and permits the two-core split. |
| `SORT_TRIANGLES` | 1 | Painter's algorithm in place of a depth buffer. |
| `LIGHTING` | 1 | Directional and ambient lighting, Gouraud interpolation. |
| `PERSPECTIVE_CORRECT_TEXTURES` | 1 | One division per pixel. Measures as free, the frame being bound by the panel link. Affine interpolation leaves texels sliding across a face as it turns. |
| `BILINEAR_FILTER` | 0 | Texture magnification is point-sampled. |
| `SCREEN_DOOR_ALPHA` | 1 | Transparency by ordered dither, avoiding a read-modify-write per pixel. |
| `POSTFX_*` | 0 | All post-processing disabled. |

The first two together forfeit `JET_FAST_SIMPLE_SPANS`, Jet's fastest raster
path, which requires `HALF_WIDTH_BUFFERS` and `!LIGHTING`. Neither condition is
acceptable on this target.

## Measured performance

At 150 MHz with two framebuffers. Ranges reflect the varying screen coverage as
each model rotates; the upper bound of 68 fps is the panel link's ceiling.

| `picojet` model | Triangles | Frame rate |
|---|---:|---|
| cube | 12 | 43–55 |
| radio | 95 | 60–68 |
| f117 | 134 | 67–68 |
| f22 | 200 | 67–68 |
| column | 212 | 50–62 |
| efa | 224 | 65–68 |
| crab | 476 | 34–65 |
| sphere | 498 | 29–31 |
| biplane | 597 | 48–67 |

| JetExamples scene | Frame rate | Peak heap |
|---|---|---:|
| Rotating cube | 68–69 | 4 KB |
| Particle Lab | 67–69 | 53 KB |
| Textured crate | 26–68 | 5 KB |
| Texture Lab | 24–28 | 9 KB |
| CRT / Arcade | 23–28 | 7 KB |
| Woodland | 24–32 | 94 KB |
| Air Mail | 21–26 | 108 KB |
| Tropical island | 19–23 | 123 KB |
| After Hours | 9–18 | 179 KB |

Ranges span the modes each scene cycles through. The heap available is about
180 KB with two framebuffers and about 305 KB with one.

`picojet-flight` runs at 33–58 fps. Unlike the viewer it is bound by the
processor rather than the panel, and the cloud pass dominates: of a
21.5 ms frame, 17.5 ms is the two-core raster pass, 2.0 ms the transform and
sort, 1.4 ms input polling, and 0.2 ms the panel handover.

## Licence

picojet is distributed under **BSD-2-Clause**. See `LICENSE`.

Dependencies retain their own terms. [Jet](https://github.com/CubeCoders/Jet) is
MIT, and its copyright notice must accompany any binary built from this
repository; the same applies to the JetExamples scenes under `examples/`, whose
MIT licence is in `examples/LICENSE`. picosdl and `pio-st7789` are BSD-2-Clause. Imported models are CC0
or originate from the pikuma course renderer.
