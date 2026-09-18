# picojet

A 3D software rasteriser on a Raspberry Pi Pico 2 W, driving an ST7789 panel at
320×200 in direct colour, using both cores.

It is a frontend: [Jet](https://github.com/CubeCoders/Jet) fills a framebuffer,
[picosdl](https://github.com/anight/picosdl) owns the panel and the input, and
this is what sits between them. Most of the lines explain a decision.

Two programs:

| | |
|---|---|
| `picojet` | a model viewer - ten meshes, one at a time, spinning on two axes. **68 fps**, which is the panel link's ceiling rather than the processor's. |
| `picojet-flight` | an F22 over a cloud deck, flown with the stick. **47 fps**, and here it is the processor. |

## The one idea

Jet renders RGB565. The ST7789 wants RGB565. Nothing converts anything.

That identity is not free by default, and getting it is most of the work. It
needed a 16bpp mode in the display driver, where the DMA reads a framebuffer and
hands it to the panel untouched, and a matching `PSDL_COLOR_DEPTH=16` in picosdl,
where a pixel is an RGB565 value and the client owns the buffer.

With both, Jet's framebuffer *is* the thing the DMA reads. No pass converts,
repacks or copies a pixel between the rasteriser and the glass.

## Both cores

Jet's `Scene::render()` takes a `RasterExecutor` — a hook replacing the raster
pass, which must join its workers and publish statistics before returning. With
audio compiled out of picosdl, core 1 is free to be one of those workers: core 0
prepares the frame and rasterises the top half, core 1 the bottom.

This is safe because `Z_BUFFERING` is off. The two bands write disjoint rows and
share nothing else, which is the condition `rasterizeBand()` documents. Jet's own
`scene_texture_queue` test, built against this `JetConfig.hpp`, confirms banded
output is pixel-identical to a whole-frame render across 144 frames.

The handshake is the inter-core FIFO rather than a flag in memory: it is a
hardware mailbox with the ordering already guaranteed, where a volatile flag would
need explicit barriers to say the same thing.

## Two framebuffers

125 KB each, and worth every byte. With one, the frame is a push followed by a
render, serially — core 0 measured 49% idle waiting for the panel. With two, Jet
rasterises into the back buffer while the DMA reads the front, and the push stops
costing the frame anything the render was not already spending.

|                  | `.bss`  | fps   |
|------------------|---------|-------|
| 1 framebuffer    | 240,088 | 29–32 |
| 2 framebuffers   | 368,088 | 56–58 |

(at 125 MHz; see the clock below)

`PICOJET_FRAMEBUFFERS` selects it. The loop differs only in how it learns the
back buffer is free: at 1 that is `PSDL_PresentSync()`, which blocks and which
picosdl counts as core 0 idle; at 2 it is a poll on `PSDL_BufferBusy()`, which
never actually waits — the back buffer is by construction not the one in flight —
but is written that way because it is correct for any number of buffers.

Most of what pays for the second buffer is the z-buffer this configuration does
not have.

## The models

Ten, shown one at a time, cycling on a timer or the gamepad's A button. The
flight demo uses one of them, the F22.

| model | triangles | source |
|---|---|---|
| cube, f117, f22, efa, sphere, crab | 12–498 | the [pikuma course](https://pikuma.com/courses/learn-3d-computer-graphics-programming) renderer |
| radio, column, biplane | 95–597 | [OpenGameArt](https://opengameart.org), CC0 |

`tools/convert_assets.py` turns `.obj` and `.png` into C arrays, because there is
no filesystem here and Jet's own `ObjLoader` reads files. It de-indexes — an
`.obj` indexes position, uv and normal separately where Jet's `Object` has one
vertex array carrying all three — normalises every model to the same **bounding
sphere** radius, and flips V, which `.obj` runs bottom-up and Jet samples
top-down.

Normalising by bounding sphere rather than longest axis is what makes models
*look* the same size. A cube fills its box; an aircraft is mostly empty air around
a wingspan. Matching widths leaves the cube far larger on screen, and larger still
once it rotates into its diagonal.

Textures are 128×128 RGB565 and stay in flash — Jet only reads texel data, so a
`const_cast` is the entire cost of a flash-resident texture.

### Decimation, and when not to

Models above a triangle budget are reduced by vertex clustering. It degrades an
organic shape gracefully and a hard-surface one badly: a crab loses detail evenly
across its bulk, while anything built from flat panels and sharp creases has
exactly those destroyed, because the creases are where the clustering merges
across. Vehicles at 2800 triangles reduced to 440 look broken; a sphere does not.
Prefer models that never need it.

Two details matter for colour-atlas textures, which many low-poly kits ship — one
image of flat patches with every vertex pointing at the middle of one:

* the decimator takes each cluster's UV from the member vertex nearest its centre
  rather than averaging, because an averaged UV lands *between* two patches and
  samples a colour that appears nowhere in the model;
* `--tex-nearest` resamples without filtering, because smoothing an atlas down
  bleeds neighbouring patches across their boundaries.

## The flight demo

The same renderer with a background that costs something, and it is the
background that shapes everything else.

### The sky is the clear

`clouds.cpp` fills every pixel of the frame, so Jet's clear is turned off and the
clouds replace it rather than being drawn over it. Two horizontal planes above
the camera and one below, textured with a precomputed tileable fBm tile.

That geometry is the reason it is affordable. For a pinhole camera the distance
to a horizontal plane is constant along a scanline, so a row costs one divide of
setup and then one add per axis per pixel - the floor mapper, which is as old as
the technique gets. Colour is never computed per pixel either: a ladder indexed
by `[fog band][density]` is built once and already contains the sky gradient,
cloud shading, coverage curve and distance haze, so a pixel is two texture
fetches and one ladder lookup.

The base tile is 128 square rather than 256, which is what makes the tables fit
beside two framebuffers.

**The bilinear filter is the whole frame budget.** Filtering every row costs
30 ms of a 33 ms frame. The rows that need it are the near ones, where a texel
covers enough pixels for the grid to show; beyond that the mip chain is already
resolving the detail and the interpolation is averaging texels that are nearly
equal. `FILT_DUT` is the threshold in texels per pixel:

| | clouds | fps |
|---|---|---|
| 0.75 | 30 ms | 29 |
| 0.42 | 21 ms | 38 |
| **0.25** | 14 ms | 57 |
| off | 13 ms | 60 |

### One camera, two renderers

The aircraft has to sit *in* the sky rather than in front of a picture of one,
which means both renderers must project identically. `clouds.cpp` uses a focal
length of 220 pixels; Jet's `setFOV()` derives its own as
`(screenWidth/2) / tan(fov/2)`. Solving that for 220 over a 320-wide screen gives
2·atan(160/220) = **72 degrees**.

The camera is raised above the aircraft and stays **level** - raised, never
pitched. Pitching would be the obvious way to look down at it, and it would break
the sky: the cloud renderer pins its horizon to the middle row and has no notion
of pitch, so tilting Jet's camera slides the two apart. Level and raised, both
still look down +Z, so their vanishing point is the same pixel.

Camera height and distance are tied together. The drop below the horizon is
`height × focal / distance`, so closing the distance to make the aircraft larger
also steepens the angle down to it; both scale by the same factor or it sinks out
of frame.

### Flying it

The stick commands a **bank**, the aircraft rolls toward it through a first-order
lag, and the **bank is what produces the yaw**. Driving the heading from the stick
directly and animating the bank alongside it looks wrong in a way that is hard to
place: centring the stick stops the sky dead under a wing that is still down.
One rate constant, `ROLL_RESPONSE`, is the only inertia in the model and the
heading inherits it.

### The afterburner

Two plumes off the mesh's own nozzle rings, each a pair of ribbons crossed at
right angles. A cone of any decent roundness costs ten times the triangles for a
shape the eye cannot tell apart at this size, and a single flat ribbon vanishes
when the aircraft banks it edge-on.

The plume is short because it grows *toward* the camera, and perspective works
against it hard - a plume of the length a side-on photograph suggests reaches
within a few hundred units of the lens and sweeps off the bottom of the frame as
a pair of enormous wedges.

One generated 64×32 texture, and the two axes do different jobs. **U runs along
the plume and carries the colour ramp**; **V is time**, and the flame is animated
by scrolling it. Baking both onto one axis means scrolling to animate the fire
also scrolls the ramp, and the white-hot part marches down the plume and off the
end; pinned to U it stays welded to the geometry while V boils underneath.

`UNLIT` rather than the `ADDITIVE` mode Jet also offers, which is the obvious
choice for a flame: an add against this sky saturates. The cloud deck is a bright
lavender, so adding any hot colour pins every channel and the plume comes out
white.

## Memory, which is the whole game

512 KB of SRAM. Two framebuffers take 250 KB of it, and what is left has to hold
Jet's heap.

Jet allocates, and it is worth knowing what for. `Object` owns `std::vector`s for
the mesh, and `Scene` builds a per-frame render queue holding every triangle
submitted — about **160 bytes per triangle**, roughly twice what the mesh costs.
Neither needs a general allocator; the renderer knows its own bounds, and a
fixed-capacity array sized at build time would do the same job. That it is a heap
has consequences:

* **A model has to be budgeted before it is built.** With exceptions compiled
  out, a failed allocation inside `std::vector` does not return — it panics. The
  check in `model_ram()` is a hard gate, and it counts the whole scene's
  triangles, because the queue is per frame rather than per object.

* **The largest model is loaded first.** The queue needs one contiguous block and
  keeps whatever capacity it grows to. Grown while the heap is pristine it
  reaches the size the biggest model needs and never grows again; grown late,
  after meshes of assorted sizes have been loaded and freed around it, the single
  large request fails with plenty free but none of it adjacent.

Textures are the counter-example and the easy case: read-only, so they stay in
flash and cost nothing.

## Building

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja
cmake --build build
picosdl/picodev.sh flash-and-logs build/picojet.elf        # the model viewer
picosdl/picodev.sh flash-and-logs build/picojet-flight.elf # the flight demo
```

Needs the Pico SDK (`PICO_SDK_PATH`, or `~/.pico-sdk`) and a CMSIS-DAP probe.

To regenerate the assets from source `.obj` and `.png`:

```bash
python3 tools/convert_assets.py --assets <dir> --out app/assets \
    --tex-size 128 --mesh-scale 270 --max-tris 600
```

## Configuration

Everything about the renderer is in `app/JetConfig.hpp`, with the reasoning next
to each choice. The ones that shape the rest:

* **`HALF_WIDTH_BUFFERS 0`** — the panel wants 320 full-width pixels per row, so
  a half-width buffer would need the CPU to double every one on the way out,
  which is exactly the copy this design exists to avoid.
* **`Z_BUFFERING 0`, `SORT_TRIANGLES 1`** — saves 125 KB and is what makes the
  two-core split safe.
* **`PERSPECTIVE_CORRECT_TEXTURES 1`** — costs a divide per pixel and measures as
  free here, because the frame is bound by the panel link rather than the
  processor. Affine interpolation leaves texels sliding across a face as it
  turns.

Together the first two forfeit `JET_FAST_SIMPLE_SPANS`, Jet's fastest path, which
requires `HALF_WIDTH_BUFFERS` and `!LIGHTING`. That is a deliberate trade: this
target cannot accept either condition.

## The clock

`PICOJET_SYS_CLOCK_KHZ`, default **150000**.

150 MHz is the RP2350 SDK's own default, so the core is not being pushed — the
panel is. The ST7789 driver clocks SCK from PIO sideset at clkdiv 1, so SCK is
sysclk/2 always: 75 MHz against a rated 62.5, twenty per cent over.

It is worth it because the frame is bound by that link. The push is
64000 × 34 / sysclk, so it falls from 17.4 ms to 14.5 and the ceiling rises from
57 fps to 69 — measured, 56 becomes 68 on the models that reach it.

This is a panel-by-panel judgement rather than a safe default in general. A panel
that tolerates 20% over is not obliged to, and one that does not shows torn,
speckled or shifted pixels rather than a blank screen. Build with
`-DPICOJET_SYS_CLOCK_KHZ=125000` for a clock with nothing out of spec at either
end, at 57 fps.

## Hardware

A Pico 2 W with an ST7789 panel. Every pin is in
`picosdl/backend/pico/board.h`. picosdl builds for all four Pico boards, though
this frontend's memory budget assumes the RP2350's 512 KB.

## Licence

**AGPL-3.0-or-later**, because it links [Jet](https://github.com/CubeCoders/Jet),
which is AGPL. Anyone you distribute a binary to — including over a network — is
entitled to the corresponding source of the whole work. CubeCoders sell a
commercial Jet licence for those who cannot accept that; picojet itself is
offered only under the AGPL.

The dependencies keep their own, more permissive terms: picosdl and `pio-st7789`
are BSD-2-Clause, and are not affected by the licence of this frontend. The
imported models are CC0 or come from the pikuma course renderer.
