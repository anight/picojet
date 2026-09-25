#
# picojet - the Jet 3D software rasteriser on an RP2350, through picosdl.
#
# Three pieces, each untouched as a library:
#
#   Jet/      the rasteriser. Owns no display and no main loop; it fills a
#             framebuffer. Configured entirely by the JetConfig.hpp this project
#             puts on its include path - see app/JetConfig.hpp.
#   picosdl/  the panel, the input and the clock, built here at
#             PSDL_COLOR_DEPTH=16 so Jet's RGB565 output reaches the DMA as-is.
#   app/      this program: the scene, the frame loop, and the two-core split.
#
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pico 2 W (RP2350) at 125 MHz - picosdl's board.h is the authority on both, and
# the clock comment in it explains why 125 rather than the SDK's 150.
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Speed over size. The rasteriser is the whole program's cost and -Os is not what
# it wants; this matches picosdl's own reasoning for its mixer.
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/picosdl/cmake/pico_sdk_bootstrap.cmake)
project(picojet C CXX ASM)
pico_sdk_init()

# --------------------------------------------------------------- picosdl
#
# 16bpp is the point of this build: a pixel is an RGB565 value, which is what Jet
# produces and what the panel wants, so the framebuffer reaches the DMA as-is.
#
set(PICOSDL_COLOR_DEPTH 16 CACHE STRING "" FORCE)

# Audio off, and not only to save space: it frees core 1, which this program uses
# as its second rasteriser. With the mixer there, one of the two bands would have
# nowhere to run.
set(PICOSDL_AUDIO OFF CACHE BOOL "" FORCE)

# Bluetooth off - 431 KB of flash and 22 KB of RAM for a keyboard nothing here
# reads. The stick and the I2C pad stay: they cost 4 KB between them.
set(PICOSDL_INPUT_BT_KEYBOARD OFF CACHE BOOL "" FORCE)
set(PICOSDL_INPUT_JOYSTICK    ON  CACHE BOOL "" FORCE)
set(PICOSDL_INPUT_GAMEPAD     ON  CACHE BOOL "" FORCE)

add_subdirectory(picosdl)

# ------------------------------------------------------------------- Jet
#
# Jet's own CMakeLists globs src/*.cpp and builds a static library. Nothing in it
# needs changing for this target: the ESP32 paths are behind ESP_PLATFORM, which
# no one defines here, and the rest is portable C++17.
#
add_subdirectory(Jet/src)

#
# LTO on Jet: worth 30 KB of flash, and nothing measurable in speed.
#
# It was added expecting the second. The rasteriser's general inner loop samples
# through Texture::getPixel(), which lives in its own translation unit, so every
# textured pixel pays a cross-TU call whose WRAP path does four signed modulos -
# an obvious thing for link-time inlining to fix, especially as the fast span
# path that would avoid it (JET_FAST_SIMPLE_SPANS) needs HALF_WIDTH_BUFFERS and
# !LIGHTING and is therefore unavailable to this target.
#
# Measured, it changed the frame rate by nothing at all, in either the light or
# the heavy part of the scene. The sampler was not the bottleneck. Kept anyway
# for the flash, and recorded here so the next person does not spend the
# afternoon a second time.
#
# Scoped to Jet rather than set globally: the SDK is not built with it and there
# is no reason to disturb it.
#
set_property(TARGET Jet PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

# Every Jet header does #include "JetConfig.hpp", so the directory holding this
# target's copy has to be on Jet's own include path - not just the app's.
target_include_directories(Jet PUBLIC ${CMAKE_CURRENT_LIST_DIR}/app)

# --------------------------------------------------------------- picojet

#
# Framebuffers the renderer cycles.
#
# 2 lets Jet rasterise into one while the DMA pushes the other, which is worth
# most of the frame rate on this board: with one buffer the render and the push
# are serial, and the loop spends about half its time waiting for the panel.
#
# 1 halves the 250 KB this costs, for a board or a scene that needs the RAM back.
# app/main.cpp handles both; the difference is which call it uses to know the
# buffer is free.
#
set(PICOJET_FRAMEBUFFERS 2 CACHE STRING "Framebuffers the renderer cycles (1 or 2)")
set_property(CACHE PICOJET_FRAMEBUFFERS PROPERTY STRINGS 1 2)

add_executable(picojet app/main.cpp)

target_compile_definitions(picojet PRIVATE PICOJET_FRAMEBUFFERS=${PICOJET_FRAMEBUFFERS})

#
# System clock, in kHz.
#
# 150 MHz is the RP2350 SDK's own default, so the core is not being pushed - what
# is being pushed is the panel. The ST7789 driver clocks SCK from PIO sideset at
# clkdiv 1, so SCK is sysclk/2 always: 75 MHz against a rated 62.5, twenty per
# cent over.
#
# It is worth it here because the frame is bound by that link rather than by the
# processor. The push is 64000 pixels x 34 cycles / sysclk, so it falls from
# 17.4 ms to 14.5 and the ceiling rises from 57 fps to 69 - measured, 56 becomes
# 68 on the models that reach it.
#
# It is a panel-by-panel judgement, not a safe default in general: one that
# tolerates 20% over is not obliged to, and the symptom of one that does not is
# torn, speckled or shifted pixels rather than a blank screen. Set this to 125000
# for a clock with nothing out of spec at either end.
#
set(PICOJET_SYS_CLOCK_KHZ 150000 CACHE STRING "System clock in kHz (125000 keeps the panel in spec)")

target_compile_definitions(picojet PRIVATE PICOJET_SYS_CLOCK_KHZ=${PICOJET_SYS_CLOCK_KHZ})

target_link_libraries(picojet PRIVATE Jet picosdl)

set_source_files_properties(app/main.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

# Jet allocates its meshes with new/std::vector at start-up - Primitives::create*
# returns an owned Object. Nothing allocates in the frame loop, so this only has
# to cover the scene built once in main().
# The heap holds one imported mesh plus Scene's per-frame render queue, and the
# queue is the larger half - about 160 bytes per triangle submitted. 168 KB is
# what two framebuffers leave; app/main.cpp checks a model against a budget
# before building it rather than finding out by exhausting this, because with
# exceptions off a failed allocation panics rather than returning.
target_compile_definitions(picojet PRIVATE PICO_HEAP_SIZE=163840)

pico_enable_stdio_uart(picojet 1)
pico_enable_stdio_usb(picojet 1)
pico_add_extra_outputs(picojet)

# ------------------------------------------------------------- picojet-flight
#
# The second demo: one aircraft over a cloud deck, steered with the stick.
#
# A separate binary rather than a mode of the first, because the two want the
# memory arranged differently. The viewer spends it on a heap big enough for
# whichever model is loaded; this one has a single mesh it never swaps, and
# spends what that frees on the cloud renderer's noise and colour tables.
#
add_executable(picojet-flight app/flight.cpp app/clouds.cpp)

target_link_libraries(picojet-flight PRIVATE Jet picosdl)

set_source_files_properties(app/flight.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

# One mesh, built once at start-up and never replaced, so this is the F22 plus
# Scene's render queue for 200 triangles and nothing else.
target_compile_definitions(picojet-flight PRIVATE PICO_HEAP_SIZE=49152)
target_compile_definitions(picojet-flight PRIVATE
    PICOJET_SYS_CLOCK_KHZ=${PICOJET_SYS_CLOCK_KHZ})

pico_enable_stdio_uart(picojet-flight 1)
pico_enable_stdio_usb(picojet-flight 1)
pico_add_extra_outputs(picojet-flight)

# ------------------------------------------------------------------ JetExamples
#
# CubeCoders' JetExamples, vendored under examples/, each with its own scene and
# its own renderer configuration, running on a picosdl implementation of the
# upstream runtime contract (examples/runtime/).
#
# Jet is compiled once per example rather than once for the project. JetConfig.hpp
# is a set of compile-time switches and the examples disagree about them - depth
# buffering, sorting, lighting, texturing and post-processing are exactly what
# they demonstrate - so a single Jet build cannot serve them all.
#
# Each example's configuration is used as written, then examples/runtime/
# JetConfigPico.hpp overrides the frame layout, which belongs to the display: the
# examples render half-width interlaced fields for their scanout, this panel is
# pushed whole frames.

file(GLOB PICOJET_JET_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/Jet/src/*.cpp)

# picojet_jet_example(name title [FRAMEBUFFERS 1|2])
function(picojet_jet_example name title)
    cmake_parse_arguments(EX "" "FRAMEBUFFERS" "" ${ARGN})
    if (NOT EX_FRAMEBUFFERS)
        set(EX_FRAMEBUFFERS 2)
    endif()
    set(src ${CMAKE_CURRENT_LIST_DIR}/examples/${name})
    set(runtime ${CMAKE_CURRENT_LIST_DIR}/examples/runtime)
    set(cfg ${CMAKE_CURRENT_BINARY_DIR}/examples/${name})

    # Jet includes "JetConfig.hpp" by name. This one comes first on the path and
    # layers the frame-layout overrides onto the example's own configuration,
    # which is included by absolute path so it cannot be found by name instead.
    file(WRITE ${cfg}/JetConfig.hpp
"#pragma once
#include \"${src}/firmware/JetConfig.hpp\"
#include \"${runtime}/JetConfigPico.hpp\"
")

    add_library(jet-${name} STATIC ${PICOJET_JET_SOURCES})
    target_include_directories(jet-${name} PUBLIC ${cfg} ${CMAKE_CURRENT_LIST_DIR}/Jet/src)
    target_compile_options(jet-${name} PRIVATE -fno-math-errno)
    set_property(TARGET jet-${name} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

    file(GLOB example_sources CONFIGURE_DEPENDS ${src}/*.cpp)
    add_executable(example-${name} ${example_sources} ${runtime}/Runtime.cpp)
    target_include_directories(example-${name} PRIVATE ${cfg} ${src} ${runtime})
    target_compile_features(example-${name} PRIVATE cxx_std_17)
    # ESP-IDF's headers bring <cstdio> in transitively and some scenes rely on
    # it for std::printf; supplying it here keeps the scene sources unmodified.
    target_compile_options(example-${name} PRIVATE -fno-math-errno
        $<$<COMPILE_LANGUAGE:CXX>:-include> $<$<COMPILE_LANGUAGE:CXX>:cstdio>)
    target_compile_definitions(example-${name} PRIVATE
        PICOJET_SYS_CLOCK_KHZ=${PICOJET_SYS_CLOCK_KHZ}
        PICOJET_EXAMPLE_NAME="${title}"
        PICOJET_EXAMPLE_FRAMEBUFFERS=${EX_FRAMEBUFFERS})
    target_link_libraries(example-${name} PRIVATE
        jet-${name} picosdl pico_multicore hardware_watchdog)

    pico_enable_stdio_uart(example-${name} 1)
    pico_enable_stdio_usb(example-${name} 1)
    pico_add_extra_outputs(example-${name})
endfunction()

# Measured on the board. The first seven fit alongside two framebuffers; the
# last two need the second framebuffer's 128 KB for their render queues.
picojet_jet_example(template-cube    "Rotating cube")
picojet_jet_example(particles        "Particle Lab")
picojet_jet_example(textured-boxes   "Textured crate")
picojet_jet_example(texture-features "Texture Lab")
picojet_jet_example(postfx-crt       "CRT / Arcade")
picojet_jet_example(lod-billboards   "Woodland")
picojet_jet_example(sprite-controls  "Air Mail")
picojet_jet_example(tropical-island  "Tropical island" FRAMEBUFFERS 1)
picojet_jet_example(sprites-blending "After Hours"     FRAMEBUFFERS 1)
