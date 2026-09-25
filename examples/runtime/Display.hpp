// Display.hpp - the output geometry the JetExamples scenes lay themselves out
// against.
//
// Upstream this header also owns panel init, framebuffers and scanout. Here all
// of that belongs to Runtime.cpp and picosdl, and the scenes only ever read the
// dimensions - so the dimensions are all this provides.
//
// The scenes were authored for 480x320. They derive their projection from these
// values, so they adapt to 320x200; anything placed at fixed pixel coordinates
// does not, and lands where it lands.
#pragma once

namespace Display {

constexpr int SCREEN_WIDTH  = 320;
constexpr int SCREEN_HEIGHT = 200;
constexpr int EDGE_INSET_X  = 0;
constexpr int EDGE_INSET_Y  = 0;
constexpr int RENDER_WIDTH  = SCREEN_WIDTH  - EDGE_INSET_X;
constexpr int RENDER_HEIGHT = SCREEN_HEIGHT - EDGE_INSET_Y;
constexpr int RENDER_TOP    = (SCREEN_HEIGHT - RENDER_HEIGHT) / 2;
constexpr int RENDER_LEFT   = (SCREEN_WIDTH  - RENDER_WIDTH ) / 2;

/* Whole frames, not interlaced fields: the panel is pushed a complete image. */
constexpr int RENDER_FIELDS = 1;

} // namespace Display
