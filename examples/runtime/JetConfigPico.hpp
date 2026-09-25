// JetConfigPico.hpp - applied on top of each example's own JetConfig.hpp.
//
// Every example keeps its own renderer configuration: its lighting, texturing,
// depth, sorting and post-processing choices are what the example demonstrates.
// What changes is only the frame layout, which belongs to the display rather
// than to the scene.
//
// Upstream renders half-width, interlaced fields and doubles and interleaves them
// during scanout. This panel is pushed whole RGB565 frames straight out of the
// framebuffer, with no expansion pass between the rasteriser and the glass, so
// the rasteriser writes full-width full-height frames instead.
#pragma once

#undef  HALF_WIDTH_BUFFERS
#define HALF_WIDTH_BUFFERS 0

#undef  FIELD_BUFFERS
#define FIELD_BUFFERS 0

/* Field reflections read the other field; with whole frames the runtime points
 * reflectBuffer at the previous frame instead, which serves the same purpose. */
#undef  SSR_FIELD_REFLECT
#define SSR_FIELD_REFLECT 0
