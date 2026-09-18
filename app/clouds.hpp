// clouds.hpp - a flight through a cloud deck, as a background for the renderer.
//
// Two horizontal planes above the camera and one below, textured with a
// precomputed tileable fBm noise tile. For a pinhole camera a horizontal plane
// maps to the screen so that on a given scanline the distance to the plane is
// constant and the texture coordinate is linear in x - so a row costs one divide
// of setup and then one add per axis per pixel. That is the old floor mapper, and
// it is why a full-screen background is affordable on this part at all.
//
// Colour is never computed per pixel. A ladder indexed by [fog band][density] is
// built once and already contains the sky gradient, cloud shading, coverage curve
// and distance haze; a pixel costs two texture fetches and one ladder fetch.
//
// It fills every pixel it is given, so it replaces clearing the framebuffer
// rather than being drawn over a cleared one.
//
// The algorithm is from tools/clouds.c, reduced to the parts a
// frame needs: the PNG writer, the benchmark and the scripted flight path are
// not here, and the base noise tile is 128 square rather than 256 so the tables
// fit beside two framebuffers.
#pragma once

#include <cstdint>

/* Build the noise tile and the colour ladders. Call once, before any render. */
void clouds_init(void);

/*
 * Start a frame: fix the heading, drift the camera, and work out the fog and
 * glow state every scanline will read.
 *
 * `yaw` is the heading in radians and `dt` the seconds since the last frame,
 * which is how far the camera moves forward along that heading.
 *
 * Everything this writes is read-only for the duration of the band calls below,
 * so those may run concurrently on both cores.
 */
void clouds_begin_frame(float yaw, float dt);

/*
 * Fill rows [y0, y1) of an RGB565 framebuffer. `pitch` is in pixels.
 *
 * Safe to call from two cores at once with disjoint row ranges: a row depends
 * only on the frame state and its own y.
 */
void clouds_render_band(uint16_t *fb, int pitch, int y0, int y1);

/* Where the horizon sits, in rows. The scene's camera is aimed to match. */
float clouds_horizon(void);
