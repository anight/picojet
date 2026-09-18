// models.hpp - the imported meshes and their textures, as one table.
//
// Two sources, both free to use:
//
//   * the meshes shipped with the 3d-renderer-in-c reference project, which come
//     from Gustavo Pezzi's pikuma.com course;
//   * OpenGameArt (opengameart.org), CC0 - the radio, the advertising column and
//     the biplane, each with a painted diffuse texture of its own.
//
// Everything here lives in flash: the vertex and index arrays are const, and the
// textures are sampled in place because Jet only ever reads texel data. Together
// they are about 510 KB of a 4 MB part, which is the cheap half of importing a
// model.
//
// The expensive half is RAM. Jet's Object owns std::vectors, so loading a mesh
// copies it out of flash into the heap - and the render queue Scene builds each
// frame grows with the triangles that survive culling, on top of that. A model is
// therefore affordable or not depending on its triangle count twice over, which
// is why the viewer in main.cpp loads exactly one at a time and checks the cost
// before it tries.
#pragma once

#include <cstdint>
#include <cstddef>

#include "assets/mesh_cube.h"
#include "assets/mesh_f117.h"
#include "assets/mesh_f22.h"
#include "assets/mesh_efa.h"
#include "assets/mesh_sphere.h"
#include "assets/mesh_crab.h"

#include "assets/tex_cube.h"
#include "assets/tex_f117.h"
#include "assets/tex_f22.h"
#include "assets/tex_efa.h"
#include "assets/tex_crab.h"
#include "assets/tex_pikuma.h"

/*
 * OpenGameArt, all CC0, and each with a texture of its own rather than a shared
 * atlas of flat colour. That is the difference worth having here: an atlas makes
 * a model read as solid-coloured panels, where a painted diffuse puts detail on
 * a surface that only has a few hundred triangles to give - which is the whole
 * trick this renderer is for.
 */
#include "assets/mesh_radio.h"
#include "assets/tex_radio.h"
#include "assets/mesh_column.h"
#include "assets/tex_column.h"
#include "assets/mesh_biplane.h"
#include "assets/tex_biplane.h"



struct ModelDef {
	const char      *name;
	const int16_t  (*verts)[8];   ///< x,y,z,u,v,nx,ny,nz - see convert_assets.py
	int              vertCount;
	const uint16_t (*tris)[3];
	int              triCount;
	const uint16_t  *texels;      ///< flash-resident RGB565
	int              texW, texH;
	int              halfY;       ///< how far it reaches below its own centre
};

/* The generated size macros are upper case and the arrays lower, and the
 * preprocessor cannot change case - so each entry names both. */
#define MODEL(n, N, tex, TEX) \
	{ #n, mesh_##n##_verts, MESH_##N##_VERTS, mesh_##n##_tris, MESH_##N##_TRIS, \
	  tex_##tex##_data, TEX_##TEX##_W, TEX_##TEX##_H, MESH_##N##_HALF_Y }

/*
 * Ordered by triangle count, which is the order they stop being affordable in -
 * so the viewer walks from the ones that run well toward the ones that do not,
 * and the first failure is also the first model it cannot fit.
 *
 * cube reuses pikuma's texture: the original demo's cube.png is a UV grid, and
 * the pikuma logo shows the mapping more clearly on twelve triangles.
 */
static const ModelDef g_models[] = {
	MODEL(cube,   CUBE,   pikuma, PIKUMA),
	MODEL(f117,   F117,   f117,   F117),
	MODEL(f22,    F22,    f22,    F22),
	MODEL(efa,    EFA,    efa,    EFA),
	MODEL(sphere, SPHERE, crab,   CRAB),   /* sphere.obj ships no texture of its own */
	MODEL(crab,   CRAB,   crab,   CRAB),

	/* OpenGameArt, CC0, each with its own painted diffuse. */
	MODEL(radio,   RADIO,   radio,   RADIO),
	MODEL(column,  COLUMN,  column,  COLUMN),
	MODEL(biplane, BIPLANE, biplane, BIPLANE),
};

#undef MODEL

static constexpr int MODEL_COUNT = (int)(sizeof(g_models) / sizeof(g_models[0]));
