#!/usr/bin/env python3
"""
Convert Wavefront .obj meshes and PNG textures into C arrays for picojet.

There is no filesystem on the board, so assets are compiled in. Jet's own
ObjLoader reads files and cannot be used here; this does the same job offline and
emits data the firmware can point at directly.

Two things are worth knowing about the output.

Meshes are DE-INDEXED. An .obj indexes position, uv and normal separately, so one
position can carry several uv/normal pairs; Jet's Object has a single vertex array
where each entry holds all three. Every distinct (v, vt, vn) triple therefore
becomes one Jet vertex, which is why the vertex counts below are larger than the
`v` count in the file.

Meshes land in flash as const arrays and are copied into an Object at load time,
which is where the RAM cost is. Textures stay in flash and are sampled in place -
they are read-only to Jet, so there is no reason to spend SRAM on them.
"""
import argparse
import os
import struct
import sys

from PIL import Image

# Jet fixed-point: TrigLUT.hpp's FIXED_POINT_SCALE. UVs are 0..1024 = 0..1.
FIXED_POINT_SCALE = 1024


def ident(path):
    """
    A C identifier from a filename.

    Asset packs name files with hyphens - race-future, tractor-shovel - which are
    valid in a filename and not in an identifier, so the emitted arrays would not
    compile. Everything that is not alphanumeric becomes an underscore, and the
    header file is named to match so the two never disagree.
    """
    stem = os.path.splitext(os.path.basename(path))[0]
    return "".join(c if c.isalnum() else "_" for c in stem)


def rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def convert_texture(path, out_dir, size, nearest=False):
    name = ident(path)
    img = Image.open(path).convert("RGB")
    if size and img.size != (size, size):
        # An atlas of flat colour patches must not be filtered down: smoothing
        # bleeds neighbouring patches into each other across their boundaries,
        # and every UV in the model points at a patch. Photographic textures want
        # the opposite, so the caller says which this is.
        img = img.resize((size, size), Image.NEAREST if nearest else Image.LANCZOS)
    w, h = img.size
    px = img.load()

    words = [rgb565(*px[x, y]) for y in range(h) for x in range(w)]

    out = os.path.join(out_dir, f"tex_{name}.h")
    with open(out, "w") as f:
        f.write(f"// Generated from {os.path.basename(path)} by tools/convert_assets.py\n")
        f.write("// RGB565, row-major. const so it stays in flash and is sampled in place.\n")
        f.write("#pragma once\n#include <cstdint>\n\n")
        f.write(f"#define TEX_{name.upper()}_W {w}\n")
        f.write(f"#define TEX_{name.upper()}_H {h}\n\n")
        f.write(f"static const uint16_t tex_{name}_data[{w * h}] = {{\n")
        for i in range(0, len(words), 12):
            f.write("\t" + ", ".join(f"0x{v:04X}" for v in words[i:i + 12]) + ",\n")
        f.write("};\n")
    return name, w, h, w * h * 2


def decimate(verts, tris, unpack, target):
    """
    Reduce a mesh to roughly `target` triangles by vertex clustering.

    Snap every vertex to a grid, merge the ones that land in the same cell, and
    drop the triangles that collapse to a line or a point as a result. Crude
    beside a quadric edge collapse, but it is a dozen lines, it never produces a
    hole, and it degrades a shape evenly - which is what a model being shown at
    a few hundred triangles needs.

    Position and normal are averaged within a cell. The UV is NOT: it is taken
    from whichever member vertex sits nearest the cell's centre.

    That matters for atlas textures, which is what a lot of low-poly kits ship -
    one image of flat colour patches, with every vertex pointing at the middle of
    a patch. Averaging the UVs of a cluster spanning two patches lands the sample
    between them and produces a colour that appears nowhere in the model. Taking
    one real vertex's UV always lands on a real patch. On a continuously mapped
    texture the nearest-to-centre UV is close to the average anyway, so nothing is
    lost there.
    """
    lo = [min(unpack(t)[0][i] for t in verts) for i in range(3)]
    hi = [max(unpack(t)[0][i] for t in verts) for i in range(3)]
    span = max(hi[i] - lo[i] for i in range(3)) or 1.0

    # Binary search the grid resolution that lands nearest the target.
    best = None
    lo_r, hi_r = 2, 256
    while lo_r <= hi_r:
        r = (lo_r + hi_r) // 2
        cell = span / r
        clusters, remap = {}, {}
        for i, t in enumerate(verts):
            pos, uv, nrm = unpack(t)
            key = tuple(int((pos[j] - lo[j]) / cell) for j in range(3))
            if key not in clusters:
                clusters[key] = [len(clusters), [0.0] * 3, [0.0] * 3, 0, []]
            c = clusters[key]
            for j in range(3):
                c[1][j] += pos[j]
                c[2][j] += nrm[j]
            c[3] += 1
            c[4].append((pos, uv))
            remap[i] = c[0]

        out_tris = []
        seen = set()
        for a, b, cc in tris:
            ra, rb, rc = remap[a], remap[b], remap[cc]
            if ra == rb or rb == rc or ra == rc:
                continue
            key = tuple(sorted((ra, rb, rc)))
            if key in seen:
                continue
            seen.add(key)
            out_tris.append((ra, rb, rc))

        if best is None or abs(len(out_tris) - target) < abs(best[2] - target):
            reps = [None] * len(clusters)
            for c in clusters.values():
                n = c[3]
                avg_pos = tuple(v / n for v in c[1])
                avg_nrm = tuple(v / n for v in c[2])
                # The UV of the member nearest the centre, never a blend of them.
                _, uv = min(c[4], key=lambda pu: sum((pu[0][j] - avg_pos[j]) ** 2
                                                     for j in range(3)))
                reps[c[0]] = (avg_pos, uv, avg_nrm)
            best = (reps, out_tris, len(out_tris))

        if len(out_tris) > target:
            hi_r = r - 1
        else:
            lo_r = r + 1

    return best[0], best[1]


def convert_mesh(path, out_dir, scale, max_tris=0):
    name = ident(path)
    positions, uvs, normals = [], [], []
    verts, vert_index, tris = [], {}, []

    for line in open(path):
        parts = line.split()
        if not parts:
            continue
        tag = parts[0]
        if tag == "v":
            positions.append(tuple(float(x) for x in parts[1:4]))
        elif tag == "vt":
            uvs.append(tuple(float(x) for x in parts[1:3]))
        elif tag == "vn":
            normals.append(tuple(float(x) for x in parts[1:4]))
        elif tag == "f":
            face = []
            for token in parts[1:]:
                if token not in vert_index:
                    vert_index[token] = len(verts)
                    verts.append(token)
                face.append(vert_index[token])
            # Fan-triangulate: these exports are already triangles, but a quad
            # would otherwise be silently dropped.
            for i in range(1, len(face) - 1):
                tris.append((face[0], face[i], face[i + 1]))

    def unpack(token):
        bits = (token.split("/") + ["", ""])[:3]
        vi = int(bits[0]) - 1
        ti = int(bits[1]) - 1 if bits[1] else None
        ni = int(bits[2]) - 1 if bits[2] else None
        pos = positions[vi]
        uv = uvs[ti] if ti is not None and ti < len(uvs) else (0.0, 0.0)
        nrm = normals[ni] if ni is not None and ni < len(normals) else (0.0, 1.0, 0.0)
        return pos, uv, nrm

    decimated = False
    if max_tris and len(tris) > max_tris:
        reps, tris = decimate(verts, tris, unpack, max_tris)
        # Replace the token-indexed vertex list with the clustered one, and make
        # unpack() read from it instead of the .obj arrays.
        verts = list(range(len(reps)))
        _reps = reps
        def unpack(i, _r=_reps):          # noqa: F811
            return _r[i]
        decimated = True

    # Normalise to a unit-ish size so every model arrives at a comparable scale
    # and the demo can place them all with one number.
    pts = [unpack(t)[0] for t in verts] if decimated else \
          [positions[int((t.split("/")[0])) - 1] for t in verts]
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]

    # Normalise by BOUNDING SPHERE, not by the longest axis.
    #
    # The longest axis makes a cube and an aircraft the same width, which is not
    # the same as making them look the same size: the cube fills its box and the
    # aircraft is mostly empty around a wingspan, so matching widths leaves the
    # cube far larger on screen - and its diagonal larger still once it rotates.
    # Matching the radius of the sphere that contains each model is what actually
    # frames them alike, and it is also what makes a single camera distance safe
    # for all of them however they spin.
    import math
    radius = max(math.dist(p, centre) for p in pts) or 1.0
    k = scale / radius

    half_y = round((hi[1] - lo[1]) / 2 * k)

    out = os.path.join(out_dir, f"mesh_{name}.h")
    with open(out, "w") as f:
        f.write(f"// Generated from {os.path.basename(path)} by tools/convert_assets.py\n")
        f.write("// De-indexed for Jet: one vertex per distinct (v, vt, vn) triple.\n")
        if decimated:
            f.write("// DECIMATED by vertex clustering to fit the render queue's per-triangle\n"
                    "// cost - see MODEL_RAM_BUDGET in app/main.cpp.\n")
        f.write("// Centred on the origin and scaled so the bounding sphere has a "
                f"radius of {scale}\n// world units - see the note in convert_assets.py.\n")
        f.write("#pragma once\n#include <cstdint>\n\n")
        f.write(f"#define MESH_{name.upper()}_VERTS {len(verts)}\n")
        f.write(f"#define MESH_{name.upper()}_TRIS  {len(tris)}\n")
        # How far the mesh reaches below its own centre, so a caller can sit it on
        # a surface instead of burying half of it. Flat models (aircraft) reach a
        # long way less than chunky ones, which is exactly why this is per-model.
        f.write(f"#define MESH_{name.upper()}_HALF_Y {half_y}\n\n")

        f.write("// x, y, z, u, v, nx, ny, nz - positions in world units, UVs in\n"
                f"// Jet fixed point (0..{FIXED_POINT_SCALE}), normals scaled the same way.\n")
        f.write(f"static const int16_t mesh_{name}_verts[{len(verts)}][8] = {{\n")
        for token in verts:
            pos, uv, nrm = unpack(token)
            x, y, z = ((pos[i] - centre[i]) * k for i in range(3))
            # .obj V runs bottom-up; Jet samples top-down.
            u = uv[0] * FIXED_POINT_SCALE
            v = (1.0 - uv[1]) * FIXED_POINT_SCALE
            nx, ny, nz = (n * FIXED_POINT_SCALE for n in nrm)
            f.write("\t{%d,%d,%d, %d,%d, %d,%d,%d},\n" % (
                round(x), round(y), round(z),
                round(u), round(v),
                round(nx), round(ny), round(nz)))
        f.write("};\n\n")

        idx_t = "uint16_t" if len(verts) <= 0xFFFF else "uint32_t"
        f.write(f"static const {idx_t} mesh_{name}_tris[{len(tris)}][3] = {{\n")
        for t in tris:
            f.write("\t{%d,%d,%d},\n" % t)
        f.write("};\n")

    flash = len(verts) * 8 * 2 + len(tris) * 3 * (2 if len(verts) <= 0xFFFF else 4)
    # What an Object costs in RAM once built: Jet's Vertex is pos+uv+normal as
    # int32/Vector, plus a Triangle per face.
    ram = len(verts) * 32 + len(tris) * 20
    return name, len(verts), len(tris), flash, ram


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tex-size", type=int, default=128)
    ap.add_argument("--tex-nearest", action="store_true",
                    help="resample textures with nearest neighbour (for colour atlases)")
    ap.add_argument("--mesh-scale", type=float, default=375.0,
                    help="bounding-sphere radius in world units, before JET32_WORLD_SCALE")
    ap.add_argument("--max-tris", type=int, default=0,
                    help="decimate meshes above this triangle count (0 = never)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    print("%-10s %7s %7s %10s %10s" % ("mesh", "verts", "tris", "flash KB", "RAM KB"))
    total_flash = 0
    for f in sorted(os.listdir(args.assets)):
        if f.endswith(".obj"):
            n, nv, nt, fl, ram = convert_mesh(os.path.join(args.assets, f),
                                              args.out, args.mesh_scale, args.max_tris)
            total_flash += fl
            print("%-10s %7d %7d %10.1f %10.1f" % (n, nv, nt, fl / 1024, ram / 1024))

    print()
    print("%-10s %7s %7s %10s" % ("texture", "w", "h", "flash KB"))
    for f in sorted(os.listdir(args.assets)):
        if f.endswith(".png"):
            n, w, h, fl = convert_texture(os.path.join(args.assets, f),
                                          args.out, args.tex_size, args.tex_nearest)
            total_flash += fl
            print("%-10s %7d %7d %10.1f" % (n, w, h, fl / 1024))

    print("\ntotal flash: %.1f KB" % (total_flash / 1024))


if __name__ == "__main__":
    sys.exit(main())
