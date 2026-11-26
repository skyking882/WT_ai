"""Exact hull-height obstacle raster of a ground map from the game's own collision geometry.

For every pregenerated rendinst in the scan rectangle (build_ri_obstacles.rigz_decode) and every node of its
CollisionResource (colres.ColRes: mesh triangles, box nodes as 12 triangles), the geometry is cut with the slab
BAND above the instance base (where a hull is), projected on xz and rasterized at CELL m. Each cell keeps the
hardest class that touches it:
  0 free, 1 soft (bush / tree / destructionImpulse <= --soft), 2 medium (larger destructionImpulse, or hp only),
  3 hard (immortal, not in the destruction table, or listed without any destruction parameter).
Thresholds are NOT calibrated against tanks yet (trees can basically all be knocked down).
Sampling: clipped polygons are sampled at STEP m (edges always, interiors when they have area), so a wall is
marked in every cell it passes through up to STEP.

Output: <out>/<level>_ri_raster.npz (u8 classes, x0, z0, cell), a stats json, and review PNGs.
usage: build_ri_raster.py --level avg_sweden (--scan <obstacles/x.bin> | --box x0 z0 step nx nz)
       --dmg <common/rendinst_dmg.blk> --out <dir> [--soft 20000] [--crop x0 z0 x1 z1] [--terrain <terrain/x.bin>]
"""
import argparse
import json
import math
import struct
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_ri_obstacles as bro  # noqa: E402
import scan_area  # noqa: E402
from colres import ColRes  # noqa: E402

BAND = (0.25, 2.6)
CELL = 0.25
STEP = 0.1
BOX_TRIS = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
            (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]


def node_triangles(nd):
    """(verts (n,3) in the model frame, tris (m,3)) of a node, or None."""
    tm = np.array(nd["tm"], dtype=np.float64).reshape(4, 3)
    if nd["tris"]:
        v = np.array(nd["verts"], dtype=np.float64)
        t = np.array(nd["tris"], dtype=np.int64)
    elif nd["type"] == 2:
        lo, hi = nd["lo"], nd["hi"]
        v = np.array([(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        t = np.array(BOX_TRIS, dtype=np.int64)
    else:
        return None
    return v @ tm[:3] + tm[3], t


def clip_slab(tri, lo, hi):
    """Polygon of a triangle (3x3) inside lo <= y <= hi (list of points)."""
    poly = [tuple(p) for p in tri]
    for bound, keep_above in ((lo, True), (hi, False)):
        out = []
        n = len(poly)
        for k in range(n):
            a, b = poly[k], poly[(k + 1) % n]
            ina = a[1] >= bound if keep_above else a[1] <= bound
            inb = b[1] >= bound if keep_above else b[1] <= bound
            if ina:
                out.append(a)
            if ina != inb:
                t = (bound - a[1]) / (b[1] - a[1])
                out.append(tuple(a[i] + t * (b[i] - a[i]) for i in range(3)))
        poly = out
        if not poly:
            return []
    return poly


def sample_polygon(poly2):
    """Points (k,2) covering a convex 2D polygon: its edges every STEP and, if it has area, a STEP grid inside."""
    pts = []
    n = len(poly2)
    for k in range(n if n > 2 else n - 1):
        a, b = poly2[k], poly2[(k + 1) % n]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        m = max(1, int(L / STEP) + 1)
        for i in range(m + 1):
            s = i / m
            pts.append((a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])))
    if n >= 3:
        area = 0.0
        for k in range(n):
            a, b = poly2[k], poly2[(k + 1) % n]
            area += a[0] * b[1] - a[1] * b[0]
        if abs(area) / 2 > STEP * STEP:
            sgn = 1 if area > 0 else -1
            xs = [p[0] for p in poly2]
            zs = [p[1] for p in poly2]
            gx = np.arange(min(xs), max(xs) + STEP, STEP)
            gz = np.arange(min(zs), max(zs) + STEP, STEP)
            X, Z = np.meshgrid(gx, gz)
            inside = np.ones(X.shape, bool)
            for k in range(n):
                a, b = poly2[k], poly2[(k + 1) % n]
                inside &= sgn * ((b[0] - a[0]) * (Z - a[1]) - (b[1] - a[1]) * (X - a[0])) >= -1e-9
            pts.extend(zip(X[inside].tolist(), Z[inside].tolist()))
    return pts


def section_points(verts, tris, lo, hi):
    """Sample points (k,2) (x,z) of the geometry inside the slab."""
    ys = verts[:, 1]
    tri_lo = ys[tris].min(axis=1)
    tri_hi = ys[tris].max(axis=1)
    sel = np.nonzero((tri_hi >= lo) & (tri_lo <= hi))[0]
    pts = []
    for k in sel:
        poly = clip_slab(verts[tris[k]], lo, hi)
        if len(poly) >= 2:
            pts.extend(sample_polygon([(p[0], p[2]) for p in poly]))
    return np.array(pts, dtype=np.float64).reshape(-1, 2)


def material_class(cls, impulse, params, soft):
    if cls in ("bush", "tree"):
        return 1
    if cls == "impulse":
        return 1 if impulse <= soft else 2
    if cls == "unknown" and params and "hp" in params:
        return 2
    return 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", required=True)
    ap.add_argument("--scan", help="WTOBS001 obstacle scan: the raster covers its rectangle")
    ap.add_argument("--box", type=float, nargs=5, metavar=("X0", "Z0", "STEP", "NX", "NZ"),
                    help="without a scan: the rectangle the scan would have (prep/scan_area.py)")
    ap.add_argument("--dmg", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--soft", type=float, default=20000.0)
    ap.add_argument("--crop", type=float, nargs=4)
    ap.add_argument("--terrain", help="WTTER001 terrain file: the band is measured from the ground under each "
                                      "instance (else from the instance base)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    codec = bro.cr.Codecs(str(bro.CDK))
    inst = bro.rigz_decode(codec, (bro.WT / "levels" / f"{a.level}.bin").read_bytes())
    nx, nz, sx0, sz0, sstep, raw = scan_area.grid_from_args(a)
    gx, gz = int(nx * sstep / CELL), int(nz * sstep / CELL)
    grid = np.zeros((gz, gx), np.uint8)
    col = bro.Collision(codec)
    dmg = bro.parse_dmg(a.dmg)
    terr = None
    if a.terrain and Path(a.terrain).is_file():
        tb = Path(a.terrain).read_bytes()
        tnx, tnz = struct.unpack_from("<II", tb, 8)
        tx0, tz0, tstep, toff, tscale = struct.unpack_from("<5f", tb, 16)
        th = np.frombuffer(tb, dtype="<u2", count=tnx * tnz, offset=48).reshape(tnz, tnx).astype(np.float64) * tscale + toff
        terr = (th, tx0, tz0, tstep)

    def ground(x, z):
        if terr is None:
            return None
        th, tx0, tz0, tstep = terr
        fx, fz = (x - tx0) / tstep, (z - tz0) / tstep
        i, j = int(math.floor(fx)), int(math.floor(fz))
        if not (0 <= i < th.shape[1] - 1 and 0 <= j < th.shape[0] - 1):
            return None
        u, v = fx - i, fz - j
        return (th[j, i] * (1 - u) * (1 - v) + th[j, i + 1] * u * (1 - v)
                + th[j + 1, i] * (1 - u) * v + th[j + 1, i + 1] * u * v)

    res_cache = {}
    sec_cache = {}
    stats = Counter()
    for name, px, py, pz, ax, ay, az in inst:
        if not (sx0 - 30 <= px <= sx0 + nx * sstep + 30 and sz0 - 30 <= pz <= sz0 + nz * sstep + 30):
            continue
        stats["instances"] += 1
        if name not in res_cache:
            body = col.raw(name)
            try:
                res_cache[name] = ColRes(body) if body else None
            except (ValueError, struct.error, IndexError) as e:
                print("parse failed", name, e)
                res_cache[name] = None
        cr_ = res_cache[name]
        if cr_ is None:
            stats["no_collision"] += 1
            continue
        params = dmg.get(name.lower())
        cls, imp = bro.classify(params)
        mcls = material_class(cls, imp, params, a.soft)
        A = np.array([ax, ay, az], dtype=np.float64)       # rows: model axes in world
        sy = float(np.linalg.norm(A[1]))
        upright = abs(A[1][1]) >= 0.98 * sy and abs(A[0][1]) < 0.02 * sy + 1e-6 and abs(A[2][1]) < 0.02 * sy + 1e-6
        gy = ground(px, pz)
        dy = 0.0 if gy is None else round((gy - py) * 4) / 4   # ground above (+) or below the instance base
        if gy is not None:
            stats["band_from_terrain"] += 1
            if abs(dy) > 1.0:
                stats["base_off_ground_gt_1m"] += 1
        if upright:
            key = (name, round(sy, 2), dy)
            if key not in sec_cache:
                pts = []
                for nd in cr_.nodes:
                    g = node_triangles(nd)
                    if g is not None:
                        pts.append(section_points(g[0], g[1], (dy + BAND[0]) / sy, (dy + BAND[1]) / sy))
                sec_cache[key] = np.concatenate(pts) if pts else np.zeros((0, 2))
            p2 = sec_cache[key]
            if not len(p2):
                stats["above_or_below_band"] += 1
                continue
            wx = px + p2[:, 0] * A[0][0] + p2[:, 1] * A[2][0]
            wz = pz + p2[:, 0] * A[0][2] + p2[:, 1] * A[2][2]
        else:
            stats["tilted"] += 1
            pts = []
            for nd in cr_.nodes:
                g = node_triangles(nd)
                if g is not None:
                    w = g[0] @ A + np.array([px, py, pz])
                    pts.append(section_points(w, g[1], py + dy + BAND[0], py + dy + BAND[1]))
            p2 = np.concatenate(pts) if pts else np.zeros((0, 2))
            if not len(p2):
                stats["above_or_below_band"] += 1
                continue
            wx, wz = p2[:, 0], p2[:, 1]
        ci = ((wx - sx0) / CELL).astype(np.int64)
        cj = ((wz - sz0) / CELL).astype(np.int64)
        ok = (ci >= 0) & (ci < gx) & (cj >= 0) & (cj < gz)
        np.maximum.at(grid, (cj[ok], ci[ok]), mcls)
        stats[f"class{mcls}_instances"] += 1
    np.savez_compressed(out / f"{a.level}_ri_raster.npz", grid=grid, x0=sx0, z0=sz0, cell=CELL)
    cells = Counter({int(k): int(v) for k, v in zip(*np.unique(grid, return_counts=True))})
    summary = {"level": a.level, "seconds": round(time.time() - t0, 1), "grid": [gx, gz], "cell": CELL,
               "band": BAND, "soft_impulse": a.soft, "stats": stats, "cells_by_class": cells,
               "resources": len(res_cache), "sections_cached": len(sec_cache)}
    (out / f"{a.level}_ri_raster.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary, indent=1))
    # review render: scan solid grey underneath (nothing without a scan), classes on top
    sg = np.frombuffer(raw, dtype="<u2", count=nx * nz, offset=28).reshape(nz, nx) if raw is not None \
        else np.zeros((nz, nx), np.uint16)
    kind, h = (sg >> 8) & 7, (sg & 63) * 0.5
    solid = ((sg & 0xC0) != 0) | (kind == 2) | ((h >= 0.5) & (h <= 3.5))
    rep = int(round(sstep / CELL))
    base = np.kron(solid, np.ones((rep, rep), bool))[:gz, :gx]
    rgb = np.full((gz, gx, 3), 255, np.uint8)
    rgb[base] = (200, 200, 200)
    for k, c in ((1, (40, 170, 40)), (2, (240, 150, 0)), (3, (200, 20, 20))):
        rgb[grid == k] = c
    crop = a.crop
    if crop:
        i0, j0 = int((crop[0] - sx0) / CELL), int((crop[1] - sz0) / CELL)
        i1, j1 = int((crop[2] - sx0) / CELL), int((crop[3] - sz0) / CELL)
        Image.fromarray(rgb[j0:j1, i0:i1][::-1]).save(out / f"{a.level}_ri_raster_crop.png")
    Image.fromarray(rgb[::-1][::4, ::4]).save(out / f"{a.level}_ri_raster_full_1m.png")


if __name__ == "__main__":
    main()
