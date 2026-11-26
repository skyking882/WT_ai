"""Terrain heights of a mission -> data/terrain/<mission>.bin (WTTER001). WINDOWS ONLY, needs the installed game.

usage: python prep/gen_terrain.py <mission> [<mission> ...] [--runs DIR [DIR ...]] [--no-runs]

The heights are the level's HM2 samples unchanged (prep/extract_terrain.py decodes levels/<level>.bin with the game
CDK's codec DLL; offset / scale are the HM2's own), cropped to the mission's scan rectangle united with its battle
rectangle (ground_battle_areas.json "bbox"), plus MARGIN on every side. Without an obstacle scan the scan rectangle
is the one the scan would have (prep/scan_area.py: battle rectangle + 150 m), so the crop is the same either way.

Deck layer (optional): from logged battles that have ground_samples.csv (columns t, seat, x, y, z; one row per tank
and sample). A sample whose logged Y is more than DECK_M above the HM2 ground was on something the heightfield does
not have (a bridge deck, a causeway); the track from the sample before to the sample after (same seat, < 4 s and
< 40 m apart) is marked with a DECK_RADIUS band, so the ramps are included. prep/build_map30.py does not apply the
water and slope rules on marked samples. Battles are found under --runs (default data/runs/): every folder below
that holds ground_samples.csv (>= MIN_SAMPLES_BYTES) and ground_plan.json whose "level" is the mission's level
(a battle copied twice, same folder name and size, counts once). No such battle, or --no-runs: no deck layer
(flags bit 0 clear) and the output says so.

File layout, little endian:
  0  char[8] "WTTER001"
  8  u32 nx (samples along +X)       12 u32 nz (samples along +Z)
  16 f32 x0, 20 f32 z0                world position of sample (0, 0)
  24 f32 step                         metres between samples
  28 f32 height_offset, 32 f32 height_scale   Y = offset + u16 * scale
  36 f32 water_level                  NaN: no water rule on this map
  40 u32 flags (bit 0: a sample flag plane follows the heights), 44 u32 reserved (0)
  48 u16[nz][nx] heights, row = Z
  then, with flag bit 0: u8[nz][nx] sample flags, bit 0 = deck (see above).
"""
import argparse
import csv
import json
import math
import os
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
import catalog_reader as cr  # noqa: E402
import extract_terrain  # noqa: E402
import paths  # noqa: E402
import scan_area  # noqa: E402

MARGIN = 300.0

# Water surface height per level (m); None = no water rule. The values are per level and were found by observation
# (where tanks in logged battles drowned and where they stood on low ground and lived); a level without logged
# battles on low ground has sea level 0 assumed. Levels not listed: no water rule.
WATER = {
    "avg_poland": 0.0, "avg_eastern_europe": 0.0, "avg_western_europe": None, "avg_fulda": None,
    "avg_japan": None, "avg_red_desert": None, "avg_finland": 0.0, "avg_normandy": 0.0, "avg_sweden": 0.0,
    "avg_tunisia_desert": 0.0, "avg_abandoned_factory": 0.0, "avg_american_valley": 0.0, "avg_korea_lake": None,
}

# Per mission, overrides WATER (found by observation as well). eastern_europe_02_dom: the heightfield has water under
# bridges and causeways that it does not show; with the water rule one spawn group is cut off from the rest of the
# map, so the rule is off for this mission.
MISSION_WATER = {"eastern_europe_02_dom": None}

DECK_M = 0.8              # logged Y this far above the HM2 ground: the tank was on something else
DECK_RADIUS = 5.0         # metres marked around the track through such a sample
MIN_SAMPLES_BYTES = 20000  # ground_samples.csv files smaller than this (aborted battles) are not used


class Terrain:
    """Full-level heights as float32 metres, bilinear lookup (NaN outside)."""

    def __init__(self, raw, meta):
        self.cell = float(meta["cell_size_m"])
        self.x0 = float(meta["origin_x_m"])
        self.z0 = float(meta["origin_z_m"])
        self.h = (np.float32(meta["height_offset_m"])
                  + raw.astype(np.float32) * np.float32(meta["height_scale_m"] / 65535)).astype(np.float32)
        self.rows, self.cols = self.h.shape

    def height(self, x, z):
        x = np.asarray(x, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)
        fx = (x - self.x0) / self.cell
        fz = (z - self.z0) / self.cell
        i = np.floor(fx).astype(np.int64)
        j = np.floor(fz).astype(np.int64)
        ok = (i >= 0) & (j >= 0) & (i < self.cols - 1) & (j < self.rows - 1)
        ic = np.clip(i, 0, self.cols - 2)
        jc = np.clip(j, 0, self.rows - 2)
        tx = fx - ic
        tz = fz - jc
        h = self.h
        v = ((1 - tx) * (1 - tz) * h[jc, ic] + tx * (1 - tz) * h[jc, ic + 1]
             + (1 - tx) * tz * h[jc + 1, ic] + tx * tz * h[jc + 1, ic + 1])
        return np.where(ok, v, np.nan)


def load_samples(path):
    """ground_samples.csv -> array (n, 5): t, seat, x, y, z (rows that do not parse are skipped)."""
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append((float(row["t"]), int(row["seat"]), float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, ValueError):
                pass
    return np.array(rows, dtype=np.float64) if rows else np.zeros((0, 5))


def find_runs(roots):
    """level -> [run folder] of every logged battle with ground_samples.csv below the given folders."""
    by_level = {}
    seen = set()
    for root in roots:
        for dirpath, _dirnames, filenames in os.walk(root):
            if "ground_samples.csv" not in filenames or "ground_plan.json" not in filenames:
                continue
            size = os.path.getsize(os.path.join(dirpath, "ground_samples.csv"))
            if size < MIN_SAMPLES_BYTES:
                continue
            key = (os.path.basename(dirpath), size)
            if key in seen:
                continue                     # the same battle copied into another folder
            seen.add(key)
            try:
                with open(os.path.join(dirpath, "ground_plan.json"), encoding="utf-8") as f:
                    level = json.load(f).get("level", "?")
            except (OSError, ValueError):
                level = "?"
            by_level.setdefault(level, []).append((size, dirpath))
    return {k: [p for _, p in sorted(v, reverse=True)] for k, v in by_level.items()}


def deck_mask(terrain, runs, x0, z0, step, nx, nz):
    """Samples on or beside a logged track that ran above the HM2 ground (see the module text)."""
    mask = np.zeros((nz, nx), bool)
    xs = x0 + np.arange(nx) * step
    zs = z0 + np.arange(nz) * step
    marked = 0
    for run in runs:
        s = load_samples(os.path.join(run, "ground_samples.csv"))
        if not len(s):
            continue
        s = s[np.abs(s[:, 2]) < 30000]
        s = s[np.lexsort((s[:, 0], s[:, 1]))]  # per seat, in time order: neighbours are the same track
        g = terrain.height(s[:, 2], s[:, 4])
        above = s[:, 3] - g
        for k in np.where(above > DECK_M)[0]:
            pts = [(s[k, 2], s[k, 4])]
            for j in (k - 1, k + 1):
                if 0 <= j < len(s) and s[j, 1] == s[k, 1] and abs(s[j, 0] - s[k, 0]) < 4.0 \
                        and math.hypot(s[j, 2] - s[k, 2], s[j, 4] - s[k, 4]) < 40.0:
                    pts.append((s[j, 2], s[j, 4]))
            for (ax, az) in pts:
                bx, bz = s[k, 2], s[k, 4]
                lo_x, hi_x = min(ax, bx) - DECK_RADIUS, max(ax, bx) + DECK_RADIUS
                lo_z, hi_z = min(az, bz) - DECK_RADIUS, max(az, bz) + DECK_RADIUS
                c0, c1 = np.searchsorted(xs, lo_x), np.searchsorted(xs, hi_x)
                r0, r1 = np.searchsorted(zs, lo_z), np.searchsorted(zs, hi_z)
                if c1 <= c0 or r1 <= r0:
                    continue
                X, Z = np.meshgrid(xs[c0:c1], zs[r0:r1])
                ux, uz = bx - ax, bz - az
                l2 = ux * ux + uz * uz
                tt = np.clip(((X - ax) * ux + (Z - az) * uz) / l2, 0, 1) if l2 > 1e-6 else np.zeros_like(X)
                d = np.hypot(X - (ax + tt * ux), Z - (az + tt * uz))
                mask[r0:r1, c0:c1] |= d <= DECK_RADIUS
            marked += 1
    return mask, marked


def write(mission, areas, hm2, runs_by_level):
    if mission not in areas:
        print(f"{mission}: not in {paths.BATTLE_AREAS.name}: skipped")
        return False
    level = areas[mission]["level"]
    if level not in hm2:
        hm2[level] = extract_terrain.level_hm2(level, hm2.setdefault("_codec", cr.Codecs(str(paths.cdk_dir()))))
    raw, meta = hm2[level]
    step = float(meta["cell_size_m"])
    ox, oz = float(meta["origin_x_m"]), float(meta["origin_z_m"])
    nx_s, nz_s, sx0, sz0, sstep, has_scan = scan_area.header(mission)
    boxes = [(sx0, sz0, sx0 + nx_s * sstep, sz0 + nz_s * sstep), tuple(areas[mission]["bbox"])]
    x0 = min(b[0] for b in boxes) - MARGIN
    z0 = min(b[1] for b in boxes) - MARGIN
    x1 = max(b[2] for b in boxes) + MARGIN
    z1 = max(b[3] for b in boxes) + MARGIN
    rows, cols = raw.shape
    c0 = max(0, int(math.floor((x0 - ox) / step)))
    r0 = max(0, int(math.floor((z0 - oz) / step)))
    c1 = min(cols - 1, int(math.ceil((x1 - ox) / step)))
    r1 = min(rows - 1, int(math.ceil((z1 - oz) / step)))
    crop = np.ascontiguousarray(raw[r0:r1 + 1, c0:c1 + 1].astype("<u2"))
    water = MISSION_WATER[mission] if mission in MISSION_WATER else WATER.get(level)
    fx0, fz0 = ox + c0 * step, oz + r0 * step
    runs = runs_by_level.get(level, [])
    if runs:
        deck, marked = deck_mask(Terrain(raw, meta), runs, fx0, fz0, step, crop.shape[1], crop.shape[0])
        flags, tail = 1, deck.astype(np.uint8).tobytes()
        deck_note = f"deck layer from {len(runs)} logged battle(s): {marked} samples above the ground, " \
                    f"{deck.mean() * 100:.2f}% of the samples marked"
    else:
        flags, tail = 0, b""
        deck_note = f"no deck layer (no logged battle with ground_samples.csv of {level}): flags bit 0 clear"
    header = b"WTTER001" + struct.pack("<IIffffffII", crop.shape[1], crop.shape[0], fx0, fz0, step,
                                       float(meta["height_offset_m"]), float(meta["height_scale_m"]) / 65535.0,
                                       float("nan") if water is None else float(water), flags, 0)
    assert len(header) == 48
    paths.TERRAIN.mkdir(parents=True, exist_ok=True)
    target = paths.TERRAIN / f"{mission}.bin"
    target.write_bytes(header + crop.tobytes() + tail)
    print(f"{mission}: {level}, {crop.shape[1]} x {crop.shape[0]} samples at {step} m from ({fx0:.0f}, {fz0:.0f}), "
          f"crop from the {'obstacle scan' if has_scan else 'scan rectangle without a scan'} + battle rectangle "
          f"+ {MARGIN:.0f} m, water {water}; {deck_note} -> data/terrain/{target.name} ({target.stat().st_size} bytes)",
          flush=True)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("missions", nargs="+")
    ap.add_argument("--runs", nargs="+", type=Path, help="folders holding logged battles (default data/runs/)")
    ap.add_argument("--no-runs", action="store_true", help="no deck layer")
    a = ap.parse_args()
    if os.name != "nt":
        raise SystemExit("gen_terrain.py needs Windows and the installed game (daKernel-dev.dll)")
    areas = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))
    roots = [] if a.no_runs else [r for r in (a.runs or [paths.RUNS]) if r.is_dir()]
    runs_by_level = find_runs(roots)
    hm2 = {}
    bad = 0
    for m in a.missions:
        try:
            bad += not write(m, areas, hm2, runs_by_level)
        except Exception as e:  # noqa: BLE001 (report and go on with the next mission)
            print(f"{m}: FAILED {e}", flush=True)
            bad += 1
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
