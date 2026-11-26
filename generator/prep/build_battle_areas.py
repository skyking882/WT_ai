"""Battle area, capture zones and tank spawns of every tank mission -> data/common/ground_battle_areas.json.

usage: python prep/build_battle_areas.py MISSIONS_DIR [OUT_JSON]
  MISSIONS_DIR  the game's tank mission folder (gamedata/missions/cta/tanks inside mis.vromfs.bin) unpacked to JSON,
                one <location>/<mission>.blk file per mission with JSON content, as a BLK unpacker such as
                wt_ext_cli writes it (its JSON output format). The unpacker is not part of this tree.
  OUT_JSON      default data/common/ground_battle_areas.json (paths.BATTLE_AREAS)

A location has several layouts (e.g. tunisia_* and tunisia_02_*, Dom / Conq1-3 / Bttl), each with its own, often much
smaller, battle area inside the same level. A mission's areas come from the file itself plus its imports
(battleareas/ and mainareas/ templates, resolved inside MISSIONS_DIR, case-insensitive). Areas named *battle_area*
are a Box (unit cube scaled by tm: full size = axis length) or a Cylinder / Sphere (radius = axis length; a circle
becomes a 24-gon). "hardcore" (RB / SB) is preferred over "arcade". Only missions on an avg_* level are written.
Output per mission: level, area name, XZ polygon, bounding box, size, capture zone centres / radii (circles; the
other mode's copies left out) and tank spawn centres per team (areas t1_tankspawn* / t2_tankspawn*).
"""
import json
import math
import re
import sys
from pathlib import Path

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import paths  # noqa: E402


def load(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def resolve(root, game_path):
    # "gameData/missions/cta/tanks/red_desert/x.blk" -> ROOT/red_desert/x.blk (case-insensitive)
    rel = re.sub(r"(?i)^gamedata/missions/cta/tanks/", "", game_path.replace("\\", "/"))
    p = root
    for part in rel.split("/"):
        hit = [c for c in p.iterdir() if c.name.lower() == part.lower()] if p.is_dir() else []
        if not hit:
            return None
        p = hit[0]
    return p


def imports(blk):
    imp = blk.get("imports") or {}
    recs = imp.get("import_record", [])
    if isinstance(recs, dict):
        recs = [recs]
    return [r["file"] for r in recs if isinstance(r, dict) and r.get("file")]


def battle_areas(root, path, seen=None):
    seen = seen or set()
    if path in seen:
        return {}
    seen.add(path)
    blk = load(path)
    if not blk:
        return {}
    found = {}
    for f in imports(blk):
        sub = resolve(root, f)
        if sub:
            found.update(battle_areas(root, sub, seen))
    for name, a in (blk.get("areas") or {}).items():
        if isinstance(a, dict) and "tm" in a and a.get("type") in ("Box", "Cylinder", "Sphere"):
            found[name] = (a["type"], a["tm"])
    return found


def corners(kind, tm):
    ax, _, az, c = tm
    if kind != "Box":
        r = math.hypot(ax[0], ax[2])
        return [[round(c[0] + r * math.cos(k * math.pi / 12), 1), round(c[2] + r * math.sin(k * math.pi / 12), 1)]
                for k in range(24)]
    pts = []
    for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        pts.append([round(c[0] + 0.5 * (sx * ax[0] + sz * az[0]), 1),
                    round(c[2] + 0.5 * (sx * ax[2] + sz * az[2]), 1)])
    return pts


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    root = Path(sys.argv[1])
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else paths.BATTLE_AREAS
    if not root.is_dir():
        raise SystemExit(f"{root} is not a folder")
    out = {}
    for path in sorted(root.glob("*/*.blk")):
        blk = load(path)
        if not blk:
            continue
        mission = (blk.get("mission_settings") or {}).get("mission") or {}
        level = Path(mission.get("level", "")).stem
        if not level.startswith("avg_"):
            continue  # templates (levels/water.bin) and non-tank levels
        every = battle_areas(root, path)
        areas = {n: a for n, a in every.items() if "battle_area" in n.lower()}
        if not areas:
            continue
        pick = (next((n for n in areas if "hardcore" in n.lower()), None)
                or next((n for n in areas if "arcade" not in n.lower()), None) or next(iter(areas)))
        poly = corners(*areas[pick])
        rb = "arcade" not in pick.lower()

        def centre(a):
            return [round(a[1][3][0], 1), round(a[1][3][2], 1)]
        caps = {}
        for n, a in every.items():
            low = n.lower()
            if "capture_area" in low and not low.startswith("briefing") and a[0] != "Box":
                if ("arcade" in low) == rb and ("hardcore" in low or "arcade" in low):
                    continue  # other mode's copy
                caps[n] = centre(a) + [round(math.hypot(a[1][0][0], a[1][0][2]), 1)]
        spawns = {t: [centre(a) for n, a in every.items() if n.lower().startswith(t + "_tankspawn")]
                  for t in ("t1", "t2")}
        xs, zs = [p[0] for p in poly], [p[1] for p in poly]
        out[path.stem] = {"level": level, "area": pick, "polygon": poly,
                          "bbox": [min(xs), min(zs), max(xs), max(zs)],
                          "size_m": [round(max(xs) - min(xs)), round(max(zs) - min(zs))],
                          "captures": caps, "spawns": spawns}
    target.parent.mkdir(parents=True, exist_ok=True)
    with open(target, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print(f"{len(out)} missions -> {target}")


if __name__ == "__main__":
    main()
