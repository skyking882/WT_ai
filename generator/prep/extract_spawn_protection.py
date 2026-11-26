"""Spawn protection areas (team kill areas) of the Ground RB missions -> data/common/spawn_protection.json.
Optional input of the generator; needs the third-party vromfs / blk Python packages (prep/blk_archive.py). Reads the
game's mis.vromfs.bin, read only.

usage: python prep/extract_spawn_protection.py <mission> [<mission> ...] [--mis ARCHIVE] [--out FILE]
  defaults: ARCHIVE = <game>/mis.vromfs.bin (env WT_GAME_DIR), FILE = data/common/spawn_protection.json
  (paths.SPAWN_PROTECTION). FILE is rewritten with the missions given.

The mission logic (gamedata/missions/templates/tank_templates/team_killareas_template.blk) turns area "t1_killarea"
into a kill + detection area for army 2, and "t2_killarea" for army 1. On difficulty "realistic" (Ground RB) it uses
"<name>_realistic" when that area exists, else the plain name. So the spawn protection area of army N is
t<N>_killarea[_realistic]; the enemy of army N must never be placed inside it. A name may also be an area squad
(its members' shapes are used).
The mission file is gamedata/missions/cta/tanks/<location>/<mission>.blk, found in the archive by its name; areas
come from it and (recursively, 3 levels) from the files it imports.

Output json: {mission: {"1": {"name", "shapes": [{"name", "type", "tm", "file"}]}, "2": {...}}};
tm = 4 rows (x axis, y axis, z axis, position).
"""
import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
import paths  # noqa: E402

AREA = re.compile(r'"(\w+)"\s*\{\s*"type":t\s*=\s*"(\w+)"\s*"tm":m\s*=\s*\[(.*?)\]\]', re.S)
SQUAD = re.compile(r'"area_squad"\s*\{\s*"name":t\s*=\s*"(\w+)"\s*"tm":m\s*=\s*\[.*?\]\]\s*"props"\s*\{(.*?)\}', re.S)
MEMBER = re.compile(r'"squad_members":t\s*=\s*"(\w+)"')
IMPORT = re.compile(r'"file":t\s*=\s*"([^"]+\.blk)"', re.I)
MISSION_DIR = "gamedata/missions/cta/tanks/"


def areas_of(arc, path, seen, areas, squads, depth=0):
    """Every area and area squad defined in path and (recursively, 3 levels) in the files it imports."""
    if path.lower() in seen or depth > 3:
        return
    seen.add(path.lower())
    t = arc.text(path)
    if t is None:
        return
    for name, typ, body in AREA.findall(t):
        rows = [[float(v) for v in r.split(",")] for r in re.findall(r"\[([^\[\]]+)", body + "]")]
        areas.setdefault(name, {"name": name, "type": typ, "tm": rows, "file": path})
    for name, props in SQUAD.findall(t):
        squads.setdefault(name, MEMBER.findall(props))
    for imp in IMPORT.findall(t):
        areas_of(arc, imp, seen, areas, squads, depth + 1)


def resolve(name, areas, squads):
    """-> list of area shapes for an area or an area squad name."""
    if name in areas:
        return [areas[name]]
    if name in squads:
        return [areas[m] for m in squads[name] if m in areas]
    return []


def mission_file(arc, mission):
    want = "/" + mission.lower() + ".blk"
    hits = [p for p in arc.paths() if p.startswith(MISSION_DIR) and p.endswith(want) and p.count("/") == 5]
    if len(hits) != 1:
        raise SystemExit(f"{mission}: {len(hits)} mission files {hits} in the archive (expected exactly one)")
    return hits[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("missions", nargs="+")
    ap.add_argument("--mis", type=Path, help="archive (default <game>/mis.vromfs.bin)")
    ap.add_argument("--out", type=Path, default=paths.SPAWN_PROTECTION)
    a = ap.parse_args()
    archive_path = a.mis or paths.game_dir() / "mis.vromfs.bin"
    import blk_archive
    arc = blk_archive.Archive(archive_path)
    result = {}
    for m in a.missions:
        areas, squads = {}, {}
        areas_of(arc, mission_file(arc, m), set(), areas, squads)
        entry = {}
        for army in ("1", "2"):
            for name in (f"t{army}_killarea_realistic", f"t{army}_killarea"):
                shapes = resolve(name, areas, squads)
                if shapes:
                    entry[army] = {"name": name, "shapes": shapes}
                    break
        result[m] = entry
        print(m, {k: (v["name"], [(s["name"], s["type"], [round(x) for x in s["tm"][3]]) for s in v["shapes"]])
                  for k, v in entry.items()})
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(result, indent=1), encoding="utf-8")
    print(f"{len(result)} missions -> {a.out}")


if __name__ == "__main__":
    main()
