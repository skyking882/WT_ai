"""Whole chain for one mission: lanes (gen/lanes37.py), then the road network (gen/network38.py).

usage: python run_map.py <mission> [--workers N] [--check]

  --check  dry run: inputs complete? zones and spawn areas readable? what is unusual? Stops before the heavy work
           (loads the base map once to snap the spawn poses; ~10-20 s). Writes out/<mission>/check.json.

Outputs in out/<mission>/: the four pack files as the game wants them -- <mission>.plan.json, <mission>.routes.bin,
<mission>.map.bin, <mission>.network.bin -- plus read_check.txt, lanes_summary.json, network_report.txt,
net_queries.txt, pictures, logs (lanes.log, network.log) and run_report.json (status, phase times, lane counts,
limits, read_check, network checks, pack sizes). Exit code 0 = ok, 1 = failed (the reason is in run_report.json).
"""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
import paths  # noqa: E402

NET_KEYS = ("nodes ", "loader checks:", "routes ", "heading steps > 12 deg", "spacing steps outside", "time ratio",
            "undirected:", "turn rule, from", "target end nodes not reachable", "paths found", "paths touching:",
            "pivots:", "heading steps > 12 deg outside", "every edge alone", "run time")


def _env():
    return dict(os.environ, PYTHONPATH=os.pathsep.join([str(paths.GEN), str(ROOT)]), PYTHONIOENCODING="utf-8")


def inputs(mission):
    """-> (info, problems). Every file the generator reads for this mission."""
    probs, info = [], {}
    areas = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))
    if mission not in areas:
        return info, [f"{mission} is not in {paths.BATTLE_AREAS.relative_to(ROOT)}"]
    level = areas[mission]["level"]
    info["level"] = level
    for f in paths.BASE_FILES:
        if not (paths.MAPS / mission / f).is_file():
            probs.append(f"missing data/maps/{mission}/{f} (run prep/prep_base.py {mission} on Windows with the game)")
    import common30 as C
    info["runs_of_level"] = C.runs_of_level(level)
    if not info["runs_of_level"]:
        info["speed_note"] = "no logged battle of this level: speeds fall back to 9.0 m/s road, 7.0 m/s off road"
    if mission == "sweden_dom" and not paths.SWEDEN_ROUTES.is_file():
        info["poses_note"] = ("no data/sweden/routes.json: spawn poses are the mission's spawn points with unknown "
                              "heading (as for every other mission)")
    prot = json.loads(paths.SPAWN_PROTECTION.read_text(encoding="utf-8")) if paths.SPAWN_PROTECTION.is_file() else {}
    info["spawn_protection"] = mission in prot
    info["closed_list"] = (paths.CLOSED / f"{mission}.closed.json").is_file()
    return info, probs


def stage_base(mission):
    od = paths.mission_out(mission)
    for f in paths.BASE_FILES:
        shutil.copy2(paths.MAPS / mission / f, od / f)
    return od


def check(mission):
    info, probs = inputs(mission)
    out = dict(mission=mission, inputs=info, missing=probs, unusual=[])
    if probs:
        return out
    import common30 as C
    import field30 as F
    stage_base(mission)
    caps = sorted(C.battle_area(mission)["captures"])
    sets = {}
    for c in caps:
        sets.setdefault(C.capture_set(c) or "(none)", []).append(c)
    out["capture_sets"] = sets
    try:
        zones = C.zones(mission)
    except ValueError as e:
        out["missing"].append(f"zones: {e}")
        return out
    out["zones"] = [dict(id=L, x=round(x, 1), z=round(z, 1), r=r) for L, x, z, r in zones]
    if len(sets) > 1:
        out["unusual"].append(f"capture areas listed once per set {sorted(sets)}: the '{C.CAPTURE_MODE}' set is used")
    n = len(zones)
    if n != 3:
        out["unusual"].append(f"{n} zone(s), not 3: plans " + ("single (the zone main, share 1.0)" if n == 1 else
                              "balanced + main_effort when a zone leads by the rule (no two_point)" if n == 2 else
                              "balanced, two_point (every zone after the two mains delayed or skipped), main_effort"))
    Fl = F.Field(mission)
    areas, poses = F.spawn_areas(mission, Fl)
    out["spawn_areas"] = {str(a): [dict(id=x["id"], poses=len(x["poses"]), centre=x["centre"]) for x in areas if x["army"] == a]
                          for a in (1, 2)}
    nos = [p["id"] for p in poses if p["cell"] is None]
    if nos:
        out["unusual"].append(f"{len(nos)} spawn pose(s) without a hull cell within 20 m: {nos}")
    for a in (1, 2):
        if not any(x["army"] == a for x in areas):
            out["unusual"].append(f"army {a} has no spawn point")
    rep = json.loads((paths.MAPS / mission / "map_report.json").read_text(encoding="utf-8"))
    out["map"] = dict(road=rep.get("road"), water=rep.get("water_note"),
                      bridges=len(rep.get("scan_rules", {}).get("bridge_decks", [])),
                      area=rep.get("area"))
    if rep.get("obstacle_scan"):
        out["unusual"].append(rep["obstacle_scan"])
    if info.get("poses_note"):
        out["unusual"].append(info["poses_note"])
    if "no Lnav" in str(rep.get("road")):
        out["unusual"].append("no road mesh (Lnav) for the level: every cell is off road (one speed)")
    if not info["runs_of_level"]:
        out["unusual"].append(info["speed_note"])
    if not info["spawn_protection"]:
        out["unusual"].append("no spawn-protection (kill area) entry")
    return out


def _tail(p, n=6):
    try:
        return p.read_text(encoding="utf-8", errors="replace").strip().splitlines()[-n:]
    except OSError:
        return []


def run(mission, workers):
    t_all = time.time()
    od = paths.mission_out(mission)
    rep = dict(mission=mission, status="failed", reason=None, phases={}, workers=workers, system=sys.platform,
               omp_num_threads=os.environ.get("OMP_NUM_THREADS"),
               runs_excluded=sorted(paths.RUNS_EXCLUDE))
    info, probs = inputs(mission)
    rep["inputs"] = info
    if info.get("poses_note"):
        rep["notes"] = [info["poses_note"]]
    if probs:
        rep["reason"] = "; ".join(probs)
        return rep
    t = time.time()
    stage_base(mission)
    rep["phases"]["stage_base_s"] = round(time.time() - t, 1)
    steps = [("lanes", [sys.executable, str(paths.GEN / "lanes37.py"), mission, "--workers", str(workers)]),
             ("network", [sys.executable, str(paths.GEN / "network38.py"), mission, "--lib", str(od), "--out", str(od)])]
    for name, cmd in steps:
        t = time.time()
        with open(od / f"{name}.log", "w", encoding="utf-8") as lf:
            r = subprocess.run(cmd, cwd=str(paths.GEN), env=_env(), stdout=lf, stderr=subprocess.STDOUT)
        rep["phases"][f"{name}_s"] = round(time.time() - t, 1)
        if r.returncode:
            rep["reason"] = f"{name} failed (exit {r.returncode}): " + " | ".join(_tail(od / f"{name}.log"))
            rep["total_s"] = round(time.time() - t_all, 1)
            return rep
    # the pack files under the names the game reads
    for src, dst in (("routes.bin", f"{mission}.routes.bin"), ("map.bin", f"{mission}.map.bin")):
        os.replace(od / src, od / dst)
    summ = json.loads((od / "lanes_summary.json").read_text(encoding="utf-8"))
    rep["lanes"] = summ.get("lanes")
    rep["lanes_by_side"] = summ.get("lanes_by_side")
    rep["limits"] = summ.get("limits")
    rep["read_check"] = summ.get("read_check")
    rep["dropped_lanes"] = summ.get("dropped_lanes")
    rep["lanes_timing"] = summ.get("timing")
    net = (od / "network_report.txt").read_text(encoding="utf-8").splitlines()
    rep["network"] = [l for l in net if l.startswith(NET_KEYS)]
    rep["pack"] = {f: (od / f).stat().st_size for f in (f"{mission}.plan.json", f"{mission}.routes.bin", f"{mission}.map.bin",
                                                        f"{mission}.network.bin") if (od / f).is_file()}
    ok = len(rep["pack"]) == 4 and (rep["limits"] or {}).get("within_limits") \
        and any("loader checks:" in l and "False" not in l for l in rep["network"])
    rep["status"] = "ok" if ok else "failed"
    if not ok:
        rep["reason"] = "pack incomplete, limits exceeded or a loader check failed"
    rep["total_s"] = round(time.time() - t_all, 1)
    return rep


def main():
    if len(sys.argv) < 2 or sys.argv[1].startswith("-"):
        raise SystemExit(__doc__)
    mission = sys.argv[1]
    workers = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else min(13, os.cpu_count() or 1)
    # the C helpers' OpenMP threads per process = the worker count, set here before any helper loads (the lanes /
    # network processes inherit it), so a run does not depend on the caller's environment
    os.environ["OMP_NUM_THREADS"] = str(workers)
    od = paths.mission_out(mission)
    if "--check" in sys.argv:
        out = check(mission)
        (od / "check.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
        print(json.dumps(out, indent=1))
        sys.exit(1 if out["missing"] else 0)
    rep = run(mission, workers)
    (od / "run_report.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: rep.get(k) for k in ("mission", "status", "reason", "phases", "total_s", "lanes", "limits", "pack")},
                     indent=1, default=str))
    sys.exit(0 if rep["status"] == "ok" else 1)


if __name__ == "__main__":
    main()
