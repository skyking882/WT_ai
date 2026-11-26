"""The plan step alone: lanes37's analysis part (field, spawn areas, measured speeds, run30.run_once, flank variants)
and then plan35.plans exactly as lanes37 calls it; no lanes, routes or network. Writes out/<mission>/plans_only.json.
Optionally compares with the plans of an existing <mission>.plan.json.
usage: python tools/plans_only.py <mission> [--workers N] [--compare <plan.json>]"""
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]


def main():
    m = sys.argv[1]
    workers = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else min(13, os.cpu_count() or 1)
    os.environ["OMP_NUM_THREADS"] = str(workers)          # as run_map.py
    import run_map
    import common30 as C
    import field30 as F
    import pack31 as PK
    import plan35
    import run30
    t0 = time.time()
    run_map.stage_base(m)
    Fl = F.Field(m)
    zones = C.zones(m)
    kill = C.kill_areas(m)
    areas, poses30 = F.spawn_areas(m, Fl)
    Fl.set_reach([p["cell"] for p in poses30 if p["cell"] is not None])
    run30.MISSION_LEVEL = C.mission_level(m)
    runs = C.runs_of_level(run30.MISSION_LEVEL)
    speeds = F.log_speeds(Fl, runs)
    P = dict(C.BASE)
    P["v_road"], P["v_off"] = speeds["v_road"], speeds["v_off"]
    base = run30.run_once(Fl, m, P, areas, poses30, zones, kill, speeds)
    corrs = base["cache"]["corrs"]
    for c in corrs:
        zx, zz = next((z[1], z[2]) for z in zones if z[0] == c["zone"])
        c["entry_deg"] = math.degrees(math.atan2(c["entry"]["z"] - zz, c["entry"]["x"] - zx))
    PK.flank_variants(corrs)
    plans = plan35.plans(base, areas, zones, corrs)
    out = dict(mission=m, zones=[z[0] for z in zones], runs=len(runs), v_road=speeds["v_road"], v_off=speeds["v_off"],
               seconds=round(time.time() - t0, 1), plans=plans)
    (C.out_dir(m) / "plans_only.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"{m}: zones {out['zones']}, {len(runs)} runs (v_road {speeds['v_road']}, v_off {speeds['v_off']}), {out['seconds']} s")
    for a, pls in plans.items():
        for pl in pls:
            print(f"  {pl['id']:<16} {pl['kind']:<12} w{pl['weight']} " +
                  " ".join(f"{L}:{v['role']}:{v['share']}" for L, v in pl["zones"].items()))
    if "--compare" in sys.argv:
        ref = json.loads(Path(sys.argv[sys.argv.index("--compare") + 1]).read_text(encoding="utf-8"))["plans"]
        print("plans identical to", sys.argv[sys.argv.index("--compare") + 1], ":", ref == plans)
        if ref != plans:
            for a in ref:
                for p_, q_ in zip(ref[a], plans.get(a, [])):
                    if p_ != q_:
                        print("  differs:", p_["id"], {k: (p_.get(k), q_.get(k)) for k in p_ if p_.get(k) != q_.get(k)})


if __name__ == "__main__":
    main()
