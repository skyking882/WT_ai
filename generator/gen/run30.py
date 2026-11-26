"""Driver: steps 0-5 for one mission -> out/<mission>/analysis.json + pictures (render30.py).
usage: run30.py <mission> [--no-sens] [--no-pics]
"""
import csv
import json
import sys
import time
from collections import Counter

import numpy as np

import common30 as C
import corr30
import field30 as F
import sheet30
import sight30


def log(msg, t0=[time.time()]):
    print(f"[{time.time() - t0[0]:6.1f}s] {msg}", flush=True)


# ---------------------------------------------------------------- variants and tracks
def variants_report(mission):
    level = C.mission_level(mission)
    areas = json.loads(C.paths.BATTLE_AREAS.read_text(encoding="utf-8"))
    ka = json.loads(C.PROTECT.read_text(encoding="utf-8")) if C.PROTECT.is_file() else {}
    rep = {}
    for m, v in sorted(areas.items()):
        if v.get("level") != level or not m.endswith("_dom"):
            continue
        sp = {k: np.array(p) for k, p in v["spawns"].items()}
        rep[m] = dict(zones=C.zones(m), spawns={k: dict(n=len(p), centre=np.round(p.mean(0), 1).tolist()) for k, p in sp.items()},
                      kill_areas={a: dict(name=s["name"], shapes=[(sh["type"], np.round(np.array(sh["tm"])[3], 1).tolist())
                                                                  for sh in s["shapes"]]) for a, s in ka.get(m, {}).items()},
                      has_scan=(C.paths.OBSTACLES / f"{m}.bin").is_file())
    runs = C.runs_of_level(level)
    used = {}
    for r in runs:
        zr = C.run_zones(r)
        best = None
        for m, v in rep.items():
            zz = [(x, z) for _, x, z, _ in v["zones"]]
            d = max(min(np.hypot(x - a, z - b) for a, b in zz) for x, z in zr) if zr else 1e9
            if best is None or d < best[1]:
                best = (m, d)
        used[r] = dict(mission=best[0], zone_match_m=round(best[1], 1))
    return dict(missions=rep, runs=used)


def spawn_check(runs_used):
    """First logged position of every unit vs the mission's spawn points (mission data)."""
    out = {}
    for r, u in runs_used.items():
        sp = C.spawns(u["mission"])
        pts = np.concatenate([sp[1], sp[2]])
        first = {}
        with open(C.RUNS_DIR / r / "units.csv", newline="") as f:
            for row in csv.DictReader(f):
                if row["active"] != "1" or row["id"] in first:
                    continue
                first[row["id"]] = (float(row["x"]), float(row["z"]))
        d = [float(np.hypot(pts[:, 0] - x, pts[:, 1] - z).min()) for x, z in first.values()]
        out[r] = dict(units=len(d), median_m=round(float(np.median(d)), 1) if d else None,
                      within_15m=int(sum(1 for v in d if v <= 15)))
    return out


def tracks_check(Fl, runs):
    """Real positions (bots: ground_movement.csv, all units: units.csv) on the merged map. A hull centre with
    clearance < 1.0 m means the hull overlaps a hard cell by more than 0.8 m: the map is wrong there (or the
    position glitched). Clusters within 25 m are listed with the object names nearby."""
    pts, src = [], []
    for run in runs:
        d = C.RUNS_DIR / run
        for name in ("ground_movement.csv", "units.csv"):
            p = d / name
            if not p.is_file():
                continue
            with open(p, newline="") as f:
                for row in csv.DictReader(f):
                    if name == "units.csv" and row["active"] != "1":
                        continue
                    try:
                        x, z = float(row["x"]), float(row["z"])
                    except (KeyError, ValueError, TypeError):
                        continue
                    pts.append((x, z))
                    src.append(run)
    P = np.array(pts)
    src = np.array(src)
    r = ((P[:, 1] - Fl.Z0) // 0.5).astype(int)
    c = ((P[:, 0] - Fl.X0) // 0.5).astype(int)
    ok = (r >= 0) & (r < Fl.H5) & (c >= 0) & (c < Fl.W5)
    P, src, r, c = P[ok], src[ok], r[ok], c[ok]
    clr = Fl.clr05[r, c]
    cls = Fl.cls[np.clip(((P[:, 1] - Fl.Z0) // 0.25).astype(int), 0, Fl.H - 1), np.clip(((P[:, 0] - Fl.X0) // 0.25).astype(int), 0, Fl.W - 1)]
    bad = clr < 1.0
    rep = dict(samples=int(len(P)), runs=len(runs), below_1m_clearance=int(bad.sum()),
               below_2_3m_clearance=int((clr < C.REQ_CLR).sum()),
               on_hard_class={C.CLASS_NAMES[k]: int(v) for k, v in Counter(cls[np.isin(cls, C.HARD_CODES)].tolist()).items()})
    B = P[bad]
    lab = -np.ones(len(B), int)
    k = 0
    for i in range(len(B)):
        if lab[i] >= 0:
            continue
        lab[i] = k
        st = [i]
        while st:
            u = st.pop()
            dd = np.hypot(B[:, 0] - B[u, 0], B[:, 1] - B[u, 1])
            for v in np.nonzero((dd <= 25) & (lab < 0))[0]:
                lab[v] = k
                st.append(v)
        k += 1
    names = instance_index()
    cl = []
    for g in range(k):
        m = lab == g
        cx, cz = B[m].mean(0)
        bc = cls[bad][m]
        near = names(cx, cz, 10.0)
        cl.append(dict(x=round(float(cx), 1), z=round(float(cz), 1), samples=int(m.sum()), runs=len(set(src[bad][m])),
                       min_clearance=round(float(clr[bad][m].min()), 2),
                       classes={C.CLASS_NAMES[q]: int(v) for q, v in Counter(bc.tolist()).items()},
                       objects_within_10m=near))
    cl.sort(key=lambda d: -d["samples"])
    rep["clusters"] = cl
    return rep, P[bad]


_INST = None


def instance_index():
    global _INST
    if _INST is None:
        sys.path.insert(0, str(C.S18))
        import build_ri_obstacles as bro
        codec = bro.cr.Codecs(str(bro.CDK))
        _INST = bro.rigz_decode(codec, (bro.WT / "levels" / f"{MISSION_LEVEL}.bin").read_bytes())
        _INST = (np.array([(i[1], i[3]) for i in _INST]), [i[0] for i in _INST])
    xy, nm = _INST

    def near(x, z, r):
        d = np.hypot(xy[:, 0] - x, xy[:, 1] - z)
        return [n for n, _ in Counter(nm[i] for i in np.nonzero(d <= r)[0]).most_common(6)]
    return near


def rec_arrivals(arr, corrs, poses, zones, zf, areas):
    """Arrival by the recommended corridor: per pose, its fastest time to the zone + (recommended - fastest corridor
    time of its spawn area; both corridors start at the area's fastest pose). 1st / 5th tank per army and area."""
    delta = {}
    for c in corrs:
        if c["role"] in ("recommended", "both"):
            fast = min((d for d in corrs if d["area"] == c["area"] and d["zone"] == c["zone"]), key=lambda d: d["time"])
            delta[(c["area"], c["zone"])] = (c["time"] - fast["time"], c["id"])
    out = {}
    for (L, x, z, r) in zones:
        f = zf[L].ravel()
        row = {}
        for k in (1, 2):
            allt = []
            for a in areas:
                if a["army"] != k or (a["id"], L) not in delta:
                    continue
                dl, cid = delta[(a["id"], L)]
                ta = sorted(float(f[p["cell"]]) + dl for p in poses if p["area"] == a["id"] and p["cell"] is not None)
                ta = [t for t in ta if np.isfinite(t)]
                row[a["id"]] = dict(first=round(ta[0], 1) if ta else None, fifth=round(ta[4], 1) if len(ta) >= 5 else None,
                                    corridor=cid, extra_s=round(dl, 1))
                allt += ta
            allt.sort()
            row[k] = dict(first=round(allt[0], 1) if allt else None, fifth=round(allt[4], 1) if len(allt) >= 5 else None)
        out[L] = row
    return out


def bridge_check(Fl, od):
    """Per deck of the map report: is a cell a hull can stand on, reachable from the spawns, within 6 m of each end?"""
    rep = json.loads((od / "map_report.json").read_text(encoding="utf-8"))
    out = []
    mask = Fl.valid05 & Fl.reach
    for n, b in enumerate(rep.get("scan_rules", {}).get("bridge_decks", [])):
        e = []
        for xz in (b["end_a"], b["end_b"]):
            sn = Fl.snap(xz[0], xz[1], 6.0, mask)
            e.append(sn is not None)
        out.append(dict(n=n + 1, x=b["x"], z=b["z"], deck_length=b["deck_length"], deck_width=b["deck_width"],
                        lane_width=b["lane_width"], rule_connected=b["connected"],
                        ends_failed={k: v["fail"] for k, v in b["ends"].items() if v["fail"]},
                        closed_before_m2=b["lane_classes_before_m2"],
                        end_closures_m2={k: v["closed_by_m2"] for k, v in b["ends"].items()},
                        end_a_reached=e[0], end_b_reached=e[1]))
    return out


def ne_unreached(Fl, zones):
    """Drivable but unreached area north-east of the zones' bounding box ('NE quadrant')."""
    xs = Fl.X0 + (np.arange(Fl.W5) + 0.5) * 0.5
    zs = Fl.Z0 + (np.arange(Fl.H5) + 0.5) * 0.5
    zx, zz = max(z[1] for z in zones), max(z[2] for z in zones)
    m = (xs[None, :] > zx) & (zs[:, None] > zz)
    return int((Fl.valid05 & ~Fl.reach & m).sum()) // 4


def corr_table(corrs):
    return [dict(id=c["id"], area=c["area"], zone=c["zone"], role=c["role"], time=c["time"], length=c["length"],
                 expo_s=c["expo_s"], first_seen_t=c["first_seen_t"], narrowest=c["narrowest"]["width"],
                 entry=c["entry"]["side"]) for c in corrs]


# ---------------------------------------------------------------- one full analysis
def run_once(Fl, mission, P, areas, poses, zones, kill, speeds, corr_cache=None):
    out = {}
    v_road = P["v_road"] if P["v_road"] else P["v_road_fallback"]
    v_off = P["v_off"] if P["v_off"] else P["v_off_fallback"]
    out["speeds"] = dict(v_road=v_road, v_off=v_off)
    if corr_cache is None:
        cost = Fl.time_cost(v_road, v_off)
        fields, army, zf, zsrc = F.travel(Fl, cost, areas, poses, zones)
        log("  travel fields")
        res = F.corridors(Fl, cost, areas, poses, zones, zf, zsrc, max(v_road, v_off))
        corrs = corr30.describe(Fl, res, zones, areas)
        log(f"  corridors: {len(corrs)}")
        arr = F.arrivals(poses, zones, zf, areas)
        corr_cache = dict(fields=fields, army=army, zf=zf, corrs=corrs, arr=arr,
                          tries={f"{r['area']}>{r['zone']}": r.get("tries") for r in res})
    fields, army, zf, corrs, arr = (corr_cache[k] for k in ("fields", "army", "zf", "corrs", "arr"))
    S = sight30.Sight(Fl, P)
    zin = sight30.zone_points(S, zones)
    zin_exp, _ = S.accumulate([(zin[L], np.zeros(len(zin[L]), np.float32), np.full(len(zin[L]), 1.0 / len(zin[L]), np.float32))
                               for (L, *_r) in zones], P["target_h"], P["eye_h"])
    cexp = {a: sight30.exposure_phase(S, Fl, a, 3 - a, army, corrs, kill) for a in (1, 2)}
    sight30.recommend(corrs, {**cexp[1], **cexp[2]})
    arr_rec = rec_arrivals(arr, corrs, poses, zones, zf, areas)
    log("  corridor exposure and recommended corridors")
    res = {}
    for a in (1, 2):
        res[a] = sight30.analyse_army(S, Fl, a, 3 - a, army[a], army[3 - a], corrs, zones, kill, zin, zin_exp, areas, fields,
                                      cexp[a])
        log(f"  sight army {a}: {len(res[a]['interdiction_areas'])} interdiction areas")
    sh = sheet30.sheet(arr, corrs, res, zones, areas, arr_rec)
    sent = sheet30.sentences(arr_rec, corrs, res, sh, zones, areas, P["setup_s"])
    concl = sheet30.conclusions(sh, None, res, zones, areas, arr_rec, corrs)
    nums = sheet30.numbers(sh, res, zones)
    rep =json.loads((Fl.od / "map_report.json").read_text(encoding="utf-8"))
    return dict(S=S, zin=zin, zin_exp=zin_exp, res=res, sheet=sh, sentences=sent, conclusions=concl, numbers=nums,
                arr_rec=arr_rec, corr_table=corr_table(corrs),
                cache=corr_cache, speeds=out["speeds"], P=P, bridges=rep.get("scan_rules", {}).get("bridge_decks", []))


def strip(o):
    """JSON-ready copy: drop big arrays."""
    if isinstance(o, dict):
        return {str(k): strip(v) for k, v in o.items() if k not in ("interdiction_cells", "fire_cells", "own_t2", "enemy_t2",
                                                                   "cells", "observed", "seen")}
    if isinstance(o, list):
        return [strip(v) for v in o]
    if isinstance(o, np.ndarray):
        return np.round(o, 2).tolist() if o.dtype.kind == "f" else o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    return o


MISSION_LEVEL = None


def main():
    global MISSION_LEVEL
    mission = sys.argv[1]
    sens = "--no-sens" not in sys.argv
    pics = "--no-pics" not in sys.argv
    t_all = time.time()
    MISSION_LEVEL = C.mission_level(mission)
    od = C.out_dir(mission)
    Fl = F.Field(mission)
    log(f"field {Fl.W5}x{Fl.H5} (0.5 m), road mesh {'yes' if Fl.has_road else 'no'}, {Fl.t_build:.0f}s")
    zones = C.zones(mission)
    kill = C.kill_areas(mission)
    areas, poses = F.spawn_areas(mission, Fl)
    Fl.set_reach([p["cell"] for p in poses if p["cell"] is not None])
    runs = C.runs_of_level(MISSION_LEVEL)
    speeds = F.log_speeds(Fl, runs)
    log(f"speeds from {len(runs)} logged battles: road {speeds['v_road']} off {speeds['v_off']}")
    P = dict(C.BASE)
    P["v_road"], P["v_off"] = speeds["v_road"], speeds["v_off"]
    if not Fl.has_road and P["v_off"] is None and speeds.get("v_road") is None:
        pass
    base = run_once(Fl, mission, P, areas, poses, zones, kill, speeds)
    log("base analysis done")
    var = variants_report(mission)
    sc = spawn_check(var["runs"])
    tr, bad_pts = tracks_check(Fl, runs) if runs else (None, np.zeros((0, 2)))
    log("variants / tracks checked")
    sens_out = {}
    if sens:
        tests = [("range_m", 600.0), ("range_m", 1500.0), ("fol_depth", 10.0), ("fol_depth", 50.0),
                 ("setup_s", 0.0), ("setup_s", 20.0), ("house_scale", 0.6), ("house_add", 3.0),
                 ("v_off", 0.8), ("v_off", 1.2)]
        for key, val in tests:
            Q = dict(P)
            if key == "v_off":
                Q["v_off"] = round((P["v_off"] or P["v_off_fallback"]) * val, 2)
                label = f"v_off x{val}"
                r = run_once(Fl, mission, Q, areas, poses, zones, kill, speeds)
            else:
                Q[key] = val
                label = f"{key}={val}"
                r = run_once(Fl, mission, Q, areas, poses, zones, kill, speeds, corr_cache=base["cache"])
            ch = {k: (base["conclusions"][k], r["conclusions"].get(k)) for k in base["conclusions"]
                  if base["conclusions"][k] != r["conclusions"].get(k)}
            moved = {}
            for a in (1, 2):
                bp = base["res"][a]["interdiction"][:3]
                vp = r["res"][a]["interdiction"]
                moved[a] = [dict(n=p["n"], kept=any(np.hypot(p["x"] - q["x"], p["z"] - q["z"]) <= C.BASE["pos_sep"] for q in vp))
                            for p in bp]
            nd = {k: (base["numbers"][k], r["numbers"].get(k)) for k in base["numbers"]
                  if base["numbers"][k] != r["numbers"].get(k)}
            sens_out[label] = dict(changed=ch, unchanged=len(base["conclusions"]) - len(ch), top3_positions_kept=moved,
                                   numbers_changed=nd, speeds=r["speeds"],
                                   sentences=r["sentences"])
            log(f"  sensitivity {label}: {len(ch)} conclusions changed")
            del r
    c = base["cache"]
    analysis = dict(
        mission=mission, level=MISSION_LEVEL, generated=time.strftime("%Y-%m-%d %H:%M"),
        grids=dict(class_map="0.25 m", search="0.5 m (travel times, corridors, clearance)",
                   sight="2 m (heights, sight lines, positions, fire support, exposure)"),
        assumptions=P, map_report=json.loads((od / "map_report.json").read_text(encoding="utf-8")),
        variants=var, spawn_check_against_logs=sc, speeds_from_logs=speeds, tracks_check=tr,
        zones=[dict(id=L, x=x, z=z, r=r) for L, x, z, r in zones], kill_areas={a: [dict(name=s["name"], kind=s["kind"],
                                                                                         centre=s["c"]) for s in v] for a, v in kill.items()},
        bridge_check=bridge_check(Fl, od), reach=dict(
            drivable_m2=int(Fl.valid05.sum()) // 4, reached_m2=int((Fl.valid05 & Fl.reach).sum()) // 4,
            unreached_drivable_m2=int((Fl.valid05 & ~Fl.reach).sum()) // 4,
            unreached_NE_quadrant_m2=ne_unreached(Fl, zones)),
        spawn_areas=areas, poses=poses, arrivals=c["arr"], arrivals_recommended=base["arr_rec"],
        corridor_table=base["corr_table"], corridors=c["corrs"], corridor_search_notes=c["tries"],
        results={a: base["res"][a] for a in (1, 2)}, sheet=base["sheet"], sentences=base["sentences"],
        conclusions=base["conclusions"], sensitivity=sens_out)
    if "--pics-only" not in sys.argv:
        C.save_json(od / "analysis.json", strip(analysis))
        log("analysis.json written")
    if pics:
        import render30
        render30.render_all(Fl, base, zones, areas, poses, kill, c, od, mission, bad_pts)
        log("pictures written")
    print(f"TOTAL {time.time() - t_all:.0f} s")


if __name__ == "__main__":
    main()
