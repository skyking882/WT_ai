"""Route library (routes.bin WTRTE004, map.bin WTMAP004; binary_v4.py layout) and plan pack
(<mission>.plan.json WTPLAN001) for one mission, from run30's analysis. lanes37.py uses its functions; its own
main() stops with a message after the analysis step (use run_map.py).
"""
import json
import math
import multiprocessing as mp
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

import common30 as C
import field30 as F
import plan31
import route31 as R
import run30

WORKERS = 13
BETWEEN_CHUNK = 8                   # ends per zone <-> target job (each job repeats the zone's search: 1-2 s)
INNER_MARGIN = 12.0                 # zone routes end inside the zone radius - 12 m
JOIN0, JOIN1 = 40.0, 100.0          # lead-ins join a trunk between 40 and 100 m from its start
LEAD_MAXD_S = 90.0                  # lead-in search bound (s of driving)
FLANK_ANGLE = 45.0                  # a flank variant enters the zone at least 45 deg away from the recommended one
FLANK_TIME = 2.0
FRONT_TIME, FRONT_EXPO_S = 1.3, 10.0   # front corridors: within 1.3 x the fastest, at most 10 s more observed
CROSS_LEN = 150.0                       # designation: crossings counted in the first 150 m
POST_STAND_R, POST_STAND_CLR = 10.0, 4.3
SLOT_N, SLOT_SEP, SLOT_REACH, SLOT_CLR = 15, 10.0, 60.0, 3.0   # zone slots
ZZ_TIME, ZZ_OVERLAP = 1.5, 0.5     # zone <-> zone alternative (time limit, overlap limit)
CLOSE_R, CLOSE_STEP, CLOSE_END = 25.0, 40.0, 60.0
import build_core  # noqa: E402
READ_CHECK = build_core.read_check_path()   # built from read_check.c per platform (build/read_check[.exe])


def log(msg, t0=[time.time()]):
    print(f"[{time.time() - t0[0]:6.1f}s] {msg}", flush=True)


def ang(dx, dz):
    return math.degrees(math.atan2(dz, dx))


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


# ---------------------------------------------------------------- worker side
_W = {}


def _init(cache):
    cache = Path(cache)
    _W["G"] = R.Geo(cache)
    _W["cost"] = np.load(cache / "cost.npy", mmap_mode="r")
    _W["unit"] = np.load(cache / "unit.npy", mmap_mode="r")
    meta = np.load(cache / "meta.npy")
    _W["X0"], _W["Z0"] = float(meta[0]), float(meta[1])
    _W["W5"], _W["H5"] = _W["cost"].shape[1], _W["cost"].shape[0]
    _W["lib"] = C.dll()


def _search(cost, src, maxd=0.0, goal=-1):
    lib = _W["lib"]
    c = np.ascontiguousarray(cost)
    dist = np.empty(c.shape, np.float32)
    par = np.empty(c.shape, np.int32)
    s = np.array(src, np.int32)
    lib.dijkstra(c, c.shape[0], c.shape[1], s, len(s), int(goal), float(maxd), 0.0, dist, par)
    return dist, par


def _chain(par, start):
    out = []
    i = int(start)
    pf = par.ravel()
    while i >= 0:
        out.append(i)
        i = int(pf[i])
    return np.array(out, np.int64)


def _xy(cells):
    W5 = _W["W5"]
    return np.stack([_W["X0"] + (cells % W5 + 0.5) * 0.5, _W["Z0"] + (cells // W5 + 0.5) * 0.5], 1)


def _cell(x, z):
    return int((z - _W["Z0"]) // 0.5) * _W["W5"] + int((x - _W["X0"]) // 0.5)


def _pose_start(p):
    return [p["x"], p["z"]]


def _rec(r, **kw):
    r = dict(r)
    r.update(kw)
    return r


def leadins(G, trunk, poses, label, trunk_rec_base):
    """Routes for every pose: lead-in (shortest by time to the trunk samples 40-100 m from its start) + trunk."""
    P = trunk["P"]
    s = trunk["s"]
    win = np.nonzero((s >= JOIN0) & (s <= JOIN1))[0]
    if not len(win):
        win = np.arange(len(P) // 3, max(len(P) // 3 + 1, len(P) * 2 // 3))
    cells = np.array([_cell(*P[i]) for i in win], np.int64)
    dist, par = _search(_W["cost"], np.unique(cells), maxd=LEAD_MAXD_S)
    out, notes = [], []
    for p in poses:
        pc = p["cell"]
        if pc is None or dist.ravel()[pc] >= 1e19:
            notes.append(f"{label}: pose {p['id']} not within {LEAD_MAXD_S:.0f} s of the trunk")
            continue
        ch = _chain(par, pc)
        j = win[int(np.argmin(np.hypot(P[win, 0] - _xy(ch[-1:])[0, 0], P[win, 1] - _xy(ch[-1:])[0, 1])))]
        raw = np.vstack([[p["x"], p["z"]], _xy(ch)])
        rec = R.finish(G, raw, pose=p, tail=P[j:])
        if rec is None:
            notes.append(f"{label}: pose {p['id']} rejected (touching samples away from the ends)")
            continue
        out.append(_rec(trunk_rec_base, pose=p["id"], **{k: rec[k] for k in rec}))
    return out, notes


def job_zone(job):
    """One (spawn area, zone, variant): trunk along the analysis corridor (+ into the inner disc), lead-ins of the poses."""
    G = _W["G"]
    t0 = time.time()
    cells = np.asarray(job["cells"], np.int64)
    raw = _xy(cells)
    # into the inner disc: unit-cost search from the corridor end to the nearest inner cell
    if job["inner"]:
        d, par = _search(_W["unit"], [int(cells[-1])], maxd=200.0)
        inner = np.asarray(job["inner"], np.int64)
        k = inner[np.argmin(d.ravel()[inner])]
        if d.ravel()[k] < 1e19:
            tail = _chain(par, k)[::-1]
            raw = np.vstack([raw, _xy(tail[1:])])
    trunk = R.finish(G, raw, pose=None)
    if trunk is None:
        return dict(kind="zone", key=job["key"], routes=[], notes=[f"{job['key']}: trunk rejected"], seconds=time.time() - t0)
    base = dict(route_kind="spawn_zone", target=job["target"], variant=job["variant"], corridor=job["corridor"])
    out, notes = leadins(G, trunk, job["poses"], job["key"], base)
    return dict(kind="zone", key=job["key"], routes=out, notes=notes, seconds=time.time() - t0,
                trunk=dict(P=trunk["P"]))


def job_target(job):
    """One post or plan slot: Dijkstra from its standing cell; per spawn area a trunk from the area's best pose,
    lead-ins for the other poses of that area."""
    G = _W["G"]
    t0 = time.time()
    dist, par = _search(_W["cost"], [job["cell"]])
    out, notes = [], []
    for area, poses in job["areas"].items():
        pc = [p for p in poses if p["cell"] is not None and dist.ravel()[p["cell"]] < 1e19]
        if not pc:
            notes.append(f"{job['target']}: no pose of {area} reaches it")
            continue
        bp = min(pc, key=lambda p: dist.ravel()[p["cell"]])
        raw = np.vstack([[bp["x"], bp["z"]], _xy(_chain(par, bp["cell"]))])
        trunk = R.finish(G, raw, pose=bp)
        if trunk is None:
            notes.append(f"{job['target']}: trunk from {bp['id']} rejected")
            continue
        base = dict(route_kind="spawn_post", target=job["target"], variant=0)
        out.append(_rec(base, pose=bp["id"], **trunk))
        tcells = {_cell(*q) for q in trunk["P"][::2]}
        for p in pc:
            if p is bp:
                continue
            ch = _chain(par, p["cell"])
            m = next((i for i, c in enumerate(ch) if int(c) in tcells), None)
            if m is None:
                rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(ch)]), pose=p)
            else:
                cx, cz = _xy(ch[m:m + 1])[0]
                j = int(np.argmin(np.hypot(trunk["P"][:, 0] - cx, trunk["P"][:, 1] - cz)))
                if j >= len(trunk["P"]) - 3:
                    rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(ch)]), pose=p)
                else:
                    rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(ch[:m + 1])]), pose=p, tail=trunk["P"][j:])
            if rec is None:
                notes.append(f"{job['target']}: pose {p['id']} rejected")
                continue
            out.append(_rec(base, pose=p["id"], **rec))
    return dict(kind="target", key=job["target"], routes=out, notes=notes, seconds=time.time() - t0)


def _exact_suffix(A, T):
    """Number of trailing samples of A equal to the trailing samples of T."""
    n = min(len(A), len(T))
    eq = (A[::-1][:n] == T[::-1][:n]).all(axis=1)
    return n if eq.all() else int(np.argmin(eq))


def _exact_prefix(A, T):
    n = min(len(A), len(T))
    eq = (A[:n] == T[:n]).all(axis=1)
    return n if eq.all() else int(np.argmin(eq))


def leadout(G, T, end, keep=None):
    """Route from T's start to another end near T's end: shortest lead-out (by time, within 90 s) from T's samples
    10-150 m before its end to the end; smoothed in reverse with T as the frozen tail. Returns the finished record
    (P from T's start to the end) or None."""
    P = T["P"]
    s = T["s"]
    L = s[-1]
    win = np.nonzero((s >= max(L - 150.0, 25.0)) & (s <= L - 10.0))[0]
    dist, par = _search(_W["cost"], [end["cell"]])
    if keep is not None:
        keep["dist"], keep["par"] = dist, par
    if not len(win):
        return None
    wc = np.array([_cell(*P[i]) for i in win], np.int64)
    dv = dist.ravel()[wc]
    if not np.isfinite(dv).any() or dv.min() >= 1e19:
        return None
    j = int(win[int(np.argmin(dv))])
    ch = _chain(par, int(wc[int(np.argmin(dv))]))            # trunk cell -> end
    raw_rev = np.vstack([[end["x"], end["z"]], _xy(ch[::-1])])
    rec = R.finish(G, raw_rev, tail=P[j::-1].copy(), trim_end=False)
    if rec is None:
        return None
    O = rec["P"][::-1].copy()
    if R.bad_samples(G, O)[0].any():
        return None
    pr = R.curve_props(G, O)
    return dict(P=O, s=pr["s"], kappa=pr["kappa"], clr=pr["clr"], time_s=pr["t"], length=float(pr["s"][-1]),
                pinned=False, cut_start_m=0.0, cut_end_m=rec["cut_start_m"], pivot=0.0, gear="none", fwd_free=False,
                rev_free=False, no_start_turn_data=True)


def splice(G, Rp, T, O):
    """Rp: a pose's route ending with T's exact samples; O: T's exact head + a lead-out. Join them on a trunk sample
    both share exactly. Returns the record or None."""
    nf = _exact_suffix(Rp["P"], T["P"])
    f = len(T["P"]) - nf
    e = _exact_prefix(O["P"], T["P"]) - 1
    if nf < 3 or e < f + 2:
        return None
    m = (f + e) // 2
    P = np.vstack([Rp["P"][: len(Rp["P"]) - nf + (m - f)], O["P"][m:]])
    if R.bad_samples(G, P, max(0, len(Rp["P"]) - nf + (m - f) - 3), len(Rp["P"]) - nf + (m - f) + 3)[0].any():
        return None
    pr = R.curve_props(G, P)
    rec = dict(Rp)
    rec.update(P=P, s=pr["s"], kappa=pr["kappa"], clr=pr["clr"], time_s=pr["t"], length=float(pr["s"][-1]),
               cut_end_m=O.get("cut_end_m", 0.0))
    return rec


def job_area(job):
    """One plan area (its slots): pose routes for the poses of its first spawn area (Dijkstra from slot 1, trunk from
    the area's best pose, lead-ins; slots 2-4 by lead-outs off the trunk end, spliced onto every pose's route)."""
    G = _W["G"]
    t0 = time.time()
    ends = job["ends"]
    s1 = ends[0]
    dist, par = _search(_W["cost"], [s1["cell"]])
    out, notes = [], []
    pc = [p for p in job["poses"] if p["cell"] is not None and dist.ravel()[p["cell"]] < 1e19]
    if not pc:
        return dict(kind="area", key=job["area"], routes=[], notes=[f"{job['area']}: no pose reaches it"], seconds=time.time() - t0)
    bp = min(pc, key=lambda p: dist.ravel()[p["cell"]])
    T = R.finish(G, np.vstack([[bp["x"], bp["z"]], _xy(_chain(par, bp["cell"]))]), pose=bp)
    if T is None:
        return dict(kind="area", key=job["area"], routes=[], notes=[f"{job['area']}: trunk rejected"], seconds=time.time() - t0)
    base = dict(route_kind="spawn_post", variant=0)
    per_pose = {bp["id"]: T}
    tcells = {_cell(*q) for q in T["P"][::2]}
    for p in pc:
        if p is bp:
            continue
        ch = _chain(par, p["cell"])
        m = next((i for i, c in enumerate(ch) if int(c) in tcells), None)
        rec = None
        if m is not None:
            cx, cz = _xy(ch[m:m + 1])[0]
            j = int(np.argmin(np.hypot(T["P"][:, 0] - cx, T["P"][:, 1] - cz)))
            if j < len(T["P"]) - 3:
                rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(ch[:m + 1])]), pose=p, tail=T["P"][j:])
        if rec is None:
            rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(ch)]), pose=p)
        if rec is None:
            notes.append(f"{s1['id']}: pose {p['id']} rejected")
            continue
        per_pose[p["id"]] = rec
    for pid, rec in per_pose.items():
        out.append(_rec(base, pose=pid, target=s1["id"], **rec))
    for e in ends[1:]:
        kp = {}
        O = leadout(G, T, e, kp)
        for p in pc:
            if p["id"] not in per_pose:
                continue
            rec = splice(G, per_pose[p["id"]], T, O) if O is not None else None
            if rec is None:                    # direct route for this pose
                d2, p2 = kp["dist"], kp["par"]
                if d2.ravel()[p["cell"]] >= 1e19:
                    continue
                rec = R.finish(G, np.vstack([[p["x"], p["z"]], _xy(_chain(p2, p["cell"]))]), pose=p)
                notes.append(f"{e['id']}: pose {p['id']} by a direct route (no splice)")
            if rec is None:
                notes.append(f"{e['id']}: pose {p['id']} rejected")
                continue
            out.append(_rec(base, pose=p["id"], target=e["id"], **rec))
    return dict(kind="area", key=job["area"], routes=out, notes=notes, seconds=time.time() - t0)


def _close(paths):
    mark = np.zeros((_W["H5"], _W["W5"]), np.uint8)
    centres = []
    for P in paths:
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        for sv in np.arange(CLOSE_END, s[-1] - CLOSE_END + 1e-6, CLOSE_STEP):
            centres.append(_cell(*P[min(len(P) - 1, int(np.searchsorted(s, sv)))]))
    if centres:
        idx = np.array(centres, np.int32)
        _W["lib"].geodisc(np.ascontiguousarray(_W["unit"]), _W["H5"], _W["W5"], idx, len(idx), CLOSE_R / 0.5, mark)
    return mark


def job_between(job):
    """From one zone: Dijkstra from its inner disc; best route to every post / slot / later zone (start at the
    zone); zone <-> zone also one alternative with the first one's middle closed."""
    G = _W["G"]
    t0 = time.time()
    src = np.asarray(job["inner"], np.int64)
    dist, par = _search(_W["cost"], src)
    out, notes = [], []
    for t in job["ends"]:
        if t["kind"] == "area":
            s1 = t["slots"][0]
            if dist.ravel()[s1["cell"]] >= 1e19:
                notes.append(f"{job['zone']}->{t['id']}: unreachable")
                continue
            T = R.finish(G, _xy(_chain(par, s1["cell"])[::-1]), pose=None)
            if T is None:
                notes.append(f"{job['zone']}->{s1['id']}: rejected")
                continue
            out.append(_rec(dict(route_kind="zone_post", start_target=job["zone"], target=s1["id"], variant=0), **T))
            for e in t["slots"][1:]:
                rec = leadout(G, T, e)
                if rec is None and dist.ravel()[e["cell"]] < 1e19:
                    rec = R.finish(G, _xy(_chain(par, e["cell"])[::-1]), pose=None)
                if rec is None:
                    notes.append(f"{job['zone']}->{e['id']}: rejected")
                    continue
                out.append(_rec(dict(route_kind="zone_post", start_target=job["zone"], target=e["id"], variant=0), **rec))
            continue
        if t["kind"] == "zone":
            tc = np.asarray(t["inner"], np.int64)
            k = int(tc[np.argmin(dist.ravel()[tc])])
        else:
            k = t["cell"]
        if dist.ravel()[k] >= 1e19:
            notes.append(f"{job['zone']}->{t['id']}: unreachable")
            continue
        ch = _chain(par, k)[::-1]               # zone -> target
        raw = _xy(ch)
        if t["kind"] != "zone":
            raw = np.vstack([raw, [t["x"], t["z"]]]) if math.hypot(raw[-1, 0] - t["x"], raw[-1, 1] - t["z"]) > 0.3 else raw
        rec = R.finish(G, raw, pose=None)
        if rec is None:
            notes.append(f"{job['zone']}->{t['id']}: rejected")
            continue
        kind = "zone_zone" if t["kind"] == "zone" else "zone_post"
        out.append(_rec(dict(route_kind=kind, start_target=job["zone"], target=t["id"], variant=0), **rec))
        if t["kind"] == "zone":
            mark = _close([rec["P"]])
            mark.ravel()[src] = 0
            mark.ravel()[np.asarray(t["inner"], np.int64)] = 0
            c2 = np.where(mark > 0, 0, _W["cost"]).astype(np.float32)
            d2, p2 = _search(c2, src)
            k2 = int(tc[np.argmin(d2.ravel()[tc])])
            if d2.ravel()[k2] < 1e19 and d2.ravel()[k2] <= ZZ_TIME * dist.ravel()[k]:
                rec2 = R.finish(G, _xy(_chain(p2, k2)[::-1]), pose=None)
                if rec2 is not None:
                    Q = rec["P"][::4]
                    dm = np.sqrt(((rec2["P"][::4, None, :] - Q[None]) ** 2).sum(-1)).min(1)
                    if float((dm <= 10.0).mean()) <= ZZ_OVERLAP:
                        out.append(_rec(dict(route_kind=kind, start_target=job["zone"], target=t["id"], variant=1), **rec2))
    return dict(kind="between", key=job["zone"], routes=out, notes=notes, seconds=time.time() - t0)


def work(job):
    return {"zone": job_zone, "target": job_target, "between": job_between, "area": job_area}[job["type"]](job)


# ---------------------------------------------------------------- main side
def poses_for(mission, Fl, areas_s30, poses_s30):
    """routes.bin poses: sweden_dom from data/sweden/routes.json (logged positions and headings) when that file
    exists, otherwise (and for every other mission) the mission's spawn points with heading NaN. Each gets the
    analysis spawn area of its army whose centre is nearest."""
    if mission == "sweden_dom" and C.paths.SWEDEN_ROUTES.is_file():
        d = json.loads(C.paths.SWEDEN_ROUTES.read_text(encoding="utf-8"))
        out = []
        for p in d["poses"]:
            army = 1 if p["side"] == "S" else 2
            cand = [a for a in areas_s30 if a["army"] == army]
            a = min(cand, key=lambda a: math.hypot(a["centre"][0] - p["x"], a["centre"][1] - p["z"]))
            sn = Fl.snap(p["x"], p["z"], 20.0, Fl.valid05 & Fl.reach)
            out.append(dict(id=p["id"], army=army, area=a["id"], x=p["x"], z=p["z"], heading=p["heading"],
                            cell=None if sn is None else int(sn[0] * Fl.W5 + sn[1]), s25_area=p.get("area")))
        return out
    out = []
    for p in poses_s30:
        sn = Fl.snap(p["x"], p["z"], 20.0, Fl.valid05 & Fl.reach)
        out.append(dict(id=p["id"], army=p["army"], area=p["area"], x=p["x"], z=p["z"], heading=float("nan"),
                        cell=None if sn is None else int(sn[0] * Fl.W5 + sn[1])))
    return out


def standing_cell(Fl, x, z):
    """Post standing point: within 10 m the nearest cell with clearance >= 4.3 m, else the largest clearance
    (cells a hull centre may hold and reached from the spawns)."""
    r0, c0 = Fl.cell05(x, z)
    k = int(POST_STAND_R / 0.5)
    ra, rb, ca, cb = max(0, r0 - k), min(Fl.H5, r0 + k + 1), max(0, c0 - k), min(Fl.W5, c0 + k + 1)
    ok = (Fl.valid05 & Fl.reach)[ra:rb, ca:cb]
    rr, cc = np.nonzero(ok)
    if not len(rr):
        return None
    xs, zs = Fl.X0 + (cc + ca + 0.5) * 0.5, Fl.Z0 + (rr + ra + 0.5) * 0.5
    d = np.hypot(xs - x, zs - z)
    m = d <= POST_STAND_R
    if not m.any():
        return None
    rr, cc, xs, zs, d = rr[m], cc[m], xs[m], zs[m], d[m]
    clr = Fl.clr05[rr + ra, cc + ca]
    good = clr >= POST_STAND_CLR
    i = int(np.argmin(np.where(good, d, np.inf))) if good.any() else int(np.argmax(clr))
    return dict(cell=int((rr[i] + ra) * Fl.W5 + cc[i] + ca), sx=float(xs[i]), sz=float(zs[i]),
                clearance=float(clr[i]), rule="clearance>=4.3" if good.any() else "largest clearance")


ZONE_SLOT_SEP = 8.0        # zone slots at least 8 m apart ...
ZONE_SLOT_HALF_HULL = 3.7  # ... anywhere in the capture circle a hull fits in (radius - half a hull length)


def zone_slots(Fl, t, routes):
    """Slots on every 0.5 m cell a hull centre may hold (clearance >= 2.3 m, reached from
    the spawns) inside the capture circle minus half a hull length, at least 8 m apart: the cell nearest the centre
    first, then farthest-point picks until no cell is 8 m from all slots. No count limit."""
    rr, cc = np.nonzero(Fl.valid05 & Fl.reach)
    xs, zs = Fl.X0 + (cc + 0.5) * 0.5, Fl.Z0 + (rr + 0.5) * 0.5
    inside = np.hypot(xs - t["x"], zs - t["z"]) <= t["radius"] - ZONE_SLOT_HALF_HULL
    rr, cc, xs, zs = rr[inside], cc[inside], xs[inside], zs[inside]
    if not len(rr):
        return []
    first = int(np.argmin(np.hypot(xs - t["x"], zs - t["z"])))
    sel = [first]
    emin = np.hypot(xs - xs[first], zs - zs[first])
    while True:
        k = int(np.argmax(emin))
        if emin[k] < ZONE_SLOT_SEP:
            break
        sel.append(k)
        emin = np.minimum(emin, np.hypot(xs - xs[k], zs - zs[k]))
    return [dict(x=float(xs[k]), z=float(zs[k]), clearance=float(Fl.clr05[rr[k], cc[k]])) for k in sel]


def flank_variants(corrs):
    """Per (area, zone):
    front corridors (variant 0, one of them designated per pose): every corridor within 1.3 x the fastest time whose
      seconds under observation exceed the recommended one's by at most 10 s (the recommended one always);
    flank variants 1, 2: other corridors (not front, not the merely fastest) within 2 x the fastest time whose entry
      is >= 45 deg from the recommended one's and from each other, by time, at most two."""
    groups = defaultdict(list)
    for c in corrs:
        groups[(c["area"], c["zone"])].append(c)
    for key, cs in groups.items():
        fast = min(c["time"] for c in cs)
        rec = next(c for c in cs if c["role"] in ("recommended", "both"))
        for c in cs:
            c["front"] = c is rec or (c["time"] <= FRONT_TIME * fast and c["expo_s"] <= rec["expo_s"] + FRONT_EXPO_S)
            c["variant"] = 0 if c["front"] else -1
        chosen = [rec]
        for c in sorted(cs, key=lambda c: c["time"]):
            if c["front"] or c["role"] == "fastest" or c["time"] > FLANK_TIME * fast or len(chosen) >= 3:
                continue
            if all(angdiff(c["entry_deg"], d["entry_deg"]) >= FLANK_ANGLE for d in chosen):
                c["variant"] = len(chosen)
                chosen.append(c)
        for c in cs:
            c["pack_role"] = "recommended" if c is rec else ("flank" if c["variant"] > 0 else "other")


def _first(P, length=CROSS_LEN):
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    return P[: max(2, int(np.searchsorted(s, length)))][::2]


def _crosses(P, Q):
    """Do two polylines cross (proper segment intersection)?"""
    def orient(a, b, c):
        return (b[..., 0] - a[..., 0]) * (c[..., 1] - a[..., 1]) - (b[..., 1] - a[..., 1]) * (c[..., 0] - a[..., 0])
    A0, A1 = P[:-1][:, None, :], P[1:][:, None, :]
    B0, B1 = Q[:-1][None, :, :], Q[1:][None, :, :]
    d1, d2 = orient(A0, A1, B0), orient(A0, A1, B1)
    d3, d4 = orient(B0, B1, A0), orient(B0, B1, A1)
    return bool(((((d1 > 0) & (d2 < 0)) | ((d1 < 0) & (d2 > 0))) & (((d3 > 0) & (d4 < 0)) | ((d3 < 0) & (d4 > 0)))).any())


def designate(routes, corrs, pose_by_area):
    """Per (spawn area, zone): every pose has a candidate route (variant 0) on each front corridor; choose one per pose
    in this order: fewest pairs of chosen routes crossing in their first 150 m, then the most even number of poses
    per front corridor (smallest max - min), then the least total time. Local search from the fastest choice: single
    pose moves until no move improves the objective. Unchosen candidates are dropped."""
    cand = defaultdict(lambda: defaultdict(dict))        # (area, zone) -> pose -> corridor -> route
    keep = []
    front_ids = defaultdict(list)
    for c in corrs:
        if c.get("front"):
            front_ids[(c["area"], c["zone"])].append(c["id"])
    area_of = {p["id"]: a for a, ps in pose_by_area.items() for p in ps}
    for r in routes:
        if r["route_kind"] == "spawn_zone" and r["variant"] == 0:
            cand[(area_of[r["pose"]], r["target"][-1])][r["pose"]][r["corridor"]] = r
        else:
            keep.append(r)
    report = {}
    for key, per_pose in cand.items():
        fids = front_ids[key]
        poses = sorted(per_pose)
        opts = {p: [f for f in fids if f in per_pose[p]] for p in poses}
        first = {(p, f): _first(per_pose[p][f]["P"]) for p in poses for f in opts[p]}
        X = {}
        items = list(first)
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if items[i][0] != items[j][0]:
                    X[(items[i], items[j])] = X[(items[j], items[i])] = _crosses(first[items[i]], first[items[j]])

        def objective(ch):
            cr = sum(1 for i, p in enumerate(poses) for q in poses[i + 1:] if X.get(((p, ch[p]), (q, ch[q])), False))
            cnt = Counter(ch.values())
            counts = [cnt.get(f, 0) for f in fids if any(f in opts[p] for p in poses)]
            imb = max(counts) - min(counts) if counts else 0
            tt = sum(per_pose[p][ch[p]]["time_s"] for p in poses)
            return (cr, imb, tt)
        ch = {p: min(opts[p], key=lambda f: per_pose[p][f]["time_s"]) for p in poses if opts[p]}
        poses = [p for p in poses if opts[p]]
        best = objective(ch)
        for _ in range(60):
            improved = False
            for p in poses:
                for f in opts[p]:
                    if f == ch[p]:
                        continue
                    old = ch[p]
                    ch[p] = f
                    ob = objective(ch)
                    if ob < best:
                        best, improved = ob, True
                    else:
                        ch[p] = old
            if not improved:
                break
        for p in poses:
            keep.append(per_pose[p][ch[p]])
        cnt = Counter(ch.values())
        report[f"{key[0]}>{key[1]}"] = dict(front_corridors=len(fids), poses_per_corridor={f: cnt.get(f, 0) for f in fids},
                                           crossings_first_150m=best[0], imbalance=best[1])
    return keep, report


def write_bins(od, Fl, poses, targets, zslots, routes):
    import struct
    RH = struct.Struct("<8sIIIII")
    RP = struct.Struct("<8s8sfff")
    RT = struct.Struct("<16sBBBBffffffff")
    RS = struct.Struct("<HHfff")
    RR = struct.Struct("<32sBBBBHHHBBffffI")
    KIND = {"spawn_zone": 0, "spawn_post": 1, "zone_zone": 2, "zone_post": 3}
    PKIND = {"choke": 1, "second crew": 2, "overwatch": 3, "gate": 4}
    GEAR = {"none": 0, "forward": 1, "reverse": 2, "blocked": 3}
    enc = lambda v, n: (v or "").encode()[:n]
    pi = {p["id"]: i for i, p in enumerate(poses)}
    ti = {t["id"]: i for i, t in enumerate(targets)}
    sl = [(ti[tid], k, s) for tid, lst in zslots.items() for k, s in enumerate(lst)]
    # ranks: per pose, targets (zones + own army's posts / slots) by best total time
    best = {}
    for r in routes:
        if r["route_kind"] in ("spawn_zone", "spawn_post"):
            k = (r["pose"], r["target"])
            best[k] = min(best.get(k, 1e9), r["total_s"])
    ranks = np.zeros((len(poses), len(targets)), np.uint8)
    for p in poses:
        ts = sorted((v, t) for (pp, t), v in best.items() if pp == p["id"])
        for k, (_, t) in enumerate(ts):
            ranks[pi[p["id"]], ti[t]] = min(255, k + 1)
    with open(od / "routes.bin", "wb") as f:
        f.write(RH.pack(b"WTRTE004", 4, len(poses), len(targets), len(sl), len(routes)))
        for p in poses:
            f.write(RP.pack(enc(p["id"], 8), enc(p["area"], 8), p["x"], p["z"], p["heading"]))
        for t in targets:
            fx, fz = t.get("faces") or (float("nan"), float("nan"))
            f.write(RT.pack(enc(t["id"], 16), 0 if t["kind"] == "zone" else 1, t.get("army") or 0,
                            PKIND.get(t.get("post_kind"), 0), 1 if t.get("standing", True) else 0, t["x"], t["z"],
                            t["sx"], t["sz"], t.get("radius") or 0.0, t.get("inner_radius") or 0.0, fx, fz))
        for k, n, s in sl:
            f.write(RS.pack(k, n, s["x"], s["z"], s["clearance"]))
        f.write(ranks.tobytes())
        for r in routes:
            P = r["P"]
            flags = (1 if r.get("pinned") else 0) | (2 if r.get("designated") else 0) | \
                    (4 if r.get("fwd_free") else 0) | (8 if r.get("rev_free") else 0)
            sp = r["route_kind"] in ("spawn_zone", "spawn_post")
            f.write(RR.pack(enc(r["id"], 32), KIND[r["route_kind"]], min(255, r["rank"]), flags, min(255, r["fallback"]),
                            pi[r["pose"]] if sp else 0xFFFF, 0xFFFF if sp else ti[r["start_target"]], ti[r["target"]],
                            GEAR[r.get("gear", "none")], 0, r["total_s"], r["time_s"], r.get("pivot") or 0.0,
                            r["length"], len(P)))
            f.write(np.asarray(P, "<f4").tobytes())
            f.write(np.clip(np.floor(np.asarray(r["clr"]) * 10 + 1e-6), 0, 255).astype(np.uint8).tobytes())
            f.write(np.asarray(r["kappa"], "<f4").tobytes())
    hard = np.isin(Fl.cls, C.HARD_CODES)
    bits = np.packbits(hard, axis=1, bitorder="little")
    c05 = np.clip(np.floor(Fl.clr05 * 10 + 1e-6), 0, 255).astype(np.uint8)
    MH = struct.Struct("<8sffffIIII")
    with open(od / "map.bin", "wb") as f:
        f.write(MH.pack(b"WTMAP004", Fl.X0, Fl.Z0, C.CELL, 0.5, hard.shape[1], hard.shape[0], c05.shape[1], c05.shape[0]))
        f.write(bits.tobytes())
        f.write(c05.tobytes())


def main():
    mission = sys.argv[1]
    workers = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else WORKERS
    timing = {}
    t_all = time.time()
    od = C.out_dir(mission)
    cache = od / "cache"
    cache.mkdir(exist_ok=True)
    # ---------------- analysis (run30 code, base run)
    t = time.time()
    Fl = F.Field(mission)
    zones = C.zones(mission)
    kill = C.kill_areas(mission)
    areas, poses30 = F.spawn_areas(mission, Fl)
    Fl.set_reach([p["cell"] for p in poses30 if p["cell"] is not None])
    run30.MISSION_LEVEL = C.mission_level(mission)
    speeds = F.log_speeds(Fl, C.runs_of_level(run30.MISSION_LEVEL))
    P = dict(C.BASE)
    P["v_road"], P["v_off"] = speeds["v_road"], speeds["v_off"]
    base = run30.run_once(Fl, mission, P, areas, poses30, zones, kill, speeds)
    corrs = base["cache"]["corrs"]
    for c in corrs:
        zx, zz = next((z[1], z[2]) for z in zones if z[0] == c["zone"])
        c["entry_deg"] = ang(c["entry"]["x"] - zx, c["entry"]["z"] - zz)
    flank_variants(corrs)
    timing["analysis_s"] = round(time.time() - t, 1)
    log(f"analysis done ({timing['analysis_s']} s)")
    # ---------------- cache for the workers
    t = time.time()
    hard = np.isin(Fl.cls, C.HARD_CODES)
    D = (C.edt(Fl.lib, hard) * C.CELL - C.HALF_DIAG).astype(np.float32)
    v_road = P["v_road"] or P["v_road_fallback"]
    v_off = P["v_off"] or P["v_off_fallback"]
    np.save(cache / "D.npy", D)
    np.save(cache / "hard.npy", hard)
    np.save(cache / "tcost.npy", Fl.tcost.astype(np.float32))
    np.save(cache / "cost.npy", Fl.time_cost(v_road, v_off))
    np.save(cache / "unit.npy", Fl.unitR)
    np.save(cache / "meta.npy", np.array([Fl.X0, Fl.Z0], np.float64))
    del D, hard
    timing["cache_s"] = round(time.time() - t, 1)
    # ---------------- poses, targets
    poses = poses_for(mission, Fl, areas, poses30)
    pose_by_area = defaultdict(list)
    for p in poses:
        pose_by_area[p["area"]].append(p)
    targets = []
    for (L, x, z, r) in zones:
        inner = []
        rr, cc = np.nonzero(Fl.valid05 & Fl.reach)
        ri = r - INNER_MARGIN
        m = np.hypot(Fl.X0 + (cc + 0.5) * 0.5 - x, Fl.Z0 + (rr + 0.5) * 0.5 - z) <= ri
        inner = (rr[m] * Fl.W5 + cc[m]).astype(np.int64)
        targets.append(dict(id=f"zone_{L}", zone=L, kind="zone", army=0, x=x, z=z, sx=x, sz=z, radius=r, inner_radius=ri,
                            inner=inner))
    raise SystemExit("pack31.main is the retired s31 entry (it generated the old posts); use run_map.py")
    no_room = []
    for p in posts:
        st = standing_cell(Fl, p["x"], p["z"])
        t_ = dict(id=p["id"], kind="post", army=p["army"], post_kind=p["kind"], x=p["x"], z=p["z"], faces=p.get("faces"))
        if st is None:
            t_.update(standing=False, sx=float("nan"), sz=float("nan"))
            no_room.append(p["id"])
        else:
            t_.update(standing=True, sx=st["sx"], sz=st["sz"], cell=st["cell"], stand_rule=st["rule"], stand_clr=st["clearance"])
        targets.append(t_)
    pack_areas = plan31.pack_areas(base, Fl, zones, areas)
    for a in pack_areas:
        for k, s in enumerate(a["slots"]):
            sid = f"{a['id']}_{k + 1}"
            s["target_id"] = sid
            targets.append(dict(id=sid, kind="post", army=a["army"], post_kind="overwatch", x=s["x"], z=s["z"], sx=s["x"],
                                sz=s["z"], faces=[a["watch_x"], a["watch_z"]], standing=True, cell=s["cell"],
                                plan_area=a["id"], first_area=a["first_area"]))
    log(f"{len(poses)} poses, {len(targets)} targets ({len(posts)} posts, {sum(len(a['slots']) for a in pack_areas)} plan slots)")
    # ---------------- jobs
    jobs = []
    zt = {t["zone"]: t for t in targets if t["kind"] == "zone"}
    for c in corrs:
        if c["variant"] < 0:
            continue
        jobs.append(dict(type="zone", key=f"{c['id']}#v{c['variant']}", cells=c["cells"].tolist(),
                         inner=zt[c["zone"]]["inner"].tolist(), target=f"zone_{c['zone']}", variant=c["variant"],
                         corridor=c["id"], poses=[_strip(p) for p in pose_by_area[c["area"]]]))
    for t_ in targets:
        if t_["kind"] != "post" or not t_.get("standing") or t_.get("plan_area"):
            continue
        ar = {a["id"]: [_strip(p) for p in pose_by_area[a["id"]]] for a in areas if a["army"] == t_["army"]}
        jobs.append(dict(type="target", target=t_["id"], cell=t_["cell"], areas=ar))
    area_ends = []
    for a in pack_areas:
        sl = [dict(id=s["target_id"], cell=s["cell"], x=s["x"], z=s["z"]) for s in a["slots"]]
        if not sl:
            continue
        area_ends.append(dict(kind="area", id=a["id"], slots=sl))
        jobs.append(dict(type="area", area=a["id"], ends=sl, poses=[_strip(p) for p in pose_by_area[a["first_area"]]]))
    zlist = [t for t in targets if t["kind"] == "zone"]
    for i, zt_ in enumerate(zlist):
        ends = [dict(kind="zone", id=o["id"], inner=o["inner"].tolist()) for o in zlist[i + 1:]]
        ends += [dict(kind="post", id=t_["id"], cell=t_["cell"], x=t_["sx"], z=t_["sz"]) for t_ in targets
                 if t_["kind"] == "post" and t_.get("standing") and not t_.get("plan_area")]
        ends += area_ends
        for k in range(0, len(ends), BETWEEN_CHUNK):
            jobs.append(dict(type="between", zone=zt_["id"], inner=zt_["inner"].tolist(), ends=ends[k:k + BETWEEN_CHUNK]))
    # longest first
    jobs.sort(key=lambda j: {"between": 0, "zone": 1, "area": 2, "target": 3}[j["type"]])
    t = time.time()
    results = []
    with mp.get_context("spawn").Pool(workers, initializer=_init, initargs=(str(cache),)) as pool:
        for k, r in enumerate(pool.imap_unordered(work, jobs)):
            results.append(r)
            if k % 50 == 0:
                log(f"  job {k + 1}/{len(jobs)}")
    timing["routes_parallel_s"] = round(time.time() - t, 1)
    job_secs = Counter()
    for r in results:
        job_secs[r["kind"]] += r["seconds"]
    timing["cpu_s_by_job_kind"] = {k: round(v, 1) for k, v in job_secs.items()}
    log(f"routes done ({timing['routes_parallel_s']} s wall)")
    # ---------------- assemble routes
    routes, notes = [], []
    for r in results:
        routes += r["routes"]
        notes += r["notes"]
    pose_d = {p["id"]: p for p in poses}
    for r in routes:
        r["total_s"] = round(r["time_s"] + abs(r.get("pivot") or 0.0) / R.START_TURN_DEG_S, 2)
        r["time_s"] = round(r["time_s"], 2)
    routes, designation = designate(routes, corrs, pose_by_area)
    by_pt = defaultdict(list)
    for r in routes:
        key = (r.get("pose"), r.get("start_target"), r["target"])
        by_pt[key].append(r)
    for key, rs in by_pt.items():
        rs.sort(key=lambda r: (r["variant"], r["total_s"]))
        order = sorted(rs, key=lambda r: r["total_s"])
        for r in rs:
            r["rank"] = order.index(r)
            r["fallback"] = r["variant"]
            r["designated"] = r["variant"] == 0
            if r["route_kind"].startswith("spawn"):
                r["id"] = f"{r['pose']}>{r['target']}#{r['variant']}"
            else:
                r["id"] = f"{r['start_target']}<>{r['target']}#{r['variant']}"
    kind_order = {"spawn_zone": 0, "spawn_post": 1, "zone_zone": 2, "zone_post": 3}
    routes.sort(key=lambda r: (kind_order[r["route_kind"]], r["id"]))
    t = time.time()
    zslots = {t_["id"]: zone_slots(Fl, t_, routes) for t_ in targets if t_["kind"] == "zone"}
    timing["zone_slots_s"] = round(time.time() - t, 1)
    for t_ in targets:
        t_.pop("inner", None)
    t = time.time()
    write_bins(od, Fl, poses, targets, zslots, routes)
    timing["write_s"] = round(time.time() - t, 1)
    t = time.time()
    rc = subprocess.run([str(READ_CHECK), str(od / "routes.bin"), str(od / "map.bin")], capture_output=True, text=True)
    timing["read_check_s"] = round(time.time() - t, 1)
    (od / "read_check.txt").write_text(rc.stdout + rc.stderr, encoding="utf-8")
    log("read_check:\n" + rc.stdout)
    # ---------------- plan pack
    tindex = {t["id"]: i for i, t in enumerate(targets)}
    pack = plan31.build_pack(mission, base, areas, poses, zones, corrs, pack_areas, tindex, zt,
                             capacity={L: len(zslots[f"zone_{L}"]) for L, *_r in zones}, designation=designation)
    (od / f"{mission}.plan.json").write_text(json.dumps(pack, indent=1), encoding="utf-8")
    # ---------------- pictures
    t = time.time()
    import render31
    pics = render31.render(Fl, base, mission, od, zones, areas, poses, routes, pack, pack_areas, corrs)
    timing["pictures_s"] = round(time.time() - t, 1)
    timing["total_s"] = round(time.time() - t_all, 1)
    summary = dict(mission=mission, timing=timing, workers=workers, jobs=Counter(j["type"] for j in jobs),
                   poses=len(poses), targets=Counter(t["kind"] for t in targets), plan_slot_targets=sum(1 for t in targets if t.get("plan_area")),
                   posts=len(posts), posts_without_standing_room=no_room, post_derivation_log=post_log,
                   post_standing_rule=Counter(t.get("stand_rule") for t in targets if t["kind"] == "post" and not t.get("plan_area")),
                   routes=Counter(r["route_kind"] for r in routes),
                   routes_by_variant=Counter(f"{r['route_kind']}#{r['variant']}" for r in routes),
                   designated=sum(1 for r in routes if r["designated"]), zone_slots={k: len(v) for k, v in zslots.items()},
                   designation=designation, support=pack.get("support"),
                   cut_ends=dict(start=sum(1 for r in routes if r.get("cut_start_m")), end=sum(1 for r in routes if r.get("cut_end_m"))),
                   no_start_turn_data=sum(1 for r in routes if r.get("no_start_turn_data")),
                   notes=notes, files={p.name: p.stat().st_size for p in [od / "routes.bin", od / "map.bin", od / f"{mission}.plan.json"]},
                   pictures=pics, read_check=rc.stdout.strip().splitlines())
    (od / "pack_summary.json").write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("timing", "poses", "targets", "routes", "routes_by_variant", "designated",
                                              "cut_ends", "files")}, indent=1, default=str))
    print(f"notes: {len(notes)}", notes[:20])


def _strip(p):
    return {k: p[k] for k in ("id", "x", "z", "heading", "cell")}


if __name__ == "__main__":
    main()
