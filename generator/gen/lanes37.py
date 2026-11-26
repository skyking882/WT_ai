"""Lanes: the lane routed on the grid, bounds beside it, toll positions.

A lane = one corridor from a spawn area to ONE end (a zone's capture circle, a fire-support position onto a zone, or
a position that watches the enemy's traffic). The curve is not pinned to cover points (that made tanks weave and
drive slower; driving straight is often faster; cover matters only where the enemy can plausibly have a gun on the
tank):
  * route: least-cost path on the 0.5 m grid, cell cost = cost.npy x (1 + OPEN_PEN x open), open = cover35's
    time-aware open grid of the spawn area (E(c) <= t + MARGIN_S); no dash term, no cover graph. Before the
    first open cell this is simply the fastest route. graph_time_s = plain
    driving time along it (cost.npy).
  * circle alternatives per (spawn area, zone) by banning the cells within ALT_BAN_R of earlier tries (not near the
    spawn area or the zone), accepted within CIRCLE_TIME x the first lane's time and OVERLAP_MAX overlap.
  * driving curve: route31 finish (smoothing, repair, trim) of the raw route + the float32 check; no pinning.
  * contact line = arc position of the first curve sample in an open cell; bounds = cover points of the spawn area
    within BESIDE_S of driving from the curve, from contact - 20 m on, one per BOUND_STEP of arc (turn room first,
    then the fewest seconds from the curve), at most MAX_BOUNDS (thin_bounds). The game drives to a bound only when
    engaged, by its own grid search, so a bound need not lie on the curve.
  * toll positions: army a's standing positions that overlook the enemy's
    circle / support routes (traffic points every 10 m weighted by the seconds spent there, within TOLL_RANGE_M),
    picked greedily by coverage; lanes to them from every spawn area within TOLL_AREA_TIME x the fastest.
Files: routes.bin WTRTE004 / map.bin WTMAP004 (binary_v4.py layouts, read_check), <mission>.plan.json WTPLAN002.
usage: lanes37.py <mission> [--workers N] [--resume]
"""
import copy
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
import cover35 as CV
import field30 as F
import pack31 as PK
import plan35
import route31 as R
import run30
import sight30

WORKERS = 13
LANE_CAP = 2
OPEN_PEN = 1.0             # an exposed second counts double in the route search
CIRCLE_MAX, CIRCLE_TIME, CIRCLE_TRIES = 4, 1.5, 14   # per (spawn area, zone): at most 4 distinct lanes within 1.5 x
CONTACT_DASH_M = 20.0      # contact line: start of the first open dash of at least 20 m (no such dash: no contact)
LANE_CLEAR_M = 10.0        # position standing cells: not within 10 m of a circle / support lane route of either army
ALT_KEEP_START, ALT_KEEP_END, ALT_BAN_R = 150.0, 60.0, 25.0
OVERLAP_R, OVERLAP_MAX = 10.0, 0.5
SUPPORT_MAX, SUPPORT_GAP, SUPPORT_RMAX, SUPPORT_SEE = 4, 15.0, 400.0, 0.25
OWN_SIDE_DEG, FLANK_DEG = 50.0, 120.0
POS_SEP = 80.0
SLOT2_R = (10.0, 30.0)
FIRE_MARGIN_S = 15.0
ZONE_HALF_HULL = 3.7
CONNECT_MAXT = 20.0
BRIDGE_NEAR_M = 8.0
END_BASE = 1_000_000       # end ids
# bounds beside the lane
BESIDE_S = 6.0             # a bound is reached from the curve by at most 6 s of driving
BOUND_STEP = 40.0          # one bound per 40 m of arc
BOUND_BEFORE_CONTACT = 20.0   # bounds from contact_m - 20 m on
# toll positions (watch the enemy's traffic)
TRAFFIC_STEP = 10.0        # enemy traffic points every 10 m of route
GUARD_SPAWN_SKIP = 150.0   # ... not within 150 m of the enemy's spawn areas
TOLL_RANGE_M = 500.0
TOLL_MIN_S = 20.0
TOLL_MAX = 6
TOLL_SLOT_R = 30.0         # standing cells within 30 m of a picked position
TOLL_HULL_DOWN = 0.5       # hull-down: S.hull_down >= 0.5 towards the points seen
TOLL_HD_POINTS = 400       # hull-down test against at most 400 of the points seen (even stride)
TOLL_AREA_TIME = 1.5       # lanes from every spawn area within 1.5 x the fastest area's time
# table limits of the route-following program that reads the pack
MAX_LANES_PACK, MAX_SLOTS_LANE, MAX_BOUNDS, MAX_NEIGHBOURS, MAX_OPPOSES, MAX_SG_SLOTS_ARMY = 96, 4, 16, 4, 12, 40
GUARD_RESERVE = 12         # position lanes an army can have (two areas x six positions), kept free when trimming


def log(msg, t0=[time.time()]):
    print(f"[{time.time() - t0[0]:7.1f}s] {msg}", flush=True)


def ang(dx, dz):
    return math.degrees(math.atan2(dz, dx))


# ================================================================ route workers
def _init_routes(cache):
    PK._init(cache)
    PK._W["cache"] = cache


def _arr_of(kind, tag):
    """open_<tag>.npy (uint8 0.5 m open grid) / wcost_<tag>.npy (weighted search cost), memory-mapped."""
    k = f"{kind}_{tag}"
    if k not in PK._W:
        PK._W[k] = np.load(Path(PK._W["cache"]) / f"{kind}_{tag}.npy", mmap_mode="r")
    return PK._W[k]


def _nodes_of(tag):
    k = f"nodes_{tag}"
    if k not in PK._W:
        z = np.load(Path(PK._W["cache"]) / f"nodes_{tag}.npz")
        PK._W[k] = {n: z[n] for n in z.files}
    return PK._W[k]


def _connect(a, b):
    """0.5 m cells from cell a to cell b (bounded search from b with a as the goal); [a, b] if not found."""
    if a == b:
        return np.array([a], np.int64)
    dist, par = PK._search(PK._W["cost"], [int(b)], maxd=CONNECT_MAXT, goal=int(a))
    if dist.ravel()[a] < 1e19:
        return PK._chain(par, int(a))
    return np.array([a, b], np.int64)


def _route(wc, src, dst, maxd=0.0):
    """Least-cost 0.5 m path from the source cells (spawn poses: the best one) to the best of the destination cells.
    -> (cells source .. destination, weighted cost) or (None, None)."""
    dst = np.asarray(dst, np.int64)
    goal = int(dst[0]) if len(dst) == 1 else -1
    dist, par = PK._search(wc, [int(c) for c in src], maxd=maxd, goal=goal)
    dd = dist.ravel()[dst]
    k = int(np.argmin(dd))
    if dd[k] >= 1e19 or (maxd > 0 and dd[k] > maxd):
        return None, None
    return PK._chain(par, int(dst[k]))[::-1].copy(), float(dd[k])


def _route_rec(cells, wcost_s, tag):
    """Raw route record: cells, samples every 2 m (raw), their arc (raw_s) and plain driving time (raw_t)."""
    cost = PK._W["cost"]
    W5 = PK._W["W5"]
    cells = np.asarray(cells, np.int64)
    c = np.asarray(cost.ravel()[cells], np.float64)
    rr, cc = cells // W5, cells % W5
    diag = (np.diff(rr) != 0) & (np.diff(cc) != 0)
    step = np.where(diag, math.sqrt(2.0), 1.0)
    dt = step * 0.5 * (c[:-1] + c[1:])
    ct = np.concatenate([[0.0], np.cumsum(dt)])
    s = np.concatenate([[0.0], np.cumsum(step * 0.5)])
    o = np.asarray(_arr_of("open", tag).ravel()[cells]) > 0
    open_m = float((step * 0.5)[o[:-1] & o[1:]].sum())
    idx = np.unique(np.append(np.arange(0, len(cells), 4), len(cells) - 1))
    return dict(cells=cells, raw=PK._xy(cells[idx]), raw_s=s[idx], raw_t=ct[idx], graph_time_s=round(float(ct[-1]), 1),
                graph_cost_s=round(float(wcost_s), 1), graph_open_m=round(open_m, 1))


ZONE_CLOSE_HULL = 4.1     # zone rule: a lane not ending in zone Z may not enter Z's radius + the hull's half diagonal


def _zone_cells(L, x, z, r):
    """0.5 m cells (flat indices) within r + ZONE_CLOSE_HULL of a zone centre (cached per worker)."""
    k = f"zclose_{L}"
    if k not in PK._W:
        H5, W5 = PK._W["H5"], PK._W["W5"]
        R_ = r + ZONE_CLOSE_HULL
        c0 = max(0, int((x - R_ - PK._W["X0"]) / 0.5) - 1)
        c1 = min(W5 - 1, int((x + R_ - PK._W["X0"]) / 0.5) + 1)
        r0 = max(0, int((z - R_ - PK._W["Z0"]) / 0.5) - 1)
        r1 = min(H5 - 1, int((z + R_ - PK._W["Z0"]) / 0.5) + 1)
        rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
        X = PK._W["X0"] + (cc + 0.5) * 0.5
        Z = PK._W["Z0"] + (rr + 0.5) * 0.5
        m = np.hypot(X - x, Z - z) <= R_
        PK._W[k] = (rr[m] * W5 + cc[m]).astype(np.int64)
    return PK._W[k]


def _closed_wc(wc, zones):
    """Copy of the search cost with the cells of the given zones closed (cost 0 = closed)."""
    out = np.array(wc, dtype=np.float32, copy=True)
    for (L, x, z, r) in zones:
        out.ravel()[_zone_cells(L, x, z, r)] = 0
    return out


def _passes(cells, zones):
    """Zones (ids) whose closed disk the cells enter."""
    cs = set(np.asarray(cells, np.int64).tolist())
    return [L for (L, x, z, r) in zones if not cs.isdisjoint(_zone_cells(L, x, z, r).tolist())]


def _route_rule(wc, job, maxd=0.0):
    """Zone rule: route with every zone of job['close'] closed (a zone holding one of the spawn's own source cells
    is exempt). No route: the zones the unrestricted route passes are opened and the search repeated (if that fails
    too, the unrestricted route is used). -> (cells, cost, through_zone ids, the cost grid used)."""
    close = [zz for zz in job.get("close", []) if not set(int(c) for c in job["src"]) & set(_zone_cells(*zz).tolist())]
    wcz = _closed_wc(wc, close) if close else wc
    ch, wd = _route(wcz, job["src"], job["dst"], maxd=maxd)
    if ch is not None or not close:
        return ch, wd, [], wcz
    ch0, wd0 = _route(wc, job["src"], job["dst"], maxd=maxd)
    if ch0 is None:
        return None, None, [], wc
    thru = _passes(ch0, close)
    rest = [zz for zz in close if zz[0] not in thru]
    wcz2 = _closed_wc(wc, rest) if rest else wc
    ch, wd = _route(wcz2, job["src"], job["dst"], maxd=maxd)
    if ch is None:
        return ch0, wd0, _passes(ch0, close), wc
    return ch, wd, _passes(ch, close), wcz2


def job_circle(job):
    """Circle lanes of one (spawn area, zone): the least-cost route, then alternatives with the cells near earlier
    tries banned (bans accumulate; not within ALT_KEEP_START of the spawn centre or r + ALT_KEEP_END of the zone).
    Other zones closed (_route_rule); the alternatives search on the same closed grid."""
    t0 = time.time()
    tag = job["tag"]
    ch, wd, thru, wc = _route_rule(_arr_of("wcost", tag), job)
    if ch is None:
        return dict(kind="circle", key=job["key"], lanes=[], tries=["no route"], seconds=time.time() - t0, routes=[])
    first = _route_rec(ch, wd, tag)
    first["through_zone"] = thru
    got, tries, last = [first], [], first
    H5, W5 = PK._W["H5"], PK._W["W5"]
    xs = PK._W["X0"] + (np.arange(W5) + 0.5) * 0.5
    zs = PK._W["Z0"] + (np.arange(H5) + 0.5) * 0.5
    keep = (((xs[None, :] - job["sx"]) ** 2 + (zs[:, None] - job["sz"]) ** 2 <= ALT_KEEP_START ** 2)
            | ((xs[None, :] - job["zx"]) ** 2 + (zs[:, None] - job["zz"]) ** 2 <= (job["r"] + ALT_KEEP_END) ** 2))
    banned = np.zeros((H5, W5), bool)
    maxd = CIRCLE_TIME * (1.0 + OPEN_PEN) * first["graph_time_s"] + 1.0   # weighted cost <= (1 + OPEN_PEN) x time
    for _ in range(CIRCLE_TRIES):
        if len(got) >= CIRCLE_MAX:
            break
        mark = np.zeros((H5, W5), np.uint8)
        mark.ravel()[last["cells"]] = 1
        new = (C.edt(PK._W["lib"], mark) * 0.5 <= ALT_BAN_R) & ~keep & ~banned
        if not new.any():
            tries.append("nothing new to ban")
            break
        banned |= new
        wcb = np.where(banned, np.float32(0), wc).astype(np.float32)
        ch, wd = _route(wcb, job["src"], job["dst"], maxd=maxd)
        if ch is None:
            tries.append("no route within the time bound")
            break
        cand = _route_rec(ch, wd, tag)
        cand["through_zone"] = _passes(ch, [zz for zz in job.get("close", []) if zz[0] in thru]) if thru else []
        last = cand
        if cand["graph_time_s"] > CIRCLE_TIME * first["graph_time_s"]:
            tries.append(f"time {cand['graph_time_s']} > {CIRCLE_TIME} x {first['graph_time_s']}")
            break
        dup = None
        for g in got:
            dmin = np.sqrt(((cand["raw"][:, None, :] - g["raw"][None, :, :]) ** 2).sum(-1)).min(1)
            ov = float((dmin <= OVERLAP_R).mean())
            if ov > OVERLAP_MAX:
                dup = ov
                break
        if dup is None:
            got.append(cand)
            tries.append(f"accepted {cand['graph_time_s']} s")
        else:
            tries.append(f"overlap {dup:.2f}")
    return dict(kind="circle", key=job["key"], lanes=got, tries=tries, seconds=time.time() - t0, routes=[])


def job_point(job):
    """One lane's raw route from a spawn area's poses to one standing cell (support / position lane)."""
    t0 = time.time()
    ch, wd, thru, _ = _route_rule(_arr_of("wcost", job["tag"]), job)
    rec = None if ch is None else _route_rec(ch, wd, job["tag"])
    if rec is not None:
        rec["through_zone"] = thru
    return dict(kind="point", key=job["key"], lane=rec, seconds=time.time() - t0, routes=[])


def _rec_of(G, P, **kw):
    pr = R.curve_props(G, P)
    rec = dict(P=P, s=pr["s"], kappa=pr["kappa"], clr=pr["clr"], time_s=pr["t"], length=float(pr["s"][-1]),
               pinned=False, cut_start_m=0.0, cut_end_m=0.0, pivot=0.0, gear="none", fwd_free=False, rev_free=False,
               no_start_turn_data=True)
    rec.update(kw)
    return rec


def _f4(P):
    """Round to float32 as routes.bin stores it (read_check sees these values)."""
    return np.asarray(P, np.float32).astype(np.float64)


def _cells_of(P):
    r = np.clip(((P[:, 1] - PK._W["Z0"]) // 0.5).astype(np.int64), 0, PK._W["H5"] - 1)
    c = np.clip(((P[:, 0] - PK._W["X0"]) // 0.5).astype(np.int64), 0, PK._W["W5"] - 1)
    return r * PK._W["W5"] + c


def lane_stats(G, T, tag):
    P, s = T["P"], T["s"]
    o = np.asarray(_arr_of("open", tag).ravel()[_cells_of(P)]) > 0
    ds = np.diff(s)
    tc = G.tcost_at(P[:, 0], P[:, 1])
    kap = np.abs(np.asarray(T["kappa"]))
    Rr = np.where(kap > 1e-6, 1 / np.maximum(kap, 1e-6), np.inf)
    v = np.minimum(R.V_FREE / tc, np.sqrt(R.A_LAT * Rr))
    dt = ds / np.maximum(0.5 * (v[:-1] + v[1:]), 0.1)
    so = o[1:] & o[:-1]
    dl = []
    run = 0.0
    for k in range(len(ds)):
        if so[k]:
            if run == 0.0:
                dl.append([float(s[k]), 0.0, k, k])
            run += ds[k]
            dl[-1][1] = run
            dl[-1][3] = k + 1
        else:
            run = 0.0
    longest = max(dl, key=lambda d: d[1]) if dl else None
    where = None
    if longest is not None:
        a_, b_ = longest[2], longest[3]
        where = dict(from_m=round(longest[0], 1), to_m=round(longest[0] + longest[1], 1),
                     from_xz=[round(float(P[a_, 0]), 1), round(float(P[a_, 1]), 1)],
                     to_xz=[round(float(P[b_, 0]), 1), round(float(P[b_, 1]), 1)])
    long_dash = [d for d in dl if d[1] >= CONTACT_DASH_M]
    contact = round(float(long_dash[0][0]), 1) if long_dash else None
    return dict(time_s=round(float(T["time_s"]), 1), length_m=round(float(s[-1]), 1), open_m=round(float(ds[so].sum()), 1),
                open_s=round(float(dt[so].sum()), 1), longest_dash_m=round(float(longest[1]) if longest else 0.0, 1),
                longest_dash_at=where, dashes=len(dl), contact_m=contact,
                dash_list=[(round(d[0], 1), round(d[1], 1)) for d in dl], open_flags=so)


def beside_bounds(T, tag, contact):
    """Cover points of the lane's spawn area reached from the curve within BESIDE_S (one bounded multi-source
    Dijkstra on cost.npy from the curve's cells); at_m = arc of the curve point a cover point is reached from;
    from contact - 20 m on; per BOUND_STEP window of arc: turn room first, then the fewest seconds.
    -> (bounds, number reached within BESIDE_S, number from contact - 20 m on)."""
    if contact is None:
        return [], 0, 0
    nd = _nodes_of(tag)
    if not len(nd["cell"]):
        return [], 0, 0
    P, s = T["P"], T["s"]
    cells = _cells_of(P)
    uc, first = np.unique(cells, return_index=True)
    s_at = s[first]
    dist, par = PK._search(PK._W["cost"], uc, maxd=BESIDE_S)
    dn = dist.ravel()[nd["cell"]]
    near = np.nonzero(dn <= BESIDE_S)[0]
    lo = contact - BOUND_BEFORE_CONTACT
    found = []
    for k in near:
        src = int(PK._chain(par, int(nd["cell"][k]))[-1])
        j = int(np.searchsorted(uc, src))
        if j >= len(uc) or uc[j] != src:
            continue
        at = float(s_at[j])
        if at < lo:
            continue
        found.append(dict(x=round(float(nd["x"][k]), 2), z=round(float(nd["z"][k]), 2), face_deg=float(nd["face"][k]),
                          at_m=round(at, 1), off_s=round(float(dn[k]), 1), turn=bool(nd["turn"][k]), node=int(k)))
    best = {}
    for b in found:
        w = int((b["at_m"] - lo) // BOUND_STEP)
        key = (not b["turn"], b["off_s"])
        if w not in best or key < (not best[w]["turn"], best[w]["off_s"]):
            best[w] = b
    return sorted(best.values(), key=lambda b: b["at_m"]), len(near), len(found)


def job_trunk(job):
    """Realize one lane's driving curve (spawn pose -> first end slot) on its raw route; contact line and bounds."""
    G = PK._W["G"]
    t0 = time.time()
    cl = np.asarray(job["cells"], np.int64)
    s1 = int(job["slot1"])
    if int(cl[-1]) != s1:
        con = _connect(int(cl[-1]), s1)
        cl = np.concatenate([cl, con[1:]])
    keep = np.concatenate([[True], cl[1:] != cl[:-1]])
    cl = cl[keep]

    def checked(rec):
        if rec is None:
            return None
        Q = _f4(rec["P"])
        if R.bad_samples(G, Q)[0].any():
            Q, _ = R.repair(G, Q, 0, len(Q))
            Q = _f4(Q)
            if R.bad_samples(G, Q)[0].any():
                return None
        return _rec_of(G, Q, cut_start_m=rec.get("cut_start_m", 0.0), cut_end_m=rec.get("cut_end_m", 0.0))
    T = checked(R.finish(G, PK._xy(cl), pose=None))
    if T is None:
        return dict(kind="trunk", key=job["lane"], trunk=None, notes=[f"{job['lane']}: no driving curve (s25 finish rejected)"],
                    seconds=time.time() - t0, routes=[])
    st = lane_stats(G, T, job["tag"])
    bounds, n_near, n_after = beside_bounds(T, job["tag"], st["contact_m"])
    n_windows = len(bounds)
    bounds = thin_bounds(bounds, st["dash_list"])
    st.update(bounds=bounds, bounds_reached=n_near, bounds_after_contact=n_after, bounds_windows=n_windows,
              cut_start_m=T["cut_start_m"], cut_end_m=T["cut_end_m"])
    st["open_flags"] = st["open_flags"].copy()
    return dict(kind="trunk", key=job["lane"], trunk=dict(P=T["P"].astype(np.float32), **st), notes=[],
                seconds=time.time() - t0, routes=[])


def job_lane(job):
    """Pose routes along a realized lane curve: lead-ins for every pose of the area, other slots by lead-outs."""
    G = PK._W["G"]
    t0 = time.time()
    T = _rec_of(G, np.asarray(job["P"], np.float64))
    slots = job["slots"]
    notes = []
    base = dict(route_kind="spawn_post", target=slots[0]["id"], variant=0, lane=job["lane"])
    per_pose, n1 = PK.leadins(G, T, job["poses"], job["lane"], base)
    notes += n1
    out = list(per_pose)
    for e in slots[1:]:
        kp = {}
        O = PK.leadout(G, T, e, kp)
        for rp in per_pose:
            rec = PK.splice(G, rp, T, O) if O is not None else None
            if rec is None:
                p = next(p for p in job["poses"] if p["id"] == rp["pose"])
                d2, p2 = kp["dist"], kp["par"]
                if p["cell"] is None or d2.ravel()[p["cell"]] >= 1e19:
                    notes.append(f"{e['id']}: pose {p['id']} unreachable")
                    continue
                rec = R.finish(G, np.vstack([[p["x"], p["z"]], PK._xy(PK._chain(p2, p["cell"]))]), pose=p)
                notes.append(f"{e['id']}: pose {p['id']} by a direct route (no splice)")
                if rec is None:
                    notes.append(f"{e['id']}: pose {p['id']} rejected")
                    continue
                rec = PK._rec(rp, **rec)
            out.append(PK._rec(rec, target=e["id"]))
    # the file stores float32: check the stored values, repair the few that tip over
    fin = []
    for rec in out:
        Q = _f4(rec["P"])
        if R.bad_samples(G, Q)[0].any():
            Q, _ = R.repair(G, Q, 0, len(Q))
            Q = _f4(Q)
            if R.bad_samples(G, Q)[0].any():
                notes.append(f"{rec['target']}: pose {rec['pose']} dropped (float32 values touch)")
                continue
            pr = R.curve_props(G, Q)
            rec = PK._rec(rec, P=Q, s=pr["s"], kappa=pr["kappa"], clr=pr["clr"], time_s=pr["t"], length=float(pr["s"][-1]))
        fin.append(rec)
    return dict(kind="lane", key=job["lane"], routes=fin, notes=notes, seconds=time.time() - t0)


def work(job):
    t = job["type"]
    if t == "circle":
        return job_circle(job)
    if t == "point":
        return job_point(job)
    if t == "trunk":
        return job_trunk(job)
    if t == "lane":
        return job_lane(job)
    return PK.work(job)


def run_pool(cache, jobs, workers, label):
    out = []
    if not jobs:
        return out
    with mp.get_context("spawn").Pool(min(workers, len(jobs)), initializer=_init_routes, initargs=(str(cache),)) as pool:
        for k, r in enumerate(pool.imap_unordered(work, jobs)):
            out.append(r)
            if k % 50 == 0:
                log(f"  {label} job {k + 1}/{len(jobs)}")
    return out


# ================================================================ main side
def zone_inner(Fl, x, z, r):
    rr, cc = np.nonzero(Fl.valid05 & Fl.reach)
    m = np.hypot(Fl.X0 + (cc + 0.5) * 0.5 - x, Fl.Z0 + (rr + 0.5) * 0.5 - z) <= r
    return (rr[m] * Fl.W5 + cc[m]).astype(np.int64)


class ArmyView:
    """Army a's enemy positions (with times) and the earliest watch per cell, for one pass."""

    def __init__(self, ctx, a, pas, lane_ends=None):
        self.a, self.b, self.pas = a, 3 - a, pas
        S = ctx["S"]
        self.obs, self.times, self.first_zones = CV.observers(S, ctx["base"], a, ctx["poses"], ctx["zones"], ctx["corrs"],
                                                              ctx["T2"], lane_ends)
        self.E = CV.earliest(S, self.obs, self.times)
        self.Eall = CV.emin(self.E)
        self.obs_xy = S.xy(np.concatenate(list(self.obs.values())))
        self.ends = {}

    def add_end(self, **kw):
        e = dict(id=END_BASE + len(self.ends), **kw)
        self.ends[e["id"]] = e
        return e


class AreaGrid:
    """One spawn area in one pass: its open grid (depends on the area's travel times), the weighted search cost
    cost x (1 + OPEN_PEN x open) and (pass 2) its cover points; all written to the cache for the workers."""

    def __init__(self, ctx, AV, area, with_nodes):
        S, Fl = ctx["S"], ctx["Fl"]
        self.a, self.area = AV.a, area["id"]
        self.tag = f"a{AV.a}p{AV.pas}_{area['id']}"
        tA = ctx["base"]["cache"]["fields"][area["id"]]
        self.open2, self.open5, self.slack = CV.open_grids(S, AV.Eall, tA)
        valid2 = ctx["bclr"] >= C.REQ_CLR
        self.open_frac = float(self.open2[valid2].mean())
        np.save(ctx["cache"] / f"open_{self.tag}.npy", self.open5)
        np.save(ctx["cache"] / f"wcost_{self.tag}.npy",
                (ctx["cost"] * (1.0 + OPEN_PEN * self.open5.astype(np.float32))).astype(np.float32))
        self.nodes = CV.cover_points(Fl, S, self.open2, self.slack, ctx["bidx"], ctx["bclr"], ctx["bbox"], AV.obs_xy) \
            if with_nodes else []
        for k, n in enumerate(self.nodes):
            n["id"] = k
        np.savez(ctx["cache"] / f"nodes_{self.tag}.npz",
                 cell=np.array([n["cell"] for n in self.nodes], np.int64),
                 x=np.array([n["x"] for n in self.nodes], np.float64), z=np.array([n["z"] for n in self.nodes], np.float64),
                 face=np.array([n["face_deg"] for n in self.nodes], np.float64),
                 turn=np.array([n["turn"] for n in self.nodes], bool))
        cells = [p["cell"] for p in ctx["poses"] if p["area"] == area["id"] and p["cell"] is not None]
        self.spawn = dict(area=area["id"], cells=cells, x=area["centre"][0], z=area["centre"][1])


def support_positions(ctx, AV, L, k, x, z, r):
    """Up to SUPPORT_MAX fire-support positions for army AV.a onto zone L: never watched by the enemy's positions
    other than zone L itself, sees >= 25 % of the zone's points from the sight, r + 15 .. 400 m from the centre,
    own arrival no later than the enemy's + 15 s; one on the own side, one on each flank, then the next best of these
    (>= 80 m apart). Two slots each (10-30 m apart) when the ground gives them."""
    S = ctx["S"]
    a, b = AV.a, AV.b
    E_ex = CV.emin(AV.E, skip=(f"zone:{L}",))
    see = ctx["base"]["zin_exp"][k]
    d = np.hypot(S.XX - x, S.ZZ - z)
    bb = ctx["bbox"]
    T2 = ctx["T2"]
    cand = ((ctx["bclr"] >= C.REQ_CLR) & (S.XX >= bb[0]) & (S.XX <= bb[2]) & (S.ZZ >= bb[1]) & (S.ZZ <= bb[3])
            & (d >= r + SUPPORT_GAP) & (d <= SUPPORT_RMAX) & (see >= SUPPORT_SEE) & ~np.isfinite(E_ex)
            & np.isfinite(T2[a]) & (T2[a] <= np.where(np.isfinite(T2[b]), T2[b], np.inf) + FIRE_MARGIN_S))
    own = np.array(ctx["axes"][a]["centroid"]) - np.array([x, z])
    vx, vz = S.XX - x, S.ZZ - z
    th = np.degrees(np.arctan2(own[0] * vz - own[1] * vx, own[0] * vx + own[1] * vz))
    u = ctx["axes"][a]["axis"]
    lat = vx * u[0] + vz * u[1]
    sides = {"own": np.abs(th) <= OWN_SIDE_DEG,
             "left": (np.abs(th) > OWN_SIDE_DEG) & (np.abs(th) <= FLANK_DEG) & (lat < 0),
             "right": (np.abs(th) > OWN_SIDE_DEG) & (np.abs(th) <= FLANK_DEG) & (lat > 0)}
    turn = (ctx["bclr"] >= C.TURN_CLR).astype(np.float32)
    score = (turn, see, -d)
    picks = []
    for side in ("own", "left", "right"):
        c = sight30.pick_positions(score, cand & sides[side], 1, POS_SEP, S.XX, S.ZZ)
        if c:
            picks.append((side, c[0]))
    anyside = cand & (sides["own"] | sides["left"] | sides["right"])
    for (_, c) in picks:
        cx, cz = S.xy(c)
        anyside &= np.hypot(S.XX - cx, S.ZZ - cz) > POS_SEP
    extra = sight30.pick_positions(score, anyside, SUPPORT_MAX - len(picks), POS_SEP, S.XX, S.ZZ)
    picks += [("extra", c) for c in extra]
    out = []
    for side, c in picks[:SUPPORT_MAX]:
        slots = two_slots(ctx, c, cand & (sides["own"] | sides["left"] | sides["right"]), score, (x, z))
        out.append(dict(side=side, cell2=int(c), slots=slots, sees=round(float(see.ravel()[c]), 2), dist=round(float(d.ravel()[c]), 1)))
    return out


def two_slots(ctx, c, cand, score, face_xy, face_deg=None):
    S, Fl = ctx["S"], ctx["Fl"]
    x1, z1, c05 = CV.sub_xy(Fl, S, ctx["bidx"], c)
    fd = face_deg if face_deg is not None else round(ang(face_xy[0] - x1, face_xy[1] - z1), 1)
    slots = [dict(x=round(x1, 2), z=round(z1, 2), cell=c05, cell2=int(c), face_deg=fd)]
    cx, cz = S.xy(c)
    dd = np.hypot(S.XX - cx, S.ZZ - cz)
    m = cand & (dd >= SLOT2_R[0]) & (dd <= SLOT2_R[1])
    c2 = sight30.pick_positions(score, m, 1, 1.0, S.XX, S.ZZ)
    if c2:
        x2, z2, c05b = CV.sub_xy(Fl, S, ctx["bidx"], c2[0])
        if math.hypot(x2 - x1, z2 - z1) >= SLOT2_R[0]:
            fd2 = face_deg if face_deg is not None else round(ang(face_xy[0] - x2, face_xy[1] - z2), 1)
            slots.append(dict(x=round(x2, 2), z=round(z2, 2), cell=c05b, cell2=int(c2[0]), face_deg=fd2))
    return slots


def make_lane(g, end, rec, extra):
    return dict(army=g.a, **{"from": g.area}, tag=g.tag, end_id=end["id"], end=end["kind"], serves=end.get("zone"),
                cells=rec["cells"], raw=rec["raw"], raw_s=rec["raw_s"], raw_t=rec["raw_t"], graph_time_s=rec["graph_time_s"],
                graph_cost_s=rec["graph_cost_s"], graph_open_m=rec["graph_open_m"],
                through_zone=list(rec.get("through_zone", [])), **extra)


def trim_circle(lanes, budget):
    circ = [l for l in lanes if l["end"] == "circle"]
    if len(circ) <= budget:
        return lanes, []
    groups = defaultdict(list)
    for l in circ:
        groups[(l["from"], l["serves"])].append(l)
    order = []
    for g in groups.values():
        g.sort(key=lambda l: l["graph_time_s"])
        for k, l in enumerate(g[1:], 1):
            order.append((k, l["graph_time_s"] / g[0]["graph_time_s"], l))
    order.sort(key=lambda t: (-t[0], -t[1]))
    drop = [t[2] for t in order[:len(circ) - budget]]
    ids = {id(l) for l in drop}
    return [l for l in lanes if id(l) not in ids], [(l["from"], l["serves"], l["graph_time_s"]) for l in drop]


def thin_bounds(bounds, dashes, n=MAX_BOUNDS):
    """At most n bounds: the ones right before the longest dashes first, in order along the lane."""
    if len(bounds) <= n:
        return bounds
    keyed = []
    for b in bounds:
        nxt = [d for d in dashes if d[0] >= b["at_m"] - 1.0]
        follow = nxt[0][1] if nxt else 0.0
        last_before = bool(nxt) and all(not (b2["at_m"] > b["at_m"] and b2["at_m"] <= nxt[0][0] + 1.0) for b2 in bounds)
        keyed.append((1 if last_before else 0, follow, b))
    keyed.sort(key=lambda t: (-t[0], -t[1]))
    return sorted([t[2] for t in keyed[:n]], key=lambda b: b["at_m"])


def circle_slots(ctx, AV, lanes):
    """Two circle slots per circle lane: zone slots (8 m apart in r - 3.7) not yet used by this army first, then never
    watched by an enemy position, then the nearest to the lane's entry point."""
    for L, x, z, r in ctx["zones"]:
        zs = ctx["zslots"][L]
        used = [0] * len(zs)
        hid = [not np.isfinite(AV.Eall.ravel()[s["cell2"]]) for s in zs]
        mine = sorted([l for l in lanes if l["end"] == "circle" and l["serves"] == L], key=lambda l: l["graph_time_s"])
        for l in mine:
            ex, ez = l["raw"][-1]
            order = sorted(range(len(zs)), key=lambda i: (used[i], not hid[i], math.hypot(zs[i]["x"] - ex, zs[i]["z"] - ez)))
            l["slots"] = []
            for i in order[:LANE_CAP]:
                used[i] += 1
                s = zs[i]
                l["slots"].append(dict(x=round(s["x"], 2), z=round(s["z"], 2), cell=s["cell"], cell2=s["cell2"],
                                       face_deg=CV.face_to(s["x"], s["z"], AV.obs_xy[0], AV.obs_xy[1]), cover=bool(hid[i]),
                                       shared=used[i] > 1))


# ---------------------------------------------------------------- toll positions
def traffic_points(ctx, lanes_b, b):
    """Enemy traffic: points every TRAFFIC_STEP m along the raw route of each circle / support lane of army b (not
    within GUARD_SPAWN_SKIP of b's spawn areas); weight = seconds for those metres there (the route's own time).
    Points in the same 2 m sight cell add up. -> cells, weights, lane sets per cell, all point xy, point weights."""
    S = ctx["S"]
    skip = [(sp["x"], sp["z"]) for sp in ctx["spawn_xy"][b]]
    xs, zs, ws, li = [], [], [], []
    for k, l in enumerate(lanes_b):
        s, t, Q = l["raw_s"], l["raw_t"], l["raw"]
        if s[-1] <= 0:
            continue
        sv = np.arange(0.0, s[-1], TRAFFIC_STEP)
        se = np.minimum(sv + TRAFFIC_STEP, s[-1])
        xs.append(np.interp(sv, s, Q[:, 0]))
        zs.append(np.interp(sv, s, Q[:, 1]))
        ws.append(np.interp(se, s, t) - np.interp(sv, s, t))
        li.append(np.full(len(sv), k))
    if not xs:
        return np.zeros(0, np.int32), np.zeros(0), [], np.zeros((0, 2)), np.zeros(0)
    x, z, w, lid = np.concatenate(xs), np.concatenate(zs), np.concatenate(ws), np.concatenate(li)
    far = np.ones(len(x), bool)
    for (sx, sz) in skip:
        far &= np.hypot(x - sx, z - sz) > GUARD_SPAWN_SKIP
    x, z, w, lid = x[far], z[far], w[far], lid[far]
    cells = S.cell(x, z)
    qc, inv = np.unique(cells, return_inverse=True)
    wq = np.bincount(inv, weights=w, minlength=len(qc))
    lsets = [set() for _ in range(len(qc))]
    for i, k in zip(inv, lid):
        lsets[i].add(int(k))
    return qc.astype(np.int32), wq, lsets, np.stack([x, z], 1), w


def toll_value(S, qc, wq):
    """V(c) = sum of the weights of the traffic points seen from c (observer eye_h at c, hull target_h at the
    point; the heights guard_ends used) within TOLL_RANGE_M, by the sight30 model (S.accumulate)."""
    m = wq > 0
    if not m.any():
        return np.zeros((S.Hs, S.Ws), np.float64)
    q, w = qc[m], wq[m]
    groups = [(q[i], np.zeros(len(i), np.float32), w[i].astype(np.float32))
              for i in np.array_split(np.arange(len(q)), max(1, min(16, len(q)))) if len(i)]
    exp, _ = S.accumulate(groups, S.P["target_h"], S.P["eye_h"], range_m=TOLL_RANGE_M)
    return exp.sum(0, dtype=np.float64)


def _toll_slot(ctx, near, V, seen, face_xy):
    """Standing cell among `near`: hull-down towards the points seen first, then the largest V; else the best V."""
    S, Fl = ctx["S"], ctx["Fl"]
    idx = np.flatnonzero(near)
    if not len(idx):
        return None
    idx = idx[np.argsort(-V.ravel()[idx], kind="stable")]
    hd_pts = seen if len(seen) <= TOLL_HD_POINTS else seen[np.linspace(0, len(seen) - 1, TOLL_HD_POINTS).astype(int)]
    pick, hd, share = int(idx[0]), False, None
    for c in idx:
        sh = S.hull_down(int(c), hd_pts)
        if share is None:
            share = sh
        if sh >= TOLL_HULL_DOWN:
            pick, hd, share = int(c), True, sh
            break
    x, z, c05 = CV.sub_xy(Fl, S, ctx["bidx"], pick)
    return dict(x=round(x, 2), z=round(z, 2), cell=c05, cell2=pick, face_deg=round(ang(face_xy[0] - x, face_xy[1] - z), 1),
                hull_down=hd, hull_down_share=round(float(share), 2), value_s=round(float(V.ravel()[pick]), 1))


def standing_clear(ctx, route_lanes):
    """Per 2 m cell, at its standing sub-cell (best_sub, where slots are placed): the distance (m) to the nearest
    0.5 m cell of the raw routes given (circle / support lanes of both armies), and the mask of cells a position may
    stand on: farther than LANE_CLEAR_M from those routes and not within BRIDGE_NEAR_M of a bridge deck segment."""
    S, Fl = ctx["S"], ctx["Fl"]
    mark = np.zeros((Fl.H5, Fl.W5), np.uint8)
    for l in route_lanes:
        mark.ravel()[np.asarray(l["cells"], np.int64)] = 1
    d05 = C.edt(Fl.lib, mark) * 0.5
    sub = ctx["bidx"]
    J, I = np.mgrid[0:S.Hs, 0:S.Ws]
    r05, c05 = J * 4 + sub // 4, I * 4 + sub % 4
    dist = d05[r05, c05].astype(np.float32)
    X, Z = Fl.X0 + (c05 + 0.5) * 0.5, Fl.Z0 + (r05 + 0.5) * 0.5
    on_bridge = np.zeros(dist.shape, bool)
    for b in ctx["bridges"]:
        a_, b_ = np.array(b["end_a"], float), np.array(b["end_b"], float)
        v = b_ - a_
        t = np.clip(((X - a_[0]) * v[0] + (Z - a_[1]) * v[1]) / max(float(v @ v), 1e-9), 0, 1)
        on_bridge |= np.hypot(X - (a_[0] + t * v[0]), Z - (a_[1] + t * v[1])) <= BRIDGE_NEAR_M
    return (dist > LANE_CLEAR_M) & ~on_bridge, dist, on_bridge


def toll_positions(ctx, AV, lanes_b, clear=None):
    """Army AV.a's positions that watch the enemy's traffic (circle / support routes of AV.b, pass 2): greedy by
    coverage, at most TOLL_MAX, value >= TOLL_MIN_S, >= POS_SEP apart. Candidates: a hull fits, in the battle area,
    AV.a there no later than AV.b, never watched by AV.b's spawn exits / zone points / lane ends, and (clear) off the
    circle / support routes of both armies and off bridge decks; the pick and both slots come from these cells."""
    S = ctx["S"]
    a, b = AV.a, AV.b
    qc, wq, lsets, pxy, pw = traffic_points(ctx, lanes_b, b)
    notes = [f"army {a}: {len(pxy)} enemy traffic points in {len(qc)} sight cells, {float(pw.sum()):.0f} s of driving, "
             f"from {len(lanes_b)} enemy circle / support lanes"]
    if not len(qc):
        return [], notes
    T2 = ctx["T2"]
    bb = ctx["bbox"]
    E_g = CV.emin(AV.E, skip=("early",))
    cand = ((ctx["bclr"] >= C.REQ_CLR) & (S.XX >= bb[0]) & (S.XX <= bb[2]) & (S.ZZ >= bb[1]) & (S.ZZ <= bb[3])
            & ~np.isfinite(E_g) & np.isfinite(T2[a]) & (T2[a] <= np.where(np.isfinite(T2[b]), T2[b], np.inf)))
    if clear is not None:
        cand &= clear
    wcur = wq.astype(np.float64).copy()
    V = toll_value(S, qc, wcur)
    qx, qz = S.xy(qc)
    out, excluded = [], np.zeros(cand.shape, bool)
    for _ in range(200):
        if len(out) >= TOLL_MAX:
            break
        m = cand & (V >= TOLL_MIN_S) & ~excluded
        for p in out:
            m &= np.hypot(S.XX - p["x2"], S.ZZ - p["z2"]) >= POS_SEP
        if not m.any():
            break
        idx = np.flatnonzero(m)
        c = int(idx[int(np.argmax(V.ravel()[idx]))])
        vis, rng = S.seen(c, qc)
        live = vis & (rng <= TOLL_RANGE_M) & (wcur > 0)
        if not live.any():
            # the sweep counted points the exact sight lines do not confirm: skip this cell
            excluded.ravel()[c] = True
            continue
        Vr = V.copy()
        seen = qc[live]
        mx, mz = float(np.mean(qx[live])), float(np.mean(qz[live]))
        cx, cz = S.xy(c)
        rcand = cand & (Vr >= TOLL_MIN_S)
        near = rcand & (np.hypot(S.XX - cx, S.ZZ - cz) <= TOLL_SLOT_R)
        s1 = _toll_slot(ctx, near, Vr, seen, (mx, mz))
        slots = [s1]
        sx1, sz1 = S.xy(s1["cell2"])
        dd = np.hypot(S.XX - sx1, S.ZZ - sz1)
        s2 = _toll_slot(ctx, rcand & (dd >= SLOT2_R[0]) & (dd <= SLOT2_R[1]), Vr, seen, (mx, mz))
        if s2 is not None and math.hypot(s2["x"] - s1["x"], s2["z"] - s1["z"]) >= SLOT2_R[0]:
            slots.append(s2)
        if ctx.get("route_dist") is not None:
            for s_ in slots:
                s_["route_clear_m"] = round(float(ctx["route_dist"].ravel()[s_["cell2"]]), 1)
        lanes_seen = set()
        for i in np.nonzero(live)[0]:
            lanes_seen |= lsets[i]
        out.append(dict(cell2=c, x2=float(cx), z2=float(cz), value=round(float(Vr.ravel()[c]), 1), slots=slots,
                        enemy_lanes_seen=len(lanes_seen), points_seen=int(live.sum()),
                        range_m=[round(float(rng[live].min())), round(float(rng[live].max()))],
                        seen_xy=np.stack([qx[live], qz[live]], 1), seen_s=round(float(wcur[live].sum()), 1)))
        # the points it sees no longer count for the next positions
        sub = toll_value(S, qc[live], wcur[live])
        V = np.maximum(V - sub, 0.0)
        wcur[live] = 0.0
    notes.append(f"army {a}: {len(out)} toll position(s), values " + ", ".join(f"{p['value']:.0f}" for p in out))
    return out, notes


def cap_sg_slots(lanes_a, a, notes):
    """Slots of non-circle lanes <= MAX_SG_SLOTS_ARMY: if more, position lanes are dropped, lanes from a slower spawn
    area first, then the lowest value."""
    def count():
        return sum(len(l["slots"]) for l in lanes_a if l["end"] != "circle")
    drops = []
    while count() > MAX_SG_SLOTS_ARMY:
        tl = [l for l in lanes_a if l.get("side") == "toll"]
        if not tl:
            break
        worst = min(tl, key=lambda l: (not l.get("slower_area", False), l["value"]))
        lanes_a = [l for l in lanes_a if l is not worst]
        drops.append((worst["from"], worst["toll"], worst["value"]))
        notes.append(f"army {a}: position lane {worst['from']}>P{worst['toll']} dropped (non-circle slots > {MAX_SG_SLOTS_ARMY})")
    return lanes_a, drops


def neighbours_and_sectors(ctx, lanes_a, a):
    ax = ctx["axes"][a]
    for l in lanes_a:
        ex = float(np.mean([s["x"] for s in l["slots"]]))
        ez = float(np.mean([s["z"] for s in l["slots"]]))
        l["sector"], l["proj"] = CV.sector_of(ax, ex, ez)
    for l in lanes_a:
        left = [m for m in lanes_a if m is not l and m["proj"] < l["proj"] - 1e-6]
        right = [m for m in lanes_a if m is not l and m["proj"] > l["proj"] + 1e-6]
        nb = []
        if left:
            nb.append(max(left, key=lambda m: m["proj"])["id"])
        if right:
            nb.append(min(right, key=lambda m: m["proj"])["id"])
        l["neighbours"] = nb


OPP_STEP, OPP_NEAR, OPP_ANGLE, OPP_MIN_M = 2.0, 6.0, 120.0, 20.0


def _samples2(P):
    P = np.asarray(P, np.float64)
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    t = np.arange(0, s[-1], OPP_STEP)
    Q = np.stack([np.interp(t, s, P[:, 0]), np.interp(t, s, P[:, 1])], 1)
    d = np.diff(Q, axis=0)
    d = np.vstack([d, d[-1:]]) if len(d) else np.zeros((len(Q), 2))
    h = np.arctan2(d[:, 1], d[:, 0])
    return Q, h


def opposes(lanes_a):
    """Two lanes of one army oppose when >= 20 m of one curve (samples every 2 m) lies within
    6 m of the other with driving directions more than 120 deg apart (counted on either lane). Sets l['opposes'],
    l['opp_segs'] (opposed stretches for the pictures) and returns the pairs."""
    S2 = {l["id"]: _samples2(l["stats"]["P"]) for l in lanes_a}
    for l in lanes_a:
        l["opposes"], l["opp_segs"] = [], []
    pairs = []
    cosmax = math.cos(math.radians(OPP_ANGLE))
    for i, l in enumerate(lanes_a):
        Qa, ha = S2[l["id"]]
        for m in lanes_a[i + 1:]:
            Qb, hb = S2[m["id"]]
            if len(Qa) < 2 or len(Qb) < 2:
                continue
            lo = np.maximum(Qa.min(0), Qb.min(0)) - OPP_NEAR
            hi = np.minimum(Qa.max(0), Qb.max(0)) + OPP_NEAR
            if (lo > hi).any():
                continue
            D = np.hypot(Qa[:, None, 0] - Qb[None, :, 0], Qa[:, None, 1] - Qb[None, :, 1])
            cd = np.cos(ha[:, None] - hb[None, :])
            M = (D <= OPP_NEAR) & (cd < cosmax)
            ma, mb = M.any(1), M.any(0)
            la, lb = ma.sum() * OPP_STEP, mb.sum() * OPP_STEP
            if max(la, lb) < OPP_MIN_M:
                continue
            sel = Qa[ma] if la >= lb else Qb[mb]
            mid = sel[len(sel) // 2]
            l["opposes"].append(m["id"])
            m["opposes"].append(l["id"])
            l["opp_segs"].append(Qa[ma])
            m["opp_segs"].append(Qb[mb])
            pairs.append(dict(lanes=[l["id"], m["id"]], areas=[l["from"], m["from"]], at=[round(float(mid[0]), 1), round(float(mid[1]), 1)],
                              length_m=round(float(max(la, lb)), 1)))
    return pairs


def worst_free(lanes_a):
    """Per spawn area: lane places (sum of capacities) and the places left usable in the worst case, when one lane
    of another spawn area of the army is in use and blocks every lane of this area that opposes it."""
    out = {}
    by = defaultdict(list)
    for l in lanes_a:
        by[l["from"]].append(l)
    for ar, ls in by.items():
        tot = sum(l["capacity"] for l in ls)
        worst, who = 0, None
        for m in lanes_a:
            if m["from"] == ar:
                continue
            blk = sum(l["capacity"] for l in ls if m["id"] in l["opposes"])
            if blk > worst:
                worst, who = blk, m["id"]
        out[ar] = dict(places=tot, worst_blocked=worst, by_lane=who, free_worst=tot - worst, below_8=(tot - worst) < 8)
    return out


def bridges_crossed(P, bridges):
    out = []
    for k, b in enumerate(bridges):
        a_, b_ = np.array(b["end_a"]), np.array(b["end_b"])
        v = b_ - a_
        t = np.clip(((P - a_) @ v) / max(float(v @ v), 1e-9), 0, 1)
        d = np.hypot(*(P - (a_ + t[:, None] * v)).T)
        if (d <= BRIDGE_NEAR_M).sum() >= 10 and np.ptp(t[d <= BRIDGE_NEAR_M]) > 0.5:
            out.append(f"bridge {k + 1} at ({b['x']:.0f}, {b['z']:.0f})")
    return out


def search_lanes(ctx, views, grids, workers, label):
    """Circle lanes (one job per (spawn area, zone)) and support lanes (one job per (spawn area, support end); the
    area with the least driving time wins) of both armies, in the worker pool."""
    jobs = []
    ends = {}
    for a in (1, 2):
        AV = views[a]
        for g in grids[a]:
            for e in AV.ends.values():
                ends[(a, e["id"])] = e
                if e["kind"] == "circle":
                    x, z, r = ctx["zc"][e["zone"]]
                    jobs.append(dict(type="circle", key=(a, g.area, e["id"]), tag=g.tag, src=g.spawn["cells"],
                                     dst=e["src"], sx=g.spawn["x"], sz=g.spawn["z"], zx=x, zz=z, r=r,
                                     close=[zz for zz in ctx["zones"] if zz[0] != e["zone"]]))
                elif e["kind"] == "support":
                    jobs.append(dict(type="point", key=(a, g.area, e["id"]), tag=g.tag, src=g.spawn["cells"], dst=e["src"],
                                     close=list(ctx["zones"])))
    jobs.sort(key=lambda j: j["type"] != "circle")
    res = run_pool(ctx["cache"], jobs, workers, label)
    gby = {(a, g.area): g for a in (1, 2) for g in grids[a]}
    lanes = {1: [], 2: []}
    tries = {}
    sup = defaultdict(list)
    for r in res:
        a, area, eid = r["key"]
        e = ends[(a, eid)]
        g = gby[(a, area)]
        if r["kind"] == "circle":
            tries[f"{area}>{e['zone']}"] = r["tries"]
            for rec in r["lanes"]:
                lanes[a].append(make_lane(g, e, rec, {}))
        elif r["lane"] is not None:
            sup[(a, eid)].append((g, r["lane"]))
    for (a, eid), lst in sup.items():
        e = ends[(a, eid)]
        g, rec = min(lst, key=lambda t: t[1]["graph_time_s"])
        lanes[a].append(make_lane(g, e, rec, dict(slots=e["slots"], side=e.get("side"))))
    for a in (1, 2):
        lanes[a].sort(key=lambda l: ({"circle": 0, "support": 1}[l["end"]], l["serves"] or "", l["from"], l["graph_time_s"]))
    return lanes, tries, sum(r["seconds"] for r in res)


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
    PK.flank_variants(corrs)
    timing["analysis_s"] = round(time.time() - t, 1)
    log(f"analysis done ({timing['analysis_s']} s)")
    t = time.time()
    hard = np.isin(Fl.cls, C.HARD_CODES)
    D = (C.edt(Fl.lib, hard) * C.CELL - C.HALF_DIAG).astype(np.float32)
    v_road = P["v_road"] or P["v_road_fallback"]
    v_off = P["v_off"] or P["v_off_fallback"]
    np.save(cache / "D.npy", D)
    np.save(cache / "hard.npy", hard)
    np.save(cache / "tcost.npy", Fl.tcost.astype(np.float32))
    cost = Fl.time_cost(v_road, v_off).astype(np.float32)
    np.save(cache / "cost.npy", cost)
    np.save(cache / "unit.npy", Fl.unitR)
    np.save(cache / "meta.npy", np.array([Fl.X0, Fl.Z0], np.float64))
    del D, hard
    timing["cache_s"] = round(time.time() - t, 1)
    # ---------------- context
    poses = PK.poses_for(mission, Fl, areas, poses30)
    pose_by_area = defaultdict(list)
    for p in poses:
        pose_by_area[p["area"]].append(p)
    S = base["S"]
    bidx, bclr = CV.best_sub(Fl, S)
    bbox = C.battle_area(mission)["bbox"]
    axes = CV.axes(Fl, poses, base, zones)
    T2 = {a: S.block_min(np.where(np.isfinite(base["cache"]["army"][a]), base["cache"]["army"][a], np.inf).astype(np.float32))
          for a in (1, 2)}
    zslots = {}
    for (L, x, z, r) in zones:
        zl = PK.zone_slots(Fl, dict(x=x, z=z, radius=r), None)
        for s in zl:
            s["cell"] = Fl.idx(s["x"], s["z"])
            s["cell2"] = int(S.cell(np.array([s["x"]]), np.array([s["z"]]))[0])
        zslots[L] = zl
    bridges = json.loads((od / "map_report.json").read_text(encoding="utf-8"))["scan_rules"]["bridge_decks"]
    ctx = dict(Fl=Fl, S=S, base=base, poses=poses, zones=zones, corrs=corrs, areas=areas, bidx=bidx, bclr=bclr, bbox=bbox,
               axes=axes, T2=T2, zslots=zslots, cache=cache, cost=cost, bridges=bridges, zc={L: (x, z, r) for L, x, z, r in zones},
               spawn_xy={a: [dict(x=ar["centre"][0], z=ar["centre"][1]) for ar in areas if ar["army"] == a] for a in (1, 2)})
    ck_path = od / "lanes_checkpoint.pkl"
    if "--resume" in sys.argv and ck_path.is_file():
        import pickle
        ck = pickle.loads(ck_path.read_bytes())
        (lanes_by_pass, grid_stats, toll_notes, trimmed, no_lane, final_views, final_grids, tolls, alt_tries, sg_drops, tck) = ck
        timing.update(tck)
        log("resumed from the lane checkpoint")
    else:
        # ---------------- two passes: enemy positions, open grids per spawn area, lanes
        lanes_by_pass, grid_stats, toll_notes, trimmed, no_lane = {}, {}, [], {}, {}
        final_views, final_grids, tolls, alt_tries, sg_drops = {}, {}, {1: [], 2: []}, {}, {}
        for pas in (1, 2):
            t = time.time()
            views, grids = {}, {}
            for a in (1, 2):
                ends_b = None
                if pas == 2:
                    ends_b = [(s["x"], s["z"]) for l in lanes_by_pass[1][3 - a] for s in l["slots"]]
                AV = ArmyView(ctx, a, pas, ends_b)
                for (L, x, z, r) in zones:
                    AV.add_end(kind="circle", zone=L, src=zone_inner(Fl, x, z, r - ZONE_HALF_HULL))
                for k, (L, x, z, r) in enumerate(zones):
                    for sp_ in support_positions(ctx, AV, L, k, x, z, r):
                        AV.add_end(kind="support", zone=L, src=[sp_["slots"][0]["cell"]], **sp_)
                views[a] = AV
                grids[a] = []
                for ar in [x for x in areas if x["army"] == a]:
                    te = time.time()
                    g = AreaGrid(ctx, AV, ar, with_nodes=(pas == 2))
                    grids[a].append(g)
                    grid_stats[g.tag] = dict(open_share_of_drivable=round(g.open_frac, 3), cover_points=len(g.nodes),
                                             seconds=round(time.time() - te, 1), observers={k: int(len(v)) for k, v in AV.obs.items()},
                                             first_zones=AV.first_zones,
                                             support_ends=sum(1 for e in AV.ends.values() if e["kind"] == "support"))
                    log(f"pass {pas} {g.tag}: open {g.open_frac:.1%} of drivable 2 m cells, {len(g.nodes)} cover points "
                        f"({time.time() - te:.0f} s)")
            timing[f"pass{pas}_grids_s"] = round(time.time() - t, 1)
            ts = time.time()
            lanes, tries, cpu = search_lanes(ctx, views, grids, workers, f"pass {pas} search")
            timing[f"pass{pas}_search_s"] = round(time.time() - ts, 1)
            timing[f"pass{pas}_search_cpu_s"] = round(cpu, 1)
            alt_tries[f"pass{pas}"] = tries
            for a in (1, 2):
                nsup = sum(1 for l in lanes[a] if l["end"] == "support")
                lanes[a], trimmed[f"army{a}_pass{pas}"] = trim_circle(lanes[a], MAX_LANES_PACK // 2 - nsup - GUARD_RESERVE)
                circle_slots(ctx, views[a], lanes[a])
            if pas == 2:
                # ---------------- toll positions and their lanes (both armies' circle / support lanes exist now)
                tt = time.time()
                gends = {}
                cs_all = [l for a in (1, 2) for l in lanes[a] if l["end"] in ("circle", "support")]
                clear, ctx["route_dist"], on_bridge = standing_clear(ctx, cs_all)
                toll_notes.append(f"standing cells: {int((ctx['route_dist'] <= LANE_CLEAR_M).sum())} 2 m cells within "
                                  f"{LANE_CLEAR_M:.0f} m of a circle / support route, {int(on_bridge.sum())} on bridge decks: excluded")
                for a in (1, 2):
                    AV = views[a]
                    pos, tn = toll_positions(ctx, AV, [l for l in lanes[3 - a] if l["end"] in ("circle", "support")], clear)
                    toll_notes += tn
                    tolls[a] = pos
                    gends[a] = []
                    for k, p in enumerate(pos):
                        p["index"] = k + 1
                        e = AV.add_end(kind="guard", zone=None, src=[p["slots"][0]["cell"]], side="toll", toll=k + 1,
                                       slots=p["slots"], value=p["value"])
                        gends[a].append(e)
                jobs = [dict(type="point", key=(a, g.area, e["id"]), tag=g.tag, src=g.spawn["cells"], dst=e["src"],
                             close=list(ctx["zones"]))
                        for a in (1, 2) for e in gends[a] for g in grids[a]]
                res = run_pool(cache, jobs, workers, "position lanes")
                gby = {(a, g.area): g for a in (1, 2) for g in grids[a]}
                got = defaultdict(list)
                for r in res:
                    a, area, eid = r["key"]
                    if r["lane"] is not None:
                        got[(a, eid)].append((gby[(a, area)], r["lane"]))
                for a in (1, 2):
                    for e in gends[a]:
                        lst = got.get((a, e["id"]), [])
                        p = tolls[a][e["toll"] - 1]
                        p["areas"] = {}
                        if not lst:
                            toll_notes.append(f"army {a}: no route to position P{e['toll']}")
                            continue
                        fast = min(r_["graph_time_s"] for _, r_ in lst)
                        for g, rec in sorted(lst, key=lambda t_: t_[1]["graph_time_s"]):
                            ok = rec["graph_time_s"] <= TOLL_AREA_TIME * fast
                            p["areas"][g.area] = dict(time_s=rec["graph_time_s"], lane=ok)
                            if not ok:
                                continue
                            lanes[a].append(make_lane(g, e, rec, dict(slots=copy.deepcopy(e["slots"]), side="toll", toll=e["toll"],
                                                                      value=e["value"],
                                                                      slower_area=rec["graph_time_s"] > fast)))
                    lanes[a], sg_drops[str(a)] = cap_sg_slots(lanes[a], a, toll_notes)
                # check only (no change): position lanes passing near the slots of the army's OTHER positions;
                # slot 1's distance to the nearest lane route of either army (lanes to the position itself left out)
                W5 = Fl.W5
                cxy = lambda l: np.stack([Fl.X0 + (l["cells"] % W5 + 0.5) * 0.5, Fl.Z0 + (l["cells"] // W5 + 0.5) * 0.5], 1)
                xy_of = {id(l): cxy(l) for a in (1, 2) for l in lanes[a]}
                for a in (1, 2):
                    for l in [l for l in lanes[a] if l.get("side") == "toll"]:
                        Q = xy_of[id(l)]
                        for p in tolls[a]:
                            if p["index"] == l["toll"]:
                                continue
                            for k, s_ in enumerate(p["slots"]):
                                dmin = float(np.hypot(Q[:, 0] - s_["x"], Q[:, 1] - s_["z"]).min())
                                if dmin <= LANE_CLEAR_M:
                                    toll_notes.append(f"army {a}: position lane {l['from']}>P{l['toll']} passes {dmin:.1f} m "
                                                      f"from slot {k + 1} of P{p['index']} (not changed)")
                    for p in tolls[a]:
                        s_ = p["slots"][0]
                        best_d, best_k = float("inf"), None
                        for b_ in (1, 2):
                            for l in lanes[b_]:
                                if b_ == a and l.get("side") == "toll" and l.get("toll") == p["index"]:
                                    continue
                                Q = xy_of[id(l)]
                                dd = float(np.hypot(Q[:, 0] - s_["x"], Q[:, 1] - s_["z"]).min())
                                if dd < best_d:
                                    best_d = dd
                                    best_k = f"army {b_} {l.get('side') if l.get('side') == 'toll' else l['end']} " \
                                             f"{l['from']}>{l['serves'] if l['serves'] else 'P' + str(l.get('toll'))}"
                        p["nearest_route_m"], p["nearest_route"] = round(best_d, 1), best_k
                timing["toll_s"] = round(time.time() - tt, 1)
                # ends without a lane (all kinds)
                for a in (1, 2):
                    have = {l["end_id"] for l in lanes[a]}
                    no_lane[str(a)] = [dict(end=e["kind"], serves=e.get("zone"), side=e.get("side"),
                                            at=e["slots"][0] if e.get("slots") else None)
                                       for e in views[a].ends.values() if e["id"] not in have and e["kind"] != "circle"]
                    for e in views[a].ends.values():
                        if e["kind"] == "circle":
                            for g in grids[a]:
                                if not any(l["end_id"] == e["id"] and l["from"] == g.area for l in lanes[a]):
                                    no_lane[str(a)].append(dict(end="circle", serves=e["zone"], **{"from": g.area}))
                final_views, final_grids = views, grids
            lanes_by_pass[pas] = lanes
            timing[f"pass{pas}_s"] = round(time.time() - t, 1)
            log(f"pass {pas}: lanes " + ", ".join(f"army {a} {dict(Counter(l['end'] for l in lanes[a]))}" for a in (1, 2)))
        import pickle
        from types import SimpleNamespace as NS
        final_views = {a: NS(obs=v.obs, obs_xy=v.obs_xy) for a, v in final_views.items()}
        final_grids = {a: [NS(area=g.area, open5=g.open5, nodes=g.nodes, open_frac=g.open_frac) for g in gs]
                       for a, gs in final_grids.items()}
        ck_path.write_bytes(pickle.dumps((lanes_by_pass, grid_stats, toll_notes, trimmed, no_lane, final_views, final_grids,
                                          tolls, alt_tries, sg_drops,
                                          {k: v for k, v in timing.items() if k.startswith(("pass", "toll"))})))
    lanes = lanes_by_pass[2]
    # ---------------- realize the lane curves (before any target exists)
    tmp = []
    for a in (1, 2):
        for k, l in enumerate(lanes[a]):
            l["tmp_id"] = f"{a}T{k}"
            tmp.append(dict(type="trunk", lane=l["tmp_id"], tag=l["tag"], cells=l["cells"], slot1=l["slots"][0]["cell"]))
    t = time.time()
    res = run_pool(cache, tmp, workers, "curve")
    timing["curves_s"] = round(time.time() - t, 1)
    trunks = {r["key"]: r["trunk"] for r in res}
    curve_notes = [n for r in res for n in r["notes"]]
    dropped = []
    for a in (1, 2):
        keep = []
        for l in lanes[a]:
            tr = trunks.get(l["tmp_id"])
            if tr is None:
                dropped.append((l["from"], l["end"], l["serves"], l.get("toll")))
                continue
            l["stats"] = tr
            keep.append(l)
        lanes[a] = keep
    # ---------------- ids, sectors, neighbours
    kind_order = {"circle": 0, "support": 1, "guard": 2}
    for a in (1, 2):
        lanes[a].sort(key=lambda l: (kind_order[l["end"]], l["serves"] or "", l.get("toll") or 0, l["from"], l["graph_time_s"]))
        for n, l in enumerate(lanes[a]):
            l["id"] = f"{a}L{n + 1:02d}"
            l["capacity"] = min(LANE_CAP, len(l["slots"]))
            l["bridges"] = bridges_crossed(l["stats"]["P"].astype(np.float64), bridges)
            Pc = l["stats"]["P"].astype(np.float64)
            od_ = [float(np.hypot(Pc[:, 0] - zx_, Pc[:, 1] - zz_).min() - zr_) for (L_, zx_, zz_, zr_) in zones
                   if not (l["end"] == "circle" and L_ == l["serves"])]
            l["min_dist_other_zone_m"] = round(min(od_), 1) if od_ else None   # curve to another zone's circle edge
        neighbours_and_sectors(ctx, lanes[a], a)
    lane_by_id = {l["id"]: l for a in (1, 2) for l in lanes[a]}
    # ---------------- targets (only realized lanes)
    targets = []
    for (L, x, z, r) in zones:
        ri = r - PK.INNER_MARGIN
        targets.append(dict(id=f"zone_{L}", zone=L, kind="zone", army=0, x=x, z=z, sx=x, sz=z, radius=r, inner_radius=ri,
                            inner=zone_inner(Fl, x, z, ri)))
    # the old posts (choke / second crew / overwatch) are not used where a lane pack exists; they are neither
    # computed nor written (no targets, no routes, no rank columns)
    posts = []
    no_room = []
    for p in posts:
        st = PK.standing_cell(Fl, p["x"], p["z"])
        t_ = dict(id=p["id"], kind="post", army=p["army"], post_kind=p["kind"], x=p["x"], z=p["z"], faces=p.get("faces"))
        if st is None:
            t_.update(standing=False, sx=float("nan"), sz=float("nan"))
            no_room.append(p["id"])
        else:
            t_.update(standing=True, sx=st["sx"], sz=st["sz"], cell=st["cell"], stand_rule=st["rule"], stand_clr=st["clearance"])
        targets.append(t_)
    for a in (1, 2):
        for l in lanes[a]:
            for k, s in enumerate(l["slots"]):
                sid = f"{l['id']}_{k + 1}"
                s["target_id"] = sid
                fx = s["x"] + 50 * math.cos(math.radians(s["face_deg"]))
                fz = s["z"] + 50 * math.sin(math.radians(s["face_deg"]))
                targets.append(dict(id=sid, kind="post", army=a, post_kind=None, x=s["x"], z=s["z"], sx=s["x"], sz=s["z"],
                                    faces=[fx, fz], standing=True, cell=s["cell"], lane=l["id"]))
    tindex = {t_["id"]: i for i, t_ in enumerate(targets)}
    log(f"{len(poses)} poses, {len(targets)} targets ({len(posts)} posts, {sum(1 for t_ in targets if t_.get('lane'))} lane slots)")
    # ---------------- route jobs
    jobs = []
    for a in (1, 2):
        for l in lanes[a]:
            jobs.append(dict(type="lane", lane=l["id"], P=l["stats"]["P"],
                             slots=[dict(id=s["target_id"], cell=s["cell"], x=s["x"], z=s["z"]) for s in l["slots"]],
                             poses=[PK._strip(p) for p in pose_by_area[l["from"]]]))
    for t_ in targets:
        if t_["kind"] != "post" or not t_.get("standing") or t_.get("lane"):
            continue
        ar = {x["id"]: [PK._strip(p) for p in pose_by_area[x["id"]]] for x in areas if x["army"] == t_["army"]}
        jobs.append(dict(type="target", target=t_["id"], cell=t_["cell"], areas=ar))
    lane_ends = [dict(kind="area", id=l["id"], slots=[dict(id=s["target_id"], cell=s["cell"], x=s["x"], z=s["z"]) for s in l["slots"]])
                 for a in (1, 2) for l in lanes[a]]
    zlist = [t_ for t_ in targets if t_["kind"] == "zone"]
    for i, zt_ in enumerate(zlist):
        ends = [dict(kind="zone", id=o["id"], inner=o["inner"].tolist()) for o in zlist[i + 1:]] + lane_ends
        for k in range(0, len(ends), PK.BETWEEN_CHUNK):
            jobs.append(dict(type="between", zone=zt_["id"], inner=zt_["inner"].tolist(), ends=ends[k:k + PK.BETWEEN_CHUNK]))
    jobs.sort(key=lambda j: {"between": 0, "lane": 1, "target": 2}[j["type"]])
    t = time.time()
    results = run_pool(cache, jobs, workers, "route")
    timing["routes_parallel_s"] = round(time.time() - t, 1)
    job_secs = Counter()
    for r in results + res:
        job_secs[r["kind"]] += r["seconds"]
    timing["cpu_s_by_job_kind"] = {k: round(v, 1) for k, v in job_secs.items()}
    log(f"routes done ({timing['routes_parallel_s']} s wall)")
    # ---------------- assemble
    routes, notes = [], list(curve_notes)
    for r in results:
        notes += r["notes"]
        for rec in r["routes"]:
            if rec.get("variant", 0) != 0:
                continue
            routes.append(rec)
    best_circle = {}
    for a in (1, 2):
        for l in lanes[a]:
            if l["end"] == "circle":
                k = (l["from"], l["serves"])
                if k not in best_circle or l["stats"]["time_s"] < best_circle[k]["stats"]["time_s"]:
                    best_circle[k] = l
    sz_routes = []
    for r in routes:
        if r["route_kind"] == "spawn_post" and r.get("lane"):
            l = lane_by_id[r["lane"]]
            if l["end"] == "circle" and best_circle.get((l["from"], l["serves"])) is l and r["target"] == l["slots"][0]["target_id"]:
                sz_routes.append(PK._rec(r, route_kind="spawn_zone", target=f"zone_{l['serves']}"))
    routes += sz_routes
    for r in routes:
        r["total_s"] = round(r["time_s"] + abs(r.get("pivot") or 0.0) / R.START_TURN_DEG_S, 2)
        r["time_s"] = round(r["time_s"], 2)
    by_pt = defaultdict(list)
    for r in routes:
        by_pt[(r.get("pose"), r.get("start_target"), r["target"])].append(r)
    for key, rs in by_pt.items():
        rs.sort(key=lambda r: r["total_s"])
        for k, r in enumerate(rs):
            r["rank"] = k
            r["fallback"] = k
            r["designated"] = k == 0
            r["variant"] = k
            r["id"] = (f"{r['pose']}>{r['target']}" if r["route_kind"].startswith("spawn") else f"{r['start_target']}<>{r['target']}")[:32]
    korder = {"spawn_zone": 0, "spawn_post": 1, "zone_zone": 2, "zone_post": 3}
    routes.sort(key=lambda r: (korder[r["route_kind"]], r["id"]))
    # slot targets every pose of the lane's area reaches (no orphans)
    reached = Counter(r["target"] for r in routes if r["route_kind"] == "spawn_post" and r.get("lane"))
    orphan = [t_["id"] for t_ in targets if t_.get("lane") and reached.get(t_["id"], 0) == 0]
    if orphan:
        osets = set(orphan)
        targets = [t_ for t_ in targets if t_["id"] not in osets]
        routes = [r for r in routes if r["target"] not in osets]
        for a in (1, 2):
            for l in lanes[a]:
                l["slots"] = [s for s in l["slots"] if s["target_id"] not in osets]
                l["capacity"] = min(LANE_CAP, len(l["slots"]))
            gone = {l["id"] for l in lanes[a] if not l["slots"]}
            lanes[a] = [l for l in lanes[a] if l["slots"]]
            for l in lanes[a]:
                l["neighbours"] = [n for n in l["neighbours"] if n not in gone]
            dropped += [(g, "no slot reached") for g in gone]
        tindex = {t_["id"]: i for i, t_ in enumerate(targets)}
    opp_pairs = {str(a): opposes(lanes[a]) for a in (1, 2)}
    opp_free = {str(a): worst_free(lanes[a]) for a in (1, 2)}
    zsl = {f"zone_{L}": [dict(x=s["x"], z=s["z"], clearance=s["clearance"]) for s in zslots[L]] for L, *_r in zones}
    for t_ in targets:
        t_.pop("inner", None)
    t = time.time()
    PK.write_bins(od, Fl, poses, targets, zsl, routes)
    timing["write_s"] = round(time.time() - t, 1)
    t = time.time()
    import build_core
    build_core.build_read_check()
    rc = subprocess.run([str(PK.READ_CHECK), str(od / "routes.bin"), str(od / "map.bin")], capture_output=True, text=True)
    timing["read_check_s"] = round(time.time() - t, 1)
    (od / "read_check.txt").write_text(rc.stdout + rc.stderr, encoding="utf-8")
    log("read_check:\n" + rc.stdout)
    # ---------------- plan pack
    pi = {p["id"]: i for i, p in enumerate(poses)}
    pack = dict(format="WTPLAN002", mission=mission)
    pack["armies"] = {str(a): {k: axes[a][k] for k in ("axis", "origin", "width_m", "sectors")} for a in (1, 2)}
    pack["spawn_areas"] = [dict(id=x["id"], army=x["army"], x=x["centre"][0], z=x["centre"][1],
                                poses=[pi[p["id"]] for p in poses if p["area"] == x["id"]]) for x in areas]
    pack["zones"] = [dict(id=L, target=tindex[f"zone_{L}"], x=x, z=z, r=r) for L, x, z, r in zones]
    pack["lanes"] = []
    for a in (1, 2):
        for l in lanes[a]:
            st = l["stats"]
            pack["lanes"].append({"id": l["id"], "army": a, "from": l["from"], "serves": l["serves"], "end": l["end"],
                                  "sector": l["sector"], "capacity": l["capacity"], "time_s": st["time_s"],
                                  "open_s": st["open_s"], "open_m": st["open_m"], "longest_dash_m": st["longest_dash_m"],
                                  "contact_m": st["contact_m"],
                                  "value": round(float(l["value"]), 1) if l.get("side") == "toll" else 0,
                                  "slots": [dict(target=tindex[s["target_id"]], x=s["x"], z=s["z"], face_deg=s["face_deg"])
                                            for s in l["slots"]],
                                  "bounds": [dict(x=b["x"], z=b["z"], face_deg=b["face_deg"], at_m=b["at_m"]) for b in st["bounds"]],
                                  "neighbours": l["neighbours"][:MAX_NEIGHBOURS], "opposes": sorted(l["opposes"])})
    pack["plans"] = plan35.plans(base, areas, zones, corrs)
    import closed35
    closed_rep = closed35.apply(pack, closed35.load(mission))
    plan_check = closed_rep["plan_check"]
    (od / f"{mission}.plan.json").write_text(json.dumps(pack, indent=1), encoding="utf-8")
    lim = dict(lanes=len(pack["lanes"]), max_slots=max((len(x["slots"]) for x in pack["lanes"]), default=0),
               max_bounds=max((len(x["bounds"]) for x in pack["lanes"]), default=0),
               bounds_before_thinning_max=max((l["stats"]["bounds_windows"] for a in (1, 2) for l in lanes[a]), default=0),
               max_neighbours=max((len(x["neighbours"]) for x in pack["lanes"]), default=0),
               max_opposes=max((len(x["opposes"]) for x in pack["lanes"]), default=0),
               sg_slots={str(a): sum(len(x["slots"]) for x in pack["lanes"] if x["army"] == a and x["end"] != "circle") for a in (1, 2)})
    lim["within_limits"] = bool(lim["lanes"] <= MAX_LANES_PACK and lim["max_slots"] <= MAX_SLOTS_LANE
                                and lim["max_bounds"] <= MAX_BOUNDS and lim["max_neighbours"] <= MAX_NEIGHBOURS
                                and lim["max_opposes"] <= MAX_OPPOSES
                                and all(v <= MAX_SG_SLOTS_ARMY for v in lim["sg_slots"].values()))
    log(f"game-side limits: {lim}")
    # ---------------- position records (summary + picture)
    pos_rec = {}
    for a in (1, 2):
        pos_rec[a] = []
        for p in tolls[a]:
            ls = [l for l in lanes[a] if l.get("side") == "toll" and l.get("toll") == p["index"]]
            pos_rec[a].append(dict(index=p["index"], value=p["value"], enemy_lanes_seen=p["enemy_lanes_seen"],
                                   points_seen=p["points_seen"], range_m=p["range_m"], seen_s=p["seen_s"],
                                   nearest_route_m=p.get("nearest_route_m"), nearest_route=p.get("nearest_route"),
                                   slots=[dict(x=s["x"], z=s["z"], face_deg=s["face_deg"], hull_down=s["hull_down"],
                                               hull_down_share=s["hull_down_share"], value_s=s["value_s"],
                                               route_clear_m=s.get("route_clear_m")) for s in p["slots"]],
                                   areas=p.get("areas", {}),
                                   lanes=[dict(id=l["id"], **{"from": l["from"]}, time_s=l["stats"]["time_s"]) for l in ls],
                                   seen_xy=p["seen_xy"]))
    # ---------------- pictures
    t = time.time()
    import render37
    trunk_by_id = {l["id"]: l["stats"] for a in (1, 2) for l in lanes[a]}
    covers = {a: final_grids[a] for a in (1, 2)}
    pics = render37.render(ctx, od, mission, lanes, covers, final_views, pack, trunk_by_id, pos_rec)
    timing["pictures_s"] = round(time.time() - t, 1)
    timing["total_s"] = round(time.time() - t_all, 1)
    per_sector = {str(a): Counter(l["sector"] for l in lanes[a]) for a in (1, 2)}
    tbl = []
    for a in (1, 2):
        for l in lanes[a]:
            st = l["stats"]
            tbl.append(dict(id=l["id"], **{"from": l["from"]}, serves=l["serves"], end=l["end"], sector=l["sector"],
                            capacity=l["capacity"], side=l.get("side"), value=round(float(l.get("value", 0.0)), 1),
                            time_s=st["time_s"], length_m=st["length_m"], open_m=st["open_m"], open_s=st["open_s"],
                            longest_dash_m=st["longest_dash_m"], longest_dash_at=st["longest_dash_at"],
                            contact_m=st["contact_m"],
                            contact_pct=(round(100.0 * st["contact_m"] / st["length_m"], 1)
                                         if st["contact_m"] is not None and st["length_m"] > 0 else None),
                            cut_start_m=st["cut_start_m"], cut_end_m=st["cut_end_m"],
                            bounds=len(st["bounds"]), bounds_reached=st["bounds_reached"],
                            bounds_after_contact=st["bounds_after_contact"], bounds_windows=st["bounds_windows"],
                            bound_off_s=[b["off_s"] for b in st["bounds"]], bound_at_m=[b["at_m"] for b in st["bounds"]],
                            graph_time_s=l["graph_time_s"], graph_cost_s=l["graph_cost_s"], graph_open_m=l["graph_open_m"],
                            bridges=l["bridges"], slots_cover=[s.get("cover") for s in l["slots"]],
                            slots_hull_down=[s.get("hull_down") for s in l["slots"]], opposes=sorted(l["opposes"]),
                            through_zone=l.get("through_zone", []),
                            min_dist_other_zone_m=l.get("min_dist_other_zone_m")))
    thru = [(t_["id"], t_["from"], t_["serves"] or t_["side"], t_["through_zone"]) for t_ in tbl if t_["through_zone"]]
    log(f"s40 zone rule: lanes through another zone's circle (r + {ZONE_CLOSE_HULL} m): {len(thru)}"
        + "".join(f"; {i} {f}>{s} through {z}" for i, f, s, z in thru))
    for a in (1, 2):
        for p in pos_rec[a]:
            p.pop("seen_xy", None)
    summary = dict(mission=mission, timing=timing, workers=workers, grids=grid_stats,
                   rule=dict(SETUP_S=CV.SETUP_S, MARGIN_S=CV.MARGIN_S, OPEN_PEN=OPEN_PEN, NODE_SEP=CV.NODE_SEP, LANE_CAP=LANE_CAP,
                             CIRCLE_MAX=CIRCLE_MAX, CIRCLE_TIME=CIRCLE_TIME, CIRCLE_TRIES=CIRCLE_TRIES, ALT_BAN_R=ALT_BAN_R,
                             ALT_KEEP_START=ALT_KEEP_START, ALT_KEEP_END=ALT_KEEP_END, OVERLAP_MAX=OVERLAP_MAX,
                             SUPPORT_MAX=SUPPORT_MAX, BESIDE_S=BESIDE_S, BOUND_STEP=BOUND_STEP, MAX_BOUNDS=MAX_BOUNDS,
                             CONTACT_DASH_M=CONTACT_DASH_M, LANE_CLEAR_M=LANE_CLEAR_M, BRIDGE_NEAR_M=BRIDGE_NEAR_M,
                             TRAFFIC_STEP=TRAFFIC_STEP, GUARD_SPAWN_SKIP=GUARD_SPAWN_SKIP, TOLL_RANGE_M=TOLL_RANGE_M,
                             TOLL_MIN_S=TOLL_MIN_S, TOLL_MAX=TOLL_MAX, TOLL_SLOT_R=TOLL_SLOT_R, TOLL_HULL_DOWN=TOLL_HULL_DOWN,
                             TOLL_HD_POINTS=TOLL_HD_POINTS, TOLL_AREA_TIME=TOLL_AREA_TIME, GUARD_RESERVE=GUARD_RESERVE,
                             POS_SEP=POS_SEP, ZONE_CLOSE_HULL=ZONE_CLOSE_HULL),
                   axes=axes, lanes_pass1={str(a): Counter(l["end"] for l in lanes_by_pass[1][a]) for a in (1, 2)},
                   lanes={str(a): Counter(l["end"] for l in lanes[a]) for a in (1, 2)},
                   lanes_by_side={str(a): Counter(l.get("side") or l["end"] for l in lanes[a]) for a in (1, 2)},
                   lanes_per_sector={a: {str(k): v[k] for k in range(CV.SECTORS)} for a, v in per_sector.items()},
                   lanes_per_area_zone={str(a): Counter(f"{l['from']}>{l['serves']}" for l in lanes[a] if l["end"] == "circle") for a in (1, 2)},
                   circle_alternative_tries=alt_tries, positions=pos_rec, position_slot_cap_drops=sg_drops, toll_notes=toll_notes,
                   opposing_pairs={a: dict(count=len(v), pairs=v) for a, v in opp_pairs.items()}, free_places_worst=opp_free,
                   closed_by_list=closed_rep, lane_table=tbl, limits=lim, trimmed_circle_alternatives=trimmed, no_lane=no_lane,
                   dropped_lanes=dropped, orphan_slot_targets=orphan, plan_check=plan_check,
                   poses=len(poses), targets=Counter(t_["kind"] for t_ in targets), lane_slot_targets=sum(1 for t_ in targets if t_.get("lane")),
                   posts=len(posts), posts_without_standing_room=no_room,
                   routes=Counter(r["route_kind"] for r in routes), designated=sum(1 for r in routes if r["designated"]),
                   notes=notes, files={p.name: p.stat().st_size for p in [od / "routes.bin", od / "map.bin", od / f"{mission}.plan.json"]},
                   pictures=pics, read_check=rc.stdout.strip().splitlines())
    (od / "lanes_summary.json").write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("timing", "lanes", "lanes_per_sector", "lanes_per_area_zone", "routes", "files",
                                              "dropped_lanes", "orphan_slot_targets", "plan_check", "no_lane", "free_places_worst")}, indent=1, default=str))
    print(f"notes: {len(notes)}", notes[:20])


if __name__ == "__main__":
    main()
