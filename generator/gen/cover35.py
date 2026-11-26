"""Cover (time dependent): the enemy's likely positions with the time the enemy can be there, the
earliest moment an enemy watches each 2 m cell, cover points, the open grid, catch discs, the armies' axes / sectors.

Sight: the sight30 model (straight 3-D lines from the observer's sight, eye_h above the ground, to the hull, target_h
above the ground; terrain and hard objects block, foliage after fol_depth, range_m at most).
Enemy positions (army a's enemy b): b's spawn exits (its poses), b's early positions (every
EARLY_STEP_S s of driving along all of b's analysis corridors), points of the zones b reaches first, and (pass 2) the ends
of b's pass-1 lanes. Each carries T_o = b's travel time to it (fastest of b's spawn areas) + SETUP_S.
Earliest watch E(c) = min T_o over the positions that see the cell (inf: never seen).
Open for a lane from spawn area A: a cell passed at time t (A's travel time to it, the time field of the area) is
open when E(c) <= t + MARGIN_S, i.e. an enemy can be watching it when we pass. Cover = not open (hidden from every
such position). Ground near our spawn crossed before any enemy can watch it is not open, bridges included.
Ends where tanks stay (circle slots, support / guard positions) are judged without time: E = inf (never watched)
from the positions that matter for them.
"""
import math

import numpy as np

import common30 as C
import sight30

SETUP_S = 10.0             # an enemy position counts from its travel time + 10 s (stop, turn, lay the gun)
MARGIN_S = 10.0            # ... and is counted against a cell we pass at t when that is <= t + 10 s
NODE_SEP = 20.0            # cover points thinned to one per 20 m
CATCH_R = 12.0             # a cover node's catch disc (its hidden cells within 12 m): where a dash ends
SPAWN_CATCH_R = 12.0
EARLY_STEP_S = 20.0        # early positions: every 20 s of driving along the enemy's corridors ...
EARLY_FROM_S = 15.0        # ... from 15 s after the start
ZONE_OBS = 9               # zone interior points per zone used as positions
SECTORS = 5
OCC_FACTOR = 1.5           # occupied width: cells both armies reach within 1.5 x the latest first arrival at a zone


def observers(S, base, a, poses, zones, corrs, T2, lane_ends=None):
    """-> obs {category: 2 m cells}, times {category: T_o per cell}, zones b reaches first. Zone points are split
    per zone (zone:A ...) so a support position can leave its own zone out."""
    b = 3 - a
    obs = {}
    obs["spawn"] = np.unique(S.cell(np.array([p["x"] for p in poses if p["army"] == b]),
                                    np.array([p["z"] for p in poses if p["army"] == b])))
    ex, ez = [], []
    for c in corrs:
        if c["army"] != b:
            continue
        sm = c["samples"]
        t = np.asarray(sm["t"])
        for tk in np.arange(EARLY_FROM_S, t[-1] if len(t) else 0, EARLY_STEP_S):
            k = int(np.searchsorted(t, tk))
            if k < len(t):
                ex.append(sm["x"][k])
                ez.append(sm["z"][k])
    if ex:
        obs["early"] = np.unique(S.cell(np.array(ex), np.array(ez)))
    arr = base["cache"]["arr"]
    first_zones = []
    for (L, x, z, r) in zones:
        fb, fa = arr[L][b]["first"], arr[L][a]["first"]
        if fb is not None and (fa is None or fb < fa):
            first_zones.append(L)
            obs[f"zone:{L}"] = np.unique(sight30.sample_cells(base["zin"][L], ZONE_OBS))
    if lane_ends is not None and len(lane_ends):
        obs["lane_ends"] = np.unique(S.cell(np.array([e[0] for e in lane_ends]), np.array([e[1] for e in lane_ends])))
    obs = {k: v.astype(np.int32) for k, v in obs.items() if len(v)}
    tb = T2[b].ravel()
    times = {k: (np.where(np.isfinite(tb[v]), tb[v], 0.0) + SETUP_S).astype(np.float32) for k, v in obs.items()}
    return obs, times, first_zones


def earliest(S, obs, times):
    """Per category: earliest T_o of its positions that see a hull at each 2 m cell (inf: none)."""
    P = S.P
    groups, cats = [], []
    for cat, cells in obs.items():
        k = max(1, min(16, len(cells) // 3))
        for idx in np.array_split(np.arange(len(cells)), k):
            if len(idx):
                groups.append((cells[idx], times[cat][idx], np.ones(len(idx), np.float32)))
                cats.append(cat)
    out = {cat: np.full((S.Hs, S.Ws), np.inf, np.float32) for cat in obs}
    for k0 in range(0, len(groups), 32):
        _, tf = S.accumulate(groups[k0:k0 + 32], P["eye_h"], P["target_h"])
        for k in range(tf.shape[0]):
            out[cats[k0 + k]] = np.minimum(out[cats[k0 + k]], np.where(tf[k] > 1e29, np.inf, tf[k]))
    return out


def emin(E, skip=()):
    g = None
    for c, v in E.items():
        if c in skip:
            continue
        g = v if g is None else np.minimum(g, v)
    return g


def best_sub(Fl, S):
    """Per 2 m cell: the 0.5 m sub-cell with the largest clearance (valid and reached), its clearance."""
    clr05 = np.where(Fl.valid05 & Fl.reach, Fl.clr05, 0)
    b = clr05.reshape(S.Hs, 4, S.Ws, 4).transpose(0, 2, 1, 3).reshape(S.Hs, S.Ws, 16)
    return b.argmax(-1), b.max(-1)


def sub_xy(Fl, S, bidx, cell):
    sub = int(bidx.ravel()[cell])
    j, i = divmod(int(cell), S.Ws)
    r05, c05 = j * 4 + sub // 4, i * 4 + sub % 4
    return float(Fl.X0 + (c05 + 0.5) * 0.5), float(Fl.Z0 + (r05 + 0.5) * 0.5), int(r05 * Fl.W5 + c05)


def open_grids(S, Eall, tA05):
    """Open for a spawn area: E <= t + MARGIN_S. tA05: the area's travel time field (0.5 m). -> (open 2 m bool,
    open 0.5 m uint8, slack 2 m = E - t)."""
    E05 = np.repeat(np.repeat(Eall, 4, 0), 4, 1)
    t05 = np.where(np.isfinite(tA05), tA05, np.inf)
    o05 = (E05 <= t05 + MARGIN_S)
    t2 = S.block_min(t05.astype(np.float32))
    slack = (Eall - t2).astype(np.float32)
    o2 = Eall <= t2 + MARGIN_S
    return o2, np.ascontiguousarray(o05.astype(np.uint8)), slack


def cover_points(Fl, S, open2, slack, bidx, bclr, bbox, obs_xy):
    """Cover points: 2 m cells a hull fits (best sub-cell clearance >= 2.3 m, reached) inside the battle area that are
    not open; turn room (>= 4.3 m) first, then the larger time slack (E - t); thinned to one per NODE_SEP."""
    ok = (bclr >= C.REQ_CLR) & ~open2
    ok &= (S.XX >= bbox[0]) & (S.XX <= bbox[2]) & (S.ZZ >= bbox[1]) & (S.ZZ <= bbox[3])
    turn = (bclr >= C.TURN_CLR).astype(np.float32)
    sl = np.minimum(np.where(np.isfinite(slack), slack, 300.0), 300.0)
    picked = sight30.pick_positions((turn, sl), ok, 10 ** 6, NODE_SEP, S.XX, S.ZZ)
    out = []
    ox, oz = obs_xy
    for c in picked:
        x, z, c05 = sub_xy(Fl, S, bidx, c)
        out.append(dict(cell2=int(c), x=round(x, 2), z=round(z, 2), cell=c05, slack=round(float(sl.ravel()[c]), 1),
                        turn=bool(turn.ravel()[c] > 0), face_deg=face_to(x, z, ox, oz)))
    return out


def face_to(x, z, ox, oz, rng=1000.0):
    """Facing towards the open side: the direction to the enemy's likely positions within range (weight 1 / d)."""
    d = np.hypot(ox - x, oz - z)
    m = (d <= rng) & (d > 1.0)
    if not m.any():
        m = d > 1.0
    w = 1.0 / d[m]
    vx, vz = float(((ox[m] - x) / d[m] * w).sum()), float(((oz[m] - z) / d[m] * w).sum())
    return round(math.degrees(math.atan2(vz, vx)), 1)


def catch_map(Fl, nodes, open5, spawn_nodes):
    """Catch discs: a cover node owns its hidden, valid cells within CATCH_R (nearest node wins); a spawn node owns
    the valid cells within SPAWN_CATCH_R of its poses. Returns int32 (H5, W5), -1 elsewhere."""
    H5, W5 = Fl.H5, Fl.W5
    cid = -np.ones((H5, W5), np.int32)
    best = np.full((H5, W5), np.inf, np.float32)
    valid = Fl.valid05 & Fl.reach
    k = int(math.ceil(CATCH_R / 0.5))
    for n in nodes:
        r0, c0 = divmod(n["cell"], W5)
        ra, rb, ca, cb = max(0, r0 - k), min(H5, r0 + k + 1), max(0, c0 - k), min(W5, c0 + k + 1)
        R, Cc = np.mgrid[ra:rb, ca:cb]
        d = np.hypot(R - r0, Cc - c0) * 0.5
        m = (d <= CATCH_R) & valid[ra:rb, ca:cb] & (open5[ra:rb, ca:cb] == 0) & (d < best[ra:rb, ca:cb])
        best[ra:rb, ca:cb][m] = d[m]
        cid[ra:rb, ca:cb][m] = n["id"]
    ks = int(math.ceil(SPAWN_CATCH_R / 0.5))
    for sn in spawn_nodes:
        for pc in sn["cells"]:
            r0, c0 = divmod(pc, W5)
            ra, rb, ca, cb = max(0, r0 - ks), min(H5, r0 + ks + 1), max(0, c0 - ks), min(W5, c0 + ks + 1)
            R, Cc = np.mgrid[ra:rb, ca:cb]
            d = np.hypot(R - r0, Cc - c0) * 0.5
            m = (d <= SPAWN_CATCH_R) & valid[ra:rb, ca:cb]
            cid[ra:rb, ca:cb][m] = sn["id"]
    return cid


def axes(Fl, poses, base, zones):
    """Per army: forward = from its spawn centroid to the enemy's; axis u = the army's right (x east, z north);
    occupied width = 2nd..98th percentile of the projections of the cells both armies reach within OCC_FACTOR x
    the latest first arrival at a zone; origin = the left edge on the line through the midpoint of the centroids."""
    cen = {a: np.array([np.mean([p["x"] for p in poses if p["army"] == a]), np.mean([p["z"] for p in poses if p["army"] == a])])
           for a in (1, 2)}
    arr = base["cache"]["arr"]
    tl = max(arr[L][k]["first"] for (L, *_r) in zones for k in (1, 2) if arr[L][k]["first"] is not None)
    A1, A2 = base["cache"]["army"][1], base["cache"]["army"][2]
    occ = np.isfinite(A1) & np.isfinite(A2) & (A1 <= OCC_FACTOR * tl) & (A2 <= OCC_FACTOR * tl)
    rr, cc = np.nonzero(occ[::4, ::4])
    xs, zs = Fl.X0 + (cc * 4 + 0.5) * 0.5, Fl.Z0 + (rr * 4 + 0.5) * 0.5
    mid = (cen[1] + cen[2]) / 2
    out = {}
    for a in (1, 2):
        f = cen[3 - a] - cen[a]
        f = f / np.linalg.norm(f)
        u = np.array([f[1], -f[0]])
        pr = (xs - mid[0]) * u[0] + (zs - mid[1]) * u[1]
        lo, hi = float(np.percentile(pr, 2)), float(np.percentile(pr, 98))
        org = mid + u * lo
        out[a] = dict(axis=[round(float(u[0]), 5), round(float(u[1]), 5)], origin=[round(float(org[0]), 1), round(float(org[1]), 1)],
                      width_m=round(hi - lo, 1), sectors=SECTORS, forward=[float(f[0]), float(f[1])],
                      centroid=[float(cen[a][0]), float(cen[a][1])], occupied_t_s=round(OCC_FACTOR * tl, 1))
    return out


def sector_of(ax, x, z):
    u, o = ax["axis"], ax["origin"]
    p = (x - o[0]) * u[0] + (z - o[1]) * u[1]
    return int(min(SECTORS - 1, max(0, math.floor(p / ax["width_m"] * SECTORS)))), p
