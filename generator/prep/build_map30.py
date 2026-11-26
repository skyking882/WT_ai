"""Step 0 base layers of one mission -> out/<mission>/map.npz, sight.npz, foliage.npz, map_report.json.

map.npz (0.25 m classes + 0.5 m terrain cost): merged-map rules, parameterised (area = the mission's battle
rectangle from ground_battle_areas.json, cut to the scan and aligned to its 2 m cells):
  hard_ri            class 5 of the object raster (build_ri_s30.py: immortal / unlisted / no destruction parameter)
  hard_building_part class 4 (hp but no physRes)
  hard_static_scan   scan cell solid (gt_scan_solid) with hit kind 2, whole 2 m cell
  unknown_ri_scan    scan cell solid, kind 4, no raster object within 0.5 m: hard, whole 2 m cell
  unknown_edge_scan  same, kind 0 (edge ray blocked)
  overhead           unknown_ri cell solid only by hit height code 7, no edge bit: free (awning rule)
  water / steep      gt_judge at every 0.5 m centre: deeper than 1 m (unless deck / scan hit), grade > 45 %
  medium / soft / drivable object: free
For a mission listed in common30.PREMERGED (empty) that prebuilt map.npz is read instead.

Without an obstacle scan (data/obstacles/<mission>.bin absent; prep/scan_area.py): the grid is the one the scan would
have, every scan cell is "not known", so there are no scan classes (4, 5, 6, 11), water is judged everywhere (no
suppression under hits), no bridge decks are found (scan_rules.bridge_decks = []), object heights are the collision top
where there is one, else low_obj_h; map_report.json "obstacle_scan" says so. Static scene objects that are not placed
instances (bridges, some structures) are then missing from the map.

sight.npz (2 m = the scan's own cells):
  G     terrain height at the cell centre (bilinear on the exact WTTER001 heights)
  objh  height of a hard object above the ground, on cells holding >= 4 of 64 hard-object 0.25 m sub-cells
        (hard_ri, building part, static / unknown scan): the larger of (a) the scan's downward-ray hit height of the
        cell (code x 0.5 m), else the highest hit of the 8 neighbours, else common30 low_obj_h (2.5 m), and (b) the
        collision top of any hard-class rendinst whose node box (+1 m) covers the cell. 0 elsewhere.
        (b) was added because the scan's hits on the town's two-floor houses read 5-7 m while their collision tops
        are 8-11 m.
  road  share of the cell covered by the level's Lnav polygons (the game's road navigation mesh), 0.5 m grid
foliage.npz: every bush / tree instance of the level in the area: x, z, kind, collision top height.
  bush = destruction class bush or 'bush' in the name; tree = name starting tree_ / burned_tree, not bush,
  collision taller than 3 m. Crowns are drawn later from common30.BASE (radius / bottom / top rules).
"""
import json
import math
import sys
from collections import Counter
import time

import numpy as np

import common30 as C


def area_of(mission, sx0, sz0, nx, nz):
    a = C.battle_area(mission)["bbox"]
    si0, sj0 = max(0, int(np.floor((a[0] - sx0) / 2))), max(0, int(np.floor((a[1] - sz0) / 2)))
    si1, sj1 = min(nx, int(np.ceil((a[2] - sx0) / 2))), min(nz, int(np.ceil((a[3] - sz0) / 2)))
    return si0, sj0, si1, sj1


def gt_judge(T, X0, Z0, H5, W5, S5):
    xs = X0 + (np.arange(W5) + 0.5) * 0.5
    zs = Z0 + (np.arange(H5) + 0.5) * 0.5
    XX, ZZ = np.meshgrid(xs.astype(np.float64), zs.astype(np.float64))
    ht = lambda x, zz: C.bilinear(T["h"], T["x0"], T["z0"], T["step"], x, zz)
    h = ht(XX, ZZ)
    e = C.GT_HALF_M
    hx0, hx1, hz0, hz1 = ht(XX - e, ZZ), ht(XX + e, ZZ), ht(XX, ZZ - e), ht(XX, ZZ + e)
    gx = np.where(np.isfinite(hx0) & np.isfinite(hx1), (hx1 - hx0) / (2 * e), 0.0)
    gz = np.where(np.isfinite(hz0) & np.isfinite(hz1), (hz1 - hz0) / (2 * e), 0.0)
    grade = np.hypot(gx, gz)
    deck = np.zeros(XX.shape, bool)
    if T["deck"] is not None:
        dk = T["deck"]
        for ox, oz in ((0, 0), (e, 0), (-e, 0), (0, e), (0, -e)):
            fx = (XX + ox - T["x0"]) / T["step"]
            fz = (ZZ + oz - T["z0"]) / T["step"]
            ii = np.clip(fx.astype(np.int64), 0, dk.shape[1] - 2)
            jj = np.clip(fz.astype(np.int64), 0, dk.shape[0] - 2)
            deck |= ((dk[jj, ii] | dk[jj, ii + 1] | dk[jj + 1, ii] | dk[jj + 1, ii + 1]) & 1) != 0
    water = T["water"]
    depth = np.where(((S5 & 0x0700) == 0), water - h, 0.0) if np.isfinite(water) else np.zeros_like(h)
    depth = np.where(np.isfinite(h), depth, 0.0)
    grade = np.where(np.isfinite(h), grade, 0.0)
    depth = np.where(deck, 0.0, depth)
    grade = np.where(deck, 0.0, grade)
    is_water = depth > C.GT_WADE_M
    is_steep = (grade > C.GT_SLOPE_HARD) & ~is_water
    steep_f = np.clip((grade - C.GT_SLOPE_SOFT) / (C.GT_SLOPE_HARD - C.GT_SLOPE_SOFT), 0, 1)
    tcost = (1 + C.GT_SLOPE_COST * steep_f + C.GT_WATER_COST * np.clip(depth / C.GT_WADE_M, 0, 1)).astype(np.float32)
    deep_raw = (water - h > C.GT_WADE_M) & np.isfinite(h) if np.isfinite(water) else np.zeros_like(is_water)
    return is_water, is_steep, tcost, grade, depth, int(deck.sum()), deep_raw


BRIDGE_MIN_CELLS = 25      # a deck: >= 100 m2 of rendinst hits over deep water
RAIL_TRIM = 0.75           # the outermost 0.75 m of each deck side (railings, kerbs) stays closed
EXT_MAX, EXT_STEP = 40.0, 0.5
OPEN_WATER_MAX = 6.0       # an end may cross at most 6 m of water whose scan cells have no hit at all
CLOSING = (C.WATER, C.STEEP, C.UNKNOWN_RI, C.UNKNOWN_EDGE)


JOIN_ANGLE, JOIN_LATERAL, JOIN_GAP = 10.0, 3.0, 30.0   # decks joined across a gap the 2 m scan missed


def bridge_lanes(cls, tcost, S, deep2, X0, Z0):
    """Bridge rule. Deck = 8-connected group (>= 25 cells of 2 m) of scan cells whose downward ray hit a
    rendinst (kind 4, any height) over deep water. Its long axis (principal axis of the cells) and width (5-95 %
    of the cells across the axis + one cell) give a lane: the deck width minus 0.75 m on each side.
    Join: two decks whose axes agree within 10 deg, that face each other along the axis with a lateral offset of
    at most 3 m and a gap of at most 30 m (the lane centre in the gap over water) are one bridge whose middle span the
    scan missed; the lane runs across the gap ('joined across a gap without scan hits').
    The lane continues at each outer end, in 0.5 m steps up to 40 m, until a full lane-wide cross-section holds no
    closing class (water, steep, scan-only hard) for 1 m: ground a hull can stand on. An end that would cross more
    than 6 m of water without any scan hit, or finds no ground within 40 m, is reported and NOT opened.
    Every 0.25 m cell of the lane that is water / steep / scan-only hard / free becomes 'drivable object'; hard raster
    objects on it stay. Returns (list of bridges, 0.25 m mask of the joined gap spans)."""
    H, W = cls.shape
    known = (S & 0x8000) != 0
    kind = (S >> 8) & 7
    cand = known & (kind == 4) & deep2
    comps = [c for c in _components(cand) if len(c) >= BRIDGE_MIN_CELLS]

    def cls_at(x, z):
        i = np.floor((x - X0) / 0.25).astype(np.int64)
        j = np.floor((z - Z0) / 0.25).astype(np.int64)
        ok = (i >= 0) & (i < W) & (j >= 0) & (j < H)
        v = np.full(np.shape(x), 255, np.uint8)
        v[ok] = cls[j[ok], i[ok]]
        return v

    decks = []
    for cells in comps:
        jj, ii = np.array(cells).T
        xs, zs = X0 + (ii + 0.5) * 2, Z0 + (jj + 0.5) * 2
        cx, cz = xs.mean(), zs.mean()
        w_, v_ = np.linalg.eigh(np.cov(np.stack([xs - cx, zs - cz])))
        u = v_[:, 1]
        p = np.array([-u[1], u[0]])
        s = (xs - cx) * u[0] + (zs - cz) * u[1]
        t = (xs - cx) * p[0] + (zs - cz) * p[1]
        width = float(np.percentile(t, 95) - np.percentile(t, 5) + 2.0)
        decks.append(dict(xs=xs, zs=zs, jj=jj, ii=ii, cx=cx, cz=cz, u=u, p=p, s0=s.min() - 1.0, s1=s.max() + 1.0,
                          tc=float(np.median(t)), width=width, hw=max(width / 2 - RAIL_TRIM, 0.0)))
    # ---- join decks facing each other across a gap
    parent = list(range(len(decks)))

    def root(i):
        while parent[i] != i:
            i = parent[i]
        return i
    joins = []
    cosmax = math.cos(math.radians(JOIN_ANGLE))
    for i in range(len(decks)):
        for j in range(i + 1, len(decks)):
            A, B = decks[i], decks[j]
            if abs(float(A["u"] @ B["u"])) < cosmax:
                continue
            sb = (B["xs"] - A["cx"]) * A["u"][0] + (B["zs"] - A["cz"]) * A["u"][1]
            tb = (B["xs"] - A["cx"]) * A["p"][0] + (B["zs"] - A["cz"]) * A["p"][1]
            lateral = abs(float(np.median(tb)) - A["tc"])
            if sb.min() - 1.0 > A["s1"]:
                g0, g1 = A["s1"], sb.min() - 1.0
            elif sb.max() + 1.0 < A["s0"]:
                g0, g1 = sb.max() + 1.0, A["s0"]
            else:
                continue
            gap = g1 - g0
            if lateral > JOIN_LATERAL or gap > JOIN_GAP:
                continue
            tm = (A["tc"] + float(np.median(tb))) / 2
            ss = np.arange(g0, g1, 0.5)
            mx, mz = A["cx"] + ss * A["u"][0] + tm * A["p"][0], A["cz"] + ss * A["u"][1] + tm * A["p"][1]
            if len(ss) and float(np.mean(cls_at(mx, mz) == C.WATER)) < 0.5:
                continue
            parent[root(j)] = root(i)
            joins.append((i, j, round(float(gap), 1), round(lateral, 1)))
    groups = {}
    for i in range(len(decks)):
        groups.setdefault(root(i), []).append(i)
    out = []
    plans = []
    gap_spans = []
    for members in groups.values():
        base = max(members, key=lambda k: decks[k]["s1"] - decks[k]["s0"])
        D = decks[base]
        cx, cz, u, p = D["cx"], D["cz"], D["u"], D["p"]
        xs = np.concatenate([decks[k]["xs"] for k in members])
        zs = np.concatenate([decks[k]["zs"] for k in members])
        jj = np.concatenate([decks[k]["jj"] for k in members])
        ii = np.concatenate([decks[k]["ii"] for k in members])
        spans = []
        tcs = []
        for k in members:
            sk = (decks[k]["xs"] - cx) * u[0] + (decks[k]["zs"] - cz) * u[1]
            tk = (decks[k]["xs"] - cx) * p[0] + (decks[k]["zs"] - cz) * p[1]
            spans.append((sk.min() - 1.0, sk.max() + 1.0))
            tcs.append(float(np.median(tk)))
        spans.sort()
        s0, s1 = spans[0][0], max(e for _, e in spans)
        tc = float(np.mean(tcs))
        hw = min(decks[k]["hw"] for k in members)
        width = min(decks[k]["width"] for k in members)
        gaps = []
        reach = spans[0][1]
        for a_, b_ in spans[1:]:
            if a_ > reach:
                gaps.append((reach, a_))
            reach = max(reach, b_)
        tt = tc + np.arange(-hw, hw + 1e-6, 0.25)
        ends = {}
        for side, sgn, sst in (("a", -1, s0), ("b", 1, s1)):
            closed = Counter()
            run = 0
            found = None
            open_water = 0.0
            reason = "no lane-wide ground within 40 m"
            for k in range(1, int(EXT_MAX / EXT_STEP) + 1):
                sv = sst + sgn * k * EXT_STEP
                px, pz = cx + sv * u[0] + tt * p[0], cz + sv * u[1] + tt * p[1]
                c = cls_at(px, pz)
                if (c == 255).any():
                    reason = "leaves the map area"
                    break
                mx, mz = cx + sv * u[0] + tc * p[0], cz + sv * u[1] + tc * p[1]
                si, sj = int((mx - X0) // 2), int((mz - Z0) // 2)
                hit = 0 <= sj < S.shape[0] and 0 <= si < S.shape[1] and ((S[sj, si] >> 8) & 7) != 0
                if cls_at(np.array([mx]), np.array([mz]))[0] == C.WATER and not hit:
                    open_water += EXT_STEP
                    if open_water > OPEN_WATER_MAX:
                        reason = f"more than {OPEN_WATER_MAX:.0f} m of open water without any scan hit (no deck)"
                        break
                else:
                    open_water = 0.0
                bad = np.isin(c, CLOSING)
                if bad.any():
                    run = 0
                    for q in c[bad]:
                        closed[C.CLASS_NAMES[int(q)]] += 1
                else:
                    run += 1
                    if run >= 2:
                        found = (k - 2) * EXT_STEP
                        break
            ends[side] = dict(ext_m=found, fail=None if found is not None else reason,
                              closed_by_m2={kk: round(v * EXT_STEP * 0.25, 1) for kk, v in closed.items()})
        ss = np.arange(s0, s1, 0.5)
        S_, T_ = np.meshgrid(ss, tt)
        c_in = cls_at(cx + S_ * u[0] + T_ * p[0], cz + S_ * u[1] + T_ * p[1])
        inside = {C.CLASS_NAMES[int(q)]: round(float(n) * 0.5 * 0.25, 1) for q, n in zip(*np.unique(c_in, return_counts=True)) if q != 255}
        a = s0 - (ends["a"]["ext_m"] or 0.0)
        b = s1 + (ends["b"]["ext_m"] or 0.0)
        plans.append((cx, cz, u, p, a, b, tc, hw))
        for g0, g1 in gaps:
            gap_spans.append((cx, cz, u, p, g0, g1, tc, hw))
        out.append(dict(x=round(float(xs.mean()), 1), z=round(float(zs.mean()), 1), cells_2m=int(len(xs)),
                        decks=len(members), joined=len(members) > 1,
                        joined_note="joined across a gap without scan hits" if len(members) > 1 else None,
                        gaps_m=[round(float(g1 - g0), 1) for g0, g1 in gaps],
                        axis_bearing=round(float(C.bearing(u[0], u[1])) % 180, 0), deck_length=round(float(s1 - s0), 1),
                        deck_width=round(width, 1), lane_width=round(2 * hw, 1),
                        hit_height_median_m=float(np.median((S[jj, ii] & 63) * 0.5)),
                        ends=ends, connected=ends["a"]["ext_m"] is not None and ends["b"]["ext_m"] is not None,
                        narrower_than_hull=bool(2 * hw < 2 * C.REQ_CLR),
                        lane_classes_before_m2=inside,
                        end_a=[round(float(cx + (a - 2) * u[0] + tc * p[0]), 1), round(float(cz + (a - 2) * u[1] + tc * p[1]), 1)],
                        end_b=[round(float(cx + (b + 2) * u[0] + tc * p[0]), 1), round(float(cz + (b + 2) * u[1] + tc * p[1]), 1)],
                        crop=[round(float(min(cx + a * u[0], cx + b * u[0]) - 30), 1), round(float(min(cz + a * u[1], cz + b * u[1]) - 30), 1),
                              round(float(max(cx + a * u[0], cx + b * u[0]) + 30), 1), round(float(max(cz + a * u[1], cz + b * u[1]) + 30), 1)]))
    out.sort(key=lambda b: (b["x"], b["z"]))
    lane05 = np.zeros(tcost.shape, bool)
    gapmask = np.zeros(cls.shape, bool)
    for (cx, cz, u, p, a, b, tc, hw), is_gap in [(q, False) for q in plans] + [(q, True) for q in gap_spans]:
        corners = [(cx + sv * u[0] + tv * p[0], cz + sv * u[1] + tv * p[1]) for sv in (a, b) for tv in (tc - hw, tc + hw)]
        xs_, zs_ = [q[0] for q in corners], [q[1] for q in corners]
        i0, i1 = max(0, int((min(xs_) - X0) / 0.25)), min(W - 1, int((max(xs_) - X0) / 0.25) + 1)
        j0, j1 = max(0, int((min(zs_) - Z0) / 0.25)), min(H - 1, int((max(zs_) - Z0) / 0.25) + 1)
        J, I = np.mgrid[j0:j1 + 1, i0:i1 + 1]
        X, Z = X0 + (I + 0.5) * 0.25, Z0 + (J + 0.5) * 0.25
        sv = (X - cx) * u[0] + (Z - cz) * u[1]
        tv = (X - cx) * p[0] + (Z - cz) * p[1]
        m = (sv >= a) & (sv <= b) & (np.abs(tv - tc) <= hw)
        if is_gap:
            gapmask[j0:j1 + 1, i0:i1 + 1] |= m
            continue
        sub = cls[j0:j1 + 1, i0:i1 + 1]
        sub[m & (np.isin(sub, CLOSING) | (sub == C.FREE))] = C.DRV_OBJ
        lane05[np.clip(J[m] // 2, 0, tcost.shape[0] - 1), np.clip(I[m] // 2, 0, tcost.shape[1] - 1)] = True
    tcost[lane05] = 1.0
    return out, gapmask, joins


def _components(cand):
    H, W = cand.shape
    lab = -np.ones(cand.shape, np.int32)
    comps = []
    for j0, i0 in zip(*np.nonzero(cand)):
        if lab[j0, i0] >= 0:
            continue
        lab[j0, i0] = len(comps)
        st, cells = [(j0, i0)], []
        while st:
            j, i = st.pop()
            cells.append((j, i))
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    a, b = j + dj, i + di
                    if 0 <= a < H and 0 <= b < W and cand[a, b] and lab[a, b] < 0:
                        lab[a, b] = len(comps)
                        st.append((a, b))
        comps.append(cells)
    return comps


def bridge_components(cand, deep2, min_share=0.5):
    """(not used) 8-connected components of scan-only hard 2 m cells over deep water."""
    H, W = cand.shape
    lab = -np.ones(cand.shape, np.int32)
    comps = []
    for j0, i0 in zip(*np.nonzero(cand)):
        if lab[j0, i0] >= 0:
            continue
        k = len(comps)
        lab[j0, i0] = k
        st, cells = [(j0, i0)], []
        while st:
            j, i = st.pop()
            cells.append((j, i))
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    a, b = j + dj, i + di
                    if 0 <= a < H and 0 <= b < W and cand[a, b] and lab[a, b] < 0:
                        lab[a, b] = k
                        st.append((a, b))
        comps.append(cells)
    mask = np.zeros(cand.shape, bool)
    out = []
    for cells in comps:
        jj, ii = np.array(cells).T
        share = float(deep2[jj, ii].mean())
        if share >= min_share and len(cells) >= 25:
            mask[jj, ii] = True
            out.append(dict(cells_2m=len(cells), deep_share=round(share, 2), j=jj, i=ii))
    return mask, out


def merged(mission, S, X0, Z0, H, W, T, si0, sj0):
    level = C.mission_level(mission)
    z = np.load(C.out_dir(mission) / "ri_v3" / f"{level}_ri_raster.npz")
    i0, j0 = si0 * 8, sj0 * 8
    Rv = z["grid"][j0:j0 + H, i0:i0 + W]
    known = (S & 0x8000) != 0
    kind = (S >> 8) & 7
    hgt = S & 63
    solid = known & (((S & 0x02C0) != 0) | (((S & 0x0700) != 0) & (hgt >= 1) & (hgt < 8)))
    static = solid & (kind == 2)
    nz_ = Rv > 0
    g = nz_.copy()
    for d in (1, 2):
        g[d:, :] |= nz_[:-d, :]
        g[:-d, :] |= nz_[d:, :]
    g2 = g.copy()
    for d in (1, 2):
        g2[:, d:] |= g[:, :-d]
        g2[:, :-d] |= g[:, d:]
    blk = g2.reshape(H // 8, 8, W // 8, 8).any(axis=(1, 3))
    expl = blk.copy()
    ex = np.zeros_like(blk)
    ex[:, :-1] = blk[:, 1:]
    expl |= ((S & 0x40) != 0) & ex
    ez = np.zeros_like(blk)
    ez[:-1, :] = blk[1:, :]
    expl |= ((S & 0x80) != 0) & ez
    unk_ri = solid & (kind == 4) & ~expl & ~static
    unk_edge = solid & (kind == 0) & ~expl & ~static
    overhead = unk_ri & ((S & 0xC0) == 0) & (hgt == C.OVERHEAD_HEIGHT_CODE)
    unk_ri = unk_ri & ~overhead
    S5 = np.repeat(np.repeat(S, 4, axis=0), 4, axis=1)
    is_water, is_steep, tcost, grade, depth, deck_n, deep_raw = gt_judge(T, X0, Z0, H // 2, W // 2, S5)
    up2 = lambda m: np.repeat(np.repeat(m, 2, axis=0), 2, axis=1)
    up8 = lambda m: np.repeat(np.repeat(m, 8, axis=0), 8, axis=1)
    up4 = lambda m: np.repeat(np.repeat(m, 4, axis=0), 4, axis=1)
    deep2 = deep_raw.reshape(H // 8, 4, W // 8, 4).mean(axis=(1, 3)) >= 0.5
    cls = np.zeros((H, W), np.uint8)
    cls[Rv == 1] = C.DRV_OBJ
    cls[up8(overhead)] = C.OVERHEAD
    cls[Rv == 2] = C.SOFT
    cls[Rv == 3] = C.MEDIUM
    cls[Rv == 4] = C.HARD_BLDG
    cls[up2(is_steep)] = C.STEEP
    cls[up2(is_water)] = C.WATER
    cls[up8(unk_edge)] = C.UNKNOWN_EDGE
    cls[up8(unk_ri)] = C.UNKNOWN_RI
    cls[up8(static)] = C.HARD_STATIC
    cls[Rv == 5] = C.HARD_RI
    cls_before = cls.copy()
    bl, gapmask, joins = bridge_lanes(cls, tcost, S, deep2, X0, Z0)
    import trench31
    trep, tmask = trench31.trench_lanes(mission, cls, X0, Z0)
    stats = dict(bridge_decks=bl, cls_before=cls_before, gapmask=gapmask, bridge_joins=joins, trench=trep, tmask=tmask,
                 scan_solid=int(solid.sum()), static=int(static.sum()), unknown_ri=int(unk_ri.sum()),
                 unknown_edge=int(unk_edge.sum()), overhead=int(overhead.sum()), deck_cells_05=deck_n,
                 water_05=int(is_water.sum()), steep_05=int(is_steep.sum()))
    return cls, tcost, grade.astype(np.float32), depth.astype(np.float32), stats


def foliage_instances(mission, X0, Z0, X1, Z1):
    sys.path.insert(0, str(C.S18))
    import build_ri_obstacles as bro
    codec = bro.cr.Codecs(str(bro.CDK))
    inst = bro.rigz_decode(codec, (bro.WT / "levels" / f"{C.mission_level(mission)}.bin").read_bytes())
    dmg = bro.parse_dmg(C.DMG_BLK)
    col = bro.Collision(codec)
    top_cache = {}
    rows = []
    for name, px, py, pz, ax, ay, az in inst:
        if not (X0 - 10 <= px <= X1 + 10 and Z0 - 10 <= pz <= Z1 + 10):
            continue
        cls, _ = bro.classify(dmg.get(name.lower()))
        low = name.lower()
        is_bush = cls == "bush" or "bush" in low
        is_tree = (low.startswith("tree_") or low.startswith("burned_tree") or cls == "tree") and not is_bush
        if not (is_bush or is_tree):
            continue
        if name not in top_cache:
            nodes = col.nodes(name)
            top = 0.0
            for nm, ntype, lo, hi, tm in nodes or []:
                for cy in (lo[1], hi[1]):
                    top = max(top, cy * tm[4] + tm[10])
            top_cache[name] = top
        sy = float(np.linalg.norm(ay))
        top = top_cache[name] * sy
        if is_tree and top < 3.0:
            continue
        rows.append((px, pz, 1 if is_tree else 0, top))
    return np.array(rows, np.float32).reshape(-1, 4)


def collision_tops(mission, X0, Z0, Hs, Ws, T):
    """Top height above the ground of every hard-class rendinst (immortal / unlisted / no destruction parameter /
    building part), from its collision node boxes, painted at 2 m on the node's oriented footprint (max)."""
    sys.path.insert(0, str(C.S18))
    import build_ri_obstacles as bro
    codec = bro.cr.Codecs(str(bro.CDK))
    inst = bro.rigz_decode(codec, (bro.WT / "levels" / f"{C.mission_level(mission)}.bin").read_bytes())
    dmg = bro.parse_dmg(C.DMG_BLK)
    col = bro.Collision(codec)
    out = np.zeros((Hs, Ws), np.float32)
    X1, Z1 = X0 + Ws * 2, Z0 + Hs * 2
    ncache = {}
    for name, px, py, pz, ax, ay, az in inst:
        if not (X0 - 30 <= px <= X1 + 30 and Z0 - 30 <= pz <= Z1 + 30):
            continue
        p = dmg.get(name.lower())
        cls, _ = bro.classify(p)
        if not (cls in ("immortal", "unlisted") or (cls == "unknown" and not (p and "physRes" in p))):
            continue
        if name not in ncache:
            ncache[name] = col.nodes(name) or []
        g = C.bilinear(T["h"], T["x0"], T["z0"], T["step"], px, pz)
        if not np.isfinite(g):
            continue
        for nm, ntype, lo, hi, tm in ncache[name]:
            pts = []
            for cx in (lo[0], hi[0]):
                for cy in (lo[1], hi[1]):
                    for cz in (lo[2], hi[2]):
                        m = (cx * tm[0] + cy * tm[3] + cz * tm[6] + tm[9], cx * tm[1] + cy * tm[4] + cz * tm[7] + tm[10],
                             cx * tm[2] + cy * tm[5] + cz * tm[8] + tm[11])
                        pts.append((px + m[0] * ax[0] + m[1] * ay[0] + m[2] * az[0],
                                    py + m[0] * ax[1] + m[1] * ay[1] + m[2] * az[1],
                                    pz + m[0] * ax[2] + m[1] * ay[2] + m[2] * az[2]))
            P = np.array(pts)
            top = float(P[:, 1].max() - g)
            if top < 1.0 or P[:, 1].min() - g > 2.6:
                continue
            ux = np.array([tm[0] * ax[0] + tm[1] * ay[0] + tm[2] * az[0], tm[0] * ax[2] + tm[1] * ay[2] + tm[2] * az[2]])
            yaw = math.atan2(ux[1], ux[0]) if np.hypot(*ux) > 1e-4 else 0.0
            c, s = math.cos(yaw), math.sin(yaw)
            us, vs = P[:, 0] * c + P[:, 2] * s, -P[:, 0] * s + P[:, 2] * c
            i0, i1 = int((P[:, 0].min() - X0) // 2), int((P[:, 0].max() - X0) // 2)
            j0, j1 = int((P[:, 2].min() - Z0) // 2), int((P[:, 2].max() - Z0) // 2)
            i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, Ws - 1), min(j1, Hs - 1)
            if i1 < i0 or j1 < j0:
                continue
            J, I = np.mgrid[j0:j1 + 1, i0:i1 + 1]
            cxs, czs = X0 + (I + 0.5) * 2, Z0 + (J + 0.5) * 2
            u, v = cxs * c + czs * s, -cxs * s + czs * c
            m_ = (u >= us.min() - 1) & (u <= us.max() + 1) & (v >= vs.min() - 1) & (v <= vs.max() + 1)
            sub = out[j0:j1 + 1, i0:i1 + 1]
            sub[m_] = np.maximum(sub[m_], top)
    return out


PAL = np.array([[235, 235, 225], [150, 210, 150], [230, 190, 120], [200, 30, 30], [120, 0, 120], [255, 0, 255],
                [255, 120, 0], [60, 110, 230], [140, 100, 60], [160, 0, 0], [0, 190, 190], [255, 255, 0],
                [255, 0, 150]], np.uint8)


def bridge_picture(before, after, X0, Z0, bridges, path, mission):
    """One crop per deck: classes before (left) and after (right) the bridge rule, 2 px per metre."""
    from PIL import Image, ImageDraw, ImageFont
    try:
        f, fb = ImageFont.truetype("arial.ttf", 18), ImageFont.truetype("arialbd.ttf", 20)
        ft = ImageFont.truetype("arialbd.ttf", 30)
    except OSError:
        f = fb = ft = ImageFont.load_default()
    H, W = before.shape
    tiles = []
    for n, b in enumerate(bridges):
        x0, z0, x1, z1 = b["crop"]
        i0, i1 = max(0, int((x0 - X0) / 0.25)), min(W, int((x1 - X0) / 0.25))
        j0, j1 = max(0, int((z0 - Z0) / 0.25)), min(H, int((z1 - Z0) / 0.25))
        pics = []
        for arr in (before, after):
            im = Image.fromarray(PAL[arr[j0:j1, i0:i1]][::-1][::2, ::2])
            sc = min(1.0, 420 / max(im.size))
            pics.append(im.resize((max(1, int(im.size[0] * sc)), max(1, int(im.size[1] * sc))), Image.NEAREST))
        w = pics[0].size[0] * 2 + 30
        h = pics[0].size[1] + 136
        t = Image.new("RGB", (max(w, 520), h), (255, 255, 255))
        d = ImageDraw.Draw(t)
        t.paste(pics[0], (0, 86))
        t.paste(pics[1], (pics[0].size[0] + 30, 86))
        if b["joined"]:
            d.text((4, 54), "JOINED ACROSS A GAP WITHOUT SCAN HITS: " + ", ".join(str(g) for g in b["gaps_m"]) + " m (pink)",
                   fill=(200, 0, 150), font=fb)
        e = b["ends"]
        d.text((4, 2), f"bridge {n + 1} at ({b['x']:.0f}, {b['z']:.0f}): deck {b['deck_length']:.0f} m x {b['deck_width']:.0f} m, "
                       f"lane {b['lane_width']:.1f} m", fill=(0, 0, 0), font=fb)
        d.text((4, 28), f"ends: a {'+' + str(e['a']['ext_m']) + ' m' if e['a']['ext_m'] is not None else 'NO GROUND'}, "
                        f"b {'+' + str(e['b']['ext_m']) + ' m' if e['b']['ext_m'] is not None else 'NO GROUND'}"
                        f" -> {'connected' if b['connected'] else 'NOT connected'}",
               fill=(0, 120, 0) if b["connected"] and not b["joined"] else ((200, 0, 150) if b["joined"] else (200, 0, 0)), font=f)
        d.text((4, h - 44), "before", fill=(0, 0, 0), font=f)
        d.text((pics[0].size[0] + 34, h - 44), "after", fill=(0, 0, 0), font=f)
        tiles.append(t)
    cols = 3
    tw = max(t.size[0] for t in tiles) if tiles else 400
    th = max(t.size[1] for t in tiles) if tiles else 100
    rows = (len(tiles) + cols - 1) // cols
    out = Image.new("RGB", (cols * (tw + 20) + 20, rows * (th + 20) + 270), (255, 255, 255))
    d = ImageDraw.Draw(out)
    d.text((14, 8), f"Bridges of {mission}: merged map before / after the deck-lane rule (one crop per bridge, north up)",
           fill=(0, 0, 0), font=ft)
    names = ["free", "bush/tree (soft)", "knockable", "hard object", "hard static", "scan-only hard", "scan-only edge",
             "deep water", "steep", "building part", "drivable (deck lane)", "overhead", "lane joined across a gap (no scan hit)"]
    for k, nm in enumerate(names):
        x, y = 14 + (k % 6) * 300, 60 + (k // 6) * 34
        d.rectangle([x, y, x + 26, y + 22], fill=tuple(int(v) for v in PAL[k]), outline=(0, 0, 0))
        d.text((x + 34, y), nm, fill=(0, 0, 0), font=f)
    d.text((14, 175), "deck = rendinst hits over deep water; lane = deck width - 0.75 m per side; ends continue until 1 m "
                      "of lane-wide ground (no water / steep / scan-only hard); 'NO GROUND' = not opened", fill=(0, 0, 0), font=f)
    for n, t in enumerate(tiles):
        out.paste(t, (20 + (n % cols) * (tw + 20), 225 + (n // cols) * (th + 20)))
    out.save(path)


def trench_picture(before, after, tmask, X0, Z0, trep, path, mission):
    """One crop per trench group: merged map before / after the trench-lane rule; trench footprint dark red, opened
    lane cells cyan, lane centres marked."""
    from PIL import Image, ImageDraw, ImageFont
    try:
        f, fb, ft = ImageFont.truetype("arial.ttf", 18), ImageFont.truetype("arialbd.ttf", 20), ImageFont.truetype("arialbd.ttf", 30)
    except OSError:
        f = fb = ft = ImageFont.load_default()
    H, W = before.shape
    tiles = []
    for g in trep["group_list"]:
        x0, z0, x1, z1 = g["box"]
        x0, z0, x1, z1 = x0 - 15, z0 - 15, x1 + 15, z1 + 15
        i0, i1 = max(0, int((x0 - X0) / 0.25)), min(W, int((x1 - X0) / 0.25))
        j0, j1 = max(0, int((z0 - Z0) / 0.25)), min(H, int((z1 - Z0) / 0.25))
        pics = []
        for arr in (before, after):
            rgb = PAL[np.minimum(arr[j0:j1, i0:i1], len(PAL) - 1)].copy()
            tm = tmask[j0:j1, i0:i1] & np.isin(arr[j0:j1, i0:i1], C.HARD_CODES)
            rgb[tm] = (110, 30, 30)
            im = Image.fromarray(rgb[::-1])
            sc = min(1.0, 380 / max(im.size))
            pics.append(im.resize((max(1, int(im.size[0] * sc)), max(1, int(im.size[1] * sc))), Image.NEAREST))
        lanes = [L for L in trep["lane_list"] if L["group"] == g["id"]]
        w = pics[0].size[0] * 2 + 30
        h = pics[0].size[1] + 110
        t = Image.new("RGB", (max(w, 560), h), (255, 255, 255))
        d = ImageDraw.Draw(t)
        t.paste(pics[0], (0, 60))
        t.paste(pics[1], (pics[0].size[0] + 30, 60))
        sc = pics[0].size[0] / max(1, (i1 - i0))
        for L in lanes:
            px = pics[0].size[0] + 30 + (L["x"] - X0 - i0 * 0.25) / 0.25 * sc
            py = 60 + (j1 * 0.25 - (L["z"] - Z0)) / 0.25 * sc
            d.ellipse([px - 6, py - 6, px + 6, py + 6], outline=(255, 0, 150), width=3)
        d.text((4, 2), f"trench group {g['id']} at ({g['x']:.0f}, {g['z']:.0f}): {g['cells'] * 0.0625:.0f} m2, "
                       f"{len(lanes)} lane(s)", fill=(0, 0, 0), font=fb)
        d.text((4, 28), ", ".join(p.replace('trench_', '') for p in g["pieces"])[:70], fill=(60, 60, 60), font=f)
        d.text((4, h - 44), "before", fill=(0, 0, 0), font=f)
        d.text((pics[0].size[0] + 34, h - 44), "after (lane cells cyan, centres pink)", fill=(0, 0, 0), font=f)
        tiles.append(t)
    if not tiles:
        return
    cols = 3
    tw = max(t.size[0] for t in tiles)
    th = max(t.size[1] for t in tiles)
    rows = (len(tiles) + cols - 1) // cols
    out = Image.new("RGB", (cols * (tw + 20) + 20, rows * (th + 20) + 150), (255, 255, 255))
    d = ImageDraw.Draw(out)
    d.text((14, 8), f"Trenches of {mission}: footprint hard (dark red), crossing lanes (cyan) where narrow and straight",
           fill=(0, 0, 0), font=ft)
    d.text((14, 56), f"{trep['trench_instances']} trench pieces in {trep['groups']} groups, {trep['narrow_stretches']} narrow "
                     f"stretches (width <= {trep['rule']['TRENCH_CROSS_MAX_W']} m), {trep['lanes']} lanes; lane = "
                     f"{2 * trep['rule']['LANE_HALF']:.1f} m wide band across the trench + {trep['rule']['RUN_UP']} m run-up "
                     f"each side, one per {trep['rule']['LANE_STEP']:.0f} m", fill=(0, 0, 0), font=f)
    for n, t in enumerate(tiles):
        out.paste(t, (20 + (n % cols) * (tw + 20), 110 + (n // cols) * (th + 20)))
    out.save(path)


def road_raster(mission, X0, Z0, H5, W5):
    p = C.LNAV_DIR / f"{C.mission_level(mission)}.bin"
    if not p.is_file() or p.stat().st_size == 0:
        return None, "no Lnav file for the level"
    sys.path.insert(0, str(C.S18))          # lnav_extract.py lives in prep/ (C.S18 = prep)
    import lnav_extract as lx
    from PIL import Image, ImageDraw
    d = lx.parse(p.read_bytes())
    V = np.array(d["verts"], np.float64)
    img = Image.new("L", (W5, H5), 0)
    dr = ImageDraw.Draw(img)
    for vs, _ in d["polys"]:
        if len(vs) >= 3:
            dr.polygon([((V[q][0] - X0) / 0.5 - 0.5, (V[q][1] - Z0) / 0.5 - 0.5) for q in vs], fill=1)
    return np.array(img, np.uint8), f"{len(d['polys'])} Lnav polygons"


def main(mission):
    t0 = time.time()
    od = C.out_dir(mission)
    import scan_area
    has_scan = scan_area.has_scan(mission)
    if has_scan:
        S_full, sx0, sz0, sstep = C.read_scan(mission)
    else:
        # no obstacle scan: the grid the scan would have, every cell "not known" (no scan classes, no bridge decks)
        S_full, sx0, sz0, sstep, _ = scan_area.cells(mission)
    T = C.read_terrain(mission)
    rep = dict(mission=mission, level=C.mission_level(mission), water_level=float(T["water"]) if np.isfinite(T["water"]) else None,
               water_note=T["water_note"])
    if not has_scan:
        rep["obstacle_scan"] = scan_area.NO_SCAN_NOTE
    if mission in C.PREMERGED:
        z = np.load(C.PREMERGED[mission])
        cls, tcost = z["cls"], z["tcost"]
        X0, Z0 = float(z["X0"]), float(z["Z0"])
        H, W = cls.shape
        si0, sj0 = int(round((X0 - sx0) / 2)), int(round((Z0 - sz0) / 2))
        S = S_full[sj0:sj0 + H // 8, si0:si0 + W // 8]
        rep["merged_map"] = f"read from {C.PREMERGED[mission]} (s25, same rules)"
        cls_before = None
        np.savez_compressed(od / "map.npz", cls=cls, tcost=tcost, X0=X0, Z0=Z0, cell=C.CELL)
    else:
        si0, sj0, si1, sj1 = area_of(mission, sx0, sz0, S_full.shape[1], S_full.shape[0])
        S = S_full[sj0:sj1, si0:si1]
        X0, Z0 = sx0 + si0 * 2.0, sz0 + sj0 * 2.0
        H, W = S.shape[0] * 8, S.shape[1] * 8
        cls, tcost, grade, depth, st = merged(mission, S, X0, Z0, H, W, T, si0, sj0)
        cls_before = st.pop("cls_before")
        gapmask = st.pop("gapmask")
        tmask = st.pop("tmask")
        rep["merged_map"] = "built (s25 rules)"
        rep["scan_rules"] = st
        np.savez_compressed(od / "map.npz", cls=cls, tcost=tcost, X0=X0, Z0=Z0, cell=C.CELL)
    rep["area"] = dict(x0=X0, z0=Z0, x1=X0 + W * C.CELL, z1=Z0 + H * C.CELL, cols=W, rows=H)
    counts = {C.CLASS_NAMES[k]: int(v) for k, v in zip(*np.unique(cls, return_counts=True))}
    rep["cells_by_class_025"] = counts
    # ---- sight layers (2 m)
    Hs, Ws = H // 8, W // 8
    xs = X0 + (np.arange(Ws) + 0.5) * 2.0
    zs = Z0 + (np.arange(Hs) + 0.5) * 2.0
    XX, ZZ = np.meshgrid(xs, zs)
    G = C.bilinear(T["h"], T["x0"], T["z0"], T["step"], XX, ZZ).astype(np.float32)
    G = np.where(np.isfinite(G), G, np.nanmin(G)).astype(np.float32)
    objc = np.isin(cls, C.OBJ_HARD).reshape(Hs, 8, Ws, 8).sum(axis=(1, 3))
    obj = objc >= 4
    hit = np.where(((S & 0x8000) != 0) & (((S >> 8) & 7) != 0), (S & 63) * 0.5, 0.0).astype(np.float32)
    nb = hit.copy()
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            nb = np.maximum(nb, np.roll(np.roll(hit, dj, 0), di, 1))
    src = np.where(hit > 0, 1, np.where(nb > 0, 2, 3)).astype(np.uint8)
    objh = np.where(obj, np.where(hit > 0, hit, np.where(nb > 0, nb, C.BASE["low_obj_h"])), 0.0).astype(np.float32)
    ctop = collision_tops(mission, X0, Z0, Hs, Ws, T)
    # with a scan: the collision top where it is higher than the scan-based height; without a scan there are no hit
    # heights: the collision top where there is one, else the assumed height
    raised = obj & (ctop > objh) if has_scan else obj & (ctop > 0)
    src = np.where(raised, 4, src).astype(np.uint8)
    objh = np.where(raised, ctop, objh).astype(np.float32)
    rep["sight"] = dict(cells_2m=int(Hs * Ws), object_cells=int(obj.sum()),
                        object_height_from_own_hit=int((obj & (src == 1)).sum()),
                        object_height_from_neighbour_hit=int((obj & (src == 2)).sum()),
                        object_height_assumed=int((obj & (src == 3)).sum()),
                        object_height_from_collision_top=int((obj & (src == 4)).sum()),
                        object_height_pct=dict(zip(("p10", "p50", "p90", "max"),
                                                   [float(v) for v in np.percentile(objh[obj], [10, 50, 90, 100])])) if obj.any() else None)
    road, rnote = road_raster(mission, X0, Z0, H // 2, W // 2)
    rep["road"] = rnote + (f", {road.mean() * 100:.1f}% of the area" if road is not None else "")
    if cls_before is not None:
        after = cls.copy()
        after[gapmask & (cls == C.DRV_OBJ)] = 12
        bridge_picture(cls_before, after, X0, Z0, rep["scan_rules"]["bridge_decks"], od / "bridges_before_after.png", mission)
        trench_picture(cls_before, cls, tmask, X0, Z0, rep["scan_rules"]["trench"], od / "trenches_lanes.png", mission)
    np.savez_compressed(od / "sight.npz", G=G, objh=objh, objsrc=src, X0=X0, Z0=Z0, cell=2.0,
                        road=road if road is not None else np.zeros((1, 1), np.uint8), has_road=road is not None)
    fol = foliage_instances(mission, X0, Z0, X0 + W * C.CELL, Z0 + H * C.CELL)
    np.savez_compressed(od / "foliage.npz", inst=fol)
    rep["foliage"] = dict(bushes=int((fol[:, 2] == 0).sum()), trees=int((fol[:, 2] == 1).sum()),
                          tree_top_p50=float(np.median(fol[fol[:, 2] == 1, 3])) if (fol[:, 2] == 1).any() else None)
    rep["seconds"] = round(time.time() - t0, 1)
    (od / "map_report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
