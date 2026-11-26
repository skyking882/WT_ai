"""Step 3: sight model on the 2 m grid and its uses (interdiction, fire support, corridor exposure, delay).

Sight line (s30_vis.c): straight 3-D line between an observer's sight (eye_h above ground) and a target point
(target_h above ground); blocked by terrain and by hard objects of height objh (scan hit heights, see build_map30);
bush / tree crowns block once the line has run fol_depth metres through them; at most range_m long.
"""
import math

import numpy as np

import common30 as C

DELAY_R = 400.0      # delay positions: within 400 m of the defended zone
FALLBACK_R = 250.0   # fall-back positions: within 250 m of the zone
FRAC_SEE = 0.25      # "sees the zone" = sees at least a quarter of its interior points
ZONE_STEP = 6.0      # zone interior points every 6 m


class Sight:
    def __init__(self, Fl, P):
        self.Fl, self.P, self.lib = Fl, P, Fl.lib
        s = np.load(Fl.od / "sight.npz")
        self.G = np.ascontiguousarray(s["G"], np.float32)
        self.Hs, self.Ws = self.G.shape
        assert self.Hs * 4 == Fl.H5 and self.Ws * 4 == Fl.W5
        self.X0, self.Z0 = Fl.X0, Fl.Z0
        self.objh_raw = s["objh"]
        objh = np.where(self.objh_raw > 0, self.objh_raw * P["house_scale"] + P["house_add"], 0.0)
        self.T = np.ascontiguousarray(self.G + objh, np.float32)
        fol = np.load(Fl.od / "foliage.npz")["inst"]
        FBr = np.full(self.G.shape, 1e9, np.float32)
        FTr = np.full(self.G.shape, -1e9, np.float32)
        for x, z, kind, top in fol:
            if kind == 0:
                r, bot, tp = P["bush_r"], 0.0, P["bush_top"]
            else:
                r, bot, tp = P["tree_r_frac"] * top, P["tree_bot_frac"] * top, top
            k = int(math.ceil((r + 1.0) / 2.0))
            ci, cj = int((x - self.X0) // 2), int((z - self.Z0) // 2)
            for dj in range(-k, k + 1):
                for di in range(-k, k + 1):
                    i, j = ci + di, cj + dj
                    if 0 <= i < self.Ws and 0 <= j < self.Hs:
                        cx, cz = self.X0 + (i + 0.5) * 2, self.Z0 + (j + 0.5) * 2
                        if math.hypot(cx - x, cz - z) <= r + 1.0:
                            FBr[j, i] = min(FBr[j, i], bot)
                            FTr[j, i] = max(FTr[j, i], tp)
        self.fol = FTr > FBr
        self.FB = np.ascontiguousarray(np.where(self.fol, self.G + FBr, 1e9), np.float32)
        self.FT = np.ascontiguousarray(np.where(self.fol, self.G + FTr, -1e9), np.float32)
        xs = self.X0 + (np.arange(self.Ws) + 0.5) * 2
        zs = self.Z0 + (np.arange(self.Hs) + 0.5) * 2
        self.XX, self.ZZ = np.meshgrid(xs, zs)
        # foliage within conceal_r (concealment test)
        d = C.edt(self.lib, self.fol) * 2.0
        self.fol_near = d <= P["conceal_r"]

    # ---- grid helpers
    def cell(self, x, z):
        i = np.clip(((np.asarray(x) - self.X0) // 2).astype(np.int64), 0, self.Ws - 1)
        j = np.clip(((np.asarray(z) - self.Z0) // 2).astype(np.int64), 0, self.Hs - 1)
        return (j * self.Ws + i).astype(np.int32)

    def xy(self, idx):
        idx = np.asarray(idx)
        return self.X0 + (idx % self.Ws + 0.5) * 2, self.Z0 + (idx // self.Ws + 0.5) * 2

    def block_min(self, f05, mask05=None):
        f = f05 if mask05 is None else np.where(mask05, f05, np.inf)
        return f.reshape(self.Hs, 4, self.Ws, 4).min(axis=(1, 3)).astype(np.float32)

    def block_any(self, m05):
        return m05.reshape(self.Hs, 4, self.Ws, 4).any(axis=(1, 3))

    # ---- C wrappers
    def accumulate(self, groups, hq, hp, range_m=None):
        """groups: list of (cells int32, t float32, dt float32). -> exp (K, N), tf (K, N).
        range_m: sight range for this call (default P["range_m"])."""
        rng = float(self.P["range_m"] if range_m is None else range_m)
        K = len(groups)
        N = self.Hs * self.Ws
        gstart = np.zeros(K + 1, np.int32)
        for k, g in enumerate(groups):
            gstart[k + 1] = gstart[k] + len(g[0])
        cells = np.ascontiguousarray(np.concatenate([g[0] for g in groups]).astype(np.int32)) if K else np.zeros(0, np.int32)
        tt = np.ascontiguousarray(np.concatenate([g[1] for g in groups]).astype(np.float32)) if K else np.zeros(0, np.float32)
        dt = np.ascontiguousarray(np.concatenate([g[2] for g in groups]).astype(np.float32)) if K else np.zeros(0, np.float32)
        exp = np.zeros(K * N, np.float32)
        tf = np.full(K * N, np.inf, np.float32)
        self.lib.vs_accumulate(self.G, self.T, self.FB, self.FT, self.Hs, self.Ws, 2.0, K, gstart, cells, tt, dt,
                               float(hq), float(hp), rng, float(self.P["fol_depth"]), exp, tf)
        return exp.reshape(K, self.Hs, self.Ws), tf.reshape(K, self.Hs, self.Ws)

    def query(self, cells, thr, A, hq, hp):
        n = len(cells)
        minA = np.empty(n, np.float32)
        cnt = np.empty(n, np.int64)
        self.lib.vs_query(self.G, self.T, self.FB, self.FT, self.Hs, self.Ws, 2.0, n,
                          np.ascontiguousarray(cells, np.int32), np.ascontiguousarray(thr, np.float32), float(hq), float(hp),
                          float(self.P["range_m"]), float(self.P["fol_depth"]), np.ascontiguousarray(A, np.float32).ravel(),
                          minA, cnt)
        return minA, cnt

    def los(self, qcells, hq, pcells, hp):
        n = len(qcells)
        out = np.empty(3 * n, np.float32)
        self.lib.los_pairs(self.G, self.T, self.FB, self.FT, self.Hs, self.Ws, 2.0, n,
                           np.ascontiguousarray(qcells, np.int32), np.ascontiguousarray(np.broadcast_to(hq, n), np.float32),
                           np.ascontiguousarray(pcells, np.int32), np.ascontiguousarray(np.broadcast_to(hp, n), np.float32), out)
        o = out.reshape(n, 3)
        return o[:, 0] > 0.5, o[:, 1] > 0.5, o[:, 2]

    def seen(self, pcell, qcells):
        """Exact lines from an observer at pcell (eye_h) to targets qcells (target_h): seen flags and ranges."""
        P = self.P
        okT, okG, fd = self.los(qcells, P["target_h"], np.full(len(qcells), pcell, np.int32), P["eye_h"])
        px, pz = self.xy(pcell)
        qx, qz = self.xy(qcells)
        rng = np.hypot(qx - px, qz - pz)
        return okT & (fd < P["fol_depth"]) & (rng <= P["range_m"]), rng

    def hull_down(self, pcell, qcells):
        """Share of the given target cells from which an enemy sight (eye_h) sees the turret (turret_h) at pcell
        but terrain hides its hull (hull_h)."""
        P = self.P
        pc = np.full(len(qcells), pcell, np.int32)
        _, gH, _ = self.los(qcells, P["eye_h"], pc, P["hull_h"])
        _, gT, _ = self.los(qcells, P["eye_h"], pc, P["turret_h"])
        return float(np.mean(gT & ~gH)) if len(qcells) else 0.0


def zone_points(S, zones):
    """Zone interior points every ZONE_STEP m inside the capture circle, on cells without a hard object."""
    out = {}
    for (L, x, z, r) in zones:
        g = np.arange(-r, r + 1e-6, ZONE_STEP)
        X, Z = np.meshgrid(x + g, z + g)
        m = np.hypot(X - x, Z - z) <= r
        cells = np.unique(S.cell(X[m], Z[m]))
        cells = cells[S.objh_raw.ravel()[cells] <= 0]
        out[L] = cells
    return out


def corridor_groups(S, corrs, part=None, approach_m=150.0):
    groups = []
    for c in corrs:
        sm = c["samples"]
        m = np.ones(len(sm["x"]), bool)
        if part == "approach":
            m = sm["to_end"] <= approach_m
        elif part == "far":
            m = sm["to_end"] > approach_m
        groups.append((S.cell(sm["x"][m], sm["z"][m]), sm["t"][m], sm["dt"][m]))
    return groups


def pick_positions(score, cand, n, sep, Sx, Sz):
    """Greedy: best remaining cell, then suppress cells within sep metres. score: tuple of arrays (lexicographic)."""
    idx = np.flatnonzero(cand)
    if not len(idx):
        return []
    keys = [np.asarray(s).ravel()[idx] for s in score]
    order = np.lexsort(tuple(-k for k in reversed(keys)))
    idx = idx[order]
    xs, zs = Sx.ravel()[idx], Sz.ravel()[idx]
    alive = np.ones(len(idx), bool)
    out = []
    for i in range(len(idx)):
        if not alive[i]:
            continue
        out.append(int(idx[i]))
        alive &= np.hypot(xs - xs[i], zs - zs[i]) > sep
        if len(out) >= n:
            break
    return out


def label_components(mask):
    """8-connected components of a 2-D bool mask -> label array (-1 = none) and the number of labels."""
    H, W = mask.shape
    lab = -np.ones(mask.shape, np.int32)
    n = 0
    idx = np.flatnonzero(mask)
    flat = mask.ravel()
    lf = lab.ravel()
    for s in idx:
        if lf[s] >= 0:
            continue
        lf[s] = n
        st = [int(s)]
        while st:
            u = st.pop()
            j, i = divmod(u, W)
            for dj in (-1, 0, 1):
                jj = j + dj
                if jj < 0 or jj >= H:
                    continue
                for di in (-1, 0, 1):
                    ii = i + di
                    if 0 <= ii < W:
                        v = jj * W + ii
                        if flat[v] and lf[v] < 0:
                            lf[v] = n
                            st.append(v)
        n += 1
    return lab, n


def shape_of(S, cells):
    """Length (extent along the principal axis + one cell) and area of a set of 2 m cells."""
    x, z = S.xy(cells)
    if len(cells) < 2:
        return 2.0, 4 * len(cells)
    X = np.stack([x - x.mean(), z - z.mean()])
    w, v = np.linalg.eigh(np.cov(X))
    s = v[:, 1] @ X
    return float(s.max() - s.min() + 2.0), 4 * len(cells)


FLANK_TIME = 2.0         # class F: enemy 'other' corridors within 2 x the fastest time to their zone
SPAWN_REACH_S = 60.0     # per spawn area lists: areas it reaches first within 60 s of the start
MAX_LEN = 150.0         # an area longer than this is cut into equal pieces along its main axis


def split_long(S, cells):
    length, _ = shape_of(S, cells)
    if length <= MAX_LEN:
        return [cells]
    x, z = S.xy(cells)
    X = np.stack([x - x.mean(), z - z.mean()])
    w, v = np.linalg.eigh(np.cov(X))
    s = v[:, 1] @ X
    n = int(np.ceil(length / MAX_LEN))
    edges = np.linspace(s.min(), s.max() + 1e-6, n + 1)
    k = np.clip(np.searchsorted(edges, s, side="right") - 1, 0, n - 1)
    out = []
    for i in range(n):
        part = cells[k == i]
        if len(part) == 0:
            continue
        if len(part) < len(cells) and shape_of(S, part)[0] > MAX_LEN:
            out += split_long(S, part)
        else:
            out.append(part)
    return out


def sample_cells(cells, k):
    if len(cells) <= k:
        return cells
    return cells[np.linspace(0, len(cells) - 1, k).astype(int)]


TANK_FRONTAGE = 10.0     # one tank per 10 m of a position's length
MIN_AREA_CELLS = 3       # areas under 12 m2 are dropped (single cells at the edge of a test)
DETAIL_AREAS = 15        # areas per army (and per zone for fire support) with sampled hull-down / exposure detail
SAMPLE_K = 10            # sampled cells per area for the hull-down and own-exposure tests


def exposure_phase(S, Fl, a, b, army_t05, corrs, kill):
    """Step 3c: seconds each corridor of army a is under observation from places army b holds earlier (cells with
    turn room, b's time + setup <= the moment our tank passes), outside a's spawn protection."""
    P = S.P
    own = [c for c in corrs if c["army"] == a]
    enemy_pos_t2 = S.block_min(army_t05[b], Fl.turn05)
    ka = C.inside_kill(kill.get(a, []), S.XX, S.ZZ)
    A = np.where(ka | ~np.isfinite(enemy_pos_t2), np.inf, enemy_pos_t2 + P["setup_s"]).astype(np.float32)
    cexp = {}
    if not own:
        return cexp
    q = np.concatenate([S.cell(c["samples"]["x"], c["samples"]["z"]) for c in own])
    t = np.concatenate([c["samples"]["t"] for c in own]).astype(np.float32)
    ow = np.concatenate([np.full(len(c["samples"]["x"]), i) for i, c in enumerate(own)])
    minA, _ = S.query(q, t, A, P["target_h"], P["eye_h"])
    for i, c in enumerate(own):
        m = ow == i
        obs = minA[m] <= t[m]
        sm = c["samples"]
        cexp[c["id"]] = dict(expo_s=round(float(sm["dt"][obs].sum()), 1),
                             first_t=round(float(sm["t"][obs].min()), 1) if obs.any() else None,
                             observed=np.flatnonzero(obs))
    return cexp


REC_TIME = 1.3
W_ROLE = {"recommended": 1.0, "both": 1.0, "fastest": 0.6, "other": 0.3}


def recommend(corrs, cexp):
    """Per (spawn area, zone): fastest = least time; recommended = least seconds under observation among the
    corridors within 1.3 x the fastest time (ties: later first-seen moment, never seen counts as latest; then time)."""
    groups = {}
    for c in corrs:
        groups.setdefault((c["area"], c["zone"]), []).append(c)
    for key, cs in groups.items():
        fast = min(cs, key=lambda c: c["time"])
        ok = [c for c in cs if c["time"] <= REC_TIME * fast["time"]]
        rec = min(ok, key=lambda c: (cexp[c["id"]]["expo_s"],
                                     -(cexp[c["id"]]["first_t"] if cexp[c["id"]]["first_t"] is not None else 1e9),
                                     c["time"]))
        for c in cs:
            c["expo_s"] = cexp[c["id"]]["expo_s"]
            c["first_seen_t"] = cexp[c["id"]]["first_t"]
            c["role"] = "both" if (c is rec and c is fast) else ("recommended" if c is rec else ("fastest" if c is fast else "other"))
            c["weight"] = W_ROLE[c["role"]]
    return groups


def analyse_army(S, Fl, a, b, own_t05, enemy_t05, corrs, zones, kill, zin, zin_exp, areas, area_fields, cexp):
    """Step-3 results for army a against army b (corridor exposure cexp already computed for a)."""
    P = S.P
    enem = [c for c in corrs if c["army"] == b]
    wk = np.array([c["weight"] for c in enem], np.float32)
    own_t2 = S.block_min(own_t05, Fl.turn05)
    enemy_t2 = S.block_min(enemy_t05, Fl.valid05 & Fl.reach)
    kb = C.inside_kill(kill.get(b, []), S.XX, S.ZZ)      # b's spawn protection: a may not stand here
    ka = C.inside_kill(kill.get(a, []), S.XX, S.ZZ)
    allowed = np.isfinite(own_t2) & ~kb
    ids = [x["id"] for x in areas if x["army"] == a]
    af = np.stack([S.block_min(area_fields[i], Fl.turn05) for i in ids])
    Aen = np.where(ka | ~np.isfinite(enemy_t2), np.inf, enemy_t2).astype(np.float32)
    res = dict(army=a)
    # ---------------- interdiction: two classes of areas, each from every 2 m cell that passes its own test
    exp, tf = S.accumulate(corridor_groups(S, enem), P["target_h"], P["eye_h"])
    feas = (exp >= P["min_expo_s"]) & (own_t2[None] + P["setup_s"] < tf) & allowed[None]
    is_rec = np.array([c["role"] in ("recommended", "both") for c in enem])
    is_fast = np.array([c["role"] == "fastest" for c in enem])
    main = is_rec | is_fast
    fastest_t = {}
    for c in enem:
        fastest_t[(c["area"], c["zone"])] = min(fastest_t.get((c["area"], c["zone"]), 1e9), c["time"])
    # class F corridors: the enemy's OTHER corridors within FLANK_TIME x the fastest time to their zone
    flank = np.array([(not m) and c["time"] <= FLANK_TIME * fastest_t[(c["area"], c["zone"])] for c, m in zip(enem, main)])
    ksets = {"M": main, "F": flank}
    cellM = feas[main].any(0)
    cellF = feas[flank].any(0) & ~cellM          # F: watches a flank corridor in time and is not an M cell
    cellsets = {"M": cellM, "F": cellF}
    # how early each cell could be occupied, per own spawn area
    res["corridor_classes"] = {c["id"]: ("M" if m else ("F" if f else "-")) for c, m, f in zip(enem, main, flank)}
    classes = {}
    for cl in ("M", "F"):
        km = ksets[cl]
        if not km.any():
            classes[cl] = []
            continue
        fe = np.where(feas & km[:, None, None], exp, 0)
        cell_n = (fe > 0).sum(0)
        cell_s = fe.sum(0)
        deadline = np.where(feas & km[:, None, None], tf, np.inf).min(0) - P["setup_s"]
        # areas: neighbouring cells that mainly watch the same enemy (spawn area, zone) group, cut into <= 150 m pieces
        gkeys = sorted({(c["area"], c["zone"]) for c, m in zip(enem, km) if m})
        gw = np.stack([fe[[k for k, c in enumerate(enem) if km[k] and (c["area"], c["zone"]) == g]].sum(0) for g in gkeys])
        sig = np.where(cellsets[cl], np.argmax(gw, axis=0), -1)
        pieces = []
        for g in range(len(gkeys)):
            lab_g, n_g = label_components(sig == g)
            lf = lab_g.ravel()
            order = np.argsort(lf)
            bounds = np.searchsorted(lf[order], np.arange(n_g + 1))
            for r in range(n_g):
                pieces += split_long(S, order[bounds[r]:bounds[r + 1]])
        regions = []
        for cells in pieces:
            if len(cells) < MIN_AREA_CELLS:
                continue
            fk = feas.reshape(len(enem), -1)[:, cells]
            ek = np.where(fk, exp.reshape(len(enem), -1)[:, cells], 0)
            tk = np.where(fk, tf.reshape(len(enem), -1)[:, cells], np.inf)
            em = ek.max(1)
            n_cls = int(((em > 0) & km).sum())
            s_cls = float(em[km].sum())
            ot_min = float(own_t2.ravel()[cells].min())
            regions.append(dict(cells=cells, key=(n_cls, s_cls, -ot_min), ek=ek, tk=tk))
        # rank by the class's own facts: corridors of the class watched, then seconds watched, then earliest occupation
        regions.sort(key=lambda r: tuple(-v for v in r["key"]))
        lex = cell_n.astype(np.float64) * 1e6 + cell_s
        out_regions = []
        for n, r in enumerate(regions):
            cells = r["cells"]
            length, area = shape_of(S, cells)
            ot = own_t2.ravel()[cells]
            seen_k = [k for k in range(len(enem)) if r["ek"][k].max() > 0]
            cuts = [dict(corridor=enem[k]["id"], zone=enem[k]["zone"], role=enem[k]["role"],
                         cls="M" if main[k] else ("F" if flank[k] else "-"),
                         expo_s=round(float(r["ek"][k].max()), 1), first_seen_t=round(float(r["tk"][k].min()), 1))
                    for k in seen_k]
            bc0 = int(cells[int(np.argmax(lex.ravel()[cells]))])
            by = float(deadline.ravel()[bc0])
            # per own spawn area: earliest time it reaches any cell of the area
            reach_by = {ids[i]: round(float(af[i].ravel()[cells].min()), 1) for i in range(len(ids))}
            first = min(reach_by, key=reach_by.get)
            rec = dict(label=f"{cl}{n + 1}", cls=cl, rank=n + 1, cells_n=len(cells), area_m2=area, length_m=round(length, 0),
                       tanks=max(1, int(length // TANK_FRONTAGE)),
                       class_corridors=r["key"][0], class_seconds=round(r["key"][1], 1),
                       x=round(float(S.xy(bc0)[0]), 1), z=round(float(S.xy(bc0)[1]), 1),
                       own_t=round(float(own_t2.ravel()[bc0]), 1), own_t_min=round(float(ot.min()), 1),
                       reach_by=reach_by, first_area=first,
                       by=round(by, 1), spare_s=round(by - float(own_t2.ravel()[bc0]), 1),
                       cuts=sorted(cuts, key=lambda x: (x["cls"] != cl, -x["expo_s"])),
                       zones=sorted({x["zone"] for x in cuts if x["cls"] == cl}),
                       concealed_share=round(float(S.fol_near.ravel()[cells].mean()), 2), cells=cells,
                       seen_corridors=[enem[k]["id"] for k in seen_k])
            if n < DETAIL_AREAS:
                rng, qall = [], []
                for k in seen_k:
                    if not km[k]:
                        continue
                    sm = enem[k]["samples"]
                    qc = S.cell(sm["x"], sm["z"])
                    vis, rr = S.seen(bc0, qc)
                    if vis.any():
                        rng += [float(rr[vis].min()), float(rr[vis].max())]
                        qall.append(qc[vis])
                rec["range_m"] = [round(min(rng)), round(max(rng))] if rng else None
                qall = np.unique(np.concatenate(qall)) if qall else np.zeros(0, np.int32)
                sc = sample_cells(cells, SAMPLE_K)
                hd = [S.hull_down(int(p), qall) for p in sc] if len(qall) else [0.0]
                rec["hull_down_share"] = round(float(np.mean([h >= 0.5 for h in hd])), 2)
                minA, cnt = S.query(sc.astype(np.int32), np.full(len(sc), by + P["early_extra_s"], np.float32), Aen,
                                    P["turret_h"], P["eye_h"])
                rec["own_exposure"] = dict(enemy_area_seeing_m2_median=int(np.median(cnt)) * 4,
                                           earliest_enemy_seeing_t=None if minA.min() > 1e29 else round(float(minA.min()), 1))
            out_regions.append(rec)
        classes[cl] = out_regions
        del fe
    res["interdiction_M"] = classes["M"]
    res["interdiction_F"] = classes["F"]
    res["interdiction_areas"] = classes["M"] + classes["F"]
    # per own spawn area: its three best areas of each class among those it reaches first within 60 s
    per_area = {}
    for i in ids:
        per_area[i] = {cl: [r["label"] for r in classes[cl] if r["first_area"] == i and r["reach_by"][i] <= SPAWN_REACH_S][:3]
                       for cl in ("M", "F")}
    res["per_spawn_area"] = per_area
    # enemy corridors (per zone) that no area of either class watches
    watched = set()
    for cl in ("M", "F"):
        for r in classes[cl]:
            watched.update(r["seen_corridors"])
    res["unwatched"] = {L: [dict(corridor=c["id"], role=c["role"], cls=res["corridor_classes"][c["id"]])
                            for c in enem if c["zone"] == L and c["id"] not in watched] for (L, *_r) in zones}
    # labels: the dozen best points of class M, each tied to its area
    wM = np.where(feas & main[:, None, None], exp, 0).sum(0)
    pos_cells = pick_positions((wM,), cellM, P["n_positions"], P["pos_sep"], S.XX, S.ZZ)
    area_of_cell = {}
    for rr in classes["M"]:
        for c_ in rr["cells"]:
            area_of_cell[int(c_)] = rr["label"]
    res["interdiction"] = [dict(n=k + 1, x=round(float(S.xy(pc)[0]), 1), z=round(float(S.xy(pc)[1]), 1), cell=pc,
                                area=area_of_cell.get(int(pc)), own_t=round(float(own_t2.ravel()[pc]), 1))
                           for k, pc in enumerate(pos_cells)]
    # ---------------- delay potential and fall-back, per zone this army defends
    far_exp, _ = S.accumulate(corridor_groups(S, enem, "far", P["approach_m"]), P["eye_h"], P["turret_h"])
    zone_ids = [z[0] for z in zones]
    delay = {}
    for (L, zx, zz, zr) in zones:
        ks = [k for k, c in enumerate(enem) if c["zone"] == L]
        near = allowed & (np.hypot(S.XX - zx, S.ZZ - zz) <= DELAY_R)
        F = feas[ks] & near[None]
        picks = []
        covered = np.zeros(len(ks), bool)
        best_e = np.zeros(len(ks))
        wks = wk[ks]
        for step in range(2):
            gain = ((F & ~covered[:, None, None]) * wks[:, None, None]).sum(0)
            tot = np.where(F, np.maximum(exp[ks] - best_e[:, None, None], 0), 0).sum(0)
            if not F.any():
                break
            score = np.where(near, gain * 1e4 + tot, -1)
            pc = int(np.argmax(score))
            if score.ravel()[pc] <= 0:
                break
            picks.append(pc)
            fk = F[:, pc // S.Ws, pc % S.Ws]
            covered |= fk
            best_e = np.maximum(best_e, np.where(fk, exp[ks][:, pc // S.Ws, pc % S.Ws], 0))
        cov1 = int(F[:, picks[0] // S.Ws, picks[0] % S.Ws].sum()) if picks else 0
        zk = zone_ids.index(L)
        fb = (np.isfinite(own_t2) & ~kb & (own_t2 <= enemy_t2) & (np.hypot(S.XX - zx, S.ZZ - zz) <= FALLBACK_R) &
              (zin_exp[zk] >= FRAC_SEE) & (far_exp[ks].sum(0) == 0 if ks else True))
        fbc = pick_positions((zin_exp[zk],), fb, 1, 1.0, S.XX, S.ZZ)
        rec_ks = [i for i, k in enumerate(ks) if enem[k]["role"] in ("recommended", "both")]
        delay[L] = dict(enemy_corridors=len(ks), covered_by_1=cov1, covered_by_2=int(covered.sum()),
                        enemy_recommended=len(rec_ks), recommended_covered_by_2=int(covered[rec_ks].sum()) if rec_ks else 0,
                        positions=[dict(x=round(float(S.xy(p)[0]), 1), z=round(float(S.xy(p)[1]), 1),
                                        own_t=round(float(own_t2.ravel()[p]), 1)) for p in picks],
                        attacker_expo_s={enem[k]["id"]: round(float(best_e[i]), 1) for i, k in enumerate(ks)},
                        uncovered=[enem[k]["id"] for i, k in enumerate(ks) if not covered[i]],
                        fallback=bool(fbc), fallback_area_m2=int(fb.sum()) * 4,
                        fallback_pos=dict(x=round(float(S.xy(fbc[0])[0]), 1), z=round(float(S.xy(fbc[0])[1]), 1),
                                          zone_seen=round(float(zin_exp[zk].ravel()[fbc[0]]), 2)) if fbc else None)
    res["delay"] = delay
    del exp, tf, feas, far_exp
    # ---------------- fire support onto zones, as areas
    appr_exp, _ = S.accumulate(corridor_groups(S, enem, "approach", P["approach_m"]), P["target_h"], P["eye_h"])
    ka_zone = [[k for k, c in enumerate(enem) if c["zone"] == L] for (L, *_r) in zones]
    appr_by_zone = np.stack([appr_exp[k].sum(0) if k else np.zeros(S.G.shape, np.float32) for k in ka_zone])
    fire = {}
    for zi, (L, zx, zz, zr) in enumerate(zones):
        d = np.hypot(S.XX - zx, S.ZZ - zz)
        with np.errstate(invalid="ignore"):
            hold = allowed & (own_t2 - enemy_t2 <= P["fire_margin_s"]) & (d > zr)
        cand = hold & (zin_exp[zi] >= FRAC_SEE)
        flab, fn = label_components(cand)
        fl = flab.ravel()
        o2 = np.argsort(fl)
        b2 = np.searchsorted(fl[o2], np.arange(fn + 1))
        regs = []
        for r in range(fn):
            for cells in split_long(S, o2[b2[r]:b2[r + 1]]):
                if len(cells) < MIN_AREA_CELLS:
                    continue
                zs_ = zin_exp[zi].ravel()[cells]
                regs.append((float(zs_.max()) + 0.001 * float(appr_by_zone[zi].ravel()[cells].max()), cells))
        regs.sort(key=lambda t: -t[0])
        fa = []
        for n, (_, cells) in enumerate(regs):
            length, area = shape_of(S, cells)
            ot = own_t2.ravel()[cells]
            bo = int(cells[int(np.argmin(ot))])
            bz = int(cells[int(np.argmax(zin_exp[zi].ravel()[cells]))])
            rec = dict(id=n + 1, cells_n=len(cells), area_m2=area, length_m=round(length, 0),
                       tanks=max(1, int(length // TANK_FRONTAGE)),
                       x=round(float(S.xy(cells)[0].mean()), 1), z=round(float(S.xy(cells)[1].mean()), 1),
                       zone_seen_max=round(float(zin_exp[zi].ravel()[cells].max()), 2),
                       zone_seen_mean=round(float(zin_exp[zi].ravel()[cells].mean()), 2),
                       approach_seen_s=round(float(appr_by_zone[zi].ravel()[cells].max()), 1),
                       range_m=[round(float(d.ravel()[cells].min())), round(float(d.ravel()[cells].max()))],
                       own_t=round(float(ot.min()), 1), first_area=ids[int(np.argmin(af[:, bo // S.Ws, bo % S.Ws]))],
                       enemy_t=None if not np.isfinite(enemy_t2.ravel()[bz]) else round(float(enemy_t2.ravel()[bz]), 1),
                       concealed_share=round(float(S.fol_near.ravel()[cells].mean()), 2), cells=cells)
            if n < DETAIL_AREAS:
                sc = sample_cells(cells, SAMPLE_K)
                rec["hull_down_share"] = round(float(np.mean([S.hull_down(int(p), zin[L]) >= 0.5 for p in sc])), 2)
                minA, cnt = S.query(sc.astype(np.int32), np.full(len(sc), float(ot.min()) + P["early_extra_s"], np.float32),
                                    Aen, P["turret_h"], P["eye_h"])
                rec["own_exposure"] = dict(enemy_area_seeing_m2_median=int(np.median(cnt)) * 4,
                                           earliest_enemy_seeing_t=None if minA.min() > 1e29 else round(float(minA.min()), 1))
            fa.append(rec)
        pcs = pick_positions((np.round(zin_exp[zi], 1), appr_by_zone[zi]), cand, 5, P["pos_sep"], S.XX, S.ZZ)
        fire[L] = dict(area_m2=int(cand.sum()) * 4, areas=fa, positions=[
            dict(x=round(float(S.xy(p)[0]), 1), z=round(float(S.xy(p)[1]), 1), own_t=round(float(own_t2.ravel()[p]), 1),
                 zone_seen=round(float(zin_exp[zi].ravel()[p]), 2), approach_seen_s=round(float(appr_by_zone[zi].ravel()[p]), 1),
                 range=round(float(d.ravel()[p]), 0)) for p in pcs])
    matrix = {}
    for xi, (X, xx, xz, xr) in enumerate(zones):
        row = {}
        near = allowed & (np.hypot(S.XX - xx, S.ZZ - xz) <= P["support_r"])
        for yi, (Y, yx, yz, yr) in enumerate(zones):
            if X == Y:
                continue
            m = near & (np.hypot(S.XX - yx, S.ZZ - yz) > yr)
            if not m.any():
                row[Y] = dict(best_seen=0.0, area_m2=0, approach_s=0.0)
                continue
            fr = np.where(m, zin_exp[yi], 0)
            p = int(np.argmax(fr))
            row[Y] = dict(best_seen=round(float(fr.ravel()[p]), 2), area_m2=int((m & (zin_exp[yi] >= FRAC_SEE)).sum()) * 4,
                          approach_s=round(float(np.where(m, appr_by_zone[yi], 0).max()), 1),
                          best_pos=dict(x=round(float(S.xy(p)[0]), 1), z=round(float(S.xy(p)[1]), 1)))
        matrix[X] = row
    res["fire"] = fire
    res["matrix"] = matrix
    res["corridor_exposure"] = cexp
    res["own_t2"], res["enemy_t2"] = own_t2, enemy_t2
    return res
