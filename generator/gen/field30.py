"""Steps 0-2: drivable field, spawn areas, speeds from the logs, travel-time fields, corridors.

Grids: 0.25 m class map (map.npz) -> clearance D = exact EDT distance to the nearest hard cell minus the cell
half-diagonal; 0.5 m search grid: clearance = mean of its 4 sub-cells; hull centre allowed where >= 2.3 m.
Travel time per 0.5 m step = route cost (terrain factor x (1 + clip(1 - (clr - 2.3) / 4, 0, 1))) x
step length / speed, speed = road speed on cells covered by the level's Lnav road mesh, else off-road speed.
"""
import csv
import math
import multiprocessing as mp
import time

import numpy as np

import common30 as C

H05 = 0.5


class Field:
    def __init__(self, mission, lib=None):
        t0 = time.time()
        self.mission = mission
        self.od = C.out_dir(mission)
        z = np.load(self.od / "map.npz")
        cls = z["cls"]
        self.X0, self.Z0 = float(z["X0"]), float(z["Z0"])
        self.tcost = z["tcost"]
        self.H, self.W = cls.shape
        self.H5, self.W5 = self.H // 2, self.W // 2
        self.lib = lib or C.dll()
        hard = np.isin(cls, C.HARD_CODES)
        D = (C.edt(self.lib, hard) * C.CELL - C.HALF_DIAG).astype(np.float32)
        del hard
        self.clr05 = D.reshape(self.H5, 2, self.W5, 2).mean(axis=(1, 3)).astype(np.float32)
        del D
        # class summaries kept at 0.5 m for gate labels / pictures (any sub-cell of that class)
        c4 = cls.reshape(self.H5, 2, self.W5, 2)
        self.water05 = (c4 == C.WATER).any(axis=(1, 3))
        self.obj05 = np.isin(c4, C.OBJ_HARD).any(axis=(1, 3))
        self.steep05 = (c4 == C.STEEP).any(axis=(1, 3))
        self.cls = cls
        self.valid05 = self.clr05 >= C.REQ_CLR
        self.clrf = (1 + C.CLR_W * np.clip(1 - (self.clr05 - C.REQ_CLR) / C.CLR_SPAN, 0, 1)).astype(np.float32)
        s = np.load(self.od / "sight.npz")
        self.has_road = bool(s["has_road"])
        self.road05 = s["road"].astype(bool) if self.has_road else np.zeros((self.H5, self.W5), bool)
        self.reach = None
        self.t_build = time.time() - t0

    # ---- coordinates
    def cell05(self, x, z):
        return int((z - self.Z0) // H05), int((x - self.X0) // H05)

    def idx(self, x, z):
        r, c = self.cell05(x, z)
        return r * self.W5 + c

    def centre05(self, r, c):
        return self.X0 + (c + 0.5) * H05, self.Z0 + (r + 0.5) * H05

    def xy_of(self, idx):
        idx = np.asarray(idx)
        r, c = idx // self.W5, idx % self.W5
        return self.X0 + (c + 0.5) * H05, self.Z0 + (r + 0.5) * H05

    def snap(self, x, z, radius=20.0, mask=None):
        r0, c0 = self.cell05(x, z)
        k = int(radius / H05)
        ra, rb, ca, cb = max(0, r0 - k), min(self.H5, r0 + k + 1), max(0, c0 - k), min(self.W5, c0 + k + 1)
        v = (self.valid05 if mask is None else mask)[ra:rb, ca:cb]
        if not v.any():
            return None
        rr, cc = np.nonzero(v)
        xs, zs = self.X0 + (cc + ca + 0.5) * H05, self.Z0 + (rr + ra + 0.5) * H05
        d = np.hypot(xs - x, zs - z)
        i = int(np.argmin(d))
        return int(rr[i] + ra), int(cc[i] + ca), float(d[i])

    def search(self, cost, sources, goal=-1, maxd=0.0, hscale=0.0):
        dist = np.empty(cost.shape, np.float32)
        par = np.empty(cost.shape, np.int32)
        src = np.array(sources, np.int32)
        self.lib.dijkstra(cost, cost.shape[0], cost.shape[1], src, len(src), int(goal), float(maxd),
                          float(hscale) if goal >= 0 else 0.0, dist, par)
        return dist, par

    def set_reach(self, pose_cells):
        unit = np.where(self.valid05, 1.0, 0.0).astype(np.float32)
        d, _ = self.search(unit, pose_cells)
        self.reach = d < 1e19
        self.unitR = np.where(self.valid05 & self.reach, 1.0, 0.0).astype(np.float32)
        self.turn05 = self.reach & (self.clr05 >= C.TURN_CLR)

    def time_cost(self, v_road, v_off):
        v = np.where(self.road05, v_road, v_off).astype(np.float32)
        return np.where(self.valid05 & self.reach, self.tcost * self.clrf * H05 / v, 0).astype(np.float32)


# ---------------------------------------------------------------- spawn areas
def spawn_areas(mission, Fl):
    """Spawn points of both armies (mission data), single-linkage clusters within SPAWN_CLUSTER_M -> areas named
    army + compass direction of the cluster centre from the battle-area centre."""
    sp = C.spawns(mission)
    bb = C.battle_area(mission)["bbox"]
    cx, cz = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
    areas, poses = [], []
    for army in (1, 2):
        P = sp[army]
        lab = -np.ones(len(P), int)
        k = 0
        for i in range(len(P)):
            if lab[i] >= 0:
                continue
            lab[i] = k
            stack = [i]
            while stack:
                u = stack.pop()
                d = np.hypot(P[:, 0] - P[u, 0], P[:, 1] - P[u, 1])
                for v in np.nonzero((d <= C.SPAWN_CLUSTER_M) & (lab < 0))[0]:
                    lab[v] = k
                    stack.append(v)
            k += 1
        names = []
        for g in range(k):
            m = lab == g
            ax, az = P[m, 0].mean(), P[m, 1].mean()
            nm = f"{army}{C.compass(C.bearing(ax - cx, az - cz))}"
            while nm in names:
                nm += "'"
            names.append(nm)
            areas.append(dict(id=nm, army=army, centre=[round(float(ax), 1), round(float(az), 1)], poses=[]))
        for i, (x, z) in enumerate(P):
            a = next(a for a in areas if a["id"] == names[lab[i]])
            sn = Fl.snap(x, z, 20.0)
            pid = f"{a['id']}.{len(a['poses']) + 1}"
            pose = dict(id=pid, army=army, area=a["id"], x=float(x), z=float(z),
                        cell=None if sn is None else int(sn[0] * Fl.W5 + sn[1]),
                        snap_m=None if sn is None else round(sn[2], 2))
            a["poses"].append(pid)
            poses.append(pose)
    return areas, poses


# ---------------------------------------------------------------- speeds from the logs
def log_speeds(Fl, runs):
    """Bot samples (ground_movement.csv measured_speed) and human samples (units.csv, 5 s chords) inside the area,
    split by the road mesh. Returns stats and the moving-sample medians."""
    out = dict(runs=runs)
    vals = {"bot_road": [], "bot_off": [], "human_road": [], "human_off": []}
    for run in runs:
        d = C.RUNS_DIR / run
        p = d / "ground_movement.csv"
        if p.is_file():
            with open(p, newline="") as f:
                for row in csv.DictReader(f):
                    try:
                        x, z, v = float(row["x"]), float(row["z"]), float(row["measured_speed"])
                    except (KeyError, ValueError, TypeError):
                        continue
                    r, c = Fl.cell05(x, z)
                    if 0 <= r < Fl.H5 and 0 <= c < Fl.W5 and v > 2.0:
                        vals["bot_road" if Fl.road05[r, c] else "bot_off"].append(v)
        p = d / "units.csv"
        if p.is_file():
            last = {}
            with open(p, newline="") as f:
                for row in csv.DictReader(f):
                    if row["is_bot"] != "0" or row["active"] != "1":
                        continue
                    t, x, z = float(row["t"]), float(row["x"]), float(row["z"])
                    k = row["id"]
                    if k in last:
                        t0, x0, z0 = last[k]
                        dt = t - t0
                        if 3.0 <= dt <= 7.0:
                            v = math.hypot(x - x0, z - z0) / dt
                            r, c = Fl.cell05((x + x0) / 2, (z + z0) / 2)
                            if 0 <= r < Fl.H5 and 0 <= c < Fl.W5 and 2.0 < v < 25.0:
                                vals["human_road" if Fl.road05[r, c] else "human_off"].append(v)
                    last[k] = (t, x, z)
    for k, v in vals.items():
        a = np.array(v)
        out[k] = dict(n=len(a), p25=round(float(np.percentile(a, 25)), 2) if len(a) else None,
                      p50=round(float(np.median(a)), 2) if len(a) else None,
                      p75=round(float(np.percentile(a, 75)), 2) if len(a) else None)
    road = vals["bot_road"] + vals["human_road"]
    off = vals["bot_off"] + vals["human_off"]
    out["v_road"] = round(float(np.median(road)), 2) if len(road) >= 200 else None
    out["v_off"] = round(float(np.median(off)), 2) if len(off) >= 200 else None
    return out


# ---------------------------------------------------------------- travel fields
def travel(Fl, cost, areas, poses, zones):
    fields = {}
    for a in areas:
        src = [p["cell"] for p in poses if p["area"] == a["id"] and p["cell"] is not None]
        d, _ = Fl.search(cost, src)
        fields[a["id"]] = np.where(d < 1e19, d, np.inf).astype(np.float32)
    army = {}
    for k in (1, 2):
        ids = [a["id"] for a in areas if a["army"] == k]
        army[k] = np.min([fields[i] for i in ids], axis=0)
        army[k + 10] = np.argmin([fields[i] for i in ids], axis=0).astype(np.int8)   # which area is first
    zf, zsrc = {}, {}
    for (L, x, z, r) in zones:
        rr, cc = np.nonzero(Fl.valid05 & Fl.reach)
        # cells within the zone disc (centres within r)
        r0, c0 = Fl.cell05(x - r, z - r)
        r1, c1 = Fl.cell05(x + r, z + r)
        sub = np.zeros((Fl.H5, Fl.W5), bool)
        rs, cs_ = slice(max(0, r0), min(Fl.H5, r1 + 2)), slice(max(0, c0), min(Fl.W5, c1 + 2))
        R, Cc = np.mgrid[rs, cs_]
        X, Z = Fl.X0 + (Cc + 0.5) * H05, Fl.Z0 + (R + 0.5) * H05
        sub[rs, cs_] = (np.hypot(X - x, Z - z) <= r) & Fl.valid05[rs, cs_] & Fl.reach[rs, cs_]
        src = np.flatnonzero(sub)
        zsrc[L] = src
        d, _ = Fl.search(cost, src)
        zf[L] = np.where(d < 1e19, d, np.inf).astype(np.float32)
    return fields, army, zf, zsrc


def arrivals(poses, zones, zf, areas):
    """Per zone and army: sorted pose times; 1st and 5th tank, also per spawn area."""
    out = {}
    for (L, x, z, r) in zones:
        f = zf[L].ravel()
        row = {}
        for k in (1, 2):
            ts = sorted((float(f[p["cell"]]), p["id"]) for p in poses if p["army"] == k and p["cell"] is not None)
            ts = [t for t in ts if np.isfinite(t[0])]
            row[k] = dict(first=round(ts[0][0], 1) if ts else None, fifth=round(ts[4][0], 1) if len(ts) >= 5 else None,
                          first_pose=ts[0][1] if ts else None)
            for a in areas:
                if a["army"] != k:
                    continue
                ta = sorted(float(f[p["cell"]]) for p in poses if p["area"] == a["id"] and p["cell"] is not None)
                row[a["id"]] = dict(first=round(ta[0], 1) if ta else None, fifth=round(ta[4], 1) if len(ta) >= 5 else None)
        out[L] = row
    return out


# ---------------------------------------------------------------- corridors (worker processes)
_W = {}


def _init(path, H5, W5, lib_needed=True):
    z = np.load(path)
    _W.update({k: z[k] for k in z.files})
    _W["lib"] = C.dll()
    _W["H5"], _W["W5"] = H5, W5


def _chain(par, start, W5):
    out = []
    i = int(start)
    pf = par.ravel()
    while i >= 0:
        out.append(i)
        i = int(pf[i])
    return np.array(out, np.int64)


def _search(cost, src, goal, hscale):
    lib = _W["lib"]
    dist = np.empty(cost.shape, np.float32)
    par = np.empty(cost.shape, np.int32)
    s = np.array(src, np.int32)
    lib.dijkstra(cost, cost.shape[0], cost.shape[1], s, len(s), int(goal), 0.0, float(hscale), dist, par)
    return dist, par


def _close(cells_list, s_list, W5, X0, Z0):
    """Geodesic discs (25 m) every 30 m along the accepted corridors from 60 m after the start to their end."""
    lib = _W["lib"]
    mark = np.zeros((_W["H5"], W5), np.uint8)
    centres = []
    for cells, s in zip(cells_list, s_list):
        for sv in np.arange(C.CLOSE_START, s[-1] + 1e-6, C.CLOSE_STEP):
            centres.append(int(cells[min(len(cells) - 1, int(np.searchsorted(s, sv)))]))
        centres.append(int(cells[-1]))
    idx = np.array(centres, np.int32)
    lib.geodisc(_W["unitR"], _W["H5"], W5, idx, len(idx), C.CLOSE_R / H05, mark)
    return mark, centres


def corridor_job(job):
    """job: (area id, pose cell (goal), zone letter, zone source cells, hscale, X0, Z0) -> list of corridors."""
    aid, goal, L, src, hscale, X0, Z0 = job
    W5 = _W["W5"]
    cost = _W["cost"]
    res = []
    cells_acc, s_acc, xy_acc = [], [], []
    tries = []
    t0 = time.time()
    dist, par = _search(cost, src, goal, hscale)
    if dist.ravel()[goal] >= 1e19:
        return dict(area=aid, zone=L, corridors=[], note="unreachable")
    best_t = float(dist.ravel()[goal])
    for k in range(C.CORR_MAX):
        if k > 0:
            mark, centres = _close(cells_acc, s_acc, W5, X0, Z0)
            mark.ravel()[goal] = 0
            c2 = np.where(mark > 0, 0, cost).astype(np.float32)
            dist, par = _search(c2, src, goal, hscale)
            if dist.ravel()[goal] >= 1e19:
                tries.append(dict(k=k, result="no path after closing"))
                break
        cells = _chain(par, goal, W5)            # pose -> zone edge
        r, c = cells // W5, cells % W5
        x, z = X0 + (c + 0.5) * H05, Z0 + (r + 0.5) * H05
        seg = np.hypot(np.diff(x), np.diff(z))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        dv = dist.ravel()[cells]
        T = float(dv[0])
        if k > 0:
            if T > C.CORR_TIME * best_t:
                tries.append(dict(k=k, result=f"time ratio {T / best_t:.2f} > {C.CORR_TIME}"))
                break
            far = s >= C.CLOSE_START
            P = np.stack([x[far][::4], z[far][::4]], 1)
            Q = np.concatenate([np.stack([xa[::4], za[::4]], 1) for xa, za in xy_acc])
            dmin = np.sqrt(((P[:, None, :] - Q[None, :, :]) ** 2).sum(-1)).min(1) if len(P) else np.zeros(0)
            ov = float((dmin <= C.OVERLAP_R).mean()) if len(dmin) else 1.0
            if ov > C.OVERLAP_MAX:
                tries.append(dict(k=k, result=f"overlap {ov:.2f} > {C.OVERLAP_MAX}"))
                break
        else:
            ov = 0.0
        res.append(dict(cells=cells, t=(T - dv).astype(np.float32), s=s.astype(np.float32), time=T, overlap=ov))
        cells_acc.append(cells)
        s_acc.append(s)
        xy_acc.append((x, z))
    return dict(area=aid, zone=L, corridors=res, tries=tries, best_t=best_t, seconds=round(time.time() - t0, 1))


def corridors(Fl, cost, areas, poses, zones, zf, zsrc, v_max, workers=8):
    cache = Fl.od / "_corr_cache.npz"
    np.savez(cache, cost=cost, unitR=Fl.unitR)
    jobs = []
    for a in areas:
        pc = [p for p in poses if p["area"] == a["id"] and p["cell"] is not None]
        for (L, x, z, r) in zones:
            f = zf[L].ravel()
            best = min(pc, key=lambda p: f[p["cell"]])
            if not np.isfinite(f[best["cell"]]):
                continue
            jobs.append((a["id"], best["cell"], L, zsrc[L], H05 / v_max, Fl.X0, Fl.Z0))
    out = []
    with mp.get_context("spawn").Pool(min(workers, len(jobs)), initializer=_init,
                                      initargs=(str(cache), Fl.H5, Fl.W5)) as pool:
        for r in pool.imap_unordered(corridor_job, jobs):
            out.append(r)
    return out
