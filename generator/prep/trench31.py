"""Trench crossing lanes: a trench's footprint stays hard; where it is narrow and straight,
straight crossing lanes are opened, the way bridge lanes are.

Footprint: every rendinst whose name starts with 'trench_', cut with the object raster's hull band (0.25-2.6 m above
the ground under the instance) and rasterized at 0.25 m with the raster's own functions (build_ri_raster.py):
the same cells the merged map has as hard for these objects. Groups = 8-connected footprint cells.
Narrow stretch: at a footprint cell (candidates every 1 m), the local axis = principal axis of the group's cells within
3 m; local width = extent of the cells across the axis in a 1 m strip; narrow and straight when the widths in strips
at -1.8, 0 and +1.8 m along the axis are all <= TRENCH_CROSS_MAX_W and the three axes agree within 15 deg.
Lane: perpendicular to the axis, centred on the stretch; free band half-width LANE_HALF (= hull half-width 1.8 +
side margin 0.5 + 0.25 m for the 0.25 m cells, so the 2.3 m clearance rule holds against the trench cells left and
right of it); length = local width + a straight run-up of half a hull length (3.7 m) on each side. Opened only when
every hard cell in the lane rectangle is a cell of this trench within the crossing (free ground on both sides, run-ups
clear). One lane every 15 m along a group. Inside the lane only the band is freed, so a hull centre (clearance >= 2.3 m)
can only cross straight. trench_bridge / trench_enter / emplacement pieces get no lanes of their own (their collision is
a single box: no deck geometry to read); a lane is only opened where the narrow-stretch test holds.
"""
import math
import sys

import numpy as np

import common30 as C

TRENCH_CROSS_MAX_W = 2.0
LANE_HALF = 1.8 + 0.5 + 0.25
RUN_UP = 3.7
LANE_STEP = 15.0
AXIS_R = 3.0
AXIS_AGREE = 15.0


def trench_mask(mission, X0, Z0, H, W):
    sys.path.insert(0, str(C.S18))
    import build_ri_obstacles as bro
    import build_ri_raster as brr
    T = C.read_terrain(mission)
    codec = bro.cr.Codecs(str(bro.CDK))
    inst = bro.rigz_decode(codec, (bro.WT / "levels" / f"{C.mission_level(mission)}.bin").read_bytes())
    col = bro.Collision(codec)
    from colres import ColRes
    mask = np.zeros((H, W), bool)
    owner = -np.ones((H, W), np.int32)
    names = []
    res = {}
    for k, (name, px, py, pz, ax, ay, az) in enumerate(inst):
        if not name.lower().startswith("trench_"):
            continue
        if not (X0 - 30 <= px <= X0 + W * 0.25 + 30 and Z0 - 30 <= pz <= Z0 + H * 0.25 + 30):
            continue
        if name not in res:
            body = col.raw(name)
            try:
                res[name] = ColRes(body) if body else None
            except Exception:
                res[name] = None
        cr_ = res[name]
        if cr_ is None:
            continue
        A = np.array([ax, ay, az], float)
        sy = float(np.linalg.norm(A[1]))
        gy = float(C.bilinear(T["h"], T["x0"], T["z0"], T["step"], px, pz))
        dy = 0.0 if not np.isfinite(gy) else round((gy - py) * 4) / 4
        pts = []
        for nd in cr_.nodes:
            g = brr.node_triangles(nd)
            if g is not None:
                w = g[0] @ A + np.array([px, py, pz])
                pts.append(brr.section_points(w, g[1], py + dy + brr.BAND[0], py + dy + brr.BAND[1]))
        p2 = np.concatenate(pts) if pts else np.zeros((0, 2))
        if not len(p2):
            continue
        i = ((p2[:, 0] - X0) / 0.25).astype(np.int64)
        j = ((p2[:, 1] - Z0) / 0.25).astype(np.int64)
        ok = (i >= 0) & (i < W) & (j >= 0) & (j < H)
        mask[j[ok], i[ok]] = True
        owner[j[ok], i[ok]] = len(names)
        names.append((name, px, pz))
    return mask, owner, names


def _label(mask):
    from sight30 import label_components
    return label_components(mask)


def trench_lanes(mission, cls, X0, Z0):
    H, W = cls.shape
    mask, owner, names = trench_mask(mission, X0, Z0, H, W)
    hard = np.isin(cls, C.HARD_CODES)
    mask &= hard
    lab, n = _label(mask)
    jj, ii = np.nonzero(mask)
    xs, zs = X0 + (ii + 0.5) * 0.25, Z0 + (jj + 0.5) * 0.25
    gl = lab[jj, ii]
    lanes, narrow_pts = [], []
    groups = []
    for g in range(n):
        m = gl == g
        if m.sum() < 8:
            continue
        gx, gz = xs[m], zs[m]
        groups.append(dict(id=g, cells=int(m.sum()), x=float(gx.mean()), z=float(gz.mean()),
                           box=[float(gx.min()), float(gz.min()), float(gx.max()), float(gz.max())],
                           pieces=sorted({names[o][0] for o in owner[jj[m], ii[m]] if o >= 0})))
        # candidates every 1 m
        key = np.unique(np.stack([np.floor(gx), np.floor(gz)], 1), axis=0)
        chosen = []
        gx_all, gz_all = gx, gz
        bucket = {}
        for k_, (bx, bz) in enumerate(zip((gx_all // 8).astype(int), (gz_all // 8).astype(int))):
            bucket.setdefault((bx, bz), []).append(k_)
        for cx, cz in key + 0.5:
            bx, bz = int(cx // 8), int(cz // 8)
            idx = [k_ for dx in (-1, 0, 1) for dz in (-1, 0, 1) for k_ in bucket.get((bx + dx, bz + dz), [])]
            gx, gz = gx_all[idx], gz_all[idx]
            near = np.hypot(gx - cx, gz - cz) <= AXIS_R
            if near.sum() < 10:
                continue

            def axis_at(px, pz):
                q = np.hypot(gx - px, gz - pz) <= AXIS_R
                if q.sum() < 10:
                    return None
                X = np.stack([gx[q] - px, gz[q] - pz])
                w_, v_ = np.linalg.eigh(np.cov(X))
                return v_[:, 1]

            def width_at(px, pz, u):
                nrm = np.array([-u[1], u[0]])
                al = (gx - px) * u[0] + (gz - pz) * u[1]
                ac = (gx - px) * nrm[0] + (gz - pz) * nrm[1]
                s = (np.abs(al) <= 0.5) & (np.abs(ac) <= 4.0)
                if not s.any():
                    return None, 0.0
                return float(ac[s].max() - ac[s].min() + 0.25), float((ac[s].max() + ac[s].min()) / 2)
            u = axis_at(cx, cz)
            if u is None:
                continue
            ok, mids = True, []
            for off in (-1.8, 0.0, 1.8):
                px, pz = cx + off * u[0], cz + off * u[1]
                u2 = axis_at(px, pz)
                if u2 is None or math.degrees(math.acos(min(1.0, abs(float(u @ u2))))) > AXIS_AGREE:
                    ok = False
                    break
                w, mid = width_at(px, pz, u)
                if w is None or w > TRENCH_CROSS_MAX_W:
                    ok = False
                    break
                mids.append((w, mid))
            if not ok:
                continue
            narrow_pts.append((cx, cz, g))
            w0, mid0 = mids[1]
            nrm = np.array([-u[1], u[0]])
            ox, oz = cx + mid0 * nrm[0], cz + mid0 * nrm[1]
            half_len = w0 / 2 + RUN_UP
            # lane rectangle: |along| <= LANE_HALF, |across| <= half_len; every hard cell inside must be this trench's
            r = half_len + LANE_HALF + 1
            i0, i1 = max(0, int((ox - r - X0) / 0.25)), min(W - 1, int((ox + r - X0) / 0.25))
            j0, j1 = max(0, int((oz - r - Z0) / 0.25)), min(H - 1, int((oz + r - Z0) / 0.25))
            J, I = np.mgrid[j0:j1 + 1, i0:i1 + 1]
            Xc, Zc = X0 + (I + 0.5) * 0.25, Z0 + (J + 0.5) * 0.25
            al = (Xc - ox) * u[0] + (Zc - oz) * u[1]
            ac = (Xc - ox) * nrm[0] + (Zc - oz) * nrm[1]
            rect = (np.abs(al) <= LANE_HALF) & (np.abs(ac) <= half_len)
            sub_h = hard[j0:j1 + 1, i0:i1 + 1]
            sub_t = (lab[j0:j1 + 1, i0:i1 + 1] == g) & (np.abs(ac) <= w0 / 2 + 0.3)
            if (rect & sub_h & ~sub_t).any():
                continue
            if any(math.hypot(ox - L["x"], oz - L["z"]) < LANE_STEP for L in chosen):
                continue
            L = dict(group=g, x=round(float(ox), 2), z=round(float(oz), 2), axis_deg=round(math.degrees(math.atan2(u[1], u[0])), 1),
                     width=round(w0, 2), length=round(2 * half_len, 1), rect=(j0, i0, rect & sub_t))
            chosen.append(L)
        lanes += chosen
    # open: free the trench cells of each lane's band
    opened = 0
    for L in lanes:
        j0, i0, m = L.pop("rect")
        sub = cls[j0:j0 + m.shape[0], i0:i0 + m.shape[1]]
        sub[m] = C.DRV_OBJ
        opened += int(m.sum())
    # narrow stretches: single-link runs (1.5 m) of narrow candidate points
    stretches = 0
    if narrow_pts:
        P = np.array([(p[0], p[1]) for p in narrow_pts])
        seen = np.zeros(len(P), bool)
        for k in range(len(P)):
            if seen[k]:
                continue
            stretches += 1
            st = [k]
            seen[k] = True
            while st:
                a = st.pop()
                d = np.hypot(P[:, 0] - P[a, 0], P[:, 1] - P[a, 1])
                for b in np.nonzero((d <= 1.5) & ~seen)[0]:
                    seen[b] = True
                    st.append(b)
    rep = dict(rule=dict(TRENCH_CROSS_MAX_W=TRENCH_CROSS_MAX_W, LANE_HALF=LANE_HALF, RUN_UP=RUN_UP, LANE_STEP=LANE_STEP),
               trench_instances=len(names), groups=len(groups), narrow_stretches=stretches, lanes=len(lanes),
               cells_opened_025=opened, lane_list=lanes, group_list=groups)
    return rep, mask
