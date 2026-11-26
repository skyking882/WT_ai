"""Route geometry for the route library (WTRTE004 / WTMAP004, layouts of binary_v4.py).

From the earlier route generator:
  clearance field D (0.25 m, exact EDT minus the cell half-diagonal, bilinear between cell centres), resampling at
  0.5 m, bending-energy smoothing with clearance projection (smooth_pinned), curve properties (heading, signed
  curvature, time with 10 m/s and 2.5 m/s2 lateral), pinned (tangent) start and the start-turn pivot check.
Added here:
  * requirements checked exactly as read_check.c does them: hull 7.4 x 3.6 m, bare and grown by 0.3 m, lattice every
    0.125 m, heading = direction to the next sample; clearance >= 2.332 m on the curve densified to 0.125 m;
  * repair: samples whose (grown) hull touches a hard cell are pushed sideways, away from the touching cells, in
    0.1 m steps (never below 2.36 m clearance), up to 60 rounds;
  * ends: an end whose last (first) 15 m still touch is cut at the last (first) clean sample minus (plus) 1 m;
    a route with touching samples elsewhere is rejected;
  * a route may keep a frozen tail (samples copied from an already finished trunk) that is not moved again.
"""
import math

import numpy as np

H05 = 0.5
CELL = 0.25
HALF_DIAG = CELL * 0.5 * 2 ** 0.5
REQ = 2.3                 # hull centre cells (search grid)
REQ_ROUTE = 2.332         # route clearance, densified to 0.125 m (contract)
SMOOTH_REQ, UNDO_REQ = 2.40, 2.36
HL, HW, GROW = 3.7, 1.8, 0.3
LEVELS = ((64, 80), (32, 120), (16, 120), (8, 120), (4, 120), (2, 80), (1, 80))
V_FREE, A_LAT = 10.0, 2.5
PIN_LEN, PIN_ANGLE, PIN_DIR_LEN = 3.0, 60.0, 10.0
PIN_R_MIN, PIN_CHECK_LEN = 8.0, 25.0
END_TRIM = 15.0
START_TURN_DEG_S = 15.0
PIVOT_R, PIVOT_STEP, PIVOT_DONE_DEG, PIVOT_GROW, PIVOT_AHEAD = 2.4, 0.1, 8.0, 0.3, 5.0


class Geo:
    """Map arrays for route finishing (loaded from the cache written by pack31; memory-mapped in workers, used as
    plain arrays so indexing has no memmap overhead)."""

    def __init__(self, cache):
        z = {k: np.asarray(np.load(cache / f"{k}.npy", mmap_mode="r")) for k in ("D", "hard", "tcost")}
        meta = np.load(cache / "meta.npy")
        self.X0, self.Z0 = float(meta[0]), float(meta[1])
        self.D, self.hard, self.tcost = z["D"], z["hard"], z["tcost"]
        self.Df = self.D.reshape(-1)
        self.H, self.W = self.hard.shape
        self.H5, self.W5 = self.H // 2, self.W // 2

    def _bil(self, x, z):
        fx = (np.asarray(x, np.float64) - (self.X0 + 0.125)) * (1 / CELL)
        fz = (np.asarray(z, np.float64) - (self.Z0 + 0.125)) * (1 / CELL)
        ok = (fx >= 0) & (fz >= 0) & (fx < self.W - 1) & (fz < self.H - 1)
        i = np.minimum(np.maximum(fx.astype(np.int64), 0), self.W - 2)
        j = np.minimum(np.maximum(fz.astype(np.int64), 0), self.H - 2)
        tx, tz = fx - i, fz - j
        k = j * self.W + i
        D = self.Df
        a, b, c, d = D[k], D[k + 1], D[k + self.W], D[k + self.W + 1]
        return ok, tx, tz, a, b, c, d

    def clr(self, x, z):
        ok, tx, tz, a, b, c, d = self._bil(x, z)
        v = (1 - tz) * ((1 - tx) * a + tx * b) + tz * ((1 - tx) * c + tx * d)
        return np.where(ok, v, -1.0)

    def clr_vg(self, x, z):
        """Clearance and its gradient (analytic, bilinear) at the points."""
        ok, tx, tz, a, b, c, d = self._bil(x, z)
        v = (1 - tz) * ((1 - tx) * a + tx * b) + tz * ((1 - tx) * c + tx * d)
        gx = ((1 - tz) * (b - a) + tz * (d - c)) * (1 / CELL)
        gz = ((1 - tx) * (c - a) + tx * (d - b)) * (1 / CELL)
        return np.where(ok, v, -1.0), gx, gz

    def clr_grad(self, x, z, e=0.125):
        _, gx, gz = self.clr_vg(x, z)
        return gx, gz

    def tcost_at(self, x, z):
        r = np.minimum(np.maximum(((np.asarray(z) - self.Z0) // H05).astype(int), 0), self.H5 - 1)
        c = np.minimum(np.maximum(((np.asarray(x) - self.X0) // H05).astype(int), 0), self.W5 - 1)
        return self.tcost[r, c]


def resample(P, h=H05):
    d = np.hypot(*np.diff(P, axis=0).T)
    s = np.concatenate([[0], np.cumsum(d)])
    n = max(2, int(round(s[-1] / h)) + 1)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, P[:, 0]), np.interp(t, s, P[:, 1])], axis=1)


def project(G, P, req, lo, hi, iters=4):
    for _ in range(iters):
        Q = P[lo:hi]
        if not len(Q):
            break
        c, gx, gz = G.clr_vg(Q[:, 0], Q[:, 1])
        need = req - c
        m = need > 0
        if not m.any():
            break
        g = np.hypot(gx[m], gz[m]) + 1e-9
        step = np.minimum(need[m], 0.25)
        Q[m, 0] += gx[m] / g * step
        Q[m, 1] += gz[m] / g * step
        P[lo:hi] = Q
    return P


def smooth(G, P0, pin=None, frozen=None, levels=LEVELS, alpha=0.1):
    """smooth_pinned (bending-energy flow, coarse to fine, clearance projection, moves that end under 2.36 m
    undone). Levels whose stencil exceeds a quarter of the curve are skipped (they cannot act on short curves).
    The first sample (pose) and the last are fixed."""
    P = resample(P0)

    def fix(P):
        if pin is not None:
            k = min(int(PIN_LEN / H05) + 1, len(P) - 1)
            P[:k, 0] = pin[0] + np.arange(k) * H05 * pin[2]
            P[:k, 1] = pin[1] + np.arange(k) * H05 * pin[3]
        return P
    P = fix(P)
    for k, iters in levels:
        if 4 * k > len(P) and k > 1:
            continue
        for it in range(iters):
            n = len(P)
            if n < 5:
                break
            old = P.copy()
            i = np.arange(2, n - 2)
            kk = np.minimum(k, np.minimum(i // 2, (n - 1 - i) // 2))
            d4 = P[i - 2 * kk] - 4 * P[i - kk] + 6 * P[i] - 4 * P[i + kk] + P[i + 2 * kk]
            step = -alpha * d4
            nrm = np.hypot(step[:, 0], step[:, 1])
            step *= np.minimum(1, 0.25 / (nrm + 1e-12))[:, None]
            P[i] += step
            project(G, P, SMOOTH_REQ, 1, n - 1, iters=2)
            fix(P)
            bad = G.clr(P[:, 0], P[:, 1]) < UNDO_REQ
            P[bad] = old[bad]
            if it % 10 == 9 or it == iters - 1:
                P = fix(resample(P))
                project(G, P, SMOOTH_REQ, 1, len(P) - 1)
                fix(P)
    P = fix(resample(P))
    project(G, P, SMOOTH_REQ, 1, len(P) - 1, iters=8)
    return fix(P)



def headings(P):
    seg = np.diff(P, axis=0)
    h = np.arctan2(seg[:, 1], seg[:, 0])
    return np.concatenate([h, h[-1:]]) if len(h) else np.zeros(len(P))


_LAT = {}


def lattice(hl, hw):
    k = (hl, hw)
    if k not in _LAT:
        u = -hl + np.arange(0, int(round(2 * hl / 0.125)) + 1) * 0.125
        v = -hw + np.arange(0, int(round(2 * hw / 0.125)) + 1) * 0.125
        u, v = u[u <= hl + 1e-6], v[v <= hw + 1e-6]
        U, V = [a.ravel() for a in np.meshgrid(u, v)]
        _LAT[k] = (U, V)
    return _LAT[k]


def sweep(G, P, hl, hw, want_points=False, idx=None):
    """read_check.c sweep: per sample, does the hull rectangle at the sample (heading to the next sample) contain
    a hard cell? Only samples whose centre clearance is below the rectangle's half-diagonal + 0.35 m can touch (D is
    a lower bound of the distance to the nearest hard square up to the bilinear error); the others are skipped.
    idx: optional sample indices to test. Returns bool array (and the mean lateral offset of the touching points)."""
    U, V = lattice(hl, hw)
    h = headings(P)
    n = len(P)
    hit = np.zeros(n, bool)
    side = np.zeros(n)
    cand = np.arange(n) if idx is None else np.asarray(idx, np.int64)
    if len(cand):
        clr = G.clr(P[cand, 0], P[cand, 1])
        cand = cand[clr < math.hypot(hl, hw) + 0.35]
    for a in range(0, len(cand), 300):
        q = cand[a:a + 300]
        hx, hz = np.cos(h[q])[:, None], np.sin(h[q])[:, None]
        X = P[q, 0:1] + U * hx - V * hz
        Z = P[q, 1:2] + U * hz + V * hx
        c = np.clip(np.floor((X - G.X0) / CELL).astype(np.int64), 0, G.W - 1)
        r = np.clip(np.floor((Z - G.Z0) / CELL).astype(np.int64), 0, G.H - 1)
        hm = G.hard[r, c]
        hit[q] = hm.any(axis=1)
        if want_points:
            cnt = hm.sum(axis=1)
            side[q] = np.where(cnt > 0, (hm * V).sum(axis=1) / np.maximum(cnt, 1), 0.0)
    return (hit, side) if want_points else hit


def clear_dense(G, P, idx=None):
    """Per 0.5 m sample the least clearance on its segment to the next sample, densified to 0.125 m."""
    n = len(P)
    q = np.arange(n) if idx is None else np.asarray(idx, np.int64)
    out = np.full(n, 99.0)
    if not len(q):
        return out
    nxt = np.minimum(q + 1, n - 1)
    t = np.linspace(0, 1, 5)[:4]
    X = P[q, 0:1] + (P[nxt, 0:1] - P[q, 0:1]) * t
    Z = P[q, 1:2] + (P[nxt, 1:2] - P[q, 1:2]) * t
    out[q] = G.clr(X.ravel(), Z.ravel()).reshape(-1, 4).min(axis=1)
    return out


def bad_samples(G, P, lo=0, hi=None):
    """Samples in lo..hi that break a requirement: grown hull touching (covers the bare hull), or clearance under
    2.332 m on the densified curve. The heading of sample hi-1 uses sample hi, so the window is checked as is."""
    hi = len(P) if hi is None else hi
    idx = np.arange(lo, hi)
    grown, side = sweep(G, P, HL + GROW, HW + GROW, want_points=True, idx=idx)
    lowc = clear_dense(G, P, idx) < REQ_ROUTE
    return grown | lowc, side


def repair(G, P, lo=0, hi=None, rounds=60, step=0.1):
    """Push touching samples (index range lo..hi) sideways, away from the touching side, keeping >= 2.36 m."""
    hi = len(P) if hi is None else hi
    for _ in range(rounds):
        bad, side = bad_samples(G, P, max(0, lo - 1), min(len(P), hi + 1))
        idx = np.nonzero(bad[lo:hi])[0] + lo
        if not len(idx):
            return P, True
        h = headings(P)
        nx, nz = -np.sin(h), np.cos(h)            # left normal (v > 0 side)
        mv = np.zeros((len(P), 2))
        for i in idx:
            if side[i] == 0:                     # low clearance only: follow the clearance gradient
                gx, gz = G.clr_grad(P[i:i + 1, 0], P[i:i + 1, 1])
                g = math.hypot(float(gx[0]), float(gz[0])) + 1e-9
                d = np.array([float(gx[0]) / g, float(gz[0]) / g])
            else:
                d = (-1.0 if side[i] > 0 else 1.0) * np.array([nx[i], nz[i]])
            for k in range(-4, 5):
                j = i + k
                if lo <= j < hi and 0 < j < len(P) - 1:
                    mv[j] += d * step * (1 - abs(k) / 5)
        nn = np.hypot(mv[:, 0], mv[:, 1])
        mv *= np.minimum(1, step / (nn + 1e-12))[:, None]
        Q = P + mv
        ok = G.clr(Q[:, 0], Q[:, 1]) >= UNDO_REQ
        P = np.where(ok[:, None], Q, P)
    bad, _ = bad_samples(G, P, lo, hi)
    return P, not bad[lo:hi].any()


def trim(G, P, start=True, end=True, hi=None):
    """Cut touching ends (within 15 m of the end) at the last clean sample (minus 1 m). Returns (P, cut_start_m,
    cut_end_m) or None when samples elsewhere still touch. hi: samples from hi on are known clean (trunk)."""
    hi = len(P) if hi is None else hi
    bad, _ = bad_samples(G, P, 0, hi)
    if not bad.any():
        return P, 0.0, 0.0
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    L = s[-1]
    b = np.nonzero(bad)[0]
    a0, a1 = 0, len(P)
    tail = b[s[b] >= L - END_TRIM] if end else np.zeros(0, int)
    head = b[s[b] <= END_TRIM] if start else np.zeros(0, int)
    mid = np.setdiff1d(b, np.concatenate([tail, head]))
    if len(mid):
        return None
    if len(tail):
        a1 = int(np.searchsorted(s, s[tail.min()] - 1.0))
    if len(head):
        a0 = int(np.searchsorted(s, s[head.max()] + 1.0)) + 1
    if a1 - a0 < 5:
        return None
    Q = P[a0:a1]
    bad2, _ = bad_samples(G, Q, 0, min(len(Q), hi - a0 + 2) if hi < len(P) else len(Q))
    if bad2.any():
        return None
    return Q, float(s[a0]), float(L - s[a1 - 1])


def curve_props(G, P):
    seg = np.diff(P, axis=0)
    ds = np.hypot(seg[:, 0], seg[:, 1])
    s = np.concatenate([[0], np.cumsum(ds)])
    n = len(P)
    kappa = np.zeros(n)
    m = 4
    for i in range(n):
        a, b = max(0, i - m), min(n - 1, i + m)
        if a == i or b == i:
            continue
        p, q, r = P[a], P[i], P[b]
        u, v, w = q - p, r - q, r - p
        cr = u[0] * v[1] - u[1] * v[0]
        den = math.hypot(*u) * math.hypot(*v) * math.hypot(*w)
        kappa[i] = 2 * cr / den if den > 1e-9 else 0.0
    clr = G.clr(P[:, 0], P[:, 1])
    tc = G.tcost_at(P[:, 0], P[:, 1])
    R = np.where(np.abs(kappa) > 1e-6, 1 / np.maximum(np.abs(kappa), 1e-6), np.inf)
    v = np.minimum(V_FREE / tc, np.sqrt(A_LAT * R))
    dt = np.concatenate([[0], ds / np.maximum(0.5 * (v[:-1] + v[1:]), 0.1)])
    return dict(s=s, kappa=kappa, clr=clr, R=R, t=float(dt.sum()))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def pin_for(G, pose, Q):
    """Tangent start: pose heading within 60 deg of the path's first 10 m and 3 m ahead clear."""
    hd = pose.get("heading")
    if hd is None or not np.isfinite(hd):
        return None, None
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
    k = int(np.searchsorted(s, PIN_DIR_LEN)) if s[-1] > PIN_DIR_LEN else len(Q) - 1
    dv = Q[k] - Q[0]
    raw_pivot = wrap(math.degrees(math.atan2(dv[1], dv[0])) - hd)
    hx, hz = math.cos(math.radians(hd)), math.sin(math.radians(hd))
    ahead = (pose["x"] + PIN_LEN * hx, pose["z"] + PIN_LEN * hz)
    if abs(raw_pivot) <= PIN_ANGLE and float(G.clr(np.array([ahead[0]]), np.array([ahead[1]]))[0]) >= REQ and s[-1] > 12:
        return (pose["x"], pose["z"], hx, hz), raw_pivot
    return None, raw_pivot


def pivot_check(G, pose, P):
    """Start turn: turn on the spot at the pose (circle of 2.4 m about a centre 2.4 m to the side) until the heading is
    within 8 deg of the route point 5 m ahead; hull grown by 0.3 m swept; forward and reverse gear."""
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    k5 = min(len(P) - 1, int(np.searchsorted(s, PIVOT_AHEAD)))
    tx, tz = P[k5]
    x0, z0, h0 = pose["x"], pose["z"], math.radians(pose["heading"])

    def need(x, z, h):
        return (math.atan2(tz - z, tx - x) - h + math.pi) % (2 * math.pi) - math.pi
    sgn = 1.0 if need(x0, z0, h0) > 0 else -1.0
    out = {}
    U, V = lattice(HL + PIVOT_GROW, HW + PIVOT_GROW)
    for gear, side in (("fwd", 1.0), ("rev", -1.0)):
        na = h0 + sgn * side * math.pi / 2
        cx, cz = x0 + PIVOT_R * math.cos(na), z0 + PIVOT_R * math.sin(na)
        free = True
        for k in range(int(2 * math.pi / PIVOT_STEP) + 2):
            a = sgn * PIVOT_STEP * k
            ca, sa = math.cos(a), math.sin(a)
            x = cx + (x0 - cx) * ca - (z0 - cz) * sa
            z = cz + (x0 - cx) * sa + (z0 - cz) * ca
            h = h0 + a
            X = x + U * math.cos(h) - V * math.sin(h)
            Z = z + U * math.sin(h) + V * math.cos(h)
            c = np.clip(np.floor((X - G.X0) / CELL).astype(np.int64), 0, G.W - 1)
            r = np.clip(np.floor((Z - G.Z0) / CELL).astype(np.int64), 0, G.H - 1)
            if G.hard[r, c].any():
                free = False
                break
            if abs(need(x, z, h)) <= math.radians(PIVOT_DONE_DEG):
                break
        out[gear] = free
    return out


FREE_TAIL = 40            # samples (20 m) of a trunk re-smoothed together with a lead-in


def _smooth_route(G, Q, pin, tail):
    if tail is None:
        return smooth(G, Q, pin=pin), None
    k = min(FREE_TAIL, len(tail) - 1)
    Ps = smooth(G, np.vstack([Q, tail[1:k + 1]]), pin=pin)
    return np.vstack([Ps[:-1], tail[k:]]), len(Ps) - 1


def finish(G, raw, pose=None, tail=None, trim_start=True, trim_end=True):
    """raw: polyline from the start (pose) to the end, or to the join point tail[0] when tail (finished trunk
    samples, 0.5 m) is given: then the lead-in and the first 20 m of the tail are smoothed, the rest is kept.
    Returns a record dict or None (rejected: touching samples away from the ends)."""
    Q = resample(np.asarray(raw, float), H05)
    if tail is not None:
        Q = np.vstack([Q[:-1], tail[:1]]) if len(Q) > 1 else tail[:1]
        if len(Q) < 2:
            Q = np.vstack([np.asarray(raw, float)[:1], tail[:1]])
    pin, raw_pivot = (None, None)
    if pose is not None:
        pin, raw_pivot = pin_for(G, pose, Q if tail is None else np.vstack([Q, tail[1:41]]))
    P, nfree = _smooth_route(G, Q, pin, tail)
    if pin is not None:
        pr = curve_props(G, P[: int(2 * PIN_CHECK_LEN / H05)])
        r25 = float(pr["R"][pr["s"] <= PIN_CHECK_LEN].min())
        if r25 < PIN_R_MIN or sweep(G, P[: max(5, int(PIN_CHECK_LEN / H05))], HL, HW).any():
            pin = None
            P, nfree = _smooth_route(G, Q, None, tail)
    hi = len(P) if nfree is None else min(len(P), nfree + 1)
    P, ok = repair(G, P, 0, hi)
    cut_s = cut_e = 0.0
    if not ok or bad_samples(G, P, 0, hi)[0].any():
        t = trim(G, P, start=trim_start, end=trim_end and tail is None, hi=hi)
        if t is None:
            return None
        P, cut_s, cut_e = t
        if cut_s > 0:
            pin = None
    pr = curve_props(G, P)
    rec = dict(P=P, s=pr["s"], kappa=pr["kappa"], clr=pr["clr"], time_s=pr["t"], length=float(pr["s"][-1]),
               pinned=pin is not None, cut_start_m=round(cut_s, 1), cut_end_m=round(cut_e, 1))
    if pose is not None and pose.get("heading") is not None and np.isfinite(pose["heading"]) and cut_s == 0:
        h0 = math.degrees(math.atan2(P[min(4, len(P) - 1), 1] - P[0, 1], P[min(4, len(P) - 1), 0] - P[0, 0]))
        pivot = 0.0 if pin is not None else round(wrap(h0 - pose["heading"]), 1)
        rec["pivot"] = pivot
        if pivot:
            pc = pivot_check(G, pose, P)
            rec["gear"] = "forward" if pc["fwd"] else ("reverse" if pc["rev"] else "blocked")
            rec["fwd_free"], rec["rev_free"] = pc["fwd"], pc["rev"]
        else:
            rec["gear"], rec["fwd_free"], rec["rev_free"] = "none", True, True
    else:
        rec["pivot"], rec["gear"], rec["fwd_free"], rec["rev_free"] = 0.0, "none", False, False
        rec["no_start_turn_data"] = True
    return rec
