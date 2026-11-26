"""Road network from the lane generator's route library curves.

usage: python network38.py <map> [--lib <folder>] [--out <folder>]   (run_map.py calls it with both = out/<map>)
       python network38.py <map> --limit N  (first N routes only; development)

Input : <lib>/routes.bin, map.bin, <map>.plan.json, cache/ (D, hard, tcost of the generator; route31.Geo)
Output: <out>/<map>.network.bin, network_report.txt, net_queries.txt, network.png
(Rules in use: smoothstep blends, nodes = linked chain samples, crossings > 30 deg as nodes.)

Build (outline; see the report for the departures):
 1. canonical chains: routes in file order; a sample matches the nearest canonical sample within 0.5 m whose
    orientation (mod 180 deg) is within 15 deg; once on a chain the route stays on it while within 0.75 m / 20 deg;
    matched runs shorter than 8 m are unmatched; unmatched runs become new chains (resampled uniformly at ~0.5 m).
 2. joints: own samples within 6 m of a joint are blended linearly towards the chain (weight 1 at the joint);
    the node is the chain sample at the joint.
 3. nodes: piece ends (joints, route first / last samples, chain ends), merged within 0.25 m; chains split there.
 4. edges: length, time (route31.curve_props speed rule on the chain samples), min clearance, uses.
File layout: see write_network_bin / read_network_bin.
"""
import heapq
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))            # route31, binary_v4, picture38 live beside this file
if str(HERE.parent) not in sys.path:
    sys.path.insert(1, str(HERE.parent))
import paths  # noqa: E402
import route31 as R  # noqa: E402   (speed rule: curve_props; hull check: sweep; Geo)
import binary_v4 as BV  # noqa: E402  (read_routes_bin, read_map_bin)

STEP = 0.5
TOL_IN, ANG_IN = 0.5, math.radians(15.0)
TOL_OUT, ANG_OUT = 0.75, math.radians(20.0)
MIN_RUN = 8.0
BLEND = 6.0
NODE_MERGE = 0.25
MAX_STEP_DEG = 12.0
SPACE_TOL = 0.15
JUMP = 4                  # chain index jump inside a matched run that splits it (samples)
SWITCH_K = 12             # samples unmatched on each side of a chain switch without a shared node (6 m)
SHARED_BACK, SHARED_FWD = 40, 6   # shared-node search window (samples back / forward of the switch)
OWN_MIN_ONE = 2.0         # own piece with one joint (route start / end): at least this long (m)
TB = int(BLEND / STEP) + 6
TURN_FREE_DEG = 30.0      # turn at a node up to this: smooth, no cost
PIVOT_CLR = 4.3           # node pivot free: map.bin 0.5 m clearance at the node >= this (byte >= 43)
CROSS_ANG_DEG = 30.0     # crossings: edges crossing at more than this ...
CROSS_NODE_DIST = 1.0     # ... with no node within this become a node
HEAD_WIN = 2.0            # turn headings from the edge samples over this length next to the node
# variants (set from the command line for the comparison runs)
OPT = dict(shape="smooth",          # blend weight profile: "linear" (the outline) or "smooth" (3t^2 - 2t^3)
           own_two=2 * BLEND,       # own piece between two chains: at least this long (m, incl. both joints)
           shared_eps=0.01)       # chain switch: shared node only where the two chain samples are this close (m)


def blend_w(s):
    t = np.clip(s / BLEND, 0, 1)
    return 1 - t if OPT["shape"] == "linear" else 1 - t * t * (3 - 2 * t)

HDR = struct.Struct("<8sIIIIIIIffII")
NODE = struct.Struct("<ffIHH")
EDGE = struct.Struct("<IIIIffHBB")


def log(msg, t0=[time.time()]):
    print(f"[{time.time() - t0[0]:7.1f}s] {msg}", flush=True)


def heads(P):
    d = np.diff(P, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    return np.concatenate([h, h[-1:]]) if len(h) else np.zeros(len(P))


def orient(P):
    return np.mod(heads(P), math.pi)


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------ canonical store
class Canon:
    def __init__(self, x0, z0, ext, cap):
        self.cell = TOL_OUT
        self.x0, self.z0 = x0 - 2.0, z0 - 2.0
        self.gw = int((ext + 4.0) / self.cell) + 3
        self.head = np.full(self.gw * self.gw, -1, np.int32)
        self.x = np.zeros(cap)
        self.z = np.zeros(cap)
        self.ori = np.zeros(cap)
        self.clr = np.zeros(cap, np.uint8)
        self.chain = np.zeros(cap, np.int32)
        self.nxt = np.full(cap, -1, np.int32)
        self.n = 0
        self.cs = np.zeros(200000, np.int64)
        self.cn = np.zeros(200000, np.int64)
        self.nc = 0

    def key(self, x, z):
        ci = np.clip(((x - self.x0) / self.cell).astype(np.int64), 1, self.gw - 2)
        cj = np.clip(((z - self.z0) / self.cell).astype(np.int64), 1, self.gw - 2)
        return ci, cj

    def add_chain(self, P, clr):
        n = len(P)
        g0 = self.n
        g = np.arange(g0, g0 + n)
        self.x[g], self.z[g] = P[:, 0], P[:, 1]
        self.ori[g] = orient(P) if n > 1 else 0.0
        self.clr[g] = clr
        c = self.nc
        self.chain[g] = c
        self.cs[c], self.cn[c] = g0, n
        self.nc += 1
        self.n += n
        ci, cj = self.key(P[:, 0], P[:, 1])
        k = cj * self.gw + ci
        o = np.argsort(k, kind="stable")
        ks, gs = k[o], g[o]
        same = ks[1:] == ks[:-1]
        self.nxt[gs[:-1][same]] = gs[1:][same]
        last = np.r_[~same, True]
        first = np.r_[True, ~same]
        self.nxt[gs[last]] = self.head[ks[last]]
        self.head[ks[first]] = gs[first]
        return c

    def pts(self, c, j):
        g = self.cs[c] + np.asarray(j)
        return np.stack([self.x[g], self.z[g]], axis=-1)

    def query(self, P, ori):
        n = len(P)
        ci, cj = self.key(P[:, 0], P[:, 1])
        I, Gs, D, A = [], [], [], []
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                cur = self.head[(cj + dj) * self.gw + ci + di].astype(np.int64)
                si = np.arange(n)
                m = cur >= 0
                si, cur = si[m], cur[m]
                while len(cur):
                    d = np.hypot(self.x[cur] - P[si, 0], self.z[cur] - P[si, 1])
                    da = np.abs(self.ori[cur] - ori[si])
                    da = np.minimum(da, math.pi - da)
                    k = (d < TOL_OUT) & (da < ANG_OUT)
                    if k.any():
                        I.append(si[k]), Gs.append(cur[k]), D.append(d[k]), A.append(da[k])
                    cur = self.nxt[cur].astype(np.int64)
                    m = cur >= 0
                    si, cur = si[m], cur[m]
        if not I:
            e = np.zeros(0, np.int64)
            return e, e, np.zeros(0), np.zeros(0)
        return np.concatenate(I), np.concatenate(Gs), np.concatenate(D), np.concatenate(A)


# ------------------------------------------------------------------------------------------------ per route
def label_route(cv, P):
    n = len(P)
    lab = np.full(n, -1, np.int64)
    labj = np.full(n, -1, np.int64)
    if cv.n == 0:
        return lab, labj
    I, Gs, D, A = cv.query(P, orient(P))
    if not len(I):
        return lab, labj
    C = cv.chain[Gs].astype(np.int64)
    J = Gs - cv.cs[C]
    ii = np.nonzero((D < TOL_IN) & (A < ANG_IN))[0]
    if not len(ii):
        return lab, labj
    ii = ii[np.lexsort((D[ii], I[ii]))]
    u, f = np.unique(I[ii], return_index=True)
    best = np.full(n, -1, np.int64)
    best[u] = ii[f]
    has = np.nonzero(best >= 0)[0]
    i = 0
    while True:
        k = np.searchsorted(has, i)
        if k >= len(has):
            break
        s = has[k]
        c = C[best[s]]
        sel = np.nonzero(C == c)[0]
        okc = np.zeros(n + 1, bool)
        okc[I[sel]] = True
        e = s + int(np.argmin(okc[s:]))
        ss = sel[(I[sel] >= s) & (I[sel] < e)]
        ss = ss[np.lexsort((D[ss], I[ss]))]
        u, f = np.unique(I[ss], return_index=True)
        lab[u] = c
        labj[u] = J[ss[f]]
        i = e
    return lab, labj


def pieces_of(lab, labj):
    n = len(lab)
    if n == 0:
        return []
    br = (lab[1:] != lab[:-1]) | ((lab[1:] >= 0) & (np.abs(labj[1:] - labj[:-1]) > JUMP))
    b = np.r_[0, np.nonzero(br)[0] + 1, n]
    out = []
    for a, e in zip(b[:-1], b[1:]):
        if lab[a] < 0:
            out.append(dict(m=False, i0=int(a), i1=int(e - 1)))
        else:
            jf, jl = int(labj[a]), int(labj[e - 1])
            out.append(dict(m=True, i0=int(a), i1=int(e - 1), c=int(lab[a]), jf=jf, jl=jl, d=int(np.sign(jl - jf))))
    return out


def find_shared(cv, p, q):
    """Shared node between the end of matched piece p and the start of matched piece q (chain samples < 0.25 m
    apart, heading continuous within 12 deg). Returns (k on p's chain, k2 on q's chain) or None."""
    c, jf, jl, dp = p["c"], p["jf"], p["jl"], p["d"]
    c2, jf2, jl2, dq = q["c"], q["jf"], q["jl"], q["d"]
    n0, n1 = cv.cn[c], cv.cn[c2]
    u = np.arange(-SHARED_BACK, SHARED_FWD + 1)
    k = jl + dp * u
    ok = (k >= 0) & (k < n0) & ((k - jf) * dp >= 1)
    u, k = u[ok], k[ok]
    u2 = np.arange(-SHARED_FWD, SHARED_BACK + 1)
    k2 = jf2 + dq * u2
    ok2 = (k2 >= 0) & (k2 < n1) & ((jl2 - k2) * dq >= 1)
    u2, k2 = u2[ok2], k2[ok2]
    if not len(k) or not len(k2):
        return None
    X1, X2 = cv.pts(c, k), cv.pts(c2, k2)
    Dm = np.hypot(X1[:, None, 0] - X2[None, :, 0], X1[:, None, 1] - X2[None, :, 1])
    a, b = np.nonzero(Dm < min(NODE_MERGE, OPT["shared_eps"]))
    if not len(a):
        return None
    ka, kb = k[a], k2[b]
    vi = cv.pts(c, ka) - cv.pts(c, ka - dp)
    vo = cv.pts(c2, kb + dq) - cv.pts(c2, kb)
    dh = np.abs(wrapd(np.degrees(np.arctan2(vo[:, 1], vo[:, 0]) - np.arctan2(vi[:, 1], vi[:, 0]))))
    g = dh <= MAX_STEP_DEG
    if not g.any():
        return None
    cost = np.abs(u[a]) + np.abs(u2[b]) + 1000 * ~g
    w = int(np.argmin(cost))
    return int(ka[w]), int(kb[w])


def settle(cv, lab, labj):
    """Apply MIN_RUN, chain switches (shared node or an own window), own piece minimum lengths. Returns pieces with
    the matched end indices adjusted to shared nodes."""
    n = len(lab)
    stats = dict(short=0, switch_shared=0, switch_window=0, own_grow=0)
    while True:
        ps = pieces_of(lab, labj)
        ch = False
        for p in ps:
            if p["m"] and ((p["i1"] - p["i0"]) * STEP < MIN_RUN or p["d"] == 0):
                lab[p["i0"]:p["i1"] + 1] = -1
                stats["short"] += 1
                ch = True
        if ch:
            continue
        for a in range(len(ps) - 1):
            p, q = ps[a], ps[a + 1]
            if p["m"] and q["m"]:
                r = find_shared(cv, p, q)
                if r is None:
                    lab[max(p["i0"], p["i1"] - SWITCH_K + 1):p["i1"] + 1] = -1
                    lab[q["i0"]:min(q["i1"], q["i0"] + SWITCH_K - 1) + 1] = -1
                    stats["switch_window"] += 1
                    ch = True
                    break
                p["jl"], q["jf"] = r
        if ch:
            continue
        for a, p in enumerate(ps):
            if p["m"]:
                continue
            left = a > 0 and ps[a - 1]["m"]
            right = a + 1 < len(ps) and ps[a + 1]["m"]
            if not (left or right):
                continue
            m = p["i1"] - p["i0"] + 1
            ln = (m + (1 if left and right else 0)) * STEP
            need = OPT["own_two"] if (left and right) else OWN_MIN_ONE
            if ln + 1e-9 < need:
                g = int(math.ceil((need - ln) / STEP / (2 if (left and right) else 1)))
                if left:
                    lab[max(0, p["i0"] - g):p["i0"]] = -1
                if right:
                    lab[p["i1"] + 1:min(n, p["i1"] + 1 + g)] = -1
                stats["own_grow"] += 1
                ch = True
        if ch:
            continue
        stats["switch_shared"] = sum(1 for a in range(len(ps) - 1) if ps[a]["m"] and ps[a + 1]["m"])
        return ps, stats


def local_line(cv, c, j, d, T):
    """Chain c from sample j stepping d, T+1 points, extended straight past the chain end."""
    n0 = cv.cn[c]
    idx = j + d * np.arange(T + 1)
    m = int(((idx >= 0) & (idx < n0)).sum())
    pts = cv.pts(c, idx[:m])
    if m < T + 1:
        if m >= 2:
            v = pts[-1] - pts[-2]
        else:
            jj = j - d
            v = pts[0] - cv.pts(c, jj) if 0 <= jj < n0 else np.array([1.0, 0.0])
        v = v / (np.hypot(*v) + 1e-12) * STEP
        ext = pts[-1] + v * np.arange(1, T + 2 - m)[:, None]
        pts = np.vstack([pts, ext])
    return pts


def project_poly(Q, L):
    A, B = L[:-1], L[1:]
    AB = B - A
    l2 = (AB ** 2).sum(1) + 1e-12
    t = np.clip(((Q[:, None, :] - A[None]) * AB[None]).sum(2) / l2[None], 0, 1)
    F = A[None] + t[..., None] * AB[None]
    d = np.hypot(F[..., 0] - Q[:, None, 0], F[..., 1] - Q[:, None, 1])
    k = np.argmin(d, axis=1)
    return F[np.arange(len(Q)), k]


def resample_uniform(V, vc):
    seg = np.hypot(*np.diff(V, axis=0).T)
    keep = np.r_[True, seg > 1e-6]
    V, vc = V[keep], vc[keep]
    if len(V) < 2:
        return V, vc
    seg = np.hypot(*np.diff(V, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    L = s[-1]
    ns = max(1, int(round(L / STEP)))
    t = np.linspace(0, L, ns + 1)
    out = np.stack([np.interp(t, s, V[:, 0]), np.interp(t, s, V[:, 1])], axis=1)
    out[0], out[-1] = V[0], V[-1]
    k = np.clip(np.searchsorted(s, t, side="right") - 1, 0, len(s) - 2)
    c = np.minimum(vc[k], vc[k + 1])
    return out, c


def build_own(cv, P, clr, p, left, right):
    Q = P[p["i0"]:p["i1"] + 1].copy()
    qc = clr[p["i0"]:p["i1"] + 1].astype(np.int64)
    m = len(Q)
    wl = np.zeros(m)
    wr = np.zeros(m)
    Fl = Fr = Q
    if left is not None:
        nodeL = cv.pts(left["c"], left["jl"])
        L = local_line(cv, left["c"], left["jl"], left["d"], TB)
        wl = blend_w(STEP * np.arange(1, m + 1))
        Fl = project_poly(Q, L)
    if right is not None:
        nodeR = cv.pts(right["c"], right["jf"])
        L = local_line(cv, right["c"], right["jf"], -right["d"], TB)
        wr = blend_w(STEP * np.arange(m, 0, -1))
        Fr = project_poly(Q, L)
    tot = wl + wr
    sc = np.where(tot > 1, 1 / np.maximum(tot, 1e-12), 1.0)
    Q2 = Q + (wl * sc)[:, None] * (Fl - Q) + (wr * sc)[:, None] * (Fr - Q)
    keep = np.ones(m, bool)
    V, vc = [], []
    if left is not None:
        keep &= np.hypot(*(Q2 - nodeL).T) >= NODE_MERGE
        V.append(nodeL[None])
        vc.append([int(cv.clr[cv.cs[left["c"]] + left["jl"]])])
    V.append(Q2[keep])
    vc.append(qc[keep])
    if right is not None:
        keep2 = np.hypot(*(Q2[keep] - nodeR).T) >= NODE_MERGE
        V[-1], vc[-1] = Q2[keep][keep2], qc[keep][keep2]
        V.append(nodeR[None])
        vc.append([int(cv.clr[cv.cs[right["c"]] + right["jf"]])])
    V = np.vstack(V)
    vc = np.concatenate([np.asarray(v, np.int64) for v in vc])
    return resample_uniform(V, vc)


SLIDE = 8                 # samples (4 m) a joint slides along its matched run when the blended piece touches


def hull_hits(G, S):
    """Per sample: the hull (bare or grown 0.3 m) touches hard, driving S forward or backward (route31.sweep takes
    the heading towards the next sample, so the two directions differ)."""
    f = R.sweep(G, S, R.HL + R.GROW, R.HW + R.GROW) | R.sweep(G, S, R.HL, R.HW)
    b = R.sweep(G, S[::-1], R.HL + R.GROW, R.HW + R.GROW) | R.sweep(G, S[::-1], R.HL, R.HW)
    return f | b[::-1]


def process_route(cv, r, G):
    """Label, settle, build the own pieces; every own piece is swept (hull bare and grown 0.3 m). A piece that
    touches within BLEND + 2 m of a joint makes that joint slide 4 m further along the matched run (the run's samples
    become own samples), and the route is settled and built again; when the run is used up the route keeps its own
    samples there."""
    P = r["xz"].astype(np.float64)
    clr = r["clr"]
    lab, labj = label_route(cv, P)
    sl = dict(slides=0, slide_m=[], gave_up=0, unfixable=0)
    joint_slid = {}
    for it in range(1000):
        ps, st = settle(cv, lab, labj)
        new = []        # (piece index, samples, clr)
        changed = False
        for a, p in enumerate(ps):
            if p["m"]:
                continue
            left = ps[a - 1] if a > 0 else None
            right = ps[a + 1] if a + 1 < len(ps) else None
            S, c = build_own(cv, P, clr, p, left, right)
            if len(S) >= 2:
                hit = hull_hits(G, S)
                if hit.any():
                    s = arc(S)
                    hs = s[np.nonzero(hit)[0]]
                    nl = left is not None and bool((hs <= BLEND + 2.0).any())
                    nr = right is not None and bool((hs >= s[-1] - BLEND - 2.0).any())
                    if not (nl or nr):
                        sl["unfixable"] += 1
                    else:
                        for side, q in (("l", left), ("r", right)):
                            if (side == "l" and not nl) or (side == "r" and not nr):
                                continue
                            if side == "l":
                                cut = max(q["i0"], q["i1"] - SLIDE + 1)
                                lab[cut:q["i1"] + 1] = -1
                                key = ("r", q["i0"])
                                rest = cut - q["i0"]
                            else:
                                cut = min(q["i1"], q["i0"] + SLIDE - 1)
                                lab[q["i0"]:cut + 1] = -1
                                key = ("l", q["i1"])
                                rest = q["i1"] - cut
                            sl["slides"] += 1
                            joint_slid[key] = joint_slid.get(key, 0) + 1
                            if (rest - 1) * STEP < MIN_RUN:
                                sl["gave_up"] += 1
                        changed = True
                        break
            new.append((a, S, c))
        if not changed:
            break
    sl["slide_m"] = [v * SLIDE * STEP for v in joint_slid.values()]
    st.update(sl)
    out = []
    for a, S, c in new:
        if len(S) < 2:     # degenerate (route of coincident samples): duplicate the point
            S, c = np.vstack([S, S]), np.r_[c, c]
            st["degenerate"] = st.get("degenerate", 0) + 1
        ch = cv.add_chain(S, np.clip(c, 0, 255).astype(np.uint8))
        ps[a].update(c=ch, jf=0, jl=len(S) - 1, d=1)
    for p in ps:
        out.append((p["c"], p["jf"], p["jl"], bool(p["m"])))
    return out, st


# ------------------------------------------------------------------------------------------------ graph
SAME_EPS = 0.01           # two chain samples are the same node only when they coincide (or are linked joints)


def node_ids(cv, keys, links):
    """keys: list of (chain, j) (unique). links: pairs of key indices that are one joint (consecutive pieces of a route).
    Union-find over the links and over exact coincidence (< 0.01 m). Returns node id per key, node x, z (position of
    the group's first key) and the greatest distance inside a group."""
    n = len(keys)
    par = list(range(n))

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            par[max(a, b)] = min(a, b)
    for a, b in links:
        union(a, b)
    xs = np.array([cv.x[cv.cs[c] + j] for c, j in keys])
    zs = np.array([cv.z[cv.cs[c] + j] for c, j in keys])
    grid = {}
    for i in range(n):
        ci, cj = int(math.floor(xs[i] / 0.05)), int(math.floor(zs[i] / 0.05))
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                for k in grid.get((ci + di, cj + dj), ()):
                    if math.hypot(xs[k] - xs[i], zs[k] - zs[i]) < SAME_EPS:
                        union(i, k)
        grid.setdefault((ci, cj), []).append(i)
    roots = [find(i) for i in range(n)]
    rid = {}
    nid = np.zeros(n, np.int64)
    for i, r_ in enumerate(roots):
        if r_ not in rid:
            rid[r_] = len(rid)
        nid[i] = rid[r_]
    nx = np.zeros(len(rid))
    nz = np.zeros(len(rid))
    for r_, k in rid.items():
        nx[k], nz[k] = xs[r_], zs[r_]
    spread = float(np.hypot(xs - nx[nid], zs - nz[nid]).max()) if n else 0.0
    return nid, nx, nz, spread


def merge_nodes(xs, zs):
    """Greedy: each registration joins the first node within 0.25 m, else makes a new node."""
    cell = NODE_MERGE
    grid = {}
    nid = np.zeros(len(xs), np.int64)
    nx, nz = [], []
    for i in range(len(xs)):
        x, z = xs[i], zs[i]
        ci, cj = int(math.floor(x / cell)), int(math.floor(z / cell))
        best, bd = -1, NODE_MERGE
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                for k in grid.get((ci + di, cj + dj), ()):
                    d = math.hypot(nx[k] - x, nz[k] - z)
                    if d < bd:
                        best, bd = k, d
        if best < 0:
            best = len(nx)
            nx.append(x)
            nz.append(z)
            grid.setdefault((ci, cj), []).append(best)
        nid[i] = best
    return nid, np.array(nx), np.array(nz)


def kappa_vec(P):
    """route31.curve_props curvature (three points 4 samples apart, ends 0), vectorized."""
    n = len(P)
    i = np.arange(n)
    a, b = np.maximum(0, i - 4), np.minimum(n - 1, i + 4)
    u, v, w = P - P[a], P[b] - P, P[b] - P[a]
    cr = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    den = np.hypot(*u.T) * np.hypot(*v.T) * np.hypot(*w.T)
    k = np.where(den > 1e-9, 2 * cr / np.maximum(den, 1e-300), 0.0)
    k[(a == i) | (b == i)] = 0.0
    return k


def chain_time(G, P):
    """Cumulative time and length along P by the route31.curve_props speed rule."""
    k = kappa_vec(P)
    Rr = np.where(np.abs(k) > 1e-6, 1 / np.maximum(np.abs(k), 1e-6), np.inf)
    tc = G.tcost_at(P[:, 0], P[:, 1])
    v = np.minimum(R.V_FREE / tc, np.sqrt(R.A_LAT * Rr))
    ds = np.hypot(*np.diff(P, axis=0).T)
    dt = ds / np.maximum(0.5 * (v[:-1] + v[1:]), 0.1)
    return np.r_[0.0, np.cumsum(dt)], np.r_[0.0, np.cumsum(ds)]


# ------------------------------------------------------------------------------------------------ file
def write_network_bin(path, NET, lib_routes, lib_samples):
    nn, ne = len(NET["nx"]), len(NET["ea"])
    adj = NET["adj"]
    tf, tn = NET["target_first"], NET["target_nodes"]
    with open(path, "wb") as f:
        f.write(HDR.pack(b"WTNET001", 1, nn, ne, len(NET["sx"]), len(adj), len(tf) - 1, len(tn), STEP, TOL_OUT,
                         lib_routes, lib_samples))
        nd = np.zeros(nn, dtype=[("x", "<f4"), ("z", "<f4"), ("af", "<u4"), ("an", "<u2"), ("fl", "<u2")])
        nd["x"], nd["z"], nd["af"], nd["an"], nd["fl"] = NET["nx"], NET["nz"], NET["adj_first"], NET["adj_n"], NET["nflags"]
        f.write(nd.tobytes())
        ed = np.zeros(ne, dtype=[("a", "<u4"), ("b", "<u4"), ("fs", "<u4"), ("ns", "<u4"), ("l", "<f4"), ("t", "<f4"),
                                 ("u", "<u2"), ("c", "u1"), ("f", "u1")])
        ed["a"], ed["b"], ed["fs"], ed["ns"] = NET["ea"], NET["eb"], NET["efirst"], NET["en"]
        ed["l"], ed["t"], ed["u"], ed["c"] = NET["elen"], NET["etime"], np.minimum(NET["euses"], 65535), NET["eclr"]
        f.write(ed.tobytes())
        f.write(np.asarray(adj, "<u4").tobytes())
        f.write(np.asarray(tf, "<u4").tobytes())
        f.write(np.asarray(tn, "<u4").tobytes())
        f.write(np.stack([NET["sx"], NET["sz"]], axis=1).astype("<f4").tobytes())
        f.write(np.asarray(NET["sclr"], np.uint8).tobytes())


def read_network_bin(path):
    b = open(path, "rb").read()
    magic, ver, nn, ne, ns, na, nt, ntn, step, tol, lr, ls = HDR.unpack_from(b, 0)
    assert magic == b"WTNET001" and ver == 1
    o = HDR.size
    nd = np.frombuffer(b, dtype=[("x", "<f4"), ("z", "<f4"), ("af", "<u4"), ("an", "<u2"), ("fl", "<u2")], count=nn, offset=o)
    o += 16 * nn
    ed = np.frombuffer(b, dtype=[("a", "<u4"), ("b", "<u4"), ("fs", "<u4"), ("ns", "<u4"), ("l", "<f4"), ("t", "<f4"),
                                 ("u", "<u2"), ("c", "u1"), ("f", "u1")], count=ne, offset=o)
    o += 28 * ne
    adj = np.frombuffer(b, "<u4", na, o)
    o += 4 * na
    tf = np.frombuffer(b, "<u4", nt + 1, o)
    o += 4 * (nt + 1)
    tn = np.frombuffer(b, "<u4", ntn, o)
    o += 4 * ntn
    xz = np.frombuffer(b, "<f4", 2 * ns, o).reshape(ns, 2)
    o += 8 * ns
    clr = np.frombuffer(b, np.uint8, ns, o)
    o += ns
    assert o == len(b), (o, len(b))
    return dict(n_nodes=nn, n_edges=ne, n_samples=ns, n_adj=na, n_targets=nt, n_target_nodes=ntn, step=step, tol=tol,
                lib_routes=lr, lib_samples=ls, nodes=nd, edges=ed, adj=adj, target_first=tf, target_nodes=tn, xz=xz,
                clr=clr, bytes=len(b))


# ------------------------------------------------------------------------------------------------ helpers for checks
def seg_dist(Q, A, B):
    AB = B - A
    l2 = (AB ** 2).sum(-1) + 1e-12
    t = np.clip(((Q - A) * AB).sum(-1) / l2, 0, 1)
    F = A + t[..., None] * AB
    return np.hypot(*(F - Q).T) if Q.ndim == 2 else np.hypot(F[..., 0] - Q[..., 0], F[..., 1] - Q[..., 1])


def max_dev(O, Rb, win=15.0):
    """Per original sample: distance to the rebuilt polyline (windowed by scaled arc length, exact fallback)."""
    if len(Rb) == 1:
        return np.hypot(*(O - Rb[0]).T)
    so = np.r_[0.0, np.cumsum(np.hypot(*np.diff(O, axis=0).T))]
    sr = np.r_[0.0, np.cumsum(np.hypot(*np.diff(Rb, axis=0).T))]
    A, B = Rb[:-1], Rb[1:]
    nseg = len(A)
    est = so * (sr[-1] / so[-1] if so[-1] > 0 else 1.0)
    lo = np.clip(np.searchsorted(sr, est - win) - 1, 0, nseg - 1)
    hi = np.clip(np.searchsorted(sr, est + win) + 1, 1, nseg)
    W = int((hi - lo).max())
    out = np.empty(len(O))
    for a in range(0, len(O), 2000):
        b = min(len(O), a + 2000)
        idx = np.minimum(lo[a:b, None] + np.arange(W)[None], nseg - 1)
        d = seg_dist(O[a:b, None, :], A[idx], B[idx])
        out[a:b] = d.min(axis=1)
    far = np.nonzero(out > 0.9)[0]
    for i in far:
        out[i] = seg_dist(O[i][None].repeat(nseg, 0), A, B).min()
    return out


def wrapr(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def arc(P):
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))] if len(P) > 1 else np.zeros(len(P))


# ------------------------------------------------------------------------------------------------ crossings
def find_crossings(Plist):
    """Proper intersections of segments of different edges at > 20 deg.
    Returns e1, p1 (segment index + t along it), e2, p2, X."""
    ne = len(Plist)
    nseg = np.array([max(len(P) - 1, 0) for P in Plist])
    A = np.vstack([P[:-1] for P in Plist if len(P) > 1])
    B = np.vstack([P[1:] for P in Plist if len(P) > 1])
    se = np.repeat(np.arange(ne), nseg)
    si = np.arange(len(se)) - np.repeat(np.cumsum(nseg) - nseg, nseg)
    mid = 0.5 * (A + B)
    ci = np.floor(mid[:, 0]).astype(np.int64)
    cj = np.floor(mid[:, 1]).astype(np.int64)
    key = (cj + 100000) * 1000003 + ci + 100000
    o = np.argsort(key)
    ks = key[o]
    out = [[], [], [], [], []]
    for dj, di in ((0, 0), (0, 1), (1, -1), (1, 0), (1, 1)):
        k2 = key + dj * 1000003 + di
        lo = np.searchsorted(ks, k2, "left")
        cnt = np.searchsorted(ks, k2, "right") - lo
        for a0 in range(0, len(key), 200000):
            a1 = min(len(key), a0 + 200000)
            c = cnt[a0:a1]
            I = np.repeat(np.arange(a0, a1), c)
            if not len(I):
                continue
            off = np.arange(len(I)) - np.repeat(np.cumsum(c) - c, c)
            J = o[np.repeat(lo[a0:a1], c) + off]
            m = (se[I] < se[J]) if (dj, di) == (0, 0) else (se[I] != se[J])
            I, J = I[m], J[m]
            if not len(I):
                continue
            p, r = A[I], B[I] - A[I]
            q, s = A[J], B[J] - A[J]
            rxs = r[:, 0] * s[:, 1] - r[:, 1] * s[:, 0]
            ok = np.abs(rxs) > 1e-12
            den = np.where(ok, rxs, 1.0)
            qp = q - p
            t = (qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]) / den
            u = (qp[:, 0] * r[:, 1] - qp[:, 1] * r[:, 0]) / den
            ang = np.abs(np.arctan2(rxs, (r * s).sum(1)))
            ang = np.minimum(ang, math.pi - ang)
            hit = ok & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1) & (ang > math.radians(CROSS_ANG_DEG))
            if hit.any():
                out[0].append(se[I[hit]])
                out[1].append(si[I[hit]] + t[hit])
                out[2].append(se[J[hit]])
                out[3].append(si[J[hit]] + u[hit])
                out[4].append(p[hit] + t[hit][:, None] * r[hit])
    if not out[0]:
        z = np.zeros(0)
        return z.astype(int), z, z.astype(int), z, np.zeros((0, 2))
    return (np.concatenate(out[0]), np.concatenate(out[1]), np.concatenate(out[2]), np.concatenate(out[3]),
            np.vstack(out[4]))


class PointGrid:
    def __init__(self, cell=1.0):
        self.cell, self.g = cell, {}

    def add(self, x, z):
        self.g.setdefault((int(math.floor(x / self.cell)), int(math.floor(z / self.cell))), []).append((x, z))

    def near(self, x, z, r):
        ci, cj = int(math.floor(x / self.cell)), int(math.floor(z / self.cell))
        k = int(math.ceil(r / self.cell))
        for dj in range(-k, k + 1):
            for di in range(-k, k + 1):
                for (px, pz) in self.g.get((ci + di, cj + dj), ()):
                    if math.hypot(px - x, pz - z) < r:
                        return True
        return False


def resample_piece(V, vc, vs):
    """Uniform resample of polyline V (clearance vc, original arc vs per vertex). Returns points, clr, original arc."""
    seg = np.hypot(*np.diff(V, axis=0).T)
    keep = np.r_[True, seg > 1e-6]
    V, vc, vs = V[keep], vc[keep], vs[keep]
    s = arc(V)
    L = s[-1]
    ns = max(1, int(round(L / STEP)))
    t = np.linspace(0, L, ns + 1)
    Q = np.stack([np.interp(t, s, V[:, 0]), np.interp(t, s, V[:, 1])], axis=1)
    Q[0], Q[-1] = V[0], V[-1]
    k = np.clip(np.searchsorted(s, t, side="right") - 1, 0, len(s) - 2)
    return Q, np.minimum(vc[k], vc[k + 1]), np.interp(t, s, vs)


def split_crossings(E, nx, nz):
    """E: dict of lists a, b, P, clr, T (cumulative time per sample). Crossings (> 20 deg, no node within 1.0 m,
    taken in edge order) become nodes; both edges are split there and the pieces resampled uniformly on their own
    polyline (the crossing point is the end sample). Returns new E, piece map, nx, nz, stats."""
    e1, p1, e2, p2, X = find_crossings(E["P"])
    nfound = len(X)
    order = np.lexsort((p1, e1))
    grid = PointGrid(1.0)
    for x, z in zip(nx, nz):
        grid.add(x, z)
    nx, nz = list(nx), list(nz)
    cuts = {}
    skipped = 0
    used = {}
    bad_idx = 0
    shallow = 0
    offs = []
    for k in order:
        x, z = float(X[k, 0]), float(X[k, 1])
        if grid.near(x, z, CROSS_NODE_DIST):
            skipped += 1
            continue
        ea_, eb_ = int(e1[k]), int(e2[k])
        PA, PB = E["P"][ea_], E["P"][eb_]
        iA = int(p1[k]) + (1 if p1[k] - int(p1[k]) >= 0.5 else 0)
        iB = int(p2[k]) + (1 if p2[k] - int(p2[k]) >= 0.5 else 0)
        if not (0 < iA < len(PA) - 1 and 0 < iB < len(PB) - 1) or iA in used.get(ea_, ()) or iB in used.get(eb_, ()):
            bad_idx += 1
            continue
        # every transfer A <-> B must be a counted turn by the turn rule's own 2 m headings (> 30 deg)
        kw = int(round(HEAD_WIN / STEP))

        def hd(P, i, d):
            j = min(max(i + d * kw, 0), len(P) - 1)
            return math.atan2(P[j, 1] - P[i, 1], P[j, 0] - P[i, 0])
        arr = [hd(PA, iA, -1) + math.pi, hd(PA, iA, 1) + math.pi]       # arriving over A (forward / backward)
        lvB = [hd(PB, iB, 1), hd(PB, iB, -1)]
        arrB = [hd(PB, iB, -1) + math.pi, hd(PB, iB, 1) + math.pi]
        lvA = [hd(PA, iA, 1), hd(PA, iA, -1)]
        tmin = min([abs(math.degrees(wrapr(o - i))) for i in arr for o in lvB] +
                   [abs(math.degrees(wrapr(o - i))) for i in arrB for o in lvA])
        if tmin <= TURN_FREE_DEG + 0.5:
            shallow += 1
            continue
        n = len(nx)
        nx.append(float(PA[iA, 0]))          # the node is chain A's own sample
        nz.append(float(PA[iA, 1]))
        grid.add(nx[-1], nz[-1])
        used.setdefault(ea_, set()).add(iA)
        used.setdefault(eb_, set()).add(iB)
        cuts.setdefault(ea_, []).append((iA, n))
        cuts.setdefault(eb_, []).append((iB, n))
        offs.append(math.hypot(PB[iB, 0] - PA[iA, 0], PB[iB, 1] - PA[iA, 1]))
    F = dict(a=[], b=[], P=[], clr=[], T=[])
    pmap = []
    for e in range(len(E["P"])):
        if e not in cuts:
            pmap.append([len(F["a"])])
            for k_ in F:
                F[k_].append(E[k_][e])
            continue
        P, clr, T = E["P"][e], E["clr"][e], E["T"][e]
        cs = sorted(cuts[e])
        bi = [0] + [i for i, _ in cs] + [len(P) - 1]
        bn = [E["a"][e]] + [n for _, n in cs] + [E["b"][e]]
        ids = []
        for q in range(len(bi) - 1):
            i0, i1 = bi[q], bi[q + 1]
            ids.append(len(F["a"]))
            F["a"].append(bn[q])
            F["b"].append(bn[q + 1])
            F["P"].append(P[i0:i1 + 1].copy())
            F["clr"].append(clr[i0:i1 + 1].copy())
            F["T"].append(T[i0:i1 + 1] - T[i0])
        pmap.append(ids)
    return F, pmap, np.array(nx), np.array(nz), skipped, nfound, (bad_idx, shallow), (max(offs) if offs else 0.0)


# ------------------------------------------------------------------------------------------------ turn-rule search
class Graph:
    """Search state = (edge, f): the edge traversed in direction f (1 = a -> b), standing at its end node.
    Turn at a node = angle between the arrival heading and the leaving heading, each over the 2 m of the edge next to
    the node. <= 30 deg: free; > 30 deg: only at a pivot-free node, cost angle / START_TURN_DEG_S (route31, 15 deg/s)."""

    def __init__(self, NET, pivot):
        self.N = NET
        self.ea, self.eb, self.et, self.el = NET["ea"], NET["eb"], NET["etime"], NET["elen"]
        self.nn = len(NET["nx"])
        self.pivot = pivot
        ne = len(self.ea)
        self.out = [[] for _ in range(self.nn)]
        for e in range(ne):
            self.out[self.ea[e]].append((e, 1))
            self.out[self.eb[e]].append((e, 0))
        self.h_out = np.zeros((ne, 2))
        self.h_in = np.zeros((ne, 2))
        kw = int(round(HEAD_WIN / STEP))
        for e in range(ne):
            P = edge_pts(NET, e)
            n = len(P)
            k = min(n - 1, kw)
            if k < 1:
                continue
            h0 = math.atan2(P[k, 1] - P[0, 1], P[k, 0] - P[0, 0])
            h1 = math.atan2(P[-1, 1] - P[-1 - k, 1], P[-1, 0] - P[-1 - k, 0])
            self.h_out[e, 1], self.h_in[e, 1] = h0, h1
            self.h_out[e, 0], self.h_in[e, 0] = h1 + math.pi, h0 + math.pi

    def end(self, e, f):
        return int(self.eb[e] if f else self.ea[e])

    def turn(self, n, e, f, e2, f2):
        th = abs(math.degrees(wrapr(self.h_out[e2, f2] - self.h_in[e, f])))
        if th <= TURN_FREE_DEG:
            return 0.0, th
        if self.pivot[n]:
            return th / R.START_TURN_DEG_S, th
        return None, th

    def search(self, srcs, goal_fn=None, finals=()):
        """srcs [(e, f, time, length)], already at the end node of e. goal_fn(e, f) -> None or (dt, dl, tag).
        finals [(time, length, tag)]. Returns (time, length, [(e, f)...], tag) or None; with goal_fn None the
        whole reachable state set {(e, f): (time, length)}."""
        dist, pred, hp = {}, {}, []
        cnt = [0]

        def push(t, l, e, f, fin):
            cnt[0] += 1
            heapq.heappush(hp, (t, cnt[0], l, e, f, fin))
        for t, l, tag in finals:
            push(t, l, -1, -1, (None, tag))
        for e, f, t, l in srcs:
            if t < dist.get((e, f), (np.inf,))[0]:
                dist[(e, f)] = (t, l)
                pred[(e, f)] = None
                push(t, l, e, f, None)
        done = set()
        while hp:
            t, _, l, e, f, fin = heapq.heappop(hp)
            if e < 0:
                seq, s = [], fin[0]
                while s is not None:
                    seq.append(s)
                    s = pred[s]
                return t, l, seq[::-1], fin[1]
            if (e, f) in done:
                continue
            done.add((e, f))
            if goal_fn is not None:
                g = goal_fn(e, f)
                if g is not None:
                    push(t + g[0], l + g[1], -1, -1, ((e, f), g[2]))
            n = self.end(e, f)
            for e2, f2 in self.out[n]:
                c, _ = self.turn(n, e, f, e2, f2)
                if c is None:
                    continue
                nt, nl = t + c + self.et[e2], l + self.el[e2]
                if nt < dist.get((e2, f2), (np.inf,))[0]:
                    dist[(e2, f2)] = (nt, nl)
                    pred[(e2, f2)] = (e, f)
                    push(nt, nl, e2, f2, None)
        return dist if goal_fn is None else None

    def turns_of(self, seq):
        """[(node, angle)] of the transitions in seq (all of them, counted or not)."""
        return [(self.end(e, f), self.turn(self.end(e, f), e, f, e2, f2)[1]) for (e, f), (e2, f2) in zip(seq[:-1], seq[1:])]


def edge_pts(NET, e, forward=True):
    a = NET["efirst"][e]
    P = np.stack([NET["sx"][a:a + NET["en"][e]], NET["sz"][a:a + NET["en"][e]]], axis=1).astype(np.float64)
    return P if forward else P[::-1]


def assemble(parts):
    """Concatenate pieces sharing their end / start sample; returns points and the joint indices."""
    out, joints, n0 = [], [], 0
    for i, P in enumerate(parts):
        if i:
            P = P[1:]
            joints.append(n0 - 1)
        out.append(P)
        n0 += len(P)
    return (np.vstack(out) if out else np.zeros((0, 2))), joints


def path_steps(P, joints, pivot_joints):
    """Heading steps > 12 deg of a composed path, split into: at a counted turn (pivot joint), at another joint,
    inside an edge. dh[i] is the change at point i + 1."""
    if len(P) < 3:
        return dict(maxh=0.0, n_pivot=0, n_joint=0, n_inside=0, max_other=0.0)
    h = heads(P)[:-1]
    dh = np.abs(wrapd(np.degrees(np.diff(h))))
    big = np.nonzero(dh > MAX_STEP_DEG)[0]
    pj = set()
    for j in pivot_joints:
        pj.update((j - 2, j - 1, j))
    oj = set()
    for j in joints:
        oj.update((j - 2, j - 1, j))
    r = dict(maxh=float(dh.max()), n_pivot=0, n_joint=0, n_inside=0, max_other=0.0)
    for i in big:
        if i in pj:
            r["n_pivot"] += 1
        else:
            r["max_other"] = max(r["max_other"], float(dh[i]))
            r["n_joint" if i in oj else "n_inside"] += 1
    return r


def pivot_sweep(G, x, z, h0, h1):
    """Bare hull turned on the spot at (x, z) from h0 to h1 (the short way), 3 deg steps: touches a hard cell?"""
    U, V = R.lattice(R.HL, R.HW)
    d = wrapr(h1 - h0)
    ns = max(1, int(math.ceil(abs(d) / math.radians(3.0))))
    for s in range(ns + 1):
        h = h0 + d * s / ns
        X = x + U * math.cos(h) - V * math.sin(h)
        Z = z + U * math.sin(h) + V * math.cos(h)
        c = np.clip(np.floor((X - G.X0) / R.CELL).astype(np.int64), 0, G.W - 1)
        r = np.clip(np.floor((Z - G.Z0) / R.CELL).astype(np.int64), 0, G.H - 1)
        if G.hard[r, c].any():
            return True
    return False


def run_query(Gr, NET, ecum_t, ecum_l, sample_edge, sample_k, st_, goal, targets, tid, tsets):
    """Pure network mode with the turn rule. Entry: nearest network sample, both directions, no turn cost."""
    sx, sz = NET["sx"], NET["sz"]
    d_all = np.hypot(sx - st_[0], sz - st_[1])
    i = int(np.argmin(d_all))
    e, k = int(sample_edge[i]), int(sample_k[i])
    Te, Le = ecum_t[e], ecum_l[e]
    srcs = [(e, 1, float(Te[-1] - Te[k]), float(Le[-1] - Le[k])), (e, 0, float(Te[k]), float(Le[k]))]
    res = dict(d_in=float(d_all[i]), entry=f"{e}:{k}", P=None, pivots=[], nodes=[], reached=False, time=float("nan"),
               length=float("nan"), nedges=0, d_out=float("nan"), exit="-")
    E1 = edge_pts(NET, e)
    if goal[0] == "t":
        ti = tid[goal[1]]
        t = targets[ti]
        gp = (t[7], t[8]) if t[1] == 1 else (t[5], t[6])
        res["goal_s"] = f"target:{goal[1]}({gp[0]:.1f},{gp[1]:.1f})"
        res["straight"] = math.hypot(gp[0] - st_[0], gp[1] - st_[1])
        S = tsets[ti]
        r = Gr.search(srcs, lambda e_, f_: (0.0, 0.0, None) if Gr.end(e_, f_) in S else None)
        if r is None:
            return res
        tt, ll, seq, _ = r
        hit = Gr.end(*seq[-1])
        parts = [E1[k:] if seq[0][1] == 1 else E1[:k + 1][::-1]] + [edge_pts(NET, e_, f_ == 1) for e_, f_ in seq[1:]]
        P, joints = assemble(parts)
        tr = Gr.turns_of(seq)
        res.update(reached=True, time=tt, length=ll, nedges=len(seq), exit=f"node:{hit}",
                   d_out=math.hypot(NET["nx"][hit] - gp[0], NET["nz"][hit] - gp[1]), nodes=[Gr.end(*s) for s in seq])
    else:
        gp = goal[1]
        res["goal_s"] = f"point({gp[0]:.1f},{gp[1]:.1f})"
        res["straight"] = math.hypot(gp[0] - st_[0], gp[1] - st_[1])
        d2 = np.hypot(sx - gp[0], sz - gp[1])
        j = int(np.argmin(d2))
        e2, k2 = int(sample_edge[j]), int(sample_k[j])
        a2, b2 = int(NET["ea"][e2]), int(NET["eb"][e2])
        T2, L2 = ecum_t[e2], ecum_l[e2]
        res.update(d_out=float(d2[j]), exit=f"{e2}:{k2}")

        def gf(e_, f_):
            if e_ == e2:
                return None
            n = Gr.end(e_, f_)
            best = None
            for node, f2, dt, dl in ((a2, 1, T2[k2], L2[k2]), (b2, 0, T2[-1] - T2[k2], L2[-1] - L2[k2])):
                if n != node:
                    continue
                c, th = Gr.turn(n, e_, f_, e2, f2)
                if c is not None and (best is None or c + dt < best[0]):
                    best = (c + float(dt), float(dl), (f2, n, th))
            return best
        fin = [(abs(float(Te[k2] - Te[k])), abs(float(Le[k2] - Le[k])), "same")] if e2 == e else []
        r = Gr.search(srcs, gf, finals=fin)
        if r is None:
            return res
        tt, ll, seq, tag = r
        if tag == "same":
            res.update(reached=True, time=tt, length=ll, nedges=1, P=E1[min(k, k2):max(k, k2) + 1])
            return res
        f2, n_ex, th_ex = tag
        E2 = edge_pts(NET, e2)
        parts = [E1[k:] if seq[0][1] == 1 else E1[:k + 1][::-1]] + [edge_pts(NET, e_, f_ == 1) for e_, f_ in seq[1:]]
        parts.append(E2[:k2 + 1] if f2 == 1 else E2[k2:][::-1])
        P, joints = assemble(parts)
        tr = Gr.turns_of(seq) + [(n_ex, th_ex)]
        res.update(reached=True, time=tt, length=ll, nedges=len(seq) + 1, nodes=[Gr.end(*s) for s in seq])
    res["P"] = P
    res["pivots"] = [(n, th) for (n, th) in tr if th > TURN_FREE_DEG]
    pj = [joints[q] for q, (n, th) in enumerate(tr) if th > TURN_FREE_DEG]
    res["steps"] = path_steps(P, joints, pj)
    res["turn_list"] = tr
    return res


# ------------------------------------------------------------------------------------------------ loader checks
def loader_checks(RN):
    nd, ed = RN["nodes"], RN["edges"]
    ne, nn = RN["n_edges"], RN["n_nodes"]
    out = {}
    fs, ns = ed["fs"].astype(np.int64), ed["ns"].astype(np.int64)
    out["samples_in_one_edge"] = bool(fs[0] == 0 and np.all(fs[1:] == fs[:-1] + ns[:-1]) and fs[-1] + ns[-1] == RN["n_samples"])
    out["edges_with_lt2_samples"] = int((ns < 2).sum())
    out["n_adj_eq_2ne"] = RN["n_adj"] == 2 * ne
    seen = np.zeros((ne, 2), np.int64)
    bad = 0
    for i in range(nn):
        for v in RN["adj"][nd["af"][i]:nd["af"][i] + nd["an"][i]]:
            e, end = int(v) >> 1, int(v) & 1
            seen[e, end] += 1
            bad += int((ed["b"][e] if end else ed["a"][e]) != i)
    out["edge_ends_listed_once"] = bool((seen == 1).all()) and bad == 0
    xz = RN["xz"].astype(np.float64)
    lerr = 0
    dend = 0.0
    for e in range(ne):
        P = xz[fs[e]:fs[e] + ns[e]]
        L = arc(P)[-1]
        lerr += int(abs(ed["l"][e] - L) > 0.02 * L + 0.5)
        a, b = int(ed["a"][e]), int(ed["b"][e])
        dend = max(dend, math.hypot(P[0, 0] - nd["x"][a], P[0, 1] - nd["z"][a]), math.hypot(P[-1, 0] - nd["x"][b], P[-1, 1] - nd["z"][b]))
    out["length_mismatch_edges"] = lerr
    out["greatest_end_sample_to_node_m"] = round(dend, 4)
    out["end_sample_within_tol"] = dend <= RN["tol"] + 0.05
    tf = RN["target_first"].astype(np.int64)
    out["target_offsets_monotonic"] = bool(tf[0] == 0 and np.all(np.diff(tf) >= 0) and tf[-1] == RN["n_target_nodes"])
    out["target_nodes_without_bit0"] = int(((nd["fl"][RN["target_nodes"]] & 1) == 0).sum())
    return out


# ------------------------------------------------------------------------------------------------ main
def main():
    mapname = sys.argv[1] if len(sys.argv) > 1 else "eastern_europe_02_dom"
    limit = int(sys.argv[sys.argv.index("--limit") + 1]) if "--limit" in sys.argv else None
    for a in sys.argv:
        if a.startswith("--opt="):          # e.g. --opt=shape:linear,own_two:6,shared_eps:0.25
            for kv in a[6:].split(","):
                k_, v_ = kv.split(":")
                OPT[k_] = v_ if k_ == "shape" else float(v_)
    T0 = time.time()
    # optional: --lib <pack folder holding routes.bin, map.bin, cache\> and --out <output folder>; default out/<map> for both
    LIB = Path(sys.argv[sys.argv.index("--lib") + 1]) if "--lib" in sys.argv else paths.OUT / mapname
    OUT = Path(sys.argv[sys.argv.index("--out") + 1]) if "--out" in sys.argv else paths.OUT / mapname
    OUT.mkdir(parents=True, exist_ok=True)
    RB = BV.read_routes_bin(LIB / "routes.bin")
    M = BV.read_map_bin(LIB / "map.bin")
    G = R.Geo(LIB / "cache")
    hard_same = (G.hard.shape == M["hard"].shape and abs(G.X0 - M["x0"]) < 1e-3 and abs(G.Z0 - M["z0"]) < 1e-3
                 and bool(np.array_equal(G.hard, M["hard"])))
    G.hard = M["hard"]                       # the hull check runs on map.bin's hard cells
    lib_routes = RB["routes"][:limit] if limit else RB["routes"]
    lib_samples = int(sum(len(r["xz"]) for r in lib_routes))
    degen = [r for r in lib_routes if arc(r["xz"].astype(np.float64))[-1] < NODE_MERGE]
    routes = [r for r in lib_routes if arc(r["xz"].astype(np.float64))[-1] >= NODE_MERGE]
    targets = RB["targets"]
    log(f"library: {len(lib_routes)} routes ({len(degen)} shorter than {NODE_MERGE} m skipped), {lib_samples} samples")
    ext = M["hard"].shape[1] * M["cell"]
    cv = Canon(M["x0"], M["z0"], ext, lib_samples + 100000)
    # ------------------------------------------------ 1-2 chains
    rpieces, stt = [], {}
    for k, r in enumerate(routes):
        pcs, st = process_route(cv, r, G)
        rpieces.append(pcs)
        for a, b in st.items():
            stt[a] = stt.get(a, [] if isinstance(b, list) else 0) + b
        if k % 500 == 0:
            log(f"route {k}/{len(routes)}: canonical {cv.n}, chains {cv.nc}")
    log(f"chains: {cv.nc}, canonical samples {cv.n}; settle stats " +
        str({a: (b if not isinstance(b, list) else len(b)) for a, b in stt.items()}))
    # ------------------------------------------------ 3 nodes
    rc, rj, rfl, rtg = [], [], [], []
    for k, r in enumerate(routes):
        pcs = rpieces[k]
        for (c, jf, jl, m) in pcs:
            for j in (jf, jl):
                rc.append(c), rj.append(j), rfl.append(0), rtg.append(-1)
        c, jf = pcs[0][0], pcs[0][1]
        if r["kind"] in (0, 1):
            rc.append(c), rj.append(jf), rfl.append(2), rtg.append(-1)
        else:
            rc.append(c), rj.append(jf), rfl.append(1), rtg.append(r["start_target"])
        c, jl = pcs[-1][0], pcs[-1][2]
        rc.append(c), rj.append(jl), rfl.append(1), rtg.append(r["end_target"])
    for c in range(cv.nc):
        for j in (0, cv.cn[c] - 1):
            rc.append(c), rj.append(int(j)), rfl.append(0), rtg.append(-1)
    rc, rj = np.array(rc), np.array(rj)
    kidx = {}
    for i in range(len(rc)):
        kidx.setdefault((int(rc[i]), int(rj[i])), len(kidx))
    keys = list(kidx)
    links = []
    link_d = 0.0
    for pcs in rpieces:
        for (c1, _, jl1, _), (c2, jf2, _, _) in zip(pcs[:-1], pcs[1:]):
            links.append((kidx[(c1, jl1)], kidx[(c2, jf2)]))
            link_d = max(link_d, float(np.hypot(*(cv.pts(c1, jl1) - cv.pts(c2, jf2)))))
    knid, nx, nz, spread = node_ids(cv, keys, links)
    nid = np.array([knid[kidx[(int(rc[i]), int(rj[i]))]] for i in range(len(rc))])
    n_route_nodes = len(nx)
    node_of = {key: int(knid[i]) for i, key in enumerate(keys)}
    merge_off = np.array([spread])
    nmerged = 0
    dup = {}
    for (c, j), n in node_of.items():
        dup.setdefault((c, n), set()).add(j)
    multi = sum(1 for v in dup.values() if len(v) > 1)
    # ------------------------------------------------ 4 edges (chain pieces)
    bps = {}
    for (c, j) in node_of:
        bps.setdefault(c, set()).add(j)
    E = dict(a=[], b=[], P=[], clr=[], T=[])
    chain_edges = {}
    for c in range(cv.nc):
        bp = sorted(bps[c])
        s0, n0 = int(cv.cs[c]), int(cv.cn[c])
        P = np.stack([cv.x[s0:s0 + n0], cv.z[s0:s0 + n0]], axis=1)
        T, S = chain_time(G, P)
        ids = []
        for a, b in zip(bp[:-1], bp[1:]):
            ids.append(len(E["a"]))
            E["a"].append(node_of[(c, a)])
            E["b"].append(node_of[(c, b)])
            E["P"].append(P[a:b + 1].copy())
            E["clr"].append(cv.clr[s0 + a:s0 + b + 1].copy())
            E["T"].append(T[a:b + 1] - T[a])
        chain_edges[c] = (np.array(bp), ids)
    redges0, cont_fail, cont_ids = [], 0, []
    for k, r in enumerate(routes):
        seq = []
        for (c, ja, jb, m) in rpieces[k]:
            bp, ids = chain_edges[c]
            p, q = int(np.searchsorted(bp, ja)), int(np.searchsorted(bp, jb))
            if ja < jb:
                seq += [(ids[i], True) for i in range(p, q)]
            elif ja > jb:
                seq += [(ids[i], False) for i in range(p - 1, q - 1, -1)]
        bad = sum(int((E["b"][e1] if f1 else E["a"][e1]) != (E["a"][e2] if f2 else E["b"][e2]))
                  for (e1, f1), (e2, f2) in zip(seq[:-1], seq[1:]))
        if bad:
            cont_fail += bad
            cont_ids.append(r["id"])
        redges0.append(seq)
    n_chain_edges = len(E["a"])
    # ------------------------------------------------ crossings -> nodes
    E, pmap, nx, nz, cross_skipped, cross_found, cross_badidx, cross_off = split_crossings(E, nx, nz)
    n_cross_nodes = len(nx) - n_route_nodes
    redges = []
    for seq in redges0:
        s2 = []
        for e, f in seq:
            s2 += [(q, True) for q in pmap[e]] if f else [(q, False) for q in pmap[e][::-1]]
        redges.append(s2)
    log(f"nodes {len(nx)} ({n_cross_nodes} crossing nodes; crossings found {cross_found}, skipped {cross_skipped}); "
        f"edges {n_chain_edges} -> {len(E['a'])}")
    nn, ne = len(nx), len(E["a"])
    nflags = np.zeros(nn, np.int64)
    np.bitwise_or.at(nflags, nid, np.array(rfl))
    # pivot free: map.bin clearance of the 0.5 m cell at the node >= 4.3 m
    c5 = np.clip(np.floor((nx - M["x0"]) / M["cell05"]).astype(np.int64), 0, M["clr05"].shape[1] - 1)
    r5 = np.clip(np.floor((nz - M["z0"]) / M["cell05"]).astype(np.int64), 0, M["clr05"].shape[0] - 1)
    node_clr = M["clr05"][r5, c5].astype(np.int64)
    pivot = node_clr >= int(round(PIVOT_CLR * 10))
    nflags |= np.where(pivot, 4, 0)
    # ------------------------------------------------ arrays
    ea, eb = np.array(E["a"]), np.array(E["b"])
    en = np.array([len(P) for P in E["P"]])
    efirst = np.r_[0, np.cumsum(en)[:-1]]
    ecum_t = E["T"]
    ecum_l = [arc(P) for P in E["P"]]
    uses = np.zeros(ne, np.int64)
    for seq in redges:
        for e in {e for e, _ in seq}:
            uses[e] += 1
    NET = dict(nx=nx, nz=nz, nflags=nflags, ea=ea, eb=eb, efirst=efirst, en=en,
               elen=np.array([L[-1] for L in ecum_l]), etime=np.array([T[-1] for T in ecum_t]),
               eclr=np.array([min(255, int(c.min())) for c in E["clr"]]),
               sx=np.concatenate([P[:, 0] for P in E["P"]]), sz=np.concatenate([P[:, 1] for P in E["P"]]),
               sclr=np.concatenate(E["clr"]), euses=uses)
    inc = [[] for _ in range(nn)]
    for e in range(ne):
        inc[ea[e]].append(e << 1)
        inc[eb[e]].append((e << 1) | 1)
    adj_first, adj_n, adj = np.zeros(nn, np.int64), np.zeros(nn, np.int64), []
    for i in range(nn):
        adj_first[i], adj_n[i] = len(adj), len(inc[i])
        adj += inc[i]
    assert adj_n.max() < 65536
    NET.update(adj=adj, adj_first=adj_first, adj_n=adj_n)
    tsets = [set() for _ in targets]
    for i in range(len(rc)):
        if rtg[i] >= 0:
            tsets[rtg[i]].add(int(nid[i]))
    tfirst, tnodes = [0], []
    for s in tsets:
        tnodes += sorted(s)
        tfirst.append(len(tnodes))
    NET.update(target_first=tfirst, target_nodes=tnodes)
    fn = OUT / f"{mapname}.network.bin"
    write_network_bin(fn, NET, len(lib_routes), lib_samples)
    t_write = time.time() - T0
    RN = read_network_bin(fn)
    assert RN["n_nodes"] == nn and RN["n_edges"] == ne and RN["n_samples"] == len(NET["sx"])
    LC = loader_checks(RN)
    log(f"wrote {fn.name}: {RN['bytes']} bytes; loader checks {LC}")
    rep = []

    def P_(s=""):
        rep.append(s)
        print(s, flush=True)
    P_(f"map {mapname}; library {LIB / 'routes.bin'}")
    P_(f"routes {len(lib_routes)} of {len(RB['routes'])}; library samples {lib_samples}; routes shorter than {NODE_MERGE} m "
       f"(not in the network) {len(degen)}: {' '.join(r['id'] for r in degen)}")
    P_(f"params TOL_IN {TOL_IN} ANG_IN 15 TOL_OUT {TOL_OUT} ANG_OUT 20 MIN_RUN {MIN_RUN} BLEND {BLEND} NODE_MERGE {NODE_MERGE} "
       f"SWITCH_K {SWITCH_K} SHARED_BACK {SHARED_BACK} SHARED_FWD {SHARED_FWD} OWN_MIN_ONE {OWN_MIN_ONE} "
       f"blend shape {OPT['shape']} own_two {OPT['own_two']} shared_eps {OPT['shared_eps']}; TURN_FREE {TURN_FREE_DEG} "
       f"PIVOT_CLR {PIVOT_CLR} START_TURN_DEG_S {R.START_TURN_DEG_S} CROSS {CROSS_ANG_DEG} deg / {CROSS_NODE_DIST} m HEAD_WIN {HEAD_WIN}")
    P_(f"cache hard == map.bin hard: {hard_same}")
    P_()
    P_("a. counts")
    P_(f"nodes {nn} (route / chain nodes {n_route_nodes}, crossing nodes {n_cross_nodes}); edges {ne} (before crossing split "
       f"{n_chain_edges}); chains {cv.nc}; canonical samples {cv.n}; file samples {len(NET['sx'])}; adj {len(adj)}; "
       f"targets {len(targets)}; target nodes {len(tnodes)}")
    P_(f"node flags: target end {int((nflags & 1 > 0).sum())}, spawn pose {int((nflags & 2 > 0).sum())}, pivot free "
       f"{int(pivot.sum())} (crossing nodes pivot free {int(pivot[n_route_nodes:].sum())})")
    P_(f"compression canonical/library {cv.n / lib_samples:.4f} ({lib_samples / cv.n:.2f}x); file samples/library "
       f"{len(NET['sx']) / lib_samples:.4f}")
    P_(f"network.bin bytes {RN['bytes']}; build + write s {t_write:.1f}")
    P_(f"settle: matched runs < MIN_RUN unmatched {stt.get('short', 0)}; chain switches with shared node {stt.get('switch_shared', 0)}; "
       f"switches without (own window) {stt.get('switch_window', 0)}; own pieces grown {stt.get('own_grow', 0)}")
    sm = np.array(stt.get("slide_m", []) or [0.0])
    P_(f"hull-swept blends: joint slides {stt.get('slides', 0)} (4 m each) at {len(stt.get('slide_m', []))} joints, slid "
       f"distance per joint m: p50 {np.median(sm):.1f} max {sm.max():.1f}; matched runs given up {stt.get('gave_up', 0)}; "
       f"own pieces touching away from any joint (kept, cannot slide) {stt.get('unfixable', 0)}")
    P_(f"node registrations {len(rc)} on {len(keys)} chain samples; joint links {len(links)}, greatest link distance "
       f"{link_d:.4f} m; greatest distance inside a node {spread:.4f} m; same node at two samples of one chain {multi}; "
       f"route piece continuity failures {cont_fail} {' '.join(cont_ids[:10])}")
    el = NET["elen"]
    P_(f"edge length m: min {el.min():.2f} p50 {np.median(el):.2f} max {el.max():.2f}; edges < 1 m {int((el < 1).sum())}; "
       f"uses: max {uses.max()} p50 {int(np.median(uses))}; unused edges {int((uses == 0).sum())}; min clr byte {int(NET['eclr'].min())}")
    P_(f"node degree: 1 {int((adj_n == 1).sum())}, 2 {int((adj_n == 2).sum())}, 3 {int((adj_n == 3).sum())}, "
       f">=4 {int((adj_n >= 4).sum())}")
    P_("loader checks: " + ", ".join(f"{k} {v}" for k, v in LC.items()))
    # ------------------------------------------------ b, c, time
    P_()
    devs, worst, ratios, lratios, pose_off = [], [], [], [], []
    hmax, smin, smax = 0.0, 9.0, 0.0
    nviol_h = nviol_s = inh_h = inh_s = 0
    viol_where = []
    for k, r in enumerate(routes):
        seq = redges[k]
        Rb, _ = assemble([edge_pts(NET, e, f) for e, f in seq])
        O = r["xz"].astype(np.float64)
        d = max_dev(O, Rb)
        devs.append(float(d.max()))
        worst.append((float(d.max()), r["id"], int(d.argmax()), len(O)))
        if r["kind"] in (0, 1):
            pose_off.append(math.hypot(Rb[0, 0] - O[0, 0], Rb[0, 1] - O[0, 1]))
        ratios.append(sum(NET["etime"][e] for e, _ in seq) / r["drive_s"] if r["drive_s"] > 0 else np.nan)
        lratios.append(arc(Rb)[-1] / max(arc(O)[-1], 1e-9))
        ds = np.hypot(*np.diff(Rb, axis=0).T)
        h = heads(Rb)[:-1]
        dh = np.abs(wrapd(np.degrees(np.diff(h)))) if len(h) > 1 else np.zeros(0)
        hmax = max(hmax, float(dh.max()) if len(dh) else 0.0)
        smin, smax = min(smin, float(ds.min())), max(smax, float(ds.max()))
        dso = np.hypot(*np.diff(O, axis=0).T)
        ho = heads(O)[:-1]
        dho = np.abs(wrapd(np.degrees(np.diff(ho)))) if len(ho) > 1 else np.zeros(0)
        for kind, bad in (("h", np.nonzero(dh > MAX_STEP_DEG)[0]), ("s", np.nonzero(np.abs(ds - STEP) > SPACE_TOL)[0])):
            for i in bad:
                pt = Rb[i + 1] if kind == "h" else 0.5 * (Rb[i] + Rb[i + 1])
                oi = int(np.argmin(np.hypot(*(O - pt).T)))
                lo, hi = max(0, oi - 4), oi + 4
                inh = (dho[lo:hi] > MAX_STEP_DEG).any() if kind == "h" else (np.abs(dso[lo:hi] - STEP) > SPACE_TOL).any()
                if kind == "h":
                    nviol_h, inh_h = nviol_h + 1, inh_h + int(inh)
                else:
                    nviol_s, inh_s = nviol_s + 1, inh_s + int(inh)
                if not inh:
                    viol_where.append((kind, r["id"], int(i), len(Rb), float(dh[i]) if kind == "h" else float(ds[i]),
                                       float(pt[0]), float(pt[1])))
    devs, ratios, lratios = np.array(devs), np.array(ratios), np.array(lratios)
    P_("b. replay (greatest distance original sample -> rebuilt curve, per route)")
    P_(f"routes {len(devs)}; max {devs.max():.3f} m; p99 {np.percentile(devs, 99):.3f}; p50 {np.median(devs):.3f}; "
       f"routes > 1.0 m {int((devs > 1.0).sum())}; > 0.8 m {int((devs > 0.8).sum())}")
    worst.sort(reverse=True)
    for w in worst[:3]:
        P_(f"  worst {w[1]} {w[0]:.3f} m at sample {w[2]}/{w[3]}")
    P_(f"spawn routes: first rebuilt sample -> pose: max {max(pose_off):.3f} m, > 0.01 m {sum(1 for x in pose_off if x > 0.01)}")
    P_()
    P_("c. joint quality over all route replays (route replays make no counted turns)")
    P_(f"greatest heading step {hmax:.2f} deg; spacing min {smin:.3f} max {smax:.3f} m")
    P_(f"heading steps > 12 deg {nviol_h} (library route has one within 4 samples: {inh_h}; new {nviol_h - inh_h})")
    P_(f"spacing steps outside 0.5 +- 0.15 {nviol_s} (library {inh_s}; new {nviol_s - inh_s})")
    for v in viol_where[:10]:
        P_(f"  new {v[0]} {v[1]} step {v[2]}/{v[3]} value {v[4]:.3f} at ({v[5]:.1f}, {v[6]:.1f})")
    P_()
    P_("time check: sum of edge times / drive_s per route (and rebuilt length / library length)")
    rr = ratios[np.isfinite(ratios)]
    P_(f"time ratio min {rr.min():.4f} p1 {np.percentile(rr, 1):.4f} p50 {np.median(rr):.4f} p95 {np.percentile(rr, 95):.4f} "
       f"p99 {np.percentile(rr, 99):.4f} max {rr.max():.4f}; outside 0.95..1.05: {int(((rr < 0.95) | (rr > 1.05)).sum())}")
    P_(f"length ratio min {lratios.min():.4f} p50 {np.median(lratios):.4f} p99 {np.percentile(lratios, 99):.4f} max {lratios.max():.4f}")
    bad_r = sorted([(float(ratios[k]), float(lratios[k]), routes[k]["id"], float(routes[k]["drive_s"]))
                    for k in range(len(routes)) if np.isfinite(ratios[k]) and abs(ratios[k] - 1) > 0.05], reverse=True)
    for b in bad_r[:5]:
        P_(f"  {b[2]} time ratio {b[0]:.3f} length ratio {b[1]:.3f} drive_s {b[3]:.2f}")
    # ------------------------------------------------ d connectivity
    P_()
    par = np.arange(nn)

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a
    for e in range(ne):
        a, b = find(ea[e]), find(eb[e])
        if a != b:
            par[a] = b
    roots = np.array([find(i) for i in range(nn)])
    lenby = {}
    for e in range(ne):
        lenby[roots[ea[e]]] = lenby.get(roots[ea[e]], 0.0) + el[e]
    big = max(lenby, key=lenby.get)
    P_("d. connectivity")
    P_(f"undirected: components {len(set(roots.tolist()))}; largest {int((roots == big).sum())} nodes, "
       f"{lenby[big] / el.sum() * 100:.3f}% of edge length ({lenby[big]:.0f} of {el.sum():.0f} m)")
    outside = [i for i in range(nn) if roots[i] != big and nflags[i] & 3]
    P_(f"target end / pose nodes outside the largest: {len(outside)} " +
       " ".join(f"{i}({nx[i]:.1f},{nz[i]:.1f})" for i in outside[:10]))
    Gr = Graph(NET, pivot)
    tid = {t[0]: i for i, t in enumerate(targets)}
    zfirst = min(t[0] for t in targets if t[1] == 0)      # zone_A when the map has it, else its first zone
    za = sorted(tsets[tid[zfirst]])
    srcs = [(e2, f2, float(Gr.et[e2]), float(Gr.el[e2])) for n in za for (e2, f2) in Gr.out[n]]
    # every pair of edge ends at every node: continuation (sample step <= 12 deg), turn (2 m turn > 30 deg) or between
    hs = np.zeros((ne, 2, 2))      # [e, f, 0 = leaving first step, 1 = arriving last step]
    for e in range(ne):
        P = edge_pts(NET, e)
        h0 = math.atan2(P[1, 1] - P[0, 1], P[1, 0] - P[0, 0])
        h1 = math.atan2(P[-1, 1] - P[-2, 1], P[-1, 0] - P[-2, 0])
        hs[e, 1] = (h0, h1)
        hs[e, 0] = (h1 + math.pi, h0 + math.pi)
    pc = dict(cont=0, turn=0, between=0, between_cross=0)
    bw = []
    for n in range(nn):
        for e2, f2 in Gr.out[n]:
            for e, f_ in Gr.out[n]:
                fa = 1 - f_                      # arriving at n over e
                if e == e2 and fa != f2 and ea[e] != eb[e]:
                    continue                     # straight back on the same edge
                step = abs(math.degrees(wrapr(hs[e2, f2, 0] - hs[e, fa, 1])))
                if step <= MAX_STEP_DEG:
                    pc["cont"] += 1
                elif Gr.turn(n, e, fa, e2, f2)[1] > TURN_FREE_DEG:
                    pc["turn"] += 1
                else:
                    pc["between"] += 1
                    pc["between_cross"] += int(n >= n_route_nodes)
                    if len(bw) < 12:
                        bw.append((n, nx[n], nz[n], step, Gr.turn(n, e, fa, e2, f2)[1]))
    P_(f"edge-end pairs at nodes: continuation (step <= 12 deg) {pc['cont']}, turn (2 m turn > 30 deg) {pc['turn']}, "
       f"in between {pc['between']} (at crossing nodes {pc['between_cross']})")
    for b_ in bw:
        P_(f"  between at node {b_[0]} ({b_[1]:.1f}, {b_[2]:.1f}): sample step {b_[3]:.1f} deg, 2 m turn {b_[4]:.1f} deg")
    reach = Gr.search(srcs)
    redge = {e for (e, f) in reach}
    rnode = set(za) | {Gr.end(e, f) for (e, f) in reach}
    P_(f"turn rule, from {zfirst}'s {len(za)} end nodes: reachable edge length {sum(el[e] for e in redge) / el.sum() * 100:.3f}% "
       f"({len(redge)} of {ne} edges)")
    unr = [(targets[t][0], n) for t in range(len(targets)) for n in sorted(tsets[t]) if n not in rnode]
    P_(f"target end nodes not reachable: {len(unr)}" + ("" if not unr else ": " + " ".join(
        f"{tg}/{n}({nx[n]:.1f},{nz[n]:.1f})" for tg, n in unr[:30])))
    # ------------------------------------------------ e crossings
    P_()
    P_(f"e. crossings (> {CROSS_ANG_DEG:.0f} deg); node = chain A's sample, chain B split at its own nearest sample "
       f"(greatest B-sample to node distance {cross_off:.3f} m); skipped because the nearest sample is an edge end or "
       f"already cut {cross_badidx[0]}; skipped because a transfer's 2 m turn would be <= 30.5 deg {cross_badidx[1]}")
    c1, _, c2, _, Xr = find_crossings([edge_pts(NET, e) for e in range(ne)])
    grid = PointGrid(1.0)
    for x, z in zip(nx, nz):
        grid.add(x, z)
    left = sum(1 for x, z in Xr if not grid.near(x, z, CROSS_NODE_DIST))
    P_(f"crossing points found before the split {cross_found}; made crossing nodes {n_cross_nodes}; skipped (a node within "
       f"{CROSS_NODE_DIST} m) {cross_skipped}; after the split: crossing points {len(Xr)}, without a node within "
       f"{CROSS_NODE_DIST} m {left}")
    # ------------------------------------------------ f queries
    P_()
    zc = {t[0]: (t[5], t[6]) for t in targets if t[1] == 0}
    # zone-to-zone queries for whatever zones the map has (three zones: Q1..Q6 in a fixed order);
    # the fixed points Q7..Q13 belong to eastern_europe_02_dom only
    zl = sorted(zc)
    if zl == ["zone_A", "zone_B", "zone_C"]:
        zpairs = [("zone_A", "zone_B"), ("zone_B", "zone_A"), ("zone_A", "zone_C"), ("zone_C", "zone_A"),
                  ("zone_B", "zone_C"), ("zone_C", "zone_B")]
    else:
        zpairs = [(a_, b_) for a_ in zl for b_ in zl if a_ != b_]
    Q = [(f"Q{k + 1}", zc[a_], ("t", b_)) for k, (a_, b_) in enumerate(zpairs)]
    qfrom = {f"Q{k + 1}": a_ for k, (a_, b_) in enumerate(zpairs)}
    nzq = len(Q)
    q13_note = "Q13: not this map (fixed points of eastern_europe_02_dom)"
    pj = LIB / f"{mapname}.plan.json"
    if mapname == "eastern_europe_02_dom":
        Q += [("Q7", (1988.7, 1845.6), ("t", "2L26_1")),
              ("Q8", (2252.6, 1698.6), ("p", (2070.3, 1736.3))), ("Q9", (2268.8, 1687.2), ("p", (2131.4, 1794.8))),
              ("Q10", (2214.9, 2228.6), ("p", (2155.4, 1778.0))), ("Q11", (2058.2, 1623.1), ("p", (2058.6, 1889.8))),
              ("Q12", (2052.4, 1608.7), ("p", (2009.4, 1751.6)))]
        q13_note = "Q13: no plan.json or no army 2 position lane"
    # Q13: from Q7's start to slot 1 of army 2's far-south position lane (least z of slot 1), by the pack's
    # plan.json; only when the pack has one
    if pj.is_file() and mapname == "eastern_europe_02_dom":
        plan = json.loads(pj.read_text(encoding="utf-8"))
        gl = [l for l in plan["lanes"] if l["army"] == 2 and l["end"] == "guard" and l["slots"]]
        if gl:
            fs = min(gl, key=lambda l: l["slots"][0]["z"])
            tq = targets[fs["slots"][0]["target"]][0]
            Q.append(("Q13", (1988.7, 1845.6), ("t", tq)))
            q13_note = (f"Q13: army 2 far-south position lane {fs['id']} (from {fs['from']}), slot 1 = target {tq} at "
                        f"({fs['slots'][0]['x']:.1f}, {fs['slots'][0]['z']:.1f}); start = Q7's start")
    P_(q13_note)
    sample_edge = np.repeat(np.arange(ne), en)
    sample_k = np.arange(len(NET["sx"])) - np.repeat(efirst, en)
    qlines, qpaths, qres = [], [], []
    P_("f. queries (pure network, turn rule)")
    for name, st_, goal in Q:
        if goal[0] == "t" and goal[1] not in tid:
            line = f"{name} start={st_[0]:.1f},{st_[1]:.1f} goal=target:{goal[1]} skipped: no such target"
            qlines.append(line)
            qpaths.append(None)
            qres.append(None)
            P_(line)
            continue
        res = run_query(Gr, NET, ecum_t, ecum_l, sample_edge, sample_k, st_, goal, targets, tid, tsets)
        qpaths.append(res["P"])
        qres.append(res)
        line = (f"{name} start={st_[0]:.1f},{st_[1]:.1f} goal={res['goal_s']} reached={int(res['reached'])} "
                f"time_s={res['time']:.2f} length_m={res['length']:.1f} edges={res['nedges']} straight_m={res['straight']:.1f} "
                f"start_entry_m={res['d_in']:.2f} exit_goal_m={res['d_out']:.2f} entry={res['entry']} exit={res['exit']} "
                f"nodes={','.join(str(n) for n in res['nodes'])} pivots={len(res['pivots'])} "
                f"pivot_deg={','.join(f'{th:.1f}' for _, th in res['pivots'])}")
        qlines.append(line)
        P_(line)
        if res["reached"]:
            s_ = res.get("steps", {})
            P_(f"   {name} ratio length/straight {res['length'] / res['straight']:.2f}; pivots at nodes "
               f"{[n for n, _ in res['pivots']]}; heading steps > 12 deg: at counted turns {s_.get('n_pivot', 0)}, at other "
               f"joints {s_.get('n_joint', 0)}, inside edges {s_.get('n_inside', 0)} (greatest of these {s_.get('max_other', 0):.1f})")
    (OUT / "net_queries.txt").write_text("\n".join(qlines) + "\n", encoding="utf-8")
    P_()
    P_(f"zone to zone (Q1..Q{nzq}): query time_s length_m pivots pivot_deg ratio")
    for (name, st_, goal), res in zip(Q[:nzq], qres[:nzq]):
        if res is None or not res["reached"]:
            P_(f"{name} {qfrom[name]}->{goal[1]} not reached")
            continue
        P_(f"{name} {qfrom[name]}->{goal[1]} "
           f"{res['time']:.1f} {res['length']:.1f} {len(res['pivots'])} {','.join(f'{th:.0f}' for _, th in res['pivots']) or '-'} "
           f"{res['length'] / max(res['straight'], 1e-6):.2f}")
    # ------------------------------------------------ g hull check
    P_()
    rng = np.random.default_rng(38)
    comp_nodes = np.nonzero(roots == big)[0]
    stat = dict(paths=0, unreached=0, samples=0, tb=0, tg=0, sb_j=0, sb_i=0, sg_j=0, sg_i=0, piv=0, piv_rule=0, piv_sweep=0,
                h_joint=0, h_inside=0, h_max=0.0, piv_paths=0)
    where_g, plen, ppiv = [], [], []
    gstep, gtouch = [], []
    for _ in range(300):
        a, b = (int(v) for v in rng.choice(comp_nodes, 2, replace=False))
        srcs = [(e2, f2, float(Gr.et[e2]), float(Gr.el[e2])) for (e2, f2) in Gr.out[a]]
        r = Gr.search(srcs, lambda e_, f_: (0.0, 0.0, None) if Gr.end(e_, f_) == b else None)
        if r is None:
            stat["unreached"] += 1
            continue
        stat["paths"] += 1
        seq = r[2]
        Pp, joints = assemble([edge_pts(NET, e_, f_ == 1) for e_, f_ in seq])
        tr = Gr.turns_of(seq)
        pj = [joints[q] for q, (n, th) in enumerate(tr) if th > TURN_FREE_DEG]
        stat["samples"] += len(Pp)
        plen.append(r[1])
        ppiv.append(len(pj))
        stat["piv_paths"] += int(len(pj) > 0)
        for q, (n, th) in enumerate(tr):
            if th > TURN_FREE_DEG:
                stat["piv"] += 1
                stat["piv_rule"] += int(node_clr[n] >= int(round(PIVOT_CLR * 10)))
                (e_, f_), (e2, f2) = seq[q], seq[q + 1]
                stat["piv_sweep"] += int(pivot_sweep(G, nx[n], nz[n], Gr.h_in[e_, f_], Gr.h_out[e2, f2]))
        cuts = [0] + pj + [len(Pp) - 1]
        hb_any = hg_any = False
        s = arc(Pp)
        js = s[joints] if joints else np.zeros(0)
        for q in range(len(cuts) - 1):
            Pq = Pp[cuts[q]:cuts[q + 1] + 1]
            if len(Pq) < 2:
                continue
            hb = R.sweep(G, Pq, R.HL, R.HW)
            hg = R.sweep(G, Pq, R.HL + R.GROW, R.HW + R.GROW)
            hb_any |= bool(hb.any())
            hg_any |= bool(hg.any())
            sq = s[cuts[q]:cuts[q + 1] + 1]
            nearj = np.array([bool(len(js)) and np.abs(js - x).min() <= BLEND for x in sq])
            stat["sb_j"] += int((hb & nearj).sum())
            stat["sb_i"] += int((hb & ~nearj).sum())
            stat["sg_j"] += int((hg & nearj).sum())
            stat["sg_i"] += int((hg & ~nearj).sum())
            for i in np.nonzero(hb)[0][:2]:
                if len(where_g) < 12:
                    where_g.append((float(Pq[i, 0]), float(Pq[i, 1]), "joint" if nearj[i] else "inside"))
        stat["tb"] += int(hb_any)
        stat["tg"] += int(hg_any)
        ps = path_steps(Pp, joints, pj)
        if joints and len(Pp) > 2:
            jarr = np.array(joints)
            hh = heads(Pp)[:-1]
            dh = np.abs(wrapd(np.degrees(np.diff(hh))))
            pset = set()
            for j in pj:
                pset.update((j - 2, j - 1, j))
            for i in np.nonzero(dh > MAX_STEP_DEG)[0]:
                if i in pset:
                    continue
                q = int(np.argmin(np.abs(jarr - (i + 1))))
                if abs(int(jarr[q]) - (i + 1)) <= 2 and len(gstep) < 15:
                    gstep.append((tr[q][0], float(dh[i]), tr[q][1], bool(tr[q][0] >= n_route_nodes)))
            hgp = np.zeros(len(Pp), bool)
            for q in range(len(cuts) - 1):
                Pq = Pp[cuts[q]:cuts[q + 1] + 1]
                if len(Pq) >= 2:
                    hgp[cuts[q]:cuts[q + 1] + 1] |= R.sweep(G, Pq, R.HL + R.GROW, R.HW + R.GROW)
            for i in np.nonzero(hgp)[0][:2]:
                q = int(np.argmin(np.abs(jarr - i)))
                if len(gtouch) < 15:
                    gtouch.append((tr[q][0], float(Pp[i, 0]), float(Pp[i, 1]), int(abs(int(jarr[q]) - i)), tr[q][1],
                                   tr[q][1] > TURN_FREE_DEG))
        stat["h_joint"] += ps["n_joint"]
        stat["h_inside"] += ps["n_inside"]
        stat["h_max"] = max(stat["h_max"], ps["max_other"])
    P_("g. hull check, turn-rule shortest paths, 300 random node pairs of the largest undirected component (seed 38), "
       "swept piece by piece between pivots")
    P_(f"paths found {stat['paths']}, not reachable under the turn rule {stat['unreached']}; samples {stat['samples']}; "
       f"length p50 {np.median(plen):.0f} m; paths with a pivot {stat['piv_paths']}, pivots {stat['piv']}")
    P_(f"paths touching: bare 7.4 x 3.6 {stat['tb']}, grown 0.3 m {stat['tg']}; touching samples bare: within 6 m of a joint "
       f"{stat['sb_j']}, inside an edge {stat['sb_i']}; grown: joint {stat['sg_j']}, inside {stat['sg_i']}")
    for w in where_g:
        P_(f"  bare touch at ({w[0]:.1f}, {w[1]:.1f}) {w[2]}")
    P_(f"pivots: pivot-free rule holds {stat['piv_rule']}/{stat['piv']}; bare hull turned on the spot touches hard "
       f"{stat['piv_sweep']}/{stat['piv']}")
    P_(f"heading steps > 12 deg outside counted turns: at joints {stat['h_joint']}, inside edges {stat['h_inside']}; "
       f"greatest {stat['h_max']:.1f} deg")
    for n_, st_, th_, cr_ in gstep:
        P_(f"  step {st_:.1f} deg at node {n_} ({'crossing' if cr_ else 'joint'}) ({nx[n_]:.1f}, {nz[n_]:.1f}), 2 m turn {th_:.1f}")
    for n_, x_, z_, dj_, th_, pv_ in gtouch:
        P_(f"  grown touch at ({x_:.1f}, {z_:.1f}), {dj_} samples from node {n_} "
           f"({'crossing' if n_ >= n_route_nodes else 'joint'}), 2 m turn there {th_:.1f}{' (pivot)' if pv_ else ''}")
    cnt_ = dict(fb=0, fg=0, rb=0, rg=0)
    ewhere = []
    for e in range(ne):
        Pp = edge_pts(NET, e)
        hb = R.sweep(G, Pp, R.HL, R.HW)
        hg = R.sweep(G, Pp, R.HL + R.GROW, R.HW + R.GROW)
        rb_ = R.sweep(G, Pp[::-1], R.HL, R.HW)
        rg_ = R.sweep(G, Pp[::-1], R.HL + R.GROW, R.HW + R.GROW)
        cnt_["fb"] += int(hb.sum())
        cnt_["fg"] += int(hg.sum())
        cnt_["rb"] += int(rb_.sum())
        cnt_["rg"] += int(rg_.sum())
        for i in np.nonzero(hg | rg_[::-1])[0][:1]:
            ewhere.append((e, float(Pp[i, 0]), float(Pp[i, 1])))
    P_(f"every edge alone ({len(NET['sx'])} samples): touching samples forward bare {cnt_['fb']}, grown {cnt_['fg']}; "
       f"backward bare {cnt_['rb']}, grown {cnt_['rg']}" + ("; at " + " ".join(f"e{e}({x:.1f},{z:.1f})" for e, x, z in ewhere[:10])
                                                         if ewhere else ""))
    lb = lg = 0
    for r in lib_routes:
        O = r["xz"].astype(np.float64)[::-1]
        if len(O) >= 2:
            lb += int(R.sweep(G, O, R.HL, R.HW).sum())
            lg += int(R.sweep(G, O, R.HL + R.GROW, R.HW + R.GROW).sum())
    P_(f"reference: library routes swept backward: touching samples bare {lb}, grown {lg}")
    # ------------------------------------------------ h picture
    import picture38
    picture38.picture(OUT / "network.png", M, NET, uses, RB["poses"], targets, qpaths, Q)
    P_()
    P_(f"run time {time.time() - T0:.1f} s")
    (OUT / "network_report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")
    log("done")


if __name__ == "__main__":
    main()


