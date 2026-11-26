"""Step 2 description of corridors: time, length, gates (narrow places), entry bearing, 10 m samples."""
import numpy as np

import common30 as C

GATE_CLR = 4.0       # a gate = a stretch whose clearance drops below 4 m (free width under ~8 m)
GATE_MERGE = 30.0
SKIP_START = 30.0    # the spawn yard is not a gate


def describe(Fl, res, zones, areas):
    zc = {L: (x, z, r) for (L, x, z, r) in zones}
    army_of = {a["id"]: a["army"] for a in areas}
    out = []
    for job in res:
        for k, c in enumerate(job["corridors"]):
            cells, s, t = c["cells"], c["s"], c["t"]
            x, z = Fl.xy_of(cells)
            clr = Fl.clr05.ravel()[cells]
            L = job["zone"]
            zx, zz, zr = zc[L]
            m = s >= SKIP_START
            gates = []
            run = None
            for i in np.nonzero(m)[0]:
                if clr[i] < GATE_CLR:
                    if run is None or s[i] - s[run[-1]] > GATE_MERGE:
                        run = [i]
                        gates.append(run)
                    else:
                        run.append(i)
            gl = []
            for g in gates:
                i = g[int(np.argmin(clr[g]))]
                gl.append(dict(x=round(float(x[i]), 1), z=round(float(z[i]), 1), s=round(float(s[i]), 1),
                               t=round(float(t[i]), 1), width=round(float(2 * clr[i]), 1),
                               kind=gate_kind(Fl, int(cells[i]), float(clr[i]))))
            i0 = int(np.nonzero(m)[0][np.argmin(clr[m])]) if m.any() else 0
            ex, ez = float(x[-1]), float(z[-1])
            b = C.bearing(ex - zx, ez - zz)
            # 10 m samples
            ss = np.arange(0.0, float(s[-1]) + 1e-6, C.SAMPLE_M)
            if ss[-1] < s[-1]:
                ss = np.append(ss, s[-1])
            sx, sz, st = np.interp(ss, s, x), np.interp(ss, s, z), np.interp(ss, s, t)
            dt = np.diff(np.concatenate([[0.0], st]))
            out.append(dict(id=f"{job['area']}>{L}#{k}", army=army_of[job["area"]], area=job["area"], zone=L, rank=k,
                            time=round(float(c["time"]), 1), length=round(float(s[-1]), 1),
                            ratio=round(float(c["time"]) / job["best_t"], 3), overlap=round(c["overlap"], 2),
                            narrowest=dict(width=round(float(2 * clr[i0]), 1), x=round(float(x[i0]), 1),
                                           z=round(float(z[i0]), 1), s=round(float(s[i0]), 1),
                                           kind=gate_kind(Fl, int(cells[i0]), float(clr[i0]))),
                            gates=gl, entry=dict(x=round(ex, 1), z=round(ez, 1), bearing=round(b, 0), side=C.compass(b)),
                            road_share=round(float(Fl.road05.ravel()[cells].mean()), 2), cells=cells,
                            samples=dict(x=sx.astype(np.float32), z=sz.astype(np.float32), t=st.astype(np.float32),
                                         dt=dt.astype(np.float32), to_end=(s[-1] - ss).astype(np.float32)),
                            path=dict(x=x[::4].astype(np.float32), z=z[::4].astype(np.float32), t=t[::4])))
    return out


def gate_kind(Fl, cell, clr):
    r, c = divmod(cell, Fl.W5)
    k = int(np.ceil((clr + 2.0) / 0.5))
    sl = (slice(max(0, r - k), r + k + 1), slice(max(0, c - k), c + k + 1))
    kinds = []
    if Fl.water05[sl].any():
        kinds.append("water (bridge / bank)")
    if Fl.obj05[sl].any():
        kinds.append("objects (houses / walls)")
    if Fl.steep05[sl].any():
        kinds.append("steep slope")
    return " + ".join(kinds) if kinds else "other"
