"""v4 binary files for a C loader (little endian, no padding; read_check.c reads them with fread).

routes.bin
  header   char[8] "WTRTE004", u32 version (4), u32 n_poses, u32 n_targets, u32 n_slots, u32 n_routes      (28 B)
  poses    n_poses   x { char[8] id, char[8] start area, f32 x, f32 z, f32 heading_deg }                 (28 B)
  targets  n_targets x { char[16] id, u8 kind (0 zone, 1 post), u8 army (0 zone, 1 south, 2 north),
                         u8 post kind (0 zone, 1 choke, 2 second crew, 3 overwatch, 4 gate),
                         u8 standing (1: has a hull-centre cell within 10 m and routes, 0: no standing room),
                         f32 x, f32 z (log point / zone centre), f32 sx, f32 sz (snapped point; zone centre),
                         f32 radius, f32 inner_radius (route ends inside; 0 for a post),
                         f32 faces_x, f32 faces_z (overwatch "faces" point; NaN when none) }               (52 B)
  slots    n_slots   x { u16 target index, u16 slot number, f32 x, f32 z, f32 clearance_m }                  (16 B)
  ranks    n_poses * n_targets x u8: rank of the target for the pose by best total time among the zones and
           the posts of the pose's army, 1 = nearest, 0 = no route (pose index major)
  routes   n_routes  x { char[32] id, u8 kind (0 spawn pose -> zone, 1 spawn pose -> post, 2 zone <-> zone,
                         3 zone <-> post; kinds 2 and 3 are usable in both directions),
                         u8 rank (by total time within its (pose, target) or target pair, 0 = best),
                         u8 flags (bit 0 tangent start, bit 1 designated, bit 2 start turn free in forward gear,
                         bit 3 free in reverse gear), u8 fallback order (0 = designated),
                         u16 pose index (0xFFFF for kinds 2, 3), u16 start target (0xFFFF for kinds 0, 1),
                         u16 end target, u8 start turn gear (0 none, 1 forward, 2 reverse, 3 blocked), u8 reserved,
                         f32 total_s (drive + start turn), f32 drive_s, f32 start_turn_deg, f32 length_m,
                         u32 n_samples }                                                                   (64 B)
           followed by n_samples x { f32 x, f32 z }, n_samples x u8 clearance (0.1 m steps, capped at 25.5 m),
           n_samples x f32 curvature (1/m, signed, left positive)
           Samples are 0.5 m apart along the curve; heading = direction to the next sample.
map.bin
  header   char[8] "WTMAP004", f32 x0, f32 z0, f32 cell (0.25), f32 cell05 (0.5), u32 cols, u32 rows,
           u32 cols05, u32 rows05                                                                            (40 B)
  hard     rows x ceil(cols / 8) bytes, row r = cells z0 + (r + 0.5) * cell, bit (c & 7) of byte (c >> 3) set
           (LSB first) when the 0.25 m cell (column c) is hard in the merged map
  clear05  rows05 x cols05 u8: clearance of the 0.5 m cell centre to the nearest hard cell in 0.1 m steps,
           capped at 25.5 m (0 when on or under 0)
"""
import struct

import numpy as np

import common30 as C   # only C.CELL (0.25) is used

RH = struct.Struct("<8sIIIII")
RP = struct.Struct("<8s8sfff")
RT = struct.Struct("<16sBBBBffffffff")
RS = struct.Struct("<HHfff")
RR = struct.Struct("<32sBBBBHHHBBffffI")
KIND = {"spawn_zone": 0, "spawn_post": 1, "zone_zone": 2, "zone_post": 3}
PKIND = {"choke": 1, "second crew": 2, "overwatch": 3, "gate": 4}
GEAR = {"none": 0, "forward": 1, "reverse": 2, "blocked": 3}
MH = struct.Struct("<8sffffIIII")


def _s(v, n):
    return (v or "").encode()[:n]


def write_routes_bin(path, poses, targets, slots, rank_of, routes):
    pi = {p["id"]: i for i, p in enumerate(poses)}
    ti = {t["id"]: i for i, t in enumerate(targets)}
    sl = [(ti[tid], k, s) for tid, lst in slots.items() for k, s in enumerate(lst)]
    with open(path, "wb") as f:
        f.write(RH.pack(b"WTRTE004", 4, len(poses), len(targets), len(sl), len(routes)))
        for p in poses:
            f.write(RP.pack(_s(p["id"], 8), _s(p.get("area"), 8), p["x"], p["z"], p["heading"]))
        for t in targets:
            x, z = t["point_log"]
            sx, sz = t["point_log"] if t["kind"] == "zone" else (t.get("point") or (float("nan"), float("nan")))
            fx, fz = t.get("faces") or (float("nan"), float("nan"))
            f.write(RT.pack(_s(t["id"], 16), 0 if t["kind"] == "zone" else 1, t.get("army") or 0,
                            PKIND.get(t.get("post_kind"), 0), 1 if t.get("standing", True) else 0, x, z, sx, sz,
                            t.get("radius") or 0.0, t.get("inner_radius") or 0.0, fx, fz))
        for k, n, s in sl:
            f.write(RS.pack(k, n, s["x"], s["z"], s["clearance"]))
        ranks = np.zeros((len(poses), len(targets)), np.uint8)
        for (p, t), r in rank_of.items():
            ranks[pi[p], ti[t]] = r
        f.write(ranks.tobytes())
        for r in routes:
            sm = r["samples"]
            n = len(sm["x"])
            flags = (1 if r.get("pinned_start") else 0) | (2 if r.get("designated") else 0) | \
                    (4 if r.get("pivot_fwd_free", True) else 0) | (8 if r.get("pivot_rev_free", True) else 0)
            f.write(RR.pack(_s(r["id"], 32), KIND[r["route_kind"]], r["rank"], flags,
                            min(255, r.get("fallback_order", 255)),
                            pi[r["pose"]] if r["kind"] == "spawn" else 0xFFFF,
                            ti[r["start_target"]] if r["kind"] == "between" else 0xFFFF, ti[r["target"]],
                            GEAR[r.get("start_turn_gear", "none")], 0,
                            r["total_s"], r["time_s"], r.get("pivot_at_start_deg") or 0.0, r["length"], n))
            xz = np.stack([np.asarray(sm["x"], np.float32), np.asarray(sm["z"], np.float32)], axis=1)
            f.write(xz.astype("<f4").tobytes())
            f.write(np.clip(np.floor(np.asarray(sm["clearance"]) * 10 + 1e-6), 0, 255).astype(np.uint8).tobytes())
            f.write(np.asarray(sm["curvature"], "<f4").tobytes())


def write_map_bin(path, Fl):
    hard = Fl.hard
    rows, cols = hard.shape
    bits = np.packbits(hard, axis=1, bitorder="little")
    c05 = np.clip(np.floor(Fl.clr05 * 10 + 1e-6), 0, 255).astype(np.uint8)
    with open(path, "wb") as f:
        f.write(MH.pack(b"WTMAP004", Fl.X0, Fl.Z0, C.CELL, 0.5, cols, rows, c05.shape[1], c05.shape[0]))
        f.write(bits.tobytes())
        f.write(c05.tobytes())


def read_routes_bin(path):
    b = open(path, "rb").read()
    magic, ver, npo, nt, ns, nr = RH.unpack_from(b, 0)
    assert magic == b"WTRTE004"
    o = RH.size
    poses = []
    for _ in range(npo):
        pid, area, x, z, h = RP.unpack_from(b, o)
        o += RP.size
        poses.append((pid.rstrip(b"\0").decode(), x, z, h))
    tg = []
    for _ in range(nt):
        f = RT.unpack_from(b, o)
        o += RT.size
        tg.append((f[0].rstrip(b"\0").decode(),) + f[1:])
    o += RS.size * ns
    ranks = np.frombuffer(b, np.uint8, npo * nt, o).reshape(npo, nt)
    o += npo * nt
    rs = []
    for _ in range(nr):
        f = RR.unpack_from(b, o)
        o += RR.size
        n = f[-1]
        xz = np.frombuffer(b, "<f4", 2 * n, o).reshape(n, 2)
        o += 8 * n
        clr = np.frombuffer(b, np.uint8, n, o)
        o += n
        kap = np.frombuffer(b, "<f4", n, o)
        o += 4 * n
        rs.append(dict(id=f[0].rstrip(b"\0").decode(), kind=f[1], rank=f[2], flags=f[3], fallback=f[4], pose=f[5],
                       start_target=f[6], end_target=f[7], gear=f[8], total_s=f[10], drive_s=f[11], turn_deg=f[12],
                       length=f[13], xz=xz, clr=clr, kappa=kap))
    assert o == len(b)
    return dict(poses=poses, targets=tg, n_slots=ns, ranks=ranks, routes=rs)


def read_map_bin(path):
    b = open(path, "rb").read()
    magic, x0, z0, cell, cell05, cols, rows, c5, r5 = MH.unpack_from(b, 0)
    o = MH.size
    nb = (cols + 7) // 8
    bits = np.frombuffer(b, np.uint8, rows * nb, o).reshape(rows, nb)
    o += rows * nb
    hard = np.unpackbits(bits, axis=1, bitorder="little")[:, :cols].astype(bool)
    clr = np.frombuffer(b, np.uint8, r5 * c5, o).reshape(r5, c5)
    o += r5 * c5
    assert o == len(b)
    return dict(x0=x0, z0=z0, cell=cell, cell05=cell05, hard=hard, clr05=clr)
