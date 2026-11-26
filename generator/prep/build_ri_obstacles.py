"""Exact static obstacles of a ground map from the game's own files: rendinst placements x collision resources x
destruction table. Read only; writes into --out.

Sources (all of the installed 2.61 game, read only):
  * levels/<level>.bin, block RIGz: the pregenerated rendinst instances (Dagor RendInstGenData dump; layout in
    rigz_decode() below). Mask-generated instances (forests / land-class tiles) are NOT in it and not handled.
  * content/base/res/**/*.grp: CollisionResource "<name>_collision" (class 0xACE50000): per node a local box and
    a node transform (layout reverse engineered from the files).
  * config/rendinst_dmg.blk of aces.vromfs.bin as text (prep/blk_to_text.py writes it; needs the third-party vromfs /
    blk Python packages): dmg{name, immortal, destructionImpulse, bushBehaviour, treeBehaviour ...} and
    riExtra{<name>{...}}.

Output per kept collision node (in the scan rectangle, overlapping the hull band 0.25-2.6 m above the
instance base): one record {name, cls, impulse, x, z (centre), hx, hz (half extents), yaw, y0, y1}. cls: immortal /
impulse (destructionImpulse known) / bush / tree / unknown (listed without either) / unlisted. Footprint
boxes are the node boxes, which for mesh nodes are the mesh's bounding box (conservative).

usage (Python with numpy and Pillow):
  build_ri_obstacles.py --level avg_sweden (--scan <obstacles/sweden_dom.bin> | --box x0 z0 step nx nz)
                        --dmg <common/rendinst_dmg.blk> --out <dir>
"""
import argparse
import csv
import json
import math
import re
import struct
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))          # catalog_reader.py lives beside this file
sys.path.insert(1, str(Path(__file__).resolve().parents[1]))
import catalog_reader as cr  # noqa: E402
import paths  # noqa: E402

WT = paths.game_dir()    # the installed game (env WT_GAME_DIR); prep is Windows-only (daKernel-dev.dll codecs)
CDK = paths.cdk_dir()
RES = WT / "content" / "base" / "res"
COLLISION_CLASS = 0xACE50000
BAND = (0.25, 2.6)   # m above the instance base that a hull occupies


def level_block(data, want):
    off = 0x120
    while off + 8 <= len(data):
        size = struct.unpack_from("<I", data, off)[0]
        tag = data[off + 4:off + 8]
        if tag == want:
            return off + 8, size - 4
        if size == 0 or tag == b"\0END":
            break
        off += 4 + size
    raise KeyError(want)


def cstr(d, o):
    return d[o:d.index(0, o)].decode("latin1")


def rigz_decode(codec, data):
    """[(name, x, y, z, axis_x(3), axis_y(3), axis_z(3))] of the pregenerated instances.
    RIGz payload: block A (u32 size | fmt << 30; fmt 2 = oodle: u32 raw size, stream) = u32 dump_sz + dump;
    block B header u32, base right after it. Dump (pointers = u32 offsets into the dump, tables = {u32 ofs,
    u32 count, 8 reserved}): @8 cells, @24 cellNumW/H, @32 i16 cellSz, u8 dataFlags (1 = 32-bit counters),
    u8 perInstDataDwords, @48 landCls (32 bytes each), @64 world0 xz, @112 grid2world, @128 pregenEnt (32 bytes:
    @8 name ptr, @24 bits posInst|paletteRotation<<1|zeroInstSeeds<<2). Cell (552 bytes): @24 i16 htMin,
    htDelta, i32 riDataRelOfs, @32 65 x u64 pointers to counters {u32 pool, u32 count}. Cell data at
    base + riDataRelOfs: u32 header, zstd; per subcell, per counter: count x stride (8 posInst / 24 int16 TM,
    + 4 x perInstDataDwords unless zeroInstSeeds). int16 TM rows (ax_i, ay_i, az_i, pos_i), axes / 256,
    pos = v / 32767 x (cell size, htDelta or 8192, cell size) + (cell origin x, htMin, cell origin z)."""
    start, _ = level_block(data, b"RIGz")
    hdr = struct.unpack_from("<I", data, start)[0]
    a_size, a_fmt = hdr & 0x3FFFFFFF, hdr >> 30
    if a_fmt != 2:
        raise ValueError(f"RIGz block A format {a_fmt} (only oodle handled)")
    usize = struct.unpack_from("<I", data, start + 4)[0]
    raw = codec.decompress(data[start + 8:start + 4 + a_size], size=usize, oodle=True)
    dump_sz = struct.unpack_from("<I", raw, 0)[0]
    d = raw[4:4 + dump_sz]
    cells_p, ncell = struct.unpack_from("<II", d, 8)
    cw, _ = struct.unpack_from("<ii", d, 24)
    cell_sz, flags, per_dw = struct.unpack_from("<hBB", d, 32)
    world0 = struct.unpack_from("<2f", d, 64)
    grid2world = struct.unpack_from("<f", d, 112)[0]
    pe_p, npe = struct.unpack_from("<II", d, 128)
    cxz = grid2world * cell_sz
    pools = []
    for k in range(npe):
        o = pe_p + 32 * k
        bits = struct.unpack_from("<I", d, o + 24)[0]
        pools.append((cstr(d, struct.unpack_from("<Q", d, o + 8)[0]), bits & 1, (bits >> 1) & 1, (bits >> 2) & 1))
    base = start + 4 + a_size + 4
    ent32 = flags & 1
    out = []
    for k in range(ncell):
        co = cells_p + 552 * k
        ht_min, ht_delta, rel = struct.unpack_from("<hhi", d, co + 24)
        ents = struct.unpack_from("<65Q", d, co + 32)
        if ents[0] == ents[64]:
            continue
        ox, oz = world0[0] + (k % cw) * cxz, world0[1] + (k // cw) * cxz
        hd = ht_delta if ht_delta else 8192
        bsize = struct.unpack_from("<I", data, base + rel)[0] & 0x3FFFFFFF
        blob = codec.decompress(data[base + rel + 4:base + rel + 4 + bsize])
        pos = 0
        for s in range(64):
            for c in range(ents[s], ents[s + 1], 8 if ent32 else 4):
                res, cnt = struct.unpack_from("<II" if ent32 else "<HH", d, c)
                name, pos_inst, pal, zero_seed = pools[res]
                stride = (8 if pos_inst else 24) + (0 if zero_seed else 4 * per_dw)
                for _ in range(cnt):
                    if pos_inst:
                        v = struct.unpack_from("<4h", blob, pos)
                        sc = 1.0 if pal else (v[3] & 0xFFFF) / 256.0
                        out.append((name, v[0] * cxz / 32767 + ox, v[1] * hd / 32767 + ht_min, v[2] * cxz / 32767 + oz,
                                    (sc, 0.0, 0.0), (0.0, sc, 0.0), (0.0, 0.0, sc)))
                    else:
                        v = struct.unpack_from("<12h", blob, pos)
                        out.append((name, v[3] * cxz / 32767 + ox, v[7] * hd / 32767 + ht_min, v[11] * cxz / 32767 + oz,
                                    (v[0] / 256, v[4] / 256, v[8] / 256), (v[1] / 256, v[5] / 256, v[9] / 256),
                                    (v[2] / 256, v[6] / 256, v[10] / 256)))
                    pos += stride
        if pos != len(blob):
            raise ValueError(f"cell {k}: consumed {pos} of {len(blob)} bytes")
    return out


class Collision:
    def __init__(self, codec):
        self.codec = codec
        self.where = {}
        self.cache = {}
        for p in sorted(RES.rglob("*.grp")):
            with open(p, "rb") as f:
                head = f.read(16)
                if len(head) < 16 or head[:4] != b"GRP2":
                    continue
                data = head + f.read(struct.unpack_from("<I", head, 8)[0])
            try:
                name_ofs, name_cnt = struct.unpack_from("<II", data, 16)
                res_ofs, res_cnt = struct.unpack_from("<II", data, 32)
                ofs = struct.unpack_from("<%dI" % name_cnt, data, name_ofs)
                names = [data[o:data.index(0, o)].decode("utf-8", "replace") for o in ofs]
                for i in range(res_cnt):
                    cls, off, rid = struct.unpack_from("<IIH", data, res_ofs + 12 * i)
                    if cls == COLLISION_CLASS and rid < len(names):
                        self.where.setdefault(names[rid].lower(), (p, off))
            except (struct.error, ValueError):
                continue

    def nodes(self, name):
        """[(node name, node type, local box lo, hi, node tm 12f)] or None."""
        key = name.lower() + "_collision"
        if key in self.cache:
            return self.cache[key]
        hit = self.where.get(key)
        result = None
        if hit:
            path, off = hit
            with open(path, "rb") as f:
                f.seek(off)
                label, word = struct.unpack("<II", f.read(8))
                body = f.read(word & 0x3FFFFFFF)
            if label == 0xACE50003 and word >> 30 in (0, 1):
                raw = self.codec.decompress(body) if word >> 30 == 1 else body
                count = struct.unpack_from("<I", raw, 64)[0]
                pool_len = struct.unpack_from("<I", raw, 100)[0]
                pbase = len(raw) - pool_len - 1
                first = 108 + 8 * count
                tms = first + 48 * count
                if count and pbase >= tms + 48 * count:
                    result = []
                    for i in range(count):
                        r = first + 48 * i
                        nofs, _, ntype = struct.unpack_from("<III", raw, r)
                        bb = struct.unpack_from("<6f", raw, r + 20)
                        tm = struct.unpack_from("<12f", raw, tms + 48 * i)
                        p = pbase + nofs
                        nm = raw[p:raw.find(b"\0", p)].decode("latin1") if pbase < p < len(raw) else "?"
                        result.append((nm, ntype, bb[:3], bb[3:], tm))
        self.cache[key] = result
        return result

    def raw(self, name):
        """Decompressed body of <name>_collision, or None."""
        hit = self.where.get(name.lower() + "_collision")
        if hit is None:
            return None
        path, off = hit
        with open(path, "rb") as f:
            f.seek(off)
            label, word = struct.unpack("<II", f.read(8))
            body = f.read(word & 0x3FFFFFFF)
        if label != 0xACE50003 or word >> 30 not in (0, 1):
            return None
        return self.codec.decompress(body) if word >> 30 == 1 else body


def parse_dmg(path):
    """name -> dict of the dmg{} and riExtra{} parameters (riExtra wins)."""
    out = {}
    text = Path(path).read_text(encoding="utf-8")
    for m in re.finditer(r"^dmg\{\n(.*?)^\}", text, re.M | re.S):
        body = m.group(1)
        nm = re.search(r'name:t="([^"]+)"', body)
        if nm:
            out[nm.group(1).lower()] = dict(re.findall(r"^\s+([A-Za-z]+):[a-z0-9]+=(.*)$", body, re.M))
    rx = re.search(r"^riExtra\{\n(.*?)^\}", text, re.M | re.S)
    if rx:
        for m in re.finditer(r"^  ([A-Za-z0-9_\-]+)\{\n(.*?)^  \}", rx.group(1), re.M | re.S):
            out.setdefault(m.group(1).lower(), {}).update(dict(re.findall(r"^\s+([A-Za-z]+):[a-z0-9]+=(.*)$", m.group(2), re.M)))
    return out


def classify(params):
    if params is None:
        return "unlisted", None
    imp = params.get("destructionImpulse")
    if params.get("immortal") == "yes":
        return "immortal", None
    if params.get("bushBehaviour") == "yes":
        return "bush", float(imp) if imp else None
    if params.get("treeBehaviour") == "yes":
        return "tree", float(imp) if imp else None
    if imp:
        return "impulse", float(imp)
    return "unknown", None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", required=True)
    ap.add_argument("--scan", help="WTOBS001 obstacle scan: its rectangle is used (header only)")
    ap.add_argument("--box", type=float, nargs=5, metavar=("X0", "Z0", "STEP", "NX", "NZ"),
                    help="without a scan: the rectangle the scan would have (prep/scan_area.py)")
    ap.add_argument("--dmg", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    codec = cr.Codecs(str(CDK))
    inst = rigz_decode(codec, (WT / "levels" / f"{a.level}.bin").read_bytes())
    import scan_area
    nx, nz, x0, z0, step, _ = scan_area.grid_from_args(a)
    x1, z1 = x0 + nx * step, z0 + nz * step
    col = Collision(codec)
    dmg = parse_dmg(a.dmg)
    recs = []
    kinds = Counter()
    nocol = Counter()
    for name, px, py, pz, ax, ay, az in inst:
        if not (x0 - 20 <= px <= x1 + 20 and z0 - 20 <= pz <= z1 + 20):
            continue
        kinds[name] += 1
        nodes = col.nodes(name)
        if not nodes:
            nocol[name] += 1
            continue
        cls, imp = classify(dmg.get(name.lower()))
        for nm, ntype, lo, hi, tm in nodes:
            # node box corners -> model frame (tm rows: x axis, y axis, z axis, position) -> world (instance axes)
            pts = []
            for cx in (lo[0], hi[0]):
                for cy in (lo[1], hi[1]):
                    for cz in (lo[2], hi[2]):
                        m = [cx * tm[0] + cy * tm[3] + cz * tm[6] + tm[9],
                             cx * tm[1] + cy * tm[4] + cz * tm[7] + tm[10],
                             cx * tm[2] + cy * tm[5] + cz * tm[8] + tm[11]]
                        pts.append((px + m[0] * ax[0] + m[1] * ay[0] + m[2] * az[0],
                                    py + m[0] * ax[1] + m[1] * ay[1] + m[2] * az[1],
                                    pz + m[0] * ax[2] + m[1] * ay[2] + m[2] * az[2]))
            ys = [p[1] for p in pts]
            if max(ys) < py + BAND[0] or min(ys) > py + BAND[1]:
                continue
            # footprint: oriented box along the projected node x axis
            ux = (tm[0] * ax[0] + tm[1] * ay[0] + tm[2] * az[0], tm[0] * ax[2] + tm[1] * ay[2] + tm[2] * az[2])
            yaw = math.atan2(ux[1], ux[0]) if math.hypot(*ux) > 1e-4 else 0.0
            c, s = math.cos(yaw), math.sin(yaw)
            us = [p[0] * c + p[2] * s for p in pts]
            vs = [-p[0] * s + p[2] * c for p in pts]
            uc, vc = (max(us) + min(us)) / 2, (max(vs) + min(vs)) / 2
            recs.append({"name": name, "node": nm, "ntype": ntype, "cls": cls, "impulse": imp,
                         "x": uc * c - vc * s, "z": uc * s + vc * c, "hx": (max(us) - min(us)) / 2,
                         "hz": (max(vs) - min(vs)) / 2, "yaw": yaw, "y0": min(ys), "y1": max(ys), "base": py})
    with open(out / f"{a.level}_ri_obstacles.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(recs[0].keys()))
        w.writeheader()
        for r in recs:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})
    summary = {
        "level": a.level, "instances_in_area": sum(kinds.values()), "kinds_in_area": len(kinds),
        "instances_without_collision": sum(nocol.values()), "kinds_without_collision": len(nocol),
        "records": len(recs), "records_by_class": Counter(r["cls"] for r in recs),
        "top_without_collision": nocol.most_common(20),
    }
    (out / f"{a.level}_summary.json").write_text(json.dumps(summary, indent=1, default=list), encoding="utf-8")
    print(json.dumps(summary, indent=1, default=list))


if __name__ == "__main__":
    main()
