"""CollisionResource (gameres class 0xACE50000, stream label 0xACE50003) reader with mesh triangles.

Layout (2.61 files; field meaning from the public DagorEngine sources collisionGameResLoad.cpp, daBVH/bvhIO.cpp,
daBVH/swBLASLeafDefs.hlsli, checked against the files):
  header 108 bytes: bsphere(4f) bindTraceSphere(4f) bbox(6f) radius(f) collisionFlags(u32)
    counts[11] u32 @64: NODES, AUTH_TM, AUTH_ITM, REL_TMS, NODE_BLAS (in-memory bytes, not wire), TLAS,
    PHYSMAT_POOL, CAPSULES, CONVEX_PLANES, NAMES (bytes), MAT_NAMES (bytes)
  binds N x 8; node records N x 48: u32 nameOfs, u32 indicesCount, ... u8 flags @18, u8 type @19
    (0 mesh, 2 box, ...), local bbox 6f @20, radius f @44
  AUTH_TM N x 12f (rows: x axis, y axis, z axis, position), AUTH_ITM x 12f, REL_TMS x 12f
  per node with indicesCount != 0, in node order, one chunk:
    frame: scale(3f) invScale(3f) bmin(3f)
    BLAS wire: magic u32, version u16, vertStride u8 (8), leafSize u8 (28), treeBytes u32, vertCount u32,
      bmin(3f) bmax(3f), [version >= 2: ioFlags u32 (1 = edge flags)], vertCount x u64 vert21
      (x bits 0-20, y 21-41, z 42-62; value = q16 x 32; local = frame.bmin + q16 x invScale),
      then a pre-order node stream: one count byte: 1..251 inner node (that many children follow),
      0 full leaf (W0 W1 W2 W3), 0xFE short leaf (W0 W1; W2 = W3 = 0), 0xFD short leaf with flipA
      (W2 = 1 << 29), 0xFF raw leaf (16 bytes, no triangles); then [edge flags] u16 per leaf.
    Leaf: baseA = W1 & 0xFFFFFF (vertex index); o1A = W0[0:13], o2A = W0[13:26], o3A = W0[26:31] | W1[24:32] << 5
      (13-bit signed); quad A: v0 = baseA, vk = baseA + okA; triA (v0,v1,v2), triB (v1,v2,v3) unless o3A == o2A.
      baseB = baseA + W2[0:16]; o1B = W2[16:29], o2B = W3[0:13], o3B = W3[13:26]; quad B present iff o1B != o2B.
"""
import struct


def s13(v):
    v &= 0x1FFF
    return v - 0x2000 if v & 0x1000 else v


class ColRes:
    def __init__(self, raw):
        self.raw = raw
        self.counts = struct.unpack_from("<11I", raw, 64)
        n = self.counts[0]
        o = 108 + 8 * n
        self.nodes = []
        for i in range(n):
            r = o + 48 * i
            name_ofs, idx_cnt = struct.unpack_from("<II", raw, r)
            flags, ntype = raw[r + 18], raw[r + 19]
            bb = struct.unpack_from("<6f", raw, r + 20)
            self.nodes.append({"name_ofs": name_ofs, "indices": idx_cnt, "flags": flags, "type": ntype,
                               "lo": bb[:3], "hi": bb[3:]})
        o += 48 * n
        for i in range(n):
            self.nodes[i]["tm"] = struct.unpack_from("<12f", raw, o + 48 * i)
        o += 48 * (self.counts[1] + self.counts[2] + self.counts[3])
        for nd in self.nodes:
            nd["tris"] = None
            if nd["indices"]:
                o, verts, tris = self._chunk(o, nd["indices"] // 3)
                if 3 * len(tris) != nd["indices"]:
                    raise ValueError(f"triangles {len(tris)} vs indices {nd['indices']}")
                nd["verts"], nd["tris"] = verts, tris
        self.after_blas = o
        names_at = len(raw) - self._tail_guess()
        self.names_blob = raw[names_at:names_at + self.counts[9]]
        for nd in self.nodes:
            b = self.names_blob
            p = nd["name_ofs"]
            nd["name"] = b[p:b.find(b"\0", p)].decode("latin1") if 0 <= p < len(b) else "?"

    def _tail_guess(self):
        # names + material names are the last two arrays (tail records, if any, are not handled)
        return self.counts[9] + self.counts[10]

    def _chunk(self, o, want):
        raw = self.raw
        inv = struct.unpack_from("<3f", raw, o + 12)
        fbmin = struct.unpack_from("<3f", raw, o + 24)
        o += 36
        _magic, version, stride, leaf_size, _tree, vcount = struct.unpack_from("<IHBBII", raw, o)
        if stride != 8 or leaf_size != 28:
            raise ValueError(f"BLAS stride {stride} leaf {leaf_size}")
        o += 16 + 24
        io_flags = 0
        if version >= 2:
            io_flags = struct.unpack_from("<I", raw, o)[0]
            o += 4
        verts = []
        for k in range(vcount):
            w = struct.unpack_from("<Q", raw, o + 8 * k)[0]
            q = (w & 0x1FFFFF, (w >> 21) & 0x1FFFFF, (w >> 42) & 0x1FFFFF)
            verts.append(tuple(fbmin[a] + q[a] / 32.0 * inv[a] for a in range(3)))
        o += 8 * vcount
        tris = []
        leaves = 0
        stack = 0
        roots = 0
        while stack or len(tris) < want:
            if not stack:
                stack = 1
                roots += 1
            cc = raw[o]
            o += 1
            stack -= 1
            if 1 <= cc <= 251:
                stack += cc
                continue
            leaves += 1
            if cc == 0xFF:
                o += 16
                continue
            if cc == 0:
                w0, w1, w2, w3 = struct.unpack_from("<4I", raw, o)
                o += 16
            elif cc in (0xFE, 0xFD):
                w0, w1 = struct.unpack_from("<2I", raw, o)
                w2, w3 = (1 << 29 if cc == 0xFD else 0), 0
                o += 8
            else:
                raise ValueError(f"node byte {cc:#x}")
            base = w1 & 0xFFFFFF
            o1, o2 = s13(w0), s13(w0 >> 13)
            o3 = s13(((w0 >> 26) & 0x1F) | ((w1 >> 24) << 5))
            v = (base, base + o1, base + o2, base + o3)
            tris.append((v[0], v[1], v[2]))
            if o3 != o2:
                tris.append((v[1], v[2], v[3]))
            o1b, o2b, o3b = s13(w2 >> 16), s13(w3), s13(w3 >> 13)
            if o1b != o2b:
                bb = base + (w2 & 0xFFFF)
                u = (bb, bb + o1b, bb + o2b, bb + o3b)
                tris.append((u[0], u[1], u[2]))
                if o3b != o2b:
                    tris.append((u[1], u[2], u[3]))
        if io_flags & 1:
            o += 2 * leaves
        self.last_roots = roots
        for t in tris:
            if max(t) >= vcount or min(t) < 0:
                raise ValueError("triangle vertex out of range")
        return o, verts, tris
