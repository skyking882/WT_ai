"""Extract the Lnav block (Detour single-tile navmesh, zstd) from a level .bin (DBLD) and decode its polygons.

Block payload: u32 word (low 30 bits = packed size, top bits = codec flags, 0x40 in the high byte), then a zstd
frame. Decompressed: 100 byte dtMeshHeader, verts (3 f32 each), polys (32 bytes each: firstLink u32, verts 6 u16,
neis 6 u16, flags u16, vertCount u8, areaAndType u8). Decompression uses zstd_decompress from the CDK's
daKernel-dev.dll (the game folder comes from paths.py, env WT_GAME_DIR). Read only.
  lnav_extract.py <level.bin> <out.bin>     write the decompressed block
"""
import ctypes
import os
import struct
import sys
from pathlib import Path

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(1, str(Path(__file__).resolve().parents[1]))
import paths  # noqa: E402

GAME = paths.GAME                    # None when WT_GAME_DIR is not set (parse() does not need the game)
CDK = GAME / "WarThunderCDK" / "dagor_cdk" / "windows-x86_64" if GAME else None
LEVELS = GAME / "levels" if GAME else None


def blocks(data):
    if data[:8] != b"DBLD3x64":
        raise ValueError("Unsupported level magic")
    pos = 12
    while pos + 8 <= len(data):
        word, tag = struct.unpack_from("<I4s", data, pos)
        size = word & 0x3FFFFFFF
        end = pos + 4 + size
        if size < 4 or end > len(data):
            raise ValueError(f"Invalid DBLD block at {pos}")
        yield tag, pos, data[pos + 8:end]
        pos = end
        if tag == b"\0END":
            return


def _zstd_content_size(frame):
    """Frame_Content_Size from a zstd frame header, or None."""
    if frame[:4] != b"\x28\xb5\x2f\xfd":
        return None
    fhd = frame[4]
    fcs_flag, single, dict_flag = fhd >> 6, (fhd >> 5) & 1, fhd & 3
    pos = 5 + (0 if single else 1) + (0, 1, 2, 4)[dict_flag]
    n = (1 if single else 0, 2, 4, 8)[fcs_flag]
    if n == 0:
        return None
    v = int.from_bytes(frame[pos:pos + n], "little")
    return v + 256 if n == 2 else v


def zstd(frame):
    paths.game_dir()                 # clear error when WT_GAME_DIR is not set
    handle = os.add_dll_directory(str(CDK))
    try:
        lib = ctypes.CDLL(str(CDK / "daKernel-dev.dll"))
        fn = getattr(lib, "?zstd_decompress@@YA_KPEAX_KPEBX1@Z")
        fn.argtypes = (ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t)
        fn.restype = ctypes.c_size_t
        size = _zstd_content_size(frame) or 64 << 20
        dest = ctypes.create_string_buffer(size)
        got = fn(dest, size, frame, len(frame))
        if got == 0 or got > size:
            raise ValueError(f"zstd_decompress returned {got}")
        return dest.raw[:got]
    finally:
        handle.close()


def lnav_raw(level_bin):
    data = Path(level_bin).read_bytes()
    found = [p for tag, _, p in blocks(data) if tag == b"Lnav"]
    if not found:
        return None
    p = found[0]
    # u32: decompressed size (low 30 bits) | codec flag 0x40 in the high byte; then the zstd frame and block padding
    size = struct.unpack_from("<I", p, 0)[0] & 0x3FFFFFFF
    if p[4:8] != b"\x28\xb5\x2f\xfd":
        raise ValueError("no zstd frame in Lnav block")
    body = p[4:].rstrip(b"\0")
    last = None
    for extra in range(0, 9):                # the frame may itself end in zero bytes
        try:
            raw = zstd(body + b"\0" * extra)
            if len(raw) != size:
                raise ValueError(f"decoded {len(raw)} bytes, header says {size}")
            return raw
        except ValueError as e:
            last = e
    raise last


def parse(raw):
    """-> dict(verts [(x, z)], polys [(vert idx tuple, nei tuple)], header)"""
    magic = raw[:4]
    version = struct.unpack_from("<i", raw, 4)[0]
    pc, vc = struct.unpack_from("<2i", raw, 24)
    verts = struct.unpack_from(f"<{3 * vc}f", raw, 100)
    V = [(verts[3 * i], verts[3 * i + 2]) for i in range(vc)]
    off = 100 + 12 * vc
    P = []
    for _ in range(pc):
        vs = struct.unpack_from("<6H", raw, off + 4)
        ns = struct.unpack_from("<6H", raw, off + 16)
        n = raw[off + 30]
        P.append((vs[:n], ns[:n]))
        off += 32
    return dict(magic=magic, version=version, verts=V, polys=P)


def level_path(level):
    return paths.game_dir() / "levels" / f"{level}.bin"


if __name__ == "__main__":
    raw = lnav_raw(sys.argv[1])
    if raw is None:
        print("no Lnav block")
        sys.exit(1)
    d = parse(raw)
    print(f"{len(raw)} bytes, magic {d['magic']!r} v{d['version']}, {len(d['polys'])} polys, {len(d['verts'])} verts")
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_bytes(raw)
