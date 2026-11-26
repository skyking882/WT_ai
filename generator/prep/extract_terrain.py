"""Decode the HM2 heightfield of a level file (levels/<level>.bin) with the game CDK's codec DLL. Read only.

Level file: magic "DBLD3x64", then blocks (u32 size | flags << 30, 4-char tag, payload). The HM2 block (tag
"\\0HM2") payload:
  @0  f32 cell size, f32 height offset, f32 height scale, f32 origin x, f32 origin z
  @20 u32 width (low 24 bits) | version << 24 (only version 2 is handled), u32 height (low 31 bits) | mirror flag << 31
  @44 u32 layout: bits 0-7 block shift (block = 2^shift samples square), bits 8-11 height-range hierarchy
      (sub-block = 2^n when bits 8-11 are set), bits 12-31 chunk size of the variance stream (0 = one chunk)
  @48 compressed chunks: u32 length | codec << 30 (1 zstd, 2 oodle), then the packed bytes. Chunk 0 decodes to the
      block info (count x 2 u16: block minimum, block range) + the hierarchy (16 bytes per node), followed (one chunk)
      or not (chunked) by the variance bytes; with a chunk size the variance stream follows in chunks of that size.
  Per block: the variance bytes are deltas; their running sum (mod 256) v gives the sample
      u16 = block minimum + (v * block range + 127) // 255, samples row-major within the block, blocks row-major.
  World: X = origin x + column * cell, Z = origin z + row * cell (row 0 = origin z), Y = offset + u16 * scale / 65535.
A scalar decode of 1028 samples (corners, block edges, random) is checked against the vector result.

  python prep/extract_terrain.py <level> [--out DIR]
      decodes levels/<level>.bin of the installed game (WT_GAME_DIR) and prints the grid; with --out writes
      DIR/height_u16.npz (array height_u16, [row z, column x]) and DIR/metadata.json (for inspection; prep/gen_terrain.py
      decodes the level itself and does not need these files).
"""
import argparse
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
import catalog_reader as cr  # noqa: E402
import paths  # noqa: E402


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


def header(p):
    cell, hmin, hscale, ox, oz = struct.unpack_from("<5f", p)
    width_version, height_mirror = struct.unpack_from("<II", p, 20)
    layout = struct.unpack_from("<I", p, 44)[0]
    return dict(cell_size_m=cell, height_offset_m=hmin, height_scale_m=hscale,
                origin_x_m=ox, origin_z_m=oz, width=width_version & 0xFFFFFF,
                height=height_mirror & 0x7FFFFFFF, version=width_version >> 24,
                mirror_flag=bool(height_mirror >> 31), layout=layout)


def decode(p, codec):
    """(u16 heights [rows, cols], header dict) of an HM2 payload."""
    h = header(p)
    if h["version"] != 2:
        raise ValueError(f"HM2 version {h['version']}: only version 2 is handled")
    w, rows, layout = h["width"], h["height"], h["layout"]
    shift = layout & 0xFF
    bw = 1 << shift
    if not 1 <= shift <= 5 or w % bw or rows % bw:
        raise ValueError("Unsupported block geometry")
    sub = 1 << ((layout >> 8) & 0xF) if layout & 0xF00 else 0
    levels = int(math.log2(w // sub)) if sub else 0
    if sub and (w // sub != 1 << levels):
        raise ValueError("Unsupported height-range hierarchy")
    hierarchy_bytes = ((4 ** levels - 1) // 3) * 16
    count = (w // bw) * (rows // bw)
    info_bytes, variance_bytes = count * 4, w * rows
    chunk_size = layout & ~0xFFF
    if chunk_size:
        sizes = [info_bytes + hierarchy_bytes]
        sizes += [min(chunk_size, variance_bytes - i) for i in range(0, variance_bytes, chunk_size)]
    else:
        sizes = [info_bytes + variance_bytes + hierarchy_bytes]
    decoded, audit = [], []
    pos = 48
    for index, expected in enumerate(sizes):
        word = struct.unpack_from("<I", p, pos)[0]
        length, kind = word & 0x3FFFFFFF, word >> 30
        end = pos + 4 + length
        if end > len(p) or length == 0:
            raise ValueError("Invalid compressed chunk framing")
        if kind not in (1, 2):
            raise ValueError(f"Unsupported codec flag {kind}")
        decoded.append(codec.decompress(p[pos + 4:end], size=expected, oodle=kind == 2))
        audit.append(dict(index=index, codec_flag=kind, packed_bytes=length, decoded_bytes=expected))
        pos = end
    info = np.frombuffer(decoded[0], dtype="<u2", count=count * 2).reshape(count, 2)
    variance_stream = b"".join(decoded[1:]) if chunk_size else decoded[0][info_bytes:info_bytes + variance_bytes]
    if len(variance_stream) != variance_bytes:
        raise ValueError("Unexpected variance byte count")
    deltas = np.frombuffer(variance_stream, dtype=np.uint8).reshape(count, bw * bw)
    variance = np.cumsum(deltas, axis=1, dtype=np.uint16).astype(np.uint8)
    raw = (info[:, 0].astype(np.uint32)[:, None]
           + (variance.astype(np.uint32) * info[:, 1].astype(np.uint32)[:, None] + 127) // 255)
    if raw.max() > 65535:
        raise ValueError("Height overflow")
    raw = raw.astype(np.uint16).reshape(rows // bw, w // bw, bw, bw)
    raw = raw.transpose(0, 2, 1, 3).reshape(rows, w)
    # independent scalar decode of corners, block edges and random samples against the vector result
    points = [(0, 0), (w - 1, rows - 1), (bw - 1, bw - 1), (bw, bw)]
    rng = np.random.default_rng(12345)
    points += list(zip(rng.integers(0, w, 1024), rng.integers(0, rows, 1024)))
    for x, z in points:
        block = int(x // bw + (z // bw) * (w // bw))
        inner = int(x % bw + (z % bw) * bw)
        v = sum(int(a) for a in deltas[block, :inner + 1]) & 255
        if int(raw[z, x]) != int(info[block, 0]) + (v * int(info[block, 1]) + 127) // 255:
            raise ValueError("Scalar/vector reconstruction mismatch")
    h["validation"] = dict(chunks=audit, scalar_samples_passed=len(points))
    return raw, h


def level_hm2(level, codec=None):
    """(u16 heights, header) of levels/<level>.bin of the installed game."""
    path = paths.game_dir() / "levels" / f"{level}.bin"
    data = path.read_bytes()
    found = [p for tag, _, p in blocks(data) if tag == b"\0HM2"]
    if len(found) != 1:
        raise ValueError(f"{path.name}: expected exactly one HM2 block, found {len(found)}")
    return decode(found[0], codec or cr.Codecs(str(paths.cdk_dir())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("level", help="level name, e.g. avg_eastern_europe")
    ap.add_argument("--out", type=Path, help="write height_u16.npz and metadata.json here (a new folder)")
    a = ap.parse_args()
    raw, h = level_hm2(a.level)
    meters = np.float32(h["height_offset_m"]) + raw.astype(np.float32) * np.float32(h["height_scale_m"] / 65535)
    h.update(level=a.level, height_min_m=float(meters.min()), height_max_m=float(meters.max()),
             height_formula="Y_m = height_offset_m + uint16 * height_scale_m / 65535",
             array_order="[row_z, column_x]; row 0 = origin_z",
             coordinate_formula="X = origin_x_m + column * cell_size_m; Z = origin_z_m + row * cell_size_m")
    if a.out:
        if a.out.exists():
            raise SystemExit(f"{a.out} exists: choose a new folder")
        a.out.mkdir(parents=True)
        np.savez_compressed(a.out / "height_u16.npz", height_u16=raw)
        (a.out / "metadata.json").write_text(json.dumps(h, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: h[k] for k in ("level", "width", "height", "cell_size_m", "origin_x_m", "origin_z_m",
                                        "height_min_m", "height_max_m")}, indent=1))


if __name__ == "__main__":
    main()
