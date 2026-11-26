"""Calls the compiled helpers (build/s30_core.dll or .so) with plain ctypes (no numpy) on a fixed 64 x 64 test grid and
prints checksums, so the Windows and the Linux build can be compared line by line.
usage: python tools/core_selftest.py"""
import ctypes
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
import build_core  # noqa: E402

H = W = 64
lib = ctypes.CDLL(str(build_core.build()))
F = ctypes.c_float
I32 = ctypes.c_int32
mark = (ctypes.c_uint8 * (H * W))()
for r in range(H):
    for c in range(W):
        mark[r * W + c] = 1 if ((r * 7 + c * 13) % 23 == 0 or (20 <= r <= 22 and 5 <= c <= 50)) else 0
edt = (F * (H * W))()
lib.edt.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
lib.edt(mark, H, W, edt)
print(f"edt: sum {sum(edt):.4f} max {max(edt):.4f} at 100 {edt[100]:.4f}")
cost = (F * (H * W))()
for i in range(H * W):
    cost[i] = 0.0 if mark[i] else 1.0 + (i % 5) * 0.1
src = (I32 * 1)(0 * W + 1)
dist = (F * (H * W))()
par = (I32 * (H * W))()
lib.dijkstra.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int32, F, F,
                         ctypes.c_void_p, ctypes.c_void_p]
lib.dijkstra.restype = ctypes.c_int64
n = lib.dijkstra(cost, H, W, src, 1, -1, F(0.0), F(0.0), dist, par)
fin = [d for d in dist if d < 1e19]
print(f"dijkstra: return {n}, reached {len(fin)}, sum {sum(fin):.3f}, far corner {dist[H * W - 2]:.4f}, parent of it {par[H * W - 2]}")
disc = (ctypes.c_uint8 * (H * W))()
cen = (I32 * 2)(10 * W + 10, 40 * W + 40)
lib.geodisc.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, F, ctypes.c_void_p]
lib.geodisc(cost, H, W, cen, 2, F(8.0), disc)
print(f"geodisc: cells {sum(disc)}")
out = (I32 * 200)()
lib.chain.argtypes = [ctypes.c_void_p, ctypes.c_int32, ctypes.c_void_p, ctypes.c_int]
lib.chain.restype = ctypes.c_int
k = lib.chain(par, H * W - 2, out, 200)
print(f"chain: length {k}, first {list(out[:3])}, last {out[k - 1] if k > 0 else None}")
print(f"library {build_core.lib_path().name} on {sys.platform}")
