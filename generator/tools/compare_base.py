"""Compare a mission's base layers in data/maps/<mission>/ with another folder (arrays exactly; map_report.json by key).
usage: python tools/compare_base.py <mission> <other folder>"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
m, other = sys.argv[1], Path(sys.argv[2])
a_dir = ROOT / "data" / "maps" / m
for f in ("map.npz", "sight.npz", "foliage.npz"):
    A, B = np.load(a_dir / f), np.load(other / f)
    for k in sorted(set(A.files) | set(B.files)):
        if k not in A.files or k not in B.files:
            print(f"{f}:{k} only in one")
            continue
        x, y = A[k], B[k]
        same = x.shape == y.shape and np.array_equal(x, y, equal_nan=(x.dtype.kind == "f"))
        print(f"{f}:{k} shape {x.shape} {'identical' if same else 'DIFFERENT'}")
ra = json.loads((a_dir / "map_report.json").read_text(encoding="utf-8"))
rb = json.loads((other / "map_report.json").read_text(encoding="utf-8"))
diff = [k for k in sorted(set(ra) | set(rb)) if ra.get(k) != rb.get(k)]
print("map_report.json keys that differ:", diff, {k: (ra.get(k), rb.get(k)) for k in diff if k == "seconds"})
