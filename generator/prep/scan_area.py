"""The 2 m grid prep works on, with or without an obstacle scan (data/obstacles/<mission>.bin, WTOBS001).

With the scan file: its header (nx, nz, x0, z0, step) and its cells, exactly as before.
Without it: the same header the scan would have. The scan covers the mission's battle rectangle
(ground_battle_areas.json "bbox") plus SCAN_MARGIN_M on every side at 2 m; its origin and far corner are the
rectangle +- the margin written with one decimal and read as float32, and the cell counts are
int((far - origin) / step) in float32. This rule reproduces the header of every scan file this tree was built with.
The cells are then all zero ("not known": no scan classes, no hits, no bridge decks).
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import paths  # noqa: E402

SCAN_MARGIN_M = 150.0      # margin of the scan around the battle rectangle
SCAN_STEP_M = 2.0          # scan cell
NO_SCAN_NOTE = ("no obstacle scan: static scene objects that are not placed instances (bridges and some structures) "
                "are missing")


def scan_path(mission):
    return paths.OBSTACLES / f"{mission}.bin"


def has_scan(mission):
    return scan_path(mission).is_file()


def box_header(mission):
    """(nx, nz, x0, z0, step) of the grid a scan of this mission has (header rule above)."""
    a = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))[mission]["bbox"]
    f = np.float32
    t = [float(f"{v:.1f}") for v in (a[0] - SCAN_MARGIN_M, a[1] - SCAN_MARGIN_M, a[2] + SCAN_MARGIN_M,
                                     a[3] + SCAN_MARGIN_M)]
    x0, z0, x1, z1 = (f(v) for v in t)
    step = f(SCAN_STEP_M)
    nx, nz = int((x1 - x0) / step), int((z1 - z0) / step)
    return nx, nz, float(x0), float(z0), float(step)


def header(mission):
    """(nx, nz, x0, z0, step, has_scan): from the scan file when present, else box_header()."""
    p = scan_path(mission)
    if p.is_file():
        head = p.read_bytes()[:28]
        assert head[:8] == b"WTOBS001", f"{p.name}: not a WTOBS001 file"
        nx, nz = struct.unpack_from("<II", head, 8)
        x0, z0, step = struct.unpack_from("<3f", head, 16)
        return nx, nz, x0, z0, step, True
    return box_header(mission) + (False,)


def cells(mission):
    """(S u16 (nz, nx), x0, z0, step, has_scan). Without a scan S is all zero."""
    nx, nz, x0, z0, step, scan = header(mission)
    if scan:
        raw = scan_path(mission).read_bytes()
        S = np.frombuffer(raw, "<u2", nx * nz, 28).reshape(nz, nx)
    else:
        S = np.zeros((nz, nx), np.uint16)
    return S, x0, z0, step, scan


def box_args(mission):
    """Command-line form of the grid for build_ri_raster.py / build_ri_obstacles.py: --scan <file> when the scan
    exists, else --box x0 z0 step nx nz."""
    nx, nz, x0, z0, step, scan = header(mission)
    if scan:
        return ["--scan", str(scan_path(mission))]
    return ["--box", repr(x0), repr(z0), repr(step), str(nx), str(nz)]


def grid_from_args(a):
    """(nx, nz, x0, z0, step, raw scan bytes or None) from parsed --scan / --box arguments."""
    if a.scan:
        raw = Path(a.scan).read_bytes()
        nx, nz = struct.unpack_from("<II", raw, 8)
        x0, z0, step = struct.unpack_from("<3f", raw, 16)
        return nx, nz, x0, z0, step, raw
    if not a.box:
        raise SystemExit("give --scan <obstacles/x.bin> or --box x0 z0 step nx nz")
    x0, z0, step, nx, nz = a.box
    return int(nx), int(nz), float(x0), float(z0), float(step), None
