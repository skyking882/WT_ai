"""Strategy analysis basics: paths, every assumption (with its reason), mission data loaders, C helper.

Nothing map-specific is written here by hand: a mission name selects files; zones, spawns, kill areas and the
battle rectangle come from the mission's own data (ground_battle_areas.json, spawn_protection.json).
"""
import ctypes
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
# every location comes from the tree's paths.py (one level up); nothing absolute, nothing outside the tree
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
import paths  # noqa: E402

RUNS_DIR = paths.RUNS
S18 = paths.PREP                 # prep: the game-file readers (build_ri_obstacles, build_ri_raster, colres)
LNAV_DIR = paths.LNAV
PROTECT = paths.SPAWN_PROTECTION
DMG_BLK = paths.DMG_BLK
PREMERGED = {}   # empty: every mission's merged map is built here (bridge lanes, trench lanes)

# ---------------------------------------------------------------- hull and map
HULL_L, HULL_W, SIDE_MARGIN = 7.4, 3.6, 0.5
REQ_CLR = HULL_W / 2 + SIDE_MARGIN            # 2.3 m: hull centre may stand where clearance >= this
TURN_CLR = HULL_L / 2 + 0.6                   # 4.3 m: half hull length + 0.6 m, a hull can pivot here
CELL, SUB = 0.25, 2
HALF_DIAG = CELL * 0.5 * 2 ** 0.5
GT_HALF_M, GT_SLOPE_SOFT, GT_SLOPE_HARD, GT_WADE_M, GT_SLOPE_COST, GT_WATER_COST = 3.0, 0.15, 0.45, 1.0, 2.0, 2.0
FREE, SOFT, MEDIUM, HARD_RI, HARD_STATIC, UNKNOWN_RI, UNKNOWN_EDGE, WATER, STEEP, HARD_BLDG, DRV_OBJ, OVERHEAD = range(12)
CLASS_NAMES = ["free", "soft", "medium_knockable", "hard_ri", "hard_static_scan", "unknown_ri_scan",
               "unknown_edge_scan", "water", "steep", "hard_building_part", "drivable_object", "overhead"]
HARD_CODES = (HARD_RI, HARD_STATIC, UNKNOWN_RI, UNKNOWN_EDGE, WATER, STEEP, HARD_BLDG)
OBJ_HARD = (HARD_RI, HARD_STATIC, UNKNOWN_RI, UNKNOWN_EDGE, HARD_BLDG)    # hard cells that are objects (sight)
DRIVABLE_RI = ("stone_stairs_a", "stone_stairs_b", "stone_stairs_d")       # drivable objects (names proven in Sweden)
# name prefixes of rendinst objects treated as drivable surfaces: none. Trenches are obstacles (in logged battles
# tanks got stuck in them); narrow stretches get crossing lanes instead (prep/trench31.py).
DRIVABLE_PREFIX = ()
OVERHEAD_HEIGHT_CODE = 7
CLR_W, CLR_SPAN = 1.0, 4.0                     # route cost x (1 + clip(1 - (clr - 2.3) / 4, 0, 1))

# ---------------------------------------------------------------- assumptions of this analysis (base values)
BASE = dict(
    # speeds (m/s): measured from the logged battles of the level (analyse.speeds); these are fallbacks only
    v_road=None, v_off=None,
    v_road_fallback=9.0, v_off_fallback=7.0,
    # sight
    eye_h=2.5,          # observer: gunner / commander sight of a typical MBT (2.2-2.7 m above ground)
    target_h=1.6,       # target point: upper hull / turret ring of a tank (hull roof 1.4-1.8 m)
    hull_h=1.2,         # hull-down test: middle of the hull front
    turret_h=2.3,       # hull-down test: turret / gun height
    range_m=1000.0,     # sight range limit: Ground RB fights on this map are mostly under 1 km
    fol_depth=25.0,     # foliage blocks once the sight line has run this far through bush / tree crowns
    bush_r=1.5, bush_top=3.0,            # bush crown: radius 1.5 m, ground to 3 m (collision tops are 1.5-2 m)
    tree_r_frac=0.25, tree_bot_frac=0.35,  # tree crown: radius 0.25 x collision height, from 0.35 x height to top
    house_scale=1.0, house_add=0.0,      # object height = scan hit height x scale + add
    low_obj_h=2.5,      # hard object cell without any scan hit nearby: assumed 2.5 m (wall / fence height)
    # tactics
    setup_s=10.0,       # time to stop, turn the hull and lay the gun before the enemy appears
    min_expo_s=3.0,     # a stretch counts as watched when a passing tank is in view at least this long
    fire_margin_s=15.0, # fire-support position: own arrival not later than the enemy's by more than this
    support_r=200.0,    # "holding zone X": positions within this distance of X's centre
    early_extra_s=30.0, # enemy places counted against a position: reachable by the deadline + this
    approach_m=150.0,   # enemy "last approach" = last 150 m before the zone edge
    n_positions=12,     # interdiction positions kept per army
    pos_r=15.0,         # a position = cells within 15 m of its best cell (room for one or two tanks)
    pos_sep=60.0,       # positions at least 60 m apart
    conceal_r=4.0,      # concealed: a foliage cell within 4 m of the position
)
SIGHT_CELL = 2.0
SAMPLE_M = 10.0          # route samples for sight every 10 m
CORR_MAX = 5             # corridors per (spawn area, zone)
CORR_TIME = 2.0          # alternatives up to 2 x the fastest
CLOSE_R, CLOSE_STEP, CLOSE_START = 25.0, 30.0, 60.0   # street closing discs (25 m radius)
OVERLAP_R, OVERLAP_MAX = 10.0, 0.5       # distinct when <= 50% within 10 m of an earlier corridor
SPAWN_CLUSTER_M = 80.0
ZONE_LETTERS = "ABCDEFGH"


def out_dir(mission):
    return paths.mission_out(mission)


def dll():
    from build_core import build
    lib = ctypes.CDLL(str(build()))     # s30_core.dll (Windows) or s30_core.so (Linux)
    f32p = np.ctypeslib.ndpointer(np.float32, flags="C_CONTIGUOUS")
    i32p = np.ctypeslib.ndpointer(np.int32, flags="C_CONTIGUOUS")
    i64p = np.ctypeslib.ndpointer(np.int64, flags="C_CONTIGUOUS")
    u8p = np.ctypeslib.ndpointer(np.uint8, flags="C_CONTIGUOUS")
    ci, cf = ctypes.c_int, ctypes.c_float
    lib.edt.argtypes = [u8p, ci, ci, f32p]
    lib.edt.restype = None
    lib.dijkstra.argtypes = [f32p, ci, ci, i32p, ci, ctypes.c_int32, cf, cf, f32p, i32p]
    lib.dijkstra.restype = ctypes.c_int64
    lib.geodisc.argtypes = [f32p, ci, ci, i32p, ci, cf, u8p]
    lib.geodisc.restype = None
    lib.vs_accumulate.argtypes = [f32p, f32p, f32p, f32p, ci, ci, cf, ci, i32p, i32p, f32p, f32p, cf, cf, cf, cf,
                                  f32p, f32p]
    lib.vs_accumulate.restype = None
    lib.vs_query.argtypes = [f32p, f32p, f32p, f32p, ci, ci, cf, ci, i32p, f32p, cf, cf, cf, cf, f32p, f32p, i64p]
    lib.vs_query.restype = None
    lib.los_pairs.argtypes = [f32p, f32p, f32p, f32p, ci, ci, cf, ci, i32p, f32p, i32p, f32p, f32p]
    lib.los_pairs.restype = None
    lib.cover_search.argtypes = [f32p, u8p, i32p, ci, ci, i32p, ci, ctypes.c_int32, cf, cf, cf, cf, cf, ctypes.c_int32,
                                 i32p, f32p, f32p, i32p, f32p, f32p, f32p, ci, i32p, i32p, f32p, f32p, f32p, f32p]
    lib.cover_search.restype = ci
    lib.chain.argtypes = [i32p, ctypes.c_int32, i32p, ci]
    lib.chain.restype = ci
    return lib


def edt(lib, mark):
    mark = np.ascontiguousarray(mark, dtype=np.uint8)
    out = np.empty(mark.shape, np.float32)
    lib.edt(mark, mark.shape[0], mark.shape[1], out)
    return out


# ---------------------------------------------------------------- mission data
def battle_area(mission):
    d = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))
    return d[mission]


def mission_level(mission):
    return battle_area(mission)["level"]


CAPTURE_SETS = ("arcade", "realistic", "hardcore")   # the difficulty suffix of a capture name (..._01_realistic)
CAPTURE_MODE = "realistic"                            # the target battles are Ground Realistic


def capture_set(name):
    for s in CAPTURE_SETS:
        if name.endswith("_" + s):
            return s
    return ""


def zones(mission):
    """[(letter, x, z, r)] in the order of the capture names (_01 = A).
    A mission may list its capture areas once per difficulty set (poland_dom, western_europe_dom: _hardcore and
    _realistic). With one set (or no suffix) that set is used; with several, the CAPTURE_MODE set; anything else, or a
    letter twice inside the chosen set, stops with an error naming the mission and the sets found."""
    caps = battle_area(mission)["captures"]
    sets = {}
    for name in caps:
        sets.setdefault(capture_set(name), []).append(name)
    if len(sets) == 1:
        use = next(iter(sets))
    elif CAPTURE_MODE in sets:
        use = CAPTURE_MODE
    else:
        raise ValueError(f"{mission}: capture areas of several sets {dict(sorted(sets.items()))} and none is "
                         f"'{CAPTURE_MODE}': cannot tell which set the battle uses")
    out = []
    for name in sorted(sets[use]):
        x, z, r = caps[name]
        digits = "".join(ch for ch in name if ch.isdigit())
        out.append((ZONE_LETTERS[int(digits[-2:] if len(digits) >= 2 else digits) - 1], float(x), float(z), float(r)))
    letters = [o[0] for o in out]
    if len(set(letters)) != len(letters):
        raise ValueError(f"{mission}: capture set '{use}' {sorted(sets[use])} gives a zone letter twice {letters}")
    return sorted(out)


def spawns(mission):
    """army (1, 2) -> array (n, 2) of spawn points (t1 = army 1)."""
    sp = battle_area(mission)["spawns"]
    return {1: np.array(sp["t1"], float), 2: np.array(sp["t2"], float)}


def kill_areas(mission):
    """army -> list of (centre (x, z), inverse 2x2 of the shape's xz axes, radius in shape units, kind). The
    army's own spawn protection: the OTHER army may not stand there. Shapes are unit shapes scaled by tm
    (sphere / cylinder radius 0.5 in shape units, like the game's unit box -0.5..0.5)."""
    if not PROTECT.is_file():
        return {}
    d = json.loads(PROTECT.read_text(encoding="utf-8")).get(mission, {})
    out = {}
    for a, v in d.items():
        lst = []
        for s in v["shapes"]:
            tm = np.array(s["tm"], float)
            M = np.array([[tm[0][0], tm[2][0]], [tm[0][2], tm[2][2]]])
            lst.append(dict(c=(tm[3][0], tm[3][2]), inv=np.linalg.inv(M), r=0.5, kind=s["type"], name=s["name"]))
        out[int(a)] = lst
    return out


def inside_kill(ka, X, Z):
    m = np.zeros(np.shape(X), bool)
    for s in ka:
        dx, dz = X - s["c"][0], Z - s["c"][1]
        u = s["inv"][0, 0] * dx + s["inv"][0, 1] * dz
        v = s["inv"][1, 0] * dx + s["inv"][1, 1] * dz
        m |= np.hypot(u, v) <= s["r"]
    return m


def read_scan(mission):
    raw = (paths.OBSTACLES / f"{mission}.bin").read_bytes()
    assert raw[:8] == b"WTOBS001"
    nx, nz = struct.unpack_from("<II", raw, 8)
    x0, z0, step = struct.unpack_from("<3f", raw, 16)
    return np.frombuffer(raw, "<u2", nx * nz, 28).reshape(nz, nx), x0, z0, step


def _terrain(path):
    tb = path.read_bytes()
    assert tb[:8] == b"WTTER001"
    nx, nz = struct.unpack_from("<II", tb, 8)
    x0, z0, step, off, scale, water = struct.unpack_from("<6f", tb, 16)
    flags, = struct.unpack_from("<I", tb, 40)
    h = np.frombuffer(tb, "<u2", nx * nz, 48).reshape(nz, nx).astype(np.float32) * np.float32(scale) + np.float32(off)
    deck = np.frombuffer(tb, np.uint8, nx * nz, 48 + nx * nz * 2).reshape(nz, nx) if flags & 1 else None
    return dict(h=h, deck=deck, x0=x0, z0=z0, step=step, water=water)


def read_terrain(mission):
    """Terrain of the mission. If its water level is missing (NaN), take it from another mission's terrain file
    of the same level whose heights match within 0.05 m (same level geometry); the note says which."""
    T = _terrain(paths.TERRAIN / f"{mission}.bin")
    T["water_note"] = "own file"
    if not np.isfinite(T["water"]):
        level = mission_level(mission)
        areas = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))
        for m, v in sorted(areas.items()):
            p = paths.TERRAIN / f"{m}.bin"
            if m == mission or v.get("level") != level or not p.is_file():
                continue
            o = _terrain(p)
            if np.isfinite(o["water"]) and o["h"].shape == T["h"].shape and \
                    float(np.nanmax(np.abs(o["h"] - T["h"]))) < 0.05:
                T["water"] = o["water"]
                T["water_note"] = f"own file has no water level (NaN); {o['water']} taken from {m}.bin (same level, heights equal)"
                break
        else:
            T["water_note"] = "no water level anywhere: rivers not closed"
    return T


def bilinear(field, x0, z0, step, x, z, outside=np.nan):
    fx = (np.asarray(x, np.float64) - x0) / step
    fz = (np.asarray(z, np.float64) - z0) / step
    ok = (fx >= 0) & (fz >= 0) & (fx < field.shape[1] - 1) & (fz < field.shape[0] - 1)
    i = np.clip(np.floor(fx).astype(np.int64), 0, field.shape[1] - 2)
    j = np.clip(np.floor(fz).astype(np.int64), 0, field.shape[0] - 2)
    tx, tz = fx - i, fz - j
    v = ((1 - tz) * ((1 - tx) * field[j, i] + tx * field[j, i + 1])
         + tz * ((1 - tx) * field[j + 1, i] + tx * field[j + 1, i + 1]))
    return np.where(ok, v, outside)


def runs_of_level(level):
    out = []
    if not RUNS_DIR.is_dir():
        return out
    for d in sorted(RUNS_DIR.iterdir()):
        if d.name in paths.RUNS_EXCLUDE:
            continue
        p = d / "ground_plan.json"
        if not p.is_file():
            continue
        head = p.read_text(encoding="utf-8", errors="replace")[:400]
        if f'"level": "{level}"' in head:
            out.append(d.name)
    return out


def run_zones(run):
    import csv
    out = {}
    p = RUNS_DIR / run / "zones.csv"
    if not p.is_file():
        return []
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            out.setdefault(int(row["i"]), (float(row["x"]), float(row["z"])))
    return [out[k] for k in sorted(out)]


def bearing(dx, dz):
    """Compass bearing in degrees, clockwise from +z (top of every picture = map north)."""
    return (math.degrees(math.atan2(dx, dz)) + 360.0) % 360.0


def compass(b):
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int(((b + 22.5) % 360) // 45)]


def save_json(path, obj):
    def conv(o):
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))
    Path(path).write_text(json.dumps(obj, default=conv, separators=(",", ":")), encoding="utf-8")
