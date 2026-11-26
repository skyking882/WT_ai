"""The exact object raster (build_ri_raster.py, imported unchanged) with these material classes
  0 free, 1 drivable object (common30.DRIVABLE_RI), 2 soft (bush / tree / small impulse), 3 knockable medium,
  4 building part, 5 hard
for one mission's scan rectangle (the obstacle scan's header, or without a scan the rectangle it would have:
prep/scan_area.py). Output: out/<mission>/ri_v3/<level>_ri_raster.npz.
usage: build_ri_s30.py <mission>
"""
import sys

import common30 as C

sys.path.insert(0, str(C.S18))
import build_ri_obstacles as bro  # noqa: E402
import build_ri_raster as brr  # noqa: E402

_parse = bro.parse_dmg


class DmgTable(dict):
    """The destruction table; names starting with a common30.DRIVABLE_PREFIX are drivable surfaces."""
    def get(self, key, default=None):
        k = key.lower()
        if any(k.startswith(p) for p in C.DRIVABLE_PREFIX):
            return dict(super().get(k) or {}, _drivable="yes")
        return super().get(k, default)


def parse_dmg(path):
    d = DmgTable(_parse(path))
    for n in C.DRIVABLE_RI:
        dict.__setitem__(d, n.lower(), dict(dict.get(d, n.lower(), {}), _drivable="yes"))
    return d


def material_class(cls, impulse, params, soft):
    if params and params.get("_drivable") == "yes":
        return 1
    if cls in ("bush", "tree"):
        return 2
    if cls == "impulse":
        return 2 if impulse <= soft else 3
    if cls == "unknown" and params and "hp" in params:
        return 3 if "physRes" in params else 4
    return 5


if __name__ == "__main__":
    mission = sys.argv[1]
    level = C.mission_level(mission)
    bro.parse_dmg = parse_dmg
    brr.material_class = material_class
    import scan_area   # --scan <file> when the obstacle scan exists, else --box (the rectangle the scan would have)
    sys.argv = [sys.argv[0], "--level", level] + scan_area.box_args(mission) + [
                "--dmg", str(C.DMG_BLK), "--out", str(C.out_dir(mission) / "ri_v3"),
                "--terrain", str(C.paths.TERRAIN / f"{mission}.bin")]
    brr.main()
