"""Base layers of a mission -> data/maps/<mission>/ (map.npz, sight.npz, foliage.npz, map_report.json, ri_v3 raster).

WINDOWS ONLY, needs the installed game (paths.GAME, env WT_GAME_DIR): the rendinst placements of levels/<level>.bin and
the collision resources of content/base/res are decompressed with the game CDK's daKernel-dev.dll (catalog_reader).
The generator (run_map.py, any system) only reads the result. The two steps run as separate processes:
build_ri_s30.py <mission> (0.25 m object raster, material classes 0-5), then build_map30.py <mission>
(merged class map, terrain cost, sight layers, road raster from the Lnav mesh, foliage).
Needs data/common/ground_battle_areas.json, data/common/rendinst_dmg.blk and data/terrain/<mission>.bin
(prep/gen_terrain.py); data/lnav/<level>.bin and data/obstacles/<mission>.bin are optional (data/README.txt).
usage: python prep/prep_base.py <mission> [<mission> ...]
"""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
import paths  # noqa: E402


def prep(mission):
    t0 = time.time()
    for need, how in ((paths.BATTLE_AREAS, "prep/build_battle_areas.py"), (paths.DMG_BLK, "prep/blk_to_text.py"),
                      (paths.TERRAIN / f"{mission}.bin", f"prep/gen_terrain.py {mission}")):
        if not need.is_file():
            raise RuntimeError(f"missing {need.relative_to(ROOT).as_posix()} (write it with {how})")
    if not (paths.OBSTACLES / f"{mission}.bin").is_file():
        print(f"{mission}: no data/obstacles/{mission}.bin: building without the obstacle scan", flush=True)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(HERE), str(paths.GEN), str(ROOT)]))
    od = paths.mission_out(mission)
    for script in ("build_ri_s30.py", "build_map30.py"):
        r = subprocess.run([sys.executable, str(HERE / script), mission], cwd=str(HERE), env=env, capture_output=True,
                           text=True)
        (od / f"prep_{script[:-3]}.log").write_text(r.stdout + r.stderr, encoding="utf-8")
        if r.returncode:
            raise RuntimeError(f"{script} failed ({r.returncode}): {r.stderr.strip().splitlines()[-1:]}")
    dst = paths.MAPS / mission
    dst.mkdir(parents=True, exist_ok=True)
    for f in paths.BASE_FILES:
        shutil.copy2(od / f, dst / f)
    level = json.loads(paths.BATTLE_AREAS.read_text(encoding="utf-8"))[mission]["level"]
    ri = od / "ri_v3" / f"{level}_ri_raster.npz"
    if ri.is_file():
        (dst / "ri_v3").mkdir(exist_ok=True)
        shutil.copy2(ri, dst / "ri_v3" / ri.name)
    return round(time.time() - t0, 1)


def main():
    if os.name != "nt":
        raise SystemExit("prep_base.py needs Windows and the installed game (daKernel-dev.dll); the base layers are in data/maps/")
    for m in sys.argv[1:]:
        try:
            print(f"{m}: base layers in {prep(m)} s", flush=True)
        except Exception as e:  # noqa: BLE001 (report and go on with the next mission)
            print(f"{m}: FAILED {e}", flush=True)


if __name__ == "__main__":
    main()
