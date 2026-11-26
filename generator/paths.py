"""The one place that defines the tree's root and every location in it. Everything is relative to this file, so the
tree can be copied anywhere (Windows or Linux). File names below are written exactly as they are on disk (all lower
case); keep them that way on a case-sensitive file system.

  gen/    the generator (lanes + network), Windows and Linux
  prep/   base-layer builder (Windows only: reads the installed game's level files through the game's own codec DLL)
  data/   inputs (see README.txt)
  out/    outputs, one folder per mission (+ batch_report.txt)
  build/  compiled C helpers (s30_core.dll / s30_core.so, read_check[.exe]); rebuilt when a source is newer
"""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GEN = ROOT / "gen"
PREP = ROOT / "prep"
DATA = ROOT / "data"
OUT = ROOT / "out"
BUILD = ROOT / "build"

COMMON = DATA / "common"
BATTLE_AREAS = COMMON / "ground_battle_areas.json"       # zones, spawns, battle rectangle, level per mission
SPAWN_PROTECTION = COMMON / "spawn_protection.json"      # kill areas (spawn protection) per mission
DMG_BLK = COMMON / "rendinst_dmg.blk"                    # prep only: destruction table of the installed game (BLK text)
OBSTACLES = DATA / "obstacles"                           # prep only, optional: <mission>.bin, 2 m scans (WTOBS001)
TERRAIN = DATA / "terrain"                               # prep only: <mission>.bin (WTTER001, prep/gen_terrain.py)
LNAV = DATA / "lnav"                                     # prep only: <level>.bin, the level's road mesh
RUNS = DATA / "runs"                                     # logged battles: <run>/ground_plan.json, ground_movement.csv,
#                                                          units.csv (generator); ground_samples.csv (prep/gen_terrain.py)
SWEDEN_ROUTES = DATA / "sweden" / "routes.json"          # sweden_dom only: logged spawn poses with headings
CLOSED = DATA / "closed"                                 # optional <mission>.closed.json (lanes switched off by position)
MAPS = DATA / "maps"                                     # <mission>/: the base layers prep builds, the generator reads
BASE_FILES = ("map.npz", "sight.npz", "foliage.npz", "map_report.json")

# logged battles to leave out of the speed measurement (comma-separated run folder names), e.g. to reproduce a pack
# generated before those battles were logged; default none
RUNS_EXCLUDE = {r for r in os.environ.get("WT_RUNS_EXCLUDE", "").split(",") if r}

# the installed game (prep only, Windows): taken only from the environment variable WT_GAME_DIR; None when it is not
# set. Code that needs the game calls game_dir(), which stops with a clear message in that case.
GAME = Path(os.environ["WT_GAME_DIR"]) if os.environ.get("WT_GAME_DIR") else None


def game_dir():
    """The installed game's folder (env WT_GAME_DIR); a clear error when the variable is not set."""
    if GAME is None:
        raise RuntimeError("the environment variable WT_GAME_DIR is not set: set it to the folder of the installed "
                           "game (the one that holds levels/, content/ and WarThunderCDK/); only prep/ needs it")
    return GAME


def cdk_dir():
    """Folder of the game CDK's codec DLL (daKernel-dev.dll) inside the installed game."""
    return game_dir() / "WarThunderCDK" / "dagor_cdk" / "windows-x86_64"


def mission_out(mission):
    d = OUT / mission
    d.mkdir(parents=True, exist_ok=True)
    return d
