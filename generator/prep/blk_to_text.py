"""One binary BLK from a game archive as BLK text (strict dialect). Prep only; needs the third-party vromfs / blk
Python packages (prep/blk_archive.py). Read only on the game side.

usage: python prep/blk_to_text.py [--vromfs ARCHIVE] [--path INNER] [--out FILE]
  defaults: ARCHIVE = <game>/aces.vromfs.bin (env WT_GAME_DIR), INNER = config/rendinst_dmg.blk,
            FILE = data/common/rendinst_dmg.blk (paths.DMG_BLK) -- the destruction table prep reads
            (prep/build_ri_obstacles.py:parse_dmg).
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
import paths  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vromfs", type=Path, help="archive (default <game>/aces.vromfs.bin)")
    ap.add_argument("--path", default="config/rendinst_dmg.blk", help="file inside the archive")
    ap.add_argument("--out", type=Path, default=paths.DMG_BLK)
    a = ap.parse_args()
    archive_path = a.vromfs or paths.game_dir() / "aces.vromfs.bin"
    import blk_archive
    arc = blk_archive.Archive(archive_path)
    text = arc.text(a.path, strict=True)
    if text is None:
        raise SystemExit(f"{a.path} is not in {archive_path.name}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(text, encoding="utf-8")
    print(f"{a.path} of {archive_path.name} -> {a.out} ({len(text)} characters)")


if __name__ == "__main__":
    main()
