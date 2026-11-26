"""The chain for a list of missions, one after the other; a failed mission does not stop the batch.

usage: python run_all.py [mission ...] [--workers N] [--check]
       (no missions: every mission with base layers in data/maps/)

Each mission runs as its own process (run_map.py). Writes out/batch_report.txt: per mission ok / failed and why, the
time of each phase, lane counts, the limits line, read_check, the network checks and the pack sizes.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import paths  # noqa: E402


def main():
    args = sys.argv[1:]
    workers = None
    if "--workers" in args:
        i = args.index("--workers")
        workers = args[i + 1]
        del args[i:i + 2]
    check = "--check" in args
    missions = [a for a in args if not a.startswith("--")]
    if not missions:
        missions = sorted(d.name for d in paths.MAPS.iterdir() if (d / "map.npz").is_file())
    paths.OUT.mkdir(parents=True, exist_ok=True)
    lines = [f"batch {time.strftime('%Y-%m-%d %H:%M')} on {sys.platform}, python {sys.version.split()[0]}, "
             f"{'check only' if check else 'full chain'}, missions: {' '.join(missions)}", ""]
    t_all = time.time()
    for m in missions:
        cmd = [sys.executable, str(ROOT / "run_map.py"), m] + (["--workers", workers] if workers else []) + \
              (["--check"] if check else [])
        t = time.time()
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, encoding="utf-8", errors="replace")
        wall = round(time.time() - t, 1)
        od = paths.OUT / m
        if check:
            p = od / "check.json"
            c = json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
            lines.append(f"== {m}: check {'ok' if r.returncode == 0 else 'FAILED'} ({wall} s)")
            for k in ("missing", "unusual"):
                for v in c.get(k, []):
                    lines.append(f"   {k}: {v}")
            if not c:
                lines.append("   " + " | ".join((r.stderr or r.stdout).strip().splitlines()[-4:]))
        else:
            p = od / "run_report.json"
            rep = json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
            st = rep.get("status", "failed")
            lines.append(f"== {m}: {st.upper()} ({wall} s wall)")
            if st != "ok":
                lines.append(f"   why: {rep.get('reason') or ' | '.join((r.stderr or r.stdout).strip().splitlines()[-4:])}")
            lines.append(f"   phases: {rep.get('phases')}")
            lines.append(f"   lanes: {rep.get('lanes')}")
            lines.append(f"   limits: {rep.get('limits')}")
            for l in rep.get("read_check") or []:
                lines.append(f"   read_check: {l}")
            for l in rep.get("network") or []:
                lines.append(f"   network: {l}")
            lines.append(f"   pack: {rep.get('pack')}")
        lines.append("")
        (paths.OUT / "batch_report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"{m}: finished in {wall} s (exit {r.returncode})", flush=True)
    lines.append(f"batch time {round(time.time() - t_all, 1)} s")
    (paths.OUT / "batch_report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((paths.OUT / "batch_report.txt").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
