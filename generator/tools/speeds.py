"""Measured speeds of a mission from data/runs, all runs and without the runs named on the command line.
usage: python tools/speeds.py <mission> [run to leave out ...]"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
import common30 as C  # noqa: E402
import field30 as F  # noqa: E402

m = sys.argv[1]
Fl = F.Field(m)
runs = C.runs_of_level(C.mission_level(m))
for label, rr in (("all", runs), ("without " + " ".join(sys.argv[2:]), [r for r in runs if r not in sys.argv[2:]])):
    s = F.log_speeds(Fl, rr)
    print(f"{label}: {len(rr)} runs, v_road {s['v_road']} v_off {s['v_off']}")
