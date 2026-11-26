"""What a lanes_summary.json's notes say, grouped, and whether lanes or routes are missing because of them.
usage: python tools/notes_summary.py <lanes_summary.json>"""
import json
import re
import sys
from collections import Counter

s = json.loads(open(sys.argv[1], encoding="utf-8").read())
notes = s.get("notes", [])
kinds = Counter(re.sub(r"\d+(\.\d+)?", "#", n.split(": ", 1)[1] if ": " in n else n) for n in notes)
print(f"notes {len(notes)}")
for k, v in kinds.most_common():
    print(f"  {v:4d}  {k}")
print("lanes:", s.get("lanes"))
print("dropped_lanes (lanes found by the search but no driving curve):", s.get("dropped_lanes"))
print("no_lane (ends the search found no route to):", s.get("no_lane"))
print("limits:", s.get("limits"))
print("routes:", s.get("routes"), "targets:", s.get("targets"))
nc = [n for n in notes if "no driving curve" in n]
print(f"'no driving curve' notes: {len(nc)}", nc[:10])
rej = [n for n in notes if n.endswith("rejected")]
print(f"'rejected' notes: {len(rej)}; first: {rej[:6]}")
