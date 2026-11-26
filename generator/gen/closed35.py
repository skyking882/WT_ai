"""Per-map closed list: switch lanes off by position, without changing rules or routes.

File (optional): data/closed/<mission>.closed.json
    {"closed": [{"x": 1288, "z": 2000, "r": 30, "why": "behind own spawn, watches nothing"}]}
Rule: a lane with any end slot within r m of a listed position is removed from plan.json; its id is removed from every
other lane's neighbours and opposes; the plans are re-checked (no share for a zone the army has no circle lane to).
Routes and targets of a removed lane stay in routes.bin.

The generator (lanes37.py) applies it as the last step before plan.json is written. On an existing pack:
    python closed35.py <mission>
rewrites out/<mission>/<mission>.plan.json and adds closed_by_list / plan_check_after_closed to lanes_summary.json.
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
import paths  # noqa: E402


def closed_file(mission):
    return paths.CLOSED / f"{mission}.closed.json"


def load(mission):
    p = closed_file(mission)
    if not p.is_file():
        return []
    return json.loads(p.read_text(encoding="utf-8")).get("closed", [])


def plan_check(pack):
    out = []
    circ = {}
    for l in pack["lanes"]:
        if l["end"] == "circle":
            circ.setdefault(str(l["army"]), set()).add(l["serves"])
    for a, pls in pack["plans"].items():
        for pl in pls:
            for L, v in pl["zones"].items():
                if v["share"] > 0 and L not in circ.get(str(a), set()):
                    out.append(f"army {a} plan {pl['kind']}: share {v['share']} for zone {L} without a circle lane")
    return out


def apply(pack, entries):
    """Removes the listed lanes from pack (in place). -> report dict."""
    report = dict(file_entries=len(entries), removed=[], unmatched=[])
    gone = set()
    hit = [False] * len(entries)
    for l in pack["lanes"]:
        for k, e in enumerate(entries):
            if any(math.hypot(s["x"] - e["x"], s["z"] - e["z"]) <= e["r"] for s in l["slots"]):
                gone.add(l["id"])
                hit[k] = True
                report["removed"].append(dict(lane=l["id"], entry=e))
                break
    pack["lanes"] = [l for l in pack["lanes"] if l["id"] not in gone]
    for l in pack["lanes"]:
        l["neighbours"] = [n for n in l["neighbours"] if n not in gone]
        if "opposes" in l:
            l["opposes"] = [n for n in l["opposes"] if n not in gone]
    report["unmatched"] = [e for e, h in zip(entries, hit) if not h]
    report["plan_check"] = plan_check(pack)
    return report


def main():
    mission = sys.argv[1]
    od = paths.OUT / mission
    pp = od / f"{mission}.plan.json"
    pack = json.loads(pp.read_text(encoding="utf-8"))
    rep = apply(pack, load(mission))
    pp.write_text(json.dumps(pack, indent=1), encoding="utf-8")
    sp = od / "lanes_summary.json"
    if sp.is_file():
        s = json.loads(sp.read_text(encoding="utf-8"))
        s["closed_by_list"] = rep
        s["plan_check_after_closed"] = rep["plan_check"]
        sp.write_text(json.dumps(s, indent=1, default=str), encoding="utf-8")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
