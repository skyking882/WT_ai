"""Plans (WTPLAN002): only the zone roles and shares, by plan31's balanced / two_point / main_effort rules
(plan31.build_plans, read here with its watch step switched off; no watch, no flank, no capacity fields)."""
import re

import plan31


def plans(base, areas, zones, corrs):
    old = plan31._watch
    plan31._watch = lambda pl, *a, **k: pl.__setitem__("watch", [])
    try:
        P = plan31.build_plans(base, areas, zones, corrs, {})
    finally:
        plan31._watch = old
    out = {}
    for a, lst in P.items():
        out[a] = []
        for pl in lst:
            why = re.sub(r" (Flank variants for|No flank variants).*$", "", pl["why"]).replace(" before watch tanks", "")
            why = why.replace(", flank variants used where one exists", "")
            out[a].append(dict(id=pl["id"], kind=pl["kind"], weight=pl["weight"], why=why,
                               zones={L: dict(role=v["role"], share=round(v["share"], 4)) for L, v in pl["zones"].items()}))
    return out
