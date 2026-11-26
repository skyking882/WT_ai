"""Step 4: per army and zone, the facts side by side (no mixed score), and plain sentences per army and spawn area.
'Our' route is the RECOMMENDED corridor of each (spawn area, zone): least seconds under observation among the
corridors within 1.3 x the fastest time (sight30.recommend); the fastest is reported beside it."""
import numpy as np

CLOSE_S = 5.0        # two arrival readings within 5 s are called close


def _cd(c):
    return None if c is None else dict(id=c["id"], time=c["time"], expo_s=c["expo_s"], first_seen_t=c["first_seen_t"],
                                       narrowest=c["narrowest"], entry=c["entry"]["side"])


def sheet(arr, corrs, res, zones, areas, arr_rec):
    out = {}
    for a in (1, 2):
        b = 3 - a
        R = res[a]
        rows = {}
        for (L, x, z, r) in zones:
            cz = [c for c in corrs if c["army"] == a and c["zone"] == L]
            rec = [c for c in cz if c["role"] in ("recommended", "both")]
            fast = min(cz, key=lambda c: c["time"]) if cz else None
            # the army's recommended corridor = the recommended one of the area that arrives first by it
            ar_first = None
            if rec:
                ar_first = min(rec, key=lambda c: arr_rec[L].get(c["area"], {}).get("first") or 1e9)
            per_area = {}
            for ar in areas:
                if ar["army"] != a:
                    continue
                cc = sorted([c for c in cz if c["area"] == ar["id"]], key=lambda c: c["time"])
                per_area[ar["id"]] = dict(
                    corridors=[dict(id=c["id"], role=c["role"], time=c["time"], expo_s=c["expo_s"],
                                    first_seen_t=c["first_seen_t"], narrowest=c["narrowest"]["width"],
                                    gate_kind=c["narrowest"]["kind"], entry=c["entry"]["side"]) for c in cc],
                    entry_sides=sorted({c["entry"]["side"] for c in cc}))
            d = R["delay"][L]
            af, ae = arr[L][a], arr[L][b]
            rf, re_ = arr_rec[L][a], arr_rec[L][b]
            lead = lambda o, e: None if o is None or e is None else round(e - o, 1)
            rows[L] = dict(
                arrival_fastest=dict(own_first=af["first"], own_fifth=af["fifth"], enemy_first=ae["first"],
                                     enemy_fifth=ae["fifth"], lead_first_s=lead(af["first"], ae["first"]),
                                     lead_fifth_s=lead(af["fifth"], ae["fifth"])),
                arrival=dict(own_first=rf["first"], own_fifth=rf["fifth"], enemy_first=re_["first"], enemy_fifth=re_["fifth"],
                             lead_first_s=lead(rf["first"], re_["first"]), lead_fifth_s=lead(rf["fifth"], re_["fifth"]),
                             note="by the recommended corridors of both armies"),
                corridors=dict(total=len(cz), per_area=per_area),
                recommended_corridor=_cd(ar_first), fastest_corridor=_cd(fast),
                best_corridor=_cd(ar_first),
                holding_gives={Y: dict(best_seen=v["best_seen"], area_m2=v["area_m2"], approach_s=v["approach_s"])
                               for Y, v in R["matrix"][L].items()},
                delay=dict(enemy_corridors=d["enemy_corridors"], covered_by_1=d["covered_by_1"],
                           covered_by_2=d["covered_by_2"], enemy_recommended=d["enemy_recommended"],
                           recommended_covered_by_2=d["recommended_covered_by_2"], attacker_expo_s=d["attacker_expo_s"],
                           uncovered=d["uncovered"], fallback=d["fallback"], fallback_area_m2=d["fallback_area_m2"]),
                unwatched_enemy_corridors=R["unwatched"][L],
                fire_support_area_m2=R["fire"][L]["area_m2"],
                fire_support_areas=len(R["fire"][L]["areas"]))
        out[a] = rows
    return out


def sentences(arr_rec, corrs, res, sh, zones, areas, setup):
    """Two or three sentences per army and spawn area, each citing its numbers (recommended corridors)."""
    out = {}
    for ar in areas:
        a, b = ar["army"], 3 - ar["army"]
        R = res[a]
        facts = []
        for (L, x, z, r) in zones:
            mine = arr_rec[L].get(ar["id"], {}).get("first")
            en = arr_rec[L][b]["first"]
            cc = [c for c in corrs if c["area"] == ar["id"] and c["zone"] == L]
            rec = [c for c in cc if c["role"] in ("recommended", "both")]
            if mine is None or en is None or not rec:
                continue
            rc = rec[0]
            fast = min(cc, key=lambda c: c["time"])
            facts.append(dict(zone=L, mine=mine, enemy=en, lead=round(en - mine, 1), expo=rc["expo_s"], corr=rc["id"],
                              first_seen=rc["first_seen_t"], n=len(cc), gate=rc["narrowest"], fast=fast))
        if not facts:
            out[ar["id"]] = ["no zone reachable from this area"]
            continue
        facts.sort(key=lambda f: (-f["lead"], f["expo"]))
        cheap, costly = facts[0], facts[-1]
        s = []
        fz = cheap["fast"]
        alt = "" if fz["id"] == cheap["corr"] else (f" (the fastest, {fz['id']}, is {[c for c in corrs if c['id'] == cheap['corr']][0]['time'] - fz['time']:.0f} s quicker"
                                                     f" but seen {fz['expo_s']:.0f} s)")
        txt = (f"From {ar['id']}, zone {cheap['zone']} is the cheapest to take: by the recommended corridor {cheap['corr']} our "
               f"first tank arrives at {cheap['mine']:.0f} s, {abs(cheap['lead']):.0f} s {'before' if cheap['lead'] >= 0 else 'after'} "
               f"the enemy's first ({cheap['enemy']:.0f} s), seen from early enemy positions for {cheap['expo']:.0f} s{alt}")
        if abs(cheap["lead"]) < CLOSE_S:
            txt += f"; a lead or lag under {CLOSE_S:.0f} s is a close reading: the race for {cheap['zone']} is even"
        if len(facts) > 1 and abs(facts[1]["lead"] - cheap["lead"]) < CLOSE_S:
            f2 = facts[1]
            txt += (f"; close reading: zone {f2['zone']} leads by {f2['lead']:.0f} s, only {abs(f2['lead'] - cheap['lead']):.0f} s "
                    f"less, its corridor seen {f2['expo']:.0f} s")
        s.append(txt + ".")
        if costly is not cheap:
            s.append(f"Zone {costly['zone']} is the costliest: by {costly['corr']} our first tank from {ar['id']} needs "
                     f"{costly['mine']:.0f} s against the enemy's {costly['enemy']:.0f} s ({costly['lead']:+.0f} s), out of "
                     f"{costly['n']} distinct corridor(s), narrowing to {costly['gate']['width']:.1f} m ({costly['gate']['kind']}) "
                     f"and seen for {costly['expo']:.0f} s.")
        lab = {r["label"]: r for r in R["interdiction_M"] + R["interdiction_F"]}
        for cl, what in (("M", "main routes (recommended / fastest corridors)"), ("F", "flank routes (other corridors within 2x the fastest time)")):
            lst = R["per_spawn_area"][ar["id"]][cl]
            if not lst:
                s.append(f"From {ar['id']}, no class-{cl} area ({what}) is reached first from here within 60 s of the start.")
                continue
            p = lab[lst[0]]
            cz = [c for c in p["cuts"] if c["cls"] == cl]
            ex = [c["expo_s"] for c in cz]
            rg = f" at {p['range_m'][0]}-{p['range_m'][1]} m" if p.get("range_m") else ""
            others = f"; next best {', '.join(lst[1:])}" if len(lst) > 1 else ""
            s.append(f"From {ar['id']}, {'one or two tanks' if p['tanks'] >= 2 else 'one tank'} at {p['label']} (a {p['length_m']:.0f} m "
                     f"band around ({p['x']:.0f}, {p['z']:.0f}), room for {p['tanks']}), reached at {p['reach_by'][ar['id']]:.0f} s and in "
                     f"place by {p['by']:.0f} s, can watch {len(cz)} enemy {what.split(' (')[0]} into {'/'.join(p['zones'])} "
                     f"({', '.join(c['corridor'] for c in cz[:4])}{'...' if len(cz) > 4 else ''}) for {min(ex):.0f}-{max(ex):.0f} s each{rg}"
                     f"{', mostly hull-down' if p.get('hull_down_share', 0) >= 0.5 else ''}"
                     f"{', mostly concealed' if p['concealed_share'] >= 0.5 else ''}{others}.")
        out[ar["id"]] = s
    return out


def numbers(sh, res, zones):
    """The step-4 numbers that the sensitivity table prints side by side (base vs variant)."""
    n = {}
    for a in (1, 2):
        for (L, *_r) in zones:
            row = sh[a][L]
            n[f"army{a}.{L}.lead_first_s"] = row["arrival"]["lead_first_s"]
            n[f"army{a}.{L}.lead_first_fastest_s"] = row["arrival_fastest"]["lead_first_s"]
            n[f"army{a}.{L}.recommended"] = row["recommended_corridor"]["id"] if row["recommended_corridor"] else None
            n[f"army{a}.{L}.recommended_seen_s"] = row["recommended_corridor"]["expo_s"] if row["recommended_corridor"] else None
            n[f"army{a}.{L}.delay_2_tanks"] = f"{row['delay']['covered_by_2']}/{row['delay']['enemy_corridors']}"
            n[f"army{a}.{L}.fallback_m2"] = row["delay"]["fallback_area_m2"]
            for Y, v in row["holding_gives"].items():
                n[f"army{a}.{L}->{Y}.seen"] = v["best_seen"]
        n[f"army{a}.interdiction_areas"] = len(res[a]["interdiction_areas"])
    return n


def conclusions(sh, sent_facts, res, zones, areas, arr_rec, corrs):
    """Discrete conclusions compared across the sensitivity runs."""
    c = {}
    for a in (1, 2):
        for (L, *_r) in zones:
            row = sh[a][L]
            for key, ar in (("first", row["arrival"]), ("first_fastest", row["arrival_fastest"])):
                lead = ar["lead_first_s"]
                c[f"army{a}.{L}.{key}"] = "n/a" if lead is None else ("close" if abs(lead) < CLOSE_S else ("own" if lead > 0 else "enemy"))
            c[f"army{a}.{L}.corridors"] = row["corridors"]["total"]
            c[f"army{a}.{L}.recommended"] = row["recommended_corridor"]["id"] if row["recommended_corridor"] else None
            c[f"army{a}.{L}.recommended_watched"] = bool(row["recommended_corridor"] and row["recommended_corridor"]["expo_s"] > 0)
            c[f"army{a}.{L}.two_tanks_cover_all"] = row["delay"]["covered_by_2"] >= row["delay"]["enemy_corridors"] > 0
            c[f"army{a}.{L}.two_tanks_cover_recommended"] = row["delay"]["recommended_covered_by_2"] >= row["delay"]["enemy_recommended"] > 0
            c[f"army{a}.{L}.fallback"] = row["delay"]["fallback"]
            hg = row["holding_gives"]
            c[f"army{a}.{L}.best_support_onto"] = max(hg, key=lambda Y: hg[Y]["best_seen"]) if hg and max(
                v["best_seen"] for v in hg.values()) >= 0.25 else "none"
    for ar in areas:
        a, b = ar["army"], 3 - ar["army"]
        leads = []
        for (L, *_r) in zones:
            mine, en = arr_rec[L].get(ar["id"], {}).get("first"), arr_rec[L][b]["first"]
            if mine is not None and en is not None:
                cc = [x for x in corrs if x["area"] == ar["id"] and x["zone"] == L and x["role"] in ("recommended", "both")]
                leads.append((-(en - mine), cc[0]["expo_s"] if cc else 0, L))
        leads.sort()
        if leads:
            c[f"{ar['id']}.cheapest"] = leads[0][2]
            c[f"{ar['id']}.costliest"] = leads[-1][2]
    return c
