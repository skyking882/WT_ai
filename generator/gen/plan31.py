"""Plan pack (WTPLAN001): plan areas with slots, and the commander's plans from explicit rules on the step-4 facts.
All thresholds and weights are first values and live here."""
import math

import numpy as np

import common30 as C
import sight30

# ---------------------------------------------------------------- constants (first values)
TEAM = 16                      # shares are counted against a team of 16 tanks
W_BALANCED, W_TWO_POINT, W_MAIN_EFFORT = 1, 2, 2
DELAY_SHARE = 2 / 16           # two_point: the third zone held by two tanks ...
DELAY_COVER = 0.75             # ... when two tanks cover at least 75 % of the enemy's corridors into it ...
                               # ... and a covered fall-back position exists
ME_LEAD_MIN = -15.0            # main_effort: candidate zones lead by recommended routes >= -15 s
ME_SHARE = 0.5                 # main_effort: the chosen zone gets half the team
ME_SKIP_LEAD = -30.0           # main_effort: a zone more than 30 s behind is skipped
WATCH_SPARE_S = 3.0            # watch: an M area counts when in place with at least 3 s to spare (F guards: no threshold)
WATCH_MAX = 4                  # watch: at most four watch tanks per army
FLANK_MIN_SHARE = 4 / 16       # flank variants for a zone with at least four tanks
AREA_SLOTS, SLOT_SEP_M, SLOT_CLR_M = 4, 10.0, 4.3
CAND_CELLS = 24                # candidate cells per area for slot selection
SUPPORT_TANKS = 6              # per (army, zone): S slots for six tanks beyond the zone's capacity ...
SUPPORT_MAX_AREAS = 6          # ... from at most six fire-support areas ...
SUPPORT_SLOTS_PER_AREA = 6     # ... with at most six slots each (10 m apart)


def ang(dx, dz):
    return math.degrees(math.atan2(dz, dx))


def pack_areas(base, Fl, zones, areas):
    """Per spawn area its three best M and F areas (the analysis' per-spawn-area lists), per (army, zone) the two best
    fire-support areas (cls S). Slots: up to 4 cells (2 m, the sub-cell of 0.5 m with the largest clearance,
    >= 4.3 m), >= 10 m apart; hull-down cells first, then the seconds watched from the cell."""
    S = base["S"]
    res = base["res"]
    corrs = base["cache"]["corrs"]
    out = []
    zc = {L: (x, z, r) for L, x, z, r in zones}
    clr05 = np.where(Fl.valid05 & Fl.reach, Fl.clr05, 0)
    best05 = clr05.reshape(S.Hs, 4, S.Ws, 4).transpose(0, 2, 1, 3).reshape(S.Hs, S.Ws, 16)
    bidx = best05.argmax(-1)
    bclr = best05.max(-1)
    for a in (1, 2):
        R = res[a]
        lab = {r["label"]: r for r in R["interdiction_M"] + R["interdiction_F"]}
        enem = {c["id"]: c for c in corrs if c["army"] == 3 - a}
        chosen = []
        for ar in [x for x in areas if x["army"] == a]:
            for cl in ("M", "F"):
                for lb in R["per_spawn_area"][ar["id"]][cl]:
                    if lb not in [c["label"] for c in chosen]:
                        chosen.append(lab[lb])
        for r in chosen:
            cls_c = [c for c in r["cuts"] if c["cls"] == r["cls"]]
            main = max(cls_c, key=lambda c: c["expo_s"])
            out.append(_area(S, Fl, a, f"{a}{r['label']}", r["cls"], r, r["first_area"], r["own_t"], r["by"], r["zones"],
                             r["tanks"], [enem[c["corridor"]] for c in cls_c], enem[main["corridor"]], None,
                             bidx, bclr))
        arr = base["arr_rec"]
        for (L, x, z, rad) in zones:
            # support for the tanks beyond the zone's capacity: fire-support areas on the own side of the zone
            # (own arrival not later than the enemy's) first, then the others; nearest to the zone first; slots added
            # until SUPPORT_TANKS are placed or SUPPORT_MAX_AREAS areas are used
            fa = [r for r in R["fire"][L]["areas"]]
            fa.sort(key=lambda r: (not (r["enemy_t"] is None or r["own_t"] <= r["enemy_t"]), r["range_m"][0]))
            en = arr[L][3 - a]["first"]
            got, k = 0, 0
            for r in fa:
                if got >= SUPPORT_TANKS or k >= SUPPORT_MAX_AREAS:
                    break
                ar_ = _area(S, Fl, a, f"{a}S{L}{k + 1}", "S", r, r["first_area"], r["own_t"], en, [L], r["tanks"],
                            [], None, (x, z), bidx, bclr, zone_cells=base["zin"][L], max_slots=SUPPORT_SLOTS_PER_AREA)
                if not ar_["slots"]:
                    continue
                ar_["own_side"] = r["enemy_t"] is None or r["own_t"] <= r["enemy_t"]
                ar_["zone_dist_m"] = r["range_m"][0]
                out.append(ar_)
                got += len(ar_["slots"])
                k += 1
    return out


def _area(S, Fl, army, aid, cls, r, first, own_t, by_t, zones_, tanks, corr_list, main_corr, zone_xy, bidx, bclr,
          zone_cells=None, max_slots=None):
    max_slots = max_slots or AREA_SLOTS
    cells = np.asarray(r["cells"])
    ok = bclr.ravel()[cells] >= SLOT_CLR_M
    cand = cells[ok]
    P = S.P
    if zone_xy is None:
        sm = main_corr["samples"]
        qc = S.cell(sm["x"], sm["z"])
        # watch point: middle of the stretch of the longest-watched corridor seen from the area's best cell
        bc = int(S.cell(np.array([r["x"]]), np.array([r["z"]]))[0])     # the area's best cell (its label point)
        vis, _ = S.seen(bc, qc)
        if not vis.any():
            vis, _ = S.seen(int(cand[0]) if len(cand) else int(cells[0]), qc)
        iv = np.flatnonzero(vis)
        k = int(iv[len(iv) // 2]) if len(iv) else len(qc) // 2
        wx, wz = float(sm["x"][k]), float(sm["z"][k])
        targets = [S.cell(c["samples"]["x"], c["samples"]["z"]) for c in corr_list]
        dts = [c["samples"]["dt"] for c in corr_list]
    else:
        wx, wz = zone_xy
        targets = [zone_cells]
        dts = [np.ones(len(zone_cells))]
    cs = sight30.sample_cells(cand, CAND_CELLS) if len(cand) else cand
    scored = []
    for c in cs:
        c = int(c)
        seen_s, qall = 0.0, []
        for q, dt in zip(targets, dts):
            vis, _ = S.seen(c, q)
            seen_s += float(np.asarray(dt)[vis].sum())
            if vis.any():
                qall.append(q[vis])
        qall = np.unique(np.concatenate(qall)) if qall else np.zeros(0, np.int32)
        hd = S.hull_down(c, qall) if len(qall) else 0.0
        scored.append((hd >= 0.5, seen_s, c, hd))
    scored.sort(key=lambda t: (not t[0], -t[1]))
    slots = []
    for hdok, seen_s, c, hd in scored:
        sub = int(bidx.ravel()[c])
        j, i = divmod(c, S.Ws)
        r05, c05 = j * 4 + sub // 4, i * 4 + sub % 4
        x, z = Fl.X0 + (c05 + 0.5) * 0.5, Fl.Z0 + (r05 + 0.5) * 0.5
        if all(math.hypot(x - s["x"], z - s["z"]) >= SLOT_SEP_M for s in slots):
            slots.append(dict(x=round(float(x), 2), z=round(float(z), 2), cell=int(r05 * Fl.W5 + c05),
                              face_deg=round(ang(wx - x, wz - z), 1), hull_down=bool(hdok), seen_s=round(seen_s, 1),
                              clearance=round(float(Fl.clr05[r05, c05]), 2)))
        if len(slots) >= max_slots:
            break
    return dict(id=aid, army=army, cls=cls, first_area=first, own_t=own_t, by_t=by_t, zones=zones_,
                watch_x=round(wx, 1), watch_z=round(wz, 1),
                hull_down_share=round(float(np.mean([t[0] for t in scored])), 2) if scored else 0.0,
                tanks=tanks, slots=slots, label=r.get("label"), cells=cells)


# ---------------------------------------------------------------- plans
def _fmt(s):
    return f"{s * TEAM:.0f}/16"


def build_plans(base, areas, zones, corrs, pack_areas_by_id):
    sh = base["sheet"]
    res = base["res"]
    plans = {}
    for a in (1, 2):
        zl = [L for L, *_r in zones]
        lead = {L: sh[a][L]["arrival"]["lead_first_s"] for L in zl}
        lead = {L: (v if v is not None else -999.0) for L, v in lead.items()}
        flank_ok = {L: any(c["army"] == a and c["zone"] == L and c["variant"] > 0 for c in corrs) for L in zl}
        rowsum = {L: sum(v["best_seen"] for v in sh[a][L]["holding_gives"].values()) for L in zl}
        out = []
        n = len(zl)
        # any number of zones by the same rules. 1 zone: one plan, the zone main with the whole share (no other plan
        # can differ). 2 zones: balanced and main_effort (two_point needs a third zone to delay or skip). 3 zones: the
        # three plans. More than 3: balanced 1/n each; two_point: the two largest leads are the mains
        # and every other zone gets the third zone's delay / skip rule on its own (DELAY_SHARE each when delayed),
        # the mains split the rest; main_effort unchanged (the others contest or skip by the lead rule).
        if n == 1:
            L = zl[0]
            one = dict(id=f"{a}-single", kind="single", weight=W_BALANCED,
                       why=f"The map has one zone: {L} gets {_fmt(1.0)} before watch tanks.",
                       zones={L: dict(role="main", share=1.0)}, watch=[], flank={L: flank_ok[L]})
            extra = [one]
        else:
            # balanced
            pz = {L: dict(role="contest", share=round(1 / n, 4)) for L in zl}
            bal = dict(id=f"{a}-balanced", kind="balanced", weight=W_BALANCED,
                       why=f"Equal shares ({', '.join(f'{L} {_fmt(1 / n)}' for L in zl)} before watch tanks), flank variants used "
                           f"where one exists.",
                       zones=pz, watch=[], flank={L: flank_ok[L] for L in zl})
            _watch(bal, a, areas, res, corrs, pack_areas_by_id)
            for L in zl:
                bal["zones"][L]["share"] = round(bal["zones"][L]["share"], 4)
            out.append(bal)
            extra = []
        # two_point (3 zones or more)
        order = sorted(zl, key=lambda L: -lead[L])
        two = None
        if n >= 3:
            m1, m2, rest3 = order[0], order[1], order[2:]
            zr, clauses, dsum = {}, [], 0.0
            for third in rest3:
                d = sh[a][third]["delay"]
                cover = d["covered_by_2"] / d["enemy_corridors"] if d["enemy_corridors"] else 0.0
                delay = cover >= DELAY_COVER and d["fallback"]
                ds = DELAY_SHARE if delay else 0.0
                dsum += ds
                zr[third] = dict(role="delay" if delay else "skip", share=ds)
                cond = (f"two tanks cover {d['covered_by_2']}/{d['enemy_corridors']} = {cover:.0%} of the enemy's corridors into it "
                        f"({'meets' if cover >= DELAY_COVER else 'below'} the {DELAY_COVER:.0%} needed) and a covered fall-back "
                        f"{'exists' if d['fallback'] else 'is missing'}")
                clauses.append(f"{third} is " + (f"delayed by {_fmt(DELAY_SHARE)}: {cond}" if delay
                                                 else f"skipped: {cond} (both are needed for a delay)"))
            z2 = {m1: dict(role="main", share=(1 - dsum) / 2), m2: dict(role="main", share=(1 - dsum) / 2), **zr}
            why2 = (f"Mains {m1} and {m2} have the largest leads by recommended routes ({lead[m1]:+.0f} s, {lead[m2]:+.0f} s; "
                    + ", ".join(f"{t} {lead[t]:+.0f} s" for t in rest3) + "); " + "; ".join(clauses)
                    + f"; the mains split the rest ({_fmt((1 - dsum) / 2)} each before watch tanks).")
            two = dict(id=f"{a}-two_point", kind="two_point", weight=W_TWO_POINT, why=why2, zones=z2)
        # main_effort (2 zones or more)
        cands = [L for L in zl if lead[L] >= ME_LEAD_MIN] if n >= 2 else []
        me = None
        others = "the other two" if n == 3 else ("the other one" if n == 2 else "the others")
        if cands:
            mz = max(cands, key=lambda L: (round(rowsum[L], 2), lead[L]))     # ties: the larger lead
            rest = [L for L in zl if L != mz]
            skip = [L for L in rest if lead[L] < ME_SKIP_LEAD]
            cont = [L for L in rest if L not in skip]
            z3 = {mz: dict(role="main", share=ME_SHARE)}
            for L in rest:
                z3[L] = dict(role="skip", share=0.0) if L in skip else dict(role="contest", share=(1 - ME_SHARE) / len(cont))
            if not cont:
                z3[mz]["share"] = 1.0
            why3 = (f"{mz} gets {_fmt(z3[mz]['share'])} before watch tanks: among the zones with a lead of at least {ME_LEAD_MIN:+.0f} s "
                    f"({', '.join(f'{L} {lead[L]:+.0f} s' for L in cands)}) it gives the most fire onto {others} "
                    f"(sum of zone shares seen {rowsum[mz]:.2f}; "
                    + ", ".join(f"{L} {rowsum[L]:.2f}" for L in cands if L != mz)
                    + ("; equal sums, so the larger lead decides" if any(abs(rowsum[L] - rowsum[mz]) < 0.005 for L in cands if L != mz) else "")
                    + ")"
                    + (f"; {', '.join(skip)} skipped, more than {-ME_SKIP_LEAD:.0f} s behind ({', '.join(f'{lead[L]:+.0f} s' for L in skip)})"
                       if skip else "")
                    + (f"; {', '.join(cont)} contested with the rest split evenly" if cont else "") + ".")
            me = dict(id=f"{a}-main_effort", kind="main_effort", weight=W_MAIN_EFFORT, why=why3, zones=z3)
            # equal to two_point when it gives every zone the same share (roles aside)
            same = two is not None and all(abs(me["zones"][L]["share"] - two["zones"][L]["share"]) < 1e-6 for L in zl)
            if same:
                me = None
        for pl in extra + ([two] if two else []) + ([me] if me else []):
            _watch(pl, a, areas, res, corrs, pack_areas_by_id)
            for L in zl:
                pl["zones"][L]["share"] = round(pl["zones"][L]["share"], 4)
            pl["flank"] = {L: bool(flank_ok[L] and pl["zones"][L]["share"] >= FLANK_MIN_SHARE - 1e-9) for L in zl}
            fl = [L for L in zl if pl["flank"][L]]
            pl["why"] += (f" Flank variants for {', '.join(fl)} (a flank variant exists and the share is at least "
                          f"{_fmt(FLANK_MIN_SHARE)})." if fl else f" No flank variants (none with a share of at least {_fmt(FLANK_MIN_SHARE)}).")
            out.append(pl)
        for pl in out:
            tot = sum(v["share"] for v in pl["zones"].values()) + sum(w["count"] for w in pl["watch"]) / TEAM
            assert tot <= 1.0 + 1e-6, (pl["id"], tot)
        plans[str(a)] = out
    return plans


def _watch(pl, a, areas, res, corrs, pack_areas_by_id):
    R = res[a]
    lab = {r["label"]: r for r in R["interdiction_M"] + R["interdiction_F"]}
    watch, reasons, open_ = [], [], []
    # every plan; first every spawn area's flank guard (its best F area that it reaches
    # first, no spare-time threshold), then the M entries (spare >= 3 s); at most four per army, F entries first
    entries = []
    for cl in ("F", "M"):
        for ar in [x for x in areas if x["army"] == a]:
            lst = R["per_spawn_area"][ar["id"]]
            if not lst[cl]:
                if cl == "F":
                    open_.append(ar["id"])
                continue
            entries.append((cl, ar, lab[lst[cl][0]]))
    for cl, ar, r in entries:
        if len(watch) >= WATCH_MAX:
            continue
        if True:
            ncls = r["class_corridors"]
            if cl == "M" and r["spare_s"] < WATCH_SPARE_S:
                continue
            # the zone this spawn area would otherwise feed: the shared zone its recommended corridors reach first
            feed = [(c["time"], c["zone"]) for c in corrs if c["area"] == ar["id"] and c["variant"] == 0
                    and pl["zones"][c["zone"]]["share"] > 0]
            if not feed:
                continue
            L = min(feed)[1]
            if pl["zones"][L]["share"] < 1 / TEAM - 1e-9:
                continue
            pl["zones"][L]["share"] -= 1 / TEAM
            watch.append(dict(area=f"{a}{r['label']}", **{"from": ar["id"]}, count=1))
            if cl == "F":
                reasons.append(f"flank guard: {ar['id']} posts one tank on {a}{r['label']} (its best flank-route area, "
                               f"{ncls} flank corridors, spare {r['spare_s']:+.0f} s, no spare threshold; taken from {L})")
            else:
                reasons.append(f"{ar['id']} sends one tank to {a}{r['label']} (main routes, spare {r['spare_s']:.0f} s >= "
                               f"{WATCH_SPARE_S:.0f} s, {ncls} main corridors; taken from {L})")
    pl["watch"] = watch
    pl["why"] += (" Watch (at most %d per army, flank guards first): " % WATCH_MAX + "; ".join(reasons) + ".") if reasons else \
        " No watch tank."
    if open_:
        pl["why"] += f" Flank left open: {', '.join(open_)} has no flank-route area it reaches first within 60 s."


def build_pack(mission, base, areas, poses, zones, corrs, pack_areas, tindex, zt, capacity=None, designation=None):
    pi = {p["id"]: i for i, p in enumerate(poses)}
    pack = dict(format="WTPLAN001", mission=mission)
    pack["spawn_areas"] = [dict(id=a["id"], army=a["army"], x=a["centre"][0], z=a["centre"][1],
                                poses=[pi[p["id"]] for p in poses if p["area"] == a["id"]]) for a in areas]
    front = {}
    for a in (1, 2):
        for L, *_r in zones:
            ns = [sum(1 for c in corrs if c["area"] == ar["id"] and c["zone"] == L and c.get("front"))
                  for ar in areas if ar["army"] == a]
            front[(a, L)] = min(ns) if ns else 0
    pack["zones"] = [dict(id=L, target=tindex[f"zone_{L}"], x=x, z=z, r=r, capacity=(capacity or {}).get(L, 0),
                          front_corridors={"1": front[(1, L)], "2": front[(2, L)]}) for L, x, z, r in zones]
    pack["support"] = {str(a): {L: [x["id"] for x in pack_areas if x["army"] == a and x["cls"] == "S" and x["zones"] == [L]]
                                for L, *_r in zones} for a in (1, 2)}
    pack["corridors"] = [dict(id=c["id"], army=c["army"], area=c["area"], zone=c["zone"], role=c["pack_role"],
                              variant=c["variant"], time_s=c["time"], seen_s=c["expo_s"], first_seen_s=c["first_seen_t"],
                              enters_from_deg=round(c["entry_deg"], 1)) for c in corrs]
    pack["areas"] = [dict(id=a["id"], army=a["army"], cls=a["cls"], first_area=a["first_area"], own_t=a["own_t"],
                          by_t=a["by_t"], zones=a["zones"], watch_x=a["watch_x"], watch_z=a["watch_z"],
                          hull_down_share=a["hull_down_share"], tanks=a["tanks"],
                          slots=[dict(target=tindex[s["target_id"]], x=s["x"], z=s["z"], face_deg=s["face_deg"])
                                 for s in a["slots"]]) for a in pack_areas]
    pack["plans"] = build_plans(base, areas, zones, corrs, {a["id"]: a for a in pack_areas})
    return pack
