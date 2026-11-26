"""Lane pictures: lanes per army (end kind, open dashes, contact line, bounds beside the lane, sectors, table),
positions per army (the enemy traffic each position watches), cover points per spawn area, circle + support lanes per
zone, plan tables."""
import math
import textwrap

import numpy as np
from PIL import Image, ImageDraw

from render30 import (ARMY_COL, TITLE_H, View, base_layer, canvas, draw_spawns, draw_zones, font, legend, polyline,
                      scale_bar, text_block)

KIND_COL = {"circle": (30, 90, 220), "support": (20, 150, 60), "guard": (150, 60, 190)}
DASH_COL = (235, 40, 20)
OPP_COL = (255, 220, 0)
CONTACT_COL = (255, 215, 0)
POS_COLS = [(150, 60, 190), (230, 120, 0), (0, 150, 150), (200, 30, 120), (90, 120, 0), (40, 40, 200)]


def _arc(P):
    return np.concatenate([[0], np.cumsum(np.hypot(*np.diff(np.asarray(P, np.float64), axis=0).T))])


def _at(P, s, v):
    return float(np.interp(v, s, P[:, 0])), float(np.interp(v, s, P[:, 1]))


def _diamond(d, cx, cy, r=8):
    d.polygon([(cx, cy - r), (cx + r, cy), (cx, cy + r), (cx - r, cy)], fill=CONTACT_COL, outline=(0, 0, 0), width=2)


def _lane_draw(d, V, tr, col, w=4, bounds=True):
    P = np.asarray(tr["P"], np.float64)
    polyline(d, V, P[:, 0], P[:, 1], (0, 0, 0), w + 2)
    polyline(d, V, P[:, 0], P[:, 1], col, w)
    o = np.asarray(tr["open_flags"], bool)
    n = min(len(o), len(P) - 1)
    k = 0
    while k < n:
        if o[k]:
            j = k
            while j < n and o[j]:
                j += 1
            seg = P[k:j + 1]
            polyline(d, V, seg[:, 0], seg[:, 1], DASH_COL, w)
            k = j
        else:
            k += 1
    s = _arc(P)
    if bounds:
        for b in tr["bounds"]:
            cx, cy = V.p(b["x"], b["z"])
            qx, qy = V.p(*_at(P, s, b["at_m"]))
            d.line([(cx, cy), (qx, qy)], fill=(120, 120, 120), width=1)
            d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
    if tr.get("contact_m") is not None:
        _diamond(d, *V.p(*_at(P, s, tr["contact_m"])))


def _slots(d, V, l, col):
    for s in l["slots"]:
        cx, cy = V.p(s["x"], s["z"])
        ex, ey = V.p(s["x"] + 18 * math.cos(math.radians(s["face_deg"])), s["z"] + 18 * math.sin(math.radians(s["face_deg"])))
        d.line([(cx, cy), (ex, ey)], fill=(0, 0, 0), width=3)
        d.rectangle([cx - 5, cy - 5, cx + 5, cy + 5], fill=col, outline=(0, 0, 0), width=2)


def _sectors(d, V, ax):
    u, o, w = ax["axis"], ax["origin"], ax["width_m"]
    f = (-u[1], u[0])
    for k in range(ax["sectors"] + 1):
        bx, bz = o[0] + u[0] * w * k / ax["sectors"], o[1] + u[1] * w * k / ax["sectors"]
        a = V.p(bx - f[0] * 3000, bz - f[1] * 3000)
        b = V.p(bx + f[0] * 3000, bz + f[1] * 3000)
        d.line([a, b], fill=(90, 90, 90), width=2)
    for k in range(ax["sectors"]):
        cx, cz = o[0] + u[0] * w * (k + 0.5) / ax["sectors"], o[1] + u[1] * w * (k + 0.5) / ax["sectors"]
        px, py = V.p(cx, cz)
        if 0 <= px < V.w - 30 and TITLE_H < py < TITLE_H + V.h - 30:
            d.text((px - 8, py), str(k), fill=(60, 60, 60), font=font(34, True), stroke_width=3, stroke_fill=(255, 255, 255))


def render(ctx, od, mission, lanes, covers, views, pack, trunks, positions):
    Fl, S, zones, areas, poses = ctx["Fl"], ctx["S"], ctx["zones"], ctx["areas"], ctx["poses"]
    bb = ctx["bbox"]
    V = View(max(bb[0], Fl.X0), max(bb[1], Fl.Z0), min(bb[2], Fl.X0 + Fl.W5 * 0.5), min(bb[3], Fl.Z0 + Fl.H5 * 0.5), 1.0)
    lay = base_layer(Fl, S, V, "faint")
    files = []
    pl = {x["id"]: x for x in pack["lanes"]}
    # 1 lanes per army
    for a in (1, 2):
        im, d = canvas(lay.copy(), V, f"Lanes of army {a} - {mission}",
                       "colour = end kind; red = open (an enemy can watch it when we pass); yellow diamond = contact line; "
                       "white dots = cover beside the lane (grey line to its arc point); squares = end slots")
        _sectors(d, V, ctx["axes"][a])
        for l in lanes[a]:
            tr = trunks.get(l["id"])
            if tr is None:
                continue
            _lane_draw(d, V, tr, KIND_COL[l["end"]])
        for l in lanes[a]:
            for seg in l.get("opp_segs", []):
                for x, z in seg:
                    cx, cy = V.p(x, z)
                    d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=OPP_COL, outline=(0, 0, 0), width=1)
        for l in lanes[a]:
            if trunks.get(l["id"]) is None:
                continue
            _slots(d, V, l, KIND_COL[l["end"]])
            s = l["slots"][0]
            cx, cy = V.p(s["x"], s["z"])
            lab = l["id"][1:] + (f" v{l['value']:.0f}" if l.get("side") == "toll" else "")
            d.text((cx + 6, cy - 22), lab, fill=KIND_COL[l["end"]], font=font(16, True), stroke_width=2,
                   stroke_fill=(255, 255, 255))
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        y = legend(d, V, [("line", KIND_COL["circle"], "circle lane (into a capture circle)"),
                          ("line", KIND_COL["support"], "support lane (fire onto a zone)"),
                          ("line", KIND_COL["guard"], "position lane (watches enemy traffic)"),
                          ("line", DASH_COL, "open (enemy can watch when we pass)"),
                          ("dot", CONTACT_COL, "contact line (first open stretch of 20 m or more)"),
                          ("dot", (255, 255, 255), "cover beside the lane (used only when engaged)"),
                          ("dot", OPP_COL, "opposed stretch (head-on with another lane)")],
                   title="Legend")
        lines = ["#id   from sv end sec cap time open m/s dash bnd contact value opp"]
        for l in lanes[a]:
            x = pl.get(l["id"])
            if x is None:
                lines.append(f"{l['id']:5s} {l['from']:4s} {str(l['serves'] or '-'):2s} {l['end'][:3]} closed / rejected")
                continue
            cm = "-" if x["contact_m"] is None else f"{x['contact_m']:.0f}"
            lines.append(f"{l['id']:5s} {l['from']:4s} {str(l['serves'] or '-'):2s} {l['end'][:3]} {x['sector']:3d} {x['capacity']:3d} "
                         f"{x['time_s']:4.0f} {x['open_m']:4.0f}/{x['open_s']:3.0f} {x['longest_dash_m']:4.0f} {len(x['bounds']):3d} "
                         f"{cm:>7s} {x['value']:5.0f} " + ",".join(o[2:] for o in x.get("opposes", [])))
        text_block(d, V, lines, y + 10, size=13)
        p = od / f"lanes_army{a}.png"
        im.save(p)
        files.append(p.name)
    # 2 positions per army: the enemy traffic each position watches
    for a in (1, 2):
        b = 3 - a
        im, d = canvas(lay.copy(), V, f"Positions of army {a} (watch the enemy's traffic) - {mission}",
                       f"thin = army {b}'s circle / support lanes; dots = traffic points a position sees (its colour); "
                       "squares = slots (black = hull-down), ticks = facing")
        for l in lanes[b]:
            tr = trunks.get(l["id"])
            if tr is None or l["end"] not in ("circle", "support"):
                continue
            P = np.asarray(tr["P"], np.float64)
            polyline(d, V, P[:, 0], P[:, 1], ARMY_COL[b], 2)
        for k, p in enumerate(positions[a]):
            col = POS_COLS[k % len(POS_COLS)]
            s0 = p["slots"][0]
            c0 = V.p(s0["x"], s0["z"])
            for x, z in p["seen_xy"]:
                q = V.p(x, z)
                d.line([c0, q], fill=col, width=1)
            for x, z in p["seen_xy"]:
                cx, cy = V.p(x, z)
                d.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=col)
        for k, p in enumerate(positions[a]):
            col = POS_COLS[k % len(POS_COLS)]
            for s in p["slots"]:
                cx, cy = V.p(s["x"], s["z"])
                ex, ey = V.p(s["x"] + 24 * math.cos(math.radians(s["face_deg"])), s["z"] + 24 * math.sin(math.radians(s["face_deg"])))
                d.line([(cx, cy), (ex, ey)], fill=(0, 0, 0), width=3)
                d.rectangle([cx - 7, cy - 7, cx + 7, cy + 7], fill=col, outline=(0, 0, 0) if s["hull_down"] else (255, 255, 255),
                            width=3)
            s0 = p["slots"][0]
            cx, cy = V.p(s0["x"], s0["z"])
            d.text((cx + 10, cy - 28), f"P{p['index']} {p['value']:.0f}s", fill=col, font=font(20, True), stroke_width=3,
                   stroke_fill=(255, 255, 255))
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        y = legend(d, V, [("line", ARMY_COL[b], f"army {b} circle / support lane"),
                          ("dot", POS_COLS[0], "traffic point seen (every 10 m)"),
                          ("box", (0, 0, 0), "slot, hull-down towards what it sees"),
                          ("box", (255, 255, 255), "slot, not hull-down")], title="Legend")
        lines = ["#pos lanes (from)        value lanes range m   hd  route m time s"]
        for p in positions[a]:
            ls = ",".join(f"{x['id'][2:]}({x['from']})" for x in p["lanes"]) or "none"
            hd = "".join("y" if s["hull_down"] else "n" for s in p["slots"])
            tm = ",".join(f"{x['time_s']:.0f}" for x in p["lanes"]) or "-"
            nr = "-" if p.get("nearest_route_m") is None else f"{p['nearest_route_m']:.0f}"
            lines.append(f"P{p['index']}  {ls:20s} {p['value']:5.0f} {p['enemy_lanes_seen']:5d} "
                         f"{p['range_m'][0]:3d}-{p['range_m'][1]:3d} {hd:3s} {nr:>6s} {tm}")
        lines.append("route m = slot 1 to the nearest lane route of either army")
        if not positions[a]:
            lines.append("no position reached the value threshold")
        text_block(d, V, lines, y + 10, size=14)
        p_ = od / f"positions_army{a}.png"
        im.save(p_)
        files.append(p_.name)
    # 3 cover points per spawn area
    for a, A in [(a, g) for a in (1, 2) for g in covers[a]]:
        A.obs, A.obs_xy = views[a].obs, views[a].obs_xy
        lay2 = lay.copy()
        arr = np.asarray(lay2).copy()
        o = A.open5[::-1]
        Z1 = Fl.Z0 + Fl.H5 * 0.5
        r0, c0 = int((Z1 - V.z1) / 0.5), int((V.x0 - Fl.X0) / 0.5)
        sub = o[r0:r0 + V.h * 2:2, c0:c0 + V.w * 2:2][:V.h, :V.w] > 0
        arr[:sub.shape[0], :sub.shape[1]][sub] = (arr[:sub.shape[0], :sub.shape[1]][sub] * 0.6 + np.array([250, 170, 150]) * 0.4).astype(np.uint8)
        im, d = canvas(Image.fromarray(arr), V, f"Cover points of spawn area {A.area} (army {a}) - {mission}",
                       f"pink = open for lanes from {A.area} (an enemy can watch it when we pass: E <= t + 10 s); dots = cover "
                       f"points (one per 20 m), dark = turn room; ticks = facing; squares = enemy positions")
        for n in A.nodes:
            cx, cy = V.p(n["x"], n["z"])
            ex, ey = V.p(n["x"] + 9 * math.cos(math.radians(n["face_deg"])), n["z"] + 9 * math.sin(math.radians(n["face_deg"])))
            d.line([(cx, cy), (ex, ey)], fill=(0, 0, 0), width=1)
            col = (20, 60, 20) if n["turn"] else (90, 200, 90)
            d.ellipse([cx - 2.5, cy - 2.5, cx + 2.5, cy + 2.5], fill=col)
        ox, oz = A.obs_xy
        for x, z in zip(ox, oz):
            cx, cy = V.p(x, z)
            d.rectangle([cx - 3, cy - 3, cx + 3, cy + 3], outline=ARMY_COL[3 - a], width=2)
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        y = legend(d, V, [("dot", (20, 60, 20), "cover point with turn room (4.3 m)"), ("dot", (90, 200, 90), "cover point"),
                          ("box", (250, 200, 190), "open"), ("ring", ARMY_COL[3 - a], "enemy likely position (observer)")],
                   title="Legend")
        text_block(d, V, [f"{len(A.nodes)} cover points ({sum(n['turn'] for n in A.nodes)} with turn room)",
                          f"open: {A.open_frac:.1%} of the drivable ground",
                          "enemy positions: " + ", ".join(f"{k} {len(v)}" for k, v in A.obs.items())], y + 10, size=16)
        p = od / f"cover_{A.area}.png"
        im.save(p)
        files.append(p.name)
    # 4 per zone: circle and support lanes of both armies
    for L, zx, zz, zr in zones:
        h = 450
        Vz = View(max(V.x0, zx - h), max(V.z0, zz - h), min(V.x1, zx + h), min(V.z1, zz + h), 1.5)
        layz = base_layer(Fl, S, Vz, "faint")
        im, d = canvas(layz, Vz, f"Zone {L}: circle and support lanes - {mission}",
                       "army 1 blue edge, army 2 red edge; inner colour = end kind; red = open; diamond = contact line")
        for a in (1, 2):
            for l in lanes[a]:
                tr = trunks.get(l["id"])
                if tr is None or l["serves"] != L or l["end"] not in ("circle", "support"):
                    continue
                P = tr["P"]
                polyline(d, Vz, P[:, 0], P[:, 1], ARMY_COL[a], 7)
                _lane_draw(d, Vz, tr, KIND_COL[l["end"]], 3, bounds=False)
                _slots(d, Vz, l, KIND_COL[l["end"]])
                s = l["slots"][0]
                cx, cy = Vz.p(s["x"], s["z"])
                d.text((cx + 6, cy - 20), l["id"], fill=ARMY_COL[a], font=font(16, True), stroke_width=2, stroke_fill=(255, 255, 255))
        draw_zones(d, Vz, zones)
        scale_bar(d, Vz)
        y = legend(d, Vz, [("line", KIND_COL["circle"], "circle lane"), ("line", KIND_COL["support"], "support lane"),
                           ("line", DASH_COL, "open"), ("dot", CONTACT_COL, "contact line"),
                           ("line", ARMY_COL[1], "army 1"), ("line", ARMY_COL[2], "army 2")],
                   title="Legend")
        lines = ["#id    from end      time open m dash contact slots"]
        for a in (1, 2):
            for l in lanes[a]:
                x = pl.get(l["id"])
                if x is None or l["serves"] != L or l["end"] not in ("circle", "support"):
                    continue
                cm = "-" if x["contact_m"] is None else f"{x['contact_m']:.0f}"
                lines.append(f"{l['id']:5s} {l['from']:4s} {l['end']:8s} {x['time_s']:4.0f} {x['open_m']:6.0f} {x['longest_dash_m']:4.0f} "
                             f"{cm:>7s} {len(x['slots'])}")
        text_block(d, Vz, lines, y + 10, size=15)
        p = od / f"zone_{L}_lanes.png"
        im.save(p)
        files.append(p.name)
    # 5 plans
    for a in (1, 2):
        plans = pack["plans"][str(a)]
        W, H = 1700, 200 + 300 * len(plans)
        im = Image.new("RGB", (W, H), (255, 255, 255))
        d = ImageDraw.Draw(im)
        d.text((14, 8), f"Plans of army {a} - {mission} (tanks of 16; roles and shares only)", fill=(0, 0, 0), font=font(30, True))
        zl = [L for L, *_r in zones]
        y = 70
        d.text((14, y), f"{'plan':16s} {'weight':6s} " + " ".join(f"{L + ' (role, tanks)':22s}" for L in zl), fill=(0, 0, 0),
               font=font(20, True))
        y += 36
        for p_ in plans:
            row = f"{p_['kind']:16s} {p_['weight']:<6d} " + " ".join(
                f"{p_['zones'][L]['role'] + ' ' + format(p_['zones'][L]['share'] * 16, '.1f'):22s}" for L in zl)
            d.text((14, y), row, fill=(0, 0, 0), font=font(20))
            y += 34
            for ln in textwrap.wrap("why: " + p_["why"], 140):
                d.text((40, y), ln, fill=(70, 70, 70), font=font(18))
                y += 26
            y += 20
        im = im.crop((0, 0, W, y + 20))
        p = od / f"plans_army{a}.png"
        im.save(p)
        files.append(p.name)
    return files
