"""Pictures 1-7 of the run30 analysis (north = +z = up; legends in a panel right of the map; title on top)."""
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import common30 as C

ARMY_COL = {1: (30, 90, 220), 2: (210, 40, 40)}
ZONE_COL = {"A": (230, 120, 0), "B": (150, 0, 200), "C": (0, 150, 120),
            "D": (200, 40, 90), "E": (0, 110, 200), "F": (120, 120, 0), "G": (90, 60, 30), "H": (0, 160, 200)}
PANEL_W, TITLE_H = 640, 90


def font(sz, bold=False):
    for n in (("arialbd.ttf" if bold else "arial.ttf"), "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(n, sz)
        except OSError:
            pass
    return ImageFont.load_default()


class View:
    def __init__(self, x0, z0, x1, z1, ppm):
        self.x0, self.z0, self.x1, self.z1, self.ppm = x0, z0, x1, z1, ppm
        self.w, self.h = int((x1 - x0) * ppm), int((z1 - z0) * ppm)

    def p(self, x, z):
        return ((x - self.x0) * self.ppm, TITLE_H + (self.z1 - z) * self.ppm)


def base_layer(Fl, S, V, mode="full"):
    """RGB of the view: hillshade tinted by height, water, hard objects, foliage, road mesh."""
    G = S.G
    gy, gx = np.gradient(G, 2.0)
    shade = np.clip(0.75 + 0.9 * (-gx * 0.6 + gy * 0.6) / np.sqrt(1 + gx ** 2 + gy ** 2), 0.35, 1.15)
    lo, hi = np.percentile(G, [2, 98])
    t = np.clip((G - lo) / max(hi - lo, 1e-3), 0, 1)
    base = np.stack([200 + 40 * t, 205 + 25 * t, 175 + 30 * t], -1) * shade[..., None]
    if mode == "faint":
        base = 0.55 * base + 0.45 * 235
    rgb = np.clip(base, 0, 255).astype(np.uint8)
    up = lambda m, k: np.repeat(np.repeat(m, k, 0), k, 1)
    rgb = up(rgb, 4)   # 0.5 m
    if Fl.has_road:
        rgb[Fl.road05] = (rgb[Fl.road05] * 0.7 + np.array([250, 250, 250]) * 0.3).astype(np.uint8)
    fol05 = up(S.fol, 4)
    rgb[fol05] = (rgb[fol05] * 0.45 + np.array([40, 140, 40]) * 0.55).astype(np.uint8)
    rgb[Fl.water05] = (70, 120, 210) if mode != "faint" else (150, 180, 230)
    rgb[Fl.obj05] = (70, 70, 70) if mode != "faint" else (130, 130, 130)
    rgb[Fl.steep05 & ~Fl.water05] = (140, 100, 60) if mode != "faint" else (180, 150, 120)
    img = Image.fromarray(rgb[::-1])
    # crop to the view (image row 0 = north edge)
    Z1 = Fl.Z0 + Fl.H5 * 0.5
    box = (int((V.x0 - Fl.X0) / 0.5), int((Z1 - V.z1) / 0.5), int((V.x1 - Fl.X0) / 0.5), int((Z1 - V.z0) / 0.5))
    return img.crop(box).resize((V.w, V.h), Image.BILINEAR)


def canvas(layer, V, title, subtitle=""):
    im = Image.new("RGB", (V.w + PANEL_W, V.h + TITLE_H), (255, 255, 255))
    im.paste(layer, (0, TITLE_H))
    d = ImageDraw.Draw(im)
    V.title, V.sub = title, subtitle
    d.text((14, 8), title, fill=(0, 0, 0), font=font(30, True))
    if subtitle:
        d.text((14, 48), subtitle, fill=(60, 60, 60), font=font(20))
    return im, d


def scale_bar(d, V):
    # wipe whatever map drawing spilled into the title band and the legend panel, then rewrite the title
    d.rectangle([0, 0, V.w + PANEL_W, TITLE_H - 1], fill=(255, 255, 255))
    d.rectangle([V.w, 0, V.w + PANEL_W, TITLE_H + V.h], fill=(255, 255, 255))
    d.text((14, 8), V.title, fill=(0, 0, 0), font=font(30, True))
    if V.sub:
        d.text((14, 48), V.sub, fill=(60, 60, 60), font=font(20))
    L = 200 if V.ppm >= 1.5 else 500
    x, y = 20, TITLE_H + V.h - 40
    d.rectangle([x - 6, y - 26, x + L * V.ppm + 60, y + 16], fill=(255, 255, 255))
    d.line([x, y, x + L * V.ppm, y], fill=(0, 0, 0), width=5)
    for k in range(0, 6):
        xx = x + k * L * V.ppm / 5
        d.line([xx, y - 8, xx, y + 8], fill=(0, 0, 0), width=2)
    d.text((x + L * V.ppm + 8, y - 12), f"{L} m", fill=(0, 0, 0), font=font(20, True))
    d.text((x, y - 26), "N up", fill=(0, 0, 0), font=font(16))


def draw_zones(d, V, zones, label=True):
    for L, x, z, r in zones:
        cx, cy = V.p(x, z)
        rr = r * V.ppm
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=(0, 0, 0), width=4)
        d.ellipse([cx - rr + 3, cy - rr + 3, cx + rr - 3, cy + rr - 3], outline=ZONE_COL[L], width=3)
        if label:
            d.text((cx + rr + 4, cy - rr - 4), L, fill=ZONE_COL[L], font=font(34, True), stroke_width=3, stroke_fill=(255, 255, 255))


def draw_spawns(d, V, areas, poses):
    for p in poses:
        cx, cy = V.p(p["x"], p["z"])
        d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=ARMY_COL[p["army"]], outline=(255, 255, 255))
    for a in areas:
        cx, cy = V.p(*a["centre"])
        d.text((cx + 14, cy - 34), a["id"], fill=ARMY_COL[a["army"]], font=font(28, True), stroke_width=3, stroke_fill=(255, 255, 255))


def draw_kill(d, V, kill):
    for a, lst in kill.items():
        for s in lst:
            M = np.linalg.inv(s["inv"])
            pts = []
            for k in range(73):
                ang = 2 * math.pi * k / 72
                u, v = s["r"] * math.cos(ang), s["r"] * math.sin(ang)
                pts.append(V.p(s["c"][0] + M[0, 0] * u + M[0, 1] * v, s["c"][1] + M[1, 0] * u + M[1, 1] * v))
            inside = lambda q: 0 <= q[0] < V.w and TITLE_H <= q[1] < TITLE_H + V.h
            for q0, q1 in zip(pts, pts[1:]):
                if inside(q0) and inside(q1):
                    d.line([q0, q1], fill=ARMY_COL[a], width=3)


def polyline(d, V, xs, zs, col, w):
    pts = [V.p(x, z) for x, z in zip(xs, zs)]
    if len(pts) > 1:
        d.line(pts, fill=col, width=w, joint="curve")


def dashed(d, V, xs, zs, col, w, on=6, off=4):
    pts = [V.p(x, z) for x, z in zip(xs, zs)]
    for i in range(0, len(pts) - 1, on + off):
        seg = pts[i:i + on + 1]
        if len(seg) > 1:
            d.line(seg, fill=col, width=w)


def paint_cells(im, S, V, cells, col, alpha=0.75):
    """Fill 2 m cells (sight grid indices) on the canvas."""
    x, z = S.xy(np.asarray(cells))
    d = ImageDraw.Draw(im)
    h = max(1, int(round(V.ppm)))
    for xx, zz in zip(x, z):
        px, py = V.p(xx - 1, zz + 1)
        if 0 <= px < V.w and TITLE_H <= py < TITLE_H + V.h:
            d.rectangle([px, py, px + 2 * h - 1, py + 2 * h - 1], fill=col)


def legend(d, V, items, y=TITLE_H + 10, title=None):
    x = V.w + 20
    if title:
        d.text((x, y), title, fill=(0, 0, 0), font=font(24, True))
        y += 34
    for kind, col, text in items:
        if kind == "box":
            d.rectangle([x, y + 3, x + 26, y + 23], fill=col, outline=(0, 0, 0))
        elif kind == "line":
            d.line([x, y + 13, x + 26, y + 13], fill=col, width=5)
        elif kind == "dot":
            d.ellipse([x + 6, y + 6, x + 20, y + 20], fill=col, outline=(0, 0, 0))
        elif kind == "ring":
            d.ellipse([x + 2, y + 2, x + 24, y + 24], outline=col, width=4)
        elif kind == "text":
            pass
        d.text((x + (0 if kind == "text" else 36), y), text, fill=(0, 0, 0), font=font(20))
        y += 30
    return y


def text_block(d, V, lines, y, size=18, x=None):
    x = V.w + 20 if x is None else x
    for ln in lines:
        bold = ln.startswith("#")
        d.text((x, y), ln.lstrip("#"), fill=(0, 0, 0), font=font(size + (2 if bold else 0), bold))
        y += size + 8
    return y


def cmap(v, lo, hi, kind="seq"):
    t = float(np.clip((v - lo) / (hi - lo), 0, 1))
    if kind == "div":   # blue (army 1 first) - white - red (army 2 first)
        if t < 0.5:
            s = t / 0.5
            return (int(30 + 225 * s), int(90 + 165 * s), int(220 + 35 * s))
        s = (t - 0.5) / 0.5
        return (255, int(255 - 215 * s), int(255 - 215 * s))
    stops = [(68, 1, 84), (59, 82, 139), (33, 145, 140), (94, 201, 98), (253, 231, 37)]
    k = min(3, int(t * 4))
    s = t * 4 - k
    a, b = stops[k], stops[k + 1]
    return tuple(int(a[i] + (b[i] - a[i]) * s) for i in range(3))


def colorbar(d, V, y, lo, hi, kind, label, ticks):
    x = V.w + 20
    d.text((x, y), label, fill=(0, 0, 0), font=font(20, True))
    y += 30
    for i in range(400):
        d.line([x + i, y, x + i, y + 26], fill=cmap(lo + (hi - lo) * i / 399, lo, hi, kind))
    d.rectangle([x, y, x + 400, y + 26], outline=(0, 0, 0))
    for tk in ticks:
        xx = x + 400 * (tk - lo) / (hi - lo)
        d.line([xx, y + 26, xx, y + 34], fill=(0, 0, 0), width=2)
        d.text((xx - 14, y + 36), f"{tk:g}", fill=(0, 0, 0), font=font(18))
    return y + 66


# ---------------------------------------------------------------- pictures
def render_all(Fl, base, zones, areas, poses, kill, cache, od, mission, bad_pts):
    S = base["S"]
    res = base["res"]
    corrs = cache["corrs"]
    arr = cache["arr"]
    bb = C.battle_area(mission)["bbox"]
    VF = View(max(bb[0], Fl.X0), max(bb[1], Fl.Z0), min(bb[2], Fl.X0 + Fl.W5 * 0.5), min(bb[3], Fl.Z0 + Fl.H5 * 0.5), 1.0)
    # town crop: around the zones (+250 m)
    zx = [z[1] for z in zones]
    zz = [z[2] for z in zones]
    VC = View(max(VF.x0, min(zx) - 300), max(VF.z0, min(zz) - 300), min(VF.x1, max(zx) + 300), min(VF.z1, max(zz) + 300), 2.0)
    files = []
    layers = {}

    def layer(V, mode):
        k = (V.x0, V.z0, V.ppm, mode)
        if k not in layers:
            layers[k] = base_layer(Fl, S, V, mode)
        return layers[k].copy()

    # 1 base map
    for V, tag in ((VF, ""), (VC, "_town")):
        im, d = canvas(layer(V, "full"), V, f"1. Base map{' (town crop)' if tag else ''}: terrain, water, hard objects, woods, zones, spawn areas",
                       f"{mission} - level {C.mission_level(mission)}; heights {S.G.min():.0f}-{S.G.max():.0f} m (light = high)")
        draw_kill(d, V, kill)
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        if len(bad_pts):
            for x, z in bad_pts[::5]:
                cx, cy = V.p(x, z)
                if 0 <= cx < V.w and TITLE_H <= cy < TITLE_H + V.h:
                    d.point((cx, cy), fill=(255, 0, 255))
        scale_bar(d, V)
        y = legend(d, V, [("box", (70, 120, 210), "water deeper than 1 m (closed)"), ("box", (70, 70, 70), "hard object (house, wall)"),
                          ("box", (140, 100, 60), "slope over 45 % (closed)"), ("box", (60, 145, 60), "bush / tree crowns"),
                          ("box", (225, 225, 215), "road mesh (Lnav), if any"), ("ring", (0, 0, 0), "capture zone A / B / C"),
                          ("dot", ARMY_COL[1], "army 1 spawn points"), ("dot", ARMY_COL[2], "army 2 spawn points"),
                          ("line", ARMY_COL[1], "army 1 spawn protection (army 2 barred)"),
                          ("line", ARMY_COL[2], "army 2 spawn protection (army 1 barred)"),
                          ("dot", (255, 0, 255), "logged tank on a hard cell (map error)")], title="Legend")
        lines = ["#Bridge decks (s30 deck-lane rule)"]
        for n_, b in enumerate(base.get("bridges") or []):
            lines.append(f"{n_ + 1}: ({b['x']:.0f}, {b['z']:.0f}) {b['deck_length']:.0f} x {b['lane_width']:.1f} m "
                         f"{'connected' if b['connected'] else 'NOT connected'}{', joined over a gap' if b.get('joined') else ''}")
        text_block(d, V, lines, y + 20)
        p = od / f"1_base_map{tag}.png"
        im.save(p)
        files.append(p.name)
    # 2 who is first
    t1 = S.block_min(cache["army"][1], Fl.valid05 & Fl.reach)
    t2 = S.block_min(cache["army"][2], Fl.valid05 & Fl.reach)
    for V, tag in ((VF, ""), (VC, "_town")):
        lay = np.array(layer(V, "faint")).astype(np.float32)
        diff = t1 - t2          # < 0: army 1 first (blue), > 0: army 2 first (red)
        ok = np.isfinite(diff)
        lo, hi = -60, 60
        tt = np.clip((np.where(ok, diff, 0) - lo) / (hi - lo), 0, 1)
        r = np.where(tt < 0.5, 30 + 225 * tt / 0.5, 255)
        g = np.where(tt < 0.5, 90 + 165 * tt / 0.5, 255 - 215 * (tt - 0.5) / 0.5)
        b = np.where(tt < 0.5, 220 + 35 * tt / 0.5, 255 - 215 * (tt - 0.5) / 0.5)
        col = np.stack([r, g, b], -1)
        a2 = Image.fromarray(np.where(ok[..., None], col[::-1], 60).astype(np.uint8))
        Z1 = Fl.Z0 + Fl.H5 * 0.5
        box = (int((V.x0 - Fl.X0) / 2), int((Z1 - V.z1) / 2), int((V.x1 - Fl.X0) / 2), int((Z1 - V.z0) / 2))
        a2 = np.array(a2.crop(box).resize((V.w, V.h), Image.NEAREST)).astype(np.float32)
        mix = (0.45 * lay + 0.55 * a2).astype(np.uint8)
        im, d = canvas(Image.fromarray(mix), V, f"2. Who is first: army 1 time minus army 2 time (s){' - town crop' if tag else ''}",
                       f"travel time from the nearest own spawn area; blue = army 1 is there first, red = army 2 first, white = even; dark grey = unreachable for an army")
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        for L, x, z, rz in zones:
            cx, cy = V.p(x, z)
            a1, a_2 = arr[L][1], arr[L][2]
            txt = f"{L}: 1st/5th tank\narmy1 {a1['first']:.0f}/{a1['fifth']:.0f} s\narmy2 {a_2['first']:.0f}/{a_2['fifth']:.0f} s"
            d.multiline_text((cx + 30 * V.ppm / 1.0 + 10, cy + 10), txt, fill=(0, 0, 0), font=font(20, True),
                             stroke_width=3, stroke_fill=(255, 255, 255))
        scale_bar(d, V)
        y = colorbar(d, V, TITLE_H + 10, -60, 60, "div", "army 1 time - army 2 time (s)", [-60, -30, 0, 30, 60])
        y = text_block(d, V, ["blue: army 1 first, red: army 2 first"], y)
        lines = ["#Arrival, 1st / 5th tank (s): fastest | recommended"]
        ar2 = base["arr_rec"]
        for L, *_r in zones:
            a1, a_2 = arr[L][1], arr[L][2]
            r1, r2 = ar2[L][1], ar2[L][2]
            lines.append(f"{L}: army1 {a1['first']:.0f}/{a1['fifth']:.0f} | {r1['first']:.0f}/{r1['fifth']:.0f}   "
                         f"army2 {a_2['first']:.0f}/{a_2['fifth']:.0f} | {r2['first']:.0f}/{r2['fifth']:.0f}")
            lines.append(f"    lead of army 1: {a_2['first'] - a1['first']:+.0f} s fastest, {r2['first'] - r1['first']:+.0f} s recommended")
        lines.append("(the map colours use the fastest times)")
        sp = base["speeds"]
        lines += ["", f"speeds: road {sp['v_road']} m/s, off road {sp['v_off']} m/s",
                  "time = generator cost x length / speed", "(narrow street counts up to 2x)"]
        text_block(d, V, lines, y + 10)
        p = od / f"2_who_is_first{tag}.png"
        im.save(p)
        files.append(p.name)
    # 3 corridors per army
    for a in (1, 2):
        for V, tag in ((VF, ""), (VC, "_town")):
            im, d = canvas(layer(V, "faint"), V, f"3. Corridors of army {a} to zones A/B/C{' - town crop' if tag else ''}",
                           "up to 5 distinct corridors per (spawn area, zone), streets of earlier ones closed; recommended / fastest / other drawn apart; gates = narrowest places (free width)")
            mine = [c for c in corrs if c["army"] == a]
            k_ = V.ppm if V.ppm > 1 else 1
            prio = {"other": 0, "fastest": 1, "recommended": 2, "both": 2}
            for c in sorted(mine, key=lambda c: prio[c["role"]]):
                col = ZONE_COL[c["zone"]]
                if c["role"] == "other":
                    polyline(d, V, c["path"]["x"], c["path"]["z"], col, max(2, int(2 * k_)))
                elif c["role"] == "fastest":
                    dashed(d, V, c["path"]["x"], c["path"]["z"], col, int(5 * k_))
                else:
                    polyline(d, V, c["path"]["x"], c["path"]["z"], (0, 0, 0), int(9 * k_))
                    polyline(d, V, c["path"]["x"], c["path"]["z"], col, int(6 * k_))
            for c in mine:
                for g in c["gates"] + [c["narrowest"]]:
                    cx, cy = V.p(g["x"], g["z"])
                    d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline=(0, 0, 0), width=3)
                    d.text((cx + 10, cy - 10), f"{g['width']:.1f}", fill=(0, 0, 0), font=font(16, True),
                           stroke_width=2, stroke_fill=(255, 255, 255))
            draw_zones(d, V, zones)
            draw_spawns(d, V, areas, poses)
            scale_bar(d, V)
            y = legend(d, V, [("line", ZONE_COL[L], f"corridor to zone {L}") for L, *_r in zones] +
                       [("text", None, "thick, black edge = recommended (least seen within 1.3x"),
                        ("text", None, "   the fastest time); dashed = fastest; thin = other"),
                        ("ring", (0, 0, 0), "gate / narrowest place, free width m")], title="Legend")
            lines = ["#Corridor  role  time s  seen s  first seen  narrow m  from"]
            for c in sorted(mine, key=lambda c: (c["area"], c["zone"], c["time"])):
                r_ = {"both": "R+F", "recommended": "R", "fastest": "F", "other": "-"}[c["role"]]
                fs = "-" if c["first_seen_t"] is None else f"{c['first_seen_t']:.0f}"
                lines.append(f"{c['id']:9s} {r_:3s} {c['time']:5.0f} {c['expo_s']:5.0f} {fs:>6s} {c['narrowest']['width']:5.1f}  {c['entry']['side']}")
            text_block(d, V, lines, y + 10, size=16)
            p = od / f"3_corridors_army{a}{tag}.png"
            im.save(p)
            files.append(p.name)
    # 4 fast routes (traffic density)
    for a in (1, 2):
        V = VF
        im, d = canvas(layer(V, "faint"), V, f"4. Fast routes of army {a}: where its corridors run, by time since the start",
                       "colour = time the fastest tank of this army passes (s); width = number of corridors through the place")
        mine = [c for c in corrs if c["army"] == a]
        cnt = {}
        for c in mine:
            sm = c["samples"]
            for x, z in set(zip((sm["x"] // 8).astype(int), (sm["z"] // 8).astype(int))):
                cnt[(x, z)] = cnt.get((x, z), 0) + 1
        tmin = {}
        for c in mine:
            sm = c["samples"]
            for x, z, t in zip(sm["x"], sm["z"], sm["t"]):
                k = (int(x // 8), int(z // 8))
                tmin[k] = min(tmin.get(k, 1e9), float(t))
        for (i, j), n in sorted(cnt.items(), key=lambda kv: kv[1]):
            cx, cy = V.p(i * 8 + 4, j * 8 + 4)
            r = 2 + 1.6 * n
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=cmap(tmin[(i, j)], 0, 150))
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        y = colorbar(d, V, TITLE_H + 10, 0, 150, "seq", "time since start (s)", [0, 30, 60, 90, 120, 150])
        text_block(d, V, ["#width: 1 corridor (small) .. 10+ (large)", f"{len(mine)} corridors of army {a}"], y)
        p = od / f"4_fast_routes_army{a}.png"
        im.save(p)
        files.append(p.name)
    # 5 interdiction: class M (main routes, warm colours) and class F (flank routes, cool colours) as bands
    PAL_M = [(230, 25, 75), (245, 130, 48), (200, 160, 0), (170, 60, 0), (255, 90, 150), (190, 0, 0), (240, 50, 230),
             (255, 160, 100), (150, 30, 90), (220, 110, 0), (255, 60, 60), (180, 90, 60), (230, 0, 120), (200, 120, 40),
             (140, 0, 40)]
    PAL_F = [(0, 130, 200), (60, 180, 75), (0, 170, 170), (70, 70, 220), (0, 100, 60), (90, 200, 250), (40, 120, 120),
             (120, 90, 220), (20, 160, 110), (0, 80, 160), (100, 170, 40), (60, 140, 200), (0, 120, 90), (130, 130, 255),
             (30, 90, 130)]
    for a in (1, 2):
        b = 3 - a
        R = res[a]
        enem = [c for c in corrs if c["army"] == b]
        cls_of = R["corridor_classes"]
        for V, tag in ((VF, ""), (VC, "_town")):
            im, d = canvas(layer(V, "faint"), V, f"5. Interdiction areas of army {a}: M = main routes, F = flank routes{' - town crop' if tag else ''}",
                           f"cells with turn room, in place before the enemy enters the seen stretch (own time + {base['P']['setup_s']:.0f} s), seeing >= 3 s; "
                           f"M: enemy recommended / fastest corridors; F: other corridors within 2x the fastest, not M")
            for c in enem:
                k_ = cls_of[c["id"]]
                if k_ == "M":
                    polyline(d, V, c["path"]["x"], c["path"]["z"], (40, 40, 40), 5)
                elif k_ == "F":
                    dashed(d, V, c["path"]["x"], c["path"]["z"], (40, 40, 160), 4)
                else:
                    polyline(d, V, c["path"]["x"], c["path"]["z"], (185, 185, 185), 2)
            for cl, pal, faint in (("F", PAL_F, (175, 195, 215)), ("M", PAL_M, (225, 190, 175))):
                regs = R[f"interdiction_{cl}"]
                for r in regs[len(pal):]:
                    paint_cells(im, S, V, r["cells"], faint)
                for k, r in enumerate(regs[:len(pal)]):
                    paint_cells(im, S, V, r["cells"], pal[k])
            d = ImageDraw.Draw(im)
            for cl, pal in (("F", PAL_F), ("M", PAL_M)):
                for k, r in enumerate(R[f"interdiction_{cl}"][:len(pal)]):
                    cx, cy = V.p(r["x"], r["z"])
                    if 0 <= cx < V.w and TITLE_H <= cy < TITLE_H + V.h:
                        d.text((cx + 6, cy - 26), r["label"], fill=pal[k], font=font(26, True), stroke_width=3, stroke_fill=(255, 255, 255))
            draw_zones(d, V, zones)
            draw_spawns(d, V, areas, poses)
            scale_bar(d, V)
            y = legend(d, V, [("box", PAL_M[0], "class M area (main routes), M1-M15"), ("box", (225, 190, 175), "other class M areas"),
                              ("box", PAL_F[0], "class F area (flank routes), F1-F15"), ("box", (175, 195, 215), "other class F areas"),
                              ("line", (40, 40, 40), f"army {b} main corridor (recommended / fastest)"),
                              ("text", None, f"dashed blue: army {b} flank corridor; light: other")], title="Legend")
            lines = ["#lab  len tanks first own  by spare  corr  sec  zones range   HD  conc"]
            for cl in ("M", "F"):
                for r in R[f"interdiction_{cl}"][:15]:
                    rg = f"{r['range_m'][0]}-{r['range_m'][1]}" if r.get("range_m") else "-"
                    lines.append(f"{r['label']:4s} {r['length_m']:4.0f} {r['tanks']:3d} {r['first_area']:4s} {r['own_t']:4.0f} {r['by']:4.0f} "
                                 f"{r['spare_s']:4.0f}  {r['class_corridors']:3d} {r['class_seconds']:5.0f}  {''.join(r['zones']):4s} {rg:>8s} "
                                 f"{r.get('hull_down_share', 0):.2f} {r['concealed_share']:.2f}")
            lines += ["", f"{len(R['interdiction_M'])} M and {len(R['interdiction_F'])} F areas (>= 12 m2)",
                      "corr / sec = corridors of the class watched / seconds;", "first = spawn area there first, own = its time,",
                      "by = in place by s; HD / conc = hull-down / concealed share"]
            text_block(d, V, lines, y + 10, size=15)
            p_ = od / f"5_interdiction_army{a}{tag}.png"
            im.save(p_)
            files.append(p_.name)
    # 6 fire support (areas as bands per zone)
    for a in (1, 2):
        R = res[a]
        V = VC
        im, d = canvas(layer(V, "faint"), V, f"6. Fire-support areas of army {a} onto the zones (town crop)",
                       f"cells held no later than {base['P']['fire_margin_s']:.0f} s after the enemy, with turn room, seeing >= 25 % of the zone; colour = zone seen")
        for L, v in R["fire"].items():
            for r in v["areas"]:
                paint_cells(im, S, V, r["cells"], ZONE_COL[L])
        d = ImageDraw.Draw(im)
        for L, v in R["fire"].items():
            for r in v["areas"][:6]:
                cx, cy = V.p(r["x"], r["z"])
                if 0 <= cx < V.w and TITLE_H <= cy < TITLE_H + V.h:
                    d.text((cx + 6, cy - 22), f"{L}{r['id']}", fill=(0, 0, 0), font=font(20, True), stroke_width=2, stroke_fill=(255, 255, 255))
        draw_zones(d, V, zones)
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        y = legend(d, V, [("box", ZONE_COL[L], f"area seeing zone {L} (overlaps: last drawn)") for L, *_r in zones], title="Legend")
        lines = ["#Holding X (within 200 m) gives fire onto Y", "#best share of Y seen / area seeing >= 25 % (m2)", "X\\Y    " + "        ".join(L for L, *_r in zones)]
        for X, *_r in zones:
            row = f"{X}    "
            for Y, *_q in zones:
                row += "     -      " if X == Y else f"{R['matrix'][X][Y]['best_seen']:.2f}/{R['matrix'][X][Y]['area_m2']:5d}  "
            lines.append(row)
        lines += ["", "#area len tanks first own  seen max/mean  appr s  range m  HD  conc"]
        for L, v in R["fire"].items():
            for r in v["areas"][:6]:
                lines.append(f"{L}{r['id']:<3d} {r['length_m']:4.0f} {r['tanks']:3d} {r['first_area']:4s} {r['own_t']:4.0f}  "
                             f"{r['zone_seen_max']:.2f}/{r['zone_seen_mean']:.2f}  {r['approach_seen_s']:5.0f}  "
                             f"{r['range_m'][0]}-{r['range_m'][1]}  {r.get('hull_down_share', 0):.2f} {r['concealed_share']:.2f}")
        text_block(d, V, lines, y + 10, size=15)
        p = od / f"6_fire_support_army{a}.png"
        im.save(p)
        files.append(p.name)
    # 7 summary per army
    for a in (1, 2):
        sh = base["sheet"][a]
        V = VC
        im, d = canvas(layer(V, "faint"), V, f"7. Summary for army {a}: the facts per zone (no combined score)",
                       "arrival lead (+ = we are first), corridors, exposure of our best corridor, support from the zone, delay by 1-2 tanks")
        draw_zones(d, V, zones)
        for L, x, z, r in zones:
            row = sh[L]
            ar = row["arrival"]
            af_ = row["arrival_fastest"]
            bc = row["recommended_corridor"]
            fc = row["fastest_corridor"]
            dl = row["delay"]
            hg = row["holding_gives"]
            txt = (f"{L}: lead {ar['lead_first_s']:+.0f} s by recommended routes\n"
                   f"  ({af_['lead_first_s']:+.0f} s by fastest), 5th tank {ar['lead_fifth_s']:+.0f} s\n"
                   f"corridors {row['corridors']['total']}, recommended {bc['id']} {bc['time']:.0f} s\n"
                   f"  seen {bc['expo_s']:.0f} s, narrowest {bc['narrowest']['width']:.1f} m\n"
                   + (f"  (fastest {fc['id']} {fc['time']:.0f} s, seen {fc['expo_s']:.0f} s)\n" if fc["id"] != bc["id"] else "") +
                   f"support: " + ", ".join(f"{Y} {v['best_seen']:.2f}" for Y, v in hg.items()) + "\n"
                   f"delay: 1 tank {dl['covered_by_1']}/{dl['enemy_corridors']}, 2 tanks {dl['covered_by_2']}/{dl['enemy_corridors']}\n"
                   f"fall-back {'yes' if dl['fallback'] else 'no'}\n"
                   f"unwatched enemy corridors: {', '.join(u['corridor'] for u in row['unwatched_enemy_corridors']) or 'none'}")
            cx, cy = V.p(x, z)
            d.multiline_text((cx + r * V.ppm + 8, cy - 60), txt, fill=(0, 0, 0), font=font(18, True), stroke_width=3,
                             stroke_fill=(255, 255, 255))
        for c in corrs:
            if c["army"] == a and c["role"] in ("recommended", "both"):
                polyline(d, V, c["path"]["x"], c["path"]["z"], (0, 0, 0), 7)
                polyline(d, V, c["path"]["x"], c["path"]["z"], ZONE_COL[c["zone"]], 4)
        labs = {r["label"]: r for r in res[a]["interdiction_M"] + res[a]["interdiction_F"]}
        shown = []
        for ar_id, cl_lists in res[a]["per_spawn_area"].items():
            for cl in ("M", "F"):
                for lb in cl_lists[cl]:
                    shown.append(labs[lb])
                    paint_cells(im, S, V, labs[lb]["cells"], (250, 120, 60) if cl == "M" else (60, 140, 230))
        d = ImageDraw.Draw(im)
        for r in shown:
            cx, cy = V.p(r["x"], r["z"])
            if 0 <= cx < V.w and TITLE_H <= cy < TITLE_H + V.h:
                d.text((cx + 8, cy - 24), r["label"], fill=(0, 0, 0), font=font(20, True), stroke_width=2, stroke_fill=(255, 255, 255))
        draw_spawns(d, V, areas, poses)
        scale_bar(d, V)
        lines = ["#Plain reading per spawn area"]
        import textwrap
        for ar_ in areas:
            if ar_["army"] != a:
                continue
            pl = res[a]["per_spawn_area"][ar_["id"]]
            lines.append(f"#{ar_['id']}  (within 60 s: M {', '.join(pl['M']) or '-'}; F {', '.join(pl['F']) or '-'})")
            for s in base["sentences"][ar_["id"]]:
                lines += textwrap.wrap(s, 62)
        lines += ["", "lines: our recommended corridors; bands: each spawn area's three best",
                  "areas it reaches first within 60 s (orange = M main routes, blue = F flank)"]
        text_block(d, V, lines, TITLE_H + 10, size=16)
        p = od / f"7_summary_army{a}.png"
        im.save(p)
        files.append(p.name)
    return files
