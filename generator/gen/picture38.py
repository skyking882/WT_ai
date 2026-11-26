"""network.png of the road network (Pillow only; no matplotlib). North (+Z) up, +X right, 0.5 m per pixel."""
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

QCOL = [(230, 25, 75), (60, 180, 75), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230),
        (128, 128, 0), (0, 128, 128), (170, 110, 40), (128, 0, 0), (0, 0, 128)]


def font(size):
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def viridis(t):
    stops = np.array([(68, 1, 84), (59, 82, 139), (33, 145, 140), (94, 201, 98), (253, 231, 37)], float)
    t = min(max(t, 0.0), 1.0) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    c = stops[i] + (stops[i + 1] - stops[i]) * (t - i)
    return tuple(int(v) for v in c)


def picture(path, M, NET, uses, poses, targets, qpaths, Q, res=0.5, pad=40.0):
    xs, zs = NET["sx"].astype(np.float64), NET["sz"].astype(np.float64)
    X0, X1 = xs.min() - pad, xs.max() + pad
    Z0, Z1 = zs.min() - pad, zs.max() + pad
    W, H = int((X1 - X0) / res), int((Z1 - Z0) / res)
    hard = M["hard"]
    f = int(round(res / M["cell"]))
    c0 = max(0, int((X0 - M["x0"]) / M["cell"]))
    r0 = max(0, int((Z0 - M["z0"]) / M["cell"]))
    sub = hard[r0:r0 + H * f, c0:c0 + W * f]
    hh, ww = sub.shape[0] // f, sub.shape[1] // f
    hb = sub[:hh * f, :ww * f].reshape(hh, f, ww, f).any(axis=(1, 3))
    img = np.full((H, W, 3), 255, np.uint8)
    ox, oz = int(round((M["x0"] + c0 * M["cell"] - X0) / res)), int(round((M["z0"] + r0 * M["cell"] - Z0) / res))
    hb_full = np.zeros((H, W), bool)
    y1, x1 = min(H, oz + hh), min(W, ox + ww)
    hb_full[max(0, oz):y1, max(0, ox):x1] = hb[max(0, -oz):y1 - oz, max(0, -ox):x1 - ox]
    img[hb_full] = (165, 165, 165)
    im = Image.fromarray(img[::-1].copy())
    dr = ImageDraw.Draw(im)

    def p(x, z):
        return ((x - X0) / res, (Z1 - z) / res)
    lu = np.log10(np.maximum(uses, 1))
    lmax = max(float(lu.max()), 1e-9)
    order = np.argsort(lu)
    for e in order:
        a, n = int(NET["efirst"][e]), int(NET["en"][e])
        st = max(1, n // 60)
        idx = np.r_[np.arange(a, a + n, st), a + n - 1]
        t = lu[e] / lmax
        dr.line([p(xs[i], zs[i]) for i in idx], fill=viridis(0.1 + 0.9 * t), width=1 + int(round(4 * t)))
    fl = NET["nflags"]
    for i in range(len(NET["nx"])):
        x, y = p(NET["nx"][i], NET["nz"][i])
        if fl[i] & 2:
            r, c = 3, (0, 60, 255)
        elif fl[i] & 1:
            r, c = 2.5, (220, 0, 0)
        else:
            r, c = 1.5, (0, 0, 0)
        dr.ellipse([x - r, y - r, x + r, y + r], fill=c)
    for k, P in enumerate(qpaths):
        if P is None or not len(P):
            continue
        dr.line([p(x, z) for x, z in P[::4]] + [p(*P[-1])], fill=QCOL[k % len(QCOL)], width=4)
    fb, fs = font(30), font(20)
    for t in targets:
        if t[1] == 0:
            x, y = p(t[5], t[6])
            r = t[9] / res
            dr.ellipse([x - r, y - r, x + r, y + r], outline=(220, 0, 0), width=3)
            dr.text((x - 40, y - 15), t[0], fill=(220, 0, 0), font=fb)
    areas = {}
    for pid, x, z, hd in poses:
        areas.setdefault(pid.split(".")[0], []).append((x, z))
    for a, v in areas.items():
        v = np.array(v)
        x, y = p(v[:, 0].mean(), v[:, 1].max())
        dr.text((x - 50, y - 45), f"spawn {a}", fill=(0, 60, 255), font=fb)
    for k, P in enumerate(qpaths):
        if P is None or not len(P):
            continue
        x, y = p(*P[0])
        dr.text((x + 6, y - 6 - 22 * (k % 3)), Q[k][0], fill=QCOL[k % len(QCOL)], font=fs)
    # legend strip
    leg = Image.new("RGB", (W, 150), (255, 255, 255))
    dl = ImageDraw.Draw(leg)
    dl.text((10, 8), f"s38 network: {len(NET['nx'])} nodes, {len(NET['ea'])} edges; line width / colour = log10 uses "
                     f"(max {int(uses.max())}); hard cells grey; +X right, +Z up, {res} m per pixel", fill=(0, 0, 0), font=fs)
    dl.text((10, 38), "nodes: black = joint / chain end, red = target end, blue = spawn pose; red circles = zones",
            fill=(0, 0, 0), font=fs)
    x = 10
    for k in range(len(Q)):
        dl.line([(x, 90), (x + 40, 90)], fill=QCOL[k % len(QCOL)], width=5)
        dl.text((x + 46, 78), Q[k][0], fill=(0, 0, 0), font=fs)
        x += 110
    out = Image.new("RGB", (W, H + 150), (255, 255, 255))
    out.paste(leg, (0, 0))
    out.paste(im, (0, 150))
    out.save(path, optimize=True)
