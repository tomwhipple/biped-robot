"""Angle of a long straight edge in a webcam frame, to calibrate plumb/level of
the Plan B rig's fork from the camera (2026-10-02). numpy + scipy only.

For each pixel row in [y0, y1) the horizontal intensity gradient (Sobel) is
searched in [x0, x1) for its strongest extremum of the requested polarity,
refined to sub-pixel by a parabola; a RANSAC line x = a*y + b through those
points gives the edge's tilt from image vertical: atan(a), + = leaning right
going down. Usage:
  edge_angle.py frame.jpg x0 x1 y0 y1 {rise|fall} [more x0 x1 y0 y1 pol ...]
rise: dark -> bright left to right; fall: bright -> dark.
"""
import math, sys
import numpy as np
from PIL import Image
from scipy import ndimage


def edge_points(img, x0, x1, y0, y1, pol):
    g = ndimage.sobel(ndimage.gaussian_filter(img, 1.0), axis=1)
    if pol == "fall":
        g = -g
    pts = []
    for y in range(y0, y1):
        row = g[y, x0:x1]
        i = int(np.argmax(row))
        if row[i] < 40 or i == 0 or i == len(row) - 1:
            continue
        a, b, c = row[i - 1], row[i], row[i + 1]
        den = a - 2 * b + c
        off = 0.5 * (a - c) / den if den else 0.0
        pts.append((y, x0 + i + off, row[i]))
    return np.array(pts)


def ransac_line(pts, tol=1.0, iters=400, seed=0):
    rng = np.random.default_rng(seed)
    ys, xs = pts[:, 0], pts[:, 1]
    best = None
    for _ in range(iters):
        i, j = rng.choice(len(pts), 2, replace=False)
        if ys[i] == ys[j]:
            continue
        a = (xs[j] - xs[i]) / (ys[j] - ys[i]); b = xs[i] - a * ys[i]
        inl = np.abs(xs - (a * ys + b)) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    a, b = np.polyfit(ys[best], xs[best], 1)
    res = xs[best] - (a * ys[best] + b)
    n = best.sum()
    se = res.std(ddof=2) / math.sqrt(((ys[best] - ys[best].mean()) ** 2).sum()) if n > 2 else float("nan")
    return a, b, n, res.std(), se


def measure(path, specs):
    img = np.asarray(Image.open(path).convert("L")).astype(float)
    out = []
    for x0, x1, y0, y1, pol in specs:
        pts = edge_points(img, x0, x1, y0, y1, pol)
        if len(pts) < 10:
            out.append(None); continue
        a, b, n, sd, se = ransac_line(pts)
        out.append({"tilt_deg": math.degrees(math.atan(a)), "se_deg": math.degrees(se),
                    "inliers": int(n), "rows": len(pts), "resid_px": float(sd),
                    "x_at_y0": float(a * y0 + b), "x_at_y1": float(a * y1 + b)})
    return out


if __name__ == "__main__":
    path, rest = sys.argv[1], sys.argv[2:]
    specs = [(int(rest[i]), int(rest[i + 1]), int(rest[i + 2]), int(rest[i + 3]), rest[i + 4])
             for i in range(0, len(rest), 5)]
    for s, r in zip(specs, measure(path, specs)):
        print(s, r if r is None else {k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()})
