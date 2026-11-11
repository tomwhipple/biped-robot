"""Where the feet are on the board, from the front camera.

The robot walks on a brown board laid on a pale carpet, toward the camera. One frame gives, for each
foot, how far its sole's outer edge is from the board's edge on that side, where each sole stands across
the board, and how far the feet are from the bottom of the picture (the board runs out of view there).

    margins(frame_bgr, board_w_mm=BOARD_W_MM, taper=None) -> dict(right_mm, left_mm, sole_r_mm, sole_l_mm, sep_mm, ahead_px,
                                                 mm_per_px, board_w_mm, conf, ...)   or None

right/left are the ROBOT's (it faces the camera, so its right foot is on the picture's left).
sole_r_mm / sole_l_mm: the sole's centre, mm from the board's centre line, + toward the robot's left.
conf (0..1) is the module's own confidence: lower when a board edge was inferred rather than seen, a sole
is cut off, the edge fit is thin. None means "cannot tell": a wrong number is worse.

No colour threshold is a fixed number. The frame's own statistics set them:
  * board vs carpet: an Otsu split of the saturation of the warm-hued pixels in the lower picture (the
    board is the saturated mode, the carpet the pale one, whatever the lamp);
  * soles: the carpet's saturation is the yardstick -- a sole is grey, well under half of it;
  * the soles are wide, low, pale plates, the legs above them narrow: a wide opening drops the legs.
Board edges are straight lines fitted by RANSAC to the per-row extent of the board (the robot, a sole off
the edge or a shadow cannot drag them: they are a minority of the rows). The two edges must converge
toward the horizon. If only one is trustworthy the other is inferred from the board's width, which is
returned (board_w_mm, with taper: how fast the edges converge) and can be handed back in on later frames;
an inferred edge is good to about 5 % of the board's width, and conf says so (0.4, or 0.3 without a taper).

Scale: ONE for the whole run: the board's width (board_w_mm, default BOARD_W_MM = 450 mm) over its width in
pixels at the soles' row. Every position, margin and separation is on that scale, so two frames are comparable
(a per-frame scale from the soles' own width, SOLE_W_MM = 92 mm, is only the fallback when board_w_mm=None; the
soles stand toed-out and their x-extent varies from frame to frame, 5 % or more).

    python board_view.py track <sony.mp4> [--every 6] [--board-w 450] > soles.csv
measures a recording: frame, video time, UTC (from <base>.pts next to it if present), soles, margins, sep, conf.
    python board_view.py frame.jpg ...      prints the numbers and writes frame_bv.jpg (annotated)
"""
from __future__ import annotations

import cv2
import numpy as np

BOARD_W_MM = 450.0    # mm: the board's width, the ONE scale of a run (positions = fraction across the board x this)
SOLE_W_MM = 92.0      # mm: x-extent of a sole in the picture, calibrated on the nominal stance (see above)
SEP_RANGE = (80.0, 220.0)   # mm, plausible centre-to-centre separation of the soles


def _prep(frame_bgr):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV_FULL)   # H, S, V all 0..255
    return hsv[..., 0], hsv[..., 1], hsv[..., 2]


def _cut_top(m):
    """The board ends at its far edge: going up from the bottom of the picture over the columns the board covers
    there, the first run of rows with next to nothing in them is the carpet beyond it. Everything above is not the
    board (in warm evening light the dark wall under the desk is as saturated as the board and the robot's legs
    join them)."""
    H = m.shape[0]
    cols = np.nonzero(m[H - 60:].mean(0) >= 0.5)[0]
    if len(cols) < 60:
        return m
    cov = m[:, cols[0]:cols[-1] + 1].mean(1)
    gap = 0
    for y in range(H - 61, -1, -1):
        gap = gap + 1 if cov[y] < 0.2 else 0
        if gap >= 10:
            m[:y + gap] = 0
            break
    return m


def board_mask(hue, sat, val, y0):
    """The board: the saturated mode of the warm-hued pixels below row y0, as the largest connected patch.
    Returns (mask, stats) or None. stats: board and carpet saturation, the split."""
    warm = ((hue >= 6) & (hue <= 36) & (val > 10)).astype(np.uint8)
    warm[:y0] = 0
    s = sat[warm > 0]
    if s.size < 2000:
        return None
    t, _ = cv2.threshold(s.reshape(-1, 1).astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lo, hi = s[s <= t], s[s > t]
    if lo.size < 500 or hi.size < 500:
        return None
    s_car, s_brd = float(np.median(lo)), float(np.median(hi))
    if s_brd - s_car < 35:                                   # no two modes: not this scene
        return None
    # board in a deep shadow stays warm-hued and keeps S ~ 90 while the carpet there goes grey-blue
    m = (((sat > t) | ((val < 65) & (sat > 60))) & (warm > 0)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    m = _cut_top(m)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 0.04 * m.size:
        return None
    return (lab == k), dict(t=float(t), s_car=s_car, s_brd=s_brd)


def _ransac(ys, xs, y_clean=None, tol=3.0, iters=200, seed=1):
    """x = a*y + b through the majority of the points on the rows nearer than the feet (y >= y_clean: nothing
    stands there), all the rows if those are too few. Returns (coef, inlier share of those rows, rows used)
    or None."""
    pool = np.nonzero(ys >= y_clean)[0] if y_clean is not None else np.arange(len(ys))
    if len(pool) < 40:
        pool = np.arange(len(ys))
    if len(pool) < 40:
        return None
    ys, xs = ys[pool], xs[pool]
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        i, j = rng.choice(len(ys), 2, replace=False)
        if abs(ys[i] - ys[j]) < 30:
            continue
        a = (xs[j] - xs[i]) / (ys[j] - ys[i])
        n = int((np.abs(a * ys + (xs[i] - a * ys[i]) - xs) < tol).sum())
        if n > best_n:
            best, best_n = (a, xs[i] - a * ys[i]), n
    if best is None:
        return None
    ok = np.abs(best[0] * ys + best[1] - xs) < tol
    for _ in range(2):
        if ok.sum() < 20:
            return None
        c = np.polyfit(ys[ok], xs[ok], 1)
        ok = np.abs(np.polyval(c, ys) - xs) < tol
    if ok.sum() < 30 or np.ptp(ys[ok]) < 60:
        return None
    return np.polyfit(ys[ok], xs[ok], 1), float(ok.sum()) / len(ys), len(ys)


def edges(board, y_clean=None):
    """The board's left and right edge lines x = a*y + b, each (coef, inlier share) or None (not reliably
    seen: under 70 % of the rows agree). The rows come from the board's per-row extent below the feet, where
    neither the robot nor a sole standing off the edge can hide it."""
    H, W = board.shape
    rows = np.nonzero(board.sum(1) >= 8)[0]
    if len(rows) < 60:
        return None, None
    xl = np.array([np.argmax(board[y]) for y in rows], float)
    xr = np.array([W - 1 - np.argmax(board[y][::-1]) for y in rows], float)
    out = []
    for xs, edge_gone in ((xl, xl <= 2), (xr, xr >= W - 3)):    # an edge that left the picture is not an edge
        keep = ~edge_gone
        r = _ransac(rows[keep].astype(float), xs[keep], y_clean) if keep.sum() > 30 else None
        out.append(None if r is None or r[1] < 0.7 else (r[0], r[1]))
    return out[0], out[1]


def soles(hue, sat, val, board, s_car):
    """Grey plates first; if that does not give a pair, the not-board-not-dark rule inside the board's outline is added (evening light)."""
    for bright, strict in ((False, True), (True, False), (False, False)):
        out = _soles(hue, sat, val, board, s_car, bright, strict)
        if len(out) == 2:
            return out
    return out


def _soles(hue, sat, val, board, s_car, use_bright, strict):
    """The two soles as dicts (x0, x1, y0, y1, xc, xl_row, xr_row): wide, low, pale-grey plates on or beside
    the board, in light or in shadow (grey at any brightness: the saturation stays low). Legs and servos
    above them are narrower than the opening kernel. A candidate must have the robot's own grey-or-black
    structure above it (a shadow on the carpet is wide and low too, but nothing stands on it)."""
    H, W = sat.shape
    ys = np.nonzero(board.any(1))[0]
    y_top = int(ys[0]) if len(ys) else H // 2
    # grey, or a shadow's cool tint on grey (the warm lamp does not reach a shadowed plate): not warm-saturated
    neutral = (sat < max(14.0, 0.5 * s_car)) | ((hue > 60) & (hue < 200))
    grey = (neutral & (val > 22)).astype(np.uint8)
    grey[:y_top] = 0
    pts = cv2.findNonZero(board.astype(np.uint8))
    fill = np.zeros((H, W), np.uint8)
    if pts is not None:                                           # a sole may hang off the board's edge, not stand far off it
        cv2.fillConvexPoly(fill, cv2.convexHull(pts), 1)
        # In warm evening light a plate's saturation is no longer clearly below the carpet's (and the robot's own
        # shadow tints it), but a plate is still the one thing on the board that is neither board-coloured nor
        # dark: inside the board's outline, not board, not black. Only inside the outline: the carpet beside the
        # board is just as pale.
        v_board = float(np.median(val[board])) if board.any() else 128.0
        bright = ((val > 0.55 * v_board) & (board == 0) & (fill > 0)).astype(np.uint8)
        grey = (grey | bright if use_bright else grey) & cv2.dilate(fill, np.ones((1, 121), np.uint8))
    g = cv2.morphologyEx(grey, cv2.MORPH_CLOSE, np.ones((5, 9), np.uint8))
    g = cv2.morphologyEx(g, cv2.MORPH_OPEN, np.ones((7, 41), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(g, connectivity=8)
    robot = (neutral & (val > 12)) & ~board
    out = []
    for k in range(1, n):
        x, y, w, h, a = st[k]
        if a < (700 if strict else 450) or w < 55 or w > 150 or h < (12 if strict else 8):
            continue
        m0 = lab == k
        m = m0
        # the plate: what survives an opening wider than the ankle bracket on it (60 % of the widest row);
        # a toed-out plate is a parallelogram, so this keeps all of it, not only its widest rows
        kw = max(41, int(0.72 * m.sum(1).max()))
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, kw), np.uint8)) > 0
        rows = np.nonzero(m.any(1))[0]
        if len(rows) == 0:
            continue
        rows = rows[rows >= rows.max() - 90]
        y0, y1 = int(rows.min()), int(rows.max())
        a = int(m[y0:y1 + 1].sum())
        # the opening trims the toed-out plate's pointed corners: take them back from the unopened patch
        y0, y1 = max(0, y0 - 6), min(H - 1, y1 + 12)
        rr = np.nonzero(m0[y0:y1 + 1].sum(1) >= 6)[0]
        y0, y1 = y0 + int(rr.min()), y0 + int(rr.max())
        pm = m0[y0:y1 + 1]
        cx = np.nonzero(pm.any(0))[0]
        x0, x1 = int(cx.min()), int(cx.max())
        ym = min(H - 1, (y0 + y1) // 2)
        cols_in = np.nonzero(fill[ym])[0]                          # the board's outline at its row: a sole stands on the
        if len(cols_in) and not (cols_in[0] - 0.5 * (x1 - x0) <= 0.5 * (x0 + x1) <= cols_in[-1] + 0.5 * (x1 - x0)):
            continue                                               # board or hangs half off it; a strap beside it does not
        sup = float(robot[max(0, y0 - 90):max(1, y0 - 8), x0:x1 + 1].mean())   # structure standing on it
        if sup < 0.08 or pm.sum() < 0.55 * (x1 - x0 + 1) * (y1 - y0 + 1):
            continue
        b = dict(x0=x0, x1=x1, y0=y0, y1=y1, area=int(a), xc=0.5 * (x0 + x1), sup=round(sup, 2))
        for name, col in (("xl_row", 0), ("xr_row", pm.shape[1] - 1)):
            r = np.nonzero(pm[:, max(0, col - 2):col + 3].any(1))[0]
            b[name] = float(y0 + (r.mean() if len(r) else pm.shape[0] - 1))
        out.append(b)
    # the pair: similar widths, feet at about the same row, a plausible gap between them
    best, best_sc = [], -1.0
    for i in range(len(out)):
        for j in range(i + 1, len(out)):
            p, q = sorted((out[i], out[j]), key=lambda t: t["xc"])
            wp, wq = p["x1"] - p["x0"], q["x1"] - q["x0"]
            if q["x0"] < p["x1"] - 0.3 * min(wp, wq):
                continue
            sc = min(wp, wq) / max(wp, wq) - abs(p["y1"] - q["y1"]) / 100.0 + 0.3 * (p["sup"] + q["sup"])
            if abs(p["y1"] - q["y1"]) < 60 and sc > best_sc:
                best, best_sc = [p, q], sc
    return best


def margins(frame_bgr, board_w_mm=BOARD_W_MM, taper=None):
    """board_w_mm: the board's width (mm): THE scale, the same on every frame of a run (default BOARD_W_MM): a
    sole's position is its fraction across the board at its row times this. None: scale from the soles'
    own width instead (SOLE_W_MM; noisy, for a first look at an unknown board only). taper: slope(right edge) - slope(left edge) from earlier frames (the board narrows
    toward the horizon), so an inferred edge converges like the real one. Both come back from a frame where
    both edges were seen (board_w_mm, taper); without them an inferred edge is drawn parallel (conf 0.35)."""
    H, W = frame_bgr.shape[:2]
    hue, sat, val = _prep(frame_bgr)
    bm = board_mask(hue, sat, val, int(0.4 * H))
    if bm is None:
        return None
    board, stat = bm
    ss = soles(hue, sat, val, board, stat["s_car"])
    if len(ss) != 2:
        return None
    a, b = ss                                                    # picture left = the robot's RIGHT foot
    wa, wb = a["x1"] - a["x0"], b["x1"] - b["x0"]
    if max(wa, wb) > 1.6 * min(wa, wb) or a["x1"] >= b["x0"] + 0.5 * wb:
        return None
    yb = 0.5 * (a["y1"] + b["y1"])
    mm_sole = SOLE_W_MM / (0.5 * (wa + wb))
    if not SEP_RANGE[0] <= (b["xc"] - a["xc"]) * mm_sole <= SEP_RANGE[1] * 1.15:
        return None
    EL, ER = edges(board, int(max(a["y1"], b["y1"])) + 4)
    if EL is None and ER is None:
        return None
    conf = 1.0
    both = EL is not None and ER is not None
    if both:
        L, R = EL[0], ER[0]
        bw_px = float(np.polyval(R, yb) - np.polyval(L, yb))
        if bw_px < 100 or np.polyval(R, yb - 150) - np.polyval(L, yb - 150) > bw_px * 1.02:
            return None                                          # not a board seen end-on: edges must converge upward
        mmpp = board_w_mm / bw_px if board_w_mm else mm_sole
    else:
        if not board_w_mm:
            return None                                          # one edge and no width to infer the other from
        mmpp = mm_sole
        bw_px = board_w_mm / mmpp
        t = taper or 0.0
        if EL is not None:
            L = EL[0]
            R = np.array([L[0] + t, np.polyval(L, yb) + bw_px - (L[0] + t) * yb])
        else:
            R = ER[0]
            L = np.array([R[0] - t, np.polyval(R, yb) - bw_px - (R[0] - t) * yb])
        conf *= 0.4 if taper is not None else 0.3     # below the 0.5 a run accepts: the scale of an inferred edge is the soles'
    xl, xr = np.polyval(L, a["xl_row"]), np.polyval(R, b["xr_row"])    # the edges where each sole's outer corner is
    mid = 0.5 * (np.polyval(L, yb) + np.polyval(R, yb))
    if max(a["y1"], b["y1"]) >= H - 2:                           # a sole cut off by the picture's bottom is not fully seen
        conf *= 0.5
    conf *= max(0.3, 1.0 - 0.8 * abs(wa - wb) / max(wa, wb))
    conf *= 1.0 if stat["s_brd"] - stat["s_car"] > 60 else 0.7
    return dict(right_mm=float((a["x0"] - xl) * mmpp), left_mm=float((xr - b["x1"]) * mmpp), mm_per_px=float(mmpp),
                ahead_px=int(H - max(a["y1"], b["y1"])), soles=ss, edge_l=[float(x) for x in L], edge_r=[float(x) for x in R],
                sep_mm=float((b["xc"] - a["xc"]) * mmpp),
                sole_r_mm=float((a["xc"] - mid) * mmpp), sole_l_mm=float((b["xc"] - mid) * mmpp),
                board_w_mm=float(bw_px * mmpp) if both else None, taper=float(R[0] - L[0]) if both else None,
                board_px=float(bw_px), inferred=None if both else ("left" if EL is None else "right"),
                conf=float(max(0.0, min(1.0, conf))))


def draw(frame_bgr, m):
    out = frame_bgr.copy()
    if m:
        H = out.shape[0]
        for ln in (m["edge_l"], m["edge_r"]):
            cv2.line(out, (int(np.polyval(ln, 0)), 0), (int(np.polyval(ln, H)), H), (0, 255, 0), 2)
        for s in m["soles"]:
            cv2.rectangle(out, (s["x0"], s["y0"]), (s["x1"], s["y1"]), (255, 0, 255), 2)
        cv2.putText(out, f"R {m['right_mm']:.0f}  L {m['left_mm']:.0f}  sep {m['sep_mm']:.0f} mm  ahead {m['ahead_px']} px"
                    f"  conf {m['conf']:.2f}", (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    return out


def _pts(path):
    """Per-frame UTC (s) from the mkvtimestamp_v2 file beside the video (<base>_sony.pts for <base>_sony.mp4), or []."""
    import os
    p = os.path.splitext(path)[0] + ".pts"
    if not os.path.exists(p):
        return []
    return [float(x) / 1e3 for x in open(p).read().split("\n") if x.strip() and not x.startswith("#")]


def track(path, every=6, board_w_mm=BOARD_W_MM, out=None):
    """Measure every `every`-th frame of a recording. Writes CSV to `out` (default stdout); returns the rows."""
    import csv
    import sys
    out = out or sys.stdout
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    pts = _pts(path)
    w = csv.writer(out)
    cols = ("sole_r_mm", "sole_l_mm", "right_mm", "left_mm", "sep_mm", "conf")
    w.writerow(("frame", "t_s", "utc") + cols)
    rows, i = [], 0
    while True:
        ok, im = cap.read()
        if not ok:
            break
        if i % every == 0:
            m = margins(im, board_w_mm)
            t = (pts[i] - pts[0]) if i < len(pts) else (i / fps if fps else "")
            utc = f"{pts[i]:.3f}" if i < len(pts) else ""
            row = [i, f"{t:.3f}" if t != "" else "", utc] + ([f"{m[c]:.1f}" if c != "conf" else f"{m[c]:.2f}" for c in cols] if m else [""] * len(cols))
            w.writerow(row)
            rows.append(row)
        i += 1
    cap.release()
    return rows


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "track":
        import argparse
        ap = argparse.ArgumentParser(prog="board_view.py track")
        ap.add_argument("video")
        ap.add_argument("--every", type=int, default=6)
        ap.add_argument("--board-w", type=float, default=BOARD_W_MM)
        a = ap.parse_args(sys.argv[2:])
        track(a.video, a.every, a.board_w)
        sys.exit(0)
    for p in sys.argv[1:]:
        im = cv2.imread(p)
        m = margins(im)
        print(p, None if m is None else {k: (round(v, 1) if isinstance(v, float) else v) for k, v in m.items()
                                         if k not in ("soles", "edge_l", "edge_r")})
        cv2.imwrite(p.rsplit(".", 1)[0] + "_bv.jpg", draw(im, m))
