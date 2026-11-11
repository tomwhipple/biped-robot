"""Where the feet are on the board, from the front camera.

The robot walks on a brown board laid on a pale carpet, toward the camera. One frame gives, for each
foot, how far its sole's outer edge is from the board's edge on that side, and how far the feet are from
the bottom of the picture (the board runs out of view there). Colour does the work: the board is the
only strongly saturated brown in view (S ~160 of 255, its shadows too), the carpet is pale (S ~55-100),
the printed soles are grey-white (S < 40).

    margins(frame_bgr) -> dict(right_mm, left_mm, ahead_px, mm_per_px, ...)  or None if it cannot tell

right/left are the ROBOT's (it faces the camera, so its right foot is on the picture's left).
Scale comes from the soles themselves: each is SOLE_W wide.
"""
from __future__ import annotations

import cv2
import numpy as np

SOLE_W = 84.0     # mm, a sole's width


def board_mask(hsv):
    """The board: every sizeable saturated-brown patch (the robot and its shadow can cut it in pieces),
    and the convex hull of them all."""
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = ((s > 115) & (h >= 6) & (h <= 24) & (v > 20)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = [k for k in range(1, n) if st[k, cv2.CC_STAT_AREA] > 0.004 * m.size]
    if not keep or sum(st[k, cv2.CC_STAT_AREA] for k in keep) < 0.03 * m.size:
        return None
    board = np.isin(lab, keep).astype(np.uint8)
    fill = np.zeros_like(board)
    cv2.fillConvexPoly(fill, cv2.convexHull(cv2.findNonZero(board)), 1)
    return board, fill


def _line(y, x, tol=5.0):
    """x = a*y + b through the points, dropping the ones that do not lie on it."""
    ok = np.ones(len(y), bool)
    for _ in range(4):
        if ok.sum() < 8:
            return None
        c = np.polyfit(y[ok], x[ok], 1)
        r = np.abs(np.polyval(c, y) - x)
        new = r < max(tol, 2.0 * np.median(r[ok]))
        if (new == ok).all():
            break
        ok = new
    return c if ok.sum() >= 8 else None


def edges(board, y_from):
    """The board's left and right edges as lines x = a*y + b, fitted on the rows NEARER than the feet
    (from y_from down), where nothing stands on it."""
    H, W = board.shape
    rows = np.arange(min(y_from, H - 1), H - 1, 2)
    rows = np.array([y for y in rows if board[y].any()])
    if len(rows) < 10:
        return None
    xl = np.array([np.argmax(board[y]) for y in rows], float)
    xr = np.array([W - 1 - np.argmax(board[y][::-1]) for y in rows], float)
    ok_l, ok_r = xl > 2, xr < W - 3                     # an edge that left the picture is not an edge
    if ok_l.sum() < 8 or ok_r.sum() < 8:
        return None
    L, R = _line(rows[ok_l], xl[ok_l]), _line(rows[ok_r], xr[ok_r])
    return None if L is None or R is None else (L, R)


def soles(hsv, fill):
    """The two soles: the big grey-white blobs standing in (or just off) the board."""
    s, v = hsv[..., 1], hsv[..., 2]
    near = cv2.dilate(fill, np.ones((1, 121), np.uint8))          # a sole half off the edge still counts
    m = ((s < 45) & (v > 110) & (near > 0)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for k in range(1, n):
        x, y, w, h, a = st[k]
        if a > 400 and w > 30 and w > 1.2 * h:                    # a plate seen from the front: wide and low
            out.append(dict(x0=int(x), x1=int(x + w - 1), y0=int(y), y1=int(y + h - 1), area=int(a)))
    out.sort(key=lambda b: -b["y1"])
    out = [b for b in out if b["y1"] > out[0]["y1"] - 90][:2] if out else []
    return sorted(out, key=lambda b: b["x0"])


def margins(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV_FULL)        # H, S, V all 0..255
    bm = board_mask(hsv)
    if bm is None:
        return None
    board, fill = bm
    ss = soles(hsv, fill)
    if len(ss) != 2:
        return None
    a, b = ss                                                    # picture left = the robot's RIGHT foot
    yb = max(a["y1"], b["y1"])
    e = edges(board, yb + 6)
    if e is None:
        return None
    L, R = e
    mmpp = SOLE_W / (0.5 * ((a["x1"] - a["x0"]) + (b["x1"] - b["x0"])))
    right_px = a["x0"] - np.polyval(L, a["y1"])
    left_px = np.polyval(R, b["y1"]) - b["x1"]
    H = frame_bgr.shape[0]
    return dict(right_mm=float(right_px * mmpp), left_mm=float(left_px * mmpp), mm_per_px=float(mmpp),
                ahead_px=int(H - yb), soles=ss, edge_l=[float(x) for x in L], edge_r=[float(x) for x in R],
                sep_mm=float(((b["x0"] + b["x1"]) - (a["x0"] + a["x1"])) * 0.5 * mmpp))


def draw(frame_bgr, m):
    out = frame_bgr.copy()
    if m:
        H = out.shape[0]
        for ln, col in ((m["edge_l"], (0, 255, 0)), (m["edge_r"], (0, 255, 0))):
            cv2.line(out, (int(np.polyval(ln, 0)), 0), (int(np.polyval(ln, H)), H), col, 2)
        for s in m["soles"]:
            cv2.rectangle(out, (s["x0"], s["y0"]), (s["x1"], s["y1"]), (255, 0, 255), 2)
        cv2.putText(out, f"R {m['right_mm']:.0f} mm   L {m['left_mm']:.0f} mm   ahead {m['ahead_px']} px",
                    (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return out


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        im = cv2.imread(p)
        m = margins(im)
        print(p, None if m is None else {k: (round(v, 1) if isinstance(v, float) else v) for k, v in m.items()
                                         if k not in ("soles", "edge_l", "edge_r")})
        cv2.imwrite(p.rsplit(".", 1)[0] + "_bv.jpg", draw(im, m))
