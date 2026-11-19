#!/usr/bin/env python3
"""Cheap image-only hint that something other than the robot moved near it.

    tools/vision/foreign.py VIDEO.mp4 [--start UTC] [--end UTC]

Foreground = difference from the clip's median frame (global brightness removed),
at 192x128. The robot is the foreground component(s) in the leg rows; everything
else that is foreground and large enough is "foreign". An event is >= min_s of
frames whose foreign area is >= min_area (fraction of the frame). Reported with
where it was (centroid, whether it touches the frame edge, its pixel distance to
the robot).

Needs no tags. It cannot tell a hand from the person in the chair, a sock or the
dog: it says "look at the key frames", nothing more. Measured on the 2026-11-16
footage in tools/vision/README.md.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_track import frames  # noqa: E402

W, H = 192, 128


def foreign_motion(video, start=None, end=None, step=3, min_area=0.008, min_s=0.5, thr=25):
    """-> dict(events=[...], max_area, series=[(t, area)]). t are UTC epoch s from the .pts."""
    ts, fr = [], []
    for n, t, bgr in frames(video, None, 1, start, end):
        if n % step:
            continue
        g = cv2.GaussianBlur(cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (W, H),
                                        interpolation=cv2.INTER_AREA), (0, 0), 1.5)
        fr.append(g.astype(np.float32))
        ts.append(t)
    if len(fr) < 5:
        return dict(events=[], max_area=None, series=[], note="too few frames")
    bg = np.median(np.array(fr), axis=0)
    series, info = [], []
    for g, t in zip(fr, ts):
        d = g - bg
        d -= np.median(d)
        m = cv2.morphologyEx((np.abs(d) > thr).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        # robot: the largest components reaching the leg rows (middle of the frame)
        legs = [i for i in range(1, n) if st[i, 4] >= 60 and st[i, 1] < 0.8 * H and st[i, 1] + st[i, 3] > 0.45 * H]
        legs.sort(key=lambda i: -st[i, 4])
        rob = legs[:1]
        if rob:
            x0, x1 = st[rob[0], 0] - 6, st[rob[0], 0] + st[rob[0], 2] + 6
        else:
            x0 = x1 = None
        area, best = 0, None
        for i in range(1, n):
            a = st[i, 4]
            if a < 40 or i in rob:
                continue
            xs, xe = st[i, 0], st[i, 0] + st[i, 2]
            if x0 is not None and xe > x0 and xs < x1 and st[i, 1] + st[i, 3] > 0.3 * H:
                continue         # overlapping the robot's column: part of the robot
            area += a
            if best is None or a > best[0]:
                best = (a, i)
        series.append((t, area / (W * H)))
        if best:
            i = best[1]
            x, y, w, h, a = st[i]
            dist = 0 if x0 is None else int(max(x - x1, x0 - (x + w), 0))
            info.append(dict(cx=float((x + w / 2) / W), cy=float((y + h / 2) / H),
                             edge=bool(x == 0 or x + w == W or y == 0 or y + h == H), dist_to_robot_px=dist))
        else:
            info.append(None)
    events, cur = [], None
    for (t, a), inf in zip(series, info):
        if a >= min_area:
            if cur is None:
                cur = dict(t_start=t, t_end=t, t_peak=t, peak=a, where=inf)
            cur["t_end"] = t
            if a > cur["peak"]:
                cur["peak"], cur["where"], cur["t_peak"] = a, inf, t
        elif cur is not None and t - cur["t_end"] > 0.5:
            events.append(cur)
            cur = None
    if cur:
        events.append(cur)
    events = [e for e in events if e["t_end"] - e["t_start"] >= min_s]
    return dict(events=events, max_area=float(max(a for _, a in series)), series=series)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--start", type=float)
    ap.add_argument("--end", type=float)
    a = ap.parse_args(argv)
    r = foreign_motion(a.video, a.start, a.end)
    r.pop("series")
    print(json.dumps(r, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
