#!/usr/bin/env python3
"""Pre-run check: where is the robot on the board, and is there room to walk?

    tools/vision/where.py [--image F.jpg | --camd-dir DIR] [--direction forward|back|left|right|heading]
        [--min-margin 250] [--config vision.json] [--max-age 2] [--json]

Looks at ONE frame, never holds the camera: it reads DIR/sony_latest.jpg that
tools/vision/camd.py keeps (--max-age s: refuse a stale one), or --image. With neither
it takes a single frame from the Sony itself, but only if nothing else is reading it
(camd lock / another liveview.py = refuse, exit 4).

Exit 0: margin in the requested direction >= --min-margin (or no direction asked).
Exit 3: margin below the threshold.  Exit 2: robot/board tags not found.
Exit 4: no frame (camera busy or stale).
Direction is relative to the BOARD (forward = toward the far end = +y, back = -y,
left/right as seen from the Sony) or `heading` = along the robot's own heading; the margin
is the distance from the robot (torso point, minus foot_margin_mm) to the edge reached
along that direction.

Library:  from where import where;  r = where(gray, cfg, direction="forward", min_margin=250)
Host tool: before the track starts, in tools/track_host.py:
    import subprocess; rc = subprocess.run([sys.executable, "tools/vision/where.py",
        "--camd-dir", CAMD_DIR, "--direction", "forward", "--min-margin", "300"]).returncode
    if rc not in (0,): abort / ask  (rc 2 = tags not visible, treat as "unknown", not "safe")
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vgeom as G  # noqa: E402
from camd import DEFAULT_DIR  # noqa: E402
from tag_track import Tracker  # noqa: E402
from tagdefs import detect  # noqa: E402

DIRS = {"forward": (0.0, 1.0), "back": (0.0, -1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)}


def margin_along(cfg, x, y, dx, dy):
    """Distance from (x, y) to the board edge along unit direction (dx, dy), minus the foot margin."""
    W, L, m = cfg["board"]["width_mm"], cfg["board"]["length_mm"], cfg.get("foot_margin_mm", 0.0)
    best = math.inf
    if dx > 1e-9:
        best = min(best, (W - x) / dx)
    if dx < -1e-9:
        best = min(best, (0 - x) / dx)
    if dy > 1e-9:
        best = min(best, (L - y) / dy)
    if dy < -1e-9:
        best = min(best, (0 - y) / dy)
    return best - m


def where(gray, cfg=None, direction=None, min_margin=None, tracker=None):
    """One frame -> dict(found, x_mm, y_mm, heading_deg, margins, margin_dir_mm, ok, reason)."""
    cfg = cfg or G.load_config()
    tr = tracker or Tracker(cfg, (gray.shape[1], gray.shape[0]))
    d = detect(gray, tr.det)
    if tracker is None:
        cal = tr.calibrate([d])
        if cal is None:
            return dict(found=False, ok=None, reason="fewer than 2 board tags visible", tags=sorted(d))
    r = tr.process(gray, None, d)
    if r.get("x_mm") is None:
        return dict(found=False, ok=None, reason="no robot torso tag visible", tags=sorted(d))
    out = dict(found=True, x_mm=r["x_mm"], y_mm=r["y_mm"], heading_deg=r["heading_deg"],
               pitch_deg=r["pitch_deg"], roll_deg=r["roll_deg"], ambiguous=r.get("ambiguous"),
               margins={k: r[f"margin_{k}_mm"] for k in ("left", "right", "near", "far")}, tags=r["tags"],
               f_px=tr.K[0, 0])
    if direction:
        if direction == "heading":
            h = math.radians(r["heading_deg"])
            vec = (-math.sin(h), math.cos(h))          # heading 0 = +y, counter-clockwise = toward -x
        else:
            vec = DIRS[direction]
        out["margin_dir_mm"] = margin_along(cfg, r["x_mm"], r["y_mm"], *vec)
        out["direction"] = direction
        if min_margin is not None:
            out["ok"] = bool(out["margin_dir_mm"] >= min_margin)
            out["reason"] = None if out["ok"] else f"{out['margin_dir_mm']:.0f} mm to the edge < {min_margin:.0f}"
    return out


def grab(camd_dir, max_age):
    """Newest frame without disturbing the camera: camd's latest.jpg, else one direct frame
    if nobody else holds the Sony."""
    p = os.path.join(camd_dir, "sony_latest.jpg")
    if os.path.exists(p):
        age = time.time() - os.path.getmtime(p)
        if age <= max_age:
            return cv2.imread(p, cv2.IMREAD_GRAYSCALE), f"camd latest.jpg ({age:.1f} s old)"
        return None, f"camd latest.jpg is {age:.0f} s old (daemon stopped or camera stalled)"
    if os.path.exists(os.path.join(camd_dir, "sony.lock")):
        import fcntl
        fd = os.open(os.path.join(camd_dir, "sony.lock"), os.O_RDWR)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return None, "camd holds the camera but writes no latest.jpg"
        finally:
            os.close(fd)
    if subprocess.run(["pgrep", "-f", "^/usr/bin/python3 -u .*[l]iveview\\.py"], capture_output=True).returncode == 0:
        return None, "another liveview.py reader holds the camera"
    # one direct frame, 3 s at most (the single-reader rule: nothing else runs, checked above)
    tmp = "/tmp/where_frame.mp4"
    if os.path.exists(tmp):
        os.unlink(tmp)
    subprocess.run([os.path.join(HERE, "..", "sony_record.sh"), tmp[:-4], "2", "where"], capture_output=True, timeout=40)
    cap = cv2.VideoCapture(tmp)
    ok, f = cap.read()
    cap.release()
    return (cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), "direct Sony frame") if ok else (None, "no frame from the Sony")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image")
    ap.add_argument("--camd-dir", default=DEFAULT_DIR)
    ap.add_argument("--direction", choices=["forward", "back", "left", "right", "heading"])
    ap.add_argument("--min-margin", type=float, default=250.0)
    ap.add_argument("--max-age", type=float, default=2.0)
    ap.add_argument("--config")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    if a.image:
        gray, src = cv2.imread(a.image, cv2.IMREAD_GRAYSCALE), a.image
    else:
        gray, src = grab(a.camd_dir, a.max_age)
    if gray is None:
        print(f"where: no frame: {src}", file=sys.stderr)
        return 4
    r = where(gray, G.load_config(a.config), a.direction, a.min_margin if a.direction else None)
    r["frame_source"] = src
    if a.json:
        print(json.dumps(r, indent=1, default=float))
    elif r["found"]:
        m = r["margins"]
        print(f"robot at x {r['x_mm']:.0f} y {r['y_mm']:.0f} mm, heading {r['heading_deg']:.0f} deg, "
              f"tilt p {r['pitch_deg']:.0f} r {r['roll_deg']:.0f}; margins L {m['left']:.0f} R {m['right']:.0f} "
              f"near {m['near']:.0f} far {m['far']:.0f} mm" + (" (pose ambiguous)" if r.get("ambiguous") else ""))
        if a.direction:
            print(f"{a.direction}: {r['margin_dir_mm']:.0f} mm to the edge -> {'OK' if r['ok'] else 'TOO CLOSE'}")
    else:
        print(f"where: {r['reason']} (tags seen: {r['tags']})", file=sys.stderr)
    if not r["found"]:
        return 2
    return 0 if r.get("ok") in (True, None) else 3


if __name__ == "__main__":
    sys.exit(main())
