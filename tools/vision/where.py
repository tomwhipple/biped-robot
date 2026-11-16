#!/usr/bin/env python3
"""Pre-run check: where is the robot on the board, and is there room to walk?

    tools/vision/where.py [--image F.jpg | --camd-dir DIR] [--source emeet|sony] [--direction forward|back|left|right|heading]
        [--plan-dx-mm 200 --plan-dy-mm 0] [--min-margin 250] [--config vision.json] [--max-age 2] [--json]

Looks at ONE frame, never holds the camera: it reads DIR/<source>_latest.jpg that
tools/vision/camd.py keeps (--max-age s: refuse a stale one), or --image. With neither it
takes a single frame from the camera itself, but only if nothing else is reading it (camd
lock / another reader = refuse, exit 4). The eMeet (1080p, high mount) is the default source:
the Sony is context video only.

Exit 0: margin in the requested direction >= --min-margin (or no direction asked).
Exit 3: margin below the threshold.  Exit 2: robot/board tags not found.
Exit 4: no frame (camera busy or stale).
Direction is relative to the BOARD (forward = toward the far end = +y, back = -y,
left/right as seen from the Sony) or `heading` = along the robot's own heading; the margin
is the distance from the robot (torso point, minus foot_margin_mm) to the edge reached
along that direction.

--plan-dx-mm / --plan-dy-mm: the net displacement the walk plans, in the ROBOT frame (dx forward, dy left; the
sum of the steps). The end point is the robot's position plus that, rotated by the tracker's heading into the board
frame (heading 0 = +y, counter-clockwise = toward -x; forward = (-sin h, cos h), left = (-cos h, -sin h)), and every
edge margin there must be >= --min-margin or the check exits 3 (a walk that ENDS too close to an edge is refused,
not only one that starts close). The predicted end margins are printed. Absolute position is only good to about
+-100 mm while the focal length comes from the tags' coplanarity (or the prior; README: accuracy): that
uncertainty is printed next to the margins and put in the JSON, it does not change the decision. The straight line
between start and end cannot come closer to an edge than the nearer of its two ends (margins are linear), but a
walk that wanders or steers is not covered: plan its extreme point, or the sum of the steps past the turn.

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


POSITION_UNCERTAINTY_MM = 100.0      # README: position and margins to about +-100 mm until the layout is measured and f pinned


def plan_end(cfg, x, y, heading_deg, dx_mm, dy_mm):
    """End point of a net displacement (dx forward, dy left, robot frame) from (x, y) at the tracker's heading
    -> (end_x, end_y, margins dict). Heading 0 = +y, counter-clockwise (left turn) = toward -x."""
    h = math.radians(heading_deg)
    fwd, left = (-math.sin(h), math.cos(h)), (-math.cos(h), -math.sin(h))
    ex, ey = x + dx_mm * fwd[0] + dy_mm * left[0], y + dx_mm * fwd[1] + dy_mm * left[1]
    return ex, ey, G.edge_margins(cfg, ex, ey)


def where(gray, cfg=None, direction=None, min_margin=None, tracker=None, plan_dx_mm=None, plan_dy_mm=None):
    """One frame -> dict(found, x_mm, y_mm, heading_deg, margins, margin_dir_mm, plan, ok, reason).
    plan_dx_mm / plan_dy_mm (either given): the planned net displacement, forward / left; adds `plan` with the end
    point and its margins, and with min_margin the end margins decide `ok`."""
    cfg = cfg or G.load_config()
    tr = tracker or Tracker(cfg, (gray.shape[1], gray.shape[0]))
    d = detect(gray, tr.det)
    cal = None
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
    if cal is not None:
        out["f_source"] = cal.get("f_source")
        out["f_bracket_px"] = cal.get("f_bracket_px")
    src = out.get("f_source") or ""
    out["position_uncertainty_mm"] = POSITION_UNCERTAINTY_MM if ("coplanarity" in src or src.startswith("prior")) else None
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
    if plan_dx_mm is not None or plan_dy_mm is not None:
        dxp, dyp = plan_dx_mm or 0.0, plan_dy_mm or 0.0
        ex, ey, em = plan_end(cfg, r["x_mm"], r["y_mm"], r["heading_deg"], dxp, dyp)
        edge = min(em, key=em.get)
        plan = dict(dx_mm=dxp, dy_mm=dyp, end_x_mm=ex, end_y_mm=ey, end_margins=em, end_margin_min_mm=em[edge],
                    end_margin_min_edge=edge, position_uncertainty_mm=out["position_uncertainty_mm"])
        out["plan"] = plan
        if min_margin is not None:
            plan["ok"] = bool(em[edge] >= min_margin)
            if not plan["ok"]:
                why = f"planned end point {em[edge]:.0f} mm from the {edge} edge < {min_margin:.0f}"
                out["reason"] = why if out.get("ok") is not False else f"{out['reason']}; {why}"
            out["ok"] = bool(plan["ok"] and out.get("ok") is not False)
    return out


def grab(camd_dir, max_age, source="emeet"):
    """Newest frame without disturbing the camera: camd's <source>_latest.jpg, else one direct
    frame if nobody else holds the camera."""
    if source == "emeet":
        return _grab_emeet(camd_dir, max_age)
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


def _grab_emeet(camd_dir, max_age):
    p = os.path.join(camd_dir, "emeet_latest.jpg")
    if os.path.exists(p):
        age = time.time() - os.path.getmtime(p)
        if age <= max_age:
            return cv2.imread(p, cv2.IMREAD_GRAYSCALE), f"camd emeet_latest.jpg ({age:.1f} s old)"
        return None, f"camd emeet_latest.jpg is {age:.0f} s old (daemon stopped or camera stalled)"
    lock = os.path.join(camd_dir, "emeet.lock")
    if os.path.exists(lock):
        import fcntl
        fd = os.open(lock, os.O_RDWR)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return None, "camd holds the eMeet but writes no latest.jpg"
        finally:
            os.close(fd)
    if subprocess.run(["fuser", "/dev/video0"], capture_output=True).returncode == 0:
        return None, "another process has /dev/video0 open"
    tmp = "/tmp/where_emeet.jpg"
    if os.path.exists(tmp):
        os.unlink(tmp)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "v4l2", "-input_format", "mjpeg", "-video_size",
                    "1920x1080", "-framerate", "15", "-i", "/dev/video0", "-frames:v", "8", "-update", "1", tmp],
                   capture_output=True, timeout=30)     # 8 frames: the first ones are dark while exposure settles
    g = cv2.imread(tmp, cv2.IMREAD_GRAYSCALE)
    return (g, "direct eMeet frame") if g is not None else (None, "no frame from the eMeet")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image")
    ap.add_argument("--camd-dir", default=DEFAULT_DIR)
    ap.add_argument("--source", choices=["emeet", "sony"], default="emeet", help="the eMeet is the measurement camera")
    ap.add_argument("--direction", choices=["forward", "back", "left", "right", "heading"])
    ap.add_argument("--min-margin", type=float, default=250.0)
    ap.add_argument("--plan-dx-mm", type=float, help="planned net displacement, forward (robot frame); with --plan-dy-mm")
    ap.add_argument("--plan-dy-mm", type=float, help="planned net displacement, left (robot frame)")
    ap.add_argument("--max-age", type=float, default=2.0)
    ap.add_argument("--config")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    if a.image:
        gray, src = cv2.imread(a.image, cv2.IMREAD_GRAYSCALE), a.image
    else:
        gray, src = grab(a.camd_dir, a.max_age, a.source)
    if gray is None:
        print(f"where: no frame: {src}", file=sys.stderr)
        return 4
    planned = a.plan_dx_mm is not None or a.plan_dy_mm is not None
    r = where(gray, G.load_config(a.config), a.direction, a.min_margin if (a.direction or planned) else None,
              plan_dx_mm=a.plan_dx_mm, plan_dy_mm=a.plan_dy_mm)
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
        if "plan" in r:
            p, em = r["plan"], r["plan"]["end_margins"]
            unc = p["position_uncertainty_mm"]
            print(f"plan dx {p['dx_mm']:+.0f} (forward) dy {p['dy_mm']:+.0f} (left) mm -> end x {p['end_x_mm']:.0f} y {p['end_y_mm']:.0f}; "
                  f"predicted end margins L {em['left']:.0f} R {em['right']:.0f} near {em['near']:.0f} far {em['far']:.0f} mm"
                  + (f" (+-{unc:.0f} mm: f from {r.get('f_source')})" if unc else "")
                  + (f" -> {'OK' if p['ok'] else 'END TOO CLOSE'} (min {p['end_margin_min_mm']:.0f} to the {p['end_margin_min_edge']} edge)"
                     if "ok" in p else ""))
    else:
        print(f"where: {r['reason']} (tags seen: {r['tags']})", file=sys.stderr)
    if not r["found"]:
        return 2
    return 0 if r.get("ok") in (True, None) else 3


if __name__ == "__main__":
    sys.exit(main())
