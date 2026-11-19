#!/usr/bin/env python3
"""Check the camera setup from ONE frame: camera pose, focal length, per-tag reprojection error.

    tools/vision/calibrate.py FRAME.jpg [--config vision.json] [--json]
    tools/vision/calibrate.py --camd-dir ~/cam-segments --source emeet        # newest camd frame

Run it after every re-aim. With a MEASURED config (board.tags from a tape measure) the board
pose is fitted to all board-tag corners and every board tag gets its reprojection error (a tag
that is far above ~1 px is mislocated in the config, or not flat, or a decode error). Without
measurements (the default "auto_end" layout) it uses the two most widely separated board tags
and says what it assumed; see tag_track.Tracker.auto_end_layout.

Prints: layout and focal length (and where f came from), camera position in the board frame
(compare with a tape measure: the images cannot give the board's absolute scale), pitch down,
every tag seen (size in px, reprojection RMS), and the robot pose if its tags are visible.
Exit 0 = calibrated, 2 = fewer than two board tags decoded.
"""
import argparse
import json
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vgeom as G  # noqa: E402
from camd import DEFAULT_DIR  # noqa: E402
from tag_track import Tracker  # noqa: E402
from tagdefs import detect  # noqa: E402


def calibrate_frame(gray, cfg=None):
    cfg = cfg or G.load_config()
    tr = Tracker(cfg, (gray.shape[1], gray.shape[0]))
    d = detect(gray, tr.det)
    cal = tr.calibrate([d])
    out = dict(image_size=[gray.shape[1], gray.shape[0]], tag_sizes_px={str(i): round(G.mean_edge_px(c), 1) for i, c in sorted(d.items())})
    if cal is None:
        out["error"] = "fewer than two board tags decoded"
        return out, tr, d
    out.update(cal)
    r = tr.process(gray, None, d)
    if r.get("x_mm") is not None:
        out["robot"] = {k: round(r[k], 1) for k in ("x_mm", "y_mm", "z_mm", "heading_deg", "pitch_deg", "roll_deg")}
        out["robot"]["used_tags"] = r["used_tags"]
        out["robot"]["ambiguous"] = r.get("ambiguous")
        out["robot_tag_rms_px"] = tr.robot_tag_residuals(r, d)
    return out, tr, d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", nargs="?")
    ap.add_argument("--config")
    ap.add_argument("--camd-dir", default=DEFAULT_DIR)
    ap.add_argument("--source", default="emeet")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    path = a.frame or os.path.join(a.camd_dir, f"{a.source}_latest.jpg")
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(f"calibrate: cannot read {path}", file=sys.stderr)
        return 4
    out, _, _ = calibrate_frame(gray, G.load_config(a.config))
    if a.json:
        print(json.dumps(out, indent=1, default=float))
    else:
        print(f"frame {path}  {out['image_size'][0]}x{out['image_size'][1]}  tags seen (px): {out['tag_sizes_px']}")
        if "error" in out:
            print("calibrate:", out["error"], file=sys.stderr)
        else:
            print(f"layout: {out['layout']}")
            print(f"f = {out['f_px']:.0f} px  ({out['f_source']})" + (f", bracket {out['f_bracket_px']}" if out.get('f_bracket_px') else ""))
            print(f"camera in the board frame: {out['camera_position_board_mm']} mm" + (f", {out['camera_pitch_down_deg']} deg down" if 'camera_pitch_down_deg' in out else ""))
            pt = out.get("per_tag_rms_px") or out.get("per_tag_pnp_rms_px")
            print(f"board tags reprojection (px): {pt}")
            for pair, v in (out.get("layout_fit") or {}).items():
                print(f"layout fit {pair}: layout {v['layout_mm']} mm vs measured {v['measured_mm']} mm "
                      f"(ratio {v['ratio']}, plane normals differ {v['plane_normal_disagreement_deg']} deg)")
            for k in ("tag_spacing_mm_measured", "assumed_side_inset_mm", "assumed_end_offset_mm", "board_rms_px_median", "accuracy"):
                if k in out:
                    print(f"{k}: {out[k]}")
            if "robot" in out:
                print(f"robot: {out['robot']}  tag reprojection (px): {out['robot_tag_rms_px']}")
                if any(v["rms_px"] > 3 for v in out["robot_tag_rms_px"].values()):
                    print("WARNING: a robot tag reprojects > 3 px: the tag mount offsets in the config are "
                          "probably still nominal; measure them (README, What to measure)")
    return 2 if "error" in out else 0


if __name__ == "__main__":
    sys.exit(main())
