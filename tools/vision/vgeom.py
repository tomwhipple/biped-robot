"""Geometry for the tag tracker: config, board/robot frames, camera model, pose maths.

Frames (right-handed, mm, degrees at the interface):
  board  origin = a named board corner (default near-left as seen from the Sony),
         x to the right, y along the board AWAY from the Sony, z up.
  body   robot torso reference point; x forward, y left, z up.
  tag    x right, y up (the printed arrow), z = outward normal toward the viewer;
         corners in detector order TL, TR, BR, BL.
Heading: body x projected on the board plane; 0 = along +y (to the far end),
positive counter-clockwise seen from above (toward board -x).
Pitch > 0 = leaning forward; roll > 0 = leaning to the robot's right (ZYX, Rz Ry Rx).
"""
import copy
import json

import cv2
import numpy as np

# Everything physical is a CONFIG value; these are nominal numbers to be replaced
# by a tape measure (see tools/vision/README.md "What to measure").
DEFAULT_CONFIG = {
    "image_size": [1920, 1080],
    # f_px None: estimate it from the board tags when the layout is measured, else use f_prior_px
    # (eMeet C960 at 1080p, assumed ~90 deg diagonal field of view = 1100 px; NOT measured)
    "camera": {"f_px": None, "f_prior_px": 1100.0, "cx": None, "cy": None},
    "board": {
        # "auto_end": no measured layout yet -- see tag_track.auto_end_layout (approximate);
        # becomes "config" as soon as a config file gives board.tags (measured positions)
        "layout": "auto_end", "assumed_tag_spacing_mm": None,    # None = the board width
        "width_mm": 900.0, "length_mm": 1200.0, "origin": "near-left",
        # tag centres in the board frame (mm), up-arrow along +y, black-square size
        "tags": {
            "0": {"center": [60.0, 1140.0], "size": 80.0},
            "1": {"center": [840.0, 1140.0], "size": 80.0},
            "2": {"center": [60.0, 60.0], "size": 80.0},
            "3": {"center": [840.0, 60.0], "size": 80.0},
            "4": {"center": [60.0, 600.0], "size": 80.0},
            "5": {"center": [840.0, 600.0], "size": 80.0},
        },
    },
    "robot": {
        # torso tags: centre in the body frame (mm), outward normal, up direction, size
        # as placed 2026-11-16: back (40 mm spare) and top (60 mm); there is no front tag 10 now
        # (add "10": {"center": [30,0,0], "normal": [1,0,0], "up": [0,0,1], "size": 60} to use one)
        "tags": {
            "11": {"center": [-30.0, 0.0, 0.0], "normal": [-1, 0, 0], "up": [0, 0, 1], "size": 40.0},
            "12": {"center": [0.0, 0.0, 45.0], "normal": [0, 0, 1], "up": [1, 0, 0], "size": 60.0},
        },
        # foot tags are reported separately (feet move relative to the torso)
        "foot_tags": {"13": {"size": 25.0, "side": "L"}, "14": {"size": 25.0, "side": "R"}},
    },
    "foot_margin_mm": 0.0,       # subtracted from edge distances (half foot footprint, if wanted)
    "roll_sign": 1.0,            # flip to match the firmware's roll convention
    "min_tag_px": 12.0,          # shorter mean edge than this is ignored
    "max_tag_rms_px": 4.0,       # pose fit worse than this is ignored (motion blur, bad corners)
    "hold_s": 0.5,               # a pose older than this is flagged stale
    "lost_s": 3.0,               # ... and dropped (null) after this
}


def load_config(path=None):
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if path:
        with open(path) as f:
            user = json.load(f)
        _merge(cfg, user)
        if user.get("board", {}).get("tags") and "layout" not in user["board"]:
            cfg["board"]["layout"] = "config"       # measured positions given: use them
    return cfg


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict) and k not in ("tags", "foot_tags"):
            _merge(dst[k], v)
        else:
            dst[k] = v


def tag_corners_local(size):
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def frame_from_normal_up(normal, up):
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    u = np.asarray(up, float)
    u = u - n * (u @ n)
    u /= np.linalg.norm(u)
    r = np.cross(u, n)
    return np.stack([r, u, n], axis=1)       # columns: tag x, y, z in the parent frame


def board_tag_pose(entry):
    """(R, t) of a board tag in the board frame: lies flat, arrow along +y unless the entry has
    "yaw_deg" (counter-clockwise from +y seen from above)."""
    c = entry["center"]
    y = np.radians(entry.get("yaw_deg", 0.0))
    return frame_from_normal_up([0, 0, 1], [-np.sin(y), np.cos(y), 0]), np.array([c[0], c[1], 0.0])


def robot_tag_pose(entry):
    return frame_from_normal_up(entry["normal"], entry["up"]), np.array(entry["center"], float)


def board_object_points(cfg, ids):
    """Corner points (board frame) of the given board tag ids -> {id: 4x3}."""
    out = {}
    for i in ids:
        e = cfg["board"]["tags"].get(str(i))
        if e is None:
            continue
        R, t = board_tag_pose(e)
        out[i] = tag_corners_local(e["size"]) @ R.T + t
    return out


def make_K(f, size, cx=None, cy=None):
    w, h = size
    return np.array([[f, 0, w / 2.0 if cx is None else cx],
                     [0, f, h / 2.0 if cy is None else cy], [0, 0, 1.0]])


def mean_edge_px(c):
    return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


def reproj_rms(obj, img, rvec, tvec, K):
    p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, None)
    return float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))


def solve_board_pose(obj, img, K):
    """Planar PnP of coplanar (z = 0) points -> (rvec, tvec, rms)."""
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    return rvec, tvec, reproj_rms(obj, img, rvec, tvec, K)


def stack_board(cfg, dets):
    ids = [i for i in dets if str(i) in cfg["board"]["tags"]]
    if not ids:
        return None, None, []
    pts = board_object_points(cfg, ids)
    obj = np.concatenate([pts[i] for i in ids])
    img = np.concatenate([dets[i] for i in ids])
    return obj, img, ids


def estimate_focal(cfg, frames_dets, size, grid=None):
    """f (px) minimising the board-tag reprojection error, square pixels, centre at the image
    centre. frames_dets: list of detection dicts. -> dict(f, rms, f_lo, f_hi, per_frame)"""
    grid = grid if grid is not None else np.geomspace(300, 4000, 120)
    cx, cy = (cfg["camera"].get("cx"), cfg["camera"].get("cy"))
    cost = np.zeros(len(grid))
    n = 0
    per = []
    for d in frames_dets:
        obj, img, ids = stack_board(cfg, d)
        if obj is None or len(ids) < 2:
            continue
        c = []
        for f in grid:
            r = solve_board_pose(obj, img, make_K(f, size, cx, cy))
            c.append(r[2] if r else 1e3)
        c = np.array(c)
        per.append(float(grid[int(np.argmin(c))]))
        cost += c ** 2
        n += 1
    if n == 0:
        return None
    cost = np.sqrt(cost / n)
    k = int(np.argmin(cost))
    # refine around the minimum with a finer grid
    fine = np.linspace(grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)], 41)
    fc = np.zeros(len(fine))
    for d in frames_dets:
        obj, img, ids = stack_board(cfg, d)
        if obj is None or len(ids) < 2:
            continue
        for j, f in enumerate(fine):
            r = solve_board_pose(obj, img, make_K(f, size, cx, cy))
            fc[j] += (r[2] if r else 1e3) ** 2
    fc = np.sqrt(fc / n)
    j = int(np.argmin(fc))
    ok = fine[fc <= fc[j] * 1.1 + 0.05]
    return dict(f=float(fine[j]), rms=float(fc[j]), f_lo=float(ok.min()), f_hi=float(ok.max()),
                frames=n, per_frame_median=float(np.median(per)),
                per_frame_spread=[float(np.percentile(per, 10)), float(np.percentile(per, 90))])


def rot_mean(Rs, w=None):
    w = np.ones(len(Rs)) if w is None else np.asarray(w, float)
    M = sum(wi * R for wi, R in zip(w, Rs))
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


# board frame -> "walking frame" (x' along +y, y' toward -x, z up) for heading/roll/pitch
_M = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])


def euler_from_board_R(R_board_body):
    """-> heading, pitch, roll in degrees (see module doc)."""
    R = _M @ R_board_body
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)


def board_R_from_euler(heading, pitch, roll):
    h, p, r = np.radians([heading, pitch, roll])
    Rz = np.array([[np.cos(h), -np.sin(h), 0], [np.sin(h), np.cos(h), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return _M.T @ (Rz @ Ry @ Rx)


def edge_margins(cfg, x, y):
    W, L = cfg["board"]["width_mm"], cfg["board"]["length_mm"]
    m = cfg.get("foot_margin_mm", 0.0)
    return {"left": x - m, "right": W - x - m, "near": y - m, "far": L - y - m}
