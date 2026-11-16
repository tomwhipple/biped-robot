"""Synthetic scenes for testing the tracker without a camera: tags rendered through a
pinhole camera (perspective warp, 3x supersampled, blur, noise)."""
import cv2
import numpy as np

import vgeom as G
from tagdefs import tag_image


def look_at(cam_pos, target):
    """(R_cb, t_cb): board -> camera (x right, y down, z forward), camera upright."""
    z = np.asarray(target, float) - np.asarray(cam_pos, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])          # rows: camera axes in board coords
    return R, -R @ np.asarray(cam_pos, float)


def render(tags, K, R_cb, t_cb, size=(1024, 680), blur=0.8, noise=3.0, seed=0, ss=3,
           floor=150, board_rect=None):
    """tags: list of (id, size_mm, R_bt, t_bt) with the tag pose in the board frame.
    board_rect=(W, L) paints the board brown. Returns uint8 grayscale."""
    w, h = size
    img = np.full((h * ss, w * ss), floor, np.uint8)
    Ks = K.copy()
    Ks[:2] *= ss
    Ks[2, 2] = 1

    def project(P):
        pc = (R_cb @ P.T).T + t_cb
        uv = (Ks @ pc.T).T
        return (uv[:, :2] / uv[:, 2:3]).astype(np.float32), pc[:, 2]
    if board_rect:
        Wb, Lb = board_rect
        uv, z = project(np.array([[0, 0, 0], [Wb, 0, 0], [Wb, Lb, 0], [0, Lb, 0.0]]))
        if (z > 0).all():
            cv2.fillConvexPoly(img, uv.astype(np.int32), 85)
    mod = 24
    for tid, sz, R, t in tags:
        tex = np.full((10 * mod, 10 * mod), 255, np.uint8)
        tex[mod:9 * mod, mod:9 * mod] = tag_image(tid, 8 * mod)
        q = sz / 8.0
        half = sz / 2.0 + q
        corners = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0.0]])
        uv, z = project(corners @ R.T + t)
        if (z <= 0).any():
            continue
        n_cam = R_cb @ R[:, 2]                  # tag normal in camera coords
        if n_cam @ ((R_cb @ np.asarray(t, float)) + t_cb) >= 0:
            continue                            # faces away from the camera
        M = cv2.getPerspectiveTransform(np.float32([[0, 0], [10 * mod, 0], [10 * mod, 10 * mod], [0, 10 * mod]]), uv)
        warped = cv2.warpPerspective(tex, M, (w * ss, h * ss), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full_like(tex, 255), M, (w * ss, h * ss), flags=cv2.INTER_NEAREST) > 0
        img[mask] = warped[mask]
    out = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32)
    if blur > 0:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    out += np.random.default_rng(seed).normal(0, noise, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def scene_tags(cfg, body_R, body_t, board_ids=(0, 1, 2, 3, 4, 5), robot_ids=(10, 11, 12)):
    """Board tags at their configured places + robot tags for a body pose (R, t in board frame)."""
    tags = []
    for i in board_ids:
        e = cfg["board"]["tags"][str(i)]
        R, t = G.board_tag_pose(e)
        tags.append((i, e["size"], R, t))
    for i in robot_ids:
        e = cfg["robot"]["tags"][str(i)]
        Rb, tb = G.robot_tag_pose(e)
        tags.append((i, e["size"], body_R @ Rb, body_R @ tb + np.asarray(body_t, float)))
    return tags
