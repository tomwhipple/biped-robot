"""Kinematics helpers for the v6 (6 DOF/leg, ankle-roll) body.

Analytic leg IK in the PELVIS frame (pelvis level, hip yaw 0, sole parallel
to the pelvis), pose setters for a MuJoCo model built by gen_plant_v6, and
the support-polygon / centre-of-mass geometry that Gate A needs.

Sign conventions == sim/bimo_biped_v5body.xml:
  hip_roll   axis +X : positive swings the foot toward +Y (robot left)
  hip_pitch  axis +Y : positive swings the foot BACKWARD (flexion is negative)
  knee       axis -Y : negative = human flexion (shank swings backward)
  ankle      axis +Y : positive pitches the toe DOWN (plantarflex)
  ankle_roll axis +X
Standing straight is all zeros; the level-sole crouch family of the bench
notes (hip -t, knee -2t, ankle -t) falls out of leg_ik for the forward knee.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from gen_plant_v6 import DesignParams, build_xml

JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")


def load(p: DesignParams):
    m = mujoco.MjModel.from_xml_string(build_xml(p))
    return m, mujoco.MjData(m)


def hip_roll_point(p: DesignParams, side: str) -> np.ndarray:
    """hip roll axis point in the pelvis (torso) frame; the yaw axis sits
    hip_z above the torso origin (gen_plant_v6.hip_z, 2026-09-16; 0.0 =
    torso origin == yaw axis, unchanged)."""
    s = 1.0 if side == "L" else -1.0
    return np.array([0.0, s * p.hip_sep / 2, p.hip_z - p.d_yaw_roll])


def leg_ik(p: DesignParams, v: np.ndarray, knee: str | None = None,
           sole_pitch: float = 0.0) -> np.ndarray:
    """Joint angles (yaw, roll, pitch, knee, ankle, ankle_roll) in RADIANS that
    put the ankle-ROLL axis point at v (pelvis frame, measured from the hip
    roll axis point) with the sole parallel to the pelvis (or pitched by
    sole_pitch about +Y, positive = toe down). Raises ValueError if v is out
    of reach."""
    knee = knee or p.knee
    dx, dy, dz = float(v[0]), float(v[1]), float(v[2])
    if dz >= 0:
        raise ValueError("foot above hip")
    phi = math.atan2(dy, -dz)                 # hip roll
    rho = math.hypot(dy, dz)                  # in-plane drop below the hip roll axis
    # planar problem in the rolled leg plane: hip pitch point at (0, -d_rp),
    # ankle pitch point d_ankle above the ankle roll point (foot flat)
    px = dx
    pz = -rho + p.d_ankle + p.d_roll_pitch
    D = math.hypot(px, pz)
    L1, L2 = p.thigh, p.shank
    if D > L1 + L2 + 1e-9:
        raise ValueError(f"out of reach: D {D:.4f} > {L1 + L2:.4f}")
    if D < abs(L1 - L2) - 1e-9:
        raise ValueError("too close")
    alpha = math.atan2(px, -pz)               # hip->ankle line from -z, + forward
    gamma = math.acos(max(-1.0, min(1.0, (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))))
    D = min(D, L1 + L2)
    beta = math.pi - math.acos(max(-1.0, min(1.0, (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2))))
    if knee == "fwd":
        q_hip = -(alpha + gamma)
        q_knee = -beta
    elif knee == "bwd":
        q_hip = -(alpha - gamma)
        q_knee = +beta
    else:
        raise ValueError(knee)
    # foot rotation about +Y = hip + (-knee) + ankle  ->  = sole_pitch
    q_ankle = sole_pitch + q_knee - q_hip
    return np.array([0.0, phi, q_hip, q_knee, q_ankle, -phi])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose_from_feet(p: DesignParams, pelvis: np.ndarray, footL: np.ndarray,
                   footR: np.ndarray, knee: str | None = None,
                   yaw: tuple[float, float] = (0.0, 0.0),
                   sole_pitch: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """12 joint angles for a level pelvis at `pelvis` and the two ankle-roll
    points at footL / footR (all in the PELVIS frame). Foot points are the
    ankle roll axis points, i.e. roll_h above the sole bottom. yaw = each
    foot's yaw relative to the pelvis: it becomes the hip yaw joint, and the
    rest of the leg is solved in the frame rotated by it about the hip yaw
    axis, so the sole stays flat and parallel to the yawed foot frame."""
    q = np.zeros(12)
    pel = np.asarray(pelvis, float)
    for i, (side, f) in enumerate((("L", footL), ("R", footR))):
        s = 1.0 if side == "L" else -1.0
        hip_yaw_pt = pel + np.array([0.0, s * p.hip_sep / 2, p.hip_z])
        v = _rz(-yaw[i]) @ (np.asarray(f, float) - hip_yaw_pt)   # into the yawed leg frame
        v = v - np.array([0.0, 0.0, -p.d_yaw_roll])              # from the hip roll point
        q[6 * i:6 * i + 6] = leg_ik(p, v, knee, sole_pitch[i])
        q[6 * i] = yaw[i]
    return q


def pose_world(p: DesignParams, pelvis: np.ndarray, heading: float, footL: np.ndarray,
               footR: np.ndarray, yawL: float = 0.0, yawR: float = 0.0,
               knee: str | None = None) -> np.ndarray:
    """same, with everything in the WORLD frame: pelvis (x, y, z) and heading
    (yaw about +Z), feet as ankle-roll points with their own world yaws."""
    R = _rz(-heading)
    pel = np.asarray(pelvis, float)
    fL = R @ (np.asarray(footL, float) - pel)
    fR = R @ (np.asarray(footR, float) - pel)
    return pose_from_feet(p, np.zeros(3), fL, fR, knee, yaw=(yawL - heading, yawR - heading))


def heading_quat(heading: float):
    return (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))


def set_pose(m, d, q12, torso_pos=(0.0, 0.0, 1.0), torso_quat=(1.0, 0.0, 0.0, 0.0)):
    d.qpos[:] = 0.0
    d.qpos[0:3] = torso_pos
    d.qpos[3:7] = torso_quat
    d.qpos[7:7 + 12] = q12
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def foot_frame(m, d, side: str):
    b = m.body(f"{side}_foot").id
    return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()


def in_foot(m, d, side: str, pt_world: np.ndarray) -> np.ndarray:
    pos, R = foot_frame(m, d, side)
    return R.T @ (np.asarray(pt_world) - pos)


def sole_margin(p: DesignParams, xy, side: str = "L") -> float:
    """Signed distance (m, + inside) from a point in the FOOT frame (x, y) to
    the rounded-rectangle sole outline (centreline foot_y_off outboard)."""
    cx = p.foot_toe - p.foot_len / 2
    cy = p.foot_y_off if side == "L" else -p.foot_y_off
    hx, hy, r = p.foot_len / 2 - p.foot_r, p.foot_w / 2 - p.foot_r, p.foot_r
    qx = abs(xy[0] - cx) - hx
    qy = abs(xy[1] - cy) - hy
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return -(outside + inside - r)


def com_margin(m, d, p: DesignParams, stance: str):
    """(margin_m, com_xy_in_foot) for the whole-robot CoM against the stance
    sole outline, evaluated in the stance foot frame (so the stance sole may
    be anywhere in the world -- Gate A is a pure-kinematics question)."""
    com = d.subtree_com[0]
    c = in_foot(m, d, stance, com)
    return sole_margin(p, c[:2], stance), c


def pad_heights(m, d, p: DesignParams, stance: str, other: str) -> np.ndarray:
    """heights of the OTHER foot's pad bottoms above the stance sole plane."""
    pos, R = foot_frame(m, d, stance)
    hs = []
    for g in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if n.startswith(f"{other}_pad_"):
            q = R.T @ (d.geom_xpos[g] - pos)
            hs.append(q[2] - m.geom_size[g][0] + p.roll_h)   # sphere bottom vs stance sole
    return np.array(hs)


def torso_tilt_vs_foot(m, d, stance: str) -> float:
    pos, R = foot_frame(m, d, stance)
    Rt = d.xmat[m.body("torso").id].reshape(3, 3)
    up = R.T @ Rt[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))


def self_collides(m, d) -> int:
    """number of active contacts with the robot held in the air (only the
    enumerated inter-leg pairs can fire). Call after set_pose with a high
    torso z."""
    return int(d.ncon)


def joint_ok(m, q12, tol_deg: float = 0.0) -> bool:
    lo = m.jnt_range[1:13, 0]
    hi = m.jnt_range[1:13, 1]
    t = math.radians(tol_deg)
    return bool(np.all(q12 >= lo - t) and np.all(q12 <= hi + t))
