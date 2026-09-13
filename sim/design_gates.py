"""Design-stage gates A and B for the v6 body (docs/design-stage-simulation-gates.md).

Gate A -- kinematic capability map, no dynamics, no policy:
  A1  lateral weight shift with both soles flat (hip roll / ankle roll
      parallelogram): the pelvis offset y* at which the whole-robot CoM sits on
      the stance sole's centreline, the hip roll it takes, the CoM margin
      inside the stance sole there, and the double-support margin all the way.
  A2  at y*, lift the other foot straight up and then swing it forward: CoM
      margin vs the stance sole, swing clearance, joint limits, self-collision.
  A3  grid search for the best single-foot stance margin the body can reach.
  A4  split stance (feet 6 cm apart fore-aft, both flat): CoM inside the hull.
  A5  level-sole crouch depth inside the joint limits.
Gate B -- actuator envelope: one step (shift, arc swing, shift back) is run
  forward in MuJoCo under the walker_env servo model (PD clamped to the
  measured STS3215 torque-speed line; STS3250 from its datasheet derated the
  same way) at the design cadence and at 2x. Per joint: fraction of substeps
  ON the clamp, peak tracking error, p99/peak torque, peak speed; plus the
  swing foot's achieved clearance. Pass = no joint saturated > 5 %, error
  <= 5 deg, clearance >= 15 mm, body up -- at 2x cadence for the 2x margin.

    .venv/bin/python sim/design_gates.py                     # default design
    .venv/bin/python sim/design_gates.py --knee bwd
    .venv/bin/python sim/design_gates.py --sweep             # hip_sep x foot_w x leg x knee
Results and the design record: docs/design-v6-ankle-roll.md, docs/design-v6/.
    .venv/bin/python sim/design_gates.py --set hip_sep=0.070 --json out.json
"""
from __future__ import annotations

import argparse
import dataclasses
import itertools
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gen_plant_v6 import DesignParams, params_from_args  # noqa: E402
import v6_kin as K  # noqa: E402

# STS3215 measured envelope (walker_env.py: 2.94 N*m, 4.04 rad/s at 12 V basis)
# scaled to the 3S pack the robot runs (11.1 V). STS3250: datasheet 50 kg*cm,
# 0.133 s/60 deg (7.87 rad/s) at 12 V, derated by the same 0.86 no-load
# factor the 3215 measured, and by 11.1/12.
V = 11.1 / 12.0
# kp_scale: static stiffness relative to the STS3215's fitted 12 N*m/rad.
# Third-party bench (robonine.com evaluations): STS3215 ~2.6 deg under
# 0.98 N*m (~22 N*m/rad); STS3250 14 counts (1.2 deg) under 1.96 N*m
# (~91 N*m/rad) -- ~4x. To be re-measured on OUR bench before it is relied on.
SERVOS = {
    "sts3215": dict(stall=2.94 * V, w0=4.04 * V, kp_scale=1.0),
    "sts3250": dict(stall=4.90 * V, w0=7.87 * 0.86 * V, kp_scale=4.0),
}
JN = [f"{s}_{j}" for s in "LR" for j in K.JOINTS]


# --------------------------------------------------------------------------- task-space timeline
@dataclasses.dataclass
class Key:
    pelvis: np.ndarray
    footL: np.ndarray
    footR: np.ndarray
    heading: float = 0.0     # pelvis yaw in the world (rad)
    yawL: float = 0.0        # foot yaws in the world (rad)
    yawR: float = 0.0

    def copy(self):
        return Key(self.pelvis.copy(), self.footL.copy(), self.footR.copy(),
                   self.heading, self.yawL, self.yawR)


def minjerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class Timeline:
    """piecewise task-space motion: segments (key_from -> key_to, duration)."""

    def __init__(self, start: Key):
        self.keys = [start.copy()]
        self.durs = []
        self.arcs = []

    def to(self, key: Key, dur: float, arc: float = 0.0, arc_foot: str = "R", arc_out: float = 0.0):
        """arc > 0: the named foot rises by `arc` on a sine bump during the
        segment (lift-swing-lower in ONE motion, no dwell at the top);
        arc_out: the same bump sideways, OUTWARD (away from the other foot)."""
        self.keys.append(key.copy())
        self.durs.append(dur)
        self.arcs.append((arc, arc_foot, arc_out))
        return self

    def hold(self, dur: float):
        return self.to(self.keys[-1], dur)

    @property
    def T(self):
        return float(sum(self.durs))

    def at(self, t: float) -> Key:
        if t <= 0:
            return self.keys[0]
        acc = 0.0
        for k, dur in enumerate(self.durs):
            if t <= acc + dur:
                s = minjerk((t - acc) / dur) if dur > 0 else 1.0
                a, b = self.keys[k], self.keys[k + 1]
                out = Key(a.pelvis + (b.pelvis - a.pelvis) * s,
                          a.footL + (b.footL - a.footL) * s,
                          a.footR + (b.footR - a.footR) * s,
                          a.heading + (b.heading - a.heading) * s,
                          a.yawL + (b.yawL - a.yawL) * s,
                          a.yawR + (b.yawR - a.yawR) * s)
                arc, foot, arc_out = self.arcs[k]
                if arc > 0 or arc_out > 0:
                    bump = math.sin(math.pi * s)
                    # sideways bump in the swing FOOT's yawed frame (outward =
                    # +y for the left foot, -y for the right)
                    if foot == "R":
                        yaw = out.yawR
                        out.footR[2] += arc * bump
                        out.footR[0] += arc_out * bump * math.sin(yaw)
                        out.footR[1] -= arc_out * bump * math.cos(yaw)
                    else:
                        yaw = out.yawL
                        out.footL[2] += arc * bump
                        out.footL[0] -= arc_out * bump * math.sin(yaw)
                        out.footL[1] += arc_out * bump * math.cos(yaw)
                return out
            acc += dur
        return self.keys[-1]


def standing_key(p: DesignParams, drop: float) -> Key:
    """standing with the pelvis `drop` below the straight-leg height, feet
    under the hips, sole bottoms on z = 0 (ankle roll points at z = roll_h)."""
    z_pel = p.z_yaw_above_sole - drop
    return Key(np.array([0.0, 0.0, z_pel]),
               np.array([0.0, +p.hip_sep / 2, p.roll_h]),
               np.array([0.0, -p.hip_sep / 2, p.roll_h]))


def q_of(p: DesignParams, key: Key) -> np.ndarray:
    if key.heading == 0.0 and key.yawL == 0.0 and key.yawR == 0.0:
        return K.pose_from_feet(p, key.pelvis, key.footL, key.footR)
    return K.pose_world(p, key.pelvis, key.heading, key.footL, key.footR, key.yawL, key.yawR)


# --------------------------------------------------------------------------- hull helpers (from toein_hyp)
def hull(pts):
    pts = sorted(set(map(tuple, pts)))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo = []
    for q in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    up = []
    for q in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 0:
            up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]


def hull_margin(h, c):
    n = len(h)
    best = 1e9
    for i in range(n):
        a = np.array(h[i])
        b = np.array(h[(i + 1) % n])
        e = b - a
        nrm = np.array([e[1], -e[0]])
        nrm /= np.linalg.norm(nrm) + 1e-12
        best = min(best, -float(np.dot(c - a, nrm)))
    return best


def pad_xy(m, d, side):
    out = []
    for g in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if n.startswith(f"{side}_pad_"):
            out.append(d.geom_xpos[g][:2].copy())
    return out


def double_support_margin(m, d):
    h = hull(pad_xy(m, d, "L") + pad_xy(m, d, "R"))
    return hull_margin(h, d.subtree_com[0][:2])


# --------------------------------------------------------------------------- Gate A
def eval_pose(m, d, p, key: Key, stance="L", swing="R"):
    """set the pose in the air (no floor contact) and measure everything."""
    try:
        q = q_of(p, key)
    except ValueError as e:
        return dict(ok=False, why=str(e))
    K.set_pose(m, d, q, torso_pos=key.pelvis + np.array([0, 0, 1.0]))
    mg, c = K.com_margin(m, d, p, stance)
    hs = K.pad_heights(m, d, p, stance, swing)
    return dict(ok=True, q=q, margin=mg, com_foot=c, clear=float(hs.min()),
                clear_max=float(hs.max()), jok=K.joint_ok(m, q), ncon=int(d.ncon),
                hip_roll=math.degrees(q[1]), ankle_roll=math.degrees(q[5]))


def find_shift(m, d, p, drop, lift_key_fn=None, target_y=0.0):
    """pelvis lateral offset that puts the CoM at target_y in the L-foot frame
    (secant on the kinematics). lift_key_fn(key) may raise the swing foot
    first so the shift is solved for the lifted configuration."""
    def com_y(yp):
        k = standing_key(p, drop)
        k.pelvis[1] = yp
        if lift_key_fn:
            k = lift_key_fn(k)
        r = eval_pose(m, d, p, k)
        if not r["ok"]:
            return None
        return r["com_foot"][1] - target_y
    y0, y1 = 0.0, p.hip_sep / 2
    f0, f1 = com_y(y0), com_y(y1)
    for _ in range(30):
        if f0 is None or f1 is None or abs(f1 - f0) < 1e-9:
            break
        y2 = y1 - f1 * (y1 - y0) / (f1 - f0)
        y0, f0, y1, f1 = y1, f1, y2, com_y(y2)
        if f1 is not None and abs(f1) < 1e-5:
            break
    return y1


def gate_a(p: DesignParams, drop=0.02, lift_h=0.03, step=0.06, verbose=True):
    m, d = K.load(p)
    out = dict(design=p.summary(), mass=float(m.body_subtreemass[0]))
    K.set_pose(m, d, np.zeros(12), torso_pos=(0, 0, p.z_yaw_above_sole))
    out["com_z_stand"] = float(d.subtree_com[0][2])
    # A1: shift sweep
    rows = []
    for yp in np.arange(0.0, p.hip_sep / 2 + 0.031, 0.002):
        k = standing_key(p, drop)
        k.pelvis[1] = yp
        r = eval_pose(m, d, p, k)
        if not r["ok"]:
            break
        K.set_pose(m, d, r["q"], torso_pos=k.pelvis)   # on the floor for the hull
        ds = double_support_margin(m, d)
        rows.append((yp, r["com_foot"][1], r["margin"], ds, r["hip_roll"], r["jok"], r["ncon"]))
    y_star = find_shift(m, d, p, drop)
    k = standing_key(p, drop)
    k.pelvis[1] = y_star
    r = eval_pose(m, d, p, k)
    out["A1"] = dict(y_star=y_star, hip_roll_deg=r["hip_roll"], ankle_roll_deg=r["ankle_roll"],
                     margin=r["margin"], jok=r["jok"], ncon=r["ncon"],
                     ds_margin_min=min(x[3] for x in rows if x[0] <= y_star + 1e-9),
                     com_fore_aft=float(r["com_foot"][0]))
    # A2: lift at y*, then swing forward -- and re-solve the shift for the
    # lifted configuration (the swing leg's mass moves)
    def lifted(kk, h=lift_h, dx=0.0):
        kk = kk.copy()
        kk.footR = kk.footR + np.array([dx, 0.0, h])
        return kk
    y_lift = find_shift(m, d, p, drop, lambda kk: lifted(kk, lift_h, step / 2))
    a2 = {}
    for name, h, dx in (("lift", lift_h, 0.0), ("mid_swing", lift_h, step / 2),
                        ("fore", lift_h, step), ("lift_5cm", 0.05, 0.0)):
        k = standing_key(p, drop)
        k.pelvis[1] = y_lift
        r = eval_pose(m, d, p, lifted(k, h, dx))
        a2[name] = (dict(margin=r["margin"], clear=r["clear"], jok=r["jok"], ncon=r["ncon"],
                         com_foot=[float(x) for x in r["com_foot"][:2]],
                         hip_roll=r["hip_roll"], q_deg=[round(math.degrees(x), 1) for x in r["q"]])
                    if r["ok"] else dict(ok=False, why=r["why"]))
    out["A2"] = dict(y_lift=y_lift, poses=a2)
    # A3: best single-foot margin, grid
    best = None
    for yp, dz, h, dy, dx in itertools.product(
            np.arange(p.hip_sep / 2 - 0.02, p.hip_sep / 2 + 0.035, 0.0025),
            (0.01, 0.02, 0.03, 0.04), (0.02, 0.03, 0.04), (-0.02, -0.01, 0.0, 0.01, 0.02),
            (-0.03, 0.0, 0.03, 0.06)):
        k = standing_key(p, dz)
        k.pelvis[1] = yp
        k.footR = k.footR + np.array([dx, dy, h])
        r = eval_pose(m, d, p, k)
        if not r["ok"] or not r["jok"] or r["ncon"] or r["clear"] < 0.02 - 1e-6:
            continue
        if best is None or r["margin"] > best["margin"]:
            best = dict(margin=r["margin"], yp=yp, drop=dz, h=h, dy=dy, dx=dx,
                        hip_roll=r["hip_roll"], clear=r["clear"])
    out["A3"] = best
    # A4: split stance
    k = standing_key(p, drop)
    k.footL[0] += 0.03
    k.footR[0] -= 0.03
    r = eval_pose(m, d, p, k)
    K.set_pose(m, d, r["q"], torso_pos=k.pelvis)
    out["A4"] = dict(ds_margin=double_support_margin(m, d), jok=r["jok"], ncon=r["ncon"])
    # A5: crouch depth (level sole, feet under hips)
    depth = 0.0
    for dz in np.arange(0.0, 0.25, 0.0025):
        k = standing_key(p, dz)
        r = eval_pose(m, d, p, k)
        if not r["ok"] or not r["jok"]:
            break
        K.set_pose(m, d, r["q"], torso_pos=k.pelvis)
        if double_support_margin(m, d) < 0.005:
            break
        depth = dz
    out["A5"] = dict(crouch_depth=depth)
    # verdict
    a2m = a2["mid_swing"].get("margin", -1)
    out["A_pass"] = bool(a2m >= 0.01 and a2["mid_swing"].get("jok") and not a2["mid_swing"].get("ncon")
                         and out["A1"]["ds_margin_min"] > 0.005)
    if verbose:
        print(f"== Gate A  {p.summary()}  mass {out['mass']:.3f} kg  CoM z {out['com_z_stand']:.3f}")
        print(f"  A1 shift (both soles flat): pelvis y* {1e3*y_star:5.1f} mm  hip roll {out['A1']['hip_roll_deg']:5.1f} deg  "
              f"ankle roll {out['A1']['ankle_roll_deg']:5.1f}  CoM margin in L sole {1e3*out['A1']['margin']:5.1f} mm  "
              f"double-support margin along the path >= {1e3*out['A1']['ds_margin_min']:.1f} mm  "
              f"limits {'ok' if out['A1']['jok'] else 'VIOLATED'}  self-contacts {out['A1']['ncon']}")
        print(f"  A2 lifted (pelvis y {1e3*y_lift:.1f} mm):")
        for n, v in a2.items():
            if v.get("ok") is False:
                print(f"     {n:10s} unreachable: {v['why']}")
            else:
                print(f"     {n:10s} margin {1e3*v['margin']:5.1f} mm  clearance {1e3*v['clear']:4.1f} mm  "
                      f"hip roll {v['hip_roll']:5.1f}  limits {'ok' if v['jok'] else 'VIOLATED'}  contacts {v['ncon']}  q {v['q_deg']}")
        if best:
            print(f"  A3 best one-foot margin {1e3*best['margin']:.1f} mm at pelvis y {1e3*best['yp']:.1f} mm, drop "
                  f"{1e3*best['drop']:.0f} mm, swing h {1e3*best['h']:.0f} dy {1e3*best['dy']:+.0f} dx {1e3*best['dx']:+.0f} (hip roll {best['hip_roll']:.1f})")
        print(f"  A4 split stance 6 cm: hull margin {1e3*out['A4']['ds_margin']:.1f} mm  limits {'ok' if out['A4']['jok'] else 'VIOLATED'}")
        print(f"  A5 level-sole crouch depth {1e3*depth:.0f} mm")
        print(f"  => Gate A {'PASS' if out['A_pass'] else 'FAIL'}")
    return out


# --------------------------------------------------------------------------- Gate B
def step_timeline(p: DesignParams, y_lift: float, drop=0.02, lift_h=0.03, step=0.06,
                  t_shift=0.8, t_swing=1.2, t_settle=0.2, scale=1.0) -> Timeline:
    """one step with the RIGHT foot: shift onto L, swing R forward on a sine
    arc of height lift_h (lift + advance + lower in one motion), shift back to
    centre. Segment durations x scale (0.5 = twice the cadence)."""
    k0 = standing_key(p, drop)
    tl = Timeline(k0).hold(0.5)
    k1 = k0.copy(); k1.pelvis[1] = y_lift
    tl.to(k1, t_shift * scale).hold(t_settle * scale)
    k2 = k1.copy(); k2.footR[0] += step
    tl.to(k2, t_swing * scale, arc=lift_h, arc_foot="R").hold(t_settle * scale)
    k3 = k2.copy(); k3.pelvis[1] = 0.0; k3.pelvis[0] = step / 2
    tl.to(k3, t_shift * scale).hold(0.5)
    t0 = 0.5 + (t_shift + t_settle) * scale
    tl.swing_window = (t0, t0 + t_swing * scale)
    return tl


def run_servo(p: DesignParams, tl: Timeline, servo="sts3215", per_joint=None, kp=25.0, kd=0.5,
              ctrl_hz=50.0, model_data=None):
    """forward sim under the walker_env servo model WITHOUT lag/dead time: PD on
    the commanded angle, torque clamped every substep to the linear torque-
    speed envelope of the named servo (per_joint: {joint_name: servo} overrides).
    Returns per-joint saturation fraction, peak tracking error, p99 |tau|, peak
    |qd|, plus the swing foot's achieved clearance and whether the body stayed up."""
    m, d = model_data or K.load(p)
    m.actuator_gainprm[:] = 0.0
    m.actuator_biasprm[:] = 0.0
    m.dof_damping[6:18] = 0.1               # servo_joint_damping (walker_env)
    stall = np.array([SERVOS[(per_joint or {}).get(n, servo)]["stall"] for n in JN])
    w0 = np.array([SERVOS[(per_joint or {}).get(n, servo)]["w0"] for n in JN])
    k0 = tl.at(0.0)
    q0 = q_of(p, k0)
    K.set_pose(m, d, q0, torso_pos=k0.pelvis + np.array([0, 0, 0.001]))
    dt = m.opt.timestep
    n_sub = int(round(1.0 / ctrl_hz / dt))
    sat = np.zeros(12); n = 0
    err_pk = np.zeros(12); tau_all = []; qd_pk = np.zeros(12)
    fell = False; tilt_max = 0.0; clear_max = 0.0; t_air = 0.0; roll_err_pk = 0.0
    swing_pads = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("R_pad_")]
    t = 0.0; cmd = q0.copy()
    while t < tl.T:
        try:
            cmd = q_of(p, tl.at(t))
        except ValueError:
            pass
        for _ in range(n_sub):
            q = d.qpos[7:19]; qd = d.qvel[6:18]
            cap = stall * np.clip(1.0 - np.abs(qd) / w0, 0.0, 1.0)
            raw = kp * (cmd - q) - kd * qd
            tau = np.clip(raw, -cap, cap)
            sat += (np.abs(raw) > cap + 1e-9); n += 1
            d.qfrc_applied[6:18] = tau
            mujoco.mj_step(m, d)
            tau_all.append(tau.copy())
            err_pk = np.maximum(err_pk, np.abs(cmd - d.qpos[7:19]))
            qd_pk = np.maximum(qd_pk, np.abs(d.qvel[6:18]))
        t += 1.0 / ctrl_hz
        up = d.xmat[m.body("torso").id].reshape(3, 3)[2, 2]
        tilt_max = max(tilt_max, math.degrees(math.acos(max(-1, min(1, up)))))
        h = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in swing_pads)   # lowest swing pad
        clear_max = max(clear_max, h)
        if h > 0.005:
            t_air += 1.0 / ctrl_hz
        if up < 0.7:
            fell = True
            break
    tau_all = np.array(tau_all)
    return dict(sat=sat / max(n, 1), err_pk_deg=np.degrees(err_pk), tau_p99=np.percentile(np.abs(tau_all), 99, axis=0),
                tau_pk=np.abs(tau_all).max(0), qd_pk=qd_pk, fell=fell, tilt_max=tilt_max,
                clear_max=clear_max, t_air=t_air,
                speed_margin=w0 / np.maximum(qd_pk, 1e-6), torque_margin=stall / np.maximum(np.abs(tau_all).max(0), 1e-6),
                final_pelvis_x=float(d.qpos[0]), final_pelvis_y=float(d.qpos[1]))


def gate_b(p: DesignParams, y_lift: float, verbose=True, scale=1.0, servos=("sts3215", "sts3250"),
           speed_req=2.0, torque_req=1.5):
    """one step at the design cadence under each servo's envelope. Per joint:
    speed margin = servo no-load speed / peak demanded speed, torque margin =
    stall / peak torque (both at 11.1 V), saturation fraction, tracking error.
    Gate pass (docs/design-stage-simulation-gates.md Gate B): speed margin
    >= 2 and torque margin >= 1.5 on every joint, no saturation, body up,
    swing foot clearly off the ground."""
    out = {}
    for sv in servos:
        tl = step_timeline(p, y_lift, scale=scale)
        r = run_servo(p, tl, servo=sv)
        out[sv] = dict(sat=r["sat"].tolist(), err_pk_deg=r["err_pk_deg"].tolist(), tau_p99=r["tau_p99"].tolist(),
                       tau_pk=r["tau_pk"].tolist(), qd_pk=r["qd_pk"].tolist(), fell=r["fell"], tilt_max=r["tilt_max"],
                       clear_max=r["clear_max"], t_air=r["t_air"], step_T=tl.T,
                       speed_margin=r["speed_margin"].tolist(), torque_margin=r["torque_margin"].tolist())
        out[sv]["short"] = [JN[i] for i in range(12) if r["speed_margin"][i] < speed_req or r["torque_margin"][i] < torque_req
                            or r["sat"][i] > 0.0 or r["err_pk_deg"][i] > 5.0]
        out[sv]["ok"] = (not r["fell"]) and not out[sv]["short"] and r["clear_max"] >= 0.015 and r["t_air"] >= 0.3
        if verbose:
            print(f"== Gate B  step {tl.T:.1f} s (cadence x{1/scale:.0f})  {sv} on every joint: "
                  f"{'FELL' if r['fell'] else 'stayed up'}, max tilt {r['tilt_max']:.1f} deg, swing foot peak "
                  f"{1e3*r['clear_max']:.0f} mm off the ground for {r['t_air']:.2f} s")
            print(f"     {'joint':14s} {'w_pk rad/s':>10s} {'speed x':>8s} {'tau_pk':>7s} {'torque x':>8s} {'sat':>6s} {'err_pk':>7s}")
            for i, nme in enumerate(JN):
                flag = "  <--" if nme in out[sv]["short"] else ""
                print(f"     {nme:14s} {r['qd_pk'][i]:10.2f} {r['speed_margin'][i]:8.1f} {r['tau_pk'][i]:7.2f} "
                      f"{r['torque_margin'][i]:8.1f} {100*r['sat'][i]:5.1f}% {r['err_pk_deg'][i]:6.1f}{flag}")
    out["B_pass_3215"] = out["sts3215"]["ok"] if "sts3215" in out else None
    out["B_pass_3250"] = out["sts3250"]["ok"] if "sts3250" in out else None
    out["upgrade_candidates"] = out["sts3215"]["short"] if "sts3215" in out else []
    if verbose:
        print(f"  => Gate B: STS3215 everywhere {'PASS' if out['B_pass_3215'] else 'FAIL'} (short: {out['upgrade_candidates']}); "
              f"STS3250 everywhere {'PASS' if out['B_pass_3250'] else 'FAIL'} (short: {out.get('sts3250', {}).get('short')})")
    return out


# --------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-b", action="store_true")
    a, rest = ap.parse_known_args(argv)
    p, _ = params_from_args(rest + ["-o", "/dev/null"]) if "-o" not in rest else params_from_args(rest)
    if a.sweep:
        print(f"{'hip_sep':>7s} {'foot_w':>6s} {'leg':>4s} {'knee':>4s} | {'y*':>5s} {'roll':>5s} {'A1 mg':>5s} "
              f"{'A2 mg':>5s} {'A3 mg':>5s} {'crouch':>6s} | {'speedx':>6s} {'torqx':>6s} {'clear':>5s} {'tilt':>5s}")
        for hs, fw, leg, knee in itertools.product((0.070, 0.084, 0.096), (0.056, 0.064, 0.072),
                                                   (0.090, 0.100, 0.110), ("fwd", "bwd")):
            if fw > hs - 0.012:
                continue
            q = dataclasses.replace(p, hip_sep=hs, foot_w=fw, thigh=leg, shank=leg, knee=knee)
            A = gate_a(q, verbose=False)
            B = gate_b(q, A["A2"]["y_lift"], verbose=False, servos=("sts3215",))
            a3 = A["A3"]["margin"] if A["A3"] else float("nan")
            print(f"{1e3*hs:7.0f} {1e3*fw:6.0f} {1e3*leg:4.0f} {knee:>4s} | {1e3*A['A1']['y_star']:5.1f} "
                  f"{A['A1']['hip_roll_deg']:5.1f} {1e3*A['A1']['margin']:5.1f} "
                  f"{1e3*A['A2']['poses']['mid_swing'].get('margin', float('nan')):5.1f} {1e3*a3:5.1f} "
                  f"{1e3*A['A5']['crouch_depth']:6.0f} | {min(B['sts3215']['speed_margin']):6.2f} {min(B['sts3215']['torque_margin']):6.2f} {1e3*B['sts3215']['clear_max']:5.0f} {B['sts3215']['tilt_max']:5.1f} {'up' if not B['sts3215']['fell'] else 'FELL'}")
        return
    A = gate_a(p)
    res = dict(A=A)
    if not a.no_b:
        res["B"] = gate_b(p, A["A2"]["y_lift"])
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print("wrote", a.json)


if __name__ == "__main__":
    main()
