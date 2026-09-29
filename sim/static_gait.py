"""Gates C and D for the v6 body: an open-loop quasi-static walk (no policy) on
the dynamic plant under the DEPLOY servo model, swept over the things the
2026-09-13 lessons say hid from us -- sole friction, roll-chain play, gear
backlash, body mass, payload, weak servos, a tilted floor.

The walk is the Gate A path made periodic: shift the pelvis until the whole-
robot CoM (solved on the kinematics, swing foot at mid-swing) sits on the
stance sole's centreline, then swing the other foot forward on a sine arc,
settle, and shift the other way. Every foot placement is flat, every pelvis
pose level; joint targets come from sim/v6_kin.leg_ik at 50 Hz.

Servo model: walker_env "sts3215" (PD 12 N*m/rad clamped to the measured
torque-speed line at 11.1 V), 2 Hz actuation lag + 4 ticks (80 ms) dead time
(bench 2026-08-31 / 09-05), integer goal ticks, 5 ms bus latency, 1 deg gear
backlash, 3 deg free play on the four roll joints (bench 2026-09-02).

    .venv/bin/python sim/static_gait.py                      # nominal, 6 steps, table
    .venv/bin/python sim/static_gait.py --sweep              # Gate C/D matrix
    .venv/bin/python sim/static_gait.py --render sim/renders/v6_walk.mp4
    .venv/bin/python sim/static_gait.py --knee bwd --set ankle_range=55
    .venv/bin/python sim/static_gait.py --knees3250          # STS3250 at the knees
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from gen_plant_v6 import DesignParams, params_from_args, build_xml  # noqa: E402
import v6_kin as K  # noqa: E402
from design_gates import Key, Timeline, standing_key, q_of, SERVOS, JN  # noqa: E402
from walker_env import BimoWalkerEnv  # noqa: E402

ROLLS = ("L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll")


# --------------------------------------------------------------------------- gait
def solve_pelvis(m, d, p, pelvis, feet, stance, swing_mid, bias=(0.0, 0.0),
                 heading=0.0, yaws=None):
    """pelvis (x, y) that puts the kinematic CoM on the stance sole centre (+bias,
    in the stance foot frame: x forward, y toward robot left) with the swing
    foot at swing_mid. Fixed-point iteration; converges in ~4 rounds. World
    frame: pelvis heading and per-foot yaws (rad) are honoured."""
    pel = np.array(pelvis, float)
    cx = p.foot_toe - p.foot_len / 2
    yaws = yaws or {"L": 0.0, "R": 0.0}
    Rf = K._rz(yaws[stance])[:2, :2]
    for _ in range(12):
        fl = feet["L"] if stance == "L" else swing_mid
        fr = feet["R"] if stance == "R" else swing_mid
        q = K.pose_world(p, pel, heading, fl, fr, yaws["L"], yaws["R"])
        K.set_pose(m, d, q, torso_pos=pel + np.array([0, 0, 1.0]), torso_quat=K.heading_quat(heading))
        _, c = K.com_margin(m, d, p, stance)
        err = np.array([cx + bias[0] - c[0], bias[1] - c[1]])     # in the stance foot frame
        pel[:2] += Rf @ err
        if np.linalg.norm(err) < 1e-5:
            break
    return pel


def walk_timeline(p: DesignParams, n_steps=6, step=0.06, lift_h=0.03, drop=0.02,
                  t_shift=1.6, t_swing=1.6, t_settle=0.2, t_land=0.5, bias_y=-0.005, in_place=False,
                  swing_out=0.01, land_out=0.012, turn_deg=0.0):
    """alternating steps starting with the RIGHT foot; returns the timeline and
    a list of (t0, t1, swing_side) swing windows. turn_deg: heading change per
    step (+ = left); each swing foot is placed in the NEW heading's frame and
    the pelvis heading follows the mean of the two foot yaws (the hip yaws
    then carry +-turn/2)."""
    m, d = K.load(p)
    k = standing_key(p, drop)
    # start where the robot starts: straight legs (qpos0), then ease into the
    # working crouch -- a step command to the crouch pitches the body forward
    # because the knee (2x the travel) arrives last
    tl = Timeline(standing_key(p, 0.0)).hold(0.5).to(k, 1.5).hold(0.5)
    feet = {"L": k.footL.copy(), "R": k.footR.copy()}
    yaws = {"L": 0.0, "R": 0.0}
    heading = 0.0
    pel = k.pelvis.copy()
    windows = []
    swing = "R"
    s = step / 2
    dturn = math.radians(turn_deg)
    for i in range(n_steps):
        stance = "L" if swing == "R" else "R"
        sgn = 1.0 if stance == "L" else -1.0          # +1: stance foot is on the robot's left
        last = (i == n_steps - 1)
        if in_place:
            target = feet[swing].copy()
            yaw_new = yaws[swing]
        else:
            # place the swing foot in the NEW heading's frame: `s` ahead of the
            # stance foot (0 on the last step) and hip_sep across
            yaw_new = yaws[stance] + dturn
            Rn = K._rz(yaw_new)
            target = feet[stance] + Rn @ np.array([0.0 if last else s, -sgn * p.hip_sep, 0.0])
            target[2] = p.roll_h
        heading_new = 0.5 * (yaws[stance] + yaw_new)
        mid = 0.5 * (feet[swing] + target) + np.array([0, 0, lift_h])
        yaws_mid = dict(yaws); yaws_mid[swing] = 0.5 * (yaws[swing] + yaw_new)
        pel_new = solve_pelvis(m, d, p, pel, feet, stance, mid, bias=(0.0, sgn * bias_y),
                               heading=heading_new, yaws=yaws_mid)
        pel_new[2] = pel[2]
        kk = Key(pel_new, feet["L"], feet["R"], heading_new, yaws["L"], yaws["R"])
        # shift time scales with the lateral distance (t_shift is per 60 mm):
        # the crossover from one stance to the other is twice the first shift
        dist = float(np.linalg.norm(pel_new[:2] - pel[:2]))
        tl.to(kk, t_shift * max(1.0, dist / 0.06)).hold(t_settle)
        t0 = tl.T
        # land_out: the hanging swing leg sags ~12-15 mm INWARD (hip/ankle roll
        # under the leg's own weight at 12 N*m/rad + play), so the foot is
        # commanded to land that far outboard; the command then relaxes to the
        # nominal foot position over t_land while the loaded foot stays put.
        # swing_out: extra sideways bump away from the stance foot at mid-swing.
        # t_land: the swing leg lags its command by ~0.3 s through the 2 Hz
        # shaper + 80 ms dead time; let it land before the next shift starts.
        out = K._rz(yaw_new) @ np.array([0.0, -sgn * land_out, 0.0])
        kk2 = kk.copy()
        if swing == "L":
            kk2.footL = target + out; kk2.yawL = yaw_new
        else:
            kk2.footR = target + out; kk2.yawR = yaw_new
        tl.to(kk2, t_swing, arc=lift_h, arc_foot=swing, arc_out=swing_out)
        kk3 = kk2.copy()
        if swing == "L":
            kk3.footL = target
        else:
            kk3.footR = target
        tl.to(kk3, t_land)
        windows.append((t0, t0 + t_swing + t_land, swing))
        feet[swing] = target.copy()
        yaws[swing] = yaw_new
        heading = heading_new
        pel = pel_new
        swing = "L" if swing == "R" else "R"
    # finish: pelvis centred between the feet, heading = mean foot yaw
    hend = 0.5 * (yaws["L"] + yaws["R"])
    kend = Key(np.array([0.5 * (feet["L"][0] + feet["R"][0]), 0.5 * (feet["L"][1] + feet["R"][1]), pel[2]]),
               feet["L"], feet["R"], hend, yaws["L"], yaws["R"])
    tl.to(kend, t_shift).hold(1.0)
    tl.final_heading = hend
    return tl, windows


# --------------------------------------------------------------------------- run
def make_env(p: DesignParams, xml_path, mu=0.7, play_deg=3.0, backlash_deg=1.0, lag_hz=2.0,
             delay_ticks=4, latency_ms=5.0, payload=0.0, servo_kp=12.0, servo_kd=0.25, episode_s=60.0):
    env = BimoWalkerEnv(xml_path=xml_path, actuator_model="sts3215", supply_voltage=11.1,
                        servo_kp=servo_kp, servo_kd=servo_kd, act_lag_hz=lag_hz, act_delay_ticks=delay_ticks,
                        backlash_deg=backlash_deg, play_deg=play_deg, play_joints=ROLLS if play_deg > 0 else None,
                        quantize_ticks=True, latency_ms=latency_ms, payload_mass=payload, action_map="full",
                        payload_cg_x=0.028, payload_cg_z=p.housing_h + p.deck_t + 0.008,   # the Pi bay
                        episode_seconds=episode_s, fall_up_z=0.6)
    m = env.model
    for g in range(m.ngeom):
        if m.geom_friction[g][0] > 0:
            m.geom_friction[g][0] = mu
    return env


def run_walk(p: DesignParams, xml_path, tl: Timeline, windows, seed=0, mu=0.7, play_deg=3.0, backlash_deg=1.0,
             mass_scale=1.0, servo_scale=1.0, payload=0.0, floor_tilt_deg=0.0, per_joint=None,
             lag_hz=2.0, delay_ticks=4, render=None, verbose=False, fb_ankle=0.0, fb_hip=0.0, fb_i=0.0,
             fb_sag=0.0, fb_sag_joints=None, track_torque=False, track_sway=False, track_speed=False,
             track_trace=False):
    """fb_ankle / fb_hip: proportional IMU-roll feedback (rad per rad of torso
    roll error) added to the ankle-roll / hip-roll targets of BOTH legs -- the
    simplest compliance-aware controller, what a policy would learn first.
    fb_i: integral gain on the same error (per second).
    fb_sag: joint-position-error feedback on fb_sag_joints (default: the four
    roll joints): target += fb_sag * (target - measured), i.e. software
    stiffening from the servo's own position readback -- what a policy that
    sees joint positions can do, limited by the 2 Hz shaper + dead time.
    track_torque (2026-09-17, wide_gait.py): also return res["peak_tau"] /
    res["stall"], the per-joint (order JN) peak |applied torque| over the run
    and the per-joint stall used, from the servo model's own qfrc_applied.
    track_speed (2026-09-26, gate_no3250.py): also return res["peak_qd"] /
    res["w0"], the per-joint peak |joint speed| and the no-load speed used.
    track_sway (2026-09-17, wide_gait.py): also return res["traj_t"] /
    res["traj_x"] / res["traj_y"], the pelvis (torso freejoint) x/y position
    every tick -- for measuring lateral sway vs forward stride per step.
    track_trace (2026-09-29, current_budget_v6.py): also return
    res["trace_tau"] / res["trace_qd"] (ticks x joints, every control tick:
    the servo model's torque and the joint speed) and res["w0"].
    Off by default -- no effect on the existing return dict."""
    env = make_env(p, xml_path, mu=mu, play_deg=play_deg, backlash_deg=backlash_deg, lag_hz=lag_hz,
                   delay_ticks=delay_ticks, payload=payload)
    m = env.model
    if mass_scale != 1.0:
        m.body_mass[:] *= mass_scale
        m.body_inertia[:] *= mass_scale
    if floor_tilt_deg:
        a = math.radians(floor_tilt_deg)
        m.opt.gravity[:] = [-9.81 * math.sin(a), 0.0, -9.81 * math.cos(a)]   # +tilt = downhill forward
    obs, _ = env.reset(seed=seed)
    kp, kd, stall, w0 = env._servo
    na = env._nq_act                      # 12, or 13 with the v7 neck
    stall = np.full(na, stall) * servo_scale
    w0 = np.full(na, w0) * servo_scale
    kp = np.full(na, float(kp))
    kd = np.full(na, float(kd))
    if per_joint:
        for i, n in enumerate(JN):
            if n in per_joint:
                s = SERVOS[per_joint[n]]
                stall[i] = s["stall"] * servo_scale
                w0[i] = s["w0"] * servo_scale
                kp[i] *= s["kp_scale"]
                kd[i] *= s["kp_scale"]
    env._servo = np.array([kp, kd, stall, w0], dtype=object)
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        if len(q) < na:
            # the joints the gait does not drive (neck, arms) hold the plant's
            # default pose: 0 for the neck, the arms' walking hold (qpos0, 15 deg
            # back on the robot's plant; 0 on every plant built before it)
            q = np.concatenate([q, d0[len(q):]])
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6),
                                (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    dt = env.control_dt
    pads = {s: [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{s}_pad_")]
            for s in "LR"}
    fb = {s: m.body(f"{s}_foot").id for s in "LR"}
    frames = []
    if render:
        rnd = mujoco.Renderer(m, 480, 640)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = (0.75, -10, 135) if not getattr(tl, "final_heading", 0.0) else (1.1, -30, 120)
    steps = [dict(peak=0.0, t_air=0.0, margin_min=1e9, slip=0.0, done=False) for _ in windows]
    t = 0.0
    fell = False
    tilt_max = 0.0
    stance_start = {}
    n_ticks = int(tl.T / dt) + 1
    i_roll = 0.0
    sag_idx = [JN.index(j) for j in (fb_sag_joints or ROLLS)]
    iL, iR = JN.index("L_ankle_roll"), JN.index("R_ankle_roll")
    hL, hR = JN.index("L_hip_roll"), JN.index("R_hip_roll")
    peak_tau = np.zeros(na) if track_torque else None
    peak_qd = np.zeros(na) if track_speed else None
    traj_t, traj_x, traj_y = ([], [], []) if track_sway else (None, None, None)
    tr_tau, tr_qd = ([], []) if track_trace else (None, None)
    for k in range(n_ticks):
        key = tl.at(t)
        try:
            qc = q_of(p, key)
        except ValueError:
            pass
        if fb_ankle or fb_hip or fb_i:
            # torso roll about its own x axis (the commanded attitude is level)
            Rm = env.data.xmat[env._torso_bid].reshape(3, 3)
            roll_err = math.atan2(Rm[2, 1], Rm[2, 2])
            i_roll += roll_err * dt
            corr = fb_ankle * roll_err + fb_i * i_roll
            qc = qc.copy()
            qc[iL] += corr; qc[iR] += corr
            qc[hL] += fb_hip * roll_err; qc[hR] += fb_hip * roll_err
        if fb_sag:
            qm = env.data.qpos[7:19]
            qc = qc.copy()
            qc[sag_idx] += fb_sag * (qc[sag_idx] - qm[sag_idx])
        obs, r, term, trunc, _ = env.step(inv(qc))
        if track_torque:
            # env._servo_tau inherits dtype=object from env._servo (built as an
            # object array above to hold ragged per-joint scalars/arrays) --
            # cast to float before combining with the float64 accumulator.
            np.maximum(peak_tau, np.abs(env._servo_tau.astype(np.float64)), out=peak_tau)
        if track_speed:
            np.maximum(peak_qd, np.abs(env.data.qvel[env._jqvel]), out=peak_qd)
        if track_trace:
            tr_tau.append(np.asarray(env._servo_tau, dtype=np.float64).copy())
            tr_qd.append(np.asarray(env.data.qvel[env._jqvel], dtype=np.float64).copy())
        if track_sway:
            traj_t.append(t); traj_x.append(float(env.data.qpos[0])); traj_y.append(float(env.data.qpos[1]))
        d = env.data
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        tilt_max = max(tilt_max, math.degrees(math.acos(max(-1.0, min(1.0, up)))))
        for i, (t0, t1, sw) in enumerate(windows):
            if t0 <= t <= t1:
                st = "L" if sw == "R" else "R"
                h = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads[sw])
                steps[i]["peak"] = max(steps[i]["peak"], h)
                if h > 0.005:
                    steps[i]["t_air"] += dt
                mg, _ = K.com_margin(m, d, p, st)
                steps[i]["margin_min"] = min(steps[i]["margin_min"], mg)
                if i not in stance_start:
                    stance_start[i] = d.xpos[fb[st]][:2].copy()
                steps[i]["slip"] = max(steps[i]["slip"], float(np.linalg.norm(d.xpos[fb[st]][:2] - stance_start[i])))
            if t > t1:
                steps[i]["done"] = True
        if render and k % 2 == 0:
            cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.22]
            rnd.update_scene(d, cam)
            frames.append(rnd.render().copy())
        if term or up < 0.6:
            fell = True
            break
        t += dt
    completed = sum(1 for s in steps if s["done"])
    qw = env.data.qpos[3:7]
    heading = math.atan2(2 * (qw[0] * qw[3] + qw[1] * qw[2]), 1 - 2 * (qw[2] ** 2 + qw[3] ** 2))
    res = dict(fell=fell, t_fell=t if fell else None, steps_completed=completed, n_steps=len(windows),
               heading_deg=math.degrees(heading),
               tilt_max=tilt_max, x_final=float(env.data.qpos[0]), y_final=float(env.data.qpos[1]),
               clear_peaks=[round(1e3 * s["peak"], 1) for s in steps], t_air=[round(s["t_air"], 2) for s in steps],
               margin_min=[round(1e3 * s["margin_min"], 1) if s["margin_min"] < 1e8 else None for s in steps],
               slip_max=round(1e3 * max(s["slip"] for s in steps), 1) if steps else 0.0)
    done = [s for s in steps if s["done"]]
    res["min_clear_peak"] = min((s["peak"] for s in done), default=0.0)
    res["min_t_air"] = min((s["t_air"] for s in done), default=0.0)
    res["min_margin"] = min((s["margin_min"] for s in done), default=float("nan"))
    res["ok"] = (not fell) and completed == len(windows) and res["min_clear_peak"] >= 0.015 and res["min_t_air"] >= 0.3
    if track_torque:
        res["peak_tau"] = peak_tau.tolist()
        res["stall"] = stall.tolist()
    if track_speed:
        res["peak_qd"] = peak_qd.tolist()
        res["w0"] = w0.tolist()
    if track_trace:
        res["trace_tau"] = np.array(tr_tau)
        res["trace_qd"] = np.array(tr_qd)
        res["w0"] = w0.tolist()
        res["stall"] = stall.tolist()
    if track_sway:
        res["traj_t"] = traj_t
        res["traj_x"] = traj_x
        res["traj_y"] = traj_y
    if render:
        _write_video(frames, render, fps=25)
    if verbose:
        print(fmt_row("run", res))
    return res


def fmt_row(label, r):
    return (f"{label:44s} {'FELL@%.1fs' % r['t_fell'] if r['fell'] else 'up      ':>10s} steps {r['steps_completed']}/{r['n_steps']}  "
            f"clear>= {1e3*r['min_clear_peak']:4.0f} mm  air>= {r['min_t_air']:.2f} s  CoM margin>= {1e3*r['min_margin']:5.1f} mm  "
            f"slip {r['slip_max']:4.1f} mm  tilt {r['tilt_max']:4.1f}  x {100*r['x_final']:+5.1f} y {100*r['y_final']:+5.1f} cm  "
            f"hdg {r.get('heading_deg', 0.0):+5.1f}  {'OK' if r['ok'] else 'FAIL'}")


def _write_video(frames, path, fps=25):
    import imageio
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with imageio.get_writer(path, fps=fps, codec="libx264", quality=8, macro_block_size=None) as w:
        for f in frames:
            w.append_data(f)
    # filmstrip next to it (png, committed; the mp4 is gitignored): 2 x 4
    # frames spanning the second and third steps, so a full swing is visible
    n = 8
    idx = np.linspace(int(0.22 * len(frames)), int(0.50 * len(frames)), n).astype(int)
    rows = [np.concatenate([frames[i] for i in idx[:4]], axis=1),
            np.concatenate([frames[i] for i in idx[4:]], axis=1)]
    imageio.imwrite(os.path.splitext(path)[0] + "_strip.png", np.concatenate(rows, axis=0))
    print(f"wrote {path} ({len(frames)} frames) and the filmstrip png")


# --------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--render", default=None)
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--step-len", type=float, default=0.06)
    ap.add_argument("--lift", type=float, default=0.03)
    ap.add_argument("--t-shift", type=float, default=1.6, help="s per 60 mm of lateral pelvis travel")
    ap.add_argument("--t-swing", type=float, default=1.6)
    ap.add_argument("--t-land", type=float, default=0.5)
    ap.add_argument("--bias-y", type=float, default=-0.005, help="CoM target offset from the stance sole centreline, "
                    "+ = outboard. Slightly INBOARD is the safe side: a drift then falls toward the incoming swing foot")
    ap.add_argument("--in-place", action="store_true")
    ap.add_argument("--turn", type=float, default=0.0, help="heading change per step, deg (+ = left)")
    ap.add_argument("--swing-out", type=float, default=0.01, help="outward sideways bump of the swing foot (m)")
    ap.add_argument("--land-out", type=float, default=0.012, help="feed-forward outboard landing offset (m)")
    ap.add_argument("--knees3250", action="store_true", help="STS3250 at both knees")
    ap.add_argument("--rolls3250", action="store_true", help="STS3250 at hip roll + ankle roll (4)")
    ap.add_argument("--all3250", action="store_true")
    ap.add_argument("--fb-ankle", type=float, default=0.0, help="IMU roll -> ankle roll P gain (rad/rad)")
    ap.add_argument("--fb-hip", type=float, default=0.0, help="IMU roll -> hip roll P gain (rad/rad)")
    ap.add_argument("--mu", type=float, default=0.7)
    ap.add_argument("--play", type=float, default=3.0)
    ap.add_argument("--json", default=None)
    ap.add_argument("--torso-v7", action="store_true")
    a, rest = ap.parse_known_args(argv)
    if a.torso_v7:
        rest = rest + ["--torso-v7"]
    p, _ = params_from_args(rest + ["-o", "/dev/null"])
    xml_path = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"v6_{os.getpid()}.xml")
    with open(xml_path, "w") as f:
        f.write(build_xml(p))
    tl, windows = walk_timeline(p, n_steps=a.steps, step=a.step_len, lift_h=a.lift, t_shift=a.t_shift,
                                t_swing=a.t_swing, t_land=a.t_land, bias_y=a.bias_y, in_place=a.in_place,
                                swing_out=a.swing_out, land_out=a.land_out, turn_deg=a.turn)
    per_joint = {}
    if a.knees3250 or a.all3250:
        per_joint.update({"L_knee": "sts3250", "R_knee": "sts3250"})
    if a.rolls3250 or a.all3250:
        per_joint.update({j: "sts3250" for j in ROLLS})
    if a.all3250:
        per_joint.update({j: "sts3250" for j in JN})
    per_joint = per_joint or None
    print(f"design: {p.summary()}   gait: step {100*a.step_len:.0f} cm, lift {100*a.lift:.0f} cm, turn {a.turn:.0f} deg/step, "
          f"shift {a.t_shift} s, swing {a.t_swing} s, {a.steps} steps, {tl.T:.1f} s total"
          f"{'  STS3250 at ' + ','.join(sorted(per_joint)) if per_joint else '  STS3215 everywhere'}")
    if not a.sweep:
        r = run_walk(p, xml_path, tl, windows, mu=a.mu, play_deg=a.play, per_joint=per_joint, render=a.render,
                     fb_ankle=a.fb_ankle, fb_hip=a.fb_hip)
        print(fmt_row(f"nominal mu {a.mu} play {a.play}", r))
        if a.json:
            json.dump(r, open(a.json, "w"), indent=1)
        return
    cases = [("nominal (mu 0.7, play 3, lash 1, lag 2 Hz + 80 ms)", {})]
    for mu in (0.3, 0.5, 1.0):
        cases.append((f"friction mu {mu}", dict(mu=mu)))
    cases.append(("no play, no backlash (best case chains)", dict(play_deg=0.0, backlash_deg=0.0)))
    cases.append(("play 5 deg on rolls", dict(play_deg=5.0)))
    cases.append(("backlash 2 deg", dict(backlash_deg=2.0)))
    for ms in (0.85, 1.15):
        cases.append((f"mass x{ms}", dict(mass_scale=ms)))
    for pl in (0.06, 0.12):
        cases.append((f"payload {1e3*pl:.0f} g on the deck front", dict(payload=pl)))
    cases.append(("servos -15 % (stall and speed)", dict(servo_scale=0.85)))
    cases.append(("servos -30 %", dict(servo_scale=0.70)))
    for tl_deg in (2.0, -2.0):
        cases.append((f"floor tilt {tl_deg:+.0f} deg (fore-aft)", dict(floor_tilt_deg=tl_deg)))
    cases.append(("no lag / no dead time (ideal chain)", dict(lag_hz=0.0, delay_ticks=0)))
    cases.append(("mu 0.3 + play 5 + servos -15 %", dict(mu=0.3, play_deg=5.0, servo_scale=0.85)))
    results = {}
    for label, kw in cases:
        rs = [run_walk(p, xml_path, tl, windows, seed=s, per_joint=per_joint, **kw) for s in range(3)]
        worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
        results[label] = dict(kw=kw, runs=rs)
        print(fmt_row(label, worst) + f"   ({sum(r['ok'] for r in rs)}/3 seeds ok)")
    if a.json:
        json.dump(dict(design=p.summary(), gait=vars(a), results=results), open(a.json, "w"), indent=1)
        print("wrote", a.json)


if __name__ == "__main__":
    main()
