"""Wide-hip gait candidates for the flat "bird" slab body (bird3_round /
bird3_box, hip_sep 0.18 m vs the stock body's 0.084 m).

Problem (docs/design-v6/study-side-mounted-legs.md R3.4, gateD_bird3.txt):
static_gait.walk_timeline's weight-shift moves the pelvis LATERALLY, level,
until the kinematic CoM sits on the stance sole. For a wide hip_sep the
"other" (about-to-swing) leg's hip-to-foot vector grows by roughly hip_sep
as the pelvis crosses over, and with the standing-height leg already near
its full thigh+shank extension (0.220 m) there is no reach budget left for
that lateral component: leg_ik raises "out of reach". The round-3 workaround
(solve the IK at a fictitious, narrower hip_sep, then apply those joint
angles to the real wide-hip body) does not fix this -- it plans a CoM
alignment for a body that does not exist, so the real body's CoM ends up
just off the sole (margin -2.9 to -4.3 mm on all 4 gate cases; clear 0 mm).

Three real mechanisms, all built on the SAME analytic leg IK (v6_kin) and
the SAME Key/Timeline (design_gates) that static_gait.run_walk consumes
unchanged -- q_of/Key/Timeline gained an optional `roll` field (default
0.0, backward compatible: tests/test_v6_design_gates.py still 8/8) that
lets a Key command the WHOLE pelvis/torso tilted sideways, not just
translated:

  CROUCH  -- bend the knees deeper before shifting (bigger `drop`). The
             standing-height leg is already near-fully extended, so ANY
             lateral demand exceeds reach; dropping the pelvis shortens the
             vertical leg vector and buys back reach for the lateral one.
             This alone, at the REAL hip_sep (no narrowing hack), clears
             all 4 gate cases (see docs/design-v6/study-wide-hip-gait.md).
  WADDLE  -- roll the pelvis about its own forward axis during the shift
             (a Key.roll term) instead of only translating it, so part of
             the CoM's lateral travel comes from tilting the body rather
             than stretching the stance leg sideways.
  ADDUCT  -- plan the foot placement INBOARD of the real hip line (a
             `foot_sep` narrower than the real hip_sep) via hip-roll
             adduction, but -- unlike the round-3 workaround -- solve the
             pelvis shift against the ACTUAL (real hip_sep) body, so the
             CoM alignment is not a planning fiction.

Any combination is `wide_walk_timeline(..., drop=..., roll_amp_deg=...,
foot_sep=...)` -- one function, all three knobs.

    .venv/bin/python sim/wide_gait.py sweep [name]        # triage grid, 1 seed
    .venv/bin/python sim/wide_gait.py gate NAME [kwargs]  # 4-case gate, 3 seeds
    .venv/bin/python sim/wide_gait.py torque NAME [kwargs]
    .venv/bin/python sim/wide_gait.py robust NAME [kwargs]
    .venv/bin/python sim/wide_gait.py render NAME [kwargs]
"""
from __future__ import annotations

import dataclasses
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import v6_kin as K  # noqa: E402
import design_gates as G  # noqa: E402
import static_gait as SG  # noqa: E402
from getup_v6_side import CONFIGS, BASE  # noqa: E402

ROLLS = SG.ROLLS
JN = G.JN


# --------------------------------------------------------------------------- plant
def wide_plant(name="bird3_round", **extra):
    """the side-mounted-leg body config, forced to knee='fwd' for the gait IK
    (v6_kin.leg_ik does not accept knee='both'; normal walking never needs the
    backward ROM -- same substitution R3.4 used, applied here without any
    hip_sep narrowing)."""
    p = dataclasses.replace(BASE, **{**CONFIGS[name], "knee": "fwd", **extra})
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"wide_{name}_{os.getpid()}.xml")
    with open(xml, "w") as f:
        f.write(build_xml(p))
    return p, xml


# --------------------------------------------------------------------------- gait
def standing_key_w(p: DesignParams, drop: float, foot_sep: float | None = None) -> G.Key:
    """standing_key generalised with an optional foot separation != p.hip_sep
    (ADDUCT: feet placed inboard of the real hip mount)."""
    fs = p.hip_sep if foot_sep is None else foot_sep
    z_pel = p.z_yaw_above_sole - p.hip_z - drop
    return G.Key(np.array([0.0, 0.0, z_pel]),
                np.array([0.0, +fs / 2, p.roll_h]),
                np.array([0.0, -fs / 2, p.roll_h]))


def solve_pelvis_w(m, d, p, pelvis, feet, stance, swing_mid, roll=0.0, bias=(0.0, 0.0),
                   heading=0.0, yaws=None):
    """static_gait.solve_pelvis generalised with a fixed torso ROLL (WADDLE):
    the pelvis (x, y) that puts the kinematic CoM on the stance sole centre
    GIVEN that the whole torso is tilted by `roll` about its own forward
    axis. roll=0.0 reproduces static_gait.solve_pelvis exactly."""
    pel = np.array(pelvis, float)
    cx = p.foot_toe - p.foot_len / 2
    yaws = yaws or {"L": 0.0, "R": 0.0}
    Rf = K._rz(yaws[stance])[:2, :2]
    for _ in range(12):
        fl = feet["L"] if stance == "L" else swing_mid
        fr = feet["R"] if stance == "R" else swing_mid
        q = K.pose_world(p, pel, heading, fl, fr, yaws["L"], yaws["R"], roll=roll)
        K.set_pose(m, d, q, torso_pos=pel + np.array([0, 0, 1.0]), torso_quat=K.orient_quat(heading, roll))
        _, c = K.com_margin(m, d, p, stance)
        err = np.array([cx + bias[0] - c[0], bias[1] - c[1]])
        pel[:2] += Rf @ err
        if np.linalg.norm(err) < 1e-5:
            break
    return pel


def wide_walk_timeline(p: DesignParams, n_steps=8, step=0.06, lift_h=0.04, drop=0.02,
                       foot_sep=None, roll_amp_deg=0.0, t_shift=1.6, t_swing=1.6, t_settle=0.2,
                       t_land=0.5, bias_y=-0.005, swing_out=0.01, land_out=0.012, turn_deg=0.0):
    """static_gait.walk_timeline generalised with two knobs on top of `drop`
    (CROUCH, already in the original): `foot_sep` (ADDUCT: plan the feet at
    this separation instead of the real p.hip_sep) and `roll_amp_deg`
    (WADDLE: roll the pelvis toward the stance side by this much while
    shifting, unrolling as the swing foot lands). foot_sep=None and
    roll_amp_deg=0.0 reproduce static_gait.walk_timeline's CROUCH-only
    behaviour (same fixed-point solve, roll=0 no-op)."""
    m, d = K.load(p)
    fs = p.hip_sep if foot_sep is None else foot_sep
    # k0: the robot's actual straight-leg stand (feet under the REAL hips,
    # zero crouch -- this is the only pose reachable at drop=0.0, since a
    # straight leg is already at full thigh+shank extension with zero
    # lateral budget). The ease-in to kc then crouches AND narrows the
    # stance together, never asking for an adducted foot at drop=0.
    k0 = standing_key_w(p, 0.0, None)
    kc = standing_key_w(p, drop, foot_sep)
    tl = G.Timeline(k0).hold(0.5).to(kc, 1.5).hold(0.5)
    feet = {"L": kc.footL.copy(), "R": kc.footR.copy()}
    yaws = {"L": 0.0, "R": 0.0}
    heading = 0.0
    pel = kc.pelvis.copy()
    windows = []
    swing = "R"
    s = step / 2
    dturn = math.radians(turn_deg)
    roll_amp = math.radians(roll_amp_deg)
    for i in range(n_steps):
        stance = "L" if swing == "R" else "R"
        sgn = 1.0 if stance == "L" else -1.0
        last = (i == n_steps - 1)
        yaw_new = yaws[stance] + dturn
        Rn = K._rz(yaw_new)
        target = feet[stance] + Rn @ np.array([0.0 if last else s, -sgn * fs, 0.0])
        target[2] = p.roll_h
        heading_new = 0.5 * (yaws[stance] + yaw_new)
        mid = 0.5 * (feet[swing] + target) + np.array([0, 0, lift_h])
        yaws_mid = dict(yaws); yaws_mid[swing] = 0.5 * (yaws[swing] + yaw_new)
        roll = sgn * roll_amp   # tilt toward the stance side while the pelvis shifts onto it
        pel_new = solve_pelvis_w(m, d, p, pel, feet, stance, mid, roll=roll, bias=(0.0, sgn * bias_y),
                                 heading=heading_new, yaws=yaws_mid)
        pel_new[2] = pel[2]
        kk = G.Key(pel_new, feet["L"], feet["R"], heading_new, yaws["L"], yaws["R"], roll)
        dist = float(np.linalg.norm(pel_new[:2] - pel[:2]))
        tl.to(kk, t_shift * max(1.0, dist / 0.06)).hold(t_settle)
        t0 = tl.T
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
    hend = 0.5 * (yaws["L"] + yaws["R"])
    kend = G.Key(np.array([0.5 * (feet["L"][0] + feet["R"][0]), 0.5 * (feet["L"][1] + feet["R"][1]), pel[2]]),
                feet["L"], feet["R"], hend, yaws["L"], yaws["R"], 0.0)
    tl.to(kend, t_shift).hold(1.0)
    tl.final_heading = hend
    return tl, windows


# --------------------------------------------------------------------------- named candidates
# gait kwargs on top of wide_walk_timeline's defaults (step 0.06, turn per case)
GAITS = {
    "crouch40_90":  dict(drop=0.040, lift_h=0.090, t_shift=2.0, t_swing=2.0, t_land=0.7),
    "crouch30_80":  dict(drop=0.030, lift_h=0.080, t_shift=1.6, t_swing=1.6, t_land=0.5),
    "crouch20_40":  dict(drop=0.020, lift_h=0.040, t_shift=1.6, t_swing=1.6, t_land=0.5),   # stock-tuned cadence, for contrast
    "waddle10":     dict(drop=0.020, lift_h=0.040, roll_amp_deg=10.0),          # roll alone at the stock cadence/crouch: reach-limited (fails)
    "waddle15_crouch30": dict(drop=0.030, lift_h=0.060, roll_amp_deg=15.0),    # too much roll for the servo model at this cadence (unstable)
    "waddle8_crouch40": dict(drop=0.040, lift_h=0.080, roll_amp_deg=8.0,
                             t_shift=1.8, t_swing=1.8, t_land=0.6),
    "adduct14":     dict(drop=0.020, lift_h=0.040, foot_sep=0.14),             # adduction alone at the stock crouch: still reach-limited
    "adduct12":     dict(drop=0.020, lift_h=0.040, foot_sep=0.12),
    "adduct12_crouch30": dict(drop=0.030, lift_h=0.060, foot_sep=0.12),
    "combo":        dict(drop=0.030, lift_h=0.070, foot_sep=0.14, roll_amp_deg=8.0,
                         t_shift=1.8, t_swing=1.8, t_land=0.6),
}

GATE_CASES = (("turn  +0 mu 0.7 play 3", dict()), ("turn +15 mu 0.7 play 3", dict(turn_deg=15.0)),
             ("turn  +0 mu 0.3 play 5", dict(mu=0.3, play_deg=5.0)),
             ("turn -15 mu 0.9 play 3", dict(turn_deg=-15.0, mu=0.9)))


def per_joint_3250():
    return {j: "sts3250" for j in ROLLS + ("L_knee", "R_knee")}


def build(p, xml, name, n_steps=8, step=0.06, extra_kw=None, turn=0.0):
    kw = dict(GAITS[name])
    kw.update(extra_kw or {})
    turn_deg = kw.pop("turn_deg", turn)
    tl, windows = wide_walk_timeline(p, n_steps=n_steps, step=step, turn_deg=turn_deg, **kw)
    return tl, windows, kw


# --------------------------------------------------------------------------- CLI
def cmd_sweep(names):
    """quick 1-seed triage on the straight-walk case only."""
    p, xml = wide_plant("bird3_round")
    pj = per_joint_3250()
    print(f"== sweep (straight, mu 0.7 play 3, 1 seed): bird3_round hip_sep {p.hip_sep:.3f} m, knee fwd ==")
    for name in names:
        try:
            tl, windows, kw = build(p, xml, name)
            r = SG.run_walk(p, xml, tl, windows, per_joint=pj)
            print(SG.fmt_row(f"{name:24s} {kw}", r))
        except ValueError as e:
            print(f"{name:24s} IK FAILED: {e}")


def cmd_gate(names, body="bird3_round"):
    p, xml = wide_plant(body)
    pj = per_joint_3250()
    print(f"== Gate D (4 cases, worst of 3 seeds) on {body} hip_sep {p.hip_sep:.3f} m, gait candidates from wide_gait.py ==")
    for name in names:
        print(f"-- {name}: {GAITS.get(name, {})}")
        for label, kw0 in GATE_CASES:
            kw = dict(kw0)
            turn = kw.pop("turn_deg", 0.0)
            try:
                tl, windows, _ = build(p, xml, name, turn=turn)
            except ValueError as e:
                print(f"  {label:24s} IK FAILED: {e}")
                continue
            rs = [SG.run_walk(p, xml, tl, windows, per_joint=pj, seed=s, **kw) for s in range(3)]
            worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
            dist = math.hypot(worst["x_final"], worst["y_final"])
            print(SG.fmt_row(f"  {label}", worst) + f"  dist {100*dist:.1f} cm  ({sum(r['ok'] for r in rs)}/3 seeds ok)")


def cmd_torque(names, body="bird3_round"):
    p, xml = wide_plant(body)
    pj = per_joint_3250()
    print(f"== peak joint torque vs stall, straight-walk nominal case, {body} ==")
    fams = {"hip_yaw": [], "hip_roll": [], "hip_pitch": [], "knee": [], "ankle": [], "ankle_roll": []}
    for name in names:
        try:
            tl, windows, kw = build(p, xml, name)
        except ValueError as e:
            print(f"{name:24s} IK FAILED: {e}")
            continue
        r = SG.run_walk(p, xml, tl, windows, per_joint=pj, track_torque=True)
        tau = np.array(r["peak_tau"]); stall = np.array(r["stall"])
        by_fam = {k: [] for k in fams}
        for j, t, s in zip(JN, tau, stall):
            for fam in fams:
                if j.endswith(fam):
                    by_fam[fam].append((j, t, s))
        print(f"-- {name} ({kw})  {'OK' if r['ok'] else 'FAIL'}")
        for fam, items in by_fam.items():
            if not items:
                continue
            jname, t, s = max(items, key=lambda x: x[1] / x[2])
            print(f"   {fam:12s} peak {t:5.2f} Nm  stall {s:5.2f} Nm  margin {s/max(t,1e-6):4.1f}x  ({jname})")


def cmd_robust(name, body="bird3_round"):
    p, xml = wide_plant(body)
    pj = per_joint_3250()
    tl, windows, kw = build(p, xml, name)
    print(f"== robustness of {name} ({kw}) on {body}: mass x1.1, payload 100 g, floor tilt 3 deg ==")
    cases = [("nominal", dict()), ("mass x1.1", dict(mass_scale=1.1)), ("payload 100 g", dict(payload=0.1)),
             ("floor tilt 3 deg", dict(floor_tilt_deg=3.0)),
             ("mass x1.1 + payload 100 g + tilt 3", dict(mass_scale=1.1, payload=0.1, floor_tilt_deg=3.0))]
    for label, kw2 in cases:
        rs = [SG.run_walk(p, xml, tl, windows, per_joint=pj, seed=s, **kw2) for s in range(3)]
        worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
        print(SG.fmt_row(label, worst) + f"   ({sum(r['ok'] for r in rs)}/3 seeds ok)")


def cmd_render(name, body="bird3_round", turn=0.0, out=None):
    p, xml = wide_plant(body)
    pj = per_joint_3250()
    tl, windows, kw = build(p, xml, name, turn=turn)
    family = "bird3" if body.startswith("bird3") else body   # match the study's file-naming convention
    out = out or f"sim/renders/getup_options/side/{family}_walk_{name}_{'straight' if turn == 0.0 else 'turn' + str(int(turn))}.mp4"
    r = SG.run_walk(p, xml, tl, windows, per_joint=pj, render=out)
    print(SG.fmt_row(f"{body} {name} render", r))


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print(__doc__)
        return
    mode, rest = argv[0], argv[1:]
    if mode == "sweep":
        cmd_sweep(rest or list(GAITS))
    elif mode == "gate":
        body = "bird3_round"
        names = []
        for a in rest:
            if a in CONFIGS:
                body = a
            else:
                names.append(a)
        cmd_gate(names or list(GAITS), body=body)
    elif mode == "torque":
        cmd_torque(rest or list(GAITS))
    elif mode == "robust":
        cmd_robust(rest[0] if rest else "crouch40_90")
    elif mode == "render":
        name = rest[0] if rest else "crouch40_90"
        turn = float(rest[1]) if len(rest) > 1 else 0.0
        cmd_render(name, turn=turn)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
