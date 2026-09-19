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
import multiprocessing as mp
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
    # 2026-09-17, sway/stride follow-up: `combo`'s default bias_y (-0.005,
    # inherited from static_gait's stock-tuned safety margin) targets the CoM
    # much closer to the sole's own centreline than the roll+adduct-assisted
    # body needs (kinematic minimum for margin>=0 is 10.5 mm of shift; combo
    # commands 49.1 mm) -- bias_y -0.010 cuts the commanded shift to 41.4 mm
    # and the measured pelvis sway by ~20% (187 -> 149 mm p2p/step) and still
    # clears all 4 gate cases (worst margin 9.3 mm); -0.015 does not (see
    # gateD_wide_gait_stride.txt).
    "combo_tight":  dict(drop=0.030, lift_h=0.070, foot_sep=0.14, roll_amp_deg=8.0,
                         t_shift=1.8, t_swing=1.8, t_land=0.6, bias_y=-0.010),
    # 2026-09-17, stride/cadence frontier winner on bird3_round (pooled
    # search, gateD_wide_gait_stride.txt): step 90 mm at cadence 0.8 s
    # (vs combo's 60 mm/1.8 s) is the fastest passing point on the whole
    # frontier by progress rate (1.60 cm/s vs combo's 0.41 cm/s) -- render
    # with step=0.09 (GAITS entries don't carry step length; wide_gait.py
    # render/build take it separately).
    "combo_fast":   dict(drop=0.030, lift_h=0.070, foot_sep=0.14, roll_amp_deg=12.0,
                         t_shift=0.8, t_swing=0.8, t_land=0.27),
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


# --------------------------------------------------------------------------- parallel execution
# 2026-09-17 (Tom via the coordinator): the gate/frontier sweeps were pegging
# ONE core each on a 32-core box. Every run_walk call is independent (its own
# Timeline solve + its own walker_env), so the seeds x cases x sweep points
# are natural pool.map work: each worker rebuilds its own Timeline from a
# picklable DesignParams and loads its own MjModel from the xml FILE PATH --
# no MjModel/MjData/env crosses the process boundary, only plain data in and
# a plain result dict out. fork context (not spawn): cheap, and MUJOCO_GL=egl
# is already in the environment every child inherits; no worker ever renders
# (render=None in every pooled call) so no GL context is touched post-fork.
POOL_WORKERS = 12   # coordinator cap: <= 12 for this script, box shared with the collective


def _run_job(job):
    """Pool worker for one run_walk call. `job`: p, xml, n_steps, step,
    turn_deg, kw (gait knobs), run_kw (seed/mu/play_deg/mass_scale/... passed
    straight to run_walk). Returns the run_walk result dict, or
    {'ik_failed': True, 'error': ...} if wide_walk_timeline itself raised
    (out of reach) before any physics ran."""
    try:
        tl, windows = wide_walk_timeline(job["p"], n_steps=job["n_steps"], step=job["step"],
                                         turn_deg=job["turn_deg"], **job["kw"])
    except ValueError as e:
        return {"ik_failed": True, "error": str(e)}
    r = SG.run_walk(job["p"], job["xml"], tl, windows, per_joint=per_joint_3250(), **job["run_kw"])
    r["ik_failed"] = False
    return r


def run_batch(jobs, pool=None):
    """run `jobs` (a list of _run_job dicts) through a forked pool, returning
    results in the SAME ORDER as `jobs` (Pool.map preserves order -- keeps
    printed logs deterministic/diffable run to run). Pass an existing `pool`
    to reuse it across many batches (cmd_gate/cmd_frontier/cmd_robust each
    open one pool for their whole run instead of one per batch)."""
    if not jobs:
        return []
    if pool is not None:
        return pool.map(_run_job, jobs)
    with mp.get_context("fork").Pool(min(POOL_WORKERS, len(jobs))) as p:
        return p.map(_run_job, jobs)


def _case_jobs(p, xml, n_steps, step, kw, cases=None):
    """flat (jobs, labels) for GATE_CASES (or `cases`) x 3 seeds each, in a
    fixed, deterministic order (case order, then seed 0/1/2)."""
    jobs, labels = [], []
    for label, kw0 in (cases or GATE_CASES):
        kw0 = dict(kw0)
        turn = kw0.pop("turn_deg", 0.0)
        for seed in range(3):
            run_kw = dict(kw0); run_kw["seed"] = seed
            jobs.append(dict(p=p, xml=xml, n_steps=n_steps, step=step, turn_deg=turn, kw=kw, run_kw=run_kw))
            labels.append(label)
    return jobs, labels


def _worst_by_label(results, labels, cases=None):
    """group pooled results back up by label (in the order of `cases`) and
    reduce each group of 3 seeds to the worst -- same reduction cmd_gate
    always used, just fed from the pool instead of a sequential loop."""
    out = {}
    for label, _ in (cases or GATE_CASES):
        rs = [r for lbl, r in zip(labels, results) if lbl == label]
        if rs and rs[0].get("ik_failed"):
            out[label] = None   # IK failed -- no dynamics row for this case
            continue
        out[label] = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
    return out


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
    print(f"== Gate D (4 cases, worst of 3 seeds) on {body} hip_sep {p.hip_sep:.3f} m, gait candidates from wide_gait.py (pooled, {POOL_WORKERS} workers) ==")
    with mp.get_context("fork").Pool(POOL_WORKERS) as pool:
        for name in names:
            print(f"-- {name}: {GAITS.get(name, {})}")
            jobs, labels = _case_jobs(p, xml, 8, 0.06, dict(GAITS[name]))
            results = run_batch(jobs, pool=pool)
            rows = _worst_by_label(results, labels)
            for label, _ in GATE_CASES:
                worst = rows[label]
                if worst is None:
                    err = next(r["error"] for lbl, r in zip(labels, results) if lbl == label and r.get("ik_failed"))
                    print(f"  {label:24s} IK FAILED: {err}")
                    continue
                rs = [r for lbl, r in zip(labels, results) if lbl == label]
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
    _, _, kw = build(p, xml, name)
    print(f"== robustness of {name} ({kw}) on {body}: mass x1.1, payload 100 g, floor tilt 3 deg (pooled, {POOL_WORKERS} workers) ==")
    cases = [("nominal", dict()), ("mass x1.1", dict(mass_scale=1.1)), ("payload 100 g", dict(payload=0.1)),
             ("floor tilt 3 deg", dict(floor_tilt_deg=3.0)),
             ("mass x1.1 + payload 100 g + tilt 3", dict(mass_scale=1.1, payload=0.1, floor_tilt_deg=3.0))]
    jobs, labels = [], []
    for label, kw2 in cases:
        for seed in range(3):
            run_kw = dict(kw2); run_kw["seed"] = seed
            jobs.append(dict(p=p, xml=xml, n_steps=8, step=0.06, turn_deg=0.0, kw=kw, run_kw=run_kw))
            labels.append(label)
    results = run_batch(jobs)
    for label, _ in cases:
        rs = [r for lbl, r in zip(labels, results) if lbl == label]
        worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
        print(SG.fmt_row(label, worst) + f"   ({sum(r['ok'] for r in rs)}/3 seeds ok)")


def sway_stride(res, windows):
    """per-step lateral sway (pelvis y peak-to-peak) and forward stride
    (pelvis x advance), from a track_sway=True run_walk result. One "step" =
    one shift-into-stance + swing + land cycle, bounded by consecutive
    windows' t0 (the last step also gets the final settle back to centre)."""
    t = np.array(res["traj_t"]); x = np.array(res["traj_x"]); y = np.array(res["traj_y"])
    bounds = [w[0] for w in windows] + [t[-1] + 1e-6]
    strides, sways = [], []
    for i in range(len(windows)):
        mask = (t >= bounds[i]) & (t < bounds[i + 1])
        if mask.sum() < 2:
            continue
        xs, ys = x[mask], y[mask]
        strides.append(float(xs[-1] - xs[0]))
        sways.append(float(ys.max() - ys.min()))
    return strides, sways


def _ss_mean(strides, sways):
    """mean over the full run, and over the 'steady-state' steps only (all
    but the last -- the last step's target is `last: x=0` i.e. the pelvis
    settles back to CENTRE rather than advancing, so its stride is near-zero
    or negative and its sway window is half-length; excluding it is the fair
    per-step comparison)."""
    ms, mw = float(np.mean(strides)), float(np.mean(sways))
    mss, msw = float(np.mean(strides[:-1])), float(np.mean(sways[:-1]))
    return (f"   MEAN (all {len(strides)} steps, incl. the non-advancing final settle): "
           f"stride {1e3*ms:5.1f} mm/step  sway {1e3*mw:5.1f} mm/step  ratio {ms/mw:4.2f}\n"
           f"   MEAN (steady-state, {len(strides)-1} steps): "
           f"stride {1e3*mss:5.1f} mm/step  sway {1e3*msw:5.1f} mm/step  ratio {mss/msw:4.2f}")


def cmd_sway():
    """task: quantify pelvis lateral sway (peak-to-peak y/step) vs forward
    stride (x advance/step) for the CURRENT combo gait on bird3_round and for
    the STOCK body on its OLD (static_gait.py-tuned) gait -- the two clips
    Tom watched."""
    pj = per_joint_3250()
    print("== sway (pelvis y peak-to-peak per step) vs stride (pelvis x advance per step) ==")
    # (a) bird3_round, current combo gait, nominal straight case (as rendered)
    p, xml = wide_plant("bird3_round")
    tl, windows, kw = build(p, xml, "combo")
    r = SG.run_walk(p, xml, tl, windows, per_joint=pj, track_sway=True)
    strides, sways = sway_stride(r, windows)
    print(f"-- bird3_round, combo gait {kw}")
    print(SG.fmt_row("  straight", r))
    for i, (s, w) in enumerate(zip(strides, sways)):
        print(f"   step {i+1}: stride {1e3*s:5.1f} mm  sway p2p {1e3*w:5.1f} mm  ratio(stride/sway) {s/max(w,1e-9):4.2f}")
    print(_ss_mean(strides, sways))
    # (b) stock body, OLD gait (static_gait.walk_timeline's own tuned defaults
    # -- n_steps=8, step 6 cm, lift 4 cm, drop 2 cm, 1.6 s shift/swing -- the
    # config gateD_appendages.txt's "bare" row and the pre-existing stock clip
    # both use)
    p2, xml2 = wide_plant("stock")
    tl2, windows2 = SG.walk_timeline(p2, n_steps=8, step=0.06, lift_h=0.04)
    r2 = SG.run_walk(p2, xml2, tl2, windows2, per_joint=pj, track_sway=True)
    strides2, sways2 = sway_stride(r2, windows2)
    print("-- stock, OLD gait (static_gait.walk_timeline defaults: step 6 cm, lift 4 cm, drop 2 cm, 1.6 s/1.6 s)")
    print(SG.fmt_row("  straight", r2))
    for i, (s, w) in enumerate(zip(strides2, sways2)):
        print(f"   step {i+1}: stride {1e3*s:5.1f} mm  sway p2p {1e3*w:5.1f} mm  ratio(stride/sway) {s/max(w,1e-9):4.2f}")
    print(_ss_mean(strides2, sways2))


CADENCES = (1.8, 1.5, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3)
ROLL_AMPS = (4.0, 6.0, 8.0, 10.0, 12.0)
STEP_LENS = (0.06, 0.09, 0.12, 0.15)


def _stride_kw(t_cad, roll_amp, step, drop=0.03, foot_sep=0.14):
    return dict(drop=drop, foot_sep=foot_sep, roll_amp_deg=roll_amp,
               t_shift=t_cad, t_swing=t_cad, t_land=max(0.2, round(t_cad / 3, 2)))


def _full_verify(p, xml, step, kw, pool):
    """all 4 GATE_CASES x 3 seeds, ONE pool.map of 12 jobs (was 12 sequential
    run_walk calls). Returns (ok_all, rows) -- rows has every case whose jobs
    all came back (IK-feasible); a FAILing case still gets its worst-of-3 row
    (useful for reporting exactly how it failed), an IK-infeasible case does
    not (ok_all is False either way)."""
    jobs, labels = _case_jobs(p, xml, 8, step, kw)
    results = run_batch(jobs, pool=pool)
    rows = _worst_by_label(results, labels)
    ok_all = all(r is not None and r["ok"] for r in rows.values())
    return ok_all, rows


CADENCES_DESC = tuple(sorted(CADENCES, reverse=True))   # slow -> fast


def _best_roll_at_cadence(p, xml, step, t_cad, pool):
    """try every ROLL_AMPS value at this (step, cadence) in ONE pool batch (5
    rolls x 4 cases x 3 seeds = 60 jobs); return (roll, rows) for the first
    roll (lowest first) that clears all 4 cases, or None."""
    jobs, labels, roll_of = [], [], []
    for roll in ROLL_AMPS:
        kw = _stride_kw(t_cad, roll, step)
        js, ls = _case_jobs(p, xml, 8, step, kw)
        jobs += js; labels += ls; roll_of += [roll] * len(js)
    results = run_batch(jobs, pool=pool)
    for roll in ROLL_AMPS:
        sub_r = [r for rl, r in zip(roll_of, results) if rl == roll]
        sub_l = [l for rl, l in zip(roll_of, labels) if rl == roll]
        rows = _worst_by_label(sub_r, sub_l)
        if all(r is not None and r["ok"] for r in rows.values()):
            return roll, rows
    return None


def cmd_frontier(body="bird3_round"):
    """for each step length, the fastest (t_shift=t_swing, t_land=t/3) x best
    roll_amp in {4..12} that STILL passes all 4 gate cases (3 seeds each) --
    drop 30 mm / foot_sep 140 mm held at the combo gait's own values.
    Sequential descent from the KNOWN-good 1.8 s cadence down through
    CADENCES_DESC, full 4-case x 3-seed x 5-roll verification at every
    cadence (300 run_walk calls per cadence tried, pooled to
    ceil(300/POOL_WORKERS) rounds) -- stop at the first cadence where NO roll
    clears all 4 cases."""
    p, xml = wide_plant(body)
    print(f"== stride/cadence frontier on {body} (hip_sep {p.hip_sep:.3f} m), combo's drop 30 mm / foot_sep 140 mm held fixed, pooled ({POOL_WORKERS} workers) ==")
    with mp.get_context("fork").Pool(POOL_WORKERS) as pool:
        for step in STEP_LENS:
            best = None
            for t_cad in CADENCES_DESC:
                found = _best_roll_at_cadence(p, xml, step, t_cad, pool)
                if found is None:
                    break
                roll, rows = found
                best = (t_cad, roll, _stride_kw(t_cad, roll, step), rows)
            if best is None:
                print(f"-- step {1e3*step:.0f} mm: FAILS even at the slowest cadence tried ({CADENCES_DESC[0]} s) with every roll 4-12 deg")
                continue
            t_cad, roll, kw, rows = best
            straight = rows["turn  +0 mu 0.7 play 3"]
            tl_t, windows_t = wide_walk_timeline(p, n_steps=8, step=step, **kw)
            r_sway = SG.run_walk(p, xml, tl_t, windows_t, per_joint=per_joint_3250(), track_sway=True)
            strides, sways = sway_stride(r_sway, windows_t)
            mss, msw = float(np.mean(strides[:-1])), float(np.mean(sways[:-1]))
            dist = math.hypot(straight["x_final"], straight["y_final"])
            print(f"-- step {1e3*step:.0f} mm  FASTEST PASSING: t_shift=t_swing {t_cad}s, t_land {kw['t_land']}s, roll {roll:.0f} deg")
            for label, _ in GATE_CASES:
                print("   " + SG.fmt_row(f"  {label}", rows[label]))
            print(f"   sway/stride (steady-state, straight case): stride {1e3*mss:.1f} mm/step  sway {1e3*msw:.1f} mm/step  ratio {mss/msw:.2f}")
            print(f"   8-step distance {100*dist:.1f} cm in {tl_t.T:.1f} s ({100*dist/tl_t.T:.2f} cm/s)")


def cmd_render(name, body="bird3_round", turn=0.0, out=None, step=0.06):
    p, xml = wide_plant(body)
    pj = per_joint_3250()
    tl, windows, kw = build(p, xml, name, step=step, turn=turn)
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
        step = float(rest[2]) if len(rest) > 2 else 0.06
        cmd_render(name, turn=turn, step=step)
    elif mode == "sway":
        cmd_sway()
    elif mode == "frontier":
        cmd_frontier(rest[0] if rest else "bird3_round")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
