"""Replay the 2026-11-11 stepper runs (hw/stepper, experiments/stepper) in the sim plant and score how
well the plant reproduces the robot (#288, #277, #259).

    python stepper_replay.py extract [STAMP ...]          # cache the robot's side (build/stepper-replay/)
    python stepper_replay.py run [--runs A,B] [--plants SPEC;SPEC] [--tag NAME] [--out DIR] [--workers N]
                                 [--no-plot] [--plots DIR]

A stepper run is a jsonl under hw_sessions/2026-11-11/stepper/<HHMMSS>_<lift|march|walk>.jsonl. The
file name is the host's LOCAL time (this box: MST, UTC-7) of the run's start; the run's own `t` is s
since then. The robot's side comes from the flight record that covers it
(hw_sessions/*/flight_20261111T*.frec): per tick the written goal (`goal_rad`, NaN on ticks without a
new goal: forward-filled), the encoders (`pos_rad`), the IMU (`up`, `gyro`). The run's clock is tied
to the robot's UTC by the IMU reads: every `imu` row the stepper logged is the end of an `imu` CLI
reply that the flight record also holds (event `out ... read`); the offset is their median, and the
check is the logged roll against the record's IMU at the aligned time.

The replay: the sim plant (gate_common.make_plant: `grade` | `train[:variant]`, plus probe terms) is
placed at the robot's measured joints 0.5 s before the run's first pose command and held 1.5 s
(simh.place_and_hold), then its position servos are driven tick by tick (50 Hz) with the robot's
written goal (zero-order hold of the forward-filled record). Nothing closes the loop: the stepper's
decisions are the robot's, the sim only answers "what would this body have done under those goals".

Plant specs: `grade`, `grade+play_hip=4,play_ankle=5`, `train`, `train:lash0.5+lash=0`, ... (the
part after `+` is gate_common.parse_terms). A label may be given as `label=SPEC`.

Angles: roll = -asin(up_y) (+ = leaning to the robot's RIGHT), pitch = atan2(up_x, up_z) (+ = forward),
on both sides (the sim's `up` is gate_common.up_of of the torso quaternion, the record's `up` the
firmware's yaw-stripped body z). Each trace has its value over the 0.5 s before the first pose removed
(the IMU's mount / the robot's stand offset is not modelled); the offsets are reported.

Per run and plant (results/stepper_replay/<tag>.csv and .md, printed; one PNG per run in --plots):
  roll_rms / roll_max, pitch_rms / pitch_max   sim - robot, deg, over the compared window (the first
                         pose command to the run's `end` or torque release, or to the robot's fall: the
                         first record tick past 35 deg)
  roll_rms_up, up_share  the roll RMS over the part of the window where both are upright (robot |roll|,
                         |pitch| and the sim's tilt under 20 deg), and that part's share
  air_droll_stL / _stR   the mean roll error (sim - robot) at the `air` rows, by STANCE side
  nool_droll_stL / _stR  the same at the `nool` rows (each lift increment), by STANCE side
  enc_rms(_roll)         joint encoders sim - robot (the sim's encoder: walker_env obs q, which reads
                         the servo shaft under roll play), deg: mean over the 12 (the 4 rolls) of the
                         per-joint RMS
  air_n, air_clear, air_clr_med, air_clr_min   at every `air` row of the stepper (the swing foot
                         judged up -- the stepper's own estimate, there is no contact sensor): how many,
                         the share at which the sim's swing sole (its lowest pad) is > 1 mm above the
                         floor, the median and min of that clearance, mm
  lift_up_sim            per `lift` outcome "up": did the sim's swing foot clear (> 1 mm) at that moment
  robot_fell, sim_fell   the robot: past 35 deg in the record; the sim: torso tilt > 35 deg (the
                         stepper's fall_deg) at any time up to the window's end (+1.5 s when the robot
                         fell); t_robot_fall / t_sim_fall in the CSV
  feet_cmd / feet_sim    walks: each foot's commanded displacement (the stepper's belief at its `end`;
                         none of these runs had a landing compensation in force) against the sim sole's
                         world displacement from the crouch, mm (x fwd, y left)
  steps (walks, march)   per step: the swing foot's commanded move against the sim sole's, and the
                         stance sole's slide meanwhile, mm (step_landings)
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

STEP_DIR = os.path.join(GC.HW, "2026-11-11", "stepper")
CACHE = os.path.join(GC.ROOT, "build", "stepper-replay", "cache")
OUT = os.path.join(HERE, "results", "stepper_replay")
PLOTS = os.path.join(GC.ROOT, "build", "stepper-replay", "plots")
JOINTS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
          "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
ROLLS = ["L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll"]
# the runs named in #288's brief: lifts (fit), march and walks (held out)
RUNS = ["101255_lift", "101413_lift", "103709_lift", "104032_lift", "104147_lift", "111627_lift",
        "104303_march", "104435_walk", "110045_walk", "110708_walk"]
PLANTS = ["grade", "grade+play_hip=4,play_ankle=5", "train"]
PRE_S = 0.5          # robot placement / offset window before the first pose command, s
FALL_DEG = 35.0
CLEAR_M = 0.001


# --------------------------------------------------------------------------------------------
# the robot's side
# --------------------------------------------------------------------------------------------
def run_path(stamp):
    p = os.path.join(STEP_DIR, stamp + ".jsonl")
    if not os.path.exists(p):
        g = sorted(glob.glob(os.path.join(STEP_DIR, stamp + "_*.jsonl")))
        if not g:
            raise FileNotFoundError(stamp)
        p = g[0]
    return p


def load_log(stamp):
    return [json.loads(l) for l in open(run_path(stamp)) if l.strip()]


def local_t0(stamp):
    """The run's file-name time (host local, the box that ran the stepper) as a UTC epoch."""
    hhmmss = os.path.basename(run_path(stamp))[:6]
    return time.mktime(time.strptime("2026-11-11 " + hhmmss, "%Y-%m-%d %H%M%S"))


def frecs():
    out = []
    for p in glob.glob(os.path.join(GC.HW, "*", "flight_20261111T*.frec")):
        st = os.path.basename(p).split("_")[1]           # 20261111T171003Z
        out.append((time.mktime(time.strptime(st, "%Y%m%dT%H%M%SZ")) - time.timezone, p))
    return sorted(out)


def frec_for(t_utc):
    best = None
    for st, p in frecs():
        if st <= t_utc:
            best = p
    return best


def _cache_path(stamp):
    return os.path.join(CACHE, stamp + ".npz")


def extract(stamps):
    """Cache each run's robot data (one load per flight record)."""
    sys.path.insert(0, os.path.join(GC.ROOT, "tools"))
    import flight_reader as FR
    os.makedirs(CACHE, exist_ok=True)
    by = {}
    for s in stamps:
        by.setdefault(frec_for(local_t0(s)), []).append(s)
    for path, ss in by.items():
        if path is None:
            print("no flight record for", ss)
            continue
        print("loading", os.path.relpath(path, GC.MAIN), "for", ss, flush=True)
        S = FR.load(path)
        ticks = FR.tick_rows(S)
        evs = FR.event_rows(S)
        tt = np.array([r["t_utc"] for r in ticks])
        for s in ss:
            log = load_log(s)
            t0 = local_t0(s)
            t_end = log[-1]["t"]
            # tie the run's clock to the robot's: each logged imu row is one `imu` CLI reply the
            # record also holds ("up x y z" line); the logged roll (2 decimals) picks out which
            # reply, within 1.5 s of the file-name time
            ups = []
            for e in evs:
                if e["kind"] == "out" and e["text"].startswith("up ") and t0 - 3 < e["t_utc"] < t0 + t_end + 5:
                    uy = float(e["text"].split()[2])
                    ups.append((e["t_utc"], -math.degrees(math.asin(max(-1.0, min(1.0, uy))))))
            ups = np.array(ups)
            dd = []
            for r in log:
                if r["ev"] != "imu":
                    continue
                c = np.where((np.abs(ups[:, 0] - (t0 + r["t"])) < 1.5) & (np.abs(ups[:, 1] - r["roll"]) < 0.006))[0]
                if len(c) == 1:                     # unambiguous only
                    dd.append(ups[c[0], 0] - (t0 + r["t"]))
            dd = np.array(dd)
            off = float(np.median(dd))
            resid = dd - off
            tu0 = t0 + off                                   # the run's t = 0, robot UTC
            # window: 3 s before the first pose command to 3 s after the end row
            k = (tt >= tu0 - 3.0) & (tt <= tu0 + t_end + 3.0)
            rows = [ticks[i] for i in np.where(k)[0]]
            t = np.array([r["t_utc"] for r in rows]) - tu0
            goal = np.array([[r[f"{n}.goal_rad"] for n in JOINTS] for r in rows], float)
            pos = np.array([[r[f"{n}.pos_rad"] for n in JOINTS] for r in rows], float)
            load = np.array([[r[f"{n}.load"] for n in JOINTS] for r in rows], float)
            up = np.array([[r["up_x"], r["up_y"], r["up_z"]] for r in rows], float)
            gyro = np.array([[r["gyro_x"], r["gyro_y"], r["gyro_z"]] for r in rows], float)
            src = np.array([r["source"] for r in rows])
            cmds = [(e["t_utc"] - tu0, e["text"]) for e in evs if e["kind"] == "cmd" and
                    tu0 - 3 < e["t_utc"] < tu0 + t_end + 3]
            # forward fill the goals; before the first written goal, the encoder
            g = goal.copy()
            for j in range(g.shape[1]):
                last = np.nan
                for i in range(len(g)):
                    if np.isfinite(g[i, j]):
                        last = g[i, j]
                    else:
                        g[i, j] = last
            # check: the logged roll against the record's IMU at the aligned time
            roll_rec = -np.degrees(np.arcsin(np.clip(up[:, 1], -1, 1)))
            chk = [r["roll"] - float(np.interp(r["t"], t, roll_rec)) for r in log if r["ev"] == "imu"]
            np.savez_compressed(_cache_path(s), t=t, goal=g, goal_raw=goal, pos=pos, load=load, up=up,
                                gyro=gyro, src=src, cmds=json.dumps(cmds), frec=os.path.relpath(path, GC.MAIN),
                                off=off, tu0=tu0, align_resid=resid, roll_chk=np.array(chk))
            print(f"  {s}: {len(t)} ticks, clock offset {off:+.3f} s (resid sd {1e3 * np.std(resid):.0f} ms, "
                  f"n {len(resid)}), logged roll - record roll: median {np.median(chk):+.2f}, "
                  f"max |.| {np.max(np.abs(chk)):.2f} deg", flush=True)


def robot(stamp):
    if not os.path.exists(_cache_path(stamp)):
        extract([stamp])
    z = np.load(_cache_path(stamp), allow_pickle=False)
    r = {k: z[k] for k in z.files}
    r["cmds"] = json.loads(str(r["cmds"]))
    r["log"] = load_log(stamp)
    return r


def window(r):
    """(t_first_pose, t_stop, robot_fell, t_fall): the compared window in run time. The robot's fall
    is the first record tick past 35 deg (the stepper's own `fallen` row comes later; the first
    stepper version, before 10:30, had no fall detection at all: 101413 fell at +34.7 s and was
    stood back up by hand)."""
    poses = [t for t, c in r["cmds"] if c.startswith("pose ")]
    t1 = poses[0]
    log = r["log"]
    rel = [t for t, c in r["cmds"] if c.startswith("release")]
    t2 = min([log[-1]["t"]] + [t for t in rel if t > t1])   # torque off at the release
    roll, pitch = rp_robot(r)
    big = np.where(((np.abs(roll) > FALL_DEG) | (np.abs(pitch) > FALL_DEG)) & (r["t"] >= t1) & (r["t"] <= t2))[0]
    if len(big):
        tf = float(r["t"][big[0]])
        return t1, tf, True, tf
    return t1, t2, False, None


def rp_robot(r):
    up = r["up"]
    return (-np.degrees(np.arcsin(np.clip(up[:, 1], -1, 1))), np.degrees(np.arctan2(up[:, 0], up[:, 2])))


# --------------------------------------------------------------------------------------------
# the sim's side
# --------------------------------------------------------------------------------------------
def parse_plant(spec):
    label, body = spec.split("=", 1) if "=" in spec.split("+")[0] else ("", spec)
    name, _, terms = body.partition("+")
    return (label or body), name, GC.parse_terms(terms)


def build(spec, ep_s):
    """The plant, as gate_common builds it: reset (seed 0, no DR), grade's own zero error, then the
    terms (so `zoff_La=` and the play are live). No second reset: it would undo `zoff_La`."""
    label, name, terms = parse_plant(spec)
    if name == "grade":
        env = GC.grade_env(terms=terms, ep_s=ep_s)[0]
    else:
        env = GC.make_plant(name, terms=terms, ep_s=ep_s)
    env.domain_rand = False
    env.push_prob = 0.0
    return label, env


def simulate(job):
    """(stamp, plant spec) -> dict of traces and metrics."""
    stamp, spec = job
    import mujoco
    import rec
    import simh
    r = robot(stamp)
    t1, t2, rfell, tfall = window(r)
    t_stop = t2 + (1.5 if rfell else 0.0)
    ep_s = (t_stop - t1) + 10.0
    label, env = build(spec, ep_s)
    pol = list(rec.POL)
    jidx = [JOINTS.index(n) for n in pol]                  # record (bus) order -> policy order
    t = r["t"]
    ts = t1 - PRE_S
    i0 = int(np.searchsorted(t, ts))
    q0 = r["pos"][i0][jidx]
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), 1.5)
    pads = GC.pad_gids(env)
    sole = env._sole_gids
    m, d = env.model, env.data
    goal = r["goal"][:, jidx]
    cur = {"t": q0.copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    dt = env.control_dt
    n = int(round((t_stop - ts) / dt))
    T, Q, UP, CL, SXY, QT = [], [], [], [], [], []
    for k in range(n):
        tk = ts + (k + 1) * dt
        i = int(np.searchsorted(t, tk, side="right")) - 1     # zero-order hold of the record
        gk = goal[max(i, 0)]
        cur["t"] = np.where(np.isfinite(gk), gk, cur["t"])
        obs, _, term, _, info = env.step(a0)
        T.append(tk)
        Q.append(obs[:env._nq_act].astype(np.float64))
        UP.append(GC.up_of(d.qpos[3:7]))
        QT.append(np.array(d.qpos[3:7]))
        c = GC.clearance(env, pads)
        CL.append((c["L"], c["R"]))
        SXY.append(np.concatenate([d.geom_xpos[sole[0]][:2], d.geom_xpos[sole[1]][:2]]))
    T, Q, UP, CL, SXY = map(np.array, (T, Q, UP, CL, SXY))
    roll_s = -np.degrees(np.arcsin(np.clip(UP[:, 1], -1, 1)))
    pitch_s = np.degrees(np.arctan2(UP[:, 0], UP[:, 2]))
    tilt_s = np.degrees(np.arccos(np.clip(UP[:, 2], -1, 1)))
    return dict(stamp=stamp, plant=label, spec=spec, T=T, Q=Q, pol=pol, roll=roll_s, pitch=pitch_s, tilt=tilt_s,
                clr=CL, sxy=SXY, t1=t1, t2=t2, rfell=rfell, tfall=tfall)


# --------------------------------------------------------------------------------------------
# scoring
# --------------------------------------------------------------------------------------------
def score(s, r=None, ref=None):
    """ref: "stand" (offsets over the 0.5 s before the first pose; the window from there) or "crouch"
    (offsets over the 0.5 s before the first shift pose, the stepper's own level reference; the
    window from there: the crouch's own lean is left out). Default: $SR_REF or "stand"."""
    r = r or robot(s["stamp"])
    ref = ref or os.environ.get("SR_REF", "stand")
    T = s["T"]
    t1, t2 = s["t1"], s["t2"]
    if ref == "crouch":
        poses = [t for t, c in r["cmds"] if c.startswith("pose ")]
        t1 = poses[2]                     # stand, crouch, then the first shift
    rr, pr = rp_robot(r)
    pre_r = (r["t"] >= t1 - PRE_S) & (r["t"] < t1)
    pre_s = (T >= t1 - PRE_S) & (T < t1)
    off_r = (float(np.median(rr[pre_r])), float(np.median(pr[pre_r])))
    off_s = (float(np.median(s["roll"][pre_s])), float(np.median(s["pitch"][pre_s])))
    w = (T >= t1) & (T <= t2)
    rr_i = np.interp(T, r["t"], rr) - off_r[0]
    pr_i = np.interp(T, r["t"], pr) - off_r[1]
    er = (s["roll"] - off_s[0]) - rr_i
    ep = (s["pitch"] - off_s[1]) - pr_i
    jidx = [JOINTS.index(n) for n in s["pol"]]
    pos = r["pos"][:, jidx]
    qi = np.stack([np.interp(T, r["t"], pos[:, j]) for j in range(pos.shape[1])], 1)
    eq = np.degrees(s["Q"] - qi)[w]
    jr = np.sqrt(np.nanmean(eq ** 2, 0))
    rolls = [s["pol"].index(n) for n in ROLLS]
    # the swing foot at the stepper's "up" judgements
    log = r["log"]
    air = [(x["t"], x["swing"]) for x in log if x["ev"] == "air" and t1 <= x["t"] <= t2]
    side = {"L": 0, "R": 1}
    ca = np.array([1e3 * float(np.interp(ta, T, s["clr"][:, side[sw]])) for ta, sw in air]) if air else np.array([])
    # the roll error while a foot was judged up (`air` rows), and at every lift increment (`nool`
    # rows: the foot raised, the stance hip trimmed), per STANCE side (the hip trim's side)
    nool = [(x["t"], x["swing"]) for x in log if x["ev"] == "nool" and t1 <= x["t"] <= t2]
    for st, sw in (("L", "R"), ("R", "L")):
        ta = [x for x, w_ in air if w_ == sw]
        s.setdefault("_air_err", {})[st] = float(np.mean(np.interp(ta, T, er))) if ta else None
        tn = [x for x, w_ in nool if w_ == sw]
        s.setdefault("_nool_err", {})[st] = float(np.mean(np.interp(tn, T, er))) if tn else None
        s.setdefault("_nool_n", {})[st] = len(tn)
    ups = [(x["t"], x["swing"]) for x in log if x["ev"] == "lift" and x["how"] == "up" and x["t"] <= t2]
    ups_sim = [bool(np.interp(ta, T, s["clr"][:, side[sw]]) > CLEAR_M) for ta, sw in ups]
    # sim fall
    wf = (T >= t1) & (T <= s["T"][-1])
    big = np.where(s["tilt"][wf] > FALL_DEG)[0]
    sim_fell = bool(len(big))
    t_sim_fall = float(T[wf][big[0]]) if sim_fell else None
    # both upright (robot |roll|, |pitch| and the sim's tilt under 20 deg): the comparison a fall does
    # not swamp
    rr_abs = np.interp(T, r["t"], np.maximum(np.abs(rr), np.abs(pr)))
    wu = w & (rr_abs < 20.0) & (s["tilt"] < 20.0)
    out = dict(run=s["stamp"], plant=s["plant"], dur=round(t2 - t1, 1),
               roll_rms_up=(float(np.sqrt(np.mean(er[wu] ** 2))) if wu.any() else None),
               up_share=float(wu.sum() / max(w.sum(), 1)),
               roll_rms=float(np.sqrt(np.mean(er[w] ** 2))), roll_max=float(np.max(np.abs(er[w]))),
               pitch_rms=float(np.sqrt(np.mean(ep[w] ** 2))), pitch_max=float(np.max(np.abs(ep[w]))),
               robot_roll_range=float(np.ptp(rr_i[w])), sim_roll_range=float(np.ptp((s["roll"] - off_s[0])[w])),
               enc_rms=float(np.nanmean(jr)), enc_rms_roll=float(np.nanmean(jr[rolls])),
               enc_worst=s["pol"][int(np.nanargmax(jr))], enc_worst_rms=float(np.nanmax(jr)),
               air_droll_stL=s["_air_err"]["L"], air_droll_stR=s["_air_err"]["R"],
               nool_droll_stL=s["_nool_err"]["L"], nool_droll_stR=s["_nool_err"]["R"],
               nool_n_stL=s["_nool_n"]["L"], nool_n_stR=s["_nool_n"]["R"],
               air_n=len(air), air_clear=(float(np.mean(ca > 1e3 * CLEAR_M)) if len(ca) else None),
               air_clr_med=(float(np.median(ca)) if len(ca) else None),
               air_clr_min=(float(np.min(ca)) if len(ca) else None),
               lift_up_robot=len(ups), lift_up_sim=int(sum(ups_sim)),
               robot_fell=s["rfell"], sim_fell=sim_fell, t_sim_fall=t_sim_fall, t_robot_fall=s["tfall"],
               off_robot_roll=off_r[0], off_robot_pitch=off_r[1], off_sim_roll=off_s[0], off_sim_pitch=off_s[1])
    # feet: sim world displacement from the crouch (the first pose done) vs the stepper's belief
    endrow = [x for x in log if x["ev"] == "end"]
    if endrow and ("walk" in s["stamp"] or "march" in s["stamp"]):
        crouch = [x["t"] for x in log if x["ev"] == "pose" and x["tag"] == "crouch"]
        tc = crouch[0] if crouch else t1
        k0 = int(np.searchsorted(T, tc))
        k1 = int(np.searchsorted(T, min(t2, T[-1]))) - 1
        dxy = 1e3 * (s["sxy"][k1] - s["sxy"][k0])
        out.update(sim_L_dx=float(dxy[0]), sim_L_dy=float(dxy[1]), sim_R_dx=float(dxy[2]), sim_R_dy=float(dxy[3]))
        fr = [x for x in log if x["ev"] in ("end", "fallen") and "feet" in x] or \
             [x for x in log if x["ev"] == "pose" and "feet" in x]
        if fr:
            f = fr[-1]["feet"]
            out.update(cmd_L_dx=1e3 * f["L"][0], cmd_L_dy=1e3 * f["L"][1], cmd_R_dx=1e3 * f["R"][0], cmd_R_dy=1e3 * f["R"][1])
    trace = dict(T=T, w=w, er=er, ep=ep, rr=rr_i, pr=pr_i, rs=s["roll"] - off_s[0], ps=s["pitch"] - off_s[1],
                 clr=s["clr"], air=air, qs=s["Q"], qr=qi)
    return out, trace


def step_landings(s, r=None):
    """Per step (a foot raised, moved, put down): the commanded move of that foot (the stepper's pose
    rows: its believed foot offsets) against the sim sole's world move over the same span, and how
    far the stance sole slid meanwhile, mm. The span runs from the last pose before the foot leaves
    to 0.5 s after the first pose that has it back at z = 0."""
    r = r or robot(s["stamp"])
    T, xy = s["T"], s["sxy"]
    poses = [x for x in r["log"] if x["ev"] == "pose" and "feet" in x and x["t"] <= s["t2"]]
    col = {"L": (0, 1), "R": (2, 3)}
    out, i = [], 1
    while i < len(poses):
        for sw in "LR":
            if poses[i]["feet"][sw][2] > 0 and poses[i - 1]["feet"][sw][2] == 0:
                j = i
                while j < len(poses) and poses[j]["feet"][sw][2] > 0:
                    j += 1
                if j >= len(poses):
                    break
                st = "R" if sw == "L" else "L"
                ta, tb = poses[i - 1]["t"], min(poses[j]["t"] + 0.5, T[-1])
                ka, kb = int(np.searchsorted(T, ta)), int(np.searchsorted(T, tb)) - 1
                d = 1e3 * (xy[kb] - xy[ka])
                c0, c1 = poses[i - 1]["feet"][sw], poses[j]["feet"][sw]
                out.append(dict(run=s["stamp"], plant=s["plant"], swing=sw, t=round(ta, 1),
                                cmd_dx=1e3 * (c1[0] - c0[0]), cmd_dy=1e3 * (c1[1] - c0[1]),
                                sim_dx=float(d[col[sw][0]]), sim_dy=float(d[col[sw][1]]),
                                stance_dx=float(d[col[st][0]]), stance_dy=float(d[col[st][1]])))
                i = j
                break
        i += 1
    return out


BOARD = os.path.join(GC.ROOT, "build", "stepper-replay", "board")


def board_landings(stamp, path=None, conf=0.8, span=1.5):
    """The robot's own lateral step moves from the front camera: a CSV of the stepper worktree's
    `board_view.py track <clip>_sony.mp4 --board-w 450` (sole_l_mm / sole_r_mm across the board, +
    = the robot's left; frames below `conf` dropped). Per step (the same spans as step_landings),
    the median sole position over `span` s before the foot leaves and after it is down.
    -> {t (run time, 0.1 s): (dy_swing_mm, dy_stance_mm, n_before, n_after)}, {} without a CSV."""
    path = path or os.path.join(BOARD, stamp + ".csv")
    if not os.path.exists(path):
        return {}
    r = robot(stamp)
    rows = [x for x in csv.DictReader(open(path)) if x["conf"] and float(x["conf"]) >= conf]
    tb_ = np.array([float(x["utc"]) for x in rows]) - float(r["tu0"])
    col = {s_: np.array([float(x[f"sole_{s_.lower()}_mm"]) for x in rows]) for s_ in "LR"}
    med = lambda s_, a, b: (float(np.median(col[s_][(tb_ >= a) & (tb_ <= b)])) if ((tb_ >= a) & (tb_ <= b)).any()
                            else float("nan"), int(((tb_ >= a) & (tb_ <= b)).sum()))
    out = {}
    for sw, ta, tb in _step_spans(r["log"], 1e9):
        st = "R" if sw == "L" else "L"
        a, na = med(sw, ta - span, ta)
        z, nz = med(sw, tb, tb + span)
        a2, _ = med(st, ta - span, ta)
        z2, _ = med(st, tb, tb + span)
        out[round(ta, 1)] = (z - a, z2 - a2, na, nz)
    return out


def _step_spans(log, t_end):
    """(swing side, t before it leaves, t 0.5 s after it is down, the two pose rows) per step."""
    poses = [x for x in log if x["ev"] == "pose" and "feet" in x and x["t"] <= t_end]
    out, i = [], 1
    while i < len(poses):
        for sw in "LR":
            if poses[i]["feet"][sw][2] > 0 and poses[i - 1]["feet"][sw][2] == 0:
                j = i
                while j < len(poses) and poses[j]["feet"][sw][2] > 0:
                    j += 1
                if j < len(poses):
                    out.append((sw, poses[i - 1]["t"], poses[j]["t"] + 0.5))
                    i = j
                break
        i += 1
    return out


def steps_md(steps):
    if not steps:
        return ""
    lines = ["| run | plant | t | swing | cmd dx / dy | sim dx / dy | sim - cmd y | robot dy (camera) | robot - cmd y | "
             "stance slid sim / robot y |", "|---|---|---|---|---|---|---|---|---|---|"]
    f = lambda v, fm="+.0f": "-" if v is None or not np.isfinite(v) else format(v, fm)
    for x in steps:
        rb = x.get("robot_dy")
        lines.append(f"| {x['run']} | {x['plant']} | {x['t']:.0f} | {x['swing']} | {x['cmd_dx']:.0f} / {x['cmd_dy']:.0f} | "
                     f"{x['sim_dx']:.0f} / {x['sim_dy']:.0f} | {x['sim_dy'] - x['cmd_dy']:+.0f} | {f(rb)} | "
                     f"{f(None if rb is None else rb - x['cmd_dy'])} | {x['stance_dy']:.0f} / {f(x.get('robot_stance_dy'))} |")
    lines += ["", "| plant | swing | steps | sim - cmd y, mean (range) | robot - cmd y (camera), mean (range) |",
              "|---|---|---|---|---|"]
    for p in dict.fromkeys(x["plant"] for x in steps):
        for sw in "LR":
            xs = [x for x in steps if x["plant"] == p and x["swing"] == sw]
            sm = np.array([x["sim_dy"] - x["cmd_dy"] for x in xs])
            rm = np.array([x["robot_dy"] - x["cmd_dy"] for x in xs if x.get("robot_dy") is not None
                           and np.isfinite(x["robot_dy"])])
            rng = lambda a: f"{a.mean():+.0f} ({a.min():+.0f}..{a.max():+.0f}), n {len(a)}" if len(a) else "-"
            lines.append(f"| {p} | {sw} | {len(xs)} | {rng(sm)} | {rng(rm)} |")
    return "\n".join(lines)


COLS = [("run", "run", "s"), ("plant", "plant", "s"), ("dur", "s", ".0f"),
        ("roll_rms_up", "roll rms upright", ".1f"), ("up_share", "upright share", ".2f"),
        ("roll_rms", "roll rms", ".1f"), ("roll_max", "roll max", ".1f"),
        ("pitch_rms", "pitch rms", ".1f"), ("pitch_max", "pitch max", ".1f"),
        ("robot_roll_range", "robot roll p-p", ".1f"), ("sim_roll_range", "sim roll p-p", ".1f"),
        ("enc_rms", "enc rms", ".2f"), ("enc_rms_roll", "enc rms rolls", ".2f"),
        ("air_droll_stL", "roll err up, L stance", ".1f"), ("air_droll_stR", "roll err up, R stance", ".1f"),
        ("air_n", "air rows", "d"), ("air_clear", "sim clear share", ".2f"), ("air_clr_med", "sim clr med mm", ".1f"),
        ("lift_up_robot", "ups robot", "d"), ("lift_up_sim", "ups sim", "d"),
        ("robot_fell", "robot fell", "s"), ("sim_fell", "sim fell", "s")]


def fmtv(v, f):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "-"
    if f == "s":
        return str(v)
    if f == "d":
        return str(int(v))
    return format(v, f)


def table_md(rows):
    lines = ["| " + " | ".join(c[1] for c in COLS) + " |", "|" + "---|" * len(COLS)]
    for r in rows:
        lines.append("| " + " | ".join(fmtv(r.get(k), f) for k, _, f in COLS) + " |")
    return "\n".join(lines)


def summary_md(rows, groups=None):
    """per plant (and run group): the mean upright roll RMS, fall agreement, swing-clear agreement."""
    groups = groups or {"all": None}
    lines = ["| plant | runs | mean roll rms upright | mean pitch rms | falls agree | sim falls, robot not | "
             "robot falls, sim not | mean sim clear share at `air` | mean enc rms rolls |", "|---|---|---|---|---|---|---|---|---|"]
    for g, members in groups.items():
        for p in dict.fromkeys(r["plant"] for r in rows):
            rs = [r for r in rows if r["plant"] == p and (members is None or r["run"] in members)]
            if not rs:
                continue
            mean = lambda k: np.mean([r[k] for r in rs if r.get(k) is not None]) if any(r.get(k) is not None for r in rs) else None
            agree = sum(r["robot_fell"] == r["sim_fell"] for r in rs)
            lines.append(f"| {p} | {g} ({len(rs)}) | {fmtv(mean('roll_rms_up'), '.2f')} | {fmtv(mean('pitch_rms'), '.2f')} | "
                         f"{agree}/{len(rs)} | {sum(r['sim_fell'] and not r['robot_fell'] for r in rs)} | "
                         f"{sum(r['robot_fell'] and not r['sim_fell'] for r in rs)} | {fmtv(mean('air_clear'), '.2f')} | "
                         f"{fmtv(mean('enc_rms_roll'), '.2f')} |")
    return "\n".join(lines)


def feet_md(rows):
    rs = [r for r in rows if "sim_L_dx" in r]
    if not rs:
        return ""
    lines = ["| run | plant | L cmd x / y | L sim x / y | R cmd x / y | R sim x / y |", "|---|---|---|---|---|---|"]
    for r in rs:
        c = lambda s, a: fmtv(r.get(f"cmd_{s}_{a}"), ".0f")
        m = lambda s, a: fmtv(r.get(f"sim_{s}_{a}"), ".0f")
        lines.append(f"| {r['run']} | {r['plant']} | {c('L', 'dx')} / {c('L', 'dy')} | {m('L', 'dx')} / {m('L', 'dy')} | "
                     f"{c('R', 'dx')} / {c('R', 'dy')} | {m('R', 'dx')} / {m('R', 'dy')} |")
    return "\n".join(lines)


def plot(stamp, traces, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    first = next(iter(traces.values()))
    T, w = first["T"], first["w"]
    ax[0].plot(T[w], first["rr"][w], "k", lw=1.6, label="robot")
    ax[1].plot(T[w], first["pr"][w], "k", lw=1.6, label="robot")
    for lab, tr in traces.items():
        ax[0].plot(tr["T"][tr["w"]], tr["rs"][tr["w"]], lw=1.0, label=lab)
        ax[1].plot(tr["T"][tr["w"]], tr["ps"][tr["w"]], lw=1.0, label=lab)
    for k, side in enumerate("LR"):
        for lab, tr in traces.items():
            ax[2 + k].plot(tr["T"][tr["w"]], 1e3 * tr["clr"][tr["w"], k], lw=1.0, label=lab)
        ta = [a for a, sw in first["air"] if sw == side]
        if ta:
            ax[2 + k].plot(ta, np.zeros(len(ta)), "k|", ms=10, label="stepper: judged up")
        ax[2 + k].axhline(0, color="0.6", lw=0.5)
        ax[2 + k].set_ylabel(f"{side} sole lowest pad, mm (sim)")
        ax[2 + k].set_ylim(-2, 40)
    ax[0].set_ylabel("roll deg (+ right)")
    ax[1].set_ylabel("pitch deg (+ fwd)")
    ax[0].legend(fontsize=7, loc="best")
    ax[2].legend(fontsize=7, loc="upper right")
    ax[3].set_xlabel("s since the run started (stepper clock)")
    ax[0].set_title(f"stepper {stamp}: robot vs sim plants, goals replayed open loop")
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("extract", "run"))
    ap.add_argument("stamps", nargs="*")
    ap.add_argument("--runs", default=",".join(RUNS))
    ap.add_argument("--plants", default=";".join(PLANTS), help="';'-separated plant specs")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--tag", default="baseline")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--ref", choices=("stand", "crouch"), default="stand",
                    help="the level reference and window start (score's doc)")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--plots", default=PLOTS, help="where the per-run PNGs go (gitignored build/ by default)")
    a = ap.parse_args(argv)
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    runs = a.stamps or [x for x in a.runs.split(",") if x]
    if a.what == "extract":
        extract(runs)
        return 0
    for s in runs:
        if not os.path.exists(_cache_path(s)):
            extract([s])
    plants = [p for p in a.plants.split(";") if p]
    jobs = [(s, p) for s in runs for p in plants]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        res = list(ex.map(simulate, jobs))
    os.makedirs(a.out, exist_ok=True)
    rows, traces, steps = [], {}, []
    for s in res:
        bl = board_landings(s["stamp"])
        for x in step_landings(s):
            b = bl.get(round(x["t"], 1))
            if b:
                x.update(robot_dy=b[0], robot_stance_dy=b[1])
            steps.append(x)
        o, tr = score(s, ref=a.ref)
        rows.append(o)
        traces.setdefault(s["stamp"], {})[s["plant"]] = tr
    keys = sorted({k for r in rows for k in r}, key=lambda k: ([c[0] for c in COLS] + sorted({k for r in rows for k in r})).index(k))
    with open(os.path.join(a.out, f"{a.tag}.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        for r in rows:
            wr.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    md = table_md(rows)
    fm = feet_md(rows)
    sm = summary_md(rows, {"lifts": [x for x in runs if "lift" in x], "march + walks": [x for x in runs if "lift" not in x]})
    stp = steps_md([x for x in steps if "walk" in x["run"] or "march" in x["run"]])
    open(os.path.join(a.out, f"{a.tag}.md"), "w").write(sm + "\n\n" + md + "\n\n" + fm + "\n\n" + stp + "\n")
    print(sm)
    print()
    print(md)
    print()
    print(fm)
    print()
    print(stp)
    if not a.no_plot:
        os.makedirs(a.plots, exist_ok=True)
        for st, tr in traces.items():
            plot(st, tr, os.path.join(a.plots, f"{a.tag}_{st}.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
