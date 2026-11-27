#!/usr/bin/env python3
"""The weight-shift snap (#259 Round 7, #297) through the sim plants: the robot's written goals replayed open loop,
and every step read with tip_causes.py's own per-step code.

    python snap_replay.py run --day 20261116 --from 1415 --to 2010 [--plants 'NAME=SPEC;...'] [--delay 0,0.3rt,fill]
                              [--side L] [--workers 6] [--name NAME]
    python snap_replay.py report CSV [CSV ...]

The robot side is tip_causes.py's cache (build/tip-causes/<stamp>.npz; run `tip_causes.py extract` first). The
record docs/design-v6/2026-11-25-left-stance-tips-a-vs-b.md found that every late left-stance tip of the 11-16 evening
followed a "snap" in the weight shift before the lift-off (the roll joints fall behind the shift's fastest part, catch
up with a load spike, the body rolls 3-4 deg toward the coming swing foot), and that the snap needs the shift to start
within 0.5 s of the previous touchdown. This tool asks whether a plant does the same under the same goals.

Per single support n (k >= 2 on its side; a tip_causes.py row):
- The plant (a stepper_replay.build spec, `grade+<gate_common terms>`, plus footoff_sag.py's and this tool's study
  terms) is placed at the robot's measured joints in the double support BEFORE THE PREVIOUS single support
  (footoff_sag.py's rule for that step: PLACE_S before its raise, or PLACE_TD_S after the touchdown before it, whichever
  is later), held HOLD_S, then gets the robot's goals tick by tick (zero-order hold, 50 Hz) through the previous single
  support, its touchdown, this double support and this single support to END_S after this touchdown. So the landing
  that comes before the shift is the sim's own, not a placement.
- The sim's roll, roll rate, roll-chain encoders (the servo shafts) and servo torques replace the robot's in a copy of
  the record's tick array over that window (NaN outside it); the trim's error becomes the record's (err - roll) offset
  plus the sim's roll. tip_causes.run_rows then reads it exactly as it reads the robot: ds_roll_peak (the snap),
  lag_mid_* (the lag behind the goals at the shift's fastest part), ds_roll_end, sq_hip_pre, wdev_pre, e_09 ...
- Loads: the sim's servo torque in the joint frame as STS load counts, -tau / stall x 1000 (the STS load register is
  the PWM duty in 0.1 %, signed like pos - goal). This scale is inferred, not calibrated; compare the sim's loads with
  the robot's in sign and ratio, not in counts.
- Its own tip: lean_sim = the largest (trim error toward the swing foot) between the lift-off and the touchdown, the
  firmware's TS lean_in; a tip is >= 7 deg (stance_tips.TIP_DEG) or a fall (torso past 35 deg). On the robot's rows
  the same column, from the record, matches the track's own TS lean_in to 0.05 deg.

--delay V,... (default 0): the goals each step is replayed under (variant()):
  0       as recorded.
  fill    as recorded, with every tick gap of the record longer than 50 ms filled with goals interpolated in time.
          The record's ticks are the control loop's: across a gap its tick counter advances by 1 and nothing is
          dropped, so a gap is a stall of the loop (2026-11-16: ~115 ms, 0.2-0.5 s after each touchdown, while the
          firmware runs the host's `track steps` and `imu` commands). The servos hold their goal through it and the
          goal jumps after it; the zero-order-hold replay does the same, and `fill` takes the stall out.
  D       (s) the shift D later, with every row after the previous touchdown moved D (the touchdown's row held for
          D). The stall moves with the shift.
  Drt     the shift D later with the record's ticks (and so its stall) kept at their real time up to the lift-off:
          the --td-pause counterfactual (on the robot the stall stays 0.2-0.5 s after the touchdown with the pause).
  ss      the --polls-in-ss arm on the old firmware: every stall filled as in `fill`, then a stall of SS_STALL_S in
          every single support, SS_STALL_AFTER_S after its lift-off (the goals held through it).
  cF, cFfill   the trim's correction (goal - nominal) scaled by F everywhere, as recorded / with the stalls filled.
          On a right stance the double support carries the fade of the previous left stance's large correction
          (2026-11-27 record); c0 takes it out.
  In both D variants the previous stance's trim correction (goal - nominal) and the trim's state keep their real time
  over the double support (the fade starts at the touchdown, as on the robot's --td-pause runs). Only the goals
  change; the plant replays them.

Study terms (CPU replay only, none is a plant default; on top of footoff_sag.split_study's x_* terms):
  x_dead50=<ms>, x_tau50=<ms>    the hip rolls' (STS3250) dead time and first-order lag in grade's TypedServo
  x_dead15=<ms>, x_tau15=<ms>    the same for the STS3215s (every other joint, the ankle rolls included)
  x_db50=<deg>, x_db15=<deg>     set-point hysteresis (TypedServo db) on those servos
Results: results/snap_replay/<name>.csv (one row per single support, plant and variant; not committed).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
GATE = os.path.join(ROOT, "experiments", "policy", "weight-transfer-gate")
for p in (HERE, GATE, os.path.join(ROOT, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")
import stance_tips as ST  # noqa: E402
import tip_causes as TC  # noqa: E402

OUT = os.path.join(HERE, "results", "snap_replay")
PLAY = "grade+play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0"
PLANTS = {"grade": "grade", "play_fit": PLAY, "sag": "grade+sag_L=6.4,sag_R=2.9",
          "sag_fric": "grade+sag_L=5.4,sag_R=2.9,fric_hroll=0.10,fric_aroll=0.10"}
PLACE_S, PLACE_TD_S, HOLD_S, END_S = 2.5, 0.4, 1.0, 0.3
SS_STALL_AFTER_S, SS_STALL_S = 0.06, 0.115     # variant 'ss': --polls-in-ss's earliest poll, the stall's median length
ROLLS = ("L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll")
CHAIN_KEYS = ("x_dead50", "x_tau50", "x_dead15", "x_tau15", "x_db50", "x_db15")
BAL = slice(1, 7)          # the record's balance block: state, side, ref, err, integral, u


# ------------------------------------------------------------------------------------------------
# pure parts (tests/test_snap_replay.py)
# ------------------------------------------------------------------------------------------------
def ffill(x):
    """Forward fill NaNs down axis 0 (a leading NaN takes the first finite value)."""
    x = np.array(x, float)
    for j in range(x.shape[1]):
        c = x[:, j]
        ok = np.isfinite(c)
        if not ok.any():
            continue
        idx = np.where(ok, np.arange(len(c)), 0)
        np.maximum.accumulate(idx, out=idx)
        c = c[idx]
        c[:np.argmax(ok)] = c[np.argmax(ok)]
        x[:, j] = c
    return x


def goal_cols(a, field="goal"):
    """The record's raw goal (or nominal) steps per LEG joint, (n, 12); a raw 0 (no goal written yet, or torque off
    after the walk) is NaN and forward filled."""
    g = np.stack([a[:, TC.col(n, field)] for n in TC.LEG], 1).astype(float)
    g[g == 0] = np.nan
    return ffill(g)


def delay_shift(a, i_td, i_lift, delay, dt=0.02):
    """A copy of the tick array `a` with the double support after the touchdown at row i_td stretched by `delay` s:
    the touchdown's row is held for `delay` (copies every dt), every later row moves `delay` later, and over
    (touchdown, lift-off + delay) the trim's correction (goal - nominal, every joint) and the balance block keep their
    real time (the fade starts at the touchdown; at the moved lift-off they hold their value just before the real
    one). Returns (b, the moved lift-off's row index)."""
    t = a[:, 0]
    td, tl = t[i_td], t[i_lift]
    nins = int(round(delay / dt))
    ins = np.repeat(a[i_td:i_td + 1], nins, 0)
    ins[:, 0] = td + dt * np.arange(1, nins + 1)
    post = a[i_td + 1:].copy()
    post[:, 0] += delay
    b = np.concatenate([a[:i_td + 1], ins, post])
    tb = b[:, 0]
    w = (tb > td) & (tb < tl + delay - 1e-9)
    tr = np.minimum(tb[w], tl - 1e-3)                  # the real time these rows stand for
    j = np.clip(np.searchsorted(t, tr, side="right") - 1, 0, len(a) - 1)
    for n in TC.LEG:
        cg, cn = TC.col(n, "goal"), TC.col(n, "nominal")
        res = a[j, cg] - a[j, cn]
        b[np.where(w)[0], cg] = b[w, cn] + res
    b[np.where(w)[0], BAL] = a[j, BAL]
    return b, i_lift + nins


def delay_shift_rt(a, i_td, i_lift, delay, dt=0.02):
    """delay_shift with the record's tick times kept up to the lift-off (so a control-loop stall stays where it was
    after the touchdown, as it does on the robot with --td-pause): over (touchdown, lift-off + delay) the nominal is
    the record's `delay` s earlier (interpolated in time, held at the touchdown's), the correction and the balance
    block keep their real time; rows from the lift-off on move `delay` later. Returns (b, the moved lift-off's row)."""
    t = a[:, 0]
    td, tl = t[i_td], t[i_lift]
    nins = int(round(delay / dt))
    ins = np.repeat(a[i_lift - 1:i_lift], nins, 0)
    ins[:, 0] = tl + dt * np.arange(nins)
    post = a[i_lift:].copy()
    post[:, 0] += delay
    b = np.concatenate([a[:i_lift], ins, post])
    tb = b[:, 0]
    w = np.where((tb > td) & (tb < tl + delay - 1e-9))[0]
    tr = np.minimum(tb[w], tl - 1e-3)
    j = np.clip(np.searchsorted(t, tr, side="right") - 1, 0, len(a) - 1)
    tn = np.maximum(tb[w] - delay, td)
    for n in TC.LEG:
        cg, cn = TC.col(n, "goal"), TC.col(n, "nominal")
        nom = np.interp(tn, t, a[:, cn])
        b[w, cn] = nom
        b[w, cg] = nom + (a[j, cg] - a[j, cn])
    b[w, BAL] = a[j, BAL]
    return b, i_lift + nins


def fill_gaps(a, max_dt=0.05, dt=0.04):
    """A copy of `a` with every tick gap longer than max_dt filled with rows every dt: the goals and nominals
    interpolated in time, everything else held. Replayed, the goals then move on through a stall of the record
    (the control loop's) instead of holding and jumping."""
    t = a[:, 0]
    parts = [a[:1]]
    for i in range(1, len(a)):
        g = t[i] - t[i - 1]
        if g > max_dt:
            k = int(np.floor(g / dt))
            ts_ = t[i - 1] + dt * np.arange(1, k + 1)
            ts_ = ts_[ts_ < t[i] - 1e-6]
            if len(ts_):
                rows = np.repeat(a[i - 1:i], len(ts_), 0)
                rows[:, 0] = ts_
                f = (ts_ - t[i - 1]) / g
                for n in TC.LEG:
                    for fld in ("goal", "nominal"):
                        c = TC.col(n, fld)
                        rows[:, c] = a[i - 1, c] + f * (a[i, c] - a[i - 1, c])
                parts.append(rows)
        parts.append(a[i:i + 1])
    return np.concatenate(parts)


def scale_correction(a, f):
    """A copy of `a` with the trim's correction (goal - nominal, every joint) scaled by f everywhere."""
    b = a.copy()
    for n in TC.LEG:
        cg, cn = TC.col(n, "goal"), TC.col(n, "nominal")
        b[:, cg] = a[:, cn] + f * (a[:, cg] - a[:, cn])
    return b


def stall_in_ss(a, after_s=SS_STALL_AFTER_S, length_s=SS_STALL_S):
    """The --polls-in-ss arm on the old firmware: fill_gaps(a) (no stall in any double support), then a stall of
    length_s in every single support, after_s after its lift-off (the rows there are dropped, so the replay holds the
    goal through it and the goal jumps after it, as at a recorded stall)."""
    b = fill_gaps(a)
    t = b[:, 0]
    drop = np.zeros(len(b), bool)
    for i0, _, _ in ST.chains(b[:, 1]):
        drop |= (t > t[i0] + after_s) & (t < t[i0] + after_s + length_s)
    return b[~drop]


def variant(a, i_td, i_lift, v):
    """The goals a step is replayed under: '0' as recorded; '<D>' the shift D s later with everything after the
    touchdown moved (delay_shift); '<D>rt' the shift D s later with the stall at its real time (delay_shift_rt);
    'fill' as recorded with the stalls filled (fill_gaps); 'ss' the stalls moved from the double supports into the
    single supports (stall_in_ss); 'c<F>' / 'c<F>fill' the trim's correction scaled by F (scale_correction), as
    recorded / with the stalls filled."""
    v = str(v)
    if v == "fill":
        return fill_gaps(a)
    if v == "ss":
        return stall_in_ss(a)
    if v.startswith("c"):
        fill = v.endswith("fill")
        b = scale_correction(a, float(v[1:-4] if fill else v[1:]))
        return fill_gaps(b) if fill else b
    if v.endswith("rt"):
        return delay_shift_rt(a, i_td, i_lift, float(v[:-2]))[0]
    d = float(v)
    return a if d <= 0 else delay_shift(a, i_td, i_lift, d)[0]


def window(t, ch, n):
    """(t_place, t_end) of step n's replay (1-based; ch = stance_tips.chains): placed in the double support before
    the previous single support, run to END_S after this one's touchdown (or 2.5 s after its lift-off)."""
    i0, _, i_td = ch[n - 1]
    t_end = (t[i_td] + END_S) if i_td is not None else t[i0] + 2.5
    if n >= 2:
        j0 = ch[n - 2][0]
        lo = t[ch[n - 3][2]] + PLACE_TD_S if n >= 3 and ch[n - 3][2] is not None else t[0] + 0.5
        return max(t[j0] - PLACE_S, lo), t_end
    return max(t[i0] - PLACE_S, t[0] + 0.5), t_end


def lean_in(t, err, tl, td, sign):
    """The largest trim error toward the swing foot between lift-off and touchdown (the firmware's TS lean_in)."""
    m = (t >= tl) & (t <= td) & np.isfinite(err)
    return float(np.max(sign * err[m])) if m.any() else float("nan")


def split_chain(spec):
    """(spec without this tool's chain study terms, {term: value})."""
    name, plus, terms = spec.partition("+")
    keep, study = [], {}
    for tok in terms.split(","):
        k, _, v = tok.partition("=")
        if k.strip() in CHAIN_KEYS:
            study[k.strip()] = float(v)
        elif tok.strip():
            keep.append(tok.strip())
    return name + (("+" + ",".join(keep)) if keep else ""), study


def apply_chain(env, study):
    if not study:
        return
    import simh
    sv = env._gate_chain.sv
    is50 = simh.IS3250
    for typ, mask in (("50", is50), ("15", ~is50)):
        if f"x_dead{typ}" in study:
            sv.dead = np.where(mask, study[f"x_dead{typ}"] / 1000.0, sv.dead)
        if f"x_tau{typ}" in study:
            sv.tau = np.where(mask, study[f"x_tau{typ}"] / 1000.0, sv.tau)
        if f"x_db{typ}" in study:
            sv.db = np.where(mask, math.radians(study[f"x_db{typ}"]), sv.db)
    sv.n = int(np.ceil(sv.dead.max() / 0.02)) + 2


# ------------------------------------------------------------------------------------------------
# the replay
# ------------------------------------------------------------------------------------------------
def sim_window(spec, b, zero, dir_, t_place, t_end):
    """Replay b's goals on the plant over [t_place, t_end]. Returns (T, roll deg, roll rate deg/s, enc deg (n, 12)
    LEG order joint frame, load counts (n, 12) joint frame, fell) or None."""
    import footoff_sag as FS
    import rec
    import simh
    import stepper_replay as SR
    import gate_common as GC
    base, study = FS.split_study(spec)
    base, chain = split_chain(base)
    pol = list(rec.POL)
    t = b[:, 0]
    gdeg = np.stack([TC.joint_deg(c, zero[k], dir_[k]) for k, c in enumerate(goal_cols(b).T)], 1)
    graw = np.radians(gdeg)[:, [TC.LEG.index(n) for n in pol]]
    pos = np.stack([TC.joint_deg(b[:, TC.col(n, "pos")], zero[k], dir_[k]) for k, n in enumerate(TC.LEG)], 1)
    pos = np.radians(pos)[:, [TC.LEG.index(n) for n in pol]]
    i0 = int(np.searchsorted(t, t_place))
    ok = np.where(np.all(np.isfinite(pos), 1))[0]
    if not len(ok):
        return None
    q0 = pos[ok[np.argmin(np.abs(ok - i0))]]
    _, env = SR.build(base, (t_end - t_place) + HOLD_S + 5.0)
    FS.apply_study(env, study)
    apply_chain(env, chain)
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), HOLD_S)
    d = env.data
    cur = {"t": graw[i0].copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    names = list(env._servo_names)
    sidx = [names.index(n) for n in TC.LEG]
    stall = np.asarray(env._servo[2] * env._stall_prof, float) * np.ones(len(names))
    n = int(round((t_end - t_place) / env.control_dt))
    T, R, Q, L = [], [], [], []
    fell = False
    for j in range(n):
        tk = t_place + (j + 1) * env.control_dt
        i = max(int(np.searchsorted(t, tk, side="right")) - 1, 0)
        cur["t"] = graw[i]
        obs = env.step(a0)[0]
        up = GC.up_of(d.qpos[3:7])
        T.append(tk)
        R.append(-math.degrees(math.asin(max(-1.0, min(1.0, up[1])))))
        q = obs[:env._nq_act].astype(float)
        Q.append(np.degrees([q[pol.index(nm)] for nm in TC.LEG]))
        tau = np.asarray(env._servo_tau, float)
        L.append((-tau / stall * 1000.0)[sidx])
        if math.degrees(math.acos(max(-1.0, min(1.0, up[2])))) > 35.0:
            fell = True
            break
    T, R, Q, L = map(np.array, (T, R, Q, L))
    rate = np.gradient(R, T) if len(T) > 2 else np.zeros_like(R)
    return T, R, rate, Q, L, fell


def splice(b, zero, dir_, sim):
    """b with the sim's roll, rate, trim error, encoders and loads over the sim's window, NaN outside it."""
    T, R, rate, Q, L, _ = sim
    c = b.copy()
    t = c[:, 0]
    w = (t >= T[0]) & (t <= T[-1])
    off = b[:, 4] - b[:, 7]                    # the record's trim error minus its roll (the reference, filtered)
    roll = np.where(w, np.interp(t, T, R), np.nan)
    c[:, 7] = roll
    c[:, 8] = np.where(w, np.interp(t, T, rate), np.nan)
    c[:, 4] = off + roll
    for k, n in enumerate(TC.LEG):
        deg = np.interp(t, T, Q[:, k])
        c[:, TC.col(n, "pos")] = np.where(w, deg * dir_[k] / TC.DEG_PER_STEP + zero[k], np.nan)
        c[:, TC.col(n, "load")] = np.where(w, np.interp(t, T, L[:, k]) * dir_[k], np.nan)
    return c


def rows_for(a, zero, dir_, st, n):
    """tip_causes.run_rows on the array `a` (instead of the cache), row of single support n."""
    load0 = TC.load
    TC.load = lambda st_: (a, zero, dir_)
    try:
        rows = TC.run_rows(st)
    finally:
        TC.load = load0
    for r in rows:
        if r["n"] == n:
            return r
    return None


def job(args):
    """(stamp, plant label, spec, delay, sides) -> rows (robot rows when label == 'robot')."""
    st, label, spec, delay, sides = args
    a, zero, dir_ = TC.load(st)
    if not len(a):
        return []
    ch = ST.chains(a[:, 1])
    out = []
    track = [json.loads(line) for line in open(ST.track_path(st)) if line.strip()]
    ts, _ = ST.parse_ts(track)
    count = {"L": 0, "R": 0}
    for n in sorted(ts):
        side = ts[n]["side"]
        count[side] += 1
        if side not in sides or count[side] < 2 or n - 1 >= len(ch) or n < 2 or ch[n - 2][2] is None:
            continue
        i0, _, i_td = ch[n - 1]
        sign = 1.0 if side == "L" else -1.0
        b = variant(a, ch[n - 2][2], i0, delay)
        chb = ST.chains(b[:, 1])
        if label == "robot":
            c, fell = b, False
        else:
            t_place, t_end = window(b[:, 0], chb, n)
            sim = sim_window(spec, b, zero, dir_, t_place, t_end)
            if sim is None:
                continue
            c, fell = splice(b, zero, dir_, sim), sim[5]
        try:
            r = rows_for(c, zero, dir_, st, n)
        except ValueError as ex:     # the sim fell before this step's lift-off: nothing of the step to read
            print(f"{st} n {n} {label} {delay}: skipped ({ex})", file=sys.stderr)
            continue
        if r is None:
            continue
        tb = c[:, 0]
        i_tdb = chb[n - 1][2]
        td = tb[i_tdb] if i_tdb is not None else tb[chb[n - 1][0]] + 2.5
        r.update(plant=label, spec=spec, delay=delay, fell=int(fell),
                 lean_sim=lean_in(tb, c[:, 4], tb[chb[n - 1][0]], td, sign))
        r["tip_sim"] = int(r["lean_sim"] >= ST.TIP_DEG or fell)
        out.append(r)
    return out


def stamps_for(day, t_from, t_to, timing=None):
    st = ST.stamps_for(day, t_from, t_to)
    return [s for s in st if os.path.exists(os.path.join(TC.CACHE, f"{s}.npz"))]


def run(stamps, plants, delays, sides, workers, name):
    jobs = [(st, "robot", "", dl, sides) for st in stamps for dl in delays if dl == "0"]
    jobs += [(st, lab, spec, dl, sides) for st in stamps for lab, spec in plants.items() for dl in delays]
    rows = []
    # max_tasks_per_child: each step builds its own plant, and a built plant is not all freed (~50 MB a step)
    import multiprocessing as mp
    with mp.get_context("spawn").Pool(workers, maxtasksperchild=3) as pool:
        for res in pool.imap_unordered(job, jobs, chunksize=1):
            rows += res
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{name}.csv")
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows([{k: round(v, 4) if isinstance(v, float) else v for k, v in r.items()} for r in rows])
    print(f"{len(rows)} rows -> {os.path.relpath(path, ROOT)}")
    return path


# ------------------------------------------------------------------------------------------------
# the report
# ------------------------------------------------------------------------------------------------
def fnum(r, k):
    try:
        return float(r.get(k, "nan"))
    except (TypeError, ValueError):
        return float("nan")


def med(rows, k):
    v = [fnum(r, k) for r in rows]
    v = [x for x in v if np.isfinite(x)]
    return float(np.median(v)) if v else float("nan")


def vkey(v):
    """A variant as written in the CSV: '0', '0.3', '0.3rt', 'fill' (0.0 -> '0')."""
    v = str(v)
    try:
        return "0" if float(v) == 0 else f"{float(v):g}"
    except ValueError:
        return v


def report(paths, side="L"):
    rows = []
    for p in paths:
        rows += TC.read_csv(p)
    keys = sorted({(r["plant"], vkey(r["delay"])) for r in rows}, key=lambda x: (x[0] != "robot", x[0], x[1]))
    rob = {(r["stamp"], int(float(r["n"]))): r for r in rows if r["plant"] == "robot"}
    F = ("ds_roll_peak", "ds_gyro_peak", "lag_mid_L_hip", "lag_mid_R_hip", "lag_mid_L_ank", "ds_roll_end",
         "sq_hip_pre", "wdev_pre", "ld_R_hip_pre", "e_09", "shift_t10")
    print(f"{side} stances, k >= 2; medians; snap = ds_roll_peak >= {TC.SNAP_DEG} deg; tip_sim = lean >= 7 or fell; "
          f"early = shift_t10 < {TC.LAG_S} s")
    for timing in ("normal", "fast"):
        for start in ("early", "late"):
            print(f"\n{timing} timing, {start} shift start (as recorded)")
            print("  plant          delay    n  snap%  tips_sim tips_rob " + " ".join(f"{k[:12]:>12}" for k in F))
            for lab, dl in keys:
                sel = []
                for r in rows:
                    if r["plant"] != lab or vkey(r["delay"]) != dl or r["side"] != side or r["timing"] != timing:
                        continue
                    ro = rob.get((r["stamp"], int(float(r["n"]))))
                    if ro is None:
                        continue
                    early = fnum(ro, "shift_t10") < TC.LAG_S
                    if early != (start == "early"):
                        continue
                    sel.append(r)
                if not sel:
                    continue
                snap = np.mean([fnum(r, "ds_roll_peak") >= TC.SNAP_DEG for r in sel]) * 100
                tips = sum(int(float(r.get("tip_sim") or 0)) for r in sel) if lab != "robot" else 0
                trob = sum(int(float(r["tip"])) for r in sel)
                print(f"  {lab:<14} {dl:>5} {len(sel):>4} {snap:>5.0f}  {tips:>8} {trob:>8} "
                      + " ".join(f"{med(sel, k):>12.2f}" for k in F))
    # step by step: does the sim snap on the robot's snap steps?
    print("\nstep by step, normal early-start L k >= 2 (delay 0): the sim's snap against the robot's")
    for lab, dl in keys:
        if lab == "robot" or dl != "0":
            continue
        pr, ps = [], []
        for r in rows:
            if r["plant"] != lab or vkey(r["delay"]) != "0" or r["side"] != side or r["timing"] != "normal":
                continue
            ro = rob.get((r["stamp"], int(float(r["n"]))))
            if ro is None or not fnum(ro, "shift_t10") < TC.LAG_S:
                continue
            pr.append(fnum(ro, "ds_roll_peak"))
            ps.append(fnum(r, "ds_roll_peak"))
        pr, ps = np.array(pr), np.array(ps)
        ok = np.isfinite(pr) & np.isfinite(ps)
        if ok.sum() < 5:
            continue
        rs = np.corrcoef(pr[ok], ps[ok])[0, 1]
        sr, ss = pr[ok] >= TC.SNAP_DEG, ps[ok] >= TC.SNAP_DEG
        print(f"  {lab:<14} n {ok.sum()}: r {rs:+.2f}; robot snap & sim snap {np.sum(sr & ss)}, robot only "
              f"{np.sum(sr & ~ss)}, sim only {np.sum(~sr & ss)}, neither {np.sum(~sr & ~ss)}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("stamps", nargs="*")
    r.add_argument("--day", action="append", default=[])
    r.add_argument("--from", dest="t_from", default="0000")
    r.add_argument("--to", dest="t_to", default="2359")
    r.add_argument("--plants", default=None, help="'name=SPEC;...' (default: grade, play_fit, sag, sag_fric)")
    r.add_argument("--delay", default="0", help="comma list of variants (variant()): 0, 0.3, 0.3rt, fill")
    r.add_argument("--side", default="L")
    r.add_argument("--workers", type=int, default=6)
    r.add_argument("--name", default="snap")
    p = sub.add_parser("report")
    p.add_argument("csv", nargs="+")
    p.add_argument("--side", default="L")
    a = ap.parse_args(argv)
    if a.cmd == "report":
        report(a.csv, a.side)
        return
    stamps = list(a.stamps)
    for d in a.day:
        stamps += stamps_for(d, a.t_from, a.t_to)
    if not stamps:
        ap.error("no runs (tip_causes.py extract first)")
    plants = PLANTS if a.plants is None else dict(
        (kv.split("=", 1)[0].strip(), kv.split("=", 1)[1].strip()) for kv in a.plants.split(";") if kv.strip())
    run(stamps, plants, [x.strip() for x in a.delay.split(",")], a.side, a.workers, a.name)


if __name__ == "__main__":
    main()
