"""The foot-off sag (#297): recorded track walks replayed through the sim plant with the sag term off
and on, against the robot's IMU around each foot-off.

    python footoff_sag.py extract STAMP [STAMP ...]          # cache the robot's side (build/footoff-sag/)
    python footoff_sag.py run [--runs A,B] [--plants 'off=SPEC;on=SPEC'] [--out DIR] [--workers N]

A run is a `track_host.py` log, `track_<STAMP>.jsonl` (STAMP = the host's local start time,
20261116T151416), looked up in experiments/roll-balance/results/ of this checkout and of every
~/code/robot*/ worktree ($FOOTOFF_TRACK_DIRS: more directories, colon separated). The robot's side
comes from the flight record that covers it (hw_sessions/*/flight_*.frec, the newest one created
before the run; only the run's window is decoded): per tick the written goals (forward filled), the
encoders, the IMU `up`; the trim's state edges (balance::State OFF -> TRIM_CLOSED at the raise,
TRIM_HOLD, TRIM_FADE at touchdown) give each single support: its stance side, t0 (the raise starts),
td (touchdown). The run's t = 0 is the `track play` command in the record.

The replay (as experiments/policy/weight-transfer-gate/stepper_replay.py): the plant (a
stepper_replay.build spec: `grade+<gate_common terms>`) is placed at the robot's measured joints 0.5 s
before the play and held 1.5 s, then its servos get the robot's written goals tick by tick (50 Hz,
zero-order hold). Nothing closes a loop: the trim, the lift gate and the ankle feed-forward are in
the goals the robot wrote, so the sim answers "what would this body have done under those goals".
A spec may also carry this tool's study terms (x_kd, x_damp, x_ank_L/R, x_k_L/R, x_fl_L/R;
split_study): CPU replay only, none is a plant default.

Per single support (t from its t0; s = +1 on a left stance, -1 on a right one):
  imu(t)   = s (roll(t) - roll at t0)            roll toward the lifted foot, deg (roll = -asin(up_y))
  kin(t)   = -s (enc(t) - enc at t0)             the same from the stance chain's encoders,
                                                 enc = hip roll + ankle roll (the sim's: its encoder
                                                 observation, the servo shaft under the roll play)
  delta(t) = imu - kin                           the passive rotation beyond the encoders: the sag
Per run and stance side, over the steps after the first that did not tip (median trace):
  sag      delta's mean over 0.9-1.3 s (cut 0.1 s before the median touchdown)
  t90      the first time delta reaches 0.9 x sag
  f_ring, zeta   a damped sine fitted to delta from its first peak to 0.05 s before touchdown
  DS lean  imu's change over the last 1 s of the double support before the raise (the end of
           the weight shift onto the stance foot), median over the same steps
  tips     steps whose imu peaks >= 7 deg (the track's TS lean_in tip line) before touchdown, of all
           steps; fell: torso tilt past 35 deg
Figures: one PNG per run (delta and imu, L and R stance; robot median and p10-p90, each plant);
results: <out>/summary.md and summary.json.
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import math
import os
import struct
import sys
from concurrent.futures import ProcessPoolExecutor
from zoneinfo import ZoneInfo

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
GATE = os.path.join(ROOT, "experiments", "policy", "weight-transfer-gate")
for p in (HERE, GATE, os.path.join(ROOT, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
import gate_common as GC  # noqa: E402

JOINTS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
          "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
CACHE = os.path.join(ROOT, "build", "footoff-sag", "cache")
OUT = os.path.join(HERE, "results", "footoff_sag")
LOCAL = ZoneInfo("America/Denver")      # the host that ran track_host.py names its logs in local time
# the morning grading's plants (grade.py's plant + GRADE_PLANT_TERMS): the refit play, the inset sole
# on it, and the foot-off sag (which replaces the play; the sole is the plant's own)
PLAY = "grade+play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0"
PLANTS = {"play": PLAY, "inset": PLAY + ",sole_in_L=8,sole_in_R=-12", "sag": "grade+sag_L=6.4,sag_R=2.9"}
# three dwell-gait runs of 2026-11-16 (a march, a forward 40 mm walk, Variant M's faster timing) and
# the three quick single supports of that afternoon that tipped (--ss-s 1.2; raise 0.42 s; raise
# 0.42 s with the pre-hike)
RUNS = ["20261116T152118", "20261116T150059", "20261116T174820", "20261116T150430", "20261116T151416",
        "20261116T152225"]
PRE_S = 0.5
PLACE_S = 2.5        # a step's replay starts this long before its raise (before its weight shift),
PLACE_TD_S = 0.4     # ... or this long after the touchdown before it, whichever is later,
HOLD_S = 1.0         # ... after the plant is held this long at the robot's measured joints there
TIP_DEG = 7.0
FALL_DEG = 35.0
TG = np.round(np.arange(-0.6, 2.4001, 0.02), 3)
SAG_WIN = (0.9, 1.3)
FREC_MAGIC = b"FRECSESS1\n"
REC = struct.Struct("<HQ")


# --------------------------------------------------------------------------------------------
# the robot's side
# --------------------------------------------------------------------------------------------
def track_path(stamp):
    dirs = [os.path.join(HERE, "results")]
    dirs += sorted(glob.glob(os.path.expanduser("~/code/robot*/experiments/roll-balance/results")))
    dirs += [d for d in os.environ.get("FOOTOFF_TRACK_DIRS", "").split(":") if d]
    for d in dirs:
        p = os.path.join(d, f"track_{stamp}.jsonl")
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"track_{stamp}.jsonl")


def load_track(stamp):
    rows = [json.loads(line) for line in open(track_path(stamp)) if line.strip()]
    summ = rows[-1] if rows and rows[-1].get("summary") else {}
    return rows, summ


def stamp_utc(stamp):
    return dt.datetime.strptime(stamp, "%Y%m%dT%H%M%S").replace(tzinfo=LOCAL).timestamp()


def frec_meta(path):
    with open(path, "rb") as f:
        if f.read(len(FREC_MAGIC)) != FREC_MAGIC:
            return None
        return json.loads(f.readline())


def frec_for(t_utc):
    """The newest flight record created before t_utc."""
    best = None
    for p in glob.glob(os.path.join(GC.HW, "*", "flight_*.frec")):
        m = frec_meta(p)
        if not m or "created_utc" not in m:
            continue
        c = dt.datetime.fromisoformat(m["created_utc"]).timestamp()
        if c <= t_utc and (best is None or c > best[0]):
            best = (c, p)
    return best[1] if best else None


def read_window(path, t0, t1):
    """Decode the packets of `path` whose host receive time lies in [t0, t1] (UTC s)."""
    import flightrec as F
    ticks, events, hello = [], [], []
    with open(path, "rb") as f:
        f.read(len(FREC_MAGIC))
        f.readline()
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                break
            n, rx = REC.unpack(h)
            t = rx / 1e6
            if t < t0 - 2.0:
                f.seek(n, 1)
                continue
            if t > t1 + 2.0:
                break
            data = f.read(n)
            try:
                p = F.decode(data)
            except F.DecodeError:
                continue
            p["rx"] = t
            p["t"] = p["t_utc_us"] / 1e6 if p["t_utc_us"] else t
            if p["type"] == F.T_TICK:
                ticks.append(p)
            elif p["type"] == F.T_EVENT:
                events.append(p)
            else:
                hello.append(p)
    return ticks, events, hello


def extract(stamp):
    """Cache one run's robot side (npz in build/footoff-sag/cache)."""
    import flightrec as F
    rows, summ = load_track(stamp)
    t_start = stamp_utc(stamp)
    t_end = t_start + max(r.get("t", 0.0) for r in rows[:-1]) + 3.0
    path = frec_for(t_start)
    if path is None:
        raise FileNotFoundError(f"no flight record before {stamp}")
    ticks, events, hello = read_window(path, t_start - 3.0, t_end)
    if not ticks:
        raise RuntimeError(f"{stamp}: {os.path.basename(path)} has no ticks in the run's window")
    ticks.sort(key=lambda p: p["t"])
    events.sort(key=lambda p: p["t"])
    play = [e["t"] for e in events if e["code"] == F.EV_CMD and e["text"].startswith("track play")]
    if not play:
        raise RuntimeError(f"{stamp}: no `track play` in the record")
    tu0 = play[0]
    cal = {}
    for h in hello:
        cal[h["boot_id"]] = (h["zero"], h["dir"])
    bus = [F.BUS_JOINT_NAMES.index(n) for n in JOINTS]
    nt = len(ticks)
    t = np.array([p["t"] for p in ticks]) - tu0
    goal = np.full((nt, len(JOINTS)), np.nan)
    pos = np.full((nt, len(JOINTS)), np.nan)
    up = np.array([p["up"] for p in ticks], float)
    gyro = np.array([p["gyro"] for p in ticks], float)
    bstate = np.array([p["bal"]["state"] for p in ticks])
    bside = np.array([p["bal"]["side"] for p in ticks])
    for i, p in enumerate(ticks):
        z, d = cal.get(p["boot_id"], ([2048] * F.MAX_JOINTS, [1] * F.MAX_JOINTS))
        js = {j["bus"]: j for j in p["joints"]}
        for k, b in enumerate(bus):
            j = js.get(b)
            if j is None:
                continue
            if j["flags"] & F.JF_GOAL_VALID:
                goal[i, k] = F.steps_to_rad(j["goal"], z[b], d[b])
            if j["flags"] & F.JF_FB_OK:
                pos[i, k] = F.steps_to_rad(j["pos"], z[b], d[b])
    g = goal.copy()                     # forward fill; before the first written goal, the encoder
    for k in range(g.shape[1]):
        last = pos[0, k]
        for i in range(nt):
            if np.isfinite(g[i, k]):
                last = g[i, k]
            else:
                g[i, k] = last
    # single supports from the trim's state edges (balance::State: 0 OFF, 5 CLOSED, 6 HOLD, 7 FADE,
    # 8 / 9 aborts)
    ch, cur = [], None
    for e in events:
        if e["code"] != F.EV_BAL_STATE:
            continue
        a, b, te = e["a"], e["b"], e["t"] - tu0
        if a == 0 and b == 5:
            i = min(int(np.searchsorted(t, te)) + 1, nt - 1)
            cur = dict(t0=te, side=chr(int(bside[i])) if bside[i] in (76, 82) else "?")
            ch.append(cur)
        elif cur is None:
            continue
        elif a == 5 and b == 6:
            cur["tl"] = te
        elif a == 6 and b == 7:
            cur["td"] = te
        elif a == 7 and b == 0:
            cur["te"] = te
            cur = None
        elif b in (8, 9):
            cur["abort"] = te
    falls = [e["t"] - tu0 for e in events if e["code"] == F.EV_FALL]
    os.makedirs(CACHE, exist_ok=True)
    np.savez_compressed(os.path.join(CACHE, stamp + ".npz"), t=t, goal=g, pos=pos, up=up, gyro=gyro,
                        bstate=bstate, chains=json.dumps(ch), falls=np.array(falls), tu0=tu0,
                        frec=os.path.relpath(path, GC.MAIN), note=summ.get("note", ""),
                        args=json.dumps(summ.get("args", {})), steps=json.dumps(summ.get("steps", [])))
    print(f"{stamp} {summ.get('note', '')}: {nt} ticks from {os.path.basename(path)}, {len(ch)} single "
          f"supports, falls {falls}", flush=True)


def robot(stamp):
    p = os.path.join(CACHE, stamp + ".npz")
    if not os.path.exists(p):
        extract(stamp)
    z = np.load(p, allow_pickle=False)
    r = {k: z[k] for k in z.files}
    for k in ("chains", "args", "steps"):
        r[k] = json.loads(str(r[k]))
    r["note"] = str(r["note"])
    r["frec"] = str(r["frec"])
    up = r["up"]
    r["roll"] = -np.degrees(np.arcsin(np.clip(up[:, 1], -1, 1)))
    r["tilt"] = np.degrees(np.arccos(np.clip(up[:, 2], -1, 1)))
    return r


def window(r):
    """(t_from, t_to) of the replay: 0.5 s before the play to 2.5 s after the last touchdown (or the
    run's last tick)."""
    ch = r["chains"]
    t_last = max([c.get("te", c.get("td", c["t0"] + 2.0)) for c in ch] or [r["t"][-1] - 3.0])
    return -PRE_S, min(t_last + 2.5, float(r["t"][-1]))


# --------------------------------------------------------------------------------------------
# the sim's side
# --------------------------------------------------------------------------------------------
STUDY_KEYS = ("x_kd", "x_damp", "x_ank_L", "x_ank_R", "x_k_L", "x_k_R", "x_fl_L", "x_fl_R")


def split_study(spec):
    """(spec without the study terms, {study term: value}). The study terms (CPU, this tool only;
    none is a plant default anywhere), on the four roll servos:
      x_kd=<x>            x the servo kd (the PD's damping as the link sees it)
      x_damp=<N m s/rad>  the roll joints' viscous damping (MuJoCo dof_damping; the plant's is 0.1)
      x_ank_L=, x_ank_R=<0..1>   that side's share of the foot-off sag preload at the ankle roll
                          (walker_env FOOTOFF_SAG_ANKLE_SHARE, 0.3; the rest at the hip)
      x_k_L=, x_k_R=<N m/rad>    that side's foot-off sag springs (FOOTOFF_SAG_K, 60)
      x_fl_L=, x_fl_R=<N m>      that side's hip- and ankle-roll dry friction (MuJoCo frictionloss on
                          the link; the plant's is 0.17; gate_common's fric_hroll / fric_aroll set
                          both sides)"""
    name, plus, terms = spec.partition("+")
    keep, study = [], {}
    for tok in terms.split(","):
        k, _, v = tok.partition("=")
        if k.strip() in STUDY_KEYS:
            study[k.strip()] = float(v)
        elif tok.strip():
            keep.append(tok.strip())
    return name + (("+" + ",".join(keep)) if keep else ""), study


def apply_study(env, study):
    if not study:
        return
    m = env.model
    names = env._servo_names
    roll = np.array([n.endswith(("hip_roll", "ankle_roll")) for n in names])
    if "x_kd" in study:
        env._kd_prof = np.where(roll, env._kd_prof * study["x_kd"], env._kd_prof)
    if "x_damp" in study:
        for i in np.where(roll)[0]:
            m.dof_damping[env._jv0 + i] = study["x_damp"]
    for s_ in "LR":
        if f"x_fl_{s_}" in study:
            for i in np.where(roll)[0]:
                if names[i].startswith(s_ + "_"):
                    m.dof_frictionloss[env._jv0 + i] = study[f"x_fl_{s_}"]
    if any(k in study for k in ("x_ank_L", "x_ank_R", "x_k_L", "x_k_R")):
        if getattr(env, "_fs_theta", None) is None:
            raise ValueError("x_ank_* / x_k_* need the foot-off sag (sag_L= / sag_R=)")
        part = np.array(env._fs_part, float)
        k = np.full(len(names), float(np.mean(env._fs_k)))
        for i, n in enumerate(names):
            s_ = n[0]
            a = study.get(f"x_ank_{s_}")
            if a is not None and n.endswith("ankle_roll"):
                part[i] = a
            elif a is not None and n.endswith("hip_roll"):
                part[i] = 1.0 - a
            if f"x_k_{s_}" in study and roll[i]:
                k[i] = study[f"x_k_{s_}"]
        env._fs_part = part
        env._fs_k = k


def simulate(job):
    """(stamp, label, spec) -> the sim's traces under the robot's goals, one replay per single
    support: placed at the robot's measured joints in the double support before it (PLACE_S
    before its raise, or PLACE_TD_S after the touchdown before it, whichever is later; both feet
    down, before the weight shift onto the stance foot) and held HOLD_S, then the robot's goals
    from there to 0.8 s after its touchdown (a step's replay does not inherit the drift of the
    steps before it)."""
    stamp, label, spec = job
    base_spec, study = split_study(spec)
    import rec
    import simh
    import stepper_replay as SR
    r = robot(stamp)
    pol = list(rec.POL)
    jidx = [JOINTS.index(n) for n in pol]
    t = r["t"]
    goal = r["goal"][:, jidx]
    steps = []
    prev_td = -1e9
    for k, c in enumerate(r["chains"]):
        if "td" not in c or c["side"] not in "LR":
            continue
        t_from, t_to = max(c["t0"] - PLACE_S, prev_td + PLACE_TD_S), c["td"] + 0.8
        prev_td = c["td"]
        _, env = SR.build(base_spec, (t_to - t_from) + HOLD_S + 5.0)
        apply_study(env, study)
        i0 = int(np.searchsorted(t, t_from))
        q0 = r["pos"][i0][jidx]
        if not np.all(np.isfinite(q0)):
            continue
        simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), HOLD_S)
        pads = GC.pad_gids(env)
        d = env.data
        cur = {"t": goal[i0].copy()}
        env._action_to_ctrl = lambda a_: cur["t"]
        a0 = np.zeros(env.action_space.shape[0], np.float32)
        n = int(round((t_to - t_from) / env.control_dt))
        T, Q, UP, CL = [], [], [], []
        for j in range(n):
            tk = t_from + (j + 1) * env.control_dt
            i = int(np.searchsorted(t, tk, side="right")) - 1
            gk = goal[max(i, 0)]
            cur["t"] = np.where(np.isfinite(gk), gk, cur["t"])
            obs = env.step(a0)[0]
            T.append(tk)
            Q.append(obs[:env._nq_act].astype(np.float64))
            UP.append(GC.up_of(d.qpos[3:7]))
            cl = GC.clearance(env, pads)
            CL.append((cl["L"], cl["R"]))
        T, Q, UP, CL = map(np.array, (T, Q, UP, CL))
        roll = -np.degrees(np.arcsin(np.clip(UP[:, 1], -1, 1)))
        tilt = np.degrees(np.arccos(np.clip(UP[:, 2], -1, 1)))
        enc = {s_: Q[:, pol.index(f"{s_}_hip_roll")] + Q[:, pol.index(f"{s_}_ankle_roll")] for s_ in "LR"}
        steps.append(dict(k=k + 1, t=T, roll=roll, tilt=tilt, enc=enc, clr=CL))
    return dict(stamp=stamp, label=label, spec=spec, steps=steps)


# --------------------------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------------------------
def step_traces(t, roll, enc, chains):
    """Per single support: imu / kin / delta on TG (deg, toward the lifted foot) and its fields."""
    out = []
    for k, c in enumerate(chains):
        if "td" not in c or c["side"] not in "LR":
            continue
        s = 1.0 if c["side"] == "L" else -1.0
        t0 = c["t0"]
        tt = t0 + TG
        if tt[0] < t[0] - 0.05 or tt[-1] > t[-1] + 1.0:
            continue
        r0 = float(np.interp(t0, t, roll))
        e0 = float(np.interp(t0, t, enc[c["side"]]))
        imu = s * (np.interp(tt, t, roll, right=np.nan) - r0)
        kin = -s * np.degrees(np.interp(tt, t, enc[c["side"]], right=np.nan) - e0)
        ds = s * (r0 - float(np.interp(t0 - 1.0, t, roll)))
        out.append(dict(k=k + 1, side=c["side"], td=c["td"] - t0, imu=imu, kin=kin, delta=imu - kin, ds=ds))
    return out


def damped_fit(t, y):
    """C + A exp(-sigma t) cos(w t + phi) on (t, y) -> (f Hz, zeta, r2) or Nones."""
    from scipy.optimize import least_squares
    ok = np.isfinite(y)
    t, y = t[ok] - t[ok][0], y[ok]
    if len(t) < 12 or np.ptp(y) < 0.2:
        return None, None, None
    best = None
    for f0 in (1.2, 2.0, 3.0, 4.0):
        p0 = [float(np.mean(y)), float(y[0] - np.mean(y)), 1.0, 2 * math.pi * f0, 0.0]
        try:
            res = least_squares(lambda p: p[0] + p[1] * np.exp(-p[2] * t) * np.cos(p[3] * t + p[4]) - y, p0,
                                bounds=([-30, -30, 0.0, 2 * math.pi * 0.8, -math.pi], [30, 30, 40.0, 2 * math.pi * 6.0, math.pi]))
        except ValueError:
            continue
        if best is None or res.cost < best.cost:
            best = res
    if best is None:
        return None, None, None
    sg, w = best.x[2], best.x[3]
    r2 = 1.0 - 2 * best.cost / max(float(np.sum((y - y.mean()) ** 2)), 1e-12)
    return w / (2 * math.pi), sg / math.hypot(sg, w), r2


def side_metrics(steps):
    """Median-trace metrics of one stance side (steps after the first, without a tip)."""
    clean = [x for x in steps if x["k"] > 1 and np.nanmax(x["imu"][(TG >= 0) & (TG <= x["td"])]) < TIP_DEG]
    if not clean:
        return dict(n=0)
    D = np.array([x["delta"] for x in clean])
    I_ = np.array([x["imu"] for x in clean])
    md = np.nanmedian(D, 0)
    td = float(np.median([x["td"] for x in clean]))
    w = (TG >= SAG_WIN[0]) & (TG <= min(SAG_WIN[1], td - 0.1))
    sag = float(np.nanmean(md[w])) if w.any() else float("nan")
    t90 = None
    if np.isfinite(sag) and sag > 0.3:
        hit = np.where((TG >= 0) & (md >= 0.9 * sag))[0]
        t90 = float(TG[hit[0]]) if len(hit) else None
    j = np.where((TG >= 0) & (TG <= min(1.0, td)))[0]
    tp = float(TG[j[np.nanargmax(md[j])]]) if len(j) else 0.0
    wr = (TG >= tp) & (TG <= td - 0.05)
    f, z, r2 = damped_fit(TG[wr], md[wr])
    ds = float(np.nanmedian([x["ds"] for x in clean]))
    return dict(n=len(clean), sag=sag, t90=t90, peak=float(np.nanmax(md[j])) if len(j) else None, t_peak=tp,
                f_ring=f, zeta=z, ring_r2=r2, ds_lean=ds, td=td, delta=md,
                imu=np.nanmedian(I_, 0),
                delta_p10=np.nanpercentile(D, 10, 0), delta_p90=np.nanpercentile(D, 90, 0),
                imu_p10=np.nanpercentile(I_, 10, 0), imu_p90=np.nanpercentile(I_, 90, 0))


def tips(steps):
    return [x["k"] for x in steps if np.nanmax(x["imu"][(TG >= 0) & (TG <= x["td"])]) >= TIP_DEG]


def evaluate(r, sims):
    """-> {who: {"L": side metrics, "R": ..., "tips": [...], "fell": bool, "n": steps}}."""
    out = {}
    enc_r = {s_: r["pos"][:, JOINTS.index(f"{s_}_hip_roll")] + r["pos"][:, JOINTS.index(f"{s_}_ankle_roll")]
             for s_ in "LR"}
    ok = np.isfinite(enc_r["L"]) & np.isfinite(enc_r["R"])
    t_from, t_to = window(r)
    st = step_traces(r["t"][ok], r["roll"][ok], {s_: v[ok] for s_, v in enc_r.items()}, r["chains"])
    wr = (r["t"] >= t_from) & (r["t"] <= t_to)
    out["robot"] = dict(L=side_metrics([x for x in st if x["side"] == "L"]),
                        R=side_metrics([x for x in st if x["side"] == "R"]),
                        tips=tips(st), n=len(st), fell=bool(np.nanmax(r["tilt"][wr]) > FALL_DEG))
    for s in sims:
        ss, fell = [], []
        for x in s["steps"]:
            c = r["chains"][x["k"] - 1]
            one = step_traces(x["t"], x["roll"], x["enc"], [c])
            for y in one:
                y["k"] = x["k"]
            ss += one
            if np.max(x["tilt"]) > FALL_DEG:
                fell.append(x["k"])
        out[s["label"]] = dict(L=side_metrics([x for x in ss if x["side"] == "L"]),
                               R=side_metrics([x for x in ss if x["side"] == "R"]),
                               tips=tips(ss), n=len(ss), fell=fell)
    return out


# --------------------------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------------------------
def fnum(x, f=".1f"):
    return "–" if x is None or (isinstance(x, float) and not np.isfinite(x)) else format(x, f)


def plot(stamp, r, ev, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True)
    colors = {"robot": "k", "play": "tab:green", "inset": "tab:blue", "sag": "tab:red"}
    for j, side in enumerate("LR"):
        for i, (key, name) in enumerate((("delta", "passive roll beyond the encoders (sag)"),
                                         ("imu", "IMU roll toward the lifted foot"))):
            a = ax[i, j]
            for who, m in ev.items():
                sm = m[side]
                if not sm.get("n"):
                    continue
                c = colors.get(who, None)
                lw = 2.2 if who == "robot" else 1.6
                a.plot(TG, sm[key], color=c, lw=lw, label=f"{who} (n={sm['n']})")
                if who == "robot":
                    a.fill_between(TG, sm[key + "_p10"], sm[key + "_p90"], color="0.8", label="robot p10-p90")
            rs = ev["robot"][side]
            if rs.get("n"):
                a.axvspan(0, rs["td"], color="0.93", zorder=0)
            a.axhline(0, color="0.6", lw=0.6)
            a.set_title(f"{side} stance: {name}", fontsize=10)
            a.set_ylabel("deg")
            a.grid(alpha=0.3)
            if i == 1:
                a.set_xlabel("time after the raise starts, s (shaded: to the median touchdown)")
            a.legend(fontsize=8, loc="upper right")
    fig.suptitle(f"{stamp} {r['note']}: robot vs sim plants, clean steps after the first", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def summary_md(res, plants):
    L = ["# Foot-off sag replay (footoff_sag.py)", "",
         "Plants: " + "; ".join(f"`{k}` = `{v}`" for k, v in plants.items()), "",
         "sag / t90 / f_ring / zeta: median trace of the clean steps after the first (module doc); "
         "tips: steps whose roll toward the lifted foot peaked >= 7 deg before touchdown.", "",
         "| run | note | stance | who | n | sag deg | t90 s | peak deg | f_ring Hz | zeta | DS lean deg | tips (step) | fell |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for stamp, x in res.items():
        for side in "LR":
            for who, m in x["ev"].items():
                sm = m[side]
                L.append(f"| {stamp} | {x['note']} | {side} | {who} | {sm.get('n', 0)} | {fnum(sm.get('sag'))} | "
                         f"{fnum(sm.get('t90'), '.2f')} | {fnum(sm.get('peak'))} | {fnum(sm.get('f_ring'), '.2f')} | "
                         f"{fnum(sm.get('zeta'), '.2f')} | {fnum(sm.get('ds_lean'))} | "
                         f"{m['tips'] if side == 'L' else ''} | "
                         f"{(m['fell'] if side == 'L' else '')} |")
    return "\n".join(L) + "\n"


def jsonable(res):
    out = {}
    for stamp, x in res.items():
        out[stamp] = dict(note=x["note"], frec=x["frec"], ev={})
        for who, m in x["ev"].items():
            e = dict(tips=m["tips"], n=m["n"], fell=m["fell"])
            for side in "LR":
                e[side] = {k: v for k, v in m[side].items() if not isinstance(v, np.ndarray)}
            out[stamp]["ev"][who] = e
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract")
    e.add_argument("stamps", nargs="+")
    rr = sub.add_parser("run")
    rr.add_argument("--runs", default=",".join(RUNS))
    rr.add_argument("--plants", default=";".join(f"{k}={v}" for k, v in PLANTS.items()),
                    help="'label=SPEC;label=SPEC' (stepper_replay specs)")
    rr.add_argument("--out", default=OUT)
    rr.add_argument("--workers", type=int, default=6)
    a = ap.parse_args(argv)
    if a.cmd == "extract":
        for s in a.stamps:
            extract(s)
        return 0
    runs = [x for x in a.runs.split(",") if x]
    plants = dict(p.split("=", 1) for p in a.plants.split(";") if p)
    for s in runs:
        robot(s)                                    # extract serially (one reader per record)
    jobs = [(s, k, v) for s in runs for k, v in plants.items()]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        sims = list(ex.map(simulate, jobs))
    os.makedirs(a.out, exist_ok=True)
    res = {}
    for s in runs:
        r = robot(s)
        ev = evaluate(r, [x for x in sims if x["stamp"] == s])
        res[s] = dict(note=r["note"], frec=r["frec"], ev=ev)
        plot(s, r, ev, os.path.join(a.out, f"{s}_{r['note']}.png"))
    md = summary_md(res, plants)
    open(os.path.join(a.out, "summary.md"), "w").write(md)
    json.dump(dict(plants=plants, runs=jsonable(res)), open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
