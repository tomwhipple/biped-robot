"""The robot's plant gain against the x1.5 grade plant's, standing and walking (#137, planner 2026-11-25).

grade.py's `x1.5` / grade_gain.py's walking probes scale the policy's action deviation from its 0.5 s EMA by G before
the servos: a plant G times more responsive than the sim's. G = 1.5 comes from a stability bracket on the 2026-11-02
stand (the robot diverged at x1 where the sim needed x1.5), not from a signal ratio. This tool measures the signal
ratio directly, with no policy in the loop: the robot's WRITTEN goals (flight record, every tick) are replayed open
loop into the grade plant (`bimo_biped_v6legs_meas_sole.xml`, simh.JF_PLANT, the GRADE_PLANT_TERMS foot-off sag,
grade.FAST TypedServo; nothing closes the loop), and the robot's measured response is regressed on the sim's:

    y_robot(t) ~ g * y_sim(t - lag)       on the probe's band (x - EMA(x, 0.5 s), the out_gain law's own filter)
                                          and on 0.3-1 / 1-3 / 3-8 Hz bands (zero-phase Butterworth)

g is H1 with the sim as the input: the robot x sim cross-covariance summed over EVERY window of a data set, at the
one lag (-240..+240 ms) that maximises it, over the sim's summed variance (pooled()). Both sides get the same goals, so
g is |H_robot| / |H_sim| at the goals' spectrum; the sim is noiseless, so whatever the robot does that the sim does not
predict lowers the coherence, not g. CI: resampling whole segments. Channels: the 12 encoders (servo tracking), the
gyro x/y/z (body rate) and the torso roll / pitch (body attitude). Windows: on walks one per single support (stance /
swing leg) and one per double support; on stands fixed 4 s windows; only where both are upright and the sim's attitude
is within 6 deg of the robot's.

The reading is not G itself: the plant is not linear at stand amplitudes (sub-degree goals against friction and
deadband) nor across contact changes. `selftest` calibrates it: the same segments replayed with the out_gain law at a
known G on their goals stand in for the robot, and the tool reads them like the robot (G -> reading per channel).

    python walk_gain.py extract  [--days 20261112,20261113,20261116]     # robot side -> build/walk-gain/robot/
    python walk_gain.py replay   [--workers 8]                           # sim side   -> build/walk-gain/sim/
    python walk_gain.py analyze  [--json out.json] [--md out.md] [--rows rows.pkl | --from-rows rows.pkl]
    python walk_gain.py selftest [--gains 1.25,1.5,2.0] [--n 48]         # the calibration

CPU only (JAX_PLATFORMS=cpu). Robot data: hw_sessions/ of the main checkout (GATE_MAIN), track logs from every
~/code/robot*/experiments/roll-balance/results/track_<stamp>.jsonl.
Record: docs/design-v6/2026-11-26-walking-loop-gain.md.
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

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MAIN = os.environ.get("GATE_MAIN", "/home/claw/code/robot")
HW = os.path.join(MAIN, "hw_sessions")
OUT = os.environ.get("WALK_GAIN_DIR", os.path.join(ROOT, "build", "walk-gain"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "experiments", "policy", "shaper-train"))
sys.path.insert(0, os.path.join(ROOT, "experiments", "policy", "rca"))

# the x1.5 probe's plant (x15-margin/run.sh defaults; the 2026-11-24 record's plant D) and CPU only. Set by main()
# and the replay workers before grade.py is imported (it reads GRADE_XML at import); never at module import, so a
# test that imports this module does not change grade.py for the tests after it.
ENV = dict(JAX_PLATFORMS="cpu", CUDA_VISIBLE_DEVICES="", MUJOCO_GL="disabled",
           GRADE_XML="bimo_biped_v6legs_meas_sole.xml", GRADE_PLANT_TERMS="sag_L=6.4,sag_R=2.9")


def set_env():
    for k, v in ENV.items():
        os.environ.setdefault(k, v)

DT = 0.02
FREC_MAGIC = b"FRECSESS1\n"
REC = struct.Struct("<HQ")
BUS = ("R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
       "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll")
POL = ("L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
       "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll")
B2P = [BUS.index(n) for n in POL]               # bus-order array[:, B2P] -> policy order
GROUPS = {"yaw": ("hip_yaw",), "roll": ("hip_roll", "ankle_roll"), "sag": ("hip_pitch", "knee", "ankle")}
CLOSED, HOLD, FADE = 5, 6, 7
FALL_DEG = 35.0
TAU_MS = np.arange(-60, 121, 20)          # per-window fit (ratio)
LAGS_MS = np.arange(-240, 241, 20)        # pooled fit: one lag per channel class and data set
BANDS = {"probe": None, "0.3-1": (0.3, 1.0), "1-3": (1.0, 3.0), "3-8": (3.0, 8.0)}
STAND_WIN_S = 4.0


def group_of(name):
    j = name[2:]
    return next(g for g, js in GROUPS.items() if j in js)


# ----------------------------------------------------------------------------------------------------------------
# the robot side
# ----------------------------------------------------------------------------------------------------------------
def frec_created(p):
    with open(p, "rb") as f:
        if f.read(len(FREC_MAGIC)) != FREC_MAGIC:
            return None
        m = json.loads(f.readline())
    return dt.datetime.fromisoformat(m["created_utc"]).timestamp() if "created_utc" in m else None


def frec_for(t_utc):
    best = None
    for p in glob.glob(os.path.join(HW, "*", "flight_*.frec")):
        c = frec_created(p)
        if c is not None and c <= t_utc and (best is None or c > best[0]):
            best = (c, p)
    return best[1] if best else None


def read_windows(path, wins):
    """One pass over a flight record: {key: arrays} for every (key, u0, u1) window (robot UTC, s). Only ticks inside
    a window are decoded (and every HELLO, for the calibration). Arrays (bus order): t (tick UTC), tfb (the servo
    read's UTC), goal, pos (rad, NaN where invalid), gyro (rad/s), up, src, flags, bal_state, bal_side, phase."""
    import flightrec as F
    wins = sorted(wins, key=lambda w: w[1])
    lo, hi = min(w[1] for w in wins) - 5.0, max(w[2] for w in wins) + 5.0
    rows = {k: [] for k, _, _ in wins}
    cal = {}
    with open(path, "rb") as f:
        f.read(len(FREC_MAGIC))
        f.readline()
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                break
            n, rx = REC.unpack(h)
            t = rx / 1e6
            if t > hi + 30.0:
                break
            head = f.read(min(n, 4))
            if len(head) < 4:
                break
            ptype = head[3]
            if ptype == F.T_HELLO or (ptype == F.T_TICK and lo <= t <= hi):
                body = f.read(n - 4)
                try:
                    p = F.decode(head + body)
                except F.DecodeError:
                    continue
            else:
                f.seek(n - 4, 1)
                continue
            if p["type"] == F.T_HELLO:
                cal[p["boot_id"]] = (list(p["zero"]), list(p["dir"]))
                continue
            tu = p["t_utc_us"] / 1e6 if p["t_utc_us"] else t
            hit = [k for k, u0, u1 in wins if u0 <= tu <= u1]
            if not hit:
                continue
            pv = np.full(12, np.nan); gv = np.full(12, np.nan); sp = np.full(12, np.nan)
            for j in p["joints"]:
                b = j["bus"]
                if b < 12:
                    if j["flags"] & F.JF_GOAL_VALID:
                        gv[b] = j["goal"]
                    if j["flags"] & F.JF_FB_OK:
                        pv[b] = j["pos"]
                    sp[b] = j["goal_speed"]
            rec = (tu, tu + p["fb_off_us"] / 1e6, *p["gyro"], *p["up"], p["source"], p["flags"], p["bal"]["state"],
                   p["bal"]["side"], p["ctrl"]["phase"], p["boot_id"], *gv, *pv, *sp)
            for k in hit:
                rows[k].append(rec)
    out = {}
    for k, rr in rows.items():
        if not rr:
            continue
        A = np.array(sorted(rr, key=lambda x: x[0]), float)
        goal = np.full((len(A), 12), np.nan); pos = np.full((len(A), 12), np.nan)
        for bid in np.unique(A[:, 13]):
            c = cal.get(int(bid))
            if c is None:
                continue
            z = np.array(c[0][:12], float); d_ = np.array(c[1][:12], float)
            m = A[:, 13] == bid
            goal[m] = (A[m, 14:26] - z) * (2 * math.pi / 4096) * d_
            pos[m] = (A[m, 26:38] - z) * (2 * math.pi / 4096) * d_
        out[k] = dict(t=A[:, 0], tfb=A[:, 1], gyro=A[:, 2:5], up=A[:, 5:8], src=A[:, 8].astype(int),
                      flags=A[:, 9].astype(int), bal_state=A[:, 10].astype(int), bal_side=A[:, 11].astype(int),
                      phase=A[:, 12], goal=goal, pos=pos, goal_speed=A[:, 38:50], frec=os.path.relpath(path, MAIN))
    return out


def track_logs(days):
    found = {}
    for d in glob.glob(os.path.expanduser("~/code/robot*/experiments/roll-balance/results")):
        for p in glob.glob(os.path.join(d, "track_*.jsonl")):
            st = os.path.basename(p)[6:21]
            if st[:8] in days and os.path.getsize(p) > 0:
                if st not in found or os.path.getsize(p) > os.path.getsize(found[st]):
                    found[st] = p
    return dict(sorted(found.items()))


def track_window(stamp, path):
    from zoneinfo import ZoneInfo
    t0 = dt.datetime.strptime(stamp, "%Y%m%dT%H%M%S").replace(tzinfo=ZoneInfo("America/Denver")).timestamp()
    rows = [json.loads(x) for x in open(path) if x.strip()]
    tmax = max((r.get("t", 0.0) for r in rows if isinstance(r.get("t"), (int, float))), default=0.0)
    plays = [r for r in rows if str(r.get("cmd", "")).startswith("track play")]
    summ = rows[-1] if rows and rows[-1].get("summary") else {}
    return t0, tmax, bool(plays), summ.get("note", "")


def stand_segments():
    """The 2026-11-02 policy stands: contiguous `ctrl` ticks of >= 8 s with the gait clock running (phase moving)."""
    files = sorted(glob.glob(os.path.join(HW, "2026-11-02", "flight_20261102T1*_policy_stand*.frec")))
    out = []
    for p in files:
        c = frec_created(p)
        r = read_windows(p, [("all", c - 60, c + 6 * 3600)]).get("all")
        if r is None:
            continue
        ctrl = r["src"] == 1   # flightrec.SRC_CTRL
        i = 0
        while i < len(ctrl):
            if not ctrl[i]:
                i += 1
                continue
            j = i
            while j + 1 < len(ctrl) and ctrl[j + 1] and r["t"][j + 1] - r["t"][j] < 0.5:
                j += 1
            if r["t"][j] - r["t"][i] >= 8.0:
                lab = os.path.basename(p).split("_", 2)[2].replace(".frec", "")
                key = f"stand_{lab}_{dt.datetime.utcfromtimestamp(r['t'][i]).strftime('%H%M%S')}"
                seg = {k: (v[i:j + 1] if isinstance(v, np.ndarray) else v) for k, v in r.items()}
                seg["kind"] = "stand"
                seg["servo"] = "factory" if "stand2" in lab or "stand1" in lab else "fast"
                out.append((key, seg))
            i = j + 1
    return out


def active_span(r):
    """Ticks from the first written-goal motion to the last (pad 1.5 s / 1 s), cut at the robot's fall."""
    g = r["goal"]
    ok = np.all(np.isfinite(g), 1)
    if ok.sum() < 50:
        return None
    g0 = g[np.argmax(ok)]
    mv = np.where(ok & (np.nanmax(np.abs(g - g0), 1) > math.radians(0.5)))[0]
    if len(mv) == 0:
        return None
    i0 = max(0, int(np.searchsorted(r["t"], r["t"][mv[0]] - 1.5)))
    i1 = min(len(g) - 1, int(np.searchsorted(r["t"], r["t"][mv[-1]] + 1.0)))
    tilt = np.degrees(np.arccos(np.clip(r["up"][:, 2], -1, 1)))
    big = np.where((tilt > FALL_DEG) & (np.arange(len(tilt)) >= i0))[0]
    fell = bool(len(big) and big[0] <= i1)
    if fell:
        i1 = int(big[0])
    return i0, i1, fell


def cmd_extract(a):
    os.makedirs(os.path.join(OUT, "robot"), exist_ok=True)
    if not a.no_stand:
        for key, seg in stand_segments():
            np.savez_compressed(os.path.join(OUT, "robot", key + ".npz"), **seg)
            print(key, len(seg["t"]), "ticks", seg["servo"], flush=True)
    logs = track_logs(set(a.days.split(",")))
    by = {}
    for st, p in logs.items():
        t0, tmax, played, note = track_window(st, p)
        if not played:
            continue
        fr = frec_for(t0)
        if fr is None:
            continue
        by.setdefault(fr, []).append((st, t0 - 2.0, t0 + tmax + 4.0, note))
    for fr, ws in by.items():
        print("reading", os.path.relpath(fr, MAIN), len(ws), "runs", flush=True)
        R = read_windows(fr, [(s, u0, u1) for s, u0, u1, _ in ws])
        for st, _, _, note in ws:
            r = R.get(st)
            if r is None:
                continue
            sp = active_span(r)
            if sp is None:
                continue
            i0, i1, fell = sp
            seg = {k: (v[i0:i1 + 1] if isinstance(v, np.ndarray) else v) for k, v in r.items()}
            seg.update(kind="walk", servo="fast", note=note, robot_fell=fell)
            np.savez_compressed(os.path.join(OUT, "robot", f"walk_{st}.npz"), **seg)
            print(f"  walk_{st}: {i1 - i0 + 1} ticks, {seg['t'][-1] - seg['t'][0]:.0f} s, fell {fell} {note[:50]}",
                  flush=True)


# ----------------------------------------------------------------------------------------------------------------
# the sim side
# ----------------------------------------------------------------------------------------------------------------
def build_plant(ep_s, servo="fast"):
    """The x1.5 probe's plant, as grade.episode builds it (simh.build('fit') on GRADE_XML, JF_PLANT, the
    GRADE_PLANT_TERMS, the FAST TypedServo; DR, pushes and the IMU misalignment off). Returns (env, servo)."""
    set_env()
    sys.path.insert(0, os.path.join(HERE, "..", "shaper-train"))
    import grade as G
    import simh
    env, _act, zoff = simh.build("fit", run=os.environ.get("WALK_GAIN_RUN", "robot_legs_prec16_f075_js20_s128r24"),
                                 ep_s=ep_s)
    env.domain_rand = False
    env.push_prob = 0.0
    env.reset(seed=0)
    env._zero_off = zoff.copy()
    simh.apply_model(env, simh.JF_PLANT)
    G.roll_plant(env)
    spec = G.FAST if servo == "fast" else dict(simh.MEAS_SERVO)
    return env, simh.TypedServo(**spec)


def replay_one(key, gain=1.0, out="sim"):
    """Replay one segment's written goals open loop (gain != 1: the out_gain law applied to them first, the
    selftest's synthetic robot) -> OUT/<out>/<key>.npz."""
    set_env()
    import grade  # noqa: F401  (puts stand-still and simh on the path, applies GRADE_XML)
    import simh
    r = dict(np.load(os.path.join(OUT, "robot", key + ".npz"), allow_pickle=False))
    t = r["t"]
    goal = r["goal"].copy()
    pos = r["pos"]
    for j in range(12):                       # forward fill; before the first goal, the encoder
        last = pos[np.argmax(np.isfinite(pos[:, j])), j]
        for i in range(len(goal)):
            if np.isfinite(goal[i, j]):
                last = goal[i, j]
            else:
                goal[i, j] = last
    T = np.arange(t[0], t[-1], DT)
    gi = np.searchsorted(t, T, side="right") - 1         # zero-order hold of the record
    G_ = goal[np.clip(gi, 0, len(t) - 1)][:, B2P]
    if gain != 1.0:                                     # grade's law: mean + G (x - mean), mean = 0.5 s EMA
        G_ = G_ + (gain - 1.0) * ema_hp(G_)
    env, sv = build_plant(T[-1] - T[0] + 10.0, servo=str(r["servo"]))
    i0 = int(np.argmax(np.all(np.isfinite(pos), 1)))
    q0 = pos[i0][B2P]
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), 1.5)
    sv.reset(G_[0])
    cur = {"t": G_[0].copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    nj = env._nq_act
    d = env.data
    Q, GY, UP, SH = [], [], [], []
    fell = False
    for k in range(len(T)):
        cur["t"] = sv(G_[k])
        obs, _, term, _, _ = env.step(a0)
        Q.append(obs[:nj].astype(np.float64))
        GY.append(np.array(d.qvel[3:6]))
        w, x, y, z = (float(c) for c in d.qpos[3:7])
        UP.append((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))
        SH.append((float(d.geom_xpos[env._sole_gids[0]][2]), float(d.geom_xpos[env._sole_gids[1]][2])))
        if UP[-1][2] < math.cos(math.radians(FALL_DEG)):
            fell = True
            break
    n = len(Q)
    os.makedirs(os.path.join(OUT, out), exist_ok=True)
    np.savez_compressed(os.path.join(OUT, out, key + ".npz"), t=T[:n], goal=G_[:n], q=np.array(Q),
                        gyro=np.array(GY), up=np.array(UP), sole_z=np.array(SH), fell=fell)
    return key, n * DT, fell


def cmd_replay(a):
    os.makedirs(os.path.join(OUT, "sim"), exist_ok=True)
    keys = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(OUT, "robot", "*.npz")))
    if a.only:
        keys = [k for k in keys if any(s in k for s in a.only.split(","))]
    if not a.force:
        keys = [k for k in keys if not os.path.exists(os.path.join(OUT, "sim", k + ".npz"))]
    print(len(keys), "segments to replay", flush=True)
    with ProcessPoolExecutor(a.workers) as ex:
        for key, s, fell in ex.map(replay_one, keys):
            print(f"  {key}: {s:.0f} s{'  SIM FELL' if fell else ''}", flush=True)


# ----------------------------------------------------------------------------------------------------------------
# the estimate
# ----------------------------------------------------------------------------------------------------------------
def ema_hp(x, tau=0.5):
    """x - EMA(x, tau) along axis 0: the part of a signal grade's out_gain scales."""
    k = 1 - math.exp(-DT / tau)
    e = x[0].copy()
    out = np.empty_like(x)
    for i in range(len(x)):
        e = e + k * (x[i] - e)
        out[i] = x[i] - e
    return out


def band(x, b):
    if b is None:
        return ema_hp(x)
    from scipy.signal import butter, sosfiltfilt
    sos = butter(2, [b[0], b[1]], btype="band", fs=1 / DT, output="sos")
    return sosfiltfilt(sos, x, axis=0)


def ratio(yr, ys, m):
    """(g, r, tau_ms, var_s, cov) of y_r ~ g * y_s(t - tau) over the mask m, tau maximising the correlation."""
    best = None
    for tau in TAU_MS:
        s = int(round(tau / 1e3 / DT))
        a = yr[m]
        idx = np.where(m)[0] - s
        if idx.min() < 0 or idx.max() >= len(ys):
            continue
        b = ys[idx]
        a = a - a.mean(); b = b - b.mean()
        vb, va = float(b @ b), float(a @ a)
        if vb <= 1e-14 or va <= 1e-14:
            continue
        c = float(a @ b)
        r = c / (math.sqrt(va) * math.sqrt(vb))
        if best is None or r > best[1]:
            best = (c / vb, r, float(tau), vb, c)
    return best


def lagcov(yr, ys, m):
    """For every lag in LAGS_MS, over the mask: (sum (y_r - mean)(y_s(t - lag) - mean), sum (y_s(t - lag) - mean)^2,
    sum (y_r - mean)^2), each NaN where the window cannot shift that far. The caller sums them over windows, so every
    lag is normalised by the variance of the very samples it compared. Shape (3, len(LAGS_MS))."""
    idx = np.where(m)[0]
    a = yr[idx] - yr[idx].mean()
    out = np.full((3, len(LAGS_MS)), np.nan)
    for k, lag in enumerate(LAGS_MS):
        j = idx - int(round(lag / 1e3 / DT))
        if j.min() < 0 or j.max() >= len(ys):
            continue
        b = ys[j] - ys[j].mean()
        out[:, k] = (float(a @ b), float(b @ b), float(a @ a))
    return out


def channels(r, s):
    """Robot and sim on the sim's 50 Hz grid, upright part only: dict name -> (robot, sim)."""
    # the sim's sample k is the state after the tick that applied goal k (t + DT); the robot's encoders are read
    # fb_off after the tick start (tfb), its IMU at the tick: both are sampled at t + DT here
    T = s["t"] + DT
    n = len(T)
    okr = np.all(np.isfinite(r["pos"]), 1)
    pos = np.stack([np.interp(T, r["tfb"][okr], r["pos"][okr, b]) for b in B2P], 1)
    gy = np.stack([np.interp(T, r["t"], r["gyro"][:, i]) for i in range(3)], 1)
    upr = np.stack([np.interp(T, r["t"], r["up"][:, i]) for i in range(3)], 1)
    rr = lambda up: -np.arcsin(np.clip(up[:, 1], -1, 1))  # noqa: E731  roll, + = right side down
    pp = lambda up: np.arctan2(up[:, 0], up[:, 2])  # noqa: E731
    ch = {f"q:{POL[j]}": (pos[:, j], s["q"][:n, j]) for j in range(12)}
    for i, ax in enumerate("xyz"):
        ch[f"gyro_{ax}"] = (gy[:, i], s["gyro"][:n, i])
    ch["roll"] = (rr(upr), rr(s["up"][:n]))
    ch["pitch"] = (pp(upr), pp(s["up"][:n]))
    ch["goal_ref"] = None
    return ch


def windows(r, s):
    """[(kind, side, i0, i1)] on the sim grid: walks -> ss (single support, side = stance), ds; stands -> 4 s."""
    T = s["t"]
    n = len(T)
    if str(r["kind"]) == "stand":
        w = int(STAND_WIN_S / DT)
        return [("stand", "", i, i + w) for i in range(int(1.0 / DT), n - w, w)]
    st = np.interp(T, r["t"], r["bal_state"]).round().astype(int)
    sd = np.interp(T, r["t"], r["bal_side"]).round().astype(int)
    ss = (st == CLOSED) | (st == HOLD)
    out, i = [], 0
    while i < n:
        j = i
        while j + 1 < n and ss[j + 1] == ss[i] and (not ss[i] or sd[j + 1] == sd[i]):
            j += 1
        L = j - i + 1
        if ss[i] and L >= int(0.8 / DT):
            out.append(("ss", chr(sd[i]) if sd[i] else "?", i, j + 1))
        elif not ss[i] and L >= int(0.8 / DT):
            out.append(("ds", "", i, j + 1))
        i = j + 1
    return out


def leg_role(jname, kind, side):
    if kind != "ss":
        return kind
    return "stance" if jname[0] == side else "swing"


def analyze_segment(key):
    r = dict(np.load(os.path.join(OUT, "robot", key + ".npz"), allow_pickle=False))
    s = dict(np.load(os.path.join(OUT, "sim", key + ".npz"), allow_pickle=False))
    return analyze_pair(key, r, s)


def analyze_pair(key, r, s):
    """Per window, band and channel: the robot r against the sim s (the rows summarize() pools)."""
    n = len(s["t"])
    if n < 200:
        return []
    ch = channels(r, s)
    ch.pop("goal_ref")
    # usable: both upright (tilt < 20 deg) and the sim's attitude within 6 deg of the robot's (the open-loop
    # replay has not drifted to another pose)
    upr = np.stack([np.interp(s["t"], r["t"], r["up"][:, i]) for i in range(3)], 1)
    tr = np.degrees(np.arccos(np.clip(upr[:, 2], -1, 1)))
    ts = np.degrees(np.arccos(np.clip(s["up"][:n, 2], -1, 1)))
    dr = np.degrees(np.abs(ch["roll"][0] - ch["roll"][1] - np.median(ch["roll"][0][:50] - ch["roll"][1][:50])))
    dp = np.degrees(np.abs(ch["pitch"][0] - ch["pitch"][1] - np.median(ch["pitch"][0][:50] - ch["pitch"][1][:50])))
    usable = (tr < 20) & (ts < 20) & (dr < 6) & (dp < 6)
    seg_kind = str(r["kind"])
    if seg_kind == "stand":
        # a stand whose goals move by less than ~1 encoder step (0.088 deg) is all quantization: kept apart
        exc = float(np.median(np.degrees(np.std(ema_hp(s["goal"][:n])[50:], 0))))
        seg_kind = "stand" if exc >= 0.1 else "stand_quiet"
    rows = []
    W = windows(r, s)
    for bname, b in BANDS.items():
        F = {c: (band(v[0][:, None], b)[:, 0], band(v[1][:, None], b)[:, 0]) for c, v in ch.items()}
        gb = band(s["goal"][:n], b)
        for kind, side, i0, i1 in W:
            m = np.zeros(n, bool)
            m[i0:i1] = True
            m &= usable
            if m.sum() < int(0.6 / DT):
                continue
            for c, (yr, ys) in F.items():
                res = ratio(yr, ys, m)
                if res is None:
                    continue
                g, rc, tau, vs, cv = res
                j = c[2:] if c.startswith("q:") else None
                role = leg_role(j, kind, side) if j else kind
                gi = POL.index(j) if j else None
                rows.append(dict(seg=key, kind=seg_kind, servo=str(r["servo"]), band=bname, win=kind,
                                 side=side, i0=i0, ch=c, group=group_of(j) if j else c, role=role, g=g, r=rc,
                                 tau=tau, var_s=vs / m.sum(), cov=cv / m.sum(), n=int(m.sum()),
                                 rms_r=float(np.std(yr[m])), rms_s=float(np.std(ys[m])),
                                 lc=lagcov(yr, ys, m),
                                 rms_goal=float(np.sqrt(np.mean(gb[m, gi] ** 2))) if j else None))
    return rows


def _pool(L):
    """(g, coherence, lag ms, g at lag 0) from lag sums L (3, n_lags) = summed lagcov() of a set of windows: the lag
    that maximises the normalised cross-covariance (the coherence), and the gain there."""
    C, VS, VR = L
    with np.errstate(invalid="ignore", divide="ignore"):
        coh = np.where(VS > 0, C * np.abs(C) / (VS * VR), -np.inf)
    k = int(np.argmax(coh))
    k0 = int(np.where(LAGS_MS == 0)[0][0])
    return C[k] / VS[k], float(max(coh[k], 0.0)), float(LAGS_MS[k]), C[k0] / VS[k0]


def pooled(xs):
    """H1 over a set of windows: the cross-covariance robot x sim summed over EVERY window (no selection: robot
    noise and whatever the sim does not produce average out of the cross term), at the one lag (LAGS_MS) that
    maximises the normalised cross-covariance, over the sim's summed variance at that lag. Returns (g, coherence,
    lag ms, g at lag 0, rms ratio robot / sim). The coherence is the share of the robot's variance that the scaled,
    shifted sim explains; the rms ratio counts everything, so it is an upper bound."""
    L = np.zeros((3, len(LAGS_MS)))
    vs = vr = 0.0
    for x in xs:
        L += np.nan_to_num(x["lc"])                 # a window too short for a lag adds nothing at that lag
        vs += x["n"] * x["rms_s"] ** 2
        vr += x["n"] * x["rms_r"] ** 2
    g, coh, lag, g0 = _pool(L)
    return g, coh, lag, g0, math.sqrt(vr / vs)


def summarize(rows, min_r=0.5, min_rms_deg=0.15, nb=1000, seed=0):
    """Per (kind, servo, band, channel class, role):
      g [90 % CI], lag  pooled H1 (pooled()) over every window (joints: windows whose goal moved >= min_rms_deg in
                        the band); CI by resampling whole segments (runs / stands), the lag re-chosen per draw;
      coh, g0           its coherence; the same at zero lag;
      rms_ratio         robot rms / sim rms over the same windows;
      g_med             per-window g at each window's best lag (-60..+120 ms), median over windows with r >= min_r
                        (n_fit of n): selected on fit quality, so biased up where the coherence is low."""
    out = []
    keys = {}
    for x in rows:
        cls = ("joints:" + x["group"]) if x["ch"].startswith("q:") else x["ch"]
        role = x["role"] if x["ch"].startswith("q:") else x["win"] + (":" + x["side"] if x["side"] else "")
        for k in ((x["kind"], x["servo"], x["band"], cls, role), (x["kind"], x["servo"], x["band"], cls, "all")):
            keys.setdefault(k, []).append(x)
    rng = np.random.default_rng(seed)
    for k, xs in sorted(keys.items()):
        xs = [x for x in xs if x["rms_goal"] is None or math.degrees(x["rms_goal"]) >= min_rms_deg]
        d = dict(zip(("kind", "servo", "band", "cls", "role"), k), n=len(xs))
        if len(xs) < 5:
            out.append(d)
            continue
        g, coh, lag, g0, rr = pooled(xs)
        segs = sorted({x["seg"] for x in xs})
        ix = {sg: i for i, sg in enumerate(segs)}
        S = np.zeros((len(segs), 3, len(LAGS_MS)))        # per segment: summed lag sums
        for x in xs:
            S[ix[x["seg"]]] += np.nan_to_num(x["lc"])
        pick = rng.integers(0, len(segs), (nb, len(segs)))
        bs = [_pool(S[p_].sum(0))[0] for p_ in pick]
        ok = [x for x in xs if x["r"] >= min_r]
        d.update(n_seg=len(segs), g=float(g), ci90=(float(np.percentile(bs, 5)), float(np.percentile(bs, 95))),
                 coh=float(coh), lag=lag, g0=float(g0), rms_ratio=float(rr), n_fit=len(ok),
                 g_med=float(np.median([x["g"] for x in ok])) if ok else None)
        out.append(d)
    return out


def md_table(summ, kinds=("stand", "walk"), bands=("probe", "0.3-1", "1-3", "3-8"), roles=None, classes=None):
    L = ["| data | servo | band | channel | where | windows (segments) | **g** [90 % CI] | coherence | lag (ms) | "
         "g at lag 0 | rms ratio | per-window g, r >= 0.5: median (share) |", "|" + "---|" * 12]
    for x in summ:
        if x["kind"] not in kinds or x["band"] not in bands or (roles and x["role"] not in roles) or \
                (classes and x["cls"] not in classes):
            continue
        if "g" not in x:
            L.append(f"| {x['kind']} | {x['servo']} | {x['band']} | {x['cls']} | {x['role']} | {x['n']} | too few "
                     "| | | | | |")
            continue
        gm = f"{x['g_med']:.2f} ({x['n_fit'] / x['n']:.0%})" if x["g_med"] is not None else "-"
        L.append(f"| {x['kind']} | {x['servo']} | {x['band']} | {x['cls']} | {x['role']} | {x['n']} ({x['n_seg']}) | "
                 f"**{x['g']:.2f}** [{x['ci90'][0]:.2f}, {x['ci90'][1]:.2f}] | {x['coh']:.2f} | {x['lag']:+.0f} | "
                 f"{x['g0']:.2f} | {x['rms_ratio']:.2f} | {gm} |")
    return "\n".join(L)


def seg_table(rows, kind="stand", bands=("probe", "1-3"), min_rms_deg=0.15,
              cls=("gyro_x", "gyro_y", "gyro_z", "roll", "pitch", "joints:roll", "joints:sag", "joints:yaw")):
    """Per segment: the pooled g (coherence) of each channel class."""
    L = ["| segment | band | " + " | ".join(cls) + " |", "|" + "---|" * (len(cls) + 2)]
    for seg in sorted({x["seg"] for x in rows if x["kind"] == kind}):
        for b in bands:
            xs = [x for x in rows if x["seg"] == seg and x["band"] == b and
                  (x["rms_goal"] is None or math.degrees(x["rms_goal"]) >= min_rms_deg)]
            cells = []
            for c in cls:
                cx = [x for x in xs if (("joints:" + x["group"]) if x["ch"].startswith("q:") else x["ch"]) == c]
                if cx:
                    g, coh = pooled(cx)[:2]
                    cells.append(f"{g:.2f} ({coh:.2f})")
                else:
                    cells.append("-")
            L.append(f"| {seg.replace('stand_policy_', '')} | {b} | " + " | ".join(cells) + " |")
    return "\n".join(L)


def cmd_analyze(a):
    keys = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(OUT, "sim", "*.npz")))
    if a.only:
        keys = [k for k in keys if any(s in k for s in a.only.split(","))]
    rows = []
    if a.from_rows:                            # a previous --rows json: re-summarise without re-reading the traces
        import pickle
        rows = pickle.load(open(a.from_rows, "rb"))
    else:
        with ProcessPoolExecutor(a.workers) as ex:
            for rr in ex.map(analyze_segment, keys):
                rows.extend(rr)
    seg_info = []
    for k in keys:
        s = np.load(os.path.join(OUT, "sim", k + ".npz"))
        r = np.load(os.path.join(OUT, "robot", k + ".npz"))
        seg_info.append(dict(seg=k, dur=float(len(s["t"]) * DT), sim_fell=bool(s["fell"]),
                             robot_fell=bool(r["robot_fell"]) if "robot_fell" in r.files else False))
    summ = summarize(rows, a.min_r, a.min_rms)
    if a.json:
        json.dump(dict(segments=seg_info, summary=summ), open(a.json, "w"), indent=1)
    if a.rows:
        import pickle
        pickle.dump(rows, open(a.rows, "wb"))
    md = md_table(summ) + "\n\nPer stand segment (median g, windows):\n\n" + seg_table(rows, min_rms_deg=a.min_rms)
    if a.md:
        open(a.md, "w").write(md + "\n")
    print(md)


def _selftest_one(args):
    key, gain = args
    replay_one(key, gain=gain, out=f"selftest_g{gain:g}")
    return key


def selftest_rows(keys, gain, workers=8):
    """The rows of analyze, with the gain-G replay as the robot and the plain replay as the sim."""
    with ProcessPoolExecutor(workers) as ex:
        list(ex.map(_selftest_one, [(k, gain) for k in keys]))
    rows = []
    for k in keys:
        r = dict(np.load(os.path.join(OUT, "robot", k + ".npz"), allow_pickle=False))
        g = dict(np.load(os.path.join(OUT, f"selftest_g{gain:g}", k + ".npz"), allow_pickle=False))
        s = dict(np.load(os.path.join(OUT, "sim", k + ".npz"), allow_pickle=False))
        n = min(len(g["t"]), len(s["t"]))
        s = {kk: (v[:n] if isinstance(v, np.ndarray) and v.ndim else v) for kk, v in s.items()}
        syn = dict(r)
        syn.update(t=g["t"][:n] + DT, tfb=g["t"][:n] + DT, pos=g["q"][:n][:, np.argsort(B2P)], gyro=g["gyro"][:n],
                   up=g["up"][:n], bal_state=np.interp(g["t"][:n], r["t"], r["bal_state"]).round(),
                   bal_side=np.interp(g["t"][:n], r["t"], r["bal_side"]).round())
        rows += analyze_pair(k, syn, s)
    return rows


def cmd_selftest(a):
    """The method against a known gain: segments replayed with the out_gain law at G on their written goals stand in
    for the robot, the plain replay for the sim; same windows, bands and pooling as analyze. Prints, per G, the
    pooled g it reads (walks pooled, the excited stands per segment): the calibration from a true G to the reading."""
    keys = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(OUT, "sim", "*.npz")))
    walks = [k for k in keys if k.startswith("walk_")]
    walks = walks[:: max(1, len(walks) // a.n)][:a.n]
    stands = [k for k in keys if any(x in k for x in ("stand2_", "stand3_fast", "stand4_walk16", "stand5_coast_174522",
                                                       "stand7_series_181045"))]
    out = {}
    for gain in [float(x) for x in a.gains.split(",")]:
        rows = selftest_rows(walks + stands, gain, a.workers)
        summ = summarize(rows, a.min_r, a.min_rms)
        out[gain] = summ
        print(f"\n## selftest: true G = {gain:g}\n")
        print(md_table(summ, kinds=("walk",), roles=("all", "ss:L", "ss:R", "ds"), bands=("probe", "1-3")))
        print()
        print(seg_table(rows, min_rms_deg=a.min_rms))
    if a.json:
        json.dump({str(k): v for k, v in out.items()}, open(a.json, "w"), indent=1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract")
    e.add_argument("--days", default="20261112,20261113,20261116")
    e.add_argument("--no-stand", action="store_true")
    rp = sub.add_parser("replay")
    rp.add_argument("--workers", type=int, default=8)
    rp.add_argument("--only")
    rp.add_argument("--force", action="store_true")
    an = sub.add_parser("analyze")
    an.add_argument("--workers", type=int, default=8)
    an.add_argument("--only")
    an.add_argument("--json")
    an.add_argument("--md")
    an.add_argument("--rows", help="write the per-window rows here (pickle)")
    an.add_argument("--from-rows", help="summarise the rows of an earlier --rows pickle")
    an.add_argument("--min-r", type=float, default=0.5)
    an.add_argument("--min-rms", type=float, default=0.15, help="deg: a joint window's goal rms in the band")
    st = sub.add_parser("selftest")
    st.add_argument("--gains", default="1.25,1.5,2.0")
    st.add_argument("--n", type=int, default=48, help="walk segments (every k-th)")
    st.add_argument("--json")
    st.add_argument("--workers", type=int, default=8)
    st.add_argument("--min-r", type=float, default=0.5)
    st.add_argument("--min-rms", type=float, default=0.15)
    a = ap.parse_args(argv)
    set_env()
    {"extract": cmd_extract, "replay": cmd_replay, "analyze": cmd_analyze, "selftest": cmd_selftest}[a.cmd](a)


if __name__ == "__main__":
    main()
