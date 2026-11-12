"""Roll-balance testbed (#297 step 2): the stepper's commanded trajectories played through the sim plant
with a balance law running inside the loop at the firmware tick.

Trajectory. Rebuilt from the run's own `pose` commands in the flight record (the joint ticks the firmware
received, when it received them, the move's `s<ms>`): the firmware streams a minimum-jerk move from the
goal it holds to the new target over ms, one goal per 20 ms tick, and a new pose interrupts a move in
flight. `trims="none"` takes the stepper's own closed-loop trims out of every target (its stance hip-roll
trim `hike`, the swing sole's levelling `sole_r` / `sole_p`, from the matching jsonl `pose` row): what is
left is the motion a balance law has to stabilise (shift, raise, swing, lower). `speed` plays the run
faster (every time and duration divided by it).

Loop, once per 20 ms control tick as the firmware's:
    meas  = torso roll, pitch (rad) and their rates (rad/s), `imu_delay` ticks old (+ `extra_ms`)
    phase = the stance leg and the time since the raise began (the upper layer's statement: a commanded
            swing foot above the floor = single support)
    d     = law.tick(meas, phase) -> {joint: rad} added to the commanded targets
    targets -> the plant's servo chain (grade.py's TypedServo: STS3215 dead 60 ms / tau 33 ms, STS3250
            2nd order) -> the plant (per-side roll play beyond the encoders, measured sole)
Roll + = leaning to the robot's right (-asin(up_y)), pitch + = forward (atan2(up_x, up_z)), as the IMU.

Metrics per single-support phase (a raise to the end of the lowering): the share of the hold (from 0.2 s
after the raise ends to the start of the lowering) with the swing sole's lowest pad > 5 mm up (a lift
counts when it is >= 0.5), the peak |roll - reference| (reference = the roll when the raise began), the CoM's
margin to the stance sole's outer edge (mm, min over the phase), and per run the fall (tilt > 35 deg)
with its direction (OUT = over the stance foot, IN = toward the swing foot).
"""
from __future__ import annotations

import math
import os
import re
import sys
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
GATE = os.path.join(ROOT, "experiments", "policy", "weight-transfer-gate")
sys.path.insert(0, GATE)
sys.path.insert(0, HERE)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
import gate_common as GC  # noqa: E402
import stepper_replay as SR  # noqa: E402

BUS = SR.JOINTS
DT = 0.02
PLAY_FIT = "play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0"
BOARD = "grade+" + PLAY_FIT                      # the plant the stepper replay fit (gate README Round 5)
FALL_DEG = 35.0
UP_M = 0.005          # a swing sole is "up" with its lowest pad this far above the floor
D2R = math.pi / 180.0


def _minjerk(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


# ------------------------------------------------------------------------------------------------
# the commanded trajectory
# ------------------------------------------------------------------------------------------------
class Traj:
    """The run's pose commands as segments {t, ms, q (bus joint -> rad), row (the jsonl pose row)}, and
    its single-support phases {t0, raise_end, t1, stance, swing}, in run time / speed."""

    def __init__(self, stamp, trims="logged", speed=1.0):
        self.stamp = stamp
        self.r = SR.robot(stamp)
        log = self.r["log"]
        cal = log[0]["cal"]
        cmds = [(t, c) for t, c in self.r["cmds"] if c.startswith("pose ")]
        rows = [x for x in log if x["ev"] == "pose"]
        segs = []
        for t, c in cmds:
            tok = c.split()
            ticks = [int(x) for x in tok[1:13]]
            ms = next((int(x[1:]) for x in tok[13:] if re.fullmatch(r"s\d+", x)), 0)
            q = {n: (ticks[i] - cal[n][1]) * cal[n][2] * 2 * math.pi / 4096 for i, n in enumerate(BUS)}
            segs.append(dict(t=t, ms=ms, q=q, row=None))
        # a jsonl `pose` row belongs to the command sent just before it was logged (a waited move logs
        # after it ends, ms + ~0.15 s later)
        for x in rows:
            t_sent = x["t"] - ((x["ms"] / 1e3 + 0.15) if x.get("wait", True) else 0.0)
            j = int(np.argmin([abs(s["t"] - t_sent) for s in segs]))
            if abs(segs[j]["t"] - t_sent) < 0.6 and segs[j]["row"] is None:
                segs[j]["row"] = x
        self.matched, self.n_rows = sum(s["row"] is not None for s in segs), len(rows)
        if trims == "none":
            for s in segs:
                x = s["row"]
                if x is None:
                    continue
                for side, sgn in (("L", 1.0), ("R", -1.0)):
                    s["q"][f"{side}_hip_roll"] -= sgn * x.get("hike", {}).get(side, 0.0) * D2R
                    s["q"][f"{side}_ankle_roll"] -= sgn * (x.get("sole_r") or {}).get(side, 0.0) * D2R
                    s["q"][f"{side}_ankle"] -= (x.get("sole_p") or {}).get(side, 0.0) * D2R
        for s in segs:
            s["t"] /= speed
            s["ms"] /= speed
        self.segs = segs
        self.phases = []
        cur = None
        for s in segs:
            x = s["row"]
            if x is None:
                continue
            up = [f for f in "LR" if x["feet"][f][2] > 1e-4]
            end = s["t"] + s["ms"] / 1e3
            if up and cur is None:
                cur = dict(t0=s["t"], swing=up[0], stance="R" if up[0] == "L" else "L", raise_end=end)
            if cur is not None and up:
                cur["z"] = max(cur.get("z", 0.0), x["feet"][cur["swing"]][2])
            elif cur is not None and up and "raise" in x.get("tag", ""):
                cur["raise_end"] = end
            elif cur is not None and up and "lower_start" not in cur and x["feet"][cur["swing"]][2] < cur.get("z", 0.0) - 1e-4:
                cur["lower_start"] = s["t"]
            elif cur is not None and not up:
                cur.setdefault("lower_start", s["t"])
                cur["t1"] = end
                self.phases.append(cur)
                cur = None
        if cur is not None:
            cur["t1"] = segs[-1]["t"] + segs[-1]["ms"] / 1e3
            self.phases.append(cur)

    def phase_at(self, t):
        for p in self.phases:
            if p["t0"] <= t <= p["t1"]:
                return p
        return None


FW_START_S = 0.07     # the firmware starts a pose move ~70 ms after the command arrives (serial log
                      # `pose -> 12 joints ... SMOOTH` lines; this lag gives the record's goals best)


class Stream:
    """The firmware's goal stream, advanced one tick at a time. q0: the goals (dict) before the first
    command (the servos' read-back: the first move of a run starts from where the joints are)."""

    def __init__(self, traj, q0=None):
        self.tr, self.k, self.q, self.seg = traj, 0, (dict(q0) if q0 is not None else None), None

    def step(self, t):
        segs = self.tr.segs
        while self.k < len(segs) and segs[self.k]["t"] + FW_START_S <= t:
            self.from_q = dict(self.q) if self.q is not None else dict(segs[self.k]["q"])
            self.seg = segs[self.k]
            self.k += 1
        if self.seg is None:
            return None
        u = _minjerk((t - self.seg["t"] - FW_START_S) / max(self.seg["ms"] / 1e3, 1e-6))
        self.q = {n: self.from_q[n] + u * (self.seg["q"][n] - self.from_q[n]) for n in BUS}
        return self.q


# ------------------------------------------------------------------------------------------------
# the run
# ------------------------------------------------------------------------------------------------
def _stance_margin(env, pads, stance):
    """CoM lateral margin to the stance sole's outer edge, m (+ = inside), in the sole's yaw frame."""
    d, m = env.data, env.model
    com = d.subtree_com[env._torso_bid]
    ps = np.array([d.geom_xpos[g] for g in pads[stance]])
    r = float(m.geom_size[pads[stance][0]][0])
    sole = d.geom_xmat[env._sole_gids[0 if stance == "L" else 1]].reshape(3, 3)
    ylat = sole[:2, 1] / max(np.linalg.norm(sole[:2, 1]), 1e-9)        # the sole's lateral axis, +y = left
    yc = float(np.dot(com[:2], ylat))
    yp = ps[:, :2] @ ylat
    return (float(yp.max()) + r - yc) if stance == "L" else (yc - float(yp.min()) + r)


def run(stamp, law=None, plant=BOARD, trims="none", speed=1.0, imu_delay=1, extra_ms=0.0, t_pre=0.5,
        record=False, t_end=None):
    """One replay with `law` (None = open loop) in the loop. Returns a dict of metrics (and traces)."""
    import rec
    tr = Traj(stamp, trims=trims, speed=speed) if isinstance(stamp, str) else stamp     # or a synth.Synth
    r = tr.r
    t_first = tr.segs[0]["t"]
    t_stop = t_end if t_end is not None else tr.segs[-1]["t"] + tr.segs[-1]["ms"] / 1e3 + 1.0
    label, env = SR.build(plant, (t_stop - t_first) + 10.0)
    pol = list(rec.POL)
    if r is not None:
        i0 = int(np.searchsorted(r["t"], t_first * speed - t_pre))
        q0 = r["pos"][i0][[BUS.index(n) for n in pol]]
        qs = {n: float(r["pos"][i0][BUS.index(n)]) for n in BUS}
    else:
        q0 = np.array([tr.q_start[n] for n in pol])
        qs = dict(tr.q_start)
    import simh
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), 1.5)
    pads = GC.pad_gids(env)
    stream = Stream(tr, qs)
    cur = {"t": q0.copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    d = env.data
    lag = imu_delay + int(round(extra_ms / 1e3 / DT))
    buf = deque(maxlen=lag + 1)
    if law is not None:
        law.reset()
    n = int(round((t_stop - (t_first - t_pre)) / DT))
    tr_rows = []
    ph_stats = {id(p): dict(peak=0.0, margin=1.0, clear=None, ref=None, n_hold=0, n_up=0, clr_max=-1.0, pref=None, ppeak=0.0)
                for p in tr.phases}
    fell, fall_dir, t_fall = False, None, None
    for k in range(n):
        t = t_first - t_pre + k * DT
        g = stream.step(t)
        up = GC.up_of(d.qpos[3:7])
        roll = -math.asin(max(-1.0, min(1.0, up[1])))
        pitch = math.atan2(up[0], up[2])
        w = np.array(d.qvel[3:6])                       # body-frame angular velocity: x = roll rate, y = pitch rate
        buf.append((roll, pitch, float(w[0]), float(w[1])))
        meas = buf[0]
        ph = tr.phase_at(t)
        tgt = dict(g) if g is not None else None
        if law is not None and tgt is not None:
            dq = law.tick(t, meas, ph)
            for j, v in dq.items():
                tgt[j] += v
        if tgt is not None:
            cur["t"] = np.clip(np.array([tgt[j] for j in pol]), env._lo, env._hi)
        env.step(a0)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        if ph is not None:
            s = ph_stats[id(ph)]
            if s["ref"] is None:
                s["ref"], s["pref"] = roll, pitch
            s["peak"] = max(s["peak"], abs(roll - s["ref"]))
            s["ppeak"] = max(s["ppeak"], abs(pitch - s["pref"]))
            s["margin"] = min(s["margin"], _stance_margin(env, pads, ph["stance"]))
            if s["clear"] is None and t >= ph["raise_end"]:
                s["clear"] = GC.clearance(env, pads)[ph["swing"]]
            if ph.get("hold_from", ph["raise_end"] + 0.2) <= t <= ph.get("lower_start", ph["t1"]):
                c_sw = GC.clearance(env, pads)[ph["swing"]]
                s["n_hold"] += 1
                s["n_up"] += c_sw > UP_M
                s["clr_max"] = max(s["clr_max"], c_sw)
        if not fell and tilt > FALL_DEG:
            fell, t_fall = True, t
            p_last = ph or (tr.phases[-1] if tr.phases else None)
            if p_last is not None:
                out_sign = -1.0 if p_last["stance"] == "L" else 1.0     # L stance: outward = leaning left = roll < 0
                fall_dir = "OUT" if roll * out_sign > 0 else "IN"
            else:
                fall_dir = "?"
            break
        if record:
            c = GC.clearance(env, pads)
            tr_rows.append((t, math.degrees(roll), math.degrees(pitch), c["L"], c["R"],
                            *(math.degrees(v) for v in (law.last if law is not None and hasattr(law, "last") else (0.0, 0.0)))))
    phs = [dict(stance=p["stance"], t0=p["t0"], clear_mm=(None if ph_stats[id(p)]["clear"] is None else 1e3 * ph_stats[id(p)]["clear"]),
                up_frac=ph_stats[id(p)]["n_up"] / max(ph_stats[id(p)]["n_hold"], 1),
                clr_max_mm=1e3 * ph_stats[id(p)]["clr_max"],
                peak_deg=math.degrees(ph_stats[id(p)]["peak"]), pitch_deg=math.degrees(ph_stats[id(p)]["ppeak"]), margin_mm=1e3 * ph_stats[id(p)]["margin"])
           for p in tr.phases if ph_stats[id(p)]["ref"] is not None]
    out = dict(run=tr.stamp, fell=fell, fall_dir=fall_dir, t_fall=t_fall, phases=phs,
               lifts=sum(1 for p in phs if p["up_frac"] >= 0.5), n_phases=len(tr.phases),
               peak_deg=max([p["peak_deg"] for p in phs] or [0.0]),
               pitch_deg=max([p["pitch_deg"] for p in phs] or [0.0]),
               margin_mm=min([p["margin_mm"] for p in phs] or [float("nan")]))
    if record:
        out["trace"] = np.array(tr_rows)
    return out
