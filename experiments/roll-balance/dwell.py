"""#297 B2, after the first track run on hardware (2026-11-12 16:25, T = 4 s march): a flattened sway with the
robot's STATIC shift requirement as a hard constraint.

The sine sway (sway.py) peaks over the stance foot only at the middle of single support, so its foot
leaves the floor with the pelvis at ~1.0-1.1 x 62 mm. The robot needs 1.25 x over its right foot and 1.3 x
over its left before the foot comes up (B1 lifts: 1.15 x tips in at foot-off, 1.25 x holds), and at 4 s a
step there are no dynamics to carry it: it fell toward the lifted foot on the first step. The sim did not
(it is optimistic on right stance: fw_replay.py), so the constraint is imposed, not learned from the sim.

One step of T s, per stance side:
    shift   the pelvis moves (min-jerk) from over the last stance foot to `k_static[side]` x 62 mm over the
            new one (+ that foot's own offset from where it stands), both feet down        t_shift
    settle  it holds there, both feet down                                                 t_settle
    swing   single support, the pelvis still holding: the swing sole rises (min-jerk) to h over the first
            35 %, moves (x, y, yaw) over the middle, comes down over the last 35 %          t_ss
The pelvis is at or beyond k_static from before lift-off until after touchdown. The default split is a
fraction of T (shift 0.3, settle 0.1, single support 0.6; `ds` sets shift + settle) unless T is in SCHEDULE or
t_shift / t_settle are given (absolute, s).

The firmware maps the frames' phase to the support state (gaitplay::supportFromPhase: single support =
u in [0.25, 0.75] of each half cycle, raise = its first 35 %, lowering after 65 %). The phase here is
warped piecewise-linearly in time so those points land on this gait's lift-off, raise end, lower start and
touchdown: no firmware change.

The object has the interface sil_gait.gait / testbed expect: segs (t, q, pa), phases, q_start, t_end,
phase_at(t), plus phi(t), step_ends and liftoff (per step: t, stance, the commanded shift ratio).
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import synth as S  # noqa: E402

K_STATIC = {"L": 1.3, "R": 1.25}      # x 62 mm, the robot's static requirement (hw 2026-11-12)
# (shift, settle) s per step time; single support = the rest. From the SIL (dwell_grid.py timing): the
# shift (min-jerk, ~158 mm from 1.25 x over one foot to 1.3 x over the other) needs >= 1.2 s, single
# support >= 1.8 s (raise 0.63 s); below T = 3.2 s it falls or leans 11-14 deg. 6 s is the gentle start
# (raise 1.4 s, the stepper's pace).
SCHEDULE = {6.0: (1.5, 0.5), 5.0: (1.4, 0.4), 4.0: (1.2, 0.4), 3.4: (1.2, 0.4), 3.2: (1.2, 0.2), 3.0: (1.0, 0.2)}
T_FLOOR = 3.2
# the hip yaw sign that turns the body to its LEFT about the stance foot (calibrated in the sim: see yaw_trim)
YAW_SIGN = -1.0          # sim (sil_gait, T 5, fit): yaw_trim +4 deg/step with +1 turned the body -35 deg over 6 steps


def _mj(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


class Dwell:
    def __init__(self, T=4.0, n_steps=6, kL=K_STATIC["L"], kR=K_STATIC["R"], ds=0.4, settle_frac=0.25, h=0.022,
                 dx=0.0, dy=0.0, yaw=0.0, first="L", drop=0.02, t_crouch=2.0, t_settle0=1.0, dt=0.04,
                 t_shift=None, t_settle=None, yaw_trim=0.0):
        K = S.kin(drop)
        self.oy = abs(K.model_offset("L")[1])
        self.k = {"L": float(kL), "R": float(kR)}
        t_ds = ds * T                         # shift + settle
        t_set = settle_frac * t_ds
        t_sh = t_ds - t_set
        if t_shift is None and float(T) in SCHEDULE:
            t_shift, t_settle = SCHEDULE[float(T)]
        if t_shift is not None:               # absolute durations override the fractions
            t_sh = float(t_shift)
            t_set = float(t_settle if t_settle is not None else 0.25)
            t_ds = t_sh + t_set
        t_ss = T - t_ds
        self.T, self.t_shift, self.t_settle, self.t_ss, self.h = T, t_sh, t_set, t_ss, h
        self.stamp = (f"dwell T{T:g} ds{ds:g} (shift {t_sh:.2f} settle {t_set:.2f} ss {t_ss:.2f} s) "
                      f"kL{kL:g} kR{kR:g} h{1e3 * h:.0f}mm dx{dx:+g} dy{dy:+g} yaw{yaw:+g}")
        self.r = None
        self.q_start = K.pose(straight=True)
        swing_of = (lambda i: first if i % 2 == 0 else ("R" if first == "L" else "L"))
        t0 = t_crouch + t_settle0
        feet = {"L": [0.0, 0.0], "R": [0.0, 0.0]}
        yawq = {"L": 0.0, "R": 0.0}
        # the schedule: step i = [shift, settle, single support]; then a final shift back to the centre
        steps = []
        t = t0
        for i in range(n_steps):
            sw = swing_of(i)
            st = "R" if sw == "L" else "L"
            steps.append(dict(i=i, swing=sw, stance=st, a=t, b=t + t_sh, lo=t + t_ds, td=t + T))
            t += T
        t_end_steps = t
        t_back = t_end_steps + t_sh            # back to the centre, both feet down
        self.t_end = t_back + 1.0
        # the feet's goals per step (the stepper's convention: the leading foot steps out, the other follows)
        goals = []
        f = {s: list(v) for s, v in feet.items()}
        yq = dict(yawq)
        for p in steps:
            sw, st = p["swing"], p["stance"]
            out = (dy > 0 and sw == "L") or (dy < 0 and sw == "R")
            start = list(f[sw])
            goal = [f[st][0] + dx, f[st][1] + (dy if out else 0.0)]
            y_from = dict(yq)
            y_to = dict(yq)
            y_to[sw] = (yaw if p["i"] % 2 == 0 else -yaw) if yaw else 0.0
            if yaw_trim:
                # a heading change of yaw_trim deg per step (+ = to the robot's left): over the middle of single
                # support the stance hip yaw goes from -c to +c (the body turns about the planted foot) and the
                # swing hip from +c to -c (the foot lands square to the new heading); c = half the step's turn
                c = YAW_SIGN * math.radians(yaw_trim) / 2.0
                y_to[st] = c                      # was -c, as the last step's swing (0 on the first step)
                y_to[sw] = y_to[sw] - c
            goals.append((start, goal, y_from, y_to))
            f[sw] = goal
            yq = y_to
        self._steps, self._goals = steps, goals

        def stance_target(i, fe):
            """The pelvis (absolute shift, m) over step i's stance foot: k_static x 62 mm + that foot's offset."""
            st = steps[i]["stance"]
            sg = 1.0 if st == "L" else -1.0
            return fe[st][1] + sg * self.k[st] * self.oy

        segs = [dict(t=0.0, ms=1e3 * t_crouch, q=K.pose(), row=dict(tag="crouch"))]
        phases, self.liftoff, self.step_ends = [], [], []
        fe = {s: list(v) for s, v in feet.items()}
        yq = {"L": 0.0, "R": 0.0}
        pel_from = 0.0
        tt = t0
        cur = 0
        while tt <= self.t_end + 1e-9:
            while cur < len(steps) and tt > steps[cur]["td"] + 1e-9:
                # step cur is over: its swing foot is at its goal
                start, goal, y_from, y_to = goals[cur]
                fe[steps[cur]["swing"]] = list(goal)
                yq = dict(y_to)
                pel_from = stance_target(cur, fe)
                cur += 1
            if cur < len(steps):
                p = steps[cur]
                tgt = stance_target(cur, fe)
                if tt <= p["b"]:
                    pel = pel_from + _mj((tt - p["a"]) / max(t_sh, 1e-6)) * (tgt - pel_from)
                else:
                    pel = tgt
                fz = {"L": 0.0, "R": 0.0}
                fxy = {s: list(v) for s, v in fe.items()}
                yy = dict(yq)
                if tt > p["lo"]:
                    u = (tt - p["lo"]) / t_ss
                    sw = p["swing"]
                    start, goal, y_from, y_to = goals[cur]
                    if u < 0.35:
                        fz[sw] = h * _mj(u / 0.35)
                    elif u < 0.65:
                        fz[sw] = h
                    else:
                        fz[sw] = h * (1.0 - _mj((u - 0.65) / 0.35))
                    m = _mj((u - 0.35) / 0.3)
                    fxy[sw] = [start[0] + m * (goal[0] - start[0]), start[1] + m * (goal[1] - start[1])]
                    yy[sw] = y_from[sw] + m * (y_to[sw] - y_from[sw])
                    st_ = p["stance"]
                    yy[st_] = y_from[st_] + m * (y_to[st_] - y_from[st_])
            else:
                # after the last step: back to the centre between the feet
                mid = 0.5 * (fe["L"][1] + fe["R"][1])
                pel = pel_from + _mj((tt - t_end_steps) / max(t_sh, 1e-6)) * (mid - pel_from)
                fz = {"L": 0.0, "R": 0.0}
                fxy = {s: list(v) for s, v in fe.items()}
                yy = dict(yq)
            x_pel = 0.5 * (fxy["L"][0] + fxy["R"][0])
            qd = K.pose(shift=pel, fwd=x_pel, footL=(fxy["L"][0], fxy["L"][1], fz["L"]),
                        footR=(fxy["R"][0], fxy["R"][1], fz["R"]))
            qd["L_hip_yaw"] += yy["L"]
            qd["R_hip_yaw"] += yy["R"]
            segs.append(dict(t=tt, ms=1e3 * dt, q=qd, row=None,
                             pa=dict(fwd=x_pel, shift=0.5 * (fxy["L"][1] + fxy["R"][1]), pel=pel,
                                     feet={s: [fxy[s][0], fxy[s][1], fz[s]] for s in fxy}, yaw=dict(yy))))
            tt += dt
        for i, p in enumerate(steps):
            lo, td = p["lo"], p["td"]
            phases.append(dict(t0=lo, raise_end=lo + 0.35 * t_ss, hold_from=lo + 0.35 * t_ss,
                               lower_start=lo + 0.65 * t_ss, t1=td, swing=p["swing"], stance=p["stance"],
                               ss_a=lo, ss_b=td))
            self.step_ends.append(td + min(0.1, 0.5 * t_sh))     # both feet down: a stop may end here
        # the commanded shift ratio at each lift-off, over the stance foot (the constraint the host checks)
        for i, p in enumerate(steps):
            seg = min(segs[1:], key=lambda sg: abs(sg["t"] - p["lo"]))
            st = p["stance"]
            sg = 1.0 if st == "L" else -1.0
            fst = seg["pa"]["feet"][st][1]
            self.liftoff.append(dict(i=i, t=p["lo"], stance=st, ratio=sg * (seg["pa"]["pel"] - fst) / self.oy,
                                     need=self.k[st]))
        self.segs, self.phases, self.first = segs, phases, first

    def phase_at(self, t):
        for p in self.phases:
            if p["t0"] <= t <= p["t1"]:
                return p
        return None

    def phi(self, t):
        """The gait phase for the firmware: u = 0.25 at lift-off, 0.75 at touchdown (linear between), 0.75 ->
        1.25 across the double support up to the next lift-off; half cycle k = step k, the left foot swings
        in the half cycles where sin(phi) > 0."""
        ps = self.phases
        off = 0.0 if self.first == "L" else math.pi
        if t <= ps[0]["t0"]:
            # before the first lift-off: back from u = 0.25 over one double support
            d = ps[0]["t0"] - (ps[0]["t0"] - (self.t_shift + self.t_settle))
            u = 0.25 - 0.5 * min(1.0, (ps[0]["t0"] - t) / max(d, 1e-6))
            return math.pi * u + off
        for k, p in enumerate(ps):
            if p["t0"] <= t <= p["t1"]:
                u = 0.25 + 0.5 * (t - p["t0"]) / (p["t1"] - p["t0"])
                return math.pi * (k + u) + off
            nxt = ps[k + 1]["t0"] if k + 1 < len(ps) else p["t1"] + (self.t_shift + self.t_settle)
            if p["t1"] < t < nxt:
                u = 0.75 + 0.5 * min(1.0, (t - p["t1"]) / max(nxt - p["t1"], 1e-6))
                return math.pi * (k + u) + off
        return math.pi * (len(ps) - 1 + 1.25) + off
