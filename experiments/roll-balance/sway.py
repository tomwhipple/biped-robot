"""A2 (#297): the upper layer's lateral motion for speed. A continuous LIPM-style sway instead of the
stepper's quasi-static shift-then-raise, so the step time can come down.

One step takes T s (a march or walk of n steps; one foot swings per step). The pelvis rides a lateral
sine of period 2T and amplitude k x the model's 62 mm (kin.model_offset), peaking over the stance foot at
the middle of that foot's single support:

    y_pel(t) = k * 62 mm * sin(pi * (t - t_c) / T)          t_c = the first step's centre - T/2
    single support = ss * T centred on the peak (+ lead: the lift starts `lead` x T before that window)
    swing foot z   = h * sin^2(pi u), u = 0..1 over the single support
    swing foot x, y move (min-jerk) over the middle of the single support (dx forward; dy outward only,
                   as the stepper: the leading foot steps out, the other follows)
    turn: the swing hip yaw goes to +-yaw over the single support (the swing foot lands rotated), the
          stance hip yaw follows back during the next double support

LIPM gives the CoM amplitude for alternating support every T s as (w/2)(1 - sech(omega T / 2)),
omega = sqrt(g / z0); with z0 0.213 m that is 0.93 of the static shift at T = 1 s and 0.65 at 0.5 s.
`k=None` uses it; any other value overrides. wide=True adds the stance foot's offset from the feet's mid
line to the amplitude (a side step widens the stance; B2 SIL: without it the slow side step leans into
the swing). The object has testbed.Traj's interface.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import synth as S  # noqa: E402

Z0, G = 0.213, 9.81


def k_lipm(T):
    w = math.sqrt(G / Z0)
    return 1.0 - 1.0 / math.cosh(w * T / 2.0)


def _mj(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


class Sway:
    def __init__(self, T=1.5, n_steps=8, k=None, ss=0.6, lead=0.0, h=0.025, dx=0.0, dy=0.0, yaw=0.0,
                 first="L", drop=0.02, t_crouch=2.0, t_settle=1.0, dt=0.04, kL=None, kR=None, wide=False):
        K = S.kin(drop)
        self.k = k_lipm(T) if k is None else k
        # per stance side (the robot needs more shift over its left foot); the amplitude switches where the
        # sine crosses zero (mid double support), so the pelvis path stays continuous
        kside = {"L": self.k if kL is None else kL, "R": self.k if kR is None else kR}
        self.stamp = f"sway T{T:g} k{self.k:.2f}/{kL}/{kR}{' wide' if wide else ''} ss{ss:g} lead{lead:g} dx{dx:+g} dy{dy:+g} yaw{yaw:+g}"
        self.r = None
        oy = abs(K.model_offset("L")[1])
        segs, phases = [], []
        self.q_start = K.pose(straight=True)
        q = K.pose()
        segs.append(dict(t=0.0, ms=1e3 * t_crouch, q=q, row=dict(tag="crouch", feet={"L": [0, 0, 0], "R": [0, 0, 0]})))
        t0 = t_crouch + t_settle
        # the pelvis sways one full half period before the first lift (its sine ramps in over the first
        # half of that): the first single support starts with the CoM already swinging, as every later one
        t_c = t0 + 1.5 * T                       # the first step's single-support centre
        t_end = t_c + (n_steps - 0.5) * T + 0.5 * T
        feet = {"L": [0.0, 0.0, 0.0], "R": [0.0, 0.0, 0.0]}
        start = {"L": [0.0, 0.0], "R": [0.0, 0.0]}
        goal = {"L": [0.0, 0.0], "R": [0.0, 0.0]}
        yawq = {"L": 0.0, "R": 0.0}
        swing_of = lambda i: first if i % 2 == 0 else ("R" if first == "L" else "L")
        for i in range(n_steps):
            sw = swing_of(i)
            st = "R" if sw == "L" else "L"
            c = t_c + i * T
            a, b = c - 0.5 * ss * T - lead * T, c + 0.5 * ss * T
            phases.append(dict(t0=a, raise_end=a + 0.35 * (b - a), hold_from=a + 0.35 * (b - a),
                               lower_start=a + 0.65 * (b - a), t1=b, swing=sw, stance=st, ss_a=a, ss_b=b))
        t = t0
        while t <= t_end + 1e-9:
            # pelvis: sine toward the stance foot of the current step (L stance = +y)
            env = min(1.0, max(0.0, (t - t0) / (0.5 * T)))          # the first half period ramps it in
            tail = min(1.0, max(0.0, (t_end - t) / (0.5 * T)))
            # step index whose centre is nearest
            i = int(round((t - t_c) / T))
            i = min(max(i, 0), n_steps - 1)
            sw = swing_of(i)
            stance = "R" if sw == "L" else "L"
            sgn_first = 1.0 if first == "R" else -1.0     # first swing L -> first stance R -> pelvis to -y
            y_sin = math.sin(math.pi * (t - t_c) / T + math.pi / 2) * sgn_first
            side = "L" if y_sin > 0 else "R"
            amp = kside[side] * oy
            if wide:
                # the stance foot's own offset from the feet's mid line (a side step widens the stance): the
                # pelvis peaks that much further out (continuous: the side switches where the sine is 0)
                ymid = 0.5 * (feet["L"][1] + feet["R"][1])
                amp += (1.0 if side == "L" else -1.0) * (feet[side][1] - ymid)
            y = amp * y_sin * min(env, tail)
            # feet
            for j, ph in enumerate(phases):
                fsw = ph["swing"]
                if ph["ss_a"] <= t <= ph["ss_b"]:
                    u = (t - ph["ss_a"]) / (ph["ss_b"] - ph["ss_a"])
                    if not ph.get("_init"):
                        ph["_init"] = True
                        start[fsw] = [feet[fsw][0], feet[fsw][1]]
                        fst = ph["stance"]
                        out = (dy > 0 and fsw == "L") or (dy < 0 and fsw == "R")
                        goal[fsw] = [feet[fst][0] + dx, feet[fst][1] + (dy if out else 0.0)]
                    feet[fsw][2] = h * math.sin(math.pi * u) ** 2
                    m = _mj((u - 0.2) / 0.6)
                    feet[fsw][0] = start[fsw][0] + m * (goal[fsw][0] - start[fsw][0])
                    feet[fsw][1] = start[fsw][1] + m * (goal[fsw][1] - start[fsw][1])
                    yawq[fsw] = (yaw if (j % 2 == 0) else -yaw) * m if yaw else 0.0
                elif t > ph["ss_b"] and ph.get("_init") and not ph.get("_done"):
                    ph["_done"] = True
                    feet[fsw][2] = 0.0
            x_pel = 0.5 * (feet["L"][0] + feet["R"][0])
            ybase = 0.5 * (feet["L"][1] + feet["R"][1])
            qd = K.pose(shift=ybase + y, fwd=x_pel, footL=tuple(feet["L"]), footR=tuple(feet["R"]))
            if yaw:
                qd["L_hip_yaw"] += yawq["L"]
                qd["R_hip_yaw"] += yawq["R"]
            # pa: the pose's arguments without the sway (the centred stand for these feet: a stop's rest pose)
            segs.append(dict(t=t, ms=1e3 * dt, q=qd, row=None,
                             pa=dict(fwd=x_pel, shift=ybase, feet={s: list(feet[s]) for s in feet}, yaw=dict(yawq))))
            t += dt
        for p in phases:
            p.pop("_init", None)
            p.pop("_done", None)
        self.segs, self.phases, self.t_end = segs, phases, t_end + 1.0

    def phase_at(self, t):
        for p in self.phases:
            if p["t0"] <= t <= p["t1"]:
                return p
        return None
