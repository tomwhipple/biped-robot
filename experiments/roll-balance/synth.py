"""Synthetic stepping trajectories for the testbed: the stepper's phases (shift over the stance foot, raise,
swing, lower) built with its own IK (experiments/stepper/kin.py) at any step period and direction, so the
balance laws can be tried at step rates the host stepper never reached and in every direction.

One step = shift (pelvis k x the model's 62 mm over the stance sole) -> raise the swing foot h -> move it
(dx, dy) -> lower it -> (the next step shifts over the other foot). Phase durations are fractions of the
step period P: shift 0.3, raise 0.2, swing 0.25, lower 0.25. The pose commands are min-jerk moves of those
durations, as the firmware streams them. The object has Traj's interface (segs, phases, phase_at; r=None).
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))

FR = dict(shift=0.3, raise_=0.2, swing=0.25, lower=0.25)
_KIN = {}


def kin(drop=0.02):
    if drop not in _KIN:
        from kin import Kin
        _KIN[drop] = Kin(drop)
    return _KIN[drop]


class Synth:
    def __init__(self, period=4.0, n_steps=6, dx=0.0, dy=0.0, h=0.025, k_shift=1.15, first="L", drop=0.02,
                 t_crouch=2.0, t_settle=1.0):
        K = kin(drop)
        self.stamp = f"synth P{period:g} dx{dx:+g} dy{dy:+g} h{h:g}"
        self.r = None
        feet = {"L": [0.0, 0.0, 0.0], "R": [0.0, 0.0, 0.0]}
        pel = [0.0, 0.0]
        segs, phases = [], []
        t = 0.0

        def pose(ms, tag, row_feet, pel_):
            q = K.pose(shift=pel_[1], fwd=pel_[0], footL=tuple(row_feet["L"]), footR=tuple(row_feet["R"]))
            segs.append(dict(t=t, ms=ms, q=q, row=dict(tag=tag, feet={k: list(v) for k, v in row_feet.items()})))

        self.q_start = K.pose(straight=True)
        pose(1e3 * t_crouch, "crouch", feet, pel)
        t += t_crouch + t_settle
        swing = first
        for i in range(n_steps):
            stance = "R" if swing == "L" else "L"
            ox, oy = K.model_offset(stance)
            # shift over the stance sole, wherever it stands; fore-aft between the feet
            pel = [0.75 * feet[stance][0] + 0.25 * feet[swing][0] + ox, feet[stance][1] + k_shift * oy]
            d = FR["shift"] * period
            pose(1e3 * d, f"shift {stance}", feet, pel)
            t += d
            ph = dict(t0=t, swing=swing, stance=stance)
            feet[swing][2] = h
            d = FR["raise_"] * period
            pose(1e3 * d, f"raise {swing}", feet, pel)
            t += d
            ph["raise_end"] = t
            # move the swing foot: forward dx per step (the first step half), sideways only outward
            step_dx = dx * (1.0 if i == 0 else 2.0)
            out = (dy > 0 and swing == "L") or (dy < 0 and swing == "R")
            feet[swing][0] += step_dx
            feet[swing][1] = feet[stance][1] + (dy if out else 0.0)
            d = FR["swing"] * period
            pose(1e3 * d, f"swing {swing}", feet, pel)
            t += d
            ph["lower_start"] = t
            feet[swing][2] = 0.0
            d = FR["lower"] * period
            pose(1e3 * d, f"lower {swing}", feet, pel)
            t += d
            ph["t1"] = t
            phases.append(ph)
            swing = stance
        # centre and hold
        pel = [0.5 * (feet["L"][0] + feet["R"][0]), 0.5 * (feet["L"][1] + feet["R"][1])]
        pose(1e3 * 1.5, "centre", feet, pel)
        t += 2.5
        self.segs, self.phases, self.t_end = segs, phases, t

    def phase_at(self, t):
        for p in self.phases:
            if p["t0"] <= t <= p["t1"]:
                return p
        return None
