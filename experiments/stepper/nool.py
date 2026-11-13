"""Nool: a probability where a controller would want a bool.

The robot has no contact sensors and its roll joints have several degrees of play the encoders cannot
see, so "the foot is off the floor" is never known, only estimated. Every such question returns a Nool:
the probability, how much to trust it, and the numbers it came from (after the Jev-style return of the
retired gradebook design: a typed answer with probability, confidence and evidence)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-max(-40.0, min(40.0, x))))


@dataclass
class Nool:
    p: float                      # P(the statement is true)
    conf: float = 1.0             # how much the estimator trusts its own p (0 = a guess)
    why: dict = field(default_factory=dict)   # the evidence

    def __bool__(self):
        raise TypeError("a Nool is not a bool: compare .p with a threshold")

    def __and__(self, o: "Nool") -> "Nool":
        return Nool(self.p * o.p, min(self.conf, o.conf), {**self.why, **o.why})

    def __or__(self, o: "Nool") -> "Nool":
        return Nool(1 - (1 - self.p) * (1 - o.p), min(self.conf, o.conf), {**self.why, **o.why})

    def __invert__(self) -> "Nool":
        return Nool(1 - self.p, self.conf, self.why)

    def row(self):
        return dict(p=round(self.p, 3), conf=round(self.conf, 2), **{k: (round(v, 3) if isinstance(v, float) else v)
                                                                   for k, v in self.why.items()})


def above(x: float, at: float, width: float, conf: float = 1.0, **why) -> Nool:
    """P(x is really above `at`), for a reading good to about +-width."""
    return Nool(sigmoid((x - at) / max(width, 1e-9) * 1.8), conf, dict(x=x, at=at, **why))


# -- the estimators -----------------------------------------------------------
FOOT_SEP = 0.121      # m between the sole centres (v6 legs)
SOLE_HALF = 0.042     # m, half the sole width


def clearance_m(h_cmd: float, tilt_to_swing_deg: float, lever: float = FOOT_SEP + SOLE_HALF) -> float:
    """Where the swing sole's OUTER edge is above the floor: the commanded lift less what the body's
    tilt toward that foot gives back. (2026-11-11 clamp tests: a 15 mm lift with 7.0 deg of tilt left
    the outer edge on the desk.)"""
    return h_cmd - lever * math.sin(math.radians(tilt_to_swing_deg))


def foot_clear(h_cmd, tilt_to_swing_deg, need=0.004, sigma=0.005, bias=0.0) -> Nool:
    c = clearance_m(h_cmd, tilt_to_swing_deg) - bias
    # the estimate is geometry on a tilt the IMU reads well; what it cannot see is the stance foot
    # rocking on its own edge, so trust falls as the tilt grows
    conf = max(0.2, 1.0 - abs(tilt_to_swing_deg) / 12.0)
    return above(c, need, sigma, conf, clear_mm=1e3 * c, h_mm=1e3 * h_cmd, tilt=tilt_to_swing_deg)


def tipping(tilt_to_swing_deg, pitch_dev_deg, rate_dps, tilt_lim=6.0, pitch_lim=6.0, rate_lim=25.0,
            out_lim=None) -> Nool:
    """P(the robot is on its way over): too far toward the lifted foot, too far the other way
    (`out_lim`, the dangerous side: nothing is there to catch it), pitching, or simply moving fast."""
    a = above(tilt_to_swing_deg, tilt_lim, 1.0, tilt=tilt_to_swing_deg)
    b = above(-tilt_to_swing_deg, tilt_lim if out_lim is None else out_lim, 1.0)
    c = above(abs(pitch_dev_deg), pitch_lim, 1.0, pitch=pitch_dev_deg)
    d = above(abs(rate_dps), rate_lim, 5.0, rate=rate_dps)
    return a | b | c | d


def sag_deg(h_cmd: float, lever: float = FOOT_SEP + SOLE_HALF) -> float:
    """The body roll (deg) a foot still ON the floor takes with it when its leg is shortened by h_cmd:
    the stance side's roll play lets the pelvis follow the foot down (2026-11-11, left stance: the
    measured tilt tracked this within 0.4 deg all the way to 20 mm, so that foot never left the desk)."""
    return math.degrees(math.asin(max(-1.0, min(1.0, h_cmd / lever))))


def settled(rate_dps, lim=3.0) -> Nool:
    return ~above(abs(rate_dps), lim, 1.5, rate=rate_dps)
