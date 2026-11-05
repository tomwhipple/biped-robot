"""The firmware's per-joint MOTION LIMITER (obs::MotionLimiter,
firmware/components/obs/actuation.cpp) for the sims: env_mjx (training, JAX),
walker_env (the CPU referee) and experiments/policy/rca/simh.py (eval).

Each 20 ms tick, per joint, after the C2 shaper and before the goal ticks:

    e  = target - p
    vb = min(vmax, brake_speed(|e|, amax, dt))   # braking look-ahead
    v += clip(sign(e) * vb - v, -amax dt, +amax dt)
    p += v dt

brake_speed is the exact DISCRETE stopping curve: the largest speed from which
braking by amax*dt per tick (p += v dt) stops exactly on the target, so a joint
reaches a target that holds still at zero velocity and never passes it. With
jmax > 0 the output is a boxcar (moving average) of the newest `taps` =
ceil(2 amax / (jmax dt)) ticks of (p, v): an S-curve with |jerk| <= jmax that
keeps the vmax / amax / no-overshoot bounds. The goal speed the firmware writes
with a limited target is |v_out| x 1.25 in steps/s (floor 50, cap 3400).

The arithmetic here is float32 in the firmware's operation order, so the numpy
copy matches the compiled one to the bit in practice; tests/test_fw_limit.py
pins env_mjx, walker_env and this module to the compiled code through the SIL
library's sil_limit_trace. Units: the CLI's and the flags' deg (deg/s, deg/s^2,
deg/s^3); the state is rad.
"""
from __future__ import annotations

import numpy as np

MAX_TAPS = 16                                   # obs::kLimitMaxTaps
DEG = np.float32(0.017453292519943295)          # obs::kDegToRad
DT = np.float32(0.02)                           # obs::kControlDt
TICK_RAD = 2.0 * np.pi / 4096.0                 # one encoder step, rad
SPEED_HEADROOM = 1.25                           # obs::kGoalSpeedHeadroom
SPEED_FLOOR = 50.0                              # obs::kGoalSpeedFloor, steps/s
SPEED_MAX = 3400.0                              # obs::kGoalSpeedMax, steps/s
# `limit on` (obs::kLimitVmaxDeg / kLimitAmaxDeg / kLimitJmaxDeg)
DEFAULT = (200.0, 2000.0, 0.0)


def parse(spec):
    """'vmax,amax[,jmax]' (deg units), a sequence of 2-3 numbers, or None/'' (off)
    -> (vmax, amax, jmax) floats, or None when off (vmax or amax <= 0)."""
    if spec is None:
        return None
    if isinstance(spec, str):
        spec = spec.strip()
        if spec in ("", "off", "0"):
            return None
        vals = [float(x) for x in spec.split(",")]
    else:
        vals = [float(x) for x in spec]
    if len(vals) not in (2, 3):
        raise ValueError(f"fw_limit: vmax,amax[,jmax] (deg units), got {spec!r}")
    v, a = vals[0], vals[1]
    j = vals[2] if len(vals) == 3 else 0.0
    if v <= 0.0 or a <= 0.0:
        return None
    return (v, a, max(j, 0.0))


def to_rad(cfg):
    """(vmax, amax, jmax) deg -> float32 rad, the way ctrl_task converts."""
    return tuple(np.float32(np.float32(x) * DEG) for x in cfg)


def taps(amax, jmax, dt=DT):
    """obs::limitTaps, rad units, float32: 1 without a jerk limit, else
    ceil(2 amax / (jmax dt) - 1e-3), capped at MAX_TAPS."""
    amax, jmax, dt = np.float32(amax), np.float32(jmax), np.float32(dt)
    if not (jmax > 0 and amax > 0 and dt > 0):
        return 1
    n = np.ceil(np.float32(2.0) * amax / (jmax * dt) - np.float32(1e-3))
    return int(min(max(n, 1.0), MAX_TAPS))


def brake_speed(dist, amax, dt, xp=np):
    """obs::limitBrakeSpeed: the largest speed toward a target `dist` rad away
    from which braking by amax*dt per tick stops exactly on it. dist >= 0;
    array-valued; xp = numpy or jax.numpy."""
    f = xp.float32
    u = f(amax) * f(dt) * f(dt)
    m = dist / u
    k = xp.floor(f(0.5) * (xp.sqrt(f(1.0) + f(8.0) * m) - f(1.0)))
    k = xp.maximum(k, f(0.0))
    rho = (m - f(0.5) * k * (k + f(1.0))) / (k + f(1.0))
    vb = f(amax) * f(dt) * (k + rho)
    return xp.where(dist > 0, vb, f(0.0))


def limit_step(p, v, x, vmax, amax, dt, xp=np):
    """One tick of the limiter's core (no boxcar): (p, v) -> (p', v')."""
    f = xp.float32
    e = x - p
    vb = xp.minimum(brake_speed(xp.abs(e), amax, dt, xp), f(vmax))
    vd = xp.where(e > 0, vb, xp.where(e < 0, -vb, f(0.0)))
    dvm = f(amax) * f(dt)
    v = v + xp.clip(vd - v, -dvm, dvm)
    p = p + v * f(dt)
    return p, v


def boxcar(hist, xp=np):
    """Mean of the newest-first history rows, summed newest first like the
    firmware (a python loop: taps <= 16)."""
    s = hist[0]
    for t in range(1, hist.shape[0]):
        s = s + hist[t]
    return s * (xp.float32(1.0) / xp.float32(hist.shape[0]))


def goal_speed_steps(vel, xp=np):
    """obs::limitGoalSpeedSteps: |v| x 1.25 in steps/s, floor 50, cap 3400,
    rounded half to even (lrintf)."""
    sps = xp.abs(vel) / xp.float32(TICK_RAD) * xp.float32(SPEED_HEADROOM)
    return xp.round(xp.clip(sps, SPEED_FLOOR, SPEED_MAX))


class MotionLimiter:
    """numpy copy of obs::MotionLimiter. cfg = (vmax, amax, jmax) in deg units
    (parse()). reset(q) seeds at q at rest; __call__(target) -> (pos, vel), rad
    and rad/s, float64 copies of the float32 state."""

    def __init__(self, cfg, dt=DT):
        self.cfg = tuple(float(x) for x in cfg)
        self.vmax, self.amax, self.jmax = to_rad(self.cfg)
        self.dt = np.float32(dt)
        self.taps = taps(self.amax, self.jmax, self.dt)
        self.hp = self.hv = None

    def reset(self, q):
        q = np.asarray(q, np.float32)
        self.hp = np.tile(q, (self.taps, 1))
        self.hv = np.zeros_like(self.hp)

    @property
    def primed(self):
        return self.hp is not None

    def __call__(self, target):
        x = np.asarray(target, np.float32)
        if self.hp is None:
            self.reset(x)
        p, v = limit_step(self.hp[0], self.hv[0], x, self.vmax, self.amax, self.dt)
        self.hp = np.concatenate([p[None], self.hp[:-1]], axis=0)
        self.hv = np.concatenate([v[None], self.hv[:-1]], axis=0)
        if self.taps == 1:
            return p.astype(np.float64), v.astype(np.float64)
        return (boxcar(self.hp).astype(np.float64),
                boxcar(self.hv).astype(np.float64))
