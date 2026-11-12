"""The stance trim on a gait clock, vectorised for the training envs (#297 B2, the policy path).

One control tick of firmware/components/gaitplay supportFromPhase + balance::TrimDriver +
balance::StanceTrim, written with array operators only, so the same function runs under numpy
(walker_env) and jax.numpy (env_mjx), over a batch or a scalar. The scalar twin of the trim itself is
sim/stance_trim.py; this file adds the phase -> support map and the driver, and is pinned to the C++
through firmware/host/sil_gait.cpp (`sil_trim_*`, tests/test_trim_kernel.py).

State (a dict of arrays, all float): side (+1 L stance, -1 R, 0 none), mode (0 off, 5 closed, 6 hold,
7 fade, 8 abort tip), cur (the driver's current stance, +1/-1/0), ref, err, hike, frozen, since,
T_L, T_R. Out: the stance hip roll joint correction, sign(side) * hike, on the hip of `side`.

    s = trim_init(shape, T_L, T_R)
    s, hip_L, hip_R = trim_step(s, roll, phase, lift, dt, P, xp)
"""
from __future__ import annotations

import math

import numpy as np

OFF, CLOSED, HOLD, FADE, ABORT_TIP = 0.0, 5.0, 6.0, 7.0, 8.0
TWO_PI = 2.0 * math.pi
DEFAULTS = dict(kp=0.25, ki=1.0, lim=math.radians(15.0), t_lim=math.radians(14.0), fade_s=0.6,
                abort=math.radians(15.0))
KEYS = ("side", "mode", "cur", "ref", "err", "hike", "frozen", "since", "T_L", "T_R")


def trim_init(shape=(), T_L=0.0, T_R=0.0, xp=np):
    z = xp.zeros(shape, dtype=xp.float32)
    s = {k: z for k in KEYS}
    s["T_L"] = z + T_L
    s["T_R"] = z + T_R
    return s


def up_from_quat(q, xp=np):
    """The firmware's `up` from the root quaternion (w, x, y, z): the torso's z axis in the world, yaw
    stripped (x forward, y left; tools/flightrec, gate_common.up_of); roll = -asin(up_y), + = leaning
    to the robot's right. From qpos, not sensordata: a CPU env's sensordata after mj_step is one substep
    old, MJX's (and a re-synced state's) is not, and the trim must see the same roll in both."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    zx, zy, zz = 2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)
    psi = xp.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    c, s = xp.cos(psi), xp.sin(psi)
    return xp.stack([c * zx + s * zy, -s * zx + c * zy, zz])


def support(phase, lift, xp=np):
    """gaitplay::supportFromPhase: (stance +1 L / -1 R / 0, raise progress 0..1, lowering)."""
    p = xp.mod(phase, TWO_PI)
    left_swings = p < math.pi
    u = xp.where(left_swings, p, p - math.pi) / math.pi
    ss = lift & (u >= 0.25) & (u <= 0.75)
    w = (u - 0.25) / 0.5
    stance = xp.where(ss, xp.where(left_swings, -1.0, 1.0), 0.0)
    progress = xp.where(w >= 0.35, 1.0, w / 0.35)
    return stance, progress, w >= 0.65


def _update(s, roll, dt, ramp, P, xp):
    """StanceTrim::update for every element (finite roll; the sims have no IMU dropout)."""
    mode = s["mode"]
    closed, hold = mode == CLOSED, mode == HOLD
    fading = (mode == FADE) | (mode == ABORT_TIP)
    e = roll - s["ref"]
    err = xp.where(closed | hold | fading, e, s["err"])
    # fade / abort: linear to 0
    since = xp.where(fading, s["since"] + dt, s["since"])
    k = xp.maximum(0.0, 1.0 - since / P["fade_s"])
    done = fading & (k <= 1e-4)
    hike = xp.where(fading, s["frozen"] * k, s["hike"])
    # closed: the law, the abort
    trip = closed & (xp.abs(e) > P["abort"])
    sg = xp.where(s["side"] < 0, -1.0, 1.0)
    T_side = xp.where(s["side"] < 0, s["T_R"], s["T_L"])
    r = xp.clip(ramp, 0.0, 1.0)
    unsat = r * T_side + P["kp"] * sg * e
    learn = closed & ~trip & (xp.abs(unsat) < P["lim"])
    T_new = xp.clip(T_side + P["ki"] * sg * e * dt, -P["t_lim"], P["t_lim"])
    T_L = xp.where(learn & (s["side"] > 0), T_new, s["T_L"])
    T_R = xp.where(learn & (s["side"] < 0), T_new, s["T_R"])
    law = closed & ~trip
    frozen = xp.where(trip, s["hike"], s["frozen"])
    since = xp.where(trip, 0.0, since)
    hike = xp.where(law, xp.clip(unsat, -P["lim"], P["lim"]), hike)
    mode = xp.where(trip, ABORT_TIP, mode)
    mode = xp.where(done, OFF, mode)
    hike = xp.where(done, 0.0, hike)
    side = xp.where(done, 0.0, s["side"])
    out = dict(s)
    out.update(mode=mode, err=err, since=since, hike=hike, frozen=frozen, side=side, T_L=T_L, T_R=T_R)
    return out


def trim_step(s, roll, phase, lift, dt, P=None, xp=np):
    """One tick of TrimDriver::step on the support state from the phase. -> (state, hip_L, hip_R): the
    joint corrections (rad) for the left and the right hip roll."""
    P = P or DEFAULTS
    stance, progress, lowering = support(phase, lift, xp)
    cur = s["cur"]
    # the single support is over (double support or the other foot): release (fade)
    ended = (cur != 0.0) & (stance != cur)
    live = (s["mode"] == CLOSED) | (s["mode"] == HOLD)
    rel = ended & live
    s = dict(s)
    s["frozen"] = xp.where(rel, s["hike"], s["frozen"])
    s["since"] = xp.where(rel, 0.0, s["since"])
    s["mode"] = xp.where(rel, FADE, s["mode"])
    cur = xp.where(ended, 0.0, cur)
    # a new single support begins a chain, the roll now its reference
    begin = (stance != 0.0) & (cur == 0.0)
    for k, v in (("side", stance), ("ref", roll), ("err", 0.0), ("hike", 0.0), ("frozen", 0.0), ("since", 0.0),
                 ("mode", CLOSED)):
        s[k] = xp.where(begin, v, s[k])
    cur = xp.where(begin, stance, cur)
    s["cur"] = cur
    in_chain = cur != 0.0
    s["mode"] = xp.where(in_chain & lowering & (s["mode"] == CLOSED), HOLD, s["mode"])
    s = _update(s, roll, dt, xp.where(in_chain, progress, 1.0), P, xp)
    on = s["mode"] != OFF
    hip = xp.where(on, xp.where(s["side"] < 0, -1.0, 1.0) * s["hike"], 0.0)
    return s, xp.where(on & (s["side"] > 0), hip, 0.0), xp.where(on & (s["side"] < 0), hip, 0.0)
