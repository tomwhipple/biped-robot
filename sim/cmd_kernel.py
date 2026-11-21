"""The operator command's kernels, shared by the training envs and pinned to the firmware (#263, #266).

Two pieces, each written with array operators only, so the same function runs under numpy
(sim/walker_env.py) and jax.numpy (sim/mjx/env_mjx.py), on a scalar or a batch -- the pattern of
sim/trim_kernel.py and walker_env.roll_play_substep.

The SLEW (#266). A per-channel rate limit on the planar command (vx, vy, wz) ahead of the
observation: each control tick a channel moves toward its target by at most rate x dt, and lands on
the target exactly once it is within that step. So a stick step reaches the policy as a ramp. The
envs' `cmd_slew` = (vx m/s^2, vy m/s^2, wz rad/s^2), recorded in the run's config.json; the
firmware's twin (firmware/components/cmdslew) takes its rates from the deployed run through the
generated command envelope (tools/gen_envelope.py), and firmware/host/test_cmdslew.cpp pins it to
GOLDEN below.

    cmd[:3] = slew_step(cmd[:3], target, slew_delta(rates, dt), xp)

The VOLUME draw (#263). One planar command drawn from the whole envelope rather than from six
cardinal bands: a direction uniform on the compass and a speed uniform from zero to the envelope's
edge along that direction (an ellipse with its forward and backward semi-axes apart), so there is
no dead band and every diagonal is drawn; the turn is independent, zero (straight walk) or uniform
on [-w, w], so back + turn and side + turn are drawn too. The envs map their own uniforms
(np.random / jax.random) through volume_cmd, so the law is the same function in both.

    vx, vy, wz = volume_cmd(u_r, u_th, u_turn, u_w, axes, xp)
"""
from __future__ import annotations

import math

import numpy as np

# The fraction of volume draws that turn (the legacy forward draw's 0.6).
VOLUME_P_TURN = 0.6


def parse_slew(spec):
    """cmd_slew from a kwarg, config.json or the CLI: None / "" / 0 -> None (off); "a,b,c" or a
    3-sequence -> (vx, vy, wz) rates, each > 0."""
    if spec is None or (isinstance(spec, str) and not spec.strip()):
        return None
    if isinstance(spec, str):
        spec = [float(x) for x in spec.split(",")]
    rates = tuple(float(x) for x in spec)
    if len(rates) != 3 or not all(r > 0.0 and math.isfinite(r) for r in rates):
        raise ValueError(f"cmd_slew needs three positive rates (vx m/s^2, vy m/s^2, wz rad/s^2), "
                         f"got {spec!r}")
    return rates


def slew_delta(rates, dt):
    """The per-tick step of each channel, in float32 (the firmware's arithmetic)."""
    return np.asarray(rates, dtype=np.float32) * np.float32(dt)


def slew_step(cur, tgt, delta, xp=np):
    """One control tick of the slew: each channel of `cur` moves toward `tgt` by at most `delta`, and
    takes `tgt` exactly when it is within one step (no float creep at the end of a ramp)."""
    d = tgt - cur
    return xp.where(xp.abs(d) <= delta, tgt, cur + xp.sign(d) * delta)


def volume_axes(cmd_v_range, cmd_back_range, cmd_vy_range, cmd_w_range):
    """The envelope's semi-axes (forward, backward, lateral, turn), magnitudes, from the env's
    command ranges: the volume reaches each band's far edge (tools/gen_envelope.py bands())."""
    return (float(cmd_v_range[1]), float(max(abs(x) for x in cmd_back_range)),
            float(cmd_vy_range), float(cmd_w_range))


def volume_cmd(u_r, u_th, u_turn, u_w, axes, xp=np):
    """(vx, vy, wz) from four U(0, 1) draws: u_th the compass direction, u_r the fraction of the
    envelope's reach in that direction (uniform: continuous down to zero), u_turn whether it turns
    (VOLUME_P_TURN), u_w the turn rate (uniform on [-w, w])."""
    fwd, back, lat, w = axes
    th = 2.0 * math.pi * u_th
    c, s = xp.cos(th), xp.sin(th)
    vx = u_r * c * xp.where(c >= 0.0, fwd, back)
    vy = u_r * s * lat
    wz = xp.where(u_turn < VOLUME_P_TURN, (2.0 * u_w - 1.0) * w, 0.0)
    return vx, vy, wz


# The trajectory the firmware's host test pins (firmware/host/test_cmdslew.cpp): rates, tick, the
# targets the stick asks for and the tick each is set at, starting from rest. golden() runs it.
GOLDEN_RATES = (0.5, 0.4, 1.5)
GOLDEN_DT = 0.02
GOLDEN_STEPS = ((0, (0.24, -0.25, 0.8)),     # a full-stick diagonal with a turn
                (20, (-0.4, 0.0, -1.0)),     # reversed before it arrived
                (55, (0.0, 0.0, 0.0)),       # let go: back to the stand
                (90, (0.013, 0.0, 0.0)))     # a target inside one step


def golden(n=120):
    """The slewed (vx, vy, wz) after each of n ticks of GOLDEN_STEPS, float32."""
    delta = slew_delta(GOLDEN_RATES, GOLDEN_DT)
    cur = np.zeros(3, np.float32)
    tgt = np.zeros(3, np.float32)
    steps = dict(GOLDEN_STEPS)
    out = []
    for k in range(n):
        if k in steps:
            tgt = np.asarray(steps[k], np.float32)
        cur = slew_step(cur, tgt, delta).astype(np.float32)
        out.append(cur.copy())
    return np.asarray(out, np.float32)
