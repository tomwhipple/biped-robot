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

The volume CLAMP (#263). The robot holds a command to what a volume-trained run drew: the planar
(vx, vy) inside volume_cmd's ellipse, the turn independently in [-w, w]. Per-axis limits alone pass
a corner like (0.24, 0.25), normalized radius sqrt(2), that no draw reached. volume_clamp scales a
command past the ellipse back along its own direction onto it; firmware/components/linkproto
(clampToVolume) and link/protocol.py (clamp_to_volume) are its twins, and
firmware/host/test_protocol.cpp pins the C++ bit for bit to volume_golden().

    vx, vy = volume_clamp(vx, vy, axes, xp)

The STAND BAND (#263). The reward, the gait clock, the stance trim and the firmware's stand gates
read the planar command (c0, c1, c2) as the plain stand while each |c| is at or below a band, and
as a walk above it. The six cardinal bands never draw a walk under 0.05, so STAND_BAND (0.05) cost
nothing there; the volume draws speeds continuously down to zero, and with that band 8.8 % of its
walks were graded as stands (a dead band the stick passes through). The band for a volume run is
STAND_BAND_VOLUME, set explicitly (`--cmd-stand-band 0.0005`; never implied by --cmd-volume, so a
volume arm and a band arm stay separate changes): half the link's resolution (link/protocol.py sends c0..c2 in milli-units),
so a command reaches the policy as a walk exactly when the robot receives it as non-zero, and the
stand is the zero command (the stick's dead zone, a released ENABLE, a lost link, the slew landed).
The envs' `cmd_stand_band`, recorded in the run's config.json; tools/gen_obs_spec.py puts a band
other than STAND_BAND into obs_spec.h (obs::kStandBand) for the firmware's gates.

CommandSlew is the firmware's cmdslew::CommandSlew in Python (float32, reset to the stand), for the
harnesses that run the firmware's command path in its order: experiments/policy/shaper-train/
grade.py and sim/sil_twin.py.
"""
from __future__ import annotations

import math

import numpy as np

# The fraction of volume draws that turn (the legacy forward draw's 0.6).
VOLUME_P_TURN = 0.6

# The stand band (module docstring): the legacy one, and a volume run's (half of link/protocol.py's
# milli-unit command resolution).
STAND_BAND = 0.05
STAND_BAND_VOLUME = 0.0005


def parse_stand_band(spec):
    """cmd_stand_band from a kwarg, config.json or the CLI: None -> STAND_BAND; else a finite
    band >= 0 (m/s for c0, c1; rad/s for c2)."""
    if spec is None:
        return STAND_BAND
    b = float(spec)
    if not (b >= 0.0 and math.isfinite(b)):
        raise ValueError(f"cmd_stand_band needs a finite band >= 0, got {spec!r}")
    return b


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


class CommandSlew:
    """firmware/components/cmdslew CommandSlew in Python: slew_step on (vx, vy, wz) in float32,
    starting from (and reset to) the stand. ctrl_task.cpp steps it on the watchdog's command each
    control tick, before the clock and the observation, and resets it on every (re)arm and every
    limp tick; a harness that runs the firmware's command path does the same with this."""

    def __init__(self, rates, dt):
        self.delta = slew_delta(rates, dt)
        self.cur = np.zeros(3, np.float32)

    def reset(self):
        self.cur = np.zeros(3, np.float32)

    def step(self, vx, vy, wz):
        """One tick toward the target (vx, vy, wz); returns the slewed (vx, vy, wz), float32."""
        tgt = np.asarray((vx, vy, wz), np.float32)
        self.cur = np.asarray(slew_step(self.cur, tgt, self.delta), np.float32)
        return self.cur.copy()


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


def volume_clamp(vx, vy, axes, xp=np):
    """(vx, vy) held inside the volume's ellipse: a command off both axes whose normalized radius
    r = |(vx / a, vy / lat)| (a = the forward or backward semi-axis by the sign of vx) exceeds 1 is
    scaled by 1 / r, onto the ellipse along its own direction; inside it, or ON an axis (vx = 0 or
    vy = 0: the cardinal bands and the sway, which the per-axis clamp already holds), it passes
    unchanged. axes = volume_axes(...) (its turn axis unused: the turn is drawn independently).
    The arithmetic is the firmware's: in the inputs' precision, x / a squared, 1 / sqrt(r^2)."""
    fwd, back, lat = axes[0], axes[1], axes[2]
    nx = vx / xp.where(vx >= 0.0, fwd, back)
    ny = vy / lat
    r2 = nx * nx + ny * ny
    out = (vx != 0.0) & (vy != 0.0) & (r2 > 1.0)
    s = xp.where(out, 1.0 / xp.sqrt(xp.where(out, r2, 1.0)), 1.0)
    return xp.where(out, vx * s, vx), xp.where(out, vy * s, vy)


# The volume clamp's golden cases (firmware/host/test_protocol.cpp): the axes of a volume run on
# the default ranges (forward 0.24, backward 0.4, lateral 0.25 m/s) and the commands to clamp --
# the corner the per-axis clamp passed, each quadrant, inside, on the ellipse, on each axis.
VOLUME_GOLDEN_AXES = (0.24, 0.4, 0.25)
VOLUME_GOLDEN_IN = ((0.24, 0.25), (0.24, -0.25), (-0.4, 0.25), (-0.4, -0.25), (0.1, 0.1),
                    (-0.2, 0.1), (0.2, 0.2), (0.12, 0.2), (0.24, 0.0), (0.0, 0.25), (-0.4, 0.0),
                    (0.0, -0.25), (0.0, 0.0), (0.0013, 0.25), (0.24, -0.001), (-0.01, -0.249),
                    (0.169705627, 0.176776695), (-0.3, -0.2))


def volume_golden():
    """VOLUME_GOLDEN_IN through volume_clamp on VOLUME_GOLDEN_AXES, float32: [(vx, vy), ...]."""
    ax = tuple(np.float32(a) for a in VOLUME_GOLDEN_AXES)
    vin = np.asarray(VOLUME_GOLDEN_IN, np.float32)
    vx, vy = volume_clamp(vin[:, 0], vin[:, 1], ax)
    return np.stack([np.asarray(vx, np.float32), np.asarray(vy, np.float32)], axis=1)


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
