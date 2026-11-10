"""The firmware MOTION LIMITER (obs::MotionLimiter, firmware/components/obs/actuation.cpp) in
the sims, pinned to the COMPILED firmware code (2026-11-03):

  * sim/fw_limit.py (numpy; walker_env and simh run it) == the firmware, through the SIL
    library's pure sil_limit_trace, on step, ramp, reversal and random target streams, with and
    without the jerk boxcar; the boxcar length too;
  * env_mjx's JAX copy (fw_limit, after fw_shaper) == the firmware fed the env's own shaped
    targets, tolerance as test_fw_shaper.py (2e-5 rad); off is bit-identical;
  * env_mjx's goal speed under the limiter is obs::limitGoalSpeedSteps of the limited velocity:
    the slewed set point reaches every limited goal within the tick;
  * walker_env (the referee) runs the same chain, inherits it from config.json's keys, and
    train_mjx's --fw-limit records it.

The firmware-side tests skip (with the reason) when libctrl_sil is not built; the pre-push
hook builds it before pytest (`make sil-lib`).
"""
import ctypes
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import fw_limit as FL  # noqa: E402

import robot_plant  # noqa: E402

ROBOT = robot_plant.robot_env_kwargs("bimo_biped_v6legs.xml")
STAND = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
CFGS = [(200.0, 2000.0, 0.0), (200.0, 1000.0, 0.0), (150.0, 4000.0, 0.0),
        (200.0, 2000.0, 20000.0), (250.0, 3000.0, 50000.0), (60.0, 400.0, 3000.0)]


def _find_lib():
    env = os.environ.get("SIL_LIB")
    if env:
        return env if os.path.exists(env) else None
    for n in ("libctrl_sil.so", "libctrl_sil.dylib"):
        p = os.path.join(ROOT, "firmware", "host", "build", n)
        if os.path.exists(p):
            return p
    return None


@pytest.fixture(scope="module")
def fw():
    path = _find_lib()
    if path is None:
        pytest.skip("libctrl_sil not built (make sil-lib)")
    lib = ctypes.CDLL(path)
    if not hasattr(lib, "sil_limit_trace"):
        pytest.skip(f"{path} predates sil_abi 4 (no sil_limit_trace): rebuild it")
    lib.sil_spec.restype = ctypes.c_char_p
    nj = int(json.loads(lib.sil_spec().decode())["num_joints"])
    fp = ctypes.POINTER(ctypes.c_float)
    lib.sil_limit_trace.argtypes = [ctypes.c_float] * 3 + [ctypes.c_int, fp, fp, fp, fp]
    lib.sil_limit_trace.restype = ctypes.c_int

    def trace(cfg, q0, targets):
        """The compiled limiter over targets (T, n) from q0 (n,), any n: columns are
        independent joints, packed into the library's kNumJoints-wide rows."""
        targets = np.asarray(targets, np.float32)
        T, n = targets.shape
        q0 = np.broadcast_to(np.asarray(q0, np.float32), (n,))
        pos = np.zeros((T, n), np.float32); vel = np.zeros((T, n), np.float32)
        taps = None
        for c0 in range(0, n, nj):
            c1 = min(n, c0 + nj)
            x = np.zeros((T, nj), np.float32); x[:, :c1 - c0] = targets[:, c0:c1]
            q = np.zeros(nj, np.float32); q[:c1 - c0] = q0[c0:c1]
            x[:, c1 - c0:] = 0.0
            p = np.zeros((T, nj), np.float32); v = np.zeros((T, nj), np.float32)
            taps = lib.sil_limit_trace(*(ctypes.c_float(c) for c in cfg), T,
                                       q.ctypes.data_as(fp), np.ascontiguousarray(x).ctypes.data_as(fp),
                                       p.ctypes.data_as(fp), v.ctypes.data_as(fp))
            assert taps >= 1
            pos[:, c0:c1] = p[:, :c1 - c0]; vel[:, c0:c1] = v[:, :c1 - c0]
        return pos.astype(np.float64), vel.astype(np.float64), taps
    return trace


def _streams(n=12, T=160, seed=0):
    """Target streams that exercise every branch: steps both ways (cruise at vmax, braking,
    one-tick landing), a target running away then stopping, a reversal mid-move, and noise."""
    rng = np.random.default_rng(seed)
    t = np.arange(T)[:, None]
    q0 = rng.uniform(-0.5, 0.5, n)
    x = np.empty((T, n))
    x[:, 0] = q0[0] + 1.0
    x[:, 1] = q0[1] - 0.05
    x[:, 2] = q0[2] + np.minimum(t[:, 0], 12) * 0.12                     # runs ahead, stops
    x[:, 3] = np.where(t[:, 0] < 9, q0[3] + 1.2, q0[3] - 0.3)            # reversal
    x[:, 4] = q0[4] + 0.4 * np.sin(2 * np.pi * 1.5 * t[:, 0] * 0.02)     # gait-like
    for j in range(5, n):
        x[:, j] = q0[j] + np.cumsum(rng.normal(0, 0.08, T)).clip(-1.5, 1.5)   # policy noise
    return q0, x


@pytest.mark.parametrize("cfg", CFGS)
def test_numpy_law_matches_firmware(fw, cfg):
    q0, x = _streams(seed=int(cfg[1]))
    pos, vel, taps = fw(cfg, q0, x)
    lim = FL.MotionLimiter(cfg)
    lim.reset(q0)
    got = np.array([lim(xt) for xt in x])                  # (T, 2, n)
    assert lim.taps == taps
    np.testing.assert_allclose(got[:, 0], pos, rtol=0, atol=1e-6)
    np.testing.assert_allclose(got[:, 1], vel, rtol=0, atol=1e-5)
    # and the law's bounds, judged on the compiled output itself
    vm, am, _ = (c * math.pi / 180 for c in cfg)
    assert np.abs(vel).max() <= vm * (1 + 1e-5)
    acc = np.diff(np.vstack([np.zeros(x.shape[1]), vel]), axis=0) / 0.02
    assert np.abs(acc).max() <= am * (1 + 1e-4) + 1e-3
    # column 0/1: steps toward a still target -- never past it, there at rest at the end
    for j in (0, 1):
        s = np.sign(x[0, j] - q0[j])
        assert (s * (pos[:, j] - x[0, j])).max() <= 1e-6
        assert abs(pos[-1, j] - x[0, j]) < 1e-6 and abs(vel[-1, j]) < 1e-5
    # column 2: the target outruns vmax and stops; the joint arrives at rest, never past
    assert (pos[:, 2] - x[:, 2]).max() <= 1e-6 and abs(vel[-1, 2]) < 1e-5


def test_taps_match_firmware(fw):
    q0 = np.zeros(1)
    x = np.zeros((2, 1))
    for a in (500.0, 1000.0, 2000.0, 3000.0, 4000.0):
        for j in (0.0, 1000.0, 5000.0, 10000.0, 20000.0, 25000.0, 40000.0, 1e5, 1e6):
            want = fw((200.0, a, j), q0, x)[2]
            r = FL.to_rad((200.0, a, j))
            assert FL.taps(r[1], r[2]) == want, (a, j)


def test_parse():
    assert FL.parse(None) is None and FL.parse("") is None and FL.parse("off") is None
    assert FL.parse("200,2000") == (200.0, 2000.0, 0.0)
    assert FL.parse([150, 3000, 20000]) == (150.0, 3000.0, 20000.0)
    assert FL.parse("0,2000") is None
    with pytest.raises(ValueError):
        FL.parse("200")


def _mjx(**kw):
    from env_mjx import BimoMJXEnv
    base = dict(ROBOT, ext_cmd=True, obs_hist_len=3, domain_rand=False,
                imu_noise=0.0, quantize_ticks=False, latency_ms=0.0,
                backlash_deg=0.0, cmd_fixed=STAND)
    base.update(kw)
    return BimoMJXEnv(**base)


def _targets(env, a):
    a = np.asarray(a, np.float64)
    d, hi, lo = (np.asarray(x, np.float64) for x in (env._default, env._hi, env._lo))
    if env.action_map == "full":
        return d + np.where(a >= 0.0, hi - d, d - lo) * np.clip(a, -1, 1)
    return np.clip(d + np.asarray(env._scale, np.float64) * np.clip(a, -1, 1), lo, hi)


def _shaped(targets, y0, hz, dt):
    k = 1.0 - math.exp(-2.0 * math.pi * hz * dt)
    y = np.stack([np.asarray(y0, np.float64)] * 3)
    out = []
    for t in targets:
        y[0] += k * (t - y[0]); y[1] += k * (y[0] - y[1]); y[2] += k * (y[1] - y[2])
        out.append(y[2].copy())
    return np.array(out)


ACTS = [0.9, 0.9, -0.8, -0.8, 0.5, 0.0, 0.0, 0.3, -0.6, 0.9, 0.9, 0.9]


@pytest.mark.parametrize("cfg", [(200.0, 2000.0, 0.0), (150.0, 1500.0, 30000.0)])
def test_mjx_matches_firmware(fw, cfg):
    import jax
    import jax.numpy as jp
    env = _mjx(fw_shaper_hz=10.0, fw_limit=list(cfg))
    s = env.reset(jax.random.PRNGKey(1))
    y0 = np.asarray(s.shp_y[0], np.float64)
    assert np.array_equal(np.asarray(s.lim_hp[0]), np.asarray(s.shp_y[2]))   # seeded alike
    step = jax.jit(env.step)
    rng = np.random.default_rng(3)
    acts = [a + rng.normal(0, 0.1, env.action_size) for a in ACTS]
    lim_out = []
    for a in acts:
        s = step(s, jp.asarray(a, jp.float32))
        lim_out.append(np.asarray(s.last_target, np.float64))
    raw = [_targets(env, np.clip(a, -1, 1)) for a in acts]
    shaped = _shaped(raw, y0, 10.0, env.control_dt)
    pos, vel, taps = fw(cfg, y0, shaped)
    assert env._lim_taps == taps
    # no dead time, no act-lag, no goal speed: the PD is served the limited goal
    np.testing.assert_allclose(np.array(lim_out), pos, rtol=0, atol=2e-5)
    # the velocity the goal speed is written from (the boxcar's, with a jerk limit)
    np.testing.assert_allclose(FL.boxcar(np.asarray(s.lim_hv, np.float32)), vel[-1],
                               rtol=0, atol=1e-4)
    # it limited something: the shaped targets move faster than the limit allows
    assert np.abs(np.diff(shaped, axis=0)).max() / env.control_dt > cfg[0] * math.pi / 180
    assert np.abs(np.array(lim_out) - shaped).max() > 0.05


def test_mjx_limit_off_is_bit_identical():
    import jax
    import jax.numpy as jp
    a = 0.4 * jp.ones(_mjx().action_size)
    e0 = _mjx(fw_shaper_hz=10.0)
    s0 = e0.step(e0.reset(jax.random.PRNGKey(2)), a)
    e1 = _mjx(fw_shaper_hz=10.0, fw_limit=None)
    s1 = e1.step(e1.reset(jax.random.PRNGKey(2)), a)
    e2 = _mjx(fw_shaper_hz=10.0, fw_limit="off")
    s2 = e2.step(e2.reset(jax.random.PRNGKey(2)), a)
    for s in (s1, s2):
        np.testing.assert_array_equal(np.asarray(s0.last_target), np.asarray(s.last_target))
        np.testing.assert_array_equal(np.asarray(s0.data.qpos), np.asarray(s.data.qpos))


def test_mjx_goal_speed_follows_the_limiter():
    """With fw_goal_speed the goal speed is the limited velocity x 1.25: the servo's slewed
    set point reaches every limited goal within the tick (it is never the bottleneck), and it
    is reached at the limiter's pace, not at the gap law's slam."""
    import jax
    import jax.numpy as jp
    cfg = (200.0, 2000.0, 0.0)
    env = _mjx(fw_shaper_hz=10.0, fw_limit=list(cfg), fw_goal_speed=True)
    ref = _mjx(fw_shaper_hz=10.0, fw_limit=list(cfg))
    s, r = env.reset(jax.random.PRNGKey(4)), ref.reset(jax.random.PRNGKey(4))
    step, rstep = jax.jit(env.step), jax.jit(ref.step)
    for a in ACTS:
        av = jp.full(env.action_size, a, jp.float32)
        s, r = step(s, av), rstep(r, av)
        # the served set point ends each tick ON the limited goal (rounded speed, 25 % headroom)
        np.testing.assert_allclose(np.asarray(s.last_target), np.asarray(s.lim_hp[0]),
                                   rtol=0, atol=1e-5)
        assert np.all(np.isfinite(np.asarray(s.data.qpos)))
    # and the limited goals themselves are the plain limited chain's (goal speed is downstream)
    np.testing.assert_allclose(np.asarray(s.lim_hp), np.asarray(r.lim_hp), rtol=0, atol=1e-6)
    v = np.asarray(s.lim_hv[0], np.float64)
    sps = FL.goal_speed_steps(v)
    assert np.all((sps >= FL.SPEED_FLOOR) & (sps <= FL.SPEED_MAX))


def test_walker_env_matches_firmware(fw):
    from walker_env import BimoWalkerEnv
    cfg = (200.0, 2000.0, 0.0)
    w = BimoWalkerEnv(**ROBOT, command_mode=True, ext_cmd=True,
                      actuator_model="sts3215", imu_obs=True, domain_rand=False,
                      latency_ms=0.0, backlash_deg=0.0, quantize_ticks=False,
                      fw_shaper_hz=10.0, fw_limit=list(cfg))
    w.reset(seed=0)
    n = w.action_space.shape[0]
    raw, got = [], []
    for a in ACTS:
        av = np.full(n, a, np.float32)
        raw.append(w._to_servo(w._action_to_ctrl(av)))
        w.step(av)
        got.append(np.array(w.data.ctrl))
    shaped = _shaped(raw, w._sdefault, 10.0, w.control_dt)
    pos, _, _ = fw(cfg, w._sdefault, shaped)
    np.testing.assert_allclose(np.array(got), pos, rtol=0, atol=1e-6)
    # off by default, and reset re-seeds it
    assert BimoWalkerEnv(**ROBOT, command_mode=True, ext_cmd=True,
                         actuator_model="sts3215").fw_limit is None
    w.reset(seed=1)
    assert w._lim is None


def test_train_flag_records_the_limiter():
    import train_mjx as T
    kw = T.build_env_kw(T.make_parser().parse_args(
        ["--robot", "--fw-shaper", "10", "--fw-goal-speed", "--fw-limit", "200,2000"]))
    assert kw["fw_limit"] == [200.0, 2000.0, 0.0]
    assert json.loads(json.dumps(kw["fw_limit"])) == [200.0, 2000.0, 0.0]   # config.json-safe
    kw = T.build_env_kw(T.make_parser().parse_args(["--robot"]))
    assert "fw_limit" not in kw
    # the referee inherits it by name (eval_precision keeps the config keys walker_env takes)
    import inspect
    from walker_env import BimoWalkerEnv
    assert "fw_limit" in inspect.signature(BimoWalkerEnv.__init__).parameters
