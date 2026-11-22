"""The operator command's slew and volume (#266, #263; sim/cmd_kernel.py).

--cmd-slew rate-limits the planar command (vx, vy, wz) ahead of the observation, the law the
firmware runs (firmware/components/cmdslew, pinned by tests/test_cmdslew_golden.py);
--cmd-volume draws walks from the whole envelope instead of the six cardinal bands. Both are off
by default, and off they change nothing: no kwarg in the run's env, no RNG drawn, no state. The
CPU-vs-MJX parity of both is sim/mjx/parity_test.py R23.
"""
import inspect
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import cmd_kernel as ck  # noqa: E402
import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--precision", "--mjx-impl", "jax",
        "--family", "loco"]


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        ps = inspect.signature(cls.__init__).parameters
        assert ps["cmd_slew"].default is None
        assert ps["cmd_volume"].default == 0.0
    base = _kw()
    assert "cmd_slew" not in base and "cmd_volume" not in base
    on = _kw("--cmd-slew", "0.5,0.4,1.5", "--cmd-volume", "0.7")
    assert on["cmd_slew"] == (0.5, 0.4, 1.5) and on["cmd_volume"] == 0.7
    # a volume run trains with the volume's stand band (tests/test_cmd_stand_band.py)
    assert on["cmd_stand_band"] == ck.STAND_BAND_VOLUME
    on.pop("cmd_slew")
    on.pop("cmd_volume")
    on.pop("cmd_stand_band")
    assert on == base


def test_parse_slew():
    assert ck.parse_slew(None) is None and ck.parse_slew("") is None
    assert ck.parse_slew("0.5,0.4,1.5") == (0.5, 0.4, 1.5)
    assert ck.parse_slew([1, 2, 3]) == (1.0, 2.0, 3.0)
    for bad in ("1,2", "0,1,1", "-1,1,1", "1,1,inf"):
        with pytest.raises(ValueError):
            ck.parse_slew(bad)


@pytest.mark.parametrize("xp_name", ["numpy", "jax"])
def test_slew_law(xp_name):
    if xp_name == "numpy":
        xp = np
    else:
        import jax.numpy as xp
    d = ck.slew_delta((0.5, 0.4, 1.5), 0.02)
    assert np.allclose(d, [0.01, 0.008, 0.03])
    cur = xp.asarray(np.zeros(3, np.float32))
    tgt = xp.asarray(np.array([0.1, -0.1, 0.015], np.float32))
    out = []
    for _ in range(20):
        cur = ck.slew_step(cur, tgt, xp.asarray(d), xp)
        out.append(np.asarray(cur))
    out = np.asarray(out)
    assert np.allclose(out[0], [0.01, -0.008, 0.015])          # wz lands inside one step
    assert np.all(np.abs(np.diff(out, axis=0)) <= d + 1e-7)    # never faster than the rate
    assert np.array_equal(out[-1], np.asarray(tgt))            # lands exactly, no creep
    assert np.argmax(out[:, 0] == np.float32(0.1)) in (9, 10)   # 0.1 m/s at 0.5 m/s^2: ~0.2 s


def test_golden_ramp():
    g = ck.golden()
    assert g.shape == (120, 3) and g.dtype == np.float32
    d = ck.slew_delta(ck.GOLDEN_RATES, ck.GOLDEN_DT)
    assert np.all(np.abs(np.diff(g, axis=0)) <= d + 1e-7)
    assert np.allclose(g[0], d * np.sign(ck.GOLDEN_STEPS[0][1]))
    assert np.array_equal(g[-1], np.float32([0.013, 0.0, 0.0]))


def test_command_slew_twin_is_the_golden_ramp():
    """cmd_kernel.CommandSlew (the firmware's cmdslew in Python, for grade.py and sil_twin.py) runs
    GOLDEN_STEPS to cmd_kernel.golden() bit for bit -- the table test_cmdslew.cpp pins the C++ to --
    and reset() returns it to the stand."""
    s = ck.CommandSlew(ck.GOLDEN_RATES, ck.GOLDEN_DT)
    steps = dict(ck.GOLDEN_STEPS)
    tgt = (0.0, 0.0, 0.0)
    out = []
    for k in range(120):
        tgt = steps.get(k, tgt)
        out.append(s.step(*tgt))
    out = np.asarray(out)
    assert out.dtype == np.float32 and out.tobytes() == ck.golden().tobytes()
    s.reset()
    assert np.array_equal(s.step(0.24, 0.0, 0.0), np.float32([0.01, 0.0, 0.0]))


@pytest.mark.parametrize("xp_name", ["numpy", "jax"])
def test_volume_draw(xp_name):
    if xp_name == "numpy":
        xp = np
    else:
        import jax.numpy as xp
    axes = (0.24, 0.4, 0.25, 1.0)
    u = np.random.default_rng(0).uniform(size=(4, 20000)).astype(np.float32)
    vx, vy, wz = (np.asarray(a) for a in ck.volume_cmd(*(xp.asarray(x) for x in u), axes, xp))
    ax = np.where(vx >= 0, axes[0], axes[1])
    rho = np.hypot(vx / ax, vy / axes[2])
    assert rho.max() <= 1.0 + 1e-6 and rho.max() > 0.99          # inside the envelope, to its edge
    assert np.all(np.abs(wz) <= axes[3] + 1e-6)
    # no dead bands: the draw is continuous down to zero in every channel
    for ch, lim in ((vx[vx > 0], 0.05), (-vx[vx < 0], 0.15), (np.abs(vy), 0.1)):
        assert np.mean(ch < lim) > 0.05
    # combinations: diagonals, back + turn, side + turn
    assert np.sum((np.abs(vx) > 0.05) & (np.abs(vy) > 0.05)) > 3000
    assert np.sum((vx < -0.1) & (np.abs(wz) > 0.3)) > 500
    assert np.sum((np.abs(vx) < 0.03) & (np.abs(vy) > 0.1) & (np.abs(wz) > 0.3)) > 100
    assert abs(np.mean(wz != 0.0) - ck.VOLUME_P_TURN) < 0.02
    # every compass heading equally often
    h = (np.degrees(np.arctan2(vy / axes[2], vx / ax)) + 22.5) % 360.0
    counts = np.histogram(h, bins=8, range=(0.0, 360.0))[0]
    assert counts.min() > 0.8 * counts.mean()


def _env(**kw):
    import robot_plant
    k = dict(command_mode=True, ext_cmd=True, domain_rand=False, actuator_model="sts3215")
    k.update(robot_plant.robot_env_kwargs(None))
    k.update(kw)
    return W.BimoWalkerEnv(**k)


def test_cpu_env_slew_and_volume():
    e = _env(cmd_slew=(0.5, 0.4, 1.5), cmd_volume=1.0, ext_mix=(0.0,) * 7,
             cmd_resample_s=(0.3, 0.3))
    e.reset(seed=3)
    assert np.array_equal(e._cmd[:3], np.zeros(3))         # starts from the stand
    tgt0 = e._cmd_tgt.copy()
    assert np.any(tgt0 != 0.0)
    z = np.zeros(e.action_space.shape, np.float32)
    seen = [e._cmd[:3].copy()]
    tgts = {tuple(tgt0)}
    for _ in range(40):
        e.step(z)
        seen.append(e._cmd[:3].copy())
        tgts.add(tuple(e._cmd_tgt))
    d = ck.slew_delta((0.5, 0.4, 1.5), e.control_dt)
    assert np.all(np.abs(np.diff(np.asarray(seen), axis=0)) <= d + 1e-9)
    assert len(tgts) >= 3                                   # mid-episode changes, through the slew
    # set_command is a target too: the stick step ramps
    e.set_command(-0.3, 0.0, 0.0)
    before = e._cmd[:3].copy()
    e.step(z)
    assert abs(e._cmd[0] - before[0]) <= d[0] + 1e-9 and e._cmd_tgt[0] == -0.3


def test_cpu_env_off_is_unchanged():
    a = _env()
    b = _env(cmd_volume=0.0, cmd_slew=None)
    oa, _ = a.reset(seed=11)
    ob, _ = b.reset(seed=11)
    assert np.array_equal(oa, ob) and a._cmd_tgt is None
    z = np.zeros(a.action_space.shape, np.float32)
    for _ in range(5):
        assert np.array_equal(a.step(z)[0], b.step(z)[0])
    with pytest.raises(ValueError):
        W.BimoWalkerEnv(command_mode=True, ext_cmd=False, cmd_slew=(1, 1, 1))
    with pytest.raises(ValueError):
        W.BimoWalkerEnv(command_mode=True, ext_cmd=False, cmd_volume=0.5)


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R23, in a subprocess (parity_test turns jax_enable_x64 on)."""
    import subprocess
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--cmd-slew"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


# -- the volume clamp (#263): the robot holds a command to the volume's ellipse ----------------
def test_volume_clamp_holds_the_ellipse():
    """volume_clamp scales a corner past the ellipse back onto it along its own direction; every
    point inside it, and every point on an axis, passes unchanged."""
    ax = ck.volume_axes((0.05, 0.24), (-0.4, -0.15), 0.25, 1.0)
    vx, vy = ck.volume_clamp(0.24, 0.25, ax)                 # radius sqrt(2) under per-axis limits
    assert math.hypot(vx / 0.24, vy / 0.25) == pytest.approx(1.0)
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(0.25, 0.24))
    vx, vy = ck.volume_clamp(-0.4, -0.25, ax)                # the backward semi-axis on vx < 0
    assert math.hypot(vx / 0.4, vy / 0.25) == pytest.approx(1.0) and vx < 0 and vy < 0
    for p in ((0.1, 0.1), (-0.2, 0.1), (0.24, 0.0), (0.0, 0.25), (-0.4, 0.0), (0.0, 0.0),
              (0.0, 0.3)):                                   # inside, or on an axis
        assert tuple(map(float, ck.volume_clamp(*p, ax))) == p
    # every volume draw is a fixed point: the clamp takes nothing the sampler drew
    rng = np.random.default_rng(3)
    u = rng.uniform(size=(4, 20000))
    dvx, dvy, _ = ck.volume_cmd(u[0], u[1], u[2], u[3], ax)
    cvx, cvy = ck.volume_clamp(dvx, dvy, ax)
    np.testing.assert_allclose(cvx, dvx, rtol=0, atol=1e-12)
    np.testing.assert_allclose(cvy, dvy, rtol=0, atol=1e-12)


def test_volume_clamp_twins_agree():
    """link/protocol.py clamp_to_volume (the robot link's Python twin) is the kernel's law."""
    sys.path.insert(0, os.path.join(HERE, "..", "link"))
    import protocol as P
    ax = ck.volume_axes((0.05, 0.24), (-0.4, -0.15), 0.25, 1.0)
    env = P.Envelope(0.0, 0.24, 0.0, 0.4, 0.0, 0.25, 1.0, 1.0, volume=ax[:3])
    for vx in np.linspace(-0.5, 0.3, 33):
        for vy in np.linspace(-0.3, 0.3, 25):
            want = tuple(map(float, ck.volume_clamp(float(vx), float(vy), ax)))
            assert P.clamp_to_volume(float(vx), float(vy), env) == pytest.approx(want, abs=1e-15)
    off = P.Envelope(0.0, 0.24, 0.0, 0.4, 0.0, 0.25, 1.0, 1.0)
    assert P.clamp_to_volume(0.24, 0.25, off) == (0.24, 0.25)


def test_volume_golden_table_matches_python():
    """firmware/host/test_protocol.cpp's literal golden table is volume_golden() on its axes."""
    import re
    src = open(os.path.join(HERE, "..", "firmware", "host", "test_protocol.cpp")).read()
    num = r"[-+]?(?:\d+\.?\d*(?:e[-+]?\d+)?)f"
    m = re.search(r"kVolGoldenAxes\[3\] = \{(.*?)\};", src)
    assert m
    got_ax = np.float32([float(x[:-1]) for x in re.findall(num, m.group(1))])
    assert got_ax.tolist() == np.float32(ck.VOLUME_GOLDEN_AXES).tolist()
    m = re.search(r"// BEGIN VOLUME GOLDEN(.*?)// END VOLUME GOLDEN", src, re.S)
    body = m.group(1).split("= {", 1)[1]
    vals = np.float32([float(x[:-1]) for x in re.findall(num, body)]).reshape(-1, 4)
    np.testing.assert_array_equal(vals[:, :2], np.float32(ck.VOLUME_GOLDEN_IN))
    np.testing.assert_array_equal(vals[:, 2:], ck.volume_golden())
