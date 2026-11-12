"""--w-march-unload: the ungated march lift term (#137, option 1, 2026-11-12).

Every lift reward of the march (w_lift, w_knee_high, the clearance-gated w_com_stance / foot
kernels) pays only once the commanded foot is already off the floor, so a foot that never
lifted earns nothing and each seed settled on a one-legged march. During a gait-clock march
(traj mode 4) this term pays the COMMANDED swing foot whether or not it is up:

    r = w * 0.5 * (u + h)
    u = clip(((c - p_sw) . e) / |e|^2, 0, 1),  e = p_st - p_sw      (horizontal)
    h = clip(foot_clear / (lift_height + c6), 0, 1)

u is the swing foot's static unloading by the lever rule (the CoM's progress from the swing
sole to the stance sole), h its sole height as a fraction of the commanded swing height.
Off by default; with the flag off nothing changes.
"""
import inspect
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--precision", "--mjx-impl", "jax",
        "--family", "loco", "--march-mix", "0.15", "--w-knee-high", "1.0"]


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["w_march_unload"].default == 0.0
    assert "w_march_unload" not in _kw()
    assert _kw("--w-march-unload", "1.0")["w_march_unload"] == 1.0
    # nothing else in the env kwargs moves with the flag
    a, b = _kw(), _kw("--w-march-unload", "1.0")
    b.pop("w_march_unload")
    assert a == b


@pytest.mark.parametrize("xp_name", ["numpy", "jax"])
def test_the_formula(xp_name):
    if xp_name == "numpy":
        xp = np
    else:
        import jax.numpy as xp
    T = W.march_unload_term
    sw, st = np.array([0.0, 0.06, 0.0]), np.array([0.0, -0.06, 0.0])
    f = lambda com, clr=0.0, tgt=0.04, on=1.0: float(T(xp.asarray(com), xp.asarray(st), xp.asarray(sw),
                                                      clr, tgt, on, xp))
    assert f([0.0, 0.06, 0.2]) == pytest.approx(0.0)            # all the weight on the swing foot
    assert f([0.0, 0.0, 0.2]) == pytest.approx(0.25)            # half: u = 0.5, h = 0
    assert f([0.0, -0.06, 0.2]) == pytest.approx(0.5)           # unloaded, still on the floor
    assert f([0.0, -0.06, 0.2], clr=0.02) == pytest.approx(0.75)  # half the commanded height
    assert f([0.0, -0.06, 0.2], clr=0.09) == pytest.approx(1.0)   # clipped at the target
    assert f([0.0, -0.20, 0.2], clr=-0.01) == pytest.approx(0.5)  # clipped below and beyond
    assert f([0.03, 0.0, 0.2]) == pytest.approx(0.25)           # fore-aft CoM does not count
    assert f([0.0, -0.06, 0.2], clr=0.02, on=0.0) == 0.0
    # gradient from the first millimetre: 1 mm of shift and 1 mm of lift both pay
    assert f([0.0, 0.059, 0.2]) > 0.0 and f([0.0, 0.06, 0.2], clr=0.001) > 0.0
    # L/R symmetric by construction: the mirrored state pays the same
    g = float(T(xp.asarray([0.0, 0.03, 0.2]), xp.asarray(sw), xp.asarray(st), 0.01, 0.04, 1.0, xp))
    assert g == pytest.approx(f([0.0, -0.03, 0.2], clr=0.01))


def _march_env(w):
    sys.path.insert(0, os.path.join(SIM, "mjx"))
    import robot_plant
    kw = dict(robot_plant.robot_env_kwargs(None), ext_cmd=True, gait_clock=True, w_knee_high=1.0,
              ext_mix=(0.0,) * 7, march_mix=1.0, w_march_unload=w)
    return W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False, **kw)


def test_cpu_term_is_paid_on_the_floor_and_off_without_the_flag():
    on, off = _march_env(1.0), _march_env(0.0)
    on.reset(seed=7)
    off.reset(seed=7)
    z = np.zeros(on._nq_act, np.float32)
    paid = []
    for _ in range(40):
        lift_cmd = abs(float(on._cmd[4])) > 0.5
        o1, r1, *_, info = on.step(z)
        o0, r0, *_, info0 = off.step(z)
        np.testing.assert_array_equal(o1, o0)          # the term never touches the physics
        assert "march_unload" not in info0
        assert r1 - r0 == pytest.approx(info["march_unload"], abs=1e-9)
        if lift_cmd:
            paid.append(info["march_unload"])
            assert not info["lift_ok"]                 # the feet are on the floor ...
    assert len(paid) > 30 and min(paid) > 0.2          # ... and it is paid anyway


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R17 (airborne arithmetic, the term live, the grounded stand), in a
    subprocess (parity_test turns jax_enable_x64 on at import)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--march-unload"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
