"""--w-march-both: the two-foot (AND) march term (#137, planner 2026-11-13).

23 of 23 march seeds (11-10 .. 11-12: the reference march, the play plant, the mirror loss, the
ungated --w-march-unload) marched on ONE foot. Every march reward pays each foot's swing on its
own and the two are added, so a one-legged march collects about half of everything and the warm
start picks the side. This term pays the weaker foot:

    per foot f: last_f = the best 0.5 (u + h) (march_unload_term's score) over f's last
                completed commanded swing
    r = w * on * min(last_L, last_R)

A one-legged march earns only the planted foot's unloading, and the gradient of the min flows to
that foot. Off by default; with the flag off nothing changes.
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
        assert inspect.signature(cls.__init__).parameters["w_march_both"].default == 0.0
    assert "w_march_both" not in _kw()
    assert _kw("--w-march-both", "2.0")["w_march_both"] == 2.0
    a, b = _kw(), _kw("--w-march-both", "2.0")
    b.pop("w_march_both")
    assert a == b


def _run(xp, seq):
    """seq: (score, swing_r, on) per tick -> list of (cur, last, r)."""
    cur, last = xp.zeros(2), xp.zeros(2)
    out = []
    for s, sr, on in seq:
        cur, last, r = W.march_both_update(cur, last, s, sr, on, xp)
        out.append((np.array(cur), np.array(last), float(r)))
    return out


@pytest.mark.parametrize("xp_name", ["numpy", "jax"])
def test_latch_and_min(xp_name):
    if xp_name == "numpy":
        xp = np
    else:
        import jax.numpy as xp
    # L swing (scores rise to 0.9), then R swing (to 0.7), then L again
    seq = ([(x, 0.0, 1.0) for x in (0.2, 0.9, 0.5)] + [(x, 1.0, 1.0) for x in (0.3, 0.7, 0.1)]
           + [(0.4, 0.0, 1.0)])
    o = _run(xp, seq)
    assert o[2][0] == pytest.approx([0.9, 0.0]) and o[2][2] == 0.0      # running max, nothing latched
    assert o[3][1] == pytest.approx([0.9, 0.0]) and o[3][2] == 0.0      # L latched on the switch
    assert o[3][0] == pytest.approx([0.0, 0.3])
    assert o[6][1] == pytest.approx([0.9, 0.7])                         # R latched on the switch back
    assert o[6][2] == pytest.approx(0.7)                                # min of the two
    # a one-legged march: L lifts fully every swing, R never better than 0.25
    one = []
    for _ in range(4):
        one += [(1.0, 0.0, 1.0)] * 3 + [(0.25, 1.0, 1.0)] * 3
    o = _run(xp, one)
    assert max(r for *_, r in o) == pytest.approx(0.25)
    # off (no march): everything resets and nothing is paid
    o = _run(xp, seq + [(0.9, 1.0, 0.0)])
    assert o[-1][2] == 0.0 and not o[-1][0].any() and not o[-1][1].any()


def test_cpu_term_is_live_and_off_without_the_flag():
    import robot_plant
    mk = lambda w: W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False,
                                   **dict(robot_plant.robot_env_kwargs(None), ext_cmd=True,
                                          gait_clock=True, w_knee_high=1.0, ext_mix=(0.0,) * 7,
                                          march_mix=1.0, w_march_both=w))
    on, off = mk(2.0), mk(0.0)
    on.reset(seed=7)
    off.reset(seed=7)
    z = np.zeros(on._nq_act, np.float32)
    paid = 0.0
    for _ in range(100):
        o1, r1, t1, tr1, info = on.step(z)
        o0, r0, t0, tr0, info0 = off.step(z)
        np.testing.assert_array_equal(o1, o0)          # the term never touches the physics
        assert "march_both" not in info0
        assert r1 - r0 == pytest.approx(2.0 * info["march_both"], abs=1e-9)
        paid += info["march_both"]
        if t1 or tr1:
            break
    assert paid > 0.0                                  # both feet's swings latched on the floor


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R20, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--march-both"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
