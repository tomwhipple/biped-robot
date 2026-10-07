"""--mimic-lift: the imitation reference's swing sole rises straight up by
mimic_lift * sw**2 at any walking speed (IK from the home pose), level, with the
stance leg unchanged; the CPU and MJX references agree."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

LIFT = 0.05


def _cpu_env():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    kw.update(mimic_lift=LIFT)
    return BimoWalkerEnv(**kw)


def _sole_pose(env, q):
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    pol = [names[i] for i in env._pi]
    for i, n in enumerate(pol):
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = q[i]
    mujoco.mj_kinematics(m, d)
    t = m.body("torso").id
    R = d.xmat[t].reshape(3, 3)
    out = []
    for g in env._sole_gids:
        p = R.T @ (d.geom_xpos[g] - d.xpos[t])
        Rs = R.T @ d.geom_xmat[g].reshape(3, 3)
        out.append((p, math.degrees(math.atan2(-Rs[2, 0], Rs[2, 2]))))   # sole pitch
    return out


@pytest.mark.parametrize("v", [0.1, 0.2, 0.35])
def test_swing_sole_rises_straight_up_and_level(v):
    env = _cpu_env()
    home = _sole_pose(env, env._default)
    cmd = np.zeros(7)
    cmd[0] = v
    # phase pi/2: the LEFT leg at mid-swing (sw = sin), the right in stance
    q = env._mimic_ref(cmd, math.pi / 2, 1.5)
    (pl, al), (pr, ar) = _sole_pose(env, q)
    (hl, hal), (hr, har) = home
    assert (pl - hl)[2] == pytest.approx(LIFT, abs=1.5e-3)        # up by the lift
    assert abs((pl - hl)[0] + 0.0) < 0.06                          # stride (A*xn) only
    assert abs(al - hal) < 1.0                                     # sole level
    assert np.allclose(pr[2], hr[2], atol=2e-3)                    # stance foot at home height
    assert abs(ar - har) < 1.0



@pytest.mark.parametrize("v", [0.1, 0.35])
def test_swing_height_profile_is_lift_sw_squared(v):
    # h = mimic_lift * sw**2: zero vertical speed at liftoff and touchdown;
    # the stride's hip swing only ever adds height (1 - cos), never removes it
    env = _cpu_env()
    (hl, _), _ = _sole_pose(env, env._default)
    cmd = np.zeros(7)
    cmd[0] = v
    for ph in np.linspace(0.1, math.pi - 0.1, 9):
        want = LIFT * math.sin(ph) ** 2
        (pl, al), _ = _sole_pose(env, env._mimic_ref(cmd, ph, 1.5))
        assert (pl - hl)[2] >= want - 1e-3
        assert (pl - hl)[2] <= want + 0.012
        if abs(ph - math.pi / 2) < 0.6:
            assert (pl - hl)[2] == pytest.approx(want, abs=3e-3)

def test_zero_command_is_the_home_pose():
    env = _cpu_env()
    q = env._mimic_ref(np.zeros(7), math.pi / 2, 1.5)
    assert np.allclose(q, env._default, atol=1e-9)


def test_cpu_and_mjx_references_agree():
    import jax.numpy as jp
    import robot_plant
    import env_mjx as E
    kw = robot_plant.robot_env_kwargs()
    kw.update(mimic_lift=LIFT)
    c = _cpu_env()
    g = E.BimoMJXEnv(**kw)
    worst = 0.0
    for v, w in ((0.05, 0.0), (0.2, 0.3), (0.35, -0.4), (0.0, 0.0)):
        cmd = np.zeros(7)
        cmd[0], cmd[2] = v, w
        for ph in np.linspace(0.0, 2 * math.pi, 13):
            a = c._mimic_ref(cmd, ph, 1.5)
            b = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(1.5)))
            worst = max(worst, float(np.abs(a - b).max()))
    assert worst < 1e-4, worst
