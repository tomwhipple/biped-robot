"""--mimic-sway: while walking, the imitation reference moves the torso toward
the stance foot by mimic_sway (m), hip roll + cancelling ankle roll so the soles
stay level; zero command stays the home pose; the CPU and MJX references agree;
the flags reach the env kwargs. (2026-11-09 hardware: prec16 never unloaded its
R foot -- the reference held the rolls at the standing pose while walking, so
w_mimic paid against any weight transfer.)"""
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

import walker_env as W  # noqa: E402

XML = "bimo_biped_v6legs_meas_sole.xml"
ROBOT = ["--robot", "--xml", XML, "--precision", "--mjx-impl", "jax", "--mimic-lift", "0.02"]
SWAY = 0.04


def _kw(*extra):
    import train_mjx
    p = train_mjx.make_parser()
    return dict(train_mjx.build_env_kw(p.parse_args(ROBOT + list(extra))), domain_rand=False)


def _cpu_env(sway=SWAY, lead=0.0):
    import robot_plant
    kw = robot_plant.robot_env_kwargs(XML)
    kw.update(mimic_lift=0.02, mimic_sway=sway, mimic_sway_lead=lead)
    return W.BimoWalkerEnv(**kw)


def _soles(env, q):
    """(y in the torso frame, sole roll deg) per sole for policy-joint pose q."""
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    for i, n in enumerate(names[k] for k in env._pi):
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = q[i]
    mujoco.mj_kinematics(m, d)
    t = m.body("torso").id
    R = d.xmat[t].reshape(3, 3)
    out = []
    for g in env._sole_gids:
        p = R.T @ (d.geom_xpos[g] - d.xpos[t])
        Rs = R.T @ d.geom_xmat[g].reshape(3, 3)
        out.append((p[1], math.degrees(math.atan2(Rs[2, 1], Rs[2, 2]))))
    return out


def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        sig = inspect.signature(cls.__init__).parameters
        assert sig["mimic_sway"].default == 0.0 and sig["mimic_sway_lead"].default == 0.785
    kw = _kw()
    assert "mimic_sway" not in kw and "mimic_sway_lead" not in kw
    k2 = _kw("--mimic-sway", "0.03", "--mimic-sway-lead", "0.5")
    assert k2["mimic_sway"] == 0.03 and k2["mimic_sway_lead"] == 0.5


@pytest.mark.parametrize("phase,toward", [(math.pi / 2, "R"), (-math.pi / 2, "L")])
def test_torso_moves_over_the_stance_foot_soles_level(phase, toward):
    env = _cpu_env(lead=0.0)
    home = _soles(env, env._default)
    cmd = np.zeros(7)
    cmd[0] = 0.15
    q = env._mimic_ref(cmd, phase, 0.75)
    q0 = _cpu_env(sway=0.0)._mimic_ref(cmd, phase, 0.75)   # the same reference, no sway
    (yl, rl), (yr, rr) = _soles(env, q)
    (yl0, _), (yr0, _) = _soles(env, q0)
    # torso toward R (-y) = both soles +y in the torso frame, by the sway
    want = SWAY if toward == "R" else -SWAY
    ys, ysw = (yr - yr0, yl - yl0) if toward == "R" else (yl - yl0, yr - yr0)
    assert ys == pytest.approx(want, abs=1.5e-3)       # the stance sole: the full sway
    assert ysw == pytest.approx(want, abs=5e-3)        # the swing sole (knee bent): about it
    assert abs(rl - home[0][1]) < 1.0 and abs(rr - home[1][1]) < 1.0   # soles level


def test_lead_shifts_the_phase():
    env = _cpu_env(lead=0.785)
    cmd = np.zeros(7)
    cmd[0] = 0.15
    q0 = _cpu_env(sway=0.0)._mimic_ref(cmd, 0.0, 0.75)
    _, (yr, _) = _soles(env, env._mimic_ref(cmd, 0.0, 0.75))
    _, (yr0, _) = _soles(env, q0)
    # at liftoff of L (phase 0) the torso is already sin(lead) of the way to R
    assert yr - yr0 == pytest.approx(SWAY * math.sin(0.785), abs=1.5e-3)


def test_zero_command_is_the_home_pose():
    env = _cpu_env()
    for ph in (0.0, 1.0, math.pi / 2, 4.0):
        assert np.allclose(env._mimic_ref(np.zeros(7), ph, 0.75), env._default, atol=1e-9)


def test_cpu_and_mjx_references_agree():
    import jax.numpy as jp
    import env_mjx as E
    import robot_plant
    kw = robot_plant.robot_env_kwargs(XML)
    kw.update(mimic_lift=0.02, mimic_sway=SWAY, mimic_sway_lead=0.785)
    c = W.BimoWalkerEnv(**kw)
    g = E.BimoMJXEnv(**kw)
    np.testing.assert_allclose(np.asarray(g._sway_k), c._sway_k, rtol=1e-6)
    worst = 0.0
    for v, vy, w in ((0.05, 0.0, 0.0), (0.2, 0.0, 0.3), (0.0, 0.15, 0.0), (0.0, 0.0, 0.0)):
        cmd = np.zeros(7)
        cmd[0], cmd[1], cmd[2], cmd[3] = v, vy, w, 1.0
        for ph in np.linspace(0.0, 2 * math.pi, 13):
            a = c._mimic_ref(cmd, ph, 0.75)
            b = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.75)))
            worst = max(worst, float(np.abs(a - b).max()))
    assert worst < 1e-4, worst
