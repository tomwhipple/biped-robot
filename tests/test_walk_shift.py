"""Walking weight transfer (--w-walk-shift) and the low-swing clearance target
(--low-swing-clear), 2026-11-09.

Hardware, robot_legs_prec16_f075_js20 (0.75 Hz clock, 2 cm imitation lift,
trained with --w-low-swing 0): the right knee bent 45 deg in its swing but the
foot never left the floor -- hip roll moved 4-8 deg, the CoM never went over the
stance foot, and the loaded bending leg yawed the body 46 deg. Nothing paid for
clearance or weight transfer while walking: w_com_stance only shapes the one-leg
lift skill, and w_low_swing's fixed 3 cm fought the 2 cm lift.

--low-swing-clear makes that 3 cm a parameter (default unchanged; the referee's
SWING_CLEAR stays 3 cm). --w-walk-shift pays, per leg, its _mimic_ref swing
activity x a kernel on the CoM over the OTHER (stance) sole x the swinging sole's
clearance over --low-swing-clear: leaning with both feet down pays nothing.
"""
import inspect
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

ROBOT = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--precision", "--mjx-impl", "jax"]


def test_shortfall_clear_parameter():
    p = np.array([-0.01, 0.0, 0.01, 0.02, 0.025, 0.03, 0.08])
    # the default is exactly the fixed 3 cm rule
    np.testing.assert_array_equal(W.low_swing_shortfall(p, np),
                                  W.low_swing_shortfall(p, np, W.LOW_SWING_CLEAR))
    assert float(W.low_swing_shortfall(0.025, np, 0.03)) == pytest.approx(1.0 / 6.0)
    assert float(W.low_swing_shortfall(0.025, np, 0.02)) == 0.0
    assert float(W.low_swing_shortfall(0.01, np, 0.02)) == pytest.approx(0.5)


def test_defaults_and_cli():
    import train_mjx
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["low_swing_clear"].default == W.LOW_SWING_CLEAR
    sig = inspect.signature(BimoMJXEnv.__init__).parameters
    assert sig["w_walk_shift"].default == 0.0 and sig["walk_shift_sigma"].default == 0.015
    p = train_mjx.make_parser()
    kw = train_mjx.build_env_kw(p.parse_args(ROBOT))
    for k in ("low_swing_clear", "w_walk_shift", "walk_shift_sigma"):
        assert k not in kw
    k2 = train_mjx.build_env_kw(p.parse_args(ROBOT + ["--w-walk-shift", "1", "--low-swing-clear", "0.02",
                                                     "--walk-shift-sigma", "0.02"]))
    assert k2["w_walk_shift"] == 1.0 and k2["low_swing_clear"] == 0.02 and k2["walk_shift_sigma"] == 0.02
    with pytest.raises(ValueError, match="w_walk_shift"):
        BimoMJXEnv(**dict(kw, gait_clock=False, w_mimic=0.0, w_walk_shift=1.0))


def _cpu_reward(clear, peak):
    """Step reward of a standing CPU env (w_low_swing 2) whose left foot touches
    down after a 10-tick swing that peaked at `peak` m."""
    env = W.BimoWalkerEnv(xml_path=os.path.join(SIM, "bimo_biped_v6legs.xml"), command_mode=True,
                          ext_cmd=True, actuator_model="sts3215", domain_rand=False,
                          w_low_swing=2.0, low_swing_clear=clear)
    env.reset(seed=5)
    env._air_time[:] = [10 * env.control_dt, 0.0]
    env._swing_peak[:] = [peak, 0.0]
    _, r, *_ = env.step(np.zeros(env.action_space.shape[0], np.float32))
    assert env._foot_contacts()[0]
    return r


def test_cpu_env_low_swing_clear():
    # a 2.5 cm step: short of 3 cm by 1/6, clear of 2 cm
    assert _cpu_reward(0.02, 0.025) - _cpu_reward(0.03, 0.025) == pytest.approx(2.0 / 6.0, abs=1e-6)


def test_mjx_env_low_swing_clear():
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    xml = os.path.join(SIM, "bimo_biped_v6legs.xml")
    rewards = {}
    for clear in (0.03, 0.02):
        e = BimoMJXEnv(xml_path=xml, ext_cmd=True, domain_rand=False, w_low_swing=2.0,
                       low_swing_clear=clear)
        st = jax.jit(e.reset)(jax.random.PRNGKey(5))
        st = st._replace(air_time=jp.asarray([10 * e.control_dt, 0.0], jp.float32),
                         swing_peak=jp.asarray([0.025, 0.0], jp.float32),
                         recovered=jp.ones(()))
        nxt = jax.jit(e.step)(st, jp.zeros(e.action_size))
        assert float(nxt.air_time[0]) == 0.0       # the left foot did land
        rewards[clear] = float(nxt.reward)
    assert rewards[0.02] - rewards[0.03] == pytest.approx(2.0 / 6.0, abs=1e-4)


@pytest.fixture(scope="module")
def walk_states():
    """A walking state at gait phase ~pi/2 (left swings, right stands) in two
    poses whose whole-robot CoM sits over the right sole: the left foot lifted
    ~4 cm (hip pitch +0.5, knee +1.0 rad) and both feet down. The CoM is put
    there by moving the torso's inertial frame (a per-pose model), so both poses
    have the same CoM. Returns {pose: (state, model)} plus the plain env kwargs."""
    import jax
    import jax.numpy as jp
    import mujoco
    import train_mjx
    from env_mjx import BimoMJXEnv
    kw = dict(train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ROBOT)), domain_rand=False)
    e = BimoMJXEnv(**kw)
    mm = e.mj_model
    st0 = jax.jit(e.reset)(jax.random.PRNGKey(0))
    st0 = st0._replace(cmd=jp.asarray([0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], st0.cmd.dtype),
                       traj_on=jp.zeros_like(st0.traj_on), recovered=jp.ones(()),
                       gait_phase=jp.asarray(np.pi / 2 - 0.1, jp.float32))
    out = {}
    for pose, bend in (("lifted", (0.5, 1.0)), ("planted", (0.0, 0.0))):
        q = np.asarray(st0.data.qpos).copy()
        for n, v in zip(("L_hip_pitch", "L_knee"), bend):
            q[mm.jnt_qposadr[mm.joint(n).id]] += v
        d = mujoco.MjData(mm)
        d.qpos[:] = q
        mujoco.mj_forward(mm, d)
        b = e._torso_bid
        shift = ((d.geom_xpos[e._sole_gids[1]][:2] - d.subtree_com[b][:2])
                 * mm.body_subtreemass[b] / mm.body_mass[b])
        ipos = np.asarray(e.model.body_ipos).copy()
        ipos[b, :2] += d.xmat[b].reshape(3, 3)[:2, :2].T @ shift
        model = e.model.replace(body_ipos=jp.asarray(ipos, e.model.body_ipos.dtype))
        st = st0._replace(data=st0.data.replace(qpos=jp.asarray(q, st0.data.qpos.dtype)))
        out[pose] = (st, model)
    return out, kw


def _term(walk_states, **over):
    """Walk-shift reward term (reward with the override minus the plain env's) and
    the stepped state's metrics, per pose."""
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    states, kw = walk_states
    if "base" not in _STEPS:
        _STEPS["base"] = jax.jit(BimoMJXEnv(**kw).step)
    step = jax.jit(BimoMJXEnv(**dict(kw, **over)).step)
    res = {}
    for pose, (st, model) in states.items():
        act = jp.zeros(st.prev_action.shape)
        r0 = float(_STEPS["base"](st, act, model).reward)
        nxt = step(st, act, model)
        res[pose] = (float(nxt.reward) - r0, {k: float(v) for k, v in nxt.metrics.items()})
    return res


_STEPS = {}


def test_walk_shift_pays_only_for_a_lifted_swing_foot_over_a_loaded_stance(walk_states):
    off = _term(walk_states, w_walk_shift=0.0)
    for pose in off:
        assert off[pose][0] == 0.0                           # weight 0: reward unchanged
    on = _term(walk_states, w_walk_shift=1.0)
    t_lift, m_lift = on["lifted"]
    t_flat, m_flat = on["planted"]
    print(f"walk-shift term at W=1: lifted {t_lift:.4f}, planted {t_flat:.2e}; "
          f"metrics lifted {m_lift}")
    # the stance-side CoM pays only with the swing foot up (the clearance gate)
    assert t_lift > 0.5
    assert t_flat < 0.05 * t_lift
    # the walking diagnostics: swing activity ~1, clearance ~4 cm vs ~0, CoM over the stance sole
    assert m_lift["walk_swing_w"] > 0.9 and m_flat["walk_swing_w"] > 0.9
    assert m_lift["walk_swing_clear"] / m_lift["walk_swing_w"] > 0.03
    assert m_flat["walk_swing_clear"] / m_flat["walk_swing_w"] < 0.01
    assert m_lift["walk_shift_err"] / m_lift["walk_swing_w"] < 0.01
    # a lower clearance target opens the gate further only below it: a 4 cm lift is already full
    assert _term(walk_states, w_walk_shift=1.0, low_swing_clear=0.02)["lifted"][0] == pytest.approx(t_lift, rel=1e-3)
