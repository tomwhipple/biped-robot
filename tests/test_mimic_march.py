"""--mimic-march / --march-lift: the march slice on the slow-gait recipe.

The knee-high march (--march-mix) is a LIFTED command (lift channel c4 alternating on the gait
clock, c6 raising the swing-foot target), and a lifted command gates w_mimic off. With
--mimic-march M the reference is back on during a march: the swing sole rises M metres on the
gait clock, --mimic-sway's lateral shift acts, and there is no stride (vx = vy = wz = 0).
--march-lift L sets the march's peak commanded swing height (default: the opposite knee's
standing height, 18 cm on the legs plant, which the link's 5 cm c6 clamp cannot send).
Everything is opt-in: with both at 0 the envs and the recipes' env kwargs are as they were.
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

import walker_env as W  # noqa: E402

XML = "bimo_biped_v6legs_meas_sole.xml"
ROBOT = ["--robot", "--xml", XML, "--precision", "--mjx-impl", "jax", "--mimic-lift", "0.02",
         "--family", "loco", "--gait-freq", "0.75,0.75"]
LIFT, SWAY = 0.035, 0.035


def _kw(*extra):
    import train_mjx
    p = train_mjx.make_parser()
    return dict(train_mjx.build_env_kw(p.parse_args(ROBOT + list(extra))), domain_rand=False)


def _env_kw(**over):
    import robot_plant
    kw = robot_plant.robot_env_kwargs(XML)
    kw.update(mimic_lift=0.02, mimic_sway=SWAY, mimic_sway_lead=0.0, gait_clock=True, w_mimic=1.5)
    kw.update(over)
    return kw


def _cpu_env(**over):
    return W.BimoWalkerEnv(**_env_kw(**over))


def _march_cmd():
    cmd = np.zeros(7)
    cmd[3] = 1.0
    cmd[4] = -1.0           # the left foot swings first
    return cmd


def _sole_pose(env, q):
    """(sole z, sole y in the torso frame) per sole for the policy-joint pose q."""
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    for i, n in enumerate(names[k] for k in env._pi):
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = q[i]
    mujoco.mj_kinematics(m, d)
    t = m.body("torso").id
    R = d.xmat[t].reshape(3, 3)
    return [(float(d.geom_xpos[g][2]), float((R.T @ (d.geom_xpos[g] - d.xpos[t]))[1]))
            for g in env._sole_gids]


# -- the flags ---------------------------------------------------------------------------

def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        sig = inspect.signature(cls.__init__).parameters
        assert sig["mimic_march"].default == 0.0 and sig["march_lift"].default == 0.0
    kw = _kw()
    assert "mimic_march" not in kw and "march_lift" not in kw
    k2 = _kw("--march-mix", "0.15", "--mimic-march", "0.035", "--march-lift", "0.07",
             "--mimic-sway", "0.035")
    assert (k2["mimic_march"], k2["march_lift"], k2["march_mix"]) == (0.035, 0.07, 0.15)


def test_the_recipe_with_the_march_slice_builds_on_both_engines():
    """make_parser accepts the slice; the CPU env and the MJX env (jax impl, CPU) build from it
    and agree on the march geometry."""
    from env_mjx import BimoMJXEnv
    import robot_plant
    kw = _kw("--march-mix", "0.15", "--mimic-march", "0.035", "--march-lift", "0.07",
             "--mimic-sway", "0.035", "--w-com-stance", "0.75", "--w-knee-high", "1.0")
    full = dict(robot_plant.robot_env_kwargs(XML), **kw)
    full["xml_path"] = os.path.join(SIM, XML)
    g = BimoMJXEnv(**full)
    c = W.BimoWalkerEnv(**{k: v for k, v in full.items()
                           if k in inspect.signature(W.BimoWalkerEnv.__init__).parameters
                           and k not in ("mjx_impl", "warp_envs")}, command_mode=True)
    assert g.march_mix == c.march_mix == 0.15
    assert c.march_clear == pytest.approx(0.07) and g.march_clear == pytest.approx(0.07)
    assert c._march_dz == pytest.approx(0.07 - c.lift_height) == pytest.approx(g._march_dz)


@pytest.mark.parametrize("over", [dict(mimic_lift=0.0), dict(mimic_march=0.09),
                                  dict(march_hz=1.0), dict(gait_clock=False, w_mimic=0.0)])
def test_mimic_march_refuses_what_it_cannot_follow(over):
    """It needs the IK lift table (<= 8 cm), the gait clock, and a march on that clock."""
    with pytest.raises(ValueError):
        _cpu_env(**dict(dict(mimic_march=LIFT), **over))


# -- byte-identity -------------------------------------------------------------------------

def test_default_march_geometry_is_the_knee_height():
    e = _cpu_env()
    assert e.march_clear == pytest.approx(e._knee_z0 - e._sole_z0) and e.march_clear > 0.15
    assert e._march_dz == pytest.approx(e.march_clear - e.lift_height)
    assert _cpu_env(march_lift=0.06)._march_dz == pytest.approx(0.0)


def test_reference_without_mimic_march_ignores_the_march_flag():
    """mimic_march 0: the reference is the one that trained every earlier run, whatever a caller
    passes for `march`."""
    import jax.numpy as jp
    import env_mjx as E
    c = _cpu_env()
    g = E.BimoMJXEnv(**_env_kw())
    walk = np.array([0.15, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0])
    for cmd in (np.zeros(7), _march_cmd(), walk):
        for ph in (0.0, 1.0, 2.5, 4.0):
            ref = c._mimic_ref(cmd, ph, 0.75)
            np.testing.assert_array_equal(c._mimic_ref(cmd, ph, 0.75, march=True), ref)
            a = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.75)))
            b = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.75),
                                        march=jp.asarray(True)))
            np.testing.assert_array_equal(a, b)
    # no march: the lifted command is the home pose (a lifted command is not a locomotion command)
    np.testing.assert_allclose(c._mimic_ref(_march_cmd(), 1.0, 0.75), c._default, atol=1e-9)


def test_mimic_march_leaves_a_walk_and_a_stand_alone():
    """With the flag on, only a march changes the reference."""
    on, off = _cpu_env(mimic_march=LIFT), _cpu_env()
    walk = np.array([0.15, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0])
    for cmd in (np.zeros(7), walk):
        for ph in (0.0, 1.0, 4.0):
            np.testing.assert_array_equal(on._mimic_ref(cmd, ph, 0.75), off._mimic_ref(cmd, ph, 0.75))
            np.testing.assert_array_equal(on._mimic_ref(cmd, ph, 0.75, march=False),
                                          off._mimic_ref(cmd, ph, 0.75))


# -- the march reference ---------------------------------------------------------------------

@pytest.mark.parametrize("phase,swing", [(math.pi / 2, 0), (-math.pi / 2, 1)])
def test_swing_foot_lifts_on_the_clock_and_the_torso_goes_over_the_stance_foot(phase, swing):
    e = _cpu_env(mimic_march=LIFT)
    home = _sole_pose(e, e._default)
    # the march reference at mid-swing (the sway leads by 0, so it peaks there too)
    q = e._mimic_ref(_march_cmd(), phase, 0.75, march=True)
    z, y = zip(*_sole_pose(e, q))
    assert z[swing] - home[swing][0] == pytest.approx(LIFT, abs=2e-3)         # the swing sole: M up
    assert abs(z[1 - swing] - home[1 - swing][0]) < 2e-3                      # the stance sole stays down
    # torso toward the stance foot: both soles move the other way in the torso frame
    want = SWAY if swing == 0 else -SWAY
    assert y[1 - swing] - home[1 - swing][1] == pytest.approx(want, abs=1.5e-3)


def test_no_stride_in_a_march():
    """vx = vy = wz = 0: the hip pitch carries the lift only, the same for both phases' stance leg,
    and the reference is mirror-symmetric between the two swings."""
    e = _cpu_env(mimic_march=LIFT)
    qa = e._mimic_ref(_march_cmd(), math.pi / 2, 0.75, march=True)
    qb = e._mimic_ref(_march_cmd(), -math.pi / 2, 0.75, march=True)
    d = e._default
    il, ir = e._i_pitch
    assert qa[ir] == pytest.approx(d[ir], abs=1e-9) and qb[il] == pytest.approx(d[il], abs=1e-9)
    # the two swing legs' lift joints are mirror images (same IK table, same sign convention)
    assert qa[il] - d[il] == pytest.approx(qb[ir] - d[ir], rel=0.05)
    ik, jk = e._i_knee
    assert qa[ik] - d[ik] == pytest.approx(qb[jk] - d[jk], rel=0.05)


def test_march_sway_follows_mimic_sway_lead():
    e = _cpu_env(mimic_march=LIFT, mimic_sway_lead=0.785)
    home = _sole_pose(e, e._default)
    q = e._mimic_ref(_march_cmd(), 0.0, 0.75, march=True)       # swing sw = 0, sway = sin(lead)
    _, y = zip(*_sole_pose(e, q))
    assert y[1] - home[1][1] == pytest.approx(SWAY * math.sin(0.785), abs=1.5e-3)


def test_cpu_and_mjx_references_agree():
    import jax.numpy as jp
    import env_mjx as E
    kw = _env_kw(mimic_march=LIFT, mimic_sway_lead=0.785, march_lift=0.07)
    c, g = W.BimoWalkerEnv(**kw), E.BimoMJXEnv(**kw)
    worst = 0.0
    for lift_side in (-1.0, 1.0):
        cmd = _march_cmd()
        cmd[4] = lift_side
        cmd[6] = 0.01
        for ph in np.linspace(0.0, 2 * math.pi, 13):
            a = c._mimic_ref(cmd, ph, 0.75, march=True)
            b = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.75),
                                        march=jp.asarray(True)))
            worst = max(worst, float(np.abs(a - b).max()))
            a = c._mimic_ref(cmd, ph, 0.75, march=False)
            b = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.75),
                                        march=jp.asarray(False)))
            worst = max(worst, float(np.abs(a - b).max()))
    assert worst < 1e-4, worst


# -- the reward gate ---------------------------------------------------------------------------

def _march_env(**over):
    kw = _env_kw(ext_cmd=True, w_lift=1.0, w_track_foot=1.0, march_mix=1.0,
                 ext_mix=(0.0,) * 7, **over)
    kw.pop("cmd_fixed", None)
    return W.BimoWalkerEnv(command_mode=True, domain_rand=False, **kw)


def test_the_march_reference_pays_only_in_a_march():
    """A march command is a lifted command, which gates w_mimic off; mimic_march gates it back on
    for the march (traj mode 4) and not for a one-leg balance."""
    for over, expect in ((dict(), None), (dict(mimic_march=LIFT), True)):
        e = _march_env(**over)
        e.reset(seed=1)
        assert e._traj_on == 4.0
        seen = []
        real = e._mimic_ref
        e._mimic_ref = lambda cmd, phase, freq, **k: (seen.append(k.get("march")), real(cmd, phase, freq, **k))[1]
        z0 = np.zeros(e.action_space.shape[0], dtype=np.float32)
        e.step(z0)                       # the draw's first tick: c4 = 0, the stand reference
        assert seen[-1] is (False if expect else None)
        e.step(z0)                       # the alternation has started
        assert seen[-1] == expect
    # the gate: with mimic_march the reward moves (the reference term is live) ...
    on, off = _march_env(mimic_march=LIFT), _march_env()
    on.reset(seed=2)
    off.reset(seed=2)
    z = np.zeros(on.action_space.shape[0], dtype=np.float32)
    on.step(z), off.step(z)              # the draw's first tick: c4 = 0, the stand reference either way
    r_on = sum(on.step(z)[1] for _ in range(4))
    r_off = sum(off.step(z)[1] for _ in range(4))
    assert r_on - r_off > 0.3            # w_mimic 1.5 x exp(-dq^2 / s2) per tick, near 1 at the stand
    # ... and a one-leg balance command (not a march) still gates it off
    bal = _march_env(mimic_march=LIFT)
    bal.reset(seed=2)
    bal._traj_on = 0.0
    bal._cmd[:] = (0, 0, 0, 1, 1.0, 0, 0)
    seen = []
    real = bal._mimic_ref
    bal._mimic_ref = lambda cmd, phase, freq, **k: (seen.append(k.get("march")), real(cmd, phase, freq, **k))[1]
    bal._cmd_next = 10 ** 9
    bal.step(z)
    assert seen[-1] is False        # lifted, but not a march


def test_mjx_reward_gate_is_the_march_only():
    """The MJX env pays the march reference in a march (mode 4) and only there: the same tick's
    reward rises with mimic_march by about w_mimic x the kernel, and a balance command's does not."""
    import jax
    import jax.numpy as jp
    import env_mjx as E
    base = _env_kw(ext_cmd=True, w_lift=1.0, w_track_foot=1.0, march_mix=1.0, ext_mix=(0.0,) * 7,
                   gait_freq_range=(0.75, 0.75))
    base.pop("cmd_fixed", None)

    def reward(over, mode=None):
        env = E.BimoMJXEnv(domain_rand=False, **dict(base, **over))
        st = env.reset(jax.random.PRNGKey(3))
        if mode is not None:      # a one-leg balance: lift commanded, no march mode
            st = st._replace(traj_on=jp.asarray(mode, jp.float32),
                            cmd=jp.asarray([0, 0, 0, 1, 1.0, 0, 0], jp.float32))
        assert float(st.traj_on) == (4.0 if mode is None else mode)
        step = jax.jit(env.step)
        st = step(st, jp.zeros(env.action_size))      # the draw's first tick: c4 = 0
        return float(step(st, jp.zeros(env.action_size)).reward)

    d_march = reward(dict(mimic_march=LIFT)) - reward({})
    d_bal = reward(dict(mimic_march=LIFT), mode=0.0) - reward({}, mode=0.0)
    assert d_march > 0.2, d_march
    assert abs(d_bal) < 1e-6, d_bal
