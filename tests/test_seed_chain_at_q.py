"""seed_chain_at_q: the servo chain starts at the episode's start pose (2026-11-04).

Both envs used to seed the dead-time ring, the act-lag cascade, the firmware C2
shaper and the slew limiter's last target at the STAND pose. A get-up episode
starts lying, so every episode began with the lagged targets yanking toward
standing for a few ticks. The firmware seeds its shaper at measured q, so the
robot never does that. With seed_chain_at_q the chain starts at the start
pose's q (env_mjx at reset and reseed, walker_env on the first step, which IS
the reset state). Off by default = every past run's chain.
"""
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

REF = "getup_ref_v6ar_prone_ikc.npz"
CHAIN = dict(fw_shaper_hz=10.0, act_delay_types={"sts3215": (2, 2), "sts3250": (1, 1)})


def _walker(seed):
    import eval_precision as ep
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    held = {k: v for k, v in kw["held_joints"].items()
            if robot_plant._role(k) not in ("shoulder", "elbow")}
    secs, _, _ = ep._registry()["recover_prone_fall"]
    cfg = dict(kw, held_joints=held, rise_ref_npz=REF, rise_ref_residual=0.15,
               w_rise_ref=8.0, w_rise_root=4.0, rise_ref_hold=True,
               precision=True, action_map="full", seed_chain_at_q=seed, **CHAIN)
    env = ep.make_env(cfg, secs, True, kw["xml_path"], act_lag_hz=8.0,
                      extra=ep.ENV_EXTRA["recover_prone_fall"])
    env.reset(seed=3)
    drv = ep.Driver(env, None, 3, False)
    ep._ref_setup(env, drv, REF)
    return env


@pytest.mark.parametrize("seed", [False, True])
def test_walker_chain_starts_at_the_start_pose(seed):
    env = _walker(seed)
    assert env.seed_chain_at_q == seed
    q0 = np.asarray(env.data.qpos[env._sqpos], dtype=np.float64).copy()
    far = np.abs(np.asarray(env._sdefault) - q0).max()
    assert far > 0.5                     # lying: far from the stand pose
    env.step(np.zeros(env.action_space.shape[0], np.float32))
    np.testing.assert_allclose(env._chain0, q0 if seed else env._sdefault)
    # one tick of the 10 Hz shaper (its 3rd stage moves ~37 % of a step): the served
    # target is still nearer the pose the chain was seeded at
    served = env._shp_y[2]
    d_start = np.linalg.norm(served - q0)
    d_stand = np.linalg.norm(served - np.asarray(env._sdefault))
    assert (d_start < d_stand) == seed


def _mjx(seed):
    import robot_plant
    from env_mjx import BimoMJXEnv
    kw = robot_plant.robot_env_kwargs("bimo_biped_v6ar.xml")
    kw["held_joints"] = {k: v for k, v in (kw.get("held_joints") or {}).items()
                         if robot_plant._role(k) not in ("shoulder", "elbow")}
    kw.update(xml_path=os.path.join(SIM, "bimo_biped_v6ar.xml"), ext_cmd=True, recover_mix=1.0,
              domain_rand=False, rise_ref_npz=REF, rise_ref_hold=True,
              act_lag_hz=8.0, seed_chain_at_q=seed, **CHAIN)
    return BimoMJXEnv(**kw)


@pytest.mark.parametrize("seed", [False, True])
def test_mjx_chain_starts_at_the_start_pose(seed):
    import jax
    e = _mjx(seed)
    st = jax.jit(e.reset)(jax.random.PRNGKey(3))
    q0 = np.asarray(st.data.qpos[e._jq0:e._jq1])
    want = q0 if seed else np.asarray(e._sdefault)
    assert np.abs(np.asarray(e._sdefault) - q0).max() > 0.5
    for name in ("lag_y", "shp_y", "act_hist"):
        arr = np.asarray(getattr(st, name))
        np.testing.assert_allclose(arr, np.broadcast_to(want, arr.shape), atol=1e-6,
                                   err_msg=name)
    np.testing.assert_allclose(np.asarray(st.last_target), want, atol=1e-6)
    # the cheap auto-reset reuses the cached first state: same seed
    rs = jax.jit(e.reseed)(st, jax.random.PRNGKey(4))
    for name in ("lag_y", "shp_y", "act_hist"):
        arr = np.asarray(getattr(rs, name))
        np.testing.assert_allclose(arr, np.broadcast_to(want, arr.shape), atol=1e-6,
                                   err_msg="reseed " + name)
    np.testing.assert_allclose(np.asarray(rs.last_target), want, atol=1e-6)
