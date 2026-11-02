"""rise_ref_hold: no handover at recovered (get-up campaign, 2026-10-23).

A residual get-up commands reference + 0.3 x action while down. At the held
stand (`recovered`) the env used to switch to the full action map,
default + span x action, and stop paying the reference tracking: the same
policy output then means a different target, and the tracking income (w 8 + 4
per step on the t3k recipe) stops. The retrained get-ups (t1k, t2k, u0k, u1k)
stand and then fall after that latch. With rise_ref_hold the residual map and
the tracking keep running after the stand (the reference holds its last,
standing frame). Off by default = the old handover.
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

REF = "getup_ref_v6ar_prone_ik.npz"


def _env(hold):
    import eval_precision as ep
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    held = {k: v for k, v in kw["held_joints"].items()
            if robot_plant._role(k) not in ("shoulder", "elbow")}
    secs, factory, _ = ep._registry()["recover_prone_fall"]
    cfg = dict(kw, held_joints=held, rise_ref_npz=REF, rise_ref_residual=0.3,
               w_rise_ref=8.0, w_rise_root=4.0, rise_ref_hold=hold,
               precision=True, action_map="full")
    env = ep.make_env(cfg, secs, True, kw["xml_path"],
                      extra=ep.ENV_EXTRA["recover_prone_fall"])
    env.reset(seed=3)
    drv = ep.Driver(env, None, 3, False)
    ep._ref_setup(env, drv, REF)
    return env


@pytest.mark.parametrize("hold", [False, True])
def test_action_map_after_the_stand(hold):
    env = _env(hold)
    assert env._recover_ep and not env._recovered
    a = np.full(env.action_space.shape[0], 0.5, np.float32)
    t = env._rise_t0 + env._step_i * env.control_dt
    down = env._action_to_ctrl(a)
    np.testing.assert_allclose(
        down, np.clip(env._rise_ref(t) + 0.3 * a, env._lo, env._hi))
    env._recovered = True
    up = env._action_to_ctrl(a)
    if hold:
        np.testing.assert_allclose(up, down)        # no handover
    else:
        assert np.max(np.abs(up - down)) > 0.1      # the old full-map switch


@pytest.mark.parametrize("hold", [False, True])
def test_tracking_pays_after_the_stand(hold):
    """Recovered, one step at zero action: the tracking terms pay with
    rise_ref_hold and not without (the tracking weights are the only
    difference between the two envs' reward here)."""
    rewards = {}
    for w in (0.0, 1.0):
        env = _env(hold)
        env.w_rise_ref *= w
        env.w_rise_root *= w
        env._recovered = True
        _, r, *_ = env.step(np.zeros(env.action_space.shape[0], np.float32))
        rewards[w] = r
    gain = rewards[1.0] - rewards[0.0]
    if hold:
        assert gain > 0.01
    else:
        assert gain == pytest.approx(0.0, abs=1e-9)


def test_mjx_flag_needs_a_reference():
    from env_mjx import BimoMJXEnv
    xml = os.path.join(SIM, "bimo_biped_v5body.xml")
    with pytest.raises(ValueError, match="rise_ref_hold needs rise_ref_npz"):
        BimoMJXEnv(xml_path=xml, ext_cmd=True, rise_ref_hold=True)
