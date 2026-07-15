"""set_torque_enabled() must actually release, under both actuator models.

Run:  .venv/bin/python -m pytest tests/test_torque_release.py -q
(Slower than test_protocol.py -- these build a real MuJoCo model.)

This is the RELAX half of the link watchdog: if a dead link leaves the servos
still driving, the whole failsafe is decorative.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "sim"))

from walker_env import BimoWalkerEnv                             # noqa: E402

MODELS = ["ideal", "sts3215"]


def _drive(env, steps=150, action=-1.0):
    """Hold a hard crouch command; report the peak torque actually applied.

    A crouch is the probe because it fights gravity and the standing pose at
    once -- an engaged servo must produce torque to track it, a released one
    cannot produce any.
    """
    act = np.full(8, action, dtype=np.float32)
    peak = 0.0
    for _ in range(steps):
        env.step(act)
        f = env._servo_tau if env._servo is not None \
            else env.data.actuator_force
        peak = max(peak, float(np.abs(f).max()))
    return peak


@pytest.mark.parametrize("actuator_model", MODELS)
def test_released_servos_apply_exactly_zero_torque(actuator_model):
    env = BimoWalkerEnv(command_mode=True, actuator_model=actuator_model)
    env.reset(seed=0)
    env.set_torque_enabled(False)
    assert _drive(env) == 0.0


@pytest.mark.parametrize("actuator_model", MODELS)
def test_engaged_servos_do_apply_torque(actuator_model):
    # The control: without this, the test above would pass on a broken env
    # that never applies torque at all.
    env = BimoWalkerEnv(command_mode=True, actuator_model=actuator_model)
    env.reset(seed=0)
    assert _drive(env) > 0.5


@pytest.mark.parametrize("actuator_model", MODELS)
def test_release_is_reversible(actuator_model):
    env = BimoWalkerEnv(command_mode=True, actuator_model=actuator_model)
    env.reset(seed=0)
    env.set_torque_enabled(False)
    assert _drive(env, steps=25) == 0.0
    env.set_torque_enabled(True)          # link came back
    assert _drive(env, steps=75) > 0.5


@pytest.mark.parametrize("actuator_model", MODELS)
def test_toggling_is_idempotent(actuator_model):
    env = BimoWalkerEnv(command_mode=True, actuator_model=actuator_model)
    env.reset(seed=0)
    for _ in range(3):
        env.set_torque_enabled(True)      # no-ops must not corrupt the gains
    assert _drive(env, steps=50) > 0.5


def test_limp_on_flat_ground_does_not_imply_falling_over():
    """Documents a genuinely confusing observation, so nobody 'fixes' it.

    A released robot in the straight-legged standing pose is a column at a
    symmetric equilibrium: on a flat plane with no perturbation it just keeps
    standing, torque or no torque. Torque release is therefore NOT visible as
    a slump in a clean sim render -- verify it by torque (above), not by
    watching. Nudge it and the difference is immediate.
    """
    env = BimoWalkerEnv(command_mode=True, actuator_model="sts3215")
    env.reset(seed=0)
    env.set_torque_enabled(False)
    act = np.zeros(8, dtype=np.float32)
    for _ in range(100):
        env.step(act)
    assert env.data.qpos[2] > 0.2                  # still up, unperturbed

    env.data.qvel[0] = 0.7                         # now nudge it
    for _ in range(150):
        env.step(act)
    assert env.data.qpos[2] < 0.2                  # limp: it goes down


def test_reengage_preserves_domain_randomized_gains():
    """Re-engaging must restore the gains that were live, not the nominal
    ones -- under DR this episode runs scaled gains, and handing the policy a
    different plant than it fell asleep on would be a silent, ugly bug.
    """
    env = BimoWalkerEnv(command_mode=True, actuator_model="ideal",
                        domain_rand=True, gain_range=0.2)
    env.reset(seed=3)
    before = env.model.actuator_gainprm.copy()
    env.set_torque_enabled(False)
    assert np.abs(env.model.actuator_gainprm).max() == 0.0
    env.set_torque_enabled(True)
    np.testing.assert_allclose(env.model.actuator_gainprm, before)
