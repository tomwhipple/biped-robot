"""experiments/sway-probe/sway_ref.py: the ladder generator plays the TRAINING reference.

Where the envelope is 1 the emitted keyframe IS `walker_env.BimoWalkerEnv._mimic_ref` at the same gait-clock
phase, built by a second, independent env with the sway in its constructor (as train_mjx builds it); the sway
rungs keep that reference's two roll channels and the standing pose everywhere else. CPU, no robot."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SWAY = os.path.join(HERE, "..", "experiments", "sway-probe")
sys.path.insert(0, SWAY)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import sway_ref as SR  # noqa: E402


@pytest.fixture(scope="module")
def ref():
    return SR.SwayReference()


def _training_env(M, lead):
    """An env built the way train_mjx builds the recipe's reference: the sway through the constructor."""
    return SR.make_env(SR.load_config(), None, M, lead)


def test_the_recipe_the_ladder_plays_has_the_reference_terms(ref):
    c = ref.cfg
    assert c["mimic_lift"] > 0 and c["mimic_sole_level"] and c["w_mimic"] > 0
    assert ref.env.mimic_lift == c["mimic_lift"]
    assert tuple(ref.names[:3]) == ("L_hip_yaw", "L_hip_roll", "L_hip_pitch") and len(ref.names) == 12


@pytest.mark.parametrize("M,lead", [(0.039, 0.785), (0.062, 0.0), (0.02, 1.2)])
def test_lift_rung_equals_the_training_reference_at_the_same_phase(ref, M, lead):
    r = ref.rung(1, "lift", M, lead)
    env = _training_env(M, lead)
    plateau = np.nonzero(r.plateau())[0]
    assert len(plateau) > 100
    for k in plateau[::7]:
        want = env._mimic_ref(SR.CMD_IN_PLACE, float(r.phase[k]), SR.CLOCK_HZ)
        assert np.array_equal(r.q[k], want), (k, r.q[k] - want)


def test_sway_rung_is_the_reference_rolls_on_the_standing_pose(ref):
    M, lead = 0.05, SR.LEAD
    r = ref.rung(1, "sway", M, lead)
    env = _training_env(M, lead)
    roll = ref.roll_idx
    other = np.setdiff1d(np.arange(12), roll)
    for k in np.nonzero(r.plateau())[0][::9]:
        want = env._mimic_ref(SR.CMD_IN_PLACE, float(r.phase[k]), SR.CLOCK_HZ)
        assert np.array_equal(r.q[k][roll], want[roll])
        assert np.array_equal(r.q[k][other], ref.stand[other])      # planted: no lift, no pitch
    assert np.abs(r.q[:, roll]).max() > 0.1                          # and it does move the rolls


def test_the_lift_is_the_references_and_the_sway_is_not_a_lift(ref):
    lift, sway = ref.rung(1, "lift", 0.04), ref.rung(2, "sway", 0.04)
    kn = [ref.names.index("L_knee"), ref.names.index("R_knee")]
    assert np.abs(lift.q[:, kn]).max() > 0.5            # the 2 cm IK lift bends the swing knee (~49 deg)
    assert np.abs(sway.q[:, kn]).max() == 0.0
    ph = lift.phase
    i = int(np.argmax(np.where(lift.plateau(), np.sin(ph), -9)))      # left swings on sin > 0
    assert abs(lift.q[i, kn[0]]) > abs(lift.q[i, kn[1]])


def test_envelope_ramps_in_and_out_to_the_standing_pose_at_50_hz(ref):
    r = ref.rung(1, "lift", 0.04, cycles=3, ramp_s=1.0)
    assert np.allclose(np.diff(r.t), 1 / 50)
    assert r.w[0] == 0.0 and r.w[-1] == 0.0
    assert np.array_equal(r.q[0], ref.stand) and np.array_equal(r.q[-1], ref.stand)
    assert np.all(np.diff(r.w[: 50]) >= 0) and np.all(np.diff(r.w[-50:]) <= 0)
    assert abs(r.t[-1] - (2 * 1.0 + 3 / SR.CLOCK_HZ)) < 1e-9
    # minimum jerk: zero velocity at both ends of the ramp
    assert abs(r.w[1] - r.w[0]) < 1e-3
    # the plateau lasts the cycles asked for
    assert abs(r.plateau().sum() / SR.KEY_HZ - 3 / SR.CLOCK_HZ) < 0.05


def test_ladder_is_sway_only_then_lift_at_the_same_amplitudes(ref):
    Ms = (0.02, 0.04, 0.06)
    L = ref.ladder(Ms)
    assert [x.kind for x in L] == ["sway"] * 3 + ["lift"] * 3
    assert [x.M for x in L] == list(Ms) * 2 and [x.index for x in L] == list(range(1, 7))
    assert np.abs(L[0].q[:, ref.roll_idx]).max() < np.abs(L[2].q[:, ref.roll_idx]).max()
    # the clock and lead are parameters
    r = ref.rung(1, "lift", 0.04, lead=0.3, hz=0.9, cycles=2)
    assert abs(r.phase[1] - r.phase[0] - 2 * math.pi * 0.9 / 50) < 1e-9 and r.phase[0] == -0.3


def test_reference_stays_inside_the_policy_ranges(ref):
    for r in ref.ladder():
        assert np.all(r.q >= ref.env._lo - 1e-12) and np.all(r.q <= ref.env._hi + 1e-12)
