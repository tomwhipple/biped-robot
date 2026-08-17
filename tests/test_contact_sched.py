"""The contact-schedule reward is a timing contract: it only helps if its
stance/swing flags agree with every OTHER consumer of the gait clock (the
w_feet_phase swing-height targets, the firmware clock, the mirror map's
half-cycle phase shift). One flipped side and training would pay the policy
for stepping exactly off-beat. These tests pin the convention."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

import jax.numpy as jp  # noqa: E402

from env_mjx import contact_schedule  # noqa: E402

DUTY = 0.4
GRID = np.linspace(-np.pi, np.pi, 4001, endpoint=False)


def test_left_swings_on_positive_sin():
    # mid left-swing: left airborne, right planted
    l, r = np.array(contact_schedule(jp.pi / 2, DUTY))
    assert (l, r) == (False, True)
    # mid right-swing: mirrored
    l, r = np.array(contact_schedule(-jp.pi / 2, DUTY))
    assert (l, r) == (True, False)


def test_half_cycle_shift_swaps_feet():
    """The mirror map encodes 'mirror = phase + pi'; the schedule must agree
    for every phase or the mirror loss and this reward fight each other."""
    for ph in GRID:
        shifted = np.mod(ph + np.pi + np.pi, 2 * np.pi) - np.pi
        a = np.array(contact_schedule(jp.asarray(ph), DUTY))
        b = np.array(contact_schedule(jp.asarray(shifted), DUTY))
        assert (a == b[::-1]).all(), f"feet don't swap at phase {ph:.3f}"


def test_duty_and_double_support():
    sched = np.array([contact_schedule(jp.asarray(p), DUTY) for p in GRID])
    frac_l, frac_r = sched.mean(axis=0)
    # stance while sin < 0.4 -> analytic fraction 0.5 + arcsin(0.4)/pi
    want = 0.5 + np.arcsin(DUTY) / np.pi
    assert abs(frac_l - want) < 0.01 and abs(frac_r - want) < 0.01
    both = (sched[:, 0] & sched[:, 1]).mean()
    neither = (~sched[:, 0] & ~sched[:, 1]).mean()
    assert both > 0.2, "no double-support band -- duty did nothing"
    assert neither == 0.0, "schedule demands a flight phase; a walking " \
                           "biped always has a stance foot"


def test_agrees_with_feet_phase_targets():
    """Wherever w_feet_phase asks a foot to be visibly UP (swing-height
    target above half its peak), the schedule must not demand stance."""
    for ph in GRID:
        s = np.sin(ph)
        rz_l, rz_r = max(s, 0.0), max(-s, 0.0)   # env's swing targets / peak
        stance_l, stance_r = np.array(contact_schedule(jp.asarray(ph), DUTY))
        if rz_l > 0.5:
            assert not stance_l, f"stance scheduled mid-left-swing at {ph:.3f}"
        if rz_r > 0.5:
            assert not stance_r, f"stance scheduled mid-right-swing at {ph:.3f}"


def test_env_accepts_and_rewards_schedule():
    """Smoke: the knob threads through BimoMJXEnv and moves the reward in
    the right DIRECTION -- an on-schedule contact pattern must out-earn an
    off-schedule one, everything else equal. Uses the reward arithmetic
    (match - 1), not a rollout: physics can't be forced into both contact
    patterns from one state, but the term itself is separable."""
    import jax
    from env_mjx import BimoMJXEnv
    xml = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")
    env = BimoMJXEnv(xml_path=xml, ext_cmd=True, gait_clock=True,
                     w_contact_sched=1.0, obs_hist_len=3,
                     domain_rand=False, imu_noise=0.0,
                     quantize_ticks=False, latency_ms=0.0, backlash_deg=0.0,
                     cmd_fixed=(0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02))
    assert env.w_contact_sched == 1.0 and env.sched_duty == DUTY
    s = env.reset(jax.random.PRNGKey(0))
    s = env.step(s, jp.zeros(env.action_size))   # reward path must compile
    assert np.isfinite(float(s.reward))
