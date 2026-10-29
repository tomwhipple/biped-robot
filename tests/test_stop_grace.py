"""Stop grace (stop_grace_s, 2026-10-29).

The referee's walk_steps counts a step that begins within 0.4 s of the stop,
and it must clear 3 cm. At a plain stand every gait term is off and the
w_single_support term pays double support, so the walkers closed the stance
with a ~1 cm shuffle whatever the reward weights. stop_grace_s keeps the gait
shaping on for that long after a moving command. Reward only: the clock
freeze and the obs keep the raw command (the referee's CPU env has no grace).
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

XML = os.path.join(SIM, "bimo_biped_v6legs.xml")


def _step(grace, since_move, moving=False):
    """One step from a double-support stand under a stand (or walk) command;
    returns (reward, next since_move)."""
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    e = BimoMJXEnv(xml_path=XML, ext_cmd=True, domain_rand=False,
                   w_single_support=0.3, stop_grace_s=grace)
    st = jax.jit(e.reset)(jax.random.PRNGKey(5))
    cmd = st.cmd.at[0].set(0.3 if moving else 0.0).at[1].set(0.0)
    cmd = cmd.at[2].set(0.0).at[4].set(0.0)
    st = st._replace(cmd=cmd, cmd_next=jp.asarray(10 ** 6, st.cmd_next.dtype),
                     since_move=jp.asarray(since_move, jp.float32),
                     recovered=jp.ones(()))
    nxt = jax.jit(e.step)(st, jp.zeros(e.action_size))
    return float(nxt.reward), float(nxt.since_move), e.control_dt


def test_since_move_counts_from_the_stop():
    _, sm, dt = _step(0.4, 0.1)
    assert np.isclose(sm, 0.1 + dt)
    _, sm, _ = _step(0.4, 0.1, moving=True)
    assert sm == 0.0


def test_grace_off_by_default_changes_nothing():
    r_near, _, _ = _step(0.0, 0.1)
    r_far, _, _ = _step(0.0, 10.0)
    assert np.isclose(r_near, r_far, atol=1e-6)


def test_grace_treats_the_stop_as_a_walk():
    # both feet down: a stand pays w_single_support for double support, a
    # walk pays it only for single support, so the grace window costs 0.3
    r_grace, _, _ = _step(0.4, 0.1)
    r_stand, _, _ = _step(0.4, 10.0)
    assert np.isclose(r_grace - r_stand, -0.3, atol=1e-4)
