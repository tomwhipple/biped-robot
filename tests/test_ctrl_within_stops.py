"""ctrl_within_stops: the policy range never passes a mechanical stop.

The v6 plants give the ankle a +-45 deg ctrlrange but a +-40 deg joint stop,
so a full-map action of +-1 would drive the servo 5 deg into the stop on the
robot. --ctrl-within-stops clips the policy range to the stops; it is opt-in
(off = every older run's action scaling, bit for bit) and both envs agree.
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

import mujoco  # noqa: E402

import walker_env as W  # noqa: E402

LEGS_XML = os.path.join(SIM, "bimo_biped_v6legs.xml")


def _ranges(within):
    m = mujoco.MjModel.from_xml_path(LEGS_XML)
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    lo, hi = W.policy_range(m, within_stops=within)
    return names, np.degrees(lo), np.degrees(hi), m


def test_off_is_the_ctrlrange():
    names, lo, hi, _ = _ranges(False)
    i = names.index("L_ankle")
    assert (lo[i], hi[i]) == pytest.approx((-45.0, 45.0))


def test_on_clips_only_where_the_ctrlrange_passes_the_stop():
    names, lo0, hi0, m = _ranges(False)
    _, lo, hi, _ = _ranges(True)
    jnt = m.actuator_trnid[:, 0]
    jlo, jhi = np.degrees(m.jnt_range[jnt, 0]), np.degrees(m.jnt_range[jnt, 1])
    assert np.all(lo >= jlo - 1e-9) and np.all(hi <= jhi + 1e-9)
    for n in ("L_ankle", "R_ankle"):
        i = names.index(n)
        assert (lo[i], hi[i]) == pytest.approx((-40.0, 40.0))
    changed = {n for n, a, b, c, d in zip(names, lo, hi, lo0, hi0)
               if abs(a - c) > 1e-9 or abs(b - d) > 1e-9}
    assert changed == {"L_ankle", "R_ankle"}


def test_mjx_mirror_and_env_wiring():
    import env_mjx as M
    m = mujoco.MjModel.from_xml_path(LEGS_XML)
    for within in (False, True):
        a, b = W.policy_range(m, within_stops=within), M.policy_range(m, within_stops=within)
        assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
    env = W.BimoWalkerEnv(xml_path=LEGS_XML, ctrl_within_stops=True)
    i = env._act_names.index("L_ankle")
    assert np.degrees(env._hi[i]) == pytest.approx(40.0)
    assert env.ctrl_within_stops
