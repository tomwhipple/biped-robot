"""The sim's "asbuilt" stiffness preset IS the firmware's gains table.

firmware/main/servo_gains.h carries, per servo ID, the position-loop gains the
robot arms with (kExpectedGains) and the stiffness those gains realise in the
sim's units (kSimKpScale). The trainer's `--servo-kp-scale asbuilt`
(SERVO_KP_PRESETS in sim/mjx/env_mjx.py, mirrored in sim/walker_env.py) must
be exactly those factors, joint for joint, or a policy trained "as built"
would not build against the table (servo_gains.h static_asserts it) -- or
worse, would build against a table it never trained on. Pure text + numpy:
no jax, no plant physics.
"""
import os
import re
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
sys.path.insert(0, os.path.join(ROOT, "sim"))

import walker_env as W  # noqa: E402

GAINS_H = os.path.join(ROOT, "firmware", "main", "servo_gains.h")
BUS_H = os.path.join(ROOT, "firmware", "components", "obs", "include", "obs", "bus_map.h")
LEGS_XML = os.path.join(ROOT, "sim", "bimo_biped_v6legs.xml")


def _table():
    txt = open(GAINS_H).read()
    consts = {m.group(1): float(m.group(2)) for m in re.finditer(
        r"inline constexpr float (k\w+) = ([0-9.]+)f;", txt)}
    body = re.search(r"kSimKpScale\[\] = \{(.*?)\n\};", txt, re.S).group(1)
    factors = {int(i): consts[c] for i, c in re.findall(r"\{(\d+), (k\w+)\}", body)}
    exp = re.search(r"kExpectedGains\[\] = \{(.*?)\n\};", txt, re.S).group(1)
    gains = {int(i): g for i, g in re.findall(r"\{(\d+), ([\w:{}, ]+?)\},\s*//", exp)}
    return factors, gains


def _bus():
    txt = open(BUS_H).read()
    names = re.findall(r'"(\w+)",\s*//', re.search(
        r"kBusJointNames\[kNumBusJoints\] = \{(.*?)\};", txt, re.S).group(1))
    ids = [int(x) for x in re.search(
        r"kBusServoId\[kNumBusJoints\] = \{(.*?)\};", txt, re.S).group(1).split(",")]
    assert len(names) == len(ids) == 17
    return dict(zip(names, ids))


def test_table_rows_are_row_for_row():
    factors, gains = _table()
    assert sorted(factors) == sorted(gains)
    assert set(_bus().values()) == set(factors)


def test_asbuilt_preset_is_the_firmware_table():
    factors, _ = _table()
    bus = _bus()
    m = mujoco.MjModel.from_xml_path(LEGS_XML)
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    assert len(names) == 12
    v = W.servo_kp_scale_vector(names, "asbuilt")
    for n, f in zip(names, v):
        # the firmware compares float32 to float32 (obs_spec.h vs this table)
        assert np.float32(f) == np.float32(factors[bus[n]]), (n, f, factors[bus[n]])


def test_asbuilt_is_the_factory_table_today():
    """The table as of 2026-10-30: every servo factory P 32 / D 32, STS3250
    hip rolls 4.0, every STS3215 2.8 (servo_gains.h names the evidence)."""
    factors, gains = _table()
    assert set(gains.values()) == {"scsbus::kFactoryGains"}
    bus = _bus()
    for n, i in bus.items():
        want = 4.0 if n.endswith("hip_roll") else 2.8
        assert factors[i] == want, n
