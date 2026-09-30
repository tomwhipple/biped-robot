"""Offline tests for the tools/ scripts' pose assembly after the 17-servo port.

The bench scripts (squat_bench, leg_rom, ankle_probe, hold_hunt, yaw_probe,
imu_*_check, gyro_sign_check, joint_sweep, bench_rom_sweep) all now build
their `pose` arguments as a 17-wide bus-order list via
bus_cal.BusCal.pose_ticks(). The exact values they emit are a function of
the calibration; this file pins the *invariants* rather than re-deriving
every number:

  * output length is always 17 (== N_BUS),
  * the order is servo-ID order (i.e. ticks[b] lands on servo ID b+1),
  * joints not named in the offset dict are held at their zero_steps,
  * unfitted joints also get their zero (the firmware skips them on the
    wire, so a value here is a no-op; we still fill it for shape),
  * the legacy 10-wide policy-order form is still available for tools
    that drive the prototype's 10 joints only and need the old behaviour.

The fixture BusCal is the asbuilt prototype (10 fitted leg servos, 11..17
at v3 defaults NOT FITTED); the invariants hold for any BusCal.
"""

from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import bus_cal  # noqa: E402
from bus_cal import (  # noqa: E402
    BUS_NAMES, N_BUS, TICKS_PER_DEG, asbuilt_prototype_cal, parse_cal_show,
)


@pytest.fixture
def cal():
    return asbuilt_prototype_cal()


def test_pose_ticks_has_bus_width(cal):
    ticks = cal.pose_ticks()
    assert len(ticks) == N_BUS == 17


def test_pose_ticks_lands_on_the_right_servo_id(cal):
    """ticks[b] must be the goal for servo ID b+1 (bus_map.h invariant)."""
    offsets = {"L_knee": -30.0}
    ticks = cal.pose_ticks(offsets)
    # L_knee is bus joint 6 (servo ID 7). TPD = 4096/360. dir is -1.
    expected = int(round(1581 + (-1) * -30.0 * TICKS_PER_DEG))
    assert ticks[6] == expected
    # All other joints at their zero (fitted or not).
    for b, t in enumerate(ticks):
        if b == 6:
            continue
        assert t == cal.joints[b].zero_steps, (
            f"{cal.joints[b].name} (b={b}) moved without being named"
        )


def test_pose_ticks_unfitted_joints_get_their_zero(cal):
    """IDs 11..17 are NOT FITTED on the asbuilt prototype, but pose_ticks
    still fills their slots with their calibrated zero (2048) so the pose
    frame is 17 wide and the firmware can skip them on the wire.
    """
    ticks = cal.pose_ticks({})
    for b, j in enumerate(cal.joints):
        if not j.fitted:
            assert ticks[b] == j.zero_steps == 2048, (
                f"unfitted {j.name}: ticks[{b}]={ticks[b]}, zero={j.zero_steps}"
            )


def test_pose_ticks_asymmetric_offsets(cal):
    """The level-foot family's asymmetric offsets (hip -theta, knee -2*theta,
    ankle -theta) land on the right joint, with the right sign."""
    theta = 25.0
    off = {}
    for side in ("L", "R"):
        off[f"{side}_hip_pitch"] = -theta
        off[f"{side}_knee"] = -2 * theta
        off[f"{side}_ankle"] = -theta
    ticks = cal.pose_ticks(off)
    # Spot check L_hip_pitch: id 6 -> bus index 5; zero 2001, dir +1.
    assert ticks[5] == int(round(2001 + (+1) * -theta * TICKS_PER_DEG))
    # L_knee: id 7 -> bus index 6; zero 1581, dir -1.
    assert ticks[6] == int(round(1581 + (-1) * -2 * theta * TICKS_PER_DEG))
    # L_ankle: id 8 -> bus index 7; zero 3535, dir +1.
    assert ticks[7] == int(round(3535 + (+1) * -theta * TICKS_PER_DEG))


def test_policy_pose_ticks_shape(cal):
    """The legacy 10-wide policy-order pose is still available for tools
    driving the prototype's 10 joints only."""
    ticks = cal.policy_pose_ticks({"L_knee": -30.0})
    assert len(ticks) == 10
    # L_knee is at index 3 in policy order (L_hip_yaw, L_hip_roll,
    # L_hip_pitch, L_knee, ...). dir -1:
    expected = int(round(1581 + (-1) * -30.0 * TICKS_PER_DEG))
    assert ticks[3] == expected


def test_bus_and_policy_forms_agree_on_the_prototype(cal):
    """For a joint the prototype drives, ticks-as-a-17 and ticks-as-a-10
    must compute the same goal value (the bus form's value at the right
    index equals the policy form's value at the right index).
    """
    off = {"L_hip_pitch": -15.0, "R_knee": -20.0}
    bus = cal.pose_ticks(off)
    pol = cal.policy_pose_ticks(off)
    # L_hip_pitch (bus idx 5) -> policy idx 2. R_knee (bus idx 2) -> policy idx 8.
    assert bus[5] == pol[2]
    assert bus[2] == pol[8]


def test_pos_ticks_round_trip_through_ticks_to_deg(cal):
    """busStepsToAngle in firmware/actuation.h: the same linear map inverted.
    A script's readback path uses ticks_to_deg; pin it against deg_to_ticks.
    """
    for j in cal.joints[:10]:   # fitted only
        for deg in (-12.0, 0.0, 7.5):
            t = j.deg_to_ticks(deg)
            back = j.ticks_to_deg(t)
            assert back == pytest.approx(deg, abs=0.2), (
                f"{j.name}: {deg} -> {t} -> {back}"
            )
