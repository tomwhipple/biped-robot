"""arm_script's telemetry guard: what makes it stop driving the robot.

Run:  .venv/bin/python -m pytest tests/test_arm_guard.py -q

The guard is the only thing between a supervised bench probe and a robot
that keeps being commanded after it has stopped being a robot standing up.
Every case here is one the hardware has actually produced.
"""
import os
import select
import socket
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import arm_script                                               # noqa: E402
from protocol import (NUM_JOINTS, TLM_LEN_EXT_ATT, LinkState,  # noqa: E402
                      Telemetry, encode_telemetry)


@pytest.fixture
def driver():
    d = arm_script.Driver("127.0.0.1", tlm_port=0)
    yield d
    d.close()


def beacon(d, state, up_z=1.0, servo_err=0x00, seq=1, joints=(), up_xy=()):
    """Deliver one telemetry frame to the driver, as the robot would."""
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t = Telemetry(seq_echo=seq, state=state, vbat_v=11.4, up_z=up_z,
                  vx_est=0.0, wz_est=0.0, servo_err=servo_err,
                  loop_late_pct=0, joints=joints, up_xy=up_xy)
    tx.sendto(encode_telemetry(t), ("127.0.0.1", d.rx.getsockname()[1]))
    tx.close()
    # Wait for the frame to be READABLE before pumping. Loopback UDP is not
    # instantaneous: on this Mac (2026-09-24) a datagram is not yet readable
    # 0 ms after sendto but is by ~0.5 ms, so pumping straight away made this
    # test a race it lost every time. The driver's own non-blocking read is
    # right for its real-time loop; the test was assuming instant delivery.
    select.select([d.rx], [], [], 0.2)
    return d.pump(0.0, 0.0, 0x00)


def test_standing_robot_is_not_a_verdict(driver):
    assert beacon(driver, LinkState.STAND) is None
    assert beacon(driver, LinkState.LIVE) is None


def test_latched_fall_aborts_immediately(driver):
    """A FALLEN beacon ends the run on the FIRST frame (issue #29).

    The robot disarms itself on a fall, so the tilt guard below never sees
    another LIVE beacon to debounce -- and on 2026-08-31 the script simply
    kept running, streaming commands at a robot on the floor and holding ARM
    into the next phase while it was being picked up.
    """
    verdict = beacon(driver, LinkState.FALLEN, up_z=0.1)
    assert verdict and "FALLEN" in verdict


def test_tilt_guard_still_debounces_a_live_robot(driver):
    """A leaning-but-live robot needs 0.3 s of it, not one frame."""
    assert beacon(driver, LinkState.LIVE, up_z=0.2) is None
    driver.down_since = driver.now() - 1.0        # as if held for a second
    assert beacon(driver, LinkState.LIVE, up_z=0.2)


def test_benched_fault_bits_are_not_judged(driver):
    """A benched beacon repeats a stale snapshot; 0xff there means nothing."""
    for i in range(5):
        assert beacon(driver, LinkState.BENCH, servo_err=0xFF, seq=i) is None


def test_a_pose_and_attitude_beacon_reaches_the_guard(driver):
    """The longest beacon (FLAG_POSE | FLAG_ATT, 69 B) is read whole.

    Every receiver read 64 B, which protocol v2's 65 and 69 B frames outgrew:
    UDP truncated each one, decode refused it as stray traffic, and a probe
    armed with FLAG_POSE never saw a single beacon -- FALLEN included.
    """
    joints = tuple(0.01 * i for i in range(NUM_JOINTS))
    assert len(encode_telemetry(Telemetry(
        seq_echo=1, state=LinkState.STAND, vbat_v=11.4, up_z=1.0,
        vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0,
        joints=joints, up_xy=(0.1, -0.1)))) == TLM_LEN_EXT_ATT
    assert beacon(driver, LinkState.STAND, joints=joints,
                  up_xy=(0.1, -0.1)) is None
    assert driver.n_tlm == 1
    assert len(driver.tlm.joints) == NUM_JOINTS == 17
    assert driver.tlm.joints == pytest.approx(joints, abs=1e-3)
    assert driver.tlm.up_xy == pytest.approx((0.1, -0.1), abs=1e-3)

    verdict = beacon(driver, LinkState.FALLEN, up_z=0.1, seq=2,
                     joints=joints, up_xy=(0.9, 0.0))
    assert verdict and "FALLEN" in verdict


def test_osc_probe_swing_guard_names_every_bus_joint(tmp_path):
    """osc_probe's swing guard names the joint from the protocol's list.

    Its own table was the 10-joint prototype's names and order, so on a v2
    beacon (17 joints, bus order) it mislabelled the joints and raised
    IndexError the moment the worst swing was bus joint 10 or later -- out of
    the guard, past emergency(), with the robot still armed.
    """
    import osc_probe
    d = osc_probe.OscDriver("127.0.0.1", str(tmp_path / "osc.csv"),
                            tilt=0.5, pp_rad=0.5, tlm_port=0)
    try:
        verdict = None
        for k in range(4):
            joints = [0.0] * NUM_JOINTS
            joints[10] = 0.3 * k                  # R_ankle_roll swings
            verdict = beacon(d, LinkState.LIVE, seq=k + 1,
                             joints=tuple(joints), up_xy=(0.0, 0.0))
            if verdict:
                break
        assert verdict and "R_ankle_roll" in verdict
    finally:
        d.close()
