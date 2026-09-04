"""arm_script's telemetry guard: what makes it stop driving the robot.

Run:  .venv/bin/python -m pytest tests/test_arm_guard.py -q

The guard is the only thing between a supervised bench probe and a robot
that keeps being commanded after it has stopped being a robot standing up.
Every case here is one the hardware has actually produced.
"""
import os
import socket
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import arm_script                                               # noqa: E402
from protocol import LinkState, Telemetry, encode_telemetry     # noqa: E402


@pytest.fixture
def driver():
    d = arm_script.Driver("127.0.0.1", tlm_port=0)
    yield d
    d.close()


def beacon(d, state, up_z=1.0, servo_err=0x00, seq=1):
    """Deliver one telemetry frame to the driver, as the robot would."""
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t = Telemetry(seq_echo=seq, state=state, vbat_v=11.4, up_z=up_z,
                  vx_est=0.0, wz_est=0.0, servo_err=servo_err,
                  loop_late_pct=0)
    tx.sendto(encode_telemetry(t), ("127.0.0.1", d.rx.getsockname()[1]))
    tx.close()
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
