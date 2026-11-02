"""The Qt console's core and stops at the socket (link/console/).

The window is not what is tested; what reaches the robot is. A UDP socket on
loopback stands in for the robot, and every datagram that arrives is decoded
with link/protocol.py. Design: docs/design-v6/2026-11-01-operator-console-ui.md
§12 ("Safety, at the socket").
"""
import os
import socket
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from console import core, options, stops  # noqa: E402
from console.controller import Controller  # noqa: E402
import protocol  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(core.DEFAULT_LIB),
                                reason="build it: make -C firmware/host console")


@pytest.fixture
def robot():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(0.05)
    yield s
    s.close()


def frames(sock, wait=0.2):
    """Every command frame waiting (or arriving within `wait`), decoded."""
    out = []
    end = time.monotonic() + wait
    while time.monotonic() < end:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        out.append(protocol.decode_command(data))
    return out


def make(robot_port, *extra, tmp_path):
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(robot_port),
                       "--tlm-port", "0", "--no-record", "--sessions",
                       str(tmp_path), "--keymap", str(tmp_path / "keys.conf"),
                       *extra])
    c = Controller(o)
    assert c.start()
    return c


def test_status_mirror_matches_the_library():
    lib = core.library()
    assert lib.bimo_status_size() == core.ctypes.sizeof(core.Status)
    assert lib.bimo_api_version() == core.API_VERSION


def test_an_observer_sends_only_stop_frames(robot, tmp_path):
    c = make(robot.getsockname()[1], tmp_path=tmp_path)
    assert c.readonly is True                     # the default role
    for name in ("arm", "stand", "home", "record", "reset_ext"):
        c.fire(name)
    for axis in range(core.NUM_AXES):
        assert not c.press(axis)
    c.set_ext(0.8, 0.0, 0.0, 0.0)
    c.set_mirror(True)
    for _ in range(10):
        c.tick()
        time.sleep(0.01)
    assert frames(robot) == []                    # silent, by the socket

    c.hold()
    # SOFT for the first 400 ms; nobody confirms it here, so after that it
    # escalates (test_an_unconfirmed_hold_escalates_to_hard).
    got = frames(robot, 0.25)
    assert got and all(f.stop and not f.estop for f in got)
    for b in c.stops:
        b.join(3.0)
    frames(robot, 0.05)

    c.fire("estop")
    got = frames(robot, 0.3)
    assert got and all(f.stop and f.estop and not f.arm for f in got)   # HARD
    for b in c.stops:
        b.join(3.0)
    rest = frames(robot, 0.1)
    assert all(f.stop for f in rest)              # the burst's tail, queued
    c.tick()
    c.shutdown()
    assert frames(robot, 0.1) == []               # an observer's quit: nothing


def test_a_driver_quits_disarmed(robot, tmp_path):
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.fire("arm")
    c.tick()
    assert c.press(0)
    c.tick()
    got = frames(robot)
    assert got[-1].arm and got[-1].enabled and got[-1].vx == pytest.approx(0.4, abs=1e-3)
    c.shutdown()
    got = frames(robot)
    assert len(got) == 10 and all(not f.arm and not f.enabled for f in got)


def test_an_unconfirmed_hold_escalates_to_hard(robot):
    b = stops.StopBurst(False, [("127.0.0.1", robot.getsockname()[1])],
                        seconds=1.0, confirm_ms=200.0)
    t0 = time.monotonic()
    b.start()
    got = []
    while time.monotonic() - t0 < 0.8:
        try:
            data, _ = robot.recvfrom(64)
        except socket.timeout:
            continue
        got.append((time.monotonic() - t0, protocol.decode_command(data)))
    b.join(3.0)
    soft = [t for t, f in got if f.stop and not f.estop]
    hard = [t for t, f in got if f.stop and f.estop]
    assert soft and hard and max(soft) < min(hard)
    assert 0.15 < min(hard) < 0.4
    assert b.escalated and not b.confirmed
    assert "escalated" in b.summary() and "NOT confirmed" in b.summary()


def test_a_confirmed_hold_stays_soft(robot):
    b = stops.StopBurst(False, [("127.0.0.1", robot.getsockname()[1])],
                        seconds=0.5, confirm_ms=300.0)
    b.start()
    bench = core.BENCH
    diag = protocol.pack_diag(False, True, protocol.ArmResult.DISARMED_STOP)
    b.feed(bench, diag)
    b.join(3.0)
    got = frames(robot, 0.1)
    assert got and all(not f.estop for f in got)
    assert b.confirmed and not b.escalated
    assert "CONFIRMED" in b.summary() and "operator STOP" in b.summary()


def test_stops_never_broadcast_from_loopback():
    assert stops.targets("127.0.0.1", 4210) == [("127.0.0.1", 4210)]
    assert stops.targets("192.0.2.30", 4210) == [("192.0.2.30", 4210),
                                                 ("255.255.255.255", 4210)]
    assert stops.targets("192.0.2.30", 4210, None) == [("192.0.2.30", 4210)]
    assert stops.targets(None, 4210, None) == []


def test_reset_is_refused_while_a_stop_goes_out(robot, tmp_path):
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.estop()
    c.fire("home")
    assert "RESET SERVOS refused" in c.core.note
    for b in c.stops:
        b.join(3.0)
    c.tick()
    c.fire("home")
    assert "reset servos" in c.core.note
    c.shutdown()
