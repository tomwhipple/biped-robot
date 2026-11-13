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


def test_a_library_older_than_its_sources_is_refused(tmp_path, monkeypatch):
    """A build older than any of its sources (client/console_api/keymap/
    recorder, linkproto incl. the generated cmd_envelope.h) is refused with
    the rebuild command; with no sources present (an installed copy) the
    mtime check is skipped."""
    srcs = core._sources()
    assert any(p.endswith("cmd_envelope.h") for p in srcs)
    assert any(p.endswith("console_api.cpp") for p in srcs)
    lib = tmp_path / core.LIB_NAME
    lib.write_bytes(b"")
    os.utime(lib, (1, 1))                       # 1970: older than everything
    monkeypatch.setattr(core, "_lib", None)
    with pytest.raises(core.LibraryError, match="older than .*make -C firmware/host console"):
        core.library(str(lib))
    monkeypatch.setattr(core, "_sources", lambda: [])
    assert core.stale_sources(str(lib)) == []


def test_a_library_missing_a_function_is_refused(tmp_path, monkeypatch):
    """A symbol the binding calls but the library lacks is a LibraryError
    naming the rebuild, not an AttributeError -- checked after the version."""
    monkeypatch.setattr(core, "_lib", None)
    monkeypatch.setattr(core, "_sources", lambda: [])
    monkeypatch.setitem(core._SIGS, "bimo_no_such_function", (core._I, []))
    with pytest.raises(core.LibraryError, match="no bimo_no_such_function.*rebuild"):
        core.library(core.DEFAULT_LIB)
    monkeypatch.setattr(core, "API_VERSION", core.API_VERSION + 100)
    with pytest.raises(core.LibraryError, match="API version.*rebuild"):
        core.library(core.DEFAULT_LIB)


def test_no_beacon_lag_while_benched():
    """#237: benched, seq_echo is the diagnostic byte; the lag is not shown
    (Link::lag() gives 0 there, test_client.cpp)."""
    st = core.Status()
    st.state, st.lag = core.BENCH, 0
    assert core.lag_text(st) == "lag —" and core.lag_text(st, 3) == "lag   —"
    st.state, st.lag = core.LIVE, 3
    assert core.lag_text(st) == "lag 3" and core.lag_text(st, 3) == "lag   3"


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
    # Escalates no earlier than the 200 ms confirm window; the upper bound only
    # has to catch "never escalated" (a loaded runner can be late by 100s of ms).
    assert 0.15 < min(hard) < 0.75
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


def _link_twin(port, tlm_port, seconds):
    import threading
    import link_twin
    t = threading.Thread(target=link_twin.run, daemon=True,
                         kwargs=dict(port=port, tlm_port=tlm_port,
                                     duration=seconds, quiet=True))
    t.start()
    time.sleep(0.2)
    return t


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_hold_on_a_benched_robot_is_confirmed_soft(tmp_path):
    """HOLD on a robot already BENCH (link_twin, booted benched): the robot
    reports no new reason for a benched soft stop (firmware cli.cpp), so BENCH
    with no reset going on confirms it -- no escalation to torque off."""
    port, tlm = _free_port(), _free_port()
    twin = _link_twin(port, tlm, 4.0)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port),
                       "--tlm-port", str(tlm), "--no-record", "--sessions",
                       str(tmp_path), "--keymap", str(tmp_path / "keys.conf"),
                       "--drive"])
    c = Controller(o)
    assert c.start()
    try:
        end = time.monotonic() + 2.0
        while time.monotonic() < end and not (c.st and c.st.fresh):
            c.tick()
            time.sleep(0.02)
        assert c.st.fresh and c.st.state == core.BENCH
        c.hold()
        b = c.stops[0]
        assert b.was_bench
        end = time.monotonic() + 3.0
        while c.stops and time.monotonic() < end:
            c.tick()
            time.sleep(0.02)
        assert b.confirmed and not b.escalated and not b.hard, b.summary()
        assert "SOFT STOP CONFIRMED" in b.summary()
    finally:
        c.shutdown()
        twin.join(5.0)


def test_hold_on_a_running_robot_still_needs_the_stop_reason(robot):
    """Not benched at the press: BENCH without DISARMED_STOP is no proof."""
    b = stops.StopBurst(False, [("127.0.0.1", robot.getsockname()[1])],
                        seconds=0.3, confirm_ms=150.0, was_bench=False)
    b.start()
    b.feed(core.BENCH, protocol.pack_diag(False, True, protocol.ArmResult.ACCEPTED))
    b.join(3.0)
    assert b.escalated and not b.confirmed


class _Beacons:
    """A robot that beacons `state` to the console's telemetry port."""

    def __init__(self, tlm_port, state=protocol.LinkState.LIVE):
        import threading
        self.state = state
        self.port = tlm_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.halt = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while not self.halt.is_set():
            t = protocol.Telemetry(seq_echo=0, state=self.state, vbat_v=12.0, up_z=1.0,
                                   vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
            self.sock.sendto(protocol.encode_telemetry(t), ("127.0.0.1", self.port))
            time.sleep(0.02)

    def close(self):
        self.halt.set()
        self.t.join(1.0)
        self.sock.close()


def test_drive_adopts_a_running_robots_arm_level(robot, tmp_path):
    """--drive against a robot another console drives (LIVE): it starts as an
    observer, hears the beacon and adopts, so its first frame already holds
    ARM -- no 1 -> 0 edge that would bench the robot mid-stride."""
    tlm = _free_port()
    bea = _Beacons(tlm)
    try:
        o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(robot.getsockname()[1]),
                           "--tlm-port", str(tlm), "--no-record", "--sessions",
                           str(tmp_path), "--keymap", str(tmp_path / "keys.conf"), "--drive"])
        c = Controller(o)
        assert c.start()
        assert not c.readonly and "RUNNING robot" in c.core.note
        for _ in range(5):
            c.tick()
            time.sleep(0.02)
        got = frames(robot)
        assert got and all(f.arm for f in got)
        c.shutdown()
    finally:
        bea.close()


def test_drive_with_no_beacon_starts_disarmed_and_says_so(robot, tmp_path):
    t0 = time.monotonic()
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    assert 0.9 < time.monotonic() - t0 < 3.0          # it waited ~1 s for a beacon
    assert not c.readonly and "NO beacon" in c.core.note
    c.tick()
    got = frames(robot)
    assert got and not got[-1].arm
    c.shutdown()


def test_a_failed_relink_leaves_no_link(robot, tmp_path):
    """A failed relink sends nothing anywhere -- least of all to the old
    robot -- and nothing names the old host any more."""
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.fire("arm")
    c.tick()
    frames(robot)
    assert not c.relink("not-an-address", robot.getsockname()[1], 0)
    assert c.readonly and c.host is None and c.o.host is None and not c.linked
    assert "NO LINK" in c.status_msg and "disarmed first" in c.status_msg
    st = c.tick()
    assert st.readonly and not st.has_dest
    for name in ("arm", "stand"):
        c.fire(name)
    for _ in range(5):
        c.tick()
        time.sleep(0.01)
    got = frames(robot, 0.1)
    assert got == [] or all(not f.arm for f in got)     # only the parting disarm
    ok, _ = c.core.toggle_control(c.host, core.now_ms())
    assert not ok                                        # nothing to take control of
    c.shutdown()


def test_relink_says_disarmed_only_when_it_was_armed(robot, tmp_path):
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.tick()
    assert c.relink("127.0.0.1", robot.getsockname()[1], 0)
    assert "disarmed by the move" not in c.status_msg
    c.fire("arm")
    c.tick()
    assert c.relink("127.0.0.1", robot.getsockname()[1], 0)
    assert "disarmed by the move" in c.status_msg
    c.shutdown()


def test_watch_only_keeps_its_status(robot, tmp_path):
    """The handover's message stays the status: the mirror re-request on
    the way through says nothing."""
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    assert c.toggle_control()
    assert c.status_msg.startswith("WATCHING") and "mirror off" not in c.status_msg
    c.shutdown()


def test_arm_and_reset_are_refused_until_the_real_end_of_a_stop(robot, tmp_path):
    """The countdown follows the burst's real end, which escalation moves."""
    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.hold()                                   # unconfirmed: escalates at 400 ms
    time.sleep(0.6)
    b = c.stops[0]
    assert b.escalated and b.end > b.t0 + b.seconds
    c.fire("arm")
    assert "ARM refused" in c.core.note
    left = float(c.core.note.split("(")[1].split(" s left")[0])
    assert left == pytest.approx(b.end - time.monotonic(), abs=0.15) and left > 0.45
    c.fire("home")
    assert "RESET SERVOS refused" in c.core.note
    b.join(3.0)
    c.tick()
    c.fire("arm")
    assert "arm sent" in c.core.note
    c.shutdown()


def test_stops_resolve_localhost_before_deciding_to_broadcast():
    assert stops.targets("localhost", 4210) == [("127.0.0.1", 4210)]
    assert stops.is_loopback("localhost") and not stops.is_loopback("192.0.2.30")


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


class _Lines:
    """A child's stdout, line by line, read on a thread so a test can wait for
    a line (readiness, an outcome) instead of sleeping a guessed time."""

    def __init__(self, proc):
        import threading
        self.proc, self.lines = proc, []
        self._cv = threading.Condition()
        self._t = threading.Thread(target=self._read, daemon=True)
        self._t.start()

    def _read(self):
        for ln in self.proc.stdout:
            with self._cv:
                self.lines.append(ln.rstrip("\n"))
                self._cv.notify_all()
        with self._cv:
            self.lines.append(None)       # EOF
            self._cv.notify_all()

    def wait(self, pred, timeout):
        """The first line matching `pred` (within `timeout` s), else None."""
        end = time.monotonic() + timeout
        seen = 0
        with self._cv:
            while True:
                for ln in self.lines[seen:]:
                    if ln is not None and pred(ln):
                        return ln
                seen = len(self.lines)
                left = end - time.monotonic()
                if left <= 0 or (self.lines and self.lines[-1] is None):
                    return None
                self._cv.wait(left)

    def text(self, join_timeout=5.0):
        self._t.join(join_timeout)
        return "\n".join(ln for ln in self.lines if ln is not None)


def test_sigterm_leaves_a_headless_driver_disarmed(tmp_path):
    """A console killed politely (SIGTERM, Ctrl-C) leaves through `quit`:
    the robot is benched, not left standing with nobody driving."""
    import signal
    import subprocess
    from conftest import free_udp_ports
    port = free_udp_ports(2)
    twin = subprocess.Popen([sys.executable, os.path.join(ROOT, "link", "link_twin.py"),
                             "--port", str(port), "--tlm-port", str(port + 1),
                             "--duration", "15"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    tw = _Lines(twin)
    con = None
    try:
        assert tw.wait(lambda ln: "link_twin: listening" in ln, 5.0), tw.text(0)
        con = subprocess.Popen([sys.executable, os.path.join(ROOT, "link", "bimo_console.py"),
                                "--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                                str(port + 1), "--drive", "--headless", "--no-record",
                                "--sessions", str(tmp_path), "--keymap",
                                str(tmp_path / "keys.conf")],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
        co = _Lines(con)
        assert co.wait(lambda ln: "bimo_console --headless ->" in ln, 10.0), co.text(0)
        con.stdin.write("arm\n")
        con.stdin.flush()
        state = lambda ln: " vx " in ln and "torque" in ln      # noqa: E731
        assert tw.wait(lambda ln: state(ln) and " live " in ln, 5.0), tw.text(0)
        con.send_signal(signal.SIGTERM)
        con.wait(timeout=10)
        # The disarm is in flight when the console exits: let the twin say so.
        tw.wait(lambda ln: state(ln) and " bench " in ln and "torque off" in ln, 2.0)
    finally:
        if con is not None and con.poll() is None:
            con.kill()
        twin.terminate()
        twin.wait(timeout=5)
    out, log = co.text(), tw.text()
    assert con.returncode == 0, out
    assert "sent disarm" in out, out
    states = [ln for ln in log.splitlines() if state(ln)]
    assert any(" live " in ln for ln in states), log
    assert " bench " in states[-1] and "torque off" in states[-1], log


def test_headless_leaves_through_shutdown_even_on_an_exception(robot, tmp_path, monkeypatch):
    """Whatever ends the headless loop, the driver's disarm frames go out."""
    from console import headless
    calls = []

    class Boom(Controller):
        def tick(self, now=None):
            st = super().tick(now)
            if st.armed:
                raise RuntimeError("boom")
            return st

        def shutdown(self):
            calls.append("shutdown")
            super().shutdown()

    monkeypatch.setattr(headless, "Controller", Boom)
    monkeypatch.setattr(headless.signal, "signal", lambda *a: None)   # keep pytest's
    r, w = os.pipe()
    os.write(w, b"arm\n")
    monkeypatch.setattr(sys, "stdin", os.fdopen(r))
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(robot.getsockname()[1]),
                       "--tlm-port", "0", "--no-record", "--sessions", str(tmp_path),
                       "--keymap", str(tmp_path / "keys.conf"), "--drive", "--headless"])
    with pytest.raises(RuntimeError, match="boom"):
        headless.run(o)
    os.close(w)
    assert calls == ["shutdown"]
    got = frames(robot)
    assert len(got) >= 10 and all(not f.arm for f in got[-10:])


def test_leaving_never_crashes_on_a_handle_without_a_link(robot, tmp_path):
    """A never-linked core, a failed relink and a freed core (a window's
    close already ran shutdown) all leave quietly: no frame, no segfault at
    the ctypes boundary. A second shutdown() is a no-op."""
    bare = core.Core()                                   # never opened: no socket
    bare.leave_disarmed(3, 1)
    bare.close()
    bare.free()
    bare.leave_disarmed(3, 1)                            # freed: a no-op
    bare.close()
    with pytest.raises(RuntimeError, match="after free"):
        bare.tick(core.now_ms())                         # not a null handle into C

    o = options.parse(["--tlm-port", "0", "--no-record", "--sessions", str(tmp_path),
                       "--keymap", str(tmp_path / "keys.conf")])
    watcher = Controller(o)                              # no --host: an observer
    assert watcher.start()
    watcher.shutdown()
    watcher.shutdown()
    watcher.core.leave_disarmed(10, 1)

    c = make(robot.getsockname()[1], "--drive", tmp_path=tmp_path)
    c.tick()
    assert not c.relink("not-an-address", robot.getsockname()[1], 0)
    frames(robot)
    c.core.leave_disarmed(3, 1)
    c.shutdown()
    c.shutdown()
    c.core.leave_disarmed(3, 1)
    assert frames(robot, 0.1) == []
