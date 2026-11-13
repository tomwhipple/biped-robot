"""tools/estop.py, end to end against a robot twin on localhost.

The twin is protocol.Supervisor -- the firmware's ArmLatch + Watchdog + mode
composition, which models the out-of-band operator stop (FLAG_STOP) -- behind
a UDP socket that beacons the way wifi_link.cpp does: to the last sender, at
its telemetry port. An "old firmware" twin strips FLAG_STOP before the
Supervisor sees the frame, as a robot that predates the bit would.
"""
import os
import socket
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import protocol as P  # noqa: E402

ESTOP = os.path.join(ROOT, "tools", "estop.py")


def free_port(kind=socket.SOCK_DGRAM) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class Twin(threading.Thread):
    def __init__(self, tlm_port: int, old_firmware=False, armed=True, silent=False):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.02)
        self.port = self.sock.getsockname()[1]
        self.tlm_port = tlm_port
        self.old = old_firmware
        self.silent = silent
        self.sup = P.Supervisor(armed=armed)
        self.t0 = time.monotonic()
        self.frames = 0
        self.flags_seen = set()
        self.stop_evt = threading.Event()
        self.peer = None
        self.next_tlm = 0.0

    def now_ms(self):
        return (time.monotonic() - self.t0) * 1000.0

    def run(self):
        while not self.stop_evt.is_set():
            try:
                data, src = self.sock.recvfrom(64)
            except socket.timeout:
                data = None
            if data:
                pkt = P.decode_command(data)
                self.frames += 1
                self.flags_seen.add(pkt.flags)
                if self.old:
                    pkt = pkt._replace(flags=pkt.flags & ~P.FLAG_STOP)
                self.sup.accept(pkt, self.now_ms())
                self.peer = src[0]
            now = time.monotonic()
            if self.peer and not self.silent and now >= self.next_tlm:
                self.next_tlm = now + 0.1
                t = P.Telemetry(seq_echo=self.sup.seq_echo(self.now_ms()),
                                state=self.sup.state(self.now_ms()), vbat_v=12.0,
                                up_z=0.99, vx_est=0.0, wz_est=0.0, servo_err=0,
                                loop_late_pct=0)
                self.sock.sendto(P.encode_telemetry(t), (self.peer, self.tlm_port))

    def close(self):
        self.stop_evt.set()
        self.join(timeout=2)
        self.sock.close()


def run_estop(twin_port, tlm_port, *extra, timeout=10):
    return subprocess.run([sys.executable, ESTOP, "--host", "127.0.0.1", "--port", str(twin_port),
                           "--tlm-port", str(tlm_port), "--no-broadcast", "--seconds", "0.3",
                           *extra], capture_output=True, text=True, timeout=timeout)


@pytest.fixture
def tlm_port():
    return free_port()


def test_hard_stop_releases_a_tethered_run(tlm_port):
    twin = Twin(tlm_port)
    twin.start()
    try:
        assert twin.sup.torque_on(0.0)
        r = run_estop(twin.port, tlm_port)
    finally:
        twin.close()
    assert r.returncode == 0, r.stdout + r.stderr
    assert "HARD STOP CONFIRMED" in r.stdout and "operator STOP" in r.stdout
    assert not twin.sup.torque_on(twin.now_ms())
    assert twin.sup.state(twin.now_ms()) is P.LinkState.BENCH
    # Every frame: the stop bit, the classic ESTOP for old firmware, ARM and
    # ENABLE clear.
    assert twin.flags_seen == {P.FLAG_STOP | P.FLAG_ESTOP}
    assert twin.frames >= 20                      # repeated, not a lone frame


def test_soft_stop_holds(tlm_port):
    twin = Twin(tlm_port)
    twin.start()
    try:
        r = run_estop(twin.port, tlm_port, "--soft")
    finally:
        twin.close()
    assert r.returncode == 0, r.stdout + r.stderr
    assert "SOFT STOP CONFIRMED" in r.stdout
    assert twin.sup.torque_on(twin.now_ms())     # held stiff, not limp
    assert twin.flags_seen == {P.FLAG_STOP}


def test_soft_stop_on_old_firmware_escalates_to_hard(tlm_port):
    # Old firmware ignores FLAG_STOP: a soft frame is a plain stand. The tool
    # hears no STOP verdict within --confirm-ms and goes hard; the classic
    # ESTOP latch then releases torque.
    twin = Twin(tlm_port, old_firmware=True)
    twin.start()
    try:
        r = run_estop(twin.port, tlm_port, "--soft", "--confirm-ms", "200")
    finally:
        twin.close()
    assert "escalating to HARD" in r.stdout, r.stdout
    assert r.returncode == 0, r.stdout + r.stderr
    assert twin.sup.state(twin.now_ms()) is P.LinkState.ESTOP
    assert not twin.sup.torque_on(twin.now_ms())


def test_hard_stop_on_old_firmware_still_latches_estop(tlm_port):
    twin = Twin(tlm_port, old_firmware=True)
    twin.start()
    try:
        r = run_estop(twin.port, tlm_port)
    finally:
        twin.close()
    assert r.returncode == 0, r.stdout + r.stderr
    assert twin.sup.state(twin.now_ms()) is P.LinkState.ESTOP


def test_no_beacon_says_so(tlm_port):
    twin = Twin(tlm_port, silent=True)
    twin.start()
    try:
        t0 = time.monotonic()
        r = run_estop(twin.port, tlm_port)
        took = time.monotonic() - t0
    finally:
        twin.close()
    assert r.returncode == 2
    assert "NO beacon heard" in r.stdout
    assert not twin.sup.torque_on(twin.now_ms())   # it still stopped
    assert took < 5.0


def test_the_fast_encoder_is_byte_exact_with_protocol():
    # estop.py encodes its frame itself so the first one leaves before
    # protocol.py is imported; it must be the reference's frame exactly.
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import estop as E
    assert (E.CMD_PORT, E.TLM_PORT) == (P.CMD_PORT, P.TLM_PORT)
    assert (E.FLAG_ESTOP, E.FLAG_STOP) == (P.FLAG_ESTOP, P.FLAG_STOP)
    for seq in (0, 1, 0x3FFFFFFF, 0xFFFFFFFF, 1793569163):
        for flags in (P.FLAG_STOP, P.FLAG_STOP | P.FLAG_ESTOP):
            assert E.encode_stop(seq, flags) == P.encode_command(seq, 0.0, 0.0, flags)
            pkt = P.decode_command(E.encode_stop(seq, flags))
            assert pkt.stop and not pkt.arm and not pkt.enabled and not pkt.home


def test_soft_stop_on_a_benched_robot_is_confirmed_without_escalating(tlm_port):
    """A benched robot's soft stop (firmware cli.cpp flailTickImpl) stops any
    bench motion and reports no new reason: BENCH, no reset in progress, is
    the confirmation -- not an escalation to torque off."""
    twin = Twin(tlm_port, armed=False)
    twin.start()
    try:
        r = run_estop(twin.port, tlm_port, "--soft", "--confirm-ms", "200")
    finally:
        twin.close()
    assert r.returncode == 0, r.stdout + r.stderr
    assert "SOFT STOP CONFIRMED" in r.stdout and "escalat" not in r.stdout, r.stdout
    assert twin.flags_seen == {P.FLAG_STOP}       # never a hard frame


def test_soft_stop_during_a_bench_reset_still_escalates(tlm_port):
    """HOME_PENDING is bench motion, and the firmware leaves it standing after
    a stop: nothing proves the joints stopped, so the soft stop goes hard."""
    twin = Twin(tlm_port, armed=False)
    twin.sup.result = P.ArmResult.HOME_PENDING
    twin.start()
    try:
        r = run_estop(twin.port, tlm_port, "--soft", "--confirm-ms", "200")
    finally:
        twin.close()
    assert "escalating to HARD" in r.stdout, r.stdout
    assert P.FLAG_STOP | P.FLAG_ESTOP in twin.flags_seen


def test_confirmed_rule():
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import estop
    bench = lambda r: P.Telemetry(seq_echo=P.pack_diag(False, True, r),  # noqa: E731
                                  state=P.LinkState.BENCH, vbat_v=12.0, up_z=1.0,
                                  vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
    none, stop = bench(P.ArmResult.NONE), bench(P.ArmResult.DISARMED_STOP)
    assert estop.confirmed(stop, False) and estop.confirmed(stop, True)
    assert not estop.confirmed(none, False)                 # a run's soft stop: no
    assert estop.confirmed(none, False, was_bench=True)     # already benched: yes
    assert not estop.confirmed(bench(P.ArmResult.HOME_PENDING), False, was_bench=True)
    live = none._replace(state=P.LinkState.LIVE, seq_echo=0)
    assert not estop.confirmed(live, False, was_bench=True)


def test_a_benched_stop_that_ends_a_reset_confirms_soft():
    # Issue #281: the robot reports DISARMED_STOP when a benched stop cuts a
    # servo reset short (linkproto::benchStopResult). That verdict confirms a
    # soft stop; the HOME_PENDING it replaces does not (bench motion still in
    # progress) and would escalate the HOLD to a hard stop.
    import importlib.util
    from types import SimpleNamespace
    spec = importlib.util.spec_from_file_location("estop_under_test", ESTOP)
    estop = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(estop)

    def bench(r):
        return SimpleNamespace(state=P.LinkState.BENCH, diag=P.pack_diag(False, True, r))

    assert estop.confirmed(bench(P.ArmResult.DISARMED_STOP), hard=False)
    assert not estop.confirmed(bench(P.ArmResult.HOME_PENDING), hard=False)
    assert not P.is_home_result(P.ArmResult.DISARMED_STOP)
