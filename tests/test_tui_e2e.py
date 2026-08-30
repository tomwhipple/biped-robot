"""End-to-end: the real bimo_tui binary, driven through a pty, against the
plant-free link twin over real UDP.

Run:  make -C firmware/host tui && .venv/bin/python -m pytest tests/test_tui_e2e.py -q

Skipped when the binary has not been built. Ports are offset so it never
collides with a console pointed at the robot.
"""
import os
import pty
import re
import select
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TUI = os.path.join(ROOT, "firmware", "host", "build", "bimo_tui")
TWIN = os.path.join(ROOT, "link", "link_twin.py")
CMD_PORT, TLM_PORT = 4310, 4311

KEY_UP, KEY_LEFT = b"\x1b[A", b"\x1b[D"

pytestmark = pytest.mark.skipif(not os.path.exists(TUI),
                                reason="build it: make -C firmware/host tui")


class Screen:
    """Owns the pty master: drains it (ncurses blocks if nobody reads) and
    keeps the raw stream for assertions."""

    def __init__(self, fd):
        self.fd, self.buf, self._stop = fd, bytearray(), False
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()

    def _pump(self):
        while not self._stop:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                try:
                    self.buf += os.read(self.fd, 65536)
                except OSError:
                    return

    def text(self):
        # Strip escape sequences; what remains is the words that were drawn.
        return re.sub(rb"\x1b\[[0-9;?]*[A-Za-z]|\x1b[()][A-Z0-9]|\x1b[=>]",
                      b" ", bytes(self.buf)).decode("utf-8", "replace")

    def close(self):
        self._stop = True
        self._t.join(timeout=1.0)


def _twin(extra=()):
    return subprocess.Popen(
        [sys.executable, TWIN, "--port", str(CMD_PORT), "--tlm-port",
         str(TLM_PORT), "--duration", "20", *extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def _tui(hold_ms=300):
    master, slave = pty.openpty()
    env = dict(os.environ, TERM="xterm", LINES="30", COLUMNS="100")
    proc = subprocess.Popen(
        [TUI, "--host", "127.0.0.1", "--cmd-port", str(CMD_PORT),
         "--tlm-port", str(TLM_PORT), "--hold-ms", str(hold_ms)],
        stdin=slave, stdout=slave, stderr=slave, env=env, close_fds=True)
    os.close(slave)
    return proc, master


def _hold(master, key, seconds, every=0.05):
    """A held key on a terminal is auto-repeat: the same sequence over and
    over until release, when it simply stops."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        os.write(master, key)
        time.sleep(every)


def _states(log):
    return re.findall(r"\]\s+(\w+)\s+vx ([+-]\d\.\d\d) wz ([+-]\d\.\d\d) "
                      r"torque (on|off)", log)


def test_arm_hold_release_estop_quit():
    twin = _twin()
    time.sleep(0.5)
    tui, master = _tui()
    screen = Screen(master)
    try:
        time.sleep(1.0)                       # ARM=0 frames: still BENCH
        os.write(master, b"a")                # 0 -> 1: arm
        time.sleep(1.0)
        _hold(master, KEY_UP, 1.0)            # walk forward while "held"
        time.sleep(0.8)                       # release: hold expires -> stand
        _hold(master, KEY_LEFT, 0.6)
        time.sleep(0.8)
        os.write(master, b"e")                # E-stop
        time.sleep(0.5)
        os.write(master, b" ")                # stand clears it
        time.sleep(0.5)
        os.write(master, b"q")                # quit disarms
        tui.wait(timeout=5)
    finally:
        screen.close()
        os.close(master)
        twin.terminate()
        log = twin.communicate(timeout=5)[0]
        if tui.poll() is None:
            tui.kill()

    seq = _states(log)
    kinds = [s[0] for s in seq]
    assert kinds[0] == "bench", log
    # STAND is the stale-link state; a console streaming at 20 Hz never lets
    # the link go stale, so a standing robot is LIVE with a zero command.
    assert "stand" not in kinds and "live" in kinds, log
    # Walking forward at the console's default 0.4 m/s, then a left turn at
    # +0.5 rad/s, each followed by a stand when the key "released".
    assert ("live", "+0.40", "+0.00", "on") in seq, log
    i = seq.index(("live", "+0.40", "+0.00", "on"))
    assert ("live", "+0.00", "+0.00", "on") in seq[i:], log
    assert ("live", "+0.00", "+0.50", "on") in seq, log
    assert any(s[0] == "estop" and s[3] == "off" for s in seq), log
    j = max(k for k, s in enumerate(seq) if s[0] == "estop")
    assert any(s[0] == "live" for s in seq[j:]), log       # stand cleared it
    assert kinds[-1] == "bench" and seq[-1][3] == "off", log   # quit disarmed
    assert tui.returncode == 0

    shown = screen.text()
    assert "ARMED" in shown and "LIVE" in shown and "FORWARD" in shown, shown
    assert "11.40 V" in shown, shown                         # telemetry drawn


def test_console_follows_a_robot_that_refuses_to_arm():
    twin = _twin(["--no-cal"])
    time.sleep(0.5)
    tui, master = _tui()
    screen = Screen(master)
    try:
        time.sleep(0.8)
        os.write(master, b"a")
        time.sleep(2.5)                       # > kArmSyncMs: console gives up
        _hold(master, KEY_UP, 0.5)            # must be refused client-side
        time.sleep(0.5)
        os.write(master, b"q")
        tui.wait(timeout=5)
    finally:
        screen.close()
        os.close(master)
        twin.terminate()
        log = twin.communicate(timeout=5)[0]
        if tui.poll() is None:
            tui.kill()

    assert all(s[0] == "bench" for s in _states(log)), log
    shown = screen.text()
    assert "stayed BENCH" in shown, shown
    assert "not armed -- press [a] first" in shown, shown
    # The incident of 2026-08-30: the robot refused the arm and said nothing
    # over the radio, so diagnosing it needed the serial tether. The console
    # must now name the gate that refused, without guessing.
    assert "no as-built calibration in NVS" in shown, shown
    # ... and it must say so BEFORE the arm was even attempted, from the very
    # first beacon -- an uncalibrated robot is diagnosable on sight.
    assert "arming will be REFUSED" in shown, shown
