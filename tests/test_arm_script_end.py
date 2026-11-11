"""arm_script --phase script ends with the RESET SERVOS edge, not a bare disarm.

Run:  .venv/bin/python -m pytest tests/test_arm_script_end.py -q

2026-11-10 (#276): a stride run ended with ARM-off frames, which releases
torque where the robot stands; it toppled 1.2 s later. Every armed run now ends
with ARM|FLAG_HOME and waits for the robot's read-back. The robot here is a
fake on loopback that records every command frame and answers with the home
verdicts the firmware gives (HOME_PENDING, then DISARMED_HOME or
HOME_NOT_REACHED), or says nothing at all. The link twin is exercised through
the real CLI.
"""
import argparse
import os
import socket
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import arm_script                                                # noqa: E402
from protocol import (FLAG_ARM, FLAG_ENABLE, FLAG_ESTOP, FLAG_HOME,  # noqa: E402
                      ArmResult, LinkState, Telemetry, decode_command,
                      encode_telemetry, pack_diag)


class FakeRobot(threading.Thread):
    """Loopback robot. `home` is the verdict after a FLAG_HOME edge:
    "ok", "not_reached", "pending" (never finishes) or "silent" (never
    beacons at all)."""

    def __init__(self, cmd_port, tlm_port, home="ok", home_after=0.5):
        super().__init__(daemon=True)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", cmd_port))
        self.rx.settimeout(0.02)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_port, self.home, self.home_after = tlm_port, home, home_after
        self.frames = []            # (t, flags) of every command received
        self.stop = False

    def run(self):
        armed, homed_at, prev_home = False, None, False
        peer, last_tx = None, 0.0
        while not self.stop:
            try:
                buf, peer = self.rx.recvfrom(64)
                pkt = decode_command(buf)
                self.frames.append((time.monotonic(), pkt.flags))
                armed = bool(pkt.flags & FLAG_ARM) and not (pkt.flags & FLAG_ESTOP)
                edge = bool(pkt.flags & FLAG_HOME)
                if edge and not prev_home:
                    homed_at, armed = time.monotonic(), False
                prev_home = edge
            except socket.timeout:
                pass
            if self.home == "silent" or peer is None:
                continue
            now = time.monotonic()
            if now - last_tx < 0.05:
                continue
            last_tx = now
            if armed:
                state, diag = LinkState.STAND, 0
            else:
                state = LinkState.BENCH
                if homed_at is None:
                    r = ArmResult.ACCEPTED
                elif self.home == "pending" or now - homed_at < self.home_after:
                    r = ArmResult.HOME_PENDING
                elif self.home == "not_reached":
                    r = ArmResult.HOME_NOT_REACHED
                else:
                    r = ArmResult.DISARMED_HOME
                diag = pack_diag(False, True, r)
            self.tx.sendto(encode_telemetry(Telemetry(
                seq_echo=diag if state is LinkState.BENCH else 1, state=state,
                vbat_v=11.4, up_z=1.0, vx_est=0.0, wz_est=0.0, servo_err=0,
                loop_late_pct=0)), (peer[0], self.tlm_port))

    def runs(self):
        """Frame flags, run-length compressed, in order."""
        out = []
        for _, f in self.frames:
            if not out or out[-1] != f:
                out.append(f)
        return out

    def close(self):
        self.stop = True
        self.join(1)
        self.rx.close()
        self.tx.close()


def _free_ports():
    s1, s2 = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    s1.bind(("127.0.0.1", 0))
    s2.bind(("127.0.0.1", 0))
    p = s1.getsockname()[1], s2.getsockname()[1]
    s1.close()
    s2.close()
    return p


def run_script(home, home_wait=3, stay_armed=False, home_after=0.5):
    cmd, tlm = _free_ports()
    robot = FakeRobot(cmd, tlm, home=home, home_after=home_after)
    robot.start()
    d = arm_script.Driver("127.0.0.1", cmd_port=cmd, tlm_port=tlm)
    a = argparse.Namespace(script="t_walk", settle=0.3, duration=0.4, tail=0.2,
                           stay_armed=stay_armed, home_wait=home_wait)
    try:
        verdict = arm_script.phase_script(d, a)
        time.sleep(0.1)
    finally:
        d.close()
        robot.close()
    return verdict, robot


arm_script.SCRIPTS.setdefault("t_walk", lambda t: (0.3, 0.0))
ARM, ENA = FLAG_ARM, FLAG_ARM | FLAG_ENABLE
EDGE = FLAG_ARM | FLAG_HOME


def test_script_ends_with_home_edge_and_waits_for_readback():
    verdict, robot = run_script("ok")
    assert verdict is None
    runs = robot.runs()
    # preamble, settle, script (enable while moving), tail, then the edge:
    assert runs[0] == 0 and runs[1] == ARM
    assert ENA in runs
    i = runs.index(EDGE)
    assert runs[i - 1] in (ARM, ENA) and i > runs.index(ENA)
    # never a bare disarm between the last armed stand frame and the edge,
    # and no ESTOP anywhere: the clean read-back releases torque itself
    assert not any(f & FLAG_ESTOP for f in runs)
    # after the edge the run stays quiet-but-streaming (flags 0) until the
    # DISARMED_HOME verdict, rather than going silent at the edge
    t_edge = next(t for t, f in robot.frames if f == EDGE)
    after = [(t, f) for t, f in robot.frames if t > t_edge and f == 0]
    assert after and after[-1][0] - t_edge >= 0.5   # waited out HOME_PENDING


def test_home_not_reached_sends_estop_release_and_is_a_failure():
    verdict, robot = run_script("not_reached")
    assert verdict and verdict.startswith(arm_script.HOME_FAILED)
    assert "HOME_NOT_REACHED" in verdict
    runs = robot.runs()
    assert EDGE in runs
    assert (FLAG_ARM | FLAG_ESTOP) in runs[runs.index(EDGE):]


def test_home_never_finishes_times_out_into_estop():
    verdict, robot = run_script("pending", home_wait=2)
    assert verdict and verdict.startswith(arm_script.HOME_FAILED)
    assert "HOME_PENDING" in verdict
    assert (FLAG_ARM | FLAG_ESTOP) in robot.runs()


def test_no_telemetry_is_not_a_clean_end():
    verdict, robot = run_script("silent", home_wait=2)
    assert verdict and verdict.startswith(arm_script.HOME_FAILED)
    assert "None" in verdict
    runs = robot.runs()
    assert EDGE in runs and (FLAG_ARM | FLAG_ESTOP) in runs


def test_stay_armed_sends_no_home_edge():
    verdict, robot = run_script("ok", stay_armed=True)
    assert verdict is None
    assert all(not (f & (FLAG_HOME | FLAG_ESTOP)) for f in robot.runs())
    assert robot.runs()[-1] in (ARM, ENA)


def test_cli_against_link_twin(tmp_path):
    """The real CLI against link_twin.py: ends homed, exit 0, and the twin saw
    the home before any bare disarm released the loop."""
    cmd, tlm = _free_ports()
    twin = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "link", "link_twin.py"),
         "--port", str(cmd), "--tlm-port", str(tlm), "--duration", "20"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(0.5)
        r = subprocess.run(
            [sys.executable, os.path.join(ROOT, "link", "arm_script.py"),
             "--host", "127.0.0.1", "--port", str(cmd), "--tlm-port", str(tlm),
             "--phase", "script", "--script", "stride1", "--settle", "0.5",
             "--duration", "0.5", "--tail", "0.3"],
            capture_output=True, text=True, timeout=40)
    finally:
        twin.terminate()
        out = twin.communicate(timeout=5)[0]
    assert r.returncode == 0, r.stdout + r.stderr
    assert "verdict: ArmResult.DISARMED_HOME" in r.stdout
    assert "home: benched" in out
    assert "ESTOP" not in r.stdout.split("verdict")[1]
