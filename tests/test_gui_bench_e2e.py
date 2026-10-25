"""End-to-end: the real bimo_gui --headless against the real bridge and relay
code, all synthetic -- a pty for the board, an ffmpeg test pattern for the
camera. No serial port, no /dev/video*, no robot.

Run:  make -C firmware/host gui && .venv/bin/python -m pytest tests/test_gui_bench_e2e.py -q
"""
import itertools
import json
import os
import pty
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import tty

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "tests"))
import board_bridge as bb  # noqa: E402
import cam_relay as cr  # noqa: E402
from test_board_bridge import FakeBoard  # noqa: E402

GUI = os.path.join(ROOT, "firmware", "host", "build", "bimo_gui")
pytestmark = pytest.mark.skipif(not os.path.exists(GUI),
                                reason="build it: make -C firmware/host gui")
_ports = itertools.count(4812, 4)


def gui(tmp_path, *args, record=False):
    tlm = next(_ports)
    return subprocess.Popen(
        [GUI, "--headless", *([] if record else ["--no-record"]),
         "--sessions", str(tmp_path),
         "--tlm-port", str(tlm), *args],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1)


def wait_line(proc, pattern, timeout=15):
    """First stdout line matching pattern; the lines seen on the way."""
    rx, seen = re.compile(pattern), []
    end = time.time() + timeout
    import select
    while time.time() < end:
        r, _, _ = select.select([proc.stdout], [], [], 0.2)
        if not r:
            continue
        line = proc.stdout.readline()
        if not line:
            break
        seen.append(line)
        if rx.search(line):
            return line, seen
    raise AssertionError(f"no line matching {pattern!r}; saw:\n" + "".join(seen[-8:]))


def stop(proc):
    try:
        proc.stdin.close()
    except OSError:
        pass
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        proc.kill()


def test_bad_flags_are_refused(tmp_path):
    for args in (["--cam", "emeet"],                     # no relay to ask
                 ["--cam-relay", "127.0.0.1", "--cam", "webcam9"],
                 ["--cam-relay", "127.0.0.1", "--cam", "emeet", "--cam", "brio",
                  "--cam", "sony"]):
        p = subprocess.run([GUI, "--headless", *args], capture_output=True,
                           text=True, timeout=10)
        assert p.returncode == 2, args


def test_bench_only_console_is_an_observer_that_only_subscribes(tmp_path):
    """No --host: it must not transmit to anything, and the only thing it
    says to the bridge is subscribe / unsubscribe."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    srv.settimeout(0.3)
    p = gui(tmp_path, "--bench-telemetry", f"127.0.0.1:{srv.getsockname()[1]}")
    try:
        line, _ = wait_line(p, r"OBSERVER")
        got, client = [], None
        end = time.time() + 2.8
        while time.time() < end:
            try:
                d, client = srv.recvfrom(2000)
                got.append(json.loads(d))
            except socket.timeout:
                pass
        assert got and all(g == {"sub": 1} for g in got)
        assert 1 <= len(got) <= 3                        # ~every 2 s, not a flood
        srv.sendto(b'{"v":1,"k":"bal_roll","roll":-7.6}', client)
        wait_line(p, r"bench live joints 0 roll -7\.60")
    finally:
        stop(p)
    srv.settimeout(2)
    tail = []
    try:
        while True:
            tail.append(json.loads(srv.recvfrom(2000)[0]))
    except socket.timeout:
        pass
    assert {"unsub": 1} in tail                          # it says goodbye


def test_bridge_to_console_and_stale_when_it_dies(tmp_path):
    master, slave = pty.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    board = FakeBoard(master)
    board.start()
    pub = bb.Publisher("127.0.0.1", 0)
    poller = bb.Poller(gap_s=0.05, quiet_s=0.2, balance_s=0.3, imu_s=0.5,
                       joint_s=1.0)
    br = bb.Bridge(bb.FdPort(slave), open(tmp_path / "log", "a", buffering=1),
                   pub, poller)
    br.start()
    p = gui(tmp_path, "--bench-telemetry", f"127.0.0.1:{pub.port}")
    try:
        line, _ = wait_line(
            p, r"bench live joints 12 roll \+0\.37 err -0\.11 u -0\.06 bal ACTIVE up_z 0\.996 polling")
        assert "OBSERVER" in line
        assert all(c in bb.POLL_COMMANDS for c in board.rx)
        # the bridge dies: the console must SAY so
        br.stop.set()
        pub.close()
        wait_line(p, r"bench STALE", timeout=10)
    finally:
        stop(p)
        board.stop = True


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="no ffmpeg")
def test_camera_panel_live_then_stale_then_off(tmp_path):
    srcs = [cr.Source(cr.Spec("emeet=lavfi:testsrc,size=160x120,fps=15"), 5.0)]
    srv = cr.make_server(srcs, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    p = gui(tmp_path, "--cam-relay", f"127.0.0.1:{port}", "--cam", "emeet")
    try:
        line, _ = wait_line(p, r"cam0 emeet live frames [1-9]")
        assert srcs[0].status()["clients"] == 1          # selecting opened it
        # wedge the camera: frames stop, the pane must turn STALE, not lie
        os.kill(srcs[0].proc.pid, signal.SIGSTOP)
        wait_line(p, r"cam0 emeet STALE", timeout=12)
        os.kill(srcs[0].proc.pid, signal.SIGCONT)
        wait_line(p, r"cam0 emeet live", timeout=12)
        # the pane's combo box, scripted: off releases the camera
        p.stdin.write("cam 0 off\n")
        p.stdin.flush()
        deadline = time.time() + 5
        while srcs[0].status()["clients"] and time.time() < deadline:
            time.sleep(0.1)
        assert srcs[0].status()["clients"] == 0
    finally:
        stop(p)
        for s in srcs:
            s.stop()
        srv.shutdown()


def test_unreachable_relay_is_waiting_not_a_crash(tmp_path):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    dead = s.getsockname()[1]
    s.close()                                            # nothing listens here
    p = gui(tmp_path, "--cam-relay", f"127.0.0.1:{dead}", "--cam", "sony")
    try:
        wait_line(p, r"cam0 sony waiting frames 0")
        assert p.poll() is None
    finally:
        stop(p)


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="no ffmpeg")
def test_everything_shown_is_recorded_for_replay(tmp_path):
    """The session JSONL carries each bench datagram the panel displayed
    (verbatim, valid JSON, both clocks) and which camera each pane watched;
    replay_session.py still reads it (it keys on dir == cmd)."""
    master, slave = pty.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    board = FakeBoard(master)
    board.start()
    pub = bb.Publisher("127.0.0.1", 0)
    br = bb.Bridge(bb.FdPort(slave), open(tmp_path / "log", "a", buffering=1), pub,
                   bb.Poller(gap_s=0.05, quiet_s=0.2, balance_s=0.3, imu_s=0.5,
                             joint_s=1.0))
    br.start()
    srcs = [cr.Source(cr.Spec("emeet=lavfi:testsrc,size=160x120,fps=15"), 5.0)]
    srv = cr.make_server(srcs, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    sess = tmp_path / "sess"
    p = gui(sess, "--bench-telemetry", f"127.0.0.1:{pub.port}",
            "--cam-relay", f"127.0.0.1:{srv.server_address[1]}", "--cam", "emeet",
            record=True)
    try:
        wait_line(p, r"bench live joints 12 .*cam0 emeet live")
    finally:
        stop(p)
        br.stop.set(); board.stop = True; pub.close()
        for s_ in srcs:
            s_.stop()
        srv.shutdown()
    files = [f for f in os.listdir(sess) if f.endswith(".jsonl")]
    assert len(files) == 1
    rows = [json.loads(l) for l in open(sess / files[0]) if l.strip()]
    assert rows[0]["rec"] == "header"
    bench = [r for r in rows if r.get("dir") == "bench"]
    assert len(bench) >= 10
    assert all(r["d"]["v"] == 1 and r["utc"] > 1.7e9 and "t" in r for r in bench)
    assert {"joint", "bal_roll", "imu_up", "hb"} <= {r["d"]["k"] for r in bench}
    cams = [r for r in rows if r.get("dir") == "cam"]
    assert cams and cams[0]["src"] == "emeet" and cams[0]["url"].endswith("/emeet")
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import replay_session
    assert replay_session.load(str(sess / files[0]), 0, 1e9) == []   # no cmd rows: bench is read-only
