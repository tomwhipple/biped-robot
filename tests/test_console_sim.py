"""The Qt console's sim (link/console/simproc.py, tab_sim.py): run discovery
as link/simproc.cpp's RunList, the sil_twin command line per mode, the child's
lifecycle against a fake sil_twin (a stdlib script in a temp repo), the
headless `sim` verbs, twin-in-the-loop's tee at the socket, and the tab built
offscreen.

Run:  make -C firmware/host console && .venv/bin/python -m pytest tests/test_console_sim.py -q
(the tab test needs PyQt6 + pyqtgraph and skips without them).
"""
import itertools
import json
import os
import socket
import subprocess
import sys
import textwrap
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from console import options, simproc  # noqa: E402

V2 = os.path.join(ROOT, "link", "bimo_console.py")
LIB = os.path.join(ROOT, "firmware", "host", "build", "libbimo_console."
                   + ("dylib" if sys.platform == "darwin" else "so"))
needs_lib = pytest.mark.skipif(not os.path.exists(LIB),
                               reason="build it: make -C firmware/host console")
_ports = itertools.count(4700 + (os.getpid() % 50) * 20, 4)

# A stand-in for sim/sil_twin.py: argparse (with --flight only when a FLIGHT
# file sits beside it), sil_twin's ident line, the `reset` verb, an
# unflushed print (PYTHONUNBUFFERED must carry it), a line on stderr, and
# hooks to die or to ignore SIGTERM. It keeps running after stdin closes, as
# sil_twin does.
FAKE = textwrap.dedent("""\
    import argparse, os, signal, sys, time
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    for a in ("--run-name", "--xml"):
        p.add_argument(a)
    for a in ("--port", "--tlm-port", "--stream-port"):
        p.add_argument(a, type=int)
    p.add_argument("--hang", action="store_true")
    if os.path.exists(os.path.join(here, "FLIGHT")):
        p.add_argument("--flight", help="HOST:PORT flight records")
    a = p.parse_args()
    with open(os.path.join(os.getcwd(), "fake.pid"), "w") as f:
        f.write(str(os.getpid()))
    with open(os.path.join(os.getcwd(), "fake.argv"), "w") as f:
        f.write(" ".join(sys.argv[1:]))
    print(f"sil_twin: {a.run_name} on :{a.port} (tlm -> :{a.tlm_port})"
          f"  plant={a.xml or 'run default'}  spec=deadbeef...")
    print("fake: a line on stderr", file=sys.stderr)
    for line in sys.stdin:
        cmd = line.strip()
        if cmd == "reset":
            print("  [  1.00s] episode reset")
        elif cmd.startswith("die "):
            print("fake: dying on request")
            sys.exit(int(cmd.split()[1]))
        elif cmd.startswith("long "):
            print("E" * int(cmd.split()[1]) + " -> -1")
            sys.exit(1)
        elif cmd == "ignore-term":
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            print("fake: ignoring SIGTERM")
        else:
            print(f"  unknown command: {cmd}")
    while True:
        time.sleep(1)
""")


def fake_repo(root, runs=("loco_a",), flight=False):
    repo = root / "repo"
    (repo / ".venv" / "bin").mkdir(parents=True)
    os.symlink(sys.executable, repo / ".venv" / "bin" / "python")
    (repo / "sim" / "sil" / "weights").mkdir(parents=True)
    (repo / "sim" / "sil_twin.py").write_text(FAKE)
    (repo / "sim" / "bimo_biped_v6ar.xml").write_text("<mujoco/>")
    if flight:
        (repo / "sim" / "FLIGHT").write_text("")
    for r in runs:
        add_run(repo, r)
    return repo


def add_run(repo, name, config=True, weights=True):
    d = repo / "sim" / "runs" / name
    d.mkdir(parents=True, exist_ok=True)
    if config:
        (d / "config.json").write_text(json.dumps({"xml_path": "bimo_biped_v5body.xml"}))
    if weights:
        (repo / "sim" / "sil" / "weights" / f"{name}.silw.json").write_text("{}")


def wait_for(pred, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def texts(proc):
    return [t for _, t in proc.lines()]


class FakeCore:
    def __init__(self):
        self.tee = None
        self.stood = []

    def set_tee(self, host, port):
        self.tee = (host, port)
        return True

    def clear_tee(self):
        self.tee = None

    def stand(self, why):
        self.stood.append(why)


class FakeCtl:
    """What SimControl needs of a Controller."""

    def __init__(self, argv, readonly=False):
        self.o = options.parse(argv)
        self.host = self.o.host
        self.core = FakeCore()
        self.readonly = readonly
        self.status_msg = ""
        self.rows = []

    def log(self, kind, msg):
        self.rows.append((kind, msg))

    def say_status(self, msg):
        self.status_msg = msg


# -- run discovery -----------------------------------------------------------
def test_runs_need_a_config_and_exported_weights(tmp_path):
    repo = fake_repo(tmp_path, runs=("zeta", "alpha", ".hidden"))
    add_run(repo, "config_only", weights=False)
    add_run(repo, "weights_only", config=False)
    (repo / "sim" / "runs" / "stray.log").write_text("")
    assert simproc.scan_runs(str(repo)) == ["alpha", "zeta"]
    assert simproc.scan_runs(str(tmp_path / "nowhere")) == []
    assert simproc.run_plant(str(repo), "alpha") == "bimo_biped_v5body.xml"
    assert simproc.plants(str(repo)) == ["bimo_biped_v6ar.xml"]


def test_run_list_is_capped_like_the_cpp(tmp_path):
    repo = fake_repo(tmp_path, runs=[f"r{i:03d}" for i in range(simproc.MAX_RUNS + 5)])
    runs = simproc.scan_runs(str(repo))
    assert len(runs) == simproc.MAX_RUNS and runs == sorted(runs)


# -- the command line, per mode ------------------------------------------------
def test_command_line_is_simprocs(tmp_path):
    r = str(tmp_path)
    assert simproc.command_line(r, "loco", False, 4210, 4211, 8645) == [
        f"{r}/.venv/bin/python", f"{r}/sim/sil_twin.py", "--run-name", "loco",
        "--stream-port", "8645", "--port", "4210", "--tlm-port", "4211"]
    argv = simproc.command_line(r, "loco", True, 4310, 4311, 0, ("127.0.0.1", 4222), "/x.xml")
    assert argv[9:] == ["4311", "--hang", "--xml", "/x.xml", "--flight", "127.0.0.1:4222"]


@pytest.mark.parametrize("flight", [False, True])
def test_ports_follow_the_target_and_the_twin(tmp_path, flight):
    repo = fake_repo(tmp_path, flight=flight)
    base = ["--repo", str(repo), "--cmd-port", "4210", "--tlm-port", "4211"]
    # The console drives the sim: it listens where the console points.
    sim = simproc.SimControl(FakeCtl(base + ["--host", "127.0.0.1"]))
    p = sim.plan()
    assert (p.cmd_port, p.tlm_port, p.own) == (4210, 4211, False)
    # ... and stands in for the robot on the console's flight port.
    assert p.flight == (("127.0.0.1", 4221) if flight else None)
    # Twin-in-the-loop: its own ports (--sim-cmd-port default cmd+100, +1).
    sim.twin = True
    p = sim.plan()
    assert (p.cmd_port, p.tlm_port, p.own) == (4310, 4311, True)
    assert p.flight == (("127.0.0.1", 4222) if flight else None)
    # Pointed at a robot, twin off: still its own ports (never the robot's).
    sim = simproc.SimControl(FakeCtl(base + ["--host", "192.168.2.50", "--sim-cmd-port", "5000"]))
    p = sim.plan()
    assert (p.cmd_port, p.tlm_port, p.own) == (5000, 5001, True)
    assert simproc.supports_flight(str(repo)) is flight


def test_start_passes_hang_plant_and_flight(tmp_path):
    repo = fake_repo(tmp_path, flight=True)
    ctl = FakeCtl(["--repo", str(repo), "--host", "127.0.0.1", "--cmd-port", "4210",
                   "--tlm-port", "4211", "--stream-port", "0"])
    sim = simproc.SimControl(ctl)
    sim.set_hang(True)
    sim.plant = "bimo_biped_v6ar.xml"
    try:
        assert sim.start(), sim.status
        argv_file = repo / "sim" / "fake.argv"
        assert wait_for(argv_file.exists)
        assert wait_for(lambda: argv_file.read_text().strip() != "")
        argv = argv_file.read_text().split()
        assert argv[argv.index("--run-name") + 1] == "loco_a"
        assert argv[argv.index("--port") + 1] == "4210" and "--hang" in argv
        assert argv[argv.index("--xml") + 1] == str(repo / "sim" / "bimo_biped_v6ar.xml")
        assert argv[argv.index("--flight") + 1] == "127.0.0.1:4221"
        assert "starting" in sim.status and "4210" in sim.status
    finally:
        sim.stop()


# -- the child -----------------------------------------------------------------
def test_child_start_tell_stop_and_its_output(tmp_path):
    repo = fake_repo(tmp_path)
    p = simproc.SimProc(str(repo))
    assert p.start("loco_a", False, 5001, 5002, 0)
    try:
        assert p.running()
        assert wait_for(lambda: p.ident), texts(p)
        assert p.ident == "loco_a on :5001 (tlm -> :5002)  plant=run default"
        assert wait_for(lambda: "fake: a line on stderr" in texts(p)), texts(p)
        assert texts(p)[0].startswith("$ .venv/bin/python sim/sil_twin.py --run-name loco_a")
        assert p.tell("reset")
        assert wait_for(lambda: any("episode reset" in t for t in texts(p))), texts(p)
        cursor = p.total
        assert p.tell("hello")
        assert wait_for(lambda: p.lines(cursor)), texts(p)
        assert [t for _, t in p.lines(cursor)] == ["  unknown command: hello"]
        pid = p.pid
    finally:
        assert p.stop()
    assert not p.running() and p.err == "" and texts(p)[-1] == "-- sim stopped"
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
    assert not p.tell("reset") and not p.stop()


def test_a_child_that_ignores_sigterm_is_killed(tmp_path, monkeypatch):
    monkeypatch.setattr(simproc, "STOP_WAIT_S", 0.3)
    p = simproc.SimProc(str(fake_repo(tmp_path)))
    assert p.start("loco_a", False, 5001, 5002, 0)
    assert p.tell("ignore-term")
    assert wait_for(lambda: "fake: ignoring SIGTERM" in texts(p))
    pid = p.pid
    t0 = time.monotonic()
    assert p.stop()
    assert time.monotonic() - t0 < 3.0
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_a_dead_child_is_visibly_dead_with_its_last_words(tmp_path):
    repo = fake_repo(tmp_path)
    ctl = FakeCtl(["--repo", str(repo), "--host", "10.0.0.9", "--stream-port", "0"])
    sim = simproc.SimControl(ctl)
    sim.set_twin(True)
    assert sim.start()
    assert ctl.core.tee == ("127.0.0.1", ctl.o.sim_cmd_port)     # twin: teed to its ports
    assert wait_for(lambda: sim.proc.ident)
    assert sim.proc.tell("die 3")
    assert wait_for(lambda: (sim.poll(), not sim.proc.running() and sim.proc.err)[1])
    assert sim.proc.err == "sim exited (status 3)"
    assert sim.proc.last_words(1) == ["fake: dying on request"]
    assert "sim exited (status 3): fake: dying on request" in ctl.status_msg
    assert ("SIM", "sim exited (status 3)") in ctl.rows
    assert ("sim", "-- sim exited (status 3)") in ctl.rows      # the child's lines, logged
    assert ctl.core.tee is None                                   # nothing left to tee
    assert not sim.reset()


def test_a_long_last_line_is_kept_whole_for_the_verdict(tmp_path):
    p = simproc.SimProc(str(fake_repo(tmp_path)))
    assert p.start("loco_a", False, 5001, 5002, 0)
    assert p.tell("long 450")
    assert wait_for(lambda: p.poll() != "" or p.proc is None)
    assert p.last_words(1) == ["E" * 450 + " -> -1"]
    chunks = [t for t in texts(p) if t.startswith("E") or t.endswith("-> -1")]
    assert len(chunks) == 3 and all(len(t) < simproc.LOG_COLS for t in chunks)
    assert "".join(chunks) == "E" * 450 + " -> -1"


def test_missing_interpreter_is_the_reason(tmp_path):
    repo = fake_repo(tmp_path)
    os.unlink(repo / ".venv" / "bin" / "python")
    ctl = FakeCtl(["--repo", str(repo), "--host", "127.0.0.1"])
    sim = simproc.SimControl(ctl)
    assert not sim.start()
    assert sim.status == f"no interpreter at {repo}/.venv/bin/python"


def test_twin_refuses_to_tee_the_consoles_own_target(tmp_path):
    repo = fake_repo(tmp_path)
    ctl = FakeCtl(["--repo", str(repo), "--host", "127.0.0.1", "--stream-port", "0"])
    sim = simproc.SimControl(ctl)
    try:
        assert sim.start()                  # the sim IS the target: the console's ports
        sim.set_twin(True)
        assert ctl.core.tee is None         # a second copy would be a second command
        assert "own target" in sim.twin_text()
        ctl.readonly = True
        assert "OBSERVER" in sim.twin_text() and "nothing to tee" in sim.twin_text()
    finally:
        sim.stop()


# -- headless ------------------------------------------------------------------
def _headless(script, *args):
    port = next(_ports)
    proc = subprocess.Popen(
        [sys.executable, V2, "--host", "127.0.0.1", "--cmd-port", str(port),
         "--tlm-port", str(port + 1), "--headless", "--no-record", *args],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    for delay, line in script:
        time.sleep(delay)
        if proc.poll() is not None:
            break
        proc.stdin.write(line + "\n")
        proc.stdin.flush()
    try:
        out = proc.communicate(timeout=15)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        out = proc.communicate()[0]
    return out, proc.returncode


@needs_lib
def test_headless_sim_start_on_an_empty_repo_says_so(tmp_path):
    out, rc = _headless([(0.8, "sim start"), (0.3, "sim reset"), (0.3, "sim hang"),
                         (0.3, "sim nonsense"), (0.3, "quit")],
                        "--repo", str(tmp_path), "--sessions", str(tmp_path / "s"))
    assert (f"note: no runnable sim runs under {tmp_path}/sim/runs (each needs a "
            "config.json AND sim/sil/weights/<run>.silw.json)") in out, out
    assert "note: no sim listening" in out and "note: hang on" in out, out
    assert "note: sim start [run]|stop|reset|hang|twin [off]|mirror [off]" in out, out
    assert rc == 0, out


@needs_lib
def test_headless_runs_the_sim_and_quitting_stops_it(tmp_path):
    repo = fake_repo(tmp_path, runs=("loco_a", "loco_b"))
    out, rc = _headless([(0.8, "sim start nope"), (0.2, "sim start loco_b"), (1.5, "sim reset"),
                         (0.8, "")],                # then EOF: quit, which stops the sim
                        "--repo", str(repo), "--sessions", str(tmp_path / "s"),
                        "--stream-port", "0")
    assert "note: no run `nope`" in out, out
    assert "note: sim loco_b starting on the console's own ports" in out, out
    assert "note: sim reset sent" in out, out
    assert "episode reset" in out, out                 # the child's own line, in the event log
    assert "loco_b on :" in out, out
    pid = int((repo / "sim" / "fake.pid").read_text())
    assert wait_for(lambda: not _alive(pid), 3.0), f"sil_twin {pid} outlived the console"
    assert "-- sim stopped" in out, out
    assert rc == 0, out


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


# -- twin-in-the-loop at the socket -----------------------------------------
def _udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s


def _drain(s):
    out = []
    while True:
        try:
            out.append(s.recv(256))
        except BlockingIOError:
            return out


@needs_lib
def test_twin_in_the_loop_tees_every_frame_byte_for_byte(tmp_path):
    from console.controller import Controller
    repo = fake_repo(tmp_path)
    port = next(_ports)
    robot, twin = _udp(port), _udp(port + 2)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                       str(port + 1), "--sim-cmd-port", str(port + 2), "--drive",
                       "--no-record", "--sessions", str(tmp_path / "s"), "--repo", str(repo),
                       "--keymap", str(tmp_path / "keys.conf"), "--stream-port", "0"])
    c = Controller(o)
    assert c.start()
    try:
        for _ in range(3):
            c.tick()
        assert not c.st.tee_on and c.st.teed == 0          # off by default
        c.sim.set_twin(True)
        assert c.sim.start()
        assert c.sim.active.own and c.sim.active.cmd_port == port + 2
        _drain(robot)
        for _ in range(10):
            c.tick()
            time.sleep(0.01)
        assert c.st.tee_on and c.st.teed >= 10
        sent, teed = _drain(robot), _drain(twin)
        assert sent and teed == sent                       # a byte-for-byte copy of each
        c.sim.stop()
        c.tick()
        assert not c.st.tee_on
        _drain(twin)
        for _ in range(3):
            c.tick()
        assert _drain(twin) == [] and _drain(robot)        # the robot still gets its frames
    finally:
        c.shutdown()
        robot.close()
        twin.close()


@needs_lib
def test_an_observer_tees_nothing(tmp_path):
    from console.controller import Controller
    repo = fake_repo(tmp_path)
    port = next(_ports)
    twin = _udp(port + 2)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                       str(port + 1), "--sim-cmd-port", str(port + 2), "--no-record",
                       "--sessions", str(tmp_path / "s"), "--repo", str(repo),
                       "--keymap", str(tmp_path / "keys.conf"), "--stream-port", "0"])
    c = Controller(o)
    assert c.start()
    try:
        c.tick()
        c.sim.set_twin(True)
        assert c.sim.start()
        for _ in range(5):
            c.tick()
        assert c.st.readonly and c.st.teed == 0 and _drain(twin) == []
        assert "nothing to tee" in c.sim.twin_text()
    finally:
        c.shutdown()
        twin.close()


# -- the tab, offscreen ----------------------------------------------------------
@needs_lib
def test_sim_tab_builds_runs_and_opens_its_log_on_failure(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt6")
    pytest.importorskip("pyqtgraph")
    from PyQt6 import QtWidgets

    from console.app import Window
    from console.controller import Controller
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    repo = fake_repo(tmp_path, runs=("loco_a", "loco_b"))
    port = next(_ports)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                       str(port + 1), "--no-record", "--sessions", str(tmp_path / "s"),
                       "--repo", str(repo), "--keymap", str(tmp_path / "keys.conf"),
                       "--tab", "sim"])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    w.show()

    def pump(s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)

    try:
        tab = w.tab_widgets["Sim"]
        pump(0.3)
        assert [tab.run_box.itemText(i) for i in range(tab.run_box.count())] == ["loco_a", "loco_b"]
        assert tab.state.text() == "STOPPED" and not tab.stop_btn.isEnabled()
        assert not tab.twin.isChecked()                    # off by default
        tab.run_box.setCurrentText("loco_b")
        tab._show_log(False)
        tab.start_btn.click()
        assert wait_for(lambda: (pump(0.05), tab.state.text() == "RUNNING")[1])
        assert "loco_b on :" in tab.ident.text()
        assert tab.stop_btn.isEnabled() and not tab.run_box.isEnabled()
        assert "cmd {} · tlm {}".format(port, port + 1) in tab.ports.text()
        assert "the console's own ports" in tab.ports_why.text()
        assert "$ .venv/bin/python sim/sil_twin.py" in tab.log.toPlainText()
        tab.reset_btn.click()                              # the console's target: it stands
        assert wait_for(lambda: (pump(0.05), "episode reset" in tab.log.toPlainText())[1])
        assert ctl.status_msg == "sim reset"
        tab._show_log(False)
        ctl.sim.proc.tell("die 2")
        assert wait_for(lambda: (pump(0.05), tab.state.text() == "FAILED")[1])
        assert tab.log.isVisible()                         # opened itself
        assert "sim exited (status 2)" in tab.fail.text()
        assert "fake: dying on request" in tab.log.toPlainText()
        tab.twin.setChecked(True)
        pump(0.2)
        assert ctl.sim.twin and "OBSERVER" in tab.twin_state.text()
        assert "{}".format(o.sim_cmd_port) in tab.ports.text()
    finally:
        w.close()
