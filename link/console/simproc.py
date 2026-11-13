"""sim/sil_twin.py as a child of the console: link/simproc.{h,cpp}'s rules in
Python, plus twin-in-the-loop (design §6 Sim, §15 decision 2).

sil_twin runs the REAL firmware control stack behind the real UDP link at 1x
real time and serves its render as MJPEG, so "drive the sim" and "drive the
robot" differ only in --host. A child that dies must be VISIBLY dead: its exit
status and its last lines go in the log, because a console that keeps showing
the last frame of a crashed sim is the failure tools/cam_live.py:87 already
had to fix once.

No Qt here: the window (tab_sim.py) and --headless (headless.py) both drive
SimControl, and the tests drive it with neither. The child's output is read on
a thread of its own (a pipe nobody drains stalls the child); everything else
runs on the caller's thread.
"""
from __future__ import annotations

import collections
import json
import os
import shlex
import signal
import subprocess
import threading
import time

LOG_LINES = 200            # simproc.h kSimLogLines
LOG_COLS = 200             # kSimLogCols: a longer line is split, not lost
MAX_RUNS = 128             # kMaxRuns
STOP_WAIT_S = 2.0          # SIGTERM, then this long, then SIGKILL (SimProc::stop)
LOCAL_HOSTS = ("localhost",)


def python_for(repo: str) -> str:
    return os.path.join(repo, ".venv", "bin", "python")


def script_for(repo: str) -> str:
    return os.path.join(repo, "sim", "sil_twin.py")


def run_config(repo: str, run: str) -> str:
    """A run's config.json: sim/runs/<run>/'s, else the copy tracked beside its
    weights, sim/sil/weights/<run>.config.json (sim/sil/run_config.py; #236:
    the deployed run's is tracked, so a clean clone can run it). The run
    directory's path when neither exists."""
    own = os.path.join(repo, "sim", "runs", run, "config.json")
    if os.path.exists(own):
        return own
    tracked = os.path.join(repo, "sim", "sil", "weights", f"{run}.config.json")
    return tracked if os.path.exists(tracked) else own


SIDECAR = ".silw.json"


def scan_runs(repo: str) -> list[str]:
    """RunList::scan: the runs sil_twin can actually load -- an exported
    sim/sil/weights/<name>.silw.json AND a config.json (run_config).
    Anything else fails at startup with a traceback, a poor dropdown entry.
    Directory order up to MAX_RUNS, then sorted bytewise (strcmp), as the C++."""
    d = os.path.join(repo, "sim", "sil", "weights")
    names = []
    try:
        it = os.scandir(d)
    except OSError:
        return []
    with it:
        for e in it:
            if len(names) >= MAX_RUNS:
                break
            if e.name.startswith(".") or not e.name.endswith(SIDECAR) \
                    or len(e.name) == len(SIDECAR):
                continue
            run = e.name[:-len(SIDECAR)]
            if os.path.exists(run_config(repo, run)):
                names.append(run)
    return sorted(names, key=lambda n: n.encode())


def no_runs_help(repo: str) -> str:
    """What the Sim tab says when scan_runs() finds nothing (#236): what makes
    a run runnable, and the two ways to get one into a clone."""
    wdir = os.path.join(repo, "sim", "sil", "weights")
    try:
        tracked = sorted(f[:-len(".silw.json")] for f in os.listdir(wdir)
                         if f.endswith(".silw.json"))
    except OSError:
        tracked = []
    lines = ["No runnable sim run in this clone. A run is listed when it has both "
             "sim/runs/<run>/config.json (or a tracked "
             "sim/sil/weights/<run>.config.json) and sim/sil/weights/<run>.silw.json; "
             "sim/runs/ is not in git."]
    if tracked:
        lines.append("Weights are here for: " + ", ".join(tracked) + ". Their run "
                     "directories are on the training box (mira, ~/code/robot/sim/runs/"
                     "<run>/): copy that run's config.json and params.pkl into "
                     "sim/runs/<run>/ keeping their times (scp -p), then rescan.")
    lines.append("Or export a run you have: python tools/export_policy_weights.py "
                 "--run <run> --no-golden (without --no-golden it rewrites the tracked "
                 "sim/sil/golden/*.json).")
    return "\n".join(lines)


def run_plant(repo: str, run: str) -> str:
    """The plant a run trained on (its config.json xml_path), '' if unknown."""
    try:
        with open(run_config(repo, run)) as f:
            return str(json.load(f).get("xml_path") or "")
    except (OSError, ValueError, AttributeError):
        return ""


def plants(repo: str) -> list[str]:
    """The robot plants under sim/ a run can be pointed at (sil_twin --xml)."""
    try:
        return sorted(n for n in os.listdir(os.path.join(repo, "sim"))
                      if n.startswith("bimo_biped") and n.endswith(".xml"))
    except OSError:
        return []


_help_cache: dict[tuple[str, float], bool] = {}


def supports_flight(repo: str) -> bool:
    """Does this repo's sil_twin take --flight (flight records, design §9.3)?
    Asked of its own --help (argparse exits before anything heavy loads),
    once per version of the script."""
    script, python = script_for(repo), python_for(repo)
    try:
        key = (script, os.path.getmtime(script))
    except OSError:
        return False
    if key not in _help_cache:
        try:
            out = subprocess.run([python, script, "--help"], cwd=os.path.dirname(script),
                                 capture_output=True, text=True, timeout=20).stdout
        except (OSError, subprocess.SubprocessError):
            out = ""
        _help_cache[key] = "--flight" in out
    return _help_cache[key]


def command_line(repo: str, run: str, hang: bool, cmd_port: int, tlm_port: int,
                 stream_port: int, flight: tuple[str, int] | None = None,
                 xml: str | None = None) -> list[str]:
    """`<repo>/.venv/bin/python <repo>/sim/sil_twin.py ...`, simproc.cpp's
    argument order. `flight` is (host, port) for --flight; `xml` an explicit
    plant (sil_twin --xml, which wins over --hang)."""
    argv = [python_for(repo), script_for(repo), "--run-name", run,
            "--stream-port", str(stream_port), "--port", str(cmd_port),
            "--tlm-port", str(tlm_port)]
    if hang:
        argv.append("--hang")
    if xml:
        argv += ["--xml", xml]
    if flight is not None:
        argv += ["--flight", f"{flight[0]}:{flight[1]}"]
    return argv


def parse_ident(line: str) -> str | None:
    """sil_twin's own one-line description of what it runs:
    "sil_twin: <run> on :<port> (tlm -> :<tlm>)  plant=<xml>  spec=..." -> the
    part before the spec blob (SimProc::pushLine)."""
    if line.startswith("sil_twin: ") and "plant=" in line:
        return line[len("sil_twin: "):].split("  spec=", 1)[0]
    return None


class SimProc:
    """One sil_twin child: spawn, the output ring, stdin lines, stop, and the
    reason it died."""

    def __init__(self, repo: str, keep: int = LOG_LINES):
        self.repo = repo
        self.proc: subprocess.Popen | None = None
        self.argv: list[str] = []
        self.err = ""              # why it is not running, when that was not asked for
        self.ident = ""
        self.started = 0.0         # time.monotonic() of the spawn
        self._lines: collections.deque = collections.deque(maxlen=keep)
        self._said: collections.deque = collections.deque(maxlen=8)   # whole child lines
        self._total = 0            # lines ever pushed: the ring's write index
        self._lock = threading.Lock()
        self._reader: threading.Thread | None = None

    # -- the log ring ----------------------------------------------------
    def push(self, text: str) -> None:
        with self._lock:
            self._lines.append((time.time(), text))
            self._total += 1
        ident = parse_ident(text)
        if ident is not None:
            self.ident = ident

    @property
    def total(self) -> int:
        """Lines ever pushed (a cursor for callers that tail the log)."""
        return self._total

    def lines(self, since: int = 0) -> list[tuple[float, str]]:
        """(utc, text) kept, oldest first; `since` = a previous `total`."""
        with self._lock:
            kept = list(self._lines)
            first = self._total - len(kept)
        return kept[max(0, since - first):]

    def last_words(self, n: int = 3) -> list[str]:
        """The child's own last lines, whole (the ring holds them split at
        LOG_COLS), without the console's `$` and `--` lines."""
        with self._lock:
            return list(self._said)[-n:]

    # -- the child -------------------------------------------------------
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, run: str, hang: bool, cmd_port: int, tlm_port: int, stream_port: int,
              flight: tuple[str, int] | None = None, xml: str | None = None) -> bool:
        if self.running():
            return True
        self.poll()                         # a dead child is reported before a new one
        python, script = python_for(self.repo), script_for(self.repo)
        if not os.path.exists(python):
            self.err = f"no interpreter at {python}"
            return False
        if not os.path.exists(script):
            self.err = f"no {script}"
            return False
        return self.spawn(command_line(self.repo, run, hang, cmd_port, tlm_port,
                                       stream_port, flight, xml),
                          os.path.join(self.repo, "sim"))

    def spawn(self, argv: list[str], cwd: str) -> bool:
        """Run from sim/: a run config's xml_path is often a BARE filename,
        which MuJoCo resolves against the cwd (simproc.cpp). Its own process
        group, so stop() cannot signal this console and a ctrl-C in the
        launching terminal does not race us to the child."""
        self.err = ""
        self.ident = ""
        with self._lock:
            self._said.clear()              # this child's last words, not the previous one's
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"       # its lines as they happen, not per 8 KiB
        try:
            self.proc = subprocess.Popen(argv, cwd=cwd, env=env, stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         bufsize=0, start_new_session=True)
        except OSError as e:
            self.proc = None
            self.err = f"cannot start {argv[0]} in {cwd}: {e.strerror or e}"
            return False
        self.argv = list(argv)
        self.started = time.monotonic()
        self.push("$ " + shlex.join(_short(argv, self.repo)))
        self._reader = threading.Thread(target=self._read, args=(self.proc.stdout,),
                                        daemon=True, name="sil_twin output")
        self._reader.start()
        return True

    def _read(self, pipe) -> None:
        partial = b""
        cont = False                # the next piece continues a line split at LOG_COLS
        while True:
            try:
                chunk = pipe.read(4096)
            except (OSError, ValueError):
                break
            if not chunk:
                break
            partial += chunk.replace(b"\r", b"")
            while b"\n" in partial:
                line, partial = partial.split(b"\n", 1)
                self._push_raw(line, cont)
                cont = False
            while len(partial) >= LOG_COLS - 1:
                self._push_raw(partial[:LOG_COLS - 1], cont)
                partial = partial[LOG_COLS - 1:]
                cont = True
        if partial:
            self._push_raw(partial, cont)

    def _push_raw(self, raw: bytes, continued: bool = False) -> None:
        text = raw.decode("utf-8", "replace")
        with self._lock:
            if continued and self._said:
                self._said[-1] += text
            elif text:
                self._said.append(text)
        while len(text) >= LOG_COLS:
            self.push(text[:LOG_COLS - 1])
            text = text[LOG_COLS - 1:]
        if text:
            self.push(text)

    def _drain(self) -> None:
        if self._reader is not None:
            self._reader.join(timeout=1.0)      # its last words, before the verdict
            self._reader = None

    def poll(self) -> str:
        """Reap a child that exited on its own; returns why ('' if it did not).
        Call every tick."""
        p = self.proc
        if p is None or p.poll() is None:
            return ""
        self._drain()
        rc = p.returncode
        msg = (f"sim killed (signal {-rc})" if rc < 0 else f"sim exited (status {rc})")
        self.push(f"-- {msg}")
        self.err = msg
        self._close(p)
        self.proc = None
        return msg

    def tell(self, line: str) -> bool:
        """A line down the child's stdin (sil_twin understands `reset`).
        False if nothing is running to hear it. A dead reader is EPIPE, not a
        signal: Python ignores SIGPIPE, so this console survives a sim that
        died between ticks (the C++ had to install that by hand)."""
        if self.proc is None:
            return False
        if self.proc.poll() is not None:
            self.poll()
            return False
        try:
            self.proc.stdin.write((line.rstrip("\n") + "\n").encode())
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            self.err = "the sim closed its input"
            self.push("-- sim closed its input")
            return False
        return True

    def stop(self) -> bool:
        """SIGTERM to its group, then SIGKILL after STOP_WAIT_S. Safe to call
        when nothing is running. True if there was a child to stop."""
        p = self.proc
        if p is None:
            return False
        if p.poll() is None:
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(p.pid, sig)       # the group; it set its own
                except (ProcessLookupError, PermissionError):
                    pass
                try:
                    p.send_signal(sig)
                except (ProcessLookupError, OSError):
                    pass
                try:
                    p.wait(timeout=STOP_WAIT_S)
                    break
                except subprocess.TimeoutExpired:
                    continue                    # it is not going to leave politely
            if p.poll() is None:
                p.wait()
        self._drain()
        self._close(p)
        self.proc = None
        self.err = ""                           # a deliberate stop is not a failure
        self.push("-- sim stopped")
        return True

    @staticmethod
    def _close(p: subprocess.Popen) -> None:
        for f in (p.stdin, p.stdout):
            try:
                if f is not None:
                    f.close()
            except OSError:
                pass

    @property
    def pid(self) -> int | None:
        return self.proc.pid if self.proc is not None else None

    def uptime(self) -> float:
        return time.monotonic() - self.started if self.running() else 0.0


def _short(argv: list[str], repo: str) -> list[str]:
    """The command line with the repo prefix dropped, for the log."""
    pre = repo.rstrip("/") + "/"
    return [a[len(pre):] if a.startswith(pre) else a for a in argv]


def is_local(host: str | None) -> bool:
    return bool(host) and (host.startswith("127.") or host in LOCAL_HOSTS)


class Plan:
    """Where a sim would listen and report, and why (the Sim tab's ports line)."""

    def __init__(self, cmd_port, tlm_port, own, why, flight):
        self.cmd_port, self.tlm_port, self.own, self.why, self.flight = (
            cmd_port, tlm_port, own, why, flight)


class SimControl:
    """The console's sim: the run list, the SimProc, the port rule, the tee
    (twin-in-the-loop) and the sim's MJPEG stream. Owned by the Controller so
    the window, --headless and quit all reach the same child.

    Ports (bimo_gui's rule, re-cut for a console that starts as an observer):
      * the sim is this console's own target (its --host is 127.0.0.1) and
        twin-in-the-loop is off: it listens on the console's cmd / tlm ports,
        so TAKE CONTROL drives it, exactly as it would drive the robot;
      * twin-in-the-loop on, or the console points at a robot: the sim gets
        its own ports (--sim-cmd-port, +1 for its beacon), so its beacon never
        lands in the robot's charts. With twin on, every command frame the
        console sends is also sent, byte for byte, to that port (core
        set_tee); an observer sends no frames, so there is nothing to tee.
    Flight records (when this sil_twin has --flight): to --sim-flight-port when
    the sim has its own ports (the 3D view's amber ghost); to --flight-port when
    it IS the console's target, where it stands in for the robot as its beacon
    already does.
    """

    def __init__(self, ctl):
        self.ctl = ctl
        o = ctl.o
        self.repo = o.repo
        self.proc = SimProc(self.repo)
        self.runs = scan_runs(self.repo)
        self.run = (o.run_name if o.run_name in self.runs
                    else (self.runs[0] if self.runs else None))
        self.hang = False
        self.plant = None              # explicit sim/<xml> (--xml), None = run's / hang
        self.twin = False              # twin-in-the-loop: off by default (§15 decision 2)
        self.status = ""
        self.active: Plan | None = None   # the running child's ports
        self.tee_to: tuple[str, int] | None = None
        self.stream = None             # the sim's MJPEG reader (the window's), or None
        self._stream_factory = None
        self._logged = 0
        ctl.log("runs", f"{len(self.runs)} policy run(s) with SIL weights under "
                f"{self.repo}/sim/runs")

    # -- what a Start would do --------------------------------------------
    def rescan(self) -> list[str]:
        self.runs = scan_runs(self.repo)
        if self.run not in self.runs:
            self.run = self.runs[0] if self.runs else None
        return self.runs

    def select(self, run: str) -> bool:
        if run not in self.runs:
            return False
        self.run = run
        return True

    def plan(self) -> Plan:
        o, host = self.ctl.o, self.ctl.host
        if self.twin:
            own, why = True, "its own ports (twin-in-the-loop)"
        elif is_local(host):
            own, why = False, f"the console's own ports (it drives {host})"
        elif host:
            own, why = True, (f"its own ports: the console points at {host}, not at the "
                              f"sim (nothing drives it unless twin-in-the-loop is on)")
        else:
            own, why = True, "its own ports (the console has no --host)"
        cp, tp = (o.sim_cmd_port, o.sim_cmd_port + 1) if own else (o.cmd_port, o.tlm_port)
        port = o.sim_flight_port if own else (o.flight_port or o.sim_flight_port)
        flight = ("127.0.0.1", port) if port and supports_flight(self.repo) else None
        return Plan(cp, tp, own, why, flight)

    def plant_xml(self) -> str | None:
        return os.path.join(self.repo, "sim", self.plant) if self.plant else None

    # -- start / stop / reset ---------------------------------------------
    def start(self, run: str | None = None) -> bool:
        self.poll()                     # a child that died since the last tick says so first
        if run is not None and not self.select(run):
            self._say(f"no run `{run}`")
            return False
        if self.proc.running():
            self._say(f"sim {self.run} already running (pid {self.proc.pid})")
            return True
        if not self.runs or self.run is None:
            self._say(f"no runnable sim runs under {self.repo}/sim/runs (each needs a "
                      f"config.json AND sim/sil/weights/<run>.silw.json)")
            return False
        p = self.plan()
        if not self.proc.start(self.run, self.hang, p.cmd_port, p.tlm_port,
                               self.ctl.o.stream_port, p.flight, self.plant_xml()):
            self._say(self.proc.err)
            return False
        self.active = p
        self._pump()
        self.apply_tee()
        url = self.view_url(ours=True)
        self._watch(url)
        self._say(f"sim {self.run} starting on {p.why}: cmd {p.cmd_port}, tlm {p.tlm_port}; "
                  f"view {url}")
        return True

    def stop(self) -> bool:
        """The Stop button, and quit. Also stops the stream (bimo_gui
        stopSim); a --sim-view stream the operator named is watched again."""
        had = self.proc.stop()
        self._pump()
        self.active = None
        self.apply_tee()
        self._watch(self.ctl.o.sim_view)
        if had:
            self._say("sim stopped")
        return had

    def reset(self) -> bool:
        """Re-roll the episode in place: restarting the process costs ~30 s of
        MuJoCo and policy loading, long enough that nobody would."""
        ok = self.proc.tell("reset")
        self._pump()
        return ok

    def set_hang(self, on: bool) -> None:
        self.hang = on

    def set_twin(self, on: bool) -> str:
        self.twin = on
        self.apply_tee()
        self._say(self.twin_text())
        return self.status

    # -- the tee ---------------------------------------------------------
    def apply_tee(self) -> None:
        """Tee iff twin-in-the-loop is on and a sim of ours listens on its own
        ports. Never to the console's own target: that sim already gets every
        frame, and a second copy would be a second command."""
        want = None
        if self.twin and self.active is not None and self.active.own and self.proc.running():
            want = ("127.0.0.1", self.active.cmd_port)
            if is_local(self.ctl.host) and want[1] == self.ctl.o.cmd_port:
                want = None             # --sim-cmd-port IS the console's target port
        if want == self.tee_to:
            return
        if want is None:
            self.ctl.core.clear_tee()
            self.ctl.log("sim", "twin-in-the-loop: tee off")
        elif self.ctl.core.set_tee(*want):
            self.ctl.log("sim", f"twin-in-the-loop: every command frame also to "
                         f"{want[0]}:{want[1]}")
        else:
            want = None
        self.tee_to = want

    def twin_text(self) -> str:
        """What twin-in-the-loop is doing right now, in words."""
        if not self.twin:
            return "twin-in-the-loop off"
        if self.ctl.readonly:
            return ("twin-in-the-loop on, but this console is an OBSERVER: it sends no "
                    "command frames, so there is nothing to tee. TAKE CONTROL to drive "
                    "the robot and its twin.")
        if not self.proc.running():
            p = self.plan()
            return (f"twin-in-the-loop on: Start runs the sim on its own ports (cmd "
                    f"{p.cmd_port}, tlm {p.tlm_port}) and every command frame is copied to it")
        if self.active is not None and not self.active.own:
            return (f"the running sim is this console's own target (it listens on "
                    f"{self.active.cmd_port}) and already gets every frame. Twin-in-the-loop "
                    f"needs it on its own ports: Stop, then Start.")
        return f"twin-in-the-loop on: every command frame also to 127.0.0.1:{self.active.cmd_port}"

    # -- the stream --------------------------------------------------------
    def view_url(self, ours: bool) -> str:
        """Where to watch the sim. --sim-view wins. A sim THIS console started
        is a local child, so its stream is on 127.0.0.1 whatever --host says;
        one it did not start is where --host points (gui.cpp simViewUrl)."""
        o = self.ctl.o
        if o.sim_view:
            return o.sim_view
        return f"http://{'127.0.0.1' if ours else (self.ctl.host or '127.0.0.1')}:{o.stream_port}/"

    def attach_stream(self, factory) -> None:
        """The window's MJPEG reader: factory(url) -> an object with start() and
        stop(). Starts watching --sim-view at once (an external stream)."""
        self._stream_factory = factory
        self._watch(self.view_url(ours=True) if self.proc.running() else self.ctl.o.sim_view)

    def _watch(self, url: str | None) -> None:
        if self.stream is not None and url is not None and getattr(self.stream, "url", None) == url:
            return
        if self.stream is not None:
            self.stream.stop()
            self.stream = None
        if url and self._stream_factory is not None:
            self.stream = self._stream_factory(url)
            self.stream.start()

    def external(self) -> bool:
        """A stream this console did not start is showing frames."""
        return (not self.proc.running() and self.stream is not None
                and getattr(self.stream, "frames", 0) > 0)

    # -- per tick ----------------------------------------------------------
    def poll(self) -> None:
        """The child's lines to the event log; a child that died on its own
        is said out loud, with its last words, and its tee dropped."""
        why = self.proc.poll()
        self._pump()
        if why:
            self.active = None
            self.apply_tee()
            self._watch(self.ctl.o.sim_view)
            last = [t.replace(self.repo.rstrip("/") + "/", "") for t in self.proc.last_words(1)]
            self.ctl.log("SIM", why)
            self.status = (f"the {why}" + (f": {last[0]}" if last else "")
                           + " -- its last lines are in the Sim tab's log")
            self.ctl.status_msg = self.status

    def _pump(self) -> None:
        for _, text in self.proc.lines(self._logged):
            self.ctl.log("sim", text)
        self._logged = self.proc.total

    def _say(self, msg: str) -> None:
        self.status = msg
        self.ctl.say_status(msg)
