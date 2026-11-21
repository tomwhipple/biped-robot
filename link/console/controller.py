"""The console, without a screen: what both front ends (the Qt window and
--headless) drive.

It owns the core (link/console/core.py -> link/client.h), the operator stops
(stops.py), the recording and the event log, and turns named actions into
core calls. It decides nothing about the wire itself; the rules are the
core's. Everything runs on the caller's thread except a stop burst (stops.py
says why), so a window that freezes stops ticking and the robot stands.
"""
from __future__ import annotations

import os
import time

from . import core
from .core import Core
from .simproc import SimControl
from .stops import StopBurst, stop_host, targets


class EventLog:
    """bimo_gui.log's format: `<UTC> <kind> <message>`, one line per event,
    flushed per line; kept in memory for the Session tab too."""

    def __init__(self, path: str | None, echo=None, keep: int = 2000):
        self.path = path
        self.f = None
        self.echo = echo
        self.keep = keep
        self.rows: list[tuple[float, str, str]] = []
        if path:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                self.f = open(path, "a", buffering=1)
            except OSError as e:
                self.f = None
                self.rows.append((time.time(), "ERROR", f"cannot append to {path}: {e}"))

    def __call__(self, kind: str, msg: str) -> None:
        t = time.time()
        stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))
        line = f"{stamp} {kind:<6s} {msg}"
        if self.f is not None:
            self.f.write(line + "\n")
        if self.echo is not None:
            self.echo(line)
        self.rows.append((t, kind, msg))
        if len(self.rows) > self.keep:
            del self.rows[: len(self.rows) - self.keep]

    def close(self) -> None:
        if self.f is not None:
            self.f.close()
            self.f = None


class Controller:
    def __init__(self, opts, log: EventLog | None = None):
        self.o = opts
        self.core = Core()
        self.log = log or EventLog(None)
        self.status_msg = ""          # the console's own last message
        self.mirror = False
        self.quit = False
        self.st = None                # the latest core.Status
        self.stops: list[StopBurst] = []
        self.last_stop = None
        self._fed = 0                 # beacons already fed to stop bursts
        self._last_note = ""
        self.listeners = []           # callables(st) per tick (the window)
        self.host = opts.host
        self.linked = True            # False after a failed relink: no link at all
        self.sim = SimControl(self)   # sil_twin, its ports and the tee

    # -- start-up --------------------------------------------------------
    def start(self) -> bool:
        o = self.o
        self.core.set_speeds(o.vx, o.vy, o.wz)
        # --drive starts as an observer too and then takes control the way
        # TAKE CONTROL does, adopting the arm level the robot reports: opening
        # disarmed against a robot another console drives would put a 1 -> 0
        # ARM edge on the wire and bench it mid-stride (client.h adopt).
        if not self.core.listen(o.tlm_port, o.host, o.cmd_port):
            self.log("FATAL", f"listen: cannot bind telemetry port {o.tlm_port}"
                     + (f" (or bad --host {o.host})" if o.host else ""))
            return False
        self.log("start", f"bimo_console -> {o.host or '(no host)'}:{o.cmd_port} "
                 f"(telemetry :{o.tlm_port}), repo {o.repo}")
        if o.observe:
            self.log("readonly", "OBSERVER: no transmit socket. TAKE CONTROL "
                     + ("will command " + o.host if o.host else
                        "unavailable: no --host")
                     + "; HOLD / E-STOP work regardless.")
        if not o.observe and not self.adopt_on_start():
            return False
        if o.watch and not self.set_watch(o.watch):
            self.log("FATAL", f"--watch {o.watch}: expected HOST[:PORT] with a "
                     f"dotted-quad host")
            return False
        path = o.keymap or core.default_keymap_path()
        self.keymap_path = path
        if path:
            ok, err = self.core.load_keys(path)
            if not ok:
                self.log("FATAL", f"keymap: {err}")
                return False
        if not o.no_record:
            self.toggle_record()
        if o.mirror:
            self.set_mirror(True)
        return True

    def adopt_on_start(self, wait_s: float = 1.0) -> bool:
        """--drive: wait up to `wait_s` for a beacon, then TAKE CONTROL's
        adopt path. No beacon: it starts disarmed and says so, as TAKE
        CONTROL does."""
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            now = core.now_ms()
            self.core.poll(now)
            if self.core.status(now).fresh:
                break
            time.sleep(0.01)
        self.st = self.core.status(core.now_ms())
        ok = self.toggle_control()
        if not ok:
            self.log("FATAL", f"--drive: {self.status_msg}")
        return ok

    # -- the tick --------------------------------------------------------
    def tick(self, now: float | None = None):
        now = core.now_ms() if now is None else now
        self.core.tick(now)
        st = self.core.status(now)
        self.st = st
        if st.tlm_count != self._fed:
            self._fed = st.tlm_count
            for b in self.stops:
                b.feed(st.state, st.diag)
        self.sim.poll()
        for b in list(self.stops):
            if b.done:
                self.stops.remove(b)
                self.last_stop = b
                self.status_msg = b.summary()
                self.log("stop", b.summary())
        note = self.core.note
        if note != self._last_note:
            self._last_note = note
            if note:
                self.log("note", note)
        for fn in self.listeners:
            fn(st)
        return st

    def say_status(self, msg: str) -> None:
        self.status_msg = msg
        if msg:
            self.log("status", msg)

    @property
    def readonly(self) -> bool:
        return bool(self.st.readonly) if self.st is not None else self.o.observe

    # -- actions ---------------------------------------------------------
    def press(self, axis: int) -> bool:
        return self.core.press(axis, core.now_ms())

    def release(self, axis: int) -> None:
        self.core.release(axis)

    def release_all(self) -> None:
        self.core.release_all()

    def fire(self, name: str) -> bool:
        """A non-motion keymap action by name. False if unknown."""
        now = core.now_ms()
        if name == "arm":
            st = self.st
            if self.stopping() and not (st is not None and st.armed):
                self.core.say(f"ARM refused: {self._stop_wait()}")
                return True
            self.core.toggle_arm(now)
        elif name == "stand":
            self.core.stand("stand")
        elif name == "estop":
            self.estop()
        elif name == "hold":
            self.hold()
        elif name == "home":
            self.home(now)
        elif name == "quit":
            self.quit = True
        elif name == "record":
            self.toggle_record()
        elif name == "control":
            self.toggle_control()
        elif name == "reset_ext":
            self.core.reset_ext()
        else:
            return False
        return True

    def estop(self) -> None:
        """The E-STOP button: the out-of-band HARD stop, in every role, and
        for a driver also the commander's classic latch -- what firmware
        older than kFlagStop understands."""
        if not self.readonly:
            self.core.estop()
        self._stop(hard=True)
        self.core.say("E-STOP: hard stop sent out of band -- torque off, the "
                      "run ended; re-arm to drive again"
                      + ("" if self.readonly else " (and the classic ESTOP "
                         "latch, for older firmware)"))

    def home(self, now: float) -> None:
        """RESET SERVOS. Refused while a stop burst is still going out: a
        hard-stop frame releases torque on a benched robot, so the rest of
        the burst would undo the reset. The stop wins."""
        if self.stopping():
            self.core.say(f"RESET SERVOS refused: {self._stop_wait()}")
            return
        self.core.home(now)

    def _stop_wait(self) -> str:
        """The refusal's countdown: to the last frame of the longest burst,
        escalation included (StopBurst.end moves when it escalates)."""
        left = max((b.left() for b in self.stops), default=0.0)
        when = f"{left:.1f} s left" if left > 0.05 else "waiting for its verdict"
        return f"a stop is still being sent ({when}); press it again after"

    def hold(self) -> None:
        """HOLD: the out-of-band SOFT stop (joints held stiff), every role;
        escalated to hard if the robot does not confirm it in 400 ms."""
        self._stop(hard=False)

    def _stop(self, hard: bool) -> None:
        o = self.o
        host, src = stop_host(o.stop_host or self.host)
        dests = targets(host, o.cmd_port, None if o.no_broadcast else "255.255.255.255")
        kind = "HARD" if hard else "SOFT"
        if not dests:
            msg = (f"{kind} STOP: nowhere to send it (no --host, no $BIMO_HOST, "
                   f"no robot_addr). Servo power switch.")
            self.status_msg = msg
            self.log("stop", msg)
            return
        st = self.st
        bench = bool(st is not None and st.fresh and st.state == core.BENCH)
        b = StopBurst(hard, dests, was_bench=bench)
        self.stops.append(b)
        b.start()
        msg = (f"{kind} STOP sending to {', '.join(h for h, _ in dests)} "
               f"({src})")
        self.status_msg = msg
        self.log("stop", msg)

    def stopping(self) -> bool:
        return bool(self.stops)

    def toggle_record(self) -> None:
        now = core.now_ms()
        st = self.core.status(now)
        if st.rec_active:
            self.say_status(f"recorded {st.rec_cmd} cmd / {st.rec_tlm} tlm -> "
                            f"{self.core.rec_path}")
            self.core.rec_stop()
            return
        ok, err = self.core.rec_start(self.o.sessions, self.host or "0.0.0.0", now)
        self.say_status(f"recording -> {self.core.rec_path}" if ok else err)

    def toggle_control(self) -> bool:
        was = self.readonly
        ok, msg = self.core.toggle_control(self.host, core.now_ms())
        self.st = self.core.status(core.now_ms())   # readonly is the new role now
        self.say_status(msg)
        if ok:
            self.log("control" if was else "readonly",
                     msg + (f" | {self.core.note}" if was else ""))
            # Re-ask for the mirror on the new socket, quietly: the
            # handover's own message (WATCH ONLY / DRIVING) stays the status.
            self.set_mirror(self.mirror, say=False)
        return ok

    def set_mirror(self, on: bool, say: bool = True) -> None:
        """Ask the robot for joint angles + attitude (the 3D view's measured
        body). An observer cannot ask -- that would be a frame -- so it draws
        whatever the driver's requests put in the relayed beacon."""
        self.mirror = on
        self.core.request(on, on)
        if not say:
            return
        if on and self.readonly:
            self.say_status("MIRROR (observer): the 3D view draws the robot's "
                            "measured joints when the driving console asks for "
                            "them; flight records need no driver.")
        elif on:
            self.say_status(f"MIRROR: asking {self.host} for joint angles and "
                            f"attitude for the 3D view.")
        else:
            self.say_status("mirror off")

    def set_ext(self, crouch: float, lift: float, foot_dx: float, foot_dz: float):
        self.core.set_ext(crouch, lift, foot_dx, foot_dz)

    def set_speeds(self, vx: float, vy: float, wz: float) -> None:
        self.core.set_speeds(vx, vy, wz)

    def set_watch(self, spec: str | None) -> bool:
        if not spec:
            self.core.clear_watch()
            self.say_status("beacon relay off")
            return True
        if not self.core.set_watch(spec, self.o.tlm_port):
            return False
        self.say_status(f"relaying every beacon to {spec}"
                        + (" once this console takes control" if self.readonly else ""))
        self.log("watch", self.status_msg)
        return True

    def relink(self, host: str, cmd_port: int, tlm_port: int) -> bool:
        observe = self.readonly
        was_armed = bool(self.st is not None and self.st.armed and not observe)
        ok = self.core.relink(host, cmd_port, tlm_port, observe)
        if not ok:
            # The core failed closed: no socket, no destination, read-only.
            # Nothing may go on naming the old robot as this console's.
            self.host = None
            self.o.host = None
            self.linked = False
            self.st = self.core.status(core.now_ms())
            self.say_status(f"NO LINK: relink to {host or '(no host)'}:{cmd_port} "
                            f"(telemetry :{tlm_port}) failed -- nothing is sent "
                            f"anywhere"
                            + (" (the old link was disarmed first)" if was_armed else "")
                            + "; fix the address and apply again (as an observer: "
                            "TAKE CONTROL after)")
            self.log("ERROR", self.status_msg)
            return False
        self.host = host
        self.linked = True
        self.o.host, self.o.cmd_port, self.o.tlm_port = host, cmd_port, tlm_port
        if self.o.watch:
            self.core.set_watch(self.o.watch, tlm_port)
        self.set_mirror(self.mirror, say=False)
        self.st = self.core.status(core.now_ms())
        self.say_status(f"{'watching' if observe else 'driving'} {host}:{cmd_port} "
                        f"(telemetry :{tlm_port})"
                        + (" -- disarmed by the move" if was_armed else ""))
        self.log("relink", self.status_msg)
        return True

    # -- leaving ---------------------------------------------------------
    def shutdown(self) -> None:
        """Quit and window close: disarm (10 frames), unless watching; let a
        stop burst in flight finish first. Once done (the core is freed),
        a second call is a no-op: a window's close already ran it."""
        if self.core.freed:
            return
        # A stop still going out finishes first, hearing the beacon but
        # sending nothing else.
        seen = -1
        deadline = time.monotonic() + 3.0
        while any(not b.done for b in self.stops) and time.monotonic() < deadline:
            now = core.now_ms()
            self.core.poll(now)
            st = self.core.status(now)
            if st.tlm_count != seen:
                seen = st.tlm_count
                for b in self.stops:
                    b.feed(st.state, st.diag)
            time.sleep(0.01)
        for b in self.stops:
            self.log("stop", b.summary())
        self.core.leave_disarmed(10, 20)
        st = self.core.status(core.now_ms())
        if st.rec_active:
            self.log("rec", f"{st.rec_cmd} cmd / {st.rec_tlm} tlm -> {self.core.rec_path}")
            self.core.rec_stop()
        self.sim.stop()                 # after the disarm: a twin hears it too
        self.core.close()
        self.log("quit", f"sent disarm ({st.sent} frames total)"
                 if not st.readonly else "observer: sent nothing on the way out")
        self.core.free()
