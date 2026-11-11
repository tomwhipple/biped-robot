"""The gamepad (design §7 "Gamepad" and "The Bluetooth hazard", §14).

SDL's GameController API through pygame, mapped by POSITION (never by raw
joystick index, which differs per pad and per OS), polled on the GUI thread
once per link tick and never blocking:

  SDL name          C6 (Xbox) label  does
  rightshoulder     RB               dead-man: the pad drives only while held
  b (east)          B                E-STOP: the out-of-band hard stop
                                     (Controller.estop), in every role, any host
  lefty             left stick up/dn vx through sources.stick_to_speed (the
                                     trained band of its direction); up = forward
  leftx             left stick l/r   vy through sources.stick_to_lateral (the
                                     trained sidestep band); left = +vy
  rightx            right stick l/r  wz, linear to protocol.W_MAX; left = +wz

Nothing else: the pad cannot ARM or RESET SERVOS. Axes are SDL's int16
divided by 32767; one dead zone serves all three sticks.

Rules (each one makes the robot stand rather than move):

* A released dead-man gives axes_held() None on the next poll, so the keys
  drive again and the slider speeds come back.
* release() (the window lost focus), the pad's E-STOP, a disconnect or a
  silent pad LATCH it released: it drives again only after RB is let go and
  pressed again. A pad that connects with RB already held is latched too.
* Disconnect (the OS removes the device, or SDL says it is no longer
  attached) releases everything, keys included, and says so in the log.
* The silent pad (the Bluetooth hazard): SDL reports an input only when a
  value changes, so a pad held still is silent and silence means nothing --
  unless THIS pad has been seen streaming while held still (a motion sensor's
  reports, stick noise) with gaps under half the timeout. Only then does a
  gap over `silent_ms` with the dead-man held release it; otherwise only the
  OS disconnect counts. status() and detail() say which.
* SIM ONLY (design §7: "until then the pad drives the sim only"): the
  dead-man drives only while the console's link host is loopback (127.x);
  for any other host it is ignored and the status says so. Lift this only
  with the §14 measurements written down. The pad's E-STOP works for any host.
"""
from __future__ import annotations

import collections
import ipaddress
import math
import os
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402  (a missing pygame is the app's "no gamepad")
import pygame._sdl2.controller as sdl_controller  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import protocol  # noqa: E402  link/protocol.py
import sources  # noqa: E402  link/sources.py

from .ui import AMBER, BRIGHT, DIM, GREEN, RED  # noqa: E402

SIM_ONLY = True               # design §7; see the module docstring
SILENT_MS = 150.0             # the silent-pad timeout
DEAD_ZONE = sources.STICK_DEADZONE
STILL_EPS = 0.03              # stick travel that still counts as "held still"
STILL_S = 1.0                 # a still window this long decides "streams"
LOST_SHOWN_S = 30.0           # the bar says LOST this long after a disconnect

AXES = ("leftx", "lefty", "rightx", "righty", "lefttrigger", "righttrigger")
BUTTONS = ("a", "b", "x", "y", "back", "guide", "start", "leftstick", "rightstick",
           "leftshoulder", "rightshoulder", "dpup", "dpdown", "dpleft", "dpright")
DEADMAN, ESTOP = "rightshoulder", "b"
# (dead-man label, E-STOP label) by pad family: the labels follow the pad, the
# mapping does not.
LABELS = {"xbox": ("RB", "B"), "playstation": ("R1", "○"), "nintendo": ("R", "A")}

_AXIS_ID = {"leftx": pygame.CONTROLLER_AXIS_LEFTX, "lefty": pygame.CONTROLLER_AXIS_LEFTY,
            "rightx": pygame.CONTROLLER_AXIS_RIGHTX, "righty": pygame.CONTROLLER_AXIS_RIGHTY,
            "lefttrigger": pygame.CONTROLLER_AXIS_TRIGGERLEFT,
            "righttrigger": pygame.CONTROLLER_AXIS_TRIGGERRIGHT}
_BUTTON_ID = {n: getattr(pygame, "CONTROLLER_BUTTON_" + s) for n, s in (
    ("a", "A"), ("b", "B"), ("x", "X"), ("y", "Y"), ("back", "BACK"), ("guide", "GUIDE"),
    ("start", "START"), ("leftstick", "LEFTSTICK"), ("rightstick", "RIGHTSTICK"),
    ("leftshoulder", "LEFTSHOULDER"), ("rightshoulder", "RIGHTSHOULDER"),
    ("dpup", "DPAD_UP"), ("dpdown", "DPAD_DOWN"), ("dpleft", "DPAD_LEFT"),
    ("dpright", "DPAD_RIGHT"))}
_INPUT_EVENTS = {pygame.JOYAXISMOTION, pygame.JOYBALLMOTION, pygame.JOYHATMOTION,
                 pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.CONTROLLERAXISMOTION,
                 pygame.CONTROLLERBUTTONDOWN, pygame.CONTROLLERBUTTONUP,
                 pygame.CONTROLLERSENSORUPDATE}


def family(name: str) -> str:
    n = (name or "").lower()
    if any(k in n for k in ("ps3", "ps4", "ps5", "dualshock", "dualsense", "playstation",
                            "sony")):
        return "playstation"
    if any(k in n for k in ("nintendo", "switch", "pro controller", "joy-con")):
        return "nintendo"
    return "xbox"


def is_loopback(host) -> bool:
    """127.x only: the console's own machine, i.e. the sim."""
    try:
        a = ipaddress.ip_address(host or "")
    except ValueError:
        return False
    return a.version == 4 and a.is_loopback


def stick_frac(a: float, dz: float) -> float:
    """0 inside the dead zone, then 0..1 over the remaining travel, signed."""
    mag = abs(a)
    if mag < dz:
        return 0.0
    return math.copysign(min(1.0, (mag - dz) / (1.0 - dz)), a)


def _through(f, a: float, dz: float) -> float:
    """A sources.stick_to_* map with this pad's dead zone: the travel is
    rescaled so the edge of `dz` lands where the map's own dead zone
    (sources.STICK_DEADZONE) ends -- identical when dz is that dead zone."""
    if abs(a) < dz:
        return 0.0
    d0 = sources.STICK_DEADZONE
    return f(math.copysign(d0 + abs(stick_frac(a, dz)) * (1.0 - d0), a))


def vx_from_stick(a: float, dz: float = DEAD_ZONE) -> float:
    """sources.stick_to_speed with this pad's dead zone, so the stick never
    asks for a speed in the hole under its direction's band (V_FWD_MIN
    forward, V_BACK_MIN backward; protocol.py, generated from the run)."""
    return _through(sources.stick_to_speed, a, dz)


def vy_from_stick(a: float, dz: float = DEAD_ZONE) -> float:
    """sources.stick_to_lateral with this pad's dead zone: never a |vy| in
    the (0, VY_MIN) hole the robot snaps to zero."""
    return _through(sources.stick_to_lateral, a, dz)


# -- the SDL layer -----------------------------------------------------------
class SdlPad:
    """One open SDL GameController."""

    def __init__(self, index: int):
        self.c = sdl_controller.Controller(index)
        js = self.c.as_joystick()
        self.js = js
        self.instance_id = js.get_instance_id()
        self.name = self.c.name or js.get_name() or "?"
        self.guid = js.get_guid()
        m = self.c.get_mapping()
        self.mapping = ",".join(f"{k}:{v}" for k, v in m.items()) if isinstance(m, dict) else str(m)

    def attached(self) -> bool:
        return bool(self.c.attached())

    def state(self):
        axes = {k: max(-1.0, min(1.0, self.c.get_axis(i) / 32767.0)) for k, i in _AXIS_ID.items()}
        buttons = {k: bool(self.c.get_button(i)) for k, i in _BUTTON_ID.items()}
        return axes, buttons

    def power(self) -> str:
        try:
            return self.js.get_power_level()
        except (pygame.error, AttributeError):
            return ""

    def close(self) -> None:
        try:
            self.c.quit()
        except pygame.error:
            pass


class SdlPads:
    """The only code that touches pygame's device layer. Tests inject a fake
    with the same methods (scan, open, events, quit)."""

    def __init__(self):
        # The pad's E-STOP must work whichever window has focus, and "east"
        # must be east on a Nintendo layout too (SDL2 otherwise maps those by
        # label). Both are read when SDL initialises the subsystems below.
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
        os.environ["SDL_GAMECONTROLLER_USE_BUTTON_LABELS"] = "0"
        # SDL otherwise takes SIGINT / SIGTERM for itself (turning them into a
        # quit event nobody reads), and the console could not be stopped.
        # The console handles both: it closes, disarming (app.py).
        os.environ["SDL_NO_SIGNAL_HANDLERS"] = "1"
        # pygame's event queue needs SDL's video subsystem; the dummy driver
        # keeps SDL off Cocoa / X11, which Qt owns. Restored at once so no
        # child process inherits it.
        prev = os.environ.get("SDL_VIDEODRIVER")
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        try:
            pygame.display.init()
        finally:
            if prev is None:
                os.environ.pop("SDL_VIDEODRIVER", None)
            else:
                os.environ["SDL_VIDEODRIVER"] = prev
        pygame.joystick.init()
        sdl_controller.init()
        self.sdl = ".".join(str(v) for v in pygame.get_sdl_version())

    def scan(self):
        """[(index, name, is_controller)] for every joystick SDL sees."""
        out = []
        for i in range(pygame.joystick.get_count()):
            ok = bool(sdl_controller.is_controller(i))
            name = sdl_controller.name_forindex(i) if ok else None
            if not name:
                try:
                    name = pygame.joystick.Joystick(i).get_name()
                except pygame.error:
                    name = "?"
            out.append((i, name, ok))
        return out

    def open(self, index: int) -> SdlPad:
        return SdlPad(index)

    def events(self):
        """("added", device_index) / ("removed", instance_id) /
        ("input", instance_id), drained without waiting."""
        out = []
        for ev in pygame.event.get():
            t = ev.type
            if t in (pygame.CONTROLLERDEVICEADDED, pygame.JOYDEVICEADDED):
                out.append(("added", getattr(ev, "device_index", -1)))
            elif t in (pygame.CONTROLLERDEVICEREMOVED, pygame.JOYDEVICEREMOVED):
                out.append(("removed", getattr(ev, "instance_id", -1)))
            elif t in _INPUT_EVENTS:
                out.append(("input", getattr(ev, "instance_id", -1)))
        return out

    def quit(self) -> None:
        sdl_controller.quit()
        pygame.joystick.quit()
        pygame.display.quit()


# -- the console's pad ---------------------------------------------------------
class Gamepad:
    """See the module docstring. The app calls poll() then axes_held() then
    apply(ctl) every link tick; release() on focus loss; close() on quit."""

    def __init__(self, log, backend=None, clock=time.monotonic,
                 silent_ms: float = SILENT_MS, dead_zone: float = DEAD_ZONE):
        self.log = log
        self.clock = clock
        self.silent_ms = silent_ms
        self.dead_zone = dead_zone
        self.host = lambda: None          # install(): the console's link host
        self.slider_speeds = None         # install(): the Drive tab's sliders
        self.app = None
        self.error = ""
        self.backend = backend
        if backend is None:
            try:
                self.backend = SdlPads()
            except pygame.error as e:
                self.error = f"SDL: {e}"
                log("pad", f"gamepad unavailable: {self.error}")
        self.sdl = getattr(self.backend, "sdl", "")
        self.pad = None
        self.labels = LABELS["xbox"]
        self.seen = []                    # scan(): what SDL saw at the last change
        self.axes = {k: 0.0 for k in AXES}
        self.buttons = {k: False for k in BUTTONS}
        self.deadman = self.estop_down = False
        self.latched = None               # why the pad is held released (RB up re-arms)
        self.gated = False                # dead-man held, host not loopback
        self.driving = False              # this tick: the pad drives
        self._was_driving = False         # what apply() last acted on
        self._estop_pending = False
        self._release_pending = None
        self._restore = None
        self.lost_at = None
        self.lost_why = ""
        self.last_input = None
        self.inputs = collections.deque()  # (t, n) for the last second
        self.streams = False              # seen streaming while held still
        self.still_rate = None            # inputs/s over the last still window
        self.still_gap = None             # its longest gap, s
        self._still = None
        if self.backend is not None:
            self._scan_and_open()

    # -- devices -----------------------------------------------------------
    def _scan_and_open(self) -> None:
        self.seen = self.backend.scan()
        if self.pad is not None:
            return
        for index, name, ok in self.seen:
            if ok:
                self._open(index)
                return

    def _open(self, index: int) -> None:
        try:
            pad = self.backend.open(index)
        except pygame.error as e:
            self.log("pad", f"gamepad {index}: cannot open: {e}")
            return
        self.pad = pad
        self.labels = LABELS[family(pad.name)]
        self.axes, self.buttons = pad.state()
        self.deadman = self.buttons[DEADMAN]
        self.estop_down = self.buttons[ESTOP]
        self.latched = "connected with the dead-man held" if self.deadman else None
        self.streams = False
        self.still_rate = self.still_gap = None
        self._still = None
        self.last_input = self.clock()
        self.inputs.clear()
        self.lost_at = None
        self.log("pad", f"gamepad connected: {pad.name} (GUID {pad.guid}); "
                 f"{self.labels[0]} = dead-man, {self.labels[1]} (east) = E-STOP"
                 + ("; drives the sim only (design §7)" if SIM_ONLY else ""))

    def _lost(self, why: str) -> None:
        name = self.pad.name if self.pad is not None else "gamepad"
        if self.pad is not None:
            self.pad.close()
        self.pad = None
        self.lost_at = self.clock()
        self.lost_why = why
        self.deadman = self.estop_down = self.driving = self.gated = False
        self.latched = None
        self._release_pending = f"gamepad lost ({why})"
        self.log("pad", f"{name}: {why} -- every input released")

    # -- the tick ------------------------------------------------------------
    def poll(self) -> None:
        """Drain SDL's events and read the pad. Never blocks."""
        if self.backend is None:
            return
        now = self.clock()
        n = 0
        changed = False
        for kind, ident in self.backend.events():
            if kind == "added":
                changed = True
            elif kind == "removed":
                changed = True
                if self.pad is not None and ident == self.pad.instance_id:
                    self._lost("disconnected (the OS removed it)")
            elif kind == "input" and self.pad is not None and ident == self.pad.instance_id:
                n += 1
        if changed:
            self._scan_and_open()
        if self.pad is None:
            self.driving = False
            return
        if not self.pad.attached():
            self._lost("no longer attached")
            self.driving = False
            return
        axes, buttons = self.pad.state()
        if n:
            self.last_input = now
            self.inputs.append((now, n))
        while self.inputs and now - self.inputs[0][0] > 1.0:
            self.inputs.popleft()
        self._watch_still(now, n, axes, buttons)
        self.axes, self.buttons = axes, buttons
        estop = buttons[ESTOP]
        if estop and not self.estop_down:
            self._estop_pending = True
            self._latch("E-STOP pressed")
        self.estop_down = estop
        dm = buttons[DEADMAN]
        if not dm:
            self.latched = None
        elif not self.deadman:
            if not self.host_ok():
                self.log("pad", f"dead-man ignored: host {self.host() or '(none)'} is not "
                         f"loopback -- the pad drives the sim only until measured "
                         f"(design §7); {self.labels[1]} = E-STOP still works")
        self.deadman = dm
        if dm and self.streams and self.latched is None and self.silent():
            gap = (now - self.last_input) * 1000.0
            self._latch(f"silent pad: no report for {gap:.0f} ms")
            self.log("pad", f"{self.pad.name}: no report for {gap:.0f} ms while it streams "
                     f"-- released (out of range or flat?); let go of {self.labels[0]}")
        self.gated = dm and not self.host_ok()
        self.driving = dm and self.latched is None and not self.gated

    def host_ok(self) -> bool:
        return not SIM_ONLY or is_loopback(self.host())

    def _latch(self, why: str) -> None:
        if self.deadman or self.buttons.get(DEADMAN):
            if self.latched is None:
                self.latched = why
            if self.driving:
                self._release_pending = why
        self.driving = False

    def silent(self) -> bool:
        """Silent for longer than the timeout (meaningful only if it streams)."""
        if self.pad is None or self.last_input is None:
            return False
        return (self.clock() - self.last_input) * 1000.0 > self.silent_ms

    def _watch_still(self, now, n, axes, buttons) -> None:
        """Does this pad stream while held still? Decided over windows of
        STILL_S in which no button changes and no axis moves by STILL_EPS:
        yes when inputs kept arriving with every gap under half the timeout.
        Once yes, it stays yes for this connection."""
        w = self._still
        moved = (w is None or buttons != w["buttons"]
                 or any(abs(axes[k] - w["axes"][k]) > STILL_EPS for k in AXES))
        if moved:
            self._still = dict(axes=dict(axes), buttons=dict(buttons), t0=now,
                               last=now, n=0, gap=0.0)
            return
        if n:
            w["gap"] = max(w["gap"], now - w["last"])
            w["last"] = now
            w["n"] += n
        span = now - w["t0"]
        if span < STILL_S:
            return
        gap = max(w["gap"], now - w["last"])
        self.still_rate, self.still_gap = w["n"] / span, gap
        if w["n"] and gap * 1000.0 < self.silent_ms / 2.0 and not self.streams:
            self.streams = True
            self.log("pad", f"{self.pad.name} streams {self.still_rate:.0f} inputs/s while "
                     f"held still (longest gap {gap * 1000:.0f} ms): the silent-pad check is "
                     f"ON, {self.silent_ms:.0f} ms")
        w.update(t0=now, last=now, n=0, gap=0.0)

    def command(self):
        """(vx, vy, wz) the sticks ask for, signed, in the trained envelope."""
        dz = self.dead_zone
        return (vx_from_stick(-self.axes["lefty"], dz),
                vy_from_stick(-self.axes["leftx"], dz),
                stick_frac(-self.axes["rightx"], dz) * protocol.W_MAX)

    def axes_held(self):
        """None unless the pad drives; else 6 bools in client.h axis order
        (forward, back, strafe_left, strafe_right, turn_left, turn_right)."""
        if not self.driving:
            return None
        vx, vy, wz = self.command()
        return [vx > 0, vx < 0, vy > 0, vy < 0, wz > 0, wz < 0]

    def _sliders(self, ctl):
        if self.slider_speeds is not None:
            return tuple(self.slider_speeds())
        st = getattr(ctl, "st", None)
        if st is not None:
            return (st.speed_vx, st.speed_vy, st.speed_wz)
        return (ctl.o.vx, ctl.o.vy, ctl.o.wz)

    def apply(self, ctl) -> None:
        """After the app pressed / released the held axes: the stop, the
        release, and the speeds (stick magnitudes while driving; the sliders'
        when it stops)."""
        if self._estop_pending:
            self._estop_pending = False
            ctl.estop()
            self.log("pad", f"E-STOP from the pad ({self.labels[1]}, east): hard stop sent "
                     f"out of band")
        if self._release_pending is not None:
            why, self._release_pending = self._release_pending, None
            if self.app is not None:
                self.app.release_everything(why)
            else:
                ctl.release_all()
        if self.driving:
            if not self._was_driving:
                self._restore = self._sliders(ctl)
                self.log("pad", f"{self.labels[0]} held: the gamepad drives, keys ignored")
            vx, vy, wz = self.command()
            ctl.set_speeds(abs(vx), abs(vy), abs(wz))
        elif self._was_driving:
            speeds = (tuple(self.slider_speeds()) if self.slider_speeds is not None
                      else self._restore)
            if speeds is not None:
                ctl.set_speeds(*speeds)
            self.log("pad", "gamepad stopped driving: speeds back to the sliders")
        self._was_driving = self.driving

    def release(self) -> None:
        """The window lost focus: stop driving until RB is pressed afresh."""
        self._latch("window lost focus")
        self._release_pending = None      # the app is releasing everything itself

    def close(self) -> None:
        if self.pad is not None:
            self.pad.close()
            self.pad = None
        if self.backend is not None:
            self.backend.quit()
            self.backend = None

    # -- what the window shows -------------------------------------------------
    def rate(self) -> float:
        return float(sum(n for _, n in self.inputs))

    def loss_text(self) -> str:
        if self.streams:
            return f"silence > {self.silent_ms:.0f} ms releases"
        return "lost = OS disconnect only"

    def status(self):
        """(text, colour, sub) for the safety bar."""
        if self.backend is None:
            return "PAD  n/a", DIM, self.error or "no SDL"
        if self.pad is None:
            if self.lost_at is not None and self.clock() - self.lost_at < LOST_SHOWN_S:
                return "PAD  LOST", RED, f"released · {self.lost_why}"
            return "PAD  none", DIM, "keyboard / mouse"
        rb, b = self.labels
        if self.gated:
            return "PAD  sim only", AMBER, f"sim only until measured (design §7) · {b} = E-STOP"
        if self.deadman and self.latched:
            return "PAD  released", AMBER, f"{self.latched}: let go of {rb}"
        if self.driving:
            return f"PAD ● {rb} held", GREEN, f"drives · {b} (east) = E-STOP · {self.loss_text()}"
        if self.streams and self.silent():
            return "PAD  silent", AMBER, f"no report {(self.clock() - self.last_input):.1f} s"
        hint = "sim only" if not self.host_ok() else f"hold {rb} to drive"
        return "PAD  ready", BRIGHT, f"{hint} · {b} = E-STOP · {self.loss_text()}"

    def detail(self) -> str:
        """The Drive tab's text."""
        if self.backend is None:
            return f"gamepad unavailable ({self.error})"
        if self.pad is None:
            others = [n for _, n, ok in self.seen if not ok]
            s = "no game controller"
            if others:
                s += (f" ({len(others)} joystick(s) without an SDL GameController mapping: "
                      f"{', '.join(others)}; mapping by position needs one)")
            if self.lost_at is not None:
                s += f" · last one lost {self.clock() - self.lost_at:.0f} s ago: {self.lost_why}"
            return s
        rb, b = self.labels
        if self.streams:
            loss = (f"pad loss: silence over {self.silent_ms:.0f} ms with {rb} held releases "
                    f"(streams {self.still_rate:.0f} inputs/s held still)")
        else:
            loss = ("pad loss: only the OS disconnect counts (not seen streaming while held "
                    "still, so silence means nothing)")
        host = self.host() or "(none)"
        gate = (f"host {host}: drives the sim" if self.host_ok() else
                f"host {host} is not loopback: {rb} ignored, sim only until measured "
                f"(design §7); {b} = E-STOP works")
        p = self.pad.power()
        return (f"{self.pad.name}" + (f" · battery {p}" if p and p != "unknown" else "")
                + f" · {loss}\n{gate}")

    def title(self) -> str:
        if self.pad is None:
            return "no controller" if self.backend is not None else "unavailable"
        p = self.pad.power()
        return f"{self.pad.name} · connected" + (f" · battery {p}" if p and p != "unknown" else "")

    # -- the widgets -------------------------------------------------------------
    def install(self, app) -> None:
        """Point the pad at the console (its link host, the Drive sliders) and
        put its widgets in the Drive and Setup tabs' gamepad boxes."""
        from . import pad_widgets
        self.app = app
        self.host = lambda: app.ctl.host
        drive = app.tab_widgets.get("Drive")
        if drive is not None and all(hasattr(drive, k) for k in ("svx", "svy", "swz")):
            self.slider_speeds = lambda: (drive.svx.value(), drive.svy.value(),
                                          drive.swz.value())
        if drive is not None and hasattr(drive, "gamepad_box"):
            drive.gamepad_box.insertWidget(0, pad_widgets.PadView(self))
        setup = app.tab_widgets.get("Setup")
        if setup is not None and hasattr(setup, "gamepad_box"):
            setup.gamepad_box.addWidget(pad_widgets.PadSetup(self))
