"""The Qt window: safety bar, view pane, tabs (design §4).

Threads: the link tick (a QTimer at the send rate), the beacon and flight
sockets, the gamepad poll and the 3D render all run on the GUI thread, so a
frozen window sends nothing and the robot stands (design §3). Only a stop
burst (stops.py) and the MJPEG readers have threads of their own.

Input rules (bimo_gui's, each from a fixed bug):
  * a held key or button is a LEVEL; releasing it stands on the next frame,
    and auto-repeat changes nothing;
  * losing window focus releases everything;
  * a focused text field gets the keys (typing a host name must not arm),
    and the safety bar says so for as long as it has them;
  * focus enters a text field only when the operator clicks it: a tab
    change, Tab, or the window coming back never puts it there (#234);
    Escape or a click anywhere outside the field gives the keys back;
  * while the robot is LIVE (this console armed, or a fresh LIVE beacon),
    HOLD and E-STOP fire from their keys even inside a text field;
  * a key-up always reaches the held-key state, whatever has focus, so a
    motion key cannot stay held (#235);
  * buttons never take focus (a focused button would eat Space = STAND);
  * window close and quit disarm (Controller.shutdown).
"""
from __future__ import annotations

import importlib
import math
import os
import signal
import stat
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets

from . import core, headless
from .controller import Controller, EventLog
from .safety_bar import SafetyBar
from .tab_drive import History
from .ui import AMBER, DIM, GREEN, QSS, dark_palette, is_text_input, lab, vbox, wrap

# (title, module, class): a module that is not there yet shows a placeholder.
TABS = [("Checks", "tab_checks", "ChecksTab"),
        ("Drive", "tab_drive", "DriveTab"),
        ("Joints", "tab_joints", "JointsTab"),
        ("Telemetry", "tab_telemetry", "TelemetryTab"),
        ("Sim", "tab_sim", "SimTab"),
        ("Session", "tab_session", "SessionTab"),
        ("Setup", "tab_setup", "SetupTab"),
        ("Bench", "tab_bench", "BenchTab")]
DEFAULT_TAB = "Drive"

_QT_NAMED = {
    QtCore.Qt.Key.Key_Space: "Space", QtCore.Qt.Key.Key_Return: "Enter",
    QtCore.Qt.Key.Key_Enter: "Enter", QtCore.Qt.Key.Key_Tab: "Tab",
    QtCore.Qt.Key.Key_Escape: "Escape", QtCore.Qt.Key.Key_Backspace: "Backspace",
    QtCore.Qt.Key.Key_Up: "Up", QtCore.Qt.Key.Key_Down: "Down",
    QtCore.Qt.Key.Key_Left: "Left", QtCore.Qt.Key.Key_Right: "Right",
    QtCore.Qt.Key.Key_PageUp: "PageUp", QtCore.Qt.Key.Key_PageDown: "PageDown",
    QtCore.Qt.Key.Key_Home: "Home", QtCore.Qt.Key.Key_End: "End",
    QtCore.Qt.Key.Key_Insert: "Insert", QtCore.Qt.Key.Key_Delete: "Delete",
}


_K = QtCore.Qt.Key
_NAMED = {k.value: v for k, v in _QT_NAMED.items()}


def key_code(ev: QtGui.QKeyEvent) -> int:
    """A Qt key as the keymap's own code (link/keymap.h), 0 if none."""
    k = int(ev.key())
    if _K.Key_A.value <= k <= _K.Key_Z.value:
        return ord("a") + (k - _K.Key_A.value)
    if _K.Key_0.value <= k <= _K.Key_9.value:
        return ord("0") + (k - _K.Key_0.value)
    if _K.Key_F1.value <= k <= _K.Key_F12.value:
        return core.key_from_name(f"F{k - _K.Key_F1.value + 1}")
    name = _NAMED.get(k)
    return core.key_from_name(name) if name else 0


class Placeholder(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        self.setLayout(vbox(lab(f"{title}: not built yet", DIM, 14), "stretch",
                            margins=(16, 16, 16, 16)))

    def update_from(self, st, now):
        pass


class Window(QtWidgets.QMainWindow):
    def __init__(self, opts, ctl: Controller):
        super().__init__()
        self.o = opts
        self.ctl = ctl
        self.rate_hz = opts.rate
        self.key_held = [False] * core.NUM_AXES
        self.mouse_held = [False] * core.NUM_AXES
        self.asked = [False] * core.NUM_AXES     # a press was asked for (maybe refused)
        self.rebind_action = None
        self._closed = False
        self.message = ""
        self._note = self._status = ""
        self.rec_started = None
        self.beacons = History(("t", "vbat", "tilt", "vx_cmd", "vx_est", "wz_cmd",
                                "wz_est", "late"))
        self._seen = 0
        self.flight = None        # tab_joints / flight.py: the flight-record store
        self.pad = None           # gamepad.py
        self.checks = None        # tab_checks
        self.view = None
        self._plugins()
        self._title = None
        self.update_title()
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.bar = SafetyBar(self)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.tab_widgets = {}
        for title, mod, cls in TABS:
            if title == "Bench" and not (opts.bench_telemetry or opts.cam_relay):
                continue
            w = self._make(mod, cls, title)
            self.tab_widgets[title] = w
            # A tab bigger than its share of the screen scrolls; it never
            # makes the window bigger than the screen.
            sa = QtWidgets.QScrollArea()
            sa.setWidgetResizable(True)
            sa.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
            sa.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            sa.setWidget(w)
            self.tabs.addTab(sa, title)
        if self.pad is not None:
            self.pad.install(self)
        # A text field takes focus from a click only: never from Tab, from a
        # tab change (QStackedLayout hands focus to the new page's first
        # Tab-focusable child) or from window activation. eventFilter's
        # FocusIn guard backs this up for anything that still tries.
        for f in self.findChildren(QtWidgets.QWidget):
            if is_text_input(f):
                f.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
        self.split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.split.setHandleWidth(6)
        self.split.addWidget(self.view)
        self.split.addWidget(wrap(vbox(self.tabs, margins=(4, 8, 12, 4))))
        lay = vbox(self.bar, self.split, spacing=0)
        lay.setStretchFactor(self.split, 1)
        root.setLayout(lay)
        self.setCentralWidget(root)
        self.statusBar().showMessage(f"log {ctl.log.path or '(none)'}")
        self.fit_to_screen()
        bench = "Bench" in self.tab_widgets and not opts.drive_view   # bimo_gui's bench view
        self.select_tab(opts.tab or ("Bench" if bench else DEFAULT_TAB))

        self.link_timer = QtCore.QTimer(self)
        self.link_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.link_timer.timeout.connect(self.tick)
        self.link_timer.start(round(1000.0 / self.rate_hz))
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(66)
        QtWidgets.QApplication.instance().applicationStateChanged.connect(self._app_state)
        self._stdin()
        if opts.screenshot:
            QtCore.QTimer.singleShot(int(opts.screenshot_after * 1000),
                                     lambda: (self.screenshot(opts.screenshot), self.close()))
        if opts.quit_after:
            QtCore.QTimer.singleShot(int(opts.quit_after * 1000), self.close)

    # -- assembly ----------------------------------------------------------
    def _plugins(self):
        try:
            from .flight import FlightStore
            self.flight = FlightStore(self.o, self.ctl.log)
        except ImportError:
            self.flight = None
        try:
            from .gamepad import Gamepad
            self.pad = Gamepad(self.ctl.log)
        except ImportError:
            self.pad = None
        try:
            from .view3d import ViewPane
            self.view = ViewPane(self)
        except ImportError:
            self.view = Placeholder("3D view")

    def _make(self, mod, cls, title):
        try:
            m = importlib.import_module(f".{mod}", __package__)
        except ImportError as e:
            if e.name and e.name.endswith(mod):
                return Placeholder(title)
            raise
        return getattr(m, cls)(self)

    def fit_to_screen(self) -> None:
        """Size the window to the screen it opens on, never larger: an
        operator console whose E-STOP is off the bottom of a laptop screen
        is not one. The view pane gets about 40 % of the width."""
        scr = self.screen() or QtWidgets.QApplication.primaryScreen()
        avail = scr.availableGeometry() if scr is not None else QtCore.QRect(0, 0, 1920, 1080)
        w = min(2560, avail.width())
        h = min(1440, avail.height() - 32)          # the title bar
        self.resize(w, h)
        self.move(avail.x() + (avail.width() - w) // 2, avail.y())
        view = max(380, min(760, int(w * 0.38)))
        self.split.setSizes([view, w - view])

    def select_tab(self, name: str) -> bool:
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i).lower() == name.lower():
                self.tabs.setCurrentIndex(i)
                return True
        return False

    # -- the link tick -----------------------------------------------------
    def held_now(self):
        pad = self.pad.axes_held() if self.pad is not None else None
        if pad is not None:
            return pad          # the dead-man is held: the pad drives, keys are ignored
        return [k or m for k, m in zip(self.key_held, self.mouse_held)]

    def drive_source(self) -> str:
        if self.pad is not None and self.pad.axes_held() is not None:
            return "gamepad (dead-man held)"
        if any(self.mouse_held):
            return "mouse"
        return "keyboard"

    def _apply_inputs(self):
        want = self.held_now()
        st = self.ctl.st
        for a in range(core.NUM_AXES):
            if want[a] and not self.asked[a]:
                self.asked[a] = True
                self.ctl.press(a)            # may refuse, and says why
            elif not want[a] and self.asked[a]:
                self.asked[a] = False
                if st is None or st.held[a]:
                    self.ctl.release(a)
        if self.pad is not None:
            self.pad.apply(self.ctl)

    def tick(self):
        if self.pad is not None:
            self.pad.poll()
        self._apply_inputs()
        now = core.now_ms()
        st = self.ctl.tick(now)
        if self.flight is not None:
            self.flight.poll(now)
        if st.tlm_count != self._seen:
            self._seen = st.tlm_count
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, st.up_z))))
            self.beacons.push(now / 1000.0, st.vbat_v, tilt, st.vx, st.vx_est, st.wz,
                              st.wz_est, st.loop_late_pct)
        if st.rec_active and self.rec_started is None:
            self.rec_started = time.monotonic()
        elif not st.rec_active:
            self.rec_started = None
        if self.ctl.core.note != self._note:
            self._note = self.message = self.ctl.core.note
        if self.ctl.status_msg != self._status:
            self._status = self.message = self.ctl.status_msg
        self.update_title()
        if self.ctl.quit:
            self.close()

    def update_title(self) -> None:
        """The robot this window commands or watches, kept current across
        relinks; a failed relink says so rather than naming the old robot."""
        c = self.ctl
        t = ("bimo console — NO LINK" if not getattr(c, "linked", True)
             else f"bimo console — {c.host or 'no host'}")
        if t != self._title:
            self._title = t
            self.setWindowTitle(t)

    def set_rate(self, hz: float) -> None:
        self.rate_hz = max(5.0, min(100.0, hz))
        self.link_timer.setInterval(round(1000.0 / self.rate_hz))

    def mouse_hold(self, axis: int, on: bool) -> None:
        self.mouse_held[axis] = on

    def release_everything(self, why: str) -> None:
        if self._closed:
            return              # shutdown already disarmed and freed the core
        self.key_held = [False] * core.NUM_AXES
        self.mouse_held = [False] * core.NUM_AXES
        if self.pad is not None:
            self.pad.release()
        self.ctl.release_all()
        self.asked = [False] * core.NUM_AXES
        self.ctl.log("focus", why)

    # -- the screen refresh --------------------------------------------------
    def refresh(self):
        st = self.ctl.st
        if st is None:
            return
        now = core.now_ms()
        self.bar.update_from(st, now)
        if hasattr(self.view, "update_from"):
            self.view.update_from(st, now)
        w = self.tab_widgets.get(self.tabs.tabText(self.tabs.currentIndex()))
        if hasattr(w, "update_from"):
            w.update_from(st, now)
        for title, tw in self.tab_widgets.items():
            if tw is not w and getattr(tw, "always_update", False):
                tw.update_from(st, now)

    # -- providers for the bar and the tabs ---------------------------------
    def checks_open(self) -> bool:
        return self.checks.open_rows() > 0 if self.checks is not None else False

    def pad_status(self):
        if self.pad is None:
            return "PAD  none", DIM, "keyboard / mouse"
        return self.pad.status()

    def pad_detail(self) -> str:
        return self.pad.detail() if self.pad is not None else "no gamepad support (pygame)"

    def flight_line(self) -> str:
        return self.flight.line() if self.flight is not None else ""

    def flight_detail(self) -> str:
        return self.flight.detail() if self.flight is not None else "flight records: off"

    def robot_events(self):
        return self.flight.events() if self.flight is not None else []

    def policy_saw(self):
        return self.flight.policy_cmd() if self.flight is not None else None

    def plant_path(self) -> str:
        return getattr(self.view, "plant", "") or "(3D view not built)"

    def stop_dest_text(self) -> str:
        from .stops import stop_host, targets
        host, src = stop_host(self.o.stop_host or self.ctl.host)
        t = targets(host, self.o.cmd_port, None if self.o.no_broadcast else "255.255.255.255")
        return (", ".join(f"{h}:{p}" for h, p in t) + f"  ({src})") if t else "nowhere"

    def keymap_changed(self):
        self.bar.keymap_changed()
        setup = self.tab_widgets.get("Setup")
        if setup is not None:
            setup.refresh_keys()

    def screenshot(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        ok = self.grab().save(path)
        self.ctl.say_status(f"screenshot -> {path}" if ok else f"screenshot FAILED: {path}")

    # -- keyboard and focus --------------------------------------------------
    def text_focus(self):
        """The text field that has the keys, or None."""
        f = QtWidgets.QApplication.focusWidget()
        return f if is_text_input(f) else None

    def stops_win(self) -> bool:
        """HOLD and E-STOP fire from their keys even inside a text field
        while the robot is LIVE: this console armed it, or a fresh beacon
        says LIVE (an observer watching another console drive). Not
        otherwise: a stray E-STOP drops a standing robot (torque off), so
        `e` typed into a host name on a disarmed robot must stay a letter."""
        st = self.ctl.st
        return st is not None and bool(st.armed or (st.fresh and st.state == core.LIVE))

    def give_keys_back(self, why: str) -> None:
        f = self.text_focus()
        if f is not None:
            f.clearFocus()
            self.ctl.log("focus", f"keys back to the console: {why}")

    def _focus_in(self, obj, ev) -> None:
        """Undo focus that entered a text field by any route but a click."""
        if not is_text_input(obj) or ev.reason() == QtCore.Qt.FocusReason.MouseFocusReason:
            return

        def undo():
            if obj.hasFocus():
                obj.clearFocus()
        QtCore.QTimer.singleShot(0, undo)

    def _click(self, obj) -> None:
        """A click outside the focused text field gives the keys back (Qt
        would leave them in the field: buttons here never take focus)."""
        f = self.text_focus()
        if f is None or not isinstance(obj, QtWidgets.QWidget) or obj.window() is not self:
            return
        if obj is f or f.isAncestorOf(obj) or is_text_input(obj):
            return
        self.give_keys_back("click outside the field")

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == QtCore.QEvent.Type.FocusIn:
            self._focus_in(obj, ev)
            return False
        if t == QtCore.QEvent.Type.MouseButtonPress:
            self._click(obj)
            return False
        if t not in (QtCore.QEvent.Type.KeyPress, QtCore.QEvent.Type.KeyRelease):
            return False
        if not isinstance(obj, QtGui.QWindow):
            return False                         # each key once, at the window
        code = key_code(ev)
        press = t == QtCore.QEvent.Type.KeyPress
        if self.rebind_action is not None and press:
            if ev.key() != QtCore.Qt.Key.Key_Escape and code:
                self.ctl.core.bind(self.rebind_action, code)
            self.rebind_action = None
            self.keymap_changed()
            return True
        action = self.ctl.core.action_for(code) if code else -1
        motion = 0 <= action < core.motion_actions()
        # A key-up ends a held motion whatever has focus (#235): the press
        # went to key_held, so the release must too, or the robot keeps
        # walking with nothing pressed.
        if not press and motion and not ev.isAutoRepeat() and self.key_held[action]:
            self.key_held[action] = False
            return True
        field = self.text_focus()
        if field is not None:
            if press and ev.key() == QtCore.Qt.Key.Key_Escape:
                self.give_keys_back("Escape")
                return True
            if (action >= 0 and core.action_name(action) in ("hold", "estop")
                    and self.stops_win()):
                if press and not ev.isAutoRepeat():
                    self.ctl.fire(core.action_name(action))
                return True                      # a stop key never types while LIVE
            return False                         # the field gets the key
        if action < 0:
            return False
        if ev.isAutoRepeat():
            return True
        if motion:
            self.key_held[action] = press
        elif press:
            self.ctl.fire(core.action_name(action))
        return True

    def _app_state(self, state):
        if state != QtCore.Qt.ApplicationState.ApplicationActive:
            self.release_everything("window lost focus: every key released")

    def changeEvent(self, ev):
        if ev.type() == QtCore.QEvent.Type.ActivationChange and not self.isActiveWindow():
            if any(self.key_held) or any(self.mouse_held):
                self.release_everything("window deactivated: every key released")
        super().changeEvent(ev)

    # -- stdin (scripted figures and tests) ----------------------------------
    def _stdin(self):
        self._buf = b""
        self._sn = None
        try:
            mode = os.fstat(sys.stdin.fileno()).st_mode
        except (OSError, ValueError):
            return
        if not stat.S_ISFIFO(mode):
            return                  # only a pipe is a script; a tty or /dev/null is not
        self._sn = QtCore.QSocketNotifier(sys.stdin.fileno(),
                                          QtCore.QSocketNotifier.Type.Read, self)
        self._sn.activated.connect(self._read_stdin)

    def _read_stdin(self, *_):
        chunk = os.read(sys.stdin.fileno(), 4096)
        if not chunk:
            self._sn.setEnabled(False)
            self.close()            # EOF: the operator went away; leaving disarms
            return
        self._buf += chunk
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            words = line.decode(errors="replace").split()
            if words and words[0] in ("tab", "panel") and len(words) > 1:
                self.select_tab(words[1])
            elif words and words[0] == "shot" and len(words) > 1:
                self.screenshot(words[1])
            else:
                headless.command(self.ctl, " ".join(words))

    def closeEvent(self, ev):
        # The core is freed below: a key reaching this filter, or a
        # deactivation releasing keys, would hand the library a null handle.
        QtWidgets.QApplication.instance().removeEventFilter(self)
        self._closed = True
        self.link_timer.stop()
        self.ui_timer.stop()
        # The disarm first: nothing a gamepad, a stream or the flight store
        # does on its way out may stand between the robot and its last frames.
        try:
            self.ctl.shutdown()
        finally:
            for name, fn in (("gamepad", getattr(self.pad, "close", None)),
                             ("3D view", getattr(self.view, "close_streams", None)),
                             ("flight records", getattr(self.flight, "close", None))):
                if fn is None:
                    continue
                try:
                    fn()
                except Exception as e:  # noqa: BLE001 -- closing; say it, carry on
                    print(f"close {name}: {e}", file=sys.stderr, flush=True)
            super().closeEvent(ev)


def run(opts) -> int:
    os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("bimo console")
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    log = EventLog(os.path.join(opts.sessions, "bimo_console.log"),
                   echo=lambda s: print(s, file=sys.stderr, flush=True))
    ctl = Controller(opts, log)
    if not ctl.start():
        log.close()
        return 2
    win = Window(opts, ctl)
    app.installEventFilter(win)
    # SIGTERM / Ctrl-C close the window like the close button, so a driver
    # still sends its disarm frames. Python only runs a signal handler
    # between bytecodes, so a timer keeps it running inside Qt's loop.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: QtCore.QTimer.singleShot(0, win.close))
    wake = QtCore.QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(200)
    win.show()
    rc = app.exec()
    log.close()
    return rc
