"""The console's gamepad (link/console/gamepad.py) against a fake SDL layer:
no hardware, no pygame devices, a hand-wound clock.

What is pinned is design §7's "Gamepad" and "The Bluetooth hazard": the
dead-man releases on the next poll, a lost pad releases everything, the pad
drives only a loopback (sim) host while its E-STOP fires for any host, vx
goes through the trained band, the slider speeds come back, and the silent-pad
timeout is used only for a pad seen streaming while held still. The last test
drives the link twin through the real window.
"""
import os
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("pygame")
pytest.importorskip("PyQt6")
pytest.importorskip("pyqtgraph")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from console import gamepad  # noqa: E402
import protocol  # noqa: E402
import sources  # noqa: E402

FWD, BACK, LEFT, RIGHT, TURN_L, TURN_R = range(6)


class FakePad:
    def __init__(self, name="Xbox Wireless Controller", iid=7):
        self.name, self.instance_id = name, iid
        self.guid = "050000005e040000e002000030110000"
        self.mapping = "a:b0,b:b1,rightshoulder:b7,leftx:a0,lefty:a1,rightx:a2"
        self.axes = {k: 0.0 for k in gamepad.AXES}
        self.buttons = {k: False for k in gamepad.BUTTONS}
        self.is_attached = True
        self.closed = False

    def attached(self):
        return self.is_attached

    def state(self):
        return dict(self.axes), dict(self.buttons)

    def power(self):
        return "full"

    def close(self):
        self.closed = True


class FakeSdl:
    sdl = "2.32.74"

    def __init__(self, *pads):
        self.pads = list(pads)
        self.queue = []

    def scan(self):
        return [(i, p.name, True) for i, p in enumerate(self.pads)]

    def open(self, index):
        return self.pads[index]

    def events(self):
        q, self.queue = self.queue, []
        return q

    def quit(self):
        pass

    # the test's hands
    def input(self, pad, n=1):
        self.queue += [("input", pad.instance_id)] * n

    def remove(self, pad):
        self.pads.remove(pad)
        self.queue.append(("removed", pad.instance_id))


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeCtl:
    """What apply() may call, recorded."""

    def __init__(self, host="127.0.0.1"):
        self.host = host
        self.calls = []
        self.speeds = (0.4, 0.2, 0.5)
        self.st = SimpleNamespace(speed_vx=0.4, speed_vy=0.2, speed_wz=0.5)
        self.o = SimpleNamespace(vx=0.4, vy=0.2, wz=0.5)

    def estop(self):
        self.calls.append("estop")

    def release_all(self):
        self.calls.append("release_all")

    def set_speeds(self, vx, vy, wz):
        self.calls.append(("set_speeds", vx, vy, wz))
        self.speeds = (vx, vy, wz)


class Log:
    def __init__(self):
        self.lines = []

    def __call__(self, kind, msg):
        self.lines.append((kind, msg))

    def text(self):
        return "\n".join(m for _, m in self.lines)


def rig(host="127.0.0.1", pad=None, **kw):
    pad = pad or FakePad()
    sdl = FakeSdl(pad)
    clock = Clock()
    log = Log()
    g = gamepad.Gamepad(log, backend=sdl, clock=clock, **kw)
    ctl = FakeCtl(host)
    g.host = lambda: ctl.host
    return g, pad, sdl, clock, ctl, log


def tick(g, ctl, clock, dt=0.05):
    clock.t += dt
    g.poll()
    held = g.axes_held()
    g.apply(ctl)
    return held


def test_dead_man_release_gives_none_on_the_next_poll():
    g, pad, sdl, clock, ctl, log = rig()
    assert tick(g, ctl, clock) is None                       # connected, RB not held
    pad.buttons["rightshoulder"] = True
    pad.axes["lefty"] = -0.8                                 # stick up = forward
    sdl.input(pad, 2)
    held = tick(g, ctl, clock)
    assert held == [True, False, False, False, False, False]
    assert ctl.speeds[0] == pytest.approx(sources.stick_to_speed(0.8))
    pad.buttons["rightshoulder"] = False                     # let go: the stick stays up
    sdl.input(pad)
    assert tick(g, ctl, clock) is None
    assert ctl.speeds == (0.4, 0.2, 0.5)                     # the sliders' speeds are back
    assert "keys ignored" in log.text() and "back to the sliders" in log.text()


def test_signs_map_to_the_held_axes():
    g, pad, sdl, clock, ctl, _ = rig()
    pad.buttons["rightshoulder"] = True
    pad.axes.update(lefty=0.9, leftx=-0.7, rightx=0.6)     # back, strafe left, turn right
    held = tick(g, ctl, clock)
    assert held == [False, True, True, False, False, True]
    vx, vy, wz = ctl.speeds
    assert vx == pytest.approx(-sources.stick_to_speed(-0.9))     # the backward band
    assert vy == pytest.approx(sources.stick_to_lateral(0.7))
    assert wz == pytest.approx(gamepad.stick_frac(0.6, gamepad.DEAD_ZONE) * protocol.W_MAX)
    pad.axes.update(lefty=0.05, leftx=0.0, rightx=-0.1)    # inside the dead zone: stand
    assert tick(g, ctl, clock) == [False] * 6               # still the pad's: keys ignored


def test_rb_held_at_connect_or_after_focus_loss_needs_a_fresh_press():
    pad = FakePad()
    pad.buttons["rightshoulder"] = True
    g, pad, sdl, clock, ctl, _ = rig(pad=pad)
    assert tick(g, ctl, clock) is None
    assert g.status()[0] == "PAD  released"
    pad.buttons["rightshoulder"] = False
    tick(g, ctl, clock)
    pad.buttons["rightshoulder"] = True
    assert tick(g, ctl, clock) is not None
    g.release()                                              # the window lost focus
    assert g.axes_held() is None
    assert tick(g, ctl, clock) is None                       # RB still down: stays released
    pad.buttons["rightshoulder"] = False
    tick(g, ctl, clock)
    pad.buttons["rightshoulder"] = True
    assert tick(g, ctl, clock) is not None


def test_disconnect_releases_everything_and_says_so():
    g, pad, sdl, clock, ctl, log = rig()
    app = SimpleNamespace(released=[])
    app.release_everything = lambda why: app.released.append(why)
    g.app = app
    pad.buttons["rightshoulder"] = True
    pad.axes["rightx"] = -1.0
    assert tick(g, ctl, clock)[TURN_L]
    sdl.remove(pad)
    assert tick(g, ctl, clock) is None
    assert app.released and "gamepad lost" in app.released[0]
    assert pad.closed and g.pad is None
    assert "every input released" in log.text()
    assert ctl.speeds == (0.4, 0.2, 0.5)
    text, colour, sub = g.status()
    assert text == "PAD  LOST" and colour == gamepad.RED
    # SDL saying "not attached" without an event counts the same.
    g2, pad2, sdl2, clock2, ctl2, log2 = rig()
    pad2.buttons["rightshoulder"] = True
    assert tick(g2, ctl2, clock2) is not None
    pad2.is_attached = False
    assert tick(g2, ctl2, clock2) is None
    assert "release_all" in ctl2.calls and "no longer attached" in log2.text()


def test_non_loopback_host_is_sim_only_but_estop_fires():
    g, pad, sdl, clock, ctl, log = rig(host="192.168.2.40")
    pad.buttons["rightshoulder"] = True
    pad.axes["lefty"] = -1.0
    assert tick(g, ctl, clock) is None                       # the dead-man is ignored
    assert not any(c[0] == "set_speeds" for c in ctl.calls if isinstance(c, tuple))
    text, colour, sub = g.status()
    assert text == "PAD  sim only" and "sim only until measured (design §7)" in sub
    assert "not loopback" in log.text()
    pad.buttons["b"] = True                                  # east: E-STOP
    tick(g, ctl, clock)
    tick(g, ctl, clock)                                      # held: one stop per press
    assert ctl.calls.count("estop") == 1
    pad.buttons["b"] = False
    tick(g, ctl, clock)
    pad.buttons["b"] = True
    tick(g, ctl, clock)
    assert ctl.calls.count("estop") == 2
    # A relink to the sim while RB is still held does not start driving.
    ctl.host = "127.0.0.1"
    assert tick(g, ctl, clock) is None
    for h in ("127.0.0.1", "127.3.2.1"):
        assert gamepad.is_loopback(h)
    for h in ("192.168.2.40", "0.0.0.0", "localhost", "", None, "::1"):
        assert not gamepad.is_loopback(h)


def test_estop_while_driving_stops_and_latches():
    g, pad, sdl, clock, ctl, _ = rig()
    pad.buttons["rightshoulder"] = True
    pad.axes["lefty"] = -1.0
    assert tick(g, ctl, clock)[FWD]
    pad.buttons["b"] = True
    assert tick(g, ctl, clock) is None
    assert ctl.calls.count("estop") == 1
    pad.buttons["b"] = False
    assert tick(g, ctl, clock) is None                       # RB still held: latched
    pad.buttons["rightshoulder"] = False
    tick(g, ctl, clock)
    pad.buttons["rightshoulder"] = True
    assert tick(g, ctl, clock)[FWD]


def test_vx_goes_through_the_trained_band_of_its_direction():
    for a in (-1.0, -0.6, -0.16, -0.1, 0.0, 0.1, 0.15, 0.3, 0.5, 0.99, 1.0):
        assert gamepad.vx_from_stick(a, sources.STICK_DEADZONE) == pytest.approx(
            sources.stick_to_speed(a))
    for dz in (0.08, 0.25):
        for i in range(-100, 101):
            v = gamepad.vx_from_stick(i / 100.0, dz)
            if v > 0.0:
                assert protocol.V_FWD_MIN - 1e-9 <= v <= protocol.V_FWD_MAX + 1e-9
            elif v < 0.0:
                assert protocol.V_BACK_MIN - 1e-9 <= -v <= protocol.V_BACK_MAX + 1e-9
            # the robot's own clamp leaves every stick speed as it is
            assert protocol.clamp_to_envelope(v, 0.0)[0] == pytest.approx(v)
        assert gamepad.vx_from_stick(dz, dz) == pytest.approx(protocol.V_FWD_MIN)
        assert gamepad.vx_from_stick(-dz, dz) == pytest.approx(-protocol.V_BACK_MIN)
        assert gamepad.vx_from_stick(dz - 0.01, dz) == 0.0
        assert gamepad.vx_from_stick(1.0, dz) == pytest.approx(protocol.V_FWD_MAX)
        assert gamepad.vx_from_stick(-1.0, dz) == pytest.approx(-protocol.V_BACK_MAX)


def test_vy_goes_through_the_trained_sidestep_band():
    for a in (-1.0, -0.6, -0.16, -0.1, 0.0, 0.1, 0.15, 0.3, 0.5, 0.99, 1.0):
        assert gamepad.vy_from_stick(a, sources.STICK_DEADZONE) == pytest.approx(
            sources.stick_to_lateral(a))
    for dz in (0.08, 0.25):
        for i in range(-100, 101):
            v = gamepad.vy_from_stick(i / 100.0, dz)
            assert v == 0.0 or protocol.VY_MIN - 1e-9 <= abs(v) <= protocol.VY_MAX + 1e-9
        assert gamepad.vy_from_stick(dz, dz) == pytest.approx(protocol.VY_MIN)
        assert gamepad.vy_from_stick(dz - 0.01, dz) == 0.0
        assert gamepad.vy_from_stick(-1.0, dz) == pytest.approx(-protocol.VY_MAX)


def test_speed_restore_follows_the_sliders():
    g, pad, sdl, clock, ctl, _ = rig()
    sliders = [0.4, 0.2, 0.5]
    g.slider_speeds = lambda: tuple(sliders)
    pad.buttons["rightshoulder"] = True
    pad.axes["lefty"] = -0.5
    tick(g, ctl, clock)
    sliders[0] = 0.7                                         # moved while the pad drove
    pad.buttons["rightshoulder"] = False
    tick(g, ctl, clock)
    assert ctl.speeds == (0.7, 0.2, 0.5)
    # Without the Drive tab: the speeds as they were when the pad took over.
    g2, pad2, _, clock2, ctl2, _ = rig()
    ctl2.st = SimpleNamespace(speed_vx=0.33, speed_vy=0.11, speed_wz=0.44)
    pad2.buttons["rightshoulder"] = True
    pad2.axes["rightx"] = 0.9
    tick(g2, ctl2, clock2)
    assert ctl2.speeds != (0.33, 0.11, 0.44)
    pad2.buttons["rightshoulder"] = False
    tick(g2, ctl2, clock2)
    assert ctl2.speeds == pytest.approx((0.33, 0.11, 0.44))


def test_silence_means_nothing_for_a_pad_that_does_not_stream():
    g, pad, sdl, clock, ctl, log = rig()
    pad.buttons["rightshoulder"] = True
    sdl.input(pad)
    for _ in range(80):                                      # 4 s held still, no reports
        assert tick(g, ctl, clock) is not None
    assert not g.streams
    assert "OS disconnect only" in g.status()[2]
    assert "only the OS disconnect counts" in g.detail()


def test_silent_pad_timeout_only_once_streaming_was_seen():
    g, pad, sdl, clock, ctl, log = rig(silent_ms=150.0)
    for _ in range(30):                                      # 1.5 s still, reporting each tick
        sdl.input(pad, 5)
        pad.axes["leftx"] = 0.004 if pad.axes["leftx"] == 0.0 else 0.0   # stick noise
        tick(g, ctl, clock)
    assert g.streams and "silent-pad check is ON" in log.text()
    assert "silence > 150 ms releases" in g.status()[2]
    pad.buttons["rightshoulder"] = True
    sdl.input(pad)
    assert tick(g, ctl, clock) is not None
    for _ in range(2):                                       # 100 ms of silence: still fine
        assert tick(g, ctl, clock) is not None
    assert tick(g, ctl, clock, dt=0.06) is None              # 160 ms: released
    assert "no report for" in log.text()
    assert "release_all" in ctl.calls
    sdl.input(pad, 3)                                        # reports resume, RB still down
    assert tick(g, ctl, clock) is None
    assert g.status()[0] == "PAD  released"


def test_a_pad_moving_is_not_called_streaming():
    g, pad, sdl, clock, ctl, _ = rig()
    for i in range(60):                                      # sticks moving all the time
        pad.axes["lefty"] = (-1) ** i * 0.5
        sdl.input(pad, 5)
        tick(g, ctl, clock)
    assert not g.streams


def test_a_pad_without_sdl_is_reported_not_raised():
    class Broken:
        def __init__(self):
            raise gamepad.pygame.error("no joystick subsystem")

    real = gamepad.SdlPads
    gamepad.SdlPads = Broken
    try:
        log = Log()
        g = gamepad.Gamepad(log)
    finally:
        gamepad.SdlPads = real
    g.poll()
    assert g.axes_held() is None
    assert g.status()[0] == "PAD  n/a" and "no joystick subsystem" in log.text()


# -- through the window, against the link twin -----------------------------------
@pytest.fixture(scope="module")
def qapp():
    from PyQt6 import QtWidgets
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def twin():
    port = 4600 + (os.getpid() % 150) * 2
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "link", "link_twin.py"), "--port",
                          str(port), "--tlm-port", str(port + 1), "--duration", "30"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.4)
    yield port
    p.terminate()
    p.communicate(timeout=5)


def pump(qapp, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        qapp.processEvents()
        time.sleep(0.01)


def test_the_pad_drives_the_twin_and_releases(qapp, twin, tmp_path, monkeypatch):
    from console import core, options
    if not os.path.exists(core.DEFAULT_LIB):
        pytest.skip("build it: make -C firmware/host console")
    from console.app import Window
    from console.controller import Controller
    pad = FakePad()
    sdl = FakeSdl(pad)
    monkeypatch.setattr(gamepad, "SdlPads", lambda: sdl)
    shots = os.environ.get("CONSOLE_SHOTS")
    if shots:
        from console.ui import QSS, dark_palette
        qapp.setStyle("Fusion")
        qapp.setPalette(dark_palette())
        qapp.setStyleSheet(QSS)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(twin), "--tlm-port",
                       str(twin + 1), "--no-record", "--sessions", str(tmp_path), "--keymap",
                       str(tmp_path / "keys.conf"), "--drive"])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    qapp.installEventFilter(w)
    w.show()
    try:
        assert w.pad is not None and w.pad.backend is sdl
        from console.pad_widgets import PadSetup, PadView
        assert w.tab_widgets["Drive"].findChildren(PadView)
        assert w.tab_widgets["Setup"].findChildren(PadSetup)
        pump(qapp, 0.6)
        ctl.fire("arm")
        pump(qapp, 1.6)
        assert ctl.st.armed and "LIVE" in w.bar.badge.text()
        pad.buttons["rightshoulder"] = True
        pad.axes["lefty"] = -1.0
        sdl.input(pad)
        pump(qapp, 0.3)
        assert w.drive_source().startswith("gamepad")
        assert ctl.st.held[FWD] and ctl.st.vx == pytest.approx(protocol.V_FWD_MAX, abs=1e-3)
        w.select_tab("Drive")
        pump(qapp, 0.3)
        if shots:
            w.grab().save(os.path.join(shots, "console_drive_pad.png"))
            w.select_tab("Setup")
            pump(qapp, 0.3)
            w.grab().save(os.path.join(shots, "console_setup_pad.png"))
            w.select_tab("Drive")
        pad.buttons["rightshoulder"] = False                 # let go: stands next frame
        sdl.input(pad)
        pump(qapp, 0.15)
        assert not any(ctl.st.held) and ctl.st.vx == 0.0
        assert ctl.st.speed_vx == pytest.approx(w.tab_widgets["Drive"].svx.value())
        pad.buttons["rightshoulder"] = True                  # drive again, then lose the pad
        sdl.input(pad)
        pump(qapp, 0.2)
        assert ctl.st.held[FWD]
        sdl.remove(pad)
        pump(qapp, 0.15)
        assert not any(ctl.st.held) and ctl.st.vx == 0.0
        assert w.pad.status()[0] == "PAD  LOST"
    finally:
        w.close()
