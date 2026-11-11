"""The Qt console's window, offscreen (QT_QPA_PLATFORM=offscreen): it builds,
its keys follow bimo_gui's rules, and it drives the link twin.

Needs PyQt6 + pyqtgraph (requirements.txt, the console's optional block) and
the core library (make -C firmware/host console); skips without them.
"""
import os
import subprocess
import sys
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt6")
pytest.importorskip("pyqtgraph")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: E402

from console import core, mjpeg, options  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(core.DEFAULT_LIB),
                                reason="build it: make -C firmware/host console")
TWIN = os.path.join(ROOT, "link", "link_twin.py")


@pytest.fixture(scope="module")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


@pytest.fixture
def twin():
    port = 4500 + (os.getpid() % 200) * 2
    p = subprocess.Popen([sys.executable, TWIN, "--port", str(port), "--tlm-port",
                          str(port + 1), "--duration", "30"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.4)
    yield port
    p.terminate()
    p.communicate(timeout=5)


def window(qapp, port, tmp_path, *extra):
    from console.app import Window
    from console.controller import Controller
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                       str(port + 1), "--no-record", "--sessions", str(tmp_path),
                       "--keymap", str(tmp_path / "keys.conf"), *extra])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    qapp.installEventFilter(w)
    w.show()
    return w


def pump(qapp, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        qapp.processEvents()
        time.sleep(0.01)


def key(w, qkey, press=True, auto=False, text=""):
    t = QtCore.QEvent.Type.KeyPress if press else QtCore.QEvent.Type.KeyRelease
    ev = QtGui.QKeyEvent(t, qkey, QtCore.Qt.KeyboardModifier.NoModifier, text, auto)
    QtWidgets.QApplication.sendEvent(w.windowHandle(), ev)


def click_into(qapp, w, field):
    """Focus a text field the way the operator does: a click."""
    w.activateWindow()
    pump(qapp, 0.05)
    field.setFocus(QtCore.Qt.FocusReason.MouseFocusReason)
    pump(qapp, 0.1)


def focused():
    return QtWidgets.QApplication.focusWidget()


def record_fires(w):
    """Stand-in for Controller.fire that only records: the routing is under
    test here, not the stop bursts (which would go out on the LAN)."""
    fired = []
    w.ctl.fire = lambda name: fired.append(name) or True
    return fired


def test_every_tab_builds_and_draws(qapp, twin, tmp_path):
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        pump(qapp, 1.0)
        names = [w.tabs.tabText(i) for i in range(w.tabs.count())]
        assert names[:7] == ["Checks", "Drive", "Joints", "Telemetry", "Sim", "Session", "Setup"]
        for i in range(w.tabs.count()):
            w.tabs.setCurrentIndex(i)
            pump(qapp, 0.15)
        img = w.grab()
        assert img.width() > 800 and img.height() > 500
        assert "LIVE" not in w.bar.badge.text()        # benched until armed
    finally:
        w.close()


def test_keys_follow_the_dead_man_rules(qapp, twin, tmp_path):
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        pump(qapp, 0.8)
        key(w, QtCore.Qt.Key.Key_Up)                    # not armed: refused, said so
        pump(qapp, 0.2)
        assert "not armed" in w.ctl.core.note
        key(w, QtCore.Qt.Key.Key_Up, press=False)
        key(w, QtCore.Qt.Key.Key_A, text="a")           # arm
        key(w, QtCore.Qt.Key.Key_A, press=False, text="a")
        pump(qapp, 1.8)
        assert w.ctl.st.armed and "LIVE" in w.bar.badge.text()
        key(w, QtCore.Qt.Key.Key_Up)
        pump(qapp, 0.3)
        assert w.ctl.st.held[0] and w.ctl.st.vx > 0.3
        key(w, QtCore.Qt.Key.Key_Up, auto=True)         # auto-repeat changes nothing
        pump(qapp, 0.1)
        assert w.ctl.st.held[0]
        w.release_everything("test: focus lost")        # what deactivation calls
        pump(qapp, 0.15)
        assert not w.ctl.st.held[0] and w.ctl.st.vx == 0.0
        # A focused text field takes the keys: typing must not disarm.
        setup = w.tab_widgets["Setup"]
        w.select_tab("Setup")
        click_into(qapp, w, setup.host)
        if QtWidgets.QApplication.focusWidget() is setup.host:
            key(w, QtCore.Qt.Key.Key_A, text="a")
            pump(qapp, 0.1)
            assert w.ctl.st.armed
        w.select_tab("Drive")
    finally:
        w.close()


def test_an_observer_window_greys_what_it_cannot_do(qapp, twin, tmp_path):
    w = window(qapp, twin, tmp_path)                    # the default: observer
    try:
        pump(qapp, 0.5)
        assert w.ctl.st.readonly
        assert not w.bar.arm.isEnabled() and not w.bar.reset.isEnabled()
        assert w.bar.hold.isEnabled() and w.bar.estop.isEnabled()
        assert w.bar.control.text() == "TAKE CONTROL" and w.bar.control.isEnabled()
    finally:
        w.close()


def test_mjpeg_parser_takes_parts_split_anywhere():
    jpeg = b"\xff\xd8" + b"x" * 100 + b"\xff\xd9"
    stream = (b"HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=f\r\n\r\n"
              + b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg)
              + jpeg + b"\r\n")
    for cut in (1, 7, 50, len(stream) - 3):
        p = mjpeg.Parser()
        p.feed(stream[:cut])
        p.feed(stream[cut:])
        assert p.take() == jpeg
        assert p.take() is None
    p = mjpeg.Parser()
    p.feed(b"--f\r\nContent-Length: 999999999\r\n\r\n")
    assert p.overflow and p.take() is None


def test_the_window_fits_a_laptop_screen(qapp, twin, tmp_path):
    """Everything that must be seen -- the safety bar above all -- fits a
    1440 x 860 pt laptop area; a tab bigger than its share scrolls instead of
    growing the window off the screen."""
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        pump(qapp, 0.3)
        bar = w.bar.minimumSizeHint()
        assert bar.width() <= 1440, bar
        whole = w.centralWidget().minimumSizeHint()
        assert whole.width() <= 1440 and whole.height() <= 860, whole
        for i in range(w.tabs.count()):
            assert isinstance(w.tabs.widget(i), QtWidgets.QScrollArea), w.tabs.tabText(i)
        scr = w.screen().availableGeometry()
        assert w.width() <= max(scr.width(), whole.width())
    finally:
        w.close()


def test_setup_and_joints_fit_the_pane_at_1440x900(qapp, twin, tmp_path):
    """#241: at the 1440 x 900 pt window and the default split, Setup and
    Joints are no wider than the pane they sit in: no horizontal scrollbar."""
    from console.ui import QSS, dark_palette
    qapp.setStyle("Fusion")                  # the look the console runs with: its
    qapp.setPalette(dark_palette())          # padding and fonts set the widths
    qapp.setStyleSheet(QSS)
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        w.resize(1440, 900)
        w.split.setSizes([547, 893])         # the default split: 38 % to the view
        for name in ("Setup", "Joints"):
            w.select_tab(name)
            pump(qapp, 0.3)
            sa = w.tabs.currentWidget()
            assert sa.horizontalScrollBar().maximum() == 0, (name, sa.widget().size(),
                                                              sa.viewport().size())
            assert sa.widget().minimumSizeHint().width() <= sa.viewport().width(), name
    finally:
        w.close()


def test_sigterm_closes_the_window_like_the_close_button(tmp_path):
    """SDL (pygame) would take SIGTERM for itself; the console must still
    close on it, through the window's own close path."""
    import signal
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "link", "bimo_console.py"),
                          "--host", "127.0.0.1", "--cmd-port", "4890", "--tlm-port", "4891",
                          "--no-record", "--sessions", str(tmp_path), "--keymap",
                          str(tmp_path / "keys.conf")],
                         stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, env=env)
    try:
        time.sleep(4.0)
        assert p.poll() is None, p.communicate()[0]
        p.send_signal(signal.SIGTERM)
        out = p.communicate(timeout=8)[0]
    except subprocess.TimeoutExpired:
        p.kill()
        out = p.communicate()[0]
        raise AssertionError("SIGTERM did not close the window\n" + out)
    assert "quit" in out and "observer: sent nothing" in out, out


def test_no_tab_puts_focus_in_a_text_field_and_the_stops_fire_on_every_tab(qapp, twin, tmp_path):
    """#234: selecting Session or Setup moved focus into a text field, and
    HOLD / E-STOP / every key went into it instead, silently."""
    from console.ui import is_text_input
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        w.activateWindow()
        pump(qapp, 0.5)
        assert w.isActiveWindow()
        fired = record_fires(w)
        setup, session = w.tab_widgets["Setup"], w.tab_widgets["Session"]
        host0 = setup.host.text()
        # Twice round, the second time after the operator had clicked into
        # each field: Qt restores a page's last focus child on return.
        for lap in range(2):
            for i in range(w.tabs.count()):
                w.tabs.setCurrentIndex(i)
                pump(qapp, 0.1)
                name = w.tabs.tabText(i)
                assert not is_text_input(focused()), (lap, name, focused())
                fired.clear()
                for qkey, ch in ((QtCore.Qt.Key.Key_F, "f"), (QtCore.Qt.Key.Key_E, "e")):
                    key(w, qkey, text=ch)
                    key(w, qkey, press=False, text=ch)
                pump(qapp, 0.05)
                assert fired == ["hold", "estop"], (lap, name, fired)
                if name in ("Setup", "Session"):
                    click_into(qapp, w, setup.host if name == "Setup" else session.text)
            assert setup.host.text() == host0 and session.text.text() == ""
        # Tab (the key) does not walk focus into a field either.
        w.select_tab("Setup")
        pump(qapp, 0.1)
        key(w, QtCore.Qt.Key.Key_Tab)
        key(w, QtCore.Qt.Key.Key_Tab, press=False)
        pump(qapp, 0.1)
        assert not is_text_input(focused())
    finally:
        w.close()


def test_a_focused_text_field_is_shown_and_gives_the_keys_back(qapp, twin, tmp_path):
    """While a field has the keys the bar says so; Escape or a click outside
    gives them back; while the robot is LIVE, HOLD and E-STOP fire from
    inside the field and do not type."""
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        w.activateWindow()
        pump(qapp, 0.8)
        setup = w.tab_widgets["Setup"]
        w.select_tab("Setup")
        click_into(qapp, w, setup.host)
        assert focused() is setup.host
        w.refresh()
        assert "KEYS CAPTURED" in w.bar.note.text()
        fired = record_fires(w)
        setup.host.setText("")
        key(w, QtCore.Qt.Key.Key_E, text="e")           # disarmed: a letter, not a stop
        key(w, QtCore.Qt.Key.Key_E, press=False, text="e")
        pump(qapp, 0.05)
        assert setup.host.text() == "e" and fired == []
        key(w, QtCore.Qt.Key.Key_Escape)                # Escape gives the keys back
        key(w, QtCore.Qt.Key.Key_Escape, press=False)
        pump(qapp, 0.05)
        assert focused() is not setup.host
        w.refresh()
        assert "KEYS CAPTURED" not in w.bar.note.text()
        key(w, QtCore.Qt.Key.Key_E, text="e")
        pump(qapp, 0.05)
        assert fired == ["estop"] and setup.host.text() == "e"
        # A click outside the field (on a label: it takes no focus) too.
        from PyQt6 import QtTest
        click_into(qapp, w, setup.host)
        assert focused() is setup.host
        QtTest.QTest.mouseClick(w.bar.badge, QtCore.Qt.MouseButton.LeftButton)
        pump(qapp, 0.05)
        assert focused() is not setup.host
        # Armed (LIVE): the stop keys fire from inside the field, untyped.
        del w.ctl.fire                                  # the real one, to arm
        setup.host.setText("127.0.0.1")
        key(w, QtCore.Qt.Key.Key_A, text="a")
        key(w, QtCore.Qt.Key.Key_A, press=False, text="a")
        pump(qapp, 1.8)
        assert w.ctl.st.armed
        fired = record_fires(w)
        click_into(qapp, w, setup.host)
        assert focused() is setup.host
        w.refresh()
        assert "still fire" in w.bar.note.text()
        for qkey, ch in ((QtCore.Qt.Key.Key_F, "f"), (QtCore.Qt.Key.Key_E, "e")):
            key(w, qkey, text=ch)
            key(w, qkey, press=False, text=ch)
        key(w, QtCore.Qt.Key.Key_1, text="1")           # other keys still type
        pump(qapp, 0.05)
        assert fired == ["hold", "estop"]
        assert setup.host.text() == "127.0.0.11"
    finally:
        w.close()


def test_a_motion_key_released_in_a_text_field_stops_the_robot(qapp, twin, tmp_path):
    """#235: the key-up was handed to the field, key_held stuck, and the
    console kept commanding motion with nothing pressed."""
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        w.activateWindow()
        pump(qapp, 0.8)
        key(w, QtCore.Qt.Key.Key_A, text="a")
        key(w, QtCore.Qt.Key.Key_A, press=False, text="a")
        pump(qapp, 1.8)
        assert w.ctl.st.armed
        key(w, QtCore.Qt.Key.Key_Up)
        pump(qapp, 0.3)
        assert w.key_held[0] and w.ctl.st.held[0] and w.ctl.st.vx > 0.3
        setup = w.tab_widgets["Setup"]
        w.select_tab("Setup")                           # held through the tab change...
        click_into(qapp, w, setup.host)                 # ...and a click into a field
        assert focused() is setup.host
        assert w.key_held[0]                            # still physically held
        key(w, QtCore.Qt.Key.Key_Up, press=False)
        pump(qapp, 0.3)
        assert not w.key_held[0]
        assert not w.ctl.st.held[0] and w.ctl.st.vx == 0.0
        w.select_tab("Drive")
        pump(qapp, 0.3)
        assert not w.key_held[0] and w.ctl.st.vx == 0.0
    finally:
        w.close()


def test_closing_with_a_key_held_does_not_touch_the_freed_core(qapp, twin, tmp_path):
    """Close frees the core; the deactivation that follows (a key still
    held) and any later key must not reach it -- that was a segfault."""
    w = window(qapp, twin, tmp_path, "--drive")
    w.activateWindow()
    pump(qapp, 0.3)
    key(w, QtCore.Qt.Key.Key_Up)
    assert w.key_held[0]
    w.close()
    pump(qapp, 0.2)
    other = window(qapp, twin, tmp_path, "--drive")
    try:
        pump(qapp, 0.2)
        key(other, QtCore.Qt.Key.Key_Escape)            # unbound: passes every filter
        pump(qapp, 0.1)
    finally:
        other.close()


def test_a_pending_rebind_never_swallows_a_stop(qapp, twin, tmp_path):
    """A Setup keymap row waiting for its key: the HOLD and E-STOP keys still
    fire their stop and stay the stops'; Escape, a tab change, focus loss and
    5 s cancel it; and nothing is rebound while armed or LIVE."""
    from console import app as app_mod
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        w.activateWindow()
        pump(qapp, 0.5)
        fired = record_fires(w)
        c = w.ctl.core
        estop_key = c.key_for(core.action_from_name("estop"))
        hold_key = c.key_for(core.action_from_name("hold"))
        fwd = core.action_from_name("forward")
        w.select_tab("Setup")
        pump(qapp, 0.1)
        for qkey, ch, name in ((QtCore.Qt.Key.Key_E, "e", "estop"),
                               (QtCore.Qt.Key.Key_F, "f", "hold")):
            fired.clear()
            assert w.start_rebind(fwd)
            assert "REBINDING" in w.ctl.status_msg
            key(w, qkey, text=ch)
            key(w, qkey, press=False, text=ch)
            pump(qapp, 0.05)
            assert fired == [name] and w.rebind_action is None
            assert c.key_for(core.action_from_name("estop")) == estop_key
            assert c.key_for(core.action_from_name("hold")) == hold_key
            assert c.key_for(fwd) != estop_key and c.key_for(fwd) != hold_key
        # The stop rows themselves are not rebound here.
        assert not w.start_rebind(core.action_from_name("estop"))
        assert "refused" in w.ctl.status_msg
        # An ordinary rebind works, and Escape / a tab change / the timeout cancel.
        assert w.start_rebind(fwd)
        key(w, QtCore.Qt.Key.Key_Y, text="y")
        pump(qapp, 0.05)
        assert core.key_name(c.key_for(fwd)) == "y"
        assert w.start_rebind(fwd)
        key(w, QtCore.Qt.Key.Key_Escape)
        assert w.rebind_action is None and "cancelled" in w.ctl.status_msg
        assert w.start_rebind(fwd)
        w.select_tab("Drive")
        assert w.rebind_action is None
        assert w.start_rebind(fwd)
        w.rebind_until = time.monotonic() - 0.1
        pump(qapp, 0.1)
        assert w.rebind_action is None and "no key in" in w.ctl.status_msg
        assert w.start_rebind(fwd)
        w._app_state(QtCore.Qt.ApplicationState.ApplicationInactive)
        assert w.rebind_action is None
        assert app_mod.REBIND_TIMEOUT_S == 5.0
        # Armed: refused outright.
        w.ctl.core.toggle_arm(core.now_ms())
        pump(qapp, 0.1)
        assert not w.start_rebind(fwd) and "disarm first" in w.ctl.status_msg
    finally:
        w.close()


def test_a_failed_relink_shows_no_link_everywhere(qapp, twin, tmp_path):
    """The title, the link line and Setup's role stop naming the old robot."""
    w = window(qapp, twin, tmp_path, "--drive")
    try:
        pump(qapp, 0.3)
        assert w.windowTitle().endswith("127.0.0.1")
        assert not w.ctl.relink("not-an-address", twin, twin + 1)
        w.select_tab("Setup")
        pump(qapp, 0.3)
        assert w.windowTitle() == "bimo console — NO LINK"
        assert "NO LINK" in w.bar.link.text() and "127.0.0.1:" not in w.bar.link.text()
        assert w.tab_widgets["Setup"].role.text().startswith("NO LINK")
        assert w.ctl.relink("127.0.0.1", twin, twin + 1)
        pump(qapp, 0.3)
        assert w.windowTitle().endswith("127.0.0.1")
    finally:
        w.close()
