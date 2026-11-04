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
        setup.host.setFocus()
        pump(qapp, 0.1)
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
