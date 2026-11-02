"""The Checks tab (link/console/tab_checks.py): docs/first-policy-stand.md
§2's go/no-go rows turn PASS / FAIL from synthetic beacons, open_rows() is
what the ARM label reads, the abort watch shows §5's conditions with live
values, and none of it ever sends anything (design §6, §15: label ARM, never
block it; the abort watch is advisory).
"""
import os
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt6")
pytest.importorskip("pyqtgraph")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from PyQt6 import QtWidgets  # noqa: E402

from console import core  # noqa: E402
import protocol  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(core.DEFAULT_LIB),
                                reason="build it: make -C firmware/host console")
AR = protocol.ArmResult


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class Untouchable:
    """A controller nobody may touch: the Checks tab must never act."""

    def __getattr__(self, name):
        raise AssertionError(f"the Checks tab touched the controller: .{name}")


def fake_app(dest="192.168.2.40:4210  (--host)", flight=None, events=None):
    app = SimpleNamespace(ctl=Untouchable(), flight=flight, checks=None)
    app.events = events if events is not None else []
    app.stop_dest_text = lambda: dest
    app.robot_events = lambda: app.events
    return app


def beacon(state=core.BENCH, result=AR.NONE, cal=True, vbat=11.9, up=(0.0, 0.0, 1.0),
           att=True, servo_err=0, t_us=None, rec=True):
    st = core.Status()
    st.have_tlm = 1
    st.link_lost = 0
    st.state = state
    st.diag = protocol.pack_diag(False, cal, result)
    st.vbat_v = vbat
    st.up_x, st.up_y, st.up_z = up
    st.have_att = int(att)
    st.servo_err = servo_err
    st.t_us = int(time.time() * 1e6) if t_us is None else t_us
    st.tlm_age_ms = 40.0
    st.rec_active = int(rec)
    st.rec_cmd, st.rec_tlm = 120, 60
    return st


def tab(qapp, **kw):
    from console.tab_checks import ChecksTab
    app = fake_app(**kw)
    t = ChecksTab(app)
    assert app.checks is t
    return t, app


def status(t, num, n=0):
    r = t.row(num, n)
    return r.status, r.value


def test_auto_rows_read_the_beacon(qapp):
    t, _ = tab(qapp)
    t.update_from(beacon(), 1000.0)
    assert status(t, "2")[0] == "PASS" and "11.90 V" in status(t, "2")[1]
    assert status(t, "3")[0] == "PASS"
    assert status(t, "5") == ("PASS", "no fault bits")
    assert status(t, "10b")[0] == "PASS" and "192.168.2.40" in status(t, "10b")[1]
    assert status(t, "11")[0] == "PASS" and "synced" in status(t, "11")[1]
    assert status(t, "13")[0] == "PASS"
    assert status(t, "—")[0] == "PASS"                      # console recording
    t.update_from(beacon(vbat=11.2, cal=False, servo_err=(1 << 3) | (1 << 11), t_us=0,
                         up=(0.2, 0.0, 0.98), rec=False), 1100.0)
    assert status(t, "2")[0] == "FAIL"
    assert status(t, "3")[0] == "FAIL" and "cal" in status(t, "3")[1]
    s, v = status(t, "5")
    assert s == "FAIL" and "faulted" in v
    assert status(t, "11")[0] == "FAIL" and "t_us = 0" in status(t, "11")[1]
    assert status(t, "13")[0] == "FAIL"                     # 11.5 deg off vertical
    assert status(t, "—")[0] == "FAIL"
    t.update_from(beacon(state=core.VLAND), 1200.0)
    assert status(t, "2")[0] == "FAIL" and "pack guard" in status(t, "2")[1]
    t.update_from(beacon(up=(0.0, 0.0, 0.5)), 1300.0)       # |up| = 0.5: not an IMU reading
    assert status(t, "13")[0] == "FAIL" and "|up|" in status(t, "13")[1]
    st = beacon()
    st.have_tlm = 0                                         # an observer with no driver
    t.update_from(st, 1400.0)
    assert status(t, "2")[0] == "NO DATA" and status(t, "5")[0] == "NO DATA"


def test_a_stop_needs_somewhere_to_go(qapp):
    t, _ = tab(qapp, dest="nowhere")
    t.update_from(beacon(), 1000.0)
    assert status(t, "10b")[0] == "FAIL"


def test_flight_hello_adds_calibration_fitted_and_rec(qapp):
    hello = dict(cal_ok=1, fitted=[1] * 12 + [0] * 5, enabled=1, utc_synced=1)
    flight = SimpleNamespace(hello=lambda: hello)
    t, _ = tab(qapp, flight=flight)
    t.update_from(beacon(), 1000.0)
    assert status(t, "3")[0] == "PASS" and "12 of 12 legs" in status(t, "3")[1]
    assert status(t, "11", 1)[0] == "PASS"                  # robot rec, via the HELLO
    hello["fitted"] = [1] * 11 + [0] * 6
    t.update_from(beacon(), 1100.0)
    assert status(t, "3")[0] == "FAIL" and "11 of 12" in status(t, "3")[1]
    hello["enabled"] = 0
    t.update_from(beacon(), 1200.0)
    assert status(t, "11", 1)[0] == "FAIL"


def test_home_readback_and_the_arm_window(qapp):
    t, _ = tab(qapp)
    t.update_from(beacon(result=AR.DISARMED_HOME), 1000.0)   # already there at start
    assert status(t, "12")[0] == "WAIT" and "age unknown" in status(t, "12")[1]
    t.update_from(beacon(result=AR.HOME_PENDING), 2000.0)
    assert status(t, "12")[0] == "WAIT" and "slewing" in status(t, "12")[1]
    t.update_from(beacon(result=AR.DISARMED_HOME), 3000.0)   # the readback, seen live
    assert status(t, "12")[0] == "PASS" and t.arm_left_s == pytest.approx(30.0)
    t.update_from(beacon(result=AR.DISARMED_HOME), 15000.0)
    assert status(t, "12")[0] == "PASS" and t.arm_left_s == pytest.approx(18.0)
    t.update_from(beacon(result=AR.DISARMED_HOME), 34000.0)
    assert status(t, "12")[0] == "FAIL" and "home` again" in status(t, "12")[1]
    assert t.arm_left_s is None
    t.update_from(beacon(result=AR.HOME_NOT_REACHED), 35000.0)
    assert status(t, "12")[0] == "FAIL"
    # The tether's `home` prints its readback; the flight recorder carries it.
    t2, app2 = tab(qapp)
    app2.events = [(time.time() - 5.0, "out",
                    "home readback: all 12 joints at their zeros -- torque released")]
    t2.update_from(beacon(), 50000.0)
    assert status(t2, "12")[0] == "PASS" and t2.arm_left_s == pytest.approx(25.0, abs=0.5)


def test_ticks_open_rows_and_a_refusal_clears_a_tick(qapp):
    t, _ = tab(qapp)
    t.update_from(beacon(result=AR.DISARMED_HOME), 0.0)
    t.update_from(beacon(result=AR.HOME_PENDING), 500.0)
    t.update_from(beacon(result=AR.DISARMED_HOME), 1000.0)
    ticks = [r for r in t.rows if r.tick]
    assert t.open_rows() == len(ticks)                      # every auto row passes
    for i, cb in t.boxes.items():
        cb.setChecked(True)
    assert t.open_rows() == 0
    t.render()
    assert t.pill.text() == "GO"
    t.update_from(beacon(result=AR.REFUSED_GAINS), 2000.0)  # the robot says otherwise
    gains = t.row("4")
    assert gains.status == "FAIL" and not gains.ticked
    assert t.open_rows() >= 2                               # gains, and the readback is gone
    t.render()
    assert t.pill.text() == "NO-GO"
    t.untick_all()
    assert all(not r.ticked for r in t.rows)


def test_abort_watch_values(qapp):
    t, app = tab(qapp)
    t.update_from(beacon(state=core.LIVE), 0.0)
    assert t.watch["tilt"][0] == "ok" and t.watch["fault"] == ("ok", "none")
    lean = (0.208, 0.0, 0.978)                               # 12 deg
    t.update_from(beacon(state=core.LIVE, up=lean), 1000.0)
    assert t.watch["tilt"][0] == "warn"
    t.update_from(beacon(state=core.LIVE, up=lean), 2100.0)  # 1.1 s and not back
    level, text = t.watch["tilt"]
    assert level == "abort" and "not coming back" in text
    t.update_from(beacon(state=core.LIVE), 2200.0)
    assert t.watch["tilt"][0] == "ok"
    t.update_from(beacon(state=core.LIVE, servo_err=1 << 11, vbat=10.3), 2300.0)
    assert t.watch["fault"] == ("abort", f"id {core.fault_ids(1 << 11)} faulted")
    assert t.watch["batt"][0] == "abort" and "LOW-BATT" in t.watch["batt"][1]
    t.update_from(beacon(state=core.FALLEN), 2400.0)
    assert t.watch["fallen"][0] == "abort"
    t.update_from(beacon(result=AR.DISARMED_FLAIL), 2500.0)  # benched by the flail guard
    assert t.watch["flail"][0] == "abort" and "TRIPPED" in t.watch["flail"][1]
    t.update_from(beacon(result=AR.DISARMED_STOP), 2600.0)
    assert t.watch["stop"][0] == "abort" and t.watch["flail"] == ("ok", "not tripped")
    app.events = [(time.time() - 3.0, "flail", "rate 1.9 rad/s")]
    t.update_from(beacon(state=core.LIVE), 2700.0)
    assert t.watch["flail"][0] == "abort" and "rate 1.9" in t.watch["flail"][1]
    for k in ("foot", "drift"):
        assert t.watch[k][0] == "na"                          # eyes only: no odometry
    t.render()


def test_the_tab_never_sends(qapp, tmp_path):
    """Through the real window and core: a driving console, its link tick
    stopped, fed every alarming beacon -- no frame, no stop burst."""
    from console import options
    from console.app import Window
    from console.controller import Controller
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", "4999", "--tlm-port", "4998",
                       "--no-record", "--sessions", str(tmp_path), "--keymap",
                       str(tmp_path / "keys.conf"), "--drive"])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    try:
        w.link_timer.stop()
        checks = w.tab_widgets["Checks"]
        assert w.checks is checks and w.checks_open()
        sent = ctl.core.status(core.now_ms()).sent
        now = core.now_ms()
        for i, st in enumerate([beacon(state=core.LIVE, up=(0.5, 0.0, 0.866)),
                                beacon(state=core.FALLEN), beacon(result=AR.DISARMED_FLAIL),
                                beacon(state=core.LIVE, servo_err=0xFFFF, vbat=9.5),
                                beacon(state=core.VSAFE)] * 10):
            checks.update_from(st, now + 200.0 * i)
        assert ctl.core.status(core.now_ms()).sent == sent
        assert not ctl.stops and ctl.last_stop is None
        w.bar.update_from(beacon(), core.now_ms())
        assert "checks open" in w.bar.arm.text()
    finally:
        w.close()


def test_window_checks_screenshot(qapp, tmp_path):
    """The tab against the link twin after a RESET SERVOS (CONSOLE_SHOTS=dir
    saves a picture)."""
    from console import options
    from console.app import Window
    from console.controller import Controller
    from console.ui import QSS, dark_palette
    port = 4400 + (os.getpid() % 150) * 2
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "link", "link_twin.py"), "--port",
                          str(port), "--tlm-port", str(port + 1), "--duration", "30"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.4)
    shots = os.environ.get("CONSOLE_SHOTS")
    if shots:
        qapp.setStyle("Fusion")
        qapp.setPalette(dark_palette())
        qapp.setStyleSheet(QSS)
    o = options.parse(["--host", "127.0.0.1", "--cmd-port", str(port), "--tlm-port",
                       str(port + 1), "--sessions", str(tmp_path), "--keymap",
                       str(tmp_path / "keys.conf"), "--drive", "--tab", "checks"])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    w.show()
    try:
        def pump(s):
            end = time.monotonic() + s
            while time.monotonic() < end:
                qapp.processEvents()
                time.sleep(0.01)
        pump(0.8)
        ctl.fire("home")
        pump(2.0)
        checks = w.checks
        assert checks.row("12").status == "PASS", checks.row("12").value
        assert checks.row("10b").status == "PASS"
        for i, cb in list(checks.boxes.items())[:5]:
            cb.setChecked(True)
        pump(0.3)
        assert "checks open" in w.bar.arm.text()
        if shots:
            w.grab().save(os.path.join(shots, "console_checks.png"))
    finally:
        w.close()
        p.terminate()
        p.communicate(timeout=5)
