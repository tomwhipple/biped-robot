"""The console's flight-record receiver (link/console/flight.py) and the two tabs
it feeds (tab_joints.py, tab_telemetry.py).

Synthetic HELLO / TICK / EVENT packets are built with tools/flightrec.py's own
encoders and sent over loopback UDP to a FlightStore on ephemeral ports:
decoding, steps -> radians against the HELLO's calibration, holes from
sequence gaps, the policy command and its staleness, and latest()'s shape
(the 3D view's contract). The tabs are built offscreen and fed the same
records; they skip without PyQt6 + pyqtgraph.
"""
import math
import os
import socket
import sys
import time

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import flightrec as F  # noqa: E402
from console import flight, options  # noqa: E402

BOOT = 0x5EED0001
ZERO = [2048 + 37 * b for b in range(F.MAX_JOINTS)]
DIR = [1 if b % 3 else -1 for b in range(F.MAX_JOINTS)]
LEG_BUS = list(range(12))          # bus order: the 12 leg joints come first
JF_ALL = F.JF_FB_OK | F.JF_NOM_VALID | F.JF_GOAL_VALID | F.JF_WRITTEN | F.JF_DRIVEN


# -- synthetic records (also used to render the tabs for review) ---------------
def rad_to_steps(b: int, rad: float) -> int:
    return int(round(ZERO[b] + rad / (2 * math.pi / F.STEPS_PER_REV) / DIR[b]))


def hello(seq, boot=BOOT, fitted=LEG_BUS, cal_ok=1, t_boot_us=0, t_utc_us=0):
    m = dict(fw="bimo-test-1", utc_synced=1 if t_utc_us else 0, enabled=1, idle_hz=10,
             cal_ok=cal_ok, produced=seq, sent=seq, ctrl_ticks=seq, ctrl_worst_us=13400,
             us_read=3400, us_imu=180, us_obs=7200, us_net=60, us_write=250, us_other=90,
             uptime_ms=t_boot_us // 1000, id=[b + 1 for b in range(F.MAX_JOINTS)],
             fitted=[1 if b in fitted else 0 for b in range(F.MAX_JOINTS)], dir=DIR, zero=ZERO)
    return F.encode_hello(dict(boot_id=boot, seq=seq, t_boot_us=t_boot_us,
                               t_utc_us=t_utc_us), m)


def tick(seq, t_boot_us, goal, pos=None, boot=BOOT, source=F.SRC_CTRL, cmd=None,
         fault=(), up=(0.0, 0.0, 1.0), load=200, t_utc_us=0, temp=36, status=None,
         gyro=(0.01, -0.02, 0.0), vbat_mv=11850, work_us=11000):
    """goal/pos: {bus: rad}. A bus in `fault` missed this tick's read: its
    position is the held value passed in pos."""
    pos = goal if pos is None else pos
    joints = []
    for b in sorted(goal):
        fl = JF_ALL & ~(F.JF_FB_OK if b in fault else 0)
        g = rad_to_steps(b, goal[b])
        joints.append(dict(bus=b, flags=fl, status=(status or {}).get(b, 0), temp_c=temp,
                           voltage_dv=118, moving=1, nominal=g, goal=g, goal_speed=0,
                           pos=rad_to_steps(b, pos[b]), speed=12, load=load, current=50))
    t = dict(source=source, flags=F.TF_TORQUE | F.TF_IMU_VALID | F.TF_FB_READ | F.TF_WROTE,
             link_state=0, tick=seq, work_us=work_us, gyro=gyro, up=up,
             ctrl=dict(cmd=cmd or [0.2, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0], vbat_mv=vbat_mv),
             joints=joints)
    return F.encode_tick(dict(boot_id=boot, seq=seq, t_boot_us=t_boot_us, t_utc_us=t_utc_us), t)


def event(seq, code, text="", a=0, b=0, boot=BOOT, t_utc_us=0):
    return F.encode_event(dict(boot_id=boot, seq=seq, t_utc_us=t_utc_us),
                          dict(code=code, src=0, a=a, b=b, text=text))


def gait(t: float, amp: float = 1.0, dphase: float = 0.0) -> dict:
    """{bus: rad}: a 1.4 Hz walking-like pattern for the 12 leg joints."""
    s = math.sin(2 * math.pi * 1.4 * t + dphase)
    c = math.cos(2 * math.pi * 1.4 * t + dphase)
    deg = {"L_hip_pitch": -18 * amp * s - 12, "R_hip_pitch": 18 * amp * s - 12,
           "L_knee": -(28 + 18 * amp * max(0.0, c)), "R_knee": -(28 + 18 * amp * max(0.0, -c)),
           "L_ankle": 10 + 8 * amp * s, "R_ankle": 10 - 8 * amp * s,
           "L_hip_roll": 3 * s, "R_hip_roll": 3 * s, "L_ankle_roll": -3 * s,
           "R_ankle_roll": -3 * s, "L_hip_yaw": 4 * s + 3, "R_hip_yaw": -4 * s + 3}
    return {F.BUS_JOINT_NAMES.index(n): math.radians(v) for n, v in deg.items()}


def backfill(store, seconds=60.0, fault_name="R_ankle_roll", fault_at=-1.6, sim=True):
    """Push `seconds` of 50 Hz robot (and sim) records into a store through
    its own ingest path, stamped as if they had arrived over that span."""
    now = time.monotonic()
    wall = time.time()
    rng = np.random.default_rng(7)
    fb = F.BUS_JOINT_NAMES.index(fault_name) if fault_name else None
    for name in ("robot", "sim") if sim else ("robot",):
        s = store.src[name]
        seq = 1
        s.ingest(hello(seq, t_boot_us=1_000_000, t_utc_us=int(wall * 1e6)), ("127.0.0.1", 4220),
                 now - seconds, wall - seconds)
        held = None
        n = int(seconds * 50)
        for k in range(n):
            tr = 1.0 + k * 0.02
            rel = -seconds + k * 0.02
            goal = gait(rel) if name == "robot" else gait(rel, 1.12, 0.35)
            lag = gait(rel - 0.06)
            pos = {b: goal[b] + 0.92 * (lag[b] - goal[b]) + math.radians(rng.normal(0, 0.25))
                   for b in goal}
            fault = ()
            if name == "robot" and fb is not None and rel >= fault_at:
                held = held if held is not None else pos[fb]
                pos[fb] = held
                fault = (fb,)
            if k % 50 == 0:
                seq += 1
                s.ingest(hello(seq, t_boot_us=int(tr * 1e6), t_utc_us=int((wall + rel) * 1e6)),
                         ("127.0.0.1", 4220), now + rel, wall + rel)
            if name == "robot" and k == int((seconds - 34.0) * 50):
                seq += 2                       # two lost records, 34 s ago
            seq += 1
            load = int(180 + 120 * abs(math.sin(2 * math.pi * 1.4 * rel)) + rng.normal(0, 12))
            s.ingest(tick(seq, int(tr * 1e6), goal, pos, fault=fault, load=load,
                          up=(0.02, 0.03 + 0.02 * math.sin(rel * 8.8), 0.999),
                          t_utc_us=int((wall + rel) * 1e6), temp=36 + (k % 7 == 0),
                          gyro=(0.12 * math.sin(8.8 * rel) + rng.normal(0, 0.01), 0.0, 0.0),
                          vbat_mv=int(11900 - 0.6 * (rel + seconds) + rng.normal(0, 8)),
                          work_us=int(11000 + rng.normal(0, 250))),
                     ("127.0.0.1", 4220), now + rel + 0.002, wall + rel)
    return now


# -- a store on loopback -----------------------------------------------------
def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Logged(list):
    def __call__(self, kind, msg):
        self.append((kind, msg))


@pytest.fixture
def store():
    rp, sp = free_port(), free_port()
    o = options.parse(["--flight-port", str(rp), "--sim-flight-port", str(sp)])
    log = Logged()
    st = flight.FlightStore(o, log)
    st.ports = (rp, sp)
    st.logged = log
    st.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yield st
    st.tx.close()
    st.close()


def send(store, *pkts, source="robot"):
    port = store.ports[0 if source == "robot" else 1]
    for p in pkts:
        store.tx.sendto(p, ("127.0.0.1", port))
    want = store.src[source].packets + store.src[source].undecodable + len(pkts)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        store.poll(0.0)
        s = store.src[source]
        if s.packets + s.undecodable >= want:
            return
        time.sleep(0.005)
    raise AssertionError("datagrams did not arrive")


def test_ports_off_bind_nothing():
    o = options.parse(["--flight-port", "0", "--sim-flight-port", "0"])
    st = flight.FlightStore(o, Logged())
    assert st.src["robot"].sock is None and st.src["sim"].sock is None
    assert st.poll(0.0) == 0
    assert "off" in st.line()
    assert st.latest() is None and st.policy_cmd() is None and st.events() == []
    st.close()


def test_hello_and_tick_decode_to_radians(store):
    goal = {b: 0.1 * (b + 1) * (-1) ** b for b in LEG_BUS}
    pos = {b: goal[b] + 0.01 for b in LEG_BUS}
    send(store, hello(1, t_boot_us=900_000), tick(2, 1_000_000, goal, pos))
    lt = store.latest("robot")
    assert set(lt) == {"t", "age_s", "fitted", "up", "joints"}
    assert lt["fitted"] == [F.BUS_JOINT_NAMES[b] for b in LEG_BUS]
    assert lt["up"] == pytest.approx((0.0, 0.0, 1.0))
    assert 0.0 <= lt["age_s"] < 2.0 and lt["t"] <= time.monotonic()
    assert set(lt["joints"]) == set(lt["fitted"])
    for b in LEG_BUS:
        j = lt["joints"][F.BUS_JOINT_NAMES[b]]
        assert set(j) == {"pos", "goal", "nominal", "flags", "fb_ok"}
        assert j["fb_ok"] is True and j["flags"] == JF_ALL
        # exactly flightrec.steps_to_rad of the steps on the wire, HELLO's zero and dir
        assert j["goal"] == pytest.approx(F.steps_to_rad(rad_to_steps(b, goal[b]), ZERO[b], DIR[b]))
        assert j["pos"] == pytest.approx(F.steps_to_rad(rad_to_steps(b, pos[b]), ZERO[b], DIR[b]))
        assert j["pos"] == pytest.approx(pos[b], abs=2 * math.pi / 4096)
        assert j["nominal"] == pytest.approx(j["goal"])
    t, v = store.series("robot", "R_knee.pos", 10.0)
    assert len(t) == 1 and t[0] <= 0.0
    assert v[0] == pytest.approx(lt["joints"]["R_knee"]["pos"])
    _, e = store.series("robot", "R_knee.err", 10.0)
    assert e[0] == pytest.approx(lt["joints"]["R_knee"]["pos"] - lt["joints"]["R_knee"]["goal"])
    _, tilt = store.series("robot", "tilt_deg", 10.0)
    assert tilt[0] == pytest.approx(0.0, abs=1e-3)
    _, vb = store.series("robot", "vbat", 10.0)
    assert vb[0] == pytest.approx(11.85)
    _, ld = store.series("robot", "L_knee.load", 10.0)
    assert ld[0] == pytest.approx(20.0)                # 200 x 0.1 %
    assert store.bus_id("robot", "R_knee") == 3
    fb = store.feedback("robot")["R_knee"]
    assert fb["current_ma"] == pytest.approx(325.0) and fb["voltage_v"] == pytest.approx(11.8)
    assert not fb["fault"]


def test_angles_wait_for_the_hello_calibration(store):
    goal = {b: 0.2 for b in LEG_BUS}
    send(store, tick(1, 1_000_000, goal))
    lt = store.latest("robot")
    assert lt["joints"]["L_knee"]["pos"] is None and lt["joints"]["L_knee"]["goal"] is None
    assert lt["fitted"] == [F.BUS_JOINT_NAMES[b] for b in LEG_BUS]   # from the tick
    assert np.isnan(store.series("robot", "L_knee.pos", 10.0)[1]).all()
    send(store, hello(2))
    # the tick already stored converts once the calibration is known
    _, v = store.series("robot", "L_knee.pos", 10.0)
    assert v[0] == pytest.approx(0.2, abs=2 * math.pi / 4096)
    assert store.latest("robot")["joints"]["L_knee"]["pos"] == pytest.approx(0.2, abs=0.002)


def test_a_missed_read_is_a_fault_with_the_reading_held(store):
    b = F.BUS_JOINT_NAMES.index("R_ankle_roll")
    goal = {k: 0.0 for k in LEG_BUS}
    send(store, hello(1), tick(2, 1_000_000, goal, {**goal, b: 0.05}))
    send(store, tick(3, 1_020_000, goal, {**goal, b: 0.05}, fault=(b,)))
    j = store.latest("robot")["joints"]["R_ankle_roll"]
    assert j["fb_ok"] is False and j["pos"] == pytest.approx(0.05, abs=0.002)
    f = store.feedback("robot")["R_ankle_roll"]
    assert f["fault"] and f["answered"] and f["fault_since"] is not None
    send(store, tick(4, 1_040_000, goal, status={b: 32}))
    f = store.feedback("robot")["R_ankle_roll"]
    assert not f["fault"] and f["fault_since"] is None
    assert flight.status_text(f["status"]) == "overload"


def test_holes_from_sequence_gaps_and_reboots(store):
    goal = {b: 0.0 for b in LEG_BUS}
    send(store, *(tick(s, 1_000_000 + s * 20_000, goal) for s in (1, 2, 3, 6, 7)))
    assert store.stats()["holes"] == 2
    send(store, tick(4, 1_080_000, goal))                  # a late arrival fills one
    assert store.stats()["holes"] == 1
    send(store, tick(1, 500_000, goal, boot=BOOT + 1), tick(3, 540_000, goal, boot=BOOT + 1))
    s = store.stats()
    assert s["reboots"] == 1 and s["holes"] == 2           # one per boot, kept across
    assert any(k == "boot" for _, k, _ in store.events())
    assert any("rebooted" in m for _, m in store.logged)
    t, v = store.series("robot", "holes", 10.0)
    assert v.sum() == 2 and (t <= 1.0).all()
    assert "holes" in store.line() and "2" in store.line()


def test_undecodable_datagrams_are_counted_not_fatal(store):
    send(store, b"not a flight record at all, just noise")
    send(store, b"FR\x09\x01" + b"\0" * 40)                # a newer version
    assert store.stats()["undecodable"] == 2 and store.latest() is None
    send(store, hello(1), tick(2, 1_000_000, {0: 0.1}))
    assert store.latest() is not None


def test_policy_cmd_and_staleness(store, monkeypatch):
    goal = {b: 0.0 for b in LEG_BUS}
    cmd = [0.3, -0.1, 0.25, 0.9, 1.0, 0.01, -0.02]
    send(store, hello(1), tick(2, 1_000_000, goal, cmd=cmd))
    assert store.policy_cmd() == pytest.approx(cmd, abs=1e-6)
    assert len(store.policy_cmd()) == 7
    # a bench tick carries no policy command
    send(store, tick(3, 1_020_000, goal, source=F.SRC_BENCH_IDLE, cmd=[0.0] * 7))
    assert store.policy_cmd() == pytest.approx(cmd, abs=1e-6)
    assert "Hz" in store.line()
    real = time.monotonic
    monkeypatch.setattr(flight.time, "monotonic", lambda: real() + 1.5)
    assert store.policy_cmd() is None                      # older than 1 s
    assert not store.stats()["stale"]
    monkeypatch.setattr(flight.time, "monotonic", lambda: real() + 10.0)
    store.poll(0.0)
    assert store.stats()["stale"] and "no&nbsp;records" in store.line()
    assert "no records for" in store.detail()
    assert store.latest()["age_s"] > 9.0                   # still there, with its age
    assert any("stopped" in m for _, m in store.logged)


def test_hello_and_last_tick_are_the_decoded_packets(store, monkeypatch):
    assert store.hello() is None and store.last_tick() is None
    h = hello(1, fitted=LEG_BUS, cal_ok=1)
    t = tick(2, 1_000_000, gait(0.0), temp=41)
    send(store, h, t)
    assert store.hello() == F.decode(h)
    assert store.hello()["cal_ok"] == 1 and store.hello()["enabled"] == 1
    assert store.hello()["fitted"] == [1 if b in LEG_BUS else 0 for b in range(F.MAX_JOINTS)]
    assert store.last_tick() == F.decode(t)
    assert [j["bus"] for j in store.last_tick()["joints"]] == LEG_BUS
    assert all(j["temp_c"] == 41 for j in store.last_tick()["joints"])
    assert store.hello("sim") is None and store.last_tick("sim") is None
    real = time.monotonic
    monkeypatch.setattr(flight.time, "monotonic", lambda: real() + flight.STALE_S + 0.5)
    assert store.hello() is None and store.last_tick() is None       # stale
    assert store.latest() is not None                                # still drawn, with its age


def test_rate_counts_ticks_per_second(store):
    goal = {b: 0.0 for b in LEG_BUS}
    for s in range(1, 31):
        store.tx.sendto(tick(s, s * 20_000, goal), ("127.0.0.1", store.ports[0]))
        store.poll(0.0)
        time.sleep(0.02)
    store.poll(0.0)
    assert 25.0 < store.stats()["rate_hz"] < 60.0


def test_events_carry_kind_text_and_utc(store):
    utc = int(1_790_000_000.25 * 1e6)
    send(store, event(1, F.EV_CMD, "pose hip 10", t_utc_us=utc),
         event(2, F.EV_OUT, "balance ON", a=F.CHAN_WIFI),
         event(3, F.EV_FLAIL, "tilt 16.2", a=3, b=1),
         event(4, F.EV_STOP, "", a=1, b=1),
         event(5, F.EV_LINK_STATE, "", a=0, b=1),
         event(6, F.EV_MODE, "", a=1, b=1))
    ev = store.events()
    assert [k for _, k, _ in ev] == ["cmd", "out", "flail", "stop", "link_state", "mode"]
    assert ev[0][0] == pytest.approx(1_790_000_000.25)
    assert ev[0][2] == "[tether] pose hip 10"
    assert ev[1][2] == "[wifi] balance ON"
    assert "tilt" in ev[2][2] and "torque held" in ev[2][2]
    assert "HARD" in ev[3][2] and "armed" in ev[3][2]
    assert ev[4][2] == "link LIVE -> STAND"
    assert "ACCEPTED" in ev[5][2]
    assert abs(ev[1][0] - time.time()) < 5.0               # no NTP: this host's clock


def test_sim_source_is_separate(store):
    goal = {b: 0.3 for b in LEG_BUS}
    send(store, hello(1), tick(2, 1_000_000, goal), source="sim")
    assert store.latest("robot") is None
    assert store.latest("sim")["joints"]["L_knee"]["goal"] == pytest.approx(0.3, abs=0.002)
    assert store.fresh("sim") and not store.fresh("robot")
    assert "sim" in store.line()


def test_decimate_keeps_peaks():
    t = np.linspace(-120, 0, 6000)
    v = np.zeros(6000)
    v[3001] = 5.0
    tt, vv = flight.decimate(t, v, 400)
    assert len(tt) <= 400 and vv.max() == 5.0 and tt[-1] == t[-1]


# -- the tabs, offscreen -------------------------------------------------------
@pytest.fixture(scope="module")
def qapp():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt6")
    pytest.importorskip("pyqtgraph")
    from PyQt6 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    from console.ui import QSS, dark_palette
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    yield app


class FakeApp:
    """What the tabs read from the Window: flight, beacons, ctl.st, rate_hz."""

    def __init__(self, store, readonly=True):
        from console import core
        from console.tab_drive import History
        self.flight = store
        self.beacons = History(("t", "vbat", "tilt", "vx_cmd", "vx_est", "wz_cmd", "wz_est",
                                "late"))
        self.rate_hz = 20.0
        st = core.Status()
        st.readonly = int(readonly)
        st.have_tlm = 1
        st.tlm_age_ms = 80.0
        st.rate_hz = 10.0
        st.loop_late_pct = 0
        self.ctl = type("Ctl", (), {"st": st})()


def test_joints_tab_draws_cells_faults_and_the_sim_trace(qapp, store):
    from PyQt6 import QtCore, QtGui
    from console.tab_joints import JointsTab, ORDER, TABLE_COLS
    backfill(store, 10.0)
    app = FakeApp(store)
    tab = JointsTab(app)
    tab.resize(1400, 1000)
    tab.show()
    tab.update_from(app.ctl.st, 0.0)
    qapp.processEvents()
    legs = [F.BUS_JOINT_NAMES[b] for b in LEG_BUS]
    for n in ORDER:
        assert tab.cells[n].mode == ("full" if n in legs else "compact")
    assert "not fitted" in tab.cells["neck_yaw"].c_text.text()
    fault = tab.cells["R_ankle_roll"]
    assert "reading held" in fault.stat.text() and fault._style[0]
    assert "rms" in tab.cells["L_knee"].stat.text()
    x, y = tab.cells["L_knee"].c_meas.getData()
    assert len(x) > 100 and x.min() >= -5.0 and x.max() <= 0.0
    assert len(tab.cells["L_knee"].c_sim.getData()[0]) > 100          # amber sim trace
    assert "sim twin" in tab.legend.text()
    pt = QtCore.QPointF(5, 5)
    left = QtCore.Qt.MouseButton.LeftButton
    tab.cells["R_hip_pitch"].mousePressEvent(QtGui.QMouseEvent(
        QtCore.QEvent.Type.MouseButtonPress, pt, pt, left, left,
        QtCore.Qt.KeyboardModifier.NoModifier))
    assert app.selected_joint == "R_hip_pitch" and "hip pitch" in tab.detail.title()
    tab._set_window(3)                                                # 2 min
    tab._set_unit(1)                                                  # rad
    tab._set_nominal(1)
    tab._toggle_table()
    status = TABLE_COLS.index("status")
    assert tab.table.item(ORDER.index("L_knee"), status).text() == "ok"
    assert "FAULT" in tab.table.item(ORDER.index("R_ankle_roll"), status).text()
    assert tab.table.item(ORDER.index("neck_yaw"), status).text() == "not fitted"
    tab._toggle_pause()
    frozen = tab.cells["L_knee"].c_meas.getData()
    tab.update_from(app.ctl.st, 0.0)                                  # paused: untouched
    assert tab.cells["L_knee"].c_meas.getData()[0] is frozen[0]
    tab.close()


def test_joints_detail_axes_wait_for_data(qapp, store):
    """#243: with no data the DETAIL plots' y axes (labels, ticks) are hidden
    rather than drawn over one another; they come with the data."""
    from console.tab_joints import JointsTab
    app = FakeApp(store)
    tab = JointsTab(app)
    tab.show()
    tab.update_from(app.ctl.st, 0.0)
    qapp.processEvents()
    for pi in tab.d_plots:
        ax = pi.getAxis("left")
        assert not ax.style["showValues"] and not ax.label.isVisible()
    backfill(store, 10.0)
    app.selected_joint = "L_knee"
    tab.update_from(app.ctl.st, 0.0, force=True)
    qapp.processEvents()
    for pi in tab.d_plots:
        ax = pi.getAxis("left")
        assert ax.style["showValues"] and ax.label.isVisible()
    tab.close()


def test_joints_tab_says_when_there_are_no_records(qapp, store):
    from console.tab_joints import JointsTab
    app = FakeApp(store)
    tab = JointsTab(app)
    tab.show()
    tab.update_from(app.ctl.st, 0.0)
    assert "no flight records yet" in tab.banner.text() and tab.banner.isVisibleTo(tab)
    assert all(c.mode == "compact" for c in tab.cells.values())
    app.flight = None
    tab.update_from(app.ctl.st, 0.0, force=True)      # past the redraw throttle
    assert "not available" in tab.banner.text()
    tab.close()


def test_telemetry_tab_charts_and_tables(qapp, store):
    from console.tab_telemetry import TelemetryTab
    backfill(store, 40.0, sim=False)
    app = FakeApp(store)
    for k in range(100):
        app.beacons.push(1000.0 - 10.0 + k * 0.1, 11.9, 2.0, 0, 0, 0, 0, 0)
    tab = TelemetryTab(app)
    assert tab.always_update
    tab.resize(1600, 1000)
    tab.show()
    for k in range(5):
        tab.update_from(app.ctl.st, 1000_000.0 + k * 66)
    qapp.processEvents()
    assert tab.c_tilt.src.text() == "flight rec" and len(tab.c_tilt.curve.getData()[0]) > 100
    assert 9.0 < float(tab.c_work.value.text().split()[0]) < 13.0     # ms, ~11 synthetic
    assert tab.c_holes.curve.getData()[1].max() >= 2                  # the burst at -34 s
    rows = [tab.servos.item(r, 0).text() for r in range(tab.servos.rowCount())]
    assert len(rows) == 12 and rows[0] == "L_hip_yaw"
    st_col = {tab.servos.item(r, 0).text(): tab.servos.item(r, 6).text() for r in range(12)}
    assert st_col["R_ankle_roll"].startswith("FAULT") and st_col["L_knee"] == "ok"
    link = {tab.link.item(r, 0).text(): tab.link.item(r, 1).text()
            for r in range(tab.link.rowCount())}
    assert link["commands sent"].startswith("observer")
    assert "worst tick 13.4 ms" in link["loop"] and "overruns" in link["loop"]
    assert "read 3.4" in link["ctrl tick (last)"]
    assert tab.obs.isVisibleTo(tab)
    # hidden: still samples the beacon age, draws nothing
    tab.hide()
    n = len(tab.ages)
    tab.update_from(app.ctl.st, 1000_400.0)
    assert len(tab.ages) == n + 1
    tab.close()


def test_the_window_receives_records_and_feeds_its_tabs(qapp, tmp_path):
    """The real Window: FlightStore wired in by app.py, records over UDP, the
    bar's link line, the Drive tab's "policy saw", Joints and Telemetry."""
    from console import core
    if not os.path.exists(core.DEFAULT_LIB):
        pytest.skip("build it: make -C firmware/host console")
    from console.app import Window
    from console.controller import Controller
    rp, sp = free_port(), free_port()
    o = options.parse(["--no-record", "--sessions", str(tmp_path), "--keymap",
                       str(tmp_path / "keys.conf"), "--tlm-port", str(free_port()),
                       "--flight-port", str(rp), "--sim-flight-port", str(sp)])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        w.show()
        cmd = [0.25, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0]
        tx.sendto(hello(1), ("127.0.0.1", rp))
        tx.sendto(event(2, F.EV_ARM_EDGE, "", a=1), ("127.0.0.1", rp))
        for s in range(3, 40):
            tx.sendto(tick(s, s * 20_000, gait(s * 0.02), cmd=cmd), ("127.0.0.1", rp))
            tx.sendto(tick(s, s * 20_000, gait(s * 0.02, 1.1)), ("127.0.0.1", sp))
        def show(tab, sec):
            w.select_tab(tab)
            end = time.monotonic() + sec
            while time.monotonic() < end:
                qapp.processEvents()
                time.sleep(0.01)

        show("Drive", 0.3)                      # the policy command is fresh for 1 s
        assert w.policy_saw() == pytest.approx(cmd, abs=1e-6)
        assert w.tab_widgets["Drive"].saw["vx"].text() == "+0.25"
        show("Joints", 0.3)
        show("Telemetry", 0.3)
        assert w.flight.stats()["ticks"] == 37 and w.flight.stats("sim")["ticks"] == 37
        assert "flight" in w.bar.link.text() and "Hz" in w.bar.link.text()
        assert any(k == "arm_edge" for _, k, _ in w.robot_events())
        assert w.tab_widgets["Joints"].cells["L_knee"].mode == "full"
        assert len(w.tab_widgets["Telemetry"].servos.item(0, 0).text()) > 0
        assert "12 of 17 fitted" in w.flight_detail()
    finally:
        tx.close()
        w.close()

