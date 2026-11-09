"""The console's 3D pose view (link/console/pose3d.py, view3d.py; design §5).

The mapping and ghost rules need only mujoco; the pane needs PyQt6 +
pyqtgraph (the console's optional block); a render needs a GL context
(MUJOCO_GL=cgl on the Mac, egl on Linux) and skips without one.
"""
import math
import os
import socket
import sys
import time
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import protocol  # noqa: E402

from console import pose3d  # noqa: E402

AR = os.path.join(ROOT, "sim", "bimo_biped_v6ar.xml")
LEGS = os.path.join(ROOT, "sim", "bimo_biped_v6legs.xml")
NOT_FITTED = {"neck_yaw", "R_shoulder", "R_elbow", "L_shoulder", "L_elbow"}
FITTED = [n for n in protocol.JOINT_NAMES if n not in NOT_FITTED]
RIGHT_LEG = [n for n in FITTED if n.startswith("R_")]


def stride(k=1.0):
    d = {"L_hip_pitch": -30, "R_hip_pitch": 8, "L_knee": -46, "R_knee": -28, "L_ankle": 12,
         "R_ankle": 9, "L_hip_roll": 3, "R_hip_roll": 2, "L_ankle_roll": -3,
         "R_ankle_roll": -2, "L_hip_yaw": 4, "R_hip_yaw": -2}
    return {n: math.radians(v * k) for n, v in d.items()}


@pytest.fixture(scope="module", params=[AR, LEGS], ids=["v6ar", "v6legs"])
def pm(request):
    return pose3d.PoseModel(request.param)


@pytest.fixture(scope="module")
def ar():
    return pose3d.PoseModel(AR)


def test_bus_names_round_trip_through_the_plant(pm):
    J = mujoco.mjtObj.mjOBJ_JOINT
    have = [n for n in protocol.JOINT_NAMES if mujoco.mj_name2id(pm.m, J, n) >= 0]
    assert have and set(have) == set(pm.qadr)
    for n in have:
        jid = mujoco.mj_name2id(pm.m, J, n)
        assert pm.qadr[n] == pm.m.jnt_qposadr[jid]
        assert pm.bus_joint(pm.plant_joint(n)) == n
    assert pm.extra == []                         # every plant joint is on the bus
    vec = [0.02 * (k + 1) * (-1) ** k for k in range(protocol.NUM_JOINTS)]
    d = mujoco.MjData(pm.m)
    pm.pose(d, pm.from_bus(vec))
    back = pm.to_bus(d)
    for k, n in enumerate(protocol.JOINT_NAMES):
        if n in pm.qadr:
            assert back[k] == pytest.approx(vec[k])
        else:
            assert back[k] is None and pm.plant_joint(n) is None


def test_the_plants_bus_coverage():
    assert pose3d.PoseModel(AR).missing == []
    assert set(pose3d.PoseModel(LEGS).missing) == NOT_FITTED
    assert pose3d.resolve_plant("bimo_biped_v6legs.xml") == LEGS
    assert pose3d.resolve_plant(None) == AR


def test_root_pinned_yaw_zero_feet_on_the_floor(ar):
    d = mujoco.MjData(ar.m)
    up = (0.2, -0.1, 0.97)
    root = ar.pose(d, stride(), up)
    assert root[0] == 0.0 and root[1] == 0.0
    assert root[6] == pytest.approx(0.0, abs=1e-12)      # no rotation about z: yaw drawn 0
    zaxis = d.xmat[ar.root_body].reshape(3, 3)[:, 2]
    assert zaxis == pytest.approx(np.array(up) / np.linalg.norm(up), abs=1e-9)
    assert ar.lowest(d) == pytest.approx(ar.floor_z, abs=1e-9)


def test_ghosts_never_change_the_measured_body(pm):
    sc = pose3d.Scene(pm)
    scn = mujoco.MjvScene(pm.m, maxgeom=5000)
    meas = pose3d.Layer(stride(), up=(0.05, -0.03, 0.998), measured=FITTED)
    n0 = sc.build(scn, meas)
    before = (sc.dm.qpos.copy(), sc.dm.xpos.copy(), sc.dm.xquat.copy(),
              sc.dm.geom_xpos.copy(), sc.dm.geom_xmat.copy())
    geoms = [(scn.geoms[i].pos.copy(), scn.geoms[i].mat.copy(), scn.geoms[i].rgba.copy())
             for i in range(n0)]
    ghosts = [pose3d.Layer(stride(1.3), rgba=pose3d.CMD_RGBA),
              pose3d.Layer(stride(0.6), rgba=pose3d.SIM_RGBA)]
    n1 = sc.build(scn, meas, ghosts)
    assert n1 == n0 and scn.ngeom > n0
    after = (sc.dm.qpos, sc.dm.xpos, sc.dm.xquat, sc.dm.geom_xpos, sc.dm.geom_xmat)
    for a, b in zip(before, after):
        assert np.array_equal(a, b)
    for i, (pos, mat, rgba) in enumerate(geoms):
        g = scn.geoms[i]
        assert np.array_equal(g.pos, pos) and np.array_equal(g.mat, mat)
        assert np.array_equal(g.rgba, rgba)
    assert all(d is not sc.dm for d in sc.dg)
    # Ghost geoms are the plant's own geoms, recoloured, at the ghost's pose.
    shown = [scn.geoms[i] for i in range(n0, scn.ngeom) if scn.geoms[i].rgba[3] > 0]
    assert shown
    colours = {tuple(np.round(g.rgba.astype(float), 3).tolist()) for g in shown}
    assert colours == {tuple(np.round(pose3d.CMD_RGBA, 3).tolist()),
                       tuple(np.round(pose3d.SIM_RGBA, 3).tolist())}
    moved = max(float(np.abs(scn.geoms[n0 + k].pos - scn.geoms[k].pos).max())
                for k in range(min(n0, scn.ngeom - n0)))
    assert moved > 1e-3


def test_a_joint_with_no_goal_draws_no_ghost(ar):
    sc = pose3d.Scene(ar)
    scn = mujoco.MjvScene(ar.m, maxgeom=5000)
    left_only = {n: v for n, v in stride(1.3).items() if n.startswith("L_")}
    n0 = sc.build(scn, pose3d.Layer(stride(), measured=FITTED),
                  [pose3d.Layer(left_only, rgba=pose3d.CMD_RGBA)])
    J = mujoco.mjtObj.mjOBJ_JOINT
    right = {int(ar.m.jnt_bodyid[mujoco.mj_name2id(ar.m, J, n)]) for n in RIGHT_LEG}
    G = mujoco.mjtObj.mjOBJ_GEOM
    for i in range(n0, scn.ngeom):
        g = scn.geoms[i]
        if g.objtype != G:
            continue
        body = int(ar.m.geom_bodyid[g.objid])
        if body in right or not ar.path[body]:      # the right leg, the torso
            assert g.rgba[3] == 0.0
    assert any(scn.geoms[i].rgba[3] > 0 for i in range(n0, scn.ngeom))


def test_unfitted_joints_are_dimmed_at_the_zero_pose(ar):
    sc = pose3d.Scene(ar)
    scn = mujoco.MjvScene(ar.m, maxgeom=5000)
    n0 = sc.build(scn, pose3d.Layer(stride(), measured=FITTED))
    J = mujoco.mjtObj.mjOBJ_JOINT
    for n in NOT_FITTED:                          # the plant's zero pose: qpos0 (ref)
        assert sc.dm.qpos[ar.qadr[n]] == ar.m.qpos0[ar.qadr[n]]
    arm = int(ar.m.jnt_bodyid[mujoco.mj_name2id(ar.m, J, "R_elbow")])
    shin = int(ar.m.jnt_bodyid[mujoco.mj_name2id(ar.m, J, "R_knee")])
    G = mujoco.mjtObj.mjOBJ_GEOM
    rgba = {int(ar.m.geom_bodyid[scn.geoms[i].objid]): tuple(scn.geoms[i].rgba)
            for i in range(n0) if scn.geoms[i].objtype == G}
    assert rgba[arm] == pytest.approx(pose3d.DIM_RGBA)
    assert rgba[shin] != pytest.approx(pose3d.DIM_RGBA)


@pytest.fixture(scope="module")
def renderer(ar):
    try:
        r = pose3d.Renderer(ar, 320, 256)
    except Exception as e:          # no GL context here (MUJOCO_GL)
        pytest.skip(f"no offscreen GL: {e}")
    yield r
    r.close()


def test_a_render_is_not_blank_and_follows_a_joint(renderer):
    zero = renderer.render(pose3d.Layer({}, measured=FITTED))
    knee = renderer.render(pose3d.Layer({"R_knee": -1.1, "R_hip_pitch": 0.6}, measured=FITTED))
    assert zero.shape == (256, 320, 3)
    assert zero.std() > 10 and len(np.unique(zero.reshape(-1, 3), axis=0)) > 50
    assert np.abs(zero.astype(int) - knee.astype(int)).mean() > 0.3


# -- the Qt pane ---------------------------------------------------------------
@pytest.fixture(scope="module")
def qt():
    pytest.importorskip("PyQt6")
    pytest.importorskip("pyqtgraph")
    from PyQt6 import QtWidgets
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class Flight:
    """The flight store's contract: latest(source) -> None or a dict."""

    def __init__(self, age=0.02, sim=None, goal=True):
        self.age = age
        self.sim = sim
        self.goal = goal

    def latest(self, source):
        if source == "robot":
            meas, goal = stride(), stride(1.3)
            return {"t": 100.0, "age_s": self.age, "fitted": FITTED, "up": (0.03, 0.0, 0.999),
                    "joints": {n: {"pos": meas[n],
                                   "goal": goal[n] if self.goal and not n.startswith("R_") else None,
                                   "nominal": goal[n], "flags": 0, "fb_ok": True}
                               for n in FITTED}}
        if source == "sim" and self.sim is not None:
            q = stride(0.7)
            return {"t": 100.0, "age_s": self.sim, "fitted": FITTED, "up": None,
                    "joints": {n: {"pos": q[n], "goal": None, "nominal": None, "flags": 0,
                                   "fb_ok": True} for n in FITTED}}
        return None


def pane_for(qt, flight=None, *argv):
    from console import options, view3d
    app = SimpleNamespace(o=options.parse(list(argv)), flight=flight, selected_joint=None)
    p = view3d.ViewPane(app)
    p.resize(760, 640)
    p.show()
    qt.processEvents()
    return p


def beacon(joints=None, up=None, age_ms=40.0, count=1, state=0):
    from console import core
    st = core.Status()
    st.have_tlm = 1
    st.tlm_count = count
    st.tlm_age_ms = age_ms
    st.state = state
    st.want_pose = 1
    if joints is not None:
        st.n_joints = core.NUM_JOINTS
        for k, n in enumerate(protocol.JOINT_NAMES):
            st.joints[k] = joints.get(n, 0.0)
    if up is not None:
        st.have_att = 1
        st.up_x, st.up_y, st.up_z = up
    return st


def test_frozen_after_one_second(qt):
    fl = Flight(age=0.4, sim=0.3)
    p = pane_for(qt, fl)
    try:
        p.set_overlays(commanded=True, sim=True)
        p.update_from(None, 1000.0)
        assert not p.frozen("measured") and not p.frozen("sim")
        assert not any("FROZEN" in r[2] for r in p.legend_rows())
        fl.age, fl.sim = 1.2, 1.5
        p.update_from(None, 2000.0)
        assert p.frozen("measured") and p.frozen("commanded") and p.frozen("sim")
        ages = [r[2] for r in p.legend_rows()]
        assert "FROZEN 1.2 s" in ages and "FROZEN 1.5 s" in ages
        assert "FROZEN" in p.status.text()
    finally:
        p.close_streams()


def test_commanded_ghost_is_the_goal_and_skips_joints_without_one(qt):
    p = pane_for(qt, Flight())
    try:
        p.update_from(None, 1000.0)
        assert p.info["measured"]["source"] == "flight recorder"
        assert set(p.meas.q) == set(FITTED) and p.meas.up is not None
        (cmd,) = p.ghosts
        assert set(cmd.q) == {n for n in FITTED if not n.startswith("R_")}
        p.commanded_field = "nominal"
        p.update_from(None, 1100.0)
        assert set(p.ghosts[0].q) == set(FITTED)
    finally:
        p.close_streams()


def test_beacon_path_with_no_flight_store(qt):
    p = pane_for(qt, None, "--mirror")
    try:
        p.update_from(beacon(), 1000.0)
        assert p.info["measured"]["source"] is None
        assert p.ghosts == [] and "no flight records" in p.legend_rows()[1][1]
        q = stride()
        p.update_from(beacon(q, up=(0.1, 0.0, 0.995), age_ms=50.0, count=2), 2000.0)
        m = p.info["measured"]
        assert m["source"] == "beacon" and m["age"] == pytest.approx(0.05)
        for n in FITTED:
            assert p.meas.q[n] == pytest.approx(q[n], abs=1e-6)
        assert p.meas.up == pytest.approx((0.1, 0.0, 0.995), abs=1e-6)
        # A short beacon carries no joints: the pose holds, ageing, then FROZEN.
        p.update_from(beacon(None, count=3, age_ms=10.0), 3500.0)
        assert p.info["measured"]["source"] == "beacon" and p.frozen("measured")
        p.update_from(beacon(q, count=4), 3600.0)
        assert ("attitude not sent" in [r[1] for r in p.legend_rows()])
        img = p.render_now()            # the pane guards a renderer it cannot
        if p._render_error is None:     # build: no GL here to prove pixels with
            assert img is not None and not img.isNull() and p.renders >= 1
        else:
            assert img is None          # latched, said in the pane
    finally:
        p.close_streams()


def test_a_benched_robot_says_why_there_is_no_pose(qt):
    from console import core
    p = pane_for(qt, None)
    try:
        p.update_from(beacon(state=core.BENCH), 1000.0)
        assert "BENCHED robot measures nothing" in p.status.text()
    finally:
        p.close_streams()


def test_a_failing_flight_store_does_not_take_the_window_down(qt):
    class Broken:
        def latest(self, source):
            raise RuntimeError("boom")
    p = pane_for(qt, Broken())
    try:
        p.update_from(beacon(stride(), count=5), 1000.0)
        assert p.info["measured"]["source"] == "beacon"
        assert "boom" in p.status.text()
    finally:
        p.close_streams()


def test_selected_joint_and_plant_choice(qt):
    p = pane_for(qt, Flight(), "--plant", "bimo_biped_v6legs.xml")
    try:
        assert p.plant == LEGS and set(p.model.missing) == NOT_FITTED
        p.app.selected_joint = "L_knee"
        assert p.selected() == "L_knee"
        p.app.selected_joint = "R_elbow"          # not in this plant
        assert p.selected() is None
    finally:
        p.close_streams()


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_camera_pip_waits_and_close_stops_every_stream(qt):
    port = _free_port()
    p = pane_for(qt, Flight(), "--cam-relay", f"127.0.0.1:{port}", "--cam", "emeet")
    try:
        assert p.show_pip.isChecked() and p.cam_name == "emeet"
        assert set(p.streams) == {"emeet"}
        assert p.streams["emeet"].url == f"http://127.0.0.1:{port}/emeet"
        assert p.stream_state("emeet")[0] == "○ waiting"
        p.set_source("sim")                     # only what is on screen runs
        assert set(p.streams) == {"sim render"}
        assert p.streams["sim render"].url == "http://127.0.0.1:8645/"
        p.set_source("3d")
        p.set_split(True)
        r3d, rc = p.rects(760, 600)
        assert r3d.right() < rc.left()          # beside the model, never under it
        img = p.grab()
        assert not img.isNull()
        streams = list(p.streams.values())
    finally:
        p.close_streams()
    assert p.streams == {}
    end = time.monotonic() + 3.0
    while any(s.state() != "off" for s in streams) and time.monotonic() < end:
        time.sleep(0.05)
    assert all(s.state() == "off" for s in streams)


def test_a_renderer_that_cannot_be_built_is_not_an_exception(qt, monkeypatch):
    """A GL context the box cannot give (headless, MUJOCO_GL=disabled, a
    driver with no offscreen context) breaks the 3D picture alone: the pane
    latches the failure once, paints why instead of the robot, and never
    raises into the GUI loop -- #241's pane-fit test must survive a suite
    run where sibling tests disabled MUJOCO_GL first."""
    import mujoco

    calls = []

    def broken(*a, **k):
        calls.append(1)
        raise mujoco.FatalError("gladLoadGL error")     # headless glfw; or an
        # AttributeError raised inside mujoco when a sibling test set
        # MUJOCO_GL=disabled before the pane was built

    monkeypatch.setattr(pose3d, "Renderer", broken)     # first: pane_for renders
    p = pane_for(qt, Flight())                          # eagerly (processEvents)
    try:
        assert calls and p._render_error == "FatalError: gladLoadGL error"
        n = len(calls)
        p.update_from(None, 1000.0)             # a new sample: it wants a render
        img = p.render_now()                    # the pane's own entry point
        assert img is None
        assert len(calls) == n                  # latched: no second attempt
        assert p.render_now() is None and len(calls) == n
        assert p.renders >= n
        painted = p.grab()                      # the message paints; nothing raised
        assert not painted.isNull()
        pix = painted.toImage()
        colors = {pix.pixelColor(x, y).name() for x in range(0, 700, 7)
                  for y in range(0, 600, 7)}
        assert len(colors) > 1                  # text on the sky: it is not a void
    finally:
        p.close_streams()
