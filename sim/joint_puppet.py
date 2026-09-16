"""Joint puppet for the v6/v7 body: UI sliders drive every joint while gravity
runs, with the live 3D view in the SAME window.

One window, native Qt. A slider per joint (its real range, in degrees) drives
the plant's own `position` actuators -- the same servos the walk uses -- so
dragging a slider is a joint command, and the body sags, settles, catches or
topples under gravity as you drag. The view beside the sliders is the sim,
rendered offscreen each frame. The number under each slider is the ACHIEVED
angle: the gap between it and the slider is the reachability signal -- a joint
that won't reach its target is one the servo cannot hold there against the
floor.

    .venv/bin/python sim/joint_puppet.py                 # standing
    .venv/bin/python sim/joint_puppet.py --pose supine   # start fallen, on its back
    .venv/bin/python sim/joint_puppet.py --pose prone    # start face-down
    .venv/bin/python sim/joint_puppet.py --skid          # with the pelvis skid

Buttons: the body poses (jump the fall), Record (store the settled pose as a
get-up keyframe) and Dump (print them as getup_v6-style offsets).

Every session is LOGGED: each slider command with the sim time it was given,
and each recorded frame with target vs achieved angles, pelvis height,
uprightness and floor contacts. The log is plain JSONL, one event per line, in
`sim/puppet_sessions/` -- a file to hand to someone, and a file the sim can
eat again:

    .venv/bin/python sim/joint_puppet.py --export LOG.jsonl   # -> a getup_v6 keyframe sequence
    .venv/bin/python sim/joint_puppet.py --run    LOG.jsonl   # -> feed it back through the study's
                                                              #    evaluator (getup_v6.run_sequence,
                                                              #    deploy servo model) -- does the pose
                                                              #    you found survive real servos?

Implementation note, so nobody re-litigates it: MuJoCo's own simulate UI
(viewer.launch, which has a Control-slider panel) cannot construct on this
MuJoCo 3.10 / Python 3.14 / macOS build -- `_Simulate` throws for any model,
including a 3-line stock one. viewer.launch_passive works but has no sliders.
Hence Qt for the widgets plus an offscreen mujoco.Renderer for the picture,
which is also why the physics runs on a worker thread behind a lock: GL and
the physics must never touch MjData at the same instant (that race is what
produced the "mjpython quit unexpectedly" crash dialogs). No web UI. Ever.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "glfw")      # offscreen render backend

import mujoco  # noqa: E402

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402

JOINTS = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
          "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]

POSES = {
    "stand":  ((1.0, 0.0, 0.0, 0.0), None),                # z from the plant
    "supine": ((math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0), 0.13),
    "prone":  ((math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0), 0.13),
    "side_L": ((math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0), 0.13),
    "side_R": ((math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0), 0.13),
}


class Puppet:
    """the sim: plant, targets, and a lock that serializes physics against the
    renderer (never let GL read MjData mid-step)."""

    def __init__(self, p: DesignParams, pose: str):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build_xml(p))
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()
        self.quit = False
        self.recorded = []
        self.targets = {}                 # deg, keyed by actuator/joint name
        self.act = {}
        for i in range(self.m.nu):
            a = self.m.actuator(i)
            lo, hi = self.m.jnt_range[a.trnid[0]]
            if math.degrees(hi - lo) > 0.1:
                self.targets[a.name] = 0.0
                self.act[a.name] = i
        self.names = JOINTS + [n for n in self.targets if n not in JOINTS]
        self.set_body(pose)

    # ---- geometry helpers
    def limits_deg(self, name):
        jid = self.m.actuator(self.act[name]).trnid[0]
        lo, hi = self.m.jnt_range[jid]
        return math.degrees(float(lo)), math.degrees(float(hi))

    def actual_deg(self, name):
        adr = self.m.joint(self.m.actuator(self.act[name]).trnid[0]).qposadr[0]
        return math.degrees(float(self.d.qpos[adr]))

    # ---- state changes
    def set_body(self, pose: str):
        quat, z = POSES[pose]
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self.d.qpos[0:3] = [0.0, 0.0, z if z is not None else self.p.z_yaw_above_sole]
            self.d.qpos[3:7] = quat
            self.d.qvel[:] = 0.0
            mujoco.mj_forward(self.m, self.d)
            for name, i in self.act.items():
                adr = self.m.joint(self.m.actuator(i).trnid[0]).qposadr[0]
                self.targets[name] = math.degrees(self.d.qpos[adr])

    def set_target(self, name, deg):
        with self.lock:
            self.targets[name] = float(deg)

    def step(self, n=1):
        with self.lock:
            for _ in range(n):
                for name, i in self.act.items():
                    self.d.ctrl[i] = math.radians(self.targets[name])
                mujoco.mj_step(self.m, self.d)

    def snapshot(self):
        with self.lock:
            com = self.d.subtree_com[0].copy()
            up = float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2])
            contacts = sorted({self.m.body(self.m.geom_bodyid[g]).name
                               for i in range(int(self.d.ncon))
                               for g in (self.d.contact[i].geom1, self.d.contact[i].geom2)
                               if self.m.body(self.m.geom_bodyid[g]).name != "world"
                               and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})
            return float(self.d.qpos[2]), up, com, contacts, float(self.d.time)

    def record(self):
        with self.lock:
            q = {n: round(self.actual_deg(n), 1) for n in self.names}
            tgt = {n: round(self.targets[n], 1) for n in self.names}
            z = float(self.d.qpos[2])
            up = float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2])
            t = float(self.d.time)
            contacts = self.snapshot_contacts()
        self.recorded.append(q)
        line = ", ".join(f"{n[2:]} {v:+.0f}" for n, v in q.items() if abs(v) > 0.5)
        print(f"[frame {len(self.recorded)}] pelvis z {z:.3f} up {up:+.2f} | {line or 'standing'}", flush=True)
        self.log(dict(cmd="record", t=t, frame=len(self.recorded), target=tgt,
                      achieved=q, pelvis_z=round(z, 4), up=round(up, 3), contacts=contacts))

    def snapshot_contacts(self):
        return sorted({self.m.body(self.m.geom_bodyid[g]).name
                       for i in range(int(self.d.ncon))
                       for g in (self.d.contact[i].geom1, self.d.contact[i].geom2)
                       if self.m.body(self.m.geom_bodyid[g]).name != "world"
                       and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})

    # ---- session log: every angle command, with the sim time it was given.
    # The file is the point: it can be fed back into the sim (--run), turned
    # into a getup_v6 sequence (--export), or just read.
    def open_log(self, path, meta):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.log_fh = open(path, "w", buffering=1)      # line-buffered
        self.log_path = path
        self.log(dict(cmd="init", **meta))

    def log(self, ev):
        fh = getattr(self, "log_fh", None)
        if fh is not None:
            fh.write(json.dumps(ev) + "\n")

    def dump(self):
        print("recorded keyframes (getup_v6-style offsets):", flush=True)
        for i, f in enumerate(self.recorded):
            print(f"  ({i+1}, {f})", flush=True)
        if getattr(self, "log_path", None):
            print(f"session log: {self.log_path}", flush=True)


# --------------------------------------------------------------------------- log tools
def load_log(path):
    evs = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                evs.append(json.loads(line))
    return evs


def log_to_sequence(evs, hold=1.0, move=1.5):
    """the recorded frames of a session -> a getup_v6 keyframe sequence:
    [(label, {joint: deg, ...}, move_s, hold_s), ...]. These are the ACHIEVED
    angles, which is what the body actually held under gravity."""
    seq = []
    for ev in evs:
        if ev.get("cmd") == "record":
            off = {k: v for k, v in ev["achieved"].items() if abs(v) > 0.05}
            seq.append((f"frame {ev['frame']}", off, move, hold))
    return seq


def export_sequence(path, hold=1.0, move=1.5):
    evs = load_log(path)
    init = next((e for e in evs if e.get("cmd") == "init"), {})
    seq = log_to_sequence(evs, hold, move)
    print(f"# from {os.path.basename(path)} -- start pose {init.get('pose')!r}, "
          f"skid={init.get('skid')}, knee={init.get('knee')!r}")
    print(f"# {len(seq)} recorded frames; paste into sim/getup_v6.py SEQUENCES "
          f"or run: sim/joint_puppet.py --run {os.path.basename(path)}")
    print("[")
    for label, off, mv, hd in seq:
        print(f"    ({label!r}, {off}, {mv}, {hd}),")
    print("]")
    return seq


def run_sequence_from_log(path, hold=1.0, move=1.5, play=3.0, mu=0.7):
    """feed the session's angle commands back through the STUDY's evaluator:
    getup_v6.run_sequence under the deploy servo model (play, shaper, STS3250
    knees/rolls) from the settled fall -- i.e. does what you posed by hand
    actually hold up when the real servos have to do it?"""
    import getup_v6 as G
    evs = load_log(path)
    init = next((e for e in evs if e.get("cmd") == "init"), {})
    seq = log_to_sequence(evs, hold, move)
    if not seq:
        print("no recorded frames in the log (press 'record' in the GUI to capture poses)")
        return None
    p = DesignParams(skid=bool(init.get("skid")), knee=init.get("knee", "fwd"))
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"puppet_run_{os.getpid()}.xml")
    with open(xml, "w") as fh:
        fh.write(build_xml(p))
    start = init.get("pose", "stand")
    start = start if start in ("supine", "prone") else "supine"
    print(f"replaying {len(seq)} hand-posed frames from {start} under the deploy servo model "
          f"(play {play} deg, mu {mu}):")
    r = G.run_sequence(p, xml, seq, start=start, play_deg=play, per_joint=G.PJ_DEFAULT,
                       mu=mu, verbose=True)
    print(f"=> {'STANDING' if r['ok'] else 'not standing'}: up {r['up']:+.2f}, "
          f"pelvis z {r['pelvis_z']:.3f}")
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="stand", choices=list(POSES))
    ap.add_argument("--skid", action="store_true", help="add the pelvis skid")
    ap.add_argument("--knee", choices=("fwd", "bwd"), default="fwd")
    ap.add_argument("--log", default=None,
                    help="session log path (default sim/puppet_sessions/<stamp>.jsonl)")
    ap.add_argument("--export", metavar="LOG",
                    help="print a getup_v6 keyframe sequence from a session log and exit")
    ap.add_argument("--run", metavar="LOG",
                    help="feed a session log's recorded frames back through the study's "
                         "evaluator (getup_v6.run_sequence, deploy servo model) and exit")
    ap.add_argument("--move", type=float, default=1.5, help="--export/--run: slew seconds per frame")
    ap.add_argument("--hold", type=float, default=1.0, help="--export/--run: hold seconds per frame")
    a = ap.parse_args()

    if a.export:
        export_sequence(a.export, hold=a.hold, move=a.move)
        return
    if a.run:
        run_sequence_from_log(a.run, hold=a.hold, move=a.move)
        return

    from PyQt6 import QtCore, QtGui, QtWidgets

    pup = Puppet(DesignParams(skid=a.skid, knee=a.knee), a.pose)
    log_path = a.log or os.path.join(
        HERE, "puppet_sessions",
        f"session_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl")
    pup.open_log(log_path, dict(pose=a.pose, skid=bool(a.skid), knee=a.knee,
                                joints=pup.names,
                                when=datetime.datetime.now().isoformat(timespec="seconds")))

    app = QtWidgets.QApplication(sys.argv[:1])
    win = QtWidgets.QWidget()
    win.setWindowTitle("v6/v7 joint puppet — gravity on")
    outer = QtWidgets.QHBoxLayout(win)

    # ---- left: the live 3D view
    view = QtWidgets.QLabel()
    view.setMinimumSize(520, 660)
    view.setStyleSheet("background:#111;")
    outer.addWidget(view)

    # ---- right: a slider per joint + buttons + status
    right = QtWidgets.QVBoxLayout()
    outer.addLayout(right)
    right.addWidget(QtWidgets.QLabel(
        "<b>joint targets</b> (deg) — gravity is on; the servo settles the pose.<br>"
        "<span style='color:#2a7'>green</span> = the angle actually reached."))

    grid = QtWidgets.QGridLayout()
    right.addLayout(grid)
    achieved = {}
    for row, name in enumerate(pup.names):
        lo, hi = pup.limits_deg(name)
        grid.addWidget(QtWidgets.QLabel(name), row, 0)
        sl = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        sl.setMinimum(int(round(lo)))
        sl.setMaximum(int(round(hi)))
        sl.setValue(int(round(pup.targets[name])))
        sl.setMinimumWidth(240)
        tgt = QtWidgets.QLabel(f"{pup.targets[name]:+.0f}°")
        tgt.setFixedWidth(46)
        tgt.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        ach = QtWidgets.QLabel("")
        ach.setFixedWidth(46)
        ach.setStyleSheet("color:#2a7;")
        ach.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)

        def on_change(v, nm=name, lab=tgt):
            pup.set_target(nm, float(v))
            lab.setText(f"{v:+d}°")
            pup.log(dict(cmd="set", t=round(float(pup.d.time), 3), joint=nm, deg=float(v)))
        sl.valueChanged.connect(on_change)

        grid.addWidget(sl, row, 1)
        grid.addWidget(tgt, row, 2)
        grid.addWidget(ach, row, 3)
        achieved[name] = (sl, ach)

    # ---- buttons: body poses, record, dump
    btns = QtWidgets.QHBoxLayout()
    right.addLayout(btns)

    def refresh_sliders():
        for nm, (sl, _) in achieved.items():
            sl.blockSignals(True)
            sl.setValue(int(round(pup.targets[nm])))
            sl.blockSignals(False)

    for pose_name in POSES:
        b = QtWidgets.QPushButton(pose_name)
        b.clicked.connect(lambda _, po=pose_name: (
            pup.log(dict(cmd="pose", t=round(float(pup.d.time), 3), pose=po)),
            pup.set_body(po), refresh_sliders()))
        btns.addWidget(b)
    b_rec = QtWidgets.QPushButton("record")
    b_rec.clicked.connect(lambda: pup.record())
    btns.addWidget(b_rec)
    b_dump = QtWidgets.QPushButton("dump")
    b_dump.clicked.connect(lambda: pup.dump())
    btns.addWidget(b_dump)

    status = QtWidgets.QLabel("")
    status.setStyleSheet("font-family:monospace; color:#666;")
    right.addWidget(status)
    right.addStretch(1)

    # ---- physics on a worker thread; rendering + widgets on the Qt thread
    def sim_loop():
        while not pup.quit:
            pup.step(16)                 # ~16 ms of sim per pass
            time.sleep(0.004)
    threading.Thread(target=sim_loop, daemon=True).start()

    renderer = mujoco.Renderer(pup.m, 660, 520)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.3, -16.0, 135.0

    def tick():
        z, up, com, contacts, t = pup.snapshot()
        with pup.lock:
            cam.lookat[:] = [float(pup.d.qpos[0]), float(pup.d.qpos[1]),
                             max(0.12, float(pup.d.qpos[2]) * 0.5)]
            renderer.update_scene(pup.d, cam)
            frame = renderer.render().copy()
            act = {n: pup.actual_deg(n) for n in pup.names}
        img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0],
                           frame.strides[0], QtGui.QImage.Format.Format_RGB888)
        view.setPixmap(QtGui.QPixmap.fromImage(img))
        for nm, (_, lab) in achieved.items():
            lab.setText(f"{act[nm]:+.0f}°")
        status.setText(f"t {t:6.1f}s   pelvis z {z:.3f}   up {up:+.2f}   CoM x {com[0]:+.3f}\n"
                       f"frames {len(pup.recorded)}   contacts: {' '.join(contacts)}")

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(33)                      # ~30 fps

    print(f"joint puppet: v6/{'v7' if pup.p.torso_v7 else 'v6'} body"
          f"{', pelvis skid' if a.skid else ''}, start '{a.pose}' -- "
          f"{len(pup.names)} sliders, gravity on.", flush=True)
    win.resize(1080, 700)
    win.show()
    win.raise_()                 # macOS: a window from a terminal-launched
    win.activateWindow()         # process otherwise opens behind the terminal
    app.exec()
    pup.quit = True
    pup.dump()


if __name__ == "__main__":
    main()
