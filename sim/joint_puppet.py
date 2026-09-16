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

Camera: left-drag orbits, right-drag pans, the wheel zooms, double-click
resets the view.

Keyboard: one finger column per joint, proximal on the pinky out to distal on
the index; top row +1 deg, home row zero, bottom row -1 deg. The right leg is
the same shape mirrored by finger role, so the same finger drives the same
joint on the same side of the body. Hold a key to sweep: the repeat is the
tool's own (60 steps/s, no initial delay, --key-rate to change it), not the
system's typing repeat.

              LEFT LEG                         RIGHT LEG
    hip yaw    q / a / z                        p / ; / /
    hip roll   w / s / x                        o / l / .
    hip pitch  e / d / c                        i / k / ,
    knee       r / f / v                        u / j / m
    ankle      t / g / b                        y / h / n
    ankle roll 6 (+) 5 (-)  [no zero]           7 (+) 8 (-)  [no zero]

Buttons: the body poses (jump the fall). There is no record button on purpose.

The session RECORDS ITSELF: the whole timeline is sampled continuously
(targets, achieved angles, pelvis height, uprightness, floor contacts) to a
JSONL log in `sim/puppet_sessions/`. Afterwards the keyframes of that session
are recovered from the log -- the poses you actually HELD, found by looking
for stretches where the targets stopped changing and the body stopped moving:

    .venv/bin/python sim/joint_puppet.py --keyframes LOG.jsonl  # -> a getup_v6 sequence
    .venv/bin/python sim/joint_puppet.py --run       LOG.jsonl  # -> feed those poses back through
                                                                 #    the study's evaluator
                                                                 #    (getup_v6.run_sequence, deploy
                                                                 #    servo model): do they survive
                                                                 #    real servos?

So a session from last week can be re-read, replayed, or handed over whole,
without anyone having pressed anything at the right moment.

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

# Keyboard: one finger column per joint, proximal (hip yaw) on the pinky out to
# distal (ankle) on the index; top row = +, home = zero, bottom = -. The right
# leg is the same shape mirrored by finger role (q<->p, w<->o, ... b<->n), so
# the same finger drives the same joint on the same side of the body. Ankle
# roll has no letter column left, so it sits on the number row (+ on the inner
# key) and has no zero.
KEY_STEP = 1.0                        # degrees per press (auto-repeat sweeps)
KEYMAP = {}                           # char -> (joint, delta or None for zero)
for _side, _plus, _zero, _minus in (
        ("L", "qwert", "asdfg", "zxcvb"),
        ("R", "poiuy", ";lkjh", "/.,mn"),
):
    for _lvl, _p, _z, _m in zip(
            ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"), _plus, _zero, _minus):
        KEYMAP[_p] = (f"{_side}_{_lvl}", +KEY_STEP)
        KEYMAP[_z] = (f"{_side}_{_lvl}", None)
        KEYMAP[_m] = (f"{_side}_{_lvl}", -KEY_STEP)
KEYMAP["6"] = ("L_ankle_roll", +KEY_STEP)      # no zero: the roll is only +-25
KEYMAP["5"] = ("L_ankle_roll", -KEY_STEP)
KEYMAP["7"] = ("R_ankle_roll", +KEY_STEP)
KEYMAP["8"] = ("R_ankle_roll", -KEY_STEP)
# by Qt key code, which is what an event carries (and unlike ev.text() it is
# still right on key RELEASE). For every character used here the Qt code is
# ord() of its upper-case form: Key_Q == 0x51, Key_Semicolon == 0x3B, etc.
KEYCODES = {ord(_ch.upper()): _v for _ch, _v in KEYMAP.items()}
KEY_RATE_HZ = 60.0                    # held-key steps per second (our own repeat)


def cad_rom_params(skid=False, knee="both"):
    """the plant with the HARDWARE's range of motion, not the walk's.

    The joint limits come from the CAD ROM table (cad/v6/dimensions_v6.ROM),
    which is the table `cad/check_assembly_v6.py` sweeps for interference, so
    every limit here is one the printed parts have been checked at:

        hip yaw    +-45          hip pitch  -125 .. +90
        hip roll   +-55          knee       -95 .. +130   (+ = human flexion)
        ankle      +-40          ankle roll +-25          neck +-90

    Sign note: the assembly rotates pitch joints about +Y, the sim's knee axis
    is -Y, so the sim's knee is the CAD's negated -- CAD +130 flexion is sim
    -130, CAD -95 backward is sim +95. knee="both" gives the joint as the CAD
    allows it in BOTH directions (the plant's default 'fwd' caps the backward
    side at 5 deg, which is a modelling cap inherited from v5, not hardware).

    Two of these are deliberately wider than the walking plant's defaults,
    because those defaults are design choices rather than limits: hip roll
    (the walk uses abduction 45 / adduction 30) and the knee's backward side.
    One is narrower: the walking plant allows +-45 ankle pitch where the CAD
    ROM says +-40."""
    return DesignParams(
        skid=skid, knee=knee,
        yaw_range=45.0,
        hip_roll_abd=55.0, hip_roll_add=55.0,
        hip_pitch_range=(-125.0, 90.0),
        knee_flex=130.0, knee_back=95.0,
        ankle_range=40.0,
        ankle_roll_range=25.0,
    )


class Puppet:
    """the sim: plant, targets, and a lock that serializes physics against the
    renderer (never let GL read MjData mid-step)."""

    def __init__(self, p: DesignParams, pose: str):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build_xml(p))
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()
        self.quit = False
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

    def contacts(self):
        """bodies touching the floor (the floor geom itself excluded). Call
        with the lock held."""
        return sorted({self.m.body(self.m.geom_bodyid[g]).name
                       for i in range(int(self.d.ncon))
                       for g in (self.d.contact[i].geom1, self.d.contact[i].geom2)
                       if self.m.body(self.m.geom_bodyid[g]).name != "world"
                       and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})

    def snapshot(self):
        """(pelvis_z, up_z, com, contacts, sim_time) -- the status line's data."""
        with self.lock:
            return (float(self.d.qpos[2]),
                    float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]),
                    self.d.subtree_com[0].copy(),
                    self.contacts(),
                    float(self.d.time))

    # ---- session log: the WHOLE timeline, sampled continuously, so a prior
    # session can be reconstructed and its keyframes recovered afterwards --
    # nothing depends on having pressed a button at the right moment.
    def open_log(self, path, meta):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.log_fh = open(path, "w", buffering=1)      # line-buffered
        self.log_path = path
        self.log(dict(cmd="init", **meta))

    def log(self, ev):
        fh = getattr(self, "log_fh", None)
        if fh is not None:
            fh.write(json.dumps(ev) + "\n")

    def sample(self):
        """one timeline sample: targets, achieved angles, pose, contacts. Cheap
        and compact (arrays in `joints` order from the init line)."""
        with self.lock:
            tgt = [round(self.targets[n], 1) for n in self.names]
            q = [round(self.actual_deg(n), 1) for n in self.names]
            z = round(float(self.d.qpos[2]), 4)
            up = round(float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]), 3)
            t = round(float(self.d.time), 2)
            con = self.contacts()
        self.log(dict(cmd="s", t=t, tgt=tgt, q=q, z=z, up=up, con=con))

    def close_log(self):
        if getattr(self, "log_path", None):
            print(f"session log: {self.log_path}", flush=True)
            print(f"  recover its keyframes: "
                  f"sim/joint_puppet.py --keyframes {self.log_path}", flush=True)
        fh = getattr(self, "log_fh", None)
        if fh is not None:
            fh.close()


# --------------------------------------------------------------------------- log tools
def load_log(path):
    evs = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                evs.append(json.loads(line))
    return evs


def keyframes_from_log(path, settle_s=0.8, move_tol=0.5):
    """RECOVER the keyframes of a past session automatically: no 'record'
    button needed. Walk the sampled timeline and emit one keyframe per pose
    the operator actually HELD -- a stretch where the targets stopped changing
    and the body stopped moving (joint motion under move_tol deg between
    samples) for at least settle_s. That is the definition of 'a pose he
    stopped at', which is what a keyframe is.

    Returns [(label, {joint: achieved_deg}, move_s, hold_s)] plus the per-frame
    state, so an old session can be replayed or read without any manual step."""
    evs = load_log(path)
    init = next((e for e in evs if e.get("cmd") == "init"), {})
    names = init.get("joints", [])
    samples = [e for e in evs if e.get("cmd") == "s"]
    if not samples or not names:
        return [], init, []

    frames, run_start, prev = [], None, None
    for s in samples:
        if prev is not None:
            tgt_same = all(abs(a - b) <= 1e-6 for a, b in zip(s["tgt"], prev["tgt"]))
            still = all(abs(a - b) <= move_tol for a, b in zip(s["q"], prev["q"]))
            if tgt_same and still:
                if run_start is None:
                    run_start = prev
            else:
                if run_start is not None and (prev["t"] - run_start["t"]) >= settle_s:
                    frames.append((run_start, prev))
                run_start = None
        prev = s
    if run_start is not None and (prev["t"] - run_start["t"]) >= settle_s:
        frames.append((run_start, prev))

    seq, states = [], []
    for a, b in frames:
        off = {n: v for n, v in zip(names, b["q"]) if abs(v) > 0.05}
        held = round(b["t"] - a["t"], 1)
        seq.append((f"t={b['t']:.0f}s", off, 1.5, max(0.5, min(held, 2.0))))
        states.append(b)
    return seq, init, states


def export_sequence(path, settle_s=0.8):
    seq, init, states = keyframes_from_log(path, settle_s=settle_s)
    print(f"# recovered from {os.path.basename(path)} -- start pose {init.get('pose')!r}, "
          f"skid={init.get('skid')}, knee={init.get('knee')!r}, session {init.get('when')}")
    if not seq:
        print("# no held poses found (the body never settled at a fixed target)")
        return seq
    print(f"# {len(seq)} poses the session actually HELD (auto-recovered, no manual record):")
    for (label, off, mv, hd), st in zip(seq, states):
        print(f"#   {label:>8s}  pelvis z {st['z']:.3f}  up {st['up']:+.2f}  contacts {' '.join(st['con'])}")
    print("[")
    for label, off, mv, hd in seq:
        print(f"    ({label!r}, {off}, {mv}, {hd}),")
    print("]")
    return seq


def run_sequence_from_log(path, settle_s=0.8, play=3.0, mu=0.7):
    """feed a past session's recovered keyframes back through the STUDY's
    evaluator: getup_v6.run_sequence under the deploy servo model -- does what
    was posed by hand hold up when the real servos have to do it?"""
    import getup_v6 as G
    seq, init, _ = keyframes_from_log(path, settle_s=settle_s)
    if not seq:
        print("no held poses recovered from that log")
        return None
    p = cad_rom_params(skid=bool(init.get("skid")), knee=init.get("knee", "both"))
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"puppet_run_{os.getpid()}.xml")
    with open(xml, "w") as fh:
        fh.write(build_xml(p))
    start = init.get("pose", "stand")
    start = start if start in ("supine", "prone") else "supine"
    print(f"replaying {len(seq)} recovered poses from {start} under the deploy servo model "
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
    ap.add_argument("--knee", choices=("both", "fwd", "bwd"), default="both",
                    help="'both' (default) = the knee as the CAD allows it in both "
                         "directions (-130 flexion .. +95 backward); 'fwd'/'bwd' are the "
                         "single-direction knees the walking studies use")
    ap.add_argument("--log", default=None,
                    help="session log path (default sim/puppet_sessions/<stamp>.jsonl)")
    ap.add_argument("--keyframes", "--export", dest="keyframes", metavar="LOG",
                    help="recover the keyframes of a PAST session from its log "
                         "(the poses it actually held) and print them as a getup_v6 sequence")
    ap.add_argument("--run", metavar="LOG",
                    help="recover a past session's keyframes and feed them back through the "
                         "study's evaluator (getup_v6.run_sequence, deploy servo model)")
    ap.add_argument("--settle", type=float, default=0.8,
                    help="a pose counts as HELD (and so a keyframe) after this many seconds "
                         "of unchanged targets and a still body (default 0.8)")
    ap.add_argument("--key-rate", type=float, default=KEY_RATE_HZ, dest="key_rate",
                    help=f"held-key repeat, steps per second (default {KEY_RATE_HZ:.0f}; "
                         f"at {KEY_STEP:.0f} deg a step that is "
                         f"{KEY_RATE_HZ * KEY_STEP:.0f} deg/s)")
    a = ap.parse_args()

    if a.keyframes:
        export_sequence(a.keyframes, settle_s=a.settle)
        return
    if a.run:
        run_sequence_from_log(a.run, settle_s=a.settle)
        return

    from PyQt6 import QtCore, QtGui, QtWidgets

    # camera state, driven entirely by the mouse on the view (below)
    CAM0 = dict(az=135.0, el=-16.0, dist=1.3, ox=0.0, oy=0.0, oz=0.0)
    cs = dict(CAM0)

    class View(QtWidgets.QLabel):
        """the 3D view, and the camera's input surface: left-drag orbits,
        right/middle-drag pans, the wheel zooms -- the usual 3D conventions,
        so the camera behaves like every other viewer."""

        def __init__(self):
            super().__init__()
            self._last = None
            self._btn = None

        def mousePressEvent(self, e):
            self._last = e.position()
            self._btn = e.button()

        def mouseReleaseEvent(self, e):
            self._last = None

        def mouseMoveEvent(self, e):
            if self._last is None:
                return
            p = e.position()
            dx, dy = p.x() - self._last.x(), p.y() - self._last.y()
            self._last = p
            if self._btn == QtCore.Qt.MouseButton.LeftButton:
                cs["az"] = (cs["az"] - dx * 0.4) % 360.0
                cs["el"] = max(-89.0, min(89.0, cs["el"] - dy * 0.4))
            else:                                   # pan in the camera plane
                az, el = math.radians(cs["az"]), math.radians(cs["el"])
                right = (-math.sin(az), math.cos(az), 0.0)
                up = (-math.cos(az) * math.sin(el), -math.sin(az) * math.sin(el), math.cos(el))
                s = cs["dist"] * 0.0015
                for i, k in enumerate(("ox", "oy", "oz")):
                    cs[k] += (-right[i] * dx + up[i] * dy) * s

        def wheelEvent(self, e):
            cs["dist"] = max(0.25, min(6.0, cs["dist"] * math.exp(-e.angleDelta().y() * 0.0015)))

        def mouseDoubleClickEvent(self, e):
            cs.update(CAM0)                         # back to the default view

    pup = Puppet(cad_rom_params(skid=a.skid, knee=a.knee), a.pose)
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

    # ---- left: the live 3D view (and the camera's mouse surface)
    view = View()
    view.setMinimumSize(520, 660)
    view.setStyleSheet("background:#111;")
    view.setToolTip("left-drag: orbit   right-drag: pan   wheel: zoom   double-click: reset view")
    outer.addWidget(view)

    # ---- right: a slider per joint + buttons + status
    right = QtWidgets.QVBoxLayout()
    outer.addLayout(right)
    right.addWidget(QtWidgets.QLabel(
        "<b>joint targets</b> (deg) — gravity is on; the servo settles the pose.<br>"
        "<span style='color:#2a7'>green</span> = the angle actually reached."))

    # Laid out like the robot: neck on top, then each joint level as one row
    # with left and right side by side, hip down to ankle.
    grid = QtWidgets.QGridLayout()
    grid.setHorizontalSpacing(14)
    right.addLayout(grid)
    achieved = {}

    def make_slider(name):
        """slider + target + achieved for one joint, as a single row widget."""
        lo, hi = pup.limits_deg(name)
        box = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(box)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(4)
        sl = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        sl.setMinimum(int(round(lo)))
        sl.setMaximum(int(round(hi)))
        sl.setValue(int(round(pup.targets[name])))
        sl.setMinimumWidth(170)
        tgt = QtWidgets.QLabel(f"{pup.targets[name]:+.0f}°")
        tgt.setFixedWidth(42)
        tgt.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        ach = QtWidgets.QLabel("")
        ach.setFixedWidth(42)
        ach.setStyleSheet("color:#2a7;")
        ach.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)

        def on_change(v, nm=name, lab=tgt):
            pup.set_target(nm, float(v))
            lab.setText(f"{v:+d}°")
        sl.valueChanged.connect(on_change)

        h.addWidget(sl)
        h.addWidget(tgt)
        h.addWidget(ach)
        achieved[name] = (sl, ach, tgt)
        return box

    row = 0
    if "neck_yaw" in pup.targets:                     # head, at the top
        grid.addWidget(QtWidgets.QLabel("<b>neck</b>"), row, 0)
        grid.addWidget(make_slider("neck_yaw"), row, 1, 1, 2)
        row += 1
    hdr_l = QtWidgets.QLabel("<b>left</b>")
    hdr_r = QtWidgets.QLabel("<b>right</b>")
    for w in (hdr_l, hdr_r):
        w.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
    grid.addWidget(hdr_l, row, 1)
    grid.addWidget(hdr_r, row, 2)
    row += 1
    # top of the leg to the bottom
    for level in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll"):
        grid.addWidget(QtWidgets.QLabel(level.replace("_", " ")), row, 0)
        grid.addWidget(make_slider(f"L_{level}"), row, 1)
        grid.addWidget(make_slider(f"R_{level}"), row, 2)
        row += 1

    # ---- buttons: body poses only. There is deliberately no record button --
    # the session is sampled continuously and its keyframes are recovered from
    # the log afterwards (--keyframes), so nothing depends on pressing
    # anything at the right moment.
    btns = QtWidgets.QHBoxLayout()
    right.addLayout(btns)

    def refresh_sliders():
        for nm, (sl, _ach, tgt) in achieved.items():
            sl.blockSignals(True)
            sl.setValue(int(round(pup.targets[nm])))
            sl.blockSignals(False)
            tgt.setText(f"{pup.targets[nm]:+.0f}°")

    for pose_name in POSES:
        b = QtWidgets.QPushButton(pose_name)
        b.clicked.connect(lambda _, po=pose_name: (
            pup.log(dict(cmd="pose", t=round(float(pup.d.time), 3), pose=po)),
            pup.set_body(po), refresh_sliders()))
        btns.addWidget(b)

    status = QtWidgets.QLabel("")
    status.setStyleSheet("font-family:monospace; color:#666;")
    right.addWidget(status)
    right.addStretch(1)

    # ---- keyboard: KEYMAP drives the same targets the sliders do. Installed
    # on the application so a key works wherever the focus is (a slider that
    # has focus would otherwise eat the keys itself).
    #
    # The repeat is OURS, not the OS's: holding a key starts a timer that steps
    # every 1000/rate ms with no initial delay, so a sweep begins instantly and
    # runs at a usable speed instead of the system's typing repeat.
    def apply_key(code):
        hit = KEYCODES.get(code)
        if hit is None:
            return False
        name, delta = hit
        if name not in pup.targets:
            return False
        lo, hi = pup.limits_deg(name)
        cur = pup.targets[name]
        val = 0.0 if delta is None else max(lo, min(hi, cur + delta))
        pup.set_target(name, val)
        sl, _ach, tgt = achieved[name]
        sl.blockSignals(True)
        sl.setValue(int(round(val)))
        sl.blockSignals(False)
        tgt.setText(f"{val:+.0f}°")
        return True

    held = set()                                   # key codes currently down
    repeat = QtCore.QTimer()
    repeat.setInterval(max(1, int(round(1000.0 / a.key_rate))))

    def on_repeat():
        for code in list(held):
            apply_key(code)
    repeat.timeout.connect(on_repeat)

    class Keys(QtCore.QObject):
        def eventFilter(self, obj, ev):
            t = ev.type()
            if t == QtCore.QEvent.Type.KeyPress:
                if ev.isAutoRepeat():
                    return True                    # swallow the OS repeat: ours is faster
                if ev.modifiers() != QtCore.Qt.KeyboardModifier.NoModifier:
                    return False
                hit = KEYCODES.get(ev.key())
                if hit is None:
                    return False
                apply_key(ev.key())                # act on the press itself
                if hit[1] is not None:             # +/- repeat; the zero keys do not
                    held.add(ev.key())
                    if not repeat.isActive():
                        repeat.start()
                return True
            if t == QtCore.QEvent.Type.KeyRelease:
                if ev.isAutoRepeat():
                    return True
                if ev.key() in held:
                    held.discard(ev.key())
                    if not held:
                        repeat.stop()
                    return True
            elif t in (QtCore.QEvent.Type.WindowDeactivate,
                       QtCore.QEvent.Type.ApplicationDeactivate):
                held.clear()                       # never leave a joint sweeping
                repeat.stop()
            return False
    keys = Keys()
    app.installEventFilter(keys)

    # ---- physics on a worker thread; rendering + widgets on the Qt thread
    def sim_loop():
        while not pup.quit:
            pup.step(16)                 # ~16 ms of sim per pass
            time.sleep(0.004)
    threading.Thread(target=sim_loop, daemon=True).start()

    renderer = mujoco.Renderer(pup.m, 660, 520)
    cam = mujoco.MjvCamera()

    def tick():
        z, up, com, contacts, t = pup.snapshot()
        with pup.lock:
            # the camera follows the body, offset by whatever the mouse panned
            cam.azimuth, cam.elevation, cam.distance = cs["az"], cs["el"], cs["dist"]
            cam.lookat[:] = [float(pup.d.qpos[0]) + cs["ox"],
                             float(pup.d.qpos[1]) + cs["oy"],
                             max(0.12, float(pup.d.qpos[2]) * 0.5) + cs["oz"]]
            renderer.update_scene(pup.d, cam)
            frame = renderer.render().copy()
            act = {n: pup.actual_deg(n) for n in pup.names}
        img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0],
                           frame.strides[0], QtGui.QImage.Format.Format_RGB888)
        view.setPixmap(QtGui.QPixmap.fromImage(img))
        for nm, (_sl, lab, _tgt) in achieved.items():
            lab.setText(f"{act[nm]:+.0f}°")
        status.setText(f"t {t:6.1f}s   pelvis z {z:.3f}   up {up:+.2f}   CoM x {com[0]:+.3f}\n"
                       f"contacts: {' '.join(contacts)}")

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(33)                      # ~30 fps

    # the session recorder: samples the whole timeline at 5 Hz so a past
    # session's keyframes can be recovered later without anyone having
    # pressed anything.
    sampler = QtCore.QTimer()
    sampler.timeout.connect(pup.sample)
    sampler.start(200)

    print(f"joint puppet: v6/{'v7' if pup.p.torso_v7 else 'v6'} body"
          f"{', pelvis skid' if a.skid else ''}, start '{a.pose}' -- "
          f"{len(pup.names)} sliders, gravity on.", flush=True)
    print(f"  logging the session to {log_path}", flush=True)
    print(f"  afterwards: sim/joint_puppet.py --keyframes {log_path}", flush=True)
    win.resize(1080, 700)
    win.show()
    win.raise_()                 # macOS: a window from a terminal-launched
    win.activateWindow()         # process otherwise opens behind the terminal
    app.exec()
    pup.quit = True
    pup.close_log()


if __name__ == "__main__":
    main()
