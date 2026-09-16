"""Interactive joint-puppet for the v6/v7 body. Gravity and contacts act the
whole time. You type a joint target; the servo tracks it against the floor --
so you can FEEL which poses are reachable, which collapse, and where the
get-up path pinches (the thing the keyframe searches can't show). No stronger
puppet string than the real joints: the targets drive the plant's own position
actuators (same servos, same limits).

The design the repo already trusts for "drive the joints by hand": the body
pose is the FALL (you start it lying flat), and the joints follow what you're
trying -- exactly what play_probe.py does for the real servo play, here in
the sim instead of on the bench.

    .venv/bin/python sim/joint_puppet.py                          # v7 body, standing (terminal)
    .venv/bin/python sim/joint_puppet.py --skid                   # with the pelvis skid
    .venv/bin/python sim/joint_puppet.py --pose supine            # start lying on its back
    .venv/bin/python sim/joint_puppet.py --pose prone             # start face-down
    .venv/bin/mjpython sim/joint_puppet.py --pose supine --view   # + a live-render window
                                                                  # (GL needs mjpython on macOS)

Commands (one per line):
    <joint> <deg>      set one joint target          e.g.  L_knee -90
    hips <deg>         both hip_pitch                e.g.  hips -110
    knees <deg> | ankles <deg> | rolls <deg> | yaws <deg>
    pose <stand|supine|prone|side_L|side_R>          jump the free joint
    state              pelvis height, uprightness, CoM, which bodies touch the floor
    record             store the settled pose as a get-up keyframe
    dump               print all recorded frames as paste-ready getup_v6 offsets
    quit

Angles are degrees, sim signs: hip_pitch - = thigh forward, knee - = flex,
ankle + = toe down. A joint that won't reach your target is one the servo
can't hold against the floor -- that gap is the signal you're after.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", os.environ.get("MUJOCO_GL", "disable"))
os.environ.pop("MUJOCO_GL", None) if os.environ.get("MUJOCO_GL") == "disable" else None

import mujoco  # noqa: E402
try:
    import mujoco.viewer as mujoco_viewer
    HAVE_VIEWER = True
except Exception:  # noqa: BLE001
    HAVE_VIEWER = False

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import v6_kin as VK  # noqa: E402

JOINTS = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
          "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]

# body poses for the freejoint: (quat wxyz, z). supine/prone = fallen.
POSES = {
    "stand":  ((1.0, 0.0, 0.0, 0.0), None),              # z from the plant
    "supine": ((math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0), 0.13),
    "prone":  ((math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0), 0.13),
    "side_L": ((math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0), 0.13),
    "side_R": ((math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0), 0.13),
}

GROUPS = {  # plural shorthands -> the per-side joint names
    "yaws": ["L_hip_yaw", "R_hip_yaw"],
    "rolls": ["L_hip_roll", "R_hip_roll"],
    "hips": ["L_hip_pitch", "R_hip_pitch"],
    "knees": ["L_knee", "R_knee"],
    "ankles": ["L_ankle", "R_ankle"],
    "anklerolls": ["L_ankle_roll", "R_ankle_roll"],
}


class Puppet:
    def __init__(self, p: DesignParams, pose: str):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build_xml(p))
        self.d = mujoco.MjData(self.m)
        self.targets_deg = {}
        self.recorded = []
        self._quit = False
        self.lock = threading.Lock()      # serializes the renderer thread and
        self._act = {}                    # the REPL's settle calls on self.d
        for i in range(self.m.nu):
            a = self.m.actuator(i)
            lo, hi = self.m.jnt_range[a.trnid[0]]
            if math.degrees(hi - lo) > 0.1:                 # a real joint, not a weld
                self.targets_deg[a.name] = 0.0
                self._act[a.name] = i
        self._set_body(pose)
        self._sync_targets()

    def _set_body(self, pose: str):
        quat, z = POSES[pose]
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self.d.qpos[0:3] = [0.0, 0.0, z if z is not None else self.p.z_yaw_above_sole]
            self.d.qpos[3:7] = quat
            self.d.qvel[:] = 0.0
            mujoco.mj_forward(self.m, self.d)
        self._sync_targets()

    def _sync_targets(self):
        for name, i in self._act.items():
            adr = self.m.joint(self.m.actuator(i).trnid[0]).qposadr[0]
            self.targets_deg[name] = math.degrees(self.d.qpos[adr])

    def _actual(self, name):
        adr = self.m.joint(self.m.actuator(self._act[name]).trnid[0]).qposadr[0]
        return math.degrees(self.d.qpos[adr])

    def record(self):
        q = {n: round(self._actual(n), 1) for n in JOINTS}
        self.recorded.append(q)
        line = ", ".join(f"{n[2:]} {v:+.0f}" for n, v in q.items() if abs(v) > 0.5)
        up = self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]
        print(f"[frame {len(self.recorded)}] pelvis z {self.d.qpos[2]:.3f} up {up:+.2f} | {line or 'standing'}")

    def dump(self):
        print("recorded keyframes (getup_v6-style offsets):")
        for i, f in enumerate(self.recorded):
            print(f"  ({i+1}, {f})")

    def step(self):
        """one physics tick. MUST be called with self.lock held when a viewer
        is running: the viewer reads qpos between steps and a mid-step qpos
        update under it segfaults the GL render (the crash dialogs)."""
        for name, i in self._act.items():
            self.d.ctrl[i] = math.radians(self.targets_deg[name])
        mujoco.mj_step(self.m, self.d)

    def settle(self, seconds=0.6):
        """hold the current targets and let gravity work. Takes the lock so a
        running viewer never sees a half-stepped state."""
        with self.lock:
            for _ in range(int(seconds / self.m.opt.timestep)):
                self.step()

    def settle_live(self, seconds=2.0):
        """settle in real time so an attached viewer shows it, then report."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self.lock:
                self.step()
            time.sleep(self.m.opt.timestep)
        self.report()

    def set_target(self, name: str, deg: float):
        """thread-safe target set (the REPL and the viewer both reach here)."""
        with self.lock:
            if name in self.targets_deg:
                self.targets_deg[name] = float(deg)

    def report(self):
        with self.lock:
            com = self.d.subtree_com[0].copy()
            up = float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2])
            contacts = sorted({self.m.body(self.m.geom_bodyid[g]).name
                               for i in range(int(self.d.ncon)) for g in (self.d.contact[i].geom1, self.d.contact[i].geom2)
                               if self.m.body(self.m.geom_bodyid[g]).name != "world"
                               and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})
            z, t = float(self.d.qpos[2]), float(self.d.time)
        print(f"  pelvis z {z:.3f}  up {up:+.2f}  CoM x {com[0]:+.3f} z {com[2]:.3f}  t {t:.1f}s  contacts {contacts}")

    def _actual(self, name):
        with self.lock:
            adr = self.m.joint(self.m.actuator(self._act[name]).trnid[0]).qposadr[0]
            return math.degrees(float(self.d.qpos[adr]))

    def record(self):
        q = {n: round(self._actual(n), 1) for n in JOINTS}
        self.recorded.append(q)
        line = ", ".join(f"{n[2:]} {v:+.0f}" for n, v in q.items() if abs(v) > 0.5)
        with self.lock:
            up = self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]
            z = float(self.d.qpos[2])
        print(f"[frame {len(self.recorded)}] pelvis z {z:.3f} up {up:+.2f} | {line or 'standing'}")

    def status_line(self):
        return "  ".join(f"{n.split('_',1)[0][:1]}{n.split('_',1)[1][:3]} {self._actual(n):+5.0f}"
                         for n in JOINTS if abs(self._actual(n)) > 0.5)


class LiveView:
    """the live picture: a MuJoCo viewer window sharing the sim, run on the
    main thread via `launch_passive` (which is the supported macOS path -- it
    handles the GLFW main-thread requirement; that's why --view runs under
    mjpython). The terminal REPL moves joints from a worker thread; the window
    just shows what the sim is doing each frame."""
    def __init__(self, pup: Puppet):
        self.pup = pup

    def run(self):
        assert HAVE_VIEWER, "mujoco.viewer unavailable"
        pup = self.pup
        with mujoco_viewer.launch_passive(pup.m, pup.d) as v:
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.3, -16.0, 135.0
            while v.is_running() and not pup._quit:
                with pup.lock:
                    for _ in range(int(0.016 / pup.m.opt.timestep)):
                        pup.step()
                    v.cam.lookat[:] = [float(pup.d.qpos[0]), float(pup.d.qpos[1]),
                                       max(0.12, float(pup.d.qpos[2]) * 0.5)]
                    v.sync()          # inside the lock: the render sees a stepped, consistent state
                time.sleep(0.005)
        pup._quit = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="stand", choices=list(POSES))
    ap.add_argument("--skid", action="store_true")
    ap.add_argument("--knee", choices=("fwd", "bwd"), default="fwd")
    ap.add_argument("--live", action="store_true", help="settle in real time (wall-clock), watchable")
    ap.add_argument("--view", action="store_true",
                    help="open a live-render GLFW window next to the terminal. "
                         "On macOS GL insists on the main thread, so --view runs the "
                         "script under mjpython: `.venv/bin/mjpython sim/joint_puppet.py --view`.")
    a = ap.parse_args()
    p = DesignParams(skid=a.skid, knee=a.knee)
    pup = Puppet(p, a.pose)
    print(f"puppet: v6/{'v7' if p.torso_v7 else 'v6'} body{', skid' if a.skid else ''}, pose {a.pose} -- gravity on")
    pup.report()
    print("targets are the sim's own actuators; 'state'/'record'/'dump'/'pose X'/'quit'. 'live N' settles in real time."
          + ("  The live view is a GLFW window; it opens on the main thread." if a.view else ""))

    if a.view:
        # GLFW must run on the main thread (macOS); the terminal REPL drives
        # the sim from a worker thread. When stdin isn't a real tty the
        # window alone is the whole app (the sim keeps running in this loop).
        def _repl():
            _repl_loop(pup, settle_on_pose=not a.view)
        threading.Thread(target=_repl, daemon=True).start()
        try:
            LiveView(pup).run()                    # blocks until the window closes
        finally:
            pup._quit = True
            pup.dump()
        return

    if not sys.stdin.isatty():
        # launched detached (CI / a pipe): run the command loop on whatever's
        # piped in, then a short settle so a queued 'record' reflects a settled
        # pose, then print the frames and exit -- never idle forever.
        _repl_loop(pup, settle_on_pose=not a.view)
        pup.settle(2.0)
        pup.dump()
        return

    _repl_loop(pup)
    pup.dump()


def _repl_loop(pup: Puppet, settle_on_pose=True):
    """the command loop. When a live view owns the sim's stepping it also owns
    the lock each frame, so settle() calls from here just mark the target and
    let the view do the settling; with no view we settle here."""
    while not pup._quit:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        w = line.split()
        c = w[0]
        if c in ("quit", "exit", "q"):
            break
        if c == "record":
            pup.record(); continue
        if c == "dump":
            pup.dump(); continue
        if c == "live":
            pup.settle_live(float(w[1]) if len(w) > 1 else 2.0); continue
        if c == "state":
            pup.report(); print("   " + pup.status_line()); continue
        if c == "pose" and len(w) == 2 and w[1] in POSES:
            pup._set_body(w[1])
            if settle_on_pose:
                pup.settle()          # fall to the floor under gravity
            pup.report(); continue
        if c in GROUPS and len(w) == 2:
            try:
                v = float(w[1])
            except ValueError:
                print("  want e.g. 'knees -90'"); continue
            for n in GROUPS[c]:
                pup.set_target(n, v)
            pup.settle()
            pup.report(); print("   " + pup.status_line()); continue
        if len(w) == 2 and c in pup.targets_deg:
            try:
                pup.set_target(c, float(w[1]))
            except ValueError:
                print("  want <joint> <deg>"); continue
            pup.settle()
            pup.report(); print("   " + pup.status_line()); continue
        if len(w) == 1 and c == "*":
            print("  '* -90' sets every joint"); continue
        if len(w) == 2 and c == "*":
            try:
                v = float(w[1])
            except ValueError:
                print("  want '* -90'"); continue
            for n in list(pup.targets_deg):
                pup.set_target(n, v)
            pup.settle(); pup.report(); print("   " + pup.status_line())
            continue
        print("  unknown -- joints:", ", ".join(list(GROUPS) + JOINTS + ["neck_yaw"]))
    pup._quit = True


if __name__ == "__main__":
    main()
