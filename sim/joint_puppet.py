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

    .venv/bin/python sim/joint_puppet.py                          # v7 body, standing
    .venv/bin/python sim/joint_puppet.py --skid                   # with the pelvis skid
    .venv/bin/python sim/joint_puppet.py --pose supine            # start lying on its back
    .venv/bin/python sim/joint_puppet.py --pose prone             # start face-down

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
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", os.environ.get("MUJOCO_GL", "disable"))
os.environ.pop("MUJOCO_GL", None) if os.environ.get("MUJOCO_GL") == "disable" else None

import mujoco  # noqa: E402

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
        self._act = {}
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

    def settle(self, seconds=0.6):
        """hold the current targets and let gravity work; returns nothing."""
        for _ in range(int(seconds / self.m.opt.timestep)):
            for name, i in self._act.items():
                self.d.ctrl[i] = math.radians(self.targets_deg[name])
            mujoco.mj_step(self.m, self.d)

    def settle_live(self, seconds=2.0):
        """settle in real time (each sim step sleeps one timestep) so an
        attached viewer shows it happen, then report the settled state."""
        for _ in range(int(seconds / self.m.opt.timestep)):
            for name, i in self._act.items():
                self.d.ctrl[i] = math.radians(self.targets_deg[name])
            mujoco.mj_step(self.m, self.d)
            time.sleep(self.m.opt.timestep)
        self.report()

    def report(self):
        com = self.d.subtree_com[0]
        up = self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]
        ncon = int(self.d.ncon)
        contacts = sorted({self.m.body(self.m.geom_bodyid[g]).name
                           for i in range(ncon) for g in (self.d.contact[i].geom1, self.d.contact[i].geom2)
                           if self.m.body(self.m.geom_bodyid[g]).name != "world"
                           and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})
        print(f"  pelvis z {self.d.qpos[2]:.3f}  up {up:+.2f}  CoM x {com[0]:+.3f} z {com[2]:.3f}  contacts {contacts}")

    def status_line(self):
        return "  ".join(f"{n.split('_',1)[0][:1]}{n.split('_',1)[1][:3]} {self._actual(n):+5.0f}"
                         for n in JOINTS if abs(self._actual(n)) > 0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="stand", choices=list(POSES))
    ap.add_argument("--skid", action="store_true")
    ap.add_argument("--knee", choices=("fwd", "bwd"), default="fwd")
    ap.add_argument("--live", action="store_true", help="settle in real time (wall-clock), watchable")
    a = ap.parse_args()
    p = DesignParams(skid=a.skid, knee=a.knee)
    pup = Puppet(p, a.pose)
    print(f"puppet: v6/{'v7' if p.torso_v7 else 'v6'} body{', skid' if a.skid else ''}, pose {a.pose} -- gravity on")
    pup.report()
    print("targets are the sim's own actuators; 'state'/'record'/'dump'/'pose X'/'quit'. 'live' for real-time.")
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
            pup._set_body(w[1]); pup.report(); continue
        if c in GROUPS and len(w) == 2:
            try:
                v = float(w[1])
            except ValueError:
                print("  want e.g. 'knees -90'"); continue
            for n in GROUPS[c]:
                pup.targets_deg[n] = v
            pup.settle()
            pup.report(); print("   " + pup.status_line()); continue
        if len(w) == 2 and c in pup.targets_deg:
            try:
                pup.targets_deg[c] = float(w[1])
            except ValueError:
                print("  want <joint> <deg>"); continue
            pup.settle()
            pup.report(); print("   " + pup.status_line()); continue
        if len(w) == 1 and c == "*":
            print("  '* -90' sets every joint"); continue
        if len(w) == 2 and c == "*":
            try:
                v = float(w[1])
                for n in pup.targets_deg:
                    pup.targets_deg[n] = v
                pup.settle(); pup.report(); print("   " + pup.status_line())
            except ValueError:
                pass
            continue
        print("  unknown -- joints:", ", ".join(list(GROUPS) + JOINTS + ["neck_yaw"]))
    pup._quit = True
    pup.dump()


if __name__ == "__main__":
    main()
