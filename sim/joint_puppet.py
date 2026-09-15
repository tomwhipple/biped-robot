"""Interactive joint-puppet for the v6/v7 body. Every joint is a slider; the
robot lives under gravity and settles on the floor around whatever pose the
sliders command, so you can FEEL which poses are reachable, which collapse,
and where the get-up path pinches -- the thing the keyframe searches can't
show. Gravity always applies; the sliders drive the plant's own position
actuators (same servos, same limits -- no stronger puppet string than the
real joints).

Run (on macOS use mjpython, the MuJoCo GL wrapper -- plain `python` cannot
open the 3D window there):

    .venv/bin/mjpython sim/joint_puppet.py                      # v7 body, standing
    .venv/bin/mjpython sim/joint_puppet.py --skid               # with the pelvis skid
    .venv/bin/mjpython sim/joint_puppet.py --pose supine        # start fallen
    .venv/bin/python  sim/joint_puppet.py --no-window           # headless / any platform

Controls: drag a joint slider to a target (deg) and the servo tracks it --
gravity, contacts and inertia act the whole time. Presets set the free body
+ zero the rest. "record pose" appends the CURRENT settled pose to a keyframe
list (shown in the console) that you can paste into getup_v6*/a sequence --
turns this into a get-up path designer: pose the frames, then replay them.

Camera: left-drag rotate, right-drag pan, scroll zoom (the MuJoCo viewer).
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
os.environ.setdefault("MUJOCO_GL", "glfw")

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
    "stand":  ((1.0, 0.0, 0.0, 0.0), None),                       # z from the plant
    "supine": ((math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0), 0.13),
    "prone":  ((math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0), 0.13),
    "side_L": ((math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0), 0.13),
    "side_R": ((math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0), 0.13),
}


class Puppet:
    def __init__(self, p: DesignParams, pose: str):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build_xml(p))
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()          # guards qpos/ctrl between the
        self.targets_deg = {}                 # GUI slider thread and the sim
        self.recorded = []
        self._quit = False
        for i in range(self.m.nu):
            a = self.m.actuator(i)
            lo, hi = self.m.jnt_range[a.trnid[0]]
            if math.degrees(hi - lo) > 0.1:   # an actual joint, not a weld
                self.targets_deg[a.name] = 0.0
        self._act_index = {self.m.actuator(i).name: i for i in range(self.m.nu)}
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

    def _sync_targets(self):
        """set the sliders from the current joint state (after a pose jump)."""
        with self.lock:
            for name, i in self._act_index.items():
                adr = self.m.joint(self.m.actuator(i).trnid[0]).qposadr[0]
                self.targets_deg[name] = math.degrees(self.d.qpos[adr])

    def record(self):
        with self.lock:
            q = {n: round(math.degrees(self.d.qpos[self.m.joint(
                self.m.actuator(self._act_index[n]).trnid[0]).qposadr[0]]), 1)
                for n in JOINTS}
        self.recorded.append(q)
        line = ", ".join(f"{n.split('_', 1)[0]}_{n.split('_', 1)[1]} {v:+.0f}" for n, v in q.items() if abs(v) > 0.5)
        up = self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]
        print(f"[frame {len(self.recorded)}] pelvis z {self.d.qpos[2]:.3f} up {up:+.2f} | {line or 'standing'}")

    def dump(self):
        print("recorded keyframes (paste-ready offsets):")
        for i, f in enumerate(self.recorded):
            off = {k: v for k, v in f.items()}
            print(f"  ({i+1}, {off})")

    def set_joints(self, pose: dict):
        for n, v in pose.items():
            if n in self.targets_deg:
                self.targets_deg[n] = v

    def state(self):
        with self.lock:
            com = self.d.subtree_com[0].copy()
            up = float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2])
            contacts = sorted({self.m.body(self.m.geom_bodyid[g]).name
                               for i in range(self.d.ncon) for g in
                               (self.d.contact[i].geom1, self.d.contact[i].geom2)
                               if self.m.body(self.m.geom_bodyid[g]).name != "world"
                               and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})
            return self.d.qpos[2], up, com, contacts, self.d.time

    def step_once(self):
        """one physics tick with the current slider targets on the servos. The
        panel thread sets targets_deg concurrently; targets are plain floats
        (atomic under the GIL) so no lock is needed on the hot loop."""
        for name, i in self._act_index.items():
            self.d.ctrl[i] = math.radians(self.targets_deg.get(name, 0.0))
        mujoco.mj_step(self.m, self.d)

    def run_viewer(self):
        """main thread: launch_passive (and GLFW generally) MUST run on the
        main thread on macOS, and under `mjpython`. This blocks until the
        window closes."""
        assert HAVE_VIEWER, "mujoco.viewer unavailable"
        with mujoco_viewer.launch_passive(self.m, self.d) as v:
            while v.is_running() and not self._quit:
                self.step_once()
                v.sync()
                time.sleep(self.m.opt.timestep)
        self._quit = True


def build_panel(pup: Puppet):
    """slider panel next to the 3D viewer. Prefers matplotlib (TkAgg window);
    falls back to a zero-dependency terminal REPL if matplotlib is absent, so
    `pip install matplotlib` is optional polish rather than a requirement.
    Each control is a joint TARGET (deg) -- the servo + gravity decide how
    close the body actually gets, and that gap is the point: a joint that will
    not reach its slider is one the servo cannot hold there against the floor."""
    try:
        import matplotlib  # noqa: F401
        return _panel_matplotlib(pup)
    except ImportError:
        print("matplotlib not present -- terminal control (pip install matplotlib for sliders).",
              flush=True)
        return _panel_repl(pup)


def _panel_repl(pup: Puppet):
    """no extra deps: type `L_knee -90`, `R_ankle 20`, `pose supine`,
    `record`, `dump`, `state`, or `quit`. Gravity runs the whole time
    (the viewer loop on the main thread keeps stepping the sim)."""
    print("commands: <joint> <deg> | pose %s | record | dump | state | quit"
          % "/".join(POSES), flush=True)
    while not pup._quit:
        try:
            print("joint> ", end="", flush=True)
            line = sys.stdin.readline().strip()
            if line == "":
                break                                           # EOF
        except EOFError:
            break
        if not line:
            continue
        w = line.split()
        if w[0] in ("quit", "exit", "q"):
            break
        if w[0] == "pose" and len(w) == 2 and w[1] in POSES:
            pup._set_body(w[1]); pup._sync_targets(); continue
        if w[0] == "record":
            pup.record(); continue
        if w[0] == "dump":
            pup.dump(); continue
        if w[0] == "state":
            z, up, com, contacts, t = pup.state()
            print(f"  t {t:.1f}s pelvis z {z:.3f} up {up:+.2f} CoM x {com[0]:+.3f} | {' '.join(contacts)}")
            continue
        if len(w) == 2 and w[0] in pup.targets_deg:
            try:
                pup.targets_deg[w[0]] = float(w[1])
            except ValueError:
                print("  want <joint> <deg>")
            continue
        print("  unknown joint", w[0], "-- one of:", ", ".join(pup.targets_deg))
    pup._quit = True


def _panel_matplotlib(pup: Puppet):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button

    order = JOINTS + [n for n in pup.targets_deg if n not in JOINTS]
    n_rows = len(order)
    fig, axes = plt.subplots(n_rows, 1, figsize=(5.6, 0.42 * n_rows + 1.6))
    fig.subplots_adjust(left=0.22, right=0.98, top=0.92, bottom=0.14, hspace=0.0)
    fig.canvas.manager.set_window_title("v6/v7 joint puppet — gravity on")
    fig.suptitle("joint targets (deg) — gravity on; achieved pose in the status line", fontsize=9)

    sliders, readouts = {}, {}
    for ax, n in zip(axes, order):
        lo, hi = pup.m.jnt_range[pup.m.joint(pup.m.actuator(pup._act_index[n]).trnid[0])]
        sl = Slider(ax, n, math.degrees(lo), math.degrees(hi),
                    valinit=round(pup.targets_deg.get(n, 0.0)), valstep=0.5)
        sl.label.set_fontsize(8)
        sl.valtext.set_fontsize(8)
        def handler(v, name=n):
            pup.targets_deg[name] = v
        sl.on_changed(handler)
        sliders[n] = sl
        readouts[n] = ax.text(1.02, 0.15, "", transform=ax.transAxes, fontsize=8, color="#555")

    # preset body-pose buttons along the bottom
    btn_axes = []
    poses = list(POSES) + ["record", "dump"]
    for i, name in enumerate(poses):
        axb = fig.add_axes([0.03 + i * 0.115, 0.015, 0.105, 0.05])
        b = Button(axb, name)
        btn_axes.append(axb)
        if name in POSES:
            def on_pose(ev, po=name):
                pup._set_body(po)
                pup._sync_targets()
                for n2, sc in sliders.items():
                    sc.set_val(round(pup.targets_deg[n2]))
            b.on_clicked(on_pose)
        elif name == "record":
            b.on_clicked(lambda ev: pup.record())
        else:
            b.on_clicked(lambda ev: pup.dump())
        # keep a ref
        btn_axes.append(b)

    stat = fig.text(0.03, 0.075, "", fontsize=8, family="monospace")

    def tick(_frame):
        z, up, com, contacts, t = pup.state()
        stat.set_text(f"t {t:6.1f}s  pelvis z {z:.3f}  up {up:+.2f}  CoM x {com[0]:+.3f}\n{ ' '.join(contacts) }")
        for n, ro in readouts.items():
            adr = pup.m.joint(pup.m.actuator(pup._act_index[n]).trnid[0]).qposadr[0]
            ro.set_text(f"{math.degrees(pup.d.qpos[adr]):+.0f}°")
        return [stat, *readouts.values()]

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, tick, interval=80, blit=False, cache_frame_data=False)
    _ = anim
    plt.show()
    pup._quit = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="stand", choices=list(POSES))
    ap.add_argument("--skid", action="store_true")
    ap.add_argument("--knee", choices=("fwd", "bwd"), default="fwd")
    ap.add_argument("--no-window", action="store_true",
                    help="no 3D window: drive the panel alone (headless / debugging). "
                         "A background thread steps the sim instead.")
    a = ap.parse_args()
    p = DesignParams(skid=a.skid, knee=a.knee)
    pup = Puppet(p, a.pose)
    if a.no_window or not HAVE_VIEWER:
        # headless: a background thread steps the sim (gravity) while the
        # panel (REPL or sliders) runs on the main thread.
        def _loop():
            while not pup._quit:
                pup.step_once()
                time.sleep(pup.m.opt.timestep)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()
        _panel_repl(pup)
        return
    # windowed: macOS wants GLFW (launch_passive) AND TkAgg (matplotlib) on
    # the MAIN thread -- they cannot share it. Run the sim window on the main
    # thread and the panel on a worker:
    #  - matplotlib sliders: its `plt.show()` must be main-thread too, which
    #    collides with the viewer, so even with matplotlib installed the
    #    zero-dep REPL is the reliable on-screen path on macOS.
    panel = threading.Thread(target=_panel_repl, args=(pup,), daemon=True)
    panel.start()
    try:
        pup.run_viewer()        # blocks until the 3D window closes
    finally:
        pup._quit = True
        pup.dump()


if __name__ == "__main__":
    main()
