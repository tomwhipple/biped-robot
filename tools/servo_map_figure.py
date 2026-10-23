#!/usr/bin/env python3
"""Draw the servo map: every joint of the robot, named, with its servo ID and bus port.

Run:  MUJOCO_GL=cgl .venv/bin/python tools/servo_map_figure.py   (or `make figures`)

A front view of the v6 plant (sim/bimo_biped_v6ar.xml) at its zero pose, with
a marker at each joint's anchor. The names and IDs come from
link/protocol.py's JOINT_NAMES (servo ID = index + 1), the bus joint set of
docs/servo-map.md section 2.1. Writes build/v6/renders/servo_map.png; `make
figures` copies it to docs/img/servo_map.png.
"""
import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from link.protocol import JOINT_NAMES
from paths import build_dir

PLANT = os.path.join(ROOT, "sim", "bimo_biped_v6ar.xml")
PORT_COLOR = {"A": "#1f6fb4", "B": "#c8541a"}
VIEW_H = 0.64                       # m, vertical extent of the orthographic view
LOOKAT_Z = 0.29
PX_H, PX_W = 1100, 700
LABEL_X = 0.215                     # m, label columns at +-LABEL_X
MIN_GAP = 0.03                      # m, between label rows


def port(name):
    # firmware/components/obs/include/obs/bus_map.h kBusPort: A = the right side and the neck
    return "A" if name.startswith("R_") or name == "neck_yaw" else "B"


def render(m, d):
    """Front view, orthographic, background white. Returns (rgb, extent in metres)."""
    m.vis.global_.orthographic = 1
    m.vis.global_.fovy = VIEW_H
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.0, LOOKAT_Z)
    cam.distance, cam.azimuth, cam.elevation = 1.5, 180.0, 0.0
    opt = mujoco.MjvOption()
    opt.sitegroup[:] = 0
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    with mujoco.Renderer(m, PX_H, PX_W) as r:
        r.update_scene(d, camera=cam, scene_option=opt)
        rgb = r.render().copy()
        r.enable_segmentation_rendering()
        r.update_scene(d, camera=cam, scene_option=opt)
        seg = r.render()[..., 0]
    rgb[(seg == -1) | (seg == floor)] = 255
    half_w = VIEW_H * PX_W / PX_H / 2
    return rgb, (-half_w, half_w, LOOKAT_Z - VIEW_H / 2, LOOKAT_Z + VIEW_H / 2)


def spread(zs):
    """Label heights: each joint's own, pushed apart to MIN_GAP, top first."""
    order = np.argsort(zs)[::-1]
    out = np.array(zs, float)
    for a, b in zip(order, order[1:]):
        out[b] = min(out[b], out[a] - MIN_GAP)
    return out


def main():
    m = mujoco.MjModel.from_xml_path(PLANT)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    rgb, ext = render(m, d)
    top = ext[3] - (np.flatnonzero((rgb < 250).any(axis=2).any(axis=1))[0] / PX_H) * VIEW_H

    joints = []
    for i, name in enumerate(JOINT_NAMES):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        _, y, z = d.xanchor[j]
        joints.append((i + 1, name, y, z))

    fig, ax = plt.subplots(figsize=(9, 9.2), dpi=150)
    ax.imshow(rgb, extent=ext, alpha=0.9, zorder=0)
    for side, sign in (("right", -1), ("left", 1)):
        col = [jt for jt in joints if jt[1] != "neck_yaw" and (jt[2] < 0) == (sign < 0)]
        for (sid, name, y, z), lz in zip(col, spread([jt[3] for jt in col])):
            c = PORT_COLOR[port(name)]
            lx = sign * LABEL_X
            ax.plot([y, lx], [z, lz], color=c, lw=0.8, zorder=1)
            ax.text(lx + sign * 0.006, lz, f"{sid}", color=c, fontsize=11, fontweight="bold",
                    ha="left" if sign > 0 else "right", va="center")
            ax.text(lx + sign * 0.036, lz, name, fontsize=10.5, family="monospace",
                    ha="left" if sign > 0 else "right", va="center")
    sid, name, y, z = next(jt for jt in joints if jt[1] == "neck_yaw")
    c = PORT_COLOR[port(name)]
    ax.plot([y, y], [z, top + 0.025], color=c, lw=0.8, zorder=1)
    ax.text(y, top + 0.03, f"{sid}  ", color=c, fontsize=11, fontweight="bold", ha="right", va="bottom")
    ax.text(y, top + 0.03, name, fontsize=10.5, family="monospace", ha="left", va="bottom")
    for sid, name, y, z in joints:
        ax.plot(y, z, "o", ms=7, mfc=PORT_COLOR[port(name)], mec="white", mew=1.2, zorder=2)

    for p, c in PORT_COLOR.items():
        ax.plot([], [], "o", ms=7, mfc=c, mec="white", label=f"bus port {p} ({'H5' if p == 'A' else 'H6'})")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=2, frameon=False, fontsize=9.5)
    ax.text(-LABEL_X - 0.1, top + 0.03, "robot's right", fontsize=10, color="0.4", va="bottom")
    ax.text(LABEL_X + 0.1, top + 0.03, "robot's left", fontsize=10, color="0.4", va="bottom", ha="right")
    ax.set_title("Servos and joints, front view (servo ID and joint name)", fontsize=12)
    ax.text(0.5, -0.05,
             "IDs: the proposed 17-servo map, docs/servo-map.md §2.1 (#77); the 10-joint prototype "
             "carries IDs 1–10.\nNames: sim/bimo_biped_v6ar.xml. Every servo is an STS3215.",
             transform=ax.transAxes, ha="center", va="top", fontsize=8, color="0.35")
    ax.set_xlim(-LABEL_X - 0.12, LABEL_X + 0.12)
    ax.set_ylim(ext[2] + 0.015, top + 0.07)
    ax.set_aspect("equal")
    ax.axis("off")
    out = os.path.join(build_dir("v6", "renders"), "servo_map.png")
    fig.savefig(out, bbox_inches="tight")
    print(out)


if __name__ == "__main__":
    main()
