#!/usr/bin/env python3
"""Draw the servo map: every joint of the robot, named, with its servo ID and bus port.

Run:  MUJOCO_GL=cgl .venv/bin/python tools/servo_map_figure.py   (or `make figures`)

A front view of the v6 plant (sim/bimo_biped_v6ar.xml) at its zero pose, with
a marker at each joint's anchor. The names, servo IDs and ports come from
tools/bus_cal.py's BUS (which mirrors obs/bus_map.h), the bus joint set of
docs/servo-map.md section 2.1; the STS3250 joints (square markers) from
cad/v6/dimensions_v6.py SERVO_3250_JOINTS. Writes
build/v6/renders/servo_map.png; `make figures` copies it to
docs/img/servo_map.png.
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
sys.path.insert(0, os.path.join(ROOT, "cad", "v6"))
import dimensions_v6 as V
sys.path.insert(0, os.path.join(ROOT, "tools"))
from bus_cal import BUS
from paths import build_dir

PLANT = os.path.join(ROOT, "sim", "bimo_biped_v6ar.xml")
PORT_COLOR = {"A": "#1f6fb4", "B": "#c8541a"}
VIEW_H = 0.64                       # m, vertical extent of the orthographic view
LOOKAT_Z = 0.29
PX_H, PX_W = 1100, 700
LABEL_X = 0.215                     # m, label columns at +-LABEL_X
MIN_GAP = 0.03                      # m, between label rows
HEADLIGHT = (0.18, 0.42)            # ambient, diffuse: dimmer than the plant's, so the white prints keep their shading


def port(name):
    # firmware/components/obs/include/obs/bus_map.h kBusPort: A = the right side and the neck
    return "A" if name.startswith("R_") or name == "neck_yaw" else "B"


def is_3250(name):
    return name.split("_", 1)[1] in V.SERVO_3250_JOINTS


def render(m, d):
    """Front view, orthographic, background white. Returns (rgb, extent in metres)."""
    m.vis.global_.orthographic = 1
    m.vis.global_.fovy = VIEW_H
    m.vis.headlight.ambient[:] = HEADLIGHT[0]
    m.vis.headlight.diffuse[:] = HEADLIGHT[1]
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
    for name, sid, _port in BUS:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        _, y, z = d.xanchor[j]
        joints.append((sid, name, y, z))

    fig, ax = plt.subplots(figsize=(9, 9.2), dpi=150)
    ax.imshow(rgb, extent=ext, zorder=0)
    for side, sign in (("right", -1), ("left", 1)):
        col = [jt for jt in joints if jt[1] != "neck_yaw" and (jt[2] < 0) == (sign < 0)]
        for (sid, name, y, z), lz in zip(col, spread([jt[3] for jt in col])):
            c = PORT_COLOR[port(name)]
            lx = sign * LABEL_X
            ax.plot([y, lx], [z, lz], color=c, lw=0.8, zorder=1)
            ax.text(lx + sign * 0.006, lz, f"{sid}", color=c, fontsize=11, fontweight="bold",
                    ha="left" if sign > 0 else "right", va="center")
            t = ax.text(lx + sign * 0.036, lz, name, fontsize=10.5, family="monospace",
                        ha="left" if sign > 0 else "right", va="center")
            if is_3250(name):
                ax.annotate("STS3250", xy=(1 if sign > 0 else 0, 0.5), xycoords=t,
                            xytext=(sign * 7, 0), textcoords="offset points",
                            ha="left" if sign > 0 else "right", va="center", fontsize=8.5,
                            fontweight="bold", color="white",
                            bbox=dict(boxstyle="round,pad=0.25", fc="0.15", ec="none"))
    sid, name, y, z = next(jt for jt in joints if jt[1] == "neck_yaw")
    c = PORT_COLOR[port(name)]
    ax.plot([y, y], [z, top + 0.025], color=c, lw=0.8, zorder=1)
    ax.text(y, top + 0.03, f"{sid}  ", color=c, fontsize=11, fontweight="bold", ha="right", va="bottom")
    ax.text(y, top + 0.03, name, fontsize=10.5, family="monospace", ha="left", va="bottom")
    for sid, name, y, z in joints:
        if is_3250(name):
            ax.plot(y, z, "s", ms=10, mfc=PORT_COLOR[port(name)], mec="black", mew=1.6, zorder=2)
        else:
            ax.plot(y, z, "o", ms=7, mfc=PORT_COLOR[port(name)], mec="white", mew=1.2, zorder=2)

    for p, c in PORT_COLOR.items():
        ax.plot([], [], "o", ms=7, mfc=c, mec="white", label=f"bus port {p} ({'H5' if p == 'A' else 'H6'})")
    ax.plot([], [], "o", ms=7, mfc="0.6", mec="white", label="STS3215")
    ax.plot([], [], "s", ms=9, mfc="0.6", mec="black", mew=1.6, label="STS3250")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False, fontsize=9.5)
    ax.text(-LABEL_X - 0.1, top + 0.03, "robot's right", fontsize=10, color="0.4", va="bottom")
    ax.text(LABEL_X + 0.1, top + 0.03, "robot's left", fontsize=10, color="0.4", va="bottom", ha="right")
    ax.set_title("Servos and joints, front view: servo ID, joint name, model", fontsize=12)
    ax.text(0.5, -0.05,
             "IDs: the v6 legs as assembled (2026-10-23, written on each servo), docs/servo-map.md §2.1."
             "\nNames: sim/bimo_biped_v6ar.xml. Servos: STS3215, except the STS3250 "
             "hip rolls (DESIGN.md §4).",
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
