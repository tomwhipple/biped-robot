"""Step-by-step assembly figures for docs/assembly.md.

Run:  .venv/bin/python cad/render_assembly_steps.py
      -> docs/assembly/step_*.png

One still per assembly step, in the bench order of docs/assembly.md: parts
already installed render in their natural colors, the part(s) being installed
render ORANGE, offset along their real insertion direction (same feasibility
idea as animate_assembly.py — the approach shown is one that actually clears).
Parts from later steps are hidden. Both legs are shown in the leg steps
because you build two of everything.

Parts are enumerated from export_assembly.robot, so this survives part-set
changes as long as labels keep their prefixes (extend FIGS if parts appear).
"""
import os
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import export_stl

import export_assembly as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "assembly")
TMP = os.path.join(HERE, "renders", "_steps")
os.makedirs(OUT, exist_ok=True)
os.makedirs(TMP, exist_ok=True)

ORANGE = (0.93, 0.46, 0.10, 1)
COLOR = {"servo": (0.22, 0.23, 0.27, 1), "camera": (0.10, 0.10, 0.12, 1),
         "battery": (0.16, 0.30, 0.55, 1),
         "foot": (0.70, 0.72, 0.78, 1), "": (0.80, 0.82, 0.86, 1)}
HIDE = np.array([0.0, 0.0, 8.0])          # parked far above every camera frame

# whole leg subassembly, proximal to distal (base labels, both legs match)
LEG = ["yoke_roll", "yoke_pitch", "servo_hip_pitch", "link_thigh",
       "servo_knee", "link_shin", "servo_ankle", "foot"]
ALL = ["pelvis", "tower", "battery_3s_mock", "imu_carrier", "imu_bno055_mock",
       "gopro_base", "camera_gopro_max_mock"] + LEG

# exploded-view offsets (mm) = the fly-in insertion vectors, exaggerated
EXPLODE = {"pelvis": (0, 0, 110), "servo_hip_roll": (0, 0, -75),
           "yoke_roll": (0, 0, -55), "yoke_pitch": (0, 0, -110),
           "servo_hip_pitch": (0, 0, -80), "link_thigh": (85, 0, -25),
           "servo_knee": (0, 0, -55), "link_shin": (85, 0, 20),
           "servo_ankle": (0, 0, -30), "foot": (0, 0, -55),
           "tower": (0, 0, 165), "battery_3s_mock": (-95, 0, 165),
           "imu_carrier": (0, 0, 185), "imu_bno055_mock": (0, 0, 210),
           "gopro_base": (0, 0, 235), "camera_gopro_max_mock": (0, 0, 280)}

# (file, placed base-labels, {incoming base-label: offset mm},
#  (lookat y, lookat z, distance, azimuth, elevation), name-prefix filter)
# The single-leg close-ups (steps 2-5) render the LEFT leg only -- both legs
# overlap too much in perspective; the doc says "x2" instead.
L = ("L_",)
FIGS = [
    ("step00_exploded", [], EXPLODE, (0.0, 0.30, 0.95, 140, -8), None),
    ("step01_foot_servo", ["foot"], {"servo_ankle": (0, 0, 55)},
     (0.0, 0.035, 0.30, 125, -18), None),
    ("step02_link_on_case", ["servo_knee"], {"link_shin": (50, 0, 0)},
     (0.028, 0.085, 0.28, 140, -8), L),
    ("step03_fork_to_horn", ["foot", "servo_ankle"],
     {"link_shin": (0, 0, 60), "servo_knee": (0, 0, 60)},
     (0.028, 0.065, 0.34, 125, -12), L),
    ("step04_yoke_pitch", ["servo_hip_pitch", "link_thigh"],
     {"yoke_pitch": (0, 0, 50)}, (0.028, 0.215, 0.28, 130, -8), L),
    ("step05_yoke_roll", ["yoke_pitch", "servo_hip_pitch", "link_thigh"],
     {"yoke_roll": (0, 0, 50)}, (0.028, 0.235, 0.28, 130, -8), L),
    ("step06_roll_servos", ["pelvis"], {"servo_hip_roll": (0, 0, -70)},
     (0.0, 0.245, 0.38, 140, -22), None),
    ("step07_legs_to_pelvis", ["pelvis", "servo_hip_roll"],
     {b: (0, 0, -55) for b in LEG}, (0.0, 0.16, 0.62, 140, -10), None),
    ("step08_tower", ["pelvis", "servo_hip_roll"] + LEG,
     {"tower": (0, 0, 85)}, (0.0, 0.28, 0.50, 140, -8), None),
    ("step09_battery", ["pelvis", "servo_hip_roll", "tower"] + LEG,
     {"battery_3s_mock": (-85, 0, 0)}, (0.0, 0.30, 0.44, 320, -8), None),
    ("step10_gopro_base", ["pelvis", "servo_hip_roll", "tower",
                           "battery_3s_mock"] + LEG,
     {"imu_carrier": (0, 0, 40), "imu_bno055_mock": (0, 0, 55),
      "gopro_base": (0, 0, 75)}, (0.0, 0.325, 0.34, 140, -6), None),
    ("step11_camera", ["pelvis", "servo_hip_roll", "tower", "battery_3s_mock",
                       "imu_carrier", "imu_bno055_mock", "gopro_base"] + LEG,
     {"camera_gopro_max_mock": (0, 0, 75)}, (0.0, 0.345, 0.58, 140, -6), None),
    ("step12_complete", ALL + ["servo_hip_roll"], {},
     (0.0, 0.20, 0.78, 155, -8), None),
]


def color_of(label):
    for prefix in ("servo", "camera", "battery", "foot"):
        if label.startswith(prefix):
            return COLOR[prefix]
    return COLOR[""]


parts = []                                # (name, base label, rgba)
def collect(c, tag=""):
    for ch in c.children:
        if list(ch.children):
            collect(ch, ch.label.replace("leg_", "") + "_")
        else:
            name = tag + ch.label
            stl = os.path.join(TMP, f"{name}.stl")
            export_stl(ch, stl)
            parts.append((name, ch.label, stl, color_of(ch.label)))
collect(A.robot)
print(f"{len(parts)} parts, {len(FIGS)} figures")

assets = "\n".join(
    f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
    for n, _, s, _ in parts)
bodies = "\n".join(
    f'<body name="{n}" mocap="true">'
    f'<geom name="{n}" type="mesh" mesh="{n}" contype="0" conaffinity="0"/>'
    f'</body>' for n, *_ in parts)

XML = f"""
<mujoco>
  <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.45 0.45 0.45"/>
    <global offwidth="880" offheight="660"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
    {assets}
  </asset>
  <worldbody>
    <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8"/>
    {bodies}
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
r = mujoco.Renderer(m, height=660, width=880)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)

for fname, placed, incoming, (ly, lz, dist, azim, elev), only in FIGS:
    for n, base, _, rgba in parts:
        gid = m.geom(n).id
        mid = m.body(n).mocapid[0]
        shown = only is None or n.startswith(only) or n == base
        if base in incoming and shown:
            d.mocap_pos[mid] = np.array(incoming[base]) / 1000.0
            m.geom_rgba[gid] = ORANGE if fname != "step00_exploded" else rgba
        elif base in placed and shown:
            d.mocap_pos[mid] = (0, 0, 0)
            m.geom_rgba[gid] = rgba
        else:
            d.mocap_pos[mid] = HIDE
    mujoco.mj_forward(m, d)
    cam.lookat[:] = [0.01, ly, lz]
    cam.distance, cam.azimuth, cam.elevation = dist, azim, elev
    r.update_scene(d, cam)
    path = os.path.join(OUT, f"{fname}.png")
    imageio.imwrite(path, r.render())
    print("wrote", path)

for _, _, stl, _ in parts:
    os.remove(stl)
os.rmdir(TMP)
