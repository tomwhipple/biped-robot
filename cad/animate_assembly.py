"""Fly-in assembly animation: every part travels its real insertion path.

Run:  .venv/bin/python cad/animate_assembly.py
      -> cad/renders/assembly_flyin.gif

Purpose is assembly *feasibility*, not eye candy: parts approach their seats
along the direction they must actually be inserted (roll servos slide UP into
the pelvis bays, links slide onto cases from the front, the camera drops onto
the prongs), so a blocked insertion path shows up as one part passing through
another. Complements check_assembly.py, which only tests the assembled state.

Parts are enumerated from export_assembly.robot, so this survives part-set
changes as long as labels keep their prefixes.
"""
import os
import shutil
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import export_stl

import export_assembly as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
TMP = os.path.join(OUT, "_anim")
os.makedirs(TMP, exist_ok=True)

# label prefix -> (assembly stage, insertion vector in mm, world frame)
PLAN = [
    ("pelvis",          (0, 0, 120)),   # chassis lowered in from above
    ("servo_hip_roll",  (0, 0, -80)),   # slides UP into the pelvis bay
    ("yoke_roll",       (0, 0, -60)),   # clevis up over the roll servo
    ("yoke_pitch",      (0, 0, -60)),   # bolts to the roll flange from below
    ("servo_hip_pitch", (0, 0, -60)),   # thigh servo up into the pitch clevis
    ("link_thigh",      (80, 0, 0)),    # grip channel slides on from the front
    ("servo_knee",      (0, 0, -60)),   # up into the thigh fork
    ("link_shin",       (80, 0, 0)),    # grip channel from the front
    ("servo_ankle",     (0, 0, -60)),   # up into the shin fork
    ("foot",            (0, 0, -60)),   # sole rises to pocket the ankle servo
    ("tower",           (0, 0, 120)),   # drops onto the deck bosses
    ("gopro_base",      (0, 0, 90)),    # drops onto the tower-top bosses
    ("camera",          (0, 0, 90)),    # fingers drop into the prongs
]
COLOR = {"servo": (0.22, 0.23, 0.27, 1), "camera": (0.10, 0.10, 0.12, 1),
         "foot": (0.70, 0.72, 0.78, 1), "": (0.80, 0.82, 0.86, 1)}


def stage_of(label):
    for i, (prefix, vec) in enumerate(PLAN):
        if label.startswith(prefix):
            return i, np.array(vec) / 1000.0
    raise KeyError(f"no insertion plan for part '{label}' — extend PLAN")


def color_of(label):
    for prefix in ("servo", "camera", "foot"):
        if label.startswith(prefix):
            return COLOR[prefix]
    return COLOR[""]


parts = []                                # (name, stage, vec, stl, rgba)
def collect(c, tag=""):
    for ch in c.children:
        if list(ch.children):
            collect(ch, ch.label.replace("leg_", "") + "_")
        else:
            name = tag + ch.label
            stage, vec = stage_of(ch.label)
            stl = os.path.join(TMP, f"{name}.stl")
            export_stl(ch, stl)
            parts.append((name, stage, vec, stl, color_of(ch.label)))
collect(A.robot)
print(f"{len(parts)} parts across {len(PLAN)} assembly stages")

assets = "\n".join(
    f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
    for n, _, _, s, _ in parts)
bodies = "\n".join(
    f'<body name="{n}" mocap="true">'
    f'<geom type="mesh" mesh="{n}" contype="0" conaffinity="0" '
    f'rgba="{c[0]} {c[1]} {c[2]} {c[3]}"/></body>'
    for n, _, _, _, c in parts)

XML = f"""
<mujoco>
  <visual><headlight ambient="0.45 0.45 0.47" diffuse="0.5 0.5 0.5"/>
    <global offwidth="560" offheight="460"/>
    <quality shadowsize="2048"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.93"
             rgb2="0.84 0.86 0.89" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="12 12"/>
    {assets}
  </asset>
  <worldbody>
    <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8" castshadow="true"/>
    <geom type="plane" size="2 2 0.1" material="floor"/>
    {bodies}
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
mocap = {n: m.body(n).mocapid[0] for n, *_ in parts}

FPS, INTRO, STAGGER, TRAVEL, HOLD = 16, 8, 9, 24, 20
total = INTRO + STAGGER * (len(PLAN) - 1) + TRAVEL + HOLD
r = mujoco.Renderer(m, height=460, width=560)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0.01, 0.0, 0.185]
cam.distance, cam.elevation = 0.80, -10

frames = []
for f in range(total):
    for n, stage, vec, _, _ in parts:
        t = (f - INTRO - stage * STAGGER) / TRAVEL
        t = np.clip(t, 0.0, 1.0)
        ease = t * t * (3 - 2 * t)                   # smoothstep
        d.mocap_pos[mocap[n]] = (1 - ease) * vec
    mujoco.mj_forward(m, d)
    cam.azimuth = 140 + 30 * f / total               # slow orbit for depth cues
    r.update_scene(d, cam)
    frames.append(r.render().copy())

path = os.path.join(OUT, "assembly_flyin.gif")
imageio.mimwrite(path, frames, fps=FPS, loop=0)
shutil.rmtree(TMP)
print(f"wrote {path}  ({total} frames @ {FPS} fps, "
      f"{os.path.getsize(path) // 1024} KB)")
