"""Render the assembled robot to PNG via MuJoCo (visual meshes, no physics).

Run:  .venv/bin/python cad/render_assembly.py
      -> cad/renders/assembly_mujoco.png  (used by sim/build_report.py)

Exports the printed parts and the servo mocks as two temporary STLs so they can
be colored separately, then renders one frame with a MuJoCo offscreen camera.
"""
import os
import mujoco
import imageio.v2 as imageio
from build123d import Compound, export_stl

import export_assembly as A          # builds `robot` (labeled compound tree)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
os.makedirs(OUT, exist_ok=True)

printed, servos = [], []


def collect(c):
    for ch in c.children:
        if list(ch.children):
            collect(ch)
        elif ch.label.startswith("servo"):
            servos.append(ch)
        else:
            printed.append(ch)


collect(A.robot)
mesh_printed = os.path.join(OUT, "_printed.stl")
mesh_servos = os.path.join(OUT, "_servos.stl")
export_stl(Compound(children=printed), mesh_printed)
export_stl(Compound(children=servos), mesh_servos)

XML = f"""
<mujoco>
  <visual><headlight ambient="0.45 0.45 0.47" diffuse="0.5 0.5 0.5"/>
    <global offwidth="880" offheight="720"/>
    <quality shadowsize="4096"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
    <mesh name="printed" file="{mesh_printed}" scale="0.001 0.001 0.001"/>
    <mesh name="servos" file="{mesh_servos}" scale="0.001 0.001 0.001"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.93"
             rgb2="0.84 0.86 0.89" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="12 12"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8" castshadow="true"/>
    <geom type="plane" size="2 2 0.1" material="floor"/>
    <geom type="mesh" mesh="printed" contype="0" conaffinity="0"
          rgba="0.80 0.82 0.86 1"/>
    <geom type="mesh" mesh="servos" contype="0" conaffinity="0"
          rgba="0.22 0.23 0.27 1"/>
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, height=720, width=880)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0.01, 0.0, 0.165]
cam.distance, cam.azimuth, cam.elevation = 0.62, 155, -8
r.update_scene(d, cam)
path = os.path.join(OUT, "assembly_mujoco.png")
imageio.imwrite(path, r.render())
for tmp in (mesh_printed, mesh_servos):
    os.remove(tmp)
print("wrote", path)
