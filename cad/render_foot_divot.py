"""Foot ankle-retention driver-access view: semi-transparent foot + the M3
driver shaft reaching each LOW retention screw through its new divot.

Run:  .venv/bin/python cad/render_foot_divot.py -> renders/foot_divot_driver.png
"""
import os
import mujoco
import imageio.v2 as imageio
import numpy as np
from build123d import Pos, Rot, Cylinder, Compound, export_stl, Plane

import dimensions as D
import parts

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")


def shaft(x, y0, y1, z, r):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def main():
    zp = D.FOOT_T - D.FOOT_POCKET_D
    py = D.SV_TOPFACE + D.FIT
    zlow = zp + 2.11
    ft = parts.foot()
    drivers = None
    # M3 hex driver (~3 mm shaft) + head socket, entering along Y through each
    # divot to the LOW screw head on the tab outer face.
    for xh, ysgn in ((-29.0, 1), (-32.75, -1)):
        yface = ysgn * (py + D.FOOT_WALL_T)
        d = shaft(xh, yface, ysgn * (D.FOOT_W / 2 + 14), zlow, 1.6)   # shaft
        d += Pos(xh, yface, zlow) * Rot(90, 0, 0) * Cylinder(3.6, 4)  # socket/head
        drivers = d if drivers is None else drivers + d
    fp = os.path.join(OUT, "_foot.stl")
    dp = os.path.join(OUT, "_drv.stl")
    export_stl(ft, fp)
    export_stl(drivers, dp)
    xml = f"""
    <mujoco>
      <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.55 0.55 0.55"/>
        <global offwidth="900" offheight="640"/><quality shadowsize="4096"/></visual>
      <asset>
        <texture name="sky" type="skybox" builtin="gradient" rgb1="0.95 0.96 0.98"
                 rgb2="0.84 0.87 0.92" width="256" height="256"/>
        <mesh name="foot" file="{fp}" scale="0.001 0.001 0.001"/>
        <mesh name="drv" file="{dp}" scale="0.001 0.001 0.001"/>
      </asset>
      <worldbody>
        <light pos="0.1 -0.4 0.5" dir="-0.1 0.4 -0.8" castshadow="false"/>
        <geom type="mesh" mesh="foot" contype="0" conaffinity="0" rgba="0.70 0.75 0.82 0.38"/>
        <geom type="mesh" mesh="drv" contype="0" conaffinity="0" rgba="0.85 0.55 0.10 1"/>
      </worldbody>
    </mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, height=640, width=900)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [-0.030, 0.0, 0.012]
    cam.distance, cam.azimuth, cam.elevation = 0.145, 65, -22
    r.update_scene(d, cam)
    out = os.path.join(OUT, "foot_divot_driver.png")
    imageio.imwrite(out, r.render())
    for f in (fp, dp):
        os.remove(f)
    print("wrote", out)


if __name__ == "__main__":
    main()
