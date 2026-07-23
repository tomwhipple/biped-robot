"""Before/after render of the hip stack for the v3yaw change.

Run:  .venv/bin/python cad/render_hip_yaw.py
  -> renders/hip_yaw_before.png  (8-DOF: pelvis bay -> roll yoke)
     renders/hip_yaw_after.png   (v3yaw: pelvis seat -> yaw servo -> carrier ->
                                  roll servo -> roll yoke)
     renders/hip_yaw_beforeafter.png  (the two side by side)

Printed parts are light, servo mocks dark, so the inserted yaw stack reads at a
glance. Uses the CURRENT parts for "after" and the git-HEAD pelvis STL for
"before".
"""
import os
import mujoco
import imageio.v2 as imageio
import numpy as np
from build123d import Pos, Rot, Compound, export_stl

import dimensions as D
import parts
import check_assembly as CA

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
BEFORE_PELVIS = "/tmp/pelvis_before.stl"     # git show HEAD:cad/stl/pelvis.stl


def yoke_at(z, by):
    return Pos(0, by, z) * parts.yoke_roll()


def scene_after():
    printed, servos = [], []
    printed.append(parts.pelvis())
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        ymid = D.YAW_IDLER_FACE_Z - D.SV_TOPFACE
        servos.append(Pos(0, by, ymid) * CA.servo_mock_z())        # yaw servo
        printed.append(Pos(0, by, D.YAW_HORN_FACE_Z) * parts.yaw_carrier())
        rz = D.YAW_HORN_FACE_Z + D.CARRIER_ROLL_AXIS               # roll axis
        servos.append(Pos(0, by, rz) * CA.servo_mock_x())          # roll servo
        printed.append(yoke_at(rz, by))
    return printed, servos


def scene_before(pelvis_mesh):
    # rebuild the 8-DOF hip: roll servo in the pelvis bay + roll yoke
    printed, servos = [], []
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        rz = -D.DECK_T - D.SV_AXIS_FROM_REAR                       # -40.11
        servos.append(Pos(0, by, rz) * CA.servo_mock_x())
        printed.append(yoke_at(rz, by))
    return printed, servos, pelvis_mesh


def render(printed_stl, servo_stl, extra_stl, out, zc):
    xml = f"""
    <mujoco>
      <visual><headlight ambient="0.45 0.45 0.47" diffuse="0.6 0.6 0.6"/>
        <global offwidth="640" offheight="720"/><quality shadowsize="4096"/></visual>
      <asset>
        <texture name="sky" type="skybox" builtin="gradient"
                 rgb1="0.94 0.95 0.97" rgb2="0.82 0.85 0.90" width="256" height="256"/>
        <mesh name="printed" file="{printed_stl}" scale="0.001 0.001 0.001"/>
        <mesh name="servos" file="{servo_stl}" scale="0.001 0.001 0.001"/>
        {'<mesh name="extra" file="'+extra_stl+'" scale="0.001 0.001 0.001"/>' if extra_stl else ''}
      </asset>
      <worldbody>
        <light pos="0.3 -0.5 0.6" dir="-0.3 0.5 -0.8" castshadow="true"/>
        <geom type="mesh" mesh="printed" contype="0" conaffinity="0" rgba="0.80 0.82 0.86 1"/>
        <geom type="mesh" mesh="servos" contype="0" conaffinity="0" rgba="0.22 0.23 0.27 1"/>
        {'<geom type="mesh" mesh="extra" contype="0" conaffinity="0" rgba="0.80 0.82 0.86 1"/>' if extra_stl else ''}
      </worldbody>
    </mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, height=720, width=640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, zc]
    cam.distance, cam.azimuth, cam.elevation = 0.26, 125, -12
    r.update_scene(d, cam)
    imageio.imwrite(out, r.render())
    return imageio.imread(out)


def main():
    os.makedirs(OUT, exist_ok=True)
    # AFTER
    pr, sv = scene_after()
    ap, as_ = os.path.join(OUT, "_a_pr.stl"), os.path.join(OUT, "_a_sv.stl")
    export_stl(Compound(children=pr), ap)
    export_stl(Compound(children=sv), as_)
    zc_after = (D.YAW_IDLER_FACE_Z + D.YAW_HORN_FACE_Z + D.CARRIER_ROLL_AXIS) / 2 / 1000
    img_a = render(ap, as_, None, os.path.join(OUT, "hip_yaw_after.png"), zc_after)
    # BEFORE
    prb, svb, pelvis_mesh = scene_before(BEFORE_PELVIS)
    bp, bs = os.path.join(OUT, "_b_pr.stl"), os.path.join(OUT, "_b_sv.stl")
    export_stl(Compound(children=prb), bp)
    export_stl(Compound(children=svb), bs)
    zc_before = (-D.DECK_T - D.SV_AXIS_FROM_REAR) / 2 / 1000
    img_b = render(bp, bs, pelvis_mesh, os.path.join(OUT, "hip_yaw_before.png"),
                   zc_before)
    # side by side (pad shorter to match height already equal at 720)
    combo = np.concatenate([img_b, img_a], axis=1)
    imageio.imwrite(os.path.join(OUT, "hip_yaw_beforeafter.png"), combo)
    for f in (ap, as_, bp, bs):
        os.remove(f)
    print("wrote hip_yaw_before.png / hip_yaw_after.png / hip_yaw_beforeafter.png")


if __name__ == "__main__":
    main()
