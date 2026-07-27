"""Export the DRESSED robot (cables, zip ties, battery, driver board,
pigtail — see dress.py) as one labeled STEP, plus PNG stills.

Run:  .venv/bin/python cad/export_assembly_full.py
      -> cad/step/assembly_full.step
      -> cad/renders/assembly_full.png          (front-iso + back, neutral)
      -> cad/renders/assembly_full_pike.png     (get-up pike, dress follows)

The neutral STEP is the "approximate complete" model: printed parts and
servo mocks exactly as assembly.step, plus plausible-spline wiring and
mock electronics. The pike still proves the cable dress follows the
deepest working pose (hip -110, knee -95, ankle +40).
"""
import os
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import Compound, export_step, export_stl

import dress

HERE = os.path.dirname(os.path.abspath(__file__))
STEP_OUT = os.path.join(HERE, "step")
REN_OUT = os.path.join(HERE, "renders")
TMP = os.path.join(REN_OUT, "_full")


def leaves(c):
    for ch in c.children:
        if list(ch.children):
            yield from leaves(ch)
        else:
            yield ch


def render(robot, path, views, px=640):
    """One PNG strip: the compound rendered from (azimuth, elevation) views,
    colored per-piece via one temp STL per leaf."""
    os.makedirs(TMP, exist_ok=True)
    items = []
    for i, ch in enumerate(leaves(robot)):
        stl = os.path.join(TMP, f"p{i}.stl")
        export_stl(ch, stl)
        c = tuple(ch.color)                    # (r, g, b, a)
        items.append((f"p{i}", stl, c[:3]))
    assets = "\n".join(f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
                       for n, s, _ in items)
    geoms = "\n".join(f'<geom type="mesh" mesh="{n}" contype="0" '
                      f'conaffinity="0" rgba="{c[0]} {c[1]} {c[2]} 1"/>'
                      for n, _, c in items)
    xml = f"""<mujoco>
      <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.45 0.45 0.45"/>
        <global offwidth="{px}" offheight="{px}"/>
        <quality shadowsize="2048"/></visual>
      <asset>
        <texture name="sky" type="skybox" builtin="gradient"
                 rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88"
                 width="256" height="256"/>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.93"
                 rgb2="0.84 0.86 0.89" width="512" height="512"/>
        <material name="floor" texture="grid" texrepeat="12 12"/>
        {assets}
      </asset>
      <worldbody>
        <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8" castshadow="true"/>
        <geom type="plane" size="2 2 0.1" material="floor"/>
        {geoms}
      </worldbody>
    </mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, height=px, width=px)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, 0.19]
    cam.distance = 0.85
    frames = []
    for az, el in views:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        frames.append(r.render().copy())
    imageio.imwrite(path, np.concatenate(frames, axis=1))
    r.close()
    for _, s, _ in items:
        os.remove(s)
    print(f"wrote {path}")


def main():

    os.makedirs(STEP_OUT, exist_ok=True)
    robot = dress.dressed_robot()
    path = os.path.join(STEP_OUT, "assembly_full.step")
    export_step(robot, path)
    bb = robot.bounding_box()
    print(f"assembly_full -> {path}")
    print(f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}.."
          f"{bb.max.Y:.1f}  z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
    render(robot, os.path.join(REN_OUT, "assembly_full.png"),
           [(160, -12), (250, -15), (20, -8)])
    pike = dress.dressed_robot(hip=-110, knee=-95, ankle=40)
    render(pike, os.path.join(REN_OUT, "assembly_full_pike.png"),
           [(160, -12), (250, -18)])

if __name__ == "__main__":
    main()
