"""Render one part STL to a PNG strip of named views (MuJoCo offscreen).

Run:  .venv/bin/python cad/render_part.py foot            # -> renders/foot.png
      .venv/bin/python cad/render_part.py leg_link_print --out leg_link_fins
Views: iso / bottom / back by default (bottom shows sole-side sag fixes,
back shows heel/tab details); pick others with --views.
"""
import argparse
import os

import imageio.v2 as imageio
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

VIEWS = {                # azimuth, elevation
    "iso": (135, -30),
    "front": (0, -10),
    "back": (180, -15),
    "side": (90, -10),
    "top": (90, -89),
    "bottom": (90, 89),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl", help="name under cad/stl/ (without .stl) or a path")
    ap.add_argument("--views", default="iso,bottom,back")
    ap.add_argument("--out", default=None)
    ap.add_argument("--px", type=int, default=640)
    args = ap.parse_args()

    path = args.stl if args.stl.endswith(".stl") else \
        os.path.join(HERE, "stl", args.stl + ".stl")
    name = args.out or os.path.splitext(os.path.basename(path))[0]

    xml = f"""
    <mujoco>
      <asset>
        <mesh name="part" file="{path}" scale="0.001 0.001 0.001"/>
      </asset>
      <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/>
        <global offwidth="{args.px}" offheight="{args.px}"/>
        <quality shadowsize="4096"/></visual>
      <worldbody>
        <geom type="mesh" mesh="part" rgba="0.45 0.62 0.90 1"/>
      </worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # frame the mesh: center + distance from its bounding sphere
    m = model.mesh(0)
    v = model.mesh_vert[m.vertadr[0]:m.vertadr[0] + m.vertnum[0]]
    center, radius = v.mean(0), np.linalg.norm(v - v.mean(0), axis=1).max()

    ren = mujoco.Renderer(model, args.px, args.px)
    frames = []
    for vn in args.views.split(","):
        az, el = VIEWS[vn]
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = center, 2.6 * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    out = os.path.join(HERE, "renders", name + ".png")
    imageio.imwrite(out, np.concatenate(frames, axis=1))
    print(f"saved -> {out}  ({args.views})")


if __name__ == "__main__":
    main()
