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
    ap.add_argument("--zoom", type=float, default=2.6,
                    help="camera distance in bounding-sphere radii; drop it "
                         "to ~1 for a close-up (e.g. a window's supports)")
    ap.add_argument("--lookat", default=None, metavar="X,Y,Z",
                    help="aim point in the PART's own mm coordinates "
                         "(parts.py frame), not the view-name default")
    ap.add_argument("--view-dir", default=None, metavar="X,Y,Z",
                    help="direction the camera looks ALONG, in part mm coords "
                         "(e.g. 1,0,0 to face the -X wall). Renders one frame "
                         "and ignores --views.")
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

    # frame the mesh: center + distance from its bounding sphere. NOTE the
    # compiler re-frames each mesh asset onto its own centre of mass and
    # principal axes (mesh_pos / mesh_quat) -- mesh_vert comes back in THAT
    # frame -- but it also folds the inverse into geom_pos/geom_quat, so the
    # part still renders in its parts.py coordinates. Map the centroid back,
    # or the camera aims at a point up to a part-radius off (it framed
    # yaw_carrier 18 mm high, clipping the bay off the bottom of the strip).
    m = model.mesh(0)
    v = model.mesh_vert[m.vertadr[0]:m.vertadr[0] + m.vertnum[0]]
    radius = np.linalg.norm(v - v.mean(0), axis=1).max()
    center = np.zeros(3)
    mujoco.mju_rotVecQuat(center, v.mean(0), model.mesh_quat[0])
    center += model.mesh_pos[0]

    if args.lookat:                        # part mm -> world is just a scale
        center = np.array([float(t) for t in args.lookat.split(",")]) / 1000.0
    views = args.views.split(",")
    if args.view_dir:
        f = np.array([float(t) for t in args.view_dir.split(",")], float)
        f /= np.linalg.norm(f)
        # measured: forward = (cos el cos az, cos el sin az, sin el), camera at
        # lookat - distance * forward
        views = [(np.degrees(np.arctan2(f[1], f[0])),
                  np.degrees(np.arcsin(np.clip(f[2], -1, 1))))]

    ren = mujoco.Renderer(model, args.px, args.px)
    frames = []
    for vn in views:
        az, el = VIEWS[vn] if isinstance(vn, str) else vn
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = center, args.zoom * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    out = os.path.join(HERE, "renders", name + ".png")
    imageio.imwrite(out, np.concatenate(frames, axis=1))
    print(f"saved -> {out}  ({args.views})")


if __name__ == "__main__":
    main()
