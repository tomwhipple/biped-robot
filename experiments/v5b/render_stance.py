"""Render the v5b stance strip: stand / shift / one-foot (bench pose) / one-foot at
d = 0 and d = +18.5 mm, from the front (looking along -x at the robot's front)
and from the front-left 3/4.

Poses are kinematic IK keyframes of stance_v5b.py (no dynamics), drawn with the
plant's own visual meshes (cad/stl and cad/v6/stl, geom groups 1 and 2) on a
light checker floor. d is the CoM offset from the stance ankle-roll axis
(+ outboard); the sole centreline is d = +18.5 mm.

    MUJOCO_GL=cgl .venv/bin/python experiments/v5b/render_stance.py
Writes experiments/v5b/figs/v5b_stance_strip.png.
"""
from __future__ import annotations

import os
import sys

os.environ.setdefault("MUJOCO_GL", "cgl")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "sim"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

import plant_v5b as P  # noqa: E402
import stance_v5b as S  # noqa: E402

OUTPNG = os.path.join(HERE, "figs", "v5b_stance_strip.png")
W, H = 380, 560


def light_model():
    spec = P.build_spec()
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.93, 0.95, 0.98], rgb2=[0.78, 0.82, 0.90], width=256, height=256)
    return spec.compile()


def keyframes(m):
    k = S.Kin(m)
    ty_b, rb = S.bench_pose(k)
    tz = k.zst - S.BENCH_DROP
    poses = {"stand": (0, 0, k.zst, 0, 0), "shift (both feet down)": (0, ty_b, tz, 0, 0),
             f"one foot (bench, d={(rb['com'][1]-k.foot0['L'][1])*1e3:+.1f})": (0, ty_b, tz, 0, S.BENCH_LIFT)}
    for dd in (0.0, 0.0185):
        ty, _ = S.solve_d(k, dd)
        poses[f"one foot, d={dd*1e3:+.1f} mm"] = (0, ty, tz, 0, S.BENCH_LIFT)
    out, qprev = {}, None
    for name, p in poses.items():
        q, th, ok = k.pose(*p, q0=qprev)
        qprev = q
        out[name] = (S.cmd_vector(k, q), p, th)
    return out


def main():
    m = light_model()
    kf = keyframes(m)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = opt.geomgroup[1] = opt.geomgroup[2] = 1
    views = {"front": dict(azimuth=180, elevation=-6, distance=0.72),
             "3/4": dict(azimuth=148, elevation=-14, distance=0.72)}
    rows = []
    for view, c in views.items():
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = c["azimuth"], c["elevation"], c["distance"]
        tiles = []
        for name, (q, p, th) in kf.items():
            d.qpos[:] = 0
            for i, n in enumerate(P.JOINTS):
                d.qpos[m.jnt_qposadr[m.joint(n).id]] = q[i]
            d.qpos[0:3] = [p[0], p[1], p[2]]
            d.qpos[3:7] = [np.cos(th / 2), np.sin(th / 2), 0, 0]
            mujoco.mj_forward(m, d)
            cam.lookat[:] = [0.0, 0.5 * p[1], 0.21]
            r.update_scene(d, cam, opt)
            img = Image.fromarray(r.render().copy())
            ImageDraw.Draw(img).text((8, 6), f"{name}  [{view}]", fill=(20, 20, 20))
            tiles.append(np.asarray(img))
        rows.append(np.hstack(tiles))
    os.makedirs(os.path.dirname(OUTPNG), exist_ok=True)
    Image.fromarray(np.vstack(rows)).save(OUTPNG)
    print("wrote", OUTPNG)


if __name__ == "__main__":
    main()
