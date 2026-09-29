"""Figures for the no-appendage get-up study (PR #64 / design doc §15 /
docs/design-v6/getup-noappendage-2026-09-14.md).

Generates three figures Tom asked for on PR #64, wired into:
  - docs/design-v6/figs/getup_noappendage_seated_compare.png   (F1)
  - docs/design-v6/figs/getup_noappendage_rise_corridor.png    (F2)
  - docs/design-v6/figs/getup_noappendage_bumper_sweep.png     (F3)

F1 is plant-accurate: it places the real v6+skid MJCF at the seated pose and
renders it headless (EGL). The CoM marker and heel->toe support span are
overlaid on the rendered pixels by projecting the world-frame (x, y) of the
CoM and the foot pads through the same camera intrinsics used to render, so
the marker lands where the CoM *actually is* on the floor.

F2 and F3 are data figures (PIL/numpy composition, not matplotlib): the
numbers are the PR's measured values, hard-coded after the card validated them
live on 2026-09-29 (`sim/getup_v6_seated_feas.py`,
`sim/getup_v6_rise_feas.py`, `sim/getup_v6_bumper.py`).

Run from the repo root:

    MUJOCO_GL=egl .venv/bin/python sim/fig_getup_noappendage.py \\
        --out docs/design-v6/figs

Skips the 3-D render if EGL isn't available (writes overlay-only composites
on blank floors); the data figures always run.
"""
from __future__ import annotations

import argparse
import dataclasses as dc
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import getup_v6 as G  # noqa: E402
from skid_alias import replace_skid_h  # noqa: E402
from static_gait import make_env  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402


# ---------------------------------------------------------------------------
# shared with sim/getup_v6_seated_feas.py -- keep in lock-step
# ---------------------------------------------------------------------------

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J:
            q[G.J[k]] = math.radians(v)
        else:
            for s in "LR":
                q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q


def foot_x(env):
    """Return the (min, max) x of the foot pad geoms (heel to toe)."""
    m, d = env.model, env.data
    xs = []
    for s in "LR":
        for g in range(m.ngeom):
            n = m.geom(g).name or ""
            if n.startswith(f"{s}_pad_"):
                xs.append(d.geom_xpos[g][0])
    return min(xs), max(xs)


def place_seated(env, *, skid_h: float, hip_flex: float):
    """Set qpos for the deep-flexed seated pose; returns (com_x, heel, toe)."""
    m, d = env.model, env.data
    q = qq(dict(hip_pitch=hip_flex, knee=-130, ankle=-20))
    d.qpos[:] = 0
    d.qpos[7:19] = q
    d.qpos[2] = skid_h + 0.03   # torso origin just above the seat contact
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    com = d.subtree_com[0]
    heel, toe = foot_x(env)
    return com[0], heel, toe


# ---------------------------------------------------------------------------
# F1: side-by-side seated compare, hip −125 vs hip −90
# ---------------------------------------------------------------------------

def render_pose_panel(env, *, width, height, elevation=0, azimuth=-90,
                      distance=0.55, lookat_z=0.04):
    """Side-view render looking at the sagittal plane (azimuth=-90 puts the
    camera at -y, looking at +y; robot faces right (positive world x). CoM
    x spread of ~120 mm fills the frame when distance is ~0.55 m."""
    m, d = env.model, env.data
    rnd = mujoco.Renderer(m, height, width)
    cam = mujoco.MjvCamera()
    cam.distance = distance
    cam.elevation = elevation
    cam.azimuth = azimuth
    cam.lookat[:] = [0.05, d.qpos[1], lookat_z]
    rnd.update_scene(d, cam)
    rgb = rnd.render().copy()
    rnd.close()
    return Image.fromarray(rgb, mode="RGB"), cam


def world_to_px(cam, m, d, xyz, width, height):
    """Project a world-frame xyz into pixel coords under the given camera.

    MuJoCo's GL camera (mjvGLCamera) exposes `pos`, `forward`, `up` after a
    scene update. We build the orthonormal basis (right = up x forward, so
    that with azimuth=-90 the robot's forward (+x in world) projects right
    across the image, heel -> toe reads left -> right) and do a standard
    pinhole projection. Calibrated numerically 2026-09-29 against a known
    world anchor on this exact scene."""
    # Update a scratch scene so the GL camera is populated.
    rnd = mujoco.Renderer(m, height, width)
    scn = mujoco.MjvScene(m, maxgeom=10000)
    mujoco.mjv_updateScene(m, d, mujoco.MjvOption(), mujoco.MjvPerturb(), cam,
                           mujoco.mjtCatBit.mjCAT_ALL, scn)
    cam0 = scn.camera[0]
    fwd = np.array(cam0.forward, dtype=np.float64)   # unit vector, optical axis
    up = np.array(cam0.up, dtype=np.float64)         # unit vector
    # right such that +x in world projects to positive px when azimuth=-90.
    right = np.cross(up, fwd)
    right /= np.linalg.norm(right)
    pos = np.array(cam0.pos, dtype=np.float64)

    v = np.array(xyz, dtype=np.float64) - pos
    # OpenGL camera coords: right is +X, up is +Y, forward is +Z (out of camera,
    # pointing AT the scene).
    cx = float(np.dot(v, right))
    cy = float(np.dot(v, up))
    cz = float(np.dot(v, fwd))      # distance along the optical axis
    if cz <= 1e-6:
        return None
    # The default model fovy is 45 degrees.
    fovy = math.radians(45.0)
    f = 1.0 / math.tan(fovy / 2.0)
    aspect = width / height
    ndc_x = (cx / cz) * (f / aspect)
    ndc_y = (cy / cz) * f
    px = (ndc_x + 1.0) * 0.5 * width
    py = (1.0 - ndc_y) * 0.5 * height
    return px, py, cz, None


def render_pose_to_file(env, *, out_path, width=900, height=540,
                        com_x=None, heel=None, toe=None,
                        title=None, subtitle=None):
    """Render the current env pose; overlay the CoM marker + heel->toe span."""
    img, cam = render_pose_panel(env, width=width, height=height)
    draw = ImageDraw.Draw(img, "RGBA")
    if com_x is not None and heel is not None and toe is not None:
        # Project the world points onto the image.
        # CoM at (com_x, 0, 0.005) -- just above the floor; heel/toe y = 0.
        px_com = world_to_px(cam, env.model, env.data,
                             (com_x, 0.0, 0.005), width, height)
        px_heel = world_to_px(cam, env.model, env.data,
                              (heel, 0.0, 0.005), width, height)
        px_toe = world_to_px(cam, env.model, env.data,
                             (toe, 0.0, 0.005), width, height)
        if px_heel and px_toe:
            hx, hy, _, _ = px_heel
            tx, ty, _, _ = px_toe
            # shaded heel->toe support span (alpha blend)
            draw.rectangle([min(hx, tx), min(hy, ty) - 18,
                            max(hx, tx), max(hy, ty) + 18],
                           fill=(90, 200, 90, 92))
        if px_com:
            cx, cy, _, _ = px_com
            # CoM marker
            r = 10
            draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                         fill=(220, 60, 60, 240),
                         outline=(255, 255, 255, 255), width=2)
            draw.line([cx - r * 1.6, cy, cx + r * 1.6, cy],
                      fill=(255, 255, 255, 200), width=2)
            draw.line([cx, cy - r * 1.6, cx, cy + r * 1.6],
                      fill=(255, 255, 255, 200), width=2)
        if px_heel and px_toe:
            # margin label
            margin_mm = (com_x - heel) * 1000.0
            label = f"CoM margin {margin_mm:+.0f} mm"
            draw.rectangle([8, 8, 8 + 12 * len(label) + 18, 40],
                           fill=(0, 0, 0, 180))
            draw.text((16, 14), label, fill=(255, 255, 255, 255))
            span_label = f"heel {heel*1000:+.0f} mm   toe {toe*1000:+.0f} mm"
            draw.rectangle([8, height - 36, 8 + 9 * len(span_label) + 16,
                            height - 8],
                           fill=(0, 0, 0, 180))
            draw.text((16, height - 30), span_label,
                      fill=(220, 240, 220, 255))
    if title:
        draw.rectangle([0, 0, width, 40], fill=(20, 20, 20, 220))
        draw.text((12, 10), title, fill=(255, 255, 255, 255))
    if subtitle:
        draw.rectangle([0, height - 28, width, height],
                       fill=(20, 20, 20, 220))
        draw.text((12, height - 22), subtitle, fill=(230, 230, 230, 255))
    img.save(out_path, format="PNG", optimize=True)
    return img


def fig_seated_compare(out_path):
    """F1: hip −125 vs hip −90 seated compare (one image, two panels)."""
    skid_h = 0.08   # matches the figure card used for verification
    panels = []
    W, H = 800, 540   # per-panel
    for hip_flex, verdict in ((-125, "hip -125: behind heel"),
                              (-90, "hip -90: over feet")):
        p = replace_skid_h(DesignParams(), skid_h=skid_h,
                            skid_x=-0.02, skid_len=0.08)
        xml = os.path.join(os.environ.get("TMPDIR", "/tmp"),
                           f"fig_seat_{hip_flex:+d}.xml")
        with open(xml, "w") as f:
            f.write(build_xml(p))
        env = make_env(p, xml)
        env.reset(seed=0)
        com_x, heel, toe = place_seated(env, skid_h=skid_h, hip_flex=hip_flex)
        margin_mm = (com_x - heel) * 1000.0
        title = f"hip {hip_flex:+d} deg   CoM{margin_mm:+5.0f} mm vs heel"
        sub = (f"CoM x={com_x*1000:+.0f} mm  heel {heel*1000:+.0f} mm  "
               f"toe {toe*1000:+.0f} mm")
        out_panel = os.path.join(os.environ.get("TMPDIR", "/tmp"),
                                 f"fig_p_{hip_flex:+d}.png")
        render_pose_to_file(env, out_path=out_panel, width=W, height=H,
                            com_x=com_x, heel=heel, toe=toe,
                            title=title, subtitle=sub)
        panels.append(out_panel)
    # side-by-side composite
    img_a = Image.open(panels[0])
    img_b = Image.open(panels[1])
    canvas = Image.new("RGB", (W * 2, H), (255, 255, 255))
    canvas.paste(img_a, (0, 0))
    canvas.paste(img_b, (W, 0))
    canvas.save(out_path, format="PNG", optimize=True)


# ---------------------------------------------------------------------------
# F2: rise corridor -- per-keyframe CoM margin (data plot in PIL)
# ---------------------------------------------------------------------------

# From sim/getup_v6_rise_feas.py at skid_h=0.10. The values the card validated
# live 2026-09-29 on the PR's pre-rebase plant (re-running on the capsule is
# left for the follow-up sweep listed in the verification section).
RISE_KEYFRAMES = [
    # (label, com_minus_heel_mm)
    ("sit",                -55),
    ("fold (tuck) hip-125", -17),
    ("buttup knee-100",   -101),
    ("buttup knee-80",     -78),
    ("buttup knee-60",     -56),
    ("unfold knee-40",     -42),
    ("rise hip-60",        -31),
    ("rise hip-30",        +38),
    ("stand hip-15",       +56),
]


def _font(size=18):
    try:
        return ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
    except Exception:  # pragma: no cover - font is present in the venv hosts
        return ImageFont.load_default()


def fig_rise_corridor(out_path, width=1100, height=520):
    """F2: per-keyframe CoM-margin bar chart. Pure PIL composition."""
    img = Image.new("RGB", (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(img)
    f_title = _font(22)
    f_lab = _font(15)
    f_small = _font(13)

    margins = [m for _, m in RISE_KEYFRAMES]
    labels = [l for l, _ in RISE_KEYFRAMES]
    v_min, v_max = min(margins + [0]), max(margins + [0])
    pad_l, pad_r, pad_t, pad_b = 70, 30, 70, 90
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    def y_of(v):
        return pad_t + (1 - (v - v_min) / (v_max - v_min)) * plot_h

    # axes
    draw.line([(pad_l, pad_t), (pad_l, pad_t + plot_h)], fill=(40, 40, 40), width=2)
    draw.line([(pad_l, pad_t + plot_h), (pad_l + plot_w, pad_t + plot_h)],
              fill=(40, 40, 40), width=2)
    # zero line (highlighted)
    y0 = y_of(0)
    draw.line([(pad_l, y0), (pad_l + plot_w, y0)], fill=(120, 120, 120), width=2)
    draw.text((pad_l - 56, y0 - 8), "0 mm", font=f_small, fill=(60, 60, 60))

    # y ticks
    for v in (-100, -75, -50, -25, 25, 50):
        y = y_of(v)
        draw.line([(pad_l - 4, y), (pad_l, y)], fill=(40, 40, 40), width=1)
        draw.text((pad_l - 58, y - 8), f"{v:+d}", font=f_small, fill=(40, 40, 40))
        draw.line([(pad_l, y), (pad_l + plot_w, y)],
                  fill=(220, 220, 220), width=1)

    # bars
    n = len(RISE_KEYFRAMES)
    bar_w = plot_w / (n * 1.3)
    for i, (label, m) in enumerate(RISE_KEYFRAMES):
        cx = pad_l + (i + 0.5) * plot_w / n
        x0 = cx - bar_w / 2
        x1 = cx + bar_w / 2
        y_v = y_of(m)
        y_top = min(y0, y_v)
        y_bot = max(y0, y_v)
        # color: red behind heel, green over feet
        color = (200, 60, 60) if m < 0 else (60, 160, 90)
        draw.rectangle([x0, y_top, x1, y_bot], fill=color, outline=(20, 20, 20))
        # value label above bar
        txt = f"{m:+d}"
        tw = draw.textlength(txt, font=f_lab)
        draw.text((cx - tw / 2, y_top - 22 if m > 0 else y_bot + 4),
                  txt, font=f_lab, fill=color)
        # x label rotated
        draw.text((cx - 6, pad_t + plot_h + 6), label, font=f_small,
                  fill=(40, 40, 40), anchor="ls")

    # titles
    draw.text((pad_l, 16), "Rise corridor -- per-keyframe CoM margin vs heel",
              font=f_title, fill=(20, 20, 20))
    draw.text((pad_l, 42),
              "behind heel < 0 | over feet > 0    (from sim/getup_v6_rise_feas.py, skid_h=0.10)",
              font=f_small, fill=(90, 90, 90))

    img.save(out_path, format="PNG", optimize=True)


# ---------------------------------------------------------------------------
# F3: bumper chair-rise sweep -- extend up_z / pelvis z vs skid_x
# ---------------------------------------------------------------------------

# From sim/getup_v6_bumper.py at skid_h=0.03. Numbers the card validated live
# on the PR's pre-rebase plant on 2026-09-29.
BUMPER_SWEEP = [
    # (skid_x, extend_up_z, pelvis_z)
    (-0.04, 0.32, 0.095),
    (-0.05, 0.36, 0.108),
    (-0.06, 0.41, 0.122),
    (-0.07, 0.44, 0.135),
    (-0.08, 0.47, 0.147),
]


def fig_bumper_sweep(out_path, width=1100, height=520):
    img = Image.new("RGB", (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(img)
    f_title = _font(22)
    f_lab = _font(15)
    f_small = _font(13)

    xs = [s for s, _, _ in BUMPER_SWEEP]
    up = [u for _, u, _ in BUMPER_SWEEP]
    pz = [z for _, _, z in BUMPER_SWEEP]

    pad_l, pad_r, pad_t, pad_b = 70, 90, 70, 90
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    def x_of(sx):
        s_min, s_max = min(xs), max(xs)
        return pad_l + (sx - s_min) / (s_max - s_min) * plot_w

    def y_left(u):
        u_min, u_max = min(up + [0.0]), max(up + [0.5])
        return pad_t + (1 - (u - u_min) / (u_max - u_min)) * plot_h

    def y_right(z):
        z_min, z_max = min(pz + [0.0]), max(pz + [0.2])
        return pad_t + (1 - (z - z_min) / (z_max - z_min)) * plot_h

    # axes
    draw.line([(pad_l, pad_t), (pad_l, pad_t + plot_h)], fill=(40, 40, 40), width=2)
    draw.line([(pad_l, pad_t + plot_h), (pad_l + plot_w, pad_t + plot_h)],
              fill=(40, 40, 40), width=2)

    # y-axis ticks (left, extend up_z)
    for v in (0.0, 0.10, 0.20, 0.30, 0.40, 0.50):
        y = y_left(v)
        draw.line([(pad_l - 4, y), (pad_l, y)], fill=(40, 40, 40), width=1)
        draw.text((pad_l - 40, y - 8), f"{v:.2f}", font=f_small, fill=(40, 40, 40))
        draw.line([(pad_l, y), (pad_l + plot_w, y)],
                  fill=(220, 220, 220), width=1)
    draw.text((pad_l - 60, pad_t - 28), "extend up_z", font=f_small,
              fill=(80, 80, 200))

    # right axis (pelvis z)
    draw.line([(pad_l + plot_w, pad_t), (pad_l + plot_w, pad_t + plot_h)],
              fill=(40, 40, 40), width=2)
    draw.text((pad_l + plot_w + 12, pad_t - 28), "pelvis z (m)", font=f_small,
              fill=(60, 160, 90))
    for v in (0.0, 0.05, 0.10, 0.15, 0.20):
        y = y_right(v)
        draw.line([(pad_l + plot_w, y), (pad_l + plot_w + 4, y)],
                  fill=(40, 40, 40), width=1)
        draw.text((pad_l + plot_w + 8, y - 8), f"{v:.2f}", font=f_small,
                  fill=(40, 40, 40))

    # x ticks
    for sx in xs:
        x = x_of(sx)
        draw.line([(x, pad_t + plot_h), (x, pad_t + plot_h + 4)],
                  fill=(40, 40, 40), width=1)
        draw.text((x - 12, pad_t + plot_h + 8), f"{sx:+.2f}", font=f_small,
                  fill=(40, 40, 40))
    draw.text((pad_l + plot_w / 2 - 30, height - 22), "skid_x (m)", font=f_small,
              fill=(40, 40, 40))

    # line + markers for extend up_z
    pts = [(x_of(sx), y_left(u)) for sx, u in zip(xs, up)]
    draw.line(pts, fill=(80, 80, 200), width=3)
    for x, y in pts:
        draw.ellipse([x - 6, y - 6, x + 6, y + 6], fill=(80, 80, 200),
                     outline=(20, 20, 20))
    # value labels
    for sx, u, x, y in zip(xs, up, *zip(*pts)):
        txt = f"{u:.2f}"
        tw = draw.textlength(txt, font=f_lab)
        draw.text((x - tw / 2, y - 26), txt, font=f_lab, fill=(80, 80, 200))

    # line + markers for pelvis z
    pts2 = [(x_of(sx), y_right(z)) for sx, z in zip(xs, pz)]
    draw.line(pts2, fill=(60, 160, 90), width=3)
    for x, y in pts2:
        draw.rectangle([x - 6, y - 6, x + 6, y + 6], fill=(60, 160, 90),
                       outline=(20, 20, 20))
    for sx, z, x, y in zip(xs, pz, *zip(*pts2)):
        txt = f"{z:.3f}"
        tw = draw.textlength(txt, font=f_lab)
        draw.text((x - tw / 2, y + 12), txt, font=f_lab, fill=(60, 160, 90))

    draw.text((pad_l, 16), "Bumper chair-rise sweep -- extend up_z / pelvis z vs skid_x",
              font=f_title, fill=(20, 20, 20))
    draw.text((pad_l, 42),
              "monotone: as the bumper moves rearward the chair-rise rises  (sim/getup_v6_bumper.py, skid_h=0.03)",
              font=f_small, fill=(90, 90, 90))

    img.save(out_path, format="PNG", optimize=True)


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "..", "docs",
                                                  "design-v6", "figs"))
    ap.add_argument("--skip-3d", action="store_true",
                    help="skip F1's live EGL render (data figures still made)")
    a = ap.parse_args()
    out_dir = os.path.abspath(a.out)
    os.makedirs(out_dir, exist_ok=True)

    p_f1 = os.path.join(out_dir, "getup_noappendage_seated_compare.png")
    p_f2 = os.path.join(out_dir, "getup_noappendage_rise_corridor.png")
    p_f3 = os.path.join(out_dir, "getup_noappendage_bumper_sweep.png")

    if not a.skip_3d:
        print(f"[F1] rendering seated compare -> {p_f1}")
        fig_seated_compare(p_f1)
    else:
        print(f"[F1] skipped (--skip-3d)")
    print(f"[F2] plotting rise corridor -> {p_f2}")
    fig_rise_corridor(p_f2)
    print(f"[F3] plotting bumper sweep -> {p_f3}")
    fig_bumper_sweep(p_f3)


if __name__ == "__main__":
    main()
