"""Render a range-of-motion sweep video of the legs.

Kinematic only: the base is held fixed in the air and each joint pair is
driven through its full modeled servo range (the same limits check_assembly
sweeps against), one DOF at a time, then a combined envelope move. Frames
come from the v2 model so the video shows the real CAD meshes.

Usage: python render_rom.py [--out renders/rom_sweep]
Writes <out>.mov (H.264, QuickTime-friendly).
"""
import argparse
import os
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FPS = 50
BASE_Z = 0.36            # hold the torso here: full leg extension still clears the floor
W, H = 900, 1100

# (label, [(joint, scale)]) -- scale lets hip_roll mirror so both legs abduct.
PHASES = [
    # same sign: legs lean together. Mirrored (+/-) full adduction would drive
    # the legs through each other -- not a pose the hardware can reach.
    ("hip roll",  [("L_hip_roll", 1.0), ("R_hip_roll", 1.0)]),
    ("hip pitch", [("L_hip_pitch", 1.0), ("R_hip_pitch", 1.0)]),
    ("knee",      [("L_knee", 1.0), ("R_knee", 1.0)]),
    ("ankle",     [("L_ankle", 1.0), ("R_ankle", 1.0)]),
]


def ease(t):
    """0..1 -> 0..1, cosine ease in/out."""
    return 0.5 - 0.5 * np.cos(np.pi * t)


def sweep_profile(lo, hi, n):
    """Angle track 0 -> lo -> hi -> 0 over n frames, eased."""
    n1, n2, n3 = int(n * 0.25), int(n * 0.5), n - int(n * 0.25) - int(n * 0.5)
    a = ease(np.linspace(0, 1, n1)) * lo
    b = lo + ease(np.linspace(0, 1, n2)) * (hi - lo)
    c = hi + ease(np.linspace(0, 1, n3)) * (0 - hi)
    return np.concatenate([a, b, c])


def load_font(size):
    for cand in ("/System/Library/Fonts/Helvetica.ttc",
                 "/System/Library/Fonts/Supplemental/Arial.ttf"):
        if os.path.exists(cand):
            return ImageFont.truetype(cand, size)
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "renders", "rom_sweep"))
    ap.add_argument("--seconds-per-joint", type=float, default=3.2)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "bimo_biped_v2.xml"))
    data = mujoco.MjData(model)
    jadr = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): model.jnt_qposadr[j]
            for j in range(model.njnt)}
    jrange = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): model.jnt_range[j]
              for j in range(model.njnt)}

    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.0, BASE_Z - 0.15)
    cam.distance = 0.85
    cam.elevation = -12

    renderer = mujoco.Renderer(model, height=H, width=W)
    font, font_small = load_font(44), load_font(30)
    frames = []

    def neutral():
        data.qpos[:] = 0
        data.qpos[2] = BASE_Z
        data.qpos[3] = 1.0            # identity quat

    def snap(label, live):
        mujoco.mj_forward(model, data)
        cam.azimuth = 118 + 0.028 * len(frames)      # slow orbit for depth
        renderer.update_scene(data, camera=cam)
        img = Image.fromarray(renderer.render())
        d = ImageDraw.Draw(img)
        d.text((30, 24), label, font=font, fill=(255, 255, 255),
               stroke_width=2, stroke_fill=(0, 0, 0))
        for k, (name, deg) in enumerate(live):
            d.text((30, 88 + 34 * k), f"{name:9s} {deg:+6.1f} deg",
                   font=font_small, fill=(235, 235, 235),
                   stroke_width=2, stroke_fill=(0, 0, 0))
        frames.append(np.asarray(img))

    n = int(args.seconds_per_joint * FPS)
    for label, joints in PHASES:
        lo, hi = np.degrees(jrange[joints[0][0]])
        track = sweep_profile(lo, hi, n)
        for ang in track:
            neutral()
            for name, sc in joints:
                data.qpos[jadr[name]] = np.radians(ang) * sc
            snap(f"{label}   [{lo:+.0f} to {hi:+.0f} deg]",
                 [(nm, ang * sc) for nm, sc in joints])

    # combined envelope: crouch -> extend -> forward reach -> back reach
    keyposes = [  # hip_roll, hip_pitch, knee, ankle (L; R mirrors roll)
        (0, 0, 0, 0),
        (18, -60, -95, 40),     # deep crouch, legs spread
        (0, 0, 0, 0),
        (0, 60, -20, -40),      # forward reach
        (0, -60, -5, 20),       # back reach
        (0, 0, 0, 0),
    ]
    seg = int(1.0 * FPS)
    for (a0, b0, c0, d0), (a1, b1, c1, d1) in zip(keyposes, keyposes[1:]):
        for t in ease(np.linspace(0, 1, seg)):
            r, p = a0 + t * (a1 - a0), b0 + t * (b1 - b0)
            k, an = c0 + t * (c1 - c0), d0 + t * (d1 - d0)
            neutral()
            data.qpos[jadr["L_hip_roll"]] = np.radians(r)
            data.qpos[jadr["R_hip_roll"]] = -np.radians(r)
            for s in "LR":
                data.qpos[jadr[f"{s}_hip_pitch"]] = np.radians(p)
                data.qpos[jadr[f"{s}_knee"]] = np.radians(k)
                data.qpos[jadr[f"{s}_ankle"]] = np.radians(an)
            snap("combined envelope",
                 [("hip roll", r), ("hip pitch", p), ("knee", k), ("ankle", an)])

    import imageio.v2 as imageio
    mov = args.out + ".mov"
    raw = args.out + "_frames.mp4"
    imageio.mimsave(raw, frames, fps=FPS, macro_block_size=1)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", raw,
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mov],
                   check=True)
    os.remove(raw)
    print(f"{len(frames)} frames -> {mov}")


if __name__ == "__main__":
    main()
