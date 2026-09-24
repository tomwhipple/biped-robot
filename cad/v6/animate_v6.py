"""Fly-in assembly animation for the v6 robot (the memory rule: on every CAD
layout change, animate the parts flying in along their real insertion paths
to prove they fit).

Each piece of assembly_v6.robot() gets an INSERTION vector -- the direction it
is offered in on the bench, from the part modules' SERVO_INSERT specs where
they exist (servos into grip channels along +z from below, the roll servo
down into the foot cradle, the yaw servos up into their cells, boards down
through the deck slots, the pack down through the aperture, the head down onto
the neck horn) -- and flies in from 60 mm out along that vector over its own
time slot, ground up, one group at a time. Frames are rendered with MuJoCo
from the pieces' STLs; the .mp4 is gitignored, a 2 x 4 filmstrip png is
committed next to it.

    .venv/bin/python cad/v6/animate_v6.py            # renders/assembly_v6_flyin.mp4 + _strip.png
    ARMS=1 .venv/bin/python cad/v6/animate_v6.py     # ..._flyin_arms.mp4 + _strip.png
"""
from __future__ import annotations

import os
import sys
import tempfile

os.environ.setdefault("MUJOCO_GL", "egl")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402
from build123d import export_stl  # noqa: E402
import assembly_v6 as A  # noqa: E402
import dimensions_v6 as V  # noqa: E402

# label prefix -> (insertion direction in the world, the group order)
# direction = where the part COMES FROM (unit vector), applied as an offset
INSERT = [
    ("foot_",              (0, 0, -1), 0),   # feet placed on the floor
    ("servo_ankle_roll_",  (0, 0, 1), 1),    # roll servo dropped into the cradle
    ("ankle_link_",        (0, 0, 1), 2),    # link lowered onto the roll servo discs
    ("servo_ankle_pitch_", (0, 0, 1), 3),    # already in its grip channel (entered along +z from below on
                                              # the bench); shown arriving with the link above it
    ("shin_",              (0, 0, 1), 4),
    ("servo_knee_",        (0, 0, 1), 4),
    ("thigh_",             (0, 0, 1), 5),
    ("servo_hip_pitch_",   (0, 0, 1), 5),
    ("yoke_pitch_",        (0, 0, 1), 6),
    ("yoke_roll_",         (0, 0, 1), 6),
    ("hip_yoke_",          (0, 0, 1), 6),    # HIP_YOKE_VARIANT=single: the one-print yoke lowered onto the pitch servo
                                              # discs from above (same path the pair took; the roll servo then slides in)
    ("servo_hip_roll_",    (1, 0, 0), 7),    # roll servo slides into the carrier bay from the front
    ("yaw_carrier_",       (0, 0, 1), 7),
    ("bearing_6810_",      (0, 0, 1), 8),    # option E: bearing slid DOWN over the hub onto the lip (horn-face side)
    ("yaw_cap_",           (0, 0, 1), 8),    # option E: cap down onto the race, screwed, carrier still on the bench
    ("servo_hip_yaw_",     (0, 0, -1), 8),   # yaw servo offered UP into its cell (arrives from below)
    ("pelvis_v7",          (0, 0, 1), 9),    # pelvis lowered onto the yaw servos
    ("yaw_retainer_",      (0, 0, -1), 10),  # option E: retainer offered UP under the skirt, screwed from below
    ("gd_mock",            (0, 0, 1), 10),   # boards down through the deck slots
    ("pi4_mock",           (0, 0, 1), 10),
    ("pack_mock",          (0, 0, 1), 11),   # pack down through the aperture
    ("servo_neck",         (0, 0, 1), 12),
    ("neck_collar",        (0, 0, 1), 12),
    ("head",               (0, 0, 1), 13),
    # ARMS=1 (cad/v6/arm_v6.py + cad/v6/shoulder_girdle_v6.py). The order IS
    # the bench order, and it is not arbitrary: the girdle's twelve deck pilots
    # include four under the servo bays themselves, so the girdle goes down
    # onto the deck while it is still EMPTY. Girdle down -> each shoulder servo
    # dropped straight into its open bay from above (the only way in: every bay
    # is open upward, a roof over a 32 mm span being unprintable) and screwed
    # from OUTBOARD -> upper arm offered straight IN onto the horn along the
    # joint axis, the one direction a single-sided horn plate can arrive from
    # -> elbow servo slid into the forearm's grip channel from the FRONT (the
    # same channel entry leg_link uses) -> forearm lifted UP between the fork
    # tines onto the two discs.
    ("shoulder_girdle_v6", (0, 0, 1), 14),
    ("servo_shoulder_",    (0, 0, 1), 15),
    ("arm_upper_",         (0, 1, 0), 16),
    ("servo_elbow_",       (1, 0, 0), 17),
    ("arm_fore_",          (0, 0, -1), 18),
]
FLY_MM = 60.0
FRAMES_PER_GROUP = 14
HOLD = 6


def plan(comp):
    out = []
    for i, ch in enumerate(comp.children):
        for pref, d, grp in INSERT:
            if ch.label.startswith(pref):
                v = np.array(d, float)
                # a LATERAL insertion is handed: the left arm is offered from
                # +y, the right from -y. Anything that flies in along y and
                # belongs to a side gets its vector mirrored with the part.
                if v[1] and ch.label.endswith("_R"):
                    v = v * np.array([1.0, -1.0, 1.0])
                out.append((i, ch, v, grp))
                break
        else:
            out.append((i, ch, np.array([0, 0, 1.0]), 99))
    return out


def main():
    import mujoco
    import imageio
    comp = A.robot({})
    pieces = plan(comp)
    tmp = tempfile.mkdtemp(prefix="v6fly_")
    for i, ch, d, grp in pieces:
        export_stl(ch, os.path.join(tmp, f"p{i}.stl"))
    ngroups = max(g for _, _, _, g in pieces if g < 99) + 1
    # one MJCF with every piece as a free-positioned body (mocap) so we can move it per frame
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1280" offheight="960"/><headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4"/></visual>',
           '<asset><texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.92" rgb2="0.8 0.8 0.84" width="512" height="512"/>',
           '<material name="floor" texture="grid" texrepeat="10 10" texuniform="true"/>']
    xml += [f'<mesh name="p{i}" file="p{i}.stl" scale="0.001 0.001 0.001"/>' for i, *_ in pieces]
    xml += ['</asset><worldbody><light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>',
            '<geom type="plane" size="2 2 0.1" material="floor"/>']
    for i, ch, d, grp in pieces:
        c = tuple(ch.color) if ch.color else (0.8, 0.8, 0.8, 1)
        xml.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="p{i}" rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1"/></body>')
    xml += ['</worldbody></mujoco>']
    with open(os.path.join(tmp, "fly.xml"), "w") as f:
        f.write("\n".join(xml))
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "fly.xml"))
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 960, 1280)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, V.TOP_Z / 2000.0]
    # 1.5 is the framing every committed default filmstrip was shot at -- left
    # alone so this change does not silently re-shoot them. The arms make the
    # robot 223 mm wide and add five groups, so their own film pulls in.
    cam.distance = 1.15 if A.arms_on() else 1.5
    cam.elevation = -12
    frames = []
    total = ngroups * (FRAMES_PER_GROUP + HOLD)
    for f in range(total + 20):
        grp_f = f // (FRAMES_PER_GROUP + HOLD)
        sub = f % (FRAMES_PER_GROUP + HOLD)
        for k, (i, ch, dvec, grp) in enumerate(pieces):
            if grp < grp_f:
                s = 1.0
            elif grp == grp_f:
                s = min(1.0, sub / FRAMES_PER_GROUP)
                s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            else:
                s = 0.0
            off = dvec * FLY_MM * (1.0 - s) / 1000.0
            d.mocap_pos[k] = off if s > 0.0 or grp == grp_f else off + np.array([0, 0, 5.0])  # not-yet parts parked out of view
        mujoco.mj_forward(m, d)
        cam.azimuth = 140 + 40 * f / (total + 20)
        r.update_scene(d, cam)
        frames.append(r.render().copy())
    os.makedirs(os.path.join(HERE, "renders"), exist_ok=True)
    variant = A.yaw_bearing_variant()
    suffix = "" if variant == "C" else f"_opt{variant}"
    if A.hip_yoke_variant() == "split":       # the one-print yoke is the default now
        suffix += "_split"
    if A.arms_on():
        suffix += "_arms"
    out = os.path.join(HERE, "renders", f"assembly_v6_flyin{suffix}.mp4")
    with imageio.get_writer(out, fps=20, codec="libx264", quality=8, macro_block_size=None) as w:
        for fr in frames:
            w.append_data(fr)
    # Row 1 = the whole build, row 2 = the part of it this change is ABOUT.
    # With the arms on there are 19 groups, so 8 uniform samples land 5 of the
    # 8 before anything above the knee exists and exactly one inside the five
    # arm groups -- a filmstrip that does not show the thing it was made to
    # prove. When a tail group exists (the arms), row 2 samples only that.
    first_arm = min((g for _, _, _, g in pieces if 14 <= g < 99), default=None)
    if first_arm is None:
        idx = np.linspace(0, len(frames) - 1, 8).astype(int)
    else:
        split = first_arm * (FRAMES_PER_GROUP + HOLD)
        idx = np.concatenate([np.linspace(0, split - 1, 4),
                              np.linspace(split, len(frames) - 1, 4)]).astype(int)
    rows = [np.concatenate([frames[i][::2, ::2] for i in idx[:4]], axis=1),
            np.concatenate([frames[i][::2, ::2] for i in idx[4:]], axis=1)]
    imageio.imwrite(os.path.splitext(out)[0] + "_strip.png", np.concatenate(rows, axis=0))
    print("wrote", out, "and the filmstrip;", len(frames), "frames,", len(pieces), "pieces in", ngroups, "groups")


if __name__ == "__main__":
    main()
