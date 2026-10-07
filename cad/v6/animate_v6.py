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

    .venv/bin/python cad/v6/animate_v6.py            # renders/assembly_v6_flyin_arms.mp4 + _strip.png (the robot)
    ARMS=0 .venv/bin/python cad/v6/animate_v6.py     # ..._flyin.mp4 + _strip.png (armless)

The filmstrip is two rows: the whole build at even intervals, then the parts
this build is ABOUT caught halfway along their insertion paths (FEATURE_GROUPS:
the girdle and neck floor going down, the neck servo dropping into its tube,
the arms, and the bearing housing going up with the bearings onto the
carrier hubs).
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
    ("sole_",              (0, 0, -1), 0),   # silicone sole stuck on from below
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
    ("servo_hip_yaw_",     (0, 0, -1), 8),   # yaw servo offered UP into its cell (arrives from below)
    ("pelvis_v7",          (0, 0, 1), 9),    # pelvis lowered onto the yaw servos
    ("yaw_bearing_housing", (0, 0, -1), 10), # housing UP from below with both bearings pressed into it:
    ("bearing_inner_",     (0, 0, -1), 10),  # the inner races slide onto the carriers' hubs, the housing
    ("bearing_outer_",     (0, 0, -1), 10),  # meets the cell block and is screwed from below
    ("gd_mock",            (0, 0, 1), 11),   # boards down through the deck slots
    ("pi4_mock",           (0, 0, 1), 11),
    ("pack_mock",          (0, 0, 1), 12),   # pack down through the aperture -- BEFORE the girdle:
                                              # it cannot pass the neck tube or the neck floor afterwards
    # The neck is built into its tube ON THE BENCH (neck_floor.py): the floor
    # screwed up into the tube's bosses, the servo dropped in onto it and
    # screwed from below. Then the girdle (or, armless, the collar) goes onto
    # the deck with the floor passing down through the battery aperture. The
    # film shows the tube + floor going down together, then the servo going
    # into the tube from above -- on the bench the servo is in before the
    # tube goes on (its stator screws are under the floor, over the pack).
    ("shoulder_girdle_v6", (0, 0, 1), 13),
    ("neck_collar",        (0, 0, 1), 13),
    ("neck_floor",         (0, 0, 1), 13),
    ("servo_neck",         (0, 0, 1), 14),   # dropped into the tube from above, onto the floor's pads
    ("head",               (0, 0, 1), 15),
    # ARMS (cad/v6/arm_v6.py): each shoulder servo dropped straight into its
    # open pod from above (the only way in: a roof over a 32 mm span is
    # unprintable) and screwed from OUTBOARD -> upper arm offered straight IN
    # onto the horn along the joint axis, the one direction a single-sided
    # horn plate can arrive from -> elbow servo slid into the forearm's grip
    # channel from the FRONT (the same channel entry leg_link uses) -> forearm
    # lifted UP between the fork tines onto the two discs.
    ("servo_shoulder_",    (0, 0, 1), 16),
    ("arm_upper_",         (0, 1, 0), 17),
    ("servo_elbow_",       (1, 0, 0), 18),
    ("arm_fore_",          (0, 0, -1), 19),
]
# filmstrip row 2, by arms?: the insertion groups the strip shows
FEATURE_GROUPS = {True: (10, 13, 14, 17, 19), False: (10, 12, 13, 14, 15)}
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
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1280" offheight="960"/><headlight diffuse="0.55 0.55 0.55" ambient="0.35 0.35 0.37"/></visual>',
           '<asset><texture name="sky" type="skybox" builtin="gradient" rgb1="0.95 0.96 0.97" rgb2="0.84 0.86 0.90" width="256" height="256"/>',
           '<texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.92" rgb2="0.8 0.8 0.84" width="512" height="512"/>',
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

    def pose_frame(f):
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

    for f in range(total + 20):
        pose_frame(f)
        cam.azimuth = 140 + 40 * f / (total + 20)
        r.update_scene(d, cam)
        frames.append(r.render().copy())
    os.makedirs(os.path.join(HERE, "renders"), exist_ok=True)
    suffix = ""
    if A.hip_yoke_variant() == "split":       # the one-print yoke is the default now
        suffix += "_split"
    if A.arms_on():
        suffix += "_arms"
    out = os.path.join(HERE, "renders", f"assembly_v6_flyin{suffix}.mp4")
    with imageio.get_writer(out, fps=20, codec="libx264", quality=8, macro_block_size=None) as w:
        for fr in frames:
            w.append_data(fr)
    # Row 1 = the whole build at even intervals; row 2 = the groups this
    # build is ABOUT, each caught halfway along its insertion path (uniform
    # samples of 19 groups land mostly on the legs and miss the thing the
    # film was made to prove).
    present = {g for _, _, _, g in pieces}
    feature = [g for g in FEATURE_GROUPS[A.arms_on()] if g in present]
    n = len(feature)
    per = FRAMES_PER_GROUP + HOLD
    top = np.linspace(0, len(frames) - 1, n).astype(int)
    # row 2: each feature group halfway in, the camera closed in on that
    # group's own parts (at their seats) so the insertion is legible
    close = []
    for g in feature:
        pose_frame(g * per + FRAMES_PER_GROUP // 2)
        lo, hi = np.full(3, 1e9), np.full(3, -1e9)
        for _, ch, dvec, grp in pieces:
            if grp == g:
                bb = ch.bounding_box()
                a, b = np.array([bb.min.X, bb.min.Y, bb.min.Z]) / 1e3, np.array([bb.max.X, bb.max.Y, bb.max.Z]) / 1e3
                lo = np.minimum(lo, np.minimum(a, a + dvec * FLY_MM / 2e3))
                hi = np.maximum(hi, np.maximum(b, b + dvec * FLY_MM / 2e3))
        cam.lookat[:] = (lo + hi) / 2
        cam.distance = max(0.22, 2.2 * float(np.linalg.norm(hi - lo)))
        cam.azimuth, cam.elevation = 150, -20
        r.update_scene(d, cam)
        close.append(r.render().copy())
    rows = [np.concatenate([frames[i][::2, ::2] for i in top], axis=1),
            np.concatenate([fr[::2, ::2] for fr in close], axis=1)]
    imageio.imwrite(os.path.splitext(out)[0] + "_strip.png", np.concatenate(rows, axis=0))
    print("wrote", out, "and the filmstrip;", len(frames), "frames,", len(pieces), "pieces in", ngroups, "groups")


if __name__ == "__main__":
    main()
