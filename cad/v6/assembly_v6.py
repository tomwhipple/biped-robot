"""v6 assembly: the articulated robot as a kinematic chain of build123d solids.

One table (CHAIN) places every printed part, servo mock and electronics mock
in the STANDING pose from cad/v6/dimensions_v6.py, and `robot(pose)` poses it
through the joint angles (product of rotations about each joint's standing-
pose axis, root first). Everything downstream reads this table: the STEP
export, the pose renders, the ROM interference check (check_assembly_v6.py)
and the fly-in animation (animate_v6.py) -- so a figure cannot show a
placement the checker never tried.

Parts come from the v6 part modules when they exist (leg_link_v6, ankle_link,
foot_v6, pelvis_v7, head) and from v5 for the unchanged hip stack
(yaw_carrier, yoke_roll, yoke_pitch); a missing v6 module falls back to a
labelled envelope box so the chain, the export and the checks run while the
parts are still being drawn.

Variants (env vars; the defaults are the robot to print):
    ARMS=1 (default) | 0        the two get-up arms: the shoulder girdle (both
                                shoulder servos + the neck tube), upper arm,
                                elbow servo, forearm per side (cad/v6/arm_v6.py,
                                cad/v6/shoulder_girdle_v6.py), and the pelvis
                                built with the girdle's deck pilots. ARMS=0 is
                                the armless variant: neck_collar instead of the
                                girdle, a pelvis without the pilots.
    HIP_YOKE_VARIANT=single|split  the one-print hip yoke (default) or the
                                legacy bolted yoke_roll + yoke_pitch pair.

    .venv/bin/python cad/v6/assembly_v6.py                 # step/assembly_v6_arms.step + renders
    ARMS=0 .venv/bin/python cad/v6/assembly_v6.py          # step/assembly_v6.step (armless)
    .venv/bin/python cad/v6/assembly_v6.py --pose knee=-60,hip_pitch=-30
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

os.environ.setdefault("MUJOCO_GL", "egl")   # before anything imports mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import (Box, Color, Compound, Cylinder, Location, Pos, Rot,  # noqa: E402
                       export_step, export_stl)
import dimensions_v6 as V  # noqa: E402
D = V.D
import check_assembly as CA  # noqa: E402  (v5 servo mocks)
import parts as v5parts  # noqa: E402

OUT_STEP = os.path.join(HERE, "step")
OUT_STL = os.path.join(HERE, "stl")
OUT_REN = os.path.join(HERE, "renders")

COL_PRINT = Color(0.82, 0.84, 0.87)
COL_SERVO = Color(0.22, 0.23, 0.27)
COL_FOOT = Color(0.30, 0.31, 0.34)
COL_SOLE = Color(0.93, 0.93, 0.91)       # white silicone rubber sheet
COL_MOCK = Color(0.15, 0.45, 0.75)
COL_HEAD = Color(0.90, 0.88, 0.80)

JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
ARM_JOINTS = ("shoulder", "elbow")


# --------------------------------------------------------------------------- part sources
def _try(modname, fn, *a, **k):
    try:
        mod = __import__(modname)
        return getattr(mod, fn)(*a, **k), True
    except Exception as e:  # noqa: BLE001 -- placeholder while the part is being built
        print(f"  [assembly_v6] {modname}.{fn} unavailable ({type(e).__name__}: {str(e)[:60]}); envelope box used")
        return None, False


def hip_yoke_variant():
    """Which hip yoke (link 2, the roll/pitch universal joint) the assembly
    builds: "single" (DEFAULT since 2026-09-24: cad/v6/hip_yoke_v6.py, the roll
    and pitch clevises as one print, no flange bolts) or "split" (the legacy
    v5 yoke_roll + yoke_pitch_v6 bolted flange to flange) via the
    HIP_YOKE_VARIANT env var. Promoted once both clearance findings its sweep
    exposed were resolved (hip-yoke-single-print.md section 6): the roll-
    corner relief, and hip flexion -120 with the leg link's LL_HIP_RELIEF_X0 trim."""
    return os.environ.get("HIP_YOKE_VARIANT", "single")


def arms_on():
    """Whether this build carries the two get-up arms (docs/design-v6/
    getup-decision-2026-09-17.md). ON by default -- the arms are the design;
    ARMS=0 builds the armless variant (the CAD checks run both)."""
    return os.environ.get("ARMS", "1") not in ("", "0", "no", "false", "off")


def part_hip_yoke():
    """The one-print hip yoke. No envelope fallback: a caller that opted in
    asked for this part, and two bolted pieces are not a stand-in for one."""
    import hip_yoke_v6
    return hip_yoke_v6.hip_yoke_v6()


def hip_yoke_seat_zone(side):
    """World-frame (standing pose) solid of the one-print yoke's DESIGNED
    contacts -- hip_yoke_v6.disc_seat_zones -- for check_assembly_v6's
    "~seats" rows. Cached like the parts."""
    import hip_yoke_v6
    y = V.HIP_SEP / 2 * (1 if side == "L" else -1)
    return cached(("hip_yoke_seats", side), lambda: Pos(0, y, V.HIP_ROLL_Z) * hip_yoke_v6.disc_seat_zones())


def posed_hip_yoke_seats(side, pose):
    """hip_yoke_seat_zone carried to `pose` with link 2 (after hip_yaw and
    hip_roll), exactly as robot() moves the yoke itself."""
    pieces, joints = leg_chain(side)
    angles = [pose.get(f"{side}_{j}", pose.get(j, 0.0)) for j in JOINTS]
    return pose_transforms(joints, angles)[2] * hip_yoke_seat_zone(side)


def part_yaw_carrier():
    """v6 carrier (v5's + the hip-yaw bearing hub); falls back to v5's."""
    try:
        import yaw_carrier_v6
        return yaw_carrier_v6.yaw_carrier_v6()
    except Exception as e:  # noqa: BLE001
        print(f"  [assembly_v6] yaw_carrier_v6 unavailable ({type(e).__name__}: {e}); v5 yaw_carrier")
        return v5parts.yaw_carrier()


def part_yoke_pitch():
    """v6 clevis (flange chamfer for hip flexion 125); falls back to v5's."""
    try:
        import yoke_pitch_v6
        return yoke_pitch_v6.yoke_pitch_v6()
    except Exception as e:  # noqa: BLE001
        print(f"  [assembly_v6] yoke_pitch_v6 unavailable ({type(e).__name__}: {e}); v5 yoke_pitch")
        return v5parts.yoke_pitch()


def part_leg_link():
    s, ok = _try("leg_link_v6", "leg_link_v6")
    if ok:
        return s
    # envelope: web/plates box from the grip to the pads
    return Pos(0, 1.0, -V.LL_DROP / 2) * Box(30, 44, V.LL_DROP + 20)


def part_ankle_link():
    s, ok = _try("ankle_link", "ankle_link")
    if ok:
        return s
    return Pos(0, 1.0, -V.AL_DROP / 2) * Box(44, 44, V.AL_DROP + 20)


def part_sole(side):
    """The 2 mm silicone sole stuck under the foot plate (foot_v6.sole_tpu, the
    cutting outline): z -TPU_SOLE_T..0 in the foot frame."""
    s, ok = _try("foot_v6", "sole_tpu", side)
    if ok:
        return s
    sgn = 1 if side == "L" else -1
    return Pos(V.FOOT_TOE - V.FOOT_L_LEN / 2, sgn * V.FOOT_Y_OFF, -V.TPU_SOLE_T / 2) * Box(V.FOOT_L_LEN, V.FOOT_W, V.TPU_SOLE_T)


def part_foot(side):
    s, ok = _try("foot_v6", "foot", side)
    if ok:
        return s
    sgn = 1 if side == "L" else -1
    return Pos(V.FOOT_TOE - V.FOOT_L_LEN / 2, sgn * V.FOOT_Y_OFF, V.FOOT_PLATE_T / 2) * Box(V.FOOT_L_LEN, V.FOOT_W, V.FOOT_PLATE_T)


def part_pelvis():
    s, ok = _try("pelvis_v7", "pelvis_v7", arms_on())
    if ok:
        return s
    x0, x1 = V.HOUSING_X
    return Pos((x0 + x1) / 2, 0, V.YAW_BOX_BOT_Z / 2) * Box(x1 - x0, 2 * V.HOUSING_HW, -V.YAW_BOX_BOT_Z)


def part_bearing_housing():
    """The hip-yaw bearings' outer-race seats, screwed under the pelvis
    (yaw_bearing_housing.py); pelvis-local like the pelvis."""
    import yaw_bearing_housing
    return yaw_bearing_housing.yaw_bearing_housing()


def part_collar():
    """The neck's mount. With the arms (the default) this is not a standalone collar:
    shoulder_girdle_v6 absorbs it, so the neck tube, the two shoulder pods and
    the trapezius webs are ONE print and the head rises out of the same solid
    the arms hang off."""
    if arms_on():
        s, ok = _try("shoulder_girdle_v6", "shoulder_girdle_v6")
        if ok:
            return s
    s, ok = _try("neck_collar", "neck_collar")
    if ok:
        return s
    return Pos(-12.5, 0, 16) * Box(50, 30, 32)


def part_neck_floor():
    """The neck servo's seat (issue #90): a plate screwed up into the neck
    tube, in the deck's battery aperture. No envelope fallback -- a missing
    seat is the defect this part fixes."""
    import neck_floor
    return neck_floor.neck_floor()


def part_head():
    s, ok = _try("head", "head")
    if ok:
        return s
    return Pos(0, 0, V.HEAD_H / 2 + V.HEAD_BASE_T) * Box(V.HEAD_D, V.HEAD_W, V.HEAD_H)


def mock_pack():
    return Pos((V.BATT_X[0] + V.BATT_X[1]) / 2, 0, (V.BATT_Z[0] + V.BATT_Z[1]) / 2) * Box(V.BATT[1], V.BATT[0], V.BATT[2])


def mock_pi():
    return Pos((V.PI_PCB_X0 + V.PI_PCB_X1) / 2, 0, V.PI_CZ) * Box(V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1])


def mock_gd():
    return Pos((V.GD_PCB_X0 + V.GD_PCB_X1) / 2, 0, (V.GD_BOT_Z + V.GD_TOP_Z) / 2) * Box(D.BR_PCB_T, D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])


def mock_roll_servo_in_foot(side):
    """roll servo lying across the foot: output axis +X (horn forward), case
    length along +y OUTBOARD (cable end), width vertical. Built from v5's
    axis-X mock (case length along Z in the carrier) rolled about X."""
    sgn = 1 if side == "L" else -1
    m = CA.servo_mock_x()
    bb = m.bounding_box()
    # v5 mock_x: case length runs along -z? measure and roll so the LONG side goes to sgn*+y
    long_down = abs(bb.min.Z) > abs(bb.max.Z)
    # Rot(+90 about X) maps -z -> +y; the long side must end up on sgn*+y
    ang = 90 if (long_down == (sgn > 0)) else -90
    out = Rot(ang, 0, 0) * m
    bb2 = out.bounding_box()
    assert (bb2.max.Y if sgn > 0 else -bb2.min.Y) > 30.0, "roll servo mock: long side not outboard"
    return out


# --------------------------------------------------------------------------- chain
_CACHE = {}


def cached(key, fn):
    """standing-pose solids are built once per process; posing is a cheap
    Location product on top (the ROM checker poses hundreds of times)."""
    if key not in _CACHE:
        _CACHE[key] = fn()
    return _CACHE[key]


def leg_chain(side):
    return cached(("leg", side), lambda: _leg_chain(side))


def _leg_chain(side):
    """[(link_index, label, colour, world_solid_in_standing_pose)] for one leg,
    plus the joint list [(name, axis_unit, point)] in the standing pose."""
    y = V.HIP_SEP / 2 * (1 if side == "L" else -1)
    at = lambda z: Pos(0, y, z)
    joints = [("hip_yaw", (0, 0, 1), (0, y, V.HIP_YAW_Z)),
              ("hip_roll", (1, 0, 0), (0, y, V.HIP_ROLL_Z)),
              ("hip_pitch", (0, 1, 0), (0, y, V.HIP_PITCH_Z)),
              ("knee", (0, 1, 0), (0, y, V.KNEE_Z)),
              ("ankle_pitch", (0, 1, 0), (0, y, V.ANKLE_PITCH_Z)),
              ("ankle_roll", (1, 0, 0), (0, y, V.ANKLE_ROLL_Z))]
    pieces = [
        (0, f"servo_hip_yaw_{side}", COL_SERVO, Pos(0, y, V.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z(idler_disc=False)),
        (1, f"yaw_carrier_{side}", COL_PRINT, Pos(0, y, V.HIP_YAW_Z) * part_yaw_carrier()),
        *yaw_bearing_pieces(side, y),
        (1, f"servo_hip_roll_{side}", COL_SERVO, at(V.HIP_ROLL_Z) * CA.servo_mock_x()),
        *hip_yoke_pieces(side, at),
        (3, f"servo_hip_pitch_{side}", COL_SERVO, at(V.HIP_PITCH_Z) * CA.servo_mock_y()),
        (3, f"thigh_{side}", COL_PRINT, at(V.HIP_PITCH_Z) * part_leg_link()),
        (4, f"servo_knee_{side}", COL_SERVO, at(V.KNEE_Z) * CA.servo_mock_y()),
        (4, f"shin_{side}", COL_PRINT, at(V.KNEE_Z) * part_leg_link()),
        (5, f"servo_ankle_pitch_{side}", COL_SERVO, at(V.ANKLE_PITCH_Z) * CA.servo_mock_y()),
        (5, f"ankle_link_{side}", COL_PRINT, at(V.ANKLE_PITCH_Z) * part_ankle_link()),
        (6, f"servo_ankle_roll_{side}", COL_SERVO, at(V.ANKLE_ROLL_Z) * mock_roll_servo_in_foot(side)),
        (6, f"foot_{side}", COL_FOOT, at(V.TPU_SOLE_T) * part_foot(side)),
        (6, f"sole_{side}", COL_SOLE, at(V.TPU_SOLE_T) * part_sole(side)),
    ]
    return pieces, joints


def hip_yoke_pieces(side, at):
    """Link 2 -- the hip universal joint between the roll servo (link 1) and
    the pitch servo (link 3): the bolted pair (default) or the one print.
    Both yokes' frames are the joint frames themselves (roll axis / pitch
    axis at their origins), so the split parts are placed at HIP_ROLL_Z and
    HIP_PITCH_Z; the single part carries the pitch clevis at -ROLL_TO_PITCH
    in its own frame and is placed at HIP_ROLL_Z alone."""
    if hip_yoke_variant() == "single":
        return [(2, f"hip_yoke_{side}", COL_PRINT, at(V.HIP_ROLL_Z) * part_hip_yoke())]
    return [(2, f"yoke_roll_{side}", COL_PRINT, at(V.HIP_ROLL_Z) * v5parts.yoke_roll()),
            (2, f"yoke_pitch_{side}", COL_PRINT, at(V.HIP_PITCH_Z) * part_yoke_pitch())]


def yaw_bearing_pieces(side, y):
    """The hip-yaw bearing (6810-2RS, bought), drawn as its two races split at
    SKF's ring shoulders (V.YAW_BRG_INNER_RING_R / _OUTER_RING_R): the inner
    race on the carrier's hub turns with the leg (link 1), the outer race in
    the bearing housing's recess stays put (link 0)."""
    z0, z1 = V.HIP_YAW_Z + V.YAW_BRG_BAND_Z[0], V.HIP_YAW_Z + V.YAW_BRG_BAND_Z[1]

    def ring(r0, r1):
        return Pos(0, y, (z0 + z1) / 2) * (Cylinder(r1, z1 - z0) - Cylinder(r0, z1 - z0 + 2))
    return [
        (1, f"bearing_inner_{side}", COL_MOCK, ring(V.YAW_BRG_ID / 2, V.YAW_BRG_INNER_RING_R)),
        (0, f"bearing_outer_{side}", COL_MOCK, ring(V.YAW_BRG_OUTER_RING_R, V.YAW_BRG_OD / 2)),
    ]


def torso_pieces():
    return cached("torso", _torso_pieces)


def _torso_pieces():
    z = V.DECK_TOP_Z
    return [
        (0, "pelvis_v7", COL_PRINT, Pos(0, 0, z) * part_pelvis()),
        (0, "yaw_bearing_housing", COL_PRINT, Pos(0, 0, z) * part_bearing_housing()),
        (0, "pack_mock", COL_MOCK, Pos(0, 0, z) * mock_pack()),
        (0, "pi4_mock", COL_MOCK, Pos(0, 0, z) * mock_pi()),
        (0, "gd_mock", COL_MOCK, Pos(0, 0, z) * mock_gd()),
        (0, "servo_neck", COL_SERVO, Pos(V.NECK_X, 0, z + V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()),
        (0, "neck_floor", COL_PRINT, Pos(0, 0, z) * part_neck_floor()),
        (0, "shoulder_girdle_v6" if arms_on() else "neck_collar", COL_PRINT,
         Pos(0 if arms_on() else V.NECK_X, 0, z) * part_collar()),
    ]


def arm_chain(side):
    return cached(("arm", side), lambda: _arm_chain(side))


def _arm_chain(side):
    """[(link_index, label, colour, world_solid_in_the_HANGING pose)] for one
    arm, plus its joint list. Link 0 = the torso (the bracket and the shoulder
    servo's stator ride it), 1 = after the shoulder, 2 = after the elbow. The
    hanging pose IS shoulder 0 / elbow 0 -- the actuators' own rest, and the
    idle pose the walk gate was measured in (study-shoulder-arms.md section 5).

    The bracket and the shoulder servo are drawn in the PELVIS frame, so they
    are lifted by DECK_TOP_Z here exactly as pelvis_v7 itself is."""
    import arm_v6
    import shoulder_girdle_v6 as SG
    y = V.ARM_Y * (1 if side == "L" else -1)
    deck = Pos(0, 0, V.DECK_TOP_Z)
    at_sh = Pos(V.ARM_SHOULDER_X, y, V.ARM_SHOULDER_Z)
    at_el = Pos(V.ARM_SHOULDER_X, y, V.ARM_ELBOW_Z)
    joints = [("shoulder", (0, 1, 0), (V.ARM_SHOULDER_X, y, V.ARM_SHOULDER_Z)),
              ("elbow", (0, 1, 0), (V.ARM_SHOULDER_X, y, V.ARM_ELBOW_Z))]
    pieces = [
        # the shoulder bracket is gone: shoulder_girdle_v6 carries BOTH shoulder
        # servos and the neck, and it is drawn once in torso_pieces(). Only the
        # servo itself rides link 0 here.
        (0, f"servo_shoulder_{side}", COL_SERVO, deck * SG.shoulder_servo_mock(side)),
        (1, f"arm_upper_{side}", COL_PRINT, at_sh * arm_v6.arm_upper_v6(side)),
        # the elbow servo's case rides the UPPER arm (arm_v6.elbow_servo_mock)
        (1, f"servo_elbow_{side}", COL_SERVO, at_el * arm_v6.elbow_servo_mock(side)),
        (2, f"arm_fore_{side}", COL_PRINT, at_el * arm_v6.arm_fore_v6(side)),
    ]
    return pieces, joints


def head_pieces():
    return cached("head", lambda: [(1, "head", COL_HEAD, Pos(V.NECK_X, 0, V.NECK_HORN_Z) * part_head())])


def rot_about(axis, point, deg):
    """Location: rotation of `deg` about the unit axis through `point` (world)."""
    ax = tuple(axis)
    rot = Rot(ax[0] * deg, ax[1] * deg, ax[2] * deg)   # axis-aligned axes only
    return Pos(*point) * rot * Pos(-point[0], -point[1], -point[2])


def pose_transforms(joints, angles):
    """cumulative world Location per link index (0 = root, k = after joint k)."""
    T = [Location()]
    for (name, axis, point), a in zip(joints, angles):
        T.append(T[-1] * rot_about(axis, point, a))
    return T


def robot(pose=None):
    """Compound of every piece at the given pose. pose: dict of joint name ->
    deg, keys like 'L_knee', 'knee' (both legs), 'neck'."""
    pose = pose or {}
    children = []
    for idx, label, col, solid in torso_pieces():
        s = solid
        s.label, s.color = label, col
        children.append(s)
    neck = pose.get("neck", 0.0)
    Tn = rot_about((0, 0, 1), (V.NECK_X, 0, V.NECK_HORN_Z), neck)
    for idx, label, col, solid in head_pieces():
        s = Tn * solid
        s.label, s.color = label, col
        children.append(s)
    for side in ("L", "R"):
        pieces, joints = leg_chain(side)
        angles = [pose.get(f"{side}_{j}", pose.get(j, 0.0)) for j in JOINTS]
        T = pose_transforms(joints, angles)
        for idx, label, col, solid in pieces:
            s = T[idx] * solid
            s.label, s.color = label, col
            children.append(s)
    if arms_on():
        for side in ("L", "R"):
            pieces, joints = arm_chain(side)
            angles = [pose.get(f"{side}_{j}", pose.get(j, 0.0)) for j in ARM_JOINTS]
            T = pose_transforms(joints, angles)
            for idx, label, col, solid in pieces:
                s = T[idx] * solid
                s.label, s.color = label, col
                children.append(s)
    return Compound(label="bimo_v6", children=children)


def parse_pose(txt):
    pose = {}
    for kv in (txt or "").split(","):
        if kv.strip():
            k, v = kv.split("=")
            pose[k.strip()] = float(v)
    return pose


# --------------------------------------------------------------------------- render (MuJoCo, from STLs)
def render(comp: Compound, png_path, views=((150, -12), (90, -8), (0, -12)), size=(640, 800)):
    import mujoco
    import numpy as np
    import imageio
    os.environ.setdefault("MUJOCO_GL", "egl")
    tmp = tempfile.mkdtemp(prefix="v6asm_")
    geoms = []
    for i, ch in enumerate(comp.children):
        path = os.path.join(tmp, f"p{i}.stl")
        export_stl(ch, path)
        c = ch.color
        try:
            rgba = f"{c.red:.3f} {c.green:.3f} {c.blue:.3f} 1" if c else "0.8 0.8 0.8 1"
        except AttributeError:
            try:
                t = tuple(c)
                rgba = f"{t[0]:.3f} {t[1]:.3f} {t[2]:.3f} 1"
            except Exception:  # noqa: BLE001
                rgba = "0.8 0.8 0.8 1"
        geoms.append((f"p{i}", path, rgba))
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1600" offheight="1600"/><headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4"/></visual>',
           '<asset><texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.92" rgb2="0.8 0.8 0.84" width="512" height="512"/>',
           '<material name="floor" texture="grid" texrepeat="10 10" texuniform="true"/>']
    xml += [f'<mesh name="{n}" file="{os.path.basename(p)}" scale="0.001 0.001 0.001"/>' for n, p, _ in geoms]
    xml += ['</asset><worldbody><light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>',
            '<geom type="plane" size="2 2 0.1" material="floor"/>']
    xml += [f'<geom type="mesh" mesh="{n}" rgba="{rgba}"/>' for n, _, rgba in geoms]
    xml += ['</worldbody></mujoco>']
    with open(os.path.join(tmp, "asm.xml"), "w") as f:
        f.write("\n".join(xml))
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "asm.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, V.TOP_Z / 2000.0]
    cam.distance = 1.6
    imgs = []
    for az, el in views:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        imgs.append(r.render().copy())
    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    imageio.imwrite(png_path, np.concatenate(imgs, axis=1))
    return png_path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="")
    ap.add_argument("--no-step", action="store_true")
    ap.add_argument("--png", default=os.path.join(OUT_REN, "assembly_v6.png"))
    ap.add_argument("--step", default=None)
    a = ap.parse_args(argv)
    if a.step is None:
        # the robot (arms) and the ARMS=0 variant each write their own file
        a.step = os.path.join(OUT_STEP, "assembly_v6_arms.step" if arms_on() else "assembly_v6.step")
    if arms_on() and a.png == os.path.join(OUT_REN, "assembly_v6.png"):
        a.png = os.path.join(OUT_REN, "assembly_v6_arms.png")
    comp = robot(parse_pose(a.pose))
    bb = comp.bounding_box()
    print(f"assembly bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}  z {bb.min.Z:.1f}..{bb.max.Z:.1f} mm, {len(comp.children)} pieces")
    if not a.no_step:
        os.makedirs(OUT_STEP, exist_ok=True)
        export_step(comp, a.step)
        print("wrote", a.step)
    print("wrote", render(comp, a.png))


if __name__ == "__main__":
    main()
