"""Articulated pose export for FreeCAD demonstrations.

Poses ALL FIVE joints per leg (yaw / roll / hip / knee / ankle, both legs the
same), computes every relatively-moving printed/screw pair's interference at
that pose, and writes a labeled STEP. Any pair that actually intersects is
ALSO exported as a solid-red "COLLIDE_*" body -- open the STEP in FreeCAD and
the collision is a visible part in the tree, no measuring needed.

Run:
  .venv/bin/python cad/export_pose.py --hip -60                 # design intent
  .venv/bin/python cad/export_pose.py --hip -110 --knee -95 --ankle 40  # pike

  -> cad/step/poses/pose_<spec>.step   (gitignored -- regenerate at will)

Every servo-case screw is an M2.5 FLAT head sitting flush in a countersink.
The pan-head variant was retired 2026-07-28: proud O5.0 x 2.0 heads wedged the
0.70 mm running band at the yoke/fork arm and skewed the link from ~+/-15 deg
of hip travel, which is what sent the build to flat heads in the first place.
"""
import argparse
import os
from build123d import Pos, Rot, Compound, Color, export_step

import dimensions as D
import parts
import fasteners as F
import check_assembly as CA
import export_assembly as A

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step", "poses")
COL_RED = Color(0.95, 0.12, 0.12)


def hinge(ly, z, rx=0.0, ry=0.0, rz=0.0):
    """Rotation about an axis through (0, ly, z) -- world-frame joint pivot."""
    return Pos(0, ly, z) * Rot(rx, ry, rz) * Pos(0, -ly, -z)


def posed_leg(ly, tag, yaw, roll, hip, knee, ankle):
    """One leg's pieces + the pose transform of every kinematic stage.
    Returns (pieces, stages) where stages maps stage name -> (transform,
    solids-for-collision) for the pair checks."""
    at = lambda z: Pos(0, ly, z)
    Ty = hinge(ly, 0, rz=yaw)                       # yaw axis vertical at leg
    Tr = Ty * hinge(ly, D.HIP_ROLL_Z, rx=roll)      # roll axis along X
    Th = Tr * hinge(ly, D.HIP_PITCH_Z, ry=hip)      # pitch axes along Y
    Tk = Th * hinge(ly, D.KNEE_Z, ry=knee)
    Ta = Tk * hinge(ly, D.ANKLE_Z, ry=ankle)

    grip = F.leg_link_screws()
    # collision sets: printed parts + screws only. Servos are excluded from
    # PAIRS (disc faces contact them by design) but exported for context.
    stage = {
        "carrier": (Ty, Pos(0, ly, D.HIP_YAW_Z)
                    * (parts.yaw_carrier() + F.yaw_carrier_screws())),
        "yoke": (Tr, at(D.HIP_ROLL_Z) * (
            Rot(0, 0, 0) * parts.yoke_roll() + F.disc_screws_x()
            + F.flange_bolts())
            + at(D.HIP_PITCH_Z) * (parts.yoke_pitch() + F.disc_screws_y())),
        "thigh": (Th, at(D.HIP_PITCH_Z) * (parts.leg_link() + grip)
                  + at(D.KNEE_Z) * F.disc_screws_y()),
        "shin": (Tk, at(D.KNEE_Z) * (parts.leg_link() + grip)
                 + at(D.ANKLE_Z) * F.disc_screws_y()),
        "foot": (Ta, at(D.TPU_PROUD) * (parts.foot() + F.foot_screws())),
    }
    pieces = [
        A.piece(f"servo_hip_yaw_{tag}", A.COL_SERVO,
                Pos(0, ly, D.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()),
        A.piece(f"servo_hip_roll_{tag}", A.COL_SERVO,
                Ty * at(D.HIP_ROLL_Z) * CA.servo_mock_x()),
        A.piece(f"servo_hip_pitch_{tag}", A.COL_SERVO,
                Th * at(D.HIP_PITCH_Z) * CA.servo_mock_y()),
        A.piece(f"servo_knee_{tag}", A.COL_SERVO,
                Tk * at(D.KNEE_Z) * CA.servo_mock_y()),
        A.piece(f"servo_ankle_{tag}", A.COL_SERVO,
                Ta * at(D.ANKLE_Z) * Rot(0, 90, 0) * CA.servo_mock_y()),
    ]
    for name, (T, solid) in stage.items():
        pieces.append(A.piece(f"{name}_{tag}",
                              A.COL_STEEL if "screw" in name else A.COL_PRINT,
                              T * solid))
    posed = {k: T * s for k, (T, s) in stage.items()}
    return pieces, posed


def main():
    ap = argparse.ArgumentParser()
    for j in ("yaw", "roll", "hip", "knee", "ankle"):
        ap.add_argument(f"--{j}", type=float, default=0.0)
    ap.add_argument("--name", default=None)
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    spec = (f"y{args.yaw:+.0f}_r{args.roll:+.0f}_h{args.hip:+.0f}"
            f"_k{args.knee:+.0f}_a{args.ankle:+.0f}"
            )
    name = args.name or f"pose_{spec}"

    children = [
        A.piece("pelvis", A.COL_PRINT,
                Pos(0, 0, A.DECK_TOP_Z) * parts.pelvis()),
        # v5 torso (2026-08-04): battery_tray under the deck + board_frame on
        # the housing's rear wall. Both are torso-fixed, so they join the torso
        # collision body below -- and in v5 that matters more than it did: the
        # tray hangs INSIDE the carrier's r~25.1 plan circle and is kept clear
        # of it by z-separation alone (TORSO_FLOOR_Z), so a posed check is
        # exactly where a mis-sized sweep would show up.
        A.piece("battery_tray", A.COL_PRINT,
                Pos(0, 0, A.DECK_TOP_Z) * parts.battery_tray()),
        A.piece("board_frame", A.COL_PRINT,
                Pos(0, 0, A.DECK_TOP_Z) * parts.board_frame()),
        A.piece("screws_deck", A.COL_STEEL,
                Pos(0, 0, A.DECK_TOP_Z) * F.deck_stator_screws()),
    ]
    torso_col = Pos(0, 0, A.DECK_TOP_Z) * (parts.pelvis()
                                           + parts.battery_tray()
                                           + parts.board_frame()
                                           + F.deck_stator_screws())
    legs = {}
    for ly, tag in ((D.HIP_SEP / 2, "L"), (-D.HIP_SEP / 2, "R")):
        pieces, posed = posed_leg(ly, tag, args.yaw, args.roll, args.hip,
                                  args.knee, args.ankle)
        children += pieces
        legs[tag] = posed

    # relatively-moving pairs, checked at THIS pose (adjacent stages + the
    # non-adjacent pairs the multi-axis audit watches + torso/cross-leg)
    pairs = [("carrier", "yoke"), ("yoke", "thigh"), ("thigh", "shin"),
             ("shin", "foot"), ("yoke", "shin"), ("thigh", "foot"),
             ("carrier", "thigh")]
    print(f"pose  yaw {args.yaw:+.0f}  roll {args.roll:+.0f}  "
          f"hip {args.hip:+.0f}  knee {args.knee:+.0f}  "
          f"ankle {args.ankle:+.0f}"
          )
    hit = 0
    for tag, posed in legs.items():
        for a, b in pairs:
            i = posed[a] & posed[b]
            v = 0.0 if i is None else i.volume
            if v > 0.5:
                hit += 1
                children.append(A.piece(f"COLLIDE_{a}_{b}_{tag}", COL_RED, i))
                print(f"  {tag} {a:8s} vs {b:8s}  ** {v:8.1f} mm3 COLLISION "
                      f"** -> red body in STEP")
            else:
                d = posed[a].distance_to(posed[b])
                print(f"  {tag} {a:8s} vs {b:8s}     gap {d:5.2f} mm")
        i = torso_col & posed["carrier"]
        v = 0.0 if i is None else i.volume
        if v > 0.5:
            hit += 1
            children.append(A.piece(f"COLLIDE_pelvis_carrier_{tag}", COL_RED, i))
            print(f"  {tag} pelvis   vs carrier  ** {v:8.1f} mm3 COLLISION **")
    i = legs["L"]["carrier"] & legs["R"]["carrier"]
    v = 0.0 if i is None else i.volume
    if v > 0.5:
        hit += 1
        children.append(A.piece("COLLIDE_carrier_L_R", COL_RED, i))
        print(f"  L carrier vs R carrier  ** {v:8.1f} mm3 COLLISION **")

    path = os.path.join(OUT, f"{name}.step")
    export_step(Compound(label=name, children=children), path)
    print(f"{'COLLISIONS: ' + str(hit) if hit else 'no collisions'} -> {path}")


if __name__ == "__main__":
    main()
