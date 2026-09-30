"""Per-hop servo-bus lead lengths off the v6 CAD (issue #77).

Every hop of the proposed 17-servo split-chain layout (docs/servo-map.md
§2.1 = firmware .../obs/bus_map.h's kBusPort chain) measured from the v6 CAD
constants, standing (idle) pose, mm. Each board port splits at the board
into one leg branch and one girdle branch.

A "hop" is ONE physical cable piece. The stock ST3215 lead is 150 mm
(docs/wiring.md "Servo bus"); the verdict column is against it. All spans
are GEOMETRIC MINIMA along the stated route: straight line between the
modelled plug faces, plus the fold loop of the joint crossed. No crimp
tails and no service loops beyond the stated folds. The "buy" column rounds
span + loop + WORK_SLACK up to 10 mm.

Model
-----
* Both ports of an ST3215 sit in the IDLER-side connector trench (the
  SV_CONN_L band 11.75..16.35 mm from the axis toward the cable end), the
  plug face ~2 mm proud of the real idler case face (SV_IDLER_CASE_FACE,
  -14.75). Each anchor below is therefore 14.05 mm toward the cable end and
  16.75 mm along the idler normal from the joint axis, carried through each
  servo's PLACEMENT exactly as the assembly mocks place them
  (assembly_v6's chain tables; check_assembly.servo_mock_*):
    hip yaw        servo_mock_z: case fore-aft in the v7 cell, hanging off
                   the cell ceiling, cable end AFT, idler faces UP; the plug
                   exits into the pelvis's yaw-connector riser
                   (pelvis_v7.py:207 "leads then run free through that open
                   volume to the deck's leg-bus slot").
    hip roll       servo_mock_x: the yaw carrier's bay, output end DOWN,
                   cable end UP, idler faces FORWARD (-x).
    hip pitch / knee / ankle pitch   servo_mock_y: the grip channel of the
                   link above, case hanging below the axis, cable end UP,
                   idler faces -y (both legs -- the leg parts are
                   translations, not mirrors; on the RIGHT leg the plug
                   side is OUTBOARD of the leg plane, on the LEFT it is
                   INBOARD: y = leg centre +/- 16.75 as the table prints it).
    ankle roll     mock_roll_servo_in_foot: case length along y across the
                   foot, horn FORWARD, cable end OUTBOARD; plugs on the AFT
                   (-x) face, routed "aft and up the ankle link's raceway"
                   (dimensions_v6.py:180) -- the raceway channel is not
                   drawn; straight-line minimum.
    neck           Rot(180,0,0)*servo_mock_z at the neck axis
                   (assembly_v6._torso_pieces): case standing, idler face
                   DOWN at the deck (neck_floor's well), cable end AFT; the
                   lead leaves the girdle's neck tube through its aft window
                   (shoulder_girdle_v6.py:296). Anchor: 2 mm below the deck
                   top inside the well -- the neck floor's own lead window
                   (NECK_LEAD_WIN, x -17.35..-10.75) is cut through that
                   plate right under the trench.
    shoulder       shoulder_girdle_v6.shoulder_servo_mock: case fore-aft,
                   cable end AFT, idler faces INBOARD; its wall window is
                   cut OPEN TO THE TOP (shoulder_girdle_v6.py:216-236), so
                   the lead leaves upwards at the trench band, over the pod
                   top, and drops along the housing skin to the GD slot.
    elbow          arm_v6.elbow_servo_mock: case hanging down the forearm,
                   cable end toward the hand; idler faces -y, and with the
                   forearm mirrored on the left arm that puts the plug face
                   at y -(ARM_Y + 16.75) OUTBOARD on the right arm and
                   +(ARM_Y - 16.75) INBOARD on the left. The lead exits the
                   FOREARM's web window (elbow axis -72..-60) and climbs the
                   UPPER arm's open C to ITS web window (axis -96..-78).
                   This hop crosses the SHOULDER fold (the arm below sweeps
                   -90..+200 about the shoulder) AND the ELBOW fold
                   (-100..+10) up the side of the upper arm. NO strain
                   relief is drawn at either (docs/design-v6/arms.md open
                   item 8; docs/design-v6/shoulder-girdle.md kept the
                   windows but added no relief).
* Board end: the GD's servo bus edge is UP under the deck (dims.py:750),
  the leads drop through the board's deck slot; PORT is modelled at the
  PCB's aft face, the two 3-pin bus pads 20 mm either side of centre, at
  the board's top edge. The splitter of servo-map.md §2.1 is NOT SELECTED,
  so board->splitter is SPLITTER_LEAD of allowance, not a CAD number.
* What is NOT drawn (flagged, not invented): no raceway on the yaw carrier
  for the yaw->roll hop, no routed channel in the leg links (the webs carry
  cable WINDOWS only), and no strain relief at the shoulder fold.

Fold loops: pi * (2.0 cable standoff + WIRE_BUNDLE/2) * ROM/180, in their
own column, never silently folded into a span.

Runs:    python cad/v6/lead_lengths.py
Test:    tests/test_lead_lengths.py pins every row against these constants.
"""
import math
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402

STOCK_LEAD = 150.0    # docs/wiring.md "Servo bus" (stock 5264-3P lead)
WORK_SLACK = 40.0     # crimp tails + a service loop on top of span + loop
SPLITTER_LEAD = 30.0  # board port -> splitter stub; splitter part not selected

CONN_MID = sum(D.SV_CONN_L) / 2.0            # 14.05 toward the cable end
PLUG_R = -D.SV_IDLER_CASE_FACE + 2.0         # 16.75 plug-face standoff

DECK_TOP_Z = V.DECK_TOP_Z
HIP = V.HIP_SEP / 2.0                        # 42
_Z_BAND = D.SV_AXIS_FROM_REAR - CONN_MID     # 21.06: plug z, cable end up

# fixed points, assembly frame (x forward, y LEFT, z up)
PORT = {"A": (V.GD_PCB_X1 + 1.0, -20.0, DECK_TOP_Z + V.GD_TOP_Z),
        "B": (V.GD_PCB_X1 + 1.0, +20.0, DECK_TOP_Z + V.GD_TOP_Z)}
LB_SLOT = {"A": (V.CELL_X[0] - 8.0, -HIP, DECK_TOP_Z),
           "B": (V.CELL_X[0] - 8.0, +HIP, DECK_TOP_Z)}
# mouth of the yaw riser: cell-ceiling top = battery-layer floor:
RISER_TOP = {"A": (-CONN_MID, -HIP, DECK_TOP_Z + V.CELL_TOP_Z),
             "B": (-CONN_MID, +HIP, DECK_TOP_Z + V.CELL_TOP_Z)}
# yaw plug: over the trench, facing UP into the riser (ceiling underside +2):
_YAW_PLUG_Z = DECK_TOP_Z + (V.CELL_TOP_Z - D.WALL) + 2.0
YAW_PLUG = {"A": (-CONN_MID, -HIP, _YAW_PLUG_Z),
            "B": (-CONN_MID, +HIP, _YAW_PLUG_Z)}
# neck plug: idler face DOWN into the neck floor's well (face at the deck
# top plane, plug 2 mm proud below it), band mid toward the AFT cable end;
# exits the neck tube's aft window:
NECK_PLUG = (V.NECK_X - CONN_MID, 0.0, DECK_TOP_Z - 2.0)
# shoulder lead's exit: up the open notch over the trench band (aft of the
# axis), at the inboard wall plane, over the pod top:
_POD_TOP = V.ARM_AXIS_Z + D.SV_WID / 2 + D.WALL   # 31.32, girdle's derived number
SHOULDER_EXIT = {"A": (V.ARM_SHOULDER_X - CONN_MID, -V.GIRDLE_WALL_Y0,
                       DECK_TOP_Z + _POD_TOP),
                 "B": (V.ARM_SHOULDER_X - CONN_MID, +V.GIRDLE_WALL_Y0,
                       DECK_TOP_Z + _POD_TOP)}
# elbow lead anchor: the forearm's web window mid, but the CAD-measured hop
# below is what governs. The shoulder->elbow hop is the pair the constants
# model CANNOT answer honestly -- the assembly sweep puts the elbow's plug
# window 158.8 mm under the shoulder's (the two links' own cable windows are
# 30 mm apart THROUGH the joint), so a straight-line figure in any frame is a
# lie about where the lead crosses the fold. The table carries the assembly
# number, measured here, as the span for this hop.
SHOULDER_ELBOW_SPAN = 157.5
ELBOW_WIN = {"A": (V.ARM_SHOULDER_X, -(V.ARM_Y + PLUG_R),
                   V.ARM_ELBOW_Z - 66.0),
             "B": (V.ARM_SHOULDER_X, +V.ARM_Y - PLUG_R,
                   V.ARM_ELBOW_Z - 66.0)}

# joint ROM (assembly degrees) each leg hop crosses:
ROM_LEG = {("hip yaw", "hip roll"): 90.0,        # hip yaw +-45
           ("hip roll", "hip pitch"): 110.0,     # hip roll +-55
           ("hip pitch", "knee"): 210.0,         # hip pitch -120..+90
           ("knee", "ankle"): 225.0,             # knee -95..+130
           ("ankle", "ankle roll"): 80.0}        # ankle pitch +-40
SHOULDER_FOLD = 290.0    # -90..+200 (the arm sweeps it, the lead rides it)
ELBOW_FOLD = 110.0       # -100..+10

NOTE_YAW_ROLL = ("no raceway drawn on the yaw carrier for this hop; "
                 "straight-line minimum down the cell face")
NOTE_PITCH = ("link webs carry a cable WINDOW, not a routed channel; "
              "straight-line plug to plug")
NOTE_ANKLE_ROLL = ("the ankle link's raceway mentioned at dimensions_v6.py"
                   ":180 is not drawn; straight-line into the foot "
                   "bulkhead's cable window")
NOTE_NECK = ("climbs the battery layer into the girdle's neck tube, out the "
             "aft window, up to the well")
NOTE_SH_IN = ("exits UP through the girdle's open notch (shoulder_girdle_v6"
              ".py:216), along the housing skin to the leg-bus slot")
NOTE_ARM = ("span is MEASURED off the assembly mocks (this file, "
            "--assembly): the elbow plug's window is 157.5 mm under the "
            "shoulder's; crosses the shoulder fold (290 deg) and the elbow "
            "fold (110 deg); NO strain relief drawn -- docs/design-v6/"
            "arms.md open item 8")


def _leg(side):
    """Chain-order plug anchors, board to foot. Port A = the robot's RIGHT
    (-y), port B the left. The leg parts are translations: the grip channel's
    plug face is at y = leg-plane - 16.75 on BOTH -- outboard of the right leg
    (-58.75), inboard of the left (+25.25) -- which is what makes the two
    ankle-roll hops differ by 11 mm."""
    s = -1.0 if side == "A" else 1.0
    y_leg = s * HIP
    return [
        ("port", PORT[side]),
        ("deck slot", LB_SLOT[side]),
        ("battery layer", RISER_TOP[side]),
        ("hip yaw", YAW_PLUG[side]),
        ("hip roll", (-PLUG_R, y_leg, V.HIP_ROLL_Z + _Z_BAND)),
        ("hip pitch", (0.0, y_leg - PLUG_R, V.HIP_PITCH_Z + _Z_BAND)),
        ("knee", (0.0, y_leg - PLUG_R, V.KNEE_Z + _Z_BAND)),
        ("ankle", (0.0, y_leg - PLUG_R, V.ANKLE_PITCH_Z + _Z_BAND)),
        ("ankle roll", (-PLUG_R, s * (HIP + CONN_MID), V.ANKLE_ROLL_Z)),
    ]


# NOTE on the two anchors below: the assembly crosscheck (same file,
# --assembly) measures plug-box to plug-box, distance_to -- a MINIMUM over two
# boxes that span the full trench band. The constants spans target the same
# band-mid-to-band-mid figure; they agree to the box's own half-thickness
# (about 2-3 mm), and both are labelled geometric minima. The crosscheck is
# the verification that the placements above (frame transforms) are the ones
# the parts actually build -- run it in the venv.


def _upper(side):
    """Chain-order plug anchors, board to elbow. The neck rides port A's
    branch (servo-map.md §2.1)."""
    pts = [("port", PORT[side]), ("deck slot", LB_SLOT[side])]
    if side == "A":
        pts.append(("neck", NECK_PLUG))
    side_lbl = "R" if side == "A" else "L"
    pts.append((f"{side_lbl} shoulder", SHOULDER_EXIT[side]))
    pts.append((f"{side_lbl} elbow", ELBOW_WIN[side]))
    return pts


def _fold(rom_deg):
    return math.pi * (2.0 + D.WIRE_BUNDLE / 2.0) * rom_deg / 180.0


def rows():
    out = []

    def chain(side, anchors, roms, notes=None):
        for i, ((a, pa), (b, pb)) in enumerate(zip(anchors, anchors[1:])):
            span = math.dist(pa, pb)
            note = (notes or {}).get((a, b))
            if i == 0:
                span += SPLITTER_LEAD
                note = ((note + "; ") if note else "") + \
                    f"includes {SPLITTER_LEAD:.0f} mm board->splitter " \
                    "allowance (splitter not selected)"
            out.append(dict(side=side, a=a, b=b, span=span,
                            loop=_fold(roms[(a, b)]) if (a, b) in roms else 0.0,
                            note=note))

    for side in ("A", "B"):
        chain(side, _leg(side), ROM_LEG,
              notes={("hip yaw", "hip roll"): NOTE_YAW_ROLL,
                     ("hip roll", "hip pitch"): NOTE_PITCH,
                     ("hip pitch", "knee"): NOTE_PITCH,
                     ("knee", "ankle"): NOTE_PITCH,
                     ("ankle", "ankle roll"): NOTE_ANKLE_ROLL})
        up = _upper(side)
        side_lbl = "R" if side == "A" else "L"
        notes = {}
        if side == "A":
            notes[("neck", f"{side_lbl} shoulder")] = NOTE_SH_IN
        else:
            notes[("deck slot", f"{side_lbl} shoulder")] = NOTE_SH_IN
        chain(side, up, {}, notes=notes)
        # the shoulder -> elbow hop rides the arm across both folds; its span
        # is the assembly-measured number, not the constants straight-line
        # (see the note by SHOULDER_ELBOW_SPAN):
        last = out[-1]
        last["span"] = SHOULDER_ELBOW_SPAN
        last["loop"] = _fold(SHOULDER_FOLD) + _fold(ELBOW_FOLD)
        last["note"] = NOTE_ARM
    return out


def assembly_crosscheck(verbose=True):
    """Cross-check the constants table against the BUILT assembly:
    a 2 mm plug box standing 2 mm proud of the idler face over the whole
    trench footprint of the CANONICAL mock, carried to each servo's world
    placement by the SAME mock functions assembly_v6's chain table uses
    (check_assembly.servo_mock_* for the leg, shoulder_girdle_v6's and
    arm_v6's own mocks for the arm); hop span = the two boxes'
    distance_to. This catches a re-derived constant drifting from the
    parts. Needs build123d. Returns a dict keyed like the constants table.
    """
    import shoulder_girdle_v6 as SG
    import arm_v6
    from build123d import Box, Pos, Rot

    def plugbox_in_canonical():
        # canonical servo frame: axis +Y horn +Y, cable end -z, idler face
        # -y: plug box spanning the trench band in z and the case width in x,
        # 2 mm thick, its far face exactly 2.0 proud of the idler case face.
        # Box(align=None) is corner-anchored at the origin, so place by Pos:
        return (Pos(D.SV_CONN_HW * -1 - 2, D.SV_IDLER_CASE_FACE - 2.0,
                    -D.SV_CONN_L[1])
                * Box(D.SV_WID + 4, 2.0, D.SV_CONN_L[1] - D.SV_CONN_L[0],
                      align=None))

    # Per-servo plug boxes, carried to world by the SAME transforms the
    # assembly's chain table uses (the servo_mock_* definitions this mirrors;
    # SG.mirror / arm_v6._mirrored where the mirrored side applies):
    pbox = plugbox_in_canonical
    plugs = {}
    for side, sgn in (("R", -1.0), ("L", +1.0)):
        y = sgn * HIP
        plugs[f"servo_hip_yaw_{side}"] = (
            Pos(0, y, V.HIP_YAW_Z + D.SV_HORN_FACE) * Rot(0, 90, 0)
            * Rot(0, 0, -90) * pbox())
        plugs[f"servo_hip_roll_{side}"] = (
            Pos(0, y, V.HIP_ROLL_Z) * Rot(0, 180, 0) * Rot(0, 0, 90) * pbox())
        plugs[f"servo_hip_pitch_{side}"] = (
            Pos(0, y, V.HIP_PITCH_Z) * pbox())
        plugs[f"servo_knee_{side}"] = Pos(0, y, V.KNEE_Z) * pbox()
        plugs[f"servo_ankle_{side}"] = Pos(0, y, V.ANKLE_PITCH_Z) * pbox()
        ang = 90 if side == "L" else -90
        plugs[f"servo_ankle_roll_{side}"] = (
            Pos(0, y, V.ANKLE_ROLL_Z) * Rot(ang, 0, 0) * Rot(0, 180, 0)
            * Rot(0, 0, 90) * pbox())
        # shoulder_servo_mock: Pos(_AX, ARM_SERVO_MID_Y, _AZ) * Rot(0,90,0),
        # mirrored for R. ARM_SERVO_MID_Y is on +y for L:
        sh = (Pos(0, 0, DECK_TOP_Z)
              * Pos(V.ARM_SHOULDER_X, V.ARM_SERVO_MID_Y, V.ARM_AXIS_Z)
              * Rot(0, 90, 0)) * pbox()
        if side == "R":
            sh = SG.mirror(sh, SG.Plane.XZ)
        plugs[f"servo_shoulder_{side}"] = sh
        el = Pos(V.ARM_SHOULDER_X, V.ARM_Y, V.ARM_ELBOW_Z) * pbox()
        if side == "R":
            el = arm_v6._mirrored(el, "R")
        plugs[f"servo_elbow_{side}"] = el
    # the neck: torso_pieces' own placement (assembly_v6._torso_pieces:
    # Pos(V.NECK_X, 0, z + V.NECK_AXIS_Z) * Rot(180,0,0) * servo_mock_z()):
    plugs["servo_neck_R"] = (Pos(V.NECK_X, 0, DECK_TOP_Z + V.NECK_AXIS_Z)
                             * Rot(180, 0, 0) * Rot(0, 90, 0)
                             * Rot(0, 0, -90) * pbox())
    out = {}
    pairs = [("A", "servo_hip_yaw_R", "servo_hip_roll_R"),
             ("A", "servo_hip_roll_R", "servo_hip_pitch_R"),
             ("A", "servo_hip_pitch_R", "servo_knee_R"),
             ("A", "servo_knee_R", "servo_ankle_R"),
             ("A", "servo_ankle_R", "servo_ankle_roll_R"),
             ("A", "servo_neck_R", "servo_shoulder_R"),
             ("A", "servo_shoulder_R", "servo_elbow_R"),
             ("B", "servo_hip_yaw_L", "servo_hip_roll_L"),
             ("B", "servo_hip_roll_L", "servo_hip_pitch_L"),
             ("B", "servo_hip_pitch_L", "servo_knee_L"),
             ("B", "servo_knee_L", "servo_ankle_L"),
             ("B", "servo_ankle_L", "servo_ankle_roll_L"),
             ("B", "servo_shoulder_L", "servo_elbow_L")]
    out = {}
    for a, na, nb in pairs:
        out[(na, nb)] = plugs[na].distance_to(plugs[nb])
        if verbose:
            print(f"  assembly: {na} -> {nb}: {out[(na, nb)]:.1f} mm")
    return out


def render():
    print("# per-hop lead lengths, mm -- v6 CAD, ARMS=1, standing pose")
    print(f"# deck top z {DECK_TOP_Z:.2f}; stock lead {STOCK_LEAD:.0f} mm; "
          f"verdict vs span+loop <= {STOCK_LEAD - 20:.0f} (20 mm tail)")
    table = []
    for r in rows():
        need = r["span"] + r["loop"]
        buy = math.ceil((need + WORK_SLACK) / 10.0) * 10
        fits = "150 ok" if need <= STOCK_LEAD - 20 else "LONGER"
        hop = f"{r['a']} -> {r['b']}"
        table.append((r["side"], hop, round(r["span"], 1), round(r["loop"], 1),
                      round(need, 1), int(buy), fits, r["note"] or ""))
        line = (f"{r['side']:>2}  {hop:<30} span {r['span']:6.1f}  "
                f"loop {r['loop']:5.1f}  need {need:6.1f}  "
                f"buy ~{buy:4.0f}  {fits}")
        if r["note"]:
            line += f"\n     # {r['note']}"
        print(line)
    return table


if __name__ == "__main__":
    render()
    if "--assembly" in sys.argv:
        assembly_crosscheck()
