"""Dressed-assembly builder: printed parts + servos PLUS the approximate
non-printed dress — driver board, battery, servo daisy-chain cables, zip
ties, XT30 pigtail — with the legs POSABLE (hip roll/pitch, knee, ankle).

This is a *mock* layer for visualization and routing sanity, not source of
truth: wire paths are plausible splines between the real connector
positions (ST3215 ports are on the case end OPPOSITE the output), not
catalog-exact leads. Cable segments that cross a joint are re-swept from
the posed attachment points, so a posed export shows realistic slack/wrap
instead of a rigid cable tearing off.

TORSO IS v5 (2026-08-04): one yaw housing, and battery + board both dropped
into the servos' own z band. The pack hangs UNDER the deck in `battery_tray`;
the board hangs BEHIND the housing on `board_frame`, still upright/transverse
with its CONNECTORS FACING AFT. So the torso leads no longer climb anywhere --
they run along the deck TOP, which is now the roof of the robot, and drop into
the frame's wire chases.

Chain per leg (all legs identical, horn +Y):
  board leg-servo port (aft face of the board, z ~ -10)
    -> outboard chase, up past the board's edge, through the bulkhead's
       OUTBOARD wire slot (BF_WIRE_SLOT_*, past its BF_TIE_POST anchor)
    -> forward along the deck-top corridor -- the y band between the two
       stator screws' driver cylinders, which is what sized it
    -> yaw servo's connector, standing up out of the deck TRENCH    [rigid]
  and from the trench's second socket:
    -> inboard across the deck, down the centre wire window into the
       housing's middle cell, out its open bottom
    -> hip-roll servo plug (top end, under the housing)             [rigid]
    -> over the deck rear edge, down to the hip-pitch servo's rear-  [joint]
       end ports (z -35.11 below the hip axis)
    -> thigh web raceway (zip ties through the web holes) -> knee    [joint]
       servo rear ports
    -> shin web raceway (ties) -> ankle servo rear end at the heel   [joint]
       (through the foot's bulkhead cable window)
Battery lead rises from the pack's +y end through the 3.5 mm gap the real
62 mm pack leaves beside the tray wall, up through the deck APERTURE, and aft
along the +y corridor to the board's power inlet.
Not placed (undesigned mounts): BNO085 IMU, power switch.

Used by export_assembly_full.py (STEP + stills) and animate_dressed_rom.py
(leg-motion video).
"""
import numpy as np
from build123d import (Pos, Rot, Compound, Color, Circle, Cylinder, Plane,
                       Sphere, Spline, sweep)

import dimensions as D
import parts
import fasteners as F
import check_assembly as CA
import export_assembly as A

DTOP = A.DECK_TOP_Z                              # v3yaw deck top, world z
# v5 board: upright and transverse on board_frame, component/connector face
# AFT, hanging in the servo band. World x of the two faces and the z band it
# occupies -- all of it now at or BELOW the deck except the top 24 mm.
PCB_AFT_X = D.BF_PCB_X1                          # -46.24, where the ports are
PCB_FWD_X = D.BF_PCB_X0                          # -44.61, on the standoffs
PCB_Z0 = DTOP + D.BF_BOT_Z                       # board bottom edge (-41)
PCB_Z1 = DTOP + D.BF_TOP_Z                       # board top edge   (+24)
# Port positions on the aft face. The board's own connector coordinates are not
# in the repo (docs/sensor-expansion.md §1 inventories the REFS -- H1 power in,
# H5/H6 one bus-servo port per leg -- but no positions), so these are placed
# plausibly: the two leg ports low and inboard where the chases deliver, the
# power inlet up on the +y side where the battery lead arrives. Nothing
# downstream measures them; the ROUTING is what this file is for.
PORT_LEG_Z = DTOP + D.BF_CZ - 1.5                # H5 / H6, z ~ -10
PORT_LEG_Y = 10.0
PORT_PWR = (DTOP + 8.0, 26.0)                    # H1 XH power inlet (z, y)
# The two wire routes the frame provides, as y coordinates. Each bulkhead slot
# is split by its BF_TIE_POST into an INBOARD opening (behind the board, giving
# onto the 4 mm standoff gap -- too thin for a bundle) and an OUTBOARD one that
# gives straight onto the chase between the board's edge and the side wall.
# Everything in this file uses the outboard opening.
SLOT_Y = (D.BF_WIRE_SLOT_Y[1] + D.BOARD_GD_OUTLINE[0] / 2 + D.BF_TIE_POST / 2) / 2
CHASE_Y = (D.BOARD_GD_OUTLINE[0] / 2 + D.BF_WALL_Y_IN) / 2       # 37.25
# The deck-top corridor: the y band between the two stator screws' ACCESS_D
# driver cylinders. parts.pelvis names it, check_assembly measures it, and this
# is the file that has to stay inside it.
CORR_Y = D.HIP_SEP / 2 - D.CASE_HOLE_LAT + D.ACCESS_D / 2 + 2.5   # ~29.3

COL_WIRE = Color(0.55, 0.12, 0.10)
COL_TIE = Color(0.08, 0.08, 0.09)
COL_PCB = Color(0.05, 0.32, 0.18)
COL_CHIP = Color(0.15, 0.15, 0.17)

RACE_X = -17.0                 # cable centerline down a web (face -15.16 - r)
TIE_Z = (-41.0, -52.0)         # tie rings at the web zip-tie holes


# ------------------------------------------------------------- kinematics
def _rx(a, c):
    t = np.eye(4); r = np.radians(a)
    t[1:3, 1:3] = [[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]]
    out = np.eye(4); out[:3, 3] = c
    back = np.eye(4); back[:3, 3] = -np.asarray(c, float)
    return out @ t @ back


def _ry(a, c):
    t = np.eye(4); r = np.radians(a)
    t[0, 0] = t[2, 2] = np.cos(r); t[0, 2] = np.sin(r); t[2, 0] = -np.sin(r)
    out = np.eye(4); out[:3, 3] = c
    back = np.eye(4); back[:3, 3] = -np.asarray(c, float)
    return out @ t @ back


def leg_frames(ly, roll, hip, knee, ankle):
    """World 4x4 for each moving group of one leg at (roll, hip, knee, ankle)
    degrees. Joint sign conventions match the sim (flexion negative)."""
    m_yoke = _rx(roll, (0, ly, D.HIP_ROLL_Z))
    m_thigh = m_yoke @ _ry(hip, (0, ly, D.HIP_PITCH_Z))
    m_shin = m_thigh @ _ry(knee, (0, ly, D.KNEE_Z))
    m_foot = m_shin @ _ry(ankle, (0, ly, D.ANKLE_Z))
    return {"yoke": m_yoke, "thigh": m_thigh, "shin": m_shin, "foot": m_foot}


def leg_locations(ly, roll, hip, knee, ankle):
    """Same frames as leg_frames but as build123d Locations."""
    lr = Pos(0, ly, D.HIP_ROLL_Z) * Rot(roll, 0, 0) * Pos(0, -ly, -D.HIP_ROLL_Z)
    lh = lr * Pos(0, ly, D.HIP_PITCH_Z) * Rot(0, hip, 0) * Pos(0, -ly, -D.HIP_PITCH_Z)
    lk = lh * Pos(0, ly, D.KNEE_Z) * Rot(0, knee, 0) * Pos(0, -ly, -D.KNEE_Z)
    la = lk * Pos(0, ly, D.ANKLE_Z) * Rot(0, ankle, 0) * Pos(0, -ly, -D.ANKLE_Z)
    return {"yoke": lr, "thigh": lh, "shin": lk, "foot": la}


def pt(m, p):
    return (m @ np.array([p[0], p[1], p[2], 1.0]))[:3]


# ------------------------------------------------------------------ mocks
def cable(pts, r=1.6, chain=False):
    """Round mock lead through world points: spline sweep, falling back to a
    ball-jointed cylinder chain when the sweep degenerates (OCC sweeps can
    fail outright OR tessellate absurdly on some mid-pose splines -- the
    animation hits hundreds of poses, so callers that must have a clean
    mesh can force chain=True; see animate_dressed_rom's STL validation)."""
    if not chain:
        try:
            path = Spline(*[tuple(map(float, p)) for p in pts])
            sect = Plane(origin=path @ 0, z_dir=path % 0) * Circle(r)
            s = sweep(sect, path)
            if s.volume > 1.0:
                return s
        except Exception:
            pass
    # one Chaikin pass softens the polyline corners, then cylinders+spheres
    p = [np.asarray(q, float) for q in pts]
    sm = [p[0]]
    for a, b in zip(p[:-1], p[1:]):
        sm += [a * 0.75 + b * 0.25, a * 0.25 + b * 0.75]
    sm.append(p[-1])
    out = None
    for a, b in zip(sm[:-1], sm[1:]):
        d = b - a
        n = np.linalg.norm(d)
        if n < 1e-6:
            continue
        seg = Plane(origin=tuple((a + b) / 2), z_dir=tuple(d / n)) \
            * Cylinder(r, n)
        seg += Pos(*b) * Sphere(r)
        out = seg if out is None else out + seg
    return Pos(*sm[0]) * Sphere(r) + out


def zip_tie(z):
    """One tie wrapping web + cable at web-local z (leg local frame): band
    legs pass ~through the web holes (y +/-7), back run clears the cable,
    front run squeezes the web/case gap. Head block on the cable side."""
    ring = parts.box(-19.9, -12.45, -8.4, 8.4, z - 1.25, z + 1.25)
    ring -= parts.box(-18.9, -12.70, -7.1, 7.1, z - 2, z + 2)
    ring += parts.box(-22.3, -19.4, -2.0, 2.0, z - 1.6, z + 1.6)   # head
    return ring


def board_mock():
    """The General Driver PCB where board_frame holds it -- upright, transverse
    and 65 x 65. Same solid export_assembly places, lifted to world."""
    return Pos(0, 0, DTOP) * A.board_pcb_mock()


def board_components():
    """What is fitted to the board's AFT face. The interference check uses the
    full BOARD_GD_COMP slab (conservative); here it is broken into the pieces
    the harness plugs into, so the render reads as a board rather than a brick.
    Depths are within that same 9 mm envelope."""
    x1 = PCB_AFT_X                                      # component face
    p = parts.box(x1 - 3.4, x1, -9, 9, PCB_Z1 - 28, PCB_Z1 - 10)  # ESP32 can
    p += parts.box(x1 - 9.0, x1, -30, 30, PCB_Z1 - 10, PCB_Z1 - 4)   # 40-pin
    for sy in (PORT_LEG_Y, -PORT_LEG_Y):                # H5 / H6 servo ports
        p += parts.box(x1 - 5.5, x1, sy - 3, sy + 3,
                       PORT_LEG_Z - 4, PORT_LEG_Z + 4)
    p += parts.box(x1 - 6.5, x1, PORT_PWR[1] - 4, PORT_PWR[1] + 4,
                   PORT_PWR[0] - 4, PORT_PWR[0] + 4)    # H1 XT30/XH inlet
    return p


def leg_cables(ly, roll, hip, knee, ankle, chain=False):
    """The three per-leg cable segments, re-swept for the given pose."""
    f = leg_frames(ly, roll, hip, knee, ankle)
    ZP, ZK = D.HIP_PITCH_Z, D.KNEE_Z

    def loc(name, p):        # leg-local (about that body's joint z) -> world
        base = {"thigh": ZP, "shin": ZK, "foot": D.ANKLE_Z}[name]
        return pt(f[name], (p[0], p[1] + ly, p[2] + base))

    # Every crossing between a rear port (inboard, x -5/-11) and the outer
    # raceway (x -17) threads the web's 9 x 11 CABLE WINDOW at (y 0,
    # z -48..-37) -- before the window existed the cables pierced the web
    # (27-74 mm3 measured per segment, print-review feedback 2026-07-16).
    def win_in(name):                    # outside face -> window -> IN port
        return [loc(name, (-17.5, 0, -17)), loc(name, (-17.0, 0, -27)),
                loc(name, (-16.8, 0, -36)), loc(name, (-13.5, 0, -42.5)),
                loc(name, (-6.5, 0, -41)), loc(name, (-5, 0, -36.8))]

    # B: deck cutout plug -> over the deck rear edge -> down the web outer
    # face -> through the thigh window -> hip-pitch IN port
    b0 = np.array([-9, ly, D.DECK_BOT_Z + 1.5])
    b1 = np.array([-9, ly, DTOP + 3])
    b2 = np.array([-26, ly, DTOP + 0.5])
    b3 = np.array([-28, ly, DTOP - 14])
    def outn(name):                      # world direction of the frame's -x
        n = -f[name][:3, 0]
        return n / np.linalg.norm(n)

    bw = win_in("thigh")
    # dynamic slack: bow along the bisector of world -x and the thigh's
    # posed outward normal (a fixed world -x cut into the link at hip -110)
    nb = outn("thigh") + np.array([-1.0, 0, 0])
    b4 = (b3 + bw[0]) / 2 + 8 * nb / np.linalg.norm(nb)
    seg_b = cable([b0, b1, b2, b3, b4] + bw, chain=chain)

    # C: hip-pitch OUT port -> out through the thigh window -> raceway ->
    # in through the SHIN window -> knee IN port
    c0 = loc("thigh", (-11, 0, -36.6))
    c1 = loc("thigh", (-13.5, 0, -42.5))
    tth = [pt(f["thigh"], (RACE_X, ly, ZP + z)) for z in (-46, TIE_Z[1], -57)]
    # ride the thigh's outer face all the way to the knee corner: at knee
    # -95 the free span otherwise cuts through the thigh's fork end
    tth += [pt(f["thigh"], (RACE_X, ly, ZP + z)) for z in (-74, -88)]
    cw = win_in("shin")
    # slack bows along the knee's outward bisector so a folded knee
    # (-95) doesn't pull the span through either link
    nc = outn("thigh") + outn("shin")
    mid = (tth[-1] + cw[0]) / 2 + 7 * nc / np.linalg.norm(nc)
    seg_c = cable([c0, c1] + tth + [mid] + cw, chain=chain)

    # D: knee OUT port -> out through the shin window -> shin raceway ->
    # ankle rear end (heel window)
    d0 = loc("shin", (-11, 0, -36.6))
    d1 = loc("shin", (-13.5, 0, -42.5))
    tsh = [pt(f["shin"], (RACE_X, ly, ZK + z)) for z in (-46, TIE_Z[1], -56)]
    d2 = loc("foot", (-30, 0, 13))                   # dive toward the heel
    d3 = loc("foot", (-36.3, 0, 1))                  # plug at the rear end
    mid = (tsh[-1] + d2) / 2 + np.array([-3, 0, 2])
    seg_d = cable([d0, d1] + tsh + [mid, d2, d3], chain=chain)
    return seg_b, seg_c, seg_d


def torso_cable(ly):
    """One leg's bus lead from the BOARD to the yaw servo's connector, standing
    up out of the deck trench. Rigid: every point is torso-fixed.

    v5 route, and the frame's own features dictate all of it:

      * it starts at a leg-servo port on the board's AFT face, low and inboard
        (PORT_LEG_*), and runs outboard across that face,
      * turns forward into the CHASE -- the BF_CHASE-wide slot between the
        board's edge and the frame's side wall, which exists for this,
      * climbs the chase and passes through the OUTBOARD half of the bulkhead's
        wire slot (the inboard half opens onto the 4 mm standoff gap, too thin
        for a bundle), leaving its BF_TIE_POST anchor on the inboard side --
        that post is the strain relief the bundle zip-ties to,
      * runs forward along the deck-top CORRIDOR, the y band between the two
        stator screws' driver cylinders (check_assembly measures that band),
      * and drops into the yaw connector trench.
    """
    sgn = 1 if ly > 0 else -1
    zs = DTOP + sum(D.BF_WIRE_SLOT_Z) / 2               # slot mid-height
    tx = -(D.SV_CONN_L[0] + D.SV_CONN_L[1]) / 2         # trench centre, x -14.05
    return cable([(PCB_AFT_X - 3.0, sgn * PORT_LEG_Y, PORT_LEG_Z),
                  (PCB_AFT_X - 3.5, sgn * 24.0, PORT_LEG_Z + 1),
                  # out past the board's edge into the chase, then forward
                  (PCB_AFT_X - 2.0, sgn * (CHASE_Y - 2), PORT_LEG_Z + 4),
                  (PCB_FWD_X + 1.0, sgn * (CHASE_Y - 2), DTOP - 4.0),
                  # up the chase and through the outboard slot opening, low --
                  # the battery lead shares this slot and takes the top of it
                  (D.BF_BULK_X1 - 1.5, sgn * SLOT_Y, zs - 2.5),
                  (D.BF_BULK_X + 1.5, sgn * SLOT_Y, zs - 2.5),
                  # forward along the deck-top corridor to the trench, in the
                  # INBOARD lane (the battery lead runs the outboard one)
                  (-30.0, sgn * CORR_Y, DTOP + 4.0),
                  (-20.0, sgn * CORR_Y, DTOP + 4.0),
                  (tx - 1.0, sgn * (CORR_Y + 1.0), DTOP + 3.0),
                  (tx, ly, DTOP + 0.5)])


def yaw_riser(ly):
    """The second half of the same bus: yaw servo's OTHER trench socket down to
    the hip-roll servo's plug. Also rigid (the carrier is drawn at neutral yaw).

    It crosses INBOARD over the deck -- passing over the stator screws, which is
    free because they are flat heads sunk flush in the deck top -- drops through
    the centre wire window, falls down the housing's middle cell (the channel
    the one-housing rewrite left open between the two servo cells) and out its
    open bottom to the roll servo waiting below."""
    sgn = 1 if ly > 0 else -1
    tx = -(D.SV_CONN_L[0] + D.SV_CONN_L[1]) / 2
    return cable([(tx + 2.0, ly - sgn * 4.0, DTOP + 0.5),
                  (tx + 1.0, ly - sgn * 12.0, DTOP + 4.0),
                  (-8.0, sgn * 9.0, DTOP + 4.0),
                  (-6.5, sgn * 8.0, DTOP - 12.0),      # down the deck window
                  (-7.0, sgn * 9.0, DTOP - 38.0),      # ...and the channel
                  (-7.0, ly, D.DECK_BOT_Z + 1.5)])     # roll servo plug


def pigtail():
    """Battery lead: the pack's +y end up through the deck aperture and aft to
    the board's power inlet (H1).

    The pack is UNDERSLUNG in v5, so this lead starts below the deck. It comes
    off the pack's +y end face and rises through the gap the real 62 mm pack
    leaves beside the tray's 69 mm-wide bay -- 3.5 mm, which dimensions.py calls
    out as exactly where the lead comes up -- then straight up through the deck
    APERTURE (the same hole the pack itself lifts out through), aft along the
    +y corridor and down the +y chase to the inlet. r=1.5: 3.5 mm of gap is what
    it has to pass, so a fatter mock would be drawing a lead that does not fit.
    """
    zs = DTOP + sum(D.BF_WIRE_SLOT_Z) / 2
    ygap = (D.BATT_PACK[0] / 2 + D.BT_WALL_Y_IN) / 2    # 32.75, mid of the gap
    return cable([(D.BT_SEAT_X + 12.0, ygap, DTOP + D.BT_FLOOR_TOP + 12.0),
                  (D.BT_SEAT_X + 10.0, ygap, DTOP - 2.0),
                  (D.BT_SEAT_X + 6.0, ygap, DTOP + 4.0),   # out of the aperture
                  # OUTBOARD lane of the corridor: the leg lead has the
                  # inboard one, and 7 mm of lane separation is what keeps the
                  # two mocks off each other through the shared slot
                  (-10.0, CORR_Y + 7.0, DTOP + 5.0),
                  (-30.0, CORR_Y + 7.0, DTOP + 5.0),
                  (D.BF_BULK_X + 1.5, SLOT_Y, zs + 4.0),
                  (D.BF_BULK_X1 - 1.5, SLOT_Y, zs + 4.0),  # outboard slot, high
                  (PCB_FWD_X + 1.0, CHASE_Y + 2.0, DTOP + 6.0),
                  (PCB_AFT_X - 3.0, CHASE_Y, PORT_PWR[0] + 2.0),
                  (PCB_AFT_X - 3.5, PORT_PWR[1], PORT_PWR[0])],
                 r=1.5)


# --------------------------------------------------------------- assembly
def dressed_leg(ly, tag, roll=0.0, hip=0.0, knee=0.0, ankle=0.0):
    L = leg_locations(ly, roll, hip, knee, ankle)
    at = lambda z: Pos(0, ly, z)
    segs = leg_cables(ly, roll, hip, knee, ankle)
    ch = [
        # v3yaw hip-yaw stack (torso-fixed at neutral yaw): flat yaw servo under
        # the deck + carrier on its horn, holding the roll bay. Not articulated
        # here (dress poses roll/hip/knee/ankle only), so both sit static.
        A.piece("servo_hip_yaw", A.COL_SERVO,
                Pos(0, ly, D.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()),
        A.piece("yaw_carrier", A.COL_PRINT,
                Pos(0, ly, D.HIP_YAW_Z) * parts.yaw_carrier()),
        A.piece("servo_hip_roll", A.COL_SERVO,
                at(D.HIP_ROLL_Z) * CA.servo_mock_x()),
        A.piece("yoke_roll", A.COL_PRINT,
                L["yoke"] * at(D.HIP_ROLL_Z) * parts.yoke_roll()),
        A.piece("yoke_pitch", A.COL_PRINT,
                L["yoke"] * at(D.HIP_PITCH_Z) * parts.yoke_pitch()),
        A.piece("servo_hip_pitch", A.COL_SERVO,
                L["thigh"] * at(D.HIP_PITCH_Z) * CA.servo_mock_y()),
        A.piece("link_thigh", A.COL_PRINT,
                L["thigh"] * at(D.HIP_PITCH_Z) * parts.leg_link()),
        A.piece("servo_knee", A.COL_SERVO,
                L["shin"] * at(D.KNEE_Z) * CA.servo_mock_y()),
        A.piece("link_shin", A.COL_PRINT,
                L["shin"] * at(D.KNEE_Z) * parts.leg_link()),
        A.piece("servo_ankle", A.COL_SERVO,
                L["foot"] * at(D.ANKLE_Z) * Rot(0, 90, 0) * CA.servo_mock_y()),
        A.piece("foot", A.COL_FOOT, L["foot"] * at(D.TPU_PROUD) * parts.foot()),
        # fasteners ride the same pose transforms as their host parts
        A.piece("screws_yaw_stack", A.COL_STEEL,
                Pos(0, ly, D.HIP_YAW_Z) * F.yaw_carrier_screws()),
        A.piece("screws_hip_roll", A.COL_STEEL,
                L["yoke"] * at(D.HIP_ROLL_Z) * F.disc_screws_x()),
        A.piece("screws_flange", A.COL_STEEL,
                L["yoke"] * at(D.HIP_ROLL_Z) * F.flange_bolts()),
        A.piece("screws_hip_pitch", A.COL_STEEL,
                L["yoke"] * at(D.HIP_PITCH_Z) * F.disc_screws_y()),
        A.piece("screws_thigh_grip", A.COL_STEEL,
                L["thigh"] * at(D.HIP_PITCH_Z) * F.leg_link_screws()),
        A.piece("screws_knee", A.COL_STEEL,
                L["thigh"] * at(D.KNEE_Z) * F.disc_screws_y()),
        A.piece("screws_shin_grip", A.COL_STEEL,
                L["shin"] * at(D.KNEE_Z) * F.leg_link_screws()),
        A.piece("screws_ankle", A.COL_STEEL,
                L["shin"] * at(D.ANKLE_Z) * F.disc_screws_y()),
        A.piece("screws_foot", A.COL_STEEL,
                L["foot"] * at(D.TPU_PROUD) * F.foot_screws()),
        A.piece("cable_hip", COL_WIRE, segs[0]),
        A.piece("cable_thigh", COL_WIRE, segs[1]),
        A.piece("cable_shin", COL_WIRE, segs[2]),
        A.piece("ties_thigh", COL_TIE,
                L["thigh"] * at(D.HIP_PITCH_Z) * (zip_tie(TIE_Z[0])
                                                  + zip_tie(TIE_Z[1]))),
        A.piece("ties_shin", COL_TIE,
                L["shin"] * at(D.KNEE_Z) * (zip_tie(TIE_Z[0])
                                            + zip_tie(TIE_Z[1]))),
    ]
    return Compound(label=f"leg_{tag}", children=ch)


def dressed_robot(roll=0.0, hip=0.0, knee=0.0, ankle=0.0):
    """Full robot with dress mocks; all four leg joints posable (deg), both
    legs posed identically (same-sign roll, like the ROM video)."""
    return Compound(label="bimo_biped_dressed", children=[
        A.piece("pelvis", A.COL_PRINT, Pos(0, 0, DTOP) * parts.pelvis()),
        # v5 torso: tray under the deck, frame on the housing's rear wall. The
        # camera and IMU are NOT drawn -- neither has a mount on the robot until
        # the head bolt-on exists, and drawing them on a retired tower is how
        # the renders went on quietly showing a machine that no longer existed.
        A.piece("battery_tray", A.COL_PRINT,
                Pos(0, 0, DTOP) * parts.battery_tray()),
        A.piece("board_frame", A.COL_PRINT,
                Pos(0, 0, DTOP) * parts.board_frame()),
        A.piece("battery_3s_mock", A.COL_BATT,
                Pos(0, 0, DTOP) * A.battery_mock()),
        A.piece("board_pcb_mock", COL_PCB, board_mock()),
        A.piece("board_parts_mock", COL_CHIP, board_components()),
        A.piece("pigtail_xt30", COL_WIRE, pigtail()),
        A.piece("cable_board_L", COL_WIRE, torso_cable(D.HIP_SEP / 2)),
        A.piece("cable_board_R", COL_WIRE, torso_cable(-D.HIP_SEP / 2)),
        A.piece("cable_yaw_L", COL_WIRE, yaw_riser(D.HIP_SEP / 2)),
        A.piece("cable_yaw_R", COL_WIRE, yaw_riser(-D.HIP_SEP / 2)),
        A.piece("screws_deck", A.COL_STEEL,
                Pos(0, 0, DTOP) * F.deck_stator_screws()),
        A.piece("screws_tray", A.COL_STEEL,
                Pos(0, 0, DTOP) * F.battery_tray_screws()),
        A.piece("screws_board_frame", A.COL_STEEL,
                Pos(0, 0, DTOP) * F.board_frame_screws()),
        dressed_leg(D.HIP_SEP / 2, "L", roll, hip, knee, ankle),
        dressed_leg(-D.HIP_SEP / 2, "R", roll, hip, knee, ankle),
    ])
