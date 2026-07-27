"""Dressed-assembly builder: printed parts + servos PLUS the approximate
non-printed dress — driver board, battery, servo daisy-chain cables, zip
ties, XT30 pigtail — with the legs POSABLE (hip roll/pitch, knee, ankle).

This is a *mock* layer for visualization and routing sanity, not source of
truth: wire paths are plausible splines between the real connector
positions (ST3215 ports are on the case end OPPOSITE the output), not
catalog-exact leads. Cable segments that cross a joint are re-swept from
the posed attachment points, so a posed export shows realistic slack/wrap
instead of a rigid cable tearing off.

Chain per leg (all legs identical, horn +Y):
  board (under the tower top, face-down)
    -> hip-roll servo plug (top end, in the pelvis deck cutout)      [rigid]
    -> over the deck rear edge, down to the hip-pitch servo's rear-  [joint]
       end ports (z -35.11 below the hip axis)
    -> thigh web raceway (zip ties through the web holes) -> knee    [joint]
       servo rear ports
    -> shin web raceway (ties) -> ankle servo rear end at the heel   [joint]
       (through the foot's bulkhead cable window)
Battery XT30 pigtail rises from the pack's +y end face to the board edge.
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
PCB_TOP = DTOP + D.TOWER_H - D.TOWER_TOP_T - D.BOARD_STANDOFF
PCB_T = 1.6

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
    """Waveshare ESP32 servo driver, face-DOWN on standoffs under the tower
    top: PCB slab + module can + two servo-header banks + XT30 stub."""
    p = parts.box(-15, 15, -32.5, 32.5, PCB_TOP - PCB_T, PCB_TOP)
    return p


def board_components():
    zc = PCB_TOP - PCB_T
    p = parts.box(-9, 9, 8, 26, zc - 3.4, zc)          # ESP32 module can
    p += parts.box(-13, -7, -28, -2, zc - 5.5, zc)     # servo header bank
    p += parts.box(7, 13, -28, -2, zc - 5.5, zc)       # servo header bank
    p += parts.box(-4, 4, 27, 32.5, zc - 6.5, zc)      # XT30 power entry
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
    """Board edge -> down through the deck cutout -> roll servo plug (rigid:
    every point is torso-fixed)."""
    sgn = 1 if ly > 0 else -1
    return cable([(0, sgn * 30, PCB_TOP - PCB_T - 4),
                  (-5, ly, DTOP + 16),
                  (-7, ly, DTOP + 2),
                  (-7, ly, D.DECK_BOT_Z + 1.5)])


def pigtail():
    """Battery XT30 lead: pack +y end face up to the board power entry."""
    return cable([(0, 29, DTOP + 12), (-2, 37, DTOP + 24),
                  (0, 31, PCB_TOP - PCB_T - 5), (0, 27, PCB_TOP - PCB_T - 5)],
                 r=2.0)


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
        A.piece("tower", A.COL_PRINT, Pos(0, 0, DTOP) * parts.tower()),
        A.piece("battery_3s_mock", A.COL_BATT,
                Pos(0, 0, DTOP) * A.battery_mock()),
        A.piece("board_pcb_mock", COL_PCB, board_mock()),
        A.piece("board_parts_mock", COL_CHIP, board_components()),
        A.piece("pigtail_xt30", COL_WIRE, pigtail()),
        A.piece("cable_board_L", COL_WIRE, torso_cable(D.HIP_SEP / 2)),
        A.piece("cable_board_R", COL_WIRE, torso_cable(-D.HIP_SEP / 2)),
        A.piece("imu_carrier", A.COL_PRINT,
                Pos(0, 0, A.TOWER_TOP_Z) * parts.imu_carrier()),
        A.piece("imu_bno055_mock", COL_PCB,
                Pos(0, D.IMU_CY,
                    A.TOWER_TOP_Z + D.IMU_PCB_Z)
                * parts.box(-D.IMU_PCB[0] / 2, D.IMU_PCB[0] / 2,
                            -D.IMU_PCB[1] / 2, D.IMU_PCB[1] / 2,
                            0, D.IMU_PCB[2])),
        A.piece("gopro_base", A.COL_PRINT,
                Pos(0, 0, A.TOWER_TOP_Z + D.IMU_CARRIER_T)
                * parts.gopro_base()),
        A.piece("camera_gopro_max_mock", A.COL_CAM,
                Pos(0, 0, A.TOWER_TOP_Z + D.IMU_CARRIER_T)
                * A.camera_mock()),
        A.piece("screws_deck", A.COL_STEEL,
                Pos(0, 0, DTOP) * F.deck_stator_screws()),
        A.piece("screws_tower", A.COL_STEEL,
                Pos(0, 0, DTOP) * F.tower_screws()),
        A.piece("screws_head_stack", A.COL_STEEL,
                Pos(0, 0, A.TOWER_TOP_Z) * F.head_stack_screws()),
        dressed_leg(D.HIP_SEP / 2, "L", roll, hip, knee, ankle),
        dressed_leg(-D.HIP_SEP / 2, "R", roll, hip, knee, ankle),
    ])
