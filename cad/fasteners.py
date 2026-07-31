"""Every fastener in the robot as a simple solid (shank + head [+ washer]),
plus -- since 2026-07-30 -- the INSERTION SWEEP behind each head: the volume
the screw and its driver have to travel through to REACH that seat.
grouped in the SAME local frames as the printed parts they ride with. Single
source of truth for BOTH check_assembly's head-clearance audit and the
assembled STEPs (export_assembly / dress) -- a screw that moves in one moves
in both, and a screw that exists only as a symbol in the docs can no longer
hide from the interference checks (that blindness caused the bench link-skew:
proud grip-screw pan heads rode the yoke/fork arms from hip +/-60 deg on).

Simplifications: threads are plain cylinders, Phillips recesses are omitted,
pan/button heads are cylinders, flat heads are the 90-deg cones that sit
flush in their countersinks. Lengths follow docs/assembly.md.
"""
import os

from build123d import Pos, Rot, Cone, Sphere, export_step
import dimensions as D
import parts


def _fuse(solids):
    p = None
    for s in solids:
        p = s if p is None else p + s
    return p


def _flat_head_y(x, z, y_face, sign):
    """M2.5 flat head as a 90-deg cone, flush at y_face, tapering inward
    (away from sign*Y) to the shank diameter."""
    h = (D.CASE_FLAT_D - 2.5) / 2                     # 1.1 cone height
    rot = Rot(-90, 0, 0) if sign > 0 else Rot(90, 0, 0)
    mid = y_face - sign * h / 2
    return Pos(x, mid, z) * rot * Cone(2.5 / 2, D.CASE_FLAT_D / 2, h)


# --------------------------------------------------------------- leg links
def _flat_head_x(y, z, x_face, sign):
    """_flat_head_y's twin for screws along X (carrier bay walls)."""
    h = (D.CASE_FLAT_D - 2.5) / 2
    rot = Rot(0, 90, 0) if sign > 0 else Rot(0, -90, 0)
    mid = x_face - sign * h / 2
    return Pos(mid, y, z) * rot * Cone(2.5 / 2, D.CASE_FLAT_D / 2, h)


def _flat_head_z(x, y, z_face, sign):
    """_flat_head_y's twin for screws along Z (pelvis deck stator screws)."""
    h = (D.CASE_FLAT_D - 2.5) / 2
    rot = Rot(0, 0, 0) if sign > 0 else Rot(180, 0, 0)
    mid = z_face - sign * h / 2
    return Pos(x, y, mid) * rot * Cone(2.5 / 2, D.CASE_FLAT_D / 2, h)


def leg_link_screws():
    """6x M2.5x8 FLAT-head self-tap gripping the servo case (leg_link frame:
    upper joint axis == Y at origin). Heads sit FLUSH in the grip-plate
    countersinks -- see the leg_link comment / bench skew of 2026-07-28.

    All heads are FLAT and flush -- the pan-head variant was retired 2026-07-28
    when the build standardised on M2.5 flat-head self-tappers."""
    s = []
    seat = D.SV_TOPFACE + D.GRIP_PLATE_T              # 19.75 horn plate outer
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append(parts.cyl_y(1.25, seat - 8.0, seat - 0.9, lx, -zrow))
            s.append(_flat_head_y(lx, -zrow, seat, +1))
    # idler grip plate outer face: GRIP_PLATE_T_IDLER off the seat (-17.90),
    # pulled in 2026-07-30 so the same M2.5x8 works here too (was -21 / x10)
    igo = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append(parts.cyl_y(1.25, igo + 0.9, igo + 8.0,
                             lx, -D.CASE_HOLES_BOT[1]))
        s.append(_flat_head_y(lx, -D.CASE_HOLES_BOT[1], igo, -1))
    return _fuse(s)


# ----------------------------------------------------- pitch-type joint discs
def disc_screws_y():
    """One hip-pitch / knee / ankle joint (axis == Y at origin): 4x M3x6
    button into the horn disc (heads on the fork pad outer face) + 4x M3x8
    button + thin washer into the idler disc."""
    hy1 = D.SV_HORN_FACE + D.PLATE                    # 23.45 horn pad outer
    iy0 = D.IDLER_ARM_INNER - D.PLATE                 # -21   idler pad outer
    r = D.BCD / 2
    s = []
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        # HORN: M3x5, not the M3x6 the servo bundles. The horn disc is a thick
        # hub inside a 2.5 mm FLANGE (bench 2026-07-30), and the bolt circle is
        # out in the flange -- an M3x6 through the 3.0 plate drives 3.0 mm into
        # a 2.5 mm hole.
        s.append(parts.cyl_y(1.5, hy1 - 5.0, hy1, dx, dz))
        s.append(parts.cyl_y(D.M3_HEAD_D / 2, hy1, hy1 + D.M3_HEAD_H, dx, dz))
        # IDLER: the same screw, and NO WASHER. The old M3x8 + thin washer
        # reached 3.30 mm into a 2.1 mm hole -- it bottomed out 1.20 mm early
        # and never clamped. The washer was compensating for a screw that was
        # too long for a hole shallower than anyone had measured.
        s.append(parts.cyl_y(1.5, iy0, iy0 + D.DISC_SCREW_THREAD, dx, dz))
        s.append(parts.cyl_y(D.M3_HEAD_D / 2, iy0 - D.M3_HEAD_H, iy0, dx, dz))
    return _fuse(s)


# --------------------------------------------------------------- hip roll
def disc_screws_x():
    """Hip-roll joint (axis == X at origin, yoke_roll frame): 4x M3x6 button
    into the horn + 4x M3x6 button into the idler disc through the long boss.

    The idler screw was M3x10 + THIN WASHER until 2026-07-30, then M3x8 into a
    pad sunk ROLL_IDLER_PAD_SINK. Both were chasing a 7.15 mm stack the bay
    wall will not let us shorten. 2026-07-31 it went the other way: the pad
    also gets a per-screw HEAD COUNTERBORE, which drops the stack to 3.90 and
    lets this joint use the SAME M3x6 as every other disc -- the one that
    ships with the servos and is actually on the bench. The head seats
    ROLL_IDLER_HEAD_CB below the sunk pad face, on the idler boss."""
    hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE    # 24.45 horn pad outer
    ix0 = (D.ROLL_ARM_INNER - D.PLATE + D.ROLL_IDLER_PAD_SINK
           + D.ROLL_IDLER_HEAD_CB)                    # -20.70 counterbore floor
    r = D.BCD / 2
    s = []
    for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append(parts.cyl_x(1.5, hx1 - 6.0, hx1, dy, dz))
        s.append(parts.cyl_x(D.M3_HEAD_D / 2, hx1, hx1 + D.M3_HEAD_H, dy, dz))
        s.append(parts.cyl_x(1.5, ix0, ix0 + D.ROLL_DISC_SCREW_THREAD, dy, dz))
        s.append(parts.cyl_x(D.M3_HEAD_D / 2, ix0 - D.M3_HEAD_H, ix0, dy, dz))
    return _fuse(s)


def flange_bolts():
    """4x M3x10 down through the yoke_roll flange into the yoke_pitch flange
    heat-sets (roll-axis frame: flange spans z -16..-20)."""
    zf0 = -D.ROLL_AXIS_TO_FLANGE                      # -16 flange top
    b = D.YOKE_BOLT_SQ / 2
    s = []
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        s.append(parts.cyl_z(1.5, zf0 - 10.0, zf0, sx, sy))
        s.append(parts.cyl_z(D.M3_HEAD_D / 2, zf0, zf0 + D.M3_HEAD_H, sx, sy))
    return _fuse(s)


# --------------------------------------------------------------- hip yaw
def yaw_horn_screws():
    """Carrier frame (yaw horn face == z 0): 4x M3x6 button UP into the yaw
    horn -- heads proud of the bay ceiling, the reason for
    CARRIER_ROLL_HEAD_CLEAR."""
    zc = D.CARRIER_ROLL_CEIL
    r = D.BCD / 2
    s = []
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append(parts.cyl_z(1.5, zc, zc + 6.0, dx, dy))
        s.append(parts.cyl_z(D.M3_HEAD_D / 2, zc - D.M3_HEAD_H, zc, dx, dy))
    return _fuse(s)


def yaw_wall_screws():
    """8x M2.5x8 FLAT-head self-tap through the carrier bay walls into the
    roll-servo case (heads proud of the wall outer faces; the shanks
    INTENTIONALLY enter the servo case -- exclude from servo booleans)."""
    za = D.CARRIER_ROLL_AXIS
    s = []
    for zrow in D.CASE_HOLES_TOP:                     # front wall (horn face)
        for sg in (1, -1):
            s.append(parts.cyl_x(1.25, 19.95 - 8.0, 19.95,
                                 sg * D.CASE_HOLE_LAT, za + zrow))
            s.append(_flat_head_x(sg * D.CASE_HOLE_LAT, za + zrow, 19.95, +1))
    for zrow in D.CASE_HOLES_BOT:                     # rear wall (idler face)
        for sg in (1, -1):
            s.append(parts.cyl_x(1.25, -19.95, -19.95 + 8.0,
                                 sg * D.CASE_HOLE_LAT, za + zrow))
            s.append(_flat_head_x(sg * D.CASE_HOLE_LAT, za + zrow, -19.95, -1))
    return _fuse(s)


def yaw_carrier_screws():
    """All 12 carrier fasteners (horn bolts + wall retention)."""
    return yaw_horn_screws() + yaw_wall_screws()


# --------------------------------------------------------------- pelvis
def deck_stator_screws():
    """Pelvis frame (deck top == z 0), BOTH legs: 8x M2.5x8 FLAT-head self-tap down
    through the deck into the yaw-servo idler-face rows, heads flush in the
    deck-top countersinks (the battery footprint covers the -8.30 row)."""
    s = []
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for sg in (1, -1):
                x, y = -xrow, by + sg * D.CASE_HOLE_LAT
                s.append(parts.cyl_z(1.25, -8.0, 0.0, x, y))
                s.append(_flat_head_z(x, y, 0.0, +1))
    return _fuse(s)


# --------------------------------------------------------------- foot
def foot_screws():
    """Foot frame: 3x M2.5x8 FLAT-head self-tap through the retention tabs
    into the ankle-servo case (horn row -29 on +Y, LOW ONLY since the
    2026-07-30 rib window took the high seat; idler row -32.75 on -Y, both).
    Flush in tab countersinks: a proud pan head was tangent to the shin fork
    blade at ankle -40 (audit 2026-07-28)."""
    py = D.SV_TOPFACE + D.FIT                         # 17.65 pocket half width
    zp = D.FOOT_T - D.FOOT_POCKET_D                   # 4.0 pocket floor
    fo = py + D.FOOT_WALL_T                           # 20.05 horn tab outer
    # idler tab outer face: FOOT_WALL_T off the seat (-17.30), pulled in
    # 2026-07-30 so the same M2.5x8 works here too (was -20.05 clamping air)
    ito = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.FOOT_WALL_T
    s = []
    for zh in (2.11, 22.61):
        s.append(parts.cyl_y(1.25, ito + 0.9, ito + 8.0, -32.75, zp + zh))
        s.append(_flat_head_y(-32.75, zp + zh, ito, -1))
    s.append(parts.cyl_y(1.25, fo - 8.0, fo - 0.9, -29.0, zp + 2.11))
    s.append(_flat_head_y(-29.0, zp + 2.11, fo, +1))
    return _fuse(s)


# --------------------------------------------------------------- torso
def tower_screws():
    """Tower frame (deck top == z 0): 4x M3x10 button down through the feet
    tabs into the deck heat-sets + 4x M2.5 machine pan driven along +x into the
    board partition (the General Driver board stands UPRIGHT against the +x
    side -- 2026-07-28; it used to hang face-down under the top plate)."""
    s = []
    for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X):
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            s.append(parts.cyl_z(1.5, -6.0, 3.9, sx, sy))
            s.append(parts.cyl_z(D.M3_HEAD_D / 2, 3.9, 3.9 + D.M3_HEAD_H,
                                 sx, sy))
    # board screws now run along -x: head on the board's +x (component) face,
    # shank through the PCB and into the boss standing off the partition.
    xh = D.BOARD_GD_PCB_X + 1.63                      # PCB +x face = head seat
    for sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY):
        for sz in (D.BOARD_GD_CZ + D.BOARD_GD_SCREW_DZ,
                   D.BOARD_GD_CZ - D.BOARD_GD_SCREW_DZ):
            # shank at the pilot dia: it self-taps the boss, and modeling the
            # thread OD would read as a (false) collision
            s.append(parts.cyl_x(D.M25_TAP / 2, xh - 6.0, xh, sy, sz))
            s.append(parts.cyl_x(D.M25_HEAD_D / 2, xh, xh + D.M25_HEAD_H,
                                 sy, sz))
    return _fuse(s)


def head_stack_screws():
    """Tower-top frame (tower top == z 0): 4x M3x12 self-tap through
    gopro_base + imu_carrier into the tower bosses + 2x M2.5 clamping the
    GY-BNO08X into its pocket."""
    s = []
    gz = D.IMU_CARRIER_T + D.GP_BASE_T                # 7.0 gopro base top
    for sx in (D.GP_SCREW_XY[0], -D.GP_SCREW_XY[0]):
        for sy in (D.GP_SCREW_XY[1], -D.GP_SCREW_XY[1]):
            # self-tap shanks modeled at their pilot dia (see tower_screws)
            s.append(parts.cyl_z(D.M3_ST_PILOT / 2, gz - 12.0, gz, sx, sy))
            s.append(parts.cyl_z(D.M3_HEAD_D / 2, gz, gz + D.M3_HEAD_H,
                                 sx, sy))
    bt = D.IMU_PCB_Z + D.IMU_PCB[2]                   # 5.28 IMU board top
    for sy in (D.IMU_CY + D.IMU_SCREW_DY, D.IMU_CY - D.IMU_SCREW_DY):
        s.append(parts.cyl_z(D.M25_TAP / 2, bt - 6.0, bt, D.IMU_SCREW_X, sy))
        s.append(parts.cyl_z(D.M25_HEAD_D / 2, bt, bt + D.M25_HEAD_H,
                             D.IMU_SCREW_X, sy))
    return _fuse(s)


# =============================================================================
# INSERTION (APPROACH) PATHS -- 2026-07-30
# =============================================================================
# WHY THIS EXISTS. The mocks above model the SEATED screw: a flat head flush
# inside its countersink, a shank starting just inboard of the head plane. So
# check_assembly's "fasteners seated in their own parts" boolean proves the
# recess swallows the head -- and NOTHING proved the screw could be brought to
# that seat. It cannot see material standing in front of the countersink,
# because that material is in the air the screw arrives THROUGH, not in the
# volume it finally occupies.
#
# That blindness escaped a real bug (fixed in a4ce69e, 2026-07-30): the
# leg_link's two idler grip screws seat at y -17.90, but the jog block and the
# fork wide plate ran out to y -21 -- 3.1 mm PROUD of the seat -- and their
# z -33 top edges cut straight across the lower half of each countersink
# circle. The screws could not be inserted or driven square. check_assembly
# said ALL CLEAR the whole time.
#
# The fix here is to model the approach as geometry: a cylinder on the screw
# axis, at the widest diameter that must pass, extruded from the head's seated
# plane BACKWARD (away from the part) well outside the bounding box. It is
# legitimate for that cylinder to run through open air and through the bore /
# countersink voids -- those are voids, so a clean part intersects it at ~0.
INSERT_LEN = 25.0       # how far back the approach is modelled (well outside
                        # every host part's bbox on the screw axis)

# Envelope diameters, i.e. the widest thing that has to travel down the axis:
#
#   csk25  M2.5 flat head in a 90-deg countersink. NOT the head (4.7) but the
#          COUNTERSINK MOUTH + 0.8: the PH1 bit has to enter the cone with the
#          head, and it is the bit shank that rubs the mouth rim. 6.2 is the
#          established class -- a4ce69e's access counterbores are exactly that,
#          cut for exactly this reason.
#   btn3   M3 button head: it only has to PASS (its hex key is 2.0 across
#          flats, far narrower than the head), so the class is the head OD +
#          0.4 -- e.g. the tower's O6.4 feet-screw access holes are drilled to
#          pass this head and nothing more.
#   wsh3   M3 button + THIN WASHER on the idler discs: the washer (7.0) is
#          wider than the head, so the washer sets the envelope.
#   pan25  M2.5 machine pan (driver board / IMU), head OD + 0.4.
ENVELOPE_D = {"csk25": D.CASE_CS_D + 0.8,        # 6.2
              "btn3":  D.M3_HEAD_D + 0.4,        # 6.1
              "wsh3":  D.M3_WASHER_D + 0.4,      # 7.4
              "pan25": D.M25_HEAD_D + 0.4}       # 4.9


def _sweep(seat, length=INSERT_LEN):
    """One screw's approach volume: ENVELOPE_D cylinder from the head's seated
    plane, running BACK along the axis (i.e. -sign) by `length`."""
    x, y, z, axis, sign, kind = seat
    r = ENVELOPE_D[kind] / 2
    if axis == "y":
        return parts.cyl_y(r, y, y + sign * length, x, z)
    if axis == "x":
        return parts.cyl_x(r, x, x + sign * length, y, z)
    return parts.cyl_z(r, z, z + sign * length, x, y)


def sweeps(seats, length=INSERT_LEN):
    return _fuse([_sweep(s, length) for s in seats])


def seat_probes(seats, r=0.3):
    """One small sphere per seat, ON the seated head plane. check_assembly
    intersects these with the screw group: every probe must find head metal.
    That is the anti-DRIFT guard -- the seat tables below duplicate the head
    coordinates of the screw builders above, and this is what fails loudly if
    one of the two moves without the other."""
    return _fuse([Pos(s[0], s[1], s[2]) * Sphere(r) for s in seats])


# ---- seat tables: (x, y, z, axis, sign, kind); sign = which way the head
# ---- faces, i.e. the direction the driver comes FROM.
def leg_link_seats():
    """6x M2.5 flat, leg_link frame -- mirrors leg_link_screws()."""
    seat = D.SV_TOPFACE + D.GRIP_PLATE_T                     # 19.75
    s = [(lx, seat, -zrow, "y", +1, "csk25")
         for zrow in D.CASE_HOLES_TOP
         for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT)]
    igo = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER
    s += [(lx, igo, -D.CASE_HOLES_BOT[1], "y", -1, "csk25")
          for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT)]     # the a4ce69e pair
    return s


def disc_y_seats():
    hy1 = D.SV_HORN_FACE + D.PLATE
    iy0 = D.IDLER_ARM_INNER - D.PLATE
    r = D.BCD / 2
    s = []
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append((dx, hy1, dz, "y", +1, "btn3"))
        s.append((dx, iy0, dz, "y", -1, "btn3"))   # was wsh3; the washer is gone
    return s


def disc_x_seats():
    hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE
    # The idler head seats on the COUNTERBORE FLOOR, not on the arm's outer
    # face: the pad is sunk ROLL_IDLER_PAD_SINK and then pocketed
    # ROLL_IDLER_HEAD_CB deeper so the M3x6 reaches the disc (2026-07-31).
    # This literal used to be a hardcoded -23.95 and a "wsh3" washer seat,
    # both stale -- the washer went on 2026-07-30 and the sink moved the face.
    ix0 = (D.ROLL_ARM_INNER - D.PLATE + D.ROLL_IDLER_PAD_SINK
           + D.ROLL_IDLER_HEAD_CB)                    # -20.70
    r = D.BCD / 2
    s = []
    for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append((hx1, dy, dz, "x", +1, "btn3"))
        s.append((ix0, dy, dz, "x", -1, "btn3"))
    return s


def flange_bolt_seats():
    zf0 = -D.ROLL_AXIS_TO_FLANGE
    b = D.YOKE_BOLT_SQ / 2
    return [(sx, sy, zf0, "z", +1, "btn3")
            for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b))]


def yaw_horn_seats():
    """Heads sit UNDER the bay ceiling, so the driver comes up from inside the
    empty bay -- which is why these are driven in §7b, BEFORE the roll servo
    goes in in §7c. Checked against the carrier only, for that reason."""
    zc = D.CARRIER_ROLL_CEIL
    r = D.BCD / 2
    return [(dx, dy, zc, "z", -1, "btn3")
            for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r))]


def yaw_wall_seats():
    za = D.CARRIER_ROLL_AXIS
    s = [(19.95, sg * D.CASE_HOLE_LAT, za + zrow, "x", +1, "csk25")
         for zrow in D.CASE_HOLES_TOP for sg in (1, -1)]
    s += [(-19.95, sg * D.CASE_HOLE_LAT, za + zrow, "x", -1, "csk25")
          for zrow in D.CASE_HOLES_BOT for sg in (1, -1)]
    return s


def deck_stator_seats():
    return [(-xrow, by + sg * D.CASE_HOLE_LAT, 0.0, "z", +1, "csk25")
            for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2)
            for xrow in D.YAW_CASE_HOLES_IDLER for sg in (1, -1)]


def foot_seats():
    """The 3 REAR TAB screws only (horn LOW dropped its high twin to the
    2026-07-30 rib window) -- the two front-boss screws have holes and
    countersinks in parts.foot() but no screw solid in foot_screws(), and the
    seat probe guard would (rightly) fail on a sweep with no screw behind it.
    Model them there first if they are ever wanted here."""
    zp = D.FOOT_T - D.FOOT_POCKET_D
    fo = D.SV_TOPFACE + D.FIT + D.FOOT_WALL_T                # 20.05
    ito = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.FOOT_WALL_T
    s = []
    for zh in (2.11, 22.61):
        s.append((-32.75, ito, zp + zh, "y", -1, "csk25"))
    s.append((-29.0, fo, zp + 2.11, "y", +1, "csk25"))
    return s


def tower_seats():
    s = [(sx, sy, 3.9, "z", +1, "btn3")
         for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X)
         for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y)]
    xh = D.BOARD_GD_PCB_X + 1.63
    s += [(xh, sy, sz, "x", +1, "pan25")
          for sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY)
          for sz in (D.BOARD_GD_CZ + D.BOARD_GD_SCREW_DZ,
                     D.BOARD_GD_CZ - D.BOARD_GD_SCREW_DZ)]
    return s


def head_stack_seats():
    gz = D.IMU_CARRIER_T + D.GP_BASE_T
    s = [(sx, sy, gz, "z", +1, "btn3")
         for sx in (D.GP_SCREW_XY[0], -D.GP_SCREW_XY[0])
         for sy in (D.GP_SCREW_XY[1], -D.GP_SCREW_XY[1])]
    bt = D.IMU_PCB_Z + D.IMU_PCB[2]
    s += [(D.IMU_SCREW_X, sy, bt, "z", +1, "pan25")
          for sy in (D.IMU_CY + D.IMU_SCREW_DY, D.IMU_CY - D.IMU_SCREW_DY)]
    return s


# group name -> (seat table, the screw solids those seats belong to). Both
# check_assembly (collision + drift guard) and render_assembly_steps (the
# translucent path cones in the figures) read THIS, so the doc figures and the
# check can never draw different paths.
SEATS = {
    "screws_grip":        (leg_link_seats,   lambda: leg_link_screws()),
    "screws_disc_y":      (disc_y_seats,     disc_screws_y),
    "screws_disc_x":      (disc_x_seats,     disc_screws_x),
    "screws_flange":      (flange_bolt_seats, flange_bolts),
    "screws_yaw_horn":    (yaw_horn_seats,   yaw_horn_screws),
    "screws_yaw_wall":    (yaw_wall_seats,   yaw_wall_screws),
    "screws_deck":        (deck_stator_seats, deck_stator_screws),
    "screws_foot":        (foot_seats,       foot_screws),
    "screws_tower":       (tower_seats,      tower_screws),
    "screws_head_stack":  (head_stack_seats, head_stack_screws),
}


def paths(group, length=INSERT_LEN):
    """The insertion sweep solid for a named group in SEATS."""
    return sweeps(SEATS[group][0](), length)


# ------------------------------------------------------------- STEP exports
# FreeCAD can't run build123d, so the poseable assembly (freecad_articulate.py)
# reads the fasteners as STEPs -- one per group, in the SAME local frame the
# group is defined in, so FreeCAD places a screw group exactly where the check
# places it. Regenerated by `python cad/fasteners.py`.
GROUPS = {
    "screws_grip":         lambda: leg_link_screws(),          # leg_link frame
    "screws_disc_y":       disc_screws_y,                      # pitch/knee/ankle
    "screws_roll":         lambda: disc_screws_x() + flange_bolts(),
    "screws_yaw_carrier":  yaw_carrier_screws,                 # carrier frame
    "screws_deck":         deck_stator_screws,                 # pelvis frame
    "screws_tower":        tower_screws,                       # tower frame
    "screws_head_stack":   head_stack_screws,                  # tower-top frame
    "screws_foot":         foot_screws,                        # foot frame
}


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")
    os.makedirs(out, exist_ok=True)
    for name, fn in GROUPS.items():
        solid = fn()
        export_step(solid, os.path.join(out, f"{name}.step"))
        print(f"{name:20s} {solid.volume / 1000.0:6.2f} cm3 "
              f"-> step/{name}.step")


if __name__ == "__main__":
    main()
