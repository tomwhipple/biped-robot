"""Every fastener in the robot as a simple solid (shank + head [+ washer]),
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

from build123d import Pos, Rot, Cone, export_step
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
def leg_link_screws(pan_heads=False):
    """6x M2.5x8 FLAT-head self-tap gripping the servo case (leg_link frame:
    upper joint axis == Y at origin). Heads sit FLUSH in the grip-plate
    countersinks -- see the leg_link comment / bench skew of 2026-07-28.

    pan_heads=True models the AS-FITTED bench hardware instead (uxcell pan
    heads, O5.0 x 2.0 PROUD of the plate faces) -- for demonstrating why the
    links skew: the horn-side heads stand in the 0.70 mm band the yoke/fork
    arm sweeps. Never use for the design-intent checks."""
    s = []
    seat = D.SV_TOPFACE + D.GRIP_PLATE_T              # 19.75 horn plate outer
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append(parts.cyl_y(1.25, seat - 8.0, seat - 0.9, lx, -zrow))
            if pan_heads:
                s.append(parts.cyl_y(D.CASE_HEAD_D / 2, seat,
                                     seat + D.CASE_HEAD_H, lx, -zrow))
            else:
                s.append(_flat_head_y(lx, -zrow, seat, +1))
    iy0 = D.IDLER_ARM_INNER - D.PLATE                 # -21 idler plate outer
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append(parts.cyl_y(1.25, iy0 + 0.9, iy0 + 8.0,
                             lx, -D.CASE_HOLES_BOT[1]))
        if pan_heads:
            s.append(parts.cyl_y(D.CASE_HEAD_D / 2, iy0 - D.CASE_HEAD_H, iy0,
                                 lx, -D.CASE_HOLES_BOT[1]))
        else:
            s.append(_flat_head_y(lx, -D.CASE_HOLES_BOT[1], iy0, -1))
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
        s.append(parts.cyl_y(1.5, hy1 - 6.0, hy1, dx, dz))
        s.append(parts.cyl_y(D.M3_HEAD_D / 2, hy1, hy1 + D.M3_HEAD_H, dx, dz))
        w0 = iy0 - D.M3_WASHER_T
        s.append(parts.cyl_y(D.M3_WASHER_D / 2, w0, iy0, dx, dz))
        s.append(parts.cyl_y(1.5, w0, w0 + 8.0, dx, dz))
        s.append(parts.cyl_y(D.M3_HEAD_D / 2, w0 - D.M3_HEAD_H, w0, dx, dz))
    return _fuse(s)


# --------------------------------------------------------------- hip roll
def disc_screws_x():
    """Hip-roll joint (axis == X at origin, yoke_roll frame): 4x M3x6 button
    into the horn + 4x M3x10 button + washer into the idler disc through the
    long boss."""
    hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE    # 24.45 horn pad outer
    ix0 = -19.95 - 1.0 - D.PLATE                      # -23.95 idler pad outer
    r = D.BCD / 2
    s = []
    for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append(parts.cyl_x(1.5, hx1 - 6.0, hx1, dy, dz))
        s.append(parts.cyl_x(D.M3_HEAD_D / 2, hx1, hx1 + D.M3_HEAD_H, dy, dz))
        w0 = ix0 - D.M3_WASHER_T
        s.append(parts.cyl_x(D.M3_WASHER_D / 2, w0, ix0, dy, dz))
        s.append(parts.cyl_x(1.5, w0, w0 + 10.0, dy, dz))
        s.append(parts.cyl_x(D.M3_HEAD_D / 2, w0 - D.M3_HEAD_H, w0, dy, dz))
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
    """8x M2.5x8 pan self-tap through the carrier bay walls into the
    roll-servo case (heads proud of the wall outer faces; the shanks
    INTENTIONALLY enter the servo case -- exclude from servo booleans)."""
    za = D.CARRIER_ROLL_AXIS
    s = []
    for zrow in D.CASE_HOLES_TOP:                     # front wall (horn face)
        for sg in (1, -1):
            s.append(parts.cyl_x(1.25, 19.95 - 8.0, 19.95,
                                 sg * D.CASE_HOLE_LAT, za + zrow))
            s.append(parts.cyl_x(D.CASE_HEAD_D / 2, 19.95, 19.95 + D.CASE_HEAD_H,
                                 sg * D.CASE_HOLE_LAT, za + zrow))
    for zrow in D.CASE_HOLES_BOT:                     # rear wall (idler face)
        for sg in (1, -1):
            s.append(parts.cyl_x(1.25, -19.95, -19.95 + 8.0,
                                 sg * D.CASE_HOLE_LAT, za + zrow))
            s.append(parts.cyl_x(D.CASE_HEAD_D / 2, -19.95 - D.CASE_HEAD_H,
                                 -19.95, sg * D.CASE_HOLE_LAT, za + zrow))
    return _fuse(s)


def yaw_carrier_screws():
    """All 12 carrier fasteners (horn bolts + wall retention)."""
    return yaw_horn_screws() + yaw_wall_screws()


# --------------------------------------------------------------- pelvis
def deck_stator_screws():
    """Pelvis frame (deck top == z 0), BOTH legs: 8x M2.5x8 pan self-tap down
    through the deck into the yaw-servo idler-face rows, heads sunk in the
    deck-top counterbores (the battery footprint covers the -8.30 row)."""
    s = []
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for sg in (1, -1):
                x, y = -xrow, by + sg * D.CASE_HOLE_LAT
                zs = -D.DECK_CB_DEPTH                 # head seat (cb floor)
                s.append(parts.cyl_z(1.25, zs - 8.0, zs, x, y))
                s.append(parts.cyl_z(D.CASE_HEAD_D / 2, zs,
                                     zs + D.CASE_HEAD_H, x, y))
    return _fuse(s)


# --------------------------------------------------------------- foot
def foot_screws():
    """Foot frame: 4x M2.5x8 FLAT-head self-tap through the retention tabs
    into the ankle-servo case (horn row -29 on +Y, idler row -32.75 on -Y).
    Flush in tab countersinks: a proud pan head was tangent to the shin fork
    blade at ankle -40 (audit 2026-07-28)."""
    py = D.SV_TOPFACE + D.FIT                         # 17.65 pocket half width
    zp = D.FOOT_T - D.FOOT_POCKET_D                   # 4.0 pocket floor
    fo = py + D.FOOT_WALL_T                           # 20.05 tab outer face
    s = []
    for zh in (2.11, 22.61):
        s.append(parts.cyl_y(1.25, fo - 8.0, fo - 0.9, -29.0, zp + zh))
        s.append(_flat_head_y(-29.0, zp + zh, fo, +1))
        s.append(parts.cyl_y(1.25, -fo + 0.9, -fo + 8.0, -32.75, zp + zh))
        s.append(_flat_head_y(-32.75, zp + zh, -fo, -1))
    return _fuse(s)


# --------------------------------------------------------------- torso
def tower_screws():
    """Tower frame (deck top == z 0): 4x M3x10 button down through the feet
    tabs into the deck heat-sets + 4x M2.5 machine pan up into the driver
    board standoffs (board hangs face-down under the top plate)."""
    s = []
    for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X):
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            s.append(parts.cyl_z(1.5, -6.0, 3.9, sx, sy))
            s.append(parts.cyl_z(D.M3_HEAD_D / 2, 3.9, 3.9 + D.M3_HEAD_H,
                                 sx, sy))
    zt0 = D.TOWER_H - D.TOWER_TOP_T
    zb = zt0 - D.BOARD_STANDOFF - 1.6                 # board underside (PCB 1.6)
    bx, by = D.BOARD_HOLES[1] / 2, D.BOARD_HOLES[0] / 2
    for sx in (bx, -bx):
        for sy in (by, -by):
            # shank at the M25_TAP pilot dia: it self-taps the standoff, and
            # modeling the thread OD would read as a (false) collision
            s.append(parts.cyl_z(D.M25_TAP / 2, zb, zb + 5.0, sx, sy))
            s.append(parts.cyl_z(D.M25_HEAD_D / 2, zb - D.M25_HEAD_H, zb,
                                 sx, sy))
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


# ------------------------------------------------------------- STEP exports
# FreeCAD can't run build123d, so the poseable assembly (freecad_articulate.py)
# reads the fasteners as STEPs -- one per group, in the SAME local frame the
# group is defined in, so FreeCAD places a screw group exactly where the check
# places it. Regenerated by `python cad/fasteners.py`.
GROUPS = {
    "screws_grip":         lambda: leg_link_screws(),          # leg_link frame
    "screws_grip_pan":     lambda: leg_link_screws(True),      # AS-FITTED bench
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
