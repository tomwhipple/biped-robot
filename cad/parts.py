"""Printable parts for the Bimo-like biped (build123d, algebra mode).

Run:  .venv/bin/python cad/parts.py        -> exports STLs to cad/stl/, prints
                                              per-part bbox / bed check / mass and
                                              the assembly mass rollup.

Part set (6 unique, 12 prints):
  pelvis      x1  deck + two hanging bays for the hip-roll servos
  yoke_roll   x2  clevis on the hip-roll servo horn/idler, flange below
  yoke_pitch  x2  clevis on the thigh servo horn/idler, flange above
                  (bolts to yoke_roll flange, rotated 90 deg -> hip universal)
  leg_link    x4  thigh AND shin: grips a servo case, forks to the next servo
  foot        x2  sole + ankle servo pocket + rear retention walls
  tower       x1  electronics: driver board on top, battery strapped inside

Conventions: every pitch joint has the servo HORN on +Y; the same part serves
left and right legs (legs are translations, not mirrors).
"""
import os
from build123d import *
import dimensions as D

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl_x(r, x0, x1, y, z):
    """Cylinder along X from x0..x1 at (y, z)."""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def cyl_y(r, y0, y1, x, z):
    """Cylinder along Y from y0..y1 at (x, z)."""
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_z(r, z0, z1, x, y):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def bcd_y(y0, y1, x, z):
    """4x M3 clearance holes (horn/idler bolt circle) along Y at pad (x, z)."""
    r = D.BCD / 2
    return [cyl_y(D.PAD_HOLE / 2, y0, y1, x + dx, z + dz)
            for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


def bcd_x(x0, x1, y, z):
    r = D.BCD / 2
    return [cyl_x(D.PAD_HOLE / 2, x0, x1, y + dy, z + dz)
            for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


# ---------------------------------------------------------------- yoke_roll
def yoke_roll():
    """Hip-roll clevis. Local frame: roll axis == X axis through origin.
    +X = robot forward = servo horn side. Flange faces down (mates yoke_pitch).
    Print: flange face on the bed, arms up. Qty 2.
    """
    zf0 = -D.ROLL_AXIS_TO_FLANGE                    # flange top
    zf1 = zf0 - D.YOKE_FLANGE_T                     # flange bottom
    hx0, hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H, D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE
    ix1, ix0 = -19.95 - 1.0, -19.95 - 1.0 - D.PLATE  # idler plate (wall outer -19.95, 1 gap)

    p = box(-23.95, 24.45, -D.YOKE_FLANGE_Y / 2, D.YOKE_FLANGE_Y / 2, zf1, zf0)
    # horn arm: plate + boss through nothing (horn sits outside the bay wall)
    p += box(hx0, hx1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, hx0, hx1, 0, 0)
    p += cyl_x(D.HORN_BOSS_D / 2, D.SV_HORN_FACE, hx0, 0, 0)          # boss 1.0
    # idler arm: plate + long boss reaching through the bay-wall slot
    p += box(ix0, ix1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, ix0, ix1, 0, 0)
    p += cyl_x(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, ix1, 0, 0)        # boss 4.15
    # holes
    for h in bcd_x(D.SV_IDLER_FACE - 1, hx1 + 1, 0, 0):
        p -= h
    p -= cyl_x(D.HORN_CENTER_RELIEF_D / 2, D.SV_HORN_FACE - 1, hx1 + 1, 0, 0)
    p -= cyl_x(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 1, D.SV_IDLER_FACE + 0.7, 0, 0)
    b = D.YOKE_BOLT_SQ / 2
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        p -= cyl_z(D.M3_CLEAR / 2, zf1 - 1, zf0 + 1, sx, sy)
    return p


# ---------------------------------------------------------------- yoke_pitch
def yoke_pitch():
    """Hip-pitch clevis on the thigh-servo horn/idler. Local frame: pitch axis
    == Y axis through origin; flange on top (heat-set inserts, mates yoke_roll).
    Print: flange face on the bed, arms up (i.e. modeled upside-down vs print).
    Qty 2.
    """
    zf1 = D.PITCH_ARM_REACH                          # flange bottom (arms side)
    zf0 = zf1 + D.YOKE_FLANGE_T                      # flange top (mating face)
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21

    p = box(-D.YOKE_FLANGE_X / 2, D.YOKE_FLANGE_X / 2, iy0, hy1, zf1, zf0)
    p += box(-12, 12, hy0, hy1, 0, zf1) + cyl_y(D.PAD_D / 2, hy0, hy1, 0, 0)
    p += box(-12, 12, iy0, iy1, 0, zf1) + cyl_y(D.PAD_D / 2, iy0, iy1, 0, 0)
    p += cyl_y(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, iy1, 0, 0)   # boss 1.2
    for h in bcd_y(D.SV_IDLER_FACE - 1, hy1 + 1, 0, 0):
        p -= h
    p -= cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, 0)
    p -= cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 1, D.SV_IDLER_FACE + 0.7, 0, 0)
    b = D.YOKE_BOLT_SQ / 2
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        p -= cyl_z(D.HEATSET_D / 2, zf1 - 1, zf0 + 1, sx, sy)    # heat-set M3
    return p


# ---------------------------------------------------------------- leg_link
def leg_link():
    """Thigh / shin link (same part, qty 4). Local frame: upper joint axis ==
    Y axis at origin (this is the gripped servo's horn axis); the servo hangs
    below (top end +10.11, bottom -35.11); +X = robot forward; the lower fork
    grips the NEXT servo's horn (+Y) / idler (-Y) at z = -90.
    Print: lying on the back web (web face on the bed).
    """
    t = D.GRIP_PLATE_T
    drop = -D.LINK_DROP
    web_x1 = -12.36 - D.WEB_GAP                     # web inner face (cable gap)
    web_x0 = web_x1 - 2.4                           # web outer face, -15.16
    # --- grip channel on the servo case (plates reach the web outer face)
    p = box(web_x0, 12.36, D.SV_TOPFACE, D.SV_TOPFACE + t, D.GRIP_BOT, D.GRIP_TOP_HORN)
    # relief around the O19.6 output boss / horn skirt (they spin vs this plate)
    p -= cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    p += box(web_x0, 12.36, -D.SV_TOPFACE - D.PLATE, -D.SV_TOPFACE,
             D.GRIP_BOT, D.GRIP_TOP_IDLER)
    p += box(web_x0, web_x1, -D.SV_TOPFACE - D.PLATE, D.SV_TOPFACE + t,
             D.WEB_END, D.WEB_TOP)
    # --- fork arms down to the next servo (wide near the web, narrow below)
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21
    for a0, a1 in ((hy0, hy1), (iy0, iy1)):
        p += box(web_x0, 12, a0, a1, D.FORK_WIDE_Z, -33)
        p += box(D.FORK_NARROW_X, 12, a0, a1, drop, D.FORK_WIDE_Z)
        p += cyl_y(D.PAD_D / 2, a0, a1, 0, drop)
    p += cyl_y(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, iy1, 0, drop)
    # jog block joining horn grip plate (out at 19.75) to fork plate (21.45+)
    p += box(web_x0, 12, D.SV_TOPFACE, hy1, -36.5, -33)
    p += box(web_x0, 12, iy0, -D.SV_TOPFACE, -36.5, -33)
    # --- holes: case grip screws (M3 into the servo case holes)
    for zrow in D.CASE_HOLES_TOP:                 # horn-side face rows
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= cyl_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                       D.SV_TOPFACE + t + 1, lx, -zrow)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):   # idler face: row 32.75 only
        p -= cyl_y(D.CASE_SCREW_CLEAR / 2, -D.SV_TOPFACE - D.PLATE - 1,
                   -D.SV_TOPFACE + 1, lx, -D.CASE_HOLES_BOT[1])
    # --- holes: lower joint pads
    for h in bcd_y(D.SV_IDLER_FACE - 1, hy1 + 1, 0, drop):
        p -= h
    p -= cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, drop)
    p -= cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 1,
               D.SV_IDLER_FACE + 0.7, 0, drop)
    # --- zip-tie holes in the web (servo cable runs down the back)
    for z in (-40, -52):
        for ly in (7, -7):
            p -= cyl_x(2.25, web_x0 - 1, web_x1 + 1, ly, z)
    return p


# ---------------------------------------------------------------- pelvis
def pelvis():
    """Deck + two hanging bays for the hip-roll servos. Local frame: deck top
    at z=0, robot forward = +X, bays at y = +/-28. Roll axes at z = -45.11.
    The roll servos slide UP into the bays (output end down, horn forward,
    cables exit through the deck cutouts) and take 8x M3 each through the walls.
    Print: upside down (deck top face on the bed), bays rise as walls.
    """
    zd = -D.DECK_T
    zw = zd - D.BAY_WALL_DROP                       # wall bottoms, -46
    za = zd - D.SV_AXIS_FROM_REAR                   # roll axis height, -40.11
    p = box(-D.DECK_W / 2, D.DECK_W / 2, -D.DECK_L / 2, D.DECK_L / 2, zd, 0)
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        cy0 = 12.36 + D.BAY_CHEEK_GAP               # cheek inner face offset
        # front (horn-side) / rear (idler-side) walls on the case faces
        p += box(D.SV_TOPFACE, 19.95, by - cy0 - D.WALL, by + cy0 + D.WALL, zw, zd)
        p += box(-19.95, -D.SV_TOPFACE, by - cy0 - D.WALL, by + cy0 + D.WALL, zw, zd)
        # cheek walls (across the servo width)
        for s in (1, -1):
            p += box(-D.SV_TOPFACE, D.SV_TOPFACE, by + s * cy0,
                     by + s * (cy0 + D.WALL), zw, zd)
        # axis bore: downward-open U-slot in both walls
        p -= cyl_x(D.BAY_BORE / 2, -21, 21, by, za)
        p -= box(-21, 21, by - D.BAY_BORE / 2, by + D.BAY_BORE / 2, zw - 1, za)
        # servo retention screws (M3 through wall into case holes)
        for zrow in D.CASE_HOLES_TOP:               # front wall (horn face)
            for s in (1, -1):
                p -= cyl_x(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1, 21,
                           by + s * D.CASE_HOLE_LAT, za + zrow)
        for zrow in D.CASE_HOLES_BOT:               # rear wall (idler face)
            for s in (1, -1):
                p -= cyl_x(D.CASE_SCREW_CLEAR / 2, -21, -D.SV_TOPFACE + 1,
                           by + s * D.CASE_HOLE_LAT, za + zrow)
        # deck cutout over the bay: servo cable connectors are on the top end
        p -= box(-11, 11, by - 8, by + 8, zd - 1, 1)
    # tower mounting: raised bosses + heat-set pilots (land over the cheeks)
    for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X):
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            p += cyl_z(4.5, 0, 2, sx, sy)
            p -= cyl_z(D.HEATSET_D / 2, 2 - D.HEATSET_L, 2, sx, sy)
    # center lightening / wire window
    p -= box(-11, 11, -12, 12, zd - 1, 1)
    return p


# ---------------------------------------------------------------- foot
def foot():
    """Sole plate + ankle-servo pocket. Local frame: ankle axis vertical
    projection at origin, +X = toe, z=0 at the sole bottom. Servo lies on its
    side (horn +Y), output end forward at +10.11, cable end at the heel.
    Retention: 4x M3 through the two rear walls into the case holes + front
    end stop. TPU pad glued in the bottom recess. Print: sole down. Qty 2.
    """
    x0, x1 = -D.FOOT_HEEL, D.FOOT_L - D.FOOT_HEEL   # -38 .. +58
    w = D.FOOT_W / 2
    p = box(x0, x1, -w, w, 0, D.FOOT_T)
    # TPU pad recess (bottom)
    p -= box(x0 + D.PAD_INSET, x1 - D.PAD_INSET, -w + D.PAD_INSET, w - D.PAD_INSET,
             -1, D.PAD_RECESS)
    # servo pocket (top)
    px0 = -D.SV_AXIS_FROM_REAR - D.FIT
    px1 = D.SV_AXIS_FROM_OUT_END + D.FIT
    py = D.SV_TOPFACE + D.FIT                        # 17.65
    p -= box(px0, px1, -py, py, D.FOOT_T - D.FOOT_POCKET_D, D.FOOT_T + 1)
    # relief slots under the shin-fork joint pads (O24 pads dip below the
    # ankle axis: 16.36 - 12 = 4.36 -> relieve to 3.5 for 0.8 clearance)
    for sy0, sy1 in ((17.4, 24.1), (-24.1, -17.4)):
        p -= box(-13, 13, sy0, sy1, 3.5, D.FOOT_T + 1)
    # rear retention walls + front end stop
    wx0, wx1 = D.FOOT_WALL_X
    zp = D.FOOT_T - D.FOOT_POCKET_D                  # pocket floor, 4.0
    for s in (1, -1):
        p += box(wx0, wx1, s * py, s * (py + D.FOOT_WALL_T), zp, zp + D.FOOT_WALL_H)
    p += box(px1, px1 + D.WALL, -py, py, zp, zp + 8)
    # retention screw holes: horn face row 29.0 (+Y), idler face row 32.75 (-Y)
    for zh in (2.11, 22.61):
        p -= cyl_y(D.M3_CLEAR / 2, py - 1, py + D.FOOT_WALL_T + 1, -29.0, zp + zh)
        p -= cyl_y(D.M3_CLEAR / 2, -py - D.FOOT_WALL_T - 1, -py + 1, -32.75, zp + zh)
    return p


# ---------------------------------------------------------------- tower
def tower():
    """Electronics tower. The GoPro base bolts on top (4x M3 into bosses under
    the plate); the driver board hangs INSIDE, face down on standoffs under the
    top plate (screwed M2.5 from below). The 3S battery swaps tool-free: it
    tilt-loads through a window in the -x wall onto the pelvis deck, seats
    against far-wall rail stubs (dash x-loads) between the feet-tab gussets
    (y-location), behind the window sill (x shear-stop); a 20 mm hook-loop
    belt around the tower (guide ribs set its height) closes the window for
    tumbles, and a ribbon under the pack is the pull-tab. Feet tabs screw down
    into the deck heat-sets (access holes in the top plate). Local frame: z=0
    at deck top. Print: upside down (top plate on the bed) - standoffs/bosses
    print upward and the battery window opens toward the print top, so still
    support-free.
    """
    hx = D.TOWER_W / 2                               # walls are X-normal, 21
    hy = D.TOWER_L / 2                               # spans Y like the deck, 48
    zt0, zt1 = D.TOWER_H - D.TOWER_TOP_T, D.TOWER_H
    p = box(-hx, hx, -hy, hy, zt0, zt1)
    for s in (1, -1):
        p += box(s * (hx - D.WALL), s * hx, -hy, hy, 0, zt0 + 1)
        # feet tabs (inward) + 45 deg gusset wedge to the wall (sized for the
        # 154 g camera cantilevered ~85 mm above: a 10 g side hit ~ 1.3 N*m
        # -> ~25 N per screw, well inside heat-set / tab capacity with gussets)
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            p += box(s * (hx - D.WALL - 8.5), s * hx, sy - 6, sy + 6, 0, 4)
            p += box(s * (hx - D.WALL - 6), s * hx, sy - 6, sy + 6, 4, 12)
            p -= cyl_z(D.M3_CLEAR / 2, -1, 13, s * D.TOWER_FOOT_X, sy)
            p -= cyl_z(3.2, zt0 - 6, zt1 + 1, s * D.TOWER_FOOT_X, sy)  # driver access
    # battery window in the -x wall: sill 2.5 (tilt the pack in over it),
    # opening = envelope height, posts at the ends keep the feet tabs
    bw = D.BATT[0] / 2 + 1.0
    p -= box(-hx - 1, -hx + D.WALL + 1, -bw, bw, 2.5, D.BATT[2] + 2.5)
    # far-wall rail stubs: seat the pack inner face, stepped for the inverted
    # print like the feet tabs
    seat_in = D.BATT_SEAT_X + D.BATT[1]              # pack inner (+x) face, 14
    for sy in (-20, 16):
        p += box(seat_in, hx - D.WALL, sy - 6, sy + 6, 0, 12)
        p += box(seat_in + 1.3, hx - D.WALL, sy - 6, sy + 6, 12, 15)
    # belt guide ribs: +x wall full-width, -x wall on the window posts
    for rz in (D.BATT[2] - 8, D.BATT[2] + 3):
        p += box(hx, hx + 1.5, -22, 22, rz, rz + 1.5)
        for sy in (1, -1):
            p += box(-hx - 1.5, -hx, sy * (bw + 1), sy * (hy - 1), rz, rz + 1.5)
    # driver board standoffs under the plate (board face-down, M2.5 from below)
    bx, by = D.BOARD_HOLES[1] / 2, D.BOARD_HOLES[0] / 2
    for sx in (bx, -bx):
        for sy in (by, -by):
            p += cyl_z(3.5, zt0 - D.BOARD_STANDOFF, zt0, sx, sy)
            p -= cyl_z(D.M25_TAP / 2, zt0 - D.BOARD_STANDOFF - 1, zt1 - 1, sx, sy)
    # GoPro base screw bosses (M3 self-tap from above, through-pilots)
    gx, gy = D.GP_SCREW_XY
    for sx in (gx, -gx):
        for sy in (gy, -gy):
            p += cyl_z(4.0, zt0 - 3, zt0, sx, sy)
            p -= cyl_z(D.CASE_SCREW_PILOT / 2, zt0 - 4, zt1 + 1, sx, sy)
    # wire / vent holes (clear of the 30 x 24 GoPro base footprint)
    for sy in (20, -20):
        p -= cyl_z(5, zt0 - 1, zt1 + 1, 0, sy)
    return p


# ---------------------------------------------------------------- gopro_base
def gopro_base():
    """GoPro three-prong mount base for the camera's folding two-finger mount.
    Prongs stacked along Y => lens axis fore-aft (X). Dimensions follow the
    GoProScad standard (see dimensions.py). Bolts to the tower top with 4x M3
    self-tappers; the stock M5 thumbscrew clamps the camera. Local frame: z=0
    at the base bottom (tower top plane). Print: base down, prongs up (this is
    the standard, proven orientation for printed GoPro mounts; use PETG or
    100%-infill PLA). Separate part on purpose: it is the crash fuse.
    """
    hx, hy = D.GP_BASE_X / 2, D.GP_BASE_Y / 2
    zb = D.GP_BASE_T
    zh = zb + D.GP_HOLE_H                            # M5 hole center, 13.5
    p = box(-hx, hx, -hy, hy, 0, zb)
    pitch = D.GP_PRONG_T + D.GP_SLOT                 # 6.2
    for cy in (-pitch, 0.0, pitch):
        y0, y1 = cy - D.GP_PRONG_T / 2, cy + D.GP_PRONG_T / 2
        p += box(-D.GP_PRONG_OD / 2, D.GP_PRONG_OD / 2, y0, y1, zb, zh)
        p += cyl_y(D.GP_PRONG_OD / 2, y0, y1, 0, zh)
    p -= cyl_y(D.GP_HOLE_D / 2, -hy - 1, hy + 1, 0, zh)
    gx, gy = D.GP_SCREW_XY
    for sx in (gx, -gx):
        for sy in (gy, -gy):
            p -= cyl_z(D.M3_CLEAR / 2, -1, zb + 1, sx, sy)
    return p


# ---------------------------------------------------------------- build all
PARTS = [
    # name, builder, qty, print orientation note
    ("pelvis", pelvis, 1, "upside down: deck top on bed, bay walls rise"),
    ("yoke_roll", yoke_roll, 2, "flange face on bed, arms up"),
    ("yoke_pitch", yoke_pitch, 2, "flange face on bed, arms up"),
    ("leg_link", leg_link, 4, "on its back: web face on bed"),
    ("foot", foot, 2, "sole down"),
    ("tower", tower, 1, "upside down: top plate on bed"),
    ("gopro_base", gopro_base, 1, "base down, prongs up (PETG or 100% infill)"),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    rows, print_mass = [], 0.0
    for name, fn, qty, orient in PARTS:
        part = fn()
        path = os.path.join(OUT, f"{name}.stl")
        export_stl(part, path)
        bb = part.bounding_box()
        dims = sorted((bb.size.X, bb.size.Y, bb.size.Z))
        fits = dims[0] <= 250 and dims[1] <= D.BED and dims[2] <= D.BED
        vol = part.volume / 1000.0                     # cm^3
        mass = part.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        print_mass += mass * qty
        rows.append((name, qty, bb.size, vol, mass, fits, orient))
        print(f"{name:11s} x{qty}  bbox {bb.size.X:6.1f} x {bb.size.Y:6.1f} x "
              f"{bb.size.Z:6.1f} mm  vol {vol:6.1f} cm3  ~{mass:5.1f} g  "
              f"{'BED-OK' if fits else '** TOO BIG **'}  [{orient}]")

    # battery = the actual purchased pack (Zeee 3S 850: 74 g); its pigtail
    # lives in the wiring/misc bucket, not here (issue #2)
    servos, batt, board, fasteners, tpu = 8 * D.SERVO_MASS, 74.0, 20.0, 47.0, 16.0
    total = print_mass + servos + batt + board + fasteners + tpu
    print(f"\nprinted plastic ~{print_mass:.0f} g   servos {servos:.0f} g   "
          f"battery {batt:.0f} g   board {board:.0f} g   fasteners {fasteners:.0f} g"
          f"   TPU pads {tpu:.0f} g")
    print(f"TOTAL ROBOT ~{total:.0f} g   (+{D.CAM_MASS:.0f} g GoPro MAX = "
          f"{total + D.CAM_MASS:.0f} g)   (sim model: 930 g)")
    print(f"\nheights: ankle {D.ANKLE_Z:.1f}  knee {D.KNEE_Z:.1f}  "
          f"hip-pitch {D.HIP_PITCH_Z:.1f}  hip-roll {D.HIP_ROLL_Z:.1f}  "
          f"torso-center {D.TORSO_CENTER_Z:.1f}  top {D.TOP_Z:.1f} mm")

    # segment mass rollup for the sim update
    m = {n: r[4] for n, r in ((row[0], row) for row in rows)}
    seg = {
        "torso": m["pelvis"] + m["tower"] + m["gopro_base"]
                 + 2 * D.SERVO_MASS + batt + board + 27,
        "hip(roll link)": m["yoke_roll"] + m["yoke_pitch"] + 5,
        "thigh": m["leg_link"] + D.SERVO_MASS + 4,
        "shin": m["leg_link"] + D.SERVO_MASS + 4,
        "foot": m["foot"] + D.SERVO_MASS + tpu / 2 + 4,
    }
    print("\nsegment masses for sim v2 (g):  [camera +154 g on torso when mounted]")
    for k, v in seg.items():
        print(f"  {k:16s} {v:6.1f}")

    # standing CG estimate (approximate segment CG heights, mm above ground)
    cam_z = D.TOP_Z + D.GP_BASE_T + D.GP_HOLE_H + 6 + D.CAM_BODY[2] / 2  # ~373
    items = [
        (m["pelvis"], 265), (m["tower"], 306), (m["gopro_base"], D.TOP_Z + 3),
        (batt, 297), (board, 313), (2 * D.SERVO_MASS, 262), (27, 290),
        (2 * (m["yoke_roll"] + m["yoke_pitch"] + 5), 225),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), 175),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), 85),
        (2 * (m["foot"] + D.SERVO_MASS + tpu / 2 + 4), 12),
    ]
    mt = sum(w for w, _ in items)
    cg = sum(w * z for w, z in items) / mt
    cg_cam = (sum(w * z for w, z in items) + D.CAM_MASS * cam_z) / (mt + D.CAM_MASS)
    print(f"\nstanding CG ~{cg:.0f} mm; with camera ~{cg_cam:.0f} mm "
          f"(+{cg_cam - cg:.0f} mm, camera CG ~{cam_z:.0f} mm)")


if __name__ == "__main__":
    main()
