"""torso_v8: the v6 body's torso as two prints and two covers (Tom, 2026-10-14):

  pelvis_v8       the legs' block: both yaw cells under one ceiling, and the
                  bearing housing's five pilots. Printed ceiling-down.
  torso_v8        the electronics: deck, skins, the battery layer over the
                  pelvis, the board-stack bay forward (the Pi with the General
                  Driver on it as a HAT) and the power-board bay aft, right
                  behind the pack. Printed deck-down.
  front_cover_v8  the stack bay's front wall, screwed on. Printed flat.
  power_cover_v8  the power bay's aft wall, screwed on. Printed flat.
  battery_door_v8 the +y skin over the battery layer and the power board's
                  J1: one screw, one tab. Printed flat, outer face down.

Frame: deck TOP at z = 0, forward = +X, legs at y = +/-42
(dimensions_v6 HOUSING_*, PELVIS_*, TORSO_*, GD_*, PI_*, PWRB_*, BATT_*).

WHY TWO PRINTS. The one-print torso (cad/v6/pelvis_v7.py, in git history)
printed deck-down, so each yaw cell's
ceiling printed over the 31 mm battery layer: a 46 x 25 mm island over a
61 mm cavity, and the audit's other ledges and beams with it. Split at the
ceiling, every face prints off its own bed: the pelvis lies on its ceiling
and its cell walls rise; the torso lies on its deck and every wall, column
and boss rises from it. Neither part has a ceiling over a void.

THE JOINT: tab and glue (hip_yoke_glued's method). The pelvis goes UP into
the torso from below, between the torso's two inner walls (each 0.2 mm off a
web) and inside its skins (0.2 off the cell walls). Its ceiling stops on the
torso's -y skin ledge and pack ribs at z = CELL_TOP_Z, which carry the torso's
weight straight down the cell walls. Four snap fingers in the torso's inner
walls click into windows in the pelvis's webs, in the centre channel where
nothing turns, and hold it while the glue cures (CA or epoxy on the web
faces, the ledge and the skins). The joint is permanent: nothing in the
torso needs the pelvis off -- the board stack comes out of the front bay
and the power board out of the aft bay with their covers off, and the yaw
stator screws are still driven down through the deck aperture with the
pack out.

THE BATTERY DOOR (Tom, 2026-10-14: the pack swaps with at most one screw).
The door is the +y skin from J1 to the front inner wall (dimensions_v6
DOOR_*). Its aft edge is the tab: a tongue bevelled 40 deg off the plate's
normal that tucks behind the power bay's skin, whose jamb carries the
matching bevel, so the edge cannot lift outward. Its front edge takes one
M2.5 x 8 flat head into a post on the front inner wall's aft face. To swap:
the screw out, the door swung open about its tongue, the XT60 pulled off J1
and back through the aft wall's window, the pack slid out sideways. Ribs on
the door's inner face and a block under the deck at -y hold the pack's ends.

WINDOWS have flat tops. Each one whose top (its model-bottom edge, the
print-top in the deck-down torso) spans more than a bridge (BRIDGE_OK) gets
slicer support inside it and nowhere else: a support-enforcer box per window
(support_enforcers(), carried in cad/v6/print_settings.json; the yaw
carrier's method). The audit reads those windows as filled, which is what
the support does while they print.

The bearing housing screws into the pelvis only (dimensions_v6
YAW_HOUSING_SCREWS): the torso is not in the leg's load path.

    .venv/bin/python cad/v6/torso_v8.py     # STL (cad/v6/stl), STEP + renders (build/v6), checks, audits
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import (Axis, Cone, Pos, Rectangle, Rot, chamfer, export_step,  # noqa: E402
                       export_stl, extrude, fillet, loft)
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402
import paths  # noqa: E402

box, cyl_x, cyl_y, cyl_z = P.box, P.cyl_x, P.cyl_y, P.cyl_z
wedge_x, wedge_y, wedge_z = P.wedge_x, P.wedge_y, P.wedge_z
csk_x, csk_y, csk_z = P.csk_x, P.csk_y, P.csk_z

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

PIECES = ("pelvis_v8", "torso_v8", "front_cover_v8", "power_cover_v8", "battery_door_v8")
# rows = print axes in model coords (check_printability's convention)
RX180 = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
RY_XUP = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))
RY_XDOWN = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
RY_YDOWN = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))
PRINT_ORIENT = {
    "pelvis_v8": RX180,          # ceiling on the bed, cell walls rise
    "torso_v8": RX180,           # deck top on the bed
    "torso_v8_armless": RX180,
    "front_cover_v8": RY_XDOWN,  # outer face (+x) on the bed, corner blocks rise
    "power_cover_v8": RY_XUP,    # outer face (-x) on the bed, corner blocks rise
    "battery_door_v8": RY_YDOWN, # outer face (+y) on the bed, ribs and tongue rise
}

# ----------------------------------------------------------------------------
# derived local numbers
# ----------------------------------------------------------------------------
PX0, PX1 = V.PELVIS_X                          # -38.01 / +13.01
phw = V.PELVIS_HW                              # 57.26
tin = V.TORSO_IN_HW                            # 57.46
zd = -D.DECK_T                                 # -5 deck underside
zb = V.YAW_BOX_BOT_Z                           # -72.0 pelvis rim
zt = V.TORSO_BOT_Z                             # -71.5 torso bottom
zc = V.CELL_TOP_Z                              # -36.0 ceiling top = battery floor
zc_bot = zc - D.WALL                           # -38.6 ceiling underside, the yaw servos' seat
cx0 = D.YAW_CASE_X_REAR - D.YAW_SEAT_GAP       # -35.41 cell inner (rear)
cx1 = D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP      # +10.41 cell inner (front)
cyw = D.YAW_BOX_HW_IN                          # 12.66 case half width + fit
wt = D.YAW_SEAT_WALL                           # 2.6
lead = D.YAW_BOX_LEADIN                        # 1.2 mouth chamfer
chan_hw = V.HOUSING_CHAN_HW                    # 26.74
HIPS = (V.HIP_SEP / 2, -V.HIP_SEP / 2)
YAW_SEAT_CLR = 0.3                             # yaw servo's proud idler-side features vs the ceiling
W = D.WALL

# battery-layer ribs on the torso's inner walls: they locate the pack fore and
# aft and, standing on the pelvis's ceiling over its webs, carry the torso.
# The aft pair stands inboard of the aft stator screws' drivers (y
# 28.25..35.25). Both stop at RIB_TOP_Z with a 45 deg top, leaving a passage
# for leads along the pack's front and back under the deck.
RIB_Y_FWD = 29.0
RIB_Y_AFT = 22.0
RIB_W = 6.0
RIB_D_FWD = V.FWD_WALL_X[0] - (V.BATT_X[1] + V.BATT_RIB_CLR)      # 2.96
RIB_D_AFT = (V.BATT_X[0] - V.BATT_RIB_CLR) - V.AFT_WALL_X[1]      # 2.96
RIB_TOP_Z = -14.0
# deck edge rails: under the deck over the battery layer's ends, so the
# girdle's rail pilots (V.GIRDLE_PILOT_XY, 4.5 deep) do not end 0.5 over air
DECK_RAIL = (V.DECK_APER_HY + 0.3, 3.0)        # inner y, depth under the deck
# skin ledge: along the -y skin over the pelvis's outer cell wall (the +y skin
# there is the battery door's opening), 2.6 deep,
# 2 thick, with a 45 deg gusset above (it prints over the battery layer)
LEDGE_D, LEDGE_T = D.WALL, 2.0

# snap fingers (hip_yoke_glued's clip, one-sided): a strip of each inner wall
# cut free by two slots up to SNAP_ROOT_Z, a barb on its pelvis face that
# rides up the web as the pelvis goes in and drops into a window in the web.
SNAP_Y = 14.0                                  # finger centres, +/-y, in the centre channel
SNAP_W = 6.0
SNAP_SLOT = 1.0
SNAP_ROOT_Z = -54.0
SNAP_CATCH_Z = -66.0                           # barb's flat catch face (faces +z)
SNAP_BARB = 0.5                                # engagement past the 0.2 air
SNAP_FLAT = 0.5                                # barb's flat land below the catch
SNAP_RAMP = 2.0                                # lead-in ramp height (faces -z, the way the pelvis comes)
SNAP_WIN_CLR = (0.3, 0.15, 3.0)                # window: y clearance, play over the catch, depth below it

# the bearing housing's rear-outboard lugs on the pelvis: a boss standing aft
# of the rear web with a 45 deg gusset over it (it prints over air)
LUG_GUSSET = None                              # set below from the lug's reach

# power cover: corner blocks on its inner face, screwed through the skins
COVER_CLR = 0.2
COVER_BLOCK = (8.0, 6.0, 6.0)                  # x deep, y wide, z tall
COVER_SCREW_L = 8.0                            # M2.5 x 8 flat head: skin + 5 into the block


BRIDGE_OK = 8.0        # check_printability: a ceiling this wide bridges with no support
ENF_INSET = 0.25       # an enforcer box keeps this far inside a window's faces (the yaw carrier's)
ENF_INTO = 0.3         # ... and reaches this far into its ceiling


def _window_x(y0, y1, z0, z1, wall_x):
    """A flat-topped window through an X-normal wall (faces wall_x): the cut,
    and its support-enforcer box (model frame) when its ceiling -- the
    model-bottom edge, print-top in the deck-down torso -- spans more than a
    bridge; else None."""
    x0, x1 = wall_x
    cut = box(x0 - 1, x1 + 1, y0, y1, z0, z1)
    enf = None
    if y1 - y0 > BRIDGE_OK:
        enf = (x0 + ENF_INSET, x1 - ENF_INSET, y0 + ENF_INSET, y1 - ENF_INSET, z0 - ENF_INTO, z1)
    return cut, enf


def _window_y(x0, x1, z0, z1, s):
    """The same through the s side's skin (s = +1 / -1); the box keeps inside
    the skin's inner face and its tapered outer face at the window's foot."""
    a, b = sorted((s * (tin - 1), s * (V.HOUSING_HW + 1)))
    cut = box(x0, x1, a, b, z0, z1)
    enf = None
    if x1 - x0 > BRIDGE_OK:
        e0, e1 = sorted((s * (tin + ENF_INSET), s * (_skin_outer_y(z0) - ENF_INSET)))
        enf = (x0 + ENF_INSET, x1 - ENF_INSET, e0, e1, z0 - ENF_INTO, z1)
    return cut, enf


def _roofed_boss_x(r, x0, x1, y, z):
    """A boss along X off an X-normal wall with a 45 deg roof on its +z side:
    both parts print model -z up, so +z is the boss's print-down side
    (the board standoffs)."""
    th = math.radians(D.BF_BOSS_ROOF_DEG)
    b = cyl_x(r, x0, x1, y, z)
    b += (wedge_x([(y - r / math.sin(th), z), (y + r / math.sin(th), z),
                   (y, z + r / math.cos(th))], x0, x1)
          & box(x0, x1, y - r, y + r, z, z + 2 * r))
    return b


# ============================================================================
# PELVIS
# ============================================================================
LUG_X, LUG_Y = V.YAW_HOUSING_SCREWS_L[0]      # (-40, 54)
LUG_REACH = PX0 - (LUG_X - V.YAW_HOUSING_BOSS_R)                  # 4.99 aft of the rear web
LUG_TOP_Z = zb + V.YAW_HOUSING_BOSS_H + LUG_REACH                 # top of its 45 deg gusset


def _housing_bosses():
    """The bearing housing's five pilot bosses (dimensions_v6
    YAW_HOUSING_SCREWS), each rising off a web: a cylinder from the rim, then a
    45 deg cone (front, rear-inboard: inside the pelvis) or a 45 deg gusset
    back to the rear web (rear-outboard lugs, standing aft of it). Printed
    ceiling-down, the cone and the gusset are the faces that print over air."""
    hr, hh = V.YAW_HOUSING_BOSS_R, V.YAW_HOUSING_BOSS_H
    add, cut = None, None
    for hx, hy in V.YAW_HOUSING_SCREWS:
        b = cyl_z(hr, zb, zb + hh, hx, hy)
        if hx < PX0:     # rear-outboard lug
            b += box(hx, PX0 + 0.5, hy - hr, hy + hr, zb, zb + hh)
            b += wedge_y([(hx - hr, zb + hh), (PX0 + 0.5, zb + hh),
                          (PX0 + 0.5, zb + hh + (PX0 + 0.5 - (hx - hr)))], hy - hr, hy + hr)
            b = b & box(hx - hr, PX0 + 1, hy - hr, hy + hr, zb, LUG_TOP_Z + 1)
        else:
            b += Pos(hx, hy, zb + hh + hr / 2) * Cone(hr, 0, hr)
            # clipped to the pelvis's faces: the torso's walls are 0.2 off them
            b = b & box(PX0, PX1, -phw, phw, zb, zc)
        add = b if add is None else add + b
        c = cyl_z(D.M25_TAP / 2, zb - 0.1, zb + V.YAW_HOUSING_PILOT_DEPTH, hx, hy)
        cut = c if cut is None else cut + c
    return add, cut


def _snap_windows():
    """Windows in the pelvis's front and rear webs that the torso's barbs drop
    into (centre channel, so nothing but air is behind them)."""
    cy, cz_play, cz_depth = SNAP_WIN_CLR
    cut = None
    for x0, x1 in ((cx1 - 0.5, PX1 + 0.5), (PX0 - 0.5, cx0 + 0.5)):
        for s in (1, -1):
            y0, y1 = s * SNAP_Y - SNAP_W / 2 - cy, s * SNAP_Y + SNAP_W / 2 + cy
            w = box(x0, x1, y0, y1, SNAP_CATCH_Z - cz_depth, SNAP_CATCH_Z + cz_play)
            cut = w if cut is None else cut + w
    return cut


def pelvis_v8():
    """THE PELVIS: the yaw cell block, v5/v7's cells unchanged (plan, depth,
    seat gap, mouth lead-ins), under one ceiling that also spans the centre
    channel -- printed on the bed, it costs nothing to close it, and it ties
    the two cells together with the front and rear webs."""
    p = box(PX0, PX1, -phw, phw, zb, zc)
    for by in HIPS:
        p -= box(cx0, cx1, by - cyw, by + cyw, zb - 1, zc_bot)      # servo cell
        # mouth lead-in, all four inner faces (servo offered UP into the slip
        # fit; the rim is the top of the print, so these just thin the wall)
        p -= wedge_y([(cx0, zb), (cx0, zb + lead), (cx0 - lead, zb)],
                     by - cyw - wt - 1, by + cyw + wt + 1)
        p -= wedge_y([(cx1, zb), (cx1, zb + lead), (cx1 + lead, zb)],
                     by - cyw - wt - 1, by + cyw + wt + 1)
        for s in (1, -1):
            yi = by + s * cyw
            p -= wedge_x([(yi, zb), (yi, zb + lead), (yi + s * lead, zb)],
                         cx0 - wt - 1, cx1 + wt + 1)
    # centre channel under the ceiling: between the inner cheeks and the webs
    p -= box(cx0, cx1, -chan_hw, chan_hw, zb - 1, zc_bot)
    for by in HIPS:
        # stator screws: 4x M2.5 x 5 flat heads DOWN through the ceiling into
        # the idler-side case rows, countersunk in its top face
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zc_bot - 1, zc + 1, -xrow, by + s * D.CASE_HOLE_LAT)
                p -= csk_z(-xrow, by + s * D.CASE_HOLE_LAT, zc, +1)
        # the servo seats with its idler-side case face flat on the ceiling's
        # underside (no idler disc on the hip-yaw servos, docs/assembly.md 2):
        # - the free-hub post (2.6 proud, turns with the shaft): a hole;
        p -= cyl_z(D.SV_IDLER_HUB_HW + YAW_SEAT_CLR, zc_bot - 1, zc + 1, 0, by)
        # - the moulded back-cover platform (1.9 proud, static): a window,
        #   except round the aft stator countersinks, where a 2.2 relief
        #   leaves a skin (the platform comes within 0.6 of their mouths)
        px0, px1 = D.SV_IDLER_BOSS_Z[0] - YAW_SEAT_CLR, D.SV_IDLER_BOSS_Z[1] + YAW_SEAT_CLR
        py0, py1 = by - D.SV_IDLER_BOSS_HW - YAW_SEAT_CLR, by + D.SV_IDLER_BOSS_HW + YAW_SEAT_CLR
        relief_h = (D.SV_IDLER_CASE_FACE - D.SV_IDLER_BOSS_Y) + YAW_SEAT_CLR
        p -= box(px0, px1, py0, py1, zc_bot - 1, zc_bot + relief_h)
        keep = None
        for s in (1, -1):
            k = cyl_z(D.CASE_CS_D / 2 + 1.0, zc_bot - 1, zc + 1, -max(D.YAW_CASE_HOLES_IDLER), by + s * D.CASE_HOLE_LAT)
            keep = k if keep is None else keep + k
        p -= box(px0, px1, py0, py1, zc_bot - 1, zc + 1) - keep
        # - the connector trench: the lead goes up through a slit over it
        #   (the riser), then flat under the pack between its pads
        #   (V.BATT_PAD_T) to the gap behind it
        p -= box(-D.SV_CONN_L[0] + 0.5, -10.0, by - 4.5, by + 4.5, zc_bot - 1, zc + 1)
    p -= _snap_windows()
    add, cut = _housing_bosses()
    p += add
    p -= cut
    return p


# ============================================================================
# TORSO
# ============================================================================
def _plan(hw, r=2.0):
    """The torso's plan outline at half width hw: HOUSING_X long, its four
    corners R r (both ends close with a cover between the skins)."""
    x0, x1 = V.HOUSING_X
    face = Pos((x0 + x1) / 2, 0) * Rectangle(x1 - x0, 2 * hw)
    return fillet(face.vertices(), r)


def _envelope():
    """The outer shape: tapered skins from the bottom (HOUSING_BOT_HW) to the
    deck (HOUSING_HW), the deck on top. Its top edge stays sharp: it is on the
    bed, where a fillet starts as a horizontal overhang and a 45 deg chamfer
    round the corners tessellates past 45 deg."""
    lo = Pos(0, 0, zt) * _plan(V.HOUSING_BOT_HW)
    hi = Pos(0, 0, zd) * _plan(V.HOUSING_HW)
    body = loft([lo.faces()[0], hi.faces()[0]])
    deck = Pos(0, 0, zd) * extrude(_plan(V.HOUSING_HW), D.DECK_T)
    return body + deck


def _inner_walls():
    """The two walls either side of the pelvis, each overlapping the skins by
    0.3 so the union fuses: the front one carries the Pi's standoffs, the aft
    one the power board's."""
    o = 0.3
    w = box(*V.FWD_WALL_X, -tin - o, tin + o, zt, zd + o)
    w += box(*V.AFT_WALL_X, -tin - o, tin + o, zt, zd + o)
    return w


def _rib(x_wall, sx, y, d):
    """A pack rib on an inner wall's battery-layer face (sx: the direction it
    stands out, toward the pack), from the battery floor to RIB_TOP_Z, its top
    a 45 deg slope back to the wall so it prints off the wall and the leads
    have a passage over it."""
    o = 0.3
    xa, xb = sorted((x_wall + sx * d, x_wall - sx * o))
    r = box(xa, xb, y - RIB_W / 2, y + RIB_W / 2, zc, RIB_TOP_Z)
    face = x_wall + sx * d
    r += wedge_y([(face, RIB_TOP_Z), (x_wall - sx * o, RIB_TOP_Z), (x_wall - sx * o, RIB_TOP_Z + d + o)],
                 y - RIB_W / 2, y + RIB_W / 2)
    return r


def _ribs_and_ledges():
    """The pelvis's stops: pack ribs on both inner walls (standing on the
    ceiling over the webs) and a ledge along the -y skin (over the outer cell
    wall), with the 45 deg gusset the ledge prints off -- the +y skin over the
    pelvis is the battery door's opening, which the pack slides out over; and
    the deck edge rails for the girdle's pilots."""
    o = 0.3
    f = None
    for s in (1, -1):
        r = _rib(V.FWD_WALL_X[0], -1, s * RIB_Y_FWD, RIB_D_FWD)
        r += _rib(V.AFT_WALL_X[1], +1, s * RIB_Y_AFT, RIB_D_AFT)
        ry0, ry1 = sorted((s * DECK_RAIL[0], s * (tin + o)))
        r += box(V.AFT_WALL_X[1] - o, V.FWD_WALL_X[0] + o, ry0, ry1, zd - DECK_RAIL[1], zd + o)
        if s < 0:
            yi, yo = s * (tin - LEDGE_D), s * (tin + o)
            r += box(V.AFT_WALL_X[1] - o, V.FWD_WALL_X[0] + o, min(yi, yo), max(yi, yo), zc, zc + LEDGE_T)
            r += wedge_x([(yi, zc + LEDGE_T), (s * tin, zc + LEDGE_T), (s * tin, zc + LEDGE_T + LEDGE_D)],
                         V.AFT_WALL_X[1] - o, V.FWD_WALL_X[0] + o)
        f = r if f is None else f + r
    return f


def _snap_fingers():
    """(slots to cut, barbs to add) for the four fingers: two in each inner
    wall, the barb on the pelvis-facing side."""
    slots, barbs = None, None
    for wx, sx in ((V.FWD_WALL_X, -1), (V.AFT_WALL_X, +1)):
        face = wx[0] if sx < 0 else wx[1]           # the face toward the pelvis
        tip = face + sx * (V.TORSO_FIT + SNAP_BARB)
        for s in (1, -1):
            yc = s * SNAP_Y
            for e in (1, -1):
                y_in = yc + e * SNAP_W / 2
                sl = box(wx[0] - 0.5, wx[1] + 0.5, min(y_in, y_in + e * SNAP_SLOT),
                         max(y_in, y_in + e * SNAP_SLOT), zt - 1, SNAP_ROOT_Z)
                slots = sl if slots is None else slots + sl
            prof = [(face, SNAP_CATCH_Z), (tip, SNAP_CATCH_Z), (tip, SNAP_CATCH_Z - SNAP_FLAT),
                    (face, SNAP_CATCH_Z - SNAP_FLAT - SNAP_RAMP)]
            b = wedge_y(prof, yc - SNAP_W / 2, yc + SNAP_W / 2)
            barbs = b if barbs is None else barbs + b
    return slots, barbs


STACK_MALE = 6.0                               # the Pi's standoffs' male thread
PI_PILOT = STACK_MALE - V.PI_PCB_T + 0.5       # 5.0 into the boss and the wall
PWRB_SCREW_L = 6.0                             # M2.5 x 6 pan head: 4.4 into the standoff
PWRB_PILOT = PWRB_SCREW_L - V.PWRB_T + 0.6     # 5.0, inside the 6 mm boss


def _board_bosses():
    """(add, cut): four standoff bosses for the Pi off the front inner wall
    (the stack goes in from the front, so all four are bossed) and four for
    the power board off the aft inner wall."""
    o = 0.3
    add, cut = None, None
    for sy in V.PI_SCREW_Y:
        for sz in V.PI_SCREW_Z:
            a = _roofed_boss_x(3.5, V.FWD_WALL_X[1] - o, V.PI_PCB_X0, sy, sz)
            c = cyl_x(D.M25_TAP / 2, V.PI_PCB_X0 - PI_PILOT, V.PI_PCB_X0 + 1, sy, sz)
            add = a if add is None else add + a
            cut = c if cut is None else cut + c
    for hy, hz in V.PWRB_HOLES_YZ:
        add += _roofed_boss_x(3.5, V.PWRB_PCB_X1, V.PWRB_WALL_X[0] + o, hy, hz)
        cut += cyl_x(D.M25_TAP / 2, V.PWRB_PCB_X1 - 1, V.PWRB_PCB_X1 + PWRB_PILOT, hy, hz)
    return add, cut


def _cover_blocks_yz():
    """Centres (y, z) of a cover's four corner blocks."""
    bx, by, bz = COVER_BLOCK
    yb = tin - COVER_CLR - by / 2
    return [(s * yb, z) for s in (1, -1)
            for z in (zd - COVER_CLR - bz / 2, zt + bz / 2)]


# (name, inner face x, outer face x, inward sign): the covers close the two bays
COVERS = (("front_cover_v8", V.GD_FRONT_X, V.HOUSING_X[1], -1),
          ("power_cover_v8", V.PWRB_AFT_X, V.HOUSING_X[0], +1))


def _cover_screw_x(x_in, sx):
    return x_in + sx * COVER_BLOCK[0] / 2


def _skin_outer_y(z):
    """The skins' outer face (half width) at height z: the loft is linear."""
    f = (z - zt) / (zd - zt)
    return V.HOUSING_BOT_HW + f * (V.HOUSING_HW - V.HOUSING_BOT_HW)


# ============================================================================
# THE BATTERY DOOR (dimensions_v6 DOOR_*)
# ============================================================================
DOOR_OUT_Y = _skin_outer_y(V.DOOR_SCREW[1])      # 60.95: its outer face, the skin's at its mid-height
DOOR_TONGUE_Y = tin - V.DOOR_TONGUE_T           # 55.86: the tongue's inner face, behind the skin
DOOR_STEP_X = V.DOOR_X[0] + 2.2                  # where the tongue's inboard lip steps back to the plate
BEVEL_OFF = V.DOOR_CLR * math.hypot(1.0, V.DOOR_BEVEL)   # x shift for DOOR_CLR normal to the bevel
DOOR_RIB_T = 3.0                                 # the door's two pack ribs, thick in z
STOP_BOT_Z = -18.0                               # the -y pack stop's foot
STOP_SLOPE = 1.2                                 # its top over the aperture: dz per dy (50 deg)


def _bevel_x(y):
    """The aft jamb's bevel: x at y, from the opening's aft edge at the door's
    outer face, leaning aft as it goes in (DOOR_BEVEL dx per dy)."""
    return V.DOOR_X[0] - V.DOOR_BEVEL * (DOOR_OUT_Y - y)


def door_opening():
    """(cut, enforcer box): the opening in the +y skin, its aft jamb bevelled
    to take the door's tongue. The cut stops 0.01 inside the skin's inner
    plane, so the walls and the post that meet the skin keep their faces.
    The opening's ceiling (its model-bottom edge, the skin over the pelvis)
    spans all of it: slicer support, on the strip under the deck rail."""
    z0, z1 = V.DOOR_Z
    yi, yo = tin - 0.01, V.HOUSING_HW + 1.0
    cut = wedge_z([(_bevel_x(yi), yi), (_bevel_x(yo), yo), (V.DOOR_X[1], yo), (V.DOOR_X[1], yi)], z0, z1)
    enf = (V.DOOR_X[0] + ENF_INSET, V.DOOR_X[1] - ENF_INSET, tin + ENF_INSET,
           _skin_outer_y(z0) - ENF_INSET, z0 - ENF_INTO, z1)
    return cut, enf


def _door_post():
    """(add, cut): the post on the front inner wall's aft face that takes the
    door's screw, from under the deck down past the screw, beside the pack's
    front face; and the screw's pilot."""
    o = 0.3
    x0, x1 = V.DOOR_POST_X
    add = box(x0, x1, V.DOOR_POST_Y0, tin + o, V.DOOR_POST_BOT_Z, zd + o)
    sx, sz = V.DOOR_SCREW
    cut = cyl_y(D.M25_TAP / 2, tin + 0.1, tin - DOOR_PILOT, sx, sz)
    return add, cut


# the -y stop's two blocks, x: clear of the outboard stator screws' drivers
# (x = -8.30, -32.75, a 7 mm driver each) -- they are driven through here
STOP_X = ((V.BATT_X[0] + 6.25, -13.0), (-3.5, V.BATT_X[1] - 0.25))


def _pack_stop():
    """The -y end stop: two blocks under the deck rail, their faces 0.5 off
    the pack's -y end, between the outboard stator screws' driver paths; each
    top over the deck aperture is a 50 deg slope (it hangs off the rail where
    the aperture leaves no deck over it)."""
    o = 0.3
    ya = -V.DECK_APER_HY
    prof = [(-(tin + o), zd + o), (ya, zd + o), (ya, zd - 1.0),
            (-V.DOOR_STOP_Y, zd - 1.0 - STOP_SLOPE * (-ya - V.DOOR_STOP_Y)), (-V.DOOR_STOP_Y, STOP_BOT_Z),
            (-(tin + o), STOP_BOT_Z)]
    stop = None
    for x0, x1 in STOP_X:
        b = wedge_x(prof, x0, x1)
        stop = b if stop is None else stop + b
    return stop


DOOR_PILOT = 5.5                                 # the screw's pilot in the post


def battery_door_v8():
    """THE BATTERY DOOR: a plate the skin's thickness in the opening (0.2 all
    round), its aft edge the tab -- bevelled to the jamb and carried on
    inboard of the skin by DOOR_TONGUE_T -- its front edge the screw's
    countersunk hole, and two ribs on its inner face along the pack's top and
    bottom edges (the lead leaves the pack's end between them)."""
    c = V.DOOR_CLR
    z0, z1 = V.DOOR_Z[0] + c, V.DOOR_Z[1] - c
    yi = V.DOOR_IN_Y
    xb_out = _bevel_x(DOOR_OUT_Y) + BEVEL_OFF
    xb_tip = _bevel_x(DOOR_TONGUE_Y) + BEVEL_OFF
    prof = [(xb_out, DOOR_OUT_Y), (V.DOOR_X[1] - c, DOOR_OUT_Y), (V.DOOR_X[1] - c, yi),
            (DOOR_STEP_X, yi), (DOOR_STEP_X, DOOR_TONGUE_Y), (xb_tip, DOOR_TONGUE_Y)]
    d = wedge_z(prof, z0, z1)
    rx0, rx1 = V.BATT_X[0] + 0.75, V.BATT_X[1] - 2.5       # front end: clear of the post as the door swings
    for rz0 in (V.BATT_Z[0] + 0.2, V.BATT_Z[1] - 0.2 - DOOR_RIB_T):
        d += box(rx0, rx1, V.DOOR_STOP_Y, yi + 0.3, rz0, rz0 + DOOR_RIB_T)
    sx, sz = V.DOOR_SCREW
    d -= cyl_y(D.CASE_SCREW_CLEAR / 2, yi - 1, DOOR_OUT_Y + 1, sx, sz)
    d -= csk_y(sx, sz, DOOR_OUT_Y, +1)
    return d


def _windows():
    """Every window: a list of (cut, enforcer box or None), all flat-topped:
    - cable passes in the inner walls: over the Pi's top edge from the
      battery layer, the battery layer's ends into both bays (the pack's lead
      to J1 at +y, short of the door's post in the front wall), and aft of
      the pack to the power board;
    - the +y skin over the front bay: the Pi's USB/Ethernet end and the
      General Driver's service edge (XH inlet, USB-C);
    - mass: below the battery floor in both inner walls, the skins over the
      pelvis, the battery layer (-y: the door is the +y side) and both bays,
      all clear of the door's opening and the -y pack stop.
    The door's opening is not here: torso_v8 cuts it (door_opening)."""
    out = []
    fw, aw = V.FWD_WALL_X, V.AFT_WALL_X
    for yc in (-18.0, -6.0, 6.0, 18.0):
        out.append(_window_x(yc - 4.0, yc + 4.0, -16.0, zd - 1.0, fw))
    zl = zc + LEDGE_T + LEDGE_D + 1.0
    for s in (1, -1):
        for wall, y_end in ((fw, V.DOOR_POST_Y0 - 0.5 if s > 0 else tin - LEDGE_D - 0.8),
                            (aw, tin - LEDGE_D - 0.8)):
            y0, y1 = sorted((s * 38.0, s * y_end))
            out.append(_window_x(y0, y1, zl, zd - 1.5, wall))
        y0, y1 = sorted((s * 26.5, s * 36.0))
        out.append(_window_x(y0, y1, -30.0, -12.5, aw))
    for wall, ends in ((fw, (38.5, 51.0)), (aw, (38.5, 41.5))):
        for y0, y1 in ((-8.0, 8.0), (20.0, 32.0), ends):
            for s in ((1,) if y0 < 0 else (1, -1)):
                a, b = sorted((s * y0, s * y1))
                out.append(_window_x(a, b, -52.0, zc - 2.0, wall))
    # +y skin over the front bay: the Pi's USB/Ethernet end, the GD's service edge
    out.append(_window_y(V.PI_PCB_X1 - 0.5, V.PI_COMP_X + 0.5, V.PI_BOT_Z + 3.0, V.PI_TOP_Z - 1.0, +1))
    out.append(_window_y(V.GD_PCB_X1 - 1.0, V.GD_COMP_X, -63.0, -22.0, +1))
    # skins: in 14 mm bays with 5 mm posts between; on +y they stop 4 mm under
    # the door's opening, on -y under the pack stop
    under_door = V.DOOR_Z[0] - 4.0
    for s in (1, -1):
        bays = [(V.AFT_WALL_X[1] + 3.0, V.FWD_WALL_X[0] - 3.0, -66.0, under_door if s > 0 else zc - 2.0),
                (V.PWRB_AFT_X + COVER_BLOCK[0] + 1.0, V.AFT_WALL_X[0] - 1.0, -56.0, under_door if s > 0 else -16.0)]
        if s < 0:
            bays += [(V.AFT_WALL_X[1] + 3.0, V.FWD_WALL_X[0] - 3.0, zl, STOP_BOT_Z - 1.0),
                     (V.FWD_WALL_X[1] + 2.0, V.GD_FRONT_X - COVER_BLOCK[0] - 1.0, -62.0, -18.0)]
        for xa, xb, z0, z1 in bays:
            n = max(1, int((xb - xa + 5.0) // 19.0))
            w = (xb - xa - 5.0 * (n - 1)) / n
            for k in range(n):
                u0 = xa + k * (w + 5.0)
                if z1 - z0 >= 4.0:
                    out.append(_window_y(u0, u0 + w, z0, z1, s))
    return out


def support_enforcers():
    """The torso's support-enforcer boxes, model frame, rounded as
    cad/v6/print_settings.json carries them: every window wider than a
    bridge, and the door's opening."""
    boxes = [e for _, e in _windows() if e is not None] + [door_opening()[1]]
    return [[round(v, 2) for v in e] for e in boxes]


def _deck_pockets():
    """2.4 mm off the deck's underside (it prints on top of the deck slab, so
    the pockets open upward) over both bays, in bays with 4 mm ribs, 3 mm
    clear of every wall, opening and standoff."""
    depth = D.DECK_T - 2.6
    zp = (zd - 0.5, zd + depth)
    cut = None
    regions = [(STACK_OPEN_X[1] + 3.0, V.GD_FRONT_X - COVER_BLOCK[0] - 3.0, -(tin - 3.0), tin - 3.0),
               (V.FWD_WALL_X[1] + 3.0, STACK_OPEN_X[0] - 3.0, STACK_OPEN_HY + 3.0, tin - 3.0),
               (V.FWD_WALL_X[1] + 3.0, STACK_OPEN_X[0] - 3.0, -(tin - 3.0), -(STACK_OPEN_HY + 3.0)),
               (V.PWRB_AFT_X + COVER_BLOCK[0] + 3.0, V.PWRB_PCB_X1 - 3.0, -(tin - 3.0), tin - 3.0)]
    for x0, x1, y0, y1 in regions:
        if x1 - x0 < 6.0:
            continue
        n = max(1, int((y1 - y0 + 4.0) // 24.0))
        w = (y1 - y0 - 4.0 * (n - 1)) / n
        for k in range(n):
            u0 = y0 + k * (w + 4.0)
            c = box(x0, x1, u0, u0 + w, *zp)
            cut = c if cut is None else cut + c
    return cut


# the deck opening over the gap between the Pi and the General Driver: the
# camera's ribbon (the Pi's CSI connector is on its top edge, under the HAT)
# and the boards' top-edge leads leave the bay through it
STACK_OPEN_X = (V.PI_PCB_X1 + 0.5, V.GD_PCB_X0 - V.GD_UNDER - 0.5)
STACK_OPEN_HY = 30.0


def _cover_screw_holes():
    """M2.5 x 8 flat heads in through each skin into the covers' corner
    blocks; countersunk in the skin's outer face."""
    cut = None
    for _, x_in, _, sx in COVERS:
        sxx = _cover_screw_x(x_in, sx)
        for yb, zb_ in _cover_blocks_yz():
            s = 1 if yb > 0 else -1
            yo = _skin_outer_y(zb_)
            c = cyl_y(D.CASE_SCREW_CLEAR / 2, s * (tin - 0.5), s * (yo + 1), sxx, zb_)
            c += csk_y(sxx, zb_, s * yo, s)
            cut = c if cut is None else cut + c
    return cut


def _lead_in(x0, x1, hy, c=1.5):
    """45 deg chamfer round a deck opening's top edge (on the bed, it prints as
    a 45 deg overhang)."""
    xm = (x0 + x1) / 2
    top = Pos(xm, 0, 0.01) * Rectangle(x1 - x0 + 2 * c, 2 * hy + 2 * c)
    bot = Pos(xm, 0, -c) * Rectangle(x1 - x0, 2 * hy)
    return loft([top.faces()[0], bot.faces()[0]])


def torso_v8(arm_mounts=False, audit=False):
    """THE TORSO. One solid: the envelope (skins + deck), hollowed to the skins'
    inner faces end to end (the covers close both bays), then the two inner
    walls, ribs, ledge, fingers, bosses, the door's post and the -y pack stop
    added, then every opening, window and pilot cut. arm_mounts adds the
    shoulder girdle's ten deck pilots (the robot's build: parts_v6 and
    assembly_v6 pass it whenever the arms are on). audit leaves the windows
    that print with support (and the door's opening) uncut: the support
    fills them while they print."""
    t = _envelope()
    t -= box(V.HOUSING_X[0] - 1, V.HOUSING_X[1] + 1, -tin, tin, zt - 1, zd)
    t += _inner_walls()
    t += _ribs_and_ledges()
    slots, barbs = _snap_fingers()
    t -= slots
    t += barbs
    add, cut = _board_bosses()
    t += add
    t -= cut
    post, pilot = _door_post()
    t += post
    t += _pack_stop()
    # the bearing housing's rear-outboard lugs, through the aft wall
    for s in (1, -1):
        y0, y1 = sorted((s * (LUG_Y - V.YAW_HOUSING_BOSS_R - 0.3), s * (LUG_Y + V.YAW_HOUSING_BOSS_R + 0.3)))
        t -= box(LUG_X - V.YAW_HOUSING_BOSS_R - 0.3, V.AFT_WALL_X[1] + 0.5, y0, y1, zt - 1, LUG_TOP_Z + 0.3)
    # ---- deck
    # battery aperture (the neck floor drops into it; dimensions_v6 DECK_APER_*)
    t -= box(V.DECK_APER_X[0], V.DECK_APER_X[1], -V.DECK_APER_HY, V.DECK_APER_HY, zd - 1, 1)
    t -= box(*STACK_OPEN_X, -STACK_OPEN_HY, STACK_OPEN_HY, zd - 1, 1)
    t -= _lead_in(*STACK_OPEN_X, STACK_OPEN_HY)
    if arm_mounts:
        for hx, hy in V.GIRDLE_PILOT_XY:
            t -= cyl_z(V.ARM_PILOT_D / 2, -V.ARM_PILOT_DEPTH, 0.5, hx, hy)
    for c, enf in _windows():
        if not (audit and enf is not None):
            t -= c
    if not audit:
        t -= door_opening()[0]
    t -= pilot
    t -= _deck_pockets()
    t -= _cover_screw_holes()
    return t


# ============================================================================
# COVERS
# ============================================================================
def _cover(x_in, x_out, sx):
    """A bay's end wall: a flat plate between the skins' ends, under the deck,
    with a corner block in each corner on its inner face (standing sx) that
    the skin screws thread into, and vent slots."""
    hy = tin - COVER_CLR
    ztop = zd - COVER_CLR
    x0, x1 = sorted((x_in, x_out))
    c = box(x0, x1, -hy, hy, zt, ztop)
    bx, by, bz = COVER_BLOCK
    sxx = _cover_screw_x(x_in, sx)
    for yb, zb_ in _cover_blocks_yz():
        bxa, bxb = sorted((x_in - sx * 0.3, x_in + sx * bx))
        c += box(bxa, bxb, yb - by / 2, yb + by / 2, zb_ - bz / 2, zb_ + bz / 2)
        s = 1 if yb > 0 else -1
        c -= cyl_y(D.M25_TAP / 2, s * (hy + 0.1), s * (hy - (COVER_SCREW_L - 2.0)), sxx, zb_)
    for k in range(-5, 6):
        yv = k * 8.0
        c -= box(x0 - 1, x1 + 1, yv - 2.0, yv + 2.0, zt + 12.0, ztop - 12.0)
    return c


def front_cover_v8():
    """The front bay's cover, over the General Driver's parts face."""
    return _cover(*[(xi, xo, sx) for n, xi, xo, sx in COVERS if n == "front_cover_v8"][0])


def power_cover_v8():
    """The power bay's cover, over the power board's parts face."""
    return _cover(*[(xi, xo, sx) for n, xi, xo, sx in COVERS if n == "power_cover_v8"][0])


# ============================================================================
# mocks (pelvis frame)
# ============================================================================
def mock_pack():
    return box(*V.BATT_X, -V.BATT_HY, V.BATT_HY, *V.BATT_Z)


def mock_gd():
    """The General Driver's PCB."""
    return box(V.GD_PCB_X0, V.GD_PCB_X1, *V.GD_Y, V.GD_BOT_Z, V.GD_TOP_Z)


def mock_pi4():
    """The Pi 4B's PCB."""
    return box(V.PI_PCB_X0, V.PI_PCB_X1, *V.PI_Y, V.PI_BOT_Z, V.PI_TOP_Z)


def _gd_u_z(u):
    """A distance u along the General Driver's service edge, from its bus-edge
    corner (D.BOARD_GD_U_*), as pelvis z: the bus edge is up."""
    return V.GD_TOP_Z - u


def _hole_keepout(x0, x1, y, z, r=3.5):
    """What a board keeps clear on its solder side round a mounting hole, so
    its standoff boss can seat: the boss (r) and the 45 deg roof it prints
    with (_roofed_boss_x), plus 0.3."""
    th = math.radians(D.BF_BOSS_ROOF_DEG)
    return box(x0, x1, y - r - 0.3, y + r + 0.3, z - r - 0.3, z + r / math.cos(th) + 0.3)


def mock_gd_plugs():
    """The plugs on the General Driver's service edge (+y), mated: the XH power
    inlet and the two USB-C (D.XH2_PLUG, D.USBC_PLUG), each mating along +y."""
    y0 = V.GD_Y[1]
    m = None
    for (u0, u1), (along, thick, out), mid in ((D.BOARD_GD_U_INLET, D.XH2_PLUG, D.XH2_PLUG[1] / 2),
                                              (D.BOARD_GD_U_USB, D.USBC_PLUG, V.USBC_SHELL_H / 2),
                                              (D.BOARD_GD_U_LIDAR, D.USBC_PLUG, V.USBC_SHELL_H / 2)):
        zc_ = _gd_u_z((u0 + u1) / 2)
        xc = V.GD_PCB_X1 + mid
        b = box(xc - thick / 2, xc + thick / 2, y0, y0 + out, zc_ - along / 2, zc_ + along / 2)
        m = b if m is None else m + b
    return m


def mock_stack(plugs=False):
    """The Pi + General Driver stack as it sits: both PCBs, the Pi's solder side
    (less round its holes), its parts under the HAT, its USB/Ethernet end,
    the riser along the bottom edge, the GD's underside and parts side, and
    (plugs) the mated service-edge plugs."""
    hy0, hy1 = V.GD_Y
    m = mock_pi4() + mock_gd()
    sold = box(V.PI_PCB_X0 - V.PI_COMP_BOT, V.PI_PCB_X0, *V.PI_Y, V.PI_BOT_Z, V.PI_TOP_Z)
    for sy in V.PI_SCREW_Y:
        for sz in V.PI_SCREW_Z:
            sold -= _hole_keepout(V.PI_PCB_X0 - V.PI_COMP_BOT - 1, V.PI_PCB_X0 + 1, sy, sz)
    m += sold
    m += box(V.PI_PCB_X1, V.PI_PCB_X1 + V.PI_UNDER_HAT, V.PI_Y[0], hy1, V.PI_BOT_Z, V.PI_TOP_Z)
    m += box(V.PI_PCB_X1, V.PI_COMP_X, hy1, V.PI_Y[1] + V.PI_USB_PROUD, V.PI_BOT_Z + 1.0, V.PI_TOP_Z - 1.0)
    m += box(V.PI_PCB_X1, V.GD_PCB_X0, -26.0, 26.0, V.PI_BOT_Z, V.PI_BOT_Z + 6.0)
    m += box(V.GD_PCB_X0 - V.GD_UNDER, V.GD_PCB_X0, hy0, hy1, V.GD_BOT_Z, V.GD_TOP_Z)
    m += box(V.GD_PCB_X1, V.GD_COMP_X, hy0, hy1, V.GD_BOT_Z, V.GD_TOP_Z)
    if plugs:
        m += mock_gd_plugs()
    return m


# The power board's parts taller than 2.5 mm, from pcb/power-board (branch
# design/power-circuit): (ref, courtyard bx0, bx1, by0, by1, height mated).
# Board coords: origin at the outline's lower-left corner seen from the parts
# side, bx right, by up. Heights: datasheets in docs/datasheets/power-board/
# (XT60PW shell, JST XH/PH mated, Keystone 3568 + MINI fuse) and Panasonic's
# EEU-FR can; J4-J8's Molex 5267 + 5264 mated height is NOT verified (15
# assumed). J1's mated XT60 plug runs past the left edge (PWRB_XT60_PLUG).
PWRB_PARTS = (
    ("J1", -0.75, 18.65, 33.65, 50.15, 8.4),
    ("J2", 19.05, 32.45, 32.60, 39.35, 9.8),
    ("J3", 66.55, 74.95, 57.20, 63.95, 9.8),
    ("J4-J8", 85.55, 96.45, 9.59, 41.10, 15.0),
    ("J9-J12", 15.55, 66.45, 0.70, 6.20, 8.0),
    ("F1", 18.91, 35.41, 41.88, 49.12, 16.3),
    ("F2", 49.71, 66.21, 56.68, 63.92, 16.3),
    ("C21", 78.25, 88.75, 44.25, 54.75, 22.0),
)
PWRB_SMD_H = 2.5
PWRB_XT60_PLUG = 12.0          # the mated female + solder cups past the board's left edge (estimate)


def _pwrb_yz(bx, by):
    """Board coords -> pelvis-local (y, z): parts face aft, left edge at +y."""
    return V.PWRB_Y[1] - bx, V.PWRB_BOT_Z + by


def mock_pwrb(plug=True, leads=True):
    """The power board: PCB, its SMD layer, each tall part's courtyard at its
    mated height, J1's plug past the left edge, and (leads) the THT leads'
    keep-out under it, less round its holes."""
    pcb = box(V.PWRB_PCB_X0, V.PWRB_PCB_X1, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
    m = pcb + box(V.PWRB_PCB_X0 - PWRB_SMD_H, V.PWRB_PCB_X0, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
    for ref, bx0, bx1, by0, by1, h in PWRB_PARTS:
        ya, za = _pwrb_yz(bx0, by0)
        yb, zb_ = _pwrb_yz(bx1, by1)
        m += box(V.PWRB_PCB_X0 - h, V.PWRB_PCB_X0, min(ya, yb), max(ya, yb), za, zb_)
        if ref == "J1" and plug:
            m += box(V.PWRB_PCB_X0 - h, V.PWRB_PCB_X0, V.PWRB_Y[1], V.PWRB_Y[1] + PWRB_XT60_PLUG, za, zb_)
    if leads:
        under = box(V.PWRB_PCB_X1, V.PWRB_PCB_X1 + V.PWRB_UNDER, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
        for hy, hz in V.PWRB_HOLES_YZ:
            under -= _hole_keepout(V.PWRB_PCB_X1 - 1, V.PWRB_PCB_X1 + V.PWRB_UNDER + 1, hy, hz)
        m += under
    return m


def mock_j1_plug():
    """The pack's XT60 plug, mated in J1 past the board's +y edge."""
    ref, bx0, bx1, by0, by1, h = PWRB_PARTS[0]
    assert ref == "J1"
    _, za = _pwrb_yz(bx0, by0)
    _, zb_ = _pwrb_yz(bx1, by1)
    return box(V.PWRB_PCB_X0 - h, V.PWRB_PCB_X0, V.PWRB_Y[1], V.PWRB_Y[1] + PWRB_XT60_PLUG, za, zb_)


def mock_neck_servo():
    return Pos(V.NECK_X, 0, V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()


# ============================================================================
# fasteners
# ============================================================================
MIN_BITE = 2.0          # thread a self-tapper needs in PETG to hold


def SCREWS():
    """Every fastener in the four parts and the board stack, pelvis-local:
    {name, part, kind, pos, axis, length, clamp, pilot}. `axis` points head ->
    tip; `pos` is the head's bearing point; `clamp` is what the screw passes
    before its thread bites (the clamped part plus any air); `pilot` is the
    depth it may bite without running out of its pilot or out of the far
    face (None: it threads into a metal standoff). The bearing housing's
    five are yaw_bearing_housing.SCREWS()."""
    s = []
    for by, side in ((V.HIP_SEP / 2, "L"), (-V.HIP_SEP / 2, "R")):
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for sgn in (1, -1):
                # through the ceiling into the idler-face hole, 3.0 deep (vendor STEP)
                s.append(dict(name=f"yaw_stator_{side}_{xrow:.0f}_{sgn:+d}", part="pelvis_v8",
                              kind="M2.5x5 self-tap, flat head", axis=(0, 0, -1),
                              pos=(-xrow, by + sgn * D.CASE_HOLE_LAT, zc), length=5.0,
                              clamp=D.WALL, pilot=3.0))
    for sy in V.PI_SCREW_Y:
        for sz in V.PI_SCREW_Z:
            s.append(dict(name=f"pi_standoff_{sy:+.0f}_{sz:+.0f}", part="torso_v8",
                          kind=f"M2.5 x {V.HAT_GAP:.0f} male-female standoff, {STACK_MALE:.0f} mm male: the Pi into its boss",
                          axis=(-1, 0, 0), pos=(V.PI_PCB_X1, sy, sz), length=STACK_MALE,
                          clamp=V.PI_PCB_T, pilot=PI_PILOT))
            s.append(dict(name=f"gd_{sy:+.0f}_{sz:+.0f}", part="stack",
                          kind="M2.5x6 pan head: the General Driver into the Pi's standoff",
                          axis=(-1, 0, 0), pos=(V.GD_PCB_X1, sy, sz), length=6.0,
                          clamp=D.BR_PCB_T, pilot=None))
    for hy, hz in V.PWRB_HOLES_YZ:
        s.append(dict(name=f"pwrb_{hy:+.0f}_{hz:+.0f}", part="torso_v8",
                      kind="M2.5x6 self-tap, pan head (standoff boss)", axis=(1, 0, 0),
                      pos=(V.PWRB_PCB_X0, hy, hz), length=PWRB_SCREW_L, clamp=V.PWRB_T, pilot=PWRB_PILOT))
    sx, sz = V.DOOR_SCREW
    s.append(dict(name="battery_door", part="battery_door_v8",
                  kind="M2.5x8 self-tap, flat head (through the door into its post)", axis=(0, -1, 0),
                  pos=(sx, DOOR_OUT_Y, sz), length=V.DOOR_SCREW_L, clamp=DOOR_OUT_Y - tin, pilot=DOOR_PILOT))
    for name, x_in, _, sx in COVERS:
        sxx = _cover_screw_x(x_in, sx)
        for yb, zb_ in _cover_blocks_yz():
            sg = 1 if yb > 0 else -1
            s.append(dict(name=f"{name.split('_')[0]}_cover_{yb:+.0f}_{zb_:+.0f}", part=name,
                          kind="M2.5x8 self-tap, flat head (through the skin into the cover)",
                          axis=(0, -sg, 0), pos=(sxx, sg * _skin_outer_y(zb_), zb_),
                          length=COVER_SCREW_L, clamp=_skin_outer_y(zb_) - tin + COVER_CLR,
                          pilot=COVER_SCREW_L - 2.0))
    return s


def check_screw_bite(verbose=True):
    """Each screw's thread (length - clamp) must bite at least MIN_BITE and
    stay inside its pilot (so a tip never runs past a far face: toward the
    pelvis web 0.2 mm away, or the pack)."""
    ok = True
    for sc in SCREWS():
        if sc["pilot"] is None:
            continue
        bite = sc["length"] - sc["clamp"]
        good = MIN_BITE <= bite <= sc["pilot"] + 1e-9
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  {sc['name']:26s} {sc['kind'][:16]:16s} bite {bite:5.2f} mm "
                  f"of {sc['pilot']:5.2f} (>= {MIN_BITE})")
    return ok


SLIM_DRIVER_D = 5.0     # the shank that still reaches the outboard stator screws


def _driver_hit(sc, solid, d):
    ax = np.array(sc["axis"], float)
    head = np.array(sc["pos"], float)
    start = head - ax * 1.0
    i = int(np.argmax(np.abs(ax)))
    a0, a1 = start[i], start[i] - np.sign(ax[i]) * 140.0
    if i == 2:
        cyl = cyl_z(d / 2, a0, a1, head[0], head[1])
    elif i == 0:
        cyl = cyl_x(d / 2, a0, a1, head[1], head[2])
    else:
        cyl = cyl_y(d / 2, a0, a1, head[0], head[2])
    try:
        return (cyl & solid).volume
    except Exception:  # noqa: BLE001
        return 0.0


def check_screw_access(pieces, verbose=True):
    """A straight 7 mm (ACCESS_D) driver from each head back out along -axis
    must miss the parts in place when it is driven: the cover and door
    screws with everything on; the stator screws with the covers on and the
    pack and the door out (servicing a yaw servo -- on the bench they go in
    before the pelvis goes into the torso); the boards' screws with their
    bay's cover off (the Pi's standoffs before the General
    Driver goes on). Where 7 mm fails, the line says whether a SLIM_DRIVER_D
    shank clears."""
    every = (pieces["pelvis_v8"] + pieces["torso_v8"] + pieces["power_cover_v8"]
             + pieces["front_cover_v8"] + pieces["battery_door_v8"])
    no_door = pieces["pelvis_v8"] + pieces["torso_v8"] + pieces["power_cover_v8"] + pieces["front_cover_v8"]
    open_bays = pieces["pelvis_v8"] + pieces["torso_v8"]
    ok = True
    for sc in SCREWS():
        # a yaw servo is serviced with the pack out, so the door is off too
        solid = (open_bays if sc["part"] in ("torso_v8", "stack")
                 else no_door if sc["part"] == "pelvis_v8" else every)
        hit = _driver_hit(sc, solid, D.ACCESS_D)
        clear = hit < 5.0
        note = ""
        if not clear:
            slim = _driver_hit(sc, solid, SLIM_DRIVER_D)
            note = f"  ({SLIM_DRIVER_D:.0f} mm shank: {slim:.1f} mm3, {'clears' if slim < 5.0 else 'blocked'})"
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  {sc['name']:26s} {sc['kind'][:58]:58s} overlap {hit:6.1f} mm3{note}")
    return ok


# ============================================================================
# fit checks
# ============================================================================
def _sweep(mover, fixed, path, name, tol=0.5, n=40):
    """Move `mover` along `path` (list of (dx, dy, dz) offsets) and report the
    worst intersection with `fixed`."""
    worst, at = 0.0, None
    for k in range(n):
        f = k / (n - 1)
        d = [a + (b - a) * f for a, b in zip(*path)]
        try:
            v = (Pos(*d) * mover & fixed).volume
        except Exception:  # noqa: BLE001
            v = 0.0
        if v > worst:
            worst, at = v, d
    ok = worst < tol
    print(f"  {name}: worst intersection {worst:.2f} mm3"
          f"{'' if at is None else f' at offset ({at[0]:.1f}, {at[1]:.1f}, {at[2]:.1f})'} "
          f"({'PASS' if ok else 'FAIL'})")
    return ok


DOOR_SWING_DEG = 40.0                  # the tongue's tip clears the jamb's bevel lifting out


def door_swing_steps(door, n=26):
    """The door coming off, as (deg, lift, moved door): the screw out, the
    front end swung out about the bevel's outer corner (the tongue moves
    inboard, into the power bay, as the bevel faces part) to DOOR_SWING_DEG,
    then the door lifted straight out +y."""
    from build123d import Location
    px, py = _bevel_x(DOOR_OUT_Y) + BEVEL_OFF, DOOR_OUT_Y
    steps = [(DOOR_SWING_DEG * k / (n - 1), 0.0) for k in range(n)]
    steps += [(DOOR_SWING_DEG, 30.0 * k / 12) for k in range(1, 13)]
    return [(deg, lift, Pos(px, py + lift, 0) * Location((0, 0, 0), (0, 0, 1), deg) * Pos(-px, -py, 0) * door)
            for deg, lift in steps]


def door_driver():
    """The door screw's driver path: a 7 mm (D.ACCESS_D) cylinder straight out
    +y from the head."""
    sx, sz = V.DOOR_SCREW
    return cyl_y(D.ACCESS_D / 2, DOOR_OUT_Y + 1.0, DOOR_OUT_Y + 141.0, sx, sz)


def pack_slide_steps(n=57):
    """The pack sliding out +y through the door's opening until it is clear of
    the skin: (offset, moved pack)."""
    out = []
    for k in range(n):
        dy = (2 * V.BATT_HY + 16.0) * k / (n - 1)
        out.append((dy, Pos(0, dy, 0) * mock_pack()))
    return out


def _door_swing(door, fixed, tol=0.5):
    """The door coming off (door_swing_steps): worst intersection with `fixed`."""
    worst, at = 0.0, None
    for deg, lift, moved in door_swing_steps(door):
        try:
            v = (moved & fixed).volume
        except Exception:  # noqa: BLE001
            v = 0.0
        if v > worst:
            worst, at = v, (deg, lift)
    ok = worst < tol
    print(f"  battery door swung open about its tongue, then out: worst intersection {worst:.2f} mm3"
          f"{'' if at is None else f' at {at[0]:.1f} deg, {at[1]:.1f} mm out'} ({'PASS' if ok else 'FAIL'})")
    return ok


def check_fits(pieces):
    """The installs and the seated fit, each a sweep or an intersection:
    - the Pi + General Driver stack goes in from the front onto its four
      standoffs (front cover off), and the power board from aft (aft cover
      off), each with its parts and its solder side;
    - the pelvis goes up into the torso from below -- the barbs are left out,
      they ride the webs by design (SNAP_BARB);
    - the battery swap: the door swung off about its tongue (the plug still
      in J1), J1's plug pulled off +y, and the pack slid out +y through the
      opening, each clear of the torso and the pelvis;
    - seated: pelvis, covers, door, pack, stack and power board vs the
      printed parts; the General Driver's mated service plugs vs the Pi's
      USB/Ethernet end (HAT_GAP) and vs the torso; all < 0.5 mm3."""
    pel, tor = pieces["pelvis_v8"], pieces["torso_v8"]
    fc, pc = pieces["front_cover_v8"], pieces["power_cover_v8"]
    door = pieces["battery_door_v8"]
    ok = True
    stack, pwrb = mock_stack(), mock_pwrb(plug=False)
    ok &= _sweep(stack, tor, ((40.0, 0, 0), (0, 0, 0)), "Pi + General Driver stack in from the front")
    ok &= _sweep(pwrb, tor, ((-40.0, 0, 0), (0, 0, 0)), "power board in from aft")
    _, barbs = _snap_fingers()
    ok &= _sweep(pel, tor - barbs, ((0, 0, -45.0), (0, 0, 0)), "pelvis up into the torso (barbs aside)", n=46)
    ok &= _door_swing(door, tor + pel + mock_pack() + mock_pwrb())
    ok &= _sweep(mock_j1_plug(), tor + pel, ((0, 0, 0), (0, 22.0, 0)), "J1's plug pulled off and out through the door")
    ok &= _sweep(mock_pack(), tor + pel, ((0, 0, 0), (0, 2 * V.BATT_HY + 16.0, 0)),
                 "pack slid out through the door (door off)", n=57)
    shell = tor + fc + pc + door
    pi_jacks = box(V.PI_PCB_X1, V.PI_COMP_X, V.GD_Y[1], V.PI_Y[1] + V.PI_USB_PROUD, V.PI_BOT_Z, V.PI_TOP_Z)
    seated = [("pelvis vs torso", pel, tor), ("front cover vs torso", fc, tor),
              ("power cover vs torso", pc, tor), ("battery door vs torso", door, tor),
              ("battery door vs pelvis+pack+power board", door, pel + mock_pack() + mock_pwrb()),
              ("pack vs pelvis+torso", mock_pack(), pel + tor),
              ("stack vs torso+covers", stack, shell), ("power board (plug in) vs torso+covers", mock_pwrb(), shell),
              ("GD service plugs vs the Pi's USB/Ethernet end", mock_gd_plugs(), pi_jacks),
              ("GD service plugs vs torso+covers", mock_gd_plugs(), shell)]
    for nm, a, b in seated:
        v = (a & b).volume
        good = v < 0.5
        ok &= good
        print(f"  seated {nm}: {v:.2f} mm3 ({'PASS' if good else 'FAIL'})")
    v = (pel & barbs).volume
    print(f"  barbs vs seated pelvis: {v:.2f} mm3 ({'PASS' if v < 0.05 else 'FAIL'})")
    ok &= v < 0.05
    return ok


def snap_strain():
    """Peak bending strain of a finger deflected by its barb: a cantilever from
    SNAP_ROOT_Z to the catch, t = the wall, d = the barb's interference."""
    L = SNAP_ROOT_Z - SNAP_CATCH_Z
    return 1.5 * W * SNAP_BARB / L ** 2


# ============================================================================
# printability (check_printability, NO support waiver: the torso is audited
# with the windows that print with support filled, torso_v8(audit=True))
# ============================================================================
def _expected(name, kind, desc):
    """Findings the audit may raise that are not failures, and why. Anything
    else fails."""
    return None


def audit(name, stl_path):
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT[name] = (np.array(PRINT_ORIENT[name]), "torso_v8 piece, no supports")
    CP.PRINT_STL[name] = os.path.basename(stl_path)
    CP.SUPPORTED.pop(name, None)
    fails, ok = [], []
    for kind, a, desc in CP.audit(name):
        why = _expected(name, kind, desc)
        (ok if why else fails).append((kind, a, desc, why))
    return fails, ok


def mass_g(s):
    return s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def build(arm_mounts=True):
    return {"pelvis_v8": pelvis_v8(), "torso_v8": torso_v8(arm_mounts=arm_mounts),
            "front_cover_v8": front_cover_v8(), "power_cover_v8": power_cover_v8(),
            "battery_door_v8": battery_door_v8()}


def in_print_orient(solid, name):
    from build123d import Matrix
    r = PRINT_ORIENT[name]
    s = solid.transform_geometry(Matrix([[*r[0], 0], [*r[1], 0], [*r[2], 0], [0, 0, 0, 1]]))
    bb = s.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s


def _render_strip(shots, png_path, px=640):
    """MuJoCo offscreen, one frame per shot: shot = (list of (solid, rgb),
    azimuth, elevation, distance factor on the scene's half diagonal)."""
    os.environ["MUJOCO_GL"] = "cgl" if sys.platform == "darwin" else os.environ.get("MUJOCO_GL", "egl")
    import tempfile
    import imageio.v2 as imageio
    import mujoco
    frames = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, (parts, az, el, k) in enumerate(shots):
            assets, geoms = [], []
            for j, (s, rgb) in enumerate(parts):
                stl = os.path.join(tmp, f"{i}_{j}.stl")
                export_stl(s, stl)
                assets.append(f'<mesh name="m{j}" file="{stl}" scale="0.001 0.001 0.001"/>')
                geoms.append(f'<geom type="mesh" mesh="m{j}" rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"/>')
            model = mujoco.MjModel.from_xml_string(f"""
            <mujoco>
              <asset>{''.join(assets)}</asset>
              <visual><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
                <global offwidth="{px}" offheight="{px}"/></visual>
              <worldbody>{''.join(geoms)}</worldbody>
            </mujoco>""")
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            verts = np.concatenate([model.mesh_vert[model.mesh_vertadr[n]:model.mesh_vertadr[n] + model.mesh_vertnum[n]]
                                    + model.mesh_pos[n] for n in range(model.nmesh)])
            ren = mujoco.Renderer(model, px, px)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = (verts.min(0) + verts.max(0)) / 2
            cam.distance = k * np.linalg.norm(verts.max(0) - verts.min(0)) / 2
            cam.azimuth, cam.elevation = az, el
            ren.update_scene(data, cam)
            frames.append(ren.render())
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


C_PELVIS, C_TORSO, C_COVER, C_BOARD, C_PACK = ((0.45, 0.62, 0.90), (0.82, 0.84, 0.87), (0.90, 0.70, 0.40),
                                               (0.30, 0.62, 0.38), (0.25, 0.25, 0.28))


def render(pieces, out_dir):
    """Two strips in out_dir:
    torso_v8.png           the four pieces as they lie on the bed;
    torso_v8_assembly.png  exploded (the pelvis below, the covers and the
                           door off, the stack, the power board and the
                           pack in place), assembled from below, the door
                           off with the pack half out, and a section at y = +14
                           through the snap fingers (the -y half, seen
                           from +y)."""
    pel, tor = pieces["pelvis_v8"], pieces["torso_v8"]
    fc, pc = pieces["front_cover_v8"], pieces["power_cover_v8"]
    door = pieces["battery_door_v8"]
    printed = [([(in_print_orient(pel, "pelvis_v8"), C_PELVIS)], 135, -50, 2.9),
               ([(in_print_orient(tor, "torso_v8"), C_TORSO)], 135, -50, 2.3),
               ([(in_print_orient(fc, "front_cover_v8"), C_COVER)], 135, -55, 2.6),
               ([(in_print_orient(pc, "power_cover_v8"), C_COVER)], 135, -55, 2.6),
               ([(in_print_orient(door, "battery_door_v8"), C_COVER)], 135, -55, 2.6)]
    boards = mock_stack(plugs=True) + mock_pwrb()
    keep = box(-200, 200, -200, SNAP_Y, -200, 200)
    assembly = [([(Pos(0, 0, -55) * pel, C_PELVIS), (tor, C_TORSO), (Pos(40, 0, 0) * fc, C_COVER),
                  (Pos(-40, 0, 0) * pc, C_COVER), (Pos(0, 45, 0) * door, C_COVER),
                  (Pos(0, 0, 45) * mock_pack(), C_PACK), (boards, C_BOARD)],
                 150, -18, 1.8),
                ([(pel, C_PELVIS), (tor, C_TORSO), (fc, C_COVER), (pc, C_COVER), (door, C_COVER)], 210, 30, 2.7),
                ([(pel, C_PELVIS), (tor, C_TORSO), (fc, C_COVER), (pc, C_COVER), (Pos(0, 60, 0) * door, C_COVER),
                  (Pos(0, 50, 0) * mock_pack(), C_PACK), (boards, C_BOARD)], -60, -15, 2.0),
                ([(pel & keep, C_PELVIS), (tor & keep, C_TORSO), (fc & keep, C_COVER), (pc & keep, C_COVER),
                  (mock_pack() & keep, C_PACK), (boards & keep, C_BOARD)], -90, -5, 2.2)]
    a = _render_strip(printed, os.path.join(out_dir, "torso_v8.png"))
    b = _render_strip(assembly, os.path.join(out_dir, "torso_v8_assembly.png"))
    return a, b


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    pieces = build(arm_mounts=True)
    findings = {}
    for name, s in pieces.items():
        n = len(s.solids())
        bb = s.bounding_box()
        path = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, path)
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        dims = sorted((bb.size.X, bb.size.Y, bb.size.Z), reverse=True)
        print(f"{name:16s} {n} solid(s)  {mass_g(s):6.1f} g  "
              f"{dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm "
              f"({'BED-OK' if dims[0] <= D.BED and dims[1] <= D.BED else '** TOO BIG **'})")
        apath = path
        if name == "torso_v8":
            # the windows that print with support (and the door's opening) filled
            apath = os.path.join(OUT_REN, "torso_v8_audit", f"{name}.stl")
            os.makedirs(os.path.dirname(apath), exist_ok=True)
            export_stl(torso_v8(arm_mounts=True, audit=True), apath)
        findings[name] = audit(name, apath)
    print(f"built in {time.time() - t0:.0f}s; snap finger strain {snap_strain() * 100:.2f} %")
    print("\n-- fits --")
    check_fits(pieces)
    print("\n-- screw driver access (7 mm ACCESS_D) --")
    check_screw_access(pieces)
    print("\n-- screw thread bite --")
    check_screw_bite()
    print("\n-- printability --")
    n_enf = len(support_enforcers())
    for name, (fails, ok) in findings.items():
        notes = "; ".join(sorted({w for *_, w in ok}))
        sup = f"support in its {n_enf} enforced windows only" if name == "torso_v8" else "no supports"
        print(f"  {name}: {f'PASS, {sup}' if not fails else f'** {len(fails)} FAILURE(S) **'}"
              f"{f' (accepted: {notes})' if notes else ''}")
        for kind, a, desc, _ in fails:
            print(f"      {kind:8s} {desc}")
    print("wrote", *render(pieces, OUT_REN))
