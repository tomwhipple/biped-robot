"""torso_v8: the v6 body's torso as two prints and two covers (Tom, 2026-10-14):

  pelvis_v8       the legs' block: both yaw cells under one ceiling, and the
                  bearing housing's five pilots. Printed ceiling-down.
  torso_v8        the electronics: deck, skins, the battery layer over the
                  pelvis, the board-stack bay forward (the Pi with the General
                  Driver on it as a HAT) and the power-board bay aft, right
                  behind the pack. Printed deck-down.
  front_cover_v8  the stack bay's front wall, screwed on. Printed flat.
  power_cover_v8  the power bay's cap, screwed on: deep enough for the power
                  board's relay. Printed outer face down.
  power_bracket_v8 the BMS's tray, on two standoffs in the power board's +y
                  holes. Printed flat.
  balance_park_v8 the cup that holds the free end of the balance lead's
                  extension, taped to the aft inner wall. Printed floor down.
  switch_pod_v8   the block on the cap that holds the main switch and the
                  E-stop side by side, levers up. Printed aft face down.
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

THE POWER BAY (#213). The power board (power lite) stands on the aft wall's
four bosses, as the torso was drawn for; its relay K1 stands 26.7 off it, so
the bay's cover is a cap (power_cover_v8) 4.7 proud of the torso's aft end,
on three of the torso's four skin screws. Its BMS rides the BMS bracket
(power_bracket_v8) on two standoffs in the board's +y holes: over J1, J2 and
the W pads its wires come from, beside the fuses, which pull straight out
aft with the cap off. The torso itself is as it was: its skin holes,
windows and deck pockets are drawn from the flat cover's seat
(dimensions_v6 PCOV_SEAT_X), and the cap's fourth hole (+y, bottom) stays
empty because the BMS's corner is where its block would be. The main switch
and the E-stop hang side by side in the switch pod on the cap (dimensions_v6
SW_*, ES_*, SWP_*).

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
from common import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
from common import shapes as P  # noqa: E402
from common import mocks as CA  # noqa: E402
import paths  # noqa: E402

box, cyl_x, cyl_y, cyl_z = P.box, P.cyl_x, P.cyl_y, P.cyl_z
wedge_x, wedge_y, wedge_z = P.wedge_x, P.wedge_y, P.wedge_z
csk_x, csk_y, csk_z = P.csk_x, P.csk_y, P.csk_z
teardrop_y = P.teardrop_y

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

PIECES = ("pelvis_v8", "torso_v8", "front_cover_v8", "power_cover_v8", "switch_pod_v8", "power_bracket_v8",
          "balance_park_v8", "battery_door_v8")
# rows = print axes in model coords (check_printability's convention)
IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
RX180 = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
RY_XUP = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))
RY_XDOWN = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
RY_YDOWN = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))
PRINT_ORIENT = {
    "pelvis_v8": RX180,          # ceiling on the bed, cell walls rise
    "torso_v8": RX180,           # deck top on the bed
    "torso_v8_armless": RX180,
    "front_cover_v8": RY_XDOWN,  # outer face (+x) on the bed, corner blocks rise
    "power_cover_v8": RY_XUP,    # outer face (-x) on the bed, rim and corner blocks rise
    "power_bracket_v8": RY_XDOWN,  # the face on the standoffs (+x) on the bed, the tray's ledge and stops rise
    "balance_park_v8": IDENT,    # its floor on the bed, the cup's walls rise
    "switch_pod_v8": RY_XUP,     # its aft face (-x) on the bed, every wall rises
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
          ("power_cover_v8", V.PCOV_SEAT_X, V.TORSO_AFT_X, +1))


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
                (V.PCOV_SEAT_X + COVER_BLOCK[0] + 1.0, V.AFT_WALL_X[0] - 1.0, -56.0, under_door if s > 0 else -16.0)]
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
               (V.PCOV_SEAT_X + COVER_BLOCK[0] + 3.0, V.PWRB_PCB_X1 - 3.0, -(tin - 3.0), tin - 3.0)]
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


# ============================================================================
# THE POWER BAY: the cap (power_cover_v8) and the BMS's bracket (power_bracket_v8)
# ============================================================================
PCAP_SPIGOT = (1.6, 1.2)          # the cap's locating lip inside the torso: long (x), thick
PCAP_VENT_Y = (-50.0, 50.0)       # its vent slots, either side of the switch pod (the bay is open below too)
SWP_SCREWS_YZ = [(sy * (V.SWP_HW - V.SWP_SIDE / 2), ez) for sy in (1, -1) for ez in (-12.0, -62.0)]   # in the
                                  # switch pod's side walls' front edges
SWP_SCREW_L = 8.0                 # M2.5 x 8 flat head, in through the cap's plate from inside: 5.4 into an ear
PCAP_SCREW_CLR = 0.15             # the screws' pilots stay COVER_SCREW_L - 2 deep, as the flat cover's


def _pcap_blocks_yz():
    """The cap's three corner blocks: the torso's +y bottom skin hole is left
    empty, because a block there would stand where the BMS's lower +y corner
    is (dimensions_v6 BMS_*)."""
    return [(y, z) for y, z in _cover_blocks_yz() if not (y > 0 and z < (zt + zd) / 2)]


def _cap_plan(hw):
    """The cap's plan outline at half width hw: from its outer face to the
    torso's aft end, every corner square (Tom, 2026-11-04)."""
    x0, x1 = V.PCAP_X[0], V.TORSO_AFT_X
    return Pos((x0 + x1) / 2, 0) * Rectangle(x1 - x0, 2 * hw)


def power_cover_v8():
    """THE POWER CAP: the aft bay's cover, PCAP_PROUD aft of the torso's end
    so the power board's relay K1 has PWRB_COMP_AIR under it. Its rim carries
    the torso's skins and deck aft to a flat plate, its inner faces 0.2 inside
    the torso's; a lip off them goes in under the deck and inside the -y
    skin; three corner blocks reach forward inside the skins to the torso's
    skin screws (dimensions_v6 PCOV_SCREW_*), pilots teardropped toward the
    print's up (+x). The switch pod screws onto its outer face from inside
    (SWP_SCREWS_YZ) over a notch in its bottom edge for the switches' wires
    (SW_PASS); a vent slot either side of the pod."""
    lo = Pos(0, 0, zt) * _cap_plan(V.HOUSING_BOT_HW)
    hi = Pos(0, 0, zd) * _cap_plan(V.HOUSING_HW)
    c = loft([lo.faces()[0], hi.faces()[0]])
    c += Pos(0, 0, zd) * extrude(_cap_plan(V.HOUSING_HW), D.DECK_T)
    hy, ztop = tin - COVER_CLR, zd - COVER_CLR
    c -= box(V.PCAP_X[1], V.TORSO_AFT_X + 1.0, -hy, hy, zt - 1, ztop)
    sl, st = PCAP_SPIGOT
    xs = (V.PCAP_X[1] - 0.3, V.TORSO_AFT_X + sl)     # from the plate: it prints off the bed's plate
    c += box(*xs, -hy, hy, ztop - st, ztop)
    c += box(*xs, -hy, -hy + st, zt, ztop)
    by, bh = COVER_BLOCK[1], V.PCAP_BLOCK_H
    for yb, zb_ in _pcap_blocks_yz():
        s = 1 if yb > 0 else -1
        z0, z1 = (ztop - bh, ztop) if zb_ > (zt + zd) / 2 else (zt, zt + bh)
        y0, y1 = sorted((s * hy, s * (hy - by)))
        c += box(V.PCAP_X[1] - 0.3, V.PCOV_SCREW_X + COVER_BLOCK[0] / 2, y0, y1, z0, z1)
        c -= teardrop_y(D.M25_TAP / 2, s * (hy + 0.1), s * (hy - (COVER_SCREW_L - 2.0)),
                        V.PCOV_SCREW_X, zb_, roll=90)
    for yv in PCAP_VENT_Y:
        c -= box(V.PCAP_X[0] - 1, V.PCAP_X[1] + 1, yv - 2.0, yv + 2.0, zt + 12.0, ztop - 12.0)
    (py0, py1), (pz0, pz1) = V.SW_PASS
    c -= box(V.PCAP_X[0] - 1, V.PCAP_X[1] + 1, py0, py1, pz0, pz1)
    for ey, ez in SWP_SCREWS_YZ:
        c -= cyl_x(D.CASE_SCREW_CLEAR / 2, V.PCAP_X[0] - 1, V.PCAP_X[1] + 1, ey, ez)
        c -= csk_x(ey, ez, V.PCAP_X[1], +1)
    return c


PBR_SCREW_L = 6.0                 # M2.5 x 6 flat head into each standoff's female thread: 3.5 in
PBR_HOLES_YZ = [(y, z) for y, z in V.PWRB_HOLES_YZ if y > 0]           # H1 (top), H3 (bottom)
PBR_EDGE_IN = (3.5, 2.5)          # the plate stops this far inside the BMS's -y edge (finger room at
                                  # the fuses) and +y edge (the skin)
PBR_RIM_H = V.BMS_TAPE + V.BMS_PCB_T   # 2.2: the tray's ledge and stops stand to the BMS PCB's aft face
PBR_RIM_T = 1.6
PBR_RIM_CLR = 0.3
PBR_SPINE_HW = 4.5                # solid either side of the standoffs' line
PBR_BAND = 4.0                    # the frame round its two windows
PBR_PAD_HW = 3.8                  # round each standoff
PBR_TOP_STOP_Y = (41.0, 47.0)     # the stop over the BMS's top edge: between its P+ and B2 pads
PBR_SIDE_STOP_Z = (-58.0, -48.0)  # the stop at its -y edge: below F2, clear of W7's lead
PWRB_SO_R = 2.9                   # a standoff's 5 mm hex, round its corners


def power_bracket_v8():
    """THE BMS BRACKET: a plate on the two standoffs in the power board's +y
    holes, two countersunk M2.5 x 6 flat heads into them, and the BMS on it
    component side aft on foam tape (dimensions_v6 BMS_*). A ledge under the
    BMS's bottom edge, a stop over its top edge and one at its -y edge stand
    to its PCB's aft face, so it goes straight in from aft and stays put
    between them, the +y skin (0.9) and the cap (2.2) if its tape lets go. Two
    windows leave its bare side to the air; its four corner pads overhang the
    plate, so their wires come up to them from the board."""
    xf, xa = V.PBR_X
    y0, y1 = V.BMS_Y[0] + PBR_EDGE_IN[0], V.BMS_Y[1] - PBR_EDGE_IN[1]
    zl1 = V.BMS_Z[0] - PBR_RIM_CLR
    zl0 = zl1 - PBR_RIM_T
    p = box(xa, xf, y0, y1, zl0, V.BMS_Z[1])
    hy, hz_bot = min(PBR_HOLES_YZ, key=lambda h: h[1])
    p += box(xa, xf, hy - PBR_PAD_HW, hy + PBR_PAD_HW, V.BMS_Z[1] - 1.0, V.PWRB_TOP_Z)
    p += box(xa, xf, hy - PBR_PAD_HW, hy + PBR_PAD_HW, hz_bot - 3.4, zl1)
    zw = (hz_bot + 3.2 + 1.6, V.BMS_Z[1] - PBR_BAND)
    for wy in ((y0 + PBR_BAND, hy - PBR_SPINE_HW), (hy + PBR_SPINE_HW, y1 - PBR_BAND)):
        p -= box(xa - 1, xf + 1, *wy, *zw)
    xr = (xa + 0.3, xa - PBR_RIM_H)
    p += box(*xr, y0, y1, zl0, zl1)                                     # the ledge
    zt0 = V.BMS_Z[1] + PBR_RIM_CLR
    p += box(xa, xf, *PBR_TOP_STOP_Y, V.BMS_Z[1] - 1.0, zt0 + PBR_RIM_T)
    p += box(*xr, *PBR_TOP_STOP_Y, zt0, zt0 + PBR_RIM_T)               # the top stop
    ys1 = V.BMS_Y[0] - PBR_RIM_CLR
    p += box(xa, xf, ys1 - PBR_RIM_T, y0 + 1.0, *PBR_SIDE_STOP_Z)
    p += box(*xr, ys1 - PBR_RIM_T, ys1, *PBR_SIDE_STOP_Z)              # the side stop
    for sy, sz in PBR_HOLES_YZ:
        p -= cyl_x(D.CASE_SCREW_CLEAR / 2, xf + 1, xa - 1, sy, sz)
        p -= csk_x(sy, sz, xa, -1)
    return p


BAL_SLOT = (11.0, 2.4)            # the floor's slot for the lead's four wires: across, deep


def balance_park_v8():
    """THE BALANCE PARK (dimensions_v6 BAL_*): an open-topped cup that holds
    the balance extension lead's male end, pins up, its top at the door's
    sill. Its forward wall goes on the aft inner wall's aft face on foam tape;
    the lead's wires leave through a slot in its floor, too narrow for the
    male to drop through. Printed floor down: walls rising, no bridges."""
    w = V.BAL_WALL
    x1, x2 = V.BAL_IN_X
    y1, y2 = V.BAL_IN_Y
    c = box(V.BAL_X0, x2 - w, y1 - w, y2 + w, V.BAL_FLOOR_Z - w, V.BAL_RIM_Z)
    c -= box(x1, x2, y1, y2, V.BAL_FLOOR_Z, V.BAL_RIM_Z + 1)
    sx, sy = (x1 + x2) / 2, V.BAL_Y
    c -= box(sx - BAL_SLOT[1] / 2, sx + BAL_SLOT[1] / 2, sy - BAL_SLOT[0] / 2, sy + BAL_SLOT[0] / 2,
             V.BAL_FLOOR_Z - w - 1, V.BAL_FLOOR_Z + 0.1)
    return c


def mock_balance_male():
    """The extension lead's male end in the park, and the pack's balance plug
    on it with the turn of its wires (BAL_PLUG_ABOVE)."""
    x1, x2 = V.BAL_IN_X
    c = V.BAL_CLR
    m = box(x1 - c, x2 + c, V.BAL_IN_Y[0] + c, V.BAL_IN_Y[1] - c, V.BAL_FLOOR_Z, V.BAL_TOP_Z)
    m += box(x1 - c, x2 + c, V.BAL_IN_Y[0] + c, V.BAL_IN_Y[1] - c, V.BAL_TOP_Z, V.BAL_TOP_Z + V.BAL_PLUG_ABOVE)
    return m


BAL_LEAD_R = 1.5                  # the lead's four 26 AWG wires, bundled
BAL_LEAD_X = V.PWRB_PCB_X0 - 12.4  # its run aft of J2's plug (9.8), under the BMS bracket's plate (15)
BAL_LEAD_Z = -52.0                # its run under the park


def balance_lead_path():
    """The extension lead's route, as points: out of the top of its plug in
    J2, aft, across +y under the BMS bracket and past the board's +y edge,
    down, forward under the park and up into the park's floor slot."""
    j2 = [p for p in V.PWRB_PARTS if p[0] == "J2"][0]
    (ya, za), (yb, zb_) = V.pwrb_yz(j2[1], j2[3]), V.pwrb_yz(j2[2], j2[4])
    y0, z0, x0 = (ya + yb) / 2, (za + zb_) / 2, V.PWRB_PCB_X0 - j2[5]
    sx = sum(V.BAL_IN_X) / 2
    return [(x0, y0, z0), (BAL_LEAD_X, y0, z0), (BAL_LEAD_X, V.BAL_Y, z0), (BAL_LEAD_X, V.BAL_Y, BAL_LEAD_Z),
            (sx, V.BAL_Y, BAL_LEAD_Z), (sx, V.BAL_Y, V.BAL_FLOOR_Z - V.BAL_WALL)]


def swept(points, half):
    """Boxes along a polyline of axis-aligned legs: half = (hx, hy, hz) round
    each leg (or a list of them, one per leg), its own axis ending at its
    points (a joint is covered by the next leg's cross-section)."""
    m = None
    halves = half if isinstance(half, list) else [half] * (len(points) - 1)
    for (a, b), half in zip(zip(points, points[1:]), halves):
        lo = [min(u, v) - h for u, v, h in zip(a, b, half)]
        hi = [max(u, v) + h for u, v, h in zip(a, b, half)]
        axis = [k for k in range(3) if abs(a[k] - b[k]) > 1e-9]
        if not axis:                                                    # a leg of no length
            continue
        lo[axis[0]], hi[axis[0]] = min(a[axis[0]], b[axis[0]]), max(a[axis[0]], b[axis[0]])
        s = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        m = s if m is None else m + s
    return m


def mock_balance_lead():
    """The lead along balance_lead_path, BAL_LEAD_R round each leg."""
    return swept(balance_lead_path(), (BAL_LEAD_R,) * 3)


def balance_lead_length():
    pts = balance_lead_path()
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


# ============================================================================
# THE SWITCH POD (switch_pod_v8): the main switch and the E-stop side by side
# (dimensions_v6 SW_*, ES_*, SWP_*)
# ============================================================================
def _teardrop_z(r, z0, z1, x, y):
    """A vertical bore with a capped 45 deg roof toward +x (the pod prints
    with +x up)."""
    k = r / math.sqrt(2)
    roof = wedge_z([(x, y), (x + k, y + k), (x + r, y + r * (math.sqrt(2) - 1)),
                    (x + r, y - r * (math.sqrt(2) - 1)), (x + k, y - k)], z0, z1)
    return cyl_z(r, z0, z1, x, y) + roof


def _key_tab(x, y, r_face, r_out, w, z0, z1, ang):
    """An anti-rotation tab in a vertical hole at (x, y): w wide, its face
    r_face off the axis, out to r_out, at ang degrees about z from +x."""
    return Pos(x, y, 0) * Rot(0, 0, ang) * box(r_face, r_out, -w / 2, w / 2, z0, z1)


def switch_pod_v8():
    """THE SWITCH POD: one block on the cap's outer face at its top, its top
    flush with the deck's, open toward the cap; both switches' bodies hang
    in it outside the bay in a TERM_MARGIN of room with their terminals' lugs
    (SW_ENV, ES_ENV), over a channel along its floor to the cap's wire
    passage. Its top is one POCKET round both levers: the side walls, the
    aft wall and the cap's face, each lever's tip 2 mm or more under the
    rim. The main switch's bushing goes up through the pocket's floor at +y,
    the E-stop's through a PLATFORM at the -y end, higher, behind a step;
    each hole has its tab in the bushing's keyway (SW_KEY_*, ES_KEY_*). Four
    screws from the cap's inner face into the side walls' front edges
    (SWP_SCREWS_YZ). Printed aft face down: every wall rises; the bushings'
    bores are teardropped."""
    x0, x1 = V.SWP_X
    hy = V.SWP_HW
    xi = x1 - V.SWP_IN_X
    zp = V.SWP_TOP_Z - V.SW_PANEL_T                                      # the main panel's underside
    ze = V.ES_SHELF_Z - V.SW_PANEL_T                                     # the platform's underside
    ya, yb = -hy + V.SWP_SIDE, hy - V.SWP_SIDE                           # the inside, across
    ys0, ys1 = V.ES_STEP_Y
    p = box(x0, x1, -hy, hy, V.SWP_BOT_Z, V.SWP_RIM_Z)
    p -= box(xi, x1 + 1, ya, ys1, V.ES_SHELF_Z, V.SWP_RIM_Z + 1)         # the pocket over the platform
    p -= box(xi, x1 + 1, ys1, yb, V.SWP_TOP_Z, V.SWP_RIM_Z + 1)          # and over the main switch's panel
    p -= box(xi, x1 + 1, ya, yb, V.SWP_FLOOR_Z, zp)                      # under both
    p -= box(xi, x1 + 1, ya, ys0, V.SWP_FLOOR_Z, ze)                     # and under the platform
    for ey, ez in SWP_SCREWS_YZ:
        p -= cyl_x(D.M25_TAP / 2, x1 + 0.1, x1 - (SWP_SCREW_L - 2.0), ey, ez)
    x = V.SW_AXIS_X
    p -= _teardrop_z(V.SW_HOLE_D / 2, zp - 1, V.SWP_TOP_Z + 1, x, V.SW_Y) - _key_tab(
        x, V.SW_Y, V.SW_BUSH_D / 2 - V.SW_KEY_IN, V.SW_HOLE_D / 2 + 0.5, V.SW_KEY_W,
        V.SWP_TOP_Z - V.SW_KEY_L, V.SWP_TOP_Z + 2, V.SW_KEY_ANG)
    p -= _teardrop_z(V.ES_HOLE_D / 2, ze - 1, V.ES_SHELF_Z + 1, x, V.ES_Y) - _key_tab(
        x, V.ES_Y, V.ES_KEY_R, V.ES_HOLE_D / 2 + 0.5, V.ES_KEY_W, ze - 2, V.ES_SHELF_Z + 2, V.ES_KEY_ANG)
    return p


def _rod(a, b, r):
    """A round rod of radius r from point a to point b."""
    d = [v - u for u, v in zip(a, b)]
    n = math.sqrt(sum(v * v for v in d))
    from build123d import Align, Cylinder, Location, Plane
    c = Cylinder(r, n, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return c.moved(Location(Plane(origin=a, z_dir=tuple(v / n for v in d))))


def mock_switches():
    """Both toggles as they sit in the pod, levers up: the main switch --
    body and terminal screws under its panel, the bushing and nut over it,
    the lever upright -- and the E-stop -- body and its three solder lugs
    under its platform, the bushing, lockwasher and nut over it, the lever
    thrown IN (+y: run). Each bushing has its keyway, on its tab."""
    nut_r = V.SW_NUT_AF / 2 / math.cos(math.radians(30))
    x, y = V.SW_AXIS_X, V.SW_Y
    zp = V.SWP_TOP_Z - V.SW_PANEL_T
    zb = zp - V.SW_BODY[2]
    hx, hl = V.SW_BODY[1] / 2, V.SW_BODY[0] / 2
    m = box(x - hx, x + hx, y - hl, y + hl, zb, zp)
    m += box(x - hx + 2, x + hx - 2, y - hl + 3, y + hl - 3, zb - V.SW_TERM_L, zb)
    kw, kd = V.SW_KEYWAY
    slot = box(V.SW_BUSH_D / 2 - kd, V.SW_BUSH_D / 2 + 1, -kw / 2, kw / 2, zp - 1, zp + V.SW_THREAD_L + 1)
    m += cyl_z(V.SW_BUSH_D / 2, zp, zp + V.SW_THREAD_L, x, y) - Pos(x, y, 0) * Rot(0, 0, V.SW_KEY_ANG) * slot
    m += cyl_z(nut_r, V.SWP_TOP_Z, V.SWP_TOP_Z + V.SW_WASHER_T + V.SW_NUT_T, x, y)
    m += cyl_z(V.SW_LEVER_D / 2, zp + V.SW_THREAD_L, V.SW_LEVER_TOP_Z, x, y)
    y = V.ES_Y
    ze = V.ES_SHELF_Z - V.SW_PANEL_T                                     # the platform's underside
    zb = ze - V.ES_BODY[2]
    m += box(x - V.ES_BODY[1] / 2, x + V.ES_BODY[1] / 2, y - V.ES_BODY[0] / 2, y + V.ES_BODY[0] / 2, zb, ze)
    for ty in V.ES_TERM_Y.values():
        m += box(x - V.ES_LUG_W / 2, x + V.ES_LUG_W / 2, ty - V.ES_LUG_T / 2, ty + V.ES_LUG_T / 2,
                 zb - V.ES_LUG_L, zb)
    kw, kd = V.ES_KEYWAY
    slot = box(V.ES_BUSH_D / 2 - kd, V.ES_BUSH_D / 2 + 1, -kw / 2, kw / 2, ze - 1, ze + V.ES_BUSH_L + 1)
    ztop = ze + V.ES_BUSH_L                                               # the bushing's end
    m += cyl_z(V.ES_BUSH_D / 2, ze, ztop, x, y) - Pos(x, y, 0) * Rot(0, 0, V.ES_KEY_ANG) * slot
    m += cyl_z(V.ES_NUT_AF / 2 / math.cos(math.radians(30)), V.ES_SHELF_Z,
               V.ES_SHELF_Z + V.ES_WASHER_T + V.ES_NUT_T, x, y)
    a = math.radians(V.ES_THROW_DEG)
    m += _rod((x, y, ztop - 0.5), (x, y + V.ES_TIP * math.sin(a), ze + V.ES_TIP * math.cos(a)), V.ES_LEVER_D / 2)
    return m


def switch_keepouts():
    """{name: solid}: the room each switch keeps for its terminals, their lugs
    and wires (TERM_MARGIN round the body's outline; SW_ENV, ES_ENV deep), and
    each lever's throw (each way, plus its girth, and TERM_MARGIN)."""
    out = {}
    t = V.TERM_MARGIN
    x, y = V.SW_AXIS_X, V.SW_Y
    zp = V.SWP_TOP_Z - V.SW_PANEL_T
    hx, hl = V.SW_BODY[1] / 2 + t, V.SW_BODY[0] / 2 + t
    out["main switch's terminals"] = box(x - hx, x + hx, y - hl, y + hl, zp - V.SW_ENV, zp - V.SW_BODY[2])
    reach = V.SW_LEVER_L * math.sin(math.radians(V.SW_THROW_DEG)) + V.SW_LEVER_D / 2
    out["main switch's lever"] = box(x - V.SW_LEVER_D / 2 - t, x + V.SW_LEVER_D / 2 + t, y - reach, y + reach,
                                     zp + V.SW_THREAD_L, V.SW_LEVER_TOP_Z)
    y = V.ES_Y
    ze = V.ES_SHELF_Z - V.SW_PANEL_T
    hx, hl = V.ES_BODY[1] / 2 + t, V.ES_BODY[0] / 2 + t
    out["E-stop's body, lugs and wires"] = box(x - hx, x + hx, y - hl, y + hl, ze - V.ES_ENV, ze)
    zn = V.ES_SHELF_Z + V.ES_WASHER_T + V.ES_NUT_T
    out["E-stop's lever"] = box(x - V.ES_LEVER_D / 2 - t, x + V.ES_LEVER_D / 2 + t, y - V.ES_REACH - t,
                                y + V.ES_REACH + t, zn, ze + V.ES_TIP)
    return out


# The wires, each as the swept envelope it keeps (WIRE_MARGIN round the
# wires; their ends at a terminal, a pad or a plug): (name, points, (hx, hy,
# hz) half sizes). A pair of 14 AWG side by side is WIRE_OD_14 wide each.
# The main switch's pair runs through an inline two-pin disconnect parked
# over the board's low parts behind the cap (DISC_*), so the cap comes off
# with its switches; the E-stop's comes off at J11.
DISC = (26.0, 11.0, 6.0)          # ASSUMED (an XT30 pair, mated): long (y), tall (z), thick (x) -- check it
DISC_C = (V.PWRB_PCB_X0 - 16.0, -24.0, -51.0)   # its centre: over Q1/Q2 and the resistors, clear of J8
LANE_Y = {"main": -11.0, "estop": -21.5}          # each pair's lane through the cap's passage (SW_PASS)


def _pair14():
    return (WIRE_14_T, WIRE_14_W, WIRE_14_W)


WIRE_14_W = V.WIRE_OD_14 + V.WIRE_MARGIN          # 5.4: half a side-by-side pair and its margin, across
WIRE_14_T = V.WIRE_OD_14 / 2 + V.WIRE_MARGIN      # 3.7: half its thickness and its margin
WIRE_24 = V.WIRE_OD_24 + V.WIRE_MARGIN            # 3.4: half a 24 AWG pair side by side and its margin, across


def wire_paths():
    """[(name, points, half)] for every lead the CAD keeps room for."""
    x0 = V.PWRB_PCB_X0
    xax = V.SW_AXIS_X
    zch = V.SWP_CH_Z                                                     # the pod's channel and the passage
    z_env = V.SWP_TOP_Z - V.SW_PANEL_T - V.SW_ENV
    dx, dy, dz = DISC_C
    w6, w7 = V.pwrb_yz(23, 9), V.pwrb_yz(31, 9)
    w1, w2 = V.pwrb_yz(23, 19), V.pwrb_yz(23, 26)
    j11 = [p for p in V.PWRB_PARTS if p[0] == "J11"][0]
    (j11y0, j11z0), (j11y1, j11z1) = V.pwrb_yz(j11[1], j11[3]), V.pwrb_yz(j11[2], j11[4])
    y_up = 4.0                                                            # the main pair's run up over the fuses
    es_path = estop_lead_path()
    return [
        ("main switch's pair, in the pod and to the disconnect", [
            (xax, V.SW_Y, z_env), (xax, V.SW_Y, zch), (xax, LANE_Y["main"], zch),
            (dx, LANE_Y["main"], zch), (dx, dy - DISC[0] / 2 - 2, zch), (dx, dy - DISC[0] / 2 - 2, dz)],
         _pair14()),
        ("main switch's pair, the disconnect to W6/W7", [
            (dx, dy + DISC[0] / 2, dz), (dx, y_up, dz), (x0 - 22.5, y_up, dz), (x0 - 22.5, y_up, w6[1]),
            (x0 - 9.5, y_up, w6[1]), (x0 - 9.5, (w6[0] + w7[0]) / 2, w6[1]),
            (x0 - V.PWRB_LOW_H, (w6[0] + w7[0]) / 2, w6[1])],
         _pair14()),
        # off lugs 2 and 3, down the pod to its channel, through the notch in
        # the cap's bottom edge and across over J12 to J11
        ("E-stop pair, in the pod and to J11", es_path, (WIRE_24,) * 3),
        # out of W1 and W2 (7 apart in z), up past the bracket's top, over the
        # BMS's top edge one behind the other, P-'s on along the edge
        ("BMS's 14 AWG pair, W1/W2 over its top edge", [
            (x0 - V.PWRB_LOW_H, w1[0], (w1[1] + w2[1]) / 2), (x0 - 9.5, w1[0], (w1[1] + w2[1]) / 2),
            (x0 - 9.5, w1[0], -9.0), (x0 - 21.0, w1[0], -9.0), (x0 - 21.0, 26.5, -9.0)],
         [(WIRE_14_T, WIRE_14_T, WIRE_14_T + (w1[1] - w2[1]) / 2)] + [(WIRE_14_T,) * 3] * 3),
        ("balance extension, J2 to the park", balance_lead_path(),
         (BAL_LEAD_R / 2 + 1.5, BAL_LEAD_R + V.WIRE_MARGIN + 1.0, BAL_LEAD_R + V.WIRE_MARGIN + 1.0)),
    ]


def estop_lead_path():
    """The E-stop pair's route, as points (wire_paths)."""
    x0 = V.PWRB_PCB_X0
    j11 = [p for p in V.PWRB_PARTS if p[0] == "J11"][0]
    (j11y0, j11z0), (j11y1, j11z1) = V.pwrb_yz(j11[1], j11[3]), V.pwrb_yz(j11[2], j11[4])
    jy, jz = (j11y0 + j11y1) / 2, (j11z0 + j11z1) / 2
    xax = V.SW_AXIS_X
    y23 = (V.ES_TERM_Y[2] + V.ES_TERM_Y[3]) / 2
    z_env = V.ES_SHELF_Z - V.SW_PANEL_T - V.ES_ENV
    zch = V.SWP_CH_Z
    ly = LANE_Y["estop"]
    return [(xax, y23, z_env), (xax, y23, zch), (xax, ly, zch), (x0 - 25.0, ly, zch),
            (x0 - 25.0, jy, zch), (x0 - 25.0, jy, jz), (x0 - j11[5], jy, jz)]


def estop_lead_length():
    pts = estop_lead_path()
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def wire_envelopes():
    """{name: solid}: wire_paths swept, and the disconnect with its margin."""
    out = {nm: swept(pts, h) for nm, pts, h in wire_paths()}
    dx, dy, dz = DISC_C
    m = V.WIRE_MARGIN
    out["main switch's disconnect"] = box(dx - DISC[2] / 2 - m, dx + DISC[2] / 2 + m, dy - DISC[0] / 2 - m,
                                          dy + DISC[0] / 2 + m, dz - DISC[1] / 2 - m, dz + DISC[1] / 2 + m)
    return out


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


PWRB_XT60_PLUG = 12.0          # the mated female + solder cups past the board's left edge (estimate)
PWRB_HOLE_KEEP = 3.0           # nothing round its holes on the parts side (the footprint's 2.95 courtyard)


def _pwrb_box(bx0, bx1, by0, by1, h):
    """A part's box at height h off the power board's parts face (board coords)."""
    ya, za = V.pwrb_yz(bx0, by0)
    yb, zb_ = V.pwrb_yz(bx1, by1)
    return box(V.PWRB_PCB_X0 - h, V.PWRB_PCB_X0, min(ya, yb), max(ya, yb), min(za, zb_), max(za, zb_))


def mock_pwrb(plug=True, leads=True):
    """The power board, power lite: the PCB, its low layer (resistors, D1)
    less round its four holes, each taller part at its height
    (dimensions_v6 PWRB_PARTS), J1's mated plug past the left edge, and
    (leads) the THT leads' keep-out under it, less round its holes. The BMS's
    and the main switch's wires on the W pads are not drawn."""
    x0 = V.PWRB_PCB_X0
    m = box(x0, V.PWRB_PCB_X1, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
    low = box(x0 - V.PWRB_LOW_H, x0, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
    for hy, hz in V.PWRB_HOLES_YZ:
        low -= cyl_x(PWRB_HOLE_KEEP, x0 - V.PWRB_LOW_H - 1, x0 + 0.1, hy, hz)
    m += low
    for ref, bx0, bx1, by0, by1, h in V.PWRB_PARTS:
        m += _pwrb_box(bx0, bx1, by0, by1, h)
    if plug:
        m += mock_j1_plug()
    if leads:
        under = box(V.PWRB_PCB_X1, V.PWRB_PCB_X1 + V.PWRB_UNDER, *V.PWRB_Y, V.PWRB_BOT_Z, V.PWRB_TOP_Z)
        for hy, hz in V.PWRB_HOLES_YZ:
            under -= _hole_keepout(V.PWRB_PCB_X1 - 1, V.PWRB_PCB_X1 + V.PWRB_UNDER + 1, hy, hz)
        m += under
    return m


def mock_j1_plug():
    """The pack's XT60 plug, mated in J1 past the board's +y edge."""
    ref, bx0, bx1, by0, by1, h = V.PWRB_PARTS[0]
    assert ref == "J1"
    _, za = V.pwrb_yz(bx0, by0)
    _, zb_ = V.pwrb_yz(bx1, by1)
    return box(V.PWRB_PCB_X0 - h, V.PWRB_PCB_X0, V.PWRB_Y[1], V.PWRB_Y[1] + PWRB_XT60_PLUG,
               min(za, zb_), max(za, zb_))


def mock_pwrb_standoffs():
    """The bracket's two M2.5 x 15 male-female standoffs on the board's parts
    face in its +y holes (H1, H3): each hex as a cylinder round its corners.
    Their 6 mm male studs (through the board into the bosses) are not drawn."""
    m = None
    for hy, hz in PBR_HOLES_YZ:
        c = cyl_x(PWRB_SO_R, V.PBR_X[0], V.PWRB_PCB_X0, hy, hz)
        m = c if m is None else m + c
    return m


def mock_bms():
    """The BMS (dimensions_v6 BMS_*): its PCB, bare side forward on the
    bracket's tape, and its parts as one block inside a BMS_EDGE margin."""
    xb, e = V.BMS_X[0], V.BMS_EDGE
    m = box(xb - V.BMS_PCB_T, xb, *V.BMS_Y, *V.BMS_Z)
    m += box(V.BMS_X[1], xb - V.BMS_PCB_T, V.BMS_Y[0] + e, V.BMS_Y[1] - e, V.BMS_Z[0] + e, V.BMS_Z[1] - e)
    return m


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
        if (hy, hz) in PBR_HOLES_YZ:
            s.append(dict(name=f"pwrb_so_{hy:+.0f}_{hz:+.0f}", part="torso_v8",
                          kind=f"M2.5 x {V.PWRB_SO_L:.0f} male-female standoff, {STACK_MALE:.0f} mm male: "
                               "the power board into its boss", axis=(1, 0, 0),
                          pos=(V.PWRB_PCB_X0, hy, hz), length=STACK_MALE, clamp=V.PWRB_T, pilot=PWRB_PILOT))
            s.append(dict(name=f"pbr_{hy:+.0f}_{hz:+.0f}", part="power_bracket_v8",
                          kind="M2.5x6 flat head: the BMS bracket into the standoff", axis=(1, 0, 0),
                          pos=(V.PBR_X[1], hy, hz), length=PBR_SCREW_L, clamp=V.PBR_T, pilot=None))
        else:
            s.append(dict(name=f"pwrb_{hy:+.0f}_{hz:+.0f}", part="torso_v8",
                          kind="M2.5x6 self-tap, pan head (standoff boss)", axis=(1, 0, 0),
                          pos=(V.PWRB_PCB_X0, hy, hz), length=PWRB_SCREW_L, clamp=V.PWRB_T, pilot=PWRB_PILOT))
    for ey, ez in SWP_SCREWS_YZ:
        s.append(dict(name=f"switch_pod_{ey:+.0f}_{ez:+.0f}", part="switch_pod_v8",
                      kind="M2.5x8 self-tap, flat head (through the cap into the switch pod's side wall)",
                      axis=(-1, 0, 0), pos=(V.PCAP_X[1], ey, ez), length=SWP_SCREW_L, clamp=V.COVER_T,
                      pilot=SWP_SCREW_L - 2.0))
    sx, sz = V.DOOR_SCREW
    s.append(dict(name="battery_door", part="battery_door_v8",
                  kind="M2.5x8 self-tap, flat head (through the door into its post)", axis=(0, -1, 0),
                  pos=(sx, DOOR_OUT_Y, sz), length=V.DOOR_SCREW_L, clamp=DOOR_OUT_Y - tin, pilot=DOOR_PILOT))
    for name, x_in, _, sx in COVERS:
        sxx = _cover_screw_x(x_in, sx)
        for yb, zb_ in (_pcap_blocks_yz() if name == "power_cover_v8" else _cover_blocks_yz()):
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
    bay's cover off (the Pi's standoffs before the General Driver goes on,
    the BMS bracket's before the BMS goes on). Where 7 mm fails, the line
    says whether a SLIM_DRIVER_D shank clears."""
    every = (pieces["pelvis_v8"] + pieces["torso_v8"] + pieces["power_cover_v8"]
             + pieces["front_cover_v8"] + pieces["battery_door_v8"])
    no_door = pieces["pelvis_v8"] + pieces["torso_v8"] + pieces["power_cover_v8"] + pieces["front_cover_v8"]
    open_bays = pieces["pelvis_v8"] + pieces["torso_v8"]
    ok = True
    for sc in SCREWS():
        # a yaw servo is serviced with the pack out, so the door is off too
        solid = (open_bays if sc["part"] in ("torso_v8", "stack", "power_bracket_v8")
                 else pieces["power_cover_v8"] if sc["part"] == "switch_pod_v8"   # on the bench, into the cap
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


# the power bay's clearances that are more than "no overlap": (name, a, b, mm)
POWER_BAY_GAPS = (("the cap over the board's parts (K1 2.0 under its plate)", "cap", "board", 1.5),
                  ("the BMS over the fuses, J1 and J2", "bms", "board", 2.0),
                  ("the BMS under the cap (its +y top block)", "bms", "cap", 1.0),
                  ("the BMS bracket over the board's parts", "bracket", "board", 2.0),
                  ("the BMS bracket under the cap", "bracket", "cap", 1.0),
                  ("the BMS and its bracket vs the battery door", "bms+bracket", "door", 1.0),
                  ("the balance park and the pack's plug on it vs the door", "park+plug", "door", 1.0),
                  ("the pack's balance plug on the park vs J1's plug", "plug", "j1", 1.0),
                  ("the balance park vs the board's +y edge", "park", "board", 0.4),
                  ("the balance lead vs the BMS bracket", "lead", "bracket", 1.0),
                  ("the switch pod vs the cap's vent slot", "pod", "vent", 2.0))


def check_fits(pieces):
    """The installs and the seated fit, each a sweep or an intersection:
    - the Pi + General Driver stack goes in from the front onto its four
      standoffs (front cover off), and the power board from aft (the cap
      off), each with its parts and its solder side; then, from aft, the
      balance park onto the aft wall, the BMS bracket onto its two standoffs,
      the BMS onto the bracket's tape and the cap over them;
    - the pelvis goes up into the torso from below -- the barbs are left out,
      they ride the webs by design (SNAP_BARB);
    - the battery swap: the door swung off about its tongue (the plug still
      in J1), J1's plug pulled off +y, and the pack slid out +y through the
      opening, each clear of the torso, the pelvis and the power bay;
    - seated: pelvis, covers, door, pack, stack, power board, standoffs,
      bracket and BMS vs the printed parts; the General Driver's mated
      service plugs vs the Pi's USB/Ethernet end (HAT_GAP) and vs the torso;
      all < 0.5 mm3; and the power bay's gaps (POWER_BAY_GAPS)."""
    pel, tor = pieces["pelvis_v8"], pieces["torso_v8"]
    fc, pc = pieces["front_cover_v8"], pieces["power_cover_v8"]
    door, br = pieces["battery_door_v8"], pieces["power_bracket_v8"]
    ok = True
    stack, pwrb = mock_stack(), mock_pwrb(plug=False)
    so, bms = mock_pwrb_standoffs(), mock_bms()
    park, male, lead = pieces["balance_park_v8"], mock_balance_male(), mock_balance_lead()
    bay = mock_pwrb() + so + br + bms + park + male + lead
    ok &= _sweep(stack, tor, ((40.0, 0, 0), (0, 0, 0)), "Pi + General Driver stack in from the front")
    ok &= _sweep(pwrb, tor, ((-40.0, 0, 0), (0, 0, 0)), "power board in from aft")
    ok &= _sweep(park, tor + pel + pwrb, ((-30.0, 0, 0), (0, 0, 0)), "balance park in from aft onto the aft wall")
    ok &= _sweep(br, tor + pwrb + so + park + male + lead, ((-40.0, 0, 0), (0, 0, 0)),
                 "BMS bracket in from aft onto its standoffs")
    ok &= _sweep(bms, tor + pwrb + so + br + park + male + lead, ((-30.0, 0, 0), (0, 0, 0)),
                 "BMS in from aft onto the bracket")
    pod, sw = pieces["switch_pod_v8"], mock_switches()
    ko, env = switch_keepouts(), wire_envelopes()
    capset = pc + pod + sw
    ok &= _sweep(capset, tor + pel + bay, ((-30.0, 0, 0), (0, 0, 0)), "the cap on from aft, its pod and switches on it")
    _, barbs = _snap_fingers()
    ok &= _sweep(pel, tor - barbs, ((0, 0, -45.0), (0, 0, 0)), "pelvis up into the torso (barbs aside)", n=46)
    ok &= _door_swing(door, tor + pel + mock_pack() + bay)
    ok &= _sweep(mock_j1_plug(), tor + pel + so + br + bms + park + male, ((0, 0, 0), (0, 22.0, 0)),
                 "J1's plug pulled off and out through the door")
    ok &= _sweep(mock_pack(), tor + pel, ((0, 0, 0), (0, 2 * V.BATT_HY + 16.0, 0)),
                 "pack slid out through the door (door off)", n=57)
    shell = tor + fc + pc + door
    pi_jacks = box(V.PI_PCB_X1, V.PI_COMP_X, V.GD_Y[1], V.PI_Y[1] + V.PI_USB_PROUD, V.PI_BOT_Z, V.PI_TOP_Z)
    seated = [("pelvis vs torso", pel, tor), ("front cover vs torso", fc, tor),
              ("power cover vs torso", pc, tor), ("battery door vs torso", door, tor),
              ("battery door vs pelvis+pack+power bay", door, pel + mock_pack() + bay),
              ("pack vs pelvis+torso", mock_pack(), pel + tor),
              ("stack vs torso+covers", stack, shell), ("power board (plug in) vs torso+covers", mock_pwrb(), shell),
              ("standoffs vs torso+covers+power board", so, shell + mock_pwrb()),
              ("BMS bracket vs torso+covers+standoffs", br, shell + so),
              ("BMS vs torso+covers+bracket+power board", bms, shell + br + mock_pwrb()),
              ("balance park vs torso+covers+power board", park, shell + mock_pwrb()),
              ("balance lead's male end and the pack's plug vs torso+covers+park", male, shell + mock_pwrb() + park),
              ("balance lead vs torso+covers+board+bracket+BMS+park", lead, shell + mock_pwrb() + br + bms + park),
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
    rest = {"torso": tor, "pelvis": pel, "cap": pc, "switch pod": pod, "door": door, "board": mock_pwrb(),
            "standoffs": so, "BMS bracket": br, "BMS": bms, "balance park": park}
    print("  keep-outs (TERM_MARGIN, WIRE_MARGIN in them) vs every part, mm (0.00 = touching where it ends):")
    for nm, k in [*ko.items(), *env.items()]:
        bad = [(n, (k & b).volume) for n, b in rest.items() if (k & b).volume > 0.01]
        near = min((k.distance_to(b), n) for n, b in rest.items())
        ok &= not bad
        print(f"    {nm}: {'PASS' if not bad else 'FAIL ' + str(bad)}, nearest {near[1]} {near[0]:.2f}")
    v = (sw & pod).volume + (sw & pc).volume
    print(f"  seated switches in the pod: {v:.2f} mm3 ({'PASS' if v < 0.05 else 'FAIL'})")
    ok &= v < 0.05
    vent = box(V.PCAP_X[0] - 1, V.PCAP_X[1] + 1, PCAP_VENT_Y[0] - 2.0, PCAP_VENT_Y[0] + 2.0, zt + 12.0, zd - 12.0)
    solids = {"cap": pc, "board": mock_pwrb(), "bms": bms, "bracket": br, "bms+bracket": bms + br, "door": door,
              "park": park, "park+plug": park + male, "plug": male, "j1": mock_j1_plug(), "lead": lead,
              "pod": pod, "vent": vent}
    print(f"  the balance lead's route, J2 to the park: {balance_lead_length():.0f} mm")
    print(f"  the E-stop pair's route, its lugs to J11: {estop_lead_length():.0f} mm")
    for nm, a, b, need in POWER_BAY_GAPS:
        d = solids[a].distance_to(solids[b])
        good = d >= need
        ok &= good
        print(f"  gap {nm}: {d:.2f} mm (>= {need}) ({'PASS' if good else 'FAIL'})")
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
    from common import check_printability as CP
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
            "switch_pod_v8": switch_pod_v8(), "power_bracket_v8": power_bracket_v8(),
            "balance_park_v8": balance_park_v8(),
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
C_BRACKET, C_BMS, C_PARTS, C_LEAD = (0.95, 0.52, 0.12), (0.20, 0.36, 0.78), (0.20, 0.20, 0.22), (0.80, 0.15, 0.15)
C_KEEP = (0.98, 0.88, 0.35)


def mock_pwrb_split():
    """mock_pwrb in two, for colour: (the PCB and its low layer, its taller
    parts with J1's plug)."""
    parts = mock_j1_plug()
    for ref, bx0, bx1, by0, by1, h in V.PWRB_PARTS:
        parts += _pwrb_box(bx0, bx1, by0, by1, h)
    return mock_pwrb(plug=False, leads=False) - parts, parts


def render(pieces, out_dir):
    """Two strips in out_dir:
    torso_v8.png           the pieces as they lie on the bed;
    torso_v8_assembly.png  exploded (the pelvis below, the covers and the
                           door off, the stack, the power board and the
                           pack in place), assembled from below, the door
                           off with the pack half out, and a section at y = +14
                           through the snap fingers (the -y half, seen
                           from +y)."""
    pel, tor = pieces["pelvis_v8"], pieces["torso_v8"]
    fc, pc = pieces["front_cover_v8"], pieces["power_cover_v8"]
    door, br = pieces["battery_door_v8"], pieces["power_bracket_v8"]
    printed = [([(in_print_orient(pel, "pelvis_v8"), C_PELVIS)], 135, -50, 2.9),
               ([(in_print_orient(tor, "torso_v8"), C_TORSO)], 135, -50, 2.3),
               ([(in_print_orient(fc, "front_cover_v8"), C_COVER)], 135, -55, 2.6),
               ([(in_print_orient(pc, "power_cover_v8"), C_COVER)], 135, -55, 2.6),
               ([(in_print_orient(br, "power_bracket_v8"), C_BRACKET)], 135, -55, 2.6),
               ([(in_print_orient(pieces["balance_park_v8"], "balance_park_v8"), C_BRACKET)], 135, -45, 2.6),
               ([(in_print_orient(pieces["switch_pod_v8"], "switch_pod_v8"), C_BRACKET)], 135, -45, 2.6),
               ([(in_print_orient(door, "battery_door_v8"), C_COVER)], 135, -55, 2.6)]
    boards = (mock_stack(plugs=True) + mock_pwrb() + mock_pwrb_standoffs() + br + mock_bms()
              + pieces["balance_park_v8"] + mock_balance_male() + mock_balance_lead())
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


def render_power_bay(pieces, out_dir):
    """Two strips in out_dir for docs/assembly.md section 9c, the aft half of
    the torso only:
    power_bay_v8.png         from aft: exploded (the power board on its
                             standoffs, then the BMS bracket, the BMS and the
                             cap backed off along their way in), in place with
                             the cap off (from aft and -y, the fuses beside
                             the BMS), the cap on, a section at z = -25
                             through J1, F1 and K1 seen from above (forward
                             up, +y to the left), and from +y with the +y skin
                             cut away (J1's plug, the balance park with the
                             pack's balance plug on it, the lead from J2), and
                             the keep-outs (yellow: the switches' terminals and
                             levers, every lead's swept envelope, the
                             disconnect) with the cap off; the cap carries the
                             switch pod with both switches;
    power_bay_prints_v8.png  the cap, the bracket, the park and the switch pod
                             as they lie on the bed;
    switch_pod_v8.png        the switch pod on the cap from outside: from aft
                             and -y, from aft and above, and from +y."""
    pel, tor, pc, br, pk, pod = (pieces[n] for n in ("pelvis_v8", "torso_v8", "power_cover_v8",
                                                     "power_bracket_v8", "balance_park_v8", "switch_pod_v8"))
    capset = [(pc, C_COVER), (pod, C_BRACKET), (mock_switches(), C_PARTS)]
    aft = box(-120.0, -20.0, -200, 200, -200, 200)
    pcb, parts = mock_pwrb_split()
    so, bms = mock_pwrb_standoffs(), mock_bms()
    base = [(pel & aft, C_PELVIS), (tor & aft, C_TORSO), (pcb, C_BOARD), (parts, C_PARTS), (so, C_PARTS),
            (pk, C_BRACKET), (mock_balance_male(), C_PARTS), (mock_balance_lead(), C_LEAD)]
    cut = box(-200, 200, -200, 200, -200, -25.0)
    skin = box(-200, 200, -200, V.TORSO_IN_HW - 0.01, -200, 200)
    keep = [(k, C_KEEP) for k in [*switch_keepouts().values(), *wire_envelopes().values()]]
    bay = [([*base, (Pos(-35, 0, 0) * br, C_BRACKET), (Pos(-70, 0, 0) * bms, C_BMS),
             *[(Pos(-110, 0, 0) * s_, c) for s_, c in capset]], -40, -20, 2.4),
           ([*base, (br, C_BRACKET), (bms, C_BMS)], 25, -15, 2.3),
           ([*base, (br, C_BRACKET), (bms, C_BMS), *capset], -35, -35, 2.2),
           ([(s_ & cut, c) for s_, c in [*base, (br, C_BRACKET), (bms, C_BMS), *capset]], 0, -89, 2.5),
           ([(s_ & skin, c) for s_, c in [*base, (br, C_BRACKET), (bms, C_BMS), *capset]], -90, -20, 2.3),
           ([*base, (br, C_BRACKET), (bms, C_BMS), *keep], -30, -25, 2.3)]
    prints = [([(in_print_orient(pc, "power_cover_v8"), C_COVER)], 135, -55, 2.4),
              ([(in_print_orient(br, "power_bracket_v8"), C_BRACKET)], 135, -55, 2.4),
              ([(in_print_orient(pk, "balance_park_v8"), C_BRACKET)], 135, -45, 2.6),
              ([(in_print_orient(pod, "switch_pod_v8"), C_BRACKET)], 135, -45, 2.6)]
    aft = box(-200.0, -30.0, -200, 200, -200, 200)
    back = [(tor & aft, C_TORSO), *capset]
    pod_views = [(back, 35, -40, 1.9), (back, 0, -72, 1.9), (back, -35, -40, 1.9)]
    return (_render_strip(bay, os.path.join(out_dir, "power_bay_v8.png")),
            _render_strip(prints, os.path.join(out_dir, "power_bay_prints_v8.png")),
            _render_strip(pod_views, os.path.join(out_dir, "switch_pod_v8.png")))


def power_bay_step(pieces, path, board_step=None):
    """The power bay as one STEP for FreeCAD (File > Open): the pelvis, the
    torso, the battery door and the pack, the power board, its standoffs, the
    BMS bracket, the BMS, the balance park with the extension lead's male end
    (and the pack's plug on it) and the lead, the cap with the switch pod and
    the two toggles in it, and the keep-outs the checks hold everything off (the
    switches' terminals and levers, every lead's envelope), each a named part. board_step: power
    lite's own STEP (kicad-cli pcb export step --user-origin 100x60mm, its
    origin the top-left corner of the parts side) placed on the bosses in
    place of the mock, when given."""
    from build123d import Compound, Location, Plane, import_step
    if board_step:
        board = import_step(board_step).moved(Location(Plane(
            origin=(V.PWRB_PCB_X1, V.PWRB_Y[1], V.PWRB_TOP_Z), x_dir=(0, -1, 0), z_dir=(-1, 0, 0))))
    else:
        board = mock_pwrb(leads=False)
    kids = []
    for name, s in (("pelvis_v8", pieces["pelvis_v8"]), ("torso_v8", pieces["torso_v8"]),
                    ("battery_door_v8", pieces["battery_door_v8"]), ("pack", mock_pack()),
                    ("power_lite_board", board), ("standoffs_m2.5x15", mock_pwrb_standoffs()),
                    ("power_bracket_v8", pieces["power_bracket_v8"]), ("bms_cf_3sjh25a", mock_bms()),
                    ("balance_park_v8", pieces["balance_park_v8"]),
                    ("balance_extension_male_and_pack_plug", mock_balance_male()),
                    ("balance_extension_lead", mock_balance_lead()),
                    ("switch_pod_v8", pieces["switch_pod_v8"]), ("toggles_main_and_estop", mock_switches()),
                    *[(f"keepout {k}", v) for k, v in switch_keepouts().items()],
                    *[(f"keepout {k}", v) for k, v in wire_envelopes().items()],
                    ("power_cover_v8", pieces["power_cover_v8"])):
        s.label = name
        kids.append(s)
    export_step(Compound(children=kids, label="power_bay_v8"), path)
    return path


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    if "--figures" in sys.argv:          # the docs' renders only (make figures): nothing else written
        print("wrote", *render_power_bay(build(arm_mounts=True), OUT_REN))
        sys.exit(0)
    if "--power-bay-step" in sys.argv:   # [board.step]: the FreeCAD-openable power bay
        i = sys.argv.index("--power-bay-step")
        bs = sys.argv[i + 1] if len(sys.argv) > i + 1 else None
        print("wrote", power_bay_step(build(arm_mounts=True), os.path.join(OUT_STEP, "power_bay_v8.step"), bs))
        sys.exit(0)
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
    print("wrote", *render(pieces, OUT_REN), *render_power_bay(pieces, OUT_REN))
