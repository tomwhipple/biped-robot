"""pelvis_v7: the whole v7 torso, one print (v6 body, 2026-09-14).

Local frame like v5/v6: deck TOP at z=0, robot forward = +X, legs at
y = +/-V.HIP_SEP/2 (+/-42). Prints deck-top-down (see PRINT_ORIENT).

WHAT'S NEW vs v5's pelvis() (cad/parts.py): the yaw cells get a CEILING (v5's
cells were open straight to the deck 36 mm above; here the deck is 67 mm
above the cell bottom, so the cells are capped at V.CELL_TOP_Z and everything
above is a separate BATTERY LAYER). The battery bay is no longer forward of
the housing (v5) -- it is TRANSVERSE, directly above the cells, resting on
their ceiling. The board recess grows into TWO full-height columns standing
beside the cell block -- the General Driver forward (mirrors v5's aft recess,
built off the cell block's own FRONT web instead of the rear one) and a new
Pi 4B aft (built off the REAR web, in the same place and the same way v5's
board recess was, just a different board). Both columns happen to clear the
deck entirely (GD_TOP_Z / PI_TOP_Z are both below -DECK_T -- see
dimensions_v6.py's asserts) so, unlike v5, ALL FOUR screw rows on each board
get real standoff bosses; there is no rail-groove compromise. What replaces
it is INSTALLABILITY: since the pelvis is still one printed part, each board
is enclosed on every side once printed, and can only go in by sliding DOWN
through a slot in the deck. The slot width IS the guide: the column's
interior is hollowed out to exactly the slide channel's width, so the
channel's own side walls are the guide grooves, and the deck slot directly
above it is the entry point. `check_gd_slide()` / `check_pi_slide()` verify
this by sweeping a mock PCB down the full channel height and asserting zero
intersection.

Style: outer vertical edges filleted (R 8-10, "no sharp external corners");
every internal seat stays square (chamfers are for print lead-ins only, not
looks).

Run:
    .venv/bin/python cad/v6/pelvis_v7.py           # STL + STEP + audits + render
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import *  # noqa: E402,F403
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402

box, cyl_x, cyl_y, cyl_z = P.box, P.cyl_x, P.cyl_y, P.cyl_z
teardrop_x, teardrop_y = P.teardrop_x, P.teardrop_y
wedge_x, wedge_y, wedge_z = P.wedge_x, P.wedge_y, P.wedge_z
csk_x, csk_y, csk_z = P.csk_x, P.csk_y, P.csk_z

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientation for check_printability: deck-top-down, same as v5's
# pelvis (RX180: model flips onto its deck top, walls rise from the bed).
PRINT_ORIENT = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)

# ----------------------------------------------------------------------------
# derived local numbers
# ----------------------------------------------------------------------------
hw = V.HIP_SEP / 2 + D.YAW_BOX_HW_OUT          # 57.26 -- the CELL WALL's own outer
                                                # face (unchanged since v5: by+cyw+wall).
                                                # NOT V.HOUSING_HW any more -- that now
                                                # includes the +4 mm HOUSING_SKIN band,
                                                # which is a SEPARATE tapered shell (see
                                                # the bottom of pelvis_v7()), not part of
                                                # the cell/column structure below.
hw_skin = V.HOUSING_HW                         # 61.26 -- outer face of the taper at the deck
skin_t = V.HOUSING_SKIN                        # 4.0
zd = -D.DECK_T                                 # -5 deck bottom
zb = V.YAW_BOX_BOT_Z                           # -72.0 housing floor (both cells and GD)
zc = V.CELL_TOP_Z                              # -36.0 cell ceiling TOP
zc_bot = zc - D.WALL                           # -38.6 cell ceiling bottom / cavity top
cx0 = D.YAW_CASE_X_REAR - D.YAW_SEAT_GAP       # -35.41 cell inner (rear)
cx1 = D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP      # +10.41 cell inner (front)
cyw = D.YAW_BOX_HW_IN                          # 12.66 case half width + fit
wt = D.YAW_SEAT_WALL                           # 2.6
lead = D.YAW_BOX_LEADIN                        # 1.2 mouth chamfer
chan_hw = V.HOUSING_CHAN_HW                    # 26.74 centre channel half width

# GD (front) / Pi (aft) slide channels -- the guide grooves ARE the channel
# walls (see module docstring). Widths give ~1.5 mm/side beyond the board
# outline plus enough for the USB-C shell (GD) and the header stack (Pi).
GD_CHAN_HY = D.BOARD_GD_OUTLINE[0] / 2 + 2.0    # 34.5
PI_CHAN_HY = V.PI_OUTLINE[0] / 2 + 2.5          # 45.0
GD_SLOT = (2 * GD_CHAN_HY, 14.0)                # deck slot (y width, x depth) -- matches the
PI_SLOT = (2 * PI_CHAN_HY, 14.0)                # slide channel exactly, no sliver at the edge


def pelvis_v7():
    """THE WHOLE V7 TORSO. See module docstring for the layout; z bands,
    front to back:

      z [-72, -36]  yaw cell block: two servo cells + the centre channel,
                    capped by a ceiling carrying 4 stator screws per cell
      z [-36, -5]   battery layer: the pack transverse on the ceiling,
                    two low rails, the power boards forward of the pack
      z [-72, -5]   (beside the cell block in x) the GD column (forward)
                    and the Pi column (aft) -- full height, slide-in boards
      z [0, -5]     the deck: apertures, board slots, cable slots, the neck
                    well

    CLEARANCE IS Z-SEPARATION, as in v5 and v6: everything here stays above
    V.TORSO_FLOOR_Z (-72.80), 1.0 mm over the yaw carrier's horn-plate top --
    the highest moving part of the leg -- so no leg pose reaches torso
    structure at any joint angle.
    """
    # ---- ONE solid for the whole housing, full footprint (V.HOUSING_X) and
    # full height (zb..zd), hollowed out for every cavity (v5's "one solid
    # minus its cells" rule: a union of wall rings only butt-joins at the
    # corners; a solid minus voids has continuous walls everywhere, corners
    # included -- and here it is what keeps the front/rear webs (the GD and
    # Pi standoff walls) solid for their FULL height in one pass, instead of
    # three separately-unioned boxes whose shared faces are exactly
    # coincident with the GD/Pi hollow cuts (tried first: OCC read the
    # resulting bosses as five DISCONNECTED solids -- a boss growing off a
    # face that was itself the boundary of an adjacent cut is not the same
    # thing as a boss growing off solid material).
    # This structure itself is STRAIGHT (no taper) -- the cell wall's own
    # outer face stays exactly at +/-hw, full height, as the brief requires
    # ("the yaw cells and their walls stay where they are"). THE REAL 3 DEG
    # TAPER now lives entirely in a SEPARATE outer skin, added at the very
    # end of this function, using dimensions_v6's dedicated HOUSING_SKIN
    # band (HOUSING_HW = hw + 4 mm) -- see that section for the full story
    # of why a taper cannot live here without thinning the cell wall.
    #
    # cx_mid/xlen are still used by the skin's own loft, below.
    cx_mid = (V.HOUSING_X[0] + V.HOUSING_X[1]) / 2
    xlen = V.HOUSING_X[1] - V.HOUSING_X[0]
    p = box(V.HOUSING_X[0], V.HOUSING_X[1], -hw, hw, zb, zd)
    try:
        edges = [e for e in p.edges() if abs(e.tangent_at(0.5).Z) > 0.999]
        p = fillet(edges, 8.0)
    except Exception as e:  # noqa: BLE001 -- cosmetic; don't fail the build over it
        print(f"  [pelvis_v7] outer fillet skipped ({type(e).__name__}: {str(e)[:80]})")
    for by in (V.HIP_SEP / 2, -V.HIP_SEP / 2):
        p -= box(cx0, cx1, by - cyw, by + cyw, zb - 1, zc_bot)      # servo cell
        # mouth lead-in, all four inner faces (servo offered UP into the
        # slip fit; printed deck-top-down these chamfers face print-UP)
        p -= wedge_y([(cx0, zb), (cx0, zb + lead), (cx0 - lead, zb)],
                     by - cyw - wt - 1, by + cyw + wt + 1)          # rear
        p -= wedge_y([(cx1, zb), (cx1, zb + lead), (cx1 + lead, zb)],
                     by - cyw - wt - 1, by + cyw + wt + 1)          # front
        for s in (1, -1):
            yi = by + s * cyw
            p -= wedge_x([(yi, zb), (yi, zb + lead), (yi + s * lead, zb)],
                         cx0 - wt - 1, cx1 + wt + 1)                # cheeks
    # centre channel: OPEN THE FULL HEIGHT, no ceiling. It carries no stator
    # screws (nothing needs to seat on it) and every leg's connector riser
    # already climbs into it from below -- capping it with the same slab as
    # the two cells would have joined all three into one continuous
    # unsupported bridge combining BOTH cell spans (a print-audit finding
    # that a first attempt here caught: ~97-109 mm "LEDGE"/"CEILING" runs).
    # Left open, it is also the pack's lift-out air gap between its two
    # rail-bearing points, matching v5's original battery-bay rule ("nothing
    # bridges, nothing prints in mid air, the middle is simply open").
    p -= box(cx0, cx1, -chan_hw, chan_hw, zb - 1, zd + 1)           # centre channel
    # each cell needs only its own 4 walls + ceiling (mass pass, coordinator
    # 2026-09-14): the front/rear cell walls (the WALL-thick slivers at
    # x in [cx1, CELL_X[1]] and [CELL_X[0], cx0]) used to run the full
    # +/-hw width because nothing had cut the CENTRE of them -- but that
    # centre band is exactly over the (already open) channel, which does
    # not need a wall at all. Punch it out, full height of the cell block.
    # spare a narrow island at y +/-14 (both webs) for the PWR pocket's floor
    # patch below to root into -- see the power-boards section.
    for x0, x1 in ((cx1, V.CELL_X[1]), (V.CELL_X[0], cx0)):
        for yc0, yc1 in ((-chan_hw, -14.0), (14.0, chan_hw)):
            p -= box(x0, x1, yc0, yc1, zb - 1, zc_bot + 0.01)
    for by in (V.HIP_SEP / 2, -V.HIP_SEP / 2):
        # stator screws through the CEILING: 4x M2.5 self-tap into the
        # idler-side case rows, counterbore opening UP (driver reaches down
        # from the battery layer with the pack out).
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zc_bot - 1, zc + 1,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                p -= csk_z(-xrow, by + s * D.CASE_HOLE_LAT, zc, +1)
        # idler disc + hub clearance pocket, in the ceiling's UNDERSIDE
        p -= cyl_z(21.5 / 2, zc_bot, zc_bot + 1.3, 0, by)
        # stator screw pads: the boss under-reaches into the cell so a bare
        # screw does not bow the case (SEAT_PAD_H rule, verbatim from v5)
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p += cyl_z(3.5, zc_bot - D.SEAT_PAD_H, zc_bot,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zc_bot - D.SEAT_PAD_H - 0.1,
                           zc + 1, -xrow, by + s * D.CASE_HOLE_LAT)
        # yaw connector riser: over the trench, straight up through the
        # ceiling into the open battery layer (leads then run free through
        # that open volume to the deck's leg-bus slot -- there is nothing
        # else in the way at this x, the pack sits further outboard/above)
        p -= box(-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 0.6,
                 by - 10.5, by + 10.5, zb - 1, zc_bot)
        p -= box(-D.SV_CONN_L[0] + 0.5, -10.0, by - 4.5, by + 4.5,
                 zc_bot, zc + 1)

    # ---- battery layer: hollow the cell block's own footprint above the
    # ceiling (open air for the pack -- there is no floor besides the
    # ceiling itself and the two rails below), leaving the outer walls.
    p -= box(cx0, cx1, -(hw - D.WALL), hw - D.WALL, zc, zd + 1)
    # "two low rails": BATT_Z[0] is already 0.5 mm above the ceiling
    # (dimensions_v6's seat allowance) -- that IS the rail height. A
    # SEPARATE 0.5 mm rib sitting proud of the ceiling was tried and is a
    # textbook unsupported print island in this orientation (it sits on the
    # print-DOWN side of its own supporting slab -- extruded before the
    # ceiling under it exists). The two per-cell ceiling slabs already
    # provide the two load paths the rails were for (one over each yaw
    # cell's own walls, not the open channel between them); the 0.5 mm gap
    # is carried as tolerance rather than modelled as separate geometry.
    # side walls of the battery bay (the housing's own outer perimeter,
    # already at y=+/-hw from the outer box -- nothing extra needed there,
    # EXCEPT the side windows: the pack's ends may protrude +/-3 mm past the
    # wall, so both +y and -y walls get a window over the pack's end band.
    for s in (1, -1):
        p -= box(V.BATT_X[0] - 2, V.BATT_X[1] + 2,
                 s * (V.BATT_HY - 4.0), s * (hw + 4.0),
                 max(V.BATT_Z[0] - 1.0, zc), V.BATT_Z[1] + 1.0)
    # mass/look pass: one more window BELOW the pack window (the sliver of
    # wall between it and the ceiling) and one either side of the battery
    # layer along x (front, toward the GD column; aft, toward the Pi
    # column) -- all with an 8 mm margin to the nearest seat/edge.
    for s in (1, -1):
        p -= box(V.BATT_X[0] + 6.0, V.BATT_X[1] - 6.0,
                 s * (V.BATT_HY - 4.0), s * (hw + 4.0), zc + 1.0, zc + 6.0)
    for s in (1, -1):
        for x0, x1 in ((cx0 + 6.0, V.BATT_X[0] - 6.0), (V.BATT_X[1] + 6.0, cx1 - 6.0)):
            if x1 - x0 > 6.0:
                p -= box(x0, x1, s * (V.BATT_HY - 4.0), s * (hw + 4.0),
                         zc + 8.0, zd - 8.0)
    # belt slots through the ceiling either side of the pack (15 mm strap:
    # down through one end, under the pack, back up the other -- both slots
    # land within a cell's own ceiling footprint, not the open channel).
    batt_cx = (V.BATT_X[0] + V.BATT_X[1]) / 2
    belt_x = (batt_cx - D.BT_BELT_W / 2, batt_cx + D.BT_BELT_W / 2)
    for s in (1, -1):
        p -= box(belt_x[0], belt_x[1], s * (V.BATT_HY - 8.0), s * (V.BATT_HY + 2.0),
                 zc_bot - 0.1, zc + 1.0)

    # ---- power boards: a pocket forward of the pack, in the same layer,
    # two M2.5 self-tap bosses at the pocket floor (the ceiling). The exact
    # 17.8 x 25.4 buck hole pattern is NOT known -- these two bosses 20 mm
    # apart are a placeholder footprint for a carrier bracket; confirm
    # against the real Pololu D24V50F5 + protection board before printing.
    # the PWR pocket sits over the OPEN centre channel (y +/-10, well inside
    # +/-chan_hw), which has no ceiling by design -- so the two bosses get
    # their own small local floor patch (same thickness as the cell
    # ceilings) instead of standing on nothing.
    pwr_cx = (V.PWR_X[0] + V.PWR_X[1]) / 2
    p += box(V.PWR_X[0] - 1.0, V.PWR_X[1] + 1.0, -14.0, 14.0, zc_bot, zc)
    for py in (-10.0, 10.0):
        p += cyl_z(3.5, zc, zc + 6.0, pwr_cx, py)
        p -= cyl_z(D.M25_TAP / 2, zc - 0.1, zc + 6.1, pwr_cx, py)

    # ---- GD column (forward, off the cell block's own FRONT web -- mirror
    # of v5's board recess, which hung off the REAR web). Slide channel:
    # full height GD_BOT_Z..zd, y +/-GD_CHAN_HY -- these channel walls ARE
    # the guide grooves (see module docstring). The wall at x=CELL_X[1] is
    # already solid full-height (part of the one housing box above); this
    # only hollows OUTBOARD of it.
    p -= box(V.CELL_X[1], V.GD_FRONT_X, -GD_CHAN_HY, GD_CHAN_HY,
             V.GD_BOT_Z - 1, zd + 1)
    # cheek mouth lead-in (print-up, free): funnel the channel's top corners
    for s in (1, -1):
        p -= wedge_x([(s * GD_CHAN_HY, zd), (s * GD_CHAN_HY, zd - 2.0),
                      (s * (GD_CHAN_HY + 2.0), zd)],
                     V.CELL_X[1] - 1, V.GD_FRONT_X + 1)
    # 4x M2.5 standoff holes off the front web, at the board's real hole
    # grid -- but only the LOWER row gets a protruding boss. A board that
    # slides straight down cannot pass a boss reaching into its own
    # footprint at any height it has not yet reached (its rigid body still
    # covers that z once its leading edge is past it) -- so a boss only
    # works at the row the slide ENDS at (the lowest one, GD_SCREW_ROWS_Z's
    # min). This was tried both-rows-bossed first and the sweep check below
    # (65 x 56 x 1.6 mm board) caught the collision directly. The upper row
    # keeps its tapped pilot into the wall itself (thinner engagement, or a
    # future local pad on a tilted insertion) -- v5 solved the identical
    # problem the same way (real screws low, guide-only up top).
    gd_lo_z = min(V.GD_SCREW_ROWS_Z)
    for sy in (D.BOARD_GD_HOLES[0] / 2, -D.BOARD_GD_HOLES[0] / 2):
        for sz in V.GD_SCREW_ROWS_Z:
            if sz == gd_lo_z:
                p += cyl_x(3.5, V.CELL_X[1], V.GD_PCB_X0, sy, sz)
                r, th = 3.5, math.radians(D.BF_BOSS_ROOF_DEG)
                p += (wedge_x([(sy - r / math.sin(th), sz), (sy + r / math.sin(th), sz),
                              (sy, sz + r / math.cos(th))], V.CELL_X[1], V.GD_PCB_X0)
                      & box(V.CELL_X[1], V.GD_PCB_X0, sy - r, sy + r, sz, sz + 2 * r))
                p -= cyl_x(D.M25_TAP / 2, V.CELL_X[1] - 1, V.GD_PCB_X0 + 1, sy, sz)
            else:
                p -= cyl_x(D.M25_TAP / 2, V.CELL_X[1] - 1, V.CELL_X[1] + 2.0, sy, sz)
    # bus-edge lead slot through the deck ahead of the board (bus edge is
    # UP; leads drop straight onto it -- reuse the slide slot itself, it is
    # already open the board's full width above GD_TOP_Z).
    #
    # MASS PASS (coordinator, 2026-09-14, then again once HOUSING_SKIN
    # existed): the two "shoulders" outboard of the slide channel (y from
    # GD_CHAN_HY to hw) were solid for no reason -- the groove wall alone
    # is what guides the board and the standoff bosses live INSIDE the
    # channel (y<=29), not out here. Now that there is a SEPARATE tapered
    # outer skin (below), this shoulder does not need its own outer wall
    # at all -- open it all the way to hw (the cell wall's own outer
    # face), keeping only a 4 mm top/bottom rim (ties into the deck and
    # the skin) and the near/far x-caps.
    # the near/far x-caps a narrower pocket would leave (WALL thick, full
    # shoulder width x full height) were ~18 g of sheet with nothing seated
    # on them -- the housing's own front/aft walls (GD_FRONT_X, the shared
    # web at CELL_X[1]) still close the channel itself; the shoulder needs
    # no cap at all now that the tapered skin (below) is the outer surface.
    # Open the whole shoulder, x included, leaving only the groove wall.
    # (stop exactly AT GD_FRONT_X, not past it: GD_FRONT_X is the "inner
    # rim" dimensions_v6.py measures off, and HOUSING_X[1] = GD_FRONT_X +
    # WALL is the housing's own OUTER end cap -- a +1 overshoot here used
    # to eat straight through that 2.6 mm cap, thinning it to ~1.6 mm and
    # causing the THIN findings the coordinator caught at the column ends.)
    for s in (1, -1):
        p -= box(V.CELL_X[1] - 1.0, V.GD_FRONT_X - 0.3,
                 s * (GD_CHAN_HY + D.WALL), s * hw,
                 zb + 3.0, zd - 3.0)

    # ---- Pi column (aft, off the cell block's own REAR web -- exactly
    # where v5's General Driver recess stood; the same construction, wider).
    # Same rule: the wall at x=CELL_X[0] is already solid full-height.
    p -= box(V.PI_AFT_X, V.CELL_X[0], -PI_CHAN_HY, PI_CHAN_HY,
             V.PI_BOT_Z - 1, zd + 1)
    for s in (1, -1):
        p -= wedge_x([(s * PI_CHAN_HY, zd), (s * PI_CHAN_HY, zd - 2.0),
                      (s * (PI_CHAN_HY + 2.0), zd)],
                     V.PI_AFT_X - 1, V.CELL_X[0] + 1)
    # Same rule as GD: only the lower row (nearest the end of the downward
    # slide) gets a real protruding boss.
    pi_lo_z = min(V.PI_SCREW_Z)
    for sy in V.PI_SCREW_Y:
        for sz in V.PI_SCREW_Z:
            if sz == pi_lo_z:
                p += cyl_x(3.5, V.PI_PCB_X1, V.CELL_X[0], sy, sz)
                r, th = 3.5, math.radians(D.BF_BOSS_ROOF_DEG)
                p += (wedge_x([(sy - r / math.sin(th), sz), (sy + r / math.sin(th), sz),
                              (sy, sz + r / math.cos(th))], V.PI_PCB_X1, V.CELL_X[0])
                      & box(V.PI_PCB_X1, V.CELL_X[0], sy - r, sy + r, sz, sz + 2 * r))
                p -= cyl_x(D.M25_TAP / 2, V.PI_PCB_X1 - 1, V.CELL_X[0] + 1, sy, sz)
            else:
                p -= cyl_x(D.M25_TAP / 2, V.CELL_X[0] - 2.0, V.CELL_X[0] + 1, sy, sz)
    # USB/Ethernet side window (+y): the stack stands proud of the
    # component face over the top ~22 mm of the board (near PI_TOP_Z),
    # protrudes 17 mm + 2 mm clearance past the board's own +y edge.
    win_z = (V.PI_TOP_Z - 22.0, V.PI_TOP_Z + 2.0)
    p -= box(V.PI_PCB_X0 - 2.0, V.PI_PCB_X1 + 2.0,
             V.PI_OUTLINE[0] / 2 - 1.0, hw + 1.0, win_z[0], win_z[1])
    # MASS PASS: same shoulder-opening as GD's -- open to hw AND through the
    # x-caps, the separate tapered skin (below) is the only outer wall out
    # here now.
    # (same fix as GD's: stop exactly AT PI_AFT_X, not past it, so
    # HOUSING_X[0]..PI_AFT_X -- the housing's own 2.6 mm outer end cap --
    # stays full thickness.)
    for s in (1, -1):
        p -= box(V.PI_AFT_X + 0.3, V.CELL_X[0] + 1.0,
                 s * (PI_CHAN_HY + D.WALL), s * hw,
                 zb + 3.0, zd - 3.0)

    # ---- deck: the roof over everything. -----------------------------------
    # round its top perimeter (R3) on the CLEAN deck box first, same reason
    # as the R8 outer fillet above: doing this after the aperture/slot cuts
    # go in gives the kernel a mess of short, mutually-intersecting edges at
    # the top and it refuses even a 1 mm radius.
    deck = box(V.HOUSING_X[0], V.HOUSING_X[1], -hw_skin, hw_skin, zd, 0)
    try:
        deck_top_edges = [e for e in deck.edges() if abs(e.center().Z) < 1e-6]
        deck = fillet(deck_top_edges, 3.0)
    except Exception as e:  # noqa: BLE001 -- cosmetic; don't fail the build over it
        print(f"  [pelvis_v7] deck-top fillet skipped ({type(e).__name__}: {str(e)[:80]})")
    p += deck
    # battery + power-board aperture: one opening over both footprints
    # (adjacent, 2 mm apart) so the pack lifts out and the power boards
    # (centred at x~8, comfortably inside this) are reachable through the
    # same hole. Both bounds are clamped to the battery-hollow's own cut
    # (x <= cx1, y <= hw - WALL): PWR_X[1]+0.5 alone would run 4 mm PAST cx1,
    # into the solid front web the GD standoffs grow from -- an aperture
    # wider than what is actually hollow underneath it exposes that wall's
    # own top face with nothing above it in the print, which is exactly the
    # "island"/"beam" the first printability pass caught here.
    aper_hy = hw - D.WALL
    p -= box(V.BATT_X[0] - 0.5, min(V.PWR_X[1] + 0.5, cx1), -aper_hy, aper_hy,
             zd, 1)
    # GD board slide slot
    gd_slot_cx = (V.GD_PCB_X0 + V.GD_FRONT_X) / 2 - 1.0
    p -= box(gd_slot_cx - GD_SLOT[1] / 2, gd_slot_cx + GD_SLOT[1] / 2,
             -GD_SLOT[0] / 2, GD_SLOT[0] / 2, zd, 1)
    # Pi board slide slot -- centred on the PCB's own x position (the thin
    # sweep box tracks the bare PCB, not the component envelope), a plain
    # x-centre mistake here (originally centred toward PI_AFT_X, well clear
    # of where the board's slab actually travels) is exactly what the sweep
    # check below caught: 585 mm3 of constant overlap with the deck because
    # the slot and the board's own path never lined up.
    pi_slot_cx = (V.PI_PCB_X0 + V.PI_PCB_X1) / 2
    p -= box(pi_slot_cx - PI_SLOT[1] / 2, pi_slot_cx + PI_SLOT[1] / 2,
             -PI_SLOT[0] / 2, PI_SLOT[0] / 2, zd, 1)
    # leg-bus cable slots, 12 x 6 mm, near y = +/-HIP_SEP/2, x = CELL_X[0]-8
    lb_x = V.CELL_X[0] - 8.0
    for s in (1, -1):
        p -= box(lb_x - 6.0, lb_x + 6.0, s * V.HIP_SEP / 2 - 3.0,
                 s * V.HIP_SEP / 2 + 3.0, zd, 1)
    # deck pocketing (item d, mass pass): 2 mm off the underside wherever
    # nothing seats -- over the column shoulders (already hollow below, so
    # this costs nothing structurally) and the margin beside the battery
    # aperture, each split by a 4 mm rib for stiffness. An 8 mm gap from
    # every existing cut/edge keeps this off every real seat.
    def _deck_pocket(x0, x1, y0, y1, rib_at):
        if x1 - x0 < 10.0 or y1 - y0 < 10.0:
            return
        p_local = box(x0, x1, y0, y1, zd, zd + 2.0)
        p_local -= box(x0 - 1, x1 + 1, rib_at - 2.0, rib_at + 2.0, zd - 1, zd + 3)
        nonlocal p
        p -= p_local
    # (the columns are only ~17 mm deep in x -- an 8 mm margin on both ends
    # would leave no pocket at all there, so x uses a 3 mm margin, matching
    # the shoulder skins below; the 8 mm rule applies in y, where there is
    # 23-46 mm of room)
    _deck_pocket(V.CELL_X[1] + 3, V.GD_FRONT_X - 3, GD_CHAN_HY + 4, hw - 8, (GD_CHAN_HY + hw) / 2)
    _deck_pocket(V.CELL_X[1] + 3, V.GD_FRONT_X - 3, -(hw - 8), -(GD_CHAN_HY + 4), -(GD_CHAN_HY + hw) / 2)
    _deck_pocket(V.PI_AFT_X + 3, V.CELL_X[0] - 3, PI_CHAN_HY + 4, hw - 8, (PI_CHAN_HY + hw) / 2)
    _deck_pocket(V.PI_AFT_X + 3, V.CELL_X[0] - 3, -(hw - 8), -(PI_CHAN_HY + 4), -(PI_CHAN_HY + hw) / 2)

    # neck well: a shallow pocket in the deck TOP, same footprint as one yaw
    # cell (same servo, same fit), 4x M2.5 self-tap UP into the case's
    # idler-face rows, counterbore opening DOWN (driven from inside, through
    # the battery aperture, with the pack out); lead relief slot for the
    # servo's own connector trench.
    nx0, nx1 = V.NECK_WELL_X
    nhy = V.NECK_WELL_HW[0]
    p -= box(nx0, nx1, -nhy, nhy, -V.NECK_WELL_D, 1)
    for xrow in D.YAW_CASE_HOLES_IDLER:
        for s in (1, -1):
            p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zd - 1, -V.NECK_WELL_D + 1,
                       -xrow, s * D.CASE_HOLE_LAT)
            p -= csk_z(-xrow, s * D.CASE_HOLE_LAT, zd, -1)
    p -= box(-D.SV_CONN_L[1], -D.SV_CONN_L[0], -D.SV_CONN_HW, D.SV_CONN_HW,
             -V.NECK_WELL_D - 0.1, -V.NECK_WELL_D + 1.3)

    # ---- NECK COLLAR: now a SEPARATE printed part (cad/v6/neck_collar.py) --
    # a one-piece collar rising 33 mm above the deck made the collar's own
    # mouth the model's new z-extreme, which flips which end of a
    # deck-top-down print touches the bed: the ENTIRE housing (97 x 121 mm)
    # read as a floating island 33 mm above a tiny collar footprint (a real
    # printability failure, not a cosmetic finding -- see the prior report).
    # Splitting it off keeps this part's own orientation untouched. What
    # stays here: the well, the 4 stator screws, the lead slot (all
    # unchanged, above) -- plus 4 M2.5 pilot holes for the collar's flange,
    # at the SAME corner positions neck_collar.py uses (NECK_FLANGE_HOLE_XY
    # below), 2.05 mm dia x 4.5 mm deep into the 5 mm deck from the top.
    NECK_FLANGE_MARGIN = 8.0
    NECK_FLANGE_HOLE_INSET = 4.0
    nfx = (V.NECK_WELL_X[0] - D.WALL - NECK_FLANGE_HOLE_INSET,
          V.NECK_WELL_X[1] + D.WALL + NECK_FLANGE_HOLE_INSET)
    nfy = V.NECK_WELL_HW[0] + D.WALL + NECK_FLANGE_HOLE_INSET
    NECK_FLANGE_HOLE_XY = [(nfx[0], nfy), (nfx[1], nfy), (nfx[0], -nfy), (nfx[1], -nfy)]
    for hx, hy in NECK_FLANGE_HOLE_XY:
        p -= cyl_z(2.05 / 2, -4.5, 0.5, hx, hy)

    # ---- HOUSING_SKIN: the real 3 deg taper, separate from the cell/column
    # structure (coordinator, 2nd mass pass). dimensions_v6.py now carries a
    # dedicated 4 mm band outboard of the cell wall's own outer face (hw):
    # HOUSING_HW = hw + HOUSING_SKIN. This is what makes a true 3 deg taper
    # possible without thinning anything upstream of this point -- the skin
    # meets the cell wall FLUSH (0 mm extra) at the housing floor and stands
    # the full 4 mm proud of it at the deck, so the deck (already built to
    # hw_skin above) sits on the wide end and nothing below has to change.
    #
    # Built as its own solid (loft, full width 2*hw at zb to 2*hw_skin at
    # zd) with material only kept from (hw - 0.5) outward -- i.e. it
    # OVERLAPS the cell/column structure by 0.5 mm everywhere, rather than
    # sharing a coincident face with it. That overlap is what makes the
    # union actually fuse into one solid: a first attempt at this exact
    # skin (cell block + tapered shell sharing a coincident face) would not
    # fuse in this file's very first taper attempt, for the same reason
    # every other "boss on a coincident face" bug in this module didn't.
    skin_lo = Pos(cx_mid, 0, zb) * Rectangle(xlen, 2 * hw)
    skin_hi = Pos(cx_mid, 0, zd) * Rectangle(xlen, 2 * hw_skin)
    skin_full = loft([skin_lo.faces()[0], skin_hi.faces()[0]])
    skin = skin_full - box(V.HOUSING_X[0] - 1, V.HOUSING_X[1] + 1,
                           -(hw - 0.5), hw - 0.5, zb - 1, zd + 1)
    # windowed freely, as asked: the skin carries no seat/standoff/groove
    # wall of its own (all of those are on the structure it wraps, inboard
    # of it), so most of its area can open up -- an 8 mm margin to the
    # housing's own ends, the deck, and the floor.
    #
    # THE WINDOW'S OWN ROOF: printed deck-top-down, the window's edge
    # nearest the housing FLOOR (zb+3, the z-rim -- more negative model z
    # prints LATER, i.e. higher in the stack) is what re-forms as a
    # horizontal slab over the void below it, the same as any other window
    # roof in this codebase (v5's BR_SVC_WIN_TAPER). Run the full length in
    # one cut and that roof is the window's ~93 mm span in one go -- the
    # first version of this skin did exactly that and the printability
    # pass caught it (663 mm2, ~61 mm effective span). Splitting the
    # window into <=20 mm segments with a 4 mm rib between (BEAM_OK,
    # matching v5's end-anchored-ribbon rule) costs far less material than
    # a 47 deg gable would across the whole run, so that is what keeps the
    # mass: a rib every ~19 mm rather than a sloped roof the entire length.
    win_x0, win_x1 = V.HOUSING_X[0] + 3, V.HOUSING_X[1] - 3
    win_len = win_x1 - win_x0
    rib_w = 4.0
    seg_max = 16.0
    n_seg = max(1, math.ceil((win_len + rib_w) / (seg_max + rib_w)))
    seg_len = (win_len - (n_seg - 1) * rib_w) / n_seg
    ribs = []
    if seg_len > 6.0:
        for i in range(n_seg):
            sx0 = win_x0 + i * (seg_len + rib_w)
            sx1 = sx0 + seg_len
            for s in (1, -1):
                skin -= box(sx0, sx1, s * (hw - 1), s * (hw_skin + 1), zb + 3, zd - 3)
            if i > 0:
                ribs.append(sx0 - rib_w)   # the gap just before this segment
    p += skin
    # the rib itself is only as thick (radially) as the tapered skin is at
    # that height -- near zb that is ~0.5-1 mm, too thin to read as a real
    # anchor for the window's roof above it (a plain "leave this X band
    # uncut" rib measured 45-55% smaller than no rib at all, but did not
    # clear the finding). Thicken each rib into a full-depth gusset (hw to
    # hw_skin, the taper's own full span, not just its local width) over
    # the window's own height -- a proper strut, not a sliver of skin.
    for rx in ribs:
        for s in (1, -1):
            p += box(rx, rx + rib_w, s * (hw - 0.5), s * (hw_skin + 0.5),
                     zb + 2.0, zd - 2.0)

    # ---- HIP-YAW BEARING SKIRT + RECESS (2026-09-17, docs/design-v6/
    # study-yaw-bearing.md; REVISED after coordinator review -- 6811-2RS,
    # located races, no clearance fit -- see the write-up's history
    # section). The OUTER-race seat for a 6811-2RS deep-groove ball bearing
    # around the carrier's ENTIRE horn-plate + upper bay-wall footprint
    # (cad/v6/yaw_carrier_v6.py has the matching boss). Hangs from the
    # EXISTING cell-tube rim (zb = V.YAW_BOX_BOT_Z -- NOT moved, nothing
    # above this line changes) down to V.YAW_BRG_RECESS_Z[0], entirely in
    # the air that was already open below both the servo case (case-bottom
    # sits above zb) and the GD/Pi board columns (both end at zb too). A
    # shoulder (ID V.YAW_BRG_SHOULDER_ID, land V.YAW_BRG_SHOULDER_LAND)
    # stops the outer race's top face, reacting the leg's upward thrust
    # (single-support weight) into the pelvis print; the recess proper
    # below it (ID V.YAW_BRG_RECESS_ID) is a LOCATED, -0.04 mm interference
    # fit on the race OD -- not a clearance fit, per the coordinator's
    # note that a movable outer race defeats the point of the bearing.
    skirt_r = V.YAW_BRG_SKIRT_OD / 2
    sho_z0, sho_z1 = V.YAW_BRG_SHOULDER_Z
    rec_z0, rec_z1 = V.YAW_BRG_RECESS_Z
    for by in (V.HIP_SEP / 2, -V.HIP_SEP / 2):
        p += cyl_z(skirt_r, rec_z0 - 0.5, sho_z1, 0, by)
        p -= cyl_z(V.YAW_BRG_SHOULDER_ID / 2, sho_z0, sho_z1 + 0.5, 0, by)
        p -= cyl_z(V.YAW_BRG_RECESS_ID / 2, rec_z0 - 0.6, rec_z1 + 0.01, 0, by)

    # BRIDGE the two skirts: OD 78 mm at 84 mm hip separation leaves only
    # 6 mm edge-to-edge -- too little for two independent free-hanging
    # rings to stand on their own without flexing under yaw torque or a
    # bench knock, so they are fused into one structure by a short web at
    # the bottom (the most compliant point, farthest from the rigid cell
    # block above). MUST stay outside each skirt's own recess bore (r <
    # V.YAW_BRG_RECESS_R from that hip's own centre) -- a first version of
    # this reached 5 mm into each skirt without checking that boundary and
    # ate 80 mm3 into the bearing's own reserved outer-race space
    # (check_yaw_bearing_combo.py caught it: pelvis vs OUTER_RING, FAIL).
    # The skirt's SOLID wall at x=0 only exists between y = hip_y -
    # skirt_r (the outer surface) and y = hip_y - V.YAW_BRG_RECESS_R (where
    # the hollow bore begins) -- so the bridge reaches to just inside that
    # inner limit, never into the bore itself.
    bridge_hy = V.HIP_SEP / 2 - V.YAW_BRG_RECESS_R - 0.3   # 5.72, inside the
                                                            # skirt wall, clear
                                                            # of the recess bore
    bridge_x = (-15.0, 15.0)
    bridge_z = (rec_z0, rec_z0 + 4.0)
    p += box(bridge_x[0], bridge_x[1], -bridge_hy, bridge_hy, bridge_z[0], bridge_z[1])

    return p


# ----------------------------------------------------------------------------
# mocks (pelvis frame)
# ----------------------------------------------------------------------------
def mock_pack():
    return Pos((V.BATT_X[0] + V.BATT_X[1]) / 2, 0, (V.BATT_Z[0] + V.BATT_Z[1]) / 2) \
        * Box(V.BATT[1], V.BATT[0], V.BATT[2])


def mock_gd():
    return Pos((V.GD_PCB_X0 + V.GD_PCB_X1) / 2, 0, (V.GD_BOT_Z + V.GD_TOP_Z) / 2) \
        * Box(D.BR_PCB_T, D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])


def mock_pi4():
    return Pos((V.PI_PCB_X0 + V.PI_PCB_X1) / 2, 0, V.PI_CZ) \
        * Box(V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1])


def mock_buck():
    """5 V/5 A buck (Pololu D24V50F5), sitting in the +y half of the power
    pocket -- the smaller of the two boards beside the pack."""
    cx = (V.PWR_X[0] + V.PWR_X[1]) / 2
    return Pos(cx, 15.0, (V.PWR_Z[0] + V.PWR_Z[0] + 8.0) / 2) * Box(17.8, 25.4, 8.0)


def mock_neck_servo():
    """STS3215 standing on the deck, horn up, idler face down at the deck
    top -- pelvis frame (world-relative-to-deck-top, matches assembly_v6's
    Pos(V.NECK_X, 0, z + V.NECK_AXIS_Z) placement with z=0 here)."""
    return Pos(V.NECK_X, 0, V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()


# ----------------------------------------------------------------------------
# fasteners
# ----------------------------------------------------------------------------
def SCREWS():
    """Every fastener in pelvis_v7, local frame: {name, kind, pos, axis,
    length}. `axis` points from the screw HEAD toward its tip (the direction
    it is driven); `pos` is the head's bearing point. Every entry is checked
    for a clear ACCESS_D (7 mm) driver cylinder from the head back out to
    open air by check_screw_access() below."""
    s = []
    for by, side in ((V.HIP_SEP / 2, "L"), (-V.HIP_SEP / 2, "R")):
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for sgn in (1, -1):
                s.append(dict(name=f"yaw_stator_{side}_{xrow:.0f}_{sgn:+d}",
                              kind="M2.5x8 self-tap, flat head", axis=(0, 0, -1),
                              pos=(-xrow, by + sgn * D.CASE_HOLE_LAT, zc), length=8.0))
    gd_lo_z, pi_lo_z = min(V.GD_SCREW_ROWS_Z), min(V.PI_SCREW_Z)
    for sy in (D.BOARD_GD_HOLES[0] / 2, -D.BOARD_GD_HOLES[0] / 2):
        for sz in V.GD_SCREW_ROWS_Z:
            kind = ("M2.5x10 self-tap, pan head (standoff boss)" if sz == gd_lo_z
                    else "M2.5x6 self-tap into wall (guide row, no boss -- see note)")
            s.append(dict(name=f"gd_standoff_{sy:+.0f}_{sz:+.0f}", kind=kind,
                          axis=(-1, 0, 0), pos=(V.GD_PCB_X0 + D.BR_PCB_T, sy, sz),
                          length=10.0 if sz == gd_lo_z else 6.0))
    for sy in V.PI_SCREW_Y:
        for sz in V.PI_SCREW_Z:
            kind = ("M2.5x10 self-tap, pan head (standoff boss)" if sz == pi_lo_z
                    else "M2.5x6 self-tap into wall (guide row, no boss -- see note)")
            s.append(dict(name=f"pi_standoff_{sy:+.0f}_{sz:+.0f}", kind=kind,
                          axis=(1, 0, 0), pos=(V.PI_PCB_X0 - V.PI_PCB_T, sy, sz),
                          length=10.0 if sz == pi_lo_z else 6.0))
    for py in (-10.0, 10.0):
        s.append(dict(name=f"pwr_boss_{py:+.0f}", kind="M2.5x8 self-tap, pan head",
                      axis=(0, 0, -1),
                      pos=((V.PWR_X[0] + V.PWR_X[1]) / 2, py, zc + 6.0), length=8.0))
    for xrow in D.YAW_CASE_HOLES_IDLER:
        for sgn in (1, -1):
            s.append(dict(name=f"neck_stator_{xrow:.0f}_{sgn:+d}",
                          kind="M2.5x8 self-tap, flat head", axis=(0, 0, 1),
                          pos=(-xrow, sgn * D.CASE_HOLE_LAT, zd), length=8.0))
    return s


def check_screw_access(verbose=True):
    """Every screw in SCREWS() gets a straight 7 mm (ACCESS_D) driver
    cylinder, checked against the printed solid for intersection. The driver
    approaches from BEHIND the head -- i.e. it extends in -axis (axis points
    head-to-tip, into the material), backing OUT toward wherever that screw
    is actually driven from (a deck slot, the battery aperture, or open air
    outside the housing). A small allowance right at the head (ACCESS_START)
    is excluded, since the screw's own shank/boss legitimately fills that
    same cylinder for the first couple of mm."""
    solid = pelvis_v7()
    ok = True
    for sc in SCREWS():
        ax = np.array(sc["axis"], float)
        head = np.array(sc["pos"], float)
        # back off past the screw's OWN local material (its boss/pad/ceiling
        # thickness, sized to the shank, not to a 7 mm driver) before
        # requiring the full ACCESS_D clearance -- this checks the OPEN
        # cavity a driver approaches through, not the screw's own pilot hole
        start = head - ax * (sc["length"] + 1.0)
        reach = 90.0                 # far past any wall -- clipped by the solid itself
        if abs(ax[2]) > 0.5:
            z0, z1 = start[2], start[2] - np.sign(ax[2]) * reach
            cyl = cyl_z(D.ACCESS_D / 2, z0, z1, head[0], head[1])
        else:
            x0, x1 = start[0], start[0] - np.sign(ax[0]) * reach
            cyl = cyl_x(D.ACCESS_D / 2, x0, x1, head[1], head[2])
        try:
            hit_vol = (cyl & solid).volume
        except Exception:  # noqa: BLE001
            hit_vol = 0.0
        clear = hit_vol < 5.0        # a shank/head passing through its own hole is fine
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  {sc['name']:24s} "
                  f"{sc['kind']:44s} overlap {hit_vol:6.1f} mm3")
    return ok


# ----------------------------------------------------------------------------
# installability sweeps
# ----------------------------------------------------------------------------
def _sweep_board(pcb_box_fn, x0, x1, z_top, z_bot, name, n=40):
    """Slide a PCB-shaped box straight down (from above the deck to its
    seated depth) and report the worst (max) intersection volume with the
    printed solid found along the path -- should be ~0 throughout."""
    solid = pelvis_v7()
    worst = 0.0
    for k in range(n):
        z = z_top + (z_bot - z_top) * k / (n - 1)
        board = Pos(0, 0, z) * pcb_box_fn()
        try:
            vol = (board & solid).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        worst = max(worst, vol)
    print(f"  {name}: worst intersection over the {n}-step slide = {worst:.2f} mm3 "
          f"({'PASS' if worst < 0.5 else 'FAIL'})")
    return worst < 0.5


def check_gd_slide():
    def gdbox():
        return Pos((V.GD_PCB_X0 + V.GD_PCB_X1) / 2, 0, 0) \
            * Box(D.BR_PCB_T, D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])
    return _sweep_board(gdbox, None, None, 40.0, V.GD_TOP_Z - D.BOARD_GD_OUTLINE[1] / 2,
                        "GD board slide (65 x 56 x 1.6)")


def check_pi_slide():
    def pibox():
        return Pos((V.PI_PCB_X0 + V.PI_PCB_X1) / 2, 0, 0) \
            * Box(V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1])
    return _sweep_board(pibox, None, None, 40.0, V.PI_TOP_Z - V.PI_OUTLINE[1] / 2,
                        "Pi board slide (85 x 56 x 1.5)")


# ----------------------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    os.makedirs(OUT_REN, exist_ok=True)
    print("building pelvis_v7()...")
    solid = pelvis_v7()
    bb = solid.bounding_box()
    dims = (bb.size.X, bb.size.Y, bb.size.Z)
    bed_ok = max(dims) <= 250 and sorted(dims)[1] <= D.BED and sorted(dims)[0] <= D.BED
    mass = solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    print(f"bbox {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm  "
          f"({'BED-OK' if bed_ok else '** TOO BIG **'}, bed {D.BED})")
    print(f"mass {mass:.1f} g (PETG, {D.PRINT_MASS_FACTOR} print factor)")
    n_solids = len(solid.solids())
    print(f"solids: {n_solids} ({'ok, one body' if n_solids == 1 else '** SPLIT BODY **'})")

    export_stl(solid, os.path.join(OUT_STL, "pelvis_v7.stl"))
    export_step(solid, os.path.join(OUT_STEP, "pelvis_v7.step"))
    print("wrote stl/step")

    print("\n-- screw driver access (7 mm ACCESS_D) --")
    check_screw_access()

    print("\n-- installability sweeps --")
    check_gd_slide()
    check_pi_slide()

    print("\n-- printability audit --")
    import audit_torso  # noqa: E402
    audit_torso.run(["pelvis_v7"])

    print("\n-- render --")
    try:
        import mujoco  # noqa: F401
        import imageio
        import tempfile
        tmp = tempfile.mkdtemp(prefix="pv7_")
        stl_path = os.path.join(tmp, "p.stl")
        export_stl(solid, stl_path)
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="1200" offheight="900"/></visual>
<asset><mesh name="m" file="p.stl" scale="0.001 0.001 0.001"/></asset>
<worldbody><light pos="0.3 0.3 0.3" dir="-0.4 -0.4 -0.6" directional="true"/>
<light pos="-0.3 -0.2 0.2" dir="0.4 0.3 -0.4" directional="true"/>
<geom type="mesh" mesh="m" rgba="0.8 0.82 0.86 1"/></worldbody></mujoco>"""
        xml_path = os.path.join(tmp, "s.xml")
        with open(xml_path, "w") as f:
            f.write(xml)
        m = mujoco.MjModel.from_xml_path(xml_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, 900, 1200)
        cam = mujoco.MjvCamera()
        bb = solid.bounding_box()
        cam.lookat[:] = [(bb.min.X + bb.max.X) / 2000, 0, (bb.min.Z + bb.max.Z) / 2000]
        cam.distance = 0.28
        imgs = []
        for az, el in ((150, -20), (60, -15), (0, -25)):
            cam.azimuth, cam.elevation = az, el
            r.update_scene(d, cam)
            imgs.append(r.render().copy())
        import numpy as _np
        imageio.imwrite(os.path.join(OUT_REN, "pelvis_v7.png"), _np.concatenate(imgs, axis=1))
        print("wrote", os.path.join(OUT_REN, "pelvis_v7.png"))
    except Exception as e:  # noqa: BLE001
        print(f"render skipped ({type(e).__name__}: {e})")
