"""v6 body -- single source of truth for every NEW or CHANGED dimension (mm).

Everything the v5 parts already pinned about the servo (case, horn/idler
discs, screw rows, rib/platform detents, seat faces, countersinks, walls,
fits) is imported unchanged from ../dimensions.py as `D` and re-exported; a
v6 part must take a servo-interface number from `D`, never retype it. What
changes for v6 is the KINEMATICS (ankle roll, 84 mm hips, 110 mm segments),
the FOOT, the TORSO (Pi 4B, bigger pack, neck + head) and the LEG LINK's
section. Design record: docs/design-v6-ankle-roll.md; the numbers below are
the ones the gates were run with (sim/gen_plant_v6.py DesignParams must
agree -- tests/test_v6_design_gates.py pins the shared ones).

Frames (same as v5): +X forward, +Y robot-left, +Z up. Every pitch joint has
the servo HORN on +Y; the same part serves both legs by TRANSLATION except
the FOOT, which is asymmetric (sole biased outboard) and comes as a mirrored
pair. Part-local frames: the part's upper joint axis through the origin.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import dimensions as D  # noqa: E402  (v5 -- servo truth, walls, fits, screws)

# ----------------------------------------------------------------------------
# servos
# ----------------------------------------------------------------------------
# STS3250 == STS3215 case, horn, idler, screw rows (vendor: 45.22 x 24.72 x
# 35 mm, same 4x M3 on O14 discs). Only the mass differs. Assignment per the
# option study (docs/design-v6-ankle-roll.md section 8):
SERVO_3250_JOINTS = ("hip_roll", "ankle_roll", "knee")        # 6x STS3250
SERVO_3215_JOINTS = ("hip_yaw", "hip_pitch", "ankle_pitch", "neck")   # 7x STS3215
SERVO_MASS_3215 = 55.0
SERVO_MASS_3250 = 74.5

# ----------------------------------------------------------------------------
# kinematics
# ----------------------------------------------------------------------------
HIP_SEP = 84.0          # leg centre-to-centre (v5: 66). 24+ mm between the
                        # feet's inner edges even with the wider soles below,
                        # and a battery-wide channel between the yaw cells
ROLL_TO_PITCH = D.ROLL_TO_PITCH        # 50, hip yokes unchanged
THIGH = 110.0           # hip-pitch axis -> knee axis (v5: 90)
SHIN = 110.0            # knee axis -> ankle-PITCH axis (v5: 90)
LINK_DROP = THIGH       # leg_link_v6 upper axis -> lower axis (== THIGH == SHIN)
# Ankle: the pitch servo HANGS below the pitch axis inside the ankle link
# (every joint on the robot is built this way), and the ankle link forks in
# the X-Z plane down onto the ROLL servo, which lies across the foot with its
# output axis fore-aft. 57, not the hip's 50: the roll servo lies across the
# foot (case length along Y, cable end OUTBOARD), so as the ankle rolls its
# TOP EDGE rises; the point of that edge directly under the pitch servo's
# outboard case face (y = SV_TOPFACE = 17.35) rises 12.36*cos(phi) +
# 17.35*sin(phi) above the roll axis = 18.5 at ANKLE_ROLL_ROM 25 deg, and the
# pitch servo's case bottom / the grip plates sit at ANKLE_PITCH_TO_ROLL - 36
# above the roll axis -> 21.0, i.e. 2.5 mm of air. (First cut used the case's
# 10.11 inboard END corner and got 54; the assembly sweep found the two
# servo bodies touching at 18 deg -- cad/v6/check_assembly_v6.py, 2026-09-14.)
ANKLE_PITCH_TO_ROLL = 58.0   # 57 left the roll and pitch servo bodies 0.15 mm apart at 25 deg (assembly sweep); 58 gives ~1.1
_roll_rise_25 = D.SV_WID / 2 * math.cos(math.radians(25)) + D.SV_TOPFACE * math.sin(math.radians(25))
assert ANKLE_PITCH_TO_ROLL - 36.0 - _roll_rise_25 >= 2.0, "roll servo top edge reaches the pitch servo at 25 deg"
FOOT_PLATE_T = 4.0      # printed sole plate (v5 FOOT_T 6 with a 2 mm pocket)
TPU_SOLE_T = 2.0        # printed TPU 95A sole glued under the plate
# roll servo lies on the plate top, output axis along +X (horn FORWARD),
# case width (24.72) vertical -> axis centred 12.36 above the plate top
ANKLE_ROLL_ABOVE_PLATE = FOOT_PLATE_T + D.SV_WID / 2                 # 16.36
ANKLE_ROLL_Z = ANKLE_ROLL_ABOVE_PLATE + TPU_SOLE_T                   # 18.36 above ground
ANKLE_PITCH_Z = ANKLE_ROLL_Z + ANKLE_PITCH_TO_ROLL                   # 72.36
KNEE_Z = ANKLE_PITCH_Z + SHIN                                        # 182.36
HIP_PITCH_Z = KNEE_Z + THIGH                                         # 292.36
HIP_ROLL_Z = HIP_PITCH_Z + ROLL_TO_PITCH                             # 342.36
HIP_YAW_Z = HIP_ROLL_Z - D.CARRIER_ROLL_AXIS                         # yaw horn face
DECK_BOT_Z = HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW                      # v5 stack...
# ...plus the v7 torso's battery layer (see TORSO) -- DECK_BOT_Z is redefined below.

# joint ranges (PHYSICAL rotations, one leg; the interference sweeps probe
# SIGN NOTE: the assembly rotates every pitch joint about +Y, so a POSITIVE
# knee here is human flexion (shin swings back) while the sim's knee axis is
# -Y and flexion is NEGATIVE there (gen_plant_v6: knee_flex). Hip pitch has
# the same sign in both (negative = thigh forward).
# both extremes with D.SWEEP_BUFFER of air). Hip yaw/roll/pitch and knee are
# v5's measured ROM; the ankle pitch stays +-40 (the shin fork vs the ankle
# link's grip plates is the same pair as the knee); ankle roll +-25 is what
# the roll servo's case sweep allows under the ankle link (above) and 2x
# what the walk uses (12 deg).
ROM = {
    "hip_yaw": (-45.0, 45.0),
    "hip_roll": (-55.0, 55.0),
    "hip_pitch": (-125.0, 90.0),   # was -110: yoke_pitch_v6 flange chamfer (YOKE_FLEX_CHAMFER)
    "knee": (-95.0, 130.0),        # + = flexion (see the sign note); 95 -> 130 with LL_FLEX_CUT + LL_FOLD_CHAMFER; hard limit ~132
    "ankle_pitch": (-40.0, 40.0),
    "ankle_roll": (-25.0, 25.0),
    "neck": (-90.0, 90.0),
}

# ----------------------------------------------------------------------------
# foot v4 (mirrored pair: foot_L, foot_R)
# ----------------------------------------------------------------------------
# Sole 130 x 84 with its centreline FOOT_Y_OFF OUTBOARD of the ankle roll
# axis: the open-loop walk's failure direction is outward (the swing foot
# lands, the closed chain springs the pelvis outboard), so the margin goes
# there -- 54 mm outboard, 30 inboard, 24 mm between the feet's inner edges.
# Gate D on the 1.55 kg torso passes only with this bias
# (docs/design-v6/gateD_torso_v7.txt). Foot-local frame: ankle roll axis
# vertical projection at the origin, +X toe, +Y toward the robot's LEFT for
# foot_L (so outboard is +Y on foot_L and -Y on foot_R), z = 0 at the PLATE
# bottom (the TPU sole hangs TPU_SOLE_T below).
FOOT_L_LEN = 130.0
FOOT_TOE = 75.0         # ankle axis -> toe edge
FOOT_HEEL = FOOT_L_LEN - FOOT_TOE                                    # 55
FOOT_W = 84.0           # 76 -> 84 on 2026-09-14: with the CAD part masses (1.65 kg)
                        # the walk's margin was INBOARD-limited at 8 mm; 30 mm of
                        # inboard sole restores >= 12 mm at every pelvis mass tried
FOOT_Y_OFF = 12.0       # sole centreline outboard of the roll axis
FOOT_IN = FOOT_W / 2 - FOOT_Y_OFF                                    # 30 inboard half
FOOT_OUT = FOOT_W / 2 + FOOT_Y_OFF                                   # 54 outboard half
FOOT_CORNER_R = 14.0
FOOT_INNER_GAP = HIP_SEP - 2 * FOOT_IN                               # 24
# roll servo placement: output (horn) end FORWARD (+X), case length along Y
# with the CABLE END OUTBOARD -- the case spans y -10.11..+35.11 (foot_L,
# outboard = +y) and sits on the plate top. Its horn disc face is at
# x = +SV_HORN_FACE, the idler disc face at x = SV_IDLER_FACE (-16.80); the
# ankle link's tines bolt to those two discs from the front and the rear.
FOOT_SV_LEN_IN = D.SV_AXIS_FROM_OUT_END      # 10.11 inboard of the axis
FOOT_SV_LEN_OUT = D.SV_AXIS_FROM_REAR        # 35.11 outboard (cable end)
assert FOOT_SV_LEN_OUT + D.WALL + 2.0 <= FOOT_OUT, "roll servo cable end runs off the sole"
assert FOOT_SV_LEN_IN + D.WALL <= FOOT_IN, "roll servo output end runs off the inboard edge"
# retention: the case's mounting holes live on the horn-side (+x, rows 8.30
# and 29.00 along the length) and idler-side (-x, rows 8.30 and 32.75)
# faces; along the length is +y here (outboard), lateral +-10.25 is +-z about
# the axis. Tabs stand fore and aft of the case at the FAR rows only
# (y +29.00 / +32.75): the near row's upper screw sits inside the tine pads'
# O20 sweep. 2 screws per tab (z = 12.36 +- 10.25 above the plate top), M2.5
# flat-head self-tappers, countersunk on the tab's outer face (the tine sweeps
# D.GRIP_SEAT_CLR + ~0.7 outside it, exactly the v5 foot-tab / shin-fork pair).
FOOT_TAB_ROW_HORN = D.CASE_HOLES_TOP[1]      # 29.00 (front tab)
FOOT_TAB_ROW_IDLER = D.CASE_HOLES_BOT[1]     # 32.75 (rear tab)
FOOT_TAB_HW = 5.5                             # tab half-length along y
FOOT_TAB_T = D.FOOT_WALL_T                    # 2.4, LOCKED like v5 (tine clearance)
FOOT_TAB_H = 26.0                             # above the plate top (covers z 22.61 + head)
FOOT_SCREW_Z = tuple(FOOT_PLATE_T + D.SV_WID / 2 + s * D.CASE_HOLE_LAT for s in (-1, 1))  # (6.11, 26.61) above plate bottom
# front (horn side) tab seats on the case face at +SV_TOPFACE; rear (idler
# side) tab seats on the REAL idler face (SV_IDLER_CASE_FACE), like v5's foot
FOOT_TAB_FRONT_X = (D.SV_TOPFACE + D.GRIP_SEAT_CLR, D.SV_TOPFACE + D.GRIP_SEAT_CLR + FOOT_TAB_T)   # 17.50..19.90
FOOT_TAB_REAR_X = (D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - FOOT_TAB_T, D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR)  # -17.30..-14.90
# an end stop on the INBOARD end of the case (y = -10.11 - FIT) and a low
# cradle rail either side of the case bottom keep it located; the outboard
# (cable) end is open for the lead, which exits the idler-side face ports
# (D.SV_CONN_L along the length -> y +11.75..+16.35, on the -x face) and runs
# aft and up the ankle link's raceway.
FOOT_CRADLE_H = 6.0                           # rails beside the case, above the plate
FOOT_TPU_POCKET = 0.0                         # TPU sole glued to the FLAT underside (v5 lesson)
# the tine pads (O20 on the discs) dip below the plate top by 10 - 16.36 -> no,
# they clear it: pad bottom = 16.36 - 10 = 6.36 above the plate bottom, 2.36
# above the plate top. No relief slots needed (v5 needed them at 16.36-12).
assert ANKLE_ROLL_ABOVE_PLATE - D.PAD_D / 2 > FOOT_PLATE_T + 1.0, "tine pads would cut the sole"

# ----------------------------------------------------------------------------
# ankle link (qty 2): grips the ankle-PITCH servo case (hanging, horn +Y),
# forks in X down to the roll servo's horn (+X) and idler (-X) discs
# ----------------------------------------------------------------------------
# Local frame: pitch axis == Y axis through the origin; roll axis == X axis
# at z = -ANKLE_PITCH_TO_ROLL. The grip channel (web on -x, plates on +-y,
# rib/platform detents, csk M2.5 grip screws, cable window) is leg_link's,
# byte for byte (import the helpers, do not redraw). The fork below is NEW:
# two tines in the X-Z plane, front tine inner face on the horn disc face
# (x = SV_HORN_FACE), rear tine inner face on the idler disc face
# (x = SV_IDLER_FACE) -- a 37.25 grip span, like every joint -- each ending
# in an O20 pad with the O14 BCD and the centre relief, joined to the grip
# channel by a plate across the top and a web up the -y side (outside the
# roll servo's +10.11 inboard end... note the roll servo's LONG side is
# OUTBOARD, +y on the left leg, so the ankle link's web goes INBOARD, -y,
# where the case ends 10.11 from the axis).
AL_DROP = ANKLE_PITCH_TO_ROLL
AL_TINE_T = D.PLATE                           # 3.0
AL_TINE_FRONT_X = (D.SV_HORN_FACE, D.SV_HORN_FACE + AL_TINE_T)      # 20.45..23.45
AL_TINE_REAR_X = (D.SV_IDLER_FACE - AL_TINE_T, D.SV_IDLER_FACE)     # -19.80..-16.80
AL_TINE_W = 24.0                              # tine width along y at the pad
AL_WEB_Y = (-D.SV_AXIS_FROM_OUT_END - D.FIT - D.WALL, -D.SV_AXIS_FROM_OUT_END - D.FIT)  # -13.01..-10.41 inboard web
AL_TOP_PLATE_Z = -36.0 - 0.6                  # under the grip plates' bottom (GRIP_BOT -36)
AL_TOP_PLATE_T = D.WALL
# roll servo case top under the link: -AL_DROP + SV_WID/2 = -41.64; the plate
# bottom at -39.2 leaves 2.44 -- the swept corner (+-25 deg, see above) is
# what really sets it and check_assembly_v6 sweeps it
assert (-AL_TOP_PLATE_Z - AL_TOP_PLATE_T) < AL_DROP - D.SV_WID / 2 - 2.0, "ankle link top plate hits the roll servo"

# ----------------------------------------------------------------------------
# leg link v6 (qty 4): thigh AND shin, LINK_DROP 110, stiff closed section
# ----------------------------------------------------------------------------
# Everything about the grip channel and the fork pads is leg_link's; the two
# changes are the length and the SECTION. v5 is a U (back web + two tines)
# and racks as a parallelogram about the web under lateral load. v6 closes
# the U into a box between the grip zone and the fork: a FRONT plate at
# LL_FRONT_X spanning tine to tine from LL_BOX_TOP down to LL_BOX_BOT, where
# the next servo's case top (r 16.0 about the lower axis: hypot(10.11,
# 12.36)) starts to sweep. Round the four outer vertical edges (LL_EDGE_R)
# and taper the tines below the box; print STANDING (lower fork down) so the
# box prints as four walls with no bridge -- the pad bores are horizontal
# and teardropped, the cross plates are 45 deg gables (check_printability
# is the arbiter, as always).
LL_DROP = LINK_DROP
LL_FRONT_X = (D.SV_WID / 2 + D.FIT + 0.8, D.SV_WID / 2 + D.FIT + 0.8 + D.WALL)   # 13.46..16.06
LL_BOX_TOP = -36.5                            # below the servo case bottom (-35.11) + jog
LL_BOX_BOT = -(LL_DROP - 16.0 - D.SWEEP_BUFFER - 1.0)                 # -92.5
LL_EDGE_R = 5.0
LL_WEB_END = -(LL_DROP - 32.0)                # v5's WEB_END rule: 32 above the lower axis
LL_CABLE_WINDOW_Z = (-48.0, -37.0)            # v5 (upper-anchored: the lead leaves the case bottom)
LL_BRACE_Z = (LL_BOX_BOT - 4.0, LL_BOX_BOT)   # gable brace at the box bottom
# DEEP-FLEXION relief (2026-09-14, Tom: "we could probably bend the existing
# knees further ... with slight modifications of the leg links"): the knee is
# clear to 105 deg as drawn; from 110 to 130 the only contact is the SHIN's
# rear-top corner in the idler tine band (web top + the idler grip plate's
# outer skin, 16-33 mm below the knee axis) sweeping into the THIGH's idler
# tine 8-32 mm above the axis. A triangular wedge off that corner (height
# below WEB_TOP, run forward from the web outer face; the idler band only)
# clears it; ~132 deg is the hard limit where the shin's web and the knee
# servo case meet the thigh's web end full-width. Same part serves the thigh,
# where the identical corner faces the pitch yoke at hip extension (+90,
# already 0.7 mm clear -- removing material only helps).
LL_FLEX_CUT = (20.0, 24.0)                    # (height below WEB_TOP, run from the web face), mm
# hip flexion: the thigh's grip plates' front edge sweeps the pitch yoke's
# flange front-bottom corner (x 16, z 26 above the axis) at 123 deg; the
# plate cannot be relieved (the lower grip screw's countersink sits exactly
# there), so the FLANGE corner gets a 45 deg chamfer instead (yoke_pitch_v6).
# 4 mm leaves the M3 heat-set bores (x +-10, r 2.05) untouched and moves the
# limit to ~127 deg; a bigger one runs into the bores, so the hip ROM is 125.
YOKE_FLEX_CHAMFER = 4.0
# The fold limit after LL_FLEX_CUT is 131 deg: the two links' REAR faces
# meet -- the shin's jog-block rear-top corner (x web_x0, z -33, r 36.6
# about the knee) lands on the thigh's rear face 33 mm above the knee (the
# web-end corner in the idler band; the horn slab's face in the horn band).
# Two 45 deg corner chamfers buy the last 2 deg + buffer: the web end's
# outer-bottom corner (idler band) and the jog block's rear-top corner (horn
# band only, y > the web's upper limit so the web is not notched).
LL_FOLD_CHAMFER = 3.0
# anchor classification for anyone porting v5's constants: UPPER-anchored
# (unchanged): GRIP_*, jog -36.5..-33, cable window, rib/platform detents.
# LOWER-anchored (shift by LL_DROP - 90 = +20 down): -92 -> -112, -70 -> -90,
# -54 -> -74, FORK_WIDE_Z -50 -> -70, WEB_END -58 -> -78, BRACE_Z -> box bottom.

# ----------------------------------------------------------------------------
# torso v7: pelvis (one print) + deck + neck + head
# ----------------------------------------------------------------------------
# Pelvis-local frame as v5: deck TOP at z = 0, prints deck-top-down. The
# housing keeps v5's yaw cells (plan geometry unchanged, at y = +-HIP_SEP/2)
# and grows UP by a BATTERY LAYER between the cell tops and the deck: the
# pack lies TRANSVERSE (across y) above the cells, low and central. The two
# boards stand vertical on the housing walls: General Driver on the FRONT
# wall (65 x 65, bus edge up, its v5 recess pattern), Pi 4B on the AFT wall
# (85 across, 56 tall, USB/Ethernet edge to +y through a side window,
# USB-C / HDMI / camera edge UP under the deck). The neck STS3215 sits ON the
# deck, horn up; the head is a carrier on its horn.
BATT_LAYER = 31.0                             # cell tops -> deck bottom
YAW_BOX_DEPTH = D.YAW_BOX_DEPTH               # 36, cell wall below the cell top
DECK_T = D.DECK_T                             # 5
CELL_TOP_Z = -DECK_T - BATT_LAYER             # -36.0 (v5: -5.0)
YAW_BOX_BOT_Z = CELL_TOP_Z - YAW_BOX_DEPTH    # -72.0 (v5: -41.0)
YAW_HORN_FACE_Z = YAW_BOX_BOT_Z - D.YAW_BOX_CARRIER_GAP               # -73.80
TORSO_FLOOR_Z = YAW_HORN_FACE_Z + 1.0         # nothing torso-side below this
DECK_BOT_Z = HIP_YAW_Z - YAW_HORN_FACE_Z - DECK_T                     # world z of the deck bottom
DECK_TOP_Z = DECK_BOT_Z + DECK_T

# ----------------------------------------------------------------------------
# hip-yaw bearing (2026-09-17, docs/design-v6/study-yaw-bearing.md; REVISED
# after coordinator review found the first two passes wrong -- see the
# write-up's history section). Today the yaw servo's HORN SPLINE + 4 horn
# screws are the only connection between the carrier (whole leg, 0.335 m
# lever) and the pelvis -- thrust (~15 N single-support) and the roll
# moment (~0.65 N-m) both go through the shaft.
#
# SIZE FROM THE GEOMETRY, NOT THE OTHER WAY ROUND. The carrier's horn-mount
# PLATE is bigger than the servo's O19.2 horn disc -- it is ALSO the roll
# bay's ceiling (X +/-19.95, Y +/-hw=15.26), and the bay WALLS below it
# share that SAME footprint for their whole height (front/rear/cheeks run
# the full X/Y rectangle from the plate down to the bore). So the carrier's
# max corner reach, hypot(19.95, 15.26) = 25.12 mm, is present at EVERY z
# the bearing band could occupy, not just at the plate's own 3 mm. Two
# earlier passes got this backwards -- picked a bearing first, then tried
# to shrink the carrier (a corner trim) or grow the pelvis recess (a
# clearance fit that defeated the point of a bearing) to fit around it.
# Correct order: the boss (inner-race seat) must ENCLOSE the carrier's
# whole footprint with >= 1.5 mm of wall -> bore radius >= 25.12 + 1.5 =
# 26.62 mm -> bore >= 53.24 mm. The smallest common thin/reduced-section
# size that clears this is 55 mm bore. 6711-2RS (55x68x7) has no published
# C0/C in the sources checked for this study (multiple supplier listings,
# no load-rating datasheet found) -- rather than guess a number,
# **6811-2RS (55x72x9)** is used instead: C0 published consistently
# (6.2-8.4 kN across sources; 8.1 kN used below) even though it costs 2 mm
# more width and a tighter skirt-to-skirt gap (see YAW_BRG_SKIRT_OD).
#
# The boss WRAPS the existing plate/wall footprint -- nothing is trimmed
# off the carrier. It lives entirely BELOW the existing yaw cell tube rim
# (YAW_BOX_BOT_Z, unchanged) -- the GD/Pi board columns and the servo case
# both end at or above that Z, so the whole bearing band sits in air that
# was already open. NO axis drop, no change to
# YAW_HORN_FACE_Z/TORSO_FLOOR_Z/HIP_YAW_Z/anything in the leg chain.
YAW_BRG_ID = 55.0                              # 6811-2RS bore
YAW_BRG_OD = 72.0                              # 6811-2RS OD
YAW_BRG_W = 9.0                                # 6811-2RS width
YAW_BRG_C0_KN = 8.1                            # published range 6.2-8.4 kN
                                                # across sources checked; not a
                                                # single vendor's guarantee
_carrier_corner_r = math.hypot(19.95, D.SV_WID / 2 + D.BAY_CHEEK_GAP + D.WALL)  # 25.12
YAW_BRG_BOSS_OD = YAW_BRG_ID + 0.08             # 55.08, +0.08 interference
                                                 # (within the requested
                                                 # +0.05..+0.10 band; located,
                                                 # not a clearance/adhesive fit)
YAW_BRG_BOSS_R = YAW_BRG_BOSS_OD / 2            # 27.54
YAW_BRG_BOSS_H = YAW_BRG_W                      # 9.0, full bore engagement
YAW_BRG_RECESS_ID = YAW_BRG_OD - 0.04           # 71.96, -0.04 interference
                                                 # (within the requested
                                                 # -0.03..-0.05 band)
YAW_BRG_RECESS_R = YAW_BRG_RECESS_ID / 2        # 35.98
# ESTIMATED internal race split -- NOT a published spec (bearing internal
# raceway/ball geometry is not in the basic catalog data found for either
# 6709 or 6811); scaled from the coordinator's own illustrative 6709 split
# (inner ring 2 mm, ball/cage gap 1 mm, outer ring 2 mm, over a 5 mm total
# radial span) by this bearing's larger span, (72-55)/2 = 8.5 mm, factor
# 1.7x. Used ONLY to keep the pelvis shoulder off the moving inner
# ring/balls -- verify against the real bearing before cutting metal.
YAW_BRG_EST_INNER_RING_OD = YAW_BRG_BOSS_R + 2.0 * 1.7      # ~30.94
YAW_BRG_EST_OUTER_RING_ID = YAW_BRG_EST_INNER_RING_OD + 1.0 * 1.7  # ~32.64 (est. gap)
YAW_BRG_SHOULDER_LAND = 1.5                     # ledge width (radial) that stops
                                                 # the outer race's top face --
                                                 # kept narrow (vs the estimated
                                                 # outer-ring ID above) so it
                                                 # cannot touch the moving parts
                                                 # even if the estimate is off
YAW_BRG_SHOULDER_ID = YAW_BRG_OD - 2 * YAW_BRG_SHOULDER_LAND    # 69.0 (r 34.5,
                                                 # 1.86 mm clear of the est.
                                                 # outer-ring ID above)
assert YAW_BRG_SHOULDER_ID / 2 > YAW_BRG_EST_OUTER_RING_ID, \
    "shoulder must not reach the (estimated) outer race's own ID"
YAW_BRG_SKIRT_OD = 78.0                        # ~3.0 mm wall around the recess.
                                                # Two skirts, 84 mm hip sep:
                                                # 84 - 78 = 6 mm edge-to-edge --
                                                # too tight for two independent
                                                # free-hanging rings, so they
                                                # are BRIDGED (pelvis_v7.py) by
                                                # a short connecting web at the
                                                # bottom rather than left as two
                                                # separate thin hoops.
# skirt/recess Z (pelvis-local, hangs from the EXISTING rim, nothing above
# YAW_BOX_BOT_Z moves). The recess is pinned to YAW_HORN_FACE_Z (the world
# reference the carrier's boss ALSO uses -- HIP_YAW_Z in assembly_v6.py --
# so the two align exactly): recess proper is the bearing's own width,
# ending flush at the horn face; the shoulder fills the remaining, already-
# existing 1.8 mm gap up to the tube rim (D.YAW_BOX_CARRIER_GAP, unchanged).
YAW_BRG_RECESS_Z = (YAW_HORN_FACE_Z - YAW_BRG_W, YAW_HORN_FACE_Z)              # -82.80..-73.80
YAW_BRG_SHOULDER_Z = (YAW_BRG_RECESS_Z[1], YAW_BOX_BOT_Z)                      # -73.80..-72.0
YAW_BRG_SHOULDER_H = YAW_BRG_SHOULDER_Z[1] - YAW_BRG_SHOULDER_Z[0]             # 1.80 == D.YAW_BOX_CARRIER_GAP
# carrier boss Z, in the CARRIER's own local frame (z=0 at the horn face,
# +Z toward the servo -- see cad/parts.py yaw_carrier docstring): the boss
# spans the plate's own 3 mm (z=-3..0) PLUS the top of the bay walls below
# it (z=-9..-3), which share the SAME footprint/corner reach as the plate,
# so wrapping them costs nothing extra in radius.
YAW_BRG_BOSS_CARRIER_Z = (-YAW_BRG_BOSS_H, 0.0)                                # -9.0..0.0
assert YAW_BRG_BOSS_R - _carrier_corner_r >= 1.5, \
    "boss must enclose the carrier's own corner (plate AND bay walls) with >= 1.5 mm wall"
_case_corner_r = math.hypot(D.SV_AXIS_FROM_OUT_END, D.SV_WID / 2)   # 15.97
assert YAW_BRG_BOSS_R > _case_corner_r, "boss radius must clear the servo case's own corner"
assert YAW_BRG_SHOULDER_Z[1] == YAW_BOX_BOT_Z, "skirt must hang from the EXISTING rim, unmoved"

# ----------------------------------------------------------------------------
# hip-yaw bearing OPTION STUDY, round 2 (2026-09-18, docs/design-v6/
# study-yaw-bearing.md; Tom: "the hip stack must NOT get taller" -- drop any
# axial growth -- AND "modify the yaw joints to be round"). Everything above
# (YAW_BRG_*) is OPTION C, the 2026-09-17 baseline (6811-2RS wrapping the
# carrier's WHOLE rectangular plate+wall footprint) -- left unrenamed because
# it is still what yaw_carrier_v6()/pelvis_v7() build by default. Below:
# OPTION A (a round hub, AT THE SAME AXIAL ENVELOPE -- no HIP_YAW_Z /
# CARRIER_ROLL_AXIS / d_yaw_roll change) and OPTION B's measurement (raise
# the yaw servo into the housing -- stopped at measurement, see the write-up).

# ---- OPTION A: round hub, current height ------------------------------------
# The bearing band is FIXED, not new axial length: it starts at the horn
# face (carrier z=0) and runs down the bearing's own width. Within that band
# the carrier already holds (v5, unchanged): the horn mount plate (z 0..-3,
# corner 25.12 mm, _carrier_corner_r above) and, once the band passes -3 mm,
# the TOP of the roll-servo bay -- occupied by the ROLL servo's own case,
# whose cable (top) end is at CARRIER_ROLL_AXIS + SV_AXIS_FROM_REAR =
# -40.11 + 35.11 = -5.00 mm -- a different, BULKIER footprint than the small
# yaw-horn disc the round-hub idea first (wrongly) sized to. Measured
# directly (`CA.servo_mock_x()` intersected with a box at carrier-local z in
# [-10.11, -5.00], i.e. the roll servo's own cable-end 5 mm, in a REPL this
# session): the case's own worst corner reach from the yaw axis over that
# span, including the horn-side rib (SV_HORN_RIB_H proud of SV_TOPFACE):
YAWA_BAND_CASE_CORNER_R = 22.23   # mm, measured (see comment above); bigger
                                  # than the yaw-servo/horn-disc figure
                                  # (_case_corner_r above, 15.97) and only a
                                  # little under the carrier's OWN existing
                                  # rectangular corner (25.12)
# Smallest of the three offered bore classes (45: 6709/6809, 50: 6710/6810,
# 55: 6711/6811) that clears YAWA_BAND_CASE_CORNER_R + 1.5 mm wall (need
# radius >= 23.73 mm, bore >= 47.46 mm): 45 mm FAILS (r 22.54, wall only
# 0.31 mm -- not printable, kept as an assert below so it is not re-tried);
# 50 mm clears with 2.81 mm of wall; 55 mm also clears but is not smallest.
YAWA_BRG_ID = 50.0            # 6810-2RS bore
YAWA_BRG_OD = 65.0            # 6810-2RS OD (vs 6710-2RS's 62 mm -- 6810 picked
                              # for the more consistently published static
                              # rating; see the BOM note)
YAWA_BRG_W = 7.0              # 6810-2RS width (6710-2RS is 6 mm/67 g, lighter
                              # bearing but a heavier PART, and its C0 varies
                              # 2.6-3.1 kN across sources checked; either is
                              # enormous overkill at these loads -- see YAW_BRG
                              # section 3.1's arithmetic, same order of load)
YAWA_BRG_C0_KN = 5.8          # 6810-2RS, one full spec sheet (50x65x7); a
                              # second source's DYNAMIC figure (6.6 kN) is not
                              # directly comparable and was not cross-checked
                              # against a second STATIC source this session
YAWA_BRG_MASS_G = 52.0        # cited, single source, not independently re-verified
YAWA_BRG_BOSS_OD = YAWA_BRG_ID + 0.08         # 50.08, +0.08 interference (same band as YAW_BRG_BOSS_OD)
YAWA_BRG_BOSS_R = YAWA_BRG_BOSS_OD / 2        # 25.04
YAWA_BRG_BOSS_H = YAWA_BRG_W                  # 7.0 mm, full bore engagement -- the band IS carrier z [-7, 0]
assert YAWA_BRG_BOSS_R - YAWA_BAND_CASE_CORNER_R >= 1.5, \
    "hub must enclose the roll servo's own case corner (within the band) with >= 1.5 mm wall"
_yawA_45_r = (45.0 + 0.08) / 2
assert _yawA_45_r - YAWA_BAND_CASE_CORNER_R < 1.5, \
    "45 mm bore is supposed to FAIL this margin (negative control) -- re-check the 50 mm pick if this trips"
YAWA_BRG_RECESS_ID = YAWA_BRG_OD - 0.04       # 64.96, -0.04 interference
YAWA_BRG_RECESS_R = YAWA_BRG_RECESS_ID / 2    # 32.48
_yawA_span = (YAWA_BRG_OD - YAWA_BRG_ID) / 2       # 7.5 mm, this bearing's race span
_yawA_factor = _yawA_span / 5.0                     # scale of the coordinator's illustrative 6709 split (5 mm span)
YAWA_EST_INNER_RING_OD = YAWA_BRG_BOSS_R + 2.0 * _yawA_factor       # ~28.04, ESTIMATE (no published internal geometry)
YAWA_EST_OUTER_RING_ID = YAWA_EST_INNER_RING_OD + 1.0 * _yawA_factor  # ~29.54, ESTIMATE
YAWA_SHOULDER_LAND = 1.2
YAWA_SHOULDER_ID = YAWA_BRG_OD - 2 * YAWA_SHOULDER_LAND     # 62.6 (r 31.3)
assert YAWA_SHOULDER_ID / 2 > YAWA_EST_OUTER_RING_ID, \
    "shoulder must not reach the (estimated) outer race's own ID"
YAWA_SKIRT_OD = YAWA_BRG_OD + 6.0   # 71.0, ~3 mm wall around the recess (same convention as YAW_BRG_SKIRT_OD)
# skirt gap: HIP_SEP - YAWA_SKIRT_OD = 84 - 71 = 13 mm edge-to-edge (vs 6 mm
# for option C's 78 mm skirt). A bridge is still added (pelvis_v7.py,
# bearing_variant="A") for the same reason as option C's -- kept as a
# conservative, NOT FEA-verified choice rather than assumed unnecessary just
# because the gap more than doubled.
YAWA_BAND_Z = (-YAWA_BRG_BOSS_H, 0.0)     # carrier-local -- SAME frame/origin
                                           # as today; no change anywhere to
                                           # HIP_YAW_Z, CARRIER_ROLL_AXIS or
                                           # d_yaw_roll (Tom's constraint 1).
YAWA_ROLL_CASE_TOP_Z = D.CARRIER_ROLL_AXIS + D.SV_AXIS_FROM_REAR   # -5.00, carrier-local
# roll-servo retention screw rows (carrier-local; both must stay OUTSIDE the
# band -- the SCREW HOLES themselves are v5-unchanged, only the wall/hub
# OUTSIDE them changes shape)
YAWA_ROLL_SCREW_ROWS_Z = (D.CARRIER_ROLL_AXIS + D.CASE_HOLES_TOP[1],
                          D.CARRIER_ROLL_AXIS + D.CASE_HOLES_BOT[1])   # -11.11, -7.36
assert YAWA_ROLL_SCREW_ROWS_Z[1] < YAWA_BAND_Z[0], \
    "the idler-side retention screw row must stay below (outside) the band"
YAWA_RECESS_Z = (YAW_HORN_FACE_Z - YAWA_BRG_W, YAW_HORN_FACE_Z)      # -80.80..-73.80, pelvis-local
YAWA_SHOULDER_Z = (YAWA_RECESS_Z[1], YAW_BOX_BOT_Z)                   # -73.80..-72.0 -- the SAME existing 1.8 mm gap option C reuses
assert YAWA_SHOULDER_Z[1] == YAW_BOX_BOT_Z, \
    "skirt must hang from the EXISTING rim, unmoved (no axial growth)"

# housing plan
# +4: an outer skin outboard of the yaw cell walls, so the housing can taper
# 3 deg toward the bottom for the look without thinning the cell walls
# (torso pass 2, 2026-09-14)
HOUSING_SKIN = 4.0
HOUSING_HW = HIP_SEP / 2 + D.YAW_BOX_HW_OUT + HOUSING_SKIN   # 61.26 outer half width
HOUSING_CHAN_HW = HIP_SEP / 2 - D.YAW_BOX_HW_OUT                      # 26.74 centre channel half width
CELL_X = (D.YAW_BOX_X_REAR, D.YAW_BOX_X_FRONT)                        # -38.01..+13.01 (v5)
# General Driver on the front wall (v5's BR_* recess rules, mirrored to +x)
GD_STANDOFF = D.BR_STANDOFF                   # 5
GD_PCB_X0 = CELL_X[1] + GD_STANDOFF           # +18.01 PCB aft face
GD_PCB_X1 = GD_PCB_X0 + D.BR_PCB_T            # +19.64
GD_COMP_X = GD_PCB_X1 + D.BOARD_GD_COMP       # +28.64 tallest component
GD_FRONT_X = GD_COMP_X + D.BR_RIM_PROUD       # +30.44 housing front (inner rim)
GD_BOT_Z = TORSO_FLOOR_Z + 0.8                # -72.0
GD_TOP_Z = GD_BOT_Z + D.BOARD_GD_OUTLINE[1]   # -15.99 -- BELOW the deck now (v5: +15):
GD_SCREW_ROWS_Z = (GD_BOT_Z + D.BOARD_GD_HOLE_INSET,
                   GD_BOT_Z + D.BOARD_GD_HOLE_INSET + D.BOARD_GD_HOLES[1])
assert GD_TOP_Z < -DECK_T, "General Driver must be fully under the deck in the v7 housing"
# ...so all four screws land on standoffs and v5's rail grooves are gone.
# Its bus/service edges: bus edge UP (leads drop from the deck underside),
# service edge (XH inlet, USB-C) to +y through a side window (v5 BR_SVC_*).
# Pi 4B on the aft wall
PI_OUTLINE = (85.0, 56.0)                     # across (y), tall (z)
PI_HOLES = (58.0, 49.0)                       # across, tall
PI_HOLE_INSET = 3.5                           # from the edges (both axes)
PI_HOLE_D = 2.7                               # M2.5 clearance
PI_PCB_T = 1.5
PI_COMP_TOP = 16.0                            # USB/Ethernet stack above the PCB top (no heatsink)
PI_COMP_BOT = 2.5                             # solder side
PI_STANDOFF = 6.0                             # wall face .. PCB (solder side toward the wall)
PI_PCB_X1 = CELL_X[0] - PI_STANDOFF           # -44.01 PCB (wall-side) face
PI_PCB_X0 = PI_PCB_X1 - PI_PCB_T              # -45.51
PI_COMP_X = PI_PCB_X0 - PI_COMP_TOP           # -61.51 deepest component (USB stack)
PI_AFT_X = PI_COMP_X - D.BR_RIM_PROUD         # -63.31 housing aft (inner rim)
PI_BOT_Z = TORSO_FLOOR_Z + 1.5                # -71.3 board bottom edge
PI_TOP_Z = PI_BOT_Z + PI_OUTLINE[1]           # -15.3 board top edge (USB-C / HDMI / CSI edge)
PI_CZ = PI_BOT_Z + PI_OUTLINE[1] / 2
PI_SCREW_Z = (PI_BOT_Z + PI_HOLE_INSET, PI_BOT_Z + PI_HOLE_INSET + PI_HOLES[1])
PI_SCREW_Y = (-PI_HOLES[0] / 2, PI_HOLES[0] / 2)
PI_PORT_EDGE_SIGN = +1                        # USB/Ethernet stack faces +y (side window)
assert PI_TOP_Z < -DECK_T, "Pi 4B must be fully under the deck"
assert PI_OUTLINE[0] / 2 + 2.0 < HOUSING_HW - D.WALL, "Pi 4B does not fit across the housing"
HOUSING_X = (PI_AFT_X - D.WALL, GD_FRONT_X + D.WALL)                  # -65.91..+33.04 outer
HOUSING_LEN = HOUSING_X[1] - HOUSING_X[0]                             # 98.95
# battery: 3S 2200-2600 mAh LiPo CLASS, bought to a spec filter (11.1 V, XT60
# or XT30, <= 26 mm tall, <= 36 wide, <= 105 long, 150-190 g). Transverse,
# centred, on a floor above the cells (the cell tops ARE the floor), belted.
BATT = (105.0, 36.0, 26.0)                    # length (y), width (x), height (z)
BATT_MASS = 170.0
BATT_X = (CELL_X[0] + 2.0, CELL_X[0] + 2.0 + BATT[1])                # -36.01..-0.01 (over the cells)
BATT_Z = (CELL_TOP_Z + 0.5, CELL_TOP_Z + 0.5 + BATT[2])              # -35.5..-9.5
BATT_HY = BATT[0] / 2                         # 52.5 half length
assert BATT_Z[1] <= -DECK_T - 1.0, "pack does not fit under the deck"
assert BATT_HY + 1.0 <= HOUSING_HW - D.WALL, "pack is longer than the housing is wide"
BATT_BELT_W = D.BT_BELT_W                     # 15

# ---- OPTION B measurement (hip-yaw bearing study round 2, 2026-09-18):
# raise the yaw servo into the housing. "7-10 mm of rise" (the brief) has to
# open up between the yaw servo's cable (top) end and the cell ceiling --
# i.e. the WHOLE servo (case+horn+disc) moves up, so the cell's own ceiling
# (CELL_TOP_Z, the battery bay's FLOOR) has to rise the same amount (the
# case is retained from the ceiling down, the same pattern the roll servo's
# own cable end shows one joint lower -- YAWA_ROLL_CASE_TOP_Z above).
# Measured clearance around the battery layer, which sits directly above:
YAWB_CLEAR_BELOW_BATT = BATT_Z[0] - CELL_TOP_Z    # -35.5 - (-36.0) = 0.5 mm (cell ceiling -> battery bottom)
YAWB_CLEAR_ABOVE_BATT = -DECK_T - BATT_Z[1]       # -5.0 - (-9.5) = 4.5 mm (battery top -> deck bottom)
YAWB_NEEDED_RISE_LO, YAWB_NEEDED_RISE_HI = 7.0, 10.0
YAWB_SHORTFALL_LO = YAWB_NEEDED_RISE_LO - YAWB_CLEAR_ABOVE_BATT   # 2.5 mm
YAWB_SHORTFALL_HI = YAWB_NEEDED_RISE_HI - YAWB_CLEAR_ABOVE_BATT   # 5.5 mm
assert YAWB_CLEAR_ABOVE_BATT < YAWB_NEEDED_RISE_LO, \
    "Option B: not enough rise exists above the battery without moving it (or the deck) -- see the study doc"
# power electronics beside the pack, forward of it, in the same layer:
# 3S protection board + 5 V / 5 A buck (Pololu D24V50F5, 17.8 x 25.4 x ~8)
# The pocket's forward limit is the yaw cell block's own front web (inner face
# x = CELL_X[1] - YAW_SEAT_WALL = +10.41), which must stay solid; the boards
# stand ON EDGE in an 8.4 mm deep slot (the buck is 8 mm tall with its parts)
PWR_BOARD = (60.0, 8.4, 25.0)                 # y, x, z envelope for both boards
PWR_X = (BATT_X[1] + 1.5, BATT_X[1] + 1.5 + PWR_BOARD[1])            # +1.5..+9.9
PWR_Z = BATT_Z
assert PWR_X[1] <= CELL_X[1] - D.YAW_SEAT_WALL - 0.5, "power pocket runs into the cell block's front web"
assert PWR_X[1] < GD_PCB_X0 - 2.0, "power boards collide with the General Driver standoffs"
# deck
DECK_X = HOUSING_X                            # the deck is the housing's lid, full footprint
DECK_HW = HOUSING_HW
# neck: STS3215 standing ON the deck, output axis vertical, horn UP; the case
# is retained the way the yaw cells hold theirs, inverted: a shallow well in
# the deck top locates the case (YAW_SEAT_GAP fit) and 4x M2.5 through the
# well floor into the case's IDLER-side face holes (rows 8.30 / 32.75 behind
# the axis, +-10.25). The axis sits NECK_X aft of centre so the case (10.11
# fwd / 35.11 aft of the axis) is centred on the deck.
NECK_X = 0.0
NECK_AXIS_ABOVE_DECK = D.SV_IDLER_CASE_FACE * -1.0 + 0.0             # case idler face ON the deck -> axis 14.75 up... see below
# the case stands on its idler-side face: that face is at -14.75 from the axis
# along the axis, so the axis is 14.75 above the deck top, the horn face at
# +20.45 -> 35.20 above the deck, the horn disc top ~ +21.3.
NECK_AXIS_Z = D.SV_TOPFACE - D.SV_TOPFACE + 14.75                    # 14.75 above the deck top
NECK_HORN_FACE_Z = NECK_AXIS_Z + D.SV_HORN_FACE                      # 35.20 above the deck top
NECK_WELL_D = 3.0                             # locating well depth in the deck top
NECK_WELL_HW = (D.SV_WID / 2 + D.YAW_SEAT_GAP, )                     # +-12.66 across
NECK_WELL_X = (-D.SV_AXIS_FROM_REAR - D.YAW_SEAT_GAP, D.SV_AXIS_FROM_OUT_END + D.YAW_SEAT_GAP)   # -35.41..+10.41
# head: a carrier bolted to the neck horn (4x M3x6 on the O14 BCD + the
# centre screw recess), carrying the Camera Module 3 (Wide) on its front face
HEAD_BASE_T = 4.0                             # horn plate thickness
HEAD_W, HEAD_D, HEAD_H = 62.0, 50.0, 56.0     # outer shell, rounded (organic)
HEAD_WALL = 2.0
CAM3_BOARD = (25.0, 24.0)                     # Camera Module 3 PCB (w x h)
CAM3_HOLES = (21.0, 12.5)                     # hole spacing (w x h), M2
CAM3_LENS_D = 10.0
CAM3_LENS_ABOVE_HOLES_CZ = 0.0                # lens centred between the holes (Wide: +0)
CAM_Z_ABOVE_HORN = 30.0                       # lens centre above the horn face
HEAD_CABLE_SLOT = (12.0, 4.0)                 # camera ribbon slot in the base (w x t)
NECK_ROM = ROM["neck"]
# world heights
NECK_HORN_Z = DECK_TOP_Z + NECK_HORN_FACE_Z
CAM_Z = NECK_HORN_Z + HEAD_BASE_T + CAM_Z_ABOVE_HORN
TOP_Z = NECK_HORN_Z + HEAD_BASE_T + HEAD_H

# ----------------------------------------------------------------------------
# masses for the rollup (g) -- estimates until the parts print and are weighed
# ----------------------------------------------------------------------------
PI4_MASS = 46.0
GD_MASS = 30.0
PWR_MASS = 60.0
CAM3_MASS = 4.0
WIRING_MASS = 40.0

# ============================================================================
# ARMS -- two 2-DOF arms (shoulder pitch + elbow), the get-up design decision
# of 2026-09-17 (docs/design-v6/getup-decision-2026-09-17.md section 1,
# docs/design-v6/study-shoulder-arms.md sections 4-7). NOTHING here is
# re-derived: the segment lengths, the DOF count, the aft mount and the
# hanging idle pose are the study's measured numbers. Everything below that
# is a CAD consequence of hanging a real STS3215 on a real deck, and every
# place the CAD had to DEVIATE from the plant is called out by name so the
# sim can be re-run against the as-drawn geometry.
#
# Plant config this mirrors: sim/getup_v6_shoulder.py CONFIGS
# ["top_elbow_16_16_aft"] = arms=True, arm_len=0.16, arm_elbow=True,
# arm_fore_len=0.16, arm_shoulder_x=-0.05, arm_z=0.079.
# ============================================================================
ARM_UPPER = 160.0        # shoulder axis -> elbow axis (plant arm_len 0.16).
ARM_FORE = 160.0         # elbow axis -> hand centre (plant arm_fore_len 0.16).
                         # 0.32 m end-to-end is the MEASURED reach threshold:
                         # 0.30 (15+15) never stands, 0.32 does. Do not shorten.
ARM_SHOULDER_X = -50.0   # aft mount (plant arm_shoulder_x = -0.05). This is
                         # the fix for the hanging arm walking into its own
                         # leg: 7327 arm-vs-leg contacts at the default -0.02
                         # mount, 31 here (study section 5). Do not move it forward.

# --- the shoulder servo's pose on the deck, and what forced it -------------
# The servo stands on the deck with its case LENGTH VERTICAL, output end DOWN,
# cable end up, output axis along Y and the HORN OUTBOARD. Three constraints
# pick that pose and there is no fourth option:
#   (a) the output axis must be lateral (Y) for a sagittal-plane shoulder, so
#       the case's 34.70 SV_CASE_T always runs along Y. The case therefore
#       always reaches to ARM_Y - 5.6, whatever else is done.
#   (b) laid flat (case length along X) the cradle would sit over the battery
#       aperture (x -36.51..+10.41, |y| < 54.66) -- the deck is OPEN there, so
#       the forward case-screw row would have nothing to bolt into. Standing it
#       up puts the whole 24.72 mm footprint aft of the aperture.
#   (c) the Pi 4B can only be installed by sliding DOWN through its deck slot
#       (x -51.76..-37.76, |y| <= 45, pelvis_v7's PI_SLOT). Nothing of the arm
#       mount may overhang that slot or the Pi becomes uninstallable, which is
#       what sets ARM_Y (below) rather than the plant's 80.35.
ARM_CRADLE_FLOOR = 4.0   # bracket floor under the servo's output end. 4.0, not
                         # WALL: this plate cantilevers ~22 mm past the deck edge.
ARM_SHOULDER_Z = DECK_TOP_Z + ARM_CRADLE_FLOOR + D.SV_AXIS_FROM_OUT_END   # 474.38
ARM_ELBOW_Z = ARM_SHOULDER_Z - ARM_UPPER                                  # 314.38
ARM_HAND_Z = ARM_ELBOW_Z - ARM_FORE                                       # 154.38
# DEVIATION 1 (shoulder height): the plant put the shoulder 79.0 mm above the
# yaw axis ("the deck top"). The CAD deck top is only 73.80 above the yaw axis
# (the CAD housing is 5 mm shallower than the plant's), and a real servo case
# cannot have its axis IN the deck surface -- the axis sits 4.0 (floor) +
# 10.11 (SV_AXIS_FROM_OUT_END) above it. Net: 87.91, i.e. +8.9 mm.
ARM_SHOULDER_ABOVE_YAW = ARM_SHOULDER_Z - HIP_YAW_Z                       # 87.91
ARM_SHOULDER_Z_SIM = 79.0    # what the get-up was measured at (m*1e3, above yaw)

# --- the arm plane --------------------------------------------------------
# |y| of the upper arm's horn plate == the elbow servo's mid-plane. The arm is
# straight: one plane from the shoulder horn to the hand.
# DEVIATION 2 (lateral): the plant hung the arm at 80.35 mm (deck_w/2 + SV_T/2
# + 4 on its 118 mm-wide deck). 88.0 here, +7.7 mm outboard, forced by (c)
# above: the shoulder servo's inboard case-screw wall is ARM_Y - 40.85, and it
# must clear the Pi slot's y = 45 edge. It is also the friendlier direction --
# the study's own widen sweep (arm_shoulder_y_extra +20 mm) cut arm-vs-leg
# contacts, and in CAD it is what buys the elbow fork its clearance past the
# thigh/yoke (whose outer face reaches y 65.5).
ARM_Y = 88.0
ARM_Y_SIM = 80.35
ARM_HORN_FACE_Y = ARM_Y - D.PLATE / 2 - D.HORN_BOSS_H      # 85.50 shoulder horn face
ARM_SERVO_MID_Y = ARM_HORN_FACE_Y - D.SV_HORN_FACE         # 65.05 shoulder case mid-plane
ARM_SEAT_Y = ARM_SERVO_MID_Y + D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR      # 50.15 wall seat
ARM_WALL_Y0 = ARM_SEAT_Y - D.GRIP_PLATE_T_IDLER            # 47.15 wall inner face
ARM_CASE_Y1 = ARM_SERVO_MID_Y + D.SV_TOPFACE               # 82.40 case outboard face
ARM_MOUNT_Y1 = ARM_CASE_Y1 + 0.6                           # 83.00 bracket outboard limit
assert ARM_WALL_Y0 > PI_OUTLINE[0] / 2 + 2.5 + 1.0, \
    "shoulder bracket must clear the Pi slide slot (pelvis_v7 PI_SLOT)"

# --- the arm links --------------------------------------------------------
# Section: an open C -- back web + two side rails, opening FORWARD (+x). Not a
# closed box like leg_link_v6: this part prints on its back (model +X up, v5
# leg_link's proven "the filament runs the length of the leg" orientation, the
# control case in cad/parts.py's yoke_roll comment), and in that orientation a
# front plate is a flat ceiling spanning the full tine-to-tine width. The load
# that matters here is bending in the sagittal plane, which the C carries in
# its rails; peak measured joint torque is 1.59 N-m (shoulder) / 1.18 (elbow).
ARM_WEB_X = (-15.16, -12.76)     # back web: bed face .. inner face (WEB_GAP off
                                 # the gripped case's -12.36, same as leg_link)
ARM_FRONT_X = 12.0               # rails' front edge at a joint (leg_link's
                                 # fork-arm bound; the pads need x -10..+10)
ARM_TIP_X = 5.0                  # rails' front edge at the far end of a shaft.
                                 # The bending moment in a 2-link arm loaded at
                                 # the hand falls off linearly toward the tip, so
                                 # the section depth does too: ARM_FRONT_X at the
                                 # loaded end, ARM_TIP_X at the free one. Free in
                                 # print (x is the print HEIGHT here, so a varying
                                 # x-extent is an up-facing slope, never an
                                 # overhang) and it is 20 % of the link's mass.
ARM_RAIL_T = D.WALL              # 2.6 rail thickness in y. In the jog the rails
                                 # run at ~30 deg, so ~2.25 perpendicular -- still
                                 # 5 perimeters at a 0.4 nozzle.
ARM_SHAFT_Y = (-3.5, 10.5)       # upper-arm shaft, local y (0 == the arm plane).
                                 # Inboard face is 2.1 mm outboard of the shoulder
                                 # case face and 1.5 mm off the bracket's outer limit.
ARM_FORE_SHAFT_HY = 8.0          # forearm shaft half width (nothing inboard to dodge)
ARM_JOG_Z = (-108.0, -136.0)     # upper arm: shaft -> elbow fork, widening band
ARM_HAND_R = 12.0                # hand knuckle radius == the plant's hand sphere
                                 # (12 mm, friction 1.0 0.02 0.001). BARE PETG here;
                                 # the robustness sweep covered mu 0.3-1.0, 6/6.
ARM_EDGE_R = 3.0                 # outer vertical edge fillet ("no sharp corners")
ARM_R16_BUFFER = 1.0             # how far the upper arm's fork web stands off the
                                 # elbow servo's own swept circle (r = hypot(
                                 # SV_WID/2, SV_AXIS_FROM_OUT_END) = 15.97 -- the
                                 # "r 16 rule" leg_link's check_r16 guards). The
                                 # case turns WITH the forearm, so a web that
                                 # crosses that circle is an interference, not a
                                 # clearance question: see arm_v6.arm_upper_v6.

# Declared ROM. The plant's ranges are shoulder -90..200 and elbow -150..150;
# the CAD elbow is limited by the forearm's own grip channel meeting the upper
# arm's fork, which cad/v6/check_assembly_v6.py measures. The get-up only ever
# uses shoulder 90 -> 0 and elbow -90 -> 0, and the fold-up reaches shoulder 180.
# Sign: + about +Y, so + shoulder swings the arm BACKWARD, 0 = hanging down.
ARM_ROM = {"shoulder": (-90.0, 200.0), "elbow": (-100.0, 10.0)}
ARM_ROM_SIM = {"shoulder": (-90.0, 200.0), "elbow": (-150.0, 150.0)}

# --- deck interface -------------------------------------------------------
# 4x M2.5 self-tap into blind pilots in the 5 mm deck, EXACTLY the pattern
# neck_collar already uses (2.05 dia x 4.5 deep from the deck top). Positions
# are pelvis-frame (deck top z = 0), left side; the right side mirrors in y.
# Every one is checked against the real pelvis solid by arm_v6.check_deck_pilots.
ARM_PILOT_D = 2.05
ARM_PILOT_DEPTH = 4.5
# The quadrilateral is squashed on purpose. Three cuts box it in: the Pi slot
# (|y| <= 45 over x -51.76..-37.76) keeps the forward pair OUTBOARD, the
# battery aperture (x >= -36.51) keeps everything aft of it, and the bracket's
# own case-screw wall (y 47.15..50.15) must not be undercut by a countersink,
# which costs another 3 mm either side. What is left is: two on the aft ear,
# deep inboard, and two on the cradle floor outboard of the wall.
ARM_PILOT_XY = [(-61.0, 32.0), (-55.0, 32.0), (-61.0, 56.0), (-41.0, 56.0)]

# --- what the pair costs --------------------------------------------------
ARM_SERVO_COUNT = 4      # 2 per arm: shoulder pitch + elbow, both STS3215
SERVO_MASS_3215_ARM = None   # see parts_v6 rollup; kept out of the servo count
                             # dicts so the default (armless) rollup is unchanged

# ----------------------------------------------------------------------------
# sanity: the sim plant (sim/gen_plant_v6.py DesignParams) must agree
# ----------------------------------------------------------------------------
SIM_EXPECT = dict(hip_sep=HIP_SEP / 1e3, thigh=THIGH / 1e3, shank=SHIN / 1e3,
                  d_ankle=ANKLE_PITCH_TO_ROLL / 1e3, roll_h=ANKLE_ROLL_Z / 1e3,
                  foot_len=FOOT_L_LEN / 1e3, foot_w=FOOT_W / 1e3, foot_toe=FOOT_TOE / 1e3,
                  foot_y_off=FOOT_Y_OFF / 1e3, batt_layer_h=BATT_LAYER / 1e3)

if __name__ == "__main__":
    print(f"deck bottom {DECK_BOT_Z:.1f}, deck top {DECK_TOP_Z:.1f}, neck horn {NECK_HORN_Z:.1f}, "
          f"camera {CAM_Z:.1f}, top {TOP_Z:.1f} mm above ground")
    print(f"housing {HOUSING_LEN:.1f} x {2*HOUSING_HW:.1f}, x {HOUSING_X[0]:.1f}..{HOUSING_X[1]:.1f}")
    print(f"ankle roll z {ANKLE_ROLL_Z:.2f}, pitch z {ANKLE_PITCH_Z:.2f}, knee {KNEE_Z:.2f}, hip pitch {HIP_PITCH_Z:.2f}")
    print("sim expects", SIM_EXPECT)
