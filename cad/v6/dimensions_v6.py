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
FOOT_IN = FOOT_W / 2 - FOOT_Y_OFF                                    # 25 inboard half
FOOT_OUT = FOOT_W / 2 + FOOT_Y_OFF                                   # 51 outboard half
FOOT_CORNER_R = 14.0
FOOT_INNER_GAP = HIP_SEP - 2 * FOOT_IN                               # 34
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
# hip-yaw bearing (2026-09-17, docs/design-v6/study-yaw-bearing.md): today the
# yaw servo's HORN SPLINE + 4 horn screws are the only connection between the
# carrier (whole leg, 0.335 m lever) and the pelvis -- thrust (~15 N single-
# support) and the roll moment (~0.65 N-m) both go through the shaft. A
# 6707-2RS deep-groove ball bearing (35x44x5) goes around the CARRIER'S
# HORN-PLATE RIM (not the servo's O19.2 horn disc -- a boss that tight would
# leave a 0.4 mm wall) so the pelvis carries thrust + moment through its own
# printed material and the servo shaft goes back to pure torque.
#
# Geometry that sizes the bore: the servo case's own footprint (half-width
# D.SV_WID/2 = 12.36 Y, reach D.SV_AXIS_FROM_OUT_END = 10.11 fwd) has a
# forward-corner distance from the axis of hypot(10.11, 12.36) = 15.97 mm --
# the tightest the case ever gets to the axis.
#
# But the carrier's own horn-mount PLATE is bigger than the disc it grips --
# it is ALSO the roll bay's ceiling (X +/-19.95, Y +/-hw=15.26, so its own
# CORNER is hypot(19.95, 15.26) = 25.12 mm from the axis) and that full
# plate -- not just a boss -- sweeps through the recess's Z-band as the leg
# yaws. A first pass at this used a 6707-2RS (35x44x5, boss r 17.45): its
# 22 mm recess radius does NOT clear the plate's 25.12 mm corner (found by
# check_assembly_v6.py --joint hip_yaw: 240 mm3 overlap growing with yaw
# angle). 6709-2RS (45x55x6) fixes it -- recess radius 27.475 mm clears the
# corner by 2.35 mm -- at the cost of 1 mm more axial stack (6 mm vs 5) and
# a larger boss ring welded onto the OUTSIDE of the existing plate/bay
# footprint (does not touch the plate's existing structural connections to
# the bay walls).
#
# It lives entirely BELOW the existing yaw cell tube rim (YAW_BOX_BOT_Z,
# unchanged) -- the GD/Pi board columns and the servo case both end at or
# above that Z, so the whole assembly sits in air that was already open.
# NO axis drop, no change to YAW_HORN_FACE_Z/TORSO_FLOOR_Z/HIP_YAW_Z/anything
# in the leg chain below the carrier's own plate.
YAW_BRG_ID = 45.0                              # 6709-2RS bore
YAW_BRG_OD = 55.0                              # 6709-2RS OD
YAW_BRG_W = 6.0                                # 6709-2RS width
YAW_BRG_BOSS_OD = YAW_BRG_ID + 0.10            # 45.10, +0.10 interference (press
                                                # fit; no bearing precedent in
                                                # dimensions.py -- new for this
                                                # study, verify/adjust after the
                                                # first print)
YAW_BRG_BOSS_R = YAW_BRG_BOSS_OD / 2           # 22.55 -- clears the case's
                                                # 15.97 mm corner by 6.6 mm
YAW_BRG_BOSS_H = YAW_BRG_W                     # 6.0, full bore engagement -- the
                                                # boss radius clears the case at
                                                # any height so there is no need
                                                # to shorten it
YAW_BRG_RECESS_ID = YAW_BRG_OD + 1.5            # 56.5, +1.5 CLEARANCE (not a
                                                # press fit) against the outer
                                                # race -- widened from the
                                                # first-pass -0.05 interference
                                                # after check_assembly_v6 found
                                                # only 0.38 mm min distance to
                                                # the plate's 25.12 mm corner at
                                                # +-0.05 (< D.SWEEP_BUFFER 0.5);
                                                # retention is the shoulder +
                                                # retaining compound (see the
                                                # write-up), not an interference
                                                # fit on the outer race
YAW_BRG_SHOULDER_LAND = 2.0                    # ledge width (radial) that stops
                                                # the outer race's top face --
                                                # reacts the leg's upward thrust.
                                                # The shoulder's own Z-band sits
                                                # ABOVE the plate (see the Z
                                                # derivation below), so it does
                                                # not need the plate's clearance.
YAW_BRG_SHOULDER_ID = YAW_BRG_OD - 2 * YAW_BRG_SHOULDER_LAND   # 51.0
YAW_BRG_SKIRT_OD = 62.0                        # ~3.5 mm wall around the recess;
                                                # two skirts (84 mm hip sep) leave
                                                # 22 mm edge-to-edge, and both sit
                                                # entirely below where GD/Pi/the
                                                # battery layer exist, so nothing
                                                # else routes through that gap
                                                # below z = YAW_BOX_BOT_Z
# skirt/recess Z (pelvis-local, hangs from the EXISTING rim, nothing above
# YAW_BOX_BOT_Z moves). The recess is pinned to YAW_HORN_FACE_Z (the world
# reference the carrier's boss ALSO uses -- HIP_YAW_Z in assembly_v6.py --
# so the two align exactly): recess proper is the bearing's own width,
# ending flush at the horn face; the shoulder fills the remaining, already-
# existing 1.8 mm gap up to the tube rim (D.YAW_BOX_CARRIER_GAP, unchanged).
YAW_BRG_RECESS_Z = (YAW_HORN_FACE_Z - YAW_BRG_W, YAW_HORN_FACE_Z)              # -78.80..-73.80
YAW_BRG_SHOULDER_Z = (YAW_BRG_RECESS_Z[1], YAW_BOX_BOT_Z)                      # -73.80..-72.0
YAW_BRG_SHOULDER_H = YAW_BRG_SHOULDER_Z[1] - YAW_BRG_SHOULDER_Z[0]             # 1.80 == D.YAW_BOX_CARRIER_GAP
# carrier boss Z, in the CARRIER's own local frame (z=0 at the horn face,
# +Z toward the servo -- see cad/parts.py yaw_carrier docstring): the boss
# hangs from the plate's top face DOWN into the carrier's own body, matching
# the pelvis recess when the leg hangs at HIP_YAW_Z in the standing pose.
YAW_BRG_BOSS_CARRIER_Z = (-YAW_BRG_BOSS_H, 0.0)                                # -6.0..0.0
assert YAW_BRG_BOSS_R + 1.0 > D.BCD / 2 + 3.0, "boss must clear the horn screws"
_case_corner_r = math.hypot(D.SV_AXIS_FROM_OUT_END, D.SV_WID / 2)   # 15.97
assert YAW_BRG_BOSS_R > _case_corner_r, "boss radius must clear the servo case's own corner"
assert YAW_BRG_SHOULDER_Z[1] == YAW_BOX_BOT_Z, "skirt must hang from the EXISTING rim, unmoved"

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
