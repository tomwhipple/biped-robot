"""v6 body -- single source of truth for every NEW or CHANGED dimension (mm).

Everything the v5 parts already pinned about the servo (case, horn/idler
discs, screw rows, rib/platform detents, seat faces, countersinks, walls,
fits) is imported unchanged from ../dimensions.py as `D` and re-exported; a
v6 part must take a servo-interface number from `D`, never retype it. What
changes for v6 is the KINEMATICS (ankle roll, 84 mm hips, 110 mm segments),
the FOOT, the TORSO (Pi 4B, bigger pack, neck + head) and the LEG LINK's
section. Design record: docs/design-v6/2026-09-13-design-record.md; the numbers below are
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
# 35 mm, same 4x M3 on O14 discs). Only the mass differs. The robot
# (DESIGN.md section 4; design record 2026-09-13 section 14.4a): an STS3250 at
# each HIP ROLL, STS3215s everywhere else, the ankle rolls and knees with a
# raised position-loop P.
SERVO_3250_JOINTS = ("hip_roll",)                             # 2x STS3250
# the six-joint STS3250 set of the original assignment (design record
# section 8): the studies that bracket a mixed set read it (parts_v6
# SERVO_PLAN=3250, sim/build_v6_inertia.plant_xml(m3250_g=...))
SERVO_3250_JOINTS_SIX = ("hip_roll", "ankle_roll", "knee")
SERVO_MASS_3215 = 55.0
SERVO_MASS_3250 = 74.5

# --- disc screws: M3 button heads into a servo's horn or idler disc --------
# The discs are tapped through a thin FLANGE at the O14 bolt circle: 2.5 mm on
# the horn, 2.1 on the idler (D.HORN_THREAD / D.DISC_THREAD; bench-measured and
# confirmed on the vendor STEP). A screw must engage at least
# D.DISC_THREAD_MIN_ENGAGE and no deeper than the flange -- a longer one
# bottoms out and jacks the joint apart -- so the LENGTH follows the stack
# under the head, and there are no washers. Thread reaching the hole, by
# nominal length: M3x6 5.6 (bench), M3x8 7.6, M3x5 4.6 (the same 0.4 short as
# the measured M3x6; confirm on the first joint).
DISC_SCREW_THREAD = {5: 4.6, 6: D.DISC_SCREW_THREAD, 8: D.ROLL_DISC_SCREW_THREAD}


def disc_screw(stack, side):
    """(length, engagement, flange) of the SHORTEST M3 button head that
    engages a `side` ("horn" | "idler") disc through `stack` mm under its head
    within the rule above."""
    flange = D.HORN_THREAD if side == "horn" else D.DISC_THREAD
    for length in sorted(DISC_SCREW_THREAD):
        eng = DISC_SCREW_THREAD[length] - stack
        if D.DISC_THREAD_MIN_ENGAGE - 1e-9 <= eng <= flange + 1e-9:
            return length, eng, flange
    raise ValueError(f"no M3 disc screw engages a {side} disc through a {stack:.2f} mm stack")

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
# The yaw carrier (yaw_carrier_v6.py) is v5's with a spacer of YAW_CARRIER_DROP
# between its horn plate and the roll-servo bay: it puts the roll servo and its
# two upper wall screws below the hip-yaw bearing band with a full ACCESS_D
# driver path once the bearing and its housing are on (check_assembly_v6
# --joint access). v5's D.CARRIER_ROLL_AXIS (-40.11) stays as it is: the
# prototype's carrier and the 10-joint training plant mesh it.
YAW_CARRIER_DROP = 5.0
CARRIER_ROLL_AXIS = D.CARRIER_ROLL_AXIS - YAW_CARRIER_DROP           # -45.11, carrier-local (horn face z = 0)
HIP_YAW_Z = HIP_ROLL_Z - CARRIER_ROLL_AXIS                           # yaw horn face
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
    "hip_pitch": (-120.0, 90.0),   # -110 -> -125 (yoke_pitch_v6 flange chamfer, 2026-09-14) -> -120
                                   # (2026-09-24): -125 was never reachable -- the thigh's front wall met
                                   # the ROLL flange from -118.4 in BOTH yoke variants, a pair the split
                                   # sweep never listed. -120 is the deepest the joint clears at the
                                   # 0.7 mm rule after LL_HIP_RELIEF; past it the thigh's own servo case
                                   # is next. The get-up needs >= 119 (getup_search_r5_asdrawn_hip*.txt).
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
# HIP-FLEXION RELIEF (2026-09-24, hip-yoke-single-print.md section 6 item 1).
# At deep hip flexion the thigh's front wall -- the TOP edge of this box --
# swings up into the roll flange's front (in BOTH yoke variants: the flange is
# the same; the split sweep never listed the pair). First contact -118.4 deg.
# The get-up needs the tuck at -119 or deeper (it stands at -119/-121/-125
# and not at -118/-117, docs/design-v6/getup_search_r5_asdrawn_hip*.txt), and
# -125 is out of reach without cutting through this wall or the roll horn
# arm's root. So ROM["hip_pitch"] comes back to -122 and the box's top-front
# edge gets a chamfer (x run from the front face, z drop from LL_BOX_TOP),
# sized by check_assembly_v6's thigh-vs-hip_yoke row at -122 with the 0.7 mm
# rule. It faces UP-forward in this part's standing print: self-supporting.
LL_HIP_RELIEF = (3.5, 4.0)
# ...and, once that chamfer cleared the wall, the next thing the flange's
# front-bottom edge met at -121 was the idler jog block's front, 0.5 mm ABOVE
# the box top at x 13.2 -- 0.54 mm forward of the servo case face (SV_WID/2
# + FIT = 12.66), so that sliver locates nothing. Everything forward of
# LL_HIP_RELIEF_X0 is trimmed for LL_HIP_RELIEF_UP above the box top.
LL_HIP_RELIEF_X0 = D.SV_WID / 2 + D.FIT + 0.24      # 12.90: 0.24 clear of the case's fit
LL_HIP_RELIEF_UP = 3.0
LL_WEB_END = -(LL_DROP - 32.0)                # v5's WEB_END rule: 32 above the lower axis
LL_CABLE_WINDOW_Z = (-48.0, -37.0)            # v5 (upper-anchored: the lead leaves the case bottom)
LL_WEB_RAMP = 1.3                             # web bottom V: rise over run of its ramps (self-supporting standing)
LL_BRACE_Z = (LL_BOX_BOT - 4.0, LL_BOX_BOT)   # gable brace at the box bottom
# DEEP-FLEXION relief (2026-09-14, Tom: "we could probably bend the existing
# knees further ... with slight modifications of the leg links"): the knee is
# clear to 105 deg as drawn; from 110 to 130 the only contact is the SHIN's
# rear-top corner in the idler tine band (web top + the idler grip plate's
# outer skin, 16-33 mm below the knee axis) sweeping into the THIGH's idler
# tine 8-32 mm above the axis. LL_FLEX_CUT is the outline (x, z in the link
# frame) of what that tine sweeps through on the corner from 90 to 131 deg --
# every pose nests inside 131's, 105 mm3, bounded below by a 40 deg and a
# 24 deg edge -- offset 0.6 mm; its rear and top vertices sit outside the
# part. Cut through the idler band only, it leaves the lower idler grip
# screw's access bore 3.6 mm of wall. ~132 deg is the hard limit where the
# shin's web and the knee servo case meet the thigh's web end full-width.
# Same part serves the thigh, where the identical corner faces the pitch yoke
# at hip extension (+90, already 0.7 mm clear -- removing material only helps).
LL_FLEX_CUT = ((-16.16, -15.0), (-16.16, -29.0), (-8.1, -22.15), (8.1, -15.0))
# The idler grip plate stops 5 mm above its screws' countersinks, square
# across (Tom, 2026-10-06). Above the screws it is only the ~1 mm outer wall
# over the idler-platform detent; LL_FLEX_CUT still trims its rear corner.
LL_IDLER_PLATE_TOP = -D.CASE_HOLES_BOT[1] + D.CASE_CS_D / 2 + 5.0     # -25.05
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
# web end, at both feet of its V; the horn slab's face in the horn band).
# Two 45 deg corner chamfers buy the last 2 deg + buffer: the web end's
# outer-bottom corner (its whole width) and the jog block's rear-top corner
# (horn band only, y > the web's upper limit so the web is not notched).
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
# hip-yaw bearing: a 6810-2RS (50 x 65 x 7) between the yaw carrier's round
# hub and a recess in the bearing housing screwed under the pelvis
# (yaw_bearing_housing.py; docs/design-v6/study-yaw-bearing.md, option A; #75). Without it the yaw servo's horn and its 4 horn screws are
# the only connection between the carrier (the whole leg, a 0.335 m lever)
# and the pelvis, and the thrust (~15 N single-support) and roll moment
# (~0.65 N-m) all go through the servo's output shaft.
#
# The bearing band is the top of the carrier, from the horn face (carrier z =
# 0) down the bearing's own width. In it the carrier holds its horn plate
# (z 0..-3) and the top of the bay walls (the spacer, YAW_CARRIER_DROP); the
# roll servo starts below it (YAW_ROLL_CASE_TOP_Z). The hub wraps the walls'
# rectangular footprint (corners 25.12 from the yaw axis, trimmed 0.08 to the
# hub) and keeps the bay's open interior, which the horn screws are driven
# through; the bore is the smallest class (45: 6709/6809, 50: 6710/6810, 55:
# 6711/6811) whose hub leaves a 1.5 mm wall around that interior's corners:
YAW_BAY_VOID_CORNER_R = math.hypot(D.SV_TOPFACE, D.SV_WID / 2 + D.BAY_CHEEK_GAP)   # 21.48
YAW_BRG_ID = 50.0             # 6810-2RS bore
YAW_BRG_OD = 65.0             # 6810-2RS OD (6710-2RS is 62 mm, but its C0 varies
                              # 2.6-3.1 kN across sources; either is overkill here)
YAW_BRG_W = 7.0               # 6810-2RS width
# SKF 61810-2RS1 product data (SKF-generated sheet, https://docs.rs-online.com/
# f06f/A700000007138802.pdf): the ring shoulder diameters on the seal side,
# d1 (inner ring) and D2 (outer ring), the abutment limits da/Da, r1,2 min.
YAW_BRG_D1 = 54.67            # inner ring shoulder (approx.)
YAW_BRG_D2 = 61.8             # outer ring shoulder (approx.)
YAW_BRG_DA_MAX = 63.0         # largest abutment diameter on the outer ring face
YAW_BRG_R_MIN = 0.3           # ring chamfer
YAW_BRG_C_KN = 6.76
YAW_BRG_C0_KN = 6.8
YAW_BRG_MASS_G = 53.0
YAW_BRG_BOSS_OD = YAW_BRG_ID + 0.08           # 50.08, +0.08 interference on the inner race
YAW_BRG_BOSS_R = YAW_BRG_BOSS_OD / 2          # 25.04
YAW_BRG_BOSS_H = YAW_BRG_W                    # 7.0, full bore engagement: the band IS carrier z [-7, 0]
assert YAW_BRG_BOSS_R - YAW_BAY_VOID_CORNER_R >= 1.5, \
    "hub must keep a 1.5 mm wall around the bay interior's corners (within the band)"
_bore45_r = (45.0 + 0.08) / 2
assert _bore45_r - YAW_BAY_VOID_CORNER_R < 1.5, \
    "45 mm bore is supposed to FAIL this margin (negative control) -- re-check the 50 mm pick if this trips"
YAW_BRG_RECESS_ID = YAW_BRG_OD - 0.04         # 64.96, -0.04 interference on the outer race
YAW_BRG_RECESS_R = YAW_BRG_RECESS_ID / 2      # 32.48
# the race split at the ring shoulders: the inner race (turning) out to d1/2,
# the outer race (fixed) in from D2/2. The housing shoulder must bear on the
# outer ring face only -- outside D2 (the seal), inside Da max -- and keep off
# the inner race; the carrier must keep off the outer race.
YAW_BRG_INNER_RING_R = YAW_BRG_D1 / 2         # 27.335
YAW_BRG_OUTER_RING_R = YAW_BRG_D2 / 2         # 30.9
YAW_BRG_SHOULDER_LAND = 1.2                   # the ledge that stops the outer race's top face
YAW_BRG_SHOULDER_ID = YAW_BRG_OD - 2 * YAW_BRG_SHOULDER_LAND   # 62.6 (r 31.3)
assert YAW_BRG_D2 < YAW_BRG_SHOULDER_ID <= YAW_BRG_DA_MAX, \
    "the housing shoulder must bear on the outer ring face: outside D2 (the seal), inside Da max"
YAW_BRG_SKIRT_OD = YAW_BRG_OD + 6.0           # 71.0, ~3 mm wall around the recess (yaw_bearing_housing.py)
YAW_BRG_BAND_Z = (-YAW_BRG_BOSS_H, 0.0)       # carrier-local
YAW_ROLL_CASE_TOP_Z = CARRIER_ROLL_AXIS + D.SV_AXIS_FROM_REAR        # -10.00, carrier-local
assert YAW_ROLL_CASE_TOP_Z < YAW_BRG_BAND_Z[0], "the roll servo's case must start below the bearing band"
# roll-servo retention screw rows (carrier-local, the upper row of each wall):
# a driver on either must pass under the band (and the housing skirt hanging
# 0.5 mm below it) with ~1 mm to spare
YAW_ROLL_SCREW_ROWS_Z = (CARRIER_ROLL_AXIS + D.CASE_HOLES_TOP[1],
                         CARRIER_ROLL_AXIS + D.CASE_HOLES_BOT[1])    # -16.11, -12.36
assert max(YAW_ROLL_SCREW_ROWS_Z) + D.ACCESS_D / 2 <= YAW_BRG_BAND_Z[0] - 0.5 - 1.0, \
    "the upper roll-servo wall screws need a driver path under the bearing band and housing"
# pelvis-local: the recess ends flush at the horn face, and the housing's
# shoulder fills the existing 1.8 mm gap up to the cell-tube rim
# (D.YAW_BOX_CARRIER_GAP), against which the housing is screwed
YAW_BRG_RECESS_Z = (YAW_HORN_FACE_Z - YAW_BRG_W, YAW_HORN_FACE_Z)   # -80.80..-73.80
YAW_BRG_SHOULDER_Z = (YAW_BRG_RECESS_Z[1], YAW_BOX_BOT_Z)           # -73.80..-72.0
assert YAW_BRG_SHOULDER_Z[1] == YAW_BOX_BOT_Z, \
    "the housing must hang from the cell-tube rim it screws to"

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
# The pack lies on two 1.0 mm adhesive foam pads on the cell ceiling (BOM),
# not on the ceiling itself: the yaw servos' free-hub posts come up flush with
# the ceiling's top face (pelvis_v7: the post hole), and they turn with the
# hip. Printed rails there would be islands (the ceiling's pack face prints
# facing the bed), so the clearance is the pads'.
BATT_PAD_T = 1.0
BATT_Z = (CELL_TOP_Z + BATT_PAD_T, CELL_TOP_Z + BATT_PAD_T + BATT[2])  # -35.0..-9.0
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
# hip-yaw bearing HOUSING (yaw_bearing_housing.py): the bearings' outer-race
# seats (skirt, recess, shoulder) for both hips in one print, screwed UP to
# the underside of the cell block after the yaw servos, carriers, roll servos
# and bearings are in. It is not part of the pelvis print because the yaw
# servo goes up into its cell through the space the shoulder ring occupies
# (its cable end reaches SV_AXIS_FROM_REAR = 35.11 from the yaw axis; the
# ring is r 31.3..35.5), and the roll servo's upper idler-side screws are
# driven through the space the bearing and skirt occupy.
YAW_HOUSING_Z = (YAW_BRG_RECESS_Z[0] - 0.5, YAW_BOX_BOT_Z)   # -81.3..-72.0, pelvis-local
YAW_HOUSING_EAR_T = 4.0                  # the screw bars, from the top face down
YAW_HOUSING_SCREW_L = 8.0                # M2.5 x 8 flat head: 4.0 through the bar, 4.0 into the pelvis
YAW_HOUSING_PILOT_DEPTH = 5.0            # pelvis pilot, above YAW_BOX_BOT_Z
YAW_HOUSING_BOSS_H = 7.0                 # pelvis pilot boss, above YAW_BOX_BOT_Z (then a 45 deg cone)
YAW_HOUSING_BOSS_R = 3.0
# screw positions, pelvis-local (x, y). Where there is pelvis material
# outside the skirts, far enough from both hip axes that a 7 mm driver
# (D.ACCESS_D) clears each skirt, and off the Pi board (y edge 42.5, solder
# side x -41.51), the General Driver (PCB from x 18.01) and the servo cells:
# - rear-outboard: the rear cell wall where it meets the Pi channel wall;
# - rear-inboard: the inner cheek's rear end beside the centre channel;
# - front, on the centre line: the cell block's front web (x 10.41..13.01,
#   |y| < 14). The front end cap is only 2.6 mm thick, so a boss there would
#   stand proud of the pelvis's front face.
YAW_HOUSING_SCREWS_L = (
    (-40.0, 46.8),    # rear-outboard (L; R mirrors in y)
    (-36.5, 25.5),    # rear-inboard
)
YAW_HOUSING_SCREW_FRONT = (11.7, 0.0)
YAW_HOUSING_SCREWS = tuple((x, s * y) for s in (1, -1) for x, y in YAW_HOUSING_SCREWS_L) + (YAW_HOUSING_SCREW_FRONT,)
for _hx, _hy in YAW_HOUSING_SCREWS:
    for _by in (HIP_SEP / 2, -HIP_SEP / 2):
        assert math.hypot(_hx, _hy - _by) >= YAW_BRG_SKIRT_OD / 2 + D.ACCESS_D / 2 + 0.5, \
            "a housing screw is too close to a skirt for the driver"
assert D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP <= YAW_HOUSING_SCREW_FRONT[0] <= CELL_X[1], \
    "the front housing screw must land on the cell block's front web"
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
NECK_WELL_D = 3.0                             # seat depth below the deck top (the floor's top face)
NECK_WELL_HW = (D.SV_WID / 2 + D.YAW_SEAT_GAP, )                     # +-12.66 across
NECK_WELL_X = (-D.SV_AXIS_FROM_REAR - D.YAW_SEAT_GAP, D.SV_AXIS_FROM_OUT_END + D.YAW_SEAT_GAP)   # -35.41..+10.41
# The deck's battery + power-board aperture (pelvis_v7 cuts it; the pack lifts
# out through it). The neck servo's footprint lies ENTIRELY inside it, so the
# deck cannot seat the neck: the seat is a separate FLOOR PLATE (neck_floor.py)
# screwed to the neck tube's walls, which drops into this aperture.
DECK_APER_X = (BATT_X[0] - 0.5, min(PWR_X[1] + 0.5, D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP))   # -36.51..+10.40
DECK_APER_HY = HIP_SEP / 2 + D.YAW_BOX_HW_OUT - D.WALL                                  # 54.66
# --- neck floor (issue #90) --------------------------------------------------
# A 2 mm plate under the neck servo at the old well depth, so the neck's height
# is unchanged: floor top at -NECK_WELL_D, floor bottom at the deck-bottom plane.
# Four PADS stand up from it to the case's idler face (z = 0) at the four stator
# screw rows (the yaw cells' pattern: M2.5 x 8 flat-heads UP into the idler-face
# rows 8.30 / 32.75, +-10.25), trimmed clear of the rotating idler disc and hub
# and of the moulded back-cover platform, which all stand proud of that face.
# The plate is its own print (it hangs below the girdle's print plane) and is
# held by four M2.5 x 8 flat-heads UP into bosses on the neck tube's side walls.
NECK_FLOOR_Z = (-DECK_T, -NECK_WELL_D)                                # (-5.0, -3.0)
NECK_FLOOR_X = (DECK_APER_X[0] + 0.3, DECK_APER_X[1] - 0.3)           # 0.3 in from the aperture edges
NECK_FLOOR_HY = NECK_WELL_HW[0] + D.WALL                              # 15.26 == the tube's outer face
NECK_PAD_R = 3.5                                                      # the yaw-cell stator pad radius
NECK_PAD_XY = [(-row, s * D.CASE_HOLE_LAT) for row in D.YAW_CASE_HOLES_IDLER for s in (1, -1)]
NECK_DISC_RELIEF_R = 19.2 / 2 + 0.6                                   # idler disc O19.2 turns: 0.6 air
NECK_HUB_RELIEF = (4.5, 0.6)                                          # (r, depth) pocket under the idler hub
NECK_STATOR_CB = 1.0                                                  # counterbore under each stator head:
                                                                      # the head seats 1 mm up, M2.5 x 8 bites 4.0
NECK_LUG_X = (-26.0, 4.0)                                             # between the aft tie (x <= -31) and the
                                                                      # clavicle (x >= 10.61): open air above
NECK_LUG_Y = NECK_FLOOR_HY + 2.5                                      # 17.76: bosses overlap the tube wall 1 mm
NECK_LUG_R = 4.5                                                      # 1.5 mm of wall round the 6 mm counterbore
NECK_LUG_CB = 3.0                                                     # counterbore: the head seats at z -2.0
NECK_BOSS_R = 3.5                                                     # boss on the tube wall; pilot wall 2.5
NECK_BOSS_H = 9.0                                                     # boss on the tube wall, z 0..9 (prints base-down)
NECK_BOSS_PILOT = 7.0                                                 # 2.05 pilot depth; M2.5 x 8 bites 6.0
NECK_LEAD_WIN = (-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 1.0)        # x band under the connector trench
assert NECK_FLOOR_Z[0] - (BATT_Z[1]) >= 4.0, "neck floor must keep >= 4 mm over the pack"
assert NECK_FLOOR_X[0] <= -max(D.YAW_CASE_HOLES_IDLER) - NECK_PAD_R + 0.1, "aft pads must fit on the floor"
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

# --- the shoulder servo's pose: LYING FORE-AFT, WRAPPED BY THE TORSO -------
# Round 4 stood the servo ON END on the deck lid (case length vertical) in a
# bracket bolted down with four M2.5 self-taps, hanging 21.7 mm past the deck
# edge. Tom, 2026-09-19: *"looks like the arms are just bolted on. Aside from
# being brittle, it looks ugly. We should rotate the servo bodies 90deg and
# incorporate them into the torso, then put the head above them, so they look
# like real shoulders"* ... *"and put the shoulder joint in the same plane as
# the hips."*
#
# The case is rotated 90 deg ABOUT ITS OWN OUTPUT AXIS. The axis is still
# lateral (+-Y, horn OUTBOARD) -- it has to be for a sagittal-plane shoulder,
# which is why SV_CASE_T (34.70) still runs along Y and why NOTHING about the
# arm LINKS changes. What changes is which of the other two case dimensions
# stands up:
#     round 4:  case LENGTH (45.22) vertical, axis SV_AXIS_FROM_OUT_END above
#               the case's lower END -> a tower standing on the lid
#     round 5:  case WIDTH  (24.72) vertical, axis SV_WID/2 above the case's
#               lower LONG FACE, case LENGTH running FORE-AFT
# The servo now lies along the top of the torso like a scapula instead of
# standing on it, and with ARM_SHOULDER_X = 0 its footprint is
# x -35.11..+10.11 (cable end AFT) -- the SAME x band the neck servo already
# occupies (NECK_WELL_X -35.41..+10.41). The three servos at the top of the
# torso line up in one transverse block, and shoulder_girdle_v6 is the single
# printed part that wraps all three.
ARM_SHOULDER_X = 0.0     # "the same plane as the hips": directly above the hip
                         # yaw axis, not 50 mm aft. This undoes round 4b's aft
                         # mount, whose ONLY job was keeping the hanging arm
                         # clear of the swinging leg -- so it was re-measured,
                         # not assumed. See ARM_REST below and
                         # docs/design-v6/shoulder-girdle.md section 2.
GIRDLE_FLOOR = 4.0       # bay floor under the servo's lower long face. Same
                         # 4.0 the round-4 cradle used, and it is what keeps
                         # the whole girdle FLAT-BOTTOMED at the deck plane --
                         # the part prints base-down, so nothing may hang below
                         # z = 0 (see the module docstring's print block).
ARM_AXIS_Z = GIRDLE_FLOOR + D.SV_WID / 2                                  # 16.36 above the deck top
ARM_SHOULDER_Z = DECK_TOP_Z + ARM_AXIS_Z                                  # 476.63
ARM_ELBOW_Z = ARM_SHOULDER_Z - ARM_UPPER                                  # 316.63
ARM_HAND_Z = ARM_ELBOW_Z - ARM_FORE                                       # 156.63
# DEVIATION 1 (shoulder height), unchanged in kind from round 4 and barely
# changed in size: the plant put the shoulder 79.0 mm above the yaw axis. The
# CAD deck top is 73.80 above it, and a real case lying on a 4 mm floor puts
# its axis 16.36 higher still -> 90.16, i.e. +11.2. (Round 4 was +8.9. The
# extra 2.3 mm is the difference between resting on the case's END and resting
# on its long FACE.) Measured at the as-drawn height: the walk gate config
# `r5_girdle_ondeck` uses arm_z = 0.09016 exactly, and passes 4/4.
ARM_SHOULDER_ABOVE_YAW = ARM_SHOULDER_Z - HIP_YAW_Z                       # 90.16
ARM_SHOULDER_Z_SIM = 79.0    # what the get-up was measured at (m*1e3, above yaw)
# The servo's fore-aft footprint, in the pelvis frame, with the CABLE end AFT.
# Aft on purpose: the Pi and the General Driver are both in the aft half of
# the housing, so the lead runs the short way; and it puts the case's mass
# behind the joint, which is the direction the arm swings to push.
ARM_CASE_X = (ARM_SHOULDER_X - D.SV_AXIS_FROM_REAR,
              ARM_SHOULDER_X + D.SV_AXIS_FROM_OUT_END)                    # -35.11..+10.11

# --- the arm plane, and why it is where it is ------------------------------
# Round 4 derived ARM_Y from the Pi slide slot. Round 5 derives it from the
# DECK EDGE, because the rotated case is no longer over the lid at all: it
# lies OUTBOARD of the housing skin, and the girdle's inboard wall has to fit
# in between with a driver's worth of nothing in it. Chain, outward from the
# skin -- every step is a real thickness, none of it is chosen:
GIRDLE_SKIN_AIR = 1.24                                   # wall inner face off the skin
GIRDLE_WALL_Y0 = HOUSING_HW + GIRDLE_SKIN_AIR            # 62.50
GIRDLE_WALL_T = D.GRIP_PLATE_T_IDLER                     # 3.00
ARM_SEAT_Y = GIRDLE_WALL_Y0 + GIRDLE_WALL_T              # 65.50 seat face (case lands here)
ARM_SERVO_MID_Y = ARM_SEAT_Y + D.GRIP_SEAT_CLR - D.SV_IDLER_CASE_FACE     # 80.40 case mid-plane
ARM_CASE_Y1 = ARM_SERVO_MID_Y + D.SV_TOPFACE             # 97.75 case outboard (horn-side) face
ARM_HORN_FACE_Y = ARM_SERVO_MID_Y + D.SV_HORN_FACE       # 100.85 horn mounting face
GIRDLE_GRIP_T = D.GRIP_PLATE_T                           # 2.40 outboard grip plate
GIRDLE_Y1 = ARM_CASE_Y1 + GIRDLE_GRIP_T                  # 100.15 girdle's outer face
ARM_Y = ARM_HORN_FACE_Y + D.HORN_BOSS_H + D.PLATE / 2    # 103.35 the arm plane
ARM_Y_SIM = 80.35
# The grip plate is on the HORN (outboard) face, not the idler face round 4
# used, and that is an ACCESS result, not a preference: with the case lying
# against the torso there is GIRDLE_SKIN_AIR = 1.24 mm between the girdle's
# inboard wall and the housing skin, and no driver reaches into a 1.24 mm
# slot. The horn-side face is open air. Precedent: leg_link's own horn-side
# grip plate, with D.GRIP_HORN_RELIEF around the O19.6 case boss.
assert GIRDLE_Y1 < ARM_HORN_FACE_Y, \
    "the static grip plate must stay clear of the plane the ROTATING arm plate bolts to"
GIRDLE_ARM_CLR = ARM_HORN_FACE_Y - GIRDLE_Y1             # 0.70 mm of air, arm vs girdle
GIRDLE_WIDTH = 2 * GIRDLE_Y1                             # 200.30
assert GIRDLE_WIDTH <= D.BED, "the girdle must fit the print bed in one piece"
# DEVIATION 2 (lateral): the plant hung the arm at 80.35. 103.35 here, +23.0.
# Round 4 was +7.7 and it was forced by the Pi slot; this one is forced by the
# deck edge (above) -- and it is ALSO what the walk gate wants once the
# shoulder comes forward to x = 0, so the structural number and the measured
# one agree instead of fighting. See docs/design-v6/shoulder-girdle.md.

# --- the arms' rest pose ---------------------------------------------------
# Folded (Tom, 2026-10-06): the shoulder 15 deg FORWARD and the elbow folded
# 95 deg, so the forearm points forward and 20 deg up. The arm actuators are
# not in the walk timeline; this is what they are commanded to whenever they
# are not doing something else (standing, walking), and the plant's qpos0.
# Chosen by sweep (every leg joint through its full ROM, one at a time as
# check_assembly_v6 does, plus a walking grid hip roll +-15 x hip pitch
# -45..30 x knee 0..70; docs/design-v6/arm_rest_search_2026-10-06.txt):
#   shoulder 0..+60 (down or back), elbow -90..-100: the forearm sits at the
#     hip yoke's height and an outward hip roll past ~30 deg drives the yoke,
#     the pitch servo and the thigh into it (up to 12000 mm3);
#   shoulder -10: 99 mm3 at full roll; -12.5: clear by 2.65 mm;
#   shoulder -15 .. -35: clear, by >= 7.4 mm (a bounding-box floor; the CAD
#     gate measures -15 / -95 at >= 8.55 mm); the walking grid by >= 6.0 mm.
# -15 is the smallest forward lean with the full margin, which keeps the arms'
# mass forward shift smallest (+69 mm per arm, about +7 mm on the robot's
# CoM); -95 leaves 5 deg to the elbow's -100 limit.
ARM_REST_SHOULDER = -15.0   # deg (+ = backward, 0 = straight down)
ARM_REST_ELBOW = -95.0      # deg (- = folded, the forearm swinging forward)
ARM_REST = (ARM_REST_SHOULDER, ARM_REST_ELBOW)

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
# upper-arm shaft, local y (0 == the arm plane). The INBOARD bound is not a
# free number and never was: the shaft sweeps past the shoulder mount every
# time the arm swings, so it is derived FROM the mount's outer face with
# ARM_SHAFT_CLR of air. Round 4 quoted -3.5 against a bracket whose outer
# limit was 83.0; round 5's girdle reaches GIRDLE_Y1 = 100.15 against an arm
# plane of 103.35, and -3.5 put the shaft 0.30 mm INSIDE it -- which is
# exactly what check_assembly_v6's `arm_upper vs shoulder_girdle_v6` row
# caught (0.47 mm3 of overlap at shoulder +55 deg).
# --- the head: the arm's SECTION runs to the shoulder, it does not flatten --
# Tom, 2026-09-21, selecting the upper arm's outboard head face in FreeCAD:
# "the highlighted face seems very brittle, let's reinforce that" ... and then,
# on being shown a merely thicker plate: "not quite what I had in mind. Let's
# continue the thickness of the upper arm all the way to the shoulder, using a
# box structure on 3 sides, with the outward side open for screw access."
#
# He is right twice over. The head WAS brittle, and thickening the plate was
# the wrong fix. The head is a SINGLE-SIDED plate on the shoulder horn -- there
# is no idler-side tine and there cannot be one (the servo's idler face looks
# inboard at the torso) -- so one plate carries a 320 mm arm. In its own plane
# that is easy (1.59 N-m peak against Z = 369 mm3). About X -- a sideways force
# at the hand, the robot falling onto the arm, a hand catching -- the flat
# 3 mm plate had Z = 41 mm3, so 20 N at the hand was ~157 MPa in a ~50 MPa
# material. It snaps.
#
# Measured (the arithmetic is in docs/design-v6/shoulder-girdle.md section 9):
#     flat plate, D.PLATE 3.0      Z =  41 mm3     157 MPa    <- what was there
#     plate thickened to 6.0       Z = 163 mm3      39 MPa    <- rejected
#     the SHAFT's own box section  Z = 583 mm3      11 MPa    <- this
# A thicker slab only moves material away from ONE face. The shaft's section
# already puts flanges at BOTH y extremes and joins them with the aft web, and
# carrying that section up to the shoulder instead of collapsing it into a
# plate is 14.3x the flat plate for less material than the 6 mm slab.
#
# WHICH three sides is a PRINT result, not a choice. The arm prints web-face-
# down (RY_XUP, model +X up -- the orientation that puts the filament along the
# arm; see arm_v6's print block). A FRONT wall in that orientation is a 9 mm
# unsupported ledge at the top of the print -- the same "flat ceiling spanning
# tine to tine" finding that made leg_link_v6 an open C. So the three sides are
# the three the shaft already has -- aft web + inboard flange (the horn plate)
# + outboard flange -- opening FORWARD, and the head simply stops being an
# exception to the rest of the arm.
# ...and CLOSED at the front. Tom, 2026-09-24: "strengthen the forearm-
# shoulder joint by connecting the inner and outer face in front. If there is
# to be an opening, have it on the side facing away from the body." So the
# head is a closed four-sided box -- aft web, FRONT WALL, inboard flange (the
# horn plate), outboard flange -- and its only opening is the screw-access
# bore through the outboard flange.
#
# The front wall IS printable here, where it was not in an open-outboard box:
# its first layer (in RY_XUP the front is the TOP of the print) spans from the
# inboard flange to the outboard flange, a BRIDGE anchored on both sides of
# ~6.4 mm, under check_printability's BRIDGE_OK of 8. Without the outboard
# flange under it, it would have been a 9 mm one-sided ledge.
#
# The front wall cannot stay at ARM_FRONT_X (12.0): its inner face would be at
# 9.4, and the forward disc screw's O5.7 head reaches x = 7 + 2.85 = 9.85 --
# the wall would sit ON the screw. So the head's front edge moves forward to
# give the head its clearance, and the shaft keeps its own 12.0.
ARM_HEAD_FRONT_CLR = 0.95
ARM_HEAD_WALL_IN_X = D.BCD / 2 + D.M3_HEAD_D / 2 + ARM_HEAD_FRONT_CLR   # 10.80
ARM_HEAD_FRONT_X = ARM_HEAD_WALL_IN_X + D.WALL                           # 13.40
# The opening on the side away from the body: one bore through the OUTBOARD
# flange on the shoulder axis. r >= 9.85 to pass the button heads; and r is
# capped from above too, because the bore's teardrop roof now ends against the
# front wall's underside, leaving a flat bridge 2*(r*sqrt2 - WALL_IN_X) wide
# that has to stay under BRIDGE_OK (8 mm). 10.3 -> 7.5 mm.
ARM_HEAD_ACCESS_R = 10.3
assert ARM_HEAD_ACCESS_R > D.BCD / 2 + D.M3_HEAD_D / 2, \
    "the head's access bore must clear the four M3 button heads"
assert 2 * (ARM_HEAD_ACCESS_R * 2 ** 0.5 - ARM_HEAD_WALL_IN_X) < 8.0, \
    "the access bore's roof, truncated by the front wall, would be an unbridgeable span"

ARM_SHAFT_CLR = 1.5
ARM_SHAFT_Y = (GIRDLE_Y1 + ARM_SHAFT_CLR - ARM_Y, 10.5)     # (-1.70, 10.5)
assert ARM_Y + ARM_SHAFT_Y[0] >= GIRDLE_Y1 + 1.0, \
    "the upper arm's shaft sweeps into the girdle"
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
# The girdle bolts down with M2.5 self-taps into blind pilots in the 5 mm
# deck (2.05 dia x 4.5 deep from the deck top) -- the same interface the neck
# collar was drawn with. WHERE they go is not a drawing decision: the deck
# here is cut by the Pi slide slot (x -51.76..-37.76, |y| <= 45), the battery
# aperture (x -36.51..+10.41, |y| < 54.66), the General Driver lead slot and
# the R3 top fillet, and a 4.5 mm pilot in the wrong place is a hole into the
# battery bay. Every position below was found by PROBING the real pelvis solid
# on a grid and is re-verified on every build by
# shoulder_girdle_v6.check_deck_pilots().
#
# What survives the probe is one continuous strip either side -- y 55..59, the
# deck's edge beam over the housing side wall -- plus the solid land forward
# of the battery aperture (x >= +10.5). So the girdle bolts on two long rails
# and one forward beam, which is also exactly the load path the arm wants.
#
# (The probe also found a PRE-EXISTING defect it was not looking for: all four
# of neck_collar.py's own flange pilots, (-42.01, +-19.26) and (+17.01,
# +-19.26), are 100 % outside the deck solid -- the aft pair lands in the Pi
# slide slot, the forward pair in the GD lead slot. The collar has nothing to
# bite. The girdle absorbs the collar and replaces those four with the
# verified set below; docs/design-v6/shoulder-girdle.md section 6.)
ARM_PILOT_D = 2.05
ARM_PILOT_DEPTH = 4.5
# 2026-09-24 correction: the first cut also put pilots at (14, +-20),
# (14, +-40) and (-32, +-57). All six were BURIED -- under the clavicle beam,
# the aft tie and the trapezius webs (12..31 mm of material), with only a
# O2.9 clearance bore above each seat, so no flat-head could ever reach its
# pilot. Every pilot now sits on the open rail strip, BETWEEN the two web
# bands (aft tie x -36..-31, clavicle x 10.61..16), where nothing but the
# 4 mm rail is above it and a 7 mm driver clears the pod's inboard wall.
GIRDLE_RAIL_PILOT_X = (-27.0, -18.0, -9.0, 0.0, 6.0)   # +6 not +8: a 7 mm driver at +8 clips the clavicle web
GIRDLE_RAIL_PILOT_Y = 57.0        # between the aperture edge (54.66) and the
                                  # deck's R3 top fillet (starts at 58.26)
GIRDLE_PILOT_XY = [(x, s * GIRDLE_RAIL_PILOT_Y) for x in GIRDLE_RAIL_PILOT_X for s in (1, -1)]

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
                  foot_y_off=FOOT_Y_OFF / 1e3, batt_layer_h=BATT_LAYER / 1e3,
                  d_yaw_roll=-CARRIER_ROLL_AXIS / 1e3)

if __name__ == "__main__":
    print(f"deck bottom {DECK_BOT_Z:.1f}, deck top {DECK_TOP_Z:.1f}, neck horn {NECK_HORN_Z:.1f}, "
          f"camera {CAM_Z:.1f}, top {TOP_Z:.1f} mm above ground")
    print(f"housing {HOUSING_LEN:.1f} x {2*HOUSING_HW:.1f}, x {HOUSING_X[0]:.1f}..{HOUSING_X[1]:.1f}")
    print(f"ankle roll z {ANKLE_ROLL_Z:.2f}, pitch z {ANKLE_PITCH_Z:.2f}, knee {KNEE_Z:.2f}, hip pitch {HIP_PITCH_Z:.2f}")
    print("sim expects", SIM_EXPECT)
