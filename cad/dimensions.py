"""Single source of truth for all mechanical dimensions (mm).

SERVO DATA SOURCE
-----------------
Feetech STS3215 (== Waveshare ST3215; same case as SCS215 drawing).
Measured directly from the official Waveshare STEP model and 2D drawing:
  - https://files.waveshare.com/upload/5/59/ST3215-3D.zip   (ST3215.step)
  - https://files.waveshare.com/upload/0/08/ST3215-2D.zip   (ST3215.pdf, DWG "SCS215")
  - https://www.waveshare.com/wiki/ST3215_Servo             (specs, 45.22 x 35 x 24.72)
Key verified facts (STEP-measured, matches 2D drawing):
  - Case 45.22 long x 24.72 wide; output axis 10.11 from the output end, centered
    in width.  Case is 34.7 across the output-axis direction (top face to bottom
    face); output boss O19.6 x 0.85 on top; drawing calls out 35 overall (case+boss)
    and 37.25 = horn top face to idler disc face.
  - Metal horn: O19.2 disc, mounting face 3.1 above the case top face, with
    4x M3 threaded holes (O2.5 modeled) on a O14 bolt circle at 0/90/180/270 deg,
    center screw recessed (~flush).  Included hardware: M3x6 machine screws
    (Waveshare wiki / vendor listings).
  - Rear idler: O19.2 free disc, 3.35 thick, recessed 0.55 below the case bottom
    face inside a O25 opening; SAME 4x M3 on O14 BC pattern -> joints can be
    supported on both sides (horn + idler) without extra bearings.
  - Case mounting holes (O3.5 in case, glass-filled nylon; take M3 self-tappers /
    M3 machine screws after tapping - community practice, e.g. thingiverse 7074577):
      horn-side face : rows 8.30 and 29.00 behind the axis, +/-10.25 across width
      idler-side face: rows 8.30 and 32.75 behind the axis, +/-10.25 across width
  - Servo mass ~55 g (DESIGN.md measured; vendor 55-60 g).
"""

# ----------------------------------------------------------------------------
# STS3215 servo (verified, see module docstring)
# ----------------------------------------------------------------------------
SV_LEN = 45.22          # case length (along the limb)
SV_WID = 24.72          # case width
SV_CASE_T = 34.70       # case thickness along output axis (top face .. bottom face)
SV_AXIS_FROM_OUT_END = 10.11   # output axis to the nearer (output) end of the case
SV_AXIS_FROM_REAR = SV_LEN - SV_AXIS_FROM_OUT_END   # 35.11, cable end

# distances measured from the case mid-plane along the output axis
SV_TOPFACE = SV_CASE_T / 2            # +17.35  (horn-side case face)
SV_BOTFACE = -SV_CASE_T / 2           # -17.35  (idler-side case face)
SV_HORN_FACE = 20.45                  # horn mounting face (flat, screw recessed)
SV_IDLER_FACE = -16.80                # idler disc face (0.55 recessed in case)
SV_BOSS_D = 19.6                      # output boss dia (on case top face)
SV_HORN_D = 19.2                      # horn / idler disc dia
SV_IDLER_RECESS_D = 25.0              # opening in case bottom around idler disc
SV_GRIP_SPAN = SV_HORN_FACE - SV_IDLER_FACE   # 37.25 (drawing-confirmed)

# horn/idler disc screw pattern: 4x M3 threaded on O14 bolt circle
BCD = 14.0
PAD_HOLE = 3.4          # M3 clearance in printed pads
PAD_D = 24.0            # printed pad (arm end) diameter
HORN_CENTER_RELIEF_D = 9.0   # relief over the (recessed) horn center screw
IDLER_CENTER_RELIEF_D = 10.0

# case mounting hole rows (distance behind the output axis; lateral +/-10.25)
CASE_HOLES_TOP = (8.30, 29.00)     # horn-side face
CASE_HOLES_BOT = (8.30, 32.75)     # idler-side face
CASE_HOLE_LAT = 10.25
CASE_SCREW_PILOT = 2.8             # in printed part: M3 self-tap clearance-ish
CASE_SCREW_CLEAR = 3.4             # where the screw just passes through

SERVO_MASS = 55.0       # g

# ----------------------------------------------------------------------------
# printability / fits
# ----------------------------------------------------------------------------
WALL = 2.6              # standard printed wall (>= 2.4 = 3 perimeters @ 0.4 nozzle)
PLATE = 3.0             # joint arm plates
FIT = 0.30              # moving / drop-in fit
HEATSET_D = 4.1         # M3 x D4.6 heat-set insert pilot
HEATSET_L = 6.0
M3_CLEAR = 3.4
M25_TAP = 2.2           # M2.5 self-tap pilot (electronics board)

# ----------------------------------------------------------------------------
# joint arm interface (identical at every joint)
#   horn side : plate PLATE thick; on yokes a O16.5 x 1.0 boss clears the pelvis
#               wall; on leg links the plate sits directly on the horn face.
#   idler side: plate PLATE thick + O19.0 boss that reaches into the O25 recess
#               and seats on the idler disc (case-bottom clearance 0.65).
# ----------------------------------------------------------------------------
HORN_BOSS_D = 16.5
HORN_BOSS_H = 1.0       # yokes only
IDLER_BOSS_D = 19.0
IDLER_ARM_INNER = -18.0                    # arm plate inner face (from servo mid)
IDLER_BOSS_H = IDLER_ARM_INNER - SV_IDLER_FACE  # 1.2

# ----------------------------------------------------------------------------
# kinematic layout (matches sim/bimo_biped.xml)
# ----------------------------------------------------------------------------
HIP_SEP = 56.0          # leg center-to-center
ROLL_TO_PITCH = 50.0    # hip-roll axis to hip-pitch axis
THIGH = 90.0            # hip-pitch axis to knee axis
SHIN = 90.0             # knee axis to ankle axis

# hip yokes: roll axis -> flange -> pitch axis (16 + 4 + 4 + 26 = 50)
# ROLL_AXIS_TO_FLANGE = 16 so the flange top edge (17 half-width) still clears
# the pelvis bay wall bottoms when rolled +/-25 deg (16*cos25 - 17*sin25 = 7.3
# below the roll axis vs wall bottoms at 5.9 below -> 1.4 clearance).
YOKE_FLANGE_T = 4.0
ROLL_AXIS_TO_FLANGE = 16.0
PITCH_ARM_REACH = ROLL_TO_PITCH - ROLL_AXIS_TO_FLANGE - 2 * YOKE_FLANGE_T  # 26
YOKE_FLANGE_X = 32.0     # flange fore-aft size (limited: swings past bay walls)
YOKE_FLANGE_Y = 34.0
YOKE_BOLT_SQ = 20.0      # 4x M3 on +/-10 square (90 deg rotationally symmetric)

# ----------------------------------------------------------------------------
# pelvis (deck + two hanging servo bays for the hip-roll servos)
# ----------------------------------------------------------------------------
DECK_L = 104.0          # across the robot (Y in sim)
DECK_W = 46.0           # fore-aft (X in sim)
DECK_T = 5.0
BAY_WALL_DROP = 41.0    # walls hang this far below the deck; roll-axis bore is a
                        # downward-open U-slot (servo slides up into the bay)
BAY_BORE = 22.5         # clearance bore around case boss / idler boss in bay walls
BAY_CHEEK_GAP = 0.3
TOWER_FOOT_X = 14.0     # tower feet / deck heat-set positions
TOWER_FOOT_Y = 42.0     # lands over the bay cheek walls (extra thread depth)

# ----------------------------------------------------------------------------
# leg link (thigh and shin are the SAME part)
#   grips the upper servo's case below its horn (which is the upper joint) and
#   forks down to the next servo's horn+idler, DROP = 90 between axes.
# ----------------------------------------------------------------------------
LINK_DROP = 90.0        # == THIGH == SHIN
GRIP_PLATE_T = 2.4
GRIP_TOP_HORN = -3.0    # top edge of horn-side grip plate (below upper axis);
GRIP_HORN_RELIEF = 10.3 # ...with a circular relief around the O19.6 case boss
GRIP_TOP_IDLER = -16.0  # idler side must clear the upper yoke arm sweep (R12+4)
GRIP_BOT = -36.0        # just past the case bottom end (-35.11)
WEB_GAP = 0.4
WEB_TOP = -16.0         # clears the upper joint's fork arms folding to 95 deg
WEB_END = -58.0         # web stops 32 above the lower axis: clears the foot
                        # walls and the servo case top at ankle/knee extremes
# fork arms: full-width near the web, narrowed toward the pad so the slab does
# not sweep into the foot walls / servo case top at -40..-95 deg joint angles
FORK_WIDE_Z = -50.0     # wide (to web outer face) above this
FORK_NARROW_X = -10.5   # slab back edge below FORK_WIDE_Z (front edge +12)

# ----------------------------------------------------------------------------
# foot
# ----------------------------------------------------------------------------
FOOT_L = 96.0           # fore-aft;  ankle axis 38 from the heel edge
FOOT_W = 52.0
FOOT_T = 6.0
FOOT_HEEL = 38.0        # ankle axis to rear edge (case rear end at -35.11)
FOOT_POCKET_D = 2.0     # servo lies in this recess
PAD_RECESS = 1.5        # TPU sole pad recess (pad 2.0 thick -> 0.5 proud)
PAD_INSET = 3.0
FOOT_WALL_X = (-36.5, -26.0)   # rear retention walls (cover holes at -29/-32.75)
FOOT_WALL_H = 26.0
FOOT_WALL_T = 2.4       # thin: shin-fork horn arm passes 0.4 outside it
ANKLE_AXIS_ABOVE_SOLE = FOOT_T - FOOT_POCKET_D + SV_WID / 2  # 16.36 (+0.5 TPU proud)

# ----------------------------------------------------------------------------
# torso tower (electronics: driver board on top, battery on deck under it)
# ----------------------------------------------------------------------------
TOWER_L = 96.0
TOWER_W = 42.0
TOWER_H = 32.0          # deck top .. tower top
TOWER_TOP_T = 3.5
# driver board: Waveshare Bus-Servo Driver (ESP32) ~65 x 30 -- VERIFY yours!
BOARD_HOLES = (58.0, 24.0)     # hole pattern (x span, y span), M2.5 self-tap
BATT = (75.0, 35.0, 16.0)      # 2S 450-1000 mAh LiPo envelope, velcro-strapped

# ----------------------------------------------------------------------------
# assembly heights (sole ground contact = 0); torso center Zc for reference
# ----------------------------------------------------------------------------
TPU_PROUD = 0.5
ANKLE_Z = ANKLE_AXIS_ABOVE_SOLE + TPU_PROUD          # 16.86
KNEE_Z = ANKLE_Z + SHIN                              # 106.86
HIP_PITCH_Z = KNEE_Z + THIGH                         # 196.86
HIP_ROLL_Z = HIP_PITCH_Z + ROLL_TO_PITCH             # 246.86
DECK_BOT_Z = HIP_ROLL_Z + SV_AXIS_FROM_REAR          # 282.0 (servo top end)
TORSO_CENTER_Z = HIP_ROLL_Z + 36.0                   # 282.9 (sim: 280)
TOP_Z = DECK_BOT_Z + DECK_T + TOWER_H                # 319.0 overall

PLA_RHO = 1.24e-3       # g/mm^3
PRINT_MASS_FACTOR = 0.90  # thin-walled parts print near-solid; grid infill on thick

BED = 220.0             # print bed (square)
