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
HORN_CENTER_RELIEF_D = 8.0   # relief over the (recessed) horn center screw.
IDLER_CENTER_RELIEF_D = 8.0  # Both sized so the web to the O14-BC M3 holes
                        # stays printable: gap = 7 - relief_r - 1.7. At the
                        # old O9/O10 it was 0.8/0.3 mm (flakes); O8 gives 1.3.
                        # Center screw head is ~O5.7 recessed -> 1.1 mm slack.
                        # VERIFY on the real horn/idler disc before final print.

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
IDLER_BOSS_D = 20.0     # was 19.0. Grown 2026-07-23 when the idler bolt circle
                        # was finally drilled THROUGH (see parts.py bugfix): the
                        # 4x O3.4 clearance holes on the O14 circle reach r8.7,
                        # leaving only 0.8 mm to an O19 boss OD (sub-2-perimeter,
                        # check_printability THIN). O20 restores a 1.3 mm web and
                        # still slips through the LOCKED BAY_BORE 20.6 U-slot at
                        # the yoke_roll idler arm (0.3 mm/side == FIT). Do not
                        # grow past ~20.3 without revisiting BAY_BORE.
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
# Clearance bore around case boss / idler boss in the bay walls. Sized off the
# O19.6 case output boss (the larger of the two -- the yoke idler boss is O19.0)
# at the SAME 0.5 radial slip the leg_link grip plate already proves on that
# boss (GRIP_HORN_RELIEF = 10.3). Was 22.5 (1.45 radial), which is slack the
# wall cannot afford: the case screw row at 8.30 behind the axis sits only
# hypot(10.25, 8.30) = 13.19 from the axis, so an O3.4 clearance hole reaches
# in to r 11.49 and a 11.25 bore left a 0.24 mm web -- under one extrusion, and
# the slicer merged hole and bore into a sliver (print review 2026-07-15).
# 10.3 restores a 1.19 mm web, matching leg_link against the same hole row.
# LOCKED: do not grow past ~20.9 without moving that screw row (it is on the
# servo, so it cannot move) or dropping it.
BAY_BORE = SV_BOSS_D + 1.0      # 20.6
BAY_CHEEK_GAP = 0.3
TOWER_FOOT_X = 14.0     # tower feet / deck heat-set positions
TOWER_FOOT_Y = 42.0     # lands over the bay cheek walls: heat-set pilots run
                        # through the 5 mm deck into cheek-wall material below
                        # (no raised bosses -- printed deck-top-down, bosses put
                        # the whole first layer 2 mm in the air, and they
                        # overlapped the tower feet tabs)

# ----------------------------------------------------------------------------
# HIP YAW (v3yaw variant) -- one STS3215 per leg, lying FLAT under the deck
# with its output axis VERTICAL and the horn pointing DOWN. A printed
# `yaw_carrier` bolts to the horn and carries the hip-roll bay that used to
# hang off the pelvis. The roll-bay geometry is UNCHANGED (same BAY_BORE,
# BAY_WALL_DROP, BAY_CHEEK_GAP, cheek walls, U-slot, CASE_HOLES retention) --
# it just moves from `pelvis` onto `yaw_carrier`.  See docs/hip-yaw-study.md.
#
# Servo orientation: case LENGTH along X (fore-aft), width (24.72) along Y,
# case thickness (34.70) along the vertical output axis.  Length-along-Y also
# fits (the two 45.22 servos leave 56-45.22 = 10.8 mm between them at HIP_SEP),
# but it crowds the 104 mm deck ends and puts the cable ends across the
# centre; length-along-X keeps the narrow 24.72 face across Y (clears the other
# leg easily) and routes both cables straight out the rear.  Output (near) end
# forward at +SV_AXIS_FROM_OUT_END; cable (far) end rearward.  Vertical output
# axis at each leg centre (x=0, y=+/-HIP_SEP/2).
#
# Vertical placement (pelvis frame: deck top = 0, deck bottom = -DECK_T): the
# idler-side case face is pressed flat to the deck underside and the case is
# bolted to the deck THROUGH that face (the idler-side hole rows), so the deck
# is the yaw stator bracket.  The horn face therefore hangs SV_GRIP_SPAN below.
YAW_IDLER_FACE_Z = -DECK_T                          # -5.00  case idler-side face
YAW_IDLER_DISC_Z = YAW_IDLER_FACE_Z - 0.55          # -5.55  idler disc (0.55 in)
YAW_HORN_FACE_Z = YAW_IDLER_DISC_Z - SV_GRIP_SPAN   # -42.80 horn mounting face
YAW_CASE_BOT_Z = YAW_IDLER_FACE_Z - SV_CASE_T       # -39.70 horn-side case face
YAW_CASE_X_FRONT = SV_AXIS_FROM_OUT_END             # +10.11 output end (front)
YAW_CASE_X_REAR = -SV_AXIS_FROM_REAR                # -35.11 cable end (rear)

# GRIP DECISION -- HORN-ONLY (single-sided).  Bolting the stator to the deck
# uses the idler-side case face, which is the same face the carrier would need
# to reach to grip the idler disc, so the two are mutually exclusive at this
# axis.  We take the rigid deck-bolted stator (4x M3 into the idler-side rows)
# and drive the carrier off the horn alone -- the Open Duck Mini arrangement
# for this exact servo and leg.  The both-sides alternative (float the stator
# on a keyed pocket, grip both discs, deck counterbore for the idler arm) buys
# a second bearing at the cost of stator rigidity + a tolerance stack; it is
# the flagged v3.1 upgrade if bench testing shows output-shaft bending play.
# See docs/hip-yaw-study.md "as-designed".
YAW_CARRIER_PLATE = PLATE          # 3.0 horn mount plate (== every joint arm).
                                   # Sits flat on the O19.2 horn disc; only the
                                   # recessed centre screw needs a relief
                                   # (HORN_CENTER_RELIEF_D, as the yokes use).
YAW_FOOT_REACH = 16.0              # the yaw seat side walls run forward to here
                                   # so the +x tower-foot heat-sets (x=14) land
                                   # in collar material (the -x feet already sit
                                   # over the case span); keeps TOWER_FOOT_X/Y.

# deck seat: a shallow collar hanging off the deck underside that wraps the top
# of each yaw case (keys it against reaction torque + locates it) and a
# rearward tab so all four idler-side screws (rows 8.30 AND 32.75 behind the
# axis, i.e. x = -8.30 and -35.11+2.36... the 32.75 row sits at x=-32.75, 9.8 mm
# behind the deck rear edge -23) land in deck material.
YAW_SEAT_DROP = 4.0                # collar reaches this far below the deck
YAW_SEAT_GAP = FIT                 # 0.30 slip fit of case into the collar
YAW_SEAT_WALL = WALL               # 2.6 collar wall thickness (== standard wall)
YAW_CASE_HOLES_IDLER = CASE_HOLES_BOT   # (8.30, 32.75) idler-side rows -> stator

# carrier-borne hip-roll bay, re-referenced to the carrier's own frame (horn
# face = z 0, +Z toward the servo).  The horn plate top mates the yaw horn; the
# roll bay hangs below, roll axis SV_AXIS_FROM_REAR under the bay ceiling (roll
# servo output end DOWN, cable end UP -- exactly as in the old pelvis bay).
CARRIER_ROLL_CEIL = -YAW_CARRIER_PLATE                     # -3.00 bay ceiling
CARRIER_ROLL_AXIS = CARRIER_ROLL_CEIL - SV_AXIS_FROM_REAR  # -38.11 roll axis

# --- yaw stack drop (the honest number; the study estimated ~25-30 mm) ---
# The hip-roll axis moves from (DECK_T + SV_AXIS_FROM_REAR) below the deck top
# to (yaw grip span + carrier plate + recess) deeper.  The drop is the gap
# between the old and new roll-axis planes, all arithmetic:
ROLL_BELOW_DECK_OLD = DECK_T + SV_AXIS_FROM_REAR            # 40.11 (8-DOF build)
ROLL_BELOW_DECK_YAW = -(YAW_HORN_FACE_Z + CARRIER_ROLL_AXIS)  # 80.91 (v3yaw)
YAW_STACK_DROP = ROLL_BELOW_DECK_YAW - ROLL_BELOW_DECK_OLD  # 40.80 mm

YAW_SWEEP = 45.0                   # design yaw half-range each way (deg)

# --- wire routing (v3yaw harness) ------------------------------------------
# Bus daisy-chain, 10 servos over 2 legs (design review 2026-07-23: the flat
# yaw cases sit on the deck underside exactly where the old per-bay deck
# cutouts were, so those are deleted and REPLACED by the openings below --
# without them the board->yaw lead and the roll-servo cable have no passage
# and pierce the deck / collar).
#   board (tower) --[deck rear chase]--> hip-YAW rear-end port (faces -X, below
#     the deck rear overhang) --[open gap]--> hip-ROLL top port (in the carrier
#     bay) --[carrier rear channel]--> down the leg (pitch->knee->ankle use the
#     UNCHANGED leg_link web windows + foot cable window).
# Each ST3215 lead is a 3-wire JST bundle; a daisy pass at a servo carries 2
# bundles (the incoming + the outgoing lead).
WIRE_BUNDLE = 5.0        # one 3-wire ST3215 lead bundle, outer dia
WIRE_PLUG_W = 9.0        # ST3215 JST housing width (assemble connector-first)
WIRE_CHASE_1 = 12.0      # slot width for 1 bundle+plug with grommet margin
WIRE_CHASE_2 = 14.0      # slot width for 2 bundles (daisy in + out at a servo);
                         # +/-7 keeps a 1.55 mm ligament to the roll retention
                         # screw hole at y +/-10.25 in the carrier rear wall
# The hip-ROLL servo's cable connectors point UP out of the bay ceiling, right
# where the carrier's yaw-horn plate sits -- a SOLID plate caps them (the plate
# is 3 mm thick, the gap above it to the yaw-case underside is only 3.1 mm, and
# the plate centre is taken by the O19.2 horn). The ST3215 pair sits on the REAR
# of the cable-end face (the cable exits toward the daisy), so the carrier's
# rear cable channel is opened forward to YAW_CH_FRONT to uncap them, and the
# rear-most of the 4 yaw-horn bolts is DROPPED so the channel can reach that far
# (3x M3 on the O14 circle is ample for the ~0.3 N*m yaw torque). A connector at
# the rear then pokes up through the opened ceiling into the 3.1 mm gap and the
# lead routes out. Wire audit 2026-07-23. (Verified with a connector-block
# clearance sweep across the cable-end face.)
YAW_CH_FRONT = -6.0      # carrier rear cable channel reaches this far forward:
                         # covers the roll connectors (which sit at x < -9.6,
                         # off the O19.2 horn) while keeping a 2 mm wall to the
                         # O8 centre relief (edge at -4) and to the side horn
                         # bolts (x=0, y=+/-7)

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
# The gripped servo's cable/connector exits the case BOTTOM END toward the idler
# (-Y) side, but the idler grip plate + jog block + idler fork wall that side
# solid from z-16 down past the case bottom (user report / probe 2026-07-23: a
# lead routed down the idler side cut 171 mm3 into the plate). Notch the idler
# side at the cable end for a 3-wire JST lead + plug. Sits BELOW the idler case
# screw (z-32.75) and far ABOVE the (now through-drilled) lower bolt circle
# (z-90) and boss, so it weakens neither; over the modelled port x-band (-5..-11).
LINK_IDLER_NOTCH_X = (-12.5, -4.0)    # x span of the notch (over the ports);
                                      # narrow (8.5 mm) so the idler fork arm
                                      # keeps ~19 of its 27 mm width for load
LINK_IDLER_NOTCH_Z = (-49.0, -37.0)   # z band: starts just BELOW the crowded
                                      # servo-bottom / jog-block / idler-case-
                                      # screw cluster (z -33..-36.5) so it cuts
                                      # no sub-perimeter sliver there, down to
                                      # near the fork wide/narrow break. Clears
                                      # the connector plug + lead (which hang
                                      # below the case bottom -35.11)
LINK_CABLE_R = 2.0                     # rounded notch corners (no shear on lead)
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
FOOT_L = 116.0          # fore-aft; enlarged 100 -> 116 for the get-up rise
                        # corridor (DESIGN 2026-07-15): the pike-up tips BACKWARD
                        # mid-rise and the corridor was +/-20 mm CoP on the old
                        # 90 mm pad, so growth is heel-biased (heel +10, toe +6).
                        # Width stays 52: lateral wasn't the failure mode and the
                        # shin-fork sweep bounds the wall band anyway.
FOOT_W = 52.0
FOOT_T = 6.0
FOOT_HEEL = 52.0        # ankle axis to rear edge (case rear end at -35.11;
                        # was 42 -- see FOOT_L note)
FOOT_POCKET_D = 2.0     # servo lies in this recess
# Sole underside is now FLAT (no pad recess): the old 90x46 pad pocket bridged a
# 46 mm span printed sole-down and sagged badly in PETG. The TPU/rubber pad is
# glued to the flat underside instead. Keep it THIN (~0.5 mm == TPU_PROUD) so
# stance height is unchanged; a thicker pad raises the robot by (thickness-0.5).
FOOT_WALL_X = (-40.0, -26.0)   # rear retention tabs; lengthened AFT -36.5 -> -40
                        # (+3.5 mm root toward the heel). Fore stays at -26: the
                        # shin fork sweeps forward of that at ankle -40 (relief
                        # slots). Still covers the case holes at -29/-32.75.
FOOT_WALL_H = 26.0
FOOT_WALL_T = 2.4       # thin: shin-fork horn arm passes 0.4 outside it (LOCKED:
                        # cannot thicken outward -> reinforce via the bulkhead)
# v3: heel bulkhead between the tab aft ends. v2's free-standing 2.4 mm blades
# snapped across layer lines under a LATERAL knock (the aft gusset only helped
# fore-aft); the bulkhead closes each blade into an L/U-channel section, which
# is stiff in both directions and unloads the layer-bond root. All faces
# vertical -> adds nothing for the printer to bridge. Inner face 1.1 clear of
# the servo case rear end (-35.11 - FIT).
FOOT_BULK_X = (-40.0, -36.5)
FOOT_CABLE_W = 16.0     # cable window in the bulkhead, open at the top: the
FOOT_CABLE_Z = 8.0      # servo cable exits the rear END face (same connector
                        # zone the pelvis deck cutout clears: center +/-8)
# aft base buttress (heel side, clear of the fork sweep): ONE full-width wedge
# bracing both tabs + the bulkhead. Its sloped face is a TOP surface printing
# sole-down, so any steepness is support-free. Ends at the heel edge (-42).
FOOT_WALL_GUSSET_AFT = (2.0, 18.0)  # aft buttress (x-run toward heel, z-height)
# ankle-servo retention (assembly.md §3, 4x M3x8 self-tap through the tabs into
# the case rows -29 / -32.75). The rows sit at z = pocket-floor + {2.11, 22.61}
# = 6.11 (LOW) and 26.61 (HIGH). The LOW head lands right at the sole top
# (FOOT_T = 6) and the sole extends FOOT_W/2 OUTBOARD of the tabs -- a 6 mm
# shelf that blocks both the head and the Y-driver (probe 2026-07-23; user
# report). Divot: relieve that shelf TOP over each LOW screw down to
# FOOT_DIVOT_FLOOR, from the tab outer face out through the sole edge. The pad
# still bonds to the FULL flat z=0 underside -- the divot is top-side only, so
# adhesive area is unchanged. Prints sole-down = an upward-open pocket (no
# bridge). The HIGH row clears the sole and needs no divot.
FOOT_DIVOT_HW = 5.5     # divot half-length along X (M3 button O5.7 + driver +
                        # margin so the hex driver seats square on the head)
FOOT_DIVOT_FLOOR = 2.0  # sole left under the divot (pad backs it from below);
                        # clears the driver socket down to z~2.6 at the z6.11 row
ANKLE_AXIS_ABOVE_SOLE = FOOT_T - FOOT_POCKET_D + SV_WID / 2  # 16.36 (+pad proud)

# ----------------------------------------------------------------------------
# torso tower (electronics inside: board hangs face-down on standoffs under
# the top plate -- the top now carries the GoPro mount; battery on the deck)
# ----------------------------------------------------------------------------
TOWER_L = 96.0
TOWER_W = 42.0
TOWER_H = 43.5          # deck top .. tower top. Pack sits on the deck, so the
                        # board underside (TOWER_H - TOWER_TOP_T - BOARD_STANDOFF
                        # - ~4 component) must clear BATT[2]: TOWER_H = BATT[2]
                        # + 13.5 + 3.5 gap. +6.5 over the Zeee-only bay.
TOWER_TOP_T = 3.5
# driver board: Waveshare "Servo Driver with ESP32", 65 x 30, holes O2.75 on
# a 58 x 23 grid (wiki spec 2026-07 -- still verify on the real board)
BOARD_HOLES = (58.0, 23.0)     # hole pattern (y span, x span), M2.5 self-tap
BOARD_STANDOFF = 6.0           # under-plate standoff height (clears the GoPro
                               # screw bosses by 3 mm; battery below gets ~2 mm)
# battery: DECIDED 3S (2026-07-11 gauntlet verdict). Envelope is a SUPERSET of
# the 3S 850 mAh XT30 field, not one pack -- the Zeee (67 x 30 x 18.5, 74 g)
# it was originally cut for went unavailable, and every other pack in the class
# is stubbier but taller. Measured 2026-07-15:
#   Zeee   100C  67 x 30   x 18.5   74 g   (original; direct-only, US stock out)
#   Tattu   45C  60 x 30   x 22     76 g   B0BWRR3FFP (2-pack, XT30)
#   Ovonic  80C  59 x 29.7 x 22.9   74 g   B09CTSCWYM (2-pack, XT30)
#   Tattu   75C  59 x 30   x 24    ~80 g   B07218SB7L
#   CNHL    70C  62 x 30   x 25    ~80 g   B0C4PQRTYG (2-pack, XT30)
# Height is the binding axis, so BATT[2] = 25 (tallest) + 1.5 fit/pad headroom;
# length stays 68 to keep the flat Zeee seatable if it ever returns. Short packs
# (59-62) leave up to 8 mm of Y slop -- the belt + ribbon takes it up; shim if
# it rattles. Pack side-loads through a window in the -X tower wall (tool-free
# swap: peel strap, tug pull-ribbon). Everything below derives from BATT: edit
# it and reprint the tower.
BATT = (68.0, 31.0, 26.5)      # y length, x width, z height (envelope)
# The specific pack modeled in the mocks / sim inertia: worst case of the field
# above (CNHL 70C) so fit-checks and COM are conservative. A flatter, lighter
# pack only gains clearance and lowers COM. Keep BATT >= BATT_PACK.
BATT_PACK = (62.0, 30.0, 25.0)  # y length, x width, z height (actual pack)
BATT_PACK_MASS = 80.0           # g
BATT_SEAT_X = -17.0            # pack outer (-x) face when seated: 1.4 inside
                               # the wall inner face, so the strap can preload

# ----------------------------------------------------------------------------
# GoPro three-prong mount (separate bolt-on part `gopro_base` on the tower top
# -- printable upright with zero overhangs, and it is a sacrificial crash fuse).
# Prong geometry follows the proven GoProScad standard
# (github.com/ridercz/GoProScad: legs 3.0 thick, O15 round top, M5 hole
# centered 7.5 below the top, leg height 17, 3-leg stack 3+slot+3+slot+3).
# Slot width 3.2 (~2.95 mm camera fingers + 0.25 fit) per spec; GoProScad
# ships 3.5 -- if the GoPro MAX fingers bind, ream or set GP_SLOT=3.5, reprint.
# Camera: original GoPro MAX 360, 154 g, ~64 x 69 x 25 mm body, built-in
# folding two-finger mount; prongs stacked along Y => lens axis fore-aft.
# ----------------------------------------------------------------------------
GP_PRONG_T = 3.0
GP_SLOT = 3.2
GP_STACK = 3 * GP_PRONG_T + 2 * GP_SLOT   # 15.4 across
GP_PRONG_OD = 15.0
GP_LEG_H = 17.0                # base top -> prong top
GP_HOLE_D = 5.5                # M5 + 0.5 print tolerance
GP_HOLE_H = GP_LEG_H - GP_PRONG_OD / 2    # 9.5 above the base top
# The M5 clamp bore is horizontal in the print orientation (base down), so its
# top arc bridges -> weak, sag-prone strands exactly where the thumbscrew clamps
# across the layers. Teardrop the bore (45 deg self-supporting roof peak) so no
# layer overhangs > 45 deg. The screw is a clearance fit, so the added void
# above the bolt is harmless. Set False for a plain round bore.
GP_HOLE_TEARDROP = True
GP_BASE_X, GP_BASE_Y, GP_BASE_T = 30.0, 24.0, 4.0
GP_SCREW_XY = (11.0, 8.5)      # 4x M3 self-tap into bosses under the tower top

# TODO (2026-07-22): IMU is now the GY-BNO085 (Teyleten, B0CL26J81F) -- a
# THIRD board outline. The constants below still describe the classic
# BNO055 breakout that the current printed carrier fits. MEASURE the
# GY-BNO085's outline + hole pattern on arrival, update IMU_PCB/IMU_HOLES,
# and reprint imu_carrier (both gates must re-pass). Firmware side already
# targets the BNO085 (SH-2) -- see docs/firmware-design.md.
# BNO055 IMU + its carrier plate. The ordered part (2026-07-16, Amazon
# B0GVK81HXR) is the CLASSIC Adafruit BNO055 breakout (2472 layout: solder
# header, no STEMMA jacks) -- outline/holes measured from Adafruit's Eagle
# .brd (Adafruit-BNO055-Breakout-PCB @ master, "Adafruit BNO055.brd"; the
# STEMMA QT variant is 25.4 wide with holes at 20.32 x 15.24 -- reprint the
# carrier with these two lines swapped if the board in hand has JST jacks).
# The carrier sandwiches between the tower top and gopro_base on the SAME
# 4 screws (now M3x12: +3 mm of carrier in the stack) and cantilevers a
# tongue rearward that the IMU screws onto -- no interior tower flat fits
# the breakout (all <13 mm) and tape mounting was rejected (2026-07-16).
IMU_PCB = (26.67, 20.32, 1.6)  # breakout outline x, y, pcb thickness
IMU_HOLES = (21.59, 15.24)     # mounting-hole pattern (x, y), 4x Ø2.5 plated
IMU_CY = -25.0                 # IMU center y on the tongue (x centered): holes
                               # clear the feet-screw wells at (+-14, +-42)
IMU_CARRIER_T = 3.0            # carrier plate thickness
IMU_BOSS_H = 4.5               # boss height: M2.5x8 through the 1.6 pcb needs
                               # 6.4 blind; boss+plate = 7.5 with a 0.4 floor;
                               # also clears the soldered header pins under
                               # the pcb (~3 mm proud)
CAM_MASS = 154.0               # g, incl. battery
CAM_BODY = (25.0, 64.0, 69.0)  # X depth (lens axis fore-aft), Y width, Z height

# ----------------------------------------------------------------------------
# assembly heights (sole ground contact = 0); torso center Zc for reference
# ----------------------------------------------------------------------------
TPU_PROUD = 1.6         # actual pad: 1/16" self-adhesive silicone sheet, cut
                        # to 106 x 46 (Amazon B0FJ8TBMQK, 2x 6"x6" sheets --
                        # one sheet yields both pads). Full thickness is proud
                        # of the FLAT sole, so stance height carries all 1.6.
ANKLE_Z = ANKLE_AXIS_ABOVE_SOLE + TPU_PROUD          # 17.96
KNEE_Z = ANKLE_Z + SHIN                              # 106.86
HIP_PITCH_Z = KNEE_Z + THIGH                         # 196.86
HIP_ROLL_Z = HIP_PITCH_Z + ROLL_TO_PITCH             # 246.86
DECK_BOT_Z = HIP_ROLL_Z + SV_AXIS_FROM_REAR          # 282.0 (servo top end)
TORSO_CENTER_Z = HIP_ROLL_Z + 36.0                   # 282.9 (sim: 280)
TOP_Z = DECK_BOT_Z + DECK_T + TOWER_H                # 319.0 overall

# v3yaw variant: the LEG is unchanged, so ANKLE..HIP_ROLL heights above ground
# are unchanged; the yaw servo + carrier are inserted BETWEEN the roll axis and
# the deck, so everything from the deck up rises by YAW_STACK_DROP.  The hip-yaw
# "joint plane" is the horn face, CARRIER_ROLL_AXIS above the roll axis.
HIP_YAW_Z = HIP_ROLL_Z - CARRIER_ROLL_AXIS           # 284.97 (yaw horn face)
DECK_BOT_Z_YAW = HIP_ROLL_Z + ROLL_BELOW_DECK_YAW    # 327.77 (deck bottom)
TORSO_CENTER_Z_YAW = TORSO_CENTER_Z + YAW_STACK_DROP  # 323.7
TOP_Z_YAW = TOP_Z + YAW_STACK_DROP                   # 359.8 overall (v3yaw)

# Filament density for mass/inertia estimates. Robot is printed in PETG
# (~1.27 g/mm^3); PLA would be 1.24e-3 if you switch back.
FILAMENT_RHO = 1.27e-3  # g/mm^3 (PETG)
PRINT_MASS_FACTOR = 0.90  # thin-walled parts print near-solid; grid infill on thick

BED = 220.0             # print bed (square)
