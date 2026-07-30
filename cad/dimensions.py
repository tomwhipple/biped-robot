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
# --- WHEEL ENDS, sectioned off cad/vendor/ST3215.step 2026-07-30 (radial
# profile about the output axis, our frame). Both discs stand PROUD of their
# case faces with a clear annular moat around them -- there is no "O25 recess
# with the disc 0.55 down inside it", which is what SV_IDLER_RECESS_D and the
# SV_IDLER_FACE comment above still describe. Kept only because the mocks cut
# a recess with it; nothing seats on it.
#   horn:  disc face +20.45 out to r 9.6, then NOTHING to r 10.5, case/rib
#          beyond r 11.0 -- disc stands 3.10 proud of SV_TOPFACE
#   idler: hub post -17.35 (r<=3.2), disc face -16.80 out to r 9.6, then
#          NOTHING out to r 11.0, case face -14.75 beyond -- disc 2.05 proud
# The moats are why the pads may be grown (see PAD_D): past the disc rim the
# arm is over open air until the case face, which is 2-3 mm below it.
SV_DISC_R = 9.6                       # horn AND idler disc, measured
SV_HORN_MOAT_R = 10.5                 # void around the horn disc reaches here
SV_IDLER_MOAT_R = 11.0                # void around the idler disc reaches here
SV_GRIP_SPAN = SV_HORN_FACE - SV_IDLER_FACE   # 37.25 (drawing-confirmed)

# horn/idler disc screw pattern: 4x M3 threaded on O14 bolt circle
BCD = 14.0
PAD_HOLE = 3.4          # M3 clearance in printed pads
# Disc-bolt pad OD. 24.0 -> 20.0 on 2026-07-27 (user found it on the bench).
# The pad is centred on the joint axis, and the SERVO'S OWN case screws sit at
# radius hypot(CASE_HOLE_LAT, CASE_HOLES_x[0]) = 13.19 with 5.7 heads standing
# 1.65 proud of each case face -- so their heads reach IN to radius 10.34, and
# a 12.0 pad radius buried 1.66 mm into them. Measured: 18.92 mm3 on
# yoke_pitch's drive side, 7.45 mm3 on yoke_roll's.
# 10.0 clears by 0.34 and still leaves 1.3 mm of rim outboard of the O14 bolt
# circle -- the same rim minimum yoke_pitch's hub cut already works to.
#
# HELD AT 20.0 on 2026-07-30 after re-deriving it (user: "check the wheel end
# of the servos for fit"). Two things changed since the note above and neither
# moves the number:
#   (1) the radial fight is over. Every arm seats on the DISC, and the vendor
#       solid puts the disc faces 3.10 (horn) / 2.05 (idler) PROUD of their
#       case faces, so the PAD PLATE passes UNDER a 1.65-proud case-screw head
#       with 1.45 mm (horn: plate 20.45 vs head top 19.00) and 1.60 (idler:
#       plate -18.00 vs head top -16.40) to spare. No pad radius reaches it.
#       Only the SEATING BOSS shares the head's axial band, and IDLER_BOSS_D
#       stays 20.0, inside the head's 10.34 reach.
#   (2) so the binding constraint is now the HEAD LAND: bolt circle r 7 against
#       a 10.0 rim leaves 3.0 mm, i.e. O6.0. The M3x6 horn buttons (O5.7) bear
#       fully with 0.15 to spare. The idler M3x8 + THIN WASHER (O7.0) overhangs
#       the rim by 0.5 all round -- but a full annulus centred on its own screw
#       cannot tip on a 0.5 mm overhang, and it still bears on 6.5 of its 7.0.
#       Tolerated, not a fit failure.
# Growing to 21.5 (O7.5 land) was tried and reverted: r 10.75 breaks BOTH the
# carrier wall-screw head reach (10.69, checked in check_assembly) and the
# foot's FOOT_ROTOR_CLEAR_R arc at the ankle (4.85 mm3, three ROM extremes).
# The pad is boxed in on all sides; buy nothing for 0.5 mm of washer rim.
# Drawn to scale in docs/assembly/wheel-end-pad.svg.
PAD_D = 20.0
# Centre reliefs, sized to the MEASURED screw heads (user, 2026-07-27) rather
# than left generously round. Shrinking them is a strength change: the web out
# to the O14 disc bolt circle is what carries the joint load, and every mm of
# relief radius comes straight off it.
#   horn (powered) side head 5.5 -> 6.2 relief, web 1.30 -> 2.20 mm
#   idler side      head 6.8 -> 7.5 relief, web 0.90 -> 1.55 mm
# 0.7 of clearance on each: these are non-contact reliefs, and FDM bores come
# in slightly undersize. The idler figure must ALSO clear the servo's O6.1
# free-hub post (7.5 does, by 1.4) -- the head is the binding constraint.
HORN_CENTER_RELIEF_D = 6.2
IDLER_CENTER_RELIEF_D = 7.5
                        # stays printable: gap = 7 - relief_r - 1.7. At the
                        # old O9/O10 it was 0.8/0.3 mm (flakes); O8 gives 1.3.
                        # Center screw head is ~O5.7 recessed -> 1.1 mm slack.
                        # VERIFY on the real horn/idler disc before final print.

# case mounting hole rows (distance behind the output axis; lateral +/-10.25)
CASE_HOLES_TOP = (8.30, 29.00)     # horn-side face
CASE_HOLES_BOT = (8.30, 32.75)     # idler-side face
CASE_HOLE_LAT = 10.25
# Case-mount screws -- BENCH TRUTH 2026-07-28: the ST3215 case holes take
# M2.5, NOT the M3 the community wiki suggested ("3mm is too wide of a
# screw" -- user, after driving them). Sourced: uxcell M2.5x8 pan-head
# Phillips self-tapping, stainless (Amazon B01KXTTSCI, 50 pcs).
# Head treatment splits by where the head lives:
#   PAN, proud       -- free space beside the head (foot tabs, carrier walls)
#   PAN, counterbored-- thick material available (pelvis deck: battery sits
#                       on the deck top, so the stator heads sink sub-flush)
#   FLAT, FLUSH (90 deg countersink) -- the leg_link GRIP plates. A pan head
#     there is geometrically unrecoverable: the yoke/fork arm of the joint
#     above sweeps 0.70 mm off the horn-plate face (measured overlap up to
#     25.5 mm3 from hip +/-60 deg on -- the bench "link skew"), and sinking a
#     2.0 mm pan head under a 0.5 buffer leaves a <0.8 mm web (unprintable).
#     A flat head sits flush: 0.70 mm running buffer, >=1.15 mm wall left.
#     HARDWARE: 24x M2.5x8 FLAT-head self-tappers to buy (not yet on hand).
CASE_SCREW_PILOT = 2.05            # M2.5 self-tap pilot in printed plastic
CASE_SCREW_CLEAR = 2.9             # where the screw just passes through
CASE_HEAD_D = 5.0                  # pan head dia (uxcell class)
CASE_HEAD_H = 2.0                  # pan head height, DESIGN MAX -- measure;
                                   # DIN 7985 M2.5 is 1.80, ISO 7045 ~1.95
CASE_FLAT_D = 4.7                  # M2.5 flat (countersunk) head dia, 90 deg
CASE_CS_D = 5.4                    # countersink mouth: flat head 4.7 + 0.7
CASE_CS_DEPTH = (CASE_CS_D - CASE_SCREW_CLEAR) / 2   # 1.25 (90 deg cone)
DECK_CB_D = CASE_HEAD_D + 0.8      # 5.8 deck-top counterbore over pan heads
DECK_CB_DEPTH = CASE_HEAD_H + 0.3  # 2.3: head 0.3 sub-flush under the battery
M3_ST_PILOT = 2.8                  # M3 self-tap pilot (tower gopro bosses)
# other fastener heads (for the head-clearance audit + STEP fastener models)
M3_HEAD_D, M3_HEAD_H = 5.7, 1.65   # M3 button head (KADRICK kit + servo M3x6)
M3_WASHER_D, M3_WASHER_T = 7.0, 0.5  # thin washer under the idler M3x8
M25_HEAD_D, M25_HEAD_H = 4.5, 1.7  # M2.5 machine pan (driver board / IMU)

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
# IDLER_ARM_INNER holds the arm plate off the disc, and the boss above spans
# that gap. That gap is NOT slack and must not be collapsed to "narrow the
# fork": it is what sets SCREW ENGAGEMENT. The screw has DISC_SCREW_THREAD of
# thread and the tapped hole bottoms out at DISC_THREAD, so
#     engagement = DISC_SCREW_THREAD - (boss + PLATE)
# has to land between DISC_THREAD_MIN_ENGAGE and DISC_THREAD.
# 1.20 -> 0.60 on 2026-07-30 (user: "the thickness of the fork + idle wheel is
# 6.25 mm, the length of the screw thread is 5.6, so we could shrink that
# dimension by ~.75"). At 1.20 the stack was 4.20 and the screw only reached
# 1.40 mm into a 2.1 mm hole -- under-engaged. At 0.60 the stack is 3.60 and it
# grips 2.00 mm, i.e. effectively the whole flange, with 0.10 clear of the
# bottom. Do NOT keep shrinking: 0.45 already bottoms out, and collapsing the
# boss to zero drives the tip 0.50 mm into the bottom of the hole and jacks the
# joint apart instead of clamping it.
IDLER_ARM_INNER = -17.40                   # arm plate inner face (from servo mid)
IDLER_BOSS_H = IDLER_ARM_INNER - SV_IDLER_FACE  # 0.6

# --- DISC TAPPING, bench-measured (user, 2026-07-30) and corroborated by
# sectioning cad/vendor/ST3215.step by radius. The idler disc is NOT a flat
# 3.35 slab: it is a thick central hub inside a thin outer flange.
#     hub    r <= 4.5   3.1 mm (bench) / 3.23-3.35 (STEP)
#     flange r >= 4.5   2.1 mm (bench) / 2.20 (STEP)
# The O14 bolt circle sits at r 7.00 -- out in the FLANGE. So every disc screw
# has 2.1 mm of thread to work with, not 3.35, and screw length is chosen
# against DISC_THREAD, never against the disc's overall thickness.
DISC_THREAD = 2.1
DISC_THREAD_MIN_ENGAGE = 1.5    # below this there is not enough thread to hold
DISC_SCREW_THREAD = 5.6         # threaded length of the disc screw (bench,
                                # user 2026-07-30). This -- not the nominal
                                # "M3x6" -- is what reaches into the hole.
# The HORN disc is built the same way -- hub 4.50 to r 5.5, then a flange that
# the bolt circle taps into. BENCH-CONFIRMED 2026-07-30 (user): "the horn wheel
# is 2.5mm thick", matching the STEP's 2.50 exactly (unlike the idler, where the
# STEP ran 0.1 proud of the calipers).
HORN_THREAD = 2.5

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
# NOTE 2026-07-24: the Waveshare vendor STEP shows the idler disc 0.27 mm PROUD
# of the slab plane (not 0.55 recessed) -- but the 0.55-recessed constants are
# validated by the assembled 8-DOF robot (idler bosses reach 1.2 mm IN and the
# joints close), so the STEP's disc is likely modeled floating off its seat.
# Keeping the validated arithmetic; the deck's disc/hub pocket (1.3 deep)
# clears the disc + free-hub post under EITHER reading. Measure the real disc
# stand-off with calipers at yaw assembly; if it is proud, the stack shortens
# ~0.8 mm (harmless -- the carrier hangs on the horn).
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
# Screw-head clearance inside the bay (user, 2026-07-27). The 4x yaw-horn
# bolts pass UP through the mount plate into the horn disc, so their HEADS sit
# proud of the bay ceiling -- straight into the roll servo's rear face, which
# was flush against that ceiling with a 0.00 mm gap. Measured clash 168 mm3.
# 2.0 rather than the 1.5 first suggested: the modelled M3 button head is
# 1.65 mm tall, so 1.5 would still touch by 0.15. If the servo's own bundled
# horn screws turn out to have a lower head, this is the one line to change.
CARRIER_ROLL_HEAD_CLEAR = 2.0
CARRIER_ROLL_AXIS = (CARRIER_ROLL_CEIL - CARRIER_ROLL_HEAD_CLEAR
                     - SV_AXIS_FROM_REAR)              # -40.11 roll axis

# --- yaw stack drop (the honest number; the study estimated ~25-30 mm) ---
# The hip-roll axis moves from (DECK_T + SV_AXIS_FROM_REAR) below the deck top
# to (yaw grip span + carrier plate + recess) deeper.  The drop is the gap
# between the old and new roll-axis planes, all arithmetic:
ROLL_BELOW_DECK_OLD = DECK_T + SV_AXIS_FROM_REAR            # 40.11 (8-DOF build)
ROLL_BELOW_DECK_YAW = -(YAW_HORN_FACE_Z + CARRIER_ROLL_AXIS)  # 80.91 (v3yaw)
YAW_STACK_DROP = ROLL_BELOW_DECK_YAW - ROLL_BELOW_DECK_OLD  # 40.80 mm

YAW_SWEEP = 45.0                   # design yaw half-range each way (deg)

# --- STS3215 idler-face ground truth (MEASURED 2026-07-24) -------------------
# Sources, in agreement: the user's physical servo (photo IMG_5698), the
# Waveshare vendor STEP (files.waveshare.com upload 5/59 ST3215-3D.zip,
# z-buffer measured), and the vendor 2D drawing (0/08 ST3215-2D.zip).  All
# positions are on the IDLER-side case face, axis-relative, + toward the CABLE
# end; heights are relative to the SLAB PLANE = the outermost flat of the case
# (the raised rectangle between trench and cable end) = what SV_TOPFACE/
# SV_CASE_T already measure, i.e. the surface a mount actually touches.
#   feature                       length band     width      height vs slab
#   idler disc (ROTATES!)         O19.2 at axis   --         +0.27 PROUD
#   idler hub screws (ROTATE!)    ~O8   at axis   --         +0.82 PROUD
#   connector TRENCH (2 sockets   11.75..16.35    +/-10.9    floor -4.78
#     side by side ACROSS the                                (sockets open
#     width, opening out of the                               outward)
#     face, in the trench)
#   cover slab (the datum)        19.6..29.5      +/-9       0
#   stator screw bosses           rows 8.30/32.75 +/-10.25   -1.78 (recessed)
# CONSEQUENCES: (1) any mount that presses this face flat needs a CLEARANCE
# POCKET over the disc+hub (they rotate -- clamping them binds the joint) and
# lands on the slab + 1.78 mm pads at the screw bosses; (2) connector openings
# belong over the TRENCH band 11.75..16.35, nearly full case width -- not the
# earlier guessed 11..25 x +/-7 band, and never the cable-END face.
SV_IDLER_DISC_PROUD = 0.27  # disc face above the slab plane PER VENDOR STEP --
                            # contradicts the validated SV_IDLER_FACE (0.55
                            # recessed); see the YAW_IDLER_DISC_Z note. The
                            # free-hub post protrusion (+0.82 in the STEP) is
                            # corroborated by the user's photo either way.
SV_IDLER_HUB_PROUD = 0.82   # hub/disc screws above the slab plane (rotate)
SV_IDLER_BOSS_RECESS = 1.78 # stator screw bosses below the slab plane (STEP);
                            # seat pads use 1.5 -- under-reach is harmless,
                            # over-reach would tip the case. Measure on a servo.
SEAT_PAD_H = 1.5            # pelvis yaw-seat stator pad height (see above)
SV_CONN_L = (11.75, 16.35)  # trench band along the length, axis->cable-end
SV_CONN_HW = 10.9           # trench half-width (sockets span most of it)
SV_CONN_FLOOR = 4.78        # trench floor below the slab plane

# --- STS3215 features MEASURED OFF cad/vendor/ST3215.step (2026-07-28) --------
# Booleaned in FreeCAD's OCC against the vendor solid aligned to the mock frame
# (output axis at origin, axis +Y, horn +Y); build123d mis-handles the
# transformed vendor compound and cannot be trusted for this.  User confirmed
# the horn-side structure is real: "the vendor .step is correct".
#
# HORN-SIDE RIB: a raised boss on the CABLE half of the horn-side case face.
# Anything that clamps this face flat rides up on it unless it is relieved --
# it is the reason leg_link overlaps the real servo by ~299 mm3.
SV_HORN_RIB_HW = 7.42       # half-width across the case (x); +/-7.09 for the
                            # first 0.75 mm, widening to +/-7.42 near the top
SV_HORN_RIB_L = (8.73, 34.26)   # band along the length, axis -> CABLE end
SV_HORN_RIB_H = 1.13        # proud of SV_TOPFACE (top at +18.48)
#
# IDLER HUB: the free-hub post inside the O25 recess. It ROTATES with the joint,
# so a mount must never touch it.  Reaches exactly flush with SV_BOTFACE, i.e.
# 0.55 proud of the idler disc face -- NOT the 0.82 that SV_IDLER_HUB_PROUD
# claims relative to a different datum.
SV_IDLER_HUB_HW = 3.24      # half-width; fills SV_BOTFACE .. SV_IDLER_FACE
#
# IDLER-SIDE SEATING -- measured off the same vendor solid, 2026-07-29, after
# the user flagged that "the idler side does not conform to the servo the way
# the horn side does".  It does not, and the numbers are stark.
#
# SV_BOTFACE (-17.35) IS NOT A REAL SURFACE.  SV_CASE_T assumes the case is
# symmetric about the output axis; it is not.  Slab-sectioning the vendor solid
# over the grip plate's own footprint gives ZERO mm2 of material at -17.35, and
# still zero at -16.80.  The only thing that reaches -17.35 anywhere is the
# free-hub post, which ROTATES with the joint and must never be touched.
# Measured standoff of each grip plate from the first real material it could
# bear on, same footprint bands either side:
#     horn plate  seats at +17.35, material starts +17.20   -> 0.15 mm
#     idler plate seats at -17.35, material starts -14.75   -> 2.60 mm
# So the two idler grip screws (row 32.75) have been clamping an air gap.
SV_IDLER_CASE_FACE = -14.75   # the real idler-side case face; the screws land
                              # here, so this is what the plate must bear on
# A moulded platform stands proud of that face over most of the plate footprint
# (label/back-cover area).  It sits BETWEEN the screws and the plate, so it has
# to be relieved rather than seated on -- exactly as the horn-side rib is.
SV_IDLER_BOSS_Y = -16.65            # crest, 1.90 proud of the case face
SV_IDLER_BOSS_HW = 10.70            # half-width across the case (x)
SV_IDLER_BOSS_Z = (-29.51, -16.25)  # band along the length (axis -> cable end)
GRIP_SEAT_CLR = 0.15          # designed plate standoff from a seat face. Set to
                              # the horn side's MEASURED 0.15 so both plates land
                              # the same way instead of one of them by luck.
#
# The vendor solid has NO O19.6 output boss: on-axis is EMPTY from SV_TOPFACE up
# to +18.35, where the O19.2 horn disc starts and runs to SV_HORN_FACE. The
# mocks keep a solid SV_BOSS_D column through that gap, which is conservative.
WIRE_BUNDLE = 5.0           # one 3-wire ST3215 lead bundle, outer dia
WIRE_PLUG_W = 9.0           # ST3215 JST housing width (assemble connector-first)

# ----------------------------------------------------------------------------
# leg link (thigh and shin are the SAME part)
#   grips the upper servo's case below its horn (which is the upper joint) and
#   forks down to the next servo's horn+idler, DROP = 90 between axes.
# ----------------------------------------------------------------------------
LINK_DROP = 90.0        # == THIGH == SHIN
GRIP_PLATE_T = 2.4
GRIP_PLATE_T_IDLER = 3.0    # idler grip plate, measured off its SEAT face
                        # (idler_seat, -14.90). 2026-07-30, user: the seat
                        # move had left the old -21 outer face in place, so
                        # the wall grew to 6.1 mm and the grip screws needed
                        # M2.5x10 -- pull the outer face in so M2.5x8 works
                        # everywhere. Not GRIP_PLATE_T (2.4): the platform
                        # relief pocket is 2.05 deep and must keep ~0.95 mm
                        # of skin behind it, the same class as the 0.97 the
                        # horn rib pocket leaves.
GRIP_TOP_HORN = -3.0    # top edge of horn-side grip plate (below upper axis);
GRIP_HORN_RELIEF = 10.3 # ...with a circular relief around the O19.6 case boss
# Rib detent in the horn-side grip plate (see leg_link). Depth clearance is what
# guarantees the plate lands on the CASE, not on the rib; 0.30 over a 1.13 rib
# leaves GRIP_PLATE_T - 1.43 = 0.97 mm of plate over the pocket, which spans
# between the two screw rails rather than carrying clamp load.
RIB_RELIEF_CLR = 0.4          # footprint clearance each side of the rib
RIB_RELIEF_DEPTH_CLR = 0.3    # air above the rib crest
# Toe corner radius (plan view). Square toe corners are the ones that snag on
# door frames / cable runs and they read as blocky; 14 leaves a 24 mm straight
# front edge on the 52 mm-wide sole and takes only ~2 % of the sole area.
# Break-away print-support wall thickness (leg_link_print pad stubs + island
# posts). 2.4 -> 1.2 on 2026-07-28: at full plate thickness they read as part of
# the model in the slicer and take pliers to remove. 1.2 is 3 perimeters at a
# 0.4 nozzle -- prints solid, snaps with a fingernail, and stays well clear of
# check_printability's 0.85 mm thin-wall flag.
FIN_T = 1.2
FOOT_TOE_R = 14.0
# Heel rounded to match (user, 2026-07-28). The aft buttress sits at x -42..-40,
# 10 mm clear of the heel edge, and the cable window (y +/-8) stays inside the
# straight centre section, so nothing structural rides on the removed corners.
FOOT_HEEL_R = 14.0
# Driver-access divots (user, 2026-07-28): a round channel CO-AXIAL WITH THE
# SCREW -- bored along the screw axis, not scalloped down from the sole edge --
# so the relief follows the driver instead of hacking a notch out of the plan
# silhouette. Sized to the head plus margin and nothing more: the countersink
# mouth is CASE_CS_D, so 2.70 + 0.60 = 3.30. That leaves 2.81 mm of sole under
# the channel, MORE than the 2.0 the old rectangular notch left.
FOOT_DIVOT_R = CASE_CS_D / 2 + 0.6
# Front (rotor-adjacent) retention bosses. The servo case's 8.30 hole row sits
# right behind the output end; the foot only ever used the far rows, leaving the
# case cantilevered off the heel tabs. These tie it down at the LOW lateral hole
# (z 6.11), close to the sole, which is where case-to-sole flex shows up.
FOOT_FRONT_BOSS_X = 8.30      # case row, behind the axis (== CASE_HOLES_TOP[0])
FOOT_FRONT_BOSS_HW = 4.5      # boss half-length along x
FOOT_FRONT_BOSS_TOP = 9.6     # boss top. Needs >= 9.11 to carry the csk mouth
                              # over the z 6.11 screw; the extra 0.4 mm at 10.0
                              # was the PINCH POINT against the shin at ankle
                              # -40 (0.108 mm). At 9.6 the corner drops out of
                              # the sweep and both extremes sit at 0.300 mm.
FOOT_ROTOR_CLEAR_R = 10.3     # keep the boss off the O19.2 horn disc (r 9.6).
                              # 10.0 left the countersink mouth (r 3.00, its
                              # centre 13.19 from the ankle axis) clearing the
                              # arc by 0.19 mm -- an unprintable ligament that
                              # reads as a join error. 10.3 merges the two so
                              # the mouth opens cleanly into the relief.
FOOT_CABLE_TOP_Z = 16.0       # cable window TOP. Left open to the bulkhead top
                              # it made the heel an open-topped U; capped here
                              # the section closes into a ring. 16.0 - 8.0 = 8.0
                              # mm of opening against a 5.0 mm lead bundle and a
                              # 9.0 mm plug housing, so the connector still passes.
FOOT_PAD_RELIEF_Z = 3.5       # floor of the fork-pad relief slots. The front
                              # bosses MUST reach down to this, not to the
                              # pocket floor: the slot has already taken the
                              # sole away above it, and a boss starting at 4.0
                              # floated free (81 mm3 detached solid).
# leg_link lower-joint corner cuts (user sketch, 2026-07-28). Measured about the
# lower axis, the fork profile steps from the PAD_D/2 = 10.0 pad straight out to
# r 12.0 at 0 deg (front edge) and r 15.0 at 180 deg (rear web face) -- two sharp
# corners per plate, four in all, and the first thing to swing into the mating
# part. Each is chamfered from the pad tangent out to the full plate width. Both
# cuts lie entirely OUTSIDE r 10, so the pad and its O14 bolt circle are
# untouched; the heights set how gradual the blend is.
LEG_CORNER_CUT_FRONT_H = 6.0
LEG_CORNER_CUT_REAR_H = 10.0
GRIP_TOP_IDLER = -16.0  # idler side must clear the upper yoke arm sweep (R12+4)
GRIP_BOT = -36.0        # just past the case bottom end (-35.11)
# (An idler-side cable-notch constant set lived here 2026-07-23 and was removed
# the same day -- the sweep analysis showed no ankle-cable interference with the
# fork, ~10 mm clearance at toes-pointed; see parts.py leg_link note.)
WEB_GAP = 0.4
WEB_TOP = -16.0         # clears the upper joint's fork arms folding to 95 deg
BRACE_Z = (-54.0, -50.0)  # cross brace between the fork tines, in the X-Y
                        # plane (2026-07-29). Below the cable window (-48..-37)
                        # so it cannot foul the servo lead, and rooted on the
                        # web (full depth to the back plate) so it prints as a
                        # wall, not a 38 mm tine-to-tine bridge.
                        # Pull it UP if the next servo's sweep about the lower
                        # axis wants the room; check_assembly is the arbiter.
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
# CAVEAT (2026-07-24, SV_CONN correction): this window assumes the cable exits
# the rear END face, but the STS3215's ports are actually on the IDLER-side
# face beside the disc (see the SV_CONN note). The foot is a shipped print and
# was never validated against a real servo -- re-audit whether the ankle lead
# reaches this window when servos arrive, before any reprint.
FOOT_CABLE_W = 16.0     # cable window in the bulkhead, open at the top
FOOT_CABLE_Z = 8.0
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
# 2026-07-28 -- RESIZED for the General Driver board, which is 65 x 65 and
# cannot lie down anywhere in the old torso (see BOARD_GD_* below). It is
# mounted VERTICALLY against the +x wall instead, with the pack beside it
# behind a partition. Decision: user, this session. Both numbers below are
# derived, not chosen:
#   TOWER_W: preload 1.4 + pack 31 + partition 2.6 + standoff 4 + PCB 1.63
#            + components 9 (the 40-pin header is the tall one) = 49.63
#            interior, + 2 walls = 54.8 -> 56.0.
#   TOWER_H: the GoPro bosses hang in the top 3 mm under the plate, so the
#            board must clear TOWER_H - TOWER_TOP_T - 3. Board bottom 2.5 +
#            65 = 67.5, so TOWER_H >= 74; 75 leaves a 1 mm gap.
# Consequences accepted with the decision: the tower now OVERHANGS the deck by
# 5 mm per side fore-aft (the feet tabs still land inside it), and the GoPro
# rides 31.5 mm higher, which raises torso COM. Both feed the sim plant --
# sim/build_v2_inertia.py has to be re-run and the policy re-trained.
TOWER_W = 56.0          # was 42.0
TOWER_H = 75.0          # was 43.5
TOWER_TOP_T = 3.5
# driver board: Waveshare "Servo Driver with ESP32", 65 x 30, holes O2.75 on
# a 58 x 23 grid. VERIFIED 2026-07-27 by test-fit: the real board dropped onto
# the printed tower and all four screws landed -- which checks both spans and
# the hole diameter at once, more tightly than calipers would.
BOARD_HOLES = (58.0, 23.0)     # hole pattern (y span, x span), M2.5 self-tap
BOARD_STANDOFF = 6.0           # under-plate standoff height (clears the GoPro
                               # screw bosses by 3 mm; battery below gets ~2 mm)
# REPLACEMENT BOARD -- Waveshare "General Driver for Robots", the swap that buys
# an onboard IMU with no soldering (docs/wiring-general-driver.svg, schematic in
# docs/datasheets/general-driver/). Vendor figures, 2026-07-28.
#
# WHY IT IS MOUNTED VERTICALLY: it is 65 x 65 with a 49 x 58 hole grid. The old
# tower was 42 fore-aft (36.8 between walls) and the deck is 46, so even the
# NARROWER 49 span put its screws 3.5 mm outboard of the tower's own outer wall.
# No orientation lay it down anywhere on the torso. Standing it against the +x
# wall works because the wall's OTHER axis (y, 96) has room to spare -- it just
# needed the tower to grow tall enough, hence TOWER_H above.
BOARD_GD_OUTLINE = (65.0, 65.0)  # square, vs the 65 x 30 of the current board
BOARD_GD_HOLES = (58.0, 49.0)    # AS SUPPLIED (y span, x span). Mounted upright
                                 # the 58 stays in y and the 49 becomes vertical.
BOARD_GD_HOLE_D = 3.0            # the board's own holes (M2.5 clears with slack)
# Vertical mount, all x measured in the pelvis frame (tower interior is +/-25.4):
BOARD_GD_PARTITION_X = 7.0       # -x face of the partition that seats the pack
                                 # and carries the board. Replaces the old +x
                                 # rail stubs, which the board now occupies.
BOARD_GD_STANDOFF = 4.0          # partition +x face .. PCB -x face
BOARD_GD_PCB_X = BOARD_GD_PARTITION_X + WALL + BOARD_GD_STANDOFF   # 13.6
BOARD_GD_COMP = 9.0              # component reach off the PCB's +x face; the
                                 # 40-pin header is the tall one. Unused by us,
                                 # but it is fitted, so it sets the envelope.
# Board centre height above deck top. Bottom lands at 2.5 (clear of the deck),
# top at 67.5, which clears the GoPro bosses hanging at TOWER_H - 6.5 = 68.5.
BOARD_GD_CZ = 35.0
BOARD_GD_SCREW_DY = BOARD_GD_HOLES[0] / 2        # +/-29.0, unchanged from the
                                                 # old board -- the one bit of luck
BOARD_GD_SCREW_DZ = BOARD_GD_HOLES[1] / 2        # +/-24.5 about BOARD_GD_CZ
BOARD_GD_MASS = 42.0             # g, estimated -- WEIGH IT when it arrives; it
                                 # feeds the torso rollup and therefore the plant
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
BATT_SEAT_X = -24.0            # pack outer (-x) face when seated: 1.4 inside
                               # the wall inner face, so the strap can preload.
                               # Was -17.0; followed TOWER_W 42 -> 56 so the
                               # pack still rides against the -x window wall
                               # and the freed +x space goes to the board.

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

# IMU: GY-BNO08X breakout (Amazon B0CL26J81F), photographed 2026-07-27.
# BNO080/085 family -> SH-2, which is what the firmware already targets.
#
# It is NOT the board the old carrier was built for. The retired BNO055
# breakout was 26.67 x 20.32 with FOUR mounting holes on a 21.59 x 15.24
# rectangle; this one is smaller, thinner, and has only TWO holes, both
# hard against the castellated edge.
#
# We deliberately do NOT model those two holes. The only photo available is
# at an angle, so hole positions read out of it carry ~+/-1 mm -- far worse
# than the ~0.3 mm a screw boss needs, and a boss designed from a bad number
# is a part that does not fit. The outline, by contrast, is trustworthy from
# two independent checks: 10 pads at 2.54 mm pitch span 22.86 mm plus edge
# margin ~= 25.4, and the photo's aspect ratio 0.60 matches 15.5/25.4 = 0.61.
#
# So the carrier LOCATES ON THE OUTLINE: a pocket constrains x, y and
# rotation using the dimension we trust, and snap tabs retain it. For an IMU
# that is the better mount regardless -- repeatable seating matters more than
# screw count, because any shift corrupts the gravity vector the policy reads.
IMU_PCB = (15.5, 25.4, 1.63)   # outline x (across pads), y (along pads), thickness
                               # 1.63 MEASURED 2026-07-27 (vendor spec said 0.8)
IMU_POCKET_CLEAR = 0.35        # per-side clearance: FDM walls come in ~0.2 proud
IMU_POCKET_D = 1.85            # pocket depth = pcb + 0.22, so screws clamp the
                               # board, not the pocket rim

# Screw pattern, recovered from the 2026-07-27 photo by perspective-correcting
# it: the board outline is known (15.5 x 25.4), so fitting its four edges gives
# a homography that maps pixels to millimetres and removes the camera angle
# entirely. Two independent corner estimates (extreme points vs fitted edges)
# agreed to ~0.25 mm:
#     x from pad edge   13.04 / 12.90     -> 12.90
#     y spacing         20.92 / 20.67     -> 20.80
#     hole diameter      3.24 /  3.15     ->  3.15
# Treated as symmetric in y (the 0.25 mm offset the measurement showed is
# inside its own error, and a manufactured board will be symmetric).
IMU_SCREW_X = 12.90 - 15.5 / 2  # +5.15 from board centre, toward castellations
IMU_SCREW_DY = 20.80 / 2        # +/-10.40 from board centre
IMU_SCREW_HOLE_D = 3.15         # the board's own hole -- 0.3 mm radial slack on
                                # an M2.5, which is exactly the point: the
                                # POCKET locates, the SCREWS only clamp, so a
                                # 0.25 mm error in this pattern is absorbed.

IMU_TONGUE_T = 5.5             # tongue is thicker than the 3.0 pad: the pocket
                               # floor must still be thick enough to tap. Grows
                               # UPWARD so the part still prints flat, pocket up.
IMU_SOLDER_SLOT = 3.4          # through-slot under the pad row: clears soldered
                               # pin tails on the underside AND is the wire exit.
                               # Runs the full length and out the rear -- a slot
                               # stopping short would foul the tails.
IMU_CY = -27.5                 # IMU center y on the tongue (x centered). Moved
                               # back from -25.0 on 2026-07-27: the tongue is
                               # now raised to IMU_TONGUE_T and the pocket has
                               # to sit entirely behind gopro_base's footprint
                               # (y >= -12), which the old centre did not.
                               # x half-width 9.7 still clears the feet-screw
                               # wells at (+-14, +-42).
IMU_CARRIER_T = 3.0            # carrier PAD thickness (the M3x12 stack)

# PCB underside height above the carrier's z=0 plane.
IMU_PCB_Z = IMU_TONGUE_T - IMU_POCKET_D

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

# ----------------------------------------------------------------------------
# joint ranges of motion, deg (SOURCE: sim/bimo_biped_v3yaw.xml joint ranges --
# the plant policy trains against these, so CAD must clear them). Interference
# checks probe BOTH extremes of every joint and require SWEEP_BUFFER of air
# between relatively-moving bodies (user call 2026-07-28).
ROM = {
    "hip_yaw": (-45.0, 45.0),
    "hip_roll": (-25.0, 25.0),
    "hip_pitch": (-110.0, 60.0),
    "knee": (-95.0, 5.0),
    "ankle": (-40.0, 40.0),
}
SWEEP_BUFFER = 0.5      # mm of clearance required at the ROM extremes

BED = 220.0             # print bed (square)
