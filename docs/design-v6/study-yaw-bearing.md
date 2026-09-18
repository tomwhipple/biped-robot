# Hip-yaw bearing study (2026-09-17, round 2 2026-09-18)

Tom, 2026-09-17: *"What about the hip yaw? The servo axle is the only
connection between the leg and the rest of the robot ... it seems to me
we're fighting a lot of mechanical leverage there."* Confirmed in the CAD
(`cad/parts.py` `yaw_carrier`, `cad/v6/pelvis_v7.py`): every other joint in
v6 is closed horn + idler (both faces gripped, load shared); the hip yaw is
a cantilever -- the STS3215 sits in a cell under the pelvis deck, case fixed
to the ceiling, horn down, and the carrier (the WHOLE leg, a 0.335 m lever)
bolts to the horn alone. Symptoms on v5: 1-2 deg of measured yaw play, horn
screws working loose, heading shortfall in turns (v5 build log).

**Round 2, 2026-09-18:** round 1 ended on a 6811-2RS wrapped around the
carrier's WHOLE rectangular plate+wall footprint (kept below as **Option
C**, the baseline). Tom's review: get the CAD right for a real selection,
not a rectangle-wrapped compromise -- specifically (1) **the hip stack must
not get taller** (no change to `HIP_YAW_Z`, `CARRIER_ROLL_AXIS`, `d_yaw_roll`
anywhere), and (2) **make the yaw joint round**. This round lays out three
options on that footing (**A**: a round hub at the carrier's existing
height; **B**: raise the yaw servo into the housing; **C**: the baseline)
plus a fourth considered on paper only (**D**), builds out A fully, and
recommends it. Selection is left to Tom -- BOM not touched.

## 1. Load path, before and after

**Before:** body weight (thrust, ~15 N of the ~1.5 kg robot's mass in
single support) + the single-support roll moment (~0.65 N-m,
`docs/design-v6-ankle-roll.md` §4.4) + any side load at the foot x 0.335 m
lever all go through: the servo's internal output bearing -> the horn
spline -> 4x M3 horn screws -> the carrier's flat plate. The plate mates the
horn by friction/screw clamp only; nothing pelvis-side reacts a moment
except through that same shaft.

**After (both A and C):** the SAME 4 horn screws stay (torque only -- they
still turn the leg). Thrust and moment now go: carrier hub/plate + the bay
walls under it -> a boss (round, in both options, on the carrier's own
axis) -> a deep-groove ball bearing's LOCATED inner and outer races -> a
recess in the pelvis housing -> the housing's own print material. The
servo's output shaft carries torque only, exactly like every other joint's
horn already does relative to its own idler-side grip. The two options
differ only in how big the boss/recess have to be and where, in the SAME
fixed axial envelope, they sit -- see §3.

## 2. What's actually inside the envelope (measured, both options work from this)

Everything below is from `cad/dimensions.py`, `cad/v6/dimensions_v6.py`, and
a live geometry measurement this session (`cad/check_assembly.py
servo_mock_x()`, cited where used) -- not estimates.

| feature | value | source |
|---|---|---|
| horn screws | 4x M3, bolt circle r 7, self-tap head ~1.7 mm -> 8.7 mm reach | `D.BCD`, `D.PAD_HOLE` |
| carrier horn-mount plate footprint (z 0..-3) | 39.9 x 30.52 mm, corner 25.12 mm | `parts.yaw_carrier()`, round-1 write-up |
| carrier's bay walls share that SAME 25.12 mm corner for their whole height (z -3..-44) | -- | `parts.yaw_carrier()` |
| yaw axis -> roll axis (`d_yaw_roll`, `CARRIER_ROLL_AXIS`) | 40.11 mm, **UNCHANGED by every option below** | `D.CARRIER_ROLL_AXIS` |
| roll servo's own cable-end (top) position, carrier-local | -5.00 mm (`CARRIER_ROLL_AXIS + SV_AXIS_FROM_REAR`) | `dimensions_v6.py` `YAWA_ROLL_CASE_TOP_Z` |
| roll servo's own worst corner reach from the yaw axis, at its cable-end top (incl. the horn-side rib) | **22.23 mm**, MEASURED (`CA.servo_mock_x()` sliced at carrier-local z in [-10.11,-5.00] this session) | see `yaw_carrier_v6_optA.py` docstring |
| roll-servo retention screw rows, carrier-local | -11.11 mm and -7.36 mm | `D.CARRIER_ROLL_AXIS + D.CASE_HOLES_TOP[1]/BOT[1]` |
| yaw servo/horn-disc's OWN corner reach (the round-1 mistake's basis) | 15.97 mm -- too small, it is not what is actually in the band (the ROLL servo is bulkier and does reach into it) | `_case_corner_r`, `dimensions_v6.py` |
| cell ceiling -> battery bottom (clearance below the battery) | 0.5 mm | `dimensions_v6.py` new `YAWB_CLEAR_BELOW_BATT` |
| battery top -> deck bottom (clearance above the battery) | 4.5 mm | `dimensions_v6.py` new `YAWB_CLEAR_ABOVE_BATT` |

**The round-1 mistake, corrected:** a round hub at the horn face only has to
clear the small yaw-servo horn disc (15.97 mm) if the band stays inside the
old 3 mm plate. Any bearing width from the offered classes (45/50/55 mm
bore, all 5-9 mm wide) is deeper than that 3 mm, so the band always reaches
into the roll-servo bay's top -- and what is there is the BULKIER roll
servo's own case (22.23 mm), not the small horn disc. That is the number
every hub in this round has to clear.

## 3. Option A: round hub, at the carrier's existing height (RECOMMENDED)

**No axial growth.** The bearing band is fixed, not new length: it starts
at the horn face (carrier z = 0) and runs down the bearing's own width --
`V.YAWA_BAND_Z = (-7.0, 0.0)`, entirely inside the carrier's EXISTING
envelope (z 0..-44 unchanged). `HIP_YAW_Z`, `CARRIER_ROLL_AXIS`,
`d_yaw_roll`, `DECK_BOT_Z`/`DECK_TOP_Z` are byte-identical to before this
option (verified: `cad/v6/dimensions_v6.py` adds only new `YAWA_*`
constants, touches nothing existing).

**Bearing pick.** Bore radius must clear the roll servo's own worst corner
(22.23 mm, §2) by >= 1.5 mm -> radius >= 23.73 mm -> bore >= 47.46 mm.
Of the three offered classes:

| bore class | bearing | OD x W | boss radius | wall vs 22.23 mm | verdict |
|---|---|---|---|---|---|
| 45 mm | 6709-2RS / 6809-2RS | -- | 22.54 mm | **0.31 mm** | **FAILS** -- not printable |
| 50 mm | 6710-2RS (50x62x6) / **6810-2RS (50x65x7)** | 65x7 | 25.04 mm | **2.81 mm** | clears, smallest that does |
| 55 mm | 6711-2RS / 6811-2RS | 72x9 | 27.54 mm | 5.31 mm | clears, oversized here |

**Pick: 6810-2RS (50x65x7 mm).** C0 = 5.8 kN (one full spec sheet this
session; a second source's 6.6 kN is a DYNAMIC figure, not directly
comparable, not cross-checked against a second static source). Mass 52 g
(single source, not independently re-verified). The 6710-2RS alternative
(50x62x6, 6 mm narrower) is lighter as a BEARING but heavier as a listed
PART (67 g) and its C0 varies 2.6-3.1 kN across the sources checked -- 6810
picked for the more consistent citation, not because the load needs it
(§3.1 of the round-1 write-up already showed even the smallest bearing
tried clears these loads by ~9-20x; at 5.8 kN vs a ~18-34 N applied load
this is a 170-320x margin, the same "geometry, not load" story as round 1).

**Construction** (`cad/v6/yaw_carrier_v6_optA.py`), entirely within carrier
z in [-7, 0], nothing below that touched:

1. Trim the carrier's existing material (plate corners AND the top of the
   bay walls) back to a cylinder of radius 25.04 mm. This clips the FOUR
   CORNERS of the existing 25.12 mm rectangle by **0.08 mm** -- under print
   tolerance, and not a load path (load goes through the horn screws at the
   centre and the bearing at the OD, not these corners) -- and, along the
   flat sides (19.95 and 15.26 mm half-widths, both < 25.04), ADDS a little
   material. Net: the top of the carrier becomes a true cylinder.
2. Re-open the same rectangular cavity the bay walls already had for
   z in [-7, -3] (where the roll servo's case passes through), so the round
   hub does not seal it off -- the SAME operation Option C already does one
   z-band lower, just needed here because the band reaches that territory.
3. Re-cut the 4 horn screw bores + centre relief (refilled by step 1's disk
   add wherever they fall inside the band).

**What's inside the band, disclosed:** both roll-servo retention screw rows
(-11.11, -7.36) stay OUTSIDE the band (0.36 mm and 4.11 mm clear
respectively, asserted in `dimensions_v6.py`) -- untouched. Only the very
top ~2 mm of the idler-side SEAT PAD (added 2026-08-01, a fix for a
different phantom-face defect) is trimmed where it happened to reach up to
-5.00 inside the band; that pad's actual job (locating the case against its
two screw rows) is done entirely below both rows, well clear of this
trim -- not independently re-verified by a dedicated check, flagged here
rather than assumed harmless.

**Horn screws:** unaffected -- same 4x M3, same 3 mm plate engagement, bolt
circle r 7 (8.7 mm reach with a self-tap head), 16.3 mm of hub wall outside
that. No interaction with the bearing boss at r 25.04.

**Print:** SAME orientation as v5 (`RX180`, horn-plate face on the bed, bay
walls rise) -- no new overhang class, part count unchanged (2 carriers by
translation, 1 pelvis).

**Pelvis side** (`pelvis_v7(bearing_variant="A")`): the recess/shoulder hang
from the SAME unmoved cell-tube rim (`V.YAW_BOX_BOT_Z`) the same way option
C's does -- no pelvis growth either.

| feature | value |
|---|---|
| recess ID | 64.96 mm (r 32.48), -0.04 mm interference |
| shoulder land / ID | 1.2 mm / 62.6 mm (r 31.3) |
| estimated inner-ring OD / outer-ring ID | ~28.04 / ~29.54 mm (ESTIMATE, scaled the same way as option C's -- no published internal-geometry spec found for this bearing either; margin to shoulder 1.76 mm) |
| skirt OD | 71.0 mm (~3 mm wall) |
| skirt-to-skirt gap at 84 mm hip sep | **13 mm** edge-to-edge (vs option C's 6 mm) |
| bridge | kept anyway, same style as option C's -- a conservative judgement call given the doubled gap, NOT FEA-verified either way |
| recess/shoulder Z, pelvis-local | recess -80.80..-73.80; shoulder -73.80..-72.0 (the SAME existing 1.8 mm gap option C reuses) |
| axial growth | **none** -- `HIP_YAW_Z`/`DECK_BOT_Z`/`DECK_TOP_Z` unchanged |
| pelvis mass (measured, this session) | 190.3 g (option C: 194.3 g) |
| carrier mass (measured, this session) | 23.8 g (option C: 29.9 g; v5 unmodified: 17.8 g) |

**Checks (both required by this study):**

- `check_assembly_v6.py` full suite (21 joint pairs + 4 inter-leg pairs, via
  `YAW_BEARING_VARIANT=A`, a new opt-in-only hook in `assembly_v6.py`'s
  `part_yaw_carrier()`/`part_pelvis()` that leaves every existing caller's
  default ("C") behavior byte-identical): **ALL CLEAR**, 82 s. Full output
  `docs/design-v6/yaw_bearing_check_optA.txt`.
- `cad/v6/check_yaw_bearing_optA.py` (option C's ring-based method,
  re-parametrized off the `YAWA_*` constants): the bearing modeled as two
  solid rings, checked through yaw x hip_roll x hip_pitch at their ROM
  extremes (12 poses x 5 pairs): **ALL CLEAR**. Same output file.
- Roll and pitch don't actually move the carrier or the pelvis (both are
  upstream of those joints in the kinematic chain -- only hip_yaw does), so
  the informative pose for both checks is the yaw sweep; roll/pitch are
  swept anyway for direct comparability with option C's own check.
- No walk-gate re-run needed: `d_yaw_roll`/`HIP_YAW_Z` did not change, so
  the kinematics gate is untouched by this option (unlike round 1's
  rejected axial-growth version, which would have needed one).

**Renders:** `cad/v6/renders/yaw_bearing_optA_after.png` (before is the SAME
pre-study state as option C's -- `yaw_bearing_before.png`, not
re-rendered), `cad/v6/renders/yaw_bearing_optA_section.png` (axis section,
same cut convention as option C's -- the round shoulder/collar is visible
sitting on the carrier's hub). Fly-in filmstrip for the whole robot with
this option: `cad/v6/renders/assembly_v6_flyin_optA_strip.png` (the
committed baseline strip, `assembly_v6_flyin_strip.png`, is unchanged --
option A does not change anything else in the body, and the difference at
whole-robot scale is subtle since the OD only shrinks ~10%; the section
render is the one that actually shows it).

**STEP** (for review in FreeCAD): `cad/v6/step/yaw_bearing_recommended/` --
`yaw_carrier_v6_A.step`, `pelvis_v7_A.step` (full parts) and
`joint_assembly_A.step` (a small sub-assembly: the pelvis CELL REGION only,
cropped to one hip, + the carrier + both bearing races modeled as rings +
the yaw AND roll servo mocks, all in world placement).

## 4. Option B: raise the yaw servo into the housing -- STOPPED AT MEASUREMENT

The brief: move the yaw servo UP so the bearing band sits at the cell mouth
around the case's lower end (also a 35+ mm bore class, by the same
corner-reach logic as §3), with the leg's own stack unchanged (so the robot
does not get taller) -- meaning the CELL CEILING (today `CELL_TOP_Z`, the
battery bay's floor) has to rise the same amount the servo does, since the
servo's case is retained from the ceiling down.

**Measured** (`dimensions_v6.py` `YAWB_CLEAR_BELOW_BATT`/`YAWB_CLEAR_ABOVE_BATT`):
the battery pack sits directly above the cell ceiling with only **0.5 mm**
of air below it (battery bottom to ceiling) and **4.5 mm** above it (battery
top to deck bottom) -- the entire 31 mm battery layer is already almost
fully consumed. The brief's 7-10 mm of rise:

| needed rise | available (above the battery) | shortfall |
|---|---|---|
| 7 mm (low end) | 4.5 mm | **2.5 mm** |
| 10 mm (high end) | 4.5 mm | **5.5 mm** |

**What would have to move:** the battery pack (or the deck it sits under)
by 2.5-5.5 mm, since the cell ceiling rising eats the 4.5 mm slack above the
battery before it can rise the full 7-10 mm the brief asks for. Moving the
battery layout or the deck is explicitly out of this study's authorization
(task scope: pelvis + carrier only). **Stopped here -- no CAD built for
Option B**, per the task's own instruction to stop at the measurement when
a change outside scope is required.

## 5. Option C: the 6811 wrap -- baseline, runner-up

Unchanged from round 1 (`docs/design-v6/study-yaw-bearing.md` git history
has the original write-up's §3/§5 with the two earlier REJECTED passes,
6707/6709, kept there so neither is retried): a 6811-2RS (55x72x9, C0
6.2-8.4 kN, 8.1 kN used) wraps around the OUTSIDE of the carrier's ENTIRE
horn-plate + upper bay-wall footprint (boss r 27.54, wall 2.42 mm over the
25.12 mm corner). Re-measured this session (numbers unchanged from round 1,
confirmed by rebuilding both parts): carrier mass 29.9 g, pelvis mass
194.3 g, skirt OD 78 mm (6 mm edge-to-edge at 84 mm hip sep, bridged).
`check_assembly_v6.py` (default, no env var) and `check_yaw_bearing_combo.py`:
**ALL CLEAR** (re-run this session, same result as round 1,
`docs/design-v6/yaw_bearing_check.txt`). No axial growth here either --
this was already true in round 1. **STEP**:
`cad/v6/step/yaw_bearing_runner_up/` -- `yaw_carrier_v6_C.step`,
`pelvis_v7_C.step`, `joint_assembly_C.step` (same sub-assembly convention as
option A's, for direct comparison in FreeCAD).

## 6. Option D: considered on paper, not built

- **Stub-axle with two small bearings beside the servo:** would move the
  leg's attachment point OFF the yaw axis centreline entirely, replacing
  the single coaxial thrust/moment bearing with an offset pair reacting
  moment as a couple -- a different LOAD-PATH MECHANISM, not a bearing
  swap, and it still has to fit beside the yaw servo within the same
  15.26 mm cheek-to-cheek footprint the carrier already uses (no width to
  spare there without widening the cell, which is out of scope). Not
  pursued: option A already meets the "round, no axial growth" brief with a
  much smaller change.
- **Flanged bearing (F6810/F6811), seated by its flange:** same bore/OD as
  option A's pick, but the flange itself becomes the axial thrust face,
  potentially removing the separate machined shoulder step in the pelvis
  recess. No flanged-variant datasheet was found for a 50 mm-bore thin
  section in the sources checked this session (a real gap, not a guess) --
  plausible as a refinement of option A's pelvis side, but it changes
  neither the bore choice nor the recommendation, so not built.

## 6b. Option E: Option A + positive retention of both races (round 3, 2026-09-18)

Tom's review of option A: *"I'm skeptical that press fitting into the
printed PETG will be effective... I'd think that it would fall out. Are
there bearing/joint options with built in flanges that we could screw in
to the plastic?"* Agreed: option A's interference (0.04 / 0.08 mm on
diameter) is below what the printer holds on a 65 mm circle, and PETG
creeps, so within weeks both fits are slip fits. Option A has ONE positive
stop (the pelvis shoulder above the outer race); the other three axial
directions are friction only, so in swing the leg would hang from the horn
screws exactly as it does today and the bearing would carry nothing.

**Off-the-shelf flanged / bolt-on options at this bore (searched this
session):**

| option | bore x OD x W | holes | fits the band, no stack growth | mass each |
|---|---|---|---|---|
| 6810-2RS (option A) | 50 x 65 x 7 | none | yes | 52 g |
| F6810 flanged (Lily) | 50 x 65 x 7 + 1.5 flange | none | yes; but the flange lands on the skirt's BOTTOM face (inserted from below) so it only stops the race going UP, which the shoulder already does | ~52 g |
| THK RU42 cross roller ring | 20 x 70 x 12 | both rings | **no** -- 20 mm bore vs the roll servo's 22.23 mm reach in the band, so it must sit above the roll servo's cable end: ~+7 mm stack | 290 g (x2 = a third of the robot) |
| small slewing rings (PBC, IKO) | >= 70-80 mm OD | both rings | no, same bore/height problem, heavier | -- |

None solves it. **Option E keeps the 6810-2RS and adds the three missing
stops, one of them a screwed part per side** (`cad/v6/yaw_retention_optE.py`,
STEP in `cad/v6/step/yaw_bearing_optE/`):

- **Carrier, under the inner race -- LIP (printed):** ring r 25.04..27.5,
  carrier z [-8.3, -6.8]. The race now goes on from the horn-face side and
  sits on the lip. Stance load: shoulder -> outer race -> balls -> inner
  race -> lip -> carrier.
- **Carrier, over the inner race -- CAP (separate, screwed):** ring
  r 17..28.5 x 1.0 mm in the EXISTING gap between the horn face and the
  cell rim, 3x M2.5 flat-head self-tap at r 22 into the solid hub (pilot
  2.05, the build's standard; heads countersunk flush, the cone continuing
  0.3 into the hub).
  Fitted with the carrier OUT of the pelvis (no driver access in the gap).
- **Pelvis, above the outer race -- shoulder:** unchanged from option A.
- **Pelvis, under the outer race -- RETAINER (separate, screwed):** ring
  1.5 mm with a 0.9 mm land r 29.8..32.28 up into the recess onto the
  outer race's bottom face, 3 tabs to
  three new dia-8 bosses on the skirt OD (r 38.5, outboard and +-120 deg,
  off the inboard bridge line), 3x M2.5x8 flat-head self-tap driven UP
  from below. Swing load: carrier -> cap screws -> cap -> inner race ->
  balls -> outer race -> retainer -> boss screws -> pelvis.

**Play (Tom's condition for accepting E: tolerances must still eliminate
play).** Three kinds, handled three ways:

- *Axial and tilt play* -- the kind that actually reaches the foot through
  the 0.335 m lever -- is removed by PRELOAD, not by fit. The whole bearing
  sits `E_PRELOAD` = 0.2 mm higher than in option A, so the race stands
  0.2 proud of the horn face and the pelvis shoulder's underside is raised
  0.2 to match (land 1.8 -> 1.6 mm). The cap bottoms on the inner race with
  a 0.2 gap to the hub face behind it; the retainer's 0.9 mm land reaches
  into the recess and bottoms on the outer race with a 0.2 gap to the
  skirt/boss faces behind it. Tightening the screws therefore clamps the
  RACE, never the print; print error of up to 0.2 mm in either direction
  goes into that gap. Measured in the STEP (`distToShape`): cap-race 0.00,
  cap-hub 0.20, retainer-race 0.00, retainer-pelvis 0.20, race-shoulder
  0.00, race-lip 0.00, cap-cell-rim 1.1.
- *Radial play* (the fit of the hub in the bore and the race in the
  recess): a 0.04/0.08 mm interference is inside the printer's error band,
  so the CAD fit alone is neither reliably tight nor reliably loose. The
  standard answer for a bearing in a plastic or loose housing is a
  RETAINING COMPOUND on both seats (Loctite 641 medium strength, gap fill
  0.15-0.25 mm, disassemblable; 648 if it must never come apart). With the
  axial clamps above, the compound only has to take up radial slop, not
  hold the bearing in. Print the hub and recess at nominal, measure, and
  fit with compound. Note a 0.1 mm radial gap at the hip moves the foot
  0.1 mm; it does NOT become an angle.
- *Yaw rotational play* (the 1-2 deg measured on v5) lives in the horn
  spline and the 4 horn screws and is NOT touched by any bearing option;
  what the bearing does is take the moment off those screws so they stop
  working loose. Separate item.

**Heat-setting the bearing into the PETG (Tom's question): no.** PETG has
to be taken to ~230 C to flow around a race; a 2RS bearing's seals and
grease are good to roughly 100 C, so the bearing is cooked before the seat
forms. The melted seat also has no controlled diameter (so no defined
radial fit), gives no positive stop in the lift-out direction, and still
creeps afterwards. Heat-set INSERTS remain fine (small brass, local heat)
but are not needed here: every screw is the build's M2.5 flat-head
self-tap into a 2.05 pilot, with >= 6 mm of engagement in each boss/hub.

**Checked:** 15 must-not-touch pairs (cap vs outer race / pelvis / yaw
servo / carrier / inner race; retainer vs inner race / carrier / roll
servo / pelvis / outer race; carrier vs pelvis / both races; pelvis vs
both races) all 0.000 mm3 overlap at the build pose. No stack growth:
`HIP_YAW_Z`, `CARRIER_ROLL_AXIS`, `YAW_BOX_BOT_Z` untouched. Volumes:
carrier 21.44 cm3, cap 1.62 cm3, retainer 2.39 cm3 (pelvis 169.08 cm3
incl. six bosses).

**Not yet done:** the full `check_assembly_v6.py` yaw/roll/pitch sweep
(the new parts are axisymmetric about the yaw axis, so the informative
pose is the build pose, but the sweep is owed for parity with A/C); the
fly-in strip; the BOM line for the retaining compound. Cap OD and
retainer ID hang off the same ESTIMATED ring split as A/C (1.04 / 1.76 mm
margins) -- verify on the real bearing before cutting.

## 7. Recommendation

**Option A (round hub, 6810-2RS) over Option C (6811 wrap, baseline):**

1. Genuinely round -- Tom's ask directly, not a rectangle wrapped in a
   circle. Option C's boss is round on the OUTSIDE but its whole reason for
   being that size is the carrier's own rectangular corner; option A's hub
   IS the carrier's top, no rectangle underneath it within the band.
2. Smaller everywhere it matters: bearing OD 65 vs 72 mm, skirt OD 71 vs
   78 mm, skirt-to-skirt gap 13 vs 6 mm (more than double), carrier mass
   23.8 vs 29.9 g, pelvis mass 190.3 vs 194.3 g.
3. Neither option grows the axial stack (both satisfy Tom's constraint 1
   equally) -- this was not a point of difference, checked explicitly for
   both (`HIP_YAW_Z`/`d_yaw_roll` unchanged in both `dimensions_v6.py`
   sections).
4. Load margin is enormous either way (170-320x for A, 236-458x for C at
   the same applied loads) -- not a deciding factor, exactly as round 1's
   own §3.3 already said about bearing sizing on this joint.
5. Cost of A vs C: a hair more surgical CAD (the "re-open the cavity, re-cut
   the screws" construction, vs C's simpler "wrap outside, touch nothing"),
   a 2 mm trim off the top of one existing feature (the idler seat pad,
   disclosed in §3, not independently re-verified), and a worst-corner
   figure that comes from a measured geometry slice rather than a single
   named catalogue constant (reproducible via
   `cad/v6/yaw_carrier_v6_optA.py`'s own docstring/build, but a small
   documentation trade against C's cleaner `hypot()` derivation).

Both options are fully built, checked (`check_assembly_v6.py` full suite +
the ring-based combo check, ALL CLEAR for both) and exported to STEP for
side-by-side review. Option B is a real, measured negative at this
authorization boundary (battery/deck), not a soft "not tried." Option D is
reasoned, not built, since neither variant would beat A.

## 8. Owed before the sim re-run / before cutting metal

- **Mass**: neither option's bearing rings nor the pelvis's own new print
  mass are in the sim plant yet (`sim/build_v6_inertia.py --write` needs
  the new STL + a bearing-mass lump before the walk gate is trusted again,
  per the 09-13 rule) -- unchanged from round 1's owed item, and LOWER
  URGENCY than round 1 thought since neither A nor C in this round changes
  any kinematic constant, so the EXISTING gate result stands geometrically;
  only the mass rollup is stale.
- **Internal ring split is an ESTIMATE for BOTH bearings** (6810-2RS same
  as 6811-2RS) -- no published internal raceway/ball geometry found for
  either. Verify against the real bearing's datasheet or a caliper
  measurement before cutting the shoulder metal, same caveat as round 1.
- **Bridge on option A's skirts**: kept despite the doubled gap (13 vs
  6 mm) as a conservative, NOT FEA-verified choice -- worth a bench check
  (or a deflection estimate) on whether it is still needed once a real
  6810-2RS is in hand; removing it would save a small amount of print
  material and one extra feature.
- **6710-2RS vs 6810-2RS**: 6710 is 1 mm narrower and, oddly, listed 15 g
  HEAVIER as a bought part across the sources checked (67 vs 52 g) with a
  less consistently published C0 (2.6-3.1 kN across sources vs 6810's
  single clean spec sheet) -- picked 6810 for the citation, not the load;
  if 6710 is in stock and its rating is confirmed, it is interchangeable
  (same bore, same recess, `YAWA_BRG_W`/`YAWA_BRG_OD` are the only two
  numbers that would change).
- **Flanged-bearing refinement (Option D)**: not pursued this session for
  lack of a found datasheet at this bore/section; worth a supplier search
  if Option A is selected and a simpler pelvis-side shoulder is wanted.
- **Idler-side seat pad trim** (Option A, §3): the top ~2 mm is cut back by
  the band; both retention screw rows it actually locates against are
  clear of the trim, but this was reasoned, not independently
  re-verified by a dedicated check.

## BOM delta

Not touched this session (task instruction) -- the existing 6811-2RS line
in `docs/design-v6/bom-delta.md` stands until Tom selects an option; if
Option A is picked, that line becomes 2 (+2 spares) 6810-2RS, 50x65x7 mm,
sealed, chrome steel, deep-groove "68xx" series.

## 9-sentence verdict

Round 1's 6811-2RS wrap (kept here as Option C, the baseline/runner-up)
sized a round boss to the carrier's own rectangular corner (25.12 mm),
which is round on the outside but still a rectangle-driven number
underneath; Tom asked for the joint itself to be round, and for the hip
stack to not get any taller. Option A puts the round hub at the carrier's
EXISTING height (no change anywhere to `HIP_YAW_Z`, `CARRIER_ROLL_AXIS`, or
`d_yaw_roll`), sized instead to what the geometry actually shows is inside
that fixed band: not the small yaw-servo horn disc (a mistake this round
caught and corrected), but the bulkier ROLL SERVO's own case, measured at a
22.23 mm worst corner -- which clears with a 50 mm-bore 6810-2RS (65x7,
2.81 mm of wall) after a smaller 45 mm bore FAILED outright (0.31 mm of
wall, shown explicitly as a negative control). The result is smaller
everywhere that matters -- 65 mm bearing OD vs 72, a 71 mm skirt vs 78, and
more than double the clearance between the two hip skirts (13 mm vs 6) --
at the cost of a slightly more surgical carrier construction and a 0.08 mm
trim off one existing corner, both disclosed rather than hidden. Option B
(raising the yaw servo instead) is a genuine, measured negative: only
4.5 mm of air exists above the battery pack where 7-10 mm is needed, a
2.5-5.5 mm shortfall that only the battery layout or the deck could close,
both outside this study's authorization, so it was stopped at the
measurement with no CAD built. Both A and C pass the full
`check_assembly_v6.py` suite and a bearing-modeled-as-two-rings combo check
across the yaw x roll x pitch ROM with zero interference, and both are
exported to STEP (`cad/v6/step/yaw_bearing_recommended/` and
`.../yaw_bearing_runner_up/`) as a small joint sub-assembly -- pelvis cell
region, carrier, both bearing races, and the yaw and roll servos -- so Tom
can compare them directly in FreeCAD. The recommendation is Option A; the
BOM is left untouched for Tom to act on.
