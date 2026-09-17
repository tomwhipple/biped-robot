# Hip-yaw bearing study (2026-09-17)

Tom, 2026-09-17: *"What about the hip yaw? The servo axle is the only
connection between the leg and the rest of the robot ... it seems to me
we're fighting a lot of mechanical leverage there."* Confirmed in the CAD
(`cad/parts.py` `yaw_carrier`, `cad/v6/pelvis_v7.py`): every other joint in
v6 is closed horn + idler (both faces gripped, load shared); the hip yaw is
a cantilever -- the STS3215 sits in a cell under the pelvis deck, case fixed
to the ceiling, horn down, and the carrier (the WHOLE leg, a 0.335 m lever)
bolts to the horn alone. Symptoms on v5: 1-2 deg of measured yaw play, horn
screws working loose, heading shortfall in turns (`sts-goal-writes-move-the-
robot.md`-adjacent field notes; v5 build log).

**2-hour budget. Verdict up front:** a 6709-2RS deep-groove ball bearing
(45x55x6) fits around the OUTSIDE of the carrier's own horn-mount plate --
not around the servo's horn disc, which is too small to leave a printable
wall -- entirely below the existing pelvis floor, with no axis drop and no
change to the leg's kinematic chain. `check_assembly_v6.py` passes at yaw
0/+-45 combined with hip_roll and hip_pitch at their ROM extremes (see
`docs/design-v6/yaw_bearing_check.txt`). One real compromise: the pelvis-
side outer-race seat ended up a clearance fit, not the press fit first
planned, so there is up to 0.75 mm of radial play there as modeled -- see
§6.

## 1. Load path, before and after

**Before:** body weight (thrust, ~15 N of the ~1.5 kg robot's mass in
single support) + the single-support roll moment (~0.65 N-m,
`docs/design-v6-ankle-roll.md` §4.4) + any side load at the foot x 0.335 m
lever all go through: the servo's internal output bearing -> the horn
spline -> 4x M3 horn screws -> the carrier's flat plate. The plate mates the
horn by friction/screw clamp only; nothing pelvis-side reacts a moment
except through that same shaft.

**After:** the SAME 4 horn screws stay (torque only -- they still turn the
leg). Thrust and moment now go: carrier plate -> a new boss on the plate's
own rim -> the 6709-2RS bearing -> a new recess in the pelvis housing ->
the housing's own print material. The servo's output shaft carries torque
only, exactly like every other joint's horn already does relative to its
own idler-side grip.

## 2. Envelope (measured from the CAD)

All numbers from `cad/dimensions.py`, `cad/v6/dimensions_v6.py`, and
`cad/parts.py yaw_carrier()` / `cad/v6/pelvis_v7.py` -- the actual formulas
that build the STLs, not estimates.

| feature | value | source |
|---|---|---|
| servo case half-width (Y) | 12.36 mm | `D.SV_WID/2` |
| servo case reach, forward (output end) | 10.11 mm | `D.SV_AXIS_FROM_OUT_END` |
| servo case reach, aft (cable end) | 35.11 mm | `D.SV_AXIS_FROM_REAR` |
| **servo case's own max corner reach from the axis** | **15.97 mm** | `hypot(10.11, 12.36)` |
| horn disc diameter | 19.2 mm (r 9.6) | `D.SV_HORN_D` |
| horn bolt circle | O14 (r 7), 4x M3 | `D.BCD` |
| horn disc stands proud of the case's own flat face | 3.10 mm | `D.SV_HORN_FACE - D.SV_TOPFACE` |
| carrier horn-mount plate (== bay ceiling) footprint | 39.9 x 30.52 mm (+-19.95 x +-15.26) | `parts.yaw_carrier()`, `box(-19.95,19.95,-hw,hw,...)` |
| **carrier plate's own corner reach from the axis** | **25.12 mm** | `hypot(19.95, 15.26)` |
| yaw cell tube (servo clearance) half-width | 12.66 mm | `D.YAW_BOX_HW_IN` |
| yaw cell tube wall | 2.6 mm | `D.YAW_SEAT_WALL` (`D.WALL`) |
| yaw cell tube outer face | 15.26 mm | `D.YAW_BOX_HW_OUT` |
| centre channel (open, between the two cells) | +-26.74 mm | `V.HOUSING_CHAN_HW` |
| **vertical gap, tube rim to horn face TODAY** | **1.80 mm** | `D.YAW_BOX_CARRIER_GAP` |
| that gap is 1.30 mm inside the tube (wraps the case) + 1.50 mm below it (open air) | -- | `YAW_BOX_BOT_Z - CASE_BOT` / `CASE_BOT - HORN_FACE` |
| GD/Pi board columns' own floor | z = `V.YAW_BOX_BOT_Z` (same as the tube rim) | `dimensions_v6.py` `GD_BOT_Z`/`PI_BOT_Z` |

**Max OD without moving the leg down:** essentially none of the standard
sizes -- the disc-to-case-face gap is only 3.1 mm total (of which only 1.8
mm is genuinely free of the servo's own case body), well under any listed
bearing's width (4-6 mm), and that 3.1 mm is a fixed servo fact: the disc
always stands 3.1 mm proud of the case, independent of how the case is
mounted to the pelvis. A pelvis-side spacer under the case ("moving the
axis down") does NOT create room there -- it moves the whole servo (case
+ disc) together, so the 3.1 mm gap is unchanged. What DOES have room,
with no axis change at all, is everything BELOW the yaw cell tube's rim:
the servo case and the GD/Pi board columns both end at that exact Z
(`V.YAW_BOX_BOT_Z`), so the space below it was already open on all sides.
That is where this design puts the bearing.

## 3. Bearing choice

**6709-2RS deep-groove ball bearing, 45 x 55 x 6 mm** (catalog values via
NTN/tradebearings-style listings: dynamic C = 2.58 kN, static **C0 = 2.40
kN** -- see the search trail in this session; treat as typical-catalogue,
not a specific vendor's guarantee).

Two smaller sizes were tried and rejected, in order, each for a *geometric*
reason (load capacity was never the constraint -- see §3.3):

1. **6704-2RS (20x27x4), boss around the horn disc.** Bore radius 10 mm vs
   the disc's own 9.6 mm radius leaves a 0.4 mm boss wall -- not printable,
   and the disc-to-case gap (3.1 mm total, 1.8 mm free) does not fit even a
   4 mm bearing without lengthening the stack.
2. **6707-2RS (35x44x5), around the carrier's plate rim instead of the
   disc.** Bore radius 17.5 mm clears the servo case's 15.97 mm corner by
   1.5 mm at any height -- looked right, but the check
   (`check_assembly_v6.py --joint hip_yaw`) found 240 mm3 of overlap
   growing with yaw angle: the carrier's own PLATE (not just a boss) is
   bigger than the disc -- it doubles as the roll bay's ceiling, corner
   25.12 mm from the axis -- and 44 mm OD (radius 22) does not clear that.
3. **6709-2RS (45x55x6)** -- recess radius 27.475 mm clears the plate's
   25.12 mm corner by 2.35 mm. This is the pick.

### 3.1 Radial (moment) arithmetic

A single-row deep-groove bearing takes an applied moment as a force couple
across its own ball circle, roughly the mean of bore and OD:

```
D_pw = (45 + 55) / 2 = 50 mm = 0.05 m
F(moment) = M / D_pw
  steady  (M = 0.65 N-m):  F = 0.65 / 0.05  = 13.0 N
  impact  (M = 1.7  N-m):  F = 1.7  / 0.05  = 34.0 N
```

Combined with the 15 N thrust as an equivalent static load (ISO-76-style
approximation for a single-row deep-groove bearing, P0 = Fr + 0.5 x Fa):

```
P0(steady) = 13.0 + 0.5x15 = 20.5 N   -> C0/P0 = 2400/20.5  = 117x margin
P0(impact) = 34.0 + 0.5x15 = 41.5 N   -> C0/P0 = 2400/41.5  =  58x margin
```

### 3.2 Bore vs the horn screws

Boss radius 22.55 mm clears the O14 bolt circle (r 7) + a self-tap head
(~1.7 mm) by a wide margin (`assert` in `dimensions_v6.py`); the 4 horn
screws are unaffected by any of this (see §5.2).

### 3.3 Why size is geometry-driven, not load-driven

Even the smallest bearing tried (6704, C0 = 0.73 kN by the same catalogue
class) clears these loads by ~9-20x. Every size decision in this study was
about fitting a round bore/OD around rectangular servo-case and carrier-
plate footprints, not about static load rating -- worth stating plainly so
a future pass does not "upsize for strength" when the actual lever is
clearance.

## 4. CAD changes

- **`cad/v6/yaw_carrier_v6.py`** (new; `cad/v6` had no carrier override --
  `parts_v6.py` mapped `yaw_carrier` straight to `parts.yaw_carrier()`,
  "v5 part, unchanged"). Builds on `parts.yaw_carrier()` and adds:
  - a boss, radius `V.YAW_BRG_BOSS_OD/2` = 22.55 mm (+0.10 mm nominal
    interference vs the bearing's 45 mm bore -- see §6), from the plate's
    z=0 face down 6.0 mm (`V.YAW_BRG_BOSS_H`, the bearing's own width),
    built as a full disk then re-opened along the SAME x/y footprint the
    roll bay's own front/rear/cheek walls already use, so the bay's
    interior (where the roll servo slides up) is simply ~3 mm deeper at
    its mouth -- nothing that used to be solid changes.
  - a **corner trim**: the existing plate (also the bay's ceiling) is
    39.9 x 30.52 mm, corner 25.12 mm from the axis -- too big for the
    pelvis-side shoulder to clear (found by the interference check, see
    §5.1). The top slab ONLY (z = -3..0, the horn-mount plate itself, not
    the bay walls below it) is trimmed back to r = 23.05 mm (the boss
    radius + 0.5 mm). The walls (z < -3) and the wall-to-plate joints
    inboard of that radius are untouched; a small triangular sliver of
    roof is lost directly over each wall's outermost corner, not expected
    to be structurally significant.
  - Print: **same orientation as v5** (`RX180`, "horn-plate face on bed,
    bay walls rise") -- the boss and the trim both live at MORE NEGATIVE
    local Z than the plate's z=0 mating face, the same side the bay walls
    already rise from; no new overhang class.
- **`cad/v6/pelvis_v7.py`**: a new skirt hanging from the EXISTING yaw-cell
  tube rim (`V.YAW_BOX_BOT_Z`, not moved) down 7.8 mm total:
  - 1.8 mm shoulder land (ID 51 mm, a 2 mm radial ledge -- reacts the
    leg's upward thrust into the pelvis print) -- this exactly reuses the
    gap that was already there (`D.YAW_BOX_CARRIER_GAP`, unchanged).
  - 6.0 mm recess (ID 56.5 mm, see §6) for the bearing's outer race,
    flush with the horn face (`V.YAW_HORN_FACE_Z`) at its bottom.
  - Skirt OD 62 mm (~3 mm wall around the recess). Two skirts, 84 mm hip
    separation, leave 22 mm edge-to-edge -- comfortably clear of each
    other and of the (already-open) centre channel; nothing routes
    through that gap below the tube rim today (checked against
    `V.HOUSING_CHAN_HW` and the cable/leg-bus slot positions, which are
    all at z > `V.YAW_BOX_BOT_Z`).
  - All new constants (`YAW_BRG_*`) live in `cad/v6/dimensions_v6.py`,
    parametric off `D`/`V`, with asserts that the boss clears the case
    corner and that the skirt hangs from the unmoved rim.
- Nothing above `V.YAW_BOX_BOT_Z` changes: `TORSO_FLOOR_Z`, `HIP_YAW_Z`,
  `DECK_BOT_Z`, and every joint's Z in the leg chain are byte-identical to
  before this study.

## 5. Checks

Run: `.venv/bin/python cad/v6/pelvis_v7.py`, `yaw_carrier_v6.py`,
`assembly_v6.py`, then `check_assembly_v6.py` (all pairs) and the new
`cad/v6/check_yaw_bearing_combo.py` (yaw 0/+-45 CROSSED with hip_roll and
hip_pitch at their ROM extremes -- the single-joint sweep in
`check_assembly_v6.py` does not combine joints). Full output:
`docs/design-v6/yaw_bearing_check.txt`.

### 5.1 The failure that shaped the design

First pass (6707-2RS, boss around the case footprint only) FAILED
`check_assembly_v6.py --joint hip_yaw`: 240 mm3 overlap, growing with yaw
angle, between `yaw_carrier_L` and `pelvis_v7`. Root cause: the carrier's
own plate corner (25.12 mm) exceeded the 6707's 22 mm recess radius. After
switching to 6709 alone, the SAME pair still failed at min-distance 0.38 mm
(< `D.SWEEP_BUFFER` 0.5 mm) -- this time the pelvis-side SHOULDER (radius
25.5 mm) against the still-untrimmed plate corner (25.12 mm): 25.5-25.12 =
0.38 mm, exactly the reported number. Fixed by trimming the plate's top
slab to r=23.05 (§4) rather than growing the shoulder further (which would
have eaten into the land needed to react thrust).

### 5.2 Screw access and print orientation (asked explicitly, verified)

- The 4 horn screws are unaffected: they still thread from BELOW, through
  the SAME `D.YAW_CARRIER_PLATE` = 3.0 mm plate (z = 0..-3, unchanged),
  reachable through the roll bay's cavity (now ~3 mm deeper at its mouth,
  which only helps tool access). The screw heads' seating area (bolt
  circle r 7 mm) is well inside r = 17.35/12.66 mm -- solid, untouched by
  either the boss or the corner trim (both start beyond r ~17-23 mm).
- Print orientation for `yaw_carrier_v6`: **unchanged** (`RX180`,
  horn-plate face on the bed, bay walls rise) -- confirmed in §4.

### 5.3 Results (full output in `docs/design-v6/yaw_bearing_check.txt`)

`check_assembly_v6.py` (all 21 joint pairs + 4 inter-leg pairs, default 5
samples per joint sweep): **ALL CLEAR** (81 s). The two pairs this study
touches:

```
hip_yaw   yaw_carrier_L      pelvis_v7           0.00 mm3   2.30 mm   at -45 deg   ok
hip_yaw   servo_hip_roll_L   pelvis_v7           0.00 mm3   6.54 mm   at -45 deg   ok
```

`cad/v6/check_yaw_bearing_combo.py` (new script, this study -- yaw
0/+-45 CROSSED with hip_roll at +-55 and hip_pitch at -125/+90, 12
combined poses, single leg): **ALL CLEAR**, worst case at the most extreme
combination tried (yaw -45, roll -55, pitch -125):

```
yaw_carrier_L        pelvis_v7    0.00 mm3   2.30 mm   ok   carrier (boss+plate) vs the skirt/recess
servo_hip_roll_L     pelvis_v7    0.00 mm3   6.54 mm   ok   roll servo case vs the skirt/recess
```

Both minimum distances (2.30 mm) clear `D.SWEEP_BUFFER` (0.5 mm) by 4.6x,
and are unchanged between the single-joint sweep and the combined-extremes
sweep -- the carrier's closest approach to the pelvis happens at yaw's own
extreme regardless of roll/pitch, which makes sense: roll and pitch move
parts BELOW the yaw carrier, not the carrier's own rotating footprint
against the fixed skirt.

## 6. What did NOT fully close: a compromise on the outer-race fit

The original plan was a light press fit on BOTH races (boss +0.10 mm,
recess -0.05 mm). The plate-corner interference (§5.1) forced the recess
ID from 54.95 mm (-0.05 interference) out to **56.5 mm (+1.5 mm
clearance)** to buy the check's 0.5 mm sweep buffer without also shrinking
the shoulder's thrust-reacting land. That leaves **up to 0.75 mm of radial
play at the outer race as modeled** -- retention there is the shoulder
(axial, for the real single-support load direction) plus a bed of
retaining compound (CA/threadlocker) at assembly, not an interference fit.
This is a real, disclosed shortcoming of this pass, not a solved problem:
tightening it (by trimming the carrier's plate corner further, or
redesigning the plate's own footprint instead of only its top slab) is the
natural next iteration, not attempted here given the time budget.

**Swing-phase retention target, for the bench build:** the leg mass BELOW
the yaw joint (yaw_carrier print + hip-roll servo + everything down to the
foot, from the CAD mass rollup, `parts_v6.py`) is:

```
17.8 (yaw_carrier) + 74.5 (hip-roll STS3250) + 11.5 (yoke_roll)
+ 10.9 (yoke_pitch_v6) + 55.0 (hip-pitch STS3215) + 26.3 (thigh)
+ 74.5 (knee STS3250) + 26.3 (shin) + 55.0 (ankle-pitch STS3215)
+ 12.1 (ankle_link) + 74.5 (ankle-roll STS3250) + 52.0 (foot)
+ 23.2 (TPU sole) + ~10 (hardware) = 523.6 g -> 5.14 N static weight
```

No measured swing-phase acceleration exists for this gait (the demo suite
is quasi-static weight-shift/reversal, not a dynamic swing --
`natural-responsive-demo-plan.md`), so a conservative x2-3 dynamic factor
gives a **10-15 N** target: the retaining compound + shoulder combination
at the bench needs to hold that without the leg dropping out of the
recess, checked with a pull scale before any hardware run.

## 7. Sim: yaw play / lash model

**Not changed** (per the task -- the sim plant is untouched). The bearing
replaces the horn-spline+screw load path with a bearing seat; the AXIAL/
MOMENT lash that the spline used to carry under load is now reacted by the
bearing (117-58x margin, §3.1), so the physical mechanism the v5 1-2 deg
yaw play was attributed to (screws working loose under a cantilevered
moment) is removed for that axis. The bearing's own RADIAL clearance
(0.75 mm as modeled, §6) is a NEW, smaller lash source at the outer race
that the current plant model does not represent either way -- it is closer
to the free-hub/tolerance-stack play already modeled elsewhere (v5's
"free-travel play model" for other joints) than to a servo-spline slop, so
no plant change is proposed until the retention (§6) is actually tightened
or measured.

## 8. Print / assembly

- **Boss (carrier):** OD 45.10 mm nominal, +0.10 mm vs the bearing's 45 mm
  bore -- a first-cut interference figure with NO existing bearing
  precedent in `dimensions.py` to match (every other joint's "fit" is a
  screw or a horn spline, not a bearing bore); verify and trim after the
  first print the same way `D.FIT` (0.30 mm slip) was tuned for the servo
  case originally.
- **Recess (pelvis):** ID 56.5 mm, +1.5 mm clearance (see §6) -- drop-in,
  not press.
- **Shoulder:** ID 51 mm, 1.8 mm tall land -- prints as a small internal
  step; the print-audit (`pelvis_v7.py`'s own printability pass) flags it
  as a partially-anchored ledge (38-44% ring) at this z, the same class of
  warning dozens of existing v6 features already carry -- a lead-in
  chamfer would clean it up but was not added this pass (time budget).
- **Assembly order:** bolt the carrier's plate to the horn (4x M3, exactly
  as today) BEFORE the bearing goes on -- the boss stands on the plate's
  OUTER rim, clear of the screw heads, so order does not matter for screw
  access, but doing the horn bolts first keeps the leg supported on the
  bench while the bearing/skirt mate.
- **Quantity:** 2x 6709-2RS (one per hip yaw) + 2 spares (the same "buy
  spares" rule as the STS3250 stiffness-bench order in `bom-delta.md`).

## 9. Renders

`cad/v6/render_yaw_bearing.py` (new, this study) renders the yaw joint only
(carrier + the pelvis cell above it), not the whole robot:

- `cad/v6/renders/yaw_bearing_before.png` -- the pre-study joint (v5
  `yaw_carrier` + the pre-edit `pelvis_v7`, loaded from git HEAD): the cell
  mouth ends flush at the tube rim, nothing below it.
- `cad/v6/renders/yaw_bearing_after.png` -- the same view with the new
  round skirt hanging below the cell, and the widened carrier plate
  (orange) visible above it. This pair is the clearest evidence of the
  change -- the skirt is the one new silhouette feature.
- `cad/v6/renders/yaw_bearing_section.png` -- a half-space cut through the
  yaw axis. It shows the housing wall and the skirt's general shape but
  the camera angle does not cleanly expose the boss/shoulder/recess
  stack's internal faces -- a rougher result than the before/after pair,
  not a polished section drawing; the STEP files
  (`cad/v6/step/yaw_carrier_v6.step`, `cad/v6/step/pelvis_v7.step`) are the
  reliable way to inspect the actual internal geometry.
- Fly-in assembly animation: `cad/v6/animate_v6.py`, output gitignored
  (project rule) -- run separately if wanted;
  `cad/v6/renders/assembly_v6_flyin_strip.png` is the committed filmstrip.
- `docs/assembly.md` figures: **not regenerated** this pass --
  `cad/render_assembly_steps.py` targets the v5 assembly sequence, not
  the v6/v7 hip-yaw stack; rerunning it would not pick up this change.

## BOM delta

See `docs/design-v6/bom-delta.md` (added there too):

| item | qty | spec filter | why | est. |
|---|---|---|---|---|
| 6709-2RS deep-groove ball bearing | 2 (+2 spares) | 45x55x6mm, sealed (2RS), chrome steel (SAE 52100), deep-groove/thin-section "67xx" series | hip-yaw thrust + moment, replacing the horn-screw-only load path | not sourced here -- buy to the spec filter, Tom to source per project convention |

## 5-sentence verdict

A 6709-2RS (45x55x6, C0 = 2.4 kN) fits around the OUTSIDE of the carrier's
horn-mount plate -- not the servo's horn disc, which is geometrically too
small -- entirely in the space that was already open below the pelvis
floor and the servo case, with zero change to the leg's kinematic chain or
the sim plant. The bearing's own load margin is enormous (58-117x even at
the impact moment); every sizing decision here was forced by clearance
against the servo case's and the carrier plate's own rectangular
footprints, not by load capacity. `check_assembly_v6.py` plus a new
combined yaw/roll/pitch-extremes check confirm no interference through the
full ROM once the carrier's plate corners are locally trimmed for the
pelvis-side shoulder. The one real gap: the outer-race seat ended up a
0.75 mm clearance fit rather than the planned press fit, so this pass
relies on retaining compound plus the shoulder (which correctly reacts the
real, single-support thrust direction) rather than interference alone --
flagged, not hidden, as the next thing to tighten. This directly answers
Tom's complaint: the horn screws go back to pure torque, and the leg's
0.335 m lever now loads the pelvis print, not the servo shaft.
