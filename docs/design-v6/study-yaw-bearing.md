# Hip-yaw bearing study (2026-09-17)

Tom, 2026-09-17: *"What about the hip yaw? The servo axle is the only
connection between the leg and the rest of the robot ... it seems to me
we're fighting a lot of mechanical leverage there."* Confirmed in the CAD
(`cad/parts.py` `yaw_carrier`, `cad/v6/pelvis_v7.py`): every other joint in
v6 is closed horn + idler (both faces gripped, load shared); the hip yaw is
a cantilever -- the STS3215 sits in a cell under the pelvis deck, case fixed
to the ceiling, horn down, and the carrier (the WHOLE leg, a 0.335 m lever)
bolts to the horn alone. Symptoms on v5: 1-2 deg of measured yaw play, horn
screws working loose, heading shortfall in turns (v5 build log).

**Verdict up front:** a 6811-2RS deep-groove ball bearing (55x68... no,
55x72x9) wraps around the OUTSIDE of the carrier's ENTIRE horn-plate + upper
bay-wall footprint -- not a boss sized to the servo's disc or case alone,
both of which are smaller than the carrier's own rotating cross-section and
were tried and rejected first (§3). Both races are LOCATED with real
interference (boss +0.08 mm, recess -0.04 mm), no clearance fit. The
bearing itself is modeled as two solid rings in the interference check
(`cad/v6/check_yaw_bearing_combo.py`), not just the printed parts against
each other, and both the full `check_assembly_v6.py` suite and the ring
check pass through yaw x hip_roll x hip_pitch at their ROM extremes (see
`docs/design-v6/yaw_bearing_check.txt`). This design went through two
rejected passes before this one -- §3 and §5 keep both, with the numbers
that killed each, so neither gets retried.

## 1. Load path, before and after

**Before:** body weight (thrust, ~15 N of the ~1.5 kg robot's mass in
single support) + the single-support roll moment (~0.65 N-m,
`docs/design-v6-ankle-roll.md` §4.4) + any side load at the foot x 0.335 m
lever all go through: the servo's internal output bearing -> the horn
spline -> 4x M3 horn screws -> the carrier's flat plate. The plate mates the
horn by friction/screw clamp only; nothing pelvis-side reacts a moment
except through that same shaft.

**After:** the SAME 4 horn screws stay (torque only -- they still turn the
leg). Thrust and moment now go: carrier plate + the bay walls under it ->
a boss wrapping their WHOLE outer footprint -> the 6811-2RS bearing's
LOCATED inner and outer races -> a recess in the pelvis housing -> the
housing's own print material. The servo's output shaft carries torque
only, exactly like every other joint's horn already does relative to its
own idler-side grip.

## 2. Envelope (measured from the CAD)

All numbers from `cad/dimensions.py`, `cad/v6/dimensions_v6.py`, and
`cad/parts.py yaw_carrier()` / `cad/v6/pelvis_v7.py` -- the actual formulas
that build the STLs, not estimates (the one exception, the bearing's own
internal raceway split, is called out explicitly in §4 as an estimate).

| feature | value | source |
|---|---|---|
| servo case half-width (Y) | 12.36 mm | `D.SV_WID/2` |
| servo case reach, forward (output end) | 10.11 mm | `D.SV_AXIS_FROM_OUT_END` |
| servo case reach, aft (cable end) | 35.11 mm | `D.SV_AXIS_FROM_REAR` |
| servo case's own max corner reach from the axis | 15.97 mm | `hypot(10.11, 12.36)` |
| horn disc diameter | 19.2 mm (r 9.6) | `D.SV_HORN_D` |
| horn bolt circle | O14 (r 7), 4x M3 | `D.BCD` |
| horn disc stands proud of the case's own flat face | 3.10 mm | `D.SV_HORN_FACE - D.SV_TOPFACE` |
| carrier horn-mount plate (== bay ceiling) footprint | 39.9 x 30.52 mm (+-19.95 x +-15.26) | `parts.yaw_carrier()`, `box(-19.95,19.95,-hw,hw,...)` |
| **the bay's front/rear/cheek walls share that SAME footprint for their whole height** (z = -3 down to -44) -- not just the top 3 mm plate | -- | `parts.yaw_carrier()` wall boxes |
| **carrier's max corner reach from the axis (plate AND walls)** | **25.12 mm** | `hypot(19.95, 15.26)` |
| yaw cell tube (servo clearance) half-width | 12.66 mm | `D.YAW_BOX_HW_IN` |
| yaw cell tube wall | 2.6 mm | `D.YAW_SEAT_WALL` (`D.WALL`) |
| yaw cell tube outer face | 15.26 mm | `D.YAW_BOX_HW_OUT` |
| centre channel (open, between the two cells) | +-26.74 mm | `V.HOUSING_CHAN_HW` |
| **vertical gap, tube rim to horn face TODAY** | **1.80 mm** | `D.YAW_BOX_CARRIER_GAP` |
| that gap is 1.30 mm inside the tube (wraps the case) + 1.50 mm below it (open air) | -- | `YAW_BOX_BOT_Z - CASE_BOT` / `CASE_BOT - HORN_FACE` |
| GD/Pi board columns' own floor | z = `V.YAW_BOX_BOT_Z` (same as the tube rim) | `dimensions_v6.py` `GD_BOT_Z`/`PI_BOT_Z` |

**Max OD without moving the leg down:** the disc-to-case-face gap is only
3.1 mm total (1.8 mm of it genuinely free of the servo's own case body),
under any listed bearing's width (4-9 mm), and that 3.1 mm is a fixed servo
fact -- the disc always stands 3.1 mm proud of the case, independent of how
the case is mounted to the pelvis. A pelvis-side spacer under the case
("moving the axis down") does NOT create room there -- it moves the whole
servo (case + disc) together, so the 3.1 mm gap is unchanged. What DOES
have room, with no axis change at all, is everything BELOW the yaw cell
tube's rim: the servo case and the GD/Pi board columns both end at that
exact Z (`V.YAW_BOX_BOT_Z`), so the space below it was already open. That
is where this design puts the bearing.

## 3. Bearing choice: size from the geometry, not the other way round

Two earlier passes picked a bearing size FIRST and then tried to make the
carrier or the pelvis fit around it -- both wrong, in the order the
coordinator's review caught them:

1. **6704-2RS (20x27x4), boss around the horn disc.** Bore radius 10 mm vs
   the disc's own 9.6 mm radius leaves a 0.4 mm boss wall -- not printable.
2. **6707-2RS (35x44x5) / 6709-2RS (45x55x6), boss around the servo
   CASE's footprint (15.97 mm corner) only.** Both looked right against the
   case, but the carrier's own horn-mount PLATE -- which doubles as the
   roll bay's ceiling -- is bigger than either the disc or the case: its
   own corner (`hypot(19.95, 15.26)` = 25.12 mm) is what actually sweeps
   through the bearing band, and the bay WALLS below the plate share that
   same footprint for their whole height, not just the top 3 mm. 6707
   failed outright (240 mm3 overlap, `check_assembly_v6.py --joint
   hip_yaw`); 6709 failed at 0.38 mm minimum distance (< the 0.5 mm
   buffer) once trimmed, and a second try that widened the RECESS instead
   of the carrier (a "compromise") left the outer race a 0.75 mm clearance
   fit -- defeating the purpose of a bearing (a race that can shift 0.75 mm
   in its seat is itself ~1.5 deg of tilt lash at that radius, worse than
   what it was meant to remove).

**The right order:** the boss (inner-race seat) must ENCLOSE the carrier's
whole footprint -- plate AND walls, 25.12 mm corner -- with >= 1.5 mm of
wall, so bore radius >= 25.12 + 1.5 = 26.62 mm, bore >= 53.24 mm. Nothing
in the 20-45 mm bore range works; the smallest common size that clears it
is a 55 mm bore.

- **6711-2RS (55x68x7)** was the first candidate at this bore -- but no
  published static/dynamic load rating (C0/C) was found for it across the
  supplier listings checked in this session (dimensions only). Rather than
  fabricate a number, it was dropped.
- **6811-2RS (55x72x9)** -- same 55 mm bore, C0 published consistently
  across sources (6.2-8.4 kN; 8.1 kN used below) -- **this is the pick**,
  at the cost of 2 mm more width and a tighter skirt-to-skirt gap (§4).

### 3.1 Radial (moment) arithmetic

A single-row deep-groove bearing takes an applied moment as a force couple
across its own ball circle, roughly the mean of bore and OD:

```
D_pw = (55 + 72) / 2 = 63.5 mm = 0.0635 m
F(moment) = M / D_pw
  steady  (M = 0.65 N-m):  F = 0.65 / 0.0635 = 10.2 N
  impact  (M = 1.7  N-m):  F = 1.7  / 0.0635 = 26.8 N
```

Combined with the 15 N thrust as an equivalent static load (ISO-76-style
approximation for a single-row deep-groove bearing, P0 = Fr + 0.5 x Fa):

```
P0(steady) = 10.2 + 0.5x15 = 17.7 N   -> C0/P0 = 8100/17.7 = 458x margin
P0(impact) = 26.8 + 0.5x15 = 34.3 N   -> C0/P0 = 8100/34.3 = 236x margin
```

### 3.2 Bore vs the horn screws

Boss radius 27.54 mm clears the O14 bolt circle (r 7) + a self-tap head
(~1.7 mm) by a wide margin (`assert` in `dimensions_v6.py`); the 4 horn
screws are unaffected by any of this (see §5.2).

### 3.3 Why size is geometry-driven, not load-driven

Even the smallest bearing tried (6704, C0 = 0.73 kN by the same catalogue
class) would have cleared these loads by ~9-20x. Every size decision in
this study was about fitting a round bore/OD around the carrier's own
rectangular footprint, not about static load rating -- the final margin
(236-458x) is almost comically large, worth stating plainly so a future
pass does not "upsize for strength" when the actual lever is clearance.

## 4. CAD changes

- **`cad/v6/yaw_carrier_v6.py`** (new; `cad/v6` had no carrier override --
  `parts_v6.py` mapped `yaw_carrier` straight to `parts.yaw_carrier()`,
  "v5 part, unchanged"). Builds on `parts.yaw_carrier()` and adds ONE
  feature, nothing removed or trimmed from the existing part:
  - a boss, radius `V.YAW_BRG_BOSS_R` = 27.54 mm (+0.08 mm interference vs
    the bearing's 55 mm bore -- within the requested +0.05..+0.10 band),
    from the plate's z=0 face down 9.0 mm (`V.YAW_BRG_BOSS_H` = the
    bearing's own width) -- 3 mm of that is the plate's own existing
    thickness, the other 6 mm wraps the top of the bay walls below it
    (they share the plate's exact X/Y footprint the whole way down, so
    wrapping them costs no extra radius). Built as a full disk then
    re-opened, for the part BELOW the existing plate only (z = -9..-3),
    along the SAME x/y footprint the roll bay's own front/rear/cheek walls
    already use -- so the bay's interior (where the roll servo slides up)
    is simply 6 mm deeper at its mouth. The plate's own z-band (-3..0) is
    untouched.
  - **no corner trim** -- the boss's whole point is to enclose the
    carrier's existing 25.12 mm corner (2.42 mm of wall to spare), so
    nothing about the existing plate or bay walls is cut back.
  - Print: **same orientation as v5** (`RX180`, "horn-plate face on bed,
    bay walls rise") -- the boss lives at MORE NEGATIVE local Z than the
    plate's z=0 mating face, the same side the bay walls already rise
    from; no new overhang class.
- **`cad/v6/pelvis_v7.py`**: a skirt hanging from the EXISTING yaw-cell
  tube rim (`V.YAW_BOX_BOT_Z`, not moved) down 10.8 mm total:
  - 1.8 mm shoulder land (ID 69 mm, a 1.5 mm radial ledge -- reacts the
    leg's upward thrust into the pelvis print) -- this exactly reuses the
    gap that was already there (`D.YAW_BOX_CARRIER_GAP`, unchanged). The
    land is kept narrow (1.5 mm, not the more generous 2 mm first used for
    6709) so it cannot reach the bearing's own moving parts even if the
    ESTIMATED internal raceway split below is off by a couple mm.
  - 9.0 mm recess (ID 71.96 mm, -0.04 mm interference -- within the
    requested -0.03..-0.05 band, a real located fit, not a clearance fit)
    for the bearing's outer race, flush with the horn face
    (`V.YAW_HORN_FACE_Z`) at its bottom.
  - Skirt OD 78 mm (~3 mm wall around the recess). Two skirts, 84 mm hip
    separation, leave only **6 mm edge-to-edge** -- too little for two
    independent free-hanging rings, so `pelvis_v7.py` adds a short
    connecting web between them at the bottom (the most compliant point),
    spanning the gap plus 5 mm into each skirt. This is disclosed, not
    hidden: the packaging got tighter going from 6709 to 6811, and a
    bridge is the honest fix rather than shrinking the wall further.
  - **Internal raceway split is an ESTIMATE, not a catalog fact.** No
    source checked in this session publishes a bearing's internal
    ring/ball geometry at this class. `dimensions_v6.py` scales the
    coordinator's own illustrative split for a 6709 (inner ring 2 mm,
    ball/cage gap 1 mm, outer ring 2 mm over a 5 mm total radial span) by
    this bearing's larger span (8.5 mm, factor 1.7x) to get
    `YAW_BRG_EST_INNER_RING_OD` (~30.94 mm) and `YAW_BRG_EST_OUTER_RING_ID`
    (~32.64 mm) -- used only to keep the shoulder land off the moving
    parts, with margin (1.86 mm) chosen specifically to survive being
    wrong. **Verify against the real bearing's datasheet or a caliper
    measurement before cutting metal on the shoulder.**
  - All new constants (`YAW_BRG_*`) live in `cad/v6/dimensions_v6.py`,
    parametric off `D`/`V`, with asserts that the boss encloses the
    carrier's corner with >= 1.5 mm wall, that it clears the servo case,
    and that the shoulder stays off the estimated outer-ring ID.
- Nothing above `V.YAW_BOX_BOT_Z` changes: `TORSO_FLOOR_Z`, `HIP_YAW_Z`,
  `DECK_BOT_Z`, and every joint's Z in the leg chain are byte-identical to
  before this study.

## 5. Checks

Run: `.venv/bin/python cad/v6/pelvis_v7.py`, `yaw_carrier_v6.py`,
`assembly_v6.py`, then `check_assembly_v6.py` (all pairs) and
`cad/v6/check_yaw_bearing_combo.py` -- REWRITTEN after the coordinator
pointed out the first version only checked the printed parts against each
other, never against the bearing's OWN body. It now models the bearing as
two solid rings (inner race: `V.YAW_BRG_BOSS_R` to the estimated inner-ring
OD; outer race: the estimated outer-ring ID to `V.YAW_BRG_RECESS_R`) and
checks, through yaw 0/+-45 CROSSED with hip_roll and hip_pitch at their ROM
extremes:

1. the rotating carrier stays clear of the inner ring except the boss's own
   designed interference,
2. the rotating carrier NEVER touches the outer ring (it only ever meets
   it through the balls),
3. the fixed pelvis stays clear of the outer ring except the recess wall's
   own designed interference,
4. the fixed pelvis NEVER touches the inner ring,
5. `D.SWEEP_BUFFER` (0.5 mm) everywhere else,

plus the original direct carrier-vs-pelvis check as a cross-check. Full
output: `docs/design-v6/yaw_bearing_check.txt`.

### 5.1 The failures that shaped the design (kept so none is retried)

- 6707-2RS, boss around the case footprint only: 240 mm3 overlap, growing
  with yaw angle, between `yaw_carrier_L` and `pelvis_v7` -- the carrier's
  plate corner (25.12 mm) exceeded the recess radius (22 mm).
- 6709-2RS, boss around the case footprint, plate corner trimmed to
  23.05 mm: `check_assembly_v6.py --joint hip_yaw` found only 0.38 mm
  minimum distance (< 0.5 mm buffer) at the SHOULDER (radius 25.5 mm)
  against the still-untrimmed BAY WALLS below the trimmed plate (25.12 mm)
  -- the first fix only trimmed the top slab, not the walls sharing its
  footprint one z-band down.
- 6709-2RS, recess widened to a +1.5 mm clearance fit instead of trimming
  further: passed the geometric sweep, but the coordinator's review
  correctly rejected it -- a 0.75 mm radial clearance at the outer race
  reintroduces the exact lash (~1.5 deg of tilt freedom at that radius)
  the bearing exists to remove.
- **This pass (6811-2RS, boss encloses the WHOLE carrier footprint, both
  races located):** passes both the direct check and the ring-based check
  below.

### 5.2 Screw access and print orientation (asked explicitly, verified)

- The 4 horn screws are unaffected: they still thread from BELOW, through
  the SAME `D.YAW_CARRIER_PLATE` = 3.0 mm plate (z = 0..-3, unchanged),
  reachable through the roll bay's cavity (now 6 mm deeper at its mouth,
  which only helps tool access). The screw heads' seating area (bolt
  circle r 7 mm) is well inside r = 17.35/12.66 mm -- solid, untouched;
  the boss starts beyond r 25 mm, nowhere near it.
- Print orientation for `yaw_carrier_v6`: **unchanged** (`RX180`,
  horn-plate face on the bed, bay walls rise) -- confirmed in §4.

### 5.3 Results (full output in `docs/design-v6/yaw_bearing_check.txt`)

`check_assembly_v6.py` (all 21 joint pairs + 4 inter-leg pairs): **ALL
CLEAR** (81 s); the two pairs this study touches:

```
hip_yaw   yaw_carrier_L      pelvis_v7           0.00 mm3   2.30 mm   at -45 deg   ok
hip_yaw   servo_hip_roll_L   pelvis_v7           0.00 mm3   7.30 mm   at -45 deg   ok
```

`cad/v6/check_yaw_bearing_combo.py` (the ring-based check, 12 combined
yaw/roll/pitch poses): **ALL CLEAR**, but getting there took two more
rounds of the coordinator's review catching real bugs in the CHECK itself,
kept in `docs/design-v6/yaw_bearing_check.txt` so neither is silently
reintroduced:

1. The first version built the two rings at `V.YAW_BRG_RECESS_Z` directly
   -- a PELVIS-LOCAL z range -- while `assembly_v6.robot()` places every
   piece in WORLD coordinates. Every check "passed" with reported
   distances of 400+ mm, which is physically absurd for a joint whose own
   parts are all within ~30 mm of each other; the absurdity of the number
   is what caught it, not a passing grade. Fixed by using
   `(V.HIP_YAW_Z - V.YAW_BRG_W, V.HIP_YAW_Z)`.
2. With the frame fixed, the bridge connecting the two skirts (added
   because 6811's OD leaves only 6 mm between them) showed up as 80 mm3 of
   overlap with the outer ring's own reserved space -- the bridge had
   reached 5 mm into each skirt without checking where that skirt's
   hollow bore actually begins (only ~3 mm of solid wall exists there).
   Fixed by shortening the bridge to stay inside the solid wall.

Final result:

```
part         ring       overlap mm3  min dist  at (yaw,roll,pitch)
yaw_carrier  INNER_RING        0.00      0.50  (0, -55, -125)   ok
yaw_carrier  OUTER_RING        0.00      5.10  (0, -55, -125)   ok
pelvis_v7    OUTER_RING        0.00      0.01  (-45, -55, -125) ok  (*)
pelvis_v7    INNER_RING        0.00      2.30  (-45, -55, -125) ok
yaw_carrier  (direct)          0.00      2.30  (-45, -55, -125) ok
=> ALL CLEAR
```

(*) 0.01 mm here is the INTENDED contact plane, not a near-miss: the
ring's top boundary (world z = `V.HIP_YAW_Z`) is exactly where the
pelvis's shoulder land is designed to touch the outer race's flat top
face -- that is the shoulder's entire job. It is excluded from the
"radial interference" allowance (`FIT_ALLOW`) the same way the boss-to-
bore and recess-to-OD fits are, because it is a different (axial) designed
contact, not a radial one -- both are legitimate zero-clearance seats, not
sweep-buffer violations.

## 6. Sim: yaw play / lash model

**Not changed** (per the task -- the sim plant is untouched). The bearing
replaces the horn-spline+screw load path with a LOCATED bearing seat (both
races interference-fit, no clearance anywhere in the load path) -- the
axial/moment lash that the spline used to carry under load is now reacted
by the bearing (236-458x margin, §3.1), so the physical mechanism the v5
1-2 deg yaw play was attributed to (screws working loose under a
cantilevered moment) is removed for that axis, with no new radial lash
source introduced in its place (unlike the rejected clearance-fit pass in
§5.1). No plant change is proposed -- the mechanism this study targets did
not have a corresponding plant parameter in the first place (the ROM/play
model is generic clearance, not joint-specific).

**Swing-phase load, for the bench build (context, not a retention gap this
time -- both races are located):** the leg mass BELOW the yaw joint
(yaw_carrier print + hip-roll servo + everything down to the foot, from the
CAD mass rollup, `parts_v6.py`) is:

```
29.9 (yaw_carrier_v6, incl. the boss) + 74.5 (hip-roll STS3250) + 11.5 (yoke_roll)
+ 10.9 (yoke_pitch_v6) + 55.0 (hip-pitch STS3215) + 26.3 (thigh)
+ 74.5 (knee STS3250) + 26.3 (shin) + 55.0 (ankle-pitch STS3215)
+ 12.1 (ankle_link) + 74.5 (ankle-roll STS3250) + 52.0 (foot)
+ 23.2 (TPU sole) + ~10 (hardware) = 535.7 g -> 5.26 N static weight
```

No measured swing-phase acceleration exists for this gait (the demo suite
is quasi-static weight-shift/reversal, not a dynamic swing --
`natural-responsive-demo-plan.md`), so a conservative x2-3 dynamic factor
gives 10-15 N -- now carried by the boss's own +0.08 mm interference fit
(not a clearance fit backed by retaining compound), consistent with a
standard bearing installation; still worth a pull-scale check at the bench
before the first hardware run, as with any new press fit in this codebase.

## 7. Print / assembly

- **Boss (carrier):** OD 55.08 mm, +0.08 mm vs the bearing's 55 mm bore --
  within the requested +0.05..+0.10 mm interference band. No existing
  bearing precedent in `dimensions.py` to match (every other joint's "fit"
  is a screw or a horn spline, not a bearing bore); verify and trim after
  the first print the same way `D.FIT` (0.30 mm slip) was tuned for the
  servo case originally.
- **Recess (pelvis):** ID 71.96 mm, -0.04 mm vs the bearing's 72 mm OD --
  within the requested -0.03..-0.05 mm interference band. A real located
  press fit, not a clearance fit.
- **Shoulder:** ID 69 mm, 1.8 mm tall land, 1.5 mm radial width -- prints
  as a small internal step; kept narrow deliberately (§4) against the
  estimated internal raceway split.
- **Bridge:** a short web connecting the two skirts at the bottom (§4) --
  new print feature, needed because 6811's larger OD leaves only 6 mm
  between the two skirts.
- **Assembly order:** bolt the carrier's plate to the horn (4x M3, exactly
  as today) BEFORE the bearing goes on -- the boss stands on the plate's
  OUTER rim, clear of the screw heads, so order does not matter for screw
  access, but doing the horn bolts first keeps the leg supported on the
  bench while the bearing/skirt mate. Press the inner race onto the boss,
  then offer the leg up into the pelvis recess from below and press the
  outer race home against the shoulder.
- **Quantity:** 2x 6811-2RS (one per hip yaw) + 2 spares (the same "buy
  spares" rule as the STS3250 stiffness-bench order in `bom-delta.md`).

## 8. Renders

`cad/v6/render_yaw_bearing.py` renders the yaw joint only (carrier + the
pelvis cell above it), not the whole robot; regenerated for the 6811
design (§9 has the exact status of each file at commit time).

- `cad/v6/renders/yaw_bearing_before.png` -- the pre-study joint (v5
  `yaw_carrier` + the pre-edit `pelvis_v7`, loaded from git HEAD): the cell
  mouth ends flush at the tube rim, nothing below it.
- `cad/v6/renders/yaw_bearing_after.png` -- the same view with the new
  (larger, 6811-sized) skirt hanging below the cell and the enclosing boss
  visible above it -- the clearest evidence of the change.
- `cad/v6/renders/yaw_bearing_section.png` -- a half-space cut through the
  yaw axis, at the housing's own mid-sagittal plane rather than through the
  hip's own axis, so both the near skirt's full recess/shoulder stack AND
  the carrier's boss (orange) sitting inside it are visible; a real
  improvement over the earlier pass's section render (which only grazed
  the outer wall). The shoulder ledge, the recess collar, and the boss
  wrapping the carrier's plate are all legible in this one.
- Fly-in assembly animation: `cad/v6/animate_v6.py`, output gitignored
  (project rule); `cad/v6/renders/assembly_v6_flyin_strip.png` is the
  committed filmstrip (not re-run this pass unless the silhouette changed
  enough to matter -- see the commit for what was actually regenerated).
- `docs/assembly.md` figures: **not regenerated** this pass --
  `cad/render_assembly_steps.py` targets the v5 assembly sequence, not
  the v6/v7 hip-yaw stack; rerunning it would not pick up this change.

## Owed before the sim re-run (coordinator, 2026-09-17)

- **Mass**: two 6811-2RS rings plus the pelvis skirt/bridge print are NOT
  in the plant yet; the bearing's catalogue mass was not verified here
  ("not measured"). It sits at hip-yaw height, the lowest place on the
  torso, but `sim/build_v6_inertia.py --write` must be re-run with the new
  pelvis_v7 STL and the bearing mass as a lump before the walk gate is
  trusted again (the 09-13 rule: sim margins scale with real part masses).
- **Alternative size**: 6711-2RS (55 x 68 x 7) has the same bore, is 2 mm
  thinner and lighter, and would shrink the skirt to OD 74 (10 mm between
  skirts, possibly no bridge); it was not used only because no published
  static rating was found -- irrelevant at these loads. If it is in stock,
  it is the better part; the recess depth/OD are parametric (`YAW_BRG_*`).

## BOM delta

See `docs/design-v6/bom-delta.md` (added there too):

| item | qty | spec filter | why | est. |
|---|---|---|---|---|
| 6811-2RS deep-groove ball bearing | 2 (+2 spares) | 55x72x9mm, sealed (2RS), chrome steel (SAE 52100), deep-groove "68xx" series | hip-yaw thrust + moment, replacing the horn-screw-only load path | not sourced here -- buy to the spec filter, Tom to source per project convention |

## 5-sentence verdict

A 6811-2RS (55x72x9, C0 ~8.1 kN) wraps around the OUTSIDE of the carrier's
ENTIRE horn-plate-plus-bay-wall footprint -- not a boss sized to the disc
or the case alone, both of which are smaller than the carrier's own
rotating cross-section and were tried and rejected first -- with both
races LOCATED by real interference (+0.08 mm / -0.04 mm), no clearance fit
anywhere in the load path. The bearing's own load margin is enormous
(236-458x even at the impact moment); every sizing decision was forced by
enclosing the carrier's 25.12 mm corner with real wall, not by load
capacity. `check_assembly_v6.py` plus a rewritten combined-pose check that
models the bearing as two solid rings (not just the printed parts against
each other) confirm no interference anywhere in the load path through the
full yaw x roll x pitch ROM. The tradeoff for closing the previous pass's
compromise is packaging: 6811 is 2 mm wider than the size that produced
the clearance-fit problem, and the two skirts now need a connecting bridge
at 84 mm hip separation -- disclosed, not hidden. This directly answers
Tom's complaint: the horn screws go back to pure torque, and the leg's
0.335 m lever now loads the pelvis print through a properly located
bearing, not a shaft or a compromise fit.
