# Hip yoke as one print (2026-09-19)

Tom: make the link below the hip-yaw carrier — the hip-roll clevis
`yoke_roll` (v5, `cad/parts.py`) and the hip-pitch clevis `yoke_pitch_v6`
(`cad/v6/yoke_pitch_v6.py`), today screwed flange to flange — ONE print.
Result: `cad/v6/hip_yoke_v6.py`, opt-in via `HIP_YOKE_VARIANT=single`
(default `split`, every existing caller unchanged). Not yet the default; two
pre-existing clearance findings it exposed are open (section 6).

## 1. What was merged

| | split (as built) | single (`hip_yoke_v6`) |
|---|---|---|
| parts per hip | `yoke_roll` + `yoke_pitch_v6` | one solid |
| joint | two 4 mm flanges, 4 × M3×10 button head down through `yoke_roll` into 4 × M3 heat-set inserts in `yoke_pitch` (`fasteners.flange_bolts`) | one 8 mm block, bolt columns filled (+356.5 mm³ = exactly the four O3.4 + four O4.1 bores) |
| servo interfaces | roll horn boss + sunk idler pad on X; pitch horn plate + r14 idler hub on Y | **identical** — the merged solid is `yoke_roll() + Pos(0,0,-ROLL_TO_PITCH) * yoke_pitch_v6()`, the v5 geometry itself, not a copy |
| kinematics | `HIP_ROLL_Z`, `HIP_PITCH_Z`, `ROLL_TO_PITCH` 50, `ROLL_AXIS_TO_FLANGE` 16, `PITCH_ARM_REACH` 26 | unchanged; the part is placed at `HIP_ROLL_Z` alone and carries the pitch clevis at −50 in its own frame |
| hip-flexion chamfer (`YOKE_FLEX_CHAMFER`, [design-v6-ankle-roll §11.3](../design-v6-ankle-roll.md)) | on `yoke_pitch_v6` | kept, via the shared `yoke_pitch_v6.flex_chamfer_wedge()` (also trims the two front bolt fills where the heat-set bores grazed it). `yoke_pitch_v6()` volume unchanged to the last digit after the refactor (9534.799917594863 mm³) |
| envelope | union of the two | the same union — no new material outside either part, so no pair can get *worse* |

Deliberately **not** done: thinning the 8 mm block or trimming the flange
wings that only existed as bolt land (`yoke_roll`'s ±17 mm Y wings beyond the
±12 arm plates; `yoke_pitch`'s ±16 X ends). Both are real savings (~1.6 g per
hip, estimate) but the flange thicknesses set `PITCH_ARM_REACH`, and the wings
are the bridge the arms root into — that is a separate strength decision, and
section 6's flexion finding is *on* the roll flange's front, so its shape
should be decided once, with that.

## 2. Mass (build123d volume, PETG 1.27 g/cm³, × 0.90 print factor as in `parts_v6.py`)

| | volume mm³ | printed g |
|---|---|---|
| `yoke_roll` | 10 037.8 | 11.47 |
| `yoke_pitch_v6` | 9 534.8 | 10.90 |
| **split total** | 19 572.6 | **22.37** |
| **`hip_yoke_v6`** (styled, §7) | 19 905.6 | **22.75** (+0.38) |

Hardware removed per hip (not in the CAD mass; catalogue-typical estimates):
4 × M3×10 button head ≈ 0.9 g each, 4 × M3 brass heat-set (O4.6 × 4–6) ≈
0.35 g each → ≈ 5.0 g. **Net ≈ −4.6 g per hip, −9 g per robot**, and two
part numbers, 8 screws, 8 inserts and one heat-set operation gone from the
build. Whole-robot rollup with the switch: `HIP_YOKE_VARIANT=single
.venv/bin/python cad/v6/parts_v6.py`.

## 3. Print orientation: on edge like `yoke_roll`, supports + brim

The failure this design must not reintroduce: `yoke_roll`'s arms snapped when
printed flange-down, layers stacked *along* the arm, so the cantilever's
bending tension ran across the interlayer bond. The fix (2026-07-30) was to
put the arm length in the bed plane.

In the merged part **both** clevises have their arm length along model Z
(roll arms z −16 → 0, pitch arms z −24 → −50), so any orientation with Z in
the bed plane keeps that tension along the filament for every arm. That
leaves model +X up (`yoke_pitch`'s `RY_XUP`) or model +Y up (`yoke_roll`'s
`RY_ROLL_WALL`). The 45° diagonal, which would let both arm sets lean at
exactly the overhang limit, stands the part on one edge — out.

Either way one clevis prints as walls and the other as horizontal slabs
(roll plates are normal to X, pitch plates normal to Y). A slab arm is sound
against the recorded failure: its layers are planes *containing* the arm
length, so in-plane and out-of-plane bending both load the filament, and the
only interlayer-tension direction (a pull along the plate normal) is the bolt
preload, i.e. compression onto the disc with the servo body as the spacer.
So the choice is print quality, and **+Y up** wins on every count measured
(`check_printability.py`, `RY_ROLL_WALL`):

- first layer **823 mm² (23.0 %)** after the styling pass of §7 (855 mm² /
  23.9 % before it) — the pitch idler arm's r14 hub + riser and the pitch
  flange end lie flat on the bed — vs ≈ 360 mm² (11 %) for +X up;
- height 43.85 mm vs 48.4;
- the roll arms (the ones that broke) print in exactly their proven
  orientation; the 26 mm pitch idler arm (1.6 × a roll arm's root moment)
  prints bed-flat;
- audit: **PASS**, no THIN findings. Slicer supports required for: the pitch
  *horn* arm, a 713 mm² horizontal slab 40.9 mm up over the pitch-servo void
  (its underside is the horn seating face — a support-interface finish on a
  face clamped to a metal disc by four screws; +X up would put the same
  finish on the *roll* horn boss instead, so this is a wash); the roll pad
  rims and arm plates that start in mid-air (as in the `yoke_roll` print);
  the roll flange's −Y face 3.4 mm above the bed (194 mm² island). Brim: a
  44 mm-tall T of 4 mm walls. The horizontal O3.4 bolt-circle bores (the roll
  pads') are a 4 mm-class roof, a note not a fail.

## 4. Checks

Full ROM sweep, `cad/v6/check_assembly_v6.py`, 5 samples per joint, both
variants (verbatim in [`hip_yoke_check_single.txt`](hip_yoke_check_single.txt)
and [`hip_yoke_check_split.txt`](hip_yoke_check_split.txt)).

**Switch off** (post-change baseline): `=> ALL CLEAR (54 s)`, every row
identical to the pre-change run. Default `robot({})` fingerprinted against
HEAD's code: 33 pieces, same labels, volumes and bounding boxes.

**Switch on**: `=> 2 FAIL (54 s)`. The rows that changed:

```
hip_roll     hip_yoke_L             yaw_carrier_L        0.00   0.30    0.0  ok    idler boss rides the bore, volume only
hip_roll     hip_yoke_L~seats       yaw_carrier_L        0.00   1.00  -55.0  ok    minus the disc seats: arms, pads, both flanges get the full buffer
hip_roll     hip_yoke_L             servo_hip_roll_L     0.00   0.00  -55.0  ok    seated on the roll servo discs, must not intersect
hip_roll     hip_yoke_L~seats       servo_hip_roll_L     0.00   0.08  -55.0  FAIL
hip_pitch    thigh_L                hip_yoke_L         271.70   0.00 -125.0  FAIL  thigh grip plates vs the yoke arms
```

With the switch on the check is **stricter** than the split one: the two
pairs where the part is designed to touch (carrier bore, roll servo discs) are
checked whole *and* minus `hip_yoke_v6.disc_seat_zones()` under the full
0.5 mm buffer, and the thigh pair now covers the roll half as well as the
pitch half. Both FAILs reproduce on the **split** geometry (`/tmp` diag, roll
frame, same mock):

| finding | split `yoke_roll` | `hip_yoke_v6` |
|---|---|---|
| thigh at hip −125 | ∩ yoke_roll **271.70 mm³**; ∩ yoke_pitch 0.00 (0.70 mm) | ∩ hip_yoke 271.70 mm³ |
| thigh, fine sweep | −114: 2.41 · −115: 1.84 · −116: 1.32 · **−117: 0.78** · −118: 0.24 · −119: 3.87 mm³ · −120: 29.59 mm³ | same solid |
| minus seats vs roll servo, ±55 | 0.00 mm³, **0.08 mm** | 0.00 mm³, 0.08 mm |

So: **hip flexion clearance to the thigh is not worse than before — it is
the same solid** (the pitch-half chamfer still gives 0.70 mm at −125), but
the −125° figure was only ever proven against `yoke_pitch`; `thigh_{s}` vs
`yoke_roll_{s}` was never in the v6 PAIRS list.

Re-run after the styling pass of §7: **the same two FAILs and nothing new**
(`=> 2 FAIL (57 s)`). The −125° row reads 273.85 mm³ rather than 271.70 —
accounted for to the mm³ in §7.4.

## 5. Exports and figures

- `cad/v6/stl/hip_yoke_v6.stl`, `cad/v6/step/hip_yoke_v6.step` (from the
  module's `__main__`, which also runs the audit and the mass comparison)
- `cad/v6/renders/hip_yoke_v6.png` — iso / bottom / back; the bottom view
  shows the solid cross where the bolt land was
- `cad/v6/renders/assembly_v6_flyin_hipyoke_strip.png` — fly-in with the
  switch on (`HIP_YOKE_VARIANT=single MUJOCO_GL=cgl cad/v6/animate_v6.py`;
  31 pieces in 14 groups; the yoke takes the pair's path, lowered onto the
  pitch-servo discs from above in group 6; the .mp4 is gitignored)

## 6. Open items (design decisions, not taken here)

> **Resolved 2026-09-24 — `hip_yoke_v6` is now the DEFAULT** (Tom: *"didn't we
> combine the two parts of the hip joint?"* … *"yes, go ahead. We need to re-run
> the getup with these arms anyway."*). Items 1–3 below are closed as follows;
> item 4 (bench) stands.
>
> **Item 2 — roll servo corner, 0.08 → 0.58 mm at ±55° roll.** A 0.5 mm relief
> (`hip_yoke_v6.ROLL_CORNER_RELIEF`) on the roll flange's top, *only* across the
> servo case's own thickness (x −17.15..+18.98, its idler boss to its horn rib,
> ±0.5), so both roll arm plates' root fillets — the roll clevis's load path —
> are untouched and `ROLL_AXIS_TO_FLANGE` / `PITCH_ARM_REACH` / every v5 part
> stay as they were. Servo interfaces IDENTICAL, printability PASS, −0.74 g.
>
> **Item 1 — hip flexion: −125 was never reachable; the ROM is now −120.**
> Measured, the clash is not the thigh's grip top but its **front wall** (the
> leg link box's top edge, thigh-local x 11–16, z −33..−41), meeting the roll
> flange's front from −118.4° in *both* variants. Clearing −125 would have meant
> a 5.7 mm cut straight through that 2.6 mm wall (option c) — the thigh's
> sagittal-bending flange where the hip moment peaks, and the same hole under
> the knee on the shin — or thinning the **roll horn arm's root** from 3.0 to
> ~1.1 mm (option b). Option a was tested in sim first: the get-up **needs the
> tuck at ≥ 119°** (stands at −119/−120/−121/−125, and at −118/−117 not once in
> 84 variants incl. broader ankle/knee/push-time grids —
> `getup_search_r5_asdrawn_hip*.txt`). So: `leg_link_v6` gets `LL_HIP_RELIEF`
> (a 3.5 × 4 mm chamfer on the box's top-front edge + the idler jog block's
> sliver forward of the servo case trimmed), which takes the thigh to **0.70 mm
> at −120°**; past that the thigh's own servo case is next, so −120 is the
> limit without redesigning the joint. `ROM["hip_pitch"]` −125 → **−120**. The
> get-up at the real limit (plant capped at −120): 2/12 variants stand, both
> 6/6 robust. The pair `("hip_pitch", "thigh", "yoke_roll")` now also runs in
> the legacy split sweep, so this can't hide there again.
>
> **Item 3 — promoted**: `HIP_YOKE_VARIANT` defaults to `single`; the parts
> rollup and `print-list.md` carry `hip_yoke_v6` × 2 in place of `yoke_roll` +
> `yoke_pitch_v6` (−2 part numbers, −8 M3×10 flange bolts, −8 heat-sets).
> `HIP_YOKE_VARIANT=split` still builds the bolted pair.
>
> **Still open:** the *walk* plant's `hip_pitch_range` is still (−125, 90). The
> walk policy's action scaling is tied to that range, so bringing it to −120 is
> a training-side change for the next night round, not a CAD one.

1. **Hip flexion is limited by the ROLL flange, at ≈ −117°, in both
   variants.** The thigh's grip top reaches the roll flange's front face
   (x 19.8..24.5, full ±17 width, z −20..−13 — the flange front *and* the
   horn-arm root) from −119°. The `ROM["hip_pitch"]` lower bound of −125 and
   the plant regenerated from it (§11.3) rest on the pitch-half check alone.
   Options: (a) pull the ROM back to −117 (buffer met at 0.78 mm) and
   regenerate the plant; (b) relieve the roll flange's front — but the
   overlap runs into the horn-arm plate root, which is the roll clevis's
   load path, so this is a strength call, best made together with the
   wing-trim in section 1; (c) shorten the thigh's grip top. Whatever is
   chosen, `("hip_pitch", "thigh_{s}", "yoke_roll_{s}")` should join the
   split PAIRS list so the gate covers it.
2. **Roll flange top face vs the roll servo's output-end corner: 0.08 mm at
   ±55° roll.** `ROLL_AXIS_TO_FLANGE` 16.0 was justified against the bay-wall
   bottoms at ±25°; the 2026-08-03 widening to ±55° made the servo's own case
   corner (r = √(12.36² + 10.11²) = 15.97) the binding feature. The mock is
   square-cornered where the real case is radiused, so the physical gap is
   larger, but it fails the 0.5 mm rule and the split check never listed the
   pair. Options: 0.5 mm off the flange top (moves `ROLL_AXIS_TO_FLANGE`,
   hence `PITCH_ARM_REACH`), or accept with a measured corner radius.
3. Promote the switch to default (parts_v6 rollup, print list, BOM: −8 M3×10,
   −8 heat-sets, −2 part numbers) once 1 and 2 are decided; then the
   `yoke_roll` / `yoke_pitch_v6` rows in `print-list.md` become this part.
4. Bench: print one, confirm the supported horn-seating face of the pitch
   horn arm seats flat on the disc (support interface Z gap 0.2 mm), and
   that the on-edge part with a brim does not tip during the pitch horn slab.

## 7. Styling pass (2026-09-19)

Tom, on the part above: *"This 'merged' part just looks like the two parts are
just slapped together... functional but not pretty. Let's make it more organic
and streamline all of the sharp corners as much as possible."*

He is right, and section 1 says why: the merged solid was a **pure boolean
union**, so wherever the two flange footprints disagreed it kept a bare step.
They disagree everywhere, because the two clevises are rotated 90° to each
other:

| | fore/aft (x) | across (y) |
|---|---|---|
| roll flange (z −20..−16) | −23.95 .. 24.45 (48.40) | ±17.00 (34.00) |
| pitch flange (z −24..−20) | ±16.00 (32.00) | −20.40 .. 23.45 (43.85) |

In plan the 8 mm block is a **plus sign**: the roll flange overhangs fore and
aft by 8.45 / 7.95 mm (a bare downward ledge at z −20), the pitch flange
overhangs on ±y by 6.45 / 3.40 mm (a bare upward ledge on the same plane).
That plane — the old bolted joint — is what read as "slapped together".

### 7.1 What the blend actually is

Everything is in `cad/v6/hip_yoke_v6.py`; `fused()` is the old solid, kept as
the A/B reference, and `hip_yoke_v6()` is `fused()` + the styling pass.

1. **The waist.** A cove (`R_WAIST` 2.75) on each of the four reentrant lines
   at z −20: the roll flange's underside into the pitch flange's two side
   faces, and the roll flange's two side faces into the pitch flange's top.
   The step becomes a continuous curved transition from one footprint to the
   other — this is the blend, and it is why the block no longer reads as two
   plates stacked.
2. **The silhouette.** The block's six vertical corners at `R_CORNER` 4.5
   (the pitch flange's front two are already the flexion chamfer's diagonal,
   which gets `R_PLATE` 2.0), every horizontal flange rim at `R_BROW` 1.0,
   and the arm plates' outline corners at `R_PLATE` 2.0.
3. **Arm roots** — structural, not cosmetic; this is the cantilever that
   snapped in the field. `R_ROOT` 2.5 where there is room: both roll plates'
   side roots, the pitch horn arm's three roots, the idler riser's three.
   `R_ROOT_SERVO` **0.6** on the two roots that face a servo case: the roll
   servo's corner sweeps 0.08 mm over the roll flange top at ±55° of roll and
   the bay walls run 1.0 mm off each roll arm's inner face, so a full-size
   fillet there would spend clearance the joint does not have.
4. **Shoulder tapers**, `leg_link_v6`'s corner-cut idiom: each arm plate is
   12 mm half-wide over a r 10 pad, so the two corners where the rectangle
   oversails the pad carry nothing. Taper 1.7 × 3.0, apex 0.3 mm outside the
   pad radius so no servo interface is touched. **Not on the roll idler
   arm**: behind its sunk pad (`ROLL_IDLER_PAD_SINK` 1.35 of 3.0) that plate
   is already a 1.65 mm floor, and tapering its shoulder too left a feather
   edge where the cut ran tangent to the sink — `check_printability` caught
   it as 2.6 mm² of wall under 0.85 mm, which is a wall this part is not
   allowed to grow. Its shoulders stay square.
5. **`_flex_relief()`**, last: the flexion chamfer's own plane re-cut across
   the flange band, so the front cove dies into the chamfer instead of
   bulging past it into the thigh's sweep.

Print orientation is unchanged (`PRINT_ORIENT` / `RY_ROLL_WALL`, on edge,
model −Y on the bed) and edges **lying in the bed plane are chamfered, not
rounded** — a fillet there starts horizontal at the first layer, an
unsupported 90° overhang around the part's own bed contact, where a chamfer
is a printable 45°. First-layer contact is still 823 mm² and
`check_printability` still passes.

### 7.2 The fillet helper, and why it is one edge at a time

`_round_edges()` walks `_groups()` — (name, radius, predicate) — re-querying
the edges from the *current* solid each time, so nothing holds a stale edge
reference. Each predicate pins **both** faces' planes (`_on(e, y=..., z=...)`),
never one: a fillet leaves a tangent line one radius away on each face, and a
one-plane test happily re-selects that line on the next group, which cannot be
filleted and shows up as a phantom failure. Each edge gets its target radius,
then 0.7 / 0.5 / 0.35 / 0.2 of it, then the same ladder as a **chamfer**
(`leg_link_v6`'s fallback); an edge that survives none of it is skipped **and
reported** with location and length.

Two things were measured rather than assumed:

- **Group fillets are out.** Filleting a group in one call blends the corners
  where its edges meet and is the prettier result, but on this solid it does
  not merely fail — OCC **segfaulted** on the four pitch-flange bottom rims
  (exit 139, no STL, no traceback). A crash cannot be caught in process, so
  the group path was deleted. `HIP_YOKE_TRACE=1` prints a flushed breadcrumb
  per edge for the next time this happens.
- **Order matters, and the order is corners-then-rims.** Rims first leaves
  each 4 mm corner trimmed to 1.2–2.6 mm between two blends and OCC then
  refuses it at every radius: 4 of the 6 block corners were lost that way.
  Corners first costs only the corner *arcs*, which a later group picks up.
  `R_BROW` is 1.0 rather than the 1.4 a 4 mm flange could take, for the same
  reason: at 1.4, four of the block's rims were dropped outright.

### 7.3 Edges left unrounded (5)

| edge | at | length | why |
|---|---|---|---|
| block corner arcs ×2 | (−21.92, −14.97, −20.00) and (−21.92, −14.97, −16.00) | 5.50 mm | where the roll flange's rear −y corner blend meets the top and waist planes. Refused as fillet and as chamfer at every radius down to 0.2 mm. Cosmetic: the corner itself is R4.5, this is the crease around it. |
| roll arm plate corners ×2 | (−20.95, ±12.00, −7.70) | 15.40 mm | the roll **idler** arm's two inner-face corners. Their outer twins at x −23.95 are rounded, so the plate reads slightly one-sided. Both faces here are free space (1.0 mm to the bay wall), so this is finish, not fit. |
| pitch idler hub rim | (−4.79, −20.40, −63.16) | 53.76 mm | the r14 hub's outer rim — the part's **bed contact face**. Refused everything, including chamfer-first. Leaving it square is the benign failure: it is the first layer, and a sharp rim there is what the brim wants anyway. |

Deliberately **not** attempted, and not in the list because they are not
failures:

- **the roll pads' rims.** With the bolt circle at r 7 (`BCD` 14), an M3
  head's edge lands at r 10.0 — the pad's own radius. Any round there
  undercuts a fastener seat.
- **the pitch horn plate's inner corners** (y 20.45): that face is the horn
  **disc seat**.

`check_interfaces()` is the hard gate on all of that: it intersects both the
raw and the styled solid with `_interface_zones()` (r 10.2 cylinders on all
four disc axes — pads, bolt circles, bosses, sunk idler pad, centre reliefs)
and requires the volumes to match. They do, to 6 × 10⁻⁴ mm³ of 3908.9557.

### 7.4 Mass, and every clearance that moved

| | volume mm³ | printed g |
|---|---|---|
| `fused()` (as shipped at `afd4daf`) | 19 929.1 | 22.78 |
| `hip_yoke_v6()` styled | 19 905.6 | 22.75 |
| | **−23.5** | **−0.027** |

The blends add 116.0 mm³ (coves and root fillets), the rounds and tapers take
139.5 mm³ back.

Clearances, measured in the yoke's own frame against the same mocks
`check_assembly_v6` uses (link 1 = carrier + roll servo rotated by −hip_roll,
link 3 = thigh + pitch servo rotated by +hip_pitch; the probe reproduces the
gate's numbers exactly):

| | fused | styled |
|---|---|---|
| roll servo vs yoke−seats, ±55° | 0.076 mm | **0.076 mm** |
| bay walls vs yoke−seats, ±55° | 1.000 mm | **1.000 mm** |
| thigh, first contact in flexion | −118.40° | **−118.40°** |
| thigh at −117° | no contact, 0.700 mm | **no contact, 0.700 mm** |
| thigh at −121° | 78.71 mm³ overlap | **77.31 mm³** (better) |
| thigh at −125° | 271.70 mm³ overlap | **273.85 mm³** (+2.15) |

(The constant 0.700 mm is the thigh grip plate vs the yoke arm gap, which
does not vary with the angle; deep flexion shows up as overlap volume, which
is why the table reports both.)

The one number that moved the wrong way is −125°, by 2.15 mm³ on 271.70 —
**+0.8 %, entirely the horn arm's root fillet**. Located exactly: at −125°
the only styling material inside the thigh is the 3.71 mm³ lump at
x 21.45..24.45, y 12.00..14.50, z −16.00..−13.50, which is the R2.5 fillet at
the roll **horn** arm's +y root; at −119° and −121° *no* added material is
inside the thigh at all (0.00 mm³). So the pose that got worse is 7° past the
part's own hard contact at −118.40° and 6° past the gate's first FAIL, in a
pose where both the old and the new solid are already deep inside the thigh —
and the cost is the root fillet on the arm that snapped in the field, which
is the one blend in this pass that is not cosmetic. It is called out here
rather than quietly traded; if section 6's open item 1 is settled by
relieving the roll flange front, this goes with it.

## 8. One plate, not two stacked ones (2026-09-19)

Tom, on the styled-vs-fused A/B in FreeCAD: *"not much difference... at least
combine the two former baseplates into one."*

Correct, and §7's rounding could not have fixed it. **The two flanges are a
plus sign in 3D, not in plan.** The roll flange owns the top band (z −20..−16)
and runs x −23.95..24.45 by y ±17; the pitch flange owns the bottom band
(z −24..−20) and runs x ±16 by y −20.40..23.45. On every side of the old
bolted plane one flange oversails the other and leaves a ledge, and no amount
of edge rounding removes a ledge — it only rounds its lip.

### What the ROM allows

Each candidate fill was measured **on its own**, against the thigh, the pitch
servo, the roll servo and the yaw carrier, over the whole ROM of both hip
joints, before any of it was drawn:

| notch | fill | worst overlap, whole ROM | nearest miss |
|---|---|---|---|
| rear, bottom band | 1081.2 mm³ | **0.00** | 4.26 mm (roll servo @ +55) |
| +y, top band | 825.6 mm³ | **0.00** | 7.50 mm (roll servo @ 0) |
| −y, top band | 435.2 mm³ | **0.00** | 5.58 mm (thigh @ −118) |
| rear hull corners | 313.2 mm³ | **0.00** | 8.33 mm (carrier @ −55) |
| **front, bottom band** | 1482.1 mm³ | **47.4 mm³ into the pitch servo @ −118.4, ~411 @ −125** | — |

So three of the four ledges are simply free, and they are now filled: the
flange band is **one prismatic outline through the full 8 mm** from the rear
face forward to x = +16, on all three of those sides.

### Why the front keeps its step

The front is the corner the hip-flexion chamfer exists for, and it refuses
every form of filling:

- **square fill** — 47.4 mm³ into the pitch servo at −118.4°, ~411 at −125
- **sloped ramp** leaning away from the sweep (the obvious dodge: lean the
  face back so the thigh passes outside it) — still 189 mm³ at −125
- **chamfering the roll flange back** instead of filling under it, which would
  cost mass rather than add it — not available: the material that would have
  to come off is the roll **horn arm's own root**, the load path that broke in
  the field

So the front step stays, and `_flex_relief()` carries the chamfer plane across
the whole band there so it reads as the deliberate relief it is.

### Cost

| | volume | mass |
|---|---|---|
| fused (raw union) | 19 929.1 mm³ | 22.78 g |
| §7 styled | 19 905.6 mm³ | 22.75 g |
| **one plate, styled** | **22 866.5 mm³** | **26.14 g** |

**+3.39 g per hip, +6.8 g the pair** — 0.4 % of the 1.65 kg robot, at the hip,
close to the CoM.

### It did not cost clearance — it bought some

Every gate is unchanged or better than the *raw fused* part:

| | fused | one plate |
|---|---|---|
| roll servo vs yoke−seats ±55° | 0.076 | 0.076 |
| bay walls ±55° | 1.000 | 1.000 |
| flexion first contact | −118.398° | **−118.398°** |
| thigh @ −119° | 3.870 mm³ | **3.451** |
| thigh @ −121° | 78.706 mm³ | **72.525** |
| thigh @ −125° | 271.698 mm³ | **268.752** |

`check_interfaces()` 3908.9557 → 3908.9551 mm³ (IDENTICAL). `check_printability`
PASS, and the first-layer contact **grows 823 → 1048 mm²** (29.3 % of the
bbox), which is a straight win for a 44 mm-tall part on a brim.
`HIP_YOKE_VARIANT=single check_assembly_v6.py` → the same 2 pre-existing FAIL
rows, nothing new.

### One OCC trap, recorded

Cutting `_flex_relief()` only *after* the blends — where it belongs, per §7 —
hits a boolean glitch once the one-plate fill is present: the cut returns the
**right volume** but leaves the **tool's own envelope** behind as material, so
the part's bbox silently grows to the wedge's margins (x to RX1+2, y to
PY0−1..PY1+1). That is 2 mm of phantom flange sitting directly in the thigh's
path, and *volume alone would never have caught it* — only the bbox did. The
fix is to cut once before the blends and once after; the second cut still
earns its keep, taking the 3.1 mm³ of front waist cove that blends out past
the chamfer plane. **Check the bbox, not just the volume, after every boolean
on this solid.**

### Edges still not rounded (6)

The five from §7, plus two new ones created by the fill: the front step's own
top-rim ends at (16.00, −18.70, −20.00) and (16.00, 19.22, −20.00), 3.40 and
4.45 mm, where the new plate outline runs into the surviving front ledge.
