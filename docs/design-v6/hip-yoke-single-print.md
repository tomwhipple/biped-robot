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
| **`hip_yoke_v6`** | 19 929.1 | **22.78** (+0.41) |

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

- first layer **855 mm² (23.9 %)** — the pitch idler arm's r14 hub + riser and
  the pitch flange end lie flat on the bed — vs ≈ 360 mm² (11 %) for +X up;
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
