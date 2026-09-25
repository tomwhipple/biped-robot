# The stereo-periscope head (2026-09-24): binocular vision from one camera

Tom, 2026-09-24:

> Let's redesign the head such that we can have binocular vision with a single
> camera by means of two horizontal periscopes. Field of vision should be
> horizontally maximized and eye openings should be in the same horizontal
> plane.

and, answering the clarifying questions:

> - toe-out: **balanced** (overlap about a third of the total field)
> - baseline: *the total head should not be wider than the shoulders. As long
>   as that conforms, do that* — i.e. as wide as fits
> - *We're using the raspberry pi camera module. Recommend any lenses and
>   mirrors as needed, with purchase links.*
> - *Ideally we can order something that doesn't require cutting that could be
>   glued into the PETG.*

This note is the record of that design: the optical layout and why it is the
one it is (§2 is the trade study — three layouts were traced, not guessed), the
parts to buy (§3), what was **measured** in CAD and sim (§4), how to build and
align it (§5), and what is still open (§6).

Source of truth: [`cad/v6/periscope_optics.py`](../../cad/v6/periscope_optics.py)
(the 3-D ray trace and the design search; the top/bottom alternative is
[`cad/v6/periscope_tb.py`](../../cad/v6/periscope_tb.py), §7) and
[`cad/v6/head.py`](../../cad/v6/head.py) (the CAD, every optical number derived
from the trace). Renders: `cad/v6/renders/head.png`, `head_fields.png`.

---

## 1. The design, in one table

| | old head (2026-09-14) | stereo head (this note) |
|---|---|---|
| camera | Camera Module 3 **Wide**, looking forward | the same camera, looking **aft** into a prism |
| vision | mono, 102° × 67° | **stereo**: two eyes, each **38° H × 40° V** (−30…+10°) |
| total horizontal field | 102° (mono) | **56°**, of which **20°** is seen by both eyes |
| baseline | — | **63.5 mm** between the virtual pupils (human: ~63) |
| eye openings | one lens hole | two windows, **same horizontal plane**, either side of a camera "nose" |
| envelope | 62 × 50 × 60 mm | **140 × 50 × 66 mm** (shoulders are 200 mm) |
| mass | 39 g | **111 g** (PETG 60 + glass 47.6 + camera 4) |
| prints | shell + face plate | **shell + lid + camera sled** |
| optics to buy | — | 1 knife-edge prism + 2 first-surface mirrors, ~$262 (§3) |

How it works, seen from above (+X forward):

```
        eye window L    | nose |    eye window R          <- the face
     ___________________| CM3  |___________________
     \\  mirror L        \\  |  /        mirror R  //
      \\                   \\|/                   //       chevron walls = the
       \\                 prism                  //        mirror mounts
        \\______________ [pad] ______________//            <- back wall
```

The camera looks **aft** into a 20 mm knife-edge right-angle prism mirror (two
coated legs meeting at a sharp 90° apex, 2.8 mm behind the lens's entrance
pupil). The apex cuts the image down the middle: each half-image is thrown
sideways to a 50 × 50 mm first-surface mirror glued to the inside of that
side's chevron wall, which turns it forward out through the eye window. Two
reflections per eye, so each eye's image is **rotated, never mirrored**, and
each eye sees from a *virtual* pupil behind its mirror — the two virtual pupils
are the stereo pair.

The sensor's long axis is split, so the left half of the sensor is one eye and
the right half the other: at full resolution, each eye is 2304 × 2592 px of the
4608 × 2592 frame (less the unused outer edge, §2.1).

---

## 2. Why this layout — the trade study

All of this is `periscope_optics.py`: a 3-D sequential trace of rays from the
pupil through the prism and outer mirror, with every stock aperture, the camera,
the printed nose and the other eye's glass as obstacles; the usable field is the
largest az × el rectangle **every** direction of which reaches the sensor.

### 2.1 No stock splitter gives more than ~45° per eye

A 90° knife-edge prism's leg is only reached by rays less than 45° off the apex
plane — at 45° a ray runs parallel to the leg. So each eye's horizontal field is
**capped below 45° by the prism, not by the lens**: the Wide lens's ±51° half
field cannot all be folded, and the outer ~10° of each side of the sensor goes
unused. Every stock knife-edge prism is 90°. Butting two flat mirrors at a wider
angle would reach further, but their glass edge thickness sits in the field — a
dead band several degrees wide at each eye's nose edge, which is exactly where
the stereo overlap lives. Rejected.

### 2.2 Three ways to hang the camera

| layout | what limits it | per eye (traced) |
|---|---|---|
| camera **forward**, prism in front, mirrors forward-outboard | the half-beams leave the prism already angled *forward*, so each outer mirror turns them through < 90° and the outer rays arrive at ~10° grazing: 38° per eye needs an **84 × 74 mm** mirror | 30° with 50 mm glass |
| camera **up**, sensor's short axis horizontal (each eye would get the full 67° horizontally) | the outer mirror has to turn the beam sideways *and* level it; that rolls each eye's image 20–40°, and the usable level rectangle collapses | **43° × 35° even with unlimited mirrors** |
| camera **aft** (chosen) | the half-beams leave the prism angled *backward*; the outer mirrors turn them through ~120° at near-normal incidence, so they stay small; every mirror is (near) vertical, so there is no roll | **38° × 40°** with 50 mm glass |

### 2.3 The stock sweep (camera aft, vertical field fixed at −30…+10°)

Each row is a random search plus coordinate descent over apex distance, prism
offset, mirror distance, toe-out and aim; objective = per-eye field + 0.1 × baseline
(1° of field is worth 10 mm of baseline), toe-out held "balanced".

| prism leg | outer mirror | per eye | total / overlap | baseline | glass |
|---|---|---|---|---|---|
| 10 mm | 50 × 50 | 34° | 50° / 17° | 68 mm | 38.8 g |
| 12.5 mm | 50 × 50 | 36° | 52° / 19° | 68 mm | 40.0 g |
| **20 mm** | **50 × 50** | **38°** | **56° / 20°** | **63.5 mm** | **47.6 g** |
| 25 mm | 50 × 50 | 37–38° | 54–56° / 20° | 62–67 mm | 57.1 g |
| 20 mm | 40 × 57.5 | 37° | 56° / 18° | 55 mm | 44.5 g |
| 20 mm | 35 × 50 | 35° | 54° / 16° | 50 mm | 36.3 g |
| any | 25 × 25 | 27° | 48° / 6° | 34 mm | — |

The field saturates at the 20 mm prism (the 25 mm one traces the same 38° at
twice the mass), and mirror size buys **baseline**: the 50 × 50 is the largest
stock square, and at 63.5 mm the head is still 60 mm narrower than the
shoulders. A bigger mirror (50 × 75, 28 g each) buys baseline, not field.

### 2.4 Two things the trace forced

- **The outer mirrors lean 2.3° from vertical.** A vertical mirror costs
  9° per eye: at each eye's inner edge the lens's own field only reaches ~27°
  below the horizon, and the tilt rotates each eye's view down just enough to
  keep the full −30° window.
- **The eyes sit 47 mm above the horn face, not 40.** At 40.5 the −30° cones
  sliced the socket floor into knife-edge slivers out at the face; at 47 they
  clear the whole floor, so the floor stays whole — stiff, which the stereo
  calibration needs more than it needs the 6.5 mm.

---

## 3. Parts to buy

No extra lens: the Camera Module 3 Wide's own lens is used as-is. Every optic
below has a manufacturer spec page and drawing; every URL was opened on
2026-09-24 and lands on the product. Prices are USD list, quantity 1.

| qty | part | why this one | price | mass |
|---|---|---|---|---|
| 1 | **Edmund Optics #49-414** — 20 mm enhanced-aluminium, leg-coated N-BK7 90° "specialty mirror" (knife-edge right-angle prism) — [product page](https://www.edmundoptics.com/p/20mm-enhanced-aluminum-coated-n-bk7-90deg-specialty-mirror/9848/) | smallest prism that reaches the full 38° per eye; R > 95 % 450–650 nm, λ/8, drawing + STEP | $170 | 10 g |
| 2 | **Edmund Optics #43-876** — 50 × 50 × 3.0 mm enhanced-aluminium 4–6λ first-surface mirror — [product page](https://www.edmundoptics.com/p/50-x-50mm-enhanced-aluminum-4-6lambda-mirror/5362/) | largest stock square → widest baseline; R ≥ 95 %; stock size, no cutting | $45.75 ea | 18.8 g ea |
| — | Raspberry Pi **Camera Module 3 Wide** (already on the robot) — [product page](https://www.raspberrypi.com/products/camera-module-3/) | 120° diagonal / 102° × 67°; the product page links the mechanical drawing | — | 4 g |

Glass total **$261.50**. Alternatives, both traced:

- **Thorlabs [MRAK25-G01](https://www.thorlabs.com/item/MRAK25-G01)** ($156.35, 25 mm, 19.5 g) —
  a specified *sharp, bevel-free* apex. Edmund says only "bevel: protective as
  needed", so their apex may carry a small chamfer. At 2.8 mm from a 1.25 mm
  pupil that only dims the seam a little (the pupil sees the bevel as a partial
  obstruction; nothing goes blind), so the Edmund part is the default. If the
  seam turns out worse than expected, the MRAK25 traces to the same 37–38° —
  change `STOCK` in `periscope_optics.py` to 25 mm and re-run `--search`.
- **Edmund #89-495** — the same 50 × 50 in protected silver (R > 98 %), $55.50.

Rejected: 1.1 mm Knight Optical mirrors (lighter, cheaper, but no drawing and
no flatness spec — fails the documentation rule), Edmund 0.1 mm "ultra-thin"
mirrors (unspecified flatness; they take the shape of whatever they're glued to,
and printed PETG isn't flat), hollow roof mirrors (mirrored on the *inside*,
they retro-reflect; they cannot split an image).

Hardware: 4 × M3×6 (horn, as before), 4 × M2.5×8 flat-head self-tap (lid),
4 × M2×5 self-tap (camera to sled).

---

## 4. What was measured

All from `cad/v6/head.py` (run it) and the v6 assembly gate.

| check | result |
|---|---|
| nothing printed in any light path (every part vs the beams at a tighter margin than they were cut with) | **0.00 mm³** for shell, lid and sled |
| glass seated: prism, both mirrors, camera vs the shell | **0.00 mm³** |
| horn screws: a 7 mm driver reaches all four from above (lid off, sled out) | **PASS** ×4 |
| neck ±90° vs the neck servo and the deck | **clear** |
| `check_assembly_v6 --joint neck` (ARMS=1) | **ALL CLEAR**; head vs girdle **2.10 mm** (≥ 1.5 by design) |
| `check_assembly_v6` (ARMS=1, every joint, on `main` 6bad5d5 with the one-print hip yoke and the box arm head) | **ALL CLEAR**; upper arm vs head **31.5 mm** at the fold-up, folded forearm **70.9 mm** |
| self-view: robot parts inside the eyes' fields (to 600 mm), standing, arms hanging | **none** (a 50 mm control cube ahead: fully seen; arms raised forward, shoulder −90°: both arms seen — it can watch its hands) |
| printability (`audit_torso`: shell base-down, lid flat, sled on its face) | **all three PASS**, no supports |
| mass | shell 50.2 + lid 7.8 + sled 1.8 g PETG, glass 47.6, camera 4.0 → **111.4 g**, CoM (−18.8, 0, +35.3) mm |

Geometry of the result, for navigation:

- The eyes (virtual pupils) are **545 mm** above the floor; at −30° the floor is
  in view from **0.88 m** ahead of the neck axis — about 1.5 s of walking.
- The two fields cross **~110 mm** ahead of the neck axis; nearer than that an
  object is seen by one eye only.
- Depth step for 0.5 px of disparity with the 63.5 mm baseline: **4 mm at 1 m /
  38 mm at 3 m** at full resolution; 13 mm / 114 mm streaming a 1536-px-wide
  frame.

### 4.1 The plant

`sim/bimo_biped_v6ar.xml` is regenerated (`gen_plant_v6.py` +
`build_v6_inertia.py --write`). The head body is now 111.4 g with its glass as
separate mass-only geoms (the mirrors sit 45 mm off the neck axis: head Izz
3.1e-5 → **1.85e-4 kg m²**), plus `eye_L` / `eye_R` sites at the virtual pupils;
`camera` stays at the real lens. Robot **1.790 → 1.863 kg (+72.7 g at the top
of the tower)**.

The regeneration also carries **three drifts that predate this change** — the
committed plant was last regenerated 2026-09-14, and on a clean `main` the same
two commands already produce them: the torso inertial is 725 g, not 705
(`pelvis_v7.stl`, 09-17); several geoms gained names (`gen_plant_v6.py`, 09-19);
and the thigh/shin links are 0.3 g lighter each (`leg_link_v6.stl`, 09-24).
They are in this diff because a generated file is regenerated whole, not
hand-picked.

This is the **v6 design plant**; the nightly training runs on the v5 plants and
is untouched. Anything trained on v6 from here on sees the heavier head.

---

## 5. Building and aligning it

1. **Print** in **black** PETG (the inside is an optical cavity; light walls
   scatter): `head_shell` base-down, `head_lid` flat, `camera_sled` on its front
   face. No supports.
2. **Mirrors**: glue each into its pocket on the inside of a chevron wall —
   2-part epoxy or neutral-cure silicone, a thin bead on the 5 mm frame round
   the back window. **Never cyanoacrylate**: its vapour frosts first-surface
   coatings. The pocket floor sets the mirror's plane; press flat, don't smear
   the front face (clean only with lens tissue and IPA, it's bare aluminium).
3. **Head on the neck horn**: 4 × M3×6, driven straight down through the nose
   with the sled out — the prism sits behind the screw circle, so it can go in
   before or after.
4. **Camera**: 4 × M2×5 into the sled bosses, lens aft; slide the sled down the
   nose grooves; run the ribbon down behind the board, along the floor under the
   prism, and out through the slot behind the screw circle.
5. **Prism**, with the camera live: the pad's 0.5 mm recess sets its depth and
   height; set its **sideways** position *actively* — slow epoxy on the
   hypotenuse, nudge the prism until the split is centred in the live image,
   let it cure. (Why: 0.2 mm sideways at 2.8 mm from the pupil moves the split
   ~4°. The printed pocket can't hold that; the camera can.)
6. **Lid**: 4 × M2.5×8 flat-heads. It also holds the sled down.
7. **Calibrate** as a stereo pair: crop the frame into its left and right
   halves, treat them as two cameras (OpenCV `stereoCalibrate` with a
   checkerboard). Expect each half rotated by the mirrors (a proper rotation,
   not a flip), the Wide lens's barrel distortion, and a soft brightness ramp at
   each eye's **outer** edge where the pupil straddles the prism apex —
   flat-field it out.

---

## 6. Open items

- **The lens's entrance-pupil depth is an estimate** (1.5 mm behind the front
  of the lens; `PUPIL_BELOW_FRONT`). A ±1 mm error moves the prism's apex
  distance and shifts the field by a degree or two; measure it on the bench
  (e.g. the apex-seam position vs sled shim) before trusting the last degree.
- **+72.7 g at the top of the robot.** The v6 design gates should be re-run on
  the regenerated plant before this head flies; if they object, the lighter
  trade is a 12.5 mm prism (−7.6 g, −2° per eye) and 35 × 50 mirrors
  (−11.4 g, −3° and −13 mm of baseline).
- **Dust.** The eye windows are open; first-surface mirrors don't forgive
  fingerprints or grit. A cover window (a plane-parallel plate) in each eye
  opening is a straightforward follow-up if the robot's floor life demands it.
- **Seam near the eyes' outer edges.** Each eye's outer ~10° is the part of the
  image nearest the prism apex; it carries the pupil-split brightness ramp.
  Harmless for stereo (the overlap is at the inner edges) but budget it when
  cropping.

---

## 7. Why not split the frame top/bottom? (traced, 2026-09-24)

Tom, on seeing this design:

> how are the left and right sides separated? I was imagining the bottom half
> to be one side, and the top half to be the other ... that way we get a wider
> horizontal field of view.

Right on paper: a vertical knife edge (this design) gives each eye half the
frame's **width**; a horizontal one would give each eye the **full** width at
half the height. So it was traced like everything else —
[`cad/v6/periscope_tb.py`](../../cad/v6/periscope_tb.py), output in
`stereo_topbottom_study.txt` (`.venv/bin/python cad/v6/periscope_tb.py`, ~7 min).

**The ceiling is real.** With perfect optics, the top half of the sensor gives
an eye **80° wide at a 26° band** (96° at 20°, 54° at 30°). The band and the
width trade because the half-sensor's straight edge bows once the band is
levelled.

**Inside a head, it loses.** Four families of layout (a horizontal-apex prism
or two stacked fold mirrors, then 1–3 more mirrors per eye; camera facing
forward and aft; both image flips), each random-sampled and refined with
mirrors **unbounded** — just big enough for the light — so this is each
layout's best case before any stock size is imposed:

| layout | per eye | total / overlap | last mirror needed |
|---|---|---|---|
| prism + 3 mirrors per eye | **36°** | 50° / 22° | **98 × 70 mm** |
| 2 stacked folds + 2 mirrors per eye | 33° | 53° / 13° | 454 × 216 mm |
| 2 stacked folds + 3 mirrors per eye | 27° | 44° / 10° | 106 × 77 mm |
| **this design** (left/right) | **38°** | **56° / 20°** | **50 × 50 mm, stock** |

The best top/bottom layout is 2° narrower per eye than this one, uses seven
pieces of glass instead of three, and needs a last mirror twice the size of any
stock part. (The searches are randomised; the best of several runs is quoted,
so a slightly better top/bottom layout may exist, but not a 2× one.) Why:

1. **Each eye needs four reflections, not two.** Each half-frame is
   one-sided vertically, so each eye's image has to be tilted ~17° to put both
   eyes on the same band, *and* moved sideways to make the baseline. Two
   mirrors can do one or the other, never both.
2. **The full-width fan has to turn a corner twice.** A mirror only passes a
   ±51° fan if it bends it by more than ~100°, and the fan keeps spreading
   between folds, so the last mirror grows with the path to the window.
3. **The shoulders cap that path.** Every best design fills the space
   between the shoulders. Allowing a head twice as tall and twice as deep
   changed nothing (30–35° per eye, 83–188 mm mirrors).
4. The two eyes' optics can't be mirror images (the top half can feed one side
   only): the right eye's would be the left's turned upside down.

So the left/right split stays. If more **total** horizontal field matters more
than stereo overlap, the knob is toe-out -- turning the eyes further apart.
Traced with the same stock parts (`periscope_optics.CTR_RANGE`):

| eye centre | per eye | total | shared (stereo) | baseline |
|---|---|---|---|---|
| 9.5° out (**this design**) | 38° | 56° | 20° | 63.5 mm |
| 12° | 38° | 60° | 16° | 59.8 mm |
| 15° | 38° | 66° | 10° | 52.7 mm |
| 18° | 36° | 70° | 2° | 51.7 mm |

Total width comes at almost exactly the cost of stereo overlap, and some
baseline.
