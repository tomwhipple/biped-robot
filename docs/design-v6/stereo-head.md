# The stereo-periscope head: binocular vision from one camera

*2026-09-24; prism-free since 2026-09-26.*

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

then, on cost (2026-09-25/26):

> *almost $300 is way too much* — *still way too expensive. we'll stick with
> monocular vision if this is the choice. I'd think we could get some cheap
> mirrors and glue them in, then calibrate* — *don't use a prism then* — (a V
> of front-surface squares) *they don't have to come from etsy. Look for
> another source.*

and on the V's small squares (2026-09-27):

> *the design appears to call for smaller reflectors.. where should I get
> them? I don't want to try to cut these.*

This note is the record: the design as drawn (§1), why the optical layout is
what it is (§2 and §7 are the traced trade studies), the parts (§3), what was
**measured** in CAD and sim (§4), how to build and align it (§5), and what is
still open (§6).

Source of truth: [`cad/v6/periscope_optics.py`](../../cad/v6/periscope_optics.py)
(the 3-D ray trace and the design search; the top/bottom alternative is
[`cad/v6/periscope_tb.py`](../../cad/v6/periscope_tb.py), §7) and
[`cad/v6/head.py`](../../cad/v6/head.py) (the CAD, every optical number derived
from the trace). Renders: `cad/v6/renders/head.png`, `head_fields.png`.

---

## 1. The design, in one table

| | old head (2026-09-14) | stereo head, as drawn (2026-09-26) |
|---|---|---|
| camera | Camera Module 3 **Wide**, looking forward | the same camera, looking **aft** into a V of two small mirrors |
| vision | mono, 102° × 67° | **stereo**: left eye **37°**, right eye **26°** wide, both **40°** tall (−30…+10°) |
| total horizontal field | 102° (mono) | **47°**, of which **16°** is seen by both eyes |
| baseline | — | **62.2 mm** between the virtual pupils (human: ~63) |
| eye openings | one lens hole | two windows, **same horizontal plane**, either side of a camera "nose" |
| envelope | 62 × 50 × 60 mm | **145 × 52 × 70 mm** (shoulders are 200 mm) |
| mass | 39 g | **90.6 g** (PETG 70.1 + glass 16.5 + camera 4) |
| prints | shell + face plate | **shell + lid + camera sled** |
| optics to buy | — | 4 front-surface glass mirrors, sold pre-cut, **$29–40** delivered (§3) |

How it works, seen from above (+X forward):

```
        eye window L    | nose |    eye window R          <- the face
     ___________________| CM3  |___________________
     \\  mirror L        \\  |  /        mirror R  //
      \\                   \\|/                   //       chevron walls = the
       \\              V of 2 squares           //        mirror mounts
        \\____________________________________//            <- back wall
```

The camera looks **aft** into a V of two 25 × 20 mm front-surface squares glued
into a printed block, the V's apex 3.2 mm behind the lens's entrance pupil.
The apex cuts the image down the middle: each half-image is thrown sideways to
a 50 × 50 mm front-surface square glued inside that side's chevron wall, which
turns it forward out through the eye window. Two reflections per eye, so each
eye's image is **rotated, never mirrored**, and each eye sees from a *virtual*
pupil behind its outer mirror — the two virtual pupils are the stereo pair.

The two V squares can't both reach the apex: their glass would collide. The
**left** eye's square runs to it; the **right** eye's butts against the back
of the left one, 1.1 mm (a glass thickness) out along its leg. That blanks a
strip at the right eye's **outer** edge — the right eye is 26° to the left's
37° — while the stereo overlap, at the eyes' inner edges, is untouched. (A
knife-edge prism has no such strip: 38° + 38°, but $139–185 on its own, §3.2.)

The sensor's long axis is split, so the left half of the frame is one eye and
the right half the other: at full resolution each eye is 2304 × 2592 px of the
4608 × 2592 frame, less the unused outer edge (§2.1).

---

## 2. Why this layout — the trade study

*The layout studies below were run with a knife-edge prism as the splitter;
the prism-free V (§3.2) keeps the same camera-aft, left/right layout.*

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

## 3. Parts

### 3.1 What to buy (as drawn)

No extra lens: the Camera Module 3 Wide's own lens is used as-is.

| qty | part | price, delivered | used for |
|---|---|---|---|
| 1 pack of 5 | **front-surface mirror, 50 × 50 × 1.1 mm optical glass** — eBay seller *hnpiwxny*, "Front Surface Projector Reflector Mirror" ([listing](https://www.ebay.com/itm/188927818845), 1.1 mm option) | **$16.34** (import fees included) | the 2 outer mirrors, 3 spares |
| 1 pack of 5 | **front-surface mirror, 25 × 20 × 1.1 mm, sold pre-cut** — Amazon [B0FDB2KL5F](https://www.amazon.com/dp/B0FDB2KL5F) (seller Tuoyibaihuodian, ships from China). Pick the **1.1 mm** option: the same listing's 2 and 3 mm ones cost field (below). Or, cheaper, eBay seller *explore-space* ([listing](https://www.ebay.com/itm/158008301027)) | **$23.41** on Amazon (read 2026-09-27; arrives Oct 9–22), or **$12.66** on eBay (read 2026-09-26) | the V's 2 squares, 3 spares |
| — | Raspberry Pi **Camera Module 3 Wide** (already on the robot) — [product page](https://www.raspberrypi.com/products/camera-module-3/) | — | |

Glass **$29–40 delivered** (the V squares from eBay or Amazon), **no
cutting**, arriving mid-October.

- **No manufacturer datasheets.** That's Tom's call for this part: cheap
  mirrors glued in and calibrated. It is a deliberate exception to the repo's
  sourcing rule. So every size and thickness is **measured on arrival** and set
  in `periscope_optics.V_TILE` / `OUTER_TILE`, then run
  `periscope_optics.py --search` and `head.py`. The listings state size,
  thickness and "front surface"; nothing states flatness.
- **Other pre-cut sizes work; the thickness is what matters.** Traced with
  `periscope_optics.py --sizes` ([`stereo_vsize_study.txt`](stereo_vsize_study.txt)):
  - **Size:** 1.1 mm pieces from 20 × 15 to 30 × 30 mm drop into today's
    layout at 47° total (48° for 1″ squares of 1 mm glass). Only the printed
    pockets change:
    set `V_TILE` and re-run `head.py`. 25 × 25, 1″ × 1″, 30 × 30 and
    26 × 32.5 (26 along the leg) keep the full 16° shared. Smaller pieces
    shrink it (20 × 20: 13°).
  - **Bigger pieces need a re-layout.** Pieces 32.5 mm or more along the leg
    (35 × 35 included) hit the outer mirror as laid out today. A re-search
    finds 49°, but it is a CAD rework.
  - **Thickness costs field**, all of it from the right eye, whose square
    starts one glass thickness out from the apex. 1.6 mm glass gives 42–44°
    total, 2 mm 41–43°, 3 mm 36°. **Buy 1.1 mm or thinner.**
  - **Other listings, read 2026-09-27:**
    - RUEHALF 2-packs on Amazon at 1.1 mm, $19.32 with free delivery, but no
      spares: 24 × 20 ([B0GB7QSN6M](https://www.amazon.com/dp/B0GB7QSN6M)) or
      30 × 30 ([B0GBF3LXVC](https://www.amazon.com/dp/B0GBF3LXVC)).
    - From US stock: Spectrum Scientifics'
      [30 × 30 × 2 mm](https://www.spectrum-scientifics.com/First-Surface-Mirror-30mm-x-30mm-x-2mm-p/7657.htm),
      2 × $9.99 + $7.85. It ships in 1–2 days, but the 2 mm glass costs 6°.
    - Cut to size in the US (First Surface Mirror LLC): about $44 a piece.
- **Why 25 mm and not 50 mm in the V.** The light only uses about 20 mm of
  each V leg. A 50 mm piece's far end hits the outer mirror: the optics search
  rejects any layout where glass meets glass (`glass_clash`).

Hardware: 4 × M3×6 (horn, as before), 4 × M2.5×8 flat-head self-tap (lid),
4 × M2×5 self-tap (camera to sled).

### 3.2 How it got here (all traced with `periscope_optics.py`)

The optics started optics-grade: an Edmund #49-414 20 mm knife-edge prism
($170) and two Edmund #43-876 50 × 50 × 3 mm mirrors, $261.50 in all, 38° per
eye, 56° total with 20° shared. Each step below was a response to cost.

| option | splitter | outer mirrors | eyes | total / shared | glass, delivered |
|---|---|---|---|---|---|
| optics-grade (first draft) | Edmund 20 mm prism | 2 × Edmund 50×50×3 | 38° / 38° | 56° / 20° | ~$275 |
| cheapest *documented* | V: 2 × UQG 25×25×1.2 | 2 × UQG 50×50×1.2 | 37° / 26° | 47° / 16° | ~$115–135 (UK) |
| documented knife edge | OptoSigma KRPB4-10 10 mm prism | 2 × Thorlabs ME1.5S-G01 38 mm | 33° / 33° | 50° / 16° | ~$165 |
| craft mirrors throughout | V of 2 mm **back-silvered** tiles | craft tiles | 25° / 25° | 34–36° / 14° | ~$10 |
| **as drawn** | **V: 2 × 25×20×1.1 front-surface** | **2 × 50×50×1.1 front-surface** | **37° / 26°** | **47° / 16°** | **$29–40** |

- **Craft mirrors failed at the V, not at the outer mirrors.** Craft tiles are
  back-silvered, with the glass in front of the silver. At the V's apex, the
  notch between the two tiles' cut glass ends is dead on *both* eyes, and it is
  wider the thicker the tile (§2 of `periscope_optics.py`, `v_back_t`).
  Back-silvered tiles would be fine as the outer mirrors, where a camera
  focused at infinity sees the 4 % front-surface ghost only on close objects.
- **Prisms were dropped because none is cheap.** OptoSigma's 10 mm is $116,
  Edmund's 10/15 mm are $139/153, and Thorlabs MRAK25 (25 mm, sharp apex) is
  $156. Edmund's own spec also allows a "protective bevel" on its apex.

## 4. What was measured

All from `cad/v6/head.py` (run it) and the v6 assembly gate, on the design as
drawn.

| check | result |
|---|---|
| nothing printed in any light path (every part vs the beams at a tighter margin than they were cut with) | **0.00 mm³** for shell, lid and sled |
| glass seated: both V squares, both mirrors, the camera vs the shell | **0.00 mm³** |
| horn screws: a 7 mm driver reaches all four from above (lid off, sled out) | **PASS** ×4 |
| neck ±90° vs the neck servo and the deck | **clear** |
| `ARMS=1 check_assembly_v6.py` (every joint) | **ALL CLEAR**: head vs girdle **2.10 mm** (≥ 1.5 by design), upper arm vs head **29.3 mm** at the fold-up, folded forearm **66.6 mm** |
| self-view: robot parts inside the eyes' fields (to 600 mm), standing, arms hanging | **none** |
| printability (`audit_torso`: shell base-down, lid flat, sled on its face) | **all three PASS**, no supports |
| `pytest tests/` | **190 passed**, 16 skipped |
| mass | PETG 70.1 g + glass 16.5 g + camera 4.0 g → **90.6 g**, CoM (−18.1, −0.1, +34.6) mm |

Earlier in the design, a positive control confirmed the self-view check isn't
vacuous. A cube ahead of the head was fully seen, and with the arms raised
forward (shoulder −90°) both arms came into view. The robot can watch its
hands.

Geometry of the result, for navigation:

- **Floor view:** the eyes (virtual pupils) are **549 mm** above the floor. At
  −30° the floor is in view from **0.88 m** ahead of the neck axis.
- **Stereo range:** the two fields cross **~150 mm** ahead of the neck axis.
  Anything nearer is seen by one eye only.
- **Depth resolution:** the depth step for 0.5 px of disparity at the 62.2 mm
  baseline is **4 mm at 1 m and 39 mm at 3 m** at full resolution. Streaming a
  1536-px-wide frame, it is 13 mm and 116 mm.

### 4.1 The plant

`sim/bimo_biped_v6ar.xml` is regenerated (`gen_plant_v6.py` +
`build_v6_inertia.py --write`).

- **Head body:** 90.6 g, with its glass as separate mass-only geoms, plus
  `eye_L` / `eye_R` sites at the virtual pupils. The `camera` site stays at the
  real lens.
- **Head inertia:** principal moments (6.5, 12.0, 14.7) × 10⁻⁵ kg m². The old
  head's were about 3 × 10⁻⁵.
- **Robot mass:** **1.790 → 1.842 kg**, +52 g at the top of the tower.
- **Pre-existing drifts:** the first regeneration also carried three changes
  from before this work, because a generated file is regenerated whole. The
  torso is 725 g not 705 (`pelvis_v7.stl`, 09-17), several geoms gained names
  (`gen_plant_v6.py`, 09-19), and each thigh/shin link is 0.3 g lighter
  (`leg_link_v6.stl`).
- **Training:** this is the **v6 design plant**. The nightly training runs on
  the v5 plants and is untouched.

---

## 5. Building and aligning it

1. **Print** in **black** PETG. The inside is an optical cavity, and light
   walls scatter.
   - `head_shell` base-down, `head_lid` flat, `camera_sled` on its front face.
   - No supports.
2. **Outer mirrors:** glue each 50 × 50 square into its recess inside a
   chevron wall.
   - Use a thin bead of neutral-cure silicone or 2-part epoxy on the 5 mm
     frame round the back window. Thin glass takes the shape of what it's
     rigidly bonded to, so keep the bead compliant.
   - **Never cyanoacrylate**: its vapour frosts first-surface coatings.
   - Keep the protective film on the mirror face until the glue has cured.
3. **Head on the neck horn:** 4 × M3×6, driven straight down through the
   empty nose. The V sits behind the screw circle, so it can go in before or
   after.
4. **Camera:** 4 × M2×5 into the sled bosses, lens aft.
   - Slide the sled down the nose grooves.
   - Run the ribbon down behind the board, along the floor, and out through
     the slot beside the V's right leg.
5. **V squares, with the camera live:**
   - Glue the **left** square first, into the left recess, with a factory edge
     at the apex. Slow epoxy.
   - Slide it along its leg until the image's split is centred, then let it
     cure. 0.2 mm along the leg moves the split about 4°: the recess can't
     hold that, the camera can.
   - Then glue the **right** square: butt its end against the back of the left
     one and seat it in its recess.
6. **Lid:** 4 × M2.5×8 flat-heads. It also holds the sled down.
7. **Calibrate** as a stereo pair: crop the frame into its left and right
   halves and treat them as two cameras (OpenCV `stereoCalibrate` with a
   checkerboard). Expect:
   - each half rotated by the mirrors (a proper rotation, not a flip);
   - the Wide lens's barrel distortion;
   - a soft brightness ramp at each eye's **outer** edge, where the pupil
     straddles the V's apex, to be flat-fielded out;
   - the right eye's blank strip beyond that.

---

## 6. Open items

- **Measure the glass on arrival.** Size and thickness go into `V_TILE` /
  `OUTER_TILE`; then re-run `periscope_optics.py --search` and `head.py`. No
  datasheets means no flatness spec either; calibration absorbs mild bow.
- **The lens's entrance-pupil depth is an estimate** (1.5 mm behind the front
  of the lens; `PUPIL_BELOW_FRONT`). A ±1 mm error moves the V's apex distance
  and shifts the field a degree or two. Measure it on the bench.
- **+52 g at the top of the robot** against the mono head. The v6 design
  gates should be re-run on the regenerated plant before this head flies.
- **Dust.** The eye windows are open, and front-surface mirrors don't forgive
  fingerprints or grit. A cover window in each opening is a straightforward
  follow-up if needed.
- **Stereo only covers 16° straight ahead.** The total view is 47°, against
  the mono camera's 102°. Toe-out trades overlap for total width
  (§7's table).

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
| **left/right, with the prism** (§2) | **38°** | **56° / 20°** | **50 × 50 mm, stock** |

The best top/bottom layout is 2° narrower per eye than the prism layout, uses seven
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

So the left/right split stays (and the prism-free V keeps it). If more **total** horizontal field matters more
than stereo overlap, the knob is toe-out -- turning the eyes further apart.
Traced for the prism layout (`periscope_optics.CTR_RANGE`); the V behaves the same way:

| eye centre | per eye | total | shared (stereo) | baseline |
|---|---|---|---|---|
| 9.5° out (the prism layout) | 38° | 56° | 20° | 63.5 mm |
| 12° | 38° | 60° | 16° | 59.8 mm |
| 15° | 38° | 66° | 10° | 52.7 mm |
| 18° | 36° | 70° | 2° | 51.7 mm |

Total width comes at almost exactly the cost of stereo overlap, and some
baseline.
