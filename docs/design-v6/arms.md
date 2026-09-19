# Arms in CAD (2026-09-19): drawing the get-up decision

Tom, 2026-09-19: *"I'm still waiting for a render with the arms... though
perhaps those haven't been designed in CAD yet? Let's get that done."*

They hadn't been. `docs/design-v6/getup-decision-2026-09-17.md` settled the
design — **two 2-DOF arms, shoulder pitch + elbow, 160 + 160 mm, shoulder at
the top of the torso 50 mm aft, hanging straight down when idle** — but
nothing had been drawn, so the assembly, every render and every mass rollup
were still of an armless robot. This note is the CAD half: what got drawn, the
three places the CAD could not match the plant and why, what was measured, and
what needs signing off.

**Nothing here re-derives the design.** The 160 + 160 lengths, the 2 DOF, the
aft mount and the hanging idle pose are the study's measured results and are
used as given (`sim/getup_v6_shoulder.py CONFIGS["top_elbow_16_16_aft"]`).
Everything else is a consequence of hanging a real STS3215 on the real v7
deck, and every consequence that *changed* a plant number is called out below
and next to its constant in `cad/v6/dimensions_v6.py`.

Artifacts, all local:

| what | where |
|---|---|
| whole robot with arms, STEP for FreeCAD | `cad/v6/step/assembly_v6_arms.step` |
| whole robot, three views | `cad/v6/renders/assembly_v6_arms.png` |
| the seat-push pose (shoulder 90, elbow −90) | `cad/v6/renders/assembly_v6_arms_seatpush.png` |
| the fold-up pose (shoulder 180, elbow −90) | `cad/v6/renders/assembly_v6_arms_foldup.png` |
| fly-in filmstrip (bottom row = the arms) | `cad/v6/renders/assembly_v6_flyin_arms_strip.png` |
| per-part renders | `cad/v6/renders/{shoulder_mount_v6_L,arm_upper_v6_L,arm_fore_v6_L}.png` |
| STL + STEP per part | `cad/v6/stl/`, `cad/v6/step/` |
| full ROM sweep log | `docs/design-v6/arm_check_assembly.txt` |

Everything is behind `ARMS=1`. The default build of `assembly_v6.py`,
`pelvis_v7.py`, `parts_v6.py`, `check_assembly_v6.py` and `animate_v6.py` is
unchanged (§7).

---

## 1. The parts

Three designs, six prints. Unlike the legs ("legs are translations, not
mirrors") the **arms are a genuine mirror pair**: the shoulder horn has to
face outboard on both sides, and no translation does that.

### `shoulder_mount_v6(side)` — the deck bracket

A cradle that stands the shoulder servo **on end** on the deck top: case
length vertical, output end down, cable end up, output axis lateral, **horn
outboard**. Floor + an inboard wall carrying the four case screws + two end
walls wrapping the case in x with 0.5 mm of slip; open upward (the servo drops
in) and outboard (the arm swings there).

Three constraints pick that pose, and there is no fourth option:

1. A sagittal-plane shoulder needs a lateral output axis, so the case's
   34.70 mm `SV_CASE_T` always runs along y. The case therefore always reaches
   to `ARM_Y − 5.6`, whatever else is done — which is also why the upper arm's
   shaft can only be 4 mm inboard of the arm plane (§2).
2. Laid **flat** (case length along x) the cradle sits over the **battery
   aperture** (x −36.5…+10.4, |y| < 54.7): the deck is *open* there, so the
   forward case-screw row would have nothing to bolt to. Standing it up puts
   the whole 24.7 mm footprint aft of the aperture.
3. The Pi 4B can only be installed by sliding **down through its deck slot**
   (x −51.8…−37.8, |y| ≤ 45). Nothing of the arm mount may overhang that slot
   or the Pi becomes uninstallable — which is what sets the arm plane at 88
   rather than the plant's 80.35 (§3).

Interface to the torso: **4 × M2.5 flat-head self-tap into blind 2.05 × 4.5 mm
pilots in the 5 mm deck** — byte for byte the interface `neck_collar` already
uses. Positions in `V.ARM_PILOT_XY`, mirrored in y; every one of the eight is
checked against the real pelvis solid by `arm_v6.check_deck_pilots()` (all
PASS, 0.00 mm³ outside the deck). The quadrilateral is squashed because three
cuts box it in: the Pi slot, the battery aperture, and the bracket's own
case-screw wall, whose countersinks must not undercut it.

Reliefs in the inboard wall, all forced by the servo's own geometry: one
**keyhole** opening covering both the rotating idler disc (r 9.6, stands
2.05 proud) and the connector trench (`SV_CONN_*` — a wall over it is a servo
you cannot plug in, and the window doubles as the cable's route inboard along
the deck); plus a ramped **detent** for the moulded back-cover platform.
Cutting the disc relief and the trench window *separately* left a 0.35 mm rib
of wall between them, which the printability audit caught as a THIN finding;
they are cut as one opening for that reason.

### `arm_upper_v6(side)` — shoulder horn to elbow fork

Local frame: the shoulder axis is the Y axis at the origin, local y = 0 is the
arm plane (= the elbow servo's mid-plane, so the arm is **straight**, no jog),
elbow axis at z = −160.

- **Top: single-sided horn plate** — 3.0 mm plate + a 1.0 mm seating boss +
  the 4 × M3 disc bolt circle, exactly `yoke_roll`'s horn arm, kept to plate
  thickness out to r 13 so all four screw heads are reachable from outboard.
  There is no idler-side tine **and there cannot be one**: the shoulder
  servo's idler face looks *inboard*, straight over the deck, so a second tine
  would have to wrap round the case and sweep the deck and the head every time
  the arm folds up. The precedent for a real load on one disc is the hip yaw,
  which hangs the entire leg off one horn.
- **Bottom: a fork** straddling the elbow servo's horn **and** idler discs.
  This is what keeps the forearm in the same plane as the upper arm; going
  single-sided at the elbow instead pushes the forearm 15–18 mm *inboard*,
  straight at the thigh. The fork is the widest thing on the robot after the
  feet, and its inboard tine is the closest thing to the leg — which is what
  §5's sweep is for.
- **Section: an open C** (back web + two side rails, opening forward), tapering
  from `ARM_FRONT_X` 12 mm at the shoulder to `ARM_TIP_X` 5 mm at the elbow
  because the bending moment does the same. See §4 for why it is a C and not
  leg_link_v6's closed box.

### `arm_fore_v6(side)` — elbow grip to hand

Grips the elbow servo's case in a **leg_link grip channel, unchanged in y and
z** — same servo, same two case faces, same 2.4/3.0 plates, same
`GRIP_HORN_RELIEF`, same `GRIP_TOP_IDLER`, same detents. What is *not* copied
is leg_link_v6's arched web and front plate: printed on its back, the web's z
extent lies in the bed plane, so a plain rectangular web has no overhang to
arch away from.

The hand is a **knuckle of r 12.0** — the plant's own contact radius, its hand
being a 12 mm sphere with `friction 1.0 0.02 0.001`. It is prismatic across y
(so the only down-facing geometry is in the (x, z) profile) with the back
quarter of that circle replaced by two 1.3:1 ramps down to the web plane; a
plain sphere or cylinder leaves 17 × 16 mm of its back hanging over the web
with nothing to print onto. **In the sagittal plane — the only plane this arm
pushes in — the contact arc is exactly the plant's circle.**

---

## 2. Where everything is

All in the assembly (world) frame, mm, standing pose, left side.

| | value | the plant's | delta |
|---|---|---|---|
| shoulder axis x | −50.0 | −50 (`arm_shoulder_x`) | 0 |
| shoulder axis y | ±88.0 | ±80.35 | **+7.65** |
| shoulder axis z | 474.38 | — | — |
| shoulder above the yaw axis | **87.91** | 79.0 (`arm_z`) | **+8.91** |
| elbow axis z | 314.38 | | |
| hand centre z (hanging) | **154.38** | 146 | +8 |
| upper arm | 160.0 | 160 | 0 |
| forearm | 160.0 | 160 | 0 |
| end-to-end reach | **320.0** | 320 | 0 |
| robot width at the elbow forks | 223 | — | (feet are 216) |

Derived chain, all in `dimensions_v6.py`: shoulder horn face
`ARM_Y − PLATE/2 − HORN_BOSS_H` = 85.5 → shoulder case mid-plane 65.05 → wall
seat 50.15 → wall inner face **47.15**, which is the number that has to clear
the Pi slot's y = 45 (it does, by 2.15).

### The two deviations, and why

**Shoulder 8.9 mm higher.** The plant put the shoulder at 79.0 mm above the
yaw axis, "the deck top". Two things make that unreachable in CAD: the CAD
deck top is only **73.80** above the yaw axis (the CAD housing is 5 mm
shallower than the plant's `housing_h + batt_layer_h`), and a real servo case
cannot have its axis *in* the deck surface — the axis sits 4.0 (cradle floor,
4.0 not 2.6 because that plate cantilevers ~22 mm past the deck edge) + 10.11
(`SV_AXIS_FROM_OUT_END`) above it. Net +8.91. Direction of the error is
benign: a higher shoulder hangs the hand 8 mm lower relative to the body and
reaches slightly *further* behind the hips, which is the direction the seat
push wants. It still needs re-running (§8).

**Arm plane 7.65 mm further out.** Forced by the Pi slot (§1 constraint 3).
Also the friendlier direction: the study's own widen sweep cut arm-vs-leg
contacts, and in CAD it is what buys the elbow fork its clearance past the
thigh/yoke, whose outer face reaches y 65.5 — the fork's inboard tine sits at
67.6, and at ARM_Y 80 it would have been 59.6, i.e. *inside* the yoke.

---

## 3. Mass

| part | qty | g each |
|---|---|---|
| `shoulder_mount_v6_L` / `_R` | 1 + 1 | 17.7 |
| `arm_upper_v6_L` / `_R` | 1 + 1 | 31.2 |
| `arm_fore_v6_L` / `_R` | 1 + 1 | 33.7 |
| **printed, per arm** | | **82.6** |
| + 2 × **STS3215** (shoulder pitch, elbow) | | 110.0 |
| **= per arm** | | **192.6** |
| **= the pair** | | **385.2** |

PETG at `D.FILAMENT_RHO` 1.27e-3 g/mm³ × `D.PRINT_MASS_FACTOR` 0.90, the same
basis as every other part in `parts_v6.py`.

**Servos added: 4 × STS3215** (Feetech STS3215 == Waveshare ST3215; the same
part number as the neck and the knees/ankles — no new actuator type). The
robot goes from **13 servos (6 × STS3250 + 7 × STS3215)** to **17 (6 × STS3250
+ 11 × STS3215)**.

**Effect on the robot:** `parts_v6.py` rollup goes from **1815 g** to
**2201 g**, i.e. **+386 g, +21 %**, all of it 0.47 m above the floor.

**This is 91 g more than the get-up study carried.** The study budgeted
294 g (4 × 55 g servos = 220 g + 2 × (20 g upper + 12 g forearm + 5 g hand) =
74 g). The servo half is exactly right; the *printed* half is 165 g, not 74 g
— a real CAD link with a servo grip channel, a fork, disc pads and a 27 mm
deep section is 82.6 g per arm, not 37 g. The design is already 21 % lighter
than the first cut (tapered rails, `ARM_RAIL_T` at `D.WALL`, a trimmed
knuckle), and the remaining mass is section depth, which is the thing carrying
the load. **Re-running the seat push with the as-drawn masses is sign-off item
1 (§8).**

---

## 4. Print orientation — the one thing NOT copied from leg_link_v6

| part | orientation | supports |
|---|---|---|
| `shoulder_mount_v6` | `IDENT` — footplate down, cradle walls up | none |
| `arm_upper_v6` | `RY_XUP` — **on its back, web face on bed** | none |
| `arm_fore_v6` | `RY_XUP` — **on its back, web face on bed** | none |

Read the block above `yoke_roll` in `cad/parts.py`. Both hip clevises used to
print flange-down with their arms rising as vertical columns, which lays the
layer lines **across** the arm — and a cantilevered arm carries its bending
load as tension **along** the arm, straight through the interlayer bond. They
kept snapping mid-arm. The control case named there is v5's `leg_link`, which
"prints web-down, so the filament runs the length of the leg": far longer arm,
thinner mid-span, never broken.

These are 160 mm cantilevers that the robot pushes its own mass up on. They
get the control case's orientation, **not** leg_link_v6's (which stands on its
fork end — a change made for its front plate's bridge, which puts this same
load path back across the layers).

Laid on their back, the joint-to-joint axis (model z) and the straddle
direction (model y) are *both* bed-plane axes, so the bending tension runs
along the filament — and, free, any profile drawn in the (y, z) plane extrudes
along x as plain vertical walls. The only real overhang left is a bore along
y, teardropped with `roll = ROLL_UP` (90, peak toward model +x = print up),
exactly as v5's leg_link does.

**What it costs: an open C instead of a closed box.** A front plate would be a
flat ceiling spanning tine to tine in this orientation — the exact finding
that made leg_link_v6 stand up instead. The C is weaker in **torsion**, not in
the bending that matters, and peak measured joint torque here is 1.59 N·m
(shoulder) / 1.18 N·m (elbow) against the STS3215's 2.72 N·m simulated stall.

**Verdict: all three parts PASS `cad/check_printability.py` in their declared
orientations with no slicer supports and no waivers** — they are *not* in
`SUPPORTED`, unlike both hip yokes. First-layer contact 1326 / 2535 /
2887 mm² (78 % / 32 % / 45 % of footprint). Getting there took three real
geometry changes, all recorded in the code: the keyhole opening (§1), 45°
ramps on the detent roofs, and the ramped knuckle back.

---

## 5. Collision sweep

`ARMS=1 .venv/bin/python cad/v6/check_assembly_v6.py --samples 9` → **46
pairs, ALL CLEAR, 75 s**. Full log: `docs/design-v6/arm_check_assembly.txt`.
21 of those rows are new and cover the arms against the torso, the head, the
pelvis, their own bracket and servos, and every leg joint through its ROM with
the arms in the hanging idle pose. Swept ROM: shoulder −90…200, elbow
−100…10.

The rows that matter, min distance over the sweep (buffer is
`D.SWEEP_BUFFER` 0.5 mm):

| joint swept | pair | min dist | at |
|---|---|---|---|
| elbow | `arm_fore` vs `arm_upper` — **the elbow limit** | 0.70 | −100° |
| shoulder | `arm_upper` vs `shoulder_mount` | 1.53 | 164° |
| hip_pitch | `thigh` vs `arm_upper` (the elbow fork) | **2.15** | 90° |
| hip_pitch | `thigh` vs `arm_fore` — **the walk row** | 4.65 | 63° |
| shoulder | `arm_upper` vs `pelvis_v7` | 5.61 | −18° |
| knee | `shin` vs `arm_fore` | 12.56 | 130° |
| knee | `servo_knee` vs `arm_fore` | 21.05 | 102° |
| hip_roll | `yoke_pitch` vs `arm_fore` | 22.92 | 55° |
| shoulder | `arm_fore` vs `pelvis_v7`, FOLDED (elbow −90) | 31.79 | −18° |
| shoulder | `arm_upper` vs `head` at the fold-up | **53.50** | 200° |
| shoulder | `arm_fore` vs `head`, FOLDED | 79.57 | −90° |

The two `head` rows close **the study's own open item 1** (*"fold-up swept
volume: the arm folds UP along the torso during the sit-up; only end poses
were checked"*). Swept, not spot-checked, and not close: 53.5 mm clear at the
worst point. `assembly_v6_arms_foldup.png` shows the pose.

**The sweep caught a real interference.** The first cut ran the fork's web
down to 2 mm above the elbow axis, and `servo_elbow vs arm_upper` reported
**174 mm³ of overlap at elbow −27°**. The elbow servo's case turns *with* the
forearm and its nearest corner sits at `hypot(SV_WID/2,
SV_AXIS_FROM_OUT_END)` = 15.97 mm — leg_link's "r 16 rule", the same number
`leg_link_v6.check_r16` guards. The web now ends clear of that circle by
`V.ARM_R16_BUFFER`, and `arm_v6.check_elbow_rom()` sweeps the servo too so the
part module catches it without needing the whole assembly.

**Elbow ROM: the CAD gives −101…+20°**, declared conservatively as −100…+10 in
`V.ARM_ROM`. The plant assumed ±150. The get-up only ever uses **−90 → 0**, so
the drawn arm covers the motion it was designed for with 11° to spare — but
anything that wanted a deeply folded elbow (a different get-up, a carry pose)
does not exist in this geometry.

---

## 6. Fly-in

`ARMS=1 .venv/bin/python cad/v6/animate_v6.py` →
`renders/assembly_v6_flyin_arms.mp4` (gitignored) and
`renders/assembly_v6_flyin_arms_strip.png` (committed). 400 frames, 43 pieces,
19 groups. The strip's **bottom row samples only the arm groups** — with 19
groups, 8 uniform samples put 5 frames before anything above the knee exists
and exactly one inside the arms, which is a filmstrip that does not show the
thing it was made to prove.

The five arm insertion paths are the bench order, and it is not arbitrary:

1. `shoulder_mount` **down** onto the deck, screwed to the 4 pilots — **while
   the cradle is still empty**, because two of those four pilots sit under the
   servo's own footprint.
2. `servo_shoulder` **down** into the open cradle (the only way in: a U open
   upward and outboard), then 4 case screws driven from *inboard* through the
   wall.
3. `arm_upper` **straight in along the joint axis** onto the horn — the one
   direction a single-sided horn plate can arrive from. Handed: the left arm
   comes from +y, the right from −y (`plan()` mirrors any lateral insertion
   vector with the part).
4. `servo_elbow` **in from the front** into the forearm's grip channel — the C
   section is open forward, the same channel entry leg_link uses.
5. `arm_fore` **up** between the fork tines onto the two discs.

---

## 7. What stayed byte-identical, and how that was checked

Every new behaviour is behind `ARMS=1` (`assembly_v6.arms_on()`), the same
contract `YAW_BEARING_VARIANT` and `HIP_YOKE_VARIANT` keep. Nothing sets it.

- **Default assembly, piece for piece:** 33 pieces, identical labels, volumes
  and bounding boxes to 1e-6 before and after the change.
- **Default STEP, geometry section:** 270 676 lines / 12.6 MB, **byte-identical**.
- **Full byte equality is not achievable here and that is not this change's
  fault:** exporting the *same, unmodified* assembly twice already produces
  STEP files that differ by ~1118 lines, all of them in the trailing
  `STYLED_ITEM` / `COLOUR_RGB` block. build123d's style section is emitted in
  a nondeterministic order. The geometry comparison above is the meaningful
  test, and the piece-level table is stronger than either.
- **Default `pelvis_v7()`** is unchanged; `pelvis_v7(arm_mounts=True)` removes
  exactly 118.82 mm³ = 8 × π × 1.025² × 4.5, the eight pilots and nothing else.
- **Default `check_assembly_v6.py`** → ALL CLEAR, same 25 rows.
- **Default `parts_v6.py`** rollup unchanged at 1815 g.
- **Default `animate_v6.py`** keeps its 1.5 m camera and its uniform 8-frame
  sampling, so previously committed filmstrips are not silently re-shot.

---

## 8. Open items — things to sign off before printing

1. **Re-run the seat push on the as-drawn geometry.** Three inputs moved:
   `arm_z` 0.079 → **0.0879**, the arm plane 0.0804 → **0.088**, and the
   printed arm mass 37 g → **82.6 g per arm**. All three deltas point the
   benign way individually (higher shoulder = more floor reach, wider = less
   leg interference), but 91 g more at 0.47 m is the kind of change the
   robustness sweep exists for. `sim/getup_v6_shoulder.py` with
   `arm_z=0.0879, arm_shoulder_y_extra=+0.0076`, plus the hanging-pose walk
   gate. **Do not print until this passes.**
2. **The pelvis has to be reprinted (or drilled).** `arm_mounts=True` adds 8
   blind 2.05 × 4.5 mm pilots to the deck. A pelvis printed from the default
   STL cannot take the arms. Drilling them after the fact is possible — they
   are blind holes in a flat top face — but the positions are tight against
   the Pi slot and the deck's R3 top fillet.
3. **The shoulder is a single-sided horn joint.** Proven at the hip yaw (which
   carries the whole leg), but there it is in compression down a short stack;
   here it is a 320 mm lever in bending. If anything on this robot gets a
   bench pull test before it goes on, it is this.
4. **The hand is bare PETG.** The plant contacts the floor at friction 1.0;
   PETG on a hard floor is nearer 0.3–0.4. The study's robustness sweep
   covered mu 0.3 / 0.7 / 1.0 at 6/6 standing, so this is *covered*, not
   assumed — but a TPU cap on the knuckle is cheap insurance and the knuckle
   is already a clean prismatic profile to seat one on.
5. **Open C section, not a closed box.** Chosen for the print orientation
   (§4). Torsionally softer than leg_link_v6's box. The arm's load case is
   sagittal bending, so this should not matter — but a hand planted off the
   sagittal plane (a side-seat push, a fall caught on one arm) twists it.
6. **Elbow ROM is −100…+10°, not the plant's ±150°.**
7. **The robot is now 223 mm wide** at the elbow forks (it was 216 at the
   feet). Doorways, the print bed, and the carrying case all got smaller.
8. **Cable routing is drawn only as a window.** Each shoulder servo's lead
   exits inboard through the cradle's trench window onto the deck top; each
   elbow servo's lead runs up the inside of the C through the cable windows in
   both links. Neither is strain-relieved yet, and the shoulder lead crosses a
   joint that folds 290°.

---

## Files

- `cad/v6/arm_v6.py` — the three parts, their fasteners, insertion paths and
  audits (`check_deck_pilots`, `check_elbow_rom`, printability, mass)
- `cad/v6/dimensions_v6.py` — the `ARMS` block: every number above, each with
  the constraint that set it
- `cad/v6/pelvis_v7.py` — `arm_mounts=False` (opt-in deck pilots)
- `cad/v6/assembly_v6.py` — `arms_on()`, `arm_chain()`, the `ARMS=1` STEP/PNG paths
- `cad/v6/check_assembly_v6.py` — `ARM_PAIRS` / `ARM_TOUCHING`, 5-tuple rows
- `cad/v6/animate_v6.py` — the five arm insertion vectors, handed lateral paths
- `cad/v6/parts_v6.py` — the six arm prints and the +4 STS3215 in the rollup
- `docs/design-v6/getup-decision-2026-09-17.md`, `study-shoulder-arms.md` —
  where the design came from
