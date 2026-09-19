# The shoulder girdle (2026-09-19): rotating the servos into the torso

Tom, 2026-09-19:

> we need to revisit the arm design... looks like the arms are just bolted on.
> Aside from being brittle, it looks ugly. We should rotate the servo bodies
> 90deg and incorporate them into the torso, then put the head above them, so
> they look like real shoulders.
>
> And maybe do some reading about design asthetics and organic forms...
> fuctionality is the most important consideration, but we can still build
> something that looks good.

and, mid-session:

> ... and put the shoulder joint in the same plane as the hips.

This note is the record of that change: what moved, what it cost, what was
**measured** rather than asserted, and the two things that still need signing
off. The round-4 arms note (`docs/design-v6/arms.md`) describes the design this
replaces; the get-up decision it all serves
(`docs/design-v6/getup-decision-2026-09-17.md`) is **unchanged** — segment
lengths, DOF count and the hanging idle pose are still the study's measured
results and are still used as given.

---

## 1. What changed, in one table

| | round 4 (2026-09-17) | round 5 (this note) |
|---|---|---|
| shoulder servo pose | case **length vertical**, standing on end | case rotated 90° about its **own output axis**: length **fore-aft**, width vertical |
| shoulder joint x | −50 mm (aft mount) | **0 — the hip plane** |
| arm plane `ARM_Y` | 88.0 | **103.35** |
| shoulder height above yaw | 87.91 | 90.16 |
| mount | 2 × `shoulder_mount_v6` deck brackets, 4 self-taps each | **1 × `shoulder_girdle_v6`**, 12 verified deck pilots |
| neck | separate `neck_collar` print | **absorbed into the girdle** |
| case screws | idler (inboard) face | **horn (outboard) face** — access, see §4 |
| prints for the arms | 6 | **5** |
| walk gate (4 cases) | 4/4, 0–184 arm-vs-leg contacts | **4/4, 0 contacts** |

The arm **links** (`arm_upper_v6`, `arm_fore_v6`) are untouched. Rotating the
servo about its own output axis leaves the horn exactly where it was, so
everything downstream of the shoulder disc is the same part it was yesterday.

---

## 2. "The same plane as the hips" — the part that had to be re-measured

Moving the shoulder to x = 0 **undoes round 4b's aft mount**, and that mount
was not a style choice: it was the measured fix for the hanging arm walking
into its own leg (7327 arm-vs-leg contacts at the default mount, 31 at
x = −50; `gateD_shoulder.txt`). So it was re-run, not assumed.

**Step 1 — x = 0 as-is fails.** Walk gate, 8 steps, self-collision on
(`getup_search_hipplane_walk.txt`):

| config | result | arm-vs-leg contacts / 44.6 s |
|---|---|---|
| `top_elbow_16_16_aft` (round 4) | up | 31 |
| x = 0, plant arm plane | **FELL** | 8240 |
| x = 0, as-drawn CAD arm plane | **FELL** | 28977 |
| x = 0, +20 mm wider | up | 3996 |
| x = 0, +30 mm wider | up | 146 |

Naively, x = 0 costs a **220 mm shoulder span**.

**Step 2 — the plant was wrong about the arm.** Every contact pair in every
run is `*_forearm` vs thigh/shin/hip_yaw and **never** `*_arm`. The upper arm
hangs entirely above the thigh; only the forearm reaches it. The widest
inboard thing on the forearm body is the **elbow servo box** — and the plant
placed that box half a case **inboard** of the arm plane and **centred** on
the elbow axis, while `cad/v6/arm_v6.py` centres it **on** the arm plane (the
upper arm *forks around it*) and hangs it **below** the axis. The plant's
forearm was ~20 mm too fat toward the leg and ~12 mm too high.

Fixed behind `DesignParams.arm_cad_servos` (default `False`, so every round
1–4 number stays reproducible). Re-measured
(`getup_search_hipplane_cadservos.txt`):

| config | result | contacts |
|---|---|---|
| aft mount, as-drawn servo boxes | up | **0** (was 31) |
| x = 0, as-drawn, today's arm plane | up | 1579 |
| x = 0, as-drawn, +10 mm | up | 768 |
| x = 0, as-drawn, +20 mm | up | 164 |

x = 0 now *stands* at the current width. But the full four-case gate still
failed the low-friction case, and the failure was **non-monotonic** in width
(+10 passed, +23 failed, +20 passed), i.e. grazing contacts perturbing an
open-loop gait — not a clean clearance threshold.

**Step 3 — is it contact or mass?** Ablation
(`getup_search_girdle_ablation.txt`): the same geometry with self-collision
**off** is **4/4 including mu 0.3 / play 5**, and the bare robot is 4/4. So the
fall is *contact*, not the arms' inertia at x = 0 — a geometry problem, and
therefore solvable by geometry or pose.

**Step 4 — the pose is free.** The arm actuators are not in the walk timeline;
whatever they are commanded to is what they hold. The one thing the aft mount
really bought was **fore-aft separation**, and a held shoulder angle buys the
same thing without moving the joint. Measured
(`getup_search_girdle_pose.txt`, as-drawn servos, x = 0, `ARM_Y` 103.35):

| held shoulder | contacts / 44.6 s |
|---|---|
| 0° (straight down) | 592, and mu 0.3 / play 5 **falls** |
| **10°** | **0** |
| **15°** | **0** |
| 20° | 0 |
| 25° | 0 |

and the full gate at both 10° and 15° is **4/4 with zero contacts**
(`getup_search_girdle_gate4_pose.txt`) — strictly better than round 4b's aft
mount (4/4 with 0/126/0/184).

**`ARM_WALK_HOLD = 15.0°`**, the middle of the measured zero band. It costs
~0.02 N·m of holding torque per shoulder against a 2.72 N·m stall. It is a
*controller default*, not geometry — and it is what a walking human does with
their arms anyway.

> **Sign-off item 1.** `ARM_WALK_HOLD` is currently recorded in
> `dimensions_v6.py` and used by the gate harness (`--pose=`). It is **not yet
> wired into a controller default** — the walk must command shoulder = 15°.
> Commanding 0° puts the design back to 3/4 on the gate.

---

## 3. Why `ARM_Y` is 103.35 and not a round number

Not chosen — derived, outward from the housing skin, every step a real
thickness:

```
housing skin (HOUSING_HW)                       61.26
+ 1.24  air, girdle wall off the skin           62.50   GIRDLE_WALL_Y0
+ 3.00  inboard wall (GRIP_PLATE_T_IDLER)       65.50   ARM_SEAT_Y
+ 0.15  seat clearance, + 14.75 to the case mid 80.40   ARM_SERVO_MID_Y
+ 20.45 to the horn face                       100.85   ARM_HORN_FACE_Y
+ 1.00 boss + 1.50 half-plate                  103.35   ARM_Y
```

The happy part: the walk gate independently wants ≥ +10 mm at x = 0, and the
structural chain lands at +23. **The structural number and the measured one
agree instead of fighting.** Girdle width is 200.30 mm, inside the 220 mm bed.

---

## 4. Why the case screws moved to the horn side

Every other servo on this robot is gripped on its idler face. Not this one.
With the case lying against the torso there is **1.24 mm** between the
girdle's inboard wall and the housing skin, and no driver reaches into a
1.24 mm slot. The horn-side face is open air, so the four case screws are
driven from **outboard**, through a plate with `D.GRIP_HORN_RELIEF` around the
Ø19.6 case boss — exactly `leg_link`'s own horn-side grip plate. The inboard
wall keeps its locating job and carries no screws.

Two consequences the CAD had to absorb:

* the plate needs a pocket for the moulded **horn-side rib** (1.13 mm proud
  along most of the case). Pocket, not window: all four screw rows sit outside
  the rib's band, so they still bear on full plate.
* the **connector trench** now faces inboard into that same 1.24 mm slot, so
  its window in the inboard wall is cut **open to the top** — the lead leaves
  the plug and goes straight up out of the notch, never into the slot.

## 4b. Why nothing hangs below the deck plane

The first version hung the pods *beside* the torso with the axis at exactly
the study's 79 mm above the yaw axis — which would have deleted the shoulder
height deviation entirely. It was thrown away: the part prints **base down**,
and a pod floor below z = 0 leaves the girdle's centre bridging 125 mm of air
between two skirts. Flat-bottomed, the axis sits `GIRDLE_FLOOR + SV_WID/2` =
16.36 above the deck and no lower. Both variants were gated anyway and scored
identically (3/4 without the held pose), so the printable one wins on a tie.

Every bay is open **upward**: all four pod walls are full height, which does
not block a servo dropped straight down, and a roof would — a flat 32 mm
ceiling is unprintable and a 45° vault over the same span is 32 mm tall.

---

## 5. The reading, and what it actually changed

Sources: [Core77 — *A Periodic Table of Form*](https://www.core77.com/posts/12752/a-periodic-table-of-form-the-secret-language-of-surface-and-meaning-in-product-design-by-gray-holland-12752),
[Oxford Product Design — *Surface continuity & product design aesthetics*](https://oxfordproductdesign.com/blog/articles/3d-cad-surface-continuity-product-design-aesthetics),
[Plasticity — *Continuity of curves and surfaces*](https://doc.plasticity.xyz/cad-essentials/continuity-curve-and-surface),
[*Quori: a community-informed design of a socially interactive humanoid*](https://arxiv.org/pdf/2109.00662),
[Wevolver — *Design considerations for humanoid robots*](https://www.wevolver.com/article/design-considerations-for-humanoid-robots),
[Grokipedia — *Body proportions*](https://grokipedia.com/page/Body_proportions).

Four ideas survived contact with a 3D printer and a 2.72 N·m servo:

1. **One form per joint type, repeated.** The humanoid-design literature's most
   portable rule: design a form for each kind of actuator and repeat it
   wherever that motion occurs. The pods reuse the rounded-rectangle bay, the
   R3-eased top edge and the fenestrated web the hip yoke and the head already
   use, so the shoulder does not look like a different robot's part.
2. **Continuity beats decoration.** G2 surfacing is not reachable in
   build123d, and chasing it would have been theatre. What *is* reachable is
   removing the **steps**: the girdle's inboard wall stands 1.24 mm off the
   skin and its pods pick up the torso's own width, so the eye reads torso →
   shoulder as one flare rather than a bracket bolted to a lid.
3. **Soft curves and eased edges, not arrises.** Every outer vertical corner
   of the pods is filleted and 38 of 44 top edges are eased — these are the
   surfaces a hand actually touches.
4. **Silhouette is the whole game, and the connecting tissue is what was
   missing.** The first build of the girdle rendered as *three separate lumps*
   — box, tube, box. Adding the **trapezius webs** (full-height fenestrated
   panels from the neck tube out to each pod, on the clavicle and aft-tie
   bands) is what turned it into one shoulder line running out of the neck.
   That change is also the **top chord** structurally: it ties the pods to
   each other and to the neck at the height the arm's moment acts, not just at
   the deck. Form and load path wanted the same thing.

On **proportion**: human biacromial width is ≈ 0.245 × height; this robot is
now 200 mm across the shoulders at ~555 mm tall, i.e. **0.36** — decidedly
broad. That is a deliberate, functional consequence (§3), and it is in
proportion with *this* robot rather than with a human: the feet already span
168 mm. It is worth a look in FreeCAD before it prints.

> **Sign-off item 2.** Shoulder span 200.3 mm on a 555 mm robot. Functionally
> required, but it is the single most visible proportion change — eyeball it
> in the assembly before committing to print.

---

## 6. A pre-existing defect the probe found

Deck pilots cannot be placed by drawing: the deck is cut by the Pi slide slot,
the battery aperture, the GD lead slot and the R3 top fillet. So a 2 mm grid
of candidate pilots was **probed against the real `pelvis_v7()` solid**. All
twelve girdle pilots pass, on both sides, and are re-verified on every build
by `shoulder_girdle_v6.check_deck_pilots()`.

The probe also found something it was not looking for: **all four of
`neck_collar.py`'s flange pilots are 100 % outside the deck solid** —
(−42.01, ±19.26) lands in the Pi slide slot, (+17.01, ±19.26) in the GD lead
slot, 14.85 mm³ of a 14.85 mm³ pilot. The collar as drawn has nothing to bite
into. This predates round 5 and is not caused by it; the girdle absorbs the
collar and replaces those four with the verified set, so the **assembled**
robot is fixed. `cad/v6/neck_collar.py` itself is still wrong for anyone
building the armless torso.

> **Sign-off item 3.** Either fix `neck_collar.py`'s four pilots or mark the
> armless torso as superseded by the girdle build.

---

## 7. Assembly order

Unchanged in kind, shorter in practice:

1. **battery in first.** The pack cannot pass the neck tube once the girdle is
   on — exactly as it could not pass the neck collar before. Pre-existing
   constraint, not a new one.
2. **girdle down** onto the deck and screwed to its 12 pilots **while empty**
   (four pilots sit under the servo bays).
3. **each shoulder servo dropped** straight into its open bay from above, then
   4 × M2.5 from **outboard**.
4. **upper arm** offered straight in onto the horn along the joint axis.
5. **elbow servo** slid into the forearm's grip channel from the front.
6. **forearm** lifted up between the fork tines onto the two discs.
7. neck servo and head as before, now into the girdle's own tube.

`cad/v6/animate_v6.py` flies the parts in along exactly this order.

---

## 8. What was built, and what it measures

| what | where |
|---|---|
| the part | `cad/v6/shoulder_girdle_v6.py` → `stl/`, `step/` |
| whole robot with arms, STEP for FreeCAD | `cad/v6/step/assembly_v6_arms.step` |
| whole robot, three views | `cad/v6/renders/assembly_v6_arms.png` |
| the girdle alone | `cad/renders/shoulder_girdle_v6.png` |
| fly-in filmstrip | `cad/v6/renders/assembly_v6_flyin_arms_strip.png` |
| full ROM + interference sweep | `docs/design-v6/girdle_check_assembly.txt` |
| walk gate, x=0 vs aft | `docs/design-v6/getup_search_hipplane_*.txt` |
| as-drawn servo boxes | `docs/design-v6/getup_search_hipplane_cadservos.txt` |
| contact-vs-mass ablation | `docs/design-v6/getup_search_girdle_ablation.txt` |
| held-pose sweep + gate | `docs/design-v6/getup_search_girdle_pose.txt`, `..._gate4_pose.txt` |

**Girdle:** 58.00 × 200.30 × 33.10 mm, 82.9 g PETG, one print, base down, bed-OK
(220 mm). **Checks:** all 12 deck pilots buried in the real pelvis solid; zero
servo/girdle overlap both sides; nothing below the deck plane; 0.70 mm of air
between the girdle's outer face and the rotating arm plate; head base 3.88 mm
above the pod tops.

**Full assembly sweep: `=> ALL CLEAR`**, 0 failures. The rows that matter here:

| joint | pair | clearance (mm) |
|---|---|---|
| shoulder | `arm_upper` vs `shoulder_girdle_v6`, whole sweep | 1.53 |
| shoulder | `arm_fore` vs the girdle, folded | 108.11 |
| neck | `head` vs the girdle's neck tube, ±90° | 2.10 |
| hip_pitch | **the walk row** — thigh vs the hanging forearm | 23.00 |
| knee | shin vs the hanging forearm | 29.90 |

That sweep found one real defect on the way: `ARM_SHAFT_Y`'s inboard bound was
a literal −3.5 tuned against the round-4 bracket's 83.0 outer face, and against
the girdle's 100.15 it put the upper arm's shaft **0.30 mm inside the girdle**
(0.47 mm³ of overlap at shoulder +55°). It is now *derived* from `GIRDLE_Y1`
with `ARM_SHAFT_CLR`, with an assert, so it cannot rot again.

### Cost

The girdle (82.9 g) replaces two shoulder brackets **and** the neck collar,
and is the single heaviest printed part added by the arms. It sits at the top
of the torso, so it raises the CoM slightly — worth watching, but the walk gate
was run on the as-drawn geometry and is 4/4.
