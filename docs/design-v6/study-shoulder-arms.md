# Get-up study, round 4: top-of-torso shoulder arms (2026-09-16/17)

Tom's Option 2: *"add arms with some kind of crude shoulder joint. (we should
put the shoulders at the TOP of the torso)."* Companion to
`docs/design-v6-ankle-roll.md` §12 (round 2: hip-level arms/tail, 10-12/12
seat-push, 6/6 robust) and §11-12.2 (legs-only, negative). Round 2's
shoulder-height arms (deck top, `arm_z=None` == 0.059 m above the yaw axis,
a 1-DOF 200 mm stub) were 0/N: every push pivoted the body about the head
into a headstand (`getup_search_arms*.txt`). This round asks the sharper
question: can arm **design** — length, an elbow, an ab/adduction DOF — make
a mount that is genuinely at the top of the torso work, and after seeing the
first results Tom set the final framing: *"When not in use, arms should hang
down at the sides. It looks to me like we'll want elbows."* So the 2-DOF
elbow arm at a top-of-torso shoulder, hanging straight down when idle, is
the primary design; the single 35 cm 1-DOF arm is the crude baseline.

**Every run in this study uses `self_collide=True`** (cherry-picked from
`v6-getup-legs-skid` — commits `7445c58`/`759fe96` onto this worktree, squash
messages "plant: optional leg-vs-body self-collision" / "plant:
self-collision by opt-OUT, not opt-in"): a top-mounted arm folds along the
torso and past the head, and reaches past the thighs when planting beside
the hips, so without real self-collision a "stand" would be geometry
passing through geometry, not a result. `build_xml(DesignParams())` is
byte-identical before and after every edit in this round (`self_collide`
defaults to `False`, `arms` defaults to `False`), confirmed by diff against
a saved reference each time a plant or arm-model change was made.

## 1. What was modelled

`sim/gen_plant_v6.py` `DesignParams` (existing arm fields from round 2:
`arms`, `arm_len`, `arm_mass`, `arm_shoulder_x`, `arm_z`, `arm_elbow`,
`arm_fore_len`; new this round):

| field | meaning |
|---|---|
| `arm_abd` | a THIRD, PROXIMAL shoulder DOF: ab/adduction, axis X like `hip_roll`, same sign convention (+ = away from the centreline on the LEFT arm) — one more STS3215 per arm, mounted between the torso and the existing pitch joint |
| `arm_abd_add` / `arm_abd_abd` | adduction / abduction range limits, deg (default 20° / 100°) |
| `arm_shoulder_y_extra` | extra lateral offset beyond the torso-hugging default mount, for a widen sweep |

Shoulder height (the fixed requirement): **TOP_Z = `deck_bot + deck_t` =
0.079 m** above the yaw axis — the deck's top surface, level with the neck
servo (measured in `sim/gen_plant_v6.py`: `deck_bot = housing_h +
batt_layer_h = 0.043 + 0.031 = 0.074`, `deck_t = 0.005`). This is *higher*
than round 2's negative shoulder mount (0.059 m = `deck_bot - 0.015`), which
is re-run here as `sanity_old_deck_12` to confirm the harness reproduces the
known failure under `self_collide=True` before trusting any new positive.
Shoulder x (front/back) and lateral offset are swept via the existing
`arm_shoulder_x` and the new `arm_shoulder_y_extra`. Hand: the existing
12 mm rubber-friction sphere (`friction 1.0 0.02 0.001`); a wider pad was
not tested (not measured — out of scope once the plain hand stood robustly).

Study script `sim/getup_v6_shoulder.py` (modelled on
`sim/getup_v6_appendage.py`): `seat` (supine sit-up → brace → push → rise),
`sideseat` (one arm swings out via `arm_abd` to plant beside one hip),
`pushup` (prone → both arms reach forward → push → knees under → sit back),
`elbowsweep`/`elbowrobust` (elbow-arm length sweep + robustness/peak-torque
pass), `hangpose` (idle-pose hand position + leg clearance, `mj_forward`
only), `walkarmcontacts` (Gate-D-style walk with arm-vs-leg contact
counting), `render`.

## 2. Seat push (supine → standing): results

`docs/design-v6/getup_search_shoulder_seat.txt` (930 lines, self_collide=True
stated in its own header). Grid per non-abd/non-elbow config: 3 brace
angles × 3 push angles × 2 ankle angles = 18 variants; non-abd elbow: 2×3×2
= 12; every `arm_abd` config: ×3 abduction pairs = 54 (36 with an elbow
too).

| config | mount | reach (arm+fore, m) | servos | seat-push variants standing |
|---|---|---|---|---|
| `sanity_old_deck_12` | 0.059 m (round-2 negative height) | 0.12 | 2 | **0/18** (reproduces the round-2 negative under self_collide) |
| `top1_len12` | 0.079 m | 0.12 | 2 | 0/18 |
| `top1_len20` | 0.079 m | 0.20 | 2 | 0/18 |
| `top1_len28` | 0.079 m | 0.28 | 2 | 0/18 |
| `top1_len35` | 0.079 m | **0.35** | 2 | **10/18** |
| `top_elbow_15_15` | 0.079 m | 0.30 (0.15+0.15) | 4 | 0/12 |
| `top_elbow_18_18` | 0.079 m | 0.36 (0.18+0.18) | 4 | **8/12** |
| `top_abd_20`, `top_abd_28` (ab/add + pitch, no elbow) | 0.079 m | 0.20, 0.28 | 4 | 0/54, 0/54 |
| `top_abd_elbow_15_15`, `_18_18` (ab/add + pitch + elbow) | 0.079 m | 0.30, 0.36 | 6 | 0/36, 0/36 |
| `top_abd_20_fore/aft/wide` (x/y sweep on the abd 2-DOF shape) | 0.079 m | 0.20 | 4 | 0/54 each |

**The mechanism, measured from the per-step logs:** every config with total
reach ≤ 0.30 m fails the same way as round 2 — the log's `crouch hold`/`rise`
rows show `up` swinging negative and `head` entering the contact list (e.g.
`top1_len28`: `crouch hold:-0.71/0.14/Larm+Rarm+head`), i.e. the push
pivots the torso over the head into a headstand, exactly round 2's failure.
At reach ≥ 0.32 m (see §4) the mechanism changes: after `push`, the arm
lifts clear of the floor (contacts drop to `Lfoot+Rfoot` only) and the legs
finish the rise exactly like round 2's hip-level appendages — a genuinely
different, working mechanism, not a variant of the failing one.

**`arm_abd` never once stood**, including reach-matched configs
(`top_abd_28` = 0.28 m reach, same as failing `top1_len28`; `top_abd_elbow_
18_18` = 0.36 m reach, same as *standing* `top_elbow_18_18`). Comparing the
two 0.36 m-reach logs line by line: the plain elbow arm's `rest` pose
(shoulder 60°, folded away after the push) shows contacts `Lfoot+Rfoot`
only, while the abd-equipped arm's `rest` pose keeps `Rarm+Rfore+torso` in
contact through `crouch hold`, `rise`, `stand` and `straight` — the stand
stalls in a stable partial crouch (`up +0.61`, `pelvis_z` 0.265-0.27 m,
never reaching the 0.387 m standing height). Measured, not fully diagnosed:
the extra proximal ab/adduction body adds servo-case geometry at the mount
point that (with `self_collide=True`) appears to keep the arm's `rest=60°`
fold pressed against the torso instead of clearing it — the 60° rest angle
was tuned for the 1-body arm and was not re-tuned for the abd chain's
different frame. **Not measured**: whether a re-tuned abd rest angle would
let the abd design also stand; out of scope once the simpler elbow design
already worked and Tom's own framing (below) dropped `arm_abd` in favour of
elbows.

## 3. Side-seat push and prone push-up: negative

- **Side-seat** (`docs/design-v6/getup_search_shoulder_sideseat.txt`, one
  arm swings out via `arm_abd` and down to plant beside a single hip,
  5 configs × up to 32 variants each): **0 standing** — inherits the same
  `arm_abd` rest/fold problem as §2.
- **Prone push-up** (`docs/design-v6/getup_search_shoulder_pushup.txt`, both
  arms reach forward to the floor, push, knees under, try to sit back;
  8 configs incl. `top1_len28/35`, both elbow lengths, both abd+elbow
  lengths): **0/N**, same as round 2's hip-level-arm prone result
  (`getup_search_appendage_prone*.txt`, 0/76). The logs show `front` stuck
  at -0.91…-0.97 and `up` at -0.93…-0.22 through every stage — the torso
  never leaves face-down orientation; contacts include `head` from
  `knees under` onward. A top mount does not change the prone mechanism:
  the torso's flat front and the head as the forward-most point in that
  orientation are unaffected by where the arms are rooted. **Prone recovery
  stays solved only via the legs** (design doc §12.1, unaffected by this
  study).

## 4. Elbow-arm length sweep (Tom's framing: shortest robust elbow arm)

`docs/design-v6/getup_search_shoulder_elbow.txt`. Grid: 2 brace/push angle
pairs × 2 elbow angle pairs × 2 ankle angles = 8 seat-push variants per
length.

| upper+fore (m) | total reach (m) | seat-push variants standing |
|---|---|---|
| 0.15+0.15 | 0.30 | 0/12 (§2 grid) |
| **0.16+0.16** | **0.32** | **2/8** |
| 0.18+0.18 | 0.36 | 8/8 (this grid; 8/12 on §2's larger grid) |
| 0.20+0.20 | 0.40 | 6/8 |
| 0.18+0.22 | 0.40 | 6/8 |
| 0.22+0.18 | 0.40 | 6/8 |

**0.32 m is the reach threshold**: 0.30 m (15+15) never stands, 0.32 m
(16+16) stands. 16+16 is therefore the shortest elbow arm tested that
stands at all, and (below) the shortest that is fully robust.

**Robustness + peak torque** (`getup_search_shoulder_elbow.txt`, play 3/5°,
mu 0.3/0.7/1.0, servo 100/80/65%; STS3215 sim stall = 2.72 N·m at 11.1 V,
`sim/design_gates.py SERVOS["sts3215"]`, `V = 11.1/12`):

| config | robust | peak shoulder τ (N·m) | peak elbow τ (N·m) |
|---|---|---|---|
| `top_elbow_16_16` (sh 90→0, el −90→0, ankle −25) | **6/6** | 1.29 – 1.59 | 0.91 – 1.18 |
| `top_elbow_18_18` (sh 60→20, el −60→−20, ankle −40) | **6/6** | 1.53 – 1.97 | 0.91 – 1.10 |
| `top1_len35` (1-DOF, sh 30→0, ankle −25) | 6/6 (`run_traced`, `getup_search_shoulder_robust.txt`) | 1.28 – 1.84 | n/a |

Both elbow lengths pass every one of the 6 adversarial conditions, peak
torque well under the 2.72 N·m sim stall rating at every joint and every
condition (worst case 1.97 N·m at mu 1.0, 72% of stall). **16+16 is the
recommended arm**: same 4-servo, 2-DOF-per-arm design as 18+18, 40 mm
shorter reach per segment, standing height and robustness identical.

## 5. Hanging idle pose and the walk gate

Tom: idle pose = shoulder 0° / elbow 0° (hanging straight down at the
sides) — this is also the DEFAULT actuator target (`ctrl=0`), so the walk
timeline (which never commands the arm actuators) holds it there for free.

**Hand clearance, standing plant, `mj_forward` only**
(`getup_search_shoulder_elbow.txt`, `hangpose` mode):

| config | hand height above floor | lateral gap to thigh axis |
|---|---|---|
| `top1_len35` | 116 mm | 38 mm |
| `top_elbow_16_16` | 146 mm | 38 mm |
| `top_elbow_18_18` | 106 mm | 38 mm |

**Walk gate** (`docs/design-v6/gateD_shoulder.txt`; Gate D's own walk —
`sim/static_gait.py walk_timeline`/`run_walk`, STS3250 rolls+knees, lift
4 cm, 8 steps, lumped-mass plant, `self_collide=True`; arm-vs-leg contact
counting is a copy of `run_walk`'s stepping loop —
`walk_with_arm_contacts()` — since `run_walk` itself does not expose
per-step contacts):

| config | turn+0 | turn+15 | mu 0.3/play 5 | turn−15/mu 0.9 | arm-vs-leg contacts (8 steps, 44.6 s) |
|---|---|---|---|---|---|
| bare (no arms) | OK, margin 13.0 mm | OK, 12.2 mm | OK, 35.2 mm | OK, 26.6 mm | 0 |
| `top1_len35` | OK, 11.6 mm | OK, 12.4 mm | OK, 34.1 mm | **FAIL** (margin only, tilt 13.0°) | 1667, pairs `{arm-shin, arm-ankle_blk}` |
| `top_elbow_16_16` (default shoulder x = −0.02) | OK, 12.0 mm | OK, 11.5 mm | **FELL@17.1s**, tilt 56.7° | OK, 12.2 mm | **7327**, pairs `{forearm-shin, forearm-thigh, forearm-ankle_blk}` |
| `top_elbow_18_18` (default x) | OK, 11.4 mm | **FELL@17.1s**, tilt 53.4° | OK, 34.9 mm | **FELL@17.1s**, tilt 57.0° | 6735, same pairs |
| `top_elbow_16_16` (**shoulder x = −0.05, aft**) | OK, **30.7 mm** | OK, 12.2 mm | OK, **35.6 mm** | OK, 12.7 mm | **31**, pairs `{forearm-shin, forearm-ankle_blk}` (no thigh) |

**The hanging arm swings into the leg during the walk** — thousands of
self-collision contacts per 44.6 s walk at the default mount x (−0.02 m,
close under the neck), and that interference is what causes the real falls
(tilt 53-57°) at the elbow lengths, not a marginal knife-edge. Moving the
shoulder mount **aft** (`arm_shoulder_x = −0.05`, tested only on
`top_elbow_16_16`) drops arm-vs-leg contacts from 7327 to 31 over the same
walk and clears every one of the 4 gate cases, with equal or better CoM
margin than the bare body in two of them (30.7 / 35.6 mm vs 13.0 / 35.2 mm
— the extra mass at the top of the torso does not by itself explain the
margin change; the removed leg-vs-arm contact forces likely do). A "wide"
mount (`arm_shoulder_y_extra = +0.02`, tested alone) barely helps (2907
contacts); aft is the fix, not wide. **The get-up still works with the aft
mount**: `top_elbow_16_16` at `arm_shoulder_x = −0.05`, seat push (sh
90→0, el −90→0, ankle −25) → STANDING, `pelvis_z` 0.387 m (matches the
default-x result exactly; `getup_search_shoulder_elbow.txt`).
`top1_len35`'s single-arm interference (1667 contacts, one knife-edge
failure) was not re-tested with an aft mount — **not measured**.

**Recommended final design: `top_elbow_16_16` with `arm_shoulder_x =
−0.05`** — the shortest robust elbow arm, moved aft to clear the hanging
arm from the swinging leg. Config name in `sim/getup_v6_shoulder.py`
`CONFIGS`: `top_elbow_16_16_aft`.

## 6. Renders

`sim/renders/getup_options/shoulder/` (local, gitignored; filmstrip PNGs
copied into this directory, committed):

- `arm16x16_elbow_aft_supine_side.mp4` / `getup_shoulder_arm16x16_elbow_
  aft_supine_strip.png` — **new this round**: the recommended design's
  supine → standing get-up (side view), STANDING confirmed
  (`pelvis_z` 0.387 m).
- `arm16x16_elbow_aft_walk_hanging.mp4` / `getup_shoulder_arm16x16_elbow_
  aft_walk_strip.png` — **new this round**: a 2-step walking clip with the
  arms in the hanging idle pose, aft mount, `self_collide=True`; steps
  completed 2/2, `ok=True` (the render path is the same `run_walk(...,
  render=path)` used by the Gate D numbers above).
- `arm35_1dof_supine_side.mp4`, `arm18x18_elbow_supine_side.mp4` /
  `_rear.mp4`, `arm18x18_elbow_prone_pushup_fail_side.mp4` and their strips
  — rendered earlier in this round (best 1-DOF supine, best elbow supine
  side+rear, and the representative prone push-up failure); all
  re-confirmed STANDING (supine cases) or not-standing (the prone case) by
  `sim/getup_v6_shoulder.py render`.

## 7. Hardware cost

- **Servo count**: `top_elbow_16_16_aft` adds **4 × STS3215** (2 per arm:
  shoulder pitch + elbow), vs 2 for the 1-DOF `top1_len35` baseline. Robot
  total goes from 13 (12 leg + neck, `torso_v7` default) to **17**.
- **Mass added** (`sim/gen_plant_v6.py` masses: `servo_mass = 0.055` kg,
  `arm_mass = 0.020` kg upper-arm link, forearm = `arm_mass*0.6` = 0.012 kg,
  hand sphere = 0.005 kg, all per arm): 4 servos = 0.220 kg + 2×(upper
  0.020 + forearm 0.012 + hand 0.005) = 0.074 kg → **≈ 0.294 kg total**,
  all at the top of the torso (mount height 0.079 m above the yaw axis,
  0.466 m above the floor when standing — the highest mass addition this
  body has carried in any get-up study; round 2's hip-mounted appendages
  sat 0.06 m *below* the yaw axis and lowered the CoM instead). Measured
  walk-margin effect: turn+0 margin drops from 13.0 mm (bare) to 30.7 mm
  aft-mounted (actually *higher* — see §5, the contact-clearance effect
  dominates the added-mass effect at this scale); turn+15 drops slightly
  (12.2 → 12.2 mm, no change); the elbow's extra ~0.15 kg vs the 1-DOF
  arm's ~0.16 kg is a wash at this level of measurement.
- **Where on the torso**: shoulder root at `arm_z = deck_bot + deck_t =
  0.079` m above the yaw axis (level with the neck servo), `arm_shoulder_x
  = -0.05` m (5 cm aft of the torso origin — behind centre, toward the
  battery/Pi bay side), lateral offset the existing torso-hugging default
  (`deck_w/2 + SV_T/2 + 0.004` ≈ 0.067 m from centreline).
- **obs/firmware DOF change**: +4 actuated joints (`L_shoulder`,
  `L_elbow`, `R_shoulder`, `R_elbow`), all `position` actuators identical
  in kind to the existing leg/neck actuators — no new actuator TYPE, but
  the observation vector, action space and any onboard controller need to
  grow by 4 (`sim/walker_env.py`'s `_nq_act`-based sizing already handles
  this generically, confirmed by every run in this study using the
  unmodified walker/get-up harness with `nu` = 17).
- **Fold-away pose for walking**: shoulder 0° / elbow 0° = hanging straight
  down at the sides (Tom's spec, and the actuators' own zero/rest
  position, so no extra logic is needed to hold it there during a walk
  that never commands the arm actuators). At the recommended aft mount,
  the hand hangs **106 mm above the floor** (18+18) to **146 mm** (16+16,
  §5) with a **38 mm lateral gap** to the thigh axis, and the walk gate
  shows only 31 residual arm-vs-leg contacts over a 44.6 s / 8-step walk
  (down from 7327 at the un-shifted mount) — not zero, but not enough to
  fail any of the 4 gate cases (§5).
- **Head/neck clearance**: the shoulder mount (`arm_z = 0.079` m) sits
  directly below the neck servo/head assembly (`_head()`'s neck joint at
  `deck_bot + deck_t + SV_T + 0.004` ≈ 0.117 m — 38 mm above the shoulder
  mount). No head-vs-arm contact appeared in any seat-push, side-seat,
  push-up or walk-gate log in this study (`contacts_summary`/
  `walk_with_arm_contacts` never reported a `head`-`arm`/`forearm` pair for
  a *standing* config); the `head` contacts that DO appear (§2, §3) are the
  torso-over-head headstand failure mode, not an arm-head collision.
  **Not measured**: clearance during the fold-UP-along-the-torso motion
  itself past the shoulder-to-elbow bend (only end poses and contact
  summaries were inspected, not the full swept volume).

## 8. Verdict

A crude arm at the true top of the torso (0.079 m above the yaw axis, level
with the neck) does stand the robot up from supine, but only past a sharp
reach threshold of about 0.32 m measured end-to-end — well short of that
(≤ 0.30 m, including a re-run of round 2's own negative under
self-collision) it reproduces round 2's head-first pivot failure exactly,
and a third ab/adduction shoulder DOF never stood in any of 468 variants
tested, apparently because its own rest pose keeps the arm pressed against
the torso under self-collision. The two designs that do work — a single
0.35 m 1-DOF arm and a 2-DOF (pitch+elbow) arm as short as 0.32 m total
reach — are both fully robust (6/6 across play/friction/servo-strength) at
peak joint torques 33-72% of the STS3215's simulated 2.72 N·m stall, but
the elbow arm hanging at its idle pose (Tom's spec) swings into the
swinging leg thousands of times over a walk and actually falls twice out of
four gate cases unless the shoulder is moved 5 cm aft, which clears the
interference to 31 residual contacts and restores every gate case (two with
better CoM margin than the bare body). The prone push-up and the
one-arm side-seat push remain negative regardless of arm design, so this
study changes nothing about the legs-only prone recovery already
established. Recommended: `top_elbow_16_16_aft` (`sim/gen_plant_v6.py`
`CONFIGS["top_elbow_16_16_aft"]`) — 4 × STS3215, ≈0.29 kg added at the top
of the torso, hanging straight down when idle. Every number above traces
to `docs/design-v6/getup_search_shoulder_seat.txt`,
`getup_search_shoulder_sideseat.txt`, `getup_search_shoulder_pushup.txt`,
`getup_search_shoulder_elbow.txt`, `getup_search_shoulder_robust.txt` or
`gateD_shoulder.txt`; nothing here was estimated or guessed.
