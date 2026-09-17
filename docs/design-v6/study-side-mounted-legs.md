# Option 1 study: side-mounted ("frog/bird") legs (2026-09-16)

Tom: *"redesign the chassis to be more stable, perhaps with legs mounted on
the sides, frog/bird style."* This studies whether moving the hip axis
OUTBOARD and UP the torso's own side wall -- so the torso hangs BETWEEN the
legs instead of entirely above them, as it does on the current v6 body --
opens an open-loop get-up from supine, prone and the side, and what it costs
the walker. Companion study: shoulder-mounted arms (`study-shoulder-arms.md`,
separate worktree).

**Everything in this study runs with `self_collide=True`** (gen_plant_v6,
commit 759fe96, opt-out self-collision: every solid part collides with every
other except same-body/parent-child pairs). Without it a splayed or raised
leg can pass through the torso, the battery, or the other leg, and any
"it stood up" result on this body would not be trustworthy. This worktree's
`sim/gen_plant_v6.py` was brought forward to the `759fe96` state (see the
commit "plant: pull in self_collide... from upstream") before this study's
own edits; the ported file was verified byte-identical to the pre-port file
for `build_xml(DesignParams())` except for new geom names (needed for the
self-collision pair exclusion) -- see that commit message for the diff.

## 1. What was modelled

One new `DesignParams` field, `hip_z` (default `0.0`, plant unchanged --
verified: `build_xml(DesignParams())` SHA1 identical before/after this
study's commit to `gen_plant_v6.py`): the height of the hip YAW
axis above the torso's own local origin. The torso's own internal geometry
(`_torso()`/`_head()`: deck, housing, battery, head) is **unchanged** in its
own local frame; only the leg's mount point moves up by `hip_z`, and the
torso's world height is dropped by the same amount so the leg still reaches
the floor (`z0 = z_yaw_above_sole - hip_z` in `build_xml`). The net effect:
`hip_z > 0` pulls the existing torso block DOWN relative to the hip line --
part of it (from local z=0 up to `hip_z`) now hangs BELOW the hips (a bird
"belly" between the legs) and the rest (deck, battery, head) stays above.
The hip-yaw servo's own case mass (built into `_torso()`) was moved by the
same `+hip_z` so it does not go on floating at the old, wrong height.

Combined with two EXISTING parameters (no new fields needed):
- `hip_sep` (today 0.084 m; torso half-width `deck_w/2` = 0.059 m, so
  `hip_sep > 0.118 m` puts the hip axis outboard of the torso's own side
  wall) for "legs on the SIDES".
- `hip_roll_abd` (today 45 deg) widened to 75/90 deg for "frog splay".

| config | hip_z (mm) | hip_sep (mm) | hip_roll_abd (deg) | note |
|---|---|---|---|---|
| `bird_z06_sep160` | 60 | 160 | 45 | hip mounted at "hip roll height" (the appendage study's effective push height) |
| `bird_z10_sep160` | 100 | 160 | 45 | |
| `bird_z15_sep160` | 150 | 160 | 45 | roughly mid-stack: torso straddles the hip line |
| `bird_z20_sep160` | 200 | 160 | 45 | above the deck top (79 mm): most of the torso hangs below the hips |
| `bird_z15_sep200` | 150 | 200 | 45 | wider stance, same height |
| `frog_splay_abd75` | 0 | 84 | 75 | splay only, today's hip mount |
| `frog_splay_abd90` | 0 | 84 | 90 | splay only, max tested |
| `bird_z15_splay75` | 150 | 160 | 75 | bird + frog splay combined |
| `bird_z15_splay90` | 150 | 160 | 90 | bird + frog splay combined |
| `bird_topmount` | 200 | 180 | 75 | closest proxy to Option 2, see below |

Leg segments (thigh 0.110, shank 0.110, current foot) were left as-is per
the brief; a leg-length sweep was **not run** (not measured) -- the
mechanism found (\S4) is about where the torso's own mass grounds, not
about leg reach, so a longer leg was not an obvious next axis and time did
not allow sweeping it as well.

**Option 2 ("horizontal bird torso") approximation, stated explicitly.** A
true horizontal reorientation (torso's long axis parallel to the ground,
head forward, legs under the middle) needs the hip yaw axis to be
HORIZONTAL in the rest pose; `gen_plant_v6._torso()`/`_head()` assume it is
VERTICAL and stack the torso vertically above/below it. Expressing a
genuinely horizontal torso would mean rewriting the torso/head frame
convention, which this study's time budget does not cover. The closest
proxy buildable from `hip_z` alone is `bird_topmount` (hip axis pushed to
the TOP of the housing+deck+head stack, `hip_z` 200 mm with `hip_sep` 180
mm and `hip_roll_abd` 75 deg): nearly the whole torso block hangs BELOW the
hip line, which is the "legs under the middle of the mass, short lever to
the head" idea in spirit but not the "head forward, horizontal" geometry
Tom described. Treat every `bird_topmount` number below as that
approximation, not a true horizontal-torso result.

## 2. Quasi-static probe (`getup_v6_side.py probe`, log `getup_search_side_probe.txt`)

Standing CoM height (whole-robot `subtree_com` z, straight-leg stand, from a
direct `mujoco.mj_forward`, not in the log file but reproducible with the
one-liner in the commit): hip_z 0 -> 0.292 m; hip_z 100 mm -> 0.254 m;
hip_z 200 mm -> 0.216 m; hip_z 200 mm + hip_sep 180 mm -> 0.197 m. Raising
the hip mount does lower the standing CoM, as intended.

**Sit-up threshold: hip_z <= ~60 mm keeps the classic sit-up; hip_z >= 100 mm
loses it**, measured directly:

| config | "sit up" (hip -90) pelvis z | up-vector | contacts |
|---|---|---|---|
| `bird_z06_sep160` | 0.044 | **+0.99** (near vertical) | `L_foot, L_thigh, R_foot, R_thigh` (feet+thighs grounded, same as the bare-leg baseline) |
| `frog_splay_abd75/90` | 0.104 | **+0.99** | `L_foot, L_thigh, R_foot, R_thigh` |
| `bird_z10_sep160` | 0.052 | +0.07 | `torso` only |
| `bird_z15_sep160` | 0.052 | +0.07 | `torso` only |
| `bird_z20_sep160` | 0.068 | -0.14 | `torso` only |
| `bird_topmount` | 0.068 | -0.14 | `torso` only |

Above roughly `hip_z` 100 mm, the torso's own belly (the part of the torso
block now hanging below the hip line) grounds during the hip-flexion sit-up
motion and pins the torso flat (up-vector stuck near 0, contacts = `torso`
only, feet never touch) -- the raised mount removes the classic sit-up
mechanism instead of helping it, exactly the opposite of the "bird" idea's
intent for that specific move.

**Rise-arc corridor sweep** (seated tuck, hip -125, splay 0, knee swept
-130..-10 deg x ankle swept -40..0 deg, `getup_v6_side.probe_config`): for
**every** config in the table, **no (knee, ankle) pair puts the whole-robot
CoM over a grounded sole below pelvis 0.33 m** -- the same wall measured on
the bare-legs body in `getup_v6_legs.py` (docs \S12.2). Moving or widening
the hip mount does not add reach to the thigh/shank (unchanged length), and
the corridor argument was about that reach, not about mount height -- so
this negative was expected, not a surprise, and confirms the mount-point
change alone cannot fix the fore-aft CoM-to-foot problem.

## 3. Dynamic search (`getup_v6_side.py search {supine,prone,side}`)

Sequences (see `sim/getup_v6_side.py`): `seq_situp_wide` (supine: splay ->
sit up -> tuck -> ankle-dorsiflex push -> narrow the stance while rising),
`seq_pushup_wide` (prone: splay -> tuck knees under -> push the pelvis up on
the wide splayed legs via knee/ankle extension -> hinge up -> narrow ->
stand), `seq_side_roll` + `full_from_side` (on the side: lower leg presses,
upper leg swings up and over to roll prone/supine, then the push-up chain).
`settle_fallen` in `getup_v6.py` gained `side_l`/`side_r` start states
(rotate +-90 deg about X) for this.

**Result: 0 / 180 runs stood**, across all 9 configs and all three start
poses. Exact counts, from the logs:

| start | sequences x configs | runs | standing |
|---|---|---|---|
| supine (`getup_search_side_supine.txt`) | 6 variants x 9 configs | 54 | **0** |
| prone (`getup_search_side_prone.txt`) | 6 variants x 9 configs | 54 | **0** |
| side (`getup_search_side_side.txt`) | 8 variants x 9 configs | 72 | **0** |

No run in any grid reached the standing criterion (up-vector > 0.9 and
pelvis >= 80% of standing height) at the final step; the best final-step
up-vectors cluster at -0.09..+0.03 (i.e. lying flat or worse), confirmed by
scanning every run's last "up" value in the three logs -- there is no
near-miss hiding in the grid, this is a clean negative.

**But the FAILURE MODE is different from, and in one respect better than,
the bare-legs baseline**, and the numbers say why:

- **`bird_z06_sep160`, supine, splay +60 / ankle -40** (rendered,
  `getup_side_supine_strip.png`, `sim/renders/getup_side/bird_supine_best.mp4`):
  reaches **up +0.70 at the "push" step** (pelvis z 0.038 m, both feet AND
  the torso grounded) -- further than the bare-legs `supine_situp` baseline's
  "half rise" (up +0.30, docs \S11 table) at a comparable stage. It still
  collapses back to flat by "rise 3" (up +0.27 -> final +0.02): the extra
  height from the wider/lower hip mount buys a bigger rock but not a
  standing rise, because (\S2) the CoM-over-sole corridor still does not
  exist below 0.33 m regardless of mount point.
- **`bird_z15_sep160`, prone, splay +60 / ankle -40** (rendered,
  `getup_side_prone_strip.png`, `.../bird_prone_best.mp4`): "tuck knees
  under" lifts the pelvis to **0.170 m** (vs the bare-legs body's stuck
  0.09-0.17 m range for the same move, docs \S11/\S12.2) and "push pelvis
  up" reaches **0.193 m** -- genuinely more lever authority than the bare
  body, because the hip pivot is now much closer to the torso's own CoM.
  But that same shorter lever over-rotates: the torso's up-vector goes to
  **-0.99** (an inverted arch, front_z flips from -0.50 to +0.10) instead of
  stopping near vertical, and the sequence falls back to flat prone (final
  up -0.09). This reads as a **tuning failure** (the push travels too far
  for the new, higher gain from pivot-to-CoM), not a hard geometric
  impossibility like the bare-legs case -- a finer keyframe/gain search on
  this specific path was not run (time budget) and is the most promising
  unfinished lead from this study.
- **`side_l`/`side_r`** (rendered, `getup_side_fail_strip.png`,
  `.../bird_side_fail.mp4`): the leg-swing-across roll step (`R presses, L
  up+over`) spikes to **`|tau|max` 4.53 N-m** on the hip roll -- a self-
  collision-driven jam (the wide kick swing runs the leg into the torso or
  the other leg; without `self_collide=True` this would have silently
  passed through and looked clean). After the roll the sequence falls into
  the same `pushup_wide` chain and fails the same way as the prone case.

**Dynamic standing stability (informal check, not the walk gate):** all
tested `bird_*` configs hold a zero-command straight-leg stand for 3 s under
the full deploy servo model with `self_collide=True` (up-vector 0.999,
pelvis z matches `z_yaw_above_sole - hip_z` exactly, contacts = feet only,
no self-collision artifact breaks the pose) -- the raised/widened mount is
not itself unstable to stand on, only the RECOVERY path from a fall fails.

## 4. Walker impact (`getup_v6_side.py gate`, log `gateD_side.txt`)

Reused the exact 4 cases from `docs/design-v6/gateD_appendages.txt` (straight
mu 0.7 play 3, +15 deg turn, mu 0.3/play 5, -15 deg turn mu 0.9) for direct
comparability, on the plant at rest (no get-up sequence), STS3250 at
rolls+knees, 8 steps.

**`frog_splay_abd75` and `frog_splay_abd90` (hip_z = 0, splay only): every
number is IDENTICAL to the base body's row in `gateD_appendages.txt`**
(clear 22/16/23/23 mm, air 0.62/0.54/0.70/0.66 s, CoM margin 13.0/12.2/35.2/
26.6 mm, all 4 cases `OK`) -- widening the hip-roll ROM to 75-90 deg costs
the straight/turning walk nothing, because normal gait never uses that much
abduction.

**`hip_z != 0` configs (the actual bird/frog mount change): NOT MEASURED.**
`v6_kin.leg_ik`/`hip_roll_point` (Gate C/D's foot-placement analytic IK)
assume the hip YAW axis IS the pelvis/torso origin -- there is no `hip_z`
term in that math. Running the gate on `bird_z15_sep160` throws
`ValueError: out of reach: D 0.2218 > 0.2200` (confirmed, in `gateD_side.txt`
and reproducible directly) rather than silently returning a wrong pose.
Extending `v6_kin.py` to carry a hip-mount offset through the analytic IK
(and re-verifying `tests/test_v6_design_gates.py`, which pins numbers
through that same math) is out of scope for this study's time budget. The
walking cost of the actual raised/outboard hip mount is therefore an open
question, not a negative -- only the splay-only sub-case is cleared.

## 5. Hardware cost, if this were built

- **Hip mount / bracket**: `hip_z` > 0 requires relocating the hip-yaw servo
  and its carrier from the housing floor (today) to partway up, or above,
  the torso's own side wall -- a new structural bracket, not a parameter
  change on the current CAD. The torso shell would need a load path from
  that bracket down to the housing (today the housing floor IS the mount).
- **`hip_sep` 160-200 mm vs today's 84 mm**: the legs sit outboard of the
  torso's own side wall (59 mm half-width) by 21-41 mm. This widens the
  robot's footprint and standing profile; on the plus side it removes the
  reason `hip_sep` was narrowed to 84 mm in the first place (swing-foot
  drift under roll sag/play landing on the stance foot, `gen_plant_v6.py`
  comment) since the feet are now much farther apart regardless.
- **`hip_roll_abd` 75-90 deg vs today's 45 deg**: **not CAD-verified**. The
  appendage study only validated the plant's abduction to 55 deg
  (`docs/design-v6-ankle-roll.md` \S12.1, "the plant's current 45 deg
  abduction as well as 55 deg"), and the self-collision commit (759fe96)
  found that **both legs yawed out 45 deg together already collides foot-on-
  foot** in the CAD (4180 mm3 overlap) -- a ROM the one-joint-at-a-time gate
  sweep never caught. 75-90 deg abduction in this study is a SIM-ONLY range;
  whether the real hip yoke, servo case and thigh clear the torso and each
  other at that range is unknown/not measured and would need its own CAD
  ROM sweep before cutting anything.
- **Servo count/mass**: unchanged (12 servos, same mass model) -- `hip_z`
  and the splay range are pure geometry/ROM changes, no new actuator.
- **Walking**: splay-only is free (\S4); the raised/outboard mount's walking
  cost is not measured (\S4) and would need `v6_kin.py` extended before it
  can be.

## Files (round 1)

- `sim/gen_plant_v6.py` -- `hip_z` field (default 0.0, plant unchanged)
- `sim/getup_v6.py` -- `settle_fallen` gained `side_l`/`side_r` start states
- `sim/getup_v6_side.py` -- this study (`probe`/`search`/`robust`/`gate`/`render` modes)
- `docs/design-v6/getup_search_side_probe.txt` -- quasi-static probe log
- `docs/design-v6/getup_search_side_supine.txt`, `_prone.txt`, `_side.txt` -- dynamic search logs (0/180 standing)
- `docs/design-v6/gateD_side.txt` -- walk-gate log (splay-only cleared, hip_z skipped with a stated reason)
- `docs/design-v6/getup_side_supine_strip.png`, `getup_side_prone_strip.png`, `getup_side_fail_strip.png` -- filmstrips (best supine attempt, best prone attempt, a side-start failure)
- `sim/renders/getup_side/*.mp4` -- the matching videos (local only, gitignored)

## Verdict (round 1)

Moving the hip mount outboard and up the torso's side (the "bird/frog"
layout) does not, on its own, produce an open-loop get-up from supine,
prone or the side: 0 of 180 sequences tried in this study stood up, and the
same CoM-over-sole corridor wall measured on the bare-legs body (docs
\S12.2, no reachable pose below pelvis 0.33 m) holds regardless of where the
hip is mounted, because the change does not add reach to the thigh or
shank. It is not a clean negative like the bare-legs study, though: a low
mount (`hip_z` ~60 mm) keeps the classic sit-up and rocks further than the
baseline (up +0.70 vs +0.30) before falling back, and a higher mount
(`hip_z` ~150 mm) gives the prone push far more lever authority (pelvis to
0.193 m, more than the bare body ever reaches) but over-rotates past
standing into an inverted arch -- both read as unfinished tuning problems
rather than hard geometric dead ends, unlike the bare-legs case, and are the
two threads worth pulling if this direction continues. Splay alone
(`hip_roll_abd` to 75-90 deg) costs the straight/turning walk nothing in
sim, but is not CAD-verified and the self-collision work already found the
current 45 deg range colliding foot-on-foot at full symmetric yaw, so that
headroom is not free in hardware. The raised/outboard mount's own walking
cost could not be measured with the existing analytic-IK gate and is an
open question, not a pass.

---

# Round 2 (2026-09-16): a genuinely horizontal torso, the tuned prone push, and the walk gate

The coordinator's round-1 review, verbatim: "the proxy you tested keeps the
torso vertical, which is not the bird/frog layout Tom described, and the
report itself names two unfinished leads." Three follow-ups, in order.

## R2.1 A genuinely horizontal torso

**Plant change** (`sim/gen_plant_v6.py`): the root `torso` body -- both hip
yaw axes, the freejoint, the IMU site and the `torso_up`/`torso_pos`
sensors -- is **untouched**, so standing still means "hips vertical" and
`walker_env`/the get-up scripts' qpos assumptions hold exactly as before.
`_torso()` now emits the hip-yaw servo cases and the IMU site directly in
that root frame (unchanged formulas) and, new, three params:

- `torso_pitch` (deg, default 0.0): rotates a new child body `torso_block`
  -- which carries EVERYTHING else `_torso()` used to place (deck, housing
  or a round "belly", battery, Pi, driver, power, wiring, neck servo) --
  about its own Y axis. At 90 deg, local +Z (the old "up", where the head
  sits) becomes world +X (forward) and local +X (the old fore-aft depth,
  110 mm) becomes vertical (the body's height); the width (118 mm, local Y)
  is unchanged. `_head()`'s body gets the identical transform in its own
  sibling wrapper `head_block`, emitted after both legs as always -- two
  bodies with the same `pos`/`euler`, not one nested body, specifically so
  `neck_yaw`'s qpos slot stays immediately after the 12 leg joints (verified:
  `settle_fallen`'s `d.qpos[7:7+len(q_hold)]` write depends on that order,
  and joint order was checked directly against the compiled model, see
  below).
- `torso_block_x`, `torso_block_z` (m, default 0.0): where the block's
  former origin sits relative to the hip line.
- `torso_round` (bool, default False): replaces the deck+housing boxes with
  ONE capsule (same total mass, axis along the block's local Y, radius =
  half the local-X depth) -- "a bird body is not a box."

`torso_pitch/torso_block_x/torso_block_z` all 0.0 (the defaults) skips the
wrapper entirely and emits the ORIGINAL flat layout verbatim: `build_xml(
DesignParams())` SHA1 is unchanged from round 1. Two new configs:

| config | torso_pitch | torso_block_x/z | hip_sep | torso_round |
|---|---|---|---|---|
| `horiz_box` | 90 deg | -0.06 / +0.03 m | 0.20 m | False |
| `horiz_round` | 90 deg | -0.06 / +0.03 m | 0.20 m | True |

**Chosen `torso_block_x/z` and what they measure** (`mj_forward`, direct
Python one-liners, not a log file -- reproducible from the commit): with
`tx=-0.06, tz=+0.03`, whole-robot standing CoM is `x +0.0018 m` (box) /
`x -0.0003 m` (round) -- over the hip line to within 2 mm -- at `z 0.280 m`
(vs 0.292 m stock, the horizontal layout is a genuinely lower stand). The
`torso_block`'s own lowest geom corners sit 16-22 mm BELOW the hip-yaw
world height (`torso_housing` bottom z 0.3709 m vs hip line 0.3874 m; the
thin `torso_deck` box protrudes furthest, to -22 mm) -- "at or just below
hip-yaw level," as asked. `hip_sep 0.20 m` was chosen the same way the
brief asked ("measure, do not guess"): checked directly with a self-
collision contact count at the standing pose (`self_collide=True`), `ncon
0` for both `horiz_box` and `horiz_round` -- the legs clear the block's
sides with margin (hand geometry would say `hip_sep/2 - ~20 mm servo/
carrier > 59 mm deck half-width`, i.e. `hip_sep > 158 mm`; 200 mm was
simply the next round value already in the round-1 config set).

**Zero-command 3 s stand** (deploy servo model, `self_collide=True`):
both `horiz_box` and `horiz_round` hold `pelvis z 0.387` (== `z_yaw_above
_sole`, i.e. the hip line, exactly as the stock body does), `up 1.000`, `x`
drift up to 6 mm, contacts feet only -- the horizontal torso stands fine on
its own; the get-up is the open question, as with every other config here.

**Dynamic get-up search** (`getup_search_side_h_{supine,prone,side}.txt`),
same `seq_situp_wide`/`seq_pushup_wide`/`full_from_side` sequences as round
1 (unchanged code, just new configs added to the grid): **0/12 supine,
0/12 prone, 0/16 side stood**, for BOTH `horiz_box` and `horiz_round`. The
mechanism, from the logs:

- **Supine** (`getup_search_side_h_supine.txt`, rendered
  `getup_side_h_supine_strip.png` / `sim/renders/getup_side/horiz_supine
  _best.mp4`, `horiz_box` splay +60/ankle -40): "sit up" still reaches
  **up +0.99** (the classic sit-up survives a horizontal torso, unlike the
  raised-hip-z bird bodies at hip_z >= 100 mm in round 1) and "push" reaches
  **up +0.54** (box) / **+0.52** (round) -- less far than round 1's best
  (`bird_z06_sep160`, +0.70), then falls back to flat (+0.00 / +0.07 final).
- **Prone** (`getup_search_side_h_prone.txt`): `horiz_box`'s "push pelvis
  up" over-rotates to **up -1.00** exactly like `bird_z15_sep160` in round
  1, then settles back to flat prone (+0.07 final). `horiz_round` is
  WORSE, not better: it gets stuck **inverted** (up -0.96, contacts
  `head+torso` for the rest of the sequence, never recovering even to
  flat) -- the round belly's better roll behaviour (below) comes at the
  cost of a push-up that, once it over-rotates, has nothing flat to catch
  it and settle on.
- **Side** (`getup_search_side_h_side.txt`): identical outcome to prone
  (the roll-to-prone step lands the same push-up chain and fails the same
  way).

**Supine roll-over, as asked** ("if the body cannot roll, say so with the
number"): `getup_v6_side.py roll` swings one leg via hip roll+yaw and
tracks the PEAK attitude change from the settled supine pose plus peak
servo torque (`getup_search_side_h_roll.txt`, rendered
`getup_side_h_roll_strip.png` / `.../horiz_roll_fail.mp4`, `horiz_round`
L-swing 60/45). **It cannot roll.** Across `horiz_box`, `horiz_round`,
`bird_z15_sep160` and `bird_z06_sep160`, 4 swing/yaw combinations x 2 sides:

| config | best L-swing tilt | R-swing tilt | R-swing peak torque |
|---|---|---|---|
| `horiz_box` | 24.4 deg (swing 60, yaw 45) | 3.1-6.6 deg | 2.19-2.72 N-m |
| `horiz_round` | **27.2 deg** (swing 60, yaw 45) | 6.2-10.2 deg | 2.18-2.72 N-m |
| `bird_z15_sep160` | 26.8 deg | 4.0-7.7 deg | 2.72 N-m (saturated) |
| `bird_z06_sep160` | **29.2 deg** (best of all four) | 6.5-9.6 deg | 2.71-2.72 N-m |

R-side swings hit the servo torque ceiling (2.71-2.72 N-m -- the model's
clamp, i.e. the joint is jammed, almost certainly a self-collision the
swing runs into) while barely moving the body (3-10 deg); L-side swings
move more (15-29 deg) without saturating (0.86-1.17 N-m) but never get past
about 30 deg -- a third of the way to lying on the side, nowhere near the
roughly 90-180 deg a supine-to-prone roll needs. `torso_round` DOES help
here, consistently (+3 to +5 deg of tilt over the equivalent box config at
every setting) -- the one place in this study where the round belly's
intended effect shows up in the numbers -- but the effect is small next to
the gap that needs closing.

## R2.2 Tuning the over-rotating prone push

Round 1 found `bird_z15_sep160`'s prone push travels PAST standing into an
inverted arch (pelvis 0.193 m, up -0.99) rather than stopping short. A
finer sweep (`getup_v6_side.py tune`, `getup_search_side_h_tune.txt`) opened
the knee-extension target (`knee_push`, round 1 fixed it at -40 deg; -130
is the fully tucked start), the ankle angle, and the push travel/hold time,
scored on final up-vector (mild penalty for residual front-pitch), for
`bird_z15_sep160`, `horiz_box` and `horiz_round`:

| config | best knee_push | ankle | t_push | hold | result |
|---|---|---|---|---|---|
| `bird_z15_sep160` | -80 deg | -25 | 2.0 s | 1.5 s | up -0.09, z 0.059 (flat, not standing) |
| `horiz_box` | -80 deg | -25 | 3.0 s | 0.5 s | up +0.07, z 0.108 (flat prone, NOT inverted) |
| `horiz_round` | -100 deg | -25 | 3.0 s | 0.5 s | up +0.07, z 0.108 (flat prone, NOT inverted) |

A smaller push travel (-80/-100 deg instead of -40, i.e. LESS knee
extension out of the -130 tuck) does what it was supposed to: `horiz_box`
and `horiz_round` no longer over-rotate into the inverted arch (rendered,
`getup_side_h_prone_strip.png` / `.../horiz_prone_tuned.mp4` -- "push
pelvis up" now bottoms out at up -0.93 with the HEAD grounding briefly,
not the -1.00 full inversion, and recovers to flat prone rather than
staying stuck). It still does not reach standing on any of the three
bodies -- removing the overshoot removes the arch, it does not by itself
add the lift the pelvis needs. `bird_z15_sep160`'s own best case is, if
anything, a wash (up -0.09 vs round 1's own best of -0.09 too at a
different point in that grid) -- the tuning helped the horizontal bodies
avoid the WORST outcome, not get closer to the best one.

## R2.3 Walk gate: hip_z added to the analytic IK

**Patch** (`sim/v6_kin.py`, `sim/design_gates.py`, 3 lines): `hip_roll_point`
and `pose_from_feet`'s `hip_yaw_pt` now add `p.hip_z` to the pelvis-frame Z
coordinate (the hip yaw axis sits `hip_z` above the torso origin, exactly
gen_plant_v6's own convention); `standing_key`'s `z_pel` now subtracts
`p.hip_z` too (the torso ORIGIN, not the yaw axis, is what gets written to
`qpos[0:3]`). **`tests/test_v6_design_gates.py`: 8/8 pass, unchanged**
(hip_z defaults to 0.0, so every pinned number is untouched).

**Gate D re-run, all configs** (`docs/design-v6/gateD_side_h.txt`), same 4
cases as before:

- **`hip_z != 0` now RUNS** (round 1 could not run it at all -- confirmed
  fixed). But the walk margins are bad: `bird_z06_sep160` straight clears
  8/8 (CoM margin 29.5 mm) but the +15/-15 turns now exceed the leg's reach
  (`D 0.2218-0.2221 > 0.2200 m`) and the mu-0.3 case FAILS (min clear 12 mm,
  slip inside the tolerance but flagged `FAIL` by the gate's own criterion).
  `bird_z10_sep160`/`bird_z15_sep160` straight walk STANDS but with CoM
  margin **3.8 mm / -1.3 mm** (near-zero or negative -- a fall waiting to
  happen) and tilt 11.2-11.3 deg (vs 5.7 deg stock). `bird_z20_sep160`
  outright **FALLS at 1.5 s, 0/8 steps**. Raising the hip mount does not
  just fail to help the walk -- it actively erodes the margins the whole
  Gate D exercise exists to protect, worse the higher `hip_z` goes.
- **Any `hip_sep >= 0.18` m fails the IK on EVERY case, including
  straight**, independent of `hip_z`: `bird_z15_sep200`, `bird_topmount`,
  `horiz_box`, `horiz_round` all throw `out of reach` (D 0.220-0.225 m >
  0.220 m) on all 4 cases. Isolated test (`hip_sep` swept 0.084-0.20 m,
  `hip_z=0`): 0.084-0.16 m walk straight fine, 0.18-0.20 m fail straight
  too. This is NOT the hip_z patch's doing -- it is the existing gait
  planner's foot-placement offsets (`bias_y`/`swing_out`/`turn` in
  `static_gait.walk_timeline`), tuned around `hip_sep 0.084 m`, asking for
  more diagonal reach than a straight 0.220 m leg has once the stance is
  that wide. Retuning the gait for a wide stance is a real, separate piece
  of work, not attempted here.
- `frog_splay_abd75/90` (hip_z=0, hip_sep=0.084, splay only): unchanged
  from round 1, identical numbers, all 4 cases OK.

**So: the horizontal torso's own walking cost is now measurable, and it is
a clear NO with the current gait planner** -- not from anything about the
torso's shape or mass, but from `hip_sep 0.20 m` (needed for leg-body
clearance) exceeding the walk's foot-placement reach outright. A narrower
horizontal-torso variant (say `hip_sep` at the 0.16 m ceiling found above)
was not built or tested -- time did not allow it, and it would still need
the gait's offsets checked against the block's forward-reaching mass, not
only the leg-reach number.

## Files (round 2 additions)

- `sim/gen_plant_v6.py` -- `torso_pitch`/`torso_block_x`/`torso_block_z`/`torso_round` (defaults 0.0/0.0/0.0/False, plant unchanged, SHA1-verified)
- `sim/v6_kin.py`, `sim/design_gates.py` -- `hip_z` added to the analytic leg IK (3 lines); `tests/test_v6_design_gates.py` 8/8 unchanged
- `sim/getup_v6_side.py` -- `horiz_box`/`horiz_round` configs, `seq_pushup_tuned`, `seq_roll_over`/`roll_check` (new `roll` mode), `tune` mode, `gate` mode no longer skips `hip_z != 0` (catches and reports `IK REACH EXCEEDED` instead)
- `docs/design-v6/getup_search_side_h_{supine,prone,side}.txt` -- horizontal-torso dynamic search (0/40 standing)
- `docs/design-v6/getup_search_side_h_roll.txt` -- supine roll-over attempt (peak tilt/torque, cannot roll)
- `docs/design-v6/getup_search_side_h_tune.txt` -- prone-push tuning sweep
- `docs/design-v6/gateD_side_h.txt` -- walk gate re-run with hip_z supported, all configs
- `docs/design-v6/getup_side_h_supine_strip.png`, `getup_side_h_prone_strip.png`, `getup_side_h_roll_strip.png` -- filmstrips (horizontal-torso best supine attempt, tuned prone push, roll-over failure)
- `sim/renders/getup_side/horiz_*.mp4` -- the matching videos (local only, gitignored)

## Results table (round 2)

| layout | start | best result | mechanism |
|---|---|---|---|
| `horiz_box` | supine | up +0.54, z 0.059, not standing | classic sit-up survives (+0.99), push falls short of round 1's best low-mount bird |
| `horiz_round` | supine | up +0.52, z 0.059, not standing | same |
| `horiz_box` | prone | up -1.00 (untuned) / +0.07 (tuned), not standing | untuned over-rotates into an arch; tuned avoids it but stalls flat |
| `horiz_round` | prone | up -0.96, STUCK inverted (untuned); +0.07 (tuned) | round belly's push has nowhere flat to catch an over-rotation |
| `horiz_box`/`horiz_round` | side | same as prone (rolls into the same push-up chain) | not standing |
| `horiz_box`/`horiz_round`/`bird_z*` | supine roll | peak tilt 3-29 deg, cannot reach ~90-180 deg needed | R-swings jam at 2.7 N-m; `torso_round` adds +3-5 deg of tilt over the box, consistently but not enough |
| `bird_z06/10/15/20` (walk gate) | -- | straight margin 29.5 -> 3.8 -> -1.3 mm -> FALLS as hip_z rises 60->200 mm | raising the hip mount erodes walk margins monotonically |
| any `hip_sep >= 0.18 m` (incl. both horizontal configs) | walk gate | IK reach exceeded on every case | existing gait planner's foot offsets don't reach at this stance width |

## Verdict (round 2, final)

A genuinely horizontal torso -- built by rotating a new `torso_block`
90 deg while leaving the root frame, legs and sensors untouched, verified
against a SHA1-identical default plant and an unchanged pinned test suite
-- stands on its own (zero-command, 3 s, `up` 1.000) with its CoM within
2 mm of the hip line and 12 mm lower than the stock body, but still cannot
get up: 0 of 40 dynamic attempts across supine, prone and the side stood,
the supine roll-over the brief specifically asked about tops out at 27-29
degrees of tilt against the roughly 90-180 needed (measured, not guessed,
with R-side swings jamming at the servo's torque ceiling), and a rounded
belly helps that roll by only 3-5 degrees. Tuning the prone push's travel
does fix the round-1 over-rotation (no more inverted arch) on both the
horizontal body and the original `bird_z15_sep160`, which confirms that
finding was a control problem rather than a dead end, but it does not by
itself produce a stand either. The walk gate, now able to run `hip_z != 0`
cases at all (a 3-line, test-verified fix), delivers a clean answer where
round 1 had none: raising the hip mount steadily erodes the walk's margins
even where it is kinematically reachable, and the wide stance (`hip_sep
0.20 m`) every bird/horizontal variant in this study needs for leg-body
clearance exceeds the existing gait planner's foot-placement reach
outright, on every turning case tested and even walking straight. Taken
together, round 1 and round 2 point the same way: the "bird/frog" family's
individual mechanisms (a lower CoM, a shorter hip-to-head lever, a wider
stance) are each real and measurable, but none of the four bodies built
across both rounds crosses the standing threshold, and the layout's own
wide stance is now the walker's blocker as much as the get-up ever was.

---

# Round 3 (2026-09-17): the actual bird body -- a flat slab, "closer to the original bimo inspiration"

Tom rejected round 2: *"I don't think we really studied the bird body. With
double jointed knees and the COM being lower, I find it difficult to
believe that it couldn't recover from being on its back... and it's less
likely to end up in that situation to begin with. I'd expect a flattened
body, parallel to the ground, something closer to the original bimo
inspiration."* The-bimo-project (github.com/mekion/the-bimo-project) is an
8-servo, 45 cm, ~1.6 kg "hip-head biped" -- the body is a payload pod
sitting ON the hips, no tall torso. Round 2's "horizontal torso" was the
existing vertical stack tipped on its side (110 mm tall, a separate head
still on the end, human-only knee, stock 45 deg abduction) -- not that.

This worktree was first brought up to date with `v6-getup-legs-skid`
(`git merge`, fast-forward, no conflicts -- it already contained this
worktree's round 1/2 commits plus a separate shoulder-arm study, merge
commit `1a0688b`; default plant and pinned tests unaffected, re-verified).

## R3.1 The model: one flat slab

New `DesignParams.bird_body` (default `False`, plant unchanged -- SHA1
verified) replaces the WHOLE torso_v7 stack + head with one flat slab,
centred on the hip line: `bird_L` 0.20 m (length/fore-aft, camera on the
front +X face), `bird_W` 0.14 m (width), `bird_H` 0.055 m (height). Unlike
round 2's `torso_pitch` rotation, this shape is authored flat directly (no
rotation wrapper needed) and `hip_z` stays at its default 0.0 -- the slab
is centred at local z = hip_z by construction, so "hips at the slab's
mid-height" holds for any hip_z, and the root torso frame, both hip yaw
axes, the IMU site and the `torso_up`/`torso_pos` sensors are the SAME
code, untouched (`_torso()` gained an early `if p.bird_body` branch;
`_head()` returns `""` -- no separate head, no neck servo, the slab IS the
head, matching Bimo's pod). Battery (170 g) and Pi (66 g) are boxes at the
slab's mid-height; the pelvis print mass (`m_pelvis`, 174 g) is the shell
itself -- a box, or (`torso_round=True`) one ellipsoid with the same outer
envelope, rounding every edge so it can roll onto any face, not just tip
over a flat bottom. Driver/power/wiring/neck-servo/head are NOT modelled
(Tom's list did not include them) -- **total mass 1.436 kg vs the stock
body's 1.650 kg**, 214 g lighter, all of it electronics/head mass, not
servos (still 12 leg actuators, no change).

**Hip mount**: the hip-yaw servo cases sit at the slab's sides at
mid-height (same code as the vertical body's yaw-servo emission, just at
`z = hip_z` instead of a housing-relative offset). `hip_sep = 0.18 m`,
derived as `bird_W + SV_WID + 2 x 0.006 m clearance = 0.14 + 0.0247 + 0.012
= 0.177 m`, rounded up to 0.18. This pair (slab shell, yaw servo case) is
one rigid body in MuJoCo -- `self_collide`'s contact count cannot check it
(same-body contacts are never computed), so the clearance is the geometric
derivation above, not a measurement; what IS measured is 0 self-collision
contacts for the LEG (a separate, jointed body) against the slab at the
standing pose, confirming the derivation didn't leave the legs fouling the
slab. Legs: current thigh/shank/foot (0.110/0.110 m), `knee="both"`
(+-130/+95, the CAD ROM already in the plant), `hip_roll_abd` swept 90 and
120 deg, `yaw_range` 180 deg (the servo's own limit, matching
`joint_puppet.py`'s 09-16 correction). Hip pitch was left at (-125, 90) as
now -- the frog get-up sequences below did not run into that limit (they
use hip roll/yaw/knee, not deep hip pitch, except the "stand up inverted"
variant, tested at hip pitch +-90/+120 as its own swept variable, R3.3c).

**Measured with `mj_forward`** (direct one-liners, reproducible from the
commit):

| quantity | value |
|---|---|
| total mass | 1.436 kg (stock: 1.650 kg) |
| standing CoM height | 0.247 m (stock: 0.292 m) |
| slab bottom above floor, standing | 0.360 m |
| **supine CoM height, legs neutral/straight** (worst case: legs held at the commanded-straight pose, which after a 180 deg roll point straight UP) | **0.167 m** |
| **supine CoM height, legs folded** (hip roll 90 deg both, knee 45 deg -- a realistic settle, legs splayed flat rather than commanded rigid) | **0.051 m** |
| **prone CoM height, legs folded** (hip roll 90 deg, knee -90 deg) | **0.041 m** |
| stock body supine CoM height (its own natural fall, 90 deg pitch) | 0.055 m |
| stock body prone CoM height | 0.063 m |

**This is the number the whole argument rests on, and it depends on the leg
assumption.** With legs held rigidly straight (the worst case, as if the
robot fell without ever moving its legs), the flat body's "on its back" CoM
(0.167 m) is HIGHER than the stock body's (0.055 m) -- the straight legs
stick up into the air after the 180 deg roll and dominate the height. But
with the legs folded/splayed (the realistic settle -- see R3.3, this is
also the pose the frog get-up starts from), the flat body's CoM drops to
0.051 m supine / 0.041 m prone, LOWER than the stock body's 0.055/0.063 m.
**Tom's "the CoM is lower" claim is confirmed, but only once the legs fold;
a fall that freezes the legs straight up is the one case where it is not.**

Coordinator review after the first stills: model approved ("matches the
brief"); two corrections applied before the get-up runs: (1) the prone
start settles from `BIRD_PRONE_FOLD` (hip roll 90, knee -90) instead of
straight legs, so the slab lands flat (up +1.00) instead of tipping onto an
edge (the original still, `bird3_model_prone.png`, showed the untipped
case -- kept as the "what NOT folding gives you" reference); (2) every
supine sequence below was run from BOTH `BIRD_SUPINE_STRAIGHT` (CoM 0.167)
and `BIRD_SUPINE_FOLD` (hip roll 90, knee 45, CoM 0.051), reported
separately.

**Stills** (`sim/renders/getup_options/side/bird3_model_*.png`, sent to the
coordinator before the sweeps as asked): `bird3_model_stand_front.png`,
`_stand_side.png` (standing), `_supine.png` (on its back, straight legs --
shows the legs pointing up), `_prone.png` (the un-corrected tip-onto-edge
case, kept for reference).

## R3.2 Get-up: hand-built sequences all fail; a continuous search finds a stand

**Supine, hand-built (a)/(b)/(d)** (`getup_search_bird3_supine.txt`, 226
runs x 2 configs x 2 fold states): frog-lift-and-roll (a), yaw-first (b),
sit-up-style hip-pitch (d) -- **0/~900 stood**, both fold states, both
`bird3_box`/`bird3_round`. From `BIRD_SUPINE_STRAIGHT`, the "lift" step
barely engages the floor at all (`|tau|max` 0.01-0.11 N-m -- the legs,
splaying out from a straight-up start, do not reach the ground within the
tested ranges) and the body stays inverted (up -1.00) throughout. From
`BIRD_SUPINE_FOLD` the legs DO reach the floor, but no combination of
lift/push-side/push-amount rolls the slab past being knocked flat again
(front +-0.99, CoM height stuck at 0.027-0.035 m -- lower than the fold's
own start height in most cases, meaning the "lift" often pushes the slab
flatter against the ground rather than rolling it).

**Continuous keyframe search finds it** (`getup_v6_side.py bird3search`,
hill-climb with 6 restarts, 6-DOF symmetric hip_yaw/hip_roll/knee/ankle
nodes, same method as `getup_v6_legs.py`'s round-1 search, run separately
from the straight and folded starts):

| config | fold | best score | up | pelvis z | standing? |
|---|---|---|---|---|---|
| `bird3_round` | folded | 6.09 | **+1.00** | **0.386 m** | **YES** (>0.9 up, >=0.8x0.387m) |
| `bird3_round` | straight | 6.10 | **+1.00** | **0.387 m** | **YES** |
| `bird3_box` | folded | 4.66 | +1.00 | 0.209 m | no (short of the 0.310 m height threshold) |

**This is the positive result Tom expected: an open-loop path exists from
"on its back" to standing on this body, and the hand-built sequences above
simply did not find it** (`getup_search_bird3_hillclimb_round_{folded,
straight}.txt`; the winning path is logged with the run). Two of six
restarts converge to essentially full standing height (0.386-0.387 m out
of 0.387 m possible) from BOTH the straight-leg and folded-leg supine
starts -- the double-jointed knee's extra ROM (the search's bounds include
the full -130..+95 deg range) is exactly what a hand-authored sequence
tends not to explore. The other four restarts land on lower local optima
(0.096-0.189 m, still upright-leaning but short of standing), consistent
with a genuinely hard search landscape rather than a trivial path.

**The rounded slab succeeds where the box does not, on the identical
search.** The same folded-start search on `bird3_box` (box shell, same
mass, same everything else) tops out at up +1.00 but pelvis **0.209 m**
(`getup_search_bird3_hillclimb_box_folded.txt`, best of 6 restarts,
`STANDING=False`) -- short of the 0.310 m threshold. This is the same
direction as round 2's roll-tilt result (the round shape adding 3-5 deg of
tilt over the box): a rollable shell does not just help a partial roll, it
is the difference between the search finding a full stand and getting
stuck partway, on this specific body.

**Prone** (`getup_search_bird3_prone.txt`, 54 runs x 2 configs): **0/108
stood.** The push-up itself works well -- from `BIRD_PRONE_FOLD`, extending
the folded knees reaches up +0.99-1.00 with the slab flat and feet+thighs
grounded (`|tau|max` <0.5 N-m) -- but EVERY attempt topples during the
transition from the wide push-up stance to a narrower standing stance
(measured directly: splay 90 deg -> 45 deg alone drops up from 0.99 to
0.43; the two-stage narrowing added to `seq_bird_pushup` after that
discovery did not fix it, up still falls to 0.27-0.33 by the final "stand"
step, converging to the SAME resting pose regardless of the push
parameters swept). The push-up's wide stance is stable but short (pelvis
only 0.061 m at full push, since much of the leg's length goes into the
splay rather than height); reaching stand height needs the narrow,
upright leg configuration, and that specific transition is the failure
point, not the push itself.

**Side** (`getup_search_bird3_side.txt`, `side_l`/`side_r` settle +
push-up): **0/16 stood**, same narrowing-topple mechanism as prone.

**Robustness**: not run -- nothing in the hand-built grids stood to test,
and the search's winning path was found (and is reported) too close to
this study's time budget to also robustness-sweep; noted as the natural
next step, not measured here.

## R3.3 Fall census

`getup_v6_side.py falls`: from a zero-command stand, a velocity-kick
impulse (mass-normalized -- same m/s for both bodies, so momentum scales
with mass automatically) at 12 headings x 3 magnitudes (tip speed, found by
binary search per body, then 1.5x/2x), 3 s settle under the full deploy
model, classified by which of the torso's own local axes ends up vertical
(`classify_fall`, `flat=True` for the slab: only the THIN axis (Z)
counts as a flat rest -- standing (up) or supine (up negative); the
slab's LENGTH or WIDTH axis ending up vertical is an `edge` rest, not a
stable flat state, unlike the tall body where the length axis IS the
prone/supine axis). Tip speed: `bird3_round`/`bird3_box` 0.66 m/s, `stock`
0.57 m/s -- **the flat body needs 16% more speed to tip over in the first
place**, Tom's second claim, measured (`getup_search_bird3_falls.txt`).

| end state | bird3_round (36) | bird3_box (36) | stock (36) |
|---|---|---|---|
| standing | **12 (33%)** | **12 (33%)** | 8 (22%) |
| edge | 24 (67%) | 24 (67%) | 0 |
| supine | 0 | 0 | 11 |
| prone | 0 | 0 | 7 |
| side_l | 0 | 0 | 5 |
| side_r | 0 | 0 | 5 |

**Both claims hold in this census.** The flat body recovers to standing
more often after a perturbation (33% vs 22%) and NEVER lands flipped flat
onto its back (0/36 supine, vs 11/36 for the stock body) -- every non-
recovery lands "edge" (resting on a length or width edge, tipped, neither
flat nor upright). Whether "edge" is easier to recover from than the stock
body's prone/supine/side states is not measured here (would need the same
get-up-sequence treatment R3.2 gave supine/prone) -- worth flagging since
"edge" is 67% of this body's non-standing outcomes.

**Mid-stride** (`getup_search_bird3_falls_midstride.txt`, legs held in an
asymmetric walking pose, one fixed 0.5 m/s impulse, 12 headings): `bird3_
round` lands "edge" 36/36; `stock` lands "side_r" 36/36 (both fully
deterministic at this fixed speed/pose combination -- the mid-stride
asymmetry dominates the heading-dependence that showed up in the neutral-
stance census).

## R3.4 Walk gate

`v6_kin.leg_ik` does not accept `knee="both"` (only `"fwd"`/`"bwd"` pick a
solution branch -- a real gap from the ad440f0 merge, not touched before
now); the walk-gate IK uses `knee="fwd"` for its own solve (normal walking
never asks for the backward ROM anyway, so this does not change the
simulated gait).

**hip_sep 0.18 m exceeds the gait planner's reach on every case, including
straight** (confirmed: `step`, `lift_h`, `swing_out`, `land_out`, `bias_y`
and `drop` were all sept and NONE materially changed the required reach --
`D` stayed at 0.221-0.232 m against a 0.220 m limit regardless; the reach
is set by the LATERAL weight-shift-to-stance-foot distance, which scales
with `hip_sep` directly, not by any timing/offset parameter). Per the
coordinator's instruction, **the fix is a narrower gait footprint, not a
narrower hip mount**: the walk-gate IK now solves for a SEPARATE, narrower
`hip_sep` (found by trying 0.18/0.16/0.14/0.12/0.10 until a turning case
resolves; 0.14 m worked for `bird3_*`) while the simulated PHYSICS still
uses the real, wide-hip XML -- the resulting joint angles adduct the real,
wide-mounted legs to a narrower footprint (an intentional, adducted,
knock-kneed-style gait), not a change to the mechanical design.

| body | case | clear | air | CoM margin | tilt | result |
|---|---|---|---|---|---|---|
| `bird3_round`/`bird3_box` (hip_sep 0.18, gait IK 0.14) | straight | 0 mm | 0.00 s | **-2.9 mm** | 6.3 | FAIL |
| | +15 turn | 0 mm | 0.00 s | **-3.5 mm** | 6.4 | FAIL |
| | mu 0.3/play 5 | 1 mm | 0.00 s | **-1.2 mm** | 6.0 | FAIL |
| | -15 turn, mu 0.9 | 0 mm | 0.00 s | **-4.3 mm** | 6.8 | FAIL |
| `stock` (hip_sep 0.084) | straight | 22 mm | 0.62 s | 13.0 mm | 5.7 | OK |
| | +15 turn | 16 mm | 0.54 s | 12.2 mm | 7.3 | OK |
| | mu 0.3/play 5 | 23 mm | 0.70 s | 35.2 mm | 3.3 | OK |
| | -15 turn, mu 0.9 | 23 mm | 0.66 s | 26.6 mm | 6.1 | OK |

**Even with the narrowed-gait workaround, the flat body's walk margins are
negative on all 4 cases** (`gateD_bird3.txt`) -- zero foot clearance, zero
air time (the swing foot barely leaves the ground at this narrow a
footprint relative to the wide hip mount), and negative CoM margin (the
kinematic CoM solve cannot find a fully safe pelvis position at this
adduction). This is a genuine, measured negative for the flat body's
current dimensions/hip_sep, not an artefact of the workaround being
approximate -- both `bird3_round` and `bird3_box` give IDENTICAL numbers
(the shell shape does not affect the rigid-body walk kinematics).

## R3.5 Hardware cost

- **Hip stack at the slab sides**: the yaw servo relocates from the
  housing floor (today) to the slab's own side wall at mid-height -- a new
  bracket integrated into the slab shell, not a parameter change on
  existing CAD. The yaw-roll-pitch stack below it (d_yaw_roll + d_roll_pitch
  = 91 mm, unchanged from every other v6 body) still hangs below the slab,
  visible in the stills as the same "hip stack gap" the original vertical
  body has.
- **hip_sep 0.18 m** (vs 0.084 m stock): legs mount 21 mm outboard of the
  slab's own half-width (0.07 m) plus the yaw servo's own width, by
  derivation (not self-collision-measured, see R3.1) -- widens the
  footprint substantially.
- **`hip_roll_abd` 90-120 deg, `yaw_range` 180 deg**: same CAD-unverified
  caveat as round 1/2 -- the appendage study only validated to 55 deg
  abduction, and the self-collision work found the stock 45 deg range
  already colliding foot-on-foot at full symmetric yaw. 90-120 deg and full
  180 deg yaw are sim-only ranges here; a real hip yoke/servo-case/thigh
  clearance sweep at THESE ranges was not done.
- **`knee="both"`**: the CAD ROM (-130/+95) is already in the plant
  (2026-09-16 merge) -- no new hardware, but confirms the double-jointed
  knee this study leans on is not itself a new ask.
- **Mass/servos**: 1.436 kg, 12 servos (unchanged count) -- 214 g LIGHTER
  than stock because the driver/power/wiring/neck-servo/head are not
  modelled in this pod (Tom's spec: battery + Pi + camera only). If those
  are needed on the real robot, they will have to fit inside or on the
  55 mm-thick slab -- not attempted here.
- **Walking**: R3.4's negative is the flat body's real, current
  cost -- not a modelling gap. Fixing it needs either a narrower hip_sep
  (undercutting the leg-clearance derivation in R3.1) or a gait planner
  that plans a genuinely adducted stance as a first-class feature rather
  than a post-hoc IK trick.

## R3.6 Verification of the found path (coordinator, 2026-09-17)

`sim/getup_v6_bird3_verify.py` replays the search's BEST path
(`getup_search_bird3_hillclimb_round_folded.txt`) with a 2 s final hold and
answers the three questions R3.2 left open (log
`docs/design-v6/getup_search_bird3_verify.txt`, 16/16 STANDING):

- **Quasi-static, not a flip.** At the search timing (1.4 s moves / 0.6 s
  holds) and at 3x slower (4 s / 2 s) the per-keyframe trace is the same:
  lie (up -1.00, pelvis 0.028) -> k1 slab pitched up onto its rear edge
  (up -0.24, front +0.97, pelvis 0.097) -> k2 falling over (up +0.42) -> k3
  flat on the belly on feet + thighs (up +0.98, pelvis 0.073) -> k4 crouch on
  the feet alone (pelvis 0.144) -> k5 stand (pelvis 0.386), held. Peak
  |tau| 1.43-1.49 N-m at a hip ROLL (STS3250 joint), never on a 3215.
- **Robust 7/7**: play 3/5 deg, mu 0.3/0.7/1.0, servos 100/80/65 % and the
  worst combination (play 5, mu 0.3, 65 %) all end STANDING at 0.386 m.
- **Prone is solved by the same path.** From the flat prone settle
  (`BIRD_PRONE_FOLD`) the second half, k3..k5, stands the body up nominal +
  5/5 robust; the FULL path from prone also stands (the k0-k2 edge-pitch
  simply re-lands it on its belly). R3.2's hand-built push-ups toppled
  because they narrowed the 90 deg splay AFTER rising; this path narrows at
  the crouch (k4, pelvis 0.144) before it rises. Render:
  `sim/renders/getup_options/side/bird3_prone_stand.mp4`.

So on the rounded flat body the open-loop recovery is supine -> standing
and prone -> standing, robust, one sequence. Still open: the "edge" rest
state (67 % of the fall census) has not been given a sequence, the walk
gate is negative at hip_sep 0.18 (R3.4), and the abduction/yaw ranges are
sim-only until a CAD hip is drawn.

## R3.7 The pincer on the v7 torso

Tom, watching `bird3_supine_best.mp4`: *"I see the getup for the bird starts
with legs outstretched, and then brings them together in a pincer
movement. Why wouldn't a similar concept work for the v5 two legged walking
torso?"* The round-1 legs-only search on the stock v7 body (docs sec 12.2)
only explored symmetric hip_pitch/knee/ankle with `knee="fwd"` and
`hip_roll_abd=45` -- the pincer (hip yaw + roll sweeping from splayed-in-
the-floor-plane to together, backward knee) was never in that search
space. Tested here with the SAME sim-only ranges the bird got
(`knee="both"`, `hip_roll_abd=120`, `yaw_range=180`) on the stock torso
(`bird_body=False`, everything else default), same 6-node hill-climb, same
scoring, 6 restarts x 2 fold states, plus the bird's own winning path run
VERBATIM (`getup_search_pincer_stock.txt`).

**The bird's exact path, run unperturbed on the tall torso, fails
outright** (both fold states give the identical trace): it never gets past
a rocking motion (up swings +0.43 -> +0.37 -> -0.27 -> +0.37 -> +0.07 with
a **4.48 N-m torque spike** on `R_hip_roll` at k3 -- more than double the
STS3250's rated range, a genuine jam, not a controlled move) and ends
`up -0.02, pelvis z 0.060` -- on its side, not standing. The pincer motion
itself (splayed hip roll closing toward the centreline) is what the search
found FOR the slab's specific mass distribution (CoM near the hip line);
run on a torso whose CoM sits far out along what is now the horizontal
long axis, the same joint angles put reaction forces through the hips that
the servo model cannot deliver.

**A fresh hill-climb, given the same freedom, does better but still does
not stand**: best of 12 restarts (`straight` fold) reaches **up +1.00,
pelvis z 0.211 m** (score 4.69, `STANDING=False` -- the criterion needs
pelvis >= 0.8 x 0.387 = 0.310 m). The per-keyframe trace
(`getup_search_pincer_stock.txt`) shows exactly where it stalls: by k4 the
torso is already fully vertical (`up +1.00`) at pelvis **0.051 m** (feet +
thighs grounded) -- the pitch-up itself succeeds completely, max torso
pitch-up angle reached is the full 90 deg (up 0.03 -> 1.00) -- but k5's
rise only lifts the pelvis to 0.211 m, 0.176 m short of standing height,
at 1.54 N-m on `R_hip_roll`. **The pincer gets the tall torso upright; it
does not get it tall.** The bird's identical search (bounds, scoring,
restarts) reaches 0.386/0.387 m on the slab from the same starting
attitude budget.

**The geometric answer, measured** (`mj_forward`, both bodies supine, each
in its own correct fall orientation, legs folded/settled): the whole-robot
CoM's straight-line distance from the hip axis ("radius" of the rotation a
sit-up-style pincer performs) is **0.096 m for the stock body** (CoM
`[0.096, 0, 0.055]`, hip point `[0, 0, 0.057]` -- the CoM sits almost
exactly at hip HEIGHT, 96 mm out along the now-horizontal torso) vs
**0.036 m for the bird slab** (CoM `[0.044, 0, 0.051]`, hip point
`[0.017, 0, 0.027]` -- already 23 mm above the hip line before anything
moves). The stock body's CoM has to sweep through a **2.7x larger radius**
to get overhead the hip pivot, because the mass that used to be "tall"
(torso + head) is now the mass sitting far out along the ground when
supine -- exactly the length-vs-height problem docs sec 12.2 measured
`(legs 0.335 m vs torso 0.55 m)`, seen again here as a lever-arm number
instead of a length comparison. The slab does not have this problem
because its own dimensions put its CoM close to the hip line in EVERY
orientation, not just standing.

**Render**: `sim/renders/getup_options/side/pincer_stock_best.mp4` (side
view, best-found path, filmstrip `docs/design-v6/pincer_stock_best_strip.
png`) -- the torso visibly rights itself (matching Tom's "pincer" framing)
and then squats at low height rather than rising, exactly matching the
trace above.

## R3.8 Kneel -> stand: foot brace and knee pincer

Tom, on `pincer_stock_best.mp4`: *"we have the robot up on its knees ...
can we use a foot to stabilize while getting up the rest of the way? or use
the same pincer movement at the knees?"* Docs sec 11 step 4 (kneel-sit ->
half-kneel) failed 0/16, but with `knee="fwd"` and 45 deg abduction --
retested here with the pincer's own sim-only ranges (`knee="both"`,
`hip_roll_abd=120`, `yaw_range=180`), `self_collide=True`, deploy model.
**A multiprocessing fix landed mid-task and is reported first** (the
coordinator caught this run at 100% of one core on a 32-core box).

**Parallelized `getup_v6_side.py` (coordinator, 2026-09-17)**: `kneelrise`
(the hand-built sweeps) and `kneelsearch` (the hill-climb) now run through
`multiprocessing.get_context("fork").Pool(12)` -- `MAX_WORKERS = 12`,
shared with the collective's other jobs on the same box. Workers build
their own plant/env from a picklable `(label, p, xml, seq, start)` tuple
(`p` a `dataclasses` instance, `xml` a file path -- no MuJoCo object ever
crosses the pool's pipe) and return plain dicts; results print in TASK
ORDER after `pool.map` returns, not as they complete, so the log stays
diff-able. **Verified before trusting it**: re-ran 4 already-logged
`kneelrise` rows (`via-pincer brace L yaw-90 knee-70 ...`, all 4
`troll`/`t_shift` combinations) through the new pooled path and diffed
against the original serial log -- byte-identical (`up`, `z`, `|tau|max`,
the shin-lift contacts, all four rows). The hill-climb's restarts cannot be
diffed the same way: a shared sequential RNG stream cannot be split across
processes and reproduced, so each restart now gets its OWN independent
seed (`random.Random(restart)`, restart 0 matches what a fresh single-
restart run gets, restarts 1-5 do not match the old sequential-stream
numbers bit-for-bit) -- stated here, not hidden. Only `kneelrise`/
`kneelsearch` were parallelized this round; `search`/`bird3`/`roll`/`tune`/
`falls`/`bird3search`/`pincer` are unchanged (still single-core) -- time
did not allow retrofitting the whole file, noted as scope, not an oversight.

**Two ways to reach the kneel**: (A) `seq_via_pincer` -- actually run the
pincer path k0..k5 to arrive there (up +1.00, pelvis 0.211, per R3.7); (B)
`seq_direct_kneel` -- settle the SAME joint targets (`KNEEL_POSE`, k5's
angles) directly from a neutral drop. **These are NOT the same physical
state**: direct settle reaches only `up +0.08, pelvis 0.070` (front +0.91,
contacts include a grounded shin+thigh+torso) -- the pincer's specific
DYNAMIC path matters, not just its final joint targets, for landing in the
same stable, upright kneel. Both are used as starts below; the difference
is itself a measured result.

**(1) FOOT BRACE (half-kneel)**, `brace_seq`: unwind ONE foot's yaw from
kneel's -131 deg toward `foot_yaw`, extend its knee to `foot_knee`, flatten
its ankle, shift the CoM via the trailing leg's roll, then rise. Swept
`foot_yaw` in (-90,-60,-30,0), `foot_knee` in (-70,-50,-30), trailing roll
shift (10,25 deg), timing (1.2,2.0 s), both sides, both kneel starts -- 192
runs (`getup_search_kneel_rise.txt`). **0/192 stood, and the family barely
moves the body at all**: best of the whole sweep reaches only **pelvis
0.073 m** (`via-pincer brace R yaw-90 knee-70 troll+10 t2.0`, up -0.20,
`|tau|max` 1.54 N-m) -- essentially the kneel's own settled height (0.05-
0.07 m), not a rise.

**(2) KNEE PINCER / sumo squat**, `knee_pincer_seq`: yaw+abduct BOTH legs
into a wide W/frog base, flex the knees so the soles land flat, then close
(adduct + extend) with the torso meant to stay vertical. Swept abduction
(60,90,120 deg), yaw (-131.45 to 0 deg), close/rise timing (1.5,2.5 s),
both kneel starts -- 96 runs. **Measured pelvis height at which the soles
touch, sweeping abduction from the kneel** (quasi-static probe, no servo
dynamics, direct `mj_step`, held 2 s): at abd 90-120 deg the pelvis settles
to **0.052-0.061 m with the torso staying upright** (up +0.89 to +1.00) --
confirming the task's own arithmetic (0.211 m kneel height needs the feet
under a 0.220 m leg, which cannot reach without the pelvis dropping) and
showing the WIDE splayed rest itself is stable and upright, unlike a narrow
one. But the dynamic CLOSE-and-rise from there fails: **0/96 stood**, best
of the sweep again barely above the kneel (**pelvis 0.072 m**,
`via-pincer sumo abd+60 yaw-131 t1.5/1.5`). **Peak hip torque during the
splay is 4.53 N-m on `R_hip_roll`** (`direct-kneel sumo abd+120 yaw-90`,
verbose trace) -- more than double the STS3250's rated range, the same
kind of jam the pincer's own k3 hit in R3.7 -- the wide-splay commands ask
for more roll authority than the servo model can deliver at this speed.

**The geometric failure point, measured** (docs sec 11 step 4's own
question: what happens right when the trailing shin leaves the floor):
replaying the pincer path to k4 (the step right after the last shin lifts,
`contacts ['L_foot','L_thigh','R_foot','R_thigh']`, pelvis 0.051 m) and
computing the whole-robot CoM against EACH sole's outline
(`v6_kin.sole_margin`): **L sole margin -136.0 mm, R sole margin -147.7 mm**
-- the CoM sits more than 130 mm outside BOTH feet's support polygons, at
a moment where the largest sole dimension is 130 mm. This is not a small
correction: no single-foot brace or symmetric leg motion within this
body's leg reach (thigh+shank 0.220 m) can shift the CoM back over a sole
from 136-148 mm outside it while the trailing leg is airborne, which is
exactly why every hand-built recovery attempt from this state stalls near
the kneel height rather than progressing toward it.

**(3) Hill-climb** (`kneelsearch`, torso-up bonus: `+2.0` per keyframe with
`up > 0.9`, 6 restarts x 2 kneel starts, now pooled): **0/12 stood**, but
clearly better than the hand-built families -- best **pelvis 0.209-0.211 m**
(`direct-kneel restart 0`: score 6.07, up +0.97, z 0.180; `via-pincer
restart 0`: score 5.39, up +0.97, z 0.194; overall sweep max across both
0.209-0.211 m, still short of the 0.8x0.387=0.310 m standing threshold).
The search keeps the torso up (as scored) but cannot translate that into
height, the same "upright, not tall" result R3.7 found for the pincer
itself.

**Robustness**: not run -- nothing in any family (hand-built or
hill-climbed) reached standing to test.

**Renders**: `sim/renders/getup_options/side/kneel_brace_best.mp4` (best
of the foot-brace family, filmstrip `docs/design-v6/kneel_brace_best_
strip.png`) and `kneel_pincer_best.mp4` (best of the knee-pincer family,
filmstrip `docs/design-v6/kneel_pincer_best_strip.png`) -- both show the
robot barely rising off the settled kneel before the sequence's own final
"stand" keyframe collapses it back down, matching the pelvis-height
numbers above.

## Files (round 3)

- `sim/gen_plant_v6.py` -- `bird_body`/`bird_L`/`bird_W`/`bird_H` (default False/0.20/0.14/0.055, plant unchanged); `_bird_body_torso()`
- `sim/getup_v6.py` -- `settle_fallen` gained `bird_back` (180 deg roll, "on its back") and `bird_flat` (right-way-up, dropped low) start states
- `sim/getup_v6_side.py` -- `bird3_box`/`bird3_round`/`bird3_box_abd120`/`bird3_round_abd120` configs; `seq_frog_roll`/`seq_stand_inverted`/`seq_bird_situp`/`seq_bird_pushup` (modes `bird3`, `bird3search`); `classify_fall`/`fall_census`/`fall_census_midstride` (modes `falls`, `fallsmid`); `gate` mode's hip_sep-scaling workaround for wide-stance bodies
- `sim/v6_kin.py`/`sim/design_gates.py`: unchanged this round (round 2's hip_z fix reused; the walk-gate workaround lives in getup_v6_side.py, not the shared kinematics)
- `docs/design-v6/getup_search_bird3_{supine,prone,side}.txt` -- hand-built sequence grids (0 standing)
- `docs/design-v6/getup_search_bird3_hillclimb_{round,box}_{folded,straight}.txt` -- continuous search (STANDING found, round_folded/straight)
- `docs/design-v6/getup_search_bird3_falls.txt`, `_falls_midstride.txt` -- fall census
- `docs/design-v6/gateD_bird3.txt` -- walk gate (negative on all 4 cases)
- `docs/design-v6/bird3_supine_strip.png` -- filmstrip of the WINNING supine get-up (rendered from the exact logged path: roll from on-its-back through up -1.00 -> -0.24 -> +0.42 -> +0.98 -> +1.00, pelvis rising 0.028 -> 0.386 m over the last 3 keyframes, peak torque 1.43 N-m on `L_hip_roll` -- confirms `STANDING=True` end to end under the full deploy servo model, not just the search's own scoring)
- `docs/design-v6/bird3_prone_strip.png`, `bird3_fall_bird3_round_strip.png`, `bird3_fall_stock_strip.png` -- filmstrips (best prone push-up attempt, a representative fall for each body)
- `sim/renders/getup_options/side/bird3_*.png`, `bird3_*.mp4` -- stills (sent to the coordinator) and videos (local only, gitignored)
- `sim/getup_v6_bird3_verify.py`, `docs/design-v6/getup_search_bird3_verify.txt`, `docs/design-v6/bird3_prone_stand_strip.png` -- R3.6, path verification (not authored this round; merged from `v6-getup-legs-skid`)
- `sim/getup_v6_side.py` `pincer` mode -- R3.7, the pincer test on the stock torso
- `docs/design-v6/getup_search_pincer_stock.txt` -- R3.7 log (verbatim bird path + 12-restart hill-climb + best trace)
- `docs/design-v6/pincer_stock_best_strip.png` -- R3.7 filmstrip; `sim/renders/getup_options/side/pincer_stock_best.mp4` (gitignored)
- `sim/getup_v6_side.py` -- R3.8: `MAX_WORKERS`/`kneel_run_all`/`_kneel_eval`/`_kneelsearch_restart` (multiprocessing), `brace_seq`/`knee_pincer_seq`/`seq_via_pincer`/`seq_direct_kneel`/`KNEEL_POSE`/`PINCER_STOCK_PATH` (modes `kneelrise`, `kneelsearch`)
- `docs/design-v6/getup_search_kneel_rise.txt` -- R3.8 log (288 hand-built + 12 hill-climb restarts, 0 standing)
- `docs/design-v6/kneel_brace_best_strip.png`, `kneel_pincer_best_strip.png` -- R3.8 filmstrips; `sim/renders/getup_options/side/kneel_{brace,pincer}_best.mp4` (gitignored)

## Verdict (round 3)

Tom was right about the body, and mostly right about the mechanism: a flat
slab centred on the hip line, with legs folded (not held straight), settles
LOWER when fallen than the tall stock body (0.051/0.041 m supine/prone vs
0.055/0.063 m), needs 16% more impulse to tip over in the first place, and
recovers to standing more often after a perturbation (33% vs 22%, and it
never lands flipped flat on its back at all, 0/36 vs 11/36) -- both of his
claims measured, not assumed. He was also right that hand-built sequences
were not going to find the get-up: every hand-authored frog/inverted/push-
up sequence in this round failed (0/~1000 combined), but a continuous
keyframe search -- the same hill-climb method round 1 used -- found a path
from flat-on-its-back to full standing height (up +1.00, pelvis 0.385-0.387
of 0.387 m possible) using the double-jointed knee's extra range, which is
the demonstration Tom asked for. The two costs this round adds are real
and unresolved: the wide hip_sep this body needs for leg-to-slab clearance
gives the existing walk-gait planner negative CoM margin on every case even
after adducting the gait to a narrower footprint, and the prone/side
get-up specifically fails at the wide-stance-to-narrow-stance transition,
a distinct, unfixed failure mode from the supine case's success.
