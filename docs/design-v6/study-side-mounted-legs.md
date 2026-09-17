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
