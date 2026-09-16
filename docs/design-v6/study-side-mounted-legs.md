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

## Files

- `sim/gen_plant_v6.py` -- `hip_z` field (default 0.0, plant unchanged)
- `sim/getup_v6.py` -- `settle_fallen` gained `side_l`/`side_r` start states
- `sim/getup_v6_side.py` -- this study (`probe`/`search`/`robust`/`gate`/`render` modes)
- `docs/design-v6/getup_search_side_probe.txt` -- quasi-static probe log
- `docs/design-v6/getup_search_side_supine.txt`, `_prone.txt`, `_side.txt` -- dynamic search logs (0/180 standing)
- `docs/design-v6/gateD_side.txt` -- walk-gate log (splay-only cleared, hip_z skipped with a stated reason)
- `docs/design-v6/getup_side_supine_strip.png`, `getup_side_prone_strip.png`, `getup_side_fail_strip.png` -- filmstrips (best supine attempt, best prone attempt, a side-start failure)
- `sim/renders/getup_side/*.mp4` -- the matching videos (local only, gitignored)

## Verdict

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
