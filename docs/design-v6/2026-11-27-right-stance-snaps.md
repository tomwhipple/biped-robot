# The right-stance snaps: the same stall, a smaller response, and the left trim in the next double support (#259, #297)

*A dated record of an offline analysis (2026-11-27, night run, CPU only, no hardware touched; the robot is down since
11-16). It follows [the Round 7 record](2026-11-25-weight-shift-snap-sim.md), which left open "the right side's snaps
(robot 29 %, plants 8-9 %)" and "the sag plants' false left tips". The questions here: is the right-stance snap
conditioned on the ~115 ms control-loop stall as the left one is; does the stall land on the right shift's fast part
less often; does the robot snap on the right without it; does any plant close the gap with the record's own stall
timing; what the #297 robot sequence should show on the right side.*

## Result

**The right side's snaps are the same stall. The stall lands on the right shift's fast part as often as on the left,
and the right snaps need it. What differs is the size of the response: the robot's right-stance stall response is
about half its left one, and it sits right at the 2.5 deg snap line, so the snap share is a fragile metric there. The
plants miss it for a reason that is not the stall: the stiff `grade` plant answers the stall too strongly on the
right (+1.35 deg against +0.83), but starts 1.1 deg low, because the previous left stance's large trim correction
fades through the right double support and the stiff plant rolls the wrong way under it. No existing plant closes
the gap.**

1. **The stall lands on the right shift's fast part as often as on the left (verified, robot).** 11-16 14:15-20:10,
   normal timing, early shift start (t10 < 0.5 s), stance k ≥ 2, the record's own tick gaps (`stall_split.py`):

   | | left | right |
   |---|---|---|
   | stances | 380 | 337 |
   | a tick gap > 80 ms in the double support | 380 / 380 | 337 / 337 |
   | it ends after the touchdown (p10 / median / p90, s) | 0.22 / 0.35 / 0.48 | 0.18 / 0.36 / 0.47 |
   | its length (median, s) | 0.13 | 0.13 |
   | its end minus the shift's midpoint t50 (p10 / median / p90, s) | −0.49 / −0.36 / −0.24 | −0.53 / −0.36 / −0.24 |
   | the shift's t50 after the touchdown (median, s) | 0.71 | 0.71 |
   | the stance hip roll goal's peak rate there (median, deg/s) | 40.7 | 40.4 |
   | **stall on the fast part** (t50 − 0.35 to t50 + 0.15 s) | **183 (48 %)** | **165 (49 %)** |

   The replication sessions (11-13, 11-16 before 14:15) give 157 / 336 (47 %) and 139 / 284 (49 %). There is no
   timing asymmetry between the poll and the shift.
2. **The robot's right snaps need the stall (verified, robot).**

   | right stances, normal, early start | snap with the stall on the fast part | without | Fisher p |
   |---|---|---|---|
   | 11-16 evening | 92 / 165, 56 % [48-63] | 6 / 172, 3.5 % [1.6-7.4] | 5e-29 |
   | replication | 56 / 139, 40 % [33-49] | 2 / 145, 1.4 % [0.4-4.9] | 3e-18 |

   (Wilson 95 % intervals.) By where the stall ends relative to t50 (evening):

   | stall ends | −0.80 to −0.50 s | −0.50 to −0.35 | −0.35 to −0.20 | −0.20 to −0.05 | −0.05 to +0.15 |
   |---|---|---|---|---|---|
   | left snaps | 0 / 25 | 24 / 172 | 162 / 166 | 17 / 17 | — |
   | right snaps | 1 / 40 | 5 / 132 | 83 / 156 | 7 / 7 | 2 / 2 |

   - The right snaps without the stall are few, and most follow a big left trim: of the 8 early-start right snaps
     without the stall on the fast part (both session sets), 6 came after a left stance whose hike at its touchdown was
     over 7 deg (6 of the 10 such steps snapped). Below 7 deg: 2 / 307.
   - The late-start right stances (the `--td-pause` runs and the 3.47 s recipe, no stall on the fast part) snap on
     17 / 218 (7.8 %); the left on 1 / 257. That is the "other source" Round 7 noted. The `sag` plant has it at the same
     rate on the normal-timing ones (7 / 90 against 6 / 90, r 0.90 per step; `grade` 0 / 90).
3. **The right side's stall response is half the left's (verified, robot).** The snap measure (`ds_roll_peak`, the
   peak roll toward the coming swing foot in the double support) as a number, not a threshold. Means; the increment
   is with the stall on the fast part minus without; 95 % intervals by resampling walks:

   | | left: stall / none / increment | right: stall / none / increment |
   |---|---|---|
   | robot, evening | 3.47 / 1.50 / **+1.97** [1.85, 2.09] | 2.59 / 1.76 / **+0.83** [0.64, 1.01] |
   | robot, replication | — | 2.48 / 1.56 / **+0.93** [0.81, 1.06] |

   - Without the stall the two sides are alike (1.50 against 1.76). With it the right rolls 0.9 deg less.
   - The right's stall steps sit at the line: p10 / median / p90 2.10 / 2.54 / 2.96 deg against 2.5. A 0.3 deg error
     moves the right's snap share by about 30 points (56 % to 20 % or 86 %), so on the right the share hides more
     than it shows. Compare the degrees.
   - One difference is in the goals. A right stance's double support carries the fade of the previous left stance's
     trim correction: its hike at the touchdown is 5.3-5.7 deg median (the two session sets), against 1.7-2.3 before
     a left stance (the left needs about 11 deg of stance-hip trim, the right 2-4, #297). Across the stall the swing
     hip roll's correction
     jumps +0.97 deg against the shift on a right stance, +0.26-0.44 on a left one, so that hip's goal jumps 2.2 deg
     instead of 3.0. The stance hip and both ankles jump the same 3.1-3.3 deg on both sides. The plants get these same
     goals.
   - The robot's right roll grows with that left hike, with or without the stall: +0.48 ± 0.05 deg per deg on the stall
     steps, +0.61 ± 0.06 without (walk fixed effects, R² 0.46 / 0.48; the stance count adds nothing). The left
     stance's error at its touchdown is collinear with its hike (each carries the effect, the hike a little more:
     +0.32 against +0.21 together), so this data cannot say whether it is the hike's fade or the left stance's state
     that drives it.
4. **The plants, replayed with the record's exact stall timing (verified, sim).** `snap_replay.py`, the 11-16 evening,
   the same steps; normal timing, early start, k ≥ 2:

   | plant | right: snap share (stall / none) | right: mean stall / none / increment | short of the robot on the stall steps | r per step (all / stall steps) | left: increment (robot +1.97) | left snap share (robot 53 %) |
   |---|---|---|---|---|---|---|
   | **robot** | 29 % (56 / 3.5) | 2.59 / 1.76 / +0.83 | — | — | +1.97 | 53 % |
   | `grade` | 9.5 % (19 / 0) | 1.98 / 0.63 / +1.35 | +0.61 [0.43, 0.82] | 0.39 / 0.05 | +1.96 | 48 % |
   | `sag` | 8.3 % (14 / 2.9) | 2.12 / 1.47 / +0.65 | +0.47 [0.38, 0.56] | 0.81 / 0.66 | +0.98 | 11 % |
   | `sag_fric` | 7.4 % (12 / 3.5) | 2.06 / 1.43 / +0.63 | +0.54 [0.44, 0.63] | 0.80 / 0.64 | +1.09 | 14 % |
   | `play_fit` | 0.3 % | 0.48 / 0.03 / +0.46 | +2.10 | 0.33 / 0.14 | +1.21 | 13 % |
   | inset sole (`sole_in_L=8,sole_in_R=-12`) | 80 % (88 / 72) | 3.70 / 2.83 / +0.87 | −1.10 | 0.44 / 0.24 | +1.86 | 52 %, but tips on 378 / 380 |
   | series springs (Round 3's `ser_ankle=17.75,ser_hip=11.0` + mass) | — | falls on every step at the placement | | | | |

   - **`grade`** gets the left exactly (increment +1.96 against +1.97, 0.10 deg short on the stall steps). On the
     right its stall response is too large (+1.35 against +0.83), but its baseline is 1.1 deg low (0.63 against
     1.76). Its right roll *falls* with the previous left hike (−0.35 ± 0.05 deg per deg on the stall steps; the robot
     +0.48).
   - **`sag` / `sag_fric`** have the hike's sign right (+0.27 on the stall steps, +0.45 without) and track the right
     side step by step best (r 0.81). Their stall response is short on both sides (+0.65 right, +0.98 left), so they
     sit about 0.5 deg under the robot's right stall steps, and that is enough for 14 % against 56 %.
   - **The inset sole** has the right's stall response (+0.87) but a baseline 1.1 deg high, and tips on nearly every
     left stance in this open-loop replay. The series-spring plant of Round 3 falls at the placement on every step,
     so it says nothing here.
   - With the stall filled (`fill`) every plant but the inset sole has 0-3 % on the right, as on the left; the inset
     sole keeps 58 %, with or without the stall.
   - **The replication sessions** say the same with a smaller gap: robot 20 % (increment +0.93), `grade` 16 % (+1.35,
     0.34 deg short on the stall steps), `sag` 8.8 % (+0.75, 0.28 short).
5. **The left trim's fade is what `grade` gets wrong on the right (verified in the sim, inferred for the robot).**
   Counterfactual variants (`snap_replay.py --delay cF`): the trim's correction (goal − nominal) scaled by F
   everywhere, the stall kept:

   | right stances | snap share (stall / none) | mean stall / none | short on the stall steps |
   |---|---|---|---|
   | `grade`, as recorded | 9.5 % (19 / 0) | 1.98 / 0.63 | +0.61 |
   | `grade`, correction × 0.5 | 25 % (51 / 0.6) | 2.61 / 1.35 | −0.01 [−0.13, 0.12] |
   | `grade`, correction × 0 | 23 % (46 / 0.6) | 2.56 / 1.13 | +0.04 |
   | `sag`, correction × 0.5 | 2.4 % | 1.99 / 1.14 | +0.61 |
   | `sag`, correction × 0 | 1.8 % | 1.66 / 0.69 | +0.93 |

   - On the evening set, halving the corrections brings `grade` onto the robot's right stall steps exactly (and 25 %
     against the robot's 29 %). On the replication set it overshoots a little (29 % against 20 %, 0.18 deg over).
   - The stall still makes the snap in every variant: with the correction × 0 and the stall filled, 0 %.
   - Per step it is still weak: r 0.47 (0.34 on the stall steps), against 0.93 for the left.
   - So the stiff plant takes the robot's large left trim at face value; the robot does not. That is gate item 1 (on
     the left stance the robot stands level under ~11 deg of hip trim, the plants roll 2.7-4.6 deg under it), seen
     again in the next double support. The robot's positive slope (point 3) agrees with a left side that gives under
     the trim: `sag`, which has that give, has the right sign. *Inferred:* the term the sim lacks for the right-side
     snaps is the left side's give under the trim, not anything about the stall. The plant with that give (`sag`)
     loses half of the stall response on both sides, so neither plant has both.

## The #297 robot sequence: the right side's predictions

The sequence is as in `experiments/roll-balance/README.md` ("The control-loop stall"): forward 8 at 40 mm, T 4, no
`--td-pause`, C P C P C P on the old firmware, then PR #355 flashed. An 8-step walk has about 3 right stances with
k ≥ 2, so each arm has about 9, and about 4-5 of C's have the stall on the fast part. Read the right side as degrees per
step (`tip_causes.py table`, then `stall_split.py robot` on the CSV), not only as a share.

| | right snap share (k ≥ 2) | right `ds_roll_peak`, mean | where it comes from |
|---|---|---|---|
| **1 C** (old firmware, control) | **~25 %** (evening 29 % [24-34], replication 20 % [16-26]); ~45-55 % on the stances with the stall on the fast part, ~2 % on the rest | 2.6 deg on the stall steps, 1.7 on the others | the two session sets |
| **1 P** (`--polls-in-ss`) | **~3 %** (2.5 % [1.3-4.9], 8 / 317 stall-free early-start right stances); `grade` and `sag` with the stall moved into the single support: 0 % and 1.2 % | ~1.7 deg on every stance (stall-free robot mean 1.56-1.76) | the stall leaves the double support |
| **2** (PR #355, defaults) | **~3 %**, as 1 P | ~1.7 deg | no stall anywhere |

- **Expected snaps in 9 right stances:** about 2 in C (none at all with probability 0.08), about 0.3 in P and in step 2.
  The count alone does not separate the arms on the right; the degrees do. C's stall steps against P's stances differ by
  0.8 deg with a standard error of about 0.2 at these numbers (spreads 0.34 and 0.43 deg).
- **What remains in P and step 2:** right snaps after a left stance that ended with a hike over about 7 deg (6 of 10 such
  steps snapped without the stall). The right roll's slope on the previous left hike, +0.5-0.6 deg per deg, should stay
  without the stall (it is there on the stall-free steps today).
- **What falsifies the stall as the right side's cause:**
  - in C, the right stances with the stall on the fast part do not roll more than the others (the prediction is +0.8,
    at least +0.4 at two standard errors);
  - in P or step 2, 2 or more right snaps in about 9 stances whose previous left hike was under 7 deg (probability
    about 3 % at the predicted 3 %), or a right mean of 2.2 deg or more.
- **The stall in the raise (P only, sim).** With the stall moved to 60 ms after each lift-off (`--delay ss`), the
  right stances' lean is unchanged in both plants and there are no right tips. On the left, `grade` gains 2 tips
  (0 with the stall filled) and `sag` 88 against 54. The left raise is the place to watch in P, as the README says.
- **What this data cannot tell, and the run that would:** whether the robot's right roll follows the left trim's
  fade itself or the left stance's state at its touchdown (they are collinear here). A frozen-trim walk (the trim's
  learning off, T L held at two values, e.g. 3 and 6 deg) on the fixed firmware, or with `--polls-in-ss`, would
  separate them: if the fade drives it, the right roll moves about 0.5 deg per deg of the held T L; `grade` predicts
  the opposite sign. This is the frozen-trim arm already on the #297 list, read on the right side as well.

## The sag plants' false left tips (question 3, partial)

On the same 380 normal early-start left stances (k ≥ 2; robot 12 tips):

| plant | tips | on the robot's tips | false | robot's lean (TS `lean_in`) against the written T L, walk fixed effects | the plant's lean against it |
|---|---|---|---|---|---|
| robot | 12 | — | — | +0.02 ± 0.13 deg / deg | — |
| `grade` | 0 | 0 | 0 | | −0.14 ± 0.03 |
| `sag_L=3.2` | 12 | 2 | 10 | | +0.35 ± 0.14 |
| `sag_L=4.8` | 39 | 6 | 33 | | +1.17 ± 0.21 |
| `sag_L=6.4` (`sag`) | 108 | 10 | 98 | | +2.13 ± 0.24 |
| `sag_fric` | 116 | 12 | 104 | | +2.12 ± 0.27 |

- The false tips are the late, high-trim stances: the robot's T L at the lift-off separates them from the clean
  stances with AUC 0.86 (6.30 against 5.55 deg), the stance count with 0.83. The stall barely does (AUC 0.60; with the
  stall filled, 0.51 and still 48 false tips).
- On the robot the lean does not follow T: the closed-loop trim learns the T the robot needs. Replayed open loop, the
  sag plants' lean grows 1-2 deg per deg of that T. So the sag plants do not need the trim the robot learned, and the
  extra trim tips them.
- Lowering `sag_L` cuts the false tips and the hits together (10 / 98, 6 / 33, 2 / 10): no value separates them.
- So both open items come back to gate item 1: how the left side takes the large hip trim. The stiff plant takes it
  at face value and rolls the wrong way in the next double support; the sag plant gives, but too much as T grows.
  *Inferred, not tested further:* a left-side term that makes the plant need the trim the robot needs (its lean flat
  in T, like the robot's) is what both items ask for. Nothing here was tried as a plant default.

## What this changes

- **For the gate (#259):** no plant term and no default changed. The right side's 29 % against 8-9 % is not a stall
  gap: the stall's timing and its necessity are the same on both sides, and `grade`'s stall response is if anything
  too large on the right. The gap is the left trim's effect on the next double support, which is gate item 1. Compare
  the right side in degrees (`ds_roll_peak` means by stall position), not by the 2.5 deg share.
- **For #297:** the right-stance predictions above join the left's (C ~55 % left / ~25 % right; P and step 2 ~0-3 %
  on both). The right side alone is a weak test at 9 stances per arm; read its degrees.
- **For grading and training:** nothing changes.

## Data and method

- **Robot.** `tip_causes.py extract` over the 11-13 and 11-16 flight records (207 runs), `tip_causes.py table` for
  11-16 14:15-20:10 (the evening set, 1463 single supports) and for 11-13 plus 11-16 before 14:15 (the replication
  set).
- **The stall.** `experiments/roll-balance/stall_split.py` (new; tests `tests/test_stall_split.py`). The stall of a
  single support is the longest tick gap over 80 ms that ends in the double support before it; "on the fast part" is
  its end within t50 − 0.35 to t50 + 0.15 s. It reproduces Round 7's left counts exactly (179 / 183 against 24 / 197;
  replication 151 / 157 against 15 / 179) and its right ones (92 / 165 against 6 / 172).
  - `stall_split.py robot CSV...`: the split per side, timing and shift start, Wilson intervals, Fisher p, the stall's
    timing, the shift's timing and rate, the snap by stall position.
  - `stall_split.py sim CSV... --side R`: the same split for each plant and variant of a `snap_replay.py` CSV, with
    the per-step correlation against the robot overall, on the stall steps and on the others.
- **Sim.** `snap_replay.py` as in Round 7 (placed before the previous single support, the robot's goals held over the
  record's ticks). New opt-in variants (defaults unchanged; tests in `tests/test_snap_replay.py`):
  - `ss`: every stall filled, then a 115 ms stall 60 ms after each lift-off (the `--polls-in-ss` arm on the old
    firmware).
  - `cF` / `cFfill`: the trim's correction (goal − nominal) scaled by F everywhere, with the stalls as recorded or
    filled.
  - A step whose sim fell before its lift-off is now skipped with a message instead of ending the run.
- The increments' intervals resample walks (2000 draws). The slopes are least squares with walk fixed effects
  (`tip_causes.ols`). The goal jumps across the stall are read from the cache at the gap's two ticks.
- **Verified against inferred.**
  - Verified: the stall's timing per side; the right snap's dependence on it (two session sets); the right
    response's size in degrees; the plants' numbers, the counterfactuals, the slopes.
  - Inferred: that the missing term is the left side's give under the trim (from the slopes' signs and the
    counterfactual, not from a plant that has it); the P and step 2 rates (from stall-free robot stances and the
    sims, not from a run with the stall moved); the reading of the sag plants' false tips.

```bash
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=1 MUJOCO_GL=disabled
cd experiments/roll-balance
python tip_causes.py extract --day 20261116 --day 20261113
python tip_causes.py table --day 20261116 --from 1415 --to 2010
python tip_causes.py table --day 20261113
python tip_causes.py table --day 20261116 --from 0000 --to 1414
python stall_split.py robot results/tip_causes/20261116T142658_20261116T200654.csv
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side R --delay 0,fill --workers 6 --name r8_right
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side L --delay 0,fill --workers 4 --name r8_left
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side R --delay c0,c0fill,c0.5,ss \
    --plants "grade=grade;sag=grade+sag_L=6.4,sag_R=2.9" --workers 3 --name r8_right_cf
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side L --delay ss \
    --plants "grade=grade;sag=grade+sag_L=6.4,sag_R=2.9" --workers 2 --name r8_left_ss
python snap_replay.py run --day 20261113 --day 20261116 --from 0000 --to 1414 --side R --delay 0,fill,c0.5 \
    --plants "grade=grade;sag=grade+sag_L=6.4,sag_R=2.9" --workers 3 --name r8_right_rep
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side L --delay 0 \
    --plants "sagL32=grade+sag_L=3.2,sag_R=2.9;sagL48=grade+sag_L=4.8,sag_R=2.9" --workers 5 --name r8_left_sagL
P2="sole=grade+sole_in_L=8,sole_in_R=-12;series=grade+torso_kg=0.4855,pack=0.083,zoff_La=0,ser_ankle=17.75,ser_hip=11.0"
python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side R --delay 0,fill --plants "$P2" --name r8_right_p2
python stall_split.py sim results/snap_replay/r8_right.csv --side R
```

- A variant-only CSV (`r8_right_cf`) has no robot rows; join it to a CSV that has them (same header) before
  `stall_split.py sim`.
- Each 2-4 plant pass over the 134 evening runs took 5-15 min on 3-6 processes; worker memory stayed under 1.5 GB.
- The per-group means, intervals, slopes and the false-tip table came from short scripts over these CSVs; they were not
  kept as a tool. The numbers above are the record.
