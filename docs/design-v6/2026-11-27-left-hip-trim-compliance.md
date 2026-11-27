# The left hip under the trim: one plant term for both signatures? (#259, #297)

*A dated record of an offline analysis (2026-11-27, night run, CPU only, no hardware touched; the robot is down since
11-16). It follows [the right-stance snaps record](2026-11-27-right-stance-snaps.md) of the same night. That record
traced two open items to one inferred cause, "the left side gives under a large hip trim": the robot's right-stance
roll grows with the previous left stance's hike (+0.48 deg per deg) where the stiff `grade` plant's shrinks (−0.35),
and the `sag` plants tip falsely on late, high-trim left stances (their lean grows ~2 deg per deg of T, the robot's
does not). The question here: is there one physically plausible plant term that gives both, and keeps what `grade`
already gets right on the left (stall response +1.97 deg, ~48-53 % snaps, r ≈ 0.93 per step)?*

## Result

**No. No single term gives both signatures, and the data cannot say which term the robot has. Lowering the left hip's
trim authority helps. That can be a torque-proportional compliance (k ≈ 11 N m/rad) or the correction reaching the
joint at 0.25-0.5 of its size. Either brings the plant's right-stance stall steps to within 0.2-0.3 deg of the robot
and keeps the left side whole. Neither gives the positive slope. Splitting the hike shows why. The replicated part of
the robot's slope rides on the trim's own answer to the lean at the left touchdown, which is a closed-loop coupling.
Open loop, a plant gets it only by leaning more under more trim. That is exactly what tips the `sag` plants falsely,
so the two signatures ask opposite things of an open-loop plant. The run that settles it is the frozen-trim walk
already on the #297 list.**

1. **The candidates on the 11-16 evening** (`snap_replay.py`, normal timing, early shift start, k ≥ 2, the record's
   own stall timing; `trim_compliance.py`). Right stances: n 337. Left stances: n 380. "Short" is robot minus plant
   on the stall steps. "Slope" is the stall steps' `ds_roll_peak` on the previous left hike at its touchdown (walk fixed
   effects). "False" counts plant tips on the robot's clean left stances.

   | plant | R snap share (stall / none) | R mean stall / none | R short | R slope on the hike | L increment | L snaps | L r | L tips (false) | L lean, deg | L lean on T |
   |---|---|---|---|---|---|---|---|---|---|---|
   | **robot** | **29 %** (56 / 3.5) | 2.59 / 1.76 | — | **+0.48 ± 0.05** | **+1.97** | **53 %** | — | 12 (—) | 3.69 | +0.02 ± 0.13 |
   | `grade` | 9.5 % (19 / 0) | 1.98 / 0.63 | +0.61 | −0.35 ± 0.05 | +1.96 | 48 % | 0.93 | 0 (0) | 1.33 | −0.14 |
   | *the left hip's correction × F (goal side)* | | | | | | | | | | |
   | F 0.5 (`c0.5@L_hip_roll`) | 25 % (51 / 0.6) | 2.61 / 1.35 | −0.01 | −0.06 ± 0.05 | +1.96 | 48 % | 0.93 | 0 (0) | 1.96 | −0.11 |
   | F 0.25 | 34 % (68 / 0.6) | 2.71 / 1.35 | −0.12 | −0.02 ± 0.05 | +1.96 | 48 % | 0.93 | 0 (0) | 2.29 | −0.03 |
   | F 0 | 23 % (46 / 0.6) | 2.56 / 1.13 | +0.04 | +0.01 ± 0.05 | +1.96 | 48 % | 0.93 | 0 (0) | 3.04 | +0.07 |
   | both hips × 0.5 (`c0.5`) | 25 % | 2.61 / 1.35 | −0.01 | −0.05 ± 0.05 | +2.08 | 48 % | 0.93 | 0 (0) | 1.96 | −0.11 |
   | *a spring in series with the left hip roll, k N m/rad (`x_ser_L_hip`)* | | | | | | | | | | |
   | k 44 | 16 % | 2.32 / 1.06 | +0.27 | −0.29 ± 0.04 | +2.16 | 52 % | 0.93 | 0 (0) | 1.90 | −0.19 |
   | k 22 | 15 % | 2.36 / 1.24 | +0.24 | −0.11 ± 0.04 | +2.17 | 52 % | 0.93 | 0 (0) | 2.29 | −0.09 |
   | **k 11** | 11 % (22 / 0) | 2.27 / 1.25 | +0.32 | +0.02 ± 0.03 | +2.13 | 48 % | 0.93 | 0 (0) | 3.01 | −0.25 |
   | k 7 | 4.2 % | 2.10 / 0.98 | +0.49 | +0.06 ± 0.03 | +2.09 | 41 % | 0.92 | 1 (1) | 4.25 | −0.26 |
   | k 5 | 2.1 % | 1.83 / 0.70 | +0.77 | +0.10 ± 0.03 | +1.90 | 26 % | 0.90 | 1 (1) | 5.18 | −0.24 |
   | k 11 and F 0.5 | 6.8 % | 2.17 / 0.93 | +0.43 | +0.05 ± 0.03 | +2.13 | 48 % | 0.93 | 0 (0) | 4.40 | −0.14 |
   | *both hips, or elsewhere* | | | | | | | | | | |
   | both hip rolls, k 22 | 0.3 % | 1.44 / 0.91 | +1.16 | −0.15 ± 0.02 | +0.79 | 1.3 % | 0.89 | 0 (0) | 2.33 | −0.15 |
   | both hip rolls, k 11 | 0 % | 1.19 / 0.85 | +1.41 | −0.03 ± 0.02 | +0.68 | 0 % | 0.80 | 0 (0) | 3.44 | −0.07 |
   | left ankle roll, k 22 | 30 % (58 / 2.3) | 2.63 / 1.27 | −0.04 | −0.34 ± 0.04 | +1.73 | 46 % | 0.92 | 0 (0) | 0.58 | −0.16 |
   | hip-roll dead band 2 deg (`x_db50=2`, both) | 1.2 % | 0.51 / 0.02 | +2.09 | −0.12 ± 0.05 | +2.07 | 39 % | 0.87 | 0 (0) | 1.33 | −0.16 |
   | *the foot-off sag (existing option)* | | | | | | | | | | |
   | `sag` (L 6.4, R 2.9) | 8.3 % | 2.12 / 1.47 | +0.47 | +0.27 ± 0.04 | +0.98 | 11 % | 0.82 | 108 (98) | 6.06 | +2.13 |
   | sag, hips only (`sag_ankle=0`) | 3.0 % | 1.61 / 1.08 | +0.99 | **+0.32 ± 0.05** | +1.00 | 8.2 % | 0.84 | 256 (245) | 9.07 | +0.73 |
   | sag, hips only, L 3.2 | 9.8 % | 2.26 / 1.38 | +0.33 | −0.18 ± 0.03 | +1.11 | 10.5 % | 0.88 | 3 (3) | 2.43 | −0.13 |
   | sag L 6.4 only (R 0) | 11 % | 2.29 / 1.48 | +0.31 | +0.13 ± 0.04 | +1.32 | 37 % | 0.87 | 42 (36) | 4.65 | +1.23 |
   | sag L 3.2 only (R 0) | 20 % | 2.42 / 1.42 | +0.17 | −0.22 ± 0.04 | +1.39 | 39 % | 0.89 | 0 (0) | 2.47 | −0.20 |

   - **Every term on the right hip, or on both hips, kills the left stall response** (1.3 % and 0 % left snaps). The
     right hip is the swing-side hip in the left double support, and the stall's goal jump there has to reach the
     body. Left only, or nothing.
   - **Lowering the left hip's trim authority keeps the left whole and fixes the right level.** Both forms do it: a
     left hip spring of k 11-22, or the left correction scaled by F 0.25-0.5. The left increment stays +1.96 to +2.17,
     with 48-52 % snaps, r 0.93, no false tips and a flat lean on T. On the right, the stall steps come within
     −0.12 to +0.32 deg of the robot. The goal-side scaling also gets the right snap share (25-34 %). The spring
     gets 11-15 %: it sits 0.24-0.32 deg under the robot, right at the 2.5 deg line. Both also move the left-stance
     lean toward the robot's: 1.33 deg becomes 1.96-3.04 deg (k 11: 3.01, F 0: 3.04; robot 3.69). That is
     gate item 1 (the plant rolls toward the stance foot under the trim) closing from the same side.
   - **None of them gets the slope.** Lowering the authority takes grade's −0.35 to between −0.11 and +0.10, and it
     saturates there. The left correction at F 0 (no trim on the left hip at all) gives +0.01. A softer spring
     (k 5-7) gets to +0.06 to +0.10 only by losing the right snaps (2-4 %) and the left (26-41 %).
   - **Only the sag's give gets the slope** (+0.27, and +0.32 with the sag on the hips only). But it halves the left
     stall response (+0.98 to +1.11) and gives the false tips (98-245), with the lean growing with T (+0.73 to
     +2.13). Scaled down until the false tips go (hips only at L 3.2, or the left-only sag at L 3.2), its slope turns
     negative (−0.18 to −0.22).
   - The left ankle-roll spring matches the right snap share (30 %) for the wrong reason. Its stall response is too
     large (+1.36 against +0.83), its baseline too low, its slope is grade's (−0.34), and it moves the left lean away
     from the robot's (0.58).
2. **Why: the replicated part of the robot's slope is the trim's answer to the lean (verified on the robot; the
   reading inferred).** The hike at the left touchdown is the learned T L plus the trim's answer to the lean at that
   moment, `u = hike − T L` (kp·s·e plus the integral). On the robot, u follows the left touchdown's error toward the
   right foot at +0.37 ± 0.05 deg per deg (replication +0.35 ± 0.04). That error itself rises with the hike
   (+0.73 ± 0.03, +0.75 ± 0.04): the trim is answering a lean it does not remove. Split the right stall steps'
   roll over the two parts (walk fixed effects):

   | right stall steps, roll on | the learned T L | u = hike − T L |
   |---|---|---|
   | robot, evening (n 165) | +0.37 ± 0.09 | **+0.61 ± 0.11** |
   | robot, replication (n 139) | +0.04 ± 0.08 | **+0.76 ± 0.11** |
   | `grade` (evening / replication) | −0.32 / −0.29 | −0.38 / −0.45 |
   | left hip correction × 0.5 | +0.05 / +0.01 | −0.18 / −0.09 |
   | left hip spring k 11 | +0.11 / +0.03 | −0.10 / −0.13 |
   | `sag` / sag hips only (evening) | +0.06 / +0.12 | **+0.52 / +0.55** |

   - On the robot the part that replicates is u, not the learned T. A robot that ends its left stance leaning
     toward the right foot rolls more toward the left in the next double support, and the trim's proportional answer
     shows that lean as a larger hike. The learned T's own share (+0.37) is in the evening only.
   - In an open-loop replay the hike is a recorded goal, and the plant's lean does not cause it. A plant can give u's
     positive sign only if its own lean grows with that goal. The sag does this: its left touchdown error rises
     +0.68-0.76 per deg of the hike, against −0.83 for `grade`. The same property makes its left-stance lean grow
     with T and its false tips. So the two signatures cannot both be met by an open-loop plant. The right slope
     wants the plant to lean more under more trim, and the left tips want it not to.
   - On the learned T's part, the low-authority plants (+0.01 to +0.11) sit between the robot's two session sets
     (+0.37, +0.04). `grade` (−0.29 to −0.32) is off on both.
3. **The trim's authority on each plant, paired (sim).** Each step is replayed twice, with the left hip's correction
   in full and scaled down (`trim_compliance.py --pair`). Per degree of the removed hike at the left touchdown, right
   stances:

   | plant | the left touchdown's error toward the right foot | the right double support's roll |
   |---|---|---|
   | `grade`, F 1 against 0.5 (evening / replication) | −0.98 / −0.94 | −0.24 / −0.21 |
   | `grade`, F 1 against 0 | −0.49 | −0.10 |
   | left hip spring k 11, F 1 against 0.5 | +0.12 | +0.06 |

   On `grade` the top half of the trim moves the body almost 1:1 at the touchdown. The bottom half barely does
   (the effect saturates; why was not examined). With the k 11 spring the trim has no grip on the body. The robot's
   value is the number the frozen-trim walk measures (below).
4. **Held out: the replication sessions** (11-13, and 11-16 before 14:15; right n 284, left n 336):

   | plant | R snap share (stall / none) | R mean stall / none | R short | R slope | L increment | L snaps | L r | L false tips | L lean |
   |---|---|---|---|---|---|---|---|---|---|
   | **robot** | **20 %** (40 / 1.4) | 2.48 / 1.56 | — | +0.33 ± 0.05 | +1.88 | 49 % | — | — | 3.33 |
   | `grade` | 16 % (33 / 0) | 2.14 / 0.79 | +0.34 | −0.36 ± 0.05 | +2.05 | 42 % | 0.84 | 0 | 1.35 |
   | left hip correction × 0.5 | 29 % (60 / 0) | 2.67 / 1.35 | −0.18 | −0.03 ± 0.05 | +2.05 | 42 % | 0.84 | 0 | 1.96 |
   | left hip correction × 0.25 | 35 % (70 / 0.7) | 2.76 / 1.33 | −0.27 | +0.03 ± 0.05 | +2.05 | 42 % | 0.84 | 0 | 2.29 |
   | left hip spring k 11 | 12 % (25 / 0) | 2.28 / 1.25 | +0.20 | −0.03 ± 0.04 | +2.20 | 43 % | 0.83 | 0 | 2.98 |

   The two session sets want different amounts. `grade` sits 0.61 deg under the robot's evening stall steps and
   0.34 under the replication ones. So the goal-side F 0.5, which fits the evening, overshoots here by 0.18 deg
   (29 % against 20 %). The k 11 spring is under on both (by 0.32 and 0.20). Both keep the left as `grade` has it,
   on both sets. Neither gets the slope on either set.

## Verdicts

| candidate | verdict |
|---|---|
| torque-proportional compliance, **left hip roll only** (k 11-22 N m/rad; `grade`'s hip-roll servo is 44) | **Not rejected, not identified.** It keeps the left (increment +2.13 to +2.17, 48-52 %, r 0.93, 0 false tips, lean on T −0.09 to −0.25). It moves the left lean toward the robot (3.01 at k 11, robot 3.69). On the right it brings the stall steps within 0.20-0.32 deg (both sets) and the slope to ≈ 0. It does not give the snap share (11-15 % against 29 / 20 %) or the positive slope. |
| the same on **both hips** | **Rejected.** The left stall response goes (0-1.3 % left snaps, increment +0.68 to +0.79). |
| **reduced trim authority** on the left hip (the correction × 0.25-0.5; goal side, `c<F>@L_hip_roll`) | **Not rejected, not identified.** It keeps the left exactly (the stall jump is untouched) and gets the right level and share on the evening (25 %, short −0.01). It overshoots the replication (29-35 % against 20 %), and the slope stays ≈ 0. As a mechanism it is a servo that settles short of a slow, sustained correction but follows fast goal jumps. Creep in a printed part or a load-dependent position error would act like that. Nothing here measures it. |
| left ankle-roll compliance (k 22) | **Rejected.** It matches the right share for the wrong reason (the increment is too large, the baseline too low), its slope is grade's, and it moves the left lean away from the robot. |
| hip-roll dead band (2 deg, both) | **Rejected.** 1.2 % right snaps, 39 % left. |
| the foot-off sag (`sag_L` / `sag_R` / `sag_ankle`) | **Rejected as the one term.** It is the only one with the right slope (+0.27 to +0.32), but only where it halves the left stall response and tips falsely on 36-245 clean steps. Without the false tips its slope is negative. |
| the series spring of Round 3 and the play (`play_fit`) | Not re-run: the 11-27 snaps record has them (they fall at the placement, or give 0.3 % right snaps). |

**What is identifiable from this data:** the left hip's trim authority must be well under `grade`'s on the robot's
walks. That fits the left-stance lean, the right level, and the learned-T part of the slope on the replication set.
Whether the cause is a compliance (an offset under load) or a fractional authority is not identifiable. Nor is the
robot's own authority, because the hike and the lean are coupled through the trim's proportional term. **Physical
plausibility (inferred):** the left hip still carries the original glued 35 %-infill yoke. Its roll horn arm sees
about 9× the across-layer stress of the redesign ([hip yoke record §7](2026-11-02-hip-yoke-roll-failure.md)), and
it broke in the 11-16 fall (#326). A left-only give under load is what that part would do. The reprint is the
redesign at 100 % infill, so the **robot after the repair may no longer have it**.

## The run that would settle it

The frozen-trim walk already on the #297 list ([2026-11-24 record](2026-11-24-left-stance-tips.md), "First test for
the next robot session"): `trim set` with ki 0, T held at 5.2 / 3.4 (A) and 6.8 / 1.2 (B), on the same `--cmd` walk.
Between A and B, T L differs by 1.6 deg, and that difference does not depend on the robot's lean. Read the right
stances of A against B in the `tip_causes.py table` CSV's columns `prev_e_td`, `prev_hike_td`,
`T_other` and `ds_roll_peak`, with the stall split of `stall_split.py robot`:

| per deg of held T L (B − A) | `grade` | left spring k 11 | left correction × 0.5 | the learned-T reading of the robot's slope |
|---|---|---|---|---|
| the left touchdown's error toward the right foot | about −1.0 | about +0.1 | about −0.25 to −0.5 | — |
| the right stall steps' `ds_roll_peak` | about −0.2 to −0.3 | about +0.1 | about −0.1 | +0.4 (evening) / 0 (replication) |

- **Expected size at 1.6 deg.** Take A as 3 walks and B as 2, with about 12 right stances per 24-step walk. That is
  about 33 and 22 right stances, about half of them with the stall on the fast part (old firmware).
  - The left touchdown's error: `grade` moves it by about −1.6 deg, a low-authority left hip by under 0.5. The
    robot's spread is about 1.0 deg per step, so the difference of the two means has an SE of about 0.27 deg. The
    readings are 4-6 SE apart. **This is the measurement that identifies the authority.**
  - The right stall steps' roll: −0.5 (`grade`), about 0 (low authority), +0.6 (the evening's learned-T reading).
    With about 16 and 11 stall steps and a spread of 0.65 deg, the SE is about 0.25 deg. That separates `grade`
    from the learned-T reading, but not the low authority from either.
- **The repair is the confound.** If the reprinted left yoke is stiffer, the authority measured after the repair is
  not the one in these data. Run the 11-16 re-baseline march the 11-24 record asks for first, and compare the left
  stance's lean under the trim (the TS `lean_in` against T L) with 11-16's. If it drops toward `grade`'s, the give
  was the old yoke.
- A direct bench number would identify the term outright: the left and right hip-roll angle beyond the encoder
  against a known roll torque (the 10-24 clamp test, one side at a time), on the old yoke if it is still around, and on
  the new one.

## What this changes

- **For the gate (#259):** no plant term and no default changed. Item 1 (the left stance under the trim) gets a
  candidate, a low trim authority on the left hip only, with both signs right on the left and the right level within
  0.2-0.3 deg. The positive right-stance slope is shown to be mostly closed-loop. An open-loop replay cannot
  reproduce it with any plant that keeps the left side right, and it is not a target for a plant term.
- **For #297:** the frozen-trim arm now also measures the left hip's trim authority (above). Read it before and
  after the repair.
- **For grading and training:** nothing changes. The new terms are opt-in in the replay tool only.

## Data and method

- **Robot.** The `tip_causes.py` cache of 11-13 and 11-16 (207 runs), as the 11-27 snaps record. The evening set is
  11-16 14:15-20:10. The replication set is 11-13 plus 11-16 before 14:15.
- **Sim.** `snap_replay.py run` (placed before the previous single support, the robot's goals held over the
  record's ticks, the record's own stall timing). Two new opt-in additions (defaults unchanged; tests in
  `tests/test_snap_replay.py`):
  - the variant `c<F>[fill]@<joint>[+<joint>]`: the trim's correction scaled by F on the named joints only. The
    trim writes only the hip rolls; `c0.5@L_hip_roll` and `c0.5` agree on the right stances to 0.01 deg.
  - the study terms `x_ser_L_hip=`, `x_ser_R_hip=`, `x_ser_L_ank=`, `x_ser_R_ank=` (N m/rad): walker_env's live
    roll series spring set on one joint, so a torque-proportional angle loss τ/k beyond the encoder. They are refused
    with the sag (the env takes one beyond-encoder spring).
- **Scores.** `experiments/roll-balance/trim_compliance.py` (new; tests `tests/test_trim_compliance.py`) prints the
  tables above per side for every plant and variant in the CSVs. It matches the 11-27 snaps record's `grade` and
  robot numbers exactly (right 9.5 % / 29.1 %, increments +1.35 / +0.83, slopes −0.35 / +0.48; left +1.96 / +1.97,
  48.4 % / 53.4 %, r 0.93). `--pair` gives point 3.
- Slopes are least squares with walk fixed effects (`tip_causes.ols`), ± their standard errors. The paired authority is
  a slope through the origin over the steps whose removed hike exceeds 1 deg.
- Each plant pass over the 134 evening runs, both sides, took about 1.5 min on 8 processes.
- **Verified against inferred.**
  - Verified: every number in the tables; the hike's split on the robot (both session sets); the paired authorities.
  - Inferred: that the robot's u-coupling is the closed loop (from the trim law and the robot's u-on-error slope,
    not from a run that breaks the loop); the mechanism readings of the two low-authority forms; that the old left
    yoke is the source; the frozen-trim predictions (from the open-loop pairs, not from a closed-loop sim of the
    frozen arm).

```bash
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=1 MUJOCO_GL=disabled
cd experiments/roll-balance
python tip_causes.py extract --day 20261116 --day 20261113
EV="--day 20261116 --from 1415 --to 2010"; RP="--day 20261113 --day 20261116 --from 0000 --to 1414"
python snap_replay.py run $EV --side LR --delay 0 --plants "grade=grade" --workers 8 --name ev_grade
python snap_replay.py run $EV --side LR --delay c0.5@L_hip_roll,c0.25@L_hip_roll,c0@L_hip_roll,c0.5 \
    --plants "grade=grade" --workers 8 --name ev_cL
for k in 44 22 11 7 5; do
  python snap_replay.py run $EV --side LR --delay 0 --plants "serL$k=grade+x_ser_L_hip=$k" --workers 8 --name ev_serL$k
done
python snap_replay.py run $EV --side LR --delay c0.5@L_hip_roll --plants "serL11=grade+x_ser_L_hip=11" --name ev_serL11_cL05
python snap_replay.py run $EV --side LR --delay 0 --name ev_other --plants "serLR22=grade+x_ser_L_hip=22,x_ser_R_hip=22;\
serLR11=grade+x_ser_L_hip=11,x_ser_R_hip=11;serLank22=grade+x_ser_L_ank=22;db50_2=grade+x_db50=2;\
sag=grade+sag_L=6.4,sag_R=2.9;sagHip=grade+sag_L=6.4,sag_R=2.9,sag_ankle=0;sagHip32=grade+sag_L=3.2,sag_R=2.9,sag_ankle=0;\
sagL64R0=grade+sag_L=6.4,sag_R=0;sagL32R0=grade+sag_L=3.2,sag_R=0"
python snap_replay.py run $RP --side LR --delay 0 --plants "grade=grade;serL11=grade+x_ser_L_hip=11" --name rp_plants
python snap_replay.py run $RP --side LR --delay c0.5@L_hip_roll,c0.25@L_hip_roll --plants "grade=grade" --name rp_cL
python trim_compliance.py results/snap_replay/ev_*.csv --pair 'grade:0|grade:c0.5@L_hip_roll|0.5' \
    --pair 'grade:0|grade:c0@L_hip_roll|1' --pair 'serL11:0|serL11:c0.5@L_hip_roll|0.5'
python trim_compliance.py results/snap_replay/rp_*.csv --pair 'grade:0|grade:c0.5@L_hip_roll|0.5'
```

The runs were split one plant per invocation here (the CSV is written at the end of a run). The grouping above gives
the same rows.
