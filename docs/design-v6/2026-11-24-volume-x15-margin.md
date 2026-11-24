# The x1.5 loop-gain margin on the volume recipe: a diagnosis (2026-11-24)

Issue #263. On 11-23 the planner found that the command-volume students (VW = `--cmd-volume 1.0`
warm-started from GD; M1 = VW + `--mimic-volume-ref`; M2 = VW + `--cmd-stand-band`) meet every part of
the bar except the x1.5 probe. On plant D, M1 seeds 14 and 16 fall 4/8 and 8/8. M1 seed 15 falls 0/8
on D but 8/8 on C. If tonight's T and S arms do not hold, the next nights go to this margin. This record
is the diagnosis to design those arms from. It is sim only and CPU only: nothing was trained, and
grade.py's own episode was run with its traces kept.

## The probe

`grade.py` "x1.5" (experiments/policy/shaper-train): 20 s at the zero command, standclock coast, the
shaper at 8 Hz, the fast STS3215 model. Every tick the served target is the action's 0.5 s EMA plus
**1.5 × the action's deviation from it**. It scales the **policy's output gain** on everything faster
than about 0.3 Hz. It is not the servo P gain. It comes from the 11-02 stand-still record: the robot
diverged at x1 where the sim needed x1.5, so the sim is about 1.5× optimistic in loop gain, and a
hardware stand candidate needs x1.5 in sim.

## What was run

`experiments/policy/x15-margin/` (`trace_x15.py` calls `grade.episode` and keeps the per-tick joint,
gyro, foot and torso traces; `run.sh`, `analyze.py`, `modeshare.py`). It runs on main 46d8a7d with grade's
defaults (`GRADE_XML=bimo_biped_v6legs_meas_sole.xml`, `SHAPE_HZ=8`) and 8 grade seeds per cell, read-only on
`~/code/robot-mjx/sim/runs`. It reproduces the 11-23 grade exactly. For example M1 seed 16, grade seed 1
falls at 9.34 s in both, and every x1.5 count below for a student graded on 11-22 or 11-23 matches
`/tmp/plan1122/gradeD` / `/tmp/plan1123/gradeD`. In total there were about 2,100 episodes, at most 8
processes at a time (`nice`), about 25 min wall time. The traces and CSVs are in `/tmp/x15/` on mira.

- **Gain ladder**, plant D (`sag_L=6.4,sag_R=2.9`): x1.0 / 1.25 / 1.4 / 1.5 / 1.6 / 1.7 / 1.75 / 2.0.
  It covers 17 students: VW and M1 at seeds 14–18, M2 seed 17 (8/8 at x1.5 on 11-23), GD seeds 14–16, the
  parent prec16, and the two loop-gain-DR students prec34 / prec35 (11-09).
- **The shape of each fall**: the joint angles minus the **same run and grade seed at x1.0**. Both
  start from the same state, so the difference is what the gain did. From that difference: the onset (it
  first exceeds 1°), the growth time (1° → 5°), the joints that lead, and each joint group's share of the
  squared difference when it reaches 5°.
- **Joint-group gain** (6 students): x1.5 / x2 applied to the hip yaws only, the roll joints only, or the
  sagittal joints only. Then x1.5 on everything except the yaws, or except the rolls.
- **Gyro path** (5 students): x1.5 with the policy's gyro-z (or x, y) observation scaled ×0 / ×0.5 in
  every frame of the obs (the kind of observation gain `obsfreeze gain=` applies on the firmware).
- **Teachers** of 9 of the students on D at x1.0 / 1.5 / 1.6 / 1.75. **Plant C** (play + inset sole) for
  6 students at x1.0 / 1.25 / 1.5 / 1.75.

## Results

### 1. The margin of every student is 1.5 – 1.75, and the bar sits at its bottom edge

Falls of 8 on D. g\* = the lowest gain with ≥ 4/8 falls. Shares = the joint groups' share of the
falls' joint difference at g\* (a fall is "roll" when the roll joints carry most of it).

| student (seed) | x1.25 | x1.4 | x1.5 | x1.6 | x1.7 | x1.75 | x2 | g\* | falls at g\*: roll-led / yaw-led / sagittal-led | roll / yaw / sagittal share at g\* | x1.5 fall time, s (median) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| prec16 (14) | 0 | 0 | 0 | 8 | 8 | 8 | 8 | 1.6 | 8 / 0 / 0 | 0.82 / 0.06 / 0.11 | – |
| GD (14) | 0 | 0 | 0 | 0 | 3 | 7 | 8 | 1.75 | 5 / 0 / 2 | 0.50 / 0.28 / 0.23 | – |
| GD (15) | 0 | 0 | 0 | 6 | 8 | 8 | 8 | 1.6 | 4 / 2 / 0 | 0.62 / 0.13 / 0.24 | – |
| GD (16) | 0 | 0 | 0 | 8 | 8 | 8 | 8 | 1.6 | 8 / 0 / 0 | 0.92 / 0.00 / 0.08 | – |
| VW (14) | 0 | 0 | **5** | 8 | 8 | 8 | 8 | **1.5** | 0 / 5 / 0 | 0.08 / 0.65 / 0.27 | 5.8 |
| VW (15) | 0 | 0 | 0 | 0 | 8 | 8 | 8 | 1.7 | 0 / 7 / 1 | 0.04 / 0.34 / 0.63 | – |
| VW (16) | 0 | 0 | 0 | 8 | 8 | 8 | 8 | 1.6 | 0 / 0 / 8 | 0.00 / 0.07 / 0.92 | – |
| VW (17) | 0 | 0 | 0 | 0 | 2 | 4 | 8 | 1.75 | 0 / 3 / 1 | 0.30 / 0.37 / 0.33 | – |
| VW (18) | 0 | 0 | 0 | 0 | 0 | 1 | 8 | 2.0 | 2 / 0 / 6 | 0.30 / 0.10 / 0.60 | – |
| M1 (14) | 0 | 0 | **4** | 8 | 8 | 8 | 8 | **1.5** | 0 / 3 / 1 | 0.02 / 0.48 / 0.50 | 6.7 |
| M1 (15) | 0 | 0 | 0 | 0 | 4 | 7 | 8 | 1.7 | 3 / 1 / 0 | 0.60 / 0.34 / 0.06 | – |
| M1 (16) | 0 | 0 | **8** | 8 | 8 | 8 | 8 | **1.5** | 0 / 0 / 8 | 0.04 / 0.09 / 0.87 | 12.8 |
| M1 (17) | 0 | 0 | 1 | 8 | 8 | 8 | 8 | 1.6 | 3 / 2 / 3 | 0.39 / 0.22 / 0.39 | 19.9 |
| M1 (18) | 0 | 0 | 0 | 2 | 4 | 7 | 8 | 1.7 | 2 / 0 / 2 | 0.32 / 0.06 / 0.62 | – |
| M2 (17) | 0 | 0 | **8** | 8 | 8 | 8 | 8 | **1.5** | 0 / 7 / 1 | 0.08 / 0.49 / 0.43 | 7.4 |
| prec34 LG (14) | 0 | 0 | 0 | 0 | 8 | 8 | 8 | 1.7 | 0 / 0 / 8 | 0.00 / 0.05 / 0.95 | – |
| prec35 LG (14) | 0 | 0 | 0 | 0 | 0 | **0** | 8 | 2.0 | 0 / 8 / 0 | 0.33 / 0.59 / 0.09 | – |

- No student falls at x1.4. Without loop-gain DR, every g\* is 1.5 – 1.75 (VW 18: 2.0). The median is 1.6 for the
  volume students (11) and 1.6 for the non-volume ones (4). The volume does not lower the typical margin;
  it widens the spread. 4 of 11 volume students have g\* = 1.5 and 0 of 4 non-volume ones do. (GD seed 21
  fell 4/8 in the 11-22 grade, so non-volume seeds can fall too.)
- **The x1.5 count is one bit per student.** The 8 grade seeds vary only the IMU misalignment and bias,
  not the margin, so a student falls 0, or 4 – 8 (one 1/8). "x1.5 ≤ 2/8" is in effect "g\* > 1.5". 7 of 11
  volume students pass it on D. With P = 0.64 per seed, ≥ 2 of 3 seeds pass x1.5 alone with P ≈ 0.70.
  The joint bar (also ≥ 16 compass) is, as things stand, largely a draw.
- Only loop-gain DR has put a student outside that spread: prec35 (prec16 + `--loop-gain-range 1.0,1.6`)
  is 0/8 at x1.75, where prec16 is 8/8. prec34 (prec26 + the same) is at 1.7, inside the spread; prec26's
  x1.5 loss (6/6) was later shown to be the draw (its replicate prec36: 0/0), so prec34 is weak evidence. No run in the volume line (GD, VW, M1, M2,
  T, S) has it: `loop_gain_range` is null in every config.json.

### 2. The volume students fail in a different mode: a slow yaw-and-scissor drift, not the roll lean

- **Non-volume (prec16, GD): the roll lean.** Both hip rolls move together (`Lhr+ Rhr+`): a lateral
  sway and lean over the feet. The roll joints carry 0.50 – 0.92 of the difference. It is fast: 1° → 5° in 0.8 s (median of 29 falls), and the fall comes at 4.6 s.
- **The volume students that fall at x1.5 (VW 14, M1 14, M1 16, M2 17): yaw and scissor.** The hip yaws,
  hip pitches that go opposite ways (one leg forward, one back: `Lhp− Rhp+` in 3 of 4) and the ankle
  pitches. The roll share is 0.02 – 0.08, and the body rate is mostly yaw (3 of 4). It is **slow**: in M1 14
  and M1 16 the leading joint's difference grows without crossing zero, and in VW 14 and M2 17 it wobbles
  at about 0.4 Hz while it grows. 1° → 5° takes a median 1.8 s (up to 9.6 s); the fall comes at a median 7.9 s (4.6 – 16.1), 80 % of them after 6 s. By the fall the body has
  yawed a median 65° (20 – 110°) and moved a median 0.3 m: it steps off its spot, then falls.
- So it is **a weakly unstable slow mode that the gain tips over, not a limit cycle at a servo or
  body-mode frequency** (the robot's under-modelled body mode is ~3 Hz), and not a disturbance the policy fails to reject. At x1.25, M1 16 already drifts the same
  way (Ran −4.7°, Rhp +4.0°, Rhy −3.8°, body yaw 16° in 20 s) but stays bounded.
- It grows along the posture each student already stands in. At x1.0, M1 14 stands with the largest
  hip-yaw offset of the 17 (6.5° summed, median 1.8°) and fails yaw-first. M1 16 stands with the largest
  hip-pitch split (Lhp − Rhp = −6.7°, median −1.5°) and fails scissor-first. Across the 17, though, the
  stand pose predicts g\* only weakly (|hip yaw| r = −0.50; the distance from home −0.37). Treat this as a
  hint, not evidence.

### 3. Amplifying the hip yaws is needed for the volume failures; the gyro is not the path

Falls of 8 on D, gain applied only to the listed joints:

| student | all x1.5 | yaw only x1.5 / x2 | roll only x1.5 / x2 | sagittal only x1.5 / x2 | all but yaw x1.5 | all but roll x1.5 |
|---|---|---|---|---|---|---|
| prec16 | 0 | 0 / 0 | 0 / 8 | 0 / 8 | 0 | 0 |
| GD (14) | 0 | 0 / 0 | 0 / 8 | 0 / 3 | 0 | 0 |
| VW (14) | 5 | 0 / 4 | 0 / 8 | 0 / 8 | **0** | 0 |
| M1 (14) | 4 | 0 / 0 | 0 / **0** | 0 / 8 | **0** | 2 |
| M1 (16) | 8 | 0 / 0 | 0 / **0** | 2 / 8 | **3** | 8 |
| M2 (17) | 8 | 0 / 7 | 0 / 8 | 0 / 8 | **0** | 0 |

- With the hip yaws left at x1, x1.5 on everything else loses 3 of the 4 volume failures entirely and
  cuts M1 16 from 8 to 3. Neither the yaws alone nor the sagittal joints alone fall at x1.5. **The failing
  loop couples hip yaw with the sagittal joints**, which is the twist of a turn in place. The volume
  trains turning in place and small-wz walking from zero (`volume_cmd` is uniform in radius down to zero,
  the turn uniform on ±1 rad/s). That is a plausible origin, but it is not proven here.
- M1 14 and M1 16 are the only two of the six that hold roll-only x2 (0/8; prec16, GD, VW, M2 8/8). That
  fits the reference's foot placement helping the roll lean. It does nothing for the yaw/scissor mode.
- **The yaw-rate gyro is not the path.** Scaling the gyro-z observation ×0 / ×0.5 at x1.5 leaves the
  falls in place: VW 14 7/7, M1 14 6/5, M1 16 8/8, M2 17 8/8. GD 14 at x1.75 stays 8/8. Gyro x or y ×0.5
  does not help either (VW 14 6/6, M1 14 3/2, M1 16 8/8, M2 17 8/8). The loop closes through the legs'
  own joint feedback, so an `obsfreeze gain=` on the firmware will not fix it, and neither will narrowing
  `--gyro-gain-range` (already 0.7 – 1.6 in the recipe).

### 4. The teacher sets the margin; the distil moves it about ±0.1

Teachers on D, falls at x1.5 / 1.6 / 1.75 (student at x1.5):
M1 14 8/8/8 (4); M1 15 0/0/8 (0); M1 16 7/8/8 (8); M1 17 5/8/8 (1); VW 14 0/6/8 (5); VW 16 8/8/8 (0);
M2 17 8/8/8 (8); GD 14 0/0/7 (0); GD 15 1/5/8 (0). The volume teachers fail in the same yaw/scissor mode (GD 15's
teacher in roll, GD 14's mixed). The distil moves g\* by about one ladder step either way (VW 14 down, VW 16 up). Distilling with
noise would not fix this; the margin has to come from training.

### 5. Plant C is the same, one step worse

At x1.5 on C: M1 14 3/8, M1 15 **8/8**, M1 16 8/8, VW 15 **8/8**, GD 14 2/8, prec35 LG **0/8** (8/8 at
x1.75). M1 15 and VW 15, clean on D, fall on C in the yaw mode (yaw share 0.63 / 0.62). M1 16 falls in roll
on C. prec35 is the only student clean at x1.5 on both D and C.

### 6. Not the cause: foot creep

At x1.0, every student's feet creep on the floor over the 20 s. The two soles together move 8 – 38 mm, the
stance widens by up to 38 mm, and the yaw drifts 0.2 – 10°. This holds for prec16 and GD too, and it
does not predict g\* (r = −0.1). It is noted because the sim's 20 s stand is not static, and longer stand
probes should expect it.

## What the training side already has (flags, no code)

- `--loop-gain-range LO,HI`: the probe's own law as per-episode DR. The served targets are the 0.5 s EMA
  + g × the deviation, with g ~ U(LO, HI); the policy sees its raw action. It is the only lever with a
  measured effect on g\* (results 1, 5), on one run (prec35). Its cost on 11-09 was turning: prec35 turned +62 / −77° against
  prec16's +114 / −106°, and side R was slower. On prec34 the card clearance fell 1.6 → 0.9 cm. The
  11-10 ledger parked it for that reason: "a margin tool that costs directions".
- `--servo-kp-range` (0.6 – 1.2 in the recipe) randomises the servo P, not the policy's output gain.
  `--gyro-gain-range` is already wide and is not the path (result 3).
- Stand-gated shaping: `--w-stand-home` (an L1 pull of the served target onto home at the zero command),
  `--w-still`, `--w-stand-zero`. `--w-squeeze` is already 1.0. `--w-hip-yaw` is ungated and would fight
  every turn: do not use it on the volume.
- Nothing lengthens the stand alone. `--cmd-resample-s 2.5,4.5` and 10 s episodes mean training rarely
  stands for more than ~5 s, while the yaw/scissor falls come at a median 7.9 s. That is a guess about why
  the mode is not trained out, and it would need code to test cleanly.

## Candidate arms for the planner (ranked)

Each arm is M1's recipe (`night/fallback.job`, warm start prec86 as M1, 300M + distil, seeds 14 – 16)
plus one variable. **Grade them on a ladder, not on x1.5 alone**: add x1.6 and x1.75 (`out_gain` 1.6 /
1.75, a one-line `PROBES` entry or `run.sh` here) and report g\*. The x1.5 count is one bit per student
(result 1), and the ladder shows the margin moving before the bit flips.

1. **LG: `--loop-gain-range 1.0,1.6`** (prec35's exact setting).
   - *Evidence:* it is the only lever that has moved g\* out of the untreated spread (prec16 1.6 → prec35
     2.0; one run; prec34's 1.7 is inside the spread). prec35 is the only student clean at x1.5 on both D
     and C. It trains on exactly the law the probe applies. The 1.5× was measured on the robot's stand
     (11-02), and nothing suggests the robot's gain is lower when it walks.
   - *Guess:* that it works the same on the volume and on the yaw/scissor mode, and how much compass it costs.
   - *Pre-commit:* x1.5 ≤ 2/8 on D on **3 of 3** seeds and on C on ≥ 2 of 3; g\* ≥ 1.7 on ≥ 2 of 3. The
     cost guard: compass ≥ 16 on ≥ 2 of 3, and dir turn L / R ≥ 90° (prec35 lost ~45 % of its turn).
2. **LG-half: `--loop-gain-range 1.0,1.3`.**
   - *Guess (dose):* g\* only has to clear 1.5 by a step, and the volume students sit at 1.5 – 1.75, so a
     narrower draw may be enough and cost less turning. Nothing measured supports the range; it brackets
     arm 1.
   - *Pre-commit:* g\* ≥ 1.6 on 3 of 3 (x1.5 ≤ 2/8 on D on 3 of 3); compass and turn within 1 probe /
     10 % of M1 at the same seed.
   - If arm 1 holds the margin and loses the compass while arm 2 keeps both, arm 2 is the recipe.
3. **Stand pull: `--w-stand-home 0.05`.**
   - *Guess:* the failing mode grows slowly along the stand pose's own asymmetry (result 2), so a small
     stand-gated pull onto home would give that slow drift a restoring cost without touching the walk.
   - Scale: the stand's Σ|target − home| is 0.25 – 0.41 rad, so 0.05 costs about 0.015 a tick, ~5 % of the
     alive bonus. No run in this line has used it. Older home-pull runs could not crouch (v30/v31home,
     09-06), before the gate released on a crouch command.
   - *Pre-commit:* g\* ≥ 1.6 on ≥ 2 of 3 with the yaw/scissor share at g\* < 0.5, and compass ≥ M1's at the
     same seed. Run it only if the night has room after 1 and 2; it is the weakest-evidenced arm.

A control is not needed: M1 has five graded seeds and their g\* are in the table above (1.5, 1.7, 1.5,
1.6, 1.7). The morning grade of tonight's T and S arms should add the ladder too, since the turn reference
(T) changes how the hip yaws are used.

## Open

- **Why the volume adds the yaw/scissor mode.** The pivot and small-wz draws are a plausible cause, but
  the cause is not isolated. A test: the joint-group ladder on T's students, whose reference does yaw the
  hips.
- **No dose-response for loop-gain DR.** There are only two runs, both at 1.0 – 1.6 and both non-volume,
  and only one of them (prec35) clearly moved g\*.
- These are sim-only margins on the sim's plants. On the robot the 1.5× has only been checked for a stand
  (11-02).
