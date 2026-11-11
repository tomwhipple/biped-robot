# Weight-transfer gate: does the walking plant reproduce the robot? (2026-11-10, #259)

Two checks of the walking plant (`sim/bimo_biped_v6legs_meas_sole.xml`) against what the robot has
already shown about weight transfer, a first attribution pass, and a second (Round 2: the pipeline's
yaw sign, the one-fault hypothesis, compliance beyond the encoders, grade.py's roll opt-ins), and a
refit against the 2026-11-10 robot measurements (Round 3). CPU only; no plant XML, training
default or DR default is changed. Every term below is opt-in (`--terms`).

```bash
sh experiments/policy/weight-transfer-gate/run_all.sh        # every table here, ~3 min, -> results/
.venv/bin/python experiments/policy/weight-transfer-gate/check1_oneleg.py [--plants ...] [--terms ...]
.venv/bin/python experiments/policy/weight-transfer-gate/check2_replay.py [--plants ...] [--terms ...] [--fk-trace]
```

Recordings and the prec16 run are read from the main checkout (`GATE_MAIN`, default
`/home/claw/code/robot`): `hw_sessions/2026-10-24/stand/floorR2_serial.log`,
`hw_sessions/2026-10-25/{clrR,clrL,latR,latL}_serial.log`,
`hw_sessions/2026-11-09/flight_20261109T232022Z_prec16.frec`,
`sim/runs/robot_legs_prec16_f075_js20_s128r24`. `tests/test_weight_transfer_gate.py` pins the machinery
(log parsing, chain, terms, metrics) and, where the logs exist, that the parser recovers the
NOTES.md robot numbers from the raw logs.

## The plants

| name | what it is |
|---|---|
| `train` | the prec16 recipe's training actuator chain at the **centre** of its DR, on the CPU `walker_env` built from the run's `config.json` (DR off). The CPU env lacks three MJX-only knobs, so `gate_common.TrainChain` adds them in front: per-type dead time (3 ticks STS3215 / 0.5 STS3250), the one-stage action lag (5.3 Hz, x4 at the hip rolls), servo kp x0.9 / kd x0.56. Roll lash 1.25 deg, backlash 0.125 deg, joint friction 0.25, latency 4 ms, mu 0.5, torso x1, CoM 0 |
| `train:lash0.5`, `train:lash2.0` | the same with the roll lash at its DR ends |
| `grade` | what `grade.py`'s `robot` probes run (the `stride1 0.3 robot` probe that lifts prec16's R foot 2.1 cm): `simh.build("fit")` + `JF_PLANT` + `ROBOT_SPEED_PLANT`, `TypedServo(ROBOT_SPEED_SERVO)` in front. `GRADE_ROLL_LASH=<deg>` / `GRADE_ROLL_SERIES=hip=<k>[,ankle=<k>]` (opt-in, unset = as before) give grade.py's every probe the roll play or the series spring (round 2) |
| `oneleg:fit` | reference only, not the walking plant: the 10-30 one-leg fit's own harness plant (`bimo_biped_v6legs.xml`, `simfit_plant.json`) |

**A premise correction.** The issue says the one-leg fit's values "never reached the walking
plant". For `grade` that is only half true: `simh.build("fit")` applies `simfit_plant.json`
(torso x0.702 = 0.319 kg, torso CoM -6.8 mm, 81 mm sole, carpet contact, L ankle-roll zero error
-2.12 deg), and then `JF_PLANT` sets the **roll lash and the backlash to zero**. The `train` plant
has none of the fit (torso x1 +-40 %, CoM 0 +-15 mm, lash 0.5-2.0 deg in DR).

**Harness checks.** The grade plant on the clamped-foot test gives 2.33 / 2.19 deg, the "rigid
rolls" row of [roll-compliance-2026-10-24](../../../docs/design-v6/roll-compliance-2026-10-24.md)
(2.35 / 2.37); `closed grade probe` reproduces the planner's prec16 `stride1 0.3 robot` lift
(3.12 / 2.05 cm over 8 seeds; planner 3.1 / 2.1). The 10-30 one-leg Replay on the fitted plant gives
an l30 err of -2.58 deg for 222031R (robot -3.16).

## Check 1: the one-leg protocol (10-24 / 10-25) on the walking plant

Every `pose` the robot was sent in the serial logs, with its own timing, through
`simfit_sim.Replay`: the firmware streamer and the bench roll PI, with the gains the logs print
(kp 0.25, ki 0.5, ankle +1 / hip -1, clamp 10, abort 12, no gyro term). The robot's err numbers are
its ~1 Hz `balance` polls, so the sim is read at the same instants ("polled"). Clearance on the
robot is a person's eyes or EMEET frames (NOTES.md). In the sim it is the lowest pad of the lifted
foot above the floor.

**Tip toward the lifted foot (err peak, polled, deg) and clearance** (`results/check1.md`):

| test | robot | train | train lash 0.5 / 2.0 | grade | robot clearance | sim clearance (all plants) |
|---|---|---|---|---|---|---|
| 62 mm shift, 10 mm lift (R stance) | -2.73 | -1.23 | -1.74 / -1.17 | -1.52 | no | 0-1.3 mm: no |
| 62 mm, 30 mm lift | **-5.21** | -1.24 | -1.87 / -1.16 | -1.54 | yes (frame) | 18-21 mm |
| 80.6 mm, 10 mm lift, R / L stance | -1.25 / +1.27 | -1.39 / +1.39 | | -1.33 / +1.82 | not stated | 0-1.5 mm |
| 80.6 mm, 30 mm lift, R / L stance | **-3.17 / +4.05** | -1.66 / +1.44 | -1.26 / +1.15, -2.23 / +2.05 | -1.39 / +2.60 | yes, ~1-2 cm | 15-22 mm |

**The unloading-tilt curve** (5 mm lift, no feedback, roll change in deg; R stance, L stance is
the mirror within 0.1 deg in both robot and sim):

| shift x 62 mm | 0.5 | 0.75 | 1.0 | 1.1 | 1.2 | 1.3 | 1.4 |
|---|---|---|---|---|---|---|---|
| robot | -1.91 | -1.85 | -1.69 | -1.67 | -1.45 | **-1.30** | -1.42 |
| train | -2.38 | -1.59 | -1.59 | -1.58 | -1.55 | -1.54 | -1.61 |
| grade | -1.66 | -1.55 | -1.43 | -1.43 | -1.40 | -1.36 | -1.36 |

**Clamped-foot 30 mm lift, no shift** (deg toward the lifted side; stance encoders ankle / hip):
robot 5.2 / 5.0 (1.0-1.5 / 0.7-0.9); train 4.17 / 4.20 (1.5 / 0.8); train lash 2.0 4.99 / 5.02;
**grade 2.33 / 2.19**.

**Verdict, check 1: no.**
- The robot's 30 mm tip depends on the shift: -5.2 deg at 62 mm, roughly halved at 80.6 mm. The
  walking plant's does not: -1.2 to -1.9 deg at 62 mm (a quarter to a third of the robot's), and
  about the same at 80.6 mm.
- The robot's unloading tilt keeps falling until 1.3x the shift. The plant's is flat from 0.75x
  (train) or barely slopes (grade), so in the plant the lifted foot is already unloaded at the
  model's shift. This is the robot's "needs ~1.3x the model shift", and the plant does not show it.
- Clearance agrees: a 10 mm command does not clear the floor in any plant (robot: not at 62 mm),
  and a 30 mm command clears 15-22 mm (robot: ~1-2 cm, a pixel estimate).
- Clamped tilt: `train` is 1.0 deg low at its DR centre and matches at the 2.0 deg lash end.
  **`grade` is 2.9 deg low: grade.py's robot probes run with zero roll play.**

## Check 2: the 11-09 prec16 stride replayed

Robot: the flight record; t = 0 at the first tick with the walk command (vx 0.2). The guard
tripped at +0.64 s (rate 3.02) and ramped the command to the stand over 0.3 s.
`closed <plant> record` feeds the student the robot's own per-tick command and gait phase, through
the grade.py firmware chain (shaper 8 Hz, takeover ramp, reg-58 dq obs). `goals` writes the robot's
written goals into the plant's servo chain, open loop. `joints` imposes the robot's measured joint
angles by equality constraints (the yaw-gap method). Means over 8 seeds, ranges in
`results/check2.md`. Lift is the true sole rise. FK is the sole-centre height difference computed from
the encoders and the IMU attitude, for the robot as for the sim.

| row | lift L / R mm | FK lift L / R mm | torso roll toward L, deg | hip roll travel L / R deg | knee L / R deg | yaw at +2 s deg | gz peak rad/s | guard-rate EMA peak |
|---|---|---|---|---|---|---|---|---|
| **robot** | unknown (see below) | 16.9 / 13.0 | 6.8 | 4.2 / 7.9 | 57.1 / 47.6 | **-45.0** | **-7.2** | **3.5** (trip 3.02) |
| closed grade record | 14.2 / **20.3** | 14.1 / 20.7 | 4.9 | 7.7 / 7.6 | 36.5 / 47.5 | +11.0 | +1.8 | 0.8 |
| closed train record | 20.8 / **21.4** | 19.9 / 21.0 | 4.5 | 4.8 / 9.1 | 54.6 / 45.0 | +3.7 | -0.4 | 0.6 |
| closed grade script (stride1_02s as written) | 29.4 / 19.9 | | 4.8 | 7.3 / 11.8 | 64.9 / 46.3 | -8.9 | -2.4 | 1.0 |
| goals grade | 30.5 / 18.9 | 30.8 / 20.1 | 4.3 | 5.1 / 8.3 | 63.1 / 52.3 | **+32.3** | +3.1 | 1.6 |
| goals train | 25.9 / 22.3 | 25.7 / 22.3 | 4.6 | 4.0 / 6.1 | 58.7 / 49.0 | +38.4 | +3.1 | 1.7 |
| joints (measured q imposed) | 23.2 / 21.8 | 25.2 / 21.6 | 5.8 | 4.2 / 7.9 | 56.9 / 47.5 | **+44.5** | +4.0 | 1.7 |

**Verdict, check 2: no**, and the gap is not in the policy or the actuator chain.
- **Joints match, the body does not.** Closed loop, the student under the robot's command moves the
  R leg as the robot did (knee 47.5 vs 47.6 deg, hip roll 7.6-9.1 vs 7.9). Given that motion, the
  plant lifts the R foot about 2 cm in every row, including the open-loop rows. The rate guard's
  peak is 0.6-1.7 against the robot's 3.5, and no plant variant trips it on rate.
- **The yaw has the wrong sign.** The robot turned 45 deg right (gz -7.2). With the robot's own
  joint angles imposed, the plant turns **44.5 deg left** (gz +4.0); open-loop goals turn it +32 to
  +38. So the plant's body and contact turn the robot's leg motion into the opposite rotation. This
  is the largest discrepancy, and none of the terms below changes its sign.
- The plant's hip-roll travel (4-9 deg) is the robot's (4-8). The robot's lack of shift is not a
  hip-roll command the sim did not make.

**What the robot's sensors say about the R foot** (`results/check2_fk_trace.md`, the plant's
geometry with the encoders and the fused attitude). The R sole centre rises 9.7-12.6 mm over
+0.48..+0.60 s. The lowest R pad sits up to 13.8 mm above the lowest L pad at +0.64 s. The L
(stance) sole reads 5-8 deg beyond its standing tilt (1.5 deg) from +0.5 to +1.1 s, while the
torso rolls 6.8 deg toward L. So either the robot rocked onto the edge of its L foot and briefly
raised the R foot ~1 cm, or 5-8 deg of the torso's roll is play or compliance beyond the encoders
(the 10-24 clamp signature) and the FK lift is an over-estimate. The session's "the R foot never
left the floor" is an observation, not a measurement. The EMEET frames at +0.45..+0.55 s show the
R foot tilted onto an edge, not flat, and cannot settle the question. **The robot's per-foot lift
is unmeasured.** That is the measurement this gate most needs (below).

## Attribution probes (one term set at a time; `results/probe_check{1,2}_*.md`)

| term | check 1: 62 mm / 80.6 mm R / 80.6 mm L 30 mm tip, train (lash 2.0) | clamp tilt train | check 2 closed record, train: lift R mm, yaw deg | goals grade: yaw |
|---|---|---|---|---|
| none | -1.24 / -1.66 / +1.44 (-1.16 / -2.23 / +2.05) | 4.2 | 21.4, +3.7 | +32.3 |
| `lash_fit` (0.96 ankle / 1.55 hip deg) | -1.43 / -2.01 / +1.81 | 4.1-4.2 | 20.6, +3.0 | +31.6 |
| `torso=0.702,com_y=-0.0068` | -1.38 / -1.62 / +1.39 | 3.8-3.9 | 21.2, +2.1 | +30.2 |
| `lash=2.6` (the 10-24 calibrated total) | -1.15 / -2.73 / +2.66 | 5.6-5.7 | 17.9, -0.4 | +29.2 |
| `roll_k=0.3` (roll servos x0.3: a spring) | falls at 30 mm | 11.9-12.0 (encoders 7.2: contradicts the robot's 1.0-1.5) | 12.1, +16.3 | +33.5 |
| `dead=40` (loaded tracking lag) | -1.27 / -1.70 / +1.47 | 4.2 | 21.3, -1.0 | +25.0 |
| **`torso=0.6,legs=1.2`** (mass from torso to legs, total kept) | -2.78 / -1.75 / +1.51 (**-3.60 / -2.31 / +2.05**) | 4.1 (4.9-5.0) | 20.4, +1.6 | +30.3 |

Best supported, with the evidence:
- **Check 1: mass distribution together with roll play moves the plant toward the robot.** Only
  a lighter torso (`torso=0.6`: 62 mm -2.58, 80.6 mm -1.62 on `train`; and `torso=0.6,legs=1.2`,
  the same mass moved into the legs) brings back the robot's shift dependence. With the 2.0 deg lash end, the
  62 mm tip becomes -3.6 deg (robot -5.2), the 80.6 mm R tip -2.3 (robot -3.2), and the clamp
  tilt 4.9-5.0 (robot 5.0-5.2). The 80.6/62 ratio is 0.64 against the robot's 0.61. Two things
  support this direction: the 10-30 fit's torso x0.70 (cliff at 0.78), and the kitchen-scale total
  of 1538 g (2026-11-01-floors-fit.md, against the plant's 1501 g). Neither measures the split. It
  still leaves 30 % of the 62 mm tip and the L-stance 80.6 mm tip (+2.05 vs +4.05) unexplained,
  and the unloading curve stays flatter than the robot's. The one-leg fit's lash, CoM and torso
  values alone do not move check 1. Roll compliance as a servo spring contradicts the robot's
  encoders.
- **Check 2: unexplained by every candidate.** No term brings the R lift below ~12 mm or changes
  the sign of the yaw. A roll spring (`roll_k=0.3`) lowers the lift most, to 12 mm, at the price
  of an encoder reading the robot does not show. Given the robot's own joint angles, the rigid plant
  rotates the other way, so the next candidates are in the body and contact: which foot carries the
  load and pivots (the FK says the robot's L stance sole was on an edge, 5-8 deg off level), the
  sole/board friction and torsion, and compliance beyond the encoders that the joint-equality
  replay cannot express.

## Round 2 (2026-11-10): sign check, one fault or two, compliance beyond the encoders

```bash
.venv/bin/python experiments/policy/weight-transfer-gate/check2_replay.py --record 1106t1 --schedules "" --seeds 0
.venv/bin/python experiments/policy/weight-transfer-gate/check2_replay.py --joint-variants
.venv/bin/python experiments/policy/weight-transfer-gate/fit_series.py
.venv/bin/python experiments/policy/weight-transfer-gate/wt_grade.py 8
```

### 1. The pipeline's yaw sign, on a record where the plant agreed (`results/signcheck_*.md`)

`--record 1106t1|1106t2` runs the same loader, metrics and replays on omni5's 2026-11-06 records
(`flight_20261106T160828Z_omni5.frec`, walk commands 1 and 2). Yaw in deg from the walk command:

| 11-06 | robot +0.4 / +0.7 / +2 s | joints replay, mu 0.5 | yaw-gap README (its own replay) |
|---|---|---|---|
| t1, stride1_03 | -2.4 / **+24.5** / -2.6 | -10.4 / **+46.1** / -7.4 | robot -3 to +27, replay -10 to +50 (+0.4 to +0.7 s) |
| t2, steps5_03 | -2.6 / **+43.9** / +70.0 | -18.7 / **+28.0** / -25.7 | robot +42 by +0.72 s |

The pipeline reproduces the precedent's numbers (-10.4 / +46.1 against -10 / +50) and the robot's
sign in the second swing on both 11-06 records. So the sign is not a convention error. The 11-09
flip (robot -45, replay +44) is in the plant's response to the robot's joint motion. The size does
not agree on 11-06 either (t1: +46 against +24.5), and by +2 s t2 has gone the other way (-25.7
against +70).

### 2. One fault? Keeping the R foot loaded in the 11-09 joints replay (`results/r2_item2_joint_variants.md`)

Measured joints imposed, mu 0.5. Perturbations: a constant pelvis roll toward R (both hip-roll
targets offset; 4 deg is what the 10-24 clamp puts beyond the encoders), the same at the ankle
rolls (the whole body leans), torso CoM toward R, and the torso-to-legs mass move.

| perturbation | R foot's share of the floor load, +0.4..+0.7 s, min / mean | lift L / R mm | yaw +0.7 / +2 s deg | gz peak | guard-rate peak |
|---|---|---|---|---|---|
| robot | unknown | unknown (FK 16.9 / 13.0) | -43.6 / -45.0 | -7.2 | 3.5 |
| plant | 0.00 / 0.20 | 23.0 / 21.8 | +12.5 / +43.6 | +4.0 | 1.7 |
| CoM 10 / 20 / 30 / 50 mm toward R | 0.00 / 0.13-0.20 | 22-24 / 22-23 | +8.6..+15.2 / +43.9..+45.6 | +4.0..+4.9 | 1.7-1.8 |
| torso x0.6, legs x1.2 | 0.00 / 0.20 | 23.0 / 19.1 | +11.6 / +52.2 | +3.7 | 2.0 |
| pelvis 4 / 8 deg toward R | 0.00 / 0.13, 0.20 | 27.1 / 14.7, 20.5 / 28.9 | +7.5 / +48.4, +9.4 / +43.6 | +2.3, +2.8 | 1.6, 1.9 |
| whole body 4 / 8 / 12 deg toward R | 0.00 / 0.23, 0.27, 0.93 | 27.8 / 17.6, 39.7 / 10.3, 22.4 / 35.4 | +5.6 / +59.3, +2.8 / +17.3, +11.0 / +132.7 | +2.6, +1.7, +7.0 | 2.0, 1.4, 3.6 |

**None keeps the R foot loaded.** Its share reaches 0 inside +0.4..+0.7 s in every row. None
gives the robot's yaw sign. The rows that reach the robot's rate (whole body 12 deg: 3.6) do so
by falling over with the opposite yaw (+133 deg). Within these candidates the "one fault"
hypothesis is not supported. With the robot's joint angles, the rigid plant unloads the R foot
whatever its CoM or a fixed roll offset does.

### 3. A spring beyond the encoders, fitted to the 10-24 clamp only (`fit_series.py`, `results/r2_item3_*`)

**How it is modelled.** `walker_env` needs its actuated joints consecutive in qpos, so a real
joint inside the leg chain cannot go into the walking env. The `ser_hip` / `ser_ankle` terms use
its quasi-static form. The link sees k_eff = kp k_s / (kp + k_s), the roll PD scaled by
k_s / (kp + k_s). The encoder (the policy's joint obs, the one-leg streamer, the clamp encoders)
reads the servo shaft, q + kp / (kp + k_s) (target − q), so only the servo's own deflection
shows. The shaft is massless (no extra mode), and joint velocities stay the link's. The
bare-plant joints replay uses a real sprung joint (`gate_common.add_series_joint`). Roll play is
set to 0, so the spring carries everything beyond the encoders. There is no free play in this
fit: the bench cannot split play from spring, and a spring alone reaches the signature.

| placement | k, N m/rad | clamp tilt R / L lifted (robot 5.2 / 5.0) | stance encoders ankle / hip (robot 1.0-1.5 / 0.7-0.9) |
|---|---|---|---|
| hip rolls | **14.2** | 5.08 / 5.13 | **1.39-1.41 / 0.79**: all three numbers inside the bench's |
| ankle rolls | 27.8 | 5.00 / 5.09 | 1.68-1.71 / 0.80: ankle encoder 0.2 deg above the bench |
| both | 40.1 | 5.06 / 5.14 | 1.58-1.61 / 0.80 |

Then the checks, untuned (`train` = the training chain; `grade` = grade.py's plant, same k):

| | robot | hip 14.2, train / grade | ankle 27.8, train / grade | both 40.1, train / grade |
|---|---|---|---|---|
| 62 mm, 30 mm tip | -5.21 | **-3.41 / -3.40** | -1.81 / -1.92 | -2.34 / -2.42 |
| 80.6 mm, 30 mm tip R / L | -3.17 / +4.05 | **falls both** / -3.45 / +4.94 | -1.21 / +1.17, -1.39 / +3.04 | -1.99 / +1.92, -2.15 / +3.60 |
| unloading tilt 0.5 / 1.0 / 1.3x (R) | -1.91 / -1.69 / -1.30 | -2.13 / -2.03 / -1.93, -1.82 / -1.70 / -1.57 | **-1.87 / -1.50 / -1.36**, -1.69 / -1.45 / -1.38 | -1.99 / -1.74 / -1.72, -1.71 / -1.53 / -1.44 |
| 11-09 closed loop (record), R lift mm | ~0 by eye, FK 13.0 | **16.4 / 12.2** | 23.2 / 20.7 | 22.2 / 18.4 |
| same, yaw +2 s / gz / rate | -45.0 / -7.2 / 3.5 | +12.3 / +1.2 / 0.6, +18.3 / +1.2 / 0.7 | +3.5 / -0.1 / 0.6, +3.9 / +1.3 / 0.7 | +4.8 / +0.5 / 0.6, +23.5 / +1.6 / 0.8 |
| joints replay (real sprung joint): R lift, yaw +2 s, gz, rate | | 18.7, +28.6, +3.2, 1.8 | 23.2, **-3.4**, **-2.7**, 1.7 | 22.5, +27.6, +3.6, 1.5 |
| prec16 `stride1 0.3 robot` (grade, 4 seeds): lift L / R cm, falls | | 2.89 / **1.20**, 0 falls | 3.20 / 2.07, 0 | 2.86 / 1.95, 0 |

What it shows:
- **The hip-roll spring is the first term that fits the bench's own clamp signature in full**
  (tilt and both encoders), and it moves the most numbers toward the robot. The 62 mm tip goes from
  -1.2..-1.9 to -3.4 deg (robot -5.2). On the grade plant the 80.6 mm tips become -3.45 / +4.94
  (robot -3.17 / +4.05). It cuts prec16's 11-09 R lift from 20-21 to 12-16 mm, and the grade
  probe's from 2.05 to 1.20 cm. The plant still stands and walks the probe (0 falls of 4).
- What it does not do: the tip still does not depend on the shift (grade: -3.40 at 62 mm,
  -3.45 at 80.6 mm, where the robot halves it). On `train` the 80.6 mm stand **falls** outward on
  both stances, which the robot did not. The unloading curve stays flat. The yaw keeps the plant's
  sign, and the guard rate stays at 0.6-0.8.
- The ankle-roll spring matches the unloading-tilt curve best of anything tried (-1.87 / -1.50 /
  -1.36 against -1.91 / -1.69 / -1.30). It is the only term that turns the 11-09 joints replay's
  yaw to the robot's sign, though at a tenth of the size (-3.4 against -45 deg). It reads 0.2 deg
  high at the ankle encoder and does nothing for the tip or the closed-loop lift.
- So compliance beyond the encoders is supported by the bench record and moves check 1 and the
  closed-loop lift the right way. It explains neither the shift dependence nor the 11-09 yaw and
  body rate.

The clamp test now reads its encoders through `gate_common.clamp_run`. `sim/roll_compliance.run`
subtracts the full roll deadzone from every roll encoder, which over-subtracts where the play differs
per joint (`lash_fit`: ankle 1.19 → 1.48 deg). The rows with one width for all four rolls are
unchanged.

### 4. grade.py with the roll chain (`wt_grade.py`, `results/r2_item4_wt_grade.md`)

`GRADE_ROLL_LASH` / `GRADE_ROLL_SERIES` are opt-in (`grade.roll_plant_opts`, unset = the plant as
before; `tests/test_weight_transfer_gate.py`). `wt_grade.py` re-implements the planner's
lift_cm / shift_when_up. Its "grade as is" rows reproduce the 11-10 planner's table (prec16
3.12 / 2.05 cm, -0.22 / +0.16; prec37 4.37 / 2.97 cm, -0.30 / +0.30). 8 seeds, SHAPE_HZ 8:

| student | plant | stride1 0.3 robot: lift L / R cm (min); shift L-up / R-up | steps5 0.3 robot: lift L / R cm (min); shift |
|---|---|---|---|
| prec16 | as is | 3.12 (2.95) / 2.05 (1.92); -0.22 / +0.16 | 3.12 (2.95) / 2.22 (2.16); -0.39 / +0.50 |
| prec16 | roll lash 1.25 deg | 3.17 (3.05) / 1.93 (1.76); -0.22 / +0.15 | 3.17 (3.05) / 1.94 (1.80); -0.44 / +0.44 |
| prec16 | series hip 14.2 | 2.84 (2.71) / **1.24 (1.12)**; -0.17 / +0.15 | 2.84 (2.71) / 2.12 (1.98); -0.36 / +0.46 |
| prec37 | as is | 4.37 (4.23) / 2.97 (2.82); -0.30 / +0.30 | 4.37 (4.23) / 2.97 (2.82); -0.44 / +0.60 |
| prec37 | roll lash 1.25 deg | 4.05 (3.94) / 2.43 (2.28); -0.29 / +0.25 | 4.07 (3.94) / 2.74 (2.62); -0.48 / +0.60 |
| prec37 | series hip 14.2 | 3.93 (3.82) / 2.19 (2.06); -0.28 / +0.25 | 3.93 (3.82) / 2.49 (2.44); -0.45 / +0.55 |

There are no falls in any cell. The guard trips are all the `pose` criterion at 3.98 s, under the
stand command after the stride. On `stride1 0.3 robot` they occur in 1/8 (prec16 as is) and 8/8
in every other cell; on `steps5` there are none.

With the roll chain the R lift drops on both students, most with the hip spring (prec16
2.05 → 1.24 cm, prec37 2.97 → 2.19 cm). The shifts barely move. prec37 still clears the 2 cm bar
on both feet under every variant, and prec16's R foot no longer does on stride1. Neither plant
passed check 1 or 2, so this changes which plant grade.py ranks on only if the planner chooses it.

## Round 3 (2026-11-11): refit against the 2026-11-10 robot measurements

The data are in [2026-11-10-robot/README.md](2026-11-10-robot/README.md) (#271, #277): the mass, the
scale ladder, the stride repeated with floor cameras, and the clamp test with front-on stills.

```bash
.venv/bin/python experiments/policy/weight-transfer-gate/fit_clamp3.py          # item 1
.venv/bin/python experiments/policy/weight-transfer-gate/ladder.py --plants ... # item 2
.venv/bin/python experiments/policy/weight-transfer-gate/check2_replay.py --stride3   # item 3
.venv/bin/python experiments/policy/weight-transfer-gate/wt_grade.py 8 --round 3      # item 4
```

**The plants.** These are `gate_common` constants, applied as opt-in terms on grade.py's plant.
- **`MASS_SYM`** = `torso_kg=0.4855,pack=0.083,zoff_La=0`.
  - Torso back to the XML's 0.4545 kg, plus 31 g. grade's fit has it at x0.702. The meas-sole plant weighs 1507 g against the 1538 g kitchen-scale total without the pack (11-01). The 31 g are not located, so they go into the torso at its CoM (an assumption).
  - An 83 g pack: 1621 g with the battery minus 1538 g without. It sits where the CAD puts the 3S pack: gen_plant_v6's legs torso with `m_battery` set gives a transverse box at (-0.012, 0, 0.0585) m in the torso frame, half-sizes 18 x 52.5 x 13 mm. That the real pack sits there is an assumption.
  - The L ankle-roll zero error set to 0.
  - Total 1621 g.
- **`REFIT3`** = `MASS_SYM` + `ser_ankle=17.75,ser_hip=11.0` (item 1).

### 1. Clamp refit (`fit_clamp3.py`, `results/r3_item1_clamp_fit.md`)

**Targets**, lifting L 30 mm with the R foot clamped (the stills' side):
- torso tilt 9.4 deg;
- encoders: ankle roll 1.55, hip roll 0.55;
- beyond the encoders: ankle roll 5.6 − 1.55 = 4.05, hip roll 2.1 − 0.55 + 2.0 = 3.55.

The stills' **shank → thigh 2.0 deg is lateral slop at the knee / hip-pitch joints.** No roll term
can sit there, so it is **lumped into the hip-roll term** (the nearest roll joint above it). For
the torso's tilt this is the same; for the swing leg's sideways path it is not.

On the base plant (`grade` + `MASS_SYM`) with no term: tilt 2.66, encoders 1.46 / 0.83, cost 74.5.

| family | best | tilt | encoders ankle / hip | beyond ankle / hip | cost | 15 mm lift |
|---|---|---|---|---|---|---|
| robot | | 9.4 (9.35-9.7) | 1.5-1.6 / 0.4-0.7 | ~4.1 / ~3.5 (stills, +-1) | | the foot comes back down, ~7 deg |
| **series springs** | **ankle 17.75, hip 11.0 N m/rad** | 10.08 (R lifted: 10.99) | 2.00 / 0.78 | 3.49 / 3.13 | 1.22 | foot down (lowest pad −0.0 mm), **7.50 / 6.94 deg** |
| free play | ankle 7.0, hip 6.0 deg p-p | 10.14 (10.41) | 2.08 / 0.83 | 3.50 / 3.00 | 1.51 | foot down, 9.05 / 8.93 deg |

Residuals of the spring fit: tilt +0.7, encoders +0.45 / +0.23, beyond −0.56 / −0.42 deg. Both
families over-read the encoders. The base plant's rigid servo deflection is already 1.46 / 0.83
at today's mass, so less is left for the terms to carry. **The springs are kept.** Play is
load-independent and keeps ~9 deg at 15 mm, while the robot's lifted foot came back down at
about 7 deg. The springs give 7.5 / 6.9 deg with the foot down.

**At the 10-24 numbers** (tilt 5.0-5.2): the same springs on the no-pack plant give 9.06 deg, and
round 2's `ser_hip=14.2` on today's plant gives 5.16. As expected, one set does not fit both days:
the robot changed between them (#277).

### 2. Mass and symmetry: the scale ladder (`ladder.py`, `results/r3_item2_ladder.md`)

`hw_sessions/2026-11-10/m271/part2.py`'s method: the ladder's own poses from the crouch, no
balance loop, the L share of the summed foot normal forces over the last 0.5 s.

| pose | robot | `grade` | `grade:zoff_La=0` | `+MASS_SYM` | `+MASS_SYM`, torso CoM at the XML's | `+REFIT3` |
|---|---|---|---|---|---|---|
| total g | 1621 | 1372 | 1372 | 1621 | 1621 | 1621 |
| stand | 0.504 | 0.399 | 0.486 | 0.480 | 0.502 | 0.477 |
| toward L 31 / 62 / 86.8 mm | 0.568 / 0.695 / 0.793 | 0.527 / 0.667 / 0.781 | 0.590 / 0.669 / 0.740 | 0.596 / 0.684 / 0.787 | 0.612 / 0.698 / 0.805 | 0.594 / 0.684 / **0.952** |
| toward R 31 / 62 / 86.8 mm | 0.374 / 0.276 / 0.193 | 0.308 / 0.236 / 0.187 | 0.391 / 0.317 / 0.264 | 0.376 / 0.294 / 0.219 | 0.392 / 0.308 / 0.234 | 0.367 / 0.291 / 0.214 |
| rms over all 16 poses | | 0.045 | 0.042 | **0.017** | 0.027 | 0.044 |

- **What makes grade 0.60 / 0.40 at the stand is the L ankle-roll zero error (-2.12 deg, the 10-30
  fit), not the torso CoM offset.** Removing the offset moves the stand share 0.399 → 0.409.
  Removing the zero error moves it to 0.486. `MASS_SYM` keeps the offset, which fits the ladder
  better (rms 0.017 against 0.027).
- With `MASS_SYM` the ladder is within 0.03 of the robot everywhere.
- **The refit springs break the ladder toward L.** From 80.6 mm the R foot nearly unloads (0.952 at
  86.8 mm, robot 0.793). Toward R they change nothing. The clamp fit and the both-feet-down ladder
  are not reproduced by the same roll springs.

### 3. The stride on the refit plant (`check2_replay.py --stride3`, `results/r3_item3_stride.md`)

prec16 closed loop under the recorded 2026-11-10 command (`flight_snapshot_0035Z`, 8 seeds).
Tilt = acos(up_z). up is the record's own vector (torso z in the world, yaw stripped); the sim's
is computed the same way, and on the record it reproduces 5.6 / 11.3 / 14.4. Foot times are s
from the walk command:
- "end up" = the sole's highest pad 3 mm off the floor;
- "off" = every pad 1 mm off;
- the tilt is the sole's angle from level at that moment.

| | tilt +0.3 / +0.5 / +0.6 | up_x / up_y at +0.6 | R: end up; off (sole tilt) | L off | R lift mm | yaw +2 s | gz peak | guard-rate peak | falls |
|---|---|---|---|---|---|---|---|---|---|
| **robot** | **5.6 / 11.3 / 14.4** | **+0.22 / +0.11** | ~0.42 tilted; ~0.59 (~10-15 deg), video | ~0.59, video | unknown | **-43.6** | **-7.26** | **3.63** | 0 |
| grade as is | 4.2 / 7.7 / 8.0 | +0.13 / +0.06 | 0.18; 0.58 (8.8) | 0.93 | 20.6 | +9.0 | +1.44 | 0.68 | 0/8 |
| grade + MASS_SYM | 4.1 / 8.0 / 8.4 | +0.13 / +0.06 | 0.18; 0.58 (9.0) | 1.18 | 20.4 | +8.3 | +1.32 | 0.60 | 0/8 |
| grade + REFIT3 | 4.1 / 7.8 / 8.0 | +0.14 / +0.03 | 0.28; 0.64 (8.2) | 1.06 (5 of 8) | 12.1 | +18.6 | +1.68 | 0.88 | 0/8 |
| joints, mass + refit springs (real sprung joints) | 4.7 / 8.4 / 7.1 | +0.10 / +0.07 | ~0; 0.50 (9.3) | 0.78 | 20.6 | +3.2 | +3.89 | 1.89 | 0 |

- **The tilt was never reproduced, and the refit does not change that.** Every plant tips 4 / 8 / 8
  deg against the robot's 5.6 / 11.3 / 14.4. From +0.5 s the robot keeps tipping and the plant
  stops at ~8. Both components fall short: forward (up_x 0.13-0.14 vs 0.22) and toward the stance
  side (0.03-0.06 vs 0.11).
- **The R foot does come off tilted in the sim**, at +0.58 to +0.64 s, the sole 8-9 deg off level.
  The robot's video shows it wholly off at ~+0.59 s, tilted roughly 10-15 deg. The sim raises one
  end of the sole much earlier (+0.18 to +0.28 s against the video's ~0.42; the 3 mm threshold is
  below what a 640x480 frame resolves).
- The refit halves the R lift (20.6 → 12.1 mm). The yaw sign, the gyro-z peak (1.7 vs -7.3 rad/s)
  and the rate (0.9 vs 3.6) stay the plant's. Nothing falls.

### 4. For the planner: grade.py on the refit plant (`wt_grade.py --round 3`, `results/r3_item4_wt_grade.md`)

```bash
GRADE_PLANT_TERMS="torso_kg=0.4855,pack=0.083,zoff_La=0,ser_ankle=17.75,ser_hip=11.0" python grade.py ...   # REFIT3
GRADE_PLANT_TERMS="torso_kg=0.4855,pack=0.083,zoff_La=0" python grade.py ...                                 # MASS_SYM
```

`grade.roll_plant_opts` hands any `gate_common.apply_terms` terms to every probe. Unset, grade.py
is unchanged (test). 8 seeds, SHAPE_HZ 8:

| student, probe | plant | lift L / R cm (min) | shift_when_up L-up / R-up | tilt max deg | guard-rate peak | falls / trips |
|---|---|---|---|---|---|---|
| prec16, stride1 0.3 robot | as is | 3.12 (2.95) / 2.05 (1.92) | -0.22 / +0.16 | 9.6 | 0.92 | 0 / 1 pose |
| | MASS_SYM | 3.21 (3.13) / 2.34 (2.18) | -0.56 / +0.28 | 9.3 | 0.86 | 0 / 5 pose |
| | REFIT3 | 2.89 (2.75) / **1.41 (1.26)** | **-0.60 / +0.48** | 10.5 | 0.90 | 0 / 8 pose |
| prec16, steps5 0.3 robot | as is | 3.12 / 2.22 (2.16) | -0.39 / +0.50 | 9.6 | 1.28 | 0 / 0 |
| | REFIT3 | 2.89 / 1.73 (1.68) | -0.85 / +0.74 | 10.5 | 0.88 | 0 / 0 |
| prec37, stride1 0.3 robot | as is | 4.37 (4.23) / 2.97 (2.82) | -0.30 / +0.30 | 14.9 | 1.26 | 0 / 8 pose |
| | REFIT3 | 3.70 (3.60) / 2.23 (2.09) | -0.68 / +0.64 | 14.8 | 1.01 | 0 / 8 pose |
| prec37, steps5 0.3 robot | as is | 4.37 / 2.97 (2.82) | -0.44 / +0.60 | 14.9 | 1.08 | 0 / 0 |
| | REFIT3 | 3.74 / 2.66 (2.60) | -0.82 / +0.86 | 14.8 | 0.89 | 0 / 0 |
| march5 robot, both students, every plant | | ≤ 0.12 / ≤ 0.05 | - | ≤ 2.1 | ≤ 0.04 | 0 / 0 |

- **prec16 stands and walks the probes on the refit plant** (0 falls). No softening was needed.
- **shift_when_up moves with the mass and the zero error, not only with the springs.** `MASS_SYM`
  alone takes prec16's stride1 shift from -0.22 / +0.16 to -0.56 / +0.28. On `REFIT3`, prec37 reaches
  the bar on stride1 (feet 3.70 / 2.23 cm, shift -0.68 / +0.64), and both students reach |shift| >=
  0.6 on steps5. This does not say either student transfers weight on the robot: the same plant
  under-tips the robot's stride by 6 deg (item 3).
- The `pose` guard trips sit under the stand command after the stride, at 3.98 s; they are counted,
  not analysed. march5: neither student was trained on the lift channel. grade pins it as the
  firmware does, so neither marches.

## Round 4 (2026-11-11): free play inside the roll servos, beyond the encoder (#277)

The 2026-11-11 clamp session (`hw_sessions/2026-11-11/m277/NOTES.md`, gitignored): with the R foot
clamped toe and heel, the stance roll encoders move far less than the torso, and a fingertip
swings the torso 9.4 deg (-9.74 .. -0.35) while the ankle-roll encoder moves 2.7 and the hip roll
0.6. Tom, by hand: the play is inside the servo, between the horn and the motor / sensor. So the
encoder reads the servo shaft, and the link floats beyond it. The training flag for it is
`--roll-play-deg` (walker_env / env_mjx `roll_play_deg`, [training.md §2](../../../docs/training.md)):
the shaft is position-controlled and is every encoder reading; the link floats inside +/-play/2 of
it with no torque between them and is reached by the servo only at a band edge, through a contact
that pushes only. gate_common's `play_hip` / `play_ankle` terms put it on the gate's plants.

```bash
.venv/bin/python experiments/policy/weight-transfer-gate/fit_rollplay.py --check "" \
    --grid "ankle=3,4,4.5,5,5.5,6,7,8;hip=0,2,3,3.5,4,4.5,5,6"                       # item 1
.venv/bin/python experiments/policy/weight-transfer-gate/ladder.py --plants ...      # item 2
.venv/bin/python experiments/policy/weight-transfer-gate/rp_grade.py 8               # item 3
```

### 1. Clamp fit (`fit_rollplay.py`, `results/rp_fit_*.md`)

The robot's own pose commands (the NOTES' tick poses, one_leg.CAL zeros), streamed open loop with
minimum jerk over each pose's own duration, the R foot welded, L lifted. Four tests: the 30 mm
lift from the straight stand (B), the 15 mm lift, the 81 mm shift then the 30 mm lift (G), and a
lateral push on the torso ramped at 0.5 N/s after the 30 mm lift until the torso has swung 9.4 deg
(the H rocking; the robot's force is not measured). Cost: squares of the residuals on the 30 mm
tilt (8.55), its ankle / hip encoder moves (1.3 / 0.45), the 15 mm tilt (7.0), the shifted lift's
tilt (2.7) and the rocking swing beyond the encoders (6.1), deg.

Best on `grade` + `MASS_SYM`, no roll lash: **hip 4, ankle 5 deg peak to peak** (cost 1.92):

| test | robot | play hip 4, ankle 5 | no play |
|---|---|---|---|
| 30 mm lift: tilt, deg | 8.45 / 8.70 | 7.87 | 2.67 |
| ... stance encoders ankle / hip, deg | 1.14-1.49 / 0.35-0.44 | 1.91 / 0.83 | 1.66 / 0.63 |
| 15 mm lift: tilt; lifted foot | 7.0; barely touching, no weight | 7.85; lowest pad 0.9 mm up, 0 N | 2.66; 9.0 mm up |
| 81 mm shift, then 30 mm: tilt | 2.68 | 2.45 | 1.27 |
| ... the shifted both-feet pose before vs after the lift | 4.22 apart | 0.74 apart | 0.04 |
| hand rock: swing; encoders ankle / hip | 9.4; 2.7 / 0.6 | 9.42; 3.60 / 0.12 | 9.49; 7.81 / 0.50 |
| ... beyond the encoders | ~6.1 | 5.70 | 1.18 |
| ... push, start -> end; N per deg of swing | not measured | 0.46 -> 3.02 N; 0.19 | 0.54 -> 8.87 N; 0.99 |

- **The roll lash must be off.** On the same plant with `lash=1.25` (the recipes' DR centre) the
  best is hip 2, ankle 5 at cost 5.25: the deadzone sits on the encoder side, so the stance
  encoders move 2.6 / 1.45 deg, about twice the robot's. The `train` plant (the prec16 chain at its
  DR centre) agrees: best cost 5.43 with its 1.25 deg lash, 2.22 (hip 3, ankle 5; hip 4, ankle 5:
  2.31) with `lash=0`.
- **What the play does not fix.** The stance encoders move more than the robot's at every play
  (1.9 / 0.8 deg against 1.1-1.5 / 0.35-0.45): that is the servos' stiffness under load, which the
  play does not touch. The double-stance hysteresis (the same shifted pose 4.22 deg apart by
  direction on the robot) comes out at 0.5-0.9 deg. The 15 mm lift tilts as far as the 30 mm one
  (the play is used up either way) and its foot stays ~1 mm up; on the robot the foot touches at
  7.0 (the robot's 15 mm pose was the 10-24 roll pose, whose roll zeros differ by ~2 deg).
- **For training**: `--roll-play-deg hip=3:5,ankle=4:6` around the fit, without `--roll-lash-deg`.
  A zero-action stand at the top of that range (hip 5, ankle 6, the `train` plant, no lash) holds
  4 s (`tests/test_roll_play.py`).

### 2. The scale ladder with the play (`ladder.py`, `results/rp_ladder.md`)

`ladder.py` now reports a rung the plant falls off as FELL (it divided by zero before).

| L-foot share | robot | MASS_SYM | REFIT3 springs | play 4/5 (fit) | play 3/4 | play 5/6 |
|---|---|---|---|---|---|---|
| toward L 62.0 mm | 0.695 | 0.684 | 0.684 | 0.711 | 0.737 | 0.729 |
| toward L 74.4 mm | 0.737 | 0.732 | 0.742 | 0.860 | 0.766 | FELL |
| toward L 80.6 mm | 0.771 | 0.758 | 0.815 | FELL | 0.893 | FELL |
| toward L 86.8 mm | 0.793 | 0.787 | 0.952 | FELL | FELL | FELL |
| toward R 86.8 mm | 0.193 | 0.219 | 0.214 | FELL | 0.201 | FELL |
| rms vs robot (rungs that stood) | | 0.017 | 0.044 | 0.047 (3 fell) | 0.049 (1 fell) | 0.041 (5 fell) |

**The play breaks the linear foot-load curve.** Free play in all four rolls is a sway mode of the
both-feet-down chain: at the far rungs the body tips over the loaded foot's edge and falls (the
traced 80.6 mm rung toward L: the play offsets go to the band edges and the torso rolls past 20 deg
about 2.5 s after the shift pose is reached), and the near rungs toward R are not monotonic (0.355, 0.258, 0.303,
...). The robot, with the same servos, stands on every rung with a linear curve. The plant is
missing whatever holds the play in double stance (friction inside the play, or play that shrinks
under preload are candidates, neither measured).

### 3. grade.py with the play (`rp_grade.py`, `results/rp_grade.md`)

Students on grade.py's `robot` plant, 8 seeds; `play 4/5` = `GRADE_PLANT_TERMS=play_hip=4,play_ankle=5`
(prec57 trained with `--roll-series-k`: "grade as is" carries its spring, the play rows replace it).
Full table: `results/rp_grade.md`. Tilt = acos(up_z) from the walk command (robot, 2026-11-10:
5.6 / 11.3 / 14.4 deg at +0.3 / +0.5 / +0.6 s).

| student, probe | plant | lift L / R cm | shift_when_up L-up / R-up | tilt +0.3 / +0.5 / +0.6 | falls / trips |
|---|---|---|---|---|---|
| prec16, stride1 0.3 | as is | 3.12 / 2.05 | -0.22 / +0.16 | 3.3 / 7.0 / 6.6 | 0 / 1 of 8 |
| prec16, stride1 0.3 | play 4/5 | 2.83 / 1.92 | -0.19 / +0.16 | 2.9 / 7.0 / 6.0 | 0 / 7 of 8 |
| prec16, stride1 0.3 | MASS_SYM + play | 2.81 / 1.77 | -0.50 / +0.29 | 3.0 / 6.7 / 6.1 | 0 / 8 of 8 |
| prec52_march, march5 | as is | 0.70 / 6.38 | - / +0.62 | | 0 / 0 |
| prec52_march, march5 | play 4/5 | 0.62 / 5.28 | - / +0.50 | | 0 / 0 |
| prec52_march, stride1 0.3 | play 4/5 | 2.37 / 1.96 | -0.19 / +0.28 | 6.8 / 11.2 / 12.1 | 0 / 8 of 8 |
| prec57_march_rs, march5 | as is (its spring) | 0.18 / 0.21 | - / - | | 0 / 0 |
| prec57_march_rs, march5 | play 4/5 | 0.19 / 0.15 | - / - | | 0 / 0 |
| prec57_march_rs, stride1 0.3 | play 4/5 | 2.74 / 1.38 | -0.24 / +0.11 | 4.2 / 9.2 / 10.4 | 0 / 8 of 8 |

- **The play does not make the stride tip like the robot.** prec16's tilt at +0.5 s is 7.0 deg with
  and without it (robot 11.3). The play lowers the lifts (the R foot by 0.1-0.8 cm) and adds guard
  trips (all `pose`), with no falls.
- prec52 still marches on its R foot only (L 0.6-0.7 cm); the play takes ~1 cm off the R lift.
  prec57 does not march on this plant with or without the play.

## What is still unmeasured

1. **The R foot's height on a stride.** On 2026-11-10 the foot comes off tilted from about +0.42 s
   and wholly at about +0.59 s (video), with no height in mm: 640x480 and no scale in frame.
2. **The mass split**: a leg (or a foot and a shank) weighed against the CAD numbers. Only the
   total is measured (1621 g with the pack, 1538 g without), and the pack's place is the CAD's.
3. **Where the tilt beyond the encoders lives in a stride.** The clamp stills split it (foot →
   shank 5.6, shank → thigh 2.0, thigh → torso 2.1 deg, one side, +-1 deg). But the springs fitted to
   them do not reproduce the both-feet-down ladder or the stride's 14 deg tilt, so a static clamp
   split does not yet say how the chain behaves in motion.
4. Sole/board friction and torsional grip on the board used on 11-09 and 11-10.

## Limits

- The `train` plant is the CPU twin at the DR centre plus a stand-in for three MJX-only knobs, not
  the MJX training env.
- The bench PI is the firmware's re-implementation in `simfit_sim` (validated on 10-30, not 10-25).
  The 10-25 runs had no gyro term and no integrator cap, and the replay uses the same.
- The robot's err peaks are ~1 Hz polls, and its clearances are eyes or pixels.
- The 11-09 replay is one run. The closed-loop rows depend on the takeover from the stand2 arm
  pose (grade.py's), not the robot's 5 s of armed stand.
- `grade.py`'s weight-transfer ranking runs on the `grade` plant unless `GRADE_PLANT_TERMS` is set.
  No plant passes both checks (round 3: the stride tilt is still 6 deg short), so the "done when"
  clause about ranking on the plant that passed is not met.

## Robot measurements, 2026-11-10 (#271)

A hardware session measured what the list above asked for:
[2026-11-10-robot/README.md](2026-11-10-robot/README.md). The robot weighs 1621 g with its pack and stands
laterally even; with both feet down its foot load follows the commanded shift linearly (close to `grade`, not
to `train`); an exact repeat of the 11-09 stride shows the right foot leaving the floor tilted from +0.42 s,
with the torso tipping 14 deg before the yaw spike; and the clamp test now gives 9.4 deg of torso tilt at a
30 mm lift on both sides (10-24: 5.0 to 5.2), about 7 deg of it beyond the encoders. The plants give 2.3 deg
(`grade`) and 4.2 to 5.0 deg (`train`) on that test. Still unmeasured: the mass split and a foot height in mm.
