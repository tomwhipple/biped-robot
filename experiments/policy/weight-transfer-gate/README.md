# Weight-transfer gate: does the walking plant reproduce the robot? (2026-11-10, #259)

Two checks of the walking plant (`sim/bimo_biped_v6legs_meas_sole.xml`) against what the robot has
already shown about weight transfer, a first attribution pass, and a second (Round 2: the pipeline's
yaw sign, the one-fault hypothesis, compliance beyond the encoders, grade.py's roll opt-ins). CPU only; no plant XML, training
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

## What is still unmeasured

1. **The R foot's real height on 11-09**, or on any stride: a side camera at floor level with a
   scale, or a contact switch / FSR under each sole. Without it, the check-2 lift column is
   "unknown" for the robot.
2. **The mass split**: a leg (or a foot and a shank) weighed against the CAD numbers. The check-1
   attribution hangs on it.
3. **Where the stance-side tilt lives**: an encoder-independent angle at the ankle and hip rolls
   under load (an IMU on the shank, or a dial gauge). It would tell play from compliance, and
   whether the 11-09 L sole was on an edge.
4. Sole/board friction and torsional grip on the 11-09 surface. The yaw-gap work measured the TPU
   sole (mu ~0.45, ~0.1 N m) on a different floor.

## Limits

- The `train` plant is the CPU twin at the DR centre plus a stand-in for three MJX-only knobs, not
  the MJX training env.
- The bench PI is the firmware's re-implementation in `simfit_sim` (validated on 10-30, not 10-25).
  The 10-25 runs had no gyro term and no integrator cap, and the replay uses the same.
- The robot's err peaks are ~1 Hz polls, and its clearances are eyes or pixels.
- The 11-09 replay is one run. The closed-loop rows depend on the takeover from the stand2 arm
  pose (grade.py's), not the robot's 5 s of armed stand.
- `grade.py`'s weight-transfer ranking still runs on the `grade` plant. Neither plant passed, so
  the "done when" clause about ranking on the plant that passed is not met.
