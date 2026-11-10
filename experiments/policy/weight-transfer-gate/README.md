# Weight-transfer gate: does the walking plant reproduce the robot? (2026-11-10, #259)

Two checks of the walking plant (`sim/bimo_biped_v6legs_meas_sole.xml`) against what the robot has
already shown about weight transfer, and a first attribution pass. CPU only; no plant XML, training
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
| `grade` | what `grade.py`'s `robot` probes run (the `stride1 0.3 robot` probe that lifts prec16's R foot 2.1 cm): `simh.build("fit")` + `JF_PLANT` + `ROBOT_SPEED_PLANT`, `TypedServo(ROBOT_SPEED_SERVO)` in front |
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
