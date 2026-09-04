# Roadmap: from "falls in 5 s" to joystick mode

*2026-09-04. Goal: drive bimo about from the GUI ("joystick mode": stick → vx, vy, wz;
slider → crouch) with confidence, meaning a fall is rare, announced by the guards, and
recoverable without hands. Every stage has a sim gate and a hardware gate; nothing
advances on a feeling. Status column is as of this writing.*

## What two days of arms established

Twelve spotted arms (2026-09-03/04), two students, corrected gyro, yaw-stripped
up-vector, three gyro gains, two surfaces, from home and from the policy's own stance:

- The fall loop closes through the IMU **rate** term (IMU frozen → stands 8 s; gyro at
  0.5× → stands), but the gyro itself is right: sign, scale, frame and yaw all match the
  sim on identical bench leans.
- The **learned standing posture** is the fault. Fed the robot's exact home observation,
  the student's first action adducts both hips ~7° (the sim's observation gives the same
  action). Within 2 s the hip pitches scissor 15°/13°, the yaws splay 13°, the right leg
  walks into the left and it rolls over in 3–7 s. In the sim twin the identical policy
  holds all of this under 6°: the sim floor lets the legs press where the real feet slide.
- The posture is partly **trained on purpose** (knee bias +0.10 rad for torque-off sag,
  CoM-over-midfoot with a model whose zero-pose CoM sits 26–28 mm aft of midfoot). The
  real robot stands released at zero, so the model's CoM is wrong.
- Surface changes the timing only (pad 7 s, bare wood 4 s at the same gain).

Direction (Tom): **the zero position is the stop/rest position.** `loco_v29zero` trains
it (`--w-stand-zero`, knee bias off). Everything below builds on a policy that rests at
home.

## Stage 0 — Plant truth (bench, no policy, no risk)

Make the sim plant match the robot on the things the stand depends on.

| Item | Test | Pass | Status |
|---|---|---|---|
| Zero-pose CoM | Released robot on the pad: tilt the desk/plate until it topples fore and aft; CoM offset from the two angles. Or: torque-on `pose` ankle sweep, note the ankle angle where load reverses. | Model CoM within 5 mm of measured; `w_stand_com` stops asking for a lean at q=0 | **open** (model says 26–28 mm aft; robot stands at zero) |
| Servo small-step response | `move` 3°/5°/10° steps on hip pitch + `trace`; fit the act-lag cascade | Sim `--act-lag` band brackets the measured small-step response (the 2 Hz figure came from a large-motion walk) | open |
| Foot–ground friction | Spring-scale drag of the released robot on pad and wood; tilt-slip angle | Sim friction DR band covers both surfaces (pad ≈ high, wood ≈ low) | open |
| Joint play / zero | Tilt-hysteresis method, rezero after any mechanical work | ≤ 1.5° per pitch-chain joint; zeros within 3 ticks | done 09-03 (L hip roll 2–3° remains) |
| IMU | `tools/imu_scale_check.py`, `tools/gyro_sign_check.py` vs the sim leans | gyro integral within 0.5°, up within 0.02 on ankle/hip/knee leans | **done** 09-03/04 |
| Bench motion | smooth streamer, no ringing | knee trace creep 0, ring peak < 0.05 rad/s at rest | done 09-03 |

Gate 0: the sim twin (`sim/sil_twin.py --nominal`) reproduces the released stand and the
bench leans; the mass/friction/actuator numbers are in the plant XML with a dated note.

## Stage 1 — Stand: rest is home

| | Sim gate | Hardware gate |
|---|---|---|
| Policy | `loco_v29zero_s128r24` (or successor): stand_10s 8/8 and stand_off ≥ 6/8 on the lag referee; first action from the home obs ≤ 0.05 on every joint (offline check, `sim/sil/harness.silw_forward`) | 5 arms × 30 s at **full gyro gain**, on the pad: no guard, hip-pitch swing < 0.05 rad, hip-roll and yaw drift < 3°, torso within 3° of its starting attitude |
| Then | same on bare wood | 3 arms × 30 s on wood |
| Then | push recovery scenarios in the referee | Tom nudges the torso ±2 cm at the pelvis, 5 pushes, recovers each time without a step |

Tools: `run_arm3.sh` (probe + camera + beacon CSV), `obsfreeze` (diagnostic only; must be
`none gain=1.0` for the gate), `$T/pose_stance.py` to check the policy's tick-1 request,
twin obs vs robot obs comparison for the first second.

Gate 1: all three hardware rows pass on consecutive attempts, video kept. Only then does
the GUI arm button become something other than a test.

## Stage 2 — Posture commands (no stepping)

Crouch slider and weight shift, i.e. the ext_cmd channels the GUI already has.

| | Sim gate | Hardware gate |
|---|---|---|
| Crouch | squat_reps ≥ 6/8, weight_shift ≥ 6/8 on the lag referee | 3 crouch-and-return cycles from the GUI slider, 3 s each way, no guard, torso pitch within 5° throughout, knees move in unison with hips/ankles (trace) |
| Hold | — | 60 s at half crouch, swing < 0.05 rad |

Known: the crouch mix regressed walking in v28crouch (14/144 vs 41/144); attribute with
the gyro-DR-only run (queue 45/46) before folding crouch back in.

Gate 2: three clean cycles, then a 60 s hold, filmed.

## Stage 3 — Step in place

| | Sim gate | Hardware gate |
|---|---|---|
| March | march scenario ≥ 6/8, feet clear 2 cm | 10 s march on the pad from the GUI lift command, both feet visibly clear (camera at foot level), no fall, 3/3 |
| Stop | stops within 1 s of lift → 0 | stops to a quiet stand within 1 s, no guard |

Gate 3: 3/3 on the pad, 2/3 on wood.

## Stage 4 — Walk and turn at low speed

| | Sim gate | Hardware gate |
|---|---|---|
| Line | line_1m ≥ 6/8, backward_1m ≥ 6/8 (lag referee) | vx = 0.15 for 1 m on the pad, 3/3 no fall; lateral drift < 20 cm; stop on command within 1 s |
| Turn | turn_180 ≥ 5/8 | 90° in place, 3/3, heading error < 20° |
| Reversal | reversal ≥ 5/8 | forward → backward without a stop, 2/3 |

Gate 4: the line and the turn both pass on the pad; one of them on wood.

## Stage 5 — Joystick mode

Free driving from the GUI with the safety envelope on:

- On-board: link watchdog (exists), fall latch → torque release (exists), battery guard
  (exists), tilt guard on the commander (`osc_probe` style) promoted into the GUI, speed
  caps (vx ≤ 0.2, wz ≤ 0.5 to start), **RESET SERVOS while armed** as the recovery
  gesture (releasing torque does nothing; 09-03 lesson).
- Gate 5: 5 minutes of free driving on the pad with at most one assisted catch, twice;
  then 5 minutes on wood. Filmed, beacon CSV kept.

## Cross-cutting rules (learned the hard way)

- **One variable per run.** The v28crouch run changed the gyro DR and the crouch mix
  together and we cannot say which cost the walking.
- **Every arm is recorded**: camera, beacon CSV, obs capture when it matters. The per-second
  swing table (`hw_sessions/<date>/*_beacons.csv`) is the comparison unit.
- **Twin first.** Before blaming a sensor, run the same policy in `sil_twin.py` from the
  same pose and compare the first second of joint means and the gyro RMS.
- **Nightly queue**: distinct numeric prefixes (locale sort put `39b` before `39-`), the
  recipe belongs to `sim/mjx/train_mjx.py`, distill/referee as a separate CMD job behind
  the training job, runs collected into `sim/runs` before `export_policy_weights`.
- **Spotter protocol**: Tom in position, "go" before every arm, torque released between
  runs, RESET SERVOS after any fall, `imu reinit` + `imu scan` after any flash.
- **Sim scores are gates, not proof.** v27tilt_b stood 8/8 under lag in sim and never
  stood on the robot.

## Immediate next steps

1. `loco_v29zero` (training since 11:43 09-04) → distill → lag referee → offline tick-1
   check → flash → Stage 1 hardware gate with a spotter.
2. Stage 0 CoM measurement on the bench (30 min, no policy) and the model correction; the
   servo small-step response the same session.
3. Gyro-DR-only attribution (queue 45/46) to settle whether the DR costs walking.
4. Foot friction: measure both surfaces; decide traction pads vs. friction DR vs. both.
