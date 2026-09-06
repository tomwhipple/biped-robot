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
| Zero-pose CoM | Released robot on the pad: tilt the desk/plate until it topples fore and aft; CoM offset from the two angles. Or: torque-on `pose` ankle sweep, note the ankle angle where load reverses. | Model CoM within 5 mm of measured; `w_stand_com` stops asking for a lean at q=0 | **open**. Corrected 09-05: the model's zero-pose CoM is 11.3 mm aft of the L_sole/R_sole midpoint (26–28 mm was the v21 policies' *stance*, not q=0). The kernel (σ 20 mm, w 1.0) pays 0.27/step for a 3.3° forward torso pitch that centres it; the real robot stands released at zero, which only bounds the real offset at ≲16 mm (backdrive friction). Measurement still needed. |
| Servo small-step response | `move` 3°/5°/10° steps on hip pitch + `trace`; fit the act-lag cascade | Sim `--act-lag` band brackets the measured small-step response (the 2 Hz figure came from a large-motion walk) | **measured 09-05** (left foot clamped, 16 traces, 3 loads): pure dead time ≈ 85 ms + 30 ms lag (5 Hz), amplitude- and load-independent, no rate limit below 650 steps/s. The sim has 0–8 ms latency and a 2–12 Hz 3-stage lag; it has NO dead time, so it never sees the robot's "full gain, 80° late at 2 Hz" combination. Fix: per-episode action dead-time DR (0–5 ticks) in env_mjx; hw_sessions/2026-09-05/servo_step_notes.md |
| Foot–ground friction | Spring-scale drag of the released robot on pad and wood; tilt-slip angle | Sim friction DR band covers both surfaces (pad ≈ high, wood ≈ low) | open |
| Joint play / zero | Tilt-hysteresis method, rezero after any mechanical work | ≤ 1.5° per pitch-chain joint; zeros within 3 ticks | done 09-03 (L hip roll 2–3° remains) |
| IMU | `tools/imu_scale_check.py`, `tools/gyro_sign_check.py` vs the sim leans | gyro integral within 0.5°, up within 0.02 on ankle/hip/knee leans | **done** 09-03/04 |
| Bench motion | smooth streamer, no ringing | knee trace creep 0, ring peak < 0.05 rad/s at rest | done 09-03 |

Gate 0: the sim twin (`sim/sil_twin.py --nominal`) reproduces the released stand and the
bench leans; the mass/friction/actuator numbers are in the plant XML with a dated note.

## Stage 1 — Stand: rest is home

| | Sim gate | Hardware gate |
|---|---|---|
| Policy | `loco_v29zero_s128r24` (or successor): stand_10s 8/8 and stand_off ≥ 6/8 on the lag referee; first action from the home obs ≤ 0.05 on every joint (offline check, `sim/sil/harness.silw_forward`); **and** the twin's rest pose (`sil_twin.py`, last 3 s of a 6 s stand) within 2° of home on every joint, torso within 1° | 5 arms × 30 s at **full gyro gain**, on the pad: no guard, hip-pitch swing < 0.05 rad, hip-roll and yaw drift < 3°, torso within 3° of its starting attitude |
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

## Stage 1 log

- **09-05, `loco_v29zero_s128r24`: sim gate FAILED on the tick-1/rest check.** Referee:
  stand_10s 8/8, stand_off 7/8, walking 21/144 (v27tilt_b student 41; gyro-DR-only
  v28gyro student 36). Tick-1 at the home obs: max|a| 0.175 (ankles −7°, R knee +12°);
  the teacher gives the same (0.208), so it is the objective, not the distill. Twin rest
  pose after 6 s: L ankle −8.5°, hip pitch +3.9/−3.8, yaw +1.9/−1.6, hip roll −1.1/+1.1,
  torso pitched 3.3° forward (up_x 0.057). The adduction is gone (was ∓7°); what remains
  is the forward lean, and its cause is arithmetic: at q=0 the model's CoM is 11.3 mm aft
  of midfoot, `w_stand_com` (σ 20 mm, w 1.0) gains 0.27/step by pitching the torso until
  the CoM is centred (−0.9 mm at the twin's pose), and `w_stand_zero 2.0` charges only
  0.02/step for the actions that do it. Fix queued as `loco_v30home` (held, needs the GPU
  slot): the v29zero recipe with `--w-stand-com 0 --w-stand-zero 10`, warm from v29zero,
  30M steps. Firmware with the v29zero student is built (host suite green, 225 checks) but
  NOT flashed; the robot keeps v27tilt_b.

- **09-05, hardware arms on `loco_v29zero_s128r24`** (full gain, three arms, one variable each): wood 30 s
  stood but shuffled off the table; pad fell backward at 23 s; pad + `shape 4` stood 30 s, wandered, 20° burst.
  Hip roll within 1.5° in all three: the roll-over mode is gone. Rest is the trained scissor (L hip pitch
  +5..+8°, R −5..−9°, L ankle −8°, torso 3–5° forward) with 8–14° pitch swings per 3 s that never decay;
  gyro RMS 0.42–0.49 roll / 0.72–0.75 pitch in every arm; twin at 0.005. Surface and shaper change nothing:
  the swing is the loop, the scissor is the amplifier. Notes: hw_sessions/2026-09-05/arm_v29_*_notes.md.

- **09-05 night, `loco_v30home_s128r24`** (stand-CoM off, stand-zero 10, servo dead time 0–5 ticks; 30M
  warm from v29zero): referee 19/144, stand 8/8 (drift 1.0 cm), stand_off 7/8, falls 15 % (lowest yet).
  Gate: tick-1 max|a| 0.136 FAIL; twin rest torso −1.1° (the lean is gone) but knees +5.3/−8.9°, right ankle
  −7.4° — a new knee split, same in the teacher. Cause: the squared-action term is toothless in `full` action
  units (10° knee = 0.11 action, 0.03/step at w 10) and the base reward pays ~0.5/step more for the split than
  for home (term not yet identified). Fix: `--w-stand-home` (L1 on the served target in radians, stand-gated;
  1.17/step at this split), queued as `loco_v31home` (52/53) for the 09-06 night slot. v30home firmware built,
  not flashed; the hardware question it can still answer tomorrow is whether the dead-time DR damps the swing.

## Immediate next steps

1. Morning 09-06: spotted arm on the v30home student (binary built) — does the servo dead-time DR
   damp the pitch swing at full gain? Pad, 30 s, same script. Then `loco_v31home` (queue 52/53,
   09-06 night) → offline gate (tick-1 + twin rest within 2°) → flash → Stage 1 gate.
   Open question to settle in sim: which reward term pays for a knee split at stand.
2. Action dead-time DR in env_mjx (`--act-delay-max`, ticks) from the 09-05 servo measurement,
   into the next training run. Stage 0 CoM measurement still open (30 min, no policy).
3. Gyro-DR-only attribution (queue 45/46) to settle whether the DR costs walking.
4. Foot friction: measure both surfaces; decide traction pads vs. friction DR vs. both.
