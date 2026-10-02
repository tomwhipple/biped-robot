# Plan B: how others deal with the high-P limit cycle (research, 2026-10-02)

Issue #73. Session 2 ([2026-10-01-plan-b-bench.md](2026-10-01-plan-b-bench.md))
found that the bench STS3215 at P 160 limit-cycles while holding a rigid
inertia. This note collects how other STS3215 users, Dynamixel humanoids and
the control literature deal with that kind of oscillation, and which of their
remedies this servo's registers allow. Quotes were checked against the source
(code via `gh api`, papers via their PDFs) unless marked otherwise.

## What our bench shows

Taken from the session-2 tables:

| condition | P 160 holding still |
|---|---|
| bottles on a string (load, no rigid inertia), 1–4 bottles | holds; probe overshoot ≤ 1 tick, status 0 (run C) |
| rigid clamp, lever **hanging** (inertia, no gravity load) | holds still (range 0) |
| rigid clamp, lever **level** (inertia and gravity load) | **limit cycle**: range 13–18 ticks, moving on 114–152 of 152 samples, at D 160 and D 32 alike (run D) |

- P ≤ 128 with D 32 holds still level.
- Hanging 2° moves ring at P 96–128 with the clamp in plane.
- Slow 90° sweeps shake at P ≥ 96, and more when lowering.
- So the limit cycle needs all three: high P, a rigid inertia, and a gravity
  load on the gear train. D does not remove it.

## What others do

**STS3215 users run P at or below the factory 32, never above it.**
- LeRobot (SO-100/SO-101 arms, LeKiwi) writes P 16, I 0, D 32 at connect. The
  comment in
  [`lekiwi.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/lekiwi/lekiwi.py)
  reads "Set P_Coefficient to lower value to avoid shakiness (Default is 32)".
  [`config_so_follower.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/config_so_follower.py)
  makes the same values (16 / 0 / 32) config fields.
- Open Duck Mini, a legged RL robot on STS3215, writes P 32, I 0, D 0 and
  acceleration 0 to every joint
  ([`configure_all_motors.py`](https://github.com/apirrone/Open_Duck_Mini_Runtime/blob/v2/scripts/configure_all_motors.py)).
  Instead of raising gains, it identifies each servo's dynamics (damping, kp,
  friction loss, armature) with Rhoban's [BAM](https://github.com/Rhoban/bam)
  and puts them in the sim, so the policy learns around them
  ([sim2real notes](https://github.com/apirrone/Open_Duck_Mini)).
- LeRobot's "shaking" issues (#652, #1714) are control-loop latency (camera
  fps starving the loop), not servo gains. That is a different failure.

**Humanoids on Dynamixels: low P plus feed-forward.**
- Schwarz & Behnke, "Compliant Robot Behavior using Servo Actuator Models"
  (RoboCup 2013, NimbRo-OP, Dynamixel MX,
  [PDF](https://www.ais.uni-bonn.de/papers/RC13_Schwarz.pdf)):
  - The problem: "high-gain position control often results in undesirable
    behavior like stiffness and oscillations (limit cycles)."
  - The fix: a fitted motor-and-friction model predicts the torque each joint
    needs, sent as feed-forward on top of a low P (kp 0.35 against the
    high-gain 1.0). They "did not incorporate feedback mechanisms in order to
    avoid oscillations, which could be caused by latencies."
  - The result: knee energy over 40 steps fell from 189 J to 140 J, because
    "current spikes are avoided by predicting higher loads."
- Tan et al., "Sim-to-Real: Learning Agile Locomotion for Quadruped Robots"
  (RSS 2018, [PDF](https://www.roboticsproceedings.org/rss14/p10.pdf)): "if
  large gains are used, the motors can remain stable in simulation but
  oscillate in reality". Their answer is an identified actuator model.
- ToddlerBot (Shi et al., 2025, [arXiv 2502.00893](https://arxiv.org/abs/2502.00893))
  identified its Dynamixels' kp, damping, friction loss and armature on a bench.
  It reports a gearbox "passive-active ratio" of 3: the gearbox is much harder
  to back-drive than to drive. This is a sub-agent's reading of the paper and
  was not re-checked here.
- ROBOTIS's own tuning guide (archived,
  [Wayback](https://www.robotis.us/pid-tuning-for-dynamixel)): for oscillation
  near the target, reduce P and/or I; for harsh starts, reduce Profile
  Acceleration. The old AX-12's compliance margin and slope (a zero-torque band
  and a reduced-gain band around the goal) served the same end
  ([AX-12 manual](https://www.generationrobots.com/media/Dynamixel-AX-12-user-manual.pdf)).
  Neither source was re-checked here.

**Control theory: backlash inside the loop.**
- The STS3215 reads its position at the output shaft, so the gear train's
  backlash and compliance sit *inside* the position loop.
- With feedback on the load side and backlash in between, a limit cycle appears
  above a gain threshold. The textbook fix is a dual loop: position from the
  load encoder, with damping from the *motor's* velocity, which the backlash
  does not corrupt. Sources:
  - Nordin & Gutman, "Controlling mechanical systems with backlash — a
    survey", *Automatica* 2002
    ([link](https://www.researchgate.net/publication/222691222_Controlling_mechanical_systems_with_backlash_-_A_survey));
  - "Two feedback loops are better than one", Machine Design
    ([link](http://machinedesign.com/archive/two-feedback-loops-are-better-one)).
- Raising D does not cure it. D taken on the backlash-corrupted, quantized
  output signal mostly adds tick noise
  ([velocity estimation from low-resolution encoders](https://www.researchgate.net/publication/270775373_Velocity_Estimation_of_Motion_Systems_Based_on_Low-Resolution_Encoders)).
- Friction "hunting" is the other classic cause, but it needs integral action
  ([arXiv 2006.08977](https://arxiv.org/pdf/2006.08977)). Our I is 0, so it is
  not the main mechanism here.

## Reading (inference, not measured)

The bench pattern fits backlash inside the loop, together with a gearbox that is
harder to back-drive than to drive:
- **Hanging:** with no gravity load, the inertia sits in the play without
  forcing the gears through it.
- **Level:** the motor holds the load against the mesh, so a correction in the
  load's direction back-drives the train. That direction has different friction.
- **Lowering sweeps** are the back-driven direction, and they shake most.

This would also explain why D 160 and D 32 behave alike. It is a hypothesis
that matches the sources. Nothing here measured it.

## Remedies this servo allows

The STS3215 has no motor-side encoder and closed firmware, so the dual loop is
out. In order of evidence:

1. **Stay at P ≤ 128, D 32.** This was quiet at hold on our bench, with 3.5×
   the P 32 stiffness (#73's conditional pass). Field practice is lower still
   (LeRobot 16, Open Duck 32).
2. **Put the missing stiffness in the command, not the gain.** Feed-forward:
   add the predicted sag (load / k(P)) to the goal, as NimbRo does with
   torque. Our policy outputs goal positions, so it can learn this, provided
   the plant carries the servo as identified: k at the chosen P, friction,
   back-drive asymmetry. That is the Open Duck / BAM / ToddlerBot route. It
   needs the bench numbers we are already taking, at one P.
3. **Smooth the goal stream** (register 41, or ramps from the host): a cure
   for sweep shake (ROBOTIS, S-curve practice). It does not touch the
   hold-still cycle.
4. **Dead zone 26/27 at 2–3 ticks** (0.18–0.26°): the AX-12 compliance-margin
   idea, and the backlash literature's deadband. No STS3215 user is on record
   doing it, and it costs exactly the static accuracy Plan B is after.
5. **Gain scheduling** (high P only in stance): only practical if a write to
   register 21 takes effect while the lock (55) is 1, without an EEPROM commit.
   Feetech's lock flag is described as stopping writes "from being saved after
   power off", which suggests yes. Not tested.
6. **The velocity-loop registers 37/39** exist, but no project found tunes them.
   Their effect is unknown.

## Bench questions this raises

For the Mira session, which owns the bench and its scripts:

1. **#73's leg-like inertia at P 160, level:** two M10 bolts × 12 washers at r 100 + 70 on the
   v3 fork (≈ 0.0023 kg·m², estimated masses). Does the limit cycle survive at
   the inertia #73 specifies?
2. **P 160 with dead zone 2 and 3, clamp level:** does the cycle stop, and what
   sag does the wider band add?
3. **Register 21 written with the lock at 1:** read it back, re-probe the
   stiffness, power-cycle and read again. This settles whether gain
   scheduling is possible.
4. **Back-drive asymmetry:** the probes from above and below at each load may
   already show it. If they do, fit it into the plant with the stiffness.
