# Plan B: how others deal with the high-P limit cycle (research, 2026-10-02)

Issue #73. Sessions 2 and 3 ([2026-10-01-plan-b-bench.md](2026-10-01-plan-b-bench.md),
[2026-10-02-plan-b-bench.md](2026-10-02-plan-b-bench.md)) found that the bench
STS3215 limit-cycles at P 160 with a rigid inertia, and that its sweeps get
rough from P 96–128. This note collects how other STS3215 users, Dynamixel
humanoids and the control literature deal with that kind of oscillation, and
which of their remedies this servo's registers allow. Quotes were checked
against the source (code via `gh api`, papers via their PDFs) unless marked
otherwise.

## What our bench shows

| rig, load | P 160 / D 32 holding still | source |
|---|---|---|
| v2, bottles on a string (load, no rigid inertia) | holds; probe overshoot ≤ 1 tick | session 2, run C |
| v2, C-clamp, hanging | holds (range 0) | session 2, run D |
| v2, C-clamp, level | **limit cycle**: range 13–18 ticks, moving on 114–152 of 152 samples; D 160 the same | session 2, run D |
| v3, washer weights, level | holds (range 0) | session 3 |
| v3, washer weights, hanging | **limit cycle**: range 14, moving on 145 of 152, load ±284 | session 3 |
| v3, washer weights, hanging, **D 0** | holds (range 0), but a 2° move starts an oscillation that does not die out | session 3 |

- **At P ≤ 128 with D 32:** the servo holds still on both rigs, level and
  hanging, and its 2° moves settle on v3.
- **Sweeps at P 128, by D:** lower D is smoother. D 0 has 4–6× fewer backward
  samples than D 32, but only about 0.8× the jitter and 0.6–0.85× the
  accelerometer's shaking. D 128 raises the jitter about 1.2–1.8×.
- **Sweeps at P 128, by command:** smoothing the command does not help. The
  gentler acceleration register was the same, and host-streamed minimum-jerk
  targets cut only the backward count (session 3).
- **Hanging 2° moves at P 128 / D 0:** one of four kept moving (residual 6).

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

The pattern matches backlash inside a load-side-feedback loop:
- **Where it cycles:** on v3 the cycle appears hanging, where nothing preloads
  the gears and the play floats. Level, gravity holds the mesh to one side and
  the servo is quiet.
- **On v2 it cycled level instead.** Tom saw that bracket twist under load
  (2026-10-01: "a fair amount of twisting on the vertical plate", quoted in
  `plan_b_rig_v3.py`; it was observed, not measured). That puts more
  compliance inside the loop. The two rigs cannot
  separate those effects.
- **D:** the servo's D can only act on the output encoder's signal. That signal
  is quantized to 0.088° and the backlash corrupts it, which is the wrong place
  to take damping from. More D makes the sweeps rougher, and D 0 is the
  smoothest. That is the literature's argument for motor-side damping, and Open
  Duck Mini runs D 0.
- **Without D, P 160 has nothing to stop a disturbance ringing on.** So D 0
  only moves P 160's problem from rest to small moves.

## Remedies this servo allows

The STS3215 has no motor-side encoder and closed firmware, so the dual loop is
out. In order of evidence:

1. **Stay at P ≤ 128.** It holds still on both rigs (about 3.5× the P 32
   stiffness, #73's conditional pass). Every project found runs at or below the
   factory 32 (LeRobot 16, Open Duck 32).
2. **Take D down from 32.** It is bench-confirmed for sweeps: at P 128, D 0 has
   4–6× fewer backward samples than D 32 and about 0.6–0.85× its shaking. Open Duck runs D 0. The cost: at P 128 / D 0 one of
   four hanging 2° moves did not settle. D 8–16 is the untested middle.
3. **Put the missing stiffness in the command, not the gain.** Feed-forward:
   add the predicted sag (load / k(P)) to the goal, as NimbRo does with torque.
   Our policy outputs goal positions, so it can learn this, provided the plant
   carries the servo as identified: k at the chosen P, friction, D. That is the
   Open Duck / BAM / ToddlerBot route, and it uses the bench numbers we already
   take, at one P.
4. **Dead zone 26/27 at 2–3 ticks** (0.18–0.26°): the AX-12 compliance-margin
   idea, and the backlash literature's deadband. No STS3215 user is on record
   doing it, and it costs exactly the static accuracy Plan B is after.
5. **Gain scheduling** (high P only in stance, where the load preloads the
   gears): only practical if a write to register 21 takes effect while the lock
   (55) is 1, without an EEPROM commit. Feetech's lock flag is described as
   stopping writes "from being saved after power off", which suggests yes. Not
   tested.
6. **The velocity-loop registers 37/39** exist, but no project found tunes them.
   Their effect is unknown.
7. **Smoothing the commands** (register 41, minimum-jerk streaming): the
   literature and ROBOTIS advise it, but session 3 measured no change in sweep
   jitter at P 128.

## Bench questions this raises

For the Mira session, which owns the bench and its scripts:

1. **Hold and hanging 2° moves at P 128 / D 8 and D 16:** smoother sweeps than
   D 32, but do small moves still settle?
2. **P 160 with dead zone 2 and 3, hanging:** does the cycle stop, and what sag
   does the wider band add?
3. **Register 21 written with the lock at 1:** read it back, re-probe the
   stiffness, power-cycle and read again. This settles whether gain scheduling
   is possible.
4. **Record the weights:** their radii and piece counts (session 3 used 16
   washers per bolt). Without them, the inertia relative to #73's leg-like
   0.0025 kg·m² is unknown.
