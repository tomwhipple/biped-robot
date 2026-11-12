# Roll balance through the lift (#297 step 2): Stage A, design in simulation

The host stepper (`experiments/stepper/`) keeps the pelvis level by trimming the stance hip roll against
the measured tilt, at 5-7 corrections a second. This experiment designs the same balance as a firmware
loop on the 50 Hz tick, and as a base layer that a stepper or a learned policy can command. Everything
here is CPU MuJoCo, on the per-side play plant fitted to the stepper's own runs
(`experiments/policy/weight-transfer-gate/README.md`, Round 5).

```bash
.venv/bin/python experiments/roll-balance/evaluate.py --laws off,stepper,oldPI-through,trim,t25-10 \
    --plants "fit;lo;hi;hard" --speeds 1,2 --delays 0,60 --gains 1,2 --out experiments/roll-balance/results/x
```

## The testbed (`testbed.py`, `synth.py`, `laws.py`, `evaluate.py`)

- **Trajectories.** Each run's own `pose` commands come from the flight record: the joint ticks, the
  arrival time, and `s<ms>`. They are streamed as the firmware does: a min-jerk move from the held goal,
  20 ms per goal, starting 70 ms after the command. A new pose interrupts a move in flight, and the
  first move starts from the read-back joints.
  - Against the record's own written goals the rebuilt stream is off by a median 0.03-0.09 deg (p99
    0.8-1.5, max 2.9) on 5 runs: a lift, a march and a walk of 2026-11-11 18:19-18:40, a carpet lift of
    2026-11-12 and a lift of 2026-11-11 10:40.
  - `trims="none"` removes the stepper's own closed-loop trims from every target: the stance hip roll
    `hike` and the swing sole's `sole_r` / `sole_p`, from the matching jsonl row. All 20 + 162 + 209 + 29
    + 14 rows matched a command.
  - `synth.py` builds the stepper's phases with its own IK at any step period and direction (shift,
    raise, swing, lower).
- **Loop.** Each 20 ms tick, the law reads the torso roll and pitch and their body rates, one tick old
  plus an optional 0-60 ms extra delay. Its correction rides on the streamed goals into grade.py's
  servo chain: STS3215 dead 60 ms / tau 33 ms, STS3250 2nd order. Plants:

  | plant | roll play deg p-p (L hip, R hip, L ankle, R ankle) | floor |
  |---|---|---|
  | `fit` | 6, 8, 3, 0 (the Round-5 fit) | grade's floor_tc 0.0334 |
  | `lo` | 4, 6, 2, 0 | grade's floor_tc 0.0334 |
  | `hi` | 10, 10, 3, 2 | grade's floor_tc 0.0334 |
  | `hard` | 6, 8, 3, 0 | floor_tc 0.01 |

- **Metrics per single support** (raise to foot-down):
  - lift: the swing sole's lowest pad more than 5 mm up for at least half of the hold;
  - peak |roll − roll at the raise|;
  - peak pitch;
  - the CoM's margin to the stance sole's outer edge.
- **Metrics per run:** a fall (tilt > 35 deg) and its direction (OUT = over the stance foot).

## Floors: how well they match the logs

- **Hard floor (the board).** This is the plant the stepper replay was fitted on (held-out walk roll RMS
  2.5 deg, Round 5). Replaying the stepper's own commands with its trims, the sim lifts 114 of 120
  single supports with no fall, as the robot did. With the trims removed it lifts none (0/120): the body
  leans 5-7 deg toward the foot and the sole stays within 3 mm of the board. That is the robot's "the
  lift follows the foot down", smaller than on carpet.
- **Carpet (soft floor): not reproduced.** I fitted the floor's contact softness to the seven carpet lifts
  and the carpet shift sweep of 2026-11-12. The grid covered the time constant 0.033-0.16 s, impedance
  width 1-20 mm and d0 0.02-0.9, scored by roll RMS while upright.
  - The best variants reach 4.5-5.0 deg roll RMS. They still leave the robot leaning 7-12 deg further
    toward the lifted foot on left stance than the sim does.
  - A soft contact under the sole's eight pads sinks the foot evenly. It does not let the rigid sole
    roll on its inner edge in the pile, which is what the robot and the Sony frames show.
  - Reproducing that needs a sole-roll compliance (a passive roll joint or a dense compliant pad array
    under each foot). That is not built: carpet is a later phase (decision 3).

## The two known failures

- **(a) No balance: the lift follows the foot down.** Reproduced on the hard floor with the trims
  removed: 0 lifts in 120 single supports (synthetic gaits: 35 %), peak lean median 6.7 deg. Not
  reproduced in size on carpet (see above).
- **(b) The old firmware PI run through the lift falls outward.** Reproduced: `roll_pi` engaged at the
  raise with its reference there (kp 0.15, ki 1, ankle +1 / hip −1, clamp 10, I share 3) falls in 23 of
  24 hard-floor cases, 15 of them OUT, with 0 lifts. As built (armed, starting at the end of the raise or
  past 4 deg) it falls in 20 of 24 (12 OUT).
  - Why, from the code and the 10-30 sim note: at foot-off the roll play preloaded in double support
    releases, and the torso tilts 2-4 deg toward the lifted foot with no CoM movement behind it.
  - The PI's default mix (ankle +u, hip −u) answers with a pelvis TRANSLATION outboard, mostly integral.
    That drives the CoM 25-30 mm outboard, and the robot falls over the stance foot's outer edge.
  - So the error signal is a tilt, and the old controller answered it with a CoM shift.

## Candidate laws

Here e = roll − roll at the raise (rad; + = leaning to the robot's right). "Stance hip roll += x" is in
the sim joint frame, the same sign for either side: +x rolls the pelvis level, which is the stepper's own
`hike` trim.

| law | formula |
|---|---|
| hipP | stance hip roll += clip(kp·e, 15 deg) |
| **trim** | stance hip roll += clip(r(t)·T[stance] + kp·e, 15 deg) |
| trim, ankle variant (`…a`) | as trim, plus stance ankle roll += ka·e |
| outward catch | as trim, plus one-sided pelvis translation kt·rate while rolling outward (tested, rejected) |

The **trim** law in detail:
- T[stance] += ki·e·dt while the loop is closed (anti-windup at the 15 deg clamp; |T| ≤ 14 deg).
- r(t) ramps 0 → 1 (min-jerk) over the raise.
- T is learned per stance side and kept from step to step, as the stepper's learned `hike` is. So the
  loop needs no fast integrator to carry the 5-11 deg static trim.
- Closed from the raise to the start of the lowering, then held through the touch-down, then faded over
  0.6 s once the foot is down.

### Results (`results/final_logged.*`, `results/final_synth.*`)

The stepper's own trajectories (the 18:19-18:40 lift, march and walk of 2026-11-11) with its trims
removed: 96 cases per law (4 plants × speed ×1/×2 × extra IMU delay 0/60 ms × gain ×1/×2, the
×2 being the law's gains doubled). Synthetic gaits: 240 cases per law (step period 4 / 3 / 2.5 s ×
shift 1.0 / 1.15 × 62 mm × march, forward, backward and both sidesteps × plants fit / hi × delay 0/60
× gain ×1/×2).

| law | stepper runs: lifts | falls OUT / IN | peak lean median / max | synthetic, P 3-4 s: lifts / falls | synthetic, P 2.5 s: lifts / falls |
|---|---|---|---|---|---|
| off (trims removed) | 0 % | 8 / 12 | 6.7 / 11.2 | 33 % / 28 of 160 | 25 % / 52 of 80 |
| the stepper's own trims (as logged) | 95 % | 0 / 0 | 4.8 / 6.1 | – | – |
| old PI through the lift | 4 % | 66 / 28 | 25.1 / 37.6 | 16 % / 150 of 160 | 12 % / 80 of 80 |
| old PI as built (armed) | 4 % | 62 / 28 | 15.1 / 37.8 | – | – |
| hip P, kp 0.5 | 14 % | 32 / 29 | 6.0 / 37.2 | – | – |
| trim kp 0.5 / ki 1 | 76 % (×1: 97 %, ×2: 53 %) | 17 / 11 | 9.8 / 37.8 | 79 % / 23 of 160 | 46 % / 45 of 80 |
| **trim kp 0.25 / ki 1** | **98 %** (×1: 97 %, ×2: 98 %) | **0 / 0** | 4.1 / 16.4 | **91 % / 0 of 160** | 61 % / 22 of 80 |
| trim kp 0.3 / ki 1 | 97 % | 1 / 0 | 4.0 / 25.9 | – | – |
| trim kp 0.3 / ki 1 + ankle 0.2 | 96 % | 0 / 0 | 3.7 / 17.4 | – | – |

Peak pitch stayed under 4 deg in every case (median 0.4-0.8).

## Recommendation

**Trim, kp 0.25 (0.25-0.3), ki 1.0, no ankle term, no rate term, no catch term; T starting at 0.**
- On the stepper's runs it lifts 98 % with no fall at nominal or double gain, on every play variant and
  with up to 60 ms of extra delay. Its peak lean (median 4.1 deg) is below the stepper's own trims (4.8).
- On synthetic gaits at 3-4 s per step (2-5 times the stepper's pace) it lifts 91 % with no fall in 160
  cases, in every direction.
- Seeding T from the stepper's learned trims was not tested.
- The loop's gain margin is set by kp: kp 0.5 holds at nominal gain and falls at ×2 (53 % lifts, 28 of 48
  cases fell), kp 0.25 holds at ×2. kp ≥ 1 limit-cycles at ~2.5 Hz (the play's dead band plus the
  servo lag).
- A gyro rate term made it worse. The ankle term raises lifts at slow periods and adds outward falls at
  2.5 s.

What it will NOT do:
- It does not make a quasi-static trajectory dynamic. At step periods of 2.5 s and below, the stepper's
  shift (1.0-1.15 × 62 mm in 0.75 s) carries momentum that falls OUTWARD with or without the loop
  (open loop: 52 of 80 at 2.5 s; the recommended law 22 of 80). The upper layer has to shift less and later for faster steps (a LIPM
  sway). A one-sided outward-catch translation in the base layer was tried with both signs and made
  every case worse (lifts 85 % → 16-71 %).
- It does not reach carpet with these soles and this play (above): no sim evidence either way.
- It holds roll only. On the hard floor pitch moved at most 4 deg (median 0.6), so there is no pitch
  loop.

## Interface (the base layer) and the sim plan

- **What the upper layer sends:** joint targets as now, plus the support state each tick: `DS`
  (double support), `SS_L` / `SS_R` (single support on that leg), and a raise progress 0..1.
  - Stepper: a new pose flag `S<L|R>` on the raise and swing poses. The firmware takes the raise
    progress from that move's own min-jerk progress. A pose without the flag holds, then fades.
    `B<L|R>` stays byte-for-byte as is.
  - Policy: the firmware already owns the gait clock, so stance and progress come from the clock phase
    (and c4 during a march), with the same convention as training.
  - The loop owns e, T[L/R] (in RAM, readable and settable from the CLI) and its correction. Nothing
    else is inferred.
- **Firmware:** `components/balance/stance_trim.{h,cpp}`, pure like `roll_pi`. It is called from the
  bench pose stream (`poseTick`) and, for policies, from `ctrl_task` after the shaper and before the goal
  ticks. Host unit tests pin it to golden numbers from the Python twin.
- **Sim:** one shared kernel (numpy / jax, like `walker_env.roll_play_substep`), applied between the
  action and the servo chain in `walker_env` and `env_mjx` at the control tick, with the IMU delay. T is
  part of the env state and is drawn per episode. A parity block runs it CPU vs MJX, and the SIL harness
  runs the C++ component against the CPU twin.
- **Servo gains:** no change is proposed. Stiffer servo P gains would shorten the servo lag that limits
  kp; this is not yet tested in sim.

## A2: the upper layer's lateral motion for speed (`sway.py`, `sway_grid.py`)

The stepper's quasi-static shift-then-raise carries momentum that falls OUTWARD at about 2.5 s per step,
with or without the trim loop (Stage A; the robot fell that way in `2026-11-12/132035_walk`). `sway.py`
replaces it with a continuous LIPM-style sway:

- **Pelvis lateral:** y = k·62 mm · sin(π(t − t_c)/T + π/2), alternating over the stance foot. The
  peak is at the middle of each single support. The amplitude switches per stance side where the sine
  crosses zero (mid double support): kL = 1.35 over the left foot, kR = 1.1 over the right. These are
  the stepper's own learned shift ratios (left 1.26-1.32, right 1.05-1.1).
- **Pre-sway:** the pelvis sways one half period before the first lift, so the first step starts with
  the CoM already swinging. Without it the first step from rest falls.
- **Single support:** 50 % of T, centred on the peak. The swing sole rises 40 mm · sin²(πu) and moves
  (min-jerk) over the middle 60 % of single support.
  - A sidestep moves the leading foot outward first; the other foot follows.
  - A turn puts ±0.15 rad on the swing hip yaw.
- **Trim loop** on: kp 0.25, ki 1.0, T seeded with the stepper's learned trims (hike L 8, R 3).

Results, 8 steps per run, 6 directions (march, forward, backward, sidestep L / R, turn) × 4 plants
(fit, lo, hi, hard floor) × extra IMU delay 0 / 60 ms = 48 cases per step time (`results/sway_g4.json`):

| step time T | lifts | falls | peak lean, median / max deg |
|---|---|---|---|
| 2.5 s | 100 % | 0 | 3.8 / 18.5 |
| **2.0 s** | **100 %** | **0** | 4.3 / 6.3 |
| 1.75 s | 89 % (fit 96, hard 100, lo 100, hi 60) | 0 | 4.5 / 7.0 |
| 1.5 s | 70 % (lo 100, hard 80, fit 77, hi 26) | 0 | 4.5 / 7.7 |

**The fastest step time at ≥ 95 % lifts and 0 falls across the plants is 2.0 s per step**, in every
direction. That is 4-7 × the host stepper's pace.
- Below it no case falls, but the swing sole clears 5 mm for less than half of the single support. The
  high-play plant (10 / 10 / 3 / 2) limits first.
- At 2.0 s on fit / hi, 6 directions:

  | loop | lifts | falls |
  |---|---|---|
  | off | 0 % | 0 / 12 |
  | trim with T starting at 0 | 51 % | 0 / 12 |
  | trim seeded with the stepper's learned T | 100 % | 0 / 12 |

  At these step times the per-side learned trim has to be there from the first step: the integrator has
  no time to learn it within one single support.
- With a single amplitude of 1.2 on both sides, sidestepping left falls 4 / 8. So the per-side shift
  matters as soon as the robot steps sideways.
