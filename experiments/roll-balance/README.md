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

## B2: the sway gait through the firmware's own track + trim (`sil_gait.py`, software in the loop)

`sil_gait.py` plays the sway gait through the code the robot runs: `firmware/host/build/libctrl_sil.so`
(`sil_gait.cpp` → `components/gaitplay/track` + `components/balance/stance_trim` with its `TrimDriver`).

- **Host side.** The sway (above) is sampled every 100 ms into `track add` frames in the CLI's hex form,
  with the phase, kLift while stepping, a kStepEnd frame at every mid double support and its kRest frame
  (the centred stand on those feet). The timeline ends at the last step end, so the firmware rests on its
  own when the queue runs dry there. The host appends rolling, 2 steps ahead of the play head.
- **Robot side.** The track is sampled at 50 Hz. The support state comes from the phase, and the trim
  runs from it (kp 0.25, ki 1.0, T seeded hike L 8, R 3). The 12 goals go through ticks and back
  (calibration round trip) into the sim plant (grade.py servos + the per-side play). The roll comes
  back one tick late, plus 0 / 60 ms.
- **Grid.** 4 step times × 6 directions × 4 plants (fit, lo, hi, hard) × 2 delays = 192 runs of 8 steps
  (`results/sil_gait.json`, summary `results/sil_gait.txt`).

**Found on the way: the slow sidestep needs a width-aware sway.** With A2's sway as it was, sidestepping
left at T = 4 s fell once and aborted 11 times (lean toward the swing foot up to 19.5 deg on left
stance; `results/sil_gait_narrow.txt`). The Python testbed with `laws.Trim` shows the same thing (2 / 8
lifts at 4 s), so it is the gait, not the firmware port. Left stance in a left sidestep stands on a foot
15 mm further out; at 2 s the CoM's swing covers that, at 3-4 s (quasi-static) it does not. The fix,
`Sway(wide=True)`, adds the stance foot's offset from the feet's mid line to the amplitude. A2's runs
did not use it; everything below does.

Predictions (`results/sil_gait.txt`; TS = the firmware's per-step line, deg). The sway amplitudes and the
seeds are the robot's own working values from the B1 runs (below): 1.3 × 62 mm over the left foot,
1.25 × over the right, T seeded L 5.2, R 3.4. With A2's 1.35 / 1.1 and 8 / 3 the grid is also 0 falls,
0 aborts, 100 % lifts (`results/sil_gait_a2amp.txt`).

| T | runs | falls | trim aborts | lifts | TS lean toward swing, median / max | TS outward max | T at mid support L / R | hike L / R | swing sole peak clearance median | queue max (frames) |
|---|---|---|---|---|---|---|---|---|---|---|
| 4 s | 48 | 0 | 0 | 384 / 384 | 3.0 / 6.9 | 4.1 | 8.2 / 5.7 | 8.5 / 5.9 | 36 mm | 81 |
| 3 s | 48 | 0 | 0 | 384 / 384 | 3.1 / 7.4 | 3.7 | 7.5 / 5.4 | 7.8 / 5.7 | 35 mm | 61 |
| 2.5 s | 48 | 0 | 0 | 384 / 384 | 3.4 / 7.3 | 4.0 | 7.2 / 5.1 | 7.6 / 5.6 | 34 mm | 51 |
| 2 s | 48 | 0 | 0 | 384 / 384 | 3.5 / 7.2 | 3.5 | 6.5 / 4.6 | 7.0 / 5.4 | 36 mm | 41 |

Peak torso tilt is at most 6.3 deg in every direction and step time. On the robot, a TS line far outside
these numbers (a lean toward the swing foot over ~10 deg, an ABORT, T running to its 14 deg limit)
falsifies the model; `track_host.py` stops the track on the first one.

`track_host.py` is the host for the robot. It is a dry run by default: it prints the CLI session, the
upload sizing and the prediction above for the chosen gait. `--live` sends it over the Wi-Fi CLI, on the
Stage C plan only. Sizing at 100 ms keyframes:
- 4 frames per `track add` line (181 of the 192 chars).
- 2.5 lines/s to keep pace while playing.
- Before `track play`: 20 lines at T = 4 s, 10 at 2 s (2 steps ahead).
- The per-line round trip and the frames/s it gives are measured on the first live run (`--live` prints
  them); the 100 ms used above is the stepper's pose round trip, an assumption until then.

## Stage C: the hardware plan (hard floor first; one phase at a time)

Each phase has its sim number, a stop rule and what would falsify the model. A phase starts only after the
previous one met its number. The fall guard stays on throughout; the robot is on the bench tether or on
the floor with a hand near it.

| phase | how | sim prediction | stop rule | falsified if |
|---|---|---|---|---|
| C0 flash | `flash_prec16_trim.sh` (B1) / the B2 image | `trim` prints "aborts none", kp 0.25 ki 1.00, T 0 / 0 | any other status | -- |
| C1 stand + hand push | T at 0 (`trim T clear`: a seeded T would ramp a hike in with both feet down); crouched double support; `pose <the same 17> s1000 SL`, then push the pelvis sideways ~3 deg for 2 s, release; `trim`; `trim off` | hike follows kp·s·e + ki·∫: about +0.75 deg at once for a 3 deg lean toward the right foot (L stance), growing ~3 deg/s while held; the sign is + (lifts the right hip) | `trim ABORT`, or hike > 10 deg -> `trim off` | hike moves the other way (a sign error), or it moves with no lean |
| C2 single lift each side | the stepper with S flags (`SL`/`SLh`, `SR`/`SRh`), seeded `trim T L 5.2`, `trim T R 3.4`, 2 lifts per side (done 2026-11-12 with plain S poses: below) | Stage A on the 11-12 runs: 96-97 % lifts, 0 falls, peak lean median 3.4, max 5.0 deg | ABORT, or \|e\| > 8 deg in the flight record -> land (the stepper's own lowering) | the lift fails with a level torso (play larger than the fit), or e > 8 deg with T near its seed |
| C3 march on the track | `track_host.py --T 4 --dir march --steps 6 --live` (B2 image) | the B2 table at 4 s: 100 % lifts, TS lean toward swing median 3.0 / max 6.9, outward max 4.1, T at mid support L 8.2 / R 5.7, hike L 8.5 / R 5.9 deg, swing sole ~36 mm | `track_host.py` sends `track stop` on an ABORT or a lean toward the swing foot > 10 deg; a refused line or a lost link -> `track halt` | TS lean > 10 deg on two steps in a row of one side, or T walking to the 14 deg limit, or the sole visibly not clearing while the TS numbers are in range (then the plant's play is wrong: re-fit with `stepper_replay`) |
| C4 faster | march at T = 3, then 2.5, then 2 s (one step down per green run) | the B2 table row for each T | as C3 | as C3, against that row |
| C5 walk | at T = 4 s: fwd, back, sideL, sideR, turn; then step T down as in C4 | 0 falls, 100 % lifts at every T and direction in the SIL; peak torso tilt ≤ 7 deg | as C3 | as C3; also a sidestep that leans into the swing foot at slow T (the `wide` sway is what fixed that in sim) |

The upload numbers (round trip per `track add` line, frames/s, how far ahead the host stays) are
measured on the first C3 run; `track_host.py` prints them.

## The B1 firmware on the robot, 2026-11-12, against the sim (`fw_replay.py`)

The first hardware runs of the B1 trim (hard panels; plain `pose ... s<ms> S<L|R>[h]` over Wi-Fi). Five
runs: `hw_sessions/2026-11-12/fwtrim_*.json`, flight record `flight_20261112T215156Z_prec16.frec`.
`fw_replay.py` drives the sim plant with the robot's own nominal targets, tick by tick from the flight
record. The trim twin (`sim/stance_trim.py`) runs on the robot's state transitions, with the robot's T at
each chain's start; the roll comes back one tick late (`results/fw_replay_1112.json`).

Lean = peak toward the lifted foot while closed; e, hike and T where the hold begins; robot / sim (fit
plant; lo and hi in brackets where they differ), deg:

| run | what was sent | lean | e at hold | hike at hold | T at hold |
|---|---|---|---|---|---|
| lift 1, R stance | shift 1.15×, T from 0 | **10.1** / 4.1 | −8.2 / −2.3 | 7.0 / 3.4 | 5.5 / 2.9 |
| lift 2, R stance | shift 1.15×, T from 5.5 | **10.8** / 2.6 (1.8-3.6) | −9.1 / +0.4 | 10.3 / 6.3 | 8.6 / 6.4 |
| lift 3, R stance | shift 1.25×, T from 8.6 | 0.7 / 2.0 | 0.0 / 0.1 | **3.4** / 5.9 (4.3-7.2) | 3.4 / 5.9 |
| lift 4, L stance | shift 1.3×, T from 0, 4.5 s raise | 2.8 / 3.6 | 0.0 / 0.0 | 5.2 / 5.1 | 5.2 / 5.1 |
| march, 2 × R | 1.25× | 1.3 / 3.0-3.4 | 0.3 / −0.2 | **3.0** / 5.3-5.6 | 3.1 / 5.3-5.6 |
| march, 2 × L | 1.3× | 2.0 / 2.7-2.9 | −0.3 / −0.3 | 5.1 / 5.7-5.9 | 5.2 / 5.8-6.0 |

Where the sim is wrong:
- **Right stance with too little shift.** At 1.15× the robot tipped toward the lifted foot at foot-off
  (10-11 deg); the sim leans 2-4 deg on every play variant and does not tip. At 1.25× both hold. So the
  sim's right-stance side is optimistic about how far the CoM has to go. More hip play does not explain
  it (the hi plant still leans only 3.6). Nor does a lateral torso CoM offset: with `com_y` from
  −6 to +12 mm the replayed lean changes by under 0.5 deg on every run. What is left untested is a
  softer right side at foot-off (the right ankle / sole, or the right hip's play behaving differently
  under load than the fit says).
- **Right-stance hike.** Once it holds, the robot settles at hike 3.0-3.4 deg on the right; the sim needs
  5.3-5.9. The right side is the same story from the other end: the robot needs more shift and less hip.
- **Left stance matches.** The sim reproduces the left side within 1 deg (lean 2.8 / 3.6, hike 5.2 / 5.1,
  learned T 5.2 / 5.1).
- **Lean in the march.** The robot leans 1.3-2.0 deg toward the lifted foot; the sim leans 2.7-3.4 (both
  sides). The robot is steadier than the sim there.
- In the lo-play plant the sim march fell 20.6 s into the run; the robot did not fall.

For the track and the sway: the SIL grid above now uses the robot's working values (1.3 / 1.25, seeds
5.2 / 3.4) and still holds everywhere. Because the sim is optimistic about right stance, the right-side
amplitude should not go below 1.25 on the robot even where the sim would allow it.

## B2: the provisional learned trim and the handover (`handover.py`)

**The provisional T** (firmware `StanceTrim`, the Python twin, `sim/trim_kernel.py`). T integrates only
while |e| ≤ 6 deg. A chain that ends with its peak |e| past 8 deg, with an abort, or with the hike at its
clamp puts T back to its value at the chain's begin. Why: in the 2026-11-12 shuttle a lift with too
little shift wound T L from 5.4 to the 14 deg limit, and the lifts after it failed from that T. In the
SIL grid above (re-run with the rule) no step puts T back (0 of 1536). The predictions did not change:
same falls, aborts, lifts and TS numbers.

**The handover.** With the track's phase map a single support is the middle half of each step, so at a
step time T the gap from one single support's end to the next one's begin is T / 2: 1 s at T = 2 s. The
0.6 s fade is over 0.4 s before the next ramp starts. They overlap only below T = 1.2 s (and on a
policy's faster clock, where the driver's residual carries the old hip's fade through the new chain's
ramp). In the SIL, per handover at T = 2 / 2.5 / 3 s (march, forward, sidestep left × fit / hi, 72 runs):

| variant | falls | lifts | old-hip jump at the handover | peak roll rate, median / max deg/s | roll excursion, median / max deg |
|---|---|---|---|---|---|
| residual, fade 0.6 s (the firmware) | 0 | 144 / 144 | 0 | 36.6 / 48.7 (T 2) · 29.6 / 34.2 (2.5) · 25.0 / 28.2 (3) | 3.62 / 4.03 · 2.62 / 3.36 · 2.18 / 2.41 |
| old hip dropped at the begin | 0 | 144 / 144 | 0 | identical | identical |
| residual, fade 0.4 s | 0 | 144 / 144 | 0 | 36.2 / 49.5 · 30.1 / 33.6 · 24.8 / 27.6 | 3.64 / 4.22 · 2.62 / 2.85 · 2.21 / 2.40 |
| residual, fade 0.3 s | 0 | 144 / 144 | 0 | 37.1 / 50.3 · 30.6 / 34.4 · 24.9 / 29.1 | 3.70 / 4.43 · 2.62 / 2.82 · 2.22 / 2.40 |

The roll rate around the handover is the sway's own; the fade length and the handover method do not
change it at the track's step times. On the track the fade falls in the double support, while the sway
is already carrying the pelvis to the other foot. That is where the stepper saw the fade roll the body;
the SIL shows no extra roll from it at the next ramp-in.

## C3 on hardware fell; the dwell sway (`dwell.py`, `dwell_grid.py`)

**The fall.** First track run on the robot, 2026-11-12 16:25: `track_host.py --T 4 --dir march --live` with the
sine sway. The robot fell on the first step toward the lifted left foot, over its right stance foot.
- The trim aborted at e −15.4 deg; T was put back as designed.
- From the flight record: the left knee started to flex at a pelvis shift of ~1.0 × 62 mm, and the
  pelvis was at ~1.1-1.2 × as the robot went over. The sway's 1.25 × peak came only in the middle of
  single support.
- The robot needs 1.25 × over its right foot and 1.3 × over its left before the foot comes up (B1
  lifts). At 4 s a step there are no dynamics to carry it.
- The SIL passed this gait because the sim is optimistic on right stance (above). So the SIL cannot
  set the static shift.

**The dwell sway.** The static requirement is imposed as a hard constraint (`dwell.K_STATIC`, L 1.3, R
1.25, parameters). Each step:
1. A min-jerk shift (both feet down) to k_static × 62 mm over the new stance foot, plus that foot's own
   offset from where it stands.
2. A settle there.
3. Single support with the pelvis holding. The sole rises to 22 mm (min-jerk) over the first 35 %, moves
   over the middle and comes down over the last 35 %.

The phase sent with the frames is warped so the firmware's support map lands on these instants (lift-off
u = 0.25, raise end, lower start, touchdown u = 0.75). No firmware change.

`track_host.py` now refuses before sending anything if the frames put the pelvis under the requirement
at any lift-off or touchdown. It recovers the shift by inverting the IK on the frames' stance hip roll
goals. The sine sway that fell is refused: 0.89 × right, 0.92 × left at lift-off. The dwell plans pass
(tightest 1.302 ×).

**Timing, from the SIL** (`results/dwell_sweep.json`, `dwell_timing*.json`; 6 directions × fit / lo / hi
/ hard):
- What limits the step time is the shift. It moves the pelvis ~158 mm (1.25 × over one foot to 1.3 ×
  over the other), and the roll play rings: a 0.6 s shift falls OUTWARD right after the next lift-off in
  every case, 0.8 s in a third of them, 1.0 s holds but leans up to 11.7 deg, 1.2 s holds at ≤ 5 deg.
- Single support needs ≥ 1.8 s: raise 0.63 s. At 1.2-1.5 s the raise is fast enough to fall or lean.
- A longer settle does not help (the oscillation is set off by the shift); a fixed fraction of T
  (`ds`) was worse than absolute times.

The schedule (`dwell.SCHEDULE`: shift / settle / single support) and the predictions at it
(`results/dwell_schedule.txt`: 48 runs per T, delay 0 and 60 ms, seeded T 5.2 / 3.4):

| T s | shift / settle / single support s | raise s | falls | lifts | TS lean toward swing med / max | outward max | T at mid support L / R | hike L / R | clearance med (sim; 22 mm commanded) |
|---|---|---|---|---|---|---|---|---|---|
| 6 (start here) | 1.5 / 0.5 / 4.0 | 1.40 | 0 | 288 / 288 | 2.6 / 4.7 | 3.5 | 6.5 / 6.6 | 6.3 / 6.4 | 18 mm |
| 5 | 1.4 / 0.4 / 3.2 | 1.12 | 0 | 288 / 288 | 2.6 / 4.7 | 3.6 | 6.4 / 6.2 | 6.3 / 6.2 | 18 mm |
| 4 | 1.2 / 0.4 / 2.4 | 0.84 | 0 | 288 / 288 | 2.3 / 4.7 | 3.8 | 6.5 / 5.5 | 6.6 / 5.5 | 17 mm |
| 3.4 | 1.2 / 0.4 / 1.8 | 0.63 | 0 | 288 / 288 | 2.4 / 5.0 | 3.9 | 6.6 / 5.0 | 6.9 / 5.1 | 16 mm |
| **3.2 (the floor)** | 1.2 / 0.2 / 1.8 | 0.63 | 0 | 288 / 288 | 2.4 / 5.1 | 3.7 | 6.5 / 5.0 | 6.9 / 5.1 | 17 mm |
| 3.0 | 1.0 / 0.2 / 1.8 | 0.63 | 0 | 288 / 288 | 2.7 / **11.7** | 4.0 | 6.7 / 5.0 | 7.0 / 5.0 | 16 mm |
| 2.5 | best of 7 splits tried | ≤ 0.53 | 4-9 / 24 | 115-123 / 144 | up to 13 | | | | |
| 2.0 | best of 8 splits tried | ≤ 0.42 | 14-24 / 24 | ≤ 87 / 144 | | | | | |

**T = 2 s is infeasible with the static constraint.** Every split tried falls: a 1 s single support
and a ≤ 0.8 s shift do not fit in it. The floor is 3.2 s: a 1.2 s shift, 0.2 s settle and 1.8 s single
support. Below that the shift or the raise sets the robot rolling. The 2.0 s the sine sway reached in A2
relied, as far as the sim shows, on the CoM swinging through, which the robot's static requirement at
foot-off rules out.

In the sim the T at mid support settles L 6.4-6.7, R 5.0-6.6; the robot learned L 5.2, R 3.4. The hike
the sim wants on the right is higher because its static picture is not the robot's. Watch the TS T on the
robot: it should stay near the seeds.

**On the robot:** start at T = 6 s (`track_host.py`'s default). Step down 6 → 5 → 4 → 3.4 → 3.2 only
after a green run at the step above; the host refuses T under 3.2 s unless `--below-floor`. With the
128-frame queue the host stays at most 8 s ahead. The upload measured 29 ms per line (138 frames/s), so
that is plenty.

## Stage C dwell runs on hardware (2026-11-12 16:35-16:51) and the fixes (`replay_trim.py`)

What the flight record shows (numbers from the record, not the sim):
- **Floor.** T 6 and T 5 march clean: TS lean ≤ 2.4 deg. Below that, on right stance, the body rolls
  toward the lifted left foot in mid/late single support. The roll starts at u ≈ 0.5 (u = 0 at lift-off,
  1 at touchdown), reaches −6 to −8 deg e by the lower start and peaks at 9-10.6 deg (u 0.65-0.75). Count:
  T 4 1 of 3 right steps, T 3.4 and 3.2 3 of 3, forward at T 5 1 of 4 (step 1); left stance never past 3.3.
- **Not an unfinished shift.** The shift is complete at lift-off: right hip roll tracking −0.3 deg, e ≈ 0
  through the raise. On the bad steps the right hip roll servo falls 3.5-7.5 deg behind its goal at u 0.5;
  on the good ones 2.6-2.9 deg. Next test: k_static R 1.35.
- **Stop rule.** The host's stop rule never saw a TS line during play: over Wi-Fi the firmware's async lines
  do not reach a command's reply. `track` now repeats the last TS line, and the host also polls `track steps`.
- **Learned-T hole.** The keep rule looked at the peak |e| while CLOSED only, and these tips grow in the
  hold. So T R wound 3.40 → 5.24 → 6.31 on steps that tipped. Fixed: the whole single support counts.
  Replayed through the twin, the right T stays at its 3.40 seed on every chain of both runs
  (`results/replay_trim_1112.txt`).
- **Heading.** The IMU yaw drifts at the gyro-z bias, −1.33 deg/s at rest. Bias-corrected, the forward
  8-step run turned +5 ± 4 deg left.
- **Turning.** The old turn pattern (the swing hip to ±yaw) does not turn: +2.9 deg on the robot over the
  6-step turn, +2.4 in the sim. `Dwell(yaw_trim=deg per step)` turns about the planted foot: the stance hip
  goes −c → +c, the swing hip +c → −c. In the sim, +4 → +34 deg and −1.5 → −14 deg over 6 steps, a gain of
  ~1.4-1.5. `track_host.py --yaw-trim`; calibrate the gain on the robot.

**k_static R 1.30** (the default since the 16:55 runs, `dwell.K_STATIC`, `track_host.ROBOT_AMP`):
- With `--amp 1.3,1.30`, march at T 4 / 3.4 / 3.2 and forward 8 at T 4 run with no right step past 4 deg
  (0 of 16; at 1.25: 7 of 12 at 9.2-10.6).
- The right hip roll servo's lag at u 0.5 drops from 3.2-6.2 deg to 2.3-3.7. That is the hip-yield mechanism.

**Forward at T 3.4 still tips** (17:00:05, R 1.30). All 4 right steps peak at 10.1-11.7 deg at u 0.59-0.76,
with the hip lag at 3.5-6.7 deg. It is the same mid-support hip-yield tip, and it starts at u ≈ 0.5, while
the swing foot and the pelvis move forward (u 0.35-0.65). It is not a touchdown event.

**The next limiter is the shift.** On the robot at T 3.2 the double support is 1.4 s (shift 1.2 + settle
0.2). The roll rings at 22-47 deg/s through the shift and is quiet (< 5 deg/s) for only 0.1-0.4 s before
lift-off, so there is no slack there. The single support (1.8 s) shows leans ≤ 4 deg, so that is where the
slack is.

SIL, fit plant, k R 1.30, 6 directions, 6 steps:

| T s | shift / settle / single support s | raise s | falls | lifts | lean max deg |
|---|---|---|---|---|---|
| 3.2 | 1.2 / 0.2 / 1.8 | 0.63 | 0 | 36 / 36 | 3.0 |
| 2.8 | 1.2 / 0.2 / 1.4 | 0.49 | 0 | 36 / 36 | 3.3 |
| 2.8 | 1.0 / 0.2 / 1.6 | 0.56 | 0 | 36 / 36 | 10.1 |
| 2.5 | 1.0 / 0.2 / 1.3 | 0.45 | 0 | 36 / 36 | 6.6 |
| 2.5 | 0.9 / 0.2 / 1.4 | 0.49 | 0 | 36 / 36 | 8.5 |

So the faster step comes from a shorter single support with the 1.2 s shift kept. The schedule adds
3.0 s (1.2 / 0.2 / 1.6) and 2.8 s (1.2 / 0.2 / 1.4); the host floor is now 2.8 s. On the robot the march
reaches 3.2 s and forward only 4 s, so forward is limited by its own mid-support tip first.

**Turning.** `--dir turn` / `turnL` / `turnR` are now the turn about the planted foot (`yaw_trim` ±4
deg/step asked, ~5-6 deg/step in the sim). The old pattern is `splay`.

**`track steps`.** The firmware keeps the last 16 TS lines. The jsonl showed only 6 because the
stepper's robot_io logs the last 300 characters of each reply. Every step the host parses is in the run's
`summary` row.

## Heading, turning and the first step (hw 2026-11-12 17:04-17:14, T 4-5, R 1.30)

**Turns (`turnL` / `turnR`).** Turns are made about the planted foot. Drift-corrected heading for the
deg/step asked:

| run | heading |
|---|---|
| turnR 2 | −6.4 |
| turnR 4 | −13.1 |
| turnR 6 | −24.1 |
| turnR 8 | −33.4 (−32 asked) |
| turnL 4 | +28.7 |

All runs had `--first R`, and every TS was ok. Fit: heading per step = 1.34 × asked + 1.9 deg (the sim
said 1.4-1.5 ×).

**The +1.9 deg/step to the left with nothing asked** is in every direction. It comes from the double
support, during the lateral shift (bias-corrected gyro z by phase):
- Each shift onto the right foot turns the robot left: +10.6 to +29.7 deg per run.
- Each shift onto the left foot turns it right: −8.6 to −13.6 deg per run.
- Single support contributes −6 to +3 deg.

It is not a hip-yaw servo offset: the hip-yaw tracking errors are ≤ 0.25 deg and symmetric.
`track_host.py` now adds −1.4 deg/step by default (`--yaw-trim`, added to the direction's own turn).
Forward 8 at T 4 with it: +7 deg over the run. Without it, 30-40 deg built up over the day's runs (by
camera). A counter-twist during the shift onto the right foot is the source-level option, kept in reserve.

**The first right stance.** With the first swing L, the first right stance came straight out of the rest
crouch and tipped in fwd T 5, turnR and back runs. Its lead-in, shift, T and lift-off state match the
later right steps. The pre-lift roll rate is lower (~10 deg/s vs 20-30), because the first shift is half
the distance. So it is the same mid-support hip yield, set off when step 1 also moves the swing foot.
`--first R` (the first stance is the left) is now the default.

**The spurious TS 1.** With the dwell gait, the first keyframe carried kLift at phase u = 0.75 of the
half cycle before the first one. The firmware read that as a one-tick single support, which printed a
TS line and shifted the numbering by one. In the forward run, "TS 9 R" was the 8th real step (the last
right stance), not a closing step. kLift now starts at the first lift-off.

## k R 1.35, the forward pitch, the heading hold (hw 17:45-17:58)

**k_static R 1.35** is the default. At 1.30 the first right stance of back 12 at T 5 tipped (in 10.1-10.4)
at servo P 32 and at P 64 alike, so the P gain is not the fix. At 1.35, back 12 (P 64) and back 8 (P 32)
ran clean: right in 1.3-1.7 deg, out ≤ 1.5 deg, right hip lag at mid support 2.8-3.6 deg, the same as the
clean marches.

**Forward at T 3.4 still degrades on the right steps** (in 7.4 and 8.4 on the 3rd and 4th right stance,
at 1.35):
- The shift arrives: the right hip roll is at the same +15.4 deg at every lift-off, and the pre-lift roll
  rate is no higher than on clean steps.
- The heading jumps ~9 deg between steps 6 and 7.
- In the record, the right stance pitches the body forward to +6.7 to +7.7 deg by u 0.8, while the
  pelvis moves ahead of the stance foot as the swing foot travels. The left stance pitches only +2 to +4.
- Plan-level option: `Dwell(pelvis_x="stance")` keeps the pelvis fore-aft over the stance foot through
  single support and moves it only in the shift. It is not the default yet; it needs an A/B on the
  robot.
- A "do not lift until the roll is quiet" check would need a conditional hold in the firmware track
  player: a frame flag that pauses the track clock until |gyro x| is under a threshold, about 40 lines
  plus tests. The data does not point at it: the shift arrives, and the pre-lift rates are normal.

**Heading hold** (`track_host.py --heading-hold on`, the default with `--live`):
- After every reported single support the host reads the fused yaw (`imu` quat; the online gyro bias
  keeps it from drifting). Steps not yet uploaded are asked `(dpsi_cmd + k × error − 1.9) / 1.34`
  deg/step, from today's heading model; the plan for those steps is regenerated and the upload goes on.
- `dpsi_cmd` is 0 for straight, ±5 deg/step for turnL / turnR, `--turn TOTAL / steps`, or
  `--heading-rate × T`, so a commanded heading rate is the same loop.
- k = 0.5: half the error is fixed per step. Asked turns are clipped to ±8 deg/step.
- Locked steps keep their frames unchanged; tested offline (re-planned upload, pre-check still passes).

## k R 1.30 at every T, the upper bound, the lean stop; what the tips are (hw 18:09-18:29)

**k_static R 1.30 at every T** (`dwell.K_STATIC`, `dwell.k_static_R`). At T 4 with 1.35 the robot fell
OUTWARD over the right foot in the settle, with both feet still down (18:09: roll +1 → +22 deg within 0.5 s of
the pelvis arriving). An inward tip costs a step; an outward topple is a fall nothing catches.

**The pre-check has an upper bound.** At every lift-off and touchdown the shift must lie between k_static and
k_static + 0.05 (`track_host.UPPER`), both × 62 mm. A plan past the bound is refused; `--allow-over` lets an
`--amp` A/B pass it.

**`--stop-lean` defaults to 14 deg** (it was 10). At 18:2x, march 4 at T 5 with `--amp 1.3,1.20` tipped on
both right steps (TS in 9.3 and 9.0). The walk went on through them and finished. The firmware's abort at
15 deg stays the hard stop.

**Correction: there is no hip-roll "lag" or "hip yield".**
- The lags quoted above (3.2-7.5 deg) were nominal − measured. The nominal leaves out the trim's hike, so
  that number was the hike itself, delivered.
- Against the goal actually sent, the stance hip roll tracks within 1-2 deg at mid support. Median R tips
  −0.4, R clean −0.9, L clean +0.9. Each tick the servo gets its gap ÷ 20 ms as goal speed, with acc 0.
- Load stays at 3-25 % duty and current at 0.2-0.3 A, with no protection status bits.
- What the tips are: the stance sole rolls onto its inner edge (video 18:2x).

**Where the CoM is** (`plant FK on the recorded joints`; distances from the stance ankle-roll axis; model sole:
84 mm wide, centreline 18.5 mm outboard, so the inner edge is 23.5 mm inboard):
- k 1.30 puts the CoM 6-8 mm outboard of the ankle axis. The CoM moves ~37 mm per 1.0 k, because the legs
  stay put.
- At lift-off the measured joints put it within 1 mm of the plan, and the IMU agrees with the encoders to
  within 1 deg.
- After lift-off a rotation toward the swing foot builds that the encoders do not see. At 0.4 s it is R
  +1.5 deg (IQR 1.4-1.7) and L +3.1 (2.8-3.9). The tips run away and start tipping at ~8 deg (median).
- 1 deg at the foot or ankle moves the CoM 3.7 mm. 1 deg at the stance hip roll moves it 0.6 mm outward.
  So the hike levels the torso, but it has almost no authority over the CoM.
- By the model, k 1.30 leaves ~30 mm (8.5 deg about the edge) to the inner edge. Yet the robot tips there.
  - The real CoM may sit further inboard than the model's.
  - The load-bearing inner edge may be closer to the ankle axis than 23.5 mm.
  - The ankle roll may yield.
- The record cannot separate these. Two kitchen scales under the feet, read in double support at k 1.0 /
  1.2 / 1.3, would measure the real CoM directly. A ruler gives the sole edges against the ankle axis.

## The ankle feed-forward and the two-stage shift (host flags, for the 2026-11-13 A/B)

The working explanation for the tips is the free play inside the roll servos (#277). Within the play the
stance leg pivots at the ankle-roll axis, and the sole cannot help until the play is taken up. The play-drop
after lift-off (R +1.5, L +3.1 deg at 0.4 s) moves the CoM inboard by about 3.7 mm per deg. Two plan-level
answers, both host only (the frames carry them) and both off by default:

- **`--ankle-ff L,R`** adds a fixed feed-forward in deg on the STANCE ankle roll. + = outward: the body
  rolls toward the outside of the stance foot, and the CoM moves ~3.7 mm per deg in the plant FK (2 deg on R
  stance at k 1.20 is 7.5 mm). It follows the swing foot's height profile: in over the raise, held, out over
  the lower. It is separate from the hike, which acts on the hip roll and so has ~0 authority over the CoM.
  The pre-check refuses a plan whose ankle rolls leave the plant's range less 2 deg.
- **`--k-ds L,R`** is the two-stage shift. The double support goes only to k_ds × 62 mm. The pelvis then
  travels the rest of the way to `--amp` over the raise (lift-off → raise end, min-jerk), so it reaches
  k_static only once the swing foot is off. 0 means single-stage for that side. The pre-check wants k_ds at
  lift-off and `--amp` at touchdown, both under k_static + 0.05. With `--amp 1.3,1.35 --k-ds 0,1.20` the CoM on
  R stance goes from 2.4 to 8.0 mm outboard of the ankle axis over the raise.

The A/B, mid-board, march 4 at T 5, two runs each, run from the repo root of this branch. R k 1.20 is the
provoked case: 2 of 2 R steps tipped there at 18:2x.

```
# A  baseline (tips)
python experiments/roll-balance/track_host.py --T 5 --dir march --steps 4 --stop-lean 14 --amp 1.3,1.20 --live --note "A1 mid-board"
# B  ankle feed-forward 2 deg on R stance (predicted: R tips 0 of 2, torso roll toward swing ~2 deg lower at 0.4-0.6 s)
python experiments/roll-balance/track_host.py --T 5 --dir march --steps 4 --stop-lean 14 --amp 1.3,1.20 --ankle-ff 0,2 --live --note "B1 mid-board"
# C  two-stage R 1.20 -> 1.35
python experiments/roll-balance/track_host.py --T 5 --dir march --steps 4 --stop-lean 14 --amp 1.3,1.35 --k-ds 0,1.20 --live --note "C1 mid-board"
```

Leave out `--live` for the dry run with the pre-check. Then take the winner at k R 1.30 to fwd/back 8 with
`--pelvis-x stance` (`--dir fwd --steps 8 --amp 1.3,1.30 --pelvis-x stance ...`, likewise `--dir back`), and to
L: `--ankle-ff 3,<R>` (L's play-drop is about twice R's), or `--amp 1.35,<R> --k-ds 1.20,<R k_ds>`. The
upper bound for L is 1.35 as well.

## The ankle feed-forward on the robot (hw 2026-11-12 19:10-19:28): the defaults, the outward stop

All runs were mid-board on image b5e5ff2 with the heading hold on; k R 1.30 unless noted. The full table is
on #297.

| run | `--ankle-ff` L,R | result |
|---|---|---|
| march 4, T 5, k R 1.20 (provoked) | 0,0 | R tips on 4 of 4 (9.1-9.6 deg) |
| the same | 0,2 | R clean on 4 of 6 (1.2-1.7) |
| fwd 8, T 5 | 0,0 | R tips on 4 of 4 (10.3-12.1) |
| fwd 8, T 5 | 0,2 | 9 of 9 clean |
| back / fwd / back / fwd 8, T 5 | 2,2 | 35 of 36 clean (one first-R-stance tip); heading within 4 deg per walk |
| fwd 8, back 8, T 4 | 2,2 | 9 of 9 and 9 of 9 clean, lean ≤ 4.8, outward ≤ 5.0 |
| march 6, T 4 | 2,2 | the first R step tips 11.6, the rest clean |
| fwd 8, back 8, T 3.4 | 2,2 | R tips on 4 of 4 and 3 of 4 (to 13.0) |
| fwd 8, T 3.4 | 2,4 | clean inward (≤ 2.8), but R leans OUTWARD 5.1-7.0 |
| back 8, T 3.4 | 2,4 | R outward 7.2 and 8.2, then fell (52 deg) |

**Defaults now** (`track_host.ROBOT_ANKLE_FF`, `ROBOT_PELVIS_X`): `--ankle-ff 2,2` and `--pelvis-x stance`.
`--ankle-ff 0,0 --pelvis-x mid` gives the old plan.

**Outward stop.** `--stop-out 6.0` sends `track stop` on a TS line whose outward lean exceeds the limit, as
`--stop-lean` (14) does for the inward side. Clean walks stay ≤ 5.0 outward. The fall came after R out 7.0 on
one walk and 7.2 / 8.2 on the next.

**Pace.** T 4 is the recommended pace. T 3.4 is not usable yet: 2 deg on R is too little there and 4 is too
much, so a fixed feed-forward is the wrong tool at that pace.

## By direction with ff 2,2 (hw 2026-11-12 19:30-20:00), and the learned ankle trim (host)

All runs were on host fddfd18, mid-board, with the heading hold on. There were no falls and no stop-rule
trips. The table is on #297.

| direction | T 5 | T 4 |
|---|---|---|
| fwd 8 | 9 of 9 (from the far end) | the first two R steps 13.0 / 12.9, then clean |
| sideL | 7 of 7 | 7 of 7 |
| sideR | the first R step tipped | R tips 3 of 3 (10.5-10.9) at ff 2. With `--seed 5.2,2.0`: still 3 of 3, so the hike seed is not it. With `--ankle-ff 2,3`: 7 of 7, R in 2.2, out 3.9-4.9 |
| turnR / turnL | −33.2 / +36.6 deg, clean | −36.4 / +32.6 deg, clean |

The needed A on R depends on direction: 2 deg for most, 3 for sideR.

**`--ankle-learn on`** (`AnkleLearner`; off by default) learns A[side] on the host at step rate.
- At each TS line of that side:
  - tip (in ≥ 7): A += 1.0;
  - out > 4: A −= 0.5 × (out − 4), at most 1.5;
  - out < 2 and in > 3: A += 0.5;
  - otherwise hold.
- A is clamped to 0..4.
- Keep: a clean step (in < 5, out ≤ 4) keeps the A it ran with.
- Revert: a step past 8 deg OUTWARD sets A back to the keep before the rule. Tips (9-13 deg inward) do not
  revert, or two tips in a row could never climb past keep + 1.
- The new A goes to that side's steps not yet uploaded. Use `--ahead 1`: with 2 steps ahead the side's next
  step is often already queued.
- The start comes from `results/ankle_trim_state.json` keyed by direction: seeds 2/2, sideR R 3.
  `--ankle-seed L,R` starts from given values instead.
- The state is saved at the run's end only if the run had no fall, no ABORT and no lost link.
- Each step in the log carries `A_used` / `A_next` next to the firmware's T and hike. The summary row has
  `ankle_learn`. The in-step outward guard stays with `--stop-out 6`.

The A/B (mid-board, from the repo root):
```
# T 3.4, fixed 2,2 vs learned (start 2,2)
python experiments/roll-balance/track_host.py --T 3.4 --dir fwd --steps 8 --ahead 1 --live --note "fwd34 learn off"
python experiments/roll-balance/track_host.py --T 3.4 --dir fwd --steps 8 --ahead 1 --ankle-learn on --ankle-seed 2,2 --live --note "fwd34 learn on"
python experiments/roll-balance/track_host.py --T 3.4 --dir back --steps 8 --ahead 1 --live --note "back34 learn off"
python experiments/roll-balance/track_host.py --T 3.4 --dir back --steps 8 --ahead 1 --ankle-learn on --ankle-seed 2,2 --live --note "back34 learn on"
# sideR at T 4 from seed 2 (it tipped 3 of 3 there), learning on
python experiments/roll-balance/track_host.py --T 4 --dir sideR --steps 6 --ahead 1 --ankle-learn on --ankle-seed 2,2 --live --note "sideR4 learn from 2"
```

## The inset sole: a plant term for the robot's narrow single-support window (2026-11-12 night)

The SIL holds single support down to a shift of ~0.5 × 62 mm. The robot tips inward below ~1.25-1.30 on both
feet. Plant FK on the flight record puts the robot's load-bearing inner limit near the ankle-roll axis: within
the play inside the roll servos (#277), the stance leg pivots there. The mass split cannot move the edge past
k ~0.55.

**The term.** It moves the sole's inner support edge: every sole pad inboard of a line `x` mm from the
ankle-roll axis is moved onto that line, per foot. Outer pads stay put.
- In training it is `--sole-in-mm L=8,R=-12` (both envs; the MjModel is edited before the MJX upload, parity
  R19).
- In grade.py / the SIL / the gate tools it is the `sole_in_L=`, `sole_in_R=` (`sole_in=`) terms.
- The plant's pads reach ~22 mm inboard.

The knob is chosen over a larger ankle play because it is static geometry: the same in CPU and MJX, no new
dynamics, and the referee and grade.py inherit it through config.json / the gate terms.

**Calibration (SIL, the fit plant = grade + per-side play, dwell gait as run on the robot, first R):**

| robot fact (11-12) | plant (22 / 22 mm) | L 8 / R −12 |
|---|---|---|
| march T 5, k R 1.20, ff 0: R tips 4/4 | 0/3 (in ≤ 3.3) | **3/3 (10.4)** |
| the same, ff R 2: R mostly clean | clean | **clean (in ≤ 4.2)** |
| march T 5, k R 1.30: clean | clean | **clean (4.7)** |
| march T 4, k R 1.25: R tips 7/12 (16:3x) | clean | clean (5.0) |
| L needs ≥ 1.3 (B1); L at 1.30 clean | clean at 1.20 | **L edge 1.20-1.25 (1.20: 1/3, 7.7)** |
| fwd T 5, k R 1.30, ff 0: R tips 4/4 | clean | not reproduced (in 4.6) |
| fwd / back T 5, fwd T 4, ff 2,2: clean | clean | **clean** |
| sideR T 4, ff 2,2: R tips 3/3; ff 2,3: clean | clean / clean | not reproduced (2.7) / clean |
| fwd T 3.4, ff 2,2: R tips 4/4, L clean | clean | R not reproduced; L tips 4/5 (wrong side) |
| fwd T 3.4, ff 2,4: R out 5.1-7.0 | out 2.8, then fell outward | out 2.7; L tips 4/5 |

The static edge is reproduced: k R 1.20 tips, 1.30 holds, and ff 2 rescues 1.20 on R; L's edge is at 1.20-1.25.
Not reproduced: the robot's dynamic tips at k 1.30 (forward without ff, sideR, T 3.4) and its outward overshoot
with ff 4. The sim stays ~0.05-0.1 k more forgiving there. Symmetric insets do not work: the plant's left side
already has more ankle play and a calibration zero error, so its L edge moves first (L tips everywhere at
5 / 5 mm). Hence per side: L 8, R −12 (grids at L 10 / 7 and R −8 / −11 / −14 bracket it). Grids: `/tmp` scripts, not kept; the table is
the record.

## Step length, any step, and sequences (`track_host.py`; dry run by default)

A step is a displacement of the swing foot from the stance foot: forward `dx`, left `dy`, heading change `dyaw`.
The `--dir` presets are fixed ones (`fwd` 20 mm, `back` -20, `sideL` / `sideR` ±15, `turnL` / `turnR` ±4 deg asked
≈ ±5 deg of heading); the flags below change them without changing a default.

| flag | meaning |
|---|---|
| `--stride-mm X` | fwd / back step length (default 20) |
| `--side-mm Y` | sideL / sideR step length (default 15) |
| `--step "dx_mm,dy_mm,dyaw_deg"` | one arbitrary step per step instead of `--dir`: diagonal, curved. `dyaw` is the commanded heading change per step; the planner asks `dyaw / 1.34` (the robot's measured heading gain), the default `--yaw-trim -1.4` still cancels its drift |
| `--seq "fwd:4,turnL:3,step(20,10,0):2"` | segments (a preset or `step(dx,dy,dyaw)`, a count) as one continuous track; `--steps` is ignored |
| `--drop-mm D` | crouch depth (default 20) |
| `--swing-tilt DEG` | toe-down tilt of the swing sole, in over the raise, flat at lift-off and touchdown (default 0) |

**Pre-check.** On top of the static shift (pelvis over the stance foot at every lift-off and touchdown) it refuses a
plan whose frames put any leg joint within 2 deg of the plant's joint range (knee, hip pitch / roll / yaw, ankle
pitch and roll), or whose pelvis moves faster in the floor plane than 252 mm/s (161 mm in 1.2 s: the shift the SIL
holds, `dwell.SCHEDULE`), or whose IK has no solution. It prints the tightest joint margin, the pelvis peak speed
and the commanded speed (mm/s forward, mm/s sideways, deg/s; per segment for a sequence).

**What bounds the stride.** The swing leg's ankle pitch. With the 20 mm crouch it stands at -24.6 deg and the swing
foot, raised 22 mm and still one stride behind (fwd) or ahead (back) of the stance foot at the end of the raise, needs
-36.2 deg at a 20 mm stride and about 0.23 deg more per mm; the model's range is ±40 (CAD design range; the
firmware's bench clamp is ±45, `firmware/main/mech_envelope.h`, the robot's measured travel ±78). The pelvis shift
is not the limit: the stance pelvis stays over the stance foot fore-aft in single support and moves one stride
fore-aft inside the same 1.4 s min-jerk shift as the lateral 161 mm, so the speed goes 214 -> 223 mm/s from 20 to 50 mm.

Pre-check and SIL by stride (fwd and back give the same margins; T 4 and T 5 the same pre-check; crouch 20 mm, h 22 mm,
`first R`, k 1.3, ff 2,2, `pelvis-x stance`, the robot's trims; SIL = `sil_gait.run`, 6 steps, fit plant and the same
with the inset sole `sole_in_L=8,sole_in_R=-12`; no extra delay):

| stride | pre-check | tightest joint margin | pelvis peak | SIL T 4 (fit / inset) | SIL T 5 (fit / inset) |
|---|---|---|---|---|---|
| 20 mm | PASS | 3.8 deg (swing ankle pitch) | 214 mm/s | 0 falls, in ≤ 2.8 out ≤ 3.0 / out ≤ 4.0 | 0 falls, in ≤ 3.2 out ≤ 3.4 / out ≤ 4.7 |
| 26 mm | PASS | 2.4 deg | 216 mm/s | 0 falls, in ≤ 3.0 out ≤ 3.1 / out ≤ 4.1 | 0 falls, in ≤ 3.1 out ≤ 3.5 / out ≤ 4.7 |
| 30 mm | REFUSED (ankle pitch) | 1.5 deg | 217 mm/s | 0 falls, in ≤ 3.0 out ≤ 3.1 / out ≤ 4.0 | 0 falls, in ≤ 3.1 out ≤ 3.5 / out ≤ 4.7 |
| 40 mm | REFUSED (ankle pitch) | -0.4 deg | 219 mm/s | 0 falls, in ≤ 3.1 out ≤ 3.1 / out ≤ 4.1 | 0 falls, in ≤ 3.1 out ≤ 3.4 / out ≤ 4.7 |
| 50 mm | REFUSED (ankle pitch) | -2.0 deg | 223 mm/s | not run | not run |

(Largest at 27 mm passes at 2.1 deg; 28 mm is refused at 1.9.) The SIL has no joint limit that bites inside a
plan's 2 deg margin and its plant clips targets at the model range, so its 30 / 40 mm rows are a balance check of
plans the pre-check would not send; the sim travelled 6 steps × stride (117 mm at 20, 237 mm at 40) and stays more
forgiving than the robot.

Where the cap moves (max passing stride, fwd at T 4, from the pre-check; the SIL ran tilt 3 deg at 20 and 40 mm and
crouch 15 mm at 20 and 36 mm, T 4 / 5, fwd / back, fit / inset: 0 falls, 0 aborts, in ≤ 3.3, out ≤ 5.1):

| crouch / swing height / tilt | max stride | limited by |
|---|---|---|
| 20 mm / 22 mm / 0 (default) | 27 mm | swing ankle pitch |
| 20 mm / 15 mm / 0 | 54 mm | swing ankle pitch |
| 20 mm / 22 mm / 3 deg | 43 mm | swing ankle pitch |
| 20 mm / 22 mm / 5 deg | 58 mm | the static shift (pelvis reach) |
| 15 mm / 22 mm / 0 | 36 mm | the static shift (margin 3.2 deg) |
| 10 mm or 5 mm | none | IK out of reach: the straight leg cannot lift the foot |

`--h`'s effect is by the sim's geometry only. The tilt costs toe clearance (SIL minimum swing clearance 11-14 mm
with 3 deg); neither the tilt nor a shallower crouch has run on the robot.

**Sequences.** `--seq` plans every step relative to the stance foot, so a segment's first step takes up the last one's
stagger and hip yaw, and no closing step is added at a boundary. The one exception: when the feet are laterally
apart at the boundary (a lateral segment ending on an odd step) and the next segment reverses the lateral direction,
its first out-step would be a null step, so one explicit `close` step (no displacement, the segment keeps its count)
is added. The final closing step is the gait's own, as for one direction. The heading hold's commanded heading
per step follows the active segment (the reference is the running sum), and the ankle learner keeps one state per
segment key (the preset name, or `step(dx,dy,dyaw)`, in `results/ankle_trim_state.json`), re-planning the
not-uploaded steps with each step's own segment's trim. The per-step prints, the learner lines and the summary rows
name the segment (`2:turnL`). The pre-check covers the whole sequence.

SIL, T 5, k 1.3, fit plant / inset sole, 0 falls, 0 aborts: `fwd:4,turnL:3,fwd:4` (11 steps) in ≤ 3.0 out ≤ 3.4 /
out ≤ 4.7, heading -5 deg; `fwd:4,turnL:3,fwd:4,sideR:4,back:4` (19 steps) the same leans, heading -24 deg (the sim
drifts right with the default trim; the live heading hold is what holds the reference).

```
python track_host.py --T 5 --step "20,10,0" --steps 6                    # diagonal: 4 mm/s forward, 2 mm/s left
python track_host.py --T 5 --seq "fwd:4,turnL:3,fwd:4"                  # one track, no closing step between segments
python track_host.py --T 4 --dir fwd --steps 8 --stride-mm 40 --swing-tilt 3   # longer stride with the toe-down tilt
```

## The step's timeline and the touchdown pause (`--td-pause`, `--settle`, `--lower-frac`, `--land-early-mm`; host only)

**Timeline of one step** (`dwell.Dwell`, s from the step's start; the step's start is the previous step's planned touchdown;
the `timeline:` line of the plan summary prints it for the plan at hand):

| | T 4 | T 5 | what moves |
|---|---|---|---|
| shift | 0 - 1.40 | 0 - 1.40 | pelvis min-jerk over the new stance foot (hip roll, ankle roll, fore-aft with `--pelvis-x stance`); both feet down |
| settle | 1.40 - 1.80 | 1.40 - 1.80 | everything held, both feet down (the lift gate's frame is the last one before lift-off) |
| raise | 1.80 - 2.57 | 1.80 - 2.92 | swing sole up to h over 0.35 of the single support; the stance ankle ff and the swing tilt ramp IN with its height |
| move | 2.57 - 3.23 | 2.92 - 3.88 | sole at h, forward / sideways over 0.30; the stance and swing hip yaw |
| lower | 3.23 - 4.00 | 3.88 - 5.00 | sole down over 0.35, min-jerk; the ankle ff and the swing tilt ramp OUT with its height (both are 0 at 4.00) |

The single support is 2.2 s at T 4 and 3.2 s at T 5; the shift and settle are the same 1.4 + 0.4 s (`dwell.SCHEDULE`).

**Where the next shift begins.** In the plan, at the planned touchdown, with no overlap: the next step's shift is
`_mj((t - touchdown) / 1.4)` from the touchdown on, and the foot's lowering, the ankle ff and the swing tilt are all
functions of the sole height, which is 0 at the touchdown. Measured on the T 4 / T 5 plans (stance hip roll goal of the next
step): rate 0.5 deg/s at the touchdown, 3.3 deg/s at +0.1 s, 9.4 at +0.2, 17.4 at +0.3, 32.9 at +0.5, peak 39.5 deg/s at +0.7 s.
Before the touchdown the goal moves only 2 deg/s (T 4, at -0.5 s; 0.6 at -0.1 s: the ankle ff ramping out through the leg
geometry). The sole is within 1 mm of the floor from about 0.12 s (T 4) / 0.20 s (T 5) before the touchdown and within 3 mm from 0.20 s / 0.34 s.
So **the 0.34 s overlap is not in the plan.** What does overlap the shift on the robot is the firmware: at the touchdown the
support state goes to double support and the stance trim FADES, linearly to 0 over 0.6 s (`balance/stance_trim.h`, `fade_s`),
so the hike correction of the stance foot is taken out while the shift starts; that is on top of the plan's goals, and the
servos' lag moves the goals' effect later, not earlier. Where the 0.34 s and the 36 deg/s at +0.1-0.3 s of the flight record
come from is not explained by the plan: its goal rate there is 3-17 deg/s.

**`--td-pause S`** (default 0): the next step's shift starts S s after the touchdown. All of the shift's motion is held
(`pel`, fore-aft, feet; the pose is the touchdown frame's, which has the ankle ff and the tilt out), then the old shift
runs, S later. Every step but the first and the final shift back to the centre have it, so a step is S longer and the walk
`n S` longer. **`--settle S2`** replaces the pre-lift settle (0.4 s); the shift and the single support keep their lengths, so
`--td-pause 0.3 --settle 0.2` is 4.1 s per step at T 4. The step time is printed (`timeline:`) and used for `--ahead`, the
commanded speeds, `--heading-rate` and `--cmd` (a command's displacement is v x the step time); `T` stays the schedule's key.

**`--lower-frac F`** (0.20 to 0.50, default 0.35) or **`--lower-s S3`** (= F x the single support): the lowering's share of the
single support. The move takes what the raise (0.35) and the lowering leave, the step time does not change, the phase the
firmware sees is warped piecewise so its raise end (0.35) and lowering start (0.65) still land on the plan's.
**`--land-early-mm D`** (0 to 4, default 0): the swing sole is commanded D mm BELOW the floor at the touchdown (the same
min-jerk descent, `z = h (1 - v) - D v`, so it crosses the floor early), held through the pause and let up (min-jerk) over the
next shift, so the lift-off pose is the plain one. The pre-check judges every frame, so the joint margins cover it (tested to 4 mm).
Not run on the robot: the press loads the foot the way the stepper's sole bias does, but how far the servos hold a goal below the floor is unmeasured.

**A/B on the robot** (fixed walks, T 4, `--ahead 1`; `walk` = forward 8 at 40 mm, `back` = backward 8 at 32 mm):

```
F="--T 4 --ahead 1 --swing-tilt 3 --live"; W="--dir fwd --steps 8 --stride-mm 40"; B="--dir back --steps 8 --stride-mm 32"
python experiments/roll-balance/track_host.py $F $W                                   --note "td base fwd"
python experiments/roll-balance/track_host.py $F $W --td-pause 0.3                    --note "td pause0.3 fwd"
python experiments/roll-balance/track_host.py $F $W --td-pause 0.3 --settle 0.2       --note "td pause0.3 settle0.2 fwd"
python experiments/roll-balance/track_host.py $F $W --land-early-mm 2                 --note "td land2 fwd"
python experiments/roll-balance/track_host.py $F $B                                   --note "td base back"
python experiments/roll-balance/track_host.py $F $B --td-pause 0.3                    --note "td pause0.3 back"
python experiments/roll-balance/track_host.py $F $B --td-pause 0.3 --settle 0.2       --note "td pause0.3 settle0.2 back"
python experiments/roll-balance/track_host.py $F $B --land-early-mm 2                 --note "td land2 back"
```

Without `--live` each prints the plan (the timeline, the pre-check) and sends nothing.

**On the robot (2026-11-16, fixed and `--cmd` walks at T 4, from the coordinator's runs).** Forward / backward fixed walks and a mixed
steered walk with `--td-pause 0.3`: about 144 steps clean; the weight-transfer kick on the shift onto the left foot (the touchdown
rebound) dropped from median / p90 / max 1.0 / 3.1 / 3.5 deg (baseline, 16 shifts) to 0.6 / 0.8 / 0.9 (16 shifts) and 0.7 / 0.9 / 1.0 (13
shifts, steered); headings tighter. Sidesteps: an early reading that the pause made lateral steps under-travel or go the wrong way was
not borne out. An alternating A/B/A/B with the tracker (positions at rest, the robot about 40 deg off square, six steps of +-15 mm each)
under-travelled with and without the pause (right legs 15 / 30 / 33 mm, left legs about 0 / -20 / -13 mm of about 80 mm commanded, +-40-80 mm
of fore-aft drift, no tips): sidesteps under-travel on the robot with and without `--td-pause`, cause unknown. The plan and the
uploaded frames are identical in placement with the pause (tests, and the frames of a robot run decoded against the plan).

## A quick single support: explicit phase times (`--ss-s`, `--shift-s`, `--raise-s`, `--move-s`, `--lower-s`, `--allow-under`; host only)

The step is `[td-pause] + shift + settle + single support`. Today's T-based `dwell.SCHEDULE` stays the default; giving
any of `--ss-s S` (the single support), `--shift-s S` (the lateral shift), `--raise-s` / `--move-s` / `--lower-s`
(the three parts of the single support) lays the step out from the given values, and `--settle S` and `--td-pause S` add
to it as above. `--T` is then not the step time: it only supplies the shift (1.4 s) and settle (0.4 s) that were not given
(`track_host.py` prints a note, the `timeline:` line and the effective step time, and the commanded speeds use it). The parts not
given split what is left in today's proportions (raise 0.35 : move 0.30 : lower 0.35); with the move not given, the move
takes what the given raise / lowering leave and the other keeps its 0.35 share; all three given and no `--ss-s`: the
single support is their sum. A raise or lowering under 0.10 of the single support, or a move under 0.08, is refused (the
keyframes are 100 ms apart: a raise of 0.21 s is two keyframes and the min-jerk profile is sampled that coarsely, which is
what the firmware interpolates between). The firmware's phase (raise end at 0.35, lowering start at 0.65 of the single
support) is warped piecewise onto the plan's, as for `--lower-frac`.

**Guards.** A single support under 1.8 s, a shift under 1.4 s or a step under the SIL floor (3 s) is no longer refused when
it was given explicitly: a `WARNING:` line is printed instead (without an explicit phase time the T floor still refuses).
The pre-check's other rules stay refusals: IK reach, every joint 2 deg inside its range, the pelvis speed in the shift (<= 252
mm/s: a shift of 1.0 s at k 1.3 is 260 mm/s and is refused; 1.2 s passes), and a new one, the swing leg's joint speed.

**Swing joint speed.** The peak |d q / d t| of the swing leg's six joints between consecutive keyframes over every single
support must stay under 0.8 x the STS3215's no-load speed: 4.04 rad/s at 12 V, measured (`tools/measure_servo_speed.py`,
`docs/servo-map.md`, `docs/training.md`) = 231 deg/s, so 185 deg/s. The plans of today (T 4, T 3.4, fwd 40 mm with tilt 3)
peak at 72 and 98 deg/s (R knee); fwd 20 mm at `--ss-s` 2.2 / 1.2 / 0.8 / 0.6: 67 / 113 / 162 / 148 deg/s (the
0.6 s plan is lower than the 0.8 s one because its raise is two keyframes: the coarse sampling hides part of the peak
the servo would be asked for, so read the 0.6 s figure as a lower bound). The plan summary prints the peak.

**`--allow-under`.** The static-shift pre-check does not refuse a short `--amp` today: its lower bound is the plan's OWN k (a plan at
`--amp 1.15,1.15` passes at 1.15 x), and the robot's requirement (L 1.30, R 1.30) is only the upper bound's base. So
`--amp 1.15,1.15` runs without the flag. What the flag changes: a lift-off or touchdown whose pelvis is short of the plan's own k
(a timing that cannot reach it) is a warning, not a refusal; and the pre-check prints a `WARNING:` line whenever the plan's k is under
the robot's requirement, saying whether `--allow-under` was given. IK, joint range, pelvis speed and swing joint speed stay refusals.

**SIL** (fit plant and inset-sole plant `sole_in_L=8,sole_in_R=-12`; six steps of march / fwd 20 mm, no swing tilt, T 4's
shift 1.4 and settle 0.4, k 1.3 unless stated, lift gate on; ankle ff 2,2, `--pelvis-x stance`; `sil_gait.run`; `(a)` the single
support at k 1.3, `(b)` k at the two shortest single supports). "clean" = no fall, the 7 lifts of 6 steps plus the closing
step; "gate timeout" = the firmware's lift gate waited 3 s for a level, quiet body and ended the track with both feet down
(a stop, not a fall); clearance = the swing sole's peak height per step.

(a)

| plant | walk | k | single support | result | lean in / out max (deg) | swing clearance min / mean (mm) |
|---|---|---|---|---|---|---|
| fit | march | 1.3 | 2.2 s | clean | 2.8 / 2.8 | 14.2 / 18.7 |
| fit | march | 1.3 | 1.2 s | gate timeout, track ended (2/6) | 3.1 / 2.9 | 4.3 / 14.3 |
| fit | march | 1.3 | 0.8 s | clean | 3.0 / 3.5 | 16.4 / 21.0 |
| fit | march | 1.3 | 0.6 s | gate timeout, track ended (2/6) | 3.1 / 4.4 | 3.1 / 16.8 |
| fit | fwd20 | 1.3 | 2.2 s | clean | 2.7 / 2.8 | 14.5 / 18.7 |
| fit | fwd20 | 1.3 | 1.2 s | gate timeout, track ended (2/6) | 3.1 / 2.9 | 5.8 / 14.5 |
| fit | fwd20 | 1.3 | 0.8 s | gate timeout, track ended (2/6) | 3.0 / 3.5 | 4.7 / 16.3 |
| fit | fwd20 | 1.3 | 0.6 s | FELL out | 3.1 / 4.4 | 17.7 / 23.6 |
| inset | march | 1.3 | 2.2 s | clean | 2.9 / 3.8 | 15.7 / 18.0 |
| inset | march | 1.3 | 1.2 s | clean | 9.5 / 0.9 | 13.6 / 15.5 |
| inset | march | 1.3 | 0.8 s | FELL in | 3.5 / 2.0 | 20.3 / 20.3 |
| inset | march | 1.3 | 0.6 s | gate timeout, track ended (2/6) | 4.2 / 3.3 | 2.5 / 13.3 |
| inset | fwd20 | 1.3 | 2.2 s | clean | 2.8 / 3.8 | 15.2 / 17.7 |
| inset | fwd20 | 1.3 | 1.2 s | clean | 10.0 / 0.9 | 13.2 / 15.5 |
| inset | fwd20 | 1.3 | 0.8 s | FELL in | 3.5 / 2.0 | 20.4 / 20.4 |
| inset | fwd20 | 1.3 | 0.6 s | FELL in | 5.9 / 3.3 | 12.2 / 17.1 |

(b)

| plant | walk | k | single support | result | lean in / out max (deg) | swing clearance min / mean (mm) |
|---|---|---|---|---|---|---|
| fit | march | 1.3 | 0.8 s | clean | 3.0 / 3.5 | 16.4 / 21.0 |
| fit | march | 1.15 | 0.8 s | clean | 3.1 / 3.8 | 17.4 / 21.1 |
| fit | march | 1 | 0.8 s | clean | 3.2 / 3.9 | 18.2 / 24.3 |
| fit | march | 0.85 | 0.8 s | clean | 3.3 / 3.3 | 18.6 / 24.8 |
| fit | march | 0.7 | 0.8 s | clean | 10.4 / 2.3 | 17.2 / 25.1 |
| fit | march | 1.3 | 0.6 s | gate timeout, track ended (2/6) | 3.1 / 4.4 | 3.1 / 16.8 |
| fit | march | 1.15 | 0.6 s | clean | 3.3 / 4.7 | 18.8 / 22.2 |
| fit | march | 1 | 0.6 s | clean | 3.6 / 4.5 | 20.1 / 24.1 |
| fit | march | 0.85 | 0.6 s | clean | 3.9 / 4.2 | 20.6 / 26.5 |
| fit | march | 0.7 | 0.6 s | clean | 10.6 / 2.7 | 17.1 / 23.8 |
| fit | fwd20 | 1.3 | 0.8 s | gate timeout, track ended (2/6) | 3.0 / 3.5 | 4.7 / 16.3 |
| fit | fwd20 | 1.15 | 0.8 s | clean | 3.1 / 3.8 | 16.9 / 20.7 |
| fit | fwd20 | 1 | 0.8 s | clean | 3.1 / 3.9 | 17.6 / 24.1 |
| fit | fwd20 | 0.85 | 0.8 s | clean | 4.0 / 3.3 | 18.5 / 24.6 |
| fit | fwd20 | 0.7 | 0.8 s | clean | 10.5 / 2.4 | 18.9 / 25.4 |
| fit | fwd20 | 1.3 | 0.6 s | FELL out | 3.1 / 4.4 | 17.7 / 23.6 |
| fit | fwd20 | 1.15 | 0.6 s | clean | 3.3 / 4.7 | 19.0 / 22.3 |
| fit | fwd20 | 1 | 0.6 s | clean | 3.5 / 4.5 | 19.6 / 22.9 |
| fit | fwd20 | 0.85 | 0.6 s | clean | 3.7 / 4.2 | 20.1 / 26.2 |
| fit | fwd20 | 0.7 | 0.6 s | clean | 7.1 / 2.6 | 17.9 / 25.6 |
| inset | march | 1.3 | 0.8 s | FELL in | 3.5 / 2.0 | 20.3 / 20.3 |
| inset | march | 1.15 | 0.8 s | FELL in | 9.9 / 2.6 | 9.0 / 17.6 |
| inset | march | 1 | 0.8 s | FELL out | 6.4 / 2.9 | 27.6 / 27.6 |
| inset | march | 0.85 | 0.8 s | FELL in | 11.4 / 2.5 | 8.3 / 17.4 |
| inset | march | 0.7 | 0.8 s | clean | 12.5 / 0.0 | 8.9 / 15.6 |
| inset | march | 1.3 | 0.6 s | gate timeout, track ended (2/6) | 4.2 / 3.3 | 2.5 / 13.3 |
| inset | march | 1.15 | 0.6 s | FELL out | 5.0 / 3.7 | 15.0 / 22.0 |
| inset | march | 1 | 0.6 s | FELL out | 2.7 / 3.9 | 29.7 / 29.7 |
| inset | march | 0.85 | 0.6 s | FELL out | 6.8 / 3.3 | 28.4 / 28.4 |
| inset | march | 0.7 | 0.6 s | FELL in | 12.0 / 0.5 | 8.8 / 15.1 |
| inset | fwd20 | 1.3 | 0.8 s | FELL in | 3.5 / 2.0 | 20.4 / 20.4 |
| inset | fwd20 | 1.15 | 0.8 s | clean | 4.3 / 2.6 | 12.3 / 20.3 |
| inset | fwd20 | 1 | 0.8 s | FELL out | 6.3 / 2.9 | 27.6 / 27.6 |
| inset | fwd20 | 0.85 | 0.8 s | FELL in | 11.6 / 2.5 | 7.1 / 16.8 |
| inset | fwd20 | 0.7 | 0.8 s | clean | 12.8 / 0.3 | 8.8 / 15.0 |
| inset | fwd20 | 1.3 | 0.6 s | FELL in | 5.9 / 3.3 | 12.2 / 17.1 |
| inset | fwd20 | 1.15 | 0.6 s | FELL out | 4.9 / 3.7 | 14.8 / 21.2 |
| inset | fwd20 | 1 | 0.6 s | FELL in | 11.0 / 3.9 | 11.9 / 20.8 |
| inset | fwd20 | 0.85 | 0.6 s | FELL out | 6.8 / 3.3 | 28.5 / 28.5 |
| inset | fwd20 | 0.7 | 0.6 s | FELL in | 12.5 / 0.5 | 8.7 / 15.0 |

**Where the sim stops being credible.** It is known to be far more forgiving than the robot in single support, and these numbers show
it is also not smooth: on the fit plant k 1.3 gate-times-out at 1.2 and 0.6 s but runs clean at 0.8 s, and k 1.15 / 1.0 / 0.85 run clean at
0.6 s where k 1.3 does not; on the inset plant k 1.3 falls at 0.8 s but runs at 1.2 s, and every k at 0.6 s falls or ends. With six steps per cell and one
seed these are single draws of a chaotic system, not a trend. What they do say: no short single support is a fall everywhere (the fit plant is clean
in 19 of 22 cells at k >= 0.85), the lean stays under 5 deg except at k 0.7 and on the inset plant, and a clearance of 15-20 mm at
0.6 s means the 100 ms keyframes still lift the foot. What they cannot say: whether the real single support survives a lift with the
pelvis at 0.85-1.15 x (on the robot a lift at 1.15 x tipped in; README 'k R 1.35...'), how the servos follow a 0.2 s raise (the swing speed check is the
only model of that), or any friction / sole effect. Treat the sim as a filter for plans that cannot work, not as evidence that one will.

**Robot test lines** (fixed walks, T 4, `--ahead 1`, lift gate on (the default); part 1 at k 1.3, part 2 at the shortest single support of part 1):

```
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 1.2 --live --note "ss 1.2 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --ss-s 1.2 --live --note "ss 1.2 fwd20"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 0.8 --live --note "ss 0.8 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --ss-s 0.8 --live --note "ss 0.8 fwd20"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 0.6 --live --note "ss 0.6 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --ss-s 0.6 --live --note "ss 0.6 fwd20"

python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 0.6 --amp 1.15,1.15 --allow-under --live --note "ss 0.6 k 1.15 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --ss-s 0.6 --amp 1.15,1.15 --allow-under --live --note "ss 0.6 k 1.15 fwd20"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 0.6 --amp 1.0,1.0 --allow-under --live --note "ss 0.6 k 1.0 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --swing-tilt 3 --ss-s 0.6 --amp 1.0,1.0 --allow-under --live --note "ss 0.6 k 1.0 fwd20"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 0.6 --amp 0.85,0.85 --allow-under --live --note "ss 0.6 k 0.85 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --swing-tilt 3 --ss-s 0.6 --amp 0.85,0.85 --allow-under --live --note "ss 0.6 k 0.85 fwd20"
```

Without `--live` each prints the note, the timeline, the warnings and the pre-check and sends nothing.
The two fwd lines at k 1.0 and 0.85 carry `--swing-tilt 3`: without it the plan is REFUSED (the swing foot's ankle pitch reaches -38.6 /
-39.5 deg, outside the plant's 38 deg less 2 deg margin: with the pelvis nearer the centre the swing foot has more reach to cover). With it the
margin is 4.4 / 3.4 deg. At k 1.15 no tilt is needed (2.5 deg).

**On the robot (2026-11-16/17, march, T 4).** `--ss-s 1.2` (full shift, normal settle): FELL on step 2 (R stance). The flight record shows no
ramp or scaling fault in the plan: after foot-off the body leans about 2.4 deg toward the swing foot by +0.4 s and in a normal step recovers by
0.6-0.8 s; the short step lowered at 0.8 s while still -4.2 deg, landed hard, rebounded and toppled outward. Only the raise shortened
(`--raise-s 0.42 --move-s 0.66 --lower-s 0.77`): no fall, but 3 of 7 steps tipped 11-12 deg inward (L 11.1, R 11.7, L 12.2): the fast lift
is itself the disturbance, on both sides. So shortening the single support or the raise tips or topples the robot as it is; the options stay
for experiments (they leave the default frames untouched). The SIL tables above did not predict this (the sim is the forgiving one).

## Taking up the hip-roll play before the foot leaves: `--pre-hike L,R`, `--pre-hike-frac F` (host only; default off)

Why (the flight records of 2026-11-17, from the coordinator): with `--ss-s 1.2` the first R-stance step fell, and with only the raise
shortened (`--raise-s 0.42 --move-s 0.66 --lower-s 0.77`) 3 of 7 steps tipped 11-12 deg inward on both sides: a fast lift is the
disturbance. The hypothesis under test: at foot-off the pelvis sags through the hip-roll play (6-8 deg); with a slow raise the firmware
trim's hike (learned T ramped over the raise plus the closed loop) keeps up, with a fast one the sag is abrupt. `--pre-hike` takes the
play up in the plan, with both feet down, before the lift.

**What it does.** For each step, the stance leg's hip-roll goal gets `+ theta` (the firmware trim's convention: + raises the swing-side
hip; `Kin.pose(hike=)`), min-jerk in over the settle (over `--pre-hike-frac` of it, default all of it: 0.4 s at T 4), held through
the single support, and out over the start of the next step: over `--td-pause` if there is one (the removal then happens with nothing
else moving), else over the first half of the next shift (0.7 s at T 4; the shift itself is slow at its start). `theta` is the side's
value of `--pre-hike L,R` (deg, 0 to 12; 0 = no hike on that side). The seeds to try are the trims the robot learns, `5.2,3.4`. A pre-hike
step's ramp-in and the previous step's ramp-out never overlap (the out ramp is at most as long as the pause plus the shift).

**The swing sole is kept where the plan has it.** A hike alone (the stance hip-roll goal += theta, the swing leg's goals unchanged) leaves the
stance leg where it is in the world and rolls the pelvis about the stance hip's roll axis; the swing-side hip rises, and the swing sole with
it: by the swing ankle's lateral distance from the stance hip x sin(theta), 12.5 mm at 5.2 deg with the pelvis 40 mm over the stance
foot (measured on the plant's forward kinematics, `tests/test_pre_hike.py`). With both feet down in the settle that unloads the swing foot,
and in the single support it moves the foot with the pelvis. `dwell.hiked_pose` therefore solves the swing leg's IK for the rolled pelvis
with the sole's world position fixed, and turns its ankle roll back by the pelvis roll, so the sole stays level and at the height the plan
commands (0 on the floor, h in the air); the stance leg is the level solution with the hip roll goal += theta (its other joints and its
ankle roll are unchanged: the leg does not move). Checked on the MuJoCo plant: both ankle-roll points land on their targets to 1e-6 m and
both soles are level for hikes of 0, 5.2 and 9 deg on either stance and the swing foot at 0 and 15 mm. Cost: the swing leg's joints move
a few degrees more (its peak joint speed in the swing goes from 86-92 to 156-167 deg/s at `--ss-s 1.2`, under the 185 deg/s limit).

**The firmware's trim.** The plan carries the hike, so the host seeds the firmware's learned trim T of a side with a pre-hike to 0
(`trim T L 0` instead of `trim T L 5.2`; `track_host.py` prints it) and the two do not add; the closed loop stays on and corrects the
residual, and its learned T then grows from 0 on its own (the `TS` lines' `T` shows how much it still needs: a `T` near 0 at mid single
support means the plan's hike was the right size, a `T` near the old 5.2 / 3.4 means it did nothing). The pre-check reads the lift-off and
touchdown pelvis shift through the hike (the stance hip-roll goal less theta), so its numbers are the same as without; the joint-range rule
covers the stance hip roll and the swing leg (the swing leg's IK is solved, so a swing foot out of reach refuses the plan).

**Robot test lines** (fixed walks, T 4, `--ahead 1`, lift gate on; compare the TS lean-in / lean-out and T against the same line without `--pre-hike`):

```
# (1) normal timing, no regression
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --pre-hike 5.2,3.4 --live --note "prehike march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --pre-hike 5.2,3.4 --live --note "prehike fwd20"
# (2) the same with only the raise shortened
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --raise-s 0.42 --move-s 0.66 --lower-s 0.77 --pre-hike 5.2,3.4 --live --note "prehike raise0.42 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --raise-s 0.42 --move-s 0.66 --lower-s 0.77 --pre-hike 5.2,3.4 --live --note "prehike raise0.42 fwd20"
# (3) with the short single support
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir march --steps 6 --ss-s 1.2 --pre-hike 5.2,3.4 --live --note "prehike ss1.2 march"
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --dir fwd --steps 6 --stride-mm 20 --ss-s 1.2 --pre-hike 5.2,3.4 --live --note "prehike ss1.2 fwd20"
```

Variants: `--pre-hike-frac 0.5` finishes the ramp in the first half of the settle; `--settle 0.8` gives the ramp more time (the step is 0.4 s longer);
`--pre-hike 5.2,0` hikes one side only. Without `--live` each prints the plan and sends nothing.

**On the robot (tested 2026-11-16): no benefit for the quick lift; it conflicts with the lift gate level and the trim's reference.**
`--pre-hike 5.2,3.4` at normal timing: clean 7/7 (lean in <= 4.1 deg), but (a) the firmware lift gate TIMES OUT on the roughly 4.3 deg pre-tilt
at its default level of 3 deg (the run needed `liftgate set 6 15 150 3000`, restored afterwards; `--gate 6,15,150,3000` sends it): the gate holds
when the roll is more than its level off the settled double-support roll (`lift_gate.h`); why that reference does not follow the planned
pre-tilt was not established; and (b) the firmware trim takes its roll
reference at lift-off, that is the pre-tilted posture, so its learned T climbs back (L 1.2 -> 5.2, R 1.3 -> 3.3) on top of the plan's hike
instead of staying near 0. With the quick lift (`--raise-s 0.42 ...`) it did not help: still 3 of 7 steps tipped 10.7-11.4 deg inward. The
hypothesis that the pelvis sags through the hip-roll play at a fast foot-off, and that taking the play up beforehand lets the trim keep up,
is not supported by this test; the option stays default-off.

## Live velocity commands (`track_host.py --cmd`, `cmd_walk.py`, `walk_keys.py`; dry run by default)

A walk with no fixed length: every step the host reads the CURRENT command, `vx vy wz` (mm/s forward, mm/s left, deg/s
left), and plans the next not-yet-uploaded step from it. The step's displacement is the command x T (`dx = vx T`,
`dyaw = wz T`; the sideways one is `dy = 2 vy T`, below), clamped, slewed and pre-checked before one frame of it is uploaded; the rolling upload, the
heading hold, the stop rules (`--stop-lean`, `--stop-out`, the lift-gate timeout) and the lift gate are the ones above.
At T 4: 5 mm/s = 20 mm per step, 10 mm/s = 40 mm per step, 1.25 deg/s = 5 deg per step.

**A sidestep is a PAIR of steps.** The planner's sideways step is "the leading foot out by `dy`, then the trailing foot closes
to the stance foot" (`dwell.Dwell._goals`), so the BODY moves `dy` per two steps, whichever feet lead. `vy` is the body's
velocity (the delivered one), so the lead step carries `dy = 2 vy T`, the close step the same value (the geometry ignores it:
a step that is not the leading side lands alongside the stance foot), and the body moves `vy T` per step on average. The
`--max-side-mm` cap (default 15, up to 30: the pre-check passes fixed sidesteps to 30 mm at T 4, the pelvis speed exactly at its
limit at 30, 25 has margin) is on `dy`, so the fastest sideways body velocity is cap / (2 T): 1.9 mm/s at 15 mm and T 4, 3.1 mm/s
at 25. The dry run's printed `speed (commanded ...) sideways` and the `applied` fields of the status are body velocities.
(Before this a sideways command was delivered at half its value.)

**Sources.** `--cmd file:PATH`: a small text file, re-read every step; its first line is `vx vy wz` (floats, spaces or
commas, `#` comments) or the word `stop` / `pause`. A missing, empty or unparseable file keeps the last command (the
host prints what it applied each step; the first read must succeed or nothing is sent). `--cmd stdin`: the same lines on
standard input, read without blocking (the newest parseable line wins; end of input = stop). `stop`, end of input,
Ctrl-C and SIGTERM all end the walk the same way: the next step is the closing step, then the firmware rests. A second
Ctrl-C halts (`track halt`). `--max-steps N` (default 120) and `--max-time S` (default 600; 0 = none) end it the same way.
`--status PATH` (default `PATH.status` for `file:PATH`) is rewritten every step with one JSON line: the state, what was
asked, what was applied (mm/s, mm/s, deg/s), the per-step displacement, the notes (what scaled it).

**Envelope** (all checked before a step is uploaded; flags override):

| per step | default | flag | why |
|---|---|---|---|
| forward dx | 50 mm | `--max-fwd-mm` | hw 2026-11-16, T 4, tilt 5 deg (auto above 43 mm): three fixed 50 mm walks 7/7 clean each and one steered walk through 50 mm, lean <= 3.8 deg |
| backward dx | 32 mm | `--max-back-mm` | steering mode, hw 2026-11-16: after sidesteps right, a backward command of -10 mm/s (40 mm) ramps dx -20, -40, -40, and on the third backward step (left stance) the robot tipped 13.8 deg inward once and fell once (the step before was clean, in 1.7 out 2.2) in two of three runs of that pattern; at -8 mm/s (32 mm) the same kind of pattern ran clean, including a 43-step walk. 40 mm backward is proven only as a FIXED walk (8 clean at T 4, tilt 3, with the lift gate) and stays available with `--max-back-mm 40` |
| sideways dy (lead step; the body moves it per PAIR of steps) | 15 mm | `--max-side-mm` (up to 30) | proven side step |
| heading dyaw | 5 deg | `--max-turn-deg` | proven (turns ~5 deg per step) |
| slew, change of (dx, dy, dyaw) from one step to the next | 20 mm, 10 mm, 3 deg | `--slew DX,DY,DYAW` | below |
| stance ankle ff on a sideways step, first-step boost, heading drift by step type | 2,3 deg; +1 deg; 0.94,1.9 deg/step | `--ankle-ff-side`, `--first-ankle-boost`, `--heading-drift-side` | [By step type](#by-step-type-hw-2026-11-16) below |
| swing tilt | from the larger of this and the last step's \|dx\|: 0 to 26 mm, 3 deg to 43 mm, 5 deg beyond | `--swing-tilt X` fixes it | the swing foot starts at -dx of the last step, so that sets the reach; the table above has the caps (27 mm without tilt, 43 with 3 deg) |
| pre-check | static-shift window (k 1.30 to 1.35 over the stance foot at lift-off and touchdown), every leg joint 2 deg inside its range, IK reachable, pelvis speed <= 252 mm/s | | the whole-plan pre-check, run on the one step |

A command past the envelope is scaled by ONE factor in vx, vy and wz (the direction of the command is kept; the
scaling is printed). The slew then moves the step from the last one toward that point only as far as the three change
limits allow (again one factor). Then the pre-check judges the step itself (`precheck(only_step=)`, on a two-step window:
a step's frames depend on the previous step only, through where the swing foot starts and the hip yaw it left); if it
fails, the largest fraction of the way from the last step that passes is used (from zero if the last step itself no
longer passes; none at all ends the walk). A step that fails is never uploaded.

**Why a slew of 20 mm / 10 mm / 3 deg.** In step j the swing foot starts at -dx(j-1) from the stance foot and lands at
+dx(j): it travels dx(j) + dx(j-1), and the double support after it has the feet dx(j) apart. A steady 50 mm walk
travels 100 mm per step (proven); the same-size reversal 50 -> -40 travels only 10 mm, so the swing foot's reach is not the
danger. What a jump changes is the stagger the next shift and touchdown start from (50 mm one way to 40 mm the other in
one step: the feet and the pelvis (it moves dx(j-1) fore-aft in the shift) change direction between two steps with no
step in which both feet are squared). 20 mm is under half the stride caps, so full forward to full backward takes five steps (the march step,
feet squared, is passed when the steps are 20 mm apart: 40, 20, 0, -20, -40; from 50: 30, 10, -10, -30, -40). Sideways 10 mm (two steps from 15 to -5) and
turn 3 deg (the hip yaw swing 2c is c = dyaw / 1.34 / 2: a 5 -> -5 jump reverses the asked turn by 7.5 deg between two steps). These are
a proposal from the geometry and the SIL, not measured on the robot.

**Reversal.** The sideways rule of `--seq` (feet laterally apart and the new direction opposite to the gap: one `close`
step) fires after EVERY odd step of a sideways walk that goes on (the foot that followed lands alongside), which would
turn it into out, close, out, close; `--cmd` applies it to the swing foot's lateral travel instead: when the next step's
swing foot would travel AGAINST the commanded direction, one closing step (no displacement) comes first and the command
waits a step. A full reversal (15 mm right after 15 mm left) is a null out-step, not a wrong-way one, and needs none.
`expand_seq` is unchanged.

**Heading.** (The feed-forward the hold cancels depends on the step type: below.) With wz = 0 the reference heading is held as before; with wz != 0 the reference advances by the applied
dyaw (wz T, after the clamps) per step, the same loop as `--heading-rate`. The ankle learner stays off unless
`--ankle-learn on` (then one state, key `cmd`, in `results/ankle_trim_state.json`).

**By step type (hw 2026-11-16).** Three feed-forwards that depend on the step, on by default in `--cmd`, off elsewhere
(`--seq` / `--step` / `--dir` plans stay byte-identical unless the flag is given; `--ankle-ff-side 2,2`,
`--first-ankle-boost 0`, `--heading-drift-side 1.9,1.9` restore the old `--cmd` plans):

1. `--ankle-ff-side L,R` (`--cmd` default 2,3). On a step with dy < 0 the RIGHT stance ankle gets
   `(R - 2) x min(1, |dy| / 15 mm)` deg more than its `--ankle-ff` (or the learner's) value; on dy > 0 the LEFT stance gets
   `(L - 2) x min(1, |dy| / 15 mm)`. The default adds 1 deg on the right stance of a full right step (2 -> 3) and nothing
   elsewhere. Evidence: sideways RIGHT at 15 mm tips the right stance inward (~11 deg, benign) at ff 2,2: 3 of 3 on
   11-12, 1 of 3 on 11-16 in `--cmd` mode; clean 7/7 with `--ankle-ff 2,3` on 11-12; sideways left is clean at 2,2.
   The learner is not touched: it keeps learning from its own list (`args["ankle_steps"]`), the extra is added in the plan
   only. The pre-check's joint rule judges the frames, so the ankle-roll range (+-23 deg less 2) covers it (tested).
2. `--first-ankle-boost DEG` (`--cmd` default 1, 0 elsewhere). DEG is added to the first stance step on each side (steps 1
   and 2 of the walk). Provisional: the first right-stance step tips inward more than later ones (3 of 12 first R steps
   vs 1 of 29 later ones at T >= 4 on 11-12; 13.2 deg on a 3.4 s march on 11-16). The size, 1 deg, is the step that
   cured the right stance's sideways tip (2 -> 3), taken here as the first-step amount: a guess, not a fit. The left
   first step has no evidence of tipping (the numbers above are right-stance only), it gets the same 1 deg because the
   option is per walk, not per side. The first R step of a right walk runs 2 + 1 (side) + 1 (boost) = 4 deg, the top of
   the learner's range (4 on R at T 3.4 leaned outward 5-8 deg, but that was every step at T 3.4, not the first step
   at T 4; `--stop-out` still guards it). Not run on the robot; to be re-fitted from `--cmd` runs.
3. `--heading-drift-side L,R` (`--cmd` default 0.94,1.9). The heading model is heading per step = 1.34 x asked + drift,
   drift = 1.9 deg/step to the left on forward walking (the feed-forward asked = -drift / 1.34 = -1.42). Per step the
   drift is `(1 - f) x 1.9 + f x side` with the lateral share `f = |dy| / (|dx| + |dy|)` and `side` = L for dy > 0, R for
   dy < 0; a step in place has 1.9. The hold's feed-forward, its feedback and `--yaw-trim` (scaled by drift / 1.9) all
   follow it.

   Derivation of the drift per segment, from the logs of 2026-11-16 (the `heading` rows of the summary: the fused yaw
   after each single support, and the asked turn the hold wrote for the step; per segment
   `drift = (sum of d psi - 1.34 x sum of asked) / steps`, `ff = -drift / 1.34`):

   | run (log) | segment, steps | n | sum d psi | sum asked | drift /step | ff asked |
   |---|---|---|---|---|---|---|
   | seq a (T092701) | fwd 3-4 | 2 | +2.9 | -1.50 | +2.46 | -1.83 |
   | seq b (T092858) | fwd 3-4 | 2 | +1.2 | -2.57 | +2.32 | -1.73 |
   | cmd a, b, c, d | fwd, 4-10 / 4-6 / 4-7 / 4-10 | 21 | | | +1.99 / +1.48 / +1.77 / +1.20 | -1.49 / -1.10 / -1.32 / -0.90 |
   | forward, pooled | | 26 | | | **+1.76** | **-1.32** (the calibration: 1.9 / -1.42) |
   | seq a | sideL 19-22 | 4 | -1.6 | -3.85 | +0.89 | -0.66 |
   | seq b | sideL 19-22 | 4 | -9.6 | -2.08 | -1.70 | +1.27 |
   | cmd b (T094732) | sideL 7-13 | 7 | +0.5 | -11.78 | +2.33 | -1.74 |
   | cmd c (T095353) | sideL 10-12 / 25 | 3 / 1 | -4.9 / -1.2 | -10.01 / -0.21 | +2.84 / -0.92 | -2.12 / +0.69 |
   | cmd d (T103920) | sideL 11-13 | 3 | -11.5 | -8.57 | -0.01 | 0.00 |
   | sideL, pooled | | 22 | | | **+0.94** | **-0.70** |
   | seq a | sideR 8-11 | 4 | +7.0 | -3.22 | +2.83 | -2.11 |
   | seq b | sideR 8-11 | 4 | +3.4 | -4.17 | +2.25 | -1.68 |
   | repos_sideR (T093818, fixed sideR x6) | sideR 3-6 | 4 | +1.5 | -4.47 | +1.87 | -1.40 |
   | cmd a | sideR 13-15 | 3 | +6.8 | +6.67 | -0.71 | +0.53 |
   | cmd c | sideR 20-23 / 27-28 | 4 / 2 | +3.2 / +3.8 | +2.72 / -0.43 | -0.11 / +2.19 | +0.08 / -1.63 |
   | sideR, fixed runs only / `--cmd` only | | 12 / 9 | | | +2.32 / +0.20 | -1.73 / -0.15 |

   The method gives the forward drift back (1.76 against the 1.9 calibration, 26 steps), so it is sound on average, but a
   segment of 3-4 steps is noisy (the yaw read after one step swings several degrees, and the `--cmd` asked values are the
   hold's at the last report, not necessarily the ones the step was decided with). sideL: the two `--seq` segments
   disagree (+0.89, -1.70); pooled with the `--cmd` ones, 22 steps, the drift is 0.94 deg/step, about 1 deg/step less than
   forward walking, which is the 5-7 deg of rightward drift over 4-5 steps seen with 1.9 applied: the sideL value is
   0.94 (ff asked -0.70), with a standard error of about 0.7 deg/step. sideR: the fixed runs (12 steps, drift 1.9-2.8) and
   the `--cmd` runs (9 steps, -0.7 to +2.2) point opposite ways, so the data are too thin to move it: sideR stays at
   1.9 (ff -1.42). The feedback (k 0.5) closes the rest. Provisional until `--cmd` sideways runs with the new value are logged.

**Pause.** `pause` (and `0 0 0`) marches in place: zero-displacement steps, the `march` gait. Marching in place is the
gait's own steady state with the feet squared, but how long the robot can hold it is not measured: the longest
march the README records is 6 steps at T 4. A march of more than `--pause-steps` (default 15 steps, 60 s at T 4; 0 = no limit)
in a row ends the walk the same way as `stop`. The alternative (end the track and restart from the crouch) was not
built: the restart needs the crouch lead-in (2.5 s) and a new `track play`, and a second walk in the same process.

**Latency.** The frames of step j up to its lift-off (the shift over the stance foot and the settle, both feet down) do not
depend on step j, so the host decides step j late: the first upload line that reaches past lift-off j is held back until the
command is read (`CmdWalk.limit_ms()`). With `--ahead 1` the upload reaches one step past the play head, so step j is
decided when the head is at lift-off j-1 (tested: head between lift-off j-1 and 1.6 s after it). A command written just
before that read is the step after next's at the latest: the swing foot starts to move between T and 2T after the file
is written (4 to 8 s at T 4: one step best case, two worst case), and touches down 2.2 s later (T 4), so the new
displacement is complete 6 to 10 s after the write. `stop` takes the same time. `--ahead 2` adds one step to both.
(The lift gate parks the play head, so it parks the decisions too.)

**Two terminals.**

```
python experiments/roll-balance/walk_keys.py /tmp/walk_cmd          # terminal 2: writes 0 0 0, then your keys
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --cmd file:/tmp/walk_cmd --live    # terminal 1
```

`walk_keys.py` (stdlib: termios, no robot access, it only writes the file atomically): W / S forward / back, A / D left /
right, Q / E turn left / right (one press = 5 mm or 1 deg per step; clamped to the default envelope, 50 / 40 / 15 mm, 5 deg), X zeroes the
command, SPACE or ESC writes `stop` and exits (Ctrl-C exits WITHOUT a stop). Its status line shows the command and the
host's `PATH.status` (state, step, applied, what scaled it).

**Dry run.** `python track_host.py --T 4 --cmd file:/tmp/walk_cmd --dry-steps 6` rehearses six steps from the command read
now (each step re-reads the file live), prints each decision and the plan's usual dry-run report. It sends nothing.

**Tests** (`tests/test_cmd_walk.py`, fake robot; `tests/test_cmd_bystep.py`: the by-step feed-forwards, frame level -- the ankle-roll goals of the affected steps differ by the expected amount and nothing else does): command changes take effect on the next decided step; uniform
scaling; slew; stop = closing step and a clean end; unparseable keeps the last; reversal; max-steps / pause / max-time;
the frames of a constant command equal the `--step` run's (20 mm, and 40 / -30 mm with tilt 3); frames up to a step's
lift-off do not depend on the step; every frame the live loop uploads is exactly the final plan's.

**SIL** (`cmd_walk_sil.py`: the steps the walk decides for `10 0 0` x6, `0 0 1.25` x4, `0 -3.75 0` x5, `-8 0 0` x6,
`stop` at T 4 (22 steps: dx 20, 40 x5, 20, 0 x8, -20, -32 x5, 0), k 1.3 / 1.30, ff 2,2, tilt per step, lift gate on, no
extra delay; `results/cmd_walk_sil.json`):

| plant | falls | aborts | lifts | lean in max | lean out max | peak tilt | heading end |
|---|---|---|---|---|---|---|---|
| fit | 0 | 0 | 22/22 | 2.5 deg | 3.2 deg | 3.8 deg | -23.6 deg |
| inset sole (`sole_in_L=8,sole_in_R=-12`) | 0 | 0 | 22/22 | 2.6 deg | 4.0 deg | 3.7 deg | -28.9 deg |

(Four left-turn steps ask +20 deg; the sim drifts right with the feed-forward trim alone, as for `--seq`: the live heading
hold is what holds the reference. No gate holds.)

**First robot test** (T 4, `--ahead 1`, a person at the robot; nothing here has run on it):

```
echo "0 0 0" > /tmp/walk_cmd
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --cmd file:/tmp/walk_cmd           # dry run first
python experiments/roll-balance/track_host.py --T 4 --ahead 1 --cmd file:/tmp/walk_cmd --live --note "cmd test 1"
# then, from a second terminal, one line at a time, a few steps (>= 8 s) each:
echo "0 0 0"     > /tmp/walk_cmd   # march in place
echo "5 0 0"     > /tmp/walk_cmd   # forward, 20 mm per step
echo "10 0 0"    > /tmp/walk_cmd   # forward, 40 mm per step
echo "0 0 1.25"  > /tmp/walk_cmd   # turn left, 5 deg per step
echo "0 -1.25 0" > /tmp/walk_cmd   # sideways right, the body moves 5 mm per step (dy 10 on the lead step of each pair)
echo "-5 0 0"    > /tmp/walk_cmd   # back, 20 mm per step
echo "stop"      > /tmp/walk_cmd
```

## Position hold (`--pos-hold on`, `--keep-on-board`; `--cmd` only, live only; default off)

The gait creeps sideways a few mm per step (2026-11-16: 7-step marches in place moved the robot +11 and +42 mm along the
tracker's x with the plain 4 s gait, -29 and -24 mm with the pause variant; it walked itself to a board edge twice). The
heading hold closes yaw from the IMU; the position hold closes lateral and fore-aft position from the overhead camera
tracker ([tools/vision](../../tools/vision/README.md)), one bounded correction per step. `pos_hold.py` is the module,
`cmd_walk.py` applies it. Off, nothing changes: the frames are byte-identical (a `CmdWalk` without a hold takes none of
the new branches). A dry run reads no tracker and applies no correction. Fixed walks and `--seq` are not covered: their
steps are all planned before the walk starts, so there is no next step to correct.

**Conventions** (the bug class that hurt on 2026-11-16: left and right as seen from the Sony, in the webcam image, and
the robot's own). Everything is in the tracker's BOARD frame until the last line.

```
              far end (y = board length, the eMeet's end)            eMeet looks from here toward the Sony:
        x = 0  +---------------------------------------+ x = width     its IMAGE is mirrored left-right
   (Sony's    |                                       |  (Sony's      relative to x. The tracker works in
    left)     |   heading 0   = facing +y (far end)   |   right)      the board frame, so nothing here sees
              |   heading 180 = facing the Sony       |               image left or right.
              |   counter-clockwise (a left turn) +   |
              |   robot-left at heading h =           |   x: from the board's LEFT edge as seen from the Sony
              |        (-cos h, -sin h) in (x, y)     |   y: from the near (Sony) end
        y = 0 +---------------------------------------+
              near end, the Sony camera looks along +y from here
```

So facing the far end (heading 0) the robot's left is board -x; facing the Sony (heading 180) its left is board +x.
Errors are in the ROBOT frame: `lat` positive = the robot's left (toward which a positive `dy` steps), `fwd` positive =
forward; `err` = measured minus reference. The correction is `-k x err`, so a robot found left of its line gets a
negative (rightward) `dy`. `tests/test_pos_hold.py` renders the scene with `tools/vision/vsynth.py` (the eMeet's
geometry at headings 180, 140 and 0; the Sony's at 140), runs it through the real tracker, and checks the sign of the
correction in board x for a robot found displaced +x and for a `left` command it did not follow.

**What is measured, and when.** A thread reads camd's `emeet_latest.jpg` (read-only; the camera is never opened; camd writes it at
most every 0.2 s and records about 12.5 fps with gaps up to 0.44 s), runs the tracker on each new frame as fast as it goes
(the tracker takes 0.1 to 0.2 s a frame; searching a window around the last robot position saves little) and keeps time-stamped
positions. The 2026-11-16 runs with the first version delivered 13 to 17 frames per 7 s (2 per second): the thread slept 0.2 s
after every frame; it no longer does. The board pose is calibrated ONCE from the first three frames with two board tags (or
loaded from where.py's `emeet_board_pose.json`, or this tool's `emeet_poshold_pose.json`, when a visible board tag still sits
within 4 px of where the file had it: a robot hiding one board tag costs nothing, and the board frame is the same between
runs); a visible board tag more than 4 px from its calibrated place means the camera moved: no fix.

**The sample: a fit, not a mean.** The torso tag swings about +-80 mm sideways with the stance foot, once per two steps. A mean
or median over a window that is not a whole number of REAL gait cycles aliases that swing, and the real cycle is not the
planned one: in the 2026-11-16 19:13 run (planned step 3.47 s) the spectrum of the tracked x has its peak at 7.49 s, a step of
3.74 s. So the sample is a least-squares fit, over the last `--pos-window-steps` steps (default 3 = 1.5 cycles), of offset +
slope + one sinusoid to the per-frame x and y, with the sinusoid's period searched over 0.85 to 1.25 of the planned cycle (it
found 7.5 s); the position is the fit's offset at the window's centre (so the sample is 1.5 steps old, which the loop knows:
`--pos-delay-steps` 4). Measured on that recorded run (every other frame of the camd segments, the same tracker; the metric is
the jitter between successive decisions, the standard deviation of the second difference over sqrt 6, the true motion being
smooth):

| sample | jitter |
|---|---|
| the first version: trimmed mean over 2 planned steps, frames at the live rate (2.2 Hz) | 15.4 mm |
| the same with every frame | 20.2 mm |
| mean over exactly the measured cycle | 8.3 mm |
| fit of offset + sinusoid, 1 cycle | 7.4 mm |
| frames of one phase only (+-0.4 s) | 115 mm (too few) |
| **fit of offset + slope + sinusoid, period searched, 1.5 cycles** (chosen; at 1.85 Hz: 2.4 mm) | **2.5 mm** |
| the same, 2 cycles | 2.0 mm, but 2 steps older |

On the 19:09 run (plain timing, real step 3.99 s against 4.0 planned, so nothing to alias) the same comparison is 6.3 mm for the old sample
and 4.9 mm for the fit: the fit matters when the real cycle differs from the planned one, as it does with the pause timing.

(Only the jitter is measured: the offset's bias against the true mean position is not, the fit agrees with the plain mean
over the measured cycle to within the jitter. A sample needs 8 frames spread over half the window, else NO FIX.) The
standing pose, taken before anything is sent, is a plain mean.

**The loop, per step.** Reference = the pose the walk settles at + the integrated command (what each step was asked for
before any correction: with a zero command the position is held; with a velocity, the line and pace of the command, integrated
along the commanded heading, which starts at the measured one; the robot's own heading error is not booked as lateral error
beyond the position it really causes: errors are measured in the robot's frame at the measured heading). The standing pose is not
that reference: the mean over a walking cycle sits tens of mm from where the robot stood (2026-11-16 run: +28 mm), so after the
first 3 steps (`warmup`) the reference is re-based to the walking mean (printed: `reference re-based`), and the standing pose stays
the start in the log. Error `e` (robot frame) = the sample's position - the reference at the end of the steps the sample
already shows, plus the proportional part of the corrections of the `--pos-delay-steps` (4) decided steps it does not show yet
(a Smith predictor; the integral part is left out on purpose, it is the learned creep canceller and the creep acts on those
steps too). Each step prints the sample's own error and the pending part: `err lat -81.7 = sample -33.0 + pending -48.7` (the
line the first robot test of this found confusing: the position and the error were the same sample; the error includes the
three corrections still on their way, which were then large and the other way).
Control law per axis: `u = -k x (e beyond a dead band) + I`; `k` = `--pos-k` 0.3, dead band `--pos-deadband-mm` 5 (soft: only the
excess is corrected), `I` integrates the excess with `--pos-ki` 0.06 per step, is clipped to the correction clip and is not wound
up while the output sits at the clip. `u` is clipped to `--pos-clip` (lateral 6, fore-aft 10 mm per step) and may change by at
most `--pos-rate-mm` (5) per step. The corrections are in BODY mm per step. **A lateral correction `c` is planned as a sidestep
pair**: `dy = 2 c` is added to the next undecided step AFTER the envelope and the slew (it is itself kept inside both:
`--max-side-mm`, `--slew`), the close step of the pair carries the same value (ignored by the geometry), and the hold's books are
the body's travel from the feet's geometry (`CmdWalk._body_dy`), not `dy`: the command's own travel is `dy / 2` a step, the rest is
the hold's. Corrections below `--pos-min-step-mm` (4) are held back and added up, so a march in place gets a few larger sidesteps
instead of a stream of 1 to 3 mm ones (in the first test a -3.2 mm step 9 had the swing foot land outward and the robot tipped
12.6 deg inward once; whether the 3 mm was the cause is not known, the by-step feed-forwards that switch with the step type
-- ankle ff, heading drift -- now blend over the first 8 mm of sideways travel instead of switching at 0). The lateral-reversal
rule and the pre-check judge the sum. The pause counter (`--pause-steps`) counts the command, not the correction.

The slew of the COMMAND is anchored at the last commanded step, not at the last step uploaded (which carries the hold's
corrections); otherwise a correction of 15 mm turned a zero command into a slew-limited 5 mm sideways command for the next steps.

**Sideways steps get the plain timing.** With the pause timing a sidestep is eaten: the swing foot lands deterministically to the
robot's right of its lift-off spot (L -7, R -15 mm at contact; the creep of about -4.7 mm per step), and a +10 mm sidestep
command moved the robot 52 mm to its RIGHT in 10 steps; with plain timing sidesteps deliver 78 to 170 % of the plan (mean about
100 %, with +-10 mm of stick-slip a step). See `--side-timing` below. The fake-robot tests use these numbers: creep -4.7 with the
pause timing and +0.5 with the plain one, a 0.5 to 1.0 response, 4 to 8 mm of stick-slip, 2.5 mm of sample jitter, 0 to 3 steps of
delay: the lateral error peaks at 55 to 75 mm in the first 10 steps (3 of warmup, the delay, the creep acting before the first
correction) and comes back to a mean of 16 to 35 mm over steps 25 to 40, with at most 2 sign changes of the correction. It does
not hold within 25 mm from the start.

Every step prints (and the summary row's `cmd_walk.steps[].pos` keeps) the position, the reference and the error:

```
  cmd step 7: asked +0.00 +0.00 +0.00 ... -> marching: dx +0.0 mm, dy +9.0 mm, dyaw +0.00 deg per step ...
    pos x 392 y 601 h 179 (n 14; ref 406 600, err lat -13.2 = sample -9.0 + pending -4.2, fwd +0.4 = ...) -> corr dy +4.5 dx +0.0 mm
```
(a step line also carries what the hold did: `settling`, `reference re-based`, `NO FIX`, `LATERAL CORRECTION OFF`; `corr dy` is the
body's travel, the step's own `dy` the lead step's.)

`pos_hold` in the summary has the pose at rest before and after the walk (x, y, heading), the largest |lat| and |fwd| error, the
step count, steps without a fix, keep-on-board overrides, why the lateral correction went off (if it did), and the error ranges.

**A sideways response that is wrong-signed.** A WEAK response is handled by the integral. A WRONG-SIGNED one shows as an error
that keeps growing against corrections: if, over the last 6 steps, the lateral corrections averaged 4 mm of body or more against
the error and the error (3-sample means, then and now) grew by 45 mm or more, or the corrections sit at the clip and the error is
120 mm off, the lateral correction goes OFF for the rest of the walk, and every step's `pos` line says `LATERAL CORRECTION OFF:
...` (fore-aft stays on). Tuned so that no response of 0.5 to 1.0 with the creep above trips it, and a response of -0.8 or -1 does
(fake robot). The response is not estimated as a number: a per-step regression of the measured sideways motion on the commanded
one called a working loop weak (the decision-to-sample delay is not known well enough to line them up). An earlier rule
"corrections at the clip for 4 steps with the error over 30 mm" fired during the start-up transient of the first robot run
(2026-11-16 18:50) and the robot crept 63 mm past the reference with no correction.

**`--side-timing plain|same`** (default `plain` when `--td-pause` is given, else `same`; `--cmd`). A step with more than 3 mm
of dy is a sideways step. With `plain`, that step and the one after it (the lead and the close of the pair) are laid out with the
plain timing THROUGHOUT: no pause before either, the schedule's own shift / settle / single support for `--T`; every other step
keeps the pause / explicit timing, so the pause and the fast timing apply to forward, back and turn steps only.
`dwell.Dwell(step_lateral=)` does the layout per step (the firmware's phase warp too). Because a step's own pause and shift now
depend on its displacement, the frames from the touchdown of the step before it on are held back until it is decided
(`CmdWalk.limit_ms`; the decision is about half a step earlier than without side timing, and the hold knows its correction one
step ahead, so a lateral correction is laid out plain from its first frame). Without a sideways step, or without a pause, the
frames are byte-identical (tested), and `same` keeps one timing.

**No fix.** The correction freezes at the last trim for at most `--pos-no-fix-steps` steps (default 4) and says
`NO FIX ... correction frozen`; after that it is zero (`correction off`). Nothing is extrapolated: the slip estimate and
the error are only updated from a real sample. With a NON-ZERO command active, more than that many steps without a fix turns
the command into `stop` (the closing step, then the walk ends; the reason says so). A march in place goes on.

**`--keep-on-board MM`** (default 250 with `--pos-hold on`; 0 = off). If the predicted position (the sample plus the travel
of the steps already decided) is within MM of any board edge (`foot_margin_mm` already off), the step is replaced by one
toward the board centre along the directions of the near edges, as large as the envelope allows (the slew and the pre-check
still apply; the heading command is kept) and the line `KEEP-ON-BOARD: left margin 190 < 250 mm: step toward the centre` is
printed. The reference follows that step, so once clear of the edge the hold holds the new place, it does not go back. Absolute
margins are only as good as the board layout: +-100 mm until the layout is measured and f pinned (tools/vision README), so
the default 250 leaves room for that.

**Flags.** `--pos-hold on|off`, `--pos-k`, `--pos-ki`, `--pos-rate-mm`, `--pos-min-step-mm`, `--pos-clip LAT,FWD`, `--pos-window-steps`, `--pos-delay-steps`,
`--pos-deadband-mm`, `--pos-no-fix-steps`, `--keep-on-board MM`, `--side-timing`, `--pos-camd-dir DIR` (camd's directory; default
`$CAMD_DIR` or `~/cam-segments`), `--pos-config FILE` (the tools/vision config).

**On the robot** (camd must be recording the eMeet: `emeet_latest.jpg` fresh; a person at the robot; start with the
tags in view and the robot standing; `$CAMD` is camd's `--dir`):

```
CAMD=/home/claw/code/robot/hw_sessions/2026-11-16/vision/cam
PAUSE="--td-pause 0.3 --shift-s 1.2 --settle 0.1 --raise-s 0.77 --move-s 0.45 --lower-s 0.55"   # 3.37 s per step
# (a) 40 steps marching in place, the start position held
echo "0 0 0" > /tmp/walk_cmd
python experiments/roll-balance/track_host.py --ahead 1 $PAUSE --cmd file:/tmp/walk_cmd --pos-hold on --pos-camd-dir $CAMD \
    --max-steps 40 --pause-steps 45                                          # dry run first (no tracker is read)
python experiments/roll-balance/track_host.py --ahead 1 $PAUSE --cmd file:/tmp/walk_cmd --pos-hold on --pos-camd-dir $CAMD \
    --max-steps 40 --pause-steps 45 --live --note "pos-hold march 40"
# (b) the same with a forward command, 10 mm per step (3 mm/s x 3.37 s): start near the near end (tracker y about 300),
#     facing the far end, so 400 mm of travel stays on the board
echo "3 0 0" > /tmp/walk_cmd
python experiments/roll-balance/track_host.py --ahead 1 $PAUSE --cmd file:/tmp/walk_cmd --pos-hold on --pos-camd-dir $CAMD \
    --max-steps 40 --live --note "pos-hold fwd 40"
```

Robot runs of the hold (2026-11-16, all 40-step marches in place, `--cmd "0 0 0"`; the first two with the plain timing):
1. 18:50, first version: 41 steps clean; start (283, 585) and end (264, 566) at rest by where.py; the lateral error went +3,
   +30, +61 (start-up transient and the offset of the walking mean from the standing pose), the correction acted for 3 steps and
   was turned off by the old detector (not printed), and the robot crept 2.6 mm per step to its right to 63 mm past the reference.
2. 19:09, retuned (re-based reference, PI, dead band 10, lateral clip 12): start (358, 489, 169) -> end (343, 505, 171), max
   |err| lat 28 fwd 28 mm; one inward tip on step 9 (12.6 deg, recovered by itself, a -3.2 mm lateral step).
3. 19:13, pause timing (`--td-pause 0.3 --shift-s 1.3 --settle 0.1 --raise-s 0.77 --move-s 0.45 --lower-s 0.55`): start (343, 505)
   -> end (355, 513), no tips, but 13 to 17 frames per sample, successive errors jumping +-25 to 50 mm and corrections up to 12
   mm on that noise (the sample was the trimmed mean over 2 planned steps, 15 mm jitter; see the table above for the fit).
4. A back / forward command run with the hold (18 steps, no tips): fore-aft closed to 9 mm, lateral ended 42 mm to the right, the
   same as without the hold: the lateral correction was 0 for most of it (settling, then noise inside the dead band).
Not yet run with the fit, the pair mapping and the whole-step side timing.

What the tests cover (`tests/test_pos_hold.py`, `tests/test_cmd_walk.py`): the signs through the tracker on synthetic scenes; the
fit against the mean when the real step time is not the planned one; the closed loop on a fake robot with the numbers above
(bounded, no hunting, not tripped by a weak response); a reversed response turns the lateral correction off, with the message
printed; with a forward command and a fore-aft slip the along-track position follows the command; dropouts freeze, then zero, the
trim and stop a moving command; the edge rule steps toward the centre at headings 0 and 180 and counts the steps already decided;
small corrections add up; the sideways command is the body's velocity; side timing at frame level, and a live loop against a fake
robot that uploads exactly the final plan with it. The fake robot has the model's creep, response and delay by construction.

## The lift gate (2026-11-13): wait for a level, quiet body before a foot leaves the floor

`gaitplay/lift_gate.h` (firmware), `sim/lift_gate.py` (its twin), `track_host.py --lift-gate on` (the default for the
dwell gait; the frame flag), `lift_gate_replay.py` (the record check below). Design: `docs/firmware-design.md`, the
gait track. Why: three falls on 2026-11-12/13 lifted a foot with the body already 5-10 deg off level; the trim zeroes
its error at lift-off, so nothing objected.

**Against the records** (`python lift_gate_replay.py <the four flight_20261112T2325..20261113T0050 .frec>`: 114 track
plays, 747 lift-offs, 2026-11-12 16:30 - 2026-11-13 09:00 MST; the record is open loop, so this is what the gate would
have SEEN). roll_ds: per-side slow reference seeded at the first tick (alpha 0.25).

| the gate looks | clean lift-offs held (743) | \|roll - roll_ds\| p50 / p95 / max (deg) | \|rate\| p50 / p95 / max (deg/s) |
|---|---|---|---|
| at the lift-off tick | 0 | 0.43 / 1.58 / 2.51 | 0.4 / 2.2 / 8.5 |
| 40 ms before | 1 (a 16.0 deg/s spike) | 0.47 / 1.60 / 2.51 | 0.4 / 2.6 / 16.0 |
| 100 ms before | 0 | 0.45 / 1.60 / 2.59 | 0.5 / 2.5 / 11.3 |

| fall (MST) | last lift-off | dev deg | rate deg/s | gate (0 / 40 / 100 ms before) |
|---|---|---|---|---|
| 11-13 08:40:21 | 08:40:20, 0.7 s before | -9.34 | -8.5 | HOLD / HOLD / HOLD (level only) |
| 11-13 08:20:00 | 08:19:58, 2.5 s before | -4.95 | +13.3 | HOLD / HOLD / HOLD (level only) |
| 11-12 19:27:57 | 19:27:54, 3.1 s before | -0.43 | +0.8 | pass: NOT a lift-off fall (see below) |
| 11-12 18:09:44 (not asked for) | 18:09:44, 0.2 s before | +26.8 | +104.9 | HOLD, but already toppling |

The 19:27 fall started in the double support after the touchdown of the step before it (roll +6.9 deg and -22 deg/s
0.7 s before, -16 deg and -82 deg/s 0.3 s before), before the next lift-off: no lift gate sees that. The rate test caught
neither of the other two (13.3 and -8.5 deg/s are under 15); the level test did.

**Not built: a second gate at the first shift frame after the touchdown.** At that tick (balance state HOLD -> FADE)
the clean steps are not separable from the falls: against a per-side reference seeded and updated as above,
|roll - ref| p50 0.31 / p95 2.24 / p99 4.75 / max 7.56 deg and |rate| p50 3.5 / p95 21.5 / p99 47.4 / max 63.6 deg/s
over 619 clean touchdowns; the 19:27 touchdowns before the fall read dev +2.67 / -2.18 deg, rate -48.6 / -43.7 deg/s,
and the 08:19 ones dev <= 1.4 deg, rate <= 13.8 deg/s: inside the clean range. Thresholds that hold the clean ones
<= 0.5 % do not hold those; at 3 deg / 15 deg/s 83 of 732 clean touchdowns (11 %) would be held, 22 (3 %) at 3 deg /
30 deg/s. The body is normally still moving at touchdown.

**SIL** (`sil_gait.run(..., lift_gate=True, dist=...)`, `sim/sil/test_lift_gate_sil.py`): dwell T 4 and T 5, fwd /
back / march, 4 steps: every lift-off has a gate frame, zero holds, the per-step reports and the finish identical to
the run without the gate. A roll-rate kick (200 deg/s at 1.56 s, in the double support before the first lift-off): the
gate holds 360 ms and the walk goes on. A 290 deg/s kick at 1.62 s: the run without the gate falls (35 deg); with it
the hold times out after 3000 ms, the track ends with both feet down, no fall, no foot lifted.

## The foot-off sag: a plant term for the release at lift-off (`--footoff-sag`, `sag_L=`/`sag_R=`; `footoff_sag.py`; 2026-11-16 evening)

When a foot leaves the floor the robot's body rolls toward it beyond the stance encoders, then rings
(hw_sessions/2026-11-16/DESIGN_moving_lift_gait.md §1: on a LEFT stance 6.4 deg, on a RIGHT stance 2.9 deg, released
over 0.66 s, 2.0 Hz / damping ratio 0.05 in the fit). The plants above have no such release: the per-side play lets
the body fall through a free band onto the rigid PD, which does not ring, and the inset sole moves the static edge
instead of the thing that sets it.

**The term** (walker_env `footoff_sag`, mirrored in env_mjx, parity R21): a spring (60 N m/rad) in series with each
hip-roll and ankle-roll servo, beyond its encoder, in the roll series spring's quasi-static form (the link sees the
PD × k/(kp + k), the encoder reads link + deflection). Each spring's rest angle carries a preload toward the other foot,
scaled by its own foot's share of the floor load (the sole pads' penetration, once per control tick). 30 % of a
side's preload sits at the ankle (it rolls the whole body over the foot), 70 % at the hip (it tilts the torso over the
stance leg); the design note infers a whole-body share near 30 % from the ankle feed-forward that cured the inward tips.
In double support the two legs' preloads push against each other through the floor. When one foot unloads, the stance
leg's preload is no longer held, and the body sags onto the springs as fast as the swing foot rises. So the release
time follows the raise with nothing fitted to it; a 0.42 s raise releases faster than a 0.77 s one.
- Its value is the static sag in degrees per stance side. Preload = 0.5 × that; with the springs' own give the
  replayed sag comes out within about 10 % of the value on a left stance.
- It replaces the roll play and the series spring (exclusive: three models of the same yield).
- Use it on the plant's own sole, without `sole_in`. With the inset sole as well, the two count the same inner edge
  twice: every replayed left-stance step of the clean runs tips.
- Mechanism choices tried and dropped (replay grids, `/tmp` scripts, not kept):
  - preload at the ankle alone: every left stance tips, even at a quarter of the preload;
  - the term on top of the play and the inset sole: tips the clean runs;
  - zero dry friction on the rolls: rings more but tips the clean runs.

**Validation** (`footoff_sag.py run`): each single support of six 2026-11-16 track walks is replayed on its own.
- Placement: grade.py's plant is placed at the robot's measured joints before the weight shift (2.5 s before the raise,
  or 0.4 s after the touchdown before it).
- Drive: it then gets the robot's written goals, trim, lift gate and ankle feed-forward included, to 0.8 s after
  touchdown.
- sag = the passive roll beyond the stance encoders (IMU minus hip + ankle roll encoders) over 0.9-1.3 s after the raise
  starts, median over the clean steps after the first.
- Plants:
  - `play` = `play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0`;
  - `inset` = that + `sole_in_L=8,sole_in_R=-12`;
  - `sag` = `sag_L=6.4,sag_R=2.9`.

| run | stance | robot sag / t90 / peak | play | inset | **sag** |
|---|---|---|---|---|---|
| 152118 march T 4 (clean) | L | 5.6° / 0.44 s / 7.2° | 5.1 / 0.46 / 5.4 | 2.4 / 0.34 / 4.7 | **5.5 / 0.44 / 7.0** |
| | R | 3.1 / 0.36 / 5.2 | 2.5 / 0.36 / 2.7 | 4.5 / 0.48 / 6.9 | **2.4 / 0.36 / 3.1** |
| 150059 fwd 40 mm, td-pause (clean) | L | 5.4 / 0.44 / 7.5 | 4.5 / 0.46 / 4.7 | 4.1 / 0.42 / 4.9 | **5.8 / 0.44 / 7.6** |
| | R | 1.7 / 0.34 / 2.2 | 3.8 / 0.42 / 3.9 | 4.7 / 0.60 / 4.9 | **2.3 / 0.36 / 2.8** |
| 174820 Variant M fwd 20 mm (clean) | L | 5.4 / 0.42 / 7.5 | 5.2 / 0.50 / 5.4 | 4.0 / 0.38 / 5.6 | **6.0 / 0.42 / 8.2** |
| | R | 1.5 / 0.34 / 2.1 | 3.6 / 0.40 / 3.7 | 6.1 / 0.60 / 6.8 | **2.8 / 0.36 / 3.3** |
| 151416 raise 0.42 s | L | 5.0 / 0.22 / 9.8 (n 1) | 5.0 / 0.32 / 5.5 | all tip | all tip |
| | R | 2.5 / 0.26 / 4.2 | 3.9 / 0.26 / 4.0 | 6.2 / 0.42 / 6.8 | **2.4 / 0.24 / 3.6** |
| 152225 raise 0.42 s + pre-hike | L | 5.8 / 0.28 / 8.3 | 6.1 / 0.36 / 6.4 | 2.4 / 0.24 / 6.5 | **5.9 / 0.30 / 9.8** |
| | R | 2.2 / 0.26 / 5.0 | 2.5 / 0.28 / 3.7 | 2.8 / 0.28 / 7.4 | **2.7 / 0.28 / 3.9** |

Tips: steps whose roll toward the lifted foot peaked ≥ 7° before touchdown, the track's own tip line. Odd steps are
left stances.

| run | robot | play | inset | **sag** |
|---|---|---|---|---|
| 152118, 150059, 174820 (clean) | none | none | none | **none** |
| 150430 `--ss-s 1.2` march (fell) | step 2 (R) | none | none | **step 1 (L)** |
| 151416 raise 0.42 s | 3, 6, 7 | none | 3, 5, 7 | **1, 3, 5, 7** |
| 152225 raise 0.42 s + pre-hike | 2, 5, 6 | none | 5 | **5, 7** |

What matches, with the sag on:
- Left-stance sag: within 11 % on every clean run.
- Right-stance sag: within 30 % on the march and on both quick runs.
- t90: the robot's on the T 4 steps (0.42-0.44 s); on the quick raises 0.24-0.30 s against the robot's 0.22-0.28.
- The sag's first peak on a left stance of the clean runs: 7.0-8.2° against 7.2-7.5° (the play plant: 4.7-5.4°).
- Tips: none on the three clean runs, and some on each run that tipped.

What does not match:
- **Right-stance sag on the two forward walks is too large:** 2.3 / 2.8° against 1.7 / 1.5° (+35 % / +87 %). The
  march's right stance is 3.1°, so the robot's right side varies with the gait more than the term does.
- **The sim tips only on left stances.** The robot's right-stance tips (step 2 of the `--ss-s 1.2` march, step 6 of
  the raise-0.42 march, steps 2 and 6 with the pre-hike) are not reproduced. The sim tips step 1 of the `--ss-s 1.2`
  and raise-0.42 marches, which the robot held.
- **The ringing after the first peak is too damped.**
  - Damped-sine fits on the median sag trace: robot 1.4-1.7 Hz, ζ 0.05-0.17.
  - Sim 1.7 Hz, ζ 0.57-0.61 on the forward walks' left stances; elsewhere no oscillation to fit.
  - The plant's dry friction (0.17 N m) and damping on the roll links take it out after one swing.
  - Removing the friction rings more but tips the clean runs (above).
  - The design note's 2.0 Hz / ζ 0.05 is its model's parameter. On this replay measure the robot rings at 1.4-1.7 Hz.
- **Double support leans further outward:** over the last 1 s of the shift the sag plant rolls 1.2-1.7° further away
  from the lifted foot than the robot on the forward walks (−1.9 / −2.6° against −0.7 / −0.9° on a left stance).
  - Its springs make the double-support roll chain softer.
  - The design note's double-support ringing (3.0 Hz, ζ 0.32 after the shift kicks) is not measured here.
- Calibrated and validated on grade.py's plant only. The training plant's DR centre (kp × 0.9, kd × 0.56) is not
  replayed.
- The static edge the inset sole was fitted to (march k R 1.20 tips, 1.30 holds; SIL table above) is not re-run with
  the sag on.

**Using it.**
- Grade on it as a fourth plant: `GRADE_PLANT_TERMS=sag_L=6.4,sag_R=2.9`. It drops the run's own play or spring.
- Train with `--footoff-sag L=6.4,R=2.9` in place of `--roll-play-deg ...` and `--sole-in-mm ...`.
- For domain randomisation, 0.5-1.5 × per side per episode: `--footoff-sag L=3.2:9.6,R=1.45:4.35`. The two forward
  walks' right stance (1.5-1.7°) sits at the bottom of that range, the march's (3.1°) inside it.
- Figures: `results/footoff_sag/<stamp>_<note>.png` (robot median and p10-p90, each plant). Numbers:
  `results/footoff_sag/summary.md` / `.json`.
- The robot's side is read from the flight record and cached in `build/footoff-sag/cache/`. The track logs come from
  whichever `~/code/robot*/experiments/roll-balance/results/` holds them (they are not committed).
