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
