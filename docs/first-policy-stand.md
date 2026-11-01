# First policy stand: the v6 legs policy on the ESP32, zero command

*The first armed hardware test of a trained legs policy: the robot is already
standing on the floor, with no support and Tom spotting. The firmware's MLP
runs the 12 leg servos at 50 Hz with the command held at zero ("stand"). It
passes if the robot keeps standing, quietly, until the spotter takes hold of
it.*

| | |
|---|---|
| policy | `robot_legs_walk14_s1_s128r24`: (128, 128) student of `robot_legs_walk14_s1`, action map `full`, as-built (factory P 32 / D 32) stiffness |
| image | `make firmware POLICY=v6legs` → `build/firmware-v6legs/bimo_firmware.bin` (or `idf.py -C firmware -B build/firmware-v6legs -DBIMO_POLICY=v6legs build`) |
| headers | `firmware/policies/v6legs/` (`make deploy RUN=robot_legs_walk14_s1_s128r24 POLICY=v6legs`) |
| sim evidence | `sim/mjx/eval_stand_arm.py` (below) |

The mechanics described here are the code as of this branch: `ctrl_task.cpp`,
`cli.cpp`, `linkproto/watchdog.cpp`, `link/arm_script.py`. Another branch is
unifying the armed fall latch with the bench fall guard. The procedure assumes
the result (tilt past ~45 deg for 200 ms → limp and disarm, then autohome once
the robot is picked up and held still). Today's latch trips at up_z < 0.4
(66 deg) for 200 ms; the steps below work with either.

## 1. What the robot will do

At the stand command this policy holds the calibrated zero, i.e. the `home`
pose. It does not crouch and does not step. In sim, arming from `home`:

- the joints settle within ±4 deg of zero (the left ankle roll up to −6 deg on
  the fitted plant, compensating its 2 deg zero error), and the torso height
  changes by less than 1 mm;
- peak torso tilt in the first 2 s after the arm is 1.4 deg (fitted plant) to
  3.8 deg (training plant, worst of 16), and drift over 10 s is under 1.5 cm;
- no foot lifts. Under a shove it takes about two recovery steps;
- the written target changes by at most 0.5 deg per tick in the first 2 s.
  The firmware's 1 s takeover ramp blends from the measured pose, and the
  10 Hz shaper smooths what comes after it.

The gait clock keeps running at 1.5 Hz while standing. This policy trained
that way (`kClockStandFreeze = false`). It does not mean the robot will march.

**Command channels are pinned.** The policy trained with crouch = 1 and lift
and foot = 0 only. Those channels' normaliser std sits at brax's 1e-6 floor,
so a deviation of 0.01 would reach the first layer as ~10⁴ σ. The firmware
now feeds every such channel its trained value (`policy::pinFrozen`, below).
Whatever the console sends, this policy cannot crouch, lift a foot or
foot-place, and the pack guard's landing crouch has no effect on it. A low
pack therefore means "stand, then limp after 1.5 s". **Start on a full pack.**

## 2. Before arming: go / no-go

Tethered serial console, robot benched (boot state). Every item must pass.

| # | command | pass |
|---|---|---|
| 1 | `stat` | `policy: exported, 12 joints, obs 165, run robot_legs_walk14_s1_s128r24` (not `PLACEHOLDER WEIGHTS`) and `bus: N of 17 servos fitted` with the 12 leg servos among them (item 3) |
| 2 | `volt` | pack ≥ 11.4 V (3S). The guard warns at 10.5 V and lands at 9.9 V. After a pack swap, run `batt reset`, then `batt` should show not latched |
| 3 | `cal show` | as-built calibration loaded from NVS; all 12 leg joints fitted (`run` refuses otherwise) |
| 4 | `gains` | all 12 leg servos read P 32 / D 32, matching `servo_gains.h`. `run` re-reads them and refuses on any difference (`REFUSED_GAINS`) |
| 5 | `sread 50` | 12/12 servos answer on every read, no misses; read time ~3.5 ms (bursts of 6) |
| 6 | `imu` | the QMI8658C is live (not the STUB). `imu mount` and `imu bias` were done on this board. `imu avg` shows `on` (the default) |
| 7 | `imu ring 2000` (robot standing still) | horizontal gyro RMS < 0.01 rad/s, with no peak from a bumped table |
| 8 | `shape` | 10 Hz. **Never 0**: the policy relies on the shaper's lag. With no lag in the plant, the referee's run of this same student chatters, with 65 deg single-tick target jumps and 150 W |
| 9 | `obsfreeze` | `none` |
| 10 | `fall` | guard ON. Set `fall autohome on` if you want the pickup → autohome behaviour |
| 11 | `rec`, and on mira `tools/flight_session.sh start policy_stand1` | rec ON → 192.168.2.5, wifi up, utc synced; collector running |
| 12 | robot on the floor, then `home` | `home readback: all 12 joints at their zeros -- torque released`. The robot now stands on gear friction, legs parallel |
| 13 | eyes | the robot stands upright on its own after `home` (tilt < 3 deg by eye; the `imu` up vector is about (0, 0, 1)). Floor is hard and flat or low-pile carpet, with 1 m clear all round. Spotter's hands are ~5 cm either side of the torso, not touching |

Make no changes between `home` and the arm. Arm within ~30 s of the readback,
so the pose the policy takes over is the one `home` left.

## 3. Arming

**Tethered (recommended for the first test).** Type `run`. Expect
`control loop armed -- bus handed to core 1`. Torque comes on, the joints
hold, and for 1 s the targets blend from the measured pose into the policy's.
With no radio commander running, the link watchdog never hears a frame, so it
stays in STAND. The policy gets the trained stand command (0, 0, 0, 1, 0, 0, 0)
for as long as it runs; nothing times out.

> Do not start `commander.py`, `bimo_gui` or `arm_script.py` during a tethered
> run. Once any frame arrives, the watchdog follows that sender. If its frames
> then stop, it decays to STAND after 250 ms and to **RELAX (torque off) after
> 5 s**, and the robot collapses.

A `REFUSED: ...` line means: read it, fix the cause, start again at item 1.
Never work around a refusal.

**Wireless (second session onwards).**
`.venv/bin/python link/arm_script.py --host <robot-ip> --phase arm --duration 20`
sends 0.5 s of ARM-off frames, then 20 s of ARM + stand frames with ENABLE off,
which is the zero command. Its guard sends ESTOP, then disarm, then a
RESET-SERVOS home on FALLEN, any servo fault bit, or up_z < 0.5 for 0.3 s.
When the duration ends, the frames stop: 5 s later the watchdog releases
torque. The spotter must be holding the robot by then, or run
`--phase disarm` while holding it.

## 4. Stopping: disarm and E-stop

**Every way out of a run releases torque.** The robot does not hold its pose
after a disarm; it sags onto gear friction and will tip if nobody has it.
So the normal end of a run is:

1. The spotter takes a firm hold of the torso and carries its weight.
2. Then disarm: type `bench` (`benched -- control loop released torque ...`),
   or send ARM-off frames (`arm_script.py --phase disarm`).
3. Set the robot down (or keep holding it upright) and run `home` to re-zero.

E-stop (immediate, robot goes limp):

- tethered: `bench`;
- radio: ESTOP (gamepad B in `commander.py`, the E-stop key in `bimo_gui`). It
  latches until a frame with ENABLE off arrives;
- hardware: the pack switch or connector.

If the operator is also the spotter, use the radio with the gamepad in one
hand, so the E-stop is a button and not a typed command.

**Fall handling while armed.** Tilt past the threshold, debounced 200 ms →
FALLEN → torque off and the loop benches itself (`DISARMED_FALL` in the
beacon). It does not re-arm on its own: re-arming is a fresh `run` or ARM
edge, both of which start again at item 12. With the bench guard's autohome
on, picking the robot up and holding it upright and still for 2 s homes it.

## 5. Abort conditions

Catch the robot and `bench` at once on any of:

- torso tilt beyond ~10 deg that is not coming back within 1 s. In sim the
  policy never passes 4 deg at a zero command without a shove;
- a foot lifting or a step without a shove (sim: none);
- buzzing, chatter or visible joint jitter faster than ~3 Hz, larger than a
  degree or two;
- drift beyond 5 cm, or a steady rotation;
- a servo too hot to touch, a servo fault bit or LOW-BATT in the beacon, or a
  smell.

## 6. Stages

1. **10 s**, hands ready, no contact. Then hold, `bench`, `home`.
2. If stage 1 was clean: **30 s**.
3. If stage 2 was clean: **gentle taps** at the torso, front, back, left and
   right, ≤ 0.2 m/s. In sim, 0.4 m/s kicks are survived 64/64 on the fitted
   plant and ~50/64 on the training plant's randomised plants.

After each stage, run `stat`: `late` must be 0 %, `worst` tick < 20 ms, and
`net` around 7-8 ms (budget below).

## 7. What the flight recorder will show

One record per armed tick (`tools/flight_reader.py csv <file>.frec`):

- `link_state` STAND (tethered) or LIVE (radio); `torque_on` 1; `fallen` 0;
- `cmd` exactly (0, 0, 0, 1, 0, 0, 0): the pinned channels show their
  trained value;
- `phase` advancing at 1.5 Hz;
- 12 actions in [−1, 1]. Expect a smooth, small action rate (sim: RMS
  per-tick change ~0.08), not saturating at ±1;
- per joint, `goal_rad` vs `pos_rad`: goals ramp from the measured pose over
  the first 50 ticks, then sit within a few degrees of zero; servo-side
  tracking error of a degree or two;
- the IMU block: up vector within a few degrees of vertical, gyro near zero;
- timing: `read0`/`read1` bracket a ~3.5 ms sync read.

A FALLEN tick, a torque-off tick before the `bench`, or a hole in the tick
sequence is the first thing to look for. `tools/flight_reader.py plot ...
--joints L_ankle_roll,R_ankle_roll,L_knee,R_knee` overlays goal and position.

## 8. The sim evidence

`sim/mjx/eval_stand_arm.py` grades exactly this handover. The robot settles
1.5 s on the floor under the servo hold at `home` (or at a pose ±4 / ±7 deg
off it). Then the loop takes over as the firmware does: a history filled with
one frame, prev_action 0, the shaper seeded at the measured pose, and the 1 s
ramp. It runs 10 s at the stand command. It does this on two plants:

- **train**: the referee's hardware claim (domain randomisation, 4 ms
  latency, 0.7 deg backlash, IMU noise and mount error, ±2 deg joint zero
  errors);
- **fit**: the fitted plant, `experiments/one-leg-stand/simfit_plant.json`
  (PR #179), deterministic, plus IMU noise and a per-seed IMU mount error.
  `fit:<band>` runs one of its 23 band variants.

Both run the robot's action chain: the 10 Hz shaper in series with the
servos' ~12 Hz lag, as one 3-stage cascade at 5.5 Hz (swept 2-12 Hz).

Candidates: every distilled (128, 128) student. Each cell is 16 seeds; falls
exclude episodes the hold itself tipped before the arm (those have the same
seeds for every candidate).

| student | plant | arm_home falls / tilt 0-2 s worst | arm_off ±4° falls / tilt worst | push 0.4 m/s falls (kicks survived) | push 0.7 m/s falls | joint torque Nm |
|---|---|---|---|---|---|---|
| **walk14_s1_s128r24** | train | 0/16 / 3.8° | 0/14 / 7.3° | 5/16 (50/64) | 11/16 | **0.087** |
| | fit | 0/16 / 1.4° | 0/16 / 6.2° | 0/16 (64/64) | 2/16 | **0.125** |
| walk14_s128r24 | train | 0/16 / 3.6° | 0/14 / 7.3° | 6/16 (46/64) | 6/16 | 0.118 |
| | fit | 0/16 / 1.3° | 0/16 / 6.2° | 0/16 (64/64) | 2/16 | 0.147 |
| walk13_fit_s128r24 | train | 0/16 / 4.2° | 0/14 / 7.3° | 2/16 (59/64) | 10/16 | 0.102 |
| | fit | 0/16 / 2.1° | 0/16 / 6.2° | 0/16 (64/64) | 6/16 | 0.143 |
| walk13_fit_s128r24e24 | train | 0/16 / 4.3° | 0/14 / 7.3° | 5/16 (49/64) | 6/16 | 0.104 |
| | fit | 0/16 / 2.1° | 0/16 / 6.2° | 0/16 (64/64) | 2/16 | 0.149 |
| walk13_fit_s128r24n2048 | train | 0/16 / 4.3° | 0/14 / 7.3° | 7/16 (44/64) | 5/16 | 0.100 |
| | fit | 0/16 / 2.1° | 0/16 / 6.2° | 0/16 (64/64) | 0/16 | 0.141 |
| walk13_fit_s1_s128r24 | train | 0/16 / 3.8° | 0/14 / 7.3° | 3/16 (55/64) | 11/16 | 0.120 |
| | fit | 0/16 / 1.4° | 0/16 / 6.2° | 0/16 (64/64) | 5/16 | 0.143 |

The stand_10s scenario (reset start) is 16/16 for every candidate on both
plants, with drift 1.5-2.1 cm.

**Fitted-plant band** (23 variants × 8 seeds, top four): arm_home and arm_off
0/184 falls for all four; 0.6 m/s kicks: **walk14_s1 1/184 falls**, n2048
4/184, walk13_fit_s1 29/184, walk13_fit 34/184.

**Stress** (top five, arm_home + push 0.4):

- action-chain lag 2 Hz and 12 Hz: no arm falls; push falls on the training
  plant are 4/16 for walk14_s1 at both;
- sensor slots up to 1 tick stale: no arm falls, push unchanged;
- IMU noise ×2: no arm falls, but the action rate quadruples (0.08 → 0.32);
- **1 tick stale *and* IMU noise ×2: every candidate falls 4-7/16, even from
  `home`.** That is past the realistic edge: the gyro the loop gets is the
  tick average (~0.5 tick old), `up` is the fusion filter's, and the QMI8658C
  is quieter than training's 1× noise. It is why items 6-7 matter.

**Choice: walk14_s1_s128r24.** All candidates arm cleanly. It has the best
push margin on the fitted plant, which is the best model of this robot (1/184
falls across the band), the lowest joint effort on both plants, and no
shuffling or drift under doubled IMU noise. The n2048 student shuffles there
(2-2.5 steps per 10 s from `home`). The runner-up is walk13_fit_s128r24n2048
(best under 0.7 m/s kicks on the fitted plant; weaker on the training plant).

Reproduce:

```bash
CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_stand_arm.py \
    --runs robot_legs_walk14_s1_s128r24 --plants train,fit \
    --scenarios stand_10s,arm_home,arm_off,push,walk_small --seeds 16
# + --sil: the firmware stack acts (libctrl_sil, POLICY=v6legs); --pin-clock for the python column beside it
```

### Firmware vs python, closed loop (SIL)

The same seeds and scenarios run twice: once with the firmware stack acting
(`--sil`: sim → calibrated servo ticks → firmware obs assembler, gait clock,
frozen-channel pin, MLP → goal ticks → sim), once with brax's policy, with the
gait clock pinned to the firmware's in both. 160 episodes (2 plants × 5
scenarios × 16 seeds), including `walk_small` (0.15 m/s for 4 s, about 1 m):

- falls: identical in all 160 episodes;
- standing / arm scenarios: drift, end position and tilt identical to
  0.0 cm / 0.03 deg; push and walk ≤ 2.1 cm / 2.4 deg (chaotic divergence after
  the kicks);
- per tick: the firmware MLP vs brax on the firmware's own obs ≤ 1.1e-6; the
  firmware's action vs brax on the training-obs stacking ≤ 8.8e-3 max, 1.7e-3
  mean (the tick quantiser); obs q block ≤ 6e-8 rad.

## 9. Budget (50 Hz loop, ESP32 at 240 MHz)

| phase | cost | source |
|---|---|---|
| SYNC READ, 12 servos in bursts of 6 | ~3.5 ms | measured (PR #176 `sread`) |
| obs assembly + history + frozen-channel pin | ~0.03 ms | 0.02 ms measured for the 10-joint spec |
| policy forward, 165 → 128 → 128 → 24 | ~7.7 ms (est.) | 7.2 ms measured for 147 → 128 → 128 → 20, scaled by MACs (40.6k vs 37.8k). At the stand command the 30 zero inputs per obs (linvel, height, zero commands) are skipped by `dense()`, about 7.0 ms |
| SYNC WRITE, 12 servos | ~0.1 ms | 0.07 ms measured for 10 |
| mailbox + watchdog + recorder push | ~0.1 ms + `rec` push max | `rec` prints "push cost max ctrl" |
| **total** | **~11.5 ms of 20 ms** | a tick over 20 ms counts late; one over 40 ms releases torque |

Memory (`idf.py size`, POLICY=v6legs): image 1.07 MB (66 % of the 3 MB app
partition free), weights 161 KB in flash rodata; static DRAM 138.6 KB of
180.7 KB (42 KB free), IRAM 98 KB of 131 KB. The ctrl task's 6 KB stack (ESP-IDF
sizes stacks in bytes): its deepest static path is ctrlTask 448 B +
recordFlight 160 + recordCtrl 400 + the bus write ~300, about 1.4 KB
(`-fstack-usage`). The loop allocates nothing.

## 10. Risks

- **The plant is a model.** The fitted plant comes from one-leg stands under
  the bench PI, not from a policy. Its loads disagree with the robot's
  4-5× at the rolls (2026-10-30-sim-fit.md, discrepancy 3). The servo lag is
  uncertain (2 Hz bench walk, 12 Hz joint tracking); both ends were run.
- **IMU quality is the margin that matters.** A stale *and* noisy IMU is the
  one stress that broke every candidate. Checklist items 6-8 are not
  optional.
- **The end of every run is a limp robot.** The spotter holds it before any
  disarm. A radio sender that stops mid-run releases torque 5 s later.
- **No crouch, no landing.** The pinned channels mean a low pack does not
  crouch the robot down. It stands, then goes limp.
- **The knee stands near its stop.** The policy holds the knees at about
  +2.6 deg (hyperextension); the stop is +5 deg, inside the policy range.
- **Takeover ramp.** The SIL library has no bench→run handover. The 1 s ramp
  is exercised only by the python side of `eval_stand_arm.py`, which mirrors
  `ctrl_task.cpp` (blend from the measured pose, after the shaper).
- **Fall threshold in flux.** The armed latch is being unified with the bench
  guard on another branch (assumed 45 deg / 200 ms). In sim the chosen
  policy's survived episodes peak at 7.4 deg (arm, standing, small steps) and
  21.5 deg (0.4 m/s kick recoveries), so neither threshold trips in a clean
  run.
