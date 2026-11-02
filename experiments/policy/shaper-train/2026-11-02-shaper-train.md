# Shaper in the training loop (2026-11-02)

*walk16_act stands on the robot only with the firmware's C2 shaper at 15 Hz or lower. With
the shaper off it chattered in 1 s, and at 20 Hz it tripped on body rate. It was trained
without that shaper, against an action chain (3-stage lag 2-8 Hz + 40-100 ms dead time)
slower than the robot's fast servos plus the shaper. This branch puts the firmware chain itself
into the training env:*

- *the shaper at the deployed pole, and the per-joint goal speed;*
- *the fast-servo dead time and the servo's own first-order rise;*
- *the stand clock coasting to π/2, like `standclock coast`;*
- *a stand-gated penalty on the servo position error (the "squeeze").*

*Two 300M-step fine-tunes (walk16_act → walk17_shp10 → walk18_shp10sq), distilled, give
**`robot_legs_walk18_shp10sq_s128r24`**. In sim, on the fast actuator with no obsfreeze:*

- *it passes loop gain x1.5 (walk16_act at shape 12, as on the robot: 5/8 falls);*
- *it passes every walk and stride1 probe with no flail-guard trip, at peak body rate
  0.7-0.8 rad/s (walk16_act trips the guard on every walk, 1.3-2.7 rad/s);*
- *it cuts the stand squeeze from 5.6° to 2.0°, including the hip rolls (0.4° → 0.1°) and
  the ankle rolls (0.55° → 0.43°).*

*It needs the shaper at the trained 10 Hz, which is the firmware default.*

Code: `sim/mjx/env_mjx.py`, `sim/mjx/train_mjx.py`, `sim/walker_env.py` (coast and shaper
mirrors for the referee), `experiments/policy/rca/simh.py` (no double shaping),
`tests/test_fw_shaper.py`, `experiments/policy/shaper-train/` (grader, GPU script, results,
night jobs). Runs are `robot_legs_{walk17_shp10, walk18_shp10lg, walk18_shp10sq}` and their
`_s128r24` students. They are in this worktree's `sim/runs` and copied to
`~/code/robot-mjx/sim/runs`.

## 1. What the training env now models

| flag | what | firmware it mirrors |
|---|---|---|
| `--fw-shaper HZ[,HZ_MAX]` | 3 cascaded first-order lags, k = 1 − exp(−2π f dt), on the policy's targets, seeded at the start pose, then the integer goal ticks. One pole per episode U(HZ, HZ_MAX), or fixed. | `obs::CommandShaper` (`actuation.h`), ctrl_task "shape" |
| `--fw-goal-speed` | Within the tick, the PD target slews to each new goal at 1.25 × \|goal − present\| / 20 ms, 50..3400 steps/s. | `obs::goalSpeedSteps`, register 46 |
| `--act-lag-order 1` | The act-lag becomes the servo's own first-order rise, pole 1/(2πτ). The old 3 stages stood in for shaper + servo. | — (servo) |
| `--clock-coast-stand φ` | At a plain stand the clock keeps its rate until phase φ, then holds. Same gate as the freeze; mirrored in walker_env. | `obs::GaitClock::coastTo`, `standclock coast` |
| `--w-squeeze w` | Stand-gated −w Σ\|served target − q\| (rad, policy joints): the error the servo's P-loop pushes on. | — (the robot's ~1 W armed stand) |
| `--squeeze-lash-band` | The same, less half the backlash (the sim PD's dead zone). walk17_shp10 / walk18_shp10lg used this form. | — |
| `--loop-gain-range lo,hi` | Per episode the targets follow mean + g × (action − mean), with mean = 0.5 s EMA and g ~ U(lo, hi). The policy sees its raw action. | — (the stand-still loop-gain probe as DR) |

walker_env (the CPU referee) now also takes `fw_shaper_hz` and inherits it from a run's
config.json: the shaper is the robot's firmware, not training DR. For a drawn band it grades
at the middle. `simh.build` turns it off, because the stand-still and shaper-train graders run
the firmware chain in front of the plant themselves.

`tests/test_fw_shaper.py` (10 tests) pins:

- the shaper to the firmware law, the 0 Hz pass-through, and the pole draw;
- the first-order act-lag;
- the coast: the rate is never exceeded, it lands exactly on φ, and it runs under a walk;
- the squeeze metric, goal speed, and the loop-gain law;
- walker_env's coast and shaper against env_mjx.

**Recipe** (`gpu/shaper_gpu.sh`): start from walk16_act's recipe (night 1101214001, with the
#189 actuator DR), then add:

- `--act-delay-types sts3215=0:2,sts3250=0:1`: fast servos, measured dead time 18-27 ms;
- `--fw-shaper 10 --fw-goal-speed`;
- `--act-lag 2.65,8 --act-lag-order 1 --act-lag-scale sts2026`: τ 20-60 ms, around the
  measured 30-60 ms, with the hip rolls 4x faster;
- `--clock-coast-stand 1.5708`.

Then 300M steps and the stock 24-round distill. The pole is fixed at 10 Hz =
`obs::kShaperPoleHz`, so the firmware default is the trained pole and **no firmware change
is needed**.

| run | from | squeeze term | extra | teacher eval ep_len / reward, start → end |
|---|---|---|---|---|
| walk17_shp10 | walk16_act | 0.5, lash band | — | 223 / 873 → 324 / 1550 (walk16_act's own run ended 261 / 1103) |
| walk18_shp10lg | walk17_shp10 | 0.5, lash band | loop gain 1.0-1.6 | 219 / 853 → 270 / 1240 (cut at 262M of 300M at 12:52; distilled from that checkpoint) |
| **walk18_shp10sq** | walk17_shp10 | **1.0, full error** | — | 325 / 1591 → 297 / 1390 (311M) |

## 2. Grading (`grade.py`)

The grader runs the deployed chain:

- stand_sim's arm path, the shaper at the given pole, coast to π/2;
- **no obsfreeze** (gyro x1);
- the fast STS3215 (dead time 20 ms, τ 33 ms) on the joint-fit plant;
- 8 seeds.

The flail guard (`components/flailguard`, kLimits) is judged on every tick, and each
criterion is counted separately:

| criterion | trips when |
|---|---|
| rate | EMA(\|gyro\|, 0.2 s) > 1.5 rad/s |
| tilt | tilt > 20° for 200 ms |
| chatter | EMA(mean \|Δraw\|/dt, 0.1 s) > 7 rad/s |
| pose | at a plain stand, after the ramp, a target > 15° from stand for 300 ms |

Probes:

- stand: 20 s + 2 s hold;
- x1.5 and x2 loop gain;
- push 0.4 and 0.5;
- walk at 0.3 / 0.2 / 0.15 m/s for 4 s, then stop;
- the slow (factory) servo;
- **stride1** at vx 0.6 / 0.4 / 0.3: stand, then at t = 2 s one gait cycle (0.667 s) of walk,
  then stand. This is the robot's `link/arm_script.py --script stride1`.

**How to read the cells:**

- Each cell is falls / guard trips out of 8.
- [brackets] count the episodes in which each criterion tripped.
- Walk cells add (peak EMA body rate mean / max over seeds, where the guard trips at 1.5;
  forward travel in the 2.5 s after the walk command, m).

| policy @ shaper | stand goal jitter mean / max (deg) | body rate x / y / z (rad/s) | power / hold (W proxy) | squeeze / hold (deg, Σ12) | x1.5 | x2 | push 0.4 | push 0.5 | slow servo |
|---|---|---|---|---|---|---|---|---|---|
| walk16_act @ 12 (the robot today) | 0.145 / 0.354 | 0.005 / 0.010 / 0.021 | 3.86 / 3.53 | 5.61 / 5.40 | **5/7** | 8/8 | 0/0 | 8/8 | 0/0 |
| walk16_act @ 10 | 0.119 / 0.277 | 0.004 / 0.009 / 0.018 | 3.84 / 3.60 | 5.58 / 5.48 | 5/6 | 8/8 | 0/0 | 8/8 | 0/0 |
| walk17_shp10_s128r24 @ 10 | 0.123 / 0.276 | 0.002 / 0.003 / 0.004 | 1.69 / 1.54 | 3.10 / 2.49 | 0/0 | 8/8 | 0/0 | 8/8 | 0/0 |
| walk18_shp10lg_s128r24 @ 10 | 0.104 / 0.310 | 0.002 / 0.004 / 0.003 | 1.63 / 1.66 | 3.10 / 2.89 | 0/0 | 8/8 | 0/0 | 8/8 | 0/0 |
| **walk18_shp10sq_s128r24 @ 10** | 0.126 / 0.280 | **0.002 / 0.005 / 0.003** | **0.58 / 0.21** | **2.04 / 1.15** | **0/0** | 8/8 | 0/0 | 6/6 | 0/0 |

| policy @ shaper | walk 0.3 → stop | walk 0.2 → stop | walk 0.15 → stop | stride1 0.6 | stride1 0.4 | stride1 0.3 |
|---|---|---|---|---|---|---|
| walk16_act @ 12 | 0/8 [rate 8, pose 3] (2.16 / 2.69, 0.58) | 0/8 [rate 8, pose 4] (2.09 / 2.22, 0.57) | 0/8 [rate 8, pose 2] (1.88 / 2.13, 0.55) | 0/8 **[rate 3, pose 8]** (1.47 / 1.73, 0.26) | 0/8 [pose 8] (1.31 / 1.43, 0.26) | 0/8 [rate 1, pose 8] (1.29 / 1.57, 0.26) |
| walk16_act @ 10 | 0/5 [rate 5] (1.66 / 1.89, 0.58) | 0/6 [rate 6] (1.61 / 1.85, 0.56) | 0/4 [rate 4] (1.51 / 1.71, 0.54) | 0/8 [pose 8] (1.25 / 1.31, 0.28) | 0/8 [pose 8] (1.25 / 1.34, 0.29) | 0/8 [pose 8] (1.22 / 1.27, 0.30) |
| walk17_shp10_s128r24 @ 10 | 0/0 (0.86 / 0.91, 0.61) | 0/0 (0.86 / 0.89, 0.58) | 0/0 (0.91 / 0.96, 0.55) | 0/0 (0.82 / 0.87, 0.22) | 0/0 (0.83 / 0.86, 0.22) | 0/0 (0.83 / 0.91, 0.22) |
| walk18_shp10lg_s128r24 @ 10 | 0/0 (1.32 / 1.38, 0.62) | 0/0 (1.28 / 1.37, 0.60) | 0/0 (1.23 / 1.31, 0.58) | 0/0 (1.18 / 1.22, 0.24) | 0/0 (1.17 / 1.20, 0.23) | 0/0 (1.16 / 1.19, 0.23) |
| **walk18_shp10sq_s128r24 @ 10** | **0/0** (0.78 / 0.81, 0.62) | 0/0 (0.80 / 0.86, 0.58) | 0/0 (0.80 / 0.81, 0.54) | **0/0** (0.73 / 0.82, 0.24) | 0/0 (0.70 / 0.72, 0.24) | 0/0 (0.71 / 0.74, 0.22) |

Stand squeeze by joint (deg, mean |written goal − q| at the zero command):

| policy | hip roll L / R | ankle roll L / R | hip pitch L / R | knee L / R | ankle L / R |
|---|---|---|---|---|---|
| walk16_act @ 12 | 0.32 / 0.42 | 0.51 / 0.60 | 0.61 / 0.58 | 0.60 / 0.60 | 0.63 / 0.60 |
| walk17_shp10_s128r24 | 0.38 / 0.45 | 0.52 / 0.66 | 0.18 / 0.22 | 0.13 / 0.08 | 0.17 / 0.14 |
| **walk18_shp10sq_s128r24** | **0.09 / 0.10** | **0.35 / 0.50** | 0.17 / 0.23 | 0.12 / 0.05 | 0.14 / 0.14 |

Full tables: `base.md`, `walk17_shp10.md`, `walk18_shp10lg.md`, `walk18_shp10sq.md`. Shaper
pole sweeps: `poles_w16.md`, `poles_shp10.md`, `poles_shp10sq.md`.

**Findings:**

1. **The sim reproduces the robot's stride1 failure.**
   - At shape 12, walk16_act trips the guard on body rate in 3/8 stride1 0.6 runs. The peak
     EMA reaches 1.73 rad/s; the robot tripped at 1.57, 0.28 s into the walk.
   - It trips the pose criterion in 8/8: one walk cycle ends with the swing leg still more
     than 15° from stand for over 300 ms after the stand command returns.
   - Sustained walks trip on rate in every run (peak 1.9-2.2 rad/s). Most of it is yaw:
     1.0-2.0 rad/s EMA, against 0.7-0.9 in roll and pitch.
   - So in sim, walk16_act cannot walk under today's guard at all.
2. **The shaper-trained students walk at less than half the body rate.**
   - Peak EMA is 0.7-0.9 rad/s on every walk and stride, with no trip on any criterion.
   - The closing step settles inside the pose window.
   - stride1 travels 0.22-0.24 m (one cycle) instead of lurching.
   - The gentlest command, vx 0.15, is handled with no trip. The gait's floor is about
     0.22 m/s, whatever the command below that.
3. **Loop-gain margin: x1.5 is 0/8.** walk16_act falls 5/8 at shape 12 and 5/8 at 10.
   - x2 still fails 8/8 for every policy.
   - The stand-still study put the robot at about x1.5 of the sim's loop gain, so this is
     the hardware-candidate profile ("x1.5 0, x2 8").
4. **Squeeze.**
   - walk17_shp10 halved the squeeze, but only in the pitch chain. Its penalty ignored errors
     inside half the backlash, and the roll-lash DR is 0.5-2°, so the rolls were free.
   - The servo's encoder is on the output, so the servo pushes on the full error.
     walk18_shp10sq penalizes the full error at w 1.0. Hip roll fell to 0.1° and ankle roll to
     0.35 / 0.50°. Total: 5.6° → 2.0°.
   - The sim's power proxy fell 3.86 → 0.58 W, and its static hold 3.5 → 0.21 W. The proxy
     ranked stand phases wrongly on the robot, so read it as relative only.
5. **The students depend on the shaper.** In the pole sweeps:

   | shape | walk17_shp10, x1.5 falls | walk18_shp10sq, x1.5 falls |
   |---|---|---|
   | 8 | 0/8 | 0/8 |
   | 10 | 0/8 | 0/8 |
   | 12 | 0/8 | 3/8 |
   | 15 | 5/8 | 8/8 |
   | 20 | 8/8 | 8/8 |
   | 0 | falls at a plain stand | falls at a plain stand |

   Their raw (pre-shaper) targets move more tick to tick than walk16_act's (stand chatter EMA
   1.3-1.5 rad/s vs 0.6), and the shaper absorbs it. **Deploy at `shape 10` (the boot
   default) and never with the shaper off.** The night job 1103shp03 trains a pole band
   (8-12 Hz) to widen this.
6. **Loop-gain DR did not help** (walk18_shp10lg). x2 is still 8/8, and walking body rate
   rose to 1.2-1.3 rad/s, closer to the guard. Not a candidate.
7. **Referee** (CPU walker_env, `night_distill`'s gate, now with each run's shaper):

   | policy | stand_10s | walk_steps | falls | wobble (rad/s) | watts |
   |---|---|---|---|---|---|
   | walk18_shp10sq_s128r24 | 8/8, drift 3 cm | 0/8, min peak 1.1-1.3 cm | 0 % | 0.31 | 12-13 W |
   | walk17_shp10_s128r24 | 8/8, drift 3 cm | 0/8, min peak 1.1-1.3 cm | 0 % | 0.31 | 12-13 W |
   | walk16_act_s128r24 at shape 12 | 8/8, 3.4 cm | 0/8 (1.0 / 0.8 cm) | 29 % | 0.52 | 20.8 W |

   walk_steps' 3 cm swing gate fails for all of them, as it did for walk16_act. Graded
   without the shaper, as the referee did before this branch, walk18_shp10sq stood 0/8
   (37 cm drift). That is finding 5, and why walker_env now inherits the shaper.
8. **Export.** `make deploy RUN=robot_legs_walk18_shp10sq_s128r24 POLICY=v6legs` is a drop-in:
   same layout, and `kClockStandFreeze = false`, so `standclock` applies. With it, `make -C
   firmware/host test-profiles` is ALL GREEN for v6legs and v6legs256. This PR leaves the
   v6legs profile unchanged.

## 3. On the robot (when it is back)

```
make deploy RUN=robot_legs_walk18_shp10sq_s128r24 POLICY=v6legs
make firmware POLICY=v6legs        # then flash
# fast servos (reg 85 = 0 / 81 = 1, applied before every arm since #195), tethered, hard floor
obsfreeze off                      # gyro gain 1: no obs knobs
shape 10                           # = the trained pole (the boot default); never 0 or > 12 for this policy
standclock coast                   # default target pi/2 = the trained stand phase
run
```

Then stride1 (`link/arm_script.py --script stride1`), and a gentle walk (vx 0.15-0.3).

Expected from sim:

- a motionless stand with the hip rolls no longer squeezing;
- stride1 at peak body rate ≈ 0.8 rad/s, against the guard's 1.5.

A/B: walk16_act_s128r24 at shape 12. Backup: walk17_shp10_s128r24 tolerates shape 8-12 but
keeps the roll squeeze.

**The flail guard's pose criterion.** walk16_act's clean strides trip it in sim, because the
command returns to stand while the closing step is still in the air. The shaper-trained
students settle in time in sim, but the margin on the robot is unknown. If stride1 trips on
`pose`, the guard needs a grace after a walk → stand transition, not a policy change.
Training's `stop_grace_s` is 0.4 s.

## 4. Night queue (prepared, not queued)

`night/1103shp0{1..4}-*` (BRANCH=shaper-train; the parents are already in
`~/code/robot-mjx/sim/runs`):

- `1103shp01-robot_legs_walk19_shp10sq` + `1103shp02-distill_…_s128r24`: walk18_shp10sq +600M,
  same recipe, seed 11. Target: some x2 margin at equal squeeze.
- `1103shp03-robot_legs_walk19_shp8_12sq` + `1103shp04-distill_…_s128r24`: the same with the
  shaper pole drawn U(8, 12) Hz (deploy at 10), seed 12. Target: x1.5 0/8 at shape 12 and 15.

Each 600M job takes about 75 min on the night GPU, plus about 3 min per distill. Grade with
`RUNS=<student> python grade.py 8 out.json`, and the `poles` set for 1103shp03.

## Reproduce

```bash
cd experiments/policy/shaper-train                  # CPU: CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu
python grade.py 8 base.json                         # walk16_act @ 12 and @ 10
RUNS=robot_legs_walk17_shp10_s128r24,robot_legs_walk18_shp10sq_s128r24 python grade.py 8 walk18_shp10sq.json
RUNS=robot_legs_walk18_shp10sq_s128r24 ONLY="stand,x1.5,stride1 0.6,walk 0.3 -> stop" python grade.py 8 poles_shp10sq.json poles
gpu/shaper_gpu.sh "<out>|<args>"                    # under the runner lock; "<out>|DISTILL_ONLY" to distill
```
