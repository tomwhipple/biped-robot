# Shaper in the training loop (2026-11-02)

*walk16_act stands on the robot only with the firmware's C2 shaper at 15 Hz or lower: with
the shaper off it chattered in 1 s, at 20 Hz it tripped on body rate. It was trained without
that shaper, against an action chain (3-stage lag 2-8 Hz + 40-100 ms dead time) slower than
the robot's fast servos plus the shaper. This branch puts the firmware chain itself in the
training env: the shaper at the deployed pole, the per-joint goal speed, the fast-servo dead
time, the servo's own first-order rise, the stand clock coasting to π/2 like `standclock
coast`, and a stand-gated penalty on the servo position error (the "squeeze"). A 300M-step
fine-tune of walk16_act on that env, distilled to s128r24, is
**`robot_legs_walk17_shp10_s128r24`**. In sim, on the fast actuator with no obsfreeze, it
passes loop gain x1.5 and every walk and stride1 probe with no flail-guard trip, where
walk16_act at shape 12 (as on the robot) falls 5/8 at x1.5 and trips the guard on every walk.
It halves the stand squeeze and the power proxy. It needs the shaper: at shape 0 it falls.*

Code: `sim/mjx/env_mjx.py`, `sim/mjx/train_mjx.py`, `sim/walker_env.py` (coast mirror),
`tests/test_fw_shaper.py`, `experiments/policy/shaper-train/` (grader, GPU script, results).
Runs: `sim/runs/robot_legs_walk17_shp10{,_s128r24}`, `robot_legs_walk18_shp10lg{,_s128r24}`,
`robot_legs_walk18_shp10sq{,_s128r24}` (this worktree; copied to `~/code/robot-mjx/sim/runs`).

## 1. What the training env now models

| flag | what | firmware it mirrors |
|---|---|---|
| `--fw-shaper HZ[,HZ_MAX]` | 3 cascaded first-order lags, k = 1 − exp(−2π f dt), on the policy's targets, seeded at the start pose, then the integer goal ticks; one pole per episode U(HZ, HZ_MAX) or fixed | `obs::CommandShaper` (`actuation.h`), ctrl_task "shape" |
| `--fw-goal-speed` | the PD target slews to each new goal within the tick at 1.25 × \|goal − present\| / 20 ms, 50..3400 steps/s | `obs::goalSpeedSteps`, register 46 |
| `--act-lag-order 1` | the act-lag becomes the servo's own first-order rise (pole 1/(2πτ)); 3 stages was the old stand-in for shaper + servo | — (servo) |
| `--clock-coast-stand φ` | at a plain stand the clock keeps its rate until phase φ, then holds; same gate as the freeze; mirrored in walker_env | `obs::GaitClock::coastTo`, `standclock coast` |
| `--w-squeeze w` | stand-gated −w Σ\|served target − q\| (rad, policy joints): the error the servo's P-loop pushes on | — (the robot's ~1 W armed stand) |
| `--squeeze-lash-band` | the same less half the backlash (the sim PD's dead zone); walk17/18 used this form | — |
| `--loop-gain-range lo,hi` | per episode the targets follow mean + g × (action − mean), mean = 0.5 s EMA, g ~ U(lo, hi); the policy sees its raw action | — (the stand-still loop-gain probe as DR) |

`tests/test_fw_shaper.py` pins the shaper to the firmware law, the 0 Hz pass-through, the pole
draw, the first-order act-lag, the coast (rate never exceeded, lands exactly on φ, runs under
a walk, walker_env agrees), the squeeze metric, goal speed and the loop-gain law.

**Recipe** (`gpu/shaper_gpu.sh`): walk16_act's recipe (night 1101214001, #189 actuator DR) with
`--act-delay-types sts3215=0:2,sts3250=0:1` (fast servos, dead 18-27 ms measured),
`--fw-shaper 10 --fw-goal-speed`, `--act-lag 2.65,8 --act-lag-order 1 --act-lag-scale sts2026`
(τ 20-60 ms around the measured 30-60 ms; hip rolls 4x faster), `--clock-coast-stand 1.5708`,
`--w-squeeze 0.5 --squeeze-lash-band`, 300M steps from walk16_act, then the stock 24-round
distill. The pole is fixed at 10 Hz = `obs::kShaperPoleHz`, so the firmware default is the
trained pole and no firmware change is needed.

Teacher curve (eval ep_len / reward): 223 / 873 at the warm start (walk16_act on the new
chain), 324 / 1550 at 295M. walk16_act's own run ended at 261 / 1103.

## 2. Grading (`grade.py`)

Through the deployed chain: stand_sim's arm path, the shaper at the given pole, coast to π/2,
**no obsfreeze** (gyro x1), the fast STS3215 (dead 20 ms, τ 33 ms) on the joint-fit plant,
8 seeds. The flail guard (`components/flailguard`, kLimits) is judged on every tick: rate
EMA(\|gyro\|, 0.2 s) > 1.5 rad/s, tilt > 20° for 200 ms, chatter EMA(mean \|Δraw\|/dt, 0.1 s)
> 7 rad/s, pose (plain stand, after the ramp) > 15° from stand for 300 ms. Each criterion is
counted separately. Probes: stand 20 s + 2 s hold; x1.5 and x2 loop gain; push 0.4 / 0.5;
walk 0.3 / 0.2 / 0.15 m/s for 4 s then stop; the slow (factory) servo; and **stride1** at vx
0.6 / 0.4 / 0.3 (stand, at t = 2 s one gait cycle = 0.667 s of walk, then stand: the robot's
`link/arm_script.py --script stride1`).

Cells are falls / guard trips out of 8, then [episodes per criterion]; walk cells add the peak
EMA body rate (mean / max over seeds; the guard trips at 1.5) and forward travel in the 2.5 s
after the walk command (m).

| policy @ shaper | stand goal jitter mean / max (deg) | body rate x / y / z (rad/s) | power (W proxy) | squeeze (deg, Σ12) | x1.5 | x2 | push 0.4 | push 0.5 | slow servo |
|---|---|---|---|---|---|---|---|---|---|
| walk16_act @ 12 (the robot today) | 0.145 / 0.354 | 0.005 / 0.010 / 0.021 | 3.86 | 5.61 | **5/7** | 8/8 | 0/0 | 8/8 | 0/0 |
| walk16_act @ 10 | 0.119 / 0.277 | 0.004 / 0.009 / 0.018 | 3.84 | 5.58 | 5/6 | 8/8 | 0/0 | 8/8 | 0/0 |
| **walk17_shp10_s128r24 @ 10** | 0.123 / 0.276 | **0.002 / 0.003 / 0.004** | **1.69** | **3.10** | **0/0** | 8/8 | 0/0 | 8/8 | 0/0 |
| walk18_shp10lg_s128r24 @ 10 (+ loop-gain DR) | 0.104 / 0.310 | 0.002 / 0.004 / 0.003 | 1.63 | 3.10 | 0/0 | 8/8 | 0/0 | 8/8 | 0/0 |
| walk18_shp10sq_s128r24 @ 10 (full-error squeeze, w 1) | SQ_STAND | SQ_RATE | SQ_PW | SQ_SQ | SQ_X15 | SQ_X2 | SQ_P4 | SQ_P5 | SQ_SLOW |

| policy @ shaper | walk 0.3 → stop | walk 0.2 → stop | walk 0.15 → stop | stride1 0.6 | stride1 0.4 | stride1 0.3 |
|---|---|---|---|---|---|---|
| walk16_act @ 12 | 0/8 [rate 8, pose 3] (2.16 / 2.69, 0.58) | 0/8 [rate 8] (2.09 / 2.22, 0.57) | 0/8 [rate 8] (1.88 / 2.13, 0.55) | 0/8 **[rate 3, pose 8]** (1.47 / 1.73, 0.26) | 0/8 [pose 8] (1.31 / 1.43) | 0/8 [rate 1, pose 8] (1.29 / 1.57) |
| walk16_act @ 10 | 0/5 [rate 5] (1.66 / 1.89) | 0/6 [rate 6] (1.61 / 1.85) | 0/4 [rate 4] (1.51 / 1.71) | 0/8 [pose 8] (1.25 / 1.31) | 0/8 [pose 8] | 0/8 [pose 8] |
| **walk17_shp10_s128r24 @ 10** | **0/0** (0.86 / 0.91, 0.61) | 0/0 (0.86 / 0.89, 0.58) | 0/0 (0.91 / 0.96, 0.55) | **0/0** (0.82 / 0.87, 0.22) | 0/0 (0.83 / 0.86, 0.22) | 0/0 (0.83 / 0.91, 0.22) |
| walk18_shp10lg_s128r24 @ 10 | 0/0 (1.32 / 1.38) | 0/0 (1.28 / 1.37) | 0/0 (1.23 / 1.31) | 0/0 (1.18 / 1.22) | 0/0 (1.17 / 1.20) | 0/0 (1.16 / 1.19) |
| walk18_shp10sq_s128r24 @ 10 | SQ_W3 | SQ_W2 | SQ_W15 | SQ_S6 | SQ_S4 | SQ_S3 |

Full tables: `base.md`, `walk17_shp10.md`, `walk18_shp10lg.md`, `walk18_shp10sq.md`; shaper pole
sweeps `poles_w16.md`, `poles_shp10.md`.

**Findings:**

1. **The sim reproduces the robot's stride1 failure.** walk16_act at shape 12 trips the guard
   on body rate in 3/8 stride1 0.6 runs (peak EMA up to 1.73 rad/s; the robot tripped at 1.57
   0.28 s in) and on the pose criterion in 8/8: one walk cycle ends with the swing leg still
   > 15° from stand for > 300 ms after the stand command returns. Sustained walks trip on rate
   in every run (peak 1.9-2.2 rad/s, mostly **yaw**: 1.0-2.0 rad/s EMA vs 0.7-0.9 in roll and
   pitch). So walk16_act cannot walk under today's guard at all, in sim.
2. **The shaper-trained student walks with half the body rate.** Peak EMA 0.82-0.91 rad/s on
   every walk and stride (vs 1.25-2.7), no trip on any criterion, the closing step settles
   inside the pose window, and stride1 travels 0.22 m (one cycle at 0.3-0.6 m/s) instead of
   lurching.
3. **Loop-gain margin: x1.5 0/8** (walk16_act 5/8 falls at shape 12, 5/8 at 10). x2 still fails
   8/8 for every policy. The stand-still study placed the robot at about x1.5 of the sim's
   loop gain, so this is the hardware-candidate profile ("x1.5 0, x2 8").
4. **Squeeze and power.** The stand squeeze drops 5.6° → 3.1° and the power proxy 3.9 → 1.7 W,
   but only in the pitch chain (hip pitch, knee, ankle 0.6° → 0.1-0.2° each). The rolls, where
   the robot's armed power goes, did not move (hip roll 0.32 / 0.42°, ankle roll 0.51 / 0.60°
   → 0.38 / 0.45, 0.52 / 0.66). The cause is the penalty's lash band: the roll-lash DR is
   0.5-2°, and errors inside half of it were free. The servo's encoder is on the output, so
   it pushes on the full error; `--w-squeeze` now uses it by default (walk18_shp10sq).
5. **The student depends on the shaper.** Pole sweep (`poles_shp10.md`): shape 8-12 pass
   x1.5; 15 Hz 5/8 falls; 20 Hz 8/8; **shape 0 falls at a plain stand** (5/8, jitter 4.6°). Its
   raw (pre-shaper) targets move more tick to tick (chatter EMA 1.3-1.5 rad/s at the stand vs
   0.6), which the shaper absorbs. Deploy only with `shape 10` (the boot default).
6. **Loop-gain DR did not help** (walk18_shp10lg, from walk17_shp10, cut at 262M of 300M by an
   unexplained kill at 12:52, distilled from that checkpoint): x2 still 8/8, and walking body
   rate went up to 1.2-1.3 rad/s, closer to the guard. Not a candidate.
7. **Referee** (CPU, walker_env, no shaper; `night_distill` gate): walk17_shp10_s128r24 stand_10s
   6/8 (drift 12.2 cm), walk_steps 0/8, falls 29 %, wobble 0.71 rad/s. walk16_act_s128r24:
   6/8 (13.4 cm), 0/8, falls 46 %, wobble 0.79. At least parity, off its own chain.
8. **Export.** `make deploy RUN=robot_legs_walk17_shp10_s128r24 POLICY=v6legs` is a drop-in
   (same layout; `kClockStandFreeze = false`, so `standclock` applies), and `make -C
   firmware/host test-profiles` is ALL GREEN for v6legs and v6legs256 with it.

## 3. On the robot (when it is back)

```
make deploy RUN=robot_legs_walk17_shp10_s128r24 POLICY=v6legs
make firmware POLICY=v6legs  (flash)
# fast servos (reg 85 = 0 / 81 = 1, the boot default since #195), tethered, hard floor
obsfreeze off            # gyro gain 1 -- no obs knobs
shape 10                 # = the trained pole (boot default; never 0 for this policy)
standclock coast         # default target pi/2 = the trained stand phase
run
```

Then stride1 (`link/arm_script.py --script stride1`), and a gentle walk. Expectations from sim:
the stand motionless with less squeeze in the pitch joints (the rolls unchanged); stride1 with
peak body rate ≈ 0.9 rad/s against the guard's 1.5. walk16_act at shape 12 is the A/B.

**The flail guard pose criterion.** Even walk16_act's clean strides trip it in sim: the
command returns to stand while the closing step is still in the air. walk17_shp10 settles in
time in sim, but the margin is unknown on the robot. If stride1 trips on `pose`, the guard
needs a grace after a walk → stand transition (training's `stop_grace_s` is 0.4 s), not a
policy change.

## 4. Night queue (prepared, not queued)

NIGHT_PLACEHOLDER

## Reproduce

```bash
cd experiments/policy/shaper-train                  # CPU: CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu
python grade.py 8 base.json                         # walk16_act @ 12 and @ 10
RUNS=robot_legs_walk17_shp10_s128r24,robot_legs_walk18_shp10lg_s128r24 python grade.py 8 walk18_shp10lg.json
RUNS=robot_legs_walk17_shp10_s128r24 ONLY="stand,x1.5,stride1 0.6,walk 0.3 -> stop" python grade.py 8 poles_shp10.json poles
gpu/shaper_gpu.sh "<out>|<args>"                    # under the runner lock
```
