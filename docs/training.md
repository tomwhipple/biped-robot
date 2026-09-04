# How the training approach works

*How a walking policy for this robot is produced, from the physics model to
the weights compiled into the firmware. Written 2026-09-03 against the code as
it stands; the source files named here are the authority for any number.*

Companion docs: [controls-and-training-overview.md](controls-and-training-overview.md)
(what the control system *is*), [sil-harness.md](sil-harness.md) (how the real
control code is tested against the sim), [firmware-design.md](firmware-design.md)
(what runs on the ESP32), [precision-curriculum.md](precision-curriculum.md)
(where the skill list came from), [training-plan-v2.md](training-plan-v2.md)
(the prior-art survey the current recipe is built on).

---

## 1. The thing being trained

One neural network. It reads what the robot can actually sense, and writes ten
joint targets, fifty times a second. There is no gait generator, no state
machine, no per-joint controller underneath it — the network *is* the
controller, and the servos' own position loop is the only thing below it.

It is **command-conditioned**: the same weights stand, walk forward and
backward, sidestep, turn, crouch, balance on one leg and march, depending on a
7-number command appended to the observation. You steer the robot by changing
the command, not by switching policies. That was a deliberate decision — the
one-behavior-per-run-then-distill path was tried and closed on 2026-07-13
(DESIGN.md, "Distillation round"), and command conditioning at GPU scale is
what actually worked.

The full pipeline, end to end:

```
  sim/bimo_biped_v5body.xml          the plant (CAD-true, 10 DOF)
            |
  sim/mjx/env_mjx.py                 the world: MJX physics, STS3215
            |                        actuators, DR, commands, reward
  sim/mjx/train_mjx.py               brax PPO, ~1-2k parallel envs, on Mira's
            |                        RTX 4070 Ti, overnight
            v
      teacher policy                 (512,256,128) MLP, sim/runs/<run>/params.pkl
            |
  sim/mjx/eval_precision.py          THE REFEREE: replay in the CPU env,
            |                        28 scored scenarios x 8 seeds -> scorecard
            v
  sim/mjx/distill_student.py         DAgger distillation, teacher -> (128,128)
            |
  tools/export_policy_weights.py     params.pkl -> .silw blob + golden vectors
  tools/gen_policy_weights.py        .silw -> firmware/.../weights.h (constexpr)
  tools/gen_obs_spec.py              config + MJCF -> firmware/.../obs_spec.h
            |
  sim/sil/ + firmware/host           SIL gate: the REAL C++ control code driving
            |                        the CPU plant, scored by the same referee
            v
      idf.py flash                   the ESP32 runs it at 50 Hz, untethered
```

Each stage exists because a previous version of this project shipped something
the next stage caught. The referee exists because MJX reward numbers lied. The
SIL gate exists because byte order, servo-ID permutation and a swish-vs-tanh
mismatch all made it onto a robot once.

---

## 2. The plant

`sim/bimo_biped_v5body.xml` — the current model, and the default in
`train_mjx.py`. It is CAD-true rather than hand-tuned: per-body inertia comes
from the actual STL meshes plus component boxes (`sim/build_v2_inertia.py`),
joint limits come from `cad/dimensions.py` via `sim/joint_rom.py`, and the sole
contact geometry matches the printed pads.

- **10 DOF**: `L/R_hip_yaw`, `_hip_roll`, `_hip_pitch`, `_knee`, `_ankle`. The
  hip-yaw pair was added after an A/B study decided it ([hip-yaw-study.md](hip-yaw-study.md));
  the 8-DOF plants (`bimo_biped_v2*.xml`) survive only as legacy referees for
  old runs.
- ~0.93 kg, torso centre ~0.28 m off the floor when standing.
- A 154 g GoPro payload body at the measured mount point (`payload_cg_x/z`), so
  the policy trains under the camera it will carry.
- Earlier plants (`v3yaw`, `v4rom`) no longer load at all — pelvis v6 deleted
  the `tower.stl` mesh they declare. `v5body` is the pelvis-v6 rebuild: torso
  is one print, COM 30 mm lower, hip separation 56 → 66 mm.

**The actuators are honest.** `actuator_model="sts3215"` is a per-substep PD
loop whose output torque is clamped to the DC-motor torque–speed envelope of a
real STS3215: 2.94 N·m stall, tapering to zero at 4.04 rad/s no-load — the
latter *measured on the bench* (`tools/measure_servo_speed.py`), 14 % below the
datasheet. This is the single most important honesty in the stack. Every
policy trained before it (`terrain_v4` and its whole generation) demanded ~3
N·m at 4.4 rad/s, i.e. was physically unbuildable at any supply voltage, and
looked perfectly good in sim while doing it.

---

## 3. What the policy sees and what it writes

Generated into the firmware as `obs_spec.h` by `tools/gen_obs_spec.py`, so the
layout cannot drift between sim and robot by hand:

| block | width | contents |
|---|---|---|
| `q` | 10 | joint angles |
| `dq` | 10 | joint velocities |
| `up` | 3 | torso up-vector (from the IMU) |
| `linvel` | 3 | **zeroed** — see below |
| `gyro` | 3 | body angular rate |
| `prev_action` | 10 | what it asked for last tick |
| `height` | 1 | **zeroed** |
| `phase` | 2 | gait-clock sin/cos |
| `cmd` | 7 | the command |

One frame is **49** wide; the policy reads the newest **3** frames, so the
network input is **147**. The history depth replaces the state estimator: with
three frames the network can infer what a single frame cannot.

**`imu_obs`**: linear velocity and torso height are zeroed for the actor *and*
the critic. The robot has no odometry — the IMU gives attitude, not position —
so a policy that reads them in sim would be reading a channel that is a
constant on hardware. Zeroing them in training is what makes the transfer
honest. (The surveyed prior art all does the same; see
[training-plan-v2.md](training-plan-v2.md) §2.)

Also modelled, because the robot has them: encoder **quantization** to the
STS3215's 4096-tick word and velocity to its integer steps/s register, IMU
**mounting misalignment** and **gyro bias** drawn per episode, and per-step
sensor noise.

**Action** = 10 residual joint targets around the standing pose, so a
zero action is a stand. They are squashed through `tanh` and mapped onto each
joint's *policy range* — which is deliberately narrower than the mechanical
stop (`policy_range()` in both envs). Widening it rescales the action map and
costs a retrain, which is why it has not been widened casually.

---

## 4. The command interface

Seven channels (`ext_cmd=True`), drawn from a curriculum mix each episode:

```
c0  vx      m/s     -0.4 .. 0.8    forward and backward walking
c1  vy      m/s     +/-0.25        sidestep
c2  wz      rad/s   +/-1.0         turn
c3  crouch  frac    0.6 .. 1.0     torso height x nominal
c4  lift    -1/0/+1                lift left / neither / lift right foot
c5  foot dx  m                     swing-foot target, x
c6  foot dz  m                     swing-foot target, z
```

Everything the robot can be asked to do is expressed *through these seven
channels*. Knee-high marching, for instance, is not a new mode: it is
`vx=vy=wz=0` with `c4` alternating on the gait clock and `c6` raised to knee
height. The observation contract never changes, so a new skill never costs a
firmware change.

The radio carries only `(vx, yaw_rate)` at 20 Hz for ordinary driving, plus the
extended channels when the console sends 24-byte frames
([control-channel.md](control-channel.md)). The stand is a *trained* command,
`(0,0)`, not a bolted-on pose — which is why a lost link can decay to it
safely.

`ext_mix` sets how often each command family is drawn: for the flagship
locomotion runs, roughly 20 % stand, 15 % pivot, the rest walking split by
`walk_submix` into forward / backward / sidestep. A `--family` flag selects a
specialist mix (`loco`, `skills`, `getup`), and the referee then scores that
run only on its own family's scenarios.

---

## 5. The reward

The shape is always the same: **a primary tracking term, gated by whether the
commanded skill is actually being performed, plus shaping, minus costs.**

```
primary = g_skill * (velocity kernel + yaw kernel + heading integral)
          + height kernel
reward  = primary
          + w_upright * up_z + alive_bonus
          - w_energy * energy - w_action_rate * |da| - w_power * watts
          + gait shaping (see below)
          - fall_cost   (on termination)
```

The tracking terms are Gaussian kernels, `exp(-((measured - commanded)/sigma)^2)`
with `sigma = 0.5`. Gait shaping adds the MuJoCo-Playground recipe terms:
foot air-time, a phase-locked foot-height clock reward, foot slip, orientation,
angular-velocity, pose regularization, joint-limit and foot-crossing penalties,
plus a procedural-gait **imitation prior** (`w_mimic`) that pays for matching a
generated reference stride — the Open Duck Mini recipe, and the lowest-risk
route to a natural-looking gait on this morphology.

Three lessons are baked into the arithmetic, each learned the expensive way:

1. **The do-nothing optimum.** Kernels pay a standing robot: if you are
   commanded 0.4 m/s and move at 0, `exp(-(0.4/0.5)^2)` is still ~0.53/step.
   `precision_v1` converged to zero falls and zero motion. Fixes: `cmd_dense`
   (dense directional progress instead of a kernel when a speed is commanded),
   and the **skill-compliance gate** `g_skill` — under a lift or crouch command
   the velocity/yaw kernels pay *in proportion to the skill being done*, floor
   0.2.
2. **Kernel width is a gradient decision, not a tolerance.** A 3 cm foot kernel
   pays ~0 gradient at a 15 cm foot error, so the policy never starts climbing.
   `foot_sigma` went 0.03 → 0.06 after this happened for the third time.
3. **Ratchets beat absolute values for one-shot skills.** In the get-up work,
   absolute height paid a motionless sitting robot ~0.33/step forever; the
   height *ratchet* (only new height above the episode best pays) makes the
   rise worth a bounded one-time sum and parking worth nothing.

---

## 6. Domain randomization — the hardware contract

Sim-to-real lives or dies here. Two levels:

**Batch level** (one randomized model per parallel env, the standard MJX
pattern, `domain_randomize()`): body mass and inertia ±15 %, floor friction
±40 %, payload mass, floor tilt.

**Per-episode** (redrawn on every reset, and preserved across auto-reset by our
own `reseed()` — brax's stock AutoReset would restore the cached first state and
silently freeze this diversity): servo gain ±20 %, action latency 0–8 ms with
1 ms jitter, gear backlash 0.5–1.0°, actuation lag pole, IMU misalignment and
gyro bias, and random shove impulses.

Three of these are recent and came straight off the bench:

- **`act_lag_hz` (2–12 Hz)** — a three-stage first-order cascade on the
  commanded target, modelling the servo's own tracking lag. Measured on the
  robot, then added to *both* engines; `tests/test_act_lag_referee.py` pins the
  CPU and MJX implementations to the same law, seeded and ordered identically.
  `loco_v27full` is the first policy raised inside it from step zero, and
  carrying it cost nothing in learning rate.
- **`zero_offset_deg`** (2026-09-03) — calibration-error DR: each episode draws
  a per-joint offset between the servo's zero and the policy's frame, so the
  physical target served is `target + offset` while the observation subtracts
  the same amount. The robot's zeros are hand-measured and get re-measured after
  every mechanical disturbance; this trains a policy that does not need them to
  be perfect.
- **`play_deg` / `play_joints`** — free-travel hysteresis, added to
  `walker_env` after the 09-02 bench session measured ~3° of passive play in
  the hip-roll chain. This is *not* the same thing as backlash: sim backlash is
  a deadzone on the PD error (a servo that ignores small commands), while the
  real joint moves freely under load inside the slop. That distinction is
  currently the leading suspect for the roll limit cycle the robot shows and
  the twin does not (§10).

The general rule: **the sim only gets a hardware term after the bench measures
it.** Guessing a DR range is how you get a policy that is robust to a thing the
robot does not do and fragile to the thing it does.

---

## 7. The algorithm and where it runs

`sim/mjx/train_mjx.py` — brax PPO over the MJX env, with a hand-rolled batched
wrapper (`BatchedEnv`, `wrap_env=False`) that owns vmapping, truncation and
auto-reset so the per-episode DR redraw survives.

Typical flagship run:

| knob | value |
|---|---|
| parallel envs | 1024–2048 |
| steps | 60 M (overnight) – 400 M |
| episode | 10 s @ 50 Hz = 500 steps |
| policy net | (512, 256, 128), swish |
| value net | (512, 256, 128) |
| lr / entropy / discount | 3e-4 / 5e-3 / 0.97 |
| unroll / minibatches / updates / batch | 20 / 32 / 4 / 512 |
| obs normalization | brax `running_statistics`, shipped with the weights |

Runs **warm-start** far more often than they start cold: `--init-from <run>`
restores a prior `params.pkl`, which is how a lineage like
`loco_v10turn → v11gait → …` accumulates capability instead of relitigating
walking every night. A from-scratch run is reserved for a change that
invalidates the old weights (a new plant, a new observation, a new actuator
contract) — `v27full` is one, because act-lag from birth is a different world.

**Where:** Mira, the RTX 4070 Ti box (`ssh mira`). Training happens **at night
only** — `infra/night/night_run.sh` runs from cron at 22:00, works a queue of
job files in sorted order, starts no new job after 05:00 and hard-stops at
07:00, leaving the box to its owner during the day. It also parks idle ollama
models rather than evicting live inference.

One discipline worth stating: `~/code/robot-mjx` on Mira is a **git clone**, and
every job pulls the committed tree before training. It used to be an rsync of a
hand-curated file list, which silently missed a plant XML for two days and
trained against a stale robot. **Uncommitted code does not get trained.**
Each run writes `config.json` next to its `params.pkl`, recording the full env
and trainer configuration, so any later stage can rebuild the exact world a
policy grew up in rather than a hand-written approximation of it.

---

## 8. Judging: the referee, not the reward curve

**Never trust MJX numbers alone.** A reward curve tells you a run is learning
*something*; it does not tell you what, and MJX and the CPU env are different
implementations that must be shown to agree. So every policy is re-run on the
other engine and graded on tasks:

`sim/mjx/eval_precision.py --run-name <run>` loads a run's own `config.json`,
rebuilds its world in the **CPU** `BimoWalkerEnv`, and drives **28 scored
scenarios × 8 seeds**, each a per-step command script with explicit success
criteria:

- balance on one leg (L/R), circles in the air (L/R), march in place, hip sway,
  crouch and hold
- 1 m line, backward 1 m, sidestep (L/R), 180° turn, square and circle returns,
  goal-home, rough-ground line
- push gauntlet, speed ladder, pursuit, reversal, metronome, squat reps, weight
  shift
- recover-from-sit, recover-from-fallen, stand 10 s, stand with torque released

Results land in `scorecard.json` / `scorecard.md` per run: pass/fail per
scenario for the at-a-glance table, plus continuous metrics (endpoint error,
tracking error, clearance %, drift RMS, time-to-target, power, gait symmetry,
wobble) meaned and worst-cased across seeds. A `--family loco` run is scored on
its 18 locomotion scenarios — hence the **"/144"** totals (18 × 8 seeds) that
rank the fleet.

There are three referee **columns**, and they are not interchangeable:

| column | what it runs | file |
|---|---|---|
| python | the exported policy in the CPU env | `scorecard.json` |
| `--sil` | the **real C++ control code** driving the CPU env | `scorecard_sil.json` |
| `--act-lag-hz 2.0` | the python column at the bench-measured servo lag | `scorecard_lag.json` |

Adding the lag column **inverted the fleet ranking** (2026-09-02): the sim
champion `loco_v25full_c` fell from 96/144 to 16 with 77 % falls, while
`loco_v27tilt_b` scored 46 either way — the only policy whose score did not
move. Judged against the servo we actually own rather than an idealized one,
the "best" policy changed. Treat the columns as a *ranking* under a stated
plant, never as a prediction of hardware behaviour: referee scenarios are much
harder than a bench stand-and-stride.

`sim/build_report.py` rebuilds `sim/runs/night_summary.html` after every round —
a self-contained page with the referee reels cut to the scenarios each run is
judged on, PASS/FAIL burned into the frame. That page, not this doc, is the
running log of results.

---

## 9. Distillation, export, and the road onto the robot

The training net (512, 256, 128) is ~232k parameters, ~930 KB of fp32. The
ESP32 cannot hold it. The decided path is **train big, distill small**:

**`sim/mjx/distill_student.py`** trains a (128, 128) student — ~38k params,
~150 KB, which lives in flash `.rodata` as a `constexpr`, not in SRAM. It is
DAgger, not plain behaviour cloning: round 0 is teacher-driven, then a decaying
fraction `beta` of env slots run the *student's own* actions for whole
trajectories, so the dataset contains the student's compounding mistakes. It
rolls out in the teacher's own env config, reuses the teacher's frozen
normalizer verbatim, and regresses on the teacher's deterministic action
`tanh(mean)` — the quantity the robot actually executes.

Distillation depth turned out to matter: a 12-round student of `v27tilt_b`
scored 28/144 under lag, a 24-round student of the same teacher **41/144**
(teacher 46). DAgger depth was the bottleneck, not student capacity.

Then:

1. `tools/export_policy_weights.py --run <run>` → `sim/sil/weights/<run>.silw`
   (little-endian f32, self-describing header, JSON sidecar) plus golden
   forward-pass vectors.
2. `tools/gen_policy_weights.py --run <run>` → `firmware/components/policy/include/policy/weights.h`,
   with golden vectors recomputed in numpy under the exact brax semantics the
   firmware must reproduce: normalize, swish hidden layers, linear `2*act_dim`
   head, `tanh` of the first half.
3. `tools/gen_obs_spec.py --run <run>` → `obs_spec.h`: layout, joint table, and
   the **action-index → bus-ID permutation**, which is a genuine permutation on
   the 10-DOF plant (the yaw servos were appended as IDs 9/10 after 1–8 were
   assigned, and the two bus chains went onto the opposite legs from the plan).
4. The **SIL gate**: `sim/sil/harness.py` links the real firmware control code
   as `libctrl_sil` and runs it against the CPU plant — sensors flow sim →
   firmware, firmware targets drive the simulated servos, and
   `eval_precision --sil` scores it with the same scenarios. This catches the
   class of bug the Python stack structurally cannot see: byte order, the servo
   permutation, calibration zeros, swish-vs-tanh, a 147-not-129 observation.
5. Only then `idf.py flash`.

Currently flashed: **`loco_v27tilt_b_s128r24`** (147 → 128 → 128 → 20),
deployed 2026-09-03 — 41/144 under the measured servo lag, against 19 for the
`loco_v26lag_s128` it replaced. On the robot it stands 3–6 s before a growing
hip-pitch oscillation takes it, which is the real-robot A/B that the lag column
needed in order to mean anything.

---

## 10. What is solved, and what is not

**Solved.** Sustained command-conditioned walking in sim, at scale, on the
CAD-true plant with honest actuators, under DR that includes the servo terms
measured on the bench. The observation contract is generated end to end and
gated by host tests. The robot **walked on 2026-09-01** under a distilled
policy on its own ESP32, untethered.

**Not solved, in the order they are being worked:**

- **The armed robot oscillates itself over — root cause found 2026-09-03, fix
  not yet armed.** Armed with a zero (stand) command it held a quiet stand for
  about a second, then grew alternating hip-pitch swings at 1–2 Hz until the
  torso passed 25–40°, with a time-to-fall that read as pure chance (2.4–6 s).
  The referee scored the same policy 8/8 under lag *in sim*, and the SIL twin
  stood dead still (up_y RMS 0.0009 vs 0.05–0.13 measured).

  An **observation-freeze ablation** on the robot located it: IMU live → the
  swing grew and it fell at 3.8 s; `up` and gyro frozen at nominal → stood the
  full 8 s; gyro alone frozen → stood; `up` alone frozen → swing guard at 5 s.
  The loop was closing through the IMU observation. A signed bench test against
  the identical move in sim then named it: the gyro integral matched (sim −6.28°
  vs robot −6.53°), but the `up` change did not — sim `(−0.11, 0)` against robot
  `(−0.042, −0.093)`, **the same vector rotated 66°**.

  The observation's `up` is MuJoCo's `framezaxis`: the torso z-axis in the
  **world** frame, which turns with yaw. Every simulated episode resets at
  yaw ≈ 0, so in training that distinction never showed. On the robot the fused
  yaw is a free gyro-z integral with ~0.07°/s of residual bias — 66° in fifteen
  minutes — so the policy's tilt feedback was rotated by an angle that grew over
  minutes, which is exactly why time-upright looked random. The firmware now
  strips ZYX yaw from the quaternion before taking the up-vector
  (`upYawStrippedFromQuaternion`, equal to the sim's up at yaw 0 to 1e-16 over
  2000 random poses), and `obsfreeze` stays as a standing diagnostic.

  **This is the sharpest sim-to-real lesson here so far, and it is not about
  physics.** The plant was fine. One observation channel meant two different
  things on the two sides of the contract, and the sim's initial conditions hid
  the difference for months. Generating the obs *layout* into the firmware —
  which this project already does, and which §9 is proud of — does nothing to
  stop the obs *semantics* diverging.
- Still open alongside it: ~3° of *free* hip-roll play (sim "backlash" is a
  command deadzone; the real joint moves freely under load inside the slop —
  measure → model as hysteresis → re-run the twin sweep until it rocks like the
  robot → only then re-rank the fleet), and an 18 % pitch-gyro over-read, now
  corrected by a per-axis scale in NVS (`imu gscale`). **Neither fix has been
  re-armed on the robot.**
- Also unmodelled by anything currently trained: the ankle **load-reversal
  lurch**, and a 2.5 Hz mode seen on the bench.
- **Natural movement under the measured servo.** Rhythm scores are poor across
  the whole fleet once the lag column is applied. That is the current era's
  problem statement.
- **Get-up.** Parked by decision (2026-07-28): the robot sits up, but the rise
  is a balance corridor PPO has not found in 11 rounds. Assist-to-kneel on
  hardware instead; GPU nights go to locomotion.
- **Terrain blindness.** No height scan; rough ground is handled by DR and the
  terrain mosaic, not by seeing it.
- **Odometry.** The closed-loop scenarios (`square_return`, `goal_home`) are
  driven by ground-truth position in sim. The robot cannot produce that, so
  those scenarios grade the gait and turning controller, not a shippable skill.
  Goal-conditioned locomotion is the next planned abstraction.

---

## 11. Running it yourself

```bash
# --- train (on Mira; the night queue does this unattended) ---
.venv/bin/python sim/mjx/train_mjx.py --out loco_v28 --precision --family loco \
    --steps 60_000_000 --envs 1024 --init-from loco_v27tilt_b

# --- judge (CPU, on the laptop) ---
.venv/bin/python sim/mjx/eval_precision.py --run-name loco_v28              # python column
.venv/bin/python sim/mjx/eval_precision.py --run-name loco_v28 --act-lag-hz 2.0
.venv/bin/python sim/mjx/eval_precision.py --run-name loco_v28 --sil        # real C++ code
.venv/bin/python sim/mjx/eval_ref.py --run loco_v28 --video                 # quick look

# --- shrink and deploy ---
.venv/bin/python sim/mjx/distill_student.py --teacher loco_v28 --out loco_v28_s128 \
    --rounds 24
.venv/bin/python tools/export_policy_weights.py --run loco_v28_s128
.venv/bin/python tools/gen_policy_weights.py --run loco_v28_s128
.venv/bin/python tools/gen_obs_spec.py --run loco_v28_s128
make -C firmware/host test          # golden vectors + SIL must be green
. ~/esp/esp-idf/export.sh && idf.py -C firmware build flash

# --- refresh the visual log ---
.venv/bin/python sim/build_report.py        # -> sim/runs/night_summary.html
```

`sim/train_ppo.py` (Stable-Baselines3, CPU) and the runs under `sim/runs/` with
`model.zip` are the day-1 through day-5 lineage. They are kept for provenance —
the whole `dash_*` / `cmd_*` / `terrain_*` generation predates the 10-DOF plant
and, in most cases, the honest actuator model. Nothing current trains on that
path.
