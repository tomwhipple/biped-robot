# How a walking policy is produced

*The end-to-end account, from the physics model to the weights compiled into
the firmware. The source files named here are the authority for every number;
each run's `config.json` records the exact configuration it trained under.*

**Current state.** Every policy so far was trained on the **10-joint
prototype** — 5 DOF per leg (hip yaw, hip roll, hip pitch, knee, ankle), plant
`sim/bimo_biped_v5body.xml`, servo IDs 1–10 — and the stack is proven on it end
to end: distilled policies run on its ESP32 at 50 Hz, untethered. The training
envs, the trainer and the referee also build the **robot's** plant
(`train_mjx.py --robot`); nothing has been trained on it, and the firmware side
of the port is open (§13). What the robot is: [DESIGN.md](../DESIGN.md).

Related: [AGENTS.md](../AGENTS.md) (working practices, bench safety),
[control-channel.md](control-channel.md) (commands over the link, failsafe),
[sim/sil/README.md](../sim/sil/README.md) (the SIL harness),
[firmware/README.md](../firmware/README.md) (what runs on the ESP32).

---

## 1. What is trained

One neural network. It reads what the robot can sense and writes one position
target per joint, 50 times a second. There is no gait generator, state machine
or per-joint controller underneath it: the network is the controller, and the
servos' own position loop is the only thing below it.

It is **command-conditioned**: the same weights stand, walk forward and
backward, sidestep, turn, crouch, balance on one leg and march, depending on a
7-number command in the observation (§4). The robot is steered by changing the
command, not by switching policies.

```
  sim/bimo_biped_v5body.xml        the plant (§2)
            |
  sim/mjx/env_mjx.py               MJX world: STS3215 actuator model, DR,
            |                      commands, reward (§2-6)
  sim/mjx/train_mjx.py             brax PPO, 1-2k parallel envs, nightly on mira (§7)
            v
      teacher                      (512,256,128) MLP: sim/runs/<run>/params.pkl + config.json
            |
  sim/mjx/eval_precision.py        THE REFEREE: re-run in the CPU env (sim/walker_env.py),
            |                      28 scored scenarios x 8 seeds (§8)
  sim/mjx/distill_student.py       DAgger: teacher -> (128,128) student (§9)
            |
  tools/export_policy_weights.py   params.pkl -> sim/sil/weights/<run>.silw
  make -C firmware/host deploy-headers RUN=<run>
            |                      -> obs_spec.h + weights.h + golden vectors
  eval_precision.py --sil          SIL: the real firmware code driving the CPU plant
            v
      idf.py flash                 50 Hz on the ESP32
```

Each stage catches errors the one before cannot see. MJX and the CPU env are
different implementations, so a reward curve is never evidence (§8). The SIL
gate catches the deployment bugs the Python stack is structurally blind to:
byte order, the servo-ID permutation, calibration zeros, an activation
mismatch, a wrong observation width.

**What the recipe is built on.** Open Duck Mini is the closest working
precedent: the same STS3215 servos, MJX + brax, a procedural reference gait
with a phase-locked imitation reward, and a sim-to-real transfer on hardware.
MuJoCo Playground's biped environments supply the gait-shaping terms and the
PPO settings (§5, §7). Every working small biped surveyed runs an IMU-only,
command-conditioned observation with a short history instead of a velocity
estimate (§3).

---

## 2. The plant

`sim/bimo_biped_v5body.xml` is the `--precision` default in `train_mjx.py`
(`--xml` overrides it). Both engines load the same file.

| | |
|---|---|
| joints | `{L,R}_{hip_yaw, hip_roll, hip_pitch, knee, ankle}`. Hip yaw ±45°, hip roll 25° adduction / 55° abduction, hip pitch −110…+90°, knee ±95°, ankle ±40° |
| mass | 1.11 kg. Per-body inertia from the printed parts' STL meshes, plus the servos, pack and board as boxes at their real places |
| payload | `train_mjx.py --payload` (default 0.154 kg) welds a payload body at the prototype's camera mount, 24 mm aft of and 60.1 mm above the torso centre |
| ground contact | 8 pad spheres per sole, the only floor contact. CAD meshes collide with the floor only in fall and recovery episodes |
| self-contact | inter-leg capsules with enumerated contact pairs, placed at the contact onsets measured on the bench |
| joints, passive | damping 0.1; `--precision` adds Coulomb friction 0.05 N·m and armature 0.028 kg·m² per leg joint (Open Duck Mini's identified STS3215 values) |

**The actuator model** (`env_mjx.step`, mirrored in `walker_env.py`). The MJCF
position actuators are silenced. Each 2 ms physics sub-step (10 per 20 ms
control tick) computes a PD torque `kp·err − kd·q̇` (kp 12 N·m/rad, kd 0.25
N·m·s/rad, fitted to the STS3215; per servo, times `servo_kp_scale`, §13) and
clamps it to the DC-motor torque–speed
envelope `stall·(1 − |q̇|/ω0)`. Stall is 2.94 N·m; the no-load speed is 4.04
rad/s, measured at 12 V (`tools/measure_servo_speed.py`), 14 % under the
datasheet. Both scale with the 11.1 V supply. A policy trained against ideal
actuators asks for torque at speeds no STS3215 delivers at any voltage; the
envelope is what makes a trained gait buildable.

Around the PD loop:

| effect | model | knob |
|---|---|---|
| per-servo stiffness | kp and kd × a factor per servo — the position-loop P raised on chosen IDs, modelled as N × the fitted stiffness on the same envelope, as the design gates model it. `robot` = × 4 on the hip rolls (the STS3250s, the model's credit) and × 2.8 on the ankle rolls and knees (the bench's P 96 / D 0); `planb` = × 4 on all six (DESIGN.md §4); config.json records the resolved `{actuator: factor}` | `servo_kp_scale`, `--servo-kp-scale`; `servo_kp` / `servo_kd` also take one value per actuator |
| floor contacts | `all`: the plant as drawn. `feet` (training only): only the sole pads touch the floor; the servo boxes, the fall colliders and the arms stop colliding with it, and the explicit self-collision pairs stay. Those contacts only matter in a fall, which ends the episode, but MJX reserves and solves their slots every step: on the robot plant `feet` cuts 182 contact slots to 56 and the physics step about 3.5× (laptop CPU). The referee always grades with `all` | `floor_contacts`, `--floor-contacts feet` |
| bus latency | the first sub-steps of each tick still serve the previous target; 0–8 ms per episode, ±1 ms jitter per tick | `latency_ms`, `latency_ms_max`, `latency_jitter_ms` |
| gear backlash | a deadzone of ±lash/2 on the PD error; 0.5–1.0° per episode | `--backlash-deg lo,hi` |
| actuation lag | three cascaded first-order stages on the commanded target (the firmware command shaper's structure), pole drawn per episode | `--act-lag lo,hi` (Hz) |
| dead time | the servo serves the target from 0–N control ticks ago | `--act-delay-max N` |
| quantization | observed q and q̇, and the commanded targets, on the 4096-tick grid and integer steps/s | `--quantize` |
| free play | the link floats inside ±play/2 of the shaft and is driven only at the band edges — hysteresis, where backlash is a deadzone. **CPU plant only** (`walker_env.py`): the SIL twin and the design gates use it, MJX training does not | `play_deg`, `play_joints` |
| rough floor | `sim/terrain_mosaic.npz` (from `sim/mjx/gen_terrain_mosaic.py`): 12 × 6 m of 1 m tiles, 2–6 mm carpet-like texture, a random spawn point per episode | `--terrain` |

The bench numbers behind these knobs: the servo's dead time measures ≈ 85 ms
(4 ticks); a three-stage 2 Hz pole reproduces the joint motion measured in a
bench walk; the prototype's hip-roll chain has ≈ 3° of free play.

**Engine parity.** `walker_env.py` (CPU MuJoCo, one robot) and `env_mjx.py`
(MJX, thousands in parallel) implement the same actuator, observation and
reward arithmetic. `sim/mjx/parity_test.py` checks that synced single steps
agree to float64 precision with latency and backlash active, both airborne and
in settled stance; `tests/test_act_lag_referee.py` pins the lag law in both.
The one real difference is contact-manifold generation: MJX keeps only the
points near the deepest penetration, CPU MuJoCo every penetrating corner, so
trajectories diverge at impacts. DR and the CPU referee are the mitigation.

---

## 3. Observations and actions

Built by `env_mjx._obs` (mirrored in `walker_env._obs`) and generated into the
firmware as `obs_spec.h` by `tools/gen_obs_spec.py`, so the layout cannot drift
between sim and robot by hand. One frame on the prototype:

| block | width | contents |
|---|---|---|
| `q` | 10 | joint angles (minus the per-episode zero offset, §6) |
| `dq` | 10 | joint velocities |
| `up` | 3 | torso up-vector (IMU) |
| `linvel` | 3 | **zeroed** |
| `gyro` | 3 | body angular rate (IMU) |
| `prev_action` | 10 | last tick's action |
| `height` | 1 | **zeroed** |
| `phase` | 2 | gait-clock sin / cos |
| `cmd` | 7 | the command (§4) |

A frame is `3n + 19` wide for n policy joints — 49 here, 55 on the robot's
plant (§13) — and the policy reads the newest 3 frames, so the network input is
**147** (165 on the robot). The history stands in for a state estimator. A
servo the policy does not drive (`held_joints`, §13) is in neither the
observation nor the action.

- **IMU-realizable** (`imu_obs`). Linear velocity and height are zeroed for the
  actor and the critic: the robot has no odometry, and the IMU gives attitude,
  not translation. No privileged critic is used.
- **The up-vector is yaw-stripped on the robot.** In sim, `up` is MuJoCo's
  `framezaxis` of the IMU site — the torso z-axis in the world frame, which
  turns with yaw — and every episode starts at yaw ≈ 0. On the robot the fused
  yaw is a free gyro-z integral that drifts (66° in 15 minutes, measured), so
  the firmware strips the ZYX yaw from the fusion quaternion before taking the
  up-vector (`imu::upYawStrippedFromQuaternion`). That equals the sim's `up` at
  zero yaw; without it the policy's tilt feedback rotates by an angle that
  grows over minutes. The general rule: a generated layout does not protect
  what a channel *means*; check each one means the same thing on both sides.
  On the robot, `obsfreeze up|gyro|imu|dq` freezes a channel in the policy's
  observation only (the guards still see real values) to find a loop that
  closes through it.
- **Sensor error is trained in**: IMU mounting misalignment and gyro bias per
  episode, per-step noise, and optional gyro gain and delay (§6). On the robot
  the QMI8658C is sampled at 250 Hz and fused by `imu::Fusion`; a per-axis
  gyro scale is stored in NVS (`imu gscale`).
- **Gait clock**: a per-episode frequency U(1.25, 1.75) Hz, the feet half a
  cycle apart. `--speed-clock` scales it with the commanded planar speed (√
  law, ×1 at 0.35 m/s, clipped to [0.7, `--speed-clock-hi`]).
  `--clock-freeze-stand` holds the phase under a plain stand command, so the
  policy does not have to ignore an oscillating input to stand still.

**Action.** n values in [−1, 1] — brax's deterministic action, the `tanh` of
the Gaussian mean — mapped to position targets around the home pose, so a zero
action holds home. With `--action-map full` each side of zero spans to that
side's end of the joint's *policy range*: the actuator `ctrlrange` in the
plant, deliberately inside the mechanical stop. Widening a policy range
rescales the action map and costs a retrain.

On the robot the targets do not go straight to the bus: `obs::CommandShaper`
runs three cascaded 10 Hz first-order lags on them, and every SYNC WRITE
carries a per-servo goal speed ([firmware/README.md](../firmware/README.md)).

---

## 4. Commands and skills

Seven channels (`ext_cmd`, set by `--precision`), redrawn every 2.5–4.5 s
(`--cmd-resample-s`):

| ch | meaning | training draw |
|---|---|---|
| c0 `vx` | forward speed, m/s | forward walks U(0.3, 1.0) (`--cmd-v-range`); backward walks U(−0.4, −0.15) |
| c1 `vy` | sidestep, m/s | ±U(0.1, 0.25) |
| c2 `wz` | yaw rate, rad/s | U(−1, 1) on 60 % of forward walks; pivots ±U(0.3, 1.0) |
| c3 `crouch` | torso height × nominal | crouch draws U(0.6, 0.9); otherwise 1, or the `--cmd-crouch-range` draw |
| c4 `lift` | −1 / 0 / +1 | lift left / neither / lift right |
| c5 `foot dx` | swing-foot target, m, torso-yaw frame | driven by the air-circle, march and sway scripts |
| c6 `foot dz` | swing-foot target, m | driven by the same scripts; raised to knee height in the knee-high march |

Every skill is an encoding in these channels, not a mode, so a new skill never
changes the observation contract or the firmware:

| skill | encoding |
|---|---|
| stand | zero motion, crouch 1 |
| walk / backward / sidestep / turn | c0 / c0 < 0 / c1 / c2, alone or combined |
| pivot | c2 only |
| crouch | c3 < 1 |
| one-leg balance | c4 = ±1 |
| air circles | c4 = ±1, c5/c6 on a circle (radius 2–5 cm, period 1.5–3.5 s) |
| march in place | c4 alternating, c6 bobbing the swing foot |
| knee-high march | c4 alternating on the clock (or at `--march-hz`), c6 raising the swing sole to the opposite knee's height (`--march-mix`) |
| hip sway | c1 oscillating at 0.12 m/s amplitude, feet planted |

`--family` restricts the mix to a specialist, and the referee then scores the
run only on that family's scenarios (§8):

| family | command mix |
|---|---|
| `loco` | 20 % stand, 15 % pivot, 65 % walk (of which 25 % backward, 30 % sidestep, 45 % forward); gait imitation on (`w_mimic` 1.5) |
| `skills` | 15 % stand, 15 % crouch, 25 % balance, 20 % air circles, 15 % march, 10 % sway; no walking |
| `getup` | every episode starts fallen (§11) |
| `all` | the `--precision` mix: stand 15, crouch 8, balance 12, circles 10, pivot 8, march 10, sway 7 %, the rest walking; 15 % of env slots start fallen |

`--ext-mix` overrides the seven fractions after the family preset.

**On the robot** the console sends the command over the UDP link. When the
link goes stale the command decays to the stand `(0, 0)` — a trained command
the policy practises constantly, not a bolted-on pose — and after 5 s of
silence the servos release. The robot also snaps commands outside the trained
envelope to a stand, so `V_MIN_WALK` / `V_MAX` in the link protocol must follow
`--cmd-v-range` ([control-channel.md](control-channel.md)).

---

## 5. The reward

The shape: **a primary tracking term, gated by whether the commanded skill is
being performed, plus posture and shaping, minus costs.**

```
primary = g_skill * (v_term + w_term) + h_term
reward  = primary + w_upright * up_z + alive_bonus
          - w_height * |h - h_ref| - w_energy * sum|tau*qd| - w_action_rate * sum(da^2)
          - w_power * P_elec
          + skill terms + gait shaping + imitation - regularization
          - fall_cost                                  (on the terminating step)
```

- `v_term`: under a motion command (|c0, c1| > 0.05) with `cmd_dense`, the
  velocity projected on the commanded direction over the commanded speed,
  clipped to [−1, 1] — zero for standing, negative for the wrong way.
  Otherwise the kernel `exp(−((vx−c0)² + (vy−c1)²)/0.25)`.
- `w_term`: yaw-rate kernel `exp(−((wz−c2)/0.5)²)`, plus `w_heading ×` a
  heading kernel (σ 0.25 rad) against c2 integrated since the last command
  change, so chronic under-turning accumulates error.
- `h_term`: height kernel (σ 4 cm) against the crouch target; at 0.3 weight
  while motion is commanded.
- `g_skill`: under a lift command `0.2 + 0.8 × lift_ok` (stance foot down,
  swing foot up by ≥ 3 cm); under a crouch `0.2 + 0.8 ×` the height kernel;
  otherwise 1.
- `P_elec`: `Σ max(τ·q̇, 0) + 3.75·τ²` W — mechanical output plus copper loss
  at the STS3215's stall calibration.

Weights as the `--precision` preset sets them (`train_mjx.py`); every one is a
flag:

| layer | terms (preset weight) |
|---|---|
| tracking | `w_track_v` 2, `w_track_w` 2, `w_track_h` 1; `w_heading` off by default |
| posture, alive | `w_upright` 0.8, `alive_bonus` 0.3, `w_height` 0.3 |
| costs | `w_energy` 0.002, `w_action_rate` 0.15, `w_power` 0.008, `w_pitch_rate` 0.1 (torso roll + pitch rate), `w_lateral` 0.5 (vy error) |
| skills | `w_lift` 1 (correct one-foot contact with clearance), `w_track_foot` 1 (swing-foot target, σ 6 cm), `w_foot_cross` 0.5 (soles closer than sole width + 5 mm); optional `w_knee_high`, `w_com_stance`, `w_foot_under` |
| gait shaping (moving commands; slip always) | `w_feet_air` 5 (air time toward 0.3 s), `w_single_support` 0.3 (single support while moving, double while standing), `w_feet_phase` 1 (swing height follows the clock, 6 cm peak), `w_feet_slip` 0.25, `w_symmetry` 1 (swing-duration mismatch at touchdown); optional `w_contact_sched` (stance/swing timing) |
| regularization | `w_orientation` 1 (up_x² + up_y²), `w_ang_vel_xy` 0.15, `w_pose` 0.3 (every policy joint toward home, plain stand/walk only), `w_dof_limits` 1 (past 90 % of the policy range) |
| imitation | `w_mimic` 1.5 in the `loco` family |
| stand, crouch | optional: `w_still`, `w_stand_home`, `w_stand_com`, `w_stand_knee`; `w_crouch_pull`, `w_crouch_track`, `--crouch-pose-ref`, `--crouch-deep-ref`, `--crouch-rsi-mix` |
| fall | `fall_cost` 10, on top of termination (torso below 0.18 m × c3, or up_z < 0.4) |

**The imitation prior** (`_mimic_ref`, Open Duck Mini's recipe) generates
joint targets from (vx, vy, wz, phase): a sinusoidal hip-pitch stride scaled by
command over clock frequency, a differential stride for turning, a hip-roll
oscillation for sidestep, a 0.55 rad knee bend in swing, and the ankle keeping
the sole flat. With `--mimic-sole-level` (on in the robot preset) the ankle
pitch is solved from the joint axes and the sole stays level through the whole
swing; without it — every prototype run — the ankle cancels the hip but not
the knee bend (the knee axis is −y), so the swing sole tilts toe-down by twice
the bend. Where the plant has an ankle roll, it cancels the hip roll. At zero
command the reference is the home pose. It pays
`exp(−Σ w_j (q − q_ref)² / 0.72)` over every policy joint, with the knee
weighted by `--mimic-knee-w`.
It supplies backward and sideways walking, which shaping alone did not
produce, and it carries the swing timing.

`--w-mirror-loss` adds an auxiliary PPO loss (not a reward) on
`|π(mirror(obs)) − mirror(π(obs))|`, with the signed left/right permutations
derived from the plant in `sim/mjx/mirror.py`: symmetry pressure on the policy
itself, on every sample, where the touchdown penalty only acts at touchdowns.

Rules the arithmetic encodes:

1. **The policy optimizes the reward you wrote.** If standing pays under a
   motion command, it stands: with a severe fall cost and forgiving kernels, a
   stand collects most of the tracking income at no risk. Hence the dense
   directional term (standing earns nothing) and the skill gate (ignoring a
   lift or crouch cuts the tracking income to 20 %).
2. **Kernel width is a gradient decision, not a tolerance.** A 3 cm foot
   kernel pays nothing at a 15 cm error, so the policy never starts climbing;
   the foot kernel is 6 cm, and the knee-high reward is linear in the fraction
   of the target reached.
3. **Pay progress for one-shot skills, not a state.** Absolute height pays a
   motionless sitting robot forever; the height ratchet pays only new height
   above the episode's best. A stand counts as reached only when held 0.5 s
   in a row.
4. **Gate a term that fights a skill** instead of weakening it: pose
   regularization applies only under plain stand and walk, the stand terms
   release under a crouch command, and the imitation term can be gated off
   under a crouch (`--mimic-crouch-gate`, `--mimic-crouch-gate-knee`).
5. **Warm-start only within the same objective.** An objective change trains
   from scratch, and so does anything that invalidates the old weights: a new
   observation layout, a new actuator contract.

---

## 6. Domain randomization

The hardware contract. **The sim gets a hardware term only after the bench
measures it** ([AGENTS.md](../AGENTS.md)): a guessed range trains a policy
robust to something the robot does not do and fragile to what it does.

**Batch level** — one randomized model per parallel env, drawn once per batch
size (`domain_randomize`, the standard MJX pattern):

- body mass and inertia × U(0.85, 1.15), per body;
- floor friction × U(0.6, 1.4);
- gravity tilted by U(0, `--tilt-max`)° about a random azimuth — an un-level
  floor, as the IMU sees it (off by default);
- payload mass U(0, `payload_max`), when that is set.

**Per episode** — redrawn at every reset. `reseed()` keeps them alive across
auto-reset; brax's stock `AutoReset` restores the cached first state and would
freeze them.

- servo kp and kd × U(0.8, 1.2); stall torque and no-load speed × U(0.85, 1.15);
- latency 0–8 ms (±1 ms per tick) and backlash 0.5–1.0° (§2);
- IMU mounting misalignment (a rotation of N(0, 2°) about a random axis), gyro
  bias U(±0.03 rad/s), and per-step noise (σ 0.01 on `up`, 0.03 rad/s on the
  gyro);
- velocity kicks U(0.1, 1.0) m/s at 1 % per tick (`--kick-range`,
  `--push-prob`);
- start-pose jitter ±1.7° per joint (`--init-pose-deg`);
- the gait-clock frequency and the command draws (§3, §4).

**Optional per-episode terms**, each tied to a bench measurement:

| flag | draw | why |
|---|---|---|
| `--act-lag lo,hi` | actuation-lag pole, Hz | the servo's tracking lag; 2 Hz reproduces the measured bench walk |
| `--act-delay-max N` | servo dead time, 0…N ticks | ≈ 85 ms measured |
| `--zero-offset-deg z` | per-joint zero offset U(±z): the served target is target + offset, the observation subtracts it | hardware zeros are set by eye and move by degrees after a mechanical disturbance |
| `--gyro-gain-range lo,hi`, `--gyro-delay-max N` | gyro observation gain; delay ≤ 2 ticks | IMU scale and timing error |
| `--cmd-crouch-range lo,hi` | crouch-channel draw | the firmware battery guard lowers c3 on a sagging pack; a frozen channel also collapses its normalizer |
| `--terrain`, `--quantize` | rough floor; tick quantization | §2 |

`--discovery` strips DR, payload, latency, backlash and pushes: a clean-physics
stage for finding a skill at all, before a warm-started stage puts the
hardening back.

---

## 7. The algorithm and where it runs

`sim/mjx/train_mjx.py` runs brax PPO over `BatchedEnv`, a hand-rolled batch
wrapper (`wrap_env=False`) that owns the vmapping, truncation at the episode
length, auto-reset with `reseed()`, and the batch-level DR.

| knob | default (`train_mjx.py`) |
|---|---|
| parallel envs | 2048 (`--envs`); 1024–2048 fit mira's 12 GB |
| steps | 150 M (`--steps`); the night runner caps it to the window |
| episode | 10 s at 50 Hz = 500 steps |
| policy / value nets | (512, 256, 128) / (512, 256, 128) with `--precision` (otherwise (128, 128) / (256, 256)); swish |
| learning rate | 3e-4 |
| entropy | 5e-3 with `--precision` (1e-2 otherwise) |
| discount | 0.97 |
| unroll / minibatches / updates / batch | 20 / 32 / 4 / 1024 |
| evals | 20; `params.pkl` is saved at every eval |
| obs normalization | brax `running_statistics`, shipped with the weights |

**Warm starts.** `--init-from <run>` restores that run's `params.pkl`; a lineage
accumulates capability instead of relearning walking every night (§5 rule 5
says when not to).

**Provenance.** Every run writes `config.json` next to its `params.pkl`: the
full env kwargs, the trainer arguments, and `git_sha` / `git_dirty` of the tree
that trained it. The referee, the distiller and the obs-spec generator all
rebuild the run's world from that file, never from a hand-written
approximation. `progress.jsonl` logs the eval curve.

### Nightly on mira

mira is a Linux box with an RTX 4070 Ti (12 GB), reached as `ssh mira`.
Training runs **at night only**; days belong to the box's owner.

- **Window.** cron starts `infra/night/night_run.sh` at 22:00. No new job starts
  after 05:00; `infra/night/night_stop.sh` stops a running job at 07:00, and
  the last eval checkpoint stands.
- **Committed code only.** `~/code/robot-mjx` on mira is a git clone. Before
  every job the runner fetches and hard-resets it to `origin/main`, or to the
  branch a job names with `BRANCH=`. Anything not pushed does not train. The
  runner scripts sit in the clone's untracked `night/` directory, so a reset
  cannot rewrite them mid-run.
- **Arming.** From the laptop, `infra/night_arm.sh [--branch <name>]
  <train_mjx.py args>` checks that the branch is pushed, installs the runner and
  its cron on mira, and queues a job file `night/queue/<MMDDhhmmss>-<out>`.
- **Queue.** Jobs run in sorted order; the queue is rescanned after every job.
  A job file moves to `queue/done/` at launch, so an interrupted job does not
  re-run the next night. `CMD=` jobs run an arbitrary command (a distillation,
  say) through the same GPU gate. `--steps` is capped to what the remaining
  window allows.
- **Sharing the GPU.** A job starts only when the GPU is under 20 % busy with
  ≥ 11.5 GB free. Idle ollama models are unloaded (they reload on demand),
  active inference is never touched, and after each job any model that fell
  back to CPU is moved back onto the GPU.
- **Morning.** `infra/night_collect_local.sh` (cron 07:15 on mira) copies each
  finished run into the working checkout and referees it: the python column
  with `--render`, and the SIL column. The lag column is run by hand (§8).

### Burst compute on RunPod

`infra/runpod/` rents a GPU pod through `runpodctl` when mira is busy or too
small. The same rule applies: train from a git clone of committed code. The
runbook and its gotchas are in [infra/runpod/README.md](../infra/runpod/README.md).

---

## 8. The referee

**Never trust MJX numbers alone.** A reward curve shows that a run is learning
something, not what. Every policy is re-run on the other engine and graded on
tasks. And a referee column is **a ranking under a stated plant, not a
prediction of hardware behaviour**: the scenarios are harder than a bench
stand-and-stride, and the plant is only as true as its last bench calibration.

`sim/mjx/eval_precision.py --run-name <run>` loads the run's `config.json` and
checkpoint, rebuilds its world in the CPU `BimoWalkerEnv`, and drives every
scenario — a per-step command script, most opening with 1 s of stand to settle
— for 8 seeds (`--episodes`). A fall at any point fails the seed.

The conditions are **pinned by the referee, not inherited** from the run's
training DR, so runs trained under different DR stay comparable: full DR, a
154 g payload (the prototype's GoPro; none on the robot's plant, which models
its head camera), 4 ms latency, 0.7° backlash, IMU observations, 5 N shoves at
1 % per tick. `--nominal` strips DR, latency, backlash and shoves. The plant's
own settings in `config.json` — held joints, per-servo stiffness, the hip
range — are inherited: they are the robot, not training conditions.

**Three columns**, not interchangeable:

| column | flag | what drives the plant | writes |
|---|---|---|---|
| python | — | the exported policy in the CPU env | `scorecard.{md,json}` |
| SIL | `--sil` | the real firmware control code (`libctrl_sil`: obs assembler, history, gait clock, MLP, angle↔tick calibration); build it with `make -C firmware/host sil` | `scorecard_sil.*` |
| lag | `--act-lag-hz 2.0` (+ `--act-delay-ticks 4`) | the python column with the bench-measured servo lag (and ≈ 85 ms dead time) in the plant | `scorecard_lag.*` |

`--sil --act-lag-hz 2.0` combines them (`scorecard_sil_lag.*`). `--scenarios
a,b` runs a subset into `*_partial` files, so a full card is never
overwritten. `--render` records seed 0 of every scenario into one reel
`<run>.mov`, captioned PASS or FAIL — a failing take is labelled, never left
implicit.

### Scenarios

28 in the default suite. A run records its family in `config.json` and is
scored on that family only: `loco` 18 scenarios (hence the **/144** totals, 18
× 8 seeds, that rank the locomotion fleet), `skills` 12, `getup` 2, `all` 28.
Pass criteria, in addition to not falling:

| scenario | family | script | pass |
|---|---|---|---|
| `line_1m` | loco | vx 0.4 to +1 m, then stand | crosses within 8 s; \|y\| < 0.15 m at the line; overshoot < 0.30 m; settles (< 0.15 m/s for 1 s) |
| `line_rough` | loco | `line_1m` on the terrain mosaic | as `line_1m` |
| `backward_1m` | loco | vx −0.3 to −1 m | reaches it with \|y\| < 0.20 m; settles |
| `sidestep_L/R` | loco | vy ±0.2 to 0.5 m lateral | within 6 s; x drift < 0.20 m; settles |
| `turn_180` | loco | wz 0.7 for π/0.7 s | heading within 15° of reversed; excursion ≤ 0.3 m |
| `square_return` | loco | 1 m square by waypoints (P steering on ground truth) | all 4 waypoints; ends < 0.25 m from start |
| `circle_return` | loco | vx 0.35, wz 0.7 for one circle | ends < 0.35 m from start; mean radius ≥ 0.3 m |
| `goal_home` | loco | the circle, then P homing with body-frame creep inside 0.3 m | ends < 0.10 m from start; radius ≥ 0.3 m |
| `push_gauntlet` | loco | vx 0.4 under 0.6–1.4 m/s kicks at 8 % per tick | ≥ 2.0 m covered; lateral ≤ 0.5 m; heading within 35° |
| `speed_ladder` | loco | vx 0.1 → 0.3 → 0.6 → 0.9 → 1.2 m/s, 4 s rungs | speed MAE ≤ 0.20 m/s over each rung's back half |
| `pursuit` | loco | chase a target moving at 0.18 m/s | final gap ≤ 0.40 m |
| `reversal` | loco | vx 0.5 → −0.3 → stop | velocity inside the band for 0.5 s within 2 s of each flip |
| `stand_10s` | loco, skills | stand | max drift < 0.15 m |
| `stand_off` | loco | 1.5 s powered, then torque released | still standing (height ≥ 85 %, up_z ≥ 0.9); drift < 0.10 m. Passive resistance is an *estimate* of unpowered backdrive friction |
| `metronome` | loco, skills | march at 1.4 → 1.0 → 1.7 Hz, 6 cycles each | cadence error ≤ 15 % at 1.4 and 1.7 Hz (1.0 Hz, outside the trained clock band, is reported only); drift < 0.15 m |
| `squat_reps` | loco, skills | 3 × (crouch 0.70 for 1.5 s, stand 1.5 s) | every rep within 3.5 cm of the crouch target and back to ≥ 92 % height; drift RMS < 0.10 m |
| `weight_shift` | loco, skills | 1.5 s single-leg holds L, R, twice, at crouch 0.85 | ≥ 3 of 4 clean holds; drift RMS < 0.10 m |
| `balance_L/R` | skills | lift one foot for 10 s | lifted-foot contact < 5 %; ≥ 3 cm clearance for ≥ 90 % of the window; drift RMS < 0.10 m |
| `circle_air_L/R` | skills | lift, then 2 circles of 4 cm with the foot | touchdown < 10 %; clearance ≥ 3 cm for ≥ 80 %; traced radius ≥ 3 cm and ≥ one full sweep |
| `crouch_hold` | skills | crouch 0.7 for 3 s, then stand | height within 3.5 cm of target; recovers to ≥ 90 % |
| `march_in_place` | skills | alternating lifts every 0.7 s | ≥ 3 clean lifts per leg (≥ 0.3 s at ≥ 3 cm); drift < 0.15 m |
| `march_10s` | skills | knee-high march for 10 s | ≥ 6 alternating lifts peaking above 60 % of knee height; drift < 0.15 m |
| `hip_sway` | skills | vy = 0.12 sin(2πt/1.6) for 6 s | lateral amplitude ≥ 2 cm; net drift < 0.15 m |
| `recover_sit` | getup | start seated, command stand | held stand within 5 s; ≥ 85 % height at the end |
| `recover_fallen` | getup | start from a settled ragdoll | held stand within 6 s; ≥ 85 % height at the end |

(Heights are fractions of the plant's nominal torso height.) `handover_stand`
exists behind `--scenarios` only, so it never moves the comparable totals.

**Metrics.** Pass/fail is a threshold view; every seed also records continuous
metrics — endpoint and tracking errors, clearance fraction, drift RMS,
time-to-target — plus a shared panel: **wobble** RMS (torso roll + pitch rate),
mean electrical **watts**, **cost of transport** P/(m·g·v̄), **foot slip** (sole
planar speed while in contact), and left/right **air-time asymmetry**.
`scorecard.json` holds the mean and the worst across seeds.

`sim/mjx/eval_ref.py --run <run> [--video]` is a quick six-scenario check of
plain command tracking (walk 0.6 m/s, slow 0.35, stand, pivot ±0.5 rad/s, an
arc).

### Reading the columns

- **Rank on the lag column.** Under the measured servo lag the ordering
  changes: a policy that scored 96/144 without lag scored 16/144 with it (77 %
  falls), while another held its score in both.
- **A python-vs-SIL gap in one direction on one scenario is a finding**, not
  noise: it points at a real deployment property (the command shaper, the
  permutation, quantization). Long-horizon scenarios wander by 2–3 seeds either
  way on their own.
- **Grade a claim under the conditions it claims**, with real minimums: a
  "balance" that is a 2 mm hover, or an "air circle" that never traces one,
  fails.
- The closed-loop scenarios (`square_return`, `goal_home`, `pursuit`) steer on
  the simulator's ground-truth pose, which the robot does not have (§11).

---

## 9. Distillation and deployment

The (512, 256, 128) teacher is ≈ 232k parameters (≈ 930 KB fp32), too large for
the ESP32. `sim/mjx/distill_student.py` trains a **(128, 128) student** —
≈ 38k parameters, ≈ 150 KB — that lives in flash as a `constexpr`.

- **DAgger, not behaviour cloning.** Round 0 is teacher-driven; from round 1 a
  fraction β of env slots is teacher-driven (β halves each round: 0.5, 0.25, …)
  and the rest run the student's own actions for whole chunks, so the dataset
  holds the student's compounding mistakes.
- Rollouts run in `BimoMJXEnv` from the **teacher's own `config.json`**; the
  teacher's frozen normalizer is reused verbatim; the label is the teacher's
  deterministic action `tanh(mean)`, the quantity the robot executes.
- **Depth over width: run 24 rounds** (the default is 12). On the same teacher
  a 24-round student scored 41/144 under lag against 28/144 for 12 rounds
  (teacher 46). Name students `<teacher>_s128r24`.
- The student is saved as a brax PPO network (`params.pkl` + `config.json`), so
  all three referee columns score it with no shim.

**Deployment**, in order:

1. `tools/export_policy_weights.py --run <student>` →
   `sim/sil/weights/<student>.silw` (little-endian f32 with a self-describing
   header, plus a JSON sidecar) and golden forward-pass vectors in
   `sim/sil/golden/`.
2. `make -C firmware/host deploy-headers RUN=<student>` runs both generators;
   the headers change together, and only when someone names the run:
   - `tools/gen_obs_spec.py --run` → `obs_spec.h`: the layout, joint table,
     limits, home pose, clock settings, and the action-index → bus-ID
     permutation `kServoId` (from `ID_BY_ROLE` in the generator — a genuine
     permutation on the prototype), plus `obs_vectors.h` from `walker_env._obs`.
   - `tools/gen_policy_weights.py --run` → `weights.h`, plus `policy_vectors.h`
     computed in numpy under the brax semantics the firmware reproduces:
     normalize, swish hidden layers, a linear `2·act_dim` head, `tanh` of its
     first half.
3. `make -C firmware/host test` (golden vectors, host suites) and the SIL
   column, `eval_precision.py --run-name <student> --sil`.
4. **Pre-flash checks**: `stand_10s` 8/8 and `stand_off` ≥ 6/8 on the lag
   column; the first action from the home observation ≤ 0.05 on every joint
   (`sim/sil/harness.silw_forward`); and the SIL twin (`sim/sil_twin.py`, the
   firmware stack behind the real UDP link) resting within 2° of home on every
   joint, torso within 1°, over the last 3 s of a 6 s stand.
5. Build and flash: `idf.py -C firmware build flash` (toolchain setup in
   [firmware/README.md](../firmware/README.md)). Flashing reboots the board:
   support the robot first ([AGENTS.md](../AGENTS.md), bench safety).

### Currently deployed

**`loco_v41rsi_b_s128r24`** (`obs::kRunName` and `policy::kWeightsRun` agree): a
(128, 128) student distilled over 24 DAgger rounds from the `loco`-family
teacher `loco_v41rsi_b`, which trained with part of its episodes starting in
the squat reference (`--crouch-rsi-mix`). Plant `bimo_biped_v5body.xml`; net
147 → 128 → 128 → 20; action map `full`; IMU observations; gait clock with the
stand freeze and the speed clock (×0.7–1.25). Its scorecards and reels are on
the results page.

---

## 10. The results page

`sim/build_report.py` rebuilds **`sim/runs/night_summary.html`** after every
training round: a self-contained page with images inlined, and the referee
reels cut into web-playable clips in `sim/runs/_clips/` beside it, each cut to
the scenarios its run is judged on, PASS/FAIL burned in. That page is the
running record of results; this document is not. `sim/runs/` is gitignored
except `night_summary.html`; movies and clips are never committed.

---

## 11. Limits and the next layer

- **Get-up is not trained.** An RL get-up is not pursued: across 12 rounds and
  about 1.3 B steps, PPO reward shaping reached the kneel but could not bridge
  kneel → balance-catch → held stand — including a clean-physics run with no
  DR at all, which settled that the hardening was not the obstacle. The robot
  gets up with a **scripted, arm-assisted sequence** instead ([DESIGN.md](../DESIGN.md),
  "Get-up"). If an RL get-up is ever revived, the method is **phase-indexed
  tracking** — a DeepMimic-style pose-and-velocity tracking objective at
  dominant weight over a retimed kneel-corridor trajectory, with
  reference-state initialization along it — not more shaping. The machinery
  stays in the env: the `getup` family, recovery episodes (`recover_mix`,
  `recover_start_mix`), the height ratchet and held-stand rule, `w_rise_ref`,
  `--discovery`, and the two `recover_*` scenarios.
- **Goals.** The planned next layer is **goal-conditioned locomotion**: a goal
  in the observation, and a policy judged on distance covered and goal
  reached, not on instantaneous velocity. Today the goal layer is an outer
  command loop in the referee: `goal_home` wraps a P controller on bearing and
  distance (body-frame creep inside 0.3 m) around a policy trained on the creep
  band (`--cmd-v-range 0.05,1.0`). It steers on ground truth; the robot has no
  pose estimate — the IMU gives attitude only — so a position source on the
  robot is open, and the closed-loop scenarios grade gait and turning, not a
  shippable skill.
- **Terrain.** No exteroception (issue #11); rough ground is handled by DR on
  the terrain mosaic.
- **Natural gait.** The procedural imitation prior is the gait-quality lever.
  AMP-style adversarial imitation (LocoMuJoCo) is the candidate if the gait
  still looks robotic; it is not built.

---

## 12. Running it

```bash
# --- train: queue for tonight on mira (push first; the clone trains origin) ---
infra/night_arm.sh --out loco_vN --precision --family loco \
    --steps 150000000 --init-from <run> --act-lag 2,12
#     or directly on mira: .venv/bin/python sim/mjx/train_mjx.py <same args>
#     the robot's plant instead of the prototype's: add --robot (§13)

# --- referee (CPU; laptop or mira) ---
JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_precision.py --run-name loco_vN
JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_precision.py --run-name loco_vN \
    --act-lag-hz 2.0 --act-delay-ticks 4
make -C firmware/host sil && \
JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_precision.py --run-name loco_vN --sil
.venv/bin/python sim/mjx/eval_ref.py --run loco_vN --video          # quick look

# --- distill (GPU: a CMD= job in the night queue, or by hand on mira) ---
.venv/bin/python sim/mjx/distill_student.py --teacher loco_vN \
    --out loco_vN_s128r24 --rounds 24

# --- deploy ---
JAX_PLATFORMS=cpu .venv/bin/python tools/export_policy_weights.py --run loco_vN_s128r24
make -C firmware/host deploy-headers RUN=loco_vN_s128r24
make -C firmware/host test
idf.py -C firmware build flash                  # toolchain: firmware/README.md

# --- results page ---
.venv/bin/python sim/build_report.py            # -> sim/runs/night_summary.html
```

---

## 13. Porting to the robot

The training envs, the trainer and the referee run on the robot's plant;
nothing has been trained on it. `train_mjx.py --robot` applies the robot
preset (`sim/mjx/robot_plant.py`), resolved against the plant's own actuators
and written into `config.json` explicitly, so the referee, the distiller and
the obs-spec generator rebuild the same robot:

| piece | on the robot's plant |
|---|---|
| plant | `sim/bimo_biped_v6ar.xml`, generated by `sim/gen_plant_v6.py` with CAD inertials from `sim/build_v6_inertia.py`. The committed file carries the 12 leg joints and the neck yaw; the arms (shoulder pitch + elbow per side) come with the CAD track's regeneration and need nothing here — every layout is read from the model, and `tests/test_robot_plant.py` runs a generated 17-servo plant today. No payload: the camera is in the head, which the plant models |
| policy joints | the 12 leg joints, 6 per leg with the ankle roll: frame 55, network input 165, action 12 |
| neck and arms | **held, not driven.** `held_joints` gives each a fixed servo target: neck 0°, the arms at their folded rest pose (`dimensions_v6.ARM_REST`: shoulders 15° forward, elbows folded 95°; DESIGN.md §5.4), the pose that clears every leg joint's full range. A held servo runs the same actuator model as the legs (PD, envelope, lag, dead time, backlash, ticks, zero offset), starts the episode at its target (it is the plant's `qpos0`), and is in neither the action nor the observation; torque and power metrics include it. Why not actions: the walking hold is a fixed pose by design, four arm actions would only have to learn to stay still, and the neck belongs to the Pi's camera later |
| servo stiffness | per servo (`servo_kp_scale`, §2): the `robot` preset, × 4 on kp and kd of the hip rolls (STS3250) and × 2.8 on the ankle rolls and knees (STS3215 at P 96). The STS3250's larger stall and speed are not modelled: one STS3215 envelope for every servo |
| hip pitch | policy range to −120° flexion (`hip_flex_deg`), the leg link's relief; the plant's stop stays at −125° |
| reward roles | every policy joint must resolve to a role (`joint_role` in both envs: the six per-leg roles, neck, shoulder, elbow) or the env refuses to build — a joint a plant adds is held or named, never trained unregularized by accident. The posture, dof-limit and imitation terms sum over every policy joint, so the ankle roll is in them; the imitation reference keeps the sole level in pitch and roll (`mimic_sole_level`, §5) |
| mirror | `mirror.py` maps over the policy joints (`joints=env._act_names`, as `--w-mirror-loss` passes them) and maps an unprefixed joint onto itself |
| referee | `eval_precision.py` recognises the robot's plant by its ankle rolls and head camera: no pinned payload; `recover_sit` and `recover_fallen` are n/a (the robot gets up with the scripted arm sequence; legs alone cannot get this body up), `handover_stand` is n/a; every other scenario runs. n/a scenarios are listed in the scorecard and left out of the totals. `eval_ref.py` rebuilds the same world from `config.json` |
| parity | `sim/mjx/parity_test.py` blocks R1–R3 gate the CPU/MJX arithmetic on the robot's plant — Plan B, the held neck, a 17-servo plant with the arms held, the sole-level reference, quantization, airborne and in stance; `--robot-only` runs just those. `tests/test_robot_plant.py` is the smoke test |

**Open before Gate E:**

| piece | what remains |
|---|---|
| measured plant | the masses weighed (#82) replacing the CAD inertials; the stiffness factor from the bench (#73) replacing the nominal × 4 |
| free play | modelled in the CPU plant only (`play_deg`, `play_joints`); the measured play per roll joint has to reach the training env and the referee's pinned conditions, since the walk's margins depend on it (DESIGN.md, "Walking") |
| referee thresholds | the absolute thresholds (distances, drifts, the torque-off stand's unmeasured passive friction) are the prototype's; re-derive them for the taller, heavier robot. The plant-measured targets (nominal height, knee height, crouch depth) already follow the plant; the crouch-depth limit (`crouch_theta_max_deg` 39°) and the rise and fallen-start poses are the prototype's numbers |
| firmware headers | `tools/gen_obs_spec.py` builds the env and the golden frames from a robot run as it stands, but has no servo IDs for the robot (`ID_BY_ROLE`; none is assigned yet, [servo-map.md](servo-map.md), #77) and emits neither the held servos' IDs and targets nor the per-servo P the firmware must check at boot (#81); `weights.h` and the golden vectors regenerate with it |
| SIL twin | `libctrl_sil`, its ABI and calibration files, and `sim/sil_twin.py` for the 17-servo layout; `eval_precision.py --sil` refuses the robot's plant until then |

**When.** The design is validated in stages (DESIGN.md, "Validation method"):
kinematics, actuator margins and contact realism first, then the scripted
open-loop walk in the plant and on the bench. **A policy (Gate E) comes only
after those open-loop hardware gates pass**, trained on the plant with the
measured masses, stiffness and play in it.
