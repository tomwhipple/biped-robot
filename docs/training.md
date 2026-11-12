# How a walking policy is produced

*The end-to-end account, from the physics model to the weights compiled into
the firmware. The source files named here are the authority for every number;
each run's `config.json` records the exact configuration it trained under.*

**Current state.** Policies train on the **robot's plants**:
`sim/bimo_biped_v6ar.xml` (the dressed 17-servo robot, the default plant) and
`sim/bimo_biped_v6legs.xml` (the legs-only body as built, 12 leg joints).
Walking and get-ups both train there (§11, §13). Distilled students of a legs
walk run on the ESP32 at 50 Hz, as float and int8 nets (profiles `v6legs` and
`v6legs256`, §9); the deployed one, `robot_legs_walk16_act_s128r24`, holds the
built legs standing ([first-policy-stand.md](first-policy-stand.md)). Walking
on the floor is not yet shown on hardware, and a policy for all 17 joints and
its firmware are open (§13). What the robot is:
[DESIGN.md](../DESIGN.md).

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
  sim/bimo_biped_v6ar.xml          the plant (§2; v6legs for the legs-only body)
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

`sim/bimo_biped_v6ar.xml` is the default plant of `train_mjx.py`, `env_mjx`,
`walker_env` and `eval_precision.py` (`--xml` overrides it):
`bimo_biped_v6legs.xml` is the legs-only body as built, and the `*_meas.xml`
variants carry the joint stops measured on the robot. Both engines load the
same file. The robot preset (`--robot`) is §13.

| | |
|---|---|
| joints | `{L,R}_{hip_yaw, hip_roll, hip_pitch, knee, ankle, ankle_roll}`, and in the ar plant `neck_yaw` and `{L,R}_{shoulder, elbow}`. Policy ranges: hip yaw ±45°, hip roll 30° adduction / 45° abduction, hip pitch −120…+90°, knee −130…+5°, ankle ±40°, ankle roll ±25° |
| mass | 2.48 kg (ar), 1.51 kg (legs). Per-body inertia from the printed parts' STLs, the servos at their CAD places, and the pack, boards and bearings at their real places (`sim/build_v6_inertia.py`) |
| payload | none on the robot: the camera is in the head, which the ar plant models. `train_mjx.py --payload` (default 0.154 kg, 24 mm aft of and 60.1 mm above the torso centre) applies only without `--robot` |
| ground contact | 8 pad spheres per sole, the only floor contact. CAD meshes collide with the floor only in fall and recovery episodes. On the robot's measured-sole plant the pads carry the measured sole friction (§6) |
| self-contact | inter-leg collision geoms with enumerated contact pairs |
| joints, passive | the plant's damping 0.6, friction 0.02 N·m and armature 0.008 kg·m²; `--precision` sets Coulomb friction 0.05 N·m and armature 0.028 kg·m² per leg joint (Open Duck Mini's identified STS3215 values) |

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
| per-servo stiffness | kp and kd × a factor per servo — the position-loop P raised on chosen IDs, modelled as N × the fitted stiffness on the same envelope, as the design gates model it. `robot` = × 4 on the hip rolls (the STS3250s, the model's credit) and × 2.8 on the ankle rolls and knees (the bench's P 96 / D 0); `planb` = × 4 on all six (DESIGN.md §4); `asbuilt` = the gains the robot arms with (`firmware/main/servo_gains.h`: factory P 32 / D 32 everywhere, which the bench and the one-leg lift put at × 2.8 on every STS3215 and × 4 on the STS3250 hip rolls -- the model's kp 12 is soft for a real P 32; [servo-map.md](servo-map.md) §4), the only preset a deployable legs policy builds against; config.json records the resolved `{actuator: factor}` | `servo_kp_scale`, `--servo-kp-scale`; `servo_kp` / `servo_kd` also take one value per actuator |
| floor contacts | `all`: the plant as drawn. `feet` (training only): only the sole pads touch the floor; the servo boxes, the fall colliders and the arms stop colliding with it, and the explicit self-collision pairs stay. Those contacts only matter in a fall, which ends the episode, but MJX reserves and solves their slots every step: on the robot plant `feet` cuts 182 contact slots to 56 and the physics step about 3.5× (laptop CPU). The referee always grades with `all` | `floor_contacts`, `--floor-contacts feet` |
| bus latency | the first sub-steps of each tick still serve the previous target; 0–8 ms per episode, ±1 ms jitter per tick | `latency_ms`, `latency_ms_max`, `latency_jitter_ms` |
| gear backlash | a deadzone of ±lash/2 on the PD error; 0.5–1.0° per episode | `--backlash-deg lo,hi` |
| roll play | an EXTRA deadzone on the four roll joints (hip roll, ankle roll) only, on top of the backlash: the play downstream of the encoders. Calibrated on the legs-only body so a 30 mm one-foot lift tilts the torso ~5° with the stance encoders moving ~1° (`sim/roll_compliance.py`, [roll-compliance-2026-10-24](design-v6/roll-compliance-2026-10-24.md)); total roll deadzone ≈ 2.6°. 1.0–2.6° extra per episode in the legs-only runs, MJX and CPU | `--roll-lash-deg lo,hi` (`roll_lash_deg`, `roll_lash_deg_max`) |
| roll series spring | a spring in series with the hip-roll and ankle-roll servos, beyond the encoder: the compliance the 2026-11-10 clamp test measured (30 mm one-foot lift, stance foot clamped: torso 9.4°, stance roll encoders 1.5° ankle / 0.4–0.7° hip; [the record](../experiments/policy/weight-transfer-gate/2026-11-10-robot/README.md), #277). Quasi-static, massless shaft: the link sees the roll PD in series with k (kp, kd × 1/(1 + kp/k)), and every encoder reading (the q and q̇ observations, the `--dq-obs-window` history, the `--fw-goal-speed` feedback) is the servo shaft, link + τ/k. k drawn per episode per role, uniform in k, L and R sharing it; one value = fixed; a role left out is rigid. The weight-transfer gate's Round 3 clamp fit (hip 11.0, ankle 17.75 N m/rad, no roll play) gives 9.2–9.3° on the training plant's clamp test (`tests/test_roll_series.py`); the rigid plant gives 2.8° (no play) to 4.2° (1.25°). The same springs over-unload the stance foot on the both-feet-down foot-load ladder (Round 3), so the recipes draw `hip=10:25,ankle=16:40` (the fit at the soft end, about 2.3× stiffer at the other): with `--roll-lash-deg 0.5,2.0` the clamp tilt spans 6.2–12.7° (L lifted), 7.5–9.2° at the range's centre. Off by default. MJX and CPU; config.json records it, so the referee and `grade.py` grade a run on its own spring | `--roll-series-k hip=LO:HI,ankle=LO:HI` (`roll_series_k`, N m/rad) |
| roll play beyond the encoder | free play inside the hip-roll and ankle-roll servos, between the position sensor and the output horn: the 2026-11-11 clamp session (`hw_sessions/2026-11-11/m277/NOTES.md`; the fit and its numbers are in [the weight-transfer gate](../experiments/policy/weight-transfer-gate/README.md), #277). The servo shaft is position-controlled and is what every encoder reading is (the q and q̇ observations, the `--dq-obs-window` history, the `--fw-goal-speed` feedback); the link floats inside ±play/2 of the shaft with no torque between them, and the servo's PD reaches it only at a band edge, through a contact that pushes only. A free shaft runs to the set point at up to the no-load speed. Drawn per episode per joint, uniform; `LO:HI` sets all four rolls, `hip=LO:HI,ankle=LO:HI` each role, and `L_hip=`, `R_hip=`, `L_ankle=`, `R_ankle=` one side's joint (overriding its role there); one value = fixed. The clamp fit (`fit_rollplay.py`, the robot's own poses, R foot welded, grade plant with the measured mass, no roll lash) is hip 4°, ankle 5° peak to peak: 30 mm lift 7.9° (robot 8.45–8.7°) with the stance encoders 1.9° / 0.8° (robot 1.1–1.5° / 0.35–0.45°), a hand-pushed swing of 9.4° with 5.7° of it beyond the encoders (robot ~6.1°), and 2.5° after an 81 mm shift (robot 2.7°); the rigid plant tilts 2.7°. With the roll lash on as well the encoders move about twice the robot's and the fit is worse, so a recipe with this flag drops `--roll-lash-deg`; the recipes draw `hip=3:5,ankle=4:6`. The play breaks the both-feet-down foot-load ladder at its far rungs (the plant falls off the 80.6 and 86.8 mm shifts toward L at the fit; the robot stands on them), which the model does not yet explain. Exclusive with `--roll-series-k`. Off by default. MJX and CPU; config.json records it, so the referee, `grade.py` and the distil rebuild a run on its own play | `--roll-play-deg LO:HI` or `hip=LO:HI,ankle=LO:HI`, per side `L_hip=LO:HI,R_hip=...,L_ankle=...,R_ankle=...` (`roll_play_deg`, degrees peak to peak) |
| actuation lag | three cascaded first-order stages on the commanded target (the firmware command shaper's structure), pole drawn per episode | `--act-lag lo,hi` (Hz) |
| dead time | the servo serves the target from 0–N control ticks ago | `--act-delay-max N` |
| quantization | observed q and q̇, and the commanded targets, on the 4096-tick grid and integer steps/s | `--quantize` |
| joint-velocity observation | the robot's q̇ is the STS3215 speed register 58: the mean joint velocity over the last ~40 ms (~20 ms late) in 50 steps/s (4.4°/s) steps (the 2026-11-05 kick analysis, `hw_sessions/2026-11-05/kick_analysis.md` §3, not in git). The observed q̇ becomes (q_t − q_t−N) / (N · 20 ms), N fixed or drawn per episode, then rounded to Q steps/s (replacing `--quantize`'s 1-step q̇ rounding). The robot's model is N 2, Q 50. Off by default (the instantaneous q̇). MJX and CPU; `grade.py` probes `… dqlag` and `DQ_OBS_WINDOW` apply it to a trained policy | `--dq-obs-window N` or `lo,hi`, `--dq-obs-quantum Q` |
| free play | the roll play beyond the encoders is `--roll-play-deg` (the encoders reading the servo shaft, the link free inside the band), fitted per role to the 2026-11-11 clamp session; `--roll-lash-deg` (a deadzone on the servo error, the encoders reading the link) and `--roll-series-k` (a spring, the encoders reading the shaft) are the other two forms. The referee inherits each from `config.json`. Still open: the both-feet-down ladder, where the fitted play falls off the far rungs the robot stands on; the double-stance hysteresis the robot shows (the same shifted pose 4.2° apart by direction; the sim 0.7°); and the stance encoders moving more under load than the robot's (1.9° / 0.8° against 1.1–1.5° / 0.35–0.45°) |
| rough floor | `sim/terrain_mosaic.npz` (from `sim/mjx/gen_terrain_mosaic.py`): 12 × 6 m of 1 m tiles, 2–6 mm carpet-like texture, a random spawn point per episode | `--terrain` |

The bench numbers behind these knobs: the servo's dead time measures ≈ 85 ms
(4 ticks); a three-stage 2 Hz pole reproduces the joint motion measured in a
bench walk; the roll play is calibrated on the legs (above).

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
between sim and robot by hand. One frame for the 12 leg joints:

| block | width | contents |
|---|---|---|
| `q` | 12 | joint angles (minus the per-episode zero offset, §6) |
| `dq` | 12 | joint velocities |
| `up` | 3 | torso up-vector (IMU) |
| `linvel` | 3 | **zeroed** |
| `gyro` | 3 | body angular rate (IMU) |
| `prev_action` | 12 | last tick's action |
| `height` | 1 | **zeroed** |
| `phase` | 2 | gait-clock sin / cos |
| `cmd` | 7 | the command (§4) |

A frame is `3n + 19` wide for n policy joints — 55 here (§13) — and the
policy reads the newest 3 frames, so the network input is **165**. The history stands in for a state estimator. A
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
- **Gait clock**: a per-episode frequency U(1.25, 1.75) Hz, or U(lo, hi) with
  `--gait-freq lo,hi`, the feet half a cycle apart. config.json records the
  range as `gait_freq_range`; the referee draws from it, the grader and the SIL
  run the policy at its midpoint, and `tools/gen_obs_spec.py` emits the midpoint
  as `obs::kGaitFreqHz`. The link sources' cycle lengths (`link/sources.py`,
  `1.0 / 1.5`) are written for 1.5 Hz. `--speed-clock` scales it with the commanded planar speed (√
  law, ×1 at 0.35 m/s, clipped to [0.7, `--speed-clock-hi`]).
  `--clock-freeze-stand` holds the phase under a plain stand command, so the
  policy does not have to ignore an oscillating input to stand still.

**Action.** n values in [−1, 1] — brax's deterministic action, the `tanh` of
the Gaussian mean — mapped to position targets around the home pose, so a zero
action holds home. With `--action-map full` each side of zero spans to that
side's end of the joint's *policy range*: the actuator `ctrlrange` in the
plant, deliberately inside the mechanical stop. Widening a policy range
rescales the action map and costs a retrain.

On the robot the targets do not go straight to the bus. `obs::CommandShaper`
runs three cascaded 10 Hz first-order lags on them. `obs::MotionLimiter`, when
`limit` turns it on, then bounds every joint's velocity and acceleration with
a braking look-ahead and an optional jerk limit. Every SYNC WRITE carries a
per-servo goal speed ([firmware-design.md](firmware-design.md) §5.3, §5.3a).
Training puts the same chain in the loop, so that the deployed chain is the
trained one:

| flag | what the env runs | firmware |
|---|---|---|
| `--fw-shaper HZ[,HZ_MAX]` | the shaper on the policy's targets, seeded at the start pose; one pole per episode, U(HZ, HZ_MAX) or fixed | `obs::CommandShaper`, `shape` |
| `--fw-limit VMAX,AMAX[,JMAX]` | after the shaper: per joint \|v\| ≤ VMAX deg/s, \|a\| ≤ AMAX deg/s², the braking look-ahead (a target that holds still is reached at rest and never passed), and with JMAX deg/s³ the S-curve boxcar; seeded where the shaper is, at rest. The law is `sim/fw_limit.py`, which `tests/test_fw_limit.py` pins to the compiled firmware | `obs::MotionLimiter`, `limit VMAX AMAX [JMAX]` |
| `--fw-goal-speed` | within each tick the PD set point slews to the new goal at the firmware's goal speed: 1.25 × \|goal − present\| / 20 ms, or under `--fw-limit` 1.25 × the limited velocity; 50..3400 steps/s | `obs::goalSpeedSteps` / `obs::limitGoalSpeedSteps`, register 46 |

All three land in config.json. The CPU referee (`walker_env`) rebuilds the
shaper and the limiter from it, and grades at the deployed setting. Deploy a
run with `shape` and `limit` set to the trained values: the limiter is off
at boot, and a run trained without it is graded without it. Do not switch
the limiter on under a policy trained without it. In sim such a policy trips
the flail guard on its first stride: its own goals peak at 1200–1500 deg/s
and 40,000–67,000 deg/s², far beyond any useful limit
([experiments/policy/motion-limiter](../experiments/policy/motion-limiter/)).


---

## 4. Commands and skills

Seven channels (`ext_cmd`, set by `--precision`), redrawn every 2.5–4.5 s
(`--cmd-resample-s`):

| ch | meaning | training draw |
|---|---|---|
| c0 `vx` | forward speed, m/s | forward walks U(`--cmd-v-range`) (default 0.3–1.0; the `walk16_act` and `prec` runs 0.05–0.7 and 0.05–0.24); backward walks U(−0.4, −0.15) |
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
| knee-high march | c4 alternating on the gait clock (or at `--march-hz`), c6 raising the swing-sole target to the opposite knee's height, 18 cm on the legs plant (`--march-mix`); `--march-lift L` sets the peak target (m) instead, `0.08` = c6 up to 2 cm. The link clamps c6 to 5 cm, so the knee-high target itself cannot be sent. On the gait clock (`--march-hz` 0) the half period is the clock's, 0.67 s per lift at 0.75 Hz; the plain march above runs on a per-episode clock (1.5–3.5 s per cycle), not the gait clock |
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
silence the servos release. The robot also snaps commands inside the untrained
holes to a stand and clamps the rest to the trained bands. Those bands are
generated from the run at deploy time (`tools/gen_envelope.py`, run by `make
deploy`), from these settings:

| band | draw | read from |
|---|---|---|
| forward `vx` | `--cmd-v-range` | `config.json` `cmd_v_range` |
| backward `vx` | `cmd_back_range` (−0.4, −0.15), when `walk_submix[0] > 0` | env default: `train_mjx.py` has no flag and `config.json` does not record it, so the generator reads `BimoWalkerEnv`'s default |
| lateral `\|vy\|` | U(0.1, `cmd_vy_range` 0.25), when `walk_submix[1] > 0` | `cmd_vy_range` likewise an env default; the 0.1 floor is a literal in both envs |
| turn `\|wz\|` | `cmd_w_range` | `config.json` |
| crouch | `--cmd-crouch-range`, plus `crouch_range` (0.6, 0.9) when `ext_mix` has a crouch slice | `config.json`; `crouch_range` env default |

A band whose fraction of the mix is 0 (or a run without `ext_cmd`) is
generated absent, and a run with the `ext_mix` sway slice has no lateral hole.
Where a value is not in `config.json` the generated file says `default`.
See [control-channel.md](control-channel.md) for the clamp.

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
| per-step clearance (any command) | optional `w_low_swing` (`--w-low-swing`): on the touchdown of a swing of ≥ 3 ticks, w × its sole peak's shortfall below `--low-swing-clear` (default 3 cm, the `walk_steps` rule; the referee always grades 3 cm), as a fraction of it; it also grades the start and stop steps the moving-gated gait shaping never sees |
| walking weight transfer (moving commands, MJX only) | optional `w_walk_shift` (`--w-walk-shift`): per leg, its `_mimic_ref` swing activity (`max(±sin φ, 0)`) × `exp(−(d/σ)²)` × `clip(clear/--low-swing-clear, 0, 1)`, d = horizontal distance from the CoM's ground projection to the other (stance) sole, σ = `--walk-shift-sigma` (1.5 cm), clear = the swinging sole's height. The clearance gate keeps a lean with both feet down from paying. With the gait clock on, `eval/episode_walk_swing_w`, `walk_swing_clear` and `walk_shift_err` report the swing activity and the swing-weighted clearance and CoM distance (m) of walking ticks, at any weight; a ratio to `walk_swing_w` is the swing-weighted mean |
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
swing; without it the ankle cancels the hip but not
the knee bend (the knee axis is −y), so the swing sole tilts toe-down by twice
the bend. Where the plant has an ankle roll, it cancels the hip roll. At zero
command the reference is the home pose.

On the robot plant the home pose is a straight leg, and a knee bend out of a
straight leg mostly swings the foot back: the 0.55 rad bend raises the sole
only 0–2.2 cm, and policies trained on it shuffle. `--mimic-lift H` (metres)
replaces the knee bend with an IK lift: the swing sole rises straight up by
`H·sw²` (sw = the swing half of the clock's sine), so it leaves and lands with
no vertical speed. `mimic_lift_coefs` (`sim/walker_env.py`) solves the hip-pitch
and knee offsets per leg on the plant's own kinematics (torso fixed, flexion
branch, the ankle keeping the sole level), and fits them as cubics in √h;
both environments call it on their own model. The stride's hip swing only adds
height at the ends of the swing (up to 1 cm at 0.35 m/s). The stance leg and
the standing reference are unchanged. `tests/test_mimic_lift.py` pins the
lift, the levelled sole, and the CPU/MJX agreement; parity block R4 runs the
full step with it. It pays
`exp(−Σ w_j (q − q_ref)² / 0.72)` over every policy joint, with the knee
weighted by `--mimic-knee-w`.
It supplies backward and sideways walking, which shaping alone did not
produce, and it carries the swing timing.

`--mimic-sway M` (metres) puts the weight transfer into the reference. While a
moving command is active the torso moves toward the stance foot by M, with the
hip rolls; the ankle rolls cancel them so the soles stay level, and the sway
leads the swing clock by `--mimic-sway-lead` (0.785 rad). The rad/m is solved
on the plant's kinematics (`mimic_sway_coefs`, −3.05 rad/m on the measured-sole
plant). Without it the reference holds the rolls at the standing pose and
`w_mimic` pays against any lateral shift.

A march is a lifted command, and a lifted command gates `w_mimic` off (and
`w_pose`, `w_feet_phase`, `w_walk_shift`, which are gated on a moving, unlifted
command). A march therefore has no reference unless `--mimic-march M` turns it
on: during a gait-clock march (`--march-mix`, traj mode 4, c4 alternating) the
reference lifts the swing sole M metres on the clock (the `--mimic-lift` IK
table, M ≤ 0.08) and applies `--mimic-sway`, with no stride; a one-leg balance
or an air circle stays off. It needs `--mimic-lift`, the gait clock and no
`--march-hz`. What pays for a march otherwise: `w_lift` (the swing foot ≥
`lift_clear` 3 cm with the stance foot down), `w_track_foot` (the swing-sole
target kernel, σ 6 cm), `w_single_support`, `w_knee_high` (the clearance as a
fraction of the commanded swing height) and `w_com_stance` (the CoM over the
stance sole, σ 4 cm, gated by the swing clearance): the only one of them that
pays for lateral weight transfer, and zero in the `loco` family unless set.
`tests/test_mimic_march.py` pins the reference, the gate, the defaults and the
CPU/MJX agreement; parity block R13b runs the step.
A warm start from a run that drew no march needs `--init-norm-march`: that run's
observation normaliser holds std 1e-6 on the lift and foot-dz slots (the channels
never moved), so a march reads as ~1e6 σ. The flag re-seats those slots to the
march draw (`reseat_march_norm`; c6 stays frozen when the march asks no c6,
`--march-lift 0.06`). On the robot the same frozen slots make `policy::pinFrozen`
ignore a march command, so a student that never saw one stands.

The robot's own march is `link/sources.py march5`: five full swings (after a
quarter-cycle first one) at the trained clock, then the stand. The clock holds
at the stand phase π/2 and runs from there the tick a lift is commanded, so the
script's time is the clock's phase. `grade.py`'s `march5` probe grades it:
per-foot peak lift on every step (the bar is 3 cm), the CoM's shift toward the
stance foot while each foot is up (the bar is 0.6), drift, yaw, falls and guard
trips, with the channels a run never saw move pinned to their trained value as
the firmware does.

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

- body mass and inertia × U(0.85, 1.15), per body; `--torso-mass-range r` draws the torso's scale from U(1−r, 1+r) instead (the legs-only runs use 0.4: the as-built torso is an unweighed estimate);
- `--torso-com-y c`: the torso's CoM shifted laterally by U(−c, +c) m (±15 mm in the legs-only runs: the same model weight shift balanced ~3° better on one stance foot than the other);
- floor friction × U(0.6, 1.4). MuJoCo takes the MAX of a contact's two geom
  frictions, so this alone never takes a foot contact below the sole pads' 1.0.
  `--contact-mu lo,hi` draws the EFFECTIVE sliding μ instead and sets it on the
  floor and every pad. `--contact-torsion lo,hi` does the same for the torsional
  coefficient (plant 0.02 m, a yaw brake no 3 mm pad has). Both are off by
  default ([friction-dr](../experiments/policy/friction-dr/2026-11-03-friction-dr.md)).
  The sole friction is **measured** (2026-11-04, one foot clamped, the other at
  half weight on the bench board): the foot slides at μ ≈ 0.45–0.55 and rotates
  under hip yaw with no measurable grip, ≤ 0.10–0.15 N·m at 7.5 N. The edge pads
  at μ 1 / torsion 0.02 grip 0.34 N·m in the same sim test (Coulomb bound
  0.65). `sim/bimo_biped_v6legs_meas_sole.xml` (`--xml`) is v6legs_meas with the
  measured sole: pads μ 0.5, torsion 0, geom `priority` 1 (the pad's friction is
  the foot contact's, so the floor-only draw above no longer reaches it and
  `--contact-mu` alone sets it), and 8 "flat" pads, 4 on the sole outline and 4
  inner, at a mean 42 mm from the sole centre: 0.15 N·m. Train on it with
  `--contact-mu 0.3,0.7` (0.11–0.19 N·m) and no `--contact-torsion`
  ([sole-friction](design-v6/2026-11-04-sole-friction.md), `sim/sole_friction.py`);
- the servo envelope, which is not DR but plant. `--servo-envelope role=STALL:W0` sets a
  measured stall / no-load ratio pair per role. The pair goes into `servo_types`, and the
  referee inherits it. `--supply-voltage` sets the bus the envelope scales with (default
  11.1 V; the robot reads 12.0 V). The hip pitch / knee refit is `2.0:1.325` / `2.0:1.281`
  at 12 V ([actuator-refit](../experiments/policy/actuator-refit/2026-11-03-actuator-refit.md)).
  `--w-qacc` and `--w-action-acc` are reward terms, not DR. They price the tick's joint
  acceleration and the action's second difference. `--w-gyro-hinge W --gyro-hinge H`
  (default H 1.2 rad/s, off by default) prices W·max(|ω| − H, 0)², where ω is the torso's
  angular velocity, the quantity the firmware flail guard trips on at EMA 1.5 rad/s. Yaw is
  included, which `--w-pitch-rate` leaves out (`tests/test_gyro_hinge.py`);
- gravity tilted by U(0, `--tilt-max`)° about a random azimuth — an un-level
  floor, as the IMU sees it (off by default);
- payload mass U(0, `payload_max`), when that is set.

**Per episode** — redrawn at every reset. `reseed()` keeps them alive across
auto-reset; brax's stock `AutoReset` restores the cached first state and would
freeze them.

- servo kp and kd × U(0.8, 1.2); stall torque and no-load speed × U(0.85, 1.15);
- latency 0–8 ms (±1 ms per tick) and backlash 0.5–1.0° (§2);
- IMU mounting misalignment (a rotation of N(0, `--imu-mount-deg`) about a random axis, 2° by default, 3° in the legs-only runs: the stored mount is from the old body), gyro
  bias U(±0.03 rad/s), and per-step noise (σ 0.01 on `up`, 0.03 rad/s on the
  gyro);
- velocity kicks U(0.1, 1.0) m/s at 1 % per tick (`--kick-range`,
  `--push-prob`); `--push-force N` trains with torso shoves of N newtons held
  for one control step instead of the kicks; the referee pins 5 N at 1 % whatever a
  run trained with, and grades harder shoves on their own card
  (`eval_precision.py --push-force 15 --push-prob 0.02` →
  `scorecard_push15x0.02`);
- start-pose jitter ±1.7° per joint (`--init-pose-deg`);
- the gait-clock frequency and the command draws (§3, §4).

**Optional per-episode terms**, each tied to a bench measurement:

| flag | draw | why |
|---|---|---|
| `--act-lag lo,hi` | actuation-lag pole, Hz | the servo's tracking lag; 2 Hz reproduces the measured bench walk |
| `--act-delay-max N` | servo dead time, 0…N ticks | ≈ 85 ms measured |
| `--zero-offset-deg z` | per-joint zero offset U(±z): the served target is target + offset, the observation subtracts it | hardware zeros are set by eye and move by degrees after a mechanical disturbance |
| `--gyro-gain-range lo,hi`, `--gyro-delay-max N` | gyro observation gain; delay ≤ 2 ticks | IMU scale and timing error |
| `--dq-obs-window lo,hi` | joint-velocity observation window, ticks (≤ 8) | reg 58 averages ~40 ms (§2) |
| `--yaw-kick-range lo,hi`, `--yaw-kick-prob p` | body yaw rate kicks, either way, while a move is commanded | a stand-in for the robot's step yaw, which the sim under-reads (`experiments/policy/yaw-gap/README.md`) |
| `--cmd-crouch-range lo,hi` | crouch-channel draw | the firmware battery guard lowers c3 on a sagging pack; a frozen channel also collapses its normalizer |
| `--roll-lash-deg lo,hi` | extra roll-joint deadzone, degrees | the play downstream of the roll encoders (§2) |
| `--roll-series-k hip=lo:hi,ankle=lo:hi` | spring in series with the roll servos beyond the encoder, N m/rad | the stance leg's 2026-11-10 clamp compliance (§2) |
| `--roll-play-deg lo:hi`, `hip=lo:hi,ankle=lo:hi` or per side `L_hip=lo:hi,...` | free play inside the roll servos beyond the encoder, degrees peak to peak | the stance leg's 2026-11-11 clamp play (§2) |
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
- **GPU handover.** The daytime GPU owner is the camera-watcher classifier
  (`llama3.2-vision` via ollama). The runners call `infra/night/vision_pause.sh
  pause` before training, which pauses the classifier container and unloads
  the model (the backlog waits in redis), and `resume` when the night ends or
  is stopped. While paused, a `CLASSIFIER_PAUSED` sentinel tells mira's camera
  watchdog that the lag is intentional.
- **Committed code only.** `~/code/robot-mjx` on mira is a git clone. Before
  every job the runner fetches and hard-resets it to `origin/main`, or to the
  branch a job names with `BRANCH=`. It retries a failed fetch, checks that HEAD
  is the fetched tip, and writes `SYNC=ok|STALE` into the job's state file.
  Anything not pushed does not train. The
  runner scripts sit in the clone's untracked `night/` directory, so a reset
  cannot rewrite them mid-run.
- **Arming.** From the laptop, `infra/night_arm.sh [--branch <name>]
  <train_mjx.py args>` checks that the branch is pushed, installs the runner and
  its cron on mira, and queues a job file `night/queue/<MMDDhhmmss>-<out>`.
- **Queue.** Jobs run in sorted order; the queue is rescanned after every job.
  A job file moves to `queue/done/` at launch, so an interrupted job does not
  re-run the next night. `CMD=` jobs run an arbitrary command (a distillation,
  `infra/night_distill.sh`, §9) through the same GPU gate. `--steps` is capped to what the remaining
  window allows (at 60 k steps/s for a MuJoCo Warp job, 17 k otherwise). Lines
  starting `#` are notes (parent, variable, hypothesis), never arguments.
- **Sharing the GPU.** A job starts only when the GPU is under 20 % busy with
  ≥ 11.5 GB free. Idle ollama models are unloaded (they reload on demand),
  active inference is never touched, and after each job any model that fell
  back to CPU is moved back onto the GPU.
- **Collection.** `infra/night_collect_local.sh` copies each finished run into
  the working checkout and referees it: the python column with `--render`, and
  the SIL column. The runner starts it, detached, when its queue loop ends and a
  run is waiting (never while a job is still training); cron 07:15 on mira is the
  safety net. `COLLECT_JOBS` (default 6) runs are collected at once, each into
  its own log that is appended to `night_collect.log` in a stable order; a
  second collector exits at once on `night/.collect.lock`. The lag column is run by hand (§8). It
  fast-forwards the checkout to `origin/main` first, so the references and
  plants a run trained against are there to referee it. If uncommitted files
  block the fast-forward, it stashes only the paths upstream also changed, under
  a tag. It then verifies HEAD; if that fails, the log says `STALE REFEREE`.
- **Re-planning.** `infra/night/plan_morning.sh` (cron 05:30 and 07:30; also
  started by a collector that finishes between 00:00 and 07:30) waits until the
  runner and collector locks are free and no `state.done.*` is left (3 h cap),
  skips itself if a plan already completed today, then runs a headless Claude Code session on
  `infra/night/planner_prompt.md`: it evaluates the night against the campaign
  record (today the get-up, [getup-campaign-2026-10.md](design-v6/getup-campaign-2026-10.md)),
  improves the references on the CPU if a phase is failing, queues the next
  night, refreshes the results page and reports on the campaign's issue.
  `infra/night/ensure_queue.sh` (cron 21:30) tops the queue up to five jobs
  from `night/fallback.job` with fresh seeds if the planner left it short.
  Installed copies of both live beside the runner in `night/`.
- **Fix and continue.** When a harness step fails, `infra/night/claude_fix.sh`
  starts a headless Claude Code session (Opus) with the failure and the log
  tail, and waits for it. The step is retried once if the session reports
  `FIX_STATUS: fixed`. The steps are: a failed sync, a failed pre-flight, a
  training or `CMD=` job that exits non-zero (the fixer may re-queue it), a
  failed collect sync or referee, a missing `fallback.job`, and a failed
  planner. Limits: one fixer at a time, six a day, and `CLAUDE_FIX=0` turns it
  off. Log in `night/fixer.log`, transcripts in `night/fixer/`.

### Training on the Mac (CPU physics + MPS)

`sim/mac_train.py` trains on the laptop (Apple M5 Max: 18 CPU cores, 40-core
GPU). It splits the work:

- **Physics:** `walker_env.BimoWalkerEnv`, the referee's own env, runs in N
  worker processes (default: cores − 2). Each worker steps its slice of the
  envs and runs the policy on its own CPU.
- **Training:** the policy and value networks train on the GPU through
  PyTorch's Metal backend (`--device mps`, falling back to CPU).

MJX itself has no supported path onto the Mac GPU: JAX here is CPU-only and
MuJoCo Warp needs CUDA.

**Same command line and run layout as `train_mjx.py`.** It uses train_mjx's
own `make_parser()` / `build_env_kw()`, so one set of flags means one env
config. Its additions are `--workers`, `--device`, `--hours` and `--resume`.

- `config.json` gains `"trainer": "mac_ppo"`.
- `params.pkl` is brax's `(normalizer, policy, value)` tuple, so the referee,
  the distiller and `tools/export_policy_weights.py` read a Mac run
  unchanged. A saved checkpoint acts identically in the referee's brax loader
  and in PyTorch (max |Δaction| 2e-7).
- `--init-from` reads a brax run, so a run can move from MJX to the Mac and
  back.

**The PPO is brax 0.14's, term for term:**

- a tanh-normal policy;
- (512, 256, 128) swish MLPs with lecun-uniform kernels;
- brax's own `running_statistics` normalizer, updated on each batch before the
  SGD;
- GAE (λ 0.95) recomputed per minibatch;
- clip 0.3, normalized advantages, value coefficient 0.5;
- entropy with the tanh log-det term;
- Adam with no gradient clipping;
- one set of parameters per iteration, then trajectory-shuffled minibatches.

With 16 workers the batch layout is brax's exactly: 2048 envs, 32,768
trajectories of 20 steps. Other worker counts round the trajectory count up
to whole rounds.

**Rate on the robot's plant (2026-10-07):** 16,200–17,100 env steps/s with 16
workers, measured while another CPU sweep shared the machine. That is about
12× mira's MJX rate on the same plant (1,310 steps/s). Physics is ~95 % of an
iteration (≈ 38 s of rollout against ≈ 2 s of update per 655,360 steps).

**What differs from an MJX run:**

| item | Mac trainer | consequence |
|---|---|---|
| contact physics | CPU MuJoCo, the referee's engine | none against the referee; the MJX-vs-CPU contact-manifold difference (§8) is gone, not modelled |
| mass / friction DR | drawn per episode | MJX draws per env batch; the per-episode distribution is the same |
| action-chain lag DR | per episode, uniform(`act_lag_hz`, `act_lag_hz_max`), via the CPU env's `act_lag_dr_max` | named apart from the config key so a referee rebuilding from `config.json` keeps its one pinned pole |
| feet-crossing reward (`w_foot_cross`) | applied (`foot_cross_term=True`) | the referee's env leaves it off; its scores do not use reward |
| eval curve | `eval/*` in `progress.jsonl` are the training episodes that ended in that iteration (stochastic policy) | MJX runs a separate eval env; the two curves are close but not identical |
| non-`--precision` value net | (128, 128), the policy's size | MJX uses (256, 256) without `--precision` |

With the per-episode draws pinned, the robot preset's training config
(k1's flags) matches MJX step for step on synced airborne states while the
legs are splayed apart (200 steps: |Δqpos| 6e-9, |Δreward| 3e-7). The match
ends when the legs touch each other. With the hip rolls driven inward, the
legs touch on 194 of 200 steps, and qpos differs by up to 1e-3 and reward by
up to 0.06 from step 6. That is the same engine-level contact difference as
on the floor. The Mac trainer has the referee's version of it.

```bash
.venv/bin/python sim/mac_train.py --out <run> --robot --precision --family loco \
    --walk-submix 0,0 --cmd-v-range 0.05,0.35 --w-mimic 1.5 --servo-kp-scale robot \
    --act-lag 2,12 --steps 400000000 --hours 6        # --workers 16 by default here
```

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
no payload (the plant models the head camera), 4 ms latency, 0.7° backlash, IMU observations, 5 N shoves at
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
| `walk_steps_L/R` | on request (`--scenarios`, or `--eval-scenarios` at train time) | stand 1 s, vx 0.30 for 1.8 s starting with the L / R foot (the gait clock is set so that foot swings first), then stand | no fall; ≥ 4 steps and ≥ 2 per foot, a step being a swing of ≥ 3 ticks that begins between the command and 0.4 s after the stop; **every step lifts its own foot's sole ≥ 3 cm** above the standing sole height (the per-step minimum, not a fraction); ≥ 0.25 m forward; upright at the end; lateral drift < 0.25 m. Reports `lead_ok` (the requested foot went first), informational |
| `circle_air_L/R` | skills | lift, then 2 circles of 4 cm with the foot | touchdown < 10 %; clearance ≥ 3 cm for ≥ 80 %; traced radius ≥ 3 cm and ≥ one full sweep |
| `crouch_hold` | skills | crouch 0.7 for 3 s, then stand | height within 3.5 cm of target; recovers to ≥ 90 % |
| `march_in_place` | skills | alternating lifts every 0.7 s | ≥ 3 clean lifts per leg (≥ 0.3 s at ≥ 3 cm); drift < 0.15 m |
| `march_10s` | skills | knee-high march for 10 s | ≥ 6 alternating lifts peaking above 60 % of knee height; drift < 0.15 m |
| `hip_sway` | skills | vy = 0.12 sin(2πt/1.6) for 6 s | lateral amplitude ≥ 2 cm; net drift < 0.15 m |
| `recover_sit` | getup | start seated, command stand | held stand within 5 s; ≥ 85 % height at the end |
| `recover_fallen` | getup | start from a settled ragdoll | held stand within 6 s; ≥ 85 % height at the end |

(Heights are fractions of the plant's nominal torso height.) `handover_stand`
and `walk_steps_L/R` exist behind `--scenarios` only, so they never move the comparable totals.
A run can ask for extra scenarios on its own default card with `train_mjx.py
--eval-scenarios walk_steps_L,walk_steps_R` (recorded in `config.json`); the card
then has more rows than its family's, and its total is not comparable with the
family's. `balance_L/R` also report `one_foot_s` and `one_foot_run_s` (time on one
foot with the lifted sole ≥ 3 cm, total and longest unbroken); the pass rule is unchanged.

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
  all three referee columns score it with no shim. It is written after **every
  round** (atomically), so a kill leaves the last finished round's student;
  `config.json["distill"]` records `rounds_done` and `complete`.
- **On the robot's plant** (`--robot --xml bimo_biped_v6legs.xml`) the same
  script distills a legs teacher: 12 joints with the ankle rolls, frame 55,
  net 165 → 128 → 128 → 24, ≈ 41k parameters (≈ 160 KB fp32). The teacher's
  physics is reused (`mjx_impl: warp`, contact pool re-sized to the distill
  batch); `--mjx-impl jax` distills a Warp teacher on the CPU. The student
  inherits the teacher's action map, and the exporter takes `full` only: a
  deployable student needs a teacher trained with `--action-map full`
  (`--require-full-map` refuses any other).
- **As a night job**, right after its teacher (§7): `infra/night_distill.sh
  <teacher> [<student>]` distills 24 rounds inside a wall-clock budget that ends
  at 06:45 less a referee reserve (`--time-budget-s`: no round starts that
  would not finish), referees the student on the CPU on `walk_steps_L/R`
  (the clearance gate) and `stand_10s` into `scorecard_partial.md`, and hands
  it to the morning collector (`night/state.done.<student>`), which grades it
  with the full card and the reel like any run. The queue file sorts after the
  teacher's:

  ```
  BRANCH=main
  CMD=infra/night_distill.sh robot_legs_walk11_full robot_legs_walk11_full_s128r24
  OUT=robot_legs_walk11_full_s128r24
  ```

  `infra/night/preflight.sh.py` checks a distill job at 22:00: the student name
  is new and equals `OUT=`, the teacher exists in the night clone or is the
  `--out` of an earlier queued job, its action map is `full`
  (`DISTILL_ALLOW_LEGACY=1` in the `CMD` for a referee-only student), and the
  wrapper is on the job's branch.

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
     permutation), plus `obs_vectors.h` from `walker_env._obs`.
   - `tools/gen_envelope.py --run` → `linkproto/cmd_envelope.h`, its Python
     twin `link/cmd_envelope.py` and `cmd_envelope_vectors.h`: the command
     envelope the robot enforces (§4).
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

**`robot_legs_walk16_act_s128r24`** (`obs::kRunName` and `policy::kWeightsRun`
agree): the (128, 128) student of the legs walk `robot_legs_walk16_act`, the
same run the `v6legs` profile carries (below). Plant
`bimo_biped_v6legs.xml`, the 12 leg joints; net 165 → 128 → 128 → 24; action
map `full`; IMU observations; trained at the as-built servo stiffness. Its
scorecards and reels are on the results page.

### Policy profiles: a second plant's policy

The firmware compiles in exactly one policy, chosen at build time. The
deployed one's generated files are the ones above; a **policy profile** is a
complete second set for another plant (`tools/policy_profile.py`):
`firmware/policies/<name>/{obs/obs_spec.h, policy/weights.h, linkproto/cmd_envelope.{h,py}, vectors/*.h}`
and `sim/sil/golden/<name>/`. A build selects it with `BIMO_POLICY=<name>`:

| | deployed | profile `v6legs` |
|---|---|---|
| headers | `make deploy RUN=<run>` | `make deploy RUN=<run> POLICY=v6legs` |
| host tests | `make -C firmware/host test` runs the deployed build, then every profile's | `make -C firmware/host test POLICY=v6legs` (into `build/policy-v6legs/`) |
| ESP32 image | `make firmware` → `build/firmware` | `make firmware POLICY=v6legs` → `build/firmware-v6legs` (`idf.py -DBIMO_POLICY=v6legs`) |
| SIL | `eval_precision.py --sil` and `sim/sil_twin.py` pick the profile whose `kPlantXml` is the run's plant (`$BIMO_POLICY` for `sim/sil/harness.py`), build its `libctrl_sil`, and refuse a run whose joints or obs layout differ from the compiled spec | |

The profile's directory goes first on the include path, and every generated
header carries an `#error` guard against the other kind of build, so an
include-order mistake cannot compile the wrong policy. `link/protocol.py`
reads the profile's `cmd_envelope.py` when `$BIMO_POLICY` names it. Nothing outside the
generated headers names a joint count: the assembler, the MLP, the actuation
map, the control loop and the SIL ABI are sized by `obs_spec.h`.

**Int8 profiles.** `make deploy RUN=<run> POLICY=<name> QUANT=int8`
(`gen_policy_weights.py --quant int8`) writes the profile's net as int8
(`policy/q8.h`): GPTQ-rounded on CPU-plant rollouts of the float net, which
the generator runs itself (~10 s), and the head cut to its mean logits. A
regeneration keeps an int8 profile int8. This is how a net wider than
(128, 128) fits the 20 ms tick ([firmware-design.md §6.1](firmware-design.md)).
The SIL library of an int8 build runs the compiled-in int8 net when given
that run's `.silw` (any other run runs float, and the spec's `policy.quant`
says which). Several profiles can share a plant; `eval_precision.py --sil`
takes the first, and `SIL_PROFILE=<name>` picks another.

**`v6legs256`**: the `v6legs` plant and obs layout with the walk16_act (256, 256)
student `robot_legs_walk16_act_s256r24` as an int8 net (165 → 256 → 256 → 24,
113 KB), a deploy candidate. It was distilled on the fast-servo dead time
(`distill_student.py --hidden 256,256 --env-set
'act_delay_types={"sts3215":[0,2],"sts3250":[0,1]}'`). On the fast measured actuator,
run the way the robot runs it (coast to π/2, 12 Hz shaper), it holds x1.5 loop gain
0/16, where `walk16_act_s128r24` falls 11/16
([experiments/policy/int8-256](../experiments/policy/int8-256/2026-11-02-int8-256.md)).
Until 2026-11-02 the profile held the walk14 (256, 256) student as a timing and parity
test net (§6.1 of the firmware design); the timing does not depend on the weights.

**`v6legs`**: the robot's legs-only body, `sim/bimo_biped_v6legs.xml`,
`--robot`, action map `full`. 12 policy joints in the sim's actuator order —
`L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle, L_ankle_roll`, then the
same for `R_` — on servos 9, 51, 2, 3, 4, 12 / 10, 52, 6, 7, 8, 11. Frame 55
(`q 0, dq 12, up 24, linvel 27, gyro 30, prev_action 33, height 45, phase 46,
cmd 48`, 7 command channels), 3 frames, network 165 → 128 → 128 → 24. The
profile carries **`robot_legs_walk16_act_s128r24`** (from 2026-11-02), the
(128, 128) student of `robot_legs_walk16_act`, trained at the as-built factory
gains (servo_kp_scale 2.8 / 4.0 = `servo_gains.h`'s `kSimKpScale` rows, so its
static check passes) plus actuator DR around them (per-servo dead time, kp/kd
ranges; branch `policy-stand-rca`), with the ankle pitch inside its stops
(policy range ±40 deg). It replaced `robot_legs_walk14_s1_s128r24`, the first
armed test's policy, after the stand RCA (PR #189): 0/8 falls at ×1.5 loop
gain on the measured actuator, where walk14_s1 fell from ×1.25. The comparison, the closed-loop SIL parity and the hardware procedure
are in [first-policy-stand.md](first-policy-stand.md). Its frozen command
channels (crouch 1, lift and foot 0) are pinned by the firmware
(sil-harness.md finding 2). `tools/make_test_policy_run.py` still makes the
untrained test net for exercising the path on a new plant. A placeholder
compiles as `kWeightsArePlaceholder` and the firmware refuses to arm on it
(`REFUSED_POLICY`); an exported net arms.

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

- **Get-up is trained by tracking a reference, never by shaping.** Across 12
  rounds and about 1.3 B steps, PPO reward shaping reached the kneel but could
  not bridge kneel → balance-catch → held stand — including a clean-physics run
  with no DR at all. The robot's get-up is the **scripted, arm-assisted
  sequence** ([DESIGN.md](../DESIGN.md), "Get-up"); RL makes it robust by
  **phase-indexed tracking** of that sequence's recorded open-loop rollout
  (`--rise-ref`): a DeepMimic-style tracking term at dominant weight,
  reference-state initialisation along it, residual actions around it and
  early termination off it. A reference is only as good as the open-loop
  sequence behind it — tracking one that cannot stand on the training plant
  cannot work — so references are built and checked over a robustness grid by
  `sim/getup_ref_analytic.py` (an analytic CoM-balanced rise, a searched prone
  roll). The campaign and its nightly loop:
  [getup-campaign-2026-10.md](design-v6/getup-campaign-2026-10.md), issue #137.
  A reference is indexed by time from the episode start. The prone
  reference's roll ends on the back and the supine seat push follows, so a
  policy trained on it also gets up from supine when it starts at that phase
  (`ref_back_phase`, ~11.3 s): the robot knows which way up it lies from the
  IMU, so the start phase is its choice. `recover_supine` (phase 0),
  `recover_supine_ph` (supine at the policy's on-the-back phase),
  `recover_prone` and `recover_prone_fall` grade it; `recover_sit` and
  `recover_fallen` do not apply to the robot. The joint friction is not
  measured (0.05 BAM … 0.35 assumed), so `--joint-frictionloss-range lo,hi`
  draws it per env in training, and `eval_precision.py --joint-frictionloss`
  grades a run at another value (`scorecard_fl<value>.md`).
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
#     the robot preset (legs policy, held neck and arms): add --robot (§13)

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

Walking and get-ups train on the robot's plants (`robot_legs_walk*` on the
legs-only body, `robot_getup_*` on the ar plant), and distilled legs students
run on the ESP32 (§9); no policy yet covers all 17 joints. `train_mjx.py --robot` applies the robot
preset (`sim/mjx/robot_plant.py`), resolved against the plant's own actuators
and written into `config.json` explicitly, so the referee, the distiller and
the obs-spec generator rebuild the same robot:

| piece | on the robot's plant |
|---|---|
| plant | `sim/bimo_biped_v6ar.xml` (the body as drawn: legs, neck, arms) and `sim/bimo_biped_v6legs.xml` (the body as built on the bench: the 12 leg servos and the control board, no pack, Pi, head, neck or arms; `--xml bimo_biped_v6legs.xml`, `build_v6_inertia.asbuilt_legs`). The ar plant is generated by `sim/gen_plant_v6.py` with CAD inertials from `sim/build_v6_inertia.py`. The committed file carries all 17 servos: the 12 leg joints, the neck yaw and the arms (shoulder pitch + elbow per side); every layout is read from the model. No payload: the camera is in the head, which the plant models |
| policy joints | the 12 leg joints, 6 per leg with the ankle roll: frame 55, network input 165, action 12 |
| neck and arms | **held, not driven.** `held_joints` gives each a fixed servo target: neck 0°, the arms at their folded rest pose (`dimensions_v6.ARM_REST`: shoulders 15° forward, elbows folded 95°; DESIGN.md §5.4), the pose that clears every leg joint's full range. A held servo runs the same actuator model as the legs (PD, envelope, lag, dead time, backlash, ticks, zero offset), starts the episode at its target (it is the plant's `qpos0`), and is in neither the action nor the observation; torque and power metrics include it. Why not actions: the walking hold is a fixed pose by design, four arm actions would only have to learn to stay still, and the neck belongs to the Pi's camera later |
| servo stiffness | per servo (`servo_kp_scale`, §2): `--robot` defaults to `planb` (× 4 on the six roll and knee servos); a deployable legs policy trains at `asbuilt`, the factory gains the robot runs (× 4 on the STS3250 hip rolls, × 2.8 on every STS3215; [servo-map.md](servo-map.md) §4), which `servo_gains.h`'s compile check requires. The hip rolls run the STS3250's torque–speed envelope (`servo_types`), every other servo the STS3215's |
| hip pitch | policy range to −120° flexion (`hip_flex_deg`), the leg link's relief; the plant's stop stays at −125° |
| reward roles | every policy joint must resolve to a role (`joint_role` in both envs: the six per-leg roles, neck, shoulder, elbow) or the env refuses to build — a joint a plant adds is held or named, never trained unregularized by accident. The posture, dof-limit and imitation terms sum over every policy joint, so the ankle roll is in them; the imitation reference keeps the sole level in pitch and roll (`mimic_sole_level`, §5) |
| mirror | `mirror.py` maps over the policy joints (`joints=env._act_names`, as `--w-mirror-loss` passes them) and maps an unprefixed joint onto itself |
| referee | `eval_precision.py` recognises the robot's plant by its ankle rolls and IMU (and the head camera where the plant has a head): no pinned payload; `recover_sit` and `recover_fallen` are n/a (the robot gets up with the scripted arm sequence; legs alone cannot get this body up), `handover_stand` is n/a; every other scenario runs. n/a scenarios are listed in the scorecard and left out of the totals. `eval_ref.py` rebuilds the same world from `config.json` |
| parity | `sim/mjx/parity_test.py` blocks R1–R3 gate the CPU/MJX arithmetic on the robot's plant — `planb` stiffness, the held neck, a 17-servo plant with the arms held, the sole-level reference, quantization, airborne and in stance; `--robot-only` runs just those. `tests/test_robot_plant.py` is the smoke test |

**Open before Gate E:**

| piece | what remains |
|---|---|
| measured plant | the masses weighed (#82) replacing the CAD inertials |
| free play | the roll play is in the legs-only runs as an extra roll-joint deadzone (`--roll-lash-deg`, calibrated on the 2026-10-24 bench lift); the referee inherits the band from `config.json`. The compliance beyond the encoders is `--roll-series-k` (a spring, the encoders reading the shaft). Still open: the play per joint (one lumped value for the four roll joints), and the play's own shaft/link split in the observation (with the roll deadzone the sim's encoders read the link) |
| referee thresholds | the absolute thresholds (distances, drifts, the torque-off stand's unmeasured passive friction) are the prototype's; re-derive them for the taller, heavier robot. The plant-measured targets (nominal height, knee height, crouch depth) already follow the plant; the crouch-depth limit (`crouch_theta_max_deg` 39°) and the rise and fallen-start poses are the prototype's numbers |
| the robot's bench envelope | `main/mech_envelope.h` `kMech*V6Legs`: the v6 legs plant's policy range, not the stops measured on the robot ([servo-map.md](servo-map.md) §3.3). Its ankle-pitch ctrlrange (±45°) is wider than the plant's own joint stops (±40°) |

**When.** The design is validated in stages (DESIGN.md, "Validation method"):
kinematics, actuator margins and contact realism first, then the scripted
open-loop walk in the plant, with system-identification probes on the robot
feeding the plant fit (#82, open). Policies (Gate E)
train on the plants with the measured stiffness, play and joint stops in them;
a legs policy holds the built legs standing, and walking on the floor, beginning
with marching in place (#261), is the next hardware gate.
