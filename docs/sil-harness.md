# Software-in-the-loop (SIL) harness — design

*The contract between the simulated plant and the firmware's own control
code. How to build, run and read it: [sim/sil/README.md](../sim/sil/README.md).
Where it sits in the pipeline: [training.md](training.md) §9.*

## Why

The Python stack cannot see a whole class of deployment bugs: byte order,
the action-index → bus-ID permutation, calibration zeros and signs, the
network's activation function, the observation width. SIL runs the
firmware's real control code against the CPU referee plant, so each of
those is a red test on a laptop instead of a robot walking into a wall, and
the same referee that grades policies grades the firmware stack.

## Loop topology (one 20 ms tick)

```
walker_env (CPU referee plant)
  |  SERVO-SIDE VIEW: joint angle -> calibrated ticks (uint16, per-joint
  |    zero_steps/dir, 4096/rev); velocity -> register-58 sign-magnitude
  |    steps/s. IMU: up-vector + gyro taken from the env's own obs frame
  |    (its mounting error and noise included). Exactly what SYNC READ and
  |    the IMU driver deliver.
  v
struct SilSensors                                   [C boundary]
  v
libctrl_sil -- the firmware's code, no sim knowledge:
  stepsToAngle -> gait clock -> obs::assembleFrame -> obs::History
  -> policy::forwardNet (exported weights + normaliser)
  -> actionToAngles -> CommandShaper -> angleToSteps + goal speed
  -> per-bus-ID targets
  v
struct SilTargets                                   [C boundary]
  v
harness: bus order back to joint order -> ticks to angles -> invert the
  env's action map -> env.step(action). The eval_precision scenarios run
  unchanged, so referee results apply to the firmware stack directly.
```

## C ABI (`firmware/host/sil.h` → `libctrl_sil.{dylib,so}`)

```c
typedef struct {            // what the bus + IMU deliver
    uint16_t pos_ticks[10]; // present position, register 56 semantics, bus-ID order
    int16_t  vel_ticks[10]; // present speed: the register-58 sign-magnitude
                            //   word (sign in bit 15), carried in an int16
    float    up[3];         // IMU up-vector
    float    gyro[3];       // rad/s
    float    cmd[7];        // command channels (harness-supplied)
} SilSensors;

typedef struct {
    uint16_t goal_ticks[10];  // SYNC WRITE view, bus-ID order; shaped when the
                              //   shaper is on (the default, as deployed)
    float    action[10];      // the raw [-1, 1] policy action (diagnostics)
    float    obs[147];        // the assembled observation (diagnostics, goldens)
    uint16_t goal_speed[10];  // per-servo register-46 goal speed, bus-ID order;
                              //   0 when shaping is off
} SilTargets;

int  sil_init(const char* weights_path,   // .silw; NULL/"" = the compiled-in weights
              const char* cal_path,       // JSON; NULL/"" = nominal (2048, +1)
              float gait_freq_hz);        // <= 0 = the firmware's 1.5 Hz; 0 on success
void sil_reset(void);                     // history refill + clock zero, like an arm
int  sil_tick(const SilSensors* in, SilTargets* out);
const char* sil_spec(void);               // JSON: dims, offsets, kServoId, layout hash,
                                          //   loaded weights, "sil_abi": 2
const char* sil_last_error(void);         // text of the last failure, never NULL
void sil_set_shaper(float pole_hz);       // mirror of the CLI `shape`; 0 = raw targets;
                                          //   sil_init resets it to the firmware default
```

- **Array order is bus ID**: `slot = bus_id − 1`, and joint *j* sits in slot
  `kServoId[j] − 1`. With the generated `kServoId = {10, 5, 6, 7, 8, 9, 1, 2,
  3, 4}`, slot 0 is servo 1, which is joint 6 (`R_hip_roll`). Putting the
  permutation on the boundary is what makes it testable end to end.
  `sil_spec()` reports the convention and the permutation, and the Python
  side **probes** the built library rather than assuming either (see the
  README).
- **The widths are the prototype's**: every array is static-asserted against
  `obs_spec.h` in `sil_lib.cpp`, so a retrain that changes a dimension breaks
  the build rather than silently reinterpreting bytes. The 17-joint port
  (issue #81) changes this ABI.
- `sil_abi` 2 is the current ABI: goal speeds in `SilTargets` and
  `sil_set_shaper()`, mirroring the firmware's command shaper
  ([firmware-design.md](firmware-design.md) §5.3). The harness resolves
  array order with the shaper off (the raw map is the invertible reference)
  and restores the pole, so closed-loop scoring runs against the shaped plant
  as deployed.

## Rules

- **The components are linked verbatim.** `obs/`, `policy/` and the
  actuation and calibration code are the firmware's own sources; there is no
  `#ifdef SIL` inside a component. A component that cannot be reused
  unmodified is a finding to report and fix minimally, not a thing to design
  around.
- **The tick sequence mirrors `main/ctrl_task.cpp`'s acting path line by
  line**: advance the gait clock (speed-scaled, frozen at a plain stand, as
  the spec says), assemble the frame, refill the ring on the first tick after
  a reset, build the observation, run the policy, then push the frame and
  latch `prev_action`, then shape. Getting that order wrong shifts the
  history by one tick, which no unit test of a single piece can see.
- **Not mirrored**, because they mean nothing without hardware: the bus
  transaction, the IMU driver, the pack guard, the link watchdog, the torque
  state machine and the takeover ramp. The watchdog's hold-last-target
  behaviour is a harness-side test (pyramid item 4).
- The shared library is built **without** sanitizers (a sanitized dylib
  cannot be `dlopen`ed into a stock Python); the same translation units are
  built with ASan/UBSan into `test_sil`, which feeds the parsers adversarial
  input.

## Weight export (`tools/export_policy_weights.py`)

`params.pkl` (brax) → `sim/sil/weights/<run>.silw` plus a JSON sidecar
`<run>.silw.json`. The canonical `.silw` is a self-describing binary: `SILW`
magic, dimensions, activation, layer sizes, then the normaliser mean and std
and every layer's weights and biases (layout in the README). The sidecar
carries `layer_sizes`, `activation: "swish"`, `obs_dim`, `act_dim`,
`normalizer.{mean,std}` and a `blob` block with the byte count and sha256.

- **The observation normaliser must ship with the weights**: the firmware
  net consumes normalised observations.
- The exporter asserts activation and dimensions against the run's config.
  The C reader accepts the headed blob or a headerless `--raw` one,
  cross-checks the sidecar when present, and errors on disagreement.
- The exporter also writes golden vectors to `sim/sil/golden/` unless given
  `--no-golden`. Those files are committed and consumed by the firmware host
  tests, so only a deliberate re-point should overwrite them.
- `.silw` blobs are gitignored and regenerated from the run's `params.pkl`;
  sidecars, golden vectors and calibration files are committed.

## Calibration file (for `sil_init`)

JSON with per-joint `{bus_id, zero_steps, dir}`, plus the same three as flat
joint-indexed arrays. The harness writes a **nominal** file (every zero
2048, every `dir` +1, identical to `obs::Calibration`'s default) and a
**perturbed** one (random zeros in 2048 ± 350); the tests also build one
with flipped directions. None of them is the as-built calibration, and none
is meant to be: they are round-trip fixtures for a robot whose servos agree
with the plant. A consistent `dir` cancels through angle → ticks → angle, so
closed-loop scores do not depend on the choice.

## Test pyramid

The item numbers are cited from code.

1. **Unit** (C, `make -C firmware/host test`): `test_sil` covers the C
   boundary end to end — angle → ticks → angle within one tick quantum under
   each calibration, the bus permutation, the parsers; `test_sil_golden`
   checks the firmware's frames and actions against the sim's own numbers in
   `sim/sil/golden/`; `test_policy` checks the compiled-in net against numpy
   goldens.
2. **Boundary** (pytest, `sim/sil`): the servo-side view round-trips against
   `walker_env`'s joint state, and the synthesised IMU matches the env's
   observation channels.
3. **Closed loop** (pytest, `sim/sil`, marked `slow`): `stand_10s`,
   `line_1m` and `goal_home` through `SilActAdapter` on the suite's
   reference run, with the python referee's seeds. Pass rates must agree
   within one seed per scenario; per-tick observation and action divergence
   is logged, and with the gait clocks pinned together it must be tick
   quantisation only. Any larger divergence is a bug with a name, not a
   tolerance.
4. **Timing and watchdog semantics**: a skipped `sil_tick` (a simulated
   overrun) holds the last target, and a three-tick dropout does not fell the
   standing robot.

## The standing referee column

Every collected training run is scored through the firmware stack as well as
the python policy:

```sh
cd sim/mjx && JAX_PLATFORMS=cpu ../../.venv/bin/python eval_precision.py --run-name <run> --sil
```

- `--sil` puts `SilActAdapter` in the referee's `act()` slot (one adapter per
  episode, so the library's history ring and gait clock reset with the env)
  over `libctrl_sil`, the run's exported weights and the nominal
  calibration. Scenarios, seeds (`100·i + 7`), plant and scoring are
  identical to the python column; only who computes the action differs.
- Prerequisites are built on demand and never half-run: a missing library
  runs `make -C firmware/host sil`; a missing or stale `.silw` runs the
  exporter with `--no-golden`. Any failure exits with the build output and
  writes no scorecard.
- Output: `scorecard_sil.{md,json}` in the run directory, headed
  `[SIL (firmware stack)]`. The python `scorecard.md` is never touched.
- `infra/night_collect_local.sh` runs the SIL leg after the python leg and
  never loses a collection over it.
- The closed loop does **not** pin the gait clock: on the robot the firmware
  runs its own clock from wherever the episode starts, and the referee grades
  the outcome. Per-tick parity with aligned clocks is pytest's job.

## Findings

Numbered because code cites them.

1. **The referee feeds the training stacking.** The observation handed to the
   policy is `[f_t, f_{t−1}, f_{t−2}]`, as training and `obs::History` build
   it. `eval_precision.Driver` reuses the observation `env.step` returned and
   patches only the head frame's command channels; re-reading `env._obs()`
   would prepend a recomputed `f_t` to a ring that already holds it.
   `test_referee_driver_feeds_training_stacking` pins this.
2. **Frozen normaliser channels are a deployment hazard.** brax floors the
   normaliser std at 1e-6, so a channel training never varied turns any
   deviation *d* into *d*·10⁶ at the first layer. The crouch channel
   (`cmd[3]`) is fed by the pack guard's landing ramp on the robot, so
   training varies it (the crouch-command variation in `walker_env` and
   `env_mjx`). In the deployed run, `loco_v41rsi_b_s128r24`, the frozen
   channels are `linvel` (3), `height` and `cmd[5]` (`foot_dx`). The first
   four are hard zeros on the robot. `foot_dx` is not: a console that sends
   an extended frame with `foot_dx` ≠ 0 (clamped to ±0.05 m) puts it ~5·10⁴ σ
   out, so for this policy it must stay at 0. The exporter warns about frozen
   channels and `test_normalizer_frozen_channels_are_known` pins the set for
   the suite's run.
3. **The policy takes its weights as a parameter.** `policy::Net` /
   `forwardNet` run an exported `.silw` through exactly the on-target
   arithmetic; `forward()` binds the same code to the generated header.
4. **The encoder path never clamps to the joint range.** Only goals clamp
   (a target outside the mechanical range stalls a horn against a printed
   part); an encoder reports where the joint *is*. MuJoCo joint limits are
   soft, so a joint leaning on its stop sits past it, and the sensor path
   (`angle_to_steps(..., clamp_joint_range=False)`) must pass that through.

## Non-goals

UART framing and CRC in the loop (`linkproto` has its own golden tests);
ESP32 timing (the host build is logic-exact, not cycle-exact); MJX (the CPU
env is the deployment referee by project convention).

## Current state of the Python suite

`pytest sim/sil` is **not** part of the pre-push gate; the C half (pyramid
item 1) is. The suite currently runs in seconds and has **4 failing tests out
of 32**, all from one cause: its reference run (`SIL_RUN`, default
`loco_v8foot`, trained on `bimo_biped_v3yaw.xml`) no longer matches what
the library is compiled against (`obs_spec.h` and `weights.h` from
`loco_v41rsi_b_s128r24` on `bimo_biped_v5body.xml`) or the committed goldens
(`loco_v28crouch_s128r24`).

| failing test | what it finds |
|---|---|
| `test_policy_goldens_match_exported_weights` | the goldens name `loco_v28crouch_s128r24`, not `loco_v8foot` |
| `test_env_joint_limits_match_obs_spec` | the plant's joint ranges differ from `obs_spec.h` (e.g. `R_hip_roll` lower limit −0.436 vs −0.960 rad) |
| `test_target_reproduction_through_env` | the env clips targets to its own, narrower ranges (0.87 rad at `L_knee`) |
| `test_lib_obs_matches_python` | the phase diverges: the library runs the deployed spec's speed clock and stand freeze, which that run's env does not |

The closed-loop scenarios pass. Fixing this means re-pointing `SIL_RUN` and
the goldens at one run that matches the deployed headers.
