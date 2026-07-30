# Software-in-the-loop (SIL) harness — design

*2026-07-30, task: run the REAL control code against the simulated plant.
Sensors flow sim → firmware logic; firmware output drives the simulated
servos. Every sim-to-real translation layer gets exercised and SCORED by
the same CPU referee that grades policies.*

## Why

The bench found an entire class of bugs the python stack can never see:
byte order (the vendor UI's SC/ST swap), the kServoId permutation that
wiring.md wrongly called an identity, calibration zeros, swish-vs-tanh in
the MLP, the 147-not-129 obs width. The SIL harness makes each of those a
red test on a laptop instead of a robot walking into a wall.

## Loop topology (one 20 ms tick)

```
walker_env (CPU referee plant, hardware-claim DR)
  |  SERVO-SIDE VIEW: per-joint angle -> calibrated ticks (uint16,
  |    per-joint zero_steps/dir, 4096/rev quantization); velocity ->
  |    signed steps/s. IMU: up-vector + gyro through the env's mounting
  |    error + noise. Exactly what SYNC READ + the IMU driver deliver.
  v
struct SilSensors                                  [C boundary, below]
  v
libctrl_sil -- REAL CODE, no sim knowledge:
  stepsToAngle(Calibration) -> obs::assembleFrame -> obs::History
  -> gait clock -> policy::forward (exported weights + obs normalizer)
  -> action -> target angles -> angleToSteps(Calibration)
  -> per-BUS-ID targets (kServoId order)
  v
struct SilTargets                                  [C boundary, below]
  v
harness: permute bus IDs back to sim joint order -> invert the env's
  action->target map -> env.step(action). Driver/eval_precision scenarios
  run UNCHANGED, so referee claims apply to the SIL stack directly.
```

## C ABI (firmware/host/sil_lib.cpp -> libctrl_sil.so/.dylib)

```c
typedef struct {            // what the bus + IMU actually deliver
    uint16_t pos_ticks[10]; // present position, reg 56 semantics, BUS-ID
                            //   order kServoId = {9,1,2,3,4,10,5,6,7,8}
    int16_t  vel_ticks[10]; // steps/s, sign-magnitude BIT15 like reg 58
    float    up[3];         // IMU up-vector, body frame
    float    gyro[3];       // rad/s
    float    cmd[7];        // command channels (harness-supplied)
} SilSensors;

typedef struct {
    uint16_t goal_ticks[10];  // SYNC WRITE view, BUS-ID order
    float    action[10];      // the raw [-1,1] policy action (diagnostics)
    float    obs[147];        // the assembled obs (diagnostics/golden)
} SilTargets;

int  sil_init(const char* weights_path, const char* cal_path,
              float gait_freq_hz);       // 0 on success
void sil_reset(void);                    // history refill + clock zero
int  sil_tick(const SilSensors* in, SilTargets* out);
const char* sil_spec(void);              // obs_spec hash + dims, drift check
```

Rules: the library links obs::, policy::, and the actuation/Calibration
code VERBATIM. No `#ifdef SIL` forks inside components. If a component
cannot be reused without modification, that is a finding to REPORT (and
minimally fix), not to design around.

## Weight export (tools/export_policy_weights.py)

params.pkl (brax) -> `<run>.silw`: little-endian f32 blob + JSON sidecar
{layer_sizes, activation:"swish", obs_dim, act_dim, normalizer:{mean,std}}.
The obs normalizer (brax running_statistics) MUST ship with the weights --
the firmware net consumes NORMALIZED obs. Exporter asserts activation and
dims against the run config.

## Calibration file (cal file for sil_init)

JSON: per-joint {bus_id, zero_steps, dir}. The harness generates the
NOMINAL one (zero_steps=2048, dir from the sim sign convention vs servo
convention) and one PERTURBED one (random zeros) for the roundtrip test.

## Test pyramid (all must pass)

1. **Unit (ctest, firmware/host)**: angle->ticks->angle roundtrip |err| <=
   one tick quantum under both cal files; permutation is kServoId exactly;
   obs frame vs frozen vectors (existing) plus NEW vectors generated from
   walker_env states; policy forward vs brax golden pairs, atol 1e-5.
2. **Boundary (pytest, sim/sil)**: harness's servo-side view roundtrips
   against walker_env's joint state; IMU synth matches env obs channels.
3. **Closed loop (pytest, sim/sil)**: stand_10s, line_1m, goal_home via
   the SIL adapter on loco_v6creep (complete run), same seeds as the
   python referee -- pass rates within 1 seed per scenario, and per-tick
   obs/action divergence logged (expect ~tick-quantization noise only).
4. **Timing/watchdog semantics**: a sil_tick that is skipped (simulated
   overrun) must hold the last target -- mirror of the firmware watchdog;
   test that a 3-tick dropout does not fall the standing robot.

## Acceptance

`make -C firmware/host sil && ctest` green, `pytest sim/sil -q` green,
and a SIL-vs-python referee table appended to the harness README. Any
divergence beyond tick quantization is a bug with a name, not tolerance.

## Non-goals (this round)

UART framing/CRC in the loop (linkproto has its own golden tests); ESP32
timing (host build is logic-exact, not cycle-exact); MJX (referee plant
only -- the CPU env is the deployment referee by project convention).

## Work split

- **Agent A (C++)**: sil_lib.cpp + CMake/Makefile `sil` target + ctests
  (roundtrip, permutation, policy-vs-golden consuming Agent B's exports).
  Files: firmware/host/** only (report component bugs, minimal fixes OK).
- **Agent B (python)**: tools/export_policy_weights.py, golden-vector
  generator, sim/sil/harness.py + SilActAdapter + pytest suite. Files:
  tools/export_policy_weights.py, sim/sil/** only. ctypes on the ABI
  above; skip-if-missing when the lib is not yet built.
- Integrator (main session): build both, run the closed loop, write the
  results table, commit.
