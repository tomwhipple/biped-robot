# Software-in-the-loop (SIL) harness — python side

The real firmware control path, driven by the CPU referee plant. Design
contract: [`docs/sil-harness.md`](../../docs/sil-harness.md).

```
walker_env  --calibrated ticks + IMU floats-->  libctrl_sil  --goal ticks-->  walker_env
            (SilSensors)                       (REAL code)   (SilTargets)
```

Nothing in `libctrl_sil` knows it is talking to a simulator: the obs
assembler, the history ring, the gait clock, the MLP and the
angle↔tick calibration are the firmware's own sources compiled for the host.
Everything in this directory is the translation layer plus its tests.

---

## Build and run

```sh
# 1. export the trained policy (weights + normalizer + golden vectors)
JAX_PLATFORMS=cpu .venv/bin/python tools/export_policy_weights.py --run loco_v6creep

# 2. build the library (Agent A's side)
make -C firmware/host sil          # -> firmware/host/build/libctrl_sil.dylib
make -C firmware/host test         # pyramid item 1 (ctest / host tests)

# 3. python side
JAX_PLATFORMS=cpu .venv/bin/pytest sim/sil -q                 # everything
JAX_PLATFORMS=cpu .venv/bin/pytest sim/sil -q -m "not slow"   # skip closed loop
```

Knobs (environment):

| var | default | meaning |
|---|---|---|
| `SIL_LIB` | search `firmware/host/build`, then `firmware/host` | explicit library path |
| `SIL_RUN` | `loco_v6creep` | run whose `.silw` is loaded |
| `SIL_SEEDS` | `8` | closed-loop episodes per scenario (seeds are `100*i + 7`, exactly `eval_precision`'s) |
| `SIL_ASSUME_ORDER` | *(auto-probed)* | force `joint` / `ascending_id` array order |

Without the library, the 8 lib-dependent tests **skip**; the other 22 (export,
goldens, boundary, inverse action map) still run — those are the layers the
python side owns.

---

## Files

| file | what it is |
|---|---|
| `tools/export_policy_weights.py` | `params.pkl` → `.silw` + sidecar + golden vectors |
| `harness.py` | ctypes ABI, calibration, boundary conversion, `SilActAdapter` |
| `test_sil.py` | pyramid items 2, 3, 4 |
| `weights/<run>.silw(.json)` | exported net (≈949 KB f32) |
| `golden/policy_vectors.json` | 16 normalized-obs → action pairs (brax, deterministic) |
| `golden/obs_vectors.json` | 8 `walker_env` states → `_obs()` frame + `obs::Inputs` |
| `cal/cal_{nominal,perturbed}.json` | the two calibrations `sil_init()` takes |

---

## The `.silw` layout as written

Little-endian throughout. `header_bytes` is 64 for the deployed
`147 → 512 → 256 → 128 → 20` net (`28 + 16 + 4*5`).

```
off  type          n           field
  0  char[4]       4           magic "SILW"
  4  u32           1           version = 1
  8  u32           1           header_bytes  (byte offset of the f32 payload)
 12  u32           1           obs_dim       (147)
 16  u32           1           act_dim       (10)
 20  u32           1           num_layers    (4)
 24  u32           1           num_sizes     (5 = num_layers + 1)
 28  char[16]      16          activation, NUL-padded ("swish")
 44  u32           num_sizes   layer_sizes = [147, 512, 256, 128, 20]
 -- header_bytes --
     f32           obs_dim     norm_mean
     f32           obs_dim     norm_std
     f32           in_0*out_0  W0, row-major (in, out) — flax order, no transpose
     f32           out_0       b0
     ...                       W1, b1, W2, b2, W3, b3
```

Total payload `2*147 + 242580 = 242874` f32. `--raw` writes the payload with
no header (`header_bytes = 0`); the sidecar is then the only description.

The sidecar `<run>.silw.json` carries the design doc's fields verbatim —
`layer_sizes`, `activation: "swish"`, `obs_dim`, `act_dim`,
`normalizer.{mean,std}` — plus a `blob` block (`header_bytes`, `order`,
`n_floats`, `sha256`) so a reader can cross-check the bytes.

Semantics the consumer must reproduce (all in float32):

```
x      = (obs - mean) / std          brax running_statistics.normalize, no clip
h      = swish(h @ W_i + b_i)        every hidden layer
logits = h @ W_out + b_out           linear, 2 * act_dim wide
action = tanh(logits[:act_dim])      NormalTanhDistribution.mode()
```

## The calibration file

```json
{"num_joints": 10,
 "joints": [{"index": 0, "name": "L_hip_yaw", "bus_id": 9,
             "zero_steps": 2048, "dir": 1}, ...],
 "bus_id": [...], "zero_steps": [...], "dir": [...]}
```

The three flat arrays are joint-indexed mirrors of `joints`, so a reader can
use either shape. `cal_nominal.json` is `zero_steps = 2048, dir = +1`
(identical to `obs::Calibration`'s C++ default); `cal_perturbed.json` has
random zeros in `2048 ± 350`.

---

## Results — SIL vs python referee

**TODO (integrator):** rebuild both sides, run
`JAX_PLATFORMS=cpu .venv/bin/pytest sim/sil -q -s`, and fill this in from the
printed lines.

| scenario | seeds | python referee | SIL stack | Δ |
|---|---|---|---|---|
| `stand_10s` | | | | |
| `line_1m` | | | | |
| `goal_home` | | | | |

| divergence (per tick, `stand`/`walk`, clocks aligned) | value |
|---|---|
| obs `q` / `dq` block | |
| obs `up` / `gyro` / `cmd` | |
| obs history ring | |
| firmware net vs brax, same obs (`net_err`) | |
| firmware action vs brax on training-stacked obs | |

<details>
<summary>Development reference (2026-07-30, first green run — not the official table)</summary>

Library built from `firmware/host` at 09:32, weights `loco_v6creep.silw`,
nominal calibration, gait clock pinned to 1.5 Hz, clean (`--nominal`) plant.

- ABI probe: `in_order = out_order = ascending_id`, `vel_sign_magnitude = True`
- obs `q` 7.66e-4 rad, `dq` 7.66e-4 rad/s (= ½ tick), `up`/`gyro`/`cmd`/
  `linvel`/`height` **0.0**, `phase` 6.2e-6, `prev_action` 8.7e-3, history
  ring 8.7e-3 (½ tick expressed in action units — the knee's positive span is
  only 0.087 rad, so that block's quantum is the largest)
- `net_err` (firmware MLP vs brax on the *same* obs) **4.2e-7**
- firmware action vs brax on training-stacked obs: **4.9e-3** — quantization only
- closed loop, 8 seeds, hardware-claim plant: `stand_10s` 8/8 vs 8/8,
  `line_1m` 8/8 vs 8/8, `goal_home` 8/8 vs 7/8

</details>

---

## Notes for whoever touches this next

**The ABI is auto-probed, not assumed.** `docs/sil-harness.md` describes
`pos_ticks` as "BUS-ID order kServoId = {9,1,2,3,4,10,5,6,7,8}", which reads
either as *slot k carries bus id kServoId[k]* (i.e. joint order, how
`ctrl_task.cpp` lays out `g_fb`) or as *slot = bus_id − 1*. `sil.h` chose the
latter. Rather than hard-code a guess, `SilLib.probe()` ticks once with a
distinguishable tick pattern and reads `SilTargets.obs` back to determine, in
the tick domain (so a non-uniform calibration cannot masquerade as a
permutation): the sensor array order, the `goal_ticks` order, and whether
`vel_ticks` is two's complement or the servo's sign-magnitude-bit-15 word.
A disagreement is a `ProbeFailed` with both candidate errors in the message.

**The referee's live obs is one frame of history short.** `walker_env.step()`
returns `[f_t, f_{t-1}, f_{t-2}]` and pushes `f_t` — the same stacking
`env_mjx` trains with and `obs::History` implements. But
`eval_precision.Driver` re-reads `env._obs()` at the top of each tick (it must:
`set_command` has to land in the current frame), and that read *prepends a
freshly recomputed `f_t` to a ring that already contains `f_t`*. The referee
therefore feeds the policy `[f_t, f_t, f_{t-1}]`. The firmware is right and
the referee's live-read path is not; it is the single largest SIL-vs-referee
action divergence (0.48 vs 4.9e-3 against the training stacking).
`test_referee_live_obs_is_one_frame_short` pins this, and
`SilActAdapter.training_obs()` reconstructs the correct vector for the
comparison. Fixing `eval_precision` is out of scope here — but every referee
number ever produced was measured through that path.

**The gait clocks tick at opposite ends.** `ctrl_task.cpp` advances the clock
*before* assembling the frame; `walker_env.step()` advances at the *end*. So
the firmware's phase at tick k is `(k+1)·ω` and walker_env's is `p₀ + k·ω`.
`pin_gait_clock(env, hz)` sets `p₀ = ω` (and refills the history ring) to make
them equal — needed only for the per-tick parity tests. The closed-loop
scenarios deliberately do *not* pin: on hardware the firmware runs its own
1.5 Hz clock while the episode starts wherever it starts, and the referee
grades the outcome.

**Eight obs channels are frozen, and one of them is a live hazard.**
`loco_v6creep` never varied `linvel` (slots 23–25), `height` (39) or command
channels `cmd[3..6]` — crouch, lift, foot_dx, foot_dz (45–48). brax's
`running_statistics` floors their std at `std_eps = 1e-6`, so a deviation `d`
from the recorded mean arrives at the first layer as `d · 10⁶`. The exporter
prints a warning listing them and `test_normalizer_frozen_channels_are_known`
pins the set.

The hazard: `cmd[3]` (crouch) is frozen at exactly 1.0, but
`ctrl_task.cpp` feeds it `battguard::Guard::crouch()`, which *ramps below 1.0
on a flat pack* — deliberately, to "stay inside the training distribution".
For this run it does the opposite: `crouch = 0.85` normalises to −1.5e5 and
the policy's output is undefined at exactly the moment the robot is supposed
to land gently. Either train the crouch channel or pin `in.cmd[3] = 1.0` and
land some other way. (Out of scope for this harness — reported, not fixed.)

**Why the harness takes the IMU from the obs, not from `env.data`.**
`walker_env._obs()` pushes the up-vector through a per-episode mounting
rotation and adds noise, and biases the gyro — and re-draws that noise on
every call. Reading those channels back out of the obs frame is the only way
to put the *same* IMU realisation inside the SIL loop that the python policy
would have seen; reading `sensordata` would quietly bypass the mounting error
the harness exists to exercise.
