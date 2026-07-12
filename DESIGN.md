# Bimo-like Biped — Design & Simulation Working Doc

**Status:** CAD massing model + MuJoCo physics validation complete (Stage 0–1). CPU RL baseline done incl. gait-quality shaping + uneven-terrain curriculum (Stage 2b). Printable part set done (build123d, `cad/`, interference-checked). **Buildable-robot retrain done:** CAD-true model `sim/bimo_biped_v2.xml` (mesh inertia) + honest STS3215 torque-speed actuator model + camera payload + 2 m dash — recommended policies `dash_11v1_hard` (3S, 2 m in ~2.7 s) / `dash_7v4_hard` (2S, ~4.4 s). Nothing printed yet; sim-to-real not started.
**Last updated:** 2026-07-11
**Owner:** Tom

---

## 1. Goal

Build a custom, small (~34 cm), 3D-printed **bipedal robot** using an LLM-assisted
loop: parametric CAD → physics simulation → print → RL walking policy → sim-to-real.
Inspired by (not a clone of) the open-source **Bimo Project**.

This folder holds a first-pass **parametric CAD massing model** and a **MuJoCo
physics model** used to validate the mechanism *before* committing to printed parts.

## 2. Key decisions & rationale

- **Reference platform:** The Bimo Project by Mekion — `github.com/mekion/the-bimo-project`,
  a $500 fully-3D-printable biped shipping with an NVIDIA Isaac Lab RL env and working
  sim-to-real. As of 2026-07 it is pre-order / early access: CAD files "coming soon",
  and its custom RP2040 controller board is the one non-COTS component.
- **Strategy: do NOT reverse-engineer Bimo's exact parts from photos/video.** Instead
  design a *similar* platform around **known component dimensions**. For a servo robot
  the servos are the skeleton, so the datasheet — not the video — anchors the geometry.
- **Actuator:** 8× **Feetech STS3215** serial-bus servo. Measured **45.2 × 24.6 × 35.1 mm**,
  ~55 g, ~30 kg·cm (**2.94 N·m**) stall @ 12 V. Same class as LeRobot/SO-ARM, easy to source.
- **DOF layout:** 4 per leg = **hip-roll (X), hip-pitch (Y), knee (Y), ankle (Y)** → 8 total.
- **CAD tool:** parametric **OpenSCAD** (render-in-the-loop). Chosen because it's text/
  parametric, LLM-friendly, and headless-renderable.
- **Simulator:** **MuJoCo** (not Isaac Lab). MuJoCo is CPU/laptop-friendly, `pip install`,
  gold-standard contact physics, and is the universal validation layer even for Isaac users.
  Isaac Lab needs a workstation NVIDIA GPU + 1–2 week setup; overkill at this scale.
  For GPU-parallel RL later, use **MuJoCo MJX** (JAX) on the RTX 4070. Genesis is emerging
  but bleeding-edge.
- **User hardware:** RTX 4070 (12 GB) + Mac laptop. Mac runs MuJoCo's native viewer for
  design; 4070 runs MJX training.

## 3. Folder contents

See the directory tree in [README.md](README.md) — kept in one place so it can't
drift. Highlights: `cad/` is the parametric printable design (build123d;
`dimensions.py` is the single source of truth), `sim/` is the MuJoCo env + PPO
training/eval/report tooling, `sim/runs/` (gitignored) holds trained policies.
`requirements.txt` pins all deps.

## 4. Current state — what works

- **CAD (Stage 0, done):** massing model renders correctly; proportions and the 8-servo
  layout are established. Servo bodies double as the limb segments; gray brackets connect
  them; blue box = SBC head cavity. **This is a concept, NOT printable parts** — no screw
  bosses, heat-set seats, horn-spline interfaces, wire channels, or tolerances.
- **Sim (Stage 1, done):** the physics model **stands** under gravity (settles ~2 mm,
  perfectly upright) and completes a **coordinated squat while staying balanced** — all
  eight joints actuated, uprightness stayed 1.000, recovered to standing. Mechanism is
  physically sane. Verified numerically and by rendering.

## 5. How to run

### OpenSCAD (CAD renders)
```bash
# headless render (add xvfb-run on a server); pose override via -D
xvfb-run -a openscad -o out.png --preview --projection=perspective \
  --imgsize=900,1100 --camera=0,0,0,62,0,25,0 --viewall --autocenter \
  -D 'gait=24' -D 'p_knee=-16' cad/bimo_like_biped.scad
```
Camera note (learned the hard way): `--camera=…,90,0,0,…` = **side** view, `…,90,0,90,…`
= **front** view. Use `--preview` (not `--render`) or all `color()` flattens to gold.

### MuJoCo (physics sim)
```bash
pip install -r requirements.txt                   # mujoco, gymnasium, imageio, numpy
MUJOCO_GL=osmesa python3 sim/sim_biped.py        # headless squat (Linux server only)
# On the Mac (native GL, interactive) — drag joints, push the robot:
python -m mujoco.viewer                           # then open sim/bimo_biped.xml
```
Note: `MUJOCO_GL=osmesa` is Linux-only headless rendering; on the Mac omit it
(native GL). The RL env below steps physics without rendering, so it needs no GL.

### RL env (Stage 2a — Gymnasium)
```bash
cd sim && python smoke_test.py                    # validate the env end-to-end
```
Env: 36-dim obs (joint qpos/qvel, torso up-vector, torso lin/ang vel, prev
action, height, phase clock); 8-dim action = residual target angles around the
standing pose (action 0 == stand); reward = forward-vel + upright + alive −
energy − action-rate; terminate on height < 0.18 m or up-vector z < 0.4.
Smoke test confirms a zero policy stands 500 steps upright and bad policies fall.

### RL training (Stage 2b — SB3 PPO CPU baseline)
```bash
cd sim
# baseline (uncapped forward reward):
python train_ppo.py --steps 1000000 --n-envs 8 --run-name ppo_baseline
# shaped (capped forward + posture + fall penalty) -- reward knobs are CLI flags:
python train_ppo.py --steps 2000000 --n-envs 8 --run-name shaped_v1 \
  --w-forward 1.5 --target-speed 0.4 --w-upright 0.8 --alive 0.15 \
  --w-height 0.3 --fall-cost 3.0
python eval_policy.py --run-name shaped_v1 --episodes 10 --render
tensorboard --logdir runs                          # compare runs side by side
```
SB3 PPO on CPU proves the reward produces a *sustained* gait before the MJX/GPU
port. Actions = residual angles around stand; VecNormalize on obs+reward. Each
run writes to `runs/<run-name>/` (model, vecnormalize.pkl, tensorboard, walk.gif).
Success = high episode length (survives the 10 s truncation) **and** forward
distance — not distance alone.

**Finding (baseline, 1M steps):** uncapped forward reward is exploitable — the
policy learned to **lunge/faceplant forward** (~1.3 m at 0.68 m/s, then falls;
0/10 episodes survived the full 10 s). Fix: `--target-speed` caps the forward
term so a one-step dive can't out-earn a steady gait; `--w-height`/`--fall-cost`
reward staying upright. `eval_policy.py` now reports episode length + survival
rate + avg speed so a lunge is distinguishable from a walk; `compare_runs.py`
ranks all runs and classifies each (lunge / walk-then-falls / stands / walks).

**Result (shaped_v1, 2M steps, capped 0.4 m/s + posture + fall penalty):** a real
**alternating-leg walking gait** emerged — ~2.9 m over ~6 s at a controlled
0.46 m/s (episode length 317/500 vs baseline 109). It still tips before the 10 s
truncation (0/10 full survival), and training had not plateaued, so `shaped_v2`
extends the same config to 5M steps to push toward robust full-episode survival.

**Result (shaped_v2, 5M steps, same reward):** **robust forward walk achieved.**
Survives **11/12** full 10 s episodes, walking **~4.6 m at 0.48 m/s** with a clean
upright alternating-leg gait (`runs/shaped_v2/walk.gif`, `strip.png`). This meets
the Stage-2b CPU-baseline goal — the env + reward produce a stable gait, so the
MJX/GPU port is now worth doing. Reward config is the shaped_v1/v2 flag set above.

### Domain randomization (sim-to-real hardening)
`walker_env.py` supports opt-in DR (`--domain-rand`): per-episode randomization of
body mass+inertia (±15%), floor friction (±40%), actuator gain (±20%), optional
action latency, and random horizontal torso shoves. The flat-ground shaped_v2
policy is **brittle** under this — survival drops 11/12 → 6/12 (mass/friction/gain)
→ **1/12 with shoves** — which is exactly the reality gap DR closes.

`train_ppo.py --init-from <run>` warm-starts from a saved policy+normalizer, so we
use a **curriculum**: learn to walk on flat ground first (shaped_v2), then
fine-tune under DR rather than relearning to walk while being shoved.

**Failed attempt (shaped_v3):** warm-start + *full* DR at once (12 N shoves ≈ 1.3×
body weight, ~1/s, + action latency) **destroyed the gait** — catastrophic
forgetting, nominal survival 11/12 → 0/12, training episode length collapsed to
~84. Lesson: too-hard DR from a warm start unlearns the skill. Shove default
lowered to 5 N @ 1% and the curriculum split into stages with a low fine-tune LR:
```bash
# stage 1 (shaped_v4): harden vs model error (mass/friction/gain), NO shoves, gentle LR
python train_ppo.py --steps 4000000 --n-envs 8 --run-name shaped_v4 \
  --init-from shaped_v2 --domain-rand --push-force 0 --lr 0.0001 \
  --w-forward 1.5 --target-speed 0.4 --w-upright 0.8 --alive 0.15 \
  --w-height 0.3 --fall-cost 3.0
# stage 2: warm-start from shaped_v4 and add gentle shoves (--push-force 3/5)
```
Check robustness with `python compare_runs.py --dr` (evaluates under DR + shoves).

**Robustness gauntlet** (survival over 16 episodes; nominal / model-error DR /
DR+shoves+latency @ 5 N):

| policy | nominal | model-error DR | DR + shoves + latency |
|---|---|---|---|
| shaped_v2 (flat ground) | 14/16 | 16/16 | 0/16 |
| shaped_v4 (v2 + DR, no shoves) | 16/16 | 16/16 | 0/16 |
| shaped_v5 (v4 + harsh shoves+lat) | 14/16 | 8/16 | 0/16 (dist 2× v4) |
| **shaped_v6 (v4 + gentle shoves)** | **16/16** | **16/16** | 0/16 |

**Conclusion (recommended policy = `shaped_v6`):** a perfect nominal gait
(16/16, ~5.3 m at 0.52 m/s) that is **fully robust to ±15% mass / ±40% friction /
±20% gain** model error — a genuinely sim-to-real-relevant result. Two shove-
hardening attempts did **not** crack robustness to *repeated* random shoves:
harsh shoves (v5) bought partial recovery (2× distance under shoves) but hurt the
clean gait and model-error robustness; gentle shoves (v6) preserved the gait but
taught no recovery. **Open question flagged for review:** surviving repeated
shoves needs a design decision — a realistic shove magnitude/cadence, and likely
a *balance-recovery* reward term rather than bare survival — not more of the same
fine-tuning. **Use `shaped_v6` as the MJX-port baseline.**

### Gait quality + uneven terrain (Stage 2b continued, 2026-07-10)

Two problems with `shaped_v6`: the gait *looked* wrong (quick shuffly steps —
measured mean swing time only **0.11 s** vs the ~0.25 s of a deliberate stride),
and it fell on even **5 mm** bumps (6/16 survival). Fixed both with gait-quality
reward terms + a procedural-terrain curriculum. All new knobs are constructor
kwargs on `BimoWalkerEnv` with **default 0 / off, so every old run reproduces
exactly** (verified: bit-identical rewards on flat ground with defaults).

**Terrain (env):** `terrain_amplitude` (max bump height, m) swaps the flat plane
for a MuJoCo **heightfield** at model-load time (the MJCF is patched in-memory;
`bimo_biped.xml` unchanged). Height data = smoothed uniform noise
(`terrain_smoothness` = feature size, m), re-randomized **every reset** — on CPU
MuJoCo, collisions read `model.hfield_data` live, so rewriting it before
`mj_resetData` suffices (verified: a +20 mm plateau written directly into
`hfield_data` raised the resting torso by exactly +20 mm without recompiling).
A smoothstep-flattened spawn pad keeps episode starts level.
`terrain_amplitude_min` draws a per-episode amplitude in `[min, max]` (built-in
difficulty mixing); `--terrain-mix` (train_ppo) makes a fraction of the parallel
workers use the original flat plane, because **hfield and plane contacts differ
subtly** — a policy trained only on hfields measurably degrades on the plane
(9/16 plane vs 14/16 on a flat hfield) until it trains on both. Termination and
the height observation now use **height above local ground** (bilinear hfield
lookup) instead of world z, so slopes don't falsely terminate; on flat ground
this is numerically identical to before.

**Gait shaping (env, all CLI flags):** `w_feet_air` — legged_gym-style feet
air-time reward: on each touchdown, `min(swing, air_time_target) − 0.5·target`,
so swings shorter than half the target *cost* reward (kills shuffle-taps) and
hopping isn't incentivized (capped); `w_single_support` — per-step bonus when
exactly one foot has contact; `w_lateral` — |lateral vel| + drift penalty;
`w_yaw` — heading + yaw-rate penalty; `w_pitch_rate` — torso roll/pitch-rate
penalty. Foot contact is detected from the contact list against the sole geoms
(soles are the only colliding robot geoms). `eval_policy.py`/`compare_runs.py`
now report **double-support %** and **mean swing time**, and take
`--terrain-amplitude` for terrain evaluation.

**What worked / didn't (training loop):** warm-starting `shaped_v6` with gait
terms (gait_v1/v2) could NOT escape the shuffle local optimum — swing time
stayed 0.11 s. **From scratch with gait shaping baked in** (gait_v3b → v4,
w_feet_air 20, single-support 0.3, target_speed 0.5) restructured the gait
completely: swing 0.27 s, double-support ~0%, and — although the forward term is
still capped at 0.5 m/s — the air-time/single-support rewards push a faster,
more dynamic stride (~0.8 m/s). Speed was raised by *shaping*, not by uncapping
the forward term, so the lunge exploit stays dead. DR fine-tune (gait_v6) kept
16/16 under ±15% mass/±40% friction/±20% gain. Terrain curriculum: 5 mm fixed
(terrain_v1) → mixed 0–12 mm (terrain_v2) → mixed 0–15 mm + 25% flat-plane
workers (terrain_v3) → 10M-step low-LR consolidation (terrain_v4).

**Results** (24 episodes each; survival / distance / speed; dsup = double-support
fraction, swing = mean per-step swing time; `compare_runs.py --terrain-amplitude`):

| policy | flat | 10 mm terrain | 15 mm terrain | dsup | swing |
|---|---|---|---|---|---|
| shaped_v6 (old best) | 23/24, 5.1 m @ 0.53 | 15/24, 3.8 m | — | 14% | 0.11 s |
| gait_v6 (clean gait, flat-only) | **24/24, 8.3 m @ 0.83** | 0/24 | — | 0% | 0.28 s |
| terrain_v3 | 21/24, 7.3 m @ 0.77 | 21/24, 7.4 m @ 0.80 | 18/24 | 0% | 0.28 s |
| **terrain_v4 (recommended)** | **22/24, 7.6 m @ 0.79** | **20/24, 7.6 m @ 0.81** | **20/24, 7.6 m @ 0.81** | 0% | 0.28 s |

`terrain_v4` also keeps model-error DR robustness: 22/24 flat / 20/24 on 10 mm
terrain under ±15% mass / ±40% friction / ±20% gain. Compared to `shaped_v6` it
is ~55% faster, replaces the 0.11 s shuffle with a clear 0.28 s alternating
stride (see `runs/terrain_v4/walk.gif`, `walk_terrain10mm.gif`,
`walk_terrain15mm.gif`, and `runs/gait_v6/walk.gif` for the flat specialist),
and survives 15 mm bumps — half the foot length — most episodes.
**Use `terrain_v4` as the new baseline; `gait_v6` if flat-ground speed is all
that matters.** Training throughput note: this machine trains ~12k env-steps/s
(8 workers), so 8–10M-step runs finish in ~10–15 min — iterate freely.

**Caveats / open questions:** the new gait has a flight phase (double-support
~0%), i.e. it is closer to a jog than a walk — real STS3215 servos may not
track it; if sim-to-real needs a grounded walk, re-tune with a both-feet-down
allowance (e.g. lower air_time_target, add a flight penalty). Repeated-shove
robustness remains open (unchanged from the DR section above). Terrain height
is NOT observed by the policy (blind proprioceptive walking) — an exteroceptive
height-scan obs is the natural next step for rougher ground.

### Buildable-robot retrain: CAD-true model, real actuators, camera payload, 2 m dash (2026-07-11)

Retrained everything on the **buildable robot**: `sim/bimo_biped_v2.xml` (CAD-true
masses/geometry) with an **honest STS3215 actuator model**, a **GoPro MAX 360
camera payload** option, and a **timed 2 m dash** objective. All env additions are
opt-in constructor kwargs / CLI flags defaulting to old behavior (verified
bit-exact on flat/terrain/DR reward traces; `smoke_test.py` and a `terrain_v4`
re-eval reproduce). Each training run now writes `runs/<name>/env_config.json`;
`eval_policy.py`/`compare_runs.py` read it so every run is automatically
evaluated on the model/actuator it was trained with (old runs -> old xml + ideal
actuators; overrides: `--xml --actuator-model --voltage --payload --dash`).

**v2 model (`bimo_biped_v2.xml`):** validated — same joint/qpos layout, sensors
and sole geoms as v1; total 0.892 kg; settles standing at z=0.2827 (nominal
0.283). Now carries **CAD-true per-body inertia**: explicit `<inertial>` per
body computed by `sim/build_v2_inertia.py` from the printed-part STL meshes
(`cad/stl/`, PLA rho x0.90 print factor) plus servo/battery/board boxes at their
CAD positions, scaled to the mass rollup (screws/wiring pro-rata). Box geoms
remain for collision/visuals. `bimo_biped.xml` untouched (old runs reproduce).

**Actuator model (`actuator_model="sts3215"`):** torque computed per sim substep
as PD on the commanded angle, `tau = kp (target - q) - kd qdot`, then magnitude-
clamped to the DC-motor torque-speed envelope `|tau| <= stall x max(0, 1 -
|qdot|/w_noload)`; applied via `qfrc_applied` with the MJCF position actuators
silenced. Datasheet numbers (Waveshare ST3215 wiki, checked 2026-07-11): **30
kg.cm = 2.94 N.m stall and 0.222 s/60deg = 4.71 rad/s no-load, both @ 12 V;
rated input 6-12.6 V** (2S and 3S LiPo both in-spec). Stall and no-load speed
scale linearly with voltage: **7.4 V -> 1.81 N.m / 2.91 rad/s; 11.1 V -> 2.72
N.m / 4.36 rad/s**. Fitted **kp=12, kd=0.25 N.m/rad**, and in sts3215 mode the
joint damping drops 0.6 -> **0.1** N.m.s/rad — the 0.6 was a stability proxy for
the ideal servo and double-counts motor losses the no-load speed already
includes (at 3 rad/s it alone would eat the whole stall torque). Step-response
check: 60 deg step @ 7.4 V reaches 90% in 0.394 s vs the 0.360 s envelope
minimum, ~1.4% overshoot — fast and slightly damped, servo-like. Under DR the
servo params randomize per episode: kp/kd +/-20% (gain_range), stall/no-load
+/-15% (servo_range).

**The old gait was unbuildable — design feedback:** measured on its own env,
`terrain_v4` demands **median |tau| = 3.0 N.m (the ideal clamp) at median joint
speeds of 4.3-4.4 rad/s** on hip/knee/ankle. A real STS3215 at that speed
delivers ~0.2 N.m *at 12 V* and nothing at 7.4 V — the ideal-servo policies were
exploiting infinite bandwidth, and zero-shot transfer to the sts3215 model
collapses in <1.5 s at any voltage. Every deployable policy must train against
the envelope.

**7.4 V (2S) feasibility verdict: YES for walking, with a speed cost.** At 7.4 V
the robot stands (also with the 154 g camera), and a from-scratch retrain
(`v2_scratch_7v4`) walks **16/16 full 10 s episodes at 0.45 m/s** with a clean
0.28 s stride and 2% double-support — a more grounded, upright gait than the
11.1 V jog (the envelope itself regularizes the gait). The warm-start from
`terrain_v4` could NOT restructure (0.35 m/s, falls at ~3 s, plateaued —
same lesson as gait_v1/v2: big plant changes need from-scratch). At **11.1 V
(3S)** the warm-start *does* adapt (10/12, 0.63 m/s) because the envelope is
~2.3x roomier. **Recommendation: 2S works and is the safer servo-rated choice
(wiki: 6-12.6 V), but the dash is ~1.8x slower; if dash time matters, move the
electronics bay to 3S (11.1 V) — in-spec for the servo.**

**Camera payload (`payload_mass` / `payload_max`):** GoPro MAX 360, **154 g,
64x69x25 mm box, CG +0.08 m above the torso center** (tower plate 0.319 m +
folding-finger mount), MJCF-patched as a welded child body with real box
inertia; `payload_max` re-draws the mass uniform(0, max) each episode so one
policy covers camera-on/off. It is a ~17% mass increase at the very top:
zero-shot on the no-payload 7.4 V walker costs survival (16/16 -> 7/12);
payload DR in the hardening stage recovers it (see table).

**2 m dash (`dash=True`):** success = torso x >= 2.0 m **and still upright 1.0 s
after crossing** (a dive across the line never finishes); time-to-2m recorded at
the crossing; truncation still 10 s. Reward = the same dense shaped gait reward
+ per-step time penalty (`--w-time 0.1`) + finish bonus (`--finish-bonus 25`)
on a *confirmed* finish only; forward term stays capped (`--target-speed`
0.8 @ 7.4 V / 1.0 @ 11.1 V) so the lunge exploit stays dead.
`eval_policy`/`compare_runs --dash` report confirmed-finish rate and median/
best time-to-2m.

**Training curriculum (all on v2 + sts3215):** base gait (warm-start at 11.1 V;
**from scratch at 7.4 V** after the warm-start stalled) -> dash fine-tune
(+time penalty/finish bonus, lr 1e-4) -> hardening (model-error DR + servo-param
DR + camera-payload DR 0..170 g + mixed 0-8 mm terrain with 25% flat workers,
lr 5e-5). Dash results (16 episodes, deterministic; "finishes" = crossed 2 m
and stood 1 s; `compare_runs.py --dash`):

| policy | V | finishes (flat) | med t2m | speed | cam 154 g | 10 mm terrain | cam+terrain |
|---|---|---|---|---|---|---|---|
| v2_scratch_7v4 (base walk) | 7.4 | 16/16 | 4.52 s | 0.45 | 7/12 surv | — | — |
| dash_7v4 | 7.4 | 16/16 | 4.26 s | 0.47 | — | — | — |
| **dash_7v4_hard** | 7.4 | **16/16** | **4.36 s** | 0.46 | 15/16 | 16/16 | 12/16 |
| v2_adapt_11v1 (base walk) | 11.1 | 14/16 | 3.22 s | 0.63 | — | — | — |
| dash_11v1 | 11.1 | 16/16 | 2.62 s | 0.81 | — | — | — |
| **dash_11v1_hard** | 11.1 | **16/16** | **2.72 s** | 0.78 | **16/16 (2.65 s)** | **16/16 (2.94 s)** | **16/16 (2.89 s)** |

Both hardened policies also hold **16/16 under model-error DR** (mass/friction
±, servo kp/kd ±20%, stall/no-load ±15%; no shoves). Gifs: `runs/dash_7v4_hard/
dash.gif` and `runs/dash_11v1_hard/dash.gif` (camera on), plus `walk.gif` in
`v2_scratch_7v4` / `v2_adapt_11v1` for the base gaits. The 7.4 V gait is
visibly more upright/grounded (2% double-support, 0.28 s swing) than the
11.1 V jog (0% double-support, 0.23 s) — the envelope regularizes it.

**Recommended policy: `dash_11v1_hard`** if the electronics move to a 3S pack —
2 m in ~2.7 s, camera on or off, 10 mm terrain, perfect in every eval cell.
On the current 2S design, **`dash_7v4_hard`** — 2 m in ~4.4 s and still 16/16
on terrain, but the camera+terrain combination drops to 12/16: at 7.4 V the
envelope leaves little margin, so treat 2S+camera+rough ground as the design's
edge. (Speed-only alternative: `dash_11v1` for a clean-floor demo.)

**Open questions:** (1) *Action latency* — resolved by sub-step latency
modeling, see the next section. (2) Repeated shoves remain unsolved
(unchanged). (3) The 11.1 V gait still has a flight phase; the 7.4 V gait is
the more sim-to-real-plausible one to try first on hardware.

### Sub-step action latency: the cliff, located and hardened (2026-07-11)

The first latency result above ("0/16 with one 20 ms control step of delay",
and a failed whole-step latency fine-tune `dash_11v1_lat1`) turned out to be
doubly pessimistic. First, the coarse `action_latency` knob has a pre-existing
off-by-one — its delay buffer is initialized with `latency+1` entries and
appends before popping, so **`action_latency=1` is really a 40 ms (two-step)
delay** (kept as-is for old-run reproducibility; use `latency_ms` instead).
Second, real latency on this robot (1 Mbps serial bus + 50 Hz loop) is ~2-5 ms,
far below one control step.

**Env:** new `latency_ms` (float, 0-20): the first `round(latency_ms / 2 ms)`
physics substeps of each control step run with the *previous* target still
applied, then the new one takes over. Verified: default 0 is bit-exact with
old behavior; `latency_ms=20` is physics-identical to an exact one-step delay.
`latency_ms_max` draws per-episode latency uniform(latency_ms, max) — built-in
latency DR; `latency_jitter_ms` adds per-control-step +/- jitter (bus timing
isn't constant). Composes with `action_latency` for delays > 20 ms. Train
flags `--latency-ms/--latency-ms-max/--latency-jitter-ms`; eval override
`--latency-ms` in eval_policy/compare_runs.

**Zero-shot latency sweep** (16 eps, confirmed finishes / median t2m, flat,
no payload):

| latency | dash_11v1_hard | dash_7v4_hard |
|---|---|---|
| 0 ms | 16/16, 2.70 s | 16/16, 4.36 s |
| 2 ms | 16/16, 2.70 s | 16/16, 4.40 s |
| 4 ms | 16/16, 2.74 s | 16/16, 4.42 s |
| 6 ms | 11/16, 2.84 s | 16/16, 4.45 s |
| 10 ms | 0/16 | 9/16, 4.68 s |
| 14 ms | 0/16 | 4/16, 4.89 s |
| 20 ms | 0/16 | 0/16 |

So the fragility was largely an artifact of the coarse, doubled step delay: at
realistic 2-5 ms both policies are essentially unaffected. The slower 7.4 V
gait tolerates ~2x the delay of the 11.1 V jog, as expected. The 11.1 V cliff
(6-10 ms) still sat uncomfortably close to reality, so both policies got a
latency-DR fine-tune (per-episode uniform 0-10 ms + 1 ms per-step jitter, full
payload/terrain/model-error DR stack retained, lr 5e-5, 6M steps) ->
`dash_11v1_hardlat` / `dash_7v4_hardlat`. This time the warm start *worked* —
unlike the whole-step attempt, 0-10 ms of smooth sub-step delay is close
enough to the parent's regime to adapt to rather than restructure around.
Same sweep on the hardened pair (16 eps; these and all numbers below are on
the updated 3S-inertia model):

| latency | dash_11v1_hardlat | dash_7v4_hardlat |
|---|---|---|
| 0 ms | 16/16, 2.92 s | 16/16, 4.43 s |
| 4 ms | 16/16, 2.84 s | 16/16, 4.38 s |
| 6 ms | 16/16, 2.87 s | 16/16, 4.43 s |
| 10 ms | 10/16, 2.95 s | 13/16, 4.54 s |
| 14 ms | 0/16 | 9/16, 4.90 s |
| 20 ms | 0/16 | 1/16, 5.26 s |

The cliff moved out by roughly a control-tick's worth at both voltages
(11.1 V: failing at 10 ms -> failing at 14; 7.4 V: marginal at 10 ms -> still
walking at 14), for a small flat-out speed tax vs the parents (2.92 s vs
2.70 s at zero latency). Headline check at the realistic operating point,
GoPro mounted, 4 ms latency, 32 episodes: **dash_11v1_hardlat 30/32, median
2.72 s; dash_7v4_hardlat 32/32, median 4.50 s.**

**Verdict for hardware:** at the expected 2-5 ms bus latency every current
policy works; ship the `hardlat` pair for ~2x margin. The residual exposure
is a control path slower than ~10 ms — which is a firmware decision, not a
training one: run the 50 Hz loop on the ESP32 next to the 1 Mbaud bus rather
than round-tripping through USB/WiFi (docs/wiring.md quantifies why: an
8-servo command + readback cycle at 115200 baud eats most of a 20 ms tick on
its own). No parts change either way.

### CAD meshes in the sim: real shapes, real contacts (2026-07-11)

The v2 model looked nothing like the CAD (segment boxes), and — more
importantly — only the sole boxes could touch the ground: a fallen robot's
body passed through the floor, and shin-fork/knee strikes on bumps were
non-events. Now the printed-part STLs (`cad/stl/`, via `meshdir`) are geoms
on every body: they render as the real robot and their convex hulls collide
with the floor (`contype 2` vs floor `conaffinity 3`), so falls and part
strikes are physical. Deliberately unchanged: foot-ground contact still goes
through the flat sole box (it exactly matches the real TPU pad; the foot mesh
is visual-only to avoid doubling that contact), and parts still don't
self-collide. Legacy boxes moved to render-group 3 (hidden). The payload box
is now GoPro-dark and floor-collidable. Cost: ~10 % flat, ~15 % on terrain
(hull-vs-hfield). Verification: standing contact set is exactly
{L_sole, R_sole}, and the gauntlet is statistically unchanged on the new
physics (dash_11v1_hard 16/16 @ 2.68 s GoPro; dash_11v1_hardlat 32/32 @
2.72 s and dash_7v4_hardlat 31/32 @ 4.52 s at GoPro + 4 ms) — the hulls
don't graze in nominal gaits, which is itself a useful clearance check.
All dash videos re-rendered with the real geometry.

### Sensing gap: the policy observes things the robot can't measure (2026-07-11)

Question from hardware review: "does the design include accelerometers?" It
didn't — and the audit of `_obs()` against the parts list found a real gap.
The 36-value observation vector maps to hardware as: 16 joint angles/vels →
ST3215 magnetic encoders ✅; torso up-vector + angular velocity → **needs an
IMU, which was not in the BOM** (now added: BNO085, fusion on-chip, on the
ESP32's I2C bus); torso linear velocity + terrain-relative height → **no
direct sensor exists**; prev-action + phase → software.

Zero-shot observation ablation (mask = freeze at the training mean;
dash_11v1_hardlat, GoPro + 4 ms, 16 eps):

| masked | result |
|---|---|
| nothing (baseline) | 14/16, 2.78 s |
| linear velocity (3) | 0/16 |
| height (1) | 0/16 |
| linvel + height ("IMU-only" build) | 0/16 |
| up + gyro + linvel + height (no IMU) | 0/16 |

Two conclusions. (1) The IMU is *necessary* — attitude feedback is
non-negotiable. (2) It is *not sufficient*: the current policies are brittle
to losing even the unmeasurable channels, so **the next training round must
use a hardware-realizable observation set** — encoders + IMU (+ prev action
+ phase), with linear velocity and height either dropped or replaced by an
on-ESP32 estimate — plus IMU realism (noise/bias/mounting DR). Caveat: this
masking is a lower bound, not a verdict on feasibility — policies *trained*
without an input usually learn to compensate; these were trained with it and
never had to. Same playbook as actuators and latency: model it honestly,
then train inside it. Tool: `sim/ablate_obs.py`.

### The stop-and-stand objective: three exploits and an open problem (2026-07-12)

Review of the dash video showed the "finish" was a jog-through — the criterion
(upright 1 s past the line) never asked for a stop. New `dash_stop` mode:
success = planar speed < 0.15 m/s continuously for 1 s past the line, upright
throughout; post-cross the reward flips from "go" to "brake and stand". Four
runs overnight, all at 11.1 V on the honest-actuator model with
hardware-realizable observations (`imu_obs`):

1. **Fine-tune from dash_11v1_hardlat** (`dash_11v1_stop`, 6M): kept sprinting
   through the line, 0/16. Third confirmation that behavior restructuring
   needs from-scratch training.
2. **From scratch, attempt 1** (`dash_11v1_real`, 10M): *loiter exploit* —
   post-cross, near-standing farmed ~1–2.5/step for the remaining ~7 s
   (~470 total) vs +25-and-episode-ends for finishing. Crossed slowly, then
   walked circles to truncation, 0/16. Fix: sharp brake ramp (credit only
   near standstill) + `w_time_stop` 1.5/step post-cross penalty, making
   every non-finishing step net-negative.
3. **Attempt 2** (`dash_11v1_real2`, 10M): *sprint-and-dive exploit* —
   braking from a 0.8 m/s sprint was never successfully explored (every
   attempt falls, costing the same as diving), so it dove at the line;
   0/16, all post-cross falls at ~2.7 s. Fix: approach taper (speed cap
   ramps down over the last 0.7 m, floor 0.3×) + 3× fall cost past the line.
4. **Attempt 3** (`dash_11v1_real3`, 10M): the shaping worked as intended —
   slow approach (0.58 m/s avg), crosses right at the line — and it *still*
   falls trying to halt: 0/16, episodes end ~1 s past the line.

Post-mortem: halting a dynamically-balancing biped is a capture-step skill
this curriculum never teaches, and there is a confound — all attempts also
ran with `imu_obs`, i.e. **no torso linear-velocity feedback**, exactly the
channel the ablation showed the policies lean on hardest. Braking blind to
your own speed is plausibly the real blocker. Next steps (in order of
promise): (a) isolate the confound — train stop with FULL observations; if
it works, the answer is a velocity estimate on the ESP32 (integrate IMU +
kinematics) or a recurrent policy, not more reward shaping; (b) reference-
state initialization (start episodes mid-walk near the line so braking is
densely explored); (c) a stand-and-balance curriculum stage before walking.
The overnight run `dash_11v1_imu` tests the other half of the decomposition:
the proven jog-through dash objective on realizable observations.

### Hardware-realizable observations: the dash transfers (2026-07-12)

The decomposition test after the stop campaign: train the *proven* dash
objective from scratch on `imu_obs` — 8 joint angles + 8 joint velocities
(servo encoders), noisy/biased up-vector + gyro (BNO085 model), previous
action, phase clock; **torso linear velocity and height zeroed** (no sensor
exists). This is the first policy the physical robot could actually execute.

- **`dash_11v1_imu`** (flat, 10M from scratch): **16/16, median 2.28 s** —
  faster than the full-state champion (2.67 s). Losing the unmeasurable
  channels does not block locomotion; the earlier ablation collapse was
  overfitting to inputs the policy happened to have, not true dependence.
- **`dash_11v1_imu_hard`** (+ full DR: terrain, payload 0–170 g, model error,
  0–10 ms latency, IMU noise; 8M warm-started): mixed gauntlet —

| condition | result |
|---|---|
| terrain + payload DR (training dist.) | 15/16, 2.46 s |
| GoPro fixed 154 g | **8/16**, 2.39 s |
| GoPro + 4 ms latency | **9/16**, 2.36 s |
| 6 ms latency | 15/16, 2.48 s |
| 10 ms latency | 10/16, 2.59 s |

Terrain and latency robustness carried over, but the **max-payload column is
the realizable policy's weak spot** (~8–9/16 vs the full-state policy's
16/16): without linear-velocity feedback, the tail of the payload range is
hard. The two candidate fixes are the same as for stopping — a velocity
estimate computed on the ESP32 (IMU integration + leg kinematics, fed as an
obs the sim models with estimator error), or more payload-focused training
(fixed-154 g fine-tune / wider DR). Either way this run moves sim-to-real
from "impossible obs" to "one weak column."

### Command-conditioned locomotion: stop solved, turn partial (2026-07-12)

Directive: the robot must do more than run straight — it must stop and turn.
The dash objective was retired for `command_mode`: the policy observes a
commanded (forward velocity, yaw rate) — resampled every 2.5–4.5 s within
episodes, 30 % stand commands, 15 % pivot-in-place — and is rewarded through
exp tracking kernels (legged_gym-style). This removes the terminal-finish
structure that all three dash_stop exploits fed on: stopping is practiced
continuously from the first minute of training. All runs on the
hardware-realizable observation set (obs 38 = encoders + noisy IMU + the
2 command channels). `set_command()` is the hook for scenario evals and the
future goal-seeking/teleop layers.

Iteration log (each committed):
1. *Eval harness bug*: raw commands written into VecNormalize-normalized
   obs — a stand command read as "walk at the mean speed". The first "0/8
   everywhere" was the harness, not the policy.
2. *Yaw bias*: the identical-parts legs (every horn +Y) walk with a free
   left yaw (+0.6 rad/s); at w_track_w=1 the policy paid the tracking tax
   and kept it. Rebalanced to 2.0, kernel σ 0.5→0.4.
3. *Missing pivots*: the command sampler never issued (v=0, w≠0); with no
   hip-yaw joint, pivoting is this morphology's easiest turn. Now 15 % of
   commands.

**Capability matrix — `cmd_11v1c` (flat, from scratch, 14M):**

| scenario | result |
|---|---|
| stand 8 s on command | **8/8, 4 cm drift** (also 8/8 hardened w/ GoPro + 4 ms latency) |
| stop from walking | works at moderate speed; the 0.8 m/s scenario falls *pre-line* (sustained near-max commands are the fragility, not braking) |
| pivot in place | both signs correct: +36°/−18° per 3 s commanded ±86° (≈0.21 / 0.10 rad/s authority — left cheap, right fights the gait bias) |
| turn while walking | left only (+38°); right ≈ 0 |
| velocity tracking | clean at ≤0.6 m/s (probe: cmd 0.6 → 0.55) |

**The stop problem is solved** — as a *skill*: zero-command standing is
rock-solid from any start, including under full DR. What failed overnight
as a terminal objective fell out of the command formulation in one training
run.

**Honest regressions/opens:** (1) the DR hardening stage
(`cmd_11v1c_hard`) collapsed walking (1/16 survive 10 s, 67 % double
support) while making standing bulletproof — under DR, standing is the
safest reward source and the policy retreated to it; command-mode hardening
needs a curriculum (terrain/payload ramp) or DR-scaled tracking weights.
(2) Right-turn authority is ~half of left — morphological; options are a
mirrored horn on one leg (mechanical) or letting the future goal-seeking
planner cost turns asymmetrically. (3) Sustained near-max speed commands
topple the policy; either train longer holds or cap the command envelope in
deployment. Videos: dash_stop/turn/pivot .gif/.mov in both run dirs.

### External review response (2026-07-12, GitHub issues #2–#13)

An external review filed 12 issues. Dispositions:

- **#2 battery mass (P0, fixed):** sim/CAD used 78 g vs the actual 74 g Zeee
  pack. Corrected everywhere (rollup, inertia script, XML — torso is now
  319.5 g); the pigtail lives in the wiring bucket, stated explicitly.
- **#3 sole ≠ TPU pad (P0, fixed):** the contact box was the full 96 × 52
  sole; the real contact is the 90 × 46 pad. Sole boxes resized to the pad
  (−25 % patch area); recommended policies re-verified on the corrected
  contact (numbers below).
- **#4 1-DOF sweeps (P0, fixed):** check_assembly gained 16 multi-axis
  worst-case checks — knee −95/−60 × ankle ±40/0 against the thigh link and
  ankle servo, hip ±60 × knee −95/−60 arms-vs-shin. All clear (46 checks
  total).
- **#7 TPU pad COM (P1, fixed):** the 8 g pad is now modeled where it sits
  (bottom of the sole) instead of lumped at the foot mesh COM.
- **#8 README vs DESIGN recommendation (P1, fixed):** README now names the
  current v2 policies and marks terrain_v4 as pre-honest-actuator history.
- **#6/#13 actuator model gaps (P1/P2, partially fixed):** gear backlash is
  now modeled — a ±b/2 deadzone on the PD position error, `backlash_deg`
  (+ per-episode DR via `backlash_deg_max`; STS3215-class is 0.5–1.0°).
  Voltage sag is *implicitly* covered: the existing ±15 % stall/no-load DR
  spans an effective 9.4–12.8 V supply, wider than a 3S sag band. NOT yet
  modeled: thermal derating (torque fade over minutes) — flagged for the
  endurance round, it can't matter for a 3 s dash.
- **#5 idler boss (P1, documented):** correct observation — the Ø19 boss in
  the Ø25 recess is a locator, not a seat; alignment is the M3 pattern.
  cad/README now says so, with an assembly procedure note (snug at
  mechanical zero, check runout) and the upgrade path if the disc's molded
  threads wear.
- **#10 fall threshold (P2, parametrized):** `fall_height`/`fall_up_z` are
  now env params; defaults keep legacy semantics for reproducibility,
  tighten per-run for future training.
- **#9 operating-condition DR (P1, mostly covered/roadmap):** friction DR
  (±40 %) spans indoor surfaces; the torque DR spans battery sag (above);
  backlash DR added. Open: thermal, long-horizon battery fade.
- **#11 terrain-blind (P2, roadmap):** known gap since day 2 — the plan
  remains a height-scan observation once locomotion basics are closed.
- **#12 tower gussets / camera dynamics (P2, documented):** the stepped
  gussets print inverted with ≤3 mm overhang per step (within printer
  guidance; verify on the first tower print). Dynamic camera loads are
  bounded by the fall case, not the gait (~1–2 g deceleration ≈ 0.3 N·m at
  the tower feet vs the 1.3 N·m design case already carried); the
  gopro_base remains the sacrificial fuse by design.

## 6. Design parameters (source of truth)

| Param | Value |
|---|---|
| Servo (STS3215) | 45.2 × 24.6 × 35.1 mm, 55 g, 2.94 N·m stall |
| Joint limits | hip-roll ±25°, hip-pitch ±60°, knee −95..+5°, ankle ±40° |
| Segment lengths | thigh ≈ 90 mm, shin ≈ 90 mm (servo + bracket) |
| Torso (D×W×H) | 46 × 104 × 72 mm; head 46 × 62 × 42 mm |
| Hip separation | 56 mm (leg center-to-center) |
| Standing height | torso center ≈ 0.28 m; overall ≈ 0.34 m |
| Total sim mass | ≈ 0.93 kg |

MJCF specifics: position actuators, kp=40, forcerange ±3 N·m; foot-floor friction 1.0;
internal parts set non-colliding (feet-only contact); IMU `site` on torso for orientation.

## 7. Roadmap

- **Stage 0 — CAD massing:** DONE (concept only).
- **Stage 1 — Physics validation (MuJoCo):** DONE.
- **Stage 2a — Gymnasium RL env (CPU MuJoCo):** DONE. `sim/walker_env.py` + smoke test.
- **Stage 2b — Train PPO:** CPU baseline DONE, then gait-quality shaping +
  procedural terrain (`terrain_v4`), then the **buildable-robot retrain** (see
  that section): CAD-true `bimo_biped_v2.xml` + STS3215 torque-speed actuator
  model + camera payload + 2 m dash. Recommended: **`dash_11v1_hard`** (3S,
  2 m in ~2.7 s, 16/16 in every cell incl. camera + 10 mm terrain) or
  **`dash_7v4_hard`** on the current 2S design (~4.4 s). `terrain_v4` and all
  ideal-actuator policies are physically unbuildable (they demand ~3 N·m at
  4.4 rad/s — outside the servo envelope at any voltage). **Open:** action-
  latency robustness (0/16 with 20 ms whole-step delay; needs sub-step latency
  modeling or a from-scratch latency curriculum), repeated shoves (unchanged).
  Next: port the env+reward to MJX on the RTX 4070 for scale/speed.
- **Stage 3 — Sim-to-real:** system-ID the real servos, export ONNX policy, deploy on RP2040/SBC.

## 8. TODO — known next steps (start here)

- [x] **Stage 2a — Build the Gymnasium env** around `sim/bimo_biped.xml`. DONE —
      `sim/walker_env.py`. Obs: joint qpos + qvel, torso up-vector, torso lin/ang
      vel, prev action, height, phase clock. Action: 8 residual target angles
      (action 0 == standing pose). Reward: forward vel + upright + alive − energy
      − action-rate. Terminate on height < 0.18 m or up-vector z < 0.4. Foot-
      contact obs still optional/TODO. Smoke test: zero policy stands upright 500
      steps; random/scripted fall (correct reward gradient).
- [ ] **Stage 2b — Port to MJX (JAX)** and train PPO (Brax PPO or RSL-RL) on the RTX 4070
      with a few thousand parallel envs. Target: a stable forward gait.
- [ ] **Domain randomization** (required for sim-to-real): friction, link mass ±15%,
      actuator latency, random shove forces.
- [x] **Accurate inertia from CAD meshes** — DONE for inertia: `bimo_biped_v2.xml`
      now has explicit per-body `<inertial>` from the STL meshes + component boxes
      (`sim/build_v2_inertia.py`). Collision/visual geoms are still boxes; mesh
      *contact* geometry remains open (soles are boxes — probably fine).
- [x] **Realistic actuator model** — DONE: `actuator_model="sts3215"` = PD torque
      clamped to the voltage-scaled STS3215 torque-speed envelope (see the
      buildable-robot retrain section). Backlash + serial-bus latency still open
      (env has an `action_latency` knob as a proxy).
- [ ] **Measure & set real masses/inertias** once physical parts exist.
- [ ] **CAD → printable:** take ONE joint (e.g. the knee bracket joining two servos) from
      massing block to a real printable STEP/STL with horn spline, heat-set bosses, and
      clearances; test-print to dial in tolerance (~0.3–0.4 mm on moving fits).
- [ ] **Verify joint axis directions/signs** against a real assembly before trusting any gait.
- [ ] **Sim-to-real plan:** system-identification protocol for the STS3215 (torque, speed,
      latency), ONNX export, and deploy pipeline to the RP2040 + SBC.

### Known limitations / caveats
- Massing CAD ≠ printable geometry.
- v1 (`bimo_biped.xml`) inertias are approximate boxes; v2 has mesh-derived
  inertia but still box *contact* geometry.
- No self-collision between internal parts (feet-only contact) — revisit when adding meshes.
- v1 actuator is idealized; the sts3215 model adds the torque-speed envelope but
  not backlash or serial-bus latency (use `action_latency` as a proxy).

## 9. References

- Bimo Project — `github.com/mekion/the-bimo-project`, `mekion.com/project`
- STS3215 dimensions — servodatabase.com / waveshare.com (ST3215)
- Simulator choice — roboticscenter.ai (MuJoCo vs Isaac Sim, Best RL sims 2026)
