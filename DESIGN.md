# Bimo-like Biped — Design & Simulation Working Doc

**Status:** CAD massing model + MuJoCo physics validation complete (Stage 0–1). CPU RL baseline done incl. gait-quality shaping + uneven-terrain curriculum (Stage 2b). Printable part set done (build123d, `cad/`, interference-checked). **Buildable-robot retrain done:** CAD-true model `sim/bimo_biped_v2.xml` (mesh inertia) + honest STS3215 torque-speed actuator model + camera payload + 2 m dash — recommended policies `dash_11v1_hard` (3S, 2 m in ~2.7 s) / `dash_7v4_hard` (2S, ~4.4 s). Nothing printed yet; sim-to-real not started.
**Last updated:** 2026-07-11
**Owner:** Tom

---

## 0. Working process — commit to main, no PRs

**This repo does not use pull requests for physical parts or documentation.** CAD, STLs,
print list, BOM, assembly/design docs and reports go **straight to `main`** — no branch,
no PR, no review queue.

The real review gate here is physical — a part that prints and fits, a bench measurement —
not a diff read on GitHub. Tom is the only reviewer, so a PR only delays the file reaching
the checkout he slices and builds from.

What replaces review is the repo's own gates. Run them *before* committing, and report what
they said:

| gate | covers |
|---|---|
| `sh cad/run_checks.sh` | both interference gates below, one exit code |
| `python cad/check_assembly.py` | interference / clearance at pose EXTREMES |
| `python cad/freecad_rom_collide.py` (via freecadcmd) | interference across the SWEPT ROM |
| `python cad/check_printability.py` | bridges, ceilings, first-layer contact, per part |
| `cd firmware/host && make test` | firmware logic under ASan/UBSan |
| `python -m pytest tests/` | sim + link protocol |

Keep local `main` in sync with `origin/main` — fetch and merge before working, push after.
Regenerated artifacts (STLs) are rebuilt from source with `python cad/parts.py` rather than
merged as binaries.

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
  *(Superseded 2026-07-15 — see "Battery bay widened" below: torso 327.8 g.)*
- **#3 sole ≠ TPU pad (P0, fixed — and it mattered):** the contact box was
  the full 96 × 52 sole; the real contact is the 90 × 46 pad. Sole boxes
  resized (−25 % patch area). Re-verification: `dash_11v1_hard` holds
  (15/16 @ 2.66 s GoPro) but **`dash_11v1_hardlat` collapsed to 2/16** —
  its latency robustness was partly leaning on the oversized patch. Repair
  fine-tunes on the corrected physics (which also add 0.3–1.0° backlash DR)
  recover it: **`dash_11v1_hardlat3`** is the new recommended benchmark —
  GoPro + 4 ms rebuilt from 2/16 → **28/32 @ 2.72 s** (11M repair steps
  total), flat 16/16 at 0 and 6 ms. The old 30/32 was earned on easier
  physics; these are the honest numbers now.
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

### Smoothness + power round: the honest wall (2026-07-13)

Video review caught what task metrics can't: the gaits are visibly shaky
(quantified: the benchmark dash policy runs at **1.9 rad/s RMS** torso
roll+pitch) — and the new electrical power model (P = Σ max(τω,0) +
3.75·τ², calibrated to the ST3215 stall point; every eval now prints watts
and pack runtime) priced it: **36 W ≈ 15 min on the 3S pack**. A training
round targeted both, with wobble and watts as acceptance criteria.

| run | recipe | result |
|---|---|---|
| dash_11v1_hardlat3 (baseline) | dash objective | walks 16/16, wobble 1.90, **36.0 W** (~15 min) |
| cmd_11v2 | 5× smoothness + power 0.015 | **froze**: stand 8/8 @ 1 cm, wobble 1.19, **6.3 W (~64 min)** — but 1/16 walking |
| cmd_11v2c | softened (0.15/0.008) | 2/16 walking, wobble 1.54, **54.7 W** — a struggling policy burns *more* |
| cmd_11v3a | walk-biased command mix (stand 10 %) | most walking engagement (0.36 m/s, 39 % double-support) but 0/16 survival, 43.9 W |

Findings. (1) **The power term works and the plant permits efficiency**:
6.3 W standing proves 30–60 min runtimes are physically available; the
τ²-copper-loss term correctly punishes the stiff trembling stance. (2)
**Efficiency follows competence, not penalties**: the policies that fight
for balance burn 44–55 W regardless of the penalty. (3) **Command-
conditioned *walking* on the honest plant is past what 12–14M CPU PPO
steps deliver.** Five stage-S attempts across two rounds, spanning
penalty scales and command mixes, all produce excellent standing and
fragile walking — while the single-behavior dash lineage walks 16/16 on
the same plant. Conditional skill families are simply a harder learning
problem than one behavior.

Paths forward, in order of expected value (none executed — this round is
closed): **(a) per-skill experts + distillation** — working experts
already exist for run (dash lineage) and stand (cmd_11v2); train a
turn expert, then distill into one command-conditioned policy instead of
learning all skills jointly from scratch. **(b) MJX/GPU scale** — the
roadmap's original plan; 100M+ steps with bigger nets is the standard
recipe for command-conditioned locomotion and CPU PPO is 10–50× short.
**(c) Mechanical**: a mirrored-horn leg to remove the yaw bias that taxes
every turn. **Hardware takeaway now**: budget ~10–15 min of active runtime
per pack at current gait quality (hot-swap bay + 2-pack already covers
this); efficient gaits can triple it later.

### Distillation round: killed by the walk expert; kernel bug found (2026-07-13)

Plan: train single-behavior experts (walk / stand / pivot-L / pivot-R,
command channels pinned via `cmd_fixed`) and DAgger-distill them into one
command-conditioned student (`sim/distill.py`, ready and committed). The
stand expert already existed (cmd_11v2). The chain died at step one, twice
— and the first failure surfaced a genuine bug:

- **Kernel-width bug (found, fixed, insufficient):** the tracking kernels
  were `exp(-(err/0.25)²)` — HALF the legged_gym reference width
  `exp(-err²/0.25)`. From standstill with a 0.6 m/s command the narrow
  kernel pays 0.003 for attempting to walk (reference: 0.24) while standing
  collects ~1.1/step — a reward landscape that predicts exactly the five
  observed "stands great, won't walk" results. Fixed (σ = 0.5 both kernels).
- **exp_walk (σ 0.25): 0/16, falls at 1.2 s. exp_walk2 (σ 0.5): 0/16,
  falls at 0.9 s.** The kernel bug was real but not sufficient. Failure
  mode (from rendered frames): an aggressive lunge-like first stride that
  pitches into collapse — it never finds a rhythm.

Differential vs the dash lineage (which walks 16/16 from scratch on the
same plant): the dash reward is *dense linear* progress (`w_forward ·
min(v, cap)` — any forward motion pays immediately) with time pressure;
and the dash from-scratch runs trained **without** the power penalty and
**without** backlash. The untested variables are therefore (a) the
penalty stack (power + backlash + smoothness) during from-scratch gait
discovery, and (b) tracking-kernel vs dense-progress reward shape. One
25-minute ablation — `exp_walk3` with the penalties stripped — would
assign blame cleanly; not run (session training budget spent).

Distillation remains the right architecture *once a walk expert exists*
(stand + pivot alone are insufficient to distill). Status: paused at a
clean decision point, not abandoned.

**Reframing discovery (2026-07-13 evening): NO policy has ever walked
10 seconds.** exp_walk4 (dense reward, penalties stripped) also failed
(0/16 @ 1.7 s) — which prompted testing the champions with the finish
line removed: `dash_11v1_imu` falls at a median **3.7 s**,
`dash_11v1_hardlat3` at **5.1 s**. Dash episodes terminate ~1 s after
the 2 m line, so *sustained* walking was never in any curriculum — the
dash masked it, and every exp_walk "failure" was judged against a bar
nothing meets. The four ablations (kernel width, penalty stack, reward
shape) were all fighting the wrong variable: the missing ingredient is
**horizon**. Next experiment (running): `exp_walk5` — fine-tune the best
gait (hardlat3, already 5.1 s) with dash OFF, 10 s episodes, walking-pace
target (0.7 m/s), full DR. Horizon extension of an existing behavior is
adaptation, the kind of warm start that has worked every time here.
Hardware implication if it holds: v1 demos are 2 m bursts until sustained
walking lands; if exp_walk5 sustains, it becomes both the walk teacher
for distillation and the first continuous-walking policy.

**Verdict (2026-07-13 night): sustained walking is the confirmed wall at
CPU scale.** Both horizon fine-tunes *degraded* the parent —
exp_walk5 (slower target, two variables changed) fell at 2.3 s and
exp_walk6 (horizon-only, single variable) at 1.8 s, versus the parent's
5.1 s — the dash-calibrated value function (short episodes + finish
bonus) produces wrong advantages under the new objective and corrupts
the gait before adapting. Day totals: 5 from-scratch variants (kernel
width ×2, penalties on/off, dense reward) and 3 fine-tune variants, all
short of 10 s. What stands: **2 m dash bursts are solid and validated**
(the hardware-v1 demo), stand-on-command is solid, pivots are real but
weak. Recommended next moves, in order: **(1) MJX/GPU port** — sustained
command-conditioned locomotion at 100M+ steps is the standard recipe and
every signal says scale is the binding constraint; **(2) bigger feet in
CAD** — the pad-true contact is 90 × 46 mm under a 1 kg, 32 cm-CG robot;
a modest sole enlargement is cheap static-margin insurance and one
parameter in `dimensions.py`; **(3) fresh-value-head fine-tuning**
(reset the critic when the objective changes) if more CPU rounds are
attempted. The day's instrumentation (wobble RMS, electrical watts,
runtime) and the kernel/penalty/reward ablation record all carry forward
regardless.

### Stage 2b begun: MJX port, parity-tested (2026-07-13, later that night)

Path (1) chosen. Infrastructure: **Mira** (`ssh mira`, RTX 4070 Ti 12 GB,
32 cores) benchmarks at **639k physics steps/s** at batch 4096 on the real
v2 model — ~200× the CPU rig, a 100M-step run in ~1–2 h. (A RunPod account
is set up and funded as burst capacity; secure-cloud A5000 at $0.27/hr
worked, community 4090s were out of stock. All pods deleted after testing.)

`sim/mjx/env_mjx.py` ports the command-conditioned env to pure JAX:
per-substep PD + torque-speed envelope + backlash deadzone + sub-step
latency, imu_obs, tracking kernels (σ=0.5) with the cmd_dense option, all
penalty/shaping terms, per-episode DR for servo gains / latency / backlash /
IMU error / pushes, batch-level (per-parallel-env) DR for mass / friction /
payload. Terrain and the dash objective are deliberately not ported.

`sim/mjx/parity_test.py` gates the port (all PASS, float64):
- **Airborne arithmetic** (latency 6 ms + backlash 0.5° active): worst
  |Δqpos| 8e-12 — actuator model, rewards, obs are exact.
- **Grounded stance** (matched contact manifolds): worst |Δqpos| 1e-11.
- **Known divergence, documented:** MJX's colliders prune contact points
  outside a 1 mm skin of the deepest penetration
  (`mjx/_src/collision_convex.py`); CPU MuJoCo keeps every penetrating
  corner. Manifolds differ at tilted-impact transients (66% match at gait
  amplitude, 100% in settled stance) — trajectories therefore diverge
  chaotically at impacts, like any tiny perturbation. Mitigation: DR plus
  the standing rule that **every MJX-trained policy is refereed by the CPU
  eval harness** before being believed.

### THE WALL IS DOWN: sustained walking, CPU-verified (2026-07-13 night)

**`mjx_cmd_v1`** — first GPU-scale run, and the first policy in this
project to walk 10 seconds. brax PPO on Mira, **150M steps in 2h18m**
(2048 parallel envs), the exp_walk reward shape, 10 s episodes, hardened
from step 0: imu_obs + IMU-error DR, latency 0–8 ms + 1 ms jitter,
backlash 0.5–1.0°, servo-gain DR, shoves, GoPro 154 g. Training curve:
mean episode length 36 → **465/500** steps, monotone, no plateau tricks.

**CPU referee** (the real gate — `eval_ref.py`, CPU MuJoCo, matched
conditions, 8 seeds/scenario, 10 s episodes):

| scenario | survive | alive_s | vx_err | wobble RMS | watts |
|---|---|---|---|---|---|
| walk 0.6  | **7/8** | 10.0 | 0.12 | 0.73 (calm) | 19.4 |
| slow 0.35 | **8/8** | 10.0 | 0.06 | 0.58 | 14.5 |
| stand     | **8/8** | 10.0 | 0.01 | 0.06 | 5.2 |
| pivot L   | **8/8** | 10.0 | —    | 0.08 | 6.8 |
| pivot R   | **8/8** | 10.0 | —    | 0.08 | 2.0 |
| turn 0.4/+0.4 | **8/8** | 10.0 | 0.09 | 0.66 | 15.5 |

One policy does all six behaviors — no distillation needed. The
sim-to-sim gap (MJX contact manifolds → CPU) did not break transfer; DR
covered it, as designed. Day-5's diagnosis (training scale was the
binding constraint) is confirmed by construction: same reward family,
same plant, ~19× the steps, hard mode throughout, and the wall fell on
the first attempt. Remaining soft spots: 1/8 walk-seed fall, yaw-rate
tracking is loose (|wz| err ~0.4–0.5 — heading wanders; a heading-error
term or tighter yaw kernel is the next reward iteration), and pivot
authority should be quantified vs the CPU-era weakness. Cost of the run:
electricity.

### Get-up experiment: the robot sits up — and that's the ceiling (2026-07-14)

New scenario (user request): recover from a fall. `getup=True` in both
engines (parity block 2b): episodes start from settled ragdoll falls, no
fall termination, recovery reward = height + uprightness progress + a
standing bonus. MJX keeps the cad-mesh floor contacts in this mode — a
fallen robot rests on its hulls; the parity test caught that the stripped
model had nothing holding the torso off the floor.

**Result: negative, with a clean diagnosis.** `mjx_getup_v1` (RunPod A40,
stopped at 100M steps by the pre-agreed rule — reward plateaued ~310 for
30M+ steps, `standing` never left ~0): from any fall the policy reliably
reorganizes into a stable **sit** (torso ~50°, legs forward) within ~1 s,
then parks at 0.9 W. CPU referee: 0/8 recoveries, same behavior. It
climbed the reward exactly as far as the kinematics allow.

**Why it can't finish:** sit → stand requires moving the CoM from behind
the heels to over the feet. With no arms, hip pitch capped at ±60°
(torso can't fold over the knees), and 2.7 N·m servos (not enough to
rock-and-catch dynamically at this mass), the transfer has no
statically-stable path. A static sweep with the real model shows deep
crouches are stable at every hip angle once the feet are loaded (CoM
+35..66 mm past the heel) — the missing link is only the transfer.

**Options (decision pending):**
1. **Widen hip-pitch flexion** (60° → ~100–120°) — yoke/horn redesign in
   CAD; the human-style no-arms getup. Verify with a scripted-motion
   feasibility study BEFORE committing the CAD change: hand-author the
   sit→crouch motion at candidate ranges in sim, find the minimum.
2. **Accept no self-recovery for hardware v1** — falls get a human reset;
   revisit after v1 walks on real ground.
3. Exotic props (push off the GoPro tower, etc.) — not pursued; adds
   fragile contact-rich behavior on real hardware.

Costs: ~$2.40 of A40 time; the early-stop rule saved ~2.7 h / ~$1.20 of
a run that could not succeed. Infra: `infra/runpod/run_train.sh` is the
standing launcher (policy: anything longer than a few minutes runs on
RunPod, Mira is dev/smoke only).

### Hip-pitch study: the number is 95° — and two bugs fell out (2026-07-14)

`sim/getup_study.py` (scripted-motion + static-path analysis, decision:
keep hip-60 as backup tag `v1-hip60-backup`, figure out the hip):

1. **Action-mapping bug (material, affects everything):** the residual
   action mapping (`default + 0.5*(hi-lo)*a`) is a symmetric band that
   cannot reach the far side of asymmetric joint ranges — **the knee
   (−95..+5°) was capped at −50° for every policy ever trained.** Walking
   never needed the deep half; get-up does. Fixed as `action_map="full"`
   (piecewise-linear, action 0 = stand, reaches both limits; "legacy"
   stays default so old runs reproduce).
2. **Sign note:** hip *flexion* is NEGATIVE on this model; the RL sit
   parks at −59°, pressed against the −60° stop.
3. **The kinematic answer:** a connected quasi-static sit→stand path
   (CoM within ±20 mm of sole center, feet flat, real masses + GoPro)
   exists iff hip flexion ≥ **95°** and provably not at ≤90° (BFS over
   the feasible (knee,hip) grid). The path: pike-up — knees extend under
   the folded torso first, torso unfolds last. **CAD target: −110°/+60°**
   (95° + margin). `hip_flex_deg` env param widens the range in sim ahead
   of the yoke redesign.
4. Open-loop scripted execution reaches the loaded squat but topples
   during the rise (foot-skate / no closed-loop balance) — dynamic
   validation delegated to RL retrain (`mjx_getup_v2`: hip 110 + full
   mapping), which has IMU feedback.

### Get-up v2/v3 verdicts: kneel achieved; the rise is a balance corridor
(2026-07-14, evening)

**v2** (hip 110 + full action map, stopped at 63M by rule): the ceiling
moved — from any fall the policy rises to an **upright kneel** and parks
(v1's ceiling was a reclined sit). The referee's "0/8" is the standing
height bar (kneel pelvis ~0.10 m vs 0.24 required). Diagnosis: the
uprightness reward made the kneel absorbing — the pike to standing
requires folding the torso DOWN through up_z≈0 first (a reward valley on
the only feasible path).

**v3** (height-gated uprightness + reverse curriculum with kneel/squat
starts, stopped at 63M by rule): reward climbing but `standing` still ~0
— even squat-started episodes don't convert.

**Probes that close the file on physics scapegoats:** torque along the
pike path peaks at **1.08 N·m** of 2.72 available (not torque-bound);
scripted rise fails identically at 1×/2×/4× floor friction (not
friction-bound). The failure is always the same: tips backward mid-pike.
**Conclusion: the rise corridor is real (kinematics say ≥95° hip) but
NARROW — ±20 mm CoP on a 90 mm foot — and crossing it needs active
balance control plus, ideally, a wider corridor.**

**Decision point (user):** (a) proceed with the yoke redesign at
−110°/+60° (the 95° bound is necessary regardless of everything else)
AND enlarge the sole pads (already the day-5 recommendation; directly
widens the get-up corridor and helps walking robustness) then one more
RL round on the new plant; (b) resume/extend v3-style training longer
(cheap on Mira; reward was still climbing); (c) park get-up, ship v1
with manual reset. Recommendation: (a), with (b) essentially free in
parallel.
5. Test-harness bug worth remembering: `jp.asarray(numpy_buf)` can alias
   zero-copy on the CPU backend; the CPU env mutates `_prev_action` /
   `_air_time` in place → the parity sync silently read post-step values.
   Fixed with explicit copies.

### Foot v3 + the printability gate (2026-07-15)

The first printed foot failed twice — its pad-recess ceiling bridged 46 mm and
sagged, and a heel tab snapped off in handling — and the v2 rework (flat sole,
aft-gusseted tabs, `6080d12`) was judged **not viable**: the gusset lives in
the same thin 2.4 mm Y-plane as the tab, so it stiffens the blade fore-aft
while the actual failure was a *lateral* knock breaking the tab across its
horizontal layer lines. The blade was still a blade.

**Foot v3** keeps v2's flat sole (that part was right) and attacks the real
mode: a **heel bulkhead** joins the two tabs behind the servo case, closing
each free-standing blade into an L/U-channel section — stiff both ways, and
the layer-bond root is no longer the only load path. The bulkhead gets an
open-top **cable window** (16 mm, matching the pelvis deck cutout: ST3215
cables exit the rear end face), one full-width aft buttress replaces the two
per-tab wedges, and the retention bores are teardropped. Every added face is
vertical: nothing new bridges. Foot is ~36 g; ankle height unchanged;
`check_assembly.py` ALL CLEAR. Envelope note: tab thickness stays LOCKED at
2.4 (shin-fork passes 0.4 outside), so *sections*, not thickness, were the
only way to add strength.

**The systemic fix — `cad/check_printability.py`.** Nothing in the pipeline
verified that parts *print*: `check_assembly.py` proves they fit, and the
print list asserted "no supports; every part has a support-free orientation"
untested. The new gate loads each STL, rotates it into its actual print
orientation, and classifies every down-facing facet cluster (>45°) by a
perimeter ray test: CEILING (bridge — fails past 8 mm span), LEDGE (droops
past 1.2 mm reach), ISLAND (floating start — always fails), bore-top
(teardrop candidate), plus a first-layer-contact check. It found three
would-have-failed prints among parts marked "ready":

1. **pelvis** stood on its four Ø9 tower bosses — the entire 46×104 first
   layer floated 2 mm above the bed (and the bosses overlapped the tower feet
   tabs — a latent assembly bug). Bosses deleted; heat-set pilots go through
   the deck into the bay-cheek walls.
2. **tower**: the battery-window sill printed (inverted) as a 70 mm
   single-wall bridge; belt-guide ribs as square drooping ledges. Sill top is
   now a 45° ramp (outer 2.5 mm retention lip intact); ribs carry 45°
   chamfers on their print-undersides.
3. **leg_link**: the narrow fork slabs float 4.7 mm over the bed for 50 mm —
   and *can't* be extended down, that volume is swept by the foot walls and
   ankle-servo case at joint extremes (why `FORK_NARROW_X` exists). Solution:
   `leg_link_print.stl` carries three break-away fins (0.2 mm separation gap,
   2.4 mm interface width — a 0.8 mm first cut left the strip edges drooping
   >45°, caught by the audit) while `leg_link.stl` stays clean for sim meshes
   and assembly checks.

Also swept in: all horizontal M3 bores teardropped toward each part's
print-up direction (the `gopro_base` M5 lesson, applied everywhere), and a
real bug in `wedge_y` — it assumed `extrude()` runs −Y from `Plane.XZ` when
it actually runs +Y, so the v2 foot gussets had silently landed 2.4 mm
outside their tab bands. The direction is now measured, not assumed.

Post-script (same day): the user spotted a single-filament wall inside the
v3 cable window — a **boolean-order bug**: the full-width buttress was
unioned *after* the window cut, re-filling the window band with a wedge that
tapers to zero (the cut depth was irrelevant; any fix to it produced
byte-identical junk). Fixed by cutting the window last, clear through the
heel edge. The audit grew a **THIN check** to catch this class: an inward
ray from each facet measures local wall thickness, flagging blades under
0.85 mm (two perimeters) — but only where the exit face is near-parallel
(dot < −0.8), so 45° chamfer and gusset tips, which print fine, don't
false-flag (the tower sill did until that filter). Verified: the check
catches the bad foot, and the full part set is clean.

Post-script 2 (same day): the user then caught the teardrop roofs
**piercing part edges / leaving paper shells** — the full roof reaches
r·√2 above the bore center vs r for the round hole it replaced, and holes
placed with round-hole margins couldn't afford the extra 0.7 mm: leg_link's
+10.25 case screws poked through the grip-plate edge (12.65 vs 12.36), and
the peak-side bolt-circle hole grazed every Ø19 idler boss at 0.1 mm
(9.40 vs 9.50 — flakes, doesn't print). Fix: **capped teardrops** are now
the default — the 45° roof truncates at the round bore's own top
(center + r, a 0.83r ≈ 1.4 mm flat mini-bridge), so the void *never exceeds
the round-hole envelope*: no new pierce or sliver is geometrically possible,
while the bridged span still drops 3.4 → 1.4 mm. Full peaks only by explicit
`full=True` where clearance is proven (gopro_base M5, which stays
byte-identical). All flagged shells probed 100 % solid after the change;
both gates pass. (Why the THIN check missed the 0.1 mm boss shells: curved
shell over a 45° roof — exit faces aren't near-parallel, and the area is
under the area floor. The capped envelope kills the class by
construction, which is the better guarantee anyway.)

Post-script 3 (same day): next user catch — **hole-to-edge webs down to a
single filament**. The +10.25 case screws in leg_link's grip plates ended
0.41 mm from the plate front edge (edge = case half-width 12.36, holes fixed
by the servo's pattern — a v1-era sliver, screw heads overhung the edge
too), and the Ø14-BC bolt holes sat 0.8 / 0.3 mm from the Ø9 / Ø10
center-screw reliefs on every joint pad. Since the holes can't move, the
material did: grip plates widened to 13.2 (web 1.25, heads fully seated),
center reliefs shrunk to Ø8 (webs 1.3; still 1.1 mm slack over the ~Ø5.7
recessed center screw — verify on the real horn/idler). The THIN audit now
**grid-samples large facets** (barycentric points, area-weighted) instead of
centroid-only — a web beside a hole lives on a big face whose centroid is
far away, which is exactly how these evaded the first version. Confirmed it
flags all four webs pre-fix and nothing post-fix; both gates pass.
Separately verified all exported meshes are watertight/manifold (0 bad
edges) — the visible "seams" through holes are CSG face boundaries on a
continuous surface, not defects.

Sheet-metal alternative: evaluated in `docs/sheetcut-eval.md` (draft PR #19).
Verdict: printed v3 first (free, same-day); order bent 5052 L-brackets + an
aluminum sole (~$50–80, ~1 week) only if a v3 tab fails again or the
get-up-corridor sole enlargement is adopted — an Al sole is mass-neutral
against an enlarged PETG print and drops CoM ~6 mm.

### Get-up option (a) executed: hip −110°/+60° + enlarged soles (2026-07-15)

The decision point closed as (a). What the sim had been *simulating* since
the hip-pitch study (`hip_flex_deg=110`) is now what the CAD *builds* and
the XML defaults to.

**What actually limited the hip.** Not the servo, not the flange: a fine
CSG sweep (leg_link + servo mock vs yoke_pitch, 5° steps) shows first
contact at **±105°** — the thigh leg_link's idler-side grip plate and web
share the idler yoke arm's Y band (−20.35..−18 of −21..−18; the horn side
has no overlap — that's the 0.7 mm plate/prong gap), and the grip plate's
top-front corner (r = 20.75 mm off the axis) sweeps into the arm plate's
front edge. The XML's old ±60° was simply a conservative margin under the
unexamined 105.

**Fix (yoke_pitch idler arm only).** The thigh sweep across −115..+65 only
occupies angles ≤65° (front) and ≥166° (rear) at radii ≥16 mm, leaving the
top-rear sector dead. The idler arm is now a **Ø28 hub disc** (2 mm under
the r=16 swing floor) **+ riser plate** (x −14..+4) through that dead
sector: front edge clears flexion to ~129°, rear edge clears extension to
~97°. The hub's front-upper quadrant is cut to a **45° face** so the disc
prints support-free flange-down (its print-underside); bolt rims keep
≥1.3 mm past the cut. Horn arm unchanged. `check_assembly` now sweeps the
**full thigh assembly** (leg_link + servo, not just the servo) at
0/−60/−95/−105/−110/−115/+60/+65 vs yoke_pitch, plus deep-flexion checks
vs yoke_roll and the pelvis at roll 0/±25 — ALL CLEAR.

**Soles enlarged 100 → 116 mm, heel-biased** (heel 42 → 52, toe 58 → 64;
width stays 52 — the rise fails *backward* and lateral wasn't the failure
mode). Rise corridor grows from ±20 mm on the 90 mm pad to roughly ±28 mm
on the 106 mm pad, most of it behind the heels where the pike tips. Foot
segment 91 → 110.4 g (print ~42 g, pad 9.8 g). Pads are cut from 1/16"
self-adhesive silicone sheet (B0FJ8TBMQK; one 6"×6" sheet does both feet) —
`TPU_PROUD` 0.5 → 1.6, so stance rises 1.1 mm; propagated to the sim sole
boxes and heights.

**Propagated:** XML hip range −110/+60 (walker_env's `hip_flex_deg=110` is
now a no-op, older runs unaffected), sole contact boxes 106 × 46 at the
pad-true inset, CAD-true inertials rebaked for every body
(`build_v2_inertia.py`), smoke test stands 500/500. Both gates green; all
STL/STEP/assembly artifacts, fly-in, assembly-step figures, and the ROM
sweep video (`sim/renders/rom_sweep.mov` — the finale now shows the
hip-110 pike fold) regenerated. Per PR #19: the G10/FR4 0.125" sole-plate
order trigger ("if enlargement lands") is now live — user's call.

**Next:** print yoke_pitch ×2 + foot ×2 (v3.1) in PETG, cut the 106 × 46
silicone pads, and one more RL get-up round on the new plant (option (b)
in parallel — cheap on Mira). Note: the feet already printed at 100 mm are
fine for bench bring-up, but policies trained on this plant expect the
116 mm soles.

### leg_link v2 cleanup from first-print review (2026-07-15)

User's annotated review of the printed leg_link flagged three things on the
back web. (1) The four Ø4.5 web holes are the **zip-tie anchors** for the
servo cable raceway — functional, kept. (2) The idler-side silhouette had a
0.65 mm step at `FORK_WIDE_Z`: the web/grip plate ended at the natural
−20.35 (case face + PLATE) while the fork plate/jog block sit at −21. Web
and idler grip plate now run flush to −21 (same x-z swing footprint, so all
sweep clearances hold). (3) The 0.7 mm slot between the web edge (19.75)
and the horn fork plate (20.45) over z −50…−36.5 was dead air — the
chained link's grip-plate sweep that motivates the 0.7 band tops out at
z ≈ −72 — and is now solid (the wide fork section starts at the web edge).
Verified by point-in-solid probes; both gates ALL CLEAR / PRINT CLEAN;
+0.7 g per link (segment 77.9 → 78.6 g), inertials rebaked, smoke test
stands 500/500. The printed v1 link fits and works identically — no reprint
needed; v2 is the one to slice from now on.

### tower print revision from slice reviews (2026-07-16)

Slicing the (still unprinted) tower showed bridging and unsupported
overhangs the first printability pass had missed: the feet-tab gussets were
flat-topped boxes — printed upside down, a 6 mm ceiling floating 31 mm over
the interior — the battery-rail stubs only stepped 1.3 mm per ledge, and the
window-sill ramp measured 44°, one degree past the 45° threshold, so the
whole 70 mm sill band sliced as overhang. Gussets and rail stubs became true
≥45° wedges (the gusset docstring had claimed a wedge all along; it braces
the tab exactly as the box did), and the feet screws now seat directly on
the tabs through Ø6.6 head wells in the wedges (per `docs/assembly.md`:
4× M3 into the deck inserts, unchanged).

The sill itself went through three designs in one review cycle — steeper
ramp + plain break-away posts (posts missed: the lip's underside was a knife
edge whose first ~1 mm of layers are sub-nozzle slivers the slicer drops, so
the *printed* lip floated ~1 mm above any support placed under the *modeled*
surface), then 45°-fan support trees under a flat 1.2 mm lip band — before
the right question ended it: **the horizontal member isn't needed**. The
hook-loop belt was always the battery's tumble/dash retention; the sill only
parked the pack while the belt was off. The window is now open to the deck
and two 2.5 mm **corner detents** (45° wedges off the window posts,
self-supporting inverted) do the parking. The pack tilts in over them as it
did over the sill; seated fit is exact (0 mm³).

Net: `check_printability` CEILING count 13 → 4, and a full OrcaSlicer PETG
slice confirms the four Ø6.6 counterbore roofs are the *only*
bridge/overhang blocks in the g-code (≈5 h 19 m, ~17 min faster than with
the sill). No print-variant STL needed — slice `tower.stl` upside down.
Torso inertial rebaked (327.8 g, unchanged mass, 3rd-digit inertia shift);
stand test 500/500. The tower had not been printed yet — nothing to redo.
Cost of the open window: the −x wall is a U instead of a closed frame —
acceptable, the +x wall is intact and wall loads are small (the camera
moment goes through the feet tabs, whose gussets grew in this same pass).

### imu_carrier — screwed IMU mount, and the BNO085→BNO055 swap (2026-07-16)

Tape-mounting the IMU was rejected; the replacement is a printed
**`imu_carrier`** that sandwiches between the tower top and the gopro_base
on the **same four mounting bosses** (screws grow M3×8 → M3×12 for the 3 mm
plate) and cantilevers a tongue rearward with four M2.5 bosses the breakout
screws onto. Interior mounting stays impossible — every clear flat inside
the tower is <13 mm against a >20 mm board — but the carrier needs no tower
change, so the freshly printed tower stands.

The ordered IMU (Amazon B0GVK81HXR) turned out to be the **classic BNO055
breakout**, not the BOM's BNO085/STEMMA pick: outline 26.67 × 20.32, holes
**21.59 × 15.24** Ø2.5 (taken from Adafruit's own Eagle .brd, not eyeballed;
the STEMMA variant's 20.32 × 15.24 swap is noted in `dimensions.py`), solder
header instead of JST (4× F-F jumpers now, BOM item 21), I2C 0x28, same
on-chip fusion at 100 Hz so the control plan is unchanged. Carrier bosses
clear the feet-screw wells by construction; the stack is verified
zero-intersection, both gates pass (`thin-note`s are the intentional 0.4 mm
pilot floors), and the gopro stack + camera rose 3 mm everywhere in CAD
(assembly/dressed models, step figures). Honest omission: the carrier + IMU
+ gopro_base (~13 g of top-of-tower accessories) are still outside the sim
torso inertial, same bucket as the +154 g camera adjustment.

### leg_link v3 — cable window + removable fins (2026-07-16 print review)

Second slice/print review of the leg_link raised three things; all landed:

1. **Cable window.** The servo's rear ports sit inboard of the back web
   while the raceway runs down its outer face — the dressed model measured
   **27–74 mm³ of cable-through-plastic per segment** because there was no
   opening. The web now has a 9 × 11 window at (y 0, z −48…−37) — sized to
   pass a 3-pin plug — and the zip-tie holes moved to ±9 so the −40 pair
   straddles the exit. `dress.py` reroutes every joint-crossing segment
   through the windows (frame-following waypoints riding the outer face and
   wrapping the knee corner; slack bows along posed outward bisectors), and
   the cable∩link check now reads ≤1 mm³ (surface graze at the tie line)
   across neutral, knee −95, pike, hip ±extremes, and roll+knee.
2. **Break-away fins were unremovable — so there are none anymore.** The
   root cause (user diagnosis, confirmed by first-layer island analysis of
   the sliced g-code): the fins' bed footprints overlapped the web's
   footprint, so layer 1 printed breakaway and back plate as ONE continuous
   sheet — no break line existed anywhere, and every above-bed gap/tooth
   tweak was irrelevant. The real fix came from
   re-mapping the foot sweep at ankle ±45°: the swept volume in the slab
   bands is only below z −93 (heel passing under the axis) plus one idler-
   side lobe at exactly z −68.4…−55.6 (wall corner, ankle −33…−45). So the
   slabs are **solid to the web face** everywhere else — "make it solid",
   the feedback's own words: the horn slab is a full-depth blade
   (z −92…−50) and the idler side fills both sides of the lobe. The 16 mm
   idler stretch left over is anchored on solid at BOTH ends; a bare ribbon
   bridge was rejected in review, so two **island posts** (2.4 × 3, 0.35
   under the slab) break it into 2/2.5/5.5 mm hops. Every break-away piece
   — the two posts plus two 4 mm pad stubs under the pad tangents — is a
   verified SEPARATE first-layer island (≥1 mm clear at the bed, no brim):
   attached to nothing, they lift off with a fingernail. That island rule
   is the design principle this whole review converged on: a breakaway
   sharing first-layer plastic with the part has no break line, no matter
   what happens above the bed. (`check_printability` keeps the BEAM class
   for end-anchored ribbons ≤4 mm wide, allowed to 20 mm.) The boss fin's job is likewise
   geometry now: the boss OD is a ~51° cone (Ø19 → Ø16 tip),
   self-supporting — it's a loose locator (~3 mm radial slack), so the
   taper costs nothing. +0.9 g/link; the solid blades stiffen the fork.
3. **No support past the pad end** (feedback item 3) — the old fin tail to
   −101 supported nothing and printed as a floating pedestal.

Both gates pass; thigh/shin inertials rebaked (78.6 g, window −0.3 g
against the cone's +0.3 g). The window is functional, not cosmetic:
**printed v1/v2 links want a reprint** when convenient, else the leads
detour around the web's bottom edge.

### Precision curriculum round: infrastructure (2026-07-17)

Directive: "we've been trying to run before we walk" — a skill round focused
on *control*: one-leg balance (10 s each), air circles with a lifted foot,
a straight 1 m line, circle/square return-to-start, 0.5 m sidestep, 1 m
backward walk, crouches (both/each leg). Full spec + acceptance criteria:
[docs/precision-curriculum.md](docs/precision-curriculum.md). Everything
below landed and is parity/bit-exactness gated; training round 1 follows.

- **`ext_cmd` mode, both engines** (obs 38→43): 7-channel commands (vx incl.
  backward, vy, wz, crouch height fraction, one-leg lift ±1, swing-foot
  (dx,dz) target; per-episode air-circle generator drives (dx,dz) when the
  command mix picks it). 2-D velocity kernel, height kernel vs commanded
  crouch, lift contact-pattern + swing-foot-target kernels. Fall cost 10
  ("severely penalize falling"), fall line scales with commanded crouch.
  Legacy paths bit-exact vs HEAD; parity gates 2c (airborne ext arithmetic,
  8e-12) and 2d (grounded ext stance) added.
- **Plant truth — the sim now trains the robot AS PRINTED, GoPro on:**
  `bimo_biped_v2_asbuilt.xml` = printed 100 mm foot v3 (90×46 pad boxes,
  102.8 g segment, `foot_asbuilt.stl` from 191777e) because the 116 mm
  get-up soles exist only in CAD (user: "we're running with the 100×50
  feet"). Torso inertial rebaked WITH the tower-top accessory stack
  (imu_carrier 5.0 g + BNO055 3.0 g + gopro_base 5.2 g + fasteners 0.8 g →
  341.8 g; closes that section's honest omission). New `payload_cg_z`
  param: GoPro CG at the true +0.0945 m (battery-bay tower +11.5 mm +
  carrier +3 mm over the 0.08 spec); old runs keep 0.08. Robot 921 g bare.
- **Compute policy (supersedes "RunPod for everything"): Mira's 4070 when
  idle, RunPod otherwise.** `infra/train_launcher.sh` decides (util <20%,
  ≥10 GB free, AND no ollama model loaded — ollama on Mira serves the email
  agent and routinely holds VRAM); `infra/watch_train.sh` polls to
  completion, pulls params, tears down RunPod pods, runs the referee;
  `infra/precision_round.sh` chains it all: **one command per training
  round, one completion message** (also the token-lean iteration loop —
  per round the agent reads only `sim/runs/<out>/scorecard.{json,md}`).
- **Referee: `sim/mjx/eval_precision.py`** — scripted CPU-referee scenarios
  for all seven skills + smoothness panel (wobble RMS, watts, cost of
  transport, foot slip, L/R swing symmetry). Return-to-start scenarios use
  ground-truth odometry (sim-only; the hardware odometry gap stands).
- **Parity block-2 "regression": found, diagnosed, fixed (test bug, not
  physics).** Grounded stance failed at clean HEAD (|Δqpos| 1.3e-4 vs the
  1e-11 recorded 2026-07-13). Root cause chain: the silicone-pad sole drop
  (`b4f22bc`, z −0.0129 → −0.0140 — physically correct, kept) deepened the
  settling transient; at exactly one step MJX's 1 mm-skin collider keeps an
  L_sole corner grazing at −0.05 mm that CPU's box-plane collider omits, so
  that step's contact forces differ; the block's `matched_only` guard
  compared POST-step contact counts (which re-converge, 8==8) while the
  step's forces come from the PRE-step (synced) manifold — the guard never
  fired. Exonerated by probe table: the 116 mm sole widening (1.05e-11
  with HEAD code) and package drift (the 07-13 commit passes at 9.4e-12 on
  today's venv). Fix: screen on the synced pre-step manifold. Full suite
  now PASSES: grounded 1.04e-11 (75/100 steps gated in), airborne 8.5e-12,
  ext blocks identical.

### Precision rounds 1-2: the do-nothing optimum, then first skills (2026-07-17)

**Round 1 (`precision_v1`, 150M, RunPod 4090, ~$1.7):** a perfect
cautionary tale for "severely penalize falling" — the policy achieved ZERO
falls in 112 referee episodes by NEVER MOVING. Stand 8/8 (1.6 cm drift),
everything else 0/8: walk/sidestep/backward commands ignored at standing
watts, crouch ignored (hErr 85 mm), balance half-lifts (~50 % contact).
Economics: with sigma-0.5 kernels + the height kernel + upright/alive, a
stander under a 0.4 m/s command banked ~5.4/step ≈ 84 % of perfect
tracking, risk-free. (Archived: `runs/precision_v1_frozen`.)

**Round-2 fixes** (committed `7d4e659`): dense directional progress
(velocity projected on the commanded direction, capped at the command —
zero for standing, negative against it; the dash-lineage gradient), height
kernel gated to 0.3x under motion commands, and a REAL BUG found by the
new parity block 2e: MJX unpacked the yaw command from channel 1 (legacy
2-ch layout), so in ext mode the yaw kernel tracked the SIDESTEP channel —
round 1 never rewarded pivots at all. Also: eval `circle_return` now
requires >=0.3 m actual excursion (standing had "returned to start" 8/8).

**Round 2 (`precision_v2`, 300M, RunPod RTX 4000 Ada, ~$1.9):** motion is
back — **line_1m 4/8 at t=2.1 s** (0.48 m/s, clean stops), balance
lifted-contact 57→28-37 %, air-circle tracking err 66 mm, and it attempts
every skill (falls 30 % — it tries). Still failing: sidestep/backward
(0/8, never reach), crouch (ignores height, hErr 81 mm), return-to-start
(1.05-2.67 m off). Training was NOT converged (reward +5 % over the final
10M steps) → the standard verdict applies: scale, same objective.

**Compute policy (user, final form):** RunPod retired after round 2.
Training runs on **Mira nightly: cron 23:00 start (waits for free VRAM —
never evicts ollama; a resident model is not a veto, big ones just fail
the >=10 GB gate until they self-unload), no new starts after 05:00, hard
stop 07:00** (SIGTERM-safe: params checkpoint at every eval). Arm:
`infra/night_arm.sh`, collect: `infra/night_collect.sh`.
`train_mjx --init-from` (brax restore_params, validated) carries a
checkpoint across nights. **Round 3 armed for tonight: warm-start from
precision_v2, +450M steps on the same objective.** The montage reel
(`sim/mjx/render_precision_reel.py` → `sim/renders/precision_reel_*.mov`,
honest PASS/FAIL captions per take) is the per-round visual artifact.

### Precision rounds 3-4: skills unlock via compliance gating (2026-07-18/19)

Full narrative + current scorecard:
[docs/precision-progress.md](docs/precision-progress.md). Summary:

- **User feedback round (2026-07-18):** camera tracks both planar axes;
  single-leg crouch dropped; **recovery-from-fallen integrated into the
  ext_cmd policy** (15% ragdoll-start episodes, recovery reward until first
  stand, no fall termination while down; parity gate 2f — which surfaced
  that ragdoll poses ride JOINT-LIMIT constraint rows, engine-divergent
  like contact manifolds, now screened); **lift requires ≥3 cm clearance**
  (v2's "balance" was a 2 mm hover — caught immediately by the new metric).
- **Round 3 (precision_v3, 400M, first Mira-daytime run; OOM'd at 2048
  envs with mesh-floor colliders on 12 GB → 1024 envs, XLA 0.90):** calm +
  walking (line 5/8 @ 2.0 s, wobble 0.24, CoT 2.5) but skill commands
  answered with standing — kernels paid ~4/step for vels=0 under any
  command. The round-1 disease, localized.
- **Round 4 fix — skill-compliance gate:** under lift/crouch commands the
  velocity/yaw kernels pay ×(0.2 + 0.8·compliance). First overnight-window
  run (282M, 23:00-07:00 cron, validated end-to-end incl. idle-ollama
  unload): **balance_L/R 5/8 at 92-94% real clearance, air circles 5-6/8,
  line_1m 8/8, overall seed-pass 13 → 37/104.** Recovery attempts hard
  (13 W) but no stand yet; sidestep/backward/turn-tracking remain open.
  Tonight continues v4 (same objective, warm-start).

### Research-first replan (2026-07-20): stop re-inventing the wheel

User directive after the Xiaomi review: research before more GPU nights.
Three deep-dives (Playground source extraction, AMP/get-up literature,
small-biped sim-to-real survey) produced
[docs/training-plan-v2.md](docs/training-plan-v2.md). Highlights: Open Duck
Mini (same STS3215 servos, same MJX+brax stack, sim-to-real WORKS) uses
procedural reference-gait imitation, not AMP — our route to natural gait;
IMU-only command-conditioned obs is the field standard (no onboard velocity
estimator needed — the "odometry gap" dissolves for locomotion); Playground
supplies exact missing reward terms (gait-phase clock, feet slip/clearance,
quadratic orientation, pose/limit regularization) + asymmetric privileged
critic; get-up needs head-height + rise-jerk terms + HumanUP's two-stage
stretched-reference recipe; Open Duck's BAM-identified STS3215 params to
cross-check ours. Tonight's queued runs (feet twins + v6) were preempted —
they'd have trained techniques the plan replaces. Our servo/latency/
backlash/IMU DR modeling and the CPU-referee rule survive review as ahead
of the surveyed field.

### The sim foot finally matches the CAD foot (2026-07-30)

The training plant `sim/bimo_biped_v3yaw.xml` stood on one 90 x 46 mm box per
foot — the pad of the *old* 100 mm as-built foot — while CAD has carried the
116 mm get-up foot with a 106 x 46 mm TPU pad (x −45..+61, y ±23) since
2026-07-15. The obvious fix, growing the box, was tried earlier on 2026-07-30
and reverted: it blew up MJX-vs-CPU parity gates 2g/2h (worst |dqpos| 2.9e−12
→ 9.3e−04, |dobs| 1.5e−08 → 1.3e−01). The reverting note blamed MJX's 1 mm-skin
box-on-plane manifold pruning.

**That diagnosis was wrong.** Gates 2g/2h are AIRBORNE (torso hoisted to 1.5 m,
re-hoisted every 20 steps) and CPU records zero floor contacts across all 100
steps of either block. Per-step instrumentation pinned the failure on exactly
one step in each block (2g t=70, 2h t=78) and the real culprit turned out to be
foot-ON-foot: `L_sole`/`R_sole` carried `conaffinity="1"`, making them the only
self-colliding pair in the whole model — the `class="cad"` leg meshes are
`conaffinity="0"` and pass through one another freely. `mj_geomDistance`
between the two soles over the gate-2g sequence: the 90 mm boxes clear each
other by 4.19 mm at the worst step, the CAD-size boxes by 0.31 mm. At 0.31 mm
MJX's box-box collider calls it a penetration and CPU's does not, and that lone
disagreeing step is the entire failure.

The fix has two halves:

* **Ground contact is `class="pad"` spheres** (r = 3 mm) per foot on the sole
  outline. *(First cut used four at x −45/+61, y ±23 — a "106 × 46 TPU pad".
  Corrected same day, see below.)* Sphere-on-plane is one analytic point both engines agree on;
  on flat ground those four points *are* the box manifold, so settled stance is
  unchanged (torso z 0.3249 m, four contacts per foot, max |qvel| 3.8e−11).
  Measured MJX/CPU contact-point agreement over a gait-amplitude rollout on
  v3yaw: **87 % with a CAD-size box sole, 97 % with the spheres.**
* **The two feet no longer collide** (`conaffinity="0"` on the pads). Foot
  crossing stays a reward/pose matter (`w_foot_cross`, `check_sit_pose`'s 5 mm
  gap assertion), which is how the rest of the model already treats
  self-intersection.

`L_sole`/`R_sole` survive as non-colliding, group-3 **reference** geoms frozen
at their old pose, because `walker_env`/`env_mjx` read `geom_xpos["L_sole"]` for
the swing-foot target, the CoM-over-stance kernel and the lift-clearance zero
(`_sole_z0`). Keeping them bit-identical moves the contact patch without moving
anything a trained policy observes; retarget them deliberately, not as a side
effect. Body masses/inertias are untouched (total 1.1517 kg) — every body has an
explicit `<inertial>`, so the `mass="0"` pads change nothing. Contact-flag
lookups in both envs now key off a per-foot pad-geom set (`_pad_gids`), falling
back to the sole box on the legacy v2 plants, which are unchanged.

### ...and then the calipers said 52 x 116 (2026-07-30, later)

The above landed the plant on a "CAD 106 x 46 TPU pad". User, measuring the
part on the bench: *"keep the model consistent with what's printed. However I
measure the current printed foot as 52x116mm, with rounded corners."*

That is the CAD foot exactly -- `FOOT_L` 116.0, `FOOT_W` 52.0, `FOOT_TOE_R` =
`FOOT_HEEL_R` = 14.0, and `foot.stl`'s sole plane spans x -52..+64, y -26..+26.
So **106 x 46 was neither the as-built foot nor the CAD outline**, and the
"as-built 100 mm foot with a 90 x 46 pad" this plant was built on was wrong
too. Two consequences worth keeping:

* **The plant does not lead the bench.** The note claiming a 16 mm CAD-vs-
  hardware gap rested on the 90 x 46 figure and is retired.
* **Rounded corners change the POINT COUNT, not just the size.** A 116 x 52
  rectangle with R14 corners reaches full length only at y +/-12 and full width
  only over x -38..+50 -- the extreme points in x and in y are in different
  places, so no FOUR-point set holds both. Four at the arc diagonals gives
  111.8 x 43.8; four at the box corners claims contact where there is no
  material. The plant now carries **eight** pads per foot on the corner-arc
  tangent points (the inscribed octagon): support polygon exactly
  116.00 x 52.00, area 5640 mm2 of the sole's true 5864 mm2 (96 %).

The pads keep their z, so nothing a policy observes moved: settled torso z
0.3249 m, mass 1.1517 kg, `L_sole` reference geom still frozen. `PARITY: PASS`
on every gate, no threshold touched. The `<side>_pad` name-prefix discovery in
both envs picks up eight as readily as four.

### The yoke support fins were worse than no supports (2026-07-30)

User, on the print variants: *"None of the supports break away or are done very
well. the feet have extra pads on them which screwed up the whole base... then
they stick out beyond the fork without even bothering to support anything."*
All four of those turned out to be measurable, and the base one is fatal:

- The anchor tabs were built from `lo - 0.3`, i.e. **0.3 mm below the bed
  plane**. So the tabs became the lowest geometry and the flange — the actual
  bed adhesion — floated. Real first layer: **26 mm² (roll) / 36 mm² (pitch)**
  of disconnected 2×3 stamps, against the flange's own 194 / 175 mm². That is
  14 % and 21 %. The flared feet added *for* bed area sat 0.30 mm up, touching
  nothing at all.
- Every foot splayed **2.00 mm past the part's own silhouette**.
- The pad-rim fins were flat-topped blocks under a **cylindrical** pad: contact
  on one tangent line, gap opening to **3.20 mm** over 7 mm of run.
- Each tab fused **0.3 mm into the part across 2 × 3 mm** — a weld, not a
  break-away contact. Snapping it tears the arm plate.

Deleted. Both yokes print with slicer supports now (settings in
`cad/PRINT_LIST.md`), and there is one STL per part instead of a `_print`
variant to keep in sync.

The lesson worth keeping is the root cause, not the arithmetic. Those tabs
existed to stop `check_printability` reading the plate as a BEAM, and the walls
were full plate width to stop it reading them as an ISLAND. The geometry was
shaped to satisfy the audit — which has no way to express "a support sits
0.35 mm under this" — and that geometry then shipped to the printer. An audit
that cannot model something should say so, not be argued with in solid material.
`check_printability` now has a `SUPPORTED` set: those parts report their
overhangs as `[support]` instead of failing, and thin walls and everything else
still fail normally.

### Watching the ROM in CAD, and checking it automatically (2026-07-30)

Two separate things, because they answer different questions and one of them is
not evidence.

**The check.** `cad/check_assembly.py` tests pose EXTREMES and a handful of
combined poses. A part can foul in the MIDDLE of a joint's travel and pass both
endpoints — endpoint checks miss that by construction. `cad/freecad_rom_collide.py`
walks each joint across its whole range in the real articulated assembly (13
samples per joint, both legs) and intersects every relatively-moving pair. It
now exits 0/1 rather than only printing, so it is a gate; `sh cad/run_checks.sh`
runs it next to `check_assembly.py` and returns one status.

Two traps worth recording. `freecadcmd` tears the interpreter down on
`SystemExit` **without flushing Python's stdout**, so exiting straight out of
`main()` threw away the whole report and left a check that printed nothing and
said PASS — flush before exiting. And the sweep must transform shapes extracted
once at neutral rather than setting Placements and recomputing, because
`doc.recompute()` re-runs the MbD solver on every pose (~130 solves) for a
result the solver has no say in. Current verdict: ALL CLEAR, tightest gaps
0.30 mm (yaw vs roll, the designed slip fit) and 0.32 mm (shin vs foot).

**The watching.** FreeCAD has no swept-ROM collision tool — the Assembly
workbench does no collision detection at all (issues #28165, #14528, #13390),
and the Animate workbench's Collision Detector wants DH-notation kinematics and
manual triggering. But FreeCAD 1.1's built-in Assembly → Simulation plays joint
motion natively in the viewport, which is the part worth having.
`cad/freecad_sim_setup.py` builds ten of them, one per joint per leg, so each
plays one joint at a time.

Three things that had to be got right:

- **Every simulation drives all ten joints.** Ten revolutes is ten free DOF; a
  Simulation with one Motion constrains one, and MbD puts the other nine
  wherever solves. Driving `knee_L` alone landed the foot 63 mm off with the hip
  yawed out of plane. So nine Motions pin their joints at `0*time`.
- **`generateSimulation` solves from the CURRENT placements**, not from neutral.
  Generating ten in a row without restoring compounded — the foot was 390 mm out
  by the fifth, identically on both legs, which is what gave it away.
- **The formula parser takes `sin`, `cos`, `pi` and products, but not
  comparisons or `abs`** — those parse to a *silent* zero-frame simulation. No
  piecewise, so no literal neutral → max → min → neutral ramp. Each joint gets
  `phi(t) = a·sin(2πt) + mid·(1 − cos(2πt))` in radians, with mid = (lo+hi)/2 and
  a = √(half² − mid²): a sinusoid about `mid` phase-shifted so phi(0) = phi(1) = 0.
  It starts at neutral, rises to exactly `hi`, falls to exactly `lo`, returns.

The setup self-verifies: it plays each simulation and finds the frame whose foot
lands closest to where `freecad_pose.stage_placements` puts it at each ROM limit
— which checks amplitude, sign, units and the pinning at once. All ten reach
both limits, worst error 0.036 mm. Units are radians and the sign agrees with
ours (`0.7854*time` turns the knee exactly +45.000°).

MbD does no collision detection, so the simulation shows motion and proves
nothing about interference. That is what `freecad_rom_collide.py` is for.

### The last part still seated on a face that isn't there (2026-08-01)

`SV_BOTFACE` (−17.35) is not a real surface. The ST3215's case is **not**
symmetric about its output axis: the idler side stops 2.60 mm short of the
mirrored face, at `SV_IDLER_CASE_FACE` (−14.75). That was established on
2026-07-29 and fixed in the `leg_link` grip plate, then in the `foot` tab on
2026-07-30. The `yaw_carrier`'s rear bay wall was the last part still sitting
on the phantom face, so both of its hip-roll retention screws were tightening
against a wall that touched nothing.

Placing the servo mock in the bay and probing inward from the wall face shows
why this is more than a gap. In order, the first things a naive seat would
reach are the **free hub** (−17.35) and the **idler disc** (−16.80) — *both of
which rotate with the joint* — then the moulded back-cover platform (−16.65),
and only then the case at −14.75. Seating on the first material you touch would
have clamped the joint solid.

The fix is a seat that grows **inward only**, to `SV_IDLER_CASE_FACE −
GRIP_SEAT_CLR` (−14.90), with a channel down the middle for the platform. Three
things are worth recording:

- **The wall's outer face did not move.** `ROLL_ARM_INNER` is measured off it,
  so pulling it in would have silently re-opened the hip-roll idler screw budget
  settled the day before. Screw engagement is unchanged either way — the M2.5×8
  sits flush at −19.95 and bites the case 2.80 mm whether the 2.45 mm ahead of
  it is plastic or air. What changed is that it now pulls the case against
  something.
- **The platform relief is a channel, not a pocket.** This servo slides *up*
  into the bay, so the platform has to travel the full height of the seat to
  reach its place; the `leg_link`'s closed pocket would simply be a wall the
  servo could not get past. Its side walls straddle the platform by
  `RIB_RELIEF_CLR` a side, which is what makes it a *detent* — it locates the
  servo across the bay — rather than a clearance hole. Printed horn-plate-down
  the channel's closed end is its print floor, so nothing bridges.
- **One of the two rear screws still cannot bear, and that is geometric.** The
  platform sweeps the corridor on its way in, and the lower screw row (8.30 mm
  from the axis) lies 0.85 mm inside it. With vertical insertion there is no
  shape that both lets the platform through and puts material under that screw.
  It ends up 24 % backed against the upper row's 65 %; both were 0 % before.

`check_assembly.py` gained a **standoff probe** on the side lands, because
clearing the servo and seating on it are not the same measurement and the
existing interference check passed happily throughout the 2.60 mm era. It reads
0.15 mm now and 2.60 mm against the old geometry.

### The knees bent the wrong way (2026-08-02)

The plant and the robot did not agree about which way a knee bends, and the
disagreement was a sign, not a number.

`sim/bimo_biped_v3yaw.xml` carried `axis="0 1 0" range="-95 5"` on both knees.
On that axis a *negative* knee angle swings the shank **forward** — a
bird-style backward-bending knee — so the model had 95° of hyperextension and
5° of flexion. The robot is calibrated the other way: `dir −1` on both knee
servos (docs/servo-map.md), chosen deliberately so the policy's negative knee
means human flexion, and confirmed on camera — servo 3 at raw 2278 (−20°
through zero 2050) swings the shank back and lifts the heel. **Same command,
opposite motion.** None of the knee constants in the envs (`kneel` −1.62 rad,
`sit` −0.09, the mimic swing-bend) had ever meant what its name said in the
sim; they only meant it on the robot.

The fix is one attribute per knee: **`axis="0 -1 0"`**. The range string does
not move, and that is the whole trick — `−95 … +5` now reads as 95° of flexion
and 5° of hyperextension instead of the reverse. Verified in MuJoCo after the
change: −20° moves the foot 3.1 cm backward in the torso frame, −95° moves it
9.0 cm back and 9.8 cm up.

**Why the range stayed −95/+5.** ±95° is the *mechanical* envelope measured on
servo 3 (−94.6°/+94.5°, ≤6 % of stall, 31 °C over two full sweeps), not a
sensible operating range. The flexion limit is set to the measured travel. The
hyperextension cap stays at 5° because a knee that folds backward is a failure
mode, not a gait: the modelled envelope is deliberately a strict *subset* of
what the joint can physically do, so nothing a policy learns is unreachable on
hardware. Widening it is a decision someone can take later on purpose.

**Nothing on the robot changes.** `zero_steps` and `dir` are untouched;
`obs_spec.h` regenerates byte-identical (the numbers did not move, only their
meaning); the firmware's bench clamp still resolves to servo 3 ticks
`[1993..3131]`, now correctly read as +5° of hyperextension through 95° of
flexion. No reflash, no re-zero, no cal migration.

**Everything trained before this is invalid.** Every run in `sim/runs/` learned
the mirrored knee, so its scorecards describe a plant that no longer exists.
The runs and their scorecards are kept as history and must not be read as
claims about the current plant. No retraining was launched with this change.
Two other artefacts inherit the same staleness: `sim/getup_catch_states.npz`
(harvested qpos, so its knees are mirrored — regenerate with
`sim/mjx/harvest_rise_states.py` before the next get-up round) and the exported
SIL weights under `sim/sil/weights/`.

**Three things fell out of the sweep.**

- `cad/dimensions.py ROM["knee"]` is stated as a *physical* rotation and was
  copied from the MJCF, so every CAD interference sweep through the knee had
  been probing 95° of hyperextension and 5° of flexion — the poses the robot
  never makes. Corrected to `(-5, 95)`, along with the hardcoded fold pairs in
  `check_assembly.py`. It re-runs **ALL CLEAR**: 0.70 mm of buffer between the
  thigh fork arms and the shin grip screws at +95° (it was 0.76 mm at −5°), and
  no contact anywhere in the knee × ankle fold matrix.
- `sim/sil/harness.py` was clamping the **sensor** direction to the joint range
  (`obs::angleToSteps` clamps because it writes *goal* positions). MuJoCo joint
  limits are soft, so once a stale policy started leaning on the +5°
  hyperextension stop the joint sat 1.5° past it and the harness fed the
  firmware an angle walker_env never had — a 17-tick divergence that looked
  like a boundary bug. A real encoder reports where the joint *is*; the clamp
  is off on that path now.
- Parity block **2j-b** poses a one-leg march stance so the knee-high
  clearance kernel is non-zero and its arithmetic is actually compared. Its
  pose `(knee −0.62, hip −0.31)` lifted the foot on the mirrored plant; flexing
  the same numbers now dips the swing foot 10.5 mm *through* the floor, so
  `con_swing` was true and `knee_frac` collapsed to a trivial 0 — the block
  went red not on a CPU-vs-MJX divergence (1.5e-07 reward, 1.5e-08 obs, both
  far inside their gates) but on its own liveness assertion. Re-posed to
  `(−0.90, −0.90)`, a real high-knee stance: swing pads 34 mm clear, `ncon` a
  flat 8, `knee_frac` ≈ 0.44.

The legacy 8-DOF plants (`bimo_biped.xml`, `bimo_biped_v2*.xml`) keep the old
knee axis on purpose: they are frozen referees for historical runs, not the
plant anything trains on. `sim/mjx/check_sit_pose.py` still points at
`bimo_biped_v2_asbuilt.xml` and therefore still checks the old convention.

### v4rom: the legs can touch each other, and two stops were fake (2026-08-03)

The 2026-08-03 bench + CAD session measured the robot against the plant and
found `bimo_biped_v3yaw.xml` wrong in two directions at once: it *forbade*
travel the machine has, and it *allowed* poses the machine physically cannot
reach. `sim/bimo_biped_v4rom.xml` is the successor that fixes both. It is
opt-in — `--xml` on train/eval — and **v3yaw remains the scoring plant for
every existing run**, so no scorecard in `sim/runs/` is touched or invalidated
by this commit.

**The legs went through each other.** Every internal part in this model is
`contype 0 / conaffinity 0`, and the two feet were deliberately un-collided on
2026-07-30 (foot-on-foot box manifolds were what broke MJX parity gates 2g/2h).
The result: nothing on this robot could touch anything else on this robot. The
bench says otherwise, twice:

- **hip-roll adduction** loads up at ≈ **−9°** from standing, and a torque-off
  leg comes to rest at −7° leaning on its neighbour. The plant happily drove
  both hips to their ±25° stop, legs interpenetrating. The `bench_rom_sweep`
  tool has been capping inward roll at 6° by hand because of this.
- **hip yaw**, thighs raised 90° with the shanks hanging, crosses shank-on-shank
  at ≈ **±8.5°** per side — while standing toe-in is clean out to the
  errata-tested 20°. Same joint, two different limits, because it is not a
  limit: it is a *pose-dependent contact*.

Neither is expressible as a joint range, which is exactly why
`firmware/main/mech_envelope.h` leaves them out of the bench clamp table. The
plant needs geometry, so v4rom adds four capsules — one per thigh, one per
shank, on the segment's own joint axis, 30 mm long, r = 14.7 mm — and four
explicit `<contact><pair>` entries covering L×R thigh/shank. The capsules carry
`contype 0 / conaffinity 0`, so those four pairs are the *complete* list of
things on this robot that can collide with each other; nothing is left to a
contype-bit filter that would also have collided each leg with itself.

The capsule is the top-of-segment **servo case** (the widest thing on the leg,
half-width 16.05 mm, spanning local z +10.1…−35.1 mm) reduced to its inscribed
cylinder about the joint axis — 14.7 mm is the case's *inner* half-width, the
face that actually meets the other leg. But the size is **calibrated, not
copied**: 14.7 mm × 30 mm is the pair that reproduces both measurements at once.
The capsule's lowest point sits 170 mm below the roll axis and the hips are
56 mm apart, so adduction closes at `asin((56 − 29.4)/170)` = **9.0°**; in the
raised-thigh pose the shanks stand 90 mm out from the yaw axes, so a symmetric
inward yaw closes at `asin((56 − 29.4)/180)` = **8.5°**. Measured: 9° and 8.5°.
A capsule reaching further down the shank contacts at ~6°, one sized to the
case's *outer* face at ~7° — the two constraints together pin it. Sanity: the
poses that must stay clean have ≥ 26.6 mm of air (90/90 sit, full −95° knee
flexion, deep crouch, toe-in to the ±45° range edge, the new 55° abduction,
hip pitch ±90/−110). `tests/test_rom_contacts.py` asserts the onsets to ±2°,
which is about what "load onset" is worth as a measurement given the same
sweep saw the torque-off rest at −7°.

Dynamic check, `action_map="full"`, commanding both hips to full adduction for
3 s: v3yaw drives to **−25.0°/+25.0°** (legs through each other), v4rom settles
at **−4.8°/+4.5°** — the symmetric both-legs-adducting onset is 4.5° per side,
and the contact holds it there without blow-up.

**How the new contact behaves across the two engines** (measured, airborne, so
the floor is not involved). *Geometry*: MJX and the CPU referee agree on the
capsule separation to **1e-6** at every probe angle, and both flip sign at the
same place — one analytic capsule-capsule manifold, no box corners for MJX's
1 mm skin to prune, which is the failure mode that retired the foot-on-foot
box contact in July. `tests/test_rom_contacts.py` gates this. *Manifolds*: over
120 synced steps with both hips driven into the stop, the engines saw the same
contact set on 119 — the one miss is a grazing step at zero force. *Loaded
arithmetic*: pressed hard (1–2.5 mm of penetration under a full-range roll
command) a single synced step diverges by up to **1.5e-5 rad qpos / 1.3e-3
rad/s qvel**, scaling with contact force and falling to float64 noise
(1e-11) as the load comes off. That is a constraint-solver difference, not a
manifold one, and it is the same order as — actually smaller than — the
foot-impact transients `parity_test.py` already documents and handles the same
way: DR in training, CPU referee for scoring. `sim/mjx/parity_test.py` is
**not** extended with a v4rom block; it gates arithmetic in regimes chosen to
be contact-free precisely so this class of difference stays out of it, and a
block that had to be gated at 1e-4 would not be gating anything the pytest does
not already pin harder. Worth knowing before reading a v4rom training curve: a
policy that spends its time leaning on its own legs is in the one regime where
the two engines are only approximately the same robot.

The other consequence of a soft contact: pressed at full actuator force the
capsules sink 1–6 mm, i.e. the legs overlap by up to ~2° more than the onset.
The onsets above are measured at zero load, which is what the bench measured
too ("load onset"), and `solref` is left at the model default — the same one
the pads and the floor use.

**Two stops were narrower than the machine.** Hip-roll **abduction** is
CAD-clear to 55° per side (buffer sweep: 0.60 mm at 55°, 0.33 at 70°, 0.00 at
90° — the ~90° the design was assumed to have is refuted by the geometry that
got printed), and hip **pitch** runs to **+90°** backward (0.70 mm of
grip-screw/yoke buffer, and both hips were driven there on the robot that day
after re-centring the left hip-pitch encoder). v4rom's joint limits become
`L_hip_roll −25…+55`, `R_hip_roll −55…+25` — **asymmetric per leg**, because
positive roll moves the toe toward the robot's left, so the same physical
abduction is +55 on one leg and −55 on the other — and `hip_pitch −110…+90` on
both. The −25 adduction side is kept as a modelling floor rather than a
measurement: the legs touch at 9° now, so the joint's own stop on that side can
never be reached.

**The knee stays −95…+5, having gone looking for a reason to change it.** ±95°
is the measured mechanical envelope, but hyperextension is a failure mode, not
an operating mode, and nothing in the hardware session argued for a policy that
hyperextends. It is also the one joint whose plant stop *protects* the machine
rather than describing it — the 4095-tick encoder wrap is why `R_knee` had to be
re-centred at all. Opening it stays a deliberate, separate decision.

**The action-map decision, and why the widening is free.** Both envs derive the
action map from the model: `action_map="full"` spans `hi − default` above the
pose and `default − lo` below, `"legacy"` scales by `0.5·(hi − lo)`, and both
read `lo/hi` straight off `jnt_range`. So *widening a joint limit silently
rescales every action a trained policy emits* — a 55° roll limit would mean
`action = −1` on the roll channel commands 55° instead of 25°, and every
scorecard describing that channel becomes fiction. The same numbers also set
the init-pose clips, the reset randomization and the 90 % soft-limit penalty.

The alternative is the split the **firmware already runs**: `obs_spec.h`'s
`kJointLo/kJointHi` (policy range, action scaling) versus `mech_envelope.h`
(bench clamp, mechanical truth). v4rom adopts it in the plant. The joint limits
become the mechanical stop; the **policy range** moves onto an explicit
`ctrlrange` on each `<position>` actuator, holding every joint at v3yaw's
numbers exactly. `policy_range()` in `sim/walker_env.py` (mirrored in
`sim/mjx/env_mjx.py`) prefers `ctrlrange` when the plant declares one and falls
back to `jnt_range` when it does not — so `bimo_biped_v2*.xml`, `v3yaw` and
every legacy referee load **bit-identically** and their runs keep meaning what
they meant. `hip_flex_deg`, being a training-range knob, moves the ctrlrange on
a split plant and the joint limit on the old ones.

Two consequences worth stating. `tools/gen_obs_spec.py` emits `kJointLo/kJointHi`
from `env._lo/_hi`, so a run trained on v4rom still ships the *policy* range to
the robot and `mech_envelope.h`'s containment `static_assert` still holds — no
firmware change is needed to deploy off this plant. And `ctrlrange` is in the
actuator's own units, which the compiler does **not** convert under
`angle="degree"` (it converts `jnt_range` and geometry, not controls), so those
ten lines are spelled in radians inside a degree file; the test asserts each
against the degree value it is supposed to be.

**Migration cost for the lineage.** A v3yaw-trained policy's actions mean the
*same joint angles* on v4rom — asserted, both action maps, plus identical
masses, inertias and standing height (0.3249 m), and a bit-identical
stand-still return of 410.85 over 250 steps. What it does not know is the new
contacts: any behaviour that leaned on legs passing through each other now hits
something. So the next round is a **warm start from `loco_v12knee_warm` on
`--xml sim/bimo_biped_v4rom.xml`**, not a cold retrain, and it is expected to
survive — the precedents are `v8foot` (foot contact patch replaced) and
`v12knee_warm` (the knee sign flipped outright, and the warm start still beat
the cold one). Score it on v4rom; v3yaw scorecards stay valid *for v3yaw* and
the two columns are not comparable once contacts differ. Opening the extra
abduction and back-pitch travel to the *policy* is a further, separate commit
that would cost a real retrain — v4rom deliberately does not spend that.

**CAD.** `cad/dimensions.py ROM` now sources the v4rom **joint limits** (the
mechanical stop — a part that only clears the training box is a part the bench
can break): `hip_roll ±55`, `hip_pitch −110…+90`, knee unchanged at its physical
`(−5, +95)`. Roll is written symmetric because this file describes *one* leg
that gets mirrored onto both, and the check is sign-symmetric in any case
(0.60 mm at both +55 and −55). `check_assembly.py`'s hardcoded probe lists grew
to match (roll ±55 in the hip-stack and deep-flexion sweeps, hip +90/+95 in the
thigh sweep). It re-runs **ALL CLEAR**, including thigh-at-−115° crossed with
±55° roll.

## 6. Design parameters (source of truth)

| Param | Value |
|---|---|
| Servo (STS3215) | 45.2 × 24.6 × 35.1 mm, 55 g, 2.94 N·m stall |
| Joint limits (mechanical, `v4rom`) | hip-yaw ±45°, hip-roll 25° adduction / **55° abduction** (asymmetric per leg: L −25…+55, R −55…+25), hip-pitch **−110…+90°**, knee −95…+5° (flexion is **negative**: MJCF axis `0 -1 0`, so this is 95° of flexion and 5° of hyperextension — see "The knees bent the wrong way"), ankle ±40°. Leg-on-leg contact rules before the roll and yaw stops in most poses — see "v4rom". |
| Policy range (what a policy trains in) | hip-roll ±25°, hip-pitch −110…+60°, others as above. Carried as `ctrlrange` on v4rom's actuators, unchanged from `v3yaw`; widening it rescales the action map and costs a retrain. |
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
- **Stage 3 — Sim-to-real:** system-ID the real servos, export ONNX policy, deploy on the
  ESP32 (docs/wiring.md; *not* an RP2040/SBC — that was an early sketch). The wireless
  command channel it runs under is specified and sim-verified in
  [docs/control-channel.md](docs/control-channel.md).

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
      latency), ONNX export, and deploy pipeline to the **ESP32** (the board on the order
      sheet; earlier drafts said RP2040 + SBC).
- [ ] **Firmware:** port `link/protocol.py`'s decode + `Watchdog` to C on the ESP32 and run
      the 50 Hz policy loop beside the servo bus. The protocol itself is already specified
      and exercised against MuJoCo over real UDP — see [docs/control-channel.md](docs/control-channel.md).
- [ ] **Odometry:** the goal-seeking commander needs a pose estimate the robot cannot yet
      produce (the BNO085 gives attitude only). Today `link.sources.GoalSource` is fed
      ground truth by the sim; on hardware that gap is unfilled.

### Known limitations / caveats
- Massing CAD ≠ printable geometry.
- v1 (`bimo_biped.xml`) inertias are approximate boxes; v2 has mesh-derived
  inertia but still box *contact* geometry.
- No self-collision between internal parts (feet-only contact) — revisit when adding meshes.
- v1 actuator is idealized; the sts3215 model adds the torque-speed envelope but
  not backlash or serial-bus latency (use `action_latency` as a proxy).

### Battery bay widened to the 3S 850 field (2026-07-15)

The bay was cut for one specific pack — the Zeee 3S 850 (67 × 30 × 18.5, 74 g) —
which is now unavailable on Amazon (direct-only, US stock out). That turned out
to be a **single-supplier dependency**: the Zeee is long-and-flat at 18.5 mm,
while every other 3S 850 on the market is stubby-and-tall at 22–25 mm, so
nothing was a drop-in. Height, not capacity, is the binding axis.

Fix: size the envelope to the whole field rather than one SKU.
`BATT` 68 × 31 × 20 → **68 × 31 × 26.5** (tallest pack 25 + 1.5 fit/pad), which
required `TOWER_H` 37 → **43.5** to hold the derived stack
(`TOWER_H = BATT[2] + 13.5 + 3.5`, where 13.5 = top plate + standoff + ~4 mm
board components). Board-to-pack clearance is **unchanged at 3.5 mm** — the
growth went into the tower, not into the margin. Length stayed 68 so the flat
Zeee still seats if it returns.

Consequences: tower top 324 → **330.5 mm**, robot ~884 → **~892 g** (+6 g pack
worst case, +2 g taller print), torso 319.5 → **327.8 g**, standing CG 164 →
**165 mm** (197 with camera). Gates re-run clean: `check_assembly` ALL CLEAR,
`check_printability` all parts clean, fly-in re-rendered.

Also fixed while here: `build_v2_inertia.py` placed the 20 g driver-board box at
`TOWER_H + 2.5` — *above* the tower — stale from when the board mounted on top;
it now derives from the real face-down standoff plane. And `battery_mock()` in
`export_assembly.py` hardcoded 67 × 30 × 18.5, so the fly-in was proving the old
pack fit. Both the mock and the inertia box now read `BATT_PACK` (62 × 30 × 25,
80 g — the worst case of the field), so there is one source of truth.

**Policy impact: none.** `dash_11v1_hardlat3` re-evaluated on both plants
(GoPro + 4 ms, 32 episodes):

| | old (319.5 g) | new (327.8 g) |
| --- | --- | --- |
| dash finishes | 32/32, median 2.85 s | 30/32, median **2.70 s** |
| torso wobble | 1.96 rad/s RMS | **1.88** |
| DR + shoves | 0/16 | 0/16 |

30/32 vs 32/32 is inside noise at n=32, and the new plant is *faster and
smoother*, consistent with +8.3 g being +0.9% on a policy that trains under
±15 % mass DR. No repair fine-tune needed. The DR+shoves column is 0/16 on
both — identical, so no regression, but it was already floored (see the
robustness gauntlet above); this change neither caused nor fixed it.

Note `eval_policy.py` defaults to the `xml_path` baked into each run's
`env_config.json` (an absolute path into the main checkout), so re-running
these without an explicit `--xml` silently measures the *old* plant.

**Open — battery restraint (the pack must not move).** Short packs (59–62 mm)
have up to 8 mm of slop in the 68 mm bay, and `BATT[0]` runs along **y —
the lateral axis**. That is the robot's weak plane: the sole is 90 mm fore-aft
but only 46 mm wide, and at 56 mm hip separation the single-support polygon's
inner edge sits 5 mm off centerline, so the nominal CoM is already outside the
stance foot. An unrestrained 80 g pack (9 % of robot mass) would give (a) a
permanent ~0.37 mm lateral CoM bias if it settles off-centre — `build_v2_inertia`
models it centred at y=0 and nothing enforces that — and (b) gait-phase-locked
wall impacts (~10-15 % of the 5 N test-shove impulse, but every step rather than
1 % random).

The sim **cannot represent this**: the pack is a massless visual geom whose mass
is baked into the torso `<inertial>`, and DR scales `body_inertia` by the same
scalar as mass while never touching `body_ipos` — so torso CoM position is
randomized by exactly zero in every rollout. Precedent for taking this
seriously: issue #3, where an oversized contact patch silently propped up a
policy's robustness and correcting it collapsed `dash_11v1_hardlat` 30/32 → 2/16.

Decision (2026-07-15): restrain mechanically with foam and/or two symmetric
printed shims of `(BATT[0] - BATT_PACK[0])/2` — symmetric so the pack is
*centred*, making the sim's y=0 assumption true by construction rather than
merely assumed. Sims proceed on the assumption the pack does not move.
Not yet designed; the shim part and a real test-fit are outstanding.

Also outstanding: the belt guide ribs derive from `BATT[2]` (envelope), so they
moved to z=18.5/29.5. For a 23-25 mm pack the lower belt still crosses it, but
for a flat 18.5 mm pack the belt now sits at its top edge and restrains little —
the ribs arguably want to track `BATT_PACK[2]` instead.

## 9. References

- Bimo Project — `github.com/mekion/the-bimo-project`, `mekion.com/project`
- STS3215 dimensions — servodatabase.com / waveshare.com (ST3215)
- Simulator choice — roboticscenter.ai (MuJoCo vs Isaac Sim, Best RL sims 2026)

### Measured servo speed: the datasheet envelope was 14% optimistic (2026-08-03)

`tools/measure_servo_speed.py` (robot on the stand, pack at 12.3 V, camera
rule noted — future runs record video): R_knee free sweeps over 2140 ticks,
both directions, at commanded goal speeds 500/1000/2000/3400/unlimited.
Commanded speeds track within 0.5% up to 2000; the ceiling is **~2700
steps/s = 4.14 rad/s at 12.3 V**, vs the datasheet-derived 4.712 rad/s @12 V
the sts3215 actuator model assumed (voltage-scaled: 3149 steps/s expected at
12.3 V — the real servos deliver 86% of that). Under real load (L_hip_pitch
lifting the whole leg) speed derates to ~1650 steps/s, confirming the
torque-speed slope. `_STS_NOLOAD_12V` in walker_env.py AND env_mjx.py is now
the measured 4.04 rad/s (12 V basis). Caveat: every scorecard produced
before this change was refereed against the optimistic envelope; new runs
stamp git_sha as usual. Training's `supply_voltage=11.1` had been
accidentally compensating (its 4.36 rad/s envelope was only 5% above the
real 4.14) — that near-cancellation is why nothing obviously broke in
sim-to-real bench behaviour so far.
