# Precision curriculum — walking before running

**Status:** infrastructure landed 2026-07-17; first training round pending.
**Prompted by:** "we have been trying to run before we walk." The dash/command
lineage optimizes speed; this round optimizes *control*: balance, exact
distances, returning to a point, moving slowly in every direction.

## The seven skills (user-specified) and how each is graded

All seven are commands to ONE policy (`ext_cmd` mode), not separate experts —
the one-behavior-per-run distillation path was tried and closed 2026-07-13,
and GPU-scale command-conditioning (mjx_cmd_v1) is the proven recipe.

| # | skill | command encoding | eval scenario (eval_precision.py) | success |
|---|---|---|---|---|
| 1 | balance on one leg ≥10 s (each) | lift = −1 (L) / +1 (R), vels 0 | `balance_L/R` | no fall, contact <5%, **≥3 cm clearance ≥90% of the window** (2026-07-18) |
| 2 | circles in the air with one leg | lift ±1 + foot (dx,dz) driven on a circle | `circle_air_L/R` | no fall, <10% touchdown, clearance ≥80%, foot-tracking RMS |
| 3 | walk a straight 1 m line | vx 0.4, stand at x≥1 | `line_1m` | ≤8 s, \|y\|<0.15 at the line, stops within 0.30 m |
| 4 | circle/square, return to start | closed-loop waypoints (square) / constant (vx,wz) (circle) | `square_return`, `circle_return` | return error <0.25 m / <0.35 m (+ real ≥0.3 m excursion) |
| 5 | side-step 0.5 m | vy ±0.2 | `sidestep_L/R` | ≥0.45 m lateral in 6 s, \|x\| drift <0.20 |
| 6 | walk backwards 1 m | vx −0.3 | `backward_1m` | reaches −1 m, \|y\|<0.20, settles |
| 7 | crouch (both legs) | crouch channel 0.6–1.0 (× nominal height) | `crouch_hold` | height within 3.5 cm of target, no fall. *(Single-leg crouch removed 2026-07-18, user call.)* |
| 8 | **recover from a fall** (added 2026-07-18) | 15% of training episodes start from a settled ragdoll; recovery reward until first stand, then normal rules; command = stand | `recover_fallen` | first stand ≤6 s, still tall at episode end |

**Pass/fail vs continuous:** both. Pass/fail is a threshold for the
at-a-glance table; every scenario also records continuous metrics per seed
(endpoint error, tracking error, clearance %, drift RMS, time-to-target,
time-to-stand) aggregated mean+worst in `scorecard.json`, and the training
signal itself is purely continuous (tracking kernels + dense progress).

## Command interface (`ext_cmd=True`, both engines)

7 channels replace the 2-channel (vx, wz) command; obs 38 → 43:

```
c0 vx      m/s    −0.4 … 0.8   (backward walking is a first-class command)
c1 vy      m/s    ±0.25        (sidestep)
c2 wz      rad/s  ±1.0
c3 crouch  frac   0.6 … 1.0    target torso height = c3 × nominal (0.283 m)
c4 lift    −1/0/+1             lift left / none / lift right foot
c5 foot dx m      swing-foot target offset, torso-yaw frame (fore-aft)
c6 foot dz m      swing-foot target offset (vertical)
```

Training mix per command draw (`ext_mix`): 20% stand, 8% crouch-hold, 15%
one-leg balance (30% of those with a shallow 0.8–0.95 single-leg dip), 12%
air-circles (c5/c6 ride a per-episode random circle, r 2–5 cm, period
1.5–3.5 s), 10% pivot, 35% walk (of which 15% backward, 15% pure sidestep,
70% forward with the legacy turning mix).

Reward (mirrors in `walker_env.py` and `env_mjx.py`, parity-gated):

- tracking: 2-D velocity kernel `exp(−((vx−c0)² + (vy−c1)²)/0.25)`, yaw
  kernel (σ 0.5), height kernel `exp(−((h − c3·h_nom)/0.04)²)`.
- one-leg: `w_lift` for the correct contact pattern (stance down, swing up)
  + swing-foot target kernel (σ 3 cm) on `‖foot_rel − (base + (c5,0,lift+c6))‖`.
- **falling is severely penalized**: fall_cost 10 (was 6) *on top of* episode
  termination, which forfeits ~2–4 reward/step for the remainder. Known
  risk, watched for: past DR rounds showed too-high safety pressure makes
  the policy retreat to standing; the tracking kernels are the counterweight
  (standing pays nothing under a moving command).
- fall line scales with the crouch command (0.18 m × c3) so a commanded
  crouch isn't a "fall".

## Plant truth (2026-07-17)

Trained on **`sim/bimo_biped_v2_asbuilt.xml` — the robot as printed today**:

- **Feet: the printed 100 mm foot v3** (90 × 46 mm pad-true contact,
  102.8 g/segment, mesh `cad/stl/foot_asbuilt.stl` from git 191777e). The
  CAD-current 116 mm get-up soles exist only in CAD/`bimo_biped_v2.xml`;
  switch back when the v3.1 feet are actually printed. (User confirmation:
  "we're running with the 100×50 feet".)
  *2026-07-30: `foot_asbuilt.stl` has been deleted — it was a visual-only
  mesh and having two feet in the tree was confusing. The foot physics
  quoted above is unchanged; the plants now render `cad/stl/foot.stl`.
  Recover the old mesh with `git show 191777e:cad/stl/foot.stl` if needed.*
- **Torso 341.8 g** — now includes the tower-top accessory stack
  (imu_carrier 5.0 + BNO055 3.0 + gopro_base 5.2 + fastener delta 0.8 g)
  that was previously an honest omission.
- **GoPro payload on** (154 g) at its TRUE CG: +0.0945 m above torso center
  (`payload_cg_z`; the old 0.08 predates the battery-bay tower growth
  +11.5 mm and the imu_carrier +3 mm). Old runs keep 0.08 by default.
- Robot 921 g bare / 1075 g with camera; both plants stand 500/500.

## Stability / smoothness metrics (the "how do we measure natural" question)

Reported by `eval_precision.py` for every scenario:

- **wobble RMS** (torso roll+pitch rate) — the existing shakiness metric
- **electrical watts** + **cost of transport** `P/(m·g·v)` — gait efficiency,
  comparable across robots (human walk ≈ 0.2–0.4, hopping robots ≈ 1–3)
- **foot slip** — mean sole planar speed while in contact (skating gaits
  score badly; a real-robot killer that task metrics miss)
- **gait symmetry** — |L−R| mean swing-time asymmetry
- lifted-contact %, drift RMS, endpoint errors per scenario

Not implemented (candidate next abstraction if gaits still look wrong):
**AMP-style imitation** — a discriminator rewards motion that *looks like* a
reference gait (scripted or retargeted). That is the literature answer to
"natural gait"; the metrics above are cheap proxies that catch the failure
modes we have actually seen (shuffle, tremble, skate, asymmetry).

## Compute policy + token-lean iteration loop

**Policy (user, 2026-07-17): use Mira's 4070 Ti when idle; RunPod when the
GPU is busy** (supersedes "Mira is dev/smoke only"). Ollama frequently holds
~9 GB VRAM on Mira — a loaded model counts as busy; `ssh mira 'ollama stop
<model>'` frees it.

One round = ONE command, ONE completion message, near-zero tokens in between:

```bash
nohup infra/precision_round.sh --precision --out precision_v1 \
    --steps 150000000 &   # picks Mira-vs-RunPod, trains, pulls results,
                          # referees on CPU, writes sim/runs/<out>/scorecard.md
```

The agent then reads `scorecard.json/md` (small) — never the training logs —
decides the next config, edits flags, relaunches. Renders/filmstrips are
generated from the pulled params locally, only for rounds worth looking at.

## Open items

- **Parity block-2 regression (found 2026-07-17):** grounded-stance CPU↔MJX
  parity fails at HEAD (|Δqpos| 1.3e-4 vs the 1e-11 recorded 2026-07-13);
  airborne arithmetic is still exact (8e-12), contact counts match.
  Pre-existing, not from this round's changes. Diagnosis delegated;
  mitigation meanwhile is the standing rule (CPU referee judges).
- **Odometry**: square/circle return-to-start evals use ground-truth pose
  for the waypoint commander — legitimate in sim, impossible on hardware
  until the odometry gap (DESIGN §8) closes. Dead-reckoning versions of the
  same evals can be added by integrating commanded velocity instead.
- **Single-leg crouch depth**: 0.85 × height with 2.7 N·m knees is the
  plausible envelope; a true pistol squat is likely torque-infeasible.
- The air-circle task trains blind to obstacle contact (foot-only) — it is
  a proprioception/balance demo, not manipulation.
