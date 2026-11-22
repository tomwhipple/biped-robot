# The imitation reference over the command volume: foot placement (2026-11-22)

Issue #263, open item "the imitation reference for diagonals and back + turn: foot placement
is not yet checked". With `--cmd-volume` (PR #338) training draws diagonal (vx, vy), back +
turn, side + turn and speeds down to zero. This record checks where the imitation reference
puts the feet for those commands, and what `--mimic-volume-ref` changes.

## What the night recipe uses

The volume jobs (prec94-96, 2026-11-21) are `night/fallback.job` (the prec86 sag-DR walk) plus
`--cmd-volume 1.0` (or 0.5). Their reference terms:

- `--w-mimic 1.5` on `_mimic_ref` (`sim/walker_env.py`, mirrored in `sim/mjx/env_mjx.py`),
  `--mimic-lift 0.02` (IK swing lift), `--mimic-sole-level` (robot preset), ankle rolls
  cancelling the hip rolls. No `--mimic-sway`, no `--mimic-dwell` (R22), no `--mimic-march`.
- `--w-feet-phase 2.0`, `--swing-height 0.025`: swing-height targets only (no x/y).
- `--gait-freq 0.7,0.8`: per-episode clock; 0.75 Hz is the deploy clock.

The reference is joint-space. Per leg, with the clock phase φ (L swings on sin φ > 0):

- fore-aft: hip pitch `hp0 − A·xn`, `A = clip(1.4·(vx ∓ 0.028·wz)/f, ±0.45)`, xn = (−cos φ, +cos φ);
- lateral: hip roll `r0 + B·xn`, `B = clip(1.1·vy/f, ±0.3)`, the ankle roll cancelling it;
- turn: only the ∓ 0.028·wz stride difference; the hip yaws stay at home.

So a diagonal is the sum of the fore-aft and lateral swings, both on the same cosine, and the
swing foot lands at the end of its half cycle where the two swings put it.

## Method

Forward kinematics of the reference on the recipe's plant (`bimo_biped_v6legs_meas_sole.xml`,
env built from `fallback.job` + `--cmd-volume 1.0` through `train_mjx.build_env_kw`), torso
fixed, 361 phases per cycle. Body travel per cycle = minus the stance soles' sweep (R stands
on φ in (0, π), L on (π, 2π)). Commands: eight headings × three radii of the envelope (forward
0.24, backward 0.4, lateral 0.25 m/s), with and without a turn, plus pivots, back + turn and
side + turn.

Clock 0.75 Hz; home stance width 121 mm, soles 84 mm wide (inner gap 37 mm). Travel = body travel per gait cycle from the reference (minus the stance soles' sweep); 'x' = reference travel / commanded travel; W = stance width (L-R sole centres), min over the cycle and at the L / R touchdowns.

| heading | r | vx | vy | cmd travel mm | cardinal ref: dir err ° | x | W min | W at L td / R td | volume ref: dir err ° | x | W min | W at L td / R td |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.15 | +0.036 | +0.000 | 48 | +0.0 | 1.23 | 121 | 121 / 121 | +0.0 | 1.23 | 121 | 121 / 121 |
| 0 | 0.5 | +0.120 | +0.000 | 160 | +0.0 | 1.22 | 121 | 121 / 121 | +0.0 | 1.22 | 121 | 121 / 121 |
| 0 | 1.0 | +0.240 | +0.000 | 320 | +0.0 | 1.19 | 121 | 121 / 121 | +0.0 | 1.19 | 121 | 121 / 121 |
| 45 | 0.15 | +0.026 | +0.026 | 49 | +4.5 | 1.34 | 96 | 146 / 96 | +4.5 | 1.34 | 121 | 171 / 121 |
| 45 | 0.5 | +0.087 | +0.087 | 163 | +4.3 | 1.33 | **39** | 203 / 39 | +4.1 | 1.32 | 121 | 284 / 121 |
| 45 | 1.0 | +0.173 | +0.173 | 326 | +3.7 | 1.30 | **-38** | 280 / -38 | +3.2 | 1.11 | 121 | 391 / 121 |
| 90 | 0.15 | +0.000 | +0.037 | 50 | +0.0 | 1.44 | 85 | 157 / 85 | +0.0 | 1.44 | 121 | 193 / 121 |
| 90 | 0.5 | +0.000 | +0.125 | 167 | +0.0 | 1.44 | **1** | 241 / 1 | +0.0 | 1.41 | 121 | 356 / 121 |
| 90 | 1.0 | +0.000 | +0.250 | 333 | +0.0 | 1.16 | **-73** | 315 / -73 | +0.0 | 0.83 | 121 | 398 / 121 |
| 135 | 0.15 | -0.032 | +0.032 | 60 | -4.5 | 1.34 | 90 | 152 / 90 | -4.5 | 1.34 | 121 | 182 / 121 |
| 135 | 0.5 | -0.106 | +0.106 | 200 | -4.2 | 1.32 | **21** | 221 / 21 | -3.9 | 1.32 | 121 | 319 / 121 |
| 135 | 1.0 | -0.212 | +0.212 | 400 | -2.3 | 1.25 | **-63** | 305 / -63 | -3.2 | 0.91 | 121 | 391 / 121 |
| 180 | 0.15 | -0.060 | +0.000 | 80 | +0.0 | 1.23 | 121 | 121 / 121 | +0.0 | 1.23 | 121 | 121 / 121 |
| 180 | 0.5 | -0.200 | +0.000 | 267 | +0.0 | 1.20 | 121 | 121 / 121 | +0.0 | 1.20 | 121 | 121 / 121 |
| 180 | 1.0 | -0.400 | +0.000 | 533 | +0.0 | 0.72 | 121 | 121 / 121 | +0.0 | 0.72 | 121 | 121 / 121 |
| 225 | 0.15 | -0.032 | -0.032 | 60 | +4.5 | 1.34 | 90 | 90 / 152 | +4.5 | 1.34 | 121 | 121 / 182 |
| 225 | 0.5 | -0.106 | -0.106 | 200 | +4.2 | 1.32 | **21** | 21 / 221 | +3.9 | 1.32 | 121 | 121 / 319 |
| 225 | 1.0 | -0.212 | -0.212 | 400 | +2.3 | 1.25 | **-63** | -63 / 305 | +3.2 | 0.91 | 121 | 121 / 391 |
| 270 | 0.15 | -0.000 | -0.037 | 50 | +0.0 | 1.44 | 85 | 85 / 157 | +0.0 | 1.44 | 121 | 121 / 193 |
| 270 | 0.5 | -0.000 | -0.125 | 167 | +0.0 | 1.44 | **1** | 1 / 241 | +0.0 | 1.41 | 121 | 121 / 356 |
| 270 | 1.0 | -0.000 | -0.250 | 333 | +0.0 | 1.16 | **-73** | -73 / 315 | +0.0 | 0.83 | 121 | 121 / 398 |
| 315 | 0.15 | +0.026 | -0.026 | 49 | -4.5 | 1.34 | 96 | 96 / 146 | -4.5 | 1.34 | 121 | 121 / 171 |
| 315 | 0.5 | +0.087 | -0.087 | 163 | -4.3 | 1.33 | **39** | 39 / 203 | -4.1 | 1.32 | 121 | 121 / 284 |
| 315 | 1.0 | +0.173 | -0.173 | 326 | -3.7 | 1.30 | **-38** | -38 / 280 | -3.2 | 1.11 | 121 | 121 / 391 |

Turning (wz rad/s). Body travel per cycle on the L and R stance halves (fore-aft, mm); the reference's sole yaw relative to the torso is 0 at every phase (hip yaw held at home).

| vx | vy | wz | cardinal ref: travel x on L / R stance | W min | volume ref: travel x on L / R stance | W min |
|---|---|---|---|---|---|---|
| 0 | 0 | 0.6 | -14 / +14 | 121 | -14 / +14 | 121 |
| 0 | 0 | 1.0 | -23 / +23 | 121 | -23 / +23 | 121 |
| 0.12 | 0 | 0.6 | +84 / +111 | 121 | +84 / +111 | 121 |
| 0.24 | 0 | 1.0 | +170 / +191 | 121 | +153 / +191 | 121 |
| -0.2 | 0 | 1.0 | -182 / -139 | 121 | -182 / -139 | 121 |
| -0.4 | 0 | 1.0 | -191 / -191 | 121 | -191 / -168 | 121 |
| 0 | 0.125 | 0.6 | -14 / +14 | 1 | -14 / +14 | 121 |
| 0 | 0.25 | 1.0 | -23 / +23 | -73 | -14 / +14 | 121 |
| -0.2 | -0.15 | -1.0 | -139 / -182 | -15 | -138 / -180 | 121 |
| 0.17 | 0.17 | 0.6 | +124 / +150 | -35 | +109 / +132 | 121 |

## Findings

1. **Heading: right.** The travel direction is within 4.5° of every commanded heading (the
   lateral swing over-travels a little more than the fore-aft one). Foot placement for a
   diagonal is the vector sum, as it should be.
2. **Stance width: wrong for any lateral component.** The legs swing −/+ about home, so the
   stance width swings ± half a cycle's lateral travel. Stepping left, L steps out at its
   touchdown and R then lands half a cycle's travel *inside* home: at |vy| ≈ 0.04 m/s (15 % of
   the lateral axis) the 84 mm soles touch (37 mm gap at home), at 0.125 m/s R lands on L's
   footprint (width 1 mm), at full lateral or a full diagonal the feet cross (−73 mm, −38 mm).
   The host gait (`experiments/roll-balance/track_host.py`) steps sideways the other way: the
   leading foot steps out by the step, the trailing foot closes to the stance width, never
   inside it. The cardinal sidestep band (|vy| 0.1-0.25) had the same defect; the volume puts
   a lateral component on most walk draws, so it now applies to diagonals at every speed.
3. **Saturation bends diagonals and drops fast back + turn.** The clips are per axis and per
   leg: at 0.75 Hz a −0.4 m/s back command saturates both legs, so back + turn has no stride
   difference left (−191 / −191 mm), and a saturating diagonal is bent toward the unsaturated
   axis.
4. **Speed: over-travels 1.2-1.44×** below saturation (fore-aft gain 1.4, lateral 1.1, tuned
   for the cardinal bands), 0.72× at −0.4 m/s back. Inside grade.py's compass band (0.5-1.5×);
   not changed here.
5. **Turning: no foot rotation.** The soles' yaw relative to the torso is 0 at every phase, so
   in single support the reference cannot yaw the body; the stride difference only changes
   how far the body travels on each stance half (a pivot shuffles ±14-23 mm fore-aft). Turning
   is learned from the tracking reward with the soles twisting on the floor, and `w_mimic`
   pays against using the hip yaws. Back + turn and side + turn get the same treatment as the
   trained forward + turn and pivot (the stride difference has the right sign going back:
   v_leg = vx ∓ 0.028·wz). Not changed here.

## The change: `--mimic-volume-ref` (opt-in)

- **Lead out, trail closes.** Each hip roll is offset by |B| outward (`mimic_lead_out` solves
  the outward sign per leg on the plant's kinematics), so a leg's roll spans [0, 2|B|]
  outward of home and the stance width spans [home, home + a cycle's lateral travel]: the
  leading foot steps out, the trailing one closes to the home width. Travel and heading are
  unchanged (a constant offset moves no stance sweep).
- **Saturation along the command.** One scale `min(1, 0.45/max|A|, b_max/|B|)` on both legs'
  A and on B keeps the heading and the back + turn stride difference (−191 / −168 mm).
- **Lateral cap b_max = 0.218 rad** on the robot plant: the lead-out roll 2|B| must fit the
  ±0.436 rad ankle roll that keeps the sole level (the hip roll has 0.785 outward). Full
  lateral therefore travels 0.83× instead of 1.16×.
- Off a lateral or saturated command the reference is unchanged (bit-equal, tested).

Pinned by `tests/test_mimic_volume_ref.py` (the grid above: width never below home, heading
within 5°, 0.65-1.5× speed; lead-out at the touchdowns; back + turn; equality off the lateral
axis) and parity block R26 (`parity_test.py --mimic-volume-ref`: CPU vs MJX reference over the
volume, airborne reward parity, liveness).

Without the option the CPU referee env (claim and nominal, 400 steps, 11 resets each) and the
jitted MJX `BatchedEnv` (8 envs × 200 steps, 40 auto-resets) built from `fallback.job`'s
arguments, with and without `--cmd-volume 1.0`, give bit-identical arrays (obs, reward, qpos,
command, done, reference) to origin/main `cacf43d3` on CPU.
With `--mimic-volume-ref` added, the same probes differ in the reference and the reward only
(obs, qpos, commands and dones stay identical under the probes' open-loop actions). The
generated policy headers still match `firmware/deployed/headers.sha256` (`make policy`).

## Left on #263

- A training arm: a volume run with `--mimic-volume-ref` against the same run without it (one
  change per night).
- The turn: a hip-yaw reference (feet yawed per step about the stance hip, lead-out too) if
  turning stays poor; and the over-travel calibration (finding 4).
