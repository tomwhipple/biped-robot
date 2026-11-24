# The imitation reference over the command volume: travel calibration (2026-11-24)

Issue #263, open item "the reference over-travels 1.2-1.44× the command below saturation"
(finding 4 of `2026-11-22-volume-reference-foot-placement.md`). The volume reference's stride
amplitudes are fixed gains on the command: hip pitch `A = 1.4·(vx ∓ 0.028·wz)/f`, hip roll
`B = 1.1·vy/f`. They were tuned for the cardinal bands, and they give the body more travel per
cycle than the command asks for. This record adds `--mimic-volume-cal` and checks it with the
same forward kinematics as the foot-placement and turn records.

## Why the fixed gains over-travel

Over its stance half a leg's hip pitch sweeps hp0 − A → hp0 + A, with the knee at its default
and the ankle keeping the sole level. The body travels minus the stance sole's sweep. On the
robot plant that sweep is about 0.22 m per rad of hip pitch at small A, so a cycle travels
4·0.22·A, which is 1.23 × v/f at the gain 1.4. The hip-roll sweep (lead-out: one leg 2|B| → 0
outward, the other 0 → 2|B|) gives 1.44 × vy/f at the gain 1.1. Both sweeps are arcs, so the
ratio falls as the stride grows (fore-aft 1.232 → 1.191 at the 0.45 rad cap, lateral
1.443 → 1.386 at 0.15 m/s).

**The 0.72× at −0.4 m/s back is the cap, not the gain.** At 0.75 Hz the 0.45 rad hip-pitch cap
travels 0.383 m per cycle (0.287 m/s). −0.4 m/s asks for 0.533 m, and no gain gets past the cap.
The lateral cap (0.218 rad, the lead-out roll that still fits the sole-levelling ankle roll)
travels 0.277 m per cycle (0.208 m/s), so full lateral gives 0.83×. The calibration leaves both
caps alone. Reaching them needs a longer stride cap or a faster clock for fast commands. That
is a separate change: the stride would have to be feasible on the robot.

## The change: `--mimic-volume-cal` (opt-in; needs `--mimic-volume-ref`)

- **Travel tables from the plant.** At construction `walker_env.mimic_travel_cal` runs forward
  kinematics of the reference's stance leg on the env's own model and default pose, torso fixed.
  Per leg it tabulates the fore-aft travel per cycle for a in [0, 0.45] rad. For the two legs it
  tabulates the lateral travel per cycle of the lead-out roll sweep for b in [0, b_max]. Each
  table has 46 points. The ankle is set as `_mimic_ref` sets it: sole-level with the plant's
  axis signs, and the ankle roll cancels the hip roll. Both envs call it, so the tables are
  equal (parity R28).
- **Inverted in `_mimic_ref`.** Each leg's A is the amplitude whose sweep travels
  (vx ∓ 0.028·wz)/f per cycle, and B is the one that travels vy/f. Both are odd and piecewise
  linear in the tables (`mimic_cal_stride`, `np.interp` / `jp.interp`). Past a table's end they
  continue linearly with its end slope, so the volume reference's one saturation scale (and the
  turn reference's terms on it) still binds and brings the stride back to the cap.
- The stride difference 0.028·wz goes through the same table, so a turn's per-leg travel is
  calibrated too. The turn reference's yaw (|Y| = wz/(4f)) does not depend on the gains and is
  unchanged.
- With a zero command the reference is the uncalibrated one bit for bit. With a single-axis
  saturated command both land on the same capped stride (to 1e-12 rad).

## Results (forward kinematics, the robot plant)

Method as in the foot-placement and turn records: the robot plant (`robot_plant.robot_env_kwargs`,
`--mimic-lift 0.02`, sole-level, `--mimic-volume-ref`), torso fixed, 121 phases per cycle. The
body pose is composed in SE(2) over the two stance halves (R then L). The tables come from
`python tests/test_mimic_volume_cal.py`. Its "volume ref" column matches the foot-placement
record's. The recipe's plant (`bimo_biped_v6legs_meas_sole.xml`, the env built from
`night/fallback.job` + `--mimic-volume-cal` through `train_mjx.build_env_kw` and the referee's
`eval_precision.make_env`) gives the same travel tables to the bit.

Clock 0.75 Hz. x = reference travel per cycle / commanded travel; dir err = travel heading - command heading; W min = minimum stance width (home 121 mm); sat = the calibrated reference's saturation scale binds.

| heading | r | vx | vy | cmd travel mm | volume ref: x | dir err ° | W min | + cal: x | dir err ° | W min | sat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.15 | +0.036 | +0.000 | 48 | 1.23 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 0 | 0.5 | +0.120 | +0.000 | 160 | 1.22 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 0 | 1.0 | +0.240 | +0.000 | 320 | 1.19 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 45 | 0.15 | +0.026 | +0.026 | 49 | 1.34 | +4.5 | 121 | 1.00 | -0.0 | 121 |  |
| 45 | 0.5 | +0.087 | +0.087 | 163 | 1.32 | +4.1 | 121 | 1.00 | -0.2 | 121 |  |
| 45 | 1.0 | +0.173 | +0.173 | 326 | 1.11 | +3.2 | 121 | 0.99 | -0.7 | 121 |  |
| 90 | 0.15 | +0.000 | +0.037 | 50 | 1.44 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 90 | 0.5 | +0.000 | +0.125 | 167 | 1.41 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 90 | 1.0 | +0.000 | +0.250 | 333 | 0.83 | +0.0 | 121 | 0.83 | +0.0 | 121 | yes |
| 135 | 0.15 | -0.032 | +0.032 | 60 | 1.34 | -4.5 | 121 | 1.00 | +0.0 | 121 |  |
| 135 | 0.5 | -0.106 | +0.106 | 200 | 1.32 | -3.9 | 121 | 1.00 | +0.3 | 121 |  |
| 135 | 1.0 | -0.212 | +0.212 | 400 | 0.91 | -3.2 | 121 | 0.96 | +1.0 | 121 | yes |
| 180 | 0.15 | -0.060 | +0.000 | 80 | 1.23 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 180 | 0.5 | -0.200 | +0.000 | 267 | 1.20 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 180 | 1.0 | -0.400 | +0.000 | 533 | 0.72 | +0.0 | 121 | 0.72 | +0.0 | 121 | yes |
| 225 | 0.15 | -0.032 | -0.032 | 60 | 1.34 | +4.5 | 121 | 1.00 | -0.0 | 121 |  |
| 225 | 0.5 | -0.106 | -0.106 | 200 | 1.32 | +3.9 | 121 | 1.00 | -0.3 | 121 |  |
| 225 | 1.0 | -0.212 | -0.212 | 400 | 0.91 | +3.2 | 121 | 0.96 | -1.0 | 121 | yes |
| 270 | 0.15 | -0.000 | -0.037 | 50 | 1.44 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 270 | 0.5 | -0.000 | -0.125 | 167 | 1.41 | +0.0 | 121 | 1.00 | +0.0 | 121 |  |
| 270 | 1.0 | -0.000 | -0.250 | 333 | 0.83 | +0.0 | 121 | 0.83 | +0.0 | 121 | yes |
| 315 | 0.15 | +0.026 | -0.026 | 49 | 1.34 | -4.5 | 121 | 1.00 | +0.0 | 121 |  |
| 315 | 0.5 | +0.087 | -0.087 | 163 | 1.32 | -4.1 | 121 | 1.00 | +0.2 | 121 |  |
| 315 | 1.0 | +0.173 | -0.173 | 326 | 1.11 | -3.2 | 121 | 0.99 | +0.7 | 121 |  |

The 135° / 225° r = 1.0 rows saturate on the lateral cap with or without the calibration. The
calibrated stride reaches the cap later, so it keeps more of the travel (0.96× against 0.91×).
At the clocks 0.7 and 0.8 Hz every unsaturated command on the grid, with wz 0 or ±0.6 and no
turn reference, travels 0.984-1.000× within 0.91° of its heading (57-63 commands per clock).

Turning, with `--mimic-turn-ref` (yaw x = body yaw per cycle / (wz / f); speed x = chord / a constant-curvature path's chord; dir err against that path):

| vx | vy | wz | turn ref: yaw x | speed x | dir err ° | + cal: yaw x | speed x | dir err ° | sat |
|---|---|---|---|---|---|---|---|---|---|
| +0.120 | +0.000 | +0.4 | 1.00 | 1.19 | -0.5 | 1.00 | 0.98 | -0.4 |  |
| +0.120 | +0.000 | +1.0 | 1.00 | 1.04 | -3.6 | 1.00 | 0.85 | -2.3 |  |
| +0.240 | +0.000 | +1.0 | 0.90 | 0.96 | -5.2 | 1.00 | 0.85 | -1.2 |  |
| -0.200 | +0.000 | +0.4 | 1.00 | 1.17 | +0.3 | 1.00 | 0.98 | +0.2 |  |
| -0.200 | +0.000 | +1.0 | 1.00 | 1.02 | +1.9 | 1.00 | 0.85 | +1.4 |  |
| -0.400 | +0.000 | +1.0 | 0.56 | 0.68 | -16.2 | 0.66 | 0.66 | -12.7 | yes |
| +0.000 | +0.125 | +0.4 | 0.98 | 1.33 | -0.5 | 1.00 | 0.96 | -0.1 |  |
| +0.000 | +0.125 | -1.0 | 0.77 | 1.23 | +8.8 | 0.96 | 1.12 | +1.7 | yes |
| +0.000 | +0.250 | +1.0 | 0.47 | 0.66 | -20.5 | 0.60 | 0.59 | -15.7 | yes |
| +0.087 | +0.087 | +0.4 | 1.00 | 1.28 | +3.5 | 1.00 | 0.97 | -0.6 |  |
| +0.087 | +0.087 | -1.0 | 0.96 | 1.44 | +8.0 | 1.00 | 1.15 | +3.5 |  |
| -0.106 | -0.106 | +1.0 | 0.85 | 1.05 | -1.1 | 1.00 | 0.86 | -0.1 |  |
| -0.212 | -0.212 | -1.0 | 0.53 | 0.74 | +20.6 | 0.68 | 0.71 | +10.4 | yes |


## Findings

1. **Travel matches the command.** Without a turn the calibrated reference travels 1.00× the
   command everywhere below saturation (0.99× on the full forward diagonal: the tables are per axis,
   and on a diagonal the pitch and roll sweeps interact a little). The heading error drops from ±4.5° (the lateral
   swing over-travelled more than the fore-aft one) to within 1°. The stance width is unchanged:
   never below home.
2. **More of the volume is unsaturated.** The fixed gains reached the caps at 0.24 m/s fore-aft
   and 0.15 m/s lateral (0.75 Hz). The calibrated strides reach them at 0.29 and 0.21 m/s, so
   the 1.0 r forward-diagonal corners are no longer saturated.
3. **The 0.72× fast back stride is the 0.45 rad hip-pitch cap.** The calibration does not
   change it (finding 4's "if it is a calibration issue": it is not).
4. **With the turn reference.** At a 0.4 rad/s turn the chord is 0.96-0.98× a constant-curvature
   path's, where the fixed gains gave 1.17-1.33×. At a full 1 rad/s turn it is 0.85-1.15×. The
   turn reference's strides run along the toed-out soles (up to 38° at 1 rad/s), which shortens
   a forward or back chord and lengthens a diagonal against the turn. The stride difference is
   not the cause: with the hip half-spacing 0, 0.028 or 0.06 m, the chord at (0.12, 0, 1.0)
   stays at 0.85-0.87×. This is the turn reference's own geometry. It is inside the compass band
   (0.5-1.5×). Calibrating it needs a joint (A, Y) table and is not done here. The body yaw is
   unchanged (1.00× unsaturated). Saturated commands keep at least as much yaw, because the
   calibrated stride uses less of the shared scale (fast back + turn 0.56 → 0.66×, full lateral
   + turn 0.47 → 0.60×).

What this does **not** show: anything dynamic. A policy that tracks the command is still paid
by `w_mimic` 1.5 for following the reference. With the fixed gains the reference asks for
1.2-1.44× the commanded stride, against the tracking reward. Whether calibrating it helps the
compass probes' speed band (or the F>BR transition: the BR stride was 1.32×) is a question for a
training arm.

## Default path

Without `--mimic-volume-cal` the code path is untouched. A probe was built from `night/fallback.job`'s
arguments (M1: VW + `--mimic-volume-ref`; `--mjx-impl jax` on CPU) and run on origin/main
46d8a7dc and on this branch, for VW (M1 without `--mimic-volume-ref`), M1, M1 + `--mimic-turn-ref`
(T) and M1 + `--cmd-stand-band 0.0005` (M2). It covered the CPU referee env
(`eval_precision.make_env`, claim and nominal, 400 steps, 16-17 resets each, a command schedule
with diagonals and turns) and the jitted MJX `BatchedEnv` (8 envs × 200 steps, 64 auto-resets),
plus the reference on a 9-command × 2-clock × 37-phase grid in each env. obs, reward, qpos,
command, done and the reference are bit-identical, 20/20 arrays in each of the four cases. With
`--mimic-volume-cal` added to M1 or to T, only the reference and the reward differ (14/20
identical). The CPU referee env built from the run config picks up the flag (`make_env` passes
every env parameter in config.json).

Parity block R28 (`parity_test.py --mimic-volume-cal`, also run by
`tests/test_mimic_volume_cal.py`):
- The travel tables are equal in both envs (end travel 0.3828 m per cycle per leg at
  a = 0.45; lateral 0.2772 m at b = 0.2182).
- Reference CPU vs MJX on 41 commands × 2 clocks × 180 phases, with and without
  `--mimic-turn-ref`: worst 1.3e-07 rad.
- Airborne diagonal + turn, 120 steps: worst |dqpos| 9.6e-12, |dreward| 1.4e-09.
- Live: 40-step reward 118.4 vs 117.7 without.

## Left on #263

- A training arm: M1 + `--mimic-volume-cal` against M1 (or on top of the T arm if that one
  holds), graded with `COMPASS=1`. The speed band and F>BR are the probes to watch. One change
  per night.
- The turn reference's chord at full-rate turns (0.85-1.15×), if the compass turn probes show
  it.
- The stride cap for fast back commands (0.72× at −0.4 m/s, 0.75 Hz) and the lateral cap
  (0.83× at 0.25 m/s), if the volume's fast corners matter.
