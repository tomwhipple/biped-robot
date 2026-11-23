# The imitation reference over the command volume: the turn (2026-11-23)

Issue #263, open item "the turn: a hip-yaw reference (feet yawed per step about the stance hip,
lead-out too)". Finding 5 of the foot-placement record
(`2026-11-22-volume-reference-foot-placement.md`): the reference holds the hip yaws at home, so
no sole ever yaws relative to the torso and in single support the reference cannot yaw the body.
A turn there is only the stride difference (v_leg = vx ∓ 0.028·wz), which changes how far the
body travels on each stance half. This record adds `--mimic-turn-ref` and checks it by forward
kinematics over the volume. It also answers one question about the F>BR compass probe.

## The change: `--mimic-turn-ref` (opt-in; needs `--mimic-volume-ref`)

Per leg, with the clock phase φ (L swings on sin φ > 0; R stands on (0, π), L on (π, 2π)),
xn = (−cos φ, +cos φ) as for the stride, and Y = wz / (4f):

- **Sole yaw** (relative to the torso, CCW +): ψ = Y·xn + |Y|·(+1, −1). The stance sole's yaw
  sweeps back by 2Y over its half cycle, so the body yaws 2Y on each stance, 4Y = wz/f per cycle.
  The hip yaw is the leg's first joint, so the sole turns about the stance hip. The joint sign
  comes from the model (`walker_env.mimic_turn_out`, finite difference; +1 on both legs of the
  robot plant).
- **Lead foot out.** The |Y| offset makes each sole's yaw span [0, 2|Y|] toe-out. On a left turn
  L lands toe-out 2|Y| and R, at the end of its stance, has turned toe-out 2|Y| under the body.
  Then R lands parallel to L, both straight ahead. No sole ever toes in. The symmetric profile
  (no offset) toes both feet in by |Y| at every other touchdown.
- **Heel room.** In the V the heels converge. Without room the robot plant's feet overlap from a
  0.5 rad/s pivot (legs' minimum distance 14.7 / 7.3 / −0.2 / −6.9 / −27.6 mm at wz 0.3 / 0.4 /
  0.5 / 0.6 / 1.0, 0.75 Hz). The turn-side foot therefore stands out by `MIMIC_TURN_ROOM` = 0.3
  rad of hip roll per rad of its toe-out 2|Y|, for the whole cycle. It is a constant offset, so it
  causes no lateral travel. A room that rose and fell with the yaw would sweep the stance foot and
  walk the body sideways. The ankle roll cancels it, so the sole stays level.
- **Saturation.** |Y| is capped at `MIMIC_Y_MAX` = 0.35 rad. wz = 1 rad/s at 0.75 Hz (|Y| = 0.33)
  is not saturated, and the hip yaw reaches 0.70 of its ±0.785 rad policy range. The cap is a
  third term on the volume reference's single saturation scale, so a saturated command keeps its
  path's curvature. The lateral cap there becomes |B| + 0.3·|Y| ≤ 0.218 rad, because the
  lead-out roll and the room roll are both outward and must fit the sole-levelling ankle roll.
- The stride difference stays. It is the hips' own travel when the body turns.
- **With wz = 0 it is the volume reference bit for bit** (Y = 0 adds exact zeros; tested).

**Why it requires `--mimic-volume-ref`:** the turn extends that reference's lead-out and its
saturation scale. With the cardinal reference the closing foot lands on, or across, the other on
any lateral component, which would void every clearance check here. Both envs raise if it is set
alone, and so does `train_mjx`. Do not combine it with `--w-hip-yaw`, which pays against the yaws.

## Method

Forward kinematics of the reference on the recipe's plant (`bimo_biped_v6legs_meas_sole.xml`).
The env is built from `night/fallback.job` (VW: prec86 sag-DR walk + `--cmd-volume 1.0`) plus the
flags, through `train_mjx.build_env_kw`. Torso fixed, 181 phases per cycle (91 in the sweep).
Per cycle the body pose is composed in SE(2) over the two stance halves (R then L): on each half
the stance sole is fixed in the world. Leg clearance is `mj_geomDistance` between every L and
every R collision geom (soles, foot boxes, ankle blocks, shank and thigh capsules) at every phase.
Joint ranges were checked on the reference **before** its final clip.

## Results (0.75 Hz)

Yaw = body yaw per cycle. x = turn-ref yaw / commanded wz/f. Travel = the cycle's chord.
Dir err = the chord's heading minus (the command's heading + half the cycle's yaw), as a
constant-curvature path would give. Speed x = chord / (|v|/f · the arc's chord factor).
Legs min = minimum leg-to-leg distance over the cycle. Width = L−R sole centres.

| command | vx | vy | wz | cmd yaw/cycle ° | volume ref: yaw ° | turn ref: yaw ° | x | travel mm (x, y) | dir err ° | speed x | legs min mm: volume / turn | width mm (turn) | sole yaw ° L / R (turn) | max \|hip yaw\| rad |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pivot | +0.000 | +0.000 | +0.4 | +30.6 | +0.0 | +30.6 | 1.00 | +0, -2 | - | - | 34 / 33 | 147-149 | +0..+15 / -15..+0 | 0.27 |
| pivot | +0.000 | +0.000 | +1.0 | +76.4 | +0.0 | +76.4 | 1.00 | +6, -8 | - | - | 34 / 37 | 162-190 | -0..+38 / -38..+0 | 0.67 |
| pivot R | +0.000 | +0.000 | -1.0 | -76.4 | +0.0 | -76.4 | 1.00 | +11, +14 | - | - | 34 / 37 | 162-190 | +0..+38 / -38..+0 | 0.67 |
| walk | +0.120 | +0.000 | +0.4 | +30.6 | +0.0 | +30.6 | 1.00 | +182, +48 | -0.5 | 1.19 | 34 / 39 | 147-149 | -0..+15 / -15..+0 | 0.27 |
| walk | +0.120 | +0.000 | +1.0 | +76.4 | +0.0 | +76.4 | 1.00 | +127, +87 | -3.6 | 1.04 | 34 / 35 | 162-189 | -0..+38 / -38..+0 | 0.67 |
| walk fast | +0.240 | +0.000 | +1.0 | +76.4 | +0.0 | +68.7 | 0.90 | +239, +155 | -1.4 | 0.95 | 34 / 41 | 162-182 | +0..+34 / -34..+0 | 0.60 |
| back | -0.200 | +0.000 | +0.4 | +30.6 | +0.0 | +30.6 | 1.00 | -298, -83 | +0.3 | 1.17 | 34 / 40 | 146-149 | +0..+15 / -15..+0 | 0.27 |
| back | -0.200 | +0.000 | +1.0 | +76.4 | +0.0 | +76.4 | 1.00 | -193, -162 | +1.9 | 1.02 | 34 / 46 | 160-190 | -0..+38 / -38..+0 | 0.67 |
| back fast | -0.400 | +0.000 | +1.0 | +76.4 | +0.0 | +43.0 | 0.56 | -310, -125 | +0.5 | 0.64 | 34 / 43 | 153-161 | +0..+22 / -22..+0 | 0.38 |
| side L | +0.000 | +0.125 | +0.4 | +30.6 | +0.0 | +29.9 | 0.98 | -56, +211 | -0.1 | 1.33 | 34 / 40 | 147-368 | +0..+15 / -15..+0 | 0.26 |
| side L | +0.000 | +0.125 | -1.0 | -76.4 | +0.0 | -58.8 | 0.77 | +94, +166 | +0.0 | 1.19 | 34 / 33 | 161-351 | +0..+29 / -29..+0 | 0.51 |
| side L full | +0.000 | +0.250 | +1.0 | +76.4 | +0.0 | +35.7 | 0.47 | -62, +195 | -0.2 | 0.63 | 34 / 41 | 152-359 | +0..+18 / -18..+0 | 0.31 |
| side R | +0.000 | -0.125 | +1.0 | +76.4 | +0.0 | +58.8 | 0.77 | +79, -143 | -0.5 | 1.02 | 34 / 33 | 161-351 | -0..+29 / -29..-0 | 0.51 |
| diag FL | +0.087 | +0.087 | +0.4 | +30.6 | +0.0 | +30.6 | 1.00 | +92, +186 | +3.5 | 1.28 | 34 / 40 | 147-305 | +0..+15 / -15..+0 | 0.27 |
| diag FL | +0.087 | +0.087 | -1.0 | -76.4 | -0.0 | -73.2 | 0.96 | +211, +56 | +6.4 | 1.43 | 34 / 25 | 163-337 | -0..+37 / -37..+0 | 0.64 |
| diag BR | -0.106 | -0.106 | +1.0 | +76.4 | +0.0 | +65.2 | 0.85 | -27, -193 | +4.5 | 1.03 | 35 / 44 | 162-343 | +0..+33 / -33..+0 | 0.57 |
| diag BR full | -0.212 | -0.212 | -1.0 | -76.4 | -0.0 | -40.6 | 0.53 | -242, -126 | +2.7 | 0.70 | 35 / 42 | 155-348 | +0..+20 / -20..+0 | 0.35 |

**Sweep.** 616 cycles: eight headings × radii 0.15 / 0.5 / 1.0 of the envelope, plus zero speed,
each with wz ±0.15 / ±0.4 / ±0.7 / ±1.0. Plus slow walks (vx ±0.02-0.1, vy 0 / ±0.03) with
wz ±0.7 / ±1.0, which hold the closest legs. Each at clocks 0.7 and 0.8 Hz.

- **Body yaw.** 402 of 616 cycles are within 2 % of wz/f; the rest are saturated, down to 0.44×
  (full lateral + full turn). With no lateral component and |vx| ≤ 0.24 forward / 0.3 back,
  all are ≥ 0.84×. The volume reference's body yaw is 0 on every cycle (≤ 2e-15°).
- **Legs apart.** Minimum leg-to-leg distance is 14.7 mm (a 0.06 m/s walk with a full 1 rad/s
  turn at 0.7 Hz, sole to sole). The feet never cross. The volume reference keeps ≥ 34 mm.
- **Stance width.** At least 126 mm under the turn reference (home 121).
- **Joint ranges.** The reference never leaves the policy ranges before its clip (worst excursion
  5.6e-17 rad, the volume reference's ankle roll at its edge). The largest hip yaw is 0.70 rad
  against the ±0.785 rad policy range and the model's ±90°.

## Findings

1. **The reference now yaws the body, at the commanded rate.** Unsaturated, the body yaws wz/f
   per cycle (x = 1.00) for a pivot, walk + turn, back + turn, side + turn and diagonal + turn.
   A pivot stays in place (≤ 20 mm per cycle at 1 rad/s, 0.7-0.8 Hz).
2. **Travel under a turn stays in the compass band.** The chord's direction is within 6.4° of a
   constant-curvature path (worst: the forward-left diagonal with a full right turn). Speed is
   0.95-1.43× in the table where the turn is ≥ 0.9× (the over-travel of finding 4 in the
   foot-placement record carries over).
3. **No collisions, no crossing.** That needs the heel room. The V alone overlaps the feet from
   0.5 rad/s, and the room makes the turn's stance 26-69 mm wider than home.
4. **Saturation costs side + turn most.** Full lateral + full turn keeps 0.47× of the turn and
   0.63× of the travel: the lateral cap now also carries the room roll. Fast back + turn is
   limited by the hip-pitch stride, as before (0.56×).

What this does **not** show: anything dynamic. It is kinematics of the reference with the torso
fixed. Whether a policy can follow a 38° toe-out step, whether the yawing stance foot slips on
the 0.3-0.7 μ floor, and whether `w_mimic` 1.5 on the yaws helps the compass turn probes are for
a training arm. The cap and the room are tuned on the robot plant only. A different plant is
checked by `tests/test_mimic_turn_ref.py` (on `bimo_biped_v6ar.xml`) and by rerunning this method.

## The F>BR transition (diagnostic; not changed)

The planner's 11-22 report: F>BR (forward 0.15 m/s for 3 s, then back-right at −135°, no stop,
no slew) fails on 9 of 11 students. With `--mimic-volume-ref`, at the switch (same phase, 0.75 Hz,
72 switch phases):

- **Continuous.** The clock phase, which foot swings, and the swing lift all depend only on the
  phase and on "moving", which stays true. There is no foot-state change.
- **Jump.** The targets jump. The largest joint jump is 0.478 rad (hip pitch, at a touchdown:
  the fore-aft stride reverses from A = +0.28 to −0.20). The steady stride's largest per-tick
  change is 0.08 rad. The stance sole's target jumps up to 144 mm (mean over switch phases
  91 mm): at R's touchdown (−104, −99) mm, at mid-stance 51 mm sideways (the lead-out offset
  switching on). The reference's body is teleported; the planted foot cannot follow.
- **Not specific to F>BR.** F>L jumps as much (151 mm max, mean 88), and so does F>R. F>B jumps
  122 mm (mean 78). STAND>F, which opens every probe, jumps 61 mm (mean 39). The cardinal
  reference jumps 116 mm on F>BR. Training sees the same kind of jump: `--cmd-volume` commands
  step every 2.5-4.5 s between independent volume draws, and about half of them reverse the
  fore-aft sign.
- **The BR stride itself is feasible.** Heading +3.9°, 1.32× speed, width 121-319 mm, legs
  ≥ 35 mm apart.

So the reference does nothing at F>BR that it does not do at F>L or F>R. The jump is how every
command step looks in a phase-locked joint-space reference, and it is not an infeasible target
sequence. Why F>BR in particular fails is not answered here. The open candidates are the fore-aft
reversal against the body's momentum inside the 1 s settle, and the BR stride's 1.32×
over-travel. The existing remedy for command-step jumps is `--cmd-slew` (#266): it ramps the
command, and so the reference, and grade.py applies it to runs trained with it.

## Default path

Without `--mimic-turn-ref` the code path is untouched. A probe was built from `fallback.job`'s
arguments (`--mjx-impl jax` on CPU, as for PR #346) and run on origin/main 3891d6cc and on this
branch, for VW, VW + `--mimic-volume-ref` (tonight's M1) and VW + `--cmd-stand-band 0.0005` (M2).
It covered the CPU referee env (`eval_precision.make_env`, claim and nominal, 400 steps,
11 resets each) and the jitted MJX `BatchedEnv` (8 envs × 200 steps, 40 auto-resets). obs,
reward, qpos, command, done and the reference are bit-identical, 17/17 arrays in each case. With
`--mimic-turn-ref` added to M1, only the reference and the reward differ. The policy headers still
match `firmware/deployed/headers.sha256` (`make policy`).

Parity block R27 (`parity_test.py --mimic-turn-ref`, also run by
`tests/test_mimic_turn_ref.py`):
- Calibration equal in both envs.
- Reference CPU vs MJX on 43 commands × 2 clocks × 180 phases: worst 1.3e-07 rad.
- Airborne forward + turn, 120 steps: worst |dqpos| 9.6e-12, |dreward| 1.3e-09.
- Live: 40-step reward 94.7 vs 96.6 without.

## Left on #263

- A training arm: VW + `--mimic-volume-ref` + `--mimic-turn-ref` against VW +
  `--mimic-volume-ref`, graded with `COMPASS=1` (the turn and walk + turn probes are the ones to
  watch), one change per night.
- F>BR: what the students actually do at the switch (a grade.py trace), and `--cmd-slew`.
- The over-travel calibration (finding 4 of the foot-placement record).
