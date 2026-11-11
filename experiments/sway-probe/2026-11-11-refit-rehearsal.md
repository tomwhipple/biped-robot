# 2026-11-11: the sway ladder rehearsed again on the refit plant (#260)

Why: the 2026-11-10 robot measurements (`experiments/policy/weight-transfer-gate/2026-11-10-robot/README.md`:
1621 g with the pack; the stance leg yields 9.4 deg of torso tilt on a 30 mm one-leg lift, ~7 deg beyond the roll
encoders) and Round 3 of the weight-transfer gate (#259) refit the plant; PR #280 put the series spring into the
training env. Command: `sway_sim.py --refit --n 24 --chain pose --seg-ms 200` (24 plants, seed 0, the same 12 rungs).

Plant: `gate_common.MASS_SYM` (torso 0.4855 kg, 83 g pack, no L ankle-roll zero error; weight 1.6215 kg observed in the env)
and `walker_env` `roll_series_k`, hip 10 to 25 and ankle 16 to 40 N m/rad drawn per plant (Round 3's fit is 11.0 and 17.75; the stiff
end is ~2.3x it). The recipe's other DR ranges are unchanged, roll play 0.5 to 2.0 deg included (the fit itself has none). The torso mass
is not drawn.

## What moved against the earlier plant (`predictions.json`)

- Stays up: sway-only rung 5 (5.0 cm) 24 -> 22, rung 6 (5.6 cm) 22 -> 18; lift rung 12 (5.6 cm) 24 -> 23. Rungs 1 to 4 and 7 to 11: 24/24 as before.
  The safe open-loop ladder is 4.5 cm sway-only (was 5.0) and 5.0 cm with lift (was 5.6).
- Swing-foot clearance (median): sway-only rungs 0 to 5 mm (earlier 1 to 9); lift rungs 5 to 6 mm (earlier 7 to 8). The CoM crosses over the stance
  sole between 2.0 and 3.0 cm (earlier 3.0 and 3.9).
- Peak tilt (worst plant, among plants that stayed up): rungs 7 to 11 are 5.3 to 6.8 deg (earlier 5.2 to 7.2), rungs 1 to 4 0.9 to 5.5 (earlier 2.0 to 6.1). Roll rate max on those rungs 18 to 77 deg/s (earlier 37 to 82).
  So the open-loop bands did not widen on the refit plant, and the stop limits (1.5x the worst band tilt / rate, capped 12 deg and 3 rad/s) moved by 2 deg or less on every rung that stays up.
- Which plants fall (rungs 5, 6, 12): spread over the spring range (hip 11.6 to 18.8, ankle 18.0 to 37.5), not at one end; every one has the action lag at 4.8 Hz or above.

## Alternatives

Only the bench PI was rerun (`--balance stance`, rungs 4, 5, 6, 10, 11, 12), since the open-loop ladder got shorter and the PI could have overtaken it. It did not:
sway-only 4.5 / 5.0 / 5.6 cm 24, 20, 14 of 24 up (open loop 24, 22, 18); lift 4.5 / 5.0 / 5.6 cm 22, 18, 5 of 24 (open loop 24, 24, 23).
The trained `sway` skill was not rerun: it side-steps and has no amplitude knob, which the plant does not change.

## Not established

The spring range is a bracket, not a fit. The sim's open-loop tilt is the torso tilt with both feet down; the robot's extra
yield shows when one leg carries the weight, and a rung where the foot really lifts could tilt more than the sim says. The
bench PI is emulated and the sim has no goal speed (earlier record).
