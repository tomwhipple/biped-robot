# Foot outline: inboard margin vs adduction clearance, 2026-10-07

> Dated record: accurate as of its date; the current design is [DESIGN.md](../../DESIGN.md).

## The bug

`cad/v6/foot_v6.py` `_extrude_outline` placed the sole at
`FOOT_Y_OFF - FOOT_IN .. FOOT_Y_OFF + FOOT_OUT`. `FOOT_IN` and `FOOT_OUT`
(dimensions_v6) are already measured from the ankle-roll axis, so the 12 mm
offset was applied twice. The printed `foot_L/R` and `sole_tpu_L/R` spanned
18 mm inboard / 66 outboard instead of the specified 30 / 54, with a 48 mm gap
between the feet instead of 24. The sim's sole pads and the walk gates used the
spec. Found by the Mira sim session while putting the CAD meshes into the plant.

## What the fix exposes

`check_assembly_v6` builds the real foot part. With the outline at the spec,
its interleg row fails: the walk's 3° mutual hip-roll adduction (ankles
keeping the soles flat) puts 4,840 mm³ of foot_L into foot_R. The buggy
outline is what kept that row green
([foot_outline_interleg.txt](foot_outline_interleg.txt)):

| mutual adduction | 0° | 1° | 1.5° | 2° | 2.5° | 3° |
|---|---|---|---|---|---|---|
| gap between the feet (spec outline) | 24.0 mm | 12.6 | 6.8 | 1.1 | 0 (2,076 mm³) | 0 (4,840 mm³) |

The gap closes 11.5 mm per degree. To clear 3° by 2 mm the inboard half has
to be about 23.5 mm.

## Gate D at three placements

`gate_no3250.mode_sweep`'s matrix (18 cases × 3 seeds, 8 steps, lift 4 cm) on
the robot's servo set: STS3250 at the hip rolls, the other rolls and the knees
tuned STS3215 at 2.8×, 74.5 g plant, arms. The width stays 84 mm
([foot_outline_gateD.txt](foot_outline_gateD.txt)):

| sole, inboard / outboard | feet touch at | cases OK | worst CoM margin | median |
|---|---|---|---|---|
| 30 / 54 (the spec) | 2.1° | 17/18: backlash 2° clears 14 mm (rule ≥ 15) | 17.1 mm | 35.1 mm |
| 23.5 / 60.5 | ≈ 3.2° | 18/18 | 11.5 mm | 29.6 mm |
| 18 / 66 (the buggy print) | ≈ 4.2° | 18/18 | 6.6 mm | 24.1 mm |

The worst margin in every placement is the 1° roll-play case. The margin is
inboard-limited and falls about 1 mm per mm of inboard sole removed.

## The decision

Open, for Tom. The choice is between inboard margin (the spec) and keeping
the feet apart at 3° of mutual adduction (about 23.5 mm inboard). Branch
`cad/foot-outline` carries the spec outline with the plant regenerated. Its
`run_checks` fails only the interleg foot row.

**Decided 2026-10-07: 23.5 mm inboard / 60.5 mm outboard** (`FOOT_Y_OFF` 18.5,
`foot_y_off` 0.0185 in the plant). Tom took the recommendation after seeing the
feet in CAD standing and at 3° of adduction. The angle table on the final
outline is [foot_outline_interleg_23p5.txt](foot_outline_interleg_23p5.txt).
