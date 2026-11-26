# The x1.5 margin line: three checks on night 35's students (2026-11-26)

Issue #137, for the planner. CPU only, plant D unless stated (`bimo_biped_v6legs_meas_sole.xml`,
`sag_L=6.4,sag_R=2.9`), `SHAPE_HZ=8`, 8 grade seeds. Students are the distilled `*_s128r24` runs; seeds 14 / 15 / 16 / 17
are the run name with no suffix / `_s1` / `_s2` / `_s3`.

1. L1's lost sidestep right is not the foot-off sag (below).
2. L1's teachers already lose it, so the distillation does not.
3. At the walking gain the robot's joints actually show (G ≈ 1.0-1.25, `2026-11-26-walking-loop-gain.md`), M1 already
   falls and L1 and L2 do not.

## 1. Mirrored sag

The 2026-11-25 morning plan found that L1 (`--loop-gain-range 1.0,1.6`) keeps the x1.5 margin but never sidesteps
right, and proposed plant D's asymmetric foot-off sag as the cause for night 3, "checked by day first, with L1 graded
on a mirrored sag". This is that check.

Mirroring the sag does not move L1's failure to the left. Side R stays at -0.03 to -0.09 m under every sag: plant D,
mirrored, none, and symmetric. Plant D reproduces the planner's numbers, so the setup matches. The loss is in the L1
policies, not in the grading plant. A sag-based night-3 arm would test nothing.

Mean dy (m) over 8 grade seeds, gain 1.0, `SHAPE_HZ=8`, `bimo_biped_v6legs_meas_sole.xml`; ideal +0.60 / -0.60, the
planner's bar for side R is <= -0.25. No student fell in any cell.

| student (seed) | D 6.4 / 2.9: L / R | mirrored 2.9 / 6.4: L / R | no sag: L / R | symmetric 4.65: L / R |
|---|---|---|---|---|
| L1 lg16 (14) | +0.32 / -0.04 | +0.31 / -0.05 | +0.32 / -0.03 | +0.32 / -0.04 |
| L1 lg16_s1 (15) | +0.30 / -0.05 | +0.24 / -0.06 | +0.30 / -0.04 | +0.31 / -0.05 |
| L1 lg16_s2 (16) | +0.29 / -0.08 | +0.30 / -0.09 | +0.29 / -0.07 | +0.29 / -0.08 |
| L1 lg16_s3 (17) | +0.07 / -0.04 | +0.10 / -0.05 | +0.09 / -0.04 | +0.10 / -0.04 |
| M1 prec97 (14) | +0.56 / -0.45 | +0.55 / -0.41 | +0.55 / -0.43 | +0.55 / -0.42 |
| M1 _s1 (15) | +0.51 / -0.41 | +0.54 / -0.42 | +0.51 / -0.41 | +0.52 / -0.42 |
| M1 _s2 (16) | +0.51 / -0.45 | +0.52 / -0.47 | +0.51 / -0.45 | +0.51 / -0.47 |
| M1 _s3 (17) | +0.48 / -0.51 | +0.49 / -0.53 | +0.48 / -0.50 | +0.49 / -0.53 |

- The sag moves any cell by at most about 0.05 m, which is seed noise.
- L1 also under-travels to the left (about +0.30 against M1's +0.50, seed 17 +0.1). So the loop-gain DR costs lateral
  travel on both sides, and the right side drops to nothing. The left/right asymmetry has to come from the policy or
  its training, not from this plant term.

### How it was run

Students are the distilled `*_s128r24` runs in `sim/runs`: `robot_legs_prec101_walk_sagdr_vol_i86_vref_lg16{,_s1,_s2,_s3}`
(L1) and `robot_legs_prec97_walk_sagdr_vol_i86_vref{,_s1,_s2,_s3}` (M1).

    RUNS=<the 8 students> ONLY="dir side L 0.15,dir side R -0.15" SHAPE_HZ=8 \
    GRADE_XML=bimo_biped_v6legs_meas_sole.xml GRADE_PLANT_TERMS="sag_L=2.9,sag_R=6.4" \
    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= WORKERS=8 \
    .venv/bin/python experiments/policy/shaper-train/grade.py 8 out.json runs

with `GRADE_PLANT_TERMS` set to `sag_L=6.4,sag_R=2.9` (D), `sag_L=2.9,sag_R=6.4` (mirrored), `sag_L=4.65,sag_R=4.65`
(symmetric), or unset (no sag).

## 2. L1's teachers (gain 1.0, side probes)

`grade.py` with `RUNS=<the teachers>` (the run names without `_s128r24`) and the same `ONLY` as above. No falls.

| teacher (seed) | L1: side L / R | M1: side L / R |
|---|---|---|
| 14 | +0.22 / -0.07 | +0.54 / -0.44 |
| 15 | +0.16 / -0.06 | +0.59 / -0.39 |
| 16 | +0.28 / -0.03 | +0.46 / -0.49 |
| 17 | +0.19 / -0.04 | +0.45 / -0.41 |

The L1 teachers never sidestep right either, and travel less to the left than their students. The loss is learned in
the teacher under `--loop-gain-range 1.0,1.6`; the distil copies it. Tonight's warm starts (W1, W2) test exactly that.

## 3. The walking probes at G = 1.25

`OUT_GAIN=1.25 ... grade_gain.py` with its seven walking probes. Falls out of 8 per probe (walk, fwd, back, side L,
side R, turn L, turn R) and the total out of 56; L2 is `robot_legs_prec102_walk_sagdr_vol_i86_vref_lg13`.

| student | 14 | 15 | 16 | 17 |
|---|---|---|---|---|
| M1 | 2 8 8 6 6 2 7 = **39** | 1 3 8 8 7 0 8 = **35** | 0 1 8 0 8 1 7 = **25** | 7 7 8 7 6 6 7 = **48** |
| L1 | 0 | 0 | 0 | 0 |
| L2 | 0 | 0 | 0 0 2 0 0 0 0 = 2 | 0 |

Travel at G = 1.25 (ideal side ±0.60 m, turn ±115°):

| student | side L dy | side R dy | turn L / R (°) |
|---|---|---|---|
| L1 (14-17) | +0.44 +0.43 +0.41 +0.38 | -0.03 -0.04 -0.04 -0.04 | +98 / -59, +95 / -116, +106 / -66, +97 / -99 |
| L2 (14-17) | +0.56 +0.38 +0.46 +0.41 | **-0.41 -0.42** -0.13 -0.37 | +139 / -97, +111 / -119, +133 / -97, +107 / -100 |
| M1 (14-17) | +0.40 +0.29 +0.33 -0.06 | -0.24 -0.43 -0.26 -0.53 | +154 / -83, +198 / -42, +78 / -45, +113 / +8 |

- M1 already fails at 1.25. Its 56/56 at 1.5 is not an artefact of grading above the robot's gain.
- L2 is clean at 1.25 (0-2 falls) and keeps the sidestep right on seeds 14, 15 and 17; it collapses between 1.25 and
  1.3. L1 holds to 1.5 but never steps right.
- Against the 11-25 pre-commitment with the gain bar moved from 1.5 to 1.25: L2 seeds 14 and 15 meet the side R, falls,
  x1.5 stand (g* 1.75 / 2.0) and compass (17 / 15) bars; seed 16 fails side R (-0.13 here, -0.10 at gain 1.0) and
  compass (13). Turn at 1.3 was not re-graded here. Whether to move the bar is the planner's call.
