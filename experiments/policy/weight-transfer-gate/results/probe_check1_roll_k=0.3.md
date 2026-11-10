Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'roll_k': 0.3}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -2.19 / -2.19 / +0.03; 0.0 / -0.0 | -2.16 / -2.16 / +0.02; 0.0 / -0.0 | -2.25 / -2.26 / -0.02; 0.0 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -2.51 / -4.41 / -2.51; 87.0 / +86.8 FELL | -2.45 / -4.38 / -2.45; 87.0 / +86.7 FELL | -3.19 / -3.50 / +0.05; 19.4 / +19.3 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -2.35 / -2.39 / -0.18; 0.0 / -0.0 | -2.63 / -2.67 / -0.28; 0.0 / -0.0 | -2.03 / -2.08 / -0.40; 0.0 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -2.36 / -2.68 / -2.36; 86.8 / +86.8 FELL | -2.50 / -2.82 / -2.50; 86.7 / +86.6 FELL | -4.04 / -4.33 / -4.04; 85.0 / +84.9 FELL |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +2.43 / +2.47 / +0.22; 0.0 / -0.0 | +2.75 / +2.80 / +0.45; 0.2 / -0.0 | +2.01 / +2.05 / +0.33; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +2.31 / +2.65 / +2.31; 86.8 / +86.8 FELL | +2.33 / +2.66 / +2.33; 86.7 / +86.6 FELL | +3.04 / +3.80 / +3.04; 83.9 / +83.8 FELL |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.61 (0.37) | -2.67 (0.38) | -1.73 (0.23) |
| 0.75 | -1.85 | -1.81 (0.28) | -1.76 (0.27) | -1.62 (0.20) |
| 1.00 | -1.69 | -1.51 (0.31) | -1.44 (0.31) | -1.44 (0.18) |
| 1.10 | -1.67 | -1.92 (0.33) | -2.02 (0.29) | -1.69 (0.19) |
| 1.20 | -1.45 | +73.90 (0.16) FELL | -0.51 (0.00) FELL | -1.79 (0.21) |
| 1.30 | -1.30 | -0.32 (0.00) FELL | -0.30 (0.00) FELL | -1.81 (0.22) |
| 1.40 | -1.42 | -0.29 (0.00) FELL | -0.32 (0.00) FELL | +73.20 (0.17) FELL |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.61 (0.37) | +2.69 (0.38) | +2.32 (0.50) |
| 0.75 | +1.75 | +1.81 (0.29) | +1.79 (0.28) | +2.11 (0.43) |
| 1.00 | +1.62 | +1.52 (0.30) | +1.43 (0.31) | +1.86 (0.37) |
| 1.10 | +1.56 | +2.08 (0.31) | +2.17 (0.28) | +1.95 (0.36) |
| 1.20 | +1.53 | +0.55 (0.00) FELL | +0.51 (0.00) FELL | +1.91 (0.33) |
| 1.30 | +1.35 | +0.30 (0.00) FELL | +0.30 (0.00) FELL | +1.94 (0.29) |
| 1.40 | +1.34 | +0.28 (0.00) FELL | +0.30 (0.00) FELL | -75.72 (0.20) FELL |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 11.86 (7.21 / 2.42; 13.9) | 12.87 (7.45 / 2.40; 12.5) | 8.45 (5.59 / 2.28; 18.5) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 12.01 (7.35 / 2.41; 13.7) | 13.02 (7.60 / 2.39; 12.4) | 7.98 (5.33 / 2.12; 19.1) |
