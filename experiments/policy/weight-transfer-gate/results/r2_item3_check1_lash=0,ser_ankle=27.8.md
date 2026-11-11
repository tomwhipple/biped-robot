Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'lash': 0.0, 'ser_ankle': 27.8}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.78 / -1.84 / -0.18; 1.2 / +0.2 | -1.90 / -1.94 / -0.10; 0.5 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -1.81 / -1.84 / -0.21; 21.2 / +20.1 | -1.92 / -2.01 / -0.17; 20.1 / +18.7 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.13 / -1.14 / -0.47; 3.3 / +2.5 | -1.37 / -1.44 / -0.59; 1.8 / +0.2 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.21 / -1.21 / -0.25; 23.5 / +21.7 | -1.39 / -1.39 / -0.34; 22.0 / +19.5 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.10 / +1.11 / +0.45; 3.5 / +2.8 | +1.97 / +1.97 / +0.47; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.17 / +1.18 / +0.24; 23.7 / +21.9 | +3.04 / +3.08 / +0.13; 16.2 / +15.8 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | -1.91 | -1.87 (0.37) | -1.69 (0.26) |
| 0.75 | -1.85 | -1.67 (0.31) | -1.60 (0.21) |
| 1.00 | -1.69 | -1.50 (0.26) | -1.45 (0.17) |
| 1.10 | -1.67 | -1.56 (0.25) | -1.50 (0.16) |
| 1.20 | -1.45 | -1.51 (0.23) | -1.45 (0.15) |
| 1.30 | -1.30 | -1.36 (0.20) | -1.38 (0.13) |
| 1.40 | -1.42 | -1.27 (0.19) | -1.37 (0.12) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | +1.90 | +1.86 (0.36) | +1.86 (0.51) |
| 0.75 | +1.75 | +1.67 (0.31) | +1.78 (0.45) |
| 1.00 | +1.62 | +1.50 (0.26) | +1.68 (0.38) |
| 1.10 | +1.56 | +1.57 (0.24) | +1.69 (0.35) |
| 1.20 | +1.53 | +1.46 (0.22) | +1.68 (0.32) |
| 1.30 | +1.35 | +1.32 (0.20) | +1.61 (0.29) |
| 1.40 | +1.34 | +1.23 (0.18) | +1.63 (0.26) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | grade |
|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 5.00 (1.68 / 0.80; 22.6) | 4.21 (1.44 / 0.74; 23.8) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 5.09 (1.71 / 0.80; 22.5) | 3.98 (1.37 / 0.69; 24.1) |
