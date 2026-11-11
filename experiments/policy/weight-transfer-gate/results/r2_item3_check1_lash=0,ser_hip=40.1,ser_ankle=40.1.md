Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'lash': 0.0, 'ser_hip': 40.1, 'ser_ankle': 40.1}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.92 / -1.93 / -0.14; 0.0 / -0.0 | -1.95 / -1.96 / -0.09; 0.0 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -2.34 / -2.40 / -0.16; 18.4 / +18.2 | -2.42 / -2.52 / -0.15; 18.7 / +18.2 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.66 / -1.66 / -0.55; 0.0 / -0.0 | -1.67 / -1.67 / -0.49; 0.0 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.99 / -2.00 / -0.25; 17.8 / +16.8 | -2.15 / -2.17 / -0.20; 16.9 / +16.1 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.66 / +1.66 / +0.55; 0.0 / -0.0 | +1.86 / +1.87 / +0.46; -0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.92 / +1.93 / +0.25; 18.0 / +16.9 | +3.60 / +3.94 / +0.23; 15.5 / +15.3 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | -1.91 | -1.99 (0.39) | -1.71 (0.26) |
| 0.75 | -1.85 | -1.92 (0.36) | -1.64 (0.23) |
| 1.00 | -1.69 | -1.74 (0.34) | -1.53 (0.21) |
| 1.10 | -1.67 | -1.75 (0.33) | -1.54 (0.20) |
| 1.20 | -1.45 | -1.76 (0.32) | -1.51 (0.19) |
| 1.30 | -1.30 | -1.72 (0.30) | -1.44 (0.18) |
| 1.40 | -1.42 | -1.77 (0.29) | -1.42 (0.18) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | +1.90 | +1.99 (0.39) | +2.01 (0.53) |
| 0.75 | +1.75 | +1.92 (0.36) | +1.88 (0.45) |
| 1.00 | +1.62 | +1.73 (0.34) | +1.74 (0.39) |
| 1.10 | +1.56 | +1.76 (0.33) | +1.75 (0.35) |
| 1.20 | +1.53 | +1.77 (0.32) | +1.74 (0.32) |
| 1.30 | +1.35 | +1.73 (0.30) | +1.70 (0.29) |
| 1.40 | +1.34 | +1.79 (0.28) | +1.73 (0.26) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | grade |
|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 5.07 (1.58 / 0.80; 22.8) | 4.35 (1.36 / 0.74; 23.8) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 5.14 (1.61 / 0.80; 22.7) | 4.09 (1.30 / 0.69; 24.2) |
