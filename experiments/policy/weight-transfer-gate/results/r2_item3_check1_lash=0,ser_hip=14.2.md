Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'lash': 0.0, 'ser_hip': 14.2}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.97 / -1.97 / -0.09; 0.0 / -0.0 | -1.80 / -1.80 / -0.20; 0.0 / -0.1 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -3.41 / -3.43 / -0.33; 17.3 / +16.6 | -3.40 / -3.42 / -0.33; 17.9 / +17.2 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.82 / -1.82 / -0.68; 0.0 / -0.0 | -1.60 / -1.60 / -0.59; 0.0 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.21 / -1.45 / -1.21; 87.0 / +87.0 FELL | -3.45 / -3.64 / -0.40; 16.3 / +15.4 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.82 / +1.82 / +0.66; 0.0 / -0.0 | +1.77 / +1.78 / +0.52; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.19 / +1.43 / +1.19; 87.0 / +87.0 FELL | +4.94 / +5.41 / +1.02; 14.6 / +14.0 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | -1.91 | -2.13 (0.41) | -1.82 (0.27) |
| 0.75 | -1.85 | -2.08 (0.42) | -1.83 (0.27) |
| 1.00 | -1.69 | -2.03 (0.42) | -1.70 (0.25) |
| 1.10 | -1.67 | -1.96 (0.41) | -1.63 (0.25) |
| 1.20 | -1.45 | -1.92 (0.39) | -1.60 (0.24) |
| 1.30 | -1.30 | -1.93 (0.34) | -1.57 (0.24) |
| 1.40 | -1.42 | -2.07 (0.29) | -1.57 (0.22) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|
| 0.50 | +1.90 | +2.14 (0.41) | +2.14 (0.53) |
| 0.75 | +1.75 | +2.09 (0.42) | +1.99 (0.45) |
| 1.00 | +1.62 | +2.05 (0.42) | +1.85 (0.39) |
| 1.10 | +1.56 | +1.96 (0.41) | +1.82 (0.35) |
| 1.20 | +1.53 | +1.92 (0.38) | +1.80 (0.32) |
| 1.30 | +1.35 | +1.95 (0.33) | +1.78 (0.29) |
| 1.40 | +1.34 | +2.09 (0.28) | +1.75 (0.27) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | grade |
|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 5.09 (1.39 / 0.79; 23.1) | 4.53 (1.22 / 0.74; 23.9) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 5.13 (1.41 / 0.79; 23.1) | 4.25 (1.16 / 0.69; 24.2) |
