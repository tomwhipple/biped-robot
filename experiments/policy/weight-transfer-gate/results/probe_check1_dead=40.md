Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'dead': 40.0}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.25 / -1.29 / -0.31; 0.1 / -0.0 | -1.19 / -1.22 / -0.30; 0.0 / -0.0 | -1.55 / -1.59 / -0.25; 1.2 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -1.27 / -1.29 / -0.40; 20.3 / +18.0 | -1.18 / -1.20 / -0.42; 19.0 / +17.5 | -1.58 / -1.62 / -0.48; 21.0 / +18.1 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.41 / -1.42 / -0.65; 0.0 / -0.0 | -1.45 / -1.46 / -0.64; 0.0 / -0.0 | -1.36 / -1.39 / -0.70; 1.4 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.70 / -1.70 / -0.67; 19.8 / +16.9 | -2.29 / -2.30 / -0.62; 17.4 / +16.3 | -1.42 / -1.43 / -0.77; 21.2 / +17.6 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.40 / +1.41 / +0.65; 0.0 / -0.0 | +1.45 / +1.45 / +0.66; 0.0 / -0.0 | +1.84 / +1.85 / +0.60; -0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.47 / +1.48 / +0.54; 20.1 / +16.8 | +2.08 / +2.10 / +0.71; 17.7 / +16.5 | +2.66 / +2.78 / +0.38; 15.4 / +14.5 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.37 (0.36) | -2.58 (0.37) | -1.66 (0.30) |
| 0.75 | -1.85 | -1.61 (0.28) | -1.53 (0.28) | -1.54 (0.27) |
| 1.00 | -1.69 | -1.59 (0.36) | -1.79 (0.36) | -1.42 (0.24) |
| 1.10 | -1.67 | -1.58 (0.34) | -1.77 (0.33) | -1.43 (0.22) |
| 1.20 | -1.45 | -1.55 (0.32) | -1.73 (0.29) | -1.39 (0.21) |
| 1.30 | -1.30 | -1.53 (0.29) | -1.79 (0.25) | -1.36 (0.20) |
| 1.40 | -1.42 | -1.61 (0.27) | -1.80 (0.22) | -1.35 (0.19) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.36 (0.36) | +2.60 (0.37) | +1.69 (0.47) |
| 0.75 | +1.75 | +1.58 (0.28) | +1.53 (0.27) | +1.62 (0.40) |
| 1.00 | +1.62 | +1.60 (0.36) | +1.80 (0.36) | +1.56 (0.33) |
| 1.10 | +1.56 | +1.58 (0.34) | +1.77 (0.32) | +1.55 (0.30) |
| 1.20 | +1.53 | +1.55 (0.31) | +1.70 (0.28) | +1.56 (0.27) |
| 1.30 | +1.35 | +1.54 (0.29) | +1.77 (0.25) | +1.53 (0.24) |
| 1.40 | +1.34 | +1.60 (0.26) | +1.79 (0.21) | +1.57 (0.22) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 4.17 (1.51 / 0.82; 23.9) | 4.99 (1.56 / 0.81; 22.9) | 2.33 (1.26 / 0.76; 26.5) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 4.20 (1.53 / 0.81; 24.0) | 5.02 (1.58 / 0.81; 22.9) | 2.19 (1.20 / 0.71; 26.7) |
