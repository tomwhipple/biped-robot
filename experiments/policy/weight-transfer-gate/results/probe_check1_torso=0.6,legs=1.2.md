Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'torso': 0.6, 'legs': 1.2}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.77 / -1.78 / -0.29; 0.0 / -0.0 | -1.47 / -1.48 / -0.37; 0.0 / -0.0 | -1.75 / -1.76 / -0.21; 0.5 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -2.78 / -2.84 / -0.19; 18.2 / +18.1 | -3.60 / -3.62 / -0.03; 17.9 / +17.8 | -1.74 / -1.78 / -0.40; 20.1 / +17.9 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.40 / -1.42 / -0.67; 0.0 / -0.0 | -1.42 / -1.45 / -0.60; 0.0 / -0.0 | -1.56 / -1.59 / -0.65; 0.4 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.75 / -1.76 / -0.74; 19.0 / +16.6 | -2.31 / -2.32 / -0.64; 17.1 / +16.0 | -1.60 / -1.62 / -0.58; 20.0 / +15.9 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.38 / +1.40 / +0.65; 0.0 / -0.0 | +1.42 / +1.44 / +0.62; 0.0 / -0.0 | +1.83 / +1.85 / +0.62; -0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.51 / +1.52 / +0.58; 19.4 / +16.5 | +2.05 / +2.06 / +0.76; 17.1 / +16.3 | +2.78 / +2.82 / +0.43; 15.2 / +14.3 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.39 (0.38) | -2.57 (0.39) | -1.67 (0.32) |
| 0.75 | -1.85 | -1.80 (0.30) | -1.80 (0.31) | -1.56 (0.28) |
| 1.00 | -1.69 | -1.59 (0.37) | -1.80 (0.37) | -1.47 (0.25) |
| 1.10 | -1.67 | -1.65 (0.38) | -1.83 (0.38) | -1.45 (0.24) |
| 1.20 | -1.45 | -1.59 (0.36) | -1.81 (0.34) | -1.44 (0.23) |
| 1.30 | -1.30 | -1.56 (0.33) | -1.80 (0.30) | -1.37 (0.22) |
| 1.40 | -1.42 | -1.62 (0.31) | -1.87 (0.27) | -1.36 (0.21) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.39 (0.37) | +2.60 (0.38) | +1.69 (0.48) |
| 0.75 | +1.75 | +1.75 (0.29) | +1.66 (0.30) | +1.62 (0.42) |
| 1.00 | +1.62 | +1.60 (0.37) | +1.82 (0.37) | +1.56 (0.35) |
| 1.10 | +1.56 | +1.63 (0.37) | +1.82 (0.37) | +1.55 (0.32) |
| 1.20 | +1.53 | +1.59 (0.35) | +1.77 (0.33) | +1.56 (0.30) |
| 1.30 | +1.35 | +1.58 (0.32) | +1.79 (0.29) | +1.52 (0.27) |
| 1.40 | +1.34 | +1.62 (0.30) | +1.85 (0.26) | +1.56 (0.24) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 4.12 (1.48 / 0.81; 23.9) | 4.92 (1.53 / 0.80; 22.9) | 2.39 (1.30 / 0.78; 26.4) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 4.18 (1.52 / 0.81; 23.9) | 4.98 (1.56 / 0.80; 22.9) | 2.36 (1.29 / 0.76; 26.4) |
