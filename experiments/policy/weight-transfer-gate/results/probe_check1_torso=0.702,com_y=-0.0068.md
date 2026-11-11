Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'torso': 0.702, 'com_y': -0.0068}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.36 / -1.37 / -0.30; 0.0 / -0.0 | -1.15 / -1.18 / -0.30; 0.1 / -0.0 | -1.51 / -1.51 / -0.27; 1.6 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -1.38 / -1.41 / -0.39; 20.3 / +18.1 | -1.08 / -1.10 / -0.40; 19.7 / +17.6 | -1.51 / -1.55 / -0.57; 21.4 / +18.1 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.38 / -1.42 / -0.66; 0.2 / -0.0 | -1.41 / -1.44 / -0.63; 0.0 / -0.0 | -1.32 / -1.35 / -0.73; 1.6 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.62 / -1.63 / -0.71; 20.2 / +17.0 | -2.16 / -2.17 / -0.69; 17.8 / +16.5 | -1.37 / -1.38 / -0.82; 21.4 / +17.8 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.35 / +1.37 / +0.65; 0.0 / -0.0 | +1.39 / +1.42 / +0.64; 0.0 / -0.0 | +1.82 / +1.83 / +0.62; -0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.39 / +1.39 / +0.56; 20.2 / +16.5 | +1.91 / +1.92 / +0.82; 17.8 / +16.5 | +2.63 / +2.69 / +0.45; 15.3 / +14.4 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.29 (0.35) | -2.44 (0.36) | -1.66 (0.31) |
| 0.75 | -1.85 | -1.56 (0.27) | -1.52 (0.28) | -1.55 (0.28) |
| 1.00 | -1.69 | -1.51 (0.35) | -1.76 (0.36) | -1.42 (0.25) |
| 1.10 | -1.67 | -1.51 (0.35) | -1.73 (0.35) | -1.42 (0.24) |
| 1.20 | -1.45 | -1.51 (0.32) | -1.72 (0.30) | -1.39 (0.22) |
| 1.30 | -1.30 | -1.49 (0.29) | -1.76 (0.26) | -1.35 (0.21) |
| 1.40 | -1.42 | -1.59 (0.27) | -1.79 (0.22) | -1.35 (0.20) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.36 (0.38) | +2.54 (0.39) | +1.67 (0.47) |
| 0.75 | +1.75 | +1.74 (0.30) | +1.82 (0.31) | +1.60 (0.41) |
| 1.00 | +1.62 | +1.57 (0.37) | +1.80 (0.38) | +1.54 (0.34) |
| 1.10 | +1.56 | +1.60 (0.37) | +1.79 (0.37) | +1.54 (0.31) |
| 1.20 | +1.53 | +1.54 (0.34) | +1.72 (0.32) | +1.54 (0.28) |
| 1.30 | +1.35 | +1.55 (0.32) | +1.74 (0.28) | +1.51 (0.25) |
| 1.40 | +1.34 | +1.58 (0.29) | +1.82 (0.25) | +1.54 (0.23) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 3.93 (1.37 / 0.76; 24.3) | 4.73 (1.41 / 0.75; 23.2) | 2.18 (1.17 / 0.72; 26.7) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 3.77 (1.30 / 0.71; 24.6) | 4.57 (1.34 / 0.70; 23.5) | 1.98 (1.08 / 0.65; 27.0) |
