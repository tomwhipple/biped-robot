Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml)

| test | stage | robot err peak (polled) / settled, deg | robot clearance | oneleg:fit: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash0.5: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.73 / -1.73 / -0.10; 0.0 / -0.2 | -1.23 / -1.26 / -0.31; 0.1 / -0.0 | -1.74 / -1.79 / -0.30; 0.2 / -0.0 | -1.17 / -1.20 / -0.30; 0.1 / -0.0 | -1.52 / -1.56 / -0.25; 1.3 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -2.41 / -2.45 / -0.40; 17.8 / +17.2 | -1.24 / -1.25 / -0.41; 20.4 / +18.0 | -1.87 / -1.89 / -0.34; 19.8 / +18.4 | -1.16 / -1.18 / -0.41; 19.1 / +17.5 | -1.54 / -1.58 / -0.51; 21.1 / +18.1 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.35 / -1.38 / -0.46; -0.0 / -0.2 | -1.39 / -1.40 / -0.65; 0.0 / -0.0 | -1.19 / -1.21 / -0.70; 1.4 / -0.0 | -1.45 / -1.46 / -0.64; 0.0 / -0.0 | -1.33 / -1.37 / -0.71; 1.4 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -1.55 / -1.57 / -0.61; 18.1 / +15.7 | -1.66 / -1.67 / -0.67; 19.9 / +17.0 | -1.26 / -1.26 / -0.74; 21.9 / +18.3 | -2.23 / -2.25 / -0.64; 17.5 / +16.4 | -1.39 / -1.40 / -0.80; 21.3 / +17.8 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +2.08 / +2.09 / +0.28; -0.0 / -0.2 | +1.39 / +1.40 / +0.64; 0.0 / -0.0 | +1.17 / +1.19 / +0.71; 1.5 / -0.0 | +1.44 / +1.45 / +0.66; 0.0 / -0.0 | +1.82 / +1.83 / +0.60; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +4.54 / +4.57 / -12.21; 108.3 / +15.1 ABORT FELL | +1.44 / +1.45 / +0.53; 20.2 / +16.8 | +1.15 / +1.16 / +0.68; 22.2 / +18.9 | +2.05 / +2.05 / +0.74; 17.9 / +16.6 | +2.60 / +2.72 / +0.40; 15.4 / +14.5 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | oneleg:fit (swing-foot share of the load before the lift) | train (swing-foot share of the load before the lift) | train:lash0.5 (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|---|---|
| 0.50 | -1.91 | -2.39 (0.22) | -2.38 (0.36) | -1.99 (0.35) | -2.60 (0.37) | -1.66 (0.30) |
| 0.75 | -1.85 | -2.20 (0.22) | -1.59 (0.29) | -1.75 (0.36) | -1.54 (0.28) | -1.55 (0.27) |
| 1.00 | -1.69 | -1.93 (0.25) | -1.59 (0.36) | -1.39 (0.35) | -1.79 (0.36) | -1.43 (0.24) |
| 1.10 | -1.67 | -1.63 (0.24) | -1.58 (0.34) | -1.48 (0.35) | -1.77 (0.33) | -1.43 (0.22) |
| 1.20 | -1.45 | -1.53 (0.23) | -1.55 (0.32) | -1.45 (0.34) | -1.74 (0.29) | -1.40 (0.21) |
| 1.30 | -1.30 | -1.55 (0.21) | -1.54 (0.29) | -1.39 (0.33) | -1.80 (0.25) | -1.36 (0.20) |
| 1.40 | -1.42 | -1.57 (0.19) | -1.61 (0.27) | -1.43 (0.31) | -1.81 (0.22) | -1.36 (0.19) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | oneleg:fit (swing-foot share of the load before the lift) | train (swing-foot share of the load before the lift) | train:lash0.5 (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|---|---|
| 0.50 | +1.90 | +2.44 (0.48) | +2.37 (0.36) | +1.97 (0.35) | +2.62 (0.37) | +1.69 (0.47) |
| 0.75 | +1.75 | +2.40 (0.43) | +1.60 (0.29) | +1.77 (0.36) | +1.54 (0.27) | +1.62 (0.40) |
| 1.00 | +1.62 | +2.36 (0.39) | +1.60 (0.36) | +1.40 (0.35) | +1.80 (0.36) | +1.56 (0.33) |
| 1.10 | +1.56 | +2.02 (0.34) | +1.58 (0.34) | +1.47 (0.35) | +1.77 (0.32) | +1.55 (0.30) |
| 1.20 | +1.53 | +1.99 (0.30) | +1.56 (0.31) | +1.43 (0.34) | +1.71 (0.28) | +1.56 (0.27) |
| 1.30 | +1.35 | +2.02 (0.26) | +1.55 (0.29) | +1.39 (0.33) | +1.78 (0.25) | +1.53 (0.24) |
| 1.40 | +1.34 | +2.04 (0.22) | +1.61 (0.26) | +1.43 (0.30) | +1.80 (0.21) | +1.57 (0.22) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | oneleg:fit | train | train:lash0.5 | train:lash2.0 | grade |
|---|---|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | - | 4.17 (1.51 / 0.82; 23.9) | 3.34 (1.46 / 0.82; 25.0) | 4.99 (1.56 / 0.81; 22.8) | 2.33 (1.26 / 0.76; 26.5) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | - | 4.20 (1.53 / 0.81; 23.9) | 3.38 (1.48 / 0.82; 25.0) | 5.02 (1.58 / 0.81; 22.9) | 2.19 (1.20 / 0.71; 26.7) |
