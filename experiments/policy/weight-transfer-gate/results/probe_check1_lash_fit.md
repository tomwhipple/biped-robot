Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'lash_fit': True}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.32 / -1.35 / -0.30; 0.0 / -0.0 | -1.32 / -1.35 / -0.30; 0.0 / -0.0 | -1.41 / -1.45 / -0.19; 0.0 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -1.43 / -1.44 / -0.52; 19.2 / +17.7 | -1.43 / -1.44 / -0.52; 19.2 / +17.7 | -2.33 / -2.42 / -0.50; 17.6 / +17.0 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.45 / -1.46 / -0.70; 0.0 / -0.0 | -1.45 / -1.46 / -0.70; 0.0 / -0.0 | -1.06 / -1.06 / -0.41; 0.0 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -2.01 / -2.02 / -0.70; 18.3 / +16.3 | -2.01 / -2.02 / -0.70; 18.3 / +16.3 | -1.23 / -1.24 / -0.45; 18.9 / +15.1 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.43 / +1.45 / +0.70; 0.0 / -0.0 | +1.43 / +1.45 / +0.70; 0.0 / -0.0 | +2.03 / +2.03 / +0.42; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +1.81 / +1.81 / +0.80; 18.5 / +16.5 | +1.81 / +1.81 / +0.80; 18.5 / +16.5 | +4.35 / +4.38 / +0.09; 15.4 / +14.8 |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.41 (0.36) | -2.41 (0.36) | -2.08 (0.21) |
| 0.75 | -1.85 | -1.78 (0.30) | -1.78 (0.30) | -2.14 (0.25) |
| 1.00 | -1.69 | -1.74 (0.37) | -1.74 (0.37) | -1.74 (0.26) |
| 1.10 | -1.67 | -1.77 (0.37) | -1.77 (0.37) | -1.40 (0.22) |
| 1.20 | -1.45 | -1.72 (0.34) | -1.72 (0.34) | -1.33 (0.22) |
| 1.30 | -1.30 | -1.71 (0.31) | -1.71 (0.31) | -1.36 (0.22) |
| 1.40 | -1.42 | -1.76 (0.27) | -1.76 (0.27) | -1.41 (0.21) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.41 (0.35) | +2.41 (0.35) | +2.52 (0.52) |
| 0.75 | +1.75 | +1.78 (0.31) | +1.78 (0.31) | +2.33 (0.45) |
| 1.00 | +1.62 | +1.75 (0.36) | +1.75 (0.36) | +2.30 (0.40) |
| 1.10 | +1.56 | +1.77 (0.37) | +1.77 (0.37) | +1.94 (0.36) |
| 1.20 | +1.53 | +1.69 (0.34) | +1.69 (0.34) | +1.93 (0.31) |
| 1.30 | +1.35 | +1.71 (0.31) | +1.71 (0.31) | +1.93 (0.28) |
| 1.40 | +1.34 | +1.77 (0.26) | +1.77 (0.26) | +2.04 (0.24) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 4.14 (1.48 / 0.82; 24.0) | 4.14 (1.48 / 0.82; 24.0) | 3.67 (1.31 / 0.75; 24.6) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 4.17 (1.51 / 0.81; 24.0) | 4.17 (1.51 / 0.81; 24.0) | 3.51 (1.24 / 0.70; 24.9) |
