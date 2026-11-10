Check 1, one-leg protocol on the walking plant (bimo_biped_v6legs_meas_sole.xml), terms {'lash': 2.6}

| test | stage | robot err peak (polled) / settled, deg | robot clearance | train: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | train:lash2.0: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm | grade: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm |
|---|---|---|---|---|---|---|
| floorR2 (62.0 mm, R stance) | l10 | -2.73 / -0.09 | no (Tom: did NOT clear at 10 mm) | -1.15 / -1.19 / -0.28; 0.0 / -0.0 | -1.15 / -1.19 / -0.28; 0.0 / -0.0 | -1.08 / -1.11 / -0.17; 0.1 / -0.0 |
| floorR2 (62.0 mm, R stance) | l30 | -5.21 / -0.17 | yes (frame: lifted foot visibly higher) | -1.15 / -1.17 / -0.42; 18.1 / +17.2 | -1.15 / -1.17 / -0.42; 18.1 / +17.2 | -1.06 / -1.08 / -0.40; 19.2 / +17.4 |
| clrR (80.6 mm, R stance) | l10 | -1.25 / -0.52 | not stated (NOTES' YES is the 30 mm hold) | -1.63 / -1.64 / -0.50; 0.0 / -0.0 | -1.63 / -1.64 / -0.50; 0.0 / -0.0 | -1.31 / -1.34 / -0.46; 0.0 / -0.0 |
| clrR (80.6 mm, R stance) | l30 | -3.17 / -0.31 | yes, ~1-2 cm (pixel est.) | -2.73 / -2.80 / -0.32; 17.7 / +16.1 | -2.73 / -2.80 / -0.32; 17.7 / +16.1 | -2.30 / -2.34 / -0.64; 17.3 / +16.4 |
| clrL (80.6 mm, L stance) | l10 | +1.27 / +0.35 | not stated (as R) | +1.63 / +1.65 / +0.50; 0.0 / -0.0 | +1.63 / +1.65 / +0.50; 0.0 / -0.0 | +2.55 / +2.58 / -0.04; 0.0 / -0.0 |
| clrL (80.6 mm, L stance) | l30 | +4.05 / -0.07 | yes, clear gap | +2.66 / +2.70 / +0.40; 17.7 / +16.1 | +2.66 / +2.70 / +0.40; 17.7 / +16.1 | +5.41 / +5.42 / +5.41; 88.5 / +83.1 FELL |

Unloading tilt, latR (R stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | -1.91 | -2.60 (0.38) | -2.60 (0.38) | -2.83 (0.22) |
| 0.75 | -1.85 | -1.49 (0.27) | -1.49 (0.27) | -1.83 (0.28) |
| 1.00 | -1.69 | -1.92 (0.36) | -1.92 (0.36) | -1.77 (0.36) |
| 1.10 | -1.67 | -1.91 (0.32) | -1.91 (0.32) | -1.78 (0.36) |
| 1.20 | -1.45 | -1.82 (0.27) | -1.82 (0.27) | -1.76 (0.32) |
| 1.30 | -1.30 | -1.87 (0.23) | -1.87 (0.23) | -1.78 (0.27) |
| 1.40 | -1.42 | -1.82 (0.19) | -1.82 (0.19) | -1.82 (0.22) |

Unloading tilt, latL (L stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm

| shift x | robot | train (swing-foot share of the load before the lift) | train:lash2.0 (swing-foot share of the load before the lift) | grade (swing-foot share of the load before the lift) |
|---|---|---|---|---|
| 0.50 | +1.90 | +2.64 (0.38) | +2.64 (0.38) | +3.13 (0.40) |
| 0.75 | +1.75 | +1.49 (0.27) | +1.49 (0.27) | +2.85 (0.34) |
| 1.00 | +1.62 | +1.93 (0.36) | +1.93 (0.36) | +2.72 (0.37) |
| 1.10 | +1.56 | +1.87 (0.31) | +1.87 (0.31) | +1.99 (0.34) |
| 1.20 | +1.53 | +1.80 (0.27) | +1.80 (0.27) | +1.96 (0.29) |
| 1.30 | +1.35 | +1.86 (0.23) | +1.86 (0.23) | +1.98 (0.26) |
| 1.40 | +1.34 | +1.81 (0.19) | +1.81 (0.19) | +1.99 (0.22) |

Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg (stance ankle / hip roll encoder deg; foot rise mm)

| lifted | robot | train | train:lash2.0 | grade |
|---|---|---|---|---|
| R | 5.2 (1.0-1.5 / 0.7-0.9) | 5.64 (1.60 / 0.80; 22.0) | 5.64 (1.60 / 0.80; 22.0) | 5.18 (1.42 / 0.75; 22.4) |
| L | 5.0 (1.0-1.5 / 0.7-0.9) | 5.68 (1.62 / 0.80; 22.1) | 5.68 (1.62 / 0.80; 22.1) | 5.00 (1.34 / 0.69; 22.8) |
