Clamp refit on `grade:torso_kg=0.4855|pack=0.083|zoff_La=0` (lifted L, 30 mm). Targets: tilt 9.4, enc_ankle 1.55, enc_hip 0.55, beyond_ankle 4.05, beyond_hip 3.55 deg (beyond_hip lumps the stills' shank -> thigh 2.0).

Base plant, no term: tilt 2.66, encoders 1.46 / 0.83, beyond 0.00 / 0.00, cost 74.5

### spring: best {'ser_ankle': 17.75, 'ser_hip': 11.0} (cost 1.22)

| quantity | robot | fit | residual |
|---|---|---|---|
| tilt | 9.4 | 10.08 | +0.68 |
| enc_ankle | 1.55 | 2.00 | +0.45 |
| enc_hip | 0.55 | 0.78 | +0.23 |
| beyond_ankle | 4.05 | 3.49 | -0.56 |
| beyond_hip | 3.55 | 3.13 | -0.42 |

| lifted side, lift | tilt deg | encoders ankle / hip | beyond ankle / hip | lifted-foot lowest pad, mm |
|---|---|---|---|---|
| L, 30 mm | 10.08 | 2.00 / 0.78 | 3.49 / 3.13 | 9.7 |
| L, 15 mm | 7.50 | 1.51 / 0.58 | 2.63 / 2.35 | -0.0 |
| R, 30 mm | 10.99 | 2.17 / 0.85 | 3.77 / 3.42 | 5.7 |
| R, 15 mm | 6.94 | 1.42 / 0.53 | 2.48 / 2.12 | -0.1 |

Next best: {'ser_ankle': 17.75, 'ser_hip': 14.0} cost 1.73; {'ser_ankle': 20.0, 'ser_hip': 12.0} cost 1.84; {'ser_ankle': 17.75, 'ser_hip': 17.0} cost 2.95

### play: best {'lash_ankle': 7.0, 'lash_hip': 6.0} (cost 1.51)

| quantity | robot | fit | residual |
|---|---|---|---|
| tilt | 9.4 | 10.14 | +0.74 |
| enc_ankle | 1.55 | 2.08 | +0.53 |
| enc_hip | 0.55 | 0.83 | +0.28 |
| beyond_ankle | 4.05 | 3.50 | -0.55 |
| beyond_hip | 3.55 | 3.00 | -0.55 |

| lifted side, lift | tilt deg | encoders ankle / hip | beyond ankle / hip | lifted-foot lowest pad, mm |
|---|---|---|---|---|
| L, 30 mm | 10.14 | 2.08 / 0.83 | 3.50 / 3.00 | 10.3 |
| L, 15 mm | 9.05 | 1.47 / 0.64 | 3.50 / 3.00 | -0.1 |
| R, 30 mm | 10.41 | 2.20 / 0.91 | 3.50 / 3.00 | 10.5 |
| R, 15 mm | 8.93 | 1.41 / 0.62 | 3.50 / 3.00 | -0.1 |

Next best: {'lash_ankle': 6.0, 'lash_hip': 6.0} cost 1.70; {'lash_ankle': 6.0, 'lash_hip': 6.0} cost 1.70; {'lash_ankle': 6.0, 'lash_hip': 7.0} cost 1.78

### Against the 10-24 numbers (tilt 5.0-5.2, encoders 1.0-1.5 / 0.7-0.9)

| plant, 30 mm, L lifted | tilt | encoders ankle / hip |
|---|---|---|
| no pack (the 10-24 robot was tethered), no term | 2.48 | 1.36 / 0.79 |
| no pack + today's spring fit {'ser_ankle': 17.75, 'ser_hip': 11.0} | 9.06 | 1.77 / 0.72 |
| today's plant + round 2's 10-24 fit (ser_hip=14.2) | 5.16 | 1.43 / 0.82 |
