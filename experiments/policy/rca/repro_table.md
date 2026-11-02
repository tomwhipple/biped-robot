| variant | plant | flail | falls | act-rate 1-2 s | 2-3 s | whole run | sat frac | gyro-x 6-13 Hz 2-3 s | tilt max |
|---|---|---|---|---|---|---|---|---|---|
| **robot, 2026-11-01** | real | yes | (tripped) | 0.114 | 0.239 | 0.814 | 0.022 | 0.203 | 67 |
| 1 as trained: shaper 10 Hz + servo lag 12 Hz | train | 0/8 | 0/8 | 0.072 | 0.073 | 0.071 | 0.000 | 0.018 | 2.5 |
| 1 as trained: shaper 10 Hz + servo lag 12 Hz | fit | 0/8 | 0/8 | 0.073 | 0.072 | 0.073 | 0.000 | 0.018 | 1.4 |
| 2 + servo dead time 60 ms (3 ticks), no other change | train | 0/8 | 0/8 | 0.070 | 0.071 | 0.069 | 0.000 | 0.017 | 2.6 |
| 2 + servo dead time 60 ms (3 ticks), no other change | fit | 0/8 | 0/8 | 0.069 | 0.069 | 0.069 | 0.000 | 0.017 | 1.4 |
| 3 + servo lag 2 Hz, no other change | train | 0/8 | 0/8 | 0.070 | 0.071 | 0.069 | 0.000 | 0.017 | 2.6 |
| 3 + servo lag 2 Hz, no other change | fit | 0/8 | 0/8 | 0.069 | 0.070 | 0.070 | 0.000 | 0.018 | 1.4 |
| 4 + set-point hysteresis 2 deg | train | 0/8 | 0/8 | 0.070 | 0.072 | 0.070 | 0.000 | 0.017 | 2.8 |
| 4 + set-point hysteresis 2 deg | fit | 0/8 | 0/8 | 0.070 | 0.070 | 0.070 | 0.000 | 0.017 | 1.5 |
| 5 + hard floor (contact tc 2 ms) | train | 0/8 | 0/8 | 0.073 | 0.074 | 0.072 | 0.000 | 0.018 | 2.7 |
| 5 + hard floor (contact tc 2 ms) | fit | 0/8 | 0/8 | 0.073 | 0.074 | 0.074 | 0.000 | 0.017 | 1.6 |
| 6 + floor friction 0.3 | train | 0/8 | 0/8 | 0.073 | 0.073 | 0.071 | 0.000 | 0.017 | 2.6 |
| 6 + floor friction 0.3 | fit | 0/8 | 0/8 | 0.073 | 0.072 | 0.073 | 0.000 | 0.017 | 1.4 |
| 7 + IMU noise x3 | train | 0/8 | 0/8 | 0.210 | 0.214 | 0.206 | 0.000 | 0.054 | 2.6 |
| 7 + IMU noise x3 | fit | 0/8 | 0/8 | 0.220 | 0.213 | 0.213 | 0.000 | 0.054 | 1.4 |
| 8 joint-fit actuator (fast bracket) | train | 0/8 | 0/8 | 0.069 | 0.070 | 0.068 | 0.000 | 0.021 | 2.7 |
| 8 joint-fit actuator (fast bracket) | fit | 0/8 | 0/8 | 0.068 | 0.068 | 0.069 | 0.000 | 0.022 | 1.6 |
| 9 MEASURED actuator (stepresp + sweep) | train | 0/8 | 0/8 | 0.070 | 0.070 | 0.122 | 0.000 | 0.022 | 7.8 |
| 9 MEASURED actuator (stepresp + sweep) | fit | 0/8 | 1/8 | 0.069 | 0.071 | 0.115 | 0.000 | 0.026 | 7.4 |
| 10 measured + reg-58 dq (19 ms late, 50 steps/s) | train | 0/8 | 2/8 | 0.074 | 0.073 | 0.131 | 0.001 | 0.027 | 14.3 |
| 10 measured + reg-58 dq (19 ms late, 50 steps/s) | fit | 1/8 | 2/8 | 0.073 | 0.087 | 0.199 | 0.001 | 0.048 | 16.2 |
| 11 measured, loop gain x1.25 | train | 0/8 | 6/8 | 0.085 | 0.250 | 0.233 | 0.002 | 0.471 | 37.9 |
| 11 measured, loop gain x1.25 | fit | 2/8 | 8/8 | 0.096 | 0.370 | 0.420 | 0.003 | 0.770 | 46.3 |
| 12 sweep-only fit (kp .47, kd .03, compliance past the encoder) | train | 8/8 | 8/8 | 2.279 | nan | 1.278 | 0.041 | nan | 55.3 |
| 12 sweep-only fit (kp .47, kd .03, compliance past the encoder) | fit | 8/8 | 8/8 | 2.322 | nan | 1.302 | 0.052 | nan | 61.6 |
| 13 measured + firmware fix: gyro obs x0.5, shaper 6 Hz | train | 0/8 | 0/8 | 0.056 | 0.055 | 0.054 | 0.000 | 0.009 | 2.7 |
| 13 measured + firmware fix: gyro obs x0.5, shaper 6 Hz | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.010 | 1.6 |
| 14 measured x1.25 + firmware fix | train | 0/8 | 1/8 | 0.056 | 0.055 | 0.065 | 0.000 | 0.010 | 8.3 |
| 14 measured x1.25 + firmware fix | fit | 0/8 | 0/8 | 0.055 | 0.055 | 0.055 | 0.000 | 0.013 | 1.6 |
