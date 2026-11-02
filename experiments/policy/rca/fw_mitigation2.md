| variant | plant | flail | falls | act-rate 1-2 s | 2-3 s | whole run | sat frac | gyro-x 6-13 Hz 2-3 s | tilt max |
|---|---|---|---|---|---|---|---|---|---|
| as run (gyro x1, shaper 10) :: measured, loop gain x1.5 | fit | 1/8 | 8/8 | 0.319 | 0.703 | 0.443 | 0.004 | 1.572 | 42.7 |
| as run (gyro x1, shaper 10) :: measured, loop gain x1.5 | train | 0/8 | 8/8 | 0.207 | 0.657 | 0.379 | 0.004 | 1.276 | 53.1 |
| as run (gyro x1, shaper 10) :: measured, loop gain x2 | fit | 2/8 | 8/8 | 0.602 | 0.701 | 0.451 | 0.009 | 0.946 | 58.5 |
| as run (gyro x1, shaper 10) :: measured, loop gain x2 | train | 1/8 | 8/8 | 0.553 | 0.641 | 0.403 | 0.007 | 0.973 | 57.9 |
| as run (gyro x1, shaper 10) :: measured, push 0.6 | fit | 0/8 | 7/8 | 0.069 | 0.071 | 0.328 | 0.001 | 0.032 | 44.7 |
| as run (gyro x1, shaper 10) :: measured, push 0.6 | train | 0/8 | 7/8 | 0.070 | 0.070 | 0.313 | 0.001 | 0.028 | 42.3 |
| as run (gyro x1, shaper 10) :: measured, IMU noise x2 | fit | 2/8 | 5/8 | 0.147 | 0.271 | 0.344 | 0.001 | 0.527 | 32.1 |
| as run (gyro x1, shaper 10) :: measured, IMU noise x2 | train | 2/8 | 4/8 | 0.141 | 0.253 | 0.263 | 0.001 | 0.367 | 23.1 |
| as run (gyro x1, shaper 10) :: as-trained plant (lag 12) | fit | 0/8 | 0/8 | 0.073 | 0.072 | 0.073 | 0.000 | 0.018 | 1.4 |
| as run (gyro x1, shaper 10) :: as-trained plant (lag 12) | train | 0/8 | 0/8 | 0.072 | 0.073 | 0.071 | 0.000 | 0.018 | 2.5 |
| as run (gyro x1, shaper 10) :: measured, STS3215 dead 100 tau 90 | fit | 0/8 | 0/8 | 0.069 | 0.069 | 0.069 | 0.000 | 0.022 | 1.6 |
| as run (gyro x1, shaper 10) :: measured, STS3215 dead 100 tau 90 | train | 0/8 | 1/8 | 0.070 | 0.090 | 0.085 | 0.000 | 0.046 | 8.9 |
| gyro x0.5 :: measured, loop gain x1.5 | fit | 0/8 | 6/8 | 0.058 | 0.117 | 0.151 | 0.001 | 0.101 | 45.3 |
| gyro x0.5 :: measured, loop gain x1.5 | train | 0/8 | 6/8 | 0.059 | 0.173 | 0.187 | 0.001 | 0.120 | 45.7 |
| gyro x0.5 :: measured, loop gain x2 | fit | 0/8 | 8/8 | 0.352 | 0.539 | 0.304 | 0.008 | 0.398 | 60.0 |
| gyro x0.5 :: measured, loop gain x2 | train | 0/8 | 8/8 | 0.316 | 0.511 | 0.301 | 0.004 | 0.417 | 62.3 |
| gyro x0.5 :: measured, push 0.6 | fit | 0/8 | 6/8 | 0.056 | 0.055 | 0.204 | 0.001 | 0.012 | 44.4 |
| gyro x0.5 :: measured, push 0.6 | train | 0/8 | 8/8 | 0.056 | 0.055 | 0.145 | 0.001 | 0.011 | 46.7 |
| gyro x0.5 :: measured, IMU noise x2 | fit | 0/8 | 0/8 | 0.105 | 0.103 | 0.102 | 0.000 | 0.020 | 1.6 |
| gyro x0.5 :: measured, IMU noise x2 | train | 0/8 | 0/8 | 0.104 | 0.103 | 0.101 | 0.000 | 0.020 | 2.8 |
| gyro x0.5 :: as-trained plant (lag 12) | fit | 0/8 | 0/8 | 0.056 | 0.056 | 0.056 | 0.000 | 0.009 | 1.4 |
| gyro x0.5 :: as-trained plant (lag 12) | train | 0/8 | 0/8 | 0.056 | 0.056 | 0.055 | 0.000 | 0.009 | 2.5 |
| gyro x0.5 :: measured, STS3215 dead 100 tau 90 | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.011 | 1.6 |
| gyro x0.5 :: measured, STS3215 dead 100 tau 90 | train | 0/8 | 0/8 | 0.056 | 0.055 | 0.054 | 0.000 | 0.010 | 2.7 |
| gyro x0.5 + shaper 6 :: measured, loop gain x1.5 | fit | 0/8 | 1/8 | 0.056 | 0.056 | 0.079 | 0.000 | 0.014 | 10.3 |
| gyro x0.5 + shaper 6 :: measured, loop gain x1.5 | train | 0/8 | 2/8 | 0.056 | 0.064 | 0.091 | 0.000 | 0.015 | 17.2 |
| gyro x0.5 + shaper 6 :: measured, loop gain x2 | fit | 0/8 | 8/8 | 0.108 | 0.354 | 0.258 | 0.003 | 0.318 | 57.6 |
| gyro x0.5 + shaper 6 :: measured, loop gain x2 | train | 0/8 | 8/8 | 0.103 | 0.353 | 0.229 | 0.002 | 0.288 | 56.4 |
| gyro x0.5 + shaper 6 :: measured, push 0.6 | fit | 0/8 | 8/8 | 0.056 | 0.055 | 0.077 | 0.000 | 0.010 | 50.0 |
| gyro x0.5 + shaper 6 :: measured, push 0.6 | train | 0/8 | 8/8 | 0.056 | 0.055 | 0.091 | 0.001 | 0.009 | 43.8 |
| gyro x0.5 + shaper 6 :: measured, IMU noise x2 | fit | 0/8 | 0/8 | 0.105 | 0.103 | 0.102 | 0.000 | 0.019 | 1.5 |
| gyro x0.5 + shaper 6 :: measured, IMU noise x2 | train | 0/8 | 0/8 | 0.104 | 0.103 | 0.101 | 0.000 | 0.017 | 2.7 |
| gyro x0.5 + shaper 6 :: as-trained plant (lag 12) | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.009 | 1.4 |
| gyro x0.5 + shaper 6 :: as-trained plant (lag 12) | train | 0/8 | 0/8 | 0.056 | 0.056 | 0.055 | 0.000 | 0.009 | 2.5 |
| gyro x0.5 + shaper 6 :: measured, STS3215 dead 100 tau 90 | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.009 | 1.6 |
| gyro x0.5 + shaper 6 :: measured, STS3215 dead 100 tau 90 | train | 0/8 | 0/8 | 0.056 | 0.055 | 0.054 | 0.000 | 0.009 | 2.7 |
| gyro x0.5 + shaper 4 :: measured, loop gain x1.5 | fit | 0/8 | 0/8 | 0.055 | 0.055 | 0.055 | 0.000 | 0.009 | 1.6 |
| gyro x0.5 + shaper 4 :: measured, loop gain x1.5 | train | 0/8 | 0/8 | 0.055 | 0.055 | 0.054 | 0.000 | 0.009 | 3.0 |
| gyro x0.5 + shaper 4 :: measured, loop gain x2 | fit | 0/8 | 4/8 | 0.056 | 0.057 | 0.103 | 0.000 | 0.014 | 29.4 |
| gyro x0.5 + shaper 4 :: measured, loop gain x2 | train | 0/8 | 5/8 | 0.056 | 0.071 | 0.104 | 0.001 | 0.030 | 41.1 |
| gyro x0.5 + shaper 4 :: measured, push 0.6 | fit | 0/8 | 8/8 | 0.056 | 0.055 | 0.072 | 0.000 | 0.009 | 54.7 |
| gyro x0.5 + shaper 4 :: measured, push 0.6 | train | 0/8 | 8/8 | 0.056 | 0.055 | 0.080 | 0.000 | 0.009 | 43.6 |
| gyro x0.5 + shaper 4 :: measured, IMU noise x2 | fit | 0/8 | 0/8 | 0.105 | 0.103 | 0.102 | 0.000 | 0.018 | 1.5 |
| gyro x0.5 + shaper 4 :: measured, IMU noise x2 | train | 0/8 | 0/8 | 0.104 | 0.103 | 0.101 | 0.000 | 0.017 | 2.7 |
| gyro x0.5 + shaper 4 :: as-trained plant (lag 12) | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.009 | 1.4 |
| gyro x0.5 + shaper 4 :: as-trained plant (lag 12) | train | 0/8 | 0/8 | 0.056 | 0.056 | 0.055 | 0.000 | 0.009 | 2.5 |
| gyro x0.5 + shaper 4 :: measured, STS3215 dead 100 tau 90 | fit | 0/8 | 0/8 | 0.056 | 0.055 | 0.055 | 0.000 | 0.009 | 1.6 |
| gyro x0.5 + shaper 4 :: measured, STS3215 dead 100 tau 90 | train | 0/8 | 0/8 | 0.055 | 0.055 | 0.054 | 0.000 | 0.009 | 2.7 |
