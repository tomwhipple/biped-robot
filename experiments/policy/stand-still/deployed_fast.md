servo: fast ({'dead15': 20.0, 'tau15': 33.0, 'dead50': 12.0, 'tau50': 0.0, 'zeta50': 0.55, 'fn50': 9.0})

## Compact

| variant | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold / dynamic (W, sim proxy) | falls x1.25 | falls x1.5 | falls push 0.4 | falls walk 0.15 -> stop | falls reg-58 dq | falls slow servo | falls gyro path x1.5 | falls x2 | falls push 0.5 | stand falls |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | 0.235 / 0.385 | 0.203 / 0.064 | 0.019 / 0.026 / 0.013 | 2.42 / 1.21 / 1.21 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| walk16_act + knobs, coast -> pi | 0.080 / 0.199 | 0.027 / 0.021 | 0.002 / 0.005 / 0.005 | 1.69 / 1.32 / 0.37 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | 0.104 / 0.255 | 0.030 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / 1.31 / 0.48 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |

## Full

| variant | probe | falls | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold (W, sim proxy) | tilt max |
|---|---|---|---|---|---|---|---|
| **robot 2026-11-02 stand2** (as run) | real | 0/1 | 0.275 / 0.466 | 0.193 / 0.133 | 0.046 / 0.094 / 0.114 | 0.79 / 0.13 (motor current x V) | - |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | stand | 0/8 | 0.235 / 0.385 | 0.203 / 0.064 | 0.019 / 0.026 / 0.013 | 2.42 / 1.21 | 1.2 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | x1.25 | 0/8 | 0.297 / 0.487 | 0.256 / 0.080 | 0.027 / 0.041 / 0.019 | 2.69 / - | 1.6 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | x1.5 | 0/8 | 0.376 / 0.625 | 0.322 / 0.096 | 0.038 / 0.056 / 0.032 | 3.00 / - | 2.6 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | push 0.4 | 0/8 | 0.273 / 0.449 | 0.227 / 0.067 | 0.060 / 0.167 / 0.044 | 2.18 / - | 6.2 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | walk 0.15 -> stop | 0/8 | 0.259 / 0.420 | 0.204 / 0.092 | 0.014 / 0.070 / 0.018 | 3.78 / - | 9.1 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | reg-58 dq | 0/8 | 0.231 / 0.384 | 0.197 / 0.064 | 0.018 / 0.026 / 0.013 | 2.41 / - | 1.2 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | slow servo | 0/8 | 0.206 / 0.319 | 0.179 / 0.054 | 0.014 / 0.018 / 0.010 | 2.32 / - | 1.0 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | gyro path x1.5 | 0/8 | 0.235 / 0.387 | 0.203 / 0.064 | 0.019 / 0.025 / 0.013 | 2.42 / - | 1.2 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 60.2 |
| walk16_act + knobs (gyro x0.5, shape 6), clock 1.5 Hz | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 21.5 |
| walk16_act + knobs, coast -> pi | stand | 0/8 | 0.080 / 0.199 | 0.027 / 0.021 | 0.002 / 0.005 / 0.005 | 1.69 / 1.32 | 0.7 |
| walk16_act + knobs, coast -> pi | x1.25 | 0/8 | 0.100 / 0.247 | 0.034 / 0.025 | 0.003 / 0.007 / 0.008 | 1.77 / - | 0.8 |
| walk16_act + knobs, coast -> pi | x1.5 | 0/8 | 0.122 / 0.295 | 0.042 / 0.030 | 0.004 / 0.010 / 0.011 | 1.85 / - | 1.1 |
| walk16_act + knobs, coast -> pi | push 0.4 | 0/8 | 0.116 / 0.218 | 0.031 / 0.027 | 0.064 / 0.161 / 0.020 | 1.64 / - | 6.1 |
| walk16_act + knobs, coast -> pi | walk 0.15 -> stop | 0/8 | 0.101 / 0.264 | 0.037 / 0.022 | 0.004 / 0.016 / 0.015 | 1.58 / - | 9.1 |
| walk16_act + knobs, coast -> pi | reg-58 dq | 0/8 | 0.080 / 0.198 | 0.026 / 0.021 | 0.002 / 0.005 / 0.005 | 1.69 / - | 0.8 |
| walk16_act + knobs, coast -> pi | slow servo | 0/8 | 0.070 / 0.167 | 0.025 / 0.017 | 0.003 / 0.004 / 0.004 | 1.66 / - | 0.7 |
| walk16_act + knobs, coast -> pi | gyro path x1.5 | 0/8 | 0.080 / 0.198 | 0.027 / 0.021 | 0.002 / 0.005 / 0.005 | 1.69 / - | 0.7 |
| walk16_act + knobs, coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 60.1 |
| walk16_act + knobs, coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 21.4 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | stand | 0/8 | 0.104 / 0.255 | 0.030 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / 1.31 | 0.8 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | x1.25 | 0/8 | 0.132 / 0.319 | 0.039 / 0.033 | 0.005 / 0.011 / 0.013 | 1.90 / - | 0.9 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | x1.5 | 0/8 | 0.164 / 0.392 | 0.050 / 0.041 | 0.008 / 0.017 / 0.019 | 2.05 / - | 1.5 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | push 0.4 | 0/8 | 0.145 / 0.277 | 0.035 / 0.034 | 0.065 / 0.161 / 0.025 | 1.72 / - | 6.2 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | walk 0.15 -> stop | 0/8 | 0.126 / 0.319 | 0.040 / 0.028 | 0.006 / 0.023 / 0.016 | 1.55 / - | 16.2 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | reg-58 dq | 0/8 | 0.103 / 0.253 | 0.029 / 0.026 | 0.003 / 0.007 / 0.008 | 1.78 / - | 0.8 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | slow servo | 0/8 | 0.084 / 0.199 | 0.027 / 0.021 | 0.004 / 0.005 / 0.005 | 1.72 / - | 0.7 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | gyro path x1.5 | 0/8 | 0.104 / 0.253 | 0.030 / 0.026 | 0.003 / 0.007 / 0.008 | 1.78 / - | 0.8 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 56.3 |
| walk16_act, gyro x0.5 only (shape 10), coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 18.9 |
