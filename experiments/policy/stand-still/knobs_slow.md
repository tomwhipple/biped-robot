servo: slow ({'dead15': 68.0, 'tau15': 55.0, 'dead50': 12.0, 'tau50': 0.0, 'zeta50': 0.55, 'fn50': 9.0})

## Compact

| variant | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold / dynamic (W, sim proxy) | falls x1.25 | falls x1.5 | falls push 0.4 | falls walk 0.15 -> stop | falls reg-58 dq | falls fast servo | falls gyro path x1.5 | falls x2 | falls push 0.5 | stand falls |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | 0.189 / 0.347 | 0.155 / 0.076 | 0.029 / 0.015 / 0.009 | 1.65 / 0.63 / 1.02 | 0/8 | 1/8 | 0/8 | 7/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | 0.212 / 0.377 | 0.160 / 0.090 | 0.034 / 0.020 / 0.012 | 1.73 / 0.43 / 1.31 | 7/8 | 8/8 | 7/8 | 8/8 | 4/8 | 0/8 | 8/8 | 8/8 | 8/8 | 0/8 |
| NO knobs, freeze @ stop phase | 0.084 / 0.164 | 0.025 / 0.021 | 0.003 / 0.008 / 0.006 | 1.81 / 1.81 / -0.00 | 1/8 | 7/8 | 8/8 | 8/8 | 0/8 | 0/8 | 5/8 | 8/8 | 8/8 | 0/8 |
| NO knobs, coast -> pi | 0.066 / 0.145 | 0.018 / 0.016 | 0.003 / 0.006 / 0.005 | 1.11 / 0.98 / 0.13 | 1/8 | 4/8 | 8/8 | 8/8 | 0/8 | 0/8 | 6/8 | 8/8 | 7/8 | 0/8 |
| NO knobs, coast -> -pi/2 | 0.078 / 0.143 | 0.023 / 0.019 | 0.003 / 0.005 / 0.005 | 1.07 / 0.99 / 0.08 | 3/8 | 8/8 | 6/8 | 8/8 | 0/8 | 0/8 | 2/8 | 8/8 | 8/8 | 0/8 |
| knobs, coast -> pi | 0.044 / 0.101 | 0.015 / 0.011 | 0.002 / 0.003 / 0.003 | 1.04 / 0.87 / 0.17 | 0/8 | 0/8 | 0/8 | 8/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| knobs, coast -> -pi/2 | 0.057 / 0.112 | 0.020 / 0.014 | 0.003 / 0.003 / 0.003 | 1.02 / 0.95 / 0.07 | 0/8 | 0/8 | 0/8 | 7/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| NO knobs, gyro x0.5 only, coast -> pi | 0.054 / 0.121 | 0.016 / 0.013 | 0.002 / 0.004 / 0.004 | 1.07 / 0.90 / 0.17 | 0/8 | 0/8 | 4/8 | 8/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |

## Full

| variant | probe | falls | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold (W, sim proxy) | tilt max |
|---|---|---|---|---|---|---|---|
| **robot 2026-11-02 stand2** (as run) | real | 0/1 | 0.275 / 0.466 | 0.193 / 0.133 | 0.046 / 0.094 / 0.114 | 0.79 / 0.13 (motor current x V) | - |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | stand | 0/8 | 0.189 / 0.347 | 0.155 / 0.076 | 0.029 / 0.015 / 0.009 | 1.65 / 0.63 | 1.5 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | x1.25 | 0/8 | 0.233 / 0.438 | 0.188 / 0.096 | 0.038 / 0.025 / 0.012 | 1.69 / - | 1.6 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | x1.5 | 1/8 | 0.610 / 1.140 | 0.228 / 0.134 | 0.103 / 0.123 / 0.100 | 3.39 / - | 11.1 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | push 0.4 | 0/8 | 0.256 / 0.366 | 0.174 / 0.093 | 0.050 / 0.182 / 0.045 | 1.99 / - | 6.3 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | walk 0.15 -> stop | 7/8 | 3.712 / 6.371 | 2.366 / 0.723 | 1.072 / 0.983 / 2.036 | 75.33 / - | 50.5 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | reg-58 dq | 0/8 | 0.192 / 0.339 | 0.156 / 0.076 | 0.032 / 0.016 / 0.010 | 1.67 / - | 1.5 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | fast servo | 0/8 | 0.200 / 0.342 | 0.160 / 0.084 | 0.028 / 0.016 / 0.010 | 1.70 / - | 1.4 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | gyro path x1.5 | 0/8 | 0.188 / 0.345 | 0.152 / 0.076 | 0.028 / 0.015 / 0.010 | 1.65 / - | 1.5 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 49.3 |
| knobs (gyro x0.5, shape 6), clock 1.5 Hz | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 45.1 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | stand | 0/8 | 0.212 / 0.377 | 0.160 / 0.090 | 0.034 / 0.020 / 0.012 | 1.73 / 0.43 | 1.5 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | x1.25 | 7/8 | 3.654 / 8.496 | 0.195 / 0.123 | 0.579 / 0.602 / 0.697 | 19.69 / - | 50.2 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | x1.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.9 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | push 0.4 | 7/8 | 8.219 / 17.229 | 4.454 / 1.740 | 1.857 / 1.158 / 2.618 | 145.43 / - | 49.8 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 48.5 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | reg-58 dq | 4/8 | 0.218 / 0.364 | 0.164 / 0.090 | 0.041 / 0.024 / 0.014 | 1.80 / - | 22.6 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | fast servo | 0/8 | 0.229 / 0.366 | 0.164 / 0.097 | 0.033 / 0.021 / 0.014 | 1.82 / - | 1.5 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | gyro path x1.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 55.0 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 52.3 |
| NO knobs (gyro x1, shape 10), clock 1.5 Hz | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 45.3 |
| NO knobs, freeze @ stop phase | stand | 0/8 | 0.084 / 0.164 | 0.025 / 0.021 | 0.003 / 0.008 / 0.006 | 1.81 / 1.81 | 2.2 |
| NO knobs, freeze @ stop phase | x1.25 | 1/8 | 0.106 / 0.209 | 0.030 / 0.027 | 0.004 / 0.011 / 0.008 | 1.92 / - | 7.7 |
| NO knobs, freeze @ stop phase | x1.5 | 7/8 | 0.120 / 0.244 | 0.052 / 0.029 | 0.005 / 0.014 / 0.009 | 2.02 / - | 44.0 |
| NO knobs, freeze @ stop phase | push 0.4 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 49.4 |
| NO knobs, freeze @ stop phase | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 48.2 |
| NO knobs, freeze @ stop phase | reg-58 dq | 0/8 | 0.085 / 0.170 | 0.026 / 0.022 | 0.003 / 0.008 / 0.006 | 1.81 / - | 2.2 |
| NO knobs, freeze @ stop phase | fast servo | 0/8 | 0.104 / 0.210 | 0.027 / 0.021 | 0.004 / 0.009 / 0.007 | 1.89 / - | 2.2 |
| NO knobs, freeze @ stop phase | gyro path x1.5 | 5/8 | 0.105 / 0.253 | 0.029 / 0.027 | 0.004 / 0.014 / 0.010 | 1.92 / - | 33.5 |
| NO knobs, freeze @ stop phase | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 46.6 |
| NO knobs, freeze @ stop phase | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 45.1 |
| NO knobs, coast -> pi | stand | 0/8 | 0.066 / 0.145 | 0.018 / 0.016 | 0.003 / 0.006 / 0.005 | 1.11 / 0.98 | 2.1 |
| NO knobs, coast -> pi | x1.25 | 1/8 | 0.082 / 0.186 | 0.022 / 0.021 | 0.004 / 0.007 / 0.006 | 1.21 / - | 9.1 |
| NO knobs, coast -> pi | x1.5 | 4/8 | 0.099 / 0.229 | 0.028 / 0.023 | 0.005 / 0.009 / 0.008 | 1.34 / - | 25.9 |
| NO knobs, coast -> pi | push 0.4 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 44.7 |
| NO knobs, coast -> pi | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.4 |
| NO knobs, coast -> pi | reg-58 dq | 0/8 | 0.066 / 0.144 | 0.018 / 0.016 | 0.003 / 0.006 / 0.005 | 1.09 / - | 1.9 |
| NO knobs, coast -> pi | fast servo | 0/8 | 0.083 / 0.186 | 0.019 / 0.016 | 0.003 / 0.007 / 0.006 | 1.16 / - | 1.9 |
| NO knobs, coast -> pi | gyro path x1.5 | 6/8 | 0.082 / 0.172 | 0.023 / 0.022 | 0.003 / 0.008 / 0.008 | 1.16 / - | 35.8 |
| NO knobs, coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 42.0 |
| NO knobs, coast -> pi | push 0.5 | 7/8 | 5.235 / 11.227 | 0.935 / 1.027 | 1.881 / 1.240 / 2.824 | 140.43 / - | 48.8 |
| NO knobs, coast -> -pi/2 | stand | 0/8 | 0.078 / 0.143 | 0.023 / 0.019 | 0.003 / 0.005 / 0.005 | 1.07 / 0.99 | 2.2 |
| NO knobs, coast -> -pi/2 | x1.25 | 3/8 | 0.098 / 0.188 | 0.029 / 0.027 | 0.005 / 0.007 / 0.006 | 1.15 / - | 20.9 |
| NO knobs, coast -> -pi/2 | x1.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 48.4 |
| NO knobs, coast -> -pi/2 | push 0.4 | 6/8 | 5.714 / 10.625 | 1.637 / 1.581 | 1.828 / 1.434 / 2.484 | 140.78 / - | 45.7 |
| NO knobs, coast -> -pi/2 | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 47.1 |
| NO knobs, coast -> -pi/2 | reg-58 dq | 0/8 | 0.078 / 0.143 | 0.023 / 0.020 | 0.004 / 0.005 / 0.005 | 1.08 / - | 2.2 |
| NO knobs, coast -> -pi/2 | fast servo | 0/8 | 0.095 / 0.169 | 0.025 / 0.021 | 0.004 / 0.006 / 0.006 | 1.12 / - | 2.2 |
| NO knobs, coast -> -pi/2 | gyro path x1.5 | 2/8 | 0.092 / 0.171 | 0.026 / 0.026 | 0.004 / 0.007 / 0.007 | 1.12 / - | 13.8 |
| NO knobs, coast -> -pi/2 | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.5 |
| NO knobs, coast -> -pi/2 | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 44.8 |
| knobs, coast -> pi | stand | 0/8 | 0.044 / 0.101 | 0.015 / 0.011 | 0.002 / 0.003 / 0.003 | 1.04 / 0.87 | 2.0 |
| knobs, coast -> pi | x1.25 | 0/8 | 0.055 / 0.131 | 0.019 / 0.014 | 0.002 / 0.004 / 0.004 | 1.12 / - | 2.1 |
| knobs, coast -> pi | x1.5 | 0/8 | 0.066 / 0.165 | 0.023 / 0.017 | 0.003 / 0.005 / 0.004 | 1.21 / - | 2.4 |
| knobs, coast -> pi | push 0.4 | 0/8 | 0.126 / 0.276 | 0.022 / 0.027 | 0.053 / 0.163 / 0.037 | 1.27 / - | 7.5 |
| knobs, coast -> pi | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 49.8 |
| knobs, coast -> pi | reg-58 dq | 0/8 | 0.044 / 0.101 | 0.015 / 0.011 | 0.002 / 0.003 / 0.003 | 1.02 / - | 1.9 |
| knobs, coast -> pi | fast servo | 0/8 | 0.050 / 0.116 | 0.016 / 0.012 | 0.002 / 0.004 / 0.003 | 1.05 / - | 1.9 |
| knobs, coast -> pi | gyro path x1.5 | 0/8 | 0.048 / 0.110 | 0.016 / 0.012 | 0.002 / 0.004 / 0.004 | 1.05 / - | 1.9 |
| knobs, coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 46.6 |
| knobs, coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 47.5 |
| knobs, coast -> -pi/2 | stand | 0/8 | 0.057 / 0.112 | 0.020 / 0.014 | 0.003 / 0.003 / 0.003 | 1.02 / 0.95 | 2.2 |
| knobs, coast -> -pi/2 | x1.25 | 0/8 | 0.071 / 0.150 | 0.025 / 0.018 | 0.003 / 0.003 / 0.003 | 1.04 / - | 2.2 |
| knobs, coast -> -pi/2 | x1.5 | 0/8 | 0.085 / 0.194 | 0.030 / 0.022 | 0.005 / 0.004 / 0.003 | 1.11 / - | 2.4 |
| knobs, coast -> -pi/2 | push 0.4 | 0/8 | 0.126 / 0.288 | 0.023 / 0.027 | 0.034 / 0.156 / 0.053 | 1.14 / - | 4.6 |
| knobs, coast -> -pi/2 | walk 0.15 -> stop | 7/8 | 1.653 / 3.333 | 0.403 / 0.170 | 0.572 / 1.295 / 1.008 | 49.35 / - | 49.2 |
| knobs, coast -> -pi/2 | reg-58 dq | 0/8 | 0.057 / 0.112 | 0.020 / 0.014 | 0.003 / 0.003 / 0.003 | 1.02 / - | 2.2 |
| knobs, coast -> -pi/2 | fast servo | 0/8 | 0.064 / 0.116 | 0.022 / 0.016 | 0.003 / 0.003 / 0.003 | 1.03 / - | 2.2 |
| knobs, coast -> -pi/2 | gyro path x1.5 | 0/8 | 0.060 / 0.114 | 0.021 / 0.015 | 0.003 / 0.003 / 0.003 | 1.03 / - | 2.2 |
| knobs, coast -> -pi/2 | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 49.7 |
| knobs, coast -> -pi/2 | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.2 |
| NO knobs, gyro x0.5 only, coast -> pi | stand | 0/8 | 0.054 / 0.121 | 0.016 / 0.013 | 0.002 / 0.004 / 0.004 | 1.07 / 0.90 | 2.0 |
| NO knobs, gyro x0.5 only, coast -> pi | x1.25 | 0/8 | 0.067 / 0.157 | 0.020 / 0.016 | 0.003 / 0.005 / 0.004 | 1.17 / - | 2.1 |
| NO knobs, gyro x0.5 only, coast -> pi | x1.5 | 0/8 | 0.082 / 0.197 | 0.025 / 0.020 | 0.004 / 0.006 / 0.005 | 1.26 / - | 2.4 |
| NO knobs, gyro x0.5 only, coast -> pi | push 0.4 | 4/8 | 1.644 / 3.446 | 0.262 / 0.297 | 0.562 / 0.586 / 0.923 | 40.03 / - | 29.6 |
| NO knobs, gyro x0.5 only, coast -> pi | walk 0.15 -> stop | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.2 |
| NO knobs, gyro x0.5 only, coast -> pi | reg-58 dq | 0/8 | 0.054 / 0.121 | 0.016 / 0.013 | 0.002 / 0.004 / 0.004 | 1.05 / - | 1.9 |
| NO knobs, gyro x0.5 only, coast -> pi | fast servo | 0/8 | 0.066 / 0.152 | 0.017 / 0.014 | 0.003 / 0.005 / 0.004 | 1.10 / - | 1.9 |
| NO knobs, gyro x0.5 only, coast -> pi | gyro path x1.5 | 0/8 | 0.059 / 0.131 | 0.017 / 0.014 | 0.002 / 0.005 / 0.004 | 1.09 / - | 1.9 |
| NO knobs, gyro x0.5 only, coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 53.7 |
| NO knobs, gyro x0.5 only, coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 52.7 |
