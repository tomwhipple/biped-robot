servo: fast ({'dead15': 20.0, 'tau15': 33.0, 'dead50': 12.0, 'tau50': 0.0, 'zeta50': 0.55, 'fn50': 9.0})

## Compact

| variant | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold / dynamic (W, sim proxy) | falls x1.25 | falls x1.5 | falls push 0.4 | falls walk 0.15 -> stop | falls reg-58 dq | falls slow servo | falls gyro path x1.5 | falls x2 | falls push 0.5 | stand falls |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | 0.280 / 0.459 | 0.230 / 0.083 | 0.027 / 0.034 / 0.018 | 2.65 / 1.24 / 1.41 | 0/8 | 8/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | 0.104 / 0.210 | 0.029 / 0.025 | 0.005 / 0.006 / 0.007 | 2.45 / 2.09 / 0.36 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | 0.105 / 0.253 | 0.030 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / 1.32 / 0.47 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | 0.229 / 0.366 | 0.164 / 0.097 | 0.033 / 0.021 / 0.014 | 1.82 / 0.46 / 1.37 | 1/8 | 8/8 | 0/8 | 0/8 | 1/8 | 0/8 | 0/8 | 8/8 | 0/8 | 0/8 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | 0.104 / 0.210 | 0.027 / 0.021 | 0.004 / 0.009 / 0.007 | 1.89 / 1.84 / 0.05 | 0/8 | 6/8 | 0/8 | 2/8 | 0/8 | 0/8 | 0/8 | 8/8 | 1/8 | 0/8 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | 0.083 / 0.186 | 0.019 / 0.016 | 0.003 / 0.007 / 0.006 | 1.16 / 1.02 / 0.13 | 0/8 | 3/8 | 3/8 | 3/8 | 0/8 | 0/8 | 0/8 | 8/8 | 1/8 | 0/8 |

## Full

| variant | probe | falls | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold (W, sim proxy) | tilt max |
|---|---|---|---|---|---|---|---|
| **robot 2026-11-02 stand2** (as run) | real | 0/1 | 0.275 / 0.466 | 0.193 / 0.133 | 0.046 / 0.094 / 0.114 | 0.79 / 0.13 (motor current x V) | - |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | stand | 0/8 | 0.280 / 0.459 | 0.230 / 0.083 | 0.027 / 0.034 / 0.018 | 2.65 / 1.24 | 1.4 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | x1.25 | 0/8 | 0.375 / 0.620 | 0.311 / 0.108 | 0.043 / 0.049 / 0.029 | 3.03 / - | 2.0 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | x1.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 29.4 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | push 0.4 | 0/8 | 0.340 / 0.539 | 0.261 / 0.088 | 0.053 / 0.159 / 0.048 | 2.45 / - | 6.0 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | walk 0.15 -> stop | 0/8 | 0.305 / 0.473 | 0.225 / 0.119 | 0.023 / 0.076 / 0.035 | 4.22 / - | 11.7 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | reg-58 dq | 0/8 | 0.271 / 0.449 | 0.219 / 0.080 | 0.025 / 0.033 / 0.017 | 2.61 / - | 1.4 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | slow servo | 0/8 | 0.231 / 0.353 | 0.193 / 0.065 | 0.018 / 0.024 / 0.012 | 2.46 / - | 1.1 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | gyro path x1.5 | 0/8 | 0.281 / 0.464 | 0.230 / 0.084 | 0.027 / 0.032 / 0.017 | 2.67 / - | 1.4 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 37.2 |
| robot_legs_walk16_act_s128r24 :: clock 1.5 Hz | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 22.4 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | stand | 0/8 | 0.104 / 0.210 | 0.029 / 0.025 | 0.005 / 0.006 / 0.007 | 2.45 / 2.09 | 0.8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | x1.25 | 0/8 | 0.131 / 0.264 | 0.037 / 0.031 | 0.008 / 0.009 / 0.010 | 2.59 / - | 0.7 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | x1.5 | 0/8 | 0.163 / 0.326 | 0.046 / 0.039 | 0.013 / 0.015 / 0.015 | 2.78 / - | 0.7 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | push 0.4 | 0/8 | 0.215 / 0.455 | 0.041 / 0.034 | 0.058 / 0.155 / 0.080 | 1.97 / - | 7.8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | walk 0.15 -> stop | 0/8 | 0.221 / 0.400 | 0.047 / 0.035 | 0.029 / 0.090 / 0.107 | 1.64 / - | 11.7 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | reg-58 dq | 0/8 | 0.103 / 0.210 | 0.028 / 0.024 | 0.005 / 0.006 / 0.007 | 2.46 / - | 0.8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | slow servo | 0/8 | 0.085 / 0.208 | 0.026 / 0.021 | 0.004 / 0.004 / 0.006 | 2.41 / - | 0.8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | gyro path x1.5 | 0/8 | 0.107 / 0.213 | 0.029 / 0.025 | 0.005 / 0.006 / 0.007 | 2.46 / - | 0.8 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 27.9 |
| robot_legs_walk16_act_s128r24 :: freeze @ arm phase | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 18.0 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | stand | 0/8 | 0.105 / 0.253 | 0.030 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / 1.32 | 0.8 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | x1.25 | 0/8 | 0.133 / 0.316 | 0.040 / 0.033 | 0.005 / 0.011 / 0.013 | 1.91 / - | 0.9 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | x1.5 | 0/8 | 0.166 / 0.387 | 0.050 / 0.041 | 0.008 / 0.017 / 0.019 | 2.06 / - | 1.4 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | push 0.4 | 0/8 | 0.176 / 0.324 | 0.039 / 0.037 | 0.063 / 0.157 / 0.027 | 1.80 / - | 6.2 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | walk 0.15 -> stop | 0/8 | 0.133 / 0.322 | 0.044 / 0.030 | 0.005 / 0.025 / 0.018 | 1.84 / - | 11.7 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | reg-58 dq | 0/8 | 0.104 / 0.251 | 0.029 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / - | 0.8 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | slow servo | 0/8 | 0.085 / 0.199 | 0.026 / 0.021 | 0.004 / 0.005 / 0.005 | 1.72 / - | 0.7 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | gyro path x1.5 | 0/8 | 0.107 / 0.253 | 0.031 / 0.026 | 0.003 / 0.007 / 0.008 | 1.79 / - | 0.8 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 50.3 |
| robot_legs_walk16_act_s128r24 :: coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 18.0 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | stand | 0/8 | 0.229 / 0.366 | 0.164 / 0.097 | 0.033 / 0.021 / 0.014 | 1.82 / 0.46 | 1.5 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | x1.25 | 1/8 | 0.278 / 0.449 | 0.185 / 0.118 | 0.041 / 0.036 / 0.020 | 1.94 / - | 5.3 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | x1.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 41.0 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | push 0.4 | 0/8 | 5.463 / 10.392 | 2.514 / 0.919 | 1.573 / 0.865 / 1.564 | 135.24 / - | 17.5 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | walk 0.15 -> stop | 0/8 | 5.311 / 9.766 | 3.023 / 0.964 | 1.596 / 0.863 / 1.510 | 139.08 / - | 15.6 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | reg-58 dq | 1/8 | 1.407 / 2.764 | 0.878 / 0.283 | 0.295 / 0.150 / 0.237 | 24.76 / - | 9.8 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | slow servo | 0/8 | 0.212 / 0.377 | 0.160 / 0.090 | 0.034 / 0.020 / 0.012 | 1.73 / - | 1.5 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | gyro path x1.5 | 0/8 | 1.794 / 3.225 | 0.575 / 0.279 | 0.501 / 0.257 / 0.436 | 39.66 / - | 6.0 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 38.8 |
| robot_legs_walk14_s1_s128r24 :: clock 1.5 Hz | push 0.5 | 0/8 | 4.647 / 8.081 | 1.531 / 0.668 | 1.573 / 0.854 / 1.551 | 137.72 / - | 16.4 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | stand | 0/8 | 0.104 / 0.210 | 0.027 / 0.021 | 0.004 / 0.009 / 0.007 | 1.89 / 1.84 | 2.2 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | x1.25 | 0/8 | 0.130 / 0.263 | 0.033 / 0.025 | 0.005 / 0.013 / 0.010 | 2.04 / - | 2.3 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | x1.5 | 6/8 | 0.157 / 0.319 | 0.049 / 0.027 | 0.006 / 0.021 / 0.014 | 2.25 / - | 36.1 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | push 0.4 | 0/8 | 3.448 / 6.232 | 0.799 / 0.455 | 1.199 / 0.886 / 1.352 | 113.32 / - | 14.9 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | walk 0.15 -> stop | 2/8 | 3.476 / 7.021 | 0.888 / 0.393 | 1.130 / 0.900 / 1.407 | 109.83 / - | 21.3 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | reg-58 dq | 0/8 | 0.106 / 0.215 | 0.028 / 0.022 | 0.004 / 0.009 / 0.007 | 1.89 / - | 2.2 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | slow servo | 0/8 | 0.084 / 0.164 | 0.025 / 0.021 | 0.003 / 0.008 / 0.006 | 1.81 / - | 2.2 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | gyro path x1.5 | 0/8 | 0.369 / 0.844 | 0.031 / 0.024 | 0.073 / 0.065 / 0.087 | 4.35 / - | 3.7 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 41.3 |
| robot_legs_walk14_s1_s128r24 :: freeze @ arm phase | push 0.5 | 1/8 | 3.540 / 6.492 | 0.680 / 0.457 | 1.117 / 0.890 / 1.376 | 107.46 / - | 16.1 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | stand | 0/8 | 0.083 / 0.186 | 0.019 / 0.016 | 0.003 / 0.007 / 0.006 | 1.16 / 1.02 | 2.0 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | x1.25 | 0/8 | 0.103 / 0.236 | 0.023 / 0.020 | 0.004 / 0.009 / 0.008 | 1.29 / - | 2.1 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | x1.5 | 3/8 | 0.125 / 0.288 | 0.029 / 0.027 | 0.006 / 0.012 / 0.009 | 1.40 / - | 13.4 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | push 0.4 | 3/8 | 4.529 / 9.920 | 0.992 / 0.616 | 1.733 / 0.848 / 1.618 | 150.95 / - | 29.7 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | walk 0.15 -> stop | 3/8 | 4.421 / 9.691 | 1.316 / 0.557 | 1.719 / 0.892 / 1.610 | 149.37 / - | 29.2 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | reg-58 dq | 0/8 | 0.084 / 0.188 | 0.019 / 0.017 | 0.003 / 0.007 / 0.006 | 1.14 / - | 1.9 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | slow servo | 0/8 | 0.066 / 0.145 | 0.018 / 0.016 | 0.003 / 0.006 / 0.005 | 1.11 / - | 2.0 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | gyro path x1.5 | 0/8 | 0.105 / 0.233 | 0.021 / 0.019 | 0.003 / 0.010 / 0.009 | 1.25 / - | 1.9 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 34.4 |
| robot_legs_walk14_s1_s128r24 :: coast -> pi | push 0.5 | 1/8 | 4.309 / 9.240 | 0.860 / 0.580 | 1.724 / 0.868 / 1.559 | 149.97 / - | 19.9 |
