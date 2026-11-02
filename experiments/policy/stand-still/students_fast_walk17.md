servo: fast ({'dead15': 20.0, 'tau15': 33.0, 'dead50': 12.0, 'tau50': 0.0, 'zeta50': 0.55, 'fn50': 9.0})

## Compact

| variant | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold / dynamic (W, sim proxy) | falls x1.25 | falls x1.5 | falls push 0.4 | falls walk 0.15 -> stop | falls reg-58 dq | falls slow servo | falls gyro path x1.5 | falls x2 | falls push 0.5 | stand falls |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | 9.576 / 26.864 | 8.773 / 3.273 | 0.658 / 0.719 / 0.212 | 54.12 / 2.24 / 51.88 | 0/8 | 7/8 | 0/8 | 0/8 | 1/8 | 0/8 | 0/8 | 8/8 | 0/8 | 5/8 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | 0.088 / 0.146 | 0.024 / 0.019 | 0.005 / 0.006 / 0.010 | 3.35 / 2.56 / 0.79 | 0/8 | 0/8 | 6/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | 0.075 / 0.120 | 0.019 / 0.016 | 0.003 / 0.005 / 0.005 | 1.53 / 1.38 / 0.15 | 0/8 | 4/8 | 0/8 | 0/8 | 0/8 | 0/8 | 0/8 | 8/8 | 8/8 | 0/8 |

## Full

| variant | probe | falls | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold (W, sim proxy) | tilt max |
|---|---|---|---|---|---|---|---|
| **robot 2026-11-02 stand2** (as run) | real | 0/1 | 0.275 / 0.466 | 0.193 / 0.133 | 0.046 / 0.094 / 0.114 | 0.79 / 0.13 (motor current x V) | - |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | stand | 5/8 | 9.576 / 26.864 | 8.773 / 3.273 | 0.658 / 0.719 / 0.212 | 54.12 / 2.24 | 54.5 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | x1.25 | 0/8 | 12.535 / 37.438 | 11.549 / 4.213 | 0.624 / 0.773 / 0.384 | 64.85 / - | 15.0 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | x1.5 | 7/8 | 15.120 / 42.922 | 12.743 / 4.378 | 0.905 / 0.934 / 0.981 | 70.32 / - | 26.0 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | push 0.4 | 0/8 | 9.811 / 27.064 | 8.576 / 3.131 | 0.649 / 0.744 / 0.271 | 54.49 / - | 9.4 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | walk 0.15 -> stop | 0/8 | 9.627 / 26.708 | 8.772 / 3.247 | 0.671 / 0.727 / 0.219 | 54.20 / - | 10.8 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | reg-58 dq | 1/8 | 3.379 / 8.509 | 2.420 / 0.712 | 1.455 / 0.573 / 0.924 | 97.89 / - | 14.6 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | slow servo | 0/8 | 7.827 / 21.210 | 7.315 / 2.285 | 0.800 / 0.654 / 0.540 | 55.49 / - | 8.4 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | gyro path x1.5 | 0/8 | 9.655 / 27.191 | 8.778 / 3.340 | 0.591 / 0.690 / 0.283 | 55.48 / - | 7.0 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 32.0 |
| robot_legs_walk17_still_s128r24 :: clock 1.5 Hz | push 0.5 | 0/8 | 9.853 / 27.212 | 8.588 / 3.135 | 0.653 / 0.755 / 0.288 | 54.78 / - | 11.5 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | stand | 0/8 | 0.088 / 0.146 | 0.024 / 0.019 | 0.005 / 0.006 / 0.010 | 3.35 / 2.56 | 2.6 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | x1.25 | 0/8 | 0.107 / 0.173 | 0.029 / 0.023 | 0.006 / 0.008 / 0.013 | 3.33 / - | 2.4 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | x1.5 | 0/8 | 0.126 / 0.201 | 0.034 / 0.026 | 0.009 / 0.013 / 0.017 | 3.27 / - | 2.6 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | push 0.4 | 6/8 | 0.247 / 0.498 | 0.050 / 0.033 | 0.072 / 0.198 / 0.112 | 3.26 / - | 23.8 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | walk 0.15 -> stop | 0/8 | 0.103 / 0.183 | 0.025 / 0.019 | 0.019 / 0.041 / 0.049 | 6.02 / - | 10.9 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | reg-58 dq | 0/8 | 0.087 / 0.145 | 0.023 / 0.019 | 0.004 / 0.006 / 0.010 | 3.37 / - | 2.2 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | slow servo | 0/8 | 0.068 / 0.107 | 0.021 / 0.016 | 0.003 / 0.004 / 0.008 | 3.32 / - | 2.3 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | gyro path x1.5 | 0/8 | 0.093 / 0.155 | 0.024 / 0.020 | 0.005 / 0.006 / 0.010 | 3.38 / - | 2.2 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 49.1 |
| robot_legs_walk17_still_s128r24 :: freeze @ arm phase | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 26.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | stand | 0/8 | 0.075 / 0.120 | 0.019 / 0.016 | 0.003 / 0.005 / 0.005 | 1.53 / 1.38 | 2.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | x1.25 | 0/8 | 0.093 / 0.149 | 0.024 / 0.020 | 0.004 / 0.006 / 0.006 | 1.66 / - | 2.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | x1.5 | 4/8 | 0.133 / 0.435 | 0.040 / 0.029 | 0.008 / 0.028 / 0.014 | 1.26 / - | 23.3 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | push 0.4 | 0/8 | 0.156 / 0.375 | 0.026 / 0.031 | 0.059 / 0.155 / 0.041 | 1.50 / - | 5.4 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | walk 0.15 -> stop | 0/8 | 0.100 / 0.192 | 0.020 / 0.015 | 0.022 / 0.026 / 0.057 | 4.56 / - | 10.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | reg-58 dq | 0/8 | 0.074 / 0.120 | 0.018 / 0.016 | 0.003 / 0.005 / 0.005 | 1.52 / - | 2.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | slow servo | 0/8 | 0.058 / 0.117 | 0.018 / 0.013 | 0.003 / 0.004 / 0.005 | 1.47 / - | 2.7 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | gyro path x1.5 | 0/8 | 0.081 / 0.136 | 0.020 / 0.017 | 0.003 / 0.005 / 0.006 | 1.54 / - | 2.8 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | x2 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 37.1 |
| robot_legs_walk17_still_s128r24 :: coast -> pi | push 0.5 | 8/8 | nan / nan | nan / nan | nan / nan / nan | nan / - | 21.2 |
