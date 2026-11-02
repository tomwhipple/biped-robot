| variant | stand goal jitter mean / max (deg) | body rate x / y / z | power / hold (W proxy) | squeeze / hold (deg, sum 12 joints) | x1.5 | stride1 0.6 |
|---|---|---|---|---|---|---|
| robot_legs_walk16_act_s128r24 @ shape 0 | 0.321 / 0.939 (0/0) | 0.008 / 0.017 / 0.036 | 3.99 / 3.17 | 6.19 / 4.97 | 8/8 [rate 8 pose 8 tilt 6 chatter 8] | 5/8 [rate 8 pose 8 tilt 2] (rate 2.72/2.98, dx 0.40) |
| robot_legs_walk16_act_s128r24 @ shape 20 | 0.236 / 0.646 (0/0) | 0.007 / 0.013 / 0.028 | 3.92 / 3.27 | 5.83 / 5.15 | 8/8 [rate 8 pose 8 tilt 6 chatter 5] | 1/8 [rate 8 pose 8 tilt 1] (rate 2.55/3.10, dx 0.25) |
| robot_legs_walk16_act_s128r24 @ shape 15 | 0.183 / 0.473 (0/0) | 0.006 / 0.011 / 0.024 | 3.89 / 3.43 | 5.68 / 5.28 | 7/8 [rate 8 pose 8 tilt 5 chatter 5] | 1/8 [rate 8 pose 8 tilt 1] (rate 1.86/2.76, dx 0.27) |
| robot_legs_walk16_act_s128r24 @ shape 12 | 0.145 / 0.354 (0/0) | 0.005 / 0.010 / 0.021 | 3.86 / 3.53 | 5.61 / 5.40 | 5/7 [rate 7 pose 7 tilt 4 chatter 5] | 0/8 [rate 3 pose 8] (rate 1.47/1.73, dx 0.26) |
| robot_legs_walk16_act_s128r24 @ shape 10 | 0.119 / 0.277 (0/0) | 0.004 / 0.009 / 0.018 | 3.84 / 3.60 | 5.58 / 5.48 | 5/6 [rate 6 pose 6 tilt 2 chatter 2] | 0/8 [pose 8] (rate 1.25/1.31, dx 0.28) |
| robot_legs_walk16_act_s128r24 @ shape 8 | 0.093 / 0.207 (0/0) | 0.004 / 0.008 / 0.015 | 3.83 / 3.69 | 5.56 / 5.59 | 2/3 [rate 3 pose 3 tilt 2 chatter 1] | 0/8 [pose 8] (rate 1.18/1.21, dx 0.27) |

Cells: falls/flail-guard trips out of the seeds [episodes in which each guard criterion would trip, judged separately]; walk cells add the guard's peak EMA body rate (mean/max over seeds, trip at 1.5 rad/s) and the torso's forward travel over the 2.5 s after the walk command (m).
