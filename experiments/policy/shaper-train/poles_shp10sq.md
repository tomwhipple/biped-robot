| variant | stand goal jitter mean / max (deg) | body rate x / y / z | power / hold (W proxy) | squeeze / hold (deg, sum 12 joints) | x1.5 | walk 0.3 -> stop | stride1 0.6 |
|---|---|---|---|---|---|---|---|
| robot_legs_walk18_shp10sq_s128r24 @ shape 0 | 2.805 / 10.121 (3/5) | 0.068 / 0.107 / 0.381 | 27.57 / 0.83 | 29.53 / 1.83 | 8/8 [rate 8 pose 8 tilt 3 chatter 8] | 0/8 [chatter 8] (rate 1.28/1.33, dx 0.62) | 0/8 [chatter 8] (rate 1.29/1.43, dx 0.22) |
| robot_legs_walk18_shp10sq_s128r24 @ shape 20 | 0.298 / 0.765 (0/0) | 0.002 / 0.009 / 0.006 | 0.76 / 0.38 | 3.47 / 1.49 | 8/8 [rate 8 pose 8 chatter 8] | 0/4 [chatter 4] (rate 1.09/1.30, dx 0.62) | 0/7 [chatter 7] (rate 0.91/1.01, dx 0.22) |
| robot_legs_walk18_shp10sq_s128r24 @ shape 15 | 0.218 / 0.534 (0/0) | 0.002 / 0.007 / 0.005 | 0.66 / 0.28 | 2.79 / 1.32 | 8/8 [rate 8 pose 8 tilt 4 chatter 8] | 0/0 (rate 0.90/1.06, dx 0.63) | 0/0 (rate 0.80/0.87, dx 0.22) |
| robot_legs_walk18_shp10sq_s128r24 @ shape 12 | 0.163 / 0.380 (0/0) | 0.002 / 0.006 / 0.004 | 0.61 / 0.26 | 2.33 / 1.29 | 3/3 [rate 3 pose 3 tilt 1 chatter 2] | 0/0 (rate 0.81/0.84, dx 0.62) | 0/0 (rate 0.78/0.80, dx 0.23) |
| robot_legs_walk18_shp10sq_s128r24 @ shape 10 | 0.126 / 0.280 (0/0) | 0.002 / 0.005 / 0.003 | 0.58 / 0.21 | 2.04 / 1.15 | 0/0 | 0/0 (rate 0.78/0.81, dx 0.62) | 0/0 (rate 0.73/0.82, dx 0.24) |
| robot_legs_walk18_shp10sq_s128r24 @ shape 8 | 0.092 / 0.194 (0/0) | 0.001 / 0.004 / 0.003 | 0.56 / 0.20 | 1.80 / 1.12 | 0/0 | 0/0 (rate 0.80/0.84, dx 0.61) | 0/0 (rate 0.72/0.73, dx 0.23) |

Cells: falls/flail-guard trips out of the seeds [episodes in which each guard criterion would trip, judged separately]; walk cells add the guard's peak EMA body rate (mean/max over seeds, trip at 1.5 rad/s) and the torso's forward travel over the 2.5 s after the walk command (m).
