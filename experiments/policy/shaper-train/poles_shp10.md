| variant | stand goal jitter mean / max (deg) | body rate x / y / z | power / hold (W proxy) | squeeze / hold (deg, sum 12 joints) | x1.5 | walk 0.3 -> stop | stride1 0.6 |
|---|---|---|---|---|---|---|---|
| robot_legs_walk17_shp10_s128r24 @ shape 0 | 4.571 / 17.554 (5/0) | 0.083 / 0.630 / 0.262 | 54.71 / 1.60 | 48.16 / 3.22 | 8/8 [rate 8 pose 8 tilt 5 chatter 8] | 0/0 (rate 1.41/1.50, dx 0.63) | 0/5 [chatter 5] (rate 1.26/1.32, dx 0.23) |
| robot_legs_walk17_shp10_s128r24 @ shape 20 | 0.287 / 0.839 (0/0) | 0.003 / 0.008 / 0.008 | 1.85 / 1.50 | 4.28 / 2.46 | 8/8 [rate 8 pose 8 tilt 2 chatter 8] | 0/0 (rate 1.19/1.35, dx 0.64) | 0/0 (rate 1.19/1.23, dx 0.23) |
| robot_legs_walk17_shp10_s128r24 @ shape 15 | 0.211 / 0.575 (0/0) | 0.003 / 0.005 / 0.006 | 1.76 / 1.51 | 3.72 / 2.45 | 5/5 [rate 5 pose 5 tilt 2 chatter 5] | 0/0 (rate 1.02/1.07, dx 0.62) | 0/0 (rate 1.08/1.14, dx 0.23) |
| robot_legs_walk17_shp10_s128r24 @ shape 12 | 0.158 / 0.395 (0/0) | 0.003 / 0.004 / 0.004 | 1.71 / 1.53 | 3.34 / 2.49 | 0/0 | 0/0 (rate 0.91/0.95, dx 0.61) | 0/0 (rate 0.94/0.97, dx 0.22) |
| robot_legs_walk17_shp10_s128r24 @ shape 10 | 0.123 / 0.276 (0/0) | 0.002 / 0.003 / 0.004 | 1.69 / 1.54 | 3.10 / 2.49 | 0/0 | 0/0 (rate 0.86/0.91, dx 0.61) | 0/0 (rate 0.82/0.87, dx 0.22) |
| robot_legs_walk17_shp10_s128r24 @ shape 8 | 0.090 / 0.171 (0/0) | 0.002 / 0.003 / 0.003 | 1.67 / 1.55 | 2.91 / 2.49 | 0/0 | 0/0 (rate 0.93/0.96, dx 0.60) | 0/0 (rate 0.73/0.76, dx 0.23) |

Cells: falls/flail-guard trips out of the seeds [episodes in which each guard criterion would trip, judged separately]; walk cells add the guard's peak EMA body rate (mean/max over seeds, trip at 1.5 rad/s) and the torso's forward travel over the 2.5 s after the walk command (m).
