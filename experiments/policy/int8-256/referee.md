# CPU referee, python column (eval_precision.py --scenarios ..., 8 seeds, hardware-claim)

| run | stand_10s | stand_off | walk_steps_L | walk_steps_R | push_gauntlet | reversal | line_1m | total |
|---|---|---|---|---|---|---|---|---|
| robot_legs_walk16_act_s128r24 | 6/8 (drift 13.4cm) | 6/8 (drift 5.2cm) | 0/8 (min peak 0.7cm) | 0/8 (min peak 0.6cm) | 0/8 (d=0.4m) | 0/8 (rev 0.6s) | 0/8 (n/a) | 12/56 |
| robot_legs_walk16_act_s128r24_fast | 7/8 (drift 7.6cm) | 5/8 (drift 14.8cm) | 0/8 (min peak 0.9cm) | 0/8 (min peak 0.6cm) | 0/8 (d=0.6m) | 0/8 (n/a) | 0/8 (t=9.3s) | 12/56 |
| robot_legs_walk16_act_s256r24 | 6/8 (drift 12.9cm) | 5/8 (drift 12.0cm) | 0/8 (min peak 0.6cm) | 0/8 (min peak 0.6cm) | 0/8 (d=0.5m) | 0/8 (rev 0.2s) | 0/8 (n/a) | 11/56 |
| robot_legs_walk16_act_s256r24_q8 | 7/8 (drift 6.7cm) | 7/8 (drift 6.2cm) | 0/8 (min peak 0.4cm) | 0/8 (min peak 0.5cm) | 0/8 (d=0.4m) | 0/8 (n/a) | 0/8 (n/a) | 14/56 |
| robot_legs_walk16_act_s256r24_tp | 8/8 (drift 3.6cm) | 8/8 (drift 3.6cm) | 0/8 (min peak 0.8cm) | 0/8 (min peak 0.7cm) | 0/8 (d=0.7m) | 0/8 (rev 1.2s) | 0/8 (t=10.3s) | 16/56 |
| robot_legs_walk16_act | 8/8 (drift 2.6cm) | 6/8 (drift 6.6cm) | 0/8 (min peak 1.0cm) | 0/8 (min peak 0.5cm) | 0/8 (d=0.7m) | 0/8 (rev 0.9s) | 0/8 (t=8.3s) | 14/56 |
