# Predicted foot loads, both feet down (sim, CPU, no balance loop)

Replay: stand -> crouch (logged s3000, starts 3.5 s) -> 3 s -> shift pose (logged s2000), then 3 s settle; values = mean of the last 0.5 s. Poses come directly from the crouch (as the task asked), NOT rung-to-rung as the robot ran them. Firmware streamer + same Replay code as check1_oneleg (imported), no B token (none in either log), Ctl default. Force = sum of |normal contact force| on that foot's pads, grams = N/9.81*1000. Torso roll = Replay's IMU roll (includes the plant's mount/tilt offsets), + = per simfit_sim convention (the log's sign: R ladder -, L ladder +, matches the 10-25 robot roll signs). Stand/crouch rows are identical for R and L ladders (only the label side differs); one set shown twice for completeness.


## plant `grade` (total model mass 1.372 kg; weight 13.5 N; no battery pack)


R ladder (pelvis toward the R foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 547 | 825 | 0.601 | +0.45 | 13.5 |
| crouch | 556 | 816 | 0.595 | +0.50 | 13.5 |
| x0.5 | 423 | 949 | 0.692 | +0.52 | 13.5 |
| x0.75 | 368 | 1004 | 0.732 | +0.57 | 13.5 |
| x1.0 | 324 | 1048 | 0.764 | +0.64 | 13.5 |
| x1.1 | 305 | 1067 | 0.778 | +0.68 | 13.5 |
| x1.2 | 288 | 1084 | 0.790 | +0.72 | 13.5 |
| x1.3 | 271 | 1101 | 0.802 | +0.77 | 13.5 |
| x1.4 | 256 | 1116 | 0.813 | +0.82 | 13.5 |

L ladder (pelvis toward the L foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 547 | 825 | 0.601 | +0.45 | 13.5 |
| crouch | 556 | 816 | 0.595 | +0.50 | 13.5 |
| x0.5 | 723 | 649 | 0.473 | +0.52 | 13.5 |
| x0.75 | 820 | 552 | 0.403 | +0.52 | 13.5 |
| x1.0 | 916 | 456 | 0.333 | +0.50 | 13.5 |
| x1.1 | 957 | 415 | 0.303 | +0.48 | 13.5 |
| x1.2 | 995 | 377 | 0.275 | +0.44 | 13.5 |
| x1.3 | 1035 | 337 | 0.246 | +0.40 | 13.5 |
| x1.4 | 1072 | 300 | 0.219 | +0.34 | 13.5 |

## plant `train` (total model mass 1.507 kg; weight 14.8 N; no battery pack)


R ladder (pelvis toward the R foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 763 | 745 | 0.494 | -0.00 | 14.8 |
| crouch | 766 | 742 | 0.492 | -0.01 | 14.8 |
| x0.5 | 520 | 987 | 0.655 | +0.19 | 14.8 |
| x0.75 | 506 | 1002 | 0.665 | +0.35 | 14.8 |
| x1.0 | 507 | 1001 | 0.664 | +0.48 | 14.8 |
| x1.1 | 490 | 1018 | 0.675 | +0.57 | 14.8 |
| x1.2 | 473 | 1035 | 0.686 | +0.66 | 14.8 |
| x1.3 | 466 | 1041 | 0.691 | +0.81 | 14.8 |
| x1.4 | 448 | 1060 | 0.703 | +0.98 | 14.8 |

L ladder (pelvis toward the L foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 763 | 745 | 0.494 | -0.00 | 14.8 |
| crouch | 766 | 742 | 0.492 | -0.01 | 14.8 |
| x0.5 | 995 | 512 | 0.340 | -0.20 | 14.8 |
| x0.75 | 992 | 516 | 0.342 | -0.35 | 14.8 |
| x1.0 | 1004 | 504 | 0.334 | -0.47 | 14.8 |
| x1.1 | 1025 | 483 | 0.320 | -0.57 | 14.8 |
| x1.2 | 1042 | 466 | 0.309 | -0.66 | 14.8 |
| x1.3 | 1047 | 461 | 0.306 | -0.81 | 14.8 |
| x1.4 | 1070 | 437 | 0.290 | -0.98 | 14.8 |

## plant `grade+ser_hip=14.2` (total model mass 1.372 kg; weight 13.5 N; no battery pack)


R ladder (pelvis toward the R foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 479 | 893 | 0.651 | +0.25 | 13.5 |
| crouch | 477 | 895 | 0.652 | +0.30 | 13.5 |
| x0.5 | 382 | 990 | 0.722 | +0.19 | 13.5 |
| x0.75 | 351 | 1021 | 0.744 | +0.21 | 13.5 |
| x1.0 | 328 | 1044 | 0.761 | +0.27 | 13.5 |
| x1.1 | 320 | 1052 | 0.767 | +0.33 | 13.5 |
| x1.2 | 313 | 1059 | 0.772 | +0.39 | 13.5 |
| x1.3 | 303 | 1069 | 0.779 | +0.45 | 13.5 |
| x1.4 | 294 | 1078 | 0.785 | +0.54 | 13.5 |

L ladder (pelvis toward the L foot)

| pose | L g | R g | R share | torso roll deg | total N |
|---|---|---|---|---|---|
| stand | 479 | 893 | 0.651 | +0.25 | 13.5 |
| crouch | 477 | 895 | 0.652 | +0.30 | 13.5 |
| x0.5 | 632 | 740 | 0.540 | +0.59 | 13.5 |
| x0.75 | 732 | 640 | 0.467 | +0.76 | 13.5 |
| x1.0 | 821 | 551 | 0.401 | +0.89 | 13.5 |
| x1.1 | 856 | 516 | 0.376 | +0.92 | 13.5 |
| x1.2 | 892 | 480 | 0.350 | +0.94 | 13.5 |
| x1.3 | 933 | 439 | 0.320 | +0.94 | 13.5 |
| x1.4 | 970 | 402 | 0.293 | +0.91 | 13.5 |
