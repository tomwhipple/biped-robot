# #259 Round 8's replay with the flight records' springs (2026-11-28)

`snap_replay.py run` (Round 8's harness, `docs/design-v6/2026-11-27-left-hip-trim-compliance.md`) on the 11-16 evening
walks: normal timing, early shift start, k >= 2, the stall from the robot's record, scored by `trim_compliance.py`.
The plants are `grade` plus the per-joint series springs (`x_ser_*`, N m/rad) of the gear-compliance record
(`docs/design-v6/2026-11-28-gear-compliance.md`):

| plant | springs |
|---|---|
| recAll | L hip 10.3, R hip 19.5, L ankle 11.3, R ankle 16.5 (the records' spring-only fit, both sides) |
| recH | L hip 10.3, R hip 19.5 |
| serL10 | L hip 10.3 |
| recLonly | L hip 10.3, L ankle 11.3 (the right side rigid) |
| recLRa | L hip 10.3, L ankle 11.3, R ankle 16.5 |

```bash
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=1 MUJOCO_GL=disabled
cd experiments/roll-balance
python tip_causes.py extract --day 20261116 --day 20261113
EV="--day 20261116 --from 1415 --to 2010"
python snap_replay.py run $EV --side LR --delay 0 --workers 6 --name gc_ev --plants "grade=grade;\
recH=grade+x_ser_L_hip=10.3,x_ser_R_hip=19.5;recAll=grade+x_ser_L_hip=10.3,x_ser_R_hip=19.5,x_ser_L_ank=11.3,x_ser_R_ank=16.5;\
serL10=grade+x_ser_L_hip=10.3;recLonly=grade+x_ser_L_hip=10.3,x_ser_L_ank=11.3"
python snap_replay.py run $EV --side LR --delay 0 --workers 6 --name gc_ev2 \
    --plants "recLRa=grade+x_ser_L_hip=10.3,x_ser_L_ank=11.3,x_ser_R_ank=16.5"
python trim_compliance.py results/snap_replay/gc_ev.csv results/snap_replay/gc_ev2.csv
```

`trim_compliance.py` output:

```

L stances, normal timing, early shift start, k >= 2 (stall from the robot's record)
  plant      variant               n  snap%  fast%   not%  m_fast  m_not   incr  short      r  tips  hit false   lean         lean/T
  robot      0                   380   53.4   97.8   12.2    3.47   1.50  +1.97  +0.00   1.00    12   12     0   3.69   +0.02±0.13
  grade      0                   380   48.4   93.4    6.6    3.37   1.42  +1.96  +0.10   0.93     0    0     0   1.33   -0.14±0.03
  recAll     0                   380    0.3    0.5    0.0    1.51   1.16  +0.36  +1.96   0.56     1    0     1   3.04   -0.16±0.04
  recH       0                   380    0.0    0.0    0.0    1.13   0.42  +0.71  +2.34   0.85     0    0     0   3.23   -0.14±0.05
  recLRa     0                   380   78.4   99.5   58.9    4.01   2.62  +1.39  -0.54   0.85     1    0     1   2.68   -0.19±0.04
  recLonly   0                   380   42.4   82.5    5.1    3.15   1.32  +1.83  +0.32   0.91     1    0     1   2.69   -0.26±0.04
  serL10     0                   380   47.1   91.3    6.1    3.46   1.34  +2.12  +0.01   0.93     0    0     0   3.16   -0.27±0.03

R stances, normal timing, early shift start, k >= 2 (stall from the robot's record)
  plant      variant               n  snap%  fast%   not%  m_fast  m_not   incr  short      r r_fast      hike/fast       hike/not
                                then: e_td (the left touchdown's error) mean, its slope on the hike; the stall steps' roll on hike + e_td together
  robot      0                   337   29.1   55.8    3.5    2.59   1.76  +0.83  +0.00   1.00   1.00   +0.48±0.05   +0.61±0.06
                                e_td  -0.29, on hike  +0.73±0.03; roll = +0.32±0.09 hike +0.21±0.10 e_td; lean 2.26, tips 1 (false 0)
                                roll = +0.37±0.09 T L +0.61±0.11 (hike - T L)
  grade      0                   337    9.5   19.4    0.0    1.98   0.63  +1.35  +0.61   0.39   0.05   -0.35±0.05   -0.15±0.05
                                e_td  -2.13, on hike  -0.83±0.02; roll = -0.46±0.14 hike -0.15±0.17 e_td; lean -0.83, tips 0 (false 0)
                                roll = -0.32±0.09 T L -0.38±0.11 (hike - T L)
  recAll     0                   337    6.5   13.3    0.0    2.18   1.56  +0.62  +0.41   0.19   0.13   -0.05±0.02   -0.19±0.03
                                e_td  -0.06, on hike  -0.51±0.03; roll = +0.00±0.03 hike +0.11±0.05 e_td; lean -0.69, tips 0 (false 0)
                                roll = +0.09±0.04 T L -0.22±0.05 (hike - T L)
  recH       0                   337    0.0    0.0    0.0    1.21   0.97  +0.24  +1.39   0.01   0.12   -0.03±0.02   -0.10±0.02
                                e_td  +0.60, on hike  -0.35±0.03; roll = -0.00±0.02 hike +0.07±0.03 e_td; lean 0.45, tips 0 (false 0)
                                roll = +0.00±0.03 T L -0.06±0.04 (hike - T L)
  recLRa     0                   337   42.7   81.8    5.2    3.00   1.90  +1.10  -0.40   0.34   0.28   -0.05±0.05   -0.18±0.04
                                e_td  +0.05, on hike  -0.51±0.03; roll = -0.08±0.07 hike -0.07±0.11 e_td; lean -2.56, tips 0 (false 0)
                                roll = +0.13±0.09 T L -0.26±0.10 (hike - T L)
  recLonly   0                   337   34.7   69.1    1.7    2.78   1.72  +1.06  -0.18   0.36   0.36   +0.01±0.05   -0.15±0.04
                                e_td  +0.08, on hike  -0.52±0.03; roll = -0.02±0.07 hike -0.07±0.10 e_td; lean -0.97, tips 0 (false 0)
                                roll = +0.18±0.08 T L -0.20±0.10 (hike - T L)
  serL10     0                   337    8.9   18.2    0.0    2.26   1.23  +1.03  +0.34   0.44   0.44   +0.03±0.03   -0.11±0.04
                                e_td  +0.72, on hike  -0.36±0.03; roll = +0.01±0.04 hike -0.04±0.06 e_td; lean -0.95, tips 0 (false 0)
                                roll = +0.12±0.06 T L -0.08±0.07 (hike - T L)
```
