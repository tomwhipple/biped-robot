# Why the robot's stride trips the guard and the sim's doesn't (2026-11-06)

Evidence from the 2026-11-03 (`w18_1103.npz`) and 2026-11-06 (`o5_1106_t1.npz`, stride1_03)
flight records, plus test 2 (steps5_03) of the same session in
`hw_sessions/2026-11-06/flight_20261106T160828Z_omni5.frec`. Grades are `grade.py`, 8 seeds,
measured-sole plant, cells falls/trips at the 1.5 rad/s line. Follows
`hw_sessions/2026-11-05/kick_analysis.md`.

## 1. The stride command outlives the clock cycle

`stride1_03` means one gait cycle (667 ms). On the robot on 11-06 the command ended at +700 ms,
at phase ~2.07. Coast holds only at pi/2 (1.571), which the clock had already passed, so it ran
until +1320 ms under the stand command. The right leg made a third swing the sim never makes
(knee -60, hip -29 at +960 ms), gz reached -6.1 rad/s at +840 ms, and the guard tripped at +904 ms.

The grade's schedule ended on the tick that lands on the stand phase, so the sim never saw this:

| student | stride1 0.3 refit dqlag, -40 ms | on grid | +20 ms | +40 ms |
|---|---|---|---|---|
| omni5_dqlag | 0/0 (rate 1.10) | 0/0 (1.14) | 0/8 (1.64) | 0/8 (1.73) |
| walk25_msole_d13_gh2h14_s1 | 0/0 (1.22) | 0/0 (1.28) | 0/8 (1.73) | 0/8 (1.82) |

All 13 students trip 8/8 when the command is 20 or 40 ms late. The robot's trips on 11-05 came at
+0.75 to +0.87 s, the same window.

- **No-retrain fix to try:** end the stride command 40-60 ms before the cycle (robot test: Tom's go).
- **Firmware option, sim only:** `coast_back` holds if the clock is just past the stand phase.
  The `late* hold` probes are mixed, not clean (omni5 still 0/8 at +40 ms, rate 2.10), so this is
  not a recommendation yet.

## 2. The refit actuator runs too fast, and the delay margin is thin

Peak joint speed in all three robot records: hip pitch 246-254 deg/s, knee 281-303 deg/s. The refit
sim (`w0_scale` 1.325/1.281) reaches 360+ with omni5. The `robot` probes use w0 0.87/1.0 and 60 ms of
dead time. omni5, steps5 0.3, one dq lag (window 2), 4 seeds; robot: rate 3.07, final yaw +70, R knee
-44 at +240 ms:

| w0 hip, knee | dead | rate | falls | final yaw | R knee, +240 ms |
|---|---|---|---|---|---|
| 0.87, 1.0 | 40 ms | 1.24-1.41 | 0 | +15 to +33 | -49 |
| 0.87, 1.0 | 60 ms | 1.43-2.05 | 0 | +9 to +85 | -42 |
| 0.87, 1.0 | 80 ms | 1.75-2.61 | 1 | -20 to +12 | -38 |
| refit | 40 ms | 1.71-1.82 | 0 | +7 to +11 | -61 |
| refit | 80 ms | 3.71-4.36 | 4 | -50 to -35 | -46 |

So the speed cap gets the joints and the final yaw about right, but not the body rate. Rate does rise
quickly with delay. omni5 at 0.87/1.0, 60 ms, by dq window:

| dq window (ticks) | rate | falls of 4 |
|---|---|---|
| 2 | 1.43-2.05 | 0 |
| 3 | 1.74-2.17 | 0 |
| 4 | 1.92-2.64 | 0 |
| 5 | 2.14-3.46 | 1 |
| 6 | 2.94-3.91 | 3 |

omni6_dqlag_crouch: window 5 1/4 falls, window 6 3/4. The register fit says window 2, so this does not
show the robot has more lag. It shows the margin ends close to where the robot's rate sits, which fits
a fall the window-2 gate misses. The `dq5`, `dq6` and `dead80` probes grade it.

## 3. Robot-like joint velocity is what removes the runaway

On the `robot` probes, all four students trained with `--dq-obs-window 2,3 --dq-obs-quantum 50` have
0 falls on all six probes and peak chatter <= 3.2. The other nine each fall on at least two probes.
walk25 there reads rate 3.2-3.4 and chatter 7.8-9.2 (robot 11-05: 2.57, 7.07), falling on 5 of 6
probes (7 of 8 seeds on walk -> stop).

Two limits:

- The gate is not sufficient: omni5 scores 0/8 on `steps5 0.3 robot` (rate 1.52), and on the robot
  it reached 3.07 and fell.
- The same training did not stop the robot's second-swing yaw (section 4).

## 4. The second swing's yaw is still unexplained

Test 2: the first swing matches (yaw -7.7 vs sim -5.6 at +240 ms). The second does not: gz +5.8 rad/s
at +560 ms and +42 deg by +720 ms, vs +3.6 deg in the sim. Even at the robot's speed and dead time the
sim reads about +9 at +640 ms against the robot's +40.

`replay_joints.py` prescribes the robot's measured joints on the free-base plant, with no policy or
actuator in the loop:

- On the 11-03 first swing, the right hip and knee alone yaw the body -9 deg by +180 ms. The whole
  record gives -10 to -14 for mu 0.05-1.0, pad radius x0.5, or no contact at all, against the robot's
  -19. The floor barely matters: this is the swing leg's angular momentum.
- On 11-06 the replayed joints do produce a large second-swing yaw (mu 0.5: -10 at +400 ms to +50 at
  +700 ms; free body: -26 to +15; robot: -3 to +27). So the plant can make this yaw from the robot's
  joint motion, and the closed-loop sim doesn't produce that motion. The replay has limits: double
  support is over-constrained by measured joints with zero errors.

Open hypothesis, untested: the right foot lands late (knee -17 at +480 ms on the robot, -5 in the
refit sim), so the left leg lifts with little or no ground contact, and the two legs' scissor motion
yaws a nearly free body. 11-06 was on carpet, 11-03 on a board; carpet is not modelled.

Also confirmed: fused yaw equals integrated gz to 0.3 deg over the swing, so this is not an IMU artifact.

## 5. The yaw kick

`--yaw-kick-range lo,hi --yaw-kick-prob p` (training) and the `yk*` probes (grade) add body yaw rate
in single support. A 2 rad/s kick per step takes walk25's steps5 to rate 2.5 and chatter 7.1, near its
robot numbers. All four dq-lag students stay at 0 falls and chatter ~3 on steps5 up to 3 rad/s
per step; omni5 falls 1/8 and 4/8 on the stride1 yk2 and yk3 probes. The kick is a stand-in, not a model.

## Files

- `replay_joints.py`: `YAW_REC=w18|o5 YAW_MS=... python replay_joints.py [variant ...]`
- `w18_1103.npz`, `o5_1106_t1.npz`: the decoded flight windows
- `tests/test_yaw_kick.py`
