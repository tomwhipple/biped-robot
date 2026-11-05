# Motion limiter: design, sim sweep, training flags (2026-11-03)

## Why

On the robot, the first walk step of `robot_legs_walk18_shp10sq_s128r24` (stride1, vx 0.6)
kicked the right leg fast and high: its knee goal moved to −113° and its hip goal to −65°
within 60 ms. The fast STS3215s run at about 240–275 deg/s at full duty and stop dead at
the goal. The reaction yawed the body −19° in 180 ms (gz −3.9 rad/s), the robot pivoted
on its stance foot, and it fell (`hw_sessions/2026-11-03/NOTES.md`). The C2 shaper
smooths the policy's staircase but does not bound it. Tom's ask: a deterministic limit
between the policy command and the hardware, so that no joint starts or stops harder than
a set acceleration.

## Design

The limiter is `obs::MotionLimiter` (`firmware/components/obs/actuation.{h,cpp}`). It
runs per policy joint, every 20 ms tick, and is the last stage before the goal ticks. In
the firmware the chain is shaper → takeover ramp → limiter. In the SIL and in training it
is shaper → limiter, because neither has a ramp.

- **State**: position `p` and velocity `v` per joint, plus a 16-deep history ring for the
  jerk boxcar. It is seeded from measured q at rest, and invalidated wherever the shaper
  is (bus handover, torque release, operator stop).
- **Update**:
  `e = x − p; vb = min(vmax, B(|e|)); v += clamp(sign(e)·vb − v, ±amax·dt); p += v·dt`.
- **Braking look-ahead** `B(d)` (`obs::limitBrakeSpeed`) is the exact discrete stopping
  curve. It gives the largest speed from which braking by `amax·dt` per tick, with
  `p += v·dt`, lands exactly on the target. With `u = amax·dt²` and `m = d/u`, take
  `k = ⌊(√(1+8m) − 1)/2⌋`; then `B = amax·dt·(k + (m − k(k+1)/2)/(k+1))`. The first
  draft used the continuous `√(2·amax·d)` (with a `d/dt` last step), but its last two
  braking ticks need up to 12.5 % more than `amax`. Once clamped, that overshot a still
  target by up to 0.3°. The discrete curve is exact: zero overshoot to the float32 ulp,
  and the stop is time-optimal. A 60° move at 200 deg/s and 2000 deg/s² takes 20 ticks,
  which is the trapezoid's 0.40 s.
- **Jerk** (optional) is a boxcar over the newest `⌈2·amax/(jmax·dt)⌉` ticks (≤ 16).
  The output is a convex mix of limited positions, so the vmax, amax and no-overshoot
  bounds survive. The jerk is at most `jmax`, and the boxcar delays the trajectory by
  `(taps − 1)/2` ticks.
- **Goal speed**: while the limiter is on, each policy joint gets the limited velocity
  × 1.25, floor 50, cap 3400 steps/s (`obs::limitGoalSpeedSteps`). Held servos keep the
  gap law.
- **CLI**: `limit [off|on|<vmax deg/s> [<amax deg/s²> [<jmax deg/s³>]]]`, read live every
  tick. The limiter is off at boot. `limit on` sets 200 / 2000 / no jerk limit. The Wi-Fi
  CLI allows it while benched, as it allows `shape`.
- **Parity**: `sim/fw_limit.py` is the float32 law in the firmware's operation order,
  written once for numpy and JAX. env_mjx (`fw_limit`), walker_env (`fw_limit`) and
  `simh.Limiter` all run it. `tests/test_fw_limit.py` pins it to the compiled code
  through the SIL library's pure `sil_limit_trace`.

## Sim: the limiter on walk18 as trained (no fine-tune)

The scenario is `grade.py`'s chain: fast STS3215 (dead 20 ms, τ 33 ms), the joint-fit
plant and shape 10 Hz, over 8 seeds. `grade.episode` now records:

- the written goals' peak acceleration;
- the plant's peak joint deceleration, from physics-substep joint velocities over a 10 ms
  window;
- the raw body yaw rate.

The window runs from the walk command to 2.5 s after the last command change. Means are
over seeds, with the worst seed in brackets. The full table is `sweep_walk18.md` (`sweep.py`).

| limit | probe | falls / guard trips | guard rate | peak yaw rate, rad/s | goal accel peak, deg/s² | joint decel peak, deg/s² | dx 2.5 s, m |
|---|---|---|---|---|---|---|---|
| off | stride1 0.6 | 0/0 | 0.73 (0.82) | 1.16 (1.33) | 66403 | 6231 | 0.24 |
| 200,1000 | stride1 0.6 | 0/6 pose | 1.23 (1.35) | 1.60 (1.94) | 1000 | 4296 | 0.15 |
| 200,2000 | stride1 0.6 | 0/8 pose | 1.80 (2.27) | 2.80 (4.05) | 2000 | 5421 | 0.27 |
| 200,3000 | stride1 0.6 | 0/8 pose | 1.61 (2.00) | 3.35 (4.15) | 3000 | 5133 | 0.25 |
| 200,4000 | stride1 0.6 | 0/8 pose | 1.14 (1.61) | 2.52 (3.69) | 4000 | 5170 | 0.26 |
| 200,2000,20000 | stride1 0.6 | 8/8 pose | 1.88 (2.61) | 2.17 (3.48) | 2000 | 4664 | 0.54 |
| off | walk 0.3 → stop | 0/0 | 0.78 (0.81) | 2.53 (2.90) | 41736 | 6359 | 0.62 |
| 200,1000 | walk 0.3 → stop | 7/8 rate | 2.91 (3.36) | 5.34 (6.48) | 1000 | 5727 | 0.40 |
| 200,2000 | walk 0.3 → stop | 0/7 pose rate | 1.57 (1.97) | 2.42 (3.40) | 2000 | 6633 | 0.52 |
| 200,3000 | walk 0.3 → stop | 0/4 pose | 1.30 (1.53) | 2.33 (2.82) | 3000 | 5864 | 0.55 |
| 200,4000 | walk 0.3 → stop | 0/0 | 0.98 (1.36) | 1.88 (2.13) | 4000 | 5490 | 0.58 |

At the plain stand (20 s, all settings) there were no trips, the body rate stayed at
0.05, and goal jitter, squeeze and power were unchanged. With the jerk boxcar on, jitter
fell from 0.13° to 0.04–0.07° and squeeze from 2.0° to 1.5–1.65°.

What it says:

1. **The limiter does exactly its job on the goals.** Their peak acceleration is the
   setting to four digits. Without the limiter it is 42,000–67,000 deg/s².
2. **walk18 needs its unbounded goals.** Open loop (`offline.md`), its own goals peak at
   1200–1500 deg/s and 42,000–67,000 deg/s². Even at 200,4000 the limited goal is up to
   61–85° off at the worst moment, and 5–11 % of joint-ticks are more than 0.5° off.
   Closed loop, every setting in the band trips the flail guard on the stride (pose:
   targets still > 15° off the stand 300 ms into the stand command) or raises the body
   rate on the walk → stop. 1000 deg/s² and both jerk settings fall. The policy
   compensates for an actuator it never trained on. **Do not switch `limit` on under
   walk18. A fine-tune with `--fw-limit` comes first.** Among the settings, the untrained
   policy degrades least at amax 4000 and most at 1000.
3. **The sim does not reproduce the robot's spin.** Unlimited stride1 peaks at 1.2 rad/s
   in sim against 3.9 on the robot. The friction agent finds the robot's swing about 1.6×
   faster than the sim's. So the yaw numbers above measure how much the stride is
   disturbed, not the robot event. The joint-deceleration column is dominated by foot
   contact (R hip roll / ankle at touchdown, about 5000–6000 deg/s² in every row), not
   by the commanded stop. The goal column is the one the limiter controls.

## Recommendation

- **Defaults** (`kLimit*`, `limit on`): vmax 200 deg/s, amax 2000 deg/s², jerk off. 200
  deg/s is below the servo's ~240 deg/s loaded top speed, so the limiter sets the profile
  rather than the servo's saturation. 2000 deg/s² is the log-centre of the 1000–4000 band:
  it stops from 200 deg/s in 100 ms over 10°, against the servo's dead stop. Jerk stays
  off: the boxcar's delay (30 ms at 50,000, 90 ms at 20,000) cost the untrained walk the
  most, and a trained one can add it later.
- **Training**: fine-tune with `--fw-limit 200,2000` next to the existing
  `--fw-shaper 10 --fw-goal-speed`, then deploy with `shape 10` and `limit on`
  (= `limit 200 2000`). If the fine-tune cannot recover the walk margin, 200,3000 is the
  next setting: the walk's goals deviate less, with a p95 of 11.8° against 16.4° on the
  0.3 m/s walk.
- **On the robot, before a trained run exists**: `limit` is safe at the stand (no change
  in sim), but not under a walk.
- **Open for the bench**: whether goal speed = limited velocity (not the gap law) softens
  a pushed joint. That depends on how the STS3215 applies register 46. In the sim
  (`--fw-goal-speed`) it slews the set point, so stiffness is unchanged.

## Files

`sweep.py` → `sweep_walk18.{md,json}`, the closed-loop sweep. `offline.py` →
`offline.{md,json}`, the open-loop replay of walk18's goals through the limiter. The
grader changes are in `../shaper-train/grade.py`: the `limit` variant key and the
yaw / deceleration metrics.
