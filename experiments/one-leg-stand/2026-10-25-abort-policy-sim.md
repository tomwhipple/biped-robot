# 2026-10-25: what the balance does when it aborts, chosen in sim

Input from the bench (2026-10-25 19:54Z, free standing on carpet, right stance,
left foot lifted 30 mm): with kd raised to 0.07 the ~2 Hz roll rocking grew, the roll
error ran to +12.3 deg toward the stance side (outward, the side nothing catches), the
controller froze at u = +4.14 deg with torque on, and the foot stayed up until the host
re-commanded the pre-lift pose about 1 s later. Frozen with the foot up is the wrong
end state. `balance_abort.py` compares what the abort should do instead.

## Policies

After the abort fires (|roll err| > 12 deg, or no usable attitude/rate) the loop makes no
more decisions; the policy is open loop. Torque stays on in all of them.

| policy | what happens |
|---|---|
| HOLD (before) | u frozen at its last value, the lifted foot stays up |
| DECAY(T) | u fades to 0 over T (min-jerk), the foot stays up at the lift pose |
| LAND(T) | the stream returns to the pose the B move started from (the double-support pre-lift pose) over T; u held |
| LAND_DECAY(T) | the same landing, u fading over the same T |
| LAND_FAST(T) | the landing over T, u fading over 0.3 s |
| **SEQ(T)** | **u fades over 0.3 s first, then the landing over T** |

## Setup

`balance_abort.py table`: the free-standing one-leg stand of `balance_sweep.py` (as-built
plant, STS3250 hip rolls, keyframes solved on the as-built plant, 30 mm lift over 2 s,
shifted pose to lift pose), the PI of `balance.py` at the firmware defaults (kp 0.3, ki 1.0,
kd 0.03, clamp 10 deg, abort 12 deg, 0.1 deg IMU noise, one tick of IMU delay), the
calibrated roll play of [roll-compliance-2026-10-24.md](../../docs/design-v6/roll-compliance-2026-10-24.md)
(`play_deg` 3 on the four roll joints, which that record shows reproduces the bench's
5.2 deg lift tilt, 5.6 vs 5.2 deg; plus the 1 deg global backlash) and three variants:
play 5 / backlash 2, torso 0.6x, torso 1.4x with 2 Hz lag and 4 ticks dead time. Both
stances. Triggers, 1 s into the hold, every policy sees the identical run up to the abort:

- **outward** and **inward** tips: an impulsive roll rate (5.5 and 6 rad/s outward; 6 and
  6.5 rad/s inward) toward the stance side or the lifted foot, and a slow torso CoM drift of
  30-60 mm over 2 s (the PI absorbs it: it never reaches the abort, no run counts).
- **gain**: destabilising gains, kd 0.09 and 0.12 (the loop rings up during the lift).
- **imu**: the IMU dies mid-hold (the PI freezes in the sim, as in the firmware) and a
  disturbance follows 0.5 s later: a roll-rate bump of 4, 5 or 6 rad/s toward the lifted foot
  or the stance side.

Runs where the abort never fired are not counted (the abort time does not depend on the
policy). Counts are falls (torso up_z < 0.6) over aborted runs, 89 in all.

## Result

| policy | T s | outward | inward | gain | imu | all falls |
|---|---|---|---|---|---|---|
| HOLD | | 8/16 | 14/14 | 10/11 | 10/48 | 42/89 |
| DECAY | 0.4 | 8/16 | 14/14 | 0/11 | 0/48 | 22/89 (foot never lands) |
| DECAY | 1.0 | 8/16 | 14/14 | 1/11 | 4/48 | 27/89 |
| LAND (u held) | 0.4 / 0.7 / 1.0 | 8/16 | 14/14 | 11/11 | 45 / 42 / 37 of 48 | 78 / 75 / 70 |
| LAND_DECAY | 0.4 / 0.7 / 1.0 | 8/16 | 14/14 | 10 / 9 / 6 of 11 | 10 / 4 / 6 of 48 | 42 / 35 / 34 |
| LAND_FAST | 0.4 / 0.7 / 1.0 | 8/16 | 14/14 | 7 / 0 / 0 of 11 | 10 / 7 / 0 of 48 | 39 / 29 / 22 |
| SEQ | 0.4 | 11/16 | 14/14 | 0/11 | 6/48 | 31/89 |
| SEQ | 0.7 | 9/16 | 14/14 | 0/11 | 0/48 | 23/89 |
| **SEQ** | **1.0** | 8/16 | 14/14 | **0/11** | **0/48** | **22/89** |

Excursion after the abort (worst of the non-fallen runs), SEQ 1.0: roll error peak 17.8 deg,
torso tilt 20 deg, the lifted foot back on the floor within 1.1 s of the abort (0.86 s for
the gain trigger), the stance foot slides at most 114 mm (the 6 rad/s bump cases). HOLD never
puts the foot down in the runs that survive.

## Reading

- **The impulsive tips (outward and inward, 22 of the 89) fall under every policy.** By the
  time |err| passes 12 deg the torso is already turning at several rad/s; nothing the
  stance joints do in 0.3-1 s changes that, and the lifted foot lands too late. The sweep
  shows a cliff between 5.5 and 6.5 rad/s, so this is the floor of the abort, not a thing
  the policy tunes. The abort threshold (12 deg) is a separate lever and is not touched here.
- **The controller-caused and sensor-caused aborts are where the policy matters.** HOLD falls
  in 10 of 11 high-gain runs (the oscillation goes on with the gains frozen in) and in 10 of
  48 IMU-fault runs. Taking the correction off first stops the oscillation (DECAY 0/11,
  SEQ 0/11); landing with the correction still on or half on falls (LAND 11/11, LAND_DECAY
  6-10/11): u is up to 10 deg of ankle/hip roll that is right for the lifted-foot posture and
  throws the CoM off the shifted double-support pose.
- **SEQ(1.0)** has the fewest falls (22/89, the impulsive floor), ties DECAY(0.4) on count,
  and unlike DECAY it ends with both feet down. T = 0.4 lands too quickly (3 more outward
  falls, 6 imu falls); 0.7 costs one more outward fall; 1.0 matches the floor. Chosen:
  **u fades over 0.3 s, then the landing over 1.0 s.**

## Not reproduced

- The bench's growing ~2 Hz rocking at kd 0.07. The sim's kd cliff is at 0.09 with the
  bench's one tick of IMU delay, abrupt rather than a slow ring-up (more IMU delay, more
  gyro delay or the 2-4 Hz actuator lag make it more stable, not less: nothing between
  kd 0.05 and 0.09 rocks). The "gain" rows are therefore an abrupt instability, not the
  bench's slow one; the 12.3 deg outward abort with u = +4.14 deg that the bench saw is the
  case SEQ handles (u off first, then down), but it was not run.
- Carpet: friction mu 0.7 in all the runs; the stance foot slid up to 114 mm in some.
- The servo's own speed limit while landing: the sim's servos are not the bench's (the
  firmware writes the landing as the same stream of closed-loop-speed goals as any pose).
- Bump sizes are round numbers (roll-rate impulses on the torso), not measured disturbances.

`balance_abort.py table --json out.json` reruns the whole table (about a minute on 32
cores, `CUDA_VISIBLE_DEVICES=` set); `balance_abort.py golden` prints the rows the host test
pins the C++ `abortProfile` to.
