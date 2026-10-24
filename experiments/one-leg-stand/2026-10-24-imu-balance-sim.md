# 2026-10-24: stance-leg IMU roll balance, designed in sim

Sim record behind `balance.py` / `balance_sweep.py` and the firmware `pose ...
B<L|R>`. Nothing here is a hardware measurement except the two inputs it was
built from: the bench tilt (5.2 / 5.0 deg with no shift, 6.8 deg left stance and
3.5 deg right stance with the 53 mm design shift, other foot lifted 30 mm) and
the observation that the torso tilts ~5 deg while the encoders show ~1 deg.

## What the sim reproduces and what it does not

- Design keyframes (53.1 mm shift) on the as-built plant: CoM margin 0.4 mm,
  foot never clears the floor, torso tilt 15 deg. The hardware showed the same
  failure with a smaller tilt (6.8 deg); the sim's play and backlash are the
  measured 3 deg / 1 deg, and the robot's roll compliance is probably larger.
- The left/right difference seen on the bench (6.8 vs 3.5 deg) is not in the
  model. It is represented as a torso CoM offset of +-12/15 mm, which the
  controller handles for both stances without a per-side term.
- Everything about the torso mass is assumed (0.324 kg as-built estimate).

## Sweep, chosen gains (kp 0.25, ki 0.5, ankle +1, hip -1, clamp 6 deg)

204 runs = 34 cases x 2 stances x 3 seeds. "Core" excludes the 18 runs whose
keyframes were solved 20 mm too far inboard, which no roll controller recovers
(CoM already outside the support at the lift).

| | runs | ok | fell | aborted | foot off min | CoM margin min |
|---|---|---|---|---|---|---|
| open loop, core | 186 | 156 | 0 | - | 0.0 s (30 runs never lift) | -5.4 mm |
| PI x1, core | 186 | 173 | 0 | 0 | 9.82 s | 5.6 mm |
| PI x0.5, core | 186 | 170 | 0 | 3 | 0.0 s | 2.7 mm |
| PI x2, core | 186 | 170 | 1 | 4 | 6.0 s | 2.0 mm |
| PI x1, all | 204 | 173 | 0 | 18 | 0.0 s | -2.4 mm |

The 13 core runs that are not `ok` at x1 all stand for the full hold; they miss
the 10 mm CoM-margin bar (5.6-9.5 mm): torso 0.6x with the design keyframes
(8.5 mm), torso 1.4x with 5 deg play + 2 deg backlash + 2 Hz lag + 4 ticks (5.6,
5.8, 9.2, 9.5 mm), torso 0.6x with a +15 mm CoM offset on the right stance
(7.6-8.5 mm). The output clamp is frequently reached (peak 4-6 deg): the play
absorbs most of the command, so a 6 deg command buys about 1 deg of torso roll.
Torso roll about the hold mean, last 5 s: median 0.09 deg RMS, worst 0.72 deg
(x1). At x2, one run falls (right stance, design keyframes, torso 1.4x) and the
worst-case RMS is 16 deg: that loop is no longer stable.

## Stability margin

Stable and holding at half and double the chosen kp and ki together. The limit
is kp: 0.35/0.5 falls on most cases (kp 0.7 / ki 1.0), kp 0.3 at x2 (0.6) falls in
17 of 136 runs. So the margin on kp is about 2.0-2.4x upward, and more than 2x
downward (kp 0.125 still holds the foot up but leaves 2-3 deg RMS drift).
The limit comes from the dead time, the servo lag and the play, not from the PI
structure; gyro feedback (a D term) is the obvious next step if more bandwidth is
needed.

## Controller structure (balance.py == roll_pi.h)

    err = roll - roll_ref              roll_ref captured at engage (the shifted pose)
    u   = clamp(kp*err + ki*I, +-6 deg)    I += err*dt unless saturated and err pushes further
    stance ankle roll += u ; stance hip roll += -u        (joint frame; swing leg untouched)
    |err| > 12 deg or the IMU read fails: correction frozen at its last value, torque kept

Roll is atan2(-up_y, up_z) from the fused attitude (positive = leans toward the
robot's right). The sign of the IMU roll on the robot (mount not re-derived for
v6) has not been checked against this: flip both mix signs (`balance set ... -1
1`) if a hand tilt shows the opposite sign.
