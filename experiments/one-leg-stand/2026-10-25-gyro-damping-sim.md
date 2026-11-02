# 2026-10-25: gyro roll-rate damping for the stance-leg balance, designed in sim

Sim record behind the `kd` term of `balance.py` / `roll_pi.h`. Input from the
bench (2026-10-24/25): a straight 30 mm lift over 4 s on carpet drove the roll
error to -11.5..-13 deg in ~3.5 s (tipping toward the lifted foot) and the
firmware froze at the abort limit; with a 10 mm stage first the PI (kp 0.25, ki
0.5) peaked at -2.7 / -5.2 deg and settled in ~13 s.

## What the sim does and does not reproduce

- Reproduced: the PI (kp 0.25 ki 0.5) is slow. Lifted straight to 30 mm, 12 of 20
  left-stance cases (2 s lift) and 10 of 20 (4 s lift) are still above 1 deg 3 s
  after the lift ends, median settle 4.5 s / 3.0 s; the worst plant (torso 0.6x,
  CoM +15 mm, 2 Hz lag + 4 ticks dead time) peaks at 6.9 deg, and with the PI off
  that plant tips (15 deg).
- **Not reproduced: the bench's 12-13 deg tip.** On the as-built plant with the
  sim's calibrated play (3 deg play + 1 deg backlash, open-loop tilt 2.4 deg) the
  PI's peak is ~2 deg for most cases. A larger play (9-11 deg) gives the bench's
  4-5 deg open-loop tilt but then the 10 deg clamp can no longer null it, which the
  bench did (u 9.2-9.7 deg): the sim's play (a deadzone) and the robot's roll
  compliance differ in how much correction one degree of command buys. So the
  gains below are tuned for the sim's dynamics and the bench's tip speed is an
  assumption, not a fit.

## Model

`u = kp*err + ki*I + kd*w`, `w` = freejoint `qvel[3]` (body-frame x rate; + rotation
about x tips the torso to the robot's right = the sign of `roll`), delayed one 20 ms
tick, plus gaussian noise 0.0075 rad/s rms and a per-run bias residual (0 or
+-0.010 rad/s). The sign of `kd` is the same as `kp` because the stance mix already
makes `+u` oppose `+err`. Clamp 10 deg (the
firmware's), 12 deg abort, IMU roll noise 0.1 deg and one tick of delay as before.

## Spec and sweep

Straight to 30 mm over 2 s and over 4 s, no pre-stage; the quick robustness set
(torso 0.6-1.4x, play 3-5 deg, backlash 1-2 deg, lag 0/2 Hz, dead time 0-4 ticks,
mu 0.4-1.0, torso CoM +-12/15 mm, keyframes on the design plant, shift 10 mm too far
outboard) plus bias residuals and four combined worst cases, both stances: 80 runs.
Pass: no fall, no abort, peak |err| < 6 deg, |err| < 1 deg from 3 s after the lift
ends to the end of the 10 s hold.

| gains | pass | falls | peak median / max | settle > 3 s | hold roll RMS (last 5 s) max |
|---|---|---|---|---|---|
| before: kp 0.25 ki 0.5 kd 0 | 51/80 | 0 | 2.1 / 6.9 deg | 26 | 0.97 deg |
| PI only: kp 0.3 ki 1.0 kd 0 | 74/80 | 0 | 2.1 / 6.8 deg | 4 | 0.66 deg |
| **chosen: kp 0.3 ki 1.0 kd 0.03** | **75/80** | 0 | 2.0 / 6.6 deg | 4 | 0.74 deg (median 0.06) |

The five failures with the chosen gains: three are the keyframes solved 10 mm too far
(bias +0.01: authority-limited at the clamp, 2.3-2.7 deg left over; no roll gain fixes
it), one is the worst plant above (peak 6.6 deg, settles in 0.8 s), and one settles at
exactly 3.0 s (light torso, play 5, mu 0.4, 2 Hz lag, 4 ticks, bias -0.01). Every other
plant, both stances and both lift times, stays under 3.4 deg. The 6 deg bar is missed
only with 2 Hz lag + 4 ticks of dead time + light torso + CoM offset together, a worse
actuator path than the bench streamer's (which has no 2 Hz shaper). Honest reading: **most of the gain comes
from ki** (settling), and the gyro term buys ~0.2 deg of peak and stability margin.

Stability margin (all gains scaled together, chosen ratio):

| scale | pass | falls |
|---|---|---|
| 0.5 | 50/80 | 0 (slow) |
| 1.0 | 75/80 | 0 |
| 1.5 | 75/80 | 0 |
| 2.0 | 43/80 | 4 |
| 2.5 | 21/80 | 31 |

Without kd the same PI falls in 1 run at 1.5x and 22 at 2x, so kd moves the edge
from between 1x and 1.5x (first fall at 1.5x) to between 1.5x and 2x. kd alone (kp 0.3 ki 1.0): 0.05 is fine (76/80), 0.07 oscillates
(6 falls, hold RMS 3.3 deg), 0.10 falls in 42; the margin on kd is ~2.3x, and a
gyro noise of 0.02 rad/s rms instead of 0.0075 changes nothing (75/80). Clamp 15 deg
instead of 10 falls in 2 of 80 and was not adopted. Hold roll RMS is 0.06 deg median
over the 10 s hold (0.74 worst), no limit cycle at the chosen gains.

## Measured vs assumed

Assumed: the gyro noise and bias figures, one tick of gyro delay (the firmware reads
the latest 4 ms sample, so the sim is conservative), the play model (see above), and
that the bench tip is of the speed the sim shows. Measured nothing here: the gains are
untested on the robot; first bench use should start at `balance set 0.3 1.0 1 -1 10 12 0.03`
(the defaults) with a 10 mm stage, then 30 mm over 4 s, and watch `balance` for
the `rate` field and `balance log` for ringing.
