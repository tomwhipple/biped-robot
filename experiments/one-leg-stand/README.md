# One-leg stand (v6 legs)

The Gate A path held still, as a bench experiment: crouch 20 mm, shift the pelvis
until the whole-robot CoM sits 5 mm inboard of the stance sole centre, lift the
other foot 30 mm, hold, and undo it, for either stance. The feedforward is open
loop; an optional stance-leg IMU roll PI absorbs the model error. No policy. This is the
"bench before policy" step for the curriculum's `balance_L/R` skill
([docs/precision-curriculum.md](../../docs/precision-curriculum.md)).

`one_leg.py` builds the IK keyframes (`sim/v6_kin.leg_ik` through
`design_gates.q_of`) and simulates what the robot actually runs: the firmware's
`pose ... s<ms>` streamer, a minimum-jerk move in **joint** space between
keyframes. The plant and servo model are those of `sim/static_gait.run_walk`
(2 Hz shaper + 80 ms dead time by default, integer ticks, 1° backlash, 3° play on the four
rolls), with STS3250s at the two hip rolls as built. It writes the keyframes as
`pose` bus-order ticks for the as-built calibration (`ticks = zero + dir · rad ·
4096 / 2π`, [docs/servo-map.md](../../docs/servo-map.md) §3).

```bash
.venv/bin/python experiments/one-leg-stand/one_leg.py --as-built --stance L --json build/one_leg/sweep30.json
.venv/bin/python experiments/one-leg-stand/one_leg.py --as-built --stance R --no-sweep --bal-kp 0.25 --bal-ki 0.5 --bal-kh -1
```

`--stance L|R` is the foot that stays down; the other lifts. One code path
serves both: the shift is solved from the model's CoM for the chosen stance
(inboard bias mirrored), nothing is tuned to a side or to the clamp rig, and the
two stances come out exact mirrors. The same run prints the `pose` lines for the
sequence (`CAL` ticks are those of the comment on `CAL`: refresh them from `cal
show` before sending anything).

`--solve-set KEY=VAL...` changes the plant the keyframes are solved on,
`--plant-set` the plant they are simulated on (a deliberate model error);
`--as-built` is both set to `AS_BUILT`.

## The as-built plant

The torso carries the printed frame and the General Driver board only: tethered
power, no pack, Pi, head, neck servo, power board or arms. `AS_BUILT` in
`one_leg.py` zeroes `m_battery m_pi4 m_power m_neck_servo m_head` and keeps
`m_wiring` at 10 g (a guess), with `torso_v7` still on so the deck height and
board position are the printed torso's. The robot weighs **1.240 kg** with a
**0.324 kg** torso (frame 174 g + board 30 g + leads 10 g + the two hip-yaw
servo cases 110 g), against 1.650 / 0.695 kg for the design plant. The torso mass
is unmeasured: the sweeps vary it 0.6x to 1.4x.

## Keyframes (as-built plant, left stance, degrees, sim frame)

| pose | hip roll L / R | hip pitch L / R | knee L / R | ankle L / R | ankle roll L / R |
|---|---|---|---|---|---|
| crouch 20 mm | 0 / 0 | -24.6 / -24.6 | -49.2 / -49.2 | -24.6 / -24.6 | 0 / 0 |
| shift (pelvis +62.0 mm) | -11.4 / -11.4 | -19.3 / -19.3 | -40.8 / -40.8 | -21.5 / -21.5 | +11.4 / +11.4 |
| lift R 30 mm | -11.4 / -12.6 | -19.3 / -35.2 | -40.8 / -73.0 | -21.5 / -37.8 | +11.4 / +12.6 |

The design plant (with pack, Pi and head) needs a 53.1 mm shift. Run on the
as-built robot that shift leaves the CoM 0.4 mm inside the sole when the foot
lifts; this is the failure seen on the bench. Right stance is the mirror image.

## Open loop, sim

As-built keyframes on the as-built plant: stays up, foot off for 11.2 s of the
11 s window, CoM margin >= 24.8 mm (static 18.5 mm), torso tilt 3.4 deg, every
sweep case up (mu 0.4/1.0, play 5 deg, backlash 2 deg, mass +-15 %, 120 g payload,
servos -15 %, floor +-2 deg), foot off >= 11.0 s. Real clearance is 14-18 mm
(the hanging leg sags through the roll play), so `run_walk`'s own OK column
(>= 15 mm) marks the play 5 and backlash 2 rows FAIL.

Open loop is only as good as the model: with the design keyframes on the
as-built plant, or a torso CoM 12-15 mm off-centre, or a torso 40 % off the
assumed mass, the lift fails in a good part of the cases (see below). A 15 mm
lift command does not lift the foot at all; use 30 mm.

## Closed loop: stance-leg roll PI

`balance.py` (sim) and `firmware/components/balance/roll_pi.h` (bench, `pose ...
B<L|R>`) are the same controller: the torso roll relative to the roll captured
when feedback engages (the shifted pose, before the lift) goes through a PI plus a
gyro roll-rate damping term (`kd * rate`, derivative on the measurement; rate is the
body-x gyro, + = roll increasing) with output clamp and anti-windup, and the output is added to the **stance** leg's
ankle roll (x +1) and hip roll (x -1) only. Ankle +u with hip -u is a lateral
pelvis translation with no net torso tilt: it moves the CoM, which is what the
under-compensated shift lacked. The same gains serve either stance because the
sim's roll joints mirror and the output is odd in the error.

| gain | value | |
|---|---|---|
| kp | 0.3 rad/rad | all three gains together: held from 0.5x to 1.5x, falls appear at 2x |
| ki | 1.0 /s | the settling time: 0.5 /s leaves 12 of 20 left-stance lifts above 1 deg after 3 s |
| kd | 0.03 rad per rad/s | gyro roll-rate damping; good to about 2x alone, a cliff between 0.05 and 0.07 (dead time) |
| ankle / hip mix | +1 / -1 | |
| output clamp | 10 deg | the robot needed 9.2-9.7 deg (see the hardware section); the sweep is the same at 6 and 10 |
| abort | 12 deg roll error, or a dead IMU: u fades over 0.3 s, then the pose returns to the pre-lift pose over 1.0 s | [2026-10-25-abort-policy-sim.md](2026-10-25-abort-policy-sim.md); `balance_abort.py` |

`balance_sweep.py` runs 34 cases per stance (torso mass 0.6-1.4x, play 3-5 deg,
backlash 1-2 deg, lag 0 or 2 Hz, 0-4 ticks dead time, mu 0.4-1.0, keyframes
solved on the design plant, solved 10 mm outboard or 20 mm inboard, torso CoM
+-12/15 mm off-centre, combinations of these) x 3 seeds, 0.1 deg IMU noise and
one tick of IMU delay. Success: never falls, foot off >= 90 % of the 10 s hold,
CoM margin >= 10 mm, no abort. Results are in
[2026-10-24-imu-balance-sim.md](2026-10-24-imu-balance-sim.md) (PI) and
[2026-10-25-gyro-damping-sim.md](2026-10-25-gyro-damping-sim.md) (the gyro term, the
transient spec and the retuned gains). `balance_sweep.py gyro` runs the transient
sweep: 20 cases x 2 stances x lift over 2 s and 4 s straight to 30 mm, gyro noise
0.0075 rad/s, one tick of gyro delay, bias residual +-0.01 rad/s.

## Hardware

- [2026-10-24, on the stand](2026-10-24-stand-rehearsal.md): the full sequence on
  the test stand (pelvis held). Every stage reached its targets within 6 ticks;
  the shift moved the feet to the robot's right, as predicted. This checks signs,
  ranges and tracking, **not balance**.
- Next: the same poses on the floor with a spotter, one stage per go.
