# Robot measurements for the weight-transfer gate (2026-11-10 local, 2026-11-11Z; #271, #259)

A hardware session with Tom at the robot: the four measurements #271 asked for. The session log is
[raw/NOTES.md](raw/NOTES.md); the clips, stills and the flight record are in
`hw_sessions/2026-11-10/m271/` (gitignored) and `hw_sessions/2026-11-09/flight_20261110T235251Z_prec16.frec`.

**The robot:** v6 legs, 12 servos, battery in (bench power for the last clamp runs), firmware
`v1-hip60-backup-1212-g9d2a994-d` with `robot_legs_prec16_f075_js20_s128r24`, the same image as 2026-11-09.
`cal show` equals the zeros in `experiments/one-leg-stand/one_leg.py`.

## 1. Mass and floor

- Whole robot on the kitchen scale, battery in: **1621 g**. The plants weigh 1372 g (`grade`) and 1507 g
  (`train`), and carry no pack.
- The 11-09 stride was on the brown board on carpet used today (Tom).

## 2. Foot load against lateral shift (both feet down)

Left foot on the scale, right foot on a paper stack of the same height (IMU roll +0.2 deg, as on the bare
board). The poses are the 2026-10-25 lateral ladder's both-feet-down shifts, copied verbatim from
`hw_sessions/2026-10-25/lat{L,R}_serial.log` ([raw/shift_ladder.md](raw/shift_ladder.md)): crouch, then
0.5 to 1.4 times the model's 62 mm shift, 2 s moves, no balance loop. One pass per side; the scale was read by
Tom after each pose. Sim: the same poses on three plants ([raw/shift_ladder_sim.md](raw/shift_ladder_sim.md)).

Left-foot share of the total:

| pose | scale g | robot | `grade` | `train` | `grade` + `ser_hip=14.2` |
|---|---|---|---|---|---|
| stand, torque on | 817 (799 at the end) | 0.504 | 0.40 | 0.51 | 0.35 |
| crouch | 771 (795, 742 on return) | 0.476 | 0.41 | 0.51 | 0.35 |
| toward L 31.0 mm | 921 | 0.568 | 0.53 | 0.66 | 0.46 |
| 46.5 | 1024 | 0.632 | 0.60 | 0.66 | 0.53 |
| 62.0 | 1127 | 0.695 | 0.67 | 0.67 | 0.60 |
| 68.2 | 1157 | 0.714 | 0.70 | 0.68 | 0.62 |
| 74.4 | 1194 | 0.737 | 0.72 | 0.69 | 0.65 |
| 80.6 | 1249 | 0.771 | 0.75 | 0.69 | 0.68 |
| 86.8 | 1285 | 0.793 | 0.78 | 0.71 | 0.71 |
| toward R 31.0 mm | 607 | 0.374 | 0.31 | 0.35 | 0.28 |
| 46.5 | 528 | 0.326 | 0.27 | 0.34 | 0.26 |
| 62.0 | 447 | 0.276 | 0.24 | 0.34 | 0.24 |
| 68.2 | 417 | 0.257 | 0.22 | 0.32 | 0.23 |
| 74.4 | 379 | 0.234 | 0.21 | 0.31 | 0.23 |
| 80.6 | 346 | 0.213 | 0.20 | 0.31 | 0.22 |
| 86.8 | 313 | 0.193 | 0.19 | 0.30 | 0.21 |

- The robot is laterally even at the stand; `grade` puts 0.60 on the right foot there.
- The share moves linearly with the commanded shift from the first rung: 4.0e-3 per mm toward the left,
  3.3e-3 per mm toward the right. `train` (roll lash at its DR centre) is nearly flat after the first rung.
- At 62 mm the stance foot carries 0.70 to 0.72 of the weight, at 86.8 mm 0.79 to 0.81; the other foot still
  carries 313 to 336 g.
- Hysteresis: the crouch read 771, 795 and 742 g by direction of approach (about 1.7 % of body weight).
- With both feet flat and position-held the split is statically indeterminate (the ankle-roll torques move
  load between the feet), so this is a curve for the plant to reproduce, not a centre-of-mass measurement.

## 3. The 11-09 stride, repeated with a floor camera

`arm_script.py --phase script --script stride1_02s --settle 3 --duration 5 --tail 0.5` from
`deploy/prec16-f075`, after the per-boot prep (`standclock coast 1.5708`, `shape 8`; the 11-09 flight ticks
show the coasted clock, and shape 8 on that day is inferred from the prep script setting both). Pack 11.3 V
(11-09: 11.7 to 11.8 V). Flight ticks of the run: [raw/stride_r1_ctrl_ticks.csv](raw/stride_r1_ctrl_ticks.csv).

| t from the first vx = 0.2 tick | today | 11-09 |
|---|---|---|
| rate guard | 3.32 rad/s at +0.66 s, WalkRecovery | 3.02 at +0.64 s |
| gyro z peak | -7.26 rad/s at +0.62 s | -7.20 at +0.62 s |
| yaw (integrated gyro z) at +0.5 / +0.7 / +2.0 s | +11 / -43 / -43 deg | +9 / -43 / -45 |
| largest tilt (acos up_z) | 14.4 deg at +0.60 s | 14.2 at +0.60 s |
| tilt at +0.3 / +0.5 s | 5.6 / 11.3 deg | 5.8 / 11.8 |
| knee range R / L | 48.2 / 59.5 deg | 47.6 / 57.1 |
| hip roll range R / L | 8.4 / 4.5 deg | 7.9 / 4.2 |

The run repeats 11-09 to 1 to 2 deg and 0.1 rad/s.

**The feet.** The EMEET stood at floor level on the robot's left, so its far foot is the right one
([figs/stride_r1_emeet_C.jpg](figs/stride_r1_emeet_C.jpg), zoomed on the right foot in
[figs/stride_r1_emeet_Rfoot.jpg](figs/stride_r1_emeet_Rfoot.jpg); the Sony, on the right, in
[figs/stride_r1_sony_C.jpg](figs/stride_r1_sony_C.jpg)):

- +0.25 to +0.38 s: both feet flat.
- From about +0.42 s the right foot's plate tilts and the end the camera sees rises off the board; at +0.52 to
  +0.59 s it is raised and tilted roughly 10 to 15 deg, and at +0.59 s it looks wholly off the board. Its other
  end is hidden behind the left foot until then.
- +0.59 to +0.76 s: the left foot is in the air while the body yaws; the right foot is down again, rotated
  about 45 deg.

So on an identical run the right foot does leave the floor, tilted and briefly. The torso tips 11 to 14 deg
in the first 0.6 s, before the yaw spike.

**Not measured:** a height in mm. The EMEET clip is 640x480 (`cam_record.sh`'s default; `SIZE` was not set)
and there was no scale in the frame.

**After the run.** The robot stood from about +2 s to +4.4 s. The script's disarm came at +3.5 s; at +4.8 s
the robot was toppling and Tom caught it ([figs/stride_r1_sony_end.jpg](figs/stride_r1_sony_end.jpg)).
`arm_script --phase script` ends with a bare ARM = 0 disarm, which releases torque where the robot stands.

## 4. The clamp test: tilt beyond the encoders

The 2026-10-24 protocol (`hw_sessions/2026-10-24/stand/`, Parts 5 and 6): one foot clamped flat to the desk,
the other lifted straight up with hip pitch, knee and ankle only, no shift; today's roll zeros. Torso roll from
the IMU; full-resolution Sony stills from the front
([figs/clamp_R_straight.jpg](figs/clamp_R_straight.jpg), [figs/clamp_R_liftL30.jpg](figs/clamp_R_liftL30.jpg),
[figs/clamp_L_liftR30.jpg](figs/clamp_L_liftR30.jpg)).

Torso tilt toward the lifted side:

| | 15 mm | 30 mm | 10-24 at 15 / 30 mm | stance ankle roll / hip roll encoder give at 30 mm |
|---|---|---|---|---|
| right foot clamped, left lifted | 7.1 deg | 9.7 deg, steady | 4.1 / 5.0 | 17 ticks (1.5 deg) / 8 ticks (0.7 deg); 10-24: 17 / 10 |
| the same after Tom tightened every ankle screw (servo and horn) | 6.75 | 9.35 | | 18 / 8 |
| left foot clamped, right lifted | 5.2 | 9.36, steady | 4.4 / 5.2 | 18 ticks (1.6 deg) / 5 ticks (0.4 deg); 10-24: 12 / 8 |

- At 15 mm on the first side the lifted foot was back on the desk on its outer edge, and the clamped foot was
  flat (Tom): that tilt is the limit of the lift, not an equilibrium. At 30 mm the lifted foot was in the air.
- The encoders give what they gave on 10-24. The torso tilts 4 to 4.7 deg more, so about 7 to 7.5 deg of the
  9.4 deg is beyond the encoders (10-24: about 2.6).
- Both sides give about 9.4 deg, so it is not one loose or damaged part, and tightening the ankle screws moved it
  by 0.35 deg.
- Where it sits, right foot clamped, from the stills (`raw/angles.py`, Hough edges of the leg links; the two
  baselines differ by 0.9 deg, so +-1 deg): foot to shank 5.6 deg (encoder 1.5), shank to thigh about 2.0,
  thigh to torso about 2.1 (encoder 0.7). The other side gave too few lines to split.
- With the left foot clamped the torso also pitched about 4 deg; the other side showed none.

Known differences from 10-24: the battery is in; the four roll zeros were moved 2 to 3 deg after the 10-24
clamp runs (the 18:14Z fix), so "straight" is a different roll pose; the right hip yoke was replaced by the redesigned one (the left is still the original; see the yoke record §7) and a
leg link replaced (11-06); the desk had no mat and is not level. Which of these accounts for the extra tilt is
not established.

## What this gives the gate (#259)

| quantity | robot today | `grade` | `train` |
|---|---|---|---|
| total mass | 1621 g | 1372 g | 1507 g |
| L-foot share at the stand | 0.50 | 0.40 | 0.51 |
| share slope, both feet down | 3.3 to 4.0e-3 per mm, linear | close (0.53 to 0.78 over the ladder) | flat after the first rung |
| clamp tilt, 30 mm | 9.4 deg (about 7 beyond the encoders) | 2.3 deg | 4.2 to 5.0 deg |
| stride: largest tilt | 14.4 deg at +0.60 s | not yet compared | not yet compared |
| stride: right foot | lifts tilted from +0.42 s | lifts about 2 cm | lifts about 2 cm |

The series spring fitted in round 2 (`ser_hip=14.2`) was fitted to the 10-24 clamp tilt of 5.1 deg; today's
robot gives 9.4.

## Limits

- One session, one pass of each measurement; the scale was read by eye to 1 g.
- No leg was weighed, so the mass split is still unmeasured.
- The stride gives the feet's timing and attitude, not a height.
- The clamp split by segment is from soft-focus stills on one side only.
