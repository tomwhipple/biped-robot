# The stepper on the robot, 2026-11-11 (#288)

A day of hardware work at the robot with the owner: the first free-standing forward steps, driven from the host
by the quasi-static stepper ([`experiments/stepper/`](../README.md)). No policy walked; the stepper did. The
tracking issue is #288. The stepper itself merged to main as PR #289 at round 7; rounds 8 and 9 (turning,
link-drop retry, sole levelling) are on the branch `hw/stepper`, and the catch reflex is committed locally in
the stepper worktree and was not pushed when this was written.

Times are local, MST (UTC-7), as in the log file names. Every number below is from the run logs unless it says
otherwise; where the logs and the session's own account disagree, the log is given and the difference said.
The raw data is in `hw_sessions/2026-11-11/stepper/` (gitignored).

## 1. Setup

- **Robot:** v6 legs, 12 servos, on **bench power** all day. Firmware `v1-hip60-backup-1297-g1ed8ad8` from
  11:10, when the robot was reflashed with the new pick-up rule (PR #287): after a fall the firmware goes limp and
  homes the robot by itself once it is picked up and upright within 25 deg, |gyro| below 1.0 rad/s, for 1 s.
  From 11:10 it did so after every pick-up. Before that the robot was homed by hand.
- **Floor:** a brown board about 450 mm wide on carpet in the owner's office. The camera loop's one scale is that
  width (`--board-w 450`); it was not checked against a ruler.
- **Cameras:** a Sony in front of the robot, looking along the board (the camera loop and the landing tracks), and
  an EMEET at floor level as a foot close-up. Both were recorded for each run.
- **Servos:** 26 to 27 C at the end of the afternoon; the bus read 11.9 to 12.0 V in the servo guard rows.
- **Code:** ran from frozen copies of `experiments/stepper/` (the working tree was being edited). The proven copy
  for straight walking was the round-7 code (`a80cbdb`).

## 2. Timeline

Rows are the runs that matter. "Up" counts lifts that ended with the foot judged up. File stamps are the start
of the run.

| time | run | what | outcome |
|---|---|---|---|
| 10:12 | `101255_lift` | first single lift, left foot, fixed shift 1.3x | up at 25 mm |
| 10:14 | `101413_lift` | right foot lift | three `tipping_in`, up on the fourth try at 1.6x; fell at 10:14 while lowering; the early code had no fall detection and kept commanding |
| 10:15 | `101535_march` | first march | left up, right `tipping_in`; on its side from about 10:16, no fall detection |
| 10:27 | `102724_dry` | dry run | the stepper's own first dry run |
| 10:28 | `102749_lift` | lift | fell (see section 6) |
| 10:37 to 10:41 | `103709`, `104032`, `104147_lift` | lifts with stance-hip trim | all up; hike L 10.5 deg (at 10:41), R 5.3 deg |
| 10:43 | `104303_march` | march | 4 up, one `fell_in` retried |
| 10:44 | `104435_walk` | **first free-standing forward walk**: 4 steps, feet advanced 60 to 80 mm | all up |
| 10:46 | `104632_walk` | walk, 4 steps | all up, then a lean abort and a fall at 10:47 |
| 11:00 | `110045_walk` | walk, 6 steps, first camera look (7 looks) | all up; feet advanced 100 to 120 mm; ended when the board was not seen twice |
| 11:07 | `110708_walk` | sidestep, 4 steps | all up |
| 11:10 | | reflash | firmware above |
| 11:16 to 11:27 | `ladder_1120.log`, `ladder_1135.log` | lift ladders: shift 0.9x to 1.5x, hip trim on and off, both sides | table in section 4; two falls |
| 11:28 | `112826_walk` | walk, 6 steps, camera | all up; stopped by the edge rule (a sole 13 mm over the board's edge) |
| 13:02 | `130223_march` | first march with the landing-miss learning | 6 up |
| 13:05 to 13:12 | `130557_shiftsweep`, `130837`, `131111_playsweep` | measurement sweeps for the sim fit | section 4 |
| 13:25 | `132546_march` | march back to the middle, 10 steps | 10 up (11 lifts); right margin 37 mm at the start, 80 mm and 113 mm at the end |
| 13:29 | `132916_walk` | walk, camera, 9 steps | 9 up, margins 72 to 115 mm; fell on the tenth, 13:31 |
| 15:41 | `154103_walk` | walk, 10 steps, camera, pace 1.0 | 10 up, margins 52 to 148 mm |
| 15:44 | `154454_shuttle` | shuttle: 2 cycles of 4 forward and 4 back, pace 0.8 | 16 up (19 lifts), margins at least 90 mm |
| 15:50 | `155007_walk` | pace 0.6 | no step: 4 lifts, `fell_in` and `tipping_out` |
| 15:53 to 16:01 | `155337`, `155722`, `160059_walk` | pace 0.7, backward and forward | 4 up (ended by a link drop), 8 up backward, 4 up (ended by the board rule) |
| 16:11 to 16:16 | `161112`, `161322`, `161649_walk` | turning | section 4; the last fell at 16:20 |
| 16:50 to 17:10 | `165040` to `171013` | round 9 with swing-sole levelling | a 4-step and a 6-step march, two board-rule stops, two falls (16:56, 17:10) |

## 3. Method in one paragraph

Each step is a monitored lift: shift the pelvis over the stance foot, raise the swing foot in 5 mm increments
with the stance hip trimmed against the measured tilt, judge the foot up from probabilities (no contact sensor),
confirm it stays up, swing it, lower it. The camera loop aims each swing foot relative to the measured stance foot
and learns each foot's landing miss. All of it is described in [the stepper README](../README.md).

## 4. Measurements

### Hip trim and shift (what the lift needed)

From the `end` rows of the runs that walked (10:44 to 13:29, then 15:41 to 17:07):

| stance | stance-hip trim at the lift | shift, times the model's | lift at which the foot was judged clear |
|---|---|---|---|
| left (right foot swings) | 10 to 11.2 deg at pace 1.0 in the morning; 6 to 10 deg in the slower afternoon runs | 1.24 to 1.38, peaking at 1.5 at 13:25 | 16 to 23 mm |
| right (left foot swings) | about 2 to 5 deg | 1.02 to 1.18 | 12 to 17 mm |

The trim is the hip taking up the sag of the play-loaded roll chain. With the left stance at about 11 deg of trim
the body stood level; the sim cannot reproduce that (section 8). In the 11:20 ladder at 0.9x the shift, with no trim,
the foot never cleared and the tilt reached 12 to 15 deg.

### Where the feet land

- **Blind** (no camera): both feet land about 25 mm to the robot's right of the aim. Measured from the Sony
  clips (`board_view.py track`, 4 runs, 9 steps per foot): the left foot's move relative to the commanded lateral
  move was -23 mm (range -40 to +22), the right foot's -27 mm (-43 to +12) (weight-transfer-gate README,
  Round 5). A march in place drifts about 30 mm per pair of steps. The 11:07 sidestep was commanded 30 mm left
  and ended about 38 mm right, both feet. These figures come from the issue and that README; I did not recompute
  them from the clips.
- **With the camera loop:**
  - 13:25, a 10-step march took the robot from a right margin of 37 mm back toward the middle (80 mm right, 113 mm
    left at the last look).
  - A 9-step walk (13:29) and a 10-step walk (15:41) held the centre line, with the smallest margin at either
    side 72 mm and 52 mm. The 9-step walk then fell (section 6).
  - A 16-step shuttle (15:44, pace 0.8) kept at least 90 mm of margin.
  - An 8-step backward walk (15:57, pace 0.7) ran without a fall; the first lift had one `fell_in` retry.
  - The learned landing miss `comp` ended the day at 10 mm (left foot) and 20 to 25 mm (right foot).

### Pace ladder

All phase times scaled by `--pace`:

| pace | run | steps up | median time between lifts |
|---|---|---|---|
| 1.0 | `154103_walk` | 10 | 13.8 s |
| 0.8 | `154454_shuttle` | 16 | 10.9 s |
| 0.7 | `155337`, `155722`, `160059_walk` | 4, 8, 4 | 12.2, 11.0 and 11.5 s in the three runs |
| 0.6 | `155007_walk` | 0 | the run ended in 35 s |

1.0, 0.8 and 0.7 work. At 0.6 the first lift fails: verdicts are read while the body is still moving.

### Step time and where it goes

At pace 1.0 a step took 12 to 15 s between lifts (median 13.8 s in the 15:41 walk). The poses the stepper
commanded in that time add up to a median 6.7 s; the rest is settles, holds, the camera look and the sensing
round trips over Wi-Fi. At pace 0.7 it was about 11 s, of which about 4.3 s commanded. (The commanded figure is
the sum of the `ms` of the `pose` rows between two lifts; holds and sleeps are not in it.) The turning walk at
pace 1.0 took 13.4 s, of which 8.1 s commanded.

### Turning (round 8, branch `hw/stepper`)

Gyro heading is the IMU's fused yaw less drift; commanded is the heading the run was steering to at its last
heading row.

| run | steps up | commanded | gyro heading |
|---|---|---|---|
| `161112_walk` | 4 | -6.2 deg | -6.1 deg |
| `161322_walk` | 10 | -41.7 deg | -27.2 deg |
| `161649_walk` | 15, fell on the 16th | +76.0 deg | +51.4 deg at the last logged heading before the fall |

So a turn delivered about two thirds of what was commanded. The owner reports about 40 deg of turn over the 10
steps; that is not from the logs and the camera view of it was not measured. Over a 6-step straight walk the
bias-corrected gyro heading stayed within 3 deg (issue #288, not rechecked); after the shuttle and the backward
walk the robot had turned about 45 deg in total, seen on the camera, which is what stops the camera loop (it no
longer sees the soles as plates).

### Shift sweep and play sweeps (13:05 to 13:12)

The data for the sim fit. Roll is positive leaning to the robot's right.

- **Shift sweep, both feet down.** The pelvis shifted 0, +62, -62, 0 mm in 6 mm steps, twice. The body leans
  away from the side it is shifted to: mean roll -3.1 deg at -62 mm to +1.4 deg at +62 mm (4.5 deg over 124 mm,
  about 0.036 deg per mm; the gate README gives 0.03 to 0.05 and 4.7 deg peak to peak).
- **Play sweeps, one foot up.** The stance hip-roll trim, then the stance ankle-roll trim, swept +-4 deg in 0.5 deg
  steps, twice, each side. The body follows the hip trim at 0.83 / 0.45 deg per deg (right stance, above / below
  zero) and -0.89 / -0.89 (left stance), with under 1 deg of hysteresis. The ankle trim is one-sided on the right:
  +3 deg of trim tilts the body 3.5 deg (from -3.07 to +0.44), -4 deg only 0.8 deg. Per-unit figures are those
  of the gate README, Round 5; the totals I rechecked from the `sweep` rows.

## 5. The swing foot is not parallel to the floor

The owner observed that the swing foot is not parallel to the floor and catches an edge as it comes down. The
unloaded ankle hangs outer edge down in its play, and the body's own tilt tips the sole further. What was done
(branch `hw/stepper`, `02ab954`): the stepper trims the swing ankle's roll and pitch from the IMU so the sole is
level in the world, with a learned bias (2 deg outer edge up to start), and lets the trims go as the foot is
loaded. The runs from 16:50 on had it (trims up to 8 deg in the logs). Whether the sole now lands flat has not
been verified; see section 9.

## 6. Falls

Ten falls, all on carpet. The firmware's fall guard latched every time. The cause is as
established in the session; the "log" column says what the log shows.

| time | what the log shows | cause | response in code |
|---|---|---|---|
| 10:14 | roll -79 deg at "lower R" of the right-foot lift, at 1.6x shift | over-shift with a fixed shift; no fall detection, the script kept commanding | fall detection that ends the run; the shift became learned and bounded |
| 10:16 | roll -68 deg during the march, from about 27 s; IMU glitch rows | the same early code | the same |
| 10:28 | lowering the left foot with the pelvis 103 mm over; roll +46 deg | a wrong pelvis-roll IK option stood the legs back upright, then the 103 mm shift was re-applied | the IK option corrected |
| 10:47 | a lean abort (+11.3 deg during the shift to the right), then roll +73 deg during the centring move | the owner's account: the blind walk went off the board's edge; the log shows the lean abort first | the board-edge watch with the front camera |
| 11:17 | pitch -82 deg (backward) at "lower L" of a ladder lift at k = 0.9, hip trim 12 deg | a deliberately small shift in the ladder | none; it set the lower end of the shift |
| 11:26 | roll +85, pitch -100 at "lower R"; the log has the right foot at 35 mm, hip trim 0, shift 1.5x | the right foot lifted with no hip trim (ladder) | the hip trim is always on in the stepper |
| 13:31 | a lean abort (-9.7 and -7.2 deg during the shift to the left), then roll -77 deg; the learned left-stance shift had crept to 1.5x | outward over the left foot | an outward-tip guard in every shift over a stance foot, `k_shift` remembered at lift-off with a capped rise and decay, `k_max` 1.4 (round 7); no outward fall since |
| 16:20 | roll -82 deg at "shift L", the 16th step of the turn, 51 of 76 deg turned | two frame errors in the turning bookkeeping | fixed in round 9 (`02ab954`, `hw/stepper`) |
| 16:56 | pitch -70 deg (backward) at "lower L" after a `tipping_in` | the body tipped onto the swing foot and the lowering was slow | the catch reflex (local, below) |
| 17:10 | `fell_in` on the left foot, a catch (placed 15 mm out, sole trims +8.0 and -1.9 deg), a stance shift back from 71.5 to 51.5 mm, then roll +84 deg at "over R" | the first version of the catch drove the foot into the floor and kicked the robot over the stance foot (the session's reading; the log shows the sequence, not the contact) | the catch reworked: earlier tipping-in detection, `k_floor` 1.1, free sole trims on a tipping verdict (local commit `1d46211`) |

The session account put the last two at 16:57 and 17:13. The notify rows in the logs say 16:56 and 17:10. The
same account put the first two at 10:15 and 10:16; the logs give 10:14 and 10:16.

## 7. Firmware pick-up rule

The pick-up rule of PR #287 came from three bench falls in the flight record
`flight_20261111T171003Z_prec16.frec` (started 17:10 UTC, 10:10 local, so these are the morning's falls): the old rule (10 deg, 0.35 rad/s, 2 s) never fired for a robot held in
hand and set down limp at 21.7 deg. The new rule fired within 4.4 s of the pick-up on that fall in replay. On
hardware it homed the robot by itself after every pick-up from 11:10; the stepper only waits and pings the owner
if the robot stays down 45 s.

## 8. What the sim refit concluded

The stepper's runs were replayed open loop in the gate plant and the roll chain refitted per side
([weight-transfer-gate README, Round 5](../../policy/weight-transfer-gate/README.md); PR #290, #286). Per-side
play in the roll servos (left hip 6, right hip 8, left ankle 3, right ankle 0 deg) fits the single-support trim
gains; a series spring does not, and a lateral centre-of-mass offset and ankle friction are not supported as the
cause of the left-right asymmetry. No term reproduces the level body on the left stance under 11 deg of trim, the
lean with pelvis shift in double support (robot 0.03 to 0.05 deg per mm, the plants at most 0.02), or two of the
three falls it was scored on.

## 9. Tonight's training queue (2026-11-11)

On the per-side play plant; it replaces the spring-plant march arms. The base is the queued `prec67_mrs_cont`
line (prec59's march recipe, 300M steps from a warm start of `robot_legs_prec59_march_rs_s1`). Every arm changes
the same two flags against it: `--roll-series-k` becomes
`--roll-play-deg L_hip=4:10,R_hip=6:10,L_ankle=2:3,R_ankle=0:2`, and `--roll-lash-deg` is dropped. Each arm runs
two seeds (14 and 15), 300M steps, then its s128r24 distill.

| arm | out | changed against the first arm |
|---|---|---|
| P1 play-fit march | `robot_legs_prec68_march_pp` | none |
| P2 slow gait | `robot_legs_prec69_slow_pp` | `--gait-freq 0.5,0.55`, `--mimic-sway 0.05` |
| P3 slow gait and lift above the sag | `robot_legs_prec70_slowlift_pp` | P2 plus `--mimic-march 0.05`, `--swing-height 0.035` |
| P4 mirror loss on play | `robot_legs_prec71_mir_pp` | `--w-mirror-loss 1.0` |

Grading in the morning uses `grade.py` as is and again with
`GRADE_PLANT_TERMS=play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0`, judged on lifts with a level body.
Nothing is known about learning on this plant: the warm start is a spring-trained policy at 0.7 to 0.8 Hz.

## 10. Not verified or open

- **Flat landing.** The swing-sole levelling exists; no measurement shows the sole lands flat. The close-up clips
  have not been reviewed for it.
- **Camera detection is fragile in evening light.** After about 16:50 several runs ended on "the board was not
  seen twice in a row" or on an edge reading (`170355_walk` stopped after 12 s on a sole read 72 mm over the
  edge; whether that reading was right is not known). Changes for warm light are committed locally, not pushed or tested on the robot.
- **Step time is 12 to 14 s** at pace 1.0 and about 11 s at 0.7; below 0.7 it fails. About half is not motion.
- **The catch reflex is not proven.** Its first version fell; the reworked version has not run.
- **Turning delivers about two thirds of the commanded angle** and ended in a fall at 51 of 76 deg; the cause
  found there was fixed in round 9, which has not turned the robot again.
- **The sim cannot explain** the level stance at 11 deg of left trim, the lean with pelvis shift in double
  support, or two of the falls.
- **The 25 mm rightward miss** is measured, not explained.
- **The camera scale** rests on the board being 450 mm wide; it was not checked against a ruler.
- **A "9 forward steps holding the centre line"** figure in other notes matches the 13:29 walk here (9 up,
  margins 72 to 115 mm), which then fell on the tenth step.
- **Link drops.** One Wi-Fi drop (15:53) ended a run without a stand-down; the robot was fine and was homed by
  hand. The reconnect-and-finish fix is on the branch, untested on the robot.

## 11. Data

All under `hw_sessions/2026-11-11/stepper/`, gitignored:

- `<hhmmss>_<mode>.jsonl`: the log of each run (46 files), one JSON row per event.
- `<hhmmss>_<mode>_sony.mp4`, `_emeet.mp4` and their `.pts` timestamps: the front camera and the foot close-up.
- `<hhmmss>_<mode>_board_NN.jpg`, `_latest.jpg`: annotated camera looks of the board runs.
- `ladder_1120.log`, `ladder_1135.log`: the console output of the two lift ladders.

Beside it: `hw_sessions/2026-11-11/HANDOFF.md` (the state at 16:30), `flash_prec16_main_pickup*` (the 11:10
reflash) and the flight records (`hw_sessions/2026-11-09/flight_*.frec`) that the sim replay reads.
