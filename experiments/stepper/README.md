# The stepper

A host-side, quasi-static stepper for the v6 legs. It ran the robot's first free-standing forward steps on
2026-11-11 ([the record](2026-11-11-robot/README.md); tracking issue #288).

It is an **experiment and a measuring instrument**. It runs on a computer, sends joint poses over the Wi-Fi
CLI ([wifi-cli.md](../../docs/wifi-cli.md)) and reads the IMU and the servos back the same way. It is not the
control path: nothing here is in `firmware/`, and no trained policy is involved. The firmware's own guards
(calibrated range clamps, the fall guard) stay in force because every line the stepper sends is an ordinary
bench-CLI line (`pose`, `torque`, `home`, `release`, `imu`, `pos`).

"Quasi-static" means the gait is not clocked. Each phase ends when the robot's IMU says it has happened, and the
body is never relied on to carry momentum through a step.

Files:

| file | what it is |
|---|---|
| `stepper.py` | the step loops, the guards, the command line |
| `nool.py` | `Nool`: a probability, a confidence and the numbers behind them, returned where a controller would want a bool |
| `kin.py` | poses (crouch depth, pelvis offset, one offset per foot) from the one-leg experiment's IK on the as-built legs |
| `robot_io.py` | one persistent Wi-Fi CLI session: poses out, IMU and servo readings back, fall detection |
| `board_view.py` | where the soles are on the board, from one front-camera frame |
| `fake_robot.py`, `dry_run.py` | a stand-in body and the dry run (below) |
| `testdata/` | recorded camera frames the dry run measures |

## Why it exists

The roll servos have several degrees of play the encoders cannot see. Shorten one leg to lift its foot and the
pelvis sags after that foot: the body rolls toward the lifted foot as fast as the foot is raised, and the foot stays
on the floor until the play is used up. Shifting the pelvis further over the stance foot does not stop the sag
(81 to 93 mm changed it by 0.4 deg); it only puts the weight near the stance foot's outer edge, and the robot
then falls outward. There is also no contact sensor, so "is the foot off the floor" is never known, only estimated.

## One step

A step is one lift of the swing foot with the other foot as the stance foot. Left and right alternate.

1. **Shift.** The pelvis crosses over the stance foot in moves of at most `shift_step` (20 mm by default), the IMU
   read after each. The target is the model's shift times `k_shift` for that stance side. Both feet are down, so
   the body normally leans away from the side the pelvis goes to; a lean toward the stance foot by more than
   `shift_out_lim` (2.5 deg) means the stance foot is tipping over its outer edge. The pelvis then steps back
   20 mm toward the middle and that side's `k_shift` drops one step.
2. **Raise, with stance-hip trim.** The swing foot goes up in 5 mm increments (`dh`). After each increment the
   measured tilt toward the lifted foot is partly taken out by trimming the stance hip roll (`g_hike`, up to
   `hike_max` = 12 deg), so the opposite hip takes up the sag and the body comes back level.
3. **Nool estimates.** After each increment two questions are answered as `Nool`s from the commanded lift and the
   measured tilt: P(foot clear), an estimate of how far the swing sole's outer edge is off the floor, and P(tipping).
   The foot counts as up at P(clear) >= 0.9. P(tipping) above 0.5 ends the lift with a verdict:
   `tipping_out` (going over the stance foot's outside), `tipping_in` or `tipping` (hanging toward the lifted
   foot), or `max` (the full lift did not clear the floor).
4. **Confirm and balance.** A foot judged up must stay up for `t_confirm` (0.7 s). While watching, the stance hip
   keeps the body level and the pelvis shift follows the tilt (`kb`, mm more shift per degree it hangs toward
   the lifted foot, between `k_min` and `k_max` times the model's shift). If the robot settles back onto the foot
   the verdict is `fell_in`.
5. **Swing.** With the foot up, it moves forward by `dx` and to its lateral target (below).
6. **Lower.** The foot goes down in `lower_step` (10 mm) moves; the hip trim stays until the next shift takes it
   out. A step ends with a settle and an IMU read.

If a lift does not end `up`, the foot goes down and the same step is retried up to four times with `k_shift`
corrected for that stance side: more over after `fell_in`, `tipping_in` or `max`, less after `tipping_out`.
After four attempts the result is `gave_up`.

**What is remembered.** Per stance side the stepper keeps `k_shift`, the lift at which the foot was judged clear
(`h_up`), the hip trim by then (`hike`) and the learned landing miss (`comp`, below), in `state.json` beside the
script. A lift starts at half of what the previous lift on that side needed (`ff`). `k_shift` is the value at
lift-off, rising by at most half a step per lift and decaying 0.02 toward `k_floor` after a lift that needed no
balance push, so it cannot ratchet up to its ceiling `k_max` (1.4). On 2026-11-11 the left stance settled at about
11 deg of hip trim and 1.1 to 1.4 times the model's shift; the right stance at 2 to 3 deg of trim and 1.05 to
1.1 times ([record](2026-11-11-robot/README.md)).

## The camera loop

With `--board` the stepper looks at the front camera before every step. The robot walks toward the camera on a
board (about 450 mm wide) on carpet. `board_view.py` finds, in one frame:

- the board's two long edges (straight lines fitted by RANSAC to the board's per-row extent, so a sole off the edge
  or a shadow cannot drag them), with thresholds set from the frame's own statistics instead of fixed colours;
- the two soles (wide, low, pale plates; the legs above them are narrow);
- for each sole its lateral position across the board and the room left to the board's edge, and how far the feet
  are from the bottom of the picture (the board runs out of view there).

Left and right are the robot's own (it faces the camera, so its right foot is on the picture's left). A frame it
cannot read returns `None`; a wrong number is treated as worse than none, and a result carries a confidence
(lower when an edge was inferred rather than seen).

The loop around it:

- **A look** is the median of three fresh frames, low-confidence ones dropped; at least two must be good.
- **One fixed scale per run.** The board's width (`board_w_mm`, 450) over its width in pixels at the soles' row.
  Every position and margin is on that scale, so frames from different steps are comparable. (A scale from the
  soles' own width varies 5 percent or more from frame to frame because the soles stand toed out.)
- **The first look fixes the lateral targets**: the starting stance about the starting line, and `sep0`, the
  measured separation that goes with the IK's zero offsets. `--board-centre` steers that line toward the board's
  middle (`board_gain`, at most `board_step_mm` per step, no steering inside `board_dead_mm`); `--dy` moves it.
  The stance width target is clipped to 115..131 mm.
- **Aim relative to the measured stance foot.** A swing foot lands where it is sent, relative to the stance foot,
  plus a systematic miss. So the wanted real offset between the feet is computed from the stance foot's
  *measured* position, not from the commands. The `anchor` row shows the real offset against the commanded one.
- **A learned landing miss.** After each landing, `comp[side]` takes `lat_gain` (0.5) of the error between the
  aimed and the measured sole position, and is added to the next command for that foot (at most `comp_lim`, 50
  mm). Without `--board` the saved `comp` is still applied, blind. A swing foot's offset from the stance foot
  changes by at most `lat_step_max` (40 mm) between two of its swings.
- **A reach check** before each aim and each balance shift pulls the target back, in 5 mm steps, until the legs can
  reach it; the `reach_limit` row says how much was given up. If even the unchanged position cannot be reached the
  run stops.

## Guards and stop rules

- **Fall detection.** A roll or pitch far from the level reference, or the firmware's own fall latch, ends the
  run. After a fall the stepper sends nothing that moves. The firmware goes limp and homes the robot by itself
  once it is picked up (within 25 deg of upright, |gyro| below 1.0 rad/s, for 1 s; PR #287). 45 s after a fall
  with no pick-up, `notify_cmd` runs once to tell the owner; `--no-notify` turns that off. It waits for the
  homing up to `fall_wait_s`.
- **Lean abort.** A lean past `shift_lean_lim` (8 deg) during a shift ends the run with the robot still standing:
  it is centred, stood up and released.
- **Board rules** (`--board`): stop when a sole's outer edge is within `board_min_mm` (25 mm) of the board's edge,
  when the board is not seen twice in a row, or when the feet are within `board_ahead_px` of the bottom of the
  picture while walking forward.
- **Servo guard.** Every `guard_every` steps the eight load-bearing servos are read (`servos` rows). A servo over
  `temp_max_c` (65 C) or a bus under `volt_min` (10.5 V) ends the run cleanly. On 2026-11-11 the servos stayed at
  26 to 27 C.
- **Any other fault** is logged, then the robot is stood up and released.

## Modes

```
stepper.py lift --side L          raise the left foot once (right stance), free-standing
stepper.py march --steps 4        alternate lifts in place
stepper.py walk --steps 4 --dx 0.02       forward steps of dx metres (negative dx: backward)
stepper.py walk --dx 0 --dy 0.015         a sidestep: dy metres to the robot's left per step pair
stepper.py shuttle --steps 4 --cycles 3 --dx 0.02     N steps forward, N back, M times
stepper.py shiftsweep             both feet down: the pelvis shift 0 -> +62 -> -62 -> 0 mm
stepper.py playsweep --side L     lift that foot, then in the air sweep the stance hip-roll trim, then the ankle-roll trim
```

The two sweeps are measuring modes for the sim fit; their `sweep` rows are what
[the weight-transfer gate's Round 5](../policy/weight-transfer-gate/README.md) fitted. A sidestep never narrows
the stance: moving left, the left foot goes out first.

Main flags: `--side` (the foot that comes up first), `--steps`, `--dx`, `--dy`, `--cycles`, `--dx-list`
(shuttle: cycle k uses the k-th stride), `--pace X` (every phase time times X, except the first stand-up),
`--board`, `--board-centre`, `--video SECS` (record both cameras; `--board` defaults to 180 s), `--hold`,
`--no-notify`, `--host` (default 192.168.2.90) and `--set KEY=VAL ...` for any entry of the parameter table in
`stepper.py` (`DEFAULTS`, each commented).

## What exists only on `hw/stepper`

At the time of writing this directory holds the code as merged in PR #289 (`git log origin/main --
experiments/stepper`: eight commits, up to the outward-tip guard and the capped shift learning). Everything above
is that version. The later rounds are **not on main**:

On the pushed branch `hw/stepper` (two commits, `7f8ab91` and `02ab954`):

- **Turning**, `walk --dx 0 --dyaw 7.5` (degrees per step pair to the robot's left; negative is right). The soles
  are placed turned by their share, the pelvis heading follows in the next shift, and the IMU heading (fused yaw
  less drift) corrects the next swing by `yaw_gain` of its error. With `--board`, a run whose heading passes
  `yaw_board_max` (25 deg) ends, because the board is then no longer square to the camera.
- **Link-drop survival.** A dropped command is retried after a reconnect (`link_retries`). If the stand-down cannot
  be sent even on a fresh connection, the run says the robot is left with torque on in its last pose.
- **Swing-sole levelling.** The swing ankle's roll and pitch are trimmed from the IMU so the sole is level in the
  world whatever the body is doing, with a learned bias (the unloaded ankle hangs outer edge down in its play) and
  the trims released as the foot is loaded. The reason: the owner saw that the swing foot is not parallel to the
  floor and catches an edge.
- **Step-time cuts**: a single IMU read, no settle after landing, the camera look done during the landing, a short
  settle before a verdict.
- A fix to the turning bookkeeping (a re-base rotates positions, not offsets; the swing foot's x and lateral
  offset are relative to the stance foot).

Committed locally in the stepper worktree and **not pushed to any branch** when this was written:

- **A catch reflex** for a lift that ends going over toward the swing foot (`tipping_in`, `fell_in`): one fast
  move putting the foot flat on the floor, a little further outward, then a second move bringing the pelvis back
  toward the middle if the body is still going over. Its first version ran once on the robot and the robot fell
  ([record](2026-11-11-robot/README.md)); whether the revised version works has not been tried.
- `board_view` changes for warm evening light.

Treat anything in this section as unreviewed and, for the catch reflex and turning at large angles, as unproven.

## Running it safely

It moves a real machine with torque-producing servos. [AGENTS.md](../../AGENTS.md) has the bench rules; the ones
that matter here:

- **Bench power.** 2026-11-11 ran on a bench supply, not the battery.
- **A board under the robot**, on carpet, with room to fall: the robot is free-standing with no catch line. Film
  the session (`tools/cam_record.sh`; `--video` records both cameras for the run).
- **A person watching**, the robot square to the front camera and standing near the board's middle before a
  `--board` run. The camera loop does not cope with a robot turned more than about 15 deg from square.
- **Check torque and the calibration** before starting (`cal show`), and that `fall` is not latched; the stepper
  refuses to start while the firmware says FALLEN.
- **Dynamic limits.** A run is one question: do not stack changes. After a fall, pick the robot up, let the
  firmware home it (about 1 s upright and still), and start the next run fresh.
- **Learned state carries over.** `state.json` holds `k_shift`, `h_up`, `hike` and `comp` from the last run and
  is rewritten at the end of every run. A run after a fall or a changed setup may start with stale values;
  `--set` overrides them for one run.

Records land in `<main checkout>/hw_sessions/<date>/stepper/<hhmmss>_<mode>.jsonl`, plus
`_emeet.mp4` / `_sony.mp4` clips and `_board_NN.jpg` (annotated look frames) when the cameras run. The path is
fixed in `stepper.py`. Row kinds: `start` (parameters, calibration, utc and pace in later runs), `pose`, `imu`,
`nool`, `air`, `lift` (the outcome of each attempt), `board`, `board_target`, `anchor`, `landing`, `reach_limit`,
`servos`, `sweep`, `fallen`, `notify`, `recovered`, `end`. `board_view.py track <sony.mp4> --board-w 450` measures
a recorded clip.

## Dry run

```bash
JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/stepper/dry_run.py [-q]
```

Runs the lifts, a march, a forward walk, sidesteps, the board steering and stop rules (with a stand-in for the
camera) and `main()`'s argument handling against `FakeRobot`, plus `board_view` on the recorded frames in
`testdata/`. The fake body's feet land relative to the stance foot with the robot's measured misses. No robot, no
camera, no waiting. `make pytest` does not collect `experiments/`, so this is a script; its `test_*` functions
also run under `pytest experiments/stepper/dry_run.py`.

## Known limits

- **Slow.** A step takes about 11 to 15 s. About half is motion the stepper commands; the rest is the sensing
  round trips over Wi-Fi, the settles and the camera look.
- **Flat landing is not verified.** There is no contact sensor. "Up" and "clear" are estimates; whether the swing
  sole lands flat is judged from the close-up camera by eye.
- **The camera loop is fragile.** It depends on lighting, on the robot being roughly square to the board, and on
  seeing both soles. It stops the run when it cannot see.
- **The 25 mm miss is learned, not understood.** Both feet land about 25 mm to the robot's right of the aim; the
  sim does not reproduce all of it ([gate README, Round 5](../policy/weight-transfer-gate/README.md)).
- **Falls still happen** when a learned shift creeps too high or a lift is lost late; the guards reduce them and
  the firmware homes the robot afterwards, but the stepper does not catch one.
- **One robot, one board, one floor.** The numbers in `DEFAULTS` and the per-side trims are this robot's on this
  carpet, and the left and right stances differ a lot.
