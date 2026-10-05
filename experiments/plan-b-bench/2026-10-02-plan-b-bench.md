# Plan B bench, session 3 (2026-10-02): hold on the v3 rig

Issue #73. This session repeats session 2's hold test ([2026-10-01-plan-b-bench.md](2026-10-01-plan-b-bench.md),
run D) on the v3 rig (`plan_b_rig_v3.py`, 48fcb73):

- **The rig:** the servo is gripped on both case faces, and the lever is a fork
  on the horn and the idler disc.
- **The load:** rigid washer weights on the servo's mid-plane, instead of the
  C-clamp.

How the session ran:

- **Operator:** the Claude session on Mira ran it as `claw`. Tom set up the rig
  and gave the goes.
- **Script:** `2026-10-01/hold_clamp.py`, now taking level and plumb as inputs.
- **Video:** every servo session was filmed (`tools/cam_record.sh`, `/dev/video0`).
- **Raw data** (gitignored) is on Mira under `hw_sessions/2026-10-02/gain_bench_hold/`.
  The per-pass numbers are in `2026-10-02/hold_v3_summary.json`.

**Result.**
- **P 160 still limit-cycles.** This time it happens hanging plumb (range 14
  ticks, moving in 145 of 152 samples, load ±284), not level.
- **P 128 / D 32 holds still quietly** both level and hanging, and its 2° moves
  settle. Its 90° sweeps are rough: speed ran from −950 to 1200 steps/s against
  a commanded 100.
- **P 96 / D 32 is nearly clean:** it holds, settles, and raises without a
  backward sample; lowering had 15 of 248.
- **The rig change helped below P 160.** Against the C-clamp on the v2 rig
  (session 2), P 96 and P 128 now settle while hanging, and P 96's sweeps are
  close to clean.

## The rig and the load

- **Rig v3:** a leg_link-style case grip on both faces, closed into a box, and
  a fork lever on the horn and the idler disc
  ([plan_b_rig_v3.py](plan_b_rig_v3.py)). It is clamped to the desk edge.
- **Weights:** 2 × M10 bolts through the fork's floor, 16 washers on each, on
  the servo's mid-plane. They are rigid and nothing hangs.
  - Their radii (the fork has holes at 40 / 70 / 100 mm) and their mass are not
    recorded.
  - At level, the P 32 probe midpoint was **3.5 ticks**, against 8.5 for
    session 2's C-clamp. That puts their torque at about **0.4×** the clamp's,
    roughly 0.18 N·m by session 2's P 32 stiffness, which carries the unweighed
    bottle mass.
- **Servo:** the same STS3215, ID 30, on the same board.

## Finding level and plumb

**1. From the holding load (09:47–09:53): wrong by 153 ticks.**
- The script found which way gravity pulls by probing ±40 ticks at the rest
  position (3063). The +40 side stopped 6 ticks short with load 56; the other
  side, 0 and 0.
- It then stepped 64 ticks at a time against gravity, reading the holding load.
  The load rose from 56 to a plateau of 96–104 between 2679 and 2551. A parabola
  through the top three points put level at **2660**, so plumb at 3684.
- The plateau is flat and noisy, so the peak was poorly defined.

**2. Incident.**
- The script then lowered toward 3684 and the fork pushed against the C-clamp
  that holds the stand to the desk. The load reached −388.
- The script released at about 3619, and stopped at its next prompt.
- 3684 turned out to be about 13° past true plumb (step 4), toward that clamp.

**3. From the camera: inconclusive.**
- A sub-pixel edge fit of the fork (`2026-10-01/edge_angle.py`; numpy and
  scipy) agrees with itself to within its error bars: +1.2° and +1.7° from image
  vertical.
- Two vertical references disagreed with it:
  - a long edge beside the fork leaned +4.2°;
  - a vanishing point from the room's vertical edges did not converge (median
    residual 88 px).
- The thin plumb string could not be picked out of the frame reliably.

**4. Plumb by eye: 3531.**
- Tom aligned the fork by hand to a hanging string. With the fork released
  there, the servo read **3531**.
- Plumb is therefore 3531, and level is **2507**. Gravity pulls toward higher
  ticks.
- Accuracy is that of an eye against a string at close range, about a degree
  (about 11 ticks).
- **Check:** in the run, the hanging quiet hold at P 32 and P 96 read 0 ticks
  off with load 0.

## The hold ladder (10:02–10:23)

D 32 throughout. Every move was acceleration-limited (register 41 = 2), and every
move was traced. At each P:

1. Write the gains, hanging plumb and released.
2. Quiet hold for 5 s, then four 2° moves.
3. Raise to level: quiet hold, two slow probes, four 2° moves.
4. Lower and release.

| P / D | holding still, level | holding still, hanging | 2° moves level: worst overshoot / residual | 2° moves hanging | raise: speed range, backward samples | lower: speed range, backward samples |
|---|---|---|---|---|---|---|
| 32 / 32 | 0 | 0 | 0 / 0 | 0 / 0 | 50..150, 0/312 | 50..150, 0/313 |
| 96 / 32 | 0 | 0 | 0 / 0 | 0 / 0 | 50..200, 0/310 | −300..500, 15/248 |
| 128 / 32 | 0 | 0 | 1 / 1 | 1 / 0 | **−950..1200**, 74/291 | −800..1000, 70/339; ends 8 over, residual 16 |
| 160 / 32 | 0 | **14, moving 145/152, load ±284** | 0 / 0 | 6 / 12, **never settle** | −350..500, 53/254 | −350..500, 84/329 |

- **Holding still** is the position range in ticks over 5 s. No status bit was
  set in any pass.
- **Probe midpoints at level** were +3.5 (P 32), +1.0 (P 96), +1.5 (P 128) and
  +1.0 (P 160) ticks. The friction band at P 96 and above is within the encoder's
  ±1 count, so these do not give a usable stiffness ratio at this load. Session 2's
  bottle runs remain the stiffness measurement.
- **Video:** `plumb_read.mp4` (the position read) and `run2.mp4` (the ladder).
  - The script's own start reads at 10:02:27 may fall just after `plumb_read`
    ended. `run2` failed to start because the camera was still busy, and was
    restarted at about 10:03, before any motion.
  - The calibration attempt is in `run1.mp4` (09:47–09:56).

## Reading

- **Rig and load.** With the servo gripped on both faces and the load rigid on
  the mid-plane at about 0.4× the C-clamp's torque, the 3215 is quiet at hold
  and settles after small moves up to **P 128 / D 32**. That is about 3.5× the
  stock stiffness (session 2), in #73's conditional band.
- **P 160 / D 32** (about 4.8×) is not stable. It limit-cycles hanging plumb,
  the zero-preload case where the gear play floats. In session 2, on the v2 rig
  with the heavier clamp, it limit-cycled level instead.
- **Slow 90° sweeps** at 100 steps/s are clean only up to P 96 (about 2.8×).
  At P 128 the servo's reported speed swings by about ±1000 steps/s during the
  sweep.
- **What is not known:**
  - the weights' mass and radius, so the inertia relative to #73's leg-like
    0.25 kg at 0.1 m is not known;
  - whether a walking policy's 50 Hz targets behave like these 100 steps/s
    sweeps.

## Sweep roughness vs motion profile and damping (10:52–11:09)

Tom: "test the sweep roughness at P128 with smoother motions and/or dampening".
Script: `2026-10-02/sweep_smooth.py`. At each gain setting the arm swept plumb
→ level → plumb, 90° each way, with the same weights and calibration as above.
Gains were written and torque released only at plumb. The three profiles:

- **A:** one `move` at 100 steps/s with acceleration register 2
  (200 steps/s²): session 3's sweep.
- **B:** the same with acceleration 1 (100 steps/s²), so gentler ramps.
- **C:** host-streamed minimum-jerk targets over the same 10.24 s (100 steps/s
  average). One `move` per cycle, at the trajectory's own speed, the way a
  policy streams targets. It runs at about 18 Hz against A/B's 30, because each
  cycle also sends a target.

**Metrics,** from servo telemetry through each sweep:
- **backward samples:** reported speed against the sweep direction;
- **jitter:** RMS of position minus its 0.4 s moving average, in ticks
  (1 tick = 0.088°). For C this includes under 0.5 tick from the curve itself;
- **load range.**

They see rotation about the servo axis only, below about 15 Hz (9 Hz for C),
and nothing under 1 tick.

| gains | A: backward raise / lower, jitter | B: backward, jitter | C: backward, jitter | load range |
|---|---|---|---|---|
| P 32 / D 32 | 0 / 0 of ~300, 1.2–1.3 | 0 / 0, 1.1 | 0 / 0 of ~118, 0.8–0.9 | within −60..116 |
| P 128 / D 32 | 21 / 20, 2.0–2.1 | 24 / 28, 1.9–2.0 | 17 / 18, 2.4–2.5 | −228..228 |
| P 128 / D 64 | 42 / 21, 2.2–2.4 | 50 / 30, 2.2–2.3 | 18 / 17, 2.6–2.7 | −256..272 |
| P 128 / D 128 | 104 / 30, 2.6–3.4 | 102 / 89, 3.4–3.5 | 31 / 17, 2.8–3.0 | −384..416; ends ring 10–12 ticks |
| P 128 / D 32, repeat (11:01) | 20 / 19, 1.8–2.0 | — | 11 / 12, 2.1–2.3 | −228..228 |
| P 128 / D 16 | 18 / 14, 1.7–2.0 | — | 13 / 8, 1.9–2.2 | −186..218 |
| **P 128 / D 0** | **15 / 7, 1.6–1.8** | — | **9 / 5, 1.6–1.7** | −176..176 |

- **Smoother commands do not remove the roughness at P 128.** B is the same as
  A. C cuts the backward count but not the jitter.
- **More D makes it worse, less D makes it better.** D 128 roughly doubles the
  jitter of D 32. D 0 is the smoothest P 128 setting, though still above P 32.
- In angle the jitter is small: about 0.15–0.2° RMS at P 128.

## Hold with D 0 (11:05–11:09)

`hold_clamp.py` with the D 0 ladder, the same rig and weights.

| P / D | holding still, hanging | 2° moves hanging | holding and 2° moves, level | raise: backward | lower: backward |
|---|---|---|---|---|---|
| 128 / 32 (10:02, above) | 0 | settle, overshoot ≤ 1 | quiet, settle | 74/291 | 70/339 |
| **128 / 0** | 0 | 3 of 4 settle; one keeps moving (residual 6) | quiet, settle | **17/295** | **12/260** |
| 160 / 32 (10:02, above) | **14, moving 145/152** | never settle | quiet, settle | 53/254 | 84/329 |
| 160 / 0 | **0** | 3 of 4 never settle (residual 6–7, moving 29–30 of the last ~30 samples) | quiet, settle | 25/255 | 69/340 |

- **D 0 removes P 160's limit cycle at rest,** but a 2° move while hanging (no
  gravity preload) starts an oscillation that does not die out.
- **P 128 / D 0 sweeps are about 4–6× smoother than P 128 / D 32,** and it holds
  quietly in both positions.

**Files:**
- Per-run metrics: `2026-10-02/sweep_summary.json`, and `hold_v3_summary.json`
  (the D 0 pass is added there).
- Video on Mira: `sweep_smooth/run.mp4`, `sweep_smooth/run_d.mp4` and
  `gain_bench_hold/run_d0.mp4`.
- At 10:50 a first sweep attempt found no servo on the bus: the scan showed 0
  servos with the servo unpowered, and nothing moved (`run_nobus.mp4`). Tom
  powered it back on.

## Next: an accelerometer on the arm

The servo's telemetry cannot show shaking above about 15 Hz, under 1 tick, or
out of the rotation plane, which is what Tom sees by eye. The plan:
- a BNO055 breakout glued near the fork tip, wired by 4 wires (3V3, GND, SDA,
  SCL) to the I²C header of the newer controller (the General Driver: P1 or the
  40-pin header, pins 1, 6, 3, 5). The controller stays on the table;
- a firmware burst-sample command that logs the accelerometer at a few hundred
  Hz alongside servo position.

The bench's current controller is the older board, so the bench moves to the
new one first.

## The accelerometer: a ~15 Hz oscillation in the rotation direction (14:43–14:51)

The bench moved to the newer controller, a General Driver.
Tom OKed the flash: it carries `main` plus the bench `bno` command (2e9516b).
The "BNO055" breakout turned out to be a **GY-BNO08X** at 0x4B. It is wired to
the board's P1 "IIC" connector (`figs/bno055_wiring.png`) and glued near the fork
tip.

`bno` streams its calibrated accelerometer report at about 500 Hz in the
background while `sweep_smooth.py BNO=1` drives the sweeps. At rest |a| =
988 mg, with a standard deviation of 4.5 mg.

**Axes, from gravity:**
- z runs along the arm: −1006 mg at plumb.
- **y is the rotation direction**, tangential: +970 mg at level.
- x is mostly out of plane: a constant offset of about 250 mg, so the sensor is
  tilted about 15°.

**Before the ladder,** the arm was lowered from 1899, where Tom had left it after
working on it, to plumb, and released at 3529.

Acceleration RMS above 5 Hz in mg, y / x, and the dominant frequency:

| gains | A raise | A lower | C raise | C lower | peak y / x |
|---|---|---|---|---|---|
| P 32 / D 32 | 58 / 25 | 74 / 25 | 95 / 32 | 126 / 35 | 15–17 / 15–34 Hz |
| P 128 / D 32 | 242 / 49 | 345 / 88 | 483 / 179 | 587 / 212 | 14–15 / 28–31 Hz |
| P 128 / D 0 | 137 / 40 | 287 / 64 | 406 / 124 | 387 / 139 | 14–15 / 14–31 Hz |
| P 160 / D 0 | 389 / 82 | 303 / 70 | 527 / 196 | 499 / 182 | 13–15 / 14–31 Hz |

- **The dominant shaking is about 15 Hz in the rotation direction.** It is the
  position loop oscillating the arm, not an out-of-plane wobble. The x axis,
  about 30 Hz, is smaller.
- **A resonance near 15 Hz exists even at stock P 32** (60–125 mg).
  P 128 / D 32 multiplies it by about 4–5. **D 0 roughly halves P 128's**
  (about 0.6–0.85× of D 32), and P 160 is worse again.
- **Streamed minimum-jerk targets (C) excite it more than a single move (A)**
  at every gain.
- **In angle** (assuming the sensor is about 100 mm from the axis, which was not
  measured): θ_rms ≈ a_rms / (r·ω²), so at 15 Hz:
  - P 32: about 0.04–0.08° RMS;
  - P 128 / D 32: about 0.15–0.37° RMS;
  - P 128 / D 0: about 0.09–0.26° RMS.
- **Servo telemetry did not see most of this.** Its 30 Hz poll is blind at
  15 Hz. In this run it reported 0 backward samples at P 128 / D 32, where the
  old controller run reported about 20 of 300. Why that changed (the new board,
  or the arm work) was not tested.

**Files:** per-sweep metrics in `2026-10-02/sweep_bno_summary.json`; raw data
and video on Mira under `hw_sessions/2026-10-02/sweep_bno/`, and the setup and
flash videos in `hw_sessions/2026-10-02/bno_setup/`.

## How the motion is commanded, and the command rate (14:57–15:10)

Every motion is the firmware's `move <id> <target> <ms> <speed> <acc>`, which
writes the servo's goal position, goal speed and acceleration registers. The
servo's own controller makes the motion.

- **A / B, one command:** the final position, a speed (100 steps/s) and an
  acceleration (200 or 100 steps/s²). The servo plans its own ramp.
- **C, a stream:** the host computes a minimum-jerk curve and sends the next
  point every cycle (about 15 Hz). Speed is 1.5× the curve's, and acceleration
  is 0, which is unlimited. The servo reaches each small target early and stops,
  so it moves stop-start at the command rate.
- **Cf / Cs:** the same stream at about 24 Hz (position read only every 4th
  cycle) and about 9 Hz.
- **C1:** the same as C, but speed equal to the curve's (1.0×) and acceleration
  4 (400 steps/s²).

The robot's own pose streaming sends small targets too, with speed set from
the gap to the next one.

Acceleration y (the rotation direction), RMS above 5 Hz in mg, raise / lower:

| profile, target rate | P 96 / D 0 | P 96 / D 32 | P 128 / D 0 | dominant y frequency |
|---|---|---|---|---|
| A, one move | 91 / 230 | 112 / 327 | 234 / 297 | 14–17 Hz |
| C, 15.1 Hz | 201 / 336 | 318 / 537 | 452 / 459 | 15.2 Hz |
| Cf, 24.2 Hz | 145 / 184 | 175 / 256 | 449 / 298 | 12.1–12.3 Hz |
| Cs, 9.2 Hz | 249 / 184 | 243 / 238 | 393 / 244 | 9.1 Hz |
| C1, 15.1 Hz, exact speed + acceleration limit | 81 / 120 | 96 / 482 | 419 / 401 | 11–16 Hz |

- **Streamed shaking is forced by the stepping, not a resonance.** Its frequency
  follows the target rate: 15.1 → 15.2 Hz, 9.2 → 9.1 Hz, and 24.2 → 12.1 Hz,
  half the rate. The 14:43 run's "15 Hz" for C was its 15 Hz command rate.
- **Matching speed (C1) cuts it about 2.5–3× at P 96 / D 0,** a little below a
  single move. But C1 trails its target by about 36 ticks (3°), because it has
  no lead. At P 128 / D 0 it helps by only about 10 %.
- **A single move still shakes at 14–17 Hz,** and that grows with P: 58–74 at
  P 32 (14:43 run), 91–230 at P 96 / D 0, 137–297 at P 128 / D 0. This part
  belongs to the servo and rig.
- **D 0 beats D 32 at P 96** in A, C, Cf and C1.
- No status bit was set in any sweep.

**Implication:** how targets are streamed matters as much as the gain. A
matched-speed stream with a lead, or interpolation in the firmware, is the next
thing to test.

## Changing the goal in flight, and walking speeds (15:21–15:40)

**Goal changes in flight (Tom: "always write the currently known final goal").**
- **R− / R+:** one `move` at 100 steps/s and acceleration 2, with the goal changed
  at half the travel time. R− changes it to 10° (114 ticks) short; R+ goes from
  10° short to the full end.
- **RL− / RL+:** the same change at 85 % of the travel, while the servo is
  already near its goal.

`retarget_analysis.py` compares y RMS in [−0.3, +0.8] s around the change
with the rest of the move:
- **Halfway (cruising): seamless at every gain.** The servo keeps cruising at
  100 steps/s, and the window ratio is 0.3–1.1, with one 1.47 at P 32.
- **At 85 %:** P 32 and P 96 / D 0 show no added shaking (0.4–1.0; one P 32
  raise at 1.5×, 74 mg). P 128 / D 0 lowering adds 1.2–1.5× (380–450 mg).
- **Lowering is stop-start even in a plain move at P 96 and P 128.** The
  reported speed alternates 0 / 200 / 50 / 250 instead of 100, while raising
  holds 100. With gravity helping, the servo's speed control itself oscillates.
  That is the 14–17 Hz component seen in single moves.

**Walking speeds.** The sim's walk peaks (`docs/design-v6/no3250_envelope.txt`)
are about 0.5 rad/s at the rolls (about 320 steps/s) and up to 2.25 rad/s at a
knee (about 1,470 steps/s). The ladder ran single moves at 300, 600 and 1,500
steps/s, ramping to speed in about 0.15 s (acceleration = speed / 15).

Acceleration y, RMS above 5 Hz in mg, raise / lower:

| speed | P 32 / D 32 | P 96 / D 0 | P 128 / D 0 |
|---|---|---|---|
| 100 steps/s (14:43 and 14:57 runs) | 49–74 | 91–269 | 137–316 |
| 300 | 52 / 58 | 58 / 87 | **682** / 137 |
| 600 | 62 / 61 | 62 / 67 | 70 / 78 |
| 1,500 | 82 / 85 | 93 / 111 | 356 / 188 |

- **At walking speeds P 96 / D 0 shakes about as little as stock P 32.** The
  slow 100 steps/s sweeps were the worst case.
- **P 128 / D 0 is uneven:** 70–78 at 600 steps/s, but 356 at 1,500, and one
  682 mg raise at 300 steps/s, not repeated.
- **A speed-locked vibration is present at every gain, stock included:** 12.7 Hz
  at 300 steps/s and 25.4 Hz at 600. That is one cycle per about 24 ticks (2.1°)
  of travel, a mechanical periodicity in the servo (gear mesh or encoder, not
  identified).
- **Every move settled exactly** (end range 0), and no status bit was set.

**Reading:**
- **P 96 / D 0** (about 2.8× stock stiffness, below the sim's 3–4× target) is
  quiet at hold, settles, and stays near stock shaking at walking speeds and
  through late goal changes.
- **P 128** is worse with this load.
- **P 160** limit-cycled in the hold test.

The per-run numbers are in `2026-10-02/sweep_bno_summary.json`.

## Repeats at walking speeds (16:1x–16:26)

Tom stepped away; the stand was free. Same rig and weights, with no one at
the rig. Acceleration y RMS above 5 Hz in mg, one value per repeat:

| condition | raise | lower |
|---|---|---|
| P 96 / D 0, 300 steps/s | 56, 58, 58 | 76, 76, 83 |
| P 96 / D 0, 1,500 steps/s | 90, 95 | 112, 109 |
| P 128 / D 0, 300 steps/s | 71, 61, 62 | 105, 102, 107 |
| P 128 / D 0, 1,500 steps/s | 111, 135 | 199, 191 |

- **P 96 / D 0 is repeatable** to within a few mg.
- **The earlier 682 mg at P 128 / D 0, 300 steps/s did not recur** (61–71 mg).
  Neither did 356 at 1,500 steps/s (111–135).
- At walking speeds **P 128 / D 0 shakes about 1.1–1.8× as much as P 96 / D 0**.
- No status bit was set.

## Streaming at the control loop's real 50 Hz (16:33–16:39)

The host's CLI can only stream about 15–24 Hz. So a firmware bench command,
`stream` (cli.cpp), sweeps one servo along a minimum-jerk curve **from the
board, at 50 Hz**, two ways:

- **mode 0, the control loop's own rule** (`ctrl_task.cpp`): target on the curve
  now, speed `obs::goalSpeedSteps` (gap / 20 ms × 1.25), acceleration 0;
- **mode 1, "matched":** target 40 ms ahead, speed = the curve's own speed,
  acceleration limit 600 or 2,300 steps/s².

Script: `2026-10-02/stream50.py`, plumb ↔ level over 4 s (peaking at about 480
steps/s) and 2 s (about 960 steps/s).

**Bus recovery.** At the first try the I²C bus was held: nothing answered,
including the board's own sensors, presumably the BNO08x after a reset
mid-read. `imu reinit` freed it, and both scripts now do that before giving
up.

Acceleration y RMS above 5 Hz in mg, raise / lower, and tracking RMS in ticks:

| gains | rule, 4 s | rule, 2 s | matched, 4 s | matched, 2 s |
|---|---|---|---|---|
| P 32 / D 32 | 69 / 90; track 28–32 | 65 / 81; 79–82 | 84 / 101; 128–132 | 85 / 99; 145–149 |
| P 96 / D 0 | 95 / 123; 22–23 | 181 / 91; 65–66 | 121 / 166; 123–124 | 141 / 153; 135–136 |
| P 128 / D 0 | 341 / 155; 21–22 | 81 / 127; 64 | 387 / 319; 122–123 | 145 / 191; 134 |

- **At 50 Hz the control loop's rule shakes about as much as a single move**
  (P 32: 65–90 mg; P 96 / D 0: 91–181). The dominant frequency is 13–19 Hz,
  not the command rate.
- **The stop-start forcing measured earlier is a slow-stream artifact:** the
  host at 9–24 Hz, with 1.5× speed. Plan B does not need a change to the
  firmware's streaming.
- **The rule trails the curve** by about 21–32 ticks RMS (2–3°) on 4 s sweeps
  and 64–82 on 2 s. That is inherent in "target now, speed ∝ gap". Whether the
  sim's actuation lag model matches it was not checked.
- **"Matched" as written is worse:** 122–149 ticks of lag, ends not settling
  (range 12–17), and no less shaking. Its acceleration limit and end-speed floor
  are too low, so it needs rework before it says anything.
- No status bit was set. P 128 / D 0 again had one high reading (341 mg).

## What the bench now says (summary)

| setting | stiffness × P 32 | holding still | 2° moves | walking-speed single moves (y RMS, mg) | streamed slow sweeps |
|---|---|---|---|---|---|
| P 32 / D 32 (stock) | 1 | quiet | settle | 52–85 | quiet |
| **P 96 / D 0** | **≈ 2.8** (clamp probes) | quiet, level and hanging | settle | **57–111** | host 9–24 Hz: command-rate forcing; firmware 50 Hz (the robot's rule): 91–181, like single moves |
| P 128 / D 0 | ≈ 3.0–3.5 | quiet, level and hanging | mostly settle | 61–199 | rougher |
| P 160 / D 0 | ≈ 4.8 (bottles, D 32) | quiet level; **oscillates after a 2° move while hanging** | no | — | — |

The sim side, with the tuned 3215 at 2.8×, is in the design record §14.4a.

## Open

- **Weights:** record their radius and mass, to place the inertia against #73's
  leg-like spec.
- **P 96 / D 0 stiffness with bottles** (needs someone to hang them). P 96's
  2.8× comes from clamp probes only.
- **Streaming:** done at 50 Hz (above). The robot's rule is fine; its 2–3°
  lag against the sim's actuation model is unchecked.
- **The decision #73 feeds:** STS3250 or STS3215. The sim study (design
  record §14.4a) and the gain choice (P 128 / D 0 is the smoothest stable
  setting so far) both bear on it.
- **ID:** the servo is still ID 30. Rename it back to 11 when bench work on it
  is done.
