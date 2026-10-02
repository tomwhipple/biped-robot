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

## Open

- **Weights:** record their radius and mass, to place the inertia against #73's
  leg-like spec.
- **The decision #73 feeds:** STS3250 or STS3215. The sim study (design
  record §14.4a) and the gain choice (P 128 / D 0 is the smoothest stable
  setting so far) both bear on it.
- **ID:** the servo is still ID 30. Rename it back to 11 when bench work on it
  is done.
