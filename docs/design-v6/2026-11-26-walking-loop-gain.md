# Does the x1.5 loop gain hold while walking? (2026-11-26)

Issue #137, for the planner (2026-11-25 report: "The robot's 1.5x was measured on a stand (11-02). Whether it holds
while walking is unmeasured."). Offline analysis only: CPU, no hardware, no policy. Tool:
`experiments/policy/x15-margin/walk_gain.py`; full tables `walk_gain_results.md` and `walk_gain_selftest.md` next to it.

## Result

- **In the servo path, no.** While walking, the robot's 12 encoders answer the written goals exactly as the x1.5
  grade plant's do: robot / sim **1.00** (roll joints 0.99, sagittal 1.00, hip yaws 0.92; 90 % CI ±0.01; coherence
  0.95-1.00). The same method reads 1.12-1.19 for a true G of 1.25 and 1.22-1.40 for 1.5. Walking, the joint path is
  at **G ≈ 1.0**; 1.25 is excluded.
- **In the body path, the data cannot say.** The robot's torso roll, pitch and yaw while walking cannot be compared
  open loop against the sim at the precision needed. A replay with a known G = 1.25, 1.5 or 2.0 reads 0.5-1.9 there,
  not in order of G. The robot reads 0.8-1.2 in roll, within that range.
  - The robot does move more than the sim: in rms, roll 1.0-1.8x, pitch 1.6-3.5x, yaw rate 3-5x.
  - Almost none of it is the sim's response scaled up: coherence 0.0-0.16 in pitch and yaw. It is motion the plant
    does not produce at all, so it is a disturbance, not a loop gain.
- **On the stands, the method reproduces 1.25-1.5, in roll only.** Read against the same calibration:
  - walk14_s1 on factory servos: **G ≈ 1.4-1.5** in hip/ankle-roll joints, torso roll and yaw rate;
  - walk14_s1 on fast servos: **≈ 1.2-1.25** in roll;
  - walk16_act, the stand that shook itself to a flail trip at x1: **≈ 1.1-1.4** in pitch and the sagittal joints.
  - The sagittal joints and hip yaws read below 1 on every stand except that last one.
  - So the stand's excess is a **small-amplitude roll-chain effect**: sub-degree goals at the 1.5 / 3 Hz gait clock
    feed the hip-roll servos and the robot's ~3 Hz body roll mode, which is under-modelled (the 11-01 RCA). The fast
    servos halved its 3 Hz peak, and the readings follow that. The track walks' degree-size, slow moves do not show it.

**What G to grade the walking probes at: 1.25 as the bar, 1.5 as the margin, nothing above 1.5.**
- Nothing measured while walking supports more than ~1.0 in the joints.
- A policy gait puts small-amplitude 1-3 Hz roll content into double support, where the stands read 1.25-1.5, and the
  body path while walking is unmeasured. So 1.25 is the defensible floor, not 1.0.
- 1.5 is the worst the robot has read anywhere (factory servos, which it no longer runs).
- The robot's large uncorrelated yaw while walking is a disturbance. A yaw-disturbance probe (grade.py's
  `yaw_kicks`) tests it; a higher G does not.

## What the 1.5x is

`grade.py`'s `out_gain`: every tick the served action is its 0.5 s EMA plus G x (action - EMA), on all 12 joints,
before the shaper. It scales the policy's output on everything faster than ~0.3 Hz.

The number comes from the 2026-11-02 stand-still record (`experiments/policy/stand-still/2026-11-02-stand-still.md`
§2), and it is a **stability bracket, not a signal ratio**:
- walk14_s1_s128r24 on fast servos without the bench knobs stood in sim at x1 (0/8 falls) and fell 8/8 at x1.5;
- on the robot it diverged in under 1 s at x1;
- hence "the robot is ~1.5x the sim".

No signal was compared at a frequency. The robot's goals then jittered 0.1-0.5° rms at 1.5 and 3 Hz (the gait
clock), and the instability was a roll-and-yaw shake.

## Method

1. **Robot side.** Every tick of the flight records is used: the written goal and encoder per joint, the gyro and the
   torso `up` vector.
   - Walks: 293 track-player runs on 11-12, 11-13 and 11-16, from the track logs' time windows. That is 218 min and
     2,787 single supports, on fast servos (`servocfg` 85 = 0 / 81 = 1 at boot).
   - Stands: every 11-02 policy stand of 8 s or more.
2. **Sim side.** The robot's written goals are replayed open loop, at 50 Hz with zero-order hold, into the x1.5
   probe's own plant: `bimo_biped_v6legs_meas_sole.xml`, `simh.JF_PLANT`, `sag_L=6.4,sag_R=2.9`, and the
   `grade.FAST` TypedServo (the factory-servo stand uses `MEAS_SERVO`). The sim starts from the robot's measured
   joints. Nothing closes the loop. 98 % of the walking time is usable: both upright, and the sim's attitude within 6°
   of the robot's. Sims fell in 6 of 293 walks.
3. **The estimate: H1 with the sim as input.** In each window, both responses are band-passed: by the probe's own
   filter (x − EMA 0.5 s), and 0.3-1 / 1-3 / 3-8 Hz.
   - Windows: each single support (stance leg / swing leg), each double support, and 4 s blocks on stands.
   - The robot × sim cross-covariance is summed over every window of a data set, at the one lag (±240 ms) that
     maximises it, and divided by the sim's variance.
   - Robot motion the sim does not predict lowers the coherence, not g.
   - The 90 % CI comes from resampling whole runs.
4. **The calibration (`selftest`).** The plant is not linear: sub-degree stand goals meet friction and deadband, and
   a walk changes contacts. So the reading is not G. The same segments, replayed with the out_gain law at a known G
   on their goals, stand in for the robot and are read the same way.

## Numbers

### Walking (293 runs; probe band / 1-3 Hz)

| channel | robot / sim g [90 % CI] | coherence | selftest reading at G = 1.25 / 1.5 / 2.0 |
|---|---|---|---|
| roll joints (hip + ankle roll) | **0.99** [0.99, 0.99] / 0.89 [0.87, 0.90] | 0.99 / 0.76 | 1.12 / 1.22 / 1.42 (1-3 Hz: 1.16 / 1.30 / 1.67) |
| sagittal joints | **1.00** [1.00, 1.00] / 0.98 [0.98, 0.98] | 1.00 / 0.97 | 1.17 / 1.31 / 1.42 (1-3 Hz: 1.19 / 1.40 / 1.71) |
| hip yaws | **0.92** [0.91, 0.92] / 1.00 | 0.95 / 0.97 | 1.13 / 1.22 / – |
| torso roll | 0.77 [0.74, 0.80] / 1.14 [1.11, 1.16] | 0.38 / 0.42 | 1.27 / 1.33 / 1.17 (not monotonic) |
| roll rate (gyro x) | 0.53 / 1.09 | 0.15 / 0.36 | 0.68 / 0.45 / 0.83 (not monotonic) |
| pitch, pitch rate | 0.2-0.5 | ≤ 0.03 | 0.7-1.5 |
| yaw rate (gyro z) | 0.94 / 1.20 | 0.04 / 0.09 | 0.7-1.9 |

By support phase (probe band):

| phase | roll joints | sagittal joints | hip yaws |
|---|---|---|---|
| stance leg | 0.99 | 0.96 | 0.92 |
| swing leg | 1.09 | 1.00 | 0.91 |
| double support | 0.99 | 0.98 | 0.98 |

At 1-3 Hz, the stance roll joints read 0.72.

Torso roll at 1-3 Hz:

| phase | robot / sim | coherence |
|---|---|---|
| single support | 1.16-1.22 | 0.4-0.5 |
| double support | 0.66 | 0.14 |

The body channels' selftest readings are not monotonic in G. The windows also thin out as G grows (894 → 163 → 49
body windows), because the gained replay drifts away from the plain one. So the body rows are not evidence for or
against a gain while walking.

### The 11-02 stands (probe band / 1-3 Hz; robot reading → G it matches in the selftest of the same segment)

| stand | roll joints | torso roll | roll rate | yaw rate | sagittal joints | pitch |
|---|---|---|---|---|---|---|
| stand2: walk14_s1, factory servos, knobs | 1.50 / 1.75 → **~1.4** | 4.52 / 5.97 → **~1.5** | 2.43 / 4.52 → 1.2-1.4 | 3.77 / 3.40 → **~1.5** | 1.03 / 1.05 → < 1.25 | 0.70 / 0.44 → < 1 |
| stand3: walk14_s1, fast servos, knobs | 1.21 / 1.51 → **~1.2** | 2.29 / 3.17 → **~1.25** | 1.76 / 2.80 → ~1.2 | 1.71 / 1.31 → < 1.25 | 0.81 / 0.85 → < 1 | 0.66 / 0.47 → < 1 |
| stand7 18:10:45: walk16_act, no obsfreeze, shape 12, clock running (flail trip at 21 s) | 0.77 → < 1 | 0.58 / 0.62 → < 1 | < 1 | 0.89 / 0.93 → < 1.25 | 1.07 / 1.15 → **~1.1** | 1.23 / 1.42 → **1.25-1.4** |
| stand4, stand5 (walk16_act, knobs, clock running) | 0.4 → < 1 | 0.6-0.8 → < 1 | < 1 | 0.4-0.5 → < 1 | 0.9-0.95 → < 1 | 0.7-1.05 → < 1 |

On a stand, the selftest reads G = 1.25 as 1.2-3.2 (the sim's friction makes sub-degree response superlinear). That
is why the raw robot readings above are larger than the G they match.

## Caveats

- **The walks are the track player's quasi-static gait**: 3.4-5 s per step, degree-size min-jerk moves, with the
  firmware trim in the goals. A policy walks at a 1.5 Hz clock with sub-degree jitter on top. Its 1-3 Hz content is
  closer to the stands', where the robot read 1.25-1.5 in roll.
- **The body path while walking is unmeasured** (above). Measuring it needs a known small excitation, or a policy's
  own walk on the robot read by this tool. Neither exists yet.
- **Two hardware states.** The stands are 11-02 hardware, the walks 11-12..16. The `sag_L/R` terms of the plant were
  fit to the 11-16 walks, so the plant is closest to the walking robot. That favours a ratio near 1 in roll while
  walking, but the sag is released only in single support and does not touch the stand's double-support roll mode.
- **Not modelled on either side.** The firmware writes a goal speed (register 46: 125-250 steps/s at a stand) that the
  TypedServo does not model. Goals and encoders are quantised to 0.088°, which is most of a stand's jitter; stands
  whose goals moved under ~1 step (the coast stands) are kept out.
- **The selftest applies the law to the written goals** (after the shaper), whereas grade applies it to the actions
  (before the shaper). Both are a gain above ~0.3 Hz.

## Next

**The body path on the robot.** `track_host.py --id-wiggle 1.0:roll` adds a known 1° multisine (1.07-2.87 Hz) to the
hip- and ankle-roll goals in every double support. `wiggle_gain.py robot` reads the torso's response against it.

The sim check (`experiments/policy/x15-margin/wiggle_gain_simcheck.md`) puts that excitation on the 11-12..16 walks
with the robot's own body motion as the noise. It reads a known G = 1.0 / 1.25 / 1.5 as:

| channel | reading at G 1.0 / 1.25 / 1.5 | coherence |
|---|---|---|
| torso roll | 1.03 / 1.31 / 1.59 | 0.40-0.58 (the walks' double supports: 0.14) |
| yaw rate (yaw channel) | 1.02 / 1.40 / 1.75 | 0.50-0.69 |
| pitch (pitch channel) | 0.94 / 1.24 / 1.57 | 0.13-0.27 |

Six 8-step walks separate 1.0, 1.25 and 1.5 in roll. 0.5° does not. The robot command and the predictions are in
[experiments/roll-balance/README.md](../../experiments/roll-balance/README.md#the-body-paths-loop-gain---id-wiggle-amp_degchannel-host-only-default-off-259-137):
a later, optional run after the #297 sequence.

## Reproduce

```bash
cd experiments/policy/x15-margin       # CPU: JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES=
python walk_gain.py extract            # ~10 min: one pass per flight record -> build/walk-gain/robot/
python walk_gain.py replay --workers 8 # ~3 min -> build/walk-gain/sim/
python walk_gain.py analyze --md walk_gain_results.md
python walk_gain.py selftest > walk_gain_selftest.md
```

Tests: `tests/test_walk_gain.py` (the probe filter, the lag fit, the pooled H1 and its bound, the support windows).
