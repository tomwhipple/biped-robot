# The 12-degree left-stance tips of 2026-11-16: what the record shows (#297)

*A dated record of an offline analysis (2026-11-24, night run, no hardware touched) of the inward tips on left
stance in the normal-timing steering runs of the 2026-11-16 evening session
([#297 comment](https://github.com/tomwhipple/biped-robot/issues/297#issuecomment-6937032496): "9 in about 400 steps,
cause unknown"). The robot is down (left hip broke at 20:13 that night). The question was: from the data already
recorded, find the cause or narrow it, and give the next session a hypothesis and a test.*

## 2026-11-25: follow-up

The flight records were re-read for A (trim drift) against B (a push in double support), in
[2026-11-25-left-stance-tips-a-vs-b.md](2026-11-25-left-stance-tips-a-vs-b.md) (`tip_causes.py`).

**The trigger is in double support.** Every tip (12 / 12) followed a roll snap of 3.1-4.4 deg in the weight shift
before the lift-off:
- the roll joints fall behind the shift's fastest part and catch up with a load spike;
- that leaves the feet squeezed at the lift-off;
- that state predicts the swing-back on all 365 normal-timing left stances (R² 0.46), and T and the stance count add
  nothing to it.

**It is not a growing preload.** The snap is on or off per step: 50 % of second left stances, 60 % of late ones.

**What prevents it.** The snap needs the shift to start within 0.5 s of the touchdown. With `--td-pause 0.3` (the
3.47 s recipe, or T 4 at 4.3 s) it happened in 1 of 257 left stances, with no tips. That, not the short settle, is what
protected the fast timing.

**T is not the trigger** (T up to 7 deg did not tip without a snap). On snap steps a higher T may make a tip likelier
(weak, 12 events).

**The first robot test is now** `--td-pause 0.3` against the control, read per step by the snap share. The frozen-trim
test below moves to third.

## Result

**Cause not found; narrowed.** What the data does establish:

1. **What a tip is.** The robot fails to swing back after the foot-off sag. Up to 0.6 s after lift-off a tip is the
   same as a clean step: about 2.9 deg toward the swing foot. Measured beyond the stance hip and ankle roll encoders
   it is about 8.3 deg, because the encoders move the other way as the trim hikes. A clean step then swings back outward (-16 deg/s at 0.6 s, -1.5 deg at 0.9 s). A tip stalls
   (-5 deg/s, +3.9 deg at 0.9 s) and goes over from about 1.0 s. The roll grows beyond the encoders (12.3 deg at
   0.8 s, over 20 by 1.4 s) while the trim drives the stance hip the right way (hike 11-12 deg). So the body pivots
   past the stance sole, through the hip-roll play or over the sole's inner edge. The servos kept tracking (stance
   hip-roll error 1.7 deg at 0.8 s, against 1.2 on clean steps). The peak is always 11.2-12.6 deg, and the swing foot
   comes down and catches it (frames below). Every clean step peaked at 4.4 deg or less; no step peaked in between.
2. **Where they happen.** All 12 tips of the evening were on left stance, in normal timing (4.0 s/step), in `--cmd`
   walks, at the 5th or later left stance of the walk. Early left stances (k ≤ 4): 0 of 202. Late ones (k ≥ 5): 12 of
   163 (Fisher p = 5e-5). Late left stances at the faster timing (3.47 s/step): 0 of 59 (p = 0.04 against normal).
   Fixed (non-`--cmd`) walks, late: 0 of 34 (p = 0.07). Right stance: 0 of 477.
3. **What comes before them.** In the raise (0.3-0.5 s after lift-off), before any tip has split off, the load
   pattern is different while the commanded hike is the same (4 deg):

   | load | tips | clean, normal, k ≥ 5 | clean, normal, k ≤ 4 | clean, fast, k ≥ 5 |
   |---|---|---|---|---|
   | stance (L) hip roll | 143 | 122 | 111 | 116 |
   | stance (L) ankle roll | 0 | -28 | -36 | -33 |
   | swing (R) hip roll, last 0.25 s before lift-off | -32 | -16 | -8 | -18 |

   Each value is a median, in STS3215 load counts. The tips sit at the end of a trend that builds up over each
   normal-timing walk and much less at the faster timing.
   - Every one of the 12 tips had stance ankle-roll load ≥ -15 in the raise and swing hip-roll load ≤ -16 before
     lift-off.
   - Clean left stances with both: 19 % early and 34 % late in normal timing, 6 % / 10 % in fast timing.
   - So the pattern is necessary in this data but not sufficient.
4. **Ruled out, or no evidence for it** (details below):
   - the left hip failing over the evening (no trend);
   - supply voltage and temperature;
   - the swing foot scuffing;
   - the lift gate;
   - the step type;
   - lateral stagger at lift-off;
   - where the robot was on the board.
5. **The learned trim T.** T is high on the tips, but it is mostly a marker of how far into the walk the step is (it
   climbs within every walk).
   - Late left stances with T at lift-off ≥ 6 deg: 11 tips of 142. Below 6: 1 of 80.
   - Early stances with T ≥ 6: 0 of 18 (few).
   - In clean steps T does not predict the swing-back (r = 0.07).
   - The cap at 6.9 deg still tipped, two of three late stances in `capB_L_2`.
   - It is a candidate, not a finding.

**The working hypothesis, a guess.** Over a normal-timing walk something builds up that leaves the stance foot
carrying no roll moment in the raise (ankle-roll load near 0) and the feet pushing against each other at lift-off
(swing hip-roll load). Once the foot-off sag has peaked, the body then cannot swing back over the sole. The two
candidates for what builds up:
- (a) the firmware's learned trim drifting: T L 5.2 → 6.6, T R 3.4 → 1.1;
- (b) a double-support preload between the feet that grows step by step.

The data cannot separate them, because both follow the step count. The next session's first test (end of this
record) does, with the trim's learning frozen.

## Data and method

- Flight record `hw_sessions/2026-11-09/flight_20261116T211526Z_prec16.frec`. It covers 14:15-20:07 MST; the
  20:13 fall is in `flight_20261117T031115Z_prec16.frec`, not analysed here.
- Track logs: the 77 runs of 18:26-20:07 MST in `~/code/robot-run`, `robot-poshold`, `robot-poshold-run` and
  `robot-cmd*` `experiments/roll-balance/results/`. They are not committed; the largest copy of each was used.
- Single supports come from the trim's state edges in the record:
  - OFF/FADE → CLOSED is the lift-off;
  - → FADE is the touchdown.
  - They are matched one-to-one to the logs' TS lines (all 77 runs matched).
- The step period is the median spacing of the TS lines: 3.9-4.1 s is "normal" (T 4); 3.4-3.5 s is "fast"
  (`--td-pause 0.3 --shift-s 1.3 --settle 0.1 --raise-s 0.77 --move-s 0.45 --lower-s 0.55`).
- k is the stance count on that side within the walk.
- A tip is the track's own rule: lean toward the swing foot ≥ 7 deg.
- Script: `experiments/roll-balance/stance_tips.py` (test `tests/test_stance_tips.py`). Run it as
  `python experiments/roll-balance/stance_tips.py --day 20261116 --from 1800 --to 2010`. It writes one row per
  single support to `experiments/roll-balance/results/stance_tips/20261116T182648_20261116T200654.csv` and prints
  the summary and the tip lines below.
- It reads one window of the record per run: 25 s and 40 MB for the whole evening. The deeper cuts in this record
  (per-joint traces aligned at lift-off, the AUC scan) were throwaway scripts on the same extraction.

Counting against the comment's figures, by the TS line's mid-support T on evening normal-timing left stances:
- T < 7: 2 of 234 tipped; T ≥ 7: 10 of 131.
- The comment's 0/309 against 13/178 counted another window; the direction is the same.
- The comment's "9 in about 400" was counted differently: its window and runs are not stated. This record counts
  12 tips in 365 normal-timing left stances, 18:26-20:07.

## The 12 tips

All are left stance at normal timing in `--cmd` walks.
- T lift: the learned T L at lift-off, from the record.
- T mid: the TS line's T.
- e, gyro: the roll toward the swing foot (deg, deg/s) at that time after lift-off.
- Loads: stance hip roll and ankle roll over 0.3-0.5 s; swing hip roll over the 0.25 s before lift-off.
- G is 0 for all twelve: the lift gate never held.

| run (MST) | note | step | k | step cmd (dx, dy, dyaw) | T lift / mid | e 0.5 s | gyro 0.6 s | e 0.9 s | peak | loads st hip / st ank / sw hip | lift-off (UTC) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:29:42 | square_and_left | 13 | 7 | 0, 15, 0 | 6.0 / 7.3 | 2.5 | -2 | 4.8 | 11.4 | 152 / 18 / -32 | 01:30:35.16 |
| 19:06:57 | ss_side_2 | 9 | 5 | 0, 15, 0 | 6.3 / 7.4 | 2.5 | -7 | 3.1 | 11.4 | 133 / -7 / -24 | 02:07:35.07 |
| 19:06:57 | ss_side_2 | 13 | 7 | 0, 15, 0 | 6.4 / 7.5 | 2.7 | -5 | 2.9 | 11.2 | 127 / -15 / -16 | 02:07:51.08 |
| 19:09:28 | poshold_march_plain_b | 9 | 5 | 0, -3.2, 0 | 7.0 / 8.2 | 2.6 | -8 | 3.7 | 12.6 | 153 / 11 / -24 | 02:10:04.94 |
| 19:20:12 | rc_side_1 | 9 | 5 | 0, 10, 0 | 7.2 / 8.3 | 2.3 | -3 | 3.6 | 12.2 | 155 / 19 / -40 | 02:20:48.97 |
| 19:23:16 | rc_fb_2 | 9 | 5 | -20, 0, 0 | 6.2 / 7.6 | 2.8 | +1 | 4.4 | 12.1 | 143 / 3 / -32 | 02:23:53.27 |
| 19:30:52 | rc_fb_4 | 11 | 6 | 0, 0, 0 (closing) | 7.2 / 8.5 | 2.2 | +5 | 2.3 | 12.3 | 139 / -9 / -24 | 02:31:37.98 |
| 19:48:35 | poshold_march_M_d | 15 | 8 | 0, 12, 0 | 6.3 / 7.4 | 2.4 | -7 | 4.1 | 12.3 | 138 / 8 / -32 | 02:49:33.99 |
| 19:48:35 | poshold_march_M_d | 29 | 15 | -4.1, -12, 0 | 7.5 / 8.5 | 2.4 | -7 | 3.8 | 12.5 | 148 / 2 / -25 | 02:50:29.97 |
| 19:48:35 | poshold_march_M_d | 31 | 16 | -4.1, -0.2, 0 | 7.5 / 8.5 | 2.3 | 0 | 4.6 | 12.3 | 145 / 5 / -32 | 02:50:37.98 |
| 20:03:25 | capB_L_2 (T cap 6.9) | 11 | 6 | 0, 15, 0 | 6.1 / 6.9 | 2.4 | -7 | 5.2 | 11.5 | 149 / 11 / -40 | 03:04:09.90 |
| 20:03:25 | capB_L_2 (T cap 6.9) | 13 | 7 | 0, 15, 0 | 6.1 / 6.9 | 2.3 | -5 | 5.0 | 11.2 | 141 / 8 / -32 | 03:04:17.89 |
| *clean, normal, k ≥ 5 (151), median [p10..p90]* | | | | | 6.2 | 2.9 [2.6..3.1] | -16 [-22..-9] | -1.5 | ≤ 4.4 | 122 / -28 / -16 | |

## What a tip is: the timeline

These are medians from lift-off (s); e is the roll toward the swing foot (deg). "Beyond enc." is that roll less what
the stance hip roll and ankle roll encoders account for. It is footoff_sag.py's delta: the play, the sole, the
links.

| | 0.2 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.2 | 1.4 |
|---|---|---|---|---|---|---|---|---|---|---|
| e, tips (12) | 0.1 | 1.4 | 2.4 | 2.9 | 2.8 | 3.9 | 3.9 | 4.3 | 9.0 | 10.4 |
| e, clean normal k ≥ 5 (151) | 0.0 | 1.5 | 2.9 | 2.9 | 1.3 | 0.1 | -1.5 | -3.0 | -0.4 | -0.8 |
| e, clean fast k ≥ 5 (59) | 0.0 | 1.5 | 3.0 | 3.0 | 1.0 | -1.1 | -3.0 | -3.5 | 0.5 | -0.7 |
| beyond enc., tips | 0.8 | 4.5 | 6.8 | 8.4 | 10.0 | 12.3 | 12.9 | 13.8 | 19.5 | 22.9 |
| beyond enc., clean normal k ≥ 5 | 0.9 | 4.5 | 6.8 | 8.2 | 8.0 | 7.4 | 6.9 | 5.0 | 6.7 | 6.5 |
| stance hip roll encoder (hike direction −), tips | -0.7 | -2.5 | -3.7 | -5.0 | -6.2 | -7.4 | -8.3 | -8.7 | -10.0 | -11.3 |
| ... clean normal k ≥ 5 | -0.8 | -2.5 | -3.5 | -4.7 | -5.9 | -6.5 | -6.5 | -6.3 | -5.6 | -5.6 |

- The tip starts between 0.6 and 0.8 s after lift-off. At normal timing that is the end of the raise (0.77 s) and the
  start of the move, 2.4-2.6 s into the step, matching the comment's "≈2.5 s, as the swing foot moves".
- The raise is the same 0.77 s at both timings, so the faster timing differs only before lift-off (settle 0.1 s + a
  0.3 s touchdown pause, against a 0.4 s settle) and after 1.22 s (an earlier lowering).
- In e, the clean steps of the two timings are the same up to 0.6 s.
- The swing-back weakens within a normal-timing walk and not at the faster timing. This is e at 0.9 s on clean left
  stances:

  | | k ≤ 4 | k ≥ 5 |
  |---|---|---|
  | normal | -2.3 | -1.5 |
  | fast | -2.7 | -3.0 |

  Right stance, normal: -2.9 → -2.2.

## Where: counts

Evening left stances (18:26-20:07):

| | k ≤ 4 | k ≥ 5 |
|---|---|---|
| normal timing, all | 0 / 202 | **12 / 163** |
| normal timing, `--cmd` | 0 / 142 | 12 / 129 |
| normal timing, fixed walks (`tl_*` marches, `big_side*`) | 0 / 60 | 0 / 34 |
| fast timing (48 of the 59 late ones `--cmd`) | 0 / 100 | 0 / 59 |
| right stance, both timings | 0 / 295 | 0 / 181 |

Normal-timing late left stances, split by other factors:

- **By T L at lift-off:**

  | T L at lift-off (deg) | < 6 | 6-6.5 | 6.5-7 | ≥ 7 |
  |---|---|---|---|---|
  | tips / stances | 1 / 37 | 6 / 71 | 1 / 40 | 4 / 15 |

  Fast timing never got T L above 6.5 (0 / 43 below 6, 0 / 16 at 6-6.5). The learned trims drift apart in every
  walk at both timings: T R at lift-off falls 3.4 → about 1.1, and T L − T R grows from about 2.7 to 4.8.
- **By step type:**

  | step type | tips / stances |
  |---|---|
  | sideways left | 7 / 84 |
  | sideways right | 2 / 16 |
  | backward | 2 / 15 |
  | march | 1 / 14 |
  | fixed | 0 / 34 |

  The sideL pattern of `capB_L_*` / `ss_side_*` / `square_and_left` (0 0 0, then 0 3.75 0) tipped 0 / 56 early and
  5 / 41 late.

## Ruled out, or no evidence for it

- **The left hip failing during the evening.** On normal-timing left stances, binned by 10 min from 14:20 to 20:00:
  - the stance hip-roll tracking error at 0.4 s after lift-off stays at 1.49-1.80 deg;
  - its load stays at 103-127, with no trend;
  - the foot-off sag at 0.6 s stays at 2.7-3.3 deg.

  The first tip was at 18:29; the afternoon had too few late stances to compare. The break at 20:13 shows a
  different signature (hip roll load ≈ 12, 2026-11-16 comment), which is not in any of these steps.
- **Supply and heat.** Pack/bus 12.0 V throughout; servo voltage 11.7-11.9 V. Hip and ankle roll temperatures
  28-33 °C, flat after 18:50.
- **Swing-foot scuff or early touchdown.** In the raise, the swing leg's knee, hip pitch and ankle pitch loads and
  tracking errors are the same on tips and clean steps (e.g. R knee load at 0.4 s: -205 against -198). The video
  shows the swing foot up at 0.5-0.9 s on tips and clean steps alike (frames below).
- **The lift gate.** It held 0 ms before every tip. The tips start 0.6 s after lift-off, far from the gate's frame.
- **Step type, lateral stagger.**
  - Not specific (the counts above).
  - Lift-offs with the feet squared (swing hip-roll goal within 0.3 deg of the march value) tipped 5 / 100 late.
  - Staggered ones tipped 0 / 59 early.
- **Position on the board.**
  - Measured with `tools/vision/where.py` on the eMeet frame 0.3 s before each left lift-off; the tags were found on
    364 of 524.
  - Tips were at y median 361 mm, clean late stances at 462 mm.
  - Late stances below y 450 mm tipped 6 / 56; at 450 mm or more, 2 / 55. That is not significant.
  - The power lead runs from the robot's back-left to the floor behind it. A pull from there would roll the robot
    outward on a left stance, not inward. Its tension was not measured.

## Frames looked at

Stills were extracted with ffmpeg from `hw_sessions/2026-11-16/vision/cam/{sony,emeet}_<UTC minute>Z.mp4` (the
`.pts` give epoch ms per frame; the burned-in clock agreed with the record to about 20 ms). Not committed. The Sony
is at floor level in front of the robot, so image-left is the robot's right. The eMeet is high, behind it.

- **`capB_L_2` step 11 (tip) and step 9 (clean, same walk).** Sony, foot level, at 0.5 and 0.9 s after lift-off
  (03:04:10.40 / 10.80 and 03:04:02.40 / 02.80 UTC).
  - The swing (right) foot is off the floor at 0.9 s in both, with a shadow gap under it.
  - The stance sole looks flat in both at this resolution.
  - Clearance of a few millimetres cannot be judged.
- **`capB_L_2` step 11.** eMeet at -0.3, 0.5, 0.8, 1.0, 1.2 s. The torso tilts to the robot's right from about 0.8 s
  and the right foot is down by 1.2 s.
- **`rc_fb_4` step 11 (tip) and step 9 (clean).** eMeet at 0, 0.5, 0.8, 1.0, 1.3 s. Up to 0.5 s the two look the
  same. On the tip the torso leans right from 0.8 s and the right foot is on the floor at 1.3 s. On the clean step
  the torso is upright and the right foot is still up and moving.
- **`ss_side_2` step 9, `poshold_march_M_d` step 29, `square_and_left` step 13.** eMeet at 0.7 and 1.3 s. The same
  picture each time: an inward lean caught by the right foot. The robot was in the back half of the board and the
  lead was taut behind it.

What the frames settle: a tip is caught by the swing foot landing early, and there is no visible scuff before 0.9 s.
What they do not settle: whether the stance sole peels on its inner edge. That needs a close side view of the stance
foot from the front or the back.

## Evidence against guess

| statement | standing |
|---|---|
| tips diverge after the foot-off sag peak (0.6-0.8 s), beyond the stance encoders, caught by the swing foot at ≈ 12 deg | evidence (record + frames) |
| 0 / 202 early against 12 / 163 late left stances in normal timing; 0 / 59 late in fast timing | evidence |
| the raise-window load pattern (stance ankle roll ≈ 0, stance hip roll high, swing hip roll pushing) precedes every tip and grows within normal-timing walks | evidence, 12 events |
| no left-hip degradation trend, no supply or heat effect, no scuff, gate not involved | evidence (no trend or difference found) |
| the learned trim drift drives the load pattern | **guess** (confounded with the step count; no dose-response in clean steps) |
| a double-support preload between the feet grows over a walk | **guess** (the swing hip-roll load before lift-off grows: −8 → −16; the mechanism is not shown) |
| the stance sole peels on its inner edge | **guess** (consistent with the ankle-roll load near 0 and the roll beyond the encoders, not seen on video) |
| fast timing is protected by its short settle | **guess** (the only phase that differs before the split; untested) |

## First test for the next robot session

The hip is reprinted, so run this after the comment's re-baseline march, which compares joint loads and the roll swing
against the 11-16 runs. If the new hip changes the left stance's sag, re-take the 11-16 baseline first
(`stance_tips.py` on that march).

**Question.** Does freezing the learned trim at its seed values remove the late-walk build-up and the tips? Does
holding it at the late-walk values bring them into the first stances?

The pattern that tipped 5 of 41 late left stances on 11-16 is normal timing, `--cmd`, three march steps, then
sideways left at 3.75 mm/s. Run it 24 steps a walk:

```
F="--T 4 --ahead 1 --cmd file:/tmp/walk_cmd --max-steps 24 --live"
tools/vision/where.py --direction left --min-margin 250          # room for ~150 mm left
echo "0 0 0" > /tmp/walk_cmd    # each walk: start marching, write the sideways command after TS 3, stop at 24
#   echo "0 3.75 0" > /tmp/walk_cmd

# A (x3 walks): learning OFF, T at the seeds 5.2 / 3.4
tools/wcli.py 'trim set 0.25 0 15 14 0.6 15 6 8'     # ki 0; every other value the firmware default
tools/wcli.py trim                                   # check: ki 0
python experiments/roll-balance/track_host.py $F --seed 5.2,3.4 --note "tipsA frozen 5.2/3.4"
# B (x2 walks): learning OFF, T at the late-walk values
python experiments/roll-balance/track_host.py $F --seed 6.8,1.2 --note "tipsB frozen 6.8/1.2"
tools/wcli.py 'trim set default'
# C (x2 walks, control): learning on (defaults), as on 11-16
python experiments/roll-balance/track_host.py $F --note "tipsC learn"
python experiments/roll-balance/stance_tips.py <the seven stamps>
```

Each walk has 12 left stances, 8 of them late. Read the `stance_tips.py` summary (left stance, normal):
- tips;
- the median stance ankle-roll load (raise);
- the median swing hip-roll load before lift-off.

These are for k ≤ 4 against k ≥ 5. The 11-16 reference values:
- early: ankle -33, swing hip -8;
- late: ankle -24, swing hip -16;
- tips: ankle ≥ -15 and swing hip ≤ -16.

**Pass/fail:**
- **The learned trim drift is the cause** if A has 0 tips in its 24 late left stances and its loads do not build up:
  the k ≥ 5 medians within 8 counts of A's own k ≤ 4. On 11-16's rate, 0 / 24 by chance is p ≈ 0.05.
  - B should show the late pattern from k ≤ 4 (ankle ≥ -20, swing hip ≤ -16) and may tip early, which never
    happened in 202.
  - Then the fix is in the learner: hold T, or learn only from the static part of e (not from the 0.3-0.8 s sag);
    that is a firmware change for #297.
- **Something else builds up** if A still builds the pattern (k ≥ 5 ankle ≥ 8 counts above k ≤ 4, swing hip ≥ 8
  counts more negative) or tips at k ≥ 5 (≥ 2 tips).
  - Then the trim is cleared, and the next candidate is the double-support preload.
  - Next run: the same pattern with `--settle 0.1 --td-pause 0.3` and the normal shift, the only pre-lift-off
    difference of the fast timing.
  - B should then look like C's early steps.
- **Inconclusive** if C, the control, does not reproduce: fewer than 2 tips and no load build-up. The new hip
  changed the plant; record that and move on.

Stop rule as on 11-16: a tip at or above 11 deg is benign (the swing foot catches it). Any fall, or a lean of 15 deg
or more, ends the arm. A takes about 6 min, B and C about 4 each.

Practical note until then: the faster timing had no tip in 161 left stances (59 late, 48 of those `--cmd`). For
steering, prefer it.
