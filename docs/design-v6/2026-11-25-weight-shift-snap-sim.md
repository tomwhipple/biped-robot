# The weight-shift snap in the sim: a control-loop stall, not a plant term (#259 Round 7, #297)

*A dated record of an offline analysis (2026-11-25, night run, no hardware touched; the robot is down since 11-16). It
follows [the A-vs-B record](2026-11-25-left-stance-tips-a-vs-b.md) of the same night. That record found that every late
left-stance tip of the 11-16 evening followed a "snap" in the weight shift before the lift-off, and that the snap needs
the shift to start within 0.5 s of the touchdown. It listed "replay the recorded goals, as recorded and with the shift
0.3 s later, through the plants" as an optional sim step before the robot. The question here, for the weight-transfer
gate (#259): does any plant reproduce the snap, and if not, what is missing?*

## Result

**The snap is reproduced, and its cause is in the record, not in the plant.** It is a stall of the firmware's control
loop of about 115 ms that comes 0.2-0.5 s after every touchdown. During the stall the servos get no new goals. When the
stall falls on the shift's fastest part, the joints fall behind the shift. When the loop resumes, the goal jumps, the
joints catch up with a load spike, and the body rolls.

1. **The stall (verified in the flight records).**
   - Every double support of the 11-16 afternoon and evening (14:15-20:10) has a tick gap longer than 80 ms. That is
     389 / 389 left stances with the early shift start and 257 / 257 with the late one.
   - The gap ends 0.18 / 0.35 / 0.48 s after the touchdown (p10 / median / p90). It lasts 95 / 114 / 133 ms.
   - It is a stall, not lost packets. The control loop's own tick counter advances by 1 across 49 of 53 gaps sampled
     (13 evening runs), with no ring or network drops. The other 4 are counter jumps of 200-1800 ticks, which were not
     examined.
   - It comes with two host commands, sent once per step after the touchdown:
     - `track steps`: track_host.py polls it when the status reply reports a new single support (the stop rule).
     - `imu`: `heading_step()` sends it when the heading hold is on.
   - A gap of more than 80 ms within 2 ticks of the command:

     | command | gaps |
     |---|---|
     | `imu` | 618 / 618 |
     | `track steps` | 545 / 559 |
     | `track` (the status poll) | 568 / 6855 (8 %) |
     | `track add` | 507 / 5355 (9 %) |

     The last two are about the rate at which they fall next to the other two by chance (every 3rd run of 14:15-20:10, 45 runs).
   - The firmware's `imu` command reads the IMU 50 times in a row to time the read (`cli.cpp`, "read ... mean of 50").
     Its `read` line is printed exactly when the ticks resume. Why `track steps` blocks the loop was not looked into.
2. **The stall is what makes the snap (verified, robot).** On normal-timing left stances with the early start
   (k ≥ 2, 380 stances):
   - With a stall in the shift's fast part (t50 − 0.35 to t50 + 0.15 s, where `tip_causes.py` measures the lag):
     179 / 183 snapped.
   - Without one: 24 / 197 snapped (Fisher p 5e-75). For those 24 the stall ended 0.36-0.41 s before t50, just outside
     the window.
   - All 203 snaps followed a stall that ended 0.05-0.41 s before the shift's midpoint.
   - By where the stall ends relative to t50:

     | stall ends | snapped |
     |---|---|
     | −0.80 to −0.50 s | 0 / 25 |
     | −0.50 to −0.35 s | 24 / 172 |
     | −0.35 to −0.20 s | 162 / 166 |
     | −0.20 to −0.05 s | 17 / 17 |

   - The same holds in the replication sessions (11-13, and 11-16 before 14:15): 151 / 157 with a stall against
     15 / 179 without (p 9e-69).
   - Right stances: 92 / 165 against 6 / 172 (p 5e-29). The stall is needed there too, but only about half of the
     right-stance shifts snap with it.
3. **Why the touchdown pause works (verified, robot).**
   - The stall stays 0.2-0.5 s after the touchdown whatever the timing: the median is 0.31 s with the late start and
     0.35 s with the early one.
   - With the early start the shift's midpoint is 0.71 s after the touchdown, so the stall often falls on its fast part.
   - With `--td-pause 0.3` (and in the 3.47 s recipe) the midpoint is 0.97 s, and the stall is over before the shift
     gets going: on 106 + 151 left stances, 0 and 1 had a stall in the fast part.
   - So it is neither the landing settling nor the trim's fade overlapping the shift, the two halves the A-vs-B record
     could not tell apart. It is when the host's per-step commands block the loop.
4. **The stiff plant reproduces the snap step by step (verified, sim).** `grade` (grade.py's plant, no play, no sag)
   gets the robot's written goals open loop, held over the record's ticks as the servos hold them. On the same 380
   stances:

   | | robot | grade |
   |---|---|---|
   | snap share | 53 % | 48 % |

   - Step by step the snap correlates at r 0.93: both snap 173, robot only 30, sim only 11, neither 166.
   - On the robot's snap steps: peak 3.20 deg (robot 3.40) at −1.25 s (robot −1.24); L hip-roll lag 3.11 deg (robot
     3.96).
   - On no-snap steps: 1.41 deg (robot 1.45); lag 2.25 (robot 2.37).
   - The lag correlates at r 0.89 per step.
   - With the late start as recorded: 0 % on both (106 + 151 stances).
5. **The sim's counterfactuals (sim).** Normal timing, early start, the 380 left stances; snap share on `grade`:
   - **As recorded:** 48 %.
   - **Stall taken out (`fill`):** 0 %. The goals are interpolated through the gaps; peak 0.66 deg, lag 2.08.
   - **Shift 0.3 s later, stall at its real time (`0.3rt`, the `--td-pause` counterfactual):** 0 %. Peak 1.36 deg,
     lag 2.05.
   - **Shift 0.3 s later, stall moved with it (`0.3`):** 47 %. Moving the shift alone does nothing in the sim. What
     matters is where the stall falls on the shift.

   The record's "shift 0.3 s later" step is therefore `0.3rt`: it removes the snap, as the robot's pause runs did.
6. **No plant predicts the tips.** The plants that carry the snap's consequence tip far too often:

   | plant | snap share | r step by step | tips on these 380 (robot 12) | robot tips hit | `fill` | `0.3rt` |
   |---|---|---|---|---|---|---|
   | grade | 48 % | 0.93 | 0 | 0 | 0 | 0 |
   | play_fit (play 6/8/3/0) | 13 % | 0.78 | 3 | 0 | 7 | 19 |
   | sag (6.4 / 2.9) | 11 % | 0.82 | 108 | 10 | 54 (6 hit) | 77 (10 hit) |
   | sag + friction 0.10 | 14 % | 0.85 | 116 | 12 | 50 (5 hit) | 75 (8 hit) |

   - **grade** reproduces the snap but not its consequence:
     - its lift-off state does not depend on it: squeeze −23.4 on snap steps against −22.5 without (robot −30.3
       against −20.0);
     - it swings back hard: e@0.9 −5.3 against the robot's −0.5;
     - it never tips.
   - **sag and sag + friction** carry the consequence but overdo it:
     - the snap is smaller (2.2 deg on the robot's snap steps), but it leaves a preload: squeeze −16.9 / −21.1 on snap
       steps against −5.1 / −5.9 without;
     - their swing-back follows the robot's step by step: e@0.9 r 0.42 / 0.44, the roll at the lift-off r 0.84 / 0.85;
     - they tip on most of the robot's tips, but on 100 or so clean steps too (Round 6's false left tips);
     - taking the stall out halves their tips, but they still tip on 50-54 steps where the robot had none.
   - **play_fit:** its snap is small, and the play takes the preload: squeeze 0. It tips 3 times, none of them where
     the robot did.
   - So the snap is not a plant term. The tips still need what the gate already lacks: the left-stance level and the
     falls (items 1 and 3), now on the shifts with the stall.
7. **Right stances (sim, `grade` and `sag` only).**
   - The robot snaps on 29 % of the early-start right stances (337) and on 7-9 % of the late-start ones.
   - The plants get 8-9 % (r 0.39 grade, 0.81 sag), and `fill` takes the plants' to 0-2 %.
   - The right side's snap is under-reproduced, and part of it has another source (the late-start snaps). Not pursued
     here.

## What this changes

- **For the gate (#259): no plant term.**
  - The candidates in the brief were roll lash or play catch-up, servo lag or dead time under load, the landing not
    settled, and the fade's 0.6 s overlap. None is needed: the stiff plant with grade's servo chain already
    reproduces the snap, given the record's own tick times.
  - Items 1-4 are unchanged: no plant or default was touched.
  - The replays must hold the goals over the record's tick gaps, as they do; interpolating through them removes the
    snap.
  - `footoff_sag.py` places each step 0.4 s after the touchdown before it and holds it 1 s, so it starts after most
    stalls. Its fits (Round 6, the sag plant's 6.4 / 2.9) saw few snaps. That is inferred, not re-run.
- **For #297, the first robot test.**
  - The prediction for the touchdown-pause A/B stands: C snaps on about 55 % of left stances (k ≥ 2), P on under 2 %.
    The mechanism is now known: the pause moves the shift off the stall.
  - The record's test 2 (no pause, the fade shortened to 0.3 s) is now predicted to fail to remove the snaps. The
    stall would still fall on the shift, so it would wrongly read as "the landing settling". Replace it with the arm below.
  - The sharper second arm is **no pause, with the host's per-step `track steps` and `imu` moved out of the double
    support** (e.g. sent after the next lift-off), or the heading hold off and the stop rule's poll delayed. It
    predicts the snap share falls as with the pause, at the normal 4.0 s step. That is a host change: its own PR,
    one change per run. It is not made tonight.
  - A firmware fix (the `imu` command's 50-read timing moved behind an explicit `imu bench`, and `track steps`
    served without blocking the control loop) would remove the stall for every gait. That is also its own change.
- **For the planner (#137) and grading: nothing changes in grading.**
  - Any hardware run that polls `imu` or `track steps` while walking gets ~115 ms control stalls. That includes a
    policy run, if its host polls them (inferred: not checked for the policy path).
  - The training chain's dead time (3 ticks, 60 ms, on the STS3215s) does not cover a stall of that length.
  - Until the stall is gone, a hardware candidate's walk should not poll those commands in double support.

## Data and method

- **Robot.** `tip_causes.py extract` / `table` over the 11-16 flight record (`flight_20261116T211526Z_prec16.frec`,
  134 runs, 14:15-20:10) and, for the replication, 11-13 and 11-16 before 14:15. The robot numbers match the A-vs-B
  record (12 tips, 53-55 % snaps on early-start left stances, 0 % late).
- **Sim.** `experiments/roll-balance/snap_replay.py` (tests `tests/test_snap_replay.py`).
  - Each single support is replayed open loop on a plant. The plant is placed in the double support *before the
    previous* single support (so the landing before the shift is the sim's own), held 1 s, then gets the robot's
    written goals at the record's tick times, zero-order hold, to 0.3 s after this step's touchdown.
  - The sim's roll, roll rate, encoders and servo torques are written over the record's in a copy of its tick array,
    and `tip_causes.run_rows` reads them exactly as it reads the robot.
  - Variants `fill`, `0.3`, `0.3rt`: see the tool's doc.
  - The plants are Round 6's: `grade`, `play_fit`, `sag`, `sag_fric`.
  - The sim's loads are torques scaled to STS counts by the stall torque (inferred scale). The sim's swing hip-roll
    load before the lift-off has the opposite sign to the robot's on the sag plants, so only the squeeze (L − R) and
    the encoders' width are compared.
  - The sim's trim error, and so its e@0.9 and its lean (tips), uses the robot's recorded reference. That reference
    follows the robot's own roll: at the lift-off of the tips it sits 0.6 deg further toward the swing foot than on
    clean steps (median −ref 0.87 against 0.27 deg).
    - So a little of the robot leaks into the sim's e@0.9 and its tip hits; a 0.6 deg leak against a 7 deg tip line.
    - The snap, the lag, the roll at the lift-off, the squeeze and the width are the sim's own.

  ```bash
  export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=1
  cd experiments/roll-balance
  python tip_causes.py extract --day 20261116 --from 1415 --to 2010
  python snap_replay.py run --day 20261116 --from 1415 --to 2010 --delay 0,0.3 --workers 6 --name r7_base
  python snap_replay.py run --day 20261116 --from 1415 --to 2010 --delay 0.3rt,fill --workers 6 --name r7_cf
  python snap_replay.py run --day 20261116 --from 1415 --to 2010 --side R --delay 0,fill \
      --plants "grade=grade;sag=grade+sag_L=6.4,sag_R=2.9" --workers 6 --name r7_right
  python snap_replay.py report results/snap_replay/r7_base.csv results/snap_replay/r7_cf.csv
  ```

  - Each 4-plant pass over the 134 runs takes about 8 min on 6 processes.
  - Each step builds its own plant, and a built plant is not all freed (about 50 MB a step), so the workers are
    recycled every 3 runs.
  - Worker memory stays under 2 GB.
- **The stall statistics** (gap times, the commands at them, the tick counters, the snap by the stall's position) came
  from `build/` scripts over the same caches and the flight record's events. They were not kept as a tool; the
  numbers above are the record.
- **Verified against inferred.**
  - Verified: the stall (the tick counter, the commands, the timing); the snap's dependence on it on the robot (both
    sides, two session sets); the sim's step-by-step reproduction; and the sim's counterfactuals.
  - Inferred:
    - that the `imu` command's 50-read timing is what blocks the loop (the `read` line prints as the ticks resume);
    - that a pause-free fix with the polls moved would remove the snap on the robot (the sim's `fill` says so; no
      robot run has it);
    - that `footoff_sag.py`'s fits saw few stalls;
    - that a policy run's host polls the same commands.

## 2026-11-25: the stall's cause and fix

*Code reading and host tests only (night run, no hardware; nothing flashed). The firmware fix is on PR #355 (branch `fw/cli-no-stall-297`), not on `main` and not yet flashed; the
host flag `--polls-in-ss` is on `main`. Line numbers
are `origin/main` at 303f52c4 unless marked "new".*

**Cause (verified in the code).** The track player is not in the firmware's control task. It runs on the task that
executes the CLI.
- `ctrl_task.cpp:1095` (core 1, priority `configMAX_PRIORITIES - 2`) runs only the armed policy loop.
- The bench pose stream runs in `cli::poseTick` (`cli.cpp`, new 3895): the track player (`trackTick`, new 3954), the stance
  trim and `balance`, at 50 Hz.
- `poseTick` is called from `houseTask` (`link_task.cpp:194`). That loop first drains the CLI queue (`link_task.cpp:147-155`):
  every tether line and every Wi-Fi line (`wcliExecute` -> `cli::execute`) runs to completion before the next tick.
- So task priorities and core pinning play no part. A command blocks the loop for as long as it runs, on the same task.
- The plain `imu` command (`cli.cpp:4927-4987`) read the part 51 times: `dev->read(s)` once, then 50 more
  (`kN = 50`, `cli.cpp:4979`) to print `read ... ms mean of 50`. Each read is an I2C burst plus a fusion update. The
  250 Hz IMU sampler, at a higher priority, takes the same I2C lock in between. That makes the ~115 ms. It also ran the
  shared fusion filter from a second task, alongside the sampler.

**Why `track steps` and the other commands show up (inferred from the host's command order and the counts; not
re-checked against the record).**
- `track steps` is not slow: it formats at most 17 lines from RAM.
- `track_host.py` sends it immediately before `imu`: the stop rule's `scan(r.cmd("track steps"))`, then
  `heading_step()`, which sends `imu`.
- Its reply comes back within a tick or two of the `imu` stall starting, so the "within 2 ticks of a gap" test counts it
  (545 / 559).
- The 14 without a stall would be the steps where the stop rule tripped: `heading_step()` is skipped then, so no `imu`
  follows.
- `track` (568 / 6855) and `track add` (507 / 5355) are the commands just before and just after that pair, once per
  step. 568 and 507 are about the step count, which fits one adjacency per step. They do not cause stalls of their own.
- A check against the record: the nearest `imu` event to each `track` / `track add` / `track steps` gap.

**Also in the policy path (from the code).** Armed, the policy runs in `ctrl_task` on core 1 and reads the IMU through
`takeImu()`. A host's `imu` poll does not stall that loop. It did still race the sampler's fusion. The Round 7 caveat "that
includes a policy run" does not hold for the tick.

**The fix (on PR #355, not on `main`, not yet flashed; the items below describe what that branch does).**
1. Plain `imu` prints the sampler's latest sample (`peekImu`), the one the loop uses, and reads only the ID registers.
   `up`, `gyro`, `bias`, `mount`, `levelled`, `quat` and `projgrav` keep their formats (`track_host.yaw_of_imu`,
   `robot_io.imu`, `board_bridge`). The last line `read ... mean of 50` becomes `sample <n> ms old ...`. No host tool
   parses it (`git grep`).
2. The 50-read timing is now `imu time` (allowed over Wi-Fi). `imu time`, `imu raw <n>` (n > 1), `imu ae` and `imu bias` are refused
   while the bench loop plays a pose, a track or a trace tail (`benchLoopBusy`). Each of them blocks for 0.1-2 s.
3. `track steps` formats its whole reply first and hands it to the sink once. That takes the Wi-Fi CLI lock once
   instead of 17 times. The text is the same.
4. `wifi_cli.cpp`: the wcli task held its lock (`g_mu`) across `lwip_sendto` of up to 8 chunks (`wifi_cli.cpp:82`, `:130`).
   Housekeeping's prints take that lock, including inside the loop's own iteration (`wcliAsyncOutput`, `wcliHouseTick`).
   The task now copies the datagrams under the lock and sends them after it is released.

**Not fixed: the design that made this possible.** Any other command that blocks still stalls the walking loop, because
the CLI and the loop share one task. Examples are a bus read while a track plays, `cal`, and NVS writes. The general fix is
to run `poseTick` from its own task, or to give the bench loop an allowlist of fast commands. That is a larger change with
its own risk, and it is not made here.

**Tests (host; the firmware ones are on PR #355).**
- `tests/test_cli_no_stall.py` checks the source for:
  - that the CLI and the loop share the task;
  - that plain `imu` does no read, loop or delay;
  - that the blocking `imu` forms come after the bench-loop gate;
  - that `track steps` prints once;
  - that the socket sends are outside the lock;
  - that the reply still parses with the host tools.
- On the old source, 4 of these 6 fail. `cli.cpp` builds only under ESP-IDF, so a SIL timing test cannot reach it.
- `firmware/host/test_wcli.cpp` (PR #355) allows `imu time` over Wi-Fi.
- On PR #355, `make firmware` and `make firmware POWER_LITE=1` build.

**The host arm, for the old firmware: `track_host.py --polls-in-ss`.** It is opt-in and off by default.
- The per-step `track steps` and `imu` wait for a planned single support: 60 ms after its lift-off to 300 ms before its
  touchdown, with the lift gate not holding.
- If no window comes within 3 s, the poll is sent anyway and counted as `forced` in the summary row (`polls_in_ss`).
- The cost: the stop rule acts up to one double support later.
- On the old firmware the stall still happens, but now in single support, in the raise. Watch the swing foot and the TS
  lean there.
- Tests: `tests/test_polls_in_ss.py`.

**Robot sequence (#297), mildest first; one change per run.**
1. Old firmware (as flashed), no `--td-pause`: control against `--polls-in-ss`, alternated.
   - Predicted: control snaps on about 55 % of left stances (k ≥ 2), as before.
   - Predicted: `--polls-in-ss` on about 0 %, with no tick gap > 80 ms in a double support. The sim's `fill` gives
     0 %, and the pause gave 1 of 257 on the robot. The ~115 ms gaps move into the single supports.
   - This settles the mechanism without a flash.
2. Then the new firmware (PR #355, after Tom flashes it), no flag, no pause (today's defaults).
   - Predicted: no tick gap > 80 ms anywhere in the walk.
   - Predicted: the snap share falls to about 0 % without `--td-pause`.
   - `imu time` still answers on the bench and is refused while a track plays.
