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

**Every other CLI line: the bench-loop allowlist (second pass, same night).** Any other command that blocks would still
stall the walking loop, because the CLI and the loop share one task. So `cli::execute` now asks one rule before it
dispatches any line, tether or Wi-Fi.
- While the bench loop plays (`s_traj_active || s_track_on || s_trace_on`), `benchLoopRefuses` (`cli.cpp`, new) refuses a
  line that can block the task for more than about 10 ms, half a 20 ms tick. The reply is
  `<cmd>: refused -- the bench loop is playing (a track | a pose stream) on this task and this would stall it: <why>`.
- The rule is `wcli::benchLoopRefusal` (`firmware/components/wcli/policy.cpp`). It is a pure function of the line, so it
  is host-tested. It is default deny: a command word it does not name is refused while the loop plays.
- Never refused: the loop's own commands (every `track` form), the stop, disarm and safety paths, and the commands that
  take the stream over. `cmdImu` keeps its own gate from the first pass.
- Not changed: track_host.py. Every command it sends during a walk is fast: `track`, `track add`, `track steps`, `imu`,
  `track stop`, `track halt`. Before the walk it sends `liftgate set` and the play line; after it, `trim` and `imu`.
  `board_bridge.py` polls `imu raw 1`. `stepper/robot_io.py` sends `cal show` at start-up and then `imu`, `pos <id>`,
  `pose`, `torque`, `release`, `home` and `fall`. Of those, only `cal show` is now refused, and only while a stream plays.

The audit. Estimates from code reading. The bus runs at 1 Mbaud. One register read or ping is about 0.3 ms, and a servo
that does not answer costs the 3 ms reply timeout (`scsbus/bus.h:53`). An EEPROM write waits `kEepromCommitMs` = 2 s
(`cli.cpp:79`). An NVS write can erase a flash sector, which takes tens of ms. The tether is 115200 baud with a 512 B TX
buffer (`app_main.cpp:185`), so a reply longer than 512 B blocks at about 87 us per byte beyond that. A Wi-Fi reply goes
into a 6.4 KB ring (`wcli.h` `kRing`). It waits only when the client's window is full, for up to `kStallMs` = 2 s.

| command (form) | what blocks | estimated block | now, while the loop plays |
|---|---|---|---|
| `track` (status, `add`, `steps`, `stop`, `halt`, `clear`, `rest`, `play`) | RAM; `steps` formats ≤ 17 lines once | < 1 ms of work | runs (the loop's own) |
| `imu` (plain) | the sampler's sample, plus the ID registers | ~1 ms | runs (track_host, robot_io) |
| `imu raw` / `imu raw 1` | one I2C burst | ~2 ms | runs (board_bridge) |
| `imu ring`, `imu avg`, `imu gscale`, `imu mount` (print forms), `imu bias online [on\|off]` | RAM | < 1 ms | runs |
| `release [id]`, `bench`, `fall ...`, `flail ...` | one broadcast write, or a RAM latch | < 1 ms | runs (stop / disarm / safety) |
| `rail`, `rail off`, `rail clear`, `rail panic` | a RAM request; `panic` aborts on purpose | < 1 ms | runs (safety) |
| `home` | torque on, 17 position reads, one write; power-lite: rail polls 20 + 10 + 5 ms | measured 102-122 ms (30 / 30) | runs (the recovery path; never blocked) |
| `pose`, `balance` (not `log`) | take the stream over; `pose` refuses itself while a track plays; `pose ... S<L\|R>` samples the roll 5 x 6 ms | `pose`: measured 68-119 ms (p10-p90, 341 / 346), at the handover | runs (a takeover, not a stall of a walk) |
| `torque [id]`, `move <id>`, `pos <id>`, `reg`, `ping <id>` | 1-2 bus transactions | ≤ 3-6 ms | runs |
| `stream stop`, `stream status`, `sweep stop` | RAM | < 1 ms | runs |
| `trim ...`, `liftgate ...`, `heading ...`, `stat`, `ntp`, `shape`, `limit`, `standclock`, `obsfreeze` | RAM settings and status | < 1 ms of work | runs |
| `batt`, `batt pack on\|off`, `rec`, `rec status`, `wifi`, `wcli`, `obsdump off` | RAM | < 1 ms | runs |
| `imu time` | 51 IMU reads | ~115 ms | refused (first pass) |
| `imu raw <n>` (n > 1), `imu ae`, `imu bias` (calibration) | 100 ms between samples; seconds of sampling | 0.1-2 s | refused (first pass) |
| `imu scan` | probes 127 I2C addresses | tens of ms or more | refused |
| `imu reinit` | I2C bus recovery and a re-init of the part | tens of ms (not measured) | refused |
| `imu forget`, `imu gscale <x y z>`, `imu mount <w x y z>`, `imu bias online save` | NVS write or erase | tens of ms | refused |
| `scan` | pings IDs 0-253; 3 ms per missing ID | ~0.7 s | refused |
| `ping` (no id) | every fitted servo; 3 ms per missing one | 5-51 ms | refused |
| `cal`, `cal show` | 17-line print (~0.9 KB); other forms read every servo or write NVS | ~30 ms on the tether; more | refused |
| `gains`, `gains <id>` | P/D reads of every servo, plus a 17-line print; the write form waits for EEPROM | ≥ 10 ms; 2 s | refused |
| `servocfg` | response registers of every servo | ≥ 10 ms | refused |
| `volt` | the INA219, plus a voltage read of every servo | ~6 ms; ~50 ms with servos missing | refused |
| `run` | arming re-reads every servo's gains | ≥ 10 ms | refused (arming mid-stream is not a stop path) |
| `id`, `middle`, `regw` | an EEPROM write and its commit wait | 2 s | refused |
| `sread`, `stepresp` | servo identification runs | tens of ms to s | refused |
| `nettime` | n policy forward passes per backend | hundreds of ms | refused |
| `bno` | 5-25 ms polls of the BNO085 | ≥ 25 ms | refused |
| `help`, `?` | ~4.4 KB of text | ~340 ms on the tether | refused |
| `trace` | up to 400 lines (~8 KB) | ~0.7 s on the tether | refused |
| `balance log` | up to 600 lines | ~2 s on the tether | refused |
| `stream` (start, `dump`), `sweep` (start) | a task start, the whole log; a roll sweep | > 10 ms | refused |
| `batt reset` | waits 60 ms for the control task | 60 ms | refused |
| `rec on\|off\|idle\|<ip>` | saves the recorder config to NVS (tether, benched) | tens of ms | refused |
| `obsdump on\|once`, `wifi <ssid> <psk>`, `wifi clear`, `wcli key ...`, `wcli drop` | a CSV line per tick; NVS writes | tens of ms | refused |
| `rail on` | waits for the rail (20 + 10 + 5 ms polls) | ≥ 35 ms | refused (the rail is up while anything moves) |
| any other word | (default deny) | - | refused |

**What the flight records show.** All 109 records in `hw_sessions/` (10-30 to 11-21) were scanned: 111,170 CLI lines, of
which 3,695 came over the tether. A line counts as sent "while playing" when the tick before it was a bench-pose tick
with the trajectory flag set. That flag is a proxy, not checked against the track logs. It counts as stalled when a
tick gap > 50 ms falls within 2 ticks of it. Gaps over 30 ms are a quarter of all tick pairs while playing, so 30 ms is
noise; 50 ms is 0.9 %.

| sent while playing | n | stalled | stalled gap, median / max |
|---|---|---|---|
| `imu` | 5514 | 100 % | 107 / 148 ms |
| `track steps` | 2754 | 86 % | 126 / 148 ms |
| `pose` | 346 | 99 % | 79 / 141 ms |
| `home` | 30 | 100 % | 112 / 122 ms |
| `trim` | 76 | 76 % (15 over 80 ms) | 76 / 120 ms |
| `pos <id>` | 1321 | 3.9 % | 111 / 124 ms |
| `track` | 39133 | 1.2 % (chance) | 56 / 140 ms |
| `track add`, `track stop`, `track halt`, `fall`, `stat`, `release` | 26523 | 0 | - |

- `imu` and `track steps` are the first pass's stall. `track steps` stalls as `imu`'s neighbour.
- `trim` is RAM only in the code (`trim T` sets a float). Its stalls are most likely its neighbours: track_host's
  wrap-up sends `track steps`, `track`, `trim`, `imu`. That is inferred, not checked.
- `home` and `pose` stall by themselves. They stay allowed; see below.
- Idle, with no stream playing, these stalled nearly every time: `track play`, `balance set`, `run`, `stepresp`, `gains`,
  `sread`, `scan`, `rail on`, `servocfg`, `sweep`. All but the first two are refused now. `cal show` stalled 40 % of
  the time.

**Still open after the allowlist (findings, not fixed).**
- Tether output. An allowed reply longer than the 512 B UART buffer still blocks on the tether. A full `track steps`
  (~1.1 KB) costs about 50 ms there, and the status prints of `flail` and `trim` cost about 10 ms. The walks run over
  Wi-Fi, so this does not touch them. The cheap fix is a 2-4 KB tether TX buffer (`app_main.cpp:185`). It costs RAM and
  is not made here.
- Wi-Fi output. When the client's window is full, `wifiSay` and `endCommand` wait up to 2 s. That happens with a lagging
  laptop or a bad link, and it can stall the loop on any command, allowed or not. A command rule cannot fix it. The fix
  would be to drop, not wait, while the bench loop plays.
- `home` (~110 ms) and `pose` (~80 ms) block for well over 10 ms. They are allowed on purpose: one is the recovery
  path, and the other takes the stream over. While `pose` runs, the old stream holds its last goals for ~80 ms. The
  retired stepper (`stepper/robot_io.py`) walked with pose after pose, so this matters if it comes back. Where `pose`
  spends the 80 ms was not looked into.
- The structural fix is still to run `poseTick` on its own task. It is not made here.

**Tests (host; the firmware ones are on PR #355).**
- `tests/test_cli_no_stall.py` checks the source for:
  - that the CLI and the loop share the task;
  - that plain `imu` does no read, loop or delay;
  - that the blocking `imu` forms come after the bench-loop gate;
  - that `track steps` prints once;
  - that the socket sends are outside the lock;
  - that the reply still parses with the host tools.
- On the old source, 4 of these 6 fail. `cli.cpp` builds only under ESP-IDF, so a SIL timing test cannot reach it.
- `firmware/host/test_wcli.cpp` allows `imu time` over Wi-Fi. Its `testBenchLoopGate` checks 57 lines that must run
  and 43 that must be refused while the loop plays.
- The second pass adds three source checks to `tests/test_cli_no_stall.py`:
  - every line passes the gate before it is dispatched;
  - every command word `execute` dispatches is named in the rule;
  - the stop paths and the walk's polls are decided before any refusal.
- `make firmware` and `make firmware POWER_LITE=1` build.

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
