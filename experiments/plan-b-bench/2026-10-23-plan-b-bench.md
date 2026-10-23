# Plan B bench, session 4 (2026-10-23): the STS3250 on the v3 rig

Issue #73. The STS3250 Tom ordered on 10-02 for the hip rolls, run through the
same rig, weights, controller and scripts as the STS3215 in session 3
([2026-10-02-plan-b-bench.md](2026-10-02-plan-b-bench.md)), for a direct
comparison.

How the session ran:

- **Operator:** the Claude session on Mira ran it as `claw`. Tom set up the rig,
  fixed the wiring and gave the goes.
- **Video:** every motion run was filmed on the Sony A7 III
  (`tools/sony_record.sh`, new today: live view over USB, UTC burned into every
  frame plus a `.pts` sidecar). Videos are on Mira under `hw_sessions/2026-10-23/`,
  not in git.
- **Raw data:** every run's JSON, `session.log` and console are in
  [`2026-10-23/raw/`](2026-10-23/raw/). The first line of each console gives the
  exact parameters.

**Result.**
- **The STS3250 limit-cycles from P 96 up**, at about 18–21 Hz in the rotation
  direction, after a move. At P 96 (D 32 or D 0) the fork shakes at **1.0–2.7 g
  rms** once a 90° move ends; at P 128 / D 32, 2.4–4.1 g. The servo's own
  telemetry shows it only as a 10–12-tick residual range. The STS3215 at P 96 /
  D 0 shook 58–181 mg in the same tests (session 3).
- **P 64 / D 32 is the STS3250's clean raised gain:** it holds, settles in
  0.4 s and sweeps without a backward sample. It shakes about 1.2–1.8× stock in
  sweeps and 1.6× under the firmware's 50 Hz streaming, and never limit-cycles.
- **Stock P 32 / D 32** shakes about as much as the STS3215 at stock: 36–76 mg
  in sweeps, 100–142 mg under 50 Hz streaming (STS3215: 52–85 and 65–90).
- **D works the other way from the STS3215.** At P 96, D 32 settles hanging and
  D 0 does not; at P 128, D makes no difference. On the STS3215, D 0 was the
  smoother setting.
- **Stiffness was not measured** (no bottle runs). The level probes are
  quantization-limited (midpoint +3 / +2 / +1 tick at P 32 / 64 / 96).

## The servo

`gain_bench.py read 1` (`raw/sts3250_read/`): Model_Number **2825** (genuine
STS3250), servo firmware 3.10, factory P / D / I 32 / 32 / 0, 12.1 V, 24 °C, no
status bits. Three registers differ from the STS3215's factory set
(`2026-09-30/bare_ladder_summary.json`):

| register | STS3215 | STS3250 |
|---|---|---|
| min_start_force | 16 | 0 |
| CW / CCW dead zone | 1 / 1 | 0 / 0 |
| overcurrent_time | 200 | 250 |

Renamed 1 → **31** (`id 1 31` replied `timeout` but landed, reg 5 = 31; lock
reg 55 read 0 until the first gains write relocked it, the #101 quirk).
After the session Tom had it renamed **31 → 51** (`id 31 51`: `ok, verified after
commit`, reg 5 = 51, lock reg 55 = 1; `hw_sessions/2026-10-23/setid51/`). The runs
below ran as id 31.
The second STS3250 (the other hip roll; factory id 1, Model_Number 2825, P / D
32 / 32, never benched) was renamed **1 → 52** (`id` replied `timeout` but landed,
reg 5 = 52; lock reg 55 = 0 until `gains 52 32 32` relocked it to 1;
`raw/setid52/`).

## The rig, calibration and direction

- **Same v3 rig and washer weights as session 3.** The weights are in the
  **r 70 and r 100** holes (photo against the CAD: the r 40 hole is empty).
  Their mass is still unmeasured.
- **Plumb 1254**, the released hanging rest (load 0 at P 32). **Level 230.**
- **Out is minus ticks**, as on the STS3215: `2026-10-23/direction_check.py`
  moved ±45 ticks at plumb on camera; at −45 the fork bottom sits lower and
  larger in the frame (closer to the camera, away from the table). The table's
  frame is a few cm behind the fork, so this was checked before any 90° raise.
- **Accelerometer:** the same GY-BNO08X at 0x4B on P1. It did not answer at
  first: SDA and SCL were reversed at the lead (Tom fixed it). A reversed lead
  held the bus until `imu reinit`. Rest check (`raw/bno_check/`): 510 Hz,
  |a| 1011.6 mg, sd 7.3 mg (session 3: 988 / 3.5). Its distance from the axis is
  still unmeasured.
- **Controller:** the General Driver, firmware 51c6fea (session 3's). Opening
  the port did **not** reset it today (no banner, ~21 h uptime). After a 12 V
  power cycle the ESP32 stayed silent until EN was pulsed (RTS).

## Hold ladders (servo telemetry only)

`hold_clamp.py` with `SID=31 LEVEL_T=230 DOWN_T=1254`, D 32 ladder, D 0 ladder
and P 64 / 96 / 128 with D 32 / 64 (`raw/gain_bench_hold/`). **These ran before
the accelerometer was fixed**; Tom: no good data without it. They are kept for
the telemetry, not as the comparison.

| P / D | hanging: quiet range, 2° moves | level: quiet range (after the raise), 2° moves | raise / lower backward samples |
|---|---|---|---|
| 32 / 32 | 0, settle 0.40 s | 0, settle 0.40–0.44 s | 0 / 0 |
| 64 / 32 | 1, settle 0.40–0.43 s | 0, settle 0.37–0.43 s | 0 / 1 |
| 96 / 32 | 0, settle 0.40 s | 0, settle 0.37–0.41 s | 0 / 53 |
| 96 / 64 | 0, settle 0.37–0.41 s | 0, settle 0.38–0.40 s | 0 / 61 |
| 96 / 0 | 0, **never settle (residual 8–10)** | 0, settle 0.38–0.45 s | 84 / 1 |
| 128 / 32 | 0, **never settle (10–12)** | **9**, then settle after probes | 195 / 64 |
| 128 / 0 | 0, **never settle (10–12)** | **10**, then settle | 180 / 67 |
| 128 / 64 | 0, **never settle (10–12)** | **9**, then settle after probes | 160 / 59 |
| 160 / 32 | **14, never settle** | **11, never settle** | 159 / 185 |
| 160 / 0 | 0, **never settle (14–15)** | **11**, moves partly settle | 188 / 150 |

## Sweeps with the accelerometer

`BNO=1 GAINS=32:32,64:32,96:32,96:0,128:32 PROFILES=A,A300,A1500 sweep_smooth.py`
(`raw/sweep_bno/`, run 1 then run 2 for the last two gains). Rotation-direction
(y) acceleration rms above 5 Hz, raise / lower, mg; the window includes ~2.5 s
after the move ends.

| P / D | A (100 steps/s) | A300 | A1500 | residual after the move |
|---|---|---|---|---|
| 32 / 32 | 36 / 53 | 54 / 76 | 76 / 74 | 0 |
| 64 / 32 | 60 / 97 | 63 / 91 | 86 / 89 | 0 |
| 96 / 32 | 90 / **1054** | **1445 / 1192** | **1034** / 126 | 10 ticks |
| 96 / 0 | 94 / **1022** | **1544 / 1269** | **1095** / 99 | 10 ticks |
| 128 / 32 | 173 / **2390** | **3519 / 4101** | **3078 / 4042** | 10–12 ticks |

- **The g-level rows are the limit cycle, not sweep roughness.** Peak 17.8–20.6 Hz
  in y with its 2× harmonic in x; a ±5-tick swing at 18 Hz at about 0.1 m is
  about 1 g, matching the 10-tick residual the servo reports.
- STS3215 for comparison (session 3): P 32 52–85 mg, P 96 / D 0 57–111 mg at
  walking speeds; no limit cycle below P 160.

## The firmware's 50 Hz streaming

`GAINS=32:32,64:32,96:32 stream50.py` (`raw/stream50/`). y rms, mg, raise / lower,
the robot's rule (`goalSpeedSteps`, acc 0):

| P / D | 4 s sweep | 2 s sweep |
|---|---|---|
| 32 / 32 | 100 / 103 | 142 / 142 |
| 64 / 32 | 162 / 195 | 223 / 226 |
| 96 / 32 | 242 / **1642** | 320 / **2675** |

STS3215 (session 3): P 32 65–90 mg, P 96 / D 0 91–181 mg. The "matched" mode
lags ~120–140 ticks rms as in session 3 and ends with a 13–14-tick range at
every gain.

## Script changes

- `hold_clamp.py`, `sweep_smooth.py`, `stream50.py` take `SID` from the
  environment (default 30).
- **Accelerometer guard** (Tom: if the accelerometer is lost, stop the test so
  he can fix it): `sweep_smooth.bno_collect` raises `AccelLost` on fewer than
  250 samples/s, a gap over 100 ms or a recording covering under 80 % of its
  window; a failed `bno rec` raises it too. Both scripts then lower to plumb at
  100 steps/s and release there. It tripped once falsely: the 115200 tether
  dropped digits from one timestamp (943878 between 9438716 and 9450722), so
  samples out of order with their neighbours are now dropped first and logged.
- `sweep_smooth.py` accepts a start up to 200 ticks past plumb on the table side
  (the fork rested at 1365 after the rewiring) and lowers from there.

## Not measured

- **Stiffness** with bottles at P 32 / 64 / 96 (the STS3250's ratio to stock).
- **The loads:** washer mass; the accelerometer's distance from the axis.
- Whether a dead zone of 1–2 (the STS3215's factory 1) stops the P 96 limit
  cycle.
