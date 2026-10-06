# Reproducing the Plan B bench

How to rebuild the issue #73 bench and repeat its three sessions (2026-09-30,
10-01, 10-02) on your own desk, and how to compare your numbers with ours.

The question the bench answers: if you raise an STS3215's position-loop P
above the factory 32, how much stiffer does it get, and when does it start to
buzz, limit-cycle or shake? The results are in the dated session records
beside this file. This page covers how they were measured, and it is honest
about what we did not measure.

## What you need

| item | what we used | notes |
|---|---|---|
| servo | one Feetech **STS3215**: model-number register 777, servo firmware 3.10 | Factory registers as found are in `2026-09-30/bare_ladder_summary.json` (`registers_as_found`). Read yours with `gain_bench.py read` before you start. A different firmware may behave differently |
| supply | 12.1 V at the servo (the servo's own voltage reading) | The source was not recorded. A 3S pack or a 12 V bench supply gives that range |
| controller, sessions 1–3 morning | Waveshare **Servo Driver with ESP32** (ESP32-D0WD-V3) | the robot prototype's first board |
| controller, session 3 afternoon | Waveshare **General Driver for Robots**, rev 1.2 | Needed for the accelerometer: its I²C bus is on header P1. Both boards put the servo bus on UART1 at 1 Mbaud, GPIO 18/19, so the same firmware drives the servo on either |
| accelerometer (session 3 afternoon only) | a **GY-BNO08X** breakout (BNO08x at I²C 0x4B). It was sold as a BNO055 | Wired to P1: 3V3, GND, SDA (GPIO 32), SCL (GPIO 33), see `figs/bno055_wiring.png`. Glued near the fork tip. Its distance from the axis **was not measured**: measure yours |
| host | Linux, the repo's `.venv` (`pip install -r requirements.txt`), the user in `dialout` | The tether is the board's USB serial (CP2102) at `/dev/ttyUSB0`. On macOS, pass `--port /dev/cu.usbserial-…` |
| camera | any UVC webcam, plus ffmpeg | `tools/cam_record.sh /dev/video0 <out_base> <seconds> <label>` burns the wall clock into each frame |
| printer | any FDM printer | Material and slicer settings for the prints were not recorded. The v2 parts table assumes PLA |

## Build the rig

There are two rigs. Session 2 used **v2**; session 3 used **v3**. If you build
only one, build v3: it fixes v2's twist and off-plane load.

**v2 (session 2):** `stl/plan_b_bracket.stl` + `stl/plan_b_lever_stiff.stl`
(+ `plan_b_stop_pin.stl`). Print orientation, screws (4 × M2.5×8 flat-head
into the idler face, 4 × M3×10 button-head lever to horn), seating and fit
checks are in [2026-09-30-plan-b-rig.md](2026-09-30-plan-b-rig.md).
Regenerate the STLs with `plan_b_rig.py`.
- Known weakness, reported by Tom on 10-01 (quoted in `plan_b_rig_v3.py`): "a
  fair amount of twisting on the vertical plate". The plate holds the servo by
  one face, and the lever sits about 45 mm in front of it.

**v3 (session 3):** `stl/plan_b_v3_bracket.stl` (50 × 127 × 39 mm on the bed,
94 cm³ solid) and `stl/plan_b_v3_fork.stl` (134 × 44 × 20 mm, 33 cm³), already
in print orientation: the bracket on its base, the fork on its floor.
- **Bracket:** the robot's leg-link case grip on both case faces, closed into a
  box. The servo goes in with 6 × M2.5×8 flat-head screws: 4 on the horn face,
  2 on the idler face.
- **Fork:** one arm on the horn and one on the idler disc, 4 × M3 each, joined
  by a floor. It is the robot's own joint.
- **Weight holes:** M10 holes through the floor at r = 40, 70 and 100 mm, on
  the servo's mid-plane.
- `plan_b_rig_v3.py` regenerates the STLs and runs the fit checks, including
  the fork swept from level to hanging.

**Mounting:** clamp the bracket to a desk edge with C-clamps. Keep every clamp
out of the fork's swing from level down to plumb and a little past it. On
10-02 the fork ran into the stand's own clamp (the session-3 record,
"Incident").

## Loads

| session | load | what we know | what we did not measure |
|---|---|---|---|
| 2, runs A–C | 12 fl oz Gatorade bottles in a bag, on a string from the v2 lever's 100 mm notch, 1–4 bottles | nominal 0.39 kg each, about 0.38 N·m per bottle | **unweighed**: absolute stiffness carries that error; ratios to P 32 do not |
| 2, run D | one C-clamp on the v2 lever near 100 mm | about 0.36 kg (estimate). Its P 32 probe midpoint puts its torque at about 0.45 N·m and its inertia at about 0.006 kg·m², about 2.3× #73's "leg-like" 0.25 kg at 0.1 m | mass, CoM and inertia are estimates |
| 3, all | 2 × M10 flange bolts with flange nuts, 16 × 3/8″ fender washers each, through the v3 fork floor | rigid, on the mid-plane; torque at level about 0.4× the C-clamp's (≈ 0.18 N·m) | **which holes, and the mass**: record both |

To make your results comparable, weigh every load and record its radius.
#73's target inertia is 0.25 kg at 0.1 m (0.0025 kg·m²).

## Firmware

| when | board | firmware |
|---|---|---|
| sessions 1–2 and the 10-02 morning | Servo Driver with ESP32 | `main` at 98c9ced (the bench `gains` command) |
| 10-02 14:31 | General Driver | 1d86445: adds `bno`, but its driver looked for a BNO055 at 0x28 and found nothing |
| 10-02 14:39 on | General Driver | 2e9516b: `bno` reads a BNO08x over SHTP |
| 10-02 16:3x | General Driver | the `stream` command, committed in 4e145cb |

Current `main` has every command used here (`id`, `move`, `reg`, `gains`,
`bno`, `stream`, `imu reinit`), so flash `main`. See
[firmware/README.md](../../firmware/README.md), "Build and flash". On Linux we
flashed at `-b 115200`, because higher rates dropped mid-image. The board boots
benched: nothing moves until a bench command moves it.

Two firmware quirks you will hit (#101):
- **EEPROM writes lose their ack.** A gains or ID write often reports a
  failure but has landed. `gain_bench` judges every write by reading it back
  (P/D and lock register 55). Do the same if you write by hand.
- **`id <old> <new>` can report `timeout`** and still rename the servo, leaving
  the EEPROM unlocked until the next gains write.

## Prepare the servo

1. Put it alone on the bus and give it an ID that is **not** on the robot's bus
   map, so no envelope clamp applies. We used 30 (`id 1 30`). Session 1 ran as
   11 with `--allow-bus-id`, which clamps `move` to ±25°.
2. Read and keep its registers: `.venv/bin/python
   experiments/plan-b-bench/gain_bench.py read 30`.
3. **Find level and plumb.**
   - **What worked:** let the arm hang, align it by eye to a hanging string,
     release it there, and read the position (`pos 30`). That is plumb
     (`DOWN_T`), good to about a degree. Level is plumb ∓ 1024 ticks (90°),
     toward the side gravity pulls from.
   - On our v3 rig, gravity pulled toward higher ticks: plumb 3531, level 2507.
   - **What did not work:** finding level from the peak holding load. It was
     153 ticks off (`2026-10-02/calibration_summary.json`).
4. Every script takes `LEVEL_T` and `DOWN_T`. **Set them for your rig**, because
   the defaults are ours.

## Safety rules (learned the hard way, session 2)

- **Never release torque with a load on the lever** unless the lever hangs
  plumb. A released loaded lever back-drives and falls: we dropped a bottle
  this way.
- On our servo the bare lever stayed where it was released, at every rung of
  runs B and C. After the first drop, though, the lever was released level and
  fell 81° within 2.5 s. Whether a bottle was on is not recorded (the camera
  view was blocked), and the cause is unknown. Treat any lever as able to fall.
- Write gains only with torque off, and only at plumb. The scripts enforce
  this. Any abort away from plumb keeps torque **on**.
- Film every session from before the port opens until a few seconds after it
  closes. Opening the CP2102 port resets the board. Two still frames 8 minutes
  apart could not show how the first drop happened.
- Keep the stand's clamps clear of the swing (above).

## Run the sessions

Run every script from the repo root, in a terminal: they prompt for a `go`
before each motion. Each one writes `hw_sessions/<date>/<run>/session.log` and
a JSON of every sampled window. Servo telemetry is polled at about 30 Hz.

| # | what | command (our parameters) | our raw data and summary | record |
|---|---|---|---|---|
| 1 | registers + unloaded P ladder: 5 s hold and 2° steps at full speed, P/D 32/32 … 160/160, 128:32, 160:32 | `gain_bench.py hold 11 --allow-bus-id --orient down --inertia "none (bare horn)"` | `2026-09-30/raw/`, `bare_ladder_summary.json` | 2026-09-30 |
| A | one bottle: approach level from ±40 ticks, twice each, P 32 … 160 | `SID=30 LEVEL_T=… 2026-10-01/one_bottle.py` | `2026-10-01/raw/gain_bench_onebottle/`, `one_bottle_summary.json` | 2026-10-01 run A |
| B | bare / 1 / 2 bottles / off at P 32, 128 (gains at plumb); P 160 with gains written at level | `two_bottle.py`, then `two_bottle_level.py` | `…/gain_bench_2bottle/`, `two_bottle_summary.json` | run B |
| C | **the stiffness measurement**: bare + 1–4 bottles, static sample plus ±23-tick probes at 100 steps/s, P 32 / 128 / 160 | `LEVEL_T=… 2026-10-01/four_bottle.py` | `…/gain_bench_4bottle/`, `four_bottle_summary.json` | run C |
| D | rigid inertia: quiet hold, four 2° moves, 90° raise/lower at 100 steps/s, acc register 2; hanging and level | `LADDER=32:32,128:32,160:32 LEVEL_T=… DOWN_T=… 2026-10-01/hold_clamp.py` (ladders as in the record) | `…/gain_bench_hold/`, `hold_clamp_summary.json` | run D |
| 3a | the same hold on rig v3, D 32 | `LADDER=32:32,96:32,128:32,160:32 LEVEL_T=2507 DOWN_T=3531 …/hold_clamp.py` | `2026-10-02/raw/gain_bench_hold/`, `hold_v3_summary.json` | 2026-10-02 hold ladder |
| 3b | sweep roughness vs profile (A/B/C) and D | `GAINS=32:32,128:32,128:64,128:128 PROFILES=A,B,C 2026-10-02/sweep_smooth.py`, then `GAINS=128:32,128:16,128:0 PROFILES=A,C` | `…/raw/sweep_smooth/`, `sweep_summary.json` | sweep roughness |
| 3c | D 0 hold | `LADDER=128:0,160:0 …/hold_clamp.py` | `…/raw/gain_bench_hold/hold_v3_washers_D0.json` | hold with D 0 |
| 3d | accelerometer sweeps | `BNO=1 GAINS=32:32,128:32,128:0,160:0 PROFILES=A,C sweep_smooth.py` | `…/raw/sweep_bno/`, `sweep_bno_summary.json` | the accelerometer |
| 3e | command rate | `BNO=1 GAINS=96:0,96:32,128:0 PROFILES=A,C,Cf,Cs,C1` | same | command rate |
| 3f | goal changes in flight | `BNO=1 GAINS=32:32,96:0,128:0 PROFILES=A,R-,R+`, then `PROFILES=A300,A600,A1500,RL-,RL+`; analyse with `retarget_analysis.py <json>` | same | goal in flight, walking speeds |
| 3g | repeats at walking speeds | `BNO=1 GAINS=96:0,128:0 PROFILES=A300,A300,A300,A1500,A1500` | same | repeats |
| 3h | the firmware's own 50 Hz streaming | `LEVEL_T=2507 DOWN_T=3531 2026-10-02/stream50.py` (`GAINS` default 32:32,96:0,128:0) | `…/raw/stream50/`, `stream50_summary.json` | streaming at 50 Hz |

The exact parameters of every run, including the aborted starts, are in the
first line of each `console*.txt` under `raw/`.

Once wired, check the accelerometer at rest with `bno` and `bno rec 2000`. We
read |a| = 988 mg with a standard deviation of 3.5 mg (`bno_check_summary.json`).
- **Axes:** z ran along the arm, y in the rotation direction, x out of plane.
- **If nothing answers on I²C** after a reset (the BNO08x can hold the bus),
  run `imu reinit`. The scripts do this before giving up.

## Metrics, so your numbers mean the same as ours

- **Probe midpoint / band** (`four_bottle.py`): approach the target from
  23 ticks above and from 23 below. The mean of the two rest positions cancels
  gearbox friction. Half their difference is the friction band.
- **Stiffness:** a least-squares line with an intercept, deflection on torque,
  over bare and 1–4 bottles. Quote it as a ratio to P 32.
- **Holding still:** the position range in ticks over 5 s, plus how many
  samples report nonzero speed.
- **2° move:** overshoot past the final rest, settle time (every later sample
  within ±1 tick), and the residual range over the last second
  (`gain_bench.step_stats`).
- **Backward samples:** samples during a sweep whose reported speed has the
  wrong sign. The count depends on the poll rate, and it drifted between
  identical runs (see the session-3 record), so compare it only within one run.
- **Jitter:** RMS of position minus its 0.4 s moving average over the moving
  part, in ticks.
- **Acceleration RMS above 5 Hz:** an FFT high-pass of each axis at 5 Hz, then
  RMS in mg (`sweep_smooth.bno_metrics`). The peak frequency comes from a
  Hann-windowed spectrum.

Re-derive our numbers from the raw files with `summarise_raw.py` and the
`*_summary.json` files beside each session.
