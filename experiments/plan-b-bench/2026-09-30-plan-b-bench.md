# Plan B bench, session 1 (2026-09-30): the bench servo, unloaded

Issue #73, [DESIGN.md §13](../../DESIGN.md) step 0. This session read the
bench servo and ran every rung of the P ladder on a **bare horn**: no lever, no
masses, no inertia. It could therefore rule out a servo that buzzes unloaded,
but it could not measure stiffness. The stiffness table and the stance number
(steps 0 and 1 as written) are still to be taken.

Tool: `gain_bench.py` in this folder (then `tools/gain_bench.py`), at main eef3abe. The session was run by the Claude
session on Mira as `claw`, on Tom's instructions. The raw data (JSON per step
and `session.log`) is gitignored; it is on Mira under
`hw_sessions/2026-09-30/gain_bench/` (the reads) and
`hw_sessions/2026-09-30/gain_bench_bare/` (the pass).

## The rig

- **Board**: the prototype's old controller (ESP32-D0WD-V3 rev 3.1) on Mira's
  tether. It was flashed with main 98c9ced, because the build it had predated
  `gains` (#94). The flash kept NVS, and with it the prototype's
  calibration, in which ID 11 is `NOT FITTED`.
- **Servo**: one STS3215 (Model_Number 777, servo firmware 3.10), alone on
  the bus, nothing on the horn.
  - It arrived as ID 1 and was re-IDed to **11**. On the robot it will be
    `R_ankle_roll`, one of the six Plan B servos.
  - ID 11 is on the bus map, so `move` is clamped to `R_ankle_roll`'s
    envelope, which on this board (zero 2048, dir +1, ±25°) is **1764..2332**.
  - The horn was found at 4094, on the encoder wrap, and was moved to 2050
    before the pass.

## Registers as found

| register | value |
|---|---|
| P / D / I (21/22/23) | 32 / 32 / 0 |
| dead zone CW / CCW (26/27) | 1 / 1 |
| minimum start force (24) | 16 |
| protection current (28) | 310 × 6.5 mA = 2.0 A |
| protective torque (34) / protection time (35) / overload torque (36) | 20 % / 2 s / 80 % |
| over-current time (38) | 2 s |
| speed-loop P / I (37 / 39) | 10 / 200 |
| max torque (16) / torque limit (48) | 1000 / 1000 |
| status (65) | 0 |

## The unloaded ladder

`hold 11 --allow-bus-id --orient down --inertia "none (bare horn)"`. At each
rung: the gains were written, the servo held for 5 s (sampled at 30 Hz), then
took a 23-tick (2°) step out and back at full speed.

| P | D | hold range (ticks) | moving samples | step out: overshoot / settle | step back: overshoot / settle | steady offset out / back (ticks) | status | quiet |
|---|---|---|---|---|---|---|---|---|
| 32 | 32 | 0 | 0/152 | 0 / 0.17 s | 0 / 0.17 s | −2 / +2 | – | yes |
| 64 | 64 | 0 | 0/152 | 0 / 0.13 s | 0 / 0.13 s | −1 / +1 | – | yes |
| 96 | 96 | 0 | 0/152 | 1 / 0.13 s | 0 / 0.17 s | −1 / 0 | – | yes |
| 128 | 128 | 0 | 0/152 | 1 / 0.13 s | 0 / 0.13 s | −1 / 0 | – | yes |
| 160 | 160 | 0 | 0/152 | 0 / 0.13 s | 0 / 0.17 s | +1 / 0 | – | yes |
| 128 | 32 | 0 | 0/152 | 0 / 0.13 s | 0 / 0.13 s | −1 / 0 | – | yes |
| 160 | 32 | 0 | 0/152 | 0 / 0.13 s | 0 / 0.13 s | 0 / 0 | – | yes |

Afterwards: P/D back at 32/32, EEPROM locked (register 55 = 1), status 0,
32 °C.

**What this shows.** Unloaded, the servo neither buzzes nor limit-cycles at any P
up to 160, with D raised alongside P or left at 32. No protection bit was set
at any rung. The steady error after a step falls from 2 ticks at P 32 to 0–1
above it, the direction a stiffer loop predicts, but the numbers are single
encoder ticks against a 1-step dead zone.

**What it does not show.** A bare horn has almost no inertia and no preload,
which are the conditions under which gear play limit-cycles, so "quiet" here
is weak evidence. It says nothing about stiffness.

## Found on the way

- **Gain writes lose their ack** (#101). A `gains` write reported
  `FAILED (write-failed)` although the servo held the new P/D with the EEPROM
  locked.
  - Over this session's eight writes, the acks were lost in strict
    alternation: ok, lost, ok, lost, and so on. An earlier hand sequence went
    lost, ok, ok.
  - The likely cause is the bus's 3 ms reply timeout against an EEPROM cell
    write that takes longer. The alternation is consistent with the servo's
    flash-emulated EEPROM needing a slower commit on every other write; that
    is not verified.
  - `gain_bench` settles, reads back P/D and register 55, and accepts only
    "equal and locked". Every write in the pass was verified that way.
- **`id` can rename and still report `timeout`** (#101). `id 11 30` took
  effect, but its failure path re-locked under the old ID, so the servo was
  left with its EEPROM unlocked until the next `gains` write locked it.
- **gain_bench fixes found here**, all on main:
  - the gains-write read stopped on the firmware's progress line
    ("…torque must be OFF…") instead of its verdict;
  - a bus-map ID is now taken only with `--allow-bus-id` and an envelope
    check against the board's calibration;
  - hold goals near the 0/4095 wrap are refused;
  - `park` added.

## Next

1. **Step 0 on a lever.** Horizontal lever on the horn, with 0.5 / 1.0 /
   1.5 N·m hung (0.51 / 1.02 / 1.53 kg at 0.10 m):
   `stiffness 11 --allow-bus-id`. Then the 0.25 kg / 0.10 m inertia hanging
   down and horizontal: `hold 11 --allow-bus-id`.
2. **Step 1.** `stance`, on the prototype's stance hip roll.
3. Then P/D into `firmware/main/servo_gains.h` and servo-map.md §4, in one commit.
