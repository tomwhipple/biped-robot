# The power lite board on the lab supply: Part 2 passes (2026-11-20)

*A dated record of Part 2 of [`pcb/power-lite/bringup.md`](../../pcb/power-lite/bringup.md)
(README step 2): the assembled board with no BMS and no pack, run by Tom with one lab supply
and `SERVO_EN` driven by hand from the breadboard rig. Part 1 (the BMS) is parked: the board
received is not DS-NA405's and has no under-voltage cut-off
([2026-11-19 record](2026-11-19-power-lite-bms-bench.md),
[replacement options](2026-11-19-bms-replacement-options.md)). No servo was connected and no
agent moved one, so the session was not filmed (AGENTS.md).*

## Result

**The board works as designed.** The E-stop and `SERVO_EN` each open the relay at once and
neither latches; `ESTOP_N` reads RUN low and STOP high; `RAIL_V` tracks the rail within the
divider's tolerance; K1 closes and holds at 9.0 V on less base drive than an ESP32 pin gives;
a missing F2 reads as "stop" and a missing F1 leaves no rail. The two faults found were in
the J12 test harness and the bench rig, not on the board.

## Readings

The rig is bringup.md 2.1: supply 12.0 V, limit 0.5 A, + to W7 and − to W2; the breadboard's
+ rail from the supply's + terminal; J12's far ends unplugged from the General Driver and
brought to the breadboard (`ESTOP_N` through 47 kΩ to the + rail, `SERVO_EN` to the drive row
of three 470 Ω in series). Voltages are to W2.

| step | reading | expected | measured | pass |
|---|---|---|---|---|
| 2.2 | supply current | ≈ 2 mA | 0 on the supply's display (it does not resolve a few mA; nothing near 50 mA) | yes |
| 2.2 | LED1 / LED2, K1 | on / off | on / off (K1 open: LED2 off, `RAIL_V` 0) | yes |
| 2.2 | J3 pin 1 | 12.0 V | 12 V | yes |
| 2.2 | `ESTOP_N`, J11 open | 12.0 V | 11.9 V | yes |
| 2.2 | `RAIL_V` | 0.00 V | 0 V | yes |
| 2.3 | `ESTOP_N`, J11 in | < 0.1 V (sim 0.009 V) | first 11.93 V; **0.025 V** after the harness fix below | yes |
| 2.4 | K1, LED2, `RAIL_V`, J4–J7 pin 2, supply current, Q1 saturation, `SERVO_EN` current | click, on, 2.11 V, 12.0 V, ≈ 0.17 A, 0.17–0.23 V, ≈ 5.9 mA | as expected (Tom; values not recorded) | yes |
| 2.5 | `SERVO_EN` free / grounded | K1 drops at once; LED2 fades | drops the moment `SERVO_EN` leaves the drive row; stays open grounded; LED2 fades over seconds | yes |
| 2.6 | J11 out, rail on | K1 drops, `ESTOP_N` 12 V, `RAIL_V` fades; J11 back in re-closes K1 | as expected (Tom) | yes |
| 2.7 | `RAIL_V` at 12.6 / 9.0 V | 2.21 / 1.58 V | 2.18 / 1.55 V | yes |
| 2.7 | K1 at 9.0 V: holds; re-closes after `SERVO_EN` off and on | yes | yes | yes |
| 2.8 | F2 out: LED1, J3, `ESTOP_N`, K1 | off, 0 V, 12.0 V, open | off, 0 V, 12 V, open (after the rig fix below) | yes |
| 2.8 | F1 out: K1, LED2, `RAIL_V` | clicks, off, 0 V | as expected (Tom) | yes |

- **`RAIL_V` reads 1.4–1.9 % low.** The 47k/10k divider of 1 % resistors can be 1.6 % low at
  worst, and the supply's set point has its own error; the firmware's stuck-rail threshold
  (8 V on the rail) has far more margin than that.
- **2.3 first failed at the harness, not the board.** With J11 in, the board's own nodes read
  as designed (D1's banded end, `COIL_HI`, 12 V; Q2's base about 0.65 V; Q2's collector
  below 0.1 V), while the harness's `ESTOP_N` wire read 11.93 V. A continuity check of the
  harness found the **`RAIL_V` wire open**; Tom fixed it, and `ESTOP_N` then read 0.025 V.
  What held the `ESTOP_N` wire high before the fix was not identified.
- **2.8's first F2-out reading was 0 V** at `ESTOP_N`: a breadboard lead had come off, so the
  pull-up had no supply. Reconnected, it read 12 V.
- **An open J12 wire reads like a good one in 2.2** (`ESTOP_N` sits at the pull-up, `RAIL_V`
  at 0 either way). The harness's continuity to J12's pads is worth checking before 2.2.

## Next

- **The bench check with the ESP32** ([README](../../pcb/power-lite/README.md#the-bench-check-with-the-esp32)):
  the rail firmware is on `main` behind the power-lite build option (PR #229), and it needs
  no BMS. J12 goes back on the General Driver, the breadboard rig comes off.
- Parts 1, 3 and 4 wait for a BMS that cuts off on under-voltage.
