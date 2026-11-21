# The power lite rail and pack sense with the ESP32: steps 1–7 and 9–11 pass (2026-11-21)

*A dated record of the README's
[bench check, with the ESP32](../../pcb/power-lite/README.md#the-bench-check-with-the-esp32)
(#218, #109): the power lite board on the lab supply with no BMS, no pack and no servos, the
J12 lead on the General Driver, J3 into its H1, and the power-lite image of `main` 825fa125
(`App version v1-hip60-backup-1462-g825fa125`). Built on the laptop, flashed with esptool 5.6.0
(hash verified), driven over the USB tether from the training box. Run by Tom at the bench; no
servo was connected, so it was not filmed (AGENTS.md). The board-only checks the day before are
in [the 2026-11-20 record](2026-11-20-power-lite-board-bench.md).*

## Result

**The rail firmware and the pack guard do what the README says, on hardware.** The ESP32
turns the rail on and off; the E-stop, an unplugged lead, a missing rail, a welded relay, a
firmware crash and a chip reset each leave the rail off and say why; the latches hold until
the README's way out. The pack guard warns, clears at 10.60 V as designed, latches SAFE,
refuses power while latched without dropping power already on, and its escape hatch works.
Not done: step 8 and step 12 (both want the console connected), and step 7's scope check.

## Readings

| step | check | expected | measured | pass |
|---|---|---|---|---|
| 1 | boot: `rail` | OFF, `SERVO_EN` low, `ESTOP_N` low, no latch, no fault | as expected; `RAIL_V` 0.81 V (see below) | yes |
| 2 | `rail on` | ON, `RAIL_V` ≈ supply | ON, 11.16 V; meter on SERVO+ (D2's band) 11.16 V: no error | yes |
| 2 | `rail off`: `RAIL_V` below 8 V | 2–3 s | about 2.0 s from 11.14 V (8.57 V at 2.0 s, 7.86 V at 2.5 s, sampled every 0.5 s) | yes |
| 2 | no fault after `rail off` | none within 30 s | none at 42 s | yes |
| 3 | J11 out, rail on | drops, `E-stop pressed`, latch, `ESTOP_N` high | as expected | yes |
| 3 | J11 back; `rail on`; `bench`, `rail on` | K1 stays open; refused naming the latch; K1 clicks | as expected | yes |
| 4 | J12's far ends pulled, rail on | K1 drops; `ESTOP_N` high; `rail on` refused | drops (LED2 off), latch, `REFUSED: rail: the E-stop is pressed (or its lead is out, or F2 blown)` | yes |
| 5 | welded K1 (a jumper from D3's band, VIN, to D2's band, SERVO+) | `STUCK RAIL` in 8–9 s; `rail on` and `rail clear` refused; clears once the rail falls | fault raised (time from the clip not taken); both refused; `rail clear` accepted at `RAIL_V` 2.37 V; `rail on` works | yes |
| 6 | F1 out, `rail on` | `SERVO_EN` drops within 250 ms, `NO RAIL` | `REFUSED: rail did not come up: NO RAIL`, `SERVO_EN` low before the reply; 250 ms not resolvable from the log | yes |
| 6 | F1 back, `rail on` | works | ON, 11.16 V | yes |
| 7 | `rail panic`, rail on | K1 drops at once, reboot, rail off | K1 clicked (by ear, not timed); `RAIL_V` 9 s later fits a drop at the crash; reboot with the rail off | yes |
| 7 | EN pressed, rail on | K1 drops at once | immediate click; `POWERON_RESET`; rail off | yes |
| 7 | scope on J12.2 through a power cycle and a reset | never high | not done (no scope) | — |
| 9 | `volt` vs a meter on J3 | INA219 near J3 | 11.18 V vs 11.16 V: 0.02 V high | yes |
| 9 | INA219 failures | not growing | 6 (all while J3 was plugged in with the supply off), then 6 through 9114 reads | yes |
| 10 | 10.4 V | WARN | `pack WARN (land it)` | yes |
| 10 | WARN clears, rising | 10.60 V | last WARN 10.58 V, first ok **10.60 V** | yes |
| 11 | 9.8 V | SAFE, latched | SAFE at 9.67 V; `rail on` and `home` refused | yes |
| 11 | back at 12.0 V; `batt reset` | still SAFE; then ok | as expected | yes |
| 11 | rail on at 12.0 V, then the supply to 9.3 V | SAFE, rail stays on | SAFE at 9.22 V, rail ON at 9.19 V | yes |
| 11 | `batt pack off`, 9.7 V | ok, `NOT guarded on`, `rail on` works | ok at 9.65 V; ON at 9.5 V; `batt pack on` → SAFE within 2 s | yes |

- **`RAIL_V` reads 0.81 V with the rail off**, not 0. The ESP32's ADC at 12 dB does not read
  below about 0.15 V at the pin, and 0.14 V × 5.7 is 0.81 V. The guard's lines are 8.0 V (on)
  and 7.0 V (lost), so the floor changes nothing. The README's step 1 now says so.
- **At 12.2 V on the supply, `RAIL_V` read 12.01 V and the INA219 12.17 V**; at 11.16 V they
  agreed with the meter to 0.02 V. Not followed up: the ADC's line fitting is least accurate
  near its top.
- **Every supply off and on latched the E-stop**, as the README says: the coil feed goes, so
  `ESTOP_N` reads stop. `bench` cleared it each time.
- **A slip of the meter probe** shorted J4 pin 2 (SERVO+) to a neighbouring pin with the rail
  on. The log shows the rail dropped by `E-stop pressed`, the supply in current limit having
  taken the logic feed down with it; the rail came back at full voltage after `bench`, so F1,
  K1 and the rail path were not hurt. Which pin it touched is not known; J8 was not connected.
  Probe SERVO+ on D2's banded lead or C1's + lead, not on J4–J7.
- **Step 5's method**: K1's 30 and 87 pads are on the back, out of reach of clip leads with
  the board on the bench; a jumper from D3's band (VIN) to D2's band (SERVO+) bypasses K1 and
  F1 from the top. The README's step 5 now uses it.

## Next

- Steps 8 and 12 with the console connected (`bimo_gui` or `bimo_tui`), and step 7's scope check
  when a scope is to hand.
- Then the README's steps 13–14 with the real servos, which need the BMS question settled
  ([2026-11-19](2026-11-19-bms-replacement-options.md)) or the lab supply on J1 with the W1–W2
  link.
