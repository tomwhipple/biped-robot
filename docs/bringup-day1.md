# Bring-up day 1 — board power-on + servo IDs

*2026-07-26, the day the Servo Driver with ESP32 arrived. Companion to
[wiring.md](wiring.md) §Bring-up checklist (the short form) and
[firmware-design.md](firmware-design.md) §7 (what happens after this).*

**Do this whole page on the STOCK VENDOR FIRMWARE, before flashing anything
of ours.** The board ships with a demo firmware whose web UI already does ID
assignment, middle-position calibration and torque release. Flashing our
firmware overwrites it. Our firmware's own bring-up CLI is being written now
but is not the tool for today — get 8–10 labelled servos out of the box
first, then flash.

Vendor facts confirmed 2026-07-26 from Waveshare's docs (sources at bottom):

| | |
|---|---|
| Power in | **6–12.6 V DC**, 5.5 × 2.1 mm barrel jack, **5 A max through the board** |
| WiFi AP | SSID `ESP32_DEV`, password `12345678` |
| Web UI | <http://192.168.4.1> — **use Chrome** |
| USB serial | 115200 baud |
| Power-on servo scan | IDs **0–20** (our map tops out at 10 — no `MAX_ID` edit needed) |

---

## 0. Before power — verify the part (10 min, unblocks a print)

- [ ] Confirm the silkscreen actually reads **"Servo Driver with ESP32"**,
      65 × 30 mm. If it says **"Bus Servo Driver HAT (A)"** (65 × 57 mm), it
      is the wrong board — 9–25 V in, Pi form factor, won't fit the tower.
      See [hardware-order.md](hardware-order.md) caveat 2.
- [ ] **Caliper the 4 mounting holes**: hole Ø and the two centre-to-centre
      spans. CAD currently assumes **Ø2.75 on 58 × 23 mm** (`BOARD_HOLES` in
      `cad/dimensions.py`), which is a wiki figure, not a measurement. This
      is open question 3 in [hardware-order.md](hardware-order.md) and it
      **gates printing the tower** — measure it now, not after.
- [ ] Note whether power is barrel-jack only or barrel jack **+** screw
      terminal. [wiring.md](wiring.md) assumes a screw terminal is available
      for the XT30 pigtail; if it's jack-only we need a jack pigtail instead.
- [ ] Caliper the servo bus connector pitch and confirm the pin order on the
      board matches the servo leads: **1 GND (black) · 2 V+ (red) · 3 DATA**.

## 1. First power, bare board (no servos)

Bench supply strongly preferred over the LiPo for first light: set
**11.1 V, current limit ~0.5 A**. A short shows up as a limit trip instead
of smoke.

- [ ] Power up. OLED lights, shows something (voltage / status).
- [ ] Current draw is sane (tens of mA, not amps).
- [ ] Laptop → WiFi → `ESP32_DEV` / `12345678`.
- [ ] Chrome → <http://192.168.4.1>. UI loads.
- [ ] Read bus voltage on the OLED and in the UI; sanity-check it against
      what the bench supply says. That reading is what the 10 Hz telemetry
      will report later, so a constant offset is worth knowing now.

Power down before touching the servo bus.

## 2. Servo IDs, one servo at a time

**Every servo ships as ID 1.** Two servos with the same ID on the bus do not
enumerate — you get silence, not an error. So: exactly one servo connected
at a time, and **power off the board between swaps** (don't hot-plug the bus).

Raise the bench limit to **~2 A** for this step (one servo, unloaded, idles
~0.18 A and moves well under 1 A).

Target map — matches the sim's joint order in `sim/walker_env.py`, so the
policy's action vector maps straight onto IDs with no permutation table:

| ID | Joint | | ID | Joint |
|---|---|---|---|---|
| 1 | L hip roll | | 5 | R hip roll |
| 2 | L hip pitch | | 6 | R hip pitch |
| 3 | L knee | | 7 | R knee |
| 4 | L ankle | | 8 | R ankle |
| 9 | L hip yaw | | 10 | R hip yaw |

(9/10 are the v3yaw plant's hip yaws — the current plant of record is
10-DOF. If only 8 servos are in hand, assign 1–8 and leave 9/10 for the
yaw pair.)

Assign **from the top down (10 → 1)**, so you never transiently create a
second ID 1 while a servo already at ID 1 is on the bench nearby.

For each servo:

- [ ] Board off. Connect this servo alone to bus port A.
- [ ] Board on. Web UI: `ID Select +/-` until the active ID is **1** (the
      factory ID) and the UI reports the servo responding.
- [ ] `ID to Set +/-` until it reads the target ID.
- [ ] **`Set New ID`**. The change is written to servo EEPROM and survives
      power-off.
- [ ] Verify: `ID Select` to the new ID → servo responds. `ID Select` to 1 →
      nothing.
- [ ] Nudge it with the position control and watch it move. This is also
      your free DOA test — do it now, not during assembly.
- [ ] **Label the case** with the ID *and* the joint name, in marker, on the
      flat face you'll still be able to read after assembly.
- [ ] Board off. Next servo.

Keep a tally here as you go — DOAs and surprises:

| ID | Joint | Assigned | Moves | Notes |
|---|---|---|---|---|
| 1 | L hip roll | ☐ | ☐ | |
| 2 | L hip pitch | ☐ | ☐ | |
| 3 | L knee | ☐ | ☐ | |
| 4 | L ankle | ☐ | ☐ | |
| 5 | R hip roll | ☐ | ☐ | |
| 6 | R hip pitch | ☐ | ☐ | |
| 7 | R knee | ☐ | ☐ | |
| 8 | R ankle | ☐ | ☐ | |
| 9 | L hip yaw | ☐ | ☐ | |
| 10 | R hip yaw | ☐ | ☐ | |

## 3. Chain test, one leg at a time

- [ ] Board off. Chain IDs 1→2→3→4 off port A (each ST3215 has two
      internally-paralleled ports, so it's case-to-case hops).
- [ ] Board on, limit ~3 A. Confirm the scan enumerates **exactly 4** servos
      at the expected IDs.
- [ ] Repeat for 5→6→7→8 on port B. Then both legs together: **8 enumerate.**
- [ ] Then add 9/10 if present: **10 enumerate.**

Missing servo ⇒ suspect the lead or a duplicate ID, in that order.

## 4. Two measurements to take while the servos are loose

Both feed sim decisions and are far easier now than after assembly.

- [ ] **Unpowered backdrive friction.** Release torque and measure the torque
      needed to backdrive a joint (a spring scale at a known lever arm is
      enough). [firmware-design.md](firmware-design.md) §5 "idle torque-off"
      estimates 0.35 N·m from the gear class, and the feature's feasibility
      **flips off below ~0.25 N·m** — so this number decides whether idle
      torque-off ships at all.
- [ ] **Servo case thread**: M3 self-tapping, tapped M3, or M4? Vendor STEP
      shows Ø3.5. Open question 1 in [bom-sourced.md](bom-sourced.md); it
      decides the horn/bracket screws.
- [ ] While you're at it, confirm the connector ports really are on the
      **idler-side face beside the disc** (`SV_CONN`, corrected 2026-07-24
      from the vendor STEP) — the yaw carrier's wire openings are cut for
      that geometry.

## 5. Not today

- **"Set Middle Position"** — do *not* run it now. It defines the servo's
  2047 centre at wherever the horn currently sits. It belongs at the
  CAD-neutral pose during assembly ([wiring.md](wiring.md) checklist step 4),
  not on a naked servo.
- Flashing our firmware. After IDs are set and labelled.

---

## ⚠️ Finding: the board's 5 A ceiling vs. our 10 A budget

Waveshare rates this board at **5 A max**, and warns that with many servos
you should "power them in separate groups." [wiring.md](wiring.md) budgets
**~10 A transient** (2–3 joints near stall simultaneously in the gait
gauntlet's worst cases), sized on XT30 + 20 AWG.

Nothing in this page's steps goes near that — one servo at a time draws
under an amp. But before the first powered stand we need to resolve it:
either the servo-bus V+ rail is a passthrough that isn't actually gated by
the 5 A figure (plausible — the figure may be the barrel jack / onboard
regulator), or the bus V+ needs to be fed from the pack directly and the
board only fed logic power. **Verify against the board's schematic before
the first floor test**, and update wiring.md's circuit diagram either way.

## Sources

- [Servo Driver with ESP32 — product usage](https://docs.waveshare.com/Servo_Driver_with_ESP32/Product-Use)
- [Servo Driver with ESP32 — wiki](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32)
- [ST3215 Servo user manual (PDF)](https://files.waveshare.com/upload/f/f4/ST3215_Servo_User_Manual.pdf)
