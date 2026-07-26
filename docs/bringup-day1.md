# Bring-up day 1 — board power-on + servo IDs

*2026-07-26, the day the Servo Driver with ESP32 arrived. Companion to
[wiring.md](wiring.md) §Bring-up checklist (the short form) and
[firmware-design.md](firmware-design.md) §7 (what happens after this).*

> ## ⚑ SUPERSEDED, 2026-07-26 — we flashed. Use the USB CLI.
>
> This page originally said to do all of bring-up on the stock vendor
> firmware and flash afterwards. **That advice was wrong and is withdrawn.**
> It assumed the vendor web UI was a usable tool; it is not — it byte-swaps
> every 16-bit value it exchanges with an ST3215 and corrupts angle limits
> (measured, boxed below). It also cannot be driven without rejoining the
> board's WiFi after every power cycle.
>
> Our firmware is flashed and **verified on hardware**. Bring-up is now one
> USB cable and no WiFi:
>
> ```
> .venv/bin/python -m serial.tools.miniterm /dev/cu.usbserial-0001 115200
> ```
>
> ```
> scan                     ping IDs 0-253, report position/voltage/faults
> id <old> <new>           assign an ID (EEPROM, one servo on the bus)
> pos <id>                 position, speed, load, voltage, temp, faults
> move <id> <ticks> [ms]   2048 == middle, 4096 ticks per revolution
> release [id] | torque [id]
> middle <id>              latch the current angle as 2048
> volt | stat | run | bench
> ```
>
> The stock image is backed up at `firmware/vendor-backup/` if it is ever
> needed again — see the README there for the restore command.
>
> Sections 2–3 below still describe the vendor UI. Kept for the measurements
> and the root-cause analysis, not as instructions.

Vendor facts confirmed 2026-07-26 from Waveshare's docs (sources at bottom):

|                     |                                                                          |
| ------------------- | ------------------------------------------------------------------------ |
| Power in            | **6–12.6 V DC**, 5.5 × 2.1 mm barrel jack, **5 A max through the board** |
| WiFi AP             | SSID `ESP32_DEV`, password `12345678`                                    |
| Web UI              | <http://192.168.4.1> — **use Chrome**                                    |
| USB serial          | 115200 baud                                                              |
| Power-on servo scan | IDs **0–20** (our map tops out at 10 — no `MAX_ID` edit needed)          |

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
- [ ] Confirm power in is the **DC-044 5.5 × 2.1 barrel jack (CN1) and
  
      nothing else** — the schematic shows no screw terminal, so the XT30
      pigtail needs a **barrel plug, centre positive**. The board's other
      3-pin header (H1, XH1.25) is 5 V / GND / **LED-OUT**, an output for
      addressable LEDs; don't feed the pack into it.
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

Target map — **unchanged, keep assigning to this**:

| ID  | Joint       |     | ID  | Joint       |
| --- | ----------- | --- | --- | ----------- |
| 1   | L hip roll  |     | 5   | R hip roll  |
| 2   | L hip pitch |     | 6   | R hip pitch |
| 3   | L knee      |     | 7   | R knee      |
| 4   | L ankle     |     | 8   | R ankle     |
| 9   | L hip yaw   |     | 10  | R hip yaw   |

(9/10 are the v3yaw plant's hip yaws — the current plant of record is
10-DOF. If only 8 servos are in hand, assign 1–8 and leave 9/10 for the
yaw pair.)

> **Correction 2026-07-26.** [wiring.md](wiring.md) claimed this map means
> "the policy's action vector maps to IDs 1–8 with no permutation table."
> That was true on the retired 8-DOF plant and is **false on the deployed
> 10-DOF v3yaw plant**: the sim's action order is
> `[L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle, R_hip_yaw, …]`, so
> action 0 is the servo we call bus ID **9**. **Nothing about the physical
> assignment changes** — the firmware carries a generated, tested permutation
> (`obs::kServoId = {9,1,2,3,4,10,5,6,7,8}`). Keep labelling to the table
> above. Renumbering to make the mapping an identity was considered and
> rejected: it would invalidate wiring.md, assembly.md and the connector
> guide to save a lookup table that is generated from the sim anyway.

Assign **from the top down (10 → 1)**, so you never transiently create a
second ID 1 while a servo already at ID 1 is on the bench nearby.

### Use `tools/servo_tool.py`, not the web UI

**The vendor web UI cannot do this job** — see the box below for why. Instead,
put the board into its USB↔bus bridge mode once per power cycle and drive the
servos directly:

```
# on the board's WiFi (ESP32_DEV / 12345678), once per power-up:
.venv/bin/python tools/servo_tool.py bridge on     # OLED reads SERIAL_FORWARDING

# then everything over USB — exact values, no clicking:
.venv/bin/python tools/servo_tool.py scan
.venv/bin/python tools/servo_tool.py setid 1 9
.venv/bin/python tools/servo_tool.py move 9 2048
```

For each servo:

- [ ] Board off. Connect this servo alone to bus port A. Board on.
- [ ] `servo_tool.py scan` → exactly one servo, at ID **1** (the factory ID).
- [ ] `servo_tool.py info 1` → if `angle limits` is not `0 .. 4095`, run
      `servo_tool.py fixrange 1` (the vendor UI clamps ST3215s to SC range).
- [ ] `servo_tool.py setid 1 <target>` — refuses if the target ID is already
      live, and verifies the servo answers on the new ID before reporting
      success. Written to EEPROM, survives power-off.
- [ ] `servo_tool.py move <target> 2048` then `move <target> 1400` — the DOA
      test, at the correct 0–4095 full scale.
- [ ] **Label the case** with the ID *and* the joint name, in marker, on the
      flat face you'll still be able to read after assembly.
- [ ] Board off. Next servo.

> ### 🔴 The vendor firmware talks to ST3215s with the bytes reversed
>
> **Measured on the bench 2026-07-26, not inferred.** The board runs the
> **SC-series** build. `SCSCL` sets `End = 1` (big-endian); `SMS_STS` sets
> `End = 0` (little-endian). The ST3215 is ST series — so **every 16-bit
> value the board exchanges with these servos has its two bytes swapped.**
>
> Confirmed two independent ways on servo ID 9:
>
> | | board (big-endian) | read correctly (little-endian) |
> |---|---|---|
> | present position | 772 (`03 04`) | **1027** (`04 03`) |
> | angle limits after "Set Servo Mode" | wrote 20, 1003 | **5120, 60163** |
>
> `20 = 0x0014` sent big-endian arrives as `0x1400` = 5120; `1003 = 0x03EB`
> arrives as `0xEB03` = 60163. Both predicted before reading, both exact.
>
> **What follows from this:**
>
> - ✅ **8-bit registers are safe through the web UI** — byte order cannot
>   corrupt a single byte. That means **ID** (reg 5), **mode** (33) and
>   **torque** (40). `http-setid` is therefore trustworthy, and it needs no
>   USB cable.
> - ❌ **Every 16-bit value is garbage** in both directions: position, speed,
>   and angle limits. Do not trust a position the UI reports, and do not use
>   its Position± / Middle / Set Servo Mode buttons on an ST3215.
> - ⚠️ **"Set Servo Mode" corrupts the angle limits.** Any servo that has met
>   that button is clamped by junk values until `fixrange` repairs it. Check
>   every servo with `info` before it goes into a leg.
>
> The servo itself is fine — none of this damages hardware, and ID 9 tracked
> 600 / 2048 / 3500 to within ±3 counts after repair.

> ### ⚠️ Secondary: the UI's other quirks (from its source)
>
> Read out of Waveshare's own
> [firmware source](https://github.com/waveshare/Servo-Driver-with-ESP32):
>
> 1. **It is built for the wrong servo family.** `STSCTRL.h:15` has
>    `SERVO_TYPE_SELECT = 2` — **SC series**, `ServoDigitalRange = 1023`. The
>    ST3215 is **ST series, range 4095**. The commented-out lines right below
>    are the ST settings the vendor didn't enable. So "Position+" commands
>    ~1003 counts on a servo that has 4095 — about a quarter of travel, which
>    is exactly the "doesn't do much" symptom.
> 2. **The ID field moves ±1 per click** (`servotoSet += 1`, wrapping at 250),
>    so reaching 9 means nine clicks with no overshoot — and one overshoot
>    costs 240 more clicks to wrap around.
> 3. **`setMode(id, 0)` in SC mode writes SC angle limits (20…1003) to
>    registers 9/11** — which the ST series uses for the *same* purpose. Any
>    ST3215 that has been through the UI's mode button is now clamped to ~24 %
>    of its travel. `servo_tool.py fixrange <id>` puts registers 9/11 back to
>    0…4095 and mode to 0. **Check every servo you touched with the UI.**
>
> The tool sidesteps all three by using the vendor firmware's own
> `SERIAL_FORWARDING` mode (`BOARD_DEV.h:157`), a byte-for-byte USB↔bus
> bridge, and speaking the real Feetech ST protocol from the laptop. Its
> packet encoder is checked against the worked examples in Feetech's protocol
> manual — the same golden vectors the firmware's C++ bus layer passes.
> **Nothing is flashed; the vendor firmware is untouched.**

Keep a tally here as you go — DOAs and surprises:

| ID  | Joint       | Assigned | Moves | Notes |
| --- | ----------- | -------- | ----- | ----- |
| 1   | L hip roll  | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 27 °C — factory ID, no reassignment needed, err 0x00. |
| 2   | L hip pitch | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 28 °C, err 0x00. |
| 3   | L knee      | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 28 °C, err 0x00. |
| 4   | L ankle     | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 27 °C, err 0x00. |
| 5   | R hip roll  | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 28 °C, err 0x00. |
| 6   | R hip pitch | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 29 °C, err 0x00. |
| 7   | R knee      | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 26 °C, err 0x00. |
| 8   | R ankle     | ✅        | ✅     | 2026-07-26, USB CLI. 200→3900 clean, ±3 ticks, 30 °C, err 0x00. |
| 9   | L hip yaw   | ✅        | ✅     | 2026-07-26: set from factory ID 1 over WiFi. Angle limits were corrupted (5120..60163) by the UI's Set Servo Mode; `fixrange` restored 0..4095. Tracks 600/2048/3500 to ±3 counts, 12.0 V. |
| 10  | R hip yaw   | ✅        | ✅     | 2026-07-26. First web-UI attempt did not stick (came back as ID 1); reassigned over the USB CLI and **confirmed across a servo power cycle**. 200→3900 clean, ±3 ticks. |

## 3b. Full-chain result, 2026-07-26 — 10 of 10 ✅

All ten servos on one bus, via the flashed firmware's `ping` (no argument),
which checks exactly the IDs the policy expects, **in action order**:

```
L_hip_yaw    id  9  ok      R_hip_yaw    id 10  ok
L_hip_roll   id  1  ok      R_hip_roll   id  5  ok
L_hip_pitch  id  2  ok      R_hip_pitch  id  6  ok
L_knee       id  3  ok      R_knee       id  7  ok
L_ankle      id  4  ok      R_ankle      id  8  ok
10 of 10 present            bus 12.0 V
```

That ordering is the first hardware confirmation of
`obs::kServoId = {9,1,2,3,4,10,5,6,7,8}` — the permutation wiring.md wrongly
called an identity. Bus voltage fell only 12.0 → 11.9 V worst-case with all
ten powered, consistent with the ~0.85 A mean in
[wiring.md § Power path](wiring.md).

### The duplicate-ID signature, worth recognising

An earlier run of this same test read **8 of 10**, with ID 9 missing and ID 1
answering intermittently (3/10 pings), returning `bad-reply` and a nonsense
`0.0 V`. That was **not** a loose connector — it was **two servos both
answering as ID 1**, because servo 9's assignment (made through the vendor
web UI) had silently reverted.

Mechanism, from the vendor library: **SCSCL locks EEPROM at register 48, the
ST series at 55.** The SC-build firmware's "unlock" therefore never unlocked
anything on an ST3215, so ID writes through it did not persist — and it left
a stray value in reg 48, the torque limit. Our firmware uses 55, and all ten
assignments made through it have held.

**If a chained servo is intermittent with garbage reads, suspect a duplicate
ID before suspecting the wiring.**

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

- [x] **Powered friction, measured electronically 2026-07-26.** Commanding a
      steady 200 steps/s sweep on an unloaded servo and reading the load
      register (reg 60) gives a tight, repeatable figure across four units:

      | id | n | speed | median \|load\| | p10 | p90 |
      |---|---|---|---|---|---|
      | 1 | 21 | 200 | 76 | 72 | 80 |
      | 3 | 21 | 200 | 80 | 72 | 80 |
      | 6 | 21 | 200 | 80 | 76 | 88 |
      | 8 | 28 | 200 | 80 | 72 | 80 |

      Load is 0–1000 = 0–100 % of max torque, so ~80 → **8.0 % of the 2.94 N·m
      stall ≈ 0.235 N·m**. Reproduce with `move <id> 3600 0 200` while polling
      `pos <id>`; a `move` without an explicit steps/s slews at ~3000 steps/s
      and measures acceleration transients instead.

- [ ] **Unpowered backdrive friction — still needs a spring scale.** The
      number above is **powered** friction (motor driving *through* the
      gearbox). The sim's `off_frictionloss` models the joint with torque
      **off**, being driven backwards, and back-driving a ~1:345 reduction is
      far less efficient than forward-driving it — so 0.235 N·m is a **lower
      bound**, not the answer. It is close enough to the threshold to matter:
      [firmware-design.md](firmware-design.md) §5 estimates 0.35 N·m and idle
      torque-off **flips infeasible below ~0.25 N·m**.

      The five-minute bench test, with the arithmetic pre-done:
      1. `release <id>` so the joint is unpowered.
      2. Bolt a horn with a rod giving a **50 mm** lever from the shaft axis.
      3. Pull perpendicular with a spring scale until it rotates *steadily*
         (breakaway, not a jerk). Read the force.
      4. Torque = F × 0.05 m.

      | scale reads | joint torque | verdict |
      |---|---|---|
      | < 510 gf (5.0 N) | < 0.25 N·m | **idle torque-off is not feasible** |
      | ~714 gf (7.0 N) | 0.35 N·m | matches the sim's estimate |

      Do it while the servos are loose — it is far harder once they are in legs.
- [ ] **Servo case thread**: M3 self-tapping, tapped M3, or M4? Vendor STEP
  
      shows Ø3.5. Open question 1 in [bom-sourced.md](bom-sourced.md); it
      decides the horn/bracket screws.
- [x] While you're at it, confirm the connector ports really are on the
  
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

## Resolved: the 5 A rating vs. our 10 A budget — no action needed

Both halves turned out to be wrong in our favour. Detail in
[wiring.md § Power path](wiring.md); the short version:

- **The board's servo V+ is a bare passthrough.** Waveshare's schematic puts
  CN1 (the barrel jack) and both servo headers on the same `6-12V` net, with
  **no fuse, sense resistor, e-fuse or protection FET** between them. So the
  5 A is the jack's and the copper's thermal rating — there is nothing that
  can trip, and no current sensing on the board at all.
- **Our 10 A was a hand estimate and it was high.** `sim/current_budget.py`
  now computes it from the env's stall-calibrated electrical model:
  **6.8 A peak, 5.0 A p99, 1.4 A RMS, 0.85 A mean** across the whole gait
  gauntlet. RMS is what heats copper, and 1.4 A is ~3.5× under the rating.

The one real constraint left is the **daisy chain**: the first lead in each
leg carries that whole leg — 4.0 A peak, ~0.7 A RMS through one 3-pin
contact. Fine on RMS, over a 5264 contact's ~3 A on peak. It's why we run
one leg per board port rather than all 8–10 off a single port.

Two follow-ups, neither blocking today:

- [ ] Add a low-ESR **470–1000 µF** cap across servo V+/GND at the board.
  
      The `6-12V` net has only 10 µF + 0.1 µF of local bulk; a 6.8 A step
      currently gets sourced all the way through the pack leads and jack.
- [ ] **Bench-verify the peak** with an inline shunt or clamp meter during
  
      the first walks. The table above is sim-derived, not measured.

## Confirmed from the schematic (2026-07-26)

Worth having on hand — and now feeding the firmware build:

|                       |                                                                                                                                                        |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Servo bus UART        | **U1TXD = IO19, U1RXD = IO18** (matches wiring.md's "GPIO 18/19")                                                                                      |
| Half-duplex direction | `TXEN` gates **U3 SN74LVC1G126** + **U4 SN74LVC1G125**, driven via a PNP (Q1) off U1TXD — the direction circuitry is on the board, we just assert TXEN |
| I2C                   | **SDA = IO21, SCL = IO22** (matches wiring.md)                                                                                                         |
| OLED                  | SSD1306, **0.91″ 128 × 32**                                                                                                                            |
| Status LEDs           | 2× **WS2812B** on-board (L1, L2), plus an external WS2812 output on H1                                                                                 |
| USB                   | **CP2102** (needs the CP210x driver on macOS), auto-program circuit via DTR/RTS                                                                        |
| Logic rails           | U5 buck → 5 V → AMS1117-3.3 → 3V3                                                                                                                      |
| Bus voltage sense     | R18 560 K / R19 4.7 K divider → ADC. **Voltage only, no current sense.**                                                                               |
| Power rails           | **Two, and only one reaches the servos** — see below.                                                                                                  |

### The board boots on USB alone. The servos do not.

```
USB VBUS ──|D3|──┐
                 ├── 5 V ── AMS1117 ── 3V3    ESP32, OLED, WS2812, web UI
CN1 6-12V ─U5buck─|D1|──┘

CN1 6-12V ─────────────────────────────────── H2/H3 pin 2 = servo bus V+
```

Logic 5 V is **diode-OR'd** between USB VBUS and the buck output, so a USB
cable alone lights the ESP32, the OLED and the web UI — everything *looks*
alive while the bus scan finds nothing and every servo appears dead. Servo V+
sits on the raw `6-12V` net from CN1 alone, and nothing feeds that net from
USB (the diodes point *into* the 5 V rail, not out of it).

**So the barrel jack is required whenever servos are involved.** Both supplies
connected at once is fine — the diode-OR prevents back-feed. Confirm a live
bus with the `voltage` line of `servo_tool.py info`: that figure is read out of
the servo, so it proves the V+ rail rather than just the logic.

## Sources

- [Servo Driver with ESP32 — product usage](https://docs.waveshare.com/Servo_Driver_with_ESP32/Product-Use)
- [Servo Driver with ESP32 — wiki](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32)
- [ST3215 Servo user manual (PDF)](https://files.waveshare.com/upload/f/f4/ST3215_Servo_User_Manual.pdf)
- [Servo Driver with ESP32 schematic (PDF)](https://files.waveshare.com/wiki/Servo-Driver-with-ESP32/Servo_Driver_with_ESP32.pdf) — the source for the power-path and pinout findings above
