# Wiring & control architecture

*Status 2026-07-11. One serial bus, one battery, no custom electronics —
the entire harness is the servo leads that ship in the box plus one XT30
pigtail and a switch.*

**Circuit diagram** (pin-level: connectors, nets, wire colors):

![Circuit diagram](circuit-diagram.svg)

**System block diagram** (physical layout view of the same thing):

![Wiring diagram](wiring-diagram.svg)

**Connector field guide** (photos, pinouts, counts, measured hop lengths):
[connector-guide.html](connector-guide.html) — self-contained, open in a browser.

## Power path

```
3S LiPo (XT30) ── inline switch ── CN1, DC-044 5.5×2.1 barrel jack
                                        │   (the board's ONLY power input)
                                        ├── U5 buck ── 5 V ── AMS1117 ── 3V3 logic
                                        └── H2 / H3 pin 2 = servo bus V+
                                            (same `6-12V` net, straight through)
```

**Pigtail correction, 2026-07-26:** this doc previously said "screw terminal /
DC 5.5×2.1". The schematic shows **no screw terminal** — CN1 is the only power
input. The XT30 pigtail must therefore terminate in a **5.5 × 2.1 mm barrel
plug** (centre positive: CN1 pin 4 = VCC, pins 2/3 = GND). The board's other
3-pin header, H1 (XH1.25), is **5 V / GND / LED-OUT** for addressable LEDs —
it is an output, not an alternative power inlet. Do not feed the pack into it.

- The battery is a 3S 850 mAh XT30 pack — BOM pick
  [Tattu 75C](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30) (decided
  2026-07-11, re-sourced 2026-07-15; buy two, so one charges while one flies).
  Any pack passing the `bom-by-vendor.md` spec filter works — **11.1 V, not
  11.4 V HV**, which would exceed the servos' 12.6 V ceiling. It swaps
  tool-free through the window in the rear tower wall: peel the belt, tug the
  pull-ribbon, tilt the pack out; reverse to insert (lead-end toward whichever
  y-pocket the XT30 pigtail lives in).
- The **Servo Driver with ESP32** accepts 6–12.6 V, so it runs directly from
  2S (7.4 V) or 3S (11.1 V, 12.6 V fully charged) — no regulator needed. Its
  logic is powered from an onboard buck; the servo bus carries raw battery
  voltage, which is what sets torque (the whole 2S/3S performance story in
  [hardware-order.md](hardware-order.md)).
- **Current sizing** (revised 2026-07-26 — the old "~10 A transient" was a
  hand estimate of "2–3 joints near stall"; it is now computed). The env's
  electrical model is calibrated to the ST3215 stall point (`_K_CU` =
  3.75 W/(N·m)², i.e. 2.94 N·m → ~32 W → 2.9 A at 11.1 V), so bus current is
  just watts over volts. `sim/current_budget.py` runs the referee's scenarios
  and reports the distribution — `loco_v5t`, 10 joints, 11.1 V, hardware-claim
  DR, 3 eps × 6 scenarios:

  | | peak | p99 | **RMS** | mean |
  |---|---|---|---|---|
  | whole bus | 6.8 A | 5.0 A | **1.4 A** | 0.85 A |
  | worst single leg | 4.0 A | — | ~0.7 A | — |
  | worst single joint | 3.0 A | — | — | — |

  **RMS is the number that sizes copper and contacts**; the 6.8 A peaks are
  millisecond gait transients that heat nothing. So: the real peak is ~1.5×
  *lower* than the old estimate, and sustained draw is ~7× lower again.
  XT30 (30 A) and 20 AWG are hugely comfortable; JST-terminated packs (~3 A)
  stay ruled out on peak. Regenerate with
  `.venv/bin/python sim/current_budget.py --run loco_v5t`.
- **Does 6.8 A peak hurt the 5 A-rated driver board? No — the servo V+ is a
  bare passthrough.** Confirmed from Waveshare's
  [schematic](https://files.waveshare.com/wiki/Servo-Driver-with-ESP32/Servo_Driver_with_ESP32.pdf)
  (read 2026-07-26): CN1 (DC-044 5.5×2.1) VCC and both servo headers H2/H3
  pin 2 sit on the same `6-12V` net. **No fuse, no polyfuse, no e-fuse, no
  sense resistor, no INA219, no protection FET anywhere in that path** — the
  only other loads on the net are the U5 buck's Vin (logic) and C17/C22/C23.
  So there is nothing to trip: the 5 A is the barrel jack's and the copper's
  thermal rating, and 1.4 A RMS sits ~3.5× under it. (The board's voltage
  readout is an R18 560 K / R19 4.7 K divider into an ADC — **voltage only,
  no current sensing on this board at all**, which is why telemetry reports
  millivolts and not amps.)
- **The tighter constraint is the daisy chain, and it is not the board's
  fault.** Each 3-pin lead carries the current of every servo downstream of
  it, so the first lead in a leg sees the whole leg: **4.0 A peak, ~0.7 A
  RMS**. Molex-5264-class contacts are ~3 A; the peak is over that, the RMS
  is well under. Fine as built, but it is why the two board ports are used
  one-per-leg (halving both) rather than chaining all 8–10 off one port —
  a routing choice that is now also an electrical one.
- **Recommended cheap insurance**: a low-ESR **470–1000 µF** electrolytic
  across servo V+/GND at the board. The 6-12V net carries only 10 µF + 0.1 µF
  of local bulk, so a 6.8 A step is sourced through the pack leads and jack;
  a bulk cap sources it locally, cutting rail sag and the transient the jack
  actually sees. **Bench-verify** the peak with an inline shunt or clamp
  during the first walks — the table above is sim-derived, not measured.
- The board's OLED shows measured bus voltage — but it faces the deck once
  the board is mounted, so it's a **bench-side** tool (bring-up, servo IDs).
  In operation the low-battery check is the bus voltage in the 10 Hz radio
  telemetry ([control-channel.md](control-channel.md)). Land the robot by
  **10.5 V on 3S** (3.5 V/cell) / 7.0 V on 2S. (A tower viewing window for
  the OLED is filed as a future improvement.)
- GoPro MAX is self-powered; zero wiring to the robot.

## Servo bus

- 3-pin daisy chain, half-duplex TTL at **1 Mbaud** (driver board UART1,
  GPIO 18/19). Pin order on every connector (Molex-5264-style): **1 GND
  (black) · 2 V+ (red) · 3 DATA (white/blue)**. Every ST3215 has two
  identical, internally-paralleled ports, so chains just hop case to case
  with the included 150 mm leads.
- Per-servo current (12 V class): **2.7 A stall, 0.18 A idle** — the ~10 A
  transient budget in the circuit diagram comes from 2–3 joints near stall
  simultaneously in the gauntlet's worst gaits.
- The board's two bus ports are **electrically the same bus** — we use one
  per leg purely for cable routing:
  - **Port A → left leg**: ID 1 hip roll → ID 2 hip pitch → ID 3 knee → ID 4 ankle
  - **Port B → right leg**: ID 5 hip roll → ID 6 hip pitch → ID 7 knee → ID 8 ankle
- ID order matches the sim's joint order in `sim/walker_env.py`
  (left leg then right, proximal to distal), so the policy's action vector
  maps to IDs 1–8 with no permutation table.
- Servos ship with ID 1: at bring-up, connect **one at a time** and assign
  IDs via the board's web UI (AP mode, `192.168.4.1`), then chain them.
  Two same-ID servos on the bus fail to enumerate.
- **Segment lengths** (worst pose over the full ROM, measured on the routed
  paths in `cad/dress.py`, slack loops included — 2026-07-16):

  | Hop | Worst routed | Lead |
  |---|---|---|
  | board → hip-yaw | ~50 mm (straight down from the tower into the yaw **connector deck hole** over each seat — the servo's two idler-face plugs poke UP through it; connector correction 2026-07-24) | stock 150 mm (coil excess in the tower) |
  | hip-yaw → hip-roll | ~85 mm incl. the ~15 mm service loop across the ±45° yaw sweep (back out the same deck hole, over the deck rear edge, down the back to the roll plugs sticking rearward out of the carrier's opened rear wall at the `SV_CONN` band) | stock 150 mm |
  | hip-roll → hip-pitch | **170 mm** | **≥200 mm extension required** (BOM item 19) — crosses both the roll and hip-pitch joints; longest at the knee-flexion pose |
  | hip-pitch → knee | 111 mm | stock 150 mm (~35 % slack) |
  | knee → ankle | 82 mm | stock 150 mm (worst at ankle −40°) |

  (v3yaw 2026-07-23: the old "board → hip-roll 33 mm" hop became the two
  hops above when hip yaw entered the chain — see docs/hip-yaw-study.md §6
  for the routing openings, corrected 2026-07-24 when the STS3215's ports
  were confirmed on the idler-side face beside the disc (`SV_CONN` in
  cad/dimensions.py; the old `WIRE_CHASE_1/2` cuts aimed at ports that
  don't exist and are deleted). The two ~mm figures are estimates from
  that geometry; re-measure via dress.py routed paths — and confirm the
  exact port offset on a physical servo — before ordering leads. Bus grows to 10 servos, IDs 9/10 =
  L/R hip yaw; sync-read 3.5 → ~4.3 ms, still inside the 20 ms tick.)

  The routed paths already include the service loops, so the stock-lead
  margins above are true flex margin, not taut-string numbers. An earlier
  guess here that shin→ankle was the long run was wrong — it's the shortest
  joint-crossing hop.

## IMU (torso attitude feedback)

- **BNO055 breakout** (the part actually ordered 2026-07-16 — classic
  Adafruit layout, solder header, no STEMMA jacks) on the ESP32's I2C bus
  (GPIO 21 SDA / 22 SCL — shared with the OLED; BNO055 address **0x28**, no
  conflict with the OLED's 0x3C). 4× female-female jumpers: 3V3, GND, SDA,
  SCL. Mounts on the printed **`imu_carrier`** sandwiched between the tower
  top and the gopro_base on the same 4 screws (now M3×12): 4× M2.5×8 into
  bosses on the board's true 21.59 × 15.24 hole pattern — see
  [assembly.md §9b](assembly.md). Mounted long-axis-on-x; set the BNO055's
  `AXIS_MAP_CONFIG`/`AXIS_MAP_SIGN` (standard placements P0–P7) to match
  the silkscreen arrows to the robot frame (+x forward, +z up) at bring-up.
- Why: the policy's observation vector needs the torso **up-vector and
  angular velocity** — servo encoders only cover the 8 joints. The BNO055
  does sensor fusion on-chip and outputs the orientation quaternion directly
  at 100 Hz, so the ESP32's 50 Hz loop just reads it.
- Torso *linear velocity* and *height* have no direct sensor — see the
  observation-ablation results in DESIGN.md for how much they matter.

## Control path (and where latency lives)

| Link | Rate | Role |
|---|---|---|
| Laptop ↔ ESP32 WiFi | 20 Hz cmd / 10 Hz telemetry | **the control channel**: high-level `(vx, yaw_rate)` intent + telemetry — [control-channel.md](control-channel.md) |
| Laptop ↔ USB-C (UART0) | 115200 | flashing, serial bridge, tethered debug |
| ESP32 ↔ servos (UART1) | 1,000,000 | position commands + state readback |

The latency work in DESIGN.md bears directly on this: at 115200 baud a full
8-servo command + state readback cycle eats most of a 20 ms control tick,
and WiFi adds jitter on top. The servo bus itself at 1 Mbaud is ~10× faster
than the link to the laptop. So the plan of record is the one the latency
sims validated: **run the 50 Hz policy loop on the ESP32**. No parts change —
the ESP32 on the order sheet does both jobs.

That decision is also what makes the robot **untethered**: with the policy
loop on-board, the radio never carries joint angles. It carries only the
`(vx, yaw_rate)` command the policy already consumes, 14 bytes at 20 Hz, so a
dropped packet costs staleness rather than a bad servo target — and a link
that goes quiet decays to the zero command, which is a *trained* stand
(`cmd_stand_prob`), not a bolted-on emergency pose. USB-C is for flashing and
debug; nothing about operating the robot needs it. Protocol, failsafe
timings, and the sim-verified loss measurements are in
[control-channel.md](control-channel.md).

## Bring-up checklist

1. Bench-power the bare board (no servos), confirm OLED + web UI.
2. Set servo IDs 1–8 one at a time via web UI; label each case.
3. Chain one leg at a time; confirm enumeration count on the OLED.
4. Torque-off ("Release") all servos, assemble, then use "Set Middle
   Position" at the CAD-neutral pose before first powered stand.
5. Bring up the command link before the first walk: with the robot on a
   stand (feet off the floor), run `link/commander.py --host <ip> --source
   script --script stand` and confirm telemetry comes back, then walk the
   watchdog through its states — kill the commander and check the servos
   release ~5 s later. [control-channel.md](control-channel.md).
