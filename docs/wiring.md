# Wiring and power

The robot runs on one 3S pack and one controller board, the Waveshare **General
Driver for Robots** (ESP32). The board carries all 17 servos on one serial bus. A
Raspberry Pi 4B has its own 5 V buck. There is no custom electronics.

Related documents:

- board connectors and sensor headers: [sensor-expansion.md §1](sensor-expansion.md#1-the-boards-connectors)
- servo IDs, registers and calibration: [servo-map.md](servo-map.md)
- the procedure from a bare board to the first arm: [bringup.md](bringup.md)
- bench safety: [AGENTS.md](../AGENTS.md#bench-safety)

The firmware currently drives the 10-joint prototype (servo IDs 1–10). What this
page says about 17 servos is the robot's design. Anything marked **open** is not
settled yet.

## The board

| | |
|---|---|
| board | Waveshare General Driver for Robots, rev 1.2. ESP32-WROOM-32UE module (ESP32-D0WD-V3), 4 MB flash |
| outline | **65.01 × 56.01 mm**, holes on a 58 × 49 mm grid, per the dimension drawing. The wiki says "65 × 65 mm"; that is wrong |
| power in | H1, a JST XH 2-pin connector (item 10 on the vendor's connector diagram), silkscreened "− +" |
| input range | The wiki says "DC 7-13V". The rev 1.2 silkscreen says "DC 9-12.6V". This is unresolved. A 3S pack, used between 9.9 and 12.6 V, is inside both |
| servo bus | H5 and H6 (item 13). UART1 at 1 Mbaud, GPIO 18 RX / 19 TX |
| console | the USB-C port silkscreened `USB`: a CP2102N to UART0 at 115200, with DTR/RTS auto-program |
| IMU | the on-board QMI8658C, 0x6B on I²C (GPIO 32/33) |

- GPIO constants are in `firmware/main/board.h`.
- The schematic and dimension drawing are in [datasheets/general-driver/](datasheets/general-driver/).

## Power path

```
3S LiPo, 11.1 V nominal, 12.6 V full
  │
3S protection board ─────────── ≥ 15 A continuous, over-discharge cut-off ~3.0 V/cell
  │
inline fuse ─────────────────── ATM mini-blade, 15 A (10 A once the peak is measured)
  │
switch ──────────────────────── the E-stop that does not need the radio
  ├── Pololu D24V50F5, 5 V / 5 A ── USB-C ── Raspberry Pi 4B        (tap point: open)
  ├── bulk capacitor, 1000 µF ≥ 25 V low-ESR, across V+/GND
  │
H1  JST XH 2-pin inlet ──────── 3 A per contact; everything below is on the board
  │
AO4407 P-FET (reverse polarity) ── SW1 slide switch
  │
DC_IN ─┬── H5, H6 servo ports: all of the servo current
       │
       └── R11 0.01 Ω (INA219) ── VIN ── MP8759 5 V buck ── 5 V rail ── AMS1117 ── 3V3
                                          (ESP32, on-board sensors, 40-pin header 5 V)
```

The board side of this diagram is read off the vendor schematic. The parts above
H1 are the robot's harness.

### Pack

The pack is a 3S LiPo, bought to a spec filter:

- 11.1 V nominal. Not 11.4 V "HV": a HV pack charges to 13.05 V, which is above the board's 12.6 V silkscreen.
- 2200–2600 mAh.
- ≤ 105 × 36 × 26 mm. That is the pelvis battery layer, `BATT` in `cad/v6/dimensions_v6.py`.
- 150–190 g.
- XT30 or XT60 connector, 25C or better.

An RC LiPo has no protection circuit inside it. If a pack will sit for more than a
few days, storage-charge it to 3.8 V/cell.

### Protection board

- 3S, ≥ 15 A continuous on the pack lead, over-discharge cut-off about 3.0 V/cell, on the balance leads.
- It is the backstop that does not depend on the firmware, below battguard's 3.3 V/cell landing line ([Battery protection](#battery-protection)).
- A UART "smart" BMS (JBD/Daly 3S class) in its place would report per-cell voltages. Whether that is wanted is **open**.
- The protection board and the buck share a 60 × 8.4 × 25 mm pocket beside the pack (`PWR_BOARD`).
- The pelvis bosses in that pocket are placeholders until both boards are in hand (`cad/v6/pelvis_v7.py`).

### Fuse

- The fuse is a mini-blade (ATM) fuse in an inline holder on the pack lead.
- It protects against a dead short. The board's own servo path has a reverse-polarity FET and a switch, but no fuse.
- Start at 15 A. Drop to 10 A only after the real peak has been measured.
- **Never use a polyfuse/PPTC.** It takes seconds to trip, and its series resistance sags the rail the rest of the time.

### Switch

- An inline switch on the pack lead. It is the robot's real E-stop ([control-channel.md](control-channel.md)).
- The board's own SW1 switches `DC_IN` behind the inlet. The schematic gives no rating for it. The Waveshare FAQ says the bus-servo current "is controlled by a switch" ([Current](#current-the-open-constraint)).

### Inlet

- H1 is a JST XH 2-pin connector, 2.50 mm pitch.
- JST rates each contact at 3 A AC/DC at AWG #22, and the contacts take AWG #30–#22 (`JST-XH-connector-datasheet.pdf`).
- A 20 AWG pack lead does not go into the housing. Splice it to a 22 AWG XH pigtail.
- Pin 1 is V+ (`VDD_DC_JACK`) and pin 2 is GND. Pigtail wire colours are not standardised, so check against the "− +" silkscreen before the first plug-in.

### Bulk capacitor

- 1000 µF (470 µF at minimum), ≥ 25 V (a full 3S is 12.6 V), low-ESR, 105 °C. Mind the polarity.
- `DC_IN` carries only 10 µF + 0.1 µF of local bulk (C23, C28). A servo current step is therefore sourced through the pack leads and the inlet.
- The board has no free `DC_IN` connector, because both servo ports carry chains. The capacitor therefore goes across V+/GND on the inlet pigtail.
- There it cuts rail sag, but it does not relieve the XH contact itself. Soldering it to `DC_IN` on the board would.

### Pi supply

- A Pololu D24V50F5 (5 V, 5 A, 17.8 × 25.4 mm) drives a USB-C lead to the Pi 4B.
- The Pi wants 3 A peaks. That is about 1.4 A from an 11.1 V pack (15 W before converter loss).
- Where the buck taps the pack line is **open**. Tapping upstream of H1 keeps this current off the 3 A inlet contact.
- The board has its own 5 V buck: an MP8759 labelled "5V-5A … 5V Power for RPi/Jetson nano", feeding the 40-pin header's 5 V pins. The robot powers the Pi from the D24V50F5 instead.

### What the INA219 measures

This was checked against the schematic and the firmware.

- **Wiring.** The INA219 (U2, address 0x42) has IN+ on `DC_IN` and IN− on `VIN`, across R11 (0.01 Ω).
- **What `VIN` feeds.** Only the board's MP8759 buck, and through it the 5 V rail and the 3V3 regulator.
- **Where the servos tap in.** H5 and H6 tap `DC_IN` upstream of R11.
- **What it measures.** Pack voltage at `VIN`, behind the reverse-polarity FET, SW1 and the shunt. That reading does not depend on the servo bus and is valid with torque released. It also measures the current into the board's own logic supply.
- **What it does not see.** None of the servo current. None of the Pi's current, which comes from its own buck.
- **The firmware does not read it.** `board::kPowerAddr` is marked unused, `imu scan` labels 0x42 "(unused)", and battguard reads the servos' register 62 instead.
- **Open:** reading the INA219 in firmware, and getting pack voltage to the Pi for a low-battery shutdown.

## Current: the open constraint

The Waveshare wiki FAQ says the bus-servo current "is controlled by a switch, with
a maximum current of 5A for long-term operation". For ST3215s that means "up to 5
servos (ensuring that these 5 servos are not stalled simultaneously)". All of the
servo current crosses H1 (3 A per contact), the AO4407 and SW1.

- **Per servo** (ST-3215-C018 sheet §5, §7-11; memory table V3.7):
  - 2.7 A at stall (30 kg·cm at 12 V); Kt 11 kg·cm/A (1.08 N·m/A).
  - 180 mA running with no load; 30 mA idle.
  - Over-current protection turns the output off after more than 2 A for 2 s
    (datasheet). The memory table's default over-current threshold
    (register 28) is 3.25 A. The two documents disagree, and at 11.1 V only
    the datasheet's 2 A can ever be reached.
  - Overload protection drops the servo to 20 % torque after its load stays
    above 80 % for 2 s (registers 34–36, enabled by default).
- The bus timing for 17 servos is open. Twelve servos take ≈ 3.4 ms of the 20
  ms tick ([design record](design-v6/2026-09-13-design-record.md) §4.5).

### The simulated budget

`sim/current_budget_v6.py` (log: [current_budget_v6.txt](design-v6/current_budget_v6.txt))
applies a per-servo current model to every 20 ms control tick of the design's
open-loop motions, under Plan B at 11.1 V:

- **walk**: the Gate D cases on the as-drawn plant with arms (17 servos, arms
  held at +15°), and on the CAD-inertial plant (13 servos, the four arm servos
  added at idle);
- **get-up**: the scripted seat push (`r5_asdrawn_rom120`), both sequences,
  the six robustness conditions.

The documents give the current along the servo's full-duty line; a PWM bridge
holding a load at part duty draws less from the supply. So two models bracket
it, and the shunt on the first powered run settles which end is true:

| model | per-servo current | constants |
|---|---|---|
| **upper** (supply = motor current) | 30 mA + 150 mA × \|ω\|/ω₀ + \|τ\|/Kt, capped at the stall current (2.50 A at 11.1 V) | documented: Kt, idle, no-load and stall current. Fitted: the no-load term's straight line in speed |
| **lower** (lossless bridge) | 30 mA + (max(τω, 0) + K_CU·τ²)/V | K_CU = 3.75 W/(N·m)², fitted so that the 2.94 N·m stall draws 2.7 A at 12 V (walker_env's power model, also `sim/current_budget.py`'s) |

Whole bus, amperes, upper / lower:

| motion | peak | RMS | mean | worst 2 s |
|---|---|---|---|---|
| walk, 17 servos (the four cases that walk) | 5.1 / 1.8 | 2.7 / 0.9 | 2.7 / 0.9 | 3.4 / 1.2 |
| walk, CAD plant, 13 + 4 idle servos | 3.9 / 1.3 | 2.0 / 0.7 | 2.0 / 0.7 | 2.4 / 0.9 |
| get-up, recommended sequence (shoulder 90 → 0) | 10.3 / 5.2 | 2.7 / 1.0 | 2.4 / 0.8 | 5.2 / 2.1 |
| get-up, shoulder 60 → 0 | 11.8 / 6.7 | 3.0 / 1.2 | 2.6 / 0.9 | 7.6 / 3.4 |

The walk case that falls (−15° turn at μ 0.9) peaks at 5.7 / 4.0 A in the fall.
The get-up's peak is the push: shoulders, elbows, hip pitches and knees loaded
together (shoulders 1.7 A each, knees 1.4 A, hip pitches 1.3 A in the upper
model). The worst run is at μ 1.0.

Per chain, upper / lower, for three ways to split the bus:

| split | chain | walk peak / RMS | get-up (recommended) peak / worst 2 s |
|---|---|---|---|
| **B: two ports** | L leg + L arm + neck | 3.9 / 1.5 — 1.4 / 0.5 | 5.1 / 2.6 — 2.6 / 1.1 |
| | R leg + R arm | 3.8 / 1.5 — 1.4 / 0.5 | 5.1 / 2.6 — 2.6 / 1.0 |
| C: two ports | both legs | 4.9 / 2.5 — 1.7 / 0.8 | 5.3 / 2.7 — 2.6 / 1.1 |
| | arms + neck | 0.6 / 0.2 — 0.2 / 0.2 | 5.0 / 2.5 — 2.7 / 1.0 |
| A: three chains (a splitter) | L leg / R leg / arms + neck | 3.7 / 1.4 each leg; 0.6 / 0.2 | 2.7 / 1.3 each leg; 5.0 / 2.5 |

What the numbers say:

- **The walk fits the board's 5 A continuous rating** in both models (RMS
  0.9–2.7 A, worst 2 s 1.2–3.4 A); the upper model's peaks touch 5 A.
- **The get-up's push does not**: 5.2–10.3 A peak, and in the upper model a 2 s
  mean of 5.2 A, the length of the push. The 60 → 0 sequence is worse on every
  line, which is one more reason the 90 → 0 one is recommended.
- **H1 is the tightest part.** All of the servo current crosses one 3 A XH
  contact. The walk's upper RMS (2.7 A) is 90 % of it and the get-up exceeds
  it. Feeding `DC_IN` directly, bypassing the XH inlet and SW1, is the
  indicated power path; the shunt run confirms it.
- **Split B** (one leg and one arm per port, the neck on either) spreads the
  get-up evenly: 5.1 A peak and 2.6 A worst 2 s per port in the upper model,
  against ≈ 3 A for a Molex-5264-class lead (not verified from a datasheet).
  Split C puts both legs on one lead (4.9 A walk peak, 2.5 A RMS); split A
  needs a splitter and puts all four arm servos on one lead.
- **Fuse**: stay at 15 A. The upper model's get-up peaks (10–12 A) sit at a
  10 A fuse's rating; drop to 10 A only once the shunt shows the real peak.
- **Servo protection**: no servo in the walk passes 54 % of stall (70 % on the
  duty reading), so neither cutoff can fire there. In the get-up the shoulders
  peak at 1.7 A, under the datasheet's 2 A over-current line; the overload
  margin is in [DESIGN.md §4](../DESIGN.md#4-actuation-one-servo-type-with-a-raised-position-gain).

Not in the budget: pack sag under load (the traces run at a fixed 11.1 V),
servo heating, and the Pi's ≈ 1.4 A from its own buck.

**Then the bench**: an inline watt meter (≥ 60 A, with peak hold) on the pack
lead during the floor sequence ([bringup.md §6](bringup.md#6-staged-hardware-gates)).
A sim figure is a ranking, not a measurement. RMS sizes copper and contacts;
the peak sizes the fuse and the brown-out margin.

## Servo bus

- **The bus.**
  - One bus: UART1 at 1 Mbaud, half-duplex TTL, GPIO 18 RX / 19 TX.
  - The direction is switched in hardware off the TX line (SN74LVC1G125/126 buffers). There is no direction GPIO.
  - The firmware never sees its own transmission. A reply can start microseconds after the last stop bit.
- **Pinout.**
  - H5 and H6 carry the same three nets: pin 1 `DATA`, pin 2 `DC_IN` (V+), pin 3 GND. The silkscreen reads "D V G".
  - The servo's 5264-3P lead numbers its pins the other way: 1 GND, 2 V+, 3 signal (C018 sheet). The middle pin is V+ either way.
- **Chaining.** Every ST3215 has two paralleled ports, so a chain hops from case to case on the stock 150 mm leads.
- **The first lead of a chain carries the whole chain's current.**
  - A Molex-5264-class contact is roughly a 3 A part. No connector datasheet is mirrored here, so that figure is unverified.
  - The chains are split one per port for this reason.
  - With 17 servos and two ports, at least one chain carries nine or more servos, unless a splitter is added. None is specified.
- **Open:**
  - The robot's chain layout: which servos go on which port, and in what order.
  - The routed lead length of each hop on the robot's CAD.

## Battery protection

The pack has three layers of protection against over-discharge:

1. **The protection board** cuts off at about 3.0 V/cell. This is hardware and does not depend on the firmware.
2. **battguard** (`firmware/components/battguard/`) is the firmware supervisor. It is pure, and its behaviour is asserted on the host in `firmware/host/test_battguard.cpp`.
3. **The operator** lands the robot by 10.5 V. Pack voltage arrives in the 10 Hz telemetry (`vbat`, mV) and from `volt` on the CLI.

Nothing else stops a discharge. The servos' own under-voltage protection sits at 4 V
(C018 sheet), far below the ~9 V (3.0 V/cell) at which a 3S pack is damaged.

battguard's levels are below. The firmware builds the guard with the 3S thresholds;
the 2S constants exist in `guard.h`.

| level | 3S | 2S | what the robot does |
|---|---|---|---|
| ok | > 10.5 V | > 7.0 V | normal |
| warn | ≤ 10.5 V (3.50 V/cell) | ≤ 7.0 V | a telemetry flag only: **you** land it |
| land (`VLAND`) | ≤ 9.9 V (3.30 V/cell) | ≤ 6.6 V | zeroes vx, vy, wz and lift, and ramps the crouch command to 0.60 over 1.5 s |
| safe (`VSAFE`) | 1.5 s after land | | torque off, latched |

These design points carry load:

- **Debounced.** A trip needs 25 consecutive ticks (0.5 s at 50 Hz) at or below the line. Gait transients sag the rail through the pack leads and the inlet, so an instantaneous comparator would false-fire mid-stride.
- **Latched.** An unloaded flat 3S pack rebounds above 10.5 V within seconds of torque dropping. A guard that cleared on recovery would cycle the robot up and down on a ruined pack.
  - To clear it, swap the pack, then run `batt reset` (bench mode) or reboot.
  - While it holds, `home` refuses.
- **Landing stays in distribution.** 0.60 is `walker_env`'s `crouch_range[0]`, the lowest stance the deployed policy was trained to hold. The guard does not cut torque on a standing biped.
- **A dropped bus frame is not 0 V.** The voltage comes from servo feedback, specifically register 62 of the first servo that answers each tick. A tick in which no servo answered counts as "no data".
- **It guards only while armed.** The benched loop reads no servos. On the bench, only the protection board guards the pack.
- **It shares the bus's failure.** Lose the bus and the guard goes blind. The INA219 would remove that coupling, and it is not read.

The advisory landing voltage (`board::kLandVoltage3s` = 10.5 V) and `battguard::kWarn3S` (105 dV) are the same number, and they must agree.

## Control path

| link | rate | role |
|---|---|---|
| ESP32 ↔ servos, UART1 | 1 Mbaud | sync-write targets and read feedback, every 20 ms tick |
| commander ↔ ESP32, Wi-Fi UDP | 20 Hz commands in (port 4210), 10 Hz telemetry out (4211) | the control channel: intent in, state out ([control-channel.md](control-channel.md)) |
| laptop ↔ ESP32, USB-C `USB` | 115200 | flashing, the CLI, `obsdump` |
| Pi ↔ ESP32 | — | **open**; the options are in [sensor-expansion.md §3](sensor-expansion.md#3-perception-the-pi-and-the-head-camera) |

The 50 Hz loop and the policy both run on the ESP32. The radio carries a command,
never joint angles. When the link dies, the robot decays to the trained stand, then
releases torque.

## Diagrams

`docs/wiring-general-driver.svg` shows the 10-servo prototype harness on this board:

- an 850 mAh pack, a GoPro and a BNO085 on P1;
- no protection board, buck or Pi;
- a port and ID layout that does not match the firmware's.

It does not describe the robot, and redrawing it is **open**.

## Bring-up checklist

The full procedure is [bringup.md](bringup.md). In short:

1. Flash over the `USB` port. USB powers the logic but not the servos ([§1](bringup.md#1-flash-and-first-power)).
2. First power through the XH inlet, current-limited ([§1](bringup.md#1-flash-and-first-power)).
3. Set the servo IDs with one servo on the bus at a time, and read the gains back ([§2](bringup.md#2-servo-ids-and-gains)).
4. At the assembled neutral pose, run `middle` per servo, then `cal zero`, `cal dir` and `cal save` ([§3](bringup.md#3-calibrate-zeros-directions-imu-clock)).
5. Run the torque drills: release, reset, and the watchdog decay ([§4](bringup.md#4-torque-drills)).
