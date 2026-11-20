# Wiring and power

The robot runs on one 3S pack and one controller board, the Waveshare **General
Driver for Robots** (ESP32). The board carries all 17 servos on one serial bus. A
Raspberry Pi 4B has its own 5 V buck. There is no custom electronics.

Related documents:

- board connectors and sensor headers: [sensor-expansion.md §1](sensor-expansion.md#1-the-boards-connectors)
- servo IDs, registers and calibration: [servo-map.md](servo-map.md)
- the procedure from a bare board to the first arm: [bringup.md](bringup.md)
- bench safety: [AGENTS.md](../AGENTS.md#bench-safety)

The firmware currently drives the v6 legs' 12 joints. What this
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
  ├── bulk capacitor, 1000 µF ≥ 25 V low-ESR, across V+/GND
  │
H1  JST XH 2-pin inlet ──────── 3 A per contact; everything below is on the board
  │
AO4407 P-FET (reverse polarity) ── SW1 slide switch
  │
DC_IN ─┬── H5, H6 servo ports: all of the servo current
       │
       └── R11 0.01 Ω (INA219) ── VIN ── MP8759 5 V buck ── 5 V rail ── AMS1117 ── 3V3
                                          (ESP32, on-board sensors, 40-pin header 5 V
                                           -> the Raspberry Pi 4B the board sits on)
```

The board side of this diagram is read off the vendor schematic. The parts above
H1 are the robot's harness.

### Pack

The pack is a 3S LiPo, the OVONIC 3S 4200 mAh 130C XT60 (#107, ordered 2026-10-07):

- 11.1 V nominal. Not 11.4 V "HV": a HV pack charges to 13.05 V, which is above the board's 12.6 V silkscreen.
- Weighed 205 g; its size is the listing's 96 × 44.5 × 25.5 mm, not measured yet. The torso's battery layer is sized to it (`BATT`, `BATT_MASS` in `cad/v6/dimensions_v6.py`).
- XT60 connector.

An RC LiPo has no protection circuit inside it. If a pack will sit for more than a
few days, storage-charge it to 3.8 V/cell.

### Protection board

- 3S, ≥ 15 A continuous on the pack lead, over-discharge cut-off about 3.0 V/cell, on the balance leads.
- It is the backstop that does not depend on the firmware, below battguard's 3.3 V/cell landing line ([Battery protection](#battery-protection)).
- **The board fitted to power lite (ShillehTek) has no under-voltage cut-off** ([bench record, 2026-11-19](design-v6/2026-11-19-power-lite-bms-bench.md)). Until it is replaced (#207), battguard is the pack's only under-voltage protection.
- A UART "smart" BMS (JBD/Daly 3S class) in its place would report per-cell voltages. Whether that is wanted is **open**.
- The torso has no pocket for a separate protection board and buck. It holds power lite in the bay right behind the pelvis, beside the pack, on four standoffs off the aft inner wall (`cad/v6/torso_v8.py`).

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

- **The General Driver sits on the Pi as a HAT** (`cad/v6/torso_v8.py`), so the board's 40-pin 5 V and the Pi's 5 V rail are one net. The Pi takes its power through the header from the board's own buck, an MP8759 labelled "5V-5A … 5V Power for RPi/Jetson nano" (the power-circuit design's plan, PR #112).
- Nothing else feeds the Pi: a second supply on its USB-C would put two 5 V sources in parallel.
- The Pi wants 3 A peaks. That is about 1.4 A from an 11.1 V pack (15 W before converter loss), and it now comes through the board's XH inlet H1 (3 A per contact) and its INA219 shunt.

### What the INA219 measures

This was checked against the schematic and the firmware.

- **Wiring.** The INA219 (U2, address 0x42) has IN+ on `DC_IN` and IN− on `VIN`, across R11 (0.01 Ω).
- **What `VIN` feeds.** Only the board's MP8759 buck, and through it the 5 V rail (and the Pi on its header) and the 3V3 regulator.
- **Where the servos tap in.** H5 and H6 tap `DC_IN` upstream of R11.
- **What it measures.** Pack voltage at `VIN`, behind the reverse-polarity FET, SW1 and the shunt. That reading does not depend on the servo bus and is valid with torque released. It also measures the current into the board's own logic supply, the Pi's included.
- **What it does not see.** None of the servo current.
- **The firmware reads its bus voltage** (`main/pack_sense.cpp`): 10 Hz, in both modes, off the control loop ([firmware-design.md](firmware-design.md#the-pack-sense)). It goes to battguard (below), to `volt` and `batt`, and into the telemetry beacon's power block ([control-channel.md](control-channel.md#the-power-block)). The current is not read.
- **Below 6.0 V it is no pack**, not a flat one: on USB power `DC_IN` is unpowered and the chip, on the 3V3 rail, reads near 0 V.
- **Not measured yet:** its reading against a meter at the pack, and how far the logic's load (the Pi's peaks) sags `VIN` below the pack.
- **Open:** getting pack voltage to the Pi for a low-battery shutdown (#87).

## Current: measured budget (issue #77)

The Waveshare wiki FAQ says the bus-servo current "is controlled by a switch, with
a maximum current of 5A for long-term operation". For ST3215s that means "up to 5
servos (ensuring that these 5 servos are not stalled simultaneously)". Fed from
the General Driver, all of the servo current crosses H1 (3 A per contact), the
AO4407 and SW1, and its ports H5 and H6 carry one side each.

- **Where the current goes on power lite** ([#207](../pcb/power-lite/README.md)).
  The servo rail leaves through F1 (15 A) and K1 to four branch connectors, each a
  Molex 22-03-5035 that carries its whole chain
  ([servo-map.md §2.1](servo-map.md#21-the-robot-the-map-as-assembled)):
  - J4, the right leg: 10, 52, 6, 7, 8, 11;
  - J5, the right upper body: 13, 14, 15;
  - J6, the left leg: 9, 51, 2, 3, 4, 12;
  - J7, the left upper body: 16, 17.
- **Per servo** (ST-3215-C018 sheet):
  - 2.7 A at stall.
  - 180 mA running with no load.
  - 30 mA idle.
  - Over-current protection turns the output off after more than 2 A for 2 s.
- **The STS3250 hip rolls** (51, 52): 50 kg·cm, and 4.2 A at stall at 12 V from
  retailer listings only. No Feetech document gives it, so it is unverified.
- **The per-branch budget is computed from the sim's electrical model**, on the
  robot's servo set, for the motions the robot runs now: the get-up references
  and the track player's dwell walk. Sim figure, not a measurement; the bench
  number with a watt meter on the pack lead is still owed
  ([bringup.md §6](bringup.md#6-staged-hardware-gates)).
- The bus timing is open. Twelve servos take ≈ 3.4 ms of the 20 ms tick ([design record](design-v6/2026-09-13-design-record.md) §4.5); nobody has computed the figure for 17.

The method (`sim/current_budget.py --trace refs | dwell`):

- Every 20 ms tick takes each servo's torque and speed from the plant's servo
  model. Two supply-current models from `sim/sts_servo_model.py` bracket the truth:
  - **upper** (the motor's current): 30 mA + 150 mA × |ω|/ω₀ + |τ|/Kt, capped at
    the stall current × V/12;
  - **lower** (a lossless bridge): 30 mA + (max(τω, 0) + K_CU·τ²)/V.
- Each servo is priced with its own type's constants:
  - the STS3215 with its datasheet (Kt 1.08 N·m/A) and the fitted
    K_CU = 3.75 W/(N·m)², which puts the 2.94 N·m stall at 2.7 A at 12 V;
  - the STS3250 with Kt and K_CU fitted the same way to 4.90 N·m at 4.2 A, and
    the STS3215's idle and no-load currents.
- The currents are summed per branch, per General Driver port (H5 = J4 + J5,
  H6 = J6 + J7) and over the bus. A servo the plant lacks (the neck and arms on
  the legs-only plant) counts at its 30 mA idle.
- **`--trace refs`, the get-ups.** The campaign references
  `sim/getup_ref_v6ar_prone_ikc.npz` (the one the get-up candidate tracks) and
  `sim/getup_ref_v6ar_ik.npz` (the supine seat push) are replayed open loop as
  `sim/getup_ref_overload.py` replays them:
  - on `sim/bimo_biped_v6ar_meas.xml` (17 servos, CAD inertials, measured stops,
    sole pads);
  - with the get-up's deploy servo model, kp as built, at the 11.1 V training
    envelope;
  - nominal, and with `--all-conds` in that study's nine robustness conditions:
    friction, μ, play, servos at 85 % and 65 %, mass 110 %. Both references stand
    in all of them.
- **`--trace dwell`, the walk.** The dwell gait (`experiments/roll-balance/dwell.py`)
  runs through the firmware's track player and trim in the SIL (`sil_gait.run`,
  lift gate on).
  - The plant is the legs-only `fit` plant the gait was tuned on: per-side roll
    play, 12 V, 12 servos, as built.
  - It runs at T = 6, 4 and 3.2 s a step (track_host's start, the robot's walks,
    the SIL's floor): march, forward, back, both sides and both turns, 6 steps
    each. None of the 21 runs falls.

The figures are in amps, upper / lower model:

- **Peak** is the worst 20 ms tick.
- **Worst 2 s** is the largest 2 s mean: the servo's protection timer, and
  roughly what a fuse or a contact sees.
- **Over** is the time above 5 A on the bus, or above 3 A on a lead.

| motion | lead | peak | worst 2 s | RMS | over (s) |
|---|---|---|---|---|---|
| **Prone get-up** (`prone_ikc`, 41.6 s), nominal | bus | 13.2 / 7.2 | 8.4 / 3.9 | 4.9 / 2.0 | 11.6 / 1.4 |
| | J4 R leg | 7.5 / 5.1 | 4.2 / 1.8 | 2.0 / 0.85 | 3.2 / 0.4 |
| | J5 R upper | 3.5 / 2.6 | 1.9 / 0.86 | 0.75 / 0.36 | 0.08 / 0 |
| | J6 L leg | 7.3 / 4.6 | 3.7 / 1.6 | 1.9 / 0.74 | 2.9 / 0.08 |
| | J7 L upper | 3.4 / 2.5 | 1.8 / 0.83 | 0.71 / 0.32 | 0.06 / 0 |
| | worst of 10 conditions: bus | 14.1 / 8.1 | 11.4 / 6.4 | 5.7 / 2.6 | 17.0 / 3.1 |
| | worst of 10: a leg branch | 8.5 / 5.7 | 4.4 / 2.3 | 2.2 / 0.98 | 5.2 / 0.94 |
| | worst of 10: an upper branch | 3.6 / 2.8 | 2.6 / 1.6 | 0.98 / 0.55 | 0.08 / 0 |
| **Supine get-up** (`supine_ik`, 30.3 s), nominal | bus | 10.3 / 5.5 | 8.5 / 4.0 | 4.2 / 1.7 | 4.4 / 0.86 |
| | J4, J6 legs | 3.0 / 1.7 | 2.4 / 1.2 | 1.5 / 0.55 | 0.02 / 0 |
| | J5, J7 upper | 3.5 / 2.6 | 1.95 / 0.93 | 0.79 / 0.38 | 0.06 / 0 |
| | worst of 10 conditions: bus | 13.2 / 8.2 | 12.0 / 7.0 | 5.2 / 2.5 | 8.0 / 2.0 |
| | worst of 10: any branch | 3.6 / 2.8 | 3.3 / 1.8 | 1.7 / 0.70 | 1.8 / 0 |
| **Dwell walk**, worst of 21 runs | bus | 3.8 / 1.2 | 2.6 / 0.77 | 2.2 / 0.70 | 0 / 0 |
| | J4, J6 legs | 2.3 / 0.72 | 1.6 / 0.39 | 1.1 / 0.30 | 0 / 0 |
| | J5, J7 upper | idle: 0.09 A (J5), 0.06 A (J7) | | | |

How to read the numbers:

- **The walk fits.** It clears the 5 A board rating on the bus and ~3 A on every
  branch in both models.
- **The get-ups do not fit the 5 A board rating.**
  - The bus peaks at 10–14 A (upper) or 5.5–8.2 A (lower), and its worst 2 s is
    8.4–12 A or 3.9–7.0 A.
  - It sits above 5 A for 4–17 s of a get-up (upper) or 0.9–3 s (lower).
  - Fed from the General Driver, its ports would carry up to 9.5 / 6.1 A peak and
    5.7 / 3.2 A over 2 s (H5, prone).
  - Power lite's F1 (15 A) sits above every figure in both models. The upper
    model's worst tick (14.1 A) is within 1 A of it.
- **The prone get-up loads one leg's lead.**
  - J4 and J6 peak at 7.3–8.5 A (upper) or 4.6–5.7 A (lower), and stay above 3 A
    for 2.9–5.2 s or 0.1–0.9 s.
  - The peak is one leg's hip yaw, hip roll, hip pitch and knee pushing together.
    At t ≈ 7.4 s, J4 carries hip pitch 2.3 A, knee 1.9 A, hip roll 1.5 A and hip
    yaw 1.1 A (upper model).
  - The worst single servo is the left hip roll (the STS3250), 3.1–3.3 A upper or
    2.5–2.9 A lower.
- **The supine get-up loads the arms.** The upper branches (shoulder and elbow)
  peak at 3.5 A upper or 2.6 A lower, above 3 A for under 0.5 s and in the upper
  model only. The legs' leads peak near 3 A.
- **RMS** sizes copper and contacts. Every branch stays at or under 2.2 A
  (upper) or 1.0 A (lower).
- **The peak** sizes the fuse and the brown-out margin. Whether a ~3 A-class KK
  contact takes 4–8 A for a few seconds is a datasheet question. No connector
  datasheet is mirrored here (see the Servo bus section below).
- The shoulder is an STS3215 in both designs. At 79–83 % of stall in the current get-up references, it is torque-limited for 0.4–1.0 s of each push ([getup-overload-2026-11-10.md](design-v6/getup-overload-2026-11-10.md)). That puts it close to the servo's overload protection (> 80 % for 2 s, registers 34–36). It peaks at 2.1 A in the upper model, just past the datasheet's 2 A over-current line, briefly. **Watch on the bench.**
- **Not in the budget.** The 12 V refit envelope (#214) doubles the hip pitch's
  and knee's stall torque, which neither current model covers: the upper caps at
  the datasheet stall current, and the lower grows past it with τ². Its figures
  are in the log only. Also left out: pack sag (each trace runs at a fixed bus
  voltage), servo heating, the Pi's ≈ 1.4 A from its own buck, and the arms and
  neck on the walk.

A trained walking policy can load the bus differently. For scale, the same
method on the 10-servo prototype's learned walk (`--run`, never measured on
hardware, bridge model):

| | peak | p99 | RMS | mean |
|---|---|---|---|---|
| whole bus | 6.8 A | 5.0 A | 1.4 A | 0.85 A |
| worst leg | 4.0 A | — | ~0.7 A | — |
| worst joint | 3.0 A | — | — | — |

What follows is unset; the choice of inlet, fuse and bulk capacitor stays with
the owner:

- **The get-up's leg leads** are the one place the budget does not fit, in both
  models. Three options have been named:
  - a contact rated for the peak, which needs the KK datasheet;
  - a second lead per leg;
  - a get-up that loads fewer of one leg's servos at once.
- **A trained walk** gets re-checked with `--run <run>` once a policy for the
  robot exists.

Reproduce (logs: [refs](design-v6/current_budget_2026-11-16_refs.txt),
[dwell](design-v6/current_budget_2026-11-16_dwell.txt)):

```bash
make -C firmware/host sil        # the SIL library sil_gait loads
.venv/bin/python sim/current_budget.py --trace refs --all-conds
.venv/bin/python sim/current_budget.py --trace dwell
```

- `--trace dwell` also needs the grade plant's run (`sim/runs/robot_legs_prec16_f075_js20_s128r24`, gitignored), as `sil_gait.py` does.
- `--trace walk` and `--trace getup` replay the 2026-09 scripted Gate D walk and seat push. The robot no longer runs either motion.
- The JSON goes to `sim/runs/`, which is gitignored: it is a build artifact, not a deliverable.

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

### Per-hop lead lengths (from the v6 CAD, ARMS=1)

The spans below are **geometric minima** at the standing pose, measured from
the CAD constants and cross-checked against the built assembly mocks
(`python cad/v6/lead_lengths.py` and its `--assembly` sweep; the hop model and
every routing caveat are in that file's docstring). "loop" is the fold loop
the lead needs to cross the joint named (arc over the joint's ROM at the
cable's own standoff), already folded into "need". Verdicts are against the
150 mm stock lead with a 20 mm plug tail. The board→splitter stub rides on
top of the first hop as a 30 mm allowance — the splitter itself is not
selected.

| hop (chain order, both legs symmetric except where shown) | span | loop | need | buy ~ | 150 ok? |
|---|---|---|---|---|---|
| port (H5/H6) → front inner wall's top window (via splitter) | 57 | — | 57 | 100 | ✓ |
| wall window → battery floor (under the pack) | 34 | — | 34 | 80 | ✓ |
| battery floor → hip yaw (through the ceiling slit) | 26 | — | 26 | 70 | ✓ |
| hip yaw → hip roll | 61 | 7 | 68 | 110 | ✓ |
| hip roll → hip pitch | 55 | 9 | 64 | 110 | ✓ |
| hip pitch → knee | 110 | 17 | 127 | 170 | ✓ |
| knee → ankle | 110 | 18 | 128 | 170 | ✓ |
| ankle → ankle roll (R / L) | 81 / 87 | 6 | 87 / 93 | 130 / 140 | ✓ |
| port (H5/H6) → deck opening between the boards (upper branches, via splitter) | 50 | — | 50 | 90 | ✓ |
| deck opening → neck (port A) | 47 | — | 47 | 90 | ✓ |
| shoulder lead leaving the pod (A: neck → R shoulder; B: deck opening → L shoulder) | 71 / 68 | — | 71 / 68 | 120 / 110 | ✓ |
| **shoulder → elbow (both arms)**, along its route through the girdle's aft slot | **312** | **23** | **335** | **~380** | **NO — does not fit stock** |

What the CAD says about the two named geometry problems:

- **The shoulder→elbow hop is the only hop that cannot ride the stock 150 mm
  lead.** The shoulder servo hangs in the girdle pod at the top of the torso;
  the elbow servo's case rides the upper arm, running up it from the elbow
  axis, so its connector sits 146 mm below the shoulder axis. The lead crosses
  the **shoulder fold, −90°…+200° (290° of sweep)**: it leaves the
  girdle's open notch upward (the notch is cut open to the pod top), runs aft
  through the elbow lead's slot in the aft tie beside the pod
  (`shoulder_girdle_v6.lead_slot`), round the pod's aft lug, and down the
  back of the upper arm's web to the window just above the elbow servo's case
  (`arm_v6.py`).
  There is **no strain relief drawn at the fold**. The span is the length of
  the route: a polyline through the CAD's corners at the hanging pose
  (`_elbow_route` in `cad/v6/lead_lengths.py`: up the notch, inboard over the
  wall, down to the slot's floor, aft through it, round the aft lug, outboard
  past the cheek, down the arm's back web, through its window into the elbow
  plug), 312 mm; the 23 mm is the shoulder fold's loop. It is not a swept
  measurement. The straight plug-to-plug minimum between the two servos'
  connectors (assembly mocks) is 130 mm. Buy ~380 mm leads and trim on the bench. Near the shoulder's −90° end the
  arm lies aft across the lead's outboard run, so the lead rides over the arm's
  back web there; that is checked on the bench, not in the CAD.
- **The lead never crosses the elbow.** The elbow servo turns with the upper
  arm (the forearm forks round its discs), so shoulder plug to elbow plug is
  one piece on one link below the shoulder.
- What is **flagged, not invented**: the yaw carrier has no raceway drawn for
  the yaw→roll hop (straight-line minimum), the leg links carry cable
  *windows*, not routed channels, and the ankle link's "raceway" named at
  `dimensions_v6.py:180` is not drawn. All three are straight-line minima in
  the table.

Sources: the hop model, anchor placements and the measured arm span are
`cad/v6/lead_lengths.py`; placements come from `assembly_v6`'s own mock
chains (the interference gate's placement truth), and the fold/ROM figures
from `dimensions_v6.py` and `docs/design-v6/arms.md` §5.

- **Open (remaining):** the splitter part (both ports feed one, not
  selected); the bus-timing figure for 17 servos.

## Battery protection

The pack is protected against over-discharge by:

1. **The protection board**, which should cut off at about 3.0 V/cell, in hardware. **Power lite's BMS has no under-voltage cut-off** ([bench record, 2026-11-19](design-v6/2026-11-19-power-lite-bms-bench.md)); a replacement is open (#207). Until then the two layers below are all there is.
2. **battguard** (`firmware/components/battguard/`) is the firmware supervisor. It is pure, and its behaviour is asserted on the host in `firmware/host/test_battguard.cpp`.
3. **The operator** lands the robot by 10.5 V. Pack voltage arrives in the 10 Hz telemetry (the power block's `pack_mv` from the INA219, and `vbat` from the servos) and from `volt` on the CLI; the power block also carries battguard's level.

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
  - To clear it, swap the pack, then run `batt reset` (bench mode) or reboot. A pack still at the line latches it again within a second.
  - While it holds, `home` refuses.
- **Landing stays in distribution.** 0.60 is `walker_env`'s `crouch_range[0]`, the lowest stance the deployed policy was trained to hold. The guard does not cut torque on a standing biped.
- **Two senses, the lower wins.** Each tick the guard takes register 62 of the first servo that answers and the INA219's pack voltage (truncated to 0.1 V), and judges the lower of the two, or whichever there is (`battguard::inputDv`). It therefore trips no later than it would on the servos alone. The INA219 sits on the logic's path, so under the Pi's load it may read a little below the pack and trip a little earlier; how much is not measured yet.
- **No reading is not 0 V.** A tick in which no servo answered and the INA219 has no fresh reading (none for 1 s, a missing chip, or under 6.0 V: USB power) counts as "no data".
- **It does not share the bus's failure.** Lose the servo bus and the INA219 keeps it guarding.
- **It guards on the bench too**, on the INA219 alone; nothing moves on a trip, but the latch holds: `rail on` and `home` are refused, a hold that comes upright and still is released instead of homed, and an arm lands at once and goes limp. It cannot turn the logic off: the ESP32, the board and the Pi keep drawing about 1 A until the main switch is turned off.
- **`batt pack off`** (tether, RAM only) takes the INA219 out of the guard until the next boot, should it misread on a robot.

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

[`docs/wiring-general-driver.svg`](wiring-general-driver.svg) shows the robot's harness on this board:

- the pack, the BMS and the main switch on power lite's connectors and pads ([pcb/power-lite/README.md](../pcb/power-lite/README.md));
- power lite's feeds: the servo rail through K1 and F1, the General Driver and the Pi through F2, the E-stop on J11;
- the four servo branches (J4–J7) with the servo IDs of [servo-map.md §2.1](servo-map.md#21-the-robot-the-map-as-assembled);
- the control signals to the ESP32 (`SERVO_EN`, `RAIL_V`, `ESTOP_N`).

It leaves off what the sources do not settle: the physical fan-out of J12 to the General Driver's headers (drawn on Waveshare's photo in [pcb/power-lite/harness.md](../pcb/power-lite/harness.md#j12-control-to-the-esp32); not yet checked on the board in hand) and the neck and arm IDs, which are drawn as a proposal.

## Bring-up checklist

The full procedure is [bringup.md](bringup.md). In short:

1. Flash over the `USB` port. USB powers the logic but not the servos ([§1](bringup.md#1-flash-and-first-power)).
2. First power through the XH inlet, current-limited ([§1](bringup.md#1-flash-and-first-power)).
3. Set the servo IDs with one servo on the bus at a time, and read the gains back ([§2](bringup.md#2-servo-ids-and-gains)).
4. At the assembled neutral pose, run `middle` per servo, then `cal zero`, `cal dir` and `cal save` ([§3](bringup.md#3-calibrate-zeros-directions-imu-clock)).
5. Run the torque drills: release, reset, and the watchdog decay ([§4](bringup.md#4-torque-drills)).
