# Power lite: the robot's battery protection and power distribution

*The choices behind it are in
[2026-11-02-power-lite-options.md](../../docs/design-v6/2026-11-02-power-lite-options.md). It
replaces the custom [power board](../power-board/README.md) for the robot. How to solder and
wire it: [assembly.md](assembly.md).*

![Power lite, top](../../docs/design-v6/figs/powerlite_top.png)

A 2-layer board, 97 × 64.5 mm, with through-hole parts only, soldered by hand. It carries:
- the pack's connectors;
- the solder pads for an off-the-shelf 3S BMS and for the main switch;
- the fuses;
- a relay that the E-stop drops;
- the servo distribution;
- the signals to the ESP32.

## How power flows

![Power lite, circuit](../../docs/design-v6/figs/powerlite_circuit.png)

*The circuit as a wired drawing: `tools/draw_circuit.py`, with the same parts and nets as
`tools/parts.py`. The KiCad schematic carries the same nets as labels on every pin.*

```
pack XT60 (J1) ─ BMS (W1-W5, cuts the negative lead) ─ main switch (W6/W7) ─┬─ F2 3 A ─ J3 ─ General Driver H1 (and the Pi)
pack balance lead (J2) ─ BMS cell taps                                      ├─ F2 3 A ─ J11 E-stop NC ─ K1 coil ─ Q1 ─ GND
                                                                             └─ F1 15 A ─ K1 contact ─ SERVO+ ─ J4-J7
```

- **The BMS** is a ShillehTek CF-3SJH25A-C ($6.99, ships from the US).
  - It sits on wires: its B−, B3, B2 and B1+ pads go to W1, W3, W4 and W5, and its P− pad to
    W2.
  - It switches the negative lead, so the system ground is its P−. The pack's balance lead
    goes only to the BMS, through J2.
  - Its thresholds are not documented, so it is characterised on the bench before it goes in
    the robot (below).
- **The main switch** is a 20 A panel toggle on W6 and W7, in the power bay's switch pod
  (#213). It is marked 20 A 125 VAC, with no DC rating: carrying 15 A is within it, but
  breaking DC under load is not rated. So turn it off only after the E-stop has dropped the
  servo rail, and it breaks only the logic's 1 A or so.
- **The servo rail** is fed through the TE 1-1414610-0 relay (K1; 70 A, operates at 7.2 V).
  - The relay's coil current, about 0.13 A, flows only while the E-stop's NC contact (J11) is
    closed and the ESP32 holds `SERVO_EN` high.
  - Pressing the E-stop drops the rail without firmware.
  - Until the ESP32 drives `SERVO_EN`, R2 holds Q1 off and the rail stays off.
- **The coil:** D1 clamps its turn-off. The relay's built-in 470 Ω alone would put about 63 V
  on Q1 (rated 40 V), and about −66 V on the E-stop and Q2's base when the E-stop opens.
  - TE warns that a coil diode slows the release and shortens the contact life (V23134 notes
    4 and 6).
  - The relay switches a few times a session, far inside its 10⁵-operation life, and the
    simulated release still takes only 5.3 ms.
- **The General Driver and the Pi** are fed through F2 and stay up through an E-stop.
- **Surge:** D2 and D3 (1.5KE16A) clamp the servo rail and the switched input. C1 (1000 µF)
  is the servo bulk.

## Connectors

| ref | what | part | pins |
|---|---|---|---|
| J1 | the pack | Amass XT60PW-M | 1 B− (angled side), 2 B+ |
| J2 | the pack's balance lead | JST B4B-XH-A | 1 B−, 2 cell 1+, 3 cell 2+, 4 B+ |
| J3 | General Driver H1 | JST B2B-XH-A | 1 LOGIC (+), 2 GND; the lead runs pin 1 to pin 1 |
| J4–J7 | servo branches: R leg, R upper, L leg, L upper | Molex 22-03-5035 | 1 data, 2 SERVO+, 3 GND (the General Driver's H5 order, "D V G") |
| J8 | servo data from the General Driver's H5 | Molex 22-03-5035 | 1 data, 2 unconnected, 3 GND |
| J11 | the E-stop's NC contact | JST B2B-PH-K-S | 1 LOGIC, 2 to K1's coil; any latching switch (0.13 A) |
| J12 | control to the ESP32 | JST B8B-PH-K-S | 2 `SERVO_EN` (IO16), 3 GND, 5 `RAIL_V` (GPIO34), 6 `ESTOP_N` (IO4, with its internal pull-up on); 1, 4, 7, 8 unconnected |
| W1–W5 | the BMS | wire pads | W1 B−, W2 P− (system GND), W3 B3, W4 B2, W5 B1+ |
| W6/W7 | the main switch | wire pads | W6 pack B+, W7 switched (VIN) |

- **J1's plug faces out of the left edge.** The XT60's mating face overhangs the edge by 0.25 mm,
  with its pins 16.1 mm in, as on the custom board. Ask the board house for no rail or panel on
  that edge.
- **J3 and J11 are different families** (XH and PH), so the two 2-pin leads cannot swap.
- **The ESP32's readings:**
  - `RAIL_V` is SERVO+ divided by 5.7, giving 2.21 V at 12.6 V.
  - `ESTOP_N` is an open collector (Q2), read by IO4 with its internal pull-up on.
    - It reads low while the E-stop is released, because Q2 conducts while the coil supply is up.
    - It reads high when the E-stop is pressed, the board is unpowered, F2 has blown, or the
      lead is unplugged. Every failure reads as stop.
    - The levels and polarity are rev B's ESTOP_N's.
  - C2 (100 nF) filters `RAIL_V` for the ADC.
- **J3 and J4–J8 keep the custom board's pin orders**, so the harness in
  [`../power-board/harness.md`](../power-board/harness.md) applies (checks P2–P4).

## The firmware

A robot with this board runs its own image: `make firmware POWER_LITE=1`, in
`build/firmware-powerlite` (it combines with `POLICY=`). It is a build option because on
a robot without the board IO4 is the WS2812 net and IO16 goes nowhere, and a floating
`ESTOP_N` must read as stop, so nothing can detect the board at run time: this image
would never arm that robot. A default image does not touch these pins.

The logic is `firmware/components/railguard` (its header has the derivations), and the
hardware half `firmware/main/power_rail.cpp`.

- **Arming.** `ESTOP_N` must be low, and the rail must have fallen below 8 V since it was
  last on. `SERVO_EN` goes high; `RAIL_V` must pass 8 V within 250 ms; the servos get
  500 ms to boot. Only then is the bus used. On a timeout `SERVO_EN` drops and the fault
  is reported.
- **The servos are off until something asks.**
  - An arm (`run`, an ARM edge) powers the rail itself. So does `home` or the wireless
    reset, which turn it off again when the home has released torque.
  - Bench work (`scan`, `pos`, `cal`, `pose`) needs `rail on` first, over the tether.
    Without it those commands say so. Scripts in `tools/` that talk to the servos need it
    too.
  - No servo is read at boot: the gain check is the arm's, with the rail up, and it
    re-applies the servos' RAM settings, which a power cycle loses.
- **`SERVO_EN` drops on:** a disarm (`bench`, ARM off), a hard operator stop in any mode,
  the link's RELAX and `ESTOP`, the pack guard's safe state, a fall, `ESTOP_N` going high,
  and the rail collapsing below 7 V for 500 ms. Each of those ends a run: `RAIL_DOWN` in
  the beacon, or `DISARMED_FALL` / `DISARMED_STOP`. A flail-guard or soft-stop hold keeps
  the rail, because it holds with torque on, until the bench home has released it.
- **Nothing brings the rail back by itself.**
  - `ESTOP_N` going high latches. Back at RUN, the rail stays off and an arm is refused
    until a disabled command arrives: `bench`, or a frame with `ENABLE`, `ARM` and `ESTOP`
    clear. Then it takes a fresh arm. So: RUN, disarm, arm.
  - After a fall the rail stays off when the robot is picked up, and the bench's
    pick-up autohome does not run: `home`, the wireless reset or an arm is the way back.
- **A welded K1.** `RAIL_V` above 8 V with `SERVO_EN` low, for 8 s with no new low of
  150 mV, raises the stuck-rail fault. It also runs from the first sample after boot. A
  rail that is still falling never raises it, however slowly: without servos it takes
  2.9 s to reach 8 V through LED2, and 31 s if LED2 is open. The fault refuses arming, and
  `rail clear` is refused until the rail has fallen.
- **A reset.** A panic or a restart takes `SERVO_EN` low first: the linker wraps the panic
  handler and `esp_restart_noos`, because that CPU reset alone leaves the ESP32's GPIOs as
  they were. A brown-out is a full system reset, which leaves the pin to R2. The task
  watchdog panics in this image (in a default image it only warns), so a
  wedged control loop drops the rail within 1 s. The robot goes limp where it is, and comes
  back benched with the rail off. An ARM bit held high makes no edge, so it does not re-arm.
- **Telemetry.** `stat` and `rail` print the state, `SERVO_EN`, `RAIL_V`, `ESTOP_N`, the
  latch, the fault and the last drop's reason; every change prints and is a flight-recorder
  event (`rail`). The beacon carries only `RAIL_DOWN`; the fields themselves are #109's.
  `vbat` reads 0 with the rail off, and the pack guard takes that as no data.

### The bench check, with the ESP32

Bring-up step 2 below, the ESP32 in place of the 3.3 V on J12.2, with the lab supply at
12.0 V, 0.5 A, no BMS, no pack and no servos. Flash the power-lite image; film it from
before the port opens (AGENTS.md). Type on the tether:

1. **Boot.** `rail` reads OFF, `SERVO_EN` low, `RAIL_V` about 0 V, `ESTOP_N` low, no
   latch, no fault. J12.2 reads 0 V, and K1 did not click during the reset.
2. **Arm.** `rail on`: K1 clicks, LED2 lights, `rail` reads ON with `RAIL_V` at 12 V. Compare
   it with a meter on SERVO+ and on J12.5 (2.1 V); note the error. `rail off`: K1 drops; time
   how long `RAIL_V` takes to read below 8 V (2-3 s here). No fault within 30 s.
3. **E-stop.** `rail on`, then open J11: K1 drops, and the tether prints `rail: OFF
   (E-stop pressed)` and the latch, with `ESTOP_N` high. Close J11: K1 stays open.
   `rail on` is refused, naming the latch. `bench`, then `rail on`: K1 clicks.
4. **Unplugged lead.** Pull J12 from the ESP32's headers: `rail` shows `ESTOP_N` high and
   `rail on` is refused. Replug; RUN, `bench`.
5. **Stuck rail.** With `SERVO_EN` low, feed J12.5 about 2.1 V from a divider on the supply
   (or hold SERVO+ up from a second supply): `rail FAULT: STUCK RAIL` within 8-9 s. `rail on`
   is refused. `rail clear` is refused until the feed is removed; then it clears.
6. **No rail.** Pull F1, `rail on`: `SERVO_EN` drops within 250 ms and `rail` reports NO RAIL.
7. **Reset.** `rail on`, `rail panic`: K1 must drop as soon as the line prints, before the
   reboot banner ends. Press the EN button with the rail on: the same. Scope J12.2 through a
   power cycle and a reset: it never goes high.

Then, with the real servos, as part of step 3 below:

8. **Servo boot.** Time `rail on` and read `scan` straight after. If a servo is missing,
   lengthen `railguard::kServoBootMs` (500 ms, a guess until now). The whole of `run` must
   finish inside the console's 1.5 s `kArmSyncMs`, or the console drops ARM again.
9. **First arm.** On the stand, torque-off pose: `run` must not twitch. A freshly powered
   servo's goal register has never been written, and the arm writes targets within a tick
   of enabling torque.
10. **Tick cost.** `stat` armed: the rail's ADC reads land in `other`.

## Before it goes in the robot

1. **Check the BMS on the bench** (the exception to the datasheet rule, Tom 2026-11-02). Use
   the lab supply as the pack: its + to B1+, its − to B−. Feed B3 and B2 from a divider of
   three 1 kΩ resistors across the supply, so each "cell" is a third of the supply voltage.
   Load P+ to P− with about 1 kΩ, and record each value:
   - **Under-voltage trip:** lower the supply until P− lets go. The trip per cell is the supply
     voltage ÷ 3. Raise it again and record where it recovers.
   - **No trip at 12 A for 2 s:** supply at 12.0 V with a 15 A limit, and a 1 Ω load of 150 W or
     more switched across P+ and P− for 2 s.
   - **The short-circuit trip** needs a stiff source, a pack, and is not done on the bench. F1
     and F2 protect against shorts downstream.
2. **Bring up the board without the BMS or the pack.** Put the lab supply at 12.0 V, limited
   to 0.5 A, on W7 (+) and W2 (−).
   - With J11 jumpered: LED1 lights and J3 reads 12 V. J12.6 (`ESTOP_N`), pulled up to 3.3 V
     through 47 kΩ, reads below 0.1 V. J12.5 (`RAIL_V`) reads 0.
   - 3.3 V on J12.2 (`SERVO_EN`): K1 clicks, LED2 lights, and `RAIL_V` reads about 2.1 V.
   - Remove the J11 jumper: K1 drops, and `ESTOP_N` reads 3.3 V.
3. **Check the inrush with the real load.** Wire the BMS and the pack, with the servos
   connected. Switch the rail on with `SERVO_EN` ten times.
   - The BMS must not trip, and the relay must open every time the E-stop is pressed.
   - The simulation puts the peak at 176–341 A, above 100 A for 52–132 µs, depending on the
     servos' own capacitance (below).
   - If the BMS trips, the options are a smaller C1, or a pre-charge resistor in series with C1.

## Checks

- **Schematic:** ERC reports 0, and the netlist matches `tools/parts.py` (37 parts, 19 nets).
- **Board:** DRC reports 0 with schematic parity, and 0 unconnected pads.
- **Copper on the 15 A path:** the narrowest section is the 6.1 mm strip of `VIN` between W7 and
  F1, in 1 oz copper with the GND plane beneath it. The return pads (W2, each servo header's
  pin 3, C1, D2, D3) join the plane with full copper: four 0.5 mm thermal spokes would be 2 mm
  of copper carrying up to 15 A (`tools/post_pcb.py`).
- **Simulation:** `tests/test_power_lite_sim.py` runs in `make pytest` and passes 23 of 23. It
  uses ngspice, through `sim/lite_sim.py`, which builds the netlist from `tools/parts.py`. Its
  assumptions are listed in that file.

| case | result | limit |
|---|---|---|
| the rail, over pack 9.0–12.6 V × E-stop × `SERVO_EN` high, low or floating | on only with the E-stop released and `SERVO_EN` high | — |
| K1's coil at a 9.0 V pack | 8.77 V, 97.5 mA | operates at 7.2 V, 80 mA (TE) |
| Q1 saturation; the `SERVO_EN` pin's current | 0.17–0.23 V; 4.9 mA | ESP32: 20 mA |
| `ESTOP_N`, released / pressed | 0.007 V / 3.30 V | ESP32: low ≤ 0.83 V, high ≥ 2.48 V |
| `RAIL_V` at 9.0 / 12.6 V | 1.58 / 2.21 V | ADC ~3.1 V |
| LEDs | 1.5–2.3 mA | — |
| release after the E-stop opens, or `SERVO_EN` drops | 5.3 ms (coil inductance assumed 0.3 H) | — |
| clamps during release | COIL_HI −0.78 V; Q2's base −0.2 V; Q1's collector 11.8 V | Q1 VCEO 40 V; Q2 VEBO 6 V |
| a 12 A get-up at a 9.0 V pack | logic ≥ 8.71 V, SERVO+ ≥ 8.66 V, coil ≥ 94.8 mA | K1 holds |
| the relay closing onto the rail, with 0–2 mF of servo capacitance | 176–341 A at the contact; above 100 A for 52–132 µs; F1 I²t 1.3–7.8 A²s | F1 melts at 135 A²s |

- **The inrush is the open question.** It is set by the pack, BMS and wiring resistance and by
  the servos' own input capacitance, not by this board.
  - The pulse is shorter than a typical 3-cell protection IC's short-circuit delay (ABLIC
    S-8254A: 320 µs). But the ShillehTek board's IC is not known.
  - The peak is above the 200 A capacitive make that TE lists for the 24 V-coil part.
  - A welded contact would defeat the E-stop. The firmware's stuck-rail check (see "The
    firmware") reports it, and bring-up step 3 tests it with the real servos.

## Ordering (two robots)

The quantities include spares. Prices are Digi-Key's, as listed on 2026-11-02.
- **Boards:** `fab/power-lite-gerbers.zip`, 2 layers, 97 × 64.5 mm, 1.6 mm, 1 oz, HASL. Landed
  prices for 5 boards: about $32 from JLCPCB with the duty prepaid, about $51 from PCBWay, or
  $48.49 for 3 from OSH Park (made in the US, no duty). The quotes are in
  [the 2026-11-03 record](../../docs/design-v6/2026-11-03-power-lite-quotes.md).
- **BMS:** ShillehTek CF-3SJH25A-C, $6.99 each.
- **Already on hand:** the XT60PW-M connectors, the Keystone 3568 fuse holders, MINI blade
  fuses, the main-switch toggles, hook-up wire, and a JST PH kit (housings, contacts, 2–7 pin
  headers: J11's header and the PHR-2 housing come from it).

| part | Digi-Key | qty | each |
|---|---|---|---|
| relay TE 1-1414610-0 | 17-1-1414610-0-ND | 2 | $13.37 |
| E-stop: E-Switch 100SP1T1B1M1QEH miniature toggle (5 A @ 28 VDC), wired across terminals 2 and 3 | EG2350-ND | 3 | $3.08 |
| TVS 1.5KE16A | 1.5KE16ACT-ND | 5 | $1.06 |
| 1N4007 | 4786-1N4007CT-ND | 3 | $0.16 |
| 1000 µF 25 V EEU-FR1E102 | P14424-ND | 3 | $1.42 |
| PN2222ATA (Q1, Q2) | PN2222ATACT-ND | 10 | $0.32 |
| 100 nF 50 V X7R radial KEMET C322C104K5R5TA (C2) | 399-4329-ND | 6 | $1.49 |
| resistors Yageo 1 % ¼ W metal film: 470R, 10k, 47k, 4K7 | MFR-25FBF52-470R-ND, 10.0KXBK-ND, 13-MFR-25FRF52-47KCT-ND, 13-MFR-25FRF52-4K7CT-ND | 10 each (10k: 20) | $0.04 |
| LEDs WP710A10GD / WP710A10ID | 754-1603-ND / 754-1606-ND | 3 / 3 | $0.24 |
| headers B4B-XH-A, B2B-XH-A, B8B-PH-K-S | 455-B4B-XH-A-ND, 455-B2B-XH-A-ND, 455-1710-ND | 3–4 each | $0.10–0.35 |
| servo headers Molex 22-03-5035 | 900-0022035035-ND | 11 | $0.20 |
| housings XHP-2, PHR-8 and XH crimps SXH-001T-P0.6 | 455-2266-ND, 455-1189-ND, 455-1135-100-ND | 6, 3, 100 | |

- **The 47k and 4K7 resistors** are MFR-25FRF52: the MFR-25FBF52 values in `tools/parts.py`
  are out of stock at Digi-Key until 2027-03. The two series have the same rating, size and
  tempco.
- **Total:** the Digi-Key cart for two robots, with spares, was $60.22 before shipping, tariff
  and tax on 2026-11-04. The two relays are the largest part of it.

## Regenerating

The KiCad files are generated: edit the tools, not the files.
1. Edit `tools/parts.py`, the parts and the net of every pin.
2. Run `python3 tools/gen_sch.py` to draw the schematic.
3. Run `FREEROUTING_JAR=<freerouting-2.x.jar> python3 tools/route.py <work dir>`. It checks the
   netlist against `parts.py`, places the parts (`tools/gen_pcb.py`), routes with Freerouting,
   adds the GND plane and the silkscreen (`tools/post_pcb.py`), and runs DRC.
4. Run `sh fab.sh` to write the fab outputs, after ERC and DRC.

The relay's footprint (`power-lite.pretty/TE_PowerRelay_F7_PCB`) is drawn from TE's V23134
datasheet; the XT60's is the custom board's.

![Power lite, populated](../../docs/design-v6/figs/powerlite_3d.png)

Bodies drawn by `tools/gen_3d.py` (in `power-lite.3dshapes/`) stand in where this KiCad has no 3D
model, or where the part is not a footprint:
- the XT60, the relay and the Mini-SPOX servo headers, from the makers' outlines;
- the blade fuses in their holders, 15 A light blue and 3 A violet;
- wire stubs on the W pads, for the BMS and switch leads.

The other parts use KiCad's stock models; LED1 uses the green one.

![Power lite, bottom](../../docs/design-v6/figs/powerlite_bottom.png)

## Open

- **The BMS bench check** (above).
- **The torso mounting:** the power bay (#213) holds the board on the torso's aft bosses, the
  BMS on a bracket over its +y end, and the switches on the cap.
