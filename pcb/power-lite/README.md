# Power lite: the robot's battery protection and power distribution

*The choices behind it are in
[2026-11-02-power-lite-options.md](../../docs/design-v6/2026-11-02-power-lite-options.md). It
replaces the custom [power board](../power-board/README.md) for the robot.*

![Power lite, top](../../docs/design-v6/figs/powerlite_top.png)

A 2-layer board, 97 × 64.5 mm, with through-hole parts only, soldered by hand. It carries:
- the pack's connectors;
- the solder pads for an off-the-shelf 3S BMS and for the main switch;
- the fuses;
- a relay that the E-stop drops;
- the servo distribution;
- the signals to the ESP32.

## How power flows

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
- **The main switch** is a Cole Hersee 57000-01 panel rocker (25 A DC), on W6 and W7.
- **The servo rail** is fed through the TE 1-1414610-0 relay (K1; 70 A, operates at 7.2 V).
  - The relay's coil current, about 0.13 A, flows only while the E-stop's NC contact (J11) is
    closed and the ESP32 holds `SERVO_EN` high.
  - Pressing the E-stop drops the rail without firmware.
  - Until the ESP32 drives `SERVO_EN`, R2 holds Q1 off and the rail stays off.
- **The coil:** D1 clamps its turn-off. The relay's built-in 470 Ω alone would put about 63 V
  on Q1.
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
| J12 | control to the ESP32 | JST B8B-PH-K-S | 2 `SERVO_EN` (IO16), 3 GND, 5 `RAIL_V` (GPIO34), 6 `ESTOP_N` (IO4); 1, 4, 7, 8 unconnected |
| W1–W5 | the BMS | wire pads | W1 B−, W2 P− (system GND), W3 B3, W4 B2, W5 B1+ |
| W6/W7 | the main switch | wire pads | W6 pack B+, W7 switched (VIN) |

- **J3 and J11 are different families** (XH and PH), so the two 2-pin leads cannot swap.
- **The ESP32's readings:**
  - `RAIL_V` is SERVO+ divided by 5.7, giving 2.21 V at 12.6 V.
  - `ESTOP_N` is high, about 2.2 V, while the E-stop is released and F2 is good.
- **J3 and J4–J8 keep the custom board's pin orders**, so the harness in
  [`../power-board/harness.md`](../power-board/harness.md) applies (checks P2–P4).

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
   - LED1 lights, J3 reads 12 V, J12.6 (`ESTOP_N`) reads about 2.2 V with J11 jumpered, and
     J12.5 (`RAIL_V`) reads 0.
   - 3.3 V on J12.2 (`SERVO_EN`): K1 clicks, LED2 lights, and `RAIL_V` reads about 2.1 V.
   - Remove the J11 jumper: K1 drops, and `ESTOP_N` reads 0.
3. **Check the inrush with the real load.** Wire the BMS and the pack, with the servos
   connected. Switch the rail on with `SERVO_EN` ten times; the BMS must not trip. If it does,
   the options are a smaller C1, or a pre-charge resistor in series with C1.

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

![Power lite, bottom](../../docs/design-v6/figs/powerlite_bottom.png)

## Open

- **The BMS bench check** (above).
- **The E-stop switch:** any latching switch with an NC contact rated 0.5 A at 12 V or more.
- **The torso mounting:** the four M2.5 holes sit 3.2 mm in from the corners, as on the custom
  board (#108). The BMS (56 × 45 mm) and the switch mount in the torso on their wires.
