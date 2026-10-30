# Power board: cables and connectors

How the power board connects to the rest of the robot, connector by connector and pin by pin.
The board is [README.md](README.md); the robot's other wiring is in [docs/wiring.md](../../docs/wiring.md)
and the General Driver's headers are in [docs/sensor-expansion.md](../../docs/sensor-expansion.md).

Every cable here is plugged, not soldered: board-side headers mate stock leads or housings
filled with pre-crimped leads. The seven checks marked **P1–P7** need the real parts and are
done before the board is ordered (see the end of this page).

## Summary

| board | to | board part | mating part | current |
|---|---|---|---|---|
| J1 | the pack's discharge lead | Amass XT60PW-M (right-angle, male, 45 A) | the pack's XT60 female | ≤ 15 A fused, 20 A trip |
| J2 | the pack's balance lead | JST B4B-XH-A | the pack's XHP-4 | µA |
| J1 + J2 | a bench supply instead of the pack | as above | the supply tether (below) | as the pack |
| J3 | General Driver H1 (logic power), through F2 | JST B2B-XH-A | XHP-2 ↔ XHP-2 lead, 22 AWG | ≤ 1.1 A with a Pi 4B, ≤ 1.3 A with a Pi 5 (verification record C14) |
| J4–J7 | first servo of each branch | Molex 22-03-5035 (5267 header) | stock servo lead (5264-03 both ends) | branch peak 3.7–4.3 A |
| J8 | General Driver H5 (servo data) | Molex 22-03-5035 | stock servo lead | mA |
| J9 | General Driver P1 (I²C, 3V3) | JST B4B-PH-K-S | PHR-4 ↔ PHR-4 lead | mA |
| J12 | General Driver P3, H3, H2 (control) | JST B8B-PH-K-S | PHR-8 to three housings | mA |
| J10 | POWER button | JST B2B-PH-K-S | PHR-2 lead to the button | mA |
| J11 | E-STOP button | JST B4B-ZR (ZH, 1.5 mm) | ZHR-4 lead to the button | ≤ 1.3 mA |

## J1: the pack

- **Board:** Amass XT60PW-M, board-mount right-angle **male**. Amass's spec sheet (V1.2, in
  `docs/datasheets/power-board/amass-xt60pw.pdf`) gives 45 A rated and 60 A momentary, and
  lists its use as "CONTROLLER/CHARGER". Its female twin, XT60PW-F, is listed for "BATTERY".
  The mating face is flush with the board's left edge.
- **Pins:**
  - Pin 2, the contact on the housing's straight side, is **BAT+**.
  - Pin 1, on the angled (chamfered) side, is **BAT−**.

  KiCad's footprint marks them the same way, and the silkscreen carries + and − beside the
  pads.
- **Pack:** OVONIC 4200 mAh 3S 130C (O-130C-4200-3S1P-XT60-2P), ordered 2026-10-07: 12 AWG
  leads, an XT60 and a JST-XHR-4P balance plug, 96 × 44.5 × 25.5 mm, 246 g. Neither Ovonic's
  page nor the listing states the XT60's gender, the polarity or the lead length. **P1**:
  check the pack.
- **Lead:** the pack's own discharge lead plugs in directly, with no extension.
  - The lead's inductance and the pack's internal resistance set how hard a PACK+ short hits
    Q2.
  - A stiff pack with a long lead makes that short harder on Q2 (verification record C13). **P7**
    measures the resistance.
  - If the torso (#108) puts the pack out of reach, move the board, not the pack.

## J2: the balance lead

- **Board:** JST B4B-XH-A.
- **Pins:** 1 B− (0 V), 2 cell 1+, 3 cell 2+, 4 B+.
- **Lead:** the pack's 4-pin XH balance plug mates directly.
- **P2:** measure the pack's balance plug before first connection. Taking the contact that
  mates pin 1 as 0 V, the others must read about 3.7, 7.4 and 11.1 V in order.

## J1 and J2 from a bench supply: the supply tether

The robot can run from a bench supply instead of the pack, through one adapter cable, the
supply tether. (It is not the USB-C tether of [docs/control-channel.md](../../docs/control-channel.md),
which carries the console; the two are used side by side on the bench.)
The BQ76922 still guards the board, so it has to see three cells. On the XT60 alone it
sees none: it keeps Q2 open and shuts itself down 1.25 s later (simulated,
`tether_no_adapter_refuses`). The runs are in the simulation record, §4.6.

- **The adapter.**
  - An XT60 female on two 14 AWG leads, twisted together, at most 2.5 m long, to the
    supply's + and −.
  - At the XT60 end, an XHP-4 for J2 wired as a balance lead: pin 1 to the − lead and
    pin 4 to the + lead. Pins 2 and 3 come from a divider of three 1 kΩ resistors
    between them, so each "cell" reads a third of the supply. Each resistor takes
    under 50 mW at 12.6 V.
- **The supply setting.** 12.0 V, which is 4.0 V a cell; anywhere from 9.5 to 12.4 V
  works.
  - At 12.6 V the BQ76922's cell over-voltage limit (4.2 V) trips. PACK+ stays up, but
    the charge FET switches on and off.
  - Under about 9 V the BQ76922's stack limit (9.0 V) or CUV (8.96 V) cuts the board.
- **The supply current.** 15 A or more. A get-up draws 12 A for 2 s; at 12 V through the
  supply tether, PACK+ held at 11.26 V (simulated). A smaller supply drops into constant
  current, PACK+ sags, and the BQ76922 cuts the board as above.
- **The lead.** Twist it. As two loose wires (about 1 µH each, estimated from the
  geometry), a hard short on the board's PACK+ copper takes Q2 to 31.3 V when SCD opens
  it, against its 30 V rating. Twisted (about 0.5 µH each) it sees 24.7 V.
- **Plug order.** XT60 first, then J2; unplug J2 first. That is the pack's order too:
  until rev B ties VC0 to VSS (F-9), the BQ76922 does not support the balance lead going
  in first.
- **Not simulated: energy the servos push back.** A pack absorbs the current that servos
  push into SERVO+ when they brake or are back-driven. Most bench supplies cannot sink
  current, so that energy charges C21, the servos' own capacitors and the supply's
  output capacitors instead.
  - The ST3215 stops driving above 14 V (its datasheet, §7-11: protection above 14 V or
    below 4 V, cleared when the voltage returns). C21 alone holds only about 26 mJ
    between 12 and 14 V. D9 clamps SERVO+ only from 16.7 V.
  - **T1, before the first motion on the supply tether:** scope SERVO+ while back-driving a joint by
    hand with torque on, and during a sit-down. If it passes about 13.5 V, run on the
    pack, or add a supply that can sink (or a load across its output).

## J3: logic power to the General Driver

- **Board:** JST B2B-XH-A on the top edge. Pin 1 is `LOGIC_OUT`, PACK+ through F2; pin 2 is
  GND.
- **F2** is a 3 A MINI blade fuse (Littelfuse 0297003.WXNV) in a holder beside J3, pulled and
  replaced by hand. Keep spares. A short on this cable can blow it (verification record C13).
- **Cable:** XHP-2 to XHP-2, pin 1 to pin 1, 22 AWG, as short as the torso allows. The General
  Driver's H1 takes conductors of 22–30 AWG at 3 A per contact.
- **P3:** H1 is silkscreened "− +". Confirm with a meter that H1 pin 1 is +, through to the
  AO4407.

## J4–J8: servo branches and the data line

**Board:** Molex 22-03-5035 (5267 Mini-SPOX header).

**Pins:** 1 `SERVO_DATA`, 2 SERVO+, 3 GND. That is the General Driver's H5 order ("D V G").

**Leads:** stock servo leads, with a 5264-03 housing at each end.

| header | branch | first servo, then the chain | walking peak / RMS (upper model) | get-up peak (range over the two sides) |
|---|---|---|---|---|
| J4 | right leg | 9 → 1, 2, 3, 4, 11 | 3.7 A / 1.6 A | 3.0–4.3 A |
| J5 | right upper | 13 → 14, 15 | 0.3–0.4 A / 0.1 A | 2.9–3.0 A |
| J6 | left leg | 10 → 5, 6, 7, 8, 12 | 3.7 A / 1.6 A | 3.0–4.3 A |
| J7 | left upper | 16 → 17 | 0.3–0.4 A / 0.1 A | 2.9–3.0 A |
| J8 | data in from General Driver H5 | pin 2 unconnected on the board | | |

- **Contact rating:** a 5264 contact is rated about 2.5–3 A, which is not yet read from
  Molex's datasheet. The leg branches' RMS current is under that; their peaks exceed it for
  moments. Bench check V8 measures the first lead's temperature.
- **P4:** confirm that a stock lead seats in J4–J8 with pin 1 on the same wire as in H5. Plug
  one lead into H5 and note which wire is GND; the same wire must land on pin 3 here.

## J9 and J12: control to the General Driver

**J9, I²C.** JST B4B-PH-K-S. Its pin order is the General Driver's P1 order (1 3V3, 2 GND,
3 SDA, 4 SCL), so the cable is a straight PHR-4 to PHR-4 lead, if P1 is a PH 2.0 header
(**P5**).

**J12, control.** JST B8B-PH-K-S, fanned out to three headers:

| J12 pin | net | to | ESP32 |
|---|---|---|---|
| 1 | KILL | P3 pin 5 | IO27 |
| 2 | SERVO_EN | P3 pin 4 | IO16 |
| 3 | GND | P3 pin 3 | GND |
| 4 | PWR_INT_N | H3 pin 3 | GPIO 35 (input only) |
| 5 | RAIL_V | H3 pin 4 | GPIO 34 (ADC1) |
| 6 | ESTOP_N | H2 pin 1 | IO4 |
| 7 | SERVO_IMON | not wired | bench probe |
| 8 | SERVO_FLT_N | not wired | bench probe |

- **Unused header pins:** H3 pins 1 and 6 are motor A outputs, and H2 pin 2 is 5 V. None of
  them is wired.
- **Unwired signals:** the servo current comes from pack current minus INA219 current. A
  servo-switch trip shows as RAIL_V ≈ 0 with SERVO_EN high and the E-stop released.
- **P5:** the connector families of P1, P3, H3 and H2. The photos show P1 as a PH-style
  header; P3 and H2 look like 2.54 mm pin headers. The J12 lead is a PHR-8 at the board end,
  with pre-crimped leads into whichever housings those headers take.

## J10 and J11: the buttons

**J10, POWER.** JST B2B-PH-K-S. A momentary normally-open button between pin 1 (`PWR_BTN`)
and pin 2 (GND). The part is still to be chosen, from a maker with a datasheet.

**J11, E-STOP.** JST B4B-ZR (ZH, 1.5 mm pitch; 1 A, 50 V). No other lead on the board is a
ZH, so neither the I²C lead (PH 4-pin) nor the balance lead (XH 4-pin) can go in here. The
lead is a ZHR-4 housing with pre-crimped SZH leads (AWG 26–32). A **latching** button with
**two normally-closed contacts**; the part is still to be chosen.

| contact | pins | function |
|---|---|---|
| A | 1 (`ESTOP_FEED`, PACK+ through 10 kΩ) – 2 (`ESTOP_RET`) | feeds the servo switch's EN; open = servo rail off |
| B | 3 (`ESTOP_N`) – 4 (GND) | open = E-stop pressed, read by the ESP32 |

Pins 1–2 are at pack voltage behind 10 kΩ (1.26 mA, 16 mW if shorted).

## Checks before ordering

| | check | how | why |
|---|---|---|---|
| P1 | the pack's XT60 is female, with + on the housing's straight side | look and meter the pack lead | a reversed J1 reverses the whole board |
| P2 | balance-plug order | meter the plug: 0, 3.7, 7.4, 11.1 V from the pin-1 side | J2 pin 1 must be B− |
| P3 | General Driver H1 pin 1 is + | meter to the AO4407 | J3 is wired pin 1 to pin 1 |
| P4 | H5 pin order relative to its key | meter a stock lead in H5 | J4–J8 copy H5 |
| P5 | the families of P1, P3, H3 and H2 | look, and measure the pitch | picks the J9 and J12 housings |
| P6 | the stock servo lead's wire gauge | read the insulation or measure it | branch current against the lead's rating |
| P7 | the pack's internal resistance, charged, leads and plug included | a charger's IR reading per cell, or ΔV/ΔI under a known load | calibrates the short-circuit model; under about 15 mΩ a hard short on the board's PACK+ copper exceeds Q2's pulse rating, a risk accepted in D-FET (verification record C13) |
