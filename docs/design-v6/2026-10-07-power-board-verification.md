# Power board rev A: verification before ordering (2026-10-07)

*A dated record. The board is [pcb/power-board](../../pcb/power-board/README.md), its cables
[harness.md](../../pcb/power-board/harness.md), and the design
[2026-10-06-power-circuit.md](2026-10-06-power-circuit.md). This record covers what was
checked on the drawings, in simulation and against the makers' documents before any board is
ordered: the method, the result, and what each finding changed. Tom's rule for this stage:
finalize the design before asking for quotes, because the cost of an error is a board that
has to be made again.*

## Summary

| | check | result |
|---|---|---|
| C1 | ERC, DRC, netlist against the design spec | pass: ERC 0, DRC 0 violations and 0 unconnected, all 65 nets match pin for pin |
| C2 | high-current copper: each pour one piece over all its pads; the BAT− island's net; Kelvin lines touch only their trace | pass, after fixing a generator bug the check caught (C2 note) |
| C3 | servo-rail switch values against TI's TPS4811-Q1 design calculator (SLURB09) | pass |
| C4 | servo-rail soft start, simulated | fixed by **change 1** |
| C5 | hard short on the live servo rail, simulated | pass |
| C6 | hard short on PACK+, simulated | fixed by **change 2**; re-run with the sense filter in the model |
| C7 | logic levels between the board and the ESP32 | fixed by **change 3** |
| C8 | footprints against the makers' drawings | fixed by **changes 4–6**; J1's polarity was reversed, **change 5** |
| C9 | the control connector against the General Driver's free pins | fixed by **change 7** |
| C10 | every part in stock with a maker's datasheet | pass, except three datasheets the makers would not serve |
| C11 | copper current and heating | pass at 1 oz or 2 oz outer copper |
| C12 | cables and connectors | defined in harness.md; seven physical checks need the real parts |
| C13 | the pack ordered on 2026-10-07 (OVONIC 4200 mAh 3S 130C) | compatible in use; a hard short on the board's PACK+ copper exceeds Q2's pulse rating if the pack is under ~15 mΩ, accepted (D-FET) |
| C14 | a Raspberry Pi 5 on the logic rail | no board change; runtime 48 min (upper model) |

## C2. Copper and the BAT− island

`pcb/power-board` was checked by script:
- every power pour fills as one piece covering all its pads;
- R7.1 and R8.1, the SRP/SRN Kelvin ends, each touch exactly one trace and no via;
- the In2 island under the BQ76922 is on BAT−.

The island check caught a bug in the layout script. The island took its net from "J1 pad 2",
which became BAT+ when J1's polarity was corrected (change 5). The island is now named BAT−
explicitly, and the check stays in the script.

## C3. Servo-rail switch against TI's calculator

| quantity | parts | value |
|---|---|---|
| over-current threshold | R24 2 mΩ, R25 100 Ω, R34 30.1 kΩ | 19.8 A |
| short-circuit threshold | R26 4.64 kΩ | 39.3 A |
| fault timer | C19 680 nF, latch-off by R35 100 kΩ | 10.5 ms (FLT at 9.7 ms) |
| current monitor | R36 9.1 kΩ (TI's maximum for 3.3 V: 9.27 kΩ) | within range |
| bootstrap capacitor | C17 ≥ Qg + 10 × C16 (TI's rule for gate-slew start) | 4.7 µF ≥ 3.33 µF with change 1 |
| EN/UVLO | R30 560 kΩ / R31 100 kΩ | 7.79 V rising, 7.39 V falling |
| sense-resistor dissipation at 19.8 A | R24 | 0.78 W |

## C4. Servo-rail soft start

**Model.**
- **Simulator:** ngspice 47 with TI's CSD17556Q5B transient model (SLPM066).
- **Controller:** modelled behaviourally. At enable, PU drives the gate network to SRC + 12 V
  through R27.
- **Supply:** the pack at 11.4 V behind 35 mΩ.
- **Load:** C21 1000 µF and C22; the servos' input capacitance, swept because no servo
  document gives it; the servo electronics at 0.5 A above 5 V; the 57 kΩ rail divider.

| C16 | servo capacitance | peak current | peak FET power | rail at 90 % |
|---|---|---|---|---|
| 150 nF (before) | 2 mF | 2.45 A | 20.6 W | 18.6 ms |
| 150 nF (before) | 4 mF | 3.73 A | 33.1 W | 18.8 ms |
| 330 nF (change 1) | 2 mF | 1.40 A | 9.6 W | 40.2 ms |
| 330 nF (change 1) | 4 mF | 1.99 A | 15.7 W | 40.5 ms |

The inrush never approaches the 19.8 A trip; the FET's safe-operating area is the limit. TI's
Figure 10 (SLPS392D, TC = 25 °C) allows about 3–4 A of DC at 10 V. Before change 1, a 4 mF
servo load sat on that line; after it, the peak is half that.

## C5. Hard short on the live servo rail

**Model:** the TPS48111-Q1's short-circuit trip at 78 mV on R24 (39 A) after its 1.6 µs
maximum delay, then PD at 1.34 Ω to SRC; a short of 10 mΩ + 100 nH, with 2 mF on the rail.

| C16 | pack wiring | peak Q3 current | Q3 off after | Q3 energy | peak PACK+ |
|---|---|---|---|---|---|
| 330 nF | 200 nH | 69 A | 5.9 µs | 0.09 mJ | 19.5 V |
| 330 nF | 500 nH | 58 A | 8.3 µs | 0.14 mJ | 19.2 V |

**What it shows:**
- C16 does not hold the gate up after PD fires.
- The pack current peaks at 56–66 A, so the BQ76922's short-circuit threshold goes above
  it: 150 mV on R1 (150 A), fastest delay. That way a servo-lead short leaves the robot
  powered.

## C6. Hard short on PACK+

**Model:**
- **The trip:** the BQ76922's short-circuit detector at 150 A with the fastest delay (TRM 5.2.6).
  It reads SRP−SRN behind the board's sense filter: R7 and R8 at 100 Ω, C9 100 nF, C10 100 pF.
  That is TI's recommendation (datasheet 8.2), with a 20 µs differential time constant.
- **The turn-off:** the DSG driver discharges the gate toward PACK+ (datasheet 8.5) through
  R20 + D4.
- **The FETs:** Q1 and Q2, with TI's model.
- **The pack:** 11.4 V behind its resistance (cells, leads and plug) and its lead inductance.
  The design assumed a 30C pack, modelled as 35 mΩ. C13 covers the pack actually ordered.
- **The PACK+ side:** D8, C12–C14 and the General Driver's input.
- **The short:** 10 mΩ + 100 nH.

The first pass of this check fed the detector the unfiltered shunt voltage, so it tripped
about 20 µs early. The rows below are re-run with the filter. With a 35 mΩ pack the current
has already levelled off at about 215 A by the trip, so the results moved little: at 500 nH,
19.9 → 20.0 V and 22.6 → 24.8 mJ. With a stiffer pack the filter matters (C13).

| R20 | pack-side wiring | Q2 peak current | Q2 peak V(DS) | Q2 energy |
|---|---|---|---|---|
| 1 kΩ (before; unfiltered model) | 500 nH | | **31.3 V** | 14.0 mJ |
| 4.7 kΩ (change 2) | 200 nH | 218 A | 14.7 V | 17.4 mJ |
| 4.7 kΩ (change 2) | 500 nH | 215 A | 20.0 V | 24.8 mJ |
| 4.7 kΩ (change 2) | 1 µH | 210 A | 28.6 V | 36.6 mJ |

**What it shows:**
- **The spike.** When Q2 opens, the pack lead's inductance drives the battery side of the
  FETs up. TI's datasheet (8.5) gives the remedy: a 1 kΩ or 4.7 kΩ gate resistor "will
  reduce ... the inductive spike".
- **The energy.** At 500 nH, Q2's 24.8 mJ is spread over the 46 µs from the trip to below
  1 A: about 0.5 kW on average.
  - Figure 10 (SOA, TC = 25 °C), read at 20 V, allows about 85 A (1.7 kW) for 100 µs and
    more than 500 A for 10 µs.
  - The FET's ratings are 400 A pulsed (≤ 100 µs) and 30 V.
- **The TVS alternative.** A TVS on the battery side clamps the spike whatever the lead
  length. It was not added (decision D-TVS below).

## C7. Logic levels

| signal | drive | receiver | result |
|---|---|---|---|
| KILL → RST_SHUT | 3.3 V through R12 1 kΩ / R11 10 kΩ = 3.0 V | V(IH) 0.67 × REG18 ≈ 1.2 V, 5.5 V max | pass |
| SDA, SCL | 3.3 V | 5.5 V max, V(IH) ≈ 1.2 V | pass |
| SERVO_EN → INP | 3.27 V through R32 / R33 | V(INP_H) 2 V max | pass |
| POWER → TS2 wake | TS2 is 4.5–6 V through 5 MΩ in SHUTDOWN; the button pulls it through R9 and D2 to ≈ 0.15 V | V(WAKEONTS2) 0.7 V min | pass |
| POWER → PWR_INT_N | ≈ 0.25 V through D2 | ESP32 V(IL) 0.83 V | pass |
| RAIL_V → GPIO 34 | R38 47 kΩ / R39 10 kΩ: 12.6 V → 2.21 V (change 3; 33 kΩ gave 2.93 V) | ESP32 ADC at 11 dB: effective range 150–2450 mV (ESP32 datasheet) | pass |

## C8. Footprints

- **Q1–Q3 (CSD17556Q5B).**

  | pad | KiCad TDSON-8-1 (before) | TI's pattern (SLPS392D §7.2) |
  |---|---|---|
  | source and gate pads | 0.85 × 0.50 mm | 1.372 × 0.710 mm |
  | drain pad | 4.55 × 4.41 mm | 4.44 × 4.52 mm, with three notches |

  The FETs now sit on TI's pattern, with TI's stencil (change 4).
- **J1.**
  - **Wrong footprint.** KiCad's `AMASS_XT60-M … Vertical` is the cable connector's
    footprint, 4.5 mm holes for solder cups. J1 is now the board-mount Amass XT60PW-M:
    right-angle, 45 A, with a hole pattern that matches Amass's spec sheet (change 6).
  - **Reversed polarity.** Amass's front view shows the angled-side contact on the
    footprint's pad 1. By the XT60 convention that contact is negative, and KiCad's footprint
    marks pad 1 "−" and pad 2 "+". The schematic had pin 1 = BAT+, so it would have reversed
    the pack. Change 5 puts BAT− on pin 1 and BAT+ on pin 2; physical check P1 confirms it
    on the pack.
- **D1.** The original BAS40W-7-F is a 3-pin SOT-323 by Digi-Key's description, so the
  schematic's 2-pad SOD-323 footprint did not fit it; it was also backordered. The in-stock
  Nexperia PMEG4002EJ is a SOD-323F part, on KiCad's SOD-323F footprint (change 6).

## C9. Control connector

**The problem.**
- The General Driver's free ESP32 pins: IO27 and IO16 (P3), GPIO 35 and 34 (H3, input-only,
  the only free ADC1 pins), and IO4 (H2). IO5 is reserved for the Pi-halted line.
- The 12-pin J9 had no in-stock part.
- J9 also carried SERVO_IMON and SERVO_FLT_N, which have no free pin.

**Change 7.**
- **J9:** a 4-pin I²C connector in the General Driver P1's pin order.
- **J12:** an 8-pin control connector. IMON and FLT stay on it for bench probes, unwired in
  the harness. Servo current is inferred as pack current minus INA219 current, and a trip
  as RAIL_V ≈ 0 with SERVO_EN high.

## C10. Stock and documentation

**Stock.** A parts pass on 2026-10-07 moved every part to an in-stock Digi-Key listing (≥ 50
units) with a maker's datasheet in `docs/datasheets/power-board/`:
- **Resistors and capacitors:** given specific MPNs.
- **R1:** CSS2H-2512R-1L00FE.
- **R24:** CSS2H-2512**K**-2L00F, the real 2 mΩ part number.
- **D1:** PMEG4002EJ.
- **Q4:** Vishay 2N7002K-T1-GE3.
- **F2:** Littelfuse 0297003.WXNV in a Keystone 3568 holder (change 8).
- **J9/J12:** JST B4B-PH-K-S and B8B-PH-K-S.

**Not at Digi-Key:** the XT60PW-M is at LCSC and TME.

**Datasheets still missing:** Molex 22-03-5035, Panasonic EEU-FR1E102 and Littelfuse
0297015. Their makers' sites refused the download.

## C11. Copper current and heating

**Method.** Each power pour was solved on its actual fill: the board's polygons rasterised at
0.1 mm, Laplace with the pads as electrodes, and B.Cu copies taken in parallel.

| path | 1 oz | 2 oz |
|---|---|---|
| BAT+ (J1 → F1) | 0.12 mΩ | 0.06 mΩ |
| FBAT (F1 → Q1) | 0.13 mΩ | 0.07 mΩ |
| MID (Q1 → Q2) | 0.08 mΩ | 0.04 mΩ |
| PACK+ (Q2 → R24) | 0.23 mΩ | 0.11 mΩ |
| SRV_D (R24 → Q3) | 0.11 mΩ | 0.05 mΩ |
| main path, total | 0.66 mΩ: 0.26 W at 20 A | 0.33 mΩ: 0.13 W at 20 A |
| SERVO+ (Q3 → J4 / → J7) | 1.0 / 2.8 mΩ | 0.5 / 1.4 mΩ |
| BAT− (J1 → R1) | 0.35 mΩ | 0.18 mΩ |

**What it shows:**
- **The copper is not the thermal limit.** At 20 A the parts on the same path dissipate
  about 4–5 W: the FETs, R24, R1, F1 and the XT60. At the walking current, about 4 A, losses
  fall 25-fold.
- **Peak current density** is 5.6 A/mm of copper width at 20 A, at the pad edges between R24
  and Q3.

## C13. The pack ordered on 2026-10-07

**The pack:** OVONIC O-130C-4200-3S1P-XT60-2P (Amazon B0DNLVMRVQ, two packs).

**What the maker states** (Ovonic's store page
[us.ovonicshop.com](https://us.ovonicshop.com/products/2-pack-ovonic-3s-11-1v-4200mah-130c-lipo-battery-with-xt60-plug-for-1-10-1-8-rc-cars-trucks-rock-crawlers),
and the Amazon listing sold by Ovonic Direct):
- standard LiPo, 3.7–4.2 V a cell;
- 4200 mAh, 130C continuous, soft case;
- 96 × 44.5 × 25.5 mm, 246 g;
- 12 AWG leads, XT60 discharge plug, JST-XHR-4P balance plug;
- charge at 1C at most.

**What no source states:** no maker datasheet, MSDS, internal resistance, lead length or XT60
gender was found.

| | the board and the design | the pack | result |
|---|---|---|---|
| chemistry and voltage | 3S LiPo, not HV (design record §3) | 3S, 4.2 V a cell max: 12.6 V | match |
| discharge plug | J1, XT60PW-M | XT60, gender not stated | P1 as before |
| balance plug | J2, JST B4B-XH-A | JST-XHR-4P | mates; P2 as before |
| capacity and runtime | 4000 mAh decided; 49 / 128 min (§3, upper / lower model) | 4200 mAh | 51 / 134 min by the same models |
| current | servo bus peaks 4.74 A walking and 9.4–11.8 A in get-ups (§1) | 130C | not a limit |
| charging | | 4.2 A at most | a charger setting |
| servo-rail short (C5) | the BQ76922's 150 A threshold, ±35 % (datasheet, V(SCD) accuracy): trips at 97.5 A at the lowest | the pack current peaks at 58–76 A for packs of 15–5 mΩ | the servo switch still clears it alone |
| hard short on PACK+ (C6) | Q1, Q2: 400 A pulsed, 30 V | stiffer than the 30C pack assumed | **depends on the internal resistance**, table below |
| F1 breaking capacity | 1 kA at 32 V DC (distributor listings; Littelfuse's datasheet still missing) | a short on the battery side of the FETs: 12.6 V over the pack, the leads (≈ 1 mΩ), F1 (4.58 mΩ cold), R1 and the copper → 0.57 / 0.83 / 1.03 kA for a 15 / 8 / 5 mΩ pack | at the limit if the pack is near 5 mΩ |
| battery bay | CAD: a 105 × 36 × 26 mm placeholder pack (`cad/v6/dimensions_v6.py` BATT), not the layer's room; plant mass 170 g (`sim/gen_plant_v6.py`) | 96 × 44.5 × 25.5 mm, 246 g | the torso session's notes give the layer ≈ 114 × 47 × 29 mm, which the pack fits; the CAD's BATT and BATT_MASS take the measured pack (#108) |

**A hard short on PACK+, by pack resistance.** The same model as C6, R20 4.7 kΩ. Pack
resistance means cells, leads and plug together.

| pack resistance | wiring | Q2 peak current | Q2 peak V(DS) | Q2 energy |
|---|---|---|---|---|
| 35 mΩ (C6) | 200 / 500 nH | 218 / 215 A | 14.7 / 20.0 V | 17 / 25 mJ |
| 15 mΩ | 200 / 500 nH | 344 / 324 A | 17.2 / 25.4 V | 36 / 53 mJ |
| 8 mΩ | 200 / 500 nH | **424** / 382 A | 19.0 / 28.8 V | 52 / 74 mJ |
| 5 mΩ | 200 / 500 nH | **468 / 412 A** | 20.0 / **30.6 V** | 63 / 87 mJ |
| 8 mΩ, Q1 and Q2 each two in parallel | 200 / 500 nH | 438 / 399 A, **219 / 200 A a FET** | 15.1 / 20.4 V | 68 / 94 mJ for the pair |
| 5 mΩ, two in parallel | 200 / 500 nH | 483 / 432 A, **242 / 216 A a FET** | 15.7 / 21.6 V | 82 / 111 mJ for the pair |

**What it shows:**
- **At 15 mΩ or more** the single FETs stay within their ratings.
- **At 8 mΩ or less**, Q2's peak passes its 400 A pulse rating, and with 500 nH of wiring its
  V(DS) nears or passes 30 V.
- **No setting fixes it.** The current keeps rising through the sense filter's delay. A
  sweep of R20 (1–4.7 kΩ) and the threshold (100 or 150 A) left the peak at 370–470 A. A
  faster turn-off trades the energy for a 31 V avalanche at 300 A or more, three times the
  100 A of the avalanche curve (Figure 11).
- **Two FETs in parallel** halve each FET's peak, because the peak comes while the FETs are
  fully on. It also keeps V(DS) at 21.6 V or less up to 500 nH. In the slow turn-off the
  split depends on how well the FETs' thresholds match, so one FET may take more than half
  the pair's energy.
- **Lead inductance.** A 10 cm pair of 12 AWG wires 5 mm apart is about 63 nH by the
  two-wire formula, L = (µ0/π) · l · ln(d/r). The pack's own lead plugged in directly sits
  near the 200 nH rows; an extension moves toward 500 nH.

**Measure on arrival:** the pack's internal resistance (P7 in harness.md), with a charger's IR
reading or as ΔV/ΔI under a known load. It calibrates this model. D-FET was decided without it.

**The other shorts, with the same model** (packs of 15–5 mΩ, 200–500 nH):

| short | how it could happen | Q1/Q2 | what else |
|---|---|---|---|
| servo lead (C5) | a lead pinched in a joint | not involved; Q3 clears it in 6–8 µs, at 0.1 mJ | the logic stays up |
| end of the logic lead J3: F2 + 0.3 m of 22 AWG (16 mΩ) + a 10 mΩ fault, ≈ 400 nH | the lead pinched, or a fault at the General Driver's input | with the 1206 fuse (23 mΩ): 158–184 A, V(DS) ≤ 21.4 V, ≤ 23 mJ; with the MINI fuse (change 8, 33.75 mΩ): 137–158 A, V(DS) ≤ 19.9 V, ≤ 18 mJ: within ratings | **1206 fuse:** it melts first (melting I²t 0.501 A²s: ≈ 17 µs at 170 A), against an interrupting rating of 50 A (Bourns), and the BQ76922 trips 48–81 µs after the short anyway. A 1206 part cannot be replaced by hand. **MINI fuse:** the current sits near the BQ76922's 150 A threshold. Where it trips (a 5 mΩ pack, or the threshold's −35 % end) the board turns off in 30–85 µs. Otherwise the fuse clears it: melting I²t 9.4 A²s puts it at ≈ 0.5 ms at 140 A, inside its 1 kA interrupting rating. Both values are from distributor listings (Littelfuse's datasheet refused the download). The model treats fuses as resistors, so the melt time is from I²t, not simulated |
| PACK+ copper on the board, 10 mΩ + 100 nH | a dropped screw or tool, a slipping probe, debris, a cracked capacitor | the table above | |
| battery side of the FETs | a fault at J1 or F1 | not involved | only F1 can clear it (row F1 above) |

**What else was tried for the PACK+ short:**
- **A smaller sense-filter capacitor** (C9 100 nF → 1–22 nF, threshold 150 A and its +35 % end, 202 A). It
  trips sooner, but the current has already risen.
  - 8 mΩ, 200 nH: 364–393 A at 150 A; 388–406 A at 202 A.
  - 5 mΩ, 200 nH: 391–427 A at 150 A; 421–442 A at 202 A.
  - Below 400 A only at 8 mΩ, by 2–9 %. TI recommends 100 nF.
- **A stronger single FET.** No TI 5 × 6 mm part is rated above 400 A pulsed (CSD18510Q5B, CSD17570Q5B: 400 A).
  - Infineon's BSC010N04LS6 is 40 V, 1.0 mΩ, 1140 A pulsed, in stock at $2.56.
  - Its SOA (Diagram 3, read at 20 V) is about 310 A for 10 µs and 43 A for 100 µs. Figure 10 gives the
    CSD17556Q5B more than 500 A and 85 A at the same points. This fault's long, partly-on turn-off is what
    an SOA plot rates, and there the Infineon part is weaker.
  - Its land pattern also differs from TI's. It would need its own model run before it could be preferred.

**Costs of the fixes** (three assembled boards, five bare; instant quotes on 2026-10-07):

| | parts | assembly (PCBWay) | bare boards | complexity |
|---|---|---|---|---|
| measure first (P7) | $0 | $0 | $0 | a charger IR reading |
| C9 → 10 nF | $0 | $0 | $0 | one value; small gain (above) |
| Q1 and Q2 each two in parallel | 6 × CSD17556Q5B ≈ $21 (Digi-Key $3.56 at qty 1, 2026-10-07 parts pass) | +$4.57 ($176.74 → $181.31 at 229 → 239 SMD pads) | $0 if they fit the 97 mm outline; if it grows to 112 mm, +$36.89 at PCBWay ($55.93 → $92.82) or +$21.03 at JLCPCB ($41.97 → $63.00) | a new placement and route, then C1, C2, C6, C11 again; the torso holds a longer board if it grows |
| BSC010N04LS6 in place of Q1/Q2 | −$1 a FET | — | — | a new footprint and model run; weaker SOA |


## C14. A Raspberry Pi 5 in place of the Pi 4B

**Sources:**
- Raspberry Pi's documentation (*power supplies*): the Pi 5 is specified for a 5 V / 5 A supply,
  with a typical bare-board active current of 800 mA (Pi 4B: 600 mA).
- USB peripherals get 1.6 A on a supply that negotiated 5 A, otherwise 600 mA.
- Powered through the GPIO header there is no negotiation. `usb_max_current_enable=1` in config.txt, or
  `PSU_MAX_CURRENT=5000` in the EEPROM, allows the 1.6 A
  ([Raspberry Pi white paper, USB power delivery on Raspberry Pi 5](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)).
- The low-voltage flag is at 4.63 V, as on the Pi 4B.
- A third-party measurement ([raspberry.tips](https://raspberry.tips/en/?p=8794)) gives about 3 W idle,
  8.8 W at full CPU load, and 12–16 W with an NVMe drive or an AI HAT.

| | Pi 4B (design record §3) | Pi 5 |
|---|---|---|
| logic current at 11.1 V, upper model (Pi at full load + camera + General Driver, 90 % buck) | 0.87 A | 1.11 A (8.8 W) |
| runtime, 4200 mAh, upper / lower model | 51 / 134 min | 48 / 126 min |
| with 12 / 16 W of Pi load (NVMe, AI HAT) | | 45 / 41 min (upper model) |
| current through F2 and J3 at a 9.9 V pack | ≈ 1.0 A | 1.24 A; 2.05 A at 16 W of Pi load |
| ratings on that path | F2 3 A (≈ 2.7 A at 85 °C, Bourns' derating curve); J3 and H1 3 A a contact | within them |
| the General Driver's 5 V | MP8759, 8 A part (MPS), labelled "5V-5A" on the schematic | ≈ 2 A with the camera |

**What it shows:**
- **The power board needs no change for a Pi 5.** Its protection thresholds sit above servo peaks of 12 A,
  and the Pi adds a quarter of an amp.
- **Runtime still meets 45 min** under the upper model unless the Pi carries 12 W or more.
- **Not checked here:** the torso is drawn for the Pi 4B (mounting, port side, 16 mm depth with no fan),
  and the Pi 5's camera connector differs.

## Design changes

| | change | why |
|---|---|---|
| 1 | C16 150 nF → 330 nF (Samsung CL21B334KBFNNNE); C17 2.2 µF → 4.7 µF (Samsung CL21B475KAFNNNE) | soft-start margin (C4) |
| 2 | R20 1 kΩ → 4.7 kΩ | the PACK+ short spike (C6) |
| 3 | R38 33 kΩ → 47 kΩ | RAIL_V within the ESP32 ADC's range (C7) |
| 4 | Q1–Q3 on TI's land pattern | C8 |
| 5 | J1 pin 1 = BAT−, pin 2 = BAT+ | reversed polarity (C8) |
| 6 | J1 → Amass XT60PW-M at the left edge (board 97 × 56.5 mm); D1 → SOD-323F | C8 |
| 7 | J9 (12-pin) → J9 (I²C, 4-pin) + J12 (control, 8-pin) | stock and the pin budget (C9) |
| 8 | F2: Bourns SF-1206F300-2 (1206, 50 A interrupt) → Littelfuse 0297003.WXNV, a 3 A MINI blade fuse in a Keystone 3568 holder like F1's; F2 and J3 move to a strip along the top edge, and the board grows to 97 × 64.5 mm | a pinched logic lead destroyed the 1206 fuse, which cannot be replaced by hand (C13; D-F2, Tom 2026-10-07) |

Changes 4–8 needed a new layout. It routes completely, and C1 and C2 pass on it.

## Decisions for Tom

- **Copper:** 1 oz or 2 oz outer copper. C11 shows 1 oz is enough; 2 oz costs $261 more for
  5 boards at PCBWay and $35 more at JLCPCB.
- **D-FET, decided (Tom, 2026-10-07): no parallel FETs.**
  - Q1 and Q2 stay single. A hard short on the board's own PACK+ copper is the one fault they
    do not cover with a pack under about 15 mΩ (C13), and Tom judged that fault too unlikely
    for the cost.
  - The residual risk is carried by two handling rules in Bring-up.
  - P7 still measures the pack, to calibrate the model, but no longer decides anything.
- **D-F2, decided (Tom, 2026-10-07): a replaceable logic fuse.** Change 8.
- **D-TVS:** add an SMBJ15A on the battery side. It would clamp the PACK+ short spike at
  about 20 V whatever the pack-lead length; with change 2 the spike is 14.7–20.0 V on
  realistic leads for a 35 mΩ pack. It needs room above the power strip. With a stiff pack it
  does not fix C13: the unfiltered model put Q2's energy up, to 82–92 mJ at 1 µH.
- **D-J9:** leave SERVO_IMON and SERVO_FLT_N unwired (J12 pins 7–8, bench probes), with
  servo current inferred from pack current minus INA219 current. Wiring them needs two more
  ESP32 pins, which the General Driver does not have free.
- **Buttons:** pick the POWER (momentary NO) and E-STOP (latching, two NC contacts) parts.

## Before ordering

The physical checks P1–P7 in [harness.md](../../pcb/power-board/harness.md) need the real
pack and the General Driver:
1. the pack's XT60 polarity;
2. the balance-lead order;
3. H1's polarity;
4. H5's orientation;
5. the families of the control headers;
6. the servo leads' gauge;
7. the pack's internal resistance (C13), for the model.

After them, a quote at the final size, 97 × 64.5 mm. Instant quotes for that size on
2026-10-07 gave five bare boards at $55.93 at PCBWay ($42.10 at JLCPCB) and assembly of three
at $177.76 at PCBWay.

## Bring-up

The order for a board that arrives, before the bench checks V1–V9 of the design record (§10):
1. **Unpowered.** Inspect U1, U2 and Q1–Q3 under magnification. Meter BAT+, PACK+, SERVO+,
   LOGIC_OUT and 3V3_HOST to GND and BAT− for shorts. Check J1's + and − against P1's result
   with a meter on the bare pads.
2. **First power from a bench supply,** never the pack. Set 11.1 V with a 100 mA limit on J1,
   and leave every other connector empty. Log the supply current before and after POWER;
   V1 states the expected off-state current.
3. **Configuration in RAM.** Write the BQ76922 settings over I²C, and check cells,
   current and the FET states against the supply.
4. **Then V1 and V2** from the design record, raising the supply's limit step by step.
   Burn the OTP only after the RAM half of V2 passes, with BAT at 10–12 V.
5. **Then the pack.** Connect it only after the servo rail has been switched on and off
   into a resistive load.

**With the pack connected** (D-FET accepts the risk of a hard short on the board's PACK+ copper):
- **Keep metal off the board.** Probe with the bench supply in place of the pack. With the pack
  in, insulate PACK+, and keep screws and tools off the board.
- **After any short, prove the main switch.** Turn the board off with POWER or KILL, and check
  with a meter that PACK+ falls to 0 V. A Q2 that failed shorted keeps PACK+ live. The board
  then has no low-voltage cut-off and no E-stop backup, and must not be used.
