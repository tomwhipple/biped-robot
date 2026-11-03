# A cheaper power system: options and landed prices (2026-11-02)

*A dated record. Tom, 2026-11-02, after the [rev B quote](2026-11-02-power-board-rev-b-quote.md):
"2 boards is more than $300 and 30+ days with tax and shipping and all. I think we need to go
back to the drawing board and come up with much cheaper battery protection and power
distribution system." The custom board is [pcb/power-board](../../pcb/power-board/README.md);
the earlier off-the-shelf search is
[2026-10-24-power-board-off-the-shelf-survey.md](2026-10-24-power-board-off-the-shelf-survey.md).*

## Tom's choices (2026-11-02)

- **Cell protection:** an off-the-shelf 3S BMS module.
- **Main power:** a manual switch. There is no soft power button and no automatic
  power-off.
- **E-stop:** the button's NC contact drops a relay that feeds the servo rail, behind a fuse.
- **Build:** a simple 2-layer board of through-hole parts that Tom solders, or kit and demo
  boards.
- **Sourcing:** "make sure that we compare total price". US and overseas sources are compared
  on the landed total.

## The design

```
pack XT60 ─ BMS (3S, low side) ─ main switch ─┬─ F 3 A ── General Driver (JST XH) ── Pi via its 40-pin
                                              │
                                              └─ F 15 A ── relay contact ── bulk C + TVS ── J4–J7 servo branches
relay coil:  +V ─ E-stop NC ─ coil (1N4007 across it) ─ logic FET (SERVO_EN from the ESP32) ─ GND
to the ESP32: the rail-voltage divider (GPIO34) and the E-stop's state
```

- **The E-stop acts without firmware.** Pressing it opens the coil circuit and the relay
  drops the servo rail.
- **The ESP32 holds the rail off until it is ready.** The FET in the coil path is held off by
  a pull-down, as `SERVO_EN` does on rev B.
- **The General Driver and the Pi stay up through an E-stop.** Their branch bypasses the relay.
- **Pack voltage comes from the General Driver's INA219** (0x42), as the firmware's battery
  guard already plans (2026-10-06 power-circuit record §8).
- **Parts already bought carry over:** the XT60PW-M and the Keystone 3568 fuse holders are
  through-hole. So do the harness pinouts (J3, J4–J8) and the firmware's `SERVO_EN`, rail and
  E-stop plan.

## Parts and prices

Digi-Key prices are for qty 1, read on 2026-11-02 by a research agent; the links are in its
sources below.

| part | pick | price | alternative |
|---|---|---|---|
| BMS | ShillehTek CF-3SJH25A-C, ships from the US | $6.99 | none found with a maker spec (below) |
| relay | TE 1-1414610-0: 70 A, operates at 7.2 V, 1.6 W coil; 292 in stock | $13.37 | TE 1393280-5: 20 A continuous at 85 °C, operates at 6.9 V, 0.57 W; $6.07 |
| main switch | Cole Hersee 57000-01: rocker, 25 A DC at 12 V | $19.11 | the XT60 plug itself, $0 |
| fuses | Littelfuse 0297015.WXNV (15 A) and 0297003.WXNV (3 A) | $0.81 | 20 A if 15 A blows on get-ups |
| surge, coil diode, bulk | 1.5KE16A, 1N4007, Panasonic EEU-FR1E102 1000 µF 25 V | $3.94 | |
| connectors, FET, resistors, LEDs | Molex 22-03-5035 ×5, JST XH/PH, a TO-92 logic FET | not yet priced | PCBWay quoted the connectors at about $1.70 a board |
| **per robot** | | **$44.22** + small parts | **$17.81** with the cheaper relay and no switch |
| E-stop, needed by every design | IDEC XW1E-BV402M-R | $50.65 | Eaton M22-PV + M22-K01, $34.12 + $15.11 |

Bare boards, 2-layer, about 97 × 65 mm, 1.6 mm, 1 oz:

| fab | boards | price | lead time |
|---|---|---|---|
| PCBWay (China) | 5 | $5.00 + $26.55 DHL = **$31.55** before duty | 24 h build, 2–4 days DHL |
| OSH Park (US) | 3 | about $49, free shipping, no duty | 9–12 days |

- **Duty on the PCBWay boards** is small but not exactly known:
  - De minimis is suspended for every country (Federal Register 2026-12670), so duty is owed.
  - The duty would be 12.5–37.5% of the $5 board value, about $1–2. The 12.5% applies while
    the Section 301 exclusion runs; USTR extended the exclusions to 2026-11-10. That the
    exclusion covers bare PCBs comes from secondary sources only.
  - Whether DHL adds a brokerage fee was not found.
- **2 oz copper** is not offered at 1.6 mm by OSH Park. 1 oz is enough for 15 A on two layers
  with pours on both sides; to be checked in the layout.

## The total, against rev B

E-stops are excluded on both sides: every design needs one, and rev B's quote did not
include one.

| | 2 robots | landed and lead time |
|---|---|---|
| **rev B**, PCBWay turnkey | $218.54 before shipping and duty | Tom: over $300; 30–35 days |
| **this design**, standard picks | 2 × $44.22 + $31.55 boards + about $8.49 Digi-Key shipping = **about $128** | plus duty on $5, small parts and the BMS's shipping; about 1 week |
| **this design**, cheaper relay, no switch | 2 × $17.81 + $31.55 + $8.49 = **about $76** | same |

## What it gives up against rev B

1. **The BMS's thresholds are unknown.**
   - No 3S board was found with both a maker specification and a rating for the 12 A
     get-up peaks.
     - TI's BQ77915EVM-014 is documented but rated 2.5 A typical and 4 A maximum
       (user guide SLUUBU2).
     - ShillehTek's own datasheet (DS-NA405) states its thresholds and current rating as
       undocumented.
     - Daly and JBD boards have marketplace listings only.
   - Using one is an exception to AGENTS.md's rule that electronics need manufacturer
     documentation. It needs a bench characterisation before it goes in the robot: the
     under-voltage trip per cell, no trip at 12 A for 2 s, and the short-circuit trip.
2. **There is no soft start.**
   - The relay closes straight onto the servos' input capacitance and the bulk capacitor.
   - The inrush could trip the BMS's short-circuit detector.
   - The options are a bulk capacitor on the always-on side, a pre-charge resistor, or a
     smaller bulk capacitor. Test it on the bench, and it can be simulated with
     `pcb/power-board/pbsim` once the BMS is characterised.
3. **The BMS switches the negative lead.**
   - System ground becomes P−, and B− differs from it once the BMS trips.
   - The pack's balance lead must go only to the BMS.
   - A USB laptop on the ESP32 shares P−, which is fine.
4. **Less telemetry.** There are no per-cell voltages, no protection flags and no servo
   current; pack voltage comes from the INA219 only.
5. **Manual power-off.**
   - After a low-battery landing the firmware halts the Pi, but power stays on until the
     switch is turned off.
   - The BMS's under-voltage cut is what protects the cells if it is left on.
6. **The relay coil draws power** whenever the servos are enabled. The 70 A relay uses
   1.6 W, about 1.2 Wh in 45 minutes of the pack's 46.6 Wh (4.2 Ah × 11.1 V), or 2.6%.
   The 20 A relay uses 0.57 W.
7. **Unchanged:** the servo headers' 2.5–3 A contact rating against the leg branches' peaks
   (`harness.md`, bench check V8).

## Decisions for Tom

1. **BMS:** the ShillehTek board, with the bench characterisation above, as an exception to
   the datasheet rule.
2. **Relay:** the 70 A TE ($13.37), with margin for the inrush; or the 20 A TE ($6.07).
3. **Main switch:** the Cole Hersee rocker ($19.11), or unplugging the XT60.
4. **Boards:** PCBWay (about $33 landed, about a week), or OSH Park (about $49, US, about two
   weeks).
5. **E-stop:** IDEC XW1E-BV402M-R ($50.65), or Eaton M22-PV + M22-K01 ($49.23).

Then: schematic and a 2-layer through-hole layout, simulation of the relay closing
(inrush against the BMS), and an order.

## Sources

- **Research agents, 2026-11-02:** Digi-Key product pages, and the datasheets of the TE
  V23134 (1-1414610-0) and V23086 (1393280-5), the Littelfuse 1.5KE, Good-Ark 1N400x and
  Panasonic FR series, and the IDEC XA/XW.
- **ShillehTek:**
  - [product page](https://shillehtek.com/products/3s-12v-18650-lithium-battery-protection-board-11-1v-12-6v-balanced-25a);
  - [DS-NA405 datasheet](https://cdn.shopify.com/s/files/1/0837/4340/8415/files/shillehtek-na405-datasheet.pdf?v=1790018568),
    read here.
- **TI** [BQ77915EVM user guide SLUUBU2](https://www.ti.com/lit/pdf/SLUUBU2), read here: the
  table "Charge and discharge current 0, 2.5, 4 A".
- **Eaton M22-K01** at [Digi-Key](https://www.digikey.com/en/products/detail/eaton-electrical/M22-K01/25845193).
- **PCBWay's public instant quote**, run here with `quote2l.py`: 2 layers, 97 × 65 mm, 5 pieces,
  1.6 mm, 1 oz, 8/8 mil, HASL lead-free, green, DHL to the US.
- **OSH Park** [two-layer service](https://docs.oshpark.com/services/two-layer/).
- **Federal Register**
  [2026-12670](https://www.federalregister.gov/documents/2026/06/24/2026-12670/indefinite-suspension-of-the-de-minimis-exemption-for-merchandise-arriving-through-all-modes-other)
  (de minimis).
- **USTR**
  [November 2025 exclusion extension](https://ustr.gov/about/policy-offices/press-office/press-releases/2025/november/ustr-extends-exclusions-china-section-301-tariffs-related-forced-technology-transfer-investigation).
