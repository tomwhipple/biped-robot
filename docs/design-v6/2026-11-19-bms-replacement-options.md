# Replacing the power lite's BMS: options (2026-11-19)

*A dated record. The ShillehTek CF-3SJH25A-C has no under-voltage cut-off
([bench record](2026-11-19-power-lite-bms-bench.md)), so the pack is unprotected if the robot
is left on. A research agent searched for documented replacements on 2026-11-19; "opened"
means it downloaded the maker's document and read the numbers.*

## What the replacement must do

- **Cut the pack off at low voltage**, between 2.5 and 3.2 V a cell (7.5–9.6 V). The firmware
  warns at 10.5 V and lands the robot at 9.9 V (3.3 V a cell), so the cut-off is the backstop.
- **Carry the get-up**, 12 A for about 2 s, without an over-current trip.
- **Ride through the relay's inrush**, 176–341 A for 52–132 µs (README, Checks): its
  short-circuit delay must be longer than that.
- **Switch the negative lead** (P−), as the power lite is wired (W1 B−, W2 P−, W3–W5 taps).
- **Have a maker's specification** (AGENTS.md, Sourcing).

## Candidates

| part | maker doc | cont. A | UV trip / delay / release | short-circuit delay | Iq | size (mm) | price, from |
|---|---|---|---|---|---|---|---|
| Bestech HCX-D113V1 | datasheet, opened | 15 | 2.50 ±0.05 V / 50–200 ms / 3.00 V | 200–500 µs | ≤ 20 µA | 45 × 35 × 4 | quote, CN |
| Bestech HCX-D123 (Ricoh R5432N505BD) | datasheet, opened | 20 | 2.50 V / 50–200 ms / 3.00 V | 200–500 µs | ≤ 50 µA | 50 × 50 × 5 | quote, CN |
| Bestech HCX-D690V3 (Cellwise CW1243) | datasheet, opened | 12 | 2.7 ±0.08 V / 0.5–1.5 s / 3.0 V | 140–550 µs | ≤ 50 µA | 60 × 25 × 4 | quote, CN |
| BatterySpace PCB-LI3S10A (#3511) | none: specs in the product page's text and a drawing; the board's maker is not named | 10 | 2.50 V / 10–300 ms / 3.0 V | 200–800 µs | ≤ 150 µA a cell | 60 × 34 × 2 | $20.64, US |
| Tenergy 32007 | none: specs in the product page's text | 9 (OC 17 ±4 A) | 2.5 V / 100 ms / 3.0 V | not stated | 40 µA | 50 × 16 × 3.5 | $23.99, US |
| LLT (Shenzhen Li Lu Tong) 3S 20 A | none: specs in the maker's product page's text, two sizes stated | 20 | 2.7 ±0.1 V / ≤ 1.5 s / 3.0 V | not stated | 10 µA | 62 × 45 × 10 or 36 × 18 × 5 | $26.20 for 5, CN |
| Victron Smart BatteryProtect 12/24V-65A (BPR065022000) | datasheet, opened | 65 (250 A for 30 s) | 9.5 V pack (preset) / 90 s / 11.5 V | not stated | 0.7–1.4 mA | 106 × 48 × 40, 200 g | $53.55, US |

### Where to get them

| part | maker's document | buy | seen on 2026-11-19 |
|---|---|---|---|
| Bestech HCX-D113V1 | [datasheet (PDF)](https://bestechpower.com/wp-content/uploads/2026/04/Same-port-D113V1-3S-Li-ion-BesTech-Power-datasheet.pdf) | [product page, "Get A Quote"](https://bestechpower.com/product/3s-11-1v-bms-ltn3s15a15a001/) (SKU LTN3S15A15A001) | 100 in stock, no price |
| Bestech HCX-D123 | [datasheet (PDF)](https://bestechpower.com/wp-content/uploads/2026/04/D123-3S-Li-ion-BesTech-Power-datasheet-compressed.pdf) | [product page, "Get A Quote"](https://bestechpower.com/product/3s-11-1v-bms-ltn3s20a20a001/) (SKU LTN3S20A20A001) | 100 in stock, no price |
| Bestech HCX-D690V3 | [datasheet (PDF)](https://bestechpower.com/wp-content/uploads/2026/04/Same-ports-3S-D690V3-Li-ion-BesTech-Power-datasheet.pdf) | [product page, "Get A Quote"](https://bestechpower.com/product/3s-11-1v-bms-ltn3s12a12a001/) (SKU LTN3S12A12A001) | 100 in stock, no price |
| BatterySpace PCB-LI3S10A | none ([drawing](https://www.batteryspace.com/prod-specs/3511-drawing.jpg) only) | [BatterySpace](https://www.batteryspace.com/protection-circuit-module-pcb-for-3s-li-ion-battery-pack-at-10a-limit.aspx) | $20.64, in stock, ships from California |
| Tenergy 32007 | none | [Tenergy](https://power.tenergy.com/protection-circuit-module-pcb-for-11-1v-3s-li-ion-battery-pack-working-9a-cutoff-17a/) | $23.99, in stock, ships in 1 business day |
| LLT 3S 20 A | none | [LLT's store](https://www.lithiumbatterypcb.com/product/3s-or-4s-li-ion-12v-battery-bms-with-20a-or-30a-constant-current/) | $26.20 for 5 |
| Victron Smart BatteryProtect 65 A | [datasheet (PDF)](https://www.victronenergy.com/upload/documents/Datasheet-Smart-Battery-Protect-65-A--100-A--220-A-EN.pdf), [manual (PDF)](https://www.victronenergy.com/upload/documents/Smart_BatteryProtect_12V_24V/114439-Smart_BatteryProtect-pdf-en.pdf), [product page](https://www.victronenergy.com/battery_protect/smart-battery-protect) | [Fisheries Supply](https://www.fisheriessupply.com/victron-energy-smart-batteryprotect); [eMarine](https://emarineinc.com/products/victron-smart-batteryprotect-12-24v-65a) | $53.55; 1 left at Fisheries Supply, sold out at eMarine |

- **Bestech's quotes:** the product pages' "Get A Quote" form, or
  [the contact page](https://bestechpower.com/contact/) (info@BesTechPower.com); the
  datasheets give sales@BesTechPower.com. They ship from Shenzhen/Dongguan.
- **The Bestech over-current trips** are 30 ±10 A (D113V1) and 40 ±10 A (D123), after 5–20 ms,
  so the 12 A get-up is below the lowest.
- **Daly** still publishes no thresholds: its
  [3S 20 A page](https://www.dalybms.com/good-quality-bms-3s-20a-12-6-v-standard-nmc-bms-20s-21s-24s-27s-32s-20a-250a-common-port-daly-product/)'s
  "Data download" is an email link, and its
  [hardware-BMS manual (PDF)](https://www.dalybms.com/uploads/Old-series-hardware-BMS-manual.pdf)
  gives wiring and models only. **TI's** [BQ77915EVM-014](https://www.ti.com/tool/BQ77915EVM-014)
  is documented ([SLUUBU2](https://www.ti.com/lit/pdf/SLUUBU2)) but rated 4 A at most.
- **The LLT board is not JBD's.** Its store's footer is Shenzhen Li Lu Tong Electronic
  Technology; only forum posts link LLT with JBD, and jbdbms.com refused the fetch.
- **All the boards release on voltage** (3.0 V a cell). Whether a pack that tripped under
  the logic's 1 A recovers past that at rest, and so switches the logic back on, is not known.
  The bench check of the replacement tests it (bringup.md 1.5).
- **Not chosen:** BatterySpace and Tenergy are rated under the 12 A get-up and have no
  document; the LLT has no document. The Victron is four times the bay's height and switches
  the positive lead; its 250 A is a 30 s rating, and it states nothing for a microsecond
  inrush.

## A cut-off on the board instead

The logic, the Pi and K1's coil all take their power after F2, so one 3 A switch there would
turn the whole robot off; K1 then drops the servo rail. The trip can sit at 3.0 V a cell and
latch off until the main switch is cycled. It needs a circuit design, a simulation in
`sim/lite_sim.py` and a new board. Documented through-hole parts for it:

| part | maker's document | buy | seen on 2026-11-19 |
|---|---|---|---|
| Renesas ICL7665S: over/under-voltage detector, resistor-set trip and hysteresis, 3 µA, PDIP-8 | [product page](https://www.renesas.com/en/products/icl7665s), [datasheet FN3182](https://www.renesas.com/en/document/dst/icl7665s-datasheet?r=533401) | [Digi-Key ICL7665SACPAZ](https://www.digikey.com/en/products/detail/renesas-electronics-corporation/ICL7665SACPAZ/1034930); [ICL7665SCPAZ](https://www.digikey.com/en/products/detail/renesas-electronics-corporation/ICL7665SCPAZ/1034934) | **both obsolete at Digi-Key**, $4.68 / $4.10; about 1,410 left of one of them |
| TI TL431ACLP: shunt reference, 2.495 V, TO-92 | [product page](https://www.ti.com/product/TL431), [datasheet](https://www.ti.com/lit/gpn/TL431) | [Digi-Key TL431ACLP](https://www.digikey.com/en/products/detail/texas-instruments/TL431ACLP/276229) | active, 6,091 in stock, $0.89 |
| TI LM393P: dual comparator, PDIP-8 | [product page](https://www.ti.com/product/LM393), [datasheet](https://www.ti.com/lit/gpn/LM393) | [Digi-Key LM393P](https://www.digikey.com/en/products/detail/texas-instruments/LM393P/277045) | active, 19,348 in stock, $0.53 |

The ICL7665S is the simpler circuit but is being discontinued in through-hole; the TL431 with
the LM393 is the one to design around.

## Recommendation

**Bestech HCX-D113V1.** It is the only documented board that is rated above the get-up, rides
through the inrush (its 200 µs minimum against 132 µs) and is smaller than the bay. Its
2.50 V trip is the lowest a cell maker allows rather than a hobby margin, so the firmware's
9.9 V landing stays the working limit and the BMS is the backstop. Ask Bestech, with the quote
([product page](https://bestechpower.com/product/3s-11-1v-bms-ltn3s15a15a001/)),
whether they set the trip at 2.8–3.0 V (their sheets head the table "Criteria (Adjustable)").
The bracket in the power bay is redrawn for 45 × 35 mm, and Part 1 of bringup.md is repeated
on the new board.
