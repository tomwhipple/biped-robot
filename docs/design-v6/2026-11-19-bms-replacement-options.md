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
| Bestech HCX-D113V1 | [PDF](https://bestechpower.com/wp-content/uploads/2026/04/Same-port-D113V1-3S-Li-ion-BesTech-Power-datasheet.pdf), opened | 15 | 2.50 ±0.05 V / 50–200 ms / 3.00 V | 200–500 µs | ≤ 20 µA | 45 × 35 × 4 | quote, CN |
| Bestech HCX-D123 (Ricoh R5432N505BD) | [PDF](https://bestechpower.com/wp-content/uploads/2026/04/D123-3S-Li-ion-BesTech-Power-datasheet-compressed.pdf), opened | 20 | 2.50 V / 50–200 ms / 3.00 V | 200–500 µs | ≤ 50 µA | 50 × 50 × 5 | quote, CN |
| Bestech HCX-D690V3 (Cellwise CW1243) | [PDF](https://bestechpower.com/wp-content/uploads/2026/04/Same-ports-3S-D690V3-Li-ion-BesTech-Power-datasheet.pdf), opened | 12 | 2.7 ±0.08 V / 0.5–1.5 s / 3.0 V | 140–550 µs | ≤ 50 µA | 60 × 25 × 4 | quote, CN |
| BatterySpace PCB-LI3S10A | vendor page | 10 | 2.50 V / 10–300 ms / 3.0 V | 200–800 µs | ≤ 150 µA a cell | 60 × 34 × 2 | $20.64, US |
| Tenergy 32007 | vendor page | 9 (OC 17 ±4 A) | 2.5 V / 100 ms / 3.0 V | n/s | 40 µA | 50 × 16 × 3.5 | $23.99, US |
| JBD D04S20A | maker web text, no sheet | 20 | 2.7 ±0.1 V / ≤ 1.5 s / 3.0 V | n/s | 10 µA | two sizes stated | $26.20 for 5, CN |
| Victron Smart BatteryProtect 65 A | [specs](https://www.victronenergy.com/media/pg/Smart_BatteryProtect_12V_24V/en/technical-specifications.html), opened | 65 (250 A peak) | 9.5 V pack (preset) / 90 s / 11.5 V | — | 0.7–1.4 mA | 106 × 48 × 40, 200 g | $53.55, US |

- **The Bestech over-current trips** are 30 ±10 A (D113V1) and 40 ±10 A (D123), after 5–20 ms,
  so the 12 A get-up is below the lowest.
- **Daly** still publishes no thresholds (its 3S page's download is an email address).
  **TI's BQ77915** is documented only as a 4 A EVM.
- **All the boards release on voltage** (3.0 V a cell). Whether a pack that tripped under
  the logic's 1 A recovers past that at rest, and so switches the logic back on, is not known.
  The bench check of the replacement tests it (bringup.md 1.5).
- **Not ordered from the agent's list:** BatterySpace and Tenergy are under the 12 A get-up;
  the JBD has no sheet; the Victron is four times the bay's height, switches the positive
  lead, and its 250 A peak is under the simulated inrush.

## A cut-off on the board instead

The logic, the Pi and K1's coil all take their power after F2, so one 3 A switch there would
turn the whole robot off; K1 then drops the servo rail. Documented through-hole parts for it:
Renesas ICL7665S (PDIP-8, resistor-set trip and hysteresis, 3 µA), TI TL431 (TO-92) with an
LM393 comparator. The trip can sit at 3.0 V a cell and latch off until the main switch is
cycled. It needs a circuit design, a simulation in `sim/lite_sim.py` and a new board.

## Recommendation

**Bestech HCX-D113V1.** It is the only documented board that is rated above the get-up, rides
through the inrush (its 200 µs minimum against 132 µs) and is smaller than the bay. Its
2.50 V trip is the lowest a cell maker allows rather than a hobby margin, so the firmware's
9.9 V landing stays the working limit and the BMS is the backstop. Ask Bestech, with the quote,
whether they set the trip at 2.8–3.0 V (their sheets head the table "Criteria (Adjustable)").
The bracket in the power bay is redrawn for 45 × 35 mm, and Part 1 of bringup.md is repeated
on the new board.
