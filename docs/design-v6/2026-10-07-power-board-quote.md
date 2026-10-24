# Power board rev A: PCBWay quote (2026-10-07)

*A dated record. The board is [pcb/power-board](../../pcb/power-board/README.md) at layout rev A.
Prices are as quoted on 2026-10-07 in USD, before shipping, duties and VAT. They are for the 88 × 56.5 mm
layout; verification the same day ([2026-10-07-power-board-verification.md](2026-10-07-power-board-verification.md))
grew the board to 97 × 56.5 mm and changed parts, so a final quote follows the physical checks.*

## How it was priced

- **Bare board:** PCBWay's public instant quote (pcbway.com/orderonline.aspx). This needs
  no login and no uploaded files. The settings match the board:
  - 4 layers, 88 × 56.5 mm, 1.6 mm FR-4, ENIG;
  - green mask, white silk, tented vias;
  - 6/6 mil track/space, 0.3 mm minimum hole;
  - 1 oz inner copper.

  The form was driven by a script, which read back each setting before taking the price.
- **Assembly:** the same page's turnkey assembly estimate (PCBWay buys the parts), top side
  only, from the layout's counts:
  - 50 unique parts;
  - 220 SMD pads on 80 parts;
  - 47 through-hole pins on 13 parts;
  - no BGA.

  The page states the estimate "does not include PCB fabrication or the cost of
  components, exact quotation will be updated after all the files you uploaded pass
  review."
- **Parts:** an estimate from Digi-Key's own product pages at the needed quantity breaks,
  not PCBWay's prices:
  - the BOM's MPNs, with in-stock generics for the passives;
  - the XT60 from LCSC, as Digi-Key does not carry it.

  PCBWay buys the parts itself, so its component line will differ.

## Prices

**Bare boards**

| outer copper | 5 boards | 10 boards | build time |
|---|---|---|---|
| 2 oz (as designed) | $316.77 | $328.94 | 9–10 days |
| 1 oz | $55.93 | $81.36 | 4–5 days (48 h express $175.27 / $190.74) |

**Assembly (turnkey labour and setup) and parts**

| | 5 boards | 10 boards |
|---|---|---|
| PCBWay assembly | $253.83 | $456.94 (5–6 days) |
| parts, Digi-Key estimate | ≈ $148 ($29.68 a board) | ≈ $245 ($24.45 a board) |

**Totals**

| | 5 boards | 10 boards |
|---|---|---|
| bare, 2 oz | $316.77 | $328.94 |
| bare, 1 oz | $55.93 | $81.36 |
| assembled, 2 oz | ≈ $719 (≈ $144 a board) | ≈ $1,030 (≈ $103 a board) |
| assembled, 1 oz | ≈ $458 (≈ $92 a board) | ≈ $783 (≈ $78 a board) |

## What the quote showed

- **2 oz outer copper is most of the bare-board price.**
  - Picking 2 oz made the form switch track/space from 6/6 to 8/8 mil on its own.
    The board needs 6/6 mil: its tracks and gaps are 0.15 mm, and the BQ76922's
    0.4 mm-pitch QFN has 0.2 mm between pads.
  - At that 8/8 mil default, 5 boards cost $162.37, with a 4–5 day build and express
    offered. So the heavier copper alone adds $106 over 1 oz.
  - Setting 6/6 mil back by hand was accepted, at $316.77 for 5 boards and a 9–10 day
    build with no express. So 6 mil spacing in 2 oz copper adds a further $154.
  - 8/8 mil is not an option for this board: the QFN's pad gaps are 0.2 mm (7.9 mil).
  - Whether PCBWay's engineers accept 2 oz at 6/6 mil on this QFN is decided only at
    file review.
- **The parts are known before ordering, and some are hard to get.** At Digi-Key:
  - BAS40W-7-F (D1) is backordered.
  - JST B12B-PH-K-S (J9) has 0 in stock, with 400 due on 14 December.
  - The Bourns CSS2H-2512R-1L00F (R1) has 0 in stock, but the CSS2H-2512R-1L00FE is in stock.
  - The Diodes 2N7002K-7 has 0 in stock, but onsemi's 2N7002K is in stock.
  - The R24 MPN in the schematic, CSS2H-2512R-2L00F, is not listed. The 2 mΩ part listed
    is CSS2H-2512K-2L00F. Check the MPN against Bourns' datasheet before ordering.
  - The BQ76922RSNR is in stock: 2,719 at $2.92 each, with a 26-week factory lead time.
    The TPS48111LQDGXRQ1 is also in stock: 8,081 at $5.35 each.

## JLCPCB, for comparison

**How it was priced.**
- **Bare board:** JLCPCB's public quote page (cart.jlcpcb.com/quote), same settings as above.
  Each request JLCPCB's page sent was read back to confirm the settings before taking its
  price.
- **Assembly:** "Standard" PCBA, which hand-solders through-hole parts, estimated from
  JLCPCB's published rates (jlcpcb.com/help/article/pcb-assembly-price):

  | item | rate |
  |---|---|
  | setup | $25.56 |
  | stencil | $8.21 |
  | SMT joint | $0.0016 |
  | hand-soldered joint | $0.0164 |
  | part loading, per unique part | $1.53 |
  | per board | $0.48 |

  The counts are the same as above: 220 SMT joints, 47 through-hole joints and 50 unique
  parts per board.
- **Parts:** the same Digi-Key estimate as for PCBWay.

**JLCPCB's process limits.** JLCPCB publishes 0.15/0.15 mm (6/6 mil) as its minimum track
and space for 2 oz outer copper on multilayer boards. That is this board's minimum, so 2 oz
is a standard build there.

**Prices**

| | 5 boards | 10 boards |
|---|---|---|
| bare, 1 oz outer | $41.95 | $48.65 |
| bare, 2 oz outer | $76.75 | $85.05 |
| assembly, rate-card estimate | ≈ $118 | ≈ $126 |
| assembled, 1 oz (with Digi-Key parts) | ≈ $309 | ≈ $419 |
| assembled, 2 oz (with Digi-Key parts) | ≈ $343 | ≈ $456 |
| shipping, UPS Express Saver, duties paid | $26.75 | |

All bare boards are a 3–4 day build. The 2 oz bare board costs $34.80–$36.40 more than 1 oz.
Inner copper at 1 oz adds about $17; JLCPCB's default inner copper is 0.5 oz.

**Parts JLCPCB cannot supply itself.** JLCPCB loads parts from LCSC. LCSC product pages,
checked 2026-10-07, show:
- **Not listed:**
  - the BQ76922RSNR;
  - the 2 mΩ Bourns shunt;
  - the EEU-FR1E102 (only the 25 mm-tall EEUFR1E102L, out of stock).
- **Out of stock:** CSD17556Q5BT, Keystone 3568, XT60PW-M.

Those would come through JLCPCB's global sourcing or as consigned parts; that cost is not
priced here. The TPS48111-Q1 is in stock at LCSC: 8,706 at $2.99 each.

## Assembling only 2–4 boards

Both houses make bare boards in a minimum batch of 5, but assemble fewer. JLCPCB's quote page
gave its assembly minimum as 2.

| | 2 assembled | 3 assembled | 4 assembled |
|---|---|---|---|
| PCBWay assembly (instant quote) | $132.11 | $172.68 | $213.26 |
| JLCPCB assembly (rate card, $110.27 + $1.60 a board) | ≈ $113 | ≈ $115 | ≈ $117 |
| parts (≈ $29.68 a board, Digi-Key at 5-board breaks) | ≈ $59 | ≈ $89 | ≈ $119 |
| **PCBWay, 1 oz** (bare boards $55.93) | **≈ $247** | **≈ $318** | **≈ $388** |
| **PCBWay, 2 oz** (bare boards $316.77) | ≈ $508 | ≈ $578 | ≈ $649 |
| **JLCPCB, 1 oz** (bare boards $41.95) | **≈ $215** | **≈ $246** | **≈ $277** |
| **JLCPCB, 2 oz** (bare boards $76.75) | ≈ $250 | ≈ $281 | ≈ $312 |

The parts line understates small orders. Unit prices at 2–4 boards sit at higher price
breaks, and JLCPCB charges the minimum order quantity of cheap parts it does not stock.
JLCPCB's figures also leave out the global-sourcing cost of the parts LCSC does not carry,
above.

## Quote-only vendors (checked 2026-10-07, nothing submitted)

| | WellPCB | Avanti Circuits |
|---|---|---|
| where | China | Phoenix, AZ |
| public prices | none; the "instant quote" page is a request form | none |
| what they do | PCB fab and turnkey assembly | PCB fab (1–3 oz outer copper, ENIG, 10+ layers); assembly is mentioned but not described |
| stated lead times | fab 5–6 days (24–48 h expedited); assembly "25+ Days (To Be confirmed)" | fab: multilayer in 24 h, prototypes from 1 day; assembly not stated |
| minimums | not stated | "no NRE, minimums or contracts" for fab |
| what a quote needs | form with email, phone and a Gerber zip (BOM optional); reply "within 12 hours" | form with name, company, phone, email and specs, Gerbers optional; reply time not stated |
| to ask | 6/6 mil at 2 oz outer; 0.4 mm WQFN with exposed pad; parts billed separately?; minimum quantity | turnkey with parts?; 2 oz at 6/6 mil on 4 layers; 0.4 mm WQFN; assembly lead time |

Sources: wellpcb.com (home, /capabilities/, /pcb-assembly-quote/, /contact-us/);
avanticircuits.com (home, /custom-pcb-manufacturing/pcb-quote, /pcb-fabrication-and-assembly,
/custom-pcb-manufacturing/prototype-pcb).

## Through-hole alternative (checked 2026-10-07)

- **No through-hole packages.** Per their datasheets, the BQ76922 (RSN, 32-pin QFN), the
  TPS4811x-Q1 (DGX, VSSOP-19) and the CSD17556Q5B (SON 5 × 6 mm) come in one surface-mount
  package each.
- **Evaluation boards only.** Populated boards for these chips are TI's evaluation modules,
  sized for a bench rather than a robot. No third-party breakouts were found.
  - The BQ76922EVM is out of stock at TI.
  - The BQ76942EVM and BQ76952EVM are $328.38 each at Digi-Key.
  - The TPS48111Q1EVM is set up by jumpers.
- **A ready-made BMS.** A through-hole build would use a module such as the
  [JBD 3S smart BMS](https://www.lithiumbatterypcb.com/product/3s-12-6v-or-4s-lifepo4-14-6v-16-8v-lithium-ion-smart-bluetooth-bms-with-uart-and-rs-485-communication-function/):
  - $35.20–$46.60, in 20/30/50/80 A versions;
  - 123 × 63 × 12 mm;
  - Bluetooth, UART, RS485 and isolated CAN;
  - it switches the pack's negative lead.

  The servo E-stop would then be a relay, design record §11 D3 alternative.

## Not yet done

- **A firm quote.** That needs the Gerbers, BOM and placement files
  (`pcb/power-board/fab/`) uploaded to a PCBWay account for engineering review.
- **Shipping.** It is not priced; the instant quote adds it only once a destination is
  chosen.
