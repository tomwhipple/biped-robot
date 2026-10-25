# Power board: production quotes for 2–5 assembled boards (2026-10-24)

*A dated record. The board is [pcb/power-board](../../pcb/power-board/README.md) at layout rev A
(97.1 × 64.6 mm); the earlier round is [2026-10-07-power-board-quote.md](2026-10-07-power-board-quote.md).
Tom asked for "quotes for production of at least 2 copies of the board", and to implement any
meaningful cost reduction. All prices are in USD, as shown on 2026-10-24 (the vendors' pages
dated them 2026-10-25, GMT+8).*

## Summary

- **The cheapest route is PCBWay with 1 oz outer copper:**
  - about **$203 for 2 assembled boards**, $233 for 3 and $301 for 5;
  - that is the quoted board and assembly, shipping (netted to $0 by an order discount),
    and an estimate for the parts;
  - customs duties and VAT are not included.
- **None of these is a firm price.** PCBWay says "Final price is subject to our review".
  JLCPCB shows an assembly price only after a Gerber upload. Nothing was uploaded, and no
  account, cart, order or RFQ was used.
- **Cost reduction made: 1 oz outer copper.**
  - 2 oz outer at the 6/6 mil this board needs adds about **$264** at PCBWay, whatever the
    quantity ($319.82 against $55.93 for the five bare boards).
  - The verification record's copper analysis (C11) finds 1 oz enough: the copper is not
    the thermal limit.
  - The layout's stackup is now 1 oz on all four layers, with a 1.04 mm core to keep
    1.6 mm, and the fab outputs are regenerated (`pcb/power-board/README.md`).
- **A third board costs only its parts.** PCBWay's assembly is the same $88.00 for 2 or 3
  boards, and bare boards come in fives anyway. Three boards are about $30 more than two.
- **No other lever is worth taking now:**
  - JLCPCB is dearer this round: 37 of the board's 39 unique SMD parts are "Extended" at
    $3.07 each;
  - replacing the CSD17556Q5Bs (the largest parts line, about $10.68 a board) would
    invalidate the FET simulations;
  - the rev B changes add about $0–4 of assembly and $0.13–1.22 of parts.

| assembled boards (5 bare boards made in each case) | 2 | 3 | 5 |
|---|---|---|---|
| **PCBWay, 1 oz**: board + assembly + shipping (quoted) | $143.93 | $143.93 | $152.80 |
| … + parts (est.) = **total** | **≈ $203** | **≈ $233** | **≈ $301** |
| PCBWay, 2 oz at 6/6 mil | ≈ $467 | ≈ $497 | ≈ $565 |
| JLCPCB, 1 oz: board + UPS DDP shipping (quoted), Economic assembly (rate card, est.), parts (est.) | ≈ $271 | ≈ $307 | ≈ $375 |
| JLCPCB, 2 oz | ≈ $306 | ≈ $342 | ≈ $410 |

## The board as quoted

These counts were taken from `power-board.kicad_pcb` and `fab/power-board-{bom,pos}.csv`:

- 97.1 × 64.6 mm, 4 copper layers, 1.6 mm FR-4, ENIG, green mask, white silk.
- Tracks and spaces of 0.15 mm (6/6 mil). 438 vias with 0.3 mm minimum drill: 428 at
  0.6/0.3 mm (0.150 mm annular ring) and 10 at 0.8/0.4 mm.
- 94 placed parts, all on top: 79 SMD (271 SMD pads, 218 distinct pins) and 15 through-hole
  (C21, F1, F2, J1–J12) with 53 hand-soldered joints. U1's 9 plated pads are thermal vias,
  not joints.
- 50 unique parts: 48 soldered MPNs plus the 2 plug-in fuses. Two are QFN-class (U1,
  U2), and four are leadless (U1, Q1–Q3).

## PCBWay: public instant quote

**Settings.** These were read back from the request the page sent, on every run:
- 4 layers, 97.1 × 64.6 mm, 5 pieces, 1.6 mm FR-4 (S1000H TG150), 6/6 mil, 0.3 mm minimum
  hole;
- green / white, immersion gold 1 µ", tented vias, inner copper 1 oz;
- turnkey assembly, top side, with 50 unique parts, 79 SMD, 2 QFP-class and 15
  through-hole parts;
- no function test, no firmware loading, no China-made alternatives.

| | 2 assembled | 3 assembled | 5 assembled |
|---|---|---|---|
| bare boards, 1 oz (5 pieces, 4–5 days) | $55.93 | $55.93 | $55.93 |
| bare boards, 2 oz at 6/6 mil (9–10 days, no express) | $319.82 | $319.82 | $319.82 |
| turnkey assembly, labour and setup (5–6 days) | $88.00 | $88.00 | $96.87 |
| DHL to the US, 2–4 days | $28.95 | $28.95 | $28.95 |
| "Order discount" (PCB-assembly freight deduction) | −$28.95 | −$28.95 | −$28.95 |

What the page did:

- **2 oz copper.** Picking 2 oz switches track/space to 8/8 mil on its own ($165.54). This
  board cannot use 8/8, because the BQ76922's pad gaps are 0.2 mm. Set back to 6/6, it costs
  $319.82.
- **The finish trap.** Clicking the 0.3 mm minimum-hole option silently resets the finish to
  HASL. The readback caught it, and every run reported here sent ENIG.
- **A special price.** The 2- and 3-board assembly price carries the page's special-price
  flag. The "$29 assembly" offer the page links to caps through-hole joints at 100 per
  order, and two of these boards have 106, so treat $88.00 as provisional.
- **Duties.** "Customs duties and VAT are not included!"

## JLCPCB: public quote page, plus its assembly rate card

**Bare boards (5 pieces, 3–4 days).** The settings match the PCBWay order.
- With 1 oz outer copper: **$42.10**.
- With 2 oz: **$77.30**.
- The page pre-selects a $66.20 expedite; it was switched back to standard.
- "Tented" vias are disabled at 4 layers, so the quote uses "Plugged" at $0.

**Shipping.** UPS Express Saver, duties paid, to the US: $26.77.

**Assembly.** The page asks for a Gerber upload first, so this is estimated from the
published rate card (updated 2026-10-22):

| | 2 boards | 3 boards | 5 boards |
|---|---|---|---|
| Economic PCBA, est. | $142.61 | $148.83 | $157.99 |

What drives that estimate:

- **Extended parts.** 37 of the 39 unique SMD parts are "Extended" at $3.07 each. That
  alone is $113.59.
- **X-ray.** It is charged for the 4 leadless parts on each board.
- **Standard PCBA** would compute cheaper, but the card's handling-fee rule makes it at
  least $14.93 dearer than Economic.

**Flags.**

- **Annular ring.** JLCPCB lists a 0.15 mm multilayer annular ring as its absolute minimum
  at 1 oz, and 0.254 mm at 2 oz. The board's 428 vias have 0.150 mm. A 2 oz board there is
  below its figure, and 1 oz sits at its limit.
- **Pin spacing.** Economic PCBA lists a 0.4 mm minimum IC pin spacing, which is exactly
  the BQ76922's pitch.

## Parts and stock

- **The parts estimate.** Digi-Key's bot check blocked automated pages after the first
  one, so the parts line reuses the 2026-10-07 Digi-Key estimate: $29.68 a board, or
  $59.36 / $89.04 / $148.40 for 2 / 3 / 5 boards. At 2–3 boards real unit prices sit at
  higher breaks.
- **A cross-check from today's LCSC listings:** about $59 / $78 / $110.
- **The controller ICs.**
  - BQ76922RSNR: 1,465 at Digi-Key ($2.92), with a 26-week factory lead time behind
    them; LCSC now lists it, 178 in stock.
  - TPS48111LQDGXRQ1: 6,740 at Digi-Key ($5.35), with a 12-week lead time; none at
    LCSC.
- **Out of stock at LCSC today:** XT60PW-M (TME has none either; 10,484 due 2026-11-25),
  CSD17556Q5B, Keystone 3568, the 1 mΩ shunt, SMBJ15A and PMEG4002EJ-QX. PCBWay buys
  turnkey parts itself, so its parts review will show whether any of these delays the
  order. The XT60 is the one to watch.
- **The through-hole parts among them are soldered by hand** (Tom, 2026-10-25). J1 and
  F1/F2's holders go on the assembly order as do-not-populate, and are bought from stock
  found elsewhere. Pages read on 2026-10-25:

  | part | source | stock | price |
  |---|---|---|---|
  | J1: Amass XT60PW-M | Jameco 2844439 (maker: Amass) | 301, and 1,500 more shipping 2026-11-05 | $0.79 (1+), $0.69 (10+), plus Jameco's 20 % tariff charge |
  | F1, F2: Keystone 3568 holder | Digi-Key 36-3568-ND | 98,603 | $1.31 (1), $0.989 (10+) |

  - For 2–3 boards: 4 XT60PW-M (one spare) and 10 holders, about $13 before shipping
    and the tariff charge.
  - Elsewhere on 2026-10-25: LCSC's stock API showed 0 XT60PW-M, TME 0, and Digi-Key
    does not list the part. Win Source, an independent distributor, lists 20,000; it was
    not used, because Jameco names Amass as the maker and links Amass's datasheet.
  - The fuses themselves (Littelfuse MINI blades) plug in by hand anyway (C1 of the
    layout review).

## Before ordering

1. **Upload the files for a firm quote.** The Gerbers, BOM and placement files in
   `pcb/power-board/fab/` go to a PCBWay account for engineering review. That publishes the
   design to PCBWay, so it is Tom's call, from his account.
2. **Make the rev B changes and re-quote.** The rev B list is in `pcb/power-board/README.md`,
   from the [simulation record](2026-10-24-power-board-full-simulation.md) and the
   [pin audit](2026-10-24-power-board-pin-audit.md).
   - Rev B adds about 6 parts and 12 pads, with no net change in unique parts.
   - It does not change the outline.
3. **Decide 2 or 3 boards.** Three cost about $30 more, in parts only.

## How it was priced

- **The pages were driven by scripts** (Playwright, in a scratch environment under `build/`,
  not kept in git).
- **Every setting was read back** from the request the page sent before a price was taken.
- **Raw evidence.** Request and response logs, screenshots, the JLCPCB rate card and
  capabilities pages, and the LCSC and TME listings were kept under
  `build/pbfull/quotes/raw/` on the machine that ran it.
- **What counts as an estimate.** Lines marked "est." are computed from a rate card or
  from product pages. Every other figure is what the vendor's page showed.
