# Power board rev B: PCBWay's reviewed quote for 2 and 3 boards (2026-11-02)

*A dated record. The board is [pcb/power-board](../../pcb/power-board/README.md) at rev B
([record](2026-10-29-power-board-rev-b.md), PR #166). The earlier, unreviewed prices are in
[2026-10-24-power-board-quote.md](2026-10-24-power-board-quote.md). Tom, 2026-10-29: "Let's get
the quote" (2 against 3 boards) and "Go ahead and upload and get a quote". All prices are in USD,
as PCBWay's cart showed them on 2026-11-02.*

## Summary

PCBWay reviewed rev B's files and priced both quantities. These prices exclude shipping, which
the cart adds at checkout.

| | 2 boards | 3 boards |
|---|---|---|
| bare boards, 5 pieces (PCB item) | $62.04 | $62.04 |
| assembly | $29.00 | $88.00 |
| components, turnkey | $127.50 | $149.09 |
| **total before shipping** | **$218.54** | **$299.13** |
| per assembled board | $109.27 | $99.71 |
| lead time PCBWay shows ("PCB+Assembly+IC") | 30–35 days | 30–35 days |

- **The third board costs $80.59:** $59.00 more assembly and $21.59 more parts. The parts
  difference is small because PCBWay's unit prices drop at 3 boards. A board's parts are
  $63.75 at 2 boards and $49.70 at 3.
- **The two assembly prices are not consistent.** The 3-board line is the $88.00 the form
  calculated for either quantity before the review. The 2-board line came back at $29.00, and
  its quoted BOM leaves the assembly and PCB rows blank. PCBWay gave no reason; ask the sales
  rep before relying on the $29.00.
- **Against the 2026-10-24 estimate** (about $203 for 2 and $233 for 3, shipping netted out):
  the parts cost more than the $30 a board estimated.
- **Nothing is ordered or paid.** All four items are "Pass, Payment" in the cart; Tom pays for
  one quantity and removes the other.

## What was submitted

On 2026-10-30/31, through PCBWay's assembly quote form, one cart item per quantity:

- **Bare boards:**
  - 4 layers, 97.1 × 64.6 mm (PCBWay measured 97 × 64.5 mm), 5 pieces;
  - FR-4 S1000H TG150, 1.6 mm, 6/6 mil, 0.3 mm minimum hole;
  - ENIG 1 µ", green / white, tented vias;
  - 1 oz copper on all four layers;
  - layer order F_Cu, In1_Cu, In2_Cu, B_Cu.
- **Assembly:**
  - turnkey, top side, single pieces;
  - 52 unique parts, 88 SMD, 2 fine-pitch, 12 through-hole;
  - no function test, no firmware, no substitutes made in China.
- **Files:** `fab/power-board-gerbers.zip`, `fab/power-board-pos.csv`, the BOM in PCBWay's
  template (52 lines, 100 parts, from `fab/power-board-bom.csv`), and the schematic PDF, for the
  polarity check.
- **The note to the assembler:**
  - J1, F1 and F2 not populated;
  - the placement file's Y negated, and through-hole origins at pin 1;
  - a polarity preview for approval before assembly;
  - a 0.10–0.12 mm stencil;
  - J1's 0.25 mm overhang, so no rail on that edge;
  - no substitutions.

| | 2 boards | 3 boards |
|---|---|---|
| PCB item | W1183348AS7C1 | W1183348AS1N3 |
| assembly item | T-7C2W1183348A | T-1N4W1183348A |

## The quoted BOMs

PCBWay's quoted BOMs are converted, line for line, to
[2026-11-02-power-board-rev-b-quote-bom-2.csv](2026-11-02-power-board-rev-b-quote-bom-2.csv) and
[…-3.csv](2026-11-02-power-board-rev-b-quote-bom-3.csv).

- **Every line is quoted at the MPN on the BOM.** Nothing is substituted.
- **Eight lines are $121 of the 3-board run's $149:**

  | parts | MPN | each, 3-board run | each, 2-board run |
  |---|---|---|---|
  | Q1–Q3 | CSD17556Q5B | $2.68 | $2.82 |
  | U1 | BQ76922RSNR | $7.45 | $8.39 |
  | R1 | CSS2H-2512R-1L00FE | $5.07 | $7.74 |
  | R24 | CSS2H-2512K-2L00F | $5.06 | $7.59 |
  | U2 | TPS48111LQDGXRQ1 | $4.16 | $4.53 |
  | C2 | C1206C225K5RACTU | $3.91 | $5.61 |
  | C21 | EEU-FR1E102 | $3.65 | $5.48 |
  | C22, C24–C27 | CL31B106KLHNNNE | $0.59 | $0.76 |

- **Seven lines show 7–10 working days to source:** C21, D1, D8/D9, J10, R1, R24 and U1.
- **PCBWay's two notes, the same on both quotes:**
  - D1, PMEG4002EJ-QX: "Only 300pcs total in stock, please confirm ASAP."
  - J4–J8: "The quoted part number is [22-03-5035], not [5267-03A] as specified in
    description". That text comes from the KiCad footprint's name. Distributors list 5267-03A
    as an alias of 22-03-5035 (Heilind, Octopart), so the quoted part is the one specified.

## Open before paying

- **Choose 2 or 3 boards.** Pay for that pair of items and remove the other.
- **Ask PCBWay's sales rep** why the 2-board assembly is $29.00 and the 3-board $88.00.
- **Answer the two notes:** D1 is confirmed as quoted, and J4–J8's 22-03-5035 is the intended
  part.
- **Shipping** is added at checkout. Each pair weighs about 1.0 kg in the cart. The instant
  quote's DHL was $28.95 at 0.61 kg, offset by a PCB-assembly freight discount of up to $30.
