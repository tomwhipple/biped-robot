# Yaw-Joint Bearing Sourcing — design-v6

Pulled 2026-09-17. One US-stock purchase link per size, sealed 2RS, chrome steel (or
stainless where noted). Prices/links captured from vendor pages on 2026-09-17 —
**confirm price and stock in your own browser before ordering** (cached/residual
prices may be stale; listings rotate).

CAD study result (2026-09-17): **6811-2RS**, see the bottom rows and `bom-delta.md`.

## Candidates

| Size      | ID x OD x W (mm)  | Qty/pack | Price (USD) | Vendor / link                                                        | Material / notes                                |
|-----------|-------------------|----------|-------------|----------------------------------------------------------------------|-------------------------------------------------|
| 6704-2RS  | 20 x 27 x 4       | 10 pcs   | $14.99      | Amazon — XIKE 6704-2RS, https://www.amazon.com/dp/B09D2ZM9CZ         | Chrome steel, double rubber seals, pre-lubricated |
| 6804-2RS  | 20 x 32 x 7       | 10 pcs   | $9.99       | Amazon — HiPicco 6804-2RS, https://www.amazon.com/dp/B0C74VVNJC      | Chrome steel, double rubber seals, pre-lubricated |
| 6706-2RS  | 30 x 37 x 4       | 2 pcs    | $8.89       | Amazon — uxcell 6706-2RS, https://www.amazon.com/dp/B082PS9PHD       | Chrome steel, double rubber sealed               |
| 6806-2RS  | 30 x 42 x 7       | 1 pc     | $9.99       | VXB — https://vxb.com/products/6806-2rs-bearing-30x42x7-sealed       | Chrome steel EMQ, dual rubber seals, greased     |
| 6707-2RS  | 35 x 44 x 5       | 2 pcs    | $9.19       | Amazon — uxcell 6707-2RS ABEC5, https://www.amazon.com/dp/B0FY5NL6CX | Chrome steel, double rubber seals, pre-lubricated |
| 6807-2RS  | 35 x 47 x 7       | 1 pc     | $14.99      | VXB — https://vxb.com/products/6807-2rs-bearing-35x47x7-sealed       | Chrome steel EMQ, dual rubber seals, greased     |
| 6708-2RS  | 40 x 50 x 6       | 1 pc     | $29.99      | VXB — https://vxb.com/products/6708-2rs-sealed-metric-slim-bearing   | Chrome steel 52100, 2 rubber seals, greased      |
| 6808-2RS  | 40 x 52 x 7       | 1 pc     | $16.59      | Amazon — HARFINGTON 6808-2RS 10 pcs, https://www.amazon.com/dp/B0CGM2PG3Z | Chrome steel, double sealed, high speed          |
| **6811-2RS** (FINAL PICK, CAD 2026-09-17) | 55 x 72 x 9 | 1 pc | not captured | VXB — https://vxb.com/products/6811-2rs-bearing-55x72x9-sealed ; Bearings Direct — https://bearingsdirect.com/6811-2rs-ball-bearing-55x72x9-sealed/ ; Amazon MAPLE ACE — https://www.amazon.com/MAPLE-ACE-6811-2RS-61811-2RS-55x72x9mm/dp/B0G6HPGNHS ; Amazon 10-pack — https://www.amazon.com/Bearing-6811-2RS-61811-2RS1-55x72x9-Bearings/dp/B0F28ZTFWQ | Chrome steel, 2RS; links from Claude Code web search 2026-09-17, prices not captured (rezzy follow-up pending on card t_67c6388f) |
| 6711-2RS (alternative, same bore) | 55 x 68 x 7 | — | — | not yet sourced (rezzy follow-up, card t_67c6388f) | thinner/lighter; recess is parametric (`YAW_BRG_*`) |

Notes
- VXB ships from Anaheim, CA — US stock.
- Amazon "XIKE" / "uxcell" / "HiPicco" / "HARFINGTON" listings are
  fulfilled-by-Amazon US stock; the listed pack quantities are per offer
  (e.g., the HARFINGTON 6808-2RS is a 10-pc pack at $16.59).
- For CAD's first pick (6704-2RS), the cheapest unit-cost option above is the
  XIKE 10-pack at $14.99 (~$1.50/pc). If only a single pair is needed,
  uxcell 5-pack at $11.99 or FKG 2-pack at $4.97 are alternates on the
  same ASIN family.
- Stainless variants exist (VXB lists S68xx-2RS stainless at higher cost) —
  not required unless Tom specifies stainless.
- Bearings Direct and McMaster-Carr did not yield live pages for the slim
  6704/6706/6804 sizes via headless crawl; if you want a domestic industrial
  brand quote instead of Amazon-import, ask me to follow up in a logged-in
  browser session.
