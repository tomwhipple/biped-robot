# v6 body: what to buy beyond the v5 robot (2026-09-13)

Companion to `docs/design-v6-ankle-roll.md` §9 and the CAD under `cad/v6/`. The v5 robot's parts that carry over: the 12 STS3215 on hand (7 used: hip yaw ×2, hip pitch ×2, ankle pitch ×2, neck ×1; 5 spare), the General Driver board, the horn/idler M3 hardware and the M2.5 flat-head self-tappers already sourced (counts below), the printer and PETG. Buy to the spec filters, not to ASINs (the battery lesson).

| item | qty | spec / filter | why | est. |
|---|---|---|---|---|
| **Feetech STS3250** (12 V class) | **6** (buy **2 first** for the stiffness bench test, §5.1 of the design doc, then 4) | same 45.2 × 24.7 × 35 case as the STS3215, 50 kg·cm, magnetic encoder, TTL bus | hip roll, ankle roll, knee: the roll-chain stiffness the walk needs; the knee's speed margin | 6 × $43–65 US stock (BABSCO $64.99, WowRobo $43 when in stock), $48–55 AliExpress; Waveshare does not list it (checked 2026-09-14) |
| Raspberry Pi 4B (2–4 GB) | 1 | + 32 GB card; low-profile heatsink (no fan tower: the aft bay is 16 mm deep on the component side) | onboard camera/navigation | on hand? |
| Raspberry Pi Camera Module 3 **Wide** | 1 | 102° HFOV, IMX708, with a 300 mm ribbon (the head is ~30 cm above the Pi's CSI port by cable path) | head camera | $35 |
| 3S LiPo pack | 1–2 | **11.1 V (not 11.4 HV), 2200–2600 mAh, ≤ 105 × 36 × 26 mm, 150–190 g, XT60 or XT30, 25C+** | 2× the v5 pack's energy for 13 servos + Pi (est. 25–35 min walking) | $20–25 each |
| 3S protection board | 1 | ≥ 15 A continuous, over-discharge cut-off ~3.0 V/cell, balance leads; JBD/Daly 3S "smart" (UART) if per-cell telemetry is wanted | pack protection independent of firmware | $8–25 |
| 5 V regulator for the Pi | 1 | Pololu **D24V50F5** (5 V, 5 A, 17.8 × 25.4 mm) or equal; USB-C pigtail | Pi 4B wants 3 A peaks | $20 |
| inline fuse + holder | 1 | 10–15 A mini blade on the pack lead (bom-supplemental.md) | 13 servos on one bus, 3250 stall 4.2 A | $10 |
| bulk capacitor | 1 | 1000 µF ≥ 25 V low-ESR across the bus at the board | brown-out on servo transients | $3 |
| M2.5 × 8 flat-head self-tappers | ~60 (have 24) | 90° countersunk, stainless | 6 per leg link × 4, 6 per ankle link × 2, 4 per foot × 2, 4 neck stators + 4 neck-collar flange, 8 yaw stators | $8 |
| M3 × 6 button head | 40 (have) | horn discs: 4 per joint × 13 + head | | |
| M3 × 8 button head + thin washer | 28 | idler discs: 4 per double-sided joint × 7 | | |
| M2.5 × 6 pan machine screws | 8 | Pi 4B + General Driver standoffs (self-tap bosses) | | |
| M2 × 6 self-tappers | 4 | Camera Module 3 | | |
| TPU 95A filament | 1 spool | soles (2 × ~20 g) | edge compliance / friction | $25 |
| PETG | ~0.6 kg | 11 prints, ≈ 450 g + supports | | |
| 15 mm hook-and-loop strap | 1 | battery belt (v5 pattern) | | |

| **6811-2RS deep-groove ball bearing** | **2 + 2 spares** | **55 x 72 x 9 mm, sealed (2RS), chrome steel (SAE 52100), "68xx" deep-groove series** | hip-yaw thrust + moment (`docs/design-v6/study-yaw-bearing.md`) -- lets the pelvis housing carry the leg's load instead of the yaw servo's horn screws, both races located (interference, not clearance) | **US stock, confirm price in the browser (2026-09-17 search):** [VXB single](https://vxb.com/products/6811-2rs-bearing-55x72x9-sealed) · [Bearings Direct single](https://bearingsdirect.com/6811-2rs-ball-bearing-55x72x9-sealed/) · [Amazon MAPLE ACE single](https://www.amazon.com/MAPLE-ACE-6811-2RS-61811-2RS-55x72x9mm/dp/B0G6HPGNHS) · [Amazon 10-pack](https://www.amazon.com/Bearing-6811-2RS-61811-2RS1-55x72x9-Bearings/dp/B0F28ZTFWQ); the VXB-brand 6811LU on Amazon listed at $49.40 is overpriced for this class. Alternative if stocked: 6711-2RS (55 x 68 x 7, same bore, 2 mm thinner). Per-size table below |

Not needed: a Dynamixel bus or board; a UPS HAT (the Waveshare UPS Module 3S's pack output is 2 A, an order of magnitude under the servo bus); a GoPro mount.

Open items for the side project (power telemetry to the Pi): (a) confirm the General Driver's INA219 (0x42) readings are forwarded over the link at ≥ 1 Hz — that is the bus current/voltage the Pi needs for a low-battery shutdown; (b) decide whether per-cell voltage is wanted (then a UART smart BMS replaces the plain protection board; the pelvis envelope is 60 × 12 × 25 mm beside the pack).

## Sourcing appendix

Pulled by rezzy (collective) 2026-09-17, 6811 rows added by Claude Code the same day. One US-stock link per size, sealed 2RS, chrome steel unless noted. **Confirm price and stock in your own browser before ordering**; listings rotate. Final pick is the **6811-2RS**; the 6711-2RS is the alternative if stocked; the smaller sizes are kept only as the record of what the CAD study rejected (`study-yaw-bearing.md` §3).

### Hip-yaw bearing sourcing (per size)

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

Notes:
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
