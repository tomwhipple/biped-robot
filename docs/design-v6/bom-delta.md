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

Not needed: a Dynamixel bus or board; a UPS HAT (the Waveshare UPS Module 3S's pack output is 2 A, an order of magnitude under the servo bus); a GoPro mount.

Open items for the side project (power telemetry to the Pi): (a) confirm the General Driver's INA219 (0x42) readings are forwarded over the link at ≥ 1 Hz — that is the bus current/voltage the Pi needs for a low-battery shutdown; (b) decide whether per-cell voltage is wanted (then a UART smart BMS replaces the plain protection board; the pelvis envelope is 60 × 12 × 25 mm beside the pack).
