# Bipedal Robot — Sourced Bill of Materials

*Compiled 2026-07-11. Prices in USD, delivering to Lyons, CO 80540.*

Robot: ~34 cm, ~0.9 kg 3D-printed biped, 8× Feetech STS3215 serial-bus servos.
BOM source: [`cad/README.md`](../cad/README.md). Print settings and assembly
order also there.

> **Battery protection parts are in [`bom-supplemental.md`](bom-supplemental.md)**
> (added 2026-07-27): a balance-lead low-voltage alarm, an inline fuse, the bulk
> cap, and a watt meter. The robot works without them — that file exists because
> nothing in this BOM's power path has any protection in it, and an RC LiPo has
> none inside it either. ~$31 for the protection tier.

---

## Consolidated BOM

| # | Component | Qty | Product | Unit Price | Extended | Source | Notes |
|---|-----------|-----|---------|-----------|----------|--------|-------|
| 1 | Feetech STS3215 servo | 8 (+1–2 spares) | ST3215 Series Serial Bus Servo (**30 kg·cm@12 V version**) | $21.99 | $175.92 | [Waveshare](https://www.waveshare.com/st3215-servo.htm) | ⚠️ **Buy the 12 V version only.** The $16.99 "7.4 V" class is rated **4–7.4 V** and cannot run on 3S — the sim-recommended battery ([hardware-order.md](hardware-order.md)). The 12 V version is rated 6–12.6 V and works at either voltage. Amazon alt: [B0FGNTDV3Y](https://www.amazon.com/dp/B0FGNTDV3Y?tag=tommwhipple-20) ($29.99, 12V). |
| 2 | Waveshare ESP32 Bus Servo Driver | 1 | **Servo Driver with ESP32** (65 × 30 mm) | $24.99 | $24.99 | [Waveshare](https://www.waveshare.com/servo-driver-with-esp32.htm) / [Amazon B0CFY34BX5](https://www.amazon.com/dp/B0CFY34BX5?tag=tommwhipple-20) | ⚠️ **Corrected — the previously listed [Bus Servo Driver HAT (A)](https://www.waveshare.com/bus-servo-driver-hat-a.htm) ($18.99) is the wrong board**: it's a Raspberry-Pi HAT (65 × 57 mm — doesn't fit the tower, which is built for the 65 × 30 board) and takes 9–25 V (can't run from 2S at all). The Servo Driver with ESP32 is 6–12.6 V (2S *and* 3S direct per Waveshare docs), holes Ø2.75 on 58 × 23 — matches `cad/dimensions.py`. Schematic + user manual: [`docs/datasheets/servo-driver-esp32/`](datasheets/servo-driver-esp32/). |
| 3 | 3S LiPo battery (**decided 2026-07-11**; re-sourced 2026-07-15) | 2 | Tattu 850 mAh 11.1 V 75C 3S1P XT30 | ~$15 | ~$30 | [search](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30&tag=tommwhipple-20) · [B07218SB7L](https://www.amazon.com/dp/B07218SB7L?tag=tommwhipple-20) | ✅ 60 × 30 × 23 mm, 80 g (cross-checked at 3 stockists; 80 g = the sim's `BATT_PACK_MASS`). The original Zeee (67 × 30 × 18.5) went dead on Amazon mid-project, so the bay envelope (`BATT` in `cad/dimensions.py`) is now a **superset of the whole 3S 850 XT30 class** — buy to the [spec filter](bom-by-vendor.md#notes--caveats), not an ASIN. Height is the binding axis. **11.1 V only — not 11.4 V HV** (13.05 V charged > the servo's 12.6 V ceiling). |
| 4 | Bambu Lab A1 mini 3D printer | 1 | Bambu Lab A1 mini + LED Lamp Kit | $219.00 | $219.00 | [Amazon](https://www.amazon.com/dp/B0GQMJ8QQT?tag=tommwhipple-20) | 180×180×180 mm build. AMS lite combo: [B0CRYZWJLG](https://www.amazon.com/dp/B0CRYZWJLG?tag=tommwhipple-20) ($349). Direct: [bambulab.com](https://bambulab.com). |
| 5 | PLA filament 1 kg | 1 | Bambu Lab PLA 1.75 mm 1 kg (Jade White) | $28.00 | $28.00 | [Amazon](https://www.amazon.com/dp/B0CGQYRSNT?tag=tommwhipple-20) | RFID-enabled for A1 mini auto-settings. Black: [B0CGR29R63](https://www.amazon.com/dp/B0CGR29R63?tag=tommwhipple-20) ($28.60). |
| 6 | PETG filament 1 kg | 1 | Bambu Lab PETG Translucent 1.75 mm 1 kg | $30.15 | $30.15 | [Amazon](https://www.amazon.com/dp/B0F68FKRWH?tag=tommwhipple-20) | With reusable spool. Refill-only (no spool): [B0FRQ9VX2K](https://www.amazon.com/dp/B0FRQ9VX2K?tag=tommwhipple-20) ($23.99). |
| 7 | TPU filament 500 g | 1 | Geeetech TPU 1.75 mm 500 g Shore 95A | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0DG8BZL6L?tag=tommwhipple-20) | Bambu TPU is 1 kg only ($42.99, [B0GG283YLZ](https://www.amazon.com/dp/B0GG283YLZ?tag=tommwhipple-20)). |
| 8 | M3×6 button head screws | 32 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes M3×6/8/10/12/16/20/25/30 mm. |
| 9 | M3×6 button head (idlers, no washer) 24 + M3×8 (hip-roll idler) 8 |  32 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes nuts & washers. |
| 10 | M3×10 button head screws | 16 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes M3×6/8/10/12/16/20/25/30 mm. |
| 11 | M3×8 self-tapping screws | 52 | 100 pc M3×8 mm Self-Tapping SS, Flat Head Hex | $8.28 | $8.28 | [Amazon](https://www.amazon.com/dp/B0F9XYX9BQ?tag=tommwhipple-20) | 100 pcs (52 needed). Includes drive bit. Alt: [520pc assortment](https://www.amazon.com/dp/B0BPM8J5F5?tag=tommwhipple-20) ($7.99). |
| 12 | M3 heat-set inserts (Ø4.6×4–6 mm) | 12 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes brass heat-set inserts. |
| 13 | M5×20 GoPro thumbscrew | 1 | M5 Handle Thumb Screws (pair, GoPro) | $7.22 | $7.22 | [Amazon](https://www.amazon.com/dp/B0BCJRFCLX?tag=tommwhipple-20) | Pair; only 1 needed. Stainless, GoPro Hero 4–13. |
| 14 | M2.5×8 self-tapping screws | 4 | uxcell 50 pc M2.5×8 mm Self-Tapping | $8.07 | $8.07 | [Amazon](https://www.amazon.com/dp/B01KXTTSCI?tag=tommwhipple-20) | 50 pcs (4 needed). 304 SS. |
| 15 | M3 screw + heat-set insert kit | 1 | KADRICK 420 pcs M3 Heat Set Inserts Kit | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0GYRQG7F2?tag=tommwhipple-20) | **Covers items 8, 9, 10, 12.** M3×6–30 mm socket cap screws, brass inserts, nuts, washers, installation tip, hex key. |
| 16 | TPU sheet 2 mm | 2 | BENECREAT 2-pc Silicone Rubber Sheet 2 mm | $15.59 | $15.59 | [Amazon](https://www.amazon.com/dp/B08P7P69WQ?tag=tommwhipple-20) | 2 sheets. Cut to 90×46 mm for foot pads. Silicone rubber ≈ TPU functionally. |
| 17 | Zip ties 2.5 mm | ~10 | Zip Ties 2.5 mm×200 mm 100 pc | $3.99 | $3.99 | [Amazon](https://www.amazon.com/dp/B0GR52PRHF?tag=tommwhipple-20) | 100 pcs (~10 needed). Black nylon, 30 lb. |
| 18 | 9-DOF IMU | 1 | Teyleten Robot GY-BNO085 9DOF AHRS module | $20.99 | $20.99 | [Amazon B0CL26J81F](https://www.amazon.com/dp/B0CL26J81F?tag=tommwhipple-20) | **Actual part ordered** (2026-07-16). BNO085 chip (Hillcrest Labs CEVA SH-2 fusion on-chip). Same IC as the planned Adafruit #4754 breakout but ~$4 cheaper; I2C/SPI/UART/UART-RVC interfaces. Datasheet: [`docs/datasheets/BNO08X-datasheet.pdf`](datasheets/BNO08X-datasheet.pdf) (Hillcrest Labs BNO08X v1.16, 59pp). Note: non-STEMMA breakout — pin headers instead of JST-SH jack, so item 21 cable may need swapping for jumper wires. |
| 19 | Servo bus extension leads, 3-pin ≥200 mm | 2 (+2 spares) | Feetech/Waveshare 3-pin bus servo cable, 200–300 mm | ~$1–2 | ~$8 | [search](https://www.amazon.com/s?k=feetech+3+pin+servo+cable+200mm&tag=tommwhipple-20) · [Waveshare](https://www.waveshare.com/product/robotics/accessories.htm) | For the **hip-roll → hip-pitch** hop (one per leg): worst-pose routed path is **170 mm** (measured in `cad/dress.py` across the full ROM incl. slack loop), so the 150 mm leads that ship with the servos are too short *there and only there*. ≥200 mm gives ~18 % flex margin; if only 300 mm is stocked, zip-tie the excess at the thigh raceway anchors. Every other hop fits a stock 150 mm lead (see [wiring.md](wiring.md)). |
| 20 | XT30 pigtail + inline switch + heat-shrink | 1 set | XT30 pigtail (mates the pack lead) → inline switch → bare ends into the board's screw terminal | ~$10 | ~$10 | [search](https://www.amazon.com/s?k=xt30+pigtail+connector+inline+switch&tag=tommwhipple-20) | Battery → power switch → driver-board screw terminal. Routed run is only ~30 mm, so any short pigtail works; 20 AWG or thicker (bus transient budget ~10 A). |
| 21 | IMU cable, JST-SH 4-pin → male jumpers | 1 | Adafruit STEMMA QT / Qwiic to male headers, 150 mm | $0.95 | $0.95 | [Adafruit 4209](https://www.adafruit.com/product/4209) | BNO085 → ESP32 GPIO 21/22 + 3V3/GND. The driver board has no STEMMA jack, so this (or 4 soldered wires) bridges it. IMU mounts on the deck next to the board — 150 mm is ample. |
| 22 | **General Driver swap:** power pigtail, JST XH 2-pin | 1 (+1 spare) | Pre-crimped JST-XH 2.54 mm 2-pin female pigtail (housing XHP-2 + SXH contacts), 22 AWG, ≥100 mm | ~$1–2 | ~$6 (sold in packs) | [search](https://www.amazon.com/s?k=jst+xh+2.54+2+pin+connector+pigtail+female+22awg&tag=tommwhipple-20) | The General Driver's power inlet is **XH2.54, not a barrel jack** ([official docs index](datasheets/general-driver/README.md), incl. the manufacturer's connector diagram). Splice onto the item-20 switch lead (solder + heat-shrink; XH crimps take **max 22 AWG**, so the 20 AWG lead can't go into the housing directly). ⚠️ XH contact is rated **3 A**; the board's own bus-servo path is FAQ-limited to **5 A continuous ("up to 5 ST3215s, not stalled simultaneously")** — bimo runs 8 with a ~10 A transient budget. Resolve before the swap goes on hardware (options: accept brown-out risk with per-leg current shaping, or bypass the XH inlet and solder to the pads). ⚠️ Verify polarity against the board silkscreen before first plug — pigtail wire colors are not standardized. Not a 2S "balance lead" (that's XH **3**-pin). |

---

## Cost Summary

| Category | Estimated Cost |
|----------|---------------|
| Electronics (8 servos + driver + battery + IMU) | ~$245 |
| Servo spares (1–2 recommended) | $22–44 |
| 3D printer | $219.00 |
| Filament (PLA + PETG + TPU) | $74.14 |
| Hardware (screws, inserts, thumbscrew, sheet, zip ties) | $59.14 |
| Wiring (bus extensions, XT30 pigtail + switch, IMU cable) | ~$19 |
| **Total Estimated** | **~$640–660** |

**Not yet on this list** (from [hardware-order.md](hardware-order.md)): 2S–3S
balance charger (~$30, skip if owned) and a USB-C **data** cable for
flashing/debug (skip if owned). Also note items 7 and 16 are redundant — foot pads are *printed* TPU
(item 7); the silicone sheet (item 16, $15.59) can be dropped unless you want
a fallback. The KADRICK kit is socket-cap, not button-head — check head
clearance at the idler screws before relying on it there.

### Servo cost optimization — ~~retracted~~

~~Using Waveshare 7.4V servos ($16.99 × 8 = $135.92) saves ~$40.~~
**Don't**: the 7.4 V class is rated **4–7.4 V only** (per Waveshare's own
listing), so it permanently forecloses the 3S option that the sim gauntlet
recommends (2.72 s vs 4.36 s dash; 16/16 vs 12/16 with the GoPro on rough
ground — see [hardware-order.md](hardware-order.md)). The $40 delta buys the
ability to choose the battery later. Buy 8 + 1–2 spares of the 12 V version.

---

## Notes

- **Items 8, 9, 10, 12 consolidated** into the KADRICK 420 pcs kit ($15.99), which
  includes M3×6–30 mm screws, brass heat-set inserts, nuts, washers, installation
  tip, and hex key. This single kit covers all M3 button head screws, M3 washers,
  and M3 heat-set inserts in the BOM.
- **Battery dimensions** — must verify the specific battery fits ≤ 75×35×16 mm
  before purchase. RC car batteries vary; check product dimensions carefully.
- **TPU sheet** — true TPU sheet was not found on Amazon. Silicone rubber sheet
  (2 mm) is the closest functional equivalent for grip pads / foot soles.
- **Bambu Lab store** (bambulab.com) was blocked by Cloudflare; Amazon prices
  match expected retail ($219 for A1 mini).
- **Waveshare direct** offers the best prices on the two most expensive
  electronic components: ST3215 servos ($16.99–$21.99 vs $24.35+ on Amazon) and
  the ESP32 driver HAT ($18.99 vs $24.99 on Amazon). Shipping from Waveshare
  may take longer than Amazon Prime — check lead time before ordering.
- **DigiKey** was behind Cloudflare bot detection and could not be browsed.
  DigiKey does not appear to carry Feetech servos or Waveshare boards (they are
  hobby robotics parts, not standard electronic components). DigiKey is better
  suited for the fasteners and hardware — search for "M3 button head screw" or
  "heat set insert" at [digikey.com](https://www.digikey.com) directly.
- All prices captured July 11, 2026. Prices and availability subject to change.

---

## Open BOM Questions (from cad/README.md)

- **Servo case thread**: M3 self-tap vs tap vs M4 — measure a real case.
- **Idler screw length**: ~~M3×8 + washer assumes 3.35 mm thread depth~~
  **RESOLVED 2026-07-30** — bench + vendor STEP: the bolt circle taps a 2.1 mm
  flange, not the 3.35 mm body. Re-sized to M3×6 (M3×8 at the hip-roll idler),
  no washers. Asserted in `cad/check_assembly.py`. Original note: confirm
  screws don't bottom against the gear.
- **Driver board**: exact model + hole pattern (`BOARD_HOLES` in `dimensions.py`)
  — confirm component heights and battery clearance.
- **GoPro fit**: measure your MAX's finger thickness — slots are 3.2 mm.
- **Cable service loops**: routed worst-pose lengths measured in `cad/dress.py`
  (2026-07-16) — hip-roll→hip-pitch 170 mm (**needs the item-19 extensions**),
  hip-pitch→knee 111 mm, knee→ankle 82 mm, board→hip-roll 33 mm; all but the
  first fit the stock 150 mm leads with ≥25 % slack. Confirm the shipped lead
  length really is 150 mm when the servos arrive.
- **Foot pad material**: TPU sheet vs printed TPU vs adhesive rubber; friction
  should roughly match sim (μ ≈ 1.0).