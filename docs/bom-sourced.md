# Bipedal Robot — Sourced Bill of Materials

*Compiled 2026-07-11. Prices in USD, delivering to Lyons, CO 80540.*

Robot: ~34 cm, ~0.9 kg 3D-printed biped, 8× Feetech STS3215 serial-bus servos.
BOM source: [`cad/README.md`](../cad/README.md). Print settings and assembly
order also there.

---

## Consolidated BOM

| # | Component | Qty | Product | Unit Price | Extended | Source | Notes |
|---|-----------|-----|---------|-----------|----------|--------|-------|
| 1 | Feetech STS3215 servo | 8 | ST3215 Series Serial Bus Servo (30 kg·cm@12 V) | $21.99 | $175.92 | [Waveshare](https://www.waveshare.com/st3215-servo.htm) | **Best price found.** Waveshare direct: $21.99 (30 kg@12V) or $16.99 (19.5 kg@7.4V). Amazon alt: [B0FMY17QRT](https://www.amazon.com/dp/B0FMY17QRT) ($24.35, 7.4V) or [B0FGNTDV3Y](https://www.amazon.com/dp/B0FGNTDV3Y) ($29.99, 12V). Amazon 6-pack [B0FQHCV9GP](https://www.amazon.com/dp/B0FQHCV9GP) ($134.99). |
| 2 | Waveshare ESP32 Bus Servo Driver | 1 | Serial Bus Servo Driver HAT (ESP32) | $18.99 | $18.99 | [Waveshare](https://www.waveshare.com/bus-servo-driver-hat-a.htm) | **Best price found.** Waveshare direct: $18.99. Amazon alt: [B09SZ41RJW](https://www.amazon.com/dp/B09SZ41RJW) ($24.99). Also: [Serial Bus Servo Driver Board](https://www.waveshare.com/bus-servo-adapter-a.htm) ($4.99, no ESP32 — bare driver only). |
| 3 | 2S LiPo 7.4 V, 450–1000 mAh | 1 | 2S LiPo 7.4 V 1000 mAh 35C (JST) | $18.99 | $18.99 | [Amazon](https://www.amazon.com/dp/B0FFBB59TM) | ⚠️ Verify dims ≤ 75×35×16 mm before ordering. Alt: [OVONIC 2-pack](https://www.amazon.com/dp/B07CVBJ3SL) ($23.99). |
| 4 | Bambu Lab A1 mini 3D printer | 1 | Bambu Lab A1 mini + LED Lamp Kit | $219.00 | $219.00 | [Amazon](https://www.amazon.com/dp/B0GQMJ8QQT) | 180×180×180 mm build. AMS lite combo: [B0CRYZWJLG](https://www.amazon.com/dp/B0CRYZWJLG) ($349). Direct: [bambulab.com](https://bambulab.com). |
| 5 | PLA filament 1 kg | 1 | Bambu Lab PLA 1.75 mm 1 kg (Jade White) | $28.00 | $28.00 | [Amazon](https://www.amazon.com/dp/B0CGQYRSNT) | RFID-enabled for A1 mini auto-settings. Black: [B0CGR29R63](https://www.amazon.com/dp/B0CGR29R63) ($28.60). |
| 6 | PETG filament 1 kg | 1 | Bambu Lab PETG Translucent 1.75 mm 1 kg | $30.15 | $30.15 | [Amazon](https://www.amazon.com/dp/B0F68FKRWH) | With reusable spool. Refill-only (no spool): [B0FRQ9VX2K](https://www.amazon.com/dp/B0FRQ9VX2K) ($23.99). |
| 7 | TPU filament 500 g | 1 | Geeetech TPU 1.75 mm 500 g Shore 95A | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0DG8BZL6L) | Bambu TPU is 1 kg only ($42.99, [B0GG283YLZ](https://www.amazon.com/dp/B0GG283YLZ)). |
| 8 | M3×6 button head screws | 32 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes M3×6/8/10/12/16/20/25/30 mm. |
| 9 | M3×8 button head + M3 thin washers | 24 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes nuts & washers. |
| 10 | M3×10 button head screws | 16 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes M3×6/8/10/12/16/20/25/30 mm. |
| 11 | M3×8 self-tapping screws | 52 | 100 pc M3×8 mm Self-Tapping SS, Flat Head Hex | $8.28 | $8.28 | [Amazon](https://www.amazon.com/dp/B0F9XYX9BQ) | 100 pcs (52 needed). Includes drive bit. Alt: [520pc assortment](https://www.amazon.com/dp/B0BPM8J5F5) ($7.99). |
| 12 | M3 heat-set inserts (Ø4.6×4–6 mm) | 12 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes brass heat-set inserts. |
| 13 | M5×20 GoPro thumbscrew | 1 | M5 Handle Thumb Screws (pair, GoPro) | $7.22 | $7.22 | [Amazon](https://www.amazon.com/dp/B0BCJRFCLX) | Pair; only 1 needed. Stainless, GoPro Hero 4–13. |
| 14 | M2.5×8 self-tapping screws | 4 | uxcell 50 pc M2.5×8 mm Self-Tapping | $8.07 | $8.07 | [Amazon](https://www.amazon.com/dp/B01KXTTSCI) | 50 pcs (4 needed). 304 SS. |
| 15 | M3 screw + heat-set insert kit | 1 | KADRICK 420 pcs M3 Heat Set Inserts Kit | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0GYRQG7F2) | **Covers items 8, 9, 10, 12.** M3×6–30 mm socket cap screws, brass inserts, nuts, washers, installation tip, hex key. |
| 16 | TPU sheet 2 mm | 2 | BENECREAT 2-pc Silicone Rubber Sheet 2 mm | $15.59 | $15.59 | [Amazon](https://www.amazon.com/dp/B08P7P69WQ) | 2 sheets. Cut to 90×46 mm for foot pads. Silicone rubber ≈ TPU functionally. |
| 17 | Zip ties 2.5 mm | ~10 | Zip Ties 2.5 mm×200 mm 100 pc | $3.99 | $3.99 | [Amazon](https://www.amazon.com/dp/B0GR52PRHF) | 100 pcs (~10 needed). Black nylon, 30 lb. |

---

## Cost Summary

| Category | Estimated Cost |
|----------|---------------|
| Electronics (servos + driver + battery) | $213.90 |
| 3D printer | $219.00 |
| Filament (PLA + PETG + TPU) | $74.14 |
| Hardware (screws, inserts, thumbscrew, sheet, zip ties) | $59.14 |
| **Total Estimated** | **~$566.18** |

### Servo cost optimization

Waveshare direct is the best servo price: $21.99 each for the 30 kg@12V
version, or $16.99 for the 7.4V version (matches the BOM's 7.4V operating
point). 8× at $16.99 = **$135.92** — cheaper than the Amazon 6-pack option.

Using Waveshare 7.4V servos ($16.99 × 8 = $135.92) + Waveshare driver ($18.99)
+ battery ($18.99) = **$173.90 electronics**, bringing the total to **~$526**.

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
- **Idler screw length**: M3×8 + washer assumes 3.35 mm thread depth; confirm
  screws don't bottom against the gear.
- **Driver board**: exact model + hole pattern (`BOARD_HOLES` in `dimensions.py`)
  — confirm component heights and battery clearance.
- **GoPro fit**: measure your MAX's finger thickness — slots are 3.2 mm.
- **Cable service loops**: verify lengths with real 150 mm leads; extension
  cables may be needed shin→ankle.
- **Foot pad material**: TPU sheet vs printed TPU vs adhesive rubber; friction
  should roughly match sim (μ ≈ 1.0).