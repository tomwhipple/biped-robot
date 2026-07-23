# Bipedal Robot — Sourced Bill of Materials

*Compiled 2026-07-11. Prices in USD, delivering to Lyons, CO 80540.*

Robot: ~34 cm, ~0.9 kg 3D-printed biped, 8× Feetech STS3215 serial-bus servos.
BOM source: [`cad/README.md`](../cad/README.md). Print settings and assembly
order also there.

---

## Consolidated BOM

| # | Component | Qty | Product | Unit Price | Extended | Source | Notes |
|---|-----------|-----|---------|-----------|----------|--------|-------|
| 1 | Feetech STS3215 servo | 8 (+1–2 spares) | ST3215 Series Serial Bus Servo (**30 kg·cm@12 V version**) | $21.99 | $175.92 | [Waveshare](https://www.waveshare.com/st3215-servo.htm) | ⚠️ **Buy the 12 V version only.** The $16.99 "7.4 V" class is rated **4–7.4 V** and cannot run on 3S — the sim-recommended battery ([hardware-order.md](hardware-order.md)). The 12 V version is rated 6–12.6 V and works at either voltage. Amazon alt: [B0FGNTDV3Y](https://www.amazon.com/dp/B0FGNTDV3Y) ($29.99, 12V). |
| 2 | Waveshare ESP32 Bus Servo Driver | 1 | **Servo Driver with ESP32** (65 × 30 mm) | $24.99 | $24.99 | [Waveshare](https://www.waveshare.com/servo-driver-with-esp32.htm) / [Amazon B0CFY34BX5](https://www.amazon.com/dp/B0CFY34BX5) | ⚠️ **Corrected — the previously listed [Bus Servo Driver HAT (A)](https://www.waveshare.com/bus-servo-driver-hat-a.htm) ($18.99) is the wrong board**: it's a Raspberry-Pi HAT (65 × 57 mm — doesn't fit the tower, which is built for the 65 × 30 board) and takes 9–25 V (can't run from 2S at all). The Servo Driver with ESP32 is 6–12.6 V (2S *and* 3S direct per Waveshare docs), holes Ø2.75 on 58 × 23 — matches `cad/dimensions.py`. |
| 3 | 3S LiPo battery (**decided 2026-07-11**; re-sourced 2026-07-15) | 2 | Tattu 850 mAh 11.1 V 75C 3S1P XT30 | ~$15 | ~$30 | [search](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30) · [B07218SB7L](https://www.amazon.com/dp/B07218SB7L) | ✅ 60 × 30 × 23 mm, 80 g (cross-checked at 3 stockists; 80 g = the sim's `BATT_PACK_MASS`). The original Zeee (67 × 30 × 18.5) went dead on Amazon mid-project, so the bay envelope (`BATT` in `cad/dimensions.py`) is now a **superset of the whole 3S 850 XT30 class** — buy to the [spec filter](bom-by-vendor.md#notes--caveats), not an ASIN. Height is the binding axis. **11.1 V only — not 11.4 V HV** (13.05 V charged > the servo's 12.6 V ceiling). |
| 4 | Bambu Lab A1 mini 3D printer | 1 | Bambu Lab A1 mini + LED Lamp Kit | $219.00 | $219.00 | [Amazon](https://www.amazon.com/dp/B0GQMJ8QQT) | 180×180×180 mm build. AMS lite combo: [B0CRYZWJLG](https://www.amazon.com/dp/B0CRYZWJLG) ($349). Direct: [bambulab.com](https://bambulab.com). |
| 5 | PLA filament 1 kg | 1 | Bambu Lab PLA 1.75 mm 1 kg (Jade White) | $28.00 | $28.00 | [Amazon](https://www.amazon.com/dp/B0CGQYRSNT) | RFID-enabled for A1 mini auto-settings. Black: [B0CGR29R63](https://www.amazon.com/dp/B0CGR29R63) ($28.60). |
| 6 | PETG filament 1 kg | 1 | Bambu Lab PETG Translucent 1.75 mm 1 kg | $30.15 | $30.15 | [Amazon](https://www.amazon.com/dp/B0F68FKRWH) | With reusable spool. Refill-only (no spool): [B0FRQ9VX2K](https://www.amazon.com/dp/B0FRQ9VX2K) ($23.99). |
| 7 | TPU filament 500 g | 1 | Geeetech TPU 1.75 mm 500 g Shore 95A | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0DG8BZL6L) | Bambu TPU is 1 kg only ($42.99, [B0GG283YLZ](https://www.amazon.com/dp/B0GG283YLZ)). |
| 8 | M3×6 button head screws | 0 | *not needed — horn screws ship with the servos* | — | — | — | v3yaw audit 2026-07-23: the 38 M3×6 used are the ST3215s' own bundled horn screws (10×4 on hand, 2 spare). Kit still has M3×6 if any strip. |
| 9 | M3×8 button head + M3 thin washers | 24 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes nuts & washers. |
| 10 | M3×10 button head screws | 20 | *Covered by KADRICK kit (item 15)* | — | — | — | Was 16 — never counted the 4 tower-feet bolts (assembly §9c, found in the 2026-07-23 fastener itemization). Kit includes M3×6/8/10/12/16/20/25/30 mm. |
| 11 | M3×8 self-tapping screws | 56 | 100 pc M3×8 mm Self-Tapping SS, Flat Head Hex | $8.28 | $8.28 | [Amazon](https://www.amazon.com/dp/B0F9XYX9BQ) | 100 pcs (56 needed: v3yaw adds 8 yaw-stator + the old itemization under-summed; see assembly.md §0). Includes drive bit. Alt: [520pc assortment](https://www.amazon.com/dp/B0BPM8J5F5) ($7.99). |
| 12 | M3 heat-set inserts (Ø4.6×4–6 mm) | 12 | *Covered by KADRICK kit (item 15)* | — | — | — | Kit includes brass heat-set inserts. |
| 13 | M5×20 GoPro thumbscrew | 1 | M5 Handle Thumb Screws (pair, GoPro) | $7.22 | $7.22 | [Amazon](https://www.amazon.com/dp/B0BCJRFCLX) | Pair; only 1 needed. Stainless, GoPro Hero 4–13. |
| 14 | M2.5×8 self-tapping screws | 8 | uxcell 50 pc M2.5×8 mm Self-Tapping | $8.07 | $8.07 | [Amazon](https://www.amazon.com/dp/B01KXTTSCI) | 50 pcs (8 needed: 4 driver board + 4 BNO055→imu_carrier). 304 SS. |
| 15 | M3 screw + heat-set insert kit | 1 | KADRICK 420 pcs M3 Heat Set Inserts Kit | $15.99 | $15.99 | [Amazon](https://www.amazon.com/dp/B0GYRQG7F2) | **Covers items 8, 9, 10, 12.** M3×6–30 mm socket cap screws, brass inserts, nuts, washers, installation tip, hex key. |
| 16 | TPU sheet 2 mm | 2 | BENECREAT 2-pc Silicone Rubber Sheet 2 mm | $15.59 | $15.59 | [Amazon](https://www.amazon.com/dp/B08P7P69WQ) | 2 sheets. Cut to 90×46 mm for foot pads. Silicone rubber ≈ TPU functionally. |
| 17 | Zip ties 2.5 mm | ~10 | Zip Ties 2.5 mm×200 mm 100 pc | $3.99 | $3.99 | [Amazon](https://www.amazon.com/dp/B0GR52PRHF) | 100 pcs (~10 needed). Black nylon, 30 lb. |
| 18 | 9-DOF IMU | 1 | **BNO055 breakout (ordered 2026-07-16)** — classic Adafruit layout, solder header | ~$30 | ~$30 | [B0GVK81HXR](https://www.amazon.com/dp/B0GVK81HXR) | ✅ Ordered. Torso attitude feedback (up-vector + gyro, sensor fusion on-chip, quaternion @100 Hz); 4-wire I2C at 0x28. Board 26.67×20.32, holes 21.59×15.24 Ø2.5 (from Adafruit's Eagle brd) — the printed `imu_carrier` is cut to this pattern. Supersedes the BNO085/4754 pick (either works; carrier would need the STEMMA-variant hole swap noted in `cad/dimensions.py`). |
| 19 | Servo bus extension leads, 3-pin ≥200 mm | 2 (+2 spares) | Feetech/Waveshare 3-pin bus servo cable, 200–300 mm | ~$1–2 | ~$8 | [search](https://www.amazon.com/s?k=feetech+3+pin+servo+cable+200mm) · [Waveshare](https://www.waveshare.com/product/robotics/accessories.htm) | For the **hip-roll → hip-pitch** hop (one per leg): worst-pose routed path is **170 mm** (measured in `cad/dress.py` across the full ROM incl. slack loop), so the 150 mm leads that ship with the servos are too short *there and only there*. ≥200 mm gives ~18 % flex margin; if only 300 mm is stocked, zip-tie the excess at the thigh raceway anchors. Every other hop fits a stock 150 mm lead (see [wiring.md](wiring.md)). |
| 20 | XT30 pigtail + inline switch + heat-shrink | 1 set | XT30 pigtail (mates the pack lead) → inline switch → bare ends into the board's screw terminal | ~$10 | ~$10 | [search](https://www.amazon.com/s?k=xt30+pigtail+connector+inline+switch) | Battery → power switch → driver-board screw terminal. Routed run is only ~30 mm, so any short pigtail works; 20 AWG or thicker (bus transient budget ~10 A). |
| 21 | IMU jumpers, female-female | 4 | any 10–20 cm F-F dupont jumpers | ~$0 | ~$0 | [search](https://www.amazon.com/s?k=female+female+dupont+jumper+wires) | BNO055 header → ESP32 GPIO 21/22 + 3V3/GND (the ordered classic BNO055 has a solder header, not JST — the previously listed Adafruit 4209 JST cable no longer applies). Usually already in the parts bin. |

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
clearance at the idler screws before relying on it there. The gopro_base +
imu_carrier stack needs **4× M3×12 self-tapping** — the ordered 100-pc M3×8
box (item 11) doesn't cover it, its 520-pc assortment alt does.

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
- **Idler screw length**: M3×8 + washer assumes 3.35 mm thread depth; confirm
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