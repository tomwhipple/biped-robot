# Bipedal Robot — Bill of Materials (grouped by vendor)

*Compiled from [`bom-sourced.md`](bom-sourced.md). Prices in USD, captured
2026-07-11. Prices and availability subject to
change.*

Robot: ~34 cm, ~0.9 kg 3D-printed biped, 8× Feetech STS3215 serial-bus servos.

> **The 3D printer and filament are now a separate one-time order** —
> see [`printer-order.md`](printer-order.md). This file is robot parts only.

---

## Waveshare

| Component              | Qty             | Product                                                                                                            | Unit   | Extended    | Link                                                                   |
| ---------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------ | ------ | ----------- | ---------------------------------------------------------------------- |
| Feetech STS3215 servo  | 8 (+1–2 spares) | ST3215 Serial Bus Servo — **30 kg·cm @ 12 V version** (rated 6–12.6 V; buy this, *not* the $16.99 4–7.4 V version) | $21.99 | $175.92     | https://www.waveshare.com/st3215-servo.htm                             |
| ESP32 Bus Servo Driver | 1               | Servo Driver with ESP32 (65 × 30 mm, 6–12.6 V — 2S & 3S direct)                                                    | $24.99 | $24.99      | [waveshare.com](https://www.waveshare.com/servo-driver-with-esp32.htm) |
| **Waveshare subtotal** |                 |                                                                                                                    |        | **$200.91** |                                                                        |

Best direct prices on the two most expensive electronic parts. Shipping may be
slower than Amazon Prime — check lead time. Amazon alternates: servo
[B0FGNTDV3Y](https://www.amazon.com/dp/B0FGNTDV3Y?tag=tommwhipple-20) ($29.99, 12 V), driver
[B0CFY34BX5](https://www.amazon.com/dp/B0CFY34BX5?tag=tommwhipple-20) ($24.99).

---

## Amazon

| Component                      | Qty          | Product                                                                                                                 | Unit   | Extended     | Link                                                                                                                                                          |
| ------------------------------ | ------------ | ----------------------------------------------------------------------------------------------------------------------- | ------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 3S LiPo battery                | 2            | Tattu 850 mAh 11.1 V 75C 3S1P XT30 (60 × 30 × 23 mm, 80 g) — or any pack meeting the spec filter below                  | ~$15   | ~$30.00      | [search](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30&tag=tommwhipple-20) · ASIN [B07218SB7L](https://www.amazon.com/dp/B07218SB7L?tag=tommwhipple-20) |
| M3 screw + heat-set insert kit | 1            | KADRICK 420 pc M3 kit — covers M3×6–30 screws, brass inserts, nuts, washers                                             | $15.99 | $15.99       | [B0GYRQG7F2](https://www.amazon.com/dp/B0GYRQG7F2?tag=tommwhipple-20)                                                                                         |
| M3×8 self-tapping screws       | 52 (100 pc)  | M3×8 Self-Tapping SS, flat head hex (incl. drive bit)                                                                   | $8.28  | $8.28        | [B0F9XYX9BQ](https://www.amazon.com/dp/B0F9XYX9BQ?tag=tommwhipple-20)                                                                                         |
| M2.5×8 self-tapping screws     | 4 (50 pc)    | uxcell M2.5×8 self-tapping, 304 SS                                                                                      | $8.07  | $8.07        | [B01KXTTSCI](https://www.amazon.com/dp/B01KXTTSCI?tag=tommwhipple-20)                                                                                         |
| M5×20 GoPro thumbscrew         | 1 (pair)     | M5 handle thumb screws, stainless, GoPro Hero 4–13                                                                      | $7.22  | $7.22        | [B0BCJRFCLX](https://www.amazon.com/dp/B0BCJRFCLX?tag=tommwhipple-20)                                                                                         |
| Sole pad sheet 1/16"           | 1 (2-pack)   | Self-adhesive silicone rubber sheet, 2× 6"×6" — cut two 106 × 46 mm pads (one sheet does both feet; 2nd sheet = spares) | —      | —            | [B0FJ8TBMQK](https://www.amazon.com/dp/B0FJ8TBMQK?tag=tommwhipple-20)                                                                                         |
| Zip ties 2.5 mm                | ~10 (100 pc) | 2.5 × 200 mm black nylon, 30 lb                                                                                         | $3.99  | $3.99        | [B0GR52PRHF](https://www.amazon.com/dp/B0GR52PRHF?tag=tommwhipple-20)                                                                                         |
| 9-DOF IMU                      | 1            | Teyleten Robot GY-BNO085 9DOF AHRS (BNO085 chip, CEVA SH-2 fusion — actual part ordered 2026-07-16)                     | $20.99 | $20.99       | [B0CL26J81F](https://www.amazon.com/dp/B0CL26J81F?tag=tommwhipple-20)                                                                                         |
| **Amazon subtotal**            |              |                                                                                                                         |        | **~$110.13** |                                                                                                                                                               |

---

## ~~Adafruit~~ → moved to Amazon

The IMU was originally planned as an Adafruit BNO085 breakout (#4754, $24.95).
Actual order: Teyleten Robot GY-BNO085 via Amazon [B0CL26J81F](https://www.amazon.com/dp/B0CL26J81F?tag=tommwhipple-20)
($20.99) — same BNO085 IC, ~$4 cheaper. Datasheet in
[`docs/datasheets/BNO08X-datasheet.pdf`](datasheets/BNO08X-datasheet.pdf).

Cheap spare/alt: MPU-6050 "GY-521" (~$5, needs a filter in firmware).

---

## Vendor totals

| Vendor          | Extended  |
| --------------- | --------- |
| Waveshare       | $200.91   |
| Amazon          | ~$110.13  |
| **Robot total** | **~$311** |

Servo spares (1–2 × $21.99) are the main swing item. The 3D printer + filament +
start-up consumables (~$530, FlashForge 5M Pro) is a separate one-time order —
see [`printer-order.md`](printer-order.md).
All-in from zero (robot + printer order): **~$845**.

---

## Notes & caveats

- **Servo voltage class is order-critical.** Buy the 12 V-class ST3215
  (6–12.6 V). The cheaper $16.99 "7.4 V" version is rated 4–7.4 V and forecloses
  the sim-recommended 3S battery.

- **Driver board.** Use the "Servo Driver with ESP32" (65 × 30 mm), *not* the
  Bus Servo Driver HAT (A) — that Pi HAT is the wrong size and 9–25 V input.
  Waveshare also sells PCA9685-based boards that drive *PWM* servos — those
  cannot talk to ST/SC bus servos at all. On Amazon the same board is listed as
  [B0CFY34BX5](https://www.amazon.com/dp/B0CFY34BX5?tag=tommwhipple-20),
  [B0F5W67S56](https://www.amazon.com/dp/B0F5W67S56?tag=tommwhipple-20), and
  [B09SZ41RJW](https://www.amazon.com/dp/B09SZ41RJW?tag=tommwhipple-20) — buy whichever is cheapest
  and in stock; prefer a title that states "6~12V" and "SC, ST Series".

- **Battery: buy to the spec filter, not to an ASIN.** The bay was originally cut
  for the Zeee 3S 850 (67 × 30 × 18.5, 74 g), which went unavailable — and every
  other 3S 850 is stubbier but *taller*, so nothing dropped in. The bay is now
  sized (`BATT` in `cad/dimensions.py`) to swallow the whole class. **Any pack
  meeting all four of these fits, no CAD change:**
  
  |           | requirement                      | why                                                       |
  | --------- | -------------------------------- | --------------------------------------------------------- |
  | chemistry | **3S, 11.1 V** — *not* 11.4 V HV | LiHV charges to 13.05 V, over the ST3215's 12.6 V ceiling |
  | connector | **XT30**                         | not JST / EC5 / XT60; the pigtail assumes it              |
  | height    | **≤ 26.5 mm**                    | binding axis — 3.5 mm board clearance above               |
  | footprint | **≤ 68 (L) × 31 mm (W)**         | bay envelope; L runs laterally                            |
  
  BOM pick is the **Tattu 850 mAh 75C** (60 × 30 × 23 mm, 80 g) — specs
  cross-checked against three independent stockists, and its 80 g is the mass
  the sim models (`BATT_PACK_MASS`). Ovonic 80C (60 × 30 × 24, 76 g) and CNHL
  70C (62 × 30 × 25, ~80 g) also fit. **Amazon ASINs for these packs churn**
  (the Zeee's went dead mid-project) — prefer the search link in the table and
  apply the filter above. Short packs (59–62 mm) leave up to 8 mm of *lateral*
  slop; shim it (see `DESIGN.md`, "Battery bay widened") — the sim assumes the
  pack is centered and cannot represent it moving.

- **Fastener consolidation.** The KADRICK kit covers all M3 button-head screws,
  washers, and heat-set inserts (originally 4 separate line items). It is
  socket-cap, not button-head — check head clearance at the idler screws.

- **Foot pads: cut, not printed.** Decision 2026-07-15 — all structural parts
  print in PETG, and the sole pads are cut from the self-adhesive 1/16"
  silicone sheet above (106 × 46 mm each) and stuck to the flat sole. No TPU
  filament needed for the pads.

- **Not yet sourced / not vendor-grouped here:** 2S–3S balance charger (~$30,
  skip if owned), XT30 pigtail + heat-shrink + inline power switch (~$10),
  2 longer servo extension leads (hip→board), battery belt + pull ribbon (~$5
  or scrap velcro). See [hardware-order.md](hardware-order.md).
