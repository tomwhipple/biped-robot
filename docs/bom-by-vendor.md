# Bipedal Robot — Bill of Materials (grouped by vendor)

*Compiled from [`bom-sourced.md`](bom-sourced.md). Prices in USD, captured
2026-07-11, delivering to Lyons, CO 80540. Prices and availability subject to
change.*

Robot: ~34 cm, ~0.9 kg 3D-printed biped, 8× Feetech STS3215 serial-bus servos.

> **The 3D printer and filament are now a separate one-time order** —
> see [`printer-order.md`](printer-order.md). This file is robot parts only.

---

## Waveshare

| Component | Qty | Product | Unit | Extended | Link |
|---|---|---|---|---|---|
| Feetech STS3215 servo | 8 (+1–2 spares) | ST3215 Serial Bus Servo — **30 kg·cm @ 12 V version** (rated 6–12.6 V; buy this, *not* the $16.99 4–7.4 V version) | $21.99 | $175.92 | [waveshare.com](https://www.waveshare.com/st3215-servo.htm) |
| ESP32 Bus Servo Driver | 1 | Servo Driver with ESP32 (65 × 30 mm, 6–12.6 V — 2S & 3S direct) | $24.99 | $24.99 | [waveshare.com](https://www.waveshare.com/servo-driver-with-esp32.htm) |
| **Waveshare subtotal** | | | | **$200.91** | |

Best direct prices on the two most expensive electronic parts. Shipping may be
slower than Amazon Prime — check lead time. Amazon alternates: servo
[B0FGNTDV3Y](https://www.amazon.com/dp/B0FGNTDV3Y) ($29.99, 12 V), driver
[B0CFY34BX5](https://www.amazon.com/dp/B0CFY34BX5) ($24.99).

---

## Amazon

| Component | Qty | Product | Unit | Extended | Link |
|---|---|---|---|---|---|
| 3S LiPo battery | 1 (2-pack) | Zeee Premium 3S 850 mAh 11.1 V 100C XT30, 2-pack (67 × 30 × 18.5 mm, 74 g) | ~$30 | ~$30.00 | [B08H5GD35D](https://www.amazon.com/dp/B08H5GD35D) |
| M3 screw + heat-set insert kit | 1 | KADRICK 420 pc M3 kit — covers M3×6–30 screws, brass inserts, nuts, washers | $15.99 | $15.99 | [B0GYRQG7F2](https://www.amazon.com/dp/B0GYRQG7F2) |
| M3×8 self-tapping screws | 52 (100 pc) | M3×8 Self-Tapping SS, flat head hex (incl. drive bit) | $8.28 | $8.28 | [B0F9XYX9BQ](https://www.amazon.com/dp/B0F9XYX9BQ) |
| M2.5×8 self-tapping screws | 4 (50 pc) | uxcell M2.5×8 self-tapping, 304 SS | $8.07 | $8.07 | [B01KXTTSCI](https://www.amazon.com/dp/B01KXTTSCI) |
| M5×20 GoPro thumbscrew | 1 (pair) | M5 handle thumb screws, stainless, GoPro Hero 4–13 | $7.22 | $7.22 | [B0BCJRFCLX](https://www.amazon.com/dp/B0BCJRFCLX) |
| TPU / silicone sheet 2 mm | 2 | BENECREAT silicone rubber sheet — foot pads (optional; see note) | $15.59 | $15.59 | [B08P7P69WQ](https://www.amazon.com/dp/B08P7P69WQ) |
| Zip ties 2.5 mm | ~10 (100 pc) | 2.5 × 200 mm black nylon, 30 lb | $3.99 | $3.99 | [B0GR52PRHF](https://www.amazon.com/dp/B0GR52PRHF) |
| **Amazon subtotal** | | | | **~$89.14** | |

---

## Adafruit

| Component | Qty | Product | Unit | Extended | Link |
|---|---|---|---|---|---|
| 9-DOF IMU | 1 | Adafruit BNO085 breakout (STEMMA QT, on-chip fusion → up-vector + gyro) | $24.95 | $24.95 | [Adafruit 4754](https://www.adafruit.com/product/4754) |
| **Adafruit subtotal** | | | | **$24.95** | |

Cheap spare/alt: MPU-6050 "GY-521" (~$5, needs a filter in firmware).

---

## Vendor totals

| Vendor | Extended |
|---|---|
| Waveshare | $200.91 |
| Amazon | ~$89.14 |
| Adafruit | $24.95 |
| **Robot total** | **~$315** |

Servo spares (1–2 × $21.99) are the main swing item. The 3D printer + filament +
start-up consumables (~$526, FlashForge 5M Pro) is a separate one-time order —
see [`printer-order.md`](printer-order.md).
All-in from zero (robot + printer order): **~$841**.

---

## Notes & caveats

- **Servo voltage class is order-critical.** Buy the 12 V-class ST3215
  (6–12.6 V). The cheaper $16.99 "7.4 V" version is rated 4–7.4 V and forecloses
  the sim-recommended 3S battery.
- **Driver board.** Use the "Servo Driver with ESP32" (65 × 30 mm), *not* the
  Bus Servo Driver HAT (A) — that Pi HAT is the wrong size and 9–25 V input.
- **Fastener consolidation.** The KADRICK kit covers all M3 button-head screws,
  washers, and heat-set inserts (originally 4 separate line items). It is
  socket-cap, not button-head — check head clearance at the idler screws.
- **Redundant foot-pad material.** Foot pads are *printed* TPU (the TPU
  filament in [`printer-order.md`](printer-order.md)). The silicone sheet above
  is a fallback and can be dropped (–$15.59).
- **Not yet sourced / not vendor-grouped here:** 2S–3S balance charger (~$30,
  skip if owned), XT30 pigtail + heat-shrink + inline power switch (~$10),
  2 longer servo extension leads (hip→board), battery belt + pull ribbon (~$5
  or scrap velcro). See [hardware-order.md](hardware-order.md).
