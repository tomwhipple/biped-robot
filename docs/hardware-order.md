# Hardware order checklist

*Status 2026-07-11. Goal: everything needed for the first physical build.
Sim validation state: gaits verified under real STS3215 torque limits at both
battery voltages; sub-step control-latency validation in progress (worst case
changes firmware plans, not this parts list).*

## ⚠️ The one decision that gates the order: 2S vs 3S

Vendor listings split the STS3215 into **7.4 V (19.5 kg·cm)** and **12 V
(30 kg·cm)** classes; the **Waveshare ST3215 is rated 6–12.6 V** and covers
both — and it's the exact unit our CAD was measured from (official STEP file).
**Buy the Waveshare ST3215** and the servo works at either voltage; the battery
choice then sets performance:

| | 2S (7.4 V) — current CAD | 3S (11.1 V) — recommended |
|---|---|---|
| 2 m dash (median) | 4.36 s | **2.72 s** |
| GoPro + rough ground | 12/16 | **16/16** |
| Gait style | grounded (hardware-friendlier) | jog-like flight phase |
| CAD impact | none | battery bay +~10 mm (small edit) |

**Recommendation: 3S**, based on the sim gauntlet — but 2S is viable if you
prefer the safer gait and zero CAD changes. Both policies exist either way.

## Bill of materials

| Item | Spec | Qty | Est. cost | Status |
|---|---|---|---|---|
| **Servos** | Waveshare ST3215 serial bus servo (6–12.6 V, 30 kg·cm @ 12 V, magnetic encoder) | 8 + 1–2 spares | ~$16–20 ea → ~$160–200 | ✅ **Ready to order** — sim-validated at both voltages; CAD built from its STEP |
| **Driver board** | Waveshare "Servo Driver with ESP32" (30 × 65 mm, 6–12 V in — covers 2S *and* 3S, WiFi/BLE) | 1 | ~$15–25 | ✅ Ready — verify mounting-hole pattern on arrival **before printing the tower** (CAD assumes 58 × 24 mm) |
| **Battery** | 3S 11.1 V LiPo 650–1000 mAh, XT30 (or 2S per decision above) | 2 | ~$15–25 ea | ⏳ **Blocked on 2S/3S call**; if 3S, I resize the bay first |
| Balance charger | 2S–3S LiPo charger (skip if owned) | 1 | ~$30 | check what you own |
| **Fastener kit** | M3 heat-set inserts (~20), M3×6/8/10 machine, M3 self-tap, M2.5×8 (board), washers | 1 kit | ~$25 | ✅ Ready |
| Filament | PLA (prototype), PETG (final structural), TPU 95A (foot pads) | 1 ea | ~$55 | ✅ Ready |
| Wiring | XT30 pigtail, heat-shrink; servo leads (150 mm) ship with servos — 2 longer extensions handy for hip→board | — | ~$10 | ✅ Ready |
| GoPro MAX + M5 thumbscrew | camera + its own mounting screw | — | owned | ✅ |
| 3D printer | Bambu Lab A1 mini (see docs/cad-and-printer-recommendations.md) | 1 | ~$199–219 | if not already ordered |

**Ballpark: ~$300–350** for the robot (excl. printer and charger).

## Verify on arrival, before committing plastic

1. **Servo case holes**: tapped M3 or needs M4? (vendor STEP shows Ø3.5 —
   community self-taps M3; `cad/dimensions.py` parameterizes this).
2. **Idler screw depth**: M3×8 + washer assumed; confirm no bottoming.
3. **Driver board holes** vs the CAD's 58 × 24 mm guess → then print tower.
4. **GoPro finger fit**: print `gopro_base` alone first (~20 min) and test the
   3.2 mm slots against the MAX's fingers (`GP_SLOT = 3.5` fallback).
5. **Board component height** (<4 mm on down-facing side) and battery ≤16 mm
   tall (2S bay) — re-check envelope if 3S.

## What the in-flight latency work validates

The serial-bus control loop (laptop → ESP32 driver → 8-servo daisy chain at
50 Hz) is the one architecture assumption not yet closed: policies trained
before sub-step latency modeling failed under a full 20 ms delay. Real bus
latency is ~2–5 ms; the current sim run measures the true cliff and hardens
the policies. Worst case it changes *where the control loop runs* (the ESP32
already in the BOM can run it closer to the metal) — not what to buy.
