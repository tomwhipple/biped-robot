# Hardware order checklist

*Status 2026-07-11. Goal: everything needed for the first physical build.
Sim validation state: gaits verified under real STS3215 torque limits at both
battery voltages; sub-step control-latency validated and hardened (DESIGN.md
§sub-step latency) — nothing on this parts list changed.*

## ✅ DECIDED 2026-07-11: 3S (11.1 V)

The gauntlet numbers that drove it (2 m dash median **2.72 s** vs 4.36 s;
GoPro + rough ground **16/16** vs 12/16), and what the decision changed:

- **Battery**: 2× [Tattu 850 mAh 3S 75C XT30](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30)
  (or any pack passing the `bom-by-vendor.md` spec filter — the bay takes the class)
  (67 × 30 × 18.5 mm, 74 g each) — two packs = hot-swap. ✅ ordered/orderable.
- **CAD**: tower reworked for tool-free swap — the pack now side-loads
  through a window in the rear tower wall (no more unscrewing the tower to
  reach it): tilt in over the sill, it seats on far-wall rails between the
  foot-tab gussets; a 20 mm hook-loop belt around the tower (guide ribs) closes
  the window, and a ribbon under the pack is the pull-tab. Tower grew for
  headroom (`TOWER_H` 32→37→43.5; the last +6.5 on 2026-07-15 widened the bay
  from the one flat Zeee pack to the whole 3S 850 XT30 field). Envelope is
  parametric (`BATT` in `cad/dimensions.py`) — taller pack, one edit, reprint.
- **Servos**: unchanged — the 12 V-class ST3215 (6–12.6 V) was already the pick.
- **Sim**: torso inertia rebuilt for the 3S pack (78 g vs the old 110 g 2S
  estimate, correct orientation); dash policies re-evaluated on the updated
  model: `dash_11v1_hard` 16/16 confirmed finishes, median 2.67 s (GoPro on);
  `dash_11v1_hardlat3` 28/32, median 2.72 s (GoPro + 4 ms latency, pad-true contact + backlash DR per review #3/#13).
- 2S remains a fallback: everything still runs at 7.4 V, policies exist for it.

## Bill of materials

| Item | Spec | Qty | Est. cost | Status |
|---|---|---|---|---|
| **Servos** | Waveshare ST3215, **12 V version** (6–12.6 V, 30 kg·cm @ 12 V, magnetic encoder) — *not* the $16.99 "7.4 V" version, which is rated 4–7.4 V and forecloses 3S | 8 + 1–2 spares | $21.99 ea [Waveshare direct](https://www.waveshare.com/st3215-servo.htm) → ~$176 + spares | ✅ **Ready to order** — sim-validated at both voltages; CAD built from its STEP |
| **Driver board** | Waveshare "Servo Driver with ESP32" (65 × 30 mm, 6–12.6 V in — 2S *and* 3S direct per Waveshare docs, WiFi/BLE) | 1 | $24.99 [Amazon](https://www.amazon.com/dp/B0CFY34BX5) / [direct](https://www.waveshare.com/servo-driver-with-esp32.htm) | ✅ Ready — holes Ø2.75 on 58 × 23 — **verified 2026-07-27 by test-fit, all four screws landed in the printed tower** |
| **Battery** | Tattu 850 mAh 11.1 V 75C 3S1P XT30 (60 × 30 × 23, 80 g) — bay fits the whole 3S 850 XT30 class, not just this one; **11.1 V not 11.4 V HV** | 2 | ~$15 ea [search](https://www.amazon.com/s?k=Tattu+850mAh+3S+75C+XT30) | ✅ **Ready — decision made** (3S, swap-window bay in CAD) |
| Battery belt + ribbon | 20 mm hook-loop strap ~250 mm + pull ribbon (battery retention/extraction) | 1 | ~$5 (or scrap velcro) | ✅ Ready |
| Power switch | inline XT30 rocker/slide switch (battery → board) | 1 | ~$8 | ✅ Ready |
| **IMU** | **BNO055 breakout — ordered** ([B0GVK81HXR](https://www.amazon.com/dp/B0GVK81HXR); classic Adafruit layout, solder header, I2C 0x28, shares the ESP32's OLED bus; mounts on the printed `imu_carrier` under the gopro_base) | 1 | ~$30 | ✅ **Ordered 2026-07-16** — closes the sensing gap: the policy consumes torso attitude + angular velocity that nothing else on this list measures |
| Balance charger | 2S–3S LiPo charger (skip if owned) | 1 | ~$30 | check what you own |
| **Fastener kit** | M3 heat-set inserts (~20), M3×6/8/10 machine, M3 self-tap, M2.5×8 (board), washers | 1 kit | ~$25 | ✅ Ready |
| Filament | PLA (prototype), PETG (final structural), TPU 95A (foot pads) | 1 ea | ~$55 | ✅ Ready |
| Wiring | XT30 pigtail + inline switch, heat-shrink; servo leads (150 mm) ship with servos — 2× ≥200 mm extensions **required** for the hip-roll→hip-pitch hops (worst-pose routed 170 mm, measured in `cad/dress.py`); IMU wiring = 4× F-F dupont jumpers (BNO055 solder header) | — | ~$19 | ✅ Ready ([itemized](bom-sourced.md), items 19–21) |
| GoPro MAX + M5 thumbscrew | camera + its own mounting screw | — | owned | ✅ |
| 3D printer | Bambu Lab A1 mini (see docs/cad-and-printer-recommendations.md) | 1 | ~$199–219 | if not already ordered |

**Ballpark: ~$330–380** for the robot (excl. printer and charger), using the
sourced prices below.

## Reconciliation with docs/bom-sourced.md (2026-07-11)

The sourcing pass ([docs/bom-sourced.md](bom-sourced.md)) found real links and
prices; three of its picks conflicted with the sim-validated design and are
now corrected in that file:

1. **Servo voltage class (order-critical).** The cheaper $16.99 "7.4 V"
   ST3215 is rated **4–7.4 V only** — buying it would permanently lock out
   the 3S battery the dash gauntlet recommends. The $21.99 12 V version
   (rated 6–12.6 V) runs at either voltage; the $40 total delta keeps the
   decision open.
2. **Wrong driver board.** The sourced "Bus Servo Driver HAT (A)" ($18.99) is
   a Raspberry-Pi HAT: 65 × 57 mm (doesn't fit the tower, designed for the
   65 × 30 board) and **9–25 V input, so it can't run from 2S at all**. The
   correct board is the **"Servo Driver with ESP32"** ($24.99): 6–12.6 V
   covers 2S and 3S direct, and its published hole grid (Ø2.75 on 58 × 23)
   matches the tower within 1 mm — `BOARD_HOLES` updated 58 × 24 → 58 × 23.
3. **Battery connector.** The sourced 2S pack is JST-terminated (~3 A rating);
   8 servos pull multiples of that in transients. Any pack should be
   XT30/XT60. The line stays gated on the 2S/3S call regardless.
4. **Gaps and overlaps.** Sourced list omitted the balance charger, servo
   spares, power switch, and hip→board servo extensions; it double-bought
   foot-pad material (TPU filament *and* a silicone sheet — the sheet can be
   dropped). Its KADRICK fastener-kit consolidation ($15.99 for all M3
   screws/inserts/washers) is a good find and replaces my ~$25 line item —
   note the kit is socket-cap rather than button-head, so check head
   clearance at the idler screws.

Apples-to-apples, the sourced total confirms the estimate: ~$330–380 for the
robot + ~$74 filament + $219 printer ≈ **$620–670 all-in from zero**.

## Verify on arrival, before committing plastic

1. **Servo case holes**: tapped M3 or needs M4? (vendor STEP shows Ø3.5 —
   community self-taps M3; `cad/dimensions.py` parameterizes this).
2. **Idler screw depth**: M3×8 + washer assumed; confirm no bottoming.
3. ~~**Driver board holes** vs the CAD's 58 × 24 mm guess → then print tower.~~ **Closed 2026-07-27**: the board test-fits the printed tower, all four screws land. `BOARD_HOLES = (58.0, 23.0)` confirmed.
4. **GoPro finger fit**: print `gopro_base` alone first (~20 min) and test the
   3.2 mm slots against the MAX's fingers (`GP_SLOT = 3.5` fallback).
5. **Board component height** (<4 mm on down-facing side) — this sets the
   3.5 mm board-to-pack clearance, so a board with taller underside parts eats
   into it. Confirm your pack ≤ the 68 × 31 × 26.5 mm bay envelope and test the
   window swap (tilt in over the 2.5 mm sill) with the real pack before final
   assembly.

## Wiring

Pin-level circuit diagram (connectors, nets, wire colors), system block
diagram, servo ID map, current budget, and bring-up checklist:
[docs/wiring.md](wiring.md). Short version: battery → switch →
driver board; one 3-pin bus daisy-chained per leg (IDs 1–4 left, 5–8 right,
matching the sim's action order); GoPro unpowered by the robot.

## Latency: closed ✅ (was the last open architecture assumption)

The serial-bus control loop (ESP32 driver → 8-servo daisy chain at 50 Hz) is
validated: sub-step latency modeling found the real cliff (the earlier 0/16
collapse was a doubled whole-step-delay artifact), and latency-DR fine-tunes
(`dash_11v1_hardlat3` 28/32 @ 2.72 s with GoPro + 4 ms on pad-true contact + backlash; `dash_7v4_hardlat`
32/32 @ 4.50 s) hold to ~10–14 ms — 2–3× the expected 2–5 ms bus latency.
Firmware plan of record: run the 50 Hz loop **on the ESP32** (already in the
BOM) rather than round-tripping through USB/WiFi. Nothing to buy either way.
Full sweep tables: DESIGN.md §sub-step latency.
