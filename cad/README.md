# Printable CAD — Bimo-like biped

Parametric CAD in Python ([build123d](https://build123d.readthedocs.io/)), realizing the
kinematics of `sim/bimo_biped.xml` with 8× Feetech STS3215 bus servos.

```
cad/
├── dimensions.py      # ALL dimensions (single source of truth) + servo datasheet notes
├── parts.py           # part builders; run it to export STLs + mass/bed report
├── check_assembly.py  # boolean interference checks over the full joint ranges
├── stl/               # exported STLs (one per unique part)
└── bimo_like_biped.scad  # (older massing concept, superseded by parts.py)
```

```bash
../.venv/bin/python parts.py            # build + export + verify (bed fit, masses)
../.venv/bin/python check_assembly.py   # must print ALL CLEAR
```

Servo dimensions were taken from the **official Waveshare ST3215 STEP model and
2D drawing** (measured programmatically, not eyeballed) — sources in the
`dimensions.py` docstring. Every joint is supported on **both** sides: the servo's
metal horn (Ø19.2, 4× M3 on a Ø14 bolt circle) on one side and its built-in
free-spinning **rear idler disc** (same Ø19.2 / 4× M3 pattern, recessed in a Ø25
opening) on the other — no extra bearings needed.

## Part list (12 prints, 6 unique)

| part | qty | bbox (mm) | ~mass | role |
|---|---|---|---|---|
| `pelvis` | 1 | 46 × 104 × 48 | 47 g | deck + two hanging bays clamping the hip-roll servos |
| `yoke_roll` | 2 | 48 × 34 × 32 | 12 g | clevis on roll-servo horn/idler, flange down |
| `yoke_pitch` | 2 | 32 × 45 × 42 | 12 g | clevis on thigh-servo horn/idler, bolts under `yoke_roll` rotated 90° (hip universal joint) |
| `leg_link` | 4 | 28 × 45 × 99 | 18 g | thigh **and** shin (same part): grips a servo case, forks 90 mm down to the next servo's horn/idler |
| `foot` | 2 | 96 × 52 × 30 | 24 g | sole + ankle-servo pocket + rear retention walls + TPU pad recess |
| `tower` | 1 | 42 × 96 × 32 | 32 g | electronics: driver board on top, battery strapped inside |

Printed plastic ≈ 248 g. Total robot ≈ **0.88 kg** (8 servos 440 g, 2S LiPo ~110 g,
board ~20 g, fasteners ~45 g, TPU 16 g). Heights: ankle 16.9, knee 106.9,
hip-pitch 196.9, hip-roll 246.9, torso top 319 mm.

All legs use identical parts: every pitch-joint horn faces **+Y (robot left)** —
legs are translations, not mirrors.

## BOM

| item | qty | notes |
|---|---|---|
| Feetech STS3215 (7.4 V version) | 8 | 30 kg·cm@12 V class; includes metal horn + idler disc + M3×6 screws |
| Waveshare Bus Servo Adapter / Servo Driver (ESP32) | 1 | hole pattern in `dimensions.py::BOARD_HOLES` is a 58×24 guess for the ESP32 driver — **verify against your board before printing the tower** |
| 2S LiPo 7.4 V, 450–1000 mAh | 1 | ≤ 75 × 35 × 16 mm envelope, velcro strap through tower slots |
| M3×6 button head | 32 | horn pads (4 per joint × 8) — servo kits include some |
| M3×8 button head + M3 thin washer | 24 | idler pads at hip-pitch, knee, ankle (4 × 6 joints); washer stops the tip short of the gears |
| M3×10 button head | 16 | yoke_roll idler arms (4 × 2, through the long boss) + hip flange bolts (4 × 2) |
| M3×8 self-tapping (or machine after M3-tapping the case) | 48 | case grips: 6 per leg_link (24), 8 per pelvis bay (16), 4 per foot (8) |
| M3 heat-set insert (Ø4.6 × 4–6) | 12 | 4 per yoke_pitch flange (8) + 4 in the pelvis deck for the tower |
| M2.5×6 self-tapping | 4 | driver board to tower top |
| TPU sheet 2 mm (or printed TPU pad 90 × 46) | 2 | glued into the foot sole recess, sits 0.5 mm proud |
| zip ties 2.5 mm | ~10 | cable dressing through leg_link web holes |

Servo case mounting holes measure Ø3.5 in the vendor STEP; the community
practice (glass-filled nylon case) is M3 self-tappers or tapping M3.
**Verify on one real servo first** — if the holes are truly Ø3.5, use M4
self-tappers in the pelvis/foot walls (clearance holes are parametric).

## Print settings

PLA or PETG, 0.4 mm nozzle, 0.2 mm layers, 3 perimeters (all walls ≥ 2.4 mm are
perimeter-only), 30–40 % infill, no supports needed:

| part | orientation | note |
|---|---|---|
| pelvis | upside-down (deck top on bed) | bay walls print vertically; bore is a downward-open slot |
| yoke_roll / yoke_pitch | flange face on bed | arms vertical → layer lines ⟂ arm bending is avoided; idler boss prints as a short horizontal stub (1–4 mm) — slight underside droop is cosmetic, the seat face prints clean |
| leg_link | on its back (web on bed) | strongest orientation for fore-aft bending; bosses as above |
| foot | sole down | |
| tower | upside-down (top plate on bed) | feet tabs have 45° gussets |

## Assembly order

1. **Servos**: set IDs 1–8 and center all servos (position 2048) *before* assembly.
   Bolt the metal horn on each servo at center with its spline screw.
2. **Feet**: drop the ankle servo into the pocket (output end forward, cable aft),
   4× M3 through the rear walls into the case. Glue TPU pads.
3. **Leg links** (×4): slide onto a servo case from below (horn-side plate has a
   circular relief that clears the output boss), 6× M3 into the case holes.
4. **Knees/ankles**: offer the link fork to the next servo: 4× M3×6 into the horn
   (+Y side), 4× M3×8+washer into the idler disc (−Y side). Do horn side first,
   check the mechanical zero, then the idler side.
5. **Hips**: press heat-set inserts into the yoke_pitch flanges; bolt yoke_pitch
   to the thigh-servo horn/idler; bolt yoke_roll on top (rotated 90°, 4× M3×10).
6. **Pelvis**: slide the two roll servos up into the bays (output end down, horn
   forward, cables up through the deck cutouts), 8× M3 each through the walls.
   Attach each hip: yoke_roll horn arm to the roll-servo horn (4× M3×6, in front),
   idler arm boss through the rear wall slot into the idler disc (4× M3×10).
7. **Torso**: press 4 inserts into the deck bosses, strap the battery inside the
   tower, screw the driver board on top, bolt the tower down.
8. **Wiring**: daisy chain — board → both roll servos (connectors are inside the
   torso) → down the back of each leg (zip ties through the web holes, leave a
   slack loop across each joint sized at full flexion) → thigh → shin → ankle.
9. Verify each joint's direction/sign against the sim (`DESIGN.md` TODO) before
   trusting any gait; run range-of-motion slowly at low torque limit first.

## Open questions

- **Servo case thread**: M3 self-tap vs tap vs M4 (see BOM note). Measure a real case.
- **Idler screw length**: M3×8 + washer assumes the disc's 3.35 mm thread depth;
  confirm screws don't bottom against the gear behind the disc.
- **Driver board**: exact model + hole pattern (`BOARD_HOLES`), and whether it
  lives on top (current) or inside the tower for a lower CG.
- **Cable service loops** across knee (95°) — verify lengths with real 150 mm leads;
  extension cables may be needed shin→ankle depends on port placement.
- **Foot pad material**: TPU sheet vs printed TPU vs adhesive rubber; friction
  should roughly match sim (μ ≈ 1.0).
- Ankle axis is 16.9 mm (not 14) above ground → use `sim/bimo_biped_v2.xml`.
