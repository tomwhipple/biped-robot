# Printable CAD — Bimo-like biped

Parametric CAD in Python ([build123d](https://build123d.readthedocs.io/)), realizing the
kinematics of `sim/bimo_biped.xml` with 8× Feetech STS3215 bus servos.

```
cad/
├── dimensions.py      # ALL dimensions (single source of truth) + servo datasheet notes
├── parts.py           # part builders; run it to export STLs + mass/bed report
├── check_assembly.py  # boolean interference checks over the full joint ranges
├── export_step.py     # per-part STEP exports (cad/step/)
├── export_assembly.py # assembled robot -> cad/step/assembly.step (+ camera mock)
├── render_assembly.py # PNG render of the assembly via MuJoCo
├── stl/               # exported STLs (one per unique part)
├── step/              # exported STEPs + assembly.step
└── bimo_like_biped.scad  # (older massing concept, superseded by parts.py)
```

```bash
../.venv/bin/python parts.py            # build + export + verify (bed fit, masses, CG)
../.venv/bin/python check_assembly.py   # must print ALL CLEAR
../.venv/bin/python export_step.py && ../.venv/bin/python export_assembly.py
```

Servo dimensions were taken from the **official Waveshare ST3215 STEP model and
2D drawing** (measured programmatically, not eyeballed) — sources in the
`dimensions.py` docstring. Every joint is supported on **both** sides: the servo's
metal horn (Ø19.2, 4× M3 on a Ø14 bolt circle) on one side and its built-in
free-spinning **rear idler disc** (same Ø19.2 / 4× M3 pattern, recessed in a Ø25
opening) on the other — no extra bearings needed.

## Part list (13 prints, 7 unique)

| part | qty | bbox (mm) | ~mass | role |
|---|---|---|---|---|
| `pelvis` | 1 | 46 × 104 × 48 | 47 g | deck + two hanging bays clamping the hip-roll servos |
| `yoke_roll` | 2 | 48 × 34 × 32 | 12 g | clevis on roll-servo horn/idler, flange down |
| `yoke_pitch` | 2 | 32 × 45 × 42 | 12 g | clevis on thigh-servo horn/idler, bolts under `yoke_roll` rotated 90° (hip universal joint) |
| `leg_link` | 4 | 28 × 45 × 99 | 18 g | thigh **and** shin (same part): grips a servo case, forks 90 mm down to the next servo's horn/idler |
| `foot` | 2 | 96 × 52 × 30 | 24 g | sole + ankle-servo pocket + rear retention walls + TPU pad recess |
| `tower` | 1 | 45 × 96 × 37 | 37 g | electronics: driver board hangs face-down on standoffs INSIDE; 3S battery tilt-loads through the rear-wall window onto the deck (tool-free swap: peel belt, tug ribbon); GoPro bosses on top |
| `gopro_base` | 1 | 30 × 24 × 21 | 5 g | GoPro three-prong mount, bolts to the tower top (crash fuse — cheap to reprint) |

Printed plastic ≈ 257 g. Total robot ≈ **0.86 kg** bare, **1.01 kg with the
GoPro MAX** (8 servos 440 g, 3S LiPo ~78 g, board ~20 g, fasteners ~47 g,
TPU 16 g, camera 154 g). Heights: ankle 16.9, knee 106.9, hip-pitch 196.9,
hip-roll 246.9, torso top 324 mm, camera CG ≈ 378 mm. Standing CG rises from
≈ 168 mm (bare) to ≈ 200 mm with the camera (**+31 mm**) — expect gait retuning.

### GoPro mount (`gopro_base`)

Receives the GoPro MAX's built-in folding two-finger mount, lens axis fore-aft.
Prong geometry follows the proven [GoProScad](https://github.com/ridercz/GoProScad)
standard: 3.0 mm prongs, Ø15 rounded tops, M5 hole (printed Ø5.5) 9.5 mm above
the base top, 17 mm legs. Slots are **3.2 mm** (≈0.25 mm clearance on ~2.95 mm
camera fingers) — snugger than GoProScad's 3.5 mm; if your camera's fingers
bind, ream the slots or set `GP_SLOT = 3.5` in `dimensions.py` and reprint.
Clamp with the camera's own thumbscrew or any M5×20. The base bolts down with
4× M3×8 self-tappers into bosses under the tower top plate; the driver board
moved inside the tower (face-down on 6 mm standoffs) to make room and lower
the electronics CG.

All legs use identical parts: every pitch-joint horn faces **+Y (robot left)** —
legs are translations, not mirrors.

## BOM

| item | qty | notes |
|---|---|---|
| Waveshare ST3215, 12 V version (6–12.6 V) | 8 | 30 kg·cm@12 V; includes metal horn + idler disc + M3×6 screws; do NOT buy the 4–7.4 V class (see docs/hardware-order.md) |
| Waveshare Servo Driver with ESP32 | 1 | 65 × 30 mm; `dimensions.py::BOARD_HOLES` = published Ø2.75 @ 58 × 23 — **still verify against your board before printing the tower** |
| Zeee 3S 850 mAh 100C XT30 (2-pack) | 2 | 67 × 30 × 18.5 mm, 74 g; bay is parametric on `BATT` — different pack, edit + reprint tower |
| 20 mm hook-loop strap ~250 mm + pull ribbon | 1 | battery belt (rides in the tower guide ribs) + extraction tab under the pack |
| M3×6 button head | 32 | horn pads (4 per joint × 8) — servo kits include some |
| M3×8 button head + M3 thin washer | 24 | idler pads at hip-pitch, knee, ankle (4 × 6 joints); washer stops the tip short of the gears |
| M3×10 button head | 16 | yoke_roll idler arms (4 × 2, through the long boss) + hip flange bolts (4 × 2) |
| M3×8 self-tapping (or machine after M3-tapping the case) | 48 | case grips: 6 per leg_link (24), 8 per pelvis bay (16), 4 per foot (8) |
| M3 heat-set insert (Ø4.6 × 4–6) | 12 | 4 per yoke_pitch flange (8) + 4 in the pelvis deck for the tower |
| M3×8 self-tapping | 4 | gopro_base down into the tower-top bosses |
| M5×20 GoPro thumbscrew | 1 | or use the camera's own folding-mount screw |
| M2.5×8 self-tapping | 4 | driver board, from below into the standoffs under the tower top |
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
| tower | upside-down (top plate on bed) | feet tabs have 45° gussets (enlarged for the camera load); board standoffs and GoPro bosses print upward from the plate — still support-free |
| gopro_base | base down, prongs up | standard orientation for printed GoPro mounts; use PETG or 100 % infill PLA — the M5 clamp squeezes across layer lines |

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
7. **Torso**: press 4 inserts into the deck bosses; screw the driver board
   face-down onto the standoffs under the tower top (4× M2.5×8 from below,
   connectors toward an open end) BEFORE bolting the tower down; bolt the
   tower down (4× M3 through the feet tabs — driver access holes are in the
   top plate). Lay the pull ribbon across the deck, tilt the battery in
   through the rear-wall window (over the sill, onto the far-wall rails),
   and close the hook-loop belt around the tower in its guide ribs — the
   battery goes in LAST and swaps without touching a screw. Screw the
   `gopro_base` onto the top bosses (4× M3×8); the camera clamps with its
   M5 thumbscrew, lens fore-aft.
8. **Wiring**: daisy chain — board → both roll servos (connectors are inside the
   torso) → down the back of each leg (zip ties through the web holes, leave a
   slack loop across each joint sized at full flexion) → thigh → shin → ankle.
9. Verify each joint's direction/sign against the sim (`DESIGN.md` TODO) before
   trusting any gait; run range-of-motion slowly at low torque limit first.

## Open questions

- **Servo case thread**: M3 self-tap vs tap vs M4 (see BOM note). Measure a real case.
- **Idler screw length**: M3×8 + washer assumes the disc's 3.35 mm thread depth;
  confirm screws don't bottom against the gear behind the disc.
- **Driver board**: exact model + hole pattern (`BOARD_HOLES`) — it now hangs
  inside the tower; confirm its component heights (< ~4 mm toward the battery)
  and that the 18.5 mm-tall pack still leaves ~2 mm under the board.
- **Battery swap fit**: test-fit the real Zeee pack through the window before
  final assembly — sill height 2.5 mm, opening sized to the `BATT` envelope
  (68 × 20); a puffier pack means editing `BATT` and reprinting the tower.
- **GoPro fit**: measure your MAX's finger thickness — slots are printed 3.2 mm
  (GoProScad standard is 3.5); confirm the folded-finger depth clears the
  17 mm prong height, and that 154 g up top (+29 mm CG) is retrained/retuned
  in sim before real walking.
- **Cable service loops** across knee (95°) — verify lengths with real 150 mm leads;
  extension cables may be needed shin→ankle depends on port placement.
- **Foot pad material**: TPU sheet vs printed TPU vs adhesive rubber; friction
  should roughly match sim (μ ≈ 1.0).
- Ankle axis is 16.9 mm (not 14) above ground → use `sim/bimo_biped_v2.xml`.
