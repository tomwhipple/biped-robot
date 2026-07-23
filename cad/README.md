# Printable CAD — Bimo-like biped

Parametric CAD in Python ([build123d](https://build123d.readthedocs.io/)), realizing the
kinematics of `sim/bimo_biped.xml` with **10× Feetech STS3215** bus servos
(v3yaw: 8 leg + 2 hip-yaw — see [hip-yaw study](../docs/hip-yaw-study.md)).

```
cad/
├── dimensions.py      # ALL dimensions (single source of truth) + servo datasheet notes
├── parts.py           # part builders; run it to export STLs + mass/bed report
├── check_assembly.py  # boolean interference checks over the full joint ranges
├── export_step.py     # per-part STEP exports (cad/step/)
├── export_assembly.py # assembled robot -> cad/step/assembly.step (+ camera mock)
├── dress.py           # posable dressed robot: + cables, zip ties, board, pigtail
├── export_assembly_full.py  # dressed robot -> step/assembly_full.step + stills
├── animate_dressed_rom.py   # leg ROM video with the wiring following the joints
├── render_assembly.py # PNG render of the assembly via MuJoCo
├── render_assembly_steps.py  # per-step figures for docs/assembly.md
├── animate_assembly.py # fly-in feasibility animation (real insertion paths)
├── freecad_articulate.py  # build the POSEABLE FreeCAD assembly (10 revolute joints)
├── stl/               # exported STLs (one per unique part)
├── step/              # exported STEPs + assembly.step / assembly_full.step
│                      #   + bimo_v3yaw_articulated.FCStd (poseable)
└── bimo_like_biped.scad  # (older massing concept, superseded by parts.py)
```

```bash
../.venv/bin/python parts.py            # build + export + verify (bed fit, masses, CG)
../.venv/bin/python check_assembly.py   # must print ALL CLEAR
../.venv/bin/python export_step.py && ../.venv/bin/python export_assembly.py
../.venv/bin/python export_assembly_full.py   # dressed: wiring/board/battery mocks
../.venv/bin/python render_assembly_steps.py  # refresh docs/assembly.md figures
../.venv/bin/python animate_assembly.py       # fly-in animation (gif+mov, gitignored)
```

## Pose it in FreeCAD (10-DOF articulated assembly)

`freecad_articulate.py` builds an **Assembly-workbench** model you can drag
through its full range of motion — one **Revolute** joint per axis, with the
sim's limits baked in (hip yaw ±45°, hip roll ±25°, hip pitch −110/+60°, knee
−95/+5°, ankle ±40°). The torso is grounded; both legs share the same part
STEPs. Build it (STEPs must exist — run `export_step.py` first):

```bash
# macOS (adjust the path on Linux/Windows to your freecadcmd)
/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd cad/freecad_articulate.py
# -> cad/step/bimo_v3yaw_articulated.FCStd
```

or from the GUI: **Macro → Macros… → add `freecad_articulate.py` → Execute**.

**To pose it:** open `cad/step/bimo_v3yaw_articulated.FCStd`, switch to the
**Assembly** workbench, then either **drag any part** with the mouse (the
solver keeps every joint honest and stops each axis at its limit) or
double-click a joint in the tree and type an angle. The neutral pose is the
CAD standing pose (feet on the ground). The macro injects a `GuiDocument.xml`
(view state + fitted isometric camera) so the file opens visible even though it
was built headless. *(FreeCAD prints ~20 benign "invalid Reference" warnings on
open — a PartDesign migration quirk that doesn't apply to these LCS joints; they
solve and pose fine.)*

If a future FreeCAD version still opens with hidden parts: **select-all in the
tree → press Space → View ▸ Fit All**.

`assembly_full.step` is the "approximate complete" model: everything in
`assembly.step` plus mock dress — driver board under the tower top, battery,
XT30 pigtail, per-leg servo daisy-chain cables (splines through the real
ST3215 rear-end connector positions, tied to the web raceways with zip-tie
mocks). Approximate by design: wire paths are plausible, not catalog-exact.
The BNO055 IMU now rides its printed `imu_carrier` under the gopro_base and
IS in the model (`docs/assembly.md` §9b, figures from
`render_electronics_steps.py`); the power switch mount is still undesigned.
The cable
segments regenerate from the posed joint frames, so `dress.dressed_robot(
roll, hip, knee, ankle)` and the ROM video show the wiring following the
legs rather than a rigid cable tearing off.

Servo dimensions were taken from the **official Waveshare ST3215 STEP model and
2D drawing** (measured programmatically, not eyeballed) — sources in the
`dimensions.py` docstring. Every joint is closed on **both** sides: the servo's
metal horn (Ø19.2, 4× M3 on a Ø14 bolt circle) on one side and its built-in
free-spinning **rear idler disc** (same Ø19.2 / 4× M3 pattern, recessed in a Ø25
opening) on the other — no extra bearings needed. Two honest caveats on the
idler side (issue #5): the printed Ø19 boss inside the Ø25 recess is a
**locator, not a precision seat** (~3 mm radial clearance) — concentricity
comes from the 4× M3 pattern, so *snug the idler screws with the joint at
mechanical zero and check runout before final torque*; and the idler disc's
molded M3 threads carry the far-side bending shear — inspect them after the
first hours of walking, and if they wear, the upgrade path is a shoulder
bolt through the disc or a thin 19×27 washer-bearing under the arm.

## Part list (14 prints, 8 unique)

| part | qty | bbox (mm) | ~mass | role |
|---|---|---|---|---|
| `pelvis` | 1 | 46 × 104 × 48 | 47 g | deck + two hanging bays clamping the hip-roll servos |
| `yoke_roll` | 2 | 48 × 34 × 32 | 12 g | clevis on roll-servo horn/idler, flange down |
| `yoke_pitch` | 2 | 32 × 45 × 42 | 12 g | clevis on thigh-servo horn/idler, bolts under `yoke_roll` rotated 90° (hip universal joint) |
| `leg_link` | 4 | 28 × 45 × 99 | 18 g | thigh **and** shin (same part): grips a servo case, forks 90 mm down to the next servo's horn/idler |
| `foot` | 2 | 100 × 52 × 30 | 36 g | flat sole + ankle-servo pocket + heel bulkhead tying the retention tabs into a U-channel (glued rubber sole pad) |
| `tower` | 1 | 45 × 96 × 43.5 | 39 g | electronics: driver board hangs face-down on standoffs INSIDE; 3S battery tilt-loads through the rear-wall window onto the deck (tool-free swap: peel belt, tug ribbon); GoPro bosses on top |
| `gopro_base` | 1 | 30 × 24 × 21 | 5 g | GoPro three-prong mount, bolts to the tower top (crash fuse — cheap to reprint) |
| `imu_carrier` | 1 | 30 × 48.5 × 7.5 | 5 g | BNO055 carrier between the tower top and gopro_base — same 4 screws (M3×12); IMU screws to bosses on its true 21.59 × 15.24 hole pattern, jumpers drop down the tower's open rear end |

Printed plastic ≈ 286 g (PETG). Total robot ≈ **0.88 kg** bare, **1.04 kg with
the GoPro MAX** (8 servos 440 g, 3S LiPo ~80 g, board ~20 g, fasteners ~47 g,
TPU 16 g, camera 154 g). Heights: ankle 16.9, knee 106.9, hip-pitch 196.9,
hip-roll 246.9, torso top 330.5 mm, camera CG ≈ 384 mm. Standing CG rises from
≈ 165 mm (bare) to ≈ 197 mm with the camera (**+32 mm**) — expect gait retuning.

### GoPro mount (`gopro_base`)

Receives the GoPro MAX's built-in folding two-finger mount, lens axis fore-aft.
Prong geometry follows the proven [GoProScad](https://github.com/ridercz/GoProScad)
standard: 3.0 mm prongs, Ø15 rounded tops, M5 hole (printed Ø5.5) 9.5 mm above
the base top, 17 mm legs. Slots are **3.2 mm** (≈0.25 mm clearance on ~2.95 mm
camera fingers) — snugger than GoProScad's 3.5 mm; if your camera's fingers
bind, ream the slots or set `GP_SLOT = 3.5` in `dimensions.py` and reprint.
Clamp with the camera's own thumbscrew or any M5×20. The base bolts down with
4× M3×12 self-tappers through the `imu_carrier` beneath it into bosses under
the tower top plate; the driver board
moved inside the tower (face-down on 6 mm standoffs) to make room and lower
the electronics CG.

All legs use identical parts: every pitch-joint horn faces **+Y (robot left)** —
legs are translations, not mirrors.

## BOM

| item | qty | notes |
|---|---|---|
| Waveshare ST3215, 12 V version (6–12.6 V) | 8 | 30 kg·cm@12 V; includes metal horn + idler disc + M3×6 screws; do NOT buy the 4–7.4 V class (see docs/hardware-order.md) |
| BNO055 IMU breakout (ordered: Amazon B0GVK81HXR, classic layout) | 1 | mounts on the printed imu_carrier; 4× F-F jumpers to the ESP32 |
| Waveshare Servo Driver with ESP32 | 1 | 65 × 30 mm; `dimensions.py::BOARD_HOLES` = published Ø2.75 @ 58 × 23 — **still verify against your board before printing the tower** |
| 3S 850 mAh XT30 pack (2-pack) | 2 | BOM pick Tattu 45C, 60 × 30 × 22 mm, 76 g. Bay envelope `BATT` = 68 × 31 × 26.5 is a superset of the 3S 850 field (see `docs/bom-by-vendor.md` fit table); mocks/inertia model the worst case `BATT_PACK` = 62 × 30 × 25, 80 g. Taller pack → edit `BATT` + reprint tower |
| 20 mm hook-loop strap ~250 mm + pull ribbon | 1 | battery belt (rides in the tower guide ribs) + extraction tab under the pack |
| M3×6 button head | 32 | horn pads (4 per joint × 8) — servo kits include some |
| M3×8 button head + M3 thin washer | 24 | idler pads at hip-pitch, knee, ankle (4 × 6 joints); washer stops the tip short of the gears |
| M3×10 button head | 16 | yoke_roll idler arms (4 × 2, through the long boss) + hip flange bolts (4 × 2) |
| M3×8 self-tapping (or machine after M3-tapping the case) | 48 | case grips: 6 per leg_link (24), 8 per pelvis bay (16), 4 per foot (8) |
| M3 heat-set insert (Ø4.6 × 4–6) | 12 | 4 per yoke_pitch flange (8) + 4 in the pelvis deck for the tower |
| M3×12 self-tapping | 4 | gopro_base + imu_carrier stack down into the tower-top bosses |
| M5×20 GoPro thumbscrew | 1 | or use the camera's own folding-mount screw |
| M2.5×8 self-tapping | 8 | driver board, from below into the standoffs under the tower top (4) + BNO055 onto the imu_carrier bosses (4) |
| Self-adhesive rubber sole pad (~0.5 mm) | 2 | stick onto the flat foot underside, trim to fit; keep thin so stance height is unchanged |
| zip ties 2.5 mm | ~10 | cable dressing through leg_link web holes |

Servo case mounting holes measure Ø3.5 in the vendor STEP; the community
practice (glass-filled nylon case) is M3 self-tappers or tapping M3.
**Verify on one real servo first** — if the holes are truly Ø3.5, use M4
self-tappers in the pelvis/foot walls (clearance holes are parametric).

## Print settings

PLA or PETG, 0.4 mm nozzle, 0.2 mm layers, 3 perimeters (all walls ≥ 2.4 mm are
perimeter-only), 30–40 % infill, no slicer supports needed — and this is now
**verified geometrically** by `check_printability.py`, which audits every STL
in its print orientation for bridges / >45° overhangs / floating starts and
must report `ALL PARTS PRINT CLEAN` (run it alongside `check_assembly.py`
after any CAD change):

| part | orientation | note |
|---|---|---|
| pelvis | upside-down (deck top on bed) | bay walls print vertically; bore is a downward-open slot; deck top is flush (no bosses) so the first layer is the whole deck face |
| yoke_roll / yoke_pitch | flange face on bed | arms vertical → layer lines ⟂ arm bending is avoided; idler boss prints as a short horizontal stub (1–4 mm) — slight underside droop is cosmetic, the seat face prints clean |
| leg_link | on its back (web on bed) | strongest orientation for fore-aft bending; **slice `leg_link_print.stl`** — it adds 3 break-away fins under the fork slabs, which float 4.7 mm above the bed (the swept joint envelope forbids solid material there); peel the fins out after printing |
| foot | sole down | heel tabs, bulkhead and buttress are vertical faces or top-side slopes — nothing bridges |
| tower | upside-down (top plate on bed) | fully support-free: the battery window is **open to the deck** (no sill — the belt retains the pack; 45° corner detents park it), feet-tab gussets and rail stubs are true ≥45° wedges, belt-rib undersides chamfered. Only ceilings: the Ø6.6 feet-screw counterbores (normal short bridges) |
| gopro_base | base down, prongs up | standard orientation for printed GoPro mounts; use PETG or 100 % infill PLA — the M5 clamp squeezes across layer lines |
| imu_carrier | flat on bed, bosses up | trivial print; the only notes are the intentional 0.4 mm pilot floors under the M2.5 bosses |

All horizontal M3 bores are teardropped toward each part's print-up direction
(self-supporting 45° bore roofs — no sagged strands where screws clamp).

## Assembly order

**Illustrated step-by-step guide: [docs/assembly.md](../docs/assembly.md)**
(one figure per step, rendered from the CAD by `render_assembly_steps.py` —
regenerate after any CAD change). Condensed order:

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
  inside the tower; confirm its component heights (< ~4 mm toward the battery).
  That 4 mm assumption is what sets the 3.5 mm clearance to a 26.5 mm pack
  (`TOWER_H = BATT[2] + 13.5 + 3.5`); taller underside parts eat it directly.
- **Battery swap fit**: test-fit the real pack through the window before final
  assembly — sill height 2.5 mm, opening sized to the `BATT` envelope
  (68 × 26.5); a pack taller than 26.5 means editing `BATT` and reprinting the
  tower. Short packs (59–62 mm vs the 68 mm bay) have up to 8 mm of Y slop —
  confirm the belt + ribbon actually restrain it, or add a shim.
- **GoPro fit**: measure your MAX's finger thickness — slots are printed 3.2 mm
  (GoProScad standard is 3.5); confirm the folded-finger depth clears the
  17 mm prong height, and that 154 g up top (+29 mm CG) is retrained/retuned
  in sim before real walking.
- **Cable service loops** across knee (95°) — verify lengths with real 150 mm leads;
  extension cables may be needed shin→ankle depends on port placement.
- **Foot pad material**: TPU sheet vs printed TPU vs adhesive rubber; friction
  should roughly match sim (μ ≈ 1.0).
- Ankle axis is 16.9 mm (not 14) above ground → use `sim/bimo_biped_v2.xml`.
