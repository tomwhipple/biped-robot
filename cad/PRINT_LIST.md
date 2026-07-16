# Print list — Bimo-like biped

**13 plastic prints (7 unique parts) + 2 TPU foot pads.** Generated from
`cad/parts.py` / `cad/dimensions.py`. Export STLs with
`../.venv/bin/python parts.py` (writes `cad/stl/*.stl`).

## Global print settings

**PETG for the whole robot** (0.4 mm nozzle, 0.2 mm layers, 3 perimeters — every
wall ≥ 2.4 mm is perimeter-only — 30–40 % infill, **no slicer supports**; every
part is support-free in its listed orientation, and this is now *verified
geometrically*, not asserted: `check_printability.py` audits every STL in its
print orientation for bridges, >45° overhangs, floating islands, skimpy
first-layer contact and sub-perimeter thin walls, and must print
`ALL PARTS PRINT CLEAN` before printing —
run it like `check_assembly.py` after any CAD change. (The first foot print's
46 mm sagged bridge and three other would-be failures — pelvis, tower,
leg_link, below — are exactly what it catches.) PETG chosen for its toughness
and impact resistance — the right call for a machine that falls repeatedly
during get-up training, and confirmed noticeably more solid on the
`gopro_base` test print.

Two things to watch when you print PETG:
- **Tolerances.** The design uses generous drop-in fits (`FIT = 0.30`), but PETG
  runs hotter and strings more than PLA. Check the snug interfaces on the first
  parts — servo pockets (`foot`), idler bosses (Ø19 into the Ø25 recess), and the
  GoPro slots (3.2 mm) — and tune flow / dial in a size test if anything binds.
- **Mass.** PETG (~1.27 g/cm³) is ~2–3 % denser than PLA. The mass rollup uses
  the PETG density (`dimensions.py::FILAMENT_RHO = 1.27e-3`): printed plastic
  ~303 g, total robot ~910 g (+154 g GoPro) after foot v3.1, the yoke_pitch
  hip-110 revision, the battery-bay tower, and the leg_link v2 cleanup.
  `parts.py` and the sim inertia builder (`sim/build_v2_inertia.py`) both
  reflect this.

## Parts to print

| Part | Copies | Material | Infill | Orientation | Status |
|---|---|---|---|---|---|
| `pelvis` | 1 | PETG | 30–40 % | upside-down, deck top on bed | ✅ ready (bosses removed — see audit) — *secondary* hip-angle check |
| `yoke_roll` | 2 | PETG | 30–40 % | flange face on bed, arms up | ✅ ready — *secondary* hip-angle check |
| `yoke_pitch` | 2 | PETG | 30–40 % | flange face on bed, arms up | ♻️ **revised for hip −110°** (idler arm → hub + riser; HOLD lifted) |
| `leg_link` | 4 | PETG | 30–40 % | on its back, web face on bed — **slice `leg_link_print.stl`** | ♻️ **revised v2** (print-review cleanup: idler-side edge flush, dead 0.7 mm slot by the horn fork filled; +0.7 g). *Cosmetic/stiffness only — the already-printed v1 fits and works identically* |
| `foot` | 2 | PETG | 30–40 % | sole down | ♻️ **revised v3.1** (v3 heel bulkhead + sole enlarged 100 → 116 for the get-up corridor) |
| `tower` | 1 | PETG | 30–40 % | upside-down, top plate on bed | ♻️ **revised for print, support-free** (2026-07-16 slice reviews: feet-tab gussets and battery-rail stubs are now true ≥45° wedges — the old stepped boxes left flat 6 mm ceilings drooping over the interior; the **window sill was deleted** — it printed as a 70 mm member 41 mm up in mid-air, and the hook-loop belt is the real battery retention; two 45° corner detents park the pack instead. Feet screws now seat on the tabs through Ø6.6 wells — the only remaining bridges. ~5h19m PETG) |
| `gopro_base` | 1 | PETG | — | base down, prongs up | ✅ printed in PETG |
| Sole pad | 2 | 1/16" self-adhesive silicone sheet ([B0FJ8TBMQK](https://www.amazon.com/dp/B0FJ8TBMQK), 2× 6"×6") | — | cut 106 × 46 mm, stick onto flat sole (one sheet yields both + a spare strip) | 🛒 ordered |

`leg_link` ×4 = 2 thighs + 2 shins (identical part). `gopro_base` is the
sacrificial crash fuse — the M5 clamp squeezes across layer lines, so PETG is
especially warranted there (already printed, noticeably more solid).

**Foot v3 (2026-07-15):** v1 taught two lessons — the TPU-pad recess ceiling
bridged 46 mm and sagged (fixed in v2 by the **flat sole**, kept in v3: stick a
thin self-adhesive rubber pad on, trimmed to fit), and a heel retention tab
snapped **across layer lines under a lateral knock**. v2's aft gusset only
stiffened the blades fore-aft, so v3 ties the two tabs into a **heel bulkhead**
behind the servo (U-channel; cable window opens at the top for the rear-exit
servo cable) plus one full-width aft buttress. Every added face is vertical —
nothing new bridges. Retention screw bores are teardropped; ankle height
unchanged. `check_assembly.py` ALL CLEAR, `check_printability.py` clean.
Render: `renders/foot_v3.png`.

**Foot v3.1 + yoke_pitch hip-110 revision (2026-07-15, later):** the get-up
decision landed (DESIGN: rise corridor is real but ±20 mm on the old 90 mm
pad, and the pike needs ≥95° hip flexion). Two changes:

- **`foot` sole enlarged 100 → 116 mm** fore-aft, heel-biased (heel 42 → 52,
  toe 58 → 64; the rise tips *backward*): ~42 g, bbox 116 × 52 × 30 mm. Walls,
  tabs, bulkhead, and ankle height unchanged. The pad grows with it: cut
  106 × 46 mm from the 1/16" self-adhesive silicone sheet (see pad row); its
  full 1.6 mm rides proud of the flat sole (stance +1.1 mm vs the old 0.5
  assumption — `TPU_PROUD`, propagated to sim).
- **`yoke_pitch` idler arm redesigned**: the old full-width arm plate capped
  hip flexion at ~105° (the thigh link's idler grip plate shares its Y band
  and sweeps into it). Now a Ø28 hub + riser plate routed through the unswept
  top-rear sector, with a 45° print chamfer on the hub's front-upper quadrant
  (support-free flange-down). Clears −115°/+65° with the full leg_link in the
  sweep; horn arm unchanged. Render: `renders/yoke_pitch_v2.png`.

**Printability audit (2026-07-15, `check_printability.py`):** three parts that
were marked "ready" would have failed exactly like the first foot:

- **`pelvis`** — printed deck-down it stood on its four raised tower bosses,
  holding the *entire first layer* 2 mm in the air (the bosses also overlapped
  the tower feet tabs). Bosses deleted; heat-set pilots now run through the
  deck into the bay-cheek material below (thread depth intact). Tower now
  seats flush.
- **`pelvis` (second pass, print review)** — each bay wall left a **0.24 mm**
  web between the 8.30 case-screw hole and the roll-axis U-slot: under one
  extrusion, so the slicer merged hole and bore into a sliver. `BAY_BORE` was
  Ø22.5 (1.45 mm radial slack on a Ø19.6 boss) while the screw row sits only
  13.19 mm from the axis. Bore is now `SV_BOSS_D + 1.0` = **Ø20.6** — the same
  0.5 mm radial slip `leg_link` already proves against that boss — restoring a
  **1.19 mm** web. Boss clearance and the full servo slide-up re-verified at
  0.00 mm³. Before/after: `renders/pelvis_bay_web.png`.
  **`check_printability.py` PASSed this** — it measured all 16 slivers at
  3.65 mm² and dropped every one under the 4.0 mm² knife-edge filter. Threshold
  is now 3.0 (fails the old geometry, no false positive on any current part) and
  sub-threshold zones print as `thin-note` instead of vanishing.
- **`tower`** — the battery-window sill printed as a 70 mm single-wall bridge,
  and the belt-guide ribs as drooping square ledges. A first fix ramped the
  sill top 45° and chamfered the ribs. **Second slice review (2026-07-16)**
  caught what that pass missed: the feet-tab *gussets* were still flat-topped
  boxes (6 mm ceilings 31 mm up, over nothing), the battery-rail stubs only
  stepped 1.3 mm per ledge, and the sill ramp measured 44° — one degree under
  the printable threshold, so slicers painted the whole 70 mm band as
  overhang. Gussets and stubs became true ≥45° wedges; break-away support
  trees for the sill were designed, sliced, and then made obsolete by the
  real question (*is the member even needed?*): **the sill is deleted**. The
  hook-loop belt was always the battery's tumble retention — the sill only
  parked the pack while the belt was off, and two 45° corner detents growing
  off the window posts do that support-free. `check_printability` CEILING
  count 13 → 4; the four left are the Ø6.6 feet-screw counterbores (standard
  short bridges), confirmed as the *only* bridge/overhang blocks in the
  sliced g-code. No `tower_print.stl` needed — slice `tower.stl` upside down.
- **`leg_link`** — the narrow fork slabs float 4.7 mm above the bed for their
  last 50 mm (they *cannot* reach the bed: that volume is swept by the foot
  walls / servo case top at joint extremes). The exported
  **`leg_link_print.stl`** adds three break-away fins (0.2 mm separation gap)
  under the slabs and idler-boss rim — **slice that file**, then peel the fins
  out; `leg_link.stl` stays clean for the sim meshes and assembly checks.
- All horizontal M3 bores (servo-case screws, horn/idler bolt circles, foot
  retention) are now **teardropped** toward each part's print-up direction, so
  no bore top bridges (the `gopro_base` M5 fix, applied everywhere).

## ⚠️ Hip-angle revisit — what's in flux

The get-up study (`DESIGN.md`, commit `d8c9eb8`) proved fall-recovery needs
**≥ 95° hip-pitch flexion**; the current mechanical cap is **±60°**. The CAD
target is now **−110° / +60°**. As of this list the finding is **sim/analysis
only — no CAD has been re-cut** (`parts.py`/`dimensions.py` unchanged since
2026-07-12, geometry tagged `v1-hip60-backup`). Print status by part:

- **`yoke_pitch` — HOLD.** Primary flexion limiter (the thigh-servo clevis).
  This is the part the pending "yoke redesign" re-cuts to open −110°.
  **Do not print the final pair yet.** A test print at current geometry is fine
  for fit-checking the servo interface, but expect to reprint.
- **`leg_link` (thighs) — verify.** Mating swing partner; its web was already
  dimensioned for 95° folding (`WEB_TOP = -16`, `GRIP_TOP_IDLER = -16` clear the
  upper yoke-arm sweep), so it *may* survive the redesign unchanged. Print the
  2 **shins** now if you like; hold the 2 **thighs** until `yoke_pitch` is final.
- **`yoke_roll` / `pelvis` — secondary.** `YOKE_FLANGE_X = 32` is capped so the
  yoke swings past the pelvis bay walls; deep flexion may pull these into the
  redesign. Low risk, but re-run `check_assembly.py` (must print ALL CLEAR) at
  the new range before committing to final prints.

Everything else (`foot`, `tower`, `gopro_base`, TPU pads, both roll yokes as
roll-only, shins) is unaffected by the hip-angle work and safe to print now.

## Print-now vs hold — summary

| Print now | Copies | Hold for hip redesign | Copies |
|---|---|---|---|
| `pelvis` | 1 | `yoke_pitch` | 2 |
| `yoke_roll` | 2 | `leg_link` (thighs) | 2 |
| `leg_link` (shins) | 2 | | |
| `foot` | 2 | | |
| `tower` | 1 | | |
| `gopro_base` | 1 | | |
| TPU foot pad | 2 | | |

Re-run `check_assembly.py` at the new hip range and refresh this table once
`yoke_pitch` is re-cut.
