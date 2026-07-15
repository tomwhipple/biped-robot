# Print list — Bimo-like biped

**13 plastic prints (7 unique parts) + 2 TPU foot pads.** Generated from
`cad/parts.py` / `cad/dimensions.py`. Export STLs with
`../.venv/bin/python parts.py` (writes `cad/stl/*.stl`).

## Global print settings

**PETG for the whole robot** (0.4 mm nozzle, 0.2 mm layers, 3 perimeters — every
wall ≥ 2.4 mm is perimeter-only — 30–40 % infill, **no slicer supports**; every
part is support-free in its listed orientation, and this is now *verified
geometrically*, not asserted: `check_printability.py` audits every STL in its
print orientation for bridges, >45° overhangs, floating islands and skimpy
first-layer contact, and must print `ALL PARTS PRINT CLEAN` before printing —
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
- **Mass.** PETG (~1.27 g/cm³) is ~2–3 % denser than PLA. The mass rollup now
  uses the PETG density (`dimensions.py::FILAMENT_RHO = 1.27e-3`): printed plastic
  ~263 g (was ~257 g in PLA), total robot ~860 g. `parts.py` and the sim inertia
  builder (`sim/build_v2_inertia.py`) both reflect this.

## Parts to print

| Part | Copies | Material | Infill | Orientation | Status |
|---|---|---|---|---|---|
| `pelvis` | 1 | PETG | 30–40 % | upside-down, deck top on bed | ✅ ready (bosses removed — see audit) — *secondary* hip-angle check |
| `yoke_roll` | 2 | PETG | 30–40 % | flange face on bed, arms up | ✅ ready — *secondary* hip-angle check |
| `yoke_pitch` | 2 | PETG | 30–40 % | flange face on bed, arms up | ⚠️ **HOLD — being redesigned** (hip-angle revisit) |
| `leg_link` | 4 | PETG | 30–40 % | on its back, web face on bed — **slice `leg_link_print.stl`** | ⚠️ **verify before printing** (2 of 4 are thighs, on the hip-pitch joint) |
| `foot` | 2 | PETG | 30–40 % | sole down | ♻️ **revised v3** (heel bulkhead ties the tabs into a channel) |
| `tower` | 1 | PETG | 30–40 % | upside-down, top plate on bed | ✅ ready (sill/rib chamfers — see audit) |
| `gopro_base` | 1 | PETG | — | base down, prongs up | ✅ printed in PETG |
| Sole pad (self-adhesive rubber) | 2 | rubber | — | trim to fit | 🛒 on order (stick onto flat sole) |

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
nothing new bridges. `foot` is ~36 g, bbox 100 × 52 × 30 mm; ankle height
unchanged; retention screw bores are teardropped. `check_assembly.py` ALL
CLEAR, `check_printability.py` clean. Render: `renders/foot_v3.png`.

**Printability audit (2026-07-15, `check_printability.py`):** three parts that
were marked "ready" would have failed exactly like the first foot:

- **`pelvis`** — printed deck-down it stood on its four raised tower bosses,
  holding the *entire first layer* 2 mm in the air (the bosses also overlapped
  the tower feet tabs). Bosses deleted; heat-set pilots now run through the
  deck into the bay-cheek material below (thread depth intact). Tower now
  seats flush.
- **`tower`** — the battery-window sill printed as a 70 mm single-wall bridge,
  and the belt-guide ribs as drooping square ledges. The sill top is now a 45°
  ramp (full 2.5 mm retention lip kept on the outer face) and each rib carries
  a 45° chamfer on its print-underside. All self-supporting now.
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
