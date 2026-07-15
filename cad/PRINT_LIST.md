# Print list — Bimo-like biped

**13 plastic prints (7 unique parts) + 2 TPU foot pads.** Generated from
`cad/parts.py` / `cad/dimensions.py`. Export STLs with
`../.venv/bin/python parts.py` (writes `cad/stl/*.stl`).

## Global print settings

**PETG for the whole robot** (0.4 mm nozzle, 0.2 mm layers, 3 perimeters — every
wall ≥ 2.4 mm is perimeter-only — 30–40 % infill, **no supports**; every part has
a support-free orientation, see table). PETG chosen for its toughness and impact
resistance — the right call for a machine that falls repeatedly during get-up
training, and confirmed noticeably more solid on the `gopro_base` test print.

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
| `pelvis` | 1 | PETG | 30–40 % | upside-down, deck top on bed | ✅ ready — *secondary* hip-angle check |
| `yoke_roll` | 2 | PETG | 30–40 % | flange face on bed, arms up | ✅ ready — *secondary* hip-angle check |
| `yoke_pitch` | 2 | PETG | 30–40 % | flange face on bed, arms up | ⚠️ **HOLD — being redesigned** (hip-angle revisit) |
| `leg_link` | 4 | PETG | 30–40 % | on its back, web face on bed | ⚠️ **verify before printing** (2 of 4 are thighs, on the hip-pitch joint) |
| `foot` | 2 | PETG | 30–40 % | sole down | ♻️ **revised v2** (flat sole + reinforced tabs) |
| `tower` | 1 | PETG | 30–40 % | upside-down, top plate on bed | ✅ ready |
| `gopro_base` | 1 | PETG | — | base down, prongs up | ✅ printed in PETG |
| Sole pad (self-adhesive rubber) | 2 | rubber | — | trim to fit | 🛒 on order (stick onto flat sole) |

`leg_link` ×4 = 2 thighs + 2 shins (identical part). `gopro_base` is the
sacrificial crash fuse — the M5 clamp squeezes across layer lines, so PETG is
especially warranted there (already printed, noticeably more solid).

**Foot v2 (2026-07-15):** the first foot print sagged badly on the underside —
the old TPU-pad recess bridged a 46 mm span printed sole-down. Fixed by making
the **sole flat** (no recess; stick a thin self-adhesive rubber pad on, trimmed
to fit) and **reinforcing the heel retention tabs** (one snapped in handling):
tabs lengthened toward the heel (heel +4 mm → foot now 100 mm long) with a 45°
aft base buttress. Fore is unchanged — the shin fork sweeps that zone. `foot`
is now ~33 g, bbox 100 × 52 × 30 mm; ankle height unchanged. Re-run of
`check_assembly.py` is ALL CLEAR.

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
