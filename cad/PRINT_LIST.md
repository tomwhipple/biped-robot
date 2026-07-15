# Print list — Bimo-like biped

**13 plastic prints (7 unique parts) + 2 TPU foot pads.** Generated from
`cad/parts.py` / `cad/dimensions.py`. Export STLs with
`../.venv/bin/python parts.py` (writes `cad/stl/*.stl`).

## Global print settings

**PLA for the whole robot** (0.4 mm nozzle, 0.2 mm layers, 3 perimeters — every
wall ≥ 2.4 mm is perimeter-only — 30–40 % infill, **no supports**; every part has
a support-free orientation, see table). PLA is fine structurally for every part.
PETG is an *optional* upgrade if you already have it (a little more heat/impact
margin) — not required anywhere. The only part with a real material preference is
`gopro_base`, and it's satisfied by **PLA at 100 % infill** (see its row).

## Parts to print

| Part | Copies | Material | Infill | Orientation | Status |
|---|---|---|---|---|---|
| `pelvis` | 1 | PLA | 30–40 % | upside-down, deck top on bed | ✅ ready — *secondary* hip-angle check |
| `yoke_roll` | 2 | PLA | 30–40 % | flange face on bed, arms up | ✅ ready — *secondary* hip-angle check |
| `yoke_pitch` | 2 | PLA | 30–40 % | flange face on bed, arms up | ⚠️ **HOLD — being redesigned** (hip-angle revisit) |
| `leg_link` | 4 | PLA | 30–40 % | on its back, web face on bed | ⚠️ **verify before printing** (2 of 4 are thighs, on the hip-pitch joint) |
| `foot` | 2 | PLA | 30–40 % | sole down | ✅ ready |
| `tower` | 1 | PLA | 30–40 % | upside-down, top plate on bed | ✅ ready |
| `gopro_base` | 1 | PLA (**100 % infill**) | 100 % | base down, prongs up | ✅ ready |
| TPU foot pad (90 × 46 × 2 mm) | 2 | **TPU** | — | flat | ✅ ready (or cut from TPU sheet) |

`leg_link` ×4 = 2 thighs + 2 shins (identical part). `gopro_base` is the
sacrificial crash fuse — the M5 clamp squeezes across layer lines, so print it
**PLA at 100 % infill** (or PETG if you have it). TPU pads glue into the foot
sole recess and sit 0.5 mm proud.

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
