# v6 print list (2026-09-14)

Generated from `cad/v6/parts_v6.py` (`docs/design-v6/parts_v6_rollup.txt`). PETG 0.4 mm nozzle, 3 perimeters, 30–40 % infill unless noted; every part fits the 220 mm bed. Regenerate STL/STEP with `.venv/bin/python cad/v6/parts_v6.py`; each part module's `__main__` runs its own audits.

| part | qty | g each | orientation | supports | notes |
|---|---|---|---|---|---|
| `pelvis_v7` | 1 | 174 | deck-top-down (v5 pelvis orientation) | **yes**: the two yaw-cell ceilings (the case fills the cell to 0.3 mm, no support-free geometry exists) and the skin window roofs near the housing floor | one print. Boards slide down their deck slots; pack through the deck aperture; neck servo stands in the deck-top well, screwed from below through the aperture. Residual audit findings listed in `pelvis_v7.py`'s docstring — check the sliced preview at the window roofs and the deck-slot edges before printing |
| `neck_collar` | 1 | 20 | flange down, flat | no | slides down over the neck servo standing in the deck well (2.1 mm to the head's horn disc), 4 × M2.5 flat-heads at the flange corners into pilot holes in the deck (`NECK_FLANGE_HOLE_XY`, shared with `pelvis_v7.py`); the servo's own stator screws go up through the deck aperture from below. Split out of the pelvis on 2026-09-14: fused in, its mouth was the print's low point and the whole deck became an island |
| `head_shell` + `head_face` | 1 + 1 | 27 + 7 | shell crown-up (domed top ≥ 45°), face plate flat | no | 4 × M3×6 onto the neck horn from inside before the face goes on; Camera Module 3 on 4 × M2 bosses inside the face; ribbon slot in the base beside the axis. One dome-seam CEILING flag in the audit is believed to be a boolean artifact — verify in the slicer |
| `yaw_carrier` | 2 | 18 | as v5 (`yaw_carrier_print.stl` from `cad/parts.py`) | v5 fins | unchanged v5 part |
| `hip_yoke_v6` | 2 | 25 | on edge like v5 `yoke_roll` + brim | **yes**: pitch horn arm slab over the servo void, roll pad rims / arm plates start in mid-air | roll + pitch clevis as **one print** (default since 2026-09-24): replaces `yoke_roll` + `yoke_pitch_v6` and their 8 × M3×10 flange bolts + 8 heat-sets. 0.5 mm roll-corner relief on the flange top — [hip-yoke-single-print.md](hip-yoke-single-print.md) §6 |
| `leg_link_v6` | 4 | 26 | **standing on the lower fork end** (new: the box's front plate prints as a wall) | brim recommended (round pads meet the bed on a line) | thigh and shin are the same part; 110 mm; closed box section; one 25 mm end-anchored rib at the v5 jog block prints as a short bridge; 2026-09-14 flexion relief cuts (knee 130°) are 40–45° faces, no new supports; 2026-09-24 hip-flexion relief (`LL_HIP_RELIEF`: box top-front chamfer + jog-block trim) — faces up, no new supports |
| `ankle_link` | 2 | 12 | web-down like v5 (`RY_XUP`) | **yes** (CEILING/ISLAND class, same as the yokes) | grips the ankle-pitch servo, forks fore/aft onto the roll servo's discs |
| `foot_L`, `foot_R` | 1 + 1 | 52 | sole down | no | mirrored pair; roll servo drops into the cradle, two tabs with 2 × M2.5 flat-heads each |
| `sole_tpu_L`, `sole_tpu_R` | 1 + 1 | 23 | flat | no | **TPU 95A**, glued to the flat plate underside |


**With `ARMS=1`** (the get-up arms, opt-in — `docs/design-v6/shoulder-girdle.md`). The girdle *replaces* `neck_collar`; the rest are additions. Print support is **OrcaSlicer's job** wherever it can do it (Tom, 2026-09-24) — nothing below has modelled supports; the "supports" column is what the slicer must be told.

| part | qty | g each | orientation | supports | notes |
|---|---|---|---|---|---|
| `shoulder_girdle_v6` | 1 | 84 | base down on the deck face | **yes**, build plate only: trapezius-web window tops (34 mm spans), grip-plate rib-relief roofs, bay disc-relief tops | 200 × 58 × 33 mm — centre it on the bed. Both shoulder servos + the neck tube. 10 × M2.5×8 flat-head into deck pilots on the two edge rails; the pack goes in **before** the girdle |
| `arm_upper_v6_L`, `_R` | 1 + 1 | 28 | on its back, web face on bed (`RY_XUP`) | no | open-front U with two end walls, screw-access bore on the outboard face; grips the elbow servo |
| `arm_fore_v6_L`, `_R` | 1 + 1 | 42 | on its back, web face on bed (`RY_XUP`) | **yes**, build plate only: the two elbow-pad undersides (5 mm off the bed) | the elbow fork, an end wall; 12 mm hand knuckle |

Totals (default build, `parts_v6.py` rollup 2026-09-24): 15 PETG + 2 TPU prints, 11 unique STLs, ≈ 592 g PETG + 46 g TPU. Robot ≈ 1.82 kg with 6 × STS3250 + 7 × STS3215 (832 g), a 170 g pack and 180 g of boards and wiring. With `ARMS=1`: ≈ 2.23 kg (11 × STS3215; the girdle replaces `neck_collar`).

Fasteners (from each module's `SCREWS()`; the one-print hip yoke needs no M3×10 flange bolts or heat-set inserts): 24 + 12 M2.5×8 flat-head grip screws (leg links, ankle links), 8 M2.5×8 flat-head foot tabs, 8 M2.5 yaw stators + 4 neck stators + 4 M2.5 neck-collar flange, 8 M2.5 board standoffs + 4 pilot, 2 M2.5 power bosses, 4 M2.5 head face, 52 M3×6 horn + 28 M3×8 idler with washers, 4 M2 camera. Driver access was checked with a 7 mm cylinder on every screw; the exceptions (v5-inherited idler grip counterbores, a few standoff screws close to walls) are listed in the module reports and need a slim driver, not a redesign.

Gates on this set: `cad/v6/check_assembly_v6.py` (ROM interference, every relatively-moving pair, results in `docs/design-v6/cad_rom_check.txt`), the per-part printability audits (`audit_*.py`), and `cad/v6/animate_v6.py` (fly-in along the insertion paths, `cad/v6/renders/assembly_v6_flyin_strip.png`).
