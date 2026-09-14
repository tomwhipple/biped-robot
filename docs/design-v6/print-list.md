# v6 print list (2026-09-14)

Generated from `cad/v6/parts_v6.py` (`docs/design-v6/parts_v6_rollup.txt`). PETG 0.4 mm nozzle, 3 perimeters, 30–40 % infill unless noted; every part fits the 220 mm bed. Regenerate STL/STEP with `.venv/bin/python cad/v6/parts_v6.py`; each part module's `__main__` runs its own audits.

| part | qty | g each | orientation | supports | notes |
|---|---|---|---|---|---|
| `pelvis_v7` | 1 | 174 | deck-top-down (v5 pelvis orientation) | **yes**: the two yaw-cell ceilings (the case fills the cell to 0.3 mm, no support-free geometry exists) and the skin window roofs near the housing floor | one print. Boards slide down their deck slots; pack through the deck aperture; neck servo stands in the deck-top well, screwed from below through the aperture. Residual audit findings listed in `pelvis_v7.py`'s docstring — check the sliced preview at the window roofs and the deck-slot edges before printing |
| `neck_collar` | 1 | 20 | flange down, flat | no | slides down over the neck servo standing in the deck well (2.1 mm to the head's horn disc), 4 × M2.5 flat-heads at the flange corners into pilot holes in the deck (`NECK_FLANGE_HOLE_XY`, shared with `pelvis_v7.py`); the servo's own stator screws go up through the deck aperture from below. Split out of the pelvis on 2026-09-14: fused in, its mouth was the print's low point and the whole deck became an island |
| `head_shell` + `head_face` | 1 + 1 | 27 + 7 | shell crown-up (domed top ≥ 45°), face plate flat | no | 4 × M3×6 onto the neck horn from inside before the face goes on; Camera Module 3 on 4 × M2 bosses inside the face; ribbon slot in the base beside the axis. One dome-seam CEILING flag in the audit is believed to be a boolean artifact — verify in the slicer |
| `yaw_carrier` | 2 | 18 | as v5 (`yaw_carrier_print.stl` from `cad/parts.py`) | v5 fins | unchanged v5 part |
| `yoke_roll` | 2 | 12 | as v5, on edge | slicer supports (v5) | unchanged v5 part |
| `yoke_pitch` | 2 | 11 | as v5, on its back + brim | slicer supports (v5) | unchanged v5 part |
| `leg_link_v6` | 4 | 27 | **standing on the lower fork end** (new: the box's front plate prints as a wall) | brim recommended (round pads meet the bed on a line) | thigh and shin are the same part; 110 mm; closed box section; one 25 mm end-anchored rib at the v5 jog block prints as a short bridge |
| `ankle_link` | 2 | 12 | web-down like v5 (`RY_XUP`) | **yes** (CEILING/ISLAND class, same as the yokes) | grips the ankle-pitch servo, forks fore/aft onto the roll servo's discs |
| `foot_L`, `foot_R` | 1 + 1 | 52 | sole down | no | mirrored pair; roll servo drops into the cradle, two tabs with 2 × M2.5 flat-heads each |
| `sole_tpu_L`, `sole_tpu_R` | 1 + 1 | 23 | flat | no | **TPU 95A**, glued to the flat plate underside |

Totals: 16 prints, 13 unique STLs, ≈ 591 g PETG + 46 g TPU. Robot ≈ 1.77 kg with 6 × STS3250 + 7 × STS3215 (832 g), a 170 g pack and 180 g of boards and wiring.

Fasteners (from each module's `SCREWS()`): 24 + 12 M2.5×8 flat-head grip screws (leg links, ankle links), 8 M2.5×8 flat-head foot tabs, 8 M2.5 yaw stators + 4 neck stators + 4 M2.5 neck-collar flange, 8 M2.5 board standoffs + 4 pilot, 2 M2.5 power bosses, 4 M2.5 head face, 52 M3×6 horn + 28 M3×8 idler with washers, 4 M2 camera. Driver access was checked with a 7 mm cylinder on every screw; the exceptions (v5-inherited idler grip counterbores, a few standoff screws close to walls) are listed in the module reports and need a slim driver, not a redesign.

Gates on this set: `cad/v6/check_assembly_v6.py` (ROM interference, every relatively-moving pair, results in `docs/design-v6/cad_rom_check.txt`), the per-part printability audits (`audit_*.py`), and `cad/v6/animate_v6.py` (fly-in along the insertion paths, `cad/v6/renders/assembly_v6_flyin_strip.png`).
