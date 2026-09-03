# Print list — Bimo-like biped

**14 plastic prints (7 unique parts) + 2 silicone sole pads**, ~354 g of PETG.
Generated from `cad/parts.py` / `cad/dimensions.py`. Export STLs with
`../.venv/bin/python parts.py` (writes `cad/stl/*.stl`), which prints the
authoritative table of parts, quantities, bounding boxes and masses.

> **⚠️ pelvis v6 (2026-08-05) — three parts below are RETIRED.** The torso is
> now **one print**: `tower`, `imu_carrier`, the battery tray and the board
> frame were all folded into `pelvis`, which is why the COM dropped 30 mm and
> `sim/bimo_biped_v5body.xml` exists. **Do not print `tower` or `imu_carrier`**
> — their rows below are kept only for the build history, and `cad/stl/tower.stl`
> is a stale artifact that `parts.py` no longer produces. `gopro_base` is the
> only bolt-on left, and that one is deliberate (crash fuse).
> The per-part orientation and slicing guidance for the seven live parts is
> unchanged and still correct.

> **v3yaw (2026-07-23):** hip-yaw added — one STS3215 per leg, flat under the
> deck, vertical axis, horn down (see `docs/hip-yaw-study.md`). Two changes to
> the print queue: **`pelvis` is redesigned and must be reprinted** (the hanging
> roll bays are gone; it now carries two flat yaw-servo seats + a rigid,
> deck-bolted stator mount), and **`yaw_carrier` ×2 is a new part** that takes
> over the hip-roll bay geometry (same `BAY_BORE`/`BAY_WALL_DROP`/cheeks/U-slot
> — the roll servo, yokes, leg_link, shin, foot are all UNCHANGED and are NOT
> reprinted). Obsolete: **the old `pelvis` only** (its bays live on the carrier
> now). BOM adds **2× STS3215** (10 total) and a longer stator-screw set; the
> tower/deck heat-set pattern is preserved. Both parts carry new **connector
> openings** (corrected 2026-07-24 — the STS3215's two ports are on the
> idler-side face beside the disc, so the earlier chases aimed at nothing): a
> **deck hole** in the `pelvis` over each yaw servo's up-facing ports (plus a
> Ø21.5 disc/post pocket + 4× stator pads — the seat is not flat), and a
> **rear-wall window** in `yaw_carrier` at the measured trench band
> (`SV_CONN` in dimensions.py, from the vendor STEP) — see
> `docs/hip-yaw-study.md` §6 rev 3b. Idler-arm center reliefs in
> `yoke_roll`/`yoke_pitch`/`leg_link` deepened to through-bores for the servo's
> free-hub post (existing prints: only drill the Ø8 center deeper if the arm
> stands off the disc when bolting). `check_assembly.py` ALL CLEAR (yaw
> 0/±45° sweep + roll ±25° both proven against the carrier; inward-yaw gap
> between the two carriers 6.2 mm at ±45°), `check_printability.py` clean.
> Render: `renders/hip_yaw_beforeafter.png`.
>
> **2026-07-26 — the carrier's cable window needs a breakout, and the audit
> was blind to it.** `check_printability.py` measured that window's ceiling
> across its SHORT side (2.6 mm, the wall thickness) and passed it; that side
> is open on *both* faces, so nothing bridges across it and the real span is
> the 22.8 mm long way. The rule "a bridge fails across its short side" only
> holds when the short sides are anchored — now tested (`_sides_anchored`),
> and a window cut clean through a wall is re-read as a BEAM over its long
> span. The un-propped carrier fails that check at 23 mm; the audit now reads
> `yaw_carrier_print.stl`, whose breakout passes at 4.9 mm hops. No other
> part changed verdict.

> **Design-review fixes (2026-07-23, second pass):** three part corrections
> from user review of the exploded drawings + FreeCAD model:
> 1. **Idler bolt circle now drills THROUGH** on `leg_link`, `yoke_roll`,
>    `yoke_pitch` (was a modeling bug — the bore started at the disc face and
>    never reached the outer plate; the bolted-idler intent is original, see
>    the BOM's idler washers). Existing prints: **hand-drill Ø3.4 at the 4
>    BCD positions** (the blind recesses on the servo-facing side locate the
>    drill). Future prints are correct as exported. `IDLER_BOSS_D` grew
>    19→20 for web thickness — existing 19 mm-boss prints remain usable.
> 2. ~~leg_link cable notch~~ **REVERTED after user challenge** — the
>    cutout was based on a wrong routing assumption (the gripped servo's
>    lead uses the web window → back raceway, not the idler face), and a
>    full ankle-ROM sweep shows the ankle lead's connector clears the shin
>    fork by ~10 mm even at toes-pointed (now a permanent check_assembly
>    gate). The fork plate is solid again; `leg_link` needs **no reprint
>    for cable reasons** — only the item-1 idler holes, which hand-drill.
> 3. **`foot` gains top-side screw-head divots** over the low ankle-screw
>    row (driver path was blocked by the sole shelf). Hand-carving is
>    possible but ugly; **reprint recommended: `foot` ×2** (sole pad
>    adhesive area unchanged).
> `yoke_pitch` was already on hold for the −110° recut — its reprint picks
> up the idler through-holes automatically. Checks after all three:
> `check_printability.py` ALL PARTS PRINT CLEAN, `check_assembly.py` ALL
> CLEAR. Renders: `renders/yoke_roll_idler.png`,
> `renders/leg_link_idler.png`, `renders/foot_divot.png` +
> `_driver.png`.

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
  the PETG density (`dimensions.py::FILAMENT_RHO = 1.27e-3`): as of pelvis v6,
  printed plastic **~354 g**, total robot **~1078 g** (+154 g GoPro = 1232 g).
  Run `parts.py` for the live numbers; the sim inertia builder
  (`sim/build_v2_inertia.py`) is regenerated from the same solids.

## Parts to print

| Part | Copies | Material | Infill | Orientation | Status |
|---|---|---|---|---|---|
| `pelvis` | 1 | PETG | 30–40 % | upside-down, deck top on bed | ♻️ **v6: THE WHOLE TORSO IS THIS PART** (tower + imu_carrier + battery tray + board frame folded in; board recess and battery V-seat bay are printed into it; ~30 mm lower COM). Previously — **redesigned for v3yaw** (roll bays → two flat yaw-servo seats + deck-bolted stator mount; deck/tower pattern kept; grows ~15 mm rearward for the case overhang → bbox 61 × 104 × 9). ~32 g |
| `yaw_carrier` | 2 | PETG | 30–40 % | horn-plate face on bed, bay walls rise — **slice `yaw_carrier_print.stl`** | 🆕 **new part (v3yaw)** — bolts to the yaw horn, carries the (unchanged) hip-roll bay. Prints like the old pelvis bay (walls vertical, U-slot upward-open, teardropped case screws) with one addition: the rear-wall **cable window needs a breakout** (2026-07-26). Printed horn-plate-down its ceiling is the 1.0 mm bar between the window and the bore crown — 2.6 × 22.8 mm of bare bridge with the U-slot void directly above, so no infill and no next layer to iron it flat. **Three break-away columns** split it into four 4.95 mm hops; each stands on the window sill and meets the bar through a 1.0 mm neck (body inset 0.2 mm from both wall faces for blade access). Snip/twist them out and trim the nubs flush enough to clear the plug bodies — nothing seats on that bar. Figure: `renders/yaw_carrier_breakout.png`. ~17.5 g each |
| `yoke_roll` | 2 | PETG | 30–40 % | **WALL: on edge, arms along the bed — slice `yoke_roll.stl` with SLICER SUPPORTS ON** (see below). There is no `_print` variant any more | ✅ ready — *secondary* hip-angle check |
| `yoke_pitch` | 2 | PETG | 30–40 % | **WALL: on its back like `leg_link` — slice `yoke_pitch.stl` with SLICER SUPPORTS ON + a brim** (see below). No `_print` variant any more | ♻️ **revised for hip −110°** (idler arm → hub + riser; HOLD lifted) |
| `leg_link` | 4 | PETG | 30–40 % | on its back, web face on bed — **slice `leg_link_print.stl`** | ♻️ **revised v3** (2026-07-16 print review, two passes: 9×11 **cable window** through the web — before it every joint-crossing cable pierced the plastic — zip-tie holes at ±9 straddling it; idler boss OD tapered 51°; and the fork slabs are now **solid to the web face** wherever the foot sweep allows (mapped at ankle ±45°: horn side fully; idler side except the corner-sweep lobe at z −68.4…−55.6, whose 16 mm gap is broken up by **two island posts** into 2/2.5/5.5 mm bridge hops). **There are no fins at all** — the only break-away pieces are two 4 mm pad stubs at the fork tips plus those two posts, all verified as SEPARATE first-layer islands (≥1 mm clear, attached to nothing — they lift off with a fingernail). Slice preview: `renders/leg_link_print_slice.png`. *The window is functional: reprint v1/v2 links when convenient* |
| `foot` | 2 | PETG | 30–40 % | sole down | ♻️ **revised v3.1** (v3 heel bulkhead + sole enlarged 100 → 116 for the get-up corridor) |
| ~~`tower`~~ **RETIRED (pelvis v6)** | — | — | — | — | ♻️ **revised for print, support-free** (2026-07-16 slice reviews: feet-tab gussets and battery-rail stubs are now true ≥45° wedges — the old stepped boxes left flat 6 mm ceilings drooping over the interior; the **window sill was deleted** — it printed as a 70 mm member 41 mm up in mid-air, and the hook-loop belt is the real battery retention; two 45° corner detents park the pack instead. Feet screws now seat on the tabs through Ø6.6 wells — the only remaining bridges. ~5h19m PETG) |
| `gopro_base` | 1 | PETG | — | base down, prongs up | ✅ printed in PETG |
| ~~`imu_carrier`~~ **RETIRED (pelvis v6)** | — | — | — | — | 🆕 **new part (2026-07-16)** — BNO055 carrier between tower top and gopro_base (same 4 screws → M3×12); bosses on the board's true 21.59 × 15.24 hole pattern; ~5 g, ~30 min |
| Sole pad | 2 | 1/16" self-adhesive silicone sheet ([B0FJ8TBMQK](https://www.amazon.com/dp/B0FJ8TBMQK?tag=tommwhipple-20), 2× 6"×6") | — | cut 106 × 46 mm, stick onto flat sole (one sheet yields both + a spare strip) | 🛒 ordered |

### The two yokes: turn supports on in OrcaSlicer (2026-07-30)

These are the **only** two parts in the list that need slicer supports;
everything else prints support-free or carries its own break-away pieces. Put
them on their own plate so the setting does not leak onto anything else.

In OrcaSlicer, with `yoke_roll.stl` / `yoke_pitch.stl` on the plate:

| Tab | Setting | Value |
|---|---|---|
| Support | Enable support | **on** |
| Support | **Support on build plate only** | **on** — see below, this one matters |
| Support | Type | `normal(auto)` — flat plate undersides peel off it cleanly; `tree(auto)` also works and is gentler on the round pads |
| Support | Threshold angle | 30° (default) — the overhangs here are fully horizontal, so anything sane catches them |
| Support | Top Z distance | **0.2 mm** (one layer) — PETG welds to supports at 0.1 |
| Support | Support/object XY distance | 0.35 mm (default) |
| Others → Brim | Brim type / width | `outer only`, **5 mm** — `yoke_pitch` only |

**"Support on build plate only" is not optional here.** In this orientation
*every* bore in both yokes runs horizontally — the flange screw holes, the bolt
circles, the centre reliefs — so plain auto-support packs each one solid
(user, 2026-07-30: *"the support setting is now supporting a bunch of holes that
don't need it"*). The setting separates the two cases exactly, because a support
inside a bore has to stand **on the part**, while the overhangs that matter have
a clear run down to the bed:

| | kept | dropped |
|---|---|---|
| `yoke_roll` | 208 mm² — all four real clusters (48, 48, 78, 24) | 210 mm² of bore interiors |
| `yoke_pitch` | 225 mm² — all four (51, 63, 78, 24) | 162 mm² of bore interiors |

Dropping them is safe: the widest span then left unsupported is **5.24 mm**
(roll) / **3.60 mm** (pitch), against `check_printability`'s 8.0 mm PETG
`BRIDGE_OK`. Those bore tops were never failures — the audit's only findings on
these parts are the four ISLANDs and the CONTACT floor. No teardropping needed.

Why `yoke_pitch` wants the brim and `yoke_roll` mostly does not: pitch stands
32 mm tall on the flange edge, a footprint of 175 mm² that is 44 × 4 mm — tall,
narrow and tippy. Roll lands 194 mm² and is only 30 mm tall.

What the supports are actually for (from `check_printability`, which now reports
these as `[support]` rather than failing): four floating clusters per part — the
two arm plates, and the pad/boss rims that start in mid-air.

**These used to be modelled into the STL and it was a mistake.** The fins put
their anchor tabs 0.3 mm *below* the bed plane, so the tabs became the lowest
geometry and the flange — the actual bed adhesion — floated: the real first
layer was 26 mm² (roll) / 36 mm² (pitch) of disconnected stamps, 14 % and 21 %
of the flange's own footprint. The flared feet added *for* bed area touched
nothing. On top of that, every foot splayed 2.00 mm past the part silhouette,
the pad-rim fins were flat blocks under a *cylindrical* pad (contact on one
tangent line, gap opening to 3.20 mm), and each tab fused 0.3 mm into the part
across 2 × 3 mm — a weld, not a break-away contact.

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
| `pelvis` (v3yaw reprint) | 1 | `yoke_pitch` | 2 |
| `yaw_carrier` (v3yaw new) | 2 | `leg_link` (thighs) | 2 |
| `yoke_roll` | 2 | | |
| `leg_link` (shins) | 2 | | |
| `foot` | 2 | | |
| `tower` | 1 | | |
| `gopro_base` | 1 | | |
| TPU foot pad | 2 | | |

Re-run `check_assembly.py` at the new hip range and refresh this table once
`yoke_pitch` is re-cut.
