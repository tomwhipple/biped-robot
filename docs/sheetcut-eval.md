# Sheet-cut components evaluation — SendCutSend (plastics first)

**Question:** should any of the biped's printed PETG parts become sheet-cut
parts from [sendcutsend.com](https://sendcutsend.com) — primarily **plastic
sheet** (CNC-routed acetal/Delrin, polycarbonate, HDPE, UHMW, G10/FR4), with
metal as the fallback comparison?

**Short answer:** almost everything stays printed. Plastic sheet earns exactly
one slot: a **routed G10/FR4 sole plate** for the foot, and only if the pending
sole-enlargement decision lands — it is isotropic (kills the layer-line/sag
failure class), 1.7× stiffer than the 6 mm printed sole at *lower* mass, and
bonds adhesives well. The heel-tab reinforcement story does **not** translate
to plastic: the only bendable plastic (polycarbonate) starts at 3.0 mm, which
physically does not fit the locked 2.4 mm tab envelope — bent brackets remain
a metal-only option. The printed foot v3 (heel bulkhead, on main as `4c39a7c`)
stays the first line of defense either way.

## Status update (2026-07-15, evening)

The trigger named below **has fired**: the sole enlargement landed on main
(`6834fc0` — 116 × 52 heel-biased, with the hip −110° yoke revision). The
G10/FR4 0.125" sole order is therefore live as an *option*: cut to the new
116 × 52 outline, ~43 g each (vs ~42 g printed PETG at 6 mm). Note the
current build decision is printed-PETG soles + 1/16" self-adhesive silicone
pads (`b4f22bc`), so G10 remains an upgrade path, not a blocker. The yoke
flange "mid-redesign" caveat in the table also resolved: the hip revision
changed only the idler arm, not the flange — the yoke stays printed.

## Decision table

| Part | Demonstrated problem | Best sheet option | Verdict | Why (one line) |
|---|---|---|---|---|
| `foot` sole (6 mm PETG; v1 print-sag fixed in v2) | print defect, fixed | **G10/FR4 0.125" (3.18 mm), routed**, ~30 g plain / ~41 g enlarged | **Order only if soles are enlarged** | 1.7× stiffer than 6 mm printed PETG at −8 g, isotropic, dead flat, glues well; PC 0.220" the tough-but-floppier runner-up |
| `foot` heel tabs (2.4 mm PETG, snapped across layers) | **yes** | bent PC: **doesn't fit** (min bendable 3.0 mm > 2.4 mm locked envelope, ±5° bends); flat doublers: no room | **Printed v3 first** | v3 bulkhead (`4c39a7c`) closes each blade into a channel; if tabs still fail, the only sheet fix thin enough is 1.6 mm 5052 (metal §5) |
| `pelvis` deck (104×46×5) | no | G10 0.125" plate ≈ 27 g | **Stays printed** | zero mass win (printed deck ≈ 27 g); integral hanging servo bays would become ~16 fasteners |
| `tower` top plate (96×42×3.5) | no | flat routed plate | **Stays printed** | integral standoffs/bosses/vents; crash fuse is upstream (`gopro_base`) |
| yoke flanges (4 mm, M3 20 mm square) | no (mid-redesign) | flat G10 + printed bosses | **Stays printed — revisit** | needs formed Ø16.5/Ø19 bosses; hip-angle redesign re-cuts it anyway |
| `gopro_base`, `leg_link`, tower body | fuse / 3D geometry | — | **Stays printed** | no flat decomposition wins |

**Bottom line:** no order today. One trigger justifies a plastic order — the
sole-enlargement recommendation from the get-up study is accepted — then order
**2× enlarged G10 sole plates (~$40–60 est., ~1 week)** and bolt the printed
v3-style cradle on top. A second, independent trigger — v3 tabs snapping
again — is answerable only in metal (bent 1.6 mm 5052 L-brackets, §5).

---

## 1. SendCutSend plastics capability (researched 2026-07-15)

Sources: [plastics catalog](https://sendcutsend.com/materials/plastics/),
[CNC routing guidelines](https://sendcutsend.com/guidelines/cnc-routing/),
[polycarbonate page](https://sendcutsend.com/materials/polycarbonate/),
[CNC bending](https://sendcutsend.com/services/cnc-bending/),
[tapping](https://sendcutsend.com/services/tapping/),
[processing times](https://sendcutsend.com/sendcutsend-processing-times/).

- **Process: plastics and composites are CNC-routed, not laser-cut** (confirmed;
  PC explicitly "does not respond well to laser cutting"). Routed tolerance is
  the same ±0.005" (±0.13 mm) as laser.
- **Materials / thicknesses (relevant subset):**
  - **Delrin (acetal)**: 0.125" (3.18 mm), 0.270" (6.86 mm)
  - **Polycarbonate**: 0.118" (3.0), 0.177" (4.5), 0.220" (5.6 mm)
  - **ABS**: 0.125"–0.234"; **HDPE**: 0.250"+; **UHMW**: 0.375"/0.500"
    (bearing-grade, too thick/floppy for our plates)
  - **G10/FR4** (glass-epoxy composite): 0.063"–0.375", five thicknesses
  - **Carbon fiber**: 0.040"–0.197" (premium option)
  - acrylic (brittle), Mylar, PP film, foams: not relevant
- **Routing design rules:** min hole Ø0.125" (3.18 mm — the Ø3.4 M3 clearance
  holes clear this, barely; Ø2.8 pilot holes do NOT, so no self-tap pilots in
  routed parts); min internal corner radius 1.6 mm (use dogbone fillets where
  square inside corners must mate); min part 1" × 2"; ≥0.4" between edge nodes
  (fixture tabs leave small nubs to dress off); material removal < 50 %.
- **Bending: polycarbonate is the ONE bendable plastic** (all three
  thicknesses, cold air-bent). Rules are much looser than metal: min inside
  bend radius ≈ 1× thickness, **U/C-channels need base ≥ 3× flange** (vs 2:1
  for metal), bend relief notches mandatory (PC cracks without them), visible
  springback compensated at the brake, frosted bend lines, and **bend-angle
  tolerance ±5°** (≤24") / ±7°. Delrin, HDPE, G10, etc. cannot be bent.
- **Tapping:** available in ABS, Delrin, HDPE, PC (M3×0.5 offered) — but
  plastic threads are explicitly weak; **G10 is not in the tapping list** →
  through-bolt it. PEM hardware insertion is metals-only.
- **Cost / minimum / lead time:** "from as low as $1 per part"; **$39 order
  minimum**, free US shipping. Routed plastic palm-sized parts typically land
  ~$8–20 ea at qty 1–4 (estimate — instant quote before committing). 2–4
  business days production (+1–2 for bending/tapping), free shipping →
  **~1 week to the door**; paid rush to 24 h.

## 2. Foot — the real candidate

Mass/stiffness basis (verified against `cad/dimensions.py` @ main `4c39a7c`):
sole 100 × 52 mm (enlarged scenario ~120 × 60), printed PETG ≈ 1.27 g/cm³ ×
0.90 print factor; plate bending rigidity ∝ E·t³ per unit width.

### 2.1 Sole plate — flat routed plastic works, and G10 wins

The v1 sole failure (46 mm bridged recess sagging) was a *print* defect, fixed
in v2 by the flat sole. So a sheet sole is not fixing a live failure — its
case rests on the **enlargement scenario** (get-up rise corridor is ±20 mm CoP
on a 90–100 mm foot; enlarged pads are the pending day-5 recommendation):

| Sole material (thickness) | E (GPa) | Stiffness vs 6 mm printed PETG (E·t³) | Mass 100×52 | Mass ~120×60 | Adhesion (self-adhesive rubber pad) | Notes |
|---|---|---|---|---|---|---|
| Printed PETG 6.0 mm | ~2.1 | 1.00× | 35.6 g | 49 g | good | baseline; anisotropic, 6 mm slab edge |
| **G10/FR4 3.18 mm (0.125")** | ~24 | **1.70×** | **29.7 g** | **41 g** | **excellent** (epoxy-glass, high surface energy) | stiffest per gram here; through-bolt only (no tapping) |
| Polycarbonate 5.59 mm (0.220") | ~2.3 | 0.89× | 34.9 g | 48 g | very good | toughest (impact ≫ printed PETG); tappable (weak threads) |
| Polycarbonate 4.50 mm (0.177") | ~2.3 | 0.46× | 28.1 g | 39 g | very good | too floppy as a sole |
| Delrin 6.86 mm (0.270") | ~3.1 | 2.20× | 50.3 g | 70 g | **poor** (low-surface-energy acetal — mechanical fastening only) | heaviest; glue-hostile kills it for a glued pad |
| Delrin 3.18 mm (0.125") | ~3.1 | 0.22× | 23.3 g | 32 g | poor | far too floppy |
| Carbon fiber ~3.0 mm | ~50 | ~3.0× | 24 g | 34 g | good (scuff + epoxy) | premium: est. $40–80/plate; overkill |
| (5052 aluminum 2.03 mm, §5) | 69 | 1.27× | 28.3 g | 39 g | good | thinnest edge; conductive, dents |

**Pick: G10/FR4 0.125".** Stiffer than everything except Delrin-0.270"/CF at
the lowest practical mass, isotropic in-plane, bonds adhesives well (the
self-adhesive rubber pad sticks poorly to acetal — that alone eliminates
Delrin for a glued-pad sole; polycarbonate and G10 both take adhesive
properly). PC 0.220" is the runner-up if maximum impact toughness is wanted
and 0.89× stiffness at +7 g is acceptable.

**Integration with the printed v3 cradle:** the sheet replaces only the flat
slab. The v3 heel structure (tabs + bulkhead + cable window + aft wedge +
front stop, per `4c39a7c`) becomes a printed **cradle** bolted to the plate:
give it a 1.5–2 mm base flange, 4–6× M3 countersunk from below (flat heads
sit flush in a routed countersink or a simple chamfer; nylocs on top; the
glued pad covers the heads). Printed cradle ≈ 10–12 g PETG.

- **Ankle height:** printed pocket floor is at z = 4.0 (ankle axis 16.36 above
  sole). Servo sits directly on a G10 plate at 3.18 → make the cradle floor
  under the servo 0.8 mm proud and nothing else changes. (PC 0.220" would
  *raise* the ankle 1.6 mm — G10's shim-able direction is another point for
  it.) Relief slots under the shin-fork pads (to z = 3.5) become unnecessary:
  a 3.18 mm plate top is already below 3.5.
- **Mass accounting (enlarged, per foot):** G10 41 g + cradle ~11 g ≈ **52 g
  vs ~55 g for an all-printed enlarged v4** — mass-neutral-to-better, with a
  1.7× stiffer, isotropic, 3 mm-thin sole and ~6 mm lower robot CoM per the
  same accounting as §5. Update `sim/build_v2_inertia.py` either way.

### 2.2 Heel tabs — the plastic story dies on the locked envelope

The snapped tab is the one demonstrated structural failure (layer-line
fracture under a lateral knock). Checked honestly against plastic sheet:

- **Bent polycarbonate brackets: geometrically impossible.** PC *is* CNC-
  bendable at SendCutSend (the one plastic that is), and on paper a bent PC
  L-bracket is attractive — isotropic, far tougher than printed PETG, glues
  well, one-material foot with the PC sole. But the thinnest bendable PC is
  **0.118" = 3.0 mm**, and the tab envelope is **LOCKED at 2.4 mm** (shin-fork
  horn arm sweeps at y = 20.45; tab inner face must sit at the servo case
  face, 17.65). A 3.0 mm leg puts its outer face at 20.65 — **0.2 mm inside
  the fork sweep**. It does not fit, full stop. Even if it did: PC bend-angle
  tolerance is **±5°**, which is ±2.4 mm of hole position at the top of a
  27 mm leg — swamps the M3-in-Ø3.4 clearance — and the 3:1 base:flange rule
  plus mandatory bend reliefs make a small U-channel version illegal anyway.
- **Flat routed doublers: no room.** Outward of the tab is the locked 0.4 mm
  fork clearance; inward is the servo case. There is no lamination plane
  available, so "screw a Delrin/G10 stiffener to the tab" has nowhere to live.
- **What actually translates:** nothing in plastic. The only sheet part thin
  enough AND bendable to precision is metal — 1.6 mm 5052 L-brackets (outer
  face 19.25, 1.2 mm fork clearance, ±1° bends). That option is retained in
  §5 as the fallback.

**Verdict: printed v3 first, unchanged.** The v3 bulkhead (`4c39a7c`) closes
each free blade into a channel section — stiff in both directions, unloads
the layer-bond root, and directly addresses the lateral-knock mode that broke
v2. It costs nothing. Order metal brackets only if a v3 tab fails again.

## 3. Pelvis deck and tower top under plastic-sheet assumptions

Re-checked with routed-plastic numbers; the conclusions survive:

- **Pelvis deck** (104 × 46 × 5, printed region ≈ 27.3 g): a G10 0.125" deck
  is 27.4 g — **zero mass win** — and a PC 0.220" deck is 32 g. The deck's
  value is the *integral* hanging servo bays (41 mm walls, U-slot bores, 8×
  M3 each, tower heat-set bosses); a flat routed deck turns one print into a
  plate + two printed bay blocks + ~16 fasteners + stack-up, to fix nothing.
  If deck flex ever appears, a bonded 1.6 mm G10 doubler (~14 g) is the
  retrofit. **Stays printed.**
- **Tower top plate** (96 × 42 × 3.5 ≈ 16 g): integral board standoffs, GoPro
  bosses, driver-access holes, vents; prints as the tower's bed face. A G10
  0.093" (2.4 mm) plate is ~17 g before re-attaching all of those features
  with fasteners. Loads are already gusset-sized, and the sacrificial part in
  a crash is deliberately `gopro_base` upstream. **Stays printed.**
- **Yoke flanges** (4 mm, M3 on 20 mm square): flat G10 would carry the bolt
  pattern fine, but the horn/idler interfaces need the Ø16.5 × 1.0 and
  Ø19 × 1.2 formed bosses (routing can't add bosses; min routed thickness
  steps don't help), and `yoke_pitch` is mid-redesign for the −110° hip
  target. **Stays printed; revisit only if fall training snaps a yoke** — at
  which point a G10 flange laminated to printed boss caps, or the metal
  clevis of §5, compete.

## 4. Recommendation (plastics-first)

1. **Do not order anything yet.** Foot v3 (heel bulkhead) is on main
   (`4c39a7c`), printable today, and attacks the only demonstrated structural
   failure for free.
2. **Decide the sole enlargement** (get-up ±20 mm corridor). That decision is
   the only live trigger for a sheet order:
   - **Accepted →** order **2× enlarged sole plates in G10/FR4 0.125"**
     (~120 × 60, corner radii ≥ 1.6 mm, 4–6 countersunk M3 through-holes
     Ø3.4, no relief slots needed). Estimated **$40–60 shipped, ~1 week**
     (the $39 minimum is roughly the order). Print the v3-derived cradle
     (tabs + bulkhead + front stop on a 2 mm flange, +0.8 mm servo shim)
     and through-bolt it. Runner-up material: PC 0.220" if impact toughness
     trumps stiffness (+7 g/foot, ankle +1.6 mm).
   - **Rejected →** stay all-printed; there is no plastic-sheet fix for the
     tabs (§2.2), so the fallback for a repeat tab failure is the metal
     bracket order in §5 (~$40–60).
3. **Bookkeeping if sheet ships:** update foot segment mass in
   `sim/build_v2_inertia.py`, re-run `check_assembly.py` with the cradle
   geometry, and respect routing rules in the flat pattern (Ø ≥ 3.18 mm
   holes only, 1.6 mm inside corners, edge-node spacing, expect small
   fixture-tab nubs to dress).
4. **What stays printed, permanently (at this scale):** pelvis, tower, leg
   links, gopro_base, yokes pending the hip-redesign shakeout. PETG has
   failed structurally exactly once, and that geometry (thin tall tab across
   layer lines) is precisely the one thing plastic sheet *cannot* replace.

---

## 5. Secondary: the aluminum option (kept for comparison)

Metal remains the only route to a **bent** bracket thin enough for the locked
heel envelope, and it is the fallback if printed v3 tabs fail again.
Capability summary (verified 2026-07-15): 5052-H32 in 1.02–12.7 mm, laser
±0.13 mm, CNC bending with fixed per-thickness radius/K-factor (±1° ≤ 24"
bends; U-channels need base ≥ 2× flange — which is why the heel bracket is
two L's, not one U), M3 tapping in ≥ 1.5 mm, PEM hardware, same $39
minimum / ~1 week. Also stocked: 304/316 stainless (ballast-grade density),
0.38 mm 1075 spring steel (future compliant toe/heel leaves), Grade 5
titanium (2–4× Al cost; unneeded at these loads).

- **Heel L-brackets (the metal-only part):** 2 per foot, 1.6 mm 5052, one 90°
  bend each. Vertical leg ~14 × 27 mm matching the tab band (x = −40…−26),
  M3 holes at the case rows (x = −29.0 horn side / −32.75 idler side; heights
  2.11 and 22.61 above the servo seat); base flange ~14 × 12 mm bolted to the
  sole (countersunk M3 + nyloc/PEM). **Envelope: outer face 19.25 → 1.2 mm
  fork clearance (3× the printed tab's 0.4 mm).** Strength ∝ σ·t²:
  5052-H32 193 MPa × 1.6² ≈ 494 vs printed-across-layers ~25 MPa × 2.4² ≈ 144
  → **~3.4× stronger**, and it yields instead of snapping. ~2.5 g each.
  Est. $40–60 for four with the order minimum.
- **Aluminum sole (vs the G10 pick):** 0.080" (2.03 mm) 5052, 28.3 g plain /
  39.2 g enlarged, 1.27× printed stiffness — G10 0.125" beats it on stiffness
  (1.70×) at similar mass and bonds better; aluminum wins only on edge
  thinness (2 mm) and if you're already ordering metal brackets and want one
  material/process. A 304 stainless sole (~86 g enlarged) is deliberate
  ballast only — lowers CoM harder but costs ~+106 g robot mass.
- **CoM note (either material):** like-for-like foot swaps add ~0–20 g at
  z ≈ 0 on a ~280 mm-CoM, ~860 g robot → CoM drops up to ~6 mm; small but the
  correct direction for the rise corridor.
- Pelvis/tower/yokes in metal: same verdicts as §3 (no mass win, integral
  features, fastener bloat) — see git history of this file for the full
  metal-first analysis if wanted.
