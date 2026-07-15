# Sheet-cut components evaluation — SendCutSend

**Question:** should any of the biped's printed PETG parts become laser-cut /
CNC-bent sheet parts from [sendcutsend.com](https://sendcutsend.com)?

**Short answer:** almost everything stays printed. The one part where sheet
metal genuinely attacks a demonstrated failure mode is the **foot** — the heel
retention tab snapped across print layer lines, and a bent 5052 bracket is
isotropic, ~3× stronger in tab bending, and *thinner* than the locked 2.4 mm
envelope. But the printed foot v3 (heel-bulkhead U-channel) being designed now
is free and same-day; order metal only if v3 fails, **or** fold it into the
pending sole-enlargement decision, where a 2 mm aluminum sole costs no mass
versus an enlarged printed sole and lowers the CoM ~6 mm as a bonus.

## Decision table

| Part | Demonstrated problem | Sheet-cut candidate | Verdict | Why (one line) |
|---|---|---|---|---|
| `foot` heel tabs (2.4 mm PETG, snapped) | **Yes** — layer-line fracture | 2× 1.6 mm 5052 bent L-brackets, ~$25–35/pair est. | **Conditional order** | ~3.4× tab bending strength, ductile, 1.2 mm fork clearance (vs 0.4); but printed v3 bulkhead is free — try it first |
| `foot` sole (6 mm PETG, sag was a *print* defect, fixed) | No (fixed by flat sole) | 2 mm (0.080") 5052 plate, ~28 g plain / ~39 g enlarged | **Order only if soles are enlarged** | at enlarged size, metal ≈ printed mass, stiffer, −6 mm robot CoM, widens the ±20 mm get-up corridor |
| `pelvis` deck (104×46×5) | No | 2 mm 5052 plate, ~26 g | **Stays printed** | no mass win (printed deck ≈ 27 g); hanging servo bays are integral — metal adds ~16 fasteners for nothing |
| `tower` top plate (96×42×3.5) | No | flat plate | **Stays printed** | standoffs, GoPro bosses, vents are integral; loads are trivial |
| yoke flanges (4 mm, M3 20 mm square) | No (redesign in flux) | 2–3 mm 5052 | **Stays printed — revisit** | needs Ø16.5/Ø19 bosses sheet can't make; hip-angle redesign will re-cut it anyway; reconsider only if yokes snap in fall training |
| `gopro_base` | Crash fuse by design | — | **Stays printed** | it is *supposed* to break before the tower does |
| `leg_link`, `tower` body | No | — | **Stays printed** | fully 3D geometry, no flat decomposition wins |

**Bottom line:** no order today. Two triggers would justify one (~$50–80 est.
total, ~1 week to door): (1) printed foot v3 tabs fail again, or (2) the
sole-enlargement recommendation from the get-up study is accepted — then order
enlarged 5052 soles *and* the heel brackets together in one order.

---

## 1. SendCutSend capability summary (researched 2026-07-15)

Sources: [materials catalog](https://sendcutsend.com/materials/),
[5052 page](https://sendcutsend.com/materials/5052-aluminum/),
[CNC bending](https://sendcutsend.com/services/cnc-bending/),
[tapping](https://sendcutsend.com/services/tapping/),
[hardware](https://sendcutsend.com/services/hardware/),
[processing times](https://sendcutsend.com/sendcutsend-processing-times/).

- **Materials / thicknesses (relevant subset):**
  - **5052-H32 aluminum** (the workhorse): 0.040" / 0.063" / 0.080" / 0.090" /
    0.100" / 0.125" … 0.500" (1.02–12.7 mm). Excellent formability, bendable
    at every listed gauge; min inside bend radius roughly 1× thickness.
  - **304 stainless**: 0.030"–0.500" (0.76–12.7 mm). 3× the density of Al —
    only interesting as deliberate ballast.
  - **1075 blue-temper spring steel**: 0.015" (0.38 mm) only, laser cut, not
    bent. (Future: compliant toe/heel leaf springs, clips.)
  - **Grade 5 titanium**: 0.040"–0.250". 2–4× the cost of aluminum; no need at
    our loads.
- **Laser tolerance:** ±0.005" (±0.13 mm) on parts under 12". M3 clearance
  holes (Ø3.4) and the 20 mm bolt square are trivially within spec. Min part
  size 0.25" × 0.375"; even the smallest bracket here is fine.
- **CNC bending:** bend radius + K-factor are *fixed per material/thickness*
  (published in their bending calculator — pull the exact 0.063"/0.080" 5052
  numbers into CAD before ordering, they were not on the public page). Angle
  tolerance ±1° (bends < 24"), bend-to-edge ±0.015" per bend. **Key design
  rule:** U-channels need base ≥ 2× flange length — this *rules out a single
  U heel bracket* (38.5 mm web vs ~27 mm legs) and is why the foot proposal
  below uses two L-brackets instead. Minimum flat flange length applies
  (~9–12 mm range for thin gauges; verify per thickness).
- **Threads / hardware:** tapping M2–M10 (M3×0.5 available) in ≥ 0.059"
  material; PEM press-fit nuts/standoffs/studs in metric sizes for 5052 from
  0.030". Countersinking offered. Each post-process adds +1–2 days.
- **Minimum order / cost:** ~$39 order minimum, free standard US shipping.
  Their published palm-sized example: 0.100" 5052 part, cut + tapped, **$12.09
  qty 1** → $4.40 qty 100. Small bent brackets typically land ~$10–20 ea at
  qty 1–4 (estimate — get an instant quote; prices here are indicative only).
- **Lead time (US):** 2–4 business days production, +1–2 for bending/tapping/
  hardware, free shipping → **roughly a week to the door**; paid rush options
  down to 24 h.

## 2. Per-part analysis

Mass basis: 5052 = 2.68 g/cm³ (1.6 mm sheet = 4.3 mg/mm², 2.03 mm =
5.4 mg/mm²); printed PETG ≈ 1.27 g/cm³ × 0.90 print factor = 1.14 mg/mm³.
Part dims verified against `cad/dimensions.py` / `cad/parts.py` @ `6080d12`.

### 2.1 Foot — the real candidate

**Failure history (both on foot v1):**
1. Sole underside: 46 mm bridged pad recess sagged in PETG. *Print* failure,
   already fixed in v2 by a flat sole + glue-on rubber pad. Metal does not
   earn credit for this one.
2. Heel retention tab (2.4 mm thick × 26 mm tall, vertical, printed sole-down)
   snapped **across layer lines** in handling. This is the classic
   FDM-anisotropy failure — Z-tension strength is ~30–50 % of in-plane — and
   it is load-bearing for every fall the robot takes during get-up training.
   Tab thickness is **LOCKED at 2.4 mm** (shin-fork horn arm sweeps 0.4 mm
   outside the tab face at y = 20.05 → 20.45), so the printed fix (v3, in
   progress) must add strength via the heel bulkhead/U-channel tying the tabs
   together, not via thickness.

**Sheet-metal proposal (drop-in, keeps current 100×52 footprint):**

- **Sole plate:** 100 × 52 mm, 0.080" (2.03 mm) 5052, laser cut. Corner radii
  ~5 mm, 4–6 countersunk M3 holes, relief slots under the shin-fork pads
  (currently pockets to z=3.5 — in a 2 mm plate these become through-slots or
  are simply unnecessary since the plate top is at 2 mm < 3.5). Mass ≈ **28 g**
  before holes. Plate bending rigidity ∝ E·t³: 2.03 mm Al (69 GPa) ≈ **1.3×**
  the 6 mm PETG sole (≈2 GPa), in ⅓ the thickness. Isotropic — no layer lines
  to sag or split, dead flat, and rubber pad glues to it the same way.
- **Heel retention: 2× bent L-brackets** (per foot), 0.063" (1.6 mm) 5052:
  vertical leg ≈ 14 mm (X-run, matching the tab band x = −40…−26) × 27 mm
  tall, with the two M3 clearance holes matching the servo case rows
  (x = −29.0 on the horn side, −32.75 on the idler side; hole heights 2.11 and
  22.61 above the servo seat). Base flange ≈ 14 × 12 mm bent 90° outward,
  bolted to the sole with 2× M3 (countersunk from below, nyloc or PEM nut on
  top; the glued pad covers the flush heads). Mass ≈ **2.5 g each**.
  - **Envelope check (the locked 2.4 mm):** leg outer face at
    17.65 + 1.6 = 19.25 mm → **1.2 mm clearance** to the fork sweep at 20.45,
    vs 0.4 mm for the printed tab. Sheet is *better* than printed here. The
    bend bulge (outside radius ≈ 3.2 mm) stays within the leg's outer plane
    and sits at sole level, far below/aft of the fork sweep. ✅
  - **Bend rules:** an L needs only the min flange length (both legs ≥ 12 mm ✅);
    the 2:1 U-channel base rule is what forbids doing this as one U-bracket
    (web 38.5 mm < 2 × 27 mm legs) — hence two L's. Bend angle ±1° across a
    14 mm bend ≈ ±0.25 mm at the top hole; the M3-in-Ø3.4 clearance absorbs it.
  - **Strength:** tab bending capacity ∝ σ·t². Printed tab across layers
    (~25 MPa effective) × 2.4² ≈ 144 MPa·mm²; 5052-H32 (193 MPa yield) × 1.6²
    ≈ 494 → **~3.4× stronger**, and it yields/bends instead of snapping —
    after a bad fall you bend it back or swap a $10 bracket.
- **Servo seating:** the printed foot's 2 mm pocket floor puts the servo
  bottom at z = 4.0 (ankle axis at 16.36 above sole). On a 2.03 mm plate the
  servo would sit at 2.03 → ankle drops ~2 mm. Fix with either (a) a thin
  printed locator tray (~8–10 g PETG, glued/screwed: pocket walls + front end
  stop + 2 mm shim — the metal takes the loads, the plastic only locates), or
  (b) accept the −2 mm and update `ANKLE_AXIS_ABOVE_SOLE` + sim heights.
  Option (a) is less churn.

**Mass accounting (per foot):** 28 g plate + 5 g brackets + ~9 g printed
locator ≈ **42 g vs 33 g printed → +9 g/foot, +18 g robot (+2 %)**. It's
distal mass (slightly more swing inertia — noise at these magnitudes) but at
z ≈ 0, so robot CoM (~280 mm) *drops* ~6 mm — small but the right direction
for the get-up corridor. Update `sim/build_v2_inertia.py` segment masses
either way.

**Enlarged-sole scenario (the pending get-up recommendation):** the rise
corridor is ±20 mm CoP on the 90–100 mm foot (`DESIGN.md` get-up v2/v3
verdict). If the sole grows to ~120 × 60:

| | Enlarged printed (6 mm PETG) | Enlarged metal (2.03 mm 5052) |
|---|---|---|
| Sole mass | ~49 g (+ tabs ≈ 55 g) | ~39 g (+ brackets + locator ≈ 53 g) |
| Stiffness (E·t³) | 1× | 1.3× |
| Layer-line tab risk | still present (v3 mitigates) | eliminated |
| Edge thickness at toe/heel | 6 mm slab | 2 mm blade (less trip/scuff on swing) |
| Cost | ~$1 filament | ~$25–35/foot est. |

At enlarged size the metal foot is **mass-neutral vs printed** while being
stiffer, isotropic, and thinner at the perimeter. This is the scenario where
ordering actually makes sense — one order fixes the tab failure mode, widens
the corridor, and lowers the CoM together. (A 304-stainless sole, ~86 g at
1.52 mm × 120×60, would triple down on CoM lowering as deliberate ballast —
+106 g robot mass; not recommended unless the RL study shows corridor width
is worth more than payload margin.)

**Verdict: printed v3 first; metal on failure or on enlargement.** The
bulkhead-tied printed tab may well be adequate — U-channel geometry fixes the
lever arm even if the material stays anisotropic — and it costs nothing to
find out. Don't order metal to fix a problem v3 may already solve. But if
either trigger fires, the ~$50–80 order below is well-matched to the failure
physics, not metal for metal's sake.

### 2.2 Pelvis deck — stays printed

104 × 46 × 5 mm deck ≈ 27 g printed. A 2.03 mm 5052 deck is ~26 g — **no mass
win** — and the deck is not the load problem: the hanging servo bays (41 mm
walls, U-slot bores, 8× M3 each side, tower heat-set bosses) are integral to
the print. A metal deck turns one print into a plate + two printed bay blocks
+ ~16 fasteners + alignment stack-up, for a part with no demonstrated failure.
Deck loads are compressive/spread (tower feet land over the bay cheeks). If
deck flex ever shows up in practice, a 1.6 mm 5052 *doubler plate* bonded
under the deck (~20 g) is the cheap retrofit — not a redesign.

### 2.3 Tower top plate — stays printed

96 × 42 × 3.5 ≈ 16 g, but it carries integral board standoffs, GoPro screw
bosses, driver-access holes, and vents, and prints face-down as the tower's
bed face. Loads: 154 g camera, worst case ~1.3 N·m in a fall — the printed
gussets already size for this (`parts.py` comment). The crash fuse is
deliberately the `gopro_base` upstream. Metal here protects the wrong part.

### 2.4 Yoke flanges — stays printed, revisit after hip redesign

The 4 mm flanges (32 × 34, M3 on a 20 mm square) transfer real joint torque,
and a bent 5052 clevis is *conceptually* a nice yoke. Three blockers:
1. The horn interface needs the Ø16.5 × 1.0 boss and the idler side a
   Ø19 × 1.2 boss reaching into the Ø25 recess — sheet can't form these;
   you'd need bonded printed boss inserts (fastener/bond stack-up at the
   highest-precision interface in the robot).
2. `yoke_pitch` is on HOLD mid-redesign for the −110° hip target
   (`cad/PRINT_LIST.md`); freezing it in metal now guarantees a re-order.
3. No yoke has failed. Fall loads go through them, so this is the most likely
   *future* candidate — if get-up training starts snapping yoke arms across
   layer lines, a 2 mm 5052 bent clevis with printed boss caps is the move,
   and it's a cheap add-on to any existing order. Not before.

### 2.5 Everything else

`leg_link` (4×), tower body, `gopro_base`: fully 3D or deliberately
sacrificial. No sheet decomposition improves them. Spring steel (0.38 mm
1075) is noted for the future — compliant heel/toe leaves or a battery
retainer clip — but nothing current needs it.

## 3. Recommendation

1. **Do not order anything yet.** Finish and print foot v3 (heel bulkhead).
   It attacks the same failure with zero dollars and same-day turnaround, and
   PETG has been honestly adequate everywhere else.
2. **Decide the sole enlargement** (get-up corridor). This is the swing vote:
   - Enlargement **rejected** → metal only if a v3 tab snaps again. Order:
     4× L-brackets (1.6 mm 5052, bent, ~$40–60 est. with min order) and keep
     the printed sole.
   - Enlargement **accepted** → order the combined foot kit and skip a
     printed v4: **2× enlarged sole plates** (0.080" 5052, ~120 × 60,
     countersunk M3) + **4× heel L-brackets** (0.063" 5052, one 90° bend
     each). Estimated **$50–80 shipped, ~1 week**
     (instant quote before committing; the $39 minimum is nearly met by the
     brackets alone). Add PEM M3 nuts on the bracket flanges if you want
     tool-free sole swaps.
3. **Bookkeeping if metal ships:** update foot segment mass in
   `sim/build_v2_inertia.py` (+9 g/foot drop-in, ~+20 g/foot enlarged),
   re-run `check_assembly.py` with the 1.6 mm bracket envelope, and pull
   SendCutSend's exact 0.063"/0.080" 5052 bend radius + K-factor from their
   bending calculator into the flat-pattern CAD before uploading.
4. **What stays printed, permanently (at this robot's scale):** pelvis,
   tower, leg links, gopro_base, and — pending the hip redesign shakeout —
   the yokes. PETG with 3 perimeters has failed exactly once in a structural
   sense, and that failure (thin tall tab across layer lines) is precisely
   the *only* geometry being offered to metal.
