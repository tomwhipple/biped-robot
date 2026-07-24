# Hip-yaw design study — do we revisit the hip?

**Status:** study, 2026-07-23. User question: "Do we need to revisit the hip
design? If so, what are the implications? Would it change the whole design or
can we just change the hip and maybe upper leg?" Spare servos are on hand;
the servo order hasn't arrived, so nothing is assembled — this is the
cheapest moment the change will ever have.

## 1. The problem

The robot cannot change its facing. No hip-yaw joint exists (per leg:
hip roll, hip pitch, knee, ankle pitch — all in the sagittal/frontal
planes), so the feet always point where the pelvis points. Turning must
come from friction-pivot stepping. Measured (new `turn_180` referee
scenario, 2026-07-23): **loco_v1 commanded to turn 180° turns ~8°**
(heading error 172°, no falls — it simply doesn't rotate).

Two compounding findings:

1. **Reward-side gap (fixable in software):** rate kernels forgave chronic
   under-turning. The heading integrator (loco_v3, tonight) is the fix and
   gives the software-only best case.
2. **The sim currently flatters pivoting.** Neither plant sets `condim` —
   MuJoCo's default 3 means **zero torsional friction**: a planted sole can
   spin about the vertical axis for free in sim. The real 90×46 mm silicone
   pads resist torsion strongly. So whatever pivot ability loco_v3 learns
   is an *optimistic upper bound* for hardware. (Follow-up landed with this
   study: torsional friction in the plants before any "no-yaw is fine"
   verdict is trusted.)

Precedent is one-sided: every comparable robot that turns has hip yaw —
Open Duck Mini (closest cousin: same STS3215 servos, 5 DOF/leg, yaw first
in the stack), the BD-X droids it copies, ToddlerBot. We would be the
outlier trying to turn without it.

## 2. Recommendation

**Add hip yaw — one servo per leg, yaw-first in the stack (pelvis-mounted,
vertical axis, above the existing roll servo).** Decision gate before CAD
work starts: tonight's loco_v3 (software-only best case) plus a sim A/B on
a 10-DOF variant plant. But the expectation is clear, and the user shares
it: plan for the change.

## 3. Would it change the whole design? — No. It's the pelvis + one new part.

Current stack: pelvis deck (two hanging bays holding the hip-roll servos)
→ roll yoke (50 mm) → hip-pitch servo → thigh 90 mm → knee → shin 90 mm →
ankle → foot.

Yaw-first insertion point (Open Duck arrangement):

| part | fate |
|---|---|
| **pelvis** | **redesigned** — the deck grows two horizontal yaw-servo seats (case lies flat under/in the deck, output boss down); the hanging roll bays detach from the deck |
| **yaw carrier** (×2) | **new part** — bolts to the yaw horn, carries the existing roll-servo bay geometry |
| roll yokes | unchanged |
| leg_link (thigh) | unchanged |
| shin, ankle, foot | unchanged |
| torso/tower/GoPro stack | unchanged (deck bolt pattern preserved by design) |

Upper leg does NOT need to change: yaw-first keeps both added servos on the
**pelvis side of the roll joint** — sprung mass, not swinging leg mass, so
leg inertia (and the swing dynamics every gait learned) stays close to
current. Yaw between roll and pitch (in the yoke) was rejected for exactly
that reason.

## 4. Implications (the honest bill)

**Mass:** +2 × ~55 g servos + ~30 g printed carriers ≈ **+140 g on 920 g
(+15%)**, at deck height (near the CoM — modest CG shift, slightly down
relative to the tower). Watts/CoT rise proportionally.

**Geometry:** hip line drops (or torso rises) by the flat-lying servo stack,
~25–30 mm. Standing height ~34 → ~37 cm. Leg workspace unchanged; yaw range
±45° is ample (turn strides need ±20–30°; V-stance and heel-to-heel become
trivially reachable).

**Electronics:** 10 servos on the 1 Mbaud bus — sync-read grows ~3.5 →
~4.3 ms, still comfortable in the 20 ms tick. Power: yaw is gravity-neutral
in stance (it carries swing inertia, not body weight), so the "2–3 joints
near stall" transient budget in wiring.md still holds; verify the Waveshare
board's bus connector rating for 10 IDs on delivery.

**Software:** action/obs dims 8→10 joints (obs frame 43 → 49, ×3 history =
147). The envs hardcode 8 in places — a parameterization refactor + parity
re-run (~a day). All current checkpoints become non-deployable; that costs
little since the specialist rounds retrain from scratch anyway and the
curriculum/referee transfer unchanged. Firmware design doc: obs spec +
servo count sections update (no architectural change).

**Print queue:** pelvis reprint + 2 carriers; everything else printed stays.

## 5a. VERDICT (morphology A/B, overnight 2026-07-23→24) — **yaw wins, decisively**

Same recipe (loco family, heading integrator w=1.0, full action map, 110M
from scratch), same honest torsional friction (condim=4), 8 seeds/scenario
on the CPU referee:

| scenario | 8-DOF control (loco_v3t) | 10-DOF yaw (loco_v4yaw) |
|---|---|---|
| turn_180 | 1/8, hErr 59° | **7/8, hErr 10°** |
| square_return | 0/8, ret 126 cm | **5/8, ret 55 cm** |
| circle_return | 0/8, ret 287 cm | **2/8, ret 47 cm** (first passes ever) |
| overall | 36/64 | **53/64** |

Turning is a morphology problem, not a training problem: the best-effort
8-DOF policy under the heading integrator still misses a 180° turn by 59°
on average; the yaw plant nails it to 10° and unlocks the return-to-start
block. The 63 W from-scratch energy is the known pre-fine-tune loudness
(+15% mass); loco_v2's fine-tune template (44→5.8 W) is queued as
loco_v4yaw_s. **The print decision gate is passed** — pelvis + carriers
await the user's §6 sign-offs.

## 5. Evidence plan (this week)

1. **Tonight (armed):** loco_v3 = heading integrator on current 8-DOF —
   the software-only best case, and it's friction-flattered (see §1.2).
2. **Next:** 10-DOF variant plant (`bimo_biped_v3yaw.xml`, approximated
   masses, torsional friction ON for both plants) + env DOF refactor →
   same training recipe, morphology A/B on turn_180 / square_return.
3. **If the A/B confirms** (expected): CAD the pelvis + carriers, print,
   and the spare servos go in when the order arrives.

The whole-design answer: **no redesign cascade.** Pelvis + one new bracket,
a taller robot by ~3 cm, +15% mass, two more bus IDs, and a sim refactor we
were going to need for any future DOF change anyway.

## 6. As-designed (CAD, 2026-07-23)

The change is cut in `cad/dimensions.py` + `cad/parts.py`: `pelvis` redesigned,
`yaw_carrier` ×2 new; roll servo, yokes, `leg_link`, shin, foot **untouched**.
`check_assembly.py` ALL CLEAR, `check_printability.py` clean, both parts print
support-free on the AD5M Pro. Render: `cad/renders/hip_yaw_beforeafter.png`
(dark = servos; the yaw pair seats under the deck, the hip line drops onto the
new carriers). Part renders: `renders/yaw_carrier.png`, `renders/pelvis_yaw.png`.

**Geometry chosen.** Each yaw servo lies flat under the deck, **length fore-aft
(X)**, width across (Y), vertical output axis at the leg centre, **horn DOWN**,
idler-side case face pressed to the deck underside. `yaw_carrier` bolts to the
horn below and carries the old roll bay verbatim (same `BAY_BORE` 20.6,
`BAY_WALL_DROP` 41, cheeks, U-slot, 8× case screws). The roll servo slides up
into the carrier exactly as it did into the pelvis.

**True stack height — 40.8 mm, NOT ~28.** The hip-roll axis drops from
`DECK_T + SV_AXIS_FROM_REAR` = 40.11 mm below the deck top to 80.91 mm below it
(`ROLL_BELOW_DECK_YAW`), a **`YAW_STACK_DROP` = 40.8 mm** rise of everything
above the roll joint. The study's 25–30 mm was optimistic: the drop is the
servo's own horn-to-idler grip span (`SV_GRIP_SPAN` 37.25) + the carrier horn
plate + the idler recess — the two servos cannot nest (their XY footprints
coincide about the shared vertical axis), so the second servo stacks in full.
- Standing structure top rises 330.5 → **371 mm** (`TOP_Z_YAW`); hip-yaw joint
  plane (horn face) `HIP_YAW_Z` ≈ 286 mm; deck bottom `DECK_BOT_Z_YAW` ≈ 328 mm.
- **Action for the sim sibling:** set `bimo_biped_v3yaw.xml`'s torso raise to
  **+41 mm, not +28 mm** (or accept a 13 mm standing-height error). All the
  v3yaw height constants are in `dimensions.py` (`HIP_YAW_Z`, `DECK_BOT_Z_YAW`,
  `TORSO_CENTER_Z_YAW`, `TOP_Z_YAW`, `YAW_STACK_DROP`).

**Mass (printed, PETG).** `pelvis` 47 → 32 g (bays left); `yaw_carrier` 17.5 g
×2. Printed plastic 307 → **329 g** (+22 g). With the two spare STS3215
(+110 g) and fasteners, **total robot ≈ 914 → 1054 g (+140 g, +15%)** — dead on
the §4 estimate. Segment masses for the sim are printed by `parts.py` (new
`yaw carrier` segment = carrier + roll servo, 77.5 g/leg; the two yaw servos
add to the torso).

**Idler-grip decision — HORN-ONLY (single-sided), by geometry.** Bolting the
stator to the deck uses the idler-side case face, which is the *same* face a
both-sides carrier would need to reach to grip the idler disc — the two are
mutually exclusive at a deck-mounted vertical axis. We took the **rigid,
deck-bolted stator** (4× M3 into the idler-side rows, torque keyed by a collar)
and drive the carrier off the horn alone. This is Open Duck Mini's proven
arrangement for this exact servo/leg. **The both-sides option is deferred, not
rejected:** it would float the stator on a keyed pocket, grip both discs, and
add a deck counterbore for the idler arm — a second bearing at the cost of
stator rigidity + a tolerance stack. **User sign-off wanted here** (see below).

**Sweep-clearance proof (`check_assembly.py`, all booleans 0.00 mm³).**
- Yaw carrier + its roll servo at **0 / +45 / −45°** vs the pelvis deck **and**
  vs the fixed yaw servo: no interference (the carrier rides 3–6 mm below the
  yaw case throughout the sweep).
- **Both carriers yawed inward 45° simultaneously** (worst mutual approach):
  facing-envelope gap **6.2 mm** — positive, so ±45° is mechanically clear at
  HIP_SEP = 56. (Turn strides need ±20–30°, so this is comfortable margin.)
- Roll ±25° and deep hip flexion −110/−115° re-proven against the **carrier**
  (identical bay geometry) — ALL CLEAR, matching the old pelvis-bay result.
- Carrier envelope: bbox X −20..+20.5, Y (from axis) 12.7..43.3, Z −91..−42.8
  in the pelvis frame; well clear of the deck edges (deck ±23 X, ±52 Y) and the
  other leg.

### Wiring routing (design-review fix, 2026-07-23)

Review caught it: "wires pass through the servo mounts without a hole." The
flat yaw cases sit on the deck underside *exactly where the old per-bay deck
cutouts were*, and those cutouts were deleted with the bays — so the
board→first-servo lead and the roll-servo cable had no passage. Fixed with two
new openings (constants `WIRE_*` in `dimensions.py`; render
`renders/hip_yaw_wiring.png`, semi-transparent, leads in red). The v3yaw
daisy-chain is board → **hip-yaw → hip-roll** → pitch → knee → ankle:

| Segment | Route | Opening (size, passes) |
|---|---|---|
| board → hip-yaw | tower rear → onto the deck rear tab → down behind the yaw case | **pelvis rear wire chase (NEW)** — 12 mm (`WIRE_CHASE_1`) vertical slot at x −39.0…−35.1 (behind the case rear face), cut through the rear deck tab **and** the collar rear wall, z +1…−10; sits between the −32.75 stator screws (y ±10.25), clearing them 4.25 mm. 1 bundle + plug |
| hip-yaw → hip-roll | yaw rear port → down the open gap below the collar (z −9…−42.8) → into the carrier | passes through **open air** (the gap between collar bottom and carrier top) — no material crossing |
| into/out of the roll bay | yaw→roll lead drops in from above; roll→pitch lead exits rearward-down the thigh | **carrier rear cable channel (NEW)** — 14 mm (`WIRE_CHASE_2`) wide, open at the ceiling **and** through the rear wall's top 8 mm. 2 bundles. *(Widened forward to `YAW_CH_FRONT` = −6 and the rear horn bolt dropped in rev 2 below, to uncap the roll connector.)* |
| roll → pitch → knee → ankle | down the leg | **UNCHANGED** — the leg_link web windows (9×11) + foot cable window carry it, exactly as on the 8-DOF robot |

**Yaw-sweep (service loop).** The yaw→roll lead is the only lead crossing the
new joint. Its slack lives in the open rear gap (z −9…−42.8), which
`check_assembly.py` proves clear through the whole 0/±45° sweep (carrier vs yaw
servo = 0.00 mm³). The carrier channel's front corners are rounded so no sharp
shear edge bears on the lead as the channel rotates under the fixed yaw port.
Assembly note: fit a rubber grommet in the deck chase and leave a ~15 mm
service loop at the yaw joint (add it to the `wiring.md` segment table — the
board→hip-roll hop is now board→hip-yaw→hip-roll, two hops).

**Checks after the cuts:** `check_printability.py` PASS (pelvis + yaw_carrier);
`check_assembly.py` ALL CLEAR (openings added no interference). The first cut
of the carrier channel at 16 mm left a 0.55 mm sliver against the retention
screw hole — narrowed to 14 mm (`WIRE_CHASE_2`) for a 1.55 mm ligament.

### Wiring routing — full-assembly audit (2026-07-23, rev 2)

Review pushed back ("still might need cable holes... can't see the deck
chase"). Did a definitive audit against the FULL assembly (pelvis + tower +
board + both yaw servos + carriers), not the pelvis alone. Method: section the
solids at the chase coordinates (point-in-solid), and route candidate harness
tubes for every hop and measure where they pierce material. Renders:
`renders/hip_yaw_wiring.png` (rear ¾) and `hip_yaw_wiring_front.png` (front ¾),
semi-transparent, red tubes board→thigh.

**Existing chases — verified PRESENT.** Point-in-solid confirms material is
*absent* inside both the pelvis rear chase and the carrier channel and *present*
just beside them. The deck chase reads as invisible only because it is a thin
(3.9 mm) slot tucked behind the case under the rear tab — it is really there.

**Real gap found + fixed (the review was right):** the hip-roll servo's cable
connectors point UP out of the bay ceiling, and the carrier's yaw-horn plate
capped them — a connector block hit the solid plate at 15–183 mm³ across the
cable-end face (the plate→yaw-case gap is only 3.1 mm and the plate centre is
under the Ø19.2 horn). **Fix:** drop the rear-most of the 4 yaw-horn bolts and
open the carrier cable channel forward to `YAW_CH_FRONT` = −6, uncapping the
rear-of-face connectors (where the ST3215 pair actually sits — they can't be at
centre, the horn is there). Post-fix the connector clears at 0 mm³ for every
rear position; 3× M3 on the Ø14 circle still carries the ~0.3 N·m yaw torque.

**Every deck-plane crossing (audited, printed parts CLEAR):**

| Lead | Crosses the deck plane at | Hole |
|---|---|---|
| board → hip-yaw_L / _R | **centre wire window** (x ±11, y ±12) — clear of the flat yaw cases, which sit outboard at y 15.6–40.4; then under-deck *inboard* of the case and *behind* it to the rear port (below the collar, in open air). The rear chase is an alternate relief crossing at the port. | centre window (existing) + rear chase |
| hip-yaw → hip-roll | entirely BELOW the deck (open gap → carrier channel) | carrier channel |
| hip-roll → pitch → knee → ankle | down the leg | leg_link windows + foot window (unchanged) |
| **yaw_L ↔ yaw_R** | **no such hop** — the board has two bus ports, one chain per leg (wiring.md §Servo bus), so there is no cross-deck hop between the seats and no cross-channel is needed | — |
| battery XT30, IMU jumpers, board power | **all inside the tower** (pack on the deck → board; IMU on the tower-top carrier → down the tower's open rear) — confirmed, no deck crossing | — |

Routed tubes pierce **no printed part** (pelvis/tower/carrier all 0); the only
grazes are against the servo-case *exterior* (the lead lying on the servo, as
intended). Both check scripts pass after the carrier change: printability CLEAN,
assembly ALL CLEAR (yaw 0/±45° re-confirmed unchanged).

Open item for `wiring.md` (main thread — not in my file set): the segment table
still lists the 8-DOF "board→hip-roll" hop; it becomes board→hip-yaw→hip-roll
with a ~15 mm service loop at the yaw joint, and the board now needs ID 1 = yaw.

### Open questions for the user

1. **Stack height 41 vs 28 mm.** Confirm the taller robot is acceptable
   (standing ~34 → ~38 cm) and have the sim use +41 mm. If 41 mm is too tall,
   the only lever is a lower-profile yaw actuator — the STS3215 grip span is
   fixed at 37.25 mm and sets the floor.
2. **Horn-only vs both-sides idler grip.** Sign off on single-sided (simplest,
   most rigid stator, Open Duck precedent) or ask for the both-sides v3.1
   (second bearing for the cantilevered leg). Recommendation: **print and bench
   the horn-only carrier first**; add the idler bracket only if the output shaft
   shows bending play under leg load.
3. **Pelvis grew ~15 mm rearward** (bbox 61 mm fore-aft) to cover the yaw case
   cable-end overhang and its −32.75 stator-screw row. Confirm nothing on the
   deck rear (wiring, switch) fouls the local rear tab, or move the yaw servo
   output-end rearward to overhang the front instead.
4. **Yaw + roll cable routing.** The roll-servo cable exits up through a rear
   slot in the carrier ceiling next to the yaw horn; the yaw-servo cable exits
   its rear end face over the deck rear edge; both then rise through the deck
   centre window. Verify with real 150 mm leads — a yaw-crossing service loop
   (±45°) is new and may need an extension.
5. **Stator hold.** 4× M3 into the idler-side case face + a 4 mm collar is the
   mount; confirm the case's Ø3.5 holes take M3 self-tappers (the standing BOM
   question) before committing, since the stator now carries the full leg load
   in bending, not just the roll reaction.
