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
