# No-appendage fall-recovery search on the v6 body (2026-09-14)

**Status: studied, negative for a robust no-appendage static rise on the
current body; the passive-pelvis-skid lever added to the plant but the
sim validation could not be finished this session (terminal tool wedge).**
Tom: *"return the robot (v6/v7) to standing if it falls ... without the tail
or arms ... thinking creatively, remembering the robot can rotate most of its
joints further than humans."*

## What the appendage study already established (docs/design-v6-ankle-roll.md §12, §12.1)

On the bare v6 body (knee −130, hip −125, no appendage):
- supine → **sit** is easy (the RL sit policy + the deep ROM reach up_z 0.99),
  and **prone → side → back** is solved **with the legs alone** (§12.1, the
  leg-roll / lower-leg-press / swing-up-and-back chain: 26/72 → back).
- **supine → standing** is the unsolved step. From the seated crouch the hip
  joints sit 4 cm off the floor, the torso is vertical at full hip flexion (the
  thighs point up 35°), and the CoM is 8 cm behind the feet. The rise corridor
  is ±20–28 mm CoP on a 90–116 mm foot and tips backward mid-rise. §11.3's
  deep-ROM body (knee 130/hip 125) did not change it: **0/32**, including with
  a pelvis skid, and the head lands first on every prone fold.

## What this session swept (deploy servo model, STS3250 rolls+knees)

All below ran under `getup_v6_*` harnesses with the real servo envelope
(2 Hz shaper, 80 ms dead time, torque–speed clamp). Each family ended **not
standing**; the traces show *why*:

| family | idea (the beyond-human lever) | result / failure mode |
|---|---|---|
| **tripod** wide-splay rise | splay both legs 45° abduct (L+ / R−) into a broad A so the CoM stays inside a wide support triangle | 0/27. The torso tips backward on the rise; the wide feet don't move the CoM forward over them. |
| **kneel** high-seat rise | deep knee/hip ROM raises the seat (kneel) so the CoM crosses over the feet while high | 0/16. Tucking to a deep crouch just settles back to the floor (pelvis stays 4–5 cm). |
| **kip** backward somersault (from supine) | whip legs up-and-over the head, use angular momentum to roll over the shoulders onto a caught crouch (gymnast, arms-free) | 0/18. peak_up stays ~0.4–0.7; the torso/head-lead and the deploy 2 Hz bandwidth can't generate enough angular velocity. |
| **kip-from-sit / kip-rock** | start from the upright sit (up_z 1.0), rock back then whip legs down-under | 0/8. Same: not enough momentum under the servo shaper; max_tau ~1.3–1.8 N·m (below the ~4 N·m stall) so it is **bandwidth-bound, not torque-bound**. |
| **side / prone bear-plant** | use the extended ROM to plant the FEET beside/behind the hips as hand-like third/fourth contacts, then press up into a bear/crow stance and rise | side 0/108, prone 0/96. Pressing lifts the pelvis to 0.10–0.14 m (past the tail's ~10 cm target) but the **head leads** every press (prone) or the near foot can't reach the floor behind the torso (side), piking onto the head / sliding. |
| **heel self-push** | bring one foot up-and-back beside the hip (heel-to-buttock), plant it flat as a self-made tail, press down to lift the pelvis | 0/N. The planted foot never reaches the floor behind the hips (heel probe: foot stays 23+ mm off it); the limb is not long enough/positioned to be the tail's far-behind third contact. |

## The physics wall (confirmed, all families)

**No far-behind third contact → no pelvis lift.** The appended study isolated
the mechanism: the appendage only worked because it pushed **from behind the
hips** at hip-roll height (the housing-bottom/high push lifted the torso back
instead). Without a tail/arm, every candidate either (a) pikes the boxy torso
onto its head (prone bear, kip), (b) can't reach the floor far enough behind
(side/heel plants), or (c) can't move the CoM forward over the feet from the
4 cm seat (tripod, kneel, static rises). The 2 Hz deploy servo makes dynamic
kip momentum infeasible, so the static corridor is the only path and it needs
a raised seat.

## The one lever added, not yet validated this session

`sim/gen_plant_v6.py` gained a **passive pelvis skid** under the housing
(`skid=True / skid_h / skid_x / skid_len / skid_w`), a fixed structural seat
(not a tail or arm): its bottom sits `skid_h` below the yaw axis so a seated
hip is `skid_h` mm off the floor. This is the §11.2 "seat below the hips" lever;
`getup_v6_skid.py` + `getup_v6_comprobe.py` were written to sweep skid height ×
deep-flex pose and check whether the CoM crosses over the feet (butt-up). The
terminal wedge stopped the runs before a verified result. The plant change is
**backward-compatible** (skid=False → identical plant).

## Next step (recommended)

Finish the skid × CoM-feasibility probe (unblock the terminal, then
`sim/getup_v6_comprobe.py`): if the loaded deep crouch has CoM over the feet
at skid_h ≥ ~8 cm, the passive skid + butt-up rise is a genuine no-appendage
get-up worth a CAD bracket. If not, the physics conclusion stands: **no
no-appendage quasi-static rise exists on this 6-DOF body**, and the options
are (a) ship the tail (already 5/5 standing incl. prone→back), (b) a pelvic
skid + learned RL rise (dynamic, but bandwidth-limited), or (c) accept manual
reset — which the record recommends against for a real walker that will fall.
