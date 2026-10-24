# One-leg stand (v6 legs, open loop)

The Gate A path held still, as a bench experiment: crouch 20 mm, shift the pelvis
until the whole-robot CoM sits 5 mm inboard of the **left** sole centre, lift the
**right** foot 30 mm, hold, and undo it. No policy and no feedback. This is the
"bench before policy" step for the curriculum's `balance_L/R` skill
([docs/precision-curriculum.md](../../docs/precision-curriculum.md)).

`one_leg.py` builds the IK keyframes (`sim/v6_kin.leg_ik` through
`design_gates.q_of`) and simulates what the robot actually runs: the firmware's
`pose ... s<ms>` streamer, a minimum-jerk move in **joint** space between
keyframes. The plant and servo model are those of `sim/static_gait.run_walk`
(2 Hz shaper + 80 ms dead time, integer ticks, 1° backlash, 3° play on the four
rolls), with STS3250s at the two hip rolls as built. It writes the keyframes as
`pose` bus-order ticks for the as-built calibration (`ticks = zero + dir · rad ·
4096 / 2π`, [docs/servo-map.md](../../docs/servo-map.md) §3).

```bash
.venv/bin/python experiments/one-leg-stand/one_leg.py --lift 0.03 --json build/one_leg/sweep30.json
.venv/bin/python experiments/one-leg-stand/one_leg.py --lift 0.03 --no-sweep --json build/one_leg/nominal30.json
```

## Keyframes (degrees, sim frame)

| pose | hip roll L / R | hip pitch L / R | knee L / R | ankle L / R | ankle roll L / R |
|---|---|---|---|---|---|
| crouch 20 mm | 0 / 0 | −24.6 / −24.6 | −49.2 / −49.2 | −24.6 / −24.6 | 0 / 0 |
| shift (pelvis +53 mm) | −9.8 / −9.8 | −18.8 / −18.8 | −42.9 / −42.9 | −24.1 / −24.1 | +9.8 / +9.8 |
| lift R 30 mm | −9.8 / −10.8 | −18.8 / −34.0 | −42.9 / −74.4 | −24.1 / **−40.3** | +9.8 / +10.8 |

The lift's right ankle is 0.3° past the −40° bench envelope (`mech_envelope.h`),
so the firmware clamps it. No keyframe has a self-contact. The shift is a
parallelogram (both legs roll together), so the gap between the legs stays
constant through it.

## Sim result

Nominal 30 mm lift: stays up, the right foot is off the ground for 11.1 s of
the 11 s window, CoM margin ≥ 29.9 mm (static kinematic margin at the lifted
pose: 18.5 mm), torso tilt 4.5°, stance slip 4.7 mm, every joint under 30 % of
stall. Over the sweep (μ 0.4 / 1.0, 5° roll play, 2° backlash, mass ±15 %,
120 g payload, servos −15 %, floor ±2°, and a combined worst case; 3 seeds each)
it **never falls**, the margin stays ≥ 19 mm (mass ×0.85), and the foot is off
the ground ≥ 10.8 s. Real clearance is 10–17 mm, short of the curriculum's 3 cm
bar: the hanging leg sags through the roll play and the torso tilts toward it.
`run_walk`'s own OK column wants ≥ 15 mm clearance, so it marks some of these
rows FAIL.

A **15 mm** lift command does not lift the foot at all in sim (0–3 mm): that
much is eaten by the sag. Use 30 mm.

## Hardware

- [2026-10-24, on the stand](2026-10-24-stand-rehearsal.md): the full sequence on
  the test stand (pelvis held). Every stage reached its targets within 6 ticks;
  the shift moved the feet to the robot's right, as predicted. This checks signs,
  ranges and tracking, **not balance**.
- Next: the same poses on the floor with a spotter, one stage per go.
