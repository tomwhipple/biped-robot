# Failure sweep of the kinematic walk, 2026-10-06/07

> Dated record: accurate as of its date; the current design is [DESIGN.md](../../DESIGN.md).

Tom (2026-10-06): run simulations overnight looking for failure modes, and make
sure that one foot actually lifts off the ground. `sim/failure_sweep.py` walks
`static_gait`'s kinematic gait (8 steps, open loop) on the committed plant
(`sim/bimo_biped_v6ar.xml`, main @ 146f714 / 39a12c4: the 6810-2RS hip, the
roll bay 5 mm lower, the folded arm rest pose, STS3250s at the hip rolls). The
conditions are sampled uniformly and every walk is classified. These are
open-loop results: they show where the body and the kinematic gait break, not
what a trained policy can recover.

| sweep | where | walks | space |
|---|---|---|---|
| wide | laptop, 20:50–03:03 | 382,200 | `--space wide`: edges (play to 6°, backlash to 3°, mu 0.3–1.2, servos down to 65 %, tilt ±3°, any lag, stunt gaits) |
| realistic | laptop, 03:16–06:45 | 52,224 | `--space realistic`: the expected envelope (play ≤ 3°, backlash ≤ 1.5°, mu 0.5–1.0, servos 85–100 %, the bench's stiffness band, tilt ±1.5°, normal walking) |
| wide | Mira CPU, 22:00–06:45 | (pending) | seed 100000 |

Classes: **fell**; **no_lift** (a completed step whose swing sole peaked under
15 mm); short_air (< 0.3 s airborne); slip (stance foot > 10 mm); low_margin;
torque / speed saturation; heading drift.

## Rates

| | wide | realistic |
|---|---|---|
| clean | 10.4 % | 30.8 % |
| fell | 70.6 % | 24.6 % |
| no_lift | 44.5 % | 53.4 % |
| short_air | 28.7 % | 22.2 % |
| slip | 29.7 % | 1.2 % |
| torque / speed saturation | 1.6 / 1.2 % | 0.0 / 0.0 % |

## What moves them (rate in the lowest / highest third of each range)

| parameter | realistic: fell | realistic: no_lift | wide: fell | wide: no_lift |
|---|---|---|---|---|
| roll play | **2.4 → 50.6 %** (0–1° → 2–3°) | 49.6 → 55.4 % | 45.3 → 92.3 % | 52.3 → 33.2 % |
| backlash | 16.2 → 33.1 % | 49.1 → 57.4 % | 57.8 → 83.3 % | 45.3 → 43.4 % |
| actuation lag pole | 1.6 → 36.2 % (1 → 3 Hz) | **93.5 → 33.0 %** | 67.3 → 74.0 % | 51.2 → 37.7 % |
| commanded lift | 21.0 → 27.1 % | **83.3 → 29.5 %** (3.0–3.7 → 4.3–5.0 cm) | 66.6 → 74.5 % | 75.6 → 16.5 % |
| swing time | 34.0 → 13.8 % (short → long) | 60.7 → 44.8 % | 78.1 → 62.8 % | 49.2 → 40.0 % |
| friction (mu) | 17.5 → 31.1 % | 51.1 → 55.0 % | 59.4 → 79.2 % | 47.0 → 42.6 % |
| fore-aft tilt | 30.7 → 21.7 % (backward → forward) | 58.2 → 50.9 % | 84.1 → 63.8 % | 43.4 → 46.4 % |
| mass | 20.4 → 28.7 % | 50.2 → 56.5 % | 64.2 → 76.8 % | 45.6 → 43.4 % |
| servo strength, tuned-STS3215 and STS3250 stiffness | flat (± 2–4 points) | flat | flat | flat |

Single-parameter marginals of a uniform sample: interactions are not separated.

## Findings

1. **Roll-joint play decides whether the walk falls.** Inside the ≤ 3°
   requirement, 0–1° of play falls 2.4 % of the time and 2–3° falls 51 %.
   The ≤ 1° target is a requirement for the open-loop walk, not a nicety;
   measure the printed hips' play first (#79).
2. **The swing foot loses most of a small commanded lift.** A 3.0–3.7 cm
   command leaves the sole under 15 mm in 83 % of walks; 4.3–5.0 cm in 30 %.
   The reference gait should command about 5 cm to get a real lift.
3. **Actuation response trades lift against balance.** With a 1 Hz response
   pole the foot almost never clears 15 mm (94 %) and the robot almost never
   falls; at 3 Hz it lifts (33 % no_lift) and falls 36 %. The measured servo
   (2 Hz + 80 ms) sits between: the open-loop walk has no setting that does
   both, which is the policy's job.
4. **Short swings fail both ways** (more falls and less lift): keep swings
   ≥ 1.2–1.6 s for the kinematic walk.
5. **Higher friction falls more** (17.5 → 31.1 %), most likely the swing foot
   catching at low clearance; unconfirmed, to check with the fall times and
   foot traces.
6. **Backward slopes are worse than forward ones** (30.7 vs 21.7 % falls):
   the heel margin is small.
7. **Servo strength and stiffness inside the bench's band hardly matter** to
   the open-loop walk; play, backlash and lag dominate.

Data: `sim/runs/failure_sweep/20261006_laptop/` and
`sim/runs/failure_sweep/20261007_laptop_realistic/` (results.csv, summary.md;
gitignored, regenerate with the commands in `sim/failure_sweep.py`).
