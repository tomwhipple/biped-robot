# Sole friction, measured on the robot and put in the plant (2026-11-04)

This record covers what the bench measured on 2026-11-04, how the CPU sim reproduces that
test, and the plant variant that carries the result. The session's raw data is in the
gitignored `hw_sessions/2026-11-04/` (`NOTES.md`, `fric_clamp.json`, `fric_clamp_yaw.json`,
`fric_clamp_R.json`), so the numbers that matter are copied below. The work is tracked in
issue #137 (the walk / get-up campaign). The current design is in
[training.md](../training.md) §2 and §6. The friction DR this builds on is
[friction-dr](../../experiments/policy/friction-dr/2026-11-03-friction-dr.md) (PR #214).

## What the bench measured

The robot stood in the stand pose with torque on. One foot was C-clamped to the bench
board. The other foot was free on the board at about half the weight (~7.5 N; the sim plant
puts 7.4 N on it). `fric_clamp.py` stepped one joint of the free leg and read back the
servo's position, error, load and current. The test was run twice: the right foot clamped
(16:43–16:47Z), then the left foot clamped (16:54–16:57Z).

**Yaw.** The free leg's hip yaw was stepped 1° at a time to ±20°, at about 10°/s, 2 reps
each way on each foot.

| free foot | reps | tracking error, max | load, median / max | result |
|---|---|---|---|---|
| L (R clamped) | 4 × ±20° | 0.53–0.70° | 48 / 56–72 | the foot rotates on the board, no stick-slip |
| R (L clamped) | 4 × ±20° | 0.53–0.70° | 40–56 / 56–72 | the same |

A load of 40–72 is 4–7 % duty. An unloaded joint reads the same while it moves. So the foot
adds no grip the servo can resolve: **yaw grip ≤ ~0.10–0.15 N·m on both feet**.

**Slide.** The free leg's hip roll was stepped outward 0.5° at a time.

- **L foot:** the hold load of 152 rose to 352 (917 mA). The foot then slipped once at
  cmd −8.5…−9°, with the joint going from −4.7° to −6.9°.
- **R foot:** 2 reps, no stick-slip. The joint crept steadily to +10.0…+10.4° at cmd +15°,
  and the load rose smoothly to 424–456 (~1.1 A), the same in both reps.

The session's estimate takes the ~200 extra load as ~1 N·m on the STS3250 hip roll, over a
0.29 m lever, which is ~3.4 N against ~7.5 N: **μ_slide ≈ 0.45–0.55 (rough)**. The session
did not check whether the creep came from body tilt, because there was no usable video of
the R run.

## The sim reproduction (`sim/sole_friction.py`)

The script runs the same test on the CPU plant:

1. The plant settles standing free on both feet, so each foot carries half the weight.
2. The clamp foot is welded to the world where it stands.
3. The free leg runs the same moves on the plant's own position actuators (kp 40,
   quasi-static):
   - **yaw:** hip yaw 0 → +20 → −20 → 0° at 10°/s;
   - **slide:** hip roll outward to 10° at 2°/s.

What it reports:

- **yaw τ:** the free foot's floor contacts' torque about the hip-yaw axis (sliding plus
  torsional friction). It is the median |τ| while the joint moves, scaled to 7.5 N.
- **Coulomb:** Σ Nᵢ(μ rᵢ + torsion) for the same contacts. This is the session's "0.66 N·m"
  arithmetic.
- **Ft/Fn:** the contacts' ratio of tangential to normal force while the foot slides.
- **N peak:** the peak normal force on the free foot during the slide.
- **μ, bench method:** the session's arithmetic on the sim run. It is the rise in hip-roll
  torque over (hip-roll axis height × the settled 7.4 N).

The contact overrides set the floor and every pad together, as `--contact-mu` and
`grade.set_contact` do. Results with the R foot clamped (the L-clamped run agrees within
0.005 N·m and 0.01 in μ):

| plant, contact | yaw τ @7.5 N | × bench (0.15–0.10) | Coulomb | Ft/Fn | N peak | μ, bench method |
|---|---|---|---|---|---|---|
| **v6legs_meas as drawn** (μ 1, torsion 0.02, edge pads) | **0.339** | **2.3–3.4×** | 0.652 | 0.27 | 18.7 | 0.73 |
| v6legs_meas, DR low end (0.35 / 0.001) | 0.147 | 1.0–1.5× | 0.178 | 0.24 | 17.5 | 0.58 |
| v6legs_meas, DR high end (1.1 / 0.02) | 0.366 | 2.4–3.7× | 0.699 | 0.27 | 18.5 | 0.75 |
| v6legs_meas, μ 0.5, torsion 0 (edge pads) | 0.200 | 1.3–2.0× | 0.244 | 0.25 | 18.0 | 0.63 |
| **v6legs_meas_sole as drawn** (μ 0.5, torsion 0, flat pads) | **0.155** | **1.0–1.6×** | 0.189 | 0.26 | 16.6 | 0.57 |
| v6legs_meas_sole, `--contact-mu` 0.3 | 0.113 | 0.8–1.1× | 0.140 | 0.25 | 16.0 | 0.52 |
| v6legs_meas_sole, `--contact-mu` 0.7 | 0.192 | 1.3–1.9× | 0.264 | 0.27 | 16.6 | 0.61 |

What this says:

- **The plant the legs train on grips 2.3–3.4× the bench in yaw.** The bench number is a
  bound, so the factor is at least that. The session's 4–7× is the Coulomb bound
  (0.65 N·m, which the script reproduces). In MuJoCo's soft contact (pyramidal cone,
  impratio 1), the slowly sliding foot gets only about half of that bound at μ 1, and about
  80 % of it at μ 0.3–0.5. The effective grip also depends on rate on the old plant:
  0.25–0.45 N·m from 10 to 150°/s. On the new plant it is 0.14–0.16 N·m at any of those rates.
- **Torsion and pad placement together make up most of the excess.** The pads' torsional
  0.02 m adds 0.15 N·m of Coulomb bound at 7.4 N. It is a ~2 cm contact patch on a 3 mm
  sphere. The edge octagon puts all of the sliding friction 6.8 cm from the sole centre. A
  flat, uniformly loaded sole has its mean lever at ~4 cm.
- **The yaw DR already covered the bench, but only at its low end.** μ 0.35 / torsion 0.001
  gives 0.147 N·m. The rest of the `--contact-mu 0.35,1.1 --contact-torsion 0.001,0.02` box,
  and the plant that referees every run (μ 1 / 0.02), are 1.5–3.7× too grippy.
- **The slide test does not measure μ well, in the sim or on the bench.**
  - Pushing the straight leg outward loads the foot. The normal force goes from 7.4 N to
    16–19 N because the body props itself over the clamped foot.
  - The foot rolls onto its outer edge (~5°).
  - In the sim the foot creeps outward at Ft/Fn ≈ 0.25 whatever μ is set to.
  - The bench arithmetic (constant 7.5 N) therefore reads high. It gives 0.52–0.61 for
    μ 0.3–0.7, and 0.73 at μ 1.
  - The bench's 0.45–0.55 is consistent with μ ≈ 0.3–0.7 and not with the plant's 1.0, but
    it does not pin μ more tightly than that. The tilt-board slide angle (μ = tan θ, asked
    for in PR #214) still would.

## The change: `sim/bimo_biped_v6legs_meas_sole.xml`

This is a plant variant, generated the same way as the measured-stops plants. It is
`gen_plant_v6.measured_sole()` on `measured_stops(asbuilt_legs())`, built by
`sim/build_v6_inertia.py --measured-stops --measured-sole --asbuilt-legs --write` (`make
plant`). Only the pads differ from `bimo_biped_v6legs_meas.xml`. `tests/test_plant_sole.py`
pins that, along with everything below.

- **Pad friction `0.5 0 0.001`.** μ 0.5 is the middle of the bench's 0.45–0.55. The
  torsional coefficient is 0, because a 3 mm pad has no yaw brake of its own.
- **Pad `priority="1"`.** The pad's friction is the foot contact's friction, so MuJoCo no
  longer takes the MAX of the pad and the floor. The floor stays 1.0. Every other floor
  contact (fall colliders, servo boxes) and the carpet terrain's floor keep their own values.
  On terrain, the carpet's 1.3 / 0.05 no longer reaches the feet. CPU MuJoCo, MJX and
  MuJoCo Warp all honour the attribute: the tests check CPU and MJX, and the Warp source was
  read (`collision_core.py`, `geom_priority`). The floor-only `friction_range` draw no longer
  moves a foot contact. `--contact-mu` sets the floor and the pads together, so it still
  draws the foot's μ.
- **"Flat" pads.** There are still 8 per foot, so MJX keeps the same contact slots.
  - 4 pads sit on the sole outline, at the 45° points of the corner arcs.
  - 4 inner pads sit on the same diagonals, placed so that the 8 pads' mean distance from
    the sole centre is 0.042 m. That is the yaw lever of a uniformly loaded flat sole.
  - A flat-loaded foot therefore resists yaw like the real sole. A foot rolled onto an edge
    stands on that edge's pads, as before.
  - The support polygon is 12.2 × 7.6 cm (92 cm²), against the edge octagon's 13 × 8.4 cm
    (105 cm²). That is 88 %, and 4 mm less on each side. Moving all 8 pads inward to a
    0.042 m radius would cut it to ~40 %, which would be a second change to the plant
    (balance margin) on top of the friction one.

On this plant the yaw test gives 0.155 N·m at 7.5 N, at the bench's bound. With
`--contact-mu 0.3,0.7` it spans 0.11–0.19 N·m.

**Nothing changes for a job that does not name the plant.** The four existing plants are
byte-identical, and every default (plant, DR ranges, referee) is as before. A run trained on
the new plant is refereed on it at μ 0.5, because the referee rebuilds the plant from
`config.json`.

### Training on it

Use these flags in place of the legs jobs' current `--xml bimo_biped_v6legs_meas.xml
--contact-mu 0.35,1.1 --contact-torsion 0.001,0.02`:

```
--xml bimo_biped_v6legs_meas_sole.xml --contact-mu 0.3,0.7
```

Do not pass `--contact-torsion`. With it, the draw would put a torsional brake back on every
pad. `grade.py` grades on the plant with `GRADE_XML=bimo_biped_v6legs_meas_sole.xml`. Its
`mu0.4` / `mu0.3` probes still set floor and pads together.

## Not done

- **The ar plant (`v6ar_meas`, the get-up body).** `--measured-sole` without
  `--asbuilt-legs` generates it, but it is not committed. The get-up's body–floor contacts
  are unaffected by the sole either way.
- **μ itself.** It is known to about ±0.2 (above). The tilt-board slide test would pin it.
- **Soft-contact creep.** The slowly sliding foot reaches only part of μN in MuJoCo, at any
  μ. The real sole showed no stick-slip either. The solver options (`impratio`, the cone)
  are unchanged.
- **Other floors.** The test was done on the bench board only. Carpet and the hard floor are
  not measured.
