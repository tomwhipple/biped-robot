# March in place on the slow-gait recipe (2026-11-10, #261)

Record of what the march command does on the `prec` recipes and what was added to make a march slice
usable there. Design: [training.md](../training.md) §4 (commands), §5 (the imitation prior).
Everything below was observed on CPU (the CPU env and MJX's jax impl on CPU); no training ran.

## Facts

Recipe under test: the prec46 flags (`--family loco`, `--gait-freq 0.7,0.8`, `--mimic-lift 0.02`,
`--w-mimic 1.5`, `--mimic-sway 0.035`) plus `--march-mix 0.15`. `make_parser()` accepts it, and the
robot-preset env builds on the CPU (the referee's `make_env`) and as `BimoMJXEnv` (jax impl, CPU).
The Warp impl was not built (no GPU used).

| | observed |
|---|---|
| cadence at the 0.75 Hz clock | `--march-mix` (traj mode 4) alternates c4 on the gait clock: flips every 0.66–0.68 s over 8 s (MJX and CPU agree) |
| lift asked by `--march-mix` | c6 = `march_dz`·\|sin φ\|, peak 0.122 m, so the swing-sole target is `lift_height` 0.06 + 0.122 = 18.2 cm (the standing knee height); the link clamps c6 to 5 cm (`FOOT_D_MAX`), so the robot cannot be sent it |
| the plain march (`--ext-mix` slot 6) | c4 alternates on a per-episode clock, not the gait clock: flips every 1.28–1.6 s over the draws seen; c6 ≤ 2–4 cm over `lift_height`, so a target of 8–10 cm |
| reference during a march | **off**. A march is a lifted command (`\|c4\| > 0.5`) and the `w_mimic` gate is `~lifted`. CPU zero-action march return with `w_mimic` 1.5 vs 0: 725.9 vs 724.4 over 250 ticks, the 1.5 being the draw's first tick, where c4 is still 0. `--mimic-lift` and `--mimic-sway` only act through that term, so they do nothing in a march. It does not hold the standing pose either: `w_pose`, `w_feet_phase`, `w_walk_shift`, `w_symmetry` and `w_squeeze` are all gated off with it |
| what pays for a march | `w_lift` (swing foot ≥ 3 cm, stance foot down), `w_track_foot` (σ 6 cm, target `lift_height` + c6), `w_single_support`, `w_foot_cross`; `w_knee_high` and `w_com_stance` only when set (both 0 in `loco`) |
| lateral weight transfer | only `w_com_stance` (CoM over the stance sole, σ 4 cm, gated by the swing clearance) pays for it. Nothing in the loco recipe does |
| zero-action march | stands 250 of 250 ticks; a uniform ±0.3 random policy (seed 0) fell at tick 27 |
| firmware | no change. `obs_spec.h` carries the layout, the clock and `kGaitFreqHz`, none of which a march changes. `policy::pinFrozen` pins by the net's own normaliser: every prec student so far has std 1e-6 on c3..c6 (measured on `robot_legs_prec16_f075_js20` and `_s128r24`), so the robot pins c4 and c6 to 0 and ignores a march command; a student that drew a march has them unfrozen and `pinFrozen` leaves them. `gen_obs_spec.py` needs no change |
| the warm start | **breaks**. prec16's normaliser (count 3.1e9) holds std 1e-6 on c4 and c6, so c4 = ±1 reads as ~1e6 σ: every warm start from a prec student needs the slots re-seated (the crouch problem) |

The knee-high target is therefore not the 3 cm march the plan asks for: it asks 18 cm, which the link cannot send, in 0.33 s
of upswing.

## What was added (all opt-in, defaults untouched)

- `--march-lift L`: the march's peak commanded swing height (m). `0.08` = c6 up to 2 cm; `0.06` = c6 stays 0 (c4 alone moves).
- `--mimic-march M`: during a gait-clock march the reference lifts the swing sole M metres on the clock, with `--mimic-sway`, no stride.
- `--init-norm-march`: with `--init-from`, re-seats the c4 (and c6) normaliser slots to the march draw.
- `march5` link script and the `march5 robot` / `march5 refit` grade probes.
- tests: `tests/test_mimic_march.py`, `tests/test_init_norm_march.py`, `tests/test_march_probe.py`, `tests/test_sources.py`; parity block R13b.

## march5 on the prec16 student (`grade.py`, measured sole, 8 seeds, shaper 8)

| probe | falls / trips | lift L / R | note |
|---|---|---|---|
| `march5 robot`, c4 and c6 pinned (what the robot would do) | 0 / 0 of 8 | 0.05 / 0.05 cm | stands: no foot leaves the floor, drift 0.01 m, yaw 2 deg |
| `march5 refit`, pinned | 0 / 0 of 8 | 0.05 / 0.05 cm | the same |
| `march5 robot`, `MARCH_PIN=0` (raw command, ~1e6 σ in) | 8 / 8 of 8 | n/a | falls 2.6–2.9 s into the run (0.6–0.9 s after the command), chatter trip at 2.02 s |

A student that never trained on march does not march: on the robot it ignores the command (pinned), in sim fed raw it falls.
