# Plan B bench (issue #73)

Does raising an STS3215's position-loop P about 4× make it about 4× stiffer
without buzzing? [DESIGN.md §4](../../DESIGN.md) and §13, steps 0 and 1, and
the design record's §14 say why it matters. Everything specific to this
experiment is in this folder.

| file | what it is |
|---|---|
| `gain_bench.py` | the bench tool: `scan`, `read`, `park`, `stiffness`, `hold`, `stance`, `report`, and `--rehearse` against a simulated servo. It drives the firmware's bench CLI over the tether. The procedure is in [docs/servo-map.md](../../docs/servo-map.md) §4 |
| `plan_b_rig.py` | the rig's CAD: bracket, stiffness lever, hold lever with its lid, and stop pin. Running it writes `stl/`, redraws `figs/` and runs the fit checks |
| `stl/` | the rig's printable parts, written by `plan_b_rig.py`; don't edit them by hand |
| `figs/` | the rig's side view and render, drawn by `plan_b_rig.py` |
| [2026-09-30-plan-b-rig.md](2026-09-30-plan-b-rig.md) | the rig: loads, parts, seating, fit checks, procedure |
| [2026-09-30-plan-b-bench.md](2026-09-30-plan-b-bench.md) | session 1: the bench servo's registers and the unloaded P ladder |
| [2026-10-01-plan-b-bench.md](2026-10-01-plan-b-bench.md) | session 2: loaded stiffness (bottles) and hold with a rigid inertia (C-clamp); friction-free k ×3.5 at P 128, ×4.8 at P 160, but P 160 limit-cycles with the clamp; P ≤ 128 / D 32 quiet at hold, sweeps shake from P 96 up; incidents |
| `2026-10-01/` | session 2's run scripts (incl. `hold_clamp.py`) and per-state / per-pass summaries |

Its offline tests are `tests/test_gain_bench.py`. They stay in `tests/`,
because the pre-push gate collects that folder. Raw bench data goes to
`hw_sessions/` (gitignored); each session's record goes here as
`<date>-plan-b-bench.md`.

```bash
.venv/bin/python experiments/plan-b-bench/gain_bench.py --rehearse stiffness 30 --goal 2048 --rest 3072
MUJOCO_GL=cgl .venv/bin/python experiments/plan-b-bench/plan_b_rig.py   # Mac; egl on Linux
```
