# Plan B bench (issue #73)

Does raising an STS3215's position-loop P about 4× make it about 4× stiffer
without buzzing? [DESIGN.md §4](../../DESIGN.md) and §13, steps 0 and 1, and
the design record's §14 say why it matters. Everything specific to this
experiment is in this folder.

| file | what it is |
|---|---|
| `gain_bench.py` | the bench tool: `scan`, `read`, `park`, `stiffness`, `hold`, `stance`, `report`, and `--rehearse` against a simulated servo. It drives the firmware's bench CLI over the tether. The procedure is in [docs/servo-map.md](../../docs/servo-map.md) §4 |
| `plan_b_rig.py` | the rig's CAD: bracket, stiffness lever, hold lever with its lid, and stop pin. Running it writes `stl/`, redraws `figs/` and runs the fit checks |
| `plan_b_rig_v3.py` | rig v3, held on both faces: a bracket that is leg_link's two-face case grip in a closed box, and a fork lever on the horn and the idler disc. M10 bolt weights go through the fork's floor at r 40 / 70 / 100, head under the floor. Running it writes `stl/plan_b_v3_*.stl` and runs the fit checks, including the fork and weights swept from level to hanging |
| `stl/` | the rig's printable parts, written by `plan_b_rig.py` and `plan_b_rig_v3.py`; don't edit them by hand |
| `figs/` | the rig's side view and render, drawn by `plan_b_rig.py` |
| [2026-09-30-plan-b-rig.md](2026-09-30-plan-b-rig.md) | the rig: loads, parts, seating, fit checks, procedure |
| [2026-09-30-plan-b-bench.md](2026-09-30-plan-b-bench.md) | session 1: the bench servo's registers and the unloaded P ladder |
| [2026-10-01-plan-b-bench.md](2026-10-01-plan-b-bench.md) | session 2: loaded stiffness (bottles) and hold with a rigid inertia (C-clamp); friction-free k ×3.5 at P 128, ×4.8 at P 160, but P 160 limit-cycles with the clamp; P ≤ 128 / D 32 quiet at hold, sweeps shake from P 96 up; incidents |
| [2026-10-02-oscillation-research.md](2026-10-02-oscillation-research.md) | research: how STS3215 users (LeRobot, Open Duck Mini), Dynamixel humanoids and the control literature deal with the high-P limit cycle, and which remedies the registers allow |
| `2026-10-01/` | session 2's run scripts (incl. `hold_clamp.py`) and per-state / per-pass summaries |
| [2026-10-02-plan-b-bench.md](2026-10-02-plan-b-bench.md) | session 3: hold on the v3 rig (fork, both-face grip, washer weights); plumb from Tom's string; P ≤ 128 / D 32 quiet and settles, P 160 limit-cycles hanging, sweeps clean only to P 96; D 0 smoother; an accelerometer finds the ~15 Hz shaking; P 96 / D 0 shakes like stock at walking speeds; the firmware's own 50 Hz streaming is fine |
| `2026-10-02/` | session 3's run scripts (`sweep_smooth.py`, `stream50.py`, `retarget_analysis.py`) and per-pass summaries, incl. the calibration and the accelerometer check |
| `<date>/raw/` | **the raw measurements** of every run: `session.log` (every CLI line sent and received), `console*.txt` (the operator's view; its first line is the run's exact parameters), and the per-run JSON with every sampled window. Session 3's JSON includes the 500 Hz accelerometer traces. Videos and their `.pts` / `.camlog` sidecars are not in git |
| `summarise_raw.py` | rewrites the summaries the run scripts did not write (session 1, runs A and B, the 10-02 calibration and accelerometer check) from `raw/`, with `gain_bench`'s own statistics |
| [REPRODUCE.md](REPRODUCE.md) | **how to rebuild the bench and repeat all three sessions**: hardware, the two rigs, loads, firmware per session, calibration, safety rules, every run's command and parameters, and the metrics' definitions |

Its offline tests are `tests/test_gain_bench.py`. They stay in `tests/`,
because the pre-push gate collects that folder.

**Data:**
- The scripts write raw data to `hw_sessions/<date>/<run>/` (gitignored).
- After a session, copy its logs, consoles and JSON (not video) into
  `<date>/raw/<run>/` here, and write any missing summary.
- Each session's record goes here as `<date>-plan-b-bench.md`.

## Status and open items (2026-10-06)

**What the bench has shown** (details in the dated records):
- Stiffness rises with P: **3.5 ± 0.4×** at P 128 and **4.8 ± 0.6×** at P 160,
  from bottles with D = P.
- P 160 limit-cycles with a rigid inertia on both rigs.
- **The bench's best candidate is P 96 / D 0.**
  - At walking speeds it shakes about as much as stock P 32 (57–111 mg against
    52–85). The same holds through late goal changes and under the firmware's
    own 50 Hz streaming.
  - It does shake more on slow 100 steps/s sweeps.
  - Its stiffness is ≈ 2.8–3.6× (clamp probes), below the sim's 3–4× target.

**Decided:**
- Tom ordered 2 × STS3250 on 2026-10-02 for the hip rolls, the placement the
  sim needs at a 2.8× STS3215 (design record §14.4a).
- **When they arrive:** run the same v3 rig, weights, accelerometer, firmware
  and ladder (bottle stiffness, hold, sweeps A/C/fast, stream50,
  accelerometer), with an ID off the bus map (e.g. 31), for a direct
  comparison with the STS3215 data here.
- **Check first:** that the fork fits the STS3250's horn. Measure the
  accelerometer's distance from the axis and the weights' radius.

**Not yet carried into the design:**
- `DESIGN.md` §4/§13, `docs/servo-map.md` §4 and `firmware/main/servo_gains.h`
  still list the factory P/D 32/32 on every servo.
- They change when Tom picks the gain. The record of that choice should cite
  these sessions.

**Not yet measured:**
- **P 96 / D 0:** its stiffness with bottles (Tom: later), and its hold test.
  Both are carried over from D 32 or from clamp probes today.
- **The loads:** the mass of a bottle, of the washer weights and of the clamp;
  which fork holes the washers were in; the accelerometer's distance from the
  axis.
- **A true leg-sized inertia:** the salt cup, or 0.25 kg rigid at 100 mm.
- **The research note's bench questions:** hold at P 128 / D 8 and D 16; P 160
  with dead zone 2–3; register 21 written with the lock at 1 (gain scheduling).
- **Streaming:** the "matched" mode needs rework; the robot rule's 2–3° lag is
  unchecked against the sim's actuation model.
- **Unidentified:** the speed-locked 12.7 / 25.4 Hz vibration and the
  ~100–105 Hz x/z peak. The backward-sample count also drifted across
  session 3 for the same move.
- **Unexplained:** the cause of the second lever drop on 2026-10-01.

**Housekeeping:** the bench servo is still ID 30. Rename it back to 11 when
bench work on it is done.

```bash
.venv/bin/python experiments/plan-b-bench/gain_bench.py --rehearse stiffness 30 --goal 2048 --rest 3072
MUJOCO_GL=cgl .venv/bin/python experiments/plan-b-bench/plan_b_rig.py   # Mac; egl on Linux
MUJOCO_GL=cgl .venv/bin/python experiments/plan-b-bench/plan_b_rig_v3.py
```
