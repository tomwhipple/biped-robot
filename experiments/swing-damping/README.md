# Swing-leg reaction damping (sim study)

Can the lifted leg, used as a reaction mass, damp the ~2 Hz roll rocking that starts when a
foot leaves the ground on the one-leg stand? Sim only; no firmware change, no hardware.
Result and numbers: [2026-10-25-swing-damping-study.md](2026-10-25-swing-damping-study.md).

Reuses the one-leg-stand plant, keyframes, servo model and stance PID unchanged
(`experiments/one-leg-stand/`); nothing there is modified.

| file | what |
|---|---|
| `swing_sim.py` | the sim: `run(case, ctl, swing_law)` -> metrics and a 50 Hz log; extra plant knobs (roll deadzone, per-joint kp/kd, damping, contact softness, lift-off disturbance), the swing law (`SwingHook`) |
| `hwlog.py` | parses the firmware `balance log` dumps of `hw_sessions/2026-10-25/gyrosign_serial.log` |
| `calib_search.py`, `calib_kick.py`, `calib_local.py` | the three calibration stages (random search, then the lift-off disturbance, then local refinement) |
| `calibrated.json` | the fitted plant (the base of every design run) |
| `design_search.py`, `structured.py`, `structured2.py`, `finalists.py` | swing-law search, one-term-at-a-time scans, finalists |
| `swing_eval.py`, `final_eval.py` | the robustness case set and the final tables (`2026-10-25-final-tables.txt`) |
| `fig_calibration.py` | `2026-10-25-calibration-and-swing-law.png` |

```bash
export CUDA_VISIBLE_DEVICES=
.venv/bin/python experiments/swing-damping/swing_sim.py                 # one nominal run
.venv/bin/python experiments/swing-damping/calib_search.py search --n 144 --json build/swing/calib.json
.venv/bin/python experiments/swing-damping/final_eval.py                # the tables (about 6 min on 12 cores)
```

The multiprocessing helpers use `get_context('fork')` (Python 3.14).
