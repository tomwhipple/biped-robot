"""Power board simulation suite (t_41bcad96).

Behavioural + circuit-level simulation of the custom power board rev A
(`pcb/power-board/`, design record `docs/design-v6/2026-10-06-power-circuit.md`,
verification record `docs/design-v6/2026-10-07-power-board-verification.md`).

The suite is a pytest package. Nothing in it talks to hardware; it is safe
to run on a laptop that has never seen the robot.

Run it:

    python -m pytest pcb/power-board/pbsim/tests/ -q

(From the repo root, with the repo venv activated. The vendored
ngspice binary is used for the device-level models via ctypes — see
`vendor/README.md`.)

Modules:

- `schematicsnapshot.py`   read-only parse of the KiCad schematic; every
  test that needs a component's numeric value reads it here so a schematic
  edit without a matching test update fails first.
- `ngsim.py`               ctypes driver for the vendored libngspice.
- `fetmodel.py`            SHA-pinned paths to the vendored TI
  CSD17556Q5B .lib.
- `pack.py`                3S LiPo behavioural model (V vs SOC, series
  resistance, balance taps).
- `bq76922.py`             behavioural BQ76922 (protections + SHUTDOWN/
  NORMAL states + OTP thresholds per §5.5).
- `tps48111.py`            behavioural TPS48111-Q1 servo-rail switch
  (EN/UVLO, INP, soft start, CB/SC trips).
- `sequencer_sim.py`       the §7 sequences: buttons + pack + BQ76922 +
  TPS48111 + ESP32 stub + loads, ticked at 1 ms.
- `board/`                 the whole board: the drawn netlist in ngspice with pin
  models of U1 and U2 (`make pbfull`; record 2026-10-24-power-board-full-simulation).
- `tests/`                 the suite proper, one file per card-bullet.
- `RESULTS.md`             per-test pass/fail and margins vs design limits
  (curated; not auto-generated).
- `vendor/`                the ngspice .so + the TI FET model + origin
  notes (committed — see vendor/README for the no-sudo rationale).
"""  # noqa: D205,D400
