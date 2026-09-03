# Bimo-like Biped

A small (~34 cm, ~1.1 kg) 3D-printed **bipedal robot** built around 10× Feetech
STS3215 serial-bus servos (5 DOF per leg: hip-yaw, hip-roll, hip-pitch, knee,
ankle), developed through an LLM-assisted loop:

> parametric CAD → MuJoCo/MJX physics sim → RL policy → print → firmware → sim-to-real

Inspired by (not a clone of) the open-source [Bimo Project](https://github.com/mekion/the-bimo-project);
the geometry is anchored to servo datasheet dimensions rather than
reverse-engineered parts. Full design history, decisions, and experiment logs
live in [DESIGN.md](DESIGN.md); the running results log is
[sim/runs/night_summary.html](sim/runs/night_summary.html).

![Assembled printable design, rendered from the CAD meshes](cad/renders/assembly_mujoco.png)

**The robot exists.** It is printed, assembled, calibrated, and runs its own
firmware — a distilled policy executing on the ESP32 at 50 Hz, untethered. It
walked on 2026-09-01. Most current work is closing the remaining sim-to-real
gap (see *Open items*).

## Where it stands

- **Simulation & RL** — the training target is `sim/bimo_biped_v5body.xml`, a
  CAD-true 10-DOF MuJoCo model (mesh-derived inertia, pad-true sole contacts,
  honest STS3215 torque–speed actuators at the *measured* no-load speed,
  sub-step latency, backlash, actuation lag, IMU-realizable observations).
  Training is **brax PPO on the MJX (JAX) port** at 1–2k parallel envs on a
  GPU box, overnight; every policy is then re-graded by a CPU referee on 28
  scored scenarios × 8 seeds. How the whole thing works is written up in
  **[docs/training.md](docs/training.md)**.
- **Onboard control** — the 50 Hz policy loop runs on the ESP32 itself
  (`firmware/`), reading joint state off the servo bus and attitude off the
  onboard IMU at 250 Hz. The training net is distilled from (512,256,128) to
  (128,128) and compiled into flash as a `constexpr`; the observation layout
  and servo-ID permutation are *generated* from the sim so they cannot drift by
  hand. Currently flashed: `loco_v27tilt_b_s128r24` (41/144 under the
  bench-measured servo lag, 2.2× the policy it replaced).
- **Printable CAD** — a complete parametric part set (build123d): **7 unique
  parts, 14 prints**, support-free, ~354 g of plastic, verified by boolean
  interference checks including multi-axis worst-case poses. Pelvis v6 folded
  the tower, IMU carrier, battery tray and board frame into one printed torso,
  dropping the COM 30 mm. BOM and print settings in
  [cad/README.md](cad/README.md); assembly walkthrough in
  [docs/assembly.md](docs/assembly.md).
- **Wireless command channel** — the robot is untethered by design: since the
  policy loop is onboard, the radio carries only `(vx, yaw_rate)` intent at
  20 Hz (14 bytes), or 24-byte frames for the extended
  `crouch`/`lift`/foot channels. USB-C is for flashing only. A lost link decays
  to the zero command — a *trained* stand, not a bolted-on emergency pose —
  then releases torque after 5 s. Two consoles are built from the firmware's
  own protocol sources: the single-keystroke `bimo_tui` and the windowed
  `bimo_gui` (telemetry strip charts, a live view of the sim it is driving, and
  a **mirror mode** that draws the robot's measured joints as a ghost over the
  policy's pose). Both carry a **reset-servos** control that returns every
  joint to its calibrated zero without arming, and reaches the robot through a
  latched E-stop or a tripped fall latch. See
  [docs/control-channel.md](docs/control-channel.md) and
  [docs/mirror-mode.md](docs/mirror-mode.md).
- **Software-in-the-loop** — `sim/sil/` links the *real* firmware control code
  against the simulated plant and scores it with the same referee, so byte
  order, calibration, the servo permutation and activation-function mismatches
  are red tests on a laptop instead of a robot walking into a wall. See
  [docs/sil-harness.md](docs/sil-harness.md).
- **Bench tooling** — `tools/` drives the real robot for measurement, not just
  demos: joint-play sweeps, ROM sweeps, servo speed and frequency response,
  gyro yaw probes, observation capture off the armed robot. What it measures
  goes back into the sim as a plant term (see *the hardware contract* in
  docs/training.md).
- **Open items** — the armed robot holds a quiet stand for about a second, then
  grows alternating hip-pitch swings at 1–2 Hz until it topples (2.4–6 s), while
  the same policy stands 8/8 in the referee and dead still in the SIL twin. Two
  suspects under investigation: ~3° of *free* hip-roll play, and the pitch gyro
  (27°/s raw zero-rate, 2–2.7× scale disagreement with the accelerometer on one
  move test). Natural rhythm under the measured servo lag is unsolved. Get-up is
  parked by decision. The policy is terrain-blind (no height scan), and there is
  no odometry, so goal-seeking is fed ground truth in sim only —
  waypoint-conditioned locomotion is the next abstraction.

## Quickstart

```bash
# once: venv + deps (direnv auto-activates via .envrc)
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

### Simulation and training

```bash
cd sim
python smoke_test.py                                   # env sanity check

# grade a trained policy: 28 scenarios x 8 seeds -> runs/<run>/scorecard.{md,json}
python mjx/eval_precision.py --run-name <run>                    # python column
python mjx/eval_precision.py --run-name <run> --act-lag-hz 2.0   # measured servo lag
python mjx/eval_precision.py --run-name <run> --sil              # the real C++ code
python mjx/eval_ref.py --run <run> --video                       # quick look

# train (GPU box; the night queue in infra/night/ does this unattended)
python mjx/train_mjx.py --out loco_v28 --precision --family loco \
    --steps 60_000_000 --envs 1024 --init-from loco_v27tilt_b

# shrink for the firmware, then export
python mjx/distill_student.py --teacher loco_v28 --out loco_v28_s128 --rounds 24
cd .. && python tools/export_policy_weights.py --run loco_v28_s128
python tools/gen_policy_weights.py --run loco_v28_s128     # -> policy/weights.h
python tools/gen_obs_spec.py      --run loco_v28_s128      # -> obs/obs_spec.h

python sim/build_report.py             # rebuild the visual training log
python -m pytest tests/ -q             # protocol, watchdog, ROM, referee parity
```

Full explanation of the pipeline: **[docs/training.md](docs/training.md)**.

### Driving a policy (sim twin or real robot)

```bash
# two shells; ctrl-C to stop. --host 127.0.0.1 talks to the sim twin,
# a robot's IP talks to the robot.
python sim/udp_agent.py --run-name <run> --render
make -C firmware/host tui && firmware/host/build/bimo_tui --host 127.0.0.1
#   [a] arm, hold arrows to walk/turn, [space] stand, [h] reset servos,
#   [e] e-stop, [q] quit

# ... or the windowed console, which starts and displays the sim itself:
brew install glfw && make -C firmware/host deps gui
firmware/host/build/bimo_gui --host 127.0.0.1
firmware/host/build/bimo_gui --host <robot-ip> --mirror   # ghost the real robot
firmware/host/build/bimo_gui --host <robot-ip> --readonly # watch, don't drive

python link/commander.py --host 127.0.0.1 --source gamepad
python link/commander.py --host 127.0.0.1 --source script --script square
```

### Firmware

```bash
make -C firmware/host test          # the gate: pure modules under ASan/UBSan
. ~/esp/esp-idf/export.sh           # (not inside the venv -- unset VIRTUAL_ENV)
idf.py -C firmware build flash monitor
```

See [firmware/README.md](firmware/README.md).

### CAD

```bash
python cad/parts.py                     # STLs + STEPs + bed-fit/mass checks
python cad/check_assembly.py            # joint-sweep interference checks
python cad/export_assembly.py           # assembled-robot STEP
python cad/render_assembly.py           # assembly PNG (used by the report)
python cad/freecad_articulate.py        # poseable 10-joint FreeCAD assembly
python cad/render_assembly_steps.py     # refresh docs/assembly.md figures
```

Trained policies live under `sim/runs/<run-name>/` (gitignored — local only):
`params.pkl`, `config.json`, TensorBoard logs, scorecards, and rendered reels.

## Directory structure

```
robot/
├── README.md                     # you are here
├── DESIGN.md                     # working doc: decisions, experiments, results
├── requirements.txt              # pinned Python deps (MuJoCo, brax/JAX, build123d, ...)
├── docs/
│   ├── training.md               # HOW THE TRAINING APPROACH WORKS (start here)
│   ├── controls-and-training-overview.md  # what the control system is
│   ├── control-channel.md        # wireless command link: protocol, failsafe
│   ├── firmware-design.md        # what runs on the ESP32, and why
│   ├── sil-harness.md            # real control code vs. the simulated plant
│   ├── servo-map.md              # bus ID -> joint, zeros, direction signs
│   ├── wiring.md                 # circuit + block diagrams, bring-up
│   ├── assembly.md               # assembly walkthrough with rendered figures
│   ├── mirror-mode.md            # ghosting the real robot in the sim viewer
│   ├── precision-curriculum.md   # where the scored skill list came from
│   ├── training-plan-v2.md       # the prior-art survey behind the recipe
│   ├── hip-yaw-study.md          # the A/B that made this a 10-DOF robot
│   ├── bom-sourced.md            # sourced BOM: live links & prices
│   └── ...                       # bring-up, slicing, sensors, status reports
├── sim/                          # MuJoCo + RL
│   ├── bimo_biped_v5body.xml     # the plant (10 DOF, CAD-true) -- current target
│   ├── bimo_biped_v*.xml         # its ancestors, kept for legacy referees
│   ├── walker_env.py             # CPU Gymnasium env: THE REFEREE plant
│   ├── sil_twin.py               # the plant with firmware-in-the-loop knobs
│   ├── udp_agent.py              # sim twin of the on-robot command listener
│   ├── build_report.py           # rebuild runs/night_summary.html (visual log)
│   ├── mjx/                      # the GPU training stack
│   │   ├── env_mjx.py            # JAX port of the env: physics, DR, reward
│   │   ├── train_mjx.py          # brax PPO trainer
│   │   ├── eval_precision.py     # THE REFEREE: 28 scenarios x 8 seeds
│   │   ├── eval_ref.py           # quick CPU replay + video
│   │   ├── distill_student.py    # DAgger: big teacher -> (128,128) student
│   │   └── parity_test.py        # MJX vs CPU physics/reward parity gate
│   ├── sil/                      # software-in-the-loop harness + golden vectors
│   └── runs/                     # (gitignored) policies, scorecards, reels
│       └── night_summary.html    # the visual results log (committed)
├── firmware/                     # ESP32 target: the 50 Hz policy loop
│   ├── main/                     # app, control task, CLI, link, IMU sampler
│   ├── components/               # linkproto, obs, policy, scsbus, imu, battguard
│   └── host/                     # host build + unit tests (the gate) + consoles
├── link/                         # the command channel (laptop side)
│   ├── protocol.py               # wire format + Watchdog (firmware reference)
│   ├── sources.py                # command sources: script / gamepad / goal-seeker
│   ├── commander.py              # streams intent to the robot over UDP
│   ├── tui.cpp / gui.cpp         # the two consoles
│   └── *_probe.py                # bench probes driven over the radio
├── tools/                        # sim<->firmware exporters and bench instruments
│   ├── gen_obs_spec.py           # -> firmware obs_spec.h (generated contract)
│   ├── gen_policy_weights.py     # -> firmware weights.h + golden vectors
│   ├── export_policy_weights.py  # params.pkl -> .silw blob
│   └── joint_sweep.py, yaw_probe.py, servo_frf.py, ...   # measure the robot
├── cad/                          # parametric CAD (code is the source of truth)
│   ├── dimensions.py             # every dimension, incl. measured STS3215 data
│   ├── parts.py                  # the 7 printable parts -> stl/ + mass/bed checks
│   ├── check_assembly.py         # boolean interference checks across joint ranges
│   ├── freecad_articulate.py     # poseable FreeCAD assembly (10 revolute joints)
│   ├── PRINT_LIST.md             # what to print, in what orientation
│   └── stl/ step/ renders/       # generated outputs
├── infra/                        # training-box automation
│   ├── night/                    # the nightly job queue runner (cron, 22:00-07:00)
│   └── runpod/                    # burst GPU compute
├── tests/                        # pytest: protocol, ROM, referee parity, consoles
└── hw_sessions/                  # (gitignored) bench session logs and CSVs
```

## Hardware

10× Feetech STS3215 (12 V version, ~55 g, ~30 kg·cm) · Waveshare **General
Driver for Robots** (ESP32, onboard QMI8658C IMU) · 3S 850 mAh LiPo (11.1 V) ·
PETG prints + silicone sole pads · M3 heat-set inserts. Servo bus, IMU and
power details in [docs/wiring.md](docs/wiring.md); bus-ID-to-joint map and
calibration zeros in [docs/servo-map.md](docs/servo-map.md); ordering in
[docs/bom-sourced.md](docs/bom-sourced.md).
