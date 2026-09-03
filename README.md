# Bimo-like Biped

A 34 cm, ~1.1 kg 3D-printed **bipedal robot** — 10 serial-bus servos, an ESP32,
and a reinforcement-learning policy that runs onboard at 50 Hz and decides every
joint angle from what the robot can actually sense. No gait generator, no
scripted walk.

![Assembled robot, rendered from the CAD meshes](cad/renders/assembly_mujoco.png)

**Status:** built, calibrated, and walking under its own firmware since
2026-09-01. Training is brax PPO on a MuJoCo MJX model of the robot; the trained
network is distilled small enough to live in the ESP32's flash. The open problem
is that an armed robot holds a stand for about a second and then oscillates
itself over, where the same policy stands still in simulation. One cause is
found and fixed — the pitch gyro over-read by 18 %, a phantom velocity on
exactly the axis it oscillates in — and the re-arm test under the corrected
sensor is pending.

*Inspired by (not a clone of) the open-source
[Bimo Project](https://github.com/mekion/the-bimo-project). The geometry here is
built from servo datasheet dimensions rather than reverse-engineered parts.*

## Try it

Everything below runs on a laptop with no robot and no GPU.

```bash
python -m venv .venv && source .venv/bin/activate    # direnv does this via .envrc
pip install -r requirements.txt

# look at the robot: drag its joints, push it over
python -m mujoco.viewer --mjcf=sim/bimo_biped_v5body.xml

# the test gates
python -m pytest tests/ -q          # ~3 min: physics parity, protocol, ROM, referee
make -C firmware/host test          # the firmware's pure modules under ASan/UBSan

# regenerate the printable parts from the parametric source
python cad/parts.py                 # STLs + STEPs + bed-fit and mass checks
python cad/check_assembly.py        # must print ALL CLEAR
```

**Trained policies are not in the repo.** They are large local artifacts under
`sim/runs/` (gitignored). To get one you train it — see
[docs/training.md](docs/training.md), which walks the whole pipeline from the
physics model to the weights compiled into the firmware. A full training run
wants an NVIDIA GPU; the referee, the distillation and every export step run on
a laptop.

With a policy in hand, this drives it in a simulated robot over the real command
protocol, and `--host <robot-ip>` drives a real one instead:

```bash
python sim/udp_agent.py --run-name <run> --render
brew install glfw && make -C firmware/host deps gui       # the windowed console
firmware/host/build/bimo_gui --host 127.0.0.1
#   pick a run, Start, [a] arm, arrows to walk, PgUp/PgDn to turn
```

## Where to read next

| | |
|---|---|
| [docs/training.md](docs/training.md) | **how the training approach works** — plant, rewards, domain randomization, the referee, distillation, deployment |
| [docs/controls-and-training-overview.md](docs/controls-and-training-overview.md) | what the control system *is*, in one map |
| [docs/control-channel.md](docs/control-channel.md) | the wireless command protocol and its failsafe |
| [firmware/README.md](firmware/README.md) | what runs on the ESP32 |
| [cad/README.md](cad/README.md) · [docs/assembly.md](docs/assembly.md) | the printable part set, BOM, and how to build one |
| [docs/wiring.md](docs/wiring.md) · [docs/servo-map.md](docs/servo-map.md) | electrical, servo bus IDs, calibration |
| [sim/runs/night_summary.html](sim/runs/night_summary.html) | the running results log, with gait reels |
| [AGENTS.md](AGENTS.md) | working practices for this repo: gates, branching, bench safety |
| [DESIGN.md](DESIGN.md) | the full design record and dated experiment log |

## What's in here

```
sim/        the MuJoCo plant, the CPU env (referee), and sim/mjx/ (GPU training)
firmware/   the ESP32 target: the 50 Hz policy loop, and its host test build
link/       the command channel — wire protocol, consoles, bench probes
tools/      sim <-> firmware exporters, and instruments that measure the robot
cad/        parametric CAD (build123d); cad/dimensions.py is the source of truth
docs/       design docs, protocol specs, bring-up and assembly guides
```

## Hardware

10× Feetech STS3215 servos (12 V) · Waveshare General Driver for Robots (ESP32,
onboard IMU) · 3S 850 mAh LiPo · PETG prints + silicone sole pads · M3 heat-set
inserts. Sourcing in [docs/bom-sourced.md](docs/bom-sourced.md).
