# Bimo-like Biped — Design & Simulation Working Doc

**Status:** CAD massing model + MuJoCo physics validation complete (Stage 0–1). Not yet printable; RL and sim-to-real not started.
**Last updated:** 2026-07-09
**Owner:** Tom

---

## 1. Goal

Build a custom, small (~34 cm), 3D-printed **bipedal robot** using an LLM-assisted
loop: parametric CAD → physics simulation → print → RL walking policy → sim-to-real.
Inspired by (not a clone of) the open-source **Bimo Project**.

This folder holds a first-pass **parametric CAD massing model** and a **MuJoCo
physics model** used to validate the mechanism *before* committing to printed parts.

## 2. Key decisions & rationale

- **Reference platform:** The Bimo Project by Mekion — `github.com/mekion/the-bimo-project`,
  a $500 fully-3D-printable biped shipping with an NVIDIA Isaac Lab RL env and working
  sim-to-real. As of 2026-07 it is pre-order / early access: CAD files "coming soon",
  and its custom RP2040 controller board is the one non-COTS component.
- **Strategy: do NOT reverse-engineer Bimo's exact parts from photos/video.** Instead
  design a *similar* platform around **known component dimensions**. For a servo robot
  the servos are the skeleton, so the datasheet — not the video — anchors the geometry.
- **Actuator:** 8× **Feetech STS3215** serial-bus servo. Measured **45.2 × 24.6 × 35.1 mm**,
  ~55 g, ~30 kg·cm (**2.94 N·m**) stall @ 12 V. Same class as LeRobot/SO-ARM, easy to source.
- **DOF layout:** 4 per leg = **hip-roll (X), hip-pitch (Y), knee (Y), ankle (Y)** → 8 total.
- **CAD tool:** parametric **OpenSCAD** (render-in-the-loop). Chosen because it's text/
  parametric, LLM-friendly, and headless-renderable.
- **Simulator:** **MuJoCo** (not Isaac Lab). MuJoCo is CPU/laptop-friendly, `pip install`,
  gold-standard contact physics, and is the universal validation layer even for Isaac users.
  Isaac Lab needs a workstation NVIDIA GPU + 1–2 week setup; overkill at this scale.
  For GPU-parallel RL later, use **MuJoCo MJX** (JAX) on the RTX 4070. Genesis is emerging
  but bleeding-edge.
- **User hardware:** RTX 4070 (12 GB) + Mac laptop. Mac runs MuJoCo's native viewer for
  design; 4070 runs MJX training.

## 3. Folder contents

```
robot/
├── DESIGN.md                 # this file
├── cad/
│   ├── bimo_like_biped.scad  # parametric OpenSCAD massing model (8 servos, articulated)
│   └── renders/              # v2_hero / v2_front / v2_side / v2_stride .png
└── sim/
    ├── bimo_biped.xml        # MuJoCo MJCF physics model (matches the CAD layout)
    ├── sim_biped.py          # headless sim: coordinated squat + diagnostics + render
    └── renders/              # mj_squat.gif, mj_deep.png, mj_stand.png
```

## 4. Current state — what works

- **CAD (Stage 0, done):** massing model renders correctly; proportions and the 8-servo
  layout are established. Servo bodies double as the limb segments; gray brackets connect
  them; blue box = SBC head cavity. **This is a concept, NOT printable parts** — no screw
  bosses, heat-set seats, horn-spline interfaces, wire channels, or tolerances.
- **Sim (Stage 1, done):** the physics model **stands** under gravity (settles ~2 mm,
  perfectly upright) and completes a **coordinated squat while staying balanced** — all
  eight joints actuated, uprightness stayed 1.000, recovered to standing. Mechanism is
  physically sane. Verified numerically and by rendering.

## 5. How to run

### OpenSCAD (CAD renders)
```bash
# headless render (add xvfb-run on a server); pose override via -D
xvfb-run -a openscad -o out.png --preview --projection=perspective \
  --imgsize=900,1100 --camera=0,0,0,62,0,25,0 --viewall --autocenter \
  -D 'gait=24' -D 'p_knee=-16' cad/bimo_like_biped.scad
```
Camera note (learned the hard way): `--camera=…,90,0,0,…` = **side** view, `…,90,0,90,…`
= **front** view. Use `--preview` (not `--render`) or all `color()` flattens to gold.

### MuJoCo (physics sim)
```bash
pip install mujoco imageio
MUJOCO_GL=osmesa python3 sim/sim_biped.py        # headless (Linux server)
# On the Mac (native GL, interactive) — drag joints, push the robot:
python -m mujoco.viewer                           # then open sim/bimo_biped.xml
```

## 6. Design parameters (source of truth)

| Param | Value |
|---|---|
| Servo (STS3215) | 45.2 × 24.6 × 35.1 mm, 55 g, 2.94 N·m stall |
| Joint limits | hip-roll ±25°, hip-pitch ±60°, knee −95..+5°, ankle ±40° |
| Segment lengths | thigh ≈ 90 mm, shin ≈ 90 mm (servo + bracket) |
| Torso (D×W×H) | 46 × 104 × 72 mm; head 46 × 62 × 42 mm |
| Hip separation | 56 mm (leg center-to-center) |
| Standing height | torso center ≈ 0.28 m; overall ≈ 0.34 m |
| Total sim mass | ≈ 0.93 kg |

MJCF specifics: position actuators, kp=40, forcerange ±3 N·m; foot-floor friction 1.0;
internal parts set non-colliding (feet-only contact); IMU `site` on torso for orientation.

## 7. Roadmap

- **Stage 0 — CAD massing:** DONE (concept only).
- **Stage 1 — Physics validation (MuJoCo):** DONE.
- **Stage 2 — RL walking:** NEXT. Gymnasium env + PPO via MJX on the RTX 4070.
- **Stage 3 — Sim-to-real:** system-ID the real servos, export ONNX policy, deploy on RP2040/SBC.

## 8. TODO — known next steps (start here)

- [ ] **Stage 2a — Build the Gymnasium env** around `sim/bimo_biped.xml`.
      Obs: joint qpos + qvel, torso quaternion/up-vector (IMU site), optional foot contacts.
      Action: 8 target joint angles. Reward: forward velocity + upright + alive
      − energy − action-rate. Terminate on torso height < 0.18 m or large tilt.
- [ ] **Stage 2b — Port to MJX (JAX)** and train PPO (Brax PPO or RSL-RL) on the RTX 4070
      with a few thousand parallel envs. Target: a stable forward gait.
- [ ] **Domain randomization** (required for sim-to-real): friction, link mass ±15%,
      actuator latency, random shove forces.
- [ ] **Swap box geoms for real CAD meshes** — export STL from the parametric parts,
      load as MJCF collision/visual meshes, for accurate inertia and contact.
- [ ] **Realistic actuator model** — add velocity limit + backlash and match the STS3215
      torque/speed curve (currently an ideal position servo capped at ±3 N·m).
- [ ] **Measure & set real masses/inertias** once physical parts exist.
- [ ] **CAD → printable:** take ONE joint (e.g. the knee bracket joining two servos) from
      massing block to a real printable STEP/STL with horn spline, heat-set bosses, and
      clearances; test-print to dial in tolerance (~0.3–0.4 mm on moving fits).
- [ ] **Verify joint axis directions/signs** against a real assembly before trusting any gait.
- [ ] **Sim-to-real plan:** system-identification protocol for the STS3215 (torque, speed,
      latency), ONNX export, and deploy pipeline to the RP2040 + SBC.

### Known limitations / caveats
- Massing CAD ≠ printable geometry.
- Sim inertias are approximate (uniform-density boxes, not meshes).
- No self-collision between internal parts (feet-only contact) — revisit when adding meshes.
- Actuator is idealized; real STS3215 has finite speed, latency, and backlash.

## 9. References

- Bimo Project — `github.com/mekion/the-bimo-project`, `mekion.com/project`
- STS3215 dimensions — servodatabase.com / waveshare.com (ST3215)
- Simulator choice — roboticscenter.ai (MuJoCo vs Isaac Sim, Best RL sims 2026)
