# The legs-only body and its roll compliance (2026-10-24)

A dated record of what the bench measured on 2026-10-24 and how the simulator,
the nightly training and the referee were changed for it. Status of the work
that uses it: issue #137 (get-up) and the one-leg / short-walk runs of the same
night ([getup-campaign-2026-10.md](getup-campaign-2026-10.md), night 4). The
current design lives in [training.md](../training.md) §2, §6, §8, §13.

## What the bench measured

**The body as built is the v6 legs only.** Twelve leg servos (hip yaw, hip
roll, hip pitch, knee, ankle, ankle roll per leg; the hip rolls are STS3250,
the other ten STS3215). The torso frame carries the control board (ESP32 driver
board) and nothing else: no battery pack (tethered), no Pi, no head, no neck
servo, no arms.

**Roll compliance.** One foot clamped to the desk, the other lifted 30 mm
straight up with hip pitch, knee and ankle only (no weight shift): the torso
tilts toward the lifted side by 5.2° (right foot lifted) / 5.0° (left foot
lifted), steady. Only about 1° of that shows at the encoders: the stance
ankle roll moved 12–17 ticks (1.0–1.5°) at 10–14 % load, the stance hip roll
8–10 ticks (0.7–0.9°). The rest is play and compliance beyond the encoders.

**Lateral asymmetry.** The same model weight shift (53 mm) balanced about 3°
better on one stance foot than on the other.

**The stored IMU mount is from the old body.** The hip-roll STS3250s chatter
when lightly loaded (stick-slip, dead zone 0) but are clean under body load;
this is not modelled.

## The as-built plant

`sim/bimo_biped_v6legs.xml`, built by `make plant`
(`sim/build_v6_inertia.py --asbuilt-legs --write`), the same CAD-true
inertials as the committed 17-servo plant (pelvis_v8 + torso_v8 + front cover; the power cover and battery door are left off with the pack, an assumption). `build_v6_inertia.asbuilt_legs()`
is the `DesignParams` that differ from `gen_plant_v6.DesignParams()`:

| `DesignParams` field | default | as built |
|---|---|---|
| `head` (new switch: no neck servo, neck joint/actuator, head, camera) | `True` | `False` |
| `m_battery` | 0.170 kg | 0 |
| `m_pi4` | 0.066 kg | 0 |
| `m_power` | 0.060 kg | 0 |
| `m_neck_servo` | 0.055 kg | 0 |
| `m_head` | 0.039 kg | 0 |
| `arms` | `False` (the committed plant adds them through `asdrawn()`) | `False`, and `plant_xml(arms=False)` |

A mass of 0 leaves the pack, Pi and power module out of the MJCF and out of the
CAD inertial sums (`build_v6_inertia.bodies`); `head=False` also drops the neck
servo, the neck floor, the neck collar and the head body. Everything else is
the committed plant's: the pelvis print (`pelvis_v7_armless`), the bearing
housing and halves, the two yaw servos, the control board (`V.GD_MASS`), the
wiring lump (`m_wiring` 30 g, kept: an estimate that includes the tether
leads), 12 leg servos at their places.

| | total | torso |
|---|---|---|
| `bimo_biped_v6ar.xml` (as drawn: legs, neck, head, arms) | 2.375 kg | 1.041 kg |
| armless variant (`--armless`) | 1.960 kg | 0.875 kg |
| **`bimo_biped_v6legs.xml` (as built)** | **1.501 kg** | **0.4545 kg** |

All three are CAD / catalogue masses, **not weighed**. The legs-only torso is
the pelvis print, housing, bearings, two yaw servos (110 g), the board (30 g)
and the wiring lump (30 g): an estimate whose uncertainty is large relative to
its size, so the torso mass is randomised ±40 % in training.

## Calibrating the roll play

`sim/roll_compliance.py` reproduces the bench test in the CPU plant: the stance
foot welded to the world (the clamp), the lifted foot's body solved by IK to
rise 30 mm with its orientation and x/y unchanged, joint-space targets ramped
over 1 s and held 3 s; the steady torso tilt toward the lifted side (true
attitude) and the stance roll encoders' error (target − shaft, i.e. what an
encoder on the servo side of the play reads) are averaged over the last second.
Servo set: the robot's (`robot` stiffness preset, STS3250 hip rolls), joint
friction 0.35 N·m, no latency, no DR, one run per lifted side.

| model | L lifted: tilt (ankle / hip encoder) | R lifted: tilt (ankle / hip encoder) | foot rise achieved |
|---|---|---|---|
| bench | 5.0° (1.0–1.5° / 0.7–0.9°) | 5.2° (1.0–1.5° / 0.7–0.9°) | 30 mm commanded |
| **before**: rigid rolls, no backlash | 2.37° (1.26° / 0.74°) | 2.35° (1.25° / 0.74°) | 26.4 mm |
| **before**: the referee's pinned 0.7° backlash on every joint | 3.13° (1.30° / 0.74°) | 3.11° (1.29° / 0.74°) | 25.2 mm |
| 3° free play on the roll joints (`play_deg`, the old assumption) | 5.63° | 5.62° | 22.2 mm |
| **after**: 0.7° backlash + 1.9° extra roll deadzone | **5.19°** (1.41° / 0.72°) | **5.18°** (1.40° / 0.72°) | 22.5 mm |
| the same with the 17-servo as-drawn plant (2.38 kg) | 7.43° (2.78° / 1.26°) | 7.36° (2.74° / 1.24°) | |

- **The encoders already match without any play.** The rigid-roll plant puts
  the stance ankle-roll encoder at 1.25–1.3° and the hip roll at 0.74° at the
  bench's load; the servo model (P × 2.8 / × 4, joint friction 0.35) is about
  right at the encoder. The 17-servo plant, 55 % heavier, would put the ankle
  encoder at 2.8°: another sign that the legs-only plant is the body that was
  on the desk. What the rigid plant lacks is the tilt: 3.1° against 5.0–5.2°.
- **Play adds one degree of tilt per degree of deadzone**, and does not move the
  encoders (1.4–1.5° for 0–2.6° of extra deadzone), the bench's signature. The
  walker_env shaft/link play model (`play_deg`) and the PD-error deadzone
  (`roll_lash_deg`) give the same steady tilt for the same width (5.63° with 3° of play vs ≈ 5.6° with 3° of deadzone).
- **Calibrated value: a total roll-joint deadzone of ≈ 2.6°** (peak to peak)
  per episode. The rolls already carry the global backlash (0.5–1.0° in
  training, 0.7° in the referee), so the extra is **U(1.0, 2.6)°** per episode
  (mean 1.8°): the total spans 1.5–3.6° in training (tilt 4.0–6.3°) and
  1.7–3.3° in the referee (4.2–5.9°), centred on the measurement.
- The calibration has one lumped width for the four roll joints (hip and ankle
  rolls, both sides); the bench cannot split it between the two joints in
  series. The L / R difference measured on the bench (5.0° vs 5.2°) is inside
  the model's spread and is not modelled as an asymmetry.

**Not reproduced:** the sim's observation of a roll joint is the link angle
(plus the deadzone), where the hardware encoder reads the shaft, so the
policy sees up to half the play more than the encoder does. At 1.3° of
deadzone per side the mismatch is below the zero-offset and IMU-mount draws
already in training, and the tilt (the IMU) is right. Modelling the shaft in
the observation is open (training.md §13).

## Domain randomisation added for the legs-only runs

| knob | value | basis |
|---|---|---|
| `--roll-lash-deg 1.0,2.6` | extra roll-joint deadzone per episode, degrees | the calibration above |
| `--torso-mass-range 0.4` | torso mass and inertia × U(0.6, 1.4) (replaces the generic ±15 % for the torso) | the as-built torso mass is an unweighed estimate (instruction: ±10 %, ±40 % if uncertain) |
| `--torso-com-y 0.015` | lateral torso CoM shifted U(−15, +15) mm (`body_ipos` y) | one stance foot balanced ~3° better than the other for the same weight shift |
| `--imu-mount-deg 3` | IMU mounting misalignment σ 3° (was a fixed 2°), a rotation about a random axis applied to the observed up-vector | the stored mount is from the old body |

All four are draws in the MJX env (`domain_randomize` for the torso, the
per-episode `lash` vector for the rolls) and in the CPU env, and are recorded
in `config.json`, from which the referee inherits them; a run that does not
set them is unchanged.

## Measured vs assumed

- **Measured (today):** the body composition, the 30 mm lift tilts (5.2° /
  5.0°) and the stance encoder movements, the 3° balance asymmetry.
- **Assumed:** the part masses (CAD and catalogue, wiring 30 g); that the tilt
  is play at the stance roll joints rather than flex elsewhere (the model
  cannot tell); one width for all four roll joints; the lateral CoM and IMU
  bands (round numbers around the observations, not fitted).
- **Not modelled:** the hip-roll chatter at light load; the tether's
  stiffness and pull.
