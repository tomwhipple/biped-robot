# Servo address map

Bus address (ST3215 servo ID) → physical joint on the robot. Ten servos on one
half-duplex TTL chain at 1 Mbaud off the Waveshare ESP32 driver board.

Generated-source of truth: `firmware/components/obs/include/obs/obs_spec.h`
(`kJointNames` / `kServoId`), which `tools/gen_obs_spec.py` emits from
`sim/bimo_biped_v3yaw.xml` and the deployed run config. The host tests
(`firmware/host/test_obs.cpp`) pin the permutation, so this table and the
firmware cannot drift by hand.

## By servo address

| Servo ID | Part location   | Sim joint     | Action index | Bus port  | ROM        | Axis            |
| -------- | --------------- | ------------- | ------------ | --------- | ---------- | --------------- |
| 1        | left hip roll   | `L_hip_roll`  | 1            | A (left)  | −25°…+25°  | X (roll)        |
| 2        | left hip pitch  | `L_hip_pitch` | 2            | A (left)  | −110°…+60° | Y (pitch)       |
| 3        | left knee       | `L_knee`      | 3            | A (left)  | −95°…+5°   | Y (pitch)       |
| 4        | left ankle      | `L_ankle`     | 4            | A (left)  | −40°…+40°  | Y (pitch)       |
| 5        | right hip roll  | `R_hip_roll`  | 6            | B (right) | −25°…+25°  | X (roll)        |
| 6        | right hip pitch | `R_hip_pitch` | 7            | B (right) | −110°…+60° | Y (pitch)       |
| 7        | right knee      | `R_knee`      | 8            | B (right) | −95°…+5°   | Y (pitch)       |
| 8        | right ankle     | `R_ankle`     | 9            | B (right) | −40°…+40°  | Y (pitch)       |
| 9        | **right** hip yaw | `R_hip_yaw` | 5            | B (right) | −45°…+45°  | Z (yaw)         |
| 10       | **left** hip yaw  | `L_hip_yaw` | 0            | A (left)  | −45°…+45°  | Z (yaw)         |

"Left" is the robot's own left: +Y in the plant, with the robot facing +X.

> **Hip-yaw assembly errata, 2026-08-02.** The two yaw servos went in swapped:
> the unit numbered **9 is bolted into the right hip** and **10 into the left**,
> the reverse of the original plan. Rather than pull them, the firmware map
> absorbed it (`L_hip_yaw → 10`, `R_hip_yaw → 9`; commit `eb062cf`). The rows
> above are the as-built truth. Every other ID is as originally assigned.

## By chain position

The board's two bus ports are electrically the same bus — they are split one
per leg purely for cable routing. Each leg chains outward from the pelvis:

- **Port A → left leg**: ID **10** hip yaw → ID **1** hip roll → ID **2** hip
  pitch → ID **3** knee → ID **4** ankle
- **Port B → right leg**: ID **9** hip yaw → ID **5** hip roll → ID **6** hip
  pitch → ID **7** knee → ID **8** ankle

The yaw servos carry IDs 9/10 rather than 1/2 because they were added to the
design (v3yaw, 2026-07-23) after IDs 1–8 had already been assigned to the
8-DOF plant. This is why the address column is not in chain order.

## Policy action order ≠ bus ID order

The policy's action vector is in sim actuator order:

```
[L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle,
 R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle]
```

so **action 0 drives bus ID 10**, not ID 1. The firmware applies the
permutation `obs::kServoId = {10, 1, 2, 3, 4, 9, 5, 6, 7, 8}` — index by
action, read the bus ID. (Pre-errata this was `{9, 1, …, 10, …}`; see the note
above.)

## As-built zero calibration

Measured 2026-08-02 on the assembled robot, held at the standing pose on a test
stand, via `cal zero` + `cal save`. Standing **is** angle zero for every joint
(`kJointDefault` is all zeros), so these ticks are each joint's `zero_steps`.
They are stored in NVS; the boot log reads `cal: restored from NVS`.

| Joint | ID | zero (ticks) | Δ from 2048 | dir |
| ----- | -- | ------------ | ----------- | --- |
| `L_hip_yaw`   | 10 | 1693 | −355  | +1 ⚠ |
| `L_hip_roll`  | 1  | 3532 | +1484 | +1 ⚠ |
| `L_hip_pitch` | 2  | 2503 | +455  | +1 ⚠ |
| `L_knee`      | 3  | 2048 | 0     | +1 ⚠ | 
| `L_ankle`     | 4  | 3452 | +1404 | +1 ⚠ |
| `R_hip_yaw`   | 9  | 1801 | −247  | +1 ⚠ |
| `R_hip_roll`  | 5  | 2420 | +372  | +1 ⚠ |
| `R_hip_pitch` | 6  | 3279 | +1231 | +1 ⚠ |
| `R_knee`      | 7  | 1636 | −412  | +1 ⚠ |
| `R_ankle`     | 8  | 3520 | +1472 | +1 ⚠ |

The large offsets are ordinary horn clocking — the ST3215 horn seats on discrete
splines, so a mechanical zero is never exact and `middle` was never run at the
CAD-neutral pose. **Before this was measured the firmware still believed zero
was 2048 for all ten**, which would have driven the left knee 168° on the first
`run`.

⚠ **`dir` is unverified.** All ten are at the +1 default. The sign is a physical
convention and cannot be read off the encoder, which only reports the servo's
own frame — confirming it needs someone watching each joint move. Do not trust
a gait until it is checked (`docs/assembly.md` carries the same to-do). Note the
sign gotchas: hip-pitch flexion is **negative**, knee is −95°/+5°.

### Travel probe

Each joint was driven ±10° from its zero, one servo at a time, in 20-tick steps
at 200 steps/s, with goal set to present position *before* torque enable so
engaging could not produce a jump. All ten tracked to ≤2 ticks of following
error at ≤24/1000 load, no fault flags, and returned to zero within 7 ticks.
End stops were not probed for nine of the ten and remain unrecorded.

### Measured knee ROM — the sim model is wrong

`L_knee` was then taken further, and the MJCF's `range="-95 5"` does **not**
describe the built joint. Measured 2026-08-02: the knee drives cleanly to
**−94.6° and +94.5°**, i.e. at least ±95°, at ≤172/1000 load (5.9% of a
2.94 N·m stall) and a flat 31 °C across two continuous full-range sweeps at
400–450 steps/s. The `+5°` was a modelling choice — an anatomical knee that
hyperextends barely at all — that the hardware does not share.

Reaching +95° needed the encoder re-centred first. The as-found zero of 3965
left only 130 ticks (+11.4°) before the 4095 wrap, so `middle` (bring-up step 4,
never previously run) was used to latch standing as 2048; `L_knee` is now the
only joint whose zero is centred, and ±95° = 967…3129 ticks. **This is an
encoder-placement limit, not a mechanical one** — the other joints whose zeros
sit near the ends (`L_hip_roll` 3532, `R_ankle` 3520, `L_ankle` 3452) will hit
the same wall if their real ROM also exceeds the model, and would need the same
treatment.

> **Sim-to-real gap, unresolved.** `sim/bimo_biped_v3yaw.xml` still declares
> `range="-95 5"` on both knees, so every policy trained to date believes the
> knee cannot extend past +5°. Widening it changes the plant and invalidates
> the trained runs, so it is deliberately left alone here — but a gait tuned
> against the old range is tuned against a knee the robot does not have.

## Assigning these IDs

Servos ship from the factory as ID 1, and two servos with the same ID will not
enumerate on the bus. At bring-up, connect **one servo at a time** and set its
ID before chaining it in — either through the board's web UI (AP mode,
`192.168.4.1`) or with `tools/servo_tool.py`:

```
.venv/bin/python tools/servo_tool.py http-setid 9 --yes
```

Label each case with its ID as you go.

See [wiring.md](wiring.md) for the bus electrical details, lead lengths per
hop, and current budget.
