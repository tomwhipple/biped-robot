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
| 9        | left hip yaw    | `L_hip_yaw`   | 0            | A (left)  | −45°…+45°  | Z (yaw)         |
| 10       | right hip yaw   | `R_hip_yaw`   | 5            | B (right) | −45°…+45°  | Z (yaw)         |

"Left" is the robot's own left: +Y in the plant, with the robot facing +X.

## By chain position

The board's two bus ports are electrically the same bus — they are split one
per leg purely for cable routing. Each leg chains outward from the pelvis:

- **Port A → left leg**: ID **9** hip yaw → ID **1** hip roll → ID **2** hip
  pitch → ID **3** knee → ID **4** ankle
- **Port B → right leg**: ID **10** hip yaw → ID **5** hip roll → ID **6** hip
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

so **action 0 drives bus ID 9**, not ID 1. The firmware applies the
permutation `obs::kServoId = {9, 1, 2, 3, 4, 10, 5, 6, 7, 8}` — index by
action, read the bus ID.

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
