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
| 1        | **right** hip roll  | `R_hip_roll`  | 6        | A (right) | −25°…+25°  | X (roll)        |
| 2        | **right** hip pitch | `R_hip_pitch` | 7        | A (right) | −110°…+60° | Y (pitch)       |
| 3        | **right** knee      | `R_knee`      | 8        | A (right) | ±95° meas. | Y (pitch)       |
| 4        | **right** ankle     | `R_ankle`     | 9        | A (right) | −40°…+40°  | Y (pitch)       |
| 5        | **left** hip roll   | `L_hip_roll`  | 1        | B (left)  | −25°…+25°  | X (roll)        |
| 6        | **left** hip pitch  | `L_hip_pitch` | 2        | B (left)  | −110°…+60° | Y (pitch)       |
| 7        | **left** knee       | `L_knee`      | 3        | B (left)  | ±95° meas. | Y (pitch)       |
| 8        | **left** ankle      | `L_ankle`     | 4        | B (left)  | −40°…+40°  | Y (pitch)       |
| 9        | **right** hip yaw   | `R_hip_yaw`   | 5        | A (right) | −45°…+45°  | Z (yaw)         |
| 10       | **left** hip yaw    | `L_hip_yaw`   | 0        | B (left)  | −45°…+45°  | Z (yaw)         |

"Left" is the robot's own left: +Y in the plant, with the robot facing +X.

> **Assembly errata, 2026-08-02 — the two bus chains went onto opposite legs.**
> Port A is the **right** leg and port B is the **left**, the reverse of the
> plan. This was found in stages: first the hip-yaw pair looked swapped
> (commit `eb062cf`), then the rest of the chain turned out to be swapped with
> it. Confirmed on the robot, not inferred:
>
> - driving servo **9**'s hip yaw rotates the **right** leg (toe-in at +20°),
> - hip-pitch servo **2** is on the **right**, servo **6** on the **left**,
> - knee servo **3** is the **right** knee, servo **7** the **left**.
>
> The servos are not coming back out, so the firmware map absorbs it:
> `kServoId = {10, 5, 6, 7, 8, 9, 1, 2, 3, 4}`. Note the yaw entries are
> unchanged from the first fix — a whole-chain swap and a yaw-only swap agree
> on the yaw servos, which is why the yaw-only reading survived as long as it
> did. The rows above are the as-built truth.

## By chain position

The board's two bus ports are electrically the same bus — they are split one
per leg purely for cable routing. Each leg chains outward from the pelvis:

- **Port A → RIGHT leg**: ID **9** hip yaw → ID **1** hip roll → ID **2** hip
  pitch → ID **3** knee → ID **4** ankle
- **Port B → LEFT leg**: ID **10** hip yaw → ID **5** hip roll → ID **6** hip
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
permutation `obs::kServoId = {10, 5, 6, 7, 8, 9, 1, 2, 3, 4}` — index by
action, read the bus ID. (It was `{9, 1, 2, 3, 4, 10, 5, 6, 7, 8}` as designed,
briefly `{10, 1, …, 9, …}` after the yaw-only fix; see the errata note above.)

## As-built zero calibration

Measured 2026-08-02 on the assembled robot, held at the standing pose on a test
stand, via `cal zero` + `cal save`. Standing **is** angle zero for every joint
(`kJointDefault` is all zeros), so these ticks are each joint's `zero_steps`.
They are stored in NVS; the boot log reads `cal: restored from NVS`.

Re-measured after the legs-swapped remap — `cal` is stored per *joint index*,
so remapping invalidated every entry and the blob was erased (`cal reset`)
rather than left to look valid.

| Joint | ID | zero (ticks) | Δ from 2048 | dir |
| ----- | -- | ------------ | ----------- | --- |
| `L_hip_yaw`   | 10 | 1693 | −355  | +1 |
| `L_hip_roll`  | 5  | 2420 | +372  | **−1** |
| `L_hip_pitch` | 6  | 3273 | +1225 | +1 |
| `L_knee`      | 7  | 1634 | −414  | **−1** |
| `L_ankle`     | 8  | 3516 | +1468 | +1 |
| `R_hip_yaw`   | 9  | 1803 | −245  | +1 |
| `R_hip_roll`  | 1  | 3533 | +1485 | **−1** |
| `R_hip_pitch` | 2  | 2501 | +453  | +1 |
| `R_knee`      | 3  | 2050 | +2    | **−1** |
| `R_ankle`     | 4  | 3450 | +1402 | +1 |

The large offsets are ordinary horn clocking — the ST3215 horn seats on discrete
splines, so a mechanical zero is never exact and `middle` was never run at the
CAD-neutral pose. **Before this was measured the firmware still believed zero
was 2048 for all ten**, which would have driven a knee 168° on the first `run`.
`R_knee` sits at ~2048 only because its encoder was deliberately re-centred
(see below); the rest are wherever the horn happened to seat.

### Direction signs — all ten verified 2026-08-02

**`dir` cannot be read off the encoder**, which only reports the servo's own
frame; the sign is a physical convention, so every one was settled by driving
that joint alone and having a human say which way it went. The reference for
"which way is positive" is the sim itself: `tools/` aside, a throwaway MuJoCo
script set each joint to +20° in isolation and reported where the toe moved in
the torso frame, giving an unambiguous prediction per joint:

| joint | sim-positive moves the toe |
| ----- | -------------------------- |
| hip yaw   | toward the robot's **left** (right leg toe-in, left leg toe-out) |
| hip roll  | toward the robot's **left** (left leg abducts, right leg adducts) |
| hip pitch | **backward** — hip extension |
| knee      | **backward/down** — see the knee note below |
| ankle     | **down** — plantarflexion |

The result is symmetric between the legs: **roll and knee are inverted, yaw,
pitch and ankle are not.** Both legs share the same pattern, which fits servos
mounted the same way on each side rather than mirrored.

Two things this caught that no encoder-only check could:

- **The knees.** At `+1` the sim's negative knee angle — which the policy
  treats as flexion — drove the joint into *hyperextension*. This is the exact
  failure `docs/assembly.md` warns about.
- **`L_hip_yaw` was briefly recorded wrong.** It was first inferred from a
  collision during a four-joint compound pose, which attributed the leg's
  inward swing to the yaw. A clean single-joint test showed the opposite, and
  the real culprit was `L_hip_roll` (`−1`, so positive ticks swing that leg
  *inward*). Inferring a sign from a compound motion does not work; drive one
  joint at a time.

### Travel probe

Each joint was driven ±10° from its zero, one servo at a time, in 20-tick steps
at 200 steps/s, with goal set to present position *before* torque enable so
engaging could not produce a jump. All ten tracked to ≤2 ticks of following
error at ≤24/1000 load, no fault flags, and returned to zero within 7 ticks.
End stops were not probed for nine of the ten and remain unrecorded.

### Measured knee ROM — the sim model is wrong

Servo 3 — the **right** knee — was then taken further, and the MJCF's
`range="-95 5"` does **not** describe the built joint. Measured 2026-08-02: the
knee drives cleanly to **−94.6° and +94.5°**, i.e. at least ±95°, at ≤172/1000
load (5.9% of a 2.94 N·m stall) and a flat 31 °C across two continuous
full-range sweeps at 400–450 steps/s. The `+5°` was a modelling choice — an
anatomical knee that hyperextends barely at all — that the hardware does not
share. (This measurement was taken while servo 3 was still mis-named `L_knee`;
the joint is physically the right knee either way.)

Reaching +95° needed the encoder re-centred first. The as-found zero of 3965
left only 130 ticks (+11.4°) before the 4095 wrap, so `middle` (bring-up step 4,
never previously run) was used to latch standing as 2048; servo 3 (`R_knee`) is
the only joint whose zero is centred, and ±95° = 967…3129 ticks. **This is an
encoder-placement limit, not a mechanical one** — the other joints whose zeros
sit near the ends (`R_hip_roll` 3533, `L_ankle` 3516, `R_ankle` 3450) will hit
the same wall if their real ROM also exceeds the model, and would need the same
treatment. `L_knee` (servo 7) needs none: its zero of 1634 leaves −143°/+216°.

> **Sim-to-real gap, unresolved — and worse than just the range.**
> `sim/bimo_biped_v3yaw.xml` declares `range="-95 5"` on both knees, so every
> policy trained to date believes the knee cannot extend past +5°. But the
> *sign* is wrong too: driving `L_knee` to −95° in MuJoCo puts the toe forward
> and 14.8 cm **up**, i.e. the shank swings forward — a bird-style
> backward-bending knee. Human flexion (heel toward buttock) is the model's
> `+` direction, capped at 5°. So as modelled the knee gets 95° of
> hyperextension and 5° of flexion.
>
> The hardware is now calibrated the human-natural way (`dir −1`, confirmed by
> eye), which means **sim and robot bend opposite ways for the same command**.
> Fixing the MJCF changes the plant and invalidates every trained run, so it is
> deliberately left alone here — but no gait should be trusted to transfer
> until this is resolved.

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
