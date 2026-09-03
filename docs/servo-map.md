# Servo address map

Bus address (ST3215 servo ID) → physical joint on the robot. Ten servos on one
half-duplex TTL chain at 1 Mbaud off the Waveshare ESP32 driver board.

Generated-source of truth: `firmware/components/obs/include/obs/obs_spec.h`
(`kJointNames` / `kServoId`), which `tools/gen_obs_spec.py` emits from the
deployed run's config and its plant MJCF (currently
`sim/bimo_biped_v5body.xml`). The host tests (`firmware/host/test_obs.cpp`) pin
the permutation, so this table and the firmware cannot drift by hand.

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

> **Re-zeroed 2026-09-03** after loose servo-horn screws were found and tightened on
> both legs. Stand set by eye on the desk, `cal zero` + `cal save`. Shifts from the
> 2026-08-02 values: L hip pitch −3.8°, L knee −4.7°, L ankle +1.7°, R hip pitch −1.9°,
> R knee +1.1°, R ankle −1.1°; yaws and rolls under 0.3°. Table below is the new set.

Measured 2026-08-02 on the assembled robot, held at the standing pose on a test
stand, via `cal zero` + `cal save`. Standing **is** angle zero for every joint
(`kJointDefault` is all zeros), so these ticks are each joint's `zero_steps`.
They are stored in NVS; the boot log reads `cal: restored from NVS`.

This table is also compiled into the firmware as
`firmware/main/asbuilt_cal.h`, where it gates the automatic v1→v2
calibration-blob migration (a v1 blob cannot prove its servo map, but one
that exactly matches this measurement is this measurement). **If the robot is
re-zeroed, update both files in the same commit.**

Re-measured after the legs-swapped remap — `cal` is stored per *joint index*,
so remapping invalidated every entry and the blob was erased (`cal reset`)
rather than left to look valid.

| Joint | ID | zero (ticks) | Δ from 2048 | dir |
| ----- | -- | ------------ | ----------- | --- |
| `L_hip_yaw`   | 10 | 1692 | −356  | +1 |
| `L_hip_roll`  | 5  | 2418 | +370  | **−1** |
| `L_hip_pitch` | 6  | 2001 | −47    | +1 |
| `L_knee`      | 7  | 1581 | −467  | **−1** |
| `L_ankle`     | 8  | 3535 | +1487 | +1 |
| `R_hip_yaw`   | 9  | 1806 | −242  | +1 |
| `R_hip_roll`  | 1  | 3532 | +1484 | **−1** |
| `R_hip_pitch` | 2  | 2479 | +431  | +1 |
| `R_knee`      | 3  | 2063 | +15    | **−1** |
| `R_ankle`     | 4  | 3437 | +1389 | +1 |

The large offsets are ordinary horn clocking — the ST3215 horn seats on discrete
splines, so a mechanical zero is never exact and `middle` was never run at the
CAD-neutral pose. **Before this was measured the firmware still believed zero
was 2048 for all ten**, which would have driven a knee 168° on the first `run`.
`R_knee` and `L_hip_pitch` sit at ~2048 because their encoders were
deliberately re-centred (R_knee at bring-up — see below; L_hip_pitch on
2026-08-03, when its old zero of 3273 capped backward pitch at +72° against
the 4095 wrap during the +90° envelope test); the rest are wherever the horn
happened to seat.

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
| knee      | **forward/up** — hyperextension; flexion is **negative** ¹ |
| ankle     | **down** — plantarflexion |

¹ The knee row was **restated on 2026-08-02** when the plant's knee sign was
corrected (see "Measured knee ROM" below). It read "backward/down" while
`bimo_biped_v3yaw.xml` still had `axis="0 1 0"` on the knees. The `dir` column
did **not** change: `−1` was chosen so that the *policy's* negative knee means
flexion on the robot, and the corrected plant now agrees with the robot instead
of contradicting it. Nothing on the board was reflashed or re-zeroed for this.

The result is symmetric between the legs: **roll and knee are inverted, yaw,
pitch and ankle are not.** Both legs share the same pattern, which fits servos
mounted the same way on each side rather than mirrored.

Two things this caught that no encoder-only check could:

- **The knees.** At `+1` the sim's negative knee angle — which the policy
  treats as flexion — drove the joint into *hyperextension*. This is the exact
  failure `docs/assembly.md` warns about. (At the time this also disagreed with
  the MJCF, whose knee axis made negative mean hyperextension too; the robot
  was left calibrated the human-natural way and the **plant** was corrected on
  2026-08-02 to match it.)
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

### Mechanical envelope — the bench clamp's table (2026-08-03)

The CLI bench clamp no longer uses the plant's policy range: it clamps to
`firmware/main/mech_envelope.h`, the measured/CAD-verified mechanical truth.
The split exists because the two ranges answer different questions — what a
policy trains in (widening it rescales action maps and invalidates runs)
vs how far the bench may drive a joint. **Change the envelope header and
this table in the same commit**, and name the evidence for every widening:

| joint | envelope (sim frame) | evidence |
| ----- | -------------------- | -------- |
| hip yaw   | ±45°           | plant range; stops never probed wider |
| hip roll  | 25° adduction / **55° abduction** | CAD buffer sweep 2026-08-03: 0.60 mm at 55°, under-buffer at 70°, touching at 90°. Creep-verify on hardware before a full 55° sweep |
| hip pitch | −110°…+60°     | plant range; stand blocks measuring past +45° back |
| knee      | **±95°**       | measured 2026-08-02, both directions, ≤5.9 % load |
| ankle     | ±40°           | plant range; toes clear the shin at both extremes |

Pose-dependent leg-on-leg contact (adduction ~9° standing, yaw cross ~8.5°
with thighs raised) is deliberately NOT in this table — a per-joint clamp
cannot express it; the plant needs inter-leg collision geoms instead.

### Model-range sweep — all ten joints, visually confirmed (2026-08-02 night)

`tools/bench_rom_sweep.py` (single-servo torque, stepped waypoints, webcam
frame + pos/load at each, auto-release on anomaly) swept every joint to 90 %
of its model range on the test stand, after the knee-sign fix (`0e7d866`):

- **All ten joints track their commanded range**: following error ≤8 ticks,
  loads ≤72/1000 (worst: hip pitch holding the thigh near-horizontal at
  −99°), temps 33–35 °C, no fault flags, clean return to zero every time.
- **Directions visually confirmed on camera** at the extremes, including
  knee −85.5° = deep human-style flexion (heel toward buttock) and hip
  pitch −99° = leg raised far forward — both as the corrected convention
  predicts.
- **Finding — inter-leg contact the sim cannot see:** hip-roll *adduction*
  brings the legs into contact from ≈ **−9°** (load onset; torque-off the
  left leg rests at −7° leaning on its neighbor). The plant's ±25° roll
  range is optimistic inward: internal parts have no collision geoms, so
  sim legs pass through each other. `feet_distance`/`foot_cross` penalties
  are the current mitigation; the sweep tool caps inward roll at 6°.
  Outward (abduction) 22.5° is clean.
- Yaw toe-in swept to the errata-tested 20°, clean; ankles ±36° clean
  (toe-down droop is also each ankle's torque-off rest). Frames + JSON log
  in `sweeps/` (local only, gitignored).

### Measured knee ROM — and the plant fix it forced

Servo 3 — the **right** knee — was then taken further, and the MJCF's
`range="-95 5"` did **not** describe the built joint. Measured 2026-08-02: the
knee drives cleanly to **−94.6° and +94.5°**, i.e. at least ±95°, at ≤172/1000
load (5.9% of a 2.94 N·m stall) and a flat 31 °C across two continuous
full-range sweeps at 400–450 steps/s. The `5°` was a modelling choice — an
anatomical knee that hyperextends barely at all. The hardware has no such stop,
and on the model's wrong axis that cap had landed on *flexion*, the one
direction the robot actually uses. The cap is **kept**, on the correct side
now, as the deliberate hyperextension limit (see the callout below). (This
measurement was taken while servo 3 was still mis-named `L_knee`; the joint is
physically the right knee either way.)

Reaching +95° needed the encoder re-centred first. The as-found zero of 3965
left only 130 ticks (+11.4°) before the 4095 wrap, so `middle` (bring-up step 4,
never previously run) was used to latch standing as 2048; servo 3 (`R_knee`) is
the only joint whose zero is centred, and ±95° = 967…3129 ticks. **This is an
encoder-placement limit, not a mechanical one** — the other joints whose zeros
sit near the ends (`R_hip_roll` 3533, `L_ankle` 3516, `R_ankle` 3450) will hit
the same wall if their real ROM also exceeds the model, and would need the same
treatment. `L_knee` (servo 7) needs none: its zero of 1634 leaves −143°/+216°.

> **Sim-to-real gap — RESOLVED 2026-08-02, in the sim.**
>
> *What was wrong.* `sim/bimo_biped_v3yaw.xml` had `axis="0 1 0" range="-95 5"`
> on both knees. Driving `L_knee` to −95° in MuJoCo put the toe forward and
> up — the shank swung forward, a bird-style backward-bending knee. Human
> flexion (heel toward buttock) was the model's `+` direction, capped at 5°.
> So as modelled the knee had 95° of hyperextension and 5° of flexion, while
> the robot — calibrated `dir −1` — flexed the human way on the *same*
> negative command. Sim and robot bent opposite ways.
>
> *The fix.* Both knee joints became **`axis="0 -1 0"`**. The range string is
> unchanged, and that is the point: `−95 … +5` now means **95° of flexion and
> 5° of hyperextension** instead of the reverse. Re-checked in MuJoCo after the
> change: `L_knee`/`R_knee` at −20° put the foot 3.1 cm **backward** and at
> −95° put it 9.0 cm backward and 9.8 cm up. On the robot, servo 3 at raw 2278
> (= −20° through zero 2050 / `dir −1`) was filmed doing exactly that — shank
> back, heel lifted. Same command, same motion, both sides.
>
> *Why −95 / +5 and not ±95.* ±95° is the measured **mechanical** envelope, not
> a sensible operating range. The flexion limit is set to the measured travel
> (−95°). Hyperextension is capped at +5°: a knee that folds backward is a
> failure mode, and holding the modelled envelope strictly inside the measured
> one means nothing a policy learns is beyond what the joint can do. Widening
> it later is a deliberate decision, not a default.
>
> *Hardware impact: none.* `zero_steps` and `dir` are untouched, and the
> generated `obs_spec.h` limits are byte-identical (the numbers did not move,
> only their meaning), so the firmware bench clamp still resolves to servo 3
> ticks `[1993..3131]` — which is now correctly read as +5° of hyperextension
> through 95° of flexion. **No reflash and no recalibration are required.**
>
> *Everything trained before this is invalid.* Every policy in `sim/runs/`
> learned the mirrored knee; their scorecards describe a plant that no longer
> exists. Runs and scorecards are kept as history, not as claims. See
> `DESIGN.md`, "The knees bent the wrong way".

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
