# Servo reference

Every joint on the robot is a Feetech **STS3215**: the 12 V class, part number
ST-3215-C018. All of them sit on one half-duplex TTL bus at 1 Mbaud off the General
Driver board ([wiring.md](wiring.md#servo-bus)).

This page covers:

- ID assignment, and the ID → joint map;
- calibration, and how it is stored;
- the registers the robot depends on;
- what has been measured on the servo.

The robot's 17-servo ID map is **proposed** in
[§2.1](#21-the-robot-the-proposed-map), awaiting the owner's sign-off. The
firmware carries it as its bus joint set; the compiled policy drives the
prototype's 10 joints, IDs 1–10 ([§2.2](#22-what-the-policy-drives-today-the-10-joint-prototype)). The procedure from a bare board to the first arm is
[bringup.md](bringup.md). The bench-safety rules are in
[AGENTS.md](../AGENTS.md#bench-safety).

## 1. The bus and ID assignment

- **Every servo ships as ID 1.** Two servos with the same ID on the bus do not enumerate reliably.
- **Assign IDs one servo at a time.** Put exactly one servo on the bus, with the board powered off between swaps (do not hot-plug the bus). Set its ID, then label the case with the ID and the joint name on a face that stays readable after assembly.
- **Use the firmware CLI to set IDs.** Run it over the `USB` port at 115200, in bench mode:

  ```
  scan              # expect exactly one servo, at id 1
  reg 1 3 2         # Model_Number: 777 = STS3215 (an STS3250 reads 2825)
  reg 1 9 2         # min angle limit, expect 0
  reg 1 11 2        # max angle limit, expect 4095
  reg 1 33          # mode, expect 0 (position)
  gains 1           # position P / D / I (registers 21/22/23), factory 32/32/0
  id 1 <new>
  ```

- **What `id` does** (`cmdId` in `firmware/main/cli.cpp`):
  1. It refuses if `<new>` already answers.
  2. It unlocks the EEPROM (register 55 ← 0), writes register 5, and locks it again under the new ID.
  3. It waits 2 s for the EEPROM commit, then pings the new ID.
  4. On success it prints `ok, verified after commit -- safe to power down`. `WROTE BUT DID NOT VERIFY` means rescan before trusting it.
- **Checking the whole bus.**
  - `scan` names each ID's joint in the firmware's map.
  - `ping` with no argument checks every bus servo (IDs 1–17) against the fitted set in the calibration.
- **The duplicate-ID signature.** One servo answers intermittently, returns `bad-reply` and a nonsense 0.0 V, and another ID is missing. That is two servos answering to one ID. **Suspect a duplicate ID before suspecting the wiring.**
- **`tools/servo_tool.py` does not work with this firmware.**
  - It speaks the Feetech protocol through the stock firmware of Waveshare's *Servo Driver with ESP32*: its web UI at 192.168.4.1 and its `SERIAL_FORWARDING` USB↔bus bridge.
  - This repo's firmware owns the USB port as the CLI.
  - Its `setid` (unlock 55, write 5, lock), `info` and `fixrange` (mode 0, limits 0..4095) remain the register-level reference for these operations.
- **The CLI writes no arbitrary register.** `reg` is read-only by design. The
  register writes it has are named and gated: `id` (5), `middle` (40 ← 128),
  and `gains <id> <P> <D>` (21/22, [§4](#4-registers)).

## 2. ID → joint map

### 2.1 The robot: the proposed map

> **PROPOSED — awaiting the owner's sign-off (issue #77).** Nothing is
> assigned on hardware yet. Until it is signed off, treat IDs 11–17 and the
> splitter topology as a proposal.

The robot has 17 servos: 12 in the legs, one in the neck, four in the arms.
The rules behind the proposal:

- **IDs 1–10 keep the prototype's joints.** The prototype's servos, their case
  labels, the NVS calibration and the deployed 10-joint policy's `kServoId`
  all carry over unchanged; the robot's map is a superset of the prototype's.
- **New servos are appended, right before left**, as the hip yaws were (9 right,
  10 left): the ankle rolls are 11/12, then the neck 13, then the arms,
  shoulder before elbow, which is also their chain order.
- **Port A (H5) is the robot's right side, port B (H6) its left**, as on the
  prototype. Each leg is one chain outward from the pelvis, ending at the
  ankle roll.
- **The upper body needs a third and fourth branch.** Every STS3215 has two
  paralleled ports, so a chain is a line: two board ports make two lines, and
  two legs, two arms and a neck cannot be covered by two lines that start at
  the pelvis without leading the bus out to an elbow first. So each port
  feeds a passive bus splitter at the board: one branch down the leg, one up
  to the shoulder girdle. The neck rides the right arm's branch, which
  balances the ports at 9 and 8 servos.

| ID | joint | port | hop from the board | position P (Plan B) |
|---|---|---|---|---|
| 9 | `R_hip_yaw` | A, leg branch | 1 | stock |
| 1 | `R_hip_roll` | A, leg branch | 2 | **≈ 4×** |
| 2 | `R_hip_pitch` | A, leg branch | 3 | stock |
| 3 | `R_knee` | A, leg branch | 4 | **≈ 4×** |
| 4 | `R_ankle` (pitch) | A, leg branch | 5 | stock |
| 11 | `R_ankle_roll` | A, leg branch | 6 | **≈ 4×** |
| 13 | `neck_yaw` | A, upper branch | 1 | stock |
| 14 | `R_shoulder` | A, upper branch | 2 | stock |
| 15 | `R_elbow` | A, upper branch | 3 | stock |
| 10 | `L_hip_yaw` | B, leg branch | 1 | stock |
| 5 | `L_hip_roll` | B, leg branch | 2 | **≈ 4×** |
| 6 | `L_hip_pitch` | B, leg branch | 3 | stock |
| 7 | `L_knee` | B, leg branch | 4 | **≈ 4×** |
| 8 | `L_ankle` (pitch) | B, leg branch | 5 | stock |
| 12 | `L_ankle_roll` | B, leg branch | 6 | **≈ 4×** |
| 16 | `L_shoulder` | B, upper branch | 1 | stock |
| 17 | `L_elbow` | B, upper branch | 2 | stock |

```
H5 (A) ── splitter ─┬─ 9 R_hip_yaw ─ 1 R_hip_roll ─ 2 R_hip_pitch ─ 3 R_knee ─ 4 R_ankle ─ 11 R_ankle_roll
                    └─ 13 neck_yaw ─ 14 R_shoulder ─ 15 R_elbow
H6 (B) ── splitter ─┬─ 10 L_hip_yaw ─ 5 L_hip_roll ─ 6 L_hip_pitch ─ 7 L_knee ─ 8 L_ankle ─ 12 L_ankle_roll
                    └─ 16 L_shoulder ─ 17 L_elbow
```

Open with the proposal (issue #77):

- **The splitter** is not selected. It must be a passive 3-way part for the
  servo's 3-pin lead with manufacturer documentation, like every other part.
- **Current.** Each board port and its splitter carry that side's whole
  current: 9 servos on A, 8 on B, on a ~3 A-class connector
  ([wiring.md](wiring.md#servo-bus)); the budget is open.
- **Lead lengths** per hop are measured in
  [wiring.md](wiring.md#per-hop-lead-lengths-from-the-v6-cad-arms1): every hop
  fits the stock 150 mm lead with the fold loop folded in **except the
  shoulder→elbow hop, which needs ~200 mm** — it crosses the 290° shoulder
  fold with no strain relief drawn. The shoulder lead itself exits the pod
  through the girdle's open-to-top notch (shoulder_girdle_v6.py §216); the
  elbow servo rides the upper arm, so the lead does not cross the elbow.
  Source: `cad/v6/lead_lengths.py` (constants + an assembly-sweep
  crosscheck).

The joint names are the plant's: `sim/bimo_biped_v6ar.xml`, with the arms from
`sim/gen_plant_v6.py`.

The proposal is already compiled in, as a proposal, in three places that
must agree (the SIL suite's `test_bus_map_is_one_map_everywhere` checks it):

- `firmware/components/obs/include/obs/bus_map.h`, the firmware's **bus joint
  set**: calibration, the mechanical envelope, `pose`, `home`, the readback,
  the gain check, the telemetry joint block and the SIL arrays are all
  indexed by it (index *b* is servo ID *b* + 1);
- `JOINT_NAMES` in `link/protocol.py`, the telemetry joint block;
- `ID_BY_ROLE` in `tools/gen_obs_spec.py`, so a policy trained on the
  17-joint plant generates its `kServoId` from the same map.

On the 10-servo prototype, IDs 11–17 are **not fitted** (§3), and nothing
touches them. If the owner changes the map, those three, this table, and the
rows of §3.3 and §4 change in one commit.

### 2.2 What the policy drives today: the 10-joint prototype

The **policy joint set** is the joints the compiled policy observes and
commands, a subset of the bus joints. Its source of truth is
`firmware/components/obs/include/obs/obs_spec.h` (`kJointNames`, `kServoId`):

- `tools/gen_obs_spec.py` generates it from the deployed run's config, the plant `sim/bimo_biped_v5body.xml`, and `ID_BY_ROLE`.
- `firmware/host/test_obs.cpp` pins it.

| servo ID | joint | action index | bus port |
|---|---|---|---|
| 1 | `R_hip_roll` | 6 | A (right) |
| 2 | `R_hip_pitch` | 7 | A (right) |
| 3 | `R_knee` | 8 | A (right) |
| 4 | `R_ankle` | 9 | A (right) |
| 5 | `L_hip_roll` | 1 | B (left) |
| 6 | `L_hip_pitch` | 2 | B (left) |
| 7 | `L_knee` | 3 | B (left) |
| 8 | `L_ankle` | 4 | B (left) |
| 9 | `R_hip_yaw` | 5 | A (right) |
| 10 | `L_hip_yaw` | 0 | B (left) |

- Each leg chains outward from the pelvis:
  - Port A is the right leg: 9 → 1 → 2 → 3 → 4.
  - Port B is the left leg: 10 → 5 → 6 → 7 → 8.
- "Left" is the robot's own left: +Y in the plant, with the robot facing +X.

**The policy's action order is not bus-ID order.** The action vector is in sim
actuator order:

```
[L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle,
 R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle]
```

The firmware indexes `obs::kServoId = {10, 5, 6, 7, 8, 9, 1, 2, 3, 4}` by action
index, so action 0 drives bus ID 10. `obs::policyToBus(j)` is the bus joint of
policy joint *j*.

While armed, the loop reads every **fitted** bus servo, feeds the policy its
joints, and **holds every other fitted servo**: at the trained target when
the deployed run holds that servo (the robot's runs hold the neck at 0 and
the arms at their walking pose; `kHeldServoId`/`kHeldTarget` in
`obs_spec.h`), reached along the takeover ramp; otherwise where it was
measured at the takeover (on the robot, under the prototype's 10-joint
policy, the ankle rolls, neck and arms). `run` refuses unless every policy
joint is fitted.

## 3. Calibration: zero, direction, envelope

**Standing is angle zero for every joint** (`kJointDefault` is all zeros). Each
joint has two calibration values:

- `zero_steps`: the encoder ticks at the standing pose;
- `dir`: the sign that maps the sim's joint angle onto servo ticks.

There are two zeroing mechanisms, and they are **not interchangeable**:

| | `middle <id>` | `cal …` |
|---|---|---|
| writes | the servo's own offset (register 40 ← 128 latches the current position as 2048) | the firmware's calibration in NVS |
| lives | in the servo's EEPROM. It travels with the servo and survives reflashing the board | on the board. It is bound to the servo map |
| sets | a coarse zero, and keeps the zero away from the 0/4095 encoder wrap | a fine zero trim, plus the direction sign, which the servo cannot express |
| side effect | leaves torque **off** | none |

1. Hold the assembled robot at the neutral (standing) pose.
2. Run `middle` on each servo. Never run it on a bare servo: it defines 2048 wherever the horn happens to sit.
3. Run `cal zero` to absorb the residual. The horn seats on a 25-tooth spline, so a purely mechanical zero is never exact.
4. Run `cal save`.

A zero left near 0 or 4095 truncates the joint's range at the wrap.

The `cal` subcommands, all in bench mode. Rows are the 17 bus joints; a
`<joint>` is a joint name (`L_knee`) or a servo ID (`7`):

```
cal show                          # per joint: id, zero, dir, fitted; and whether NVS was loaded
cal zero [joint]                  # latch the current position as zero (all joints, or one)
cal dir <joint> <1|-1>
cal set <joint> <zero> <1|-1>     # type a row in by hand, when the robot cannot be posed
cal fit <joint> <0|1>             # say by hand whether this robot carries the servo
cal save | load | reset           # reset erases NVS and returns to 2048 / +1, all fitted
cal migrate                       # explicit v1-blob path; see below
```

**Fitted.** Each bus joint is fitted (the robot carries it) or not. The loop,
`pose`, `home`, `ping`, `volt` and the gain check touch fitted servos only: a
SYNC READ that waits on an absent servo costs its whole timeout every tick.
`cal zero` with no joint named sets it from the bus: a servo that answers is
fitted, one that does not is marked `NOT FITTED` (a 10-servo prototype comes
out as IDs 1–10). `cal fit` sets it by hand.

**How calibration is stored.**

- The servo calibration is a versioned NVS blob (`firmware/main/cal_store.h`, magic "BMC1", version 3). It holds, for every bus joint:
  - the joint count (17);
  - `zero_steps`, `dir` and `fitted`;
  - the bus map (`kBusServoId`) it was measured under;
  - a CRC-32.
- The boot load rejects a blob in any of these cases, and the robot then boots uncalibrated:
  - it was measured under a different servo map;
  - its joint count is different;
  - it is corrupt.
- An uncalibrated robot refuses `run` and `home`.
- **A version-2 blob migrates at boot, exactly, with no re-calibration.** v2 was the prototype's: indexed by its 10 policy joints, with their servo map. Boot checks that map, puts each servo's zero and direction on that servo's bus joint, marks IDs 1–10 fitted and 11–17 not, and re-saves the result as v3 under the same NVS key. The boot log says `cal: migrated v2 -> v3`.
- A v1 blob cannot prove its map. It migrates automatically only if it exactly equals the compiled reference table, `firmware/main/asbuilt_cal.h`. Otherwise `cal migrate` loads it into the live calibration for a human to check against §3.1 before `cal save`.
- The IMU calibration (bias, mount, gyro scale) is a separate blob. It does not depend on the servo map.
- **Re-zero after any horn or mechanical work.** Horn screws work loose.

**Directions.**

- `dir` cannot be read from the encoder.
- To settle each one, drive the joint alone (`torque <id>`, then small `move` steps) and compare with the sim's prediction for that joint at +20°.
- **Never infer a sign from a compound motion.**

**Travel probes.**

- Set the goal to the present position *before* enabling torque.
- Drive one servo at a time, in small steps (20 ticks at 200 steps/s), watching load in `pos`.

**The mechanical envelope.**

- `firmware/main/mech_envelope.h` is the bench clamp that `move`, `pose` and `home` apply, one row per bus joint.
- It is deliberately separate from the policy range (`kJointLo`/`kJointHi` in `obs_spec.h`). Widening the policy range rescales the action map and invalidates trained runs.
- A `static_assert` requires the envelope to contain the policy range.
- Change the header and §3.3 in the same commit, and name the evidence for every widening.

### 3.1 As-built zero calibration

These are the prototype's current values, measured at the standing pose with
`cal zero` + `cal save` after the horn screws were tightened on both legs. They are
also compiled into `firmware/main/asbuilt_cal.h` as the migration reference. **If
the robot is re-zeroed, update both files in the same commit.**

| joint | ID | zero (ticks) | Δ from 2048 | dir |
|---|---|---|---|---|
| `L_hip_yaw` | 10 | 1692 | −356 | +1 |
| `L_hip_roll` | 5 | 2418 | +370 | **−1** |
| `L_hip_pitch` | 6 | 2001 | −47 | +1 |
| `L_knee` | 7 | 1581 | −467 | **−1** |
| `L_ankle` | 8 | 3535 | +1487 | +1 |
| `R_hip_yaw` | 9 | 1806 | −242 | +1 |
| `R_hip_roll` | 1 | 3532 | +1484 | **−1** |
| `R_hip_pitch` | 2 | 2479 | +431 | +1 |
| `R_knee` | 3 | 2063 | +15 | **−1** |
| `R_ankle` | 4 | 3437 | +1389 | +1 |

- The large offsets are horn clocking.
- `R_knee` and `L_hip_pitch` sit near 2048 because `middle` re-centred them at the standing pose, which keeps their range clear of the 4095 wrap.

### 3.2 Direction signs

This is where the toe moves at +20° in the prototype plant, for each joint in
isolation:

| joint | sim-positive moves the toe |
|---|---|
| hip yaw | toward the robot's **left** (right leg toe-in, left leg toe-out) |
| hip roll | toward the robot's **left** (left leg abducts, right leg adducts) |
| hip pitch | **backward**: hip extension |
| knee | forward/up: hyperextension. **Flexion is negative** (knee axis `0 -1 0`) |
| ankle | **down**: plantarflexion |

- On the prototype, roll and knee have `dir` −1 on both legs; yaw, pitch and ankle have +1. The servos are mounted the same way on each side, not mirrored.
- The robot adds ankle roll, the neck, the shoulders and the elbows, and remounts the legs. **Every sign is re-derived on the robot.**

### 3.3 Mechanical envelope

The prototype's bench clamp, as it stands in `mech_envelope.h`. Angles are in the
sim joint frame.

| joint | envelope | evidence |
|---|---|---|
| hip yaw | ±45° | plant range; the stops have never been probed wider |
| hip roll | 25° adduction / **55° abduction** | CAD buffer sweep: 0.60 mm at 55°, under the buffer at 70°, touching at 90°. Creep-verify on hardware before a full 55° sweep |
| hip pitch | −110° … **+90°** | +90° backward: 0.70 mm CAD buffer, hardware-verified with both hips at +90° |
| knee | **±95°** | measured, both directions, ≤ 5.9 % load |
| ankle | ±40° | plant range; the toes clear the shin at both extremes |
| ankle roll (11, 12) | ±25° | plant range (`sim/bimo_biped_v6ar.xml`); never probed |
| neck yaw (13) | ±90° | plant range; never probed |
| shoulder (14, 16) | −90° … +200° | plant range (`sim/gen_plant_v6.py`: 0 hanging, 90 straight back, 180 up along the torso); never probed |
| elbow (15, 17) | ±150° | plant range; never probed |

The leg rows (IDs 1–10) are the **prototype's** legs, measured on it. The
robot's legs are different mechanisms: its knee hyperextends only 5° (the
prototype's +95° row would let the bench drive it past its stop) and its hip
pitch flexes to −125°. They need their own rows, probed on the robot, before
the bench drives the robot's legs toward either limit.

Pose-dependent leg-on-leg contact is not in this table, because a per-joint clamp
cannot express it. The plant handles it with inter-leg collision geoms.

### 3.4 Model-range sweep and measured knee ROM

These are prototype measurements.

`tools/bench_rom_sweep.py` swept every joint to 90 % of its model range on the
test stand. It uses single-servo torque, stepped waypoints, a webcam frame plus
position and load at each waypoint, and auto-release on any anomaly. Results:

- Every joint tracked. Following error was ≤ 8 ticks and loads ≤ 72/1000, at 33–35 °C, with no fault flags and a clean return to zero.
- Hip-roll adduction brings the legs into contact from about **−9°**. A torque-off leg rests at −7°, leaning on the other one.
  - The plant's inter-leg collision capsules are calibrated to these onsets.
  - The sweep tool caps inward roll at 6°.
- Yaw toe-in was clean to 20°, and the ankles to ±36°.

The knee drives cleanly to **−94.6° and +94.5°**:

- load ≤ 172/1000 (5.9 % of stall), 31 °C flat, over two continuous full-range sweeps at 400–450 steps/s;
- reaching +95° needed the encoder re-centred (`middle`), because the zero it had left only +11.4° before the 4095 wrap. That is an encoder-placement limit, not a mechanical one.

The two plants set the knee range differently:

- The prototype plant (`sim/bimo_biped_v5body.xml`) allows ±95°.
- The robot's plant (`sim/bimo_biped_v6ar.xml`) allows −130° to +5°: flexion is negative, and hyperextension is capped at 5°.

## 4. Registers

Addresses come from the Feetech ST3215 memory table v3.7. The firmware constants
are in `firmware/components/scsbus/include/scsbus/registers.h`, which also cites the
table's URL. Things to know about the table:

- Two-byte values are **little-endian**. The older SC series is big-endian.
- EEPROM writes stick only while the lock (register 55) is 0.
- The defaults below are the memory table's. That table was written against the 7.4 V part (its voltage-limit defaults, registers 14/15, are 8.0/4.0 V). **Read the registers on a C018; don't assume the defaults.**

| addr | name | bytes | default | notes |
|---|---|---|---|---|
| 3 | Model_Number (the table lists 3/4 as servo major/minor version, 9/3) | 2, read-only | 777 | STS3215 = **777**, STS3250 = **2825** (LeRobot's Feetech table). The check for a genuine part on arrival |
| 5 | ID | 1 | 1 | 0–253; 254 is broadcast |
| 9 / 11 | min / max angle limit | 2 | 0 / 4095 | must read 0 / 4095 in position mode |
| 21 | **position-loop P** | 1 | 32 | 0–254. Plan B: ≈ 4× (≈ 128) on hip roll, ankle roll and knee |
| 22 | position-loop D | 1 | 32 | raised with P |
| 23 | position-loop I | 1 | 0 | the first fallback if the P route fails (needs an integral term in the sim servo model first) |
| 26 / 27 | CW / CCW dead zone | 1 | 1 / 1 step | |
| 28 | protection current | 2 | 500 (× 6.5 mA) | |
| 33 | operating mode | 1 | 0 | 0 = position |
| 34 | protective torque | 1 | 20 % | output after overload protection trips |
| 35 | protection time | 1 | 200 (× 10 ms = 2 s) | how long the overload must last |
| 36 | overload torque | 1 | 80 % | the threshold that starts that timer |
| 38 | over-current protection time | 1 | 200 (× 10 ms) | |
| 40 | torque enable | 1 | 0 | 0 off, 1 on, **128 latches the current position as 2048** |
| 41 | acceleration | 1 | 0 | 100 steps/s² per LSB |
| 42 | goal position | 2 | | **a write auto-enables torque**: any goal write is a motion command |
| 46 | goal speed | 2 | | steps/s; 0 = unlimited |
| 48 | torque limit | 2 | | loaded from register 16 at power-on |
| 55 | EEPROM lock | 1 | | 0 = unlocked (writes persist), 1 = locked |
| 56 | present position | 2 | | sign in bit 15 |
| 58 | present speed | 2 | | steps/s, sign in bit 15 |
| 60 | present load | 2 | | 0.1 % duty, sign in bit 10 |
| 62 | present voltage | 1 | | 0.1 V. battguard and telemetry read this |
| 63 | present temperature | 1 | | °C |
| 65 | status | 1 | | error bits: voltage, sensor, temperature, current, angle, overload |
| 69 | present current | 2 | | 6.5 mA per LSB |

**Plan B gains.**

- The robot needs about 4× the stock static stiffness at the six roll and knee joints. Stock STS3215s there fail every walk and get-up gate ([DESIGN.md §4](../DESIGN.md)).
- The plan is register 21 at ≈ 4×, with D raised with it, on those six servos.
- That rests on one bench measurement (issue #73). Pass means:
  - ≥ 4× the P = 32 stiffness in the same rig;
  - with ≤ ±1 count of hold jitter.
- 3× passes only if the printed roll chains measure ≤ 1° of play.
- A high P can buzz or limit-cycle through gear play. LeRobot lowers the same register to 16 on its STS3215 arms for that reason.

**The measurement: `experiments/plan-b-bench/gain_bench.py`.** It drives this CLI over the tether,
one typed `go` per motion. Raw data goes to `hw_sessions/<date>/gain_bench/`;
the record goes in `experiments/plan-b-bench/`, with the rig and its dated sessions.

1. `scan`, then `read <id>`: which ID answers; P, D, I, the dead zones, the protection registers and the model number (777).
   - An ID on the bus map (1–17) has its `move` clamped to that joint's envelope, after it is sent. The motion steps take one only with `--allow-bus-id`, and then check every target against the envelope computed from the board's `cal show` and `mech_envelope.h`.
   - `park <id>` turns the bare horn into that band, and clear of the 0/4095 encoder wrap, before the lever goes on. It writes no EEPROM.
2. `stiffness <id> --goal <ticks> --rest <ticks>`: at each P the servo raises the bare lever from hanging straight down (REST) to straight out (GOAL), holds it there while the loads are hung, and lowers it back. Torque is released, and gains written, only at REST.
   - k is the slope of deflection on torque over the loaded points, with an intercept so the friction share drops out.
   - What passes is the ratio to P = 32.
   - The ladder is 32 / 64 / 96 / 128 / 160, with D raised with P. 128 is exactly 4 × 32, so a servo whose stiffness is proportional to P lands on the line there and a tick decides; 160 shows whether the trend carries on. A ratio within one standard error under a threshold reads MARGINAL.
3. `hold <id>`: the leg-like inertia, hanging down (the gear play floats) and horizontal (preloaded).
   - At each P: a quiet hold, then a 2° step out and back.
   - Quiet means ≤ 2 counts peak to peak and no protection bit.
   - If a raised P buzzes with D raised too, re-run it with D at 32 (`--ladder 128:32`). D differentiates a quantized encoder.
4. `stance`: step 1. It writes each P to whichever servo sits in the stance hip roll, runs `squat_bench.py --balance <side>:8:0 --balance-mode stance`, reads the roll's shortfall and restores the gains.
5. `report`: the tables and the verdict.

`--rehearse` runs the servo steps against a simulated servo, with no hardware.

**Writing the gains: `gains <id> <P> <D>`** (bench mode, one servo by ID,
never broadcast; `scsbus::writePositionGains`):

1. Reads register 40 and **refuses unless torque is off** on that servo (an
   unreadable torque state is a refusal too). `release <id>` first.
2. Unlocks the EEPROM (55 ← 0).
3. Writes P and D in one 2-byte write at 21.
4. Locks the EEPROM (55 ← 1). A failed step 2 or 3 still re-sends the lock.
5. Waits 2 s for the EEPROM commit, as `id` does.
6. Reads 21/22 back and prints `verified after commit -- safe to power down`,
   or `WROTE BUT READ BACK ...`.

It never writes a goal and never touches torque. P must be 1–254 and D 0–254.
`gains <id>` reads one servo's P, D and I against its expected row; `gains` with
no argument checks every servo. Power-cycle and `gains` again to prove the value
persisted.

**The arm gate.** A factory reset or a swapped spare silently returns to P = 32,
and the robot would then fall at its first crossover. So the firmware reads
registers 21/22 from every fitted servo **at boot and again before every
arm**, and compares them with the compiled expected table,
`firmware/main/servo_gains.h`:

- any difference, or a servo that does not answer, **refuses the arm** with
  `ArmResult` `REFUSED_GAINS` (10), which the `BENCH` beacon carries and every
  console prints ([control-channel.md](control-channel.md#saying-why-the-bench-diagnostic));
- the tether prints the failing rows (`expect P .. D ..  read P .. D ..`);
- a boot check that fails puts the same verdict on the beacon before anyone arms.
  A board powered from USB with the pack off reads no servo, so it says so; the
  arm re-reads once the pack is on.

The table is compiled in rather than stored in NVS: it is a design value with
evidence behind it, reviewed in git. It is also checked at compile time
against the deployed policy: the run's per-servo stiffness factor
(`kServoKpScale` in the generated `obs_spec.h`, from its `servo_kp_scale`;
Plan B trains ×4 on hip roll, ankle roll and knee) must be above 1 exactly
where the table raises P, so a Plan B policy cannot be flashed onto
factory-P servos, nor a stock policy onto raised ones. The register value
that realises a factor is the #73 measurement; the check is on the pattern. `gains <id> <P> <D>` changes a servo, not
what the robot expects; writing a value the table does not hold makes the next
arm refuse, and the CLI says so. **Change `servo_gains.h` and the table below
in the same commit, and name the measurement.**

**Per-ID gains.** Every row is the factory 32/32 until the Plan B bench test
(issue #73) measures the raised P. The six roll and knee servos then take it.

| ID | joint | P (21) | D (22) | I (23) | evidence |
|---|---|---|---|---|---|
| 1 | `R_hip_roll` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 2 | `R_hip_pitch` | 32 | 32 | 0 | factory |
| 3 | `R_knee` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 4 | `R_ankle` | 32 | 32 | 0 | factory |
| 5 | `L_hip_roll` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 6 | `L_hip_pitch` | 32 | 32 | 0 | factory |
| 7 | `L_knee` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 8 | `L_ankle` | 32 | 32 | 0 | factory |
| 9 | `R_hip_yaw` | 32 | 32 | 0 | factory |
| 10 | `L_hip_yaw` | 32 | 32 | 0 | factory |
| 11 | `R_ankle_roll` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 12 | `L_ankle_roll` | 32 | 32 | 0 | factory; Plan B raises it after #73 |
| 13 | `neck_yaw` | 32 | 32 | 0 | factory |
| 14 | `R_shoulder` | 32 | 32 | 0 | factory |
| 15 | `R_elbow` | 32 | 32 | 0 | factory |
| 16 | `L_shoulder` | 32 | 32 | 0 | factory |
| 17 | `L_elbow` | 32 | 32 | 0 | factory |

The gate checks P and D; I is shown by `gains <id>` but not gated (it stays 0
until the integral fallback, which needs an integral term in the sim first).

**Protections** (C018 sheet §7-11; thresholds in registers 28 and 34–38):

- **Overload.** More than 80 % of stall held for 2 s drops the output to 20 % torque. A new position command clears the flag.
  - In the get-up, the shoulder runs at 74–78 % of stall, which is close. Watch it on the bench.
- **Over-current.** More than 2 A for 2 s turns the output off.
- **Voltage.** Above 14 V or below 4 V the servo protects itself and recovers automatically.
- **Temperature.** Above 70 °C, torque goes off.

## 5. Measured STS3215 behaviour

| quantity | value | source |
|---|---|---|
| stall torque | 30 kg·cm = **2.94 N·m** at 12 V (±10 %) | C018 sheet; `_STS_STALL_12V` in `sim/walker_env.py` |
| current | stall 2.7 A, no-load running 180 mA, stopped 30 mA | C018 sheet |
| no-load speed | **4.04 rad/s at 12 V**, measured: a ceiling of ~2700 steps/s at 12.3 V (4.14 rad/s), 14 % under the datasheet's 0.222 s/60° | `tools/measure_servo_speed.py`, `R_knee` free sweeps over 2140 ticks. `_STS_NOLOAD_12V` |
| speed tracking | commanded goal speeds of 500–2000 steps/s are exact to 0.5 %. Lifting a leg, the speed derates to ~1650 steps/s | same run |
| response | **≈ 85 ms of pure dead time plus a ≈ 30 ms first-order lag (5 Hz)**, independent of amplitude and load. No rate limit below 650 steps/s | clamped-foot step test, 16 traces at 3 loads. The referee uses `act_delay_ticks` 4 (80 ms) and training randomises dead time |
| static stiffness at P = 32 | stance hip roll 1.2° short at load 120 (≈ 0.35 N·m), i.e. **≈ 17 N·m/rad**. The deploy model fits 12 N·m/rad, on the soft side | [design record](design-v6/2026-09-13-design-record.md) §4.4 |
| powered friction | **≈ 0.235 N·m** (8 % of stall): load ≈ 80/1000 at a steady 200 steps/s, four servos | `move <id> 3600 0 200` while polling `pos` |
| unpowered backdrive friction | **not measured**. The sim assumes 0.35 N·m (`off_frictionloss`); 0.235 N·m is a lower bound | |
| gear backlash | ≤ 0.5° (datasheet). The deploy model uses 1° | C018 sheet |
| joint play, printed chains | 1–2° in the yaw/roll chains and 2–3° free at the hip roll, by tilt hysteresis. Pitch chains ≤ 1.5° once the horn screws were tightened | prototype. The robot's rule: ≤ 3° per roll joint, ≤ 1° targeted, measured before the first walk |
| resolution | 4096 steps/rev (0.088°/step); middle = 2048 | |
| mass | 55 ± 1 g | C018 sheet |

The measurements behind the speed, response and friction rows are in
`docs/archive/2026-07-to-09-prototype-design-log.md`,
`docs/archive/2026-09-04-joystick-roadmap.md` and
`docs/archive/2026-07-26-bringup-day1.md`.
