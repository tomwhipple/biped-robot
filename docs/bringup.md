# Bring-up: bare board to first arm

This is the procedure that takes a General Driver board, a bag of STS3215s and an
assembled robot to a first armed run. The references it uses:

- electrical detail: [wiring.md](wiring.md)
- IDs, registers and calibration storage: [servo-map.md](servo-map.md)
- the CLI and firmware internals: [firmware/README.md](../firmware/README.md)
- the link: [control-channel.md](control-channel.md)

The firmware currently drives the 10-joint prototype. On the robot, every step that
depends on the joint list (IDs, `cal`, `pose`, `home`, the IMU check scripts,
policies) needs the 17-joint firmware port first. That port is open work.

**Bench safety.** The rules are in [AGENTS.md](../AGENTS.md#bench-safety). Read
them before a session. Two of them are procedure, and every step below assumes them:

- **Opening the serial port reboots the board**, and the reboot eats the first command you send. After connecting, wait for the boot log, send an empty line, and check the prompt answers before any command that writes state (`id`, `cal …`, `middle`, `imu bias`, `wifi …`).
- **Any goal write is a motion command.** A goal-position write auto-enables torque on an STS3215. Never "verify" a calibration by driving the live robot at it. The firmware refuses `move` to a servo whose torque is off; `torque <id>` is the deliberate first half of every move.

## 0. Before you start

- **Laptop.**
  - ESP-IDF v5.4. `export.sh` will not run inside a virtualenv: `env -u VIRTUAL_ENV bash` first.
  - The repo's `.venv`, with pyserial.
  - On macOS, the CP210x driver.
- **Bench supply.**
  - A current-limited supply for first power. The 3S pack comes later, behind its protection board and fuse ([wiring.md](wiring.md#power-path)).
- **Stand and spotter.**
  - A test stand that holds the pelvis with the feet free.
  - A spotter for anything armed. The spotter says "go" before every arm.

## 1. Flash and first power

**Host gates first.**

```
make -C firmware/host test          # pure modules under ASan/UBSan
env -u VIRTUAL_ENV bash
. ~/esp/esp-idf/export.sh
idf.py -C firmware set-target esp32
idf.py -C firmware build
idf.py -C firmware -p /dev/cu.usbserial-XXXX flash monitor
```

**Flashing.**

- Flash through the USB-C port silkscreened **`USB`**. The one silkscreened `LIDAR` enumerates a serial port that never syncs.
- Flash with the pack disconnected. USB alone powers the ESP32's logic, because both USB-C ports are diode-OR'd into the 5 V rail. USB does not power the servo rail (`DC_IN`), so `scan` finds nothing until the XH inlet is powered. That is expected.

**The terminal.**

```
.venv/bin/python -m serial.tools.miniterm /dev/cu.usbserial-XXXX 115200
```

**What a good boot looks like.**

- The board boots in **bench mode with torque released**. Nothing moves until `run`, or an arm over the radio.
- The boot log reports:
  - `cal:` either `restored from NVS` or `DEFAULTS`;
  - `gains:` whether every servo's position-loop P/D read back as expected. With the pack off no servo answers, so this says `NOT verified`; `run` re-reads once the pack is on;
  - the IMU driver and whether its calibration came from NVS;
  - the joint count, the observation width, and the policy run (or `PLACEHOLDER`).
- `help` lists the commands.

**Checks after every flash.**

- `stat` must name the run you meant to flash.
- Run `imu reinit`, then `imu scan`. A flash can reset the board mid-read and leave SDA held, and boot then falls back to the stub IMU.
- `imu scan` must show 0x6B, the QMI8658C. The other on-board I²C parts are listed in [sensor-expansion.md §1](sensor-expansion.md#1-the-boards-connectors).

**Wi-Fi.** `wifi <ssid> <psk>` stores the credentials in NVS. Bench mode only.

**First power.**

1. Check the XH pigtail's polarity against the board's "− +" silkscreen. Pin 1 is V+.
2. Set the bench supply to **11.1 V, current limit ~0.5 A**, and power the bare board through the XH inlet. A short shows up as a limit trip instead of smoke.
3. Raise the limit to ~2 A for a single unloaded servo.

**The landing voltage.**

- **Land the robot by 10.5 V** (3.5 V/cell on 3S). battguard warns at 10.5 V and lands the robot at 9.9 V ([wiring.md](wiring.md#battery-protection)).
- The firmware's only voltage reading is a servo's register 62, at 0.1 V resolution (`scan`, `pos`, `volt`). The board's INA219 is not read.
- With one servo connected, compare its reading with the supply. The telemetry and the guard both use this number, so a constant offset is worth knowing now.

## 2. Servo IDs and gains

The robot's ID map is **not yet assigned** ([servo-map.md §2.1](servo-map.md#21-the-robot-not-yet-assigned)).
Assign it before this step. Every servo ships as ID 1, so set IDs **one servo on
the bus at a time**.

For each servo:

1. Board off. Connect this servo alone to a bus port. Board on.
2. `scan`: expect exactly one servo, at ID 1, with a sane voltage and `err 0x00`.
3. Check the part and its factory state:
   - `reg 1 3 2` = **777** (STS3215);
   - `reg 1 9 2` = 0 and `reg 1 11 2` = 4095 (angle limits);
   - `reg 1 33` = 0 (position mode);
   - `gains 1` = P 32, D 32, I 0 (registers 21/22/23).
4. `id 1 <new>`. Wait for `ok, verified after commit -- safe to power down`. `WROTE BUT DID NOT VERIFY` means rescan before trusting it.
5. Check it is alive:
   - `torque <new>`, then `move <new> 2048 0 200`, then a second target, then `release <new>`;
   - it should track to within a few ticks, with no fault flag;
   - a bare servo has nothing to hit, but only 0..4095 limits a servo that is not in the firmware's map.
6. Label the case with the ID and the joint name. Board off.

**Then chain.**

- Chain one port at a time. `scan` must enumerate exactly the expected IDs; `ping` with no argument checks the firmware's own list.
- If a servo is missing, or answers intermittently with `bad-reply` and 0.0 V, suspect a **duplicate ID** before the lead.

**Gains.**

- `gains` reads registers 21/22 on every servo the firmware drives and compares them with the expected table (`firmware/main/servo_gains.h`, [servo-map.md §4](servo-map.md#4-registers)). It must end `all N servos at their expected P/D`: `run` refuses otherwise (`REFUSED_GAINS` on the beacon).
- The six hip-roll, ankle-roll and knee servos get P ≈ 4× only after the stiffness bench test passes (issue #73). The expected table changes first, in a commit that names the measurement; then each servo is written.
- To write one servo: `release <id>`, then `gains <id> <P> <D>`. It refuses unless that servo's torque is off, unlocks the EEPROM (55 ← 0), writes 21/22, locks it again (55 ← 1), waits for the commit and reads back: wait for `verified after commit -- safe to power down`. It writes no goal.
- Power-cycle, then `gains` again: the values must have persisted.
- A replacement servo arrives at the factory 32/32. It fails the gate until it is written.

## 3. Calibrate: zeros, directions, IMU, clock

Do this on the assembled robot, on the stand.

### Zeros

There are two mechanisms, and they are not interchangeable (`cli.cpp`, "calibration"):

- `middle <id>` writes the **servo's** own offset. It is a coarse zero, stored in the servo's EEPROM.
- `cal …` writes the **firmware's** calibration in NVS: a fine zero trim, plus the direction sign.

1. Hold the robot at the standing pose, which is angle zero for every joint, with torque released.
2. `middle <id>` for each servo. It latches the current angle as 2048 and leaves that servo's torque **off**. This keeps every zero away from the 0/4095 encoder wrap. Never run it on a bare servo.
3. `cal zero` (all joints) or `cal zero <joint>`, then `cal show`.
4. `cal save`. Reboot, and check the boot log says `cal: restored from NVS`.

`run` and `home` refuse while the calibration is missing, or was measured under
another servo map.

### Directions

`dir` cannot be read from the encoder. For each joint, alone:

1. `torque <id>`.
2. `move <id> <zero ± ~228 ticks> 0 200` (about 20°, at 200 steps/s).
3. Watch which way it goes, and compare with the sim's prediction for +20° on that joint ([servo-map.md §3.2](servo-map.md#32-direction-signs)).
4. `cal dir <joint> <1|-1>`, then `release <id>`.
5. `cal save` at the end.

Never infer a sign from a compound motion.

### Envelope and play

- **Envelope.** Probe each joint's travel.
  - Set the goal to the present position before enabling torque.
  - Move one servo at a time, in small steps, watching the load in `pos`.
  - Record the result in `firmware/main/mech_envelope.h` and servo-map.md §3.3, in one commit.
- **Play.** Measure the play of every roll joint by tilt hysteresis. **≤ 3° is required; ≤ 1° is the target.**
  - Re-zero after any horn or mechanical work.

### IMU, in the standing pose

The QMI8658C is on the board, so the mount is a property of how the board is
screwed into the pelvis. Measure it on the robot; do not reuse another mounting.

1. `imu`: `who_am_i 0x05` and `levelled yes`.
2. **Mount.**
   - Run `imu raw` while tilting the robot by hand to identify the chip's axes.
   - Then set `imu mount <w> <x> <y> <z>` (sensor → body; saved to NVS).
   - Gravity cannot tell a mount from the same mount turned 180° about the vertical. So lean the robot forward deliberately: `imu` must show `projgrav.x` negative and `up.x` positive.
3. **Bias.** This comes after the mount. The CLI warns if the mount is still identity.
   - Hold the robot **still, in the pose it will run in**, and run `imu bias`.
   - Bias moves with orientation and temperature, so a bias taken on the desk does not transfer.
   - If `spread` is flagged, the robot moved: redo it.
   - The reported yaw drift after 30 s is the number that matters.
4. **Gyro scale.**
   - With one stance foot clamped, run `tools/imu_scale_check.py --leg L` (or `R`). It compares the gyro integral against the accelerometer's tilt change through a stance-ankle move.
   - Set the per-axis result with `imu gscale <x> <y> <z>` (sensor frame, saved to NVS).
5. **Sign and frame.**
   - `tools/gyro_sign_check.py --joint ankle --amp 6`, `tools/imu_axes_check.py`, `tools/imu_drift_check.py`.
   - Pass: the gyro integral within 0.5° and `up` within 0.02 of the sim, on the same ankle, hip and knee leans.

These scripts **command poses** (`home`, `pose`), so the robot moves. They also
hard-code the prototype's calibration table and `/dev/ttyUSB0`; porting them is part
of the open port.

### Clock and link

1. `wifi` until `CONNECTED`.
2. `ntp`: expect `synced`, with a server from DHCP and a step in the low milliseconds.
   - `UNSYNCED` after a minute means the router does not hand out DHCP option 42, and pool.ntp.org is unreachable from the robot's network.
   - Until the clock syncs, every telemetry `t_us` is 0.
3. `.venv/bin/python link/verify_udp.py --host <robot-ip>` sends disabled stand frames and listens for the 10 Hz beacon. It checks both directions with **no motion**.
4. `bimo_gui --host <robot-ip>`: the session JSONL in `hw_sessions/` must carry `robot_utc` on every beacon (not `null`).

## 4. Torque drills

Start on the stand, then move to the floor with the spotter.

**Check torque before anything else.** Servos have been found with torque dropped
overnight, refusing goal writes with no fault flag and no reboot. `reg <id> 40`
reads 1 when torque is on.

1. **Engage and release.**
   - `torque [id]` and `release [id]`; with no ID they broadcast.
   - Confirm that `move` refuses on a released servo.
2. **Reset to the stand.** `home` does the following:
   1. Enables torque.
   2. Slews every joint to its calibrated zero on a minimum-jerk profile (2.5–8 s).
   3. Takes up hip-roll play with a 5° out-and-back on each hip.
   4. Reads every joint back, which must be within 12 ticks (~1°).
   5. **Releases torque**. Gear friction holds the stand.

   About the verdict:
   - `home` refuses without calibration, or with the pack guard latched.
   - RESET SERVOS on the link runs the same sequence. Its `DISARMED_HOME` verdict means the joints were read back at their zeros, not merely written.
3. **Passive stand.** Stand the robot up, release torque, and watch.
   - If it holds, joint friction is enough, and idle torque-off is feasible.
   - If it collapses or drifts more than 10 cm, it is not.
   - This is the unpowered backdrive friction measurement that no bench tool has made ([servo-map.md §5](servo-map.md#5-measured-sts3215-behaviour)).
4. **Arm on the stand and read the loop timing.**
   - Arm with `run`, or over the link.
   - Read `stat`: late ticks at 0 %, the worst tick, and the per-phase microseconds. The bus time for the full servo set is measured here.
5. **The watchdog.** With a commander connected and the robot armed on the stand:
   - Kill the commander. The robot should go to `STAND` after 250 ms, then to `RELAX` (torque off) at 5 s.
   - E-stop latches, and clears only on a frame with `ENABLE` off.
6. **Disarm does not lower the robot.** A disarm releases torque, friction holds the joints, and a rigid body tips. End every probe with RESET SERVOS **while still armed**.
7. **Guards.**
   - `FALLEN`: `up_z` below 0.4 for 200 ms releases torque and benches the loop. Re-arming takes a fresh arm.
   - The pack guard: `batt` shows its level. `batt reset` is for after a pack swap only.
   - After any fall or guard trip, RESET SERVOS.

## 5. The offline flash gate

No policy is flashed until it passes all of the following, offline:

1. **Referee.** On the lag referee: `stand_10s` 8/8 and `stand_off` ≥ 6/8.
2. **Tick 1 at home.**
   - Feed the robot's home observation to the exported student (`.silw` through `sim/sil/harness.py` `silw_forward`).
   - The first action must be **≤ 0.05 on every joint**.
   - The zero pose is the stop/rest pose. A policy that asks for motion at home walks the robot off its own stance.
3. **Twin rest.**
   - Run the SIL twin, `sim/sil_twin.py` (nominal plant) with `--act-delay-ticks 4` for the servo's dead time.
   - Over the last 3 s of a 6 s stand, every joint must rest **within 2° of home** and the torso **within 1°**.
4. **Build.** Then run:
   - `make -C firmware/host deploy-headers RUN=<run>`;
   - the host tests;
   - SIL ([sim/sil/README.md](../sim/sil/README.md));
   - `idf.py -C firmware build`.
5. **Check.** Flash, then check that `stat` names the run, and run `imu reinit` and `imu scan`.

A policy that scores well in the referee can still fail tick 1 or twin rest.
Referee scores are gates, not proof.

## 6. Staged hardware gates

- Every stage has a sim gate and a hardware gate, and nothing advances on a feeling.
- Record every arm: camera, and the beacon CSV (`tools/record_attempt.sh`).
- Change one variable per run.
- Before blaming a sensor, run the same input through the twin.

**Before any policy.**

1. Weigh every part.
2. Confirm roll-joint play is within bounds (§3).
3. Put the measured stiffness, play and masses into the plant.
4. Re-run the gates: `sim/gate_no3250.py walk | sweep | arms | getup`.
5. Then run the floor sequence **over the tether, one motion per go**: stand → crouch → shift → one-foot → step in place → six steps.
   - The motions are the `sim/static_gait.py` keyframes, streamed through the firmware's `pose` path.
   - Record each step as a margin, clearance and slip number, against the sim's.
6. Then the scripted get-up.
7. Measure the bus current here ([wiring.md](wiring.md#current-the-open-constraint)).

**Policy stages**, once a policy is trained on the measured plant:

| stage | sim gate (lag referee) | hardware gate |
|---|---|---|
| 1. Stand: rest is home | §5 | 5 arms × 30 s on the pad at full gyro gain: no guard, hip-pitch swing < 0.05 rad, hip-roll and yaw drift < 3°, torso within 3°. Then 3 × 30 s on bare wood. Then 5 pushes of ±2 cm at the pelvis, each recovered without a step |
| 2. Posture | `squat_reps` ≥ 6/8, `weight_shift` ≥ 6/8 | 3 crouch-and-return cycles from the slider, 3 s each way, no guard, torso pitch within 5°. A 60 s hold at half crouch, swing < 0.05 rad |
| 3. Step in place | `march_in_place` ≥ 6/8, feet clear 2 cm | 10 s march with both feet visibly clear: 3/3 on the pad, 2/3 on wood. Stops to a quiet stand within 1 s |
| 4. Walk and turn | `line_1m`, `backward_1m` ≥ 6/8; `turn_180`, `reversal` ≥ 5/8 | vx 0.15 for 1 m, 3/3, lateral drift < 20 cm, stops within 1 s. 90° in place, 3/3, heading error < 20°. Forward → backward, 2/3. The line and the turn on the pad, and one of them on wood |
| 5. Joystick | — | Caps vx ≤ 0.2 and wz ≤ 0.5. 5 min of free driving on the pad with at most one assisted catch, twice; then 5 min on wood |

The recovery gesture throughout is RESET SERVOS while armed. Releasing torque does
not lower the robot.
