# firmware/ — Waveshare ESP32 robot driver board

*Status 2026-09-03. Design doc: [docs/firmware-design.md](../docs/firmware-design.md).
Wire protocol: [docs/control-channel.md](../docs/control-channel.md). Electrical:
[docs/wiring.md](../docs/wiring.md).*

**This is flashed and running on the robot.** The board is a Waveshare
**General Driver for Robots** (swapped in 2026-08-14 from the "Servo Driver
with ESP32" — see `main/board.h`, the two are not interchangeable). It boots
into bench mode with torque released, arms over the radio, and runs the 50 Hz
policy loop on core 1 with the distilled network in flash. The robot walked
under it on 2026-09-01.

*Historical note:* flashing **overwrote the vendor web UI** at 192.168.4.1 that
was originally used to assign servo IDs. The CLI's `scan` / `id` cover that job
now.

Two independent gates, both green and both run by CI on every push:

| gate | command | status |
|---|---|---|
| host unit tests | `make -C firmware/host test` | 9747 checks, 0 failures |
| target build | `. ~/esp/esp-idf/export.sh && idf.py -C firmware build` | links, 0 warnings |

The target build is ESP-IDF **v5.4** on macos-arm64 (`~/esp/esp-idf`,
toolchains in `~/.espressif`). With §6's distilled policy weights now
compiled in it produces a 0x60d20-byte image — **62 % of the app partition
still free**; the 150 KB of weights land in flash `.rodata` (Flash Data
202 648 B), not SRAM, so DRAM is unchanged at 17.5 %. Two environment
gotchas, both hit and fixed on first run:

- `export.sh` and `install.sh` **refuse to run inside a Python virtualenv**.
  This repo's `.venv` is usually active, so unset `VIRTUAL_ENV` and drop
  `.venv/bin` from `PATH` first.
- ninja is not installed; the build goes through cmake's `Unix Makefiles`
  generator. `brew install ninja` if you want the faster incremental builds.

## Build and test on the host (this is the gate)

```
make -C firmware/host test
```

Compiles the pure modules with `-Wall -Wextra -Werror -Wshadow -Wconversion
-Wsign-conversion` under `-fsanitize=address,undefined` and runs seven suites:

```
protocol          256 checks, 0 failures
scsbus            147 checks, 0 failures
obs              1605 checks, 0 failures
policy            624 checks, 0 failures
battguard         132 checks, 0 failures
sil              4024 checks, 0 failures
sil_golden       2959 checks, 0 failures
ALL GREEN
```

There is a `CMakeLists.txt` beside the Makefile for CI (`cmake -S . -B build &&
cmake --build build && ctest --test-dir build`). The Makefile is still the
routinely-exercised path; keep the two in step.

Regenerate every committed golden vector and generated header from the sim:

```
make -C firmware/host vectors     # or run the three tools/ scripts directly
```

## Build and flash for the board

```
env -u VIRTUAL_ENV bash        # export.sh will not run inside a virtualenv
. ~/esp/esp-idf/export.sh
idf.py -C firmware set-target esp32
idf.py -C firmware build
idf.py -C firmware -p /dev/cu.usbserial-XXXX flash monitor
```

The board enumerates over **CP2102N**, so the port is `/dev/cu.usbserial-*` and
the [CP210x driver](https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip)
may be needed on macOS. **There are two USB-C ports and only one flashes**: the
one silkscreened `USB` carries the console with the auto-program circuit; the
one silkscreened `LIDAR` enumerates a port that never syncs. That is the trap.

Opening the serial port **resets the board**, which eats the first command sent
after connecting — drain and verify before any state-writing command.

The board boots into **bench mode with torque released**. Nothing moves until
someone types `run` or arms over the radio. Note that a goal-position write
auto-enables torque on an STS3215, so *any* bus write is a motion command:
never "verify" a calibration against a live robot.

## Module map

```
firmware/
  CMakeLists.txt          ESP-IDF project; -Wall -Wextra -Werror, no exceptions/RTTI
  sdkconfig.defaults      240 MHz, dual core, 1 kHz tick, static allocation, TWDT
  main/                   everything that touches ESP-IDF
    board.h               the confirmed pinout (see "hardware facts" below)
    app_main.cpp          init, then three static tasks
    ctrl_task.cpp         core 1, 50 Hz, esp_timer notification, owns the bus
    link_task.cpp         core 0, UART0 reader + 10 Hz telemetry
    wifi_link.cpp         core 0, UDP command frames in / telemetry beacon out
    imu_sampler.cpp       250 Hz IMU task: attitude filter at 4 ms, tick-averaged
                            gyro to the control loop
    cli.cpp               core 0, the bench CLI
    cal_store.cpp         as-built calibration + IMU mount, CRC'd, in NVS
    obs_dump.cpp          read the live observation off the armed robot
    scs_port_idf.cpp      scsbus::Port over UART1 -- the one hardware seam
    shared.h              the only cross-core state, and the rules for it
    joint_pose.h          measured pose, core 1 -> the WiFi beacon (mirror mode)
    mech_envelope.h       per-joint mechanical limits, enforced below the policy
  components/
    scsbus/               Feetech SCS/STS protocol (pure codec + transactions)
    linkproto/            C++ port of link/protocol.py + Watchdog + UART demux
    obs/                  obs assembler, history ring, gait clock, action maps
    policy/               static-MLP forward pass
    imu/                  Imu interface, mounting maths, StubImu
  host/                   host test target (Makefile + CMakeLists + 7 suites)
tools/
  gen_protocol_vectors.py  link/protocol.py  -> host/vectors/protocol_vectors.h
  gen_obs_spec.py          walker_env       -> obs/obs_spec.h + obs_vectors.h
  gen_policy_weights.py    <run>.silw       -> policy/weights.h + policy_vectors.h
```

Task/core split follows firmware-design section 4: `ctrl` is pinned to core 1 at
`configMAX_PRIORITIES - 2`, woken by a hardware `esp_timer` through
`vTaskNotifyGiveFromISR`; `link` and `housekeeping` sit on core 0; the command
mailbox is a length-1 queue written with `xQueueOverwrite` so a stale command
can never queue up behind a late tick. Every task, queue and buffer is
statically allocated — nothing allocates after `app_main`.

### Who owns the servo bus

The CLI runs on core 0 but needs the bus, which core 1 owns during a run. Rather
than put a mutex in the control path, there is a mode:

| mode | `ctrl` does | CLI may touch the bus |
|---|---|---|
| `kBench` (boot default) | services its watchdog, nothing else | yes |
| `kRun` (`run` command) | the full 50 Hz loop | no — commands are refused |

Handover is one-way through the ctrl task: it releases torque and clears
`g_ctrl_owns_bus` before the CLI is allowed in.

## What is real, and what is still scaffolded

**Real, tested, and running on the robot:**

- **`scsbus/`** — packet framing, checksum, PING, READ/WRITE, REG WRITE +
  ACTION, RESET, SYNC READ, SYNC WRITE, torque enable/release (including
  broadcast), EEPROM lock/unlock, ID change, "set middle position", explicit
  position-correction offset, and the full ST3215 error-flag decode. The golden
  packets in the tests are the **worked examples from Feetech's own protocol
  manual**, so the encoder is diffed against the vendor's bytes. The
  transaction layer is exercised against a scripted fake port, including a
  missing servo mid-sync-read and a reply from the wrong ID.
- **`linkproto/`** — byte-exact port of `link/protocol.py`: CRC-16/CCITT-FALSE
  (`crc16Ccitt("123456789") == 0x29B1`), command and telemetry
  encode/decode, `clamp_to_envelope`, and the `Watchdog` with its
  LIVE/STAND/RELAX/ESTOP semantics, sequence-reorder rejection, "a stale frame
  must not pet the watchdog", and sender-restart resync. Every vector is
  generated by running the Python reference (`tools/gen_protocol_vectors.py`).
- **`obs/`** — the observation frame, checked against **real
  `walker_env._obs()` output** for the deployed 10-DOF spec, plus the history
  ring's ordering, the gait clock's wrap, the `action_map="full"`
  action↔angle map and the angle↔encoder-tick map. Also `obs::CommandShaper`:
  three cascaded 10 Hz first-order lags on the policy targets, so the
  commanded trajectory is C2 (finite jerk) instead of a per-tick slam-and-stop.
- **`policy/`** — the deployed net itself. Currently **`loco_v27tilt_b_s128r24`**: a
  (512, 256, 128) brax MLP distilled by `sim/mjx/distill_student.py` to
  (128, 128) — 38 036 weights + a 147-wide normaliser, 150 KB fp32 of
  `constexpr` in flash DROM — refereed on BOTH scorecard columns before it was
  compiled in. The arithmetic is brax's: `running_statistics`
  normaliser, **swish** hidden layers, a linear `2 * action_size` output, and
  `tanh(mean)` for deterministic inference, checked against numpy golden
  vectors at the 147→128→128→20 widths. `kWeightsArePlaceholder` is false and
  `test_policy` asserts it, along with `kWeightsRun == obs::kRunName`.
- **`imu/`** — the QMI8658C driver (`imu::Qmi8658Imu`), the complementary
  attitude filter (`imu::Fusion`), the up-vector-from-quaternion maths, the
  mounting rotation, and a stub that lets the whole loop run with no part
  fitted. Sampled at 250 Hz by `main/imu_sampler.cpp`, with **I2C bus recovery
  at init** — a reset mid-transaction leaves SDA held low, and without the
  recovery the firmware silently falls back to the `StubImu`.
- **The CLI** — the primary bench interface, exercised daily:
  `scan`, `ping`, `id`, `pos`, `move`, `pose`, `home`, `release`, `torque`,
  `middle`, `reg`, `cal` (save/load/show/migrate/forget), `imu`
  (raw/bias/mount/ring), `wifi`, `obsdump`, `trace`, `shape`, `batt`, `volt`,
  `run`, `bench`, `stat`, `help`. A commanded position is **smooth by default**
  (minimum-jerk pose streaming with a computed duration) — reaching a target
  smoothly is the controller's job, not the caller's.

**Scaffolded — the v2 seams, in the order they will matter:**

1. ~~**The BNO085 driver.**~~ **Done differently, 2026-08-14.** The IMU is
   the QMI8658C on the board itself, not an external breakout:
   `imu::Qmi8658Imu` + `imu::Fusion`, with `imu::StubImu` kept as the
   fallback when the part does not answer. What remains open is the
   **mount quaternion**: the mechanism, NVS record and `imu mount` command
   all exist, but the value is still identity because it can only be
   measured with the board bolted into the pelvis.
2. ~~**WiFi/UDP.**~~ **Done, 2026-08-24.** `main/wifi_link.cpp`: station
   mode, UDP command frames on 4210 into the same mailbox, 10 Hz telemetry
   back to the last commander on 4211 (not gated on `run`, unlike the UART
   path -- there is no CLI text to interleave with, and it makes the link
   verifiable while benched, reported as BENCH). Credentials live in NVS,
   never in git: `wifi <ssid> <psk>` over the tether, bench-only because an
   NVS commit stalls the control tick. Verified two-way from mira with
   `link/verify_udp.py` (disabled stand frames -- no motion).
   **Arming over the link, 2026-08-27:** the `ARM` flag's edges are the
   wireless `run`/`bench` (`linkproto::ArmLatch`, run by ctrl in both modes,
   applied by housekeeping through the same `cmdMode()` as the CLI). The
   operator's console is `link/tui.cpp`; see docs/control-channel.md.
   **And it says why it refused, 2026-08-30:** `cmdMode()` used to print its
   "no as-built calibration in NVS" refusal to the UART sink only, so a
   wireless arm that was declined looked identical to a dead link — six
   seconds of ignored `ARM` frames cost a tether to diagnose. It now
   publishes the verdict to `robot::g_arm_result` (written by `cmdMode` on
   the housekeeping task, and since 2026-08-31 also by ctrl's fall latch,
   which stores `kDisarmedFall` when a fall ends the run) and `wifi_link.cpp`
   folds it, plus
   `g_cal_from_nvs`, into a one-byte diagnostic carried in the `BENCH`
   beacon's `seq_echo` — a field that means nothing while benched, so no
   version bump and every existing commander keeps working. Layout and
   rationale: `linkproto::packDiag` and docs/control-channel.md.
   **Mirror mode beacon, 2026-09-01** (docs/mirror-mode.md): a commander
   whose frames carry `kFlagPose` gets the `kTlmLenExt` beacon (48 B; 40 B
   before the 2026-09-02 timestamp) — the classic body plus ten
   milli-radian joint angles in obs_spec order. The
   pose is ctrl's `g_q`, published every armed tick through
   `main/joint_pose.h` (ObsDump's double-buffer-plus-counter, so a beacon
   can never mix two ticks' joints) and read by `wifi_link.cpp`, which sends
   the long frame only when asked AND the publish count moved since the
   last beacon — a benched loop measures nothing, so a mirror-on client sees
   classic frames until it arms. The beacon's destination and the pose
   request are one `Commander` snapshot replaced from each accepted frame,
   so a long frame can only go to the address that asked for it (the
   two-console trap in docs/mirror-mode.md). Nobody who does not ask sees a
   byte change;
   `wifi` prints `tlm tx N (pose M, asked|not asked)` so the bench check can
   confirm that. The UART tether stays 20 B (link_task.cpp says why).
3. ~~**Calibration persistence.**~~ **Done.** `main/cal_store.cpp` holds the
   as-built joint calibration and the IMU mount quaternion in NVS behind a
   CRC-32, with `cal save/load/show/migrate/forget` on the CLI and
   `main/asbuilt_cal.h` as the compiled-in fallback. Arming is **refused** when
   no as-built calibration is present, and the refusal reason is reported over
   the radio rather than only to the UART.
4. **Telemetry's servo-fault field is 8 bits** but the plant has 10 joints.
   The two hip-yaw faults are currently folded into the spare top bits.
   Widening the frame is a `link/protocol.py` change first, by rule.
5. **Velocity source not settled.** Both the servo's register-58 speed and a
   filtered finite difference are computed each tick; the loop feeds the
   register value. A bench check put the reported-vs-differenced slope at 0.69,
   which is a scale question, not a unit error. Firmware-design section 8
   leaves this to sys-ID.
6. **The IMU mount quaternion** is measurable now that the board is bolted into
   the pelvis; the mechanism, the NVS record and `imu mount` all exist.

## Hardware facts

The board is the Waveshare **General Driver for Robots** (rev 1.2), which
replaced the "Servo Driver with ESP32" on 2026-08-14. `main/board.h` is the
authority and cites the schematic; the differences are not cosmetic:

| fact | value |
|---|---|
| servo bus | UART1, **GPIO 18 = RX, GPIO 19 = TX**, 1 Mbaud, 8N1 — *the one thing unchanged between the two boards* |
| half-duplex direction | **automatic in hardware**, off the TX line. No GPIO. |
| I2C | **GPIO 32 SDA / 33 SCL**, and not an empty bus: QMI8658C IMU @ 0x6B, AK09918C magnetometer @ 0x0C, BMP280 @ 0x77, INA219 power monitor @ 0x42. Header P1 brings the same bus out. |
| battery sense | **INA219 @ 0x42**, seeing pack voltage ahead of the buck. (`battguard` still reads the servo bus; switching it is a separate job.) |
| RGB | GPIO 4, header H2, through a 10 Ω series resistor |
| OLED | **none on this board** — the old SSD1306 at 0x3C does not exist |
| USB | **two CP2102N ports**; only the one silkscreened `USB` flashes |
| module | ESP32-D0WD-V3 rev 3.1, 4 MB flash, PSRAM disabled |

Unresolved: the board's **5 A bus limit against 8–10 servos**, and the fact
that the retired tower mount does not fit this board — see
`docs/wiring.md` and the controller-board notes.

Where this contradicts our docs (all recorded in
[docs/firmware-design.md](../docs/firmware-design.md)'s changelog):

- `wiring.md` describes an **external** IMU breakout (BNO085/BNO055) on GPIO
  21/22. Superseded twice over: the IMU is the **QMI8658C on the board itself**,
  on GPIO 32/33. `main/board.h` and `docs/sensor-expansion.md` are the correct
  account.
- `wiring.md` says the action vector "maps to IDs 1–8 with no permutation
  table". True on the 8-DOF plant; **false on the deployed 10-DOF plant**,
  where the sim's action order puts `L_hip_yaw` at index 0 but the bus numbered
  it 9 — and where the two bus chains went onto the opposite legs from the plan
  (assembly errata, 2026-08-02: **port A is the RIGHT leg**). `obs::kServoId`
  is the generated, host-tested permutation, and `docs/servo-map.md` is the
  as-built table.
- `firmware-design.md` section 5 says the policy uses "tanh/identity
  activations to match brax exactly". brax's `make_ppo_networks` default is
  **`linen.swish`**; only the final `tanh` (the tanh-normal's mode) is a tanh.
- `firmware-design.md` calls the obs "129-dim". The deployed spec is
  **49 per frame × 3 frames = 147**.
- `docs/wiring.md` credited the old board with an ADC divider, then was
  corrected to "no battery sense at all". On *this* board there is one — the
  INA219. `battguard` still reads a servo's register 62 at 0.1 V resolution,
  which remains adequate for wiring.md's 10.5 V floor.
- `components/battguard/` is the pack's only over-discharge protection — an RC
  LiPo has no protection circuit and the servos' own under-voltage flag sits
  ~3 V/cell too low to help. It is pure and host-tested; the thresholds and the
  reasoning are in `docs/wiring.md` "Battery protection". `batt` on the CLI
  shows its state, `batt reset` clears a latched trip (bench mode only).

## On-target bring-up order

Firmware-design section 7's order, now walked end to end: bus echo → single
servo → sync-loop timing capture → IMU cal → torque-release drills → policy
loop with the robot on a stand → floor. The CLI covers the first three; `stat`
reports the tick timing the third one wants. The robot reached the floor and
walked on 2026-09-01.

Bench discipline learned since, worth repeating before any session:

- **Check torque before anything else.** Servos have been found with torque
  dropped overnight, refusing goal writes with no fault flag and no reboot.
- **Any bus write is a motion command** — goal-position writes auto-enable
  torque. Two feet were broken learning this.
- **Opening the serial port reboots the board** and eats the first command.
- A reset-servos verdict means **moved**, not "written": `homeAll` returns
  `HOME_PENDING` and the housekeeping loop reads all ten joints back before
  storing `DISARMED_HOME`, naming the offenders otherwise.
