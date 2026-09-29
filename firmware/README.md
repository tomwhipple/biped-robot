# firmware/ — what runs on the ESP32

The robot's control firmware: an ESP-IDF v5.4, C++17 project for the
Waveshare **General Driver for Robots** (ESP32-D0WD-V3). It boots benched
with torque released, arms over the radio or the USB tether, and runs the
50 Hz policy loop on core 1 with the distilled network compiled into flash.

- Design and rationale: [docs/firmware-design.md](../docs/firmware-design.md)
- Wire protocol and failsafe: [docs/control-channel.md](../docs/control-channel.md)
- First power-up and calibration on the robot: [docs/bringup.md](../docs/bringup.md)
- Servo IDs and calibration values: [docs/servo-map.md](../docs/servo-map.md)
- Electrical: [docs/wiring.md](../docs/wiring.md)
- Bench safety rules: [AGENTS.md](../AGENTS.md#bench-safety)

**Sized for the prototype.** The firmware currently drives the prototype's
10 joints (servo IDs 1–10, plant `sim/bimo_biped_v5body.xml`). The joint
count comes from the generated `obs_spec.h` (`kNumJoints = 10`,
`kServoId = {10, 5, 6, 7, 8, 9, 1, 2, 3, 4}`), and the calibration blob, the
telemetry frame, the SIL ABI and `main/mech_envelope.h` are all 10 wide.
Porting to the robot's 17 joints is open work (issue #81). The compiled-in policy is
`loco_v41rsi_b_s128r24`.

## Build and flash

```sh
env -u VIRTUAL_ENV bash                 # export.sh will not run inside a virtualenv
. ~/esp/esp-idf/export.sh
idf.py -C firmware set-target esp32     # once, for a fresh build directory
idf.py -C firmware build
idf.py -C firmware -p /dev/cu.usbserial-XXXX flash monitor
```

- `export.sh` uses the first `python3` on `PATH`, and it must be the Python
  ESP-IDF's `install.sh` was run with. ninja is optional (`brew install
  ninja` for faster incremental builds).
- The image is 1 008 672 B against the 1 MB app partition (4 % free), and the
  build says so with a warning.
- The board enumerates as a CP2102N (`/dev/cu.usbserial-*`; macOS may need the
  [CP210x driver](https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip)).
  **Only the USB-C port silkscreened `USB` flashes**; the one silkscreened
  `LIDAR` enumerates a port that never syncs.
- **Opening the serial port resets the board**, which eats the first command
  sent after connecting. Drain and verify before any state-writing command.
- The board boots **benched, torque released**. Nothing moves until `run` on
  the tether or an ARM edge over the radio. Any goal-position write enables
  torque on an STS3215, so every bus write is a motion command.

The pre-push hook (`.githooks/pre-push`) also runs the target build; it finds
ESP-IDF through `idf.py` on `PATH` or `$IDF_PATH`.

## On-target bring-up

[docs/bringup.md](../docs/bringup.md) is the procedure, from a bare board to
the first arm: servo IDs, calibration, the IMU's bias, mount and scale taken
on the robot in the standing pose, WiFi and NTP, torque-release drills, then
the policy loop with the robot on a stand before the floor. `stat` reports
the tick timing and fault counters along the way.

## Host tests (the hardware-free gate)

```sh
make -C firmware/host test     # 10 suites under ASan/UBSan, plus libctrl_sil
make -C firmware/host check    # test + the bimo_tui build (+ bimo_gui on macOS)
```

`check` is what the pre-push hook runs, alongside a CMake + `ctest` build of
the same tests (`firmware/host/CMakeLists.txt`; keep it in step with the
Makefile). Everything compiled here is the same source the board runs, built
with `-Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion`; the
hardware-touching files are excluded by construction.

| suite | covers |
|---|---|
| `protocol` | `linkproto` against golden vectors generated from `link/protocol.py`: frames, CRC, watchdog, arm and home latches, bench diagnostic, UART demux |
| `scsbus` | packet codec against the worked examples in Feetech's protocol manual; transactions against a scripted fake port (a missing servo mid-sync-read, a reply from the wrong ID) |
| `obs` | the assembler against real `walker_env._obs()` frames, history order, gait clock, action/angle/tick maps, command shaper, calibration blob pack/unpack |
| `policy` | the forward pass against numpy golden vectors; asserts the weights are the deployed run, not a placeholder |
| `battguard` | the pack guard's thresholds, debounce and landing ramp |
| `fusion` | the attitude filter, cross-checked against MuJoCo's `framezaxis` |
| `obsdump` | the observation dump's double buffer and CSV writer; the mirror-mode pose buffer |
| `sil` | `libctrl_sil`'s C ABI end to end: tick round trips, the bus permutation, `.silw` and calibration parsing on adversarial input |
| `sil_golden` | the firmware against the sim's own numbers in `sim/sil/golden/*.json`. The obs cases always run; the policy cases run only when the golden run's `.silw` is present and are skipped otherwise |
| `client` | the console core in `link/`: intent, frame-length rule, keymap file, MJPEG parser, recorder |

The Python half of software-in-the-loop is [sim/sil/README.md](../sim/sil/README.md);
`pytest sim/sil` is **not** part of the pre-push gate.

Other targets:

```sh
make -C firmware/host sil       # build/libctrl_sil.{dylib,so} only
make -C firmware/host tui       # build/bimo_tui
make -C firmware/host deps gui  # build/bimo_gui (fetches pinned Dear ImGui + stb into third_party/)
make -C firmware/host vectors   # regenerate protocol_vectors.h from link/protocol.py
make -C firmware/host deploy-headers RUN=<run>   # obs_spec.h + weights.h (+ vectors) from one run
```

`obs_spec.h`, `weights.h` and the files in `host/vectors/` are generated
(AGENTS.md lists the generators); never hand-edit them. The two deployed
headers change only together, and only because somebody named the run to
deploy.

## Module map

```
firmware/
  CMakeLists.txt        ESP-IDF project; -Wall -Wextra -Werror, no exceptions/RTTI
  sdkconfig.defaults    240 MHz, dual core, 1 kHz FreeRTOS tick, static allocation,
                        1 s task watchdog, 4 MB flash / no PSRAM, SNTP settings
  main/                 everything that touches ESP-IDF
    board.h             the pinout, from the vendor schematic
    app_main.cpp        NVS, UARTs, IMU, calibration restore, torque off, start tasks
    ctrl_task.cpp       core 1, 50 Hz: sense, guard, observe, act (owns the bus when armed)
    link_task.cpp       core 0: UART0 reader (`link`) and the `housekeeping` task
    wifi_link.cpp       core 0: WiFi station, UDP commands in, telemetry beacon out
    imu_sampler.cpp     core 0: 250 Hz IMU read + fusion, tick-averaged gyro
    timesync.cpp        SNTP wall clock for telemetry timestamps
    cli.cpp             the bench CLI
    cal_store.cpp       servo and IMU calibration blobs in NVS, CRC'd
    asbuilt_cal.h       the as-built zero table (reference for blob migration)
    mech_envelope.h     per-joint mechanical limits for bench moves
    obs_dump.cpp        the observation dump (`obsdump`)
    joint_pose.h        measured pose, core 1 -> the beacon (mirror mode)
    scs_port_idf.cpp    scsbus::Port over UART1 -- the one servo-bus hardware seam
    shared.h            the only cross-core state, and the rules for it
  components/
    scsbus/             Feetech SCS/STS protocol: codec, transactions, registers
    linkproto/          C++ port of link/protocol.py: frames, Watchdog, latches, UART demux
    obs/                obs assembler, history, gait clock, actuation maps, shaper;
                        obs_spec.h (generated)
    policy/             static MLP forward pass; weights.h (generated)
    imu/                Imu interface, frame maths, Fusion, QMI8658C driver, StubImu
    battguard/          pack under-voltage guard
  host/                 host build: Makefile + CMakeLists, 10 test suites, the SIL
                        library (sil.h, sil_lib.cpp, sil_net.cpp), vectors/
tools/
  gen_protocol_vectors.py   link/protocol.py -> host/vectors/protocol_vectors.h
  gen_obs_spec.py           run config + MJCF -> obs/obs_spec.h + obs_vectors.h
  gen_policy_weights.py     <run>.silw -> policy/weights.h + policy_vectors.h
```

## The CLI

USB serial at 115200 on the `USB` port. `help` prints the list. Commands that
touch the bus work only while benched (`run` refuses them); commands that
write NVS are bench-only because a flash write stalls the control tick.

| command | does |
|---|---|
| `scan` | ping IDs 0–253; report position, voltage, faults |
| `ping [id]` | no id: check exactly the policy's servos |
| `id <old> <new>` | assign a servo ID (EEPROM; one servo on the bus) |
| `pos <id>` | position, speed, load, voltage, temperature, faults |
| `move <id> <ticks> [ms] [steps/s]` | one servo; 2048 = middle, 4096 ticks/rev; clamped to the mechanical envelope |
| `pose <t0..t9> [...]` | all ten targets in joint order, streamed as a smooth minimum-jerk move by default |
| `trace` | dump the per-servo trace recorded during the last `pose` stream |
| `reg <id> <addr> [1\|2]` | read a servo register |
| `home [steps/s]` | every joint to its calibrated zero, hip play take-up, readback, torque released |
| `release [id]` / `torque [id]` | torque off / on; no id = broadcast |
| `middle <id>` | latch the current angle as the servo's 2048 (register 40 ← 128), torque off |
| `volt` | pack voltage, read off the servos |
| `batt [reset]` | pack guard state; `reset` clears a latched trip after a pack swap (bench) |
| `cal [show\|zero [j]\|dir j d\|set j z d\|migrate\|save\|load\|reset]` | per-joint zero and direction in NVS |
| `shape [hz]` | command-shaper pole; 0 = raw targets |
| `obsfreeze [none\|up\|gyro\|imu\|dq\|gain=k]` | pin or scale parts of the policy observation (diagnostic, not persisted) |
| `run` / `bench` | hand the bus to the control loop / take it back |
| `imu [scan\|raw [n]\|ring [ms]\|avg [on\|off]\|gscale [x y z]\|reinit\|bias\|mount\|forget\|ae]` | IMU status, I²C scan, raw axes, ringing meter, calibration |
| `wifi [<ssid> <psk>\|clear]` | link status and counters; credentials in NVS |
| `ntp` | SNTP sync state and server |
| `stat` | tick timing per phase, overruns, worst tick, IMU sampler, policy |
| `obsdump [on\|off\|once]` | stream the policy observation as CSV (armed only) |

## Known gaps, as they stand

- The telemetry servo-fault field is 8 bits; joints 8 and 9 are OR-ed into
  bits 0 and 1. `vx_est`/`wz_est` are not estimated and go out as 0.
- `battguard` reads pack voltage from servo register 62 (0.1 V); the on-board
  INA219 is not read.
- The velocity observation is servo register 58; whether a finite difference
  fits better is open (reported/differenced slope 0.69 on the bench).
- The IDF task watchdog on `ctrl` reports only; the loop's own overrun check
  (> 40 ms → torque off) is the protection.
- 10-joint sizing, above.
