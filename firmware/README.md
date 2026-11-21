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

**Two joint sets.** The **bus** is the robot's 17 servos
(`components/obs/include/obs/bus_map.h`, the proposed ID map of
[docs/servo-map.md](../docs/servo-map.md) §2.1): the calibration blob,
`main/mech_envelope.h`, `pose`/`home`, the gain check, the telemetry joint
block and the SIL arrays are all 17 wide. The **policy** is what the compiled
network observes and commands, from the generated `obs_spec.h`: the
v6 legs' 12 joints (`kNumJoints = 12`,
`kServoId = {9, 51, 2, 3, 4, 12, 10, 52, 6, 7, 8, 11}`, plant
`build/v6/sim/bimo_biped_v6legs.xml`), policy `robot_legs_walk16_act_s128r24`. The loop drives
the policy's joints and holds every other fitted servo; the legs-only robot's
calibration marks the neck and arms (IDs 13–17) not fitted
([docs/firmware-design.md](../docs/firmware-design.md) §5.8).

## Build and flash

```sh
make firmware                           # the policy headers, then the image, in build/firmware
env -u VIRTUAL_ENV bash                 # export.sh will not run inside a virtualenv
. ~/esp/esp-idf/export.sh
idf.py -C firmware -B build/firmware -p /dev/cu.usbserial-XXXX flash monitor
```

- The policy headers the image compiles in (`obs/obs_spec.h`,
  `policy/weights.h`, `linkproto/cmd_envelope.h`) are generated, into
  `build/policy/<set>/`, from the deployed run's inputs in `deployed/`. `make
  firmware` runs `make policy` first, so it needs the repo's Python venv and
  the plant (`make plant-data`) as the tests do. A bare `idf.py build` refuses
  to configure until `make policy` has run.

- `export.sh` uses the first `python3` on `PATH`, and it must be the Python
  ESP-IDF's `install.sh` was run with. ninja is optional (`brew install
  ninja` for faster incremental builds).
- The partition table is `partitions.csv`: a 3 MB app partition, with `nvs`
  and `phy_init` at the stock offsets so a reflash keeps the calibration. The
  image is 1 012 832 B, 68 % of the app partition free.
- `sdkconfig` is gitignored and `sdkconfig.defaults` only seeds a new one. A
  checkout configured before the partition table changed is refused at
  configure time: delete `firmware/sdkconfig` and build again.
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

**Which policy.** The image carries the deployed policy unless a **policy
profile** is named: `make firmware POLICY=v6legs` (`idf.py -C firmware -B
build/firmware-v6legs -DBIMO_POLICY=v6legs build`) builds the v6 legs-only
robot's 12-joint policy from `build/policy/v6legs/` instead. Give each
profile its own build directory: `BIMO_POLICY` is a CMake cache variable. The
policy sets are the lines of `deployed/policies.txt`. The
`v6legs` profile holds the (128, 128) walk16_act student
([training.md §9](../docs/training.md), "Policy profiles"). `v6legs256` holds an **int8** (256, 256) deploy candidate
(`robot_legs_walk16_act_s256r24`, the fast-servo distill of walk16_act;
experiments/policy/int8-256), the way a net wider than (128, 128) fits the tick
([firmware-design.md §6.1](../docs/firmware-design.md)).

**Which power board.** Off by default: an image for a robot **with power lite**
is a separate build, `make firmware POWER_LITE=1` (`idf.py -DBIMO_POWER_LITE=ON`,
into `build/firmware-powerlite`; it combines with `POLICY=`). Such an image can
never arm a robot without the board, because a floating `ESTOP_N` reads as stop. A
default image does not touch IO4, IO16 or GPIO34.
[The power-lite README](../pcb/power-lite/README.md), "The firmware", says what it
does.

## On-target bring-up

[docs/bringup.md](../docs/bringup.md) is the procedure, from a bare board to
the first arm: servo IDs, calibration, the IMU's bias, mount and scale taken
on the robot in the standing pose, WiFi and NTP, torque-release drills, then
the policy loop with the robot on a stand before the floor. `stat` reports
the tick timing and fault counters along the way.

## Host tests (the hardware-free gate)

```sh
make -C firmware/host test     # 11 suites under ASan/UBSan, plus libctrl_sil -- the deployed policy, then each profile
make -C firmware/host check    # test + the bimo_tui build (+ bimo_gui on macOS)
```

`check` is what the pre-push hook runs, alongside a CMake + `ctest` build of
the same tests (`firmware/host/CMakeLists.txt`; keep it in step with the
Makefile). Everything compiled here is the same source the board runs, built
with `-Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion`; the
hardware-touching files are excluded by construction.

| suite | covers |
|---|---|
| `protocol` | `linkproto` against golden vectors generated from `link/protocol.py`: frames, CRC, watchdog, arm and home latches, bench diagnostic, UART demux; the volume clamp bit for bit against `sim/cmd_kernel.volume_golden()` (a literal table, `tests/test_cmd_kernel.py` keeps it equal), and a volume-trained deployed run's own ellipse from its generated vectors |
| `scsbus` | packet codec against the worked examples in Feetech's protocol manual; transactions against a scripted fake port (a missing servo mid-sync-read, a reply from the wrong ID); the guarded gain write (refused with torque on or unreadable, re-locks on failure, never a goal) and the gain check |
| `obs` | the assembler against real `walker_env._obs()` frames, history order, gait clock, action/angle/tick maps, command shaper, the motion limiter (bounds, braking, no overshoot, S-curve), the bus map against the policy spec, calibration blob pack/unpack and the v2 → v3 and v1 migrations |
| `policy` | the forward pass against numpy golden vectors; asserts the weights are the deployed run, not a placeholder (a policy profile may carry an untrained test net, which the firmware refuses to arm on) |
| `battguard` | the pack guard's thresholds, debounce and landing ramp; the INA219's bus-register decode, read schedule (10 Hz, 2 s once missing) and staleness; the guard's input from both senses (the lower) |
| `roll_pi` | the stance-leg roll PI: sim-twin golden rows, anti-windup, abort/IMU freeze, mirror symmetry, roll from the up vector |
| `fallguard` | the shared fall detector: the armed (45 deg / 200 ms) and bench (30 deg / 100 ms) limits, debounce, no decision on a stale attitude, the 1 s upright-and-still (25 deg, 1 rad/s) pick-up, the armed-to-bench hand-over -> autohome |
| `flailguard` | the armed loop's flail guard: pose / rate / tilt / chatter each trips on what it should and not on a short excursion, a walk command (pose), the takeover ramp (pose) or a smooth gait; and a replay of the 2026-11-01 flailing stand (`vectors/flail_vectors.h`): quiet for the clean first 3 s, trips by 3.5 s |
| `cmdslew` | the operator command's slew limit, bit for bit against `sim/cmd_kernel.golden()` (a literal 120 x 3 table; `tests/test_cmdslew_golden.py` keeps it equal to the Python law), disabled and per-channel zero rate pass through, reset to the stand, a limp period restarts it at the stand (`tests/test_cmdslew_golden.py` pins the reset in `ctrl_task.cpp`'s limp branch) |
| `fusion` | the attitude filter, cross-checked against MuJoCo's `framezaxis` |
| `obsdump` | the observation dump's double buffer and CSV writer; the mirror-mode pose buffer |
| `sil` | `libctrl_sil`'s C ABI end to end: tick round trips, the bus permutation, the held bus slots, `.silw` and calibration parsing on adversarial input |
| `sil_golden` | the firmware against the sim's own numbers in the set's `golden/*.json` (`make policy`). The obs cases always run; the policy cases run against the exported `.silw` beside them (`weights/`), or else against the compiled-in net |
| `flightrec` | the flight recorder's tick/event/hello encoders byte-for-byte against vectors from `tools/flightrec.py` (the collector's decoder), decode round trips and rejects, the producer ring |
| `client` | the console core in `link/`: intent, frame-length rule, keymap file, MJPEG parser, recorder, the servo-fault ID list |
| `wcli` | the Wi-Fi CLI ([docs/wifi-cli.md](../docs/wifi-cli.md)): SHA-256/HMAC against FIPS/RFC 4231 vectors, the wire format against vectors from `link/wcliproto.py`, the session's authentication, replay and cross-session replay, at-most-once execution, cancel, busy, the output window, and the Wi-Fi allowlist |

The Python half of software-in-the-loop is [sim/sil/README.md](../sim/sil/README.md);
`pytest sim/sil` runs in the pre-push hook after `check` (it needs the library
`check` builds), against the deployed run.

Other targets:

```sh
make -C firmware/host sil       # build/libctrl_sil.{dylib,so} only
make -C firmware/host tui       # build/bimo_tui
make -C firmware/host deps gui  # build/bimo_gui (fetches pinned Dear ImGui + stb into third_party/)
make -C firmware/host build/wcli_twin   # the Wi-Fi CLI's robot half on a localhost socket (tests/test_wcli.py)
make -C firmware/host test POLICY=v6legs   # one policy profile's build (`test` alone runs every profile too)
```

At the repo root:

```sh
make policy                     # every policy set, from deployed/, into build/policy/<set>/
make vectors                    # protocol, flight-recorder, Wi-Fi CLI vectors into build/host/vectors/
make deploy RUN=<run>           # deploy a run: its inputs into deployed/, then make policy
make deploy RUN=<run> POLICY=v6legs   # point a policy profile at a run (QUANT=int8: an int8 net)
```

`make host` builds both first; `make -C firmware/host` alone does not, and
says so. The policy headers (`obs/obs_spec.h`, `policy/weights.h`,
`linkproto/cmd_envelope.h` and its Python twin `cmd_envelope.py`, which
`link/protocol.py` reads), their vectors and the SIL golden files are
generated per set into `build/policy/<set>/`, never committed and never
hand-edited (AGENTS.md lists the generators). Git tracks their inputs in
`deployed/`: `policies.txt` (each set's run and net format), `runs/<run>/`
(`config.json`, `params.pkl`), `q8/<set>.npz` (an int8 set's kernels, quantized
on the plant of its deploy day) and `headers.sha256` (the headers' bytes as
deployed, which every rebuild must reproduce). The headers change only
together, and only because somebody ran `make deploy`; commit `deployed/`
after it. `host/vectors/flail_vectors.h` stays committed: it comes from a
gitignored recording.

## Module map

```
firmware/
  CMakeLists.txt        ESP-IDF project; -Wall -Wextra -Werror, no exceptions/RTTI
  sdkconfig.defaults    240 MHz, dual core, 1 kHz FreeRTOS tick, static allocation,
                        1 s task watchdog, 4 MB flash / no PSRAM, SNTP settings,
                        the custom partition table
  partitions.csv        nvs + phy_init at the stock offsets, a 3 MB app partition
  deployed/             what the policy headers are generated from: policies.txt (the
                        sets), runs/<run>/ (config.json, params.pkl), q8/, headers.sha256
  main/                 everything that touches ESP-IDF
    board.h             the pinout, from the vendor schematic
    app_main.cpp        NVS, UARTs, IMU, calibration restore, torque off, start tasks
    ctrl_task.cpp       core 1, 50 Hz: sense, guard, observe, act (owns the bus when armed)
    link_task.cpp       core 0: UART0 reader (`link`) and the `housekeeping` task
    wifi_link.cpp       core 0: WiFi station, UDP commands in, telemetry beacon out
    wifi_cli.cpp        core 0: the authenticated Wi-Fi CLI on UDP 4213 (docs/wifi-cli.md)
    imu_sampler.cpp     core 0: 250 Hz IMU read + fusion, tick-averaged gyro
    timesync.cpp        SNTP wall clock for telemetry timestamps
    cli.cpp             the bench CLI
    cal_store.cpp       servo and IMU calibration blobs in NVS, CRC'd
    asbuilt_cal.h       the as-built zero table (reference for blob migration)
    servo_gains.h       expected position-loop P/D per servo ID (the arm gate)
    mech_envelope.h     per-bus-joint mechanical limits for bench moves
    obs_dump.cpp        the observation dump (`obsdump`)
    net_bench.cpp       `nettime`: the policy forward pass timed every way the image can run it
    flight_rec.cpp      the flight recorder: rings, UDP sender, `rec` config (docs/flight-recorder.md)
    joint_pose.h        measured pose, core 1 -> the beacon (mirror mode)
    scs_port_idf.cpp    scsbus::Port over UART1 -- the one servo-bus hardware seam
    shared.h            the only cross-core state, and the rules for it
    power_rail.cpp      power lite (a build option): SERVO_EN, ESTOP_N, RAIL_V; the rail's
                        hardware half and the reset fail-safe
    pack_sense.cpp      the INA219's pack voltage, 10 Hz on housekeeping, bounded I2C reads
  components/
    railguard/          power lite's rail state machine: arm, drops, E-stop latch,
                        stuck rail (pure, host-tested)
    cmdslew/            the operator command's slew limit on (vx, vy, wz), the twin of
                        sim/cmd_kernel.py (pure, host-tested); envelope_rates.h picks the
                        deployed run's rates: none generated = off
    scsbus/             Feetech SCS/STS protocol: codec, transactions, registers;
                        the guarded gain write and the gain check (gains.h)
    linkproto/          C++ port of link/protocol.py: frames, Watchdog, latches, UART demux;
                        (cmd_envelope.h, generated: the trained command envelope)
    obs/                obs assembler, history, gait clock, actuation maps, shaper;
                        bus_map.h (the 17-servo bus); (obs_spec.h, generated)
    policy/             static MLP forward pass; (weights.h, generated); q8.h/.cpp and
                        q8_mac16.S, the int8 forward pass of a quantized profile
    imu/                Imu interface, frame maths, Fusion, QMI8658C driver, StubImu
    battguard/          pack under-voltage guard; pack.h: the INA219 sense, pure
    balance/            stance-leg roll PI for the bench one-leg stand (pose B<L|R>), and the
                        stance-hip trim through the lift (pose S<L|R>, `trim`)
    gaitplay/           the gait track (`track`): keyframes appended while playing, the phase ->
                        support state the stance trim runs on
    fallguard/          fall detection + pick-up recovery, one module for the armed loop and the bench
    flailguard/         the armed loop's judge of the policy (pose, rate, tilt, chatter) -> hold + disarm
    flightrec/          flight-recorder wire format (tick/event/hello) and the lock-free ring
    wcli/               the Wi-Fi CLI: SHA-256/HMAC, wire codec, session logic, allowlist
  host/                 host build: Makefile + CMakeLists, 15 test suites, the SIL
                        library (sil.h, sil_lib.cpp, sil_net.cpp), wcli_twin,
                        vectors/flail_vectors.h
build/policy/<set>/     generated (make policy): obs/, policy/, linkproto/, vectors/,
                        golden/, weights/ -- first on the include path
build/host/vectors/     generated (make vectors): the protocol, flightrec, wcli vectors
tools/
  policy_profile.py         the policy sets: make policy's driver, make deploy
  gen_protocol_vectors.py   link/protocol.py -> protocol_vectors.h
  gen_flightrec_vectors.py  tools/flightrec.py -> flightrec_vectors.h
  gen_wcli_vectors.py       link/wcliproto.py -> wcli_vectors.h
  gen_obs_spec.py           run config + MJCF -> obs/obs_spec.h + obs_vectors.h
  gen_envelope.py           run config.json (+ env defaults) -> linkproto/cmd_envelope.h,
                            linkproto/cmd_envelope.py + cmd_envelope_vectors.h
  gen_policy_weights.py     <run>.silw -> policy/weights.h + policy_vectors.h (--quant int8:
                            int8 kernels from deployed/q8/, or GPTQ-rounded afresh at
                            deploy, policy_q8.py)
  policy_q8.py              numpy mirror of the int8 forward pass; offline GPTQ rounding
```

### The operator command's path

A link frame is clamped to the trained envelope (`clampToEnvelope` / `clampExtToEnvelope`), accepted
by the `Watchdog` (which answers the stand once the link is lost), and read each control tick by
`ctrl_task.cpp`. When the deployed run was trained with `cmd_slew`, `tools/gen_envelope.py` writes
its rates into `linkproto/cmd_envelope.h` (`LINKPROTO_CMD_SLEW`, `kSlewVx/Vy/Wz`) and
`cmdslew::CommandSlew` (`g_cmd_slew`) moves (vx, vy, wz) toward the command by at most rate x 20 ms
per tick, ahead of the observation: a stick step reaches the policy as the ramp it was trained on.
Every way the operator's command falls to the stand ramps at the same rates, because the watchdog's
stand is the slew's target like any other command:

- **releasing ENABLE** (the dead-man): the next frame carries the stand, and the ramp starts there;
- **a lost link**: the watchdog holds the last command for its stale delay (`kStaleMs`, 250 ms),
  then answers the stand, and the ramp starts then.

Each channel ramps on its own, |command| / rate: at rates 0.5, 0.4, 1.5 from the trained edges
(vx 0.24 m/s forward, vy 0.25 m/s, wz 1.0 rad/s) that is vx 0.48 s, vy 0.625 s and wz 0.67 s (0.8 s
for vx from a 0.4 m/s backward walk), plus the 250 ms on a lost link. Without those rates (every
run trained without `cmd_slew`) the limiter is off and the command passes through unchanged.

The slew restarts at the stand on every (re)arm and on every limp tick (RELAX, a link E-stop, a
fall, the pack guard's release): torque can come back without a re-arm, and the policy must not be
handed the walk from before the limp period. Disarm, E-stop, the fall and flail guards and the pack
latch act outside it and are immediate. `libctrl_sil` takes the observation's command as given:
slewing it is the caller's job.

When the deployed run was trained with `cmd_volume`, `tools/gen_envelope.py` also writes its planar
ellipse (`LINKPROTO_CMD_VOLUME`, `kVolFwd/Back/Lat`), and the watchdog clamps (vx, vy) together after
the per-axis bands (`clampToVolume`, the twin of `sim/cmd_kernel.py` `volume_clamp`): a corner the
bands pass but the volume never drew (vx 0.24, vy 0.25 is normalized radius √2) is scaled back along
its own direction onto the ellipse. A command on an axis is left to the bands. Without it (every run
trained without `cmd_volume`) the clamp is per axis, as before.

## The CLI

USB serial at 115200 on the `USB` port, and the same commands over Wi-Fi
through the authenticated Wi-Fi CLI (`tools/wcli.py`; bench only, and the
NVS/EEPROM writers stay tether-only -- [docs/wifi-cli.md](../docs/wifi-cli.md)
has the list). `help` prints the list. Commands that touch the bus work only
while benched (`run` refuses them); commands that write NVS are bench-only
because a flash write stalls the control tick.

| command | does |
|---|---|
| `scan` | ping IDs 0–253; report position, voltage, faults |
| `ping [id]` | no id: every bus servo, against the fitted set |
| `id <old> <new>` | assign a servo ID (EEPROM; one servo on the bus) |
| `pos <id>` | position, speed, load, voltage, temperature, faults |
| `move <id> <ticks> [ms] [steps/s]` | one servo; 2048 = middle, 4096 ticks/rev; clamped to the mechanical envelope |
| `pose <17 ticks> [...]` / `pose <10 ticks> [...]` | 17 targets in servo-ID order (every bus servo), or the policy's joints in policy order; unfitted servos ignored; streamed as a smooth minimum-jerk move by default |
| `trace` | dump the per-servo trace recorded during the last `pose` stream |
| `reg <id> <addr> [1\|2]` | read a servo register |
| `gains [id [P D]]` | position-loop P/D (registers 21/22): no id checks every driven servo against `main/servo_gains.h` (the arm gate); `<id>` reads one; `<id> <P> <D>` writes one, only with its torque off (EEPROM unlock, write, lock, read back; no goal) |
| `home [steps/s]` | every joint to its calibrated zero, hip play take-up, readback, torque released |
| `release [id]` / `torque [id]` | torque off / on; no id = broadcast |
| `middle <id>` | latch the current angle as the servo's 2048 (register 40 ← 128), torque off |
| `volt` | pack voltage: the INA219's (no servo needed), then off the servos |
| `batt [reset\|pack on\|off]` | pack guard state and both senses; `reset` clears a latched trip after a pack swap (bench); `pack off` guards on the servos only, until reboot (tether) |
| `cal [show\|zero [j]\|dir j d\|set j z d\|fit j 0\|1\|migrate\|save\|load\|reset]` | per-bus-joint zero, direction and fitted flag in NVS; `j` is a joint name or a servo ID |
| `shape [hz]` | command-shaper pole; 0 = raw targets |
| `limit [off\|on\|vmax [amax [jmax]]]` | per-joint motion limiter (deg/s, deg/s², deg/s³; braking look-ahead, optional jerk boxcar; its velocity is the goal speed); off at boot, `on` = 200 / 2000 / no jerk limit |
| `obsfreeze [none\|up\|gyro\|imu\|dq\|gain=k]` | pin or scale parts of the policy observation (diagnostic, not persisted) |
| `run` / `bench` | hand the bus to the control loop / take it back; `run` refuses without an NVS calibration or with any servo's gains off the expected table |
| `imu [scan\|raw [n]\|ring [ms]\|avg [on\|off]\|gscale [x y z]\|reinit\|bias\|mount\|forget\|ae]` | IMU status, I²C scan, raw axes, ringing meter, calibration |
| `wifi [<ssid> <psk>\|clear]` | link status and counters; credentials in NVS |
| `wcli [key <64 hex>\|key clear\|drop]` | the Wi-Fi CLI: key fingerprint, session, counters; the shared key in NVS (tether only, bench) |
| `ntp` | SNTP sync state and server |
| `rec [status\|<ip>[:port]\|on\|off\|idle <hz>]` | the flight recorder: one record per control tick to the UDP collector on mira ([docs/flight-recorder.md](../docs/flight-recorder.md)); address, on/off and idle rate in NVS |
| `stat` | tick timing per phase, overruns, worst tick, IMU sampler, policy (`net:` int8/float, layers in RAM, kernel, boot self-test) |
| `nettime [n]` | benched: time the policy forward pass every way this image can run it (float/int8, flash/RAM, C/MAC16 kernel, per layer, swish), min/median/max µs over n passes on core 1 ([firmware-design.md §6.1](../docs/firmware-design.md)) |
| `obsdump [on\|off\|once]` | stream the policy observation as CSV (armed only) |

## Known gaps, as they stand

- `vx_est`/`wz_est` are not estimated and go out as 0.
- The INA219's reading is not yet checked against a meter on the robot; until it
  is, `batt pack off` is the way to take it out of the guard.
- The velocity observation is servo register 58; whether a finite difference
  fits better is open (reported/differenced slope 0.69 on the bench).
- The IDF task watchdog on `ctrl` reports only; the loop's own overrun check
  (> 40 ms → torque off) is the protection.
- The 17-servo bus is verified on the host only: a 17-servo `pose` on the
  bench, the robot's leg envelope rows and a policy trained on the robot's
  plant (#86) remain (docs/firmware-design.md §8).
