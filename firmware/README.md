# firmware/ — Waveshare ESP32 servo board

*Status 2026-07-26, v1. Design doc: [docs/firmware-design.md](../docs/firmware-design.md).
Wire protocol: [docs/control-channel.md](../docs/control-channel.md). Electrical:
[docs/wiring.md](../docs/wiring.md).*

**Nothing here has been flashed.** The board is on the bench running Waveshare's
stock firmware, and flashing is a deliberate, user-gated step. Note that
flashing **overwrites the vendor web UI** used to assign servo IDs — finish
[docs/bringup-day1.md](../docs/bringup-day1.md) §2 first (the CLI's `id`
command can redo it, but the web UI is easier while it's there).

Two independent gates, both green as of 2026-08-03:

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

## Build and flash for the board (build executed; flash not)

```
env -u VIRTUAL_ENV bash        # export.sh will not run inside a virtualenv
. ~/esp/esp-idf/export.sh
idf.py -C firmware set-target esp32
idf.py -C firmware build       # <- verified 2026-07-26, 0 warnings
idf.py -C firmware -p /dev/cu.usbserial-XXXX flash monitor   # <- NOT run
```

The board enumerates over its **CP2102**, so the port is `/dev/cu.usbserial-*`
and the [CP210x driver](https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip)
may be needed on macOS.

Before anyone runs that last line: it **overwrites the vendor firmware**,
including the web UI at `192.168.4.1` that bring-up currently uses to set servo
IDs. Our CLI's `scan` / `id` cover that job, but it is a one-way step until the
vendor image is re-flashed, so it is the user's call and nobody else's.

The board boots into **bench mode with torque released**. Nothing moves until
someone types `run`.

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
    cli.cpp               core 0, the bring-up CLI
    scs_port_idf.cpp      scsbus::Port over UART1 -- the one hardware seam
    shared.h              the only cross-core state, and the rules for it
    joint_pose.h          measured pose, core 1 -> the WiFi beacon (mirror mode)
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

## What is real in v1, and what is scaffolded

**Real, tested, and ready for a bench:**

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
  `walker_env._obs()` output** for the deployed 10-DOF
  `loco_v12knee_warm_s128` spec, plus the history ring's ordering, the gait clock's wrap, the `action_map="full"`
  action↔angle map and the angle↔encoder-tick map.
- **`policy/`** — the deployed net itself. `loco_v12knee_warm` (a
  (512, 256, 128) brax MLP) distilled by `sim/mjx/distill_student.py` to
  (128, 128) — 38 036 weights + a 147-wide normaliser, 150 KB fp32 of
  `constexpr` in flash DROM — and refereed on BOTH scorecard columns before
  it was compiled in: python 52/88 and SIL 48/88 seed-pass, against the
  teacher's 51/88 and 37/88. The arithmetic is brax's: `running_statistics`
  normaliser, **swish** hidden layers, a linear `2 * action_size` output, and
  `tanh(mean)` for deterministic inference, checked against numpy golden
  vectors at the 147→128→128→20 widths. `kWeightsArePlaceholder` is false and
  `test_policy` asserts it, along with `kWeightsRun == obs::kRunName`.
- **`imu/`** — the interface, the up-vector-from-quaternion maths, the mounting
  rotation, and a stub that lets the whole loop run with no part fitted.
- **The CLI** — `scan`, `ping [id]`, `id <old> <new>`, `pos <id>`,
  `move <id> <ticks> [ms]`, `release [id]`, `torque [id]`, `middle <id>`,
  `volt`, `run`, `bench`, `stat`, `help`. Written, not yet run on hardware.

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
3. **Calibration persistence.** `obs::Calibration` exists and is used
   everywhere; loading and storing it in NVS (and a guided zeroing flow) is not
   written. Defaults are zero = 2048 ticks, direction +1.
4. **Telemetry's servo-fault field is 8 bits** but the plant has 10 joints.
   The two hip-yaw faults are currently folded into the spare top bits.
   Widening the frame is a `link/protocol.py` change first, by rule.
5. **Velocity source not settled.** Both the servo's register-58 speed and a
   filtered finite difference are computed each tick; the loop feeds the
   register value. Firmware-design section 8 leaves this to sys-ID.
6. **The OLED is unused.** It faces the deck once mounted (wiring.md), so it is
   a bench-only display; nothing depends on it.

## Hardware facts, confirmed vs. assumed

Confirmed 2026-07-26 from Waveshare's schematic, user manual and firmware
source (URLs are cited in `main/board.h`):

| fact | value |
|---|---|
| servo bus | UART1, **GPIO 18 = RX, GPIO 19 = TX**, 1 Mbaud, 8N1 |
| half-duplex direction | **automatic in hardware** — a PNP off the TX line drives the '125/'126 OE pins. No GPIO. |
| I2C | GPIO 21 SDA / 22 SCL; SSD1306 128×32 at 0x3C is the **only** device on it |
| battery sense | **none** — no divider, no `analogRead()` anywhere in vendor code |
| RGB | 2× WS2812B on GPIO 23, chained out to header H1 |
| module | ESP-32S / WROOM-32 class, 4 MB flash, PSRAM disabled |
| stock firmware | AP `ESP32_DEV` / `12345678`, web UI at 192.168.4.1, **SC-series build** (ST3215 needs the ST build) |

Assumed / unverified: the exact silkscreen module variant (WROOM-32 vs -32D/-32E
vs Ai-Thinker ESP-32S), and every mechanical fit — check the physical board.

Where this contradicts our docs (all recorded in
[docs/firmware-design.md](../docs/firmware-design.md)'s changelog):

- `wiring.md` calls the I2C bus "shared with the IMU". The pins are right but
  **there is no IMU on this board**; the BNO085 is an external breakout on the
  same two pins. `wiring.md` also still describes a BNO055 at 0x28 —
  firmware-design section 3 supersedes it.
- `wiring.md` says the action vector "maps to IDs 1–8 with no permutation
  table". True on the 8-DOF plant; **false on the deployed 10-DOF v3yaw
  plant**, where the sim's action order puts `L_hip_yaw` at index 0 but the bus
  numbered it 9. `obs::kServoId` is the (generated, tested) permutation.
- `firmware-design.md` section 5 says the policy uses "tanh/identity
  activations to match brax exactly". brax's `make_ppo_networks` default is
  **`linen.swish`**; only the final `tanh` (the tanh-normal's mode) is a tanh.
- `firmware-design.md` calls the obs "129-dim". The deployed `loco_v5t` spec is
  **49 per frame × 3 frames = 147**.
- Pack voltage cannot be read on the board; it comes off a servo's register 62
  at 0.1 V resolution, which is still adequate for wiring.md's 10.5 V floor.
  `docs/wiring.md` credited the board with an ADC divider until 2026-07-27;
  `main/board.h` is the correct account and the doc now says so.
- `components/battguard/` is the pack's only over-discharge protection — an RC
  LiPo has no protection circuit and the servos' own under-voltage flag sits
  ~3 V/cell too low to help. It is pure and host-tested; the thresholds and the
  reasoning are in `docs/wiring.md` "Battery protection". `batt` on the CLI
  shows its state, `batt reset` clears a latched trip (bench mode only).

## On-target bring-up order (when the user decides to flash)

Unchanged from firmware-design section 7: bus echo → single servo → sync-loop
timing capture → IMU cal → torque-release drills → policy loop with the robot on
a stand → floor. The CLI covers the first three; `stat` reports the tick timing
the third one wants.
