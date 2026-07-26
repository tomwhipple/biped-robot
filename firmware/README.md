# firmware/ — Waveshare ESP32 servo board

*Status 2026-07-26, v1. Design doc: [docs/firmware-design.md](../docs/firmware-design.md).
Wire protocol: [docs/control-channel.md](../docs/control-channel.md). Electrical:
[docs/wiring.md](../docs/wiring.md).*

**Nothing here has been flashed.** The board is on the bench running Waveshare's
stock firmware, and flashing is a deliberate, user-gated step. What is proven
today is the host test suite; the ESP-IDF project has never been compiled,
because ESP-IDF is not installed on the development laptop.

## Build and test on the host (this is the gate)

```
make -C firmware/host test
```

Compiles the pure modules with `-Wall -Wextra -Werror -Wshadow -Wconversion
-Wsign-conversion` under `-fsanitize=address,undefined` and runs four suites:

```
protocol          240 checks, 0 failures
scsbus            124 checks, 0 failures
obs              1069 checks, 0 failures
policy            433 checks, 0 failures
ALL GREEN
```

There is a `CMakeLists.txt` beside the Makefile for CI (`cmake -S . -B build &&
cmake --build build && ctest --test-dir build`), but cmake is not installed on
this laptop, so **the Makefile is the verified path** and the CMake file is
unproven. Keep them in step.

Regenerate every committed golden vector and generated header from the sim:

```
make -C firmware/host vectors     # or run the three tools/ scripts directly
```

## Build and flash for the board (documented, not executed)

```
. $IDF_PATH/export.sh
cd firmware
idf.py set-target esp32
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

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
  components/
    scsbus/               Feetech SCS/STS protocol (pure codec + transactions)
    linkproto/            C++ port of link/protocol.py + Watchdog + UART demux
    obs/                  obs assembler, history ring, gait clock, action maps
    policy/               static-MLP forward pass
    imu/                  Imu interface, mounting maths, StubImu
  host/                   host test target (Makefile + CMakeLists + 4 suites)
tools/
  gen_protocol_vectors.py  link/protocol.py  -> host/vectors/protocol_vectors.h
  gen_obs_spec.py          walker_env       -> obs/obs_spec.h + obs_vectors.h
  gen_policy_weights.py    (placeholder)    -> policy/weights.h + policy_vectors.h
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
  `walker_env._obs()` output** for the deployed 10-DOF `loco_v5t` spec, plus
  the history ring's ordering, the gait clock's wrap, the `action_map="full"`
  action↔angle map and the angle↔encoder-tick map.
- **`imu/`** — the interface, the up-vector-from-quaternion maths, the mounting
  rotation, and a stub that lets the whole loop run with no part fitted.
- **The CLI** — `scan`, `ping [id]`, `id <old> <new>`, `pos <id>`,
  `move <id> <ticks> [ms]`, `release [id]`, `torque [id]`, `middle <id>`,
  `volt`, `run`, `bench`, `stat`, `help`. Written, not yet run on hardware.

**Scaffolded — the v2 seams, in the order they will matter:**

1. **The policy weights are a placeholder.** `sim/export_policy.py` does not
   exist yet, and the deployed `loco_v5t` net is (512, 256, 128) ≈ 232 k
   params ≈ 930 KB fp32, which does not fit WROOM SRAM — firmware-design
   section 6 already calls for distilling to ~128×128 first. What *is* real is
   the arithmetic: brax's `running_statistics` normaliser, **swish** hidden
   layers, a linear `2 * action_size` output, and `tanh(mean)` for
   deterministic inference, all checked against numpy golden vectors at the
   correct 147→…→20 widths. When the real net lands, only `weights.h` changes.
2. **The BNO085 driver.** `imu::Bno085Imu` does not exist; `imu::StubImu`
   reports level and still. The SH-2 component is the work, and the interface
   is already the seam.
3. **WiFi/UDP.** v1 carries the protocol over UART0 only. The framing is
   identical over both transports by design, so the UDP socket is a second
   producer into the same mailbox, not a second protocol.
4. **Calibration persistence.** `obs::Calibration` exists and is used
   everywhere; loading and storing it in NVS (and a guided zeroing flow) is not
   written. Defaults are zero = 2048 ticks, direction +1.
5. **Telemetry's servo-fault field is 8 bits** but the plant has 10 joints.
   The two hip-yaw faults are currently folded into the spare top bits.
   Widening the frame is a `link/protocol.py` change first, by rule.
6. **Velocity source not settled.** Both the servo's register-58 speed and a
   filtered finite difference are computed each tick; the loop feeds the
   register value. Firmware-design section 8 leaves this to sys-ID.
7. **The OLED is unused.** It faces the deck once mounted (wiring.md), so it is
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

## On-target bring-up order (when the user decides to flash)

Unchanged from firmware-design section 7: bus echo → single servo → sync-loop
timing capture → IMU cal → torque-release drills → policy loop with the robot on
a stand → floor. The CLI covers the first three; `stat` reports the tick timing
the third one wants.
