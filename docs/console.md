# The operator console (`bimo_console`)

The laptop's window onto the robot: a native Qt window (PyQt6 + pyqtgraph)
over the same C++ console core as `bimo_gui` and `bimo_tui`
([`link/client.h`](../link/client.h)). Design and mockups:
[design-v6/2026-11-01-operator-console-ui.md](design-v6/2026-11-01-operator-console-ui.md).
The wire it speaks is [control-channel.md](control-channel.md)'s.

## Running it

```sh
make -C firmware/host console                 # the core library, once per change
uv pip install --python .venv/bin/python PyQt6 pyqtgraph pygame   # the window's deps

.venv/bin/python link/bimo_console.py --host <robot-ip>           # observe
.venv/bin/python link/bimo_console.py --host <robot-ip> --drive   # drive
.venv/bin/python link/bimo_console.py --host 127.0.0.1 --drive    # the sim
```

`--help` lists every flag. `--headless` needs only the core library (no Qt):
the same link loop with line commands on stdin, which is what
[`tests/test_gui_e2e.py`](../tests/test_gui_e2e.py) drives.

## Roles

- **It starts as an observer.** An observer has no transmit socket (the
  core's `Link::listen`), so it cannot put a command on the wire; opening it
  during a tethered run changes nothing on the robot
  ([first-policy-stand.md](first-policy-stand.md) §3). TAKE CONTROL in the
  safety bar (or `--drive`) makes it the commander, adopting the arm level
  the robot reports, so the handover moves nothing; WATCH ONLY hands back
  with no parting disarm.
- **HOLD and E-STOP work in every role.** They are the out-of-band operator
  stops (`kFlagStop`, [`tools/estop.py`](../tools/estop.py)'s frames, sent
  from a thread of their own at 100 Hz for 1 s to the robot and, unless the
  robot is on loopback or `--no-broadcast`, the LAN broadcast). The robot
  never puts them in its command mailbox, so no latch or watchdog sees them
  and no commander can undo them.
  - **HOLD** (`f`): the soft stop. The policy stops, every joint holds where
    it is with torque on, the run ends, and the bench homes the robot once it
    is upright and still. Not confirmed by the beacon within 400 ms, it is
    escalated to a hard stop.
  - **E-STOP** (`e`): the hard stop. Torque off now and the run ends; for a
    driver also the classic E-stop latch, which firmware older than
    `kFlagStop` understands. Re-arming is a fresh ARM.
  - **RESET SERVOS** is refused while a stop is still being sent: a hard-stop
    frame releases torque on a benched robot, so the rest of the burst would
    undo the reset.
  - Without `--host` the stops go to `--stop-host`, `$BIMO_HOST`, or the
    robot address the flight collector last heard, as `tools/estop.py` does.
    The servo power switch is still the last word.
- **Leaving.** Quit and closing the window disarm (10 frames); an observer's
  quit sends nothing.

## Input

- A held key or button is a level: release stands the robot on the next
  frame; auto-repeat changes nothing; the window losing focus releases
  everything.
- While a text field has focus it takes every key, including the keymap's:
  typing a host name with an `a` in it must not arm the robot. The buttons
  never take focus.
- Keys are `~/.bimo/keymap.conf`, the file `bimo_gui` uses; rebind them in
  Setup. Defaults: arrows translate, PgUp/PgDn turn, `a` arm, `space` stand,
  `f` hold, `e` E-stop, `h` reset servos, `r` record, `0` reset the extended
  channels, `q` quit; take control is unbound.

## The window

The safety bar is on every tab: the state badge (FLAIL / STOPPED / FALLEN
when that is why the robot is benched), the controls, pack voltage and tilt,
servo faults by servo ID, the gamepad, recording, the handover button, a
fixed-width link line, the BENCH reason, and the console's last message. The
view pane on the left shows the 3D pose, a camera or the sim's render. Tabs:
Checks, Drive, Joints, Telemetry, Sim, Session, Setup, and Bench when started
with `--bench-telemetry` / `--cam-relay`.

## Data

| source | port | carries |
| --- | --- | --- |
| beacon | UDP 4211 (`--tlm-port`) | state, pack, up vector, faults; joints / attitude when asked |
| robot flight records | UDP 4221 (`--flight-port`) | per joint goal vs measured at 50 Hz; forwarded by mira's `tools/flight_collector.py --forward <laptop>:4221` |
| sim flight records | UDP 4222 (`--sim-flight-port`) | the same, from `sim/sil_twin.py --flight` |
| sim render | HTTP 8645 | `sil_twin`'s MJPEG |
| bench cameras | HTTP 8647 (`--cam-relay`) | `tools/cam_relay.py` |
| bench telemetry | UDP 8650 (`--bench-telemetry`) | `tools/board_bridge.py`, watch-only |

The console records every command and beacon to `hw_sessions/*.jsonl`
(on by default, `bimo_gui`'s schema) and logs to `hw_sessions/bimo_console.log`.

## How it is built

- [`link/console_api.h`](../link/console_api.h): the core as a C library
  (`libbimo_console`), which decides nothing itself; [`link/console/core.py`](../link/console/core.py)
  loads it with ctypes and refuses a stale build.
- [`link/console/controller.py`](../link/console/controller.py): what both
  front ends drive; [`stops.py`](../link/console/stops.py): HOLD / E-STOP;
  [`headless.py`](../link/console/headless.py); [`app.py`](../link/console/app.py):
  the window. The link tick is a timer on the GUI thread, so a frozen window
  sends nothing and the robot stands.
- Tests: `tests/test_gui_e2e.py` (every scenario against both consoles),
  `tests/test_console_core.py` (at the socket: an observer sends only stop
  frames, a driver quits disarmed, an unconfirmed HOLD escalates),
  `tests/test_console_window.py` (offscreen; skips without pyqtgraph),
  `firmware/host/test_client.cpp` (the C boundary under ASan/UBSan).
