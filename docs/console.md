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
- A text field gets the keys only after the operator clicks into it: no tab
  change, `Tab` or window switch puts focus there. While one has focus it
  takes every key, including the keymap's (typing a host name with an `a` in
  it must not arm the robot), and the safety bar's note line says
  **KEYS CAPTURED**; `Escape` or a click anywhere outside the field gives the
  keys back. The exceptions:
  - while the robot is LIVE (this console armed it, or a fresh beacon says
    LIVE), the HOLD and E-STOP keys still fire from inside a field and do
    not type;
  - a key-up always ends a held motion key, whatever has focus, so the
    console never keeps commanding motion with nothing pressed.

  The HOLD and E-STOP buttons always work. The buttons never take focus.
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

- **3D pose** (`view3d.py`, `pose3d.py`): the plant posed from the measured
  joints (flight records, else the beacon's joint block when `--mirror` asks
  for it), kinematics only, root pinned and yaw drawn 0. The commanded pose
  (each joint's goal) is a cyan ghost and the sim twin's an amber one, each its
  own geometry, never a blend. A camera is a picture-in-picture or a split,
  never drawn under the model. Every layer shows its age and reads FROZEN past
  1 s.
- **Checks** (`tab_checks.py`): [first-policy-stand.md](first-policy-stand.md)
  §2. Rows the console can read pass or fail by themselves; the rest are
  ticked by the operator. ARM reads "ARM · checks open" until the list is
  clear; it is never blocked. The abort watch (§5) is advisory and sends
  nothing.
- **Drive**: hold-to-move buttons, what was sent beside what the policy saw
  (the flight record's command), the extended channels, speeds and send rate,
  30 s of pack, tilt and vx.
- **Joints** (`tab_joints.py`): goal vs measured for every bus joint on the
  robot's clock, error RMS and load, a faulted servo boxed red ("reading
  held"), the sim twin as an amber trace; click a joint to highlight it in 3D.
- **Telemetry** (`tab_telemetry.py`): pack, tilt, gyro, tick work time, beacon
  age and flight-record holes on a time axis; the servo table; the link table.
  Records from the sim twin (its HELLO's firmware reads `sil_twin ...`, or
  they arrive on `--sim-flight-port`) are labelled the sim's, and the
  servo fields the sim has no model of (load, current, voltage, temperature,
  status) show `—` in Joints, Telemetry and Checks, never `0`.
- **Sim**: start, stop and reset `sim/sil_twin.py`, its log, and
  twin-in-the-loop (off by default): every command frame sent to the robot is
  also sent to the sim on its own ports. A run is listed when it has both
  `sim/runs/<run>/config.json` and `sim/sil/weights/<run>.silw.json`; with
  none, the tab says how to get one (`sim/runs/` is not in git).
- **Session**: the recording, the flight recorder's state, and one event log
  for the console and the robot.
- **Setup**: the link (applied on the button; a relink while armed disarms
  first), the beacon relay, the keymap, the gamepad mapping, a screenshot.
- **Bench** (`bench.py`, `tab_bench.py`): the serial-bridge telemetry and two
  camera panes, watch-only ([bench-console.md](bench-console.md)).

## Gamepad

`gamepad.py`, through SDL's GameController API (pygame), mapped by position:
the right shoulder (RB) is a held dead-man, east (B) is E-STOP in every role,
the left stick is vx (the trained band) and vy, the right stick x is wz; the
pad cannot arm or reset the servos. Losing the pad releases everything. **It
drives the sim only** (a loopback host) until the Bluetooth disconnect and
silence latencies are measured on the pad
([design §7, §14](design-v6/2026-11-01-operator-console-ui.md)): `tools/pad_latency.py`
is that measurement. Its E-STOP works for any host.

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
