# Wireless command channel

*The link between a laptop console and the robot. `link/protocol.py` is the
reference implementation; `firmware/components/linkproto` is its byte-exact
C++ port, diffed against golden vectors the Python generates
(`tools/gen_protocol_vectors.py`). Tests: `tests/test_protocol.py`,
`firmware/host/test_protocol.cpp`, and the console end-to-end tests
`tests/test_tui_e2e.py` and `tests/test_gui_e2e.py`. How the firmware runs
it: [firmware-design.md](firmware-design.md). The robot drawn from its own
telemetry: [mirror-mode.md](mirror-mode.md).*

The USB-C tether stays for flashing and bench work; running the robot does
not need it.

## The idea: the radio carries intent, not the control loop

The 50 Hz policy loop runs on the ESP32, beside the servo bus. The radio
carries only the operator's intent — the command vector
`walker_env.set_command()` takes (`vx`, `wz`, optionally `vy`, crouch, lift
and foot offsets) — at 20 Hz.

- **A lost packet costs staleness, never a bad joint angle.** The worst a
  dropped frame does is leave the robot tracking the previous intent for
  another 50 ms.
- **The failsafe is a trained behaviour, not an emergency pose.** A stale
  link decays to the zero command, and training draws that stand command a
  fixed share of the time (`cmd_stand_prob`, default 0.3). Losing the radio
  puts the robot in its most-practised state.
- **Bandwidth is a non-issue**: 14 B at 20 Hz is 2.2 kbit/s.

## Wire format

Little-endian (ESP32 and host are both LE). **Identical framing over UDP and
UART0**: one encoder, both paths. On UART0 the frames share the line with
CLI text, and `linkproto::Demux` separates the two.

**Command**, console → robot, UDP port 4210, 20 Hz, 14 B or 24 B:

| off | size | field | notes |
|---|---|---|---|
| 0 | 2 | magic | `"BM"` |
| 2 | 1 | version | 2 |
| 3 | 1 | flags | below |
| 4 | 4 | seq | u32, monotonic per sender |
| 8 | 2 | vx | i16 mm/s, body-frame forward |
| 10 | 2 | wz | i16 mrad/s, yaw rate |
| 12 | 10 | vy, crouch, lift, foot_dx, foot_dz | 5 × i16 milli-units, **24 B frame only** |
| last | 2 | crc16 | CRC-16/CCITT-FALSE over everything before it |

Length selects the layout. A 14 B frame decodes with the extra channels at
their trained defaults (0, 1.0, 0, 0, 0), and senders emit the 24 B frame
only when an extra is off its default. The UART path takes 14 B frames only.

| bit | flag | meaning |
|---|---|---|
| 0 | `ENABLE` | 0 = stand still whatever the channels say |
| 1 | `ESTOP` | latching torque release |
| 2 | `ARM` | the operator wants the loop armed; the robot acts on its edges |
| 3 | `POSE` | beacon joint angles too (a level) |
| 4 | `HOME` | reset the servos to the stand (rising edge) |
| 5 | `ATT` | beacon the torso up vector too (a level) |

**Telemetry**, robot → the last console that commanded it, UDP port 4211,
10 Hz:

| off | size | field | notes |
|---|---|---|---|
| 0 | 2 | magic | `"BT"` |
| 2 | 1 | version | 2 |
| 3 | 1 | state | `LinkState`, wire value below |
| 4 | 4 | seq_echo | u32, the last command seq applied; the [bench diagnostic](#saying-why-the-bench-diagnostic) while `BENCH` |
| 8 | 2 | vbat | u16 mV |
| 10 | 2 | up_z | i16 milli; torso up-vector z, 1.0 = upright |
| 12 | 2 | vx_est | i16 mm/s; the firmware does not estimate it and sends 0 |
| 14 | 2 | wz_est | i16 mrad/s; likewise 0 |
| 16 | 4 | servo_err | u32, bit *b* = servo ID *b* + 1 faulted (no reply, or a status flag); only servos fitted in the calibration set a bit |
| 20 | 1 | loop_late_pct | % of control ticks over 20 ms |
| 21 | 8 | t_us | u64 µs since the Unix epoch, UTC, robot clock; **0 = not synced** |
| 29 | 34 | joints | 17 × i16 milli-rad, one per bus servo in servo-ID order (`JOINT_NAMES`); 0 for a servo the robot does not carry; **only when asked with `POSE`** |
| next | 4 | up_x, up_y | 2 × i16 milli; **only when asked with `ATT`** |
| last | 2 | crc16 | over everything before it |

Four lengths: 31 base, 35 + attitude, 65 + joints, 69 + both. Every block
has a fixed size, so the length alone selects the layout. The UART tether
sends the 31 B frame only, and only while armed: it shares 115200 baud with
CLI text and the observation dump.

The rules behind the format:

- **Fixed-point milli-units**, not float32, so the frame is byte-identical
  from Python and C.
- **A CRC although UDP checksums**: it rejects stray traffic on an open port
  and covers UART0, which has no checksum at all.
  `crc16_ccitt(b"123456789") == 0x29B1`, the standard check value, so a C
  port can be diffed against a number rather than against "whatever Python
  said".
- **Telemetry blocks are requested, never volunteered** (`POSE`, `ATT`). The
  robot is the sender and every client rejects a length it does not know, so
  an unrequested new length would blind every console at once. A client that
  asks is by construction one that can read the answer.
- **The timestamp is the one exception**: it is on every frame, because its
  value is that any frame from any console can be laid against a video.
- **The joint block and `servo_err` cover the robot's bus, not the policy.**
  They carry all 17 servos (`firmware/components/obs/include/obs/bus_map.h`,
  [servo-map.md](servo-map.md) §2.1) whatever the compiled policy drives; a
  consumer maps the angles by name. The firmware static-asserts
  `linkproto::kNumJoints` against the bus map, and a change to the bus
  changes `JOINT_NAMES` in `link/protocol.py` first.
- **A change the length cannot express bumps the version, in both
  directions.** Version 2 widened `servo_err` from one byte to a u32 and the
  joint block from the prototype's 10 policy joints to the 17 bus servos:
  the base block itself changed. A version-1 frame of either kind is refused
  (`kBadVersion`), so an old console and a new robot refuse each other
  outright instead of one driving blind. Consoles and firmware from one tree
  always agree; a robot flashed before the bump needs a reflash before a
  console from this tree can command it.

## Link states (the failsafe)

![Link-state filmstrip](control-channel-failsafe.png)

*Sim over real UDP, a trained policy, commander killed mid-stride. The stripe
is the watchdog's per-frame state: amber `STAND` (armed, no commander yet) →
green `LIVE` (walking under command) → amber `STAND` (radio dead, decayed to
the stand) → red `RELAX` (torque released; a straight-legged robot stays up,
which is why RELAX is verified by torque, not by eye).*

`link/protocol.py:Watchdog` is the reference. It takes its clock as an
argument, so link loss is tested exactly rather than slept through.

| state | wire | entered when | robot does |
|---|---|---|---|
| `LIVE` | 0 | a valid frame less than 250 ms ago | track the command |
| `STAND` | 1 | no valid frame for 250 ms, or none yet since arming | command decays to the trained stand |
| `RELAX` | 2 | no valid frame for 5 s | torque off |
| `ESTOP` | 3 | the `ESTOP` flag | torque off, latched |
| `VLAND` | 4 | pack ≤ 9.9 V for 0.5 s | stop travelling, crouch down under control |
| `VSAFE` | 5 | 1.5 s after `VLAND` | torque off, latched until `batt reset` or a power cycle |
| `FALLEN` | 6 | `up_z` < 0.4 for 200 ms | torque off; the run ends and the loop benches |
| `BENCH` | 7 | boot, an `ARM` 1→0 edge, a fall, a home, or the tethered `bench` | the CLI owns the bus; the link commands nothing |

The first four describe the link. The rest are decided by the robot and
appended to the enum, so existing states keep their encoding. The pack
states outrank everything and no command clears them; `FALLEN` outranks the
link states. Details of the pack guard: [firmware-design.md](firmware-design.md)
§5.6 and [wiring.md](wiring.md).

One line of reason per rule:

- **250 ms** is 5 missed frames at 20 Hz: an ordinary WiFi hiccup is
  invisible, a runaway is brief.
- **5 s → `RELAX`**: holding a stand forever on a dead link heats the
  servos and drains the pack for a pose nobody wants; limp is the right
  state for an unattended robot.
- **An armed loop that has heard no frame is `STAND`, not `RELAX`**: a robot
  armed on its feet with no commander yet should stand, not fold.
- **E-stop clears only through a frame with `ENABLE` off**, so a released
  dead-man cannot re-arm straight into motion. Every arm also starts a fresh
  watchdog.
- **Stale and reordered frames are dropped by sequence number and do not pet
  the watchdog**; otherwise a reordered burst could hold the robot `LIVE` on
  a command from the past. A backwards jump of more than 1000 means the
  sender restarted, and the watchdog resyncs to it.
- **`FALLEN` ends the run.** Below the sim's own `fall_up_z` the policy is
  outside its training and holding torque only grinds the servos against the
  floor. There is no auto-clear: re-arming is a fresh `ARM` edge or `run`,
  by someone looking at the robot.
- **The real E-stop is the inline battery switch.** No flag that has to
  arrive over a radio can be a safety guarantee.

Measured against the sim twin with injected loss, walking at 20 Hz for 8 s:
30 % packet loss caused 0 dropouts to `STAND`, 60 % caused 5 and 85 %
caused 16 (the 0.6⁵-per-window prediction); even at 85 % the failure is a
brief stand. Killing the commander decays exactly as specified: `LIVE` →
`STAND` at +250 ms, `STAND` → `RELAX` at +5.0 s. `RELAX` is verified by
torque, not by watching: `tests/test_torque_release.py` asserts that a
released sim servo applies exactly 0 N·m.

## Arming over the link

The loop boots **benched**: the CLI owns the servo bus and nothing the link
sends moves the robot. `ARM` (bit 2) arms it over the radio;
`link/protocol.py:ArmLatch` is the reference (`linkproto::ArmLatch`).

- `ARM` is a **level** on the wire, held in every frame like `ENABLE`, but
  the robot acts on its **edges**: 0→1 is `run`, 1→0 is `bench` (torque off,
  bus back to the CLI).
- **The first frame from a sender never counts.** A robot that reboots under
  a console still holding `ARM=1` stays `BENCH` until that console
  deliberately re-arms; the console sees `BENCH` and says so.
- **There is one latch per robot**, fed every decoded frame in arrival order
  whoever sent it. A sender that never sets `ARM` (`link/commander.py`) makes
  no edge against a robot armed over the tether, but its `ARM=0` frames
  interleaved with an arming console's `ARM=1` frames make 1→0 edges and
  bench the robot. `link/arm_script.py` holds `ARM` itself.
- **Refused without an as-built calibration in NVS**, and **refused unless
  every servo's position-loop gains read back as expected** (registers 21/22,
  read before every arm), the same gates as the typed `run`.
- The control loop only publishes the edge; the housekeeping task applies it
  through the same `cmdMode()` as the tethered `run`, between CLI lines, so a
  wireless arm never lands inside a bench-mode bus command.

## Saying why: the bench diagnostic

While the state is `BENCH`, `seq_echo` carries a diagnostic byte instead of
a sequence number: a benched loop applies no commands, so in exactly that
state the field has nothing to lose. There is no version bump and no length
change, so every console decodes every frame. (The alternatives are worse: a
new length or version is dropped by every existing decoder, which checks
both; overloading `servo_err` or `loop_late_pct` makes an older console show
a false fault.)

| bits | meaning |
|---|---|
| 0 | `RUN`: mode is run (0 = benched) |
| 1 | `CAL_OK`: the as-built calibration was loaded from NVS |
| 2–3 | reserved, sent 0 |
| 4–7 | `ArmResult` |

| `ArmResult` | meaning |
|---|---|
| 0 `NONE` | nothing has asked for a mode change yet |
| 1 `ACCEPTED` | the last request was honoured (arm or disarm) |
| 2 `REFUSED_NO_CAL` | arm refused: no as-built calibration in NVS |
| 3 `DISARMED_FALL` | the fall latch tripped; run over |
| 4 `DISARMED_HOME` | a home request benched the loop; joints read back at their zeros |
| 5 `HOME_NO_CAL` | home refused: no calibration |
| 6 `HOME_LOW_BATT` | home refused: the pack guard holds torque off |
| 7 `HOME_BUS_FAILED` | home failed: the servo bus did not accept it |
| 8 `HOME_PENDING` | home written, joints moving, readback not done |
| 9 `HOME_NOT_REACHED` | home failed: readback found joints off their zeros |
| 10 `REFUSED_GAINS` | arm refused: a servo's position-loop P/D (registers 21/22) is not its expected value, or did not read back ([servo-map.md §4](servo-map.md#4-registers)) |

The layout is **append-only**, like `LinkState`: reserved bits are sent zero
and ignored, the upper 24 bits of `seq_echo` are reserved zero, and a value a
client does not know reads *"reason unknown to this client (newer
firmware)"*, never an older reason. The operator-facing sentences
(`diag_reason` / `linkproto::diagReason`) are frozen in the golden vectors
beside the bytes, because the sentence is what the operator reads.

`bimo_tui` and `bimo_gui` draw the reason whenever the beacon says `BENCH`,
so an uncalibrated robot is diagnosable the moment a console opens;
`link/commander.py` appends it to its status line. `link/link_twin.py
--no-cal` emits it for tests.

## Reset the servos (`HOME`)

`HOME` (bit 4) is the consoles' **RESET SERVOS**: every joint to its
calibrated zero, the standing pose. It is the one request honoured while the
robot is benched, fallen or E-stopped — the states in which every other
channel correctly refuses to move anything, and exactly the states a robot
on the floor is in.

- **A rising edge, and the first frame from a sender never counts**, so a
  client that reboots with the bit set cannot move a robot nobody is
  watching.
- **It benches the loop first**, because the move is a bench-mode bus
  transaction, then homes. **It never arms**: re-arming stays a deliberate
  `ARM` edge.
- **Refused without calibration** (the defaults are 2048 on every joint, not
  a stand) and **refused while the pack guard holds torque off**. Targets are
  clamped to the mechanical envelope, and the move is slow: minimum-jerk,
  the longest joint at no more than 300 steps/s, 2.5–8 s, then each hip
  rolled 5° out and back to take up roll play.
- **It reports the verdict, not the intention**: `HOME_PENDING` while
  moving, then a readback of every joint (±12 ticks) and `DISARMED_HOME` or
  `HOME_NOT_REACHED`. Torque is then released; gear friction holds the stand.
- The console holds the `HOME` level for 8 frames (400 ms), so one dropped
  packet does not lose the request and the level falls again before the next
  press. If no home verdict arrives within 1.5 s, it says the robot's
  firmware may not know the request or the frames never arrived.
  `link_twin.py --deaf-home` plays that robot in `tests/test_gui_e2e.py`.

## Time on the wire

One time base, NTP, on both ends, so a video frame and a telemetry line can
be joined by subtraction.

- **The robot syncs itself.** `firmware/main/timesync.{h,cpp}` starts SNTP as
  soon as the WiFi link holds a DHCP lease. The server comes with the lease
  (DHCP option 42), with `pool.ntp.org` as the fallback; it polls every 30 s
  and skips the RFC 4330 startup delay. The board has no RTC, so time is 0
  until the first reply.
- **Every beacon carries `t_us`**; 0 means not synced. With joints, the stamp
  is the instant they were read (the middle of the bus read); otherwise it is
  the beacon's assembly time. Either way it is within one 20 ms tick of the
  values beside it.
- **Every video frame carries its capture time.** `tools/cam_record.sh`
  burns UTC into the picture to the millisecond and writes a `.pts` sidecar
  of one epoch-ms per frame; `tools/record_attempt.sh` records both cameras
  and whether the host clock was NTP-synced.
- **Every log line carries both clocks.** `link/recorder.cpp` (`bimo_gui`'s
  JSONL in `hw_sessions/`) writes `utc` (host) on every record, `robot_utc`
  (from `t_us`, `null` when 0) and the joints on every beacon, and
  `host_clock` in its header. `link/arm_script.py` prints `utc=` and `robot=`
  on every line. `utc − robot_utc` is link latency plus the two clocks'
  disagreement.

Checking it:

1. Flash; on the tether, `wifi` until CONNECTED, then `ntp`: expect `synced`
   with a server from DHCP. `UNSYNCED` after a minute means the router hands
   out no option 42 and `pool.ntp.org` is unreachable from the robot's
   network.
2. `bimo_gui --host <robot>`: the session JSONL has `robot_utc` on every
   beacon (not `null`) and `"host_clock":"ntp"` in its header.
3. `tools/record_attempt.sh <name> ...`: `<name>_timing.txt` has
   `host_ntp_synced: yes` and `cam0_first_frame_epoch`.
4. Pick a beacon with a state change in `<name>_arm.log`, read its `utc=`,
   find the first sidecar line at or after it: the burned-in time on that
   frame agrees to the millisecond and the picture agrees with the state.

## The training envelope is part of the protocol

Training draws a stand or a walk with |vx| in **[0.3, 1.0] m/s** and
|wz| ≤ 1.0 rad/s. The band (0, 0.3) was never trained: it is a hole in the
distribution, not a slow walk. So `clamp_to_envelope()` snaps it to a stand,
**robot-side**, because the robot is what eats a bad command and not every
future sender is ours. Turn-in-place (`vx = 0, wz ≠ 0`) is trained, so the
snap keeps `wz`. The extra channels are clamped to their training draws too
(|vy| ≤ 0.3, crouch 0.6–1.0, lift ±1, foot offsets ±0.05 m).
`sources.stick_to_speed()` maps stick travel onto [0.3, 1.0] for the same
reason. `V_MIN_WALK`, `V_MAX`, `W_MAX` and the extra-channel limits in
`protocol.py` (and their `linkproto` copies) must track the training config
of the deployed policy.

**Legal is not reliable.** Commanding the edge of the envelope (pivots at
`wz = 1.0`, sprints at 1.0 m/s) measurably falls; walking while turning is
the best-trained regime. Aim at the middle: the goal source cruises at
0.6 m/s, caps yaw at 0.8, arcs toward the goal and pivots only when the goal
is behind it.

## Network

Station mode only: the robot joins the LAN (WPA2 minimum, hostname `bimo`)
with credentials stored in NVS by `wifi <ssid> <psk>` on the tether (bench
mode). Modem power save is off, because its ~100 ms latency spikes are the
wrong trade on a 20 Hz link with a 250 ms stale window. The robot beacons to
the address of the last valid command frame, and to nobody else.

## Consoles

All commanders live laptop-side, so the robot learns one wire format and a
new commander is a class in `link/sources.py` or a front end on
`link/client.h`, not a firmware change.

- **`bimo_tui`** (`link/tui.cpp`; `make -C firmware/host tui`): the terminal
  console, for SSH. Built from the firmware's own `linkproto` sources, so the
  bytes it sends come from the code the robot decodes with. Keys: `a`
  arm/disarm, arrows walk (0.4 m/s) and turn (±0.5 rad/s) while held,
  `space`/`s` stand (also clears an E-stop), `e` E-stop, `h` reset servos,
  `q` quit (disarms first). A terminal has no key-up event, so a motion key
  holds for `--hold-ms` (default 700) after the last auto-repeat; the console
  measures the repeat delay on screen.
- **`bimo_gui`** (`link/gui.cpp`; `brew install glfw`, then
  `make -C firmware/host deps gui`): the windowed console. Arming, the E-stop
  latch and frame building are shared with `bimo_tui` in `link/client.h`, so
  the two cannot disagree about what a keypress puts on the wire. A window
  adds:
  - **a real dead-man**: key release stands the robot on the next frame, and
    so does the window losing focus;
  - **the extended channels** as sliders (the 24 B frame, shown on screen);
  - **strip charts** of `vbat`, `up_z` and each commanded channel against its
    estimate;
  - **a SIM panel** that starts `sim/sil_twin.py` and draws its view in the
    window;
  - **RESET SERVOS** (`h`), amber and apart from the controls that stop
    things, because it is the way out of an emergency, not one;
  - **a SETTINGS panel** with every command-line flag editable live;
    socket-binding fields are applied on a button, not per keystroke.

  Default keys: arrows translate (`vx`, `vy`), PgUp/PgDn rotate, `a` arm,
  `space` stand, `e` E-stop, `h` reset servos, `r` record, `0` reset the
  extended channels, `q` quit. Every binding is rebindable in-app
  (`~/.bimo/keymap.conf`). `--headless` runs the same link loop with no
  window, taking `press forward` / `release forward` / `set crouch 0.8` on
  stdin; the end-to-end tests drive it that way, and it makes any figure
  reproducible from a command line. Sessions record to `hw_sessions/*.jsonl`.
- **`bimo_gui --readonly`**: an observer that cannot touch the robot.
  - **It has no transmit socket.** The socket is closed entering readonly
    and created leaving it, so the promise is enforced by the file
    descriptor, not the UI. Every control refuses out loud, and E-STOP reads
    **"(not yours)"**: an E-stop that seems to work while nothing goes out is
    worse than none.
  - **It draws no command traces**, since the beacon carries what the robot
    measured, never what another console asked for, and records telemetry
    rows only.
  - **TAKE CONTROL** (a button; unbound by default, because a stray key would
    hand the robot between consoles) makes it the commander; **WATCH ONLY**
    closes the socket again. Taking control needs `--host`: the beacons come
    from the relaying console, so the robot's address cannot be guessed.
- **A watcher needs the driver's help.** The robot beacons only to its last
  commander, and the only way to become that is to send a command, which
  steals the beacon and drives the robot. So the driving console relays:
  `--watch HOST[:PORT]` re-sends every accepted beacon byte for byte, and the
  watcher runs the same decode over the same bytes. `--watch` exists on
  `bimo_gui`, `bimo_tui` and `link/commander.py`; a `--readonly` console
  refuses it at startup.

  ```sh
  # driving machine
  bimo_gui --host 192.168.2.90 --watch 192.168.2.30
  # watching machine
  bimo_gui --readonly
  ```

- **Taking control must not drop the robot.** The arm latch tracks a level
  across all senders, so a console that starts commanding an armed robot
  with `ARM` low puts a 1→0 edge on the wire and benches it mid-stride.
  `Intent::adopt()` adopts the level the beacon reports instead:

  | beacon says | adopted | why |
  |---|---|---|
  | any state but `BENCH` | armed | the loop is running; includes `FALLEN` and the pack states, where torque is off but the level is still high |
  | `ESTOP` | armed + E-stopped | the E-stop latch is a level too; dropping it would clear it at handover |
  | `BENCH` | disarmed | adopting "armed" would be a rising edge nobody asked for |
  | no fresh beacon | disarmed | nothing to adopt; benching a robot you cannot see is the right answer |

  Releasing control sends no parting disarm: someone else's frames hold the
  level high, and a disarm would bench the robot out from under them. With
  nobody driving, the robot's own watchdog is the failsafe.
- **`link/commander.py`**: streams a `link/sources.py` source at 20 Hz and
  never sets `ARM`. `--source script --script <name>` runs a timed sequence
  (`stand`, `walk`, `dash_stop`, `line_1m`, `turn`, `pivot`, `square`,
  `march`, `stride1`, `sidestep1`, `crouch1`, `rom_feet`); `--source
  gamepad` reads sticks through pygame with a **held** dead-man (right
  shoulder; letting go must stop the robot) and B/circle as E-stop.
  `--source goal --goal X Y` is a goal-seeker that needs a world (x, y, yaw)
  feed as 12-byte datagrams on UDP 4212 (`sources.PoseFeed`); the robot has
  no odometry to provide one and neither twin publishes one.
- **`link/arm_script.py`**: arms over the radio, runs a `sources.py` script
  with `ARM` held, then disarms. It E-stops, disarms and exits 2 on
  `FALLEN`, a servo fault or `up_z` < 0.5 for 0.3 s.

## Simulated robots behind the link

Both answer on the robot's ports (4210/4211 by default) with the same
`link/protocol.py:Supervisor` composition the firmware runs, so every
console can be exercised on a laptop.

- **`link/link_twin.py`**: plant-free. Protocol, arm latch, watchdog, mode
  and 10 Hz telemetry, nothing else: `vx_est`/`wz_est` echo the applied
  command, `vbat` is 11.4 V, `up_z` 1.0, and requested joint angles are a
  slow deterministic wobble. Flags: `--boot-armed`, `--no-cal` (refuses to
  arm, like an uncalibrated robot), `--deaf-home` (ignores `HOME`, like
  firmware that does not know it). It is what the console end-to-end tests
  drive.
- **`sim/sil_twin.py`**: the firmware's own control code (`libctrl_sil`:
  observation, gait clock, network, actuation) driving the CPU plant behind
  the real link, so a console drives the same control law the robot runs.
  `--run-name` picks the policy (it needs `sim/runs/<run>/config.json` and
  `sim/sil/weights/<run>.silw.json`), `--hang` welds the torso at stand
  height (the test-stand analogue), `--stream-port` serves the rendered view
  that `bimo_gui`'s SIM panel draws, `--record` writes an mp4, `--obs-csv`
  writes the fed observations in `obsdump` format, and `--plant`,
  `--act-delay-ticks`, `--act-lag-hz`, `--play-deg` load it like the
  hardware. `--viewer` is [mirror mode](mirror-mode.md).

```sh
.venv/bin/python link/link_twin.py                       # plant-free robot
.venv/bin/python sim/sil_twin.py --hang                  # firmware stack + plant
firmware/host/build/bimo_tui --host 127.0.0.1
```

## Checking the link

```sh
.venv/bin/python link/verify_udp.py --host <robot-ip> [--seconds 10]
```

Sends **disabled** stand frames (`ENABLE` off, zero command, no `ARM`) at
20 Hz and listens for the beacon: proves both directions and measures loss
without moving anything. A benched robot answers `BENCH`. It passes if at
least one valid frame arrives and more than half the expected 10 Hz beacons
are heard. Point it at a benched robot only: its `ARM`-off frames disarm a
robot another console has armed. On the tether, `wifi` shows association,
IP, RSSI, frame counters (`cmd rx`, `bad`, `tlm tx`) and the current
commander; `ntp` shows the clock.

## How the firmware implements it

The C side is a transcription of the Python reference, checked against its
bytes:

1. `crc16Ccitt`, `encode/decodeCommand`, `encode/decodeTelemetry`: ports,
   checked against `0x29B1` and the golden vectors.
2. `Watchdog`: a port with no I/O; it takes `now_ms`.
3. `clampToEnvelope` / `clampExtToEnvelope`: the limits must match
   `protocol.py` and the deployed policy's training config.
4. The loop: receive → decode → `ArmLatch` / `HomeLatch` →
   `Watchdog.accept()` → `state()` → `commandExt()` → policy → servo bus at
   50 Hz. `link/protocol.py:Supervisor` is that composition in Python, used
   by both twins. On the robot: `main/wifi_link.cpp` (UDP),
   `main/link_task.cpp` (UART), `main/ctrl_task.cpp` (the loop).
5. `RELAX`, `ESTOP`, `FALLEN`, `VSAFE` and `BENCH` → torque-enable
   (register 40) off, one broadcast frame to every servo.

Not modelled anywhere: WiFi's real jitter distribution, and ESP32 socket
behaviour under load.
