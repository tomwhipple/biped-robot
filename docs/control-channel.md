# Wireless command channel

*Status 2026-07-15. Untethered by design. No parts change: the ESP32 already
on the order sheet does this. Specified in `link/protocol.py`, exercised
against MuJoCo over real UDP by `sim/udp_agent.py`, tested in `tests/`.*

The USB-C tether stays for flashing and debug ([wiring.md](wiring.md)), but
nothing about running the robot needs it.

## The idea: the radio carries intent, not the control loop

This is the whole design, and everything else follows from it.

The 50 Hz policy loop runs **on the ESP32**, next to the 1 Mbaud servo bus —
already the plan of record, because an 8-servo command+readback cycle at
115200 baud eats most of a 20 ms tick (DESIGN.md's latency work). So the
radio does not carry joint angles at 50 Hz. It carries the **2-vector
`(vx, yaw_rate)`** that `walker_env.set_command()` already takes, at 20 Hz.

Consequences worth being explicit about:

- **A lost packet costs staleness, never a bad joint angle.** The worst a
  dropped frame can do is leave the robot tracking the previous intent for
  another 50 ms. Compare a radio in the control loop, where a dropout is a
  gap in the servo command stream.
- **The failsafe is a trained behavior, not an emergency pose.** When the
  link goes stale the command decays to `(0, 0)` — and `walker_env` draws a
  stand command 30 % of the time during training (`cmd_stand_prob`). The
  robot's response to losing its radio is the single most-practiced thing it
  knows. Nothing bespoke to write, tune, or trust.
- **Bandwidth is a non-issue.** 14 bytes at 20 Hz is 2.2 kbit/s. The link
  budget stops being an engineering problem.

## Wire format

Little-endian (ESP32 and host are both LE, so the firmware can memcpy).
Identical framing over UDP and over UART0 for tethered debug — one encoder,
both paths.

**Command** — laptop → robot, 14 B, UDP port 4210, 20 Hz:

| off | size | field | notes |
|---|---|---|---|
| 0 | 2 | magic | `"BM"` |
| 2 | 1 | version | 1 |
| 3 | 1 | flags | bit0 `ENABLE`, bit1 `ESTOP`, bit2 `ARM`, bit3 `POSE`, bit4 `HOME` |
| 4 | 4 | seq | u32, monotonic |
| 8 | 2 | vx | i16, mm/s, body-frame forward |
| 10 | 2 | wz | i16, mrad/s, yaw rate |
| 12 | 2 | crc16 | CCITT-FALSE over bytes 0–11 |

**Telemetry** — robot → laptop, 28 B, UDP port 4211, 10 Hz: magic `"BT"`,
version, link state, echoed seq, bus millivolts, torso up-z, estimated vx/wz,
a per-servo fault bitmask, the percentage of control ticks that overran
20 ms, and — since 2026-09-02 — the robot's wall clock, `t_us`, as
microseconds since the Unix epoch. The OLED shows bus voltage too, but
telemetry is what a laptop can log.
The echoed-seq field does double duty while the state is `BENCH` — see
[the bench diagnostic](#saying-why-over-the-radio-2026-08-30). The joints
extension (48 B, [mirror-mode.md](mirror-mode.md)) and the timestamp are
described in [Time on the wire](#time-on-the-wire-2026-09-02).

| off | size | field | notes |
|---|---|---|---|
| 0 | 2 | magic | `"BT"` |
| 2 | 1 | version | 1 |
| 3 | 1 | state | `LinkState`, declaration order |
| 4 | 4 | seq_echo | u32; the bench diagnostic byte while `BENCH` |
| 8 | 2 | vbat | u16, mV |
| 10 | 2 | up_z | i16, milli |
| 12 | 2 | vx_est | i16, mm/s |
| 14 | 2 | wz_est | i16, mrad/s |
| 16 | 1 | servo_err | bitmask, obs_spec joint order |
| 17 | 1 | loop_late_pct | u8 |
| 18 | 8 | **t_us** | u64, µs since the epoch, UTC, robot clock; **0 = not synced** |
| 26 | 20 | joints | 10 × i16 milli-rad, **48 B frame only** (`FLAG_POSE`) |
| last | 2 | crc16 | CCITT-FALSE over everything before it |

Fixed-point milli-units rather than float32 so the frame is byte-identical
across the Python sender and a C firmware. ±32.7 m/s of headroom on a plant
that does 1.0.

**Why a CRC when UDP already checksums?** Not bit rot. It (a) rejects stray
traffic that hits the port — in AP mode the robot's network is open — and (b)
lets the same frame ride UART0, where there is no transport checksum at all.
`crc16_ccitt(b"123456789") == 0x29B1`, the standard check value, so a C port
can be diffed against a number rather than against "whatever Python said".

## Link states (the failsafe)

![Link-state filmstrip](control-channel-failsafe.png)

*Real UDP, real policy, commander killed mid-stride. Stripe colour is the
watchdog's own per-tick state: amber `STAND` (powered up, no commander yet) →
green `LIVE` (walking under command) → amber `STAND` (radio dead, decayed to
the trained zero command) → red `RELAX` (torque released). Regenerate with
`.venv/bin/python sim/render_link_demo.py --run-name cmd_11v1`.*

`link/protocol.py:Watchdog` is the reference implementation. It takes its
clock as an argument, so link loss is tested exactly and instantly rather
than slept through.

| state | entered when | robot does |
|---|---|---|
| `LIVE` | valid frame < 250 ms ago | track the command |
| `STAND` | no valid frame for 250 ms | command `(0,0)` — the trained stand |
| `RELAX` | no valid frame for 5 s | **torque off**; servos release |
| `ESTOP` | operator flag; latching | torque off immediately |
| `VLAND` | pack ≤ 9.9 V for 0.5 s | stop travelling, crouch down under control |
| `VSAFE` | 1.5 s after `VLAND` | **torque off**, latched until a pack swap |
| `FALLEN` | torso `up_z` < 0.4 for 200 ms | **torque off**, latched; clears when righted |
| `BENCH` | boot, or an `ARM` 1→0 edge, or the tethered `bench` | **torque off**; CLI owns the bus; the link commands nothing |

The first four come from the watchdog and describe the *link*. The rest are
decided by the ROBOT and appended to the enum, so existing states keep their
wire encoding. `VLAND`/`VSAFE` come from `battguard::Guard`, describe the
*pack*, outrank everything, and no command clears them (see
[wiring.md](wiring.md#battery-protection)). `FALLEN` (2026-08-26) is the
control loop reading its own IMU: below `up_z` 0.4 — the sim's own
`fall_up_z` termination threshold — the policy is outside anything training
produced, and holding torque only grinds the servos against the floor, so the
robot goes limp and says why. Unlike the pack states it clears without a
reboot, but deliberately: upright (`up_z` > 0.7) for 2 s **and** the
operator's frames saying `ENABLE` off — so a robot picked up mid-fumble
cannot spring back to life in someone's hands, and a dead link leaves it
safely down. Tethered equivalent: `bench` then `run`.

- **250 ms** is 5 missed packets at 20 Hz — long enough that one ordinary
  WiFi hiccup is invisible, short enough that a runaway is brief.
- **5 s → RELAX** exists because holding a stand forever on a dead link
  cooks eight servos and drains the pack to hold a pose nobody wants. Limp
  is the right state for an unattended 28 cm robot; it falls better than it
  overheats. (`walker_env.set_torque_enabled()` — the sim analogue of the
  STS3215's torque-enable register, i.e. the "Release" in wiring.md's
  bring-up checklist.)
- **Before the first packet ever arrives** the state is `STAND`, not
  `RELAX`: a robot powered up on its feet with no commander yet should stand,
  not flop.
- **E-stop latches** and clears only via a frame with `ENABLE` *off*, so a
  released dead-man cannot re-arm straight back into motion.
- **The real E-stop is the inline battery switch.** No flag that has to
  arrive over a radio can be a safety guarantee; the watchdog is the property
  that holds when the radio is gone, and the switch is what holds when
  everything is gone.

## Arming over the link (2026-08-27)

The control loop boots **benched**: the CLI owns the servo bus and nothing
the link sends moves the robot. Until now only the tethered `run` armed it.
The `ARM` flag (bit 2) does the same over the radio, and everything about it
is in `link/protocol.py:ArmLatch` (C++: `linkproto::ArmLatch`):

- `ARM` is a **level** on the wire — "the operator wants the loop armed",
  held in every frame like `ENABLE` — but the robot acts on its **edges**
  only. 0→1 is `run`; 1→0 is `bench` (torque off, bus back to the CLI).
- **The first frame from a sender never counts.** A robot that reboots under
  a console still holding `ARM=1` stays `BENCH` until that console
  deliberately re-arms; the console sees `BENCH` in telemetry and says so.
- **A sender that never sets the bit never makes an edge**, so the
  script/gamepad commanders can still drive a robot armed over the tether
  without benching it on their first frame.
- Same refusal as the typed `run`: no as-built calibration in NVS, no run.
  ~~The refusal is printed on the tether; the link only sees the robot stay
  `BENCH`.~~ The refusal now rides the radio — next section.

Firmware-side the latch lives in the control loop, which now drains the
mailbox in bench mode too; the edge is published to the housekeeping task,
which calls the same `cmdMode()` as the CLI. That keeps the mode's single
writer, and — because housekeeping only looks between CLI lines — a
wireless arm can never land in the middle of a bench-mode bus command.

## Saying WHY, over the radio (2026-08-30)

**The incident.** The robot ignored six seconds of `ARM` frames and sat in
`BENCH`. Every part of the mechanism above worked; the refusal happened
exactly where it should, in `cmdMode()` — and its explanation went to the
UART sink, which nobody was plugged into. Diagnosing "the robot is not
listening" required fetching a cable. That is the failure this closes: a
silent `BENCH` beacon is not a diagnostic, it is a shrug.

### The wire decision, and the two options it beat

A **protocol version bump appending bytes** to telemetry was the obvious
move and is the wrong one. Both decoders check the length *and* the version
(`decode_telemetry`, `decodeTelemetry`), so every commander built before the
bump — including the console already running on the operator's screen —
would reject every frame. The change would blind the client at precisely the
moment the diagnostic matters. Rejected.

**Overloading `servo_err` or `loop_late_pct`** keeps compatibility but lies:
an old console would draw `servo FAULT 00100000`, or `137% late ticks`, and
send someone hunting a servo that is fine. A diagnostic that produces a
false alarm on old clients is worse than silence. Rejected.

**The diagnostic rides `seq_echo`, and only while the state is `BENCH`.**
`seq_echo` is defined as *the last command sequence the robot applied*, and
a benched loop applies nothing — so in exactly that state, and no other, the
field carries no information to destroy. No version bump, no length change:
**every existing commander decodes every frame unmodified**, and what it
shows is a small sequence number, which is what a fresh boot looks like
anyway. The one client-side cost is that `bimo_tui` must not compute a "lag"
from it while benched, so it prints the raw byte instead.

The byte is `linkproto::packDiag` / `protocol.py:pack_diag`, and its layout
is **append-only** like the `LinkState` enum — reserved bits are sent zero
and a reader ignores what it does not know:

| bits | meaning |
|---|---|
| 0 | `RUN` — mode is `kRun` (0 = benched) |
| 1 | `CAL_OK` — as-built calibration was loaded from NVS |
| 2–3 | reserved, sent zero |
| 4–7 | `ArmResult`: 0 none yet, 1 accepted, 2 **refused, no calibration** |

A future refusal reason takes value 3. A client that meets a value it does
not know says *"reason unknown to this client (newer firmware)"* rather than
reporting an older reason — guessing "no calibration?" at an unrelated fault
is how an operator ends up re-running `cal` for nothing. The upper 24 bits of
`seq_echo` are reserved zero, so the diagnostic can grow past a byte without
another wire decision.

The reason *sentences* are frozen in the golden vectors alongside the bytes.
Two copies of a message drift the moment one side is reworded, and the
sentence is the part the operator actually reads.

**Firmware-side** `cmdMode()` — the one place that decides an arm request —
publishes its verdict to `robot::g_arm_result` before printing it, and
`wifi_link.cpp` folds it into the `BENCH` beacon with `g_cal_from_nvs`. No
new task: `cmdMode()` only ever runs on the housekeeping loop (a typed
`run`/`bench` through `cli::execute`, a wireless edge through
`cli::linkMode`), so the atomic keeps its single writer, exactly like the
mode itself.

**Client-side**, `bimo_tui` draws the reason whenever the beacon says
`BENCH` — not only after a failed arm. An uncalibrated robot therefore reads
*"BENCH: no arm requested yet — and there is NO calibration in NVS, so
arming will be REFUSED"* the moment the console opens, instead of after a
silent 1.5 s and a press of `[a]`. `link/commander.py` appends the same
sentence to its status line and repeats it on exit. `link/link_twin.py`
emits the byte faithfully, `--no-cal` included, which is what
`tests/test_tui_e2e.py` asserts the console displays.

Reordered and duplicate frames are dropped by sequence number, and — subtly —
**a stale frame must not pet the watchdog**. If it did, a reordered burst
could hold the robot `LIVE` on a command from the past. A backwards seq jump
larger than 1000 is read as "the commander restarted", not "ancient packet",
so a restarted sender at seq 0 resyncs instead of being ignored forever.

## Time on the wire (2026-09-02)

**The problem.** A hardware take is video plus a telemetry log, and until
today nothing joined them better than a second. The robot had no clock at
all: the Waveshare board has no RTC chip, no 32 kHz crystal and no backup
cell (the vendor schematic, read end to end — GPIO32/33, the only pins that
could take a slow crystal, are the I2C bus), and the firmware never asked
for the time. Telemetry carried none. Host side, the recorder's `t` column
was seconds since the recording started, the video's only anchor was the
mp4's whole-second mtime, and "where in the video is the beacon that says
the robot fell" was a guess.

**The design.** One time base, NTP, on both ends:

- **The robot syncs itself.** `firmware/main/timesync.{h,cpp}` runs SNTP
  from the moment the WiFi link holds a DHCP lease. The server comes with
  the lease (DHCP option 42 — the LAN router hands it out), with
  `pool.ntp.org` as the static fallback; polls every 30 s, so the crystal's
  drift between steps stays in the low milliseconds; the RFC 4330 startup
  delay (a random 1–5 min before the first request) is off. The tethered
  `ntp` command is the only diagnostic, since logging is silenced after
  boot — type it after a flash and before trusting a single stamp.
- **Every beacon carries `t_us`.** A u64 at offset 18, microseconds since
  the Unix epoch on the robot's clock. **0 means "not synced"** — the robot
  has not heard from NTP yet, or the frame came from firmware older than
  this section. Nothing ever sends the boot-relative time dressed up as an
  epoch. When the frame carries joints (mirror mode) the stamp is the
  instant they were read off the servo bus (the midpoint of the ten reads),
  not the instant the beacon was sent; otherwise it is the beacon's assembly
  time. Either way it is within one 20 ms tick of the values beside it.
- **Every video frame carries its capture time.** `tools/cam_record.sh`
  records a webcam with the UTC time burned into the picture to the
  millisecond and a `.pts` sidecar of one epoch-ms per frame, from the
  kernel's V4L2 capture timestamps converted to `CLOCK_REALTIME` — which
  chrony disciplines to NTP on mira (0.1 ms, measured). Verified against
  `date` on `/dev/video0`. `tools/record_attempt.sh` uses it for both
  cameras and records whether the host clock was NTP-synced at the time.
- **Every log line carries both clocks.** `link/recorder.cpp` (bimo_gui's
  JSONL) writes `utc` (host, CLOCK_REALTIME) on every record and
  `robot_utc` (from `t_us`, `null` when 0) plus the joints `q` on every
  beacon, and says in its header whether the host clock was disciplined
  (`host_clock`, from the kernel's own flag). `link/arm_script.py` prints
  `utc=` and `robot=` on every line. `utc − robot_utc` is link latency plus
  the two clocks' disagreement, and is the number to look at when a
  filmstrip and a joint trace disagree.

### The wire decision, and why this one is volunteered

The mirror-mode extension established the rule for telemetry: the robot is
the sender, every client rejects an unknown length, so **a new length is
requested, never volunteered** ([mirror-mode.md](mirror-mode.md)). The
timestamp breaks that rule on purpose, and both halves of the reasoning are
worth writing down.

*Why not request it with a flag bit.* A timestamp is not a payload a client
opts into; it is metadata about the frame, and its whole value is that the
log of *any* frame from *any* commander can be laid against a video. Making
it depend on each commander remembering to set a bit is exactly the "thing
nobody writes down at the time" that `link/recorder.cpp` exists to prevent.
A passive listener on port 4211 would get stamps only if someone else had
asked.

*Why it is safe anyway.* Both decoders — `decode_telemetry` and
`decodeTelemetry` — now accept **four** lengths: 28 and 48 with the stamp,
and the legacy 20 and 40 without it (decoded with `t_us = 0`, never
emitted). So a console built from this tree keeps working against a robot
flashed before the change, which is the case that matters on the bench
(the robot as flashed on 2026-09-02, `2f38b0c`, beacons 20/40 B). The only
pairing that goes blind is an *old* console against *new* firmware, and
every console lives in this tree and is rebuilt with it. The version byte
stays 1: length selects the layout, as it has for every extension so far.

### Checking it

1. Flash; on the tether, `wifi` until CONNECTED, then `ntp` — expect
   `synced` with a server address from DHCP and a step in the low ms.
   `UNSYNCED` after a minute means the router is not handing out option 42
   and pool.ntp.org is unreachable from the robot's network.
2. `bimo_gui --host <robot>`: the JSONL in `hw_sessions/` has `robot_utc`
   on every beacon (not `null`) and `"host_clock":"ntp"` in its header.
3. `tools/record_attempt.sh <name> ...`: `<name>_timing.txt` has
   `host_ntp_synced: yes` and `cam0_first_frame_epoch`, and frame *i* of
   `<name>_cam0.mp4` shows the time on line *i+2* of `<name>_cam0.pts`.
4. Cross-check: pick a beacon with a state change in `<name>_arm.log`,
   read its `utc=`, find the first sidecar line ≥ that value; the burned-in
   time on that frame agrees to the millisecond and the picture agrees with
   the state.

Firmware-side the module is 10 kB of flash (esp_netif_sntp), and the
beacon grows by 8 B at 10 Hz — 0.6 kbit/s, still a non-issue.

## Reset the servos, from the states that refuse to move (2026-09-02)

`HOME` (bit 4) is the console's **RESET SERVOS** button: *every joint to its
calibrated zero — the standing pose — now.* It is the only bit on this wire
that moves the robot without arming it, and it exists because of a gap the
other four leave open.

Consider the state an operator is actually in when they need it. The robot
fell: the fall latch tripped, the run is over, torque is off, and the robot
is a heap on the floor. Or they hit E-stop: same picture, on purpose. In both
states every channel here correctly refuses — `ENABLE` is gated on an armed
loop, `ARM` is refused after a fall until a deliberate fresh edge, and the
E-stop latch outranks the lot. All of that is right, and none of it helps:
the one thing wanted is the robot back on its feet, and until 2026-09-02 the
only way to get it was to walk over with a USB cable and type `pose`.

The design, and what each piece is buying:

- **An edge, not a level** (`linkproto::HomeLatch`, mirrored in
  `protocol.py`). A level would re-issue the move 20 times a second. And, as
  with `ARM`, *the first frame from a sender never counts*: a client that
  reboots with the bit set makes no edge, so it cannot re-pose ten joints on
  a robot nobody is watching. That rule matters more here than it does for
  arming — this request moves the robot from a standstill.
- **It benches the loop first.** The move is a servo-bus transaction, and on
  this firmware the bench half (the CLI) owns the bus while the loop is
  benched. So `ctrl_task` only *publishes* the edge (`g_link_home_edges`,
  exactly like an arm edge) and the housekeeping task benches, waits for the
  handover, and runs `cmdHome`. One writer for the mode, no bus command
  landing mid-tick, and a request that is honoured identically whether it
  arrived benched, fallen or E-stopped.
- **It does not arm.** Re-arming stays what it was: a deliberate ARM edge.
  The BENCH beacon says why the robot is benched — a new `ArmResult`,
  `kDisarmedHome`, reading *"disarmed — servos reset to the standing pose;
  torque is HOLDING it, re-arm to walk"*.
- **It turns torque back ON, deliberately.** Everything else in the CLI
  refuses to write a goal while torque is off — an STS servo auto-enables
  torque on a goal write, which is what broke both feet on 2026-08-02, and
  `torqueGate` exists so that can never happen *silently*. This command *is*
  the motion, asked for by name, so it enables torque itself and says so. The
  safety is bought elsewhere instead: **no calibration, no home** (the
  default zeros are 2048 everywhere — the raw-middle pose that broke the
  feet — so the gate is the same one `run` uses), targets clamped to the
  measured mechanical envelope, and a slew of 300 steps/s (~0.45 rad/s), half
  the already-gentle bench default, because the robot may be lying in a pose
  the move has to walk out of and a slow limb is one a hand can catch.
- **Torque stays on at the end,** holding the stand. A robot that homed and
  then went limp has only fallen over more tidily.
- **The pack guard still outranks it.** A latched under-voltage guard is the
  robot's own decision, not the operator's, and a flat pack is exactly when
  powering ten servos to hold a stand is the wrong answer. Home is refused
  there, and says so.

### Every outcome comes back over the radio (2026-09-02, same day)

The button shipped and did nothing on the first press, for the dullest
possible reason: the console had been rebuilt and the robot had not been
re-flashed. Old firmware ignores bit 4 — it is a bit it has never heard of —
and the console's request rides in alongside an ordinary disarm, so the robot
benches, reports *"disarmed on request"*, and every field on screen reads
perfectly plausibly. From the operator's chair that is indistinguishable from
a dead button, a dead radio or a seized servo.

Two changes, both the same lesson this channel already learned once for
arming:

1. **The robot reports the verdict, not the intention.** `linkHome` used to
   publish "reset done" *before* attempting it. It now publishes what
   actually happened: `kDisarmedHome`, or `kHomeNoCal` / `kHomeLowBatt` /
   `kHomeBusFailed`. A refusal that only ever reached the UART is a refusal
   the wireless operator never sees.
2. **The console notices silence.** All four verdicts are recognisable *as*
   home verdicts (`isHomeResult`). If none arrives within `kArmSyncMs` of the
   request — the same grace the arm-sync rule uses — the console says: *"no
   answer to the reset: this robot's firmware may predate the request
   (re-flash it), or the frames never arrived."*

`link_twin.py --deaf-home` is that old robot, and
`tests/test_gui_e2e.py` drives the real console against it, so the failure
that shipped is now a test.

Console-side the request is a `HOME` *level* held for 8 frames (400 ms at
20 Hz): the robot acts on the first flagged frame it receives and ignores the
rest, so the hold costs nothing and buys survival of a dropped packet — and
the level must fall again, or the next press would make no edge at all.

## The training envelope is part of the protocol

`walker_env` draws commands from a stand `(0,0)` or a walk in
**[0.3, 1.0] m/s**. The band **(0, 0.3) was never trained** — it is a hole in
the distribution, not a slow walk. A gamepad stick nudged to 20 % asking for
0.2 m/s is an out-of-distribution query whose behavior is undefined.

So `clamp_to_envelope()` snaps that band to a stand, and it runs **robot-side**
— the robot eats the bad command, and we do not control every future sender.
Turn-in-place (`vx=0, wz≠0`) *is* trained, so the snap preserves yaw.
`sources.stick_to_speed()` maps stick travel onto `[0.3, 1.0]` for the same
reason. If `cmd_v_range` changes in training, `V_MIN_WALK`/`V_MAX` in
`protocol.py` must change with it.

## Topology

Station mode (robot joins your WiFi) is the default; the laptop keeps its
internet and can drive the robot from anywhere on the network. AP mode (robot
hosts its own SSID, `192.168.4.1`) is the fallback for testing away from a
router — the board's stock web UI already uses it, so it is known-good. The
ESP32-WROOM-32 supports STA, AP, and both at once; it is a config choice, not
a hardware fork.

## Commanders

All of them live laptop-side, so the robot learns exactly one wire format and
another commander is a class in `link/sources.py`, not a firmware change.

- **`bimo_tui`** (`link/tui.cpp`) — the operator's console, and the only
  commander that arms. Single keystrokes: `a` arm/disarm, arrows walk
  (`up`/`down`, 0.4 m/s) and turn (`left`/`right`, ±0.5 rad/s) **while
  held**, `space` stand, `e` E-stop, `h` reset servos (works E-stopped or
  fallen — see [above](#reset-the-servos-from-the-states-that-refuse-to-move-2026-09-02)),
  `q` quit (disarms first). Shows every
  telemetry field live. Built from the firmware's own `linkproto` sources
  (`make -C firmware/host tui`), so the bytes it sends come from the code
  the robot decodes with. A terminal has no key-up event, only auto-repeat:
  a motion key extends a hold that expires `--hold-ms` (default 700) after
  the last repeat, and the console measures the terminal's repeat delay on
  screen so that number is set from evidence. Exercised end to end by
  `tests/test_tui_e2e.py` against `link/link_twin.py`, the plant-free twin.
- **`bimo_gui`** (`link/gui.cpp`) — the windowed console (Dear ImGui + GLFW;
  `brew install glfw && make -C firmware/host deps gui`). Everything about
  arming, the E-stop latch and the frame that goes out at 20 Hz is *shared*
  with `bimo_tui` in `link/client.h`, so the two consoles cannot disagree
  about what a keypress puts on the wire. What a window buys, and a terminal
  cannot:
  - **A real dead-man.** True key-release, so `--hold-ms` is gone: letting go
    stands the robot on the very next frame, and so does clicking away from
    the window. That last one is the only new failure mode a window
    introduces, and it is handled explicitly.
  - **The extended channels.** `crouch` / `lift` / `foot_dx` / `foot_dz` are
    sliders — the only commander that reaches them. Off their defaults the
    frame becomes 24 B, which is drawn on screen, because firmware older than
    2026-08-31 drops a long frame by length.
  - **Telemetry as a shape.** 30-second strip charts of `vbat`, `up_z` and
    each commanded channel against its estimate.
  - **The sim, in the window.** The SIM panel starts `sim/sil_twin.py` and
    draws its MJPEG view, so a policy can be flown before it is flown.
  - **A RESET SERVOS button** (`h`), on its own row below the row that stops
    things, amber rather than red because it is not an emergency control — it
    is the way out of one. Needs no arm and reaches the robot through a
    latched E-stop or a tripped fall latch.

  Keys default to arrows *translating* on the surface (`up`/`down` = `vx`,
  `left`/`right` = `vy` strafe) and `PgUp`/`PgDn` *rotating*, differing from
  the console because a window can hold two at once. Every action is also a
  button labelled with its binding, and every binding is rebindable in-app
  (`~/.bimo/keymap.conf`). `--headless` runs the identical link loop with no
  window, taking `press forward` / `release forward` / `set crouch 0.8` on
  stdin; that is what `tests/test_gui_e2e.py` drives, and what makes the
  client testable in CI. Sessions record to `hw_sessions/*.jsonl`.
- **`script`** — time-scripted sequences (`walk`, `dash_stop`, `line_1m`,
  `turn`, `pivot`, `square`). The on-hardware twin of `sim/eval_commands.py`,
  so a real run and a sim eval are the same shape of test. Never sets `ARM`:
  needs a robot armed by the console or the tether.
- **`gamepad`** — sticks via pygame, **held** dead-man (letting go must stop
  the robot; a toggle fails that test), B/circle for E-stop.
- **`goal`** — proportional goal-seeker; the seam the goal-conditioning work
  plugs into. **Sim-only today**: it needs world `(x, y, yaw)`, which MuJoCo
  hands over for free but the robot cannot produce — the BNO085 gives
  attitude, and torso linear velocity/height have no direct sensor at all.
  `sources.PoseFeed` isolates that gap to one swappable object; closing it
  (encoder dead-reckoning, or the GoPro) is its own piece of work.

**Aim at the middle of the trained distribution, not its edge.** The obvious
goal-seeker — high yaw gain, turn-to-face, then sprint at `V_MAX` — measurably
does not work. It asks for pivots at `wz=1.0`, the exact upper bound of
`cmd_w_range` and a regime `eval_commands` only ever validates at ±0.5, and
`cmd_11v1` falls over repeatedly instead of arriving. Every value it commands
is legal; the policy is simply worst there. Backing off to a mid-band 0.6 m/s
cruise, yaw capped at 0.8, and arcing toward the goal (walking-while-turning
is the best-trained regime there is — 60 % of `walker_env`'s moving commands
pair a `v` with a `w`) reaches a goal 2.5 m away, closest approach 0.18 m.
Pivot in place only when the goal is genuinely behind you. This generalizes
to any future commander: legal ≠ reliable, and the envelope has a *shape*,
not just bounds.

## Verification

The robot-side protocol half is ordinary Python, so it runs against MuJoCo
over a real UDP socket. Every byte, timeout, and stale-sequence rejection is
exercised before a part is ordered:

```
.venv/bin/python sim/udp_agent.py --run-name cmd_11v1 --render &
.venv/bin/python link/commander.py --host 127.0.0.1 --source gamepad
```

`--drop` and `--delay-ms` inject WiFi loss and latency. Measured, commanding
`walk` at 20 Hz for 8 s (`cmd_11v1`):

| packet loss | dropouts to `STAND` |
|---|---|
| 0 % | 0 |
| 30 % | 0 |
| 60 % | 5 |
| 85 % | 16 |

30 % loss is invisible; real WiFi is typically under 5 %. At 60 % the count
matches the `0.6^5`-per-5-packet-window prediction, and even at 85 % the
failure mode is benign — brief stands, then resume.

Killing the commander mid-stride decays exactly as specified:
`live -> stand` at +250 ms, `stand -> relax` at +5.0 s.

**A note on watching RELAX:** a released robot in the straight-legged
standing pose is a column at a symmetric equilibrium — on a flat plane with
no perturbation it keeps standing, torque or no torque. Torque release is
therefore *not* visible as a slump in a clean sim render. Verify it by torque
(`tests/test_torque_release.py` asserts exactly 0.000 N·m), not by watching.

## Porting to firmware

*Ported 2026-08-24: `firmware/main/wifi_link.cpp` carries this protocol over
station-mode WiFi, verified two-way against mira with `link/verify_udp.py`.*

The C side is a transcription of a debugged module, not a design job.

1. `crc16_ccitt`, `encode/decode_command` — direct ports. Check against
   `0x29B1`, then against `tests/test_protocol.py`'s vectors.
2. `Watchdog` — direct port. It has no I/O and takes `now_ms`, so feed it
   `millis()`. The bit-flip and reordering tests transfer as-is.
3. `clamp_to_envelope` — must match `V_MIN_WALK`/`V_MAX`/`W_MAX` in the
   training config the deployed policy came from.
4. The loop: `recv` non-blocking → decode → `ArmLatch` → `accept()` →
   `state()` → `command()` → policy → servo bus at 50 Hz.
   `link/protocol.py:Supervisor` is that composition in Python, and
   `sim/udp_agent.py:run()` / `link/link_twin.py` are the loop, in order.
5. `RELAX`/`ESTOP`/`BENCH` → STS3215 torque-enable (register 40) off on
   IDs 1–8.

Not yet modeled anywhere: WiFi's real jitter distribution, and ESP32 socket
behavior under load.
