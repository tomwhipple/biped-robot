# Flight recorder

*What the robot was told, what it wrote, and what it measured — every control
tick of every hardware run, in one file on mira, so the robot can be laid
against the simulation and the discrepancy is a subtraction.*

Firmware: `firmware/main/flight_rec.{h,cpp}`, wire format
`firmware/components/flightrec/`. Host: `tools/flightrec.py` (codec),
`tools/flight_collector.py` (UDP service), `tools/flight_session.sh`
(start/stop), `tools/flight_reader.py` (summary, CSV, video/serial alignment,
plot). The session files stay on mira; nothing here uploads anywhere. The
collector can re-send the live stream to a console on the LAN
([Forwarding](#forwarding)).

## Running it

On mira, around a session:

```bash
tools/flight_session.sh start walk2     # collector on UDP 4220, background
# ... the session ...
tools/flight_session.sh stop            # closes the file, prints its summary
```

On the robot, once (over the tether; saved in NVS, so every later boot
records without being asked):

```
rec 192.168.2.5          # mira's LAN address; port 4220 by default
rec                      # status: on/off, WiFi, NTP, counts, drops, costs
rec off | rec on         # stop / resume (kept across reboots)
rec idle 10              # idle sample rate while benched and not streaming (0 = none)
```

Each session is `hw_sessions/<local date>/flight_<UTC start>[_label].frec`
(under the main checkout) plus `<same>.summary.json`: packets by type, every
sequence hole, how many of those the robot itself dropped (ring full / no
WiFi) and how many the network lost, tick rates per source, and the skew
between the robot's NTP stamps and mira's receive time. A new file starts on
`stop`/`start`, `flight_session.sh rotate`, a robot reboot, or 30 s of
silence.

Reading one:

```bash
tools/flight_reader.py summary S.frec
tools/flight_reader.py csv S.frec                 # S_ticks.csv (one row per tick), S_events.csv
tools/flight_reader.py align S.frec --pts hw_sessions/<d>/walk_sony.pts --serial hw_sessions/<d>/walk_serial.log
tools/flight_reader.py plot S.frec --joints R_knee,L_knee,L_ankle_roll --pts ... --serial ... --t0 17:44:05 --t1 17:44:20
```

`align` writes `S_<video>_frames.csv` (each video frame → the nearest tick by
its servo-read instant, with the offset) and `S_timeline.csv` (robot events
and serial-log lines on one UTC axis). In Python, `flight_reader.load()` →
`tick_rows()` gives per tick `<joint>.nom_rad / .goal_rad / .pos_rad /
.err_rad` in the joint frame (steps converted with the calibration the robot
itself sent in its HELLO), the IMU block and the balance block. The balance
block carries the `B<L|R>` PI (states 0-4) or, while it runs, the stance trim
of `pose ... S<L|R>` (states 5-9, `TRIM_*`: `integral` = the learned trim T of
the stance side, `u` = hike, `hip_rad` / `hip_ticks` = the joint correction).

### Forwarding

```bash
tools/flight_collector.py --forward 192.168.2.105:4221 [--forward HOST:PORT ...]
FLIGHT_FORWARD="192.168.2.105:4221" tools/flight_session.sh start walk2
```

`--forward HOST:PORT` (repeatable) re-sends every datagram the collector
receives — undecodable ones included — byte for byte and in arrival order,
to each destination, right after writing it to the session file: mira keeps
the file, a console gets the live stream and decodes it with
`tools/flightrec.py`. The forward is best effort: a non-blocking send from
the collector's own socket, so a refused port, an unreachable host or a full
buffer costs one counted error and never delays or stops the file. The
counts (sent, errors, the last error) are on the status line, in the
`session closed` line and in the summary's `forward` block, per destination.
A malformed spec is refused at argument parsing; each host is resolved once,
at start, and an unresolvable one stops the collector before it listens.
Nothing goes to the robot: a destination at the address a datagram came
from is skipped (and counted as `skipped_sender`). The session file is the
same with or without forwarding.

## What is recorded

| producer | when | one record per | feedback read |
|---|---|---|---|
| `ctrl` (core 1) | armed (`run` / wireless ARM), every tick incl. RELAX / ESTOP / FALLEN | 50 Hz tick | the tick's own SYNC READ — no extra bus traffic |
| `housekeeping`, source `pose` | a bench smooth stream: `pose ... s<ms>`, `pose ... B<L\|R>` (and its hold), `pose ... S<L\|R>` (and its fade), `home` and its play take-up | stream tick (`poseTick`) | one SYNC READ of the fitted servos, **after** the goal write |
| `housekeeping`, source `idle` | benched, nothing streaming | `rec idle` Hz (default 10) | one SYNC READ |
| `housekeeping`, events | always (while `rec` is on) | event | — |

Every TICK carries all of the following, at the record's own rate:

- header: boot id, sequence number, NTP UTC µs at the tick start (0 until
  SNTP answers), boot-clock µs, the robot's cumulative drop counters;
- tick: source, flags (torque, streaming, home, balance engaged, IMU valid,
  feedback read, goals written, fallen), reported link state, producer tick
  count, the producer's work time, the servo read's mid-point relative to the
  tick start and its duration, the IMU sample's age;
- IMU: accelerometer (m/s², body frame, mount applied), gyro (rad/s body;
  `ctrl`: averaged over the tick as the policy saw it, bench: the latest 4 ms
  sample), fused body→world quaternion, the policy's `up` vector, roll and
  pitch (ZYX from the quaternion);
- balance (bench): state (OFF / ACTIVE / ABORT_TIP / ABORT_IMU / ARMED),
  stance side, the corrected bus joints, ref, err, I, u, rate, the roll the PI
  used, ankle/hip correction in rad and ticks, envelope clamps;
- control (armed): the 7-wide command the policy saw, gait phase, pack mV,
  battery-guard state; bench: home stage;
- per fitted joint: **nominal** target (bench: the min-jerk profile before
  balance/abort; armed: the raw policy action through the trained action map,
  before shaper and takeover ramp; a held servo: its hold), **goal written**
  (incl. balance correction / landing / shaper / ramp) with its goal speed,
  whether it went out this tick, and from the servo's 15-byte feedback block:
  **present position**, speed, **load**, current, voltage, temperature, error
  flags, moving.

All of a joint's fields come from one STS3215 SYNC READ of registers 56–70,
so every field has the record's rate — there is no round robin.

Events: every CLI line executed (`pose` with all its arguments, `home`,
`balance set`, …), every line the CLI printed (balance ON / CONTINUES /
ARMED / PI starts / ABORT / landed, refusals, home readbacks), wireless ARM
and home edges, bench/run handovers with the arm verdict, balance state
changes, `ctrl` link-state changes, a bench read that backed off, and every
fall-guard step (fall on the bench, an armed fall handed over, pick-up ->
autohome, a clear), every flail-guard hold (its criterion and value) and its
autohome, and every operator stop (once per burst).
HELLO, 1 Hz: firmware version, NTP state, the calibration (servo ID, fitted,
dir, zero per bus joint), recorder counters and the `ctrl` tick budget
(`us_read` … `us_other`, overruns, worst tick).

### Rates

| source | rate | why |
|---|---|---|
| `ctrl` | 50 Hz | the hardware-timer tick |
| `pose` | the stream's own tick: 20 ms + the streamer's work. Logged 2026-10-29: 20 ms holding, 23–26 ms with a few joints writing (≈ 40–50 Hz); with all 12 writing, ≈ 40 ms (25 Hz, estimate) | `poseTick` evaluates when ≥ 20 ms have passed, from a loop that sleeps 10 ms after its work; each joint it writes first reads that servo's position with a single READ, which waits out the 3 ms reply timeout (`Bus::transact` asks the UART for a full buffer) |
| `idle` | `rec idle`, default 10 Hz | |
| HELLO | 1 Hz | |

The collector's summary and `flight_reader.py summary` report the achieved
median rate and the 5–95 % period per source, and per joint the share of
ticks it answered.

## The SIL twin as a source

`sim/sil_twin.py --flight HOST:PORT` sends the same stream from the
simulation: the firmware's control code (`libctrl_sil`) driving the CPU
plant ([control-channel.md](control-channel.md#simulated-robots-behind-the-link)). The packets are
`tools/flightrec.py`'s, built by the pure functions in
`sim/sil_twin_wire.py`; point it at a console, or at the collector
(`--flight mira:4220`), which files it like a robot. It fills only what the
sim has, and leaves the rest 0 with the flags saying so:

| field | the twin's value |
|---|---|
| header | random `boot_id` per run; one `seq` for every packet; `t_boot_us` from the twin's start; `t_utc_us` from the host clock; `drop_ring` 0; `drop_net` = sends that failed |
| HELLO, 1 Hz | `fw` = `sil_twin <run>` (31 B cap); `utc_synced` 1, `enabled` 1, `idle_hz` = the control rate, `cal_ok` 1; per bus joint `id` (`bus_map.h`), `fitted` = the plant has the joint, `dir`/`zero` = the calibration below; counters `produced`, `sent`, `drop_net`, ctrl ticks, overruns, worst tick, uptime — the rest 0 |
| TICK source and rate | `ctrl` while armed, `idle` while benched; one per control tick in both, at the twin's loop rate (paced to the 50 Hz control rate; the summary reports what it achieved) |
| tick flags | torque = the plant's torque is on; wrote = it tracked the goals this tick; feedback read; IMU valid (the plant has its `imu` site). Never traj, home, balance or fallen: the twin has no bench streams, balance loop or bench fall latch (its armed fall guard benches the run and sends the fall event) |
| `link_state`, `tick`, `work_us` | `Supervisor.state` (BENCH while benched), the loop's tick count, the tick's work time. `fb_off_us`, `fb_read_us`, `imu_age_us` 0: no bus, no IMU latency |
| IMU, at the tick start | `up`, `gyro`: the frame the SIL library fed its policy. `quat`: the `imu` site's true body→world attitude; `roll`, `pitch` from it (ZYX, as `rollPitchFromQuat`). `accel`: the site's true specific force, m/s², body frame, gravity included (MuJoCo's accelerometer) |
| balance block | 0 (state OFF, no buses) |
| control block | `cmd`: the 7-wide command the policy saw; `phase`: the library's gait clock — both from the frame it assembled; `vbat_mv`: the plant's supply voltage. `batt`, `home_stage` 0 |
| per joint | every plant joint on the bus, ascending bus index. `pos`, `speed`: the sim's angle and rate at the tick start — for a policy joint the observation the library was fed (its servo-side view), otherwise `qpos`/`qvel`. `nominal`: the raw action through `actionToAngles` (the robot's nominal). `goal`, `goal_speed`: `SilTargets.goal_ticks` / `goal_speed`, the shaped SYNC WRITE. `status`, `temp_c`, `voltage_dv`, `moving`, `load`, `current` 0 |
| joint flags | answered: always. Driven and nominal valid: while the controller's output is reported (armed, or torque held after a soft stop). Goal valid and written: while torque is on |
| events | arm edge (`a` = level), home edge, mode (`bench`/`run`, `a` = run \| owns-bus << 1, `b` = ArmResult), link state while armed (`a` = old or −1, `b` = new), an armed fall (`a` = 1 policy, `b` = 1 fell), operator stop (once per 2 s burst) |

Every step value is in one calibration per bus joint, the one the HELLO
sends: for the policy's joints the SIL library's own (the calibration
`goal_ticks` is written in), for the rest (2048, +1). `flight_reader`
therefore gives back the sim's angles in the joint frame, exactly as it
does the robot's.

## Timing

**Armed.** The record is built after the SYNC WRITE from what the tick holds;
the encode and ring push are the only cost (`rec` shows the worst,
"push cost max ctrl"). The tick budget with 12 servos: SYNC READ ≈ 3.4 ms
(0.28 ms per servo, measured for 10), policy 7.2 ms, write, mailbox and the
rest ≈ 0.2 ms — about 11 ms of 20.

**Bench.** A 12-servo SYNC READ is ≈ 3.4 ms on the bus (12 × 21-byte replies
at 1 Mbaud is 2.5 ms of wire). It runs after the stream tick's goal write, so
it never sits between the attitude read and the goals going out, and the
housekeeping loop takes exactly the time it spent back out of that
iteration's 10 ms sleep: the stream's — and so the bench balance loop's —
tick phase is the same with the recorder on as off (to the 1 ms FreeRTOS
tick). A read slower than 8 ms means a silent servo (each costs a 3 ms
timeout); the recorder then skips the read for 1 s, records without feedback,
and logs a `fb_backoff` event.

## Transport and loss

One UDP datagram per record to the configured collector. Producers encode
into their own lock-free single-producer ring (16 slots each); the
`flightrec` task (core 0, priority 3) drains them every 4 ms. A full ring
drops the record and counts it (`drop_ring`); a send with no WiFi or no
buffer drops and counts (`drop_net`). Nothing in either producer waits.
Every packet takes a number from one per-boot sequence, so a hole is a loss
wherever it happened, and the header's counters say which side admitted it.
There is no serial fallback: 50 records/s × 440 B is twice what the 115200
baud tether carries.

## Wire format (version 1)

Little-endian, float32. Header, 36 B:

| off | type | field |
|---|---|---|
| 0 | char[2] | `"FR"` |
| 2 | u8 | version (1) |
| 3 | u8 | type: 1 TICK, 2 EVENT, 3 HELLO |
| 4 | u32 | boot_id (random per boot) |
| 8 | u32 | seq |
| 12 | u64 | t_utc_us (0 = not synced) |
| 20 | u64 | t_boot_us |
| 28 | u32 | drop_ring (cumulative) |
| 32 | u32 | drop_net (cumulative) |

TICK body: 164 B fixed + 20 B × njoint (12 fitted: a 440 B datagram):

| off | type | field |
|---|---|---|
| 0 | u8 ×4 | source (1 ctrl, 2 pose, 3 idle), flags, link_state, njoint |
| 4 | u32 | tick |
| 8 | u32 | work_us |
| 12 | i32 | fb_off_us (servo read mid-point − tick start) |
| 16 | u32 | fb_read_us |
| 20 | i32 | imu_age_us |
| 24 | f32 ×15 | accel[3], gyro[3], quat[4] (w x y z), up[3], roll, pitch |
| 84 | u8, u8, i8, i8 | bal state, side ('L'/'R'/0), ankle bus, hip bus |
| 88 | f32 ×8 | ref, err, integral, u, rate, roll, ankle_rad, hip_rad |
| 120 | i16 ×2, u32 | ankle_ticks, hip_ticks, clamps |
| 128 | f32 ×8 | cmd[7], phase |
| 160 | u16, u8, u8 | vbat_mv, batt (0 ok, 1 landing, 2 limp), home_stage |
| 164 + 20k | u8 ×6 | bus, flags, status, temp_c, voltage_dv, moving |
| +6 | i16, i16, u16 | nominal, goal, goal_speed (steps, steps/s) |
| +12 | i16 ×4 | pos, speed, load (0.1 %), current (6.5 mA) |

Joint flags: 1 answered this tick, 2 nominal valid, 4 goal valid, 8 written
this tick, 16 driven (in the stream / by the policy).

EVENT body: u8 code, u8 src (0 housekeeping, 1 ctrl), u16 len, i32 a, i32 b,
then `len` ≤ 180 bytes of text. Codes: 1 cmd, 2 out, 3 arm edge (a = level),
4 home edge, 5 rec, 6 mode (a = run|ctrl-owns-bus<<1, b = ArmResult),
7 balance state (a = old, b = new), 8 link state (a = old, b = new),
9 feedback backoff (a = µs the read took), 10 fall (a = the motion it fell
in: 0 bench, 1 policy; b = 1 fell, 2 picked up -> autohome, 3 cleared;
`components/fallguard`), 11 flail (an armed-loop hold: a = 1 pose, 2 rate,
3 tilt, 4 chatter, 255 operator soft stop; b = 1 torque held, 0 already limp,
2 the hold's autohome started; `components/flailguard`), 12 stop (an operator
stop, `linkproto::kFlagStop`: a = 1 hard / 0 soft, b = 1 armed / 0 benched),
13 Wi-Fi CLI session (a = peer IPv4, b = peer port; text says). On cmd and
out events `a` is the channel: 0 the tether, 1 the Wi-Fi CLI
(docs/wifi-cli.md), and a Wi-Fi cmd's `b` is its command seq. Logged command
text has `wcli key <hex>` and the PSK of `wifi <ssid> <psk>` redacted -- the
recorder sends in clear.

HELLO body, 116 B + 5 B × 17: fw[32], utc_synced, enabled, idle_hz, cal_ok,
19 × u32 (produced, sent, drop_ring, drop_net, max push ctrl/house µs, last
bench read µs, ctrl ticks, overruns, worst tick µs, us_read, us_imu, us_obs,
us_net, us_write, us_other, uptime ms, NTP syncs, reserved), u8 17 + 3 pad,
then per bus joint: u8 id, u8 fitted, i8 dir, i16 zero.

The C++ encoder is pinned to `tools/flightrec.py` byte for byte
(`make vectors` → `firmware/host/vectors/flightrec_vectors.h`, checked by
`firmware/host/test_flightrec.cpp`). A layout change bumps the version on
both sides.

Session file (`.frec`): `FRECSESS1\n`, one line of JSON metadata, then per
datagram `u16 length, u64 mira receive time (UTC µs), packet` — raw, so a
decoder fix re-reads old sessions.
