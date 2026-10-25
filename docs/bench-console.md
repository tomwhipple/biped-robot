# Bench console: cameras and telemetry on the Mac

Wi-Fi to the ESP32 is the robot's primary interface (UDP 4210 command, 4211
beacon; [control-channel.md](control-channel.md)); USB is for flashing and for
what Wi-Fi cannot do. Today that includes the bench balance mode
(`pose ... B<L|R>`), which exists only in the serial CLI (115200 baud), and
the CLI sends no beacon, so the console's usual strip charts are empty. This
is how Tom watches such a session on his Mac: live camera views and the CLI's
readings, in the one `bimo_gui` window, and all of it saved for sim playback.

**One model, two sources.** The console's bench panel is fed by the datagram
schema below, not by the serial port. Any producer of those samples fills it:
the serial bridge today; a Wi-Fi-side producer later, once the beacon carries
balance and servo readings. The beacon already feeds the one field both
share, the IMU `up` vector, directly in the console. Nothing in the panel
depends on the CLI's text.

```
 robot ──USB serial── mira                                       Mac
                       board_bridge.py  ──UDP JSON, :8650──▶  bimo_gui
                         (holds /dev/ttyUSB0,                    BENCH view
                          FIFO in, log out)                      ├ camera panes
                       cam_relay.py     ──MJPEG/HTTP, :8647─▶   └ telemetry
                         (ffmpeg per camera, on demand)
```

Everything the console shows is recorded (below). Both servers bind mira's **Tailscale** address (100.105.166.4), never the LAN.
Everything the Mac does is watch-only: the console without `--host` is an
observer with no transmit socket, and the only datagram it sends is the
bridge subscription.

## Run it

**On mira**, once per bench session:

```sh
# 1. The serial bridge. The port is opened ONCE, here (opening resets the
#    board and boot releases torque). Commands go in through the FIFO.
mkfifo /tmp/board.fifo 2>/dev/null
.venv/bin/python tools/board_bridge.py /tmp/board.fifo hw_sessions/$(date +%F)/serial.log \
    --poll --cal-from hw_sessions/2026-10-24/stand/serial.log &
echo "pos 52" > /tmp/board.fifo            # an operator command
echo "__quit__" > /tmp/board.fifo          # closes the port, exits

# 2. The camera relay (cameras open only while a console watches them).
.venv/bin/python tools/cam_relay.py &
```

`--poll` is optional. Without it the console shows whatever the operator's
own commands (`pos 52`, `balance`, `imu`) print; with it the bridge also asks
by itself (below). `--cal-from LOG` seeds the per-joint zero from an old
`cal show`; a `cal show` typed at the CLI is picked up live.

**On the Mac:**

```sh
brew install glfw
make -C firmware/host deps gui
firmware/host/build/bimo_gui --bench-telemetry mira:8650 --cam-relay mira:8647 --cam emeet
```

`mira` resolves through Tailscale. `--cam NAME` (up to two, `emeet`, `brio`
or `sony`) opens panes at start; the combo box in each pane changes source and
`(off)` releases the camera. `--drive-view` starts on the DRIVE view instead;
the `bench view` / `drive view` button switches. Add `--host <robot-ip>` only
when this console should also be able to take control of the robot over
Wi-Fi (it still starts as an observer; with `--readonly` it listens for the
beacon of whoever drives, see the watcher recipe in
[control-channel.md](control-channel.md)).

## What the window shows

- **CAMERAS**: one or two panes from the relay. A pane with no new frame for
  2 s says **STALE** in red and is tinted: a frozen frame never looks live.
- **BENCH TELEMETRY**: the bridge status (waiting / live / **STALE** when no
  datagram for 3.5 s), the balance state, 30 s strip charts of balance
  `roll`, `err` and `u`, the last ankle/hip correction.
- **IMU**: `up` and the tilt from vertical, `levelled`, roll, and the last raw
  accel/gyro sample.
- **SERVOS**: the 12 servos with position, position minus the cal zero (raw
  ticks and degrees; direction not applied), load, temperature, bus voltage,
  fault byte and the age of the reading. Readings older than 15 s, or all of
  them when the bridge is silent, are greyed.

## Camera relay (`tools/cam_relay.py`, TCP 8647)

One process, one URL per source, in the house MJPEG convention that
`link/mjpeg.h` consumes (multipart/x-mixed-replace, Content-Length per part,
newest frame wins):

| URL | source |
|---|---|
| `/emeet` | EMEET C960, `/dev/video0`, MJPEG 1280x720 at 10 fps (floor-level view of the feet) |
| `/brio` | Brio 101, `/dev/video2`, MJPEG 960x720 at 10 fps |
| `/sony` | Sony A7 III: ffmpeg decodes mediamtx's `rtsp://127.0.0.1:8554/camera` and re-encodes MJPEG at 10 fps |
| `/<name>.jpg` | the newest frame once |
| `/` | JSON status of every source |

- **MJPEG in, never raw.** A webcam is asked for its own MJPEG
  (`-input_format mjpeg`) and passed through undecoded; two raw YUYV streams
  once wedged the shared USB2 controller. The camera is asked for 10 fps, so
  the bus carries no more than that.
- **On demand.** ffmpeg starts at the first viewer and stops 10 s (`--idle`)
  after the last. Nothing holds a camera nobody watches. The Sony is
  exclusive: while a console watches `/sony`, mediamtx holds the camera, so
  `tools/sony_record.sh` and `liveview.py` cannot have it (and the reverse).
- **A webcam serves one reader**: `tools/cam_live.py` (the side-by-side
  composite, :8646) and `tools/cam_record.sh` cannot run while the relay is
  viewing the same device.
- `--source NAME=KIND:TARGET[,size=WxH][,fps=N]` overrides the presets;
  `lavfi:testsrc` gives a synthetic source that opens no hardware.

## Serial bridge (`tools/board_bridge.py`, UDP 8650)

**The port and the log.** The bridge opens the port once, sends each FIFO
(or `--stdin`) line to the board and appends every line in both directions to
the log with a UTC stamp: `>` an operator command, `>~` a command the poller
sent, two spaces for a board line, `#` the bridge's own notes. (The format of
`hw_sessions/*/*serial.log`; `>~` lines are the only addition.)

**Subscribing.** A console sends `{"sub":1}` to the port and repeats it every
2 s; the lease is 10 s, at most 4 consoles, `{"unsub":1}` leaves. The bridge
never connects out. A new subscriber is first sent the newest sample of each
kind so it is not blank until the next poll. A console cannot make the bridge
write to the port: a datagram is read only as subscribe/unsubscribe.

**Datagrams.** One JSON object, at most 1200 bytes, per UDP datagram. Every
object has `v` (1), `t` (bridge UTC epoch seconds; informational: the console
charts on its own arrival clock) and `k`:

| `k` | fields | from |
|---|---|---|
| `joint` | `id name pos spd load volt temp err` | `pos <id>` |
| `cal` | `id name zero dir fitted` | `cal show` |
| `bal_gains` | `kp ki ankle hip clamp abort` | `balance` |
| `bal_roll` | `roll` (deg; `null` = no fresh IMU attitude) | `balance` |
| `bal_state` | `state stance since` (`state` is `off`, `ACTIVE`, `FROZEN (...)`) | `balance`, periodic status |
| `bal_ref` | `ref err I u` (deg) | `balance` |
| `bal_corr` | `side ankle_deg ankle_ticks hip_deg hip_ticks clamped` | `balance` |
| `imu_up` / `imu_proj` / `imu_gyro` | `x y z` | `imu` |
| `imu_level` | `levelled streak` | `imu` |
| `imu_raw` | `ax ay az amag gx gy gz` | `imu raw N` |
| `hb` | `polling rx_lines polls rx_age subs` (1 Hz) | the bridge |

Lines are parsed whatever their origin, so the board's own periodic balance
status is published as it prints. Matching is not anchored to the line start:
the firmware's lines tear under load (`...the mostate    ACTIVE, L stance`),
and those are the lines that matter while balancing. Unknown kinds are
ignored by the console; a version other than 1 is refused.

**Polling (`--poll`, off by default).** The only commands the bridge ever
composes itself are `pos <id>` (the 12 servos), `balance`, `imu` and, with
`--poll-raw`, `imu raw 1`: read-only queries, pinned by
`tests/test_board_bridge.py`. A poll goes out only when

1. no operator line is waiting and none was sent in the last 1.0 s, so an
   operator's command and its reply always go first;
2. the previous poll's reply is finished (board silent 0.12 s, or 0.6 s
   elapsed), so at most one query is ever outstanding;
3. at least 0.25 s since the last poll (a ceiling of 4 queries/s).

The most overdue query goes first: `balance` every 1.0 s, `imu` every 2.0 s,
each servo every 6.0 s (`--poll-balance/-imu/-joint`). That is about 3.5
queries/s and roughly 1 kB/s of the 11.5 kB/s link; under load every query
slows in proportion and none starves.

## Recording for sim playback

Nothing the bench view shows is ephemeral:

| what | where | clock |
|---|---|---|
| bench datagrams the panel accepted, verbatim, plus `cam` rows naming each pane's source | the console's session file `hw_sessions/<stamp>-<host>.jsonl` (the usual recorder, on by default; `--no-record` opts out) as `{"t","utc","dir":"bench","d":{...}}` and `{"dir":"cam","pane","src","url"}` rows, beside the beacon `tlm` and command `cmd` rows | `utc` = the console host's NTP clock |
| camera frames | on mira, `cam_relay.py --record-dir DIR`: `DIR/<UTC stamp>_<source>.mp4` + `.pts` (epoch ms per frame, as `tools/sony_record.sh`) while someone watches | mira's NTP clock |
| the serial transcript | on mira, the bridge's log | UTC stamp per line |

The `utc` columns, `.pts` and log stamps are all NTP wall time, so a frame, a
datagram and a CLI line are joined by subtracting. `tools/replay_session.py`
reads only `dir == "cmd"` rows and skips the rest, so the same file still
replays its commands into `sim/sil_twin.py`. Copy the mp4/pts/log next to the
JSONL (`hw_sessions/<date>/<take>/`) to keep a take together.

## Forwarding the Wi-Fi beacon (`--beacon-listen`, `--watch`)

The robot beacons only to the console that commanded it last. If that console
runs on mira, give the bridge a relay so other machines can decode the same
beacons:

```sh
# mira: the bridge listens on :4311 and forwards each beacon byte for byte
.venv/bin/python tools/board_bridge.py /tmp/board.fifo serial.log --beacon-listen 4311 \
    --watch 192.168.2.105:4211
# the driving console on mira points its own relay at the bridge
bimo_gui --host 192.168.2.90 --watch 127.0.0.1:4311
# the laptop: an observer, decoding with the driver's own code
bimo_gui --readonly --bench-telemetry mira:8650 --cam-relay mira:8647
```

`--watch` may repeat (default port 4211). Datagrams are never parsed or
re-framed, and the relay never touches the serial port.

## Tests

`tests/test_board_bridge.py` (parser against the real `hw_sessions`
transcripts, the poller on a fake clock, the bridge against a pty standing in
for the board), `tests/test_cam_relay.py` (ffmpeg test patterns only, including the recording; the
v4l2 and RTSP command lines are checked as strings, never run),
`firmware/host/test_client.cpp` (JSON reader, model, time series, UDP link)
and `tests/test_gui_bench_e2e.py` (the real `bimo_gui --headless` against the
real bridge and relay, including the session file's `bench` and `cam` rows). The window itself is the one part with no automated
test.
