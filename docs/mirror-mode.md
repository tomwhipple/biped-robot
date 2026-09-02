# Mirror mode: driving the robot with a sim following alongside

*Status 2026-09-01. **The laptop half is done and exercised; the firmware half
is not written.** Both twins beacon joint angles on request, and `bimo_gui`
mirrors against them end to end. What remains is on the board — see
[the hardware TODO](#the-hardware-todo).*

The idea: `bimo_gui` drives the **real robot**, the robot beacons back its
**measured joint angles**, and `sim/sil_twin.py` runs the same command through
the same policy while drawing the robot's observed pose over it as a ghost.
Divergence between the two is then visible directly, which is the sim-to-real
question stated as a picture instead of a number.

## The wire decision, and the one it could not copy

The extended *command* frame (2026-08-31) solved "more channels" by selecting
on **length**: `kCmdLen` 14 or `kCmdLenExt` 24, no version bump. The robot is
the decoder there, and it was taught both.

**Telemetry cannot do that**, and the difference is worth being explicit
about. Here the robot is the *sender* and every client is a decoder that
checks the length and rejects anything else — `decode_telemetry`,
`decodeTelemetry`, and through them `bimo_tui`, `link/commander.py`,
`link/link_twin.py` and `sim/udp_agent.py`. A robot that simply started
beaconing 40 B would **blind every one of them at once**, which is precisely
the trap the bench diagnostic had to dodge in
[control-channel.md](control-channel.md#saying-why-over-the-radio-2026-08-30).

So the long frame is **requested, never volunteered**:

| bit | flag | |
|---|---|---|
| 0 | `ENABLE` | |
| 1 | `ESTOP` | |
| 2 | `ARM` | |
| **3** | **`POSE`** | **"beacon joint angles too"** |

A commander sets `FLAG_POSE` in its command frames; only then does the robot
answer with the long beacon. A client that never asks never sees a byte it
does not understand, and a client that asks is *by construction* one that can
read the answer. No version bump, no length surprise, and the request rides a
flags bit that was already spare.

### The invariant the firmware must hold: {peer, pose} is one snapshot

"A client that asks is by construction one that can read the answer" is only
true if the *pose request and the destination come from the same command*.

The robot beacons to whoever commanded last — every valid-CRC frame updates
`peer` (`wifi_link.cpp`). So if the pose level and `peer` are read from
different commands, a mirror-aware console **B** that sends one plain command
between console **A**'s `FLAG_POSE` commands can receive a 40 B frame it never
asked for, and **A**'s mirror keeps working only by luck of which frame landed
last.

**Snapshot `{peer, pose_wanted}` together, from the same accepted command, and
treat pose as a level tied to that snapshot** — not as task state updated
independently of the peer. Writing it down here because this is exactly the
one-sender-many-decoders trap the rest of this document is careful about, and
because it is invisible until two consoles are pointed at one robot.

**Extended telemetry** — 48 B, `TLM_LEN_EXT` (40 B before the timestamp
of 2026-09-02, still decoded):

| off | size | field |
|---|---|---|
| 0–17 | 18 | exactly the classic body, unchanged |
| 18–25 | 8 | `t_us`, the robot's clock — the instant these joints were read ([control-channel.md](control-channel.md#time-on-the-wire-2026-09-02)) |
| 26–45 | 20 | 10 × i16, milli-radians, **obs_spec joint order** |
| 46–47 | 2 | crc16 over bytes 0–45 |

*Note (2026-09-02).* The timestamp that now precedes the joints is the one
telemetry field that IS volunteered on every frame, against the rule this
section lays down. Both decoders accept the pre-stamp lengths too, so no
console in this tree goes blind against older firmware; the reasoning, and
the one pairing that does break, are in control-channel.md's "Time on the
wire".

Joint order is obs_spec's, not servo-id order: `L_hip_yaw, L_hip_roll,
L_hip_pitch, L_knee, L_ankle`, then the same five right. This is the same
order `servo_err`'s bits already use, and the order `g_q` is stored in — the
servo ids are *not* in joint order and mixing the two is a bug that looks like
a mirrored robot.

±32.767 rad of headroom on joints that travel about ±2. Bandwidth at 10 Hz
goes from 1.6 to 3.2 kbit/s, which remains a non-issue.

## What runs where

```
  bimo_gui ──cmd (FLAG_POSE)──> ROBOT ──beacon: state + 10 joint angles──┐
      │                                                                  │
      ├──the same cmd, UDP──> sil_twin (its own policy, own ports)       │
      └──"pose q0..q9" down sil_twin's stdin @10 Hz <────────────────────┘
                                   │
                                   └─> renders: policy pose SOLID
                                                observed pose GHOST
```

The pose goes down `sil_twin`'s **stdin**, not a new socket: the console
already owns that pipe (it is how `reset` works), so mirror mode costs no new
listener and no new port to collide with.

`sil_twin` runs on ports offset from the robot's, because in mirror mode both
are being commanded at once and they cannot share 4210/4211.

## The hardware TODO

Everything below is firmware, and **none of it has been written or flashed**.
It is specified against files as they stand at this commit.

- [ ] **`linkproto` C++ mirror of the wire format.**
      `firmware/components/linkproto/include/linkproto/protocol.h`: add
      `kFlagPose`, `kNumJoints = 10`, `kTlmLenExt = 40`, and give `Telemetry`
      a `float joints[10]` plus an `n_joints` (0 = classic frame). Teach
      `encodeTelemetry` to emit the long frame when `n_joints` is set and
      `decodeTelemetry` to accept both lengths. `link/protocol.py` is the
      reference and is already done — port it, do not re-derive it.
- [ ] **Golden vectors.** `tools/gen_protocol_vectors.py` →
      `firmware/host/vectors/protocol_vectors.h`, then assertions in
      `firmware/host/test_protocol.cpp` and `tests/test_protocol.py`. The
      point of these is that the two implementations are diffed against
      *bytes*, not against each other's opinions.
- [ ] **Publish `g_q` without tearing.** `ctrl_task.cpp:53` already holds
      `g_q[obs::kNumJoints]` — calibrated joint angles in radians, which is
      exactly the payload. It must reach the housekeeping core.
      **Do not add ten loose atomics.** `shared.h:116-127` already argues this
      case for `ObsDump`: a 10-wide payload "is too big to tear harmlessly",
      and a beacon that mixed tick *t*'s `q[0]` with tick *t+1*'s `q[9]` would
      invent exactly the kind of skew mirror mode exists to detect. Use the
      same double-buffer-plus-sequence-counter pattern, and **drop** a copy
      the writer overran rather than send it.
- [ ] **Remember whether the commander asked.** The link mailbox must carry
      the last accepted command's `FLAG_POSE` through to the beacon builder.
      Treat it as a *level*, not an edge — unlike `ARM`, there is nothing to
      latch: a commander that stops asking should simply stop getting long
      frames on the next beacon.
- [ ] **Build the long beacon.** `wifi_link.cpp:145-176`. Note the buffer is
      declared `uint8_t wire[linkproto::kTlmLen]` and the length is passed to
      `lwip_sendto` as `kTlmLen` in two places — both must become the length
      `encodeTelemetry` actually returned, or the frame is silently truncated
      and every CRC fails.
- [ ] **Decide about `link_task.cpp`.** The UART tether builds telemetry too
      (`wifi_link.cpp:169` says "same as link_task"). Either give it the same
      treatment or state in a comment that the tether stays classic-only, so
      the next person does not have to work out which it is.
- [ ] **Bench check, in this order.** Flash, then `bimo_gui --host <ip>`
      (no `--mirror`, so the mode is off) first and confirm the classic 20 B beacon is unchanged —
      that is the regression that matters, because it is what every other
      commander depends on. Only then turn mirror on.
- [ ] **Then, and only then, with the robot on the stand.** Joint angles are
      read-only, so this adds no new way to move the robot — but arming does.
      Remember that any goal-position write is a motion command, and that
      opening the CP2102 port reboots the board and eats the first CLI
      command.

### A benched robot sends no pose

Reported from the firmware side while implementing it, and worth knowing
before it looks like a bug: while `BENCH`ed the control loop publishes no
joint angles, so a mirror-on console sees **classic** frames until it arms.
The ghost appears on arm, not on ticking the box. This is consistent with
everything else about `BENCH` — the loop is not running, so there is nothing
measured to report — and it is why the console says *"asked for joint angles;
robot sends none yet"* rather than claiming the feed is broken.

### State of the laptop side

All done, and none of it needed a board:

- `link/protocol.py` and the `linkproto` C++ port — `FLAG_POSE`,
  `TLM_LEN_EXT`, joints on `Telemetry`, both frame lengths. Cross-checked
  byte-for-byte, and frozen in the golden vectors.
- `link/link_twin.py` — beacons a slow deterministic wobble when asked, and
  logs the pose on/off transition, so a client in mirror mode can tell a
  working feed from ten zeros.
- `sim/sil_twin.py` — beacons its own joint angles when asked; takes
  `pose q0..q9` on stdin and composites it as a ghost, by re-posing the SAME
  model and camera rather than rendering a second one.
- `bimo_gui` — the mirror toggle: sets `FLAG_POSE`, relays every command to
  the sim's own ports (`--sim-cmd-port`, default `cmd_port + 100`) as well as
  the robot's, and pushes each observed pose down the child's stdin. One
  encode, two destinations, so the sim cannot be commanded differently from
  the robot by construction.
- `--mirror` **starts that sim**, rather than only arming the mode. Mirror
  with no child is a console that requests joint angles and draws them
  nowhere: the pose relay is gated on a running sim, the long beacon is not
  recorded, and the relayed commands land on a port nobody is bound to. The
  flag is equivalent to coming up and pressing Start (on `--run-name`, or the
  first run with SIL weights); `--sim-view` opts out, since that says the
  operator already has a stream. Watch it at `127.0.0.1:<stream-port>` — the
  child is forked *here*, so in mirror mode its stream is NOT on `--host`,
  which is the robot.
- `tests/test_protocol.py`, `firmware/host/test_protocol.cpp`,
  `tests/test_gui_e2e.py`.

Verified end to end on 2026-09-01 with `link_twin` standing in for the robot:
the twin logged `pose requested -- beaconing joint angles`, `sil_twin` logged
`ghost pose feed live`, and both reverted to the classic beacon when the
console stopped asking.

Still open, and deliberately not done here:

- [ ] **Plant-config match.** Supply voltage, payload and control latency
      taken from the real robot rather than the run's training config, so the
      sim is loaded like the hardware is. `vbat` is already on the beacon;
      payload and latency are not, and guessing them would make the ghost
      agree with the robot for the wrong reason.
