# Mirror mode: the robot drawn from its own telemetry

*A 3D picture of the real robot in the console, posed from what it measured.
The wire protocol it rides on is [control-channel.md](control-channel.md).*

`bimo_gui --mirror` (or the mirror toggle) does three things:

1. sets `POSE` and `ATT` in its command frames, so the robot's beacon
   carries its measured joint angles and torso up vector (the 69 B frame);
2. starts `sim/sil_twin.py --viewer` as a child process;
3. for each beacon that carries joints, writes one line to the child's stdin:
   `pose q1 … q17 [ux uy uz]` (radians, servo-ID order, then the up vector
   when the robot sent one).

The viewer poses the plant from that line, **by joint name**
(`protocol.JOINT_NAMES`): a plant joint the line names is posed, and a wire
joint the plant does not have (the prototype plant has no ankle rolls, neck
or arms) is skipped. It renders the result. **No policy, no
physics, no integration**: the picture is a mannequin held in the last state
the robot reported, and every part of it is a measurement. The console draws
the viewer's stream in its own window.

## The invariant: only measurements, and say when they stop

A view that blends a measurement with a prediction, and degrades to pure
prediction without looking any different, is worse than no view. So the
viewer never runs the sim forward, and when the pose feed stops the console
says **FROZEN** with the age of the pose on screen, rather than letting a
held picture pass for a live one.

**A benched robot sends no pose.** While `BENCH` the control loop reads no
servos, so the beacon carries no joint block even when asked, and the console
says *"a BENCHED robot measures nothing; the pose appears on arm"*.

## Attitude

`up_z` alone is the tilt magnitude, never its direction: joint angles and
`up_z` draw a robot lying on its face as one standing to attention. `ATT`
adds `up_x` and `up_y`, the torso's z axis in the **world** frame (obs_spec
`kOffUp`, `imu::Sample::up`, MuJoCo's `framezaxis`).

The viewer turns `up` into the shortest rotation taking world +Z onto it.
That fixes roll and pitch and draws **yaw as zero**: the robot has no
magnetometer in use and its heading is a drifting gyro integral, so a drawn
heading would dress a drifting number up as a measurement. For the same
reason `up_x`/`up_y` are yaw-dependent where `up_z` is not; they are good
enough to draw an attitude, not a heading. Torso **position** is pinned,
because the robot has no odometry. With `--hang` the torso is welded and only
the joints move.

## On the wire

Both blocks are **requested, never volunteered**: the robot is the sender
and every client rejects a telemetry length it does not know, so a robot that
began sending joints unasked would blind every console at once. A client
that sets `POSE` is by construction one that can read the answer.

| length | blocks | asked for with |
|---|---|---|
| 31 | base (with `t_us`) | — |
| 35 | base + attitude | `ATT` |
| 65 | base + joints | `POSE` |
| 69 | base + joints + attitude | both |

- The joint block is 17 × i16 milli-radians, one per **bus** servo in
  **servo-ID order** (`protocol.JOINT_NAMES`, `obs/bus_map.h`): IDs 1–10 the
  legs as on the prototype, then the ankle rolls, the neck and the arms
  ([servo-map.md](servo-map.md) §2.1). It is not the policy's joint order;
  consumers map it by name. A servo the robot does not carry (IDs 11–17 on
  the prototype) reads 0.0.
- With joints present, `t_us` is the instant they were read off the bus.
- The robot publishes the pose every armed tick through `main/joint_pose.h`
  (a double buffer with a sequence counter, so one beacon never mixes two
  ticks' joints) and sends the long frame only when the publish count has
  moved since the last beacon, so a stale pose is never sent as fresh.
- The UART tether stays at the 31 B base frame; nothing consumes joints
  over the cable.

### The invariant the firmware must hold: {peer, pose} is one snapshot

The robot beacons to whoever commanded last. If the destination and the
`POSE`/`ATT` levels came from different frames, a plain console B sending one
frame between mirror console A's frames could be handed a long frame it
never asked for, and drop it. So `main/wifi_link.cpp` keeps
`Commander{address, pose, att}` as one value, replaced whole from each
accepted frame. `POSE` and `ATT` are levels tied to that snapshot, not
latches: a commander that stops asking gets the short frame on the very next
beacon.

## Ports and watchers

In mirror mode the robot owns 4210/4211, so the viewer runs on its own ports:
`--sim-cmd-port` (default the command port + 100) and the next one up. Its
stream is served on `127.0.0.1:<stream-port>`, because the console forked
it locally; in mirror mode `--host` is the robot.

A `--readonly` watcher can mirror too, but cannot ask for joints (that would
be a frame on the wire). It draws the joints that arrive in the relayed
beacon, which requires the **driving** console to run `--mirror`; the
watcher says so instead of showing an empty picture.

## Bench check

1. Flash, then run `bimo_gui --host <ip>` **without** mirror. On the tether,
   `wifi` prints `tlm tx N (pose M, asked|not asked)`: `M` must not move and
   the line must say `not asked`. Every other console depends on this.
2. Turn mirror on, arm with the robot on the stand: `asked`, and `M` climbs at
   the beacon rate. Joint angles are read-only, so mirror mode adds no way to
   move the robot; arming does, so the bench rules in
   [AGENTS.md](../AGENTS.md#bench-safety) apply.

Covered by `tests/test_protocol.py` and `firmware/host/test_protocol.cpp`
(the frame lengths, byte for byte), `firmware/host/test_obsdump.cpp` (the pose
buffer) and `tests/test_gui_e2e.py` (the console against `link/link_twin.py`,
which beacons a slow deterministic wobble when asked and logs the request
turning on and off).
