"""SIL twin: the REAL firmware control stack behind the real UDP link.

    .venv/bin/python sim/sil_twin.py --hang --stream-port 8645
    make -C firmware/host tui && firmware/host/build/bimo_tui --host mira

Where udp_agent.py runs the PYTHON policy stack behind the link, this runs
libctrl_sil -- the firmware's own obs assembler, gait clock, MLP and
actuation code, compiled for the host -- so what you drive from the console
is the same control law the robot executes, tick for tick. With --hang the
plant is welded to the world at stand height, feet free: the sim analogue of
the test stand, for eyeballing sim-vs-bench with the SAME console inputs
(2026-08-30: the bench gait resembled nothing; the question is whether the
sim twin, hanging, does the same thing for the same reasons).

Live view: --stream-port serves MJPEG on 0.0.0.0 (http://mira:<port>/) so
the rendered sim is watchable next to the physical robot. --record writes
the same frames to an mp4 on exit (hw_sessions/... is the natural home).

Flight records: --flight HOST:PORT (off by default) sends the stream the
robot's flight recorder sends -- HELLO at 1 Hz, a TICK every control tick
(goal written, nominal target and measured position per joint, IMU, the
command the policy saw), events for arm/home edges, mode, link state and
operator stops -- encoded by tools/flightrec.py, to a console or to mira's
collector (tools/flight_collector.py, UDP 4220). sim/sil_twin_wire.py builds
the packets; docs/flight-recorder.md lists which fields the sim fills.

    .venv/bin/python sim/sil_twin.py --hang --flight 127.0.0.1:4221

The beacon follows the firmware's rules: joint angles for a commander that
asks with FLAG_POSE, the up-vector x/y for one that asks with FLAG_ATT.
"""
import argparse
import io
import select
import json
import os
import re
import socket
import struct
import sys
import threading
import time

# EGL is the right headless backend on the Linux training box (mira, under
# cron with no X display). macOS has no EGL AT ALL -- MuJoCo raises
# "invalid value for environment variable MUJOCO_GL: egl" before it renders a
# single frame -- so let it pick its own there. Passing MUJOCO_GL by hand
# masked this for a long time; bimo_gui's Start button does not pass one, and
# died every time as a result.
if sys.platform != "darwin":
    os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("JAX_PLATFORMS", "cpu")   # eval_precision imports jax

import numpy as np                                          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in ("mjx", "sil", ""):
    sys.path.insert(0, os.path.join(HERE, p))
sys.path.insert(0, os.path.join(ROOT, "link"))

from protocol import (CMD_PORT, JOINT_NAMES, NUM_JOINTS, TLM_PORT,  # noqa: E402
                      ProtocolError, Supervisor, decode_command)
import sil_twin_wire as W                                   # noqa: E402
import flightrec as FR                                      # noqa: E402  (W put tools/ on the path)

# Needs BOTH a sim/runs/<name>/config.json and an exported
# sim/sil/weights/<name>.silw.json (bimo_gui's SIM panel builds its dropdown
# from that same pair), and the run must match the compiled-in spec: the
# DEPLOYED run, obs_spec.h's kRunName -- any other run fails sil_check_spec.
def _deployed_run():
    spec_h = os.path.join(ROOT, "firmware", "components", "obs", "include",
                          "obs", "obs_spec.h")
    try:
        m = re.search(r'kRunName\s*=\s*"([^"]+)"', open(spec_h).read())
    except OSError:
        m = None
    return m.group(1) if m else "loco_v11gait"


DEFAULT_RUN = _deployed_run()
HANG_XML = os.path.join(HERE, "bimo_biped_v5body_hang.xml")


# -- MJPEG live view --------------------------------------------------------
class Mjpeg:
    """Latest-frame MJPEG server: one JPEG shared across viewers, ~10 fps.
    LAN-only convenience (0.0.0.0, the mira:<port> convention)."""

    BOUNDARY = b"--siltwinframe"

    def __init__(self, port):
        import http.server
        self.jpeg = None
        self.lock = threading.Lock()
        outer = self

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=siltwinframe")
                self.end_headers()
                try:
                    while True:
                        with outer.lock:
                            buf = outer.jpeg
                        if buf:
                            self.wfile.write(outer.BOUNDARY + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(buf)}\r\n\r\n".encode())
                            self.wfile.write(buf + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return

        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"sil_twin: MJPEG live view on http://0.0.0.0:{port}/")

    def push(self, rgb):
        import imageio.v2 as imageio
        buf = io.BytesIO()
        imageio.imwrite(buf, rgb, format="jpeg")
        with self.lock:
            self.jpeg = buf.getvalue()


def _local_xml(xml):
    """A run's config.json may store the plant relative to sim/ (or just its
    basename: train_mjx.py on a remote host) -- resolve it against the local
    sim/ directory, as eval_precision does, so the twin starts from any cwd."""
    if not xml or os.path.isabs(xml) or os.path.exists(xml):
        return xml
    for cand in (os.path.join(HERE, xml),
                 os.path.join(HERE, os.path.basename(xml))):
        if os.path.exists(cand):
            return cand
    return xml


def run(args):
    from eval_precision import (make_env, sil_check_spec,   # noqa: E402
                                sil_profile_for, sil_setup)

    run_dir = os.path.join(HERE, "runs", args.run_name)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = args.xml or (HANG_XML if args.hang else None) \
        or _local_xml(cfg.get("xml_path")) or None
    # the policy profile is the TRAINING plant's (a --hang override is a
    # variant of the same body)
    prof = sil_profile_for(cfg.get("xml_path") or "")
    H, lib, info = sil_setup(args.run_name, run_dir, profile=prof)
    extra = {}
    if args.backlash_deg is not None:
        extra.update(backlash_deg=args.backlash_deg, backlash_deg_max=None)
    if args.act_delay_ticks > 0:
        extra.update(act_delay_ticks=int(args.act_delay_ticks))
    if args.play_deg > 0.0:
        extra.update(play_deg=args.play_deg,
                     play_joints=tuple(args.play_joints.split(","))
                     if args.play_joints else None)
    for kv in (args.plant or []):
        # --plant servo_kp=8 payload_mass=0.15 friction_range=0 : any
        # BimoWalkerEnv kwarg, pinned on top of the claim/nominal plant
        # (make_env applies extra LAST). Numbers parse as float, else str.
        k, _, v = kv.partition("=")
        try:
            v = float(v)
        except ValueError:
            v = {"true": True, "false": False, "none": None}.get(v.lower(), v)
        extra[k] = v
    env = make_env(cfg, episode_seconds=1e9, nominal=args.nominal,
                   xml=xml if xml else _local_xml(cfg.get("xml_path")),
                   extra=extra or None, act_lag_hz=args.act_lag_hz)
    print(f"sil_twin: plant backlash {env.backlash_deg:g} deg, "
          f"act_lag {env.act_lag_hz:g} Hz, dead time {getattr(env, 'act_delay_ticks', 0)} ticks, play {env.play_deg:g} deg "
          f"on {env.play_joints or 'all joints'}", flush=True)
    sil_check_spec(H, env)
    act = H.SilActAdapter(lib, env, log=False)
    obs, _ = env.reset(seed=args.seed)
    fw, nc = act.spec.frame_dim, act.spec.num_cmd

    obs_writer = None
    if args.obs_csv:
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import obs_csv                                       # noqa: E402
        spec = json.loads(lib.spec_string())
        obs_writer = obs_csv.Writer(args.obs_csv, spec["joint_names"],
                                    spec["frame_offsets"], fw, nc)
        print(f"sil_twin: obs capture -> {args.obs_csv} "
              f"(firmware obsdump format)")
    foot_writer = None
    if args.foot_csv:
        # swing clearance + foot separation per tick (2026-09-08, Tom: "keeping
        # the feet from binding together and raising the foot higher") --
        # sole heights above the standing sole height and the torso-frame
        # lateral separation of the two sole centres (env_mjx y_sep).
        foot_writer = open(args.foot_csv, "w")
        foot_writer.write("tick,t,z_l,z_r,y_sep,cmd_vx\n")
        print(f"sil_twin: foot metrics -> {args.foot_csv}")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", args.port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Mirror mode (docs/mirror-mode.md): beacon our own joint angles to a
    # commander that asks with FLAG_POSE, and draw a pose it pushes back at us
    # as a ghost over our own. The console drives the real robot; this is the
    # sim running the same command beside it, so divergence is a picture.
    import mujoco                                            # noqa: E402
    want_pose = False
    ghost_q = None
    ghost_up = None
    ghost_at = 0.0
    # The wire's joint block is the robot's BUS, servo-ID order
    # (protocol.JOINT_NAMES, 17 wide). Both directions map it onto this
    # plant BY NAME: the ghost poses the plant joints it has (the prototype
    # plant has no ankle rolls, neck or arms), and our own beacon reports the
    # policy's joints and 0.0 for the rest -- what the 10-servo prototype's
    # firmware sends for servos it does not carry.
    ghost_adr = []                       # (wire index, qpos address)
    for k, jn in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            ghost_adr.append((k, int(env.model.jnt_qposadr[jid])))
    if not ghost_adr:
        print("sil_twin: no wire joint is in this plant -- ghost disabled",
              flush=True)
    _pol_names = json.loads(lib.spec_string())["joint_names"]
    beacon_src = [(_pol_names.index(jn) if jn in _pol_names else -1)
                  for jn in JOINT_NAMES]     # wire index -> policy joint
    # The torso free joint, for the attitude half of a pose. qpos layout is
    # [x y z qw qx qy qz] at its address; -1 when the plant is welded (--hang),
    # where the torso has no pose to set and the joints are the whole picture.
    _root = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_adr = int(env.model.jnt_qposadr[_root]) if _root >= 0 else -1

    # Flight records (--flight): the robot's recorder stream, from the SIL
    # controller's targets and the plant's joints (sim/sil_twin_wire.py).
    flight = None
    spec = lib.spec
    plant_jnt = {}                       # bus joint name -> (qpos adr, dof adr)
    imu_sid = -1
    if args.flight:
        if tuple(H.BUS_NAMES) != tuple(FR.BUS_JOINT_NAMES):
            raise SystemExit("sil_twin: obs/bus_map.h and tools/flightrec.py "
                             "disagree on the bus joint order")
        for k, jn in enumerate(spec.joint_names):
            b = W.BUS_INDEX[jn]
            if H.BUS_IDS[b] != int(spec.servo_id[k]):
                raise SystemExit(f"sil_twin: {jn} is servo {spec.servo_id[k]} in "
                                 f"obs_spec.h, but bus slot {b} is servo "
                                 f"{H.BUS_IDS[b]} in bus_map.h")
        for jn in FR.BUS_JOINT_NAMES:
            jid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                plant_jnt[jn] = (int(env.model.jnt_qposadr[jid]),
                                 int(env.model.jnt_dofadr[jid]))
        flight_cal = W.bus_calibration(
            plant_jnt, H.BUS_IDS,
            {jn: (int(lib.cal.zero_steps[k]), int(lib.cal.dir[k]))
             for k, jn in enumerate(spec.joint_names)})
        try:
            flight_send = W.UdpSend(*args.flight)
        except OSError as e:
            raise SystemExit(f"sil_twin: --flight {args.flight[0]}:"
                             f"{args.flight[1]}: cannot resolve ({e})")
        flight = W.TwinFlight(f"sil_twin {args.run_name}", flight_cal,
                              send=flight_send,
                              idle_hz=int(round(1.0 / env.control_dt)))
        imu_sid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        print(f"sil_twin: flight records -> {flight_send.addr[0]}:"
              f"{flight_send.addr[1]} (boot {flight.boot_id:08x}, "
              f"{sum(flight_cal.fitted)} joints"
              f"{'' if imu_sid >= 0 else ', no imu site: IMU block not valid'})",
              flush=True)

    def up_to_quat(up):
        """Torso orientation from the robot's up vector, as a wxyz quaternion.

        `up` is the torso's own z axis EXPRESSED IN THE WORLD FRAME -- MuJoCo
        framezaxis, and the same convention the obs frame uses
        (obs/obs_spec.h kOffUp, filled from imu::Sample::up). So the rotation
        wanted is the one taking world +Z onto `up`: the shortest such
        rotation, because that is all the information there is.

        It fixes roll and pitch and leaves yaw at zero, which is honest --
        this robot has no magnetometer, and the filter's heading is
        dead-reckoned and drifts. Drawing a mannequin with an invented heading
        would make a drifting number look like a measurement.
        """
        v = np.asarray(up, dtype=np.float64)
        n = float(np.linalg.norm(v))
        if not np.isfinite(n) or n < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])
        v = v / n
        z = np.array([0.0, 0.0, 1.0])
        axis = np.cross(z, v)                 # world +Z -> up, not the reverse
        sn = float(np.linalg.norm(axis))
        c = float(np.dot(z, v))
        if sn < 1e-9:
            # Parallel or antiparallel: upright, or flat on its back.
            return (np.array([1.0, 0.0, 0.0, 0.0]) if c > 0
                    else np.array([0.0, 1.0, 0.0, 0.0]))
        axis = axis / sn
        ang = float(np.arctan2(sn, c))
        return np.concatenate(([np.cos(ang / 2)], np.sin(ang / 2) * axis))

    dog = Supervisor(armed=args.boot_armed)
    stream = Mjpeg(args.stream_port) if args.stream_port else None
    frames = []
    peer = None
    want_att = False
    was_armed = args.boot_armed
    last_state = None
    flight_mode = None                   # last EV_MODE value recorded
    worst_us = 0
    t0 = time.monotonic()
    late = 0
    dt = env.control_dt

    def boot_us():
        return int((time.monotonic() - t0) * 1e6)

    def utc_us():
        return time.time_ns() // 1000

    print(f"sil_twin: {args.run_name} on :{args.port} (tlm -> :{args.tlm_port})"
          f"  plant={os.path.basename(xml) if xml else 'run default'}"
          f"  spec={info['spec'][:60]}...")

    stdin_buf = [""]

    def _stdin_commands():
        """Line commands from whoever launched us (bimo_gui's SIM panel).

        `reset` re-rolls the episode in place; `pose q0..q9` feeds the ghost.
        Restarting the process instead costs ~30 s of MuJoCo and policy
        loading, which is long enough that nobody does it, which means nobody
        resets.

        Reads the raw fd rather than sys.stdin.readline(): select() reports the
        KERNEL buffer, but readline() pulls a whole chunk into Python's
        TextIOWrapper, so a burst of two commands leaves the second stranded in
        userspace with select() saying "not ready". At 10 Hz of pose lines that
        is not hypothetical. (Percy, PR #53.)"""
        out = []
        while select.select([0], [], [], 0)[0]:
            try:
                chunk = os.read(0, 65536)
            except OSError:
                break
            if not chunk:                      # EOF: the console went away
                break
            stdin_buf[0] += chunk.decode("utf-8", "replace")
        while "\n" in stdin_buf[0]:
            line, stdin_buf[0] = stdin_buf[0].split("\n", 1)
            if line.strip():
                out.append(line.strip())
        return out

    i = 0
    try:
        while args.duration is None or i * dt < args.duration:
            tick = time.monotonic()
            now_ms = (tick - t0) * 1000.0
            if flight is not None:
                # The TICK's header stamps are the tick start (ctrl's t0);
                # HELLOs and events carry the instant they are posted
                # (flight_rec.cpp pushEvent / buildHello).
                tick_boot_us, tick_utc_us = int((tick - t0) * 1e6), utc_us()
                if flight.hello_due(tick_boot_us):
                    flight.post(flight.hello(boot_us(), utc_us(), ticks=i,
                                             late=late, worst_us=worst_us))

            for cmd in _stdin_commands():
                if cmd.startswith("pose "):
                    # "pose q1 .. q17 [ux uy uz]": the bus joint angles the
                    # console just read off the real robot, radians in
                    # servo-ID order (protocol.JOINT_NAMES), optionally
                    # followed by the torso up vector from its IMU. The up vector is what turns a set
                    # of joint angles into an ATTITUDE -- without it a robot
                    # lying on its face draws standing to attention.
                    try:
                        vals = [float(v) for v in cmd.split()[1:]]
                    except ValueError:
                        vals = []
                    if len(vals) in (NUM_JOINTS, NUM_JOINTS + 3):
                        if ghost_q is None:
                            # Say it once: a ghost that silently never appears
                            # is indistinguishable from one drawn at zero.
                            print(f"  [{now_ms / 1000:6.2f}s] pose feed live"
                                  f"{'' if len(vals) > NUM_JOINTS else ' (no '
                                  'up vector: torso drawn upright)'}",
                                  flush=True)
                        ghost_q = vals[:NUM_JOINTS]
                        ghost_up = (tuple(vals[NUM_JOINTS:])
                                    if len(vals) > NUM_JOINTS else None)
                        ghost_at = time.monotonic()
                    else:
                        print(f"  pose wants {NUM_JOINTS} angles "
                              f"(+3 optional up), got {len(vals)}", flush=True)
                elif cmd == "reset":
                    obs, _ = env.reset(seed=args.seed)
                    act.begin_episode()
                    print(f"  [{now_ms / 1000:6.2f}s] episode reset",
                          flush=True)
                elif cmd:
                    print(f"  unknown command: {cmd}", flush=True)

            while True:                       # drain; last valid packet wins
                try:
                    buf, addr = rx.recvfrom(64)
                except BlockingIOError:
                    break
                peer = addr[0]
                try:
                    pkt = decode_command(buf)
                except ProtocolError:
                    continue
                arm_level, homes, armed_before = (dog.latch.level, dog.homes,
                                                  dog.armed)
                dog.accept(pkt, now_ms)
                # Like the firmware's Commander: both levels come from the
                # latest valid frame, replaced together.
                want_pose = pkt.pose
                want_att = pkt.att
                if flight is not None:
                    tb, tu = boot_us(), utc_us()
                    if pkt.stop:            # out of band: no latch saw it
                        flight.post(flight.stop(tb, tu, pkt.estop, armed_before))
                    elif arm_level is not None and dog.latch.level != arm_level:
                        flight.post(flight.event(tb, tu, FR.EV_ARM_EDGE,
                                                 a=int(dog.latch.level)))
                    if dog.homes != homes:
                        flight.post(flight.event(tb, tu, FR.EV_HOME_EDGE))

            if args.viewer:
                # VISUALISE ONLY. No policy, no env.step, no integration, no
                # beacon -- the plant is a mannequin held in whatever pose the
                # robot last reported. Anything else would be a SECOND robot
                # drawn beside the real one and diverging from it in silence,
                # which is the thing this mode exists not to do.
                if (stream or args.record) and i % 3 == 0:
                    fresh = ghost_q is not None and \
                        (time.monotonic() - ghost_at) < 1.0
                    if fresh and ghost_adr:
                        for k, adr in ghost_adr:
                            env.data.qpos[adr] = ghost_q[k]
                        if root_adr >= 0:
                            # Attitude from the IMU; position PINNED. There is
                            # no odometry on this robot, so a world position
                            # would be invented -- and a mannequin that drifts
                            # is worse than one that stands still and is
                            # honest about only knowing which way is up.
                            env.data.qpos[root_adr + 3:root_adr + 7] = (
                                up_to_quat(ghost_up) if ghost_up is not None
                                else np.array([1.0, 0.0, 0.0, 0.0]))
                        # Kinematics only: mj_forward places the bodies from
                        # qpos. mj_step would integrate, which is the whole
                        # thing we are not doing.
                        mujoco.mj_forward(env.model, env.data)
                    if stream:
                        stream.push(env.render())
                    if args.record:
                        frames.append(env.render())
                i += 1
                slack = dt - (time.monotonic() - tick)
                if slack > 0:
                    time.sleep(slack)
                continue

            state = dog.state(now_ms)
            if state is not last_state:
                print(f"  [{now_ms / 1000:6.2f}s] link -> {state.value}",
                      flush=True)
                last_state = state

            # Firmware run-entry: fresh history, fresh clock (ctrl_task's
            # bench handover). Mirror it on the arm edge.
            if dog.armed and not was_armed:
                act.begin_episode()
            was_armed = dog.armed
            if flight is not None:
                # EV_MODE as flight_rec.cpp houseTick: a = run | ctrl owns
                # the bus << 1 (both, armed), b = the ArmResult.
                mode = 3 if dog.armed else 0
                if mode != flight_mode:
                    flight_mode = mode
                    flight.post(flight.event(
                        boot_us(), utc_us(), FR.EV_MODE, a=mode,
                        b=dog.result.value, text="run" if mode & 1 else "bench"))

            # Full 7-wide ext_cmd off the link (vx, vy, wz, crouch, lift,
            # foot_dx, foot_dz) -- extended frames drive the foot-target
            # channels, classic frames decode to the trained defaults.
            env.set_command(*dog.command_ext(now_ms))
            env.set_torque_enabled(dog.torque_on(now_ms))

            # Driver.step's obs contract: current cmd patched into the frame
            # the firmware stack is fed.
            obs = np.asarray(obs, dtype=np.float64).copy()
            obs[fw - nc:fw] = env._cmd
            a = act(obs)
            if obs_writer is not None and dog.torque_on(now_ms):
                # Mirror the firmware record: raw sensor fields, then the
                # frame the stack was fed. Raw phase is recovered from the
                # frame's sin/cos (the sim lib does not expose the scalar).
                fields = act.split_obs(obs)
                o = json.loads(lib.spec_string())["frame_offsets"]
                phase = float(np.arctan2(obs[o["phase"]], obs[o["phase"] + 1]))
                row = (list(fields["q"]) + list(fields["dq"]) +
                       list(fields["up"]) + list(fields["gyro"]) + [phase] +
                       list(fields["cmd"]) + list(obs[:fw]))
                obs_writer.row(i, row)
            if foot_writer is not None:
                d = env.data
                pL = d.geom_xpos[env._sole_gids[0]] - d.xpos[env._torso_bid]
                pR = d.geom_xpos[env._sole_gids[1]] - d.xpos[env._torso_bid]
                # torso yaw from the body x-axis (same lateral axis as env_mjx)
                xm = d.xmat[env._torso_bid].reshape(3, 3)
                yaw = float(np.arctan2(xm[1, 0], xm[0, 0]))
                sth, cth = np.sin(yaw), np.cos(yaw)
                y_sep = abs((-sth * pL[0] + cth * pL[1]) - (-sth * pR[0] + cth * pR[1]))
                foot_writer.write(f"{i},{i * env.control_dt:.3f},"
                                  f"{d.geom_xpos[env._sole_gids[0]][2] - env._sole_z0:.4f},"
                                  f"{d.geom_xpos[env._sole_gids[1]][2] - env._sole_z0:.4f},"
                                  f"{y_sep:.4f},{float(env._cmd[0]):.2f}\n")
            if flight is not None:
                # What the tick STARTED from -- the robot's record carries the
                # positions its SYNC READ fed the policy, not the ones after
                # the goals went out. Policy joints: the obs the SIL stack
                # was fed (its servo-side view); any other plant joint: qpos.
                torque_now = dog.torque_on(now_ms)
                f_in = act.split_obs(obs)
                measured = {jn: (float(env.data.qpos[qa]), float(env.data.qvel[da]))
                            for jn, (qa, da) in plant_jnt.items()}
                for k, jn in enumerate(spec.joint_names):
                    measured[jn] = (float(f_in["q"][k]), float(f_in["dq"][k]))
                imu_truth = None
                if imu_sid >= 0:
                    # The IMU site's true attitude and specific force (m/s^2,
                    # site = body frame, gravity included: MuJoCo's
                    # accelerometer). rnePostConstraint only fills cacc /
                    # cfrc_*, which nothing here reads and mj_step recomputes.
                    qw = np.zeros(4)
                    mujoco.mju_mat2Quat(qw, env.data.site_xmat[imu_sid])
                    mujoco.mj_rnePostConstraint(env.model, env.data)
                    acc6 = np.zeros(6)
                    mujoco.mj_objectAcceleration(env.model, env.data,
                                                 mujoco.mjtObj.mjOBJ_SITE,
                                                 imu_sid, acc6, 1)
                    imu_truth = (qw, acc6[3:].copy())
            obs, _, _, _, step_info = env.step(a)
            # the beacon's up vector: the same sample as step_info["up_z"]
            up_now = env.data.sensordata[env._up_id:env._up_id + 3].copy()

            if (stream or args.record) and i % 3 == 0:      # ~16 fps
                rgb = env.render()
                # The ghost: the SAME model and camera re-posed to the angles
                # the real robot reported, composited over our own. Rendering
                # a second model would risk a second viewpoint, and a ghost
                # drawn from a different camera is worse than none.
                fresh = ghost_q is not None and \
                    (time.monotonic() - ghost_at) < 1.0
                if fresh and ghost_adr:
                    saved = env.data.qpos.copy()
                    for k, adr in ghost_adr:
                        env.data.qpos[adr] = ghost_q[k]
                    mujoco.mj_forward(env.model, env.data)
                    ghost_rgb = env.render()
                    env.data.qpos[:] = saved
                    mujoco.mj_forward(env.model, env.data)
                    rgb = (0.60 * rgb.astype(np.float32) +
                           0.40 * ghost_rgb.astype(np.float32)
                           ).clip(0, 255).astype(np.uint8)
                if stream:
                    stream.push(rgb)
                if args.record:
                    frames.append(rgb)

            if peer and i % 5 == 0:                          # 10 Hz beacon
                tx.sendto(W.beacon(
                    seq_echo=dog.seq_echo(now_ms), state=state,
                    vbat_v=float(env.supply_voltage), up=up_now,
                    vx_est=float(step_info.get("vx_body", 0.0)),
                    wz_est=float(step_info.get("wz", 0.0)),
                    loop_late_pct=int(100 * late / max(1, i)),
                    t_us=int(time.time() * 1e6),   # the host clock IS ours
                    # Only for a commander that asked. obs frame_offsets put
                    # the policy's joint angles at the front of the frame;
                    # the wire wants every bus joint in servo-ID order.
                    joints=(tuple(float(obs[j]) if j >= 0 else 0.0
                                  for j in beacon_src)
                            if want_pose else ()),
                    att=want_att,
                ), (peer, args.tlm_port))

            if flight is not None:
                # SilActAdapter keeps the tick's SilTargets (no skip ticks here).
                view = W.sil_view(H, lib, act._last_targets)
                # The controller's targets are reported while the loop is
                # armed or the plant tracks them (a homed hold); goals only
                # while it does (torque on).
                reported = dog.armed or torque_now
                imu = (dict(accel=imu_truth[1], gyro=view["gyro"],
                            quat=imu_truth[0], up=view["up"])
                       if imu_truth is not None else None)
                work_us = int((time.monotonic() - tick) * 1e6)
                worst_us = max(worst_us, work_us)
                code = W.link_state_code(state)
                flight.post(flight.tick(tick_boot_us, tick_utc_us, W.tick_body(
                    tick=i, armed=dog.armed, torque=torque_now, link_state=code,
                    work_us=work_us,
                    joints=W.joint_block(flight_cal, measured,
                                         view["targets"] if reported else None,
                                         written=torque_now),
                    imu=imu, view=view if reported else None,
                    vbat_mv=int(round(env.supply_voltage * 1000)))))
                if dog.armed:               # ctrl's link-state events
                    flight.post(flight.link_change(boot_us(), utc_us(), code))

            i += 1
            slack = dt - (time.monotonic() - tick)
            if slack > 0:
                time.sleep(slack)
            else:
                late += 1
    except KeyboardInterrupt:
        print("\nsil_twin: stopped")
    finally:
        if flight is not None:
            flight_send.close()
            print(f"sil_twin: flight records: {flight.sent} sent, "
                  f"{flight.drop_net} failed sends (boot {flight.boot_id:08x})")
        if obs_writer is not None:
            obs_writer.close()
            print(f"sil_twin: obs capture closed ({obs_writer.n} records)")
        if args.record and frames:
            import imageio.v2 as imageio
            imageio.mimwrite(args.record, frames, fps=int(round(1 / dt / 3)))
            print(f"sil_twin: wrote {args.record} ({len(frames)} frames)")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-name", default=DEFAULT_RUN)
    p.add_argument("--hang", action="store_true",
                   help="welded-torso plant (test-stand analogue)")
    p.add_argument("--xml", default=None, help="explicit plant override")
    p.add_argument("--nominal", action="store_true", default=True,
                   help="clean plant (no DR/latency): bench-comparable")
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--stream-port", type=int, default=8645,
                   help="MJPEG live view port (0 disables)")
    p.add_argument("--record", default=None, help="write an mp4 on exit")
    p.add_argument("--foot-csv", default=None,
                   help="per-tick swing clearance (sole heights) + torso-frame "
                        "lateral foot separation CSV")
    p.add_argument("--obs-csv", default=None,
                   help="write the fed observations in firmware obsdump "
                        "format (tools/obs_capture.py --replay/--compare)")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--backlash-deg", type=float, default=None,
                   help="pin gear backlash deadzone (deg) on the plant")
    p.add_argument("--play-deg", type=float, default=0.0,
                   help="free travel (mechanical hysteresis) between servo "
                        "shaft and link, peak-to-peak deg")
    p.add_argument("--play-joints", default=None,
                   help="comma list of joint names the play applies to "
                        "(default all), e.g. L_hip_roll,R_hip_roll")
    p.add_argument("--plant", nargs="*", metavar="KEY=VAL",
                   help="pin BimoWalkerEnv plant kwargs on the twin, e.g. "
                        "servo_kp=8 supply_voltage=6.5 payload_mass=0.15 "
                        "friction_range=0 (numbers parse as float)")
    p.add_argument("--act-delay-ticks", type=int, default=0,
                   help="servo dead time in 20 ms control ticks (4 = the 09-05 bench ~85 ms)")
    p.add_argument("--act-lag-hz", type=float, default=0.0,
                   help="measured-servo lag pole (2.0 = the 08-31 bench)")
    p.add_argument("--boot-armed", action="store_true")
    p.add_argument("--viewer", action="store_true",
                   help="VISUALISE ONLY: no policy, no physics, no beacon. "
                        "The plant is posed from `pose` lines on stdin and "
                        "rendered. This is what mirror mode wants -- a picture "
                        "of the real robot, not a second robot beside it.")
    p.add_argument("--flight", default=None, type=flight_dest,
                   metavar="HOST:PORT",
                   help="send flight records here (the robot's recorder "
                        "stream, tools/flightrec.py: HELLO 1 Hz, a TICK per "
                        "control tick, events) -- a console, or mira's "
                        "collector on 4220. Off by default")
    args = p.parse_args()
    if args.flight and args.viewer:
        p.error("--flight needs the twin's control loop; --viewer runs none")
    run(args)


def flight_dest(s):
    try:
        return FR.parse_hostport(s)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


if __name__ == "__main__":
    main()
