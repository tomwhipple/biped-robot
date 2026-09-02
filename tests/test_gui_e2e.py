"""End-to-end: the real bimo_gui binary against the plant-free link twin over
real UDP.

Run:  make -C firmware/host gui && .venv/bin/python -m pytest tests/test_gui_e2e.py -q

The window is not what is tested here, and does not need to be: bimo_gui
--headless runs the IDENTICAL link loop (link/client.h's Link + Intent, the
same 20 Hz send, the same arm-sync rule) with stdin instead of a keyboard, so
everything below exercises the real client and needs no display. It is also
what makes this runnable in CI at all.

The oracle is link_twin.py's stdout -- what the ROBOT decided it was told --
exactly as tests/test_tui_e2e.py does it. What can be asserted here and not
there:

  * a RELEASE stands on the very next frame. bimo_tui has no key-up event and
    fakes a dead-man with --hold-ms; this one is exact.
  * a slider reaches the wire as a 24 B extended frame with the crouch the
    operator asked for -- a channel no keyboard console can command.

Ports are offset so it never collides with a console pointed at the robot.
"""
import itertools
import os
import re
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUI = os.path.join(ROOT, "firmware", "host", "build", "bimo_gui")
TWIN = os.path.join(ROOT, "link", "link_twin.py")

# A PAIR PER TEST, not one pair for the file. The twins bind with
# SO_REUSEADDR, so a previous test's twin that has not finished exiting will
# happily bind alongside the next one and split the packet stream between
# them -- which fails as "the robot ignored my command", nowhere near the
# cause. Only ever seen on a loaded CI runner, which is exactly the kind of
# flake that is impossible to find locally.
_next_port = itertools.count(4312, 4)

pytestmark = pytest.mark.skipif(not os.path.exists(GUI),
                                reason="build it: make -C firmware/host gui")


def _ports():
    base = next(_next_port)
    return base, base + 1


def _twin(cmd_port, tlm_port, extra=()):
    return subprocess.Popen(
        [sys.executable, TWIN, "--port", str(cmd_port), "--tlm-port",
         str(tlm_port), "--duration", "20", *extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def _gui(script, cmd_port, tlm_port, extra=()):
    """Feed `script` -- a list of (delay_s, line) -- to a headless console.

    Written from a thread-free generator into the child's stdin: the delays
    ARE the test (a press held for 0.4 s, then released), so they cannot be
    batched up front.
    """
    proc = subprocess.Popen(
        [GUI, "--host", "127.0.0.1", "--cmd-port", str(cmd_port),
         "--tlm-port", str(tlm_port), "--headless", *extra],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1)
    for delay, line in script:
        time.sleep(delay)
        if proc.poll() is not None:
            break
        proc.stdin.write(line + "\n")
        proc.stdin.flush()
    return proc


def _states(log):
    """(state, vx, wz, torque) per change, from the twin's own trace."""
    import re
    return re.findall(r"\]\s+(\w+)\s+vx ([+-]\d\.\d\d) wz ([+-]\d\.\d\d)"
                      r"(?: [^\n]*?)? torque (on|off)", log)


def _ext_lines(log):
    """The twin only prints the extended channels when they are off default."""
    return [ln for ln in log.splitlines() if "crouch" in ln]


def _run(script, twin_args=(), gui_args=()):
    cmd_port, tlm_port = _ports()
    twin = _twin(cmd_port, tlm_port, twin_args)
    time.sleep(0.5)
    gui = _gui(script, cmd_port, tlm_port, gui_args)
    try:
        out = gui.communicate(timeout=15)[0]
    except subprocess.TimeoutExpired:
        gui.kill()
        out = gui.communicate()[0]
    finally:
        twin.terminate()
        log = twin.communicate(timeout=5)[0]
    return out, log, gui.returncode


def test_arm_press_release_estop_quit():
    # The ARM level must be seen LOW before it can be seen going high:
    # ArmLatch deliberately ignores a sender's first frame, so that a robot
    # rebooting under a console still holding ARM=1 stays benched.
    out, log, rc = _run([
        (1.0, "arm"),
        (1.2, "press forward"),
        (0.6, "release forward"),
        (0.5, "press turn_left"),
        (0.5, "release turn_left"),
        (0.4, "estop"),
        (0.5, "stand"),
        (0.5, "quit"),
        (0.5, ""),
    ])

    seq = _states(log)
    kinds = [s[0] for s in seq]
    assert kinds and kinds[0] == "bench", log
    # A console streaming at 20 Hz never lets the link go stale, so a standing
    # robot is LIVE with a zero command, never STAND.
    assert "stand" not in kinds and "live" in kinds, log
    assert ("live", "+0.40", "+0.00", "on") in seq, log

    # The dead-man: the frame after the release is a stand. Not "within
    # --hold-ms" -- the very next one.
    i = seq.index(("live", "+0.40", "+0.00", "on"))
    assert ("live", "+0.00", "+0.00", "on") in seq[i:], log
    assert ("live", "+0.00", "+0.50", "on") in seq, log

    assert any(s[0] == "estop" and s[3] == "off" for s in seq), log
    j = max(k for k, s in enumerate(seq) if s[0] == "estop")
    assert any(s[0] == "live" for s in seq[j:]), log    # stand cleared it
    assert kinds[-1] == "bench" and seq[-1][3] == "off", log   # quit disarmed
    assert rc == 0, out

    assert "ARMED" in out and "LIVE" in out and "FORWARD" in out, out
    assert "11.40 V" in out, out                        # telemetry decoded


def test_a_slider_reaches_the_wire_as_an_extended_frame():
    """crouch is a channel no keyboard console can command at all."""
    out, log, rc = _run([
        (1.0, "arm"),
        (1.2, "set crouch 0.80"),
        (0.8, "press strafe_left"),
        (0.6, "release strafe_left"),
        (0.4, "reset_ext"),
        (0.6, "quit"),
        (0.5, ""),
    ])

    ext = _ext_lines(log)
    assert ext, "the twin never saw an extended frame\n" + log
    assert any("crouch 0.80" in ln for ln in ext), log
    # Strafe is vy, which only exists in the 24 B frame.
    assert any("vy +0.20" in ln for ln in ext), log
    # ... and resetting goes back to the short frame, so the extended trace
    # stops entirely rather than pinning the robot at the last crouch.
    assert "crouch" not in log.splitlines()[-1], log

    assert "len 24" in out, out
    assert "len 14" in out, out
    assert rc == 0, out


def test_console_follows_a_robot_that_refuses_to_arm():
    out, log, rc = _run([
        (0.8, "arm"),
        (2.5, "press forward"),      # > kArmSyncMs: the console has given up
        (0.5, "quit"),
        (0.5, ""),
    ], twin_args=["--no-cal"])

    assert all(s[0] == "bench" for s in _states(log)), log
    assert "stayed BENCH" in out, out
    assert "not armed -- press [a] first" in out, out
    # The incident of 2026-08-30: the robot refused the arm and said nothing
    # over the radio. The reason must ride the beacon and be quoted, not
    # guessed at.
    assert "no as-built calibration in NVS" in out, out
    # ... and it must say so from the very first beacon, before an arm is even
    # attempted: an uncalibrated robot is diagnosable on sight.
    assert "arming will be REFUSED" in out, out
    assert rc == 0, out


def test_closing_stdin_disarms():
    """A console that goes away must leave a limp robot, not a standing one.

    Killing the operator (here: EOF on stdin) is the case that matters -- the
    quit path is the one an operator takes deliberately, and this is the one
    they do not.
    """
    cmd_port, tlm_port = _ports()
    twin = _twin(cmd_port, tlm_port)
    time.sleep(0.5)
    gui = _gui([(1.0, "arm"), (1.2, "press forward"), (0.5, "")],
               cmd_port, tlm_port)
    try:
        gui.stdin.close()                 # EOF: the console should leave
        # wait() + read(), not communicate(): communicate() closes stdin
        # itself, and closing it twice raises "I/O operation on closed file"
        # -- which reads as a console bug and is not one.
        gui.wait(timeout=10)
        out = gui.stdout.read()
    finally:
        twin.terminate()
        log = twin.communicate(timeout=5)[0]

    seq = _states(log)
    assert any(s[0] == "live" for s in seq), log
    assert seq[-1][0] == "bench" and seq[-1][3] == "off", log
    assert gui.returncode == 0, out


def _pose_lines(log):
    return [ln for ln in log.splitlines() if "pose " in ln]


def test_mirror_mode_asks_for_joint_angles_and_stops_asking():
    """The whole point of FLAG_POSE being a request rather than a default.

    A robot must beacon the classic 20 B frame to everyone until a commander
    explicitly asks -- otherwise every existing client goes blind at once (see
    docs/mirror-mode.md). So: silent by default, long frames while mirroring,
    classic again the moment the console stops asking.
    """
    out, log, rc = _run([
        (1.0, "sim mirror"),
        (2.0, "sim mirror off"),
        (1.0, "quit"),
        (0.5, ""),
    ], gui_args=["--no-record"])

    poses = _pose_lines(log)
    assert poses, "the twin never saw a pose request\n" + log
    # Before anything is asked for, and after it stops being asked for, the
    # beacon is classic.
    assert "off" in poses[0], log
    assert any("requested" in ln for ln in poses), log
    assert "off" in poses[-1], log
    assert "MIRROR" in out, out
    assert rc == 0, out


def test_mirror_flag_brings_up_the_sim_it_mirrors_against(tmp_path):
    """`--mirror` must reach the SPAWN path, not just flip the mode.

    Mirroring with no sim is the console asking the robot for joint angles and
    drawing them nowhere -- the pose relay in tick() is gated on a running
    child. So the flag boots the sim, and the proof is that it gets as far as
    picking a run: pointed at an empty --repo there is nothing to run, and it
    says so out loud instead of coming up silently mirror-on and blank.
    """
    out, log, rc = _run([(1.5, "quit"), (0.5, "")],
                        gui_args=["--no-record", "--mirror",
                                  "--repo", str(tmp_path)])
    assert "MIRROR" in out, out
    assert "no runnable sim runs" in out, out
    # ... and it really did ask the robot for angles, empty repo or not.
    assert any("requested" in ln for ln in _pose_lines(log)), log
    assert rc == 0, out


def test_mirror_is_off_unless_asked_for():
    """A console that never mentions mirroring must never request pose."""
    out, log, rc = _run([
        (1.0, "arm"),
        (1.5, "press forward"),
        (0.5, "quit"),
        (0.5, ""),
    ], gui_args=["--no-record"])
    assert not any("requested" in ln for ln in _pose_lines(log)), log
    assert rc == 0, out


def _home_lines(log):
    return [ln for ln in log.splitlines() if "home:" in ln]


def test_reset_servos_works_with_the_estop_latched():
    """The one button that must survive the state everything else refuses in.

    An E-stopped robot is limp by design, and that is the state an operator is
    in when it is on the floor: ARM is refused, every motion key is refused,
    and the console is correctly sending nothing but the latch. `home` is the
    way out -- it needs no arm, it reaches the robot through the E-stop, and
    the robot benches and holds the standing pose rather than staying limp.
    """
    out, log, rc = _run([
        (1.0, "arm"),
        (1.2, "press forward"),
        (0.5, "estop"),
        (0.5, "home"),
        (0.8, "quit"),
        (0.5, ""),
    ], gui_args=["--no-record"])

    seq = _states(log)
    kinds = [s[0] for s in seq]
    assert "estop" in kinds, log
    assert _home_lines(log), "the robot never saw the request\n" + log

    # After the reset: benched (the run is over) but torque ON -- the one
    # combination nothing else on this wire produces, and the whole point.
    i = max(k for k, s in enumerate(seq) if s[0] == "estop")
    held = [s for s in seq[i:] if s[0] == "bench" and s[3] == "on"]
    assert held, log
    assert "servos reset to the standing pose" in log, log

    # It is a request, not a command: the robot never leaves the bench on the
    # strength of it -- re-arming stays a deliberate, separate act.
    assert not any(s[0] == "live" for s in seq[i:]), log
    # And the console says what it sent. (The HOME level itself lasts
    # kHomeFrames = 400 ms, which is deliberately shorter than the headless
    # status interval, so the flag string is not what to assert on here.)
    assert "reset servos" in out, out
    assert rc == 0, out


def test_reset_servos_needs_no_arm_and_is_one_edge():
    """From the boot state, and exactly once however long the level is held.

    The robot acts on the RISING edge (HomeLatch), so a level held for
    kHomeFrames must still home once -- not eight times at 20 Hz.
    """
    out, log, rc = _run([
        (1.5, "home"),
        (1.0, "home"),
        (0.8, "quit"),
        (0.5, ""),
    ], gui_args=["--no-record"])

    assert len(_home_lines(log)) == 2, log     # two presses, two homes
    assert "ARM" not in out, out               # it never armed anything
    assert rc == 0, out


def test_a_robot_that_never_heard_of_the_reset_is_called_out():
    """The failure that actually happened the day the button shipped.

    A console rebuilt with RESET SERVOS, pointed at firmware that predates
    bit 4: the robot ignores it and reports the disarm that rode in with the
    request, so every field on screen reads plausibly and the robot just sits
    there. The console has to notice the difference between "answered" and
    "answered about something else" -- otherwise the operator goes looking
    for a dead radio or a seized servo.
    """
    out, log, rc = _run([
        (1.5, "home"),
        (2.5, ""),
        (0.5, "quit"),
        (0.5, ""),
    ], twin_args=["--deaf-home"], gui_args=["--no-record"])

    assert not _home_lines(log), log            # the robot never homed
    assert "no answer to the reset" in out, out
    assert "re-flash" in out, out
    assert rc == 0, out


# -- the observer -----------------------------------------------------------
# `--readonly` exists so a second pair of eyes can watch a robot somebody else
# is driving -- another session on another machine -- without the watching
# console being able to touch it. Two claims, both tested end to end over real
# UDP here rather than only as unit tests, because the interesting part is the
# TOPOLOGY: the robot beacons to whoever commanded last and to nobody else
# (link_twin.py holds the same rule the firmware does, `peer`), so an observer
# is only reachable because the driver forwards.


def _observer(tlm_port, extra=()):
    """A headless watcher. No --host: it has nowhere to send, by design."""
    return subprocess.Popen(
        [GUI, "--headless", "--readonly", "--tlm-port", str(tlm_port),
         "--no-record", *extra],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1)


def test_readonly_console_cannot_command_the_robot():
    """Point a watcher straight at the twin and try every control on it.

    This is the adversarial case, not the intended wiring: the observer is
    given the robot's own --host and --cmd-port, so the ONLY thing standing
    between `arm` and a moving robot is the mode. The twin's trace is the
    oracle -- it prints a line per command it would hand the policy, so an
    empty trace is proof that nothing arrived, not merely that nothing
    obviously happened.
    """
    cmd_port, tlm_port = _ports()
    twin = _twin(cmd_port, tlm_port)
    time.sleep(0.5)
    obs = _observer(tlm_port, ["--host", "127.0.0.1",
                               "--cmd-port", str(cmd_port)])
    for delay, line in [(0.8, "arm"), (0.6, "press forward"),
                        (0.5, "estop"), (0.5, "home"), (0.5, "quit"),
                        (0.5, "")]:
        time.sleep(delay)
        if obs.poll() is not None:
            break
        obs.stdin.write(line + "\n")
        obs.stdin.flush()
    try:
        out = obs.communicate(timeout=15)[0]
    except subprocess.TimeoutExpired:
        obs.kill()
        out = obs.communicate()[0]
    finally:
        twin.terminate()
        log = twin.communicate(timeout=5)[0]

    # The robot heard nothing at all. Two independent proofs:
    #  * its trace never moves off the line it prints at boot, so no command
    #    was ever handed to the policy; and
    #  * the observer got no telemetry -- which is the CAUSAL one, because the
    #    twin beacons only to a peer, and a peer is set by receiving a
    #    command. Telemetry arriving here would have meant a frame got out.
    assert [st for st, _, _, _ in _states(log)] == ["bench"], log
    assert "robot (no telemetry)" in out, out
    assert "OBSERVER" in out, out
    # And it said so for each refusal rather than going quiet, which is the
    # difference between a mode and a bug.
    assert out.count("OBSERVER: this console only watches") >= 1, out
    assert "ARMED" not in out, out
    assert obs.returncode == 0, out


def test_the_driver_forwards_the_beacon_to_the_watcher():
    """The intended wiring: driver --watch, watcher --readonly.

    The watcher is bound to a port the twin never sends to, so every beacon it
    reports arrived by way of the driving console -- and it reports the
    robot's real state (LIVE, once the driver arms), not a default.
    """
    cmd_port, tlm_port = _ports()
    watch_port = tlm_port + 1                  # _next_port strides 4
    twin = _twin(cmd_port, tlm_port)
    time.sleep(0.5)
    obs = _observer(watch_port)
    time.sleep(0.3)
    drv = _gui([(1.0, "arm"), (1.5, "press forward"), (0.8, "release forward"),
                (0.4, "quit"), (0.4, "")], cmd_port, tlm_port,
               ["--no-record", "--watch", "127.0.0.1:%d" % watch_port])
    try:
        drv_out = drv.communicate(timeout=15)[0]
    except subprocess.TimeoutExpired:
        drv.kill()
        drv_out = drv.communicate()[0]
    try:
        obs.stdin.write("quit\n")
        obs.stdin.flush()
        obs_out = obs.communicate(timeout=10)[0]
    except (subprocess.TimeoutExpired, BrokenPipeError):
        obs.kill()
        obs_out = obs.communicate()[0]
    finally:
        twin.terminate()
        log = twin.communicate(timeout=5)[0]

    assert "ARMED" in drv_out, drv_out          # the driver really did drive
    assert _states(log), log
    # The watcher saw beacons, and they were the robot's -- LIVE is a state
    # only an armed robot reports, and only the driver could have armed it.
    assert "OBSERVER" in obs_out, obs_out
    watched = [int(m) for m in re.findall(r"watched\s+(\d+) beacon", obs_out)]
    assert watched and max(watched) > 0, obs_out
    assert "robot LIVE" in obs_out, obs_out
    # ...and it stayed silent throughout: the driver never lost the link to a
    # second commander stealing the beacon.
    assert "LINK LOST" not in drv_out, drv_out
    assert obs.returncode == 0, obs_out
