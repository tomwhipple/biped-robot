"""--headless: the identical link loop with stdin instead of a window.

The same verbs as `bimo_gui --headless` (press / release / set / sim / cam /
view / any keymap action name), and the same status line every 500 ms, so
tests/test_gui_e2e.py runs against both consoles. EOF on stdin quits, and
quitting disarms.
"""
from __future__ import annotations

import os
import select
import signal
import sys
import time

from . import bench, core
from .controller import Controller, EventLog

EXT_NAMES = ("crouch", "lift", "foot_dx", "foot_dz")


def status_line(c: Controller, st, now: float, t0: float) -> str:
    fl = "".join(n for bit, n in ((core.FLAG_ARM, "ARM "), (core.FLAG_ENABLE, "ENABLE "),
                                  (core.FLAG_ESTOP, "ESTOP "), (core.FLAG_HOME, "HOME "))
                 if st.flags & bit)
    t = (now - t0) / 1000.0
    if st.readonly:
        # No send half at all; printing zeros would read as "commanding a
        # stop", which an observer must never look like it is doing.
        line = (f"[{t:7.2f}s] OBSERVER  watched {st.tlm_count:5d} beacon(s) "
                f"{min(st.rate_hz, 999.9):5.1f} Hz")
    else:
        line = (f"[{t:7.2f}s] arm {'ARMED' if st.armed else 'disarmed':<8s} "
                f"motion {c.core.motion_text():<12s} send vx {st.vx:+.2f} "
                f"vy {st.vy:+.2f} wz {st.wz:+.2f} crouch {st.sent_crouch:.2f} "
                f"len {st.last_len} flags {fl or '(none)'}")
    if st.fresh:
        line += (f" | robot {core.state_name(st.state)}  {st.vbat_v:.2f} V  "
                 f"up_z {st.up_z:.2f}  vx_est {st.vx_est:+.2f}")
        if st.state == core.BENCH:
            line += f"  BENCH: {core.diag_reason(st.diag)}"
    else:
        line += " | robot (no telemetry)"
    b = getattr(c, "bench", None)
    if b is not None:
        line += b.status_text(now)
    if c.stopping():
        line += " | STOP sending"
    return line


def command(c: Controller, line: str) -> None:
    words = line.split()
    if not words:
        return
    verb, args = words[0], words[1:]
    arg = args[0] if args else None
    if verb in ("press", "release"):
        if arg is None:
            return
        axis = core.action_from_name(arg)
        if not 0 <= axis < core.motion_actions():
            print(f"note: `{arg}` is not a motion action", flush=True)
            return
        if verb == "press":
            c.press(axis)
        else:
            c.release(axis)
    elif verb == "set":
        if len(args) < 2:
            return
        if arg not in EXT_NAMES:
            print(f"note: no channel `{arg}`", flush=True)
            return
        st = c.core.status(core.now_ms())
        ext = {n: getattr(st, n) for n in EXT_NAMES}
        ext[arg] = float(args[1])
        c.set_ext(ext["crouch"], ext["lift"], ext["foot_dx"], ext["foot_dz"])
    elif verb == "sim":
        # The Start button, scriptable (bimo_gui's verbs): the spawn path is
        # the part of the console a test cannot otherwise reach.
        arg2 = args[1] if len(args) > 1 else None
        if arg == "start":
            # An unknown run starts nothing (bimo_gui started the current
            # selection anyway: a typo ran a policy nobody named).
            c.sim.start(arg2)
            print(f"note: {c.sim.status}", flush=True)
        elif arg == "stop":
            c.sim.stop()
        elif arg == "reset":
            print(f"note: {'sim reset sent' if c.sim.reset() else 'no sim listening'}",
                  flush=True)
        elif arg == "hang":
            c.sim.set_hang(not c.sim.hang)
            print(f"note: hang {'on' if c.sim.hang else 'off'}", flush=True)
        elif arg == "twin":
            print(f"note: {c.sim.set_twin(arg2 != 'off')}", flush=True)
        elif arg == "mirror":
            c.set_mirror(arg2 != "off")
            print(f"note: {c.status_msg}", flush=True)
        else:
            print("note: sim start [run]|stop|reset|hang|twin [off]|mirror [off]", flush=True)
    elif verb in ("cam", "view"):
        note = bench.verb(getattr(c, "bench", None), verb, args)
        if note:
            print(f"note: {note}", flush=True)
    elif verb in ("tab", "panel"):
        pass                                  # window-only; harmless here
    elif not c.fire(verb):
        print(f"note: unknown command `{verb}`", flush=True)


def run(opts) -> int:
    log = EventLog(os.path.join(opts.sessions, "bimo_console.log"),
                   echo=lambda s: print(s, file=sys.stderr, flush=True))
    c = Controller(opts, log)
    if not c.start():
        log.close()
        return 2
    b = bench.attach(c) if opts.bench_telemetry or opts.cam_relay else None
    # SIGTERM / Ctrl-C leave through the same path as `quit`: disarming.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(c, "quit", True))
    if opts.observe:
        print(f"bimo_console --headless: OBSERVER, no transmit socket; listening "
              f"for beacons on :{opts.tlm_port}", flush=True)
    else:
        print(f"bimo_console --headless -> {opts.host}:{opts.cmd_port} "
              f"(telemetry :{opts.tlm_port})", flush=True)
    # Whatever ends the loop -- quit, EOF, a signal, an exception -- leaves
    # through shutdown(): the driver's disarm frames go out first.
    try:
        period = 1000.0 / opts.rate
        t0 = core.now_ms()
        nxt = next_status = t0
        last_note = ""
        buf = b""
        fd = sys.stdin.fileno()
        while not c.quit:
            now = core.now_ms()
            while select.select([fd], [], [], 0)[0]:
                chunk = os.read(fd, 4096)
                if not chunk:
                    c.quit = True             # EOF: leave, disarming
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    command(c, line.decode(errors="replace"))
            if c.quit:
                break
            st = c.tick(now)
            if c.core.note != last_note:
                last_note = c.core.note
                if last_note:
                    print(f"note: {last_note}", flush=True)
            if now >= next_status:
                print(status_line(c, st, now, t0), flush=True)
                next_status = now + 500.0
            if opts.quit_after and now - t0 >= opts.quit_after * 1000.0:
                c.quit = True
            nxt += period
            sleep = nxt - core.now_ms()
            if sleep > 0:
                time.sleep(sleep / 1000.0)
            else:
                nxt = core.now_ms()
    finally:
        try:
            c.shutdown()
        finally:
            if b is not None:
                b.close()
            log.close()
    return 0
