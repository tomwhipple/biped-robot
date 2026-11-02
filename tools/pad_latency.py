#!/usr/bin/env python3
"""The gamepad's arrival checks (docs/design-v6/2026-11-01-operator-console-ui.md
§14, and §7 "The Bluetooth hazard"): what SDL sees, whether the pad streams
reports while held still, and how long the OS takes to report a pad that was
switched off.

    tools/pad_latency.py --list     the controllers SDL sees: name, GUID, mapping
    tools/pad_latency.py            the list, then both measurements; every
                                    event goes to a CSV with its time

1. HELD STILL (--still S, default 10 s): put the pad down, sticks centred, no
   buttons. SDL queues an input only when a value changes, so a pad that
   sends nothing here is silent whenever it is held still, and the console's
   silent-pad check cannot be used with it: only the OS disconnect counts. A
   pad that keeps reporting (a motion sensor, stick noise) with its longest
   gap under half the console's timeout (150 ms) can use it.
2. SWITCH-OFF (--off-timeout S, default 60 s): keep moving a stick and switch
   the pad off (or walk out of range). The last input before the disconnect
   marks the moment the radio stopped, to within one report; press Enter at
   the moment you switch it off for a second marker. What is reported is the
   time to SDL's device-removed event: while the dead-man is held, this is how
   long the console would keep sending the pad's last command.

Run it on macOS and on Linux, over Bluetooth and through the dongle, and write
the numbers down (§14) before the pad drives hardware. No console, no robot,
no network: stdlib and pygame only. Times are this host's monotonic clock at
the poll (a 1 ms loop); pygame does not pass SDL's own event stamps on.
"""
from __future__ import annotations

import argparse
import csv
import os
import platform
import select
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
# As the console does (link/console/gamepad.py): inputs while another window
# has focus (this terminal), buttons by position, and SDL's event queue
# without a real video driver.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ["SDL_GAMECONTROLLER_USE_BUTTON_LABELS"] = "0"
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame  # noqa: E402
import pygame._sdl2.controller as sdl_controller  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONSOLE_TIMEOUT_MS = 150.0    # link/console/gamepad.py SILENT_MS

INPUT = {pygame.JOYAXISMOTION: "joy_axis", pygame.JOYBALLMOTION: "joy_ball",
         pygame.JOYHATMOTION: "joy_hat", pygame.JOYBUTTONDOWN: "joy_button_down",
         pygame.JOYBUTTONUP: "joy_button_up", pygame.CONTROLLERAXISMOTION: "ctl_axis",
         pygame.CONTROLLERBUTTONDOWN: "ctl_button_down",
         pygame.CONTROLLERBUTTONUP: "ctl_button_up",
         pygame.CONTROLLERSENSORUPDATE: "ctl_sensor"}
DEVICE = {pygame.JOYDEVICEADDED: "joy_added", pygame.JOYDEVICEREMOVED: "joy_removed",
          pygame.CONTROLLERDEVICEADDED: "ctl_added",
          pygame.CONTROLLERDEVICEREMOVED: "ctl_removed",
          pygame.CONTROLLERDEVICEREMAPPED: "ctl_remapped"}


def init() -> None:
    pygame.display.init()
    pygame.joystick.init()
    sdl_controller.init()


def describe() -> list[dict]:
    """Every joystick SDL sees, opened long enough to read it."""
    out = []
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        d = dict(index=i, name=js.get_name(), guid=js.get_guid(),
                 instance_id=js.get_instance_id(), axes=js.get_numaxes(),
                 buttons=js.get_numbuttons(), hats=js.get_numhats(),
                 power=js.get_power_level(), controller=bool(sdl_controller.is_controller(i)),
                 mapping="")
        if d["controller"]:
            c = sdl_controller.Controller(i)
            m = c.get_mapping()
            d["mapping"] = (",".join(f"{k}:{v}" for k, v in m.items()) if isinstance(m, dict)
                            else str(m))
            d["controller_name"] = c.name
        out.append(d)
    return out


def print_list(devs) -> None:
    print(f"SDL {'.'.join(map(str, pygame.get_sdl_version()))} · pygame {pygame.version.ver} · "
          f"{platform.system()} {platform.release()} ({platform.machine()})")
    if not devs:
        print("no joystick seen by SDL (pair or plug the pad, then run again)")
    for d in devs:
        print(f"[{d['index']}] {d['name']}  GUID {d['guid']}  instance {d['instance_id']}  "
              f"{d['axes']} axes / {d['buttons']} buttons / {d['hats']} hats  "
              f"power {d['power']}")
        if d["controller"]:
            print(f"     GameController: yes ({d.get('controller_name')}); mapping "
                  f"{d['mapping']}")
        else:
            print("     GameController: NO -- no SDL mapping, so the console cannot map it "
                  "by position (it ignores this device)")


class Recorder:
    """Every SDL event, stamped, to the CSV; the loop the phases share."""

    def __init__(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_mono_s", "t_utc", "phase", "event", "instance_id", "detail"])
        self.path = path
        self.phase = "list"

    def row(self, t, event, iid="", detail=""):
        utc = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()) + f".{int(time.time() % 1 * 1e3):03d}Z"
        self.w.writerow([f"{t:.6f}", utc, self.phase, event, iid, detail])

    def poll(self):
        """[(t, kind, instance_id)] drained now; kind in INPUT / DEVICE values."""
        t = time.monotonic()
        out = []
        for ev in pygame.event.get():
            kind = INPUT.get(ev.type) or DEVICE.get(ev.type)
            if kind is None:
                continue
            iid = getattr(ev, "instance_id", getattr(ev, "device_index", ""))
            detail = " ".join(f"{k}={getattr(ev, k)}" for k in ("axis", "value", "button",
                                                                 "hat", "ball", "rel",
                                                                 "device_index")
                              if hasattr(ev, k))
            self.row(t, kind, iid, detail)
            out.append((t, kind, iid))
        return out

    def mark(self, what):
        self.row(time.monotonic(), what)
        self.f.flush()

    def close(self):
        self.f.close()


def enter_pressed() -> bool:
    if not sys.stdin.isatty():
        return False
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        sys.stdin.readline()
        return True
    return False


def wait_enter(prompt: str, rec: Recorder) -> None:
    print(prompt, end="", flush=True)
    while not enter_pressed():
        rec.poll()
        time.sleep(0.005)
        if not sys.stdin.isatty():
            print("(stdin is not a terminal: starting now)")
            return


def still(rec: Recorder, iid, seconds: float) -> dict:
    rec.phase = "still"
    wait_enter(f"\nHELD STILL: put the pad down, sticks centred, hands off. Enter starts "
               f"{seconds:.0f} s ... ", rec)
    rec.mark("still_start")
    t0 = time.monotonic()
    times, kinds = [], {}
    while time.monotonic() - t0 < seconds:
        for t, kind, i in rec.poll():
            if kind in INPUT.values() and (iid is None or i == iid):
                times.append(t)
                kinds[kind] = kinds.get(kind, 0) + 1
        time.sleep(0.001)
    t1 = time.monotonic()
    rec.mark("still_end")
    edges = [t0] + times + [t1]
    gap = max(b - a for a, b in zip(edges, edges[1:]))
    out = dict(events=len(times), rate=len(times) / (t1 - t0), max_gap_ms=gap * 1000.0,
               kinds=kinds)
    streams = bool(times) and gap * 1000.0 < CONSOLE_TIMEOUT_MS / 2.0
    print(f"  {len(times)} input events in {t1 - t0:.1f} s = {out['rate']:.1f}/s held still; "
          f"longest gap {gap * 1000.0:.0f} ms; by type {kinds or '{}'}")
    print("  -> streams while still: the console's silent-pad check can be used "
          f"(timeout {CONSOLE_TIMEOUT_MS:.0f} ms)" if streams else
          "  -> does NOT stream while still: silence means nothing, only the OS disconnect "
          "counts")
    out["streams"] = streams
    return out


def switch_off(rec: Recorder, iid, timeout: float) -> dict:
    rec.phase = "off"
    print(f"\nSWITCH-OFF: keep moving a stick, and switch the pad off (or walk out of "
          f"range). Press Enter at the moment you switch it off, if you can. Waiting up to "
          f"{timeout:.0f} s for the disconnect ...", flush=True)
    rec.mark("off_start")
    t0 = time.monotonic()
    last_input = t_enter = None
    while time.monotonic() - t0 < timeout:
        if enter_pressed():
            t_enter = time.monotonic()
            rec.mark("enter")
        for t, kind, i in rec.poll():
            if kind in INPUT.values() and (iid is None or i == iid):
                last_input = t
            elif kind in ("ctl_removed", "joy_removed") and (iid is None or i == iid):
                rec.mark("removed_seen")
                out = dict(removed=t,
                           after_last_input_ms=(t - last_input) * 1000.0 if last_input else None,
                           after_enter_ms=(t - t_enter) * 1000.0 if t_enter else None)
                a, e = out["after_last_input_ms"], out["after_enter_ms"]
                print(f"  device removed {('%.0f ms' % a) if a is not None else '(no input seen)'}"
                      f" after the last input"
                      + (f", {e:.0f} ms after Enter" if e is not None else ""))
                print("  -> with the dead-man held, the console keeps sending the pad's last "
                      "command this long unless the silent-pad check applies")
                return out
        time.sleep(0.001)
    print(f"  no disconnect within {timeout:.0f} s"
          + (f" ({(time.monotonic() - last_input):.1f} s since the last input)"
             if last_input else ""))
    return dict(removed=None)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("\n\n", 1)[1])
    p.add_argument("--list", action="store_true", help="list what SDL sees, then quit")
    p.add_argument("--index", type=int, default=None,
                   help="the joystick to measure (default: the first GameController)")
    p.add_argument("--still", type=float, default=10.0, help="held-still window, s")
    p.add_argument("--off-timeout", type=float, default=60.0,
                   help="how long to wait for the disconnect, s")
    p.add_argument("--skip-off", action="store_true", help="only the held-still check")
    p.add_argument("--csv", help="event log (default hw_sessions/pad_latency_<UTC>.csv)")
    a = p.parse_args(argv)
    init()
    # SDL announces devices already attached as "added" events; let them land.
    t_end = time.monotonic() + 0.5
    while time.monotonic() < t_end:
        pygame.event.pump()
        time.sleep(0.01)
    devs = describe()
    print_list(devs)
    if a.list:
        return 0
    pick = [d for d in devs if (a.index is None and d["controller"]) or d["index"] == a.index]
    if not pick:
        print("nothing to measure")
        return 2
    dev = pick[0]
    path = a.csv or os.path.join(ROOT, "hw_sessions", time.strftime(
        "pad_latency_%Y%m%dT%H%M%SZ.csv", time.gmtime()))
    rec = Recorder(path)
    rec.row(time.monotonic(), "device", dev["instance_id"],
            f"name={dev['name']} guid={dev['guid']} controller={dev['controller']} "
            f"mapping={dev['mapping']} sdl={'.'.join(map(str, pygame.get_sdl_version()))} "
            f"os={platform.system()} {platform.release()}")
    print(f"\nmeasuring [{dev['index']}] {dev['name']}; every event -> {path}")
    try:
        still(rec, dev["instance_id"], a.still)
        if not a.skip_off:
            switch_off(rec, dev["instance_id"], a.off_timeout)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        rec.close()
        pygame.quit()
    print(f"events: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
