"""The console core (link/console_api.h, link/client.h) through ctypes.

Every rule that decides what goes on the wire -- arming, the E-stop latch, the
home frames, the handover, the short-vs-long frame -- runs in the C++ core
bimo_gui and bimo_tui share; this module only calls it. Build it with

    make -C firmware/host console

$BIMO_CONSOLE_LIB overrides the library path. The library is refused when it
is older than any of its sources (when they are present), or its API version,
functions or status-struct size differ from the mirror below, so a stale build
fails loudly at start-up instead of misreading a field.

Stdlib only: the headless console (and its tests) run on a bare python.
"""
from __future__ import annotations

import ctypes
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LINK_DIR = os.path.dirname(HERE)
ROOT = os.path.dirname(LINK_DIR)

API_VERSION = 2
NUM_JOINTS = 17
NUM_AXES = 6
LIB_NAME = "libbimo_console." + ("dylib" if sys.platform == "darwin" else "so")
DEFAULT_LIB = os.path.join(ROOT, "firmware", "host", "build", LIB_NAME)

# client.h Axis, in order; keymap.h's first six actions are the same.
AXES = ("forward", "back", "strafe_left", "strafe_right", "turn_left",
        "turn_right")
# linkproto::LinkState wire order (protocol.py list(LinkState) order).
LIVE, STAND, RELAX, ESTOP, VLAND, VSAFE, FALLEN, BENCH = range(8)
STATE_SHORT = ("LIVE", "STAND", "RELAX", "ESTOP", "VLAND", "VSAFE", "FALLEN",
               "BENCH")
# Command flags (linkproto kFlag*).
FLAG_ENABLE, FLAG_ESTOP, FLAG_ARM, FLAG_POSE = 1, 2, 4, 8
FLAG_HOME, FLAG_ATT, FLAG_STOP = 16, 32, 64


class Status(ctypes.Structure):
    """link/console_api.h BimoStatus, field for field."""
    _fields_ = [
        ("readonly", ctypes.c_int32), ("has_dest", ctypes.c_int32),
        ("watch_on", ctypes.c_int32), ("have_tlm", ctypes.c_int32),
        ("link_lost", ctypes.c_int32),
        ("seq", ctypes.c_uint32), ("sent", ctypes.c_uint32),
        ("tlm_count", ctypes.c_uint32), ("tlm_bad", ctypes.c_uint32),
        ("watched", ctypes.c_uint32), ("last_len", ctypes.c_uint32),
        ("tlm_age_ms", ctypes.c_double), ("rate_hz", ctypes.c_double),
        ("lag", ctypes.c_int64),
        ("seq_echo", ctypes.c_uint32), ("servo_err", ctypes.c_uint32),
        ("state", ctypes.c_int32), ("diag", ctypes.c_int32),
        ("vbat_v", ctypes.c_float), ("up_z", ctypes.c_float),
        ("vx_est", ctypes.c_float), ("wz_est", ctypes.c_float),
        ("up_x", ctypes.c_float), ("up_y", ctypes.c_float),
        ("loop_late_pct", ctypes.c_int32), ("n_joints", ctypes.c_int32),
        ("have_att", ctypes.c_int32),
        ("t_us", ctypes.c_uint64),
        ("joints", ctypes.c_float * NUM_JOINTS),
        ("armed", ctypes.c_int32), ("estop", ctypes.c_int32),
        ("want_pose", ctypes.c_int32), ("want_att", ctypes.c_int32),
        ("home_frames", ctypes.c_int32), ("moving", ctypes.c_int32),
        ("held", ctypes.c_int32 * NUM_AXES),
        ("crouch", ctypes.c_float), ("lift", ctypes.c_float),
        ("foot_dx", ctypes.c_float), ("foot_dz", ctypes.c_float),
        ("speed_vx", ctypes.c_float), ("speed_vy", ctypes.c_float),
        ("speed_wz", ctypes.c_float),
        ("vx", ctypes.c_float), ("vy", ctypes.c_float), ("wz", ctypes.c_float),
        ("sent_crouch", ctypes.c_float), ("sent_lift", ctypes.c_float),
        ("sent_foot_dx", ctypes.c_float), ("sent_foot_dz", ctypes.c_float),
        ("flags", ctypes.c_uint32),
        ("rec_active", ctypes.c_int32),
        ("rec_cmd", ctypes.c_uint32), ("rec_tlm", ctypes.c_uint32),
        ("rec_bench", ctypes.c_uint32),
        ("tee_on", ctypes.c_int32), ("teed", ctypes.c_uint32),
    ]

    @property
    def fresh(self) -> bool:
        """A beacon, and not a stale one."""
        return bool(self.have_tlm) and not self.link_lost


class LibraryError(RuntimeError):
    pass


_P = ctypes.c_void_p
_S = ctypes.c_char_p
_I = ctypes.c_int
_D = ctypes.c_double
_F = ctypes.c_float
_Z = ctypes.c_size_t
_SIGS = {
    "bimo_api_version": (_I, []),
    "bimo_status_size": (_Z, []),
    "bimo_now_ms": (_D, []),
    "bimo_new": (_P, []),
    "bimo_free": (None, [_P]),
    "bimo_open": (_I, [_P, _S, _I, _I]),
    "bimo_listen": (_I, [_P, _I, _S, _I]),
    "bimo_relink": (_I, [_P, _S, _I, _I, _I]),
    "bimo_set_watch": (_I, [_P, _S, _I]),
    "bimo_clear_watch": (None, [_P]),
    "bimo_set_tee": (_I, [_P, _S, _I]),
    "bimo_clear_tee": (None, [_P]),
    "bimo_close": (None, [_P]),
    "bimo_tick": (None, [_P, _D]),
    "bimo_status": (None, [_P, _D, ctypes.POINTER(Status)]),
    "bimo_poll": (None, [_P, _D]),
    "bimo_toggle_control": (_I, [_P, _S, _D, ctypes.c_char_p, _Z]),
    "bimo_leave_disarmed": (None, [_P, _I, _I]),
    "bimo_note": (_S, [_P]),
    "bimo_say": (None, [_P, _S]),
    "bimo_press": (_I, [_P, _I, _D]),
    "bimo_release": (None, [_P, _I]),
    "bimo_release_all": (None, [_P]),
    "bimo_toggle_arm": (None, [_P, _D]),
    "bimo_stand": (None, [_P, _S]),
    "bimo_estop": (None, [_P]),
    "bimo_home": (None, [_P, _D]),
    "bimo_set_ext": (None, [_P, _F, _F, _F, _F]),
    "bimo_reset_ext": (None, [_P]),
    "bimo_set_speeds": (None, [_P, _F, _F, _F]),
    "bimo_request": (None, [_P, _I, _I]),
    "bimo_motion_text": (_S, [_P]),
    "bimo_rec_start": (_I, [_P, _S, _S, _D, ctypes.c_char_p, _Z]),
    "bimo_rec_stop": (None, [_P]),
    "bimo_rec_path": (_S, [_P]),
    "bimo_rec_cam": (None, [_P, _D, _I, _S, _S]),
    "bimo_rec_bench": (None, [_P, _D, _S, _Z]),
    "bimo_state_name": (_S, [_I]),
    "bimo_diag_reason": (_S, [_I]),
    "bimo_diag_arm_result": (_I, [_I]),
    "bimo_fault_ids": (_S, [ctypes.c_uint32, ctypes.c_char_p, _Z]),
    "bimo_axis_name": (_S, [_I]),
    "bimo_action_count": (_I, []),
    "bimo_motion_actions": (_I, []),
    "bimo_action_name": (_S, [_I]),
    "bimo_action_label": (_S, [_I]),
    "bimo_action_from_name": (_I, [_S]),
    "bimo_key_name": (_S, [_I]),
    "bimo_key_from_name": (_I, [_S]),
    "bimo_keymap_key": (_I, [_P, _I]),
    "bimo_keymap_action_for": (_I, [_P, _I]),
    "bimo_keymap_bind": (None, [_P, _I, _I]),
    "bimo_keymap_reset": (None, [_P]),
    "bimo_keymap_load": (_I, [_P, _S, ctypes.c_char_p, _Z]),
    "bimo_keymap_save": (_I, [_P, _S, ctypes.c_char_p, _Z]),
    "bimo_default_keymap_path": (_I, [ctypes.c_char_p, _Z]),
}

_lib = None


REBUILD = "rebuild it: make -C firmware/host console"


def _sources() -> list[str]:
    """What the library is built from (firmware/host/Makefile's console
    rule), or [] when they are not here -- an installed copy, checked by its
    API version and status size alone."""
    import glob
    link = [os.path.join(LINK_DIR, f"{n}.{x}")
            for n in ("client", "console_api", "keymap", "recorder")
            for x in ("h", "cpp")]
    lp = os.path.join(ROOT, "firmware", "components", "linkproto")
    proto = (glob.glob(os.path.join(lp, "include", "linkproto", "*.h"))
             + glob.glob(os.path.join(lp, "*.cpp")))
    if not all(os.path.exists(p) for p in link) or not proto:
        return []
    return link + proto


def stale_sources(path: str) -> list[str]:
    """Sources newer than the library at `path` (repo-relative names)."""
    try:
        built = os.path.getmtime(path)
    except OSError:
        return []
    out = []
    for src in _sources():
        try:
            if os.path.getmtime(src) > built:
                out.append(os.path.relpath(src, ROOT))
        except OSError:
            pass
    return out


def library(path: str | None = None) -> ctypes.CDLL:
    """The loaded, checked core library (once per process).

    Refused, with the rebuild command, when it is older than any of its
    sources, speaks another API version, lacks a function this module calls,
    or lays the status struct out differently -- a stale build fails loudly
    at start-up instead of misreading a field or sending the wrong frame."""
    global _lib
    if _lib is not None:
        return _lib
    path = path or os.environ.get("BIMO_CONSOLE_LIB") or DEFAULT_LIB
    if not os.path.exists(path):
        raise LibraryError(f"{path} not found -- build it: "
                           f"make -C firmware/host console")
    newer = stale_sources(path)
    if newer:
        more = f" (+{len(newer) - 3} more)" if len(newer) > 3 else ""
        raise LibraryError(f"{path} is older than {', '.join(newer[:3])}{more} "
                           f"-- {REBUILD}")
    try:
        lib = ctypes.CDLL(path)
    except OSError as e:
        raise LibraryError(f"{path}: cannot load ({e}) -- {REBUILD}") from e
    try:
        ver = lib.bimo_api_version
        ver.restype, ver.argtypes = ctypes.c_int, []
    except AttributeError as e:
        raise LibraryError(f"{path}: no bimo_api_version -- {REBUILD}") from e
    if ver() != API_VERSION:
        raise LibraryError(f"{path}: API version {ver()}, this console "
                           f"speaks {API_VERSION} -- {REBUILD}")
    for name, (res, args) in _SIGS.items():
        try:
            fn = getattr(lib, name)
        except AttributeError as e:
            raise LibraryError(f"{path}: no {name}() -- {REBUILD}") from e
        fn.restype = res
        fn.argtypes = args
    if lib.bimo_status_size() != ctypes.sizeof(Status):
        raise LibraryError(f"{path}: status is {lib.bimo_status_size()} B, "
                           f"core.py expects {ctypes.sizeof(Status)} -- rebuild "
                           f"it, or fix the mirror")
    _lib = lib
    return lib


def _b(s: str | None) -> bytes | None:
    return None if s is None else s.encode()


def _s(b: bytes | None) -> str:
    return "" if b is None else b.decode(errors="replace")


def now_ms() -> float:
    """The core's monotonic clock: the one its link times are on."""
    return library().bimo_now_ms()


def state_name(state: int) -> str:
    return _s(library().bimo_state_name(state))


def diag_reason(diag: int) -> str:
    return _s(library().bimo_diag_reason(diag))


def diag_arm_result(diag: int) -> int:
    return library().bimo_diag_arm_result(diag)


def lag_text(st, width: int = 0) -> str:
    """The beacon lag for a label: "lag —" while the robot reports BENCH,
    when seq_echo carries the diagnostic byte and there is no lag (#237;
    Link::lag() gives 0 there)."""
    if st.state == BENCH:
        return "lag " + "—".rjust(width)
    return f"lag {st.lag:{width}d}" if width else f"lag {st.lag}"


def fault_ids(servo_err: int) -> str:
    buf = ctypes.create_string_buffer(128)
    return _s(library().bimo_fault_ids(servo_err & 0xFFFFFFFF, buf, len(buf)))


def axis_name(axis: int) -> str:
    return _s(library().bimo_axis_name(axis))


def action_count() -> int:
    return library().bimo_action_count()


def motion_actions() -> int:
    return library().bimo_motion_actions()


def action_name(action: int) -> str:
    return _s(library().bimo_action_name(action))


def action_label(action: int) -> str:
    return _s(library().bimo_action_label(action))


def action_from_name(name: str) -> int:
    return library().bimo_action_from_name(_b(name))


def key_name(key: int) -> str:
    return _s(library().bimo_key_name(key))


def key_from_name(name: str) -> int:
    return library().bimo_key_from_name(_b(name))


def default_keymap_path() -> str:
    buf = ctypes.create_string_buffer(1024)
    return buf.value.decode() if library().bimo_default_keymap_path(buf, len(buf)) else ""


class Core:
    """One console: a Link, an Intent, speeds, a keymap and a recorder."""

    def __init__(self, lib_path: str | None = None):
        self._lib = library(lib_path)
        self._c = self._lib.bimo_new()
        self._st = Status()

    def free(self) -> None:
        if self._c:
            self._lib.bimo_free(self._c)
            self._c = None

    # -- the link --------------------------------------------------------
    def open(self, host: str, cmd_port: int, tlm_port: int) -> bool:
        return bool(self._lib.bimo_open(self._c, _b(host), cmd_port, tlm_port))

    def listen(self, tlm_port: int, host: str | None, cmd_port: int) -> bool:
        return bool(self._lib.bimo_listen(self._c, tlm_port, _b(host), cmd_port))

    def relink(self, host: str, cmd_port: int, tlm_port: int, observe: bool) -> bool:
        return bool(self._lib.bimo_relink(self._c, _b(host), cmd_port, tlm_port,
                                          int(observe)))

    def set_watch(self, spec: str, default_port: int) -> bool:
        return bool(self._lib.bimo_set_watch(self._c, _b(spec), default_port))

    def clear_watch(self) -> None:
        self._lib.bimo_clear_watch(self._c)

    def set_tee(self, host: str, port: int) -> bool:
        """Twin-in-the-loop: every command frame also to host:port."""
        return bool(self._lib.bimo_set_tee(self._c, _b(host), port))

    def clear_tee(self) -> None:
        self._lib.bimo_clear_tee(self._c)

    def close(self) -> None:
        self._lib.bimo_close(self._c)

    def tick(self, now: float) -> None:
        self._lib.bimo_tick(self._c, now)

    def poll(self, now: float) -> None:
        """Drain beacons; send nothing."""
        self._lib.bimo_poll(self._c, now)

    def status(self, now: float) -> Status:
        """A fresh snapshot (a new object each call)."""
        st = Status()
        self._lib.bimo_status(self._c, now, ctypes.byref(st))
        return st

    def toggle_control(self, host: str | None, now: float) -> tuple[bool, str]:
        buf = ctypes.create_string_buffer(512)
        ok = self._lib.bimo_toggle_control(self._c, _b(host), now, buf, len(buf))
        return bool(ok), buf.value.decode(errors="replace")

    def leave_disarmed(self, frames: int = 10, gap_ms: int = 20) -> None:
        self._lib.bimo_leave_disarmed(self._c, frames, gap_ms)

    # -- the intent ------------------------------------------------------
    @property
    def note(self) -> str:
        return _s(self._lib.bimo_note(self._c))

    def say(self, msg: str) -> None:
        self._lib.bimo_say(self._c, _b(msg))

    def press(self, axis: int, now: float) -> bool:
        return bool(self._lib.bimo_press(self._c, axis, now))

    def release(self, axis: int) -> None:
        self._lib.bimo_release(self._c, axis)

    def release_all(self) -> None:
        self._lib.bimo_release_all(self._c)

    def toggle_arm(self, now: float) -> None:
        self._lib.bimo_toggle_arm(self._c, now)

    def stand(self, why: str = "stand") -> None:
        self._lib.bimo_stand(self._c, _b(why))

    def estop(self) -> None:
        self._lib.bimo_estop(self._c)

    def home(self, now: float) -> None:
        self._lib.bimo_home(self._c, now)

    def set_ext(self, crouch: float, lift: float, foot_dx: float,
                foot_dz: float) -> None:
        self._lib.bimo_set_ext(self._c, crouch, lift, foot_dx, foot_dz)

    def reset_ext(self) -> None:
        self._lib.bimo_reset_ext(self._c)

    def set_speeds(self, vx: float, vy: float, wz: float) -> None:
        self._lib.bimo_set_speeds(self._c, vx, vy, wz)

    def request(self, pose: bool, att: bool) -> None:
        self._lib.bimo_request(self._c, int(pose), int(att))

    def motion_text(self) -> str:
        return _s(self._lib.bimo_motion_text(self._c))

    # -- the recording ---------------------------------------------------
    def rec_start(self, directory: str, host: str, now: float) -> tuple[bool, str]:
        err = ctypes.create_string_buffer(256)
        ok = self._lib.bimo_rec_start(self._c, _b(directory), _b(host), now,
                                      err, len(err))
        return bool(ok), err.value.decode(errors="replace")

    def rec_stop(self) -> None:
        self._lib.bimo_rec_stop(self._c)

    @property
    def rec_path(self) -> str:
        return _s(self._lib.bimo_rec_path(self._c))

    def rec_cam(self, now: float, pane: int, src: str, url: str) -> None:
        self._lib.bimo_rec_cam(self._c, now, pane, _b(src), _b(url))

    def rec_bench(self, now: float, datagram: bytes) -> None:
        self._lib.bimo_rec_bench(self._c, now, datagram, len(datagram))

    # -- the keymap ------------------------------------------------------
    def key_for(self, action: int) -> int:
        return self._lib.bimo_keymap_key(self._c, action)

    def action_for(self, key: int) -> int:
        return self._lib.bimo_keymap_action_for(self._c, key)

    def bind(self, action: int, key: int) -> None:
        self._lib.bimo_keymap_bind(self._c, action, key)

    def reset_keys(self) -> None:
        self._lib.bimo_keymap_reset(self._c)

    def load_keys(self, path: str) -> tuple[bool, str]:
        err = ctypes.create_string_buffer(256)
        ok = self._lib.bimo_keymap_load(self._c, _b(path), err, len(err))
        return bool(ok), err.value.decode(errors="replace")

    def save_keys(self, path: str) -> tuple[bool, str]:
        err = ctypes.create_string_buffer(256)
        ok = self._lib.bimo_keymap_save(self._c, _b(path), err, len(err))
        return bool(ok), err.value.decode(errors="replace")
