"""ctypes driver for libngspice.

The suite talks to ngspice via its shared-library C API. We use exactly four
entry points — `ngSpice_Init`, `ngSpice_Command`, `ngSpice_CurPlot`,
`ngSpice_Get_Vec_Info` — and define the ABI structs per the ngspice manual
(§19.6, ngspice-42 sharedspice.h):

    struct vector_info {
        char *name; int type; short flags;
        double *realdata; COMPLEX *compdata; int length;
    };

Which library loads (`find_lib`): `$PBSIM_NGSPICE` if set; else the vendored
libngspice.so.0.0.9 on linux-x86_64 (an ELF, so nowhere else); else a native
libngspice (Homebrew's `brew install libngspice` on the Mac). Verified live
by test_ngsmoke.py against the vendored ngspice 42 and Homebrew's ngspice 47.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import os
import platform
import sys
import tempfile
from ctypes import (POINTER, PYFUNCTYPE, Structure, c_bool, c_char_p,
                    c_double, c_int, c_short, c_void_p)
from pathlib import Path
from typing import Any

VENDOR_DIR = Path(__file__).resolve().parent / "vendor" / "ngspice"
LIB_SO = VENDOR_DIR / "libngspice.so.0.0.9"
NATIVE_CANDIDATES = (
    "/opt/homebrew/lib/libngspice.dylib",   # Homebrew, Apple silicon
    "/usr/local/lib/libngspice.dylib",      # Homebrew, Intel
)


def vendored_lib_runs_here() -> bool:
    return sys.platform.startswith("linux") and \
        platform.machine() in ("x86_64", "AMD64")


def find_lib() -> str | None:
    """The libngspice this host can load, or None."""
    env = os.environ.get("PBSIM_NGSPICE")
    if env:
        return env
    if vendored_lib_runs_here() and LIB_SO.exists():
        return str(LIB_SO)
    for cand in NATIVE_CANDIDATES:
        if Path(cand).exists():
            return cand
    return ctypes.util.find_library("ngspice")


class VectorInfo(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("type", c_int),
        ("flags", c_short),
        ("realdata", POINTER(c_double)),
        ("compdata", c_void_p),
        ("length", c_int),
    ]


class VecValues(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("creal", c_double),
        ("cimag", c_double),
        ("is_scale", c_bool),
        ("is_complex", c_bool),
        ("length", c_int),
        ("flags", c_short),
    ]


SendChar = PYFUNCTYPE(c_int, c_char_p, c_int, c_void_p)
SendStat = PYFUNCTYPE(c_int, c_char_p, c_int, c_void_p)
ControlledExit = PYFUNCTYPE(c_int, c_int, c_bool, c_bool, c_int, c_void_p)
SendData = PYFUNCTYPE(c_int, POINTER(VecValues), c_int, c_int, c_void_p)
SendInitData = PYFUNCTYPE(c_int, c_void_p, c_int, c_void_p)
BGThreadRunning = PYFUNCTYPE(c_int, c_bool, c_int, c_void_p)


class NgSpiceError(RuntimeError):
    pass


class _Engine:
    """One loaded, initialised libngspice per process. ngspice's shared
    mode expects the caller to unload the library after `quit`, so a
    second ngSpice_Init on the same handle is undefined (it SIGTRAPs
    Homebrew's ngspice 47); sessions share this instead and clear their
    circuits and plots on close."""

    def __init__(self, path: str):
        if path == str(LIB_SO):
            os.environ.setdefault("SPICE_LIB_DIR",
                                  "/usr/lib/x86_64-linux-gnu/ngspice")
        self.path = path
        self.lib = ctypes.CDLL(path)
        self.sink: list[str] | None = None
        self._setup()
        rc = self.lib.ngSpice_Init(*self._cbs, None)
        if rc != 0:
            raise NgSpiceError(f"ngSpice_Init returned {rc}")

    def _setup(self):
        lib = self.lib
        lib.ngSpice_Init.restype = c_int
        lib.ngSpice_Init.argtypes = [
            SendChar, SendStat, ControlledExit, SendData, SendInitData,
            BGThreadRunning, c_void_p,
        ]
        lib.ngSpice_Command.argtypes = [c_char_p]
        lib.ngSpice_Command.restype = c_int
        lib.ngSpice_CurPlot.argtypes = []
        lib.ngSpice_CurPlot.restype = c_char_p
        lib.ngGet_Vec_Info.argtypes = [c_char_p]
        lib.ngGet_Vec_Info.restype = POINTER(VectorInfo)
        lib.ngSpice_AllVecs.argtypes = [c_char_p]
        lib.ngSpice_AllVecs.restype = POINTER(c_char_p)
        this = self

        @SendChar
        def _on_char(txt_p, _id, _user):
            if txt_p and this.sink is not None:
                this.sink.append(txt_p.decode("utf-8", "replace"))
            return 0

        @SendStat
        def _on_stat(_status, _id, _user):
            return 0

        @ControlledExit
        def _on_cexit(status, _immediate, _exit_on_quit, _id, _user):
            return status

        @SendData
        def _on_data(_values, _count, _id, _user):
            return 0

        @SendInitData
        def _on_init(_vecs, _id, _user):
            return 0

        @BGThreadRunning
        def _on_bg(_running, _id, _user):
            return 0

        self._cbs = (_on_char, _on_stat, _on_cexit, _on_data, _on_init, _on_bg)


_ENGINES: dict[str, _Engine] = {}


class NgSpiceSession:
    """One ngspice session per test, on the process's shared engine."""

    def __init__(self):
        lib = find_lib()
        if lib is None:
            raise NgSpiceError(
                "no libngspice for this host: the vendored one is "
                "linux-x86_64 only; elsewhere install one (macOS: `brew "
                "install libngspice`) or set PBSIM_NGSPICE — see "
                "vendor/README.md"
            )
        self.lib_path = lib
        self._captured: list[str] = []
        self._circuits = 0
        self._closed = True

    # ---------------- lifecycle ---------------------------------------------

    def open(self):
        eng = _ENGINES.get(self.lib_path)
        if eng is None:
            eng = _ENGINES[self.lib_path] = _Engine(self.lib_path)
        self._eng = eng
        self._lib = eng.lib
        eng.sink = self._captured
        self._closed = False
        return self

    def close(self):
        if not self._closed:
            for _ in range(self._circuits):
                self._lib.ngSpice_Command(b"remcirc")
            self._lib.ngSpice_Command(b"destroy all")
            self._eng.sink = None
            self._closed = True

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    # ---------------- commands -----------------------------------------------

    def cmd(self, text: str) -> str:
        before = len(self._captured)
        rc = self._lib.ngSpice_Command(text.encode("utf-8"))
        new = "".join(self._captured[before:])
        if rc != 0:
            raise NgSpiceError(
                f"ngspice cmd {text!r} rc={rc}: {new.strip()}"
            )
        lc = new.lower()
        for marker in ("error on line", "no such vector",
                       "cannot compute substitute", "undefined parameter",
                       "no such file", "cannot find", "no such model"):
            if marker in lc:
                raise NgSpiceError(
                    f"ngspice cmd {text!r} printed error: {new.strip()}"
                )
        return new

    def source(self, path: os.PathLike | str):
        out = self.cmd(f"source {path}")
        self._circuits += 1
        return out

    # ---------------- vectors ------------------------------------------------

    def curplot(self) -> str:
        p = self._lib.ngSpice_CurPlot()
        return p.decode() if p else ""

    def vector_names(self) -> list[str]:
        ptr = self._lib.ngSpice_AllVecs(self.curplot().encode())
        out: list[str] = []
        if not ptr:
            return out
        i = 0
        while ptr[i]:
            out.append(ptr[i].decode())
            i += 1
        return out

    def vector(self, name: str):
        import numpy as np
        info_p = self._lib.ngGet_Vec_Info(name.encode("utf-8"))
        if not info_p:
            raise NgSpiceError(
                f"no such vector {name!r} in plot {self.curplot()!r}"
            )
        v = info_p.contents
        if v.length < 0 or not v.realdata:
            raise NgSpiceError(
                f"vector {name!r}: length={v.length}, realdata empty"
            )
        arr = np.ctypeslib.as_array(v.realdata, shape=(v.length,))
        return arr.copy()

    # ---------------- convenience ----------------------------------------------

    def run_netlist(self, netlist: str,
                    vectors: list[str] | None = None
                    ) -> dict[str, Any]:
        """Write netlist to a temp .cir, source it, run, return dict of
        vector name -> ndarray (every vector when `vectors` is None)."""
        with tempfile.TemporaryDirectory(prefix="power-sim-") as td:
            p = Path(td) / "net.cir"
            p.write_text(netlist)
            self.source(p)
            self.cmd("run")
            names = vectors if vectors is not None else self.vector_names()
            out: dict[str, Any] = {n: self.vector(n) for n in names}
            out["__plot__"] = self.curplot()
            return out
