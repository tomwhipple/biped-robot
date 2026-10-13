"""ctypes driver for the vendored libngspice.

The suite talks to ngspice via its shared-library C API. We use exactly four
entry points — `ngSpice_Init`, `ngSpice_Command`, `ngSpice_CurPlot`,
`ngSpice_Get_Vec_Info` — and define the ABI structs per the ngspice manual
(§19.6, ngspice-42 sharedspice.h):

    struct vector_info {
        char *name; int type; short flags;
        double *realdata; COMPLEX *compdata; int length;
    };

Verified live against the vendored libngspice.so.0.0.9 by test_ngsmoke.py.
"""
from __future__ import annotations

import ctypes
import os
import tempfile
from ctypes import (POINTER, PYFUNCTYPE, Structure, c_bool, c_char_p,
                    c_double, c_int, c_short, c_void_p)
from pathlib import Path
from typing import Any

VENDOR_DIR = Path(__file__).resolve().parent / "vendor" / "ngspice"
LIB_SO = VENDOR_DIR / "libngspice.so.0.0.9"


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


class NgSpiceSession:
    """One ngspice instance per test (~1 ms warm, ~15 ms cold)."""

    def __init__(self):
        if not LIB_SO.exists():
            raise NgSpiceError(
                f"vendored ngspice missing: {LIB_SO} — see vendor/README.md"
            )
        os.environ.setdefault("SPICE_LIB_DIR",
                              "/usr/lib/x86_64-linux-gnu/ngspice")
        self._lib = ctypes.CDLL(str(LIB_SO))
        self._captured: list[str] = []
        self._closed = True
        self._setup()

    # ---------------- callback wiring -------------------------------------

    def _setup(self):
        lib = self._lib
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
            if txt_p:
                this._captured.append(txt_p.decode("utf-8", "replace"))
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

    # ---------------- lifecycle ---------------------------------------------

    def open(self):
        rc = self._lib.ngSpice_Init(*self._cbs, None)
        if rc != 0:
            raise NgSpiceError(f"ngSpice_Init returned {rc}")
        self._closed = False
        return self

    def close(self):
        if not self._closed:
            try:
                self._lib.ngSpice_Command(b"quit")
            except Exception:
                pass
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
        return self.cmd(f"source {path}")

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
