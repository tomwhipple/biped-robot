"""ngspice co-simulation engine: the analog board in ngspice, the ICs'
digital state machines in Python, coupled at every accepted time step.

ngspice's shared-library API (sharedspice.h, ngspice 42–47) gives three hooks
this uses:

- ``EXTERNAL`` voltage sources: ngspice asks ``GetVSRCData(value*, time,
  name, ident, user)`` for their value whenever it evaluates them. The
  Python side answers from its current digital state (an IC's gate-driver
  enable, a button contact, an ESP32 pin).
- ``SendData(vecvaluesall*, ...)``: called once per accepted time point with
  every saved vector's value. The Python side reads pin voltages there and
  advances its state machines (comparators, timers, latches).
- ``GetSyncData(time, delta*, ...)``: lets the Python side shorten the next
  step so a scheduled event (a button press, a timer expiry) lands on a time
  point instead of inside a long step.

ngspice's shared mode cannot be re-initialised in one process (a second
``ngSpice_Init`` after ``quit`` is undefined, see ``ngsim.py``), so one
``Engine`` serves every run in its process, and callers that need many runs
use a process pool. The suite's own ``ngsim`` engine is never loaded in the
same process as this one.
"""
from __future__ import annotations

import ctypes
import math
import os
import re
import tempfile
from ctypes import (CFUNCTYPE, POINTER, Structure, c_bool, c_char_p,
                    c_double, c_int, c_void_p)
from pathlib import Path
from typing import Protocol

from pbsim import ngsim


class VecValues(Structure):
    _fields_ = [("name", c_char_p), ("creal", c_double), ("cimag", c_double),
                ("is_scale", c_bool), ("is_complex", c_bool)]


class VecValuesAll(Structure):
    _fields_ = [("veccount", c_int), ("vecindex", c_int),
                ("vecsa", POINTER(POINTER(VecValues)))]


SendChar = CFUNCTYPE(c_int, c_char_p, c_int, c_void_p)
SendStat = CFUNCTYPE(c_int, c_char_p, c_int, c_void_p)
ControlledExit = CFUNCTYPE(c_int, c_int, c_bool, c_bool, c_int, c_void_p)
SendData = CFUNCTYPE(c_int, POINTER(VecValuesAll), c_int, c_int, c_void_p)
SendInitData = CFUNCTYPE(c_int, c_void_p, c_int, c_void_p)
BGThreadRunning = CFUNCTYPE(c_int, c_bool, c_int, c_void_p)
GetVSRCData = CFUNCTYPE(c_int, POINTER(c_double), c_double, c_char_p, c_int,
                        c_void_p)
GetISRCData = CFUNCTYPE(c_int, POINTER(c_double), c_double, c_char_p, c_int,
                        c_void_p)
GetSyncData = CFUNCTYPE(c_int, c_double, POINTER(c_double), c_double, c_int,
                        c_int, c_int, c_void_p)


class Controller(Protocol):
    """What a run needs from the Python side."""

    def source(self, name: str, t: float) -> float:
        """Value of EXTERNAL source ``name`` (lower case) at time ``t``."""

    def step(self, t: float, v: dict[str, float]) -> None:
        """One accepted time point; ``v`` maps saved vector name → value."""

    def next_event(self, t: float) -> float:
        """Time of the next scheduled discontinuity after ``t`` (inf if none)."""


class SimError(RuntimeError):
    pass


class Engine:
    """One initialised libngspice for this process."""

    _instance: "Engine | None" = None

    @classmethod
    def get(cls) -> "Engine":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        path = ngsim.find_lib()
        if path is None:
            raise SimError("no libngspice for this host (macOS: brew install "
                           "libngspice, or set PBSIM_NGSPICE)")
        if path == str(ngsim.LIB_SO):
            os.environ.setdefault("SPICE_LIB_DIR",
                                  "/usr/lib/x86_64-linux-gnu/ngspice")
        self.lib = ctypes.CDLL(path)
        self.text: list[str] = []
        self.ctl: Controller | None = None
        self._names: list[str] | None = None
        self._error: BaseException | None = None
        self._min_step = 1e-12
        lib = self.lib
        lib.ngSpice_Command.argtypes = [c_char_p]
        lib.ngSpice_Command.restype = c_int
        this = self

        @SendChar
        def on_char(txt, _id, _u):
            if txt:
                this.text.append(txt.decode("utf-8", "replace"))
            return 0

        @SendStat
        def on_stat(_s, _id, _u):
            return 0

        @ControlledExit
        def on_exit(status, _imm, _quit, _id, _u):
            return status

        @SendData
        def on_data(pall, _count, _id, _u):
            if this._error is not None or this.ctl is None:
                return 0
            try:
                va = pall.contents
                n = va.veccount
                if this._names is None:
                    this._names = [va.vecsa[i].contents.name.decode().lower()
                                   for i in range(n)]
                names = this._names
                vals = {names[i]: va.vecsa[i].contents.creal for i in range(n)}
                t = vals.get("time", 0.0)
                this.ctl.step(t, vals)
            except BaseException as e:  # noqa: BLE001 - re-raised after run
                this._error = e
                lib.ngSpice_Command(b"bg_halt")
            return 0

        @SendInitData
        def on_init(_p, _id, _u):
            return 0

        @BGThreadRunning
        def on_bg(_r, _id, _u):
            return 0

        @GetVSRCData
        def on_vsrc(val, t, name, _id, _u):
            if this.ctl is None or this._error is not None:
                val[0] = 0.0
                return 0
            try:
                val[0] = float(this.ctl.source(name.decode().lower(), t))
            except BaseException as e:  # noqa: BLE001
                this._error = e
                val[0] = 0.0
            return 0

        @GetISRCData
        def on_isrc(val, t, name, _id, _u):
            return on_vsrc(val, t, name, _id, _u)

        @GetSyncData
        def on_sync(t, delta, _old, _redo, _id, _loc, _u):
            if this.ctl is None or this._error is not None:
                return 0
            try:
                te = this.ctl.next_event(t)
                if math.isfinite(te) and te > t + this._min_step:
                    if t + delta[0] > te:
                        delta[0] = max(te - t, this._min_step)
            except BaseException as e:  # noqa: BLE001
                this._error = e
            return 0

        self._cbs = (on_char, on_stat, on_exit, on_data, on_init, on_bg,
                     on_vsrc, on_isrc, on_sync)
        lib.ngSpice_Init.argtypes = [SendChar, SendStat, ControlledExit,
                                     SendData, SendInitData, BGThreadRunning,
                                     c_void_p]
        if lib.ngSpice_Init(on_char, on_stat, on_exit, on_data, on_init,
                            on_bg, None) != 0:
            raise SimError("ngSpice_Init failed")
        lib.ngSpice_Init_Sync.argtypes = [GetVSRCData, GetISRCData,
                                          GetSyncData, POINTER(c_int),
                                          c_void_p]
        ident = c_int(0)
        if lib.ngSpice_Init_Sync(on_vsrc, on_isrc, on_sync, ctypes.byref(ident),
                                 None) != 0:
            raise SimError("ngSpice_Init_Sync failed")

    def cmd(self, s: str) -> str:
        before = len(self.text)
        rc = self.lib.ngSpice_Command(s.encode())
        out = "".join(self.text[before:])
        if rc != 0:
            raise SimError(f"ngspice {s!r} rc={rc}: {out.strip()[-2000:]}")
        return out

    def run(self, deck: str, ctl: Controller, *, min_step: float = 1e-12,
            t_valid: float | None = None) -> str:
        """Load ``deck`` (which carries its own .tran and .save lines), run it
        with ``ctl`` attached, and return ngspice's console text.

        ``t_valid``: the run's real stop time. Decks run a little past it,
        because ngspice occasionally lands a hair short of its final time and
        then fails on a 1e-14 s step; a 'timestep too small' after
        ``t_valid`` leaves every point up to it intact and is accepted."""
        self.ctl = ctl
        self._names = None
        self._error = None
        self._min_step = min_step
        self.text.clear()
        with tempfile.TemporaryDirectory(prefix="pbfull-") as td:
            p = Path(td) / "board.cir"
            p.write_text(deck)
            try:
                out = self.cmd(f"source {p}")
                low = out.lower()
                for bad in ("error on line", "unknown subckt", "no such model",
                            "unable to find definition", "could not find"):
                    if bad in low:
                        raise SimError(f"deck did not load: {out[-3000:]}")
                out += self.cmd("run")
            finally:
                self.ctl = None
                self.cmd("remcirc")
                self.cmd("destroy all")
        if self._error is not None:
            raise self._error
        low = out.lower()
        m = re.search(r"timestep too small; time = ([0-9.eE+-]+)", out, re.I)
        if m and t_valid is not None and float(m.group(1)) >= t_valid:
            return out
        for bad in ("timestep too small", "singular matrix", "simulation(s) aborted",
                    "doanalyses: ", "transient solution failed"):
            if bad in low:
                raise SimError(f"ngspice failed: {out[-3000:]}")
        return out
