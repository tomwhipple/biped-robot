"""Telemetry: strip charts on a real time axis, the link table and the
servos' feedback blocks (design §6 Telemetry).

Charts (the last 60 s, seconds before now): pack, tilt with the 10° abort
line, gyro, the control tick's work time against its 20 ms budget, the
beacon's age against the 250 ms stale line, flight-recorder holes. Flight
records (link/console/flight.py) are preferred where they carry the value --
50 Hz, and they need no driver -- and the beacon fills in otherwise; each
chart says which it shows. The link table has loop_late and the robot's own
tick budget from its HELLO. Per fitted servo: temperature, load, current,
voltage, status from the flight recorder's feedback block.

An observer shows no command traces: a flat zero would read as "told to
stand". Room is kept for the power board's block (#109, PWR_ESTOP #106).

always_update: the beacon's age is sampled every refresh, hidden or not, so
its chart has history when the tab is opened; drawing happens only while
the tab is visible.
"""
from __future__ import annotations

import math
import warnings

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

from .flight import decimate, status_text
from .tab_drive import History
from .tab_joints import ORDER
from .ui import (AMBER, BRIGHT, DIM, FAINT, GREEN, RED, SIMC, group, hbox, lab, mkplot,
                 set_cell, table, vbox, wrap)

WINDOW_S = 60.0
CHART_POINTS = 700
TILT_ABORT_DEG = 10.0        # first-policy-stand.md §5
TICK_BUDGET_MS = 20.0        # the 50 Hz control tick
BEACON_STALE_MS = 250.0      # the robot's watchdog: link stale -> STAND (control-channel.md)
BEACON_AGE_CLIP_MS = 1000.0
TEMP_WARN_C, TEMP_HOT_C = 45, 60   # display colours only
LOAD_MEAN_S = 1.0


class Chart:
    def __init__(self, name, unit, yr, ref=None, color=BRIGHT, bottom=False, fixed=False):
        self.unit = unit
        self.yr = yr
        self.fixed = fixed
        self.plot = mkplot(None, (-WINDOW_S, 0), yr, bottom=bottom, left_w=44)
        self.plot.setMinimumHeight(110 if bottom else 86)
        self.curve = self.plot.plot(pen=pg.mkPen(color, width=1.6), connect="finite")
        if ref is not None:
            self.plot.addItem(pg.InfiniteLine(ref, angle=0, pen=pg.mkPen(
                AMBER, width=1, style=QtCore.Qt.PenStyle.DashLine)))
        self.name = lab(name, DIM, mono=True)
        self.src = lab("", FAINT, 10, mono=True)
        self.value = lab("", BRIGHT, 13, True, True)
        self.value.setMinimumWidth(110)

    def set(self, t, v, src: str, fmt: str = "{:.2f}", note: str | None = None) -> None:
        if len(t):
            self.curve.setData(*decimate(t, v, CHART_POINTS))
        else:
            self.curve.setData([], [])
        self.src.setText(src)
        f = v[np.isfinite(v)] if len(v) else v
        if note is not None:
            self.value.setText(note)
        elif len(f):
            self.value.setText(f"{fmt.format(float(f[-1]))} {self.unit}")
        else:
            self.value.setText("—")
        if not self.fixed and len(f):
            lo, hi = float(f.min()), float(f.max())
            lo = min(lo, self.yr[0])
            hi = max(hi, self.yr[1])
            self.plot.getPlotItem().setYRange(lo, hi, padding=0.06)


class TelemetryTab(QtWidgets.QWidget):
    always_update = True

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.ages = History(("t", "age"), cap=4000)
        self.c_vbat = Chart("vbat", "V", (11.0, 12.6))
        self.c_tilt = Chart("tilt", "°", (0, 12), ref=TILT_ABORT_DEG)
        self.c_gyro = Chart("gyro |ω|", "rad/s", (0, 0.4))
        self.c_work = Chart("tick work", "ms", (0, 22), ref=TICK_BUDGET_MS)
        self.c_age = Chart("beacon age", "ms", (0, 400), ref=BEACON_STALE_MS)
        self.c_holes = Chart("rec holes", "/s", (0, 5), color=SIMC, bottom=True)
        self.charts = (self.c_vbat, self.c_tilt, self.c_gyro, self.c_work, self.c_age,
                       self.c_holes)
        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(3)
        for i, c in enumerate(self.charts):
            grid.addWidget(wrap(vbox(c.name, c.src, c.value, "stretch", spacing=0)), i, 0)
            grid.addWidget(c.plot, i, 1)
            if c is not self.c_vbat:
                c.plot.setXLink(self.c_vbat.plot)
        grid.setColumnStretch(1, 1)
        refs = lab(f"seconds before now · flight records are spaced by the robot's clock · "
                   f"dashed amber: {TILT_ABORT_DEG:.0f}° abort line · {TICK_BUDGET_MS:.0f} ms "
                   f"tick budget · {BEACON_STALE_MS:.0f} ms link stale (STAND)", DIM, 11)
        refs.setWordWrap(True)
        left_lay = vbox(grid, refs)
        left_lay.setStretchFactor(grid, 1)
        left = group(f"ROBOT   last {WINDOW_S:.0f} s", left_lay)

        # The joint names set the first column; the slack goes to status.
        self.servos = table(["joint", "id", "temp", "load", "current", "volt", "status"], 0,
                            stretch_col=6)
        self._servo_rows = None
        right_servos = group("SERVOS   flight recorder feedback block, per fitted joint",
                             vbox(self.servos))
        self.link_rows = ("commands sent", "beacon", "flight recorder", "sim records",
                          "robot clock", "loop", "ctrl tick (last)", "recorder")
        self.link = table(["link", "value"], len(self.link_rows), stretch_col=1,
                          height=26 * len(self.link_rows) + 34)
        for r, k in enumerate(self.link_rows):
            set_cell(self.link, r, 0, k)
        self.power = lab("no source yet: the power board's block (#109) and PWR_ESTOP (#106) "
                         "go here", FAINT, 12)
        self.power.setWordWrap(True)
        self.obs = lab("Observer: no command traces — a flat zero would read as "
                       "'told to stand'.", DIM, 11)
        self.obs.setWordWrap(True)
        right = vbox(right_servos, group("LINK", vbox(self.link)),
                     group("POWER BOARD", vbox(self.power)), self.obs, "stretch", spacing=10)
        lay = hbox(left, right, spacing=12, margins=(12, 12, 12, 12))
        lay.setStretch(0, 2)          # the 60 s traces get the width, the tables the rest
        lay.setStretch(1, 1)
        self.setLayout(lay)

    # -- the refresh -----------------------------------------------------------
    def update_from(self, st, now_ms: float) -> None:
        if st is not None:
            self.ages.push(now_ms / 1000.0, st.tlm_age_ms if st.have_tlm else np.nan)
        if not self.isVisible():
            return
        fs = getattr(self.app, "flight", None)
        self._charts(st, now_ms, fs)
        self._servo_table(fs)
        self._link_table(st, fs)
        self.obs.setVisible(st is None or bool(st.readonly))

    def _charts(self, st, now_ms, fs) -> None:
        nows = now_ms / 1000.0
        h = self.app.beacons
        bt = h.column("t") - nows if len(h) else np.zeros(0)
        bm = bt >= -WINDOW_S
        bt = bt[bm]

        def beacon(col):
            return bt, h.column(col)[bm] if len(h) else np.zeros(0)

        def flight(name):
            if fs is None:
                return np.zeros(0), np.zeros(0)
            t, v = fs.series("robot", name, WINDOW_S)
            if not np.isfinite(v).any():
                return np.zeros(0), np.zeros(0)
            return t, v

        for chart, fname, bcol in ((self.c_vbat, "vbat", "vbat"),
                                   (self.c_tilt, "tilt_deg", "tilt")):
            t, v = flight(fname)
            if len(t):
                chart.set(t, v, "flight rec")
            elif len(bt):
                chart.set(*beacon(bcol), "beacon")
            else:
                chart.set(np.zeros(0), np.zeros(0), "no data")
        for chart, fname, fmt in ((self.c_gyro, "gyro", "{:.2f}"),
                                  (self.c_work, "work_ms", "{:.2f}")):
            t, v = flight(fname)
            chart.set(t, v, "flight rec" if len(t) else "no flight records", fmt)
        at = self.ages.column("t") - nows if len(self.ages) else np.zeros(0)
        av = self.ages.column("age") if len(self.ages) else np.zeros(0)
        m = at >= -WINDOW_S
        at, av = at[m], np.minimum(av[m], BEACON_AGE_CLIP_MS)
        note = None
        if st is not None and not st.have_tlm:
            note = "none"
        elif st is not None and st.link_lost:
            note = f"LOST {st.tlm_age_ms / 1000.0:.1f} s"
        self.c_age.set(at, av, "beacon", "{:.0f}", note)
        if fs is not None and fs.stats("robot")["seen"]:
            t, v = fs.series("robot", "holes", WINDOW_S)
            self.c_holes.set(t, v, "flight rec", "{:.0f}")
        else:
            self.c_holes.set(np.zeros(0), np.zeros(0), "no flight records")

    def _servo_table(self, fs) -> None:
        lt = fs.latest("robot") if fs is not None else None
        fitted = [n for n in ORDER if lt and n in lt["fitted"]]
        if fitted != self._servo_rows:
            self._servo_rows = fitted
            self.servos.setRowCount(max(1, len(fitted)))
            self.servos.setFixedHeight(26 * max(1, len(fitted)) + 34)
            self.servos.setSpan(0, 0, 1, 1 if fitted else 7)   # the empty-state text spans
            for r, n in enumerate(fitted):
                set_cell(self.servos, r, 0, n)
        if not fitted:
            set_cell(self.servos, 0, 0, "no flight records" if lt is None else "none fitted",
                     FAINT)
            for c in range(1, 7):
                set_cell(self.servos, 0, c, "")
            return
        fb = fs.feedback("robot")
        for r, n in enumerate(fitted):
            f = fb.get(n, {})
            bid = fs.bus_id("robot", n)
            set_cell(self.servos, r, 1, "–" if bid is None else str(bid), DIM)
            temp = f.get("temp_c")
            tcol = (RED if temp is not None and temp >= TEMP_HOT_C else
                    AMBER if temp is not None and temp >= TEMP_WARN_C else BRIGHT)
            set_cell(self.servos, r, 2, "—" if temp is None else f"{temp:.0f} °C", tcol)
            t, load = fs.series("robot", f"{n}.load", LOAD_MEAN_S)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                ml = float(np.nanmean(np.abs(load))) if len(load) else math.nan
            set_cell(self.servos, r, 3, "—" if not math.isfinite(ml) else f"{ml:4.0f} %", BRIGHT)
            cur, volt = f.get("current_ma"), f.get("voltage_v")
            set_cell(self.servos, r, 4, "—" if cur is None else f"{cur:4.0f} mA", BRIGHT)
            set_cell(self.servos, r, 5, "—" if volt is None else f"{volt:.1f} V", BRIGHT)
            if f.get("fault"):
                set_cell(self.servos, r, 6, "FAULT · held" if f.get("answered") else "no reply",
                         RED)
            elif f.get("status"):
                set_cell(self.servos, r, 6, status_text(f["status"]), RED)
            elif not f.get("answered"):
                set_cell(self.servos, r, 6, "no reading yet", DIM)
            else:
                set_cell(self.servos, r, 6, "ok", GREEN)

    def _link_table(self, st, fs) -> None:
        app = self.app
        vals = {}
        if st is None:
            vals["commands sent"] = ("—", DIM)
            vals["beacon"] = ("—", DIM)
        else:
            vals["commands sent"] = (("observer: none · HOLD / E-STOP are out of band", AMBER)
                                     if st.readonly else
                                     (f"{app.rate_hz:.1f} Hz · {st.last_len} B · seq {st.seq}",
                                      BRIGHT))
            if not st.have_tlm:
                vals["beacon"] = ("none" + (": the robot beacons only to its commander"
                                            if st.readonly else ""), DIM)
            elif st.link_lost:
                vals["beacon"] = (f"LINK LOST {st.tlm_age_ms / 1000.0:.1f} s ago", RED)
            else:
                vals["beacon"] = (f"{st.rate_hz:.1f} Hz · lag {st.lag} · bad {st.tlm_bad} · "
                                  f"age {st.tlm_age_ms:.0f} ms", BRIGHT)
        for key, src in (("flight recorder", "robot"), ("sim records", "sim")):
            if fs is None:
                vals[key] = ("not available", DIM)
                continue
            s = fs.stats(src)
            if not s["port"]:
                vals[key] = ("off (port 0)", DIM)
            elif not s["bound"]:
                vals[key] = (s["error"], RED)
            elif not s["seen"]:
                vals[key] = (f":{s['port']} nothing received", DIM)
            else:
                txt = (f"{s['rate_hz']:.1f} Hz · holes {s['network_holes']} (network) · "
                       f"drops {s['robot_drops']} (robot) · undecodable {s['undecodable']} · "
                       f"reboots {s['reboots']}")
                if s["stale"]:
                    txt = f"NO RECORDS for {s['age_s']:.0f} s · " + txt
                vals[key] = (txt, AMBER if s["stale"] else BRIGHT)
        h = fs.hello("robot") if fs is not None else None
        if h is None:
            seen = fs is not None and fs.src["robot"].hello is not None
            for k in ("robot clock", "ctrl tick (last)", "recorder"):
                vals[k] = ("no HELLO for 3 s" if seen else "no HELLO yet", AMBER if seen else DIM)
            loop_hello = ""
        else:
            sk = fs.stats("robot")["skew_ms"]
            vals["robot clock"] = (("NTP synced" if h["utc_synced"] else "NTP NOT synced")
                                   + (f" · Δ {sk:+.0f} ms vs this host (incl. transit)"
                                      if sk is not None else ""),
                                   BRIGHT if h["utc_synced"] else AMBER)
            vals["ctrl tick (last)"] = (
                " · ".join(f"{k[3:]} {h[k] / 1000.0:.1f}" for k in
                           ("us_read", "us_imu", "us_obs", "us_net", "us_write", "us_other"))
                + " ms", BRIGHT)
            vals["recorder"] = (f"{'on' if h['enabled'] else 'OFF'} · produced {h['produced']} · "
                                f"sent {h['sent']} · drops {h['drop_ring']} ring / "
                                f"{h['drop_net']} net · push max {h['max_push_ctrl_us']} µs",
                                BRIGHT)
            loop_hello = (f" · worst tick {h['ctrl_worst_us'] / 1000.0:.1f} ms · "
                          f"overruns {h['ctrl_late']}")
        late = (f"late {st.loop_late_pct} % (beacon)" if st is not None and st.have_tlm
                else "late — (no beacon)")
        vals["loop"] = (late + loop_hello, BRIGHT)
        for r, k in enumerate(self.link_rows):
            text, col = vals.get(k, ("", DIM))
            set_cell(self.link, r, 1, text, col)
