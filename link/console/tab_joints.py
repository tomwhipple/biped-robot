"""Joints: commanded vs measured for every bus joint (design §6 Joints).

One cell per bus joint, LEFT | RIGHT, limb by limb, proximal to distal: the
goal the robot wrote (dashed cyan; or the policy's nominal target) and the
position it read (solid white) on one robot-time axis, the latest values, the
error RMS and the load. Source: the flight recorder (link/console/flight.py),
one SYNC READ per tick, so every joint has the same stamp. A servo that missed
a read gets a red border and "reading held" (the firmware holds its last good
angle); an error bit in its status byte is red too. Unfitted joints are one
line. The sim twin's measured joints, when its records arrive, are a second,
amber trace. A detail plot of the selected joint adds the error and the load;
"table view" gives the same numbers as a table.

Drawn only while the tab is visible, at the tab rate, each trace decimated
to about a pixel per point (flight.decimate keeps the peaks), so 12 joints x
3 traces at 50 Hz over 2 min stay smooth.
"""
from __future__ import annotations

import math
import time
import warnings

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

from .flight import BUS_INDEX, decimate, status_text
from .ui import (AMBER, BG, BRIGHT, CYAN, DIM, FAINT, LINE, MONO, RED, SIMC, Flow, btn, mono_font,
                 group, hbox, lab, mkplot, restyle, set_cell, table, vbox, wrap)

LEGS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")
ARMS = ("shoulder", "elbow")
ORDER = [f"{s}_{p}" for p in LEGS + ARMS for s in "LR"] + ["neck_yaw"]
WINDOWS = (("5 s", 5.0), ("10 s", 10.0), ("30 s", 30.0), ("2 min", 120.0))
CELL_POINTS = 480          # per trace: about the cell plot's width in pixels
DETAIL_POINTS = 1600
TABLE_COLS = ("joint", "id", "goal", "meas", "sim meas", "|err| rms", "load", "speed",
              "current", "temp", "volt", "status")
TC = {n: i for i, n in enumerate(TABLE_COLS)}
DENSE_WINDOW_S = 30.0      # from here on a cell draws solid and without antialiasing
PX_PER_REDRAW = 2.0
RMS_S = 1.0                # the error RMS and mean load cover the newest second
ERR_WARN_DEG = 3.0
MEAS = "#f0f2f5"
LOADC = "#9b8cff"
DEFAULT_SELECTED = "L_knee"
FB_OK = 1                  # flightrec.JF_FB_OK


def _title(name: str) -> str:
    part = name.split("_", 1)[1] if name[1] == "_" else name
    return part.replace("_", " ")


def _newest(v):
    """The newest sample, or None if there is none or it is invalid."""
    return float(v[-1]) if len(v) and np.isfinite(v[-1]) else None


def segmented(labels, checked, on_change):
    """Exclusive tool buttons; on_change(index)."""
    lay = QtWidgets.QHBoxLayout()
    lay.setSpacing(0)
    grp = QtWidgets.QButtonGroup(lay)
    grp.setExclusive(True)
    for i, t in enumerate(labels):
        b = QtWidgets.QToolButton()
        b.setText(t)
        b.setCheckable(True)
        b.setChecked(i == checked)
        b.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        grp.addButton(b, i)
        lay.addWidget(b)
    grp.idClicked.connect(on_change)
    return lay, grp


class YRange:
    """A plot's y range with hysteresis: it widens at once to fit the data and
    narrows only when the data use well under half of it, so a cell does not
    rescale on every frame."""

    def __init__(self, plot_item, max_ticks: int = 2):
        self.pi = plot_item
        self.max_ticks = max_ticks
        self.cur = None

    def reset(self):
        self.cur = None

    def fit(self, lo: float, hi: float, minpad: float):
        pad = max(minpad, 0.15 * (hi - lo))
        want = (lo - pad, hi + pad)
        c = self.cur
        if c is None or lo < c[0] or hi > c[1] or (c[1] - c[0]) > 2.5 * (want[1] - want[0]):
            self.cur = want
            self.pi.setYRange(*want, padding=0)
            self.pi.getAxis("left").setTicks([nice_ticks(*want, self.max_ticks)])


def nice_ticks(lo: float, hi: float, n: int):
    """At most n round-valued ticks inside [lo, hi]: a 60 px plot has room for
    two labels, not pyqtgraph's five."""
    span = hi - lo
    if span <= 0:
        return []
    lo, hi = lo + 0.12 * span, hi - 0.12 * span     # a label at the very edge is clipped
    span = hi - lo
    raw = span / max(1, n)
    mag = 10 ** math.floor(math.log10(raw))
    step = next((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), 10 * mag)
    first = math.ceil(lo / step) * step
    ticks = []
    v = first
    while v <= hi + 1e-12 and len(ticks) < n + 1:
        ticks.append(v)
        v += step
    fmt = "{:.0f}" if step >= 1 else ("{:.1f}" if step >= 0.1 else "{:.2f}")
    return [(t, fmt.format(t + 0.0)) for t in ticks]


class JointCell(QtWidgets.QFrame):
    def __init__(self, tab, name: str):
        super().__init__()
        self.tab = tab
        self.name = name
        self.setObjectName("cell")
        title = _title(name)
        self.id_lab = lab("id –", DIM, 11, mono=True)
        self.val = lab("", size=12, mono=True, rich=True)
        self.stat = lab("", DIM, 11, mono=True)
        info = wrap(vbox(hbox(lab(title, "#e8e8e8", 13, True), "stretch", self.id_lab),
                         self.val, self.stat, spacing=0))
        info.setFixedWidth(218)
        self.plot = mkplot(None, (-5, 0), (-1, 1), bottom=False, left_w=34)
        self.pi = self.plot.getPlotItem()
        # No grid in the cells: in pyqtgraph it costs more to draw than the three
        # traces (measured offscreen: 12 cells, 35 -> 17 ms a frame).
        self.pi.showGrid(x=False, y=False)
        self.c_goal = self.plot.plot(connect="finite")
        self.c_sim = self.plot.plot(pen=pg.mkPen(SIMC, width=1), connect="finite")
        self.c_meas = self.plot.plot(pen=pg.mkPen(MEAS, width=1), connect="finite")
        self.dense = None
        self.set_dense(False)
        self.fault_line = pg.InfiniteLine(0, angle=90, pen=pg.mkPen(
            RED, width=1, style=QtCore.Qt.PenStyle.DotLine))
        self.fault_line.hide()
        self.pi.addItem(self.fault_line)
        self.yr = YRange(self.pi)
        self.plot.scene().sigMouseClicked.connect(lambda _ev: tab.select(name))
        self.full = wrap(hbox(info, self.plot, spacing=6, margins=(8, 4, 6, 4)))
        self.c_text = lab("", FAINT, 12)
        self.compact = wrap(hbox(lab(title, DIM, 13, True), 12, self.c_text, "stretch",
                                 margins=(10, 4, 10, 4)))
        lay = vbox(self.full, self.compact, spacing=0)
        self.setLayout(lay)
        self.mode = None
        self._style = None
        self.set_compact("waiting for flight records")

    def set_dense(self, dense: bool) -> None:
        """Long windows pack ~100 gait cycles into a cell: antialiased or
        dashed, that path costs ~10x a 5 s window's to draw, and a dash
        pattern is unreadable at that density anyway. Dense: solid goal, no
        antialiasing. 1 px cosmetic pens throughout (Qt's fast path)."""
        if dense == self.dense:
            return
        self.dense = dense
        style = QtCore.Qt.PenStyle.SolidLine if dense else QtCore.Qt.PenStyle.DashLine
        self.c_goal.setPen(pg.mkPen(CYAN, width=1, style=style))
        for c in (self.c_goal, self.c_sim, self.c_meas):
            c.opts["antialias"] = not dense
            c.curve.opts["antialias"] = not dense
            c.curve.update()

    def mousePressEvent(self, ev):
        self.tab.select(self.name)
        super().mousePressEvent(ev)

    def set_compact(self, text: str) -> None:
        if self.mode != "compact":
            self.mode = "compact"
            self.full.hide()
            self.compact.show()
            self.setFixedHeight(34)
        if self.c_text.text() != text:
            self.c_text.setText(text)
        self.set_border(False, False)

    def set_full(self) -> None:
        if self.mode != "full":
            self.mode = "full"
            self.compact.hide()
            self.full.show()
            self.setFixedHeight(66)

    def set_border(self, fault: bool, selected: bool) -> None:
        key = (fault, selected)
        if key == self._style:
            return
        self._style = key
        col = RED if fault else ("#5a95e8" if selected else LINE)
        width = 2 if (fault or selected) else 1
        self.setStyleSheet(f"QFrame#cell {{ border: {width}px solid {col}; "
                           f"border-radius: 6px; background: #1b1e23; }}")


class JointsTab(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        if not hasattr(app, "selected_joint"):
            app.selected_joint = None
        self.window_s = 5.0
        self.deg = True
        self.nominal = False
        self.paused_at = None
        self._drawn = 0.0
        w_lay, self._g_win = segmented([w for w, _ in WINDOWS], 0, self._set_window)
        u_lay, self._g_unit = segmented(["deg", "rad"], 0, self._set_unit)
        n_lay, self._g_nom = segmented(["goal written", "nominal"], 0, self._set_nominal)
        self.pause_btn = btn("pause", 80, h=30)
        self.pause_btn.clicked.connect(self._toggle_pause)
        self.table_btn = btn("table view", 110, h=30)
        self.table_btn.clicked.connect(self._toggle_table)
        self.legend = lab("", rich=True)
        self.legend.setWordWrap(True)        # its own row: it is the widest thing on the tab
        for lay in (w_lay, u_lay, n_lay):
            lay.setContentsMargins(0, 0, 0, 0)
        # The view controls wrap onto a second line on a narrow pane.
        top = vbox(Flow(wrap(hbox(lab("window", DIM), w_lay, spacing=8)), self.pause_btn,
                        wrap(u_lay), wrap(n_lay), spacing=14),
                   hbox(self.legend, self.table_btn, spacing=8), spacing=4)
        self.banner = lab("", AMBER, 12, True)
        self.banner.setWordWrap(True)

        self.cells = {n: JointCell(self, n) for n in ORDER}
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(5)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.addWidget(lab("LEFT", DIM, 12, True), 0, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
        grid.addWidget(lab("RIGHT", DIM, 12, True), 0, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        for r, part in enumerate(LEGS + ARMS, start=1):
            grid.addWidget(self.cells[f"L_{part}"], r, 0)
            grid.addWidget(self.cells[f"R_{part}"], r, 1)
        grid.addWidget(self.cells["neck_yaw"], len(LEGS + ARMS) + 1, 0, 1, 2)
        grid.setRowStretch(len(LEGS + ARMS) + 2, 1)
        grid_w = wrap(grid)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(grid_w)
        scroll.setStyleSheet("QScrollArea { background: transparent; }")
        grid_w.setStyleSheet("background: transparent;")

        self.table = table(list(TABLE_COLS), len(ORDER), stretch_col=TC["status"])
        for r, n in enumerate(ORDER):
            set_cell(self.table, r, 0, n)
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(scroll)
        self.stack.addWidget(self.table)

        self.gl = pg.GraphicsLayoutWidget()
        self.gl.setBackground(BG)
        self.gl.setFixedHeight(214)
        self.d_plots = [self.gl.addPlot(row=i, col=0) for i in range(3)]
        for pi in self.d_plots:
            pi.showGrid(x=True, y=True, alpha=0.12)
            pi.hideButtons()
            pi.setMenuEnabled(False)
            pi.setMouseEnabled(False, False)
            pi.setXRange(-self.window_s, 0, padding=0)
            for ax in ("left", "bottom"):
                pi.getAxis(ax).setTextPen(DIM)
                pi.getAxis(ax).setPen("#3a3f47")
                pi.getAxis(ax).setStyle(tickFont=mono_font(9))
            pi.getAxis("left").setWidth(48)
        a, e, ld = self.d_plots
        for pi in (e, ld):
            pi.setXLink(a)
        a.hideAxis("bottom")
        e.hideAxis("bottom")
        dash = QtCore.Qt.PenStyle.DashLine
        self.d_goal = a.plot(pen=pg.mkPen(CYAN, width=2, style=dash), connect="finite")
        self.d_sim = a.plot(pen=pg.mkPen(SIMC, width=1.6), connect="finite")
        self.d_meas = a.plot(pen=pg.mkPen(MEAS, width=2), connect="finite")
        self.d_err = e.plot(pen=pg.mkPen(AMBER, width=1.8), connect="finite")
        e.addItem(pg.InfiniteLine(0, angle=0, pen=pg.mkPen("#4a5059")))
        self.d_load = ld.plot(pen=pg.mkPen(LOADC, width=1.6), connect="finite")
        self.d_yr = [YRange(pi, 3) for pi in self.d_plots]
        a.setLabel("left", "pos °", color=DIM)        # axis labels: "<what> <unit>"
        e.setLabel("left", "err °", color=DIM)
        ld.setLabel("left", "load %", color=DIM)
        ld.setLabel("bottom", "s (robot time)", color=DIM)
        for pi in self.d_plots:
            self._axis_shown(pi, False)       # until the first data
        self.detail = group("DETAIL", vbox(self.gl))
        self.detail.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred,
                                  QtWidgets.QSizePolicy.Policy.Fixed)
        lay = vbox(top, self.banner, self.stack, self.detail, spacing=8,
                   margins=(12, 10, 12, 10))
        lay.setStretchFactor(self.stack, 1)
        self.setLayout(lay)
        self.banner.hide()

    # -- toggles -------------------------------------------------------------
    def _set_window(self, i: int) -> None:
        self.window_s = WINDOWS[i][1]
        for c in self.cells.values():
            c.pi.setXRange(-self.window_s, 0, padding=0)
            c.set_dense(self.window_s >= DENSE_WINDOW_S)
        self.d_plots[0].setXRange(-self.window_s, 0, padding=0)
        self._redraw()

    def _set_unit(self, i: int) -> None:
        self.deg = i == 0
        for c in self.cells.values():
            c.yr.reset()
        for y in self.d_yr:
            y.reset()
        u = "°" if self.deg else "rad"
        self.d_plots[0].setLabel("left", f"pos {u}", color=DIM)
        self.d_plots[1].setLabel("left", f"err {u}", color=DIM)
        self._redraw()

    def _set_nominal(self, i: int) -> None:
        self.nominal = i == 1
        self._redraw()

    def _toggle_pause(self) -> None:
        self.paused_at = None if self.paused_at is not None else time.monotonic()
        self.pause_btn.setText("paused" if self.paused_at is not None else "pause")
        restyle(self.pause_btn, "held" if self.paused_at is not None else "")
        self._redraw()

    def _toggle_table(self) -> None:
        on = self.stack.currentIndex() == 0
        self.stack.setCurrentIndex(1 if on else 0)
        self.table_btn.setText("cell view" if on else "table view")
        self._redraw()

    def select(self, name: str) -> None:
        self.app.selected_joint = name
        for y in self.d_yr:
            y.reset()
        self._redraw()

    def selected(self, fitted) -> str:
        s = getattr(self.app, "selected_joint", None)
        if s in BUS_INDEX:
            return s
        if DEFAULT_SELECTED in fitted or not fitted:
            return DEFAULT_SELECTED
        return fitted[0]

    def _redraw(self) -> None:
        st = getattr(self.app.ctl, "st", None) if hasattr(self.app, "ctl") else None
        self.update_from(st, 0.0, force=True)

    # -- the refresh -----------------------------------------------------------
    def update_from(self, st, now_ms: float, force: bool = False) -> None:
        fs = getattr(self.app, "flight", None)
        if self.paused_at is not None and not force:
            return
        mono = time.monotonic()
        if not force:
            # Redraw when the traces would have moved ~2 px: every refresh at
            # 5-10 s, a few times a second at 2 min.
            px = max(200, max((c.plot.width() for c in self.cells.values()
                               if c.mode == "full"), default=600))
            if mono - self._drawn < self.window_s / px * PX_PER_REDRAW:
                return
        self._drawn = mono
        if fs is None:
            self._show_banner("flight records are not available in this console")
            for c in self.cells.values():
                c.set_compact("no flight records")
            return
        now = self.paused_at if self.paused_at is not None else time.monotonic()
        lt = fs.latest("robot")
        fitted = lt["fitted"] if lt else []
        sim_on = fs.fresh("sim")
        rs = fs.stats("robot")
        self._legend(rs, sim_on, fs)
        self._banner(rs, lt)
        fb = fs.feedback("robot")
        k = 180.0 / math.pi if self.deg else 1.0
        ref = "nominal" if self.nominal else "goal"
        sel = self.selected(fitted)
        table_on = self.stack.currentIndex() == 1
        rows = {}
        for name, cell in self.cells.items():
            bid = fs.bus_id("robot", name)
            idt = f"id {bid}" if bid is not None else "id –"
            if lt is None:
                cell.set_compact(f"{idt} · waiting for flight records")
                continue
            if name not in fitted:
                cell.set_compact(f"{idt} · not fitted — no goal, no reading")
                continue
            d = self._joint(fs, name, ref, now, k, sim_on)
            f = fb.get(name, {})
            fault = bool(f.get("fault"))
            sbits = f.get("status") or 0
            d.update(fault=fault, status=sbits, fb=f, idt=idt)
            rows[name] = d
            if not table_on:
                self._draw_cell(cell, d, k, now, name == sel)
        if table_on:
            self._fill_table(rows, fitted, fs, lt is not None)
        self._draw_detail(fs, sel, ref, now, k, sim_on, sel in fitted, rows.get(sel))

    def _joint(self, fs, name, ref, now, k, sim_on) -> dict:
        w = self.window_s
        t, pos = fs.series("robot", f"{name}.pos", w, now)
        _, goal = fs.series("robot", f"{name}.{ref}", w, now)
        _, load = fs.series("robot", f"{name}.load", w, now)
        _, flags = fs.series("robot", f"{name}.flags", w, now)
        ok = np.isfinite(flags) & ((np.nan_to_num(flags).astype(int) & FB_OK) != 0)
        recent = t >= (t[-1] - RMS_S) if len(t) else np.zeros(0, bool)
        err = np.where(ok, pos - goal, np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            e = err[recent]
            e = e[np.isfinite(e)]
            rms = float(np.sqrt(np.mean(e ** 2))) * k if len(e) else None
            lr = np.abs(load[recent & ok])
            lr = lr[np.isfinite(lr)]          # the sim twin has no load: NaN, not 0
            mload = float(np.mean(lr)) if len(lr) else None
        if sim_on:
            ts, sim = fs.series("sim", f"{name}.pos", w, now)
        else:
            ts, sim = np.zeros(0), np.zeros(0)
        _, speed = fs.series("robot", f"{name}.speed", w, now)
        return dict(t=t, pos=pos * k, goal=goal * k, err=err * k, load=load, ok=ok,
                    ts=ts, sim=sim * k, rms=rms, mload=mload,
                    goal_now=_newest(goal), pos_now=_newest(pos), speed_now=_newest(speed),
                    sim_now=_newest(sim))

    def _fmt(self, v, k) -> str:
        if v is None:
            return "   —  "
        return f"{round(v * k, 1) + 0.0:+6.1f}°" if self.deg else f"{v:+6.3f}"

    def _draw_cell(self, cell: JointCell, d: dict, k: float, now: float, selected: bool):
        cell.set_full()
        cell.id_lab.setText(d["idt"])
        cell.set_border(d["fault"] or bool(d["status"]), selected)
        gtxt = "nom" if self.nominal else "goal"
        cell.val.setText(f"<span style='color:{CYAN}'>{self._fmt(d['goal_now'], k)}</span>&nbsp;"
                         f"<span style='color:#e8e8e8'>{self._fmt(d['pos_now'], k)}</span>"
                         f"<span style='color:{DIM}'>&nbsp;{gtxt}/meas</span>")
        f = d["fb"]
        if d["fault"]:
            txt = (f"FAULT {d['idt']} · reading held" if f.get("answered")
                   else f"FAULT {d['idt']} · no reply yet")
            cell.stat.setText(txt)
            cell.stat.setStyleSheet(f"color:{RED}; font-size:11px; font-weight:700; "
                                    f"font-family:{MONO}")
        elif d["status"]:
            cell.stat.setText(f"{d['idt']} status: {status_text(d['status'])}")
            cell.stat.setStyleSheet(f"color:{RED}; font-size:11px; font-weight:700; "
                                    f"font-family:{MONO}")
        else:
            unit = "°" if self.deg else " rad"
            rms = "  —" if d["rms"] is None else (f"{d['rms']:3.1f}" if self.deg
                                                   else f"{d['rms']:.3f}")
            load = " —" if d["mload"] is None else f"{d['mload']:2.0f}"
            warn = d["rms"] is not None and d["rms"] / k * 180.0 / math.pi > ERR_WARN_DEG
            cell.stat.setText(f"|err| {rms}{unit} rms · {load} % load")
            cell.stat.setStyleSheet(f"color:{AMBER if warn else DIM}; font-size:11px; "
                                    f"font-family:{MONO}")
        t = d["t"]
        cell.c_goal.setData(*decimate(t, d["goal"], CELL_POINTS))
        cell.c_meas.setData(*decimate(t, d["pos"], CELL_POINTS))
        if len(d["ts"]):
            cell.c_sim.setData(*decimate(d["ts"], d["sim"], CELL_POINTS))
        else:
            cell.c_sim.setData([], [])
        fs_t = f.get("fault_since")
        if d["fault"] and fs_t is not None and fs_t - now > -self.window_s:
            cell.fault_line.setValue(fs_t - now)
            cell.fault_line.show()
        else:
            cell.fault_line.hide()
        vals = [v[np.isfinite(v)] for v in (d["goal"], d["pos"], d["sim"])]
        vals = [v for v in vals if len(v)]
        if vals:
            lo = min(float(v.min()) for v in vals)
            hi = max(float(v.max()) for v in vals)
            cell.yr.fit(lo, hi, 2.0 if self.deg else 0.035)

    def _draw_detail(self, fs, sel, ref, now, k, sim_on, fitted, d=None) -> None:
        name = sel.replace("_", " ")
        gtxt = "nominal" if self.nominal else "goal"
        if not fitted:
            self.detail.setTitle(f"DETAIL   {name} · not fitted · click any cell")
            for c in (self.d_goal, self.d_meas, self.d_sim, self.d_err, self.d_load):
                c.setData([], [])
            for pi in self.d_plots:
                self._axis_shown(pi, False)
            return
        self.detail.setTitle(f"DETAIL   {name} · {gtxt} / measured · error (meas − {gtxt}) · "
                             f"load · click any cell · highlighted in the 3D view")
        d = d or self._joint(fs, sel, ref, now, k, sim_on)
        t = d["t"]
        self.d_goal.setData(*decimate(t, d["goal"], DETAIL_POINTS))
        self.d_meas.setData(*decimate(t, d["pos"], DETAIL_POINTS))
        self.d_err.setData(*decimate(t, d["err"], DETAIL_POINTS))
        self.d_load.setData(*decimate(t, np.where(d["ok"], d["load"], np.nan), DETAIL_POINTS))
        if len(d["ts"]):
            self.d_sim.setData(*decimate(d["ts"], d["sim"], DETAIL_POINTS))
        else:
            self.d_sim.setData([], [])
        minpad = 2.0 if self.deg else 0.035
        for yr, arrs, mp in ((self.d_yr[0], (d["goal"], d["pos"], d["sim"]), minpad),
                             (self.d_yr[1], (d["err"],), minpad / 2),
                             (self.d_yr[2], (np.where(d["ok"], d["load"], np.nan),), 5.0)):
            vals = [v[np.isfinite(v)] for v in arrs]
            vals = [v for v in vals if len(v)]
            self._axis_shown(yr.pi, bool(vals))
            if vals:
                yr.fit(min(float(v.min()) for v in vals), max(float(v.max()) for v in vals), mp)

    @staticmethod
    def _axis_shown(pi, shown: bool) -> None:
        """A plot's y-axis ticks and label only once it has data: the empty
        axis is a crowd of overlapping labels in a 70 pt plot."""
        ax = pi.getAxis("left")
        ax.setStyle(showValues=shown)
        ax.showLabel(shown)

    def _fill_table(self, rows, fitted, fs, have) -> None:
        k = 180.0 / math.pi if self.deg else 1.0
        for r, name in enumerate(ORDER):
            bid = fs.bus_id("robot", name)
            set_cell(self.table, r, 1, "–" if bid is None else str(bid), DIM)
            d = rows.get(name)
            if d is None:
                text = "not fitted" if have else "no records"
                for c in range(2, len(TABLE_COLS)):
                    set_cell(self.table, r, c, text if c == TC["status"] else "", FAINT)
                set_cell(self.table, r, 0, name, FAINT)
                continue
            f = d["fb"]
            rms, sp, ml = d["rms"], d["speed_now"], d["mload"]
            cur, temp, volt = f.get("current_ma"), f.get("temp_c"), f.get("voltage_v")
            if d["fault"]:
                status, scol = "FAULT · reading held", RED
            elif f.get("simulated"):
                status, scol = "— (sim: no servo status)", DIM
            elif d["status"]:
                status, scol = status_text(d["status"]), RED
            else:
                status, scol = "ok", "#2fa35a"
            vals = {
                "joint": (name, BRIGHT),
                "goal": (self._fmt(d["goal_now"], k), CYAN),
                "meas": (self._fmt(d["pos_now"], k), BRIGHT),
                "sim meas": ("—" if d["sim_now"] is None else self._fmt(d["sim_now"], k), SIMC),
                "|err| rms": ("—" if rms is None else
                              (f"{rms:.1f}°" if self.deg else f"{rms:.3f}"), BRIGHT),
                "load": ("—" if ml is None else f"{ml:.0f} %", BRIGHT),
                "speed": ("—" if sp is None else
                          (f"{sp * k:+.0f}°/s" if self.deg else f"{sp:+.2f} rad/s"), BRIGHT),
                "current": ("—" if cur is None else f"{cur:.0f} mA", BRIGHT),
                "temp": ("—" if temp is None else f"{temp:.0f} °C", BRIGHT),
                "volt": ("—" if volt is None else f"{volt:.1f} V", BRIGHT),
                "status": (status, scol),
            }
            for col, (text, colour) in vals.items():
                set_cell(self.table, r, TC[col], text, colour)

    def _legend(self, rs: dict, sim_on: bool, fs=None) -> None:
        g = "nominal" if self.nominal else "goal"
        # A console driving the sim gets the twin's records on the robot's
        # port: say so, or its traces read as the robot's.
        src = (f"<span style='color:{SIMC}'>SIM TWIN records "
               f"({rs.get('fw') or 'sil_twin'}): no servo load, current, V, temp, "
               f"status</span> · " if rs.get("simulated") else "flight recorder ")
        rate = f"{rs['rate_hz']:.0f} Hz" if rs["seen"] and not rs["stale"] else "—"
        sim = (f"&nbsp;&nbsp;&nbsp;<span style='color:{SIMC}'>—— sim twin</span>"
               if sim_on else "")
        mark = "——" if self.window_s >= DENSE_WINDOW_S else "– –"
        txt = (f"<span style='color:{CYAN}'>{mark} {g}</span>&nbsp;&nbsp;&nbsp;"
               f"<span style='color:{MEAS}'>—— measured</span>{sim}&nbsp;&nbsp;&nbsp;"
               f"<span style='color:{DIM}'>{src}{rate} · one SYNC READ per tick · "
               f"robot time · rms over {RMS_S:.0f} s</span>")
        if self.legend.text() != txt:
            self.legend.setText(txt)

    def _banner(self, rs: dict, lt) -> None:
        msg = ""
        if self.paused_at is not None:
            msg = (f"PAUSED {time.monotonic() - self.paused_at:4.0f} s ago — the traces are "
                   f"frozen; the store keeps the newest ~2.7 min")
        elif not rs["port"]:
            msg = "flight records off (--flight-port 0)"
        elif not rs["bound"]:
            msg = f"flight records: {rs['error']}"
        elif not rs["seen"]:
            msg = (f"no flight records yet on :{rs['port']} — mira's collector forwards them "
                   f"(tools/flight_collector.py --forward <this host>:{rs['port']})")
        elif rs["stale"]:
            msg = (f"NO RECORDS for {rs['age_s']:.0f} s — the traces are the last received, "
                   f"sliding out of the window")
        elif not rs["calibrated"]:
            msg = "no HELLO yet: angles wait for the robot's calibration (1 Hz)"
        else:
            h = self.app.flight.hello("robot")
            if h is not None and not h["cal_ok"]:
                msg = "the robot reports NO calibration loaded: angles use its default zeros"
        self._show_banner(msg)

    def _show_banner(self, msg: str) -> None:
        if msg:
            if self.banner.text() != msg:
                self.banner.setText(msg)
            self.banner.show()
        else:
            self.banner.hide()
