"""Bench: a tethered session watched from the Mac (docs/bench-console.md) --
camera panes from tools/cam_relay.py on the left, the serial bridge's
telemetry (tools/board_bridge.py) on the right: balance strip charts, the
IMU, the servo table. bimo_gui's bench view, in the Qt console.

Watch-only: nothing here transmits, to the robot or to the bridge, beyond the
bridge's own subscribe lease (bench.py). The tab exists only when the console
was given --bench-telemetry or --cam-relay.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets

from . import bench
from .ui import (AMBER, BG, BRIGHT, DIM, FAINT, GREEN, MONO, RED, TEXT, group, hbox,
                 lab, mkplot, set_cell, table, vbox, wrap)

BINS = 300                                   # bimo_gui's chart width
X = np.arange(1, BINS + 1) * (bench.WINDOW_MS / 1000.0 / BINS) - bench.WINDOW_MS / 1000.0
SILENT_PEN = "#80504d"                       # bimo_gui's dead-bridge line colour
GREY = "#8c8c8c"
SERVO_COLS = ("id", "joint", "pos", "dzero t", "dzero deg", "load", "temp C", "V", "err",
              "age s")


class FrameView(QtWidgets.QWidget):
    """The newest frame, scaled to fit. A stale one is tinted red and says
    STALE: a frozen frame must never look live."""

    def __init__(self):
        super().__init__()
        self.img = None
        self.stale = False
        self.msg, self.msg_col = "", DIM
        self.setMinimumSize(240, 135)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Expanding)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        r = QtCore.QRectF(self.rect())
        p.fillRect(r, QtGui.QColor(BG))
        img = self.img
        if img is not None and not img.isNull() and img.width() and img.height():
            s = min(r.width() / img.width(), r.height() / img.height())
            w, h = img.width() * s, img.height() * s
            tgt = QtCore.QRectF((r.width() - w) / 2, (r.height() - h) / 2, w, h)
            p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
            p.drawImage(tgt, img)
            if self.stale:
                p.fillRect(tgt, QtGui.QColor(150, 30, 30, 110))
                f = QtGui.QFont(MONO, 22)
                f.setBold(True)
                p.setFont(f)
                p.setPen(QtGui.QColor("#ffd0cc"))
                p.drawText(tgt, QtCore.Qt.AlignmentFlag.AlignCenter, "STALE")
        elif self.msg:
            p.setPen(QtGui.QColor(self.msg_col))
            p.drawText(r.adjusted(12, 12, -12, -12),
                       QtCore.Qt.AlignmentFlag.AlignCenter | QtCore.Qt.TextFlag.TextWordWrap,
                       self.msg)
        p.setPen(QtGui.QColor("#343941"))
        p.drawRect(r.adjusted(0, 0, -1, -1))
        p.end()


class CamPaneBox(QtWidgets.QWidget):
    """One pane: the source combo (bimo_gui's), its state and age, the picture."""

    def __init__(self, tab, index: int):
        super().__init__()
        self.tab = tab
        self.index = index
        self.combo = QtWidgets.QComboBox()
        self.combo.addItem("(off)")
        for n in bench.CAM_NAMES:
            self.combo.addItem(bench.CAM_LABELS[n])
        self.combo.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.combo.setMinimumWidth(220)
        self.combo.activated.connect(self._picked)
        self.state = lab("", DIM, 12, True, True)
        self.view = FrameView()
        self._stream = None
        self._seq = -1
        self.setLayout(vbox(hbox(lab(f"pane {index}", DIM, 11), self.combo, 10, self.state,
                                 "stretch"), self.view, spacing=4))

    def _picked(self, i: int) -> None:
        self.tab.bench.select(self.index, None if i == 0 else bench.CAM_NAMES[i - 1])

    def refresh(self) -> None:
        b = self.tab.bench
        p = b.panes[self.index]
        want = 0 if p.sel is None else 1 + bench.CAM_NAMES.index(p.sel)
        if self.combo.currentIndex() != want:      # a scripted `cam` changed it
            self.combo.blockSignals(True)
            self.combo.setCurrentIndex(want)
            self.combo.blockSignals(False)
        st = p.state()
        age = p.age_s()
        if st == "STALE":
            self.state.setText(f"STALE  no frame for {age:.1f} s")
            self.state.setStyleSheet(f"color:{RED}; font-weight:700; font-family:{MONO}")
        elif st == "live":
            self.state.setText(f"live  {p.frames()} frames · {age:.1f} s")
            self.state.setStyleSheet(f"color:{GREEN}; font-weight:700; font-family:{MONO}")
        else:
            self.state.setText("waiting for the first frame …" if st == "waiting" else "off")
            self.state.setStyleSheet(f"color:{DIM}; font-family:{MONO}")
        v = self.view
        if p.stream is not self._stream:           # never show the previous camera
            self._stream, self._seq, v.img = p.stream, -1, None
        if p.stream is not None:
            img, seq = p.stream.latest()
            if seq != self._seq:
                self._seq, v.img = seq, img
        v.stale = st == "STALE"
        err = p.error()
        if err:
            v.msg, v.msg_col = err, AMBER
        elif not b.cam_host:
            v.msg, v.msg_col = "no camera relay: pass --cam-relay HOST[:PORT]", DIM
        elif p.sel is None:
            v.msg, v.msg_col = "pick a camera above", DIM
        else:
            v.msg, v.msg_col = "", DIM
        v.update()


class BenchTab(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.bench = bench.attach(app.ctl)
        self.bench.on_view = lambda name: app.select_tab("Bench" if name == "bench" else "Drive")
        app.installEventFilter(self)               # the window's close releases the cameras

        self.panes = [CamPaneBox(self, i) for i in range(bench.CAM_PANES)]
        cams = group("CAMERAS   tools/cam_relay.py · a camera opens on mira only while watched",
                     vbox(*self.panes, spacing=10))

        # -- bench telemetry --------------------------------------------------------
        self.bridge = lab("", DIM, 14, True, True)
        self.bridge_note = lab("", DIM, 12)
        self.board_quiet = lab("", AMBER, 12, True)
        self.bal = lab("", DIM, 14, True)
        self.stance = lab("", TEXT, 13, mono=True)
        self.gains = lab("", DIM, 12, mono=True)
        self.roll_na = lab("", AMBER, 12, True)
        self.corr = lab("", DIM, 12, mono=True)
        charts = QtWidgets.QGridLayout()
        charts.setVerticalSpacing(2)
        self.charts = []
        for i, (name, attr) in enumerate((("roll °", "s_roll"), ("err °", "s_err"),
                                          ("u °", "s_u"))):
            p = mkplot(70, (-bench.WINDOW_MS / 1000.0, 0), bottom=(i == 2))
            curve = p.plot(pen=pg.mkPen(BRIGHT, width=1.6))
            val = lab("", BRIGHT, 14, True, True)
            val.setFixedWidth(80)
            charts.addWidget(lab(name, DIM, mono=True), i, 0)
            charts.addWidget(p, i, 1)
            charts.addWidget(val, i, 2)
            self.charts.append((attr, p, curve, val))
        charts.setColumnStretch(1, 1)
        tele = group("BENCH TELEMETRY   tools/board_bridge.py · last 30 s, host arrival clock",
                     vbox(hbox(self.bridge, 10, self.bridge_note, "stretch"), self.board_quiet,
                          hbox(self.bal, 12, self.stance, "stretch", self.gains),
                          self.roll_na, charts, self.corr, spacing=4))

        # -- IMU --------------------------------------------------------------------
        self.up = lab("", TEXT, 13, mono=True)
        self.up_age = lab("", DIM, 12)
        self.roll = lab("", TEXT, 13, mono=True)
        self.level = lab("", DIM, 13, True)
        self.raw = lab("", DIM, 12, mono=True)
        imu = group("IMU", vbox(hbox(self.up, 10, self.up_age, "stretch"),
                                hbox(self.roll, 14, self.level, 14, self.raw, "stretch"),
                                spacing=4))

        # -- servos -----------------------------------------------------------------
        self.servos = table(list(SERVO_COLS), 0, stretch_col=1)
        self.servos.setMinimumHeight(26 * 12 + 34)
        self.servo_note = lab("", DIM, 11)
        self.servo_note.setWordWrap(True)
        sv = group("SERVOS", vbox(self.servos, self.servo_note, spacing=4))

        right = QtWidgets.QScrollArea()
        right.setWidgetResizable(True)
        right.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        right.setWidget(wrap(vbox(tele, imu, sv, "stretch", spacing=10)))
        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        split.setHandleWidth(6)
        split.addWidget(cams)
        split.addWidget(right)
        split.setSizes([460, 700])
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 3)
        self.setLayout(vbox(split, margins=(12, 12, 12, 12)))

    def eventFilter(self, obj, ev):
        if obj is self.app and ev.type() == QtCore.QEvent.Type.Close:
            self.bench.close()
        return False

    # -- the refresh -------------------------------------------------------------------
    def update_from(self, st, now: float) -> None:
        for p in self.panes:
            p.refresh()
        link = self.bench.link
        m = link.model
        live = m.live(now)
        self._bridge(link, m, now, live)
        self._balance(m, now, live)
        self._imu(m, now)
        self._servos(m, now, live)

    def _bridge(self, link, m, now, live):
        if not link.active:
            self.bridge.setText("no bridge")
            self.bridge.setStyleSheet(f"color:{DIM}; font-weight:700")
            self.bridge_note.setText("pass --bench-telemetry HOST[:PORT]  (tools/board_bridge.py "
                                     "on mira)" + (f" · {link.err}" if link.err else ""))
            self.board_quiet.setText("")
            return
        if m.datagrams == 0:
            text, col = f"waiting for the bridge at {link.target}", AMBER
            note = ""
        elif not live:
            text, col = f"STALE  bridge silent for {(now - m.last_rx_ms) / 1000.0:.1f} s", RED
            note = link.target
        else:
            text, col = f"live  {link.target}", GREEN
            note = (f"polling on ({m.hb_polls} queries)" if m.hb_polling
                    else "passive: only what the operator asks for")
        self.bridge.setText(text)
        self.bridge.setStyleSheet(f"color:{col}; font-weight:700; font-family:{MONO}")
        self.bridge_note.setText(f"{note}   {m.datagrams} datagrams, {m.rejected} refused")
        self.board_quiet.setText(f"the board has said nothing for {m.hb_rx_age:.0f} s"
                                 if live and m.hb_rx_age > 5.0 else "")

    def _balance(self, m, now, live):
        if m.state:
            on = m.state != "off"
            bad = m.state.startswith("FROZEN")
            self.bal.setText(f"balance {m.state}")
            # A silent bridge's ACTIVE is history: grey, not green. FROZEN
            # stays red -- an alarm does not fade because the link did.
            col = RED if bad else (GREEN if on and live else GREY)
            self.bal.setStyleSheet(f"color:{col}; font-weight:700")
            self.stance.setText(f"{m.stance} stance  {m.since:.1f} s" if on and m.stance else "")
        else:
            self.bal.setText("balance: no state yet (poll, or `balance` at the CLI)")
            self.bal.setStyleSheet(f"color:{DIM}")
            self.stance.setText("")
        self.gains.setText(f"kp {m.kp:.2f} ki {m.ki:.2f} clamp {m.clamp:.1f} "
                           f"abort {m.abort_deg:.1f}°" if m.have_gains else "")
        self.roll_na.setText("roll: no fresh IMU attitude" if m.roll_na else "")
        pen = pg.mkPen(BRIGHT if live else SILENT_PEN, width=1.6)
        for attr, plot, curve, val in self.charts:
            s = getattr(m, attr)
            first, out = s.resample(now, bench.WINDOW_MS, BINS)
            curve.setPen(pen)
            curve.setData(X[first:], np.asarray(out[first:], dtype=float))
            val.setText(f"{s.last():+.2f}" if s.count() else "—")
            val.setStyleSheet(f"color:{BRIGHT if live else FAINT}; font-size:14px; "
                              f"font-weight:700; font-family:{MONO}")
        if m.have_corr:
            self.corr.setText(f"correction {m.corr_side or '?'} ankle {m.ankle_deg:+.2f}° "
                              f"({m.ankle_ticks:+d} t)  hip {m.hip_deg:+.2f}° "
                              f"({m.hip_ticks:+d} t)  clamped {m.clamped}")
        else:
            self.corr.setText("")

    def _imu(self, m, now):
        if m.have_up:
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, m.up[2]))))
            self.up.setText(f"up  {m.up[0]:+.3f} {m.up[1]:+.3f} {m.up[2]:+.3f}   "
                            f"tilt {tilt:.1f}°")
            self.up_age.setText(f"{(now - m.t_up) / 1000.0:.0f} s ago")
        else:
            self.up.setText("up: none yet (`imu`)")
            self.up_age.setText("")
        self.roll.setText(f"roll {m.roll:+.2f}°" if m.have_roll else "")
        if m.have_level:
            self.level.setText("levelled" if m.levelled else "NOT levelled")
            self.level.setStyleSheet(f"color:{GREEN if m.levelled else AMBER}; font-weight:700")
        else:
            self.level.setText("")
        if m.have_raw:
            a, g = m.acc, m.graw
            self.raw.setText(f"a {a[0]:.2f} {a[1]:.2f} {a[2]:.2f} |a| {m.accmag:.2f}  "
                             f"g {g[0]:.3f} {g[1]:.3f} {g[2]:.3f}")
        else:
            self.raw.setText("")

    def _servos(self, m, now, live):
        ids = [i for i in bench.SERVO_ORDER if m.joints[i].have]
        ids += [i for i in range(bench.MAX_ID) if m.joints[i].have and i not in ids]
        t = self.servos
        if t.rowCount() != len(ids):
            t.setRowCount(len(ids))
        for r, sid in enumerate(ids):
            j = m.joints[sid]
            age = (now - j.t_ms) / 1000.0
            col = GREY if age > bench.OLD_S or not live else TEXT
            if j.have_cal:
                d = j.pos - j.zero
                dz, dd, dcol = f"{d:+d}", f"{d * 360.0 / 4096.0:+.1f}", col
            else:
                dz, dd, dcol = "-", "-", FAINT
            cells = ((str(j.id), col), (j.name or "-", col), (str(j.pos), col), (dz, dcol),
                     (dd, dcol), (str(j.load), col), (str(j.temp), col), (f"{j.volt:.1f}", col),
                     ((f"0x{j.err:02X}", RED) if j.err else ("ok", col)), (f"{age:.0f}", col))
            for c, (txt, cc) in enumerate(cells):
                set_cell(t, r, c, txt, cc)
        if not ids:
            self.servo_note.setText("no servo readings yet: `pos <id>` at the CLI, or run the "
                                    "bridge with --poll")
        else:
            self.servo_note.setText(
                "dzero = pos - the cal zero, raw servo ticks (4096/rev), direction not applied; "
                "'-' until a `cal show` is seen or --cal-from seeds it. Grey: older than 15 s, "
                "or the bridge is silent.")
