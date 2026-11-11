"""Drive: hold to move, what was sent beside what the policy saw, the
extended channels, speeds and send rate, and 30 s of telemetry (design §6)."""
from __future__ import annotations

import os
import sys

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import protocol  # noqa: E402  link/protocol.py: the generated command envelope

from . import core  # noqa: E402
from .ui import (AMBER, BRIGHT, CYAN, DIM, GREEN, Columns, Slider, btn, group, hbox, lab,
                 mkplot, vbox, wrap)

# (label, axis, grid row, grid col)
PAD_BUTTONS = (("fwd ↑", 0, 0, 1), ("← strafe", 2, 1, 0), ("back ↓", 1, 1, 1),
               ("strafe →", 3, 1, 2), ("↺ turn L", 4, 2, 0), ("turn R ↻", 5, 2, 2))
WINDOW_S = 30.0


class DriveTab(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        c = app.ctl
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        self.pad_btns = {}
        for text, axis, r, col in PAD_BUTTONS:
            b = btn(text, 96, h=40)
            b.pressed.connect(lambda a=axis: app.mouse_hold(a, True))
            b.released.connect(lambda a=axis: app.mouse_hold(a, False))
            grid.addWidget(b, r, col)
            self.pad_btns[axis] = b
        wire = QtWidgets.QGridLayout()
        wire.setHorizontalSpacing(10)
        wire.addWidget(lab("SENT", DIM, 11, True), 0, 1)
        wire.addWidget(lab("POLICY SAW", DIM, 11, True), 0, 2)
        self.sent, self.saw = {}, {}
        for i, (k, unit) in enumerate((("vx", "m/s"), ("vy", "m/s"), ("wz", "rad/s"),
                                       ("crouch", "")), start=1):
            wire.addWidget(lab(k, DIM, mono=True), i, 0)
            self.sent[k] = lab("", BRIGHT, 16, True, True)
            self.saw[k] = lab("", BRIGHT, 16, True, True)
            wire.addWidget(self.sent[k], i, 1)
            wire.addWidget(self.saw[k], i, 2)
            wire.addWidget(lab(unit, DIM), i, 3)
        self.source = lab("", GREEN, 11, True)
        self.wire_len = lab("", DIM, 11, mono=True)
        wire.addWidget(self.source, 5, 0, 1, 4)
        wire.addWidget(self.wire_len, 6, 0, 1, 4)
        wire.addWidget(lab("policy saw: the flight recorder's 7-wide command", DIM, 11), 7, 0, 1, 4)
        motion = group("MOTION   hold to move · release stands", hbox(grid, 14, wire, "stretch"))
        self.gamepad_box = QtWidgets.QVBoxLayout()
        self.gamepad_label = lab("no gamepad", DIM)
        self.gamepad_box.addWidget(self.gamepad_label)
        gp = group("GAMEPAD", self.gamepad_box)

        st = c.core.status(core.now_ms())
        # The ranges are the deployed run's envelope (protocol.py, generated):
        # the core clamps into it anyway, so a wider track would only snap back.
        env, fd = protocol.ENVELOPE, protocol.FOOT_D_MAX
        crouch_lo = min(env.crouch_min, 1.0)
        self.crouch = Slider("crouch ", crouch_lo, 1.0, st.crouch, "{:.2f}",
                             note=None if crouch_lo < 1.0 else "trained at 1 only")
        self._crouch_ok = crouch_lo < 1.0
        self.crouch.setEnabled(self._crouch_ok)
        self.fdx = Slider("foot dx", -fd, fd, st.foot_dx, "{:+.3f} m")
        self.fdz = Slider("foot dz", -fd, fd, st.foot_dz, "{:+.3f} m")
        self.lift = {}
        lift_row = [lab("lift   ", DIM)]
        grp = QtWidgets.QButtonGroup(self)
        for name, v in (("left", -1.0), ("none", 0.0), ("right", 1.0)):
            r = QtWidgets.QRadioButton(name)
            r.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            grp.addButton(r)
            r.toggled.connect(self._ext_changed)
            self.lift[v] = r
            lift_row.append(r)
        self.lift[0.0].setChecked(True)
        reset = self.reset_ext = btn("reset [0]", 110, h=32)
        reset.clicked.connect(lambda: c.fire("reset_ext"))
        for s in (self.crouch, self.fdx, self.fdz):
            s.changed.connect(self._ext_changed)
        extg = group("EXTENDED CHANNELS   off default → 24 B · reset → 14 B",
                     vbox(self.crouch, self.fdx, self.fdz,
                          hbox(*lift_row, "stretch", reset, spacing=10)))

        self.svx = Slider("vx max", 0.0, max(env.v_fwd_max, env.v_back_max), st.speed_vx,
                          "{:.2f} m/s")
        self.svy = Slider("vy max", 0.0, env.vy_max, st.speed_vy, "{:.2f} m/s")
        self.swz = Slider("wz max", 0.0, env.w_max, st.speed_wz, "{:.2f} rad/s")
        self.rate = Slider("send   ", 5.0, 100.0, app.rate_hz, "{:.0f} Hz", steps=95)
        for s in (self.svx, self.svy, self.swz):
            s.changed.connect(self._speeds_changed)
        self.rate.changed.connect(app.set_rate)
        envelope = lab(f"trained envelope: vx {env.v_fwd_min:.2f}–"
                       f"{env.v_fwd_max:.2f}, back {env.v_back_min:.2f}–{env.v_back_max:.2f} "
                       f"m/s · |vy| {env.vy_min:.2f}–{env.vy_max:.2f} · |wz| ≤ {env.w_max:.1f}; "
                       f"under a minimum the robot stands", DIM, 11)
        envelope.setWordWrap(True)
        spg = group("SPEEDS   live", vbox(self.svx, self.svy, self.swz, self.rate, envelope))

        self.p_vbat = mkplot(90, (-WINDOW_S, 0), bottom=False)
        self.p_tilt = mkplot(90, (-WINDOW_S, 0), (0, 15), bottom=False)
        self.p_tilt.addItem(pg.InfiniteLine(10, angle=0, pen=pg.mkPen(
            AMBER, width=1, style=QtCore.Qt.PenStyle.DashLine)))
        vlim = max(env.v_fwd_max, env.v_back_max, 0.5) + 0.1     # the trained top speed
        self.p_vx = mkplot(104, (-WINDOW_S, 0), (-vlim, vlim))
        self.c_vbat = self.p_vbat.plot(pen=pg.mkPen(BRIGHT, width=1.6))
        self.c_tilt = self.p_tilt.plot(pen=pg.mkPen(BRIGHT, width=1.6))
        self.c_vx = self.p_vx.plot(pen=pg.mkPen(CYAN, width=1.8, style=QtCore.Qt.PenStyle.DashLine))
        charts = QtWidgets.QGridLayout()
        charts.setVerticalSpacing(2)
        self.v_vbat, self.v_tilt, self.v_vx = (lab("", BRIGHT, 14, True, True) for _ in range(3))
        for i, (name, val, plot) in enumerate((("vbat", self.v_vbat, self.p_vbat),
                                               ("tilt °", self.v_tilt, self.p_tilt),
                                               ("vx sent", self.v_vx, self.p_vx))):
            charts.addWidget(lab(name, DIM, mono=True), i, 0)
            charts.addWidget(plot, i, 1)
            charts.addWidget(val, i, 2)
        # The robot does not estimate its speed: the beacon's vx_est is always
        # 0 (firmware/README.md), so the vx chart is what was sent.
        caption = lab("host receive time, last 30 s · tilt from the up vector; dashed "
                      "amber = the 10° abort line (first-policy-stand §5) · vx: dashed "
                      "cyan = sent (driver only); the robot's speed is not estimated "
                      "(vx_est is 0)", DIM, 11)
        caption.setWordWrap(True)
        charts.addWidget(caption, 3, 1)
        charts.setColumnStretch(1, 1)
        chg = group("TELEMETRY   the beacon, 10 Hz", charts)
        self.cols = Columns(wrap(vbox(motion, gp, spacing=10)),
                            wrap(vbox(extg, spg, spacing=10)), spacing=12)
        self.setLayout(vbox(self.cols, chg, "stretch", spacing=10, margins=(12, 12, 12, 12)))

    def _set_driver(self, driver: bool) -> None:
        """An observer sends nothing: every control that would is greyed (the
        safety bar's TAKE CONTROL is the way in), never hidden."""
        if getattr(self, "_driver", None) == driver:
            return
        self._driver = driver
        for w in (*self.pad_btns.values(), self.fdx, self.fdz, *self.lift.values(),
                  self.reset_ext, self.svx, self.svy, self.swz, self.rate):
            w.setEnabled(driver)
        self.crouch.setEnabled(driver and self._crouch_ok)
        self.source.setStyleSheet(f"color:{GREEN if driver else AMBER}; font-size:11px; "
                                  f"font-weight:700")
        tip = "" if driver else "observer: TAKE CONTROL in the safety bar to drive"
        for w in self.pad_btns.values():
            w.setToolTip(tip)

    def _ext_changed(self, *_):
        lift = next((v for v, r in self.lift.items() if r.isChecked()), 0.0)
        self.app.ctl.set_ext(self.crouch.value(), lift, self.fdx.value(), self.fdz.value())

    def _speeds_changed(self, *_):
        self.app.ctl.set_speeds(self.svx.value(), self.svy.value(), self.swz.value())

    def update_from(self, st, now: float) -> None:
        app = self.app
        held = st.held         # what the core holds: keys, mouse, pad or a script
        for axis, b in self.pad_btns.items():
            b.setObjectName("held" if held[axis] else "")
            b.style().unpolish(b)
            b.style().polish(b)
        if st.readonly:
            for k in self.sent:
                self.sent[k].setText("  —  ")
            self.wire_len.setText("observer: nothing on the wire")
        else:
            for k, v, f in (("vx", st.vx, "{:+.2f}"), ("vy", st.vy, "{:+.2f}"),
                            ("wz", st.wz, "{:+.2f}"), ("crouch", st.sent_crouch, "{:.2f}")):
                self.sent[k].setText(f.format(v))
            self.wire_len.setText(f"on the wire: {st.last_len} B "
                                  f"({'extended' if st.last_len == 24 else 'classic'})")
        saw = app.policy_saw()
        for k, i, f in (("vx", 0, "{:+.2f}"), ("vy", 1, "{:+.2f}"), ("wz", 2, "{:+.2f}"),
                        ("crouch", 3, "{:.2f}")):
            self.saw[k].setText(f.format(saw[i]) if saw is not None else "  —  ")
        self._set_driver(not st.readonly)
        if st.readonly:
            self.source.setText("observer: nothing here drives · TAKE CONTROL (safety bar)")
        else:
            self.source.setText(f"source: {app.drive_source()}")
        # Mirror the intent's extended channels, which `set` / reset change too.
        for s, v in ((self.crouch, st.crouch), (self.fdx, st.foot_dx), (self.fdz, st.foot_dz)):
            if abs(s.value() - v) > 1e-3 and not s.s.isSliderDown():
                s.set(v)
        lift = self.lift.get(round(st.lift))
        if lift is not None and not lift.isChecked():
            lift.blockSignals(True)
            lift.setChecked(True)
            lift.blockSignals(False)
        self.gamepad_label.setText(app.pad_detail())
        h = app.beacons
        if len(h):
            t = h.column("t") - now / 1000.0
            m = t >= -WINDOW_S
            t = t[m]
            vb = h.column("vbat")[m]
            self.c_vbat.setData(t, vb)
            if len(vb):
                lo, hi = float(vb.min()), float(vb.max())
                self.p_vbat.setYRange(lo - 0.1, hi + 0.1, padding=0)
                self.v_vbat.setText(f"{vb[-1]:5.2f} V")
            tilt = h.column("tilt")[m]
            self.c_tilt.setData(t, tilt)
            self.v_tilt.setText(f"{tilt[-1]:4.1f}°" if len(tilt) else "")
            if st.readonly:
                self.c_vx.setData([], [])
                self.v_vx.setText("  —  ")
            else:
                vx = h.column("vx_cmd")[m]
                self.c_vx.setData(t, vx)
                self.v_vx.setText(f"{vx[-1]:+.2f} m/s" if len(vx) else "")


class History:
    """A fixed-size ring of beacon samples, read as numpy columns."""

    def __init__(self, fields, cap=3000):
        self.fields = fields
        self.cap = cap
        self.a = np.zeros((cap, len(fields)))
        self.n = 0
        self.i = 0

    def __len__(self):
        return self.n

    def push(self, *vals):
        self.a[self.i] = vals
        self.i = (self.i + 1) % self.cap
        self.n = min(self.n + 1, self.cap)

    def column(self, name):
        k = self.fields.index(name)
        if self.n < self.cap:
            return self.a[: self.n, k]
        return np.concatenate((self.a[self.i:, k], self.a[: self.i, k]))
