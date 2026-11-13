"""The gamepad's widgets: the live pad on the Drive tab and the mapping with
its settings on the Setup tab (design §6, mockups' pad_widget and Setup
GAMEPAD group). Gamepad.install puts them in place; each repaints itself
while visible, from the Gamepad's state, and changes nothing but the pad's
own timeout and dead zone."""
from __future__ import annotations

from PyQt6 import QtCore, QtGui, QtWidgets

from .ui import (AMBER, BLUE, BRIGHT, DIM, FAINT, GREEN, RED, hbox, lab, mono_font, set_cell, table,
                 vbox)

REFRESH_MS = 66


class _Ticking(QtWidgets.QWidget):
    def __init__(self, pad):
        super().__init__()
        self.pad = pad
        self._t = QtCore.QTimer(self)
        self._t.timeout.connect(self._tick)
        self._t.start(REFRESH_MS)

    def _tick(self):
        if self.isVisible():
            self.refresh()

    def refresh(self):
        self.update()


class PadView(_Ticking):
    """Live sticks, the dead-man, E-STOP, and what a lost pad does."""

    def __init__(self, pad):
        super().__init__(pad)
        self.setFixedHeight(150)
        self.setMinimumWidth(500)

    def refresh(self):
        g = self.parentWidget()
        if isinstance(g, QtWidgets.QGroupBox):
            t = f"GAMEPAD   {self.pad.title()}"
            if g.title() != t:
                g.setTitle(t)
        self.update()

    def paintEvent(self, _):
        pad = self.pad
        live = pad.pad is not None
        rb, b = pad.labels
        a = pad.axes
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.setFont(mono_font(10))
        for cx, label, (x, y) in ((60, "L  vx / vy", (a["leftx"], a["lefty"])),
                                  (190, "R  wz", (a["rightx"], a["righty"]))):
            c = QtCore.QPointF(cx, 62)
            p.setPen(QtGui.QPen(QtGui.QColor("#3a3f47"), 2))
            p.setBrush(QtGui.QColor("#121418"))
            p.drawEllipse(c, 40, 40)
            p.setPen(QtGui.QPen(QtGui.QColor("#2c3036"), 1, QtCore.Qt.PenStyle.DashLine))
            p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            r = max(3.0, 34 * pad.dead_zone)
            p.drawEllipse(c, r, r)
            if live:
                p.setPen(QtCore.Qt.PenStyle.NoPen)
                p.setBrush(QtGui.QColor(BLUE if pad.driving else "#4a5059"))
                p.drawEllipse(c + QtCore.QPointF(x * 34, y * 34), 11, 11)
            p.setPen(QtGui.QColor(DIM))
            p.drawText(QtCore.QPointF(cx - 34, 122), label)
        if pad.driving:
            dm = (f"{rb}  dead-man held", GREEN, True)
        elif pad.gated:
            dm = (f"{rb}  held: sim only", AMBER, False)
        elif pad.deadman and pad.latched:
            dm = (f"{rb}  released: let go", AMBER, False)
        else:
            dm = (f"{rb}  dead-man", GREEN if live else FAINT, False)
        es = (f"{b} (east)  E-STOP", RED if live else FAINT, live and pad.estop_down)
        for (y, (text, col, on)) in ((14, dm), (48, es),
                                     (82, ("arm / reset: console only", FAINT, False))):
            r = QtCore.QRectF(268, y, 220, 26)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1.5))
            p.setBrush(QtGui.QColor(col) if on else QtGui.QColor("#121418"))
            p.drawRoundedRect(r, 13, 13)
            p.setPen(QtGui.QColor("white" if on else col))
            p.drawText(r, QtCore.Qt.AlignmentFlag.AlignCenter, text)
        p.setPen(QtGui.QColor(DIM))
        p.drawText(QtCore.QPointF(268, 126), "pad lost = released")
        if pad.driving:
            vx, vy, wz = pad.command()
            p.setPen(QtGui.QColor(BRIGHT))
            p.drawText(QtCore.QPointF(268, 144), f"vx {vx:+.2f}  vy {vy:+.2f}  wz {wz:+.2f}")
        elif not live:
            p.setPen(QtGui.QColor(FAINT))
            p.drawText(QtCore.QPointF(268, 144), "no controller")
        p.end()


class PadSetup(_Ticking):
    """The mapping by position with live values, the silent-pad timeout and
    the dead zone, and what SDL sees."""

    def __init__(self, pad):
        super().__init__(pad)
        self.map = table(["control", "SDL name (by position)", "label", "live"], 6,
                         stretch_col=0, height=6 * 26 + 40)
        self.timeout = QtWidgets.QSpinBox()
        self.timeout.setRange(50, 2000)
        self.timeout.setSingleStep(10)
        self.timeout.setValue(int(pad.silent_ms))
        self.timeout.setFixedWidth(90)
        self.timeout.valueChanged.connect(self._timeout)
        self.dz = QtWidgets.QDoubleSpinBox()
        self.dz.setRange(0.05, 0.5)
        self.dz.setSingleStep(0.01)
        self.dz.setDecimals(2)
        self.dz.setValue(pad.dead_zone)
        self.dz.setFixedWidth(90)
        self.dz.valueChanged.connect(self._dz)
        self.silent_note = lab("", DIM, 12)
        self.silent_note.setWordWrap(True)
        self.info = lab("", "#b8bec6", 12, mono=True)
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(8)
        form.addRow(lab("silent-pad timeout", DIM),
                    hbox(self.timeout, lab("ms · used only if this pad streams reports "
                                           "while held still", DIM), "stretch"))
        form.addRow(lab("stick dead zone", DIM),
                    hbox(self.dz, lab("vx through the trained band; vy, wz linear", DIM),
                         "stretch"))
        self.setLayout(vbox(self.map, form, self.silent_note, self.info,
                            lab("tools/pad_latency.py runs the §14 arrival checks: report "
                                "rate held still, switch-off to disconnect", DIM, 11),
                            spacing=8))
        self.refresh()

    def _timeout(self, v):
        self.pad.silent_ms = float(v)

    def _dz(self, v):
        self.pad.dead_zone = float(v)

    def refresh(self):
        pad = self.pad
        live = pad.pad is not None
        rb, b = pad.labels
        a = pad.axes
        vx, vy, wz = pad.command() if live else (0.0, 0.0, 0.0)
        dot = lambda on: "● held" if on else "○"  # noqa: E731
        rows = (("dead-man (hold)", "rightshoulder", rb,
                 dot(pad.deadman) + ("  sim only" if pad.gated else ""),
                 GREEN if pad.driving else AMBER if pad.deadman else None),
                ("E-STOP (any role, host)", "b (east)", b, dot(pad.estop_down),
                 RED if pad.estop_down else None),
                ("vx", "lefty (up = +)", "left stick ↕",
                 f"{a['lefty']:+.2f} → {vx:+.2f} m/s", None),
                ("vy", "leftx (left = +)", "left stick ↔",
                 f"{a['leftx']:+.2f} → {vy:+.2f} m/s", None),
                ("wz", "rightx (left = +)", "right stick ↔",
                 f"{a['rightx']:+.2f} → {wz:+.2f} rad/s", None),
                ("arm / reset servos", "—", "not on the pad", "", FAINT))
        for r, (c0, c1, c2, c3, col) in enumerate(rows):
            set_cell(self.map, r, 0, c0)
            set_cell(self.map, r, 1, c1, DIM)
            set_cell(self.map, r, 2, c2, FAINT if r == 5 else None)
            set_cell(self.map, r, 3, c3 if live else "", col or BRIGHT)
        if not live:
            self.silent_note.setText("no controller: nothing to watch")
        elif pad.streams:
            self.silent_note.setText(
                f"this pad streams {pad.still_rate:.0f} inputs/s while held still (longest gap "
                f"{pad.still_gap * 1000:.0f} ms): silence over {pad.silent_ms:.0f} ms with "
                f"{rb} held releases it")
        else:
            seen = ("" if pad.still_rate is None else
                    f" (last still window: {pad.still_rate:.0f} inputs/s, longest gap "
                    f"{pad.still_gap * 1000:.0f} ms)")
            self.silent_note.setText(
                f"not seen streaming while held still{seen}: silence means nothing, so only "
                f"the OS disconnect counts")
        lines = []
        if pad.backend is None:
            lines.append(f"SDL unavailable: {pad.error}")
        else:
            lines.append(f"SDL {pad.sdl} · {len(pad.seen)} joystick(s) seen")
            for i, name, ok in pad.seen:
                lines.append(f"  [{i}] {name}" + ("" if ok else "  (no GameController mapping)"))
        if live:
            lines.append(f"GUID {pad.pad.guid}")
            lines.append(f"mapping {pad.pad.mapping[:160]}")
            lines.append(f"inputs {pad.rate():.0f}/s now")
        host = pad.host() or "(none)"
        lines.append(f"host {host}: " + ("the dead-man drives the sim" if pad.host_ok() else
                                         f"{rb} ignored -- sim only until measured "
                                         f"(design §7); {b} = E-STOP works"))
        self.info.setText("\n".join(lines))
