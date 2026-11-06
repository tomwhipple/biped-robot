"""Session: the console's own recording (on by default, bimo_gui's JSONL
schema), the flight recorder's state, and one event log for console notes
and robot events on one UTC axis, filterable (design §6)."""
from __future__ import annotations

import time

from PyQt6 import QtCore, QtWidgets

from .ui import CYAN, DIM, RED, btn, group, hbox, lab, set_cell, table, vbox

ALARM_KINDS = {"FATAL", "ERROR", "stop", "fault", "alarm", "flail", "fall"}


class SessionTab(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.rec_btn = btn("stop recording [r]", 190, h=34)
        self.rec_btn.clicked.connect(lambda: app.ctl.fire("record"))
        self.rec_state = lab("", DIM, 14, True)
        self.rec_path = lab("", "#b8bec6", 12, mono=True)
        self.rec_path.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.rec_counts = lab("", DIM)
        rec = group("RECORDING   on by default", vbox(
            hbox(self.rec_btn, 14, self.rec_state, 8, self.rec_path, "stretch", self.rec_counts,
                 spacing=6),
            lab("the console's own JSONL (bimo_gui's schema): every command sent, every beacon "
                "received, both clocks", DIM, 12)))
        self.flight = lab("", "#b8bec6", 12, mono=True)
        self.flight.setWordWrap(True)
        fr = group("FLIGHT RECORDER   the robot's file lives on mira", vbox(self.flight))
        self.filters = {}
        row = [lab("show", DIM)]
        for k in ("robot", "console", "alarms"):
            cb = QtWidgets.QCheckBox(k)
            cb.setChecked(True)
            cb.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            self.filters[k] = cb
            row.append(cb)
        self.text = QtWidgets.QLineEdit()
        self.text.setPlaceholderText("filter…")
        self.text.setFixedWidth(260)
        self.log = table(["UTC", "from", "kind", "message"], 0, stretch_col=3)
        self.log.setWordWrap(False)
        evg = group("EVENT LOG   console and robot on one UTC axis",
                    vbox(hbox(*row, "stretch", self.text, spacing=14), self.log))
        self.setLayout(vbox(rec, fr, evg, spacing=10, margins=(12, 12, 12, 12)))
        self._shown = None

    def rows(self):
        """(utc, source, kind, message) newest last: the console log plus the
        robot's flight-record events."""
        out = [(t, "console", k, m) for t, k, m in self.app.ctl.log.rows]
        out += [(t, "robot", k, m) for t, k, m in self.app.robot_events()]
        out.sort(key=lambda r: r[0])
        return out

    def update_from(self, st, now: float) -> None:
        app = self.app
        if st.rec_active:
            self.rec_btn.setText("stop recording [r]")
            self.rec_state.setText("● REC")
            self.rec_state.setStyleSheet("color:#ff5a50; font-size:14px; font-weight:700")
            self.rec_path.setText(app.ctl.core.rec_path)
            self.rec_counts.setText(f"{st.rec_cmd} cmd · {st.rec_tlm} tlm")
        else:
            self.rec_btn.setText("start recording [r]")
            self.rec_state.setText("○ off")
            self.rec_state.setStyleSheet(f"color:{DIM}; font-size:14px; font-weight:700")
            self.rec_counts.setText("")
        self.flight.setText(app.flight_detail())
        want = self.text.text().strip().lower()
        rows = []
        for t, src, kind, msg in self.rows():
            alarm = kind in ALARM_KINDS
            if alarm and not self.filters["alarms"].isChecked():
                continue
            if not alarm and not self.filters[src].isChecked():
                continue
            if want and want not in msg.lower() and want not in kind.lower():
                continue
            rows.append((t, src, kind, msg, alarm))
        rows = rows[-500:]
        key = (len(rows), rows[-1][0] if rows else 0, want)
        if key == self._shown:
            return
        self._shown = key
        self.log.setRowCount(len(rows))
        for r, (t, src, kind, msg, alarm) in enumerate(rows):
            stamp = time.strftime("%H:%M:%S", time.gmtime(t)) + f".{int((t % 1) * 1000):03d}"
            set_cell(self.log, r, 0, stamp, DIM)
            set_cell(self.log, r, 1, src, CYAN if src == "robot" else DIM)
            set_cell(self.log, r, 2, kind, RED if alarm else None)
            set_cell(self.log, r, 3, msg, RED if alarm else None)
        self.log.scrollToBottom()
