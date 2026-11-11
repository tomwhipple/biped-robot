"""The safety bar: on every tab, never scrolls (design §4).

State badge, ARM / STAND, HOLD and E-STOP (out of band, every role), RESET
SERVOS on its own amber, battery, servo health, pad, recording, the
handover button; a fixed-width link line; the BENCH reason whenever the
robot gives one; the console's last message, or KEYS CAPTURED while a text
field has the keys.

Every numeric field is fixed width (bimo_gui's "text that moves has to be
re-read"), E-STOP is always red, and a control that does not apply in this
role is greyed, never hidden.
"""
from __future__ import annotations

import math
import time

from PyQt6 import QtCore, QtGui, QtWidgets

from . import core
from .core import lag_text
from .ui import (AMBER, BENCHC, BRIGHT, DIM, GREEN, ORANGE, RED, STATE_COLOUR, btn, hbox,
                 lab, restyle, vbox)

SEP = "<span style='color:#4a5059'>&nbsp;&nbsp;│&nbsp;&nbsp;</span>"


def _nb(s: str) -> str:
    return s.replace(" ", "&nbsp;")


def flag_names(flags: int) -> str:
    return " ".join(n for bit, n in ((core.FLAG_ARM, "ARM"), (core.FLAG_ENABLE, "ENABLE"),
                                    (core.FLAG_ESTOP, "ESTOP"), (core.FLAG_HOME, "HOME"),
                                    (core.FLAG_POSE, "POSE"), (core.FLAG_ATT, "ATT"))
                    if flags & bit) or "-"


def _fit(b: QtWidgets.QPushButton, px: int, texts, least: int) -> None:
    """Fix a stop button's width at the widest text it can show (in its
    style sheet's font), never under `least`: a stop button never clips."""
    f = QtGui.QFont(b.font())
    f.setPixelSize(px)
    f.setWeight(QtGui.QFont.Weight.ExtraBold)
    fm = QtGui.QFontMetrics(f)
    # padding 14 px a side (ui.QSS) + a 3 px border + slack for the font.
    b.setFixedWidth(max(least, max(fm.horizontalAdvance(t) for t in texts) + 2 * 17 + 14))


class SafetyBar(QtWidgets.QFrame):
    def __init__(self, app):
        super().__init__()
        self.app = app
        c = app.ctl
        self.setObjectName("safety")
        self.badge = lab("")
        self.badge.setFixedHeight(46)
        # Fixed at the widest text it can show, so it never clips and never
        # changes width (a field that moves has to be re-read).
        f = QtGui.QFont(self.badge.font())
        f.setPixelSize(19)
        f.setWeight(QtGui.QFont.Weight.ExtraBold)
        fm = QtGui.QFontMetrics(f)
        widest = max(fm.horizontalAdvance(f"●  {t} · WATCH")
                     for t in ("NO BEACON", "LINK LOST", "STOPPED", "FALLEN", "FLAIL",
                               "BENCH", "ESTOP", "VLAND", "VSAFE", "RELAX", "STAND", "LIVE"))
        self.badge.setFixedWidth(widest + 32)
        self.badge.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.arm = btn("ARM", 212, "arm")
        self.stand = btn("STAND", 124)
        # The stop buttons are sized from the widest text they can show
        # (_labels), so neither is ever clipped; "sending" is beside them on
        # the second row, not in them.
        self.hold = btn("HOLD", None, "hold", h=46)
        self.estop = btn("E-STOP", None, "estop", h=46)
        self.reset = btn("RESET SERVOS", 176, "reset")
        self.control = btn("TAKE CONTROL", 150, "take", h=34)
        self.arm.clicked.connect(lambda: c.fire("arm"))
        self.stand.clicked.connect(lambda: c.fire("stand"))
        self.hold.clicked.connect(c.hold)
        self.estop.clicked.connect(c.estop)
        self.reset.clicked.connect(lambda: c.fire("home"))
        self.control.clicked.connect(c.toggle_control)
        self.vbat = lab("--.-- V", BRIGHT, 22, True, True)
        self.vbat_sub = lab("pack", DIM, 11)
        self.servo = lab("servos --", DIM, 14, True)
        self.servo_sub = lab("", DIM, 11)
        self.pad = lab("PAD  none", DIM, 14, True)
        self.pad_sub = lab("keyboard / mouse", DIM, 11)
        self.rec = lab("○ not recording", DIM, 14, True)
        self.rec_sub = lab("", DIM, 11)
        self.sending = lab("", rich=True)
        self.vbat.setMinimumWidth(110)
        # The small print gives way before the window grows past the screen.
        for w in (self.vbat_sub, self.servo_sub, self.pad_sub, self.rec_sub):
            w.setMinimumWidth(1)
        # Two rows, so the bar fits a laptop screen (1280 pt wide): the robot
        # controls on top; the readouts, a stop being sent and the handover
        # button beneath them.
        row1 = hbox(self.badge, 6, self.arm, self.stand, 14, self.hold, self.estop, 22,
                    self.reset, "stretch", spacing=10)
        row2 = hbox(self.vbat, self.vbat_sub, 24, self.servo, self.servo_sub, 24,
                    self.pad, self.pad_sub, 24, self.rec, self.rec_sub, "stretch",
                    self.sending, 12, self.control, spacing=8)
        self.link = lab("", mono=True, rich=True)
        self.bench = lab("", rich=True)
        self.note = lab("", rich=True)
        # Text never sets the window's width: it wraps to whatever is there.
        for w in (self.link, self.bench, self.note):
            w.setWordWrap(True)
            w.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                            QtWidgets.QSizePolicy.Policy.Preferred)
        self.setLayout(vbox(row1, row2, self.link, self.bench, self.note, spacing=6,
                            margins=(14, 10, 14, 8)))
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred,
                           QtWidgets.QSizePolicy.Policy.Fixed)
        self._labels()

    def _key(self, action: str) -> str:
        k = self.app.ctl.core.key_for(core.action_from_name(action))
        name = core.key_name(k)
        return f"   [{name.lower() if name == 'Space' else name}]" if name else ""

    def _labels(self) -> None:
        self.stand.setText("STAND" + self._key("stand"))
        self.hold.setText("HOLD" + self._key("hold"))
        self.reset.setText("RESET SERVOS" + self._key("home"))
        # ui.QSS: #hold is 16 px, #estop / #estop_latched 19 px, all weight 800.
        _fit(self.hold, 16, ["HOLD" + self._key("hold")], 124)
        _fit(self.estop, 19, [t + self._key("estop") for t in ("E-STOP", "E-STOP LATCHED")],
             204)

    def update_from(self, st, now: float) -> None:
        app = self.app
        c = app.ctl
        driver = not st.readonly
        # -- badge ---------------------------------------------------------
        if not st.have_tlm:
            text, col = "NO BEACON", "#3a3f47"
        elif st.link_lost:
            text, col = "LINK LOST", RED
        else:
            text = core.STATE_SHORT[st.state] if 0 <= st.state < 8 else "?"
            col = STATE_COLOUR[st.state] if 0 <= st.state < 8 else RED
            if st.state == core.BENCH:
                r = core.diag_arm_result(st.diag)
                text = {12: "FLAIL", 13: "STOPPED", 3: "FALLEN"}.get(r, "BENCH")
                col = RED if r in (12, 13, 3) else BENCHC
        if st.readonly:
            text = f"{text} · WATCH"
        self.badge.setText(f"●  {text}")
        self.badge.setStyleSheet(f"background:{col}; color:white; font-weight:800; "
                                 f"font-size:19px; border-radius:8px; padding:4px 12px;")
        # -- controls --------------------------------------------------------
        arm_txt = "DISARM" if st.armed else "ARM"
        if not st.armed and app.checks_open():
            arm_txt = "ARM · checks open"
        self.arm.setText(arm_txt + self._key("arm"))
        self.arm.setEnabled(driver)
        self.stand.setEnabled(driver)
        self.reset.setEnabled(driver)
        stopping = c.stopping()
        latched = bool(st.estop)
        self.estop.setText(("E-STOP LATCHED" if latched else "E-STOP") + self._key("estop"))
        restyle(self.estop, "estop_latched" if latched else "estop")
        self.hold.setText("HOLD" + self._key("hold"))
        sending = ("" if not stopping else
                   f"<span style='color:#ff5a50; font-weight:800'>■ E-STOP sending</span>"
                   if c.stops[-1].hard else
                   f"<span style='color:{ORANGE}; font-weight:800'>■ HOLD sending</span>")
        if sending != self.sending.text():
            self.sending.setText(sending)
        if driver:
            self.control.setText("WATCH ONLY")
            restyle(self.control, "watch")
            self.control.setEnabled(True)
        else:
            self.control.setText("TAKE CONTROL")
            restyle(self.control, "take")
            self.control.setEnabled(bool(st.has_dest))
            self.control.setToolTip("" if st.has_dest else
                                    "started without --host: nowhere to take control to")
        # -- readouts --------------------------------------------------------
        if st.fresh:
            self.vbat.setText(f"{st.vbat_v:5.2f} V")
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, st.up_z))))
            self.vbat_sub.setText(f"pack · tilt {tilt:4.1f}°")
            if st.servo_err:
                self.servo.setText("SERVO FAULT")
                self.servo.setStyleSheet(f"color:{RED}; font-size:14px; font-weight:700")
                self.servo_sub.setText(f"id {core.fault_ids(st.servo_err)}")
            else:
                self.servo.setText("servos ok")
                self.servo.setStyleSheet(f"color:{GREEN}; font-size:14px; font-weight:700")
                self.servo_sub.setText(f"loop late {st.loop_late_pct:3d} %")
        else:
            self.vbat.setText("--.-- V")
            self.vbat_sub.setText("pack · no beacon")
            self.servo.setText("servos --")
            self.servo.setStyleSheet(f"color:{DIM}; font-size:14px; font-weight:700")
            self.servo_sub.setText("")
        pt, pcol, psub = app.pad_status()
        self.pad.setText(pt)
        self.pad.setStyleSheet(f"color:{pcol}; font-size:14px; font-weight:700")
        self.pad_sub.setText(psub)
        if st.rec_active:
            el = time.monotonic() - app.rec_started if app.rec_started else 0.0
            self.rec.setText("● REC")
            self.rec.setStyleSheet("color:#ff5a50; font-size:14px; font-weight:700")
            self.rec_sub.setText(f"{int(el) // 60:02d}:{int(el) % 60:02d} · "
                                 f"{st.rec_cmd} cmd / {st.rec_tlm} tlm")
        else:
            self.rec.setText("○ not recording")
            self.rec.setStyleSheet(f"color:{DIM}; font-size:14px; font-weight:700")
            self.rec_sub.setText("")
        # -- the link line ---------------------------------------------------
        d = f"<span style='color:{DIM}'>"
        o = c.o
        if driver:
            tx = (f"{d}→</span> {_nb(f'{o.host}:{o.cmd_port}')}&nbsp;&nbsp;"
                  f"{_nb(f'{app.rate_hz:3.0f} Hz')}&nbsp;&nbsp;{_nb(f'{st.last_len:2d} B')}"
                  f"&nbsp;&nbsp;{d}{_nb(f'{flag_names(st.flags):<26s}')}</span>")
        else:
            tx = f"<span style='color:{AMBER}'>OBSERVER · no commands · stops are out of band</span>"
        if not st.have_tlm:
            rx = (f"{d}← :{o.tlm_port} beacon none"
                  + (": the robot beacons only to its commander" if st.readonly else "")
                  + "</span>")
        elif st.link_lost:
            rx = (f"<span style='color:#ff5a50'>← :{o.tlm_port} LINK LOST "
                  f"{_nb(f'{st.tlm_age_ms / 1000.0:6.1f}')} s ago</span>")
        else:
            rx = (f"{d}←</span> :{o.tlm_port} beacon&nbsp;{_nb(f'{st.tlm_age_ms:4.0f} ms')}"
                  f"&nbsp;&nbsp;{_nb(f'{st.rate_hz:4.1f} Hz')}&nbsp;&nbsp;"
                  f"{_nb(lag_text(st, 3))}&nbsp;&nbsp;{_nb(f'bad {st.tlm_bad:3d}')}")
        parts = [tx, rx, app.flight_line()]
        if st.watch_on:
            parts.append(f"{d}relay</span>&nbsp;{st.watched}")
        self.link.setText(SEP.join(p for p in parts if p))
        # -- BENCH reason and the last message -------------------------------
        if st.fresh and st.state == core.BENCH:
            self.bench.setText(f"<span style='color:{AMBER}; font-weight:700'>BENCH:</span>"
                               f"&nbsp;&nbsp;<span style='color:#f0c56a'>"
                               f"{core.diag_reason(st.diag)}</span>")
            self.bench.show()
        else:
            self.bench.hide()
        if app.text_focus() is not None:
            self.note.setText(self.captured_text(app.stops_win()))
        else:
            self.note.setText(f"<span style='color:{DIM}'>note</span>&nbsp;&nbsp;"
                              f"{app.message.replace('<', '&lt;')}")

    def captured_text(self, stops_win: bool) -> str:
        """The note line while a text field has the keys (#234): they are
        never captured without the bar saying so."""
        stops = f"HOLD{self._key('hold')} / E-STOP{self._key('estop')}".replace("   [", " [")
        what = (f"{stops} still fire (robot LIVE)" if stops_win else f"{stops} too")
        return (f"<span style='background:{AMBER}; color:#111; font-weight:800'>"
                f"&nbsp;KEYS CAPTURED&nbsp;</span>&nbsp;&nbsp;"
                f"<span style='color:{AMBER}; font-weight:700'>robot keys go to the text "
                f"field, {what} · Escape or a click outside gives them back</span>")

    def keymap_changed(self) -> None:
        self._labels()
