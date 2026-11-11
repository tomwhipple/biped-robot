"""Checks: the pre-arm go/no-go of docs/first-policy-stand.md §2 and the
abort watch of its §5 (design §6 "Checks").

Each row says where its value comes from. Rows the console can read from
data it already receives turn PASS or FAIL by themselves; the rest (what
only the tether or the eyes can answer) are ticked by the operator, and a
robot report that contradicts a tick (an arm REFUSED for gains, say) clears
it. Nothing here blocks ARM -- the firmware's refusals are authoritative
(design §15, open question 1, default "label") -- the safety bar's ARM reads
"ARM · checks open" while open_rows() > 0.

The abort watch is ADVISORY (design §15, open question 2): it flags, the
operator and the spotter act. It never sends anything: no stop, no frame,
no call on the controller. The robot judges itself at 50 Hz (the flail guard
holds, the fall guard goes limp); a console-side automatic E-stop would only
add a limp robot nobody is holding (first-policy-stand.md §4).
"""
from __future__ import annotations

import math
import os
import sys
import time

from PyQt6 import QtCore, QtWidgets

from . import core
from .ui import (AMBER, BRIGHT, DIM, FAINT, GREEN, RED, btn, group, hbox, lab, pill,
                 set_cell, set_pill, table, vbox, wrap)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import protocol  # noqa: E402  link/protocol.py

AR = protocol.ArmResult
PACK_MIN_V = 11.4              # §2 item 2
PACK_WARN_V = 10.5             # §2 item 2: the pack guard warns here (and lands at 9.9 V)
UPRIGHT_DEG = 3.0              # §2 item 13
ARM_WINDOW_S = 30.0            # §2: "Arm within ~30 s of the readback"
TILT_ABORT_DEG = 10.0          # §5: tilt beyond ~10 deg ...
TILT_ABORT_S = 1.0             # ... not coming back within 1 s
RECENT_S = 30.0                # a flight event this recent counts as "now"
LEG_JOINTS = 12                # the first 12 bus joints are the legs (flightrec.BUS_JOINT_NAMES)
BUS_NAMES = ("R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch",
             "L_knee", "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll",
             "neck_yaw", "R_shoulder", "R_elbow", "L_shoulder", "L_elbow")

PASS, FAIL, WAIT, NODATA = "PASS", "FAIL", "WAIT", "NO DATA"
STATUS_COLOUR = {PASS: GREEN, FAIL: RED, WAIT: AMBER, NODATA: DIM}
HOME_FAILED = (AR.HOME_NO_CAL.value, AR.HOME_LOW_BATT.value, AR.HOME_BUS_FAILED.value,
               AR.HOME_NOT_REACHED.value)


def tilt_deg(up_z: float) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def _get(obj, *names):
    """The first of `names` on obj, called if callable. app.flight's API
    (flight.py) is duck-typed here: hello() -> the last HELLO as
    tools/flightrec.decode returns it, last_tick() -> the last TICK."""
    if obj is None:
        return None
    for n in names:
        v = getattr(obj, n, None)
        if v is None:
            continue
        try:
            return v() if callable(v) else v
        except (TypeError, AttributeError, KeyError, IndexError):
            return None
    return None


class Row:
    def __init__(self, num, item, source, tick=False, fn=None):
        self.num, self.item, self.source = num, item, source
        self.tick = tick              # the operator answers it
        self.fn = fn                  # evaluator name: (status | None, value)
        self.status = None if tick else NODATA
        self.value = ""
        self.ticked = False

    @property
    def passed(self) -> bool:
        return self.status == PASS

    @property
    def open(self) -> bool:
        return not self.passed and not self.ticked


class ChecksTab(QtWidgets.QWidget):
    always_update = True              # the watch keeps time on every tab

    def __init__(self, app):
        super().__init__()
        self.app = app
        app.checks = self
        R = Row
        self.rows = [
            R("1", "policy: `stat` says exported (not PLACEHOLDER WEIGHTS), 12 joints, obs 165",
              "tether: stat · beacon: an arm REFUSED_POLICY fails it", True, "_policy"),
            R("2", "pack ≥ 11.4 V (3S); after a pack swap, `batt reset`",
              "beacon vbat · state VLAND / VSAFE", fn="_pack"),
            R("3", "as-built calibration loaded; all 12 leg joints fitted",
              "BENCH diag cal bit · flight HELLO cal_ok, fitted", fn="_cal"),
            R("4", "gains: all 12 leg servos P 32 / D 32 (`run` re-reads them)",
              "tether: gains · beacon: an arm REFUSED_GAINS fails it", True, "_gains"),
            R("5", "every fitted servo answers (`sread 50` for misses and timing)",
              "beacon servo_err", fn="_servos"),
            R("6", "IMU: QMI8658C live (not the STUB), mount + bias done, `imu avg` on",
              "tether: imu", True),
            R("7", "gyro still: horizontal RMS < 0.01 rad/s, no bumped-table peak",
              "tether: imu ring 2000", True),
            R("8", "shaper 10 Hz (never 0)", "tether: shape", True),
            R("9", "obsfreeze none", "tether: obsfreeze", True),
            R("10", "fall guard ON (fall autohome as wanted)", "tether: fall", True),
            R("10a", "flail guard ON: pose 15° / 300 ms, rate 1.5 rad/s, tilt 20° / 200 ms, "
              "chatter 7 rad/s", "tether: flail", True),
            R("10b", "a stop is ready: HOLD / E-STOP have somewhere to go",
              "this console's out-of-band stop frames", fn="_stop_ready"),
            R("10c", "one hand on the servo power switch, for the whole run", "the operator",
              True),
            R("11", "robot clock synced (UTC)", "beacon t_us (0 = not synced)", fn="_clock"),
            R("11", "robot `rec` ON; collector running (flight_session.sh start)",
              "tether: rec · flight HELLO when it arrives", True, "_flight_rec"),
            R("—", "console recording on", "this console's JSONL (record by default)",
              fn="_console_rec"),
            R("12", "home readback: all 12 joints at their zeros; arm within ~30 s",
              "BENCH diag DISARMED_HOME · flight CLI output", fn="_home"),
            R("13", "upright on its own after home: tilt < 3°, up ≈ (0, 0, 1)",
              "beacon up vector (ATT when asked)", fn="_upright"),
            R("13", "floor hard and flat, 1 m clear; spotter's hands ~5 cm either side",
              "eyes", True),
            R("—", "filming: tools/cam_record.sh from before the port opens",
              "AGENTS.md: film every hardware session", True),
        ]
        self.aborts = [
            ("tilt", "torso tilt beyond ~10° not coming back within 1 s",
             "beacon up vector, 10 Hz"),
            ("foot", "a foot lifting, or a step without a shove",
             "eyes (no foot contact sensing)"),
            ("chatter", "buzzing, chatter or jitter faster than ~3 Hz, over 1–2°",
             "eyes and ears; the flail guard's chatter limit acts by itself"),
            ("drift", "drift beyond 5 cm, or a steady rotation", "eyes (no odometry)"),
            ("hot", "a servo too hot to touch", "touch · flight recorder temperature"),
            ("fault", "a servo fault bit", "beacon servo_err"),
            ("batt", "LOW-BATT", "beacon vbat (guard warns 10.5 V, lands 9.9 V) · VLAND / VSAFE"),
            ("fallen", "FALLEN: the fall guard went limp",
             "beacon state FALLEN · BENCH DISARMED_FALL"),
            ("flail", "the robot's flail guard tripped (holds stiff, then homes)",
             "BENCH DISARMED_FLAIL · flight flail event"),
            ("stop", "an operator stop ended the run", "BENCH DISARMED_STOP · flight stop event"),
            ("smell", "a smell", "the nose"),
        ]
        self.watch = {k: ("na", "") for k, _, _ in self.aborts}
        self._tilt_since = None
        self._last_result = None      # the last BENCH arm result seen
        self._home_at = None          # ms the readback was seen; -1 = before this console
        self._home_verdict = None     # (status, value) frozen at the arm
        self.arm_left_s = None
        self._build()

    # -- widgets -------------------------------------------------------------
    def _build(self):
        self.pill = pill("NO DATA", DIM, True, 110)
        self.summary = lab("", "#f0c56a", 15, True)
        self.window = lab("—", BRIGHT, 18, True, True)
        untick = btn("untick all", 120, h=30)
        untick.clicked.connect(self.untick_all)
        # The summary wraps rather than widening the tab (it grows with the
        # number of open rows).
        self.summary.setWordWrap(True)
        self.summary.setMinimumWidth(160)
        head = hbox(self.pill, 12, self.summary, 1, untick, 14,
                    lab("arm window after home readback", DIM), 8, self.window, spacing=8)
        head.setStretchFactor(self.summary, 1)
        self.table = table(["#", "go / no-go (docs/first-policy-stand.md §2)", "status", "value",
                            "source"], len(self.rows), stretch_col=4)
        self.boxes = {}
        self.marks = {}
        for r, row in enumerate(self.rows):
            set_cell(self.table, r, 0, row.num, DIM)
            set_cell(self.table, r, 1, row.item)
            set_cell(self.table, r, 4, row.source, DIM)
            if row.tick:
                cb = QtWidgets.QCheckBox()
                cb.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
                cb.toggled.connect(lambda on, rr=row: setattr(rr, "ticked", on))
                mark = lab("CHECK", AMBER, bold=True)
                self.boxes[r], self.marks[r] = cb, mark
                self.table.setCellWidget(r, 2, wrap(hbox(cb, mark, "stretch", spacing=4,
                                                         margins=(6, 0, 0, 0))))
        self.table.setColumnWidth(2, 130)
        self.table.setFixedHeight(40 + 26 * len(self.rows))
        note = lab("PASS rows are read from data the console already receives; CHECK rows are typed "
                   "on the tether or seen by eye and ticked by the operator (a robot report that "
                   "contradicts a tick clears it). The console never blocks ARM -- the firmware's own "
                   "refusals are authoritative -- but ARM reads 'ARM · checks open' until the list "
                   "is clear.", DIM, 12)
        note.setWordWrap(True)
        self.abort_table = table(["abort condition (§5)", "watched from", "now"],
                                 len(self.aborts), stretch_col=1)
        for r, (_, cond, src) in enumerate(self.aborts):
            set_cell(self.abort_table, r, 0, cond)
            set_cell(self.abort_table, r, 1, src, DIM)
        self.abort_table.setFixedHeight(40 + 26 * len(self.aborts))
        self.setLayout(vbox(
            group("PRE-ARM", vbox(head, self.table, note, spacing=10)),
            group("ABORT WATCH   advisory: it sends nothing",
                  vbox(self.abort_table, lab("the console flags; the operator and the spotter "
                                             "act", DIM, 12))),
            "stretch", spacing=10, margins=(12, 12, 12, 12)))

    def untick_all(self):
        for r, cb in self.boxes.items():
            cb.setChecked(False)

    # -- the API the window reads ----------------------------------------------
    def open_rows(self) -> int:
        """Rows neither PASS nor ticked."""
        return sum(1 for r in self.rows if r.open)

    def row(self, num: str, n: int = 0) -> Row:
        """The n-th row numbered `num` (tests, other tabs)."""
        return [r for r in self.rows if r.num == num][n]

    def update_from(self, st, now_ms: float) -> None:
        self.st, self.now = st, now_ms
        self.fresh = bool(st.fresh)
        self.bench = self.fresh and st.state == core.BENCH
        self.result = core.diag_arm_result(st.diag) if self.bench else None
        self.events = list(self.app.robot_events() or [])
        self.hello = _get(self.app.flight, "hello", "last_hello")
        self.arm_left_s = None
        for i, row in enumerate(self.rows):
            if row.fn is None:
                continue
            status, value = getattr(self, row.fn)()
            if row.tick:
                if status == FAIL and row.status != FAIL and row.ticked:
                    row.ticked = False                 # the robot says otherwise
                    if i in self.boxes:
                        self.boxes[i].setChecked(False)
            row.status, row.value = status, value
        if self.fresh:
            self._last_result = self.result if self.bench else "armed"
        self._watch()
        if self.isVisible():
            self.render()

    # -- the go / no-go rows -----------------------------------------------------
    def _refused(self, result, why):
        if self.result == result.value:
            return FAIL, f"the last arm was REFUSED: {why}"
        return None, ""

    def _policy(self):
        s, v = self._refused(AR.REFUSED_POLICY, "placeholder net, not a policy")
        return s, v or "the tether's `stat`"

    def _gains(self):
        s, v = self._refused(AR.REFUSED_GAINS, "a servo's P / D is off (`gains`)")
        return s, v or "the tether's `gains`"

    def _pack(self):
        st = self.st
        if not self.fresh:
            return NODATA, "no beacon"
        if st.state in (core.VLAND, core.VSAFE):
            return FAIL, f"{st.vbat_v:.2f} V · pack guard latched: swap it, `batt reset`"
        if self.result == AR.HOME_LOW_BATT.value:
            return FAIL, f"{st.vbat_v:.2f} V · reset refused: pack under-voltage latch"
        if round(st.vbat_v, 2) >= PACK_MIN_V:
            return PASS, f"{st.vbat_v:.2f} V"
        return FAIL, f"{st.vbat_v:.2f} V < {PACK_MIN_V} V: start on a full pack"

    def _cal(self):
        h = self.hello
        legs = None
        if isinstance(h, dict) and "fitted" in h:
            legs = sum(1 for f in list(h["fitted"])[:LEG_JOINTS] if f)
        if isinstance(h, dict) and "cal_ok" in h and not h["cal_ok"]:
            return FAIL, "flight HELLO: no calibration in NVS (`cal`)"
        if legs is not None and legs < LEG_JOINTS:
            return FAIL, f"flight HELLO: {legs} of {LEG_JOINTS} leg joints fitted"
        fit = (f" · {legs} of {LEG_JOINTS} legs fitted (HELLO)" if legs is not None
               else " · fitted: `run` refuses short of 12")
        if self.bench:
            if self.st.diag & protocol.DIAG_CAL_OK:
                return PASS, "cal ok (BENCH diag)" + fit
            return FAIL, "NO calibration in NVS (BENCH diag): run `cal`"
        if isinstance(h, dict) and h.get("cal_ok"):
            return PASS, "cal ok (flight HELLO)" + fit
        if self.fresh:
            return PASS, "armed: the arm itself requires it"
        return NODATA, "no beacon"

    def _servos(self):
        if not self.fresh:
            return NODATA, "no beacon"
        if self.st.servo_err:
            return FAIL, f"id {core.fault_ids(self.st.servo_err)} faulted"
        return PASS, "no fault bits"

    def _stop_ready(self):
        dest = self.app.stop_dest_text()
        if not dest or dest == "nowhere":
            return FAIL, "nowhere: no --host, $BIMO_HOST or robot_addr"
        return PASS, dest

    def _clock(self):
        if not self.fresh:
            return NODATA, "no beacon"
        if not self.st.t_us:
            return FAIL, "t_us = 0: not synced (wifi, NTP)"
        sent = time.time() - self.st.tlm_age_ms / 1000.0
        return PASS, f"synced · Δ ≈ {(self.st.t_us / 1e6 - sent) * 1000:+.0f} ms"

    def _flight_rec(self):
        h = self.hello
        if isinstance(h, dict) and "enabled" in h:
            if h["enabled"]:
                return PASS, "flight records arriving · HELLO: rec on"
            return FAIL, "flight HELLO: rec off (`rec`)"
        return None, "no flight records here: the tether's `rec`, mira's collector"

    def _console_rec(self):
        st = self.st
        if st.rec_active:
            return PASS, f"{st.rec_cmd} cmd / {st.rec_tlm} tlm"
        return FAIL, "off: Session tab, or the record key"

    def _home_event(self):
        """The newest "home readback" CLI line among the robot's events, as
        (ms on the console clock, ok, text)."""
        for t, kind, msg in reversed(self.events):
            if "home readback" in msg:
                ok = "all" in msg and "zeros" in msg
                return self.now - (time.time() - t) * 1000.0, ok, msg
        return None

    def _home(self):
        r = self.result
        if self.bench:
            if r == AR.DISARMED_HOME.value:
                if self._home_at is None:
                    self._home_at = -1.0 if self._last_result is None else self.now
            else:
                self._home_at = None
            self._home_verdict = None
        elif self.fresh and self._home_verdict is not None:
            s, v = self._home_verdict
            return s, v + " · then armed"
        ev = self._home_event()
        at, ok, text = None, True, ""
        if self._home_at is not None and self._home_at >= 0:
            at = self._home_at
        if ev is not None and (at is None or ev[0] > at):
            at, ok, text = ev
        out = self._home_status(r, at, ok, text)
        if self.bench:
            self._home_verdict = out
        return out

    def _home_status(self, r, at, ok, text):
        if at is not None:
            if not ok:
                return FAIL, text
            age = (self.now - at) / 1000.0
            if age <= ARM_WINDOW_S:
                self.arm_left_s = ARM_WINDOW_S - age
                return PASS, f"{age:.0f} s ago · arm within ~{ARM_WINDOW_S:.0f} s"
            return FAIL, f"{age:.0f} s ago: past the ~{ARM_WINDOW_S:.0f} s window, `home` again"
        if self._home_at == -1.0:
            return WAIT, "reported before this console started: age unknown, `home` again"
        if r == AR.HOME_PENDING.value:
            return WAIT, "joints slewing, readback pending"
        if r in HOME_FAILED:
            return FAIL, core.diag_reason(self.st.diag)
        if not self.fresh:
            return NODATA, "no beacon"
        if not self.bench:
            return NODATA, "armed: no readback seen before the arm"
        return WAIT, "no readback yet: RESET SERVOS, or `home` on the tether"

    def _upright(self):
        st = self.st
        if not self.fresh:
            return NODATA, "no beacon"
        tilt = tilt_deg(st.up_z)
        if st.have_att:
            n = math.sqrt(st.up_x ** 2 + st.up_y ** 2 + st.up_z ** 2)
            if not 0.9 <= n <= 1.1:
                return FAIL, f"|up| = {n:.2f}: not a unit vector, check `imu`"
            v = f"tilt {tilt:.1f}° · up ({st.up_x:+.2f}, {st.up_y:+.2f}, {st.up_z:+.2f})"
            if st.up_x == 0.0 and st.up_y == 0.0 and st.up_z == 1.0:
                v += " exactly: a STUB IMU reads this (item 6)"
        else:
            v = f"tilt {tilt:.1f}° from up_z (x, y: ask for ATT)"
        if tilt < UPRIGHT_DEG:
            return PASS, v
        return FAIL, v + f" ≥ {UPRIGHT_DEG:.0f}°"

    # -- the abort watch -----------------------------------------------------------
    def _recent(self, kind):
        for t, k, msg in reversed(self.events):
            if k == kind:
                return time.time() - t, msg
        return None

    def _watch(self):
        st, w = self.st, self.watch
        fresh = self.fresh
        if fresh:
            tilt = tilt_deg(st.up_z)
            if tilt <= TILT_ABORT_DEG:
                self._tilt_since = None
                w["tilt"] = ("ok", f"{tilt:4.1f}°")
            else:
                if self._tilt_since is None:
                    self._tilt_since = self.now
                dur = (self.now - self._tilt_since) / 1000.0
                w["tilt"] = (("abort", f"{tilt:.1f}° for {dur:.1f} s: not coming back")
                             if dur >= TILT_ABORT_S else ("warn", f"{tilt:.1f}° for {dur:.1f} s"))
        else:
            self._tilt_since = None
            w["tilt"] = ("na", "no beacon")
        for k in ("foot", "drift", "chatter"):
            w[k] = ("na", "eyes")
        w["smell"] = ("na", "")
        tick = _get(self.app.flight, "last_tick", "tick")
        temps = []
        if isinstance(tick, dict):
            for j in tick.get("joints", []):
                b = j.get("bus", -1)
                temps.append((j.get("temp_c", 0), BUS_NAMES[b] if 0 <= b < len(BUS_NAMES)
                              else f"bus {b}"))
        if temps and _get(self.app.flight, "simulated"):
            w["hot"] = ("na", "sim records: no servo temperature")
        elif temps:
            t, name = max(temps)
            w["hot"] = ("info", f"max {t} °C ({name})")
        else:
            w["hot"] = ("na", "touch: no flight records here")
        if not fresh:
            for k in ("fault", "batt", "fallen"):
                w[k] = ("na", "no beacon")
        else:
            w["fault"] = (("abort", f"id {core.fault_ids(st.servo_err)} faulted")
                          if st.servo_err else ("ok", "none"))
            if st.state in (core.VLAND, core.VSAFE):
                w["batt"] = ("abort", f"{st.vbat_v:.2f} V · pack guard latched "
                             f"({core.STATE_SHORT[st.state]})")
            elif round(st.vbat_v, 2) < PACK_WARN_V:
                w["batt"] = ("abort", f"LOW-BATT {st.vbat_v:.2f} V")
            elif round(st.vbat_v, 2) < PACK_MIN_V:
                w["batt"] = ("warn", f"{st.vbat_v:.2f} V: below the {PACK_MIN_V} V start")
            else:
                w["batt"] = ("ok", f"{st.vbat_v:.2f} V")
            fell = st.state == core.FALLEN or self.result == AR.DISARMED_FALL.value
            w["fallen"] = ("abort", "FALLEN: torque off, the run is over") if fell else ("ok", "no")
        for k, res, yes, no in (
                ("flail", AR.DISARMED_FLAIL, "TRIPPED: joints held stiff; it homes once "
                 "upright and still", "not tripped"),
                ("stop", AR.DISARMED_STOP, "STOPPED: an operator stop ended the run", "none")):
            ev = self._recent(k)
            if self.result == res.value:
                w[k] = ("abort", yes)
            elif ev is not None and ev[0] < RECENT_S:
                w[k] = ("abort", f"{yes} ({ev[0]:.0f} s ago: {ev[1]})")
            elif ev is not None:
                w[k] = ("ok", f"{no} · last {time.strftime('%H:%M:%S', time.gmtime(time.time() - ev[0]))} UTC")
            elif not fresh:
                w[k] = ("na", "no beacon")
            else:
                w[k] = ("ok", no)

    # -- drawing -----------------------------------------------------------------
    def render(self):
        fails = [r for r in self.rows if r.status == FAIL]
        ticks_open = [r for r in self.rows if r.tick and r.open and r.status != FAIL]
        nodata = [r for r in self.rows if not r.tick and r.status in (NODATA, WAIT)]
        n_open = self.open_rows()
        if n_open == 0:
            set_pill(self.pill, "GO", GREEN)
        elif fails:
            set_pill(self.pill, "NO-GO", RED)
        else:
            set_pill(self.pill, "OPEN", AMBER)
        parts = []
        if ticks_open:
            parts.append(f"{len(ticks_open)} tether / eye checks open")
        if fails:
            names = ", ".join(dict.fromkeys(r.num if r.num != "—" else r.item.split(":")[0]
                                            for r in fails))
            parts.append(f"{len(fails)} failed: {names}")
        if nodata:
            parts.append(f"{len(nodata)} waiting for data")
        self.summary.setText(" · ".join(parts) if parts else "every row passes or is ticked")
        self.summary.setStyleSheet(f"color:{GREEN if n_open == 0 else '#f0c56a'}; "
                                   f"font-size:15px; font-weight:700")
        self.window.setText(f"{self.arm_left_s:.0f} s left" if self.arm_left_s is not None
                            else "—")
        for i, row in enumerate(self.rows):
            if row.tick:
                mark = self.marks[i]
                if row.status in (PASS, FAIL):
                    text, col = row.status, STATUS_COLOUR[row.status]
                elif row.ticked:
                    text, col = "ticked", GREEN
                else:
                    text, col = "CHECK", AMBER
                if mark.text() != text:
                    mark.setText(text)
                    mark.setStyleSheet(f"color:{col}; font-weight:700")
            else:
                set_cell(self.table, i, 2, row.status, STATUS_COLOUR.get(row.status, DIM))
            set_cell(self.table, i, 3, row.value,
                     RED if row.status == FAIL else BRIGHT if row.status == PASS else DIM)
        colour = {"ok": BRIGHT, "info": BRIGHT, "warn": AMBER, "abort": RED, "na": FAINT}
        for r, (k, _, _) in enumerate(self.aborts):
            level, text = self.watch[k]
            set_cell(self.abort_table, r, 2, text, colour[level])

    def showEvent(self, ev):
        if hasattr(self, "st"):
            self.render()
        super().showEvent(ev)
