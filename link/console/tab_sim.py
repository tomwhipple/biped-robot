"""Sim: sim/sil_twin.py from the console (design §6 Sim) -- policy run, plant,
hang, Start / Stop / Reset, sil_twin's own ident line, the sim's ports, its
log (which opens itself when the sim fails), the sim's MJPEG stream, and
twin-in-the-loop (off by default, §15 decision 2).

The child, the port rule and the tee live in simproc.SimControl (owned by
the Controller, so --headless and quit reach the same child); this tab only
shows them and presses their buttons. The twin itself is drawn elsewhere: its
flight records are the 3D view's amber ghost and a second trace in Joints.
"""
from __future__ import annotations

import time

from PyQt6 import QtCore, QtWidgets

from . import mjpeg, simproc
from .ui import (AMBER, DIM, GREEN, RED, SIMC, Columns, btn, group, hbox, lab, pill, set_pill,
                 vbox, wrap)

ID_COLOUR = "#b8bec6"
FLIGHT_STALE_S = 1.0          # design §5: a layer older than 1 s reads FROZEN


def _check(text, color=None):
    c = QtWidgets.QCheckBox(text)
    c.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    if color:
        c.setStyleSheet(f"color:{color}; font-weight:700")
    return c


def _combo():
    c = QtWidgets.QComboBox()
    c.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    c.setMinimumWidth(320)
    return c


class SimTab(QtWidgets.QWidget):
    always_update = True          # the flight-record age is sampled, so sample it always

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.sim: simproc.SimControl = app.ctl.sim
        self.sim.attach_stream(lambda url: mjpeg.MjpegStream(url, "sim"))
        self._plants = simproc.plants(self.sim.repo)

        # -- the twin --------------------------------------------------------
        self.run_box = _combo()
        self.run_box.currentTextChanged.connect(self._run_picked)
        rescan = btn("rescan", 80, h=30)
        rescan.clicked.connect(self._rescan)
        self.plant_box = _combo()
        self.plant_box.currentIndexChanged.connect(self._plant_picked)
        self.hang = _check("hang: weld the torso, feet free")
        self.hang.toggled.connect(self._hang_toggled)
        self.twin = _check("twin-in-the-loop (option, off by default)", SIMC)
        self.twin.setToolTip("every command frame sent to the robot is also sent to the sim "
                             "on its own ports")
        self.twin.toggled.connect(self.sim.set_twin)
        self.ports = lab("", mono=True)
        self.ports.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.ports_why = lab("", DIM, 12)
        self.ports_why.setWordWrap(True)
        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(10)
        form.addRow(lab("policy run", DIM), hbox(self.run_box, rescan, spacing=8))
        form.addRow(lab("plant", DIM), self.plant_box)
        form.addRow(lab("options", DIM), hbox(self.hang, self.twin, "stretch", spacing=20))
        form.addRow(lab("ports", DIM), vbox(self.ports, self.ports_why, spacing=2))
        self.no_runs = lab("", AMBER, 12)              # #236: the empty list says how
        self.no_runs.setWordWrap(True)
        self.no_runs.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.start_btn = btn("Start", 110, h=36)
        self.start_btn.clicked.connect(self._start)
        self.stop_btn = btn("Stop", 110, h=36)
        self.stop_btn.clicked.connect(self.sim.stop)
        self.reset_btn = btn("Reset", 110, h=36)
        self.reset_btn.clicked.connect(self._reset)
        self.state = pill("STOPPED", DIM, False, 120)
        ctl = hbox(self.start_btn, self.stop_btn, self.reset_btn, "stretch", self.state,
                   spacing=10)
        self.ident = lab("", ID_COLOUR, 12, mono=True)
        self.ident.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.twin_state = lab("", DIM, 12)
        self.twin_state.setWordWrap(True)
        self.tee = lab("", ID_COLOUR, 12, mono=True)
        about = lab("twin-in-the-loop: every command frame sent to the robot is also sent to "
                    "the sim on its own ports; the sim's flight records drive the amber ghost "
                    "in the 3D view and a second trace in Joints. Joint space only: both roots "
                    "are pinned.", DIM, 12)
        about.setWordWrap(True)
        left = group("SIM TWIN   sim/sil_twin.py · the firmware's control code against the "
                     "CPU plant",
                     vbox(form, self.no_runs, ctl, self.ident, self.twin_state, self.tee, about, spacing=12))

        # -- the stream ------------------------------------------------------
        self.stream_line = lab("", ID_COLOUR, 12, mono=True)
        self.stream_line.setWordWrap(True)
        self.stream_note = lab("", AMBER, 12)
        self.stream_note.setWordWrap(True)
        stale = lab(f"the sim's own MJPEG world view (View pane → Sim render) · STALE after "
                    f"{mjpeg.STALE_S:g} s without a frame · retries forever (sil_twin takes "
                    f"~25 s to listen)", DIM, 12)
        stale.setWordWrap(True)
        stream = group("SIM RENDER", vbox(self.stream_line, self.stream_note, stale))

        # -- the log ---------------------------------------------------------
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.log.setMaximumBlockCount(4 * simproc.LOG_LINES)
        self.log.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
        self.fail = lab("", RED, 12, True)
        self.fail.setWordWrap(True)
        self.log_toggle = btn("hide", 70, h=26)
        self.log_toggle.clicked.connect(lambda: self._show_log(not self.log.isVisible()))
        right = group("SIL_TWIN LOG   opens itself when the sim fails",
                      vbox(hbox(self.fail, "stretch", self.log_toggle), self.log))

        self.cols = Columns(wrap(vbox(left, stream, "stretch", spacing=10)), right, spacing=12)
        self.setLayout(vbox(self.cols, "stretch", margins=(12, 12, 12, 12)))

        self._logged = 0
        self._was_failed = False
        self._teed = (0, time.monotonic(), 0.0)        # count, when, rate
        self._frames = (0, time.monotonic(), 0.0)
        fl = self._flight_latest()
        self._fl_last, self._fl_at = fl, None        # only a change counts as arriving
        self._fill_runs()
        self._fill_plants()

    # -- the controls ----------------------------------------------------------
    def _fill_runs(self):
        self.run_box.blockSignals(True)
        self.run_box.clear()
        if self.sim.runs:
            self.run_box.addItems(self.sim.runs)
            self.run_box.setCurrentText(self.sim.run or self.sim.runs[0])
            self.run_box.setToolTip("the trained brain the sim runs (sim/runs/, needs "
                                    "exported SIL weights)")
        else:
            self.run_box.addItem(f"(no runs with SIL weights under {self.sim.repo}/sim/runs)")
        self.no_runs.setText("" if self.sim.runs else simproc.no_runs_help(self.sim.repo))
        self.no_runs.setVisible(not self.sim.runs)
        self.run_box.blockSignals(False)

    def _fill_plants(self):
        self.plant_box.blockSignals(True)
        keep = self.sim.plant
        self.plant_box.clear()
        self.plant_box.addItem(self._default_plant_text())
        self.plant_box.addItems(self._plants)
        if keep in self._plants:
            self.plant_box.setCurrentIndex(1 + self._plants.index(keep))
        self.plant_box.blockSignals(False)

    def _default_plant_text(self) -> str:
        if self.sim.hang:
            return "hang: sil_twin's welded plant (--hang)"
        xml = simproc.run_plant(self.sim.repo, self.sim.run) if self.sim.run else ""
        return f"the run's own: {xml or '(config.json names none)'}"

    def _run_picked(self, name):
        if self.sim.select(name):
            self.plant_box.setItemText(0, self._default_plant_text())

    def _plant_picked(self, i):
        self.sim.plant = self._plants[i - 1] if 0 < i <= len(self._plants) else None
        # --xml wins over --hang in sil_twin: an explicit plant has no hang.
        self.hang.setEnabled(self.sim.plant is None and not self.sim.proc.running())
        self.hang.setToolTip("" if self.sim.plant is None else
                             "an explicit plant replaces the hang plant (sil_twin --xml wins)")

    def _hang_toggled(self, on):
        self.sim.set_hang(on)
        self.plant_box.setItemText(0, self._default_plant_text())

    def _rescan(self):
        self.sim.rescan()
        self._plants = simproc.plants(self.sim.repo)
        self._fill_runs()
        self._fill_plants()
        self.app.ctl.say_status(f"{len(self.sim.runs)} runnable sim run(s) under "
                                f"{self.sim.repo}/sim/runs")

    def _start(self):
        if self.sim.runs:
            self.sim.select(self.run_box.currentText())
        self.sim.start()
        self._show_log(True)

    def _reset(self):
        if not self.sim.reset():
            self.app.ctl.say_status("the sim is not listening")
            return
        # The episode restarted at home: a console driving THIS sim stands, so
        # the next command is not a stale one (bimo_gui). A twin is reset
        # under a robot that keeps its own command.
        if self.sim.active is not None and not self.sim.active.own:
            self.app.ctl.core.stand("sim episode reset")
        self.app.ctl.say_status("sim reset")

    def _show_log(self, on: bool):
        self.log.setVisible(on)
        self.log_toggle.setText("hide" if on else "show")

    # -- providers -------------------------------------------------------------
    def _flight_latest(self):
        fl = getattr(self.app, "flight", None)
        latest = getattr(fl, "latest", None)
        return latest("sim") if latest is not None else None

    # -- the refresh -----------------------------------------------------------
    def update_from(self, st, now: float) -> None:
        mono = time.monotonic()
        rec = self._flight_latest()
        if rec is not None and rec is not self._fl_last:
            self._fl_last, self._fl_at = rec, mono
        if not self.isVisible():
            return
        sim, proc = self.sim, self.sim.proc
        running = proc.running()
        failed = not running and bool(proc.err)
        external = sim.external()
        # Buttons and the controls that describe the NEXT start.
        self.start_btn.setEnabled(not running and bool(sim.runs))
        self.start_btn.setText("Replace" if external else "Start")
        self.stop_btn.setEnabled(running)
        self.reset_btn.setEnabled(running)
        for w in (self.run_box, self.plant_box):
            w.setEnabled(not running and bool(sim.runs))
        self.hang.setEnabled(not running and sim.plant is None)
        if self.twin.isChecked() != sim.twin:
            self.twin.blockSignals(True)
            self.twin.setChecked(sim.twin)
            self.twin.blockSignals(False)
        if self.hang.isChecked() != sim.hang:
            self.hang.blockSignals(True)
            self.hang.setChecked(sim.hang)
            self.hang.blockSignals(False)
            self.plant_box.setItemText(0, self._default_plant_text())
        # State pill and the ident line: what sil_twin SAYS it runs.
        if running and proc.ident:
            set_pill(self.state, "RUNNING", GREEN, True)
        elif running:
            set_pill(self.state, "STARTING", AMBER, True)
        elif failed:
            set_pill(self.state, "FAILED", RED, True)
        elif external:
            set_pill(self.state, "EXTERNAL", AMBER, False)
        else:
            set_pill(self.state, "STOPPED", DIM, False)
        if running:
            up = proc.uptime()
            self.ident.setText(
                f"sil_twin: {proc.ident or 'starting ... (MuJoCo and the policy load first)'}\n"
                f"pid {proc.pid} · up {int(up) // 60:02d}:{int(up) % 60:02d} · SIL firmware "
                f"stack · 1x real time")
        elif external:
            self.ident.setText("external stream -- not started by this console\n"
                               "(what it runs is not carried by the stream)")
        elif failed:
            self.ident.setText(f"the {proc.err}")
        else:
            self.ident.setText("not running")
        self._ports(st)
        self._twin_lines(st, mono)
        self._stream(mono)
        self._log(failed)

    def _ports(self, st) -> None:
        sim = self.sim
        p = sim.active if sim.proc.running() and sim.active is not None else sim.plan()
        o = self.app.ctl.o
        if p.flight is not None:
            fl = f"flight rec → :{p.flight[1]}"
        elif not simproc.supports_flight(sim.repo):
            fl = "flight rec off (this sil_twin has no --flight)"
        else:
            fl = "flight rec off (port 0)"
        self.ports.setText(f"cmd {p.cmd_port} · tlm {p.tlm_port} · stream {o.stream_port} · {fl}")
        lead = "listening on " if sim.proc.running() else "Start listens on "
        self.ports_why.setText(lead + p.why
                               + ("" if p.own or p.flight is None else
                                  "; its flight records stand in for the robot's"))

    def _twin_lines(self, st, mono) -> None:
        sim = self.sim
        text = sim.twin_text()
        attention = sim.twin and (st.readonly or (sim.active is not None and not sim.active.own))
        self.twin_state.setText(text)
        self.twin_state.setStyleSheet(f"color:{AMBER if attention else DIM}; font-size:12px")
        n, t, rate = self._teed
        if mono - t >= 1.0:
            rate = (st.teed - n) / (mono - t) if st.teed >= n else 0.0
            self._teed = (st.teed, mono, rate)
        if st.tee_on:
            dest = f"127.0.0.1:{sim.tee_to[1]}" if sim.tee_to else "?"
            tee = f"tee  on → {dest} · {st.teed} frames teed ({rate:.0f}/s)"
        else:
            tee = f"tee  off · {st.teed} frames teed"
        o = self.app.ctl.o
        if getattr(self.app, "flight", None) is None:
            fl = "sim flight records: no receiver in this console"
        elif self._fl_at is None:
            fl = f"sim flight records: none yet on :{o.sim_flight_port}"
        else:
            age = mono - self._fl_at
            fl = (f"sim flight records: arriving, {age:.1f} s old" if age < FLIGHT_STALE_S
                  else f"sim flight records: STALE, last {age:.1f} s ago")
        self.tee.setText(f"{tee}\n{fl}")

    def _stream(self, mono) -> None:
        s = self.sim.stream
        if s is None:
            self.stream_line.setText("no stream -- press Start, or pass --sim-view")
            self.stream_note.setText("")
            return
        n, t, fps = self._frames
        if mono - t >= 1.0:
            fps = (s.frames - n) / (mono - t) if s.frames >= n else 0.0
            self._frames = (s.frames, mono, fps)
        state = s.state(mono)
        age = s.age_s(mono)
        last = f"last frame {age:.1f} s" if age is not None else "no frame yet"
        self.stream_line.setText(f"{s.url} · {fps:.1f} fps · {last} · {state}")
        if self.sim.external():
            self.stream_note.setText("external stream: this console did not start it, so the "
                                     "controls above do not describe it. Start replaces it.")
        elif state == "STALE":
            self.stream_note.setText(f"STALE: no frame for {age:.1f} s")
        elif s.error and state != "live":
            self.stream_note.setText(s.error)
        else:
            self.stream_note.setText("")

    def _log(self, failed: bool) -> None:
        proc = self.sim.proc
        new = proc.lines(self._logged)
        self._logged = proc.total
        if new:
            bar = self.log.verticalScrollBar()
            at_end = bar.value() >= bar.maximum() - 2
            for t, text in new:
                stamp = time.strftime("%H:%M:%S", time.gmtime(t)) + f".{int((t % 1) * 1000):03d}"
                self.log.appendPlainText(f"{stamp}  {text}")
            if at_end:
                bar.setValue(bar.maximum())
        if failed and not self._was_failed:
            self._show_log(True)                # opened for you: the reason is in there
        self._was_failed = failed
        self.fail.setText(f"the {proc.err} -- its last lines are below" if failed else "")
