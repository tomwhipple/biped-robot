"""Setup: the link (applied on the button), the role, the beacon relay, data
sources, the keymap (~/.bimo/keymap.conf, the same file and format as
bimo_gui), and a screenshot (design §6)."""
from __future__ import annotations

import os

from PyQt6 import QtCore, QtWidgets

from . import core
from .ui import AMBER, DIM, Columns, btn, group, hbox, lab, set_cell, table, vbox, wrap


def edit(text, width=None, shrink=False):
    """A line edit; `shrink` makes `width` a minimum, not a fixed width, so the
    row gives way before the tab outgrows its pane."""
    e = QtWidgets.QLineEdit(text)
    if width and shrink:
        e.setMinimumWidth(width)
    elif width:
        e.setFixedWidth(width)
    return e


def form(rows):
    f = QtWidgets.QFormLayout()
    f.setHorizontalSpacing(16)
    f.setVerticalSpacing(8)
    for k, v in rows:
        f.addRow(lab(k, DIM), v)
    return f


class SetupTab(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        o = app.ctl.o
        # -- link ----------------------------------------------------------
        self.host = edit(o.host or "", 120, shrink=True)
        self.cmd_port = edit(str(o.cmd_port), 80)
        self.tlm_port = edit(str(o.tlm_port), 80)
        apply_btn = btn("apply && reconnect", 160, h=32)
        apply_btn.clicked.connect(self._apply_link)
        self.role = lab("", AMBER)
        self.role.setWordWrap(True)
        self.watch_on = QtWidgets.QCheckBox("forward every beacon to")
        self.watch_on.setChecked(bool(o.watch))
        self.watch_on.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.watch = edit(o.watch or "", 100, shrink=True)
        watch_btn = btn("apply", 80, h=30)
        watch_btn.clicked.connect(self._apply_watch)
        relink = lab("a relink while armed disarms first", DIM)
        relink.setWordWrap(True)
        link = group("LINK   edits apply on the button", vbox(form([
            ("robot host", self.host),
            ("ports", hbox(self.cmd_port, self.tlm_port, lab("cmd / tlm", DIM), "stretch")),
            ("role", self.role),
            ("relay (--watch)", hbox(self.watch_on, self.watch, watch_btn, "stretch")),
        ]), hbox(apply_btn, relink, "stretch", spacing=10)))
        # -- data sources ----------------------------------------------------
        self.sources = lab("", DIM, mono=True)
        self.sources.setWordWrap(True)      # long paths wrap rather than widen the tab
        self.sources.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        data = group("DATA SOURCES   (command line)", vbox(self.sources))
        # -- keymap ----------------------------------------------------------
        self.keys = table(["action", "key"], core.action_count(), stretch_col=0)
        self.keys.cellClicked.connect(self._rebind)
        self.key_hint = lab("click a binding, then press a key (Escape cancels)", DIM, 11)
        restore = btn("restore defaults", 160, h=30)
        restore.clicked.connect(self._restore)
        save = btn("save", 90, h=30)
        save.clicked.connect(self._save)
        key_path = lab(app.ctl.keymap_path or "(no $HOME)", DIM, 11)
        key_path.setWordWrap(True)          # a long path wraps; a group title cannot
        keyg = group("KEYMAP", vbox(key_path, self.keys, self.key_hint,
                                    hbox(restore, save, "stretch")))
        # -- screenshot ------------------------------------------------------
        self.shot = edit(os.path.join(o.sessions, "console.png"))
        self.shot.setMinimumWidth(120)
        shot_btn = btn("save screenshot", 160, h=30)
        shot_btn.clicked.connect(lambda: app.screenshot(self.shot.text()))
        shotg = group("SCREENSHOT   the window, no screen-recording permission",
                      hbox(self.shot, shot_btn, "stretch"))
        self.gamepad_box = QtWidgets.QVBoxLayout()
        self.gamepad_label = lab("", DIM)          # without gamepad support: says so
        self.gamepad_box.addWidget(self.gamepad_label)
        padg = group("GAMEPAD", self.gamepad_box)
        self.cols = Columns(wrap(vbox(link, data, shotg, spacing=10)),
                            wrap(vbox(keyg, padg, spacing=10)), spacing=12)
        self.setLayout(vbox(self.cols, "stretch", margins=(12, 12, 12, 12)))
        self.refresh_keys()

    def _apply_link(self):
        try:
            cp, tp = int(self.cmd_port.text()), int(self.tlm_port.text())
        except ValueError:
            self.app.ctl.say_status("ports must be numbers")
            return
        self.app.ctl.relink(self.host.text().strip(), cp, tp)

    def _apply_watch(self):
        spec = self.watch.text().strip() if self.watch_on.isChecked() else None
        self.app.ctl.o.watch = spec
        if not self.app.ctl.set_watch(spec):
            self.watch_on.setChecked(False)
            self.app.ctl.say_status(f"--watch {spec}: expected HOST[:PORT] with a dotted-quad host")

    def _rebind(self, row, _col):
        # The window decides (app.start_rebind): never the stop keys, never
        # while armed or LIVE; it says REBINDING in the safety bar.
        if self.app.start_rebind(row):
            self.key_hint.setText(f"REBINDING `{core.action_label(row)}`: press a key "
                                  f"(Escape cancels)")

    def _restore(self):
        self.app.ctl.core.reset_keys()
        self.app.keymap_changed()

    def _save(self):
        path = self.app.ctl.keymap_path
        ok, err = self.app.ctl.core.save_keys(path) if path else (False, "no keymap path")
        self.app.ctl.say_status(f"keymap saved -> {path}" if ok else f"keymap: {err}")

    def refresh_keys(self):
        c = self.app.ctl.core
        for a in range(core.action_count()):
            set_cell(self.keys, a, 0, core.action_label(a))
            set_cell(self.keys, a, 1, core.key_name(c.key_for(a)) or "none")
        self.key_hint.setText("click a binding, then press a key (Escape cancels)")

    def update_from(self, st, now: float) -> None:
        o = self.app.ctl.o
        nopad = getattr(self.app, "pad", None) is None
        self.gamepad_label.setVisible(nopad)
        if nopad:
            self.gamepad_label.setText(self.app.pad_detail())
        if not getattr(self.app.ctl, "linked", True):
            self.role.setText("NO LINK: the last relink failed, so this console sends "
                              "nothing; fix the address and apply")
        else:
            self.role.setText("OBSERVER: sends nothing until TAKE CONTROL; HOLD / E-STOP work"
                              if st.readonly else "DRIVING: this console commands the robot")
        self.sources.setText("\n".join([
            f"flight records  :{o.flight_port} (robot, via the collector's --forward)",
            f"sim flight      :{o.sim_flight_port}",
            f"3D plant        {self.app.plant_path()}",
            f"sim stream      {o.sim_view or f'http://127.0.0.1:{o.stream_port}/'}",
            f"camera relay    {o.cam_relay or '(none: --cam-relay)'}",
            f"bench telemetry {o.bench_telemetry or '(none: --bench-telemetry)'}",
            f"sessions        {o.sessions}",
            f"stops go to     {self.app.stop_dest_text()}",
        ]))
