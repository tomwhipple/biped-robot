"""Render the operator console mockups (docs/design-v6/2026-11-01-operator-console-ui.md).

A static PyQt6 + pyqtgraph window filled with SYNTHETIC data, grabbed to PNGs
beside this file. The 3D views are real MuJoCo renders of the v6 plant posed
from synthetic joint angles; ghosts are separate geometry (mjv_addGeoms from a
second MjData, rgba overridden), never a pixel blend. Camera panes are
placeholders: no camera frame is faked.

    uv run --no-project --with PyQt6 --with pyqtgraph --with mujoco==3.15.0 \\
        python docs/design-v6/figs/console_ui_mockups.py
"""
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
XML = REPO / "sim" / "bimo_biped_v6ar.xml"

W, H = 1920, 1080
BG, PANEL, LINE = "#16181c", "#1e2126", "#343941"
TEXT, DIM, FAINT = "#d6d9de", "#8a9099", "#5a6068"
GREEN, AMBER, RED, ORANGE, BENCHC = "#2fa35a", "#c98a1b", "#c62828", "#d9731f", "#7f8ca6"
CYAN, SIMC, BLUE = "#3cc8ff", "#ffa41f", "#2f6fca"
MONO = "Menlo"
MOCK = "MOCKUP · synthetic data, not a measurement · proposed operator console layout"

pg.setConfigOptions(antialias=True, background=BG, foreground=TEXT)

QSS = f"""
QWidget {{ font-size: 13px; color: {TEXT}; }}
QMainWindow, QWidget#root {{ background: {PANEL}; }}
QFrame#safety {{ background: #111317; border-bottom: 1px solid #2c3036; }}
QGroupBox {{ border: 1px solid {LINE}; border-radius: 7px; margin-top: 16px;
            padding: 10px 10px 8px 10px; background: #1b1e23; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 5px;
                   color: #9aa3ad; font-weight: 700; letter-spacing: 1px; }}
QPushButton {{ background: #2a2e35; border: 1px solid #3a3f47; border-radius: 7px;
              padding: 7px 14px; }}
QPushButton:disabled {{ background: #202328; color: #555b63; border-color: #2c3036; }}
QPushButton#held {{ background: {BLUE}; border: 1px solid #5a95e8; color: white;
                   font-weight: 700; }}
QPushButton#estop {{ background: {RED}; color: white; font-size: 19px; font-weight: 800;
                    border: 2px solid #ff6b60; border-radius: 8px; }}
QPushButton#estop_latched {{ background: #ff2d20; color: white; font-size: 19px;
                            font-weight: 800; border: 3px solid #ffd0cc; border-radius: 8px; }}
QPushButton#hold {{ background: #9a4a10; color: #fff0e0; font-size: 16px; font-weight: 800;
                   border: 2px solid {ORANGE}; border-radius: 8px; }}
QPushButton#reset {{ background: #7d540c; color: #fff1d6; font-weight: 700;
                    border: 1px solid {AMBER}; }}
QPushButton#reset:disabled {{ background: #2a2418; color: #6b5a3a; border-color: #3a3222; }}
QPushButton#arm {{ font-weight: 700; }}
QPushButton#watch {{ background: #3a2c0c; color: #f0c56a; border: 1px solid #6b5016; }}
QPushButton#take {{ background: #5a4210; color: #ffe2a0; border: 1px solid {AMBER};
                   font-weight: 700; }}
QTabWidget::pane {{ border: 1px solid {LINE}; border-radius: 7px; top: -1px;
                   background: {PANEL}; }}
QTabBar::tab {{ background: #23272d; padding: 8px 20px; border: 1px solid {LINE};
               border-bottom: none; border-top-left-radius: 7px;
               border-top-right-radius: 7px; margin-right: 3px; color: #9aa3ad; }}
QTabBar::tab:selected {{ background: {PANEL}; color: white; font-weight: 700; }}
QToolButton {{ background: #2a2e35; border: 1px solid #3a3f47; padding: 5px 13px; }}
QToolButton:checked {{ background: {BLUE}; color: white; border-color: #5a95e8;
                      font-weight: 700; }}
QStatusBar {{ background: #111317; color: {AMBER}; border-top: 1px solid #2c3036; }}
QSlider::groove:horizontal {{ height: 5px; background: #343941; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {BLUE}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: #d6d9de; width: 14px; margin: -6px 0;
                             border-radius: 7px; }}
QComboBox, QLineEdit, QSpinBox {{ background: #2a2e35; border: 1px solid #3a3f47;
            border-radius: 6px; padding: 4px 10px; }}
QSplitter::handle {{ background: #2c3036; }}
QTableWidget {{ background: {BG}; gridline-color: #2c3036; border: 1px solid {LINE};
               border-radius: 6px; }}
QHeaderView::section {{ background: #23272d; color: #9aa3ad; border: none;
                       border-bottom: 1px solid {LINE}; padding: 5px 8px; font-weight: 700; }}
QPlainTextEdit {{ background: #0f1114; border: 1px solid {LINE}; border-radius: 6px;
                 font-family: {MONO}; font-size: 12px; color: #b8bec6; }}
"""


def dark_palette():
    p = QtGui.QPalette()
    R = QtGui.QPalette.ColorRole
    for role, col in [(R.Window, PANEL), (R.WindowText, TEXT), (R.Base, BG),
                      (R.AlternateBase, "#1b1e23"), (R.Text, TEXT), (R.Button, "#2a2e35"),
                      (R.ButtonText, TEXT), (R.Highlight, BLUE),
                      (R.HighlightedText, "#ffffff"), (R.PlaceholderText, DIM)]:
        p.setColor(role, QtGui.QColor(col))
    return p


# -- small builders ----------------------------------------------------------
def lab(text, color=None, size=None, bold=False, mono=False, rich=False):
    w = QtWidgets.QLabel(text)
    css = []
    if color:
        css.append(f"color:{color}")
    if size:
        css.append(f"font-size:{size}px")
    if bold:
        css.append("font-weight:700")
    if mono:
        css.append(f"font-family:{MONO}")
    if css:
        w.setStyleSheet(";".join(css))
    if rich:
        w.setTextFormat(QtCore.Qt.TextFormat.RichText)
    return w


def btn(text, key=None, width=None, name=None, h=44, enabled=True):
    b = QtWidgets.QPushButton(f"{text}   [{key}]" if key else text)
    if name:
        b.setObjectName(name)
    if width:
        b.setFixedWidth(width)
    b.setFixedHeight(h)
    b.setEnabled(enabled)
    return b


def _box(layout, items):
    for it in items:
        if it == "stretch":
            layout.addStretch(1)
        elif isinstance(it, int):
            layout.addSpacing(it)
        elif isinstance(it, QtWidgets.QLayout):
            layout.addLayout(it)
        else:
            layout.addWidget(it)
    return layout


def hbox(*items, spacing=8, margins=(0, 0, 0, 0)):
    l = QtWidgets.QHBoxLayout()
    l.setSpacing(spacing)
    l.setContentsMargins(*margins)
    return _box(l, items)


def vbox(*items, spacing=6, margins=(0, 0, 0, 0)):
    l = QtWidgets.QVBoxLayout()
    l.setSpacing(spacing)
    l.setContentsMargins(*margins)
    return _box(l, items)


def wrap(layout, name=None):
    w = QtWidgets.QWidget()
    if name:
        w.setObjectName(name)
    w.setLayout(layout)
    return w


def group(title, layout):
    g = QtWidgets.QGroupBox(title)
    g.setLayout(layout)
    return g


def pill(text, color, filled=True, width=None):
    w = lab(text)
    w.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    bg = color if filled else "transparent"
    fg = "white" if filled else color
    w.setStyleSheet(f"background:{bg}; color:{fg}; border:1px solid {color}; border-radius:10px;"
                    f" padding:2px 10px; font-weight:700; font-size:12px;")
    if width:
        w.setFixedWidth(width)
    return w


def segmented(labels, checked=0):
    l = QtWidgets.QHBoxLayout()
    l.setSpacing(0)
    grp = QtWidgets.QButtonGroup(l)
    for i, t in enumerate(labels):
        b = QtWidgets.QToolButton()
        b.setText(t)
        b.setCheckable(True)
        b.setChecked(i == checked)
        grp.addButton(b)
        l.addWidget(b)
    return l


def colored_check(text, color, checked):
    c = QtWidgets.QCheckBox(text)
    c.setChecked(checked)
    c.setStyleSheet(f"QCheckBox {{ color: {color}; font-weight: 600; }}")
    return c


def table(headers, rows, colors=None, stretch_col=None, height=None):
    t = QtWidgets.QTableWidget(len(rows), len(headers))
    t.setHorizontalHeaderLabels(headers)
    t.verticalHeader().setVisible(False)
    t.setShowGrid(False)
    t.setAlternatingRowColors(True)
    t.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    for r, row in enumerate(rows):
        for c, v in enumerate(row):
            it = QtWidgets.QTableWidgetItem(str(v))
            if colors and colors.get((r, c)):
                it.setForeground(QtGui.QColor(colors[(r, c)]))
            t.setItem(r, c, it)
    hh = t.horizontalHeader()
    hh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
    if stretch_col is not None:
        hh.setSectionResizeMode(stretch_col, QtWidgets.QHeaderView.ResizeMode.Stretch)
    t.verticalHeader().setDefaultSectionSize(26)
    t.setFixedHeight(height or 26 * len(rows) + 34)
    t.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    return t


def mkplot(h=None, xr=(-10, 0), yr=None, bottom=True, left_w=38):
    p = pg.PlotWidget()
    p.setBackground(BG)
    pi = p.getPlotItem()
    pi.showGrid(x=True, y=True, alpha=0.12)
    pi.setMenuEnabled(False)
    pi.hideButtons()
    pi.setMouseEnabled(False, False)
    pi.setXRange(*xr, padding=0)
    if yr:
        pi.setYRange(*yr, padding=0.06)
    for ax in ("left", "bottom"):
        a = pi.getAxis(ax)
        a.setTextPen(DIM)
        a.setPen("#3a3f47")
        a.setStyle(tickFont=QtGui.QFont(MONO, 9))
    pi.getAxis("left").setWidth(left_w)
    if not bottom:
        pi.hideAxis("bottom")
    if h:
        p.setFixedHeight(h)
    return p


def slider_row(name, lo, hi, val, fmt, note=None):
    s = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
    s.setRange(0, 1000)
    s.setValue(int(1000 * (val - lo) / (hi - lo)))
    items = [lab(name, DIM), s, lab(fmt.format(val), mono=True)]
    if note:
        items.append(lab(note, AMBER, 11))
    return hbox(*items, spacing=10)


# -- synthetic robot ---------------------------------------------------------
BUS_ID = {"R_hip_roll": 1, "R_hip_pitch": 2, "R_knee": 3, "R_ankle": 4,
          "L_hip_roll": 5, "L_hip_pitch": 6, "L_knee": 7, "L_ankle": 8,
          "R_hip_yaw": 9, "L_hip_yaw": 10, "R_ankle_roll": 11, "L_ankle_roll": 12,
          "neck_yaw": 13, "R_shoulder": 14, "R_elbow": 15, "L_shoulder": 16, "L_elbow": 17}
NAMES = list(BUS_ID)
LEGS = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll"]
ARMS = ["shoulder", "elbow"]
NOT_FITTED = {"neck_yaw", "L_shoulder", "R_shoulder", "L_elbow", "R_elbow"}
FITTED = [n for n in NAMES if n not in NOT_FITTED]
FAULT_JOINT, FAULT_T = "R_ankle_roll", -1.6
GAIT_HZ, LAG = 1.4, 0.06
T = np.round(np.arange(-5.0, 0.00001, 0.02), 3)        # flight recorder: 50 Hz


def gait(phase, amp=1.0):
    s, c = math.sin(phase), math.cos(phase)
    return {
        "L_hip_pitch": -18 * amp * s - 12, "R_hip_pitch": 18 * amp * s - 12,
        "L_knee": -(28 + 18 * amp * max(0.0, c)), "R_knee": -(28 + 18 * amp * max(0.0, -c)),
        "L_ankle": 10 + 8 * amp * s, "R_ankle": 10 - 8 * amp * s,
        "L_hip_roll": 3 * s, "R_hip_roll": 3 * s,
        "L_ankle_roll": -3 * s, "R_ankle_roll": -3 * s,
        "L_hip_yaw": 4 * s + 3, "R_hip_yaw": -4 * s + 3,
    }


def cmd_at(t, amp=1.0, dphase=0.0):
    q = gait(2 * math.pi * GAIT_HZ * t + dphase, amp)
    return {n: q.get(n, 0.0) for n in NAMES}


rng = np.random.default_rng(7)
CMD = {n: np.array([cmd_at(t)[n] for t in T]) for n in NAMES}
MEAS, LOAD = {}, {}
for _n in NAMES:
    lagged = np.array([cmd_at(t - LAG)[_n] for t in T])
    mean = lagged.mean()
    m = mean + 0.92 * (lagged - mean) + rng.normal(0, 0.25, len(T))
    if _n == FAULT_JOINT:
        k = int(np.searchsorted(T, FAULT_T))
        m[k:] = m[k]                    # a faulted servo's reading is its last good angle
    MEAS[_n] = m
    LOAD[_n] = 18 + 14 * np.abs(np.gradient(lagged, T)) / 60 + rng.normal(0, 1.2, len(T))
SIM = {n: np.array([cmd_at(t, 1.12, 0.35)[n] for t in T]) for n in NAMES}


def now(series):
    return {n: float(series[n][-1]) for n in FITTED}


# -- 3D view -----------------------------------------------------------------
class PoseRenderer:
    def __init__(self, w, h):
        self.m = mujoco.MjModel.from_xml_path(str(XML))
        self.r = mujoco.Renderer(self.m, height=h, width=w)
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = 0
        self.opt.geomgroup[0] = self.opt.geomgroup[2] = 1
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        dim_bodies = set()
        for j in NOT_FITTED:
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)
            dim_bodies.add(int(self.m.jnt_bodyid[jid]))
        for b in range(self.m.nbody):            # descendants of an unfitted joint
            p = b
            while p > 0:
                if p in dim_bodies:
                    dim_bodies.add(b)
                    break
                p = int(self.m.body_parentid[p])
        self.dim_geoms = {g for g in range(self.m.ngeom) if int(self.m.geom_bodyid[g]) in dim_bodies}

    def data_for(self, q_deg):
        d = mujoco.MjData(self.m)
        d.qpos[3] = 1.0
        for name, v in q_deg.items():
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            d.qpos[self.m.jnt_qposadr[jid]] = math.radians(v)
        mujoco.mj_kinematics(self.m, d)
        lows = []
        for i in range(self.m.ngeom):
            if i == self.floor or not self.opt.geomgroup[self.m.geom_group[i]]:
                continue
            R = d.geom_xmat[i].reshape(3, 3)
            ext = np.abs(R) @ self.m.geom_aabb[i][3:]
            lows.append(d.geom_xpos[i][2] + (R @ self.m.geom_aabb[i][:3])[2] - ext[2])
        d.qpos[2] -= min(lows)                   # root pinned: lowest point on the floor
        mujoco.mj_kinematics(self.m, d)
        return d

    def render(self, meas, ghosts=(), azimuth=118, elevation=-9, distance=1.08):
        dm = self.data_for(meas)
        tor = dm.xpos[1]
        self.cam.lookat[:] = (tor[0], tor[1], tor[2] * 0.62)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = azimuth, elevation, distance
        self.r.update_scene(dm, camera=self.cam, scene_option=self.opt)
        scn = self.r.scene
        for i in range(scn.ngeom):
            if scn.geoms[i].objid in self.dim_geoms and scn.geoms[i].objtype == mujoco.mjtObj.mjOBJ_GEOM:
                scn.geoms[i].rgba[:] = (0.45, 0.47, 0.5, 0.45)
        for q, rgba in ghosts:
            dg = self.data_for(q)
            n0 = scn.ngeom
            mujoco.mjv_addGeoms(self.m, dg, self.opt, mujoco.MjvPerturb(),
                                mujoco.mjtCatBit.mjCAT_DYNAMIC, scn)
            for i in range(n0, scn.ngeom):
                if scn.geoms[i].objid in self.dim_geoms:
                    scn.geoms[i].rgba[3] = 0.0
                else:
                    scn.geoms[i].rgba[:] = rgba
        img = self.r.render()
        return QtGui.QImage(img.data, img.shape[1], img.shape[0], 3 * img.shape[1],
                            QtGui.QImage.Format.Format_RGB888).copy()


_RENDERERS = {}


def render3d(w, h, cmd=True, sim=False, **cam):
    pr = _RENDERERS.get((w, h)) or _RENDERERS.setdefault((w, h), PoseRenderer(w, h))
    ghosts = []
    if cmd:
        ghosts.append((now(CMD), (0.24, 0.78, 1.0, 0.24)))
    if sim:
        ghosts.append((now(SIM), (1.0, 0.64, 0.12, 0.26)))
    return QtGui.QPixmap.fromImage(pr.render(now(MEAS), ghosts, **cam))


def paint_legend(p, cmd, sim, x=14, y=14):
    rows = [("#e8e8e8", "measured   solid", "0.02 s"),
            (CYAN, "commanded  ghost", "0.02 s") if cmd else (FAINT, "commanded  off", ""),
            (SIMC, "sim        ghost", "0.04 s") if sim else (FAINT, "sim        off", ""),
            ("#73777d", "not fitted zero pose", "dim")]
    p.setBrush(QtGui.QColor(10, 12, 15, 205))
    p.setPen(QtGui.QPen(QtGui.QColor("#3a3f47")))
    p.drawRoundedRect(QtCore.QRectF(x, y, 304, 22 * len(rows) + 14), 8, 8)
    p.setFont(QtGui.QFont(MONO, 11))
    for i, (col, txt, age) in enumerate(rows):
        yy = y + 22 + i * 22
        p.fillRect(QtCore.QRectF(x + 12, yy - 10, 12, 12), QtGui.QColor(col))
        p.setPen(QtGui.QColor(TEXT if col != FAINT else DIM))
        p.drawText(QtCore.QPointF(x + 34, yy), txt)
        p.setPen(QtGui.QColor(DIM))
        p.drawText(QtCore.QPointF(x + 238, yy), age)


def paint_camera(p, r, title="bench cam · side   mira:8646"):
    p.setBrush(QtGui.QColor("#0b0d10"))
    p.setPen(QtGui.QPen(QtGui.QColor(FAINT), 1.5))
    p.drawRoundedRect(r, 6, 6)
    p.setPen(QtGui.QPen(QtGui.QColor("#2c3036"), 1, QtCore.Qt.PenStyle.DashLine))
    p.drawLine(r.topLeft() + QtCore.QPointF(10, 30), r.bottomRight() - QtCore.QPointF(10, 10))
    p.drawLine(r.topRight() + QtCore.QPointF(-10, 30), r.bottomLeft() + QtCore.QPointF(10, -10))
    p.fillRect(QtCore.QRectF(r.x() + 1, r.y() + 1, r.width() - 2, 24), QtGui.QColor("#1a1d22"))
    p.setFont(QtGui.QFont(MONO, 10))
    p.setPen(QtGui.QColor(TEXT))
    p.drawText(QtCore.QPointF(r.x() + 10, r.y() + 17), title)
    p.setPen(QtGui.QColor(GREEN))
    p.drawText(QtCore.QPointF(r.right() - 58, r.y() + 17), "● live")
    p.setPen(QtGui.QColor(DIM))
    p.setFont(QtGui.QFont(MONO, 11))
    p.drawText(QtCore.QRectF(r.x(), r.y() + 26, r.width(), r.height() - 26),
               QtCore.Qt.AlignmentFlag.AlignCenter, "camera frame\n(not registered to 3D)")


def view_image(w, h, cmd=True, sim=False, camera="pip", hint=True):
    pm = render3d(w, h, cmd, sim)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    paint_legend(p, cmd, sim)
    if camera == "pip":
        paint_camera(p, QtCore.QRectF(w - 314, h - 204, 300, 190))
    if hint:
        p.setFont(QtGui.QFont(MONO, 10))
        p.setPen(QtGui.QColor(DIM))
        p.drawText(QtCore.QRectF(w - 330, 12, 316, 40), QtCore.Qt.AlignmentFlag.AlignRight,
                   "drag: orbit · right-drag: pan\nwheel: zoom · dbl-click: reset")
    p.end()
    return pm


# -- safety bar --------------------------------------------------------------
LIVE_STATE = dict(badge=("●  LIVE", GREEN), driver=True, armed=True, estop=False,
                  vbat="11.8 V", servos=("servos ok", GREEN, "loop late 0 %"),
                  pad=("PAD  ● R held", GREEN, "drives · A (east) = E-STOP"),
                  rec=True, lost=None, bench=None,
                  note="arm sent → robot left BENCH, now LIVE · driving from the gamepad (R held)")


def safety_bar(s=LIVE_STATE):
    f = QtWidgets.QFrame()
    f.setObjectName("safety")
    text, col = s["badge"]
    badge = lab(text)
    badge.setFixedHeight(46)
    badge.setMinimumWidth(170)
    badge.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    badge.setStyleSheet(f"background:{col}; color:white; font-weight:800; font-size:19px;"
                        f" border-radius:8px; padding:4px 18px;")
    drv = s["driver"]
    arm = btn("DISARM" if s["armed"] else "ARM", "a", 132, "arm", enabled=drv)
    stand = btn("STAND", "space", 132, enabled=drv)
    hold = btn("HOLD", "f", 132, "hold", h=46)            # soft stop: out of band, any role
    if s["estop"]:
        estop = btn("E-STOP LATCHED", "e", 230, "estop_latched", h=46)
    else:
        estop = btn("E-STOP", "e", 230, "estop", h=46)    # hard stop: out of band, any role
    reset = btn("RESET SERVOS", "h", 190, "reset", enabled=drv)
    ctl = (btn("WATCH ONLY", None, 140, "watch", h=34) if drv
           else btn("TAKE CONTROL", None, 150, "take", h=34))
    sv_t, sv_c, sv_sub = s["servos"]
    pad_t, pad_c, pad_sub = s["pad"]
    vb = vbox(lab(s["vbat"], "#e8e8e8", 22, True, True), lab("pack · lands at 9.9 V", DIM, 11),
              spacing=0)
    sv = vbox(lab(sv_t, sv_c, 14, True), lab(sv_sub, DIM, 11), spacing=2)
    pad = vbox(lab(pad_t, pad_c, 14, True), lab(pad_sub, DIM, 11), spacing=2)
    rec = vbox(lab("● REC" if s["rec"] else "○ not recording", "#ff5a50" if s["rec"] else DIM,
                   14, True), lab("02:41 · 3218 cmd / 1609 tlm" if s["rec"] else "", DIM, 11),
               spacing=2)
    row1 = hbox(badge, 6, arm, stand, 14, hold, estop, 22, reset, "stretch", vb, 20, sv, 20, pad, 20,
                rec, 20, ctl, spacing=10)

    sep = "<span style='color:#4a5059'>&nbsp;&nbsp;│&nbsp;&nbsp;</span>"
    d = f"<span style='color:{DIM}'>"
    if drv:
        tx = (f"{d}→</span> bimo.local:4210&nbsp;&nbsp;20&nbsp;Hz&nbsp;&nbsp;24&nbsp;B&nbsp;&nbsp;"
              f"{d}ARM ENABLE POSE ATT</span>")
    else:
        tx = f"<span style='color:{AMBER}'>OBSERVER · no commands · stops are out of band</span>"
    if s["lost"]:
        rx = f"<span style='color:#ff5a50'>LINK LOST&nbsp;&nbsp;{s['lost']}&nbsp;s ago</span>"
    elif drv:
        rx = (f"{d}←</span> beacon&nbsp;&nbsp;98&nbsp;ms&nbsp;&nbsp;10.1&nbsp;Hz&nbsp;&nbsp;"
              f"lag&nbsp;1&nbsp;&nbsp;bad&nbsp;0")
    else:
        rx = f"{d}← beacon&nbsp;&nbsp;none: the robot beacons only to its commander</span>"
    fr = (f"{d}flight rec</span>&nbsp;mira:4220&nbsp;→&nbsp;here&nbsp;&nbsp;50.0&nbsp;Hz&nbsp;&nbsp;"
          f"holes&nbsp;0")
    clock = f"robot clock {d}synced&nbsp;Δ</span>&nbsp;+3&nbsp;ms"
    row2 = lab(tx + sep + rx + sep + fr + sep + clock, mono=True, rich=True)
    lines = [row1, row2]
    if s["bench"]:
        lines.append(lab(f"<span style='color:{AMBER}; font-weight:700'>BENCH:</span>&nbsp;&nbsp;"
                         f"<span style='color:#f0c56a'>{s['bench']}</span>", rich=True))
    lines.append(lab(f"<span style='color:{DIM}'>note</span>&nbsp;&nbsp;{s['note']}", rich=True))
    f.setLayout(vbox(*lines, spacing=7, margins=(14, 12, 14, 9)))
    f.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Fixed)
    return f


# -- view pane ---------------------------------------------------------------
def view_pane(w, h):
    combo = QtWidgets.QComboBox()
    combo.addItems(["view: side cam ≈", "front", "rear", "top"])
    top = hbox(segmented(["3D pose", "Camera", "Sim render"]), 18,
               colored_check("Commanded", CYAN, True), colored_check("Sim", SIMC, False),
               colored_check("Camera PiP", TEXT, True), "stretch", combo, spacing=12)
    img = QtWidgets.QLabel()
    img.setPixmap(view_image(w, h))
    img.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    status = lab(f"<span style='color:{GREEN}'>● pose live</span>&nbsp;&nbsp;"
                 f"<span style='color:{DIM}'>flight recorder 50 Hz · 12 fitted joints + IMU · "
                 f"0.02 s old · root pinned, yaw drawn 0 · kinematics only, no physics</span>",
                 rich=True)
    return wrap(vbox(top, img, status, "stretch", spacing=8, margins=(12, 10, 8, 8)))


# -- tabs --------------------------------------------------------------------
def pad_widget():
    class Pad(QtWidgets.QWidget):
        def paintEvent(self, _):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
            p.setFont(QtGui.QFont(MONO, 10))
            for cx, label, (dx, dy) in [(60, "L  vx / vy", (0.0, -0.62)),
                                        (190, "R  wz", (0.35, 0.0))]:
                c = QtCore.QPointF(cx, 62)
                p.setPen(QtGui.QPen(QtGui.QColor("#3a3f47"), 2))
                p.setBrush(QtGui.QColor("#121418"))
                p.drawEllipse(c, 40, 40)
                p.setPen(QtGui.QPen(QtGui.QColor("#2c3036"), 1, QtCore.Qt.PenStyle.DashLine))
                p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
                p.drawEllipse(c, 6, 6)
                p.setPen(QtCore.Qt.PenStyle.NoPen)
                p.setBrush(QtGui.QColor(BLUE))
                p.drawEllipse(c + QtCore.QPointF(dx * 34, dy * 34), 11, 11)
                p.setPen(QtGui.QColor(DIM))
                p.drawText(QtCore.QPointF(cx - 34, 122), label)
            for x, y, t, col, on in [(268, 16, "R  dead-man", GREEN, True),
                                     (268, 50, "A (east)  E-STOP", RED, False),
                                     (268, 84, "arm / reset: console only", FAINT, False)]:
                r = QtCore.QRectF(x, y, 214, 26)
                p.setPen(QtGui.QPen(QtGui.QColor(col), 1.5))
                p.setBrush(QtGui.QColor(col) if on else QtGui.QColor("#121418"))
                p.drawRoundedRect(r, 13, 13)
                p.setPen(QtGui.QColor("white" if on else col))
                p.drawText(r, QtCore.Qt.AlignmentFlag.AlignCenter, t)
            p.setPen(QtGui.QColor(DIM))
            p.drawText(QtCore.QPointF(268, 128), "pad lost = released")
            p.end()

    w = Pad()
    w.setFixedHeight(136)
    return w


def drive_tab():
    grid = QtWidgets.QGridLayout()
    grid.setSpacing(6)
    for t, r, c in [("fwd ↑", 0, 1), ("← strafe", 1, 0), ("back ↓", 1, 1), ("strafe →", 1, 2),
                    ("↺ turn L", 2, 0), ("turn R ↻", 2, 2)]:
        grid.addWidget(btn(t, None, 96, "held" if t == "fwd ↑" else None, h=40), r, c)
    wire = QtWidgets.QGridLayout()
    wire.setHorizontalSpacing(8)
    wire.addWidget(lab("", DIM), 0, 0)
    wire.addWidget(lab("SENT", DIM, 11, True), 0, 1)
    wire.addWidget(lab("POLICY SAW", DIM, 11, True), 0, 2)
    for i, (k, v, saw, u) in enumerate([("vx", "+0.40", "+0.40", "m/s"), ("vy", "+0.00", "+0.00", "m/s"),
                                        ("wz", "+0.25", "+0.25", "rad/s"),
                                        ("crouch", "0.90", "1.00", "pinned")], start=1):
        wire.addWidget(lab(k, DIM, mono=True), i, 0)
        wire.addWidget(lab(v, "#e8e8e8", 16, True, True), i, 1)
        wire.addWidget(lab(saw, AMBER if u == "pinned" else "#e8e8e8", 16, True, True), i, 2)
        wire.addWidget(lab(u, AMBER if u == "pinned" else DIM), i, 3)
    wire.addWidget(lab("source: gamepad", GREEN, 11, True), 5, 0, 1, 4)
    wire.addWidget(lab("policy saw: flight recorder", DIM, 11), 6, 0, 1, 4)
    motion = group("MOTION   hold to move · release stands", hbox(grid, 14, wire, "stretch"))
    gp = group("GAMEPAD   8BitDo SN30 Pro · Bluetooth", vbox(pad_widget()))

    ext = QtWidgets.QVBoxLayout()
    ext.addLayout(slider_row("crouch ", 0.6, 1.0, 0.90, "{:.2f}", "pinned"))
    ext.addLayout(slider_row("foot dx", -0.05, 0.05, 0.0, "{:+.3f} m"))
    ext.addLayout(slider_row("foot dz", -0.05, 0.05, 0.0, "{:+.3f} m"))
    lift = hbox(lab("lift   ", DIM), *[QtWidgets.QRadioButton(t) for t in ("left", "none", "right")],
                "stretch", btn("reset", "0", 110, h=32), spacing=10)
    lift.itemAt(2).widget().setChecked(True)
    ext.addLayout(lift)
    extg = group("EXTENDED CHANNELS   off default → 24 B · reset → 14 B", ext)

    sp = QtWidgets.QVBoxLayout()
    for args in [("vx max", 0.0, 1.0, 0.40, "{:.2f} m/s"), ("vy max", 0.0, 0.3, 0.20, "{:.2f} m/s"),
                 ("wz max", 0.0, 1.0, 0.50, "{:.2f} rad/s"), ("send   ", 5, 100, 20, "{:.0f} Hz")]:
        sp.addLayout(slider_row(*args))
    spg = group("SPEEDS   live", sp)

    t30 = np.arange(-30, 0.0001, 0.1)
    vb = mkplot(96, (-30, 0), (11.4, 12.4), bottom=False)
    vb.getPlotItem().getAxis("left").setTickSpacing(0.4, 0.2)
    vb.plot(t30, 11.86 - 0.0008 * (t30 + 30) + rng.normal(0, 0.012, len(t30)),
            pen=pg.mkPen("#e8e8e8", width=1.6))
    tilt = mkplot(96, (-30, 0), (0, 12), bottom=False)
    tilt.getPlotItem().getAxis("left").setTickSpacing(5, 2.5)
    tilt.plot(t30, 2.2 + 1.1 * np.abs(np.sin(2 * np.pi * GAIT_HZ * t30)) + rng.normal(0, 0.15, len(t30)),
              pen=pg.mkPen("#e8e8e8", width=1.6))
    tilt.addItem(pg.InfiniteLine(10, angle=0, pen=pg.mkPen(AMBER, width=1, style=QtCore.Qt.PenStyle.DashLine)))
    vx = mkplot(110, (-30, 0), (-0.05, 0.5))
    vx.plot(t30, np.where(t30 > -21, 0.40, 0.0),
            pen=pg.mkPen(CYAN, width=1.8, style=QtCore.Qt.PenStyle.DashLine))
    charts = QtWidgets.QGridLayout()
    charts.setVerticalSpacing(2)
    for i, (name, val, plot) in enumerate([("vbat", "11.8 V", vb), ("tilt°", "3.0°", tilt),
                                           ("vx cmd", "+0.40", vx)]):
        charts.addWidget(lab(name, DIM, mono=True), i, 0)
        charts.addWidget(plot, i, 1)
        charts.addWidget(lab(val, "#e8e8e8", 14, True, True), i, 2)
    charts.addWidget(lab("tilt from the IMU up vector; dashed amber = the 10° abort line "
                         "(first-policy-stand §5) · vx est: not estimated by the firmware", DIM, 11), 3, 1)
    charts.setColumnStretch(1, 1)
    chg = group("TELEMETRY   last 30 s · robot time", charts)

    return wrap(vbox(hbox(vbox(motion, gp, spacing=10), vbox(extg, spg, spacing=10), spacing=12),
                     chg, "stretch", spacing=10, margins=(12, 12, 12, 12)))


def joint_cell(name):
    part = name.split("_", 1)[1] if name[1] == "_" else name
    title = part.replace("_", " ")
    f = QtWidgets.QFrame()
    f.setObjectName("cell")
    fault = name == FAULT_JOINT
    f.setStyleSheet(f"QFrame#cell {{ border: {2 if fault else 1}px solid {RED if fault else LINE}; "
                    f"border-radius: 6px; background: #1b1e23; }}")
    if name in NOT_FITTED:
        f.setLayout(hbox(lab(title, DIM, 13, True), 12,
                         lab(f"id {BUS_ID[name]} · not fitted — no goal, no reading", FAINT, 12),
                         "stretch", margins=(10, 4, 10, 4)))
        f.setFixedHeight(34)
        return f
    c, m = CMD[name], MEAS[name]
    rms = float(np.sqrt(np.mean((c - m)[-50:] ** 2)))
    info = vbox(hbox(lab(title, "#e8e8e8", 13, True), "stretch",
                     lab(f"id {BUS_ID[name]}", DIM, 11, mono=True)),
                lab(f"<span style='color:{CYAN}'>{round(c[-1], 1) + 0.0:+6.1f}°</span>&nbsp;"
                    f"<span style='color:#e8e8e8'>{round(m[-1], 1) + 0.0:+6.1f}°</span>"
                    f"<span style='color:{DIM}'>&nbsp;goal/meas</span>", size=12, mono=True, rich=True),
                lab("FAULT id 11 · reading held" if fault else
                    f"|err| {rms:3.1f}° rms · {LOAD[name][-50:].mean():2.0f} % load",
                    RED if fault else (AMBER if rms > 3 else DIM), 11, fault, True),
                spacing=0)
    iw = wrap(info)
    iw.setFixedWidth(218)
    lo, hi = min(c.min(), m.min()), max(c.max(), m.max())
    pad = max(2.0, 0.15 * (hi - lo))
    p = mkplot(None, (-5, 0), (lo - pad, hi + pad), bottom=False, left_w=34)
    p.plot(T, c, pen=pg.mkPen(CYAN, width=1.6, style=QtCore.Qt.PenStyle.DashLine))
    p.plot(T, m, pen=pg.mkPen("#f0f2f5", width=1.8))
    if fault:
        p.addItem(pg.InfiniteLine(FAULT_T, angle=90,
                                  pen=pg.mkPen(RED, width=1, style=QtCore.Qt.PenStyle.DotLine)))
    f.setLayout(hbox(iw, p, spacing=6, margins=(8, 4, 6, 4)))
    f.setFixedHeight(66)
    return f


def joints_tab():
    top = hbox(lab("window", DIM), segmented(["5 s", "10 s", "30 s", "2 min"]), 14,
               btn("pause", None, 80, h=30), 14, segmented(["deg", "rad"]), 14,
               segmented(["goal written", "nominal"]), 20,
               lab(f"<span style='color:{CYAN}'>– – goal</span>&nbsp;&nbsp;&nbsp;"
                   f"<span style='color:#f0f2f5'>—— measured</span>&nbsp;&nbsp;&nbsp;"
                   f"<span style='color:{DIM}'>flight recorder 50 Hz · one SYNC READ per tick · "
                   f"robot time</span>", rich=True),
               "stretch", btn("table view", None, 110, h=30), spacing=8)
    grid = QtWidgets.QGridLayout()
    grid.setHorizontalSpacing(10)
    grid.setVerticalSpacing(5)
    grid.addWidget(lab("LEFT", DIM, 12, True), 0, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
    grid.addWidget(lab("RIGHT", DIM, 12, True), 0, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
    for r, part in enumerate(LEGS + ARMS, start=1):
        grid.addWidget(joint_cell(f"L_{part}"), r, 0)
        grid.addWidget(joint_cell(f"R_{part}"), r, 1)
    grid.addWidget(joint_cell("neck_yaw"), len(LEGS + ARMS) + 1, 0, 1, 2)

    sel = "L_knee"
    gl = pg.GraphicsLayoutWidget()
    gl.setBackground(BG)
    gl.setFixedHeight(214)
    plots = [gl.addPlot(row=i, col=0) for i in range(3)]
    for pi in plots:
        pi.showGrid(x=True, y=True, alpha=0.12)
        pi.hideButtons()
        pi.setMenuEnabled(False)
        pi.setXRange(-5, 0, padding=0)
        for ax in ("left", "bottom"):
            pi.getAxis(ax).setTextPen(DIM)
            pi.getAxis(ax).setPen("#3a3f47")
            pi.getAxis(ax).setStyle(tickFont=QtGui.QFont(MONO, 9))
        pi.getAxis("left").setWidth(48)
    a, e, ld = plots
    for pi in (e, ld):
        pi.setXLink(a)
    a.hideAxis("bottom")
    e.hideAxis("bottom")
    a.plot(T, CMD[sel], pen=pg.mkPen(CYAN, width=2, style=QtCore.Qt.PenStyle.DashLine))
    a.plot(T, MEAS[sel], pen=pg.mkPen("#f0f2f5", width=2))
    e.plot(T, CMD[sel] - MEAS[sel], pen=pg.mkPen(AMBER, width=1.8))
    e.addItem(pg.InfiniteLine(0, angle=0, pen=pg.mkPen("#4a5059")))
    ld.plot(T, LOAD[sel], pen=pg.mkPen("#9b8cff", width=1.6))
    a.setLabel("left", "deg", color=DIM)
    e.setLabel("left", "err°", color=DIM)
    ld.setLabel("left", "load %", color=DIM)
    ld.setLabel("bottom", "s (robot time)", color=DIM)
    detail = group(f"DETAIL   {sel.replace('_', ' ')} · goal / measured · error · load · click any "
                   f"cell · highlighted in the 3D view", vbox(gl))
    return wrap(vbox(top, grid, detail, spacing=8, margins=(12, 10, 12, 10)))


def checks_tab():
    P, C, F = ("PASS", GREEN), ("CHECK", AMBER), ("FAIL", RED)
    rows = [
        ("1", "policy is a real policy", "beacon diag · REFUSED_POLICY never seen", P, "robot_legs_walk14_s1_s128r24"),
        ("2", "pack ≥ 11.4 V", "beacon vbat · flight rec pack mV", P, "11.86 V"),
        ("3", "calibration loaded, 12 leg joints fitted", "flight rec HELLO: cal_ok, fitted", P, "cal ok · 12 of 17 fitted"),
        ("4", "position-loop gains P 32 / D 32", "tether: gains", C, "run re-reads them and refuses"),
        ("5", "every fitted servo answers", "flight rec idle ticks: joint 'answered'", P, "12/12 on 300 of 300 reads"),
        ("6", "IMU live, up ≈ (0, 0, 1)", "flight rec IMU block", P, "up (0.01, −0.02, 1.00)"),
        ("7", "gyro still: horizontal RMS < 0.01 rad/s", "flight rec idle gyro (10 Hz, approx.)", P, "0.004 rad/s"),
        ("8", "shaper 10 Hz (never 0)", "tether: shape", C, ""),
        ("9", "obsfreeze none", "tether: obsfreeze", C, ""),
        ("10", "fall guard on", "flight rec events · tether: fall", P, "guard ON · autohome on"),
        ("10a", "flail guard on (15° / 300 ms, 1.5 rad/s, 20°, 7 rad/s)", "tether: flail", C, ""),
        ("10b", "a stop button open for the run", "this console's HOLD / E-STOP (out of band)", P, "ready · robot addr known"),
        ("10c", "a hand on the servo power switch", "the operator", C, ""),
        ("11", "recorder on, UTC synced, collector live", "flight rec HELLO + packets here", P, "rec ON · NTP synced · 50 Hz"),
        ("12", "home readback: all joints at zero", "flight rec CLI-output event", P, "12 s ago · arm within ~30 s"),
        ("13", "upright, floor clear 1 m, spotter's hands ready", "eyes", C, ""),
        ("—", "filming: both bench cams recording", "tools/cam_record.sh on mira", F, "no recording seen"),
    ]
    colors = {}
    data = []
    for r, (n, item, src, (st, col), val) in enumerate(rows):
        data.append([n, item, "" if st == "CHECK" else st, val, src])
        colors[(r, 2)] = col
        colors[(r, 3)] = "#e8e8e8"
        colors[(r, 4)] = DIM
    t = table(["#", "go / no-go (docs/first-policy-stand.md §2)", "status", "value", "source"],
              data, colors, stretch_col=4)
    for r, (_, _, _, (st, _), _) in enumerate(rows):
        if st == "CHECK":
            t.setCellWidget(r, 2, wrap(hbox(QtWidgets.QCheckBox(), lab("CHECK", AMBER, bold=True),
                                            "stretch", spacing=4, margins=(6, 0, 0, 0))))
    t.setColumnWidth(2, 150)
    head = hbox(pill("NO-GO", RED, True, 110), 12,
                lab("6 tether / eye checks open · 1 failed: filming", "#f0c56a", 15, True),
                "stretch", lab("arm window after home readback", DIM), 8,
                lab("18 s left", "#e8e8e8", 18, True, True), spacing=8)
    note = lab("PASS items are read from data the console already receives; CHECK items are typed on the "
               "tether or seen by eye and ticked by the operator. The console never blocks ARM — the "
               "firmware's own refusals are authoritative — but ARM reads 'ARM · checks open' until the "
               "list is clear.", DIM, 12)
    note.setWordWrap(True)
    abort = table(["abort condition (§5)", "watched from", "now"],
                  [["tilt beyond ~10° not recovering within 1 s", "IMU up vector, 50 Hz", "3.0°"],
                   ["joint chatter faster than ~3 Hz, larger than 1–2°", "goal/pos spectrum per joint", "none"],
                   ["a servo too hot to touch", "flight rec temp per joint", "max 41 °C (R knee)"],
                   ["servo fault bit or LOW-BATT", "beacon servo_err · pack guard", "id 11 faulted"],
                   ["the robot's own flail guard tripped", "flight rec flail event · DISARMED_FLAIL", "not tripped"],
                   ["drift beyond 5 cm, steady rotation", "not observable (no odometry)", "eyes only"]],
                  {(3, 2): RED, (5, 1): FAINT, (5, 2): FAINT}, stretch_col=1)
    return wrap(vbox(group("PRE-ARM", vbox(head, t, note, spacing=10)),
                     group("ABORT WATCH   advisory: the console flags, the operator acts", vbox(abort)),
                     "stretch", spacing=10, margins=(12, 12, 12, 12)))


def telemetry_tab():
    t60 = np.arange(-60, 0.0001, 0.1)
    plots = []

    def chart(name, unit, y, yr, color="#e8e8e8", ref=None, ticks=None):
        p = mkplot(118, (-60, 0), yr, bottom=False, left_w=44)
        if ticks:
            p.getPlotItem().getAxis("left").setTickSpacing(*ticks)
        p.plot(t60, y, pen=pg.mkPen(color, width=1.6))
        if ref is not None:
            p.addItem(pg.InfiniteLine(ref, angle=0, pen=pg.mkPen(AMBER, width=1,
                                                                 style=QtCore.Qt.PenStyle.DashLine)))
        plots.append((name, unit, f"{y[-1]:.2f}" if abs(y[-1]) < 100 else f"{y[-1]:.0f}", p))

    chart("vbat", "V", 11.9 - 0.0006 * (t60 + 60) + rng.normal(0, 0.01, len(t60)), (11.4, 12.4),
          ref=None, ticks=(0.4, 0.2))
    chart("tilt", "°", 2.2 + np.abs(np.sin(2 * np.pi * GAIT_HZ * t60)) + rng.normal(0, 0.15, len(t60)),
          (0, 12), ref=10, ticks=(5, 2.5))
    chart("gyro", "rad/s", np.abs(0.12 * np.sin(2 * np.pi * GAIT_HZ * t60) + rng.normal(0, 0.01, len(t60))),
          (0, 0.4), ticks=(0.2, 0.1))
    chart("tick work", "ms", 11 + rng.normal(0, 0.25, len(t60)), (0, 22), ref=20, ticks=(10, 5))
    chart("beacon age", "ms", 100 + np.abs(rng.normal(0, 12, len(t60))), (0, 400), ref=250, ticks=(200, 100))
    chart("rec holes", "/s", np.where(np.abs(t60 + 34) < 0.4, 2.0, 0.0), (0, 5), color=SIMC, ticks=(5, 1))
    grid = QtWidgets.QGridLayout()
    grid.setVerticalSpacing(3)
    for i, (n, u, v, p) in enumerate(plots):
        grid.addWidget(lab(n, DIM, mono=True), i, 0)
        grid.addWidget(p, i, 1)
        grid.addWidget(lab(f"{v} {u}", "#e8e8e8", 13, True, True), i, 2)
    last = mkplot(26, (-60, 0), (0, 1), bottom=True, left_w=44)
    last.getPlotItem().hideAxis("left")
    grid.addWidget(last, len(plots), 1)
    grid.setColumnStretch(1, 1)
    refs = lab("dashed amber: 10° abort line · 20 ms tick budget · 250 ms link stale (STAND)", DIM, 11)
    left = group("ROBOT   last 60 s · robot time", vbox(grid, refs))

    rows, colors = [], {}
    for r, n in enumerate([f"{s}_{p}" for p in LEGS for s in ("L", "R")]):
        temp = 34 + (7 if n == "R_knee" else 0) + int(rng.integers(0, 4))
        load = float(LOAD[n][-50:].mean())
        cur = 120 + load * 9 + float(rng.normal(0, 10))
        status = "FAULT" if n == FAULT_JOINT else "ok"
        rows.append([n, BUS_ID[n], f"{temp} °C", f"{load:4.0f} %", f"{cur:4.0f} mA", "11.8 V", status])
        colors[(r, 6)] = RED if status == "FAULT" else GREEN
        if temp >= 40:
            colors[(r, 2)] = AMBER
    jt = table(["joint", "id", "temp", "load", "current", "volt", "status"], rows, colors, stretch_col=0)
    right = group("SERVOS   flight recorder feedback block, per fitted joint", vbox(jt))
    link = table(["link", "value"],
                 [["commands sent", "20.0 Hz · 24 B · seq 18342"],
                  ["beacon", "10.1 Hz · lag 1 · bad 0"],
                  ["flight recorder", "50.0 Hz · holes 2 (network) · drops 0 (robot)"],
                  ["robot clock", "NTP synced · Δ +3 ms vs this host"],
                  ["loop", "late 0 % · worst tick 13.4 ms · overruns 0"]],
                 stretch_col=1)
    obs = lab("Observer: no command traces — a flat zero would read as 'told to stand'. "
              "Room is reserved for the power board block (#109) and PWR_ESTOP (#106).", DIM, 11)
    obs.setWordWrap(True)
    return wrap(hbox(left, vbox(right, group("LINK", vbox(link)), obs, "stretch", spacing=10),
                     spacing=12, margins=(12, 12, 12, 12)))


def sim_tab():
    run = QtWidgets.QComboBox()
    run.addItems(["robot_legs_walk14_s1_s128r24"])
    plant = QtWidgets.QComboBox()
    plant.addItems(["bimo_biped_v6ar.xml"])
    form = QtWidgets.QFormLayout()
    form.setHorizontalSpacing(16)
    form.setVerticalSpacing(10)
    form.addRow(lab("policy run", DIM), run)
    form.addRow(lab("plant", DIM), plant)
    form.addRow(lab("options", DIM), hbox(QtWidgets.QCheckBox("hang: weld the torso, feet free"),
                                          colored_check("twin-in-the-loop", SIMC, True), "stretch",
                                          spacing=20))
    form.addRow(lab("ports", DIM), lab("cmd 4310 · tlm 4311 · stream 8645 · flight rec 4320", mono=True))
    ctl = hbox(btn("Stop", None, 110, h=36), btn("Reset", None, 110, h=36), "stretch",
               pill("RUNNING", GREEN, True, 110), spacing=10)
    twin = lab("twin-in-the-loop: every command frame sent to the robot is also sent to the sim on its own "
               "ports; the sim's flight records drive the amber ghost in the 3D view and a second trace "
               "in Joints. Joint space only: both roots are pinned.", DIM, 12)
    twin.setWordWrap(True)
    ident = lab("sil_twin: run=robot_legs_walk14_s1_s128r24 plant=bimo_biped_v6ar.xml\n"
                "SIL firmware stack · 1× real time", "#b8bec6", 12, mono=True)
    left = group("SIM TWIN   sim/sil_twin.py · the firmware's control code against the CPU plant",
                 vbox(form, ctl, ident, twin, spacing=12))
    log = QtWidgets.QPlainTextEdit()
    log.setReadOnly(True)
    log.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
    log.setPlainText("\n".join([
        "22:14:03.112  sil_twin: run=robot_legs_walk14_s1_s128r24 plant=bimo_biped_v6ar.xml",
        "22:14:03.540  listening: cmd 4310  tlm 4311",
        "22:14:28.904  stream: http://127.0.0.1:8645/  (render every 3rd tick)",
        "22:14:29.020  flight records -> 127.0.0.1:4320 (50 Hz)",
        "22:14:31.377  first command from 127.0.0.1:52144 -- watchdog follows it",
        "22:14:31.880  ARM edge -> LIVE",
        "22:15:02.411  reset: episode restarted at home",
        "",
        "(mock log lines)",
    ]))
    right = group("SIL_TWIN LOG   opens itself when the sim fails", vbox(log))
    stream = group("SIM RENDER   the sim's own MJPEG world view (View pane → Sim render)",
                   vbox(lab("http://127.0.0.1:8645/ · 9.8 fps · last frame 0.1 s", "#b8bec6", 12, mono=True),
                        lab("STALE after 2 s without a frame · retries forever (sil_twin takes ~25 s to "
                            "listen)", DIM, 12)))
    return wrap(hbox(vbox(left, stream, "stretch", spacing=10), right, spacing=12,
                     margins=(12, 12, 12, 12)))


def session_tab():
    rec = group("RECORDING   on by default", vbox(
        hbox(btn("stop recording", "r", 170, h=34), 14, lab("● REC", "#ff5a50", 14, True), 8,
             lab("hw_sessions/2026-11-01/2026-11-01T22-14-03Z-bimo.jsonl", "#b8bec6", 12, mono=True),
             "stretch", lab("3218 cmd · 1609 tlm · 02:41", DIM), spacing=6),
        lab("the console's own JSONL (today's bimo_gui schema): every command sent, every beacon "
            "received, both clocks", DIM, 12)))
    fr = group("FLIGHT RECORDER   the robot's file lives on mira", vbox(
        hbox(lab("collector", DIM), lab("mira:4220 → forwarding to this console :4221", "#b8bec6", 12, mono=True),
             "stretch", lab("session file", DIM),
             lab("flight_2026-11-01T22-13-50Z_policy_stand1.frec", "#b8bec6", 12, mono=True), spacing=10),
        lab("holes 2 (network) · robot drops 0 · rate 50.0 Hz ctrl / 10.0 Hz idle · "
            "the console decodes with tools/flightrec.py", DIM, 12)))
    ev = [["22:13:50.204", "robot", "cmd", "home"],
          ["22:13:54.881", "robot", "out", "home readback: all 12 joints at their zeros -- torque released"],
          ["22:14:01.003", "console", "control", "TAKE CONTROL: robot benched, press [a] to arm"],
          ["22:14:03.517", "console", "note", "arm sent: waiting for the robot to leave BENCH"],
          ["22:14:03.560", "robot", "arm edge", "level 1"],
          ["22:14:03.562", "robot", "mode", "run · ARMED"],
          ["22:14:04.571", "robot", "link", "STAND → LIVE"],
          ["22:14:31.090", "console", "pad", "SN30 Pro: R held -- gamepad drives"],
          ["22:15:12.430", "robot", "fault", "servo id 11: no reply (reading held)"],
          ["22:15:12.431", "console", "alarm", "servo fault id 11 -- abort condition (§5)"]]
    colors = {}
    for r, row in enumerate(ev):
        colors[(r, 1)] = CYAN if row[1] == "robot" else DIM
        if row[2] in ("fault", "alarm"):
            colors[(r, 2)] = colors[(r, 3)] = RED
    filt = hbox(lab("show", DIM), *[colored_check(t, TEXT, True) for t in
                                    ("robot events", "robot CLI output", "console notes", "link", "alarms")],
                "stretch", QtWidgets.QLineEdit("filter…"), spacing=14)
    log = group("EVENT LOG   console and robot on one UTC axis",
                vbox(filt, table(["UTC", "from", "kind", "message"], ev, colors, stretch_col=3)))
    return wrap(vbox(rec, fr, log, spacing=10, margins=(12, 12, 12, 12)))


def setup_tab():
    def form(rows):
        f = QtWidgets.QFormLayout()
        f.setHorizontalSpacing(16)
        f.setVerticalSpacing(8)
        for k, v in rows:
            f.addRow(lab(k, DIM), v)
        return f

    def edit(t, w=None):
        e = QtWidgets.QLineEdit(t)
        if w:
            e.setFixedWidth(w)
        return e

    link = group("LINK   edits apply on the button", vbox(form([
        ("robot host", edit("bimo.local", 240)),
        ("ports", hbox(edit("4210", 80), edit("4211", 80), lab("cmd / tlm", DIM), "stretch")),
        ("start as", hbox(segmented(["observer", "driver"]), lab("observer sends nothing until TAKE CONTROL",
                                                                 DIM), "stretch", spacing=10)),
        ("relay (--watch)", hbox(QtWidgets.QCheckBox("forward every beacon to"), edit("", 180), "stretch")),
    ]), hbox(btn("apply && reconnect", None, 190, h=32), lab("a relink while armed disarms first", DIM),
             "stretch", spacing=10)))
    data = group("DATA SOURCES", vbox(form([
        ("flight recorder", hbox(segmented(["via mira collector", "direct"]), edit("mira:4220 → :4221", 200),
                                 "stretch", spacing=10)),
        ("3D plant", hbox(QtWidgets.QComboBox(), "stretch")),
        ("sim stream", edit("http://127.0.0.1:8645/", 300)),
        ("bench cams", edit("http://mira:8646/", 300)),
    ])))
    data.findChildren(QtWidgets.QComboBox)[0].addItems(["sim/bimo_biped_v6ar.xml"])
    keys = table(["action", "key"], [["forward", "Up"], ["back", "Down"], ["strafe left", "Left"],
                                     ["strafe right", "Right"], ["turn left", "PageUp"],
                                     ["turn right", "PageDown"], ["arm / disarm", "a"], ["stand", "Space"],
                                     ["E-STOP", "e"], ["reset servos", "h"], ["record", "r"],
                                     ["extended → defaults", "0"], ["take control / watch", "none"],
                                     ["quit (disarms)", "q"]], stretch_col=0)
    keyg = group("KEYMAP   ~/.bimo/keymap.conf · click a binding, press a key",
                 vbox(keys, hbox(btn("restore defaults", None, 160, h=30), btn("save", None, 90, h=30),
                                 "stretch"), "stretch"))
    pad = table(["control", "SDL name (by position)", "SN30 Pro label", "live"],
                [["dead-man (hold)", "rightshoulder", "R", "● held"],
                 ["E-STOP", "b (east)", "A", "○"],
                 ["vx", "lefty", "left stick ↕", "−0.62"],
                 ["vy", "leftx", "left stick ↔", "+0.03"],
                 ["wz", "rightx", "right stick ↔", "+0.35"],
                 ["arm / reset servos", "—", "not on the pad", ""]],
                {(0, 3): GREEN, (5, 2): FAINT}, stretch_col=0)
    padg = group("GAMEPAD   8BitDo SN30 Pro · Bluetooth · connected", vbox(
        pad, form([("silent-pad timeout", hbox(edit("150", 70), lab("ms · pads that stream reports only", DIM), "stretch")),
                   ("stick dead zone", hbox(edit("0.15", 70), "stretch"))])))
    return wrap(hbox(vbox(link, data, "stretch", spacing=10), vbox(keyg, spacing=10),
                     vbox(padg, "stretch", spacing=10), spacing=12, margins=(12, 12, 12, 12)))


TABS = ["Checks", "Drive", "Joints", "Telemetry", "Sim", "Session", "Setup"]
BUILDERS = {"Checks": checks_tab, "Drive": drive_tab, "Joints": joints_tab,
            "Telemetry": telemetry_tab, "Sim": sim_tab, "Session": session_tab, "Setup": setup_tab}


def window(tab, view=True, state=LIVE_STATE, footnote=""):
    win = QtWidgets.QMainWindow()
    root = QtWidgets.QWidget()
    root.setObjectName("root")
    tabs = QtWidgets.QTabWidget()
    for t in TABS:
        tabs.addTab(BUILDERS[t]() if t == tab else QtWidgets.QWidget(), t)
    tabs.setCurrentIndex(TABS.index(tab))
    split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
    split.setHandleWidth(6)
    vw = 800
    if view:
        split.addWidget(view_pane(vw, 800))
    else:
        strip = lab("3D\n▸", DIM, 12, True)
        strip.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        strip.setFixedWidth(26)
        strip.setStyleSheet(f"background:#111317; color:{DIM}; font-weight:700;")
        split.addWidget(strip)
    split.addWidget(wrap(vbox(tabs, margins=(4, 8, 12, 4))))
    split.setSizes([vw + 24, W - vw - 24] if view else [26, W - 26])
    rl = vbox(safety_bar(state), split, spacing=0)
    rl.setStretchFactor(split, 1)
    root.setLayout(rl)
    win.setCentralWidget(root)
    sb = QtWidgets.QStatusBar()
    sb.showMessage(MOCK + (f" · {footnote}" if footnote else ""))
    win.setStatusBar(sb)
    win.resize(W, H)
    return win


def pose3d_sheet():
    pw, ph = 760, 600
    panels = [("3D pose · measured only", dict(cmd=False, sim=False, camera=None)),
              ("+ Commanded: the goal written this tick (cyan ghost)", dict(cmd=True, sim=False, camera=None)),
              ("+ Sim: the twin's own pose, same commands (amber ghost)", dict(cmd=False, sim=True, camera=None)),
              ("Camera PiP: beside the model, never composited under it", dict(cmd=True, sim=False, camera="pip"))]
    gap, head = 18, 34
    sheet = QtGui.QPixmap(2 * pw + 3 * gap, 2 * (ph + head) + 3 * gap + 30)
    sheet.fill(QtGui.QColor(PANEL))
    p = QtGui.QPainter(sheet)
    p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    for i, (title, kw) in enumerate(panels):
        x = gap + (i % 2) * (pw + gap)
        y = gap + (i // 2) * (ph + head + gap)
        p.setPen(QtGui.QColor("#e8e8e8"))
        p.setFont(QtGui.QFont("Helvetica", 15, QtGui.QFont.Weight.Bold))
        p.drawText(QtCore.QPointF(x + 2, y + 22), title)
        p.drawPixmap(x, y + head, view_image(pw, ph, kw["cmd"], kw["sim"], kw["camera"], hint=False))
    p.setPen(QtGui.QColor(AMBER))
    p.setFont(QtGui.QFont("Helvetica", 12))
    p.drawText(QtCore.QPointF(gap, sheet.height() - 14),
               "MOCKUP · real MuJoCo renders of sim/bimo_biped_v6ar.xml posed from synthetic angles · "
               "ghosts are separate geometry, never a pixel blend · arms and neck not fitted: zero pose, dimmed")
    p.end()
    return sheet


BAR_STATES = [
    ("BENCH — arm refused: the policy is a placeholder net",
     dict(LIVE_STATE, badge=("●  BENCH", BENCHC), armed=False, pad=("PAD  ○ R up", DIM, "connected"),
          bench="arm REFUSED -- this firmware's compiled-in net is a placeholder/untrained test net, not a "
                "policy (flash a build with real weights)",
          note="robot stayed BENCH after arm: see the BENCH line")),
    ("LIVE — driving", LIVE_STATE),
    ("E-STOP latched — STAND clears it", dict(LIVE_STATE, badge=("●  ESTOP", RED), estop=True,
                                              pad=("PAD  ○ R up", DIM, "A (E-STOP) pressed 2 s ago"),
                                              note="E-STOP sent (latched until [space])")),
    ("OBSERVER — watching a tethered run through the flight recorder; HOLD and E-STOP still work",
     dict(LIVE_STATE, badge=("●  STAND", AMBER), driver=False, pad=("PAD  stops only", DIM, "A (east) = E-STOP"),
          note="observing a tethered run: sends no commands; HOLD / E-STOP are operator-stop frames "
               "(tools/estop.py), which no latch or watchdog sees")),
    ("LINK LOST — no beacon for 3.4 s", dict(LIVE_STATE, badge=("(no beacon)", "#3a3f47"), lost="3.4",
                                            note="link lost: the robot stands after 250 ms and goes limp after 5 s")),
    ("FLAIL — the robot's flail guard stopped the policy and HELD the joints",
     dict(LIVE_STATE, badge=("●  BENCH", BENCHC), armed=False, pad=("PAD  ○ R up", DIM, "connected"),
          bench="disarmed -- FLAIL guard: the policy misbehaved, so it was stopped and the joints HELD "
                "stiff; it homes once upright and still (`flail` on the tether says why)",
          note="flight rec event: flail (rate) at 3.31 s")),
    ("FALLEN — tilt past 45° for 200 ms: torque off, the loop benched",
     dict(LIVE_STATE, badge=("●  FALLEN", RED), armed=False, servos=("servos ok", GREEN, "loop late 0 %"),
          bench="disarmed -- FALL latch tripped; re-arm deliberately (ARM edge or run)",
          note="robot reports FALLEN")),
]


def bar_sheet():
    w = QtWidgets.QWidget()
    w.setObjectName("root")
    items = []
    for title, st in BAR_STATES:
        items.append(lab(title, "#e8e8e8", 15, True))
        items.append(safety_bar(st))
    items.append(lab(MOCK, AMBER, 12))
    w.setLayout(vbox(*items, spacing=8, margins=(16, 14, 16, 14)))
    w.resize(W, 10)
    return w


def grab(widget, path, size=None):
    widget.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen)
    if size:
        widget.resize(*size)
    widget.show()
    for _ in range(5):
        QtWidgets.QApplication.processEvents()
    if size is None:
        widget.resize(W, widget.sizeHint().height())
        for _ in range(3):
            QtWidgets.QApplication.processEvents()
    widget.grab().save(str(path))
    print("wrote", path.relative_to(REPO))
    widget.close()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    shots = [("drive", "Drive", True, ""),
             ("joints", "Joints", False, "3D view collapsed (splitter) to give Joints the full width"),
             ("checks", "Checks", True, ""),
             ("telemetry", "Telemetry", False, "3D view collapsed"),
             ("sim", "Sim", True, ""),
             ("session", "Session", True, ""),
             ("setup", "Setup", False, "3D view collapsed")]
    for name, tab, view, foot in shots:
        grab(window(tab, view, footnote=foot), HERE / f"console_ui_{name}.png", (W, H))
    path = HERE / "console_ui_pose3d.png"
    pose3d_sheet().save(str(path))
    print("wrote", path.relative_to(REPO))
    grab(bar_sheet(), HERE / "console_ui_bar_states.png")


if __name__ == "__main__":
    main()
