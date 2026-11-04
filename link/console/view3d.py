"""The view pane (design §5): the robot in 3D from what it measured, with the
commanded and sim poses as ghosts, the bench cameras and the sim's own render.

  * Measured, solid: the robot's latest flight-recorder tick
    (app.flight.latest("robot")), else the beacon's joint block (--mirror)
    and attitude (ATT). Unfitted or unread joints sit at the plant's zero
    pose, dimmed. Kinematics only (pose3d.py), on the GUI thread.
  * Commanded, cyan ghost: each joint's goal in the same tick (or its
    nominal, `commanded_field`); a joint with no goal draws no ghost.
  * Sim, amber ghost: the sim twin's pose (app.flight.latest("sim")).
  * Every layer carries its age; one older than FROZEN_S reads FROZEN.
  * Cameras (tools/cam_relay.py) and the sim render (sil_twin's MJPEG) are
    shown beside the model -- picture-in-picture, split, or alone -- and
    never composited under it: no camera here is registered to the plant.

The 3D picture is rendered offscreen into a QImage only when what it shows
changed (a new sample, a camera move, a toggle, a resize), at most once per
screen refresh.
"""
from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

from PyQt6 import QtCore, QtGui, QtWidgets

from . import core, pose3d
from .mjpeg import MjpegStream
from .options import CAM_NAMES, CAM_PORT, STREAM_PORT
from .ui import AMBER, BRIGHT, CYAN, DIM, FAINT, GREEN, MONO, RED, SIMC, TEXT, hbox, lab, vbox

FROZEN_S = 1.0
SIM_KEY = "sim render"
SEL_COLOUR = "#db5cf2"            # pose3d.SEL_RGBA
DIM_COLOUR = "#73777d"
SKY = "#000000"

CAM0 = dict(az=118.0, el=-9.0, dist=1.08, ox=0.0, oy=0.0, oz=0.0)
FREE = "view: free (mouse)"
# (label, azimuth, elevation, distance). +X is the robot's front, +Y its left.
# The bench cameras are not calibrated, so their two angles only resemble them.
PRESETS = (("view: side cam ≈ (approx.)", 118.0, -9.0, 1.08),
           ("view: rear cam ≈ (approx.)", 20.0, -22.0, 1.15),
           ("view: front", 180.0, -6.0, 1.08),
           ("view: side", 90.0, -6.0, 1.08),
           ("view: rear", 0.0, -6.0, 1.08),
           ("view: top", 180.0, -89.0, 1.1))
SOURCES = (("3d", "3D pose"), ("camera", "Camera"), ("sim", "Sim render"))


def age_text(age):
    """(text, colour) for a layer's age; FROZEN past FROZEN_S."""
    if age is None:
        return "", DIM
    if age > FROZEN_S:
        return (f"FROZEN {age:.1f} s" if age < 100 else f"FROZEN {age:.0f} s"), RED
    return f"{age:.2f} s", DIM


def split_hostport(text: str, port: int):
    host, sep, p = text.rpartition(":")
    if sep and p.isdigit() and host:
        return host, int(p)
    return text, port


def _bucket(px: float) -> int:
    return int(min(pose3d.MAX_PX, max(64, math.ceil(px / 32.0) * 32)))


class _Canvas(QtWidgets.QWidget):
    """The picture and the camera's input surface: left-drag orbits,
    right/middle-drag pans, the wheel zooms, a double-click resets
    (sim/joint_puppet.py's conventions)."""

    def __init__(self, pane):
        super().__init__()
        self.pane = pane
        self._last = None
        self._btn = None
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        self.pane.paint(p, self.width(), self.height())
        p.end()

    def resizeEvent(self, ev):
        self.pane.request_render()
        super().resizeEvent(ev)

    def showEvent(self, ev):
        self.pane.request_render()
        super().showEvent(ev)

    def mousePressEvent(self, e):
        if self.pane.in_3d(e.position()):
            self._last = e.position()
            self._btn = e.button()

    def mouseReleaseEvent(self, e):
        self._last = None

    def mouseMoveEvent(self, e):
        if self._last is None:
            return
        pos = e.position()
        dx, dy = pos.x() - self._last.x(), pos.y() - self._last.y()
        self._last = pos
        self.pane.drag(dx, dy, self._btn == QtCore.Qt.MouseButton.LeftButton)

    def wheelEvent(self, e):
        if self.pane.in_3d(e.position()):
            self.pane.zoom(e.angleDelta().y())

    def mouseDoubleClickEvent(self, e):
        if self.pane.in_3d(e.position()):
            self.pane.reset_camera()


def _check(text, colour, checked):
    c = QtWidgets.QCheckBox(text)
    c.setChecked(checked)
    c.setStyleSheet(f"QCheckBox {{ color: {colour}; font-weight: 600; }}")
    c.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    return c


def _segmented(labels, checked=0):
    grp = QtWidgets.QButtonGroup()
    grp.setExclusive(True)
    lay = QtWidgets.QHBoxLayout()
    lay.setSpacing(0)
    for i, t in enumerate(labels):
        b = QtWidgets.QToolButton()
        b.setText(t)
        b.setCheckable(True)
        b.setChecked(i == checked)
        b.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        grp.addButton(b, i)
        lay.addWidget(b)
    return grp, lay


class ViewPane(QtWidgets.QWidget):
    """ViewPane(app): app.o (the options), app.flight (None or a store with
    latest(source)), app.selected_joint (optional). update_from(st, now_ms)
    at the window's refresh; close_streams() on close."""

    def __init__(self, app):
        super().__init__()
        self.app = app
        o = getattr(app, "o", None) or SimpleNamespace()
        self.o = o
        self.plant = pose3d.resolve_plant(getattr(o, "plant", None), getattr(o, "repo", None))
        self.plant_error = ""
        try:
            self.model = pose3d.PoseModel(self.plant)
        except Exception as e:     # a bad --plant must not stop the console; the view says why
            self.model = None
            self.plant_error = f"{type(e).__name__}: {e}"
        self.commanded_field = "goal"       # or "nominal" (the Joints tab's toggle)
        self.cam = dict(CAM0)
        self.source = "3d"
        self.split = False
        self.streams: dict[str, MjpegStream] = {}
        self.info: dict = {}
        self.errors: list[str] = []
        self.meas = pose3d.Layer({}, measured=())
        self.ghosts: list[pose3d.Layer] = []
        self._beacon = None                 # (q, up, sample time ms, tlm_count)
        self._rate = {"robot": [None, 0.0], "sim": [None, 0.0]}
        self._renderer = None
        self._img = None
        self._img_key = None
        self._want = False
        self._last_render = 0.0
        self.renders = 0

        relay = getattr(o, "cam_relay", None)
        cams = [c for c in (getattr(o, "cam", None) or []) if c and c != "off"]
        # -- controls --------------------------------------------------------
        self.src_group, src_lay = _segmented([t for _, t in SOURCES])
        self.src_group.idClicked.connect(lambda i: self.set_source(SOURCES[i][0]))
        self.show_cmd = _check("Commanded", CYAN, True)
        self.show_sim = _check("Sim", SIMC, False)
        self.show_pip = _check("Camera PiP", TEXT, bool(cams))
        for c in (self.show_cmd, self.show_sim, self.show_pip):
            c.toggled.connect(self._toggled)
        self.preset = QtWidgets.QComboBox()
        self.preset.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.preset.addItems([FREE] + [p[0] for p in PRESETS])
        self.preset.setCurrentIndex(1)
        self.preset.activated.connect(self._preset_chosen)
        self.cam_pick = QtWidgets.QComboBox()
        self.cam_pick.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        if relay:
            self.cam_pick.addItems(list(CAM_NAMES))
        self.cam_pick.addItem(SIM_KEY)
        if not relay:
            self.cam_pick.addItem("bench cams: no --cam-relay")
            self.cam_pick.model().item(self.cam_pick.count() - 1).setEnabled(False)
        first = cams[0] if cams and relay else (CAM_NAMES[0] if relay else SIM_KEY)
        self.cam_pick.setCurrentText(first)
        self.cam_pick.currentTextChanged.connect(lambda _t: self._toggled())
        self.lay_group, lay_lay = _segmented(["PiP", "split"])
        self.lay_group.idClicked.connect(lambda i: self.set_split(i == 1))
        row1 = hbox(src_lay, 18, self.show_cmd, self.show_sim, self.show_pip, "stretch",
                    spacing=12)
        row2 = hbox(lab("camera", DIM), self.cam_pick, lay_lay, "stretch", self.preset,
                    spacing=10)
        note = lab("camera: not registered to the 3D view, never drawn under the model",
                   FAINT, 11)
        note.setWordWrap(True)
        note.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                           QtWidgets.QSizePolicy.Policy.Preferred)
        self.canvas = _Canvas(self)
        self.status = lab("", rich=True)
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                  QtWidgets.QSizePolicy.Policy.Preferred)
        lay = vbox(row1, row2, note, self.canvas, self.status, spacing=8,
                   margins=(12, 10, 8, 8))
        lay.setStretchFactor(self.canvas, 1)
        self.setLayout(lay)

        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._render_due)
        self._rebuild_timer = QtCore.QTimer(self)
        self._rebuild_timer.setSingleShot(True)
        self._rebuild_timer.setInterval(200)
        self._rebuild_timer.timeout.connect(self._rebuild)
        self._gather(None, None)
        self._set_status()
        self._sync_streams()

    # -- public controls -----------------------------------------------------
    @property
    def cam_name(self) -> str:
        return self.cam_pick.currentText()

    def set_source(self, source: str) -> None:
        ids = [k for k, _ in SOURCES]
        self.source = source if source in ids else "3d"
        b = self.src_group.button(ids.index(self.source))
        if b is not None and not b.isChecked():
            b.setChecked(True)
        self._toggled()

    def set_split(self, on: bool) -> None:
        self.split = bool(on)
        b = self.lay_group.button(1 if self.split else 0)
        if b is not None and not b.isChecked():
            b.setChecked(True)
        self._toggled()

    def set_overlays(self, commanded=None, sim=None, pip=None) -> None:
        for box, v in ((self.show_cmd, commanded), (self.show_sim, sim), (self.show_pip, pip)):
            if v is not None:
                box.setChecked(bool(v))

    def set_camera(self, name: str) -> None:
        self.cam_pick.setCurrentText(name)

    def set_preset(self, name: str) -> None:
        for i, p in enumerate(PRESETS):
            if name.lower() in p[0].lower():
                self.preset.setCurrentIndex(i + 1)
                self._preset_chosen(i + 1)
                return
        raise KeyError(name)

    def reset_camera(self) -> None:
        self.cam = dict(CAM0)
        self.preset.setCurrentIndex(1)
        self.request_render()

    def drag(self, dx: float, dy: float, orbit: bool) -> None:
        c = self.cam
        if orbit:
            c["az"] = (c["az"] - dx * 0.4) % 360.0
            c["el"] = max(-89.0, min(89.0, c["el"] - dy * 0.4))
        else:                                   # pan in the camera plane
            az, el = math.radians(c["az"]), math.radians(c["el"])
            right = (-math.sin(az), math.cos(az), 0.0)
            up = (-math.cos(az) * math.sin(el), -math.sin(az) * math.sin(el), math.cos(el))
            s = c["dist"] * 0.0015
            for i, k in enumerate(("ox", "oy", "oz")):
                c[k] += (-right[i] * dx + up[i] * dy) * s
        self._camera_moved()

    def zoom(self, delta: float) -> None:
        self.cam["dist"] = max(0.25, min(6.0, self.cam["dist"] * math.exp(-delta * 0.0015)))
        self._camera_moved()

    def _camera_moved(self):
        self.preset.setCurrentIndex(0)
        self.request_render()

    def _preset_chosen(self, i: int) -> None:
        if i <= 0:
            return
        _, az, el, dist = PRESETS[i - 1]
        self.cam = dict(az=az, el=el, dist=dist, ox=0.0, oy=0.0, oz=0.0)
        self.request_render()

    def _toggled(self, *_):
        self._sync_streams()
        self._gather_ghosts()
        self.request_render()
        self._set_status()
        self.canvas.update()

    # -- data ----------------------------------------------------------------
    def _latest(self, source: str):
        fl = getattr(self.app, "flight", None)
        if fl is None:
            return None
        try:
            rec = fl.latest(source)
        except Exception as e:     # another module's bug must not take the window down
            self.errors.append(f"flight store latest({source!r}): {type(e).__name__}: {e}")
            return None
        return rec if isinstance(rec, dict) else None

    def _tick_rate(self, key: str, t) -> float:
        """Samples per second from distinct sample times (seconds)."""
        r = self._rate[key]
        if t is not None and r[0] is not None and t > r[0]:
            hz = 1.0 / (t - r[0])
            r[1] = hz if r[1] <= 0 else 0.8 * r[1] + 0.2 * hz
        if t is not None and (r[0] is None or t > r[0]):
            r[0] = t
        return r[1]

    def _gather(self, st, now_ms) -> None:
        """Read every source into self.meas / self.ghosts / self.info."""
        self.errors = []
        self._st = st
        names = set(self.model.qadr) if self.model is not None else set()
        rob = self._latest("robot")
        self._rob = rob
        m = dict(source=None, age=None, up=None, fitted=None, unfitted=0, noread=0,
                 nofb=0, n=0, rate=0.0, why="")
        if rob is not None:
            joints = rob.get("joints") or {}
            fitted = rob.get("fitted")
            fitted = list(fitted) if fitted is not None else list(joints)
            q, ok = {}, set()
            for n in fitted:
                j = joints.get(n) or {}
                pos = j.get("pos")
                if pos is None:
                    m["noread"] += 1
                    continue
                q[n] = float(pos)
                if j.get("fb_ok", True):
                    ok.add(n)
                else:
                    m["nofb"] += 1
            up = rob.get("up")
            m.update(source="flight recorder", age=_f(rob.get("age_s")), up=up,
                     fitted=len(fitted), n=len(q),
                     unfitted=len([n for n in names if n not in fitted]),
                     rate=self._tick_rate("robot", _f(rob.get("t"))))
            self.meas = pose3d.Layer(q, up=up, measured=ok)
        else:
            self._from_beacon(st, now_ms, m, names)
        self.info["measured"] = m
        self._gather_ghosts()

    def _from_beacon(self, st, now_ms, m, names) -> None:
        if (st is not None and now_ms is not None and getattr(st, "have_tlm", 0)
                and getattr(st, "n_joints", 0) == core.NUM_JOINTS):
            if self._beacon is None or self._beacon[3] != st.tlm_count:
                q = dict(zip(pose3d.JOINT_NAMES, (float(v) for v in st.joints)))
                up = (float(st.up_x), float(st.up_y), float(st.up_z)) if st.have_att else None
                t = now_ms - max(0.0, float(st.tlm_age_ms))
                self._tick_rate("robot", t / 1000.0)
                self._beacon = (q, up, t, st.tlm_count)
        if self._beacon is None:
            m.update(why=_no_pose_reason(st))
            self.meas = pose3d.Layer({}, measured=())
            return
        q, up, t, _ = self._beacon
        now = now_ms if now_ms is not None else t
        m.update(source="beacon", age=max(0.0, (now - t) / 1000.0), up=up, fitted=None,
                 n=len([n for n in q if n in names]), rate=self._rate["robot"][1])
        self.meas = pose3d.Layer(q, up=up)        # fitted set unknown: nothing dimmed

    def _gather_ghosts(self) -> None:
        rob = getattr(self, "_rob", None)
        ghosts = []
        c = dict(on=self.show_cmd.isChecked(), age=None, why="", n=0)
        if c["on"]:
            if rob is None:
                c["why"] = "no flight records"
            else:
                q = {n: _f(j.get(self.commanded_field))
                     for n, j in (rob.get("joints") or {}).items() if isinstance(j, dict)}
                q = {n: v for n, v in q.items() if v is not None}
                if q:
                    c.update(age=_f(rob.get("age_s")), n=len(q))
                    ghosts.append(pose3d.Layer(q, rgba=pose3d.CMD_RGBA))
                else:
                    c["why"] = f"no {self.commanded_field}"
        s = dict(on=self.show_sim.isChecked(), age=None, why="", n=0)
        if s["on"]:
            sim = self._latest("sim")
            if sim is None:
                s["why"] = "no sim records"
            else:
                q = {n: _f(j.get("pos")) for n, j in (sim.get("joints") or {}).items()
                     if isinstance(j, dict)}
                q = {n: v for n, v in q.items() if v is not None}
                self._tick_rate("sim", _f(sim.get("t")))
                if q:
                    s.update(age=_f(sim.get("age_s")), n=len(q))
                    ghosts.append(pose3d.Layer(q, rgba=pose3d.SIM_RGBA))
                else:
                    s["why"] = "no joints"
        self.info["commanded"], self.info["sim"] = c, s
        self.ghosts = ghosts

    def update_from(self, st, now_ms) -> None:
        self._gather(st, now_ms)
        if self._scene_key() != self._img_key:
            self.request_render()
        self._set_status()
        self.canvas.update()

    def frozen(self, layer: str) -> bool:
        age = self.info.get(layer, {}).get("age")
        return age is not None and age > FROZEN_S

    # -- rendering -----------------------------------------------------------
    def selected(self):
        sel = getattr(self.app, "selected_joint", None)
        if self.model is None or not isinstance(sel, str):
            return None
        return sel if sel in self.model.joint_names else None

    def _scene_key(self):
        def qk(lay):
            return tuple(sorted((n, round(v, 4)) for n, v in lay.q.items()))
        up = self.meas.up
        upk = None if up is None else tuple(round(float(x), 4) for x in up)
        return (qk(self.meas), upk, tuple(sorted(self.meas.measured)),
                tuple((qk(g), g.rgba) for g in self.ghosts), self.selected(),
                tuple(round(v, 4) for v in self.cam.values()), self._rects_key())

    def _rects_key(self):
        r3d, _ = self.rects(self.canvas.width(), self.canvas.height())
        if r3d is None:
            return None
        dpr = self.canvas.devicePixelRatioF()
        return (_bucket(r3d.width() * dpr), _bucket(r3d.height() * dpr))

    def request_render(self) -> None:
        self._want = True
        if self._timer.isActive():
            return
        hz = 60.0
        scr = self.screen() if self.isVisible() else None
        if scr is not None and scr.refreshRate() > 1:
            hz = scr.refreshRate()
        wait = (self._last_render + 1.0 / hz - time.monotonic()) * 1000.0
        self._timer.start(max(0, int(wait)))

    def _render_due(self):
        if self._want and self.canvas.isVisible():
            self.render_now()
            self.canvas.update()

    def _rebuild(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self.request_render()

    def render_now(self):
        """Render the 3D layers now; returns the QImage (None: no 3D shown)."""
        self._want = False
        if self.model is None:
            return None
        size = self._rects_key()
        if size is None:
            return None
        r = self._renderer
        if r is not None and (r.width, r.height) != size:
            if self._img is None:
                r.close()
                r = self._renderer = None
            elif not self._rebuild_timer.isActive():
                self._rebuild_timer.start()          # mid-resize: scale the old one
        if r is None:
            r = self._renderer = pose3d.Renderer(self.model, *size)
        cam = pose3d.camera(self.model.look_z, self.cam["az"], self.cam["el"], self.cam["dist"],
                            (self.cam["ox"], self.cam["oy"], self.cam["oz"]))
        frame = r.render(self.meas, self.ghosts, cam, self.selected())
        img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], 3 * frame.shape[1],
                           QtGui.QImage.Format.Format_RGB888).copy()
        img.setDevicePixelRatio(self.canvas.devicePixelRatioF())
        self._img = img
        self._img_key = self._scene_key()
        self._last_render = time.monotonic()
        self.renders += 1
        return img

    def rects(self, w: float, h: float):
        """(3D rect or None, camera rect or None) in canvas coordinates."""
        full = QtCore.QRectF(0, 0, w, h)
        if self.source in ("camera", "sim"):
            return None, full
        if not self.show_pip.isChecked():
            return full, None
        if self.split:
            if w >= h:
                half = (w - 6) / 2
                return QtCore.QRectF(0, 0, half, h), QtCore.QRectF(half + 6, 0, half, h)
            half = (h - 6) / 2
            return QtCore.QRectF(0, 0, w, half), QtCore.QRectF(0, half + 6, w, half)
        pw = max(200.0, min(340.0, w * 0.42))
        ph = pw * 9 / 16 + 26
        return full, QtCore.QRectF(w - pw - 14, h - ph - 14, pw, ph)

    def in_3d(self, pos) -> bool:
        r3d, rc = self.rects(self.canvas.width(), self.canvas.height())
        if r3d is None or self.model is None or not r3d.contains(pos):
            return False
        return rc is None or self.split or not rc.contains(pos)

    # -- painting ------------------------------------------------------------
    def paint(self, p: QtGui.QPainter, w: int, h: int) -> None:
        p.fillRect(QtCore.QRectF(0, 0, w, h), QtGui.QColor("#0b0d10"))
        r3d, rc = self.rects(w, h)
        if r3d is not None:
            self._paint_3d(p, r3d)
        if rc is not None:
            key = SIM_KEY if self.source == "sim" else self.cam_name
            self._paint_stream(p, rc, key)

    def _paint_3d(self, p, r):
        p.save()
        p.setClipRect(r)
        p.fillRect(r, QtGui.QColor(SKY))
        img = self._img
        if self.model is None:
            p.setPen(QtGui.QColor(RED))
            p.setFont(QtGui.QFont(MONO, 11))
            p.drawText(r.adjusted(16, 16, -16, -16), int(QtCore.Qt.TextFlag.TextWordWrap),
                       f"plant failed to load: {self.plant}\n{self.plant_error}")
        elif img is not None:
            dpr = img.devicePixelRatio() or 1.0
            iw, ih = img.width() / dpr, img.height() / dpr
            s = max(r.width() / iw, r.height() / ih)
            s = 1.0 if s <= 1.0 else s               # larger: crop; smaller (mid-resize): cover
            tw, th = iw * s, ih * s
            p.drawImage(QtCore.QRectF(r.center().x() - tw / 2, r.center().y() - th / 2, tw, th),
                        img)
        if self.model is not None:
            self._paint_legend(p, r.x() + 14, r.y() + 14)
            if r.width() > 560:
                p.setFont(QtGui.QFont(MONO, 10))
                p.setPen(QtGui.QColor(DIM))
                p.drawText(QtCore.QRectF(r.right() - 330, r.y() + 12, 316, 40),
                           int(QtCore.Qt.AlignmentFlag.AlignRight),
                           "drag: orbit · right-drag: pan\nwheel: zoom · dbl-click: reset")
        p.restore()

    def legend_rows(self):
        """[(swatch colour, text, age text, age colour)], top to bottom."""
        m = self.info.get("measured", {})
        rows = []
        if m.get("source") is None:
            rows.append((FAINT, "measured   none", "zero pose", DIM))
        else:
            rows.append((BRIGHT, "measured   solid", *age_text(m.get("age"))))
        for key, colour, label in (("commanded", CYAN, "commanded"), ("sim", SIMC, "sim")):
            L = self.info.get(key, {})
            if not L.get("on"):
                rows.append((FAINT, f"{label:<10} off", "", DIM))
            elif L.get("age") is None:
                rows.append((FAINT, f"{label:<10} {L.get('why', '')}", "", DIM))
            else:
                what = "ghost" if key == "sim" or self.commanded_field == "goal" else "nominal"
                rows.append((colour, f"{label:<10} {what}", *age_text(L.get("age"))))
        if m.get("source") == "beacon":
            rows.append((DIM_COLOUR, "fitted     unknown", "beacon", DIM))
        elif m.get("unfitted") or m.get("source") is None:
            rows.append((DIM_COLOUR, "not fitted zero pose", "dim", DIM))
        if m.get("noread"):
            rows.append((DIM_COLOUR, f"no reading {m['noread']} joint{'s' * (m['noread'] > 1)}",
                         "dim", DIM))
        if m.get("nofb"):
            rows.append((DIM_COLOUR, f"no feedbk  {m['nofb']} joint{'s' * (m['nofb'] > 1)}",
                         "dim", DIM))
        if m.get("source") is not None and m.get("up") is None:
            rows.append((AMBER, "attitude not sent", "upright", AMBER))
        sel = self.selected()
        if sel:
            rows.append((SEL_COLOUR, f"selected   {sel}", "", DIM))
        return rows

    def _paint_legend(self, p, x, y):
        rows = self.legend_rows()
        font = QtGui.QFont(MONO, 11)
        fm = QtGui.QFontMetrics(font)
        tw = max(fm.horizontalAdvance(t) for _, t, _, _ in rows)
        aw = max([fm.horizontalAdvance(a) for _, _, a, _ in rows] + [40])
        w = 34 + tw + 18 + aw + 12
        p.setBrush(QtGui.QColor(10, 12, 15, 205))
        p.setPen(QtGui.QPen(QtGui.QColor("#3a3f47")))
        p.drawRoundedRect(QtCore.QRectF(x, y, w, 22 * len(rows) + 14), 8, 8)
        p.setFont(font)
        for i, (col, txt, age, acol) in enumerate(rows):
            yy = y + 22 + i * 22
            p.fillRect(QtCore.QRectF(x + 12, yy - 10, 12, 12), QtGui.QColor(col))
            p.setPen(QtGui.QColor(TEXT if col != FAINT else DIM))
            p.drawText(QtCore.QPointF(x + 34, yy), txt)
            p.setPen(QtGui.QColor(acol))
            p.drawText(QtCore.QPointF(x + 34 + tw + 18, yy), age)

    def stream_state(self, key: str):
        """(state text, colour, stream or None) for one camera or the sim render."""
        s = self.streams.get(key)
        if s is None:
            if key != SIM_KEY and not getattr(self.o, "cam_relay", None):
                return "no --cam-relay", FAINT, None
            return "○ off", FAINT, None
        st = s.state()
        if st == "live":
            return "● live", GREEN, s
        if st == "STALE":
            age = s.age_s() or 0.0
            return f"STALE {age:.1f} s", RED, s
        if st == "waiting":
            return "○ waiting", AMBER, s
        return "○ off", FAINT, s

    def _url(self, key: str):
        o = self.o
        if key == SIM_KEY:
            view = getattr(o, "sim_view", None)
            return view or f"http://127.0.0.1:{getattr(o, 'stream_port', None) or STREAM_PORT}/"
        relay = getattr(o, "cam_relay", None)
        if not relay or key not in CAM_NAMES:
            return None
        host, port = split_hostport(relay, CAM_PORT)
        return f"http://{host}:{port}/{key}"

    def _paint_stream(self, p, r, key):
        p.save()
        p.setClipRect(r.adjusted(-2, -2, 2, 2))
        p.setBrush(QtGui.QColor("#0b0d10"))
        p.setPen(QtGui.QPen(QtGui.QColor(FAINT), 1.5))
        p.drawRoundedRect(r, 6, 6)
        p.fillRect(QtCore.QRectF(r.x() + 1, r.y() + 1, r.width() - 2, 24), QtGui.QColor("#1a1d22"))
        state, colour, s = self.stream_state(key)
        url = self._url(key) or ""
        where = url.split("//", 1)[-1].split("/", 1)[0] if url else ""
        font = QtGui.QFont(MONO, 10)
        fm = QtGui.QFontMetrics(font)
        p.setFont(font)
        sw = fm.horizontalAdvance(state) + 4
        room = int(r.width() - sw - 28)
        title = f"{'bench cam · ' if key != SIM_KEY else ''}{key}   {where}"
        if fm.horizontalAdvance(title) > room:
            title = fm.elidedText(f"{key}   {where}", QtCore.Qt.TextElideMode.ElideRight, room)
        p.setPen(QtGui.QColor(TEXT))
        p.drawText(QtCore.QRectF(r.x() + 10, r.y() + 4, room, 18),
                   int(QtCore.Qt.AlignmentFlag.AlignLeft), title)
        p.setPen(QtGui.QColor(colour))
        p.drawText(QtCore.QRectF(r.right() - sw - 10, r.y() + 4, sw, 18),
                   int(QtCore.Qt.AlignmentFlag.AlignRight), state)
        body = QtCore.QRectF(r.x() + 2, r.y() + 26, r.width() - 4, r.height() - 28)
        img = s.latest()[0] if s is not None else None
        if img is not None and not img.isNull():
            sc = min(body.width() / img.width(), body.height() / img.height())
            tw, th = img.width() * sc, img.height() * sc
            p.drawImage(QtCore.QRectF(body.center().x() - tw / 2, body.center().y() - th / 2,
                                      tw, th), img)
            if state.startswith("STALE"):
                band = QtCore.QRectF(body.x(), body.center().y() - 18, body.width(), 36)
                p.fillRect(band, QtGui.QColor(10, 12, 15, 200))
                p.setPen(QtGui.QColor(RED))
                p.setFont(QtGui.QFont(MONO, 13, QtGui.QFont.Weight.Bold))
                p.drawText(band, int(QtCore.Qt.AlignmentFlag.AlignCenter),
                           f"{state}: the last frame, not live")
        else:
            p.setPen(QtGui.QColor(DIM))
            p.setFont(QtGui.QFont(MONO, 11))
            if s is None:
                msg = ("no camera: start the console with --cam-relay HOST[:PORT]"
                       if url == "" else f"{key}: not open")
            else:
                msg = f"waiting for {url}"
                if s.error:
                    msg += f"\n\n{s.error}"
            p.drawText(body.adjusted(10, 10, -10, -10),
                       int(QtCore.Qt.AlignmentFlag.AlignCenter | QtCore.Qt.TextFlag.TextWordWrap),
                       msg)
        if key != SIM_KEY:
            p.setFont(QtGui.QFont(MONO, 9))
            p.setPen(QtGui.QColor(FAINT))
            p.drawText(QtCore.QPointF(body.x() + 8, body.bottom() - 6), "not registered to 3D")
        p.restore()

    # -- status line -----------------------------------------------------------
    def _set_status(self):
        sep = f"<span style='color:{DIM}'> · </span>"
        d = f"<span style='color:{DIM}'>"
        if self.source in ("camera", "sim"):
            key = SIM_KEY if self.source == "sim" else self.cam_name
            state, colour, _ = self.stream_state(key)
            url = self._url(key) or "no --cam-relay"
            extra = ("sil_twin's own world view (floor, contacts, position)" if key == SIM_KEY
                     else "a picture, not registered to the 3D view")
            html = f"<span style='color:{colour}'>{state}</span>&nbsp;&nbsp;{d}{key} · {url} · {extra}</span>"
        else:
            m = self.info.get("measured", {})
            tail = "root pinned, yaw drawn 0 · kinematics only, no physics"
            plant = self.plant.rsplit("/", 1)[-1]
            if self.model is None:
                html = f"<span style='color:{RED}'>no 3D view</span>&nbsp;&nbsp;{d}plant {self.plant} failed to load</span>"
            elif m.get("source") is None:
                html = (f"<span style='color:{AMBER}'>○ no pose</span>&nbsp;&nbsp;{d}{m.get('why', '')}"
                        f"{sep}plant {plant} at its zero pose{sep}{tail}</span>")
            else:
                age = m.get("age") or 0.0
                rate = m.get("rate") or 0.0
                what = (f"{m['fitted']} fitted joints" if m.get("fitted") is not None
                        else f"{m.get('n', 0)} joints (fitted set unknown)")
                att = " + IMU" if m.get("up") is not None else " · attitude not sent"
                hz = f" {rate:.1f} Hz" if rate > 0 else ""
                head = (f"<span style='color:{RED}'>■ pose FROZEN {age:.1f} s</span>"
                        if age > FROZEN_S else f"<span style='color:{GREEN}'>● pose live</span>")
                html = (f"{head}&nbsp;&nbsp;{d}{m['source']}{hz}{sep}{what}{att}{sep}"
                        f"{age:.2f} s old{sep}{plant}{sep}{tail}</span>")
        if self.errors:
            html += f"<br><span style='color:{AMBER}'>{'; '.join(self.errors)}</span>"
        if html != self.status.text():
            self.status.setText(html)

    # -- streams -----------------------------------------------------------------
    def shown_streams(self) -> set:
        if self.source == "sim":
            return {SIM_KEY}
        if self.source == "camera" or self.show_pip.isChecked():
            return {self.cam_name} if self._url(self.cam_name) else set()
        return set()

    def _sync_streams(self) -> None:
        """Run only the streams on screen: the relay starts a camera for its
        first viewer and the Sony serves one, so a hidden pane holds none."""
        want = self.shown_streams()
        for key in list(self.streams):
            if key not in want:
                _stop_async([self.streams.pop(key)])
        for key in want:
            if key not in self.streams:
                url = self._url(key)
                if url:
                    s = self.streams[key] = MjpegStream(url, key)
                    s.start()

    def close_streams(self) -> None:
        """Stop every stream (and free the offscreen renderer)."""
        threads = _stop_async(list(self.streams.values()))
        self.streams.clear()
        end = time.monotonic() + 0.5
        for t in threads:
            t.join(timeout=max(0.0, end - time.monotonic()))
        self._timer.stop()
        self._rebuild_timer.stop()
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def _stop_async(streams):
    """MjpegStream.stop() joins its reader; a reader still connecting to a
    dead host would hold the GUI thread (and with it the link tick)."""
    out = []
    for s in streams:
        t = threading.Thread(target=s.stop, daemon=True, name=f"stop {s.name}")
        t.start()
        out.append(t)
    return out


def _f(v):
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _no_pose_reason(st) -> str:
    if st is None or not getattr(st, "have_tlm", 0):
        return "no flight records and no beacon yet"
    if st.state == core.BENCH:
        return "a BENCHED robot measures nothing; the pose appears on arm"
    if st.readonly:
        return ("an observer gets joints from the flight recorder (mira's collector "
                "--forward), or from a driver running --mirror")
    if not st.want_pose:
        return "the beacon carries joints only when asked: --mirror"
    return "the last beacon carried no joints"
