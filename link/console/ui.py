"""Look and small builders for the Qt console: the palette and style sheet of
the design mockups (docs/design-v6/figs/console_ui_mockups.py)."""
from __future__ import annotations

import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets

BG, PANEL, LINE = "#16181c", "#1e2126", "#343941"
TEXT, DIM, FAINT = "#d6d9de", "#8a9099", "#5a6068"
GREEN, AMBER, RED, ORANGE, BENCHC = "#2fa35a", "#c98a1b", "#c62828", "#d9731f", "#7f8ca6"
CYAN, SIMC, BLUE = "#3cc8ff", "#ffa41f", "#2f6fca"
BRIGHT = "#e8e8e8"
# Monospace: Menlo (macOS), then the usual Linux / Windows faces, then the
# platform's own monospace (a family list in style sheets, the Monospace
# style hint in a QFont: mono_font).
MONO_FAMILIES = ("Menlo", "DejaVu Sans Mono", "Liberation Mono", "Consolas", "monospace")
MONO = ", ".join(f"'{f}'" if " " in f else f for f in MONO_FAMILIES)

# Badge colour per linkproto::LinkState (wire order), bimo_gui's stateColour.
STATE_COLOUR = (GREEN, AMBER, ORANGE, RED, RED, RED, RED, BENCHC)

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
QPushButton:pressed {{ background: {BLUE}; color: white; }}
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
QPushButton#take:disabled {{ background: #2a2418; color: #6b5a3a; border-color: #3a3222; }}
QTabWidget::pane {{ border: 1px solid {LINE}; border-radius: 7px; top: -1px;
                   background: {PANEL}; }}
QTabBar::tab {{ background: #23272d; padding: 8px 20px; border: 1px solid {LINE};
               border-bottom: none; border-top-left-radius: 7px;
               border-top-right-radius: 7px; margin-right: 3px; color: #9aa3ad; }}
QTabBar::tab:selected {{ background: {PANEL}; color: white; font-weight: 700; }}
QToolButton {{ background: #2a2e35; border: 1px solid #3a3f47; padding: 5px 13px; }}
QToolButton:checked {{ background: {BLUE}; color: white; border-color: #5a95e8;
                      font-weight: 700; }}
QStatusBar {{ background: #111317; color: {DIM}; border-top: 1px solid #2c3036; }}
QSlider::groove:horizontal {{ height: 5px; background: #343941; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {BLUE}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: #d6d9de; width: 14px; margin: -6px 0;
                             border-radius: 7px; }}
QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{ background: #2a2e35; border: 1px solid #3a3f47;
            border-radius: 6px; padding: 4px 10px; }}
QSplitter::handle {{ background: #2c3036; }}
QTableWidget {{ background: {BG}; gridline-color: #2c3036; border: 1px solid {LINE};
               border-radius: 6px; }}
QHeaderView::section {{ background: #23272d; color: #9aa3ad; border: none;
                       border-bottom: 1px solid {LINE}; padding: 5px 8px; font-weight: 700; }}
QPlainTextEdit {{ background: #0f1114; border: 1px solid {LINE}; border-radius: 6px;
                 font-family: {MONO}; font-size: 12px; color: #b8bec6; }}
"""

pg.setConfigOptions(antialias=True, background=BG, foreground=TEXT)


def dark_palette() -> QtGui.QPalette:
    p = QtGui.QPalette()
    R = QtGui.QPalette.ColorRole
    for role, col in [(R.Window, PANEL), (R.WindowText, TEXT), (R.Base, BG),
                      (R.AlternateBase, "#1b1e23"), (R.Text, TEXT), (R.Button, "#2a2e35"),
                      (R.ButtonText, TEXT), (R.Highlight, BLUE),
                      (R.HighlightedText, "#ffffff"), (R.PlaceholderText, DIM)]:
        p.setColor(role, QtGui.QColor(col))
    return p


def mono_font(pt: int, weight=None) -> QtGui.QFont:
    """A monospace QFont at `pt` points, whatever the platform has."""
    f = QtGui.QFont()
    f.setFamilies(list(MONO_FAMILIES))
    f.setStyleHint(QtGui.QFont.StyleHint.Monospace)
    f.setFixedPitch(True)
    f.setPointSize(pt)
    if weight is not None:
        f.setWeight(weight)
    return f


def lab(text="", color=None, size=None, bold=False, mono=False, rich=False):
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


def btn(text, width=None, name=None, h=44, enabled=True):
    b = QtWidgets.QPushButton(text)
    if name:
        b.setObjectName(name)
    if width:
        b.setFixedWidth(width)
    b.setFixedHeight(h)
    b.setEnabled(enabled)
    # Buttons never take keyboard focus: a focused button eats Space, which
    # is STAND. The keymap owns the keyboard.
    b.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    return b


def restyle(w: QtWidgets.QWidget, name: str) -> None:
    """Switch a widget's object-name style (e.g. estop -> estop_latched)."""
    if w.objectName() != name:
        w.setObjectName(name)
        w.style().unpolish(w)
        w.style().polish(w)


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
    lay = QtWidgets.QHBoxLayout()
    lay.setSpacing(spacing)
    lay.setContentsMargins(*margins)
    return _box(lay, items)


def vbox(*items, spacing=6, margins=(0, 0, 0, 0)):
    lay = QtWidgets.QVBoxLayout()
    lay.setSpacing(spacing)
    lay.setContentsMargins(*margins)
    return _box(lay, items)


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
    set_pill(w, text, color, filled)
    if width:
        w.setFixedWidth(width)
    return w


def set_pill(w, text, color, filled=True):
    bg = color if filled else "transparent"
    fg = "white" if filled else color
    w.setText(text)
    w.setStyleSheet(f"background:{bg}; color:{fg}; border:1px solid {color}; border-radius:10px;"
                    f" padding:2px 10px; font-weight:700; font-size:12px;")


def mkplot(h=None, xr=(-30, 0), yr=None, bottom=True, left_w=38):
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
        a.setStyle(tickFont=mono_font(9))
    pi.getAxis("left").setWidth(left_w)
    if not bottom:
        pi.hideAxis("bottom")
    # A PlotWidget's sizeHint is 640 x 480 pt: left to it, a column of plots
    # makes its tab 480 pt per row and wider than the pane. Take what the
    # layout gives (with the caller's minimum), not what the plot would like.
    P = QtWidgets.QSizePolicy.Policy
    p.setSizePolicy(P.Ignored, P.Ignored)
    if h:
        p.setFixedHeight(h)
    return p


def table(headers, rows=0, stretch_col=None, height=None):
    t = QtWidgets.QTableWidget(rows, len(headers))
    t.setHorizontalHeaderLabels(headers)
    t.verticalHeader().setVisible(False)
    t.setShowGrid(False)
    t.setAlternatingRowColors(True)
    t.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
    hh = t.horizontalHeader()
    hh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
    if stretch_col is not None:
        hh.setSectionResizeMode(stretch_col, QtWidgets.QHeaderView.ResizeMode.Stretch)
    t.verticalHeader().setDefaultSectionSize(26)
    if height:
        t.setFixedHeight(height)
    return t


def set_cell(t, r, c, text, color=None):
    it = t.item(r, c)
    if it is None:
        it = QtWidgets.QTableWidgetItem()
        t.setItem(r, c, it)
    if it.text() != text:
        it.setText(text)
    if color:
        it.setForeground(QtGui.QColor(color))


class Slider(QtWidgets.QWidget):
    """A labelled float slider: name, track, live value."""
    changed = QtCore.pyqtSignal(float)

    def __init__(self, name, lo, hi, val, fmt, steps=1000, note=None):
        super().__init__()
        self.lo, self.hi, self.fmt, self.steps = lo, hi, fmt, steps
        self.s = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.s.setRange(0, steps)
        self.s.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.v = lab("", mono=True)
        self.v.setFixedWidth(96)
        items = [lab(name, DIM), self.s, self.v]
        self.note = lab(note or "", AMBER, 11)
        items.append(self.note)
        self.setLayout(hbox(*items, spacing=10))
        self.set(val)
        self.s.valueChanged.connect(self._moved)

    def value(self) -> float:
        return self.lo + (self.hi - self.lo) * self.s.value() / self.steps

    def set(self, v: float) -> None:
        self.s.blockSignals(True)
        span = self.hi - self.lo     # 0 for a channel the run never varied
        self.s.setValue(round((v - self.lo) / span * self.steps) if span else 0)
        self.s.blockSignals(False)
        self.v.setText(self.fmt.format(v))

    def _moved(self, _):
        v = self.value()
        self.v.setText(self.fmt.format(v))
        self.changed.emit(v)


def is_text_input(w) -> bool:
    """A widget that takes typed text: while one has focus it gets the keys,
    as bimo_gui's WantCaptureKeyboard does -- typing a host name with an `a`
    in it must not arm the robot."""
    return isinstance(w, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                          QtWidgets.QPlainTextEdit, QtWidgets.QTextEdit))


class Flow(QtWidgets.QLayout):
    """Widgets left to right, wrapping onto another line when the width runs
    out (Qt's flow-layout example): a row of controls that must never set the
    width of the pane it sits in."""

    def __init__(self, *widgets, spacing=12, line_spacing=6):
        super().__init__()
        self._items: list[QtWidgets.QLayoutItem] = []
        self._h, self._v = spacing, line_spacing
        self.setContentsMargins(0, 0, 0, 0)
        for w in widgets:
            self.addWidget(w)

    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return QtCore.Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._place(QtCore.QRect(0, 0, width, 0), move=False)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._place(rect, move=True)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        s = QtCore.QSize()
        for it in self._items:
            s = s.expandedTo(it.minimumSize())
        m = self.contentsMargins()
        return s + QtCore.QSize(m.left() + m.right(), m.top() + m.bottom())

    def _place(self, rect, move):
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line = r.x(), r.y(), 0
        for it in self._items:
            if it.isEmpty():
                continue
            hint = it.sizeHint()
            if x > r.x() and x + hint.width() > r.right() + 1:
                x, y, line = r.x(), y + line + self._v, 0
            if move:
                it.setGeometry(QtCore.QRect(QtCore.QPoint(x, y), hint))
            x += hint.width() + self._h
            line = max(line, hint.height())
        return y + line - rect.y() + m.bottom()


def fit_table(t: QtWidgets.QTableWidget, width_from_contents: bool = True) -> None:
    """A table shown whole: its fixed height holds the header and every row
    (no scrollbar hides the last one or an empty-state line), and -- with
    width_from_contents -- it is never narrower than its columns, so none is
    cut off or scrolled out of sight; the layout around it wraps instead."""
    A = QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
    t.setHorizontalScrollBarPolicy(A)
    t.setVerticalScrollBarPolicy(A)
    hh = t.horizontalHeader()
    if width_from_contents:
        w = sum(max(hh.sectionSizeHint(c), t.sizeHintForColumn(c))
                for c in range(t.columnCount()) if not t.isColumnHidden(c))
        w += 2 * t.frameWidth()
        if t.minimumWidth() != w:
            t.setMinimumWidth(w)
    h = (hh.sizeHint().height() + 2 * t.frameWidth()
         + sum(t.rowHeight(r) for r in range(t.rowCount())))
    if t.height() != h or t.minimumHeight() != h:
        t.setFixedHeight(h)


class Columns(QtWidgets.QWidget):
    """Widgets side by side while the width allows, stacked when it does not:
    a tab's columns never make it wider than its pane (the tab scrolls down,
    never sideways)."""

    def __init__(self, *widgets, spacing=12, stretch=None):
        super().__init__()
        self._widgets = widgets
        self._stretch = stretch or (0,) * len(widgets)
        self._lay = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.Direction.LeftToRight)
        self._lay.setSpacing(spacing)
        self._lay.setContentsMargins(0, 0, 0, 0)
        for w, k in zip(widgets, self._stretch):
            self._lay.addWidget(w, k, QtCore.Qt.AlignmentFlag.AlignTop)
        self.setLayout(self._lay)

    def _side_by_side_width(self) -> int:
        return (sum(w.minimumSizeHint().width() for w in self._widgets)
                + self._lay.spacing() * (len(self._widgets) - 1))

    def minimumSizeHint(self):
        s = super().minimumSizeHint()
        return QtCore.QSize(max(w.minimumSizeHint().width() for w in self._widgets), s.height())

    def resizeEvent(self, ev):
        self._arrange(ev.size().width())
        super().resizeEvent(ev)

    def event(self, ev):
        # A child's minimum can grow with its contents (a table filling in):
        # look again whenever the children ask for a new layout.
        if ev.type() == QtCore.QEvent.Type.LayoutRequest:
            self._arrange(self.width())
        return super().event(ev)

    def _arrange(self, width: int) -> None:
        D = QtWidgets.QBoxLayout.Direction
        want = D.LeftToRight if width >= self._side_by_side_width() else D.TopToBottom
        if self._lay.direction() != want:
            self._lay.setDirection(want)
            align = QtCore.Qt.AlignmentFlag.AlignTop if want == D.LeftToRight else \
                QtCore.Qt.AlignmentFlag(0)
            for i, w in enumerate(self._widgets):
                self._lay.setAlignment(w, align)
                self._lay.setStretch(i, self._stretch[i] if want == D.LeftToRight else 0)
            self.updateGeometry()

    def stacked(self) -> bool:
        return self._lay.direction() == QtWidgets.QBoxLayout.Direction.TopToBottom
