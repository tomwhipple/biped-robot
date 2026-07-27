"""Classic isometric exploded-assembly line drawings from the real CAD.

Hidden-line-removed three-quarter-from-above projection (pythonOCC
`HLRBRep_PolyAlgo`) of every printed part and the servo mocks -- exploded along
their real insertion axes.  Every piece of mechanical hardware (screws, washers,
heat-set inserts, spline screws, the M5 thumbscrew) is drawn instead as a
SYMBOLIC 2D glyph: a solid, labelled side-profile (button-head dome + hatched
threads, self-tapper's pointed tip, washer's edge-on ring, insert's knurl bands,
thumbscrew's knurled head) aligned to the fastener's projected axis at its
projected seat, so you can tell a screw from a washer from an insert at a glance.
Horn / idler discs and the rubber sole pads stay real HLR line-art -- their
bolt-circle holes identify them.  Glyphs are drawn ENLARGED for clarity: one
representative glyph per fastener group with an "NX" multiplier, exactly like a
production exploded sheet, rather than four overlapping copies.

Dash-dot centerlines, numbered balloon callouts + an ITEM/QTY/DESCRIPTION table
(master, along the bottom so the drawing dominates the sheet) or direct leader
callouts (joint / hip-yaw).  Feet-DOWN, camera above the horizon, off-centre.

Run:
  .venv/bin/python cad/render_exploded_drawing.py joint|hip_yaw|upper_leg|full|all
  -> docs/assembly/exploded_{joint,hip_yaw,upper_leg,full}.svg (+ .png)

Excluded (electronic, per spec): wires / leads / zip-ties. Counts reconcile with
the docs/assembly.md §0 fastener table (M3×6 = 38 used, idler M3×8+washer = 24,
M3×10 = 20, M2.5×8 flat self-tap = 32, M2.5×8 pan self-tap = 24, M3×12 = 4,
M2.5×8 machine = 8, heat-set = 12, M5×20 = 1,
spline = 10, rubber pad = 2).

Idler-side screws are drawn as NORMAL hardware: through-holes are the confirmed
design intent (2026-07-23); the CAD/prints are being corrected to add them.
"""
import argparse
import math
import os
import shutil
import subprocess

from build123d import Cylinder, Box, Pos, Rot
import dimensions as D
import parts
import check_assembly as CA

from OCP.HLRBRep import HLRBRep_PolyAlgo, HLRBRep_PolyHLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.BRepMesh import BRepMesh_IncrementalMesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "assembly")

# --------------------------------------------------- camera: feet-down 3/4 above
AZ, EL = math.radians(35), math.radians(30)     # azimuth, look-down elevation


def _norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return (v[0] / m, v[1] / m, v[2] / m)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


# N = object -> camera (plane normal); Vx horizontal; Vy has +Z (world up -> up)
_N = _norm((math.cos(EL) * math.cos(AZ), math.cos(EL) * math.sin(AZ), math.sin(EL)))
_VX = _norm(_cross((0, 0, 1), _N))
_VY = _cross(_N, _VX)
_AX = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*_N), gp_Dir(*_VX))


def proj2d(x, y, z):
    return (x * _VX[0] + y * _VX[1] + z * _VX[2],
            x * _VY[0] + y * _VY[1] + z * _VY[2])


def project(solid):
    """HLR visible edges of a build123d solid -> list of 2D polylines."""
    shape = solid.wrapped
    BRepMesh_IncrementalMesh(shape, 0.2)
    algo = HLRBRep_PolyAlgo(shape)
    algo.Projector(HLRAlgo_Projector(_AX))
    algo.Update()
    hlr = HLRBRep_PolyHLRToShape()
    hlr.Update(algo)
    vis = hlr.VCompound()
    polys = []
    if vis is None:
        return polys
    exp = TopExp_Explorer(vis, TopAbs_EDGE)
    while exp.More():
        c = BRepAdaptor_Curve(TopoDS.Edge_s(exp.Current()))
        try:
            gc = GCPnts_QuasiUniformDeflection(c, 0.15)
            pts = [c.Value(gc.Parameter(i)) for i in range(1, gc.NbPoints() + 1)]
        except Exception:
            pts = [c.Value(c.FirstParameter()), c.Value(c.LastParameter())]
        polys.append([(p.X(), p.Y()) for p in pts])
        exp.Next()
    return polys


# ------------------------------------------------- real HLR hardware (discs/pads)
def _orient(solid, ax):
    if ax == "y":
        return Rot(90, 0, 0) * solid
    if ax == "x":
        return Rot(0, 90, 0) * solid
    return solid


def disc(od=D.SV_HORN_D, t=2.6):                   # horn / idler disc (real HLR)
    d = Cylinder(od / 2, t)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        d -= Pos(dx, dy, 0) * Cylinder(D.PAD_HOLE / 2, t + 0.3)
    return d - Cylinder(2.4, t + 0.3)


def pad():                                         # rubber sole pad (real HLR)
    return Box(106, 46, D.TPU_PROUD)


# =============================================================== fastener glyphs
# A fastener is drawn directly as SVG strokes -- NOT run through HLR.  It is a
# solid, labelled side-profile aligned to its projected axis so each species is
# distinguishable.  `pos` is the glyph's projected seat, `ax` a unit head->outward
# direction; the glyph's head sits on the outward side, shank points at the part.
AXV = {"x+": (1, 0, 0), "x-": (-1, 0, 0), "y+": (0, 1, 0), "y-": (0, -1, 0),
       "z+": (0, 0, 1), "z-": (0, 0, -1)}

# per-species base head/glyph size in *pixels* (>= 14 px so every head is legible)
GSIZE = {"screw": 17, "selftap": 16.5, "washer": 18, "heatset": 16,
         "spline": 14, "thumb": 18, "m2.5": 14}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def fastener_svg(cx, cy, ux, uy, f, kind, count, L):
    """Return SVG strings for one fastener glyph centred (seat) at (cx,cy).
    (ux,uy): unit screen vector pointing head->tip (into the assembly).
    f: axis foreshortening (0 = axis into the page -> draw a head-on symbol)."""
    g = GSIZE.get(kind, 16)
    px, py = -uy, ux                       # screen perpendicular
    hx, hy = -ux, -uy                      # head / outward direction (screen)
    FILL = "#fff"

    def M(t, n):
        return (cx + t * ux + n * px, cy + t * uy + n * py)

    def poly(P, w=1.15, fill="none", close=False):
        d = "M " + " L ".join("%.2f %.2f" % M(t, n) for t, n in P)
        if close:
            d += " Z"
        return ('<path d="%s" fill="%s" stroke="#111" stroke-width="%.2f" '
                'stroke-linecap="round" stroke-linejoin="round"/>' % (d, fill, w))

    def seg(t0, n0, t1, n1, w=0.85):
        x0, y0 = M(t0, n0); x1, y1 = M(t1, n1)
        return ('<path d="M %.2f %.2f L %.2f %.2f" stroke="#111" '
                'stroke-width="%.2f" fill="none" stroke-linecap="round"/>'
                % (x0, y0, x1, y1, w))

    def circle(t, n, r, w=1.15, fill="none"):
        x, y = M(t, n)
        return ('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s" stroke="#111" '
                'stroke-width="%.2f"/>' % (x, y, r, fill, w))

    out = []
    HEADON = f < 0.33

    # ---- head-on symbols (axis ~ view normal): tell-tale is the face pattern ---
    if HEADON:
        if kind in ("screw", "selftap", "spline"):
            out.append(circle(0, 0, g / 2, fill=FILL))
            if kind == "selftap":                       # cross-recess
                out += [seg(0, -g * 0.3, 0, g * 0.3), seg(-g * 0.3, 0, g * 0.3, 0)]
            elif kind == "spline":                      # splined boss
                for k in range(10):
                    a = 2 * math.pi * k / 10
                    out.append(seg(0.30 * g * math.cos(a), 0.30 * g * math.sin(a),
                                   0.46 * g * math.cos(a), 0.46 * g * math.sin(a)))
            else:                                       # hex socket
                hp = [(0.28 * g * math.cos(math.pi / 6 + math.pi / 3 * k),
                       0.28 * g * math.sin(math.pi / 6 + math.pi / 3 * k))
                      for k in range(6)]
                out.append(poly(hp, 1.0, close=True))
        elif kind == "washer":
            out += [circle(0, 0, g * 0.60, fill=FILL), circle(0, 0, g * 0.30)]
        elif kind == "heatset":
            out.append(circle(0, 0, g * 0.5, fill=FILL))
            for k in range(12):
                a = 2 * math.pi * k / 12
                out.append(seg(0.5 * g * math.cos(a), 0.5 * g * math.sin(a),
                               0.62 * g * math.cos(a), 0.62 * g * math.sin(a)))
        elif kind == "thumb":
            out.append(circle(0, 0, g * 0.7, fill=FILL))
            for k in range(16):
                a = 2 * math.pi * k / 16
                out.append(seg(0.7 * g * math.cos(a), 0.7 * g * math.sin(a),
                               0.82 * g * math.cos(a), 0.82 * g * math.sin(a)))
        lt, ln = 0, -g * 0.95
    # ---- side profiles --------------------------------------------------------
    elif kind == "washer":                     # foreshortened ring (annulus) = washer
        cosn = max(0.26, math.sqrt(max(0.0, 1 - f * f)))   # disc tilt -> ellipse
        Rmaj, ri = 0.66 * g, 0.34 * g

        def ell(R):
            return [(R * cosn * math.sin(a), R * math.cos(a))
                    for a in (2 * math.pi * k / 24 for k in range(25))]
        out.append(poly(ell(Rmaj), 1.15, FILL, close=True))
        out.append(poly(ell(ri), 1.0, FILL, close=True))
        lt, ln = 0, -Rmaj - 3
    elif kind == "heatset":                    # short cylinder + two knurl bands
        HL, w = 0.5 * g, 0.44 * g
        out.append(poly([(-HL, -w), (HL, -w), (HL, w), (-HL, w)], 1.1, FILL, close=True))
        for t0, t1 in ((-HL, -0.14 * g), (0.14 * g, HL)):
            for k in range(9):
                nn = -w * 0.86 + 2 * w * 0.86 * k / 8
                out.append(seg(t0, nn, t1, nn, 0.7))
        lt, ln = 0, -w - 3
    elif kind == "thumb":                       # wide knurled head + long shank
        HD, Hh, sw = 1.4 * g, 0.9 * g, 0.15 * g
        out.append(poly([(-Hh, -HD / 2), (0, -HD / 2), (0, HD / 2), (-Hh, HD / 2)],
                        1.15, FILL, close=True))
        for k in range(7):
            nn = -HD / 2 * 0.82 + HD * 0.82 * k / 6
            out.append(seg(-Hh + 1, nn, -0.05 * g, nn, 0.7))
        Lsh = 3.1 * g
        out.append(poly([(0, -sw), (Lsh, -sw), (Lsh, sw), (0, sw)], 1.15, FILL, close=True))
        lt, ln = 0, -HD / 2 - 3
    else:                                       # screw family (button / self-tap / spline)
        if kind == "spline":
            HD, Hh, sw = 1.0 * g, 0.34 * g, 0.20 * g
            Lsh = 1.5 * g
        else:
            HD, Hh, sw = g, 0.46 * g, 0.19 * g
            Lsh = _clamp(g * (1.5 + 0.11 * L), 1.9 * g, 3.5 * g)
        # dome head (outward, t<0) -> underside -> shank silhouette (one solid)
        P = []
        for k in range(13):
            ph = -math.pi / 2 + math.pi * k / 12
            P.append((-Hh * math.cos(ph), (HD / 2) * math.sin(ph)))
        if kind == "selftap":                   # flat pan head instead of dome
            P = [(0, -HD / 2), (-Hh, -HD / 2 * 0.9), (-Hh, HD / 2 * 0.9), (0, HD / 2)]
        P += [(0, sw)]
        if kind == "selftap":                   # pointed, self-tapping tip
            tip = 0.72 * Lsh
            P += [(tip, sw), (Lsh, 0), (tip, -sw), (0, -sw)]
        else:
            P += [(Lsh, sw), (Lsh, -sw), (0, -sw)]
        out.append(poly(P, 1.2, FILL, close=True))
        # thread ticks
        if kind == "selftap":                   # coarse, wide-spaced chevrons
            for fr in (0.30, 0.50, 0.68):
                ti = fr * Lsh
                out += [seg(ti, -sw, ti + 0.16 * g, 0), seg(ti + 0.16 * g, 0, ti, sw)]
        else:
            reps = (0.55, 0.72, 0.89) if kind != "spline" else (0.55, 0.85)
            for fr in reps:
                ti = fr * Lsh
                out.append(seg(ti, -sw, ti + 0.14 * g, sw))
        lt, ln = -Hh, -HD / 2 - 3

    # ---- "NX" multiplier just outside the head, on the outward side -----------
    if count and count > 1:
        if HEADON:
            lx, ly = cx, cy - g * 1.05 - 4
        else:
            lx, ly = cx + hx * (g * 1.15 + 6), cy + hy * (g * 1.15 + 6)
        out.append('<text x="%.2f" y="%.2f" font-size="9.5" font-weight="700" '
                   'text-anchor="middle" fill="#111">%dX</text>' % (lx, ly + 3, count))
    return out


# ------------------------------------------------ fastener alignment rays
# A thin dash-dot ray riding each fastener's projected axis (lighter than the
# main explosion centerlines): from just beyond the glyph's outer/head end,
# through the glyph body, ending past the tip so it visibly enters the part at
# its hole -- exactly like the alignment rays on a production exploded sheet.
RAY_DASH = "8 3 1.5 3"

# Muted-but-distinct, printable-on-white palette keyed by fastener group.  The
# matching callout leader (or balloon ring on the master) is tinted the same hue
# so every ray is traceable to its callout at a glance.  Part linework and the
# main explosion centerlines stay black.
FGROUP_COLOR = {
    "screw":   "#2563b8",   # blue        — machine (button-head) screws
    "idler":   "#0d9488",   # teal        — idler-disc screws (ride with washers)
    "washer":  "#0d9488",   # teal        — thin washers
    "selftap": "#d9531e",   # orange-red  — self-tappers
    "heatset": "#7c3aed",   # purple      — heat-set inserts
    "spline":  "#159141",   # green       — disc / spline centre screws
    "thumb":   "#b8860b",   # goldenrod   — M5 thumbscrew
}


def _fgroup(name, kind):
    """Map a fastener spec to its colour group.  Idler-disc machine screws share
    the washers' teal (they seat on the same axis); everything else keys on kind."""
    if kind in ("washer", "selftap", "heatset", "spline", "thumb"):
        return "washer" if kind == "washer" else kind
    n = (name or "").lower()           # screw family: split idler screws out
    if "idler" in n or n.startswith("is") or n.startswith("ri"):
        return "idler"
    return "screw"


def fgroup_color(name, kind):
    return FGROUP_COLOR.get(_fgroup(name, kind), "#6b6b6b")


def _glyph_extent(kind, f, L):
    """Axial half-extents (t_head<0 outward, t_tip>0 toward the part) of a glyph,
    matching the geometry drawn in fastener_svg.  None => axis into the page."""
    if f < 0.33:                       # head-on: axis ~ view normal, no axial ray
        return None
    g = GSIZE.get(kind, 16)
    if kind == "washer":
        cosn = max(0.26, math.sqrt(max(0.0, 1 - f * f)))
        return -0.66 * g * cosn, 0.66 * g * cosn
    if kind == "heatset":
        return -0.5 * g, 0.5 * g
    if kind == "thumb":
        return -0.9 * g, 3.1 * g
    if kind == "spline":
        return -0.34 * g, 1.5 * g
    Hh = 0.46 * g                      # screw / self-tap
    return -Hh, _clamp(g * (1.5 + 0.11 * L), 1.9 * g, 3.5 * g)


def fastener_ray(cx, cy, ux, uy, f, kind, L, color="#6b6b6b", min_px=30):
    """SVG dash-dot alignment ray for one fastener glyph seated at (cx,cy) with
    head->tip unit screen vector (ux,uy).  None if the axis points into the page
    or the projected ray is too short to read (declutter)."""
    ext = _glyph_extent(kind, f, L)
    if ext is None:
        return None
    t_head, t_tip = ext
    # Spline / centre screws mount the horn & idler discs onto the servo output
    # shaft -> carry their ray on toward that shaft; the rest just enter the hole.
    tip_ext = 58 if kind == "spline" else 24
    t0, t1 = t_head - 6, t_tip + tip_ext
    x0, y0 = cx + t0 * ux, cy + t0 * uy
    x1, y1 = cx + t1 * ux, cy + t1 * uy
    if math.hypot(x1 - x0, y1 - y0) < min_px:
        return None
    return ('<path d="M %.2f %.2f L %.2f %.2f" stroke="%s" '
            'stroke-width="0.5" stroke-dasharray="%s" fill="none" '
            'stroke-linecap="round"/>' % (x0, y0, x1, y1, color, RAY_DASH))


# ------------------------------------------------------------------- SVG sheet
def emit(parts_polys, thin_polys, fdata, centerlines, callouts, balloons,
         item_table, caption, out_svg, sheet=(1360, 900), table_mode="right",
         label_counts=True):
    W, H = sheet
    margin = 26
    COL = 210 if callouts else 40          # callout-column width (direct mode)
    if table_mode == "bottom" and item_table:
        ncol = 3
        per = -(-len(item_table) // ncol)   # ceil
        strip = per * 15 + 24
        tblw = 0
    else:
        strip = 0
        tblw = 330 if item_table else 0
    top_pad = 40
    dl = margin + COL
    dr = W - margin - max(COL, tblw + 10)
    dt = margin + top_pad
    db = (H - margin - strip - 52) if strip else (H - margin - 74)

    allpts = [p for _, pls in parts_polys + thin_polys for pl in pls for p in pl]
    allpts += [(u, v) for _, u, v, *_ in fdata]
    xs = [p[0] for p in allpts]; ys = [p[1] for p in allpts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    s = min((dr - dl) / (maxx - minx + 1e-6), (db - dt) / (maxy - miny + 1e-6))
    ox = dl + ((dr - dl) - (maxx - minx) * s) / 2
    oy = dt + ((db - dt) - (maxy - miny) * s) / 2

    def X(x): return ox + (x - minx) * s
    def Y(y): return oy + (maxy - y) * s

    def esc(t):
        return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    def path(pls, w):
        d = " ".join("M " + " L ".join("%.2f %.2f" % (X(x), Y(y)) for x, y in pl)
                     for pl in pls if len(pl) > 1)
        return ('<path d="%s" fill="none" stroke="#111" stroke-width="%.2f" '
                'stroke-linecap="round" stroke-linejoin="round"/>' % (d, w))

    el = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
          'font-family="Helvetica,Arial,sans-serif">' % (W, H),
          '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H),
          '<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#111" '
          'stroke-width="1.5"/>' % (margin, margin, W - 2 * margin, H - 2 * margin)]

    for x0, y0, x1, y1 in centerlines:
        el.append('<path d="M %.2f %.2f L %.2f %.2f" stroke="#111" stroke-width="0.7" '
                  'stroke-dasharray="12 4 2 4" fill="none"/>' % (X(x0), Y(y0), X(x1), Y(y1)))

    # ---- per-fastener alignment rays (BENEATH the part linework so they never
    # obscure an edge; colour-keyed to the fastener group and its callout) ----
    for name, u, v, au, av, f, kind, count, L in fdata:
        cx, cy = X(u), Y(v)
        hs = math.hypot(au, -av)
        if hs < 1e-6:
            continue                          # axis into the page -> no useful ray
        ux, uy = -au / hs, av / hs            # head->tip; Y is flipped
        ray = fastener_ray(cx, cy, ux, uy, f, kind, L, color=fgroup_color(name, kind))
        if ray:
            el.append(ray)

    for _, pls in parts_polys:
        el.append(path(pls, 1.1))
    for _, pls in thin_polys:
        el.append(path(pls, 0.85))

    # ---- fastener glyphs (drawn over the line-art so heads stay legible) ----
    for name, u, v, au, av, f, kind, count, L in fdata:
        cx, cy = X(u), Y(v)
        hs = math.hypot(au, -av)              # head screen vector (pre-normalise)
        if hs < 1e-6:
            ux, uy = 0.0, 1.0
        else:
            ux, uy = -au / hs, av / hs        # head->tip = -(head dir); Y is flipped
        el += fastener_svg(cx, cy, ux, uy, f, kind, count if label_counts else 1, L)

    # ---- direct leader callouts (stacked columns) ----
    for side in ("L", "R"):
        grp = [c for c in callouts if c[3] == side]
        grp.sort(key=lambda c: -c[1])
        cur = dt + 4
        tx = dl - 12 if side == "L" else dr + 12
        anchor = "end" if side == "L" else "start"
        for gx, gy, lines, _, col in grp:
            bh = len(lines) * 14.5 + 14
            ty = min(max(cur, Y(gy) - len(lines) * 7), db - bh)
            cur = ty + bh
            el.append('<circle cx="%.2f" cy="%.2f" r="2.1" fill="%s"/>' % (X(gx), Y(gy), col))
            el.append('<path d="M %.2f %.2f L %.2f %.2f L %.2f %.2f" stroke="%s" '
                      'stroke-width="0.9" fill="none"/>'
                      % (X(gx), Y(gy), tx + (10 if side == "L" else -10), ty + 4, tx, ty + 4, col))
            for i, ln in enumerate(lines):
                el.append('<text x="%.2f" y="%.2f" font-size="12.5" font-weight="%s" '
                          'text-anchor="%s" fill="#111">%s</text>'
                          % (tx, ty + 4 + i * 14.5, "700" if i == 0 else "400", anchor, esc(ln)))

    # ---- numbered balloons (circle + item no, short leader) ----
    for gx, gy, num, ang, col in balloons:
        bx = X(gx) + 34 * math.cos(math.radians(ang))
        by = Y(gy) - 34 * math.sin(math.radians(ang))
        el.append('<path d="M %.2f %.2f L %.2f %.2f" stroke="%s" stroke-width="0.7" '
                  'fill="none"/>' % (X(gx), Y(gy), bx, by, col))
        el.append('<circle cx="%.2f" cy="%.2f" r="2.0" fill="%s"/>' % (X(gx), Y(gy), col))
        el.append('<circle cx="%.2f" cy="%.2f" r="11" fill="#fff" stroke="%s" '
                  'stroke-width="%.1f"/>' % (bx, by, col, 1.6 if col != "#111" else 1.1))
        el.append('<text x="%.2f" y="%.2f" font-size="12" font-weight="700" '
                  'text-anchor="middle" fill="#111">%s</text>' % (bx, by + 4.2, num))

    # ---- item table: right column (default) or compact bottom strip (master) ---
    if item_table and table_mode == "bottom":
        rh = 15
        y0 = H - margin - strip + 2
        colw = (W - 2 * margin) / ncol
        el.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#111" '
                  'stroke-width="1"/>' % (margin, y0 - 6, W - margin, y0 - 6))
        el.append('<text x="%.1f" y="%.1f" font-size="11" font-weight="700" '
                  'fill="#111">ITEM · QTY · DESCRIPTION</text>' % (margin + 2, y0 + 8))
        for i, (num, qty, desc) in enumerate(item_table):
            col = i // per
            row = i % per
            bx = margin + col * colw + 12
            ry = y0 + 20 + row * rh
            el.append('<circle cx="%.1f" cy="%.1f" r="6.6" fill="#fff" stroke="#111" '
                      'stroke-width="1"/>' % (bx, ry - 3.5))
            el.append('<text x="%.1f" y="%.1f" font-size="9" font-weight="700" '
                      'text-anchor="middle" fill="#111">%s</text>' % (bx, ry - 1, num))
            el.append('<text x="%.1f" y="%.1f" font-size="9.5" font-weight="700" '
                      'fill="#111">%s</text>' % (bx + 12, ry, qty))
            el.append('<text x="%.1f" y="%.1f" font-size="9.5" fill="#111">%s</text>'
                      % (bx + 30, ry, esc(desc)))
    elif item_table:
        rh, tx0 = 18, W - margin - tblw - 6
        ty0 = margin + 12
        el.append('<rect x="%.1f" y="%.1f" width="%d" height="%.1f" fill="none" '
                  'stroke="#111" stroke-width="1.2"/>'
                  % (tx0, ty0, tblw, rh * (len(item_table) + 1)))
        hdr = ["ITEM", "QTY", "DESCRIPTION"]
        cols = [tx0 + 6, tx0 + 46, tx0 + 96]
        for j, hx in enumerate(cols):
            el.append('<text x="%.1f" y="%.1f" font-size="11.5" font-weight="700" '
                      'fill="#111">%s</text>' % (hx, ty0 + 13, hdr[j]))
        el.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#111" '
                  'stroke-width="1"/>' % (tx0, ty0 + rh, tx0 + tblw, ty0 + rh))
        for i, (num, qty, desc) in enumerate(item_table):
            ry = ty0 + rh * (i + 1)
            el.append('<circle cx="%.1f" cy="%.1f" r="8" fill="#fff" stroke="#111" '
                      'stroke-width="1"/>' % (cols[0] + 8, ry + 12))
            el.append('<text x="%.1f" y="%.1f" font-size="10.5" font-weight="700" '
                      'text-anchor="middle" fill="#111">%s</text>' % (cols[0] + 8, ry + 15.5, num))
            el.append('<text x="%.1f" y="%.1f" font-size="11" fill="#111">%s</text>'
                      % (cols[1], ry + 13, qty))
            el.append('<text x="%.1f" y="%.1f" font-size="11" fill="#111">%s</text>'
                      % (cols[2], ry + 13, esc(desc)))

    cap_y = (H - margin - strip - 16) if strip else (H - margin - 24)
    el.append('<text x="%d" y="%d" font-size="22" font-weight="700" '
              'text-anchor="middle" fill="#111" font-style="italic">%s</text>'
              % (W / 2, cap_y, esc(caption)))
    el.append('<text x="%d" y="%d" font-size="11" letter-spacing="1.5" '
              'text-anchor="middle" fill="#111">FASTENERS ENLARGED FOR CLARITY · '
              'ONE GLYPH SHOWN PER GROUP (NX)</text>' % (W / 2, cap_y + 15))
    el.append("</svg>")
    with open(out_svg, "w") as f:
        f.write("\n".join(el))
    png = out_svg[:-4] + ".png"
    if shutil.which("qlmanage"):
        subprocess.run(["qlmanage", "-t", "-s", str(W), "-o", "/tmp", out_svg],
                       capture_output=True)
        tmp = os.path.join("/tmp", os.path.basename(out_svg) + ".png")
        if os.path.exists(tmp):
            shutil.move(tmp, png)
    return out_svg, png


# ------------------------------------------------------------- build dispatcher
def render(items, centerlines_3d, caption, out_name, callouts_raw=(),
           balloon_defs=(), item_table=(), fasteners=(), sheet=(1360, 900),
           table_mode="right", label_counts=True):
    """items: [(name, solid, thin?)] -> real HLR line-art.
    fasteners: [(name, pos(x,y,z), axdir, kind, count, L)] -> symbolic glyphs.
    callouts_raw: [(name, lines, side)].  balloon_defs: [(name, number, angle)].
    Both callouts and balloons may target a part OR a fastener by name."""
    parts_polys, thin_polys, centroids = [], [], {}
    for name, solid, thin in items:
        pls = project(solid)
        if not pls:
            continue
        pts = [p for pl in pls for p in pl]
        centroids[name] = (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))
        (thin_polys if thin else parts_polys).append((name, pls))

    fdata = []
    for name, pos, ax, kind, count, L in fasteners:
        u, v = proj2d(*pos)
        au, av = proj2d(*AXV[ax])
        f = math.hypot(au, av)
        centroids[name] = (u, v)
        fdata.append((name, u, v, au, av, f, kind, count, L))

    # callout/balloon targeting a fastener is tinted its ray colour; parts -> black
    fcolor = {name: fgroup_color(name, kind)
              for name, _, ax, kind, count, L in fasteners}
    callouts = [(centroids[t][0], centroids[t][1], lines, side, fcolor.get(t, "#111"))
                for t, lines, side in callouts_raw if t in centroids]
    balloons = [(centroids[t][0], centroids[t][1], num, ang, fcolor.get(t, "#111"))
                for t, num, ang in balloon_defs if t in centroids]
    centerlines = [proj2d(*a) + proj2d(*b) for a, b in centerlines_3d]
    return emit(parts_polys, thin_polys, fdata, centerlines, callouts, balloons,
                list(item_table), caption, os.path.join(OUT, out_name),
                sheet=sheet, table_mode=table_mode, label_counts=label_counts)


# ==================================================================== JOINT
def joint():
    items = [("SERVO STS3215", CA.servo_mock_y(), False)]
    fork = Pos(90, 0, D.LINK_DROP) * parts.leg_link()
    items.append(("LEG_LINK  fork", fork, False))
    # real HLR discs (bolt-circle holes identify them)
    items.append(("horn_disc", Pos(0, D.SV_HORN_FACE + 16, 0) * _orient(disc(), "y"), False))
    items.append(("idler_disc", Pos(0, D.SV_IDLER_FACE - 16, 0) * _orient(disc(), "y"), False))

    r = D.BCD / 2
    fasteners = [
        # horn side (+Y): 4X M3×6 button head + spline centre screw
        ("hornscrew", (0, D.SV_HORN_FACE + 33, r), "y+", "screw", 4, 6),
        ("spline", (0, D.SV_HORN_FACE + 54, 0), "y+", "spline", 1, 4),
        # idler side (-Y): 4X M3×8 button head + thin washer (screw carries the 4X);
        # both exploded well clear of the fork plate into open space
        ("washer", (0, D.SV_IDLER_FACE - 48, r), "y-", "washer", 1, 0),
        ("idlerscrew", (0, D.SV_IDLER_FACE - 62, r), "y-", "screw", 4, 8),
        # 6X M2.5×8 FLAT self-tap case grips (flush; into the adjacent link)
        ("casegrip", (D.CASE_HOLE_LAT, D.SV_TOPFACE + 24, -8.3), "y+", "selftap", 6, 8),
    ]
    cl = [((0, -60, 0), (0, 90, 0)), ((-40, 0, 0), (128, 0, 0))]
    callouts = [
        ("SERVO STS3215", ["SERVO  STS3215", "case gripped by", "the adjacent link"], "L"),
        ("LEG_LINK  fork", ["LEG_LINK  (fork)", "rotates with the horn"], "R"),
        ("horn_disc", ["HORN DISC + 4X M3×6", "button head into the", "horn (servo's kit)"], "R"),
        ("spline", ["SPLINE SCREW", "horn onto shaft (§1)"], "R"),
        ("idlerscrew", ["4X M3×8 + WASHER", "into the idler disc", "(same Ø14 pattern)"], "L"),
        ("idler_disc", ["IDLER DISC", "free-spinning; same", "Ø14 pattern"], "L"),
        ("casegrip", ["6X M2.5×8 FLAT S-TAP", "case grip, heads FLUSH", "(bench skew fix)"], "R"),
    ]
    return render(items, cl, "FIGURE J — TYPICAL LEG JOINT (KNEE)  ·  all hardware",
                  "exploded_joint.svg", callouts_raw=callouts, fasteners=fasteners)


# ==================================================================== HIP YAW
def hip_yaw():
    by = 0.0
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    items = [
        ("PELVIS", Pos(0, by, DECK + 78) * parts.pelvis(), False),
        ("YAW SERVO", Pos(0, by, D.HIP_YAW_Z + D.SV_HORN_FACE + 26) * CA.servo_mock_z(), False),
        ("YAW_CARRIER", Pos(0, by, D.HIP_YAW_Z - 46) * parts.yaw_carrier(), False),
        ("ROLL SERVO", Pos(0, by, D.HIP_ROLL_Z - 96) * CA.servo_mock_x(), False),
        ("yaw_idler", Pos(0, by, D.HIP_YAW_Z + D.SV_GRIP_SPAN + 8) * disc(), False),
    ]
    r = D.BCD / 2
    fasteners = [
        # 4X M2.5×8 pan self-tap yaw stators, down through the deck (exploded up)
        ("ys", (-D.YAW_CASE_HOLES_IDLER[0], by + D.CASE_HOLE_LAT, DECK + 46), "z+", "selftap", 4, 8),
        # 4X M3×6 carrier -> yaw horn (O14 circle)
        ("ch", (0, by + r, D.HIP_YAW_Z - 22), "z-", "screw", 4, 6),
        ("yaw_spline", (0, by, D.HIP_YAW_Z - 40), "z-", "spline", 1, 4),
        # 8X M2.5×8 pan self-tap roll servo into the carrier bay walls (exploded sideways)
        ("rb", (0, by + D.CASE_HOLE_LAT + 26, D.HIP_YAW_Z + D.CARRIER_ROLL_AXIS + 4),
         "y+", "selftap", 8, 8),
    ]
    cl = [((0, by, D.HIP_ROLL_Z - 120), (0, by, DECK + 92))]
    callouts = [
        ("PELVIS", ["PELVIS", "deck + 2 flat yaw seats"], "L"),
        ("ys", ["4X M2.5×8 PAN S-TAP", "down through deck ->", "case idler rows"], "L"),
        ("YAW SERVO", ["YAW SERVO  STS3215", "flat, horn DOWN"], "R"),
        ("yaw_idler", ["YAW IDLER DISC", "(against deck)"], "R"),
        ("ch", ["4X M3×6 -> yaw horn", "(O14 bolt circle)"], "R"),
        ("YAW_CARRIER", ["YAW_CARRIER", "holds the roll bay"], "L"),
        ("rb", ["8X M2.5×8 PAN S-TAP", "roll servo into the", "carrier bay walls"], "R"),
        ("ROLL SERVO", ["ROLL SERVO  STS3215", "slides UP into carrier"], "L"),
    ]
    return render(items, cl, "FIGURE Y — HIP-YAW STACK (v3yaw)  ·  all hardware",
                  "exploded_hip_yaw.svg", callouts_raw=callouts, fasteners=fasteners)


# ============================================================== UPPER LEG (knee->pelvis)
def upper_leg():
    """One leg from the KNEE up through pitch, roll and yaw into the pelvis.

    Positions come straight from export_assembly.leg() -- same Z heights, same
    servo mocks -- so this cannot drift from the real assembly. Only the explode
    offsets are drawing-specific.
    """
    by = 0.0
    at = lambda z, dz=0.0: Pos(0, by, z + dz)
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    items = [
        ("PELVIS", Pos(0, by, DECK + 96) * parts.pelvis(), False),
        ("YAW SERVO", at(D.HIP_YAW_Z + D.SV_HORN_FACE, 44) * CA.servo_mock_z(), False),
        ("yaw_idler", at(D.HIP_YAW_Z + D.SV_GRIP_SPAN, 26) * disc(), False),
        ("YAW_CARRIER", at(D.HIP_YAW_Z, -14) * parts.yaw_carrier(), False),
        ("ROLL SERVO", at(D.HIP_ROLL_Z, -58) * CA.servo_mock_x(), False),
        ("YOKE_ROLL", at(D.HIP_ROLL_Z, -104) * parts.yoke_roll(), False),
        ("YOKE_PITCH", at(D.HIP_PITCH_Z, -150) * parts.yoke_pitch(), False),
        ("PITCH SERVO", at(D.HIP_PITCH_Z, -196) * CA.servo_mock_y(), False),
        ("LINK_THIGH", at(D.HIP_PITCH_Z, -244) * parts.leg_link(), False),
        ("KNEE SERVO", at(D.KNEE_Z, -290) * CA.servo_mock_y(), False),
    ]
    r = D.BCD / 2
    fasteners = [
        ("ys", (-D.YAW_CASE_HOLES_IDLER[0], by + D.CASE_HOLE_LAT, DECK + 66),
         "z+", "selftap", 4, 8),
        ("ch", (0, by + r, D.HIP_YAW_Z + 6), "z-", "screw", 4, 6),
        ("rb", (0, by + D.CASE_HOLE_LAT + 26,
                D.HIP_YAW_Z + D.CARRIER_ROLL_AXIS - 30), "y+", "selftap", 8, 8),
        ("rh", (0, by + r, D.HIP_ROLL_Z - 82), "y+", "screw", 4, 6),
        # the ONLY thing joining the two yokes: 4X M3x10 down through
        # yoke_roll's flange into the heat-sets in yoke_pitch's flange
        ("fl", (0, by + D.YOKE_BOLT_SQ / 2, D.HIP_ROLL_Z - 132),
         "z-", "screw", 4, 10),
        ("hs", (0, by - D.YOKE_BOLT_SQ / 2, D.HIP_ROLL_Z - 146),
         "z-", "insert", 4, 5),
        ("ph", (0, by + r, D.HIP_PITCH_Z - 172), "y+", "screw", 4, 6),
        ("pg", (D.CASE_HOLE_LAT, by + D.SV_TOPFACE + 22, D.HIP_PITCH_Z - 220),
         "y+", "selftap", 6, 8),
        ("kg", (D.CASE_HOLE_LAT, by + D.SV_TOPFACE + 22, D.KNEE_Z - 268),
         "y+", "selftap", 6, 8),
    ]
    cl = [((0, by, D.KNEE_Z - 312), (0, by, DECK + 112))]
    callouts = [
        ("PELVIS", ["PELVIS", "deck + flat yaw seat"], "L"),
        ("ys", ["4X M2.5x8 PAN S-TAP", "deck -> yaw case"], "L"),
        ("YAW SERVO", ["HIP YAW  STS3215", "flat, horn DOWN"], "R"),
        ("ch", ["4X M3x6 -> yaw horn"], "R"),
        ("YAW_CARRIER", ["YAW_CARRIER", "carries the roll bay"], "L"),
        ("rb", ["8X M2.5x8 PAN S-TAP", "roll servo -> bay walls"], "R"),
        ("ROLL SERVO", ["HIP ROLL  STS3215", "slides UP into carrier"], "L"),
        ("YOKE_ROLL", ["YOKE_ROLL", "roll horn+idler ->", "flange onto yoke_pitch"], "R"),
        ("fl", ["4X M3x10 -> HEAT-SETS", "yoke_roll flange down", "into yoke_pitch"], "R"),
        ("hs", ["4X M3 HEAT-SET", "pressed into the", "yoke_pitch flange (\u00a72)"], "L"),
        ("YOKE_PITCH", ["YOKE_PITCH", "bolts on ROTATED 90\u00b0 --", "that is the universal joint"], "L"),
        ("PITCH SERVO", ["HIP PITCH  STS3215"], "R"),
        ("LINK_THIGH", ["LINK_THIGH  (leg_link)", "hip pitch -> knee"], "L"),
        ("KNEE SERVO", ["KNEE  STS3215", "case gripped by thigh"], "R"),
    ]
    return render(items, cl,
                  "FIGURE U - UPPER LEG: KNEE THROUGH HIP TO PELVIS  ·  all hardware",
                  "exploded_upper_leg.svg", callouts_raw=callouts,
                  fasteners=fasteners)


# ==================================================================== FULL master
def full():
    import export_assembly as A
    items, fasteners, balloons, table = [], [], [], []
    LIFT = {"pelvis": 150, "tower": 250, "battery": 235, "imu": 330, "gopro": 380,
            "camera": 470, "servo_hip_yaw": 92, "yaw_carrier": 30,
            "servo_hip_roll": -34, "yoke_roll": -95, "yoke_pitch": -150,
            "link_thigh": -175, "servo_hip_pitch": -175, "servo_knee": -330,
            "link_shin": -355, "servo_ankle": -500, "foot": -540}

    def lift(label):
        for k, v in LIFT.items():
            if label.startswith(k):
                return v
        return 0

    def collect(c, tag=""):
        for ch in c.children:
            if list(ch.children):
                collect(ch, ch.label.replace("leg_", "") + "_")
            else:
                items.append((tag + ch.label, Pos(0, 0, lift(ch.label)) * ch, False))
    collect(A.robot)

    for s, tag in ((1, "L"), (-1, "R")):
        items.append(("pad_%s" % tag, Pos(0, s * D.HIP_SEP / 2, lift("foot") - 40)
                      * pad(), True))

    rep = {"pelvis": ("1", 90), "L_yaw_carrier": ("2", 90), "L_yoke_roll": ("3", 180),
           "L_yoke_pitch": ("4", 180), "L_link_thigh": ("5", 0),
           "L_foot": ("6", 180), "tower": ("7", 200),
           "gopro_base": ("8", 200), "imu_carrier": ("9", 20),
           "L_servo_knee": ("10", 0), "L_servo_hip_roll": ("11", 180),
           "pad_L": ("12", 200)}
    for name, (num, ang) in rep.items():
        balloons.append((name, num, ang))

    r = D.BCD / 2
    # ---- one representative glyph per fastener group, at true joint heights ----
    # Hardware is shown on the LEFT leg only (typical, both sides) so the master
    # stays legible; the QTY table gives the robot-total counts.
    for s in (1,):
        y = s * D.HIP_SEP / 2
        for jz in (D.ANKLE_Z, D.KNEE_Z, D.HIP_PITCH_Z):
            jk = int(jz)
            fasteners.append(("h%d_%d" % (s, jk),
                              (0, y + D.SV_HORN_FACE + 30, jz + r), "y+", "screw", 4, 6))
            fasteners.append(("iw%d_%d" % (s, jk),
                              (0, y + D.SV_IDLER_FACE - 34, jz + r), "y-", "washer", 4, 0))
            fasteners.append(("is%d_%d" % (s, jk),
                              (0, y + D.SV_IDLER_FACE - 48, jz + r), "y-", "screw", 4, 8))
            fasteners.append(("sp%d_%d" % (s, jk),
                              (0, y + D.SV_HORN_FACE + 52, jz), "y+", "spline", 1, 4))
        # hip-roll (axis X): horn M3×6 + idler M3×10 + spline
        fasteners.append(("rh%d" % s, (D.SV_HORN_FACE + 30, y + r, D.HIP_ROLL_Z),
                          "x+", "screw", 4, 6))
        fasteners.append(("ri%d" % s, (-D.SV_HORN_FACE - 34, y + r, D.HIP_ROLL_Z),
                          "x-", "screw", 4, 10))
        fasteners.append(("spr%d" % s, (D.SV_HORN_FACE + 52, y, D.HIP_ROLL_Z),
                          "x+", "spline", 1, 4))
        # hip-yaw: 4X carrier->horn + spline
        fasteners.append(("ch%d" % s, (0, y + r, D.HIP_YAW_Z - 22), "z-", "screw", 4, 6))
        fasteners.append(("spy%d" % s, (0, y, D.HIP_YAW_Z - 40), "z-", "spline", 1, 4))
        # yoke_pitch flange heat-sets (4) + M3×10 flange bolts (4)
        fasteners.append(("hs%d" % s, (12, y + 12, D.HIP_PITCH_Z + 40), "z+", "heatset", 4, 0))
        fasteners.append(("fb%d" % s, (12, y + 12, D.HIP_PITCH_Z + 56), "z+", "screw", 4, 10))
        # foot / case-grip self-tap group representative
        fasteners.append(("ft%d" % s, (-24, y + s * 24, D.ANKLE_Z - 8), "y+", "selftap", 0, 8))

    # torso hardware
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    fasteners += [
        ("dh", (D.TOWER_FOOT_X, D.TOWER_FOOT_Y, DECK + 60), "z+", "heatset", 4, 0),
        ("tf", (D.TOWER_FOOT_X, D.TOWER_FOOT_Y, DECK + 150), "z+", "screw", 4, 10),
        ("gp", (11, 8.5, DECK + 380 + 24), "z+", "selftap", 4, 12),
        ("bd", (18, 0, DECK + 330 + 10), "z+", "selftap", 8, 8),
        ("m5", (0, 0, DECK + 470 + 34), "z+", "thumb", 1, 20),
    ]

    hw_rep = [("h1_%d" % int(D.KNEE_Z), "13", 0), ("is1_%d" % int(D.KNEE_Z), "14", 200),
              ("iw1_%d" % int(D.KNEE_Z), "15", 160), ("rh1", "16", 0),
              ("ri1", "17", 200), ("ft1", "18", 200), ("hs1", "19", 30),
              ("fb1", "20", 60), ("tf", "21", 0), ("gp", "22", 0),
              ("m5", "23", 0), ("ch1", "24", 30), ("spy1", "25", 210),
              ("bd", "26", 340)]
    for nm, num, ang in hw_rep:
        balloons.append((nm, num, ang))

    table = [
        ("1", "1", "pelvis"), ("2", "2", "yaw_carrier"), ("3", "2", "yoke_roll"),
        ("4", "2", "yoke_pitch"), ("5", "4", "leg_link (thigh+shin)"),
        ("6", "2", "foot"), ("7", "1", "tower"), ("8", "1", "gopro_base"),
        ("9", "1", "imu_carrier"), ("10", "10", "STS3215 servo"),
        ("11", "20", "horn + idler disc (w/ servos)"),
        ("12", "2", "rubber sole pad"),
        ("13", "24", "M3×6 button — pitch/knee/ankle horns"),
        ("14", "24", "M3×8 button — idler-disc screws"),
        ("15", "24", "M3 thin washer (idler)"),
        ("16", "8", "M3×6 button — hip-roll horns"),
        ("17", "8", "M3×10 button — hip-roll idler"),
        ("18", "32", "M2.5×8 FLAT self-tap — grips + feet (flush)"),
        ("18b", "24", "M2.5×8 pan self-tap — stators + bay walls"),
        ("19", "12", "M3 heat-set insert (Ø4.6)"),
        ("20", "8", "M3×10 button — yoke flange -> inserts"),
        ("21", "4", "M3×10 button — tower feet -> deck"),
        ("22", "4", "M3×12 self-tap — gopro/imu stack"),
        ("23", "1", "M5×20 thumbscrew — camera"),
        ("24", "8", "M3×6 button — yaw carriers (4× ea)"),
        ("25", "10", "servo spline/centre screw"),
        ("26", "8", "M2.5×8 self-tap — board + IMU"),
    ]
    cl = [((0, 0, lift("foot") - 60), (0, 0, DECK + 470 + 70))]
    return render(items, cl, "FIGURE 1 — BIMO BIPED v3yaw  ·  COMPLETE MECHANICAL ASSY",
                  "exploded_full.svg", balloon_defs=balloons, item_table=table,
                  fasteners=fasteners, sheet=(1440, 1440), table_mode="bottom",
                  label_counts=False)


DRAWINGS = {"joint": joint, "hip_yaw": hip_yaw, "upper_leg": upper_leg,
            "full": full}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=list(DRAWINGS) + ["all"])
    a = ap.parse_args()
    for n in (list(DRAWINGS) if a.which == "all" else [a.which]):
        svg, png = DRAWINGS[n]()
        print("wrote %s  +  %s" % (svg, png))
