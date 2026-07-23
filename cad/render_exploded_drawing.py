"""Classic isometric exploded-assembly line drawings from the real CAD.

Hidden-line-removed three-quarter-from-above projection (pythonOCC
`HLRBRep_PolyAlgo`) of every printed part, servo mock AND every piece of
mechanical hardware -- screws, washers, heat-set inserts, horn/idler discs,
spline screws, the M5 thumbscrew and the rubber sole pads -- each a glyph at its
true hole/seat position, exploded along its real insertion axis. Dash-dot
centerlines, numbered balloon callouts + an ITEM/QTY/DESCRIPTION table (master)
or direct leader callouts (joint / hip-yaw). Feet-DOWN, camera above the
horizon, off-centre -- like standing over the workbench.

Run:
  .venv/bin/python cad/render_exploded_drawing.py joint|hip_yaw|full|all
  -> docs/assembly/exploded_{joint,hip_yaw,full}.svg (+ .png)

Excluded (electronic, per spec): wires / leads / zip-ties. Everything mechanical
is included; counts reconcile with the docs/assembly.md §0 fastener table
(M3×6 = 38 used, idler M3×8+washer = 24, M3×10 = 20, M3×8 self-tap = 56,
M3×12 = 4, M2.5×8 = 8, heat-set = 12, M5×20 = 1, spline = 10, rubber pad = 2).

Idler-side screws are drawn as NORMAL hardware: through-holes are the confirmed
design intent (2026-07-23); the CAD/prints are being corrected to add them.
"""
import argparse
import math
import os
import shutil
import subprocess

from build123d import Cylinder, Cone, Box, Pos, Rot
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


# --------------------------------------------------------------- glyph library
# All glyphs are modelled along +Z (head up), then rotated to the bolt axis by
# `ax` in ('x','y','z'), and shifted `gap` mm back along that axis (explosion).
def _orient(solid, ax):
    if ax == "y":
        return Rot(90, 0, 0) * solid
    if ax == "x":
        return Rot(0, 90, 0) * solid
    return solid


def screw(length=8.0, head_d=6.0, shank_d=3.2):
    return Cylinder(head_d / 2, 2.2) + Pos(0, 0, -length / 2 - 1.1) * Cylinder(shank_d / 2, length)


def washer(od=7.4, idd=3.6, t=0.9):
    return Cylinder(od / 2, t) - Cylinder(idd / 2, t + 0.2)


def heatset(d=D.HEATSET_D, l=5.0):                 # brass insert glyph
    return Cylinder(d / 2, l) - Cylinder(1.5, l + 0.2)


def disc(od=D.SV_HORN_D, t=2.6):                   # horn / idler disc
    d = Cylinder(od / 2, t)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        d -= Pos(dx, dy, 0) * Cylinder(D.PAD_HOLE / 2, t + 0.3)
    return d - Cylinder(2.4, t + 0.3)


def thumbscrew():
    return Cylinder(5.0, 3.2) + Pos(0, 0, -11) * Cylinder(2.5, 20)


def pad():                                         # rubber sole pad
    return Box(106, 46, D.TPU_PROUD)


def spline():
    return screw(length=4, head_d=5, shank_d=2.6)


# ------------------------------------------------------------------- SVG sheet
def emit(parts_polys, thin_polys, centerlines, callouts, balloons, item_table,
         caption, out_svg, sheet=(1360, 900)):
    W, H = sheet
    margin = 26
    COL = 210 if callouts else 40          # callout-column width (direct mode)
    tblw = 330 if item_table else 0        # item-table reserve (right side)
    top_pad, bot_pad = 40, 74
    dl = margin + COL
    dr = W - margin - max(COL, tblw + 10)
    dt, db = margin + top_pad, H - margin - bot_pad

    allpts = [p for _, pls in parts_polys + thin_polys for pl in pls for p in pl]
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
    for _, pls in parts_polys:
        el.append(path(pls, 1.1))
    for _, pls in thin_polys:
        el.append(path(pls, 0.85))

    # ---- direct leader callouts (stacked columns) ----
    for side in ("L", "R"):
        grp = [c for c in callouts if c[3] == side]
        grp.sort(key=lambda c: -c[1])
        cur = dt + 4
        tx = dl - 12 if side == "L" else dr + 12
        anchor = "end" if side == "L" else "start"
        for gx, gy, lines, _ in grp:
            bh = len(lines) * 14.5 + 14
            ty = min(max(cur, Y(gy) - len(lines) * 7), db - bh)
            cur = ty + bh
            el.append('<circle cx="%.2f" cy="%.2f" r="2.1" fill="#111"/>' % (X(gx), Y(gy)))
            el.append('<path d="M %.2f %.2f L %.2f %.2f L %.2f %.2f" stroke="#111" '
                      'stroke-width="0.7" fill="none"/>'
                      % (X(gx), Y(gy), tx + (10 if side == "L" else -10), ty + 4, tx, ty + 4))
            for i, ln in enumerate(lines):
                el.append('<text x="%.2f" y="%.2f" font-size="12.5" font-weight="%s" '
                          'text-anchor="%s" fill="#111">%s</text>'
                          % (tx, ty + 4 + i * 14.5, "700" if i == 0 else "400", anchor, esc(ln)))

    # ---- numbered balloons (circle + item no, short leader) ----
    for gx, gy, num, ang in balloons:
        bx = X(gx) + 34 * math.cos(math.radians(ang))
        by = Y(gy) - 34 * math.sin(math.radians(ang))
        el.append('<path d="M %.2f %.2f L %.2f %.2f" stroke="#111" stroke-width="0.7" '
                  'fill="none"/>' % (X(gx), Y(gy), bx, by))
        el.append('<circle cx="%.2f" cy="%.2f" r="2.0" fill="#111"/>' % (X(gx), Y(gy)))
        el.append('<circle cx="%.2f" cy="%.2f" r="11" fill="#fff" stroke="#111" '
                  'stroke-width="1.1"/>' % (bx, by))
        el.append('<text x="%.2f" y="%.2f" font-size="12" font-weight="700" '
                  'text-anchor="middle" fill="#111">%s</text>' % (bx, by + 4.2, num))

    # ---- item table ----
    if item_table:
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

    el.append('<text x="%d" y="%d" font-size="22" font-weight="700" '
              'text-anchor="middle" fill="#111" font-style="italic">%s</text>'
              % (W / 2, H - margin - 22, esc(caption)))
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
def render(items, centerlines_3d, caption, out_name,
           callouts_raw=(), balloon_defs=(), item_table=()):
    """items: [(name, solid, thin?)].  callouts_raw: [(name, lines, side)].
    balloon_defs: [(name, number, angle_deg)] -> balloon at that part's centroid.
    item_table: [(number, qty, desc)]."""
    parts_polys, thin_polys, centroids = [], [], {}
    for name, solid, thin in items:
        pls = project(solid)
        if not pls:
            continue
        pts = [p for pl in pls for p in pl]
        centroids[name] = (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))
        (thin_polys if thin else parts_polys).append((name, pls))
    callouts = [(centroids[t][0], centroids[t][1], lines, side)
                for t, lines, side in callouts_raw if t in centroids]
    balloons = [(centroids[t][0], centroids[t][1], num, ang)
                for t, num, ang in balloon_defs if t in centroids]
    centerlines = [proj2d(*a) + proj2d(*b) for a, b in centerlines_3d]
    return emit(parts_polys, thin_polys, centerlines, callouts, balloons,
                list(item_table), caption, os.path.join(OUT, out_name))


# ============================================================ hardware helpers
def screws_on_circle(center, ax, n, gap, length=6, head_d=6, shank_d=3.2,
                     skip_rear=False, tag="s"):
    """n screws on the Ø14 bolt circle about `center`, axis `ax`, exploded `gap`
    along +axis. Circle positions in the plane ⊥ ax. skip_rear drops the -X
    (rear) position (yaw carrier cable channel)."""
    r = D.BCD / 2
    if ax == "y":
        pos4 = [(r, 0), (-r, 0), (0, r), (0, -r)]   # (x,z)
    elif ax == "x":
        pos4 = [(0, r), (0, -r), (r, 0), (-r, 0)]   # (y,z); rear = -x handled by skip
    else:
        pos4 = [(r, 0), (-r, 0), (0, r), (0, -r)]   # (x,y)
    out = []
    axis = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[ax]
    for i, a in enumerate(pos4):
        if skip_rear and i == 1:      # drop one position (cable channel)
            continue
        if ax == "y":
            off = (a[0], 0, a[1])
        elif ax == "x":
            off = (0, a[0], a[1])
        else:
            off = (a[0], a[1], 0)
        p = tuple(center[k] + off[k] + axis[k] * gap for k in range(3))
        out.append(("%s%d" % (tag, i),
                    Pos(*p) * _orient(screw(length, head_d, shank_d), ax), True))
    return out


# ==================================================================== JOINT
def joint():
    items = [("SERVO STS3215", CA.servo_mock_y(), False)]
    fork = Pos(90, 0, D.LINK_DROP) * parts.leg_link()
    items.append(("LEG_LINK  fork", fork, False))
    # horn disc + 4 horn screws + spline, exploded +Y
    items.append(("horn_disc", Pos(0, D.SV_HORN_FACE + 16, 0) * _orient(disc(), "y"), False))
    items += screws_on_circle((0, D.SV_HORN_FACE, 0), "y", 4, 34, length=6, tag="hs")
    items.append(("spline", Pos(0, D.SV_HORN_FACE + 52, 0) * _orient(spline(), "y"), True))
    # idler disc + 4 idler screws + 4 washers, exploded -Y  (PENDING)
    items.append(("idler_disc", Pos(0, D.SV_IDLER_FACE - 16, 0) * _orient(disc(), "y"), False))
    for i, (dx, dz) in enumerate(((D.BCD / 2, 0), (-D.BCD / 2, 0), (0, D.BCD / 2), (0, -D.BCD / 2))):
        items.append(("iw%d" % i, Pos(dx, D.SV_IDLER_FACE - 30, dz) * _orient(washer(), "y"), True))
        items.append(("is%d" % i, Pos(dx, D.SV_IDLER_FACE - 40, dz) * _orient(screw(8), "y"), True))
    # 6 case-grip self-tap screws on the case faces (into the adjacent link)
    for i, (yy, zz) in enumerate(((D.SV_TOPFACE, -8.3), (D.SV_TOPFACE, -29),
                                  (-D.SV_TOPFACE, -32.75))):
        for sx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            ax = "y"
            g = 22 if yy > 0 else -22
            items.append(("cg%d_%d" % (i, sx), Pos(sx, yy + g, zz)
                          * _orient(screw(8, shank_d=3.0), "y"), True))
    cl = [((0, -60, 0), (0, 90, 0)), ((-40, 0, 0), (128, 0, 0))]
    callouts = [
        ("SERVO STS3215", ["SERVO  STS3215", "case gripped by", "the adjacent link"], "L"),
        ("LEG_LINK  fork", ["LEG_LINK  (fork)", "rotates with the horn"], "R"),
        ("horn_disc", ["HORN DISC + 4X M3×6", "into the horn", "(servo's own kit)"], "R"),
        ("spline", ["SPLINE SCREW", "horn onto shaft (§1)"], "R"),
        ("is0", ["4X M3×8 + WASHER", "into the idler disc", "(same Ø14 pattern)"], "L"),
        ("idler_disc", ["IDLER DISC", "free-spinning; same", "Ø14 pattern"], "L"),
        ("cg0_10", ["6X M3×8 SELF-TAP", "case grip (into the", "adjacent link)"], "R"),
    ]
    return render(items, cl, "FIGURE J — TYPICAL LEG JOINT (KNEE)  ·  all hardware",
                  "exploded_joint.svg", callouts_raw=callouts)


# ==================================================================== HIP YAW
def hip_yaw():
    by = 0.0
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    items = [
        ("PELVIS", Pos(0, by, DECK + 78) * parts.pelvis(), False),
        ("YAW SERVO", Pos(0, by, D.HIP_YAW_Z + D.SV_HORN_FACE + 26) * CA.servo_mock_z(), False),
        ("YAW_CARRIER", Pos(0, by, D.HIP_YAW_Z - 46) * parts.yaw_carrier(), False),
        ("ROLL SERVO", Pos(0, by, D.HIP_ROLL_Z - 96) * CA.servo_mock_x(), False),
    ]
    # 4 yaw stator self-tap screws down through the deck (exploded up)
    for i, xr in enumerate(D.YAW_CASE_HOLES_IDLER):
        for s in (1, -1):
            items.append(("ys%d_%d" % (i, s), Pos(-xr, by + s * D.CASE_HOLE_LAT, DECK + 44)
                          * screw(8, shank_d=3.0), True))
    # yaw idler disc (up, against deck) + 3 carrier->horn screws (rear = cable channel)
    items.append(("yaw_idler", Pos(0, by, D.HIP_YAW_Z + D.SV_GRIP_SPAN + 8) * disc(), False))
    items += screws_on_circle((0, by, D.HIP_YAW_Z), "z", 3, -22, length=6,
                              skip_rear=True, tag="ch")
    items.append(("yaw_spline", Pos(0, by, D.HIP_YAW_Z - 34) * spline(), True))
    # 8 roll-bay self-tap screws (through carrier walls) exploded sideways
    for i in range(4):
        zz = D.CARRIER_ROLL_AXIS + (8.3 if i % 2 else 29) - 20
        for s in (1, -1):
            items.append(("rb%d_%d" % (i, s),
                          Pos(0, by + s * (D.CASE_HOLE_LAT + 24), D.HIP_YAW_Z + zz)
                          * _orient(screw(8, shank_d=3.0), "y"), True))
    cl = [((0, by, D.HIP_ROLL_Z - 120), (0, by, DECK + 92))]
    callouts = [
        ("PELVIS", ["PELVIS", "deck + 2 flat yaw seats"], "L"),
        ("ys0_1", ["4X M3×8 SELF-TAP", "down through deck ->", "case idler rows"], "L"),
        ("YAW SERVO", ["YAW SERVO  STS3215", "flat, horn DOWN"], "R"),
        ("yaw_idler", ["YAW IDLER DISC", "(against deck)"], "R"),
        ("ch0", ["3X M3×6 -> yaw horn", "rear position = the", "cable channel (open)"], "R"),
        ("YAW_CARRIER", ["YAW_CARRIER", "holds the roll bay"], "L"),
        ("rb0_1", ["8X M3×8 SELF-TAP", "roll servo into the", "carrier bay walls"], "R"),
        ("ROLL SERVO", ["ROLL SERVO  STS3215", "slides UP into carrier"], "L"),
    ]
    return render(items, cl, "FIGURE Y — HIP-YAW STACK (v3yaw)  ·  all hardware",
                  "exploded_hip_yaw.svg", callouts_raw=callouts)


# ==================================================================== FULL master
def full():
    import export_assembly as A
    items, balloons, table = [], [], []
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

    # printed parts + servo mocks (left leg + torso; right leg mirrors, drawn too)
    def collect(c, tag=""):
        for ch in c.children:
            if list(ch.children):
                collect(ch, ch.label.replace("leg_", "") + "_")
            else:
                items.append((tag + ch.label, Pos(0, 0, lift(ch.label)) * ch, False))
    collect(A.robot)

    # rubber sole pads under each foot
    for s, tag in ((1, "L"), (-1, "R")):
        items.append(("pad_%s" % tag, Pos(0, s * D.HIP_SEP / 2, lift("foot") - 40)
                      * pad(), True))

    # map a representative rendered part -> balloon number
    rep = {"pelvis": ("1", 90), "L_yaw_carrier": ("2", 90), "L_yoke_roll": ("3", 180),
           "L_yoke_pitch": ("4", 180), "L_link_thigh": ("5", 0),
           "L_foot": ("6", 180), "tower": ("7", 200),
           "gopro_base": ("8", 200), "imu_carrier": ("9", 20),
           "L_servo_knee": ("10", 0), "L_servo_hip_roll": ("11", 180),
           "pad_L": ("12", 200)}
    for name, (num, ang) in rep.items():
        balloons.append((name, num, ang))

    # hardware balloons (one per fastener group, on a representative glyph) + all glyphs
    def hw(name, solid, thin=True):
        items.append((name, solid, thin))

    # --- per-leg fastener glyphs at true joint heights (both legs) ---
    for s in (1, -1):
        y = s * D.HIP_SEP / 2
        # pitch-type joints: horn (M3×6) + idler (M3×8+washer PENDING) + spline
        for jz in (D.ANKLE_Z, D.KNEE_Z, D.HIP_PITCH_Z):
            for i, (dx, dz) in enumerate(((D.BCD / 2, 0), (-D.BCD / 2, 0),
                                          (0, D.BCD / 2), (0, -D.BCD / 2))):
                hw("h_%d_%d_%d" % (s, jz, i),
                   Pos(dx, y + D.SV_HORN_FACE + 30, jz + dz) * _orient(screw(6), "y"))
                hw("iw_%d_%d_%d" % (s, jz, i),
                   Pos(dx, y + D.SV_IDLER_FACE - 26, jz + dz) * _orient(washer(), "y"))
                hw("is_%d_%d_%d" % (s, jz, i),
                   Pos(dx, y + D.SV_IDLER_FACE - 34, jz + dz) * _orient(screw(8), "y"))
        # hip-roll joint (axis X): horn M3×6 + idler M3×10
        for i, (dy, dz) in enumerate(((D.BCD / 2, 0), (-D.BCD / 2, 0),
                                      (0, D.BCD / 2), (0, -D.BCD / 2))):
            hw("rh_%d_%d" % (s, i),
               Pos(D.SV_HORN_FACE + 30, y + dy, D.HIP_ROLL_Z + dz) * _orient(screw(6), "x"))
            hw("ri_%d_%d" % (s, i),
               Pos(-D.SV_HORN_FACE - 34, y + dy, D.HIP_ROLL_Z + dz) * _orient(screw(10), "x"))
        # hip-yaw: 3 carrier->horn (rear = cable channel) + spline
        for it in screws_on_circle((0, y, D.HIP_YAW_Z), "z", 3, -22, length=6,
                                   skip_rear=True, tag="chL%d" % s):
            items.append(it)
        # spline screws (5 leg servos/leg)
        for jz in (D.ANKLE_Z, D.KNEE_Z, D.HIP_PITCH_Z):
            hw("sp_%d_%d" % (s, jz), Pos(0, y + D.SV_HORN_FACE + 56, jz) * _orient(spline(), "y"))
        hw("sp_%d_roll" % s, Pos(D.SV_HORN_FACE + 56, y, D.HIP_ROLL_Z) * _orient(spline(), "x"))
        hw("sp_%d_yaw" % s, Pos(0, y, D.HIP_YAW_Z - 34) * spline())
        # yoke_pitch flange heat-sets (4) + M3×10 flange bolts (4)
        for i in range(4):
            dx = (i % 2 * 2 - 1) * 10; dyv = (i // 2 * 2 - 1) * 10
            hw("hset_%d_%d" % (s, i), Pos(dx, y + dyv, D.HIP_PITCH_Z + 44) * heatset())
            hw("fb_%d_%d" % (s, i), Pos(dx, y + dyv, D.HIP_PITCH_Z + 58) * screw(10))
        # foot / case-grip / yaw-stator self-tap glyphs (grouped, few reps)
        for i in range(4):
            hw("ft_%d_%d" % (s, i),
               Pos(-30 + i * 8, y + s * 24, D.ANKLE_Z - 10) * _orient(screw(8, shank_d=3.0), "y"))

    # torso hardware: 4 deck heat-sets + tower-feet M3×10, gopro M3×12, board+IMU M2.5, thumbscrew
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X):
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            hw("dh_%d_%d" % (sx, sy), Pos(sx, sy, DECK + 150 + 20) * heatset())
            hw("tf_%d_%d" % (sx, sy), Pos(sx, sy, DECK + 150 + 34) * screw(10))
    for sx in (11, -11):
        for sy in (8.5, -8.5):
            hw("gp_%d_%d" % (sx, sy), Pos(sx, sy, DECK + 380 + 30) * screw(12, shank_d=3.0))
    hw("m5", Pos(0, 0, DECK + 470 + 40) * thumbscrew(), True)

    # hardware balloons: one representative glyph per group (subgroups sum to §0)
    hw_rep = [("h_1_%d_0" % D.KNEE_Z, "13", 0), ("is_1_%d_0" % D.KNEE_Z, "14", 200),
              ("iw_1_%d_0" % D.KNEE_Z, "15", 160), ("rh_1_0", "16", 0),
              ("ri_1_0", "17", 200), ("ft_1_0", "18", 200), ("hset_1_0", "19", 30),
              ("fb_1_0", "20", 60), ("tf_14_42", "21", 0), ("gp_11_8.5", "22", 0),
              ("m5", "23", 0), ("chL12", "24", 30), ("sp_1_yaw", "25", 210),
              ("imu_carrier", "26", 340)]
    for nm, num, ang in hw_rep:
        balloons.append((nm, num, ang))

    # ITEM table — subgroups sum to the §0 totals with no double-counting:
    #   M3×6 (38) = 24 + 8 + 6   ·   M3×10 (20) = 8 + 8 + 4
    table = [
        ("1", "1", "pelvis"), ("2", "2", "yaw_carrier"), ("3", "2", "yoke_roll"),
        ("4", "2", "yoke_pitch"), ("5", "4", "leg_link (thigh+shin)"),
        ("6", "2", "foot"), ("7", "1", "tower"), ("8", "1", "gopro_base"),
        ("9", "1", "imu_carrier"), ("10", "10", "STS3215 servo"),
        ("11", "20", "horn + idler disc (incl. w/ servos)"),
        ("12", "2", "rubber sole pad"),
        ("13", "24", "M3×6 — pitch/knee/ankle horns"),
        ("14", "24", "M3×8 — idler-disc screws"),
        ("15", "24", "M3 thin washer (idler)"),
        ("16", "8", "M3×6 — hip-roll horns"),
        ("17", "8", "M3×10 — hip-roll idler (long boss)"),
        ("18", "56", "M3×8 self-tap — case grips + stators"),
        ("19", "12", "M3 heat-set insert (Ø4.6)"),
        ("20", "8", "M3×10 — yoke flange -> inserts"),
        ("21", "4", "M3×10 — tower feet -> deck inserts"),
        ("22", "4", "M3×12 self-tap — gopro/imu stack"),
        ("23", "1", "M5×20 thumbscrew — camera"),
        ("24", "6", "M3×6 — yaw carriers (3× ea, rear=cable)"),
        ("25", "10", "servo spline/center screw"),
        ("26", "8", "M2.5×8 self-tap — board + IMU"),
    ]
    cl = [((0, 0, lift("foot") - 60), (0, 0, DECK + 470 + 70))]
    return render(items, cl, "FIGURE 1 — BIMO BIPED v3yaw  ·  COMPLETE MECHANICAL ASSY",
                  "exploded_full.svg", balloon_defs=balloons, item_table=table)


DRAWINGS = {"joint": joint, "hip_yaw": hip_yaw, "full": full}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=list(DRAWINGS) + ["all"])
    a = ap.parse_args()
    for n in (list(DRAWINGS) if a.which == "all" else [a.which]):
        svg, png = DRAWINGS[n]()
        print("wrote %s  +  %s" % (svg, png))
