"""Classic isometric exploded-assembly line drawings from the real CAD.

Hidden-line-removed isometric projection (pythonOCC `HLRBRep_PolyAlgo`) of each
printed part + servo mock + fastener glyph, exploded along real insertion axes,
with dash-dot centerlines, screw glyphs on their own axes, leader callouts and a
figure caption -- the engineering-drawing style of a shop exploded view.

Run:
  .venv/bin/python cad/render_exploded_drawing.py joint    -> docs/assembly/exploded_joint.svg (+png)
  .venv/bin/python cad/render_exploded_drawing.py hip_yaw  -> docs/assembly/exploded_hip_yaw.svg (+png)
  .venv/bin/python cad/render_exploded_drawing.py full     -> docs/assembly/exploded_full.svg (+png)
  .venv/bin/python cad/render_exploded_drawing.py all

Fastener specs/counts come from cad/dimensions.py + the reconciled table in
docs/assembly.md. NOTE (2026-07-23): the printed fork/yoke idler arms currently
have NO through-holes on their outer face (only blind reliefs on the servo-
facing side) -- see check in the report -- so idler-side screws are drawn as a
FLAGGED callout, not as installed bolts, until parts.py is reconciled.
"""
import argparse
import os
import subprocess
import shutil

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

# ---- isometric projector (true iso: view dir (1,1,1), sheet-x along (1,-1,0)) ----
_AX = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 1, 1), gp_Dir(1, -1, 0))


def project(solid):
    """HLR-projected VISIBLE edges of a build123d solid -> list of 2D polylines."""
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
        edge = TopoDS.Edge_s(exp.Current())
        c = BRepAdaptor_Curve(edge)
        try:
            gc = GCPnts_QuasiUniformDeflection(c, 0.15)
            pts = [c.Value(gc.Parameter(i)) for i in range(1, gc.NbPoints() + 1)]
        except Exception:
            pts = [c.Value(c.FirstParameter()), c.Value(c.LastParameter())]
        polys.append([(p.X(), p.Y()) for p in pts])
        exp.Next()
    return polys


# direct isometric point projection matching _AX (verified against HLR coords):
#   Vx=(1,-1,0)/√2 , Vy=N×Vx=(1,1,-2)/√6  ->  2D=(P·Vx, P·Vy)
_R2, _R6 = 2 ** 0.5, 6 ** 0.5


def proj2d(x, y, z):
    return ((x - y) / _R2, (x + y - 2 * z) / _R6)


# --------------------------------------------------------------- fastener glyph
def screw(length=8.0, head_d=6.0, shank_d=3.2, axis="z", explode=0.0):
    """A button-head screw glyph solid, head at the top of the shank, pointing
    down -Z, then rotated so the shank runs along `axis`. `explode` shifts it
    back along its own axis (for the exploded gap)."""
    s = Cylinder(head_d / 2, 2.2) + Pos(0, 0, -length / 2 - 1.1) * Cylinder(shank_d / 2, length)
    if axis == "y":
        s = Rot(-90, 0, 0) * s
    elif axis == "x":
        s = Rot(0, 90, 0) * s
    return s


# ------------------------------------------------------------------- SVG sheet
def emit(parts_polys, screws_polys, centerlines, callouts, caption, out_svg,
         sheet=(1240, 860)):
    """parts_polys / screws_polys: [(name, [polylines])] in MODEL 2D coords.
    centerlines: [(x0,y0,x1,y1)] model 2D.
    callouts: [(gx, gy, [lines], side)] -- leader target in model 2D, text stacked
    in a fixed sheet-space column on `side` ('L'/'R')."""
    W, H = sheet
    margin = 26
    COL = 190                       # reserved callout-column width each side
    top_pad, bot_pad = 44, 78       # room for title band / caption
    dl, dr = margin + COL, W - margin - COL
    dt, db = margin + top_pad, H - margin - bot_pad

    allpts = [p for _, pls in parts_polys + screws_polys for pl in pls for p in pl]
    xs = [p[0] for p in allpts]; ys = [p[1] for p in allpts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    s = min((dr - dl) / (maxx - minx + 1e-6), (db - dt) / (maxy - miny + 1e-6))
    ox = dl + ((dr - dl) - (maxx - minx) * s) / 2
    oy = dt + ((db - dt) - (maxy - miny) * s) / 2

    def X(x):
        return ox + (x - minx) * s

    def Y(y):
        return oy + (maxy - y) * s      # flip Y (model up -> sheet up)

    def path(pls, w, color="#111", dash=None):
        d = " ".join("M " + " L ".join("%.2f %.2f" % (X(x), Y(y)) for x, y in pl)
                     for pl in pls if len(pl) > 1)
        da = ' stroke-dasharray="%s"' % dash if dash else ""
        return '<path d="%s" fill="none" stroke="%s" stroke-width="%.2f"%s ' \
               'stroke-linecap="round" stroke-linejoin="round"/>' % (d, color, w, da)

    el = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
          'font-family="Helvetica,Arial,sans-serif">' % (W, H),
          '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H),
          '<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#111" '
          'stroke-width="1.5"/>' % (margin, margin, W - 2 * margin, H - 2 * margin)]

    for x0, y0, x1, y1 in centerlines:       # dash-dot centerlines
        el.append('<path d="M %.2f %.2f L %.2f %.2f" stroke="#111" stroke-width="0.7" '
                  'stroke-dasharray="12 4 2 4" fill="none"/>'
                  % (X(x0), Y(y0), X(x1), Y(y1)))
    for _, pls in parts_polys:
        el.append(path(pls, 1.15))
    for _, pls in screws_polys:
        el.append(path(pls, 0.9))

    # ---- callout columns: stack text blocks top-to-bottom, leaders to targets ----
    def esc(t):
        return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    for side in ("L", "R"):
        grp = [c for c in callouts if c[3] == side]
        grp.sort(key=lambda c: -c[1])         # by target y, top first
        cur = dt + 6
        tx = dl - 12 if side == "L" else dr + 12
        anchor = "end" if side == "L" else "start"
        for gx, gy, lines, _ in grp:
            block_h = len(lines) * 15 + 16
            ty = max(cur, Y(gy) - len(lines) * 7)
            ty = min(ty, db - block_h)
            cur = ty + block_h
            el.append('<circle cx="%.2f" cy="%.2f" r="2.1" fill="#111"/>'
                      % (X(gx), Y(gy)))
            el.append('<path d="M %.2f %.2f L %.2f %.2f L %.2f %.2f" stroke="#111" '
                      'stroke-width="0.7" fill="none"/>'
                      % (X(gx), Y(gy), tx + (10 if side == "L" else -10), ty + 4, tx, ty + 4))
            for i, ln in enumerate(lines):
                el.append('<text x="%.2f" y="%.2f" font-size="12.5" font-weight="%s" '
                          'text-anchor="%s" fill="#111">%s</text>'
                          % (tx, ty + 4 + i * 15, "700" if i == 0 else "400",
                             anchor, esc(ln)))

    el.append('<text x="%d" y="%d" font-size="22" font-weight="700" '
              'text-anchor="middle" fill="#111" font-style="italic">%s</text>'
              % (W / 2, H - margin - 24, esc(caption)))
    el.append("</svg>")
    with open(out_svg, "w") as f:
        f.write("\n".join(el))
    # PNG via qlmanage (macOS) if available
    png = out_svg[:-4] + ".png"
    if shutil.which("qlmanage"):
        subprocess.run(["qlmanage", "-t", "-s", str(W), "-o", "/tmp",
                        out_svg], capture_output=True)
        tmp = os.path.join("/tmp", os.path.basename(out_svg) + ".png")
        if os.path.exists(tmp):
            shutil.move(tmp, png)
    return out_svg, png


# ============================================================ assemblies
def build(items, callouts_raw, centerlines_3d, caption, out_name):
    """items: [(name, solid_at_exploded_position, is_screw)].
    callouts_raw: [(target_name, text_lines, side)] -> leader to the part's projected
    centroid, text stacked in the `side` column.
    centerlines_3d: [((x0,y0,z0),(x1,y1,z1)), ...] projected to model 2D."""
    parts_polys, screws_polys = [], []
    centroids = {}
    for name, solid, is_screw in items:
        pls = project(solid)
        if not pls:
            continue
        pts = [p for pl in pls for p in pl]
        centroids[name] = (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))
        (screws_polys if is_screw else parts_polys).append((name, pls))
    callouts = []
    for target, lines, side in callouts_raw:
        if target not in centroids:
            continue
        gx, gy = centroids[target]
        callouts.append((gx, gy, lines, side))
    centerlines = [proj2d(*a) + proj2d(*b) for a, b in centerlines_3d]
    return emit(parts_polys, screws_polys, centerlines, callouts, caption,
                os.path.join(OUT, out_name))


# a small "screw column" helper: n screws strung along an axis at a point
def screw_column(base, axis_vec, gaps, **kw):
    """Return [(name, solid, True)...] screws stacked along axis from `base`."""
    out = []
    for i, g in enumerate(gaps):
        p = tuple(base[k] + axis_vec[k] * g for k in range(3))
        ax = "x" if abs(axis_vec[0]) > 0.5 else ("y" if abs(axis_vec[1]) > 0.5 else "z")
        out.append(("screw%d" % i, Pos(*p) * screw(axis=ax, **kw), True))
    return out


# ------------------------------------------------------------------ JOINT (knee)
def joint():
    Y = D.SV_HORN_FACE
    items = []
    # servo body (output axis +Y), horn at +Y, idler recess -Y
    items.append(("SERVO STS3215", CA.servo_mock_y(), False))
    # distal link = the fork (leg_link lower fork at origin) exploded +X (its insertion)
    fork = Pos(85, 0, D.LINK_DROP) * parts.leg_link()
    items.append(("LEG_LINK  fork -> horn + idler", fork, False))
    # 4 horn screws on the Ø14 circle, exploded +Y along the bolt axis
    r = D.BCD / 2
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        items.append(("hs", Pos(dx, Y + 26, dz) * screw(length=6, axis="y"), True))
    # idler-side: the fork's locator boss (Ø19->16 taper) shown as a stub, NO screws
    items.append(("idler_boss", Pos(0, D.SV_IDLER_FACE - 10, 0)
                  * Rot(90, 0, 0) * Cone(8.0, 9.5, 3.0), False))
    cl = [((0, -30, 0), (0, 60, 0)),        # Y = output / horn-bolt axis
          ((-32, 0, 0), (118, 0, 0))]       # X = fork insertion axis
    callouts = [
        ("SERVO STS3215", ["SERVO  STS3215", "case is gripped by", "the adjacent link"], "L"),
        ("LEG_LINK  fork -> horn + idler",
         ["LEG_LINK  (fork)", "distal link — rotates", "with the horn"], "R"),
        ("hs", ["4X  M3×6  BTN-HD", "INTO SERVO HORN", "(servo's own kit)"], "R"),
        ("idler_boss", ["IDLER SIDE = LOCATOR BOSS", "no through-holes in the",
                        "printed fork — idler", "screws cannot be fitted", "as modeled (see note)"], "L"),
    ]
    return build(items, callouts, cl, "FIGURE J — TYPICAL LEG JOINT (KNEE)",
                 "exploded_joint.svg")


# ------------------------------------------------------------------ HIP YAW stack
def hip_yaw():
    by = 0.0
    items = []
    DECK = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW
    # explode the v3yaw stack vertically along the leg centre
    # pelvis (deck) at top
    items.append(("PELVIS  (deck + yaw seats)", Pos(0, by, DECK + 70) * parts.pelvis(), False))
    # yaw servo (flat, horn down) just below the deck
    items.append(("YAW SERVO  STS3215",
                  Pos(0, by, D.HIP_YAW_Z + D.SV_HORN_FACE + 20) * CA.servo_mock_z(), False))
    # yaw carrier below the yaw servo
    items.append(("YAW_CARRIER", Pos(0, by, D.HIP_YAW_Z - 40) * parts.yaw_carrier(), False))
    # roll servo below the carrier
    items.append(("ROLL SERVO  STS3215",
                  Pos(0, by, D.HIP_ROLL_Z - 90) * CA.servo_mock_x(), False))
    # yaw stator screws: 4 down through the deck, exploded up
    for xr in D.YAW_CASE_HOLES_IDLER:
        for s in (1, -1):
            items.append(("ys", Pos(-xr, by + s * D.CASE_HOLE_LAT, DECK + 40)
                          * screw(length=8, axis="z"), True))
    # carrier->horn screws (3) between yaw servo and carrier -- the rear (-X)
    # position is deliberately open: it is the carrier cable channel
    # (wiring audit 2026-07-23)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (0, r), (0, -r)):
        items.append(("ch", Pos(dx, by + dy, D.HIP_YAW_Z - 12) * screw(length=6, axis="z"), True))
    cl = [((0, by, D.HIP_ROLL_Z - 110), (0, by, DECK + 85))]   # leg-centre vertical axis
    callouts = [
        ("PELVIS  (deck + yaw seats)", ["PELVIS", "deck + 2 flat yaw seats"], "L"),
        ("YAW SERVO  STS3215", ["YAW SERVO  STS3215", "flat, horn DOWN"], "R"),
        ("ys", ["4X  M3×8 SELF-TAP", "down through deck", "into case idler rows", "(8.3 & 32.75)"], "L"),
        ("YAW_CARRIER", ["YAW_CARRIER", "holds the roll bay"], "R"),
        ("ch", ["3X  M3×6  BTN-HD", "carrier -> yaw horn", "(rear position open =", "cable channel)"], "R"),
        ("ROLL SERVO  STS3215", ["ROLL SERVO  STS3215", "slides UP into carrier;",
                                 "8X M3×8 self-tap", "through bay walls"], "L"),
    ]
    return build(items, callouts, cl, "FIGURE Y — HIP-YAW STACK (v3yaw)",
                 "exploded_hip_yaw.svg")


# ------------------------------------------------------------------ FULL robot
def full():
    import export_assembly as A
    items = []
    # explode each leaf of the neutral robot along a mostly-vertical axis, spread out
    LIFT = {"pelvis": 120, "tower": 220, "battery": 210, "imu": 300, "gopro": 340,
            "camera": 400, "servo_hip_yaw": 70, "yaw_carrier": 20,
            "servo_hip_roll": -30, "yoke_roll": -80, "yoke_pitch": -130,
            "link_thigh": -150, "servo_hip_pitch": -150, "servo_knee": -300,
            "link_shin": -320, "servo_ankle": -470, "foot": -500}

    def lift_of(label):
        for k, v in LIFT.items():
            if label.startswith(k):
                return v
        return 0

    def collect(c, tag=""):
        for ch in c.children:
            if list(ch.children):
                collect(ch, ch.label.replace("leg_", "") + "_")
            else:
                dz = lift_of(ch.label)
                items.append((tag + ch.label, Pos(0, 0, dz) * ch, False))
    collect(A.robot)
    # one callout per unique part (left leg + torso parts), split L/R columns by height
    callouts = []
    seen = set()
    uniq = []
    for name, solid, _ in items:
        base = name[2:] if name[:2] in ("L_", "R_") else name
        if base in seen or name.startswith("R_"):
            continue
        seen.add(base)
        uniq.append((name, base))
    for i, (name, base) in enumerate(uniq):
        side = "L" if i % 2 == 0 else "R"
        callouts.append((name, [base.replace("_", " ").upper()], side))
    cl = [((0, 0, -520), (0, 0, D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW + 260))]
    return build(items, callouts, cl, "FIGURE 1 — BIMO BIPED v3yaw MASTER ASSY",
                 "exploded_full.svg")


DRAWINGS = {"joint": joint, "hip_yaw": hip_yaw, "full": full}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=list(DRAWINGS) + ["all"])
    a = ap.parse_args()
    names = list(DRAWINGS) if a.which == "all" else [a.which]
    for n in names:
        svg, png = DRAWINGS[n]()
        print("wrote %s  +  %s" % (svg, png))
