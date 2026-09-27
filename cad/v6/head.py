"""head: the STEREO-PERISCOPE camera head on its own neck servo (v6 body).

ONE Camera Module 3 (Wide) sees in stereo through two horizontal periscopes --
binocular vision from a single sensor, both eye openings in one horizontal
plane. The optics are designed, and every optical number here is derived, in
periscope_optics.py; the design note (trade studies, parts, assembly) is
docs/design-v6/stereo-head.md.

Seen from above (+X forward, the neck yaw axis at the origin):

         eye window L     | nose |     eye window R        <- the FACE
      ____________________|  CM3 |____________________
      \\  outer mirror L   \\  |  /   outer mirror R  //
       \\                    \\|/                    //     the chevron walls ARE
        \\                  V of 2 squares          //      the mirror mounts
         \\____________________________________//          <- back wall

  * PRISM-FREE (2026-09-26, Tom: "don't use a prism then"). The camera looks
    AFT (-X) into a V of two small FRONT-SURFACE squares glued into a printed
    block, its apex just behind the lens. The V splits the sensor's long axis;
    each half-image goes sideways to an outer front-surface square that turns
    it forward out of that side's eye window. Two reflections per eye: rotated,
    never mirrored.
  * The two V squares can't both reach the apex (their glass would collide):
    the LEFT one runs to it, the RIGHT one butts against its back one glass-
    thickness out. That blanks a strip at the right eye's OUTER edge; the
    stereo overlap, at the inner edges, is untouched.
  * The mirrors are cheap and undocumented (Tom's call for this part): their
    size and thickness are periscope_optics.V_TILE / OUTER_TILE, MEASURED ON
    ARRIVAL; the stereo calibration absorbs the rest.
  * The V sits BEHIND the horn's screw circle and the camera rides a removable
    sled in the nose above it: lift the lid, pull the sled, and the four M3
    horn screws are driven straight down.
  * The chevron walls lean a few degrees: that is the optics, not slop -- a
    vertical mirror loses ~9 deg per eye (the lens's field edge at each eye's
    inner edge only reaches ~27 deg down; the tilt keeps the full -30).

Parts (all print base-down; the sled prints lying on its front face):
  head_shell()   base plate on the horn + the chevron body -- both mirror
                 walls, back wall, face frame with the two eye windows, the
                 nose, and the V block. Open on top.
  head_lid()     the roof; 4x M2.5 flat-head self-tappers into shell bosses.
                 It also holds the camera sled down.
  camera_sled()  the plate the Camera Module 3 screws to (4x M2 self-tap);
                 slides down grooves in the nose.
  head()         all three fused -- the assembly, the ROM checks, the mass.
Mocks (not printed): v_tiles_mock(), mirror_mock(side), camera_mock(), and
view_cones() -- the eyes' fields, for renders and the self-occlusion check.

Every beam -- pupil to V, V to mirror, mirror out through the window -- is
built as a solid from the traced design (with margin) and subtracted from
every printed part, so nothing printed can sit in the light path;
check_optics() re-proves it on the finished parts.

Glue the glass with 2-part epoxy or neutral-cure silicone, NEVER cyanoacrylate:
its vapour frosts first-surface coatings. Print in black PETG -- the inside of
the head is an optical cavity, and light-coloured walls scatter.

Run:
    .venv/bin/python cad/v6/head.py     # STL + STEP + checks + render
"""
from __future__ import annotations

import math
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import *  # noqa: E402,F403
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402
import periscope_optics as PO  # noqa: E402

box, cyl_x, cyl_y, cyl_z = P.box, P.cyl_x, P.cyl_y, P.cyl_z

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientation: the shell and lid lie base-down as modelled; the sled
# lies on its front (+X) face, bosses up.
PRINT_ORIENT = np.eye(3)
PRINT_ORIENT_SLED = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])   # +X -> -Z

# ---------------------------------------------------------------- the optics
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    OPT = PO.design()            # left/right eye fields, total, overlap, baseline, vp, cfg
CFG = OPT["cfg"]
EYE_L, EYE_R = OPT["left"], OPT["right"]

# the camera's entrance pupil, head-local. XP puts the V's apex just aft
# of the horn screws' driver cylinders (see check_horn_access). ZP is set by
# the eyes' lowest rays, not by the mirrors: at 40.5 (mirrors a rim above
# the base) the -30 deg cones sliced the socket floor into knife-edge
# slivers out at the face; at 48.5 they clear the whole floor up to the
# face, so the floor stays whole -- and stiff, which the stereo calibration
# needs more than it needs the 8 mm. (face_windows() asserts it.)
XP, ZP = -6.5, 48.5
PUPIL = np.array([XP, 0.0, ZP])


def H(p_rel):
    """Optics frame (about the pupil, same axes) -> head-local."""
    return np.asarray(p_rel, float) + PUPIL


def _v(p):
    return Vector(float(p[0]), float(p[1]), float(p[2]))


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


FLIP_Y = np.diag([1.0, -1.0, 1.0])

# the V: two front-surface squares (PO.V_TILE), apex line vertical. The LEFT
# eye's square (+Y leg) runs to the apex; the RIGHT eye's butts against its
# back, one glass-thickness out along its own leg.
VT_L, VT_H, VT_T = PO.V_TILE                           # along the leg, height, glass
LEG = CFG.leg                                          # +Y leg: n out of the face, e2 out along the leg
APEX = H(CFG.apex)                                     # apex-line centre = the squares' mid-height
X_APEX = APEX[0]
V_UP = LEG.e1 if LEG.e1[2] > 0 else -LEG.e1           # up the apex line
V_BACK = 2.2                                           # backing plate behind each square
V_RIM = 2.0                                            # rim past each square's far end
GLASS_RHO = 2.5e-3                                     # g/mm^3, soda-lime float glass
VT_MASS = VT_L * VT_H * VT_T * GLASS_RHO               # each
V_AL = {+1: (0.0, VT_L), -1: (VT_T, VT_T + VT_L)}      # each square's span along its leg
V_UPR = (-VT_H / 2, VT_H / 2)
V_FRAME = (APEX, LEG.e2, V_UP, LEG.n)                  # built on +Y; the right square is mirrored

# outer mirrors: front-surface squares (PO.OUTER_TILE)
MIR_C = H(CFG.outer.c)                                 # reflective-face centre, +Y side
MIR_N = CFG.outer.n                                    # out of the reflective face
MIR_E1 = CFG.outer.e1                                  # along its 50 mm length (horizontal)
MIR_E2 = CFG.outer.e2                                  # along its 50 mm height (~vertical)
MIR_L, MIR_H, MIR_T = CFG.mirror_L, CFG.mirror_H, CFG.mirror_t
assert PO.OUTER_FRONT, "back-silvered outer squares need the pocket flipped (glass in front)"
MIRROR_MASS = MIR_L * MIR_H * MIR_T * GLASS_RHO        # g each

# ---------------------------------------------------------------- the body
BCD_R = D.BCD / 2
BASE_T = V.HEAD_BASE_T                                 # 4.0 horn plate
WALL = 2.2                                             # back, sides, face
LID_T = 1.8
POCKET_D = 0.8                                         # glue pockets locate the glass
MIR_BACK = 2.0                                         # wall behind the glass
WALL_OUT = -(MIR_T + MIR_BACK)                         # chevron outer surface, along n
WALL_IN = -(MIR_T - POCKET_D)                          # its inner surface
MIR_RIM = 2.0                                          # wall past each glass edge
MIR_WINDOW = 10.0                                      # glass bears on a 5 mm frame; the
                                                       # wall behind it is open (weight)


def _mirror_corners(side=+1, n_off=0.0):
    """The 4 corners of the +Y (side=+1) or -Y glass, on the plane n_off
    along n from its reflective face."""
    pts = []
    for a in (-1, 1):
        for b in (-1, 1):
            p = MIR_C + MIR_N * n_off + MIR_E1 * a * MIR_L / 2 + MIR_E2 * b * MIR_H / 2
            pts.append(p if side > 0 else FLIP_Y @ p)
    return np.array(pts)


_glass = np.vstack([_mirror_corners(1, 0.0), _mirror_corners(1, -MIR_T)])
Z_ROOF = float(np.ceil((_glass[:, 2].max() + 1.3) * 10) / 10)     # lid underside
ZT = Z_ROOF + LID_T                                                # head top
assert _glass[:, 2].min() - 0.2 > BASE_T + 1.0, "mirror must clear the base plate"

# the chevron's two ends, on its OUTER surface, at the mirror's mid-height
_aft = MIR_C + MIR_N * WALL_OUT - MIR_E1 * (MIR_L / 2 + MIR_RIM)
_fwd = MIR_C + MIR_N * WALL_OUT + MIR_E1 * (MIR_L / 2 + MIR_RIM)
_v_aft = min((APEX + LEG.e2 * (V_AL[-1][1] + V_RIM) - LEG.n * (VT_T + V_BACK))[0], _aft[0] + 0.0)
X_BACK = float(np.floor(min(_aft[0], _v_aft - WALL - 1.0) * 10) / 10)   # back wall's outer face
# the side wall clears both the outer mirror's forward end AND the eye's field
# where it leaves through the face (the cone's outer plane, margin + pencil)
_vp = H(CFG.virtual_pupil())
_cone_y = _vp[1] + (XP - PO.NOSE_C[1] + 0.6 - _vp[0]) * math.tan(math.radians(EYE_L["az_hi"] + 1.0)) + 0.8
Y_OUT = float(np.ceil(max(_fwd[1], _cone_y + WALL + 0.8) * 10) / 10)    # outer side's outer face

# the face and the nose -- the numbers the optics trace assumed (PO.NOSE_*)
X_FACE0 = XP - PO.PUPIL_BELOW_FRONT                    # face aft surface == lens front
X_FACE1 = XP - PO.NOSE_C[1]                            # face front == nose's aft end
XN_FRONT = XP - PO.NOSE_C[0]                           # nose front
NOSE_HW = PO.NOSE_HW                                   # 15.5
NOSE_FRONT_T = 1.6
NOSE_IN = PO.CAM_BOARD[0] / 2 + 0.4                    # 12.9: board + 0.4 each side

# the camera and its sled (CM3 Wide: board 25 x 24 x 1, lens 11.4 proud)
CAM_BOARD_X = (XP + PO.CAM_LENS_H - PO.PUPIL_BELOW_FRONT,
               XP + PO.CAM_LENS_H - PO.PUPIL_BELOW_FRONT + PO.CAM_BOARD[2])  # 3.4..4.4
SLED_X = (CAM_BOARD_X[1] + PO.CAM_BOARD_BACK, CAM_BOARD_X[1] + PO.CAM_BOARD_BACK + 2.0)
SLED_HW = NOSE_IN + 0.8                                # edges run in 1 mm grooves
SLED_Z0 = ZP - V.CAM3_HOLES[1] / 2 - 4.0               # clears the FPC connector below
CAM_HOLES = tuple((sy * V.CAM3_HOLES[0] / 2, ZP + sz * V.CAM3_HOLES[1] / 2)
                  for sy in (1, -1) for sz in (1, -1))
assert SLED_X[1] <= XN_FRONT - NOSE_FRONT_T + 0.01, "sled must sit behind the nose front wall"

# ribbon slot: behind the screw circle, and beside the V's right leg -- the
# V's apex sits right over the old centred slot, which undercut its foot
CABLE_X = -(BCD_R + 2.0 + V.HEAD_CABLE_SLOT[1] / 2)    # -11
CABLE_Y = -(V.HEAD_CABLE_SLOT[0] / 2 + 3.0)            # -9: the slot spans y -15..-3

# lid screws: two against the back wall behind the V (no beam passes there),
# two in the nose side walls just ahead of the face
LID_SCREWS = ((X_BACK + WALL + 2.2, 7.0), (X_BACK + WALL + 2.2, -7.0),
              (X_FACE1 + 2.5, NOSE_HW - 2.6), (X_FACE1 + 2.5, -(NOSE_HW - 2.6)))


# ---------------------------------------------------------------- helpers
def _frame_box(center, x_dir, z_dir, size):
    """A Box of `size` = (along x_dir, along z_dir x x_dir, along z_dir),
    centred at `center`."""
    return Plane(origin=_v(center), x_dir=_v(_unit(x_dir)), z_dir=_v(_unit(z_dir))).location * Box(*size)


def _halfspace(point, normal, size=900.0):
    """The side n.(x - point) >= 0, as a big box."""
    n = _unit(normal)
    x = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.cross(n, [1.0, 0, 0])
    return _frame_box(np.asarray(point, float) + n * size / 2, x, n, (size, size, size))


def _mirror_y(solid):
    return mirror(solid, about=Plane.XZ)


def _pyramid(apex, dirs, d_far, pencil):
    """The solid pyramid of rays from `apex` along the 4 corner `dirs`
    (cyclic order), out to d_far along their mean direction, with every
    cross-section grown by `pencil` (the apex is pulled back so each face
    moves out by that much -- the 1.25 mm pupil's pencil, not a point)."""
    dirs = [_unit(d) for d in dirs]
    c = _unit(np.sum(dirs, axis=0))
    s_min = 1.0
    for i in range(4):
        nf = _unit(np.cross(dirs[i], dirs[(i + 1) % 4]))
        s_min = min(s_min, abs(np.dot(c, nf)))
    apex = np.asarray(apex, float) - c * (pencil / max(s_min, 1e-3))

    def ring(dist):
        return Wire.make_polygon([_v(apex + d * (dist / np.dot(d, c))) for d in dirs], close=True)
    return Solid.make_loft([ring(0.2), ring(d_far)], ruled=True)


# ---------------------------------------------------------------- the beams
def _corner_dirs(margin, eye=None):
    """The eye's field corners (left eye by default: the right eye's field is
    the left's minus the seam strip, so its mirrored beams contain it)."""
    eye = EYE_L if eye is None else eye
    a0, a1 = eye["az_lo"] - margin, eye["az_hi"] + margin
    e0, e1 = OPT["el_lo"] - margin, OPT["el_hi"] + margin
    return [PO.dir_az_el(a, e) for a, e in ((a0, e0), (a1, e0), (a1, e1), (a0, e1))]


def _beams_left(margin, pencil):
    """(view cone, lateral beam, input beam) of the +Y (left) eye, head-local.
    Planar mirrors map a pyramid of rays to a pyramid, so each leg of the
    path is exactly a clipped pyramid from that leg's (virtual) pupil."""
    R = CFG.R()
    n1 = CFG.leg.n
    M1 = np.eye(3) - 2 * np.outer(n1, n1)
    E = _corner_dirs(margin)
    cam = [R.T @ e for e in E]
    lat = [M1 @ d for d in cam]
    vp = H(CFG.virtual_pupil())
    vp1 = H(-2 * np.dot(-CFG.leg.c, n1) * n1)
    leg_c, out_c = H(CFG.leg.c), H(CFG.outer.c)
    cone = _pyramid(vp, E, 400.0, pencil) & _halfspace(out_c, CFG.outer.n)
    lateral = (_pyramid(vp1, lat, 200.0, pencil) & _halfspace(leg_c, n1)
               & _halfspace(out_c, CFG.outer.n))
    inp = _pyramid(PUPIL, cam, 60.0, pencil) & _halfspace(leg_c, n1)
    return cone, lateral, inp


_BEAM_CACHE = {}


def beams(margin=1.0, pencil=0.8):
    """All six beam solids (both eyes), fused. margin: deg added round the
    field window; pencil: mm added round every ray."""
    key = (margin, pencil)
    if key not in _BEAM_CACHE:
        parts = list(_beams_left(margin, pencil))
        parts += [_mirror_y(s) for s in parts]
        out = parts[0]
        for s in parts[1:]:
            out = out + s
        _BEAM_CACHE[key] = out
    return _BEAM_CACHE[key]


def face_windows():
    """Each eye window in the face, cut as a plain rectangle that contains
    the cone's cross-section through the face (with the same margin the
    beams are cut with), running up to the lid and out to the side wall.
    The cone itself crosses the 2.5 mm face obliquely; cut by the cone, the
    face's edges were sloped knife-edges and its top rail a 46 mm bridge.
    Cut square, the stiles are plain walls and the lid, printed flat,
    closes each window's top."""
    cone = _beams_left(1.0, 0.8)[0]
    bb = (cone & box(X_FACE0 - 0.6, X_FACE1 + 0.6, 0, 200, -50, 200)).bounding_box()
    y0, z0 = bb.min.Y - 0.3, bb.min.Z - 0.3
    assert z0 > BASE_T + 0.8, f"eye cone reaches the base plate at the face (z {z0:.2f})"
    assert bb.max.Y < Y_OUT - WALL, f"eye cone cuts the side wall at the face (y {bb.max.Y:.2f})"
    assert y0 > NOSE_HW + 2.0, f"inner stile beside the nose too thin (y {y0:.2f})"
    win = box(X_FACE0 - 0.5, X_FACE1 + 0.5, y0, Y_OUT - WALL, z0, Z_ROOF + 0.5)
    return win + _mirror_y(win)


def view_cones(reach=250.0):
    """The two eyes' fields (exact window, no margin), out to `reach` from
    the virtual pupils, clipped to in front of the mirrors -- for renders
    and the self-occlusion check."""
    vp = H(CFG.virtual_pupil())
    front = _halfspace(H(CFG.outer.c), CFG.outer.n)
    left = _pyramid(vp, _corner_dirs(0.0, EYE_L), reach, 0.0) & front
    right = _pyramid(vp, _corner_dirs(0.0, EYE_R), reach, 0.0) & front
    return left + _mirror_y(right)


# ---------------------------------------------------------------- body solids
def _main_body(extra=0.0):
    """The chevron block, outer surfaces, grown inward by `extra`."""
    b = box(X_BACK + extra, X_FACE1, -(Y_OUT - extra), Y_OUT - extra, 0, ZT)
    hs = _halfspace(MIR_C + MIR_N * (WALL_OUT + extra), MIR_N)
    hs_r = _halfspace(FLIP_Y @ (MIR_C + MIR_N * (WALL_OUT + extra)), FLIP_Y @ MIR_N)
    return b & hs & hs_r


def _interior(extra=0.0, z0=BASE_T, z1=None):
    """The cavity inside the chevron, shrunk by `extra` all round (in plan),
    spanning z0..z1."""
    z1 = Z_ROOF if z1 is None else z1
    b = box(X_BACK + WALL + extra, X_FACE0, -(Y_OUT - WALL - extra), Y_OUT - WALL - extra, z0, z1)
    hs = _halfspace(MIR_C + MIR_N * (WALL_IN + extra), MIR_N)
    hs_r = _halfspace(FLIP_Y @ (MIR_C + MIR_N * (WALL_IN + extra)), FLIP_Y @ MIR_N)
    return b & hs & hs_r


def _nose_outer():
    return box(X_FACE0, XN_FRONT, -NOSE_HW, NOSE_HW, 0, ZT)


def _nose_inner(extra=0.0, z0=BASE_T, z1=None):
    z1 = Z_ROOF if z1 is None else z1
    return box(X_FACE0 - 1.0, XN_FRONT - NOSE_FRONT_T - extra, -(NOSE_IN - extra), NOSE_IN - extra, z0, z1)


# ---------------------------------------------------------------- the glass
# Every mirror is a flat FRONT-SURFACE square glued into a recess: reflective
# face on the design plane, glass behind it. A tile frame is (a point on the
# face, along, up, n out of the face); ranges are (lo, hi) mm along those axes.
OUT_FRAME = (MIR_C, MIR_E1, MIR_E2, MIR_N)
OUT_AL, OUT_UP = (-MIR_L / 2, MIR_L / 2), (-MIR_H / 2, MIR_H / 2)


def _tile_box(fr, s_al, s_up, s_dep):
    c0, al, up, n = fr
    ctr = c0 + al * sum(s_al) / 2 + up * sum(s_up) / 2 + n * sum(s_dep) / 2
    return _frame_box(ctr, al, n, (s_al[1] - s_al[0], s_up[1] - s_up[0], s_dep[1] - s_dep[0]))


def _tile_prism(fr, pts_al_up, d0, d1):
    """A prism with its profile in the tile's (along, up) plane, spanning
    depths d0..d1 along n."""
    c0, al, up, n = fr
    pl = Plane(origin=_v(c0 + n * d0), x_dir=_v(al), z_dir=_v(n))
    ysgn = 1.0 if np.dot(np.cross(n, al), up) > 0 else -1.0
    pts = [(a, ysgn * u) for a, u in pts_al_up]
    area2 = sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(pts, pts[1:] + pts[:1]))
    if area2 < 0:
        pts = pts[::-1]                                  # CCW -> face normal +n
    out = extrude(pl * Polygon(*pts, align=None), amount=d1 - d0)
    bb = out.bounding_box()
    mid = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
    assert np.dot(n, mid - (c0 + n * d0)) >= -1e-6, "extruded the wrong way"
    return out


def _tile_glass(fr, s_al, s_up, t):
    return _tile_box(fr, s_al, s_up, (-t, 0.0))


POCKET_FRONT = 1.5      # the recess cutter reaches this far in front of the glass: enough to clear
                        # the pocket lip -- at 6 mm it shaved the V block's far end (outer
                        # mirror pocket) and the other V square's plate (at the apex)


def _tile_pocket(fr, s_al, s_up, t):
    """The glass's own volume (+0.15 all round) and just in front of it:
    floor exactly at the glass's back face."""
    return _tile_box(fr, (s_al[0] - 0.15, s_al[1] + 0.15), (s_up[0] - 0.15, s_up[1] + 0.15), (-t, POCKET_FRONT))


def _tile_window(fr, s_al, s_up, t, back, frame=5.0):
    """Opening in the plate behind the glass (it bears on a `frame` mm
    border). Peaked top, ~55 deg: printed base-down, a flat top would
    bridge, and exactly 45 sits on the audit's line when the plate is
    vertical (the V's are)."""
    w0, w1 = s_al[0] + frame, s_al[1] - frame
    u0, u1 = s_up[0] + frame, s_up[1] - frame
    if w1 - w0 < 4.0 or u1 - u0 < 4.0:
        return None
    half, mid = (w1 - w0) / 2, (w0 + w1) / 2
    shoulder = max(u0 + 0.5, u1 - half * 1.43)             # tan 55 deg
    pts = [(w0, u0), (w1, u0), (w1, shoulder), (mid, u1), (w0, shoulder)]
    return _tile_prism(fr, pts, -(t + back) - 1.0, -t + 0.1)


def _tile_top_chamfer(fr, s_al, s_up, d_floor, d_surface):
    """60-deg chamfer on the recess's top edge: the plate above would
    otherwise overhang the recess by its depth (d_surface - d_floor)."""
    c0, al, up, n = fr
    ht = s_up[1] + 0.15
    run = (d_surface + 0.3) - (d_floor - 0.05)
    mid = c0 + al * sum(s_al) / 2
    pl = Plane(origin=_v(mid), x_dir=_v(up), z_dir=_v(al))           # local y = al x up
    ysgn = 1.0 if np.dot(np.cross(al, up), n) > 0 else -1.0
    tri = [(ht - 0.05, ysgn * (d_floor - 0.05)), (ht - 0.05, ysgn * (d_surface + 0.3)),
           (ht - 0.05 + run * 1.75, ysgn * (d_surface + 0.3))]
    return extrude(pl * Polygon(*tri, align=None), amount=(s_al[1] - s_al[0]) / 2, both=True)


def _sides(solid_left, side):
    return solid_left if side > 0 else _mirror_y(solid_left)


def _mirror_pocket(side):
    return _sides(_tile_pocket(OUT_FRAME, OUT_AL, OUT_UP, MIR_T), side)


def _mirror_window(side):
    return _sides(_tile_window(OUT_FRAME, OUT_AL, OUT_UP, MIR_T, MIR_BACK), side)


def _pocket_top_chamfer(side):
    return _sides(_tile_top_chamfer(OUT_FRAME, OUT_AL, OUT_UP, -MIR_T, WALL_IN), side)


def _v_backing():
    """The V block: a backing plate behind each square, from the floor to just
    above the squares, the two meeting at the apex (an L-section, stiff on its
    own). Recesses, windows and chamfers are cut later."""
    b = None
    for side in (1, -1):
        slab = _sides(_tile_box(V_FRAME, (V_AL[side][0], V_AL[side][1] + V_RIM), (-200.0, VT_H / 2 + 3.0),
                                (-(VT_T + V_BACK), 0.0)), side)
        b = slab if b is None else b + slab
    return b


def _v_cuts():
    """Each square's recess, the window behind it, and its top chamfer."""
    cut = None
    for side in (1, -1):
        parts = [_tile_pocket(V_FRAME, V_AL[side], V_UPR, VT_T),
                 _tile_top_chamfer(V_FRAME, V_AL[side], V_UPR, -VT_T, 0.0),
                 _tile_window(V_FRAME, V_AL[side], V_UPR, VT_T, V_BACK)]
        for q in parts:
            if q is not None:
                q = _sides(q, side)
                cut = q if cut is None else cut + q
    return cut


def _body():
    """Shell + lid as one solid, before the lid is split off."""
    b = (_main_body() + _nose_outer()) - (_interior() + _nose_inner())
    b += _v_backing()
    for x, y in LID_SCREWS:                              # screw bosses, full height
        b += cyl_z(2.6, BASE_T - 0.1, Z_ROOF, x, y)
    b &= (_main_body() + _nose_outer())                  # bosses stay inside the skin
    return b


# ---------------------------------------------------------------- the parts
_PART_CACHE = {}


def _cached(fn):
    def wrap():
        if fn.__name__ not in _PART_CACHE:
            _PART_CACHE[fn.__name__] = fn()
        return _PART_CACHE[fn.__name__]
    wrap.__name__, wrap.__doc__ = fn.__name__, fn.__doc__
    return wrap


@_cached
def head_shell():
    """Base plate (bolted to the neck horn) + the chevron body, open on top."""
    p = _body() & box(-200, 200, -200, 200, -1, Z_ROOF)
    # glass pockets and the open windows behind the outer mirrors
    for side in (1, -1):
        p -= _mirror_pocket(side)
        p -= _pocket_top_chamfer(side)
        p -= _mirror_window(side)
    p -= _v_cuts()
    # sled grooves: 1 mm into each nose side wall, open at the top, a stop at SLED_Z0
    p -= box(SLED_X[0] - 0.15, SLED_X[1] + 0.15, -(SLED_HW + 0.15), SLED_HW + 0.15,
             SLED_Z0 - 0.2, Z_ROOF + 1.0)
    # every beam, with margin, and the two eye windows cut square
    p -= beams()
    p -= face_windows()
    # the base plate: horn screws on the O14 BCD, centre relief, ribbon slot,
    # and a lightening pocket from ABOVE (a pocket from below would print as
    # a 30 x 133 mm bridge) leaving a 1.3 mm floor, a boss round the horn,
    # a rim under every wall, and solid under the V's backing plates
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        p -= cyl_z(D.PAD_HOLE / 2, -1, BASE_T + 1, x, y)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -1, BASE_T + 1, 0, 0)
    p -= box(CABLE_X - V.HEAD_CABLE_SLOT[1] / 2, CABLE_X + V.HEAD_CABLE_SLOT[1] / 2,
             CABLE_Y - V.HEAD_CABLE_SLOT[0] / 2, CABLE_Y + V.HEAD_CABLE_SLOT[0] / 2, -1, BASE_T + 1)
    zp = 1.3
    pocket = _interior(extra=1.5, z0=zp, z1=BASE_T + 0.01) + _nose_inner(extra=1.5, z0=zp, z1=BASE_T + 0.01)
    pocket -= cyl_z(BCD_R + 5.0, -2, BASE_T + 1, 0, 0)
    for side in (1, -1):
        pocket -= _sides(_tile_box(V_FRAME, (V_AL[side][0] - 2.0, V_AL[side][1] + V_RIM + 2.0), (-200.0, 200.0),
                                   (-(VT_T + V_BACK) - 2.0, 2.0)), side)
    pocket -= box(CABLE_X - 4.0, CABLE_X + 4.0, CABLE_Y - 9.0, CABLE_Y + 9.0, -2, BASE_T + 1)
    for x, y in LID_SCREWS:
        pocket -= cyl_z(4.0, -2, BASE_T + 1, x, y)
    p -= pocket
    for x, y in LID_SCREWS:
        p -= cyl_z(D.M25_TAP / 2, BASE_T + 2.0, Z_ROOF + 1, x, y)
    return p


@_cached
def head_lid():
    """The roof over the whole body, screwed down at 4 points; flat top,
    countersunk for M2.5 flat heads."""
    p = _body() & box(-200, 200, -200, 200, Z_ROOF, ZT + 1)
    p -= beams()
    for x, y in LID_SCREWS:
        p -= cyl_z(D.CASE_SCREW_CLEAR / 2, Z_ROOF - 1, ZT + 1, x, y)
        p -= P.csk_z(x, y, ZT, +1)
    return p


@_cached
def camera_sled():
    """The plate the Camera Module 3 (Wide) screws to, lens aft: 4 M2
    self-tap bosses standing the board off by its back-side components.
    Slides down the nose grooves; the lid holds it."""
    p = box(SLED_X[0], SLED_X[1], -SLED_HW, SLED_HW, SLED_Z0, Z_ROOF - 0.3)
    for y, z in CAM_HOLES:
        p += cyl_x(2.2, CAM_BOARD_X[1], SLED_X[0] + 0.5, y, z)
        p -= cyl_x(0.85, CAM_BOARD_X[1] - 0.1, SLED_X[1] + 0.1, y, z)
    # pull hole near the top edge (a bent paperclip lifts the sled out)
    p -= cyl_x(1.6, SLED_X[0] - 0.1, SLED_X[1] + 0.1, 0, Z_ROOF - 4.0)
    return p


def head():
    """head_shell() + head_lid() + camera_sled(), fused -- the printed head
    for the assembly, the ROM checks and the mass rollup (glass separate:
    see GLASS_MASS)."""
    return head_shell() + head_lid() + camera_sled()


GLASS_MASS = 2 * VT_MASS + 2 * MIRROR_MASS


def check_dimension_copies(tol=0.06):
    """dimensions_v6 carries copies of this head's numbers for the modules
    that must not import the CAD (plant, inertia, assembly fallback). They
    must still match."""
    mc = MIR_C - MIR_N * MIR_T / 2
    want = {
        "HEAD_X": (X_BACK, XN_FRONT), "HEAD_W": 2 * Y_OUT, "HEAD_D": XN_FRONT - X_BACK,
        "HEAD_H": ZT - BASE_T, "CAM_Z_ABOVE_HORN": ZP - BASE_T,
        "HEAD_VTILE_C_L": tuple(_v_tile_centroid(1)), "HEAD_VTILE_C_R": tuple(_v_tile_centroid(-1)),
        "HEAD_CAM_C": (-1.8, 0.0, ZP + sum(PO.CAM_LENS_A) / 2),
        "HEAD_MIRROR_C": tuple(mc),
        "HEAD_MIRROR_YAW": math.degrees(math.atan2(MIR_N[1], MIR_N[0])),
        "HEAD_PUPIL": tuple(PUPIL), "HEAD_EYE_VP": tuple(H(CFG.virtual_pupil())),
        "HEAD_VTILE_MASS": VT_MASS, "HEAD_MIRROR_MASS": MIRROR_MASS,
    }
    bad = []
    for k, v in want.items():
        got = np.atleast_1d(np.asarray(getattr(V, k), float))
        if np.max(np.abs(got - np.atleast_1d(np.asarray(v, float)))) > tol:
            bad.append(f"{k}: dimensions_v6 {tuple(np.round(got, 2))} vs CAD {tuple(np.round(np.atleast_1d(v), 2))}")
    for b in bad:
        print("  FAIL  " + b)
    if not bad:
        print(f"  PASS  dimensions_v6 head copies match the CAD ({len(want)} values)")
    return not bad


# ---------------------------------------------------------------- mocks
def _v_tile_centroid(side):
    c = APEX + LEG.e2 * sum(V_AL[side]) / 2 - LEG.n * VT_T / 2
    return c if side > 0 else FLIP_Y @ c


def v_tiles_mock():
    return _tile_glass(V_FRAME, V_AL[1], V_UPR, VT_T) + _mirror_y(_tile_glass(V_FRAME, V_AL[-1], V_UPR, VT_T))


def mirror_mock(side=+1):
    return _sides(_tile_glass(OUT_FRAME, OUT_AL, OUT_UP, MIR_T), side)


def camera_mock():
    board = box(CAM_BOARD_X[0], CAM_BOARD_X[1], -PO.CAM_BOARD[0] / 2, PO.CAM_BOARD[0] / 2,
                ZP + PO.CAM_LENS_A[0], ZP + PO.CAM_LENS_A[1])
    lens = box(XP - PO.PUPIL_BELOW_FRONT, CAM_BOARD_X[0], -PO.CAM_LENS_BLOCK / 2, PO.CAM_LENS_BLOCK / 2,
               ZP - PO.CAM_LENS_BLOCK / 2, ZP + PO.CAM_LENS_BLOCK / 2)
    return board + lens


def mock_neck_servo():
    """The neck STS3215's case, hanging BELOW the horn face."""
    return Pos(0, 0, -D.SV_HORN_FACE) * Rot(180, 0, 0) * CA.servo_mock_z()


def mock_deck_plane():
    z = -V.NECK_HORN_FACE_Z
    return box(-200, 200, -200, 200, z - 4, z)


# ---------------------------------------------------------------- fasteners
def SCREWS():
    s = []
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        s.append(dict(name=f"horn_{ang}", kind="M3x6 machine (into horn)",
                      axis=(0, 0, -1), pos=(x, y, 0.0), length=6.0))
    for i, (x, y) in enumerate(LID_SCREWS):
        s.append(dict(name=f"lid_{i}", kind="M2.5x8 self-tap, flat head",
                      axis=(0, 0, -1), pos=(x, y, ZT), length=8.0))
    for i, (y, z) in enumerate(CAM_HOLES):
        s.append(dict(name=f"cam_{i}", kind="M2x5 self-tap", axis=(1, 0, 0),
                      pos=(CAM_BOARD_X[0], y, z), length=5.0))
    return s


# ---------------------------------------------------------------- checks
def _vol(a, b):
    try:
        return (a & b).volume
    except Exception:  # noqa: BLE001
        return 0.0


def check_optics(verbose=True):
    """Nothing printed in any beam, and the glass seated but not
    interfering: (1) every printed part vs the beams at a SMALLER margin
    than they were cut with; (2) the glass and camera mocks vs the printed
    parts (pockets have 0.15 mm clearance, so ~0); (3) the V and camera
    vs the beams they are not meant to be in."""
    ok = True
    tight = beams(margin=0.3, pencil=0.4)
    for name, part in (("head_shell", head_shell()), ("head_lid", head_lid()),
                       ("camera_sled", camera_sled())):
        v = _vol(part, tight)
        good = v < 0.5
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  {name:12s} in the light path: {v:7.2f} mm3")
    shell = head_shell()
    for name, m in (("V squares", v_tiles_mock()), ("mirror L", mirror_mock(1)), ("mirror R", mirror_mock(-1)),
                    ("camera", camera_mock())):
        v = _vol(shell, m)
        good = v < 1.0
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  {name:9s} vs shell: {v:6.2f} mm3 overlap")
    return ok


def check_horn_access(verbose=True):
    """With the lid off and the sled pulled, a straight 7 mm driver reaches
    each horn screw from above: the cylinder from the base plate's top to
    the roof must miss the shell AND the glued V squares."""
    shell = head_shell() + v_tiles_mock()
    ok = True
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        cyl = cyl_z(D.ACCESS_D / 2, BASE_T + 0.5, Z_ROOF + 5, x, y)
        vol = _vol(cyl, shell)
        clear = vol < 2.0
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  horn_{ang:<4d} driver from above "
                  f"(lid off, sled out) overlap {vol:6.1f} mm3")
    return ok


def check_rom_clearance(verbose=True, samples=13):
    """Sweep neck yaw over V.NECK_ROM: the head clears the neck servo case
    and the pelvis deck plane at every angle."""
    h = head()
    servo = mock_neck_servo()
    deck = mock_deck_plane()
    lo, hi = V.NECK_ROM
    worst = {"servo": 0.0, "deck": 0.0}
    for k in range(samples):
        ang = lo + (hi - lo) * k / (samples - 1)
        hp = Rot(0, 0, ang) * h
        for name, mock in (("servo", servo), ("deck", deck)):
            worst[name] = max(worst[name], _vol(hp, mock))
    ok = True
    for name, vol in worst.items():
        clear = vol < 0.5
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  head vs {name:6s} over +/-{hi:.0f} deg yaw: "
                  f"worst overlap {vol:.2f} mm3")
    return ok


def check_self_view(reach=600.0, verbose=True, pose=None):
    """What of its OWN body does the robot see? Both eyes' fields (exact,
    no margin) out to `reach` mm, placed on the head at the standing pose
    (arms on, hanging), against every piece that is not the head itself.
    Any intersection is a patch of the image that is permanently robot."""
    os.environ.setdefault("ARMS", "1")
    import assembly_v6 as A
    comp = A.robot(pose or {})
    cones = Pos(V.NECK_X, 0, V.NECK_HORN_Z) * view_cones(reach)
    ok, hits = True, []
    for child in comp.children:
        label = getattr(child, "label", "")
        if label.startswith("head"):
            continue
        v = _vol(cones, child)
        if v > 1.0:
            hits.append((label, v))
    ok = not hits
    if verbose:
        if hits:
            for label, v in sorted(hits, key=lambda t: -t[1]):
                print(f"  SEEN  {label:24s} {v:9.1f} mm3 inside the eyes' fields")
        print(f"  {'PASS' if ok else 'NOTE'}  self-view at pose {pose or 'standing, arms hanging'}, "
              f"fields out to {reach:.0f} mm: {len(hits)} piece(s) in view")
    return ok


def mass_report(verbose=True):
    rho = D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    parts = {"head_shell": head_shell(), "head_lid": head_lid(), "camera_sled": camera_sled()}
    petg = {k: v.volume * rho for k, v in parts.items()}
    total = sum(petg.values()) + GLASS_MASS + V.CAM3_MASS
    if verbose:
        for k, m in petg.items():
            print(f"  {k:12s} {m:5.1f} g PETG")
        print(f"  glass        {GLASS_MASS:5.1f} g (2 x V square {VT_MASS:.1f} + 2 x mirror {MIRROR_MASS:.1f})")
        print(f"  camera       {V.CAM3_MASS:5.1f} g")
        print(f"  HEAD TOTAL   {total:5.1f} g")
    return total, petg


# ----------------------------------------------------------------------------
def _render(path, with_cones=False):
    import mujoco
    import imageio
    import tempfile
    tmp = tempfile.mkdtemp(prefix="head_")
    items = [("shell", head_shell() + head_lid(), "0.20 0.20 0.22 1"),
             ("sled", camera_sled(), "0.35 0.35 0.38 1"),
             ("glassL", mirror_mock(1), "0.70 0.85 0.95 1"),
             ("glassR", mirror_mock(-1), "0.70 0.85 0.95 1"),
             ("vtiles", v_tiles_mock(), "0.60 0.80 0.95 1"),
             ("cam", camera_mock(), "0.10 0.45 0.20 1")]
    if with_cones:
        items.append(("cones", view_cones(120.0), "1.0 0.8 0.2 0.25"))
    assets, geoms = [], []
    for name, s, rgba in items:
        stl = os.path.join(tmp, f"{name}.stl")
        export_stl(s, stl)
        assets.append(f'<mesh name="{name}" file="{name}.stl" scale="0.001 0.001 0.001"/>')
        geoms.append(f'<geom type="mesh" mesh="{name}" rgba="{rgba}"/>')
    bb = head().bounding_box()
    cz = (bb.min.Z + bb.max.Z) / 2000.0
    cx = (bb.min.X + bb.max.X) / 2000.0
    xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="1200" offheight="900"/></visual>
<asset>{''.join(assets)}</asset>
<worldbody><light pos="0.3 0.3 0.3" dir="-0.4 -0.4 -0.6" directional="true"/>
<light pos="-0.3 -0.2 0.2" dir="0.4 0.3 -0.4" directional="true"/>
{''.join(geoms)}</worldbody></mujoco>"""
    xml_path = os.path.join(tmp, "s.xml")
    with open(xml_path, "w") as f:
        f.write(xml)
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 900, 1200)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [cx, 0, cz]
    cam.distance = 0.36 if with_cones else 0.21
    imgs = []
    views = ((180, -89.9), (150, -25), (0, -12)) if with_cones else ((150, -25), (0, -12), (-120, -30))
    for az, el in views:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        imgs.append(r.render().copy())
    imageio.imwrite(path, np.concatenate(imgs, axis=1))
    print("wrote", path)


if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    os.makedirs(OUT_REN, exist_ok=True)

    print(f"optics: left eye {EYE_L['w']:.0f}, right eye {EYE_R['w']:.0f} x {OPT['el_hi'] - OPT['el_lo']:.0f} deg, total "
          f"{OPT['total']:.0f} deg, overlap {OPT['overlap']:.0f} deg, baseline {OPT['baseline']:.1f} mm")
    print(f"head: x {X_BACK:.1f}..{XN_FRONT:.1f}, |y| <= {Y_OUT:.1f}, z 0..{ZT:.1f}; "
          f"pupil ({XP}, 0, {ZP}); V apex x {X_APEX:.2f}")

    print("building head_shell(), head_lid(), camera_sled()...")
    shell, lid, sled = head_shell(), head_lid(), camera_sled()
    whole = shell + lid + sled
    for name, s in (("head_shell", shell), ("head_lid", lid), ("camera_sled", sled), ("head", whole)):
        bb = s.bounding_box()
        dims = (bb.size.X, bb.size.Y, bb.size.Z)
        mass = s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        n_solids = len(s.solids())
        print(f"{name:12s} bbox {dims[0]:5.1f} x {dims[1]:5.1f} x {dims[2]:5.1f} mm  "
              f"mass {mass:5.1f} g  solids {n_solids} "
              f"({'ok' if n_solids == 1 or name == 'head' else '** SPLIT **'})  "
              f"{'BED-OK' if max(dims) <= D.BED else '** TOO BIG **'}")
        export_stl(s, os.path.join(OUT_STL, f"{name}.stl"))
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
    # not printed, for looking at it in FreeCAD: the glass, the camera, and the
    # eyes' fields (exact windows, 180 mm out)
    for name, s in (("head_vtiles_mock", v_tiles_mock()), ("head_mirrors_mock", mirror_mock(1) + mirror_mock(-1)),
                    ("head_camera_mock", camera_mock()), ("head_view_cones", view_cones(180.0))):
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))

    print("\n-- mass --")
    mass_report()
    check_dimension_copies()
    print("\n-- optics: light paths clear, glass seated --")
    check_optics()
    print("\n-- horn screw driver access (lid off, sled out) --")
    check_horn_access()
    print("\n-- ROM clearance (+/-90 deg yaw vs the neck servo and the deck) --")
    check_rom_clearance()
    print("\n-- self-view: robot parts inside the eyes' fields --")
    check_self_view()
    print("\n-- printability audit --")
    import audit_torso  # noqa: E402
    audit_torso.run(["head_shell", "head_lid", "camera_sled"])
    print("\n-- render --")
    try:
        _render(os.path.join(OUT_REN, "head.png"))
        _render(os.path.join(OUT_REN, "head_fields.png"), with_cones=True)
    except Exception as e:  # noqa: BLE001
        print(f"render skipped ({type(e).__name__}: {e})")
