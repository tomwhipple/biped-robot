"""head: the STEREO-PERISCOPE camera head on its own neck servo (v6 body).

2026-09-24, replacing the 2026-09-14 single forward camera: ONE Camera Module 3
(Wide) sees in stereo through two horizontal periscopes -- binocular vision from
a single sensor, both eye openings in one horizontal plane, the horizontal field
as wide as stock optics allow. The optics are designed, and every optical number
here is derived, in periscope_optics.py; the design note with the trade study
and the parts list is docs/design-v6/stereo-head.md.

Seen from above (+X forward, the neck yaw axis at the origin):

         eye window L     | nose |     eye window R        <- the FACE
      ____________________|  CM3 |____________________
      \\  outer mirror L   \\  |  /   outer mirror R  //
       \\                    \\|/                    //     the chevron walls ARE
        \\                  prism                   //      the mirror mounts
         \\_______________ [pad] _______________//          <- back wall

  * The camera looks AFT (-X) into a 20 mm knife-edge right-angle prism mirror
    2.8 mm behind its entrance pupil. The prism's two coated legs split the
    sensor's long axis; each half-image goes sideways to a 50 x 50 mm
    first-surface mirror that turns it forward out of that side's eye window.
    Two reflections per eye: each eye's image is rotated, never mirrored.
  * Per eye 38 deg (H) x 40 deg (V, -30..+10 deg); the eyes are toed out so
    the total field is 56 deg with 20 deg of stereo overlap; 63.5 mm baseline
    between the two virtual pupils. periscope_optics.py records why the camera
    faces aft (it keeps the outer mirrors small) and why no stock splitter
    gives more than ~45 deg per eye.
  * The prism sits BEHIND the horn's screw circle and the camera rides a
    removable sled in the nose above it: lift the lid, pull the sled, and the
    four M3 horn screws are driven straight down, as on the old head.
  * The chevron walls lean 2.3 deg: that is the optics, not slop -- a vertical
    mirror loses 9 deg per eye (the lens's field edge at each eye's inner
    edge only reaches ~27 deg down; the tilt rotates the eye's view to keep
    the full -30).

Parts (all print base-down; the sled prints lying on its front face):
  head_shell()   base plate on the horn + the chevron body -- both mirror
                 walls, back wall, face frame with the two eye windows, the
                 nose, and the prism pad. Open on top.
  head_lid()     the roof; 4x M2.5 flat-head self-tappers into shell bosses.
                 It also holds the camera sled down.
  camera_sled()  the plate the Camera Module 3 screws to (4x M2 self-tap);
                 slides down grooves in the nose.
  head()         all three fused -- the assembly, the ROM checks, the mass.
Mocks (not printed): prism_mock(), mirror_mock(side), camera_mock(), and
view_cones() -- the eyes' fields, for renders and the self-occlusion check.

Every beam -- pupil to prism, prism to mirror, mirror out through the window --
is built as a solid from the traced design (with margin) and subtracted from
every printed part, so nothing printed can sit in the light path; check_optics()
re-proves it on the finished parts.

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
    OPT = PO.design()            # w, az_lo/az_hi, el_lo/el_hi, baseline, vp, cfg
CFG = OPT["cfg"]

# the camera's entrance pupil, head-local. XP puts the prism's apex just aft
# of the horn screws' driver cylinders (see check_horn_access). ZP is set by
# the eyes' lowest rays, not by the mirrors: at 40.5 (mirrors a rim above
# the base) the -30 deg cones sliced the socket floor into knife-edge
# slivers out at the face; at 47.0 they clear the whole floor up to the
# face, so the floor stays whole -- and stiff, which the stereo calibration
# needs more than it needs the 6.5 mm.
XP, ZP = -6.5, 47.0
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

# prism: Edmund #49-414, legs = height = 20, apex line vertical
PRISM_LEG = CFG.prism_leg
PRISM_LEN = CFG.prism_len
PRISM_HYP = PRISM_LEG * math.sqrt(2)                   # 28.28 across the back
APEX = H(CFG.apex)                                     # apex-line centre
X_APEX = APEX[0]                                       # -9.27
X_HYP = X_APEX - PRISM_LEG / math.sqrt(2)              # -23.42, the glued back face
PRISM_Z = (APEX[2] - PRISM_LEN / 2, APEX[2] + PRISM_LEN / 2)
PRISM_MASS = 10.0                                      # g, Edmund spec (N-BK7)

# outer mirrors: Edmund #43-876, 50 x 50 x 3.0, enhanced Al 4-6 lambda
MIR_C = H(CFG.outer.c)                                 # reflective-face centre, +Y side
MIR_N = CFG.outer.n                                    # out of the reflective face
MIR_E1 = CFG.outer.e1                                  # along its 50 mm length (horizontal)
MIR_E2 = CFG.outer.e2                                  # along its 50 mm height (~vertical)
MIR_L, MIR_H, MIR_T = CFG.mirror_L, CFG.mirror_H, CFG.mirror_t
MIRROR_MASS = 18.8                                     # g each, Edmund spec

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
X_BACK = float(np.floor(_aft[0] * 10) / 10)            # back wall's outer face
Y_OUT = float(np.ceil(_fwd[1] * 10) / 10)              # outer side's outer face

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

# the prism mount: a plate on the back face, a 45-deg ledge under it
PAD_T = 3.0
PAD_HW = PRISM_HYP / 2 - 1.5                           # inside the hypotenuse, clear of the beams
PRISM_POCKET = 0.5
LEDGE = 4.0

CABLE_X = -(BCD_R + 2.0 + V.HEAD_CABLE_SLOT[1] / 2)    # -11: ribbon slot behind the screw circle

# lid screws: two against the back wall behind the prism pad (no beam passes
# |y| < the pad there), two in the nose side walls just ahead of the face
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
def _corner_dirs(margin):
    a0, a1 = OPT["az_lo"] - margin, OPT["az_hi"] + margin
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
    E = _corner_dirs(0.0)
    vp = H(CFG.virtual_pupil())
    cone = _pyramid(vp, E, reach, 0.0) & _halfspace(H(CFG.outer.c), CFG.outer.n)
    return cone + _mirror_y(cone)


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


def _mirror_pocket(side):
    """The glass's own volume plus everything in front of it: 0.2 mm
    clearance round the outline, floor exactly at the glass's back face."""
    c = MIR_C + MIR_N * (-MIR_T + 6.0) / 2
    s = _frame_box(c, MIR_E1, MIR_N, (MIR_L + 0.4, MIR_H + 0.4, MIR_T + 6.0))
    return s if side > 0 else _mirror_y(s)


def _in_mirror_plane(pts_e1e2, n0, n1):
    """A prism whose profile is given in the mirror's own (e1, e2) coords,
    spanning n0..n1 along its normal (+Y mirror frame)."""
    pl = Plane(origin=_v(MIR_C + MIR_N * n0), x_dir=_v(MIR_E1), z_dir=_v(MIR_N))
    area2 = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts_e1e2, pts_e1e2[1:] + pts_e1e2[:1]))
    pts = list(pts_e1e2) if area2 > 0 else list(reversed(pts_e1e2))    # CCW -> face normal +n
    out = extrude(pl * Polygon(*pts, align=None), amount=n1 - n0)
    bb = out.bounding_box()
    assert np.dot(_unit(MIR_N), [(bb.min.X + bb.max.X) / 2 - pl.origin.X, (bb.min.Y + bb.max.Y) / 2 - pl.origin.Y,
                                 (bb.min.Z + bb.max.Z) / 2 - pl.origin.Z]) >= -1e-6, "extruded the wrong way"
    return out


def _mirror_window(side):
    """Opening in the wall behind the glass (the glass bears on a 5 mm
    frame round it). The top is a 45-deg peak, not a flat 40 mm bridge:
    printed base-down, a flat top would be an unsupported ceiling."""
    w, b = (MIR_L - MIR_WINDOW) / 2, -(MIR_H - MIR_WINDOW) / 2
    top = (MIR_H - MIR_WINDOW) / 2
    pts = [(-w, b), (w, b), (w, top - w), (0.0, top), (-w, top - w)]
    s = _in_mirror_plane(pts, WALL_OUT - 1.0, -MIR_T + 0.1)
    return s if side > 0 else _mirror_y(s)


def _pocket_top_chamfer(side):
    """60-deg chamfer on the glue pocket's top edge: the wall above the
    recess would otherwise overhang it by the pocket depth. (45 is not
    enough: the wall's own 2.3 deg lean tips a 45 over the audit's line.)"""
    ht = MIR_H / 2 + 0.2
    pl = Plane(origin=_v(MIR_C), x_dir=_v(MIR_E2), z_dir=_v(MIR_E1))    # local x = e2, y = n
    run = WALL_IN + 0.3 - (-MIR_T - 0.05)
    tri = [(ht - 0.05, -MIR_T - 0.05), (ht - 0.05, WALL_IN + 0.3), (ht - 0.05 + run * 1.75, WALL_IN + 0.3)]
    s = extrude(pl * Polygon(*tri, align=None), amount=MIR_L / 2 + 0.2, both=True)
    return s if side > 0 else _mirror_y(s)


def _prism_mount():
    """Pad plate on the hypotenuse (full height, floor to lid) + a 45-deg
    ledge under the prism's back edge that sets its height."""
    pad = box(X_HYP - PAD_T, X_HYP + PRISM_POCKET, -PAD_HW, PAD_HW, BASE_T - 0.1, Z_ROOF)
    ledge = P.wedge_y([(X_HYP, PRISM_Z[0]), (X_HYP + LEDGE, PRISM_Z[0]), (X_HYP, PRISM_Z[0] - LEDGE)],
                      -PAD_HW + 3.5, PAD_HW - 3.5)
    return pad + ledge


def _prism_pocket():
    """0.5 mm recess in the pad for the hypotenuse face (sets x and z; y is
    set by ACTIVE alignment -- nudge the prism against the live image until
    the split is centred, then let the epoxy cure)."""
    return box(X_HYP, X_HYP + 2.0, -(PRISM_HYP / 2 + 0.15), PRISM_HYP / 2 + 0.15,
               PRISM_Z[0] - 0.15, PRISM_Z[1] + 0.15)


def _body():
    """Shell + lid as one solid, before the lid is split off."""
    b = (_main_body() + _nose_outer()) - (_interior() + _nose_inner())
    b += _prism_mount()
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
    p -= _prism_pocket()
    zt = PRISM_Z[1] + 0.15                                   # 45-deg chamfer over the prism recess
    p -= P.wedge_y([(X_HYP - 0.05, zt - 0.05), (X_HYP + PRISM_POCKET + 0.3, zt - 0.05),
                    (X_HYP + PRISM_POCKET + 0.3, zt + PRISM_POCKET + 0.35)],
                   -(PRISM_HYP / 2 + 0.15), PRISM_HYP / 2 + 0.15)
    # sled grooves: 1 mm into each nose side wall, open at the top, a stop at SLED_Z0
    p -= box(SLED_X[0] - 0.15, SLED_X[1] + 0.15, -(SLED_HW + 0.15), SLED_HW + 0.15,
             SLED_Z0 - 0.2, Z_ROOF + 1.0)
    # every beam, with margin, and the two eye windows cut square
    p -= beams()
    p -= face_windows()
    # the base plate: horn screws on the O14 BCD, centre relief, ribbon slot,
    # and a lightening pocket from ABOVE (a pocket from below would print as
    # a 30 x 133 mm bridge) leaving a 1.3 mm floor, a boss round the horn,
    # a rim under every wall, and solid under the prism pad
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        p -= cyl_z(D.PAD_HOLE / 2, -1, BASE_T + 1, x, y)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -1, BASE_T + 1, 0, 0)
    p -= box(CABLE_X - V.HEAD_CABLE_SLOT[1] / 2, CABLE_X + V.HEAD_CABLE_SLOT[1] / 2,
             -V.HEAD_CABLE_SLOT[0] / 2, V.HEAD_CABLE_SLOT[0] / 2, -1, BASE_T + 1)
    zp = 1.3
    pocket = _interior(extra=1.5, z0=zp, z1=BASE_T + 0.01) + _nose_inner(extra=1.5, z0=zp, z1=BASE_T + 0.01)
    pocket -= cyl_z(BCD_R + 5.0, -2, BASE_T + 1, 0, 0)
    pocket -= box(X_HYP - PAD_T - 2.0, X_HYP + LEDGE + 1.0, -(PAD_HW + 2.0), PAD_HW + 2.0, -2, BASE_T + 1)
    pocket -= box(CABLE_X - 4.0, CABLE_X + 4.0, -9.0, 9.0, -2, BASE_T + 1)
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


GLASS_MASS = PRISM_MASS + 2 * MIRROR_MASS


def check_dimension_copies(tol=0.06):
    """dimensions_v6 carries copies of this head's numbers for the modules
    that must not import the CAD (plant, inertia, assembly fallback). They
    must still match."""
    mc = MIR_C - MIR_N * MIR_T / 2
    want = {
        "HEAD_X": (X_BACK, XN_FRONT), "HEAD_W": 2 * Y_OUT, "HEAD_D": XN_FRONT - X_BACK,
        "HEAD_H": ZT - BASE_T, "CAM_Z_ABOVE_HORN": ZP - BASE_T,
        "HEAD_PRISM_C": ((X_APEX + 2 * X_HYP) / 3, 0.0, APEX[2]), "HEAD_MIRROR_C": tuple(mc),
        "HEAD_MIRROR_YAW": math.degrees(math.atan2(MIR_N[1], MIR_N[0])),
        "HEAD_PUPIL": tuple(PUPIL), "HEAD_EYE_VP": tuple(H(CFG.virtual_pupil())),
        "HEAD_PRISM_MASS": PRISM_MASS, "HEAD_MIRROR_MASS": MIRROR_MASS,
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
def prism_mock():
    tri = Wire.make_polygon([_v((X_APEX, 0, PRISM_Z[0])),
                             _v((X_HYP, PRISM_HYP / 2, PRISM_Z[0])),
                             _v((X_HYP, -PRISM_HYP / 2, PRISM_Z[0]))], close=True)
    return extrude(Face(tri), amount=PRISM_LEN)


def mirror_mock(side=+1):
    s = _frame_box(MIR_C - MIR_N * MIR_T / 2, MIR_E1, MIR_N, (MIR_L, MIR_H, MIR_T))
    return s if side > 0 else _mirror_y(s)


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
    parts (pockets have 0.2 mm clearance, so ~0); (3) the prism and camera
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
    for name, m in (("prism", prism_mock()), ("mirror L", mirror_mock(1)), ("mirror R", mirror_mock(-1)),
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
    the roof must miss the shell AND the glued prism."""
    shell = head_shell() + prism_mock()
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
        print(f"  glass        {GLASS_MASS:5.1f} g (prism {PRISM_MASS} + 2 x mirror {MIRROR_MASS})")
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
             ("prism", prism_mock(), "0.60 0.80 0.95 1"),
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

    print(f"optics: per eye {OPT['w']:.0f} x {OPT['el_hi'] - OPT['el_lo']:.0f} deg, total "
          f"{OPT['total']:.0f} deg, overlap {OPT['overlap']:.0f} deg, baseline {OPT['baseline']:.1f} mm")
    print(f"head: x {X_BACK:.1f}..{XN_FRONT:.1f}, |y| <= {Y_OUT:.1f}, z 0..{ZT:.1f}; "
          f"pupil ({XP}, 0, {ZP}); prism apex x {X_APEX:.2f}, back x {X_HYP:.2f}")

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
    for name, s in (("head_prism_mock", prism_mock()), ("head_mirrors_mock", mirror_mock(1) + mirror_mock(-1)),
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
