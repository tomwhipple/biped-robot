"""Solid-building helpers for the printed parts (build123d, algebra mode).

Boxes and cylinders by their extents, self-supporting teardrop bores,
extruded wedges, countersinks, the horn's 4-hole bolt circle and the L/R
letters the parts carry. Every part module imports these as one module so a
helper exists once.
"""
import math

from build123d import *  # noqa: F401,F403

from common import dimensions as D


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl_x(r, x0, x1, y, z):
    """Cylinder along X from x0..x1 at (y, z)."""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def cyl_y(r, y0, y1, x, z):
    """Cylinder along Y from y0..y1 at (x, z)."""
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_z(r, z0, z1, x, y):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def teardrop_y(r, y0, y1, x, z, roll=0, full=False):
    """Self-supporting horizontal hole (axis along Y): a round bore plus a
    45 deg roof, so a horizontal hole prints without a bridged (sagging) top.
    Returns the SOLID void to subtract (bore + roof), not a hole in a part.
    `roll` spins the roof about the bore axis toward the part's PRINT-up
    direction: 0 = +z (parts printed model-up), 90 = +x (leg_link prints
    web-down, print-up = model +x), 180 = -z (parts modeled upside-down vs
    their print, e.g. yoke_pitch).
    Default is CAPPED: the roof is truncated at the round bore's own top
    (center + r), leaving a 2r*(sqrt(2)-1) ~ 0.83r flat mini-bridge. The void
    then never reaches past the round hole it replaces -- a full r*sqrt(2)
    peak pierced plate edges and left ~0.1 mm shells on the O19 idler bosses
    wherever holes were placed with round-hole margins. Pass full=True only
    where clearance above the hole is proven (e.g. gopro_base M5)."""
    L = abs(y1 - y0)
    bore = Rot(90, 0, 0) * Cylinder(r, L)
    # A square of side 2r rotated 45 deg about Y peaks at (0, r*sqrt(2)); clip
    # it to |dx| <= r and above the bore center so it only adds the top roof
    # (its 45 deg faces meet the circle exactly at the tangent points).
    diamond = Rot(0, 45, 0) * Box(2 * r, L, 2 * r)
    keep = box(-r, r, -L / 2, L / 2, 0, 2 * r if full else r)
    return Pos(x, (y0 + y1) / 2, z) * Rot(0, roll, 0) * (bore + (diamond & keep))


def teardrop_x(r, x0, x1, y, z, roll=0, full=False):
    """teardrop_y's sibling with the bore along X. roll about the bore axis:
    0 = peak +z, 180 = peak -z (pelvis prints deck-top-down)."""
    L = abs(x1 - x0)
    bore = Rot(0, 90, 0) * Cylinder(r, L)
    diamond = Rot(45, 0, 0) * Box(L, 2 * r, 2 * r)
    keep = box(-L / 2, L / 2, -r, r, 0, 2 * r if full else r)
    return Pos((x0 + x1) / 2, y, z) * Rot(roll, 0, 0) * (bore + (diamond & keep))


def wedge_y(pts_xz, y0, y1):
    """Triangular (or any polygon) prism: a profile of (x, z) points extruded
    thin along Y into the band [y0, y1]. Used for gussets and print chamfers.
    NOTE: don't assume which way extrude() runs from Plane.XZ -- measure and
    shift, so the prism lands exactly in [y0, y1] on any build123d version
    (the old +Pos(0, hi, 0) guess put the v2 foot gussets 2.4 mm outside
    their tab bands without anything catching it)."""
    lo, hi = (y0, y1) if y0 < y1 else (y1, y0)
    s = extrude(Plane.XZ * Polygon(*pts_xz, align=None), amount=(hi - lo))
    return Pos(0, lo - s.bounding_box().min.Y, 0) * s


def wedge_z(pts_xy, z0, z1):
    """Same idea as wedge_y, but the profile is (x, y) and the prism runs along
    Z. Used for print chamfers on pockets whose overhang is in the x-y plane
    (leg_link's rib and platform detents, which print with +x up)."""
    lo, hi = (z0, z1) if z0 < z1 else (z1, z0)
    s = extrude(Plane.XY * Polygon(*pts_xy, align=None), amount=(hi - lo))
    return Pos(0, 0, lo - s.bounding_box().min.Z) * s


def wedge_x(pts_yz, x0, x1):
    """wedge_y's third sibling: a (y, z) profile extruded thin along X into the
    band [x0, x1]. Needed wherever the chamfer/gusset slope lives in the Y-Z
    plane -- X-NORMAL plates (board_frame's bulkhead gable, the yaw box side
    lead-ins, the guard's foot gussets). Same bbox-measured placement as
    wedge_y: do not assume which way extrude() runs off Plane.YZ."""
    lo, hi = (x0, x1) if x0 < x1 else (x1, x0)
    s = extrude(Plane.YZ * Polygon(*pts_yz, align=None), amount=(hi - lo))
    return Pos(lo - s.bounding_box().min.X, 0, 0) * s


def csk_y(x, z, y_face, sign):
    """90-deg countersink void for an M2.5 FLAT-head self-tapper (CASE_CS_D
    mouth), mouth on the face at y_face opening toward sign*Y, apex meeting
    the CASE_SCREW_CLEAR bore at CASE_CS_DEPTH. 0.3 overshoot past the face.
    45-deg walls: prints self-supporting even as a horizontal bore."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(-90, 0, 0) if sign > 0 else Rot(90, 0, 0)
    mid = y_face + sign * (0.3 - h / 2)
    return Pos(x, mid, z) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def csk_x(y, z, x_face, sign):
    """csk_y's twin for bores along X (yaw_carrier bay walls)."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(0, 90, 0) if sign > 0 else Rot(0, -90, 0)
    mid = x_face + sign * (0.3 - h / 2)
    return Pos(mid, y, z) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def csk_z(x, y, z_face, sign):
    """csk_y's twin for bores along Z (pelvis deck stator screws)."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(0, 0, 0) if sign > 0 else Rot(180, 0, 0)
    mid = z_face + sign * (0.3 - h / 2)
    return Pos(x, y, mid) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def bcd_y(y0, y1, x, z, roll=0):
    """4x M3 clearance holes (horn/idler bolt circle) along Y at pad (x, z).
    Teardropped (roll = print-up, see teardrop_y): these bores are horizontal
    in every part's print orientation, and a sagged bore top binds the bolt."""
    r = D.BCD / 2
    return [teardrop_y(D.PAD_HOLE / 2, y0, y1, x + dx, z + dz, roll)
            for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


def bcd_x(x0, x1, y, z, roll=0):
    r = D.BCD / 2
    return [teardrop_x(D.PAD_HOLE / 2, x0, x1, y + dy, z + dz, roll)
            for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


def _rect2(u0, u1, v0, v1):
    return Pos((u0 + u1) / 2, (v0 + v1) / 2) * Rectangle(u1 - u0, v1 - v0)


def letter_2d(ch, h, w=None):
    """Capital L or R, h tall and centred on the origin, as a face in the (u, v)
    plane (v up), drawn from strokes of width w (default h / 6). Strokes, not a
    font: a font's outline depends on the fonts of the machine that builds it,
    and the STLs it cuts are committed."""
    w = w or h / 6
    hh = h / 2
    if ch == "L":
        hw = 0.3 * h
        return _rect2(-hw, -hw + w, -hh, hh) + _rect2(-hw, hw, -hh, -hh + w)
    if ch != "R":
        raise ValueError(f"letter_2d draws L and R, not {ch!r}")
    hw = 0.34 * h
    rb = 0.28 * h                                  # the bowl's outer radius
    uc, vc = hw - rb, hh - rb                      # its centre
    vm = hh - 2 * rb                               # the bowl's foot: the leg starts here
    face = _rect2(-hw, -hw + w, -hh, hh)           # stem
    face += _rect2(-hw, uc, hh - w, hh) + _rect2(-hw, uc, vm, vm + w)
    face += (Pos(uc, vc) * (Circle(rb) - Circle(rb - w))) & _rect2(uc, hw + 1, vc - rb, vc + rb)
    # the leg: from under the bowl's foot to the bottom right corner, w wide
    # across its own direction
    top_u, top_v, bot_v = uc - w / 2, vm + w, -hh
    k = w / math.sin(math.atan2(top_v - bot_v, hw - top_u))   # its width along u
    bot_u = hw - k / 2
    face += Polygon((top_u - k / 2, top_v), (bot_u - k / 2, bot_v),   # counter-clockwise,
                    (bot_u + k / 2, bot_v), (top_u + k / 2, top_v),   # so its normal is +v x u
                    align=None)
    return face


def letter_y(ch, y_face, outward, x, z, h, depth):
    """The void that debosses letter_2d(ch, h) `depth` into a Y-normal face at
    y = y_face whose outside is +y (outward = +1) or -y (-1), centred at (x, z),
    upright along +z and reading correctly from outside. Cut it AFTER any
    mirror: a mirrored letter reads backwards."""
    plane = Plane(origin=(x, y_face, z), x_dir=(-outward, 0, 0), z_dir=(0, outward, 0))
    s = extrude(plane * letter_2d(ch, h), amount=depth + 0.2)
    lo = y_face - depth if outward > 0 else y_face - 0.2     # 0.2 mm proud of the face
    return Pos(0, lo - s.bounding_box().min.Y, 0) * s
