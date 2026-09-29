"""periscope_tb: the TOP/BOTTOM split -- each eye keeps the frame's full width.

Tom, 2026-09-24, on the first stereo head (head.py, a LEFT/RIGHT split):

> I was imagining the bottom half to be one side, and the top half to be the
> other ... that way we get a wider horizontal field of view.

head.py cuts the frame with a VERTICAL knife edge, so each eye gets half the
frame's width (capped below 45 deg, 38 in practice) at its full height. Cut it
top/bottom and each eye keeps the full width -- the Wide lens's 102 deg -- at
half the height (<= 33.5 deg). This module traces whether that width survives
the trip to two level, side-by-side eye windows.

Two families of optical train, each eye:

  PRISM  a knife-edge prism with its apex HORIZONTAL sends the top half up and
         the bottom half down; k mirrors then bring each half to its window.
  FOLD   no prism: two plain mirrors stacked in front of the lens, their edges
         meeting in the camera's horizontal mid-plane -- the upper one takes the
         top half, the lower one the bottom half; k more mirrors per eye.

A top/bottom split cannot be mirror-symmetric left-to-right (the top half can
only feed one side). The symmetry it does allow is a 180-deg turn about the
camera axis: the right eye's optics are the left eye's, upside down (C2 below).
That is what keeps both windows level -- each exit beam leaves on the camera's
mid-plane -- and both eyes on the same band, which is centred on the camera
axis. Pitch the whole head (camera and all) to aim the band at the floor.

Every design here is parametrised by the path of its centre ray: the
splitter's hit, then waypoints, the last one on the mid-plane (the level
window), then the eye's aim. Each mirror's normal is the bisector at its
waypoint. The field is whatever survives: every direction in the eye's
az x el rectangle must back-trace to the correct half of the sensor, land
inside every stock aperture, and clear the camera, the prism and every
piece of glass (both eyes') on the way. Same pinhole model and margins as
periscope_optics.py.

Run:
    .venv/bin/python cad/v6/periscope_tb.py              # the study (all families)
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import periscope_optics as PO  # noqa: E402
from periscope_optics import (unit, reflect, dir_az_el, Plane, Convex, box_solid,  # noqa: E402
                              U_MAX, V_MAX, PUPIL_MARGIN)

X, Y, Z = np.eye(3)
C2 = np.diag([1.0, -1.0, -1.0])        # 180 deg about the camera axis line (X)
V_BAND = 26.0                          # eye's vertical field, centred on the axis (deg)
BIG = 400.0                            # "unbounded" aperture (mm)
GLASS_T = 3.0                          # stock first-surface mirrors are 3.0 mm


def _frame(n, edge=None):
    """In-plane axes for a mirror with normal n: e1 horizontal (or `edge`),
    e2 = n x e1."""
    if edge is None:
        e1 = np.cross(n, Z)
        if np.linalg.norm(e1) < 0.2:
            e1 = np.cross(n, X)
    else:
        e1 = edge - np.dot(edge, n) * n
    e1 = unit(e1)
    return e1, np.cross(n, e1)


def c2_plane(p: Plane) -> Plane:
    return Plane(p.name + "'", C2 @ p.c, C2 @ p.n, C2 @ p.e1)


def glass_box(p: Plane, lo, hi, t=GLASS_T):
    c = p.c + p.e1 * (lo[0] + hi[0]) / 2 + p.e2 * (lo[1] + hi[1]) / 2 - p.n * t / 2
    h = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, t / 2 - 0.05]
    return box_solid(p.name + "_glass", c, (p.e1, p.e2, p.n), [-v for v in h], h)


@dataclass
class Camera:
    aft: bool

    def __post_init__(self):
        # sensor long axis S horizontal, short axis A vertical
        self.C = -X if self.aft else X
        self.S, self.A = Y, Z

    def solids(self):
        C, S, A = self.C, self.S, self.A
        lens_front = PO.PUPIL_BELOW_FRONT
        board_front = lens_front - PO.CAM_LENS_H
        board = box_solid("cam_board", np.zeros(3), (C, S, A),
                          (board_front - PO.CAM_BOARD[2] - PO.CAM_BOARD_BACK, -PO.CAM_BOARD[0] / 2, PO.CAM_LENS_A[0]),
                          (board_front, PO.CAM_BOARD[0] / 2, PO.CAM_LENS_A[1]))
        lens = box_solid("cam_lens", np.zeros(3), (C, S, A),
                         (board_front, -PO.CAM_LENS_BLOCK / 2, -PO.CAM_LENS_BLOCK / 2),
                         (lens_front, PO.CAM_LENS_BLOCK / 2, PO.CAM_LENS_BLOCK / 2))
        # the camera board is 25 x 24 with the LONG side horizontal here:
        # S carries the 25 mm, A the 24 (the lens sits 8.25 below the top edge)
        return [board, lens]


# ---------------------------------------------------------------- trains
@dataclass
class Train:
    """One eye's sequence of reflecting planes from the pupil, the +A (top)
    half of the sensor. apertures[i] = (lo, hi) in that plane's (e1, e2);
    edge0: the splitter's split edge is at e2 = 0 (no margin there)."""
    cam: Camera
    planes: list
    apertures: list
    glass_t: list
    prism: Convex | None = None
    splitter_is_leg: bool = False

    def R(self):
        R = np.eye(3)
        for p in self.planes:
            R = (np.eye(3) - 2 * np.outer(p.n, p.n)) @ R
        return R

    def roll(self):
        h = self.R() @ self.cam.S
        return math.degrees(math.asin(np.clip(h[2], -1, 1)))

    def vpupil(self):
        q = np.zeros(3)
        for p in self.planes:
            q = q - 2 * np.dot(q - p.c, p.n) * p.n
        return q

    def partner(self):
        return [c2_plane(p) for p in self.planes]

    def obstacles(self, with_glass=True):
        obs = list(self.cam.solids())
        if self.prism is not None:
            obs.append(self.prism)
        if with_glass:
            for trainp in (self.planes, self.partner()):
                for i, (p, ap, t) in enumerate(zip(trainp, self.apertures, self.glass_t)):
                    if t > 0:
                        lo, hi = ap
                        obs.append(glass_box(p, lo, hi, t))
        return obs

    def trace(self, Dc, apertures=True, obstacles=(), m=PUPIL_MARGIN):
        """Vectorised: ok mask + per-plane local hit coords."""
        N = len(Dc)
        O = np.zeros((N, 3))
        D = Dc
        ok = np.ones(N, bool)
        hits, segs = [], []
        for i, p in enumerate(self.planes):
            t, P, s1, s2, okh = p.hit(O, D)
            ok &= okh
            if apertures:
                lo, hi = self.apertures[i]
                e2_lo = lo[1] if i == 0 else lo[1] + m          # split edge: no margin
                ok &= (s1 >= lo[0] + m) & (s1 <= hi[0] - m) & (s2 >= e2_lo) & (s2 <= hi[1] - m)
            hits.append((s1, s2))
            if i > 0:
                segs.append((O, D, t))
            D = reflect(D, p.n)
            O = np.where(ok[:, None], P, 0.0)
        segs.append((O, D, np.full(N, 250.0)))
        for (O_, D_, T_) in segs:
            On = np.where(ok[:, None], O_ + D_ * 0.05, 0.0)
            Tn = np.where(ok, T_ - 0.1, 0.0)
            for ob in obstacles:
                ok &= ~ob.blocks(On, D_, Tn)
        return ok, hits


def eye_grid(train: Train, az, el):
    AZ, EL = np.meshgrid(az, el, indexing="xy")
    E = dir_az_el(AZ.ravel(), EL.ravel())
    Dc = E @ train.R()                    # R^T e, row-wise (R orthogonal, proper or not)
    cam = train.cam
    z = Dc @ cam.C
    with np.errstate(divide="ignore", invalid="ignore"):
        u, v = (Dc @ cam.S) / z, (Dc @ cam.A) / z
    on = (z > 0) & (np.abs(u) <= U_MAX) & (v >= 0) & (v <= V_MAX)
    return AZ, EL, unit(Dc), on


def best_band(mask, az, el, band=V_BAND):
    rows = np.abs(el) <= band / 2 + 1e-9
    cols = mask[rows].all(axis=0)
    run, start, bl = 0, 0, (0, 0, 0)
    for j, c in enumerate(cols):
        if c:
            if run == 0:
                start = j
            run += 1
            if run > bl[0]:
                bl = (run, start, j)
        else:
            run = 0
    if bl[0] < 2:
        return None
    return az[bl[2]] - az[bl[1]], az[bl[1]], az[bl[2]]


# ---------------------------------------------------------------- builders
# Zero roll is built in, not searched for: the camera-to-eye rotation R is
# chosen first (it sends the half-field's centre ray to the eye's aim, keeps
# the sensor's long axis horizontal), then factored into reflections. Any
# rotation is two reflections in planes containing its axis, half its angle
# apart (a one-parameter family: the pair's orientation about the axis);
# three reflections = one free mirror followed by such a pair.

def rodrigues(u, ang):
    u = unit(u)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def axis_angle(Q):
    w, V = np.linalg.eig(Q)
    i = int(np.argmin(np.abs(w - 1.0)))
    u = unit(np.real(V[:, i]))
    c = np.clip((np.trace(Q) - 1.0) / 2.0, -1.0, 1.0)
    Kv = np.array([Q[2, 1] - Q[1, 2], Q[0, 2] - Q[2, 0], Q[1, 0] - Q[0, 1]]) / 2.0
    return u, math.atan2(float(np.dot(Kv, u)), float(c))


def perp_basis(u):
    a = X if abs(u[0]) < 0.9 else Y
    p = unit(np.cross(u, a))
    return p, np.cross(u, p)


def reflection_pair(Q, beta):
    """Normals (n_a, n_b) with M_b M_a = Q (proper rotation); beta picks
    the pair's orientation about Q's axis."""
    u, th = axis_angle(Q)
    p, q = perp_basis(u)
    na = math.cos(beta) * p + math.sin(beta) * q
    nb = rodrigues(u, th / 2.0) @ na
    return na, nb


def target_R(cam, v_c, toe, s, t):
    """Camera -> eye: the half-field's centre ray (u = 0, v = v_c) to the
    aim (toe, el 0); the sensor's long axis S to the eye's horizontal
    (times s); d0 x S to vertical (times t). det = s*t."""
    d0 = unit(cam.C + v_c * cam.A)
    F = np.column_stack([d0, cam.S, np.cross(d0, cam.S)])
    E = dir_az_el(toe, 0.0)
    Hh = np.array([-math.sin(math.radians(toe)), math.cos(math.radians(toe)), 0.0])
    G = np.column_stack([E, s * Hh, t * np.cross(E, Hh)])
    return G @ F.T, d0, E


def M(n):
    return np.eye(3) - 2.0 * np.outer(n, n)


def _facing(n, d_in):
    return n if np.dot(n, d_in) < 0 else -n


def _chain(P1, d1, normals, lengths_or_z0):
    """Place mirrors along the centre ray: after P1 (arriving along d1),
    each subsequent mirror is at the given path length, except the LAST,
    which sits where the ray crosses the mid-plane z = 0 (level window)."""
    planes, P, d = [], P1, d1
    for i, n in enumerate(normals):
        last = i == len(normals) - 1
        if last:
            if abs(d[2]) < 1e-6 or (-P[2] / d[2]) <= 1.0:
                raise ValueError("ray never reaches the mid-plane ahead")
            L = -P[2] / d[2]
        else:
            L = lengths_or_z0[i]
        P = P + d * L
        n = _facing(n, d)
        e1, _ = _frame(n)
        planes.append(Plane(f"M{i + 1}", P, n, e1))
        d = reflect(d, n)
    return planes, d


def build_fold1(cam, alpha, x_e, toe, v_c, sgn):
    """Fold mirror F + one outer mirror O per eye. R proper: M_O M_F = R."""
    R, d0, E = target_R(cam, v_c, toe, sgn, sgn)
    nF, nO = reflection_pair(R, math.radians(alpha))
    return _fold_train(cam, nF, x_e, d0, [nO], [], E)


def build_fold2(cam, azF, elF, x_e, beta, l1, toe, v_c, s):
    """Fold mirror F (free normal) + two mirrors. R improper (s*t = -1)."""
    R, d0, E = target_R(cam, v_c, toe, s, -s)
    nF = dir_az_el(azF, elF)
    na, nb = reflection_pair(R @ M(nF), math.radians(beta))
    return _fold_train(cam, nF, x_e, d0, [na, nb], [l1], E)


def build_fold3(cam, azF, elF, x_e, azA, elA, beta, l1, l2, toe, v_c, s):
    """Fold F + a free mirror A + a reflection pair: 4 reflections, R proper."""
    R, d0, E = target_R(cam, v_c, toe, s, s)
    nF = dir_az_el(azF, elF)
    nA = dir_az_el(azA, elA)
    nb, nc = reflection_pair(R @ M(nF) @ M(nA), math.radians(beta))
    return _fold_train(cam, nF, x_e, d0, [nA, nb, nc], [l1, l2], E)


def _fold_train(cam, nF, x_e, d0, normals, lengths, E):
    q = cam.C * x_e
    nF = _facing(nF, d0)
    if nF[2] > 0.0:
        raise ValueError("fold glass would hang into the bottom half")
    edge = np.cross(nF, Z)
    if np.linalg.norm(edge) < 1e-3:
        raise ValueError("horizontal fold")
    e1 = unit(edge)
    if np.cross(nF, e1)[2] < 0:
        e1 = -e1
    fold = Plane("fold", q, nF, e1)
    t, P, *_, ok = fold.hit(np.zeros((1, 3)), d0[None])
    if not ok[0] or P[0][2] <= 0:
        raise ValueError("centre ray misses the fold")
    d1 = reflect(d0, nF)
    planes, d_out = _chain(P[0], d1, normals, lengths)
    if np.dot(d_out, E) < 0.999:
        raise ValueError("construction drifted")
    allp = [fold] + planes
    aps = [((-BIG, 0.0), (BIG, BIG))] + [((-BIG, -BIG), (BIG, BIG))] * len(planes)
    return Train(cam, allp, aps, [GLASS_T] * len(allp), None, False)


def build_prism3(cam, a, rho, azP, elP, beta, l1, l2, toe, v_c, s, leg=25.0, length=25.0):
    """Horizontal-apex prism + a free mirror P + a reflection pair: FOUR
    reflections per eye, R proper -- room for a pitch fold and two sideways
    folds that each bend the full-width fan by >100 deg."""
    return build_prism2(cam, a, rho, beta, l1, toe, v_c, s, leg, length,
                        extra=(dir_az_el(azP, elP), l2))


def build_prism2(cam, a, rho, beta, l1, toe, v_c, s, leg=25.0, length=25.0, extra=None):
    """Horizontal-apex knife-edge prism (the top leg sends the top half up)
    + two mirrors. R improper."""
    C, S, A = cam.C, cam.S, cam.A
    R, d0, E = target_R(cam, v_c, toe, s, (s if extra is not None else -s))
    r = math.radians(rho)
    cp = C * math.cos(r) + A * math.sin(r)
    apex = C * a
    n_up = unit(A - cp)
    out_up = unit(A + cp)
    e1 = S if np.dot(np.cross(n_up, S), out_up) > 0 else -S
    leg_p = Plane("leg", apex, n_up, e1)
    t, P, *_, ok = leg_p.hit(np.zeros((1, 3)), d0[None])
    if not ok[0]:
        raise ValueError("centre ray misses the leg")
    d1 = reflect(d0, n_up)
    if extra is None:
        na, nb = reflection_pair(R @ M(n_up), math.radians(beta))
        planes, d_out = _chain(P[0], d1, [na, nb], [l1])
    else:
        nP, l2 = extra
        na, nb = reflection_pair(R @ M(n_up) @ M(nP), math.radians(beta))
        planes, d_out = _chain(P[0], d1, [nP, na, nb], [l1, l2])
    if np.dot(d_out, E) < 0.999:
        raise ValueError("construction drifted")
    n_dn = unit(-A - cp)
    prism = Convex("prism", [(n_up, float(np.dot(n_up, apex))), (n_dn, float(np.dot(n_dn, apex))),
                             (cp, float(np.dot(cp, apex) + leg / math.sqrt(2))),
                             (S, float(np.dot(S, apex) + length / 2)),
                             (-S, float(-np.dot(S, apex) + length / 2))])
    allp = [leg_p] + planes
    aps = [((-length / 2, 0.0), (length / 2, leg))] + [((-BIG, -BIG), (BIG, BIG))] * len(planes)
    return Train(cam, allp, aps, [0.0] + [GLASS_T] * len(planes), prism, True)


# ---------------------------------------------------------------- evaluation
AZ_RNG, AZ_STEP = (-80.0, 100.0), 1.5
EL_STEP = 1.0


def _grid():
    az = np.arange(AZ_RNG[0], AZ_RNG[1] + 1e-9, AZ_STEP)
    el = np.arange(-V_BAND / 2 - 2, V_BAND / 2 + 2 + 1e-9, EL_STEP)
    return az, el


def fit_stock(train: Train, stock):
    """Pass 1 (apertures unbounded except the prism's, no glass): the best
    band; then each mirror's stock rectangle is centred on that field's
    footprint, in whichever orientation fits it better. stock: per plane
    after the prism (or incl. the fold) a (L, H) or None = unbounded."""
    az, el = _grid()
    AZ, EL, Dc, on = eye_grid(train, az, el)
    ok, hits = train.trace(Dc, apertures=True, obstacles=train.obstacles(with_glass=False))
    mask = (on & ok).reshape(len(el), len(az))
    r = best_band(mask, az, el)
    if r is None:
        return None
    sel = ((AZ >= r[1]) & (AZ <= r[2]) & (np.abs(EL) <= V_BAND / 2)).ravel() & on & ok
    if sel.sum() < 4:
        return None
    first = 1 if train.splitter_is_leg else 0
    for i in range(first, len(train.planes)):
        size = stock[i - first] if stock is not None else None
        s1, s2 = hits[i][0][sel], hits[i][1][sel]
        if size is None:
            # "unbounded" means just big enough: the glass is exactly the
            # footprint plus the margin (an 800 mm placeholder slab would
            # block every other beam in the head)
            mg = PUPIL_MARGIN + 0.5
            if i == 0 and not train.splitter_is_leg:
                train.apertures[i] = ((s1.min() - mg, 0.0), (s1.max() + mg, s2.max() + mg))
            else:
                train.apertures[i] = ((s1.min() - mg, s2.min() - mg), (s1.max() + mg, s2.max() + mg))
            continue
        L, Hh = size
        best = None
        for (a1, a2) in ((L, Hh), (Hh, L)):
            if i == 0 and not train.splitter_is_leg:      # fold: edge stays at e2 = 0
                c1 = (s1.min() + s1.max()) / 2
                lo, hi = (c1 - a1 / 2, 0.0), (c1 + a1 / 2, a2)
            else:
                c1, c2 = (s1.min() + s1.max()) / 2, (s2.min() + s2.max()) / 2
                lo, hi = (c1 - a1 / 2, c2 - a2 / 2), (c1 + a1 / 2, c2 + a2 / 2)
            over = (max(0, lo[0] - s1.min()) + max(0, s1.max() - hi[0])
                    + max(0, lo[1] - s2.min()) + max(0, s2.max() - hi[1]))
            if best is None or over < best[0]:
                best = (over, lo, hi)
        train.apertures[i] = (best[1], best[2])
    return r


def evaluate(train: Train, stock=None):
    """-> dict(w, lo, hi, base, roll, vp) or None."""
    if abs(train.roll()) > 3.0:
        return None
    if not train.splitter_is_leg and train.planes[0].n[2] > 0.0:
        return None                          # fold glass would dip into the bottom half
    r = fit_stock(train, stock)
    if r is None:
        return None
    az, el = _grid()
    AZ, EL, Dc, on = eye_grid(train, az, el)
    ok, _ = train.trace(Dc, apertures=True, obstacles=train.obstacles(with_glass=True))
    mask = (on & ok).reshape(len(el), len(az))
    r = best_band(mask, az, el)
    if r is None:
        return None
    vp = train.vpupil()
    sel = ((AZ >= r[1]) & (AZ <= r[2]) & (np.abs(EL) <= V_BAND / 2)).ravel() & on & ok
    _, hits = train.trace(Dc[sel], apertures=False)
    pts = []
    for p, (s1, s2) in zip(train.planes, hits):
        pts.append(p.c + np.outer(s1, p.e1) + np.outer(s2, p.e2))
    pts = np.vstack(pts)
    lo3, hi3 = pts.min(axis=0), pts.max(axis=0)
    # both eyes: the partner is the C2 image, so |y| and |z| are symmetric
    return dict(w=r[0], lo=r[1], hi=r[2], base=2 * abs(vp[1]), roll=train.roll(), vp=vp,
                total=2 * r[2], overlap=max(0.0, -2 * r[1]),
                ext_x=(lo3[0], hi3[0]), ext_y=max(abs(lo3[1]), abs(hi3[1])),
                ext_z=max(abs(lo3[2]), abs(hi3[2])))


# ---------------------------------------------------------------- search
def make(family, aft, x, sign=1.0):
    cam = Camera(aft)
    if family == "fold1":
        alpha, x_e, toe, v_c = x
        return build_fold1(cam, alpha, x_e, toe, v_c, sign)
    if family == "fold2":
        azF, elF, x_e, beta, l1, toe, v_c = x
        return build_fold2(cam, azF, elF, x_e, beta, l1, toe, v_c, sign)
    if family == "prism2":
        a, rho, beta, l1, toe, v_c = x
        return build_prism2(cam, a, rho, beta, l1, toe, v_c, sign)
    if family == "prism3":
        a, rho, azP, elP, beta, l1, l2, toe, v_c = x
        return build_prism3(cam, a, rho, azP, elP, beta, l1, l2, toe, v_c, sign)
    if family == "fold3":
        azF, elF, x_e, azA, elA, beta, l1, l2, toe, v_c = x
        return build_fold3(cam, azF, elF, x_e, azA, elA, beta, l1, l2, toe, v_c, sign)
    raise ValueError(family)


BOUNDS = {
    "fold1": [(0, 360), (4, 30), (-20, 50), (0.15, 0.45)],
    "fold2": [(0, 360), (-89, 0), (4, 30), (0, 360), (4, 60), (-20, 50), (0.15, 0.45)],
    "prism2": [(2.5, 6), (-35, 35), (0, 360), (4, 60), (-20, 50), (0.15, 0.45)],
    "prism3": [(2.5, 6), (-35, 35), (0, 360), (-89, 89), (0, 360), (4, 50), (4, 60), (-20, 50), (0.15, 0.45)],
    "fold3": [(0, 360), (-89, 0), (4, 30), (0, 360), (-89, 89), (0, 360), (4, 60), (4, 60), (-20, 50), (0.15, 0.45)],
}
BASE_WEIGHT = 0.05
MAX_HALF_WIDTH = 100.0      # head no wider than the 200 mm shoulders
MAX_HALF_HEIGHT = 45.0      # optics within +-45 mm of the eye plane (a ~100 mm tall head)
MAX_DEPTH = 110.0


def objective(x, family, aft, stock, sign=1.0):
    try:
        tr = make(family, aft, x, sign)
        res = evaluate(tr, stock)
    except (ValueError, np.linalg.LinAlgError, FloatingPointError):
        return 1e3
    if res is None:
        return 1e3
    # where the light lands must fit a head: inside the shoulders, and not
    # a tower -- |z| is the optics' half-height above/below the mid-plane,
    # x the depth (the camera sits at x = 0)
    if res["ext_y"] > MAX_HALF_WIDTH or res["ext_z"] > MAX_HALF_HEIGHT or \
            res["ext_x"][1] - res["ext_x"][0] > MAX_DEPTH:
        return 1e3 - res["w"]
    ctr = (res["lo"] + res["hi"]) / 2
    pen = 2.0 * max(0.0, abs(ctr - 0.24 * res["w"]) - 3.0)     # "balanced" toe-out
    # Tom's order: horizontal field first. Baseline is a tiebreak, capped at
    # 80 mm (past that it is a virtual pupil somewhere outside the head)
    return -(res["w"] + BASE_WEIGHT * min(res["base"], 80.0) - pen)


def search(family, aft, stock=None, sign=1.0, maxiter=120, popsize=18, seed=0, workers=16):
    from scipy.optimize import differential_evolution
    r = differential_evolution(objective, BOUNDS[family], args=(family, aft, stock, sign),
                               maxiter=maxiter, popsize=popsize, seed=seed, polish=False,
                               workers=workers, updating="deferred", tol=1e-6)
    tr = make(family, aft, r.x, sign)
    res = evaluate(tr, stock)
    return r.x, res, tr


def glass_mass(stock, prism_leg=None):
    m = 0.0 if prism_leg is None else 2.51e-3 * prism_leg ** 3 / 2
    for s in stock or ():
        if s is not None:
            m += 2 * 2.5e-3 * s[0] * s[1] * GLASS_T         # both eyes
    return m


# ---------------------------------------------------------------- the study
def ceiling(band, aft=False):
    """Perfect optics: the widest az run the TOP half of the sensor gives an
    eye whose band (height `band`, centred) is level -- the most a top/bottom
    split could ever deliver per eye."""
    cam = Camera(aft)
    best = 0.0
    for v_c in np.linspace(0.15, 0.45, 31):
        R, d0, E = target_R(cam, v_c, 0.0, 1, 1)
        az = np.arange(-90, 90.01, 0.5)
        el = np.arange(-band / 2, band / 2 + 0.01, 0.5)
        AZ, EL = np.meshgrid(az, el)
        Dc = dir_az_el(AZ.ravel(), EL.ravel()) @ R
        z = Dc @ cam.C
        u, v = (Dc @ cam.S) / z, (Dc @ cam.A) / z
        ok = ((z > 0) & (np.abs(u) <= U_MAX) & (v >= 0) & (v <= V_MAX)).reshape(AZ.shape)
        best = max(best, ok.all(axis=0).sum() * 0.5)
    return best


def _sample(args):
    fam, aft, sign, seed, n = args
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        x = [rng.uniform(lo, hi) for lo, hi in BOUNDS[fam]]
        f = objective(x, fam, aft, None, sign)
        if f < 900:
            out.append((f, x))
    return out


def _refine(args):
    from scipy.optimize import minimize
    fam, aft, sign, x0 = args
    bounds = BOUNDS[fam]

    def f(x):
        return objective([min(max(v, lo), hi) for v, (lo, hi) in zip(x, bounds)], fam, aft, None, sign)
    bx, bf = list(x0), f(x0)
    for scale in (0.15, 0.05, 0.02):
        simplex = [bx] + [list(bx) for _ in bx]
        for i, (lo, hi) in enumerate(bounds):
            simplex[i + 1][i] += scale * (hi - lo)
        r = minimize(f, bx, method="Nelder-Mead", options=dict(initial_simplex=np.array(simplex), maxfev=500))
        if r.fun < bf:
            bx, bf = list(r.x), r.fun
    x = [min(max(v, lo), hi) for v, (lo, hi) in zip(bx, bounds)]
    tr = make(fam, aft, x, sign)
    res = evaluate(tr, None)
    return bf, x, res, [(round(h[1][0] - h[0][0]), round(h[1][1] - h[0][1])) for h in tr.apertures]


def study(families=("fold2", "fold3", "prism2", "prism3"), n=48000, keep=12, workers=16):
    """Random sample each family (camera forward/aft, both image flips),
    refine the best seeds with Nelder-Mead, report the best per family.
    Mirrors are UNBOUNDED here (each just big enough) -- the ceiling of each
    layout inside the head envelope, before any stock size is imposed."""
    from concurrent.futures import ProcessPoolExecutor
    rows = []
    for fam in families:
        best = None
        for aft in (False, True):
            for sign in (1.0, -1.0):
                with ProcessPoolExecutor(workers) as ex:
                    seeds = []
                    for o in ex.map(_sample, [(fam, aft, sign, sd, n // workers) for sd in range(workers)]):
                        seeds += o
                seeds.sort(key=lambda t: t[0])
                with ProcessPoolExecutor(workers) as ex:
                    for bf, x, res, glass in ex.map(_refine, [(fam, aft, sign, x) for _, x in seeds[:keep]]):
                        if res is not None and (best is None or bf < best[0]):
                            best = (bf, fam, aft, sign, x, res, glass)
        rows.append(best)
        if best:
            bf, fam_, aft, sign, x, r, glass = best
            print(f"{fam_:7s} ({'aft' if aft else 'fwd'} camera): per eye {r['w']:.1f} deg, total {r['total']:.0f}, "
                  f"overlap {r['overlap']:.0f}, baseline {r['base']:.0f} mm; glass needed {glass}", flush=True)
        else:
            print(f"{fam:7s}: no feasible design", flush=True)
    return rows


if __name__ == "__main__":
    import warnings
    warnings.filterwarnings("ignore")
    for band in (20, 26, 30):
        print(f"ceiling, perfect optics, top/bottom split, {band} deg band: {ceiling(band):.0f} deg per eye")
    study()
