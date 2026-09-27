"""periscope_optics: size the stereo periscope that splits ONE Camera Module 3
(Wide) into two eyes, by tracing rays in 3D -- not by guessing.

The principle. A knife-edge right-angle prism mirror (two coated legs meeting
at a sharp 90 deg apex) sits just in front of the lens with its apex line
across the optical axis. It cuts the image down the middle: the half-field on
one side of the apex hits one leg and is thrown sideways to that side, the
other half to the other side. An outer mirror on each side turns its half
forward again. Two reflections per eye, so each eye's image is a ROTATED --
never mirrored -- view of half the sensor, seen from a virtual pupil out at
the outer mirror. The two virtual pupils are the stereo pair; their
separation is the baseline.

Two ways to hang the camera, both traced here (`Config.kind`):

  AFT  camera looks backward (-X), apex line VERTICAL. The sensor's LONG axis
       is split, so each eye's horizontal field is half of it (and see 1.);
       its vertical field is the sensor's short axis. Every mirror is near
       vertical.
  UP   camera looks up (+Z), apex line fore-aft. The long axis is split
       VERTICALLY: each eye's vertical field is half the long axis and its
       HORIZONTAL field is the sensor's whole short axis. The prism is rolled
       by `rho` about the lateral axis so both half-beams leave it angled
       backward (a V seen from above) -- without that, the outer mirror turns
       the wide fan through only ~90 deg and its forward edge rays arrive at a
       grazing angle, which is what makes a mirror enormous (see 2.).

Facts the trace makes unavoidable -- recorded so nobody re-derives them:

1. THE SPLIT HALF-FIELD IS CAPPED BELOW 45 deg by the prism, not the lens.
   A ray at angle theta off the apex plane only reaches a 45 deg leg while
   theta < 45 (rolled by rho, while u < cos(rho) + v sin(rho), u/v the ray's
   image tangents). No stock knife-edge prism is anything but 90 deg, and
   two flat mirrors butted at the apex put their glass edge thickness in the
   field: a dead band several degrees wide at each eye's nose edge, which is
   exactly where the stereo overlap lives.
2. AN OUTER MIRROR'S SIZE ~ (path length) x (field in that direction), and
   along its plane of incidence divided again by sin(grazing angle). Its
   height carries the out-of-plane field, its length the in-plane field. A
   wider baseline is a longer path, so baseline costs mirror, one for one.

Pinhole model with the published Wide FOV (102 x 67 deg) taken as the edge
tangents; the real lens's barrel distortion puts the edge rays at SMALLER
tangents, so every size here is conservative. The pupil is traced as a
point; PUPIL_MARGIN pads every aperture for the real 1.25 mm pupil
(f 2.75 / f2.2) plus glue slop.

Run:
    .venv/bin/python cad/v6/periscope_optics.py      # the chosen design, both configs compared
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---- Camera Module 3 Wide (Raspberry Pi product brief) --------------------
CAM_HFOV = 102.0            # deg, along the sensor's long axis
CAM_VFOV = 67.0             # deg, along its short axis
U_MAX = math.tan(math.radians(CAM_HFOV / 2))     # 1.235, long-axis edge tangent
V_MAX = math.tan(math.radians(CAM_VFOV / 2))     # 0.662, short-axis edge tangent
CAM_PUPIL_D = 2.75 / 2.2    # 1.25 mm entrance pupil
PUPIL_MARGIN = 1.0          # mm kept clear inside every aperture edge

# Camera Module 3 Wide mechanics (board 25 x 24; lens centred across the
# 25 mm width and midway between the M2 holes, i.e. 8.25 mm below the top
# edge of the 24 mm height -- see dimensions_v6.CAM3_*). A = the board's
# 24 mm axis, +A toward the top edge (away from the ribbon).
CAM_BOARD = (25.0, 24.0, 1.0)         # S x A x thickness
CAM_LENS_A = (-15.75, 8.25)           # board extent along A about the lens axis
CAM_BOARD_BACK = 2.5                  # components behind the board
CAM_LENS_BLOCK = 8.5                  # square lens holder
CAM_LENS_H = 11.4                     # board front -> lens front (12.4 total - 1.0 board)
PUPIL_BELOW_FRONT = 1.5               # the entrance pupil, this far inside the lens front
CAM_FRAME_WALL = 1.6                  # printed frame round the board's edges: exit beams clear it too
# The printed NOSE the camera sled slides into (see head.py): the exit beams
# pass either side of it, so it -- not the bare board -- is the obstacle.
# Along the camera axis it runs from just ahead of the lens front to its
# front wall; full head height (the beams pass BESIDE it, never over it).
NOSE_HW = 15.5                        # outer half-width, |y|
NOSE_C = (-17.0, -1.0)                # extent along C (AFT: x = -C) about the pupil


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def reflect(D, n):
    return D - 2.0 * (D @ n)[..., None] * n


def dir_az_el(az, el):
    a, e = np.radians(az), np.radians(el)
    return np.stack([np.cos(a) * np.cos(e), np.sin(a) * np.cos(e), np.sin(e)], axis=-1)


def az_el(D):
    D = unit(D)
    return np.degrees(np.arctan2(D[..., 1], D[..., 0])), np.degrees(np.arcsin(np.clip(D[..., 2], -1, 1)))


# ---------------------------------------------------------------- primitives
@dataclass
class Plane:
    """A flat mirror face through `c`, unit normal `n` OUT of the reflective
    face, local axes e1 (length) and e2 = n x e1 (height)."""
    name: str
    c: np.ndarray
    n: np.ndarray
    e1: np.ndarray

    @property
    def e2(self):
        return np.cross(self.n, self.e1)

    def hit(self, O, D):
        """Vectorised: (t, P, s1, s2, ok) -- ok False where the ray runs away
        from, or along, the reflective face."""
        den = D @ self.n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((self.c - O) @ self.n) / den
        ok = (den < -1e-12) & (t > 1e-9)
        t = np.where(ok, t, np.inf)
        P = O + np.where(ok, t, 0.0)[..., None] * D      # misses: P = O (masked by ok)
        Q = np.where(ok[..., None], P - self.c, 0.0)
        return t, P, Q @ self.e1, Q @ self.e2, ok


@dataclass
class Convex:
    """A convex solid as half-spaces n.x <= d -- a ray obstacle."""
    name: str
    planes: list

    def blocks(self, O, D, t_max):
        n_r = D.shape[0]
        t0 = np.full(n_r, 1e-6)
        t1 = np.asarray(t_max, float) - 1e-6 + np.zeros(n_r)
        alive = np.ones(n_r, bool)
        for n, d in self.planes:
            den = D @ n
            num = d - O @ n
            par = np.abs(den) < 1e-12
            alive &= ~(par & (num < 0))
            with np.errstate(divide="ignore", invalid="ignore"):
                t = num / den
            t1 = np.where(~par & (den > 0), np.minimum(t1, t), t1)
            t0 = np.where(~par & (den < 0), np.maximum(t0, t), t0)
        return alive & (t0 <= t1)

    def contains(self, P, eps=0.0):
        P = np.atleast_2d(P)
        inside = np.ones(len(P), bool)
        for n, d in self.planes:
            inside &= (P @ n) <= d - eps
        return inside


def box_solid(name, center, axes, lo, hi):
    planes = []
    for ax, a, b in zip(axes, lo, hi):
        ax = unit(ax)
        planes.append((ax, float(np.dot(ax, center) + b)))
        planes.append((-ax, float(-np.dot(ax, center) - a)))
    return Convex(name, planes)


def _corners(solid_box_center, axes, lo, hi):
    out = []
    for i in (0, 1):
        for j in (0, 1):
            for k in (0, 1):
                out.append(solid_box_center + axes[0] * (lo[0], hi[0])[i]
                           + axes[1] * (lo[1], hi[1])[j] + axes[2] * (lo[2], hi[2])[k])
    return np.array(out)


# ---------------------------------------------------------------- the design
@dataclass
class Config:
    kind: str = "UP"           # "UP" or "AFT"
    rho: float = 25.0          # UP: prism roll about the lateral axis (deg) -> V-bias
    tau: float = 0.0           # camera (and prism with it) pitched about the lateral axis (deg)
    a: float = 6.0             # pupil -> apex point, along the camera axis (mm)
    prism_offset: float = 0.0  # slide the prism along its apex line (mm)
    prism_leg: float = 25.0    # stock knife-edge prism: leg x length
    prism_len: float = 25.0
    ell: float = 25.0          # prism hit -> outer mirror, along the centre ray (mm)
    toe_out: float = 12.0      # eye centre azimuth, deg outward
    el0: float = -8.0          # eye centre elevation, deg
    u_c: float = 0.30          # camera split-axis tangent sent to the eye centre
    v_c: float = 0.0           # camera short-axis tangent sent to the eye centre
    mirror_L: float = 50.0     # stock outer mirror, length (horizontal) x height
    mirror_H: float = 50.0
    mirror_t: float = 3.0
    mirror_off: tuple = (0.0, 0.0)   # outer mirror centre, offset in its own plane
    leg_setback: float = 0.0   # DIY splitter from two flat mirrors: the mirror that yields at the
                               # apex starts this far along its leg (= the other's glass thickness)
    v_back_t: float = 0.0      # DIY splitter from two BACK-silvered tiles: glass this thick in
                               # front of each silver leg. Both reach the apex (the glass is outside
                               # the V), but a ray must enter through the tile's face, not the cut
                               # end at the apex -- the notch between the two ends is dead
    # built
    C: np.ndarray = field(default=None, repr=False)
    S: np.ndarray = field(default=None, repr=False)
    A: np.ndarray = field(default=None, repr=False)

    # ---------------------------------------------------------- geometry
    def build(self):
        if self.kind == "UP":
            self.C, self.S, self.A = np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0])
            r = math.radians(self.rho)
            self.pc = self.C * math.cos(r) + self.A * math.sin(r)       # prism bisector (into glass)
            self.pa = self.A * math.cos(r) - self.C * math.sin(r)       # apex line
        elif self.kind == "AFT":
            self.C, self.S, self.A = np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
            self.pc, self.pa = self.C.copy(), self.A.copy()
        else:
            raise ValueError(self.kind)
        if self.tau:
            t = math.radians(self.tau)
            Ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
            self.C, self.A, self.pc, self.pa = Ry @ self.C, Ry @ self.A, Ry @ self.pc, Ry @ self.pa
        self.apex = self.C * self.a + self.pa * self.prism_offset      # apex-line centre
        n_leg = unit(self.S - self.pc)
        out_leg = unit(self.S + self.pc)
        e1 = self.pa if np.dot(np.cross(n_leg, self.pa), out_leg) > 0 else -self.pa
        self.leg = Plane("prism_leg", self.apex, n_leg, e1)
        # the eye's centre ray fixes the outer mirror's orientation and place
        d0 = unit(self.C + self.u_c * self.S + self.v_c * self.A)
        t, P, *_ , ok = self.leg.hit(np.zeros((1, 3)), d0[None])
        if not ok[0]:
            raise ValueError("centre ray misses the prism leg")
        d1 = reflect(d0, n_leg)
        E = dir_az_el(self.toe_out, self.el0)
        n2 = unit(E - d1)
        c2 = P[0] + d1 * self.ell
        e1o = np.cross([0, 0, 1.0], n2)
        e1o = unit(e1o) if np.linalg.norm(e1o) > 1e-6 else np.array([1.0, 0, 0])
        e2o = np.cross(n2, e1o)
        c2 = c2 + e1o * self.mirror_off[0] + e2o * self.mirror_off[1]
        self.outer = Plane("outer", c2, n2, e1o)
        return self

    def R(self):
        M1 = np.eye(3) - 2 * np.outer(self.leg.n, self.leg.n)
        M2 = np.eye(3) - 2 * np.outer(self.outer.n, self.outer.n)
        return M2 @ M1

    def virtual_pupil(self):
        p = np.zeros(3)
        p = p - 2 * np.dot(p - self.leg.c, self.leg.n) * self.leg.n
        p = p - 2 * np.dot(p - self.outer.c, self.outer.n) * self.outer.n
        return p

    def prism_solid(self):
        c, s, a = self.pc, self.S, self.pa
        planes = [(unit(sd * s - c), float(np.dot(unit(sd * s - c), self.apex))) for sd in (1, -1)]
        planes.append((c, float(np.dot(c, self.apex) + self.prism_leg / math.sqrt(2))))
        planes.append((a, float(np.dot(a, self.apex) + self.prism_len / 2)))
        planes.append((-a, float(-np.dot(a, self.apex) + self.prism_len / 2)))
        return Convex("prism", planes)

    def prism_vertices(self):
        h = self.prism_leg / math.sqrt(2)
        v = []
        for sa in (-1, 1):
            base = self.apex + self.pa * sa * self.prism_len / 2
            v += [base, base + self.pc * h + self.S * h, base + self.pc * h - self.S * h]
        return np.array(v)

    def camera_solids(self):
        """Board (with its back components) and the lens block, in the
        camera frame about the pupil -- and, for AFT, the printed nose
        they sit in."""
        C, S, A = self.C, self.S, self.A
        lens_front = PUPIL_BELOW_FRONT
        board_front = lens_front - CAM_LENS_H
        w = CAM_FRAME_WALL
        board = box_solid("cam_board", np.zeros(3), (C, S, A),
                          (board_front - CAM_BOARD[2] - CAM_BOARD_BACK - w, -CAM_BOARD[0] / 2 - w, CAM_LENS_A[0] - w),
                          (board_front, CAM_BOARD[0] / 2 + w, CAM_LENS_A[1] + w))
        lens = box_solid("cam_lens", np.zeros(3), (C, S, A),
                         (board_front, -CAM_LENS_BLOCK / 2, -CAM_LENS_BLOCK / 2),
                         (lens_front, CAM_LENS_BLOCK / 2, CAM_LENS_BLOCK / 2))
        out = [board, lens]
        if self.kind == "AFT":
            out.append(box_solid("nose", np.zeros(3), (C, S, A),
                                 (NOSE_C[0], -NOSE_HW, -80.0), (NOSE_C[1], NOSE_HW, 80.0)))
        return out

    def mirror_glass(self, side, lo, hi):
        """The outer mirror's glass slab (aperture lo..hi in its local
        coords) as an obstacle, mirrored to side -1 on request."""
        m = self.outer
        ctr = m.c + m.e1 * (lo[0] + hi[0]) / 2 + m.e2 * (lo[1] + hi[1]) / 2 - m.n * self.mirror_t / 2
        axes = [m.e1, m.e2, m.n]
        half = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, self.mirror_t / 2 - 0.05]
        if side < 0:
            F = np.diag([1.0, -1.0, 1.0])
            ctr, axes = F @ ctr, [F @ ax for ax in axes]
        return box_solid(f"glass{side:+d}", ctr, axes, [-h for h in half], half)

    def prism_clears_camera(self):
        """Solid-vs-solid, sampled: no prism vertex/edge point inside the
        lens block or board, and no lens-block corner inside the prism."""
        pv = self.prism_vertices()
        pts = [pv]
        for i in range(len(pv)):
            for j in range(i + 1, len(pv)):
                pts.append(pv[i] + (pv[j] - pv[i]) * np.linspace(0, 1, 12)[:, None])
        pts = np.vstack(pts)
        for cs in self.camera_solids():
            if cs.contains(pts, eps=-0.3).any():
                return False
        C, S, A = self.C, self.S, self.A
        lens_front = PUPIL_BELOW_FRONT
        lc = _corners(np.zeros(3), (C, S, A),
                      (lens_front - CAM_LENS_H, -CAM_LENS_BLOCK / 2, -CAM_LENS_BLOCK / 2),
                      (lens_front, CAM_LENS_BLOCK / 2, CAM_LENS_BLOCK / 2))
        return not self.prism_solid().contains(lc, eps=-0.3).any()

    # ---------------------------------------------------------- tracing
    def trace_dirs(self, Dc):
        """Trace camera directions Dc (N,3) through the +S side with NO
        aperture limits; returns a dict of per-ray arrays: leg/outer local
        coords, exit directions, and a `clear` mask for obstacles that do not
        depend on the apertures (prism body, camera)."""
        N = len(Dc)
        O = np.zeros((N, 3))
        t1, P1, a1, b1, ok1 = self.leg.hit(O, Dc)
        if self.v_back_t > 0:
            face = Plane("tile_face", self.leg.c + self.leg.n * self.v_back_t, self.leg.n, self.leg.e1)
            _, _, _, bf, okf = face.hit(O, Dc)
            ok1 = ok1 & okf & (bf >= 0.0)            # enters through the face, not the cut end
        D1 = reflect(Dc, self.leg.n)
        t2, P2, a2, b2, ok2 = self.outer.hit(P1, D1)
        ok = ok1 & ok2
        D2 = reflect(D1, self.outer.n)
        clear = ok.copy()
        prism = self.prism_solid()
        big = 300.0
        # lateral leg must not pass through the prism body or the camera
        P1n = np.where(ok[:, None], P1 + D1 * 0.05, 0.0)
        tl = np.where(ok, t2 - 0.1, 0.0)
        clear &= ~prism.blocks(P1n, D1, tl)
        P2n = np.where(ok[:, None], P2 + D2 * 0.05, 0.0)
        clear &= ~prism.blocks(P2n, D2, big)
        for cs in self.camera_solids():
            clear &= ~cs.blocks(P1n, D1, tl)
            clear &= ~cs.blocks(P2n, D2, big)
        return dict(ok=ok, clear=clear, leg=(a1, b1), outer=(a2, b2), P1=P1, P2=P2,
                    D1=D1, D2=D2, t2=t2)

    def eye_grid(self, az_rng=(-40, 70), el_rng=(-45, 40), step=1.0):
        az = np.arange(az_rng[0], az_rng[1] + 1e-9, step)
        el = np.arange(el_rng[0], el_rng[1] + 1e-9, step)
        AZ, EL = np.meshgrid(az, el, indexing="xy")          # rows = el, cols = az
        E = dir_az_el(AZ.ravel(), EL.ravel())
        Dc = E @ self.R()                                     # R^T E, row-wise
        z = Dc @ self.C
        with np.errstate(divide="ignore", invalid="ignore"):
            u, v = (Dc @ self.S) / z, (Dc @ self.A) / z
        on_sensor = (z > 0) & (u >= 0) & (u <= U_MAX) & (np.abs(v) <= V_MAX)
        tr = self.trace_dirs(unit(Dc))
        return az, el, on_sensor, tr

    def valid_mask(self, grid, prism_lo=None, prism_hi=None, mirror_lo=None, mirror_hi=None,
                   other_side=True, m=PUPIL_MARGIN):
        """Apply stock apertures to a traced grid. Apertures default to the
        stock sizes centred where build() put them."""
        az, el, on_sensor, tr = grid
        if prism_lo is None:
            prism_lo, prism_hi = (-self.prism_len / 2, 0.0), (self.prism_len / 2, self.prism_leg)
        if mirror_lo is None:
            mirror_lo = (-self.mirror_L / 2, -self.mirror_H / 2)
            mirror_hi = (self.mirror_L / 2, self.mirror_H / 2)
        a1, b1 = tr["leg"]
        a2, b2 = tr["outer"]
        ok = on_sensor & tr["clear"]
        # NO margin at the knife edge (b1 = 0): the apex IS the split. A
        # pencil straddling it is shared by both eyes -- a brightness ramp
        # over atan(pupil radius / a) either side of the seam, flat-fielded
        # out, not a cutoff.
        ok &= (a1 >= prism_lo[0] + m) & (a1 <= prism_hi[0] - m) & (b1 >= prism_lo[1] + self.leg_setback) & (b1 <= prism_hi[1] - m)
        ok &= (a2 >= mirror_lo[0] + m) & (a2 <= mirror_hi[0] - m) & (b2 >= mirror_lo[1] + m) & (b2 <= mirror_hi[1] - m)
        if other_side:
            g = self.mirror_glass(-1, mirror_lo, mirror_hi)
            P1n = tr["P1"] + tr["D1"] * 0.05
            P2n = tr["P2"] + tr["D2"] * 0.05
            okm = ok.copy()
            idx = np.where(okm)[0]
            if len(idx):
                bl = g.blocks(P1n[idx], tr["D1"][idx], tr["t2"][idx]) | g.blocks(P2n[idx], tr["D2"][idx], 300.0)
                ok[idx[bl]] = False
            gs = self.mirror_glass(+1, mirror_lo, mirror_hi)
            idx = np.where(ok)[0]
            if len(idx):
                # this side's own glass can only be hit on the way OUT if the exit
                # ray wraps round the mirror edge -- check anyway
                bl = gs.blocks(P2n[idx], tr["D2"][idx], 300.0)
                ok[idx[bl]] = False
        return ok.reshape(len(el), len(az))


def best_rect(mask, az, el, v_min):
    """Widest az range fully valid over some el window of height >= v_min.
    Returns (w_az, az_lo, az_hi, el_lo, el_hi) or None."""
    step = el[1] - el[0]
    k = int(round(v_min / step)) + 1
    best = None
    for i in range(0, mask.shape[0] - k + 1):
        cols = mask[i:i + k].all(axis=0)
        # longest run of True
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
        if bl[0] > 1:
            w = az[bl[2]] - az[bl[1]]
            if best is None or w > best[0]:
                best = (w, az[bl[1]], az[bl[2]], el[i], el[i + k - 1])
    return best


def best_az(mask, az, el, el_lo, el_hi):
    """Widest az run fully valid over the FIXED elevation window
    el_lo..el_hi. Returns (w_az, az_lo, az_hi, el_lo, el_hi) or None."""
    rows = (el >= el_lo - 1e-9) & (el <= el_hi + 1e-9)
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
    return az[bl[2]] - az[bl[1]], az[bl[1]], az[bl[2]], el_lo, el_hi


def aperture_fit(grid, rect, cfg):
    """The prism-leg and outer-mirror footprints of the rays inside `rect`
    (az_lo, az_hi, el_lo, el_hi) -- what each part must actually cover."""
    az, el, on_sensor, tr = grid
    AZ, EL = np.meshgrid(az, el, indexing="xy")
    sel = ((AZ >= rect[0]) & (AZ <= rect[1]) & (EL >= rect[2]) & (EL <= rect[3])).ravel()
    sel &= tr["ok"]
    a1, b1 = tr["leg"]
    a2, b2 = tr["outer"]
    return ((a1[sel].min(), a1[sel].max(), b1[sel].min(), b1[sel].max()),
            (a2[sel].min(), a2[sel].max(), b2[sel].min(), b2[sel].max()))


def evaluate(cfg, v_min, step=1.0, el_win=None):
    """Build, trace, and centre the stock outer mirror on the footprint of
    the field the stock PRISM passes; return (result dict). With `el_win`
    = (el_lo, el_hi) the vertical window is fixed rather than free."""
    pick = (lambda m, g: best_az(m, g[0], g[1], *el_win)) if el_win else \
        (lambda m, g: best_rect(m, g[0], g[1], v_min))
    cfg.build()
    if not cfg.prism_clears_camera():
        return None
    grid = cfg.eye_grid(step=step)
    # pass 1: prism aperture only (outer mirror unbounded)
    big = (-1e3, -1e3), (1e3, 1e3)
    mask = cfg.valid_mask(grid, mirror_lo=big[0], mirror_hi=big[1], other_side=False)
    r = pick(mask, grid)
    if r is None:
        return None
    # centre the stock mirror on the footprint of that field, then re-trace
    _, fo = aperture_fit(grid, r[1:], cfg)
    off = ((fo[0] + fo[1]) / 2, (fo[2] + fo[3]) / 2)
    cfg.mirror_off = (cfg.mirror_off[0] + off[0], cfg.mirror_off[1] + off[1])
    cfg.build()
    grid = cfg.eye_grid(step=step)
    mask = cfg.valid_mask(grid)
    r = pick(mask, grid)
    if r is None:
        return None
    w, az_lo, az_hi, el_lo, el_hi = r
    vp = cfg.virtual_pupil()
    return dict(w=w, az_lo=az_lo, az_hi=az_hi, el_lo=el_lo, el_hi=el_hi,
                total=2 * az_hi, overlap=max(0.0, -2 * az_lo),
                baseline=2 * abs(vp[1]), vp=vp, cfg=cfg)


def describe(res, title=""):
    c = res["cfg"]
    o = c.outer
    print(f"-- {title}")
    print(f"   per eye {res['w']:.0f} deg H (az {res['az_lo']:+.0f}..{res['az_hi']:+.0f}) x "
          f"{res['el_hi'] - res['el_lo']:.0f} deg V (el {res['el_lo']:+.0f}..{res['el_hi']:+.0f})")
    print(f"   TOTAL {res['total']:.0f} deg, stereo overlap {res['overlap']:.0f} deg, "
          f"baseline {res['baseline']:.1f} mm")
    print(f"   virtual pupil (x, y, z) = ({res['vp'][0]:.1f}, {res['vp'][1]:.1f}, {res['vp'][2]:.1f}) mm")
    print(f"   outer mirror {c.mirror_L:.0f} x {c.mirror_H:.0f} centre "
          f"({o.c[0]:.1f}, {o.c[1]:.1f}, {o.c[2]:.1f}) normal ({o.n[0]:.2f}, {o.n[1]:.2f}, {o.n[2]:.2f})")


# ---------------------------------------------------------------- design search
# What "best" means, in the user's order (2026-09-24): (1) horizontal field
# maximised, (2) toe-out "balanced" -- overlap about a third of the total, (3)
# baseline as wide as fits under the shoulders. (1) and (3) fight: a wider
# baseline is a longer path, so a bigger footprint on the same stock mirror.
# The exchange rate used: 1 deg of per-eye field is worth 10 mm of baseline.
EL_WINDOW = (-30.0, 10.0)      # eye vertical field: floor from 0.9 m at the
                               # 0.53 m eye height, and 10 deg above the horizon
CTR_RANGE = (8.5, 10.5)        # eye centre azimuth -> overlap ~ 35 % of total
BASE_WEIGHT = 0.1              # deg of field per mm of baseline
SIMPLE_FIXED = dict(kind="AFT", rho=0.0, tau=0.0, v_c=0.0)
SIMPLE_RANGES = dict(a=(2.5, 4.0), prism_offset=(-8.0, 8.0), ell=(20.0, 55.0),
                     toe_out=(-5.0, 18.0), el0=(-20.0, 0.0), u_c=(0.15, 0.6))


def objective(res):
    if res is None:
        return -1e9
    ctr = (res["az_lo"] + res["az_hi"]) / 2
    pen = 0.0
    if not (CTR_RANGE[0] <= ctr <= CTR_RANGE[1]):
        pen = 5.0 * min(abs(ctr - CTR_RANGE[0]), abs(ctr - CTR_RANGE[1]))
    return res["w"] + BASE_WEIGHT * res["baseline"] - pen


def _eval_params(p, stock, step=1.0):
    try:
        return evaluate(Config(**SIMPLE_FIXED, **stock, **p), 0.0, step=step, el_win=EL_WINDOW)
    except (ValueError, np.linalg.LinAlgError):
        return None


def refine(p, stock, iters=6):
    """Coordinate descent from p on the SIMPLE (all-vertical) family."""
    best = dict(p)
    fbest = objective(_eval_params(best, stock))
    steps = dict(a=0.25, prism_offset=1.0, ell=2.0, toe_out=1.0, el0=1.0, u_c=0.03)
    for _ in range(iters):
        improved = False
        for k, h in steps.items():
            for sgn in (1, -1):
                q = dict(best)
                q[k] = float(np.clip(q[k] + sgn * h, *SIMPLE_RANGES[k]))
                f = objective(_eval_params(q, stock))
                if f > fbest + 1e-6:
                    best, fbest, improved = q, f, True
        if not improved:
            steps = {k: v / 2 for k, v in steps.items()}
    return best, fbest


def search(stock, n=4000, seed=0, top=6):
    """Random sample the SIMPLE family, then refine the best few."""
    rng = np.random.default_rng(seed)
    cands = []
    for _ in range(n):
        p = {k: float(rng.uniform(*r)) for k, r in SIMPLE_RANGES.items()}
        cands.append((objective(_eval_params(p, stock)), p))
    cands.sort(key=lambda t: -t[0])
    out = [refine(p, stock) for _, p in cands[:top]]
    out.sort(key=lambda t: -t[1])
    return out[0]


# ---------------------------------------------------------------- the design
# PRISM-FREE (2026-09-26). Tom, after the prism quotes: "don't use a prism
# then" -- and cheap mirrors, glued in and calibrated, instead of optics-grade
# parts. The split is a V of two FRONT-SURFACE squares glued into a printed
# block. Their glass can't both reach the apex: the LEFT eye's square runs to
# it; the RIGHT eye's butts against the back of the left one, a glass-
# thickness short -- a dead strip at the right eye's OUTER edge (leg_setback;
# the stereo overlap, at the inner edges, is untouched). Both eyes are scored
# on the one physical layout (two_eyes()).
#
# The squares have no manufacturer datasheets (Tom's call for this part):
# their size and thickness are MEASURED ON ARRIVAL and set here, and the
# stereo calibration absorbs the rest.
V_TILE = (25.0, 20.0, 1.1)       # V squares: along the leg, height, glass thickness (front-surface).
                                 # 25 mm along the leg, not 50: a longer piece's far end reaches the
                                 # outer mirror (glass_clash), and the light only uses ~20 mm of it
OUTER_TILE = (50.0, 50.0, 1.1)   # outer squares: length, height, glass thickness
# The parts (2026-09-26), all front-surface 1.1 mm optical glass, no datasheets
# (Tom's call: cheap, glued in, calibrated), measured on arrival:
#   outer  5 x 50 x 50 -- eBay hnpiwxny "Front Surface Projector Reflector
#          Mirror", $16.34 delivered (2 used, 3 spare)
#   V      5 x 25 x 20 -- eBay explore-space, $12.66 delivered (2 used)
# No cutting. (Cutting a spare 50 x 50 into 25 x 25 quarters instead traces
# 2 deg wider -- 48 vs 46 total -- for $0: set V_TILE = (25.0, 25.0, 1.1).)
OUTER_FRONT = True               # front-surface outer squares (False: back-silvered craft tiles)
STOCK = dict(prism_leg=V_TILE[0], prism_len=V_TILE[1], mirror_L=OUTER_TILE[0], mirror_H=OUTER_TILE[1],
             mirror_t=OUTER_TILE[2])
SETBACK = V_TILE[2]              # the right eye's square starts this far from the apex
DESIGN = dict(a=3.166, prism_offset=-1.98, ell=34.551, toe_out=4.136, el0=-6.977, u_c=0.505)   # --search, 2026-09-26 (the eBay parts)
V_RANGES = dict(SIMPLE_RANGES, a=(2.5, 12.0))


def two_eyes(p, stock=None, setback=None, step=1.0):
    """Place the outer mirrors once (centred on the left eye's field), then
    score BOTH eyes on that layout: the left eye on the apex square, the
    right on the set-back one. Returns None if either eye has no field."""
    stock = STOCK if stock is None else stock
    setback = SETBACK if setback is None else setback
    try:
        r0 = evaluate(Config(**SIMPLE_FIXED, **stock, **p), 0.0, step=step, el_win=EL_WINDOW)
    except (ValueError, np.linalg.LinAlgError):
        return None
    if r0 is None:
        return None
    cfg = r0["cfg"]
    if glass_clash(cfg):
        return None
    out = {}
    for name, sb in (("left", 0.0), ("right", setback)):
        cfg.leg_setback = sb
        g = cfg.eye_grid(step=step)
        r = best_az(cfg.valid_mask(g), g[0], g[1], *EL_WINDOW)
        if r is None:
            return None
        out[name] = dict(w=r[0], az_lo=r[1], az_hi=r[2])
    cfg.leg_setback = 0.0
    vp = cfg.virtual_pupil()
    L, R_ = out["left"], out["right"]
    out.update(cfg=cfg, total=L["az_hi"] + R_["az_hi"], overlap=2 * min(-L["az_lo"], -R_["az_lo"]),
               baseline=2 * abs(vp[1]), vp=vp, el_lo=EL_WINDOW[0], el_hi=EL_WINDOW[1])
    return out


def glass_clash(cfg, clear=1.0):
    """Solid against solid, which the ray trace never looks at: the V's
    squares (both, the right one mirrored and set back) and the outer mirrors'
    glass must not overlap, with `clear` mm to spare for their mounts."""
    V_pts = []
    for side, s0 in ((1, 0.0), (-1, SETBACK)):
        for sa in (s0, s0 + cfg.prism_leg):
            for su in (-cfg.prism_len / 2, cfg.prism_len / 2):
                for dep in (0.0, -V_TILE[2]):
                    q = cfg.leg.c + cfg.leg.e2 * sa + cfg.leg.e1 * su + cfg.leg.n * dep
                    V_pts.append(q if side > 0 else q * np.array([1, -1, 1]))
    V_pts = np.array(V_pts)
    lo, hi = (-cfg.mirror_L / 2, -cfg.mirror_H / 2), (cfg.mirror_L / 2, cfg.mirror_H / 2)
    for side in (1, -1):
        g = cfg.mirror_glass(side, (lo[0] - clear, lo[1] - clear), (hi[0] + clear, hi[1] + clear))
        if g.contains(V_pts).any():
            return True
        # and the other way round: an outer glass corner inside the V's footprint
    corners = []
    o = cfg.outer
    for a1 in (lo[0], hi[0]):
        for a2 in (lo[1], hi[1]):
            for dep in (0.0, -cfg.mirror_t):
                corners.append(o.c + o.e1 * a1 + o.e2 * a2 + o.n * dep)
    tri = cfg.prism_solid()
    return bool(tri.contains(np.array(corners), eps=-clear).any())


def v_objective(res):
    """Total field (the right eye pays the seam), a real stereo zone, some baseline."""
    if res is None:
        return -1e9
    f = res["total"] + 0.05 * min(res["baseline"], 70.0)
    if res["overlap"] < 16.0:
        f -= 3.0 * (16.0 - res["overlap"])
    return f


def v_search(stock=None, setback=None, n=1500, seed=0):
    rng = np.random.default_rng(seed)
    best = (-1e9, None)
    for _ in range(n):
        p = {k: float(rng.uniform(*r)) for k, r in V_RANGES.items()}
        f = v_objective(two_eyes(p, stock, setback))
        if f > best[0]:
            best = (f, p)
    f, p = best
    steps = dict(a=0.5, prism_offset=1.0, ell=2.0, toe_out=1.0, el0=1.0, u_c=0.03)
    for _ in range(6):
        improved = False
        for k, h in steps.items():
            for sg in (1, -1):
                q = dict(p)
                q[k] = float(np.clip(q[k] + sg * h, *V_RANGES[k]))
                g = v_objective(two_eyes(q, stock, setback))
                if g > f:
                    p, f, improved = q, g, True
        if not improved:
            steps = {k: v / 2 for k, v in steps.items()}
    return p, f


def _v_search_job(seed):
    import warnings
    warnings.filterwarnings("ignore")
    return v_search(seed=seed)


def design():
    """The chosen layout, both eyes -- the single source for the CAD."""
    return two_eyes(DESIGN, step=0.5)


def describe_v(res, title="DESIGN"):
    L, R_ = res["left"], res["right"]
    o = res["cfg"].outer
    print(f"-- {title}")
    print(f"   left eye {L['w']:.0f} deg (az {L['az_lo']:+.0f}..{L['az_hi']:+.0f}), right eye {R_['w']:.0f} deg "
          f"(the seam's strip), both {res['el_hi'] - res['el_lo']:.0f} deg tall (el {res['el_lo']:+.0f}..{res['el_hi']:+.0f})")
    print(f"   TOTAL {res['total']:.0f} deg, stereo overlap {res['overlap']:.0f} deg, baseline {res['baseline']:.1f} mm")
    print(f"   V apex {res['cfg'].a:.2f} mm behind the pupil; outer mirror centre "
          f"({o.c[0]:.1f}, {o.c[1]:.1f}, {o.c[2]:.1f}) normal ({o.n[0]:.2f}, {o.n[1]:.2f}, {o.n[2]:.2f})")


if __name__ == "__main__":
    import sys
    import warnings
    warnings.filterwarnings("ignore")
    if "--search" in sys.argv:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(8) as ex:
            runs = list(ex.map(_v_search_job, range(8)))
        p, f = max(runs, key=lambda t: t[1])
        print("best:", {k: round(v, 3) for k, v in p.items()}, "objective", round(f, 2))
        describe_v(two_eyes(p, step=0.5), "searched")
    else:
        describe_v(design())
