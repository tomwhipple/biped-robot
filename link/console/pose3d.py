"""The 3D pose view's MuJoCo half (design §5): pose the plant from bus joint
angles, build the scene with its ghosts, render offscreen. No Qt here, so the
mapping and the ghost rules test without a display or a GL context.

Kinematics only: every pose goes through mj_kinematics. Nothing here calls
mj_step or mj_forward, and every MjData belongs to this module and the GUI
thread, so the GUI-vs-physics MjData race in AGENTS.md cannot happen.

  * Joints map BY NAME between the bus (protocol.JOINT_NAMES, bus order) and
    the plant; a bus joint the plant lacks is skipped, a plant joint the bus
    lacks stays at the plant's zero pose.
  * The root is pinned: x, y at 0, yaw drawn 0, attitude from the IMU's up
    vector (the shortest rotation taking world +Z onto it, as sil_twin's
    mirror viewer does), and the measured body lowered until its lowest point
    touches the floor. The robot has no odometry, so no position is drawn.
  * Ghosts are separate geometry: a second MjData posed with the measured
    root, added to the scene with mjv_addGeoms and its rgba overridden --
    never a pixel blend of two renders. A ghost draws only the links whose
    every joint up to the torso has a ghost value, so a joint with no goal
    draws no ghost (and the torso, which no joint moves, is never ghosted).
  * A link downstream of a joint that was not measured (not fitted, no
    reading, feedback failed) is dimmed; an unmeasured joint sits at the
    plant's zero pose, qpos0 (its `ref`: the arms' rest pose as drawn).
"""
from __future__ import annotations

import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LINK_DIR = os.path.dirname(HERE)
ROOT = os.path.dirname(LINK_DIR)
if LINK_DIR not in sys.path:
    sys.path.insert(0, LINK_DIR)

from protocol import JOINT_NAMES  # noqa: E402  link/protocol.py, bus order

PLANTS = ("bimo_biped_v6ar.xml", "bimo_biped_v6legs.xml")
DEFAULT_PLANT = os.path.join(ROOT, "sim", PLANTS[0])

GROUPS = (0, 2)                         # floor + CAD meshes (the mockups' look)
CMD_RGBA = (0.24, 0.78, 1.0, 0.24)      # ui.CYAN
SIM_RGBA = (1.0, 0.64, 0.12, 0.26)      # ui.SIMC
DIM_RGBA = (0.45, 0.47, 0.5, 0.45)
SEL_RGBA = (0.86, 0.36, 0.95, 1.0)      # the selected joint's link
MAX_PX = 2048


def resolve_plant(path: str | None, repo: str | None = None) -> str:
    """--plant as given, else the v6 plant. A bare file name is looked up in
    <repo>/sim, so `--plant bimo_biped_v6legs.xml` works from anywhere."""
    repo = repo or ROOT
    if not path:
        return os.path.join(repo, "sim", PLANTS[0])
    for cand in (path, os.path.join(repo, path), os.path.join(repo, "sim", path)):
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    return path


def up_to_quat(up) -> np.ndarray:
    """Torso orientation (wxyz) from the up vector: the torso's z axis in the
    world frame (obs kOffUp, MuJoCo framezaxis). The shortest rotation taking
    world +Z onto it -- roll and pitch, yaw drawn 0 (sim/sil_twin.py)."""
    if up is None:
        return np.array([1.0, 0.0, 0.0, 0.0])
    v = np.asarray(up, dtype=np.float64)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    v = v / n
    axis = np.cross((0.0, 0.0, 1.0), v)
    sn = float(np.linalg.norm(axis))
    c = float(v[2])
    if sn < 1e-9:
        return (np.array([1.0, 0.0, 0.0, 0.0]) if c > 0
                else np.array([0.0, 1.0, 0.0, 0.0]))
    axis = axis / sn
    ang = math.atan2(sn, c)
    return np.concatenate(([math.cos(ang / 2)], math.sin(ang / 2) * axis))


class PoseModel:
    """A plant, its bus-name mapping, and the link sets the layers need."""

    def __init__(self, path: str):
        self.path = path
        self.m = m = mujoco.MjModel.from_xml_path(path)
        J = mujoco.mjtObj.mjOBJ_JOINT
        hinge = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        self.qadr: dict[str, int] = {}          # bus name -> qpos address
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(m, J, name)
            if jid >= 0 and int(m.jnt_type[jid]) in hinge:
                self.qadr[name] = int(m.jnt_qposadr[jid])
        self.missing = [n for n in JOINT_NAMES if n not in self.qadr]
        self.joint_names: list[str] = []        # the plant's own 1-dof joints
        self.root_adr = -1
        self.root_body = 1 if m.nbody > 1 else 0
        for j in range(m.njnt):
            if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
                if self.root_adr < 0:
                    self.root_adr = int(m.jnt_qposadr[j])
                    self.root_body = int(m.jnt_bodyid[j])
            elif int(m.jnt_type[j]) in hinge:
                self.joint_names.append(mujoco.mj_id2name(m, J, j) or f"joint{j}")
        self.extra = [n for n in self.joint_names if n not in JOINT_NAMES]

        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = 0
        for g in GROUPS:
            self.opt.geomgroup[g] = 1
        self.floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.floor_z = float(m.geom_pos[self.floor][2]) if self.floor >= 0 else 0.0
        shown = np.isin(m.geom_group, GROUPS)
        if self.floor >= 0:
            shown[self.floor] = False
        self.body_geoms = np.flatnonzero(shown & (m.geom_bodyid > 0))

        # Each body's 1-dof joints from the torso down, by plant name.
        own: list[list[str]] = [[] for _ in range(m.nbody)]
        for j in range(m.njnt):
            if int(m.jnt_type[j]) in hinge:
                own[int(m.jnt_bodyid[j])].append(mujoco.mj_id2name(m, J, j) or f"joint{j}")
        self.path: list[frozenset] = [frozenset()] * m.nbody
        for b in range(1, m.nbody):           # parents precede children
            self.path[b] = self.path[int(m.body_parentid[b])] | frozenset(own[b])
        self._geom_body = m.geom_bodyid.copy()
        self._dim_cache: dict = {}
        self._ghost_cache: dict = {}

        d = mujoco.MjData(m)
        self.pose(d, {})
        self.look_z = float(d.xpos[self.root_body][2]) * 0.62

    # -- the mapping -------------------------------------------------------
    def plant_joint(self, bus_name: str) -> str | None:
        """The plant joint a bus joint drives (they share the name), or None."""
        return bus_name if bus_name in self.qadr else None

    def bus_joint(self, plant_name: str) -> str | None:
        return plant_name if plant_name in JOINT_NAMES and plant_name in self.qadr else None

    def from_bus(self, values) -> dict[str, float]:
        """A bus-order vector (the beacon's 17 angles) as {bus name: rad},
        only the joints this plant has."""
        return {n: float(v) for n, v in zip(JOINT_NAMES, values) if n in self.qadr}

    def to_bus(self, d: mujoco.MjData) -> list[float | None]:
        """A posed MjData back to a bus-order vector (None: not in the plant)."""
        return [float(d.qpos[self.qadr[n]]) if n in self.qadr else None for n in JOINT_NAMES]

    # -- posing --------------------------------------------------------------
    def pose(self, d: mujoco.MjData, q: dict, up=None, root=None):
        """Pose `d` from {bus name: rad}; every other joint at the plant's zero.
        root=None: the measured body (attitude from `up`, lowest point on the
        floor); else a ghost on that root (pos + wxyz). Returns the root."""
        m = self.m
        d.qpos[:] = m.qpos0
        a = self.root_adr
        if a >= 0:
            if root is None:
                d.qpos[a:a + 3] = (0.0, 0.0, m.qpos0[a + 2])
                d.qpos[a + 3:a + 7] = up_to_quat(up)
            else:
                d.qpos[a:a + 7] = root
        for name, v in q.items():
            adr = self.qadr.get(name)
            if adr is not None and v is not None and math.isfinite(v):
                d.qpos[adr] = v
        mujoco.mj_kinematics(m, d)
        if a < 0:
            return None
        if root is None:
            d.qpos[a + 2] -= self.lowest(d) - self.floor_z
            mujoco.mj_kinematics(m, d)
        return d.qpos[a:a + 7].copy()

    def lowest(self, d: mujoco.MjData) -> float:
        g = self.body_geoms
        if not len(g):
            return self.floor_z
        R = d.geom_xmat[g].reshape(-1, 3, 3)
        aabb = self.m.geom_aabb[g]
        centre = d.geom_xpos[g, 2] + np.einsum("nj,nj->n", R[:, 2, :], aabb[:, :3])
        half = np.einsum("nj,nj->n", np.abs(R[:, 2, :]), aabb[:, 3:])
        return float(np.min(centre - half))

    # -- link sets -----------------------------------------------------------
    def dim_geoms(self, measured) -> frozenset:
        """Geoms downstream of any plant joint not in `measured` (bus names)."""
        key = frozenset(measured)
        hit = self._dim_cache.get(key)
        if hit is None:
            unmeasured = frozenset(self.joint_names) - key
            bodies = [b for b in range(self.m.nbody) if self.path[b] & unmeasured]
            hit = frozenset(np.flatnonzero(np.isin(self._geom_body, bodies)).tolist())
            self._dim_cache[key] = hit
        return hit

    def ghost_geoms(self, have) -> frozenset:
        """Geoms a ghost draws: links moved by at least one joint, every one of
        which has a ghost value."""
        key = frozenset(have)
        hit = self._ghost_cache.get(key)
        if hit is None:
            bodies = [b for b in range(self.m.nbody) if self.path[b] and self.path[b] <= key]
            hit = frozenset(np.flatnonzero(np.isin(self._geom_body, bodies)).tolist())
            self._ghost_cache[key] = hit
        return hit

    def link_geoms(self, joint: str | None) -> frozenset:
        """The geoms of the link a joint moves (its own body, not below)."""
        if not joint:
            return frozenset()
        jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if jid < 0:
            return frozenset()
        b = int(self.m.jnt_bodyid[jid])
        return frozenset(np.flatnonzero(self._geom_body == b).tolist())


class Layer:
    """One pose to draw: {bus name: rad}, the up vector (measured only), the
    bus joints that count as measured (the rest are dimmed), a ghost colour."""

    def __init__(self, q: dict, up=None, measured=None, rgba=None):
        self.q = {n: v for n, v in q.items() if v is not None and math.isfinite(v)}
        self.up = up
        self.measured = frozenset(self.q) if measured is None else frozenset(measured)
        self.rgba = rgba


def camera(look_z: float, az: float, el: float, dist: float, off=(0.0, 0.0, 0.0)):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (off[0], off[1], look_z + off[2])
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    return cam


class Scene:
    """The posed MjData objects (reused across frames) and the scene builder.
    Works on any MjvScene, so it needs no GL context."""

    def __init__(self, pm: PoseModel):
        self.pm = pm
        self.dm = mujoco.MjData(pm.m)                         # measured
        self.dg = [mujoco.MjData(pm.m), mujoco.MjData(pm.m)]  # commanded, sim
        self.pert = mujoco.MjvPerturb()
        self.n_measured = 0

    def build(self, scn: mujoco.MjvScene, meas: Layer, ghosts=(), cam=None,
              selected: str | None = None) -> int:
        """Fill `scn`: the measured body, then each ghost. Returns the number
        of measured geoms (they come first and no ghost touches them)."""
        pm = self.pm
        root = pm.pose(self.dm, meas.q, meas.up)
        cam = cam or camera(pm.look_z, 118.0, -9.0, 1.08)
        mujoco.mjv_updateScene(pm.m, self.dm, pm.opt, None, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, scn)
        dim = pm.dim_geoms(meas.measured)
        sel = pm.link_geoms(selected)
        G = mujoco.mjtObj.mjOBJ_GEOM
        for i in range(scn.ngeom):
            g = scn.geoms[i]
            if g.objtype != G:
                continue
            if g.objid in sel:
                g.rgba[:] = SEL_RGBA
            elif g.objid in dim:
                g.rgba[:] = DIM_RGBA
        self.n_measured = scn.ngeom
        for k, lay in enumerate(ghosts):
            if not lay.q:
                continue
            while len(self.dg) <= k:
                self.dg.append(mujoco.MjData(pm.m))
            dg = self.dg[k]
            pm.pose(dg, lay.q, root=root)
            show = pm.ghost_geoms(lay.q)
            n0 = scn.ngeom
            mujoco.mjv_addGeoms(pm.m, dg, pm.opt, self.pert, mujoco.mjtCatBit.mjCAT_DYNAMIC, scn)
            for i in range(n0, scn.ngeom):
                g = scn.geoms[i]
                if g.objtype == G and g.objid in show:
                    g.rgba[:] = lay.rgba or CMD_RGBA
                else:
                    g.rgba[3] = 0.0
        return self.n_measured


class Renderer:
    """mujoco.Renderer at a fixed size, around one Scene. GUI thread only."""

    def __init__(self, pm: PoseModel, width: int, height: int):
        width = max(16, min(MAX_PX, int(width)))
        height = max(16, min(MAX_PX, int(height)))
        g = pm.m.vis.global_
        g.offwidth = max(int(g.offwidth), width)
        g.offheight = max(int(g.offheight), height)
        self.width, self.height = width, height
        self.r = mujoco.Renderer(pm.m, height=height, width=width)
        self.scene = Scene(pm)

    def render(self, meas: Layer, ghosts=(), cam=None, selected=None) -> np.ndarray:
        self.scene.build(self.r.scene, meas, ghosts, cam, selected)
        return self.r.render()

    def close(self) -> None:
        if self.r is not None:
            self.r.close()
            self.r = None
