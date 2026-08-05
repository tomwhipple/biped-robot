"""Compute CAD-true per-body inertia for bimo_biped_v2.xml (one-off tool).

bimo_biped_v2.xml has CAD-true masses but box-geom inertia. This script builds
each sim body as a composite of its real components -- printed-part STL meshes
(cad/stl/, uniform PLA density scaled to the mass rollup) plus servo/battery/
board boxes at their CAD positions -- and prints <inertial> elements (COM +
principal axes) to bake into the XML.

Component placement mirrors cad/export_assembly.py exactly (same transforms the
interference checks use); frames were verified against the STL bounding boxes.
Mass rollup targets are the v2 XML comments (torso 344 g, hip 29 g, thigh/shin
77 g, foot 91 g); the print/servo split scales the printed parts (screws,
wiring) and never the measured 55 g servo.

Run: ../.venv/bin/python build_v2_inertia.py   (from sim/)
"""
import numpy as np
import trimesh
import os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad"))
import dimensions as D

STL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad", "stl")
PLA = D.FILAMENT_RHO * D.PRINT_MASS_FACTOR  # g/mm^3 effective printed density (PETG)


def mesh_part(name, dz, mass=None):
    """(mass g, COM mm, inertia g*mm^2 about COM) for an STL at offset dz.

    dz is a z-offset, or an (x, y, z) triple -- gopro_base is the first part
    that is not on the centreline (v6 bolts it to the deck at GP_MOUNT_X).
    """
    m = trimesh.load(os.path.join(STL, f"{name}.stl"))
    mm = m.volume * PLA if mass is None else mass
    off = [0, 0, dz] if np.isscalar(dz) else list(dz)
    com = m.center_mass + off
    inertia = m.moment_inertia * (mm / m.volume)   # trimesh default density=1
    return mm, com, inertia


def box_part(mass, center, full_dims):
    dx, dy, dz = full_dims
    I = mass / 12.0 * np.diag([dy*dy + dz*dz, dx*dx + dz*dz, dx*dx + dy*dy])
    return mass, np.asarray(center, float), I


def combine(parts, total=None):
    """Merge (m, com, I_com) components; optionally rescale to a total mass."""
    ms = np.array([p[0] for p in parts])
    if total is not None:
        ms *= total / ms.sum()       # distribute screws/wiring pro-rata
    M = ms.sum()
    com = sum(m * p[1] for m, p in zip(ms, parts)) / M
    I = np.zeros((3, 3))
    for m, p in zip(ms, parts):
        r = p[1] - com
        I += p[2] * (m / p[0])       # component inertia, rescaled mass
        I += m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return M, com, I


def inertial_xml(name, M, com, I):
    """Convert g/mm to kg/m, diagonalize, emit an MJCF <inertial> element."""
    M_kg = M * 1e-3
    com_m = com * 1e-3
    I_kg = I * 1e-9
    w, V = np.linalg.eigh(I_kg)
    # canonicalize the eigen-frame to the MINIMAL rotation: eigh's arbitrary
    # column order/signs can hand back a near-180-degree frame (the 2026-07-28
    # foot came out w ~ 0.0006), which is physically identical but numerically
    # hostile -- it alone pushed MJX-vs-CPU airborne parity from 3e-12 to 2e-7
    # qpos. Pick the column permutation + sign pattern closest to identity.
    from itertools import permutations
    best = None
    for perm in permutations(range(3)):
        Vp = V[:, perm]
        sgn = np.sign(np.diag(Vp))
        sgn[sgn == 0] = 1.0
        Vp = Vp * sgn
        if np.linalg.det(Vp) < 0:
            continue
        tr = np.trace(Vp)
        if best is None or tr > best[0]:
            best = (tr, Vp, np.asarray(perm))
    if best is None:                    # unreachable in practice; keep safe
        best = (np.trace(V), V, np.arange(3))
    _, V, perm = best
    w = w[perm]
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    # rotation matrix -> quaternion (w,x,y,z)
    q = np.empty(4)
    t = np.trace(V)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q[:] = [0.25*s, (V[2,1]-V[1,2])/s, (V[0,2]-V[2,0])/s, (V[1,0]-V[0,1])/s]
    else:
        i = np.argmax(np.diag(V)); j, k = (i+1) % 3, (i+2) % 3
        s = np.sqrt(1.0 + V[i,i] - V[j,j] - V[k,k]) * 2
        q[0] = (V[k,j] - V[j,k]) / s
        q[1+i] = 0.25 * s
        q[1+j] = (V[j,i] + V[i,j]) / s
        q[1+k] = (V[k,i] + V[i,k]) / s
    q /= np.linalg.norm(q)
    # diaginertia order must match the quat's axes = eigh column order
    print(f'  {name}: mass {M:.1f} g  com {np.round(com,1)} mm')
    print(f'      <inertial pos="{com_m[0]:.4f} {com_m[1]:.4f} {com_m[2]:.4f}" '
          f'quat="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}" '
          f'mass="{M_kg:.4f}" '
          f'diaginertia="{w[0]:.4e} {w[1]:.4e} {w[2]:.4e}"/>')


# Servo block. SV_T is the REAL case thickness along the output axis (32.10),
# and SV_AXMID is its centre (+1.30, toward the horn) -- the case is not
# symmetric about the axis. This used to be SV_CASE_T (34.70) centred on zero,
# which is the mirrored phantom the CAD mocks also carried until 2026-07-30; it
# put every servo's 55 g 1.30 mm off along its own axis.
#
# Density is still uniform, which a servo is NOT -- the motor and gear train sit
# at the output end and the PCB at the cable end. Getting that right needs the
# real mass distribution, which nobody has measured. So this is a correct
# ENVELOPE with an approximate interior, and the envelope is the part that was
# actually wrong.
SV_L, SV_W = D.SV_LEN, D.SV_WID                      # 45.22, 24.72
SV_T, SV_AXMID = D.SV_CASE_AXIAL_T, D.SV_CASE_AXIAL_MID   # 32.10, +1.30
SV_ZMID_Y = (D.SV_AXIS_FROM_OUT_END - D.SV_AXIS_FROM_REAR) / 2   # -12.5 (mock_y)

# ---- torso (frame at TORSO_CENTER_Z = 282.86) --------------------------------
dz_deck = (D.DECK_BOT_Z + D.DECK_T) - D.TORSO_CENTER_Z            # +4.11
# Seat drop of the real pack in the v6 V-bay, deck-relative. Same expression as
# export_assembly.battery_mock(): the pack is narrower than the bay, so it sits
# lower than where the chamfers start by (half the slack) * tan(seat angle).
BATT_Z0 = D.BT_SEAT_Z - ((D.BT_SEAT_BAY_W - D.BATT_PACK[1]) / 2
                         * np.tan(np.radians(D.BT_SEAT_DEG)))
torso = combine([
    # v6 (2026-08-04): the torso is ONE print. tower and imu_carrier are gone
    # -- the tower's job (holding the board and the camera up high) went away
    # when the board dropped into a recess in the pelvis and the gopro pad
    # moved onto the deck. There is no printed structure above the deck now.
    mesh_part("pelvis", dz_deck),
    # hip-roll servos (servo_mock_x: axis +X, output end DOWN, case z -10.11..+35.11)
    box_part(D.SERVO_MASS, (SV_AXMID,  D.HIP_SEP/2,
                            D.HIP_ROLL_Z - D.TORSO_CENTER_Z - SV_ZMID_Y),
             (SV_T, SV_W, SV_L)),
    box_part(D.SERVO_MASS, (SV_AXMID, -D.HIP_SEP/2,
                            D.HIP_ROLL_Z - D.TORSO_CENTER_Z - SV_ZMID_Y),
             (SV_T, SV_W, SV_L)),
    # 3S 850 mAh pack on deck (long axis along y, seated toward -x). The bay is
    # a superset of the XT30 850 field (see dimensions.BATT), so model the
    # worst case for mass+height: CNHL 70C 62 x 30 x 25, ~80 g. A lighter/flatter
    # pack (Zeee 74 g x 18.5) only lowers torso COM. Pigtail is in the wiring
    # bucket.
    # v6: the pack no longer sits ON the deck -- it drops through the deck
    # aperture into a bay and rests in the V of two 47 deg seat chamfers, which
    # centre it between the bay walls. Mirrors export_assembly.battery_mock()
    # exactly (same V centreline, same seat drop for the REAL 30 mm pack rather
    # than the 31 mm envelope), so the plant and the fly-in agree.
    box_part(D.BATT_PACK_MASS,
             ((D.BT_SEAT_X + D.BT_WALL_X1) / 2, 0,
              dz_deck + BATT_Z0 + D.BATT_PACK[2] / 2),
             (D.BATT_PACK[1], D.BATT_PACK[0], D.BATT_PACK[2])),
    # driver board: still upright, but v6 moved it from the tower's +x face
    # into a recess in the AFT wall, and the corrected outline (65.01 x 56.01,
    # from the manufacturer's drawing -- the wiki's 65x65 is wrong) is what let
    # it fit. Its centre drops from BOARD_GD_CZ +35.0 to BR_CZ -13.0, which is
    # most of the torso COM change. Slab spans deepest component to PCB face.
    box_part(D.BOARD_GD_MASS,
             ((D.BR_COMP_X + D.BR_PCB_X0) / 2, 0, dz_deck + D.BR_CZ),
             (D.BR_PCB_X0 - D.BR_COMP_X,
              D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])),
    # gopro_base is the ONLY bolt-on left, and it is back on the roof: bolted
    # flat to the deck at GP_MOUNT_X, not stacked on a tower top 75 mm up.
    # imu_carrier is retired with the tower. The camera itself is NOT here --
    # its 154 g rides as the DR payload (payload_cg_z), as it did in v4rom.
    mesh_part("gopro_base", (D.GP_MOUNT_X, 0, dz_deck)),
    # wiring, belt and misc: the same 27 g at deck-2 that the CAD standing-CG
    # rollup in parts.py carries. Without it the plant torso is ~8 % light and
    # its COM sits ~1.4 mm low, because everything unmodelled here is cable and
    # belt sitting just under the deck, not down in the bay. Coarse box: at 27 g
    # its own inertia is noise next to the parallel-axis term.
    box_part(27.0, (0, 0, dz_deck - 2.0), (60.0, 60.0, 10.0)),
    # NO separate IMU breakout any more (2026-07-28): the IMU is inside the
    # General Driver board, so its mass is already in the slab above. This also
    # retires the D.IMU_BOSS_H reference, which had gone stale against
    # dimensions.py and was breaking this script on import.
])               # NO total= for v6, deliberately. The old 402.3 was a curated
                 # rollup that distributed screws and wiring pro-rata over the
                 # modelled parts; nobody has done that rollup for v6, and
                 # inventing a target would bake a guess into the plant. So the
                 # torso mass here is the sum of what is actually modelled --
                 # pelvis print + 2 roll servos + pack + board + gopro_base --
                 # and it UNDERSTATES the build by the fasteners and wiring
                 # (v6 cut those to 6 screws + 4 heat-sets, so the gap is a few
                 # grams, not tens). BOARD_GD_MASS is still an estimate: weigh
                 # the real board and rerun.

# ---- hip (frame at HIP_ROLL_Z) ----------------------------------------------
hip = combine([
    mesh_part("yoke_roll", 0.0),
    mesh_part("yoke_pitch", -D.ROLL_TO_PITCH),
], total=29.6)   # 2026-07-15 rollup (yoke_pitch idler-arm redesign for hip -110)

# ---- thigh / shin (frame at the upper joint axis) ----------------------------
# servo_mock_y: axis +Y, case z -35.11..+10.11, x +/-12.36, y +/-17.35
leg_servo = box_part(D.SERVO_MASS, (0, SV_AXMID, SV_ZMID_Y), (SV_W, SV_T, SV_L))
link = mesh_part("leg_link", 0.0, mass=78.6 - D.SERVO_MASS)   # ~24 g w/ screws
# (78.6 = 2026-07-15 rollup after the flush idler edge + filled horn slot)
leg = combine([leg_servo, link])

# ---- foot (frame at the ankle axis, ANKLE_Z = 17.96) -------------------------
# ankle servo: Rot(0,90,0) * servo_mock_y -> length along x (rear -35.11..+10.11)
foot = combine([
    box_part(D.SERVO_MASS, (SV_ZMID_Y, SV_AXMID, 0), (SV_L, SV_T, SV_W)),
    # printed foot + 4 screws at the mesh COM (segment 110.4 g, 2026-07-15
    # sole enlargement 100 -> 116 for the get-up rise corridor) ...
    mesh_part("foot", D.TPU_PROUD - D.ANKLE_Z, mass=110.4 - D.SERVO_MASS - 9.8),
    # ... and the 9.8 g silicone pad where it actually sits: a 106 x 46 x 1.6
    # self-adhesive sheet (B0FJ8TBMQK) at the very bottom of the sole (issue
    # #7 - it was previously lumped at the mesh COM, ~3 mm too high). Pad
    # tracks the sole with the same 7 mm heel / 3 mm toe insets: x -45..+61.
    box_part(9.8, (8.0, 0, 0.8 - D.ANKLE_Z), (106.0, 46.0, 1.6)),
])

print("torso z-offset check: deck top local z =", dz_deck)
for name, (M, com, I) in [("torso", torso), ("hip", hip), ("thigh/shin", leg),
                          ("foot", foot)]:
    inertial_xml(name, M, com, I)
