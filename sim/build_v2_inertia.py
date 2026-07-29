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
    """(mass g, COM mm, inertia g*mm^2 about COM) for an STL at z-offset dz."""
    m = trimesh.load(os.path.join(STL, f"{name}.stl"))
    mm = m.volume * PLA if mass is None else mass
    com = m.center_mass + [0, 0, dz]
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


SV_L, SV_W, SV_T = D.SV_LEN, D.SV_WID, D.SV_CASE_T   # 45.22, 24.72, 34.70
SV_ZMID_Y = (D.SV_AXIS_FROM_OUT_END - D.SV_AXIS_FROM_REAR) / 2   # -12.5 (mock_y)

# ---- torso (frame at TORSO_CENTER_Z = 282.86) --------------------------------
dz_deck = (D.DECK_BOT_Z + D.DECK_T) - D.TORSO_CENTER_Z            # +4.14
torso = combine([
    mesh_part("pelvis", dz_deck),
    mesh_part("tower", dz_deck),
    # hip-roll servos (servo_mock_x: axis +X, output end DOWN, case z -10.11..+35.11)
    box_part(D.SERVO_MASS, (0,  D.HIP_SEP/2, D.HIP_ROLL_Z - D.TORSO_CENTER_Z - SV_ZMID_Y),
             (SV_T, SV_W, SV_L)),
    box_part(D.SERVO_MASS, (0, -D.HIP_SEP/2, D.HIP_ROLL_Z - D.TORSO_CENTER_Z - SV_ZMID_Y),
             (SV_T, SV_W, SV_L)),
    # 3S 850 mAh pack on deck (long axis along y, seated toward -x). The bay is
    # a superset of the XT30 850 field (see dimensions.BATT), so model the
    # worst case for mass+height: CNHL 70C 62 x 30 x 25, ~80 g. A lighter/flatter
    # pack (Zeee 74 g x 18.5) only lowers torso COM. Pigtail is in the wiring
    # bucket.
    # pack centre follows the seat: BATT_SEAT_X moved -17 -> -24 with the
    # 2026-07-28 tower resize (board takes the freed +x side), so the pack
    # centre is SEAT + width/2 = -9.0, not the old hand-baked -1.0
    box_part(D.BATT_PACK_MASS,
             (D.BATT_SEAT_X + D.BATT_PACK[1] / 2, 0,
              dz_deck + D.BATT_PACK[2] / 2),
             (D.BATT_PACK[1], D.BATT_PACK[0], D.BATT_PACK[2])),
    # driver board: the General Driver stands UPRIGHT against the +x side
    # (2026-07-28). It is 65 x 65, so its mock is a slab in the y-z plane,
    # centred on BOARD_GD_CZ, spanning the PCB plus its component reach in x.
    # (Was a 65 x 30 x 5 slab lying face-down under the top plate.)
    box_part(D.BOARD_GD_MASS,
             (D.BOARD_GD_PCB_X + (1.63 + D.BOARD_GD_COMP) / 2, 0,
              dz_deck + D.BOARD_GD_CZ),
             (1.63 + D.BOARD_GD_COMP,
              D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])),
    # tower-top accessory stack (2026-07-17, closing the imu_carrier section's
    # "honest omission"): both STLs have z=0 at their mounting plane. The
    # carrier sandwiches on the tower top; gopro_base sits on the carrier.
    mesh_part("imu_carrier", dz_deck + D.TOWER_H),
    mesh_part("gopro_base", dz_deck + D.TOWER_H + D.IMU_CARRIER_T),
    # NO separate IMU breakout any more (2026-07-28): the IMU is inside the
    # General Driver board, so its mass is already in the slab above. This also
    # retires the D.IMU_BOSS_H reference, which had gone stale against
    # dimensions.py and was breaking this script on import.
], total=402.3)   # was 341.8. Deltas, all from the board swap:
                  #   tower   41.8 -> 83.4 g  (+41.5) -- TOWER_W 42->56,
                  #                            TOWER_H 43.5->75, + partition
                  #   board   20.0 -> 42.0 g  (+22.0) -- General Driver, and
                  #                            BOARD_GD_MASS is an ESTIMATE:
                  #                            weigh the real board and redo
                  #   IMU      3.0 ->  0.0 g   (-3.0) -- now inside the board
                  # net +60.5 g of torso, ~18% up, and most of it high.

# ---- hip (frame at HIP_ROLL_Z) ----------------------------------------------
hip = combine([
    mesh_part("yoke_roll", 0.0),
    mesh_part("yoke_pitch", -D.ROLL_TO_PITCH),
], total=29.6)   # 2026-07-15 rollup (yoke_pitch idler-arm redesign for hip -110)

# ---- thigh / shin (frame at the upper joint axis) ----------------------------
# servo_mock_y: axis +Y, case z -35.11..+10.11, x +/-12.36, y +/-17.35
leg_servo = box_part(D.SERVO_MASS, (0, 0, SV_ZMID_Y), (SV_W, SV_T, SV_L))
link = mesh_part("leg_link", 0.0, mass=78.6 - D.SERVO_MASS)   # ~24 g w/ screws
# (78.6 = 2026-07-15 rollup after the flush idler edge + filled horn slot)
leg = combine([leg_servo, link])

# ---- foot (frame at the ankle axis, ANKLE_Z = 17.96) -------------------------
# ankle servo: Rot(0,90,0) * servo_mock_y -> length along x (rear -35.11..+10.11)
foot = combine([
    box_part(D.SERVO_MASS, (SV_ZMID_Y, 0, 0), (SV_L, SV_T, SV_W)),
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
