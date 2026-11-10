"""Free-space probe around the glued hip yoke's roll horn arm, in the yoke's
local frame (roll axis X at the origin, pitch axis Y at z -50).

Neighbours that move relative to the yoke: link 1 (roll servo, yaw carrier)
turns by -a about X for hip roll a; link 3 (pitch servo, thigh) turns by b
about Y through (0, 0, -50) for hip pitch b. Each probe box is tested at every
sampled angle for overlap volume and minimum distance.

    cd <worktree> && python probe_space.py
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

WT = os.getcwd()
for p in (os.path.join(WT, "cad", "v6"), os.path.join(WT, "cad"), WT):
    sys.path.insert(0, p)

from build123d import Pos, Rot  # noqa: E402
import numpy as np  # noqa: E402

_N = {}


def neighbours():
    if _N:
        return _N
    import dimensions_v6 as V
    from common import mocks as CA
    import yaw_carrier_v6
    import leg_link_v6
    dz = V.HIP_YAW_Z - V.HIP_ROLL_Z
    pz = V.HIP_PITCH_Z - V.HIP_ROLL_Z
    _N["roll"] = {"carrier": Pos(0, 0, dz) * yaw_carrier_v6.yaw_carrier_v6(),
                  "roll_servo": CA.servo_mock_x()}
    _N["pitch"] = {"pitch_servo": CA.servo_mock_y(), "thigh": leg_link_v6.leg_link_v6()}
    _N["pz"] = pz
    _N["rom"] = V.ROM
    return _N


def posed(joint, solid, ang, pz):
    if joint == "roll":
        return Rot(-ang, 0, 0) * solid
    return Pos(0, 0, pz) * Rot(0, ang, 0) * solid


def run_probe(args):
    name, box = args
    from common import shapes
    N = neighbours()
    probe = shapes.box(*box)
    out = []
    for joint, rom_key in (("roll", "hip_roll"), ("pitch", "hip_pitch")):
        lo, hi = N["rom"][rom_key]
        angs = sorted(set(list(np.linspace(lo, hi, 15)) + [lo, hi, 0.0]))
        for nb, s in N[joint].items():
            worst_d, worst_v, worst_a = 1e9, 0.0, None
            for a in angs:
                q = posed(joint, s, a, N["pz"])
                try:
                    v = (probe & q).volume
                except Exception:  # noqa: BLE001
                    v = 0.0
                d = probe.distance_to(q) if v < 1e-6 else 0.0
                if v > worst_v or (v == worst_v and d < worst_d):
                    worst_d, worst_v, worst_a = d, v, a
            out.append((nb, worst_v, worst_d, worst_a))
    return name, box, out


PROBES = {
    "P_a above +y widening y17..23.45 z-16..-6 x0..24.45": (0.0, 24.45, 17, 23.45, -16, -6),
    "P_a2 above +y widening z-16..-10":       (0.0, 24.45, 17, 23.45, -16, -10),
    "P_b y14..23.45 z-16..-8 x-14..21.45":    (-14.0, 21.45, 14, 23.45, -16, -8),
    "P_b2 y14..23.45 z-16..-12 x-14..21.45":  (-14.0, 21.45, 14, 23.45, -16, -12),
    "P_c horn corner y12..23.45 z-16..-4 x21.45..27.45": (21.45, 27.45, 12, 23.45, -16, -4),
    "P_d below front lip x24.45..27.45 z-24..-20": (24.45, 27.45, -10, 17, -24, -20),
    "P_e front lip deeper x24.45..30 z-20..-16":  (24.45, 30.0, -10, 17, -20, -16),
    "P_f buttress x27.45..30 z-20..-8":          (27.45, 30.0, -10, 12, -20, -8),
    "P_h under +y widening y17..20 x16..24.45 z-24..-20": (16.0, 24.45, 17, 20.0, -24, -20),
    "P_i -y side y-17..-10 z-16..-6 x12..24.45": (12.0, 24.45, -17, -10, -16, -6),
    "P_j -y side y-17..-10 z-16..-10 x-14..24.45": (-14.0, 24.45, -17, -10, -16, -10),
    "P_k under rear overhang x-23.95..-14 z-24..-20": (-23.95, -14.0, -10, 17, -24, -20),
    "P_l under front x20..24.45 z-22..-20 y-10..17": (20.0, 24.45, -10, 17, -22, -20),
    "P_m under front x16..20 z-22..-20 y-10..17": (16.0, 20.0, -10, 17, -22, -20),
}


if __name__ == "__main__":
    items = list(PROBES.items())
    with ProcessPoolExecutor(max_workers=min(len(items), os.cpu_count() or 4)) as ex:
        for name, box, out in ex.map(run_probe, items):
            print(f"\n{name}  {box}")
            for nb, v, d, a in out:
                flag = "HIT " if v > 1e-6 else ("tight" if d < 0.5 else "ok  ")
                print(f"   {flag} {nb:12s} vol {v:8.2f} mm3  min dist {d:6.2f} mm  at {a:7.1f} deg")
