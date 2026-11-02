"""First inter-leg contact as one hip adducts: sweep L_hip_roll from 0 toward
the other leg (and R_hip_roll the mirror way) and report, per angle, the
closest R-leg piece to the L leg. OLD=1 builds the yoke from another version of
hip_yoke_glued.py, copied by the caller into <output dir>/old/.
    cd <repo root> && [OLD=1] python experiments/hip-yoke-strength/adduction.py"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

WT = os.getcwd()
TMP = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   "build", "experiments", "hip-yoke-strength")
paths = [os.path.join(WT, "cad", "v6"), os.path.join(WT, "cad"), WT]
if os.environ.get("OLD"):
    paths.insert(0, os.path.join(TMP, "old"))
for p in paths[::-1]:
    sys.path.insert(0, p)

MOVING_L = ("thigh_L", "servo_hip_pitch_L", "shin_L", "servo_knee_L", "hip_yoke_L")
R_PIECES = ("hip_yoke_R", "thigh_R", "servo_hip_pitch_R", "yaw_carrier_R", "servo_hip_roll_R")


def at(args):
    joint, ang = args
    import assembly_v6 as A
    P = {c.label: c for c in A.robot({joint: ang}).children}
    movers = MOVING_L if joint.startswith("L") else tuple(m.replace("_L", "_R") for m in MOVING_L)
    others = R_PIECES if joint.startswith("L") else tuple(r.replace("_R", "_L") for r in R_PIECES)
    best = {}
    for o in others:
        d = min(P[m].distance_to(P[o]) for m in movers)
        best[o] = d
    return joint, ang, best


if __name__ == "__main__":
    jobs = [("L_hip_roll", -a) for a in range(10, 56, 5)] + [("R_hip_roll", a) for a in range(10, 56, 5)]
    with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
        for joint, ang, best in ex.map(at, jobs):
            closest = min(best, key=best.get)
            print(f"{joint} {ang:6.1f}  closest {closest:18s} {best[closest]:6.2f}   yoke {best.get('hip_yoke_R', best.get('hip_yoke_L')):6.2f}")
