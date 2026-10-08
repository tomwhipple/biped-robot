"""ROM interference gate for v5b: the pairs the ankle swap creates.

Every pair is (pieces that move relative to each other, the poses they are
swept through). A pair passes with zero boolean overlap AND a minimum distance
of at least SWEEP_BUFFER (0.5 mm), or -- for the pairs DESIGNED to touch (the
shin's pads on the ankle-pitch servo's discs, the link's plates against the
shin fork) -- zero overlap alone.

Only the pairs the swap creates are listed: the v5 shin and thigh against the
v6 ankle stack, and the two feet against each other at v5's 66 mm hips at the
stance test's poses. Hip yaw is not in the test, and the feet limit it: the
script ends with the yaw at which they meet.
Everything above the knee is the v5 robot unchanged (cad/check_assembly.py at
the pinned commit), and the ankle stack's own pairs (link vs roll servo vs
foot) are the same parts in the same relative places as on the robot, gated by
cad/v6/check_assembly_v6.py.

    .venv/bin/python experiments/v5b/check_v5b.py            # exit 0 = clear
    .venv/bin/python experiments/v5b/check_v5b.py --jobs 8 --samples 9
"""
from __future__ import annotations

import argparse
import itertools
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SWEEP_BUFFER = 0.5

# (label, A, B, designed contact?, note)
SHIN = [
    ("ankle_link_{s}", "shin_{s}", True, "link plates inside the shin's fork tines (0.7 mm running gap)"),
    ("servo_ankle_pitch_{s}", "shin_{s}", True, "shin pads on the pitch servo's horn and idler discs"),
    ("servo_ankle_roll_{s}", "shin_{s}", False, "roll servo case vs the shin's fork"),
    ("foot_{s}", "shin_{s}", False, "foot cradle and tabs vs the shin's fork"),
    ("sole_{s}", "shin_{s}", False, "sole vs the shin's fork"),
]
THIGH = [
    ("ankle_link_{s}", "thigh_{s}", False, "ankle link vs the thigh at deep knee flexion"),
    ("servo_ankle_roll_{s}", "thigh_{s}", False, "roll servo vs the thigh at deep knee flexion"),
    ("foot_{s}", "thigh_{s}", False, "foot vs the thigh at deep knee flexion"),
    ("servo_ankle_pitch_{s}", "thigh_{s}", False, "pitch servo case vs the thigh at deep knee flexion"),
]


def _grid(lo, hi, n):
    return [lo + (hi - lo) * k / (n - 1) for k in range(n)]


def jobs(samples):
    """[(group, pose, A, B, touching, note)]."""
    import assembly_v5b as A
    R = A.ROM
    out = []
    # the shin's fork against the ankle stack: pitch swept, roll at its ends and 0
    for (a, b, touch, note), p, r in itertools.product(SHIN, _grid(*R["ankle_pitch"], samples),
                                                       (R["ankle_roll"][0], 0.0, R["ankle_roll"][1])):
        out.append(("shin", {"L_ankle_pitch": p, "L_ankle_roll": r}, a.format(s="L"), b.format(s="L"), touch, note))
    # the thigh against the ankle stack: knee swept, pitch at its ends and 0
    for (a, b, touch, note), k, p in itertools.product(THIGH, _grid(*R["knee"], samples),
                                                       (R["ankle_pitch"][0], 0.0, R["ankle_pitch"][1])):
        out.append(("thigh", {"L_knee": k, "L_ankle_pitch": p}, a.format(s="L"), b.format(s="L"), touch, note))
    # foot against foot at the stance test's poses: standing and the one-foot
    # shift both ways (a parallelogram: hips and ankles roll opposite)
    for pose in [{}] + [{"L_hip_roll": sgn * 12, "R_hip_roll": sgn * 12,
                         "L_ankle_roll": -sgn * 12, "R_ankle_roll": -sgn * 12} for sgn in (1, -1)]:
        for a, b in (("foot_L", "foot_R"), ("sole_L", "sole_R")):
            out.append(("feet", pose, a, b, False, "the two feet at v5's 66 mm hips, stance-test poses"))
    return out


def run_one(job):
    import assembly_v5b as A
    group, pose, a, b, touch, note = job
    P = {}
    for side in ("L", "R"):
        P.update(A.leg_pieces(side, pose, {a, b}))
    sa, sb = P[a], P[b]
    try:
        vol = (sa & sb).volume
    except Exception:  # noqa: BLE001  (an empty boolean)
        vol = 0.0
    try:
        dist = sa.distance_to(sb)
    except Exception:  # noqa: BLE001
        dist = float("nan")
    return group, pose, a, b, touch, note, vol, dist


def ROM_YAW():
    import assembly_v5b as A
    return A.ROM["hip_yaw"]


def yaw_limit(sgn, iters=10):
    """Largest left hip yaw toward the other foot (sgn -1: toe in, +1: heel in)
    that keeps foot_L SWEEP_BUFFER clear of foot_R, by bisection."""
    import assembly_v5b as A

    def gap(deg):
        P = {**A.leg_pieces("L", {"L_hip_yaw": sgn * deg}, {"foot_L"}), **A.leg_pieces("R", {}, {"foot_R"})}
        return P["foot_L"].distance_to(P["foot_R"])
    lo, hi = 0.0, max(map(abs, ROM_YAW()))
    if gap(hi) >= SWEEP_BUFFER:
        return hi
    for _ in range(iters):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if gap(mid) >= SWEEP_BUFFER else (lo, mid)
    return lo


def fmt_pose(pose):
    return ", ".join(f"{k} {v:+.0f}" for k, v in pose.items()) or "standing"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=9)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    a = ap.parse_args(argv)
    t0 = time.time()
    todo = jobs(a.samples)
    with ProcessPoolExecutor(a.jobs) as ex:
        res = list(ex.map(run_one, todo, chunksize=4))
    # worst row per (group, A, B): overlap first, then distance
    worst = {}
    for r in res:
        group, pose, la, lb, touch, note, vol, dist = r
        key = (group, la, lb)
        w = worst.get(key)
        if w is None or (vol, -dist) > (w[6], -w[7]):
            worst[key] = r
    fails = 0
    print(f"v5b ROM gate: {len(todo)} posed pairs, {a.jobs} processes, buffer {SWEEP_BUFFER} mm\n")
    print(f"{'pair':48s} {'overlap mm3':>11s} {'min dist':>9s}  verdict   worst pose")
    for (group, la, lb), (_, pose, _, _, touch, note, vol, dist) in worst.items():
        ok = vol < 1.0 and (touch or dist >= SWEEP_BUFFER)
        fails += 0 if ok else 1
        tag = "ok" if ok else "FAIL"
        tag += " (touch)" if touch else ""
        print(f"{la + ' / ' + lb:48s} {vol:11.2f} {dist:9.2f}  {tag:9s} {fmt_pose(pose)}")
        print(f"{'':48s} {note}")
    print(f"\n{'ALL CLEAR' if not fails else f'{fails} PAIR(S) FAIL'}  ({time.time() - t0:.0f} s)")
    # hip yaw is not in the stance test, and at 66 mm the feet limit it
    with ProcessPoolExecutor(2) as ex:
        lims = list(ex.map(yaw_limit, (-1, 1)))
    print(f"\nhip yaw limit, one leg yawing, the other at 0 (feet {SWEEP_BUFFER} mm apart): "
          f"toe in {lims[0]:.1f} deg, heel in {lims[1]:.1f} deg (ROM is +-{max(map(abs, ROM_YAW()))})")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
