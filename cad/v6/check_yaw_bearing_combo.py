"""One-off combined-pose interference check for the hip-yaw bearing study
(docs/design-v6/study-yaw-bearing.md): yaw at 0/+-45 CROSSED with hip_roll
and hip_pitch at their ROM extremes, checking the new skirt (pelvis_v7)
against the carrier's own bay walls and the hip-roll servo case -- the two
pairs the single-joint sweep in check_assembly_v6.py does not combine.

    .venv/bin/python cad/v6/check_yaw_bearing_combo.py
"""
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A  # noqa: E402
D = V.D

PAIRS = [
    ("yaw_carrier_L", "pelvis_v7", "carrier (boss+plate) vs the new skirt/recess"),
    ("servo_hip_roll_L", "pelvis_v7", "roll servo case vs the new skirt/recess"),
]

yaws = (V.ROM["hip_yaw"][0], 0.0, V.ROM["hip_yaw"][1])
rolls = V.ROM["hip_roll"]
pitches = V.ROM["hip_pitch"]

worst = {i: (0.0, 1e9, None) for i in range(len(PAIRS))}
for yaw, roll, pitch in itertools.product(yaws, rolls, pitches):
    pose = {"L_hip_yaw": yaw, "L_hip_roll": roll, "L_hip_pitch": pitch}
    comp = A.robot(pose)
    P = {c.label: c for c in comp.children}
    for i, (la, lb, note) in enumerate(PAIRS):
        a, b = P[la], P[lb]
        try:
            vol = (a & b).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        try:
            dist = a.distance_to(b)
        except Exception:  # noqa: BLE001
            dist = float("nan")
        wv, wd, wp = worst[i]
        if vol > wv or (vol == wv and dist < wd):
            worst[i] = (vol, dist, (yaw, roll, pitch))

fails = 0
print(f"{'A':20s} {'B':12s} {'overlap mm3':>11s} {'min dist':>9s}  {'at (yaw,roll,pitch)':22s}  verdict  note")
for i, (la, lb, note) in enumerate(PAIRS):
    vol, dist, at = worst[i]
    ok = vol < 0.5 and dist >= D.SWEEP_BUFFER - 1e-6
    fails += 0 if ok else 1
    print(f"{la:20s} {lb:12s} {vol:11.2f} {dist:9.2f}  {str(at):22s}  {'ok' if ok else 'FAIL'}   {note}")
print(f"=> {'ALL CLEAR' if fails == 0 else f'{fails} FAIL'}")
sys.exit(1 if fails else 0)
