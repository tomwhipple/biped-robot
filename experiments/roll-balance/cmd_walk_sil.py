"""SIL of a scripted --cmd timeline (README, live velocity commands): the steps the --cmd walk would decide for a
scripted command list -- through the walk's own envelope, slew, reversal rule and incremental pre-check -- run through
the firmware's track + trim in the sim plant (sil_gait.run), on the fit plant and on the inset-sole plant.

    python cmd_walk_sil.py                    # fwd 40 -> turn left -> sideways right -> back 32 -> stop, T 4
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cmd_walk as C  # noqa: E402
import sil_gait as G  # noqa: E402
import track_host as H  # noqa: E402

TIMELINE = ["10 0 0"] * 6 + ["0 0 1.25"] * 4 + ["0 -3.75 0"] * 5 + ["-8 0 0"] * 6 + ["stop"]
PLANTS = {"fit": "fit", "inset": "inset"}


def steps_for(T, script, hold=None):
    base = dict(T_step=T, seeds=H.ROBOT_SEED, lead_ms=2500, cal=None, amp=H.ROBOT_AMP, model="dwell", ds=None, h=None,
                pelvis_x=H.ROBOT_PELVIS_X, ankle_ff=H.ROBOT_ANKLE_FF, k_ds=None, drop=None, lift_gate=True)
    cw = C.CmdWalk(H, C.ListSource(script), base, first="R", say=lambda *a, **k: None)
    cw.rehearse(len(script) + 5)
    return cw


def sil_kw(cw):
    cmds = cw.steps
    return dict(dx_steps=[c["dx"] for c in cmds], dy_steps=[c["dy"] for c in cmds],
                yaw_trim=[c["yt"] + (0.0 if c.get("yaw_free") else H.YAW_TRIM) for c in cmds],
                swing_tilt=[c["tilt"] for c in cmds], step_meta=[dict(name=c["name"], key=c["key"]) for c in cmds])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--T", type=float, default=4.0)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "cmd_walk_sil.json"))
    a = ap.parse_args(argv)
    G.PLANTS["inset"] = G.TB.BOARD + ",sole_in_L=8,sole_in_R=-12"
    cw = steps_for(a.T, TIMELINE)
    kw = sil_kw(cw)
    n = len(cw.steps)
    print(f"{n} steps; dx mm {[round(c['dx'] * 1e3) for c in cw.steps]}")
    rows = []
    for plant in ("fit", "inset"):
        r = G.run(a.T, "march", plant=plant, n_steps=n, first="R", amp=(H.ROBOT_AMP[0], 1.30), seed=H.ROBOT_SEED,
                  gkw=dict(pelvis_x=H.ROBOT_PELVIS_X, ankle_ff=H.ROBOT_ANKLE_FF), step_kw=kw, lift_gate=True, model="dwell")
        li = [x["lean_in"] for x in r["reports"]]
        lo = [x["lean_out"] for x in r["reports"]]
        row = dict(plant=plant, T=a.T, steps=n, fell=r["fell"], fall_dir=r["fall_dir"], aborts=r["aborts"],
                   lifts=f"{r['lifts']}/{len(r['phases'])}", lean_in_max=round(max(li), 1), lean_out_max=round(max(lo), 1),
                   peak_tilt=r["peak_tilt"], yaw_end=round(r["yaw_end"], 1), xy_end_mm=[round(1e3 * x) for x in r["xy_end"]],
                   gate=dict(r["gate"], events=len(r["gate"]["events"])), reports=len(r["reports"]),
                   lean_in=[round(x, 1) for x in li])
        print(row)
        rows.append(row)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(dict(timeline=TIMELINE, steps=[dict(dx_mm=round(c["dx"] * 1e3, 2), dy_mm=round(c["dy"] * 1e3, 2),
                                                      dyaw=c["dpsi"], tilt=c["tilt"], name=c["name"]) for c in cw.steps],
                       runs=rows), f, indent=1)
    return 0 if not any(r["fell"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
