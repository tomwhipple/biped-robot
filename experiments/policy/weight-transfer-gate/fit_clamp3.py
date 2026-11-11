"""Round 3, item 1: refit the beyond-encoder roll terms to the 2026-11-10 clamp signature.

    python fit_clamp3.py [--base grade:<terms>] [--md out.md] [--json out.json]

Robot (2026-11-10-robot/README.md section 4; hw_sessions/2026-11-10/m271/NOTES.md): one foot
clamped, the other lifted straight up by hip pitch / knee / ankle. 30 mm: torso tilt 9.4 deg toward
the lifted side on both sides (9.7, 9.35 after tightening, 9.36 mirrored); stance ankle-roll encoder
1.5-1.6 deg, hip-roll encoder 0.4-0.7 deg. 15 mm: the lifted foot came back down on its outer edge
(about 7 deg, first side). Front-on stills (right foot clamped, +-1 deg): foot -> shank 5.6 deg,
shank -> thigh about 2.0, thigh -> torso about 2.1.

Targets (deg): tilt 9.4; ankle-roll encoder 1.55; hip-roll encoder 0.55; beyond the ankle-roll
encoder 5.6 - 1.55 = 4.05; beyond the hip-roll encoder 2.1 - 0.55 + 2.0 = 3.55. The shank -> thigh 2 deg
sits at the knee / hip-pitch joints, whose axes are pitch: no roll term can place it there, so it is
lumped into the hip-roll term (the nearest roll joint above it). Cost = the sum of squares of the
five residuals (deg); the 15 mm lift is reported, not fitted (on the robot it is a contact limit,
not an equilibrium).

Two families on the base plant (default: grade + MASS_SYM): series springs (ser_ankle, ser_hip,
N m/rad; gate_common's quasi-static form) and free play (lash_ankle, lash_hip, deg peak to peak).
Each: a coarse grid, then a 5 x 5 grid around the best. Lifted side: L (right foot clamped, as in
the stills); the fitted terms are then run on both sides at 30 and 15 mm."""
from __future__ import annotations

import argparse
import itertools
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

TARGET = dict(tilt=9.4, enc_ankle=1.55, enc_hip=0.55, beyond_ankle=4.05, beyond_hip=3.55)
FAMILIES = {"spring": (("ser_ankle", "ser_hip"), (3.0, 5.0, 8.0, 12.0, 20.0, 35.0, 60.0)),
            "play": (("lash_ankle", "lash_hip"), (0.0, 2.0, 4.0, 6.0, 8.0, 11.0, 14.0))}
# the same terms at the 10-24 numbers (roll-compliance-2026-10-24.md)
T1024 = dict(tilt=5.1, enc_ankle=1.25, enc_hip=0.8)


def base_spec(base):
    name, _, t = base.partition(":")
    return name, GC.parse_terms(t.replace("|", ","))


def clamp(job):
    """(base, terms, lifted side, dz) -> tilt, encoders, beyond, lifted-foot clearance"""
    base, terms, side, dz = job
    import roll_compliance as RC
    name, bt = base_spec(base)
    t = dict(bt, **terms)
    stance = "L" if side == "R" else "R"
    xml = RC.clamped_xml(GC.XML, stance)
    try:
        env = GC.make_plant(name, terms=t, xml=xml)
        env.domain_rand = False
        det = {}
        tilt, enc, lift = GC.clamp_run(env, side, dz=dz, detail=det)
    finally:
        os.unlink(xml)
    return dict(tilt=tilt, enc_ankle=enc["ankle_roll"], enc_hip=enc["hip_roll"],
                beyond_ankle=det["beyond"]["ankle_roll"], beyond_hip=det["beyond"]["hip_roll"],
                clear_mm=det["clear_mm"], lift_mm=lift)


def cost(r):
    return float(sum((r[k] - v) ** 2 for k, v in TARGET.items()))


def grid_fit(base, family, workers):
    keys, vals = FAMILIES[family]
    jobs = [(base, dict(zip(keys, v)), "L", 0.030) for v in itertools.product(vals, vals)]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(clamp, jobs))
    rows = [(j[1], r, cost(r)) for j, r in zip(jobs, res)]
    best = min(rows, key=lambda x: x[2])[0]
    fine = []
    for k in keys:
        x = best[k]
        i = vals.index(x)
        lo = vals[max(0, i - 1)]
        hi = vals[min(len(vals) - 1, i + 1)]
        fine.append(sorted(set(np.round(np.linspace(lo, hi, 5), 3).tolist())))
    jobs2 = [(base, dict(zip(keys, v)), "L", 0.030) for v in itertools.product(*fine)]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res2 = list(ex.map(clamp, jobs2))
    rows += [(j[1], r, cost(r)) for j, r in zip(jobs2, res2)]
    rows.sort(key=lambda x: x[2])
    return rows


def check_fit(base, terms, workers):
    jobs = [(base, terms, side, dz) for side in ("L", "R") for dz in (0.030, 0.015)]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(clamp, jobs))
    return {f"{j[2]}{int(round(j[3] * 1e3))}": r for j, r in zip(jobs, res)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="grade:" + GC.MASS_SYM.replace(",", "|"))
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "24")))
    a = ap.parse_args(argv)
    out = {}
    L = [f"Clamp refit on `{a.base}` (lifted L, 30 mm). Targets: " +
         ", ".join(f"{k} {v}" for k, v in TARGET.items()) + " deg (beyond_hip lumps the stills' shank -> thigh 2.0).", ""]
    base0 = clamp((a.base, {}, "L", 0.030))
    L.append(f"Base plant, no term: tilt {base0['tilt']:.2f}, encoders {base0['enc_ankle']:.2f} / {base0['enc_hip']:.2f}, "
             f"beyond {base0['beyond_ankle']:.2f} / {base0['beyond_hip']:.2f}, cost {cost(base0):.1f}")
    for fam in FAMILIES:
        rows = grid_fit(a.base, fam, a.workers)
        best_t, best_r, best_c = rows[0]
        chk = check_fit(a.base, best_t, a.workers)
        out[fam] = dict(best=best_t, fit=best_r, cost=best_c, check=chk,
                        grid=[dict(terms=t, **r, cost=c) for t, r, c in rows])
        L += ["", f"### {fam}: best {best_t} (cost {best_c:.2f})", "",
              "| quantity | robot | fit | residual |", "|---|---|---|---|"]
        for k, v in TARGET.items():
            L.append(f"| {k} | {v} | {best_r[k]:.2f} | {best_r[k] - v:+.2f} |")
        L += ["", "| lifted side, lift | tilt deg | encoders ankle / hip | beyond ankle / hip | lifted-foot lowest pad, mm |",
              "|---|---|---|---|---|"]
        for key, r in chk.items():
            L.append(f"| {key[0]}, {key[1:]} mm | {r['tilt']:.2f} | {r['enc_ankle']:.2f} / {r['enc_hip']:.2f} | "
                     f"{r['beyond_ankle']:.2f} / {r['beyond_hip']:.2f} | {r['clear_mm']:.1f} |")
        L.append("")
        L.append("Next best: " + "; ".join(f"{t} cost {c:.2f}" for t, _, c in rows[1:4]))
    out["base"] = base0
    # the same springs at the 10-24 robot (tethered: no pack), and round 2's 10-24 fit on today's plant
    nopack = "grade:torso_kg=0.4855|zoff_La=0"
    sp = out["spring"]["best"]
    jobs = [(nopack, sp, "L", 0.030), (nopack, {}, "L", 0.030), (a.base, dict(ser_hip=14.2), "L", 0.030)]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        r1024, r1024_0, r2 = ex.map(clamp, jobs)
    out["at_1024"] = dict(nopack_fit=r1024, nopack_none=r1024_0, round2_on_today=r2)
    L += ["", "### Against the 10-24 numbers (tilt 5.0-5.2, encoders 1.0-1.5 / 0.7-0.9)", "",
          "| plant, 30 mm, L lifted | tilt | encoders ankle / hip |", "|---|---|---|",
          f"| no pack (the 10-24 robot was tethered), no term | {r1024_0['tilt']:.2f} | {r1024_0['enc_ankle']:.2f} / {r1024_0['enc_hip']:.2f} |",
          f"| no pack + today's spring fit {sp} | {r1024['tilt']:.2f} | {r1024['enc_ankle']:.2f} / {r1024['enc_hip']:.2f} |",
          f"| today's plant + round 2's 10-24 fit (ser_hip=14.2) | {r2['tilt']:.2f} | {r2['enc_ankle']:.2f} / {r2['enc_hip']:.2f} |"]
    md = "\n".join(L)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
