"""Bus-current budget for the 17-servo robot from the OPEN-LOOP traces (#77,
2026-09-29): the Gate D walk cases and the scripted get-up under Plan B.

sim/current_budget.py does the same for a trained policy (the referee's
command scenarios); no policy exists for this robot yet, so this script
drives the design's own open-loop motions instead and applies the STS3215
current model of sim/sts_servo_model.py to every control tick:

  motor  (upper bound)  I = 30 mA idle + (180 - 30) mA x |w|/w0 + |tau| / Kt,
                        clamped to the stall current (2.7 A x V/12);
                        Kt = 11 kg.cm/A -- the ST-3215-C018 datasheet's
                        electrical table and its section-10 I-vs-torque line
  bridge (lower bound)  I = 30 mA + (max(tau w, 0) + K_CU tau^2) / V,
                        K_CU = 3.75 W/(N.m)^2 fitted to the stall point --
                        walker_env's power model, what current_budget.py uses

Traces (every 20 ms control tick; tau = the servo model's torque):
  walk, as drawn     r5_asdrawn (lumped 2.10 kg, 17 servos, arms held at
                     shoulder +15), the four gate cases + the -15 turn at mu 0.7
  walk, CAD plant    CAD-inertial all-STS3215 plant (1.67 kg, 13 servos: no
                     arms), the four gate cases; the arms' four servos are
                     added at their idle current
  get-up             r5_asdrawn_rom120, the recommended seat push (shoulder
                     90 -> 0) and the 60 -> 0 variant, the six robustness
                     conditions; both start with the arms folded (180)
Plan B everywhere (STS3215, P x4 on the rolls and knees), 11.1 V.

Chains: the General Driver has two servo ports (H5, H6) on one electrical bus;
the first lead of a chain carries the whole chain. Splits reported:
  A  three chains  L leg | R leg | arms + neck          (needs a splitter)
  B  two ports     L leg + L arm + neck | R leg + R arm
  C  two ports     both legs | arms + neck

    .venv/bin/python sim/current_budget_v6.py > docs/design-v6/current_budget_v6.txt
"""
from __future__ import annotations

import dataclasses
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

import gate_no3250 as GN  # noqa: E402
import design_gates as DG  # noqa: E402
import sts_servo_model as SM  # noqa: E402
from gen_plant_v6 import DesignParams, build_xml  # noqa: E402

VOLTS = 11.1
SET = "3215_p4rk"
ARM_JOINTS = ("L_shoulder", "L_elbow", "R_shoulder", "R_elbow")
SPLITS = {
    "A (3 chains)": {"L leg": lambda n: n.startswith("L_") and n not in ARM_JOINTS,
                     "R leg": lambda n: n.startswith("R_") and n not in ARM_JOINTS,
                     "arms+neck": lambda n: n in ARM_JOINTS or n == "neck_yaw"},
    "B (2 ports)": {"L leg+L arm+neck": lambda n: n.startswith("L_") or n == "neck_yaw",
                    "R leg+R arm": lambda n: n.startswith("R_")},
    "C (2 ports)": {"both legs": lambda n: n not in ARM_JOINTS and n != "neck_yaw",
                    "arms+neck": lambda n: n in ARM_JOINTS or n == "neck_yaw"},
}


# ------------------------------------------------------------------ trace jobs
def _walk_cad_job(args):
    label, xml, ckw, tkw = args
    p = DesignParams()
    r = GN.walk_one(p, xml, SET, None, dict(ckw, track_trace=True), tkw)
    names = list(DG.JN) + ["neck_yaw"]
    return dict(label=f"walk, CAD plant: {label}", ok=bool(r["ok"]), tau=r["trace_tau"], qd=r["trace_qd"],
                w0=np.array(r["w0"]), names=names[:r["trace_tau"].shape[1]], extra_idle=4)


def _walk_arms_job(args):
    import getup_v6_shoulder as S
    label, xml, ckw, tkw = args
    base = dataclasses.replace(S.BASE, **S.CONFIGS["r5_asdrawn"])
    r = S.walk_with_arm_contacts(base, xml, per_joint=GN.SETS[SET], arm_pose=(15.0, 0.0), trace=True, **ckw, **tkw)
    return dict(label=f"walk, as drawn: {label}", ok=not r["fell"], tau=r["tau"], qd=r["qd"], w0=r["w0"],
                names=r["names"], extra_idle=0)


def _getup_job(args):
    import getup_v6_shoulder as S
    import getup_v6 as G
    seq_name, seq, c = args
    S.H, S.K = -120.0, -130.0
    G.PJ_DEFAULT = GN.SETS[SET]
    p, xml = S.plant("r5_asdrawn_rom120")
    (s0, e0), (s1, e1), a = seq
    rt = S.run_traced(p, xml, S.seat_push(False, s0, s1, 2.0, a, elbow=(e0, e1)), start="supine", track=(),
                      trace=True, **c)
    return dict(label=f"get-up {seq_name}: {GN._cond_label(c)}", ok=bool(rt["ok"]), tau=rt["tau"], qd=rt["qd"],
                w0=rt["w0"], names=rt["names"], extra_idle=0)


# ------------------------------------------------------------------ statistics
def currents(tr, model):
    i = SM.supply_current(tr["tau"], tr["qd"], tr["w0"][None, :], VOLTS, model)
    return i, tr["extra_idle"] * SM.I_IDLE


def window_mean_max(x, n):
    if len(x) < n:
        return float(np.mean(x))
    c = np.cumsum(np.concatenate([[0.0], x]))
    return float(((c[n:] - c[:-n]) / n).max())


def stats(x, dt=0.02):
    return dict(peak=float(x.max()), p99=float(np.percentile(x, 99)), rms=float(np.sqrt(np.mean(x ** 2))),
                mean=float(x.mean()), w2=window_mean_max(x, int(round(2.0 / dt))))


def fmt(s):
    return f"peak {s['peak']:5.2f}  p99 {s['p99']:5.2f}  RMS {s['rms']:5.2f}  mean {s['mean']:5.2f}  worst 2 s {s['w2']:5.2f}"


def report_group(title, traces):
    print(f"\n== {title}  ({len(traces)} runs; {sum(t['ok'] for t in traces)} completed OK)", flush=True)
    for model in ("motor", "bridge"):
        tot_all = []
        chain_all = {sp: {ch: [] for ch in chs} for sp, chs in SPLITS.items()}
        per_joint_pk = {}
        for tr in traces:
            i, extra = currents(tr, model)
            tot = i.sum(1) + extra
            tot_all.append(tot)
            for sp, chs in SPLITS.items():
                for ch, sel in chs.items():
                    idx = [k for k, n in enumerate(tr["names"]) if sel(n)]
                    add = 0.0
                    if tr["extra_idle"] and ch in ("arms+neck",):
                        add = tr["extra_idle"] * SM.I_IDLE
                    elif tr["extra_idle"] and ch == "L leg+L arm+neck":
                        add = 2 * SM.I_IDLE
                    elif tr["extra_idle"] and ch == "R leg+R arm":
                        add = 2 * SM.I_IDLE
                    chain_all[sp][ch].append(i[:, idx].sum(1) + add)
            for k, n in enumerate(tr["names"]):
                per_joint_pk[n] = max(per_joint_pk.get(n, 0.0), float(i[:, k].max()))
        tot = np.concatenate(tot_all)
        label = "motor model (upper bound)" if model == "motor" else "bridge model (lower bound)"
        print(f"-- {label}, bus total, A: {fmt(stats(tot))}", flush=True)
        for sp, chs in SPLITS.items():
            parts = []
            for ch, xs in chain_all[sp].items():
                s = stats(np.concatenate(xs))
                parts.append(f"{ch} pk {s['peak']:.2f} / RMS {s['rms']:.2f} / 2 s {s['w2']:.2f}")
            print(f"   split {sp}: " + ";  ".join(parts), flush=True)
        if model == "motor":
            top = sorted(per_joint_pk.items(), key=lambda kv: -kv[1])[:6]
            print("   hottest servos (peak A): " + ", ".join(f"{n} {v:.2f}" for n, v in top), flush=True)
    worst = max(traces, key=lambda t: currents(t, "motor")[0].sum(1).max())
    print(f"   worst run (motor model peak): {worst['label']}", flush=True)
    return traces


def main():
    print(f"== BUS-CURRENT BUDGET, 17 x STS3215 (Plan B: P x4 on rolls + knees), {VOLTS} V, open-loop traces, "
          f"every 20 ms control tick. Current per servo: motor model I = {1e3 * SM.I_IDLE:.0f} mA + "
          f"{1e3 * (SM.I_NOLOAD - SM.I_IDLE):.0f} mA x |w|/w0 + |tau|/Kt (Kt {SM.KT:.3f} N*m/A), <= "
          f"{SM.I_STALL_12V * VOLTS / 12:.2f} A stall; bridge model I = {1e3 * SM.I_IDLE:.0f} mA + "
          f"(max(tau w,0) + {SM.K_CU} tau^2)/V. Limits: General Driver servo path 5 A continuous (Waveshare FAQ); "
          f"its H1 inlet JST XH 3 A per contact (JST datasheet); a Molex-5264-class servo lead ~3 A (unverified). "
          f"Workers {GN.n_workers()}.", flush=True)
    xml15 = GN.cad_plant(55.0)
    import getup_v6_shoulder as S
    base = dataclasses.replace(S.BASE, **S.CONFIGS["r5_asdrawn"])
    xml_arms = os.path.join(GN.TMP, f"cb_arms_{os.getpid()}.xml")
    with open(xml_arms, "w") as fh:
        fh.write(build_xml(base))
    cases5 = GN.CASES4 + (("turn -15 mu 0.7 play 3", {}, dict(turn_deg=-15.0)),)
    walk_arms = GN.pmap(_walk_arms_job, [(cl, xml_arms, ckw, tkw) for cl, ckw, tkw in cases5])
    ok = [t for t in walk_arms if t["ok"]]
    report_group("WALK, as-drawn robot with arms (r5_asdrawn, 17 servos), 8 steps, the cases that walk", ok)
    for t in walk_arms:
        if not t["ok"]:
            for model in ("motor", "bridge"):
                i, extra = currents(t, model)
                print(f"   ... and the case that falls ({t['label']}), {model} model, bus total incl. the fall: "
                      f"{fmt(stats(i.sum(1) + extra))}", flush=True)
    walk_cad = GN.pmap(_walk_cad_job, [(cl, xml15, ckw, tkw) for cl, ckw, tkw in GN.CASES4])
    report_group("WALK, CAD-inertial all-STS3215 plant (13 servos + 4 idle arm servos), 8 steps, 4 cases", walk_cad)
    for seq_name, seq in GN.GETUP_SEQS.items():
        gu = GN.pmap(_getup_job, [(seq_name, seq, c) for c in GN.GETUP_CONDS])
        report_group(f"GET-UP {seq_name}, six conditions (r5_asdrawn_rom120)", gu)


if __name__ == "__main__":
    main()
