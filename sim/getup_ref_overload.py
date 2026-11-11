"""The STS overload cutoff on the CURRENT get-up references (#126, #83, #6; 2026-11-10).

The 2026-09-29 record (docs/design-v6/getup-overload-2026-09-29.md) put the
documented STS protection (sim/sts_servo_model.py: load > 80 % of the servo's
maximum held 2.0 s -> 20 % torque) into the scripted as-drawn seat push on the
lumped r5 plant with Plan B servos. This re-runs it on what the get-up is
now: the campaign's analytic references (sim/getup_ref_analytic.py, #137),
replayed open loop from their recorded npz targets, on the robot's plant:

  plant    sim/bimo_biped_v6ar_meas.xml (CAD inertials, the measured joint
           stops, the sole pads; the get-up policies' training plant,
           robot_getup_w19k), the get-up studies' deploy servo model
           (getup_v6 / static_gait.make_env: 2 Hz lag, 80 ms dead time,
           1 deg backlash, 3 deg play on the rolls, integer ticks)
  servos   STS3250 at the hip rolls, STS3215 elsewhere (knees included),
           position-loop stiffness as built (walker_env SERVO_KP_PRESETS
           "asbuilt": 4x hip rolls, 2.8x the other leg servos; arms 1x)
  envelope "11.1V": the get-up's training envelope (w19k, --supply-voltage 11.1)
           "12V-refit": #214's 12 V bus, hip pitch and knee refit to stall
           x2.0, no-load x1.325 / x1.281 (experiments/policy/actuator-refit)
  refs     prone_ikc  sim/getup_ref_v6ar_prone_ikc.npz: the reference the
                      get-up candidate (w19k family) tracks
           supine_ik  sim/getup_ref_v6ar_ik.npz: the supine seat push with
                      the analytic rise (the one #83 names)

The load is a fraction of each servo's OWN maximum in the plant (so the
refit's x2.0 stall halves a refit joint's torque reading; the shoulder and
elbow are never refit). Both readings of "load" (torque: |tau| / stall;
duty: |tau / stall + w / w0|) are always measured. A measuring run never
changes the plant; any condition whose longest continuous time above 80 %
reaches the 2.0 s timer is re-run with the cutoff enforced (latched).

    .venv/bin/python sim/getup_ref_overload.py run > docs/design-v6/getup_overload_2026-11-10.txt

Workers: SIM_WORKERS, else 8 (mira shares its cores with the night trainer).
"""
from __future__ import annotations

import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

import getup_v6 as G  # noqa: E402
import getup_ref_analytic as R  # noqa: E402
import sts_servo_model as SM  # noqa: E402
from walker_env import SERVO_KP_PRESETS, SERVO_TYPES  # noqa: E402

XML = os.path.join(HERE, "bimo_biped_v6ar_meas.xml")
REFS = {"prone_ikc": ("prone", "getup_ref_v6ar_prone_ikc.npz"),
        "supine_ik": ("supine", "getup_ref_v6ar_ik.npz")}
STALL_12V, W0_12V = 2.94, 4.04          # the STS3215 walker_env models (N*m, rad/s at 12 V)
# (volts, {role: (stall ratio, no-load ratio)}) -- ratios to the STS3215 at that voltage
ENVELOPES = {
    "11.1V": (11.1, {"hip_roll": SERVO_TYPES["sts3250"]}),
    "12V-refit": (12.0, {"hip_roll": SERVO_TYPES["sts3250"], "hip_pitch": (2.0, 1.325), "knee": (2.0, 1.281)}),
}
KP_SCALE = SERVO_KP_PRESETS["asbuilt"]
NOMINAL = dict(R.NOMINAL)               # joint friction 0.35, mu 0.7, play 3, servos 100 %, mass 100 %, no tilt
CONDS = [("nominal", NOMINAL),
         ("joint friction 0.05", dict(NOMINAL, fric=0.05)),
         ("joint friction 0.5", dict(NOMINAL, fric=0.5)),
         ("mu 0.45 (the measured sole)", dict(NOMINAL, mu=0.45)),
         ("mu 1.0", dict(NOMINAL, mu=1.0)),
         ("play 1", dict(NOMINAL, play=1.0)),
         ("servos 85 %", dict(NOMINAL, stall_k=0.85)),
         ("mass 110 %", dict(NOMINAL, mass_k=1.1)),
         ("servos 85 %, mass 110 %, mu 1.0", dict(NOMINAL, stall_k=0.85, mass_k=1.1, mu=1.0)),
         ("servos 65 % (09-29's weakest)", dict(NOMINAL, stall_k=0.65))]
ROLES = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll", "neck_yaw", "shoulder", "elbow")


def _role(jn):
    return jn[2:] if jn[:2] in ("L_", "R_") else jn


def make_env(cond, envelope):
    """getup_ref_analytic.make_env on the robot's plant, servo set and envelope"""
    p = R._plant()
    env = G.make_env(p, XML, mu=cond["mu"], play_deg=cond["play"])
    env.reset(seed=0)
    G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    volts, types = ENVELOPES[envelope]
    kp, kd, _, _ = env._servo
    na = env._nq_act
    names = [env.model.actuator(i).name for i in range(na)]
    stall = np.full(na, STALL_12V * volts / 12.0)
    w0 = np.full(na, W0_12V * volts / 12.0)
    kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
    for i, n in enumerate(names):
        rs, rw = types.get(_role(n), (1.0, 1.0))
        stall[i] *= rs; w0[i] *= rw
        ks = KP_SCALE.get(_role(n), 1.0)
        kpa[i] *= ks; kda[i] *= ks
    env._servo = np.array([kpa, kda, stall * cond["stall_k"], w0], dtype=object)
    env.fall_up_z = -2.0; env.fall_height = -1.0
    if cond["mass_k"] != 1.0:
        env.model.body_mass[:] *= cond["mass_k"]
    env.model.dof_frictionloss[env._sqvel] = cond["fric"]
    return env, p, names, volts


def run(ref, cond, envelope, time_scale=1.0, enforce=None):
    """replay ref's recorded targets open loop (time_scale > 1: slower);
    enforce None = measure only, else the reading the cutoff acts on"""
    start, npz = REFS[ref]
    z = np.load(os.path.join(HERE, npz))
    env, p, names, volts = make_env(cond, envelope)
    order = [list(z["joint_names"]).index(n) for n in names]
    q_ref = np.asarray(z["q"], dtype=float)[:, order]
    dt, d, inv = env.control_dt, env.data, R._inv(env)
    n = int(round(len(q_ref) * time_scale))
    tq = np.arange(len(q_ref)) * float(z["control_dt"]) * time_scale
    t_new = np.arange(n) * dt
    q_seq = np.stack([np.interp(t_new, tq, q_ref[:, j]) for j in range(q_ref.shape[1])], 1)
    G.settle_fallen(env, p, start, q_seq[0])
    prot = SM.STSProtection(env, names, enforce=enforce is not None, load=enforce or "duty",
                            overcurrent_a=SM.OVERCURRENT_A_DS, volts=volts)
    stood_t = None
    for k in range(n):
        env.step(inv(q_seq[k]))
        prot.tick(env, dt)
        if stood_t is None:
            up, _ = R.torso_axes(env)
            if up > 0.9 and d.qpos[2] > 0.8 * p.z_yaw_above_sole:
                stood_t = round((k + 1) * dt, 1)
    for _ in range(int(2.0 / dt)):     # 2 s more at the last target: still standing?
        env.step(inv(q_seq[-1]))
        prot.tick(env, dt)
    up, front = R.torso_axes(env)
    ok = bool(up > 0.9 and d.qpos[2] > 0.8 * p.z_yaw_above_sole)
    stall_eff = dict(zip(names, prot.stall0))
    return dict(ok=ok, up=up, z=float(d.qpos[2]), stood_t=stood_t, prot=prot.report(), stall=stall_eff)


def _job(a):
    try:
        return run(*a)
    except Exception as e:      # an unstable rollout is a failure, never kills a sweep
        return dict(ok=False, up=-1.0, z=0.0, stood_t=None, prot={}, stall={}, err=repr(e))


def pmap(jobs):
    n = int(os.environ.get("SIM_WORKERS", "8"))
    with Pool(min(n, len(jobs))) as pool:
        return pool.map(_job, jobs)


def role_rows(results, reading):
    """per role: max over L/R and over the results of peak load, longest and
    total time above 80 % on `reading`, peak torque (N*m), peak motor current
    and the longest time above the datasheet's 2 A over-current line"""
    rows = {}
    for r in results:
        for jn, v in r["prot"].items():
            row = rows.setdefault(_role(jn), dict(peak=0.0, cont=0.0, cum=0.0, nm=0.0, i=0.0, oc=0.0))
            if v[f"peak_{reading}"] > row["peak"]:
                row["peak"] = v[f"peak_{reading}"]
            row["cont"] = max(row["cont"], v[f"cont_max_{reading}"])
            row["cum"] = max(row["cum"], v[f"cum_{reading}"])
            row["nm"] = max(row["nm"], v["peak_torque"] * r["stall"][jn])
            row["i"] = max(row["i"], v["peak_i"])
            row["oc"] = max(row["oc"], v["oc_cont_max"])
    return rows


def print_rows(rows, reading, indent="   "):
    print(f"{indent}{'role':11s} {'peak load':>9s} {'peak N*m':>8s} {'longest >80%':>12s} {'total >80%':>10s} "
          f"{'peak I':>7s} {'longest >2A':>11s}   [{reading}]", flush=True)
    for role in ROLES:
        if role in rows:
            r = rows[role]
            print(f"{indent}{role:11s} {100 * r['peak']:8.0f}% {r['nm']:8.2f} {r['cont']:11.2f}s {r['cum']:9.2f}s "
                  f"{r['i']:6.2f}A {r['oc']:10.2f}s", flush=True)


def _hot(rep, reading):
    jn, v = max(rep.items(), key=lambda kv: kv[1][f"cont_max_{reading}"])
    return jn, v[f"cont_max_{reading}"], v[f"peak_{reading}"]


def main():
    print("== The STS overload cutoff on the current get-up references (getup_ref_overload.py, 2026-11-10)\n"
          f"plant {os.path.basename(XML)}; servos STS3250 hip rolls, STS3215 elsewhere; kp as built "
          f"{KP_SCALE}; arms 1x. Cutoff (memory table V3.7 defaults): load > {100 * SM.OVERLOAD_FRAC:.0f} % "
          f"for {SM.PROTECT_TIME_S:.1f} s -> {100 * SM.PROTECT_FRAC:.0f} %; over-current {SM.OVERCURRENT_A_DS:.1f} A "
          f"for {SM.OVERCURRENT_TIME_S:.0f} s (STS3215 motor-current model, not meaningful for the STS3250).\n"
          "Measuring runs (cutoff not enforced); any run whose longest continuous stretch above 80 % reaches "
          "2.0 s is re-run enforced on that reading.", flush=True)
    for env_name, (volts, types) in ENVELOPES.items():
        print(f"\n######## envelope {env_name}: {volts} V, ratios to the STS3215 {types}", flush=True)
        for ref in REFS:
            for ts in (1.0, 3.0):
                jobs = [(ref, c, env_name, ts, None) for _, c in CONDS]
                res = pmap(jobs)
                n_ok = sum(r["ok"] for r in res)
                print(f"\n-- {ref} ({REFS[ref][1]}), pace x{ts:.0f}: stands {n_ok}/{len(res)} open loop", flush=True)
                trip = []
                for (lab, c), r in zip(CONDS, res):
                    if "err" in r:
                        print(f"   {lab:34s} ERROR {r['err']}", flush=True)
                        continue
                    sh = max(r["prot"][j]["peak_torque"] for j in ("L_shoulder", "R_shoulder"))
                    sh_nm = max(r["prot"][j]["peak_torque"] * r["stall"][j] for j in ("L_shoulder", "R_shoulder"))
                    ht, hd = _hot(r["prot"], "torque"), _hot(r["prot"], "duty")
                    sht, shd = (max(r["prot"][j][f"cont_max_{k}"] for j in ("L_shoulder", "R_shoulder"))
                                for k in ("torque", "duty"))
                    print(f"   {lab:34s} {'STANDS' if r['ok'] else 'no    '} (up at {r['stood_t']} s, up {r['up']:+.2f})"
                          f"  shoulder peak {100 * sh:3.0f} % ({sh_nm:.2f} N*m), >80 % {sht:.2f} s / {shd:.2f} s duty"
                          f"  longest >80 %: torque {ht[0]} {ht[1]:.2f} s, duty {hd[0]} {hd[1]:.2f} s", flush=True)
                    for reading, h in (("torque", ht), ("duty", hd)):
                        if h[1] >= SM.PROTECT_TIME_S:
                            trip.append((lab, c, reading))
                full = [r for (_, c), r in zip(CONDS, res) if c["stall_k"] == 1.0 and "err" not in r]
                sh = [r["prot"][j] | dict(nm=r["prot"][j]["peak_torque"] * r["stall"][j])
                      for r in full for j in ("L_shoulder", "R_shoulder")]
                print(f"   SHOULDER at full strength ({len(full)} conditions): peak load "
                      f"{100 * min(max(a['peak_torque'] for a in sh[k:k + 2]) for k in range(0, len(sh), 2)):.0f}-"
                      f"{100 * max(a['peak_torque'] for a in sh):.0f} % of stall, max {max(a['nm'] for a in sh):.2f} N*m; "
                      f"longest >80 %: torque {max(a['cont_max_torque'] for a in sh):.2f} s, duty "
                      f"{max(a['cont_max_duty'] for a in sh):.2f} s (timer 2.0 s); peak motor current "
                      f"{max(a['peak_i'] for a in sh):.2f} A, longest >2 A {max(a['oc_cont_max'] for a in sh):.2f} s",
                      flush=True)
                for reading in ("torque", "duty"):
                    print_rows(role_rows([r for r in res if "err" not in r], reading), reading)
                if trip:
                    enf = pmap([(ref, c, env_name, ts, reading) for _, c, reading in trip])
                    print("   cutoff ENFORCED where a timer reached 2.0 s:", flush=True)
                    for (lab, _, reading), r in zip(trip, enf):
                        trips = sorted((jn, v["trip_t"]) for jn, v in r["prot"].items() if v["trip_t"] is not None)
                        print(f"     {lab:34s} [{reading}] {'STANDS' if r['ok'] else 'no    '}  trips: "
                              + (", ".join(f"{j} @ {t:.1f} s" for j, t in trips) or "none"), flush=True)
    # the margin in servo strength: nominal condition, the w19k reference, both envelopes
    print("\n-- margin in servo strength: prone_ikc, nominal condition, pace x1, every servo's stall x scale "
          "(no-load speed unchanged); the cutoff measured, then enforced on each reading", flush=True)
    scales = (1.0, 0.85, 0.75, 0.65, 0.55, 0.45)
    for env_name in ENVELOPES:
        jobs = [("prone_ikc", dict(NOMINAL, stall_k=s), env_name, 1.0, enf)
                for s in scales for enf in (None, "torque", "duty")]
        res = pmap(jobs)
        for k, s in enumerate(scales):
            m, et, ed = res[3 * k: 3 * k + 3]
            if "err" in m:
                print(f"   {env_name:9s} servos {s:.2f}: ERROR {m['err']}", flush=True)
                continue
            ht, hd = _hot(m["prot"], "torque"), _hot(m["prot"], "duty")
            sh = max(m["prot"][j]["peak_torque"] for j in ("L_shoulder", "R_shoulder"))

            def _tr(r):
                t = sorted((jn, v["trip_t"]) for jn, v in r["prot"].items() if v["trip_t"] is not None)
                return ", ".join(f"{j} @ {x:.1f} s" for j, x in t) or "no trip"
            print(f"   {env_name:9s} servos {s:.2f}: no cutoff {'STANDS' if m['ok'] else 'no    '} "
                  f"(shoulder {100 * sh:.0f} %; longest >80 % torque {ht[0]} {ht[1]:.2f} s, duty {hd[0]} {hd[1]:.2f} s)"
                  f" | enforced torque {'STANDS' if et['ok'] else 'no    '} ({_tr(et)})"
                  f" | enforced duty {'STANDS' if ed['ok'] else 'no    '} ({_tr(ed)})", flush=True)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "run":
        main()
    else:
        print(__doc__)
