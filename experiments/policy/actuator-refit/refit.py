"""Refit the STS3215 torque-speed envelope (stall torque, no-load speed) from the robot's
hanging 60 deg max-speed moves (2026-11-03), on the plant's own servo law.

  CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu python refit.py [moves.json] [out.json]
      RIDGE=1,1.5,2,3 DEADS=20,30,40,50   the stall-ratio ridge x the stride's servo dead time
      FREE=1                              the free (stall, w0, dead) fit per role instead

The plant (sim/walker_env, mirrored by env_mjx) drives every servo DOF with
    tau = clip(kp (s - q) - kd qd, -cap, cap),  cap = stall (1 - |qd| / w0)+
per 2 ms substep, stall = 2.94 N*m x V/12 x stall_ratio, w0 = 4.04 rad/s x V/12 x w0_ratio.
The robot's large moves run at full duty (present load 1000) at 240-275 deg/s while lifting
the leg, which this law cannot do at w0 4.04 rad/s (231 deg/s) -- so the ratios are refit,
per role (hip_pitch, knee), on the HANGING fixture: the plant XML with the torso welded in
the air, the moved joint servoed (set point = goal after the servo's dead time), every other
joint limp (walker_env's released state: no torque, frictionloss off_frictionloss).

The fitted numbers feed:
  training   train_mjx.py --supply-voltage 12 --servo-envelope hip_pitch=S:W,knee=S:W
  grader     grade.py / friction-dr sweep.py: plant {"supply_v": 12, "stall_scale": {...},
             "w0_scale": {...}} (simh.apply_model)
and are checked against the robot's stride1 swing (R hip pitch -11/-21/-31 deg at +100/140/180
ms, flight 2026-11-03 walk1_w18shp) by stride_check (grade.py's firmware-path episode). The
moves alone pin a ridge, not a point; the stride picks it (refit.json: the chosen point).

The fixture must zero the MJCF position actuators (kp 40, the 'ideal' model) as walker_env's
sts3215 model does: left on, they hold the leg near zero and every envelope fits absurdly."""
from __future__ import annotations
import json, math, os, sys
import numpy as np
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rca"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")
import mujoco  # noqa: E402
import rec, simh  # noqa: E402

MOVES = "/home/claw/code/robot/hw_sessions/2026-11-03/moves_fast_large.json"
XML = os.path.join(rec.ROOT, "sim", "bimo_biped_v6legs_meas.xml")
RUN = "robot_legs_walk18_shp10sq_s128r24"
STALL12, W012 = 2.94, 4.04            # walker_env _STS_STALL_12V, _STS_NOLOAD_12V
V_ROBOT = 12.0                        # the servos' bus volts on the board (flight 2026-11-03)
T_FIT = (0.0, 0.42)                   # s: the move and its settle
ROLES = {"hip_pitch": ["R_hip_pitch_out", "L_hip_pitch_out"], "knee": ["R_knee_out", "L_knee_out"]}


def plant_params():
    """The graded plant's servo + joint constants (simh 'fit' plant + JF_PLANT), per joint."""
    env, _, _ = simh.build("fit", run=RUN, ep_s=4.0)
    env.domain_rand = False
    env.reset(seed=0)
    simh.apply_model(env, simh.JF_PLANT)
    names = list(env._servo_names)
    P = dict(names=names, kp=env._servo[0] * np.asarray(env._kp_prof),
             kd=env._servo[1] * np.asarray(env._kd_prof),
             stall_prof=np.asarray(env._stall_prof, float), w0_prof=np.asarray(env._w0_prof, float),
             damping=np.array(env.model.dof_damping[env._sqvel]),
             frictionloss=np.array(env.model.dof_frictionloss[env._sqvel]),
             armature=np.array(env.model.dof_armature[env._sqvel]),
             off_frictionloss=float(env.off_frictionloss))
    return P


def cap_shape(w, w0, knee):
    """walker_env/env_mjx torque-speed shape: flat at stall up to knee x w0, then linear to
    zero at w0 (knee 0 = the DC-motor line)."""
    return min(1.0, max(0.0, (w0 - w) / ((1.0 - knee) * w0)))


class Hanging:
    """The plant with the torso welded 0.6 m up; the leg joints carry the graded plant's
    damping / armature, the limp ones the released frictionloss."""
    def __init__(self, P, xml=XML):
        sp = mujoco.MjSpec.from_file(xml)
        tb = sp.body("torso")
        for j in list(tb.joints):
            sp.delete(j)
        tb.pos = [0.0, 0.0, 0.6]
        self.m = sp.compile()
        self.d = mujoco.MjData(self.m)
        self.P = P
        self.jq = np.array([self.m.joint(n).qposadr[0] for n in P["names"]])
        self.jv = np.array([self.m.joint(n).dofadr[0] for n in P["names"]])
        # the MJCF's position actuators (kp 40) are the "ideal" actuator model; the sts3215
        # plant zeroes them and drives qfrc_applied (walker_env) -- so must the fixture
        self.m.actuator_gainprm[:] = 0.0
        self.m.actuator_biasprm[:] = 0.0
        self.m.dof_damping[self.jv] = P["damping"]
        self.m.dof_armature[self.jv] = P["armature"]
        self.dt = self.m.opt.timestep

    def move(self, joint, q_goal, t_s, ratio, dead_ms, tau_ms=0.0, volts=V_ROBOT, knee=0.0):
        """q (deg) of `joint` at times t_s (s from the goal write) for one max-speed goal."""
        P, m, d = self.P, self.m, self.d
        mujoco.mj_resetData(m, d)
        i = P["names"].index(joint)
        fl = np.full(len(P["names"]), P["off_frictionloss"]); fl[i] = P["frictionloss"][i]
        m.dof_frictionloss[self.jv] = fl
        mujoco.mj_forward(m, d)
        stall = STALL12 * volts / 12.0 * P["stall_prof"][i] * ratio[0]
        w0 = W012 * volts / 12.0 * P["w0_prof"][i] * ratio[1]
        kp, kd = P["kp"][i], P["kd"][i]
        a, v = self.jq[i], self.jv[i]
        q0 = float(d.qpos[a]); s = q0
        n = int(math.ceil(max(t_s) / self.dt)) + 1
        out = np.empty(n)
        k_tau = 1.0 - math.exp(-self.dt / (tau_ms / 1000.0)) if tau_ms > 0 else 1.0
        for k in range(n):
            t = k * self.dt
            goal = q_goal if t >= dead_ms / 1000.0 else q0
            s += k_tau * (goal - s)
            q, qd = float(d.qpos[a]), float(d.qvel[v])
            cap = stall * cap_shape(abs(qd), w0, knee)
            d.qfrc_applied[:] = 0.0
            d.qfrc_applied[v] = float(np.clip(kp * (s - q) - kd * qd, -cap, cap))
            mujoco.mj_step(m, d)
            out[k] = float(d.qpos[a])
        tt = np.arange(n) * self.dt
        return np.degrees(np.interp(t_s, tt, out))


def load_moves(path=MOVES):
    raw = json.load(open(path))
    out = {}
    for role, keys in ROLES.items():
        for k in keys:
            t = np.array(raw[k]["t"]); q = np.array(raw[k]["q"])
            w = (t >= T_FIT[0]) & (t <= T_FIT[1])
            out[k] = dict(joint=k[:-4], t=t[w], q=q[w] - q[t <= 0][-1], goal=float(np.round(q[-1] - q[t <= 0][-1], 0)))
    return out


def fit_role(H, mv, keys, dead_ms=None):
    def cost(x):
        r = np.exp(x[:2]); dm = x[2] if dead_ms is None else dead_ms
        e = []
        for k in keys:
            m = mv[k]
            q = H.move(m["joint"], math.radians(m["goal"]), m["t"], r, dm)
            e.append(q - m["q"])
        return float(np.sqrt(np.mean(np.concatenate(e) ** 2)))
    x0 = np.array([0.0, math.log(1.3), 20.0])[:2 if dead_ms is not None else 3]
    best = None
    for s0 in (0.0, 0.4):                       # two starts: the stall/w0 trade is shallow
        x = x0.copy(); x[0] = s0
        r = minimize(cost, x, method="Nelder-Mead",
                     options=dict(xatol=1e-3, fatol=1e-3, maxiter=400, initial_simplex=None))
        if best is None or r.fun < best.fun:
            best = r
    st, w0 = np.exp(best.x[:2])
    return dict(stall_ratio=float(st), w0_ratio=float(w0),
                dead_ms=float(best.x[2] if dead_ms is None else dead_ms), rmse_deg=float(best.fun))


# the robot's stride1 (flight 2026-11-03 walk1_w18shp), R hip pitch / R knee deg at the first
# 10 ticks of the walk command (tick 0 = the first walk goal), where the sim's goals still match
ROBOT_STRIDE = dict(hip=[1, 1, 0, -3, -7, -11, -16, -21, -26, -31],
                    knee=[4, 4, 3, 0, -5, -10, -15, -20, -26, -31])


def fit_ridge(H, mv, keys, stall):
    """Hanging moves at a FIXED stall ratio: (w0 ratio, dead ms, rmse). The 60 deg moves pin
    a ridge, not a point: any stall >= ~2x fits them equally (rmse ~3.6-4.0 deg) with w0
    falling toward the plateau speed as the stall rises -- the stride1 swing picks the point."""
    def cost(x):
        e = []
        for k in keys:
            m = mv[k]
            e.append(H.move(m["joint"], math.radians(m["goal"]), m["t"], (stall, math.exp(x[0])), x[1]) - m["q"])
        return float(np.sqrt(np.mean(np.concatenate(e) ** 2)))
    r = minimize(cost, [math.log(1.3), 15.0], method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-3))
    return float(math.exp(r.x[0])), float(r.x[1]), float(r.fun)


def stride_check(plant, dead15, run=RUN):
    """grade.py's firmware-path stride1 0.6 (seed 0, mu 1) on `plant` with the fast servo's dead
    time dead15: R hip / knee over the first 10 walk ticks, and their rmse to the robot's."""
    sys.path.insert(0, os.path.join(HERE, "..", "shaper-train"))
    import grade as G
    v = dict(run=run, shaper_hz=10.0, stand_phase=G.P2, clock="coast", cmd_schedule=G.stride(0.6),
             plant=plant, typed=dict(G.FAST, dead15=float(dead15)), trace=True)
    m = G.episode(v, seed=0, run_s=2.6)
    q = np.array(m["trace"]["q"])
    hip, knee = q[:10, 8], q[:10, 9]
    rh = np.array(ROBOT_STRIDE["hip"], float); rk = np.array(ROBOT_STRIDE["knee"], float)
    e = np.concatenate([(hip - hip[0]) - (rh - rh[0]), (knee - knee[0]) - (rk - rk[0])])
    return dict(hip=hip.round(1).tolist(), knee=knee.round(1).tolist(), rmse=float(np.sqrt(np.mean(e ** 2))),
                yaw180=m.get("walk_yaw_180ms"))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else MOVES
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "refit.json")
    P = plant_params()
    H = Hanging(P)
    mv = load_moves(path)
    if not os.environ.get("FREE"):       # the ridge x stride1 grid (FREE=1: the free 3-parameter fit)
        grid = []
        for st in [float(x) for x in os.environ.get("RIDGE", "1.0,1.5,2,3").split(",")]:
            fr = {role: fit_ridge(H, mv, keys, st) for role, keys in ROLES.items()}
            plant = dict(supply_v=V_ROBOT, stall_scale={r: st for r in fr},
                         w0_scale={r: round(fr[r][0], 4) for r in fr})
            for dead in [float(x) for x in os.environ.get("DEADS", "20,30,40,50").split(",")]:
                sc = stride_check(plant, dead)
                row = dict(stall=st, hang={r: dict(w0_ratio=fr[r][0], dead_ms=fr[r][1], rmse=fr[r][2]) for r in fr},
                           stride_dead15=dead, plant=plant, stride=sc)
                grid.append(row)
                print(f"stall x{st:g}: " + ", ".join(f"{r} w0 x{fr[r][0]:.3f} ({math.degrees(W012 * fr[r][0]):.0f} deg/s) "
                      f"hang rmse {fr[r][2]:.2f}" for r in fr) + f" | dead15 {dead:g}: stride rmse {sc['rmse']:.2f} "
                      f"hip {sc['hip'][-1]:+.0f} knee {sc['knee'][-1]:+.0f}", flush=True)
        best = min(grid, key=lambda g: g["stride"]["rmse"] + max(g["hang"][r]["rmse"] for r in g["hang"]))
        res = dict(source=path, volts=V_ROBOT, grid=grid, best=best,
                   servo_envelope_flag=",".join(f"{r}={best['stall']:.3f}:{best['plant']['w0_scale'][r]:.3f}"
                                                for r in best["plant"]["w0_scale"]),
                   grader_plant=best["plant"], grader_fast_dead15=best["stride_dead15"])
        json.dump(res, open(out, "w"), indent=1)
        print("best:", json.dumps({k: res[k] for k in ("servo_envelope_flag", "grader_plant", "grader_fast_dead15")}))
        return
    res = dict(source=path, volts=V_ROBOT, law="tau = clip(kp e - kd qd, +-stall (1 - |qd|/w0)+)",
               nominal=dict(stall_Nm_12V=STALL12, w0_rad_s_12V=W012), roles={})
    base = {}
    for role, keys in ROLES.items():
        f = fit_role(H, mv, keys)
        f["stall_Nm"] = STALL12 * V_ROBOT / 12 * f["stall_ratio"]
        f["w0_rad_s"] = W012 * V_ROBOT / 12 * f["w0_ratio"]
        # the plant as it is (11.1 V, ratio 1) on the same moves, for the record
        b = []
        for k in keys:
            m = mv[k]
            q = H.move(m["joint"], math.radians(m["goal"]), m["t"], (1.0, 1.0), f["dead_ms"], volts=11.1)
            b.append(dict(key=k, t=m["t"].tolist(), robot=m["q"].tolist(), plant=q.tolist(),
                          refit=H.move(m["joint"], math.radians(m["goal"]), m["t"],
                                       (f["stall_ratio"], f["w0_ratio"]), f["dead_ms"]).tolist()))
        f["traces"] = b
        res["roles"][role] = f
        print(f"{role}: stall x{f['stall_ratio']:.3f} ({f['stall_Nm']:.2f} N*m), w0 x{f['w0_ratio']:.3f} "
              f"({f['w0_rad_s']:.2f} rad/s = {math.degrees(f['w0_rad_s']):.0f} deg/s), dead {f['dead_ms']:.1f} ms, "
              f"rmse {f['rmse_deg']:.2f} deg")
        for tr in b:
            t = np.array(tr["t"])
            sel = [int(np.argmin(np.abs(t - x))) for x in (0.1, 0.18, 0.26)]
            print("   ", tr["key"], " ".join(f"{t[j]*1000:.0f}ms robot {tr['robot'][j]:+.0f} plant {tr['plant'][j]:+.0f} "
                                           f"refit {tr['refit'][j]:+.0f}" for j in sel))
    r = res["roles"]
    res["servo_envelope_flag"] = ",".join(f"{k}={r[k]['stall_ratio']:.3f}:{r[k]['w0_ratio']:.3f}" for k in r)
    res["grader_plant"] = dict(supply_v=V_ROBOT, stall_scale={k: round(r[k]["stall_ratio"], 3) for k in r},
                               w0_scale={k: round(r[k]["w0_ratio"], 3) for k in r})
    json.dump(res, open(out, "w"), indent=1)
    print("train_mjx: --supply-voltage 12 --servo-envelope", res["servo_envelope_flag"])
    print("grader PLANT:", json.dumps(res["grader_plant"]))


if __name__ == "__main__":
    main()
