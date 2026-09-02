"""Stand-command rollout of a run (teacher or s128 student) in ITS OWN MJX
env -- flat ground, no pushes, no fallen starts -- reporting mean power_w,
up_z and fall rate with the act-lag DR on and off.

The diagnostic that explained v27tilt_b (2026-09-02): calm at 11 W in MJX,
112 W in the lag-less CPU referee -- because act-lag OFF here reproduces
the referee's thrash (96 W). A policy can learn to chatter THROUGH the lag
filter; the referee has no lag, the real servo does (~1.7 Hz). Run from
sim/mjx:  JAX_PLATFORMS=cpu ../../.venv/bin/python stand_probe.py <run>..."""
import sys, os, json, numpy as np, jax, jax.numpy as jp
sys.path.insert(0, os.path.expanduser("~/code/robot-mjx/sim/mjx"))
sys.path.insert(0, os.path.expanduser("~/code/robot-mjx/sim"))
os.chdir(os.path.expanduser("~/code/robot-mjx/sim/mjx"))
from env_mjx import BimoMJXEnv
from eval_precision import load_policy
RUNS = os.path.expanduser("~/code/robot-mjx/sim/runs")

def probe(run, tilt_deg, seeds=6, secs=8.0, lag=True):
    cp = f"{RUNS}/{run}/teacher_config.json"
    cfg = json.load(open(cp if os.path.exists(cp) else f"{RUNS}/{run}/config.json"))
    import inspect
    ok = set(inspect.signature(BimoMJXEnv.__init__).parameters)
    kw = {k: v for k, v in cfg.items() if k in ok}
    kw["xml_path"] = os.path.expanduser("~/code/robot-mjx/sim/") + os.path.basename(kw["xml_path"])
    kw.update(terrain=False, push_prob=0.0, recover_mix=0.0, getup=False, imu_obs=True, ext_cmd=True,
              command_mode=True, payload_mass=0.154, payload_max=None,
              cmd_fixed=None, episode_seconds=12.0)
    if not lag:
        kw.update(act_lag_hz=0.0, act_lag_hz_max=None)
    kw = {k: v for k, v in kw.items() if k in ok}
    env = BimoMJXEnv(**kw)
    act = load_policy(f"{RUNS}/{run}", env.obs_size, env.action_size)
    th = np.deg2rad(tilt_deg)
    g = jp.array([9.81 * np.sin(th), 0.0, -9.81 * np.cos(th)])
    model = env.model.tree_replace({"opt.gravity": g})
    step = jax.jit(lambda s, a: env.step(s, a, model))
    reset = jax.jit(lambda r: env.reset(r, model))
    n = int(secs / env.control_dt)
    out = []
    for s in range(seeds):
        st = reset(jax.random.PRNGKey(100 + s))
        cmd = jp.zeros_like(st.cmd).at[3].set(1.0)      # plain stand, full height
        st = st._replace(cmd=cmd)
        P, U, F = [], [], 0.0
        for i in range(n):
            a = act(np.asarray(st.obs))
            st = step(st, jp.asarray(a))
            st = st._replace(cmd=cmd)
            if i >= n // 4:                              # skip settle
                P.append(float(st.metrics["power_w"])); U.append(float(st.metrics["up_z"]))
            F = max(F, float(st.metrics["fell"]))
        out.append((np.mean(P), np.mean(U), F))
    o = np.array(out)
    print(f"{run:16s} lag={'on ' if lag else 'OFF'} tilt={tilt_deg:>3.0f}deg  power {o[:,0].mean():6.1f} W  up_z {o[:,1].mean():.4f}  fell {o[:,2].mean():.2f}  (dt={env.control_dt:.3f}, {seeds} seeds x {secs:.0f}s)", flush=True)

for run in sys.argv[1:]:
    probe(run, 0.0, lag=True)
    probe(run, 0.0, lag=False)
