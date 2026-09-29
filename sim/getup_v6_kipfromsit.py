"""Kip FROM THE UPRIGHT SIT (the peak the bridge found: up_z 1.00, legs fwd).

The sit-up already reaches a fully upright torso with straight legs forward.
The remaining step is a KIP: from that high CoM, whip the legs DOWN and fast
(hip extension from -90 toward +, knees tuck), and the reaction rotates the
torso forward and up over the feet; catch the feet under, stand. This is the
classic no-arms 'kip-up to stand' the appended study never attempted because
it started each entry from a low seat, not from the upright sit the deep ROM
just produced.

Phases:
  lie -> sit up (legs straight forward, up_z 1.0) -> EXPLODE: hip extension +
  knee tuck swings the feet down/fast under the torso -> catch a deep crouch
  on the feet -> rise -> stand.

Run: .venv/bin/python sim/getup_v6_kipfromsit.py [--render out.mp4]
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from static_gait import make_env, _write_video
import mujoco

def qq(off):
    q=np.zeros(12)
    for k,v in off.items():
        if k in G.J: q[G.J[k]]=math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]]=math.radians(v)
    return q

def build_seq(swing_h=125, swing_k=-130, catch_h=-125, catch_k=-130, catch_ankle=-20,
              t_swing=0.45, t_catch=0.4, sit_k=10):
    """swing_h = hip extension the feet whip DOWN to (positive); swing_k = knee
    during the swing (tucked)."""
    return [
        ("lie", {}, 0.4, 0.6),
        ("sit up (upright, legs fwd)", dict(hip_pitch=-90, knee=sit_k, ankle=0), 1.4, 0.5),
        ("whip feet down/under (kip)", dict(hip_pitch=swing_h, knee=swing_k, ankle=-20), t_swing, 0.05),
        ("catch deep crouch", dict(hip_pitch=catch_h, knee=catch_k, ankle=catch_ankle), t_catch, 0.4),
        ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30), 0.9, 0.5),
        ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25), 0.9, 0.5),
        ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.0, 1.0),
        ("straight", {}, 0.8, 0.6)]

def run_one(p, env, seq, name, render=None):
    env.fall_up_z=-2; env.fall_height=-1
    m,d=env.model,env.data; d0,hi,lo=env._default,env._hi,env._lo; na=env._nq_act
    def inv(q):
        q=np.concatenate([q,np.zeros(na-len(q))]) if len(q)<na else q[:na]
        return np.clip(np.where(q>=d0,(q-d0)/np.maximum(hi-d0,1e-6),(q-d0)/np.maximum(d0-lo,1e-6)),-1,1)
    qprev=qq(seq[0][1]); G.settle_fallen(env,p,"supine",qprev)
    dt=env.control_dt; frames=[]; t=0.0; k=0
    if render:
        rnd=mujoco.Renderer(m,540,720); cam=mujoco.MjvCamera(); cam.distance,cam.elevation,cam.azimuth=(1.7,-16,95)
    from getup_v6 import contacts_summary
    peak=-1; max_tau=0.0
    for label,off,move_s,hold_s in seq:
        qt=qq(off); nstep=int(move_s/dt)
        for i in range(nstep+int(hold_s/dt)):
            s=min(1.0,(i+1)/max(nstep,1)); s=10*s**3-15*s**4+6*s**5
            q=qprev+(qt-qprev)*s; env.step(inv(q)); t+=dt; k+=1
            up=d.xmat[env._torso_bid].reshape(3,3)[2,2]; peak=max(peak,up)
            max_tau=max(max_tau,float(np.abs(env._servo_tau).max()))
            if render and k%2==0:
                cam.lookat[:]=[d.qpos[0],d.qpos[1],0.2]; rnd.update_scene(d,cam); frames.append(rnd.render().copy())
        qprev=qt
        print(f"  {label:24s} up={up:+.2f} pelZ={d.qpos[2]:.3f} | {contacts_summary(m,d)}")
    up=d.xmat[env._torso_bid].reshape(3,3)[2,2]
    ok=up>0.9 and d.qpos[2]>0.8*p.z_yaw_above_sole
    print(f"[{name}] peak_up {peak:+.2f} max_tau {max_tau:.2f} -> {'STANDING' if ok else 'no'} (up {up:+.2f} z {d.qpos[2]:.3f})")
    if render: _write_video(frames,render,fps=25)
    return ok

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--render",default=None); a=ap.parse_args()
    p=DesignParams(); xml="/tmp/v6_kfs.xml"; open(xml,"w").write(build_xml(p))
    env=make_env(p,xml); obs,_=env.reset(seed=0)
    kp,kd,stall_s,w0_s=env._servo; na=env._nq_act
    def servo():
        stall=np.full(na,float(stall_s)); w0=np.full(na,float(w0_s))
        kpa=np.full(na,float(kp)); kda=np.full(na,float(kd))
        for i,n in enumerate(G.JN):
            if n in G.PJ_DEFAULT:
                s=G.SERVOS["sts3250"]; stall[i]=s["stall"]; w0[i]=s["w0"]; kpa[i]*=s["kp_scale"]; kda[i]*=s["kp_scale"]
        return np.array([kpa,kda,stall,w0],dtype=object)
    variants=[
        (dict(swing_h=125,swing_k=-130,catch_h=-125,catch_k=-130,t_swing=0.5), "big tuck whip"),
        (dict(swing_h=125,swing_k=-80,catch_h=-125,catch_k=-130,t_swing=0.4), "less knee bend"),
        (dict(swing_h=90,swing_k=-130,catch_h=-125,catch_k=-130,t_swing=0.5), "swing to 90"),
        (dict(swing_h=125,swing_k=-30,catch_h=-125,catch_k=-130,t_swing=0.35), "straight-leg whip"),
    ]
    for i,(kw,name) in enumerate(variants):
        env._servo=servo()
        run_one(p,env,build_seq(**kw),name, render=(a.render if i==0 else None))

main()
