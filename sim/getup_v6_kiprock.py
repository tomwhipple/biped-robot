"""Kip with a rock-back run-up (the gymnast's kip, arms-free). To build enough
angular velocity for the whip, the robot first rocks BACKWARD from the sit onto
its shoulders/head (raising the legs and CoM), then reverses: legs whip DOWN-
FORWARD and UNDER as the torso pivots up over the feet. Two-part dynamic kip.

The appendage study never tried this because it started each entry from a low
seat; we start from the upright sit (up_z 1.0) the deep ROM produces, and use
the boxy head+shoulders as the backward-fulcrum (exactly what a human gymnast
uses). Success = a caught deep crouch on the feet -> rise -> stand.

Phases: lie -> sit up -> (a) rock back onto head/shoulders, legs up (build
momentum); (b) whip legs down-forward + tuck under (reversal); (c) catch a
deep crouch; (d) rise; (e) stand.

Run: .venv/bin/python sim/getup_v6_kiprock.py [--render out.mp4] [--fast 1]
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

def build_seq(rock_h=125, rock_k=-30, whip_h=125, whip_k=-130, catch_h=-125, catch_k=-130,
              catch_ankle=-20, t_rock=0.7, t_whip=0.5, t_catch=0.4):
    return [
        ("lie", {}, 0.4, 0.6),
        ("sit up (upright, legs fwd)", dict(hip_pitch=-90, knee=10, ankle=0), 1.4, 0.5),
        ("rock back onto head, legs up", dict(hip_pitch=rock_h, knee=rock_k, ankle=0), t_rock, 0.1),
        ("whip legs down/under (reversal)", dict(hip_pitch=whip_h, knee=whip_k, ankle=-20), t_whip, 0.05),
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
        rnd=mujoco.Renderer(m,540,720); cam=mujoco.MjvCamera(); cam.distance,cam.elevation,cam.azimuth=(1.8,-18,95)
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
        print(f"  {label:28s} up={up:+.2f} pelZ={d.qpos[2]:.3f} | {contacts_summary(m,d)}")
    up=d.xmat[env._torso_bid].reshape(3,3)[2,2]
    ok=up>0.9 and d.qpos[2]>0.8*p.z_yaw_above_sole
    print(f"[{name}] peak_up {peak:+.2f} max_tau {max_tau:.2f} -> {'STANDING' if ok else 'no'} (up {up:+.2f} z {d.qpos[2]:.3f})")
    if render: _write_video(frames,render,fps=25)
    return ok

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--render",default=None); a=ap.parse_args()
    p=DesignParams(); xml="/tmp/v6_kiprock.xml"; open(xml,"w").write(build_xml(p))
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
        (dict(rock_h=125,rock_k=-30,whip_h=125,whip_k=-130,t_rock=0.7,t_whip=0.5), "rock then big whip"),
        (dict(rock_h=125,rock_k=-10,whip_h=110,whip_k=-130,t_rock=0.8,t_whip=0.45), "shallower rock"),
        (dict(rock_h=100,rock_k=-50,whip_h=125,whip_k=-130,t_rock=0.6,t_whip=0.5),  "rock-100"),
        (dict(rock_h=140,rock_k=-20,whip_h=120,whip_k=-130,t_rock=0.8,t_whip=0.5),  "deep rock"),
    ]
    for i,(kw,name) in enumerate(variants):
        env._servo=servo()
        run_one(p,env,build_seq(**kw),name, render=(a.render if i==0 else None))

main()
