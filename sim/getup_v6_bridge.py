"""Bridge-to-stand (no appendage). From supine: sit up (up_z 0.99 is reachable),
then rock BACK so the head / back-of-torso contact the floor behind the hips,
with the feet tucked to the buttocks. That makes a head+feet tripod (a rigid
'bridge'). Then drive the hips FORWARD and up; because the CoM between head and
feet is already inside the foot when the head lifts, the rise becomes a forward
rock onto the feet -- the classic no-arms wrestler's escape. The boxy torso and
the deep knee/hip flex (feet to buttock) are exactly what a human bridge needs.

Phases:
  lie -> sit up (legs forward, counterweight) -> tuck feet to buttocks ->
  rock back onto head (bridge) -> drive hips up/forward -> head lifts, weight
  onto feet -> deep crouch -> rise -> stand

Run: .venv/bin/python sim/getup_v6_bridge.py [--render out.mp4]
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

def build_seq(tuck_h=-125,tuck_k=-130,tuck_ankle=-20, bridge_h=-90,bridge_k=-60,
              bridge_ankle=-20, drive_h=-50, drive_k=-40, rock_t=1.2, drive_t=1.6):
    return [("lie",{},0.4,0.6),
            ("sit up (legs fwd)",dict(hip_pitch=-90,knee=10,ankle=0),1.4,0.6),
            ("tuck feet to buttocks",dict(hip_pitch=tuck_h,knee=tuck_k,ankle=tuck_ankle),1.6,0.8),
            ("rock back onto head (bridge)",dict(hip_pitch=bridge_h,knee=bridge_k,ankle=bridge_ankle),rock_t,1.0),
            ("drive hips up/forward",dict(hip_pitch=drive_h,knee=drive_k,ankle=bridge_ankle),drive_t,1.0),
            ("weight onto feet",dict(hip_pitch=-125,knee=-120,ankle=-20),0.8,0.5),
            ("rise 2",dict(hip_pitch=-80,knee=-70,ankle=-30),0.9,0.5),
            ("rise 3",dict(hip_pitch=-45,knee=-50,ankle=-25),0.9,0.5),
            ("stand",dict(hip_pitch=-20,knee=-40,ankle=-20),1.0,1.2),
            ("straight",{},0.8,0.6)]

def run_one(p,env,seq,name,render=None):
    env.fall_up_z=-2; env.fall_height=-1
    m,d=env.model,env.data; d0,hi,lo=env._default,env._hi,env._lo; na=env._nq_act
    def inv(q):
        q=np.concatenate([q,np.zeros(na-len(q))]) if len(q)<na else q[:na]
        return np.clip(np.where(q>=d0,(q-d0)/np.maximum(hi-d0,1e-6),(q-d0)/np.maximum(d0-lo,1e-6)),-1,1)
    qprev=qq(seq[0][1]); G.settle_fallen(env,p,"supine",qprev)
    dt=env.control_dt; frames=[]; t=0.0; k=0
    if render:
        rnd=mujoco.Renderer(m,540,720); cam=mujoco.MjvCamera(); cam.distance,cam.elevation,cam.azimuth=(1.6,-16,100)
    from getup_v6 import contacts_summary
    peak=-1
    for label,off,move_s,hold_s in seq:
        qt=qq(off); nstep=int(move_s/dt)
        for i in range(nstep+int(hold_s/dt)):
            s=min(1.0,(i+1)/max(nstep,1)); s=10*s**3-15*s**4+6*s**5
            q=qprev+(qt-qprev)*s; env.step(inv(q)); t+=dt; k+=1
            up=d.xmat[env._torso_bid].reshape(3,3)[2,2]; peak=max(peak,up)
            if render and k%2==0:
                cam.lookat[:]=[d.qpos[0],d.qpos[1],0.2]; rnd.update_scene(d,cam); frames.append(rnd.render().copy())
        qprev=qt
        print(f"  {label:26s} up={up:+.2f} pelZ={d.qpos[2]:.3f} | {contacts_summary(m,d)}")
    up=d.xmat[env._torso_bid].reshape(3,3)[2,2]
    ok=up>0.9 and d.qpos[2]>0.8*p.z_yaw_above_sole
    print(f"[{name}] peak_up {peak:+.2f} -> {'STANDING' if ok else 'no'} (up {up:+.2f} z {d.qpos[2]:.3f})")
    if render: _write_video(frames,render,fps=25)
    return ok

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--render",default=None); a=ap.parse_args()
    p=DesignParams(); xml="/tmp/v6_bridge.xml"; open(xml,"w").write(build_xml(p))
    env=make_env(p,xml); obs,_=env.reset(seed=0)
    kp,kd,stall_s,w0_s=env._servo; na=env._nq_act
    def servo():
        stall=np.full(na,float(stall_s)); w0=np.full(na,float(w0_s))
        kpa=np.full(na,float(kp)); kda=np.full(na,float(kd))
        for i,n in enumerate(G.JN):
            if n in G.PJ_DEFAULT:
                s=G.SERVOS["sts3250"]; stall[i]=s["stall"]; w0[i]=s["w0"]; kpa[i]*=s["kp_scale"]; kda[i]*=s["kp_scale"]
        return np.array([kpa,kda,stall,w0],dtype=object)
    env._servo=servo()
    run_one(p,env,build_seq(),"bridge-90/60",render=a.render)

main()
