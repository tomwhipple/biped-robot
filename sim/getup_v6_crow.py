"""Lateral crow-stand rise -- the beyond-human DOF family.

From SUPINE the robot uses hip YAW +-45, hip ROLL (abd) 45, ankle ROLL +-25 to
splay the legs FULLY OUT TO THE SIDES and plant the soles flat on the floor as
two lateral outriggers (a 'crow-stand' / 'barn-gable' geometry). The torso then
presses UP between the two planted feet -- a bridge -- and the CoM is inside a
very broad support polygon (the feet are far apart laterally), so the rise
doesn't pivot over the head. Humans cannot splay legs 45 deg out and plant the
foot soles flat; the robot can, which is exactly the 'beyond-human' lever.

Phases:
  lie (supine, legs splayed out to the sides by yaw+roll)
  plant soles flat laterally (hip yaw 45 L/R, hip roll abd 45, ankle roll to
     keep the sole flat)
  bridge up (hips rise between the two planted feet as the knees/hips extend)
  retract feet under (yaw back to 0, roll back) into a deep crouch
  rise / stand

Run: .venv/bin/python sim/getup_v6_crow.py [--render out.mp4] [--bridge H,K...]
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
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def build_seq(yaw=45, roll=45, plant_k=-110, plant_h=-100, bridge_h=-60, bridge_k=-70,
              bridge_ankle=-20, retract_k=-110, retract_h=-110, t_plant=1.5, t_bridge=2.5):
    """yaw/roll: lateral splay target (deg, symmetric); plant_* : the planted
    pose; bridge_* : the legs semi-extended bridge; retract_* : feet back under."""
    spread = dict(L_hip_yaw=yaw, R_hip_yaw=-yaw, L_hip_roll=roll, R_hip_roll=-roll)
    plant = dict(hip_pitch=plant_h, knee=plant_k, ankle=0, **spread)
    bridge = dict(hip_pitch=bridge_h, knee=bridge_k, ankle=bridge_ankle, **spread)
    retract = dict(hip_pitch=retract_h, knee=retract_k, ankle=bridge_ankle,
                   L_hip_yaw=0, R_hip_yaw=0, L_hip_roll=0, R_hip_roll=0)
    return [("lie splayed", spread, 0.6, 0.8),
            ("plant soles lateral", plant, t_plant, 1.0),
            ("bridge up", bridge, t_bridge, 1.2),
            ("retract under", retract, 1.6, 0.8),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30), 0.9, 0.5),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25), 0.9, 0.5),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.0, 1.0),
            ("straight", {}, 0.8, 0.6)]

def run_one(p, xml, env, seq, name, render=None):
    env.fall_up_z = -2; env.fall_height = -1
    m, d = env.model, env.data
    d0, hi, lo = env._default, env._hi, env._lo
    na = env._nq_act
    def inv(q):
        q = np.concatenate([q, np.zeros(na-len(q))]) if len(q)<na else q[:na]
        return np.clip(np.where(q>=d0,(q-d0)/np.maximum(hi-d0,1e-6),(q-d0)/np.maximum(d0-lo,1e-6)),-1,1)
    qprev = qq(seq[0][1]); G.settle_fallen(env, p, "supine", qprev)
    dt = env.control_dt; frames=[]; t=0.0; k=0
    if render:
        rnd = mujoco.Renderer(m,560,720); cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = (1.5, -18, 135)
    from getup_v6 import contacts_summary
    peak_up=-1.0
    for label, off, move_s, hold_s in seq:
        qt=qq(off); nstep=int(move_s/dt)
        for i in range(nstep+int(hold_s/dt)):
            s=min(1.0,(i+1)/max(nstep,1)); s=10*s**3-15*s**4+6*s**5
            q=qprev+(qt-qprev)*s; env.step(inv(q)); t+=dt; k+=1
            up=d.xmat[env._torso_bid].reshape(3,3)[2,2]; peak_up=max(peak_up,up)
            if render and k%2==0:
                cam.lookat[:]=[d.qpos[0],d.qpos[1],0.2]; rnd.update_scene(d,cam); frames.append(rnd.render().copy())
        qprev=qt
        print(f"  {label:16s} up={up:+.2f} pelZ={d.qpos[2]:.3f} | {contacts_summary(m,d)}")
    up=d.xmat[env._torso_bid].reshape(3,3)[2,2]
    ok=up>0.9 and d.qpos[2]>0.8*p.z_yaw_above_sole
    print(f"[{name}] peak_up {peak_up:+.2f} -> {'STANDING' if ok else 'no'} (up {up:+.2f} z {d.qpos[2]:.3f})")
    if render: _write_video(frames, render, fps=25)
    return ok

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--render",default=None); a=ap.parse_args()
    p=DesignParams(); xml="/tmp/v6_crow.xml"; open(xml,"w").write(build_xml(p))
    # fresh env per variant is costly; we reuse a template env and re-settle
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
        (dict(yaw=45,roll=45,plant_k=-110,plant_h=-100,bridge_h=-45,bridge_k=-60), "45splay bridge-45"),
        (dict(yaw=45,roll=45,plant_k=-110,plant_h=-100,bridge_h=-30,bridge_k=-50), "45splay bridge-30"),
        (dict(yaw=45,roll=45,plant_k=-110,plant_h=-100,bridge_h=-60,bridge_k=-80), "45splay bridge-60"),
        (dict(yaw=30,roll=45,plant_k=-110,plant_h=-100,bridge_h=-45,bridge_k=-60), "30splay bridge-45"),
        (dict(yaw=45,roll=30,plant_k=-110,plant_h=-100,bridge_h=-45,bridge_k=-60), "y45r30"),
        (dict(yaw=45,roll=45,plant_k=-120,plant_h=-110,bridge_h=-45,bridge_k=-60), "deep plant"),
    ]
    for i,(kw,name) in enumerate(variants):
        env._servo=servo()
        ok=run_one(p,xml,env,build_seq(**kw),name, render=(a.render if i==1 else None))

main()
