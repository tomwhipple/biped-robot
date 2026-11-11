import sys, os, json, math, types, re, time
os.environ['JAX_PLATFORMS']='cpu'; os.environ['CUDA_VISIBLE_DEVICES']=''
for p in ('/home/claw/code/robot/experiments/policy/weight-transfer-gate','/home/claw/code/robot/experiments/one-leg-stand'):
    sys.path.insert(0,p)
import numpy as np
import check1_oneleg as C, gate_common as GC
import simfit_sim as SS, simfit_hw as H
J=json.load(open('shift_ladder.json'))
def build(plant,terms):
    pl=SS.Plant()
    env=GC.make_plant(plant,terms=terms)
    if plant=='grade': pl=SS.Plant(zoff_La=-2.12)
    SS.CTRL_DT=env.control_dt
    env._zero_off=None
    return pl,env
def run(plant,terms,side,stand_cmd,crouch_cmd,pose_cmd,label):
    pl,env=build(plant,terms)
    tk=lambda c:[int(x) for x in c.split()[:12]]
    du=lambda c:int(re.search(r's(\d+)',c).group(1))/1e3
    mv=[H.Move('stand',0.0,du(stand_cmd),tk(stand_cmd),None)]
    t=3.5
    if label!='stand':
        mv.append(H.Move('crouchm',t,du(crouch_cmd),tk(crouch_cmd),None)); t+=du(crouch_cmd)+3.0   # not named 'crouch' on purpose: Replay starts at the stand
    if label not in('stand','crouch'):
        mv.append(H.Move('shift',t,du(pose_cmd),tk(pose_cmd),None)); t+=du(pose_cmd)
    elif label=='crouch': t=3.5+du(crouch_cmd)
    else: t=3.5
    t_end=t+3.0
    st=types.SimpleNamespace(cal=dict(C.CAL),side=side,moves=mv,t_utc0=0.0)
    ctl=SS.Ctl()
    orig=SS.make_env; SS.make_env=lambda pl_,seed=0:env
    try:
        rp=SS.Replay(st,pl,ctl,t_end=t_end)
        alpha=getattr(env,'_gate_alpha',None)
        if alpha is not None and np.any(alpha>0):
            enc0=rp.encoder_rad
            def enc():
                q=env.data.qpos[env._sqpos]
                return enc0()+(alpha*(env.data.ctrl-q))[rp.jidx]
            rp.encoder_rad=enc
        r=rp.run()
    finally: SS.make_env=orig
    tt=np.array(r['t']); w=tt>=tt[-1]-0.5
    stn=side; sw='L' if side=='R' else 'R'
    F={stn:float(np.mean(np.array(r['fz_st'])[w])),sw:float(np.mean(np.array(r['fz_sw'])[w]))}
    roll=float(np.mean(np.array(r['roll'])[w]))*180/math.pi
    mass=float(env.model.body_mass.sum())
    return dict(L_N=F['L'],R_N=F['R'],roll_deg=roll,mass=mass,sd_tot=float(np.std((np.array(r['fz_st'])+np.array(r['fz_sw']))[w])))
PLANTS=[('grade','', 'grade'),('train','','train'),('grade','ser_hip=14.2','grade+ser_hip=14.2')]
out={}
only=sys.argv[1:] 
for plant,tspec,nm in PLANTS:
    terms=GC.parse_terms(tspec)
    rows=[]
    for side in 'RL':
        s=J['sides'][side]
        poses=[('stand',s['stand']['cmd'])] if False else []
        seq=[('stand',None),('crouch',None)]+[(f"x{g['mult']}",g['cmd']) for g in s['rungs']]
        for lab,cmd in seq:
            t0=time.time()
            key='stand' if lab=='stand' else ('crouch' if lab=='crouch' else 'shift')
            r=run(plant,terms,side,s['stand']['cmd'],s['crouch']['cmd'],cmd,key)
            r.update(side=side,pose=lab); rows.append(r)
            print(nm,side,lab,round(r['L_N'],1),round(r['R_N'],1),round(r['roll_deg'],2),round(time.time()-t0,1),flush=True)
    out[nm]=rows
    json.dump(out,open('shift_ladder_sim.json','w'),indent=1)
