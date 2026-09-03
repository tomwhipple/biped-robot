"""Same single-joint sweep as play_sweep.py, on the rigid sim plant (ideal position servos)."""
import sys, math, numpy as np, mujoco
sys.path.insert(0, '.')
from walker_env import BimoWalkerEnv
env = BimoWalkerEnv(xml_path='bimo_biped_v5body.xml', actuator_model='ideal', render_mode=None)
env.reset(seed=0)
m, d = env.model, env.data
names = [m.actuator(i).name for i in range(m.nu)]
tb = m.body('torso').id
def tilt():
    R = d.xmat[tb].reshape(3,3); return math.degrees(math.acos(max(-1,min(1,R[2,2]))))
def settle(sec):
    for _ in range(int(sec/m.opt.timestep)): mujoco.mj_step(m, d)
d.ctrl[:] = 0.0; settle(1.5)
print('actuators:', names); print(f'settled tilt {tilt():.2f} deg, torso z {d.xpos[tb][2]:.3f}')
base = d.qpos.copy()
STEPS = [x*0.5 for x in range(0,9)] + [x*0.5 for x in range(7,-9,-1)] + [x*0.5 for x in range(-7,1)]
for jn in ['L_hip_roll','R_hip_roll','L_hip_pitch','R_hip_pitch','L_ankle','R_ankle','L_knee','R_knee']:
    j = names.index(jn); out=[]
    for deg in STEPS:
        d.ctrl[:] = 0.0; d.ctrl[j] = math.radians(deg); settle(0.5)
        qj = d.qpos[m.jnt_qposadr[m.actuator_trnid[j,0]]]
        out.append((deg, tilt(), math.degrees(qj), d.xpos[tb][2]))
    d.ctrl[:] = 0.0; settle(0.8)
    mx = max(o[1] for o in out); qerr = max(abs(o[0]-o[2]) for o in out)
    print(f'{jn:12s} tilt max {mx:5.2f} deg  max |cmd-q| {qerr:4.2f} deg  fell={d.xpos[tb][2] < 0.2}')
    print('   '+' '.join(f'{o[0]:+.1f}:{o[1]:.1f}' for o in out))
