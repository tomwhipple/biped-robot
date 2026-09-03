import sys, math, mujoco
sys.path.insert(0, '.')
from walker_env import BimoWalkerEnv
env = BimoWalkerEnv(xml_path='bimo_biped_v5body.xml', actuator_model='ideal', render_mode=None); env.reset(seed=0)
m, d = env.model, env.data; names=[m.actuator(i).name for i in range(m.nu)]; tb=m.body('torso').id
def yaw(): R=d.xmat[tb].reshape(3,3); return math.degrees(math.atan2(R[1,0], R[0,0]))
def settle(sec):
    for _ in range(int(sec/m.opt.timestep)): mujoco.mj_step(m,d)
d.ctrl[:]=0; settle(1.5); y0=yaw()
L=names.index('L_hip_yaw'); R=names.index('R_hip_yaw')
STEPS=[0,1,2,3,4,3,2,1,0,-1,-2,-3,-4,-3,-2,-1,0]
for mode,(sL,sR) in {'both same':(1,1),'left only':(1,0),'opposite':(1,-1)}.items():
    d.ctrl[:]=0; settle(1.0); y0=yaw(); out=[]
    for deg in STEPS:
        d.ctrl[:]=0; d.ctrl[L]=math.radians(deg*sL); d.ctrl[R]=math.radians(deg*sR); settle(0.6)
        out.append((deg, yaw()-y0))
    print(f"{mode:10s} pelvis yaw per cmd: "+" ".join(f"{c:+d}:{y:+.2f}" for c,y in out))
