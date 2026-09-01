"""Stage-1 sim: drop the biped on a floor and run a coordinated squat cycle.
Exercises every pitch joint under gravity and checks it stays balanced.
Run: MUJOCO_GL=osmesa python3 sim_biped.py   (osmesa = headless software GL)"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco, imageio.v2 as imageio

m = mujoco.MjModel.from_xml_path(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "bimo_biped.xml"))
d = mujoco.MjData(m)
dt = m.opt.timestep

def hold(targets):  # radians, keyed by actuator name
    for name, val in targets.items():
        d.ctrl[m.actuator(name).id] = val

def squat_targets(t, amp=0.6):
    # one smooth down-up bump between 0.3s and 1.7s; torso stays ~vertical
    a = amp * (1 - np.cos(2*np.pi*(t-0.3)/1.4))/2 if 0.3 <= t <= 1.7 else 0.0
    return {"L_hip_roll":0.0, "R_hip_roll":0.0,
            "L_hip_pitch":0.5*a, "R_hip_pitch":0.5*a,
            "L_knee":-a, "R_knee":-a,
            "L_ankle":0.5*a, "R_ankle":0.5*a}

renderer = None
try:
    renderer = mujoco.Renderer(m, height=1100, width=900)
except Exception as e:
    print("renderer unavailable:", e)

cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0.02, 0.0, 0.14]; cam.distance = 0.60
cam.azimuth = 135; cam.elevation = -12

T, frames, heights, ups = 2.2, [], [], []
for i in range(int(T/dt)):
    t = i*dt
    hold(squat_targets(t))
    mujoco.mj_step(m, d)
    heights.append(float(d.qpos[2]))
    ups.append(float(d.sensordata[2]))
    if renderer is not None and i % 15 == 0:
        renderer.update_scene(d, cam); frames.append(renderer.render())

heights, ups = np.array(heights), np.array(ups)
i_low = int(heights.argmin())
print(f"stand height : {heights[0]*1000:5.0f} mm")
print(f"deepest squat: {heights[i_low]*1000:5.0f} mm  (dropped {(heights[0]-heights[i_low])*1000:.0f} mm)")
print(f"recovered to : {heights[-1]*1000:5.0f} mm")
print(f"uprightness  : min {ups.min():.3f}  final {ups[-1]:.3f}  (1.0 = torso vertical)")
stable = ups.min() > 0.9 and heights[-1] > 0.24
print(f"VERDICT      : {'BALANCED through squat' if stable else 'LOST BALANCE'}")

if frames:
    imageio.imsave("mj_deep.png",  frames[int(i_low/15)])
    imageio.imsave("mj_stand.png", frames[-1])
    imageio.mimsave("mj_squat.gif", frames, fps=20)
    print(f"rendered {len(frames)} frames -> mj_deep.png, mj_stand.png, mj_squat.gif")
else:
    print("no frames (headless GL failed) - numeric diagnostics only")
