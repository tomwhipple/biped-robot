"""Interactive joint-puppet for the v6/v7 body. Every joint is a slider; the
robot lives under gravity and settles on the floor around whatever pose the
sliders command, so you can FEEL which poses are reachable, which collapse,
and where the get-up path pinches -- the thing the keyframe searches can't
show. Gravity always applies; the sliders drive the plant's own position
actuators (same servos, same limits -- no stronger puppet string than the
real joints).

Run (on macOS use mjpython, the MuJoCo GL wrapper -- plain `python` cannot
open the 3D window there):

    .venv/bin/mjpython sim/joint_puppet.py                      # v7 body, standing
    .venv/bin/mjpython sim/joint_puppet.py --skid               # with the pelvis skid
    .venv/bin/mjpython sim/joint_puppet.py --pose supine        # start fallen
    .venv/bin/python  sim/joint_puppet.py --no-window           # headless / any platform

Controls: drag a joint slider to a target (deg) and the servo tracks it --
gravity, contacts and inertia act the whole time. Presets set the free body
+ zero the rest. "record pose" appends the CURRENT settled pose to a keyframe
list (shown in the console) that you can paste into getup_v6*/a sequence --
turns this into a get-up path designer: pose the frames, then replay them.

Camera: left-drag rotate, right-drag pan, scroll zoom (the MuJoCo viewer).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco  # noqa: E402
try:
    import mujoco.viewer as mujoco_viewer
    HAVE_VIEWER = True
except Exception:  # noqa: BLE001
    HAVE_VIEWER = False

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import v6_kin as VK  # noqa: E402

JOINTS = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
          "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]

# body poses for the freejoint: (quat wxyz, z). supine/prone = fallen.
POSES = {
    "stand":  ((1.0, 0.0, 0.0, 0.0), None),                       # z from the plant
    "supine": ((math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0), 0.13),
    "prone":  ((math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0), 0.13),
    "side_L": ((math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0), 0.13),
    "side_R": ((math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0), 0.13),
}


class Puppet:
    def __init__(self, p: DesignParams, pose: str):
        self.p = p
        self.m = mujoco.MjModel.from_xml_string(build_xml(p))
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()          # guards qpos/ctrl between the
        self.targets_deg = {}                 # GUI slider thread and the sim
        self.recorded = []
        self._quit = False
        for i in range(self.m.nu):
            a = self.m.actuator(i)
            lo, hi = self.m.jnt_range[a.trnid[0]]
            if math.degrees(hi - lo) > 0.1:   # an actual joint, not a weld
                self.targets_deg[a.name] = 0.0
        self._act_index = {self.m.actuator(i).name: i for i in range(self.m.nu)}
        self._set_body(pose)
        self._sync_targets()

    def _set_body(self, pose: str):
        quat, z = POSES[pose]
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self.d.qpos[0:3] = [0.0, 0.0, z if z is not None else self.p.z_yaw_above_sole]
            self.d.qpos[3:7] = quat
            self.d.qvel[:] = 0.0
            mujoco.mj_forward(self.m, self.d)

    def _sync_targets(self):
        """set the sliders from the current joint state (after a pose jump)."""
        with self.lock:
            for name, i in self._act_index.items():
                adr = self.m.joint(self.m.actuator(i).trnid[0]).qposadr[0]
                self.targets_deg[name] = math.degrees(self.d.qpos[adr])

    def record(self):
        with self.lock:
            q = {n: round(math.degrees(self.d.qpos[self.m.joint(
                self.m.actuator(self._act_index[n]).trnid[0]).qposadr[0]]), 1)
                for n in JOINTS}
        self.recorded.append(q)
        line = ", ".join(f"{n.split('_', 1)[0]}_{n.split('_', 1)[1]} {v:+.0f}" for n, v in q.items() if abs(v) > 0.5)
        up = self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2]
        print(f"[frame {len(self.recorded)}] pelvis z {self.d.qpos[2]:.3f} up {up:+.2f} | {line or 'standing'}")

    def dump(self):
        print("recorded keyframes (paste-ready offsets):")
        for i, f in enumerate(self.recorded):
            off = {k: v for k, v in f.items()}
            print(f"  ({i+1}, {off})")

    def set_joints(self, pose: dict):
        for n, v in pose.items():
            if n in self.targets_deg:
                self.targets_deg[n] = v

    def state(self):
        with self.lock:
            com = self.d.subtree_com[0].copy()
            up = float(self.d.xmat[self.m.body("torso").id].reshape(3, 3)[2, 2])
            contacts = sorted({self.m.body(self.m.geom_bodyid[g]).name
                               for i in range(self.d.ncon) for g in
                               (self.d.contact[i].geom1, self.d.contact[i].geom2)
                               if self.m.body(self.m.geom_bodyid[g]).name != "world"
                               and (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") != "floor"})
            return self.d.qpos[2], up, com, contacts, self.d.time

    def step_once(self):
        """one physics tick with the current slider targets on the servos. The
        panel thread sets targets_deg concurrently; targets are plain floats
        (atomic under the GIL) so no lock is needed on the hot loop."""
        for name, i in self._act_index.items():
            self.d.ctrl[i] = math.radians(self.targets_deg.get(name, 0.0))
        mujoco.mj_step(self.m, self.d)

    def run_viewer(self):
        """main thread: launch_passive (and GLFW generally) MUST run on the
        main thread on macOS, and under `mjpython`. This blocks until the
        window closes."""
        assert HAVE_VIEWER, "mujoco.viewer unavailable"
        with mujoco_viewer.launch_passive(self.m, self.d) as v:
            while v.is_running() and not self._quit:
                self.step_once()
                v.sync()
                time.sleep(self.m.opt.timestep)
        self._quit = True


def _panel_repl(pup: Puppet):
    """terminal fallback (used with --no-window or when the GUI child cannot
    be spawned): `L_knee -90`, `R_ankle 20`, `pose supine`, `record`, `dump`,
    `state`, or `quit`. Gravity runs the whole time."""
    print("commands: <joint> <deg> | pose %s | record | dump | state | quit"
          % "/".join(POSES), flush=True)
    while not pup._quit:
        try:
            print("joint> ", end="", flush=True)
            line = sys.stdin.readline().strip()
            if line == "":
                break                                           # EOF
        except EOFError:
            break
        if not line:
            continue
        w = line.split()
        if w[0] in ("quit", "exit", "q"):
            break
        if w[0] == "pose" and len(w) == 2 and w[1] in POSES:
            pup._set_body(w[1]); pup._sync_targets(); continue
        if w[0] == "record":
            pup.record(); continue
        if w[0] == "dump":
            pup.dump(); continue
        if w[0] == "*":
            try:
                for n in pup.targets_deg:
                    pup.targets_deg[n] = float(w[1])
            except (ValueError, IndexError):
                print("  '* -90' sets every joint")
            continue
        if w[0] == "state":
            z, up, com, contacts, t = pup.state()
            print(f"  t {t:.1f}s pelvis z {z:.3f} up {up:+.2f} CoM x {com[0]:+.3f} | {' '.join(contacts)}")
            continue
        if len(w) == 2 and w[0] in pup.targets_deg:
            try:
                pup.targets_deg[w[0]] = float(w[1])
            except ValueError:
                print("  want <joint> <deg>")
            continue
        print("  unknown joint", w[0], "-- one of:", ", ".join(pup.targets_deg))
    pup._quit = True



# --------------------------------------------------------------------------- GUI
# The GUI is a BROWSER. No native toolkit exists in this venv (no Tk/Qt/wx)
# and on macOS GLFW + a Tk window can't share one process's main thread
# anyway -- so the sim runs the MuJoCo viewer, and a Flask thread serves a
# slider page that any browser opens. Sliders POST joint targets to /set; a
# poll on /state paints the ACHIEVED angles + pelvis/contacts live. Gravity
# runs the whole time, in the browser exactly as on the real bench.

_JOINT_NAMES_HTML = []


def _jrange_deg(pup: Puppet, name: str):
    jid = pup.m.actuator(pup._act_index[name]).trnid[0]
    lo, hi = pup.m.jnt_range[jid]
    return math.degrees(float(lo)), math.degrees(float(hi))


_PAGE = """<!doctype html><meta charset=utf-8>
<title>v6/v7 joint puppet</title>
<style>
 body{font:13px -apple-system,Helvetica,sans-serif;margin:14px;background:#14151a;color:#e6e7ea}
 h1{font-size:14px;margin:0 0 4px} .sub{color:#8a8f98;margin-bottom:10px}
 .row{display:flex;align-items:center;gap:8px;margin:2px 0}
 .nm{width:120px;text-align:right;color:#9aa0ab}
 input[type=range]{flex:1;accent-color:#4f8ef7}
 .tg{width:56px;text-align:right;font-variant-numeric:tabular-nums}
 .ac{width:56px;color:#7fd08a;font-variant-numeric:tabular-nums}
 button{margin:3px 6px 3px 0;padding:5px 10px;background:#22262e;color:#e6e7ea;
  border:1px solid #373c46;border-radius:6px;cursor:pointer}
 button:active{background:#2f6bd6}
 #stat{white-space:pre;font:11px/1.5 ui-monospace,monospace;color:#9aa0ab;margin-top:8px}
 #frames{color:#7fd08a}
</style>
<h1>v6/v7 joint puppet</h1>
<div class=sub>joint targets (deg) — gravity is on; the servo settles the pose. green = achieved angle.</div>
<div id=poses></div>
<div id=sliders></div>
<div>
 <button onclick=send({cmd:'record'})>record pose</button>
 <button onclick=send({cmd:'dump'})>dump frames</button>
</div>
<div id=stat></div>
<script>
let J={};
function row(j){
 return `<div class=row><span class=nm>${j.name}</span>
  <input type=range min=${j.lo} max=${j.hi} step=0.5 value=${j.val}
   oninput=set('${j.name}',this.value) id=s_${j.name}>
  <span class=tg id=t_${j.name}>${j.val.toFixed(0)}&deg;</span>
  <span class=ac id=a_${j.name}></span></div>`;
}
function set(n,v){document.getElementById('t_'+n).innerHTML=(+v).toFixed(0)+'&deg;';
 send({cmd:'set',joint:n,value:+v});}
function send(o){fetch('/set',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify(o)});}
async function init(){
 const s=await (await fetch('/state')).json();
 document.getElementById('sliders').innerHTML=s.joints.map(row).join('');
 document.getElementById('poses').innerHTML=s.poses.map(p=>
  `<button onclick="send({cmd:'pose',pose:'${p}'})">${p}</button>`).join('');
 s.joints.forEach(j=>J[j.name]=1);
 poll();
}
async function poll(){
 try{const s=await (await fetch('/state')).json();
  document.getElementById('stat').textContent=
   `t ${s.t.toFixed(1)}s  pelvis z ${s.z.toFixed(3)}  up ${s.up>=0?'+':''}${s.up.toFixed(2)}  CoM x ${s.com_x.toFixed(3)}  `+
   `frames ${s.recorded}\\ncontacts: ${s.contacts.join(' ')}`;
  for(const n in J){const e=document.getElementById('a_'+n); if(e) e.textContent=s.q[n].toFixed(0)+'°';}}
 catch(e){}
 setTimeout(poll,120);
}
init();
</script>"""


def make_app(pup: Puppet):
    from flask import Flask, jsonify, request
    app = Flask(__name__)
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    @app.route("/")
    def index():
        return _PAGE

    @app.route("/state")
    def state():
        z, up, com, contacts, t = pup.state()
        return jsonify({
            "t": t, "z": z, "up": up, "com_x": float(com[0]),
            "contacts": contacts, "recorded": len(pup.recorded),
            "q": {n: round(math.degrees(pup.d.qpos[pup.m.joint(
                pup.m.actuator(pup._act_index[n]).trnid[0]).qposadr[0]]), 1)
                for n in pup.targets_deg},
            "joints": [{"name": n, "lo": _jrange_deg(pup, n)[0],
                        "hi": _jrange_deg(pup, n)[1],
                        "val": pup.targets_deg.get(n, 0.0)} for n in pup.targets_deg],
            "poses": list(POSES)})

    @app.route("/set", methods=["POST"])
    def set_():
        c = request.get_json(force=True, silent=True) or {}
        cmd = c.get("cmd")
        if cmd == "set" and c.get("joint") in pup.targets_deg:
            pup.targets_deg[c["joint"]] = float(c["value"])
        elif cmd == "pose" and c.get("pose") in POSES:
            pup._set_body(c["pose"])
            pup._sync_targets()
        elif cmd == "record":
            pup.record()
        elif cmd == "dump":
            pup.dump()
        return ("", 204)
    return app


def _serve_web(pup: Puppet, port: int):
    app = make_app(pup)
    # werkzeug's reloader/debugger both spawn threads; keep it bare.
    app.run(host="127.0.0.1", port=port, threaded=True, debug=False,
            use_reloader=False)


def _open_browser(port: int):
    import webbrowser
    webbrowser.open(f"http://127.0.0.1:{port}/")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="stand", choices=list(POSES))
    ap.add_argument("--skid", action="store_true")
    ap.add_argument("--knee", choices=("fwd", "bwd"), default="fwd")
    ap.add_argument("--port", type=int, default=8471, help="slider page port")
    ap.add_argument("--no-window", action="store_true",
                    help="no 3D window: the web GUI alone on a background sim.")
    a = ap.parse_args()
    p = DesignParams(skid=a.skid, knee=a.knee)
    pup = Puppet(p, a.pose)

    threading.Thread(target=_serve_web, args=(pup, a.port), daemon=True).start()
    threading.Timer(0.6, _open_browser, args=(a.port,)).start()
    print(f"slider GUI: http://127.0.0.1:{a.port}/  (opened in your browser)", flush=True)

    if a.no_window or not HAVE_VIEWER:
        def _loop():
            while not pup._quit:
                pup.step_once()
                time.sleep(pup.m.opt.timestep)
        threading.Thread(target=_loop, daemon=True).start()
        _panel_repl(pup)                          # terminal control too
        return
    try:
        pup.run_viewer()                           # 3D window, main thread
    finally:
        pup._quit = True
        pup.dump()


if __name__ == "__main__":
    main()
