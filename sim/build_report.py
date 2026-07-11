"""Rebuild the training-log report page (sim/runs/night_summary.html).

Run:  .venv/bin/python sim/build_report.py
Needs: gifs under sim/runs/<run>/ and cad/renders/assembly_mujoco.png
(regenerate the latter with .venv/bin/python cad/render_assembly.py).
The page is self-contained (base64 images) and is what gets published as the
claude.ai artifact after each training round."""
import os, base64, io
import numpy as np
import imageio.v2 as imageio

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__))))
CAD_RENDER = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'cad', 'renders', 'assembly_mujoco.png')

def strip_from_gif(path, n=8, factor=4):
    frames = imageio.mimread(path, memtest=False)
    frames = [f[..., :3] for f in frames]
    idx = np.linspace(0, len(frames) - 1, n).astype(int)
    strip = np.concatenate([frames[i] for i in idx], axis=1)
    return strip[::factor, ::factor]

def to_data_uri(arr):
    buf = io.BytesIO()
    imageio.imwrite(buf, arr, format='png')
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()

hero    = to_data_uri(strip_from_gif('runs/terrain_v4/walk_terrain15mm.gif'))
stride  = to_data_uri(strip_from_gif('runs/terrain_v4/walk.gif'))
shuffle = to_data_uri(strip_from_gif('runs/shaped_v6/walk.gif'))
lunge   = to_data_uri(strip_from_gif('runs/ppo_baseline/walk.gif', n=7))

cad = imageio.imread(CAD_RENDER)[..., :3]
cad = cad[10:700, 200:700]            # tight crop around the robot
cad_uri = to_data_uri(cad)

n_runs = len([d for d in os.listdir('runs')
              if os.path.exists(os.path.join('runs', d, 'model.zip'))])
for k, v in [('hero', hero), ('stride', stride), ('shuffle', shuffle),
             ('lunge', lunge), ('cad', cad_uri)]:
    print(k, len(v) // 1024, 'KB')
print('runs:', n_runs)

HTML = f"""<title>Biped RL — training log</title>
<style>
:root {{
  --bg:#eef1f5; --panel:#ffffff; --panel2:#e7ebf1; --fg:#16202b; --muted:#5c6b7a;
  --border:#d3dae3; --accent:#0e7c9b; --accent-soft:#0e7c9b1a;
  --good:#1f9d57; --warn:#b9820f; --bad:#cc4b4b;
  --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,Consolas,monospace;
  --sans:-apple-system,system-ui,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
}}
@media (prefers-color-scheme:dark) {{
  :root {{
    --bg:#0e141b; --panel:#161d26; --panel2:#1c2530; --fg:#d9e2ec; --muted:#8598a8;
    --border:#263140; --accent:#3fc9e6; --accent-soft:#3fc9e622;
    --good:#46c17f; --warn:#e0a53b; --bad:#e46a6a;
  }}
}}
:root[data-theme="light"] {{
  --bg:#eef1f5; --panel:#ffffff; --panel2:#e7ebf1; --fg:#16202b; --muted:#5c6b7a;
  --border:#d3dae3; --accent:#0e7c9b; --accent-soft:#0e7c9b1a;
  --good:#1f9d57; --warn:#b9820f; --bad:#cc4b4b;
}}
:root[data-theme="dark"] {{
  --bg:#0e141b; --panel:#161d26; --panel2:#1c2530; --fg:#d9e2ec; --muted:#8598a8;
  --border:#263140; --accent:#3fc9e6; --accent-soft:#3fc9e622;
  --good:#46c17f; --warn:#e0a53b; --bad:#e46a6a;
}}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--fg); font-family:var(--sans);
  line-height:1.6; -webkit-font-smoothing:antialiased; }}
.wrap {{ max-width:920px; margin:0 auto; padding:clamp(24px,5vw,64px) clamp(18px,4vw,40px); }}
.eyebrow {{ font-family:var(--mono); font-size:12px; letter-spacing:.18em; text-transform:uppercase;
  color:var(--accent); margin:0 0 14px; }}
h1 {{ font-size:clamp(30px,5.5vw,50px); line-height:1.05; letter-spacing:-.02em; font-weight:700;
  margin:0 0 16px; text-wrap:balance; }}
h2 {{ font-size:clamp(19px,3vw,24px); letter-spacing:-.01em; margin:52px 0 18px; font-weight:650;
  display:flex; align-items:baseline; gap:12px; }}
h2 .n {{ font-family:var(--mono); font-size:13px; color:var(--accent); font-weight:600; }}
p {{ margin:0 0 16px; max-width:66ch; }}
.lede {{ font-size:19px; color:var(--fg); }}
.muted {{ color:var(--muted); }}
a {{ color:var(--accent); }}
.meta {{ font-family:var(--mono); font-size:12.5px; color:var(--muted); display:flex; gap:18px;
  flex-wrap:wrap; margin-top:20px; padding-top:18px; border-top:1px solid var(--border); }}
figure {{ margin:0; }}
.film {{ width:100%; display:block; border:1px solid var(--border); border-radius:10px;
  background:var(--panel2); }}
figcaption {{ font-family:var(--mono); font-size:12px; color:var(--muted); margin-top:9px;
  letter-spacing:.02em; }}
.kpis {{ display:grid; grid-template-columns:repeat(4,1fr); gap:12px; margin:26px 0 4px; }}
@media (max-width:640px) {{ .kpis {{ grid-template-columns:repeat(2,1fr); }} }}
.kpi {{ background:var(--panel); border:1px solid var(--border); border-radius:12px; padding:16px 16px 14px; }}
.kpi .v {{ font-family:var(--mono); font-size:26px; font-weight:600; letter-spacing:-.01em;
  font-variant-numeric:tabular-nums; }}
.kpi .k {{ font-family:var(--mono); font-size:11px; letter-spacing:.1em; text-transform:uppercase;
  color:var(--muted); margin-top:6px; }}
.card {{ background:var(--panel); border:1px solid var(--border); border-radius:14px;
  padding:22px 24px; margin:24px 0; }}
.card.accent {{ border-left:3px solid var(--accent); }}
.duo {{ display:grid; grid-template-columns:1fr; gap:18px; margin:20px 0; }}
ol.timeline {{ list-style:none; counter-reset:step; margin:0; padding:0; }}
ol.timeline > li {{ position:relative; counter-increment:step; padding:0 0 26px 46px; }}
ol.timeline > li::before {{ content:counter(step,decimal-leading-zero); position:absolute; left:0; top:-2px;
  font-family:var(--mono); font-size:12px; font-weight:600; color:var(--accent);
  width:30px; height:30px; border:1px solid var(--border); border-radius:50%;
  display:flex; align-items:center; justify-content:center; background:var(--panel); }}
ol.timeline > li::after {{ content:""; position:absolute; left:15px; top:30px; bottom:0; width:1px;
  background:var(--border); }}
ol.timeline > li:last-child {{ padding-bottom:0; }}
ol.timeline > li:last-child::after {{ display:none; }}
.timeline h3 {{ margin:2px 0 5px; font-size:16.5px; font-weight:640; }}
.timeline p {{ margin:0 0 12px; font-size:15px; color:var(--muted); }}
.tag {{ font-family:var(--mono); font-size:11px; padding:2px 7px; border-radius:5px;
  background:var(--accent-soft); color:var(--accent); letter-spacing:.03em; white-space:nowrap; }}
.tblwrap {{ overflow-x:auto; border:1px solid var(--border); border-radius:12px; margin:8px 0 6px; }}
table {{ border-collapse:collapse; width:100%; font-size:14.5px; min-width:560px; }}
th,td {{ text-align:left; padding:12px 16px; border-bottom:1px solid var(--border); }}
thead th {{ font-family:var(--mono); font-size:11px; letter-spacing:.09em; text-transform:uppercase;
  color:var(--muted); font-weight:600; }}
tbody tr:last-child td {{ border-bottom:none; }}
td.name {{ font-family:var(--mono); font-size:13px; }}
td.name b {{ color:var(--accent); }}
.pill {{ font-family:var(--mono); font-size:12px; font-variant-numeric:tabular-nums; font-weight:600;
  padding:3px 9px; border-radius:20px; display:inline-block; }}
.pill.g {{ background:color-mix(in srgb,var(--good) 16%,transparent); color:var(--good); }}
.pill.a {{ background:color-mix(in srgb,var(--warn) 18%,transparent); color:var(--warn); }}
.pill.r {{ background:color-mix(in srgb,var(--bad) 16%,transparent); color:var(--bad); }}
.cadgrid {{ display:grid; grid-template-columns:minmax(0,320px) 1fr; gap:24px; align-items:center; }}
@media (max-width:680px) {{ .cadgrid {{ grid-template-columns:1fr; }} }}
.cadgrid img {{ width:100%; display:block; border:1px solid var(--border); border-radius:10px; }}
ul.plain {{ margin:0; padding-left:20px; }}
ul.plain li {{ margin-bottom:9px; }}
code {{ font-family:var(--mono); font-size:.88em; background:var(--panel2); padding:1px 6px;
  border-radius:5px; }}
.rule {{ height:1px; background:var(--border); border:0; margin:48px 0; }}
</style>

<div class="wrap">
  <p class="eyebrow">MuJoCo · PPO · training log · day 2</p>
  <h1>First it walked. Now it strides.</h1>
  <p class="lede">Night one produced a robust but shuffling walk. Day two rewrote
  the objective around gait quality — feet that actually lift — and pulled the
  ground out from under it with procedurally rough terrain. The robot came back
  50% faster, stepping cleanly over 15&nbsp;mm bumps. It also has a printable
  body now.</p>
  <div class="meta">
    <span>2026·07·09 → 07·11</span><span>8× parallel envs · CPU</span>
    <span>{n_runs} trained policies</span>
  </div>

  <div class="kpis">
    <div class="kpi"><div class="v" style="color:var(--accent)">terrain_v4</div><div class="k">recommended policy</div></div>
    <div class="kpi"><div class="v">0.79<span style="font-size:15px"> m/s</span></div><div class="k">was 0.53 — speed cap unchanged</div></div>
    <div class="kpi"><div class="v">20/24</div><div class="k">survives 15 mm rough ground</div></div>
    <div class="kpi"><div class="v">0.28<span style="font-size:15px"> s</span></div><div class="k">foot swing · was 0.11 s shuffle</div></div>
  </div>

  <figure style="margin-top:34px">
    <img class="film" src="{hero}" alt="Filmstrip of the biped striding across procedurally generated rough terrain">
    <figcaption>terrain_v4 · one 10-second episode on ±15 mm procedurally generated ground — regenerated every episode, never seen twice</figcaption>
  </figure>

  <h2><span class="n">01</span> The shuffle became a stride</h2>
  <p>The night-one gait survived everything but looked wrong: 0.11 s foot swings,
  feet skating in ground contact 14% of the time in double support. The fix was
  not more speed reward — the forward-velocity cap never moved from 0.5 m/s.
  Two new shaping terms did it: a <b>feet-air-time reward</b> (touchdowns pay in
  proportion to how long the foot was airborne, so shuffle-taps score negative)
  and a <b>single-support bonus</b>. Walking properly turned out to be the fast
  way to walk: speed rose to 0.79 m/s on its own.</p>
  <div class="duo">
    <figure>
      <img class="film" src="{shuffle}" alt="Filmstrip of the old shuffling gait">
      <figcaption>before — shaped_v6 · feet skate, torso bobs · 0.53 m/s</figcaption>
    </figure>
    <figure>
      <img class="film" src="{stride}" alt="Filmstrip of the new clean striding gait">
      <figcaption>after — terrain_v4 on flat ground · clear foot lift, alternating stride · 0.79 m/s</figcaption>
    </figure>
  </div>
  <p class="muted">One negative result worth keeping: fine-tuning from the old
  policy could not escape the shuffle — its optimum was too deep. The stride only
  appeared when training restarted from scratch with the gait terms in place.</p>

  <h2><span class="n">02</span> Rough ground</h2>
  <p>The floor is now a MuJoCo heightfield, resampled from smoothed noise at
  every reset, with fall detection measured against the local ground surface
  instead of world height. Training walked a curriculum from 5 mm to mixed
  0–12 mm bumps. For a 34 cm robot, 15 mm is roughly a human stepping on
  unseen 8 cm rubble — blind, since the policy has no terrain sensing yet.</p>
  <div class="tblwrap">
    <table>
      <thead><tr><th>Policy</th><th>Flat</th><th>10 mm terrain</th><th>15 mm terrain</th><th>Speed</th></tr></thead>
      <tbody>
        <tr><td class="name">shaped_v6 <span class="muted">night 1</span></td>
          <td><span class="pill g">23/24</span></td><td><span class="pill a">15/24</span></td><td class="muted">—</td><td>0.53 m/s</td></tr>
        <tr><td class="name">gait_v6 <span class="muted">stride, flat-only</span></td>
          <td><span class="pill g">24/24</span></td><td><span class="pill r">0/24</span></td><td class="muted">—</td><td>0.83 m/s</td></tr>
        <tr><td class="name"><b>terrain_v4</b> <span class="muted">stride + terrain</span></td>
          <td><span class="pill g">22/24</span></td><td><span class="pill g">20/24</span></td><td><span class="pill g">20/24</span></td><td>0.79 m/s</td></tr>
      </tbody>
    </table>
  </div>
  <p class="muted" style="margin-top:10px">terrain_v4 also keeps the night-one
  model-error insurance: 20/24 on 10 mm terrain under ±15% mass, ±40% friction,
  ±20% actuator-gain randomization.</p>

  <h2><span class="n">03</span> It has a body now</h2>
  <div class="card">
    <div class="cadgrid">
      <img src="{cad_uri}" alt="Rendered CAD assembly of the printable biped standing on a checkered floor">
      <div>
        <p style="margin-bottom:10px">A complete printable part set, generated as
        parametric code-CAD from measured STS3215 servo geometry: <b>6 unique
        parts, 12 prints</b>, all support-free and print-bed-checked,
        ≈ 0.88 kg assembled.</p>
        <ul class="plain" style="font-size:15px">
          <li><b>No bearings needed</b> — the servo's rear idler disc shares the
          horn's bolt pattern, so every joint is supported on both sides.</li>
          <li>30 joint-sweep interference checks pass across the full range of
          motion (four real collisions found and fixed along the way).</li>
          <li>STLs for the slicer, STEP files (incl. a full <code>assembly.step</code>)
          for FreeCAD, BOM &amp; print settings in <code>cad/README.md</code>.</li>
        </ul>
      </div>
    </div>
  </div>

  <h2><span class="n">04</span> Day 1, condensed — how it learned to walk</h2>
  <ol class="timeline">
    <li>
      <h3>A Gymnasium env around the MuJoCo model</h3>
      <p>36-dim observation, 8 target-angle actions as a residual around the
      standing pose (so a zero policy stands, not crouches). Reward: forward
      velocity + upright + alive − energy − jitter.</p>
      <span class="tag">walker_env.py</span>
    </li>
    <li>
      <h3>Baseline PPO found the cheat, not the gait</h3>
      <p>With an uncapped forward reward, the fastest way to earn it is to dive.
      The policy learned to faceplant forward — 1.3 m in one lunge, then falls.</p>
      <figure style="margin:4px 0 12px">
        <img class="film" src="{lunge}" alt="Filmstrip of the robot lunging forward and falling">
        <figcaption>ppo_baseline · a forward dive, not a walk</figcaption>
      </figure>
      <span class="tag">the exploit</span>
    </li>
    <li>
      <h3>Cap the forward reward → a real walk emerges</h3>
      <p>Rewarding speed only up to a target makes a steady gait out-earn a
      dive. An alternating-leg walk appeared and survived full episodes.</p>
      <span class="tag">shaped_v2</span>
    </li>
    <li>
      <h3>Domain randomization → robust to a wrong robot</h3>
      <p>Mass ±15%, friction ±40%, actuator gain ±20%, randomized every episode:
      a perfect nominal gait that also survives under model error — the
      reality-gap insurance sim-to-real needs.</p>
      <span class="tag">shaped_v4 → v6</span>
    </li>
  </ol>

  <h2><span class="n">05</span> Open items</h2>
  <div class="card accent">
    <ul class="plain" style="margin:0">
      <li><b>The stride is technically a jog</b> — double support ≈ 0%, so there's
      a brief flight phase. Real STS3215s may not track it; a grounded re-tune
      (shorter air-time target or a flight penalty) is cheap if hardware says so.</li>
      <li><b>The policy is terrain-blind.</b> Proprioception only — beyond ~15 mm
      it will need a height-scan observation, which pairs naturally with the
      planned goal-seeking stage (waypoint in the observation, progress reward).</li>
      <li><b>Repeated shoves remain unsolved</b> from night one: needs a realistic
      push model plus a balance-recovery reward, still a design decision.</li>
    </ul>
  </div>

  <hr class="rule">
  <p class="muted" style="font-size:14px">Reproduce: <code>cd sim &amp;&amp; python eval_policy.py --run-name terrain_v4 --render --terrain-amplitude 0.015</code>
  · rank runs with <code>python compare_runs.py</code> · full write-up in <code>DESIGN.md</code> ·
  CAD in <code>cad/</code> (open <code>step/assembly.step</code> in FreeCAD).</p>
</div>
"""

out = 'runs/night_summary.html'
with open(out, 'w') as f:
    f.write(HTML)
print("wrote", out, "|", len(HTML) // 1024, "KB")
