"""Rebuild the training-log report page (sim/runs/night_summary.html).

Run:  .venv/bin/python sim/build_report.py
Needs: gifs under sim/runs/<run>/ and cad/renders/ (regenerate the CAD images
with cad/render_assembly.py and cad/animate_assembly.py).
The page is self-contained (base64 images) and is what gets published as the
claude.ai artifact after each training round.
"""
import os, base64, io
import numpy as np
import imageio.v2 as imageio

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__))))
CAD_RENDERS = os.path.join(os.path.dirname(os.getcwd()), 'cad', 'renders')

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

hero    = to_data_uri(strip_from_gif('runs/dash_11v1_hard/dash.gif'))
dash7   = to_data_uri(strip_from_gif('runs/dash_7v4_hard/dash.gif'))
stride  = to_data_uri(strip_from_gif('runs/terrain_v4/walk.gif'))
shuffle = to_data_uri(strip_from_gif('runs/shaped_v6/walk.gif'))
lunge   = to_data_uri(strip_from_gif('runs/ppo_baseline/walk.gif', n=7))
flyin   = to_data_uri(strip_from_gif(
    os.path.join(CAD_RENDERS, 'assembly_flyin.gif'), n=7, factor=3))

cad = imageio.imread(os.path.join(CAD_RENDERS, 'assembly_mujoco.png'))[..., :3]
cad_uri = to_data_uri(cad[30:680, 230:670])       # tight crop around the robot

n_runs = len([d for d in os.listdir('runs')
              if os.path.exists(os.path.join('runs', d, 'model.zip'))])
for k, v in [('hero', hero), ('dash7', dash7), ('stride', stride),
             ('shuffle', shuffle), ('lunge', lunge), ('flyin', flyin),
             ('cad', cad_uri)]:
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
table {{ border-collapse:collapse; width:100%; font-size:14.5px; min-width:620px; }}
th,td {{ text-align:left; padding:12px 14px; border-bottom:1px solid var(--border); }}
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
.cadgrid {{ display:grid; grid-template-columns:minmax(0,300px) 1fr; gap:24px; align-items:center; }}
@media (max-width:680px) {{ .cadgrid {{ grid-template-columns:1fr; }} }}
.cadgrid img {{ width:100%; display:block; border:1px solid var(--border); border-radius:10px; }}
ul.plain {{ margin:0; padding-left:20px; }}
ul.plain li {{ margin-bottom:9px; }}
code {{ font-family:var(--mono); font-size:.88em; background:var(--panel2); padding:1px 6px;
  border-radius:5px; }}
.rule {{ height:1px; background:var(--border); border:0; margin:48px 0; }}
</style>

<div class="wrap">
  <p class="eyebrow">MuJoCo · PPO · training log · day 3</p>
  <h1>It sprints on honest muscles now.</h1>
  <p class="lede">Day three gave the simulation the real servos' torque limits —
  and discovered the old gait was physically impossible: it demanded torque no
  STS3215 can deliver at any voltage. Retrained inside the true motor envelope,
  on the CAD-true body, the robot dashes 2&nbsp;m in 2.7&nbsp;s — GoPro on its
  head, bumps underfoot, 16/16.</p>
  <div class="meta">
    <span>2026·07·09 → 07·11</span><span>8× parallel envs · CPU</span>
    <span>{n_runs} trained policies</span>
  </div>

  <div class="kpis">
    <div class="kpi"><div class="v" style="color:var(--accent)">2.72<span style="font-size:15px"> s</span></div><div class="k">median 2 m dash · 11.1 V</div></div>
    <div class="kpi"><div class="v">16/16</div><div class="k">finishes · even with GoPro + bumps</div></div>
    <div class="kpi"><div class="v">3S ✓</div><div class="k">decided · swap-window battery bay</div></div>
    <div class="kpi"><div class="v">1.04<span style="font-size:15px"> kg</span></div><div class="k">CAD-true body incl. camera</div></div>
  </div>

  <figure style="margin-top:34px">
    <img class="film" src="{hero}" alt="Filmstrip of the biped sprinting 2 meters with a GoPro mounted on top">
    <figcaption>dash_11v1_hard · 2 m dash under real STS3215 torque limits (11.1 V) — a finish only counts if it's still upright 1 s after the line</figcaption>
  </figure>

  <h2><span class="n">01</span> The reckoning: the old gait was unbuildable</h2>
  <p>Until now the sim's actuators were idealized position servos — infinitely
  strong. Day three replaced them with the real thing: PD control clamped to the
  STS3215's torque–speed envelope (2.94 N·m stall tapering to zero at no-load
  speed, scaled by supply voltage; parameters fitted to a servo-like step
  response, then randomized in training). Verdict on the old champion:
  <b>terrain_v4 demanded ~3 N·m at 4.4 rad/s — outside the motor's envelope at
  any voltage</b>. Dropped onto honest actuators it collapses within 1.5 s.
  Warm-starting couldn't fix it (same lesson as the shuffle); training from
  scratch inside the envelope could.</p>

  <h2><span class="n">02</span> The battery verdict: 2S works, 3S wins — <b>3S it is</b></h2>
  <p>Same dash task, two supply voltages (16 episodes each; a finish = crossing
  2 m and standing upright 1 s later):</p>
  <div class="tblwrap">
    <table>
      <thead><tr><th>Policy</th><th>V</th><th>Flat</th><th>Median dash</th><th>+GoPro 154 g</th><th>10 mm bumps</th><th>GoPro + bumps</th><th>Model-error DR</th></tr></thead>
      <tbody>
        <tr><td class="name"><b>dash_11v1_hard</b></td><td>11.1</td>
          <td><span class="pill g">16/16</span></td><td>2.72 s</td>
          <td><span class="pill g">16/16</span></td><td><span class="pill g">16/16</span></td>
          <td><span class="pill g">16/16</span></td><td><span class="pill g">16/16</span></td></tr>
        <tr><td class="name">dash_7v4_hard</td><td>7.4</td>
          <td><span class="pill g">16/16</span></td><td>4.36 s</td>
          <td><span class="pill g">15/16</span></td><td><span class="pill g">16/16</span></td>
          <td><span class="pill a">12/16</span></td><td><span class="pill g">16/16</span></td></tr>
      </tbody>
    </table>
  </div>
  <figure style="margin:18px 0 6px">
    <img class="film" src="{dash7}" alt="Filmstrip of the 7.4-volt dash gait">
    <figcaption>dash_7v4_hard · the 2S gait — slower but grounded (no flight phase), arguably the better first-hardware candidate</figcaption>
  </figure>
  <p class="muted" style="margin-top:12px"><b>Decision made 07·11: 3S.</b> The
  tower was reworked around it — a Zeee 3S 850 mAh pack (74 g, XT30) now
  tilt-loads through a window in the rear wall for tool-free swaps (peel the
  belt, tug the ribbon; the fly-in animation below shows the pack passing
  through the window). Torso inertia was rebuilt for the lighter pack and the
  dash policies re-verified on it: 16/16, median 2.67 s with the GoPro on.
  2S remains a tested fallback at every level.</p>

  <h2><span class="n">03</span> The camera rides on top</h2>
  <div class="card">
    <div class="cadgrid">
      <img src="{cad_uri}" alt="CAD assembly render with the GoPro MAX mounted on top of the torso">
      <div>
        <p style="margin-bottom:10px">A GoPro MAX (154 g) now mounts on a printable
        three-prong base atop the tower — a bolt-on <b>crash fuse</b> that shears
        before the tower does. The driver board moved inside; total
        1043 g, CG +29 mm. The policy trains with the payload randomized 0–170 g,
        so <b>one policy handles camera-on and camera-off</b>.</p>
        <ul class="plain" style="font-size:15px">
          <li>Slots 3.2 mm — test-fit the MAX's fingers before printing everything.</li>
          <li>All 30 joint-sweep interference checks still pass.</li>
        </ul>
      </div>
    </div>
    <figure style="margin-top:18px">
      <img class="film" src="{flyin}" alt="Frames from the fly-in assembly animation">
      <figcaption>assembly fly-in — every part approaches along its real insertion path (full animation: cad/renders/assembly_flyin.mov)</figcaption>
    </figure>
  </div>

  <h2><span class="n">04</span> How it got here</h2>
  <ol class="timeline">
    <li>
      <h3>A Gymnasium env around the MuJoCo model</h3>
      <p>36-dim observation, actions as residual target angles around the
      standing pose. Reward: forward velocity + upright + alive − energy − jitter.</p>
      <span class="tag">day 1 · walker_env.py</span>
    </li>
    <li>
      <h3>Baseline PPO found the cheat, not the gait</h3>
      <p>Uncapped forward reward → a 1.3 m faceplant dive. Fixed by capping
      rewarded speed so a steady gait out-earns a lunge; domain randomization
      then hardened the walk against ±15% model error.</p>
      <figure style="margin:4px 0 12px">
        <img class="film" src="{lunge}" alt="Filmstrip of the robot lunging forward and falling">
        <figcaption>ppo_baseline · a forward dive, not a walk</figcaption>
      </figure>
      <span class="tag">day 1 · shaped_v2 → v6</span>
    </li>
    <li>
      <h3>The shuffle became a stride</h3>
      <p>Feet-air-time + single-support shaping restructured the gait (speed
      rose 0.53→0.79 m/s with the cap unchanged), and a per-episode procedural
      heightfield made it hold up on ±15 mm rough ground.</p>
      <div class="duo" style="margin:4px 0 12px">
        <figure><img class="film" src="{shuffle}" alt="Old shuffling gait">
          <figcaption>before — shaped_v6 shuffle</figcaption></figure>
        <figure><img class="film" src="{stride}" alt="New striding gait">
          <figcaption>after — terrain_v4 stride</figcaption></figure>
      </div>
      <span class="tag">day 2 · gait_v6 → terrain_v4</span>
    </li>
    <li>
      <h3>Honest muscles, real body, a stopwatch</h3>
      <p>CAD-true masses and mesh-derived inertia, the STS3215 torque–speed
      envelope, the GoPro payload, and the timed 2 m dash — the policies above.</p>
      <span class="tag">day 3 · dash_11v1_hard / dash_7v4_hard</span>
    </li>
  </ol>

  <h2><span class="n">05</span> Open items</h2>
  <div class="card accent">
    <ul class="plain" style="margin:0">
      <li><s><b>Control latency</b> — the day-3 top risk.</s> Closed: the 0/16
      collapse was an artifact of a doubled whole-step delay; with sub-step
      modeling, realistic 2–5 ms latency barely registers. Latency-DR
      fine-tunes (<b>dash_11v1_hardlat3</b>: 28/32, median 2.72 s at 4 ms +
      GoPro; <b>dash_7v4_hardlat</b>: 32/32, 4.50 s) push the cliff to
      ~10–14 ms — beyond that it's a firmware call (run the loop on the
      ESP32), not a training problem. Details in DESIGN.md §sub-step latency.</li>
      <li><s><b>2S or 3S</b> — the battery-bay decision above.</s> Decided: 3S,
      bay reworked for tool-free swap, order unblocked.</li>
      <li><b>Overnight (day 4):</b> the robot got its real sensor suite in sim.
      <b>dash_11v1_imu_hard</b> runs on encoders + a noisy BNO085 IMU only
      (no torso velocity/height — nothing measures them): 16/16 @ 2.28 s
      flat, 15/16 on terrain and at 6 ms latency, but <b>8–9/16 with the
      GoPro fixed at 154 g</b> — the realizable policy's one weak column.
      Fix candidates: an on-ESP32 velocity estimator fed as an observation,
      or payload-focused training.</li>
      <li><s><b>Stop-and-stand is an open problem.</b></s> <b>Solved</b> — by
      changing the question. Three dash_stop attempts each found a new
      reward exploit (all documented in DESIGN.md); the fix was retiring the
      finish-line structure for <b>command-conditioned locomotion</b>
      (track a commanded velocity + yaw rate, where zero = stand). The
      command policy stands on command 8/8 with ~4 cm drift, from any
      start, even hardened with GoPro + latency. It also gives first
      <b>turning</b> capability: pivots track both directions
      (≈0.2 rad/s left / 0.1 right — the identical-parts gait turns left
      for free), walking turns left-only so far.</li>
      <li><b>Day-5 evening reframe: NO policy has ever walked 10 s.</b>
      Dash episodes end ~1 s past the line, so sustained walking was never
      in any curriculum — champions fall at 3.7–5.1 s with the finish line
      removed. Eight targeted runs (kernel-width fix, penalty ablation,
      dense reward, two horizon fine-tunes) all fell short; horizon
      fine-tunes actively degraded the parent (stale value function).
      Verdict: scale is the binding constraint — MJX/GPU port is the next
      sim move, bigger feet the cheap CAD insurance. The 2 m dash burst
      remains solid and is the hardware-v1 demo.</li>
      <li><b>Command-mode walking hit an honest wall</b> (day 5): five
      from-scratch attempts across penalty scales and command mixes all
      yield excellent standing (best: 8/8, 1 cm drift, <b>6.3 W ≈ 64 min
      of battery</b>) and fragile walking — while the single-behavior dash
      lineage walks 16/16 on the same plant. New eval metrics (torso
      wobble RMS, electrical watts + runtime) came out of a video review:
      the dash gait measures 1.9 rad/s wobble and <b>36 W ≈ 15 min</b>.
      Efficiency follows competence — struggling policies burn 44–55 W
      regardless of penalties. Paths forward, by expected value: distill
      existing per-skill experts (run + stand already work) into one
      conditioned policy; MJX/GPU scale (CPU PPO is 10–50× short of the
      standard recipe); mirrored-horn leg for the yaw bias.</li>
      <li><b>The 11.1 V gait still jogs</b> (flight phase); the grounded 7.4 V
      gait is the more sim-to-real-plausible first candidate.</li>
      <li><b>Repeated shoves</b> remain unsolved (unchanged from day 1).</li>
    </ul>
  </div>

  <hr class="rule">
  <p class="muted" style="font-size:14px">Every animation also exists as .mov next
  to its gif (macOS Preview doesn't animate gifs). Reproduce:
  <code>cd sim &amp;&amp; python eval_policy.py --run-name dash_11v1_hard --render</code>
  · rank runs with <code>python compare_runs.py</code> · full write-up in
  <code>DESIGN.md</code> · CAD in <code>cad/</code> (assembly.step, fly-in mov).</p>
</div>
"""

out = 'runs/night_summary.html'
with open(out, 'w') as f:
    f.write(HTML)
print("wrote", out, "|", len(HTML) // 1024, "KB")
