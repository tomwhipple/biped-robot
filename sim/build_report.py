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
    # legacy-era gifs (dash_*, terrain_v4, ...) live only on machines that
    # trained them -- the 2026-08-02 laptop->mira move copied loco_* runs
    # only. A missing strip renders as a 1px placeholder instead of killing
    # the whole report.
    if not os.path.exists(path):
        print(f"build_report: missing {path} (legacy run not on this "
              f"machine), placeholder used")
        return np.full((1, 1, 3), 24, dtype=np.uint8)
    frames = imageio.mimread(path, memtest=False)
    frames = [f[..., :3] for f in frames]
    idx = np.linspace(0, len(frames) - 1, n).astype(int)
    strip = np.concatenate([frames[i] for i in idx], axis=1)
    return strip[::factor, ::factor]

def to_data_uri(arr):
    buf = io.BytesIO()
    imageio.imwrite(buf, arr, format='png')
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()

def strip_from_mov(path, t0, t1, n=8, factor=4):
    # referee movies are long (200+ s) and big; seek n frames with ffmpeg
    # instead of decoding the whole file. [t0, t1] picks the scenario of
    # interest out of the scorecard-order reel.
    import subprocess, tempfile
    if not os.path.exists(path):
        print(f"build_report: missing {path}, placeholder used")
        return np.full((1, 1, 3), 24, dtype=np.uint8)
    frames = []
    for t in np.linspace(t0, t1, n):
        with tempfile.NamedTemporaryFile(suffix='.png') as f:
            subprocess.run(['ffmpeg', '-y', '-loglevel', 'error',
                            '-ss', f'{t:.2f}', '-i', path,
                            '-frames:v', '1', f.name], check=True)
            frames.append(imageio.imread(f.name)[..., :3])
    strip = np.concatenate(frames, axis=1)
    return strip[::factor, ::factor]

hero    = to_data_uri(strip_from_gif('runs/dash_11v1_hard/dash.gif'))
dash7   = to_data_uri(strip_from_gif('runs/dash_7v4_hard/dash.gif'))
stride  = to_data_uri(strip_from_gif('runs/terrain_v4/walk.gif'))
shuffle = to_data_uri(strip_from_gif('runs/shaped_v6/walk.gif', n=6, factor=6))
lunge   = to_data_uri(strip_from_gif('runs/ppo_baseline/walk.gif', n=6, factor=6))
flyin   = to_data_uri(strip_from_gif(
    os.path.join(CAD_RENDERS, 'assembly_flyin.gif'), n=7, factor=3))
# CPU-referee takes of the MJX-trained policy (mp4; mimread handles them)
mjxwalk  = to_data_uri(strip_from_gif('runs/mjx_cmd_v1/ref_walk.mp4', factor=5))
mjxturn  = to_data_uri(strip_from_gif('runs/mjx_cmd_v1/ref_turn.mp4', factor=5))
mjxstand = to_data_uri(strip_from_gif('runs/mjx_cmd_v1/ref_stand.mp4', factor=5))
mjxpivot = to_data_uri(strip_from_gif('runs/mjx_cmd_v1/ref_pivot_l.mp4', factor=5))
getup    = to_data_uri(strip_from_gif('runs/mjx_getup_v1/ref_getup.mp4', n=6, factor=6))
getup2   = to_data_uri(strip_from_gif('runs/mjx_getup_v2/ref_getup.mp4', n=6, factor=6))

cad = imageio.imread(os.path.join(CAD_RENDERS, 'assembly_mujoco.png'))[..., :3]
cad_uri = to_data_uri(cad[30:680, 230:670])       # tight crop around the robot

# marathon-era reels (v5body plant, pelvis v6). The reel is scenario takes in
# scorecard order; the early minute is the walking block (line/backward/
# sidestep), which is the part worth a strip.
v18b_walk = to_data_uri(strip_from_mov('runs/loco_v18b_mix/loco_v18b_mix.mov',
                                       2, 55, factor=5))
v18b_all  = to_data_uri(strip_from_mov('runs/loco_v18b_mix/loco_v18b_mix.mov',
                                       60, 205, n=8, factor=5))
getup12   = to_data_uri(strip_from_mov('runs/getup_v12/getup_v12.mov',
                                       1, 19, n=6, factor=6))
ghost_ab  = to_data_uri(imageio.imread(
    'runs/loco_v18d_still/stand_ghost_ab.png')[..., :3][::2, ::2])
v20_walk = to_data_uri(strip_from_mov('runs/loco_v20mirror/loco_v20mirror.mov',
                                      2, 55, factor=5))
v21_walk = to_data_uri(strip_from_mov('runs/loco_v21sched/loco_v21sched.mov',
                                      2, 55, factor=5))
v20_all  = to_data_uri(strip_from_mov('runs/loco_v20mirror/loco_v20mirror.mov',
                                      60, 205, n=8, factor=5))

n_runs = len([d for d in os.listdir('runs')
              if os.path.exists(os.path.join('runs', d, 'model.zip'))])
for k, v in [('hero', hero), ('dash7', dash7), ('stride', stride),
             ('shuffle', shuffle), ('lunge', lunge), ('flyin', flyin),
             ('cad', cad_uri), ('mjxwalk', mjxwalk), ('mjxturn', mjxturn),
             ('mjxstand', mjxstand), ('mjxpivot', mjxpivot), ('getup', getup),
             ('getup2', getup2)]:
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
  <p class="eyebrow">MuJoCo · PPO → MJX/GPU · training log · updated 2026·08·24</p>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·25</span></p>
    <p><b>Best card yet (84/144) — and the next wall has a name: the clock
    caps the speed.</b> The knee-bias A/B resolved in the most instructive
    way possible: BOTH arms fixed the passive stand. v23knee (the +0.10 rad
    bias) went stand_off 7/8 with 0% falls overall, proving the physics
    probe right — but the plain control v22fix_b went stand_off <b>8/8</b>
    on its own (the stand-com kernel just needed a second night to
    consolidate a passively stable pose) <i>and</i> recovered turn_180 to
    7/8 (hErr 8°, was 25–30° everywhere else), posting <b>84/144</b> —
    the best overall card to date. Third data point of the same lesson:
    give the recipe training time before adding terms. v22fix_b is the
    line; the knee-bias knob stays on the shelf, validated, in case the
    sag returns. Rhythm on both: 98–99% alternation, CV 0.12–0.14 (soft
    watch item — was 0.06). What's left all shares one root cause:
    <b>top speed is pinned at 0.37 m/s ≈ 1.5 Hz × 25 cm strides</b>.
    speed_ladder (top_speed 0.37 vs 1.0 commanded), reversal (t_reverse
    0.53 s — fast! — but never reaches the go-speed), and the metronome's
    slow rung (cadence_err 30% at 1.0 Hz vs 2% elsewhere) are all the
    policy locked to one cadence by the very schedule reward that cured
    the limp. Fix built today: a <b>speed-coupled gait clock</b> —
    cadence scales with the commanded planar speed (sqrt law, ×1.0 at
    0.35 m/s, clipped [0.7, 1.7]), one deterministic contract mirrored in
    the training env and the referee (parity-tested), firmware threading
    to follow before deploy, exactly the clock-freeze path. Tonight:
    v24clockv (speed-clock on, warm from v22fix_b) vs v22fix_c (plain
    control).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·24</span></p>
    <p><b>v22fix is the new line — and the passive stand has a second,
    subtler failure.</b> The 08·19 A/B: v22fix (heading 0.3 + stand-com
    1.0) kept the metronome perfectly (100% alternation, stride CV 0.06 —
    heading at 0.3 is rhythm-safe where 1.0 wasn't) and the stand-com
    kernel did exactly its job: standing CoM went from 28 mm aft to
    1–6 mm. Falls 4%, square/goal-home 8/8. The plain control
    (v21sched_c) meanwhile <i>decayed</i> — rhythm slipped to 95%/CV 0.20
    and the card fell to 58/144, so the shaping terms aren't just
    scaffolding, they hold the behavior up. But stand_off only reached
    3/8: with the CoM centered, the per-joint probe shows the remaining
    failure is a slow <i>sag-collapse</i> — knees and ankles fold
    together ~1.3°/s from a dead-straight leg. A pure-physics sweep from
    v22fix's own stand pose found the fix: a +0.10 rad knee bias flips
    the release from a 28 cm collapse to a 5 cm hold (the opposite bias
    falls backward). That's now <code>--w-stand-knee</code>; tonight:
    v23knee (+knee bias) vs v22fix_b (plain control, also watching
    whether turn_180 — currently under-rotating by 25° with zero falls in
    both arms — recovers with time). Note for the hardware session:
    stand_off passivity rests on the 0.35 Nm backdrive-friction estimate
    — it's the cheapest, safest scenario to validate sim-vs-real
    first.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·19</span></p>
    <p><b>A/B verdict: keep the rhythm, drop the heading kernel at
    1.0.</b> The plain continuation (v21sched_b, +35M) kept the
    metronomic gait (100% alternation, stride CV 0.06) <i>and</i>
    re-learned steering inside it — line 6/8, backward 7/8, turn 5/8,
    square 7/8, goal-home 7/8. The heading arm (v21head) recovered
    station-keeping brilliantly (stand_off 7/8, falls 1%, 8 scenarios
    clean) but paid with the crown jewel: alternation fell to 88%, stride
    CV back to 0.33 — at weight 1.0 the integrated-heading kernel pushes
    the policy back into off-schedule correction steps. v21sched_b is the
    line going forward. Remaining sore spots on the b-arm: stand_off
    falls 8/8 and metronome — which now tracks tempo (cadErr 2%, was the
    original failure) and fails only on wobble during changes. Both reels
    confirm the full-footprint foot collision fix: standing feet stay
    separated. <b>stand_off diagnosed same day:</b> the pose-at-cut probe
    shows the v21 line parks its standing CoM 26–28 mm aft of the midfoot
    point; with torque released the ankle gravity moment (~0.60 Nm) beats
    the servo backdrive friction (~0.35 Nm ≈ 16 mm of offset) and it
    topples backward at ~6.5 s — policies standing at ≤18 mm survive,
    including v21head. Nothing in the recipe shaped the double-support
    stand CoM (w_com_stance is single-support only), so a stand-gated
    CoM-over-midfoot kernel (<code>--w-stand-com</code>) now exists.
    Tonight: v22fix (heading 0.3 + stand-com 1.0 — disjoint gates, so
    scenario attribution stays clean) vs v21sched_c (plain control).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·18</span></p>
    <p><b>Cadence fixed — the limp is gone. Cost: steering.</b> The
    contact-schedule reward, one night in (loco_v21sched): cadence goes
    metronomic — 100% step alternation, stride period 637 ± 39 ms
    (CV 0.06 vs 0.21–0.30 before), step length 20 ± 3 cm (was
    12 ± 6).</p>
    <figure style="margin-top:14px">
      <img class="film" src="{v21_walk}" alt="Filmstrip of the v21sched policy walking with an even, metronomic cadence">
      <figcaption>loco_v21sched · the contact-schedule reward: the surge-stall limp is gone.</figcaption>
    </figure>
    <p class="muted" style="font-size:14px;margin:10px 0 0">The rhythm
    result cost lane keeping — line/backward pass rates fell to 2/8 and
    0/8 purely on lateral drift (0.21 m vs the 0.15 limit) and heading
    (18°), with zero falls: the policy used to steer with off-schedule
    correction steps and hasn't relearned steering inside the new rhythm
    in 45M steps. Tonight's A/B, both warm from v21sched: plain
    continuation vs <code>--w-heading 1.0</code> (integrated-heading
    kernel). stand_off is also still regressed (1/8) — the deploy
    candidate remains v19feet_b until this line recovers.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·17</span></p>
    <p><b>Verdict (Tom): the forward walk still looks like limping —
    confirmed, but it isn't left/right.</b> Per-leg probe at 0.35 m/s:
    step counts, lengths, and swing times now match across sides (83/83
    steps, 12.1 cm both legs, no per-seed side bias). What's wrong is the
    <i>rhythm</i>: step length 12&nbsp;±&nbsp;6 cm, touchdown phase
    0.53&nbsp;±&nbsp;0.27 of the stride cycle, and long steps arrive in
    bursts (lag-1 autocorr up to +0.57) — a surge-stall-surge cadence
    that reads as a limp. Same root cause as metronome 0/8 and speed mae
    0.30: the policy ignores its own gait clock. Next lever: a
    contact-schedule reward (feet paid for touching down on the clock's
    phase), then AMP motion priors.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·15</span></p>
    <p><b>Left/right symmetry fixed — but the walk still isn't smooth
    (see the 08·17 verdict above). And the feet stopped ghosting through
    each other.</b> Two structural fixes landed overnight. First,
    the plant had no left-foot/right-foot collision: on turns and sidesteps
    the soles interpenetrated on ~half of all steps, up to 46&nbsp;mm deep.
    Explicit sole/shank contact pairs plus a retrain
    (<b>loco_v19feet_b</b>) closed the hole — the same probe now measures
    ≤2% shallow-contact steps. Second, <b>loco_v20mirror</b> added a
    mirror-symmetry loss on the policy itself (penalize
    π(mirror(obs))&nbsp;≠&nbsp;mirror(π(obs)), the signed permutation
    derived from the plant and pinned by physics tests): <b>gait asymmetry
    fell 31% → 12%</b> — a number the per-touchdown swing penalty hadn't
    moved in 110M+ steps — with power down 17.6 → 14.0&nbsp;W and 6/18
    scenarios fully clean (best yet).</p>
    <figure style="margin-top:14px">
      <img class="film" src="{v20_walk}" alt="Filmstrip of the v20mirror policy walking the line, backward, and sidestep scenarios with a symmetric gait">
      <figcaption>loco_v20mirror · walking block — backward 8/8 · sidestep 7/8 + 8/8 · rough 8/8 · square-return 8/8 (7 cm) · gait asym 12%</figcaption>
    </figure>
    <figure style="margin-top:14px">
      <img class="film" src="{v20_all}" alt="Filmstrip across the later scenarios: turns, pushes, pursuit, drills">
      <figcaption>…the hard block: pursuit 8/8 (gap 18 cm) · goal-home 7/8 (24 cm) · pushes/speed/drills still open · watch item: torque-off stand regressed 7/8 → 0/8 vs v19feet_b</figcaption>
    </figure>
    <p class="muted" style="font-size:14px;margin:10px 0 0">The run was cut
    at the 07:00 curfew (50.8M of 70M steps), so the mirror loss hasn't
    converged — sym_loss was still falling (0.010 → 0.008).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·12</span></p>
    <p><b>The v5body era: new pelvis, honest joint limits, and a 90-hour
    marathon.</b> The robot got its as-built body (pelvis v6, camera CG
    corrected by 68 mm, every servo's measured 2026-08-02 range finally
    applied — knees ±95°), and the referee grew a responsiveness suite:
    command-reversal latency, metronome cadence, squats, weight shifts.
    Best policy so far is <b>loco_v18b_mix</b>: <b>78/144</b> scenario
    seeds, <b>11% falls</b>, goal-home to 5 cm, first-ever turn-180 passes,
    and a <b>0.5 s reversal response</b>.</p>
    <figure style="margin-top:14px">
      <img class="film" src="{v18b_walk}" alt="Filmstrip of the v18b_mix policy walking the line, backward, and sidestep scenarios">
      <figcaption>loco_v18b_mix · walking block of the referee reel — line 6/8 · backward 8/8 · sidestep 8/8 + 8/8 · rough ground 8/8</figcaption>
    </figure>
    <figure style="margin-top:14px">
      <img class="film" src="{v18b_all}" alt="Filmstrip across the later scenarios: turns, pushes, pursuit, drills">
      <figcaption>…and the hard block: turn-180 4/8 (hErr 19°) · pursuit 8/8 (gap 10 cm) · pushes/speed/drills still 0/8 — the open front</figcaption>
    </figure>
    <figure style="margin-top:14px">
      <img class="film" src="{getup12}" alt="Filmstrip of the failed get-up attempt: the robot rocks but never rises">
      <figcaption>getup_v12 · the pre-registered A/B on the new body: 0/16 — PPO shaping can't find an armless rise even with full ROM. Verdict stands: next is phase-indexed tracking, not more shaping.</figcaption>
    </figure>
    <figure style="margin-top:14px">
      <img class="film" src="{ghost_ab}" alt="Ghost composites of the standing hold, before and after the stillness work: the before is blurred by motion, the after is sharp">
      <figcaption>2026·08·13 · the shaking, fixed: 10 s stand ghost composites. v18b dithers (blur, 20.2 (rad/s)² joint motion, 40 W); v18d holds still (0.04 (rad/s)², 2.2 W standing — below the torque-off baseline). stand_10s 8/8, stand_off 8/8, falls 2%.</figcaption>
    </figure>
    <p class="muted" style="font-size:14px;margin:10px 0 0">2026·08·14, the
    frozen-gait-clock runs (v18e/v18f): stillness kept (0.9 W, wobble 0.08
    rad/s, torque-off stand 7/8) AND the cost of calm repaid — turn-180
    8/8 at 4° heading error, sidesteps and rough ground 8/8. The recipe is
    now trading scenarios against each other instead of accumulating
    (returns/reversal dipped as turning recovered; overall flat at ~77/144),
    so the next lever is structural: mirror-symmetry augmentation, then AMP
    motion priors. Full story below is the July log.</p>
  </div>

  <h1>It walks. Ten seconds, and it can be told what to do.</h1>
  <p class="lede">Five days of CPU training could sprint but never walk —
  no policy survived 10&nbsp;s, and day 5 proved the missing ingredient was
  scale. Tonight's first GPU run (150M steps in 2h18m on a desk-side RTX
  4070&nbsp;Ti) produced <b>one policy that walks, turns, pivots, and stands
  on command</b> — with the GoPro mounted, 0–8 ms latency, gear backlash,
  and IMU-only sensing, verified in the CPU engine it wasn't trained in.</p>
  <div class="meta">
    <span>2026·07·09 → 07·13</span><span>2048× parallel envs · GPU</span>
    <span>{n_runs} CPU policies + 1 that ends the era</span>
  </div>

  <div class="kpis">
    <div class="kpi"><div class="v" style="color:var(--accent)">10.0<span style="font-size:15px"> s</span></div><div class="k">sustained walk · the old wall</div></div>
    <div class="kpi"><div class="v">6/6</div><div class="k">command scenarios pass CPU referee</div></div>
    <div class="kpi"><div class="v">150M</div><div class="k">steps · 2h18m · ~19× all of day 5</div></div>
    <div class="kpi"><div class="v">5.2<span style="font-size:15px"> W</span></div><div class="k">commanded stand · ~110 min battery</div></div>
  </div>

  <figure style="margin-top:34px">
    <img class="film" src="{mjxwalk}" alt="Filmstrip of the biped walking steadily for ten seconds with a GoPro mounted on top">
    <figcaption>mjx_cmd_v1 · commanded 0.6 m/s walk, full 10 s — CPU-referee take (best of 8; 7/8 seeds survive), GoPro + latency + backlash + IMU-noise DR</figcaption>
  </figure>

  <h2><span class="n">01</span> The wall came down at scale</h2>
  <p>The day-5 diagnosis said sustained walking was a <i>training-scale</i>
  problem, not a reward problem. Tested tonight: the same reward family and the
  same plant, ported to MJX (JAX) and trained with brax PPO at 2048 parallel
  robots on Mira's RTX 4070&nbsp;Ti — <b>150M steps in 2h18m</b>, versus ~8M
  overnight on the CPU rig. Hard mode from step zero: realizable IMU-only
  observations, 0–8 ms latency, 0.5–1° backlash, servo-gain error, random
  shoves, GoPro payload. Mean episode length rose 36 → 465/500 with no
  curriculum and no warm start.</p>
  <p>Trust rule: MJX numbers are never the claim. The port is parity-tested
  against the CPU engine (per-step physics exact to 1e-11; one documented
  contact-manifold difference at impact transients), and every policy is
  re-evaluated by the <b>CPU referee</b> — the engine every previous result
  lived in — under the conditions of the claim. 8 seeds per scenario, 10 s
  episodes:</p>
  <div class="tblwrap">
    <table>
      <thead><tr><th>Command</th><th>Survive 10 s</th><th>Speed err</th><th>Wobble RMS</th><th>Servo draw</th></tr></thead>
      <tbody>
        <tr><td class="name"><b>walk 0.6 m/s</b></td><td><span class="pill g">7/8</span></td><td>0.12 m/s</td><td>0.73 rad/s · calm</td><td>19.4 W</td></tr>
        <tr><td class="name">walk 0.35 m/s</td><td><span class="pill g">8/8</span></td><td>0.06 m/s</td><td>0.58</td><td>14.5 W</td></tr>
        <tr><td class="name">stand</td><td><span class="pill g">8/8</span></td><td>0.01 m/s</td><td>0.06</td><td>5.2 W</td></tr>
        <tr><td class="name">pivot left / right</td><td><span class="pill g">16/16</span></td><td>—</td><td>0.08</td><td>2–7 W</td></tr>
        <tr><td class="name">walking turn 0.4 + 0.4 rad/s</td><td><span class="pill g">8/8</span></td><td>0.09 m/s</td><td>0.66</td><td>15.5 W</td></tr>
      </tbody>
    </table>
  </div>
  <div class="duo" style="margin:18px 0 6px">
    <figure><img class="film" src="{mjxturn}" alt="Filmstrip of a walking left turn">
      <figcaption>commanded walking turn (0.4 m/s, +0.4 rad/s) — turning while walking, first time in the project</figcaption></figure>
    <figure><img class="film" src="{mjxstand}" alt="Filmstrip of a motionless commanded stand">
      <figcaption>commanded stand — 8/8, ~zero drift, 5.2 W (a relaxed stand, not a trembling one)</figcaption></figure>
  </div>
  <p class="muted">One policy does all of it — the per-skill expert +
  distillation plan is retired. Soft spots, named: one walk seed in eight
  falls; heading wanders during straight walks (loose yaw-rate tracking,
  next reward iteration); pivot turn-rate authority still to be quantified.
  Full details in DESIGN.md §Stage 2b.</p>

  <h2><span class="n">02</span> It sits up — and that's a design finding</h2>
  <p>New scenario: get up after a fall. Trained from 2048 settled ragdoll
  falls (no fall termination; reward = height + uprightness progress + a
  standing bonus). The run was <b>stopped at 100M steps by a pre-agreed
  rule</b>: reward had plateaued for 30M+ steps while the standing metric
  never left zero. The policy's answer, verified by the CPU referee (0/8
  recoveries): from any fall it reorganizes into a stable <b>sit</b> in
  about a second — then waits, motionless, at 0.9 W.</p>
  <figure style="margin:18px 0 6px">
    <img class="film" src="{getup}" alt="Filmstrip: the fallen robot props itself into a seated position and stays there">
    <figcaption>mjx_getup_v1 · best of 8 referee takes — a reliable sit-up, never a stand-up</figcaption>
  </figure>
  <p>The follow-up study made it precise — and found a bug that had
  handicapped every policy ever trained: the action mapping couldn't reach
  the deep half of the knee's range (capped at −50° of −95°). With that
  fixed and the joint-limit question answered by static path analysis
  (<b>a balanced sit→stand exists at ≥95° of hip flexion, provably not at
  ≤90°</b> — the current yoke stops at 60°), a retrain at 110° moved the
  ceiling:</p>
  <figure style="margin:18px 0 6px">
    <img class="film" src="{getup2}" alt="Filmstrip: the fallen robot rises to an upright kneel and holds">
    <figcaption>mjx_getup_v2 (hip 110°, full action range) · best of 8 referee takes — rises from any fall to an upright <b>kneel</b> and holds; v1's ceiling was the sit</figcaption>
  </figure>
  <p>The remaining gap is the last 14 cm: the pike from kneel/squat to
  standing traverses a <b>±20 mm balance corridor</b> on a 90 mm foot.
  Probes cleared the physics suspects (peak torque 1.08 of 2.72 N·m
  available; failure identical at 1–4× floor friction) — it's a
  closed-loop balance problem, plus a corridor that bigger sole pads would
  widen directly. <b>Decision pending:</b> yoke redesign to −110°/+60°
  (the 95° bound stands regardless) + larger feet + one more training
  round, or ship v1 with manual reset after falls.</p>

  <h2><span class="n">03</span> The reckoning: the old gait was unbuildable</h2>
  <p>Until now the sim's actuators were idealized position servos — infinitely
  strong. Day three replaced them with the real thing: PD control clamped to the
  STS3215's torque–speed envelope (2.94 N·m stall tapering to zero at no-load
  speed, scaled by supply voltage; parameters fitted to a servo-like step
  response, then randomized in training). Verdict on the old champion:
  <b>terrain_v4 demanded ~3 N·m at 4.4 rad/s — outside the motor's envelope at
  any voltage</b>. Dropped onto honest actuators it collapses within 1.5 s.
  Warm-starting couldn't fix it (same lesson as the shuffle); training from
  scratch inside the envelope could.</p>

  <h2><span class="n">04</span> The battery verdict: 2S works, 3S wins — <b>3S it is</b></h2>
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
  tower was reworked around it — a 3S 850 mAh pack (~80 g, XT30) now
  tilt-loads through a window in the rear wall for tool-free swaps (peel the
  belt, tug the ribbon; the fly-in animation below shows the pack passing
  through the window). Torso inertia was rebuilt for the lighter pack and the
  dash policies re-verified on it: 16/16, median 2.67 s with the GoPro on.
  2S remains a tested fallback at every level.</p>

  <h2><span class="n">05</span> The camera rides on top</h2>
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

  <h2><span class="n">06</span> How it got here</h2>
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
    <li>
      <h3>Real senses, commands, and an honest reckoning</h3>
      <p>The robot lost every sensor it doesn't own (IMU-only observations),
      gained a command interface (velocity + yaw rate, zero = stand), survived
      an external review (pad-true feet, backlash, power metering) — and then
      a reframe landed: dash episodes end 1 s past the line, so <b>no policy
      had ever practiced walking 10 s</b>. Eight CPU runs failed to fix it;
      the write-up called scale the binding constraint.</p>
      <span class="tag">day 4–5 · imu_hard / cmd_11v2 / the wall</span>
    </li>
    <li>
      <h3>MJX at 2048 robots: the wall comes down</h3>
      <p>The env ported to JAX (parity 1e-11 vs the CPU engine), brax PPO on
      the RTX 4070 Ti, 150M hardened steps in 2h18m — and the first policy
      that walks, turns, pivots, and stands on command, confirmed by the CPU
      referee. Scale was the whole story.</p>
      <span class="tag">day 5 night · sim/mjx · mjx_cmd_v1</span>
    </li>
  </ol>

  <h2><span class="n">07</span> Open items</h2>
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
      <li><s><b>Day-5 evening reframe: NO policy has ever walked 10 s.</b></s>
      <b>Resolved the same night</b> — the scale hypothesis was correct.
      mjx_cmd_v1 (150M GPU steps) walks the full 10 s and passes all six
      command scenarios in the CPU referee (section 01). Bigger feet are now
      optional insurance rather than a blocker; the 2 m dash remains the
      snappiest hardware demo.</li>
      <li><s><b>Command-mode walking hit an honest wall</b> (day 5).</s>
      <b>Resolved by scale</b>; the distillation plan is retired — one
      GPU-trained policy handles the whole command family. The day's
      metrics stack (wobble RMS, electrical watts) carried forward and
      referees every new policy. New soft spots to work: 1/8 walk-seed
      fall, wandering heading on straight walks (yaw kernel or heading
      term), pivot turn-rate authority unquantified.</li>
      <li><b>The 11.1 V gait still jogs</b> (flight phase); the grounded 7.4 V
      gait is the more sim-to-real-plausible first candidate.</li>
      <li><b>Repeated shoves</b> remain unsolved (unchanged from day 1).</li>
    </ul>
  </div>

  <hr class="rule">
  <p class="muted" style="font-size:14px">Every animation also exists as .mov/.mp4 next
  to its gif (macOS Preview doesn't animate gifs). Reproduce the headline:
  <code>cd sim &amp;&amp; python mjx/eval_ref.py --run mjx_cmd_v1 --video</code>
  · CPU-era policies: <code>python eval_policy.py --run-name dash_11v1_hard --render</code>
  · full write-up in <code>DESIGN.md</code> · CAD in <code>cad/</code>.</p>
</div>
"""

out = 'runs/night_summary.html'

# Recover legacy strips from the last committed page. The laptop->mira move
# (2026-08-02) brought loco_* runs only, so the July-era source gifs are gone
# from this machine -- but their rendered strips live in git. Any placeholder
# image in the fresh build gets swapped for the committed image with the same
# alt text, so republishing from mira never degrades the July sections.
import re, subprocess
def _imgs(html):
    return {m.group(2): m.group(1) for m in re.finditer(
        r'<img class="film" src="(data:image/png;base64,[^"]*)" alt="([^"]*)"',
        html)}
try:
    old = subprocess.run(
        ['git', 'show', 'HEAD:sim/runs/night_summary.html'],
        capture_output=True, text=True, check=True).stdout
    committed = _imgs(old)
    swapped = 0
    for alt, src in _imgs(HTML).items():
        if len(src) < 1000 and len(committed.get(alt, '')) >= 1000:
            HTML = HTML.replace(src + '" alt="' + alt,
                                committed[alt] + '" alt="' + alt)
            swapped += 1
    print(f"recovered {swapped} legacy strips from git")
except subprocess.CalledProcessError:
    print("no committed page to recover strips from")

with open(out, 'w') as f:
    f.write(HTML)
print("wrote", out, "|", len(HTML) // 1024, "KB")
