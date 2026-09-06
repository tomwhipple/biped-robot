"""Rebuild the training-log report page (sim/runs/night_summary.html).

Run:  .venv/bin/python sim/build_report.py
Needs: gifs under sim/runs/<run>/ and cad/renders/ (regenerate the CAD images
with cad/render_assembly.py and cad/animate_assembly.py).
Images are inlined (base64); the referee REELS are not -- a 70 MB .mov does
not belong in a data: URI, so build_report cuts small web-playable .mp4 clips
into runs/_clips/ and the page points at them with relative paths. Keep the
page and that folder together (both live under sim/runs/, which is how Tom
opens it over the SMB mount).
"""
import json
import os, base64, io, subprocess, tempfile
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

# ---------------------------------------------------------------------------
#  referee reels -> embedded clips
#
#  A run's reel is every scenario's seed-0 take back to back in scorecard
#  order (eval_precision.py --render). Two things make watchable embeds out
#  of it: knowing where each scenario starts, and re-encoding the wanted
#  scenarios small enough to sit in a web page.
# ---------------------------------------------------------------------------
CLIP_DIR = 'runs/_clips'          # gitignored with the rest of runs/
CLIP_W = 520                      # clip width in px (source reels are 640)
CLIP_CRF = 30


def run_mov(run):
    return os.path.join('runs', run, f'{run}.mov')


def reel_chapters(run):
    """[{scenario, start, end}] for runs/<run>/<run>.mov, in reel order.

    eval_precision.py writes reel_index.json alongside new reels. Older reels
    predate it, so we recover the chapters from the movie itself: the caption
    bar (top 44 px: scenario + verdict + headline) is identical for every
    frame of a take and changes only at a take boundary, so a downscaled
    top-band diff gives exact cut points. The result is cached in the same
    reel_index.json, so the scan happens once per reel.
    """
    mov = run_mov(run)
    idx_path = os.path.join('runs', run, 'reel_index.json')
    if os.path.exists(idx_path) and os.path.exists(mov) and \
            os.path.getmtime(idx_path) >= os.path.getmtime(mov):
        with open(idx_path) as f:
            return json.load(f)['chapters']
    if not os.path.exists(mov):
        print(f"build_report: missing {mov}, no clips from it")
        return []

    W, H, FPS = 160, 11, 20
    band = subprocess.run(
        ['ffmpeg', '-v', 'error', '-i', mov,
         '-vf', f'crop=in_w:44:0:0,scale={W}:{H}',
         '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
        capture_output=True, check=True).stdout
    a = np.frombuffer(band, np.uint8)
    n = a.size // (W * H)
    a = a[:n * W * H].reshape(n, W * H).astype(np.int16)
    cuts = [0] + [int(i) + 1 for i in
                  np.flatnonzero(np.abs(np.diff(a, axis=0)).max(axis=1) > 10)]
    cuts.append(n)

    # name the chapters from the scorecard: same suite, same order
    names = []
    sc_path = os.path.join('runs', run, 'scorecard.json')
    if os.path.exists(sc_path):
        with open(sc_path) as f:
            names = [k for k in json.load(f) if k != 'summary']
    chapters = []
    for i, (a0, a1) in enumerate(zip(cuts[:-1], cuts[1:])):
        name = names[i] if i < len(names) else f'take{i + 1}'
        chapters.append(dict(scenario=name, frames=a1 - a0,
                             start=round(a0 / FPS, 3), end=round(a1 / FPS, 3)))
    if len(chapters) != len(names):
        # take count and scorecard disagree (partial render, --scenarios
        # subset): the times are still right, the names are not -- say so
        # rather than mislabel a take.
        print(f"build_report: {run} reel has {len(chapters)} takes but "
              f"{len(names)} scored scenarios; chapter names unreliable")
        for i, ch in enumerate(chapters):
            ch['scenario'] = f'take{i + 1}'
    with open(idx_path, 'w') as f:
        json.dump(dict(movie=os.path.basename(mov), fps=FPS,
                       source='caption-scan', chapters=chapters), f, indent=1)
    print(f"build_report: scanned {run} reel -> {len(chapters)} chapters")
    return chapters


def _encode(mov, t0, t1, out):
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error',
                    '-ss', f'{t0:.2f}', '-to', f'{t1:.2f}', '-i', mov,
                    '-vf', f'scale={CLIP_W}:-2', '-an',
                    '-c:v', 'libx264', '-crf', str(CLIP_CRF),
                    '-preset', 'veryfast', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', out], check=True)


def clip_from_reel(run, scenarios, name):
    """Cut `scenarios` out of run's reel into runs/_clips/<name>.mp4.

    Returns (src, poster, seconds) relative to the page, or (None, None, 0)
    when the reel isn't on this machine. Non-adjacent scenarios are encoded
    separately and concatenated, so a clip can be "the turns and the stand"
    without dragging the six takes in between along.
    """
    mov = run_mov(run)
    chapters = reel_chapters(run)
    if not chapters:
        return None, None, 0.0
    have = {c['scenario'] for c in chapters}
    missing = [s for s in scenarios if s not in have]
    if missing:
        print(f"build_report: {run} reel has no take for {missing}")
    picked = [c for c in chapters if c['scenario'] in set(scenarios)]
    if not picked:
        return None, None, 0.0
    # merge adjacent takes into one seek range
    spans = []
    for c in picked:
        if spans and abs(spans[-1][1] - c['start']) < 1e-6:
            spans[-1][1] = c['end']
        else:
            spans.append([c['start'], c['end']])
    secs = sum(b - a for a, b in spans)

    os.makedirs(CLIP_DIR, exist_ok=True)
    out = os.path.join(CLIP_DIR, f'{name}.mp4')
    poster = os.path.join(CLIP_DIR, f'{name}.jpg')
    fresh = (os.path.exists(out) and os.path.exists(poster)
             and os.path.getmtime(out) >= os.path.getmtime(mov))
    if not fresh:
        if len(spans) == 1:
            _encode(mov, spans[0][0], spans[0][1], out)
        else:
            with tempfile.TemporaryDirectory() as td:
                parts = []
                for i, (a, b) in enumerate(spans):
                    part = os.path.join(td, f'{i}.mp4')
                    _encode(mov, a, b, part)
                    parts.append(part)
                lst = os.path.join(td, 'list.txt')
                with open(lst, 'w') as f:
                    f.write(''.join(f"file '{q}'\n" for q in parts))
                subprocess.run(['ffmpeg', '-y', '-loglevel', 'error',
                                '-f', 'concat', '-safe', '0', '-i', lst,
                                '-c', 'copy', '-movflags', '+faststart',
                                out], check=True)
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-ss', '0.5',
                        '-i', out, '-frames:v', '1', poster], check=True)
        print(f"clip {name}: {secs:.0f}s, "
              f"{os.path.getsize(out) // 1024} KB")
    return (os.path.relpath(out, 'runs'), os.path.relpath(poster, 'runs'),
            secs)


def scored(run, scenarios):
    """'line_1m 6/8 · turn_180 5/8' -- pass counts straight from the card, so
    a clip's caption can never flatter the take it shows."""
    sc_path = os.path.join('runs', run, 'scorecard.json')
    if not os.path.exists(sc_path):
        return ''
    with open(sc_path) as f:
        sc = json.load(f)
    bits = [f"{s} {sc[s]['successes']}/{sc[s]['n']}"
            for s in scenarios if s in sc]
    return ' · '.join(bits)


def video_figure(run, scenarios, name, blurb):
    """<figure> with a playable clip, or '' when the reel isn't here."""
    src, poster, secs = clip_from_reel(run, scenarios, name)
    if not src:
        return ''
    return f"""<figure>
      <video class="film" controls preload="none" playsinline muted loop
             poster="{poster}" src="{src}"></video>
      <figcaption><b>{run}</b> · {blurb}<br>{scored(run, scenarios)}
        · seed 1/8 takes, verdict burned in · {secs:.0f}s</figcaption>
    </figure>"""


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

# The reels worth watching right now: the policy the robot is actually
# running, the best policy in sim, and the night's negative result. Each
# night's update swaps the run names here; everything else follows from the
# scorecard (see the WATCH section in the page below).
WATCH = [
    ('loco_v26lag_s128',
     ['line_1m', 'backward_1m', 'sidestep_L', 'sidestep_R'],
     'v26lag_walk',
     'deployed student (128,128) · the walking block'),
    ('loco_v26lag_s128', ['turn_180', 'goal_home', 'stand_off'],
     'v26lag_turn_stand',
     'the same student turning, homing, and holding a torque-off stand'),
    ('loco_v25full_c',
     ['line_1m', 'backward_1m', 'sidestep_L', 'sidestep_R'],
     'v25full_c_walk',
     'best card on record (96/144), from-scratch line · the walking block'),
    ('loco_v25full_c', ['turn_180', 'goal_home'],
     'v25full_c_turn',
     'turn-180 at 6 deg heading error, then goal-home'),
    ('loco_v26servo', ['line_1m', 'push_gauntlet'],
     'v26servo_fail',
     'the act-lag negative (10/144) failing under its own trained conditions'),
]
watch_figs = [video_figure(*w) for w in WATCH]
watch_grid = "\n    ".join(f for f in watch_figs if f)
watch_html = f"""
  <h2><span class="n">00</span> Watch the current policies</h2>
  <p class="muted" style="font-size:14.5px">Referee reels, cut to the
  scenarios each run is judged on — seed 1 of 8 in every case, the same take
  the scorecard grades, PASS/FAIL and headline burned into the frame. Clips
  live in <code>sim/runs/_clips/</code> next to this page; full reels are the
  <code>&lt;run&gt;.mov</code> in each run directory.</p>
  <div class="vidgrid">
    {watch_grid}
  </div>
""" if watch_grid else ""

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
video.film {{ background:#0b0d10; }}
.vidgrid {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:20px;
  margin:20px 0 6px; }}
@media (max-width:680px) {{ .vidgrid {{ grid-template-columns:1fr; }} }}
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
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·06 · hardware thread</span></p>
    <p><b>It stood still.</b> The v30home student — rest-is-home plus the
    85 ms servo dead time measured on the bench the evening before — was
    flashed at 08:10 and armed on the pad at full gyro gain with Tom
    spotting. Within 3 s it settled into a fixed point and held it for the
    remaining 27 s: joint peak-to-peak 0.0° in every window, gyro RMS
    <b>0.002 roll / 0.020 pitch rad/s</b> where the previous day's pad
    arms on v29zero read 0.49 / 0.72, the policy's action constant to
    0.003 with the loop verifiably live (1352 ticks, beacon echo
    advancing). Tom: "fantastic!" The pose it chose is the one the sim
    twin predicted to within about 2° on every joint (knees +4.8 / −9.5°,
    right ankle −5.7°, left hip roll −3.2°) — the first time the twin has
    predicted the hardware stand. The reading is clean: the servo's dead
    time was the missing plant term, and with it in training the loop no
    longer rings on the real servo. What is left is the trained rest
    posture, a knee split that the reward prefers to home; that is a
    reward question, answered by tonight's <code>loco_v31home</code>
    (joint-space L1 home pull, weight 4). Stage 1 of the roadmap now
    needs four more pad arms, three on wood and five pushes on this
    student. Data: hw_sessions/2026-09-06/.</p>
    <p><b>Midday: rest is home, in the sim at least.</b> Tom drove v30home
    from the GUI: arming was fine, a forward click set off an oscillation
    it recovered from, and one earlier GUI arm had oscillated where mine
    had not — the GUI arms from wherever the last drive left the servos,
    my script goes to home first (hypothesis, not yet recorded). Tom's
    direction: randomise the starting foot position within the play
    limits. The sim already jittered starts by ±1.7°; that is now a flag
    (<code>--init-pose-deg</code>) at ±3°, and when set the served
    servo target starts at the jittered pose the way the firmware reseeds
    its shaper from measured joints. On Tom's daytime grant
    <code>loco_v31home</code> ran 08:49–12:25 (v30home + the joint-space
    L1 home pull at 4 + the 3° jitter, 30M warm from v30home). In the
    twin it <b>rests at home</b>: every joint within 2.1°, torso −2.2°,
    gyro RMS 0.002 — the first policy in this line to do so. The referee
    is mixed: 19/144 and stand_10s 8/8 as before, but stand_off 2/8 (the
    sim's torque-off stand from dead-straight knees sag-collapses, the
    very failure the old +0.10 rad knee bias was added for; the real
    robot stands released at zero, so this row is the backdrive model's
    opinion) and falls 33 % against v30home's 15 %, which is the cost of
    the pull under the randomisation and needs a hardware read before it
    is judged. Firmware built, not flashed; the robot keeps v30home with
    servos off. The runner was stopped after the grant's two jobs; tonight
    the training thread's v27full_d distill and v27full_e run first, then
    a 60M walking continuation of v31home.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·05 · hardware thread</span></p>
    <p><b>v29zero fails the rest-is-home gate, and the reason is
    arithmetic.</b> Both overnight runs landed with their students:
    <code>loco_v29zero_s128r24</code> (stand_10s 8/8, stand_off 7/8,
    0.5 W standing, walking 21/144 under the measured servo) and the
    gyro-DR-only attribution run <code>loco_v28gyro_s128r24</code>
    (36/144, stand 8/8) — so the gyro DR costs about five seeds of the
    v27tilt_b student's 41 and the crouch mix cost the rest of
    v28crouch's 14. (The training-thread card above quotes 36 for the
    v29zero student; the file <code>scorecard_lag.md</code> in its run
    directory says 21 — 36 is the v28gyro student.) The Stage 1 offline
    check then failed: fed the robot's home observation the v29zero
    student asks for ankles −7° and a knee at +12° (max|a| 0.175 against
    a 0.05 gate), and the teacher asks the same (0.208), so this is the
    objective, not the distill. Run in the twin for 6 s it rests with the
    left ankle at −8.5°, hip pitches ±3.9°, yaws ±1.8°, torso pitched
    3.3° forward — but hip roll only ±1.1°: <b>the 7° adduction that bound
    the legs on the robot is gone.</b> What remains is a forward lean, and
    the model explains it exactly: at q=0 the plant's CoM sits 11.3 mm aft
    of the sole midpoint (the "26–28 mm" in the reward comments was the
    v21 policies' <i>stance</i>, not the zero pose), <code>w_stand_com</code>
    (σ 20 mm, weight 1.0) collects 0.27 more reward per step once the
    torso pitch centres it (−0.9 mm at the twin's pose), and
    <code>w_stand_zero 2.0</code> charges 0.02 per step for the actions
    that do it. The real robot stands released at zero, which bounds the
    real offset below the backdrive-friction figure (~16 mm) and says
    nothing about needing a lean. Fix drafted as <code>loco_v30home</code>
    (v29zero recipe, stand-CoM kernel off, stand-zero ×5, warm from
    v29zero, 30M ≈ 3.5 h) and parked in night/queue/held: tonight's slot
    is held by v27full_d from the training thread and the order is Tom's
    call. Firmware with the v29zero student is built and the host suite is
    green (225 checks), but it is not flashed; the robot keeps the v27
    build, torque released. The roadmap's Stage 1 gate gained a twin
    rest-pose criterion (every joint within 2° of home, torso within 1°),
    since tick-1 alone is a transient.</p>
    <p><b>Evening: three arms, one clamp, one number that was missing.</b>
    Tom flashed the v29zero student anyway to see whether killing the
    adduction alone would stop the legs binding. It did — <b>hip roll
    stayed within 1.5° in all three arms and the roll-over never
    appeared</b> — and the rest of the picture stayed exactly as the twin
    predicted, only louder: a fore-aft scissor (left hip +5..+8°, right
    −5..−9°, left ankle −8°, torso 3–5° forward) with 8–14° of pitch swing
    per 3 s that never decayed. On bare wood it stood 30 s and shuffled to
    the table edge (Tom pushed it back twice); on the grippy pad it fell
    <i>backward</i> at 23 s, the swing growing to 22° once the feet could
    no longer bleed energy by sliding; on the pad with the C2 shaper at
    4 Hz it stood 30 s, wandered off the pad and ended in a 20° burst.
    Gyro RMS was 0.42–0.49 roll / 0.72–0.75 pitch rad/s in every arm; the
    twin sits at 0.005. Surface and shaper change nothing, so the swing is
    the loop and the scissor is only its amplifier. Then the Stage 0 test
    the roadmap asked for: left foot clamped flat at the desk edge, right
    foot hanging, 200 ms minimum-jerk steps of 3°, 5°, 10° on the free
    hip, the loaded ankle and the stance hip, servo trace at 40 Hz. All 16
    traces say the same thing: <b>the STS3215 follows a target after
    ~85 ms of pure dead time</b> (68–106) plus a 30 ms lag, independent of
    amplitude and load, peak speeds far below the slew limit. The sim's
    actuator had 0–8 ms of latency and a 2–12 Hz three-stage lag — phase
    that always arrives with attenuation. A dead time is phase without
    attenuation: at 2 Hz the robot passes 0.94 of the command 82° late
    (0.88 at −116° through the 10 Hz shaper) while the sim's easy draw
    passes 0.96 at −31° and its hard draw 0.35 at −138°. No training
    episode ever contained the robot's combination, which is why every arm
    since 09-03 has been gain-limited through the gyro and why the
    half-gain trick stood. <code>env_mjx</code> gained a per-episode
    servo dead-time draw (<code>--act-delay-max</code>, 0–N ticks;
    <code>State.act_hist</code> serves the target from N ticks ago ahead
    of the lag cascade; tests green, semantics checked on the GPU). Tom's
    call for tonight: <code>loco_v30home</code> with BOTH fixes — stand-CoM
    kernel off, stand-zero ×5, dead time 0–5 ticks — ahead of the training
    thread's v27full_d, and start now. Runner started 16:56 on a fresh
    pull. Also learned the easy way: releasing torque never drops the
    robot, even on one clamped foot — gear friction holds (Tom: "please
    remember that"). Data: hw_sessions/2026-09-05/.</p>
    <p><b>Night: v30home lands, the lean is gone, a knee split takes its
    place.</b> 30.5M steps in 3.6 h (2400 sps on an otherwise idle GPU),
    distilled and refereed by 21:19: <b>19/144</b>, stand_10s 8/8 with
    1.0 cm drift, stand_off 7/8, falls 15 % — the lowest fall rate of any
    student — but walking is poor (line 0/8) and the training curve had
    not plateaued (reward 297 → 495 → 562 → 505). The offline gate still
    fails: tick-1 max|a| 0.136, and in the twin it rests level (torso
    −1.1°, gyro RMS 0.003) with the knees split +5.3° / −8.9° and the
    right ankle at −7.4°. The teacher rests the same way, so it is the
    objective. Two facts explain it: the stand-zero term is a mean of
    squared <i>actions</i>, and in the full action map a 10° knee is a
    0.11 action, so weight 10 charged 0.03 per step; and a direct
    step-for-step comparison shows the base reward paying about 0.5 per
    step <i>more</i> for the split than for home, from a term not yet
    identified. New term: <code>--w-stand-home</code>, an L1 pull of the
    served target onto home in radians, stand-gated — 1.17 per step at
    this split, verified against the analytic value on the GPU.
    <code>loco_v31home</code> (v30home recipe, stand-home 4, warm from
    v30home) is queued behind v27full_d for the 09-06 slot. The v30home
    firmware is built and saved, not flashed: tomorrow's first arm can
    still answer whether the dead-time DR damps the swing, posture aside.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·04 · morning · v28crouch student on the robot</span></p>
    <p><b>Overnight.</b> <code>loco_v28crouch</code> trained 18:17–01:25
    (60M steps, six evals, reward 574 → 673, peak 781) with the new
    per-episode <b>gyro observation gain 0.5–1.2 and 0–2 tick delay</b>,
    zero-offset 3° and backlash 0.5–1.5° in its config. Queue mishaps: the
    distill entry fired at launch (locale sort put <code>39b</code> before
    <code>39-</code>) and failed on a missing config; a peer session
    re-queued it correctly at 01:25 and re-added the parked v27full_b
    job, which the 07:00 stop killed. The student's <b>lag scorecard is a
    regression everywhere but standing</b>: stand_10s 8/8 (unchanged),
    stand_off 4/8 (was 8/8), line_1m 0/8 (was 4/8), overall 14/144 vs
    41/144 for v27tilt_b's student; squat_reps 0/8 either way, so the
    crouch itself did not appear. Two things changed at once, the gyro DR
    and the crouch mix; tonight's queue (44 <code>loco_v28gyro</code> +
    45 distill/referee) runs the gyro DR alone on the unchanged v27tilt_b
    recipe to attribute it.</p>
    <p><b>On the robot now:</b> <code>loco_v28crouch_s128r24</code>,
    flashed and verified (IMU back on the second <code>imu reinit</code>,
    bias/mount/scale from NVS, link 100/100 beacons). It is the only
    policy trained to tolerate a soft gyro, so a single spotted stand arm
    at full gyro gain is the cleanest test of last night's hypothesis
    that the fall loop is gain-limited through the rate term. The
    fallback for the current v27 student is <code>obsfreeze gain=0.5</code>,
    which stood 8 s last night. Found on the way: the golden obs vectors
    for a run trained with zero-offset DR carried a random ±3° draw
    (walker_env drew it with DR off); gated, regenerated, host tests
    green.</p>
    <p><b>Midday: it was never the IMU. The student's learned stance is
    the fault.</b> Six more spotted arms: v28 at full gain fell at 4.5 s,
    v28 at half gain toppled sideways at 3.5 s with the yaws splayed, v27
    at 0.75 damped its pitch swing but drifted into a 15°/13° hip-pitch
    scissor with 13° yaw splay and rolled over at 7 s; the same on bare
    varnished wood in 4 s; and armed <i>from the policy's own tick-1
    stance</i> (posed by hand: hips adducted 6.6°/7.0°, torso 13° forward
    where the sim settles at 3°) it fell in 3.4 s with the identical
    signature. Tom saw it directly: "it moves the right leg into the left,
    resulting in the two legs bound together." The offline check settles
    the origin: fed the robot's exact home observation, the policy's
    <b>first action adducts both hips</b> (−0.26 / +0.28 ≈ 7°); fed the
    sim's first observation it outputs the same, and swapping any single
    channel (q, dq, up, gyro, phase) changes nothing. The sim twin with the
    same policy holds all of it under 6°, because its floor lets the legs
    press instead of slide; a twin with hip-roll play or friction 0.4
    still does not reproduce the drift, so the contact story is
    incomplete, but the direction is unambiguous: the standing posture
    this line learned (adduction, forward lean, knee bias +0.10 rad by
    design, CoM 26–28 mm aft of midfoot in the model while the real robot
    stands released at zero) does not exist on the robot. Tom's call:
    <b>"train the policy that the zero position is the stop/rest
    position."</b> Trainer gained <code>--w-stand-zero</code>, a
    stand-gated mean(action²) so zero command means zero action means the
    home pose; <code>loco_v29zero</code> (v27tilt_b recipe + gyro DR +
    stand-zero, knee bias off) started 11:43 under Tom's GPU grant, distill
    and referee queued behind it, the gyro-DR-only attribution run after
    that. Also ruled out today: a torso rattle (a ±5° hip-pitch wiggle at
    1 Hz gives exactly the kinematic gyro peak, 0.69 vs 0.65 rad/s) and
    the arm transient (starting at the stance changes nothing). v27 build
    is back on the robot. Data: hw_sessions/2026-09-04/.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·03 · hardware thread</span></p>
    <p><b>Three follow-ups, one correction, one honest negative.</b>
    <i>Correction first:</i> the policy on the robot is
    <code>loco_v26lag_s128</code> (the board's <code>stat</code> says so),
    trained WITH the 2–12 Hz act-lag pole and 0.5–1° backlash — last
    night's sim comparisons used the v22fix student by mistake. Redone
    with the right policy: still dead still in sim (up<sub>y</sub> RMS
    0.0009 vs 0.05–0.13 on the robot). <b>(1) Fine sweep.</b> At 0.25°
    steps the left hip roll shows a textbook backlash step: no torso
    response until +3.25°, then a jump to 1.3° that holds through the
    reversal all the way down to +1.25° — <b>2° of hysteresis, ~3° from
    home to engagement</b>. Right hip roll stays unresolved (≤0.4°
    response). Knees: the left tracks smoothly (0.27° hysteresis); the
    right at +3.5° tipped the whole robot 7° (guard fired, homed cleanly) —
    the right leg carries the weight, third time it shows. <b>(2) Play in
    the twin.</b> <code>walker_env</code> gained a FREE-TRAVEL model
    (<code>play_deg</code>/<code>play_joints</code>: a massless shaft
    tracks the command at the no-load speed, the link is driven only
    outside ±play/2 and floats undamped inside), exposed as
    <code>sil_twin --play-deg --play-joints</code>. Result: 3° or 6° on the
    hip rolls or on all ten joints, with or without 2 Hz lag, and the
    flashed policy <b>still stands dead still</b> (up<sub>y</sub> RMS ≤
    0.002). So static mechanical hysteresis, on its own, is <i>not</i> the
    mechanism — the hardware loop is being excited by something the twin
    lacks. Remaining candidates, in the order I'd test them: the
    asymmetric right-foot stance (sim stands symmetric), foot/mat
    compliance, the complementary-filter up-vector (sim's is exact), and
    dq/gyro noise. <b>(3) Firmware: the reset verdict now means moved.</b>
    <code>homeAll</code> returns <b>HOME_PENDING</b> (8) after the
    broadcast write; <code>cli::homeVerify</code>, on the housekeeping
    loop, reads all ten joints back once the slew deadline passes (worst
    move ÷ speed + 0.7 s) and stores <b>DISARMED_HOME</b> only when every
    joint is within 12 ticks of its zero, else <b>HOME_NOT_REACHED</b> (9)
    with the offenders named on the tether. Flashed 09:01, verified over
    the link: PENDING at 0.45 s, DISARMED_HOME at 0.90 s, UART "home
    readback: all 10 joints at their zeros". Both probes wait out PENDING;
    <code>tools/home_verify.py</code> watches the sequence. One more
    bench note: the servos had <b>lost torque overnight</b> (home left them
    holding at 23:00; at 08:50 a goal write was refused "torque is OFF";
    pack 12.0 V, no faults, no reboot, cause unknown) — check torque before
    any bench run. Data: hw_sessions/2026-09-03/.</p>
    <p><b>Later: Tom's yaw suspicion, a wrong inference, and the eye test
    that caught it.</b> Tom: the hip yaw joints likely have the most play
    — "the entire leg is controlled by the tiny servo axis" — and the mat
    is not a factor since the feet have no traction pads. Yaw does not
    tilt the torso and the encoders sit on the shaft side of the slop, so
    <code>tools/yaw_probe.py</code> yaws the hips in 2° steps and reads
    the <b>pelvis yaw off the gyro</b> (projected on the measured gravity
    axis, bias-corrected per step, integrated over each slow move; the
    board streams <code>imu raw</code> at ~11 Hz). Rigid-sim reference
    (<code>sim/joint_yaw_sim.py</code>): both hips the same way → pelvis
    0.78°/°, one hip 0.45°/°, opposite signs ~0. The robot did the
    reverse — same-sign commands cancelled, opposite-sign turned the
    pelvis 2.0°/1.65°, right-only went the other way from left-only —
    and I read that as <b>one hip yaw mirrored against the model</b>,
    pointing at R_hip_yaw, and asked for an eye check before touching the
    calibration. <b>The eye check refuted it.</b> Right hip +10°: the
    right toe swung to the robot's left (inward) — the model's direction.
    Left hip +10°: the left foot stayed planted and the <i>rest of the
    robot</i> swung right, so the left toe points outward — also the
    robot's left in body terms, also the model's direction. Nothing is
    mirrored; both as-built yaw signs are right. What the gyro had
    measured: the weight was on the <b>left</b> foot this morning (it was
    on the right last night — the stance asymmetry flips between
    re-homings), so the right foot slides, and a right-yaw command mostly
    <i>reacts</i> back into the pelvis, which turns the other way inside
    the left yaw chain's slop while the left servo holds. Read correctly,
    that is a yaw-play measurement: the left yaw chain let the pelvis move
    ≥1.2° under reaction torque with its servo holding, and every yaw move
    stalled 10–12 ticks (~1°) short of its goal in both directions. So yaw
    play is ~1–2°, the same order as the 2–3° in roll, not dramatically
    larger. Two lessons, both already in the rules: a gyro-inferred sign
    is not a direction check, and never change <code>cal dir</code> on
    inferred evidence. Data: hw_sessions/2026-09-03/yaw2.csv,
    yaw3_opposite.csv, yawcheck_*.jpg, sim_ref/sim_yaw.txt.</p>
    <p><b>Afternoon: the clean play numbers, one clamped foot at a time.</b>
    Tom clamped one foot to the desk and the other leg was lifted clear
    (<code>joint_sweep --base</code> holds the free leg at hip −25°, knee
    −50°, ankle −25°, roll +10° — a −40° hip swing leaned the torso ~14°
    and crept; both-feet-clamped locks the torso and measures nothing).
    The clamped leg is the loaded leg by construction, so every joint
    finally reads under real load (90–170 counts) with real hysteresis.
    Torso-tilt hysteresis at the same command, hw / rigid sim: <b>L hip
    roll 1.06/0.20, L hip pitch 0.97/0.20, L ankle 1.44/0.00, L knee
    1.35/0.00, R hip pitch 0.58/0.10, R hip roll 0.82/0.10, R knee
    0.70/0.00</b>. Separately, the loaded servos sit 10–20 ticks (up to
    1.8°) short of goal — about 16 N·m/rad, which the sim's kp = 12 servo
    already models. So the un-modelled part is <b>~1–1.5° of mechanical
    hysteresis per loaded joint, both legs, all axes</b>; yesterday's
    "pitch is tighter" came from unloaded legs. Then the outlier: the
    <b>right ankle</b>. At +1.0° of ankle the torso jumped 0.4→5.2° and,
    on a repeat with the free foot raised higher, 0.5→6.5° — identical
    ticks both times: servo lag −4→+1, load −40→0. The right knee at −1.0°
    did the same (5.9°). The right ankle is holding the robot's pitch
    moment on one side of a ~5° gap and a 1° command drops the robot
    through it; the left ankle swept ±4° smoothly. <b>Inspect the right
    ankle</b> (horn, bracket, screws) before trusting any right-leg
    number. Two more of Tom's observations went straight into the tool:
    the 0.5° steps at 60 ticks/s were jerky and rang the structure on a
    single-leg stance while the big lift moves were smooth
    (<code>joint_sweep</code> now steps at 10 ticks/s with a 1 s settle),
    and the clamped-foot configuration looks like a better range-of-motion
    rig than the hang stand — the swept leg is loaded as in use and the
    free leg can be posed anywhere. Data: hw_sessions/2026-09-03/
    sweep6_Lclamped.csv, sweep12–14_Rclamped.csv.</p>
    <p><b>Evening: the horns were loose, the zeros moved, and the crouch
    oscillation was the command, not the robot.</b> Tom found and tightened
    loose screws at the servo wheels on both legs, so the stand was
    <b>re-zeroed</b> by eye (<code>cal zero</code> + <code>cal save</code>):
    the pitch chains had wandered L hip pitch −3.8°, L knee −4.7°, L ankle
    +1.7°, R hip pitch −1.9°, R knee +1.1°, R ankle −1.1°, yaws and rolls
    under 0.3° — the loose horns were the asymmetric stance. The right
    ankle's 5–7° release <i>survived</i> the tightening (five reproductions,
    the free foot verifiably clear by servo load), so that gap is elsewhere
    in the ankle. Then range of motion, one foot clamped
    (<code>tools/leg_rom.py</code>, the level-foot family hip −θ / knee
    −2θ / ankle −θ): the free leg lifts to θ = 40°, knee −80°, ~4.7 cm,
    within 4 ticks on every joint; the two-leg crouch reaches knee −80°
    within 4 ticks, torso within 2°. Tom saw the crouch <b>oscillate</b>,
    then saw hips and ankles move first with the knees catching up, then
    saw it get <i>worse</i> when all servos ran in unison. The gyro bursts
    agreed: constant speed peaks 0.3–0.7 rad/s of torso rate, unison
    0.2–0.6, dominant ~2.5 Hz — the structure, hit by every velocity step,
    and hit coherently when ten servos step together. The fix is firmware:
    <code>pose … s&lt;ms&gt;</code> streams a <b>minimum-jerk</b> profile
    from the housekeeping loop at 50 Hz with per-servo speeds
    (<code>cli::poseTick</code>), and <code>home</code> now rides it too
    (≥ 2.5 s). Result: crouch peaks <b>0.007–0.011 rad/s</b>, tails
    0.004 — the sensor's noise floor, a 30–70× reduction; home from a 20°
    crouch 1.0 → 0.14. Two side findings: the servos do not hunt at hold
    (all ten at factory P32/D32/I0, dead zone 1 tick, read with the new
    read-only <code>reg</code> command), and the "residual oscillation"
    in the tails was the <b>IMU tearing 16-bit reads</b> — isolated
    ±0.1396 rad/s spikes, exactly 256 LSB at the 1024 dps range, about one
    sample in a hundred; sync-sample mode did not stop them, a
    read-twice-median-of-three in the driver did (0 in 400). That same
    driver feeds the control loop, so the policy had been seeing an 8°/s
    rate spike every couple of seconds. The lesson for the walking policy
    is the next experiment: the armed loop's C2 shaper pole sits at 10 Hz,
    above the 2.5 Hz mode — try 3–5 Hz live (<code>shape</code>) with Tom
    spotting, or train with a jerk penalty. Robot left with torque
    released, Tom's idle state. Data: hw_sessions/2026-09-03/rom_*,
    hold_hunt20*, servo_registers.txt, rezero.log.</p>
    <p><b>Late evening, Tom watching the one-motion crouch.</b> "Much
    better — still some random shakes on the ends", then "make sure we
    don't have single ticks that are too close together in time", then
    the architecture rule: <i>the control loop runs entirely on the robot;
    if a position is commanded the controller takes it there smoothly;
    telemetry back is a debug tool we can switch off.</i> So the streamer
    grew live tokens (<code>h</code> speed headroom, <code>a</code> servo
    accel, <code>b</code>/<code>i</code> glide rule: a joint is written
    when it has ≥ b ticks to go or ≥ 1 tick and ≥ i ms since its last
    write, speed = distance over the interval, so small end-of-profile
    steps become slow continuous creeps instead of 50 Hz pokes against the
    1-tick dead zone), and <b><code>pose</code> is now smooth by default</b>
    with the duration from the longest move. A camera frame-difference
    meter (<code>tools/video_shake.py</code>) ranked the old 125 % speed
    setting worst and 90 % best, but cannot resolve the end shakes Tom
    sees — his eye is the meter there, and the A/B/C of glide settings is
    still his call. Then "increase the IMU sample and control loop rate as
    much as practical": the loop stays at 50 Hz (the policy net is 7.5 ms
    of the 20 ms tick and the policy is trained at that step — a faster
    net and a retrain is the project item), but the IMU now has its own
    <b>250 Hz task</b> (<code>imu_sampler.cpp</code>): attitude filter at
    4 ms, tick-averaged gyro to the loop instead of one aliased sample per
    tick, I2C mutex for the bench commands, counters in <code>stat</code>.
    Verified live: 250/s with zero failures, e-stop-armed loop 0 % late.
    The full armed budget and the effect on the stand wait for a live arm
    with Tom spotting.</p>
    <p><b>Night: "you were going to instrument this for hard numbers."</b>
    Two instruments, both onboard: <code>imu ring</code> reads the 250 Hz
    gyro ring as a 50 ms envelope with peak, decay and dominant frequency;
    <code>pose … T&lt;id&gt;</code> makes the streamer log one servo's
    position, speed and load every tick through the move and 1.2 s after,
    dumped by <code>trace</code>. The IMU said the torso is still at both
    ends of a crouch (~1°/s, the noise floor) — the shake is in the legs.
    The knee trace then found the real causes, and neither was the robot.
    <b>(1)</b> The bench tool's reply pattern wanted "ok"; the smooth mode
    answers "streaming"; so every smooth pose timed out and was
    <i>re-sent 3 s into the move</i>, restarting the profile from
    mid-travel — that restart was the end-of-move shake on every crouch
    Tom judged. <b>(2)</b> Open-loop speed (profile × 0.9) let ~10 % lag
    accumulate, and the servo crept the last 60 ticks off at the floor
    speed for 1.2 s after the profile ended. <b>(3)</b> The CLI kept 13
    argv, silently dropping every tuning token after the first — the
    A/B/C glide, floor and shaper comparisons were identical runs (fixed,
    24). The streamer is now <b>closed-loop</b>: each tick reads every
    servo's measured position and commands the speed that closes the gap
    to the next sub-target within the tick. Knee trace after, two reps,
    both directions: target reached at 3.95 s of a 4.0 s profile,
    <b>0 ticks of motion and 0 reversals in the 1.2 s tail</b>. Cost: the
    stream tick stretched to ~28 ms for the ten reads. Lessons carried
    forward: never resend a motion command blindly (verify by reply, else
    by a witness joint moving), and the pelvis IMU is the wrong sensor for
    leg shake. Tom watched one: "very good!"</p>
    <p><b>The right ankle, closed.</b> A new probe (<code>tools/ankle_probe.py</code>)
    sweeps the stance ankle 0 → +3° → −3° → 0 over 6 s per segment on a
    clamped single leg, with that servo traced at 50 Hz and the torso's
    250 Hz ringing meter read per segment. First it needed the IMU back:
    with the sampler running, every flash resets the MCU mid-read and the
    QMI8658C holds SDA low, the next boot's scan finds nothing, and the
    firmware silently runs on the stub IMU — three runs' torso readings
    tonight were stub zeros before I noticed; <code>busInit</code> now
    clocks the bus free at boot. Then the verdict. <b>Both ankle servos are
    fine:</b> on either leg the shaft glides through the load reversal
    (−24 → +20 counts) at exactly the commanded rate, no jump. The 5–7°
    "release" seen with 0.5° steps was the leg's whole pitch chain giving up
    its play when the moment reverses, excited by the step. <b>The right
    chain lurches 2–4× more than the left</b> at the same command (torso
    peaks 0.148 / 0.173 vs 0.040 / 0.073 rad/s on the ±3° excursions), so
    the extra play is in the right leg's couplings outside the encoder —
    foot bracket, ankle horn, knee or hip coupling — and the next step is
    hands on those with torque on, not a servo swap. One more sign lesson:
    hip-roll abduction is +L and −R in the model; a +10° "abduction" on the
    lifted right leg swung it into the left. Data: hw_sessions/2026-09-03/
    ankleR3*, ankleL2*.</p>
    <p><b>Evening: "the goal is to run the robot via policy."</b> Tom's
    words, after the v26lag student fell the instant he armed it. So:
    <b>loco_v27tilt_b_s128r24</b> — the referee's best under the measured
    servo (41/144, 38 % falls) — was exported, its obs spec and weights
    regenerated (host suite green) and flashed. Three recorded arms, Tom
    spotting: it <b>stood 6 s</b> (torso within 0.97) and toppled when my
    probe's disarm froze the joints mid-sway (Tom: "disarming torque does
    nothing" — friction holds, the rigid body tips; the probe now ends with
    the reset edge while still armed); then fell at 3.4 s; then at 2.4 s
    with the shaper pole at 4 Hz. Every trace reads the same: a quiet
    symmetric stand for a second, then <b>growing alternating hip-pitch
    swings</b> — −9, −12, −24, −31° on the left against +6, +14, −21° on
    the right, knees ±10°, at 1–2 Hz — until the torso passes 25–40°.
    Tom: "the amount of time it remains upright is random chance; all
    examples show destructive oscillations." The referee says this student
    stands 8/8 under lag in sim, so this is a plant or sensor gap, and one
    sensor got looked at hard: the gyro. Its x axis is the body pitch axis
    after the mount, its raw zero-rate is 27°/s (far out of spec, though
    steady at rest over 32 s), its stored bias was stale by ~1.5°/s on two
    axes (recalibrated, saved), it is twice as noisy as the other axes, and
    one move test integrated 2–2.7× the accelerometer's tilt change — the
    scale question is <i>open</i>, and a known rotation by hand settles it.
    Also found and fixed on the way: with the 250 Hz sampler, every flash
    left the I2C bus hung and the firmware on the stub IMU; boot now clocks
    the bus free (3 × 32 pulses) and <code>imu reinit</code> does it live.
    <b>Training, per Tom's ask:</b> the trainer and referee gained a
    per-episode joint <b>zero-offset</b> randomisation (±Z°, obs minus the
    offset, targets plus it — the rezero moved the zeros 1–4.7° today and
    the policy must not care), parity test green, and
    <code>loco_v28crouch</code> is queued <i>first</i> for the GPU run,
    which now starts at 20:00: v27tilt_b's recipe with the crouch share
    raised 6 → 30 %, crouch to half height, backlash 0.5–1.5°, zero-offset
    3°, chained distill and lag referee. What a retrain cannot fix, and what
    tonight's run does not model: the load-reversal lurch, the 2.5 Hz mode,
    and whatever the pitch gyro is doing. Robot released at home.</p>
    <p><b>Later: the pitch gyro over-read by 18 %, and the firmware now
    corrects it.</b> Tom clamped the left foot ("hand input will not
    produce accurate results"); <code>tools/imu_scale_check.py</code>
    lifts the free leg, swings the stance ankle ±8° over 2 s, and compares
    the accelerometer's tilt change <i>at rest</i> before and after each
    move with the gyro integral through it (the rest reading answers
    Tom's moment-arm question: no arm enters a static gravity direction).
    Pitch axis: ratio <b>1.16 / 1.18 / 1.19</b>; roll axis (hip roll on
    the same clamp): 1.03 / 1.03 / 1.02, which validates the method. An
    18 % hot pitch rate is a phantom velocity on exactly the axis the
    policy oscillates in. Firmware gained a per-axis gyro <b>scale</b> in
    the sensor frame (own NVS record, unity when absent, <code>imu gscale
    x y z</code> sets and saves), set to 0.851 on sensor x; the bias was
    re-taken under it (−0.4675 → −0.3951 = 0.851×, as it must be) and the
    same clamped test now reads <b>1.01 / 1.01 / 1.04</b>. Also moved
    <code>imu reinit</code> above the stub check — the one time you need
    it is when boot fell back to the stub, and that path was refusing it.
    Not yet re-armed under the corrected gyro; that is the next arm test,
    with the same probe and cameras. Data:
    hw_sessions/2026-09-03/imu_scale_L*.log; firmware 3afe7d7.</p>
    <p><b>Night: the ablation that found the loop, and the gain that
    closes it.</b> Armed under the corrected gyro it fell in 3.8 s, same
    growth as before. So the firmware gained <code>obsfreeze</code>, which
    pins parts of the policy observation at their nominal stand values
    while the beacon and guards keep the real ones, and Tom spotted seven
    arms: <b>IMU frozen → stands 8 s</b> (swing 0.03 rad, no growth);
    <b>gyro frozen only → stands</b>; <b>up frozen only → swing guard at 5
    s</b>; joint velocities frozen with the IMU live → guard at 3 s. The
    loop closes through the gyro. Then a signed bench test
    (<code>tools/gyro_sign_check.py</code>: both ankles, both hip pitches,
    both knees, +6° over 2 s, the policy's own <code>up</code> before and
    after, the ring integral through the move) against the identical
    move in the sim on the v5body plant: gyro integral matched to 0.3°
    on every joint, and the up-vector matched in magnitude but was
    <b>rotated 66°</b>. Cause: the obs <code>up</code> is MuJoCo's
    framezaxis, the body z-axis in the <i>world</i> frame, which turns
    with yaw; the sim's yaw is ~0 at every reset, the robot's fused yaw
    is a free gyro integral walking at the residual z bias (~4°/min), so
    the tilt feedback was rotated by an angle that drifted over minutes —
    that is "how long it stays up is random chance". Firmware now strips
    the ZYX yaw before forming <code>up</code> (checked to 1e-16 against
    the zero-yaw column over 2000 random poses); the leans then match the
    sim to 0.01 on every joint. Also found: a boot that fell back to the
    stub IMU never applied the NVS calibration and <code>imu reinit</code>
    brought the sensor back uncalibrated (bias 0, mount identity) — now
    re-applied on reinit. <b>But the yaw-stripped up did not stop the
    oscillation</b> (fell at 5 s). What did: <b>gyro at half gain</b>
    (<code>obsfreeze gain=0.5</code>) — stood the full 8 s in a bounded
    0.1 rad limit cycle. The loop is gain-limited: with sign, scale, frame
    and yaw all verified, the plant has more phase lag at the crossover
    than the sim's, and the student leans on the rate term harder than
    this plant allows. Training answer, queued for tonight: randomise the
    gyro observation per episode (scale and a 0–2 tick delay) so the
    policy cannot depend on a crisp rate. Data:
    hw_sessions/2026-09-03/arm_v27_{{gscale,imufrozen,gyrofrozen,upfrozen,
    yawstrip2,dqfrozen,gyrohalf}}_*, gyro_sign_*.log.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·02 · hardware thread · evening</span></p>
    <p><b>The arm-and-fall is real, it is a ROLL limit cycle, and the sim
    cannot produce it — with any backlash.</b> Tom: "when I arm the robot it
    begins a growing oscillation that eventually results in it falling
    over" and "a fair amount of play in the joints that might not be well
    modeled by the sim". Repeated from mira three times with cameras,
    beacon joint angles (the new 40 B pose frame) and <code>obsdump</code>
    all rolling (<code>link/osc_probe.py</code>: ARM|POSE zero-command
    stand with a tilt/swing guard → ESTOP + disarm; <code>--home</code>
    proved <b>RESET SERVOS over the link</b> live: verdict DISARMED_HOME,
    every joint within 0.014 rad of zero, torque holding). Run 1 stood
    8 s; runs 2 and 3 fell <b>8.5 s and 18.8 s</b> after arming, both
    <b>sideways</b> (up<sub>y</sub> → +0.50 / −0.82, roll rate 2–3.5 rad/s
    at the end). It is not a smooth exponential: bursts of ~1 Hz rocking
    (joint p-p 0.2–0.4 rad in a 1 s window) separated by quiet seconds,
    then one burst tips it; the torso stays inside ~10° until the last
    half second, so a tilt guard cannot save it and a limp robot mid-sway
    falls anyway. The sensors are not the story this time: reported
    joint velocity vs finite-differenced angle has slope 0.69 (a unit
    error would be 50×; the dq quantum is exactly 0.0767 rad/s = one
    0.732 rpm LSB) and the gyro RMS matches the roll the up-vector shows.
    The same policy (loco_v22fix_e_s128), same zero command, in the SIL
    twin: up<sub>y</sub> RMS <b>0.0009</b> vs 0.05–0.13 on the robot,
    gyro<sub>x</sub> 0.005 vs 0.67 (144×), dq 0.02 vs 0.35–0.47. So the
    twin got plant knobs (<code>sil_twin --backlash-deg --act-lag-hz</code>)
    and a sweep ran: 0 / 0.7 / 3 / 6 / <b>10°</b> backlash × 0 / 2 Hz lag
    — every point stands dead still (up<sub>y</sub> RMS ≤ 0.0024, roll p-p
    ≤ 0.04 rad). The sim's "backlash" is a deadzone on the PD error, i.e. a
    servo that ignores small commands; the robot's play is the joint
    moving FREELY under load inside the slop — passive hysteresis in the
    only lateral actuator (no ankle roll on this body), which a command
    deadzone does not model at all. That is the gap Tom pointed at, now
    with numbers. Also different: the two plants settle in different
    postures (sim leans up<sub>x</sub> +0.13 with ankles −0.10; hardware
    up<sub>x</sub> −0.08…+0.03 with ankles +0.04…+0.07, L/R hip pitch
    −0.09/+0.10). Next, in order: (1) <b>measure the play</b> —
    <code>link/play_probe.py</code> is the torque-off pose stream, Tom
    wiggles each joint, per-joint p-p in degrees falls out; (2) one stand
    on a hard floor instead of the mat, same probe, to split surface
    compliance from joint slop; (3) model what we measure as plant
    hysteresis (free travel), not a command deadzone, and re-run this
    sweep until the twin rocks like the robot does — only then re-rank
    the fleet on it. Media + CSVs: hw_sessions/2026-09-02/osc{1,2,3}_*
    and sim_ref/.</p>
    <p><b>Later the same evening — the play, estimated without hands.</b>
    Tom stepped away, so the "wiggle each joint" measurement became a
    machine one: <code>tools/joint_sweep.py</code> steps ONE joint
    ±4° in 0.5° increments from the home stand (bench <code>move</code>,
    60 steps/s) while the other nine hold, logging servo position, load
    and IMU tilt; <code>sim/joint_sweep_sim.py</code> runs the identical
    sweep on the rigid plant; <code>tools/joint_sweep_compare.py</code>
    puts them side by side. Two things the sim saved me from
    misreading. (1) The STS3215 <b>load register is useless for static
    stiffness</b>: every joint reached every goal within 6 ticks with load
    never leaving its idle ±24–56 band — the geartrain carries static load
    with the motor idle. (2) A hip-roll sweep barely tilts the torso
    <i>even in sim</i> (0.7° per 4°), so a small roll response is not
    play. <b>Hysteresis is.</b> Sim traces retrace to 0.1°; on the robot
    <b>L hip roll held the torso 1.2° tilted through ~3° of command
    reversal</b> before letting go, R hip pitch ~1°, L ankle ~0.5°, the
    rest under 0.3° (R hip roll unresolved: 0.2° torso response, too
    small to read). That 3° in the roll chain is 3–6× the 0.5–1.0°
    backlash the sim trains against, and it sits in exactly the axis that
    rocks. Also measured: the stance is <b>asymmetric — weight on the
    right foot</b> (right-leg joints move the torso more than the rigid
    sim, left-leg joints less: L ankle +4° → 0.2° vs 2.4° sim), and the
    right ankle at only −2° tipped the whole robot 8.5° (tilt guard,
    homed cleanly) — under 2° of pitch margin on the loaded foot. Knees
    not swept. Ops notes for the next bench script: the serial CLI drops
    about half of all lines at any spacing (wait for the reply pattern and
    resend); two 30 fps webcams plus the serial bridge on one USB hub
    re-enumerated three times tonight — one 640×480 camera alone was
    fine; and one RESET SERVOS over the link reported success but moved
    nothing (positions verified by <code>scan</code>; the verdict means
    "written", not "moved" — <code>homeAll</code> wants a position
    readback). Data: hw_sessions/2026-09-02/play_sweep4.csv,
    play_compare.txt, sim_ref/sim_sweep.txt.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·06 · training thread</span></p>
    <p><b>The jump arrived on schedule — the from-scratch line now leads
    the fleet under the measured servo.</b> v27full_d's first full
    uninterrupted window (+70M → ~195M total) did what the lag-naive
    benchmark did in the same band: the lag scorecard went 16 → 16 →
    18 → <b>56/144</b> overnight — past v27tilt_b's 46 — with falls
    79 → 69 → 32 → <b>22%</b>, both stands 8/8 at ~1–3 cm drift, asym
    12%, 4.0 W, reward 715→963 and still climbing. Rhythm is forming on
    the same arc (76% alternation from 55%, stride CV 0.42, 103
    surviving strides). The policy that was born expecting the real
    servo is now the best walker we have judged against it, and it
    carries the speed-coupled clock besides. Tonight: its 24-round
    distill (the deploy artifact question) and v27full_e continue
    behind the hardware thread's queued v31home pair. Referee-side, the
    measured plant kept growing too: their 09·05 bench found <b>85 ms
    of pure servo dead time</b> (their <code>--act-delay-max</code> DR
    landed in env_mjx), so the referee now mirrors it —
    <code>act_delay_ticks</code> in walker_env (target ring served
    before the lag cascade, exact env_mjx order) and
    <code>--act-delay-ticks</code> in eval_precision, tests pinning
    delay order, ring reset and delay+lag composition. The column's
    PIN is deliberately unchanged (still 2 Hz lag, no delay): their
    step test (85 ms delay + 30 ms lag) and the stride fit (2 Hz
    cascade) partially explain the same physics, and stacking both
    without a joint fit would double-count — one bench-trace fit
    decides the combo before any scores move. Flagged to the hardware
    thread on t_856978e0.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·05 · training thread</span></p>
    <p><b>The warm-start law gets a refinement, and the from-scratch
    line keeps halving its falls.</b> Yesterday's prediction was that
    v28gyro (gyro-DR-only, warm from v27tilt_b) would land "well under
    46" if contract changes always collapse warm starts. It landed
    <b>38/144</b> under the measured servo — a dent, not a collapse. So
    the law refines: the damage scales with the SIZE of the
    obs-dynamics delta (full DR bundle 46→14; gyro-only 46→38; pure
    reward-weight changes still warm cleanly, e.g. v29zero's
    rest-is-home term). The fleet under lag now: v27tilt_b 46 &gt;
    its r24 student 41 &gt; v28gyro 38 &gt; v29zero_s128r24 <b>36</b>
    (a student outscoring its own 28/144 teacher — DAgger smoothing) &gt;
    v29zero 28 &gt; v27full_c 18. The from-scratch line's story stays
    the curve, not the scorecard: falls <b>79% → 69% → 32%</b> across
    three nights (~125M total), watts calm at 3.8, reward 798 peak —
    the scorecard creeps (16→16→18) exactly the way its lag-naive
    sibling's did before jumping in the 100–175M band. Tonight it gets
    a full window for the first time since birth (v27full_d, 70M);
    every night so far but one was a shared or truncated slot. The
    practical deploy picture is unchanged — v27tilt_b_s128r24 (41,
    stands 8/8) remains the strongest walking candidate on the shelf,
    the flashed v28crouch_s128r24 (14) holds the stand/limit-cycle
    duty, and the v29zero student (36, falls 30%) is now a legitimate
    third option that also carries the rest-is-home stance the
    hardware thread wants for the real robot's arm posture.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·04 · training thread</span></p>
    <p><b>v27full keeps its promise; v28crouch re-proves the warm-start
    law.</b> Night 2 of the from-scratch line (v27full_b, +38M → ~97M
    total; its 17:34 evening launch died silently in the shared-GPU
    window — third such death, all outside the clean 22:00 path — and
    the re-launch at 02:20 ran fine): reward 649 → <b>798, still
    climbing</b>, and under the measured servo the scorecard holds at
    16/144 while what's underneath improves — falls 79% → 69% and the
    <b>best stands in the fleet under lag</b> (stand_10s 8/8 at 1.0 cm
    drift, stand_off 8/8 at 3.5 cm; rhythm still unformed at 55%/CV
    0.43 but surviving 10× the strides of night 1). Same growth
    trajectory the v25full benchmark had at this age; the era plan
    holds — continue nightly, expect the scorecard to move after
    ~200M like its lag-naive sibling did. The hardware thread's
    <b>v28crouch</b> (their squat fix + measured backlash / zero-offset
    / gyro DR, warm from v27tilt_b) refereed <b>14/144 under lag vs its
    parent's 46</b>, wobbling curve (peak 781, ended 673), student
    likewise 14: new DR terms are obs-dynamics contract changes, and
    warm-starting them onto a trained policy has now regressed the line
    four times out of four (08·27 clock, 09·01 v26servo, 09·01 tilt,
    now v28crouch). Tonight's already-queued <b>v28gyro</b> repeats the
    move more gently (gyro DR only, on terms partly in the parent's
    contract) — a clean falsifiable test: if the law is right it lands
    well under 46 too, and the v28 features belong in the NEXT
    from-scratch birth's contract (v29 = v27full recipe + crouch fix +
    measured DR) rather than in warm continuations. Housekeeping: the
    42-distill + chained lag-referee CMD pattern ran unattended
    end-to-end for the first time (34 min, card written by 02:00).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·03 · training thread</span></p>
    <p><b>v27full is born, and it is healthy — the first policy raised
    entirely inside the measured servo.</b> Night 1 of the from-scratch
    synthesis (v25full recipe + act-lag 2–12 Hz + speed-clock ×1.25 from
    step zero) reached 59M steps before the 07:00 hard stop, reward
    −75 → 713 peak / 640 at the cut — and at the 35M mark it sits at 536
    vs the v25full benchmark's 507 at the same age: <b>carrying the lag
    contract costs nothing in growth rate</b>. Its column signature
    proves the adaptation: 0/144 with 126 W of thrash on the lag-less
    plant, 16/144 at a calm 3.3 W in its own lagged world (falls 79%,
    rhythm 60%/CV 0.58 — the usual infancy, v25full looked the same at
    this age; judge by curve, reel is an infant reel). From here the
    lag column is the only honest CPU judge for the v27 era.
    <b>v27full_b continues tonight, same contract.</b> Second result:
    the 24-round re-distill worked — <b>v27tilt_b_s128r24 = 41/144</b>
    under measured lag (12-round student 28, teacher 46; stand_off 8/8,
    push_gauntlet 4/8), so DAgger depth was the distillation bottleneck
    and the chained lag-referee CMD job pattern held. That student is
    now the best deployable candidate we own — 2.2× the flashed
    v26lag_s128's 19 — pending the real-robot A/B (kanban t_856978e0),
    which the hardware thread's evening finding makes more interesting:
    their roll-limit-cycle work says ~3° of passive hip-roll PLAY —
    free hysteresis, a third plant term nothing in sim models yet — is
    what actually topples real stands; their measure-then-model plan is
    the right order, and once the twin rocks like the robot the fleet
    re-ranks again. Housekeeping: fixed the eval_precision
    <code>speed_mae</code> KeyError that fall-heavy infants trigger
    (it's why the collect gave v27full only a SIL card).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·02 · training thread</span></p>
    <p><b>The referee got the measured servo — and the fleet ranking
    inverted.</b> Following the morning card's finding (below), the CPU
    referee now has the missing actuation dynamics: <code>walker_env</code>
    gained <code>act_lag_hz</code> (the same 3-stage cascade law env_mjx
    trains against, seeded and ordered identically — parity pinned by
    <code>tests/test_act_lag_referee.py</code>), and
    <code>eval_precision --act-lag-hz 2.0</code> writes a new
    <b>scorecard_lag</b> column at the bench-measured pole, leaving every
    historical column untouched (9cecd71). Re-refereeing the fleet
    against the servo we actually own (lag-less CPU → lag):
    <b>v27tilt_b 46 → 46/144</b> — the only policy whose score does not
    move, 4/18 scenarios clean, asym 10%; the flashed v26lag_s128
    84 → <b>19</b>; the sim champion v25full_c 96 → <b>16 with 77%
    falls</b> (the lag-naive from-scratch line is the most fragile of
    all — exactly why tonight's v27full carries act-lag from birth);
    v26lag 95 → 15; v26servo 10 → 14 (15% falls, 3.1 W, wobble 0.21 —
    the calmest policy in the fleet, but gait asym 81%: stable and
    barely locomoting, "specialized" confirmed). So this morning's
    "tilt generation regressed" verdict — mine — was the third casualty
    of the lag-less columns in two days: judged against the measured
    servo, <b>v27tilt_b is the best policy we have, by 2.4×</b>. Its
    s128 student keeps only part of that: <b>28/144</b> under lag
    (43% falls, stand_off 7/8) — still ahead of the flashed 19, but
    distillation is now the bottleneck (this student's val MAE 0.031 vs
    0.022 for the cleanest prior distill), so a 24-round re-distill
    with a CHAINED lag referee is queued ahead of tonight's main run.
    Two honest caveats. (1) The 2.0 Hz / 3-stage pin comes
    from one bench session; the real robot walked on 09·01 while this
    column scores the flashed policy 19/144 — referee scenarios are far
    harder than a walk-day stand-and-stride, so treat the column as a
    RANKING, not a prediction, until a real A/B (v27tilt_b_s128 vs
    v26lag_s128 on the robot) calibrates it. (2) Rhythm under lag is
    bad everywhere (v26lag 96%/CV 0.16 → 64%/0.30; v27tilt_b 56%/0.49)
    — natural movement at the measured pole is still unsolved; that is
    now the era's problem statement. Also closed: the "distill
    self-referee dies after save" mystery — nothing crashes;
    <code>distill_student.py</code> simply <i>ends</i> at "saved →" and
    no distill job ever chained a referee step (future distill CMD jobs
    get <code>&amp;&amp; eval_precision --sil</code>). Tonight: 38 =
    v27full retry (its 2-minute 09·01 death is still unexplained; exact
    flag set is smoke-testing on CPU) with 39 = v25full_d as the
    dead-birth backup.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·02 · hardware thread</span></p>
    <p><b>v27tilt_b: 46/144. The referee can't see the servo lag — and
    tilt-trained policies learned to hide inside it.</b> Yesterday's card
    named the world-z/gravity observability bug as the cause of
    v27tilt's 54/144. That bug was real and the fix (ff320ae) verified,
    but it was <i>not</i> the cause: <b>loco_v27tilt_b</b> (v26lag's
    exact recipe + corrected tilt, 60.6 M steps, 7.5 h, PPO eval
    1112→1336 peak, 1200 final — the best MJX curve in the lineage)
    refereed <b>46/144</b> with the identical signature: 122 W to stand
    still, 34 % falls, heading/goal 0/8. So I ran the policies in
    <i>their own</i> MJX env with a stand command (flat, no pushes, no
    fallen starts — <code>sim/mjx/stand_probe.py</code>): v27tilt_b
    stands at <b>11.9 W</b> flat and <b>11.0 W at 3° tilt</b>, zero
    falls in 6 seeds — indistinguishable from v26lag (11.7 W). The 122 W
    exists only in the CPU referee — and survives <code>--nominal</code>
    (112 W with every DR/noise/latency/backlash term off; v26lag 13.6 W).
    Same <code>torso_up</code> sensor in both engines, so not a
    convention flip. The discriminator: <b>act-lag OFF in MJX reproduces
    the referee</b> — v27tilt_b 11.9 → <b>96.4 W</b>, v26lag 11.7 → 16.1 W.
    The tilt-trained policy learned a control law that is smooth only
    <i>through</i> the 2–12 Hz lag cascade it trained with (fast, large
    target swings the filter turns into motion); the CPU referee has no
    actuation lag at all (<code>walker_env</code> — none), so it sees the
    raw swings. Students: the deployed <b>v26lag_s128</b> is calm both
    ways (10.7 / 12.5 W); <b>v27tilt_b_s128</b> is 10.5 W with lag and
    <b>62.9 W, 67 % falls</b> without — more fragile than its teacher.
    <b>No flash; v26lag_s128 stays.</b> Two conclusions. (1) The referee
    is now judging policies for a servo we measured does not exist:
    the STS3215 has ~1.7 Hz tracking bandwidth (tools/servo_frf.py), the
    CPU referee actuates instantly. Every act-lag policy (v26lag,
    v26servo, v27*) is scored against the wrong plant — v26servo's 10/144
    (09·01 card) is the same story. The referee needs the measured servo
    dynamics (act-lag, or the servo-profile model on the agent branch)
    before it can rank another lag-trained run; this is the "make the
    sim match reality" item, and it is on the referee side. (2) Tilt
    itself is not yet judged: the referee has no floor tilt either, and
    in MJX v26lag <i>already</i> stands 0/6 falls at 3°, so the desk lean
    may not be why the real robot topples. Also today: the distill
    self-referee died after "saved" again (third time); v27tilt_b_s128
    was probed, not refereed.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·01 · hardware thread</span></p>
    <p><b>Floor-tilt DR, first cut, was a push in disguise — and the
    squat has never been paid for.</b> Walk day on the hard-rubber mat:
    v26lag_s128 stood ~9 s and survived a stride (vs &lt;2 s the day
    before), then toppled slowly during plain stand — both attempts, on
    a desk the bubble level says leans ~3.5°. The firmware fall⇒disarm
    fired cleanly on the sidestep fall. Tom's call: randomize the floor
    angle ±3° in training. Implemented as a per-env gravity tilt
    (<code>--tilt-max 3</code>, 80ce2a7), which crashed at launch twice
    (<code>float()</code> under jit, then a tracer leak from the lazily
    built DR cache — d51400e). <b>loco_v27tilt</b> then trained clean:
    60.6 M steps, 7.2 h, PPO reward 1112→1176 — and refereed
    <b>54/144</b> against v26lag's 95, with <b>115 W spent standing
    still</b> (v26lag: 6.9 W) and every heading/goal scenario collapsed
    (turn_180 8→2, circle 8→1, goal_home 8→2). Root cause, verified in
    code: the tilt rotated <code>opt.gravity</code>, but the IMU
    up-vector (<code>framezaxis</code>), the upright/fall
    <code>up_z</code> and both CoM-over-foot kernels measure against
    <i>world z</i>. The policy trained against a constant
    <i>unobservable</i> lateral pull while being paid to stay
    world-vertical — a push, not a floor — and learned to brace. A real
    IMU reads gravity. Fixed (ff320ae): gravity-aligned frame from the
    per-env model applied to obs, up_z and the CoM projections, gated
    so every tilt-off recipe stays bit-identical; verified on v5body
    (world-vertical robot reads 3.00° under 3° tilt, gravity-plumb robot
    reads 0.02°, CoM shifts the expected 13.1 mm). <b>loco_v27tilt_b</b>
    is tonight's run: v26lag's exact recipe + corrected tilt, a
    one-variable A/B against 95/144. Separately, <b>why squat_reps is
    0/8 for every policy</b>: both v26lag and v27tilt show depth_err
    9.7 cm with zero falls — the robot never leaves standing height. The
    referee doubling the crouch command share (6→12 %) changed nothing
    because <code>cmd_moving</code> ignores the crouch channel: a
    commanded crouch is a <i>plain stand</i>, so <code>w_still</code>
    (joint velocity) and <code>w_stand_com</code> (2 cm CoM kernel)
    penalize the descent while <code>w_height</code> pays 0.03/step
    for tracking it. Ignoring the command is the rational policy — and
    matches what the real robot did when told to crouch. v28 needs a
    height-tracking kernel weighted like <code>alive</code> and a
    <code>w_still</code> gate that releases during the crouch
    transition; that is a reward fix, not more samples.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·09·01</span></p>
    <p><b>The act-lag experiment repeated our 08·27 negative — and
    points at v27.</b> The hardware thread's excellent servo work
    (measured: the STS3215 has a ~10 rad/s² acceleration cap — THE
    sim-real gap, real joints move at ~half sim speed) produced
    <code>--act-lag</code> DR and an overnight run, v26servo (act-lag
    2–12 Hz, warm from v22fix_e). It refereed at <b>10/144</b> with 26%
    falls and 81 W — failed under its <i>own</i> trained conditions.
    Same mechanism as 08·27: act-lag changes actuation dynamics — an
    obs-dynamics contract change that cannot be warm-started, doubly so
    onto a decayed-lineage policy. Kanban card filed advising against
    flashing its s128 student. The evidence from both threads composes
    into one move: <b>loco_v27full</b>, from scratch, the proven v25full
    recipe <i>plus act-lag 2–12 Hz from step zero</i> — a policy born
    expecting the real servo's sluggishness. Queued tonight behind the
    twice-bumped v22fix_b_s128 distill (still the best near-term
    hardware stopgap, projected ~80 vs the flashed 69). v25full (96/144,
    curve still climbing) pauses but stays the sim benchmark; if
    v27full tracks its growth curve while carrying act-lag, it becomes
    the deploy line and v25full retires to reference. Note: the SIL
    clock gate was corrected again (8002802) — SIL numbers from before
    08·31 need re-refereeing before deploy comparisons.</p>
  </div>

{watch_html}
  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·31</span></p>
    <p><b>New all-time best — and it's the from-scratch line.</b>
    v25full_c (~175M, night 4): <b>96/144</b>, beating every warm-start
    policy ever trained (previous best 94). Gait asym 8%, the lowest
    wobble on record (0.44), turn_180 8/8 at 6°, sidesteps 8/8 + 8/8,
    stand_off 7/8, best-ever push_gauntlet (3/8) — and the reward curve
    is <i>still climbing</i> (737 → 1086), so this isn't the ceiling.
    Rhythm is maturing on schedule: 88% alternation / CV 0.26, up from
    72% / 0.49 two nights ago (reel is watchable but not demo-grade
    yet). Remaining gaps: metronome (cadErr 42%, still unformed) and
    speed_ladder. Reconciliation with the hardware thread also landed
    this weekend: the robot runs distilled (128,128) students, the
    deployed one (v22fix_e_s128, 69/144) came from the decayed lineage
    tail, and under the fixed SIL clock gate the better teacher
    v22fix_b re-judges at <b>85/144</b> (stand_off 8/8, falls 4%).
    After two ad-hoc distill launches lost GPU races to ollama's vision
    reload, the night runner gained <code>CMD=</code> queue jobs — all
    GPU work rides wait_gpu now. Tonight: distill v22fix_b → b_s128
    first (the swap-in deploy candidate, projected ~80), then
    v25full_d.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·30</span></p>
    <p><b>The from-scratch line is growing fast.</b> v25full_b, ~105M
    total steps: 14 → <b>72/144</b> overnight, with the parts that took
    the old lineage weeks of diagnosis arriving on their own — stand_off
    <b>6/8</b> (the stand-com term was in the contract from birth, so a
    passively stable stand was simply learned), sidesteps 8/8 + 7/8,
    turn_180 4/8 at 14°. Reward curve still steep (434 → 895, episode
    length rising) — nowhere near plateau. The young-policy tells are
    all present and expected: watts high (24.6), rhythm still unformed
    (72% alternation, CV 0.49), metronome untracked (cadErr 25%) —
    clock discipline emerged late in the old lineage too. It keeps the
    whole window (v25full_c tonight, 70M). The comparison that matters
    is a few nights out: match the frozen candidates (v22fix_c 94/144;
    v24clockv rhythm CV 0.09) with one reproducible recipe instead of
    six generations of patches.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·29</span></p>
    <p><b>The entropy experiment settled it: warm-starting is done.</b>
    Tom's (fair) challenge to yesterday's from-scratch call produced a
    split A/B: continue the best warm line with entropy raised
    0.004 → 0.010 (if the decay was entropy collapse, this should
    revive it) vs. the first 35M of the from-scratch consolidation.
    Verdict: the entropy arm declined <i>further</i> — 73/144 (parent
    94), stand_off 1/8, rhythm still degraded (94%, CV 0.18), rewards
    plateaued. The exploitative-brittleness diagnosis stands and the
    cheap fix is ruled out; the v21→v22 lineage is formally retired,
    with v22fix_c and v24clockv frozen as the hardware candidates.
    Meanwhile the infant v25full looks exactly as a healthy from-scratch
    run should at 35M: reward −91 → 507 and climbing steeply, 14/144,
    falls 59% — a toddler, not a verdict. It gets the whole window from
    tonight (70M/night, same contract it was born with) until the curve
    flattens.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·28</span></p>
    <p><b>The lineages are exhausted — breaking the continuation loop.</b>
    Last night's repair arms both declined again: the clock lineage's
    <i>pure</i> continuation (identical contract) collapsed its own
    crown jewel (rhythm CV 0.09 → 0.36, alternation 86%), and doubling
    the schedule weight didn't arrest the fix-line's slide (CV 0.19,
    82/144). Across the last four arms the picture is uniform: training
    reward at all-time highs (1680–1748) while every behavioral metric
    degrades. That's not bad luck — it's the signature of deep-
    continuation decay: both lines are now 5–6 generations of
    warm-starts deep, the policies have gone exploitative and brittle,
    and further nights on these weights buy reward, not behavior.
    Tonight breaks the loop: <b>loco_v25full</b>, trained FROM SCRATCH
    with the full consolidated contract — every proven shaping term
    plus the speed-coupled clock at the servo-feasible ×1.25 baked in
    from step zero (the mid-lineage cap change that failed on 08·27
    gets its fair test the right way). One job, the whole night
    (70M budgeted, curfew checkpoints), continuation tomorrow if the
    curve says so. Meanwhile the hardware-validation candidates are
    frozen: <b>v22fix_c</b> for scenario breadth (94/144), and
    <b>v24clockv</b> for gait quality (100% alternation, CV 0.09, slow
    metronome solved) — both reels and weights ready whenever the robot
    is.</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·27</span></p>
    <p><b>Negative result, worth its price: don't change the clock
    contract under a trained policy.</b> Both ×1.25 arms regressed on
    every axis — 62 and 63/144, stand_off back to 0/8, turn_180 0/8,
    and the rhythm broke too (88% alternation, stride CV 0.23/0.33).
    Tellingly, <i>training reward rose</i> (1600–1800, the highest yet)
    while every measured skill fell: with the clock's speed-law changed
    underfoot, PPO re-optimized into a different optimum and forgot the
    scenario skills. Contrast with history: reward-<i>weight</i> changes
    (adding the schedule, heading 0.3, stand-com) have always
    warm-started cleanly. It's the observation-dynamics contract that
    must stay fixed within a lineage. The ×1.25 idea isn't dead — it
    would need training from scratch or a multi-night transition budget
    — but it's shelved. Tonight returns to the two proven parents, each
    with a contract-safe fix: v24clockv_b continues the rhythm-champion
    clock lineage at its native ×1.7 (its top end is servo-capped
    anyway; betting time recovers scenarios like it did for the v22fix
    line), and v22fix_d continues the scenario champion with the
    contact-schedule weight doubled to 1.0 to arrest its three-night
    rhythm slide (CV 0.06 → 0.20).</p>
  </div>

  <div class="card accent">
    <p style="margin:0 0 6px"><span class="tag">update · 2026·08·26</span></p>
    <p><b>The speed-clock A/B split — and exposed a quiet decay in the
    "winning" line.</b> By scorecard the plain control won again:
    v22fix_c posted <b>94/144 (10 clean)</b>, first-ever reversal passes
    (2/8), top speed 0.41 m/s. The speed-clock arm (v24clockv, 87/144)
    fixed exactly what it was built for — the slow metronome rung went
    from 30% cadence error to <b>2%</b> — but lost the top end
    (top_speed 0.25): the ×1.7 cap demands 2.55 Hz stepping, which needs
    ~4.4 rad/s hip swings, <i>over the measured 4.04 rad/s STS3215
    ceiling</i>. The cap was asking for physically impossible cadence.
    But the rhythm probes flip the verdict: v24clockv walks at <b>100%
    alternation, stride CV 0.09</b>, while the plain line has eroded
    three nights straight — CV 0.06 → 0.12 → <b>0.20</b>, alternation
    94% — buying scenario passes by drifting back toward the surge-stall
    gait we cured. For a <i>natural</i> walker the clock architecture is
    right; the cap was wrong. Tonight, both arms at a feasible ×1.25
    (~1.9 Hz max, inside the servo envelope): v24clock_b continues the
    clock lineage; v24clock_x grafts the clock onto the
    scenario-stronger v22fix_c. Lineage race decides who carries
    forward.</p>
  </div>

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
  to its gif (macOS Preview doesn't animate gifs); the embedded clips above are
  cut from those reels into <code>sim/runs/_clips/</code> by
  <code>sim/build_report.py</code> — keep the folder beside the page when you
  copy it. Reproduce the headline:
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
