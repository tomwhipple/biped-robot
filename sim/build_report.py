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
