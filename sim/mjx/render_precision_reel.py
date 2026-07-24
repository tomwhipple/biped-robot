"""Render the precision-skill montage: one continuous video, every maneuver.

Replays each eval_precision scenario with the trained policy on the CPU
referee env and stitches the takes into a single H.264 .mov (QuickTime-
friendly, like tools/gif2mov.py output). Per scenario it prefers the first
SUCCESSFUL seed (probing up to --seeds without rendering, which is cheap);
if none succeeds it falls back to the first non-falling seed, else seed 0 --
and the caption says PASS or FAIL honestly either way (per the video-review
rule: label takes, don't cherry-pick silently).

Output: sim/renders/precision_reel_<run>.mov (gitignored; regenerate here).

Run:  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/render_precision_reel.py \
          --run-name precision_v2 [--seeds 6] [--scenarios a,b] [--nominal]
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import eval_precision as ep

RENDERS = os.path.join(HERE, "..", "renders")

# human titles for the caption bar, keyed by scenario name
TITLES = {
    "balance_L": "Balance on LEFT leg - 10 s",
    "balance_R": "Balance on RIGHT leg - 10 s",
    "circle_air_L": "Air circles - LEFT foot",
    "circle_air_R": "Air circles - RIGHT foot",
    "line_1m": "Walk a straight 1 m line + stop",
    "backward_1m": "Walk 1 m BACKWARD + stop",
    "sidestep_L": "Sidestep 0.5 m LEFT",
    "sidestep_R": "Sidestep 0.5 m RIGHT",
    "square_return": "Walk a 1 m square, return to start",
    "circle_return": "Walk a circle, return to start",
    "crouch_hold": "Crouch to 70% height + recover",
    "march_in_place": "March in place (knee articulation)",
    "hip_sway": "Lateral sway (hip articulation)",
    "recover_sit": "Stand up from sitting",
    "recover_fallen": "Recover from a fall + stand up",
    "stand_10s": "Stand still - 10 s",
    "stand_off": "Stand still, servos OFF (torque released)",
}

BAR_H = 44          # caption bar height (px)
TITLE_S = 1.2       # seconds of title card before each take


def _font(size):
    for cand in ("/System/Library/Fonts/Helvetica.ttc",
                 "/System/Library/Fonts/Supplemental/Arial.ttf"):
        if os.path.exists(cand):
            return ImageFont.truetype(cand, size)
    return ImageFont.load_default()


def caption(frame, title, verdict, headline, t, total_t):
    """Frame + top caption bar (title, PASS/FAIL, headline, timecode)."""
    h, w = frame.shape[:2]
    img = Image.new("RGB", (w, h + BAR_H), (16, 16, 20))
    img.paste(Image.fromarray(frame), (0, BAR_H))
    d = ImageDraw.Draw(img)
    d.text((10, 7), title, font=_font(19), fill=(235, 235, 235))
    color = (90, 210, 120) if verdict == "PASS" else (235, 110, 90)
    d.text((w - 200, 7), verdict, font=_font(19), fill=color)
    d.text((w - 140, 7), headline, font=_font(15), fill=(180, 180, 180))
    d.text((10, BAR_H + 6), f"{t:4.1f}s", font=_font(14), fill=(200, 200, 200))
    return np.asarray(img)


def title_card(w, h, title, idx, n):
    img = Image.new("RGB", (w, h + BAR_H), (16, 16, 20))
    d = ImageDraw.Draw(img)
    d.text((w // 2, (h + BAR_H) // 2 - 20), title, font=_font(30),
           fill=(235, 235, 235), anchor="mm")
    d.text((w // 2, (h + BAR_H) // 2 + 22), f"{idx}/{n}", font=_font(18),
           fill=(150, 150, 150), anchor="mm")
    return np.asarray(img)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--xml", default=None)
    p.add_argument("--seeds", type=int, default=6,
                   help="max seeds probed per scenario for a passing take")
    p.add_argument("--nominal", action="store_true")
    p.add_argument("--scenarios", default=None)
    p.add_argument("--fps", type=float, default=50 / 3,
                   help="output fps (Driver records every 3rd 50 Hz step -> "
                        "16.67 fps is real-time)")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    run_dir = os.path.join(ep.RUNS, args.run_name)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = args.xml or cfg.get("xml_path") or ep.DEFAULT_XML
    if not os.path.isabs(xml) and not os.path.exists(xml):
        xml = os.path.join(HERE, "..", os.path.basename(xml))

    reg = ep._registry()
    names = ep.ORDER
    fam = (cfg.get("train", {}) or {}).get("family", "all")
    if not args.scenarios and fam in ep.FAMILY_SCENARIOS:
        names = [n for n in ep.ORDER if n in ep.FAMILY_SCENARIOS[fam]]
    if args.scenarios:
        want = [s.strip() for s in args.scenarios.split(",")]
        names = [n for n in ep.ORDER if n in want]

    env_cache = {}

    def get_env(secs, name=""):
        key = (secs, name if name in ep.ENV_EXTRA else "")
        if key not in env_cache:
            env_cache[key] = ep.make_env(cfg, secs, args.nominal, xml,
                                         extra=ep.ENV_EXTRA.get(key[1]))
        return env_cache[key]

    obs_size = get_env(12.0).observation_space.shape[0]
    act_size = get_env(12.0).action_space.shape[0]
    mass = float(get_env(12.0).model.body_mass.sum())
    N = float(get_env(12.0)._nominal_h)
    act = ep.load_policy(run_dir, obs_size, act_size)

    os.makedirs(RENDERS, exist_ok=True)
    out = args.out or os.path.join(
        RENDERS, f"precision_reel_{args.run_name}.mov")
    import imageio
    wr = imageio.get_writer(out, format="FFMPEG", fps=args.fps,
                            codec="libx264", pixelformat="yuv420p",
                            output_params=["-preset", "medium", "-crf", "23"])

    passes = 0
    for k, name in enumerate(names):
        secs, factory, _ = reg[name]
        env = get_env(secs, name)
        # probe for a passing seed (no rendering: cheap), fallback non-fall
        chosen, fallback = None, None
        for i in range(args.seeds):
            res = ep.run_one(env, act, factory, seed=100 * i + 7,
                             record=False, N=N, mass=mass)
            if res["success"]:
                chosen = i
                break
            if fallback is None and not res["fell"]:
                fallback = i
        seed_i = chosen if chosen is not None else (
            fallback if fallback is not None else 0)
        res = ep.run_one(env, act, factory, seed=100 * seed_i + 7,
                         record=True, N=N, mass=mass)
        verdict = "PASS" if res["success"] else "FAIL"
        passes += res["success"]
        headline = res.get("headline", "")
        frames = res["frames"]
        title = TITLES.get(name, name)
        print(f"[{k+1:2d}/{len(names)}] {name:14s} seed {seed_i} {verdict} "
              f"{headline}  ({len(frames)} frames)", flush=True)
        h, w = frames[0].shape[:2] if frames else (480, 640)
        card = title_card(w, h, title, k + 1, len(names))
        for _ in range(int(TITLE_S * args.fps)):
            wr.append_data(card)
        for j, fr in enumerate(frames):
            t = j * 3 / 50.0          # Driver records every 3rd 50 Hz step
            wr.append_data(caption(fr, title, verdict, headline, t, secs))
    wr.close()
    print(f"\n{passes}/{len(names)} scenarios shown as PASS")
    print(f"saved -> {out}")


if __name__ == "__main__":
    main()
