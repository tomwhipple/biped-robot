"""Verification of the round-3 flat-body ("bird3_round") get-up path found by
sim/getup_v6_side.py bird3search (docs/design-v6/getup_search_bird3_hillclimb_
round_folded.txt, BEST): is it quasi-static, is it robust, and does its second
half stand the body up from PRONE too.

    .venv/bin/python sim/getup_v6_bird3_verify.py           > docs/design-v6/getup_search_bird3_verify.txt
    .venv/bin/python sim/getup_v6_bird3_verify.py render sim/renders/getup_options/side/bird3_prone_stand.mp4
    .venv/bin/python sim/getup_v6_bird3_verify.py render-slow sim/renders/getup_options/side/bird3_supine_slow.mp4

The path (6 symmetric keyframes: hip_yaw, hip_roll(L; R mirrored), knee,
ankle in deg) is copied verbatim from the search log so this file is the
single place the numbers live.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import getup_v6 as G          # noqa: E402
import getup_v6_side as S     # noqa: E402

NAME = "bird3_round"
PATH = [[77.13171266031603, 85.80916880672048, 95.0, 6.718777587798474],
        [-12.588201823624004, 2.636126282676855, -39.85451110995246, 17.094222192742624],
        [-51.807028634480716, 66.7437759474274, 9.338370086349668, -21.17056084238613],
        [149.17511584073162, 120.0, 12.610792419052698, -20.839105384406935],
        [66.33236371567602, 73.91423960332551, -32.45948088457739, -27.484442225800553],
        [8.30643538070662, 0.0, -8.488363027989752, -6.5227224243693005]]


def q_sym(yaw, roll, knee, ankle):
    return S._bird_legs(L_roll=roll, R_roll=-roll, L_hip_yaw=yaw, R_hip_yaw=yaw, knee=knee, ankle=ankle)


def seq(start_pose, first=0, move=1.4, hold=0.6, end_hold=2.0):
    s = [("lie", start_pose, 0.5, 1.0)]
    s += [(f"k{j}", q_sym(*row), move, hold) for j, row in enumerate(PATH) if j >= first]
    s.append(("hold", q_sym(*PATH[-1]), 0.1, end_hold))
    return s


def run(label, p, xml, sq, start, play=3.0, mu=0.7, sc=1.0, verbose=False, render=None, cam=(1.3, -15, 135)):
    cap = (lambda lab, t: f"{NAME}  {start}   {lab}   t={t:4.1f}s") if render else None
    r = G.run_sequence(p, xml, sq, start=start, play_deg=play, per_joint=G.PJ_DEFAULT, mu=mu, servo_scale=sc,
                       verbose=verbose, render=render, cam=cam, size=(540, 720), label_fn=cap)
    pk = max(L[4] for L in r["log"])
    print(f"{label:52s} {'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} z {r['pelvis_z']:.3f} |tau|max {pk:.2f} N-m", flush=True)
    return r


def main():
    p, xml = S.plant(NAME)
    if len(sys.argv) > 1 and sys.argv[1] == "render":
        run("prone k3..k5 render", p, xml, seq(S.BIRD_PRONE_FOLD, first=3), "bird_flat", verbose=True,
            render=sys.argv[2], cam=(1.4, -14, 90))
        return
    if len(sys.argv) > 1 and sys.argv[1] == "render-slow":
        run("supine slow (4 s moves, 2 s holds) render", p, xml, seq(S.BIRD_SUPINE_FOLD, move=4.0, hold=2.0), "bird_back",
            verbose=True, render=sys.argv[2], cam=(1.4, -14, 90))
        return
    print(f"== bird3_round get-up path verification (self_collide=True, deploy model, STS3250 rolls+knees); path from "
          f"getup_search_bird3_hillclimb_round_folded.txt BEST")
    print("-- 1. as searched (1.4 s moves, 0.6 s holds) + 2 s final hold, per-keyframe trace")
    run("supine, search timing", p, xml, seq(S.BIRD_SUPINE_FOLD), "bird_back", verbose=True)
    print("-- 2. quasi-static check: 4 s moves, 2 s holds (3x slower)")
    run("supine, 3x slower", p, xml, seq(S.BIRD_SUPINE_FOLD, move=4.0, hold=2.0), "bird_back", verbose=True)
    print("-- 3. robustness (play deg, friction, servo strength)")
    for play, mu, sc in [(3, 0.7, 1.0), (5, 0.7, 1.0), (3, 0.3, 1.0), (3, 1.0, 1.0), (3, 0.7, 0.8), (3, 0.7, 0.65), (5, 0.3, 0.65)]:
        run(f"supine play {play} mu {mu} servo {sc:.2f}", p, xml, seq(S.BIRD_SUPINE_FOLD), "bird_back", play, mu, sc)
    print("-- 4. PRONE start (bird_flat, BIRD_PRONE_FOLD): second half of the path, k3..k5")
    run("prone k3..k5 nominal", p, xml, seq(S.BIRD_PRONE_FOLD, first=3), "bird_flat", verbose=True)
    for play, mu, sc in [(5, 0.7, 1.0), (3, 0.3, 1.0), (3, 1.0, 1.0), (3, 0.7, 0.65), (5, 0.3, 0.65)]:
        run(f"prone k3..k5 play {play} mu {mu} servo {sc:.2f}", p, xml, seq(S.BIRD_PRONE_FOLD, first=3), "bird_flat", play, mu, sc)
    print("-- 5. PRONE start, the full path k0..k5 (robustness to running the whole thing regardless of start)")
    run("prone full path nominal", p, xml, seq(S.BIRD_PRONE_FOLD), "bird_flat")


if __name__ == "__main__":
    main()
