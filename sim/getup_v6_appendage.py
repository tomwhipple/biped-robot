"""Get-up study, round 2 (2026-09-14, Tom: "we have extra servos so we could
add some proto-arms. Or maybe a kangaroo / t-rex like tail?"): which appendage
gets the CoM from the seat onto the feet on the 130/125 v6 body.

The seated failure (design doc 11.3): hips 4 cm off the floor, torso vertical
at full hip flexion, CoM 8 cm behind the feet. Any push that lifts the pelvis
to ~10 cm while the feet stay planted lets the shank lean forward (ankle -40)
and puts the torso CoM over the toes -- so the candidates are things that push
on the floor behind the hips: two low-mounted arms (housing sides, hip level)
or one tail on the centreline. Shoulder-height arms were already negative
(they pivot the body over the head, getup_search_arms*.txt).

    .venv/bin/python sim/getup_v6_appendage.py seat  > docs/design-v6/getup_search_appendage_seat.txt
    .venv/bin/python sim/getup_v6_appendage.py prone > docs/design-v6/getup_search_appendage_prone.txt
    .venv/bin/python sim/getup_v6_appendage.py render <config> <variant> out.mp4
"""
import sys, os, itertools, dataclasses
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G

SP = os.environ.get("TMPDIR", "/tmp")
K, H = -130.0, -125.0
BASE = DesignParams()          # knee 130 / hip 125 defaults

CONFIGS = {
    # arms: 1 DOF at the housing sides; arm_z = root height above the yaw axis
    # (0 = housing bottom, -0.06 = beside the hip roll servos, -0.09 = hip pitch level)
    "arms_z0_20": dict(arms=True, arm_z=0.0, arm_len=0.20),
    "arms_hip_15": dict(arms=True, arm_z=-0.06, arm_len=0.15),
    "arms_hip_20": dict(arms=True, arm_z=-0.06, arm_len=0.20),
    "arms_hip_25": dict(arms=True, arm_z=-0.06, arm_len=0.25),
    "arms_hip9_20": dict(arms=True, arm_z=-0.09, arm_len=0.20),
    "arms_hip_elbow": dict(arms=True, arm_z=-0.06, arm_len=0.12, arm_elbow=True, arm_fore_len=0.12),
    # tail: root on the housing rear; tail_z = root height above the yaw axis
    "tail_z0_20": dict(tail=True, tail_len=0.20),
    "tail_hip_15": dict(tail=True, tail_len=0.15, tail_z=-0.06),
    "tail_hip_20": dict(tail=True, tail_len=0.20, tail_z=-0.06),
    "tail_hip_25": dict(tail=True, tail_len=0.25, tail_z=-0.06),
    "arms_hip_20+tail_hip_20": dict(arms=True, arm_z=-0.06, arm_len=0.20, tail=True, tail_len=0.20, tail_z=-0.06),
}
hits = []


def plant(name):
    p = dataclasses.replace(BASE, **CONFIGS[name])
    xml = os.path.join(SP, f"gu_app_{name}_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    return p, xml


def run(label, p, xml, seq, start="supine", render=None):
    r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False, render=render)
    trail = " | ".join(f"{L[0][:11]}:{L[1]:+.2f}/{L[2]:.2f}/{'+'.join(c.replace('_','')[:5] for c in L[3])}" for L in r['log'][2:])
    print(f"{label:64s} {'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} z {r['pelvis_z']:.3f}\n      {trail}", flush=True)
    if r['ok']:
        hits.append(label)
    return r


# ------------------------------------------------------------------ sequences
def seat_push(app, s0, s1, t_push, a_push, elbow=None):
    """supine -> sit up (appendage folded ALONG the torso so it cannot jam the
    sit-up: shoulder 180 = arm up along the body, tail +60 = tip toward the
    head) -> appendage swung down BEHIND the back onto the floor (a brace)
    -> tuck the feet under -> push the pelvis up while the shank leans
    forward -> rise. app = 'shoulder' | 'tail'; elbow = (e0, e1) for the
    2-DOF arm."""
    fold = {app: 180 if app == "shoulder" else 60}
    behind = {app: s0}
    push = {app: s1}
    rest = {app: 60 if app == "shoulder" else -20}      # out of the way behind
    if elbow:
        fold["elbow"] = 0; behind["elbow"] = elbow[0]; push["elbow"] = elbow[1]; rest["elbow"] = 0
    return [("lie", dict(**fold), 0.5, 0.8),
            ("sit up", dict(hip_pitch=-90, **fold), 1.5, 0.8),
            ("fold", dict(hip_pitch=-110, **fold), 1.0, 0.6),
            ("brace", dict(hip_pitch=-110, **behind), 1.0, 0.8),
            ("tuck", dict(hip_pitch=H, knee=K, ankle=0, **behind), 1.5, 0.8),
            ("push", dict(hip_pitch=H, knee=K, ankle=a_push, **push), t_push, 1.0),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, **push), 1.5, 1.0),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, **rest), 1.5, 1.0),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **rest), 1.5, 1.0),
            ("straight", dict(**rest), 1.0, 0.8)]


def prone_roll(side, s_push, kick):
    """prone -> one low arm pushes the floor to roll the body onto its back."""
    o = "R" if side == "L" else "L"
    seq = [("lie prone", dict(), 0.5, 0.8),
           ("arm to floor", {f"{side}_shoulder": -60}, 0.8, 0.4),
           ("push + kick", {f"{side}_shoulder": s_push, f"{o}_hip_roll": (kick if o == "L" else -kick), f"{side}_hip_roll": 0}, 1.2, 1.0),
           ("legs back", {f"{side}_shoulder": s_push}, 1.0, 1.0),
           ("arms clear", dict(shoulder=0), 1.0, 1.0)]
    return seq


def main():
    mode = sys.argv[1]
    if mode == "seat":
        print("== SEAT PUSH on the 130/125 body: supine -> sit up (appendage folded along the torso) -> appendage down behind onto the floor (brace) -> tuck (knee -130, hip -125) -> push while the ankle dorsiflexes -> rise")
        for name in CONFIGS:
            p, xml = plant(name)
            app = "tail" if name.startswith("tail") else "shoulder"
            extra = 0.055 * (2 if p.arms else 0) * (2 if p.arm_elbow else 1) + (0.055 if p.tail else 0) + (0.05 if p.arms else 0) + (0.04 if p.tail else 0)
            print(f"-- {name}: {CONFIGS[name]}  extra mass ~{extra:.3f} kg", flush=True)
            if p.arm_elbow:
                for (s0, e0), (s1, e1), a_push in itertools.product(((60, -60), (90, -90)), ((20, -20), (0, 0), (30, 0), (10, 30)), (-40, -25)):
                    run(f"{name:22s} sh {s0:+4d}->{s1:+4d} el {e0:+4d}->{e1:+4d} ankle {a_push}", p, xml, seat_push("shoulder", s0, s1, 2.0, a_push, elbow=(e0, e1)))
                continue
            if app == "tail":
                grid = itertools.product((-20, -40), (-70, -90, -110), (2.0,), (-40, -25))
            else:
                grid = itertools.product((70, 50), (20, 0, -20), (2.0,), (-40, -25))
            for s0, s1, t_push, a_push in grid:
                run(f"{name:22s} {app} {s0:+4d}->{s1:+4d} push {t_push:.0f}s ankle {a_push}", p, xml, seat_push(app, s0, s1, t_push, a_push))
            if "+" in name:
                for s1, t1, a in itertools.product((-90, -110), (0, 20), (-40,)):
                    seq = seat_push("tail", -30, s1, 2.0, a)
                    for st in seq:
                        st[1].update(shoulder={"lie": 180, "sit up": 180, "fold": 180, "brace": 60, "tuck": 60,
                                               "push": t1, "rise 2": t1}.get(st[0], 60))
                    run(f"{name:22s} tail -30->{s1} + arms 60->{t1} ankle {a}", p, xml, seq)
    elif mode == "prone":
        print("== PRONE ROLL with one low arm: prone -> arm to the floor -> push (+ leg kick) -> onto the back?")
        for name in ("arms_low_15", "arms_low_20", "arms_low_25", "arms_mid_20"):
            p, xml = plant(name)
            for s_push, kick in itertools.product((-90, -120, -150), (0, 30)):
                r = run(f"{name:20s} L arm -60->{s_push} kick {kick}", p, xml, prone_roll("L", s_push, kick), start="prone")
    elif mode == "robust":
        print("== ROBUSTNESS of the winners: play, friction, servo strength (nominal = play 3, mu 0.7, servo 1.0)")
        winners = [("arms_hip_20", "shoulder", (70, 0, 2.0, -40), None), ("arms_hip_25", "shoulder", (70, 0, 2.0, -40), None),
                   ("arms_hip9_20", "shoulder", (70, 0, 2.0, -40), None), ("arms_hip_elbow", "shoulder", (60, 0, 2.0, -40), (-60, 0)),
                   ("tail_hip_20", "tail", (-20, -90, 2.0, -40), None), ("tail_hip_25", "tail", (-20, -90, 2.0, -40), None)]
        conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0), dict(play_deg=3.0, mu=0.3, servo_scale=1.0),
                 dict(play_deg=3.0, mu=1.0, servo_scale=1.0), dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
        for name, app, (s0, s1, t, a), el in winners:
            p, xml = plant(name)
            seq = seat_push(app, s0, s1, t, a, elbow=el)
            for c in conds:
                r = G.run_sequence(p, xml, seq, start="supine", per_joint=G.PJ_DEFAULT, verbose=False, **c)
                worst = max(r["log"], key=lambda L: L[4])
                print(f"{name:16s} {app} {s0:+4d}->{s1:+4d}  play {c['play_deg']:.0f} mu {c['mu']:.1f} servo {c['servo_scale']:.2f}  "
                      f"{'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} z {r['pelvis_z']:.3f}  |tau|max {worst[4]:.2f} Nm ({worst[5]} @ {worst[0]})", flush=True)
                if r["ok"]: hits.append(name)
    elif mode == "prone2":
        print("== PRONE with hip-level arms: (R) roll onto the back with one arm; (K) push-up -> knees under -> kneel-sit -> hands forward -> toes under -> bear -> squat -> hands off -> rise")
        for name in ("arms_hip_20", "arms_hip_25", "arms_hip_elbow"):
            p, xml = plant(name)
            el = p.arm_elbow
            for s_push, kick in itertools.product((-90, -120), (0, 30)):
                seq = prone_roll("L", s_push, kick)
                if el:
                    for st in seq: st[1].update(elbow=0)
                run(f"{name:16s} R: L arm -60->{s_push} kick {kick}", p, xml, seq, start="prone")
            for sh_bear, k_bear, sh_sq in itertools.product((-60, -80), (-90, -70), (-20, 0)):
                e = dict(elbow=0) if el else {}
                seq = [("lie prone", dict(shoulder=-90, **e), 0.5, 0.8),
                       ("knees under", dict(hip_pitch=H, knee=K, ankle=40, shoulder=-90, **e), 2.0, 1.0),
                       ("kneel-sit", dict(hip_pitch=-60, knee=K, ankle=40, shoulder=-45, **e), 2.0, 1.0),
                       ("toes under", dict(hip_pitch=-60, knee=K, ankle=-40, shoulder=-45, **e), 1.5, 0.8),
                       ("bear", dict(hip_pitch=H, knee=k_bear, ankle=-40, shoulder=sh_bear, **e), 2.0, 1.0),
                       ("squat", dict(hip_pitch=H, knee=K, ankle=-40, shoulder=sh_sq, **e), 1.5, 1.0),
                       ("hands off", dict(hip_pitch=H, knee=K, ankle=-40, shoulder=60, **e), 1.0, 1.0),
                       ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, shoulder=60, **e), 1.5, 1.0),
                       ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, shoulder=60, **e), 1.5, 1.0),
                       ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, shoulder=60, **e), 1.5, 1.0), ("straight", dict(shoulder=60, **e), 1.0, 0.8)]
                run(f"{name:16s} K: bear sh {sh_bear} knee {k_bear}, squat sh {sh_sq}", p, xml, seq, start="prone")
    elif mode == "prone3":
        print("== PRONE with hip-level arms, round 3: (K2) child's pose FIRST (arms folded), then hands forward as a tripod -> kneel-sit -> toes under -> bear -> squat -> hands off -> rise; (R2) slow one-arm roll with a big leg swing")
        for name in ("arms_hip_20", "arms_hip_25"):
            p, xml = plant(name)
            for sh_f, hip_k, k_bear, sh_bear in itertools.product((-40, -60), (-60, -80), (-90, -70), (-60, -80)):
                seq = [("lie prone", dict(shoulder=180), 0.5, 0.8),
                       ("child's pose", dict(hip_pitch=H, knee=K, ankle=40, shoulder=180), 2.0, 1.0),
                       ("hands front", dict(hip_pitch=H, knee=K, ankle=40, shoulder=sh_f), 1.5, 0.8),
                       ("kneel-sit", dict(hip_pitch=hip_k, knee=K, ankle=40, shoulder=sh_f), 2.0, 1.0),
                       ("toes under", dict(hip_pitch=hip_k, knee=K, ankle=-40, shoulder=sh_f), 1.5, 0.8),
                       ("bear", dict(hip_pitch=H, knee=k_bear, ankle=-40, shoulder=sh_bear), 2.0, 1.0),
                       ("squat", dict(hip_pitch=H, knee=K, ankle=-40, shoulder=-20), 1.5, 1.0),
                       ("hands off", dict(hip_pitch=H, knee=K, ankle=-40, shoulder=60), 1.0, 1.0),
                       ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, shoulder=60), 1.5, 1.0),
                       ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, shoulder=60), 1.5, 1.0),
                       ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, shoulder=60), 1.5, 1.0), ("straight", dict(shoulder=60), 1.0, 0.8)]
                run(f"{name:12s} K2: hands {sh_f} kneel hip {hip_k} bear knee {k_bear} sh {sh_bear}", p, xml, seq, start="prone")
            for s_push, kick, t in itertools.product((-100, -130), (45, 60), (2.5,)):
                seq = [("lie prone", dict(shoulder=180), 0.5, 0.8),
                       ("L arm to floor", dict(L_shoulder=-60, R_shoulder=180), 0.8, 0.4),
                       ("push + swing R leg across", dict(L_shoulder=s_push, R_shoulder=180, R_hip_roll=-kick, R_hip_pitch=-60, R_knee=-60), t, 1.0),
                       ("legs back", dict(L_shoulder=s_push, R_shoulder=180), 1.0, 1.0),
                       ("arms fold", dict(shoulder=180), 1.0, 1.0)]
                run(f"{name:12s} R2: L arm -60->{s_push} R leg kick {kick} + swing", p, xml, seq, start="prone")
    elif mode == "render":
        name, variant, out = sys.argv[2], sys.argv[3], sys.argv[4]
        p, xml = plant(name)
        app = "tail" if name.startswith("tail") else "shoulder"
        v = [float(x) for x in variant.split(",")]
        s0, s1, t_push, a_push = v[:4]
        run(f"{name} {variant}", p, xml, seat_push(app, s0, s1, t_push, a_push, elbow=(v[4], v[5]) if len(v) > 4 else None), render=out)
        os.system(f"ffmpeg -loglevel error -y -i {out} -vf 'fps=0.8,scale=320:-1,tile=10x1' -frames:v 1 {out[:-4]}_strip.png")
        return
    print("standing:", hits)


if __name__ == "__main__":
    main()
