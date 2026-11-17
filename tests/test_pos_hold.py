"""--pos-hold (experiments/roll-balance/pos_hold.py, cmd_walk.py, track_host.py): the position hold from the overhead
camera tracker. The sign conventions through the vision tools' own synthetic renderer (the bug class that hurt on
2026-11-16: left/right as seen from the Sony vs the webcam image vs the robot's own left), the phase-consistent
sample, the closed loop on a fake robot with an injected lateral creep, tracker dropouts, the edge rule, and that
the walk is unchanged with the hold off."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.join(HERE, "..", "experiments", "roll-balance")
VIS = os.path.join(HERE, "..", "tools", "vision")
sys.path.insert(0, RB)
sys.path.insert(0, VIS)
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")
C = pytest.importorskip("cmd_walk")
PH = pytest.importorskip("pos_hold")
import vgeom as G  # noqa: E402
import vsynth as synth  # noqa: E402

BASE = dict(T_step=4.0, seeds=H.ROBOT_SEED, lead_ms=2500, cal=None, amp=H.ROBOT_AMP, model="dwell", ds=None, h=None,
            pelvis_x="stance", ankle_ff=(2.0, 2.0), k_ds=None, drop=None, lift_gate=True)
T = 4.0


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


# ---- scenes ------------------------------------------------------------------------------------------------------

# the real eMeet: 1920x1080, from the far end of the board, looking toward the Sony end (its image is mirrored
# left-right relative to the board x); measured-style layout with the board-end tags 2 and 3 and a known f
ESIZE, EF = (1920, 1080), 1000.0
EK = G.make_K(EF, ESIZE)
E_R, E_T = synth.look_at([400, 1100, 650], [400, 500, 0])
ETAGS = {"2": {"center": [98.0, 0.0], "size": 80.0}, "3": {"center": [802.0, 0.0], "size": 80.0}}


def ecfg():
    cfg = G.load_config()
    cfg["board"].update(layout="config", tags=ETAGS, width_mm=812.8, length_mm=1219.2)
    cfg["camera"]["f_px"] = EF
    return cfg


def erender(cfg, hd, x, y, z=170.0, seed=0):
    tags = synth.scene_tags(cfg, G.board_R_from_euler(hd, 0, 0), [x, y, z], board_ids=[2, 3])
    return synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=seed, floor=170)


# the Sony view (low, at the near end, looking along +y): board x is to the camera's right
SSIZE, SF = (1024, 680), 870.0
SK = G.make_K(SF, SSIZE)
S_R, S_T = synth.look_at([450, -500, 475], [450, 400, 150])


def scfg():
    cfg = G.load_config()
    cfg["board"].update(layout="config", width_mm=900.0, length_mm=1200.0)
    cfg["robot"]["tags"] = {
        "10": {"center": [30.0, 0.0, 0.0], "normal": [1, 0, 0], "up": [0, 0, 1], "size": 60.0},
        "11": {"center": [-30.0, 0.0, 0.0], "normal": [-1, 0, 0], "up": [0, 0, 1], "size": 60.0},
        "12": {"center": [0.0, 0.0, 45.0], "normal": [0, 0, 1], "up": [1, 0, 0], "size": 60.0},
    }
    return cfg


def srender(cfg, hd, x, y, z=170.0, seed=0):
    tags = synth.scene_tags(cfg, G.board_R_from_euler(hd, 0, 0), [x, y, z])
    return synth.render(tags, SK, S_R, S_T, SSIZE, board_rect=(900, 1200), seed=seed, blur=1.0, noise=4)


SCENES = {"emeet": (ecfg, erender), "sony": (scfg, srender)}


def sample(cfg, render, hd, x, y, loc, n=5, t0=1000.0):
    """n frames of the scene fed through the locator into a sampler; the median fix."""
    sam = PH.PosSampler()
    for k in range(n):
        r = loc(render(cfg, hd, x, y, seed=k))
        assert r is not None, loc.why
        sam.add(t0 + k, r["x"], r["y"], r["h"])
    f = sam.fix(t0 + n, n, 3)
    assert f is not None
    return f


def new_hold(f_clock, sam, **kw):
    return PH.PosHold(sam, T, say=lambda *a, **k: None, clock=f_clock, **kw)


# ---- 3. sign conventions through the real tracker ---------------------------------------------------------------------

# the Sony scene's low view decodes the robot's pose unambiguously only from the side (heading 140); the eMeet scene
# (the real measurement camera) takes all three
@pytest.mark.parametrize("scene,hd", [("emeet", 180.0), ("emeet", 140.0), ("emeet", 0.0), ("sony", 140.0)])
def test_signs_through_the_synthetic_scene(scene, hd):
    """Zero command, the robot found 15 mm to board +x of where it started: the correction must push toward board
    -x, whatever way the robot faces. Command `left` with the robot not moving: the reference runs ahead along the
    robot's left, so the correction pushes along it (board -x at heading 0, +x at 180 and 140)."""
    mk, render = SCENES[scene]
    cfg = mk()
    loc = PH.Locator(cfg)
    x0, y0 = (420.0, 600.0) if scene == "emeet" else (450.0, 600.0)
    for k in range(3):                                       # board calibration, robot standing
        loc(render(cfg, hd, x0, y0, seed=k))
    f0 = sample(cfg, render, hd, x0, y0, loc)
    assert abs(f0["x"] - x0) < 25 and abs(f0["y"] - y0) < 25
    assert abs((f0["h"] - hd + 180) % 360 - 180) < 8, (f0["h"], hd)
    clk = Clock(2000.0)
    sam = PH.PosSampler(clock=clk)

    def feed(f, t0, n=6):
        for k in range(n):
            sam.add(t0 + k, f["x"], f["y"], f["h"])
    # (1) hold: start, then 15 mm toward board +x
    ph = new_hold(clk, sam, k=0.3, margin_mm=None, delay_steps=0, warmup_steps=0)
    feed(f0, 1994.0)
    assert ph.update(0, False, now=2000.0) is None
    ph.commit((0.0, 0.0), (0.0, 0.0), 0.0)
    f1 = sample(cfg, render, hd, x0 + 15.0, y0, loc, t0=2005.0)
    feed(f1, 2005.0)
    assert ph.update(1, False, now=2011.0) is None
    cx, cy = ph.dec["corr"]
    bx, by = PH.to_board(f1["h"], cx, cy)
    assert bx < -1.0 and abs(by) < 2.0, (hd, cx, cy, bx, by, ph.dec, f0, f1)                  # toward -x, about 30 x k = 9 mm
    assert abs(math.hypot(bx, by) - 0.3 * 15.0) < 1.5
    # and in the robot's own words: heading 0 -> its left is board -x (dy > 0); heading 180 / 140 -> its right (dy < 0 at 180)
    if hd == 0.0:
        assert cy > 0
    elif hd == 180.0:
        assert cy < 0
    # (2) command left, robot stays: reference runs along the robot's left
    clk2 = Clock(3000.0)
    sam2 = PH.PosSampler(clock=clk2)
    ph2 = new_hold(clk2, sam2, k=0.3, margin_mm=None, delay_steps=0, warmup_steps=0)
    for k in range(6):
        sam2.add(2994.0 + k, f0["x"], f0["y"], f0["h"])
    ph2.update(0, True, now=3000.0)
    for i in range(3):                                                  # three steps of 12 mm to the left, none seen
        ph2.commit((0.0, 12.0), (0.0, 12.0), 0.0)
    for k in range(6):
        sam2.add(3005.0 + k, f0["x"], f0["y"], f0["h"])
    ph2.update(3, True, now=3011.0)
    cx, cy = ph2.dec["corr"]
    assert cy > 0 and abs(cx) < 1.0, (cx, cy)                           # robot-left
    bx, by = PH.to_board(f0["h"], cx, cy)
    want_x = -math.cos(math.radians(hd))                                # board x of the robot's left
    assert bx * want_x > 0, (hd, bx, want_x)
    if hd == 0.0:
        assert bx < 0
    else:
        assert bx > 0


def test_a_board_tag_hidden_does_not_lose_the_fix():
    cfg = ecfg()
    loc = PH.Locator(cfg)
    for k in range(3):
        assert loc(erender(cfg, 180, 420, 600, seed=k)) is not None or k < 2
    r_all = loc(erender(cfg, 180, 420, 600, seed=9))
    # a frame in which the robot hides board tag 3 (only tag 2 rendered)
    tags = synth.scene_tags(cfg, G.board_R_from_euler(180, 0, 0), [420, 600, 170.0], board_ids=[2])
    img = synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=4, floor=170)
    r = loc(img)
    assert r is not None and abs(r["x"] - r_all["x"]) < 3 and abs(r["y"] - r_all["y"]) < 3
    # a moved camera is not trusted: tag 2 sits far from where it was calibrated
    cfg2 = ecfg()
    cfg2["board"]["tags"] = {**ETAGS, "2": {"center": [138.0, 0.0], "size": 80.0}}
    tags = synth.scene_tags(cfg2, G.board_R_from_euler(180, 0, 0), [420, 600, 170.0], board_ids=[2])
    img2 = synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=4, floor=170)
    assert loc(img2) is None and "camera moved" in loc.why


def test_calibrated_pose_is_saved_and_reused_when_a_board_tag_is_hidden(tmp_path):
    cfg = ecfg()
    path = str(tmp_path / "pose.json")
    loc = PH.Locator(cfg, pose_path=path)
    for k in range(3):
        loc(erender(cfg, 180, 420, 600, seed=k))
    r0 = loc(erender(cfg, 180, 420, 600, seed=7))
    tags = synth.scene_tags(cfg, G.board_R_from_euler(180, 0, 0), [420, 600, 170.0], board_ids=[3])
    one = synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=4, floor=170)
    fresh = PH.Locator(cfg)                                    # no cache: one board tag is not enough
    assert fresh(one) is None and "calibrating" in fresh.why
    again = PH.Locator(cfg, pose_path=path)                    # the next run: the saved pose, same board frame
    r1 = again(one)
    assert r1 is not None and abs(r1["x"] - r0["x"]) < 3 and abs(r1["y"] - r0["y"]) < 3


# ---- the phase-consistent sample ----------------------------------------------------------------------------------------

def sway_samples(sam, t0, t1, base=(420.0, 600.0), amp=80.0, dt=0.25, drift=0.0):
    """torso lateral sway toward the stance foot, alternating each step (T s per step), plus a slow drift."""
    t = t0
    while t < t1:
        step = int((t - 1000.0) // T)
        ph = ((t - 1000.0) % T) / T
        s = (1 if step % 2 == 0 else -1) * amp * math.sin(math.pi * ph)
        sam.add(t, base[0] + s + drift * (t - 1000.0) / T, base[1], 180.0)
        t += dt


def test_the_sample_is_the_median_over_whole_cycles_whatever_the_phase():
    sam = PH.PosSampler()
    sway_samples(sam, 1000.0, 1100.0)
    xs = [sam.fix(1030.0 + 0.37 * k, 2 * T, 4)["x"] for k in range(40)]          # decision at any phase of the cycle
    assert max(xs) - min(xs) < 12.0                                              # +-80 mm of sway -> a few mm
    one = [sam.fix(1030.0 + 0.37 * k, T, 4)["x"] for k in range(40)]            # one step alone follows the sway
    assert max(one) - min(one) > 60.0


def test_no_fix_when_samples_are_few_clumped_or_old():
    sam = PH.PosSampler()
    for t in (1000.0, 1000.2, 1000.4):
        sam.add(t, 1.0, 2.0, 0.0)
    assert sam.fix(1004.0, 8.0, 4) is None                                      # too few
    for t in (1000.5, 1000.6):
        sam.add(t, 1.0, 2.0, 0.0)
    assert sam.fix(1004.0, 8.0, 4) is None                                      # all within 0.6 s of an 8 s window
    assert sam.fix(1000.7, 1.0, 4) is not None
    assert sam.fix(1020.0, 8.0, 4) is None                                      # newest is old


# ---- the closed loop on a fake robot --------------------------------------------------------------------------------------

class FakeRobot:
    """Heading 0 (facing +y), so robot-left = board -x. Each decided step's applied displacement shows `lag` steps
    later; every step the robot also creeps `creep_lat` mm (negative = to its right) and `creep_fwd`."""

    def __init__(self, cw, sam, clk, lag=1, creep_lat=-4.0, creep_fwd=0.0, sway=60.0, drop=lambda i: False,
                 h=0.0, start=(406.0, 600.0)):
        self.cw, self.sam, self.clk, self.lag = cw, sam, clk, lag
        self.creep = (creep_fwd, creep_lat)
        self.pos = list(start)
        self.h, self.sway, self.drop = h, sway, drop
        self.applied, self.t0, self.i = [], 1000.0, 0
        self.rng = np.random.default_rng(1)
        self.track = []

    def run(self, n):
        for i in range(n):
            if self.cw.finished:
                break
            self.cw.decide()                                      # step i is decided as interval i starts
            c = self.cw.steps[-1]
            self.applied.append((c["dx"] * 1e3, c["dy"] * 1e3))
            # the interval: whatever was decided `lag` steps ago executes, plus the creep
            if i - self.lag >= 0 and i - self.lag < len(self.applied):
                dx, dy = self.applied[i - self.lag]
            else:
                dx = dy = 0.0
            bx, by = PH.to_board(self.h, dx + self.creep[0], dy + self.creep[1])
            self.pos[0] += bx
            self.pos[1] += by
            self.track.append(tuple(self.pos))
            t0 = self.t0 + i * T
            lbx, lby = PH.to_board(self.h, 0.0, 1.0)
            for k in range(16):                                   # 4 Hz frames through the step
                t = t0 + k * T / 16
                if self.drop(i):
                    continue
                s = (1 if i % 2 == 0 else -1) * self.sway * math.sin(math.pi * k / 16)
                self.sam.add(t, self.pos[0] + s * lbx + self.rng.normal(0, 3), self.pos[1] + s * lby + self.rng.normal(0, 3),
                             self.h + self.rng.normal(0, 1))
            self.clk.t = t0 + T
        return self


def hold_walk(script, clk, sam, **phkw):
    kw = dict(k=0.3, margin_mm=None, delay_steps=3)
    kw.update(phkw)
    ph = PH.PosHold(sam, T, say=lambda *a, **k: None, clock=clk, **kw)
    cw = C.CmdWalk(H, C.ListSource(script), BASE, first="R", say=lambda *a, **k: None, poshold=ph, limits=dict(pause=0),
                   max_steps=200)
    return cw, ph


def lat_err(track, start=(406.0, 600.0)):
    """lateral error in the robot's frame (heading 0: left = board -x), mm."""
    return [-(x - start[0]) for x, _ in track]


def test_closed_loop_march_in_place_with_creep():
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):                                           # the standing robot before the first step
        sam.add(990.0 + k * 0.25, 406.0, 600.0, 0.0)
    clk.t = 1000.0
    cw, ph = hold_walk(["0 0 0"] * 80, clk, sam)
    fr = FakeRobot(cw, sam, clk).run(40)
    err = lat_err(fr.track)
    assert max(abs(e) for e in err) <= 25.0, max(abs(e) for e in err)
    assert abs(fr.track[-1][1] - 600.0) < 25.0
    corrs = [d["applied_corr"][1] for d in ph.log]
    assert all(abs(c) <= 6.0 + 1e-9 for c in corrs)
    assert sum(1 for c in corrs if c > 0) > 20                    # it does push left against the creep to its right
    # the same walk without the hold drifts: 40 steps x 4 mm
    clk2 = Clock(1000.0)
    sam2 = PH.PosSampler(clock=clk2)
    cw2 = C.CmdWalk(H, C.ListSource(["0 0 0"] * 80), BASE, first="R", say=lambda *a, **k: None, limits=dict(pause=0), max_steps=200)
    fr2 = FakeRobot(cw2, sam2, clk2).run(40)
    assert abs(lat_err(fr2.track)[-1]) > 150.0
    s = ph.summary()
    assert s["steps"] == 40 and s["no_fix_steps"] == 0 and s["overrides"] == 0


@pytest.mark.parametrize("lag", [0, 1, 2])
def test_closed_loop_tolerates_a_wrong_delay_estimate(lag):
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 600.0, 0.0)
    cw, ph = hold_walk(["0 0 0"] * 80, clk, sam)                   # the hold assumes 3
    err = lat_err(FakeRobot(cw, sam, clk, lag=lag).run(40).track)
    assert max(abs(e) for e in err) <= 25.0


def test_closed_loop_forward_command_holds_the_line_and_the_pace():
    """A forward command with creep to the right and a fore-aft slip of -3 mm per step: lateral held, the along-track
    position follows the integrated command (10 mm/step)."""
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 300.0, 0.0)
    cw, ph = hold_walk([f"{10.0 / T} 0 0"] * 80, clk, sam, margin_mm=None)
    fr = FakeRobot(cw, sam, clk, creep_fwd=-3.0, start=(406.0, 300.0)).run(40)
    err = lat_err(fr.track, (406.0, 300.0))
    assert max(abs(e) for e in err) <= 25.0
    along = fr.track[-1][1] - 300.0
    assert abs(along - 400.0) < 40.0, along                        # 40 steps x 10 mm asked
    # same without the hold: 40 x 7 mm
    clk2 = Clock(1000.0)
    cw2 = C.CmdWalk(H, C.ListSource([f"{10.0 / T} 0 0"] * 80), BASE, first="R", say=lambda *a, **k: None, max_steps=200)
    fr2 = FakeRobot(cw2, PH.PosSampler(clock=clk2), clk2, creep_fwd=-3.0, start=(406.0, 300.0)).run(40)
    assert abs((fr2.track[-1][1] - 300.0) - 280.0) < 15.0


def test_dropouts_freeze_the_trim_then_zero_it_and_stop_a_moving_command():
    # a march in place: the trim is held 4 steps without a fix, then off; the walk goes on
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 600.0, 0.0)
    cw, ph = hold_walk(["0 0 0"] * 80, clk, sam)
    fr = FakeRobot(cw, sam, clk, drop=lambda i: 14 <= i < 24).run(30)
    assert not cw.finished or cw.reason != "x"
    nf = [d for d in ph.log if d["frozen"]]
    assert len(nf) >= 6
    assert any("correction frozen" in n for d in nf for n in d["notes"])
    assert any("correction off" in n for d in nf for n in d["notes"])
    assert all(d["applied_corr"] == (0.0, 0.0) for d in nf[5:])
    assert cw.log[-1]["pos"]["fix"]                                # fix came back, hold resumed
    # a walk with a command: more than 4 steps without a fix turns it into stop, and says why
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 300.0, 0.0)
    cw, ph = hold_walk([f"{10.0 / T} 0 0"] * 80, clk, sam)
    FakeRobot(cw, sam, clk, drop=lambda i: i >= 10, start=(406.0, 300.0)).run(30)
    assert cw.finished and "no fix for" in cw.reason and "command active" in cw.reason
    last_cmds = [r["step_mm_deg"][0] for r in cw.log]
    assert last_cmds[-1] == 0.0 and cw.log[-1]["state"] == "stopping"


def test_edge_rule_steps_toward_the_centre_and_is_printed():
    lines = []
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    W, L = 812.8, 1219.2
    cfg = {"board": {"width_mm": W, "length_mm": L}, "foot_margin_mm": 0.0}
    for k in range(40):                                           # 150 mm from the left edge, heading 0 (left = board -x)
        sam.add(990.0 + k * 0.25, 150.0, 600.0, 0.0)
    ph = PH.PosHold(sam, T, margins=lambda x, y: G.edge_margins(cfg, x, y), margin_mm=250.0, centre=(W / 2, L / 2),
                    say=lines.append, clock=clk)
    cw = C.CmdWalk(H, C.ListSource(["0 0 0"] * 30), BASE, first="R", say=lambda *a, **k: lines.append(" ".join(map(str, a))),
                   poshold=ph, limits=dict(pause=0))
    cw.decide()
    rec = cw.log[-1]
    assert rec["pos"]["keep_on_board"]["edges"] == ["left"]
    # toward the centre = board +x = the robot's right at heading 0: dy < 0, bounded by the side cap and the slew
    assert rec["step_mm_deg"][1] < -1.0 and abs(rec["step_mm_deg"][1]) <= C.SIDE_MM + 1e-9 and abs(rec["step_mm_deg"][1]) <= 10.0 + 1e-9
    assert abs(rec["step_mm_deg"][0]) < 1.0
    assert any("KEEP-ON-BOARD" in ln for ln in lines) and any("keep-on-board" in ln for ln in lines)
    # heading 180 (facing the Sony): toward the centre is the robot's left
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 150.0, 600.0, 180.0)
    ph = PH.PosHold(sam, T, margins=lambda x, y: G.edge_margins(cfg, x, y), margin_mm=250.0, centre=(W / 2, L / 2),
                    say=lambda *a: None, clock=clk)
    cw = C.CmdWalk(H, C.ListSource(["0 0 0"] * 30), BASE, first="R", say=lambda *a, **k: None, poshold=ph, limits=dict(pause=0))
    cw.decide()
    assert cw.log[-1]["step_mm_deg"][1] > 1.0
    # near the far edge, facing it (heading 180 faces -y; the far end is +y: behind it): a step back
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 1100.0, 0.0)
    ph = PH.PosHold(sam, T, margins=lambda x, y: G.edge_margins(cfg, x, y), margin_mm=250.0, centre=(W / 2, L / 2),
                    say=lambda *a: None, clock=clk)
    cw = C.CmdWalk(H, C.ListSource([f"{10.0 / T} 0 0"] * 30), BASE, first="R", say=lambda *a, **k: None, poshold=ph)
    cw.decide()
    assert cw.log[-1]["step_mm_deg"][0] < -1.0 and cw.log[-1]["pos"]["keep_on_board"]["edges"] == ["far"]


def test_pending_steps_count_toward_the_edge_rule():
    """Already-uploaded steps toward the edge count: 150 + ... the predicted position, not the sample alone."""
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    W, L = 812.8, 1219.2
    cfg = {"board": {"width_mm": W, "length_mm": L}, "foot_margin_mm": 0.0}
    for k in range(40):
        sam.add(990.0 + k * 0.25, 300.0, 600.0, 0.0)                # 300 mm from the left edge: outside the 250 margin
    ph = PH.PosHold(sam, T, margins=lambda x, y: G.edge_margins(cfg, x, y), margin_mm=250.0, centre=(W / 2, L / 2),
                    say=lambda *a: None, clock=clk, delay_steps=3)
    ph.update(0, True, now=1000.0)
    assert ph.dec["override"] is None
    for _ in range(3):                                              # three uploaded steps of 15 mm to the left (board -x)
        ph.commit((0.0, 15.0), (0.0, 15.0), 0.0)
    ph.update(3, True, now=1000.0)
    assert ph.dec["pred"][0] == pytest.approx(300.0 - 45.0, abs=1.0)        # 255 mm from the edge: still outside 250
    for k in range(8):                                              # the robot has moved on: a sample at 290, one more step
        sam.add(1002.0 + k, 290.0, 600.0, 0.0)
    ph.commit((0.0, 15.0), (0.0, 15.0), 0.0)
    ph.update(4, True, now=1010.0)
    assert ph.dec["pred"][0] == pytest.approx(290.0 - 45.0, abs=8.0)
    assert ph.dec["override"] is not None and ph.dec["override"]["edges"] == ["left"]


# ---- off = unchanged --------------------------------------------------------------------------------------------------------

def test_hold_with_no_creep_stays_out_of_the_way():
    """The robot does exactly what is asked (no creep): no lateral correction at all and only a couple of mm fore-aft."""
    cmds = [f"{10.0 / T} 0 0"] * 6 + ["0 0 0"] * 6
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 300.0, 0.0)
    cw, ph = hold_walk(cmds, clk, sam, delay_steps=2)             # lag 1 step + the window's 1 step
    FakeRobot(cw, sam, clk, creep_lat=0.0, start=(406.0, 300.0)).run(12)
    clk2 = Clock(1000.0)
    plain = C.CmdWalk(H, C.ListSource(cmds), BASE, first="R", say=lambda *a, **k: None, limits=dict(pause=0))
    FakeRobot(plain, PH.PosSampler(clock=clk2), clk2, creep_lat=0.0, start=(406.0, 300.0)).run(12)
    # the speed ramp shows a few mm of apparent fore-aft error (the window averages the ramp): corrections stay small
    assert all(abs(c["dx"] - p["dx"]) * 1e3 < 2.0 and c["dy"] == p["dy"] for c, p in zip(cw.steps, plain.steps))
    assert all(abs(d["applied_corr"][0]) < 2.0 and d["applied_corr"][1] == 0.0 for d in ph.log)


def test_flag_is_off_by_default_and_needs_cmd(capsys):
    a = H.main(["--pos-hold", "on"])
    assert a == 2 and "needs --cmd" in capsys.readouterr().out
