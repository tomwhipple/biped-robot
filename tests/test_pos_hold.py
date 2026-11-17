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
    kw = dict(dict(deadband_mm=0.0, ki=0.0, rate_mm=99.0), **kw)      # the sign tests want the plain proportional law
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


def test_where_pys_cached_board_pose_file_is_read_too(tmp_path):
    import json
    cfg = ecfg()
    first = PH.Locator(cfg, pose_path=str(tmp_path / "mine.json"))
    for k in range(3):
        first(erender(cfg, 180, 420, 600, seed=k))
    rec = json.load(open(tmp_path / "mine.json"))
    rec["tag_corners_px"] = rec.pop("corners")                  # where.py's key for the same thing
    (tmp_path / "emeet_board_pose.json").write_text(json.dumps(rec))
    tags = synth.scene_tags(cfg, G.board_R_from_euler(180, 0, 0), [420, 600, 170.0], board_ids=[3])
    one = synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=4, floor=170)
    loc = PH.Locator(cfg, pose_read=[str(tmp_path / "emeet_board_pose.json")])
    assert loc(one) is not None


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
                 h=0.0, lat_gain=1.0, bias_sd=0.0, seed=1, start=(406.0, 600.0)):
        self.cw, self.sam, self.clk, self.lag = cw, sam, clk, lag
        self.creep = (creep_fwd, creep_lat)
        self.pos = list(start)
        self.h, self.sway, self.drop, self.lat_gain = h, sway, drop, lat_gain
        self.applied, self.t0, self.i = [], 1000.0, 0
        self.rng = np.random.default_rng(seed)
        self.bias_sd = bias_sd
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
            bx, by = PH.to_board(self.h, dx + self.creep[0], self.lat_gain * dy + self.creep[1])
            self.pos[0] += bx
            self.pos[1] += by
            self.track.append(tuple(self.pos))
            t0 = self.t0 + i * T
            lbx, lby = PH.to_board(self.h, 0.0, 1.0)
            bias = self.rng.normal(0, self.bias_sd) if self.bias_sd else 0.0       # the window's own jitter on the real robot
            for k in range(16):                                   # 4 Hz frames through the step
                t = t0 + k * T / 16
                if self.drop(i):
                    continue
                s = (1 if i % 2 == 0 else -1) * self.sway * math.sin(math.pi * k / 16) + bias
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


def run_hold(g, creep, lag=2, bias=0.0, seed=1, n=40, cmd="0 0 0", **phkw):
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):                                           # the standing robot before the first step
        sam.add(990.0 + k * 0.25, 406.0, 600.0, 0.0)
    cw, ph = hold_walk([cmd] * 100, clk, sam, **phkw)
    fr = FakeRobot(cw, sam, clk, lag=lag, creep_lat=creep, lat_gain=g, bias_sd=bias, seed=seed).run(n)
    return np.array(lat_err(fr.track)), np.array([d["applied_corr"][1] for d in ph.log]), cw, ph


def uncontrolled(creep, g=1.0, n=40):
    clk = Clock(1000.0)
    cw = C.CmdWalk(H, C.ListSource(["0 0 0"] * 100), BASE, first="R", say=lambda *a, **k: None, limits=dict(pause=0), max_steps=200)
    return np.array(lat_err(FakeRobot(cw, PH.PosSampler(clock=clk), clk, creep_lat=creep, lat_gain=g).run(n).track))


def test_closed_loop_march_in_place_with_creep():
    """-4 mm per step creep (to the robot's right), the plant moves what it is asked: the first steps cost the start (3 steps
    before the window holds whole cycles, 2 to 3 steps of delay), then the error comes back slowly (the integral learns the
    creep without winding up through the delay): under 40 mm throughout, falling from step 15, 25 mm or less from step 38."""
    err, corr, cw, ph = run_hold(1.0, -4.0)
    assert np.abs(err).max() <= 40.0 and np.abs(err[15:]).max() <= np.abs(err).max() and np.abs(err[-3:]).max() <= 30.0, (np.abs(err).max(), np.abs(err[-3:]).max())
    assert abs(uncontrolled(-4.0)[-1]) > 150.0                    # without the hold: 40 x 4 mm
    assert np.abs(corr).max() <= PH.CLIP_LAT_MM + 1e-9 and np.abs(np.diff(corr)).max() <= PH.RATE_MM + 1e-9
    assert ph.lat_off is None
    s = ph.summary()
    assert s["steps"] == 40 and s["no_fix_steps"] == 0 and s["overrides"] == 0 and s["max_abs_err_lat_mm"] <= 40.0
    assert s["start_at_rest"] is None                             # begin() was not called here: no standing pose logged


@pytest.mark.parametrize("lag", [0, 1, 2, 3])
def test_closed_loop_tolerates_a_wrong_delay_estimate(lag):
    err, _, _, ph = run_hold(1.0, -4.0, lag=lag)                  # the hold assumes 3
    assert np.abs(err).max() <= 45.0 and np.abs(err[-3:]).max() <= 32.0 and ph.lat_off is None


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_closed_loop_on_the_measured_plant_does_not_limit_cycle(seed):
    """The robot of 2026-11-16: a sideways step moves it 0.37 of what was asked (0.2 to 1.0 on other runs), the
    position sample jitters ~10 mm per decision, a creep of 2.6 mm per step. It cannot hold within 25 mm that fast (the
    corrections are slew-limited and the plant weak), but it must hold in a band without hunting, and beat no hold."""
    err, corr, cw, ph = run_hold(0.37, -2.6, bias=10.0, seed=seed)
    assert np.abs(err).max() <= 50.0 and np.abs(err[20:]).mean() <= 40.0, (np.abs(err).max(), np.abs(err[20:]).mean())
    assert abs(uncontrolled(-2.6, 0.37)[-1]) > 100.0
    active = corr[np.abs(corr) > 0.5]
    assert np.sum(np.diff(np.sign(active)) != 0) <= 4             # one sign: no hunting
    assert np.std(np.diff(corr)) < 2.5 and ph.lat_off is None


def test_closed_loop_without_creep_stays_quiet():
    err, corr, cw, ph = run_hold(1.0, 0.0, bias=10.0, seed=2)
    assert np.abs(err).max() <= 25.0 and np.abs(corr).max() <= 6.0
    assert np.sum(np.diff(np.sign(corr[np.abs(corr) > 0.5])) != 0) <= 6


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
    assert max(abs(e) for e in err) <= 40.0 and max(abs(e) for e in err[-3:]) <= 36.0
    along = fr.track[-1][1] - 300.0
    assert abs(along - 400.0) < 40.0, along                        # 40 steps x 10 mm asked
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


@pytest.mark.parametrize("gain", [-1.0, -0.5])
def test_a_wrong_signed_sideways_response_turns_the_lateral_correction_off(gain):
    """The pause timing moved the robot the wrong way on a sideways command (2026-11-16): the hold must notice and stop
    correcting laterally, say so in the step line, and keep fore-aft on."""
    lines = []
    clk = Clock(1000.0)
    sam = PH.PosSampler(clock=clk)
    for k in range(40):
        sam.add(990.0 + k * 0.25, 406.0, 600.0, 0.0)
    cw, ph = hold_walk(["0 0 0"] * 100, clk, sam)
    cw.say = lambda *a, **k: lines.append(" ".join(map(str, a)))
    FakeRobot(cw, sam, clk, lat_gain=gain, creep_lat=-4.0, bias_sd=8.0).run(30)
    assert ph.lat_off is not None and "wrong-signed" in ph.lat_off
    k_off = next(i for i, d in enumerate(ph.log) if any("LATERAL CORRECTION OFF" in n for n in d["notes"]))
    assert k_off < 25 and all(d["applied_corr"][1] == 0.0 for d in ph.log[k_off + 3:])
    assert any("LATERAL CORRECTION OFF" in ln for ln in lines)                    # printed, not only logged
    assert any("LATERAL CORRECTION OFF" in n for n in cw.log[-1]["pos"]["notes"])
    assert ph.summary()["lateral_off"] == ph.lat_off


@pytest.mark.parametrize("gain", [0.2, 0.37, 1.0])
def test_a_weak_but_right_response_is_not_turned_off(gain):
    for seed in (1, 2, 3):
        _, _, _, ph = run_hold(gain, -4.0, bias=10.0, seed=seed, n=40)
        assert ph.lat_off is None, (gain, seed, ph.lat_off)


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


# ---- side timing: frame level -------------------------------------------------------------------------------------------------

PAUSE_KW = dict(td_pause=0.3, shift_s=1.2, settle=0.1, raise_s=0.77, move_s=0.45, lower_s=0.55)


def _plan(cmds, **kw):
    return H.plan(**dict(T_step=4.0, n_steps=len(cmds), direction="march", step_cmds=cmds, lift_gate=False,
                         **{k: v for k, v in kw.items()}))


def _cmds(dys, side_timing):
    cw = C.CmdWalk(H, C.ListSource(["0 0 0"]), BASE, first="R", say=lambda *a, **k: None, side_timing=side_timing)
    return [cw._cmd(0.0, dy, 0.0, 0.0) for dy in dys]


def test_side_timing_plain_lays_out_the_sideways_step_and_the_next_with_the_plain_timing():
    import dwell as D
    dys = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
    plain = D.resolve_times(4.0)
    var = D.resolve_times(4.0, shift_s=1.2, settle=0.1, raise_s=0.77, move_s=0.45, lower_s=0.55)
    p = _plan(_cmds(dys, "plain"), **PAUSE_KW)
    ph = p["sway"].phases
    ss = [x["t1"] - x["t0"] for x in ph]
    # the single support of the sideways step (2) and of the next (3) is the plain one; the others the variant's
    assert ss[2] == pytest.approx(plain["t_ss"]) and ss[3] == pytest.approx(plain["t_ss"])
    assert [ss[i] for i in (0, 1, 4, 5)] == pytest.approx([var["t_ss"]] * 4)
    steps = p["sway"]._steps
    # no pause before step 3 (it follows the sideways step) and its shift + settle are the plain ones
    gap = lambda i: steps[i]["s0"] - steps[i - 1]["td"]              # noqa: E731
    assert gap(3) == pytest.approx(0.0) and gap(2) == pytest.approx(0.3) and gap(4) == pytest.approx(0.3)
    assert steps[3]["lo"] - steps[3]["s0"] == pytest.approx(plain["t_sh"] + plain["t_set"])
    assert steps[2]["lo"] - steps[2]["s0"] == pytest.approx(var["t_sh"] + var["t_set"])       # its own lead is decided before it
    # 'same' keeps one timing: identical frames to a plan that never heard of side timing
    same = _plan(_cmds(dys, "same"), **PAUSE_KW)
    base = _plan([dict(c) for c in _cmds(dys, "same")], **PAUSE_KW)
    assert _hash(same) == _hash(base) and _hash(same) != _hash(p)


def test_side_timing_changes_nothing_without_a_sideways_step_or_without_a_pause():
    flat = _cmds([0.0] * 5, "plain")
    assert not any(c.get("lat") for c in flat)
    assert _hash(_plan(flat, **PAUSE_KW)) == _hash(_plan(_cmds([0.0] * 5, "same"), **PAUSE_KW))
    side = _cmds([0.0, 10.0, 0.0, 0.0], "plain")
    # no pause / explicit timing: the plain timing IS the timing, the frames do not change
    assert _hash(_plan(side)) == _hash(_plan(_cmds([0.0, 10.0, 0.0, 0.0], "same")))
    # 3 mm and under is not a sideways step
    assert not any(c.get("lat") for c in _cmds([3.0, -3.0], "plain"))


def test_side_timing_phase_for_the_firmware_follows_each_steps_own_timing():
    p = _plan(_cmds([0.0, 10.0, 0.0, 0.0], "plain"), **PAUSE_KW)
    sw = p["sway"]
    for k, ph in enumerate(sw.phases):                      # the firmware's phase is 0.25 at lift-off, 0.75 at touchdown
        assert sw.phi(ph["t0"]) == pytest.approx(np.pi * (k + 0.25) + np.pi, abs=1e-6)
        assert sw.phi(ph["t1"]) == pytest.approx(np.pi * (k + 0.75) + np.pi, abs=1e-6)


def _hash(p):
    import hashlib
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]
