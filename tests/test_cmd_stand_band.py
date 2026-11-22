"""The stand band (#263; sim/cmd_kernel.py, "The STAND BAND").

|c0|, |c1|, |c2| at or below the band are the plain stand in the reward, the gait clock and the
trim; above it a walk. The legacy band 0.05 graded 8.8 % of the volume's walk draws as stands (a
dead band), so a --cmd-volume run trains with STAND_BAND_VOLUME (half the link's milli-unit
resolution). Off the volume path nothing changes: no kwarg in the run's env, the same gates
(bit-identity on the night job's arguments is the PR's proof; the envs' arithmetic here). The
CPU-vs-MJX parity with the band on is sim/mjx/parity_test.py R25; the firmware reads the run's band
from obs_spec.h (tools/gen_obs_spec.py, obs::kStandBand)."""
import inspect
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import cmd_kernel as ck  # noqa: E402
import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--precision", "--mjx-impl", "jax",
        "--family", "loco", "--cmd-v-range", "0.05,0.24"]


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["cmd_stand_band"].default == ck.STAND_BAND
    assert ck.STAND_BAND == 0.05 and ck.STAND_BAND_VOLUME == 0.0005
    base = _kw()
    assert "cmd_stand_band" not in base                       # off the volume path: not recorded
    assert _kw("--cmd-volume", "1.0")["cmd_stand_band"] == ck.STAND_BAND_VOLUME
    assert _kw("--cmd-volume", "1.0", "--cmd-stand-band", "0.05")["cmd_stand_band"] == 0.05
    one = _kw("--cmd-stand-band", "0.01")
    assert one.pop("cmd_stand_band") == 0.01 and one == base
    assert ck.parse_stand_band(None) == 0.05 and ck.parse_stand_band(0) == 0.0
    for bad in (-0.1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            ck.parse_stand_band(bad)


def test_the_volume_dead_band_and_its_fix():
    """The night recipe's volume (forward 0.24, backward 0.4, lateral 0.25, turn 1.0): the legacy
    band grades ~8.8 % of the walk draws as stands; the volume band only the draws the link would
    deliver as zero."""
    u = np.random.default_rng(0).uniform(size=(4, 200000))
    vx, vy, wz = ck.volume_cmd(*u, ck.volume_axes((0.05, 0.24), (-0.4, -0.15), 0.25, 1.0))

    def stand(b):
        return float(np.mean((np.abs(vx) <= b) & (np.abs(vy) <= b) & (np.abs(wz) <= b)))
    assert 0.08 < stand(ck.STAND_BAND) < 0.095
    assert stand(ck.STAND_BAND_VOLUME) < 0.002
    # a stand the link delivers as non-zero is a walk: the smallest wire command is 1e-3
    assert ck.STAND_BAND_VOLUME < 0.001 / 2 + 1e-12


def _env(**kw):
    import robot_plant
    k = dict(command_mode=True, ext_cmd=True, domain_rand=False, actuator_model="sts3215",
             gait_clock=True, clock_stand_freeze=True, w_feet_air=5.0, w_single_support=0.3,
             cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
    k.update(robot_plant.robot_env_kwargs(None))
    k.update(kw)
    return W.BimoWalkerEnv(**k)


def test_cpu_env_gates_follow_the_band():
    """A slow command (0.03 m/s) is a stand under the legacy band and a walk under the volume
    band: the clock (frozen at a stand here) runs, the trim gate opens, the stepping reference and
    the reward's gait shaping follow; the zero command stays the stand under both."""
    slow = (0.03, 0.02, 0.0, 1.0, 0.0, 0.0, 0.0)
    out = {}
    for b in (ck.STAND_BAND, ck.STAND_BAND_VOLUME):
        e = _env(cmd_stand_band=b, w_mimic=1.0, mimic_lift=0.02)
        e.reset(seed=4)
        assert not e._cmd_is_moving(np.zeros(7))
        e.set_command(*slow)
        ph0 = e._gait_phase
        z = np.zeros(e.action_space.shape, np.float32)
        rs = [e.step(z)[1] for _ in range(5)]
        out[b] = dict(moving=e._cmd_is_moving(e._cmd), trim=e._trim_walking(),
                      clock=e._gait_phase != ph0, r=np.asarray(rs))
    leg, vol = out[ck.STAND_BAND], out[ck.STAND_BAND_VOLUME]
    assert not (leg["moving"] or leg["trim"] or leg["clock"])
    assert vol["moving"] and vol["trim"] and vol["clock"]
    assert not np.array_equal(out[ck.STAND_BAND]["r"], out[ck.STAND_BAND_VOLUME]["r"])
    with pytest.raises(ValueError):
        W.BimoWalkerEnv(command_mode=True, ext_cmd=False, cmd_stand_band=0.0005)


def test_cpu_env_default_band_is_unchanged():
    a = _env()
    b = _env(cmd_stand_band=0.05)
    oa, _ = a.reset(seed=9)
    ob, _ = b.reset(seed=9)
    assert np.array_equal(oa, ob)
    for c in ((0.03, 0.0, 0.0, 1.0), (0.2, 0.0, 0.3, 1.0), (0.0, 0.0, 0.0, 1.0)):
        a.set_command(*c)
        b.set_command(*c)
        for k in range(4):
            act = np.full(a.action_space.shape, 0.1 * k, np.float32)
            ra, rb = a.step(act), b.step(act)
            assert np.array_equal(ra[0], rb[0]) and ra[1] == rb[1]


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R25, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--stand-band"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


# -- the firmware's band: obs_spec.h (tools/gen_obs_spec.py) -------------------------------------
def _gen():
    pytest.importorskip("mujoco")
    import importlib.util
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    spec = importlib.util.spec_from_file_location(
        "gen_obs_spec", os.path.join(ROOT, "tools", "gen_obs_spec.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


DEPLOYED = os.path.join(ROOT, "firmware", "deployed", "runs", "robot_legs_walk16_act_s128r24")


def _spec(tmp_path, gen, **over):
    import json
    cfg = json.load(open(os.path.join(DEPLOYED, "config.json")))
    cfg.update(over)
    d = tmp_path / ("run_" + "_".join(sorted(over)) if over else "run")
    d.mkdir()
    (d / "config.json").write_text(json.dumps(cfg))
    cfg, env = gen.build_env(str(d))
    cfg["_run"] = str(d)
    inc = d / "inc" / "obs"
    inc.mkdir(parents=True)
    gen.write_spec(cfg, env, str(inc / "obs_spec.h"))
    return env, (inc / "obs_spec.h").read_text(), d / "inc"


def test_obs_spec_carries_only_a_non_default_band(tmp_path):
    gen = _gen()
    env0, h0, _ = _spec(tmp_path, gen)
    env1, h1, _ = _spec(tmp_path, gen, cmd_stand_band=0.05)
    env2, h2, _ = _spec(tmp_path, gen, cmd_stand_band=0.0005, cmd_volume=1.0)
    def body(h):                     # all but the lines naming the run (its dir here)
        return [ln for ln in h.splitlines() if not ln.startswith("// Source:") and "kRunName" not in ln]
    assert "STAND_BAND" not in h0 and "kStandBand" not in h0
    assert body(h1) == body(h0)      # an explicit 0.05 writes the bytes a run without one does
    assert env2.cmd_stand_band == 0.0005
    assert "#define OBS_STAND_BAND 1" in h2
    assert "inline constexpr float kStandBand = 0.0005f;" in h2


@pytest.mark.skipif(shutil.which("g++") is None, reason="no g++: the header's #ifdef branch is not compiled")
def test_firmware_reads_the_band_from_obs_spec(tmp_path):
    """obs/assembler.h's kStandBand is 0.05 for a run without a band and the run's own with one;
    plainStand and gaitplay::clockWalking read it (compiled against a generated obs_spec.h)."""
    gen = _gen()
    fw = os.path.join(ROOT, "firmware", "components")
    prog = tmp_path / "band.cpp"
    prog.write_text(r'''
#include <stdio.h>
#include "obs/assembler.h"
#include "gaitplay/track.h"
int main() {
    float c[obs::kNumCmd] = {};
    if (obs::kNumCmd > 3) c[3] = 1.0f;
    c[0] = 0.03f;
    printf("%.9g %d %d\n", (double)obs::kStandBand, obs::plainStand(c) ? 1 : 0,
           gaitplay::clockWalking(c, obs::kNumCmd, obs::kStandBand) ? 1 : 0);
    c[0] = 0.0f;
    printf("%d\n", obs::plainStand(c) ? 1 : 0);
    return 0;
}
''')
    res = {}
    for tag, over in (("legacy", {}), ("volume", dict(cmd_stand_band=0.0005, cmd_volume=1.0))):
        _, _, inc = _spec(tmp_path, gen, **over)
        exe = tmp_path / f"band_{tag}"
        cmd = ["g++", "-std=c++17", "-O0", f"-I{inc}", f"-I{fw}/obs/include", f"-I{fw}/gaitplay/include",
               str(prog), f"{fw}/obs/assembler.cpp", f"{fw}/gaitplay/track.cpp", "-o", str(exe), "-lm"]
        p = subprocess.run(cmd, capture_output=True, text=True)
        assert p.returncode == 0, p.stderr[-3000:]
        res[tag] = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split()
    assert res["legacy"] == ["0.0500000007", "1", "0", "1"]
    assert res["volume"] == ["0.000500000024", "0", "1", "1"]
