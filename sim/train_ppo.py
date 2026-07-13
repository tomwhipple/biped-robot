"""Stage-2b (CPU baseline): train a PPO walking policy on BimoWalkerEnv.

This is the *baseline* trainer -- Stable-Baselines3 PPO on CPU, meant to prove the
env + reward actually produce forward motion before investing in the MJX/GPU port.
It is not tuned for a competition gait; a few hundred k steps on a laptop should
move the robot's forward-distance and episode return clearly above the standing
baseline.

Usage:
    python train_ppo.py                       # 300k steps, 4 envs
    python train_ppo.py --steps 1_000_000 --n-envs 8
    python train_ppo.py --steps 50_000        # quick smoke run

Outputs (under sim/runs/ppo_baseline/):
    model.zip          trained policy
    vecnormalize.pkl   obs/reward normalization stats (needed for eval)
    tensorboard/       training curves  (tensorboard --logdir sim/runs)
"""
from __future__ import annotations
import argparse
import json
import os

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback

from walker_env import BimoWalkerEnv

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=300_000, help="total env steps")
    p.add_argument("--n-envs", type=int, default=4, help="parallel envs")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--run-name", default="ppo_baseline", help="subdir under runs/")
    # -- model / actuator -----------------------------------------------------
    p.add_argument("--xml", default=None,
                   help="MJCF file (default: bimo_biped.xml; use bimo_biped_v2.xml "
                        "for the CAD-true model). Relative paths resolve in sim/.")
    p.add_argument("--actuator-model", default="ideal", choices=["ideal", "sts3215"],
                   help="'sts3215' = PD torque clamped to the servo torque-speed "
                        "envelope at --voltage; 'ideal' = original position servos")
    p.add_argument("--voltage", type=float, default=7.4,
                   help="servo supply voltage (7.4=2S, 11.1=3S, 12=bench supply)")
    p.add_argument("--servo-kp", type=float, default=12.0)
    p.add_argument("--servo-kd", type=float, default=0.25)
    # -- torso-top camera payload (GoPro MAX 360 = 0.154 kg) -------------------
    p.add_argument("--payload", type=float, default=0.0,
                   help="fixed torso-top payload mass in kg (0.154 = GoPro MAX)")
    p.add_argument("--payload-max", type=float, default=None,
                   help="per-episode payload drawn uniform(0, this) kg -- one "
                        "policy for camera-on and camera-off (e.g. 0.17)")
    # -- 2 m dash objective ---------------------------------------------------
    p.add_argument("--dash", action="store_true",
                   help="episode succeeds on crossing 2 m + staying upright 1 s")
    p.add_argument("--dash-stop", action="store_true",
                   help="stricter finish: must come to a standstill past the "
                        "line (planar speed < 0.15 m/s for 1 s)")
    p.add_argument("--imu-obs", action="store_true",
                   help="hardware-realizable observations: linvel/height "
                        "zeroed, IMU noise/bias DR on up-vector + gyro")
    p.add_argument("--imu-noise", type=float, default=1.0,
                   help="scale on the IMU misalignment/bias/noise DR")
    p.add_argument("--w-time-stop", type=float, default=1.5,
                   help="extra per-step penalty after crossing (dash_stop): "
                        "makes loitering past the line net-negative")
    p.add_argument("--command-mode", action="store_true",
                   help="command-conditioned locomotion: track a commanded "
                        "(vx, yaw-rate), incl. zero = stand (replaces dash)")
    p.add_argument("--w-track-v", type=float, default=2.0)
    p.add_argument("--w-track-w", type=float, default=1.0)
    p.add_argument("--cmd-stand-prob", type=float, default=0.3)
    p.add_argument("--cmd-v-max", type=float, default=1.0)
    p.add_argument("--cmd-w-range", type=float, default=1.0)
    p.add_argument("--cmd-fixed", type=float, nargs=2, default=None,
                   metavar=("V", "W"),
                   help="pin the command (expert training for distillation)")
    p.add_argument("--cmd-dense", action="store_true",
                   help="dense-progress velocity reward for moving commands")
    p.add_argument("--cmd-hold-min", type=float, default=2.5)
    p.add_argument("--cmd-hold-max", type=float, default=4.5)
    p.add_argument("--w-power", type=float, default=0.0,
                   help="electrical-power penalty (see walker_env)")
    p.add_argument("--w-time", type=float, default=0.0,
                   help="per-step time penalty (dash urgency)")
    p.add_argument("--finish-bonus", type=float, default=0.0,
                   help="one-off reward on a confirmed upright finish")
    # -- reward-shaping knobs (forwarded to BimoWalkerEnv) --------------------
    p.add_argument("--w-forward", type=float, default=1.5)
    p.add_argument("--target-speed", type=float, default=None,
                   help="cap fwd-vel reward (m/s); set to kill the lunge exploit")
    p.add_argument("--w-upright", type=float, default=0.5)
    p.add_argument("--alive", type=float, default=0.1)
    p.add_argument("--w-height", type=float, default=0.0)
    p.add_argument("--w-energy", type=float, default=0.002)
    p.add_argument("--w-action-rate", type=float, default=0.05)
    p.add_argument("--fall-cost", type=float, default=1.0)
    # -- gait-quality shaping (defaults 0 -> old reward reproduces) ----------
    p.add_argument("--w-feet-air", type=float, default=0.0,
                   help="reward per-foot swing time on touchdown (anti-shuffle)")
    p.add_argument("--air-time-target", type=float, default=0.3,
                   help="cap on rewarded swing duration (s)")
    p.add_argument("--w-single-support", type=float, default=0.0,
                   help="per-step reward when exactly one foot is in contact")
    p.add_argument("--w-lateral", type=float, default=0.0,
                   help="penalty on lateral velocity + sideways drift")
    p.add_argument("--w-yaw", type=float, default=0.0,
                   help="penalty on heading error + yaw rate")
    p.add_argument("--w-pitch-rate", type=float, default=0.0,
                   help="penalty on torso roll/pitch angular rates")
    # -- procedural terrain (0 -> original flat plane) -----------------------
    p.add_argument("--terrain-amplitude", type=float, default=0.0,
                   help="max bump height in meters (e.g. 0.008 mild, 0.015 rough)")
    p.add_argument("--terrain-smoothness", type=float, default=0.15,
                   help="terrain feature size in meters")
    p.add_argument("--terrain-amplitude-min", type=float, default=None,
                   help="per-episode bump height drawn uniform(min, amplitude) "
                        "-- mixes easy and hard episodes (built-in curriculum)")
    p.add_argument("--terrain-mix", type=float, default=1.0,
                   help="fraction of parallel envs that use terrain; the rest "
                        "use the original flat plane (hfield and plane contacts "
                        "differ subtly, so train on both)")
    # -- domain randomization (sim-to-real hardening) ------------------------
    p.add_argument("--domain-rand", action="store_true",
                   help="randomize mass/friction/gain + shoves each episode")
    p.add_argument("--action-latency", type=int, default=0, help="control-step action delay")
    p.add_argument("--latency-ms", type=float, default=0.0,
                   help="sub-step action latency in ms (0..20; real bus is ~2-5)")
    p.add_argument("--latency-ms-max", type=float, default=None,
                   help="per-episode latency drawn uniform(latency-ms, this) -- latency DR")
    p.add_argument("--latency-jitter-ms", type=float, default=0.0,
                   help="per-control-step latency jitter, +/- this many ms")
    p.add_argument("--backlash-deg", type=float, default=0.0,
                   help="gear backlash deadzone (STS3215 measures ~0.5-1.0)")
    p.add_argument("--backlash-deg-max", type=float, default=None,
                   help="per-episode backlash drawn uniform(backlash-deg, this)")
    p.add_argument("--fall-height", type=float, default=0.18)
    p.add_argument("--fall-up-z", type=float, default=0.4)
    p.add_argument("--push-force", type=float, default=None,
                   help="shove magnitude N; 0 disables shoves (None=auto with DR)")
    p.add_argument("--push-prob", type=float, default=None,
                   help="per-step shove probability (None=auto with DR)")
    # -- warm start (curriculum: flat-ground walker -> DR-hardened) ----------
    p.add_argument("--init-from", default=None,
                   help="run-name to load policy+normalizer from and continue training")
    p.add_argument("--lr", type=float, default=None,
                   help="override learning rate (use a small LR to gently fine-tune)")
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run_name)
    os.makedirs(run_dir, exist_ok=True)

    env_kwargs = dict()
    if args.xml:
        xml = args.xml if os.path.isabs(args.xml) else \
            os.path.join(os.path.dirname(os.path.abspath(__file__)), args.xml)
        env_kwargs["xml_path"] = xml
    env_kwargs.update(
        actuator_model=args.actuator_model, supply_voltage=args.voltage,
        servo_kp=args.servo_kp, servo_kd=args.servo_kd,
        payload_mass=args.payload, payload_max=args.payload_max,
        dash=args.dash, dash_stop=args.dash_stop,
        command_mode=args.command_mode, w_track_v=args.w_track_v,
        cmd_v_range=(0.3, args.cmd_v_max), cmd_w_range=args.cmd_w_range,
        cmd_resample_s=(args.cmd_hold_min, args.cmd_hold_max),
        cmd_fixed=tuple(args.cmd_fixed) if args.cmd_fixed else None,
        cmd_dense=args.cmd_dense,
        w_power=args.w_power,
        w_track_w=args.w_track_w, cmd_stand_prob=args.cmd_stand_prob,
        imu_obs=args.imu_obs, imu_noise=args.imu_noise,
        backlash_deg=args.backlash_deg, backlash_deg_max=args.backlash_deg_max,
        fall_height=args.fall_height, fall_up_z=args.fall_up_z,
        w_time=args.w_time, w_time_stop=args.w_time_stop,
        finish_bonus=args.finish_bonus,
        w_forward=args.w_forward, target_speed=args.target_speed,
        w_upright=args.w_upright, alive_bonus=args.alive, w_height=args.w_height,
        w_energy=args.w_energy, w_action_rate=args.w_action_rate,
        fall_cost=args.fall_cost,
        w_feet_air=args.w_feet_air, air_time_target=args.air_time_target,
        w_single_support=args.w_single_support, w_lateral=args.w_lateral,
        w_yaw=args.w_yaw, w_pitch_rate=args.w_pitch_rate,
        terrain_amplitude=args.terrain_amplitude,
        terrain_smoothness=args.terrain_smoothness,
        terrain_amplitude_min=args.terrain_amplitude_min,
        domain_rand=args.domain_rand, action_latency=args.action_latency,
        latency_ms=args.latency_ms, latency_ms_max=args.latency_ms_max,
        latency_jitter_ms=args.latency_jitter_ms,
        push_force=args.push_force, push_prob=args.push_prob,
    )
    print("env config:", env_kwargs)
    # Persist the env config so eval_policy/compare_runs automatically evaluate
    # each run on the model/actuator it was trained with (old runs have no
    # config file and fall back to the original defaults).
    with open(os.path.join(run_dir, "env_config.json"), "w") as f:
        json.dump(env_kwargs, f, indent=2)

    # BimoWalkerEnv is importable, so SubprocVecEnv (spawn) can pickle it by
    # reference; env_kwargs (a plain dict) ship the config to each worker.
    if args.terrain_amplitude > 0 and args.terrain_mix < 1.0:
        # Mixed vec-env: the first k workers get terrain, the rest keep the
        # original flat plane. Hfield and plane contacts differ subtly, so a
        # policy that must work on both needs to train on both.
        n_terrain = round(args.terrain_mix * args.n_envs)
        flat_kwargs = dict(env_kwargs, terrain_amplitude=0.0)

        def _fn(rank, kw):
            def _init():
                env = BimoWalkerEnv(**kw)
                env.reset(seed=args.seed + rank)
                return Monitor(env)
            return _init

        venv = SubprocVecEnv([_fn(i, env_kwargs if i < n_terrain else flat_kwargs)
                              for i in range(args.n_envs)])
        print(f"mixed terrain vec-env: {n_terrain} terrain + "
              f"{args.n_envs - n_terrain} flat-plane workers")
    else:
        venv = make_vec_env(BimoWalkerEnv, n_envs=args.n_envs, seed=args.seed,
                            env_kwargs=env_kwargs, vec_env_cls=SubprocVecEnv)
    # Normalizing observations + returns is the single biggest lever for PPO
    # stability on locomotion tasks; clip keeps outliers from blowing up updates.
    if args.init_from:
        # Curriculum warm start: carry over the pretrained obs-normalizer stats
        # (keep adapting) and policy weights, then fine-tune on the new env.
        src = os.path.join(RUNS, args.init_from)
        venv = VecNormalize.load(os.path.join(src, "vecnormalize.pkl"), venv)
        venv.training = True
        venv.norm_reward = True
        model = PPO.load(os.path.join(src, "model"), env=venv, device="cpu",
                         tensorboard_log=os.path.join(run_dir, "tensorboard"))
        if args.lr is not None:   # gentle fine-tune: don't clobber the pretrained gait
            model.learning_rate = args.lr
            model.lr_schedule = lambda _: args.lr
        print(f"warm-started from run '{args.init_from}' (lr={model.learning_rate})")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True,
                            clip_obs=10.0, gamma=0.98)
        model = PPO(
            "MlpPolicy",
            venv,
            seed=args.seed,
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            gamma=0.98,
            gae_lambda=0.95,
            learning_rate=3e-4,
            ent_coef=0.0,
            clip_range=0.2,
            policy_kwargs=dict(net_arch=[128, 128]),
            tensorboard_log=os.path.join(run_dir, "tensorboard"),
            verbose=1,
        )

    ckpt = CheckpointCallback(
        save_freq=max(20_000 // args.n_envs, 1),
        save_path=os.path.join(run_dir, "checkpoints"),
        name_prefix="ppo",
    )

    print(f"Training PPO: {args.steps:,} steps across {args.n_envs} envs -> {run_dir}")
    model.learn(total_timesteps=args.steps, callback=ckpt, progress_bar=True)

    model.save(os.path.join(run_dir, "model"))
    venv.save(os.path.join(run_dir, "vecnormalize.pkl"))
    venv.close()
    print(f"\nSaved policy -> {run_dir}/model.zip"
          f"\nEvaluate  : python eval_policy.py --run-name {args.run_name}"
          f"\nCurves    : tensorboard --logdir {os.path.dirname(run_dir)}")


if __name__ == "__main__":
    main()
