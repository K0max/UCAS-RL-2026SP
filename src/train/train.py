"""
RL 训练入口脚本

用法:
  uv run src/train/train.py               # 默认 SAC
  uv run src/train/train.py --algo PPO    # 切换算法
  uv run src/train/train.py --timesteps 1000000

训练完成后模型保存在 data/models/, 日志在 data/logs/ (可用 tensorboard 查看).
"""

import argparse
import sys
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import SAC, PPO, TD3
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.env.balance_car_env import BalanceCarEnv
from src.train.config import TrainConfig


ALGO_MAP = {"SAC": SAC, "PPO": PPO, "TD3": TD3}


def make_env(cfg: TrainConfig):
    env = BalanceCarEnv(noise_std=cfg.noise_std)
    env = Monitor(env)
    return env


def train(cfg: TrainConfig):
    Path(cfg.log_dir).mkdir(parents=True, exist_ok=True)
    Path(cfg.model_dir).mkdir(parents=True, exist_ok=True)

    env = make_env(cfg)
    eval_env = make_env(cfg)

    algo_cls = ALGO_MAP[cfg.algo]

    algo_kwargs = dict(
        policy="MlpPolicy",
        env=env,
        learning_rate=cfg.learning_rate,
        gamma=cfg.gamma,
        verbose=0,
        tensorboard_log=cfg.log_dir,
        device="auto",
        policy_kwargs=dict(net_arch=cfg.net_arch),
    )

    if cfg.algo in ("SAC", "TD3"):
        algo_kwargs.update(
            batch_size=cfg.batch_size,
            buffer_size=cfg.buffer_size,
            tau=cfg.tau,
        )
    elif cfg.algo == "PPO":
        algo_kwargs.update(batch_size=cfg.batch_size)

    model = algo_cls(**algo_kwargs)

    checkpoint_cb = CheckpointCallback(
        save_freq=cfg.save_freq,
        save_path=cfg.model_dir,
        name_prefix=f"balance_{cfg.algo.lower()}",
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=cfg.model_dir,
        log_path=cfg.log_dir,
        eval_freq=cfg.save_freq,
        n_eval_episodes=10,
        deterministic=True,
    )

    print(f"=== Training {cfg.algo} for {cfg.total_timesteps} steps ===")
    print(f"    TensorBoard logs: {cfg.log_dir}")
    model.learn(
        total_timesteps=cfg.total_timesteps,
        callback=[checkpoint_cb, eval_cb],
        progress_bar=True,
    )

    final_path = Path(cfg.model_dir) / f"balance_{cfg.algo.lower()}_final"
    model.save(str(final_path))
    print(f"Model saved to {final_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo", default="SAC", choices=["SAC", "PPO", "TD3"])
    parser.add_argument("--timesteps", type=int, default=5_000)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--noise", type=float, default=0.01)
    args = parser.parse_args()

    cfg = TrainConfig(
        algo=args.algo,
        total_timesteps=args.timesteps,
        learning_rate=args.lr,
        noise_std=args.noise,
    )
    train(cfg)


if __name__ == "__main__":
    main()
