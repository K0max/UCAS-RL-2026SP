"""
Offline RL 训练: 用实车数据预填充 replay buffer, 再在仿真中微调.

流程:
  1. 加载 real_car_data.npz (由 collect_real.py 采集)
  2. 为每个 transition 计算 reward (用仿真 reward 函数)
  3. 填入 SAC 的 replay buffer
  4. 在仿真环境中继续训练 (buffer 中已有真实数据, 可加速收敛 + 缩小 sim-to-real gap)

用法:
  uv run src/train/train_offline.py
  uv run src/train/train_offline.py --data data/expert/real_car_data.npz --timesteps 200000
  uv run src/train/train_offline.py --gradient-steps-only 5000   # 纯离线, 不与仿真交互
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.monitor import Monitor

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.env.balance_car_env import BalanceCarEnv

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def compute_rewards(states, actions, next_states):
    """用仿真 reward 函数为真实 transitions 计算 reward."""
    env = BalanceCarEnv()
    rewards = np.zeros(len(states), dtype=np.float32)
    dones = np.zeros(len(states), dtype=np.float32)

    for i in range(len(states)):
        ns = next_states[i]
        theta_1 = ns[4]
        theta_2 = ns[6]
        terminated = abs(theta_1) > env.THETA1_LIMIT or abs(theta_2) > env.THETA2_LIMIT
        rewards[i] = env._compute_reward(ns, actions[i], terminated)
        dones[i] = float(terminated)

    return rewards, dones


def fill_replay_buffer(model, states, actions, next_states, rewards, dones):
    """把真实 transitions 填入 SB3 replay buffer."""
    buf = model.replay_buffer
    n = len(states)
    print(f"Filling replay buffer with {n} real transitions ...")

    for i in range(n):
        buf.add(
            obs=states[i],
            next_obs=next_states[i],
            action=actions[i],
            reward=np.array([rewards[i]]),
            done=np.array([dones[i]]),
            infos=[{}],
        )

    print(f"  Buffer size: {buf.size()}/{buf.buffer_size}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=str(REPO_ROOT / "data" / "expert" / "real_car_data.npz"))
    parser.add_argument("--timesteps", type=int, default=200_000,
                        help="Online fine-tuning timesteps (0 = skip online phase)")
    parser.add_argument("--gradient-steps-only", type=int, default=0,
                        help="Pure offline: only do N gradient steps on buffer, no env interaction")
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--net-arch", type=int, nargs="+", default=[256, 256])
    args = parser.parse_args()

    print(f"Loading real data from {args.data} ...")
    data = np.load(args.data)
    states = data["states"]
    actions = data["actions"]
    next_states = data["next_states"]
    print(f"  {len(states)} transitions loaded")

    print("Computing rewards ...")
    rewards, dones = compute_rewards(states, actions, next_states)
    print(f"  Reward range: [{rewards.min():.2f}, {rewards.max():.2f}]")
    print(f"  Terminated: {dones.sum():.0f}/{len(dones)}")

    env = Monitor(BalanceCarEnv(noise_std=0.01))

    # 需要 normalize actions 到 [-1, 1] (SB3 SAC 内部用 normalized actions)
    u_max = env.unwrapped.U_MAX
    actions_normalized = np.clip(actions / u_max, -1.0, 1.0)

    model = SAC(
        "MlpPolicy",
        env,
        learning_rate=args.lr,
        buffer_size=max(100_000, len(states) * 2),
        batch_size=256,
        gamma=0.99,
        tau=0.005,
        verbose=0,
        device="auto",
        policy_kwargs=dict(net_arch=args.net_arch),
    )

    fill_replay_buffer(model, states, actions_normalized, next_states, rewards, dones)

    model_dir = Path(REPO_ROOT / "data" / "models")
    model_dir.mkdir(parents=True, exist_ok=True)
    log_dir = REPO_ROOT / "data" / "logs"

    if args.gradient_steps_only > 0:
        print(f"\n=== Pure offline training: {args.gradient_steps_only} gradient steps ===")
        model.logger.configure(str(log_dir), ["stdout", "tensorboard"])
        model.train(gradient_steps=args.gradient_steps_only, batch_size=256)
        out_path = model_dir / "balance_sac_offline.zip"
        model.save(str(out_path))
        print(f"Model saved to {out_path}")
    else:
        print(f"\n=== Online fine-tuning: {args.timesteps} steps (buffer pre-filled) ===")
        model.learn(
            total_timesteps=args.timesteps,
            progress_bar=True,
            tb_log_name="SAC_offline",
        )
        out_path = model_dir / "balance_sac_offline.zip"
        model.save(str(out_path))
        print(f"Model saved to {out_path}")


if __name__ == "__main__":
    main()
