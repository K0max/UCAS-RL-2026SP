"""
用 LQR 控制器在仿真中收集专家数据, 供模仿学习使用.

输出: data/expert/expert_data.npz
  - states:  (N, 8)  状态 (env order)
  - actions: (N, 2)  动作 (u_L, u_R)
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.env.balance_car_env import BalanceCarEnv
from src.utils.lqr import matched_lqr_control

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def collect(n_episodes=200, max_steps=1000):
    env = BalanceCarEnv(noise_std=0.0)
    K = env.matched_lqr_gain
    all_states, all_actions = [], []

    success = 0
    for ep in range(n_episodes):
        obs, _ = env.reset()
        for step in range(max_steps):
            u_L, u_R = matched_lqr_control(obs, K)
            action = np.array([u_L, u_R], dtype=np.float32)
            action = np.clip(action, -env.U_MAX, env.U_MAX)

            all_states.append(obs.copy())
            all_actions.append(action.copy())

            obs, reward, terminated, truncated, _ = env.step(action)
            if terminated:
                break
        else:
            success += 1

    states = np.array(all_states, dtype=np.float32)
    actions = np.array(all_actions, dtype=np.float32)

    out_path = REPO_ROOT / "data" / "expert" / "expert_data.npz"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_path), states=states, actions=actions)

    print(f"Collected {len(states)} transitions from {n_episodes} episodes")
    print(f"  Success rate (survived full episode): {success}/{n_episodes}")
    print(f"  Saved to {out_path}")


if __name__ == "__main__":
    collect()
