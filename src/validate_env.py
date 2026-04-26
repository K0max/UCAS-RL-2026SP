"""
快速验证: 在仿真环境中跑 LQR 控制器, 检查环境是否正常工作.

用法:
  uv run src/validate_env.py

如果 matched LQR 能稳住大部分 episode, 说明环境动力学和 LQR 求解正确.
"""

import sys
from pathlib import Path

import numpy as np
from tqdm import trange

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from src.env.balance_car_env import BalanceCarEnv
from src.utils.lqr import matched_lqr_control


def validate(n_episodes=20, max_steps=1000):
    env = BalanceCarEnv(noise_std=0.0)
    K = env.matched_lqr_gain

    print("Env A-BK eigenvalues (all should be negative):")
    CL = env.A - env.B @ K
    eigs = np.linalg.eigvals(CL)
    print(f"  {sorted(eigs.real)}\n")

    results = []
    for ep in trange(n_episodes, desc="Validating LQR"):
        obs, _ = env.reset()
        total_reward = 0.0
        for step in range(max_steps):
            u_L, u_R = matched_lqr_control(obs, K)
            action = np.array([u_L, u_R], dtype=np.float32)
            action = np.clip(action, -env.U_MAX, env.U_MAX)
            obs, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            if terminated:
                break

        survived = step + 1
        success = not terminated
        results.append((success, survived, total_reward))

    n_success = sum(r[0] for r in results)
    avg_reward = np.mean([r[2] for r in results])

    print(f"\n=== Results: {n_success}/{n_episodes} survived full episode ===")
    print(f"    Average reward: {avg_reward:.1f}")

    if n_success < n_episodes * 0.5:
        print("\n[WARNING] LQR survival rate < 50%. Check env dynamics parameters.")
    else:
        print("\n[OK] Environment dynamics are reasonable. Ready for RL training.")

    return n_success, n_episodes


if __name__ == "__main__":
    validate()
