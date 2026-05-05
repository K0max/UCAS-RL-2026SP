"""
System Identification: 从实车数据拟合线性化 A, B 矩阵.

原理:
  线性化动力学 x_dot = A @ x + B @ u
  离散化近似: (x_{k+1} - x_k) / dt ≈ A @ x_k + B @ u_k
  整理为线性回归: Y = Theta @ X, 用最小二乘求解 Theta = [A | B]

输出:
  - 打印拟合的 A, B 与仿真 A, B 的差异
  - 保存到 data/models/real_AB.npz (可供 BalanceCarEnv 加载)

用法:
  uv run src/train/system_id.py
  uv run src/train/system_id.py --data data/expert/real_car_data.npz
  uv run src/train/system_id.py --dt 0.01
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.env.balance_car_env import BalanceCarEnv, build_AB, _ENV_TO_K

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def env_to_k_order(states_env):
    """批量转换: env order -> K-matrix order."""
    return states_env[:, _ENV_TO_K]


def fit_AB(states, actions, next_states, dt=0.01):
    """
    最小二乘拟合 A (8x8) 和 B (8x2).

    Y = (next_states - states) / dt   shape (N, 8)
    X = [states | actions]             shape (N, 10)
    Theta = [A | B]                    shape (8, 10)

    Solve: Y.T = Theta @ X.T  ->  Theta = Y.T @ X @ pinv(X.T @ X)
    """
    states_k = env_to_k_order(states)
    next_states_k = env_to_k_order(next_states)

    Y = (next_states_k - states_k) / dt  # (N, 8)
    X = np.hstack([states_k, actions])    # (N, 10)

    Theta, residuals, rank, sv = np.linalg.lstsq(X, Y, rcond=None)
    # lstsq solves X @ Theta = Y, so Theta is (10, 8)
    # We want A (8x8) and B (8x2): Theta.T = [A | B] (8x10)
    Theta_T = Theta.T  # (8, 10)
    A_fit = Theta_T[:, :8]
    B_fit = Theta_T[:, 8:]

    return A_fit, B_fit, residuals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=str(REPO_ROOT / "data" / "expert" / "real_car_data.npz"))
    parser.add_argument("--dt", type=float, default=0.01,
                        help="Time step between frames (default 10ms = firmware control period)")
    args = parser.parse_args()

    print(f"Loading data from {args.data} ...")
    data = np.load(args.data)
    states = data["states"]
    actions = data["actions"]
    next_states = data["next_states"]
    print(f"  {len(states)} transitions, dt={args.dt}s")

    print("\nFitting A, B matrices ...")
    A_fit, B_fit, residuals = fit_AB(states, actions, next_states, dt=args.dt)

    A_sim, B_sim = build_AB()

    print("\n=== Fitted A matrix (8x8) ===")
    np.set_printoptions(precision=4, suppress=True, linewidth=120)
    print(A_fit)

    print("\n=== Simulation A matrix (8x8) ===")
    print(A_sim)

    print("\n=== A difference (fitted - sim) ===")
    A_diff = A_fit - A_sim
    print(A_diff)
    print(f"  Frobenius norm: {np.linalg.norm(A_diff):.4f}")

    print("\n=== Fitted B matrix (8x2) ===")
    print(B_fit)

    print("\n=== Simulation B matrix (8x2) ===")
    print(B_sim)

    print("\n=== B difference (fitted - sim) ===")
    B_diff = B_fit - B_sim
    print(B_diff)
    print(f"  Frobenius norm: {np.linalg.norm(B_diff):.4f}")

    if residuals is not None and len(residuals) > 0:
        print(f"\n  Fit residuals (per dimension): {residuals}")

    out_path = REPO_ROOT / "data" / "models" / "real_AB.npz"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_path), A=A_fit, B=B_fit, dt=args.dt, n_samples=len(states))
    print(f"\nSaved to {out_path}")
    print("  Use in env: env = BalanceCarEnv(); env.A, env.B = A_fit, B_fit")


if __name__ == "__main__":
    main()
