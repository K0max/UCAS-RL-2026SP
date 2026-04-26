"""
训练超参数配置
"""

from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


@dataclass
class TrainConfig:
    # 算法
    algo: str = "SAC"                  # "SAC" | "PPO" | "TD3"

    # 环境
    noise_std: float = 0.01            # 观测噪声 (sim-to-real)
    max_episode_steps: int = 1000      # 10s per episode

    # 训练
    total_timesteps: int = 500_000     # 总训练步数
    learning_rate: float = 3e-4
    batch_size: int = 256
    buffer_size: int = 100_000         # SAC/TD3 replay buffer
    gamma: float = 0.99
    tau: float = 0.005

    # 网络
    net_arch: list = field(default_factory=lambda: [256, 256])

    # 日志 & 保存
    log_dir: str = str(REPO_ROOT / "data" / "logs")
    model_dir: str = str(REPO_ROOT / "data" / "models")
    save_freq: int = 10_000            # 每 N 步保存一次 checkpoint

    # 模仿学习 (可选)
    use_pretrain: bool = False
    expert_data_path: str = str(REPO_ROOT / "data" / "expert" / "expert_data.npz")
    pretrain_epochs: int = 50
