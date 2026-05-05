"""
行为克隆 (Behavior Cloning) 预训练

用专家数据预训练一个策略网络, 然后可以用 RL 继续微调.

数据来源:
  - 仿真 LQR: uv run src/imitation/collect_expert.py → data/expert/expert_data.npz
  - 实车采集: uv run src/deploy/collect_real.py    → data/expert/real_car_data.npz

用法:
  uv run src/imitation/pretrain_bc.py                                    # 默认仿真数据
  uv run src/imitation/pretrain_bc.py --data data/expert/real_car_data.npz  # 实车数据
  uv run src/imitation/pretrain_bc.py --epochs 100 --hidden 64           # 调参
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from tqdm import trange

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


class PolicyNet(nn.Module):
    def __init__(self, obs_dim=8, act_dim=2, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, act_dim),
        )

    def forward(self, x):
        return self.net(x)


def pretrain(data_path: str, epochs: int = 50, hidden: int = 256):
    data = np.load(data_path)
    states = torch.from_numpy(data["states"]).float()
    actions = torch.from_numpy(data["actions"]).float()
    print(f"Loaded {len(states)} transitions from {data_path}")

    dataset = TensorDataset(states, actions)
    loader = DataLoader(dataset, batch_size=256, shuffle=True)

    model = PolicyNet(hidden=hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.MSELoss()

    pbar = trange(epochs, desc="BC pretrain")
    for epoch in pbar:
        total_loss = 0.0
        for s_batch, a_batch in loader:
            pred = model(s_batch)
            loss = criterion(pred, a_batch)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        avg_loss = total_loss / len(loader)
        pbar.set_postfix(loss=f"{avg_loss:.6f}")

    out_path = REPO_ROOT / "data" / "models" / "bc_pretrained.pt"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), str(out_path))
    print(f"\nPretrained model saved to {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=str(REPO_ROOT / "data" / "expert" / "expert_data.npz"),
                        help="Path to .npz with 'states' and 'actions' keys")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--hidden", type=int, default=256,
                        help="Hidden layer size (use 64 for on-chip deployment)")
    args = parser.parse_args()

    pretrain(args.data, epochs=args.epochs, hidden=args.hidden)


if __name__ == "__main__":
    main()
