"""
行为克隆 (Behavior Cloning) 预训练

用 LQR 专家数据预训练一个策略网络, 然后可以用 RL 继续微调.
这一步是可选的加分项, 能加快 RL 收敛.

用法:
  uv run src/imitation/pretrain_bc.py
"""

import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.train.config import TrainConfig

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


def pretrain(cfg: TrainConfig | None = None):
    if cfg is None:
        cfg = TrainConfig()

    data = np.load(cfg.expert_data_path)
    states = torch.as_tensor(data["states"], dtype=torch.float32)
    actions = torch.as_tensor(data["actions"], dtype=torch.float32)

    dataset = TensorDataset(states, actions)
    loader = DataLoader(dataset, batch_size=256, shuffle=True)

    model = PolicyNet()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.MSELoss()

    for epoch in range(cfg.pretrain_epochs):
        total_loss = 0
        for s_batch, a_batch in loader:
            pred = model(s_batch)
            loss = criterion(pred, a_batch)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        avg_loss = total_loss / len(loader)
        if (epoch + 1) % 10 == 0:
            print(f"Epoch {epoch+1}/{cfg.pretrain_epochs}, Loss: {avg_loss:.6f}")

    out_path = REPO_ROOT / "data" / "models" / "bc_pretrained.pt"
    torch.save(model.state_dict(), str(out_path))
    print(f"Pretrained model saved to {out_path}")


if __name__ == "__main__":
    pretrain()
