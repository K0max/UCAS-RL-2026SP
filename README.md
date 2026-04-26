# UCAS-RL-2026SP

WHEELTEC B585 二阶平衡车的强化学习控制器。在 Python 仿真环境中训练 RL 策略，通过 WiFi 部署到实车替代 LQR 控制器。

## 快速开始

```bash
git clone https://github.com/<your-username>/UCAS-RL-2026SP.git
cd UCAS-RL-2026SP

# 安装依赖
uv sync

# 验证仿真环境 (LQR 能稳住 = 环境正常)
uv run src/validate_env.py

# 训练 RL 模型 (默认 SAC, ~50万步)
uv run src/train/train.py

# 查看训练曲线
uv run tensorboard --logdir data/logs
```

## 训练

```bash
# SAC (推荐, 样本效率高)
uv run src/train/train.py

# PPO
uv run src/train/train.py --algo PPO

# 自定义参数
uv run src/train/train.py --algo TD3 --timesteps 1000000 --lr 1e-4

# 加观测噪声 (提高 sim-to-real 鲁棒性)
uv run src/train/train.py --noise 0.02
```

模型保存在 `data/models/`，日志在 `data/logs/`。

## 模仿学习预训练 (可选)

```bash
# 用 LQR 采集专家数据
uv run src/imitation/collect_expert.py

# 行为克隆预训练
uv run src/imitation/pretrain_bc.py
```

## 实车部署

前置条件：小车已换 WiFi 模块、烧录修改版固件（见 `firmware/README.md`）。

```bash
# 1. 笔记本连接小车 WiFi (WHEELTEC_xxxxx)

# 2. 测试连通性
uv run src/deploy/wifi_test.py

# 3. LQR 基线 (验证通信链路正常)
uv run src/deploy/wifi_lqr.py

# 4. 部署 RL 模型
uv run src/deploy/wifi_rl.py --model data/models/best_model.zip
```

## 目录结构

```
src/
├── env/balance_car_env.py     # Gym 仿真环境
├── train/train.py         # RL 训练入口
├── deploy/                    # 实车部署 (WiFi 通信)
├── imitation/                 # 模仿学习 (可选加分)
└── utils/lqr.py               # LQR 控制器
data/
├── expert/                    # 专家数据
├── models/                    # 训练好的模型
└── logs/                      # TensorBoard 日志
```

详细进度和待办事项见 `progress.md`。
