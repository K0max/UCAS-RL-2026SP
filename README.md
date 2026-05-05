# UCAS-RL-2026SP

WHEELTEC B585 二阶平衡车的强化学习控制器. 在 Python 仿真环境中训练 RL 策略, 通过 WiFi 部署到实车替代 LQR 控制器.

## 架构

```
[STM32 固件]              [PC Python]
  传感器 200Hz   ──WiFi──>   策略推理
  电机驱动      <──WiFi──   动作输出
```

固件 (`firmware/MiniBalance.hex`, 已编译, 直接烧录) 上跑 `Control_mode = 1`, 只负责传感器读取和电机驱动. 所有控制逻辑 (LQR / RL) 在 PC 上跑.

## 离线 Pipeline 提醒

⚠️ **连上小车 WiFi (`WHEELTEC_*` / `Minibalance_*`) 后笔记本无法上网**. 任何需要网络的步骤必须先做完:

1. `git clone` 仓库
2. `uv sync` (下载 ~2GB torch + sb3)
3. 训练模型 (不需要网, 但建议提前训好)
4. (可选) `tensorboard --logdir data/logs --port 6006` 让浏览器缓存一次静态资源

完成后再切到小车 WiFi 跑 deploy 脚本.

## 快速开始

```bash
git clone https://github.com/k0max/UCAS-RL-2026SP.git
cd UCAS-RL-2026SP

# 1. 安装依赖
uv sync

# 2. 验证仿真环境 (matched LQR 能稳住 = 环境自洽)
uv run src/validate_env.py

# 3. 训练 (默认 SAC 5000 步只是 smoke test, 实战需要 200k~500k)
uv run src/train/train.py --algo SAC --timesteps 300000

# 4. 看曲线
uv run tensorboard --logdir data/logs
```

## 训练

```bash
# 三种算法
uv run src/train/train.py --algo SAC      # 推荐, 样本效率高
uv run src/train/train.py --algo PPO
uv run src/train/train.py --algo TD3

# 自定义
uv run src/train/train.py --algo SAC --timesteps 500000 --lr 1e-4

# 加观测噪声 (提高 sim-to-real 鲁棒性)
uv run src/train/train.py --noise 0.05
```

模型保存到 `data/models/balance_<algo>_final.zip`, 周期 checkpoint 自动滚动 (最多保留 2 个), TensorBoard 日志在 `data/logs/`.

### 模仿学习预训练 (可选加分)

```bash
uv run src/imitation/collect_expert.py    # 用 matched LQR 在仿真里采集专家数据
uv run src/imitation/pretrain_bc.py       # 行为克隆预训练
```

## 实车部署

前置条件: 小车已换 DT06 WiFi 模块, 烧录 `firmware/MiniBalance.hex` (详见 `firmware/README.md`).

```bash
# 1. 笔记本连接小车 WiFi (默认密码 12345678)
#    系统提示"无法访问 Internet" 忽略, 保持连接
#    确认 IP 网段:
ip addr | grep 192.168.4    # Linux
ifconfig | grep 192.168.4   # macOS

# 2. 连通性测试
uv run src/deploy/wifi_test.py

# 3. LQR 基线 (用固件原版 K 矩阵, 应该能稳)
uv run src/deploy/wifi_lqr.py

# 4. RL 部署 (默认加载 data/models/balance_sac_final.zip)
uv run src/deploy/wifi_rl.py
uv run src/deploy/wifi_rl.py --algo PPO
uv run src/deploy/wifi_rl.py --device cuda            # 强制 CUDA (会自动 fallback CPU)
uv run src/deploy/wifi_rl.py --action-scale 30        # 调大动作幅度
uv run src/deploy/wifi_rl.py --model path/to/x.zip
```

`wifi_rl.py` 默认 `device=auto`, 有 CUDA 用 CUDA, 否则 CPU. **注意**: 256x256 小网络 + bs=1 在 CPU 上通常更快 (CUDA 的 H2D/D2H 拷贝开销比 forward 大), 看 `max_pred` 数字决定要不要切 CPU.

测完恢复联网: 断开小车 WiFi 重连日常 WiFi 即可.

## 故障排查

### 1. 看控制循环健康度

跑 `wifi_lqr.py` / `wifi_rl.py` 时每秒打印:

```
[LQR] 95.3 Hz | u=(+1234.5,+1234.5) | t1=+0.012 t2=-0.008
[RL]  92.1Hz | u=(+3210,+3105) | t1=+0.012 t2=-0.008 | max_pred=2.1ms drops=0
```

- **Hz**: 80~150 健康, <30 太低 (WiFi 信号差或网卡省电)
- **u**: LQR 量级几百~几千; RL 量级 ±200 (训练时 `U_MAX=200`), 经 `--action-scale` 放大后应到几千
- **t1/t2**: 状态量; 平放静止应该接近 0 抖动, 越界 (>0.5) → 小车没放正或 IMU 漂移
- **max_pred**: RL 单次推理耗时, 应 <5ms; 大于 20ms 考虑切 CPU
- **drops**: 500ms 内没收到数据的次数, 应一直为 0

### 2. LQR 不稳 — 物理装配问题

`wifi_lqr.py` 用的是固件出厂同款 K 矩阵, 不稳大概率不是算法的锅:

- 小车没放在平地上 → 平放 5 秒重启 (按复位键), 让 IMU 重新校零
- 摆杆松动 / 没装正 → 拧紧
- 电池电量低 → 充满电 (低压时电机扭矩不够)
- 地面打滑 → 换硬地板木地板

### 3. RL 轮子不转 / 间歇微动 — 动作量级太小

RL 训练时 `U_MAX=200`, 但固件实际期望 LQR 量级 (几千). 解决: `--action-scale 20`~`30`.

### 4. RL 数据流间歇性中断

最常见原因: **5000 步的策略本质是噪声**, 输出乱抽搐 → 小车晃动剧烈 → 角度超限触发固件保护 → 停止上报.

解决: 训一个像样的模型 (≥200k 步).

### 5. LQR 能稳 + RL 不稳 — sim-to-real gap

预期之内, 是项目的核心难点. 缩小 gap 的方向:

- **加观测噪声训练**: `uv run src/train/train.py --noise 0.05`
- **Domain randomization**: 修改 `balance_car_env.py:build_AB()`, 在 `reset()` 里随机化物理参数 (l1, l2, m1, m2, I1, I2)
- **行为克隆 warm-start**: 先 `collect_expert.py` + `pretrain_bc.py`, 用 LQR 数据初始化 policy
- **延长训练**: 默认 5000 步只是 smoke test
- **System identification**: 在实车上跑 LQR 收集 (state, action, next_state), 拟合真实 A, B 矩阵, 重新调仿真参数

## 目录结构

```
UCAS-RL-2026SP/
├── pyproject.toml                 # uv 项目配置 & 依赖
├── README.md                      # ← 你在这里
├── src/
│   ├── env/balance_car_env.py     # Gym 仿真环境 (线性化 x_dot = Ax + Bu, scipy 求解 matched LQR)
│   ├── train/
│   │   ├── config.py              # TrainConfig 超参
│   │   └── train.py               # SAC / PPO / TD3 训练入口
│   ├── deploy/
│   │   ├── wifi_proto.py          # WiFi 通信工具 (drain buffer, 取最新帧, 发动作)
│   │   ├── wifi_test.py           # 连通性测试
│   │   ├── wifi_lqr.py            # LQR 基线
│   │   └── wifi_rl.py             # RL 模型部署
│   ├── imitation/
│   │   ├── collect_expert.py      # 用 matched LQR 采集专家数据
│   │   └── pretrain_bc.py         # 行为克隆预训练
│   ├── utils/lqr.py               # LQR 控制器 (实车 K 矩阵 + 仿真 K 矩阵)
│   └── validate_env.py            # 用 matched LQR 验证仿真环境
├── data/
│   ├── expert/                    # 专家数据 (.npz)
│   ├── models/                    # 训练好的模型 (.zip)
│   └── logs/                      # TensorBoard 日志
└── firmware/
    ├── README.md                  # 零基础烧录指南
    ├── MiniBalance.hex            # 预编译固件
    ├── tools/                     # mcuisp.exe + CH9102 驱动
    └── docs/                      # 烧录截图 + WiFi 配置 PDF
```

## 关键设计说明

### 仿真环境的 LQR 不是固件那个

固件实车 K 矩阵 (K13=-5492, K14=18921 等) 是为实车特定动力学手工调出来的. 仿真环境的物理参数 (m, l, I) 是估计值, 跟实车不一致 — 直接把实车 K 用在仿真里全部翻车.

解决: 仿真环境用 `scipy.linalg.solve_continuous_are` 为自己求一个 matched LQR (`env.matched_lqr_gain`). 用它验证环境自洽 (`validate_env.py` 19~20/20 通过).

代价: 仿真和实车之间存在 sim-to-real gap, 这是部署到实车时的核心难点 (见故障排查 #5).

### 训练数据从哪来?

不需要预先准备. RL 训练是 agent 和仿真环境实时交互产生 `(state, action, reward, next_state)`, 自动存入 replay buffer. 只有可选的模仿学习需要先用 LQR 采集专家数据.

### 嵌入式代码不需要改

固件 `Control_mode = 1` 已经是"接收外部指令"模式. 我们只需要:

- 通信协议 (帧解析 / 打包) → 用 Python 重新实现在 `deploy/wifi_proto.py`
- LQR 增益 → 抄到 `utils/lqr.py:K_REAL`
- 硬件常量 (轮径 / 编码器 / 控制频率) → 抄到 `env/balance_car_env.py`
- 安全限制 (theta_1 > ±45° 停机) → `env.THETA1_LIMIT`

原始 C 代码留在 `lab/` 作为参考, 不需要复制进 repo.
